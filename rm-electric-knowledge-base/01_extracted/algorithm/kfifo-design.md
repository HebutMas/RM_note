# kfifo 无锁环形缓冲区

> SPSC（单生产者单消费者）无锁环形队列，用位运算取余替代取模，`__DMB()` 保证内存可见性。
> 源码：`utils/kfifo/kfifo.c` / `kfifo.h`

## SPSC 架构

kfifo 只支持**一个生产者 + 一个消费者**。项目中：

- `rx_fifo`：HAL 中断写（唯一生产者），CAN RX Task 读（唯一消费者）—— 严格 SPSC
- `tx_fifo`：CAN TX Task 读（唯一消费者），但**生产者有多个**（电机任务、板间通信、超级电容等都会调 `BSP_CAN_Send` 往同一条总线的 tx_fifo 写）

> **注意**：tx_fifo 的多生产者场景破坏了 SPSC 前提，代码中没有加锁保护。当前依赖 motor task 统一调度时序避免实际并发。详见 [[02_code_twin/board/bsp/CAN/bsp_can#发送端多生产者问题]]。

---

## 编译期分配 — 没有动态内存

kfifo **不调用 malloc**。缓冲区内存由调用方在编译期以静态数组的形式提供，`kfifo_init` 只是把指针填进去：

```c
// bsp_can.h — 缓冲区是结构体里的静态数组，编译期就定好了大小
typedef struct {
    struct kfifo   rx_fifo;                           /* kfifo 控制块 */
    BSP_CanMsg_t   rx_fifo_buf[BSP_CAN_RX_FIFO_SIZE]; /* 实际存储区，静态数组 */
    struct kfifo   tx_fifo;
    BSP_CanMsg_t   tx_fifo_buf[BSP_CAN_TX_FIFO_SIZE];
} Can_Bus_t;
```

```c
// bsp_can.c — 初始化时把数组地址传给 kfifo
kfifo_init(&bus->rx_fifo, bus->rx_fifo_buf, BSP_CAN_RX_FIFO_SIZE, sizeof(BSP_CanMsg_t));
kfifo_init(&bus->tx_fifo, bus->tx_fifo_buf, BSP_CAN_TX_FIFO_SIZE, sizeof(BSP_CanMsg_t));
```

`kfifo_init` 做了什么：

```c
int kfifo_init(struct kfifo *fifo, void *buffer, unsigned int size, unsigned int esize)
{
    if (size < 2) return -1;

    // 不是 2 的幂就向下取整
    if ((size & (size - 1)) != 0)
        size = rounddown_pow_of_two(size);

    fifo->in    = 0;           // 写索引归零
    fifo->out   = 0;           // 读索引归零
    fifo->esize = esize;       // 每个元素多少字节
    fifo->data  = (char *)buffer;  // 指向外部数组，不分配内存
    fifo->mask  = size - 1;    // 掩码，用于 & 运算代替 %
    return 0;
}
```

> `fifo->data` 只是一个指针，指向外部提供的数组。kfifo 结构体本身不持有内存，生命周期由调用方管理。在项目中，`Can_Bus_t` 是全局静态变量，所以 `rx_fifo_buf` / `tx_fifo_buf` 数组在整个程序运行期间都有效。

### 为什么必须是 2 的幂

`mask = size - 1`，后续所有取余操作用 `& mask` 代替 `% size`。位与比取模快得多，但前提是 size 必须是 2 的幂。`rounddown_pow_of_two()` 在初始化时保证这一点：传入 64 得到 64，传入 100 得到 64，传入 33 得到 32。

---

## in 和 out — 持续递增不回绕

这是 kfifo 最核心的设计：

```
in  和 out 都是 unsigned int，只增不减（reset 除外）
            ┌──────────────────────────────────┐
            │  不会回绕到 0，一直涨到 0xFFFFFFFF │
            │  然后自然溢出回 0（unsigned 回卷）│
            └──────────────────────────────────┘

实际写入位置 = in & mask    （等价于 in % size，但快得多）
实际读取位置 = out & mask
队列内元素数 = in - out      （无符号减法，自动处理回卷）
```

> 详见 [[01_extracted/algorithm/computer-basics#无符号整数回卷]]。`in - out` 不需要考虑谁大谁小，unsigned 减法在溢出时自然回卷给出正确结果。

---

## 判空和判满

```c
// 判空：in 追上了 out，没有未读数据
int kfifo_is_empty(const struct kfifo *fifo) {
    return fifo->in == fifo->out;
}

// 判满：已用元素数 > mask（即 >= size）
int kfifo_is_full(const struct kfifo *fifo) {
    return kfifo_len(fifo) > fifo->mask;
}
```

| 状态 | 条件 | 含义 |
|------|------|------|
| 空 | `in == out` | 没有未读数据 |
| 满 | `in - out > mask` | 已用元素数 >= 容量，放不下了 |
| 有数据 | `in != out` | 可以读 |
| 没满 | `in - out <= mask` | 可以写 |

> **为什么判满用 `> mask` 而不是 `== size`？** 因为 `mask = size - 1`，`in - out > mask` 等价于 `in - out >= size`。用 `>` 是因为 `kfifo_len()` 返回 `in - out`，而 `mask` 就是 `size - 1`，比较起来直接。

辅助查询函数：

```c
unsigned int kfifo_len(const struct kfifo *fifo)    { return fifo->in - fifo->out; }            // 已用元素数
unsigned int kfifo_avail(const struct kfifo *fifo)  { return (fifo->mask + 1) - (fifo->in - fifo->out); }  // 剩余空间
unsigned int kfifo_size(const struct kfifo *fifo)   { return fifo->mask + 1; }                  // 总容量
```

---

## put — 写一个元素

```c
unsigned int kfifo_put(struct kfifo *fifo, const void *element)
{
    if (kfifo_is_full(fifo)) return 0;           // ① 满了就拒绝，返回 0

    unsigned int off = fifo->in & fifo->mask;   // ② 算写入位置（位与取余）
    memcpy(fifo->data + off * fifo->esize, element, fifo->esize);  // ③ 拷贝数据

    __DMB();                                      // ④ 内存屏障
    fifo->in++;                                   // ⑤ 发布：in 加 1

    return 1;                                     // 成功返回 1
}
```

步骤拆解：

1. **判满**：`in - out > mask` 说明缓冲区满了，直接返回 0
2. **算位置**：`in & mask` 把持续递增的 `in` 映射到 `[0, size-1]` 范围
3. **写数据**：`memcpy` 把元素拷到 `data + off * esize` 处
4. **内存屏障**：`__DMB()` 保证步骤 3 的写入对消费者可见
5. **发布**：`in++` 让消费者能看到新数据

> **关键顺序**：先写数据，再 `__DMB()`，最后 `in++`。如果顺序反了，消费者可能看到 `in` 更新了但数据还没写完。

### 项目中的实际使用

```c
// bsp_can.c — CAN 中断回调里往 rx_fifo 写
if (kfifo_put(&bus->rx_fifo, &msg)) has_msg = true;

// bsp_can.c — tx_fifo 满时丢弃最老消息（覆盖写）
if (!kfifo_put(&bus->tx_fifo, &msg)) {
    kfifo_get(&bus->tx_fifo, &dummy);  // 丢掉最老的
    kfifo_put(&bus->tx_fifo, &msg);    // 再放新的
}
```

---

## get — 读一个元素

```c
unsigned int kfifo_get(struct kfifo *fifo, void *element)
{
    if (kfifo_is_empty(fifo)) return 0;          // ① 空了就拒绝，返回 0

    unsigned int off = fifo->out & fifo->mask;   // ② 算读取位置
    memcpy(element, fifo->data + off * fifo->esize, fifo->esize);  // ③ 拷贝数据

    __DMB();                                      // ④ 内存屏障
    fifo->out++;                                  // ⑤ 消费：out 加 1

    return 1;                                     // 成功返回 1
}
```

和 put 完全对称：

1. **判空**：`in == out` 说明没有数据
2. **算位置**：`out & mask` 映射到实际地址
3. **读数据**：`memcpy` 从缓冲区拷出来
4. **内存屏障**：保证读完再更新 `out`
5. **消费**：`out++` 释放这个槽位

### 项目中的实际使用

```c
// bsp_can_task.c — RX Task 循环取数据直到空
while (kfifo_get(&g_can_bus[i].rx_fifo, &msg)) {
    // 处理 msg...
}
```

---

## peek — 预读不移除

```c
unsigned int kfifo_peek(const struct kfifo *fifo, void *element)
{
    if (kfifo_is_empty(fifo)) return 0;

    unsigned int off = fifo->out & fifo->mask;
    memcpy(element, fifo->data + off * fifo->esize, fifo->esize);
    // 注意：不更新 out，不移除元素
    return 1;
}
```

读数据但不推进 `out`，下次 `get` 还能读到同一条。用于"先看看下一条是什么再决定要不要取"的场景。

---

## 批量操作 — kfifo_in / kfifo_out

```c
unsigned int kfifo_in(struct kfifo *fifo, const void *buf, unsigned int n)
{
    unsigned int l = kfifo_avail(fifo);    // 先看能放多少
    if (n > l) n = l;                      // 放不下就截断
    fifo_copy_in(fifo, buf, n, fifo->in);  // 拷贝（自动处理回绕）

    __DMB();
    fifo->in += n;                         // 一次推进 n
    return n;                              // 返回实际放入的数量
}
```

批量操作和单元素操作逻辑一样，区别是 `fifo_copy_in` / `fifo_copy_out` 会自动处理缓冲区尾部回绕：

```c
static void fifo_copy_in(struct kfifo *fifo, const void *src, unsigned int len, unsigned int off)
{
    unsigned int size = fifo->mask + 1;
    unsigned int l;

    off &= fifo->mask;                    // 映射到实际位置
    // ... esize 换算 ...

    l = (len <= (size - off)) ? len : (size - off);  // 第一段：从 off 到缓冲区尾部

    memcpy(fifo->data + off, src, l);               // 第一段拷贝
    memcpy(fifo->data, (const char *)src + l, len - l);  // 第二段：从头部继续（回绕部分）
}
```

如果写入位置离缓冲区尾部不够放下全部数据，就分两段拷贝：先写满尾部，再从头开始写剩余部分。

```
写入 n=3 个元素到 off=3 (mask=3, size=4):
                    off=3
                    ↓
    ┌─────┬─────┬─────┬─────┐
    │ [0] │ [1] │ [2] │ [3] │
    └─────┴─────┴─────┴─────┘
    └ 后半段 (size-off=1) ┘└ 回绕到开头 (n-l=2) ┘
              ↑  memcpy #1   ↑  memcpy #2
```

```
fifo_copy_in / fifo_copy_out 的逻辑：
┌──────────────────────────────────┐
│ 1. off = off & mask              │
│ 2. l = min(len, size - off)      │
│ 3. memcpy(data + off, src, l)    │  ← 尾部段
│ 4. memcpy(data, src + l, len-l)  │  ← 回绕段
└──────────────────────────────────┘
```

---

## reset — 清空

```c
// 完全清空（生产者和消费者都能调）
void kfifo_reset(struct kfifo *fifo) {
    fifo->in  = 0;
    fifo->out = 0;
}

// 只丢弃已读位置（仅消费者调）
void kfifo_reset_out(struct kfifo *fifo) {
    fifo->out = fifo->in;
}
```

`kfifo_reset` 直接把 in 和 out 都归零。`kfifo_reset_out` 把 out 跳到 in 的位置，等效于"标记所有数据已读"。

> **注意**：`kfifo_reset` 不是线程安全的——如果在生产者写的时候调 reset，会丢数据。只能在确定没有并发访问时调。

---

## 内存屏障

```c
memcpy(fifo->data + off * esize, element, esize);  // ① 写数据
__DMB();                                             // ② 屏障
fifo->in++;                                          // ③ 发布索引
```

`__DMB()` 保证 ① 的内存写入对其他核心/中断可见后，才执行 ③。没有它，消费者可能看到 `in` 更新了但数据还没写完。

`__DMB()` 不是锁，不阻止中断打断，只约束自己的指令顺序。正确性靠 SPSC 保证：消费者只读 `out` 到 `in-1` 范围，生产者还没 `in++` 时消费者看不到新数据。

`__DMB()` 来自 CMSIS Core（`cmsis_gcc.h`），对应 ARM `DMB` 指令。Cortex-M3/M4/M7 都支持。

---

## API 接口一览

```c
int          kfifo_init(struct kfifo *fifo, void *buffer, unsigned int size, unsigned int esize);
unsigned int kfifo_size(const struct kfifo *fifo);
unsigned int kfifo_len(const struct kfifo *fifo);
unsigned int kfifo_avail(const struct kfifo *fifo);
int          kfifo_is_empty(const struct kfifo *fifo);
int          kfifo_is_full(const struct kfifo *fifo);
void         kfifo_reset(struct kfifo *fifo);
void         kfifo_reset_out(struct kfifo *fifo);
// 单元素操作
unsigned int kfifo_put(struct kfifo *fifo, const void *element);
unsigned int kfifo_get(struct kfifo *fifo, void *element);
unsigned int kfifo_peek(const struct kfifo *fifo, void *element);
// 批量操作
unsigned int kfifo_in(struct kfifo *fifo, const void *buf, unsigned int n);
unsigned int kfifo_out(struct kfifo *fifo, void *buf, unsigned int n);
unsigned int kfifo_out_peek(const struct kfifo *fifo, void *buf, unsigned int n);
```

| 操作 | 函数 | 判据 | 返回值 |
|------|------|------|--------|
| 写一个 | `kfifo_put` | 满了返回 0 | 1=成功, 0=满 |
| 读一个 | `kfifo_get` | 空了返回 0 | 1=成功, 0=空 |
| 预读 | `kfifo_peek` | 空了返回 0 | 1=成功, 0=空（不推进 out） |
| 批量写 | `kfifo_in` | 按 avail 截断 | 实际写入数 |
| 批量读 | `kfifo_out` | 按 len 截断 | 实际读出数 |
| 判空 | `kfifo_is_empty` | `in == out` | 1=空, 0=非空 |
| 判满 | `kfifo_is_full` | `in - out > mask` | 1=满, 0=不满 |
| 已用 | `kfifo_len` | `in - out` | 元素数 |
| 剩余 | `kfifo_avail` | `size - len` | 元素数 |
| 清空 | `kfifo_reset` | in=out=0 | void |

---

## 使用示例

### 字节 FIFO（esize = 1）

```c
#include "kfifo.h"

#define BUF_SIZE 64  // 2 的幂，否则自动向下取整

void byte_fifo_demo(void)
{
    uint8_t      buf[BUF_SIZE];
    struct kfifo fifo;

    kfifo_init(&fifo, buf, BUF_SIZE, 1);  // esize=1 字节模式

    // 批量写入
    const char *msg = "Hello kfifo!";
    unsigned int n  = kfifo_in(&fifo, msg, 12);
    // n = 12, len = 12

    // 批量读出
    char read_buf[64];
    n = kfifo_out(&fifo, read_buf, 12);
    // read_buf = "Hello kfifo!", len = 0
}
```

### 结构体 FIFO（esize > 1）

```c
typedef struct
{
    int   id;
    float value;
} sensor_data_t;

void struct_fifo_demo(void)
{
    sensor_data_t buf[8];
    struct kfifo  fifo;

    kfifo_init(&fifo, buf, 8, sizeof(sensor_data_t));

    // 单元素写入
    sensor_data_t s1 = {.id = 1, .value = 3.14f};
    sensor_data_t s2 = {.id = 2, .value = 2.71f};
    kfifo_put(&fifo, &s1);  // len = 1
    kfifo_put(&fifo, &s2);  // len = 2

    // peek 不移除
    sensor_data_t peek;
    kfifo_peek(&fifo, &peek);
    // peek = {1, 3.14}, len = 2 (unchanged)

    // 批量读取
    sensor_data_t batch[8];
    unsigned int n = kfifo_out(&fifo, batch, 8);
    // n = 2, batch[0] = s1, batch[1] = s2
}
```

> 注意：`kfifo_in` / `kfifo_out` 不是原子操作，SPSC 模式下安全的前提是**生产者和消费者各只有一方**，且不使用 `kfifo_reset` / `kfifo_reset_out` 等可能破坏索引一致性的操作
