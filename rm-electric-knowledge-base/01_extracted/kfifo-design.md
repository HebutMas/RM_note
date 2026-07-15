# kfifo 无锁环形缓冲区设计原理

> 提炼自 Linux 内核 kfifo 设计思想，本项目实现位于 `utils/kfifo/`。

---

## 为什么需要 kfifo

在 RTOS 环境中，中断和任务之间需要传递数据。比如 CAN 接收中断产生数据，任务循环消费数据。如果用普通队列加锁，中断里不能阻塞（会导致死锁），用关中断又影响实时性。

kfifo 解决的是**单生产者单消费者（SPSC）**场景下的无锁数据传递：生产者只管写，消费者只管读，不需要任何锁。

---

## 核心设计

### 两个持续递增的索引

```
struct kfifo {
    volatile unsigned int in;    // 写索引：生产者只写这个
    volatile unsigned int out;   // 读索引：消费者只写这个
    unsigned int mask;           // 容量 - 1，用位与替代取模
    unsigned int esize;          // 每个元素的字节数
    char        *data;           // 用户提供的缓冲区
};
```

`in` 和 `out` 永远递增，不做取模回绕。物理槽位通过 `index & mask` 计算：

```
容量 = 8, mask = 7

in  = 10 → 写入位置 = 10 & 7 = 2
out = 7  → 读取位置 = 7 & 7  = 7

有效数据量 = in - out = 3
```

`in` 和 `out` 是 `unsigned int`，溢出后自动回绕（C 标准保证），`in - out` 始终正确。

### 为什么容量必须是 2 的幂

因为 `index & mask` 只在容量为 2 的幂时等价于 `index % capacity`。位与是一条 CPU 指令，取模是除法指令。`kfifo_init` 会自动将非 2 的幂向下取整。

### 为什么无锁是安全的

SPSC 模型下：
- **生产者**只写 `in`（写完后 `in++`），只读 `out`（判断是否满）
- **消费者**只写 `out`（读完后 `out++`），只读 `in`（判断是否空）

两者修改的是不同的变量，不存在写冲突。唯一的问题是内存可见性——生产者写完数据和 `in++` 之间，消费者可能看到 `in` 已更新但数据还没写完。

### `__DMB()` 内存屏障

```c
unsigned int kfifo_put(struct kfifo *fifo, const void *element)
{
    if (kfifo_is_full(fifo)) return 0;

    unsigned int off = fifo->in & fifo->mask;
    memcpy(fifo->data + off * fifo->esize, element, fifo->esize);

    __DMB();      // 数据屏障：保证上面的 memcpy 先完成

    fifo->in++;   // 然后才更新 in，消费者看到 in 变化时数据已就绪

    return 1;
}
```

`__DMB()`（Data Memory Barrier）是 ARM Cortex-M 的内存屏障指令，确保它之前的所有内存写操作完成后，才执行之后的内存写操作。

没有这个屏障，CPU 可能乱序执行：先更新 `in` 再写数据。消费者看到 `in` 变了就去读，但数据还没写完，读到垃圾。

`kfifo_get` 同理：先 `memcpy` 读出数据，`__DMB()`，再 `out++`。保证消费者读完后才更新 `out`，生产者看到 `out` 变化时槽位已释放。

---

## 回绕拷贝

批量操作（`kfifo_in` / `kfifo_out`）时，数据可能跨越缓冲区末尾。内部用两次 `memcpy` 处理：

```
写入 3 个元素到 off=3（容量 4）:

    ┌─────┬─────┬─────┬─────┐
    │ [0] │ [1] │ [2] │ [3] │
    └─────┴─────┴─────┴─────┘
                    ↑
                   off=3

memcpy #1: 写 1 个元素到 [3]（尾部段，size - off = 1）
memcpy #2: 写 2 个元素到 [0]、[1]（回绕段，len - l = 2）
```

---

## 关键约束

1. **必须 SPSC**——只有一个生产者写，只有一个消费者读。多生产者或多消费者必须外部加锁
2. **不能在并发环境下调用 `kfifo_reset`**——它同时修改 `in` 和 `out`，破坏无锁前提
3. **缓冲区由调用者提供**——kfifo 不分配内存，`data` 指针指向的缓冲区必须在整个使用期保持有效
4. **`is_empty` / `is_full` 在并发下只提供近似值**——因为读 `in` 和 `out` 之间可能被对方修改，但这对判断"能不能读/写"来说是安全的（最坏情况是少读/少写一次，下次循环补上）

---

## 本项目中的使用

CAN 驱动中，每条 CAN 总线（`CAN_Bus_Manager`）有两个 kfifo：

| kfifo | 生产者 | 消费者 | 元素类型 | 容量 |
|-------|--------|--------|----------|------|
| `rx_fifo` | HAL 接收中断（ISR） | CAN RX Task | `BSP_CanMsg_t` | 8 |
| `tx_fifo` | 用户任务调 `BSP_CAN_Send()` | CAN TX Task | `BSP_CanMsg_t` | 8 |

两者都是标准 SPSC：中断只写 `rx_fifo`，RX Task 只读 `rx_fifo`；用户任务只写 `tx_fifo`，TX Task 只读 `tx_fifo`。无需任何锁。
