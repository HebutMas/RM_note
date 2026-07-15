# kfifo.c/h - 无锁环形缓冲区

## 文件位置

`mas_embedded_threadx/utils/kfifo/kfifo.c` / `kfifo.h`

## 作用

通用的单生产者单消费者（SPSC）无锁环形缓冲区。在本项目中被 CAN 驱动使用：中断往 `rx_fifo` 写数据，RX Task 从 `rx_fifo` 读数据；用户任务往 `tx_fifo` 写数据，TX Task 从 `tx_fifo` 读数据。

设计原理见 [[01_extracted/kfifo-design]]。

---

## 数据结构

```c
struct kfifo
{
    volatile unsigned int in;    // 写索引（生产者维护，持续递增）
    volatile unsigned int out;   // 读索引（消费者维护，持续递增）
    unsigned int mask;           // 容量 - 1，位与替代取模
    unsigned int esize;          // 每个元素的字节数
    char        *data;           // 用户提供的缓冲区
};
```

`in` 和 `out` 永远递增不回绕，物理槽位通过 `index & mask` 计算。`volatile` 确保编译器不会把跨线程的读取优化掉。

---

## kfifo_init — 初始化

```c
int kfifo_init(struct kfifo *fifo, void *buffer, unsigned int size, unsigned int esize)
{
    if (size < 2) return -1;

    if ((size & (size - 1)) != 0)
        size = rounddown_pow_of_two(size);

    fifo->in    = 0;
    fifo->out   = 0;
    fifo->esize = esize;
    fifo->data  = (char *)buffer;
    fifo->mask  = size - 1;
    return 0;
}
```

- 缓冲区由调用者提供，kfifo 不分配内存
- `size` 自动向下取整为 2 的幂（`size & (size - 1) != 0` 是判断非 2 的幂的经典写法）
- `mask = size - 1`：后续所有索引计算用 `& mask` 一步完成

CAN 驱动中的初始化：
```c
kfifo_init(&bus->rx_fifo, bus->rx_fifo_buf, BSP_CAN_RX_FIFO_SIZE, sizeof(BSP_CanMsg_t));
// 容量 8，每个元素是 BSP_CanMsg_t（含 hcan 指针 + id + 8字节数据 + len）
```

---

## kfifo_put — 写入一个元素（生产者调用）

```c
unsigned int kfifo_put(struct kfifo *fifo, const void *element)
{
    if (kfifo_is_full(fifo)) return 0;

    unsigned int off = fifo->in & fifo->mask;
    memcpy(fifo->data + off * fifo->esize, element, fifo->esize);

    __DMB();

    fifo->in++;

    return 1;
}
```

1. 判断是否满：`kfifo_len(fifo) > fifo->mask`，即 `in - out > 容量 - 1`
2. 计算物理偏移：`in & mask`，一步位与定位槽位
3. `memcpy` 写入数据
4. `__DMB()` 内存屏障：保证数据写完后才更新 `in`
5. `in++`：消费者看到 `in` 变化时数据已就绪

---

## kfifo_get — 读取一个元素（消费者调用）

```c
unsigned int kfifo_get(struct kfifo *fifo, void *element)
{
    if (kfifo_is_empty(fifo)) return 0;

    unsigned int off = fifo->out & fifo->mask;
    memcpy(element, fifo->data + off * fifo->esize, fifo->esize);

    __DMB();

    fifo->out++;

    return 1;
}
```

1. 判断是否空：`in == out`
2. 计算物理偏移：`out & mask`
3. `memcpy` 读出数据
4. `__DMB()` 内存屏障：保证数据读完后才更新 `out`
5. `out++`：生产者看到 `out` 变化时槽位已释放

---

## 批量操作 — kfifo_in / kfifo_out

批量写入和读取使用 `fifo_copy_in` / `fifo_copy_out` 辅助函数处理回绕：

```c
static void fifo_copy_in(struct kfifo *fifo, const void *src, unsigned int len, unsigned int off)
{
    unsigned int size  = fifo->mask + 1;
    unsigned int esize = fifo->esize;
    unsigned int l;

    off &= fifo->mask;
    if (esize != 1) { off *= esize; size *= esize; len *= esize; }
    l = (len <= (size - off)) ? len : (size - off);

    memcpy(fifo->data + off, src, l);           // 尾部段
    memcpy(fifo->data, (const char *)src + l, len - l);  // 回绕段
}
```

如果写入的数据跨越缓冲区末尾，分两次 memcpy：先写满尾部，再从开头继续写。读取同理。

本项目 CAN 驱动只用单元素 `kfifo_put` / `kfifo_get`（每次一条消息），没有用批量操作。

---

## 状态查询

```c
unsigned int kfifo_len(const struct kfifo *fifo)    { return fifo->in - fifo->out; }
unsigned int kfifo_avail(const struct kfifo *fifo)  { return (fifo->mask + 1) - (fifo->in - fifo->out); }
int kfifo_is_empty(const struct kfifo *fifo)        { return fifo->in == fifo->out; }
int kfifo_is_full(const struct kfifo *fifo)         { return kfifo_len(fifo) > fifo->mask; }
```

`in - out` 利用 unsigned 溢出回绕特性，始终返回正确的元素数量。在并发环境下这些函数只提供近似值（读 `in` 和 `out` 之间可能被对方修改），但对判断"能不能读/写"是安全的。

---

## CAN 驱动中的 TX kfifo 覆盖写

CAN 发送时，TX kfifo 满了不直接丢弃新消息，而是丢弃最老的：

```c
if (!kfifo_put(&bus->tx_fifo, &msg))
{
    BSP_CanMsg_t dummy;
    kfifo_get(&bus->tx_fifo, &dummy);  // 丢弃最老
    kfifo_put(&bus->tx_fifo, &msg);    // 写入最新
}
```

电机控制场景下，最新的控制指令比旧的更重要——如果旧的指令还没发出去，说明总线繁忙，旧指令已经过时了。

---

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-04 | 初始版本 |
