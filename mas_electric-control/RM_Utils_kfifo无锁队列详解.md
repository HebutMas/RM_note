# kfifo 无锁队列详解

> 本文讲解电控代码 `utils` 库中的 kfifo（内核风格环形缓冲区）。它是 CAN、UART、USB 等外设在 **中断（ISR）** 与 **任务（Task）** 之间传递数据的核心数据结构，也是整个 BSP 层"ISR 只搬运、任务做处理"设计的基石。

## 目录

1. [为什么需要无锁队列](#1-为什么需要无锁队列)
2. [kfifo 的核心思想](#2-kfifo-的核心思想)
3. [数据结构与初始化](#3-数据结构与初始化)
4. [入队与出队的实现原理](#4-入队与出队的实现原理)
5. [无锁为什么成立（单生产者单消费者）](#5-无锁为什么成立单生产者单消费者)
6. [内存屏障与 Cortex-M 可见性](#6-内存屏障与-cortex-m-可见性)
7. [在本项目中的实际应用](#7-在本项目中的实际应用)
8. [满策略：丢弃最旧数据](#8-满策略丢弃最旧数据)
9. [常见陷阱与使用规范](#9-常见陷阱与使用规范)
10. [核心 API 速查](#10-核心-api-速查)

---

## 1. 为什么需要无锁队列

### 1.1 ISR 与任务的矛盾

嵌入式实时系统里，外设数据几乎都在**中断上下文**里到达：

```
FDCAN 收到一帧 → 触发 RX FIFO 中断 → HAL_FDCAN_RxFifo0Callback()
UART 空闲       → 触发 IDLE 中断     → HAL_UARTEx_RxEventCallback()
USB 收到批量包  → 触发端点中断       → cdc_acm_out_callback()
```

ISR 有两条铁律：

- **必须短**：中断期间会阻塞更低优先级的中断和所有任务，长中断直接毁掉实时性。
- **不能阻塞**：ISR 里不允许调用会挂起的 API（等锁、等信号量超时），否则整个系统卡死。

而数据处理（解析协议、跑控制算法）往往很慢。于是标准做法是**生产者-消费者解耦**：

```
ISR（生产者）：快速把原始数据塞进缓冲区，然后发个信号就返回
任务（消费者）：被信号唤醒后，慢慢从缓冲区取数据处理
```

这个"缓冲区"就是 kfifo。

### 1.2 为什么不用互斥锁

第一直觉是给共享缓冲区加把锁（mutex）。但在 ISR 场景下这是**行不通的**：

| 方案 | 问题 |
|------|------|
| 互斥量 Mutex | ISR 里根本不能等锁；且会引入优先级反转 |
| 关中断保护 | 简单粗暴，但会拉长中断关闭时间，破坏实时性 |
| 信号量 | 只能用来"通知"，不适合保护每次读写的临界区 |

无锁队列的价值在于：**在单生产者单消费者（SPSC）场景下，读写双方各自只碰自己的指针，天然互不冲突，完全不需要加锁。** ISR 入队和任务出队可以真正并发执行。

---

## 2. kfifo 的核心思想

kfifo 来源于 Linux 内核的 `kfifo`，核心是一个**环形缓冲区（ring buffer）**加两个游标：

```
        ┌───────────────────────────────────────┐
buffer  │ D │ E │   │   │   │   │ A │ B │ C │    │  (size = 容量)
        └───────────────────────────────────────┘
              ▲                   ▲
              │                   │
            in(写入位置)        out(读取位置)
```

- **`in`**：下一个要写入的位置，只有**生产者**改它。
- **`out`**：下一个要读取的位置，只有**消费者**改它。
- 数据量 = `in - out`（用无符号数环绕自动处理翻转，见 §4）。
- 空：`in == out`；满：`in - out == size`。

关键设计有两点：

**① in / out 只增不减，一直往上加**

它们是 `uint32_t`，写/读时**不取模**，只在**访问 buffer 下标时才 `& (size-1)` 取模**。这样"满"和"空"用同一个减法就能区分，不需要额外的标志位或浪费一个槽位。

**② 容量必须是 2 的幂**

因为要用 `index & (size - 1)` 代替昂贵的 `index % size` 取模运算。例如 `size = 256`，则 `& 0xFF` 就是取低 8 位，一条指令搞定。

---

## 3. 数据结构与初始化

一个典型的 kfifo 结构体：

```c
typedef struct {
    uint8_t  *buffer;   // 数据缓冲区指针
    uint32_t  size;     // 缓冲区容量，必须是 2 的幂
    uint32_t  in;       // 写游标（只增），生产者维护
    uint32_t  out;      // 读游标（只增），消费者维护
} Kfifo;
```

初始化时校验容量是 2 的幂，并清零游标：

```c
void kfifo_init(Kfifo *fifo, uint8_t *buffer, uint32_t size)
{
    // 断言 size 是 2 的幂：size & (size - 1) == 0
    fifo->buffer = buffer;
    fifo->size   = size;
    fifo->in     = 0;
    fifo->out    = 0;
}
```

几个派生量：

```c
// 已用长度：无符号减法，即使 in 溢出翻转也正确
static inline uint32_t kfifo_len(Kfifo *fifo)
{
    return fifo->in - fifo->out;
}

// 剩余空间
static inline uint32_t kfifo_avail(Kfifo *fifo)
{
    return fifo->size - (fifo->in - fifo->out);
}

static inline bool kfifo_is_empty(Kfifo *fifo) { return fifo->in == fifo->out; }
static inline bool kfifo_is_full (Kfifo *fifo) { return kfifo_len(fifo) == fifo->size; }
```

> 注意 `kfifo_len` 用的是无符号减法。当 `in` 累加到 `UINT32_MAX` 再溢出回 0 时，`in - out` 依然给出正确的差值（模 2³² 环绕），所以游标可以永远只增不减。

---

## 4. 入队与出队的实现原理

### 4.1 入队 kfifo_put

把 `len` 字节从 `data` 拷进缓冲区：

```c
uint32_t kfifo_put(Kfifo *fifo, const uint8_t *data, uint32_t len)
{
    uint32_t l;

    // 1. 实际能写入的量 = min(请求量, 剩余空间)
    len = min(len, fifo->size - (fifo->in - fifo->out));

    // 2. 计算从 in 到缓冲区末尾的连续空间，可能需要绕回开头
    l = min(len, fifo->size - (fifo->in & (fifo->size - 1)));
    memcpy(fifo->buffer + (fifo->in & (fifo->size - 1)), data, l);      // 前半段
    memcpy(fifo->buffer, data + l, len - l);                            // 绕回的后半段

    // 3. 最后一步才更新 in（关键！见 §5）
    fifo->in += len;
    return len;
}
```

拷贝分成两段是因为环形缓冲区可能**跨越末尾绕回开头**（wrap-around）：

```
写入位置 in&mask 靠近末尾时：
        ┌───────────────────────────┐
        │       │▓▓▓▓▓▓▓▓▓▓▓│░░░░░│   │
        └───────────────────────────┘
                ▲ out       ▲ in&mask
        新数据先填 [in&mask, size)（l 字节），剩下的绕回 [0, len-l)
```

### 4.2 出队 kfifo_get

```c
uint32_t kfifo_get(Kfifo *fifo, uint8_t *data, uint32_t len)
{
    uint32_t l;

    // 1. 实际能读出的量 = min(请求量, 已用长度)
    len = min(len, fifo->in - fifo->out);

    // 2. 同样分两段拷贝，处理绕回
    l = min(len, fifo->size - (fifo->out & (fifo->size - 1)));
    memcpy(data, fifo->buffer + (fifo->out & (fifo->size - 1)), l);
    memcpy(data + l, fifo->buffer, len - l);

    // 3. 最后更新 out
    fifo->out += len;
    return len;
}
```

结构完全对称：生产者只读 `out`、只写 `in`；消费者只读 `in`、只写 `out`。

---

## 5. 无锁为什么成立（单生产者单消费者）

这是 kfifo 最精妙、也最容易出 bug 的地方。**"无锁"有严格前提：只有一个生产者、一个消费者（SPSC）。**

### 5.1 职责严格分离

| 角色 | 读 | 写 |
|------|-----|-----|
| 生产者（ISR） | `out`（只读，判断剩余空间） | `in`、`buffer` 的空闲区 |
| 消费者（Task） | `in`（只读，判断可读长度） | `out`、读走 `buffer` 的已用区 |

两者写的是**不同的变量**（`in` vs `out`），也访问**不同的内存区域**（空闲区 vs 已用区），所以即使真正并发也不会踩到对方的数据。

### 5.2 游标更新的顺序至关重要

看 `kfifo_put` 的最后一步——**先把数据拷完，才更新 `in`**：

```c
memcpy(...);        // ① 先写数据
fifo->in += len;    // ② 后移动游标（发布数据）
```

`in` 的更新起到"**发布**"的作用。消费者是靠 `in` 判断有多少数据可读的：只要 `in` 还没变大，消费者就认为那块数据不存在，绝不会去读一块**正在被写的半成品**。反过来，`kfifo_get` 先读完数据、再更新 `out`，保证生产者不会把消费者**还没读完的槽位**当成空闲区覆盖。

**如果把顺序写反**（先动游标再拷数据），消费者就可能读到脏数据 —— 这是无锁队列最经典的错误。

### 5.3 为什么多生产者会崩

如果两个 ISR（或两个任务）同时 `kfifo_put`：

```
生产者A 读 in = 100
生产者B 读 in = 100      ← 都以为从 100 开始写
A 写数据到 100，in = 108
B 写数据到 100，in = 108  ← B 覆盖了 A 的数据，且 in 只前进一次
```

结果数据互相覆盖、游标错乱。**所以多生产者/多消费者场景必须额外加锁，或改用带 CAS 原子操作的 MPMC 队列。** 本项目的用法全部是 SPSC，天然安全。

---

## 6. 内存屏障与 Cortex-M 可见性

在真正严谨的无锁实现里，§5.2 的"顺序"不仅是代码书写顺序，还要防止**编译器重排**和**CPU 乱序执行**打乱它。标准做法是在拷贝数据和更新游标之间插入内存屏障：

```c
memcpy(...);
__DMB();            // Data Memory Barrier：保证前面的写先于后面的写完成
fifo->in += len;
```

对本项目（STM32 Cortex-M4/M7）来说：

- **Cortex-M 是单核**，且对普通内存基本是顺序一致的，ISR 与任务之间不存在真正的多核缓存不一致问题，因此很多实现省略了显式屏障也能工作。
- 但把游标声明为 **`volatile`** 是有意义的：阻止编译器把 `in`/`out` 缓存到寄存器里，确保每次都从内存读最新值。
- **H7 上的真正坑是 D-Cache**，不是内存屏障。DMA 直接往 SRAM 写数据，CPU 从 Cache 读到的是旧值，所以在 `kfifo_put` 之前必须 `BSP_CACHE_INVALIDATE(buf, size)` 让 CPU 重新从内存加载。这一点在 [[RM_BSP与模块教程]] 的 Cache 一致性小节有专门说明。

> 小结：在单核 Cortex-M 上，kfifo 的无锁正确性主要靠"游标更新顺序 + volatile"，屏障是加固；而 DMA + D-Cache 的一致性要靠 Cache 维护指令单独解决，两者别混淆。

---

## 7. 在本项目中的实际应用

kfifo 属于 `utils` 库（见 [[RM_CMake教程]] 中它被编译为 OBJECT 库），被 CAN、UART、USB 三条数据链路复用。

### 7.1 CAN 收发缓冲

每条 CAN 总线内部维护一对 kfifo（见 [[RM_BSP与模块教程]]）：

```
Can_Bus
├── tx_fifo (kfifo)  ← 发送队列
└── rx_fifo (kfifo)  ← 接收队列
```

**接收路径**（ISR 生产 → 任务消费）：

```
硬件 FDCAN 收到帧 → RX FIFO 中断
  → HAL_FDCAN_RxFifo0Callback()（ISR 上下文）
    → 从硬件读出所有消息
    → kfifo_put(&bus->rx_fifo, msg)     ← 快速入队，不做解析
    → tx_semaphore_put(&g_can_rx_sem)   ← 只发个信号就返回
  → ISR 返回

can_rx_task（任务上下文）
  while (1) {
    tx_semaphore_get(&g_can_rx_sem, TX_WAIT_FOREVER);  // 等唤醒
    while (kfifo_get(&bus->rx_fifo, &msg)) {           // 排空队列
      dev = can_dev_find(bus, msg.id);                 // 慢处理放这里
      dev->callback(dev);
    }
  }
```

这正是 [[RM_ThreadX_RTOS教程]] 里反复出现的模式：**ISR 只做 `kfifo_put` + 发信号，任务被唤醒后 `kfifo_get` 慢慢处理**。

### 7.2 UART 接收

UART 用 DMA + 空闲中断接收不定长数据，同样落到 kfifo：

```
UART RX 线空闲（IDLE 中断）
  → HAL_UARTEx_RxEventCallback(huart, received_size)
    → BSP_CACHE_INVALIDATE(rx_buffer, size)          ← H7 先刷 Cache（§6）
    → kfifo_put(&dev->rx_fifo, rx_buffer, size)      ← 入队
    → tx_event_flags_set(&dev->event_flags, RX_EVENT) ← 通知等待任务
```

### 7.3 USB CDC 与视觉数据

USB 虚拟串口（CDC ACM）接收也用 kfifo 缓存视觉上位机数据。视觉模块与主控制循环之间的数据共享，正是"用 kfifo 无锁环形缓冲而非互斥量"的思路（见 [[RM_ThreadX_RTOS教程]] §9.3 线程间数据共享）。

> ⚠️ **正确用法**：USB 端点数据应先收进**独立落地 buffer**，再由 `bulk_out` 回调调用 `kfifo_in()` 正规写入。**不要**让 USB 硬件直接把数据落进 kfifo 存储区、再手动推进 `in`——那样会因包长与 4 字节对齐凑洞而破坏 `in - out` 语义，导致除帧头外的数据全丢。这个坑的完整排障过程见 [[RM_USB_CDC虚拟串口调试实录]]。另外，"DWC2 需要 4 字节对齐"仅在 **DMA 开启**时成立；本项目 DMA 关闭（slave/FIFO 模式），无此要求。

---

## 8. 满策略：丢弃最旧数据

kfifo 本身满了会**截断写入**（`kfifo_put` 返回实际写入量，写不下的部分丢弃）。但对实时控制，这个默认行为不够——我们宁可丢旧数据也要保证最新数据能进去。

所以 CAN 发送队列采用了**满时丢弃最旧消息**的策略：

```
应用调用 BSP_CAN_Send()（任务上下文）
  → kfifo_put(&bus->tx_fifo, msg)
    → 如果满了，先 kfifo_get 丢弃最旧一帧，再放入新帧
  → tx_semaphore_put(&g_can_tx_sem)
```

**为什么？** 对控制指令而言，一帧过期的电机力矩指令毫无价值，甚至有害。保留最新数据、丢弃最旧数据，能保证执行器拿到的永远是最贴近当前状态的指令。这是实时系统区别于普通通信队列（后者往往丢最新、保证不丢包）的一个典型取舍。

---

## 9. 常见陷阱与使用规范

**① 容量必须是 2 的幂**
否则 `& (size-1)` 取模失效，缓冲区会错乱。写 256、512、1024，别写 100、500。

**② 严守 SPSC 约束**
一个 kfifo 只能有一个生产者、一个消费者。想让两个任务往同一个 fifo 写？加锁，或者一人一个 fifo。这是最容易被违反的规则。

**③ 游标更新顺序不能反**
`put`：先 `memcpy` 后 `in += len`；`get`：先 `memcpy` 后 `out += len`。反了就会读到半成品。

**④ H7 上 DMA + Cache 要单独处理**
kfifo 的无锁性和 Cache 一致性是两回事。DMA 收数据后，入队前记得 `BSP_CACHE_INVALIDATE`；发数据前记得 `BSP_CACHE_CLEAN`。

**⑤ `kfifo_get` 返回值要判断**
它返回的是**实际读到的字节数**，可能小于请求量（队列里没那么多数据）。别假设一定读满。

**⑥ 返回内部指针要立即用**
像视觉模块 `Module_Vision_Receive()` 返回的是内部缓冲区指针，下一帧数据可能就把它覆盖了——拿到后立刻处理或拷走，别存着慢慢用。

---

## 10. 核心 API 速查

| 函数 | 作用 | 上下文 | 返回 |
|------|------|--------|------|
| `kfifo_init(fifo, buf, size)` | 初始化，绑定缓冲区 | 初始化期 | — |
| `kfifo_put(fifo, data, len)` | 入队（生产者） | ISR / Task | 实际写入字节数 |
| `kfifo_get(fifo, data, len)` | 出队（消费者） | Task | 实际读出字节数 |
| `kfifo_len(fifo)` | 当前已用长度 | 任意 | 字节数 |
| `kfifo_avail(fifo)` | 剩余可写空间 | 任意 | 字节数 |
| `kfifo_is_empty(fifo)` | 是否为空 | 任意 | bool |
| `kfifo_is_full(fifo)` | 是否已满 | 任意 | bool |

### 一句话总结

kfifo 用一个 2 的幂大小的环形缓冲区加两个只增游标，让**单个生产者和单个消费者各写各的指针、各碰各的内存区**，从而在不加锁的前提下完成 ISR 到任务的数据传递。它是本项目"中断只搬运、任务做处理"这套解耦架构能跑起来的底层基石。

---

## 相关文档

- [[RM_BSP与模块教程]] —— CAN/UART/USB 外设如何用 kfifo，以及 H7 Cache 一致性
- [[RM_ThreadX_RTOS教程]] —— ISR 与任务的生产者-消费者模式、线程间无锁数据共享
- [[RM_CMake教程]] —— kfifo 所在的 utils 库如何被编译链接
- [[RM_USB_CDC虚拟串口调试实录]] —— kfifo 被误用为 USB 包落点导致丢帧的真实案例，含 in/out 自增溢出自愈原理





