# kfifo 无锁环形缓冲区

> SPSC（单生产者单消费者）无锁环形队列，用位运算取余替代取模，`__DMB()` 保证内存可见性。

## SPSC 架构

kfifo 只支持**一个生产者 + 一个消费者**。项目中：

- `rx_fifo`：HAL 中断写（唯一生产者），CAN RX Task 读（唯一消费者）—— 严格 SPSC
- `tx_fifo`：CAN TX Task 读（唯一消费者），但**生产者有多个**（电机任务、板间通信、超级电容等都会调 `BSP_CAN_Send` 往同一条总线的 tx_fifo 写）

> **注意**：tx_fifo 的多生产者场景破坏了 SPSC 前提，代码中没有加锁保护。当前依赖 motor task 统一调度时序避免实际并发，但这是一个潜在隐患。详见 [[02_code_twin/board/bsp/CAN/bsp_can#发送端多生产者问题]]。

## 位运算取余快速索引

`in` 和 `out` 是持续递增的整数（不回绕），通过 `& mask` 映射到缓冲区位置：

```c
unsigned int off = fifo->in & fifo->mask;  // 等价于 in % size，但快得多
```

前提：缓冲区大小必须是 **2 的幂**。`mask = size - 1`，初始化时 `rounddown_pow_of_two()` 保证。

`in - out` 就是队列中的元素数，不需要考虑回绕，因为无符号整数自然回卷。详见 [[01_extracted/algorithm/computer-basics#无符号整数回卷]]。

## 内存屏障

```c
memcpy(fifo->data + off * esize, element, esize);  // ① 写数据
__DMB();                                             // 屏障
fifo->in++;                                          // ② 发布索引
```

`__DMB()` 保证 ① 的内存写入对其他核心/中断可见后，才执行 ②。没有它，消费者可能看到 `in` 更新了但数据还没写完。

`__DMB()` 不是锁，不阻止中断打断，只约束自己的指令顺序。正确性靠 SPSC 保证：消费者只读 `out` 到 `in-1` 范围，生产者还没 `in++` 时消费者看不到新数据。

## `__DMB()` 来自哪里

CMSIS Core 提供（`cmsis_gcc.h`），对应 ARM `DMB` 指令。Cortex-M3/M4/M7 都支持。F103 移植不需要改。

## 参考视频

链表与动态内存分配：https://www.bilibili.com/video/BV1M7w7zQEdm?t=1005.9
