# KFIFO Risk Analysis

## 1. 背景与范围

本文记录工程中 `kfifo` 相关的风险点、触发条件、影响范围和建议修复方式。当前阶段只整理已经讨论过的结论：

- 目标平台：`board/dji_c`，即 F4 / `STM32F407xx` 分支。
- 当前已展开问题：CAN RX 软件 `rx_fifo` 满时静默丢新帧；CAN2 多任务写同一个 `tx_fifo`；TX 满时 `get + put` 不是原子事务；USB RX FIFO 满时静默丢包；USB RX 事件标志与 FIFO 剩余数据不同步。
- 当前只做框架预留、暂不展开的问题：UART 事件与 FIFO 状态不同步、UART DMA RX 空间不足时的裁剪与位置更新策略、`kfifo` 的 SPSC 使用边界等。

后续每确认一个问题，再补充对应章节，避免把未完全验证的判断写成确定结论。

## 2. kfifo 原理简介

`kfifo` 是一个固定容量的环形 FIFO，用来在两个执行上下文之间传递数据。它不会动态扩容，写到缓冲区尾部后会绕回开头继续使用。

### 2.1 核心状态

当前实现通过几个字段描述 FIFO 状态：

- `data`：实际数据缓冲区。
- `mask`：容量掩码。容量必须是 2 的幂，实际下标通过 `index & mask` 计算。
- `esize`：单个元素大小。CAN 中使用的是 `sizeof(BSP_CanMsg_t)`，UART/USB 字节流通常是 `1`。
- `in`：累计写入计数，表示下一次写入的位置。
- `out`：累计读取计数，表示下一次读取的位置。

`in` 和 `out` 不是简单的数组下标，而是持续递增的计数器。真正访问数组时才通过掩码回绕：

```c
off = fifo->in & fifo->mask;   // 写入位置
off = fifo->out & fifo->mask;  // 读取位置
```

这样做的好处是满/空判断简单：

```text
len   = in - out
empty = (in == out)
full  = (in - out) > mask
```

例如容量是 8 时，`mask = 7`，当 `in - out == 8` 就表示 FIFO 已满。

### 2.2 写入与读取流程

单元素写入 `kfifo_put()` 的逻辑是：

1. 检查 FIFO 是否已满。
2. 用 `in & mask` 算出写入槽位。
3. `memcpy()` 写入一个元素。
4. 内存屏障后推进 `in`。

单元素读取 `kfifo_get()` 的逻辑是：

1. 检查 FIFO 是否为空。
2. 用 `out & mask` 算出读取槽位。
3. `memcpy()` 取出一个元素。
4. 内存屏障后推进 `out`。

也就是说，正常 SPSC 使用中：

- 生产者负责写数据并推进 `in`。
- 消费者负责读数据并推进 `out`。

双方都会读取对方的计数器来判断满/空，但只有各自修改自己的计数器。

### 2.3 为什么单写单读相对安全

当前实现更适合单生产者单消费者（SPSC）模型：

- 生产者只调用 `kfifo_put()` / `kfifo_in()`，主要推进 `in`。
- 消费者只调用 `kfifo_get()` / `kfifo_out()`，主要推进 `out`。

在这种边界内，`kfifo` 可以保持轻量，不需要每次访问都加锁。但如果多个任务同时写同一个 FIFO，或者生产者为了“覆盖旧帧”去调用 `kfifo_get()`，就会破坏这个边界，需要额外临界区或互斥保护。

SPSC 并不等于“永远不丢数据”。如果生产速度超过消费速度，FIFO 最终仍然会满。**当前 `kfifo_put()` 在满时返回失败，不会自动扩容，也不会自动覆盖旧数据。**

### 2.4 “新帧覆盖旧帧”为什么不是 kfifo 默认语义

需要特别说明：`kfifo_put()` 本身没有“新帧覆盖旧帧”的语义。FIFO 满时，它直接返回失败。当前 CAN TX 中所谓“丢旧保新”是上层代码自己用 `get + put` 组合出来的策略，不是 `kfifo` 的内建安全行为。

如果生产者在满时主动调用 `kfifo_get()` 删除旧数据，它就会修改 `out`。而在 SPSC 模型里，`out` 本来属于消费者。这样会让同一个字段被两个上下文修改，必须额外加锁或进入临界区。

因此，“覆盖旧帧”不是不能做，而是不能在现有 `kfifo` 上裸写 `get + put` 并假设它天然安全。

## 3. 严重程度定义

| 等级 | 含义 | 处理建议 |
| :--- | :--- | :--- |
| Critical | 有明确可构造的并发破坏、死锁、链路卡死，或可能直接影响控制闭环稳定性。 | 优先修复；代码层面需要互斥、临界区或结构重构。 |
| Warning | 有合理触发条件，可能造成丢帧、延迟、状态滞后，但通常不会立刻导致系统失效。 | 建议修复或增加诊断；可结合实车压力测试排序。 |
| Low | 触发概率低，或影响通常可被周期刷新、离线保护等机制吸收。 | 建议增加计数/日志；暂不作为阻塞问题。（可以完全不改就是了，但是以后出问题要可以想起来有这么个东西） |
| TBD | 问题尚未充分复核，当前只保留待讨论入口。 | 不写确定结论，等代码路径和触发条件确认后再定级。 |

## 4. 问题总表

| 模块 | ID | 位置 | 当前判断 | 风险等级 | 状态 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| CAN | KFIFO-CAN-RX-001 | `board/bsp/CAN/bsp_can.c` | F4 CAN RX 为单写单读；软件 `rx_fifo` 满时会静默丢新帧。当前工程优先级和 CAN 负载下触发概率较低。 | Low | 可以不改 |
| CAN | KFIFO-CAN-TX-001 | `board/bsp/CAN/bsp_can.c` | CAN2 存在多任务写同一个 `tx_fifo` 的风险，重点在电机发送与板间通信同时使用 CAN2 的场景。 | Warning | 能改，万一更丝滑了呢 |
| CAN | KFIFO-CAN-TX-002 | `board/bsp/CAN/bsp_can.c` | TX FIFO 满时 `get + put` 的“丢旧保新”不是原子事务；该问题可以和 CAN2 多生产者写入一起修复。 | Warning | 能改，万一更丝滑了呢 |
| USB | KFIFO-USB-001 | `board/bsp/USB/usbd_cdc_acm_user.c` | 当前 F4 源码下，USB RX FIFO 满时会静默丢包或截断。它和 CAN RX 满 FIFO 类似，属于容量/负载边界问题。 | Low      | 可以不改 |
| USB | KFIFO-USB-002 | `board/bsp/USB/usbd_cdc_acm_user.c` | `cdc_acm_recv()` 先清 RX 事件再读 FIFO；如果一次未读空，剩余数据可能留在 FIFO 中等待下一个“新事件”才能继续处理。 | Warning | 可以不改 |
| UART | KFIFO-UART-001 | `board/bsp/UART/bsp_uart.c` | UART 读取路径和 USB-002 类似：先清 RX 事件再读 FIFO；如果一次未读空，剩余数据可能等待下一次 RX 事件。 | Low | 可以不改 |
| UART | KFIFO-UART-002 | `board/bsp/UART/bsp_uart.c` | UART DMA RX 满载时，DMA 环形写入位置、`last_buf_pos` 与软件 FIFO 计数可能不同步；当前属于低概率满载边界风险。 | Low | 不改 |

## 5. 问题详情

### 5.1KFIFO-CAN-RX-001：F4 CAN RX 软件 FIFO 满时静默丢新帧

严重程度：Low

当前判断：低概率边界风险。它不是 `kfifo` 并发破坏问题，也不是当前优先级设计下的主要风险点。

代码路径

F4 CAN RX 的实际路径：

```text
CAN 硬件 FIFO0/FIFO1
  -> CANx_RX0/CANx_RX1 IRQ
  -> HAL_CAN_IRQHandler()
  -> HAL_CAN_RxFifo0/1MsgPendingCallback()
  -> can_f4_rx_callback()
  -> kfifo_put(&bus->rx_fifo, &msg)
  -> tx_semaphore_put(&g_can_rx_sem)
  -> CAN RX Task
  -> kfifo_get(&bus->rx_fifo, &msg)
  -> dev->rx_callback(...)
```

关键代码：

- F4 RX 回调入口：`board/bsp/CAN/bsp_can.c` 中 `can_f4_rx_callback()`。
- 软件 RX FIFO 写入：`kfifo_put(&bus->rx_fifo, &msg)`。
- CAN RX Task 读取：`board/bsp/CAN/bsp_can_task.c` 中 `can_rx_task_entry()`。

####  为什么这是单写单读

RX 侧的软件 `rx_fifo` 只有一个写者和一个读者：

- 写者：CAN RX ISR 回调。
- 读者：`CAN RX Task`。

模块层的 `rx_callback` 不直接读 `rx_fifo`。它拿到的是 `CAN RX Task` 从 FIFO 取出的局部消息。因此 RX 侧不属于多生产者多消费者问题，也不是当前 `kfifo` 并发破坏的重点。

####  触发条件

触发条件是软件 `rx_fifo` 被填满，而 `CAN RX Task` 还没来得及清空。典型原因包括：

- 短时间 CAN 报文集中到达。
- `CAN RX Task` 正在处理较慢的回调。
- 有较长时间的高优先级中断或临界区，使 RX 任务无法及时运行。
- 软件 FIFO 深度较小，当前 `BSP_CAN_RX_FIFO_SIZE` 为 8。

在当前工程中，CAN RX IRQ 优先级高，`CAN RX Task` 优先级也较高，因此正常负载下触发概率较低。

####  当前行为与后果

`kfifo_put()` 在 FIFO 满时返回 0。RX 回调当前只在写入成功时设置 `has_msg`，没有处理失败分支。因此满载时新收到的 CAN 帧会被静默丢弃。

可能后果：

- 偶发丢失一帧电机反馈或板间状态帧。
- 对周期性状态数据，一帧丢失通常会被下一帧覆盖，宏观影响较小。
- 如果持续丢帧，可能导致状态更新延迟，极端情况下影响离线检测或控制反馈。

#### 为什么不直接做“新帧覆盖旧帧”

如果 RX ISR 在 FIFO 满时直接执行：

```c
kfifo_get(&bus->rx_fifo, &dummy);
kfifo_put(&bus->rx_fifo, &msg);
```

那么 ISR 就会修改 `out` 指针，而 `out` 原本由 `CAN RX Task` 修改。这样会破坏 RX 侧单写单读边界，引入新的并发风险。

如果确实希望 RX 采用“丢旧保新”策略，需要专门实现：

- 在短临界区内完成 `get + put`。
- 或者对电机反馈、板间状态这类“最新状态更重要”的数据，改成按 CAN ID 保存 latest snapshot，而不是排队保存每一帧。

#### 5.1.6 建议修改

当前建议先做低侵入诊断，不直接改行为（不上更多的can设备理论上就不用改了，基本上不会触发）：

1. 增加 `rx_fifo_drop_count`，统计 `kfifo_put()` 失败次数。
2. 可选地按 bus 统计丢帧数量，便于确认是 CAN1 还是 CAN2 出现压力。
3. 实车压测确认是否真的发生 RX 软件 FIFO 满。
4. 如果确认发生，再决定是扩大 FIFO、加丢旧保新临界区，还是改为按 CAN ID 保存最新状态。

### 5.2 KFIFO-CAN-TX-001：CAN2 多任务写同一个 tx_fifo

严重程度：Warning

当前判断：这是一个真实可构造的并发问题。它不依赖 FIFO 满，只要两个任务同时向同一个 CAN2 `tx_fifo` 入队，就可能破坏 `kfifo_put()` 的单生产者假设。由于电机控制和板间通信都是周期刷新，单次丢帧通常不会立刻导致系统失效，因此暂定为 Warning；如果实测出现连续错帧或控制异常，可以上调等级。

#### 5.2.1 代码路径

F4 sentry/gimbal 下，CAN2 TX 的主要路径如下：

```text
robot_control_thread
  -> Module_BoardComm_Send()
  -> BSP_CAN_Send()
  -> bus->tx_fifo

motor thread
  -> Motor_DJI_Flush()
  -> BSP_CAN_SendMessage()
  -> bus->tx_fifo

CAN TX Task
  -> kfifo_get(&bus->tx_fifo, &msg)
  -> HAL_CAN_AddTxMessage()
```

关键代码：

- 板间通信发送：`apps/sentry/gimbal_board/robot_control.c` 中 `Module_BoardComm_Send()`。
- BoardComm 发送入口：`modules/BOARDCOMM/module_boardcomm.c` 中 `BSP_CAN_Send(boardcomm_dev, data, len)`。
- 电机线程：`modules/MOTOR/module_motor.c` 中 `motor_task_entry()`。
- DJI 电机打包发送：`modules/MOTOR/DJI/motor_dji.c` 中 `Motor_DJI_Flush()`。
- CAN2 DJI 发送分组：`modules/MOTOR/DJI/motor_dji.c` 中 `sender_assignment[5..9]`。
- BSP 入队：`board/bsp/CAN/bsp_can.c` 中 `BSP_CAN_Send()` / `BSP_CAN_SendMessage()`。
- TX 消费者：`board/bsp/CAN/bsp_can_task.c` 中 `can_tx_task_entry()`。

#### 5.2.2 为什么 CAN 回调不能隔离这个问题

`Can_Device` 的 `rx_callback` 只用于 RX 分发。TX 发送时并不会按设备回调隔离队列，而是按 `hcan` 找到对应的 `CAN_Bus_Manager`，然后写这个 bus 共享的 `tx_fifo`。

也就是说，不同电机、板间通信虽然有不同的 `Can_Device`、`tx_id` 和 `rx_callback`，但只要它们都使用 `BSP_CAN_HANDLE2`，最终都会写同一个 CAN2 `bus->tx_fifo`。

因此，CAN 回调机制不能解决 TX 多生产者同时写入的问题。

#### 5.2.3 触发条件

该问题需要同时满足：

1. 同一个 CAN bus 上有两个及以上任务级发送者。
2. 这些发送者可能在一次 `kfifo_put()` 尚未完成时发生抢占。
3. 它们最终写入同一个 `bus->tx_fifo`。

当前 基本上所有双板的 CAN2 都满足这个条件：

- `robot_control_thread` 每 2 tick 发送板间通信。
- `motor` 线程每 2 tick 刷新电机控制，并可能发送 CAN2 DJI 分组。
- `motor` 线程优先级高于 `robot_control_thread`，因此可以在 `robot_control_thread` 执行 `kfifo_put()` 中途抢占。

#### 5.2.4 一次可能的错误时序

假设 CAN2 `tx_fifo` 当前没有满，`in = N`。

1. `robot_control_thread` 调用 `Module_BoardComm_Send()`，准备发送板间通信帧。
2. 代码进入 `BSP_CAN_Send()`，组出 `BSP_CanMsg_t msg`。
3. `BSP_CAN_Send()` 调用 `kfifo_put(&bus->tx_fifo, &msg)`。
4. `kfifo_put()` 根据 `fifo->in & fifo->mask` 算出写入槽位。
5. `robot_control_thread` 把板间通信帧 `memcpy()` 到该槽位。
6. 此时还没执行 `fifo->in++`，tick 到来，`motor` 线程睡眠结束并抢占。
7. `motor` 线程进入 `Motor_DJI_Flush()`，调用 `BSP_CAN_SendMessage()` 发送 CAN2 电机帧。
8. 因为 `fifo->in` 仍然是 N，motor 线程也算出同一个写入槽位。
9. motor 线程把电机帧写到同一个槽位，覆盖刚才的板间通信帧。
10. motor 线程执行 `fifo->in++`，并释放 `g_can_tx_sem`。
11. `robot_control_thread` 恢复后继续执行自己的 `fifo->in++`。

最终可能出现两类结果：

- 板间通信帧被电机帧覆盖，本周期板间通信丢失。
- `in` 计数与实际写入内容不一致，后续 `CAN TX Task` 可能读到旧帧或重复帧。

#### 5.2.5 后果

可能后果包括：

- 偶发丢失一帧板间通信。
- 偶发丢失或覆盖一帧 CAN2 电机控制帧。
- CAN TX 队列顺序异常，发送旧帧或重复帧。

由于这些数据大多是周期刷新，单次错误通常会被下一周期覆盖。但如果时序反复命中，可能表现为板间状态偶发滞后、电机控制帧偶发抖动或 CAN2 发送顺序异常。

#### 5.2.6 建议修改

该问题的修复应和 `KFIFO-CAN-TX-002` 一起处理。原因是两者都发生在同一个 `bus->tx_fifo` 上：

- `KFIFO-CAN-TX-001` 关注多个生产者同时 `put`。
- `KFIFO-CAN-TX-002` 关注满队列时生产者执行 `get + put`，同时触碰 `in/out`。

因此，具体修复方案统一放到下一节，避免在两个问题里重复描述。

### 5.3 KFIFO-CAN-TX-002：TX FIFO 满时 get + put 不是原子事务

严重程度：Warning

当前判断：这是一个真实的并发边界问题，但触发前提是 TX FIFO 已满。当前工程中 CAN TX Task 优先级较高，正常情况下 FIFO 长时间满的概率不高，因此暂定为 Warning。它和上一节 CAN2 多生产者写入问题可以用同一套 TX FIFO 保护方案解决。

#### 5.3.1 问题原理

`kfifo` 的正常 SPSC 边界是：

- 生产者调用 `kfifo_put()`，推进 `in`。
- 消费者调用 `kfifo_get()`，推进 `out`。

但是当前 CAN TX 满队列分支写成了：

```c
if (!kfifo_put(&bus->tx_fifo, msg))
{
    BSP_CanMsg_t dummy;
    kfifo_get(&bus->tx_fifo, &dummy);
    kfifo_put(&bus->tx_fifo, msg);
}
```

这段代码的意图是“FIFO 满时丢掉最老消息，保证最新消息能入队”。问题在于：

- 第一次 `kfifo_put()` 会读写 `in`。
- `kfifo_get()` 会推进 `out`。
- 第二次 `kfifo_put()` 再次读写 `in`。

这三步组合在一起才是完整语义，但当前没有任何锁或临界区保护。中间如果被另一个生产者或 CAN TX Task 插入，语义就可能失效。

#### 5.3.2 触发条件

该问题需要满足：

1. 某个 CAN bus 的 `tx_fifo` 已满。
2. 某个任务调用 `BSP_CAN_Send()` 或 `BSP_CAN_SendMessage()`。
3. 第一次 `kfifo_put()` 失败后进入 `get + put` 分支。
4. 在 `get + put` 组合操作尚未完成时，其他生产者或 CAN TX Task 操作了同一个 `tx_fifo`。

CAN1/CAN2 理论上都可能触发，但 CAN2 风险更高，因为它同时承载电机发送和板间通信，更容易出现多生产者交错。

#### 5.3.3 一次可能的错误时序

假设 CAN2 `tx_fifo` 容量为 8，当前已满：

```text
in  = 20
out = 12
len = 8
```

1. `robot_control_thread` 调用 `BSP_CAN_Send()` 发送板间通信。
2. 第一次 `kfifo_put()` 发现 FIFO 满，返回 0。
3. 代码进入满队列分支，执行 `kfifo_get()`，删除最老的一帧，`out` 从 12 变成 13。
4. 此时 FIFO 临时空出一个槽位，但 `robot_control_thread` 还没来得及执行第二次 `kfifo_put()`。
5. `motor` 线程抢占，调用 `BSP_CAN_SendMessage()` 发送 CAN2 电机帧。
6. motor 线程看到 FIFO 现在不满，于是成功 `kfifo_put()`，占用了刚释放出来的槽位。
7. `robot_control_thread` 恢复，继续执行第二次 `kfifo_put()`。
8. 此时 FIFO 又满了，第二次 `kfifo_put()` 可能失败，但当前代码没有检查这个返回值。

最终结果可能是：

- 旧帧已经被删掉。
- motor 新帧成功入队。
- boardcomm 新帧没有入队，但调用者不知道失败。

这就破坏了原本“丢旧保新”的语义。

#### 5.3.4 可能后果

可能后果包括：

- 多删一帧旧消息。
- 最新消息没有真正入队。
- CAN TX 队列顺序与预期不一致。
- 在更复杂的抢占点下，`in/out` 与缓冲区内容短暂不一致，导致旧帧或重复帧被发送。

由于 CAN 控制帧和板间通信大多周期刷新，单次错误通常不会立刻造成系统失效。但如果在高负载下频繁触发，可能表现为 CAN2 控制帧偶发抖动、板间状态偶发滞后。

#### 5.3.5 修改方案

这个问题建议和 `KFIFO-CAN-TX-001` 一起修。可选方案如下。

##### 方案 A：per-bus TX mutex 保护所有 tx_fifo put/get（推荐）

为每个 `CAN_Bus_Manager` 增加一把 TX mutex，所有访问该 bus `tx_fifo` 的地方都走同一把锁：

- `BSP_CAN_Send()` 入队前拿锁，入队后释放。
- `BSP_CAN_SendMessage()` 入队前拿锁，入队后释放。
- `CAN TX Task` 从 `tx_fifo` 取消息时拿锁，取完立刻释放。

注意：mutex 只保护软件 FIFO，不包住 HAL 发送。也就是说，`HAL_CAN_AddTxMessage()` 和可能发生的 `tx_thread_sleep(1)` 必须在锁外执行。

推荐结构：

```c
/* 生产者侧 */
tx_mutex_get(&bus->tx_mutex, TX_WAIT_FOREVER);

if (!kfifo_put(&bus->tx_fifo, msg))
{
    BSP_CanMsg_t dummy;
    (void)kfifo_get(&bus->tx_fifo, &dummy);
    (void)kfifo_put(&bus->tx_fifo, msg);
}

tx_mutex_put(&bus->tx_mutex);
tx_semaphore_put(&g_can_tx_sem);
```

```c
/* CAN TX Task */
tx_mutex_get(&bus->tx_mutex, TX_WAIT_FOREVER);
has_msg = kfifo_get(&bus->tx_fifo, &msg);
tx_mutex_put(&bus->tx_mutex);

if (has_msg)
{
    while (HAL_CAN_AddTxMessage(msg.hcan, &TxHeader, msg.data, &mailbox) != HAL_OK)
    {
        tx_thread_sleep(1);
    }
}
```

优点：

- 同时解决多生产者写入和满队列 `get + put` 非原子问题。
- 改动集中在 BSP CAN 层。
- 不需要改动电机、板间通信等上层模块。

约束：

- 每次 CAN TX 入队/出队多一次 mutex 开销。
- 该方案要求 `BSP_CAN_Send()` / `BSP_CAN_SendMessage()` 只在任务上下文调用。当前工程的主要发送来源是任务上下文，这不是当前已知 bug。
- 如果未来需要在 ISR 中请求 CAN 发送，应改为投递到任务级队列，或额外提供 ISR-safe 的非阻塞接口。

##### 方案 B：改用 ThreadX queue 或统一 CAN TX 队列封装

如果后续 CAN 发送源继续增加，可以考虑用 ThreadX queue 承担多生产者同步，或者封装一个统一的 CAN TX 入队接口，把满队列策略、统计计数、信号量释放全部集中在一个地方。

优点：

- 多生产者同步语义更清晰。
- 后续扩展更稳。

缺点：

- 改动比 mutex 包装更大。
- 需要重新评估队列深度、消息大小和内存占用。

当前建议：优先采用方案 A。它能同时处理 `KFIFO-CAN-TX-001` 和 `KFIFO-CAN-TX-002`，改动集中，语义也最接近当前“丢旧保新”的设计。

---

### 5.4 KFIFO-USB-001：USB RX FIFO 满时静默丢包

严重程度：Low

当前判断：这和 `KFIFO-CAN-RX-001` 有相似之处，都是“生产速度暂时高于消费速度时，软件 FIFO 作为容量边界开始丢数据”的问题，而不是 `kfifo` 并发破坏问题。当前 F4 视觉链路负载不高时，触发概率预计也不高。但它和 CAN RX 的底层路径并不完全一样：CAN 是“硬件 FIFO -> ISR -> `kfifo_put()` 单帧入队”，USB 是“OUT 端点回调 -> `kfifo_in()` 字节流入队”，并且 USB 侧还包含端点重新提交和事件标志。

#### 5.4.1 代码路径

当前 F4 USB RX 的实际路径如下：

```text
USB 主机
  -> USB OTG FS OUT EP
  -> usbd_cdc_acm_bulk_out()
  -> kfifo_in(&cdc_rx_fifo, cdc_out_ep_buffer, nbytes)
  -> tx_event_flags_set(&usb_event_flags, BSP_USB_EVENT_RX, TX_OR)
  -> usbd_ep_start_read(busid, CDC_OUT_EP, cdc_out_ep_buffer, CDC_MAX_MPS)
  -> cdc_acm_recv()
  -> kfifo_out(&cdc_rx_fifo, data, to_read)
  -> vision_thread_entry()
```

关键代码：

- 初次提交 OUT 接收：`board/bsp/USB/usbd_cdc_acm_user.c` 中 `USBD_EVENT_CONFIGURED` 分支。
- USB OUT 回调：`board/bsp/USB/usbd_cdc_acm_user.c` 中 `usbd_cdc_acm_bulk_out()`。
- 软件 FIFO 写入：`kfifo_in(&cdc_rx_fifo, cdc_out_ep_buffer, nbytes)`。
- 上层读取：`board/bsp/USB/usbd_cdc_acm_user.c` 中 `cdc_acm_recv()`。
- 当前消费者：`modules/VISION/module_vision.c` 中 `vision_thread_entry()`。

#### 5.4.2触发条件

这条风险的触发条件是：

1. 上位机在一段时间内持续发送数据。
2. `vision_thread` 或其他上层消费者来不及把 `cdc_rx_fifo` 清空。
3. `cdc_rx_fifo` 可用空间耗尽。

这和 `KFIFO-CAN-RX-001` 一样，本质上都是“软件 FIFO 满时的过载策略”问题。区别只在于：

- CAN RX 是按帧用 `kfifo_put()` 入队。
- USB RX 是按字节流用 `kfifo_in()` 入队。
- USB 侧即使本次写不进去，也会继续重新提交下一次 OUT 接收。

#### 5.4.3当前行为与后果

当前行为是：

- 如果 `cdc_rx_fifo` 还有空间，`kfifo_in()` 正常写入。
- 如果空间不足，`kfifo_in()` 只会写入能放下的那一部分。
- 如果已经满了，`kfifo_in()` 实际写入 0 字节。
- 无论这次实际写入多少，当前回调尾部都会继续 `usbd_ep_start_read(...)`。

所以是：

- 新到的数据被静默丢弃或截断。
- 上层协议可能偶发丢一帧，或因为字节流被截断而跳过一次解析。
- 在当前视觉这类周期刷新场景下，单次丢失通常会被后续新包覆盖，宏观影响偏小。

#### 5.4.4 建议修改（也就是不建议修改）

当前建议和 `KFIFO-CAN-RX-001` 类似，先做低侵入诊断，不急着重构：

1. 增加 `usb_rx_fifo_drop_count` 或 `usb_rx_drop_bytes`，统计 `kfifo_in()` 实际写入小于 `nbytes` 的次数与字节数。
2. 实车或联调时做持续灌包测试，确认当前视觉链路是否真的出现 USB RX FIFO 满。
3. 如果确认发生，再决定是扩大 `CDC_RX_FIFO_SIZE`，还是在更高层做协议重组与最新状态保留。

### 5.5 KFIFO-USB-002：USB RX 事件标志与 FIFO 剩余数据可能不同步

严重程度：Warning

当前判断：这是当前源码下真实存在的逻辑风险。它不要求 USB RX FIFO 满，只要求某次 `cdc_acm_recv()` 进入时 `kfifo_len(&cdc_rx_fifo) > buf_size`，就可能出现“FIFO 里还有数据，但 RX 事件已经被清掉，读取线程重新去等新事件”的情况。

这条问题和 `KFIFO-USB-001` 不同。`KFIFO-USB-001` 需要 `cdc_rx_fifo` 接近 1024 字节满载，触发门槛较高；本问题只需要当前 FIFO 积压超过上层本次读取缓冲区大小。当前视觉线程每次只读 64 字节，因此门槛是“某次读取前 FIFO 中已经超过 64 字节”。这不代表它会高频发生，但代码逻辑上具备触发条件，应按 Warning 记录并建议验证。

#### 5.5.1 问题原理

先明确 USB RX 的软件链路：

```text
USB 主机 / 上位机
  -> USB OUT 端点
  -> usbd_cdc_acm_bulk_out()
  -> kfifo_in(&cdc_rx_fifo, cdc_out_ep_buffer, nbytes)
  -> tx_event_flags_set(&usb_event_flags, BSP_USB_EVENT_RX, TX_OR)
  -> vision_thread_entry()
  -> cdc_acm_recv()
  -> kfifo_out(&cdc_rx_fifo, data, to_read)
```

这里有两个不同的概念：

- `cdc_rx_fifo`：真实保存 USB RX 字节流的缓存，里面到底还有多少数据，要看 `kfifo_len(&cdc_rx_fifo)`。
- `BSP_USB_EVENT_RX`：ThreadX 事件标志，只表示“曾经有 USB RX 回调写入过数据，并通知过读取线程”。

当前 `cdc_acm_recv()` 的逻辑顺序是：

1. `tx_event_flags_get(&usb_event_flags, BSP_USB_EVENT_RX, TX_OR_CLEAR, ...)`
2. 读取当前 `kfifo_len(&cdc_rx_fifo)`
3. 最多只按 `buf_size` 从 FIFO 取一批数据

问题就在第 1 步：`TX_OR_CLEAR` 会先把 RX 事件清掉。事件被清掉，只能说明“这次唤醒已经消费过”，不能说明 `cdc_rx_fifo` 已经被读空。

如果第 3 步因为 `buf_size` 不够，只读走 FIFO 的前半段，那么 FIFO 里会继续保留剩余字节。但下一次 `cdc_acm_recv()` 仍然会先去等 `BSP_USB_EVENT_RX`。如果这段时间没有新的 USB OUT 回调设置事件，读取线程就会阻塞，旧数据留在 FIFO 里。

#### 5.5.2 触发条件

这条问题的精确触发条件是：

1. USB 回调已经把一笔或多笔数据写进 `cdc_rx_fifo`。
2. 某次 `cdc_acm_recv()` 被唤醒时，FIFO 可读字节数大于 `buf_size`。
3. 本次只读走前半段数据，FIFO 仍有剩余。
4. 下一次读取前没有新的 USB RX 事件到来。

当前 `vision_thread_entry()` 每次只给 `cdc_acm_recv()` 提供 `64` 字节缓冲。因此在当前视觉链路中，这条问题可以简化为：

```text
进入 cdc_acm_recv() 时，cdc_rx_fifo 中已有数据量 > 64 字节
```

实际运行中它不一定经常发生。如果上位机稳定低频发送，每次只发一帧约 21 字节的 `ReceivePacket`，且 `vision_thread` 能及时运行，那么 FIFO 通常很快被读走，不会积压超过 64 字节。

但它也不是不可能。下面几种情况会提高触发概率：

- 上位机短时间连发多帧，例如 4 帧视觉数据约 84 字节。
- `vision_thread` 被更高优先级任务、中断或临界区延迟。
- USB OUT 回调连续发生，应用线程还没来得及运行。
- 后续协议包变大，或上位机发送频率提高。
- 调试、重连、缓存 flush 时，上位机一次性吐出多包。

#### 5.5.3 可能的错误时序

以当前视觉协议为例，`ReceivePacket` 打包后大小约为 21 字节。假设上位机连续发来 4 帧，共 84 字节：

1. USB OUT 回调连续发生，`kfifo_in()` 把 84 字节数据写入 `cdc_rx_fifo`。
2. 回调过程中设置了 `BSP_USB_EVENT_RX`，通知视觉线程“有新数据到过”。
3. `vision_thread` 被唤醒，调用 `cdc_acm_recv(buf, 64, &rx_len, TX_WAIT_FOREVER)`。
4. `cdc_acm_recv()` 在进入时先用 `TX_OR_CLEAR` 清掉 `BSP_USB_EVENT_RX`。
5. 它看到 FIFO 当前可读 `84` 字节，但因为 `buf_size = 64`，这次只读走前 `64` 字节。
6. FIFO 中仍然剩下 `20` 字节旧数据。
7. `vision_thread` 处理完这 64 字节后再次调用 `cdc_acm_recv()`。
8. 如果这时没有新的 USB RX 回调到来，线程就会阻塞等下一个事件，而那 `20` 字节会继续留在 FIFO 中。

这就形成了“FIFO 里明明还有数据，但读取线程却回去等门铃”的状态。



#### 5.5.4 可能后果

可能后果包括：

- FIFO 中残留的旧数据处理延后。
- 视觉线程对上位机数据的响应出现间歇性停顿。
- 字节流协议在边界处更容易出现粘包、拆包后的解析抖动。
- 如果上层按固定包头包尾扫描，残留数据与后续新数据拼接时，可能表现为偶发漏解析一帧。

当前它不建议按 Critical 处理，原因是视觉数据大多周期刷新，单次延迟或漏解析通常会被后续新包覆盖。但由于触发门槛只是“积压超过 64 字节”，比 `KFIFO-USB-001` 的“FIFO 满 1024 字节”更低，因此也不应直接忽略。



这个问题的触发通俗来讲不就是连发之后又不发，会导致断掉一个包，等到下次发再唤醒，把断掉的后半部分一起读出去

然后稳定快发可能会导致边界抖动，有点延迟

超级稳定的快发，每次大于64，会变得不连续，不是每次FIFO进kfifo都刚好是完整的几个包，可能会有几个断的。FIFO会有滞留，但是稳定的话就影响不大。

综上所述，就是看视觉的上位机发消息，要是是在2次 vision_task之间传了四包也就是84bytes，就会有一定概率延迟不稳定。所以要综合看线程优先级和上位机回传的适配度。现在能跑也是不用改了，但是以后换车或者换视觉啥的出问题了，要想起来看是不是这个的问题，改起来应该也不难。

#### 5.5.5 建议修改

这条问题建议按和 UART 相同的思路修：

1. `cdc_acm_recv()` 进入时先检查 `kfifo_len(&cdc_rx_fifo)`。
2. 如果 FIFO 里已经有数据，直接读取，不要先等事件。
3. 只有在 FIFO 为空时，才去 `tx_event_flags_get(..., TX_OR_CLEAR, ...)` 等待新事件。
4. 被唤醒后再次检查 FIFO，再决定本次读取长度。

也就是说，事件标志应该只作为“唤醒门铃”，不能代替“FIFO 当前是否为空”的真实状态判断。

在正式改行为之前，可以先加一个很低侵入的验证计数：在 `cdc_acm_recv()` 读取前统计 `avail > buf_size` 的次数。只要这个计数不为 0，就证明当前工程中确实发生过“本次读不空 FIFO”的场景。

### 5.6 KFIFO-UART-001：UART RX 事件标志与 FIFO 剩余数据可能不同步

严重程度：Warning

当前判断：这和 `KFIFO-USB-002` 是同一类问题，区别只在于 USB 使用 `cdc_acm_recv()`，UART 使用 `BSP_UART_Read()`。它不是 `kfifo` 并发破坏问题，而是“是否继续读 FIFO”的逻辑没有以 FIFO 当前状态为准。

#### 5.6.1 问题原理

UART 的 IT / DMA 读取路径中，`BSP_UART_Read()` 会先等待并清除 RX 事件：

- `board/bsp/UART/bsp_uart.c`：`tx_event_flags_get(&device->event_flags, BSP_UART_EVENT_RX, TX_OR_CLEAR, ...)`
- 随后才读取 `kfifo_len(&device->rx_fifo)`。
- 最后按 `buf_size` 限制调用 `kfifo_out(&device->rx_fifo, buf, to_read)`。

如果这一次 `buf_size` 小于 FIFO 中已有的数据量，`BSP_UART_Read()` 只会读走前一部分。剩余数据还在 `rx_fifo` 里，但 RX 事件已经被 `TX_OR_CLEAR` 清掉了。下一次调用 `BSP_UART_Read()` 时，代码仍然先等待新的 `BSP_UART_EVENT_RX`，而不是先检查 FIFO 里是否已经有旧数据。

所以它和 USB-002 的本质完全一样：事件标志只是“有新数据到过”的门铃，不等于“FIFO 当前是否为空”的真实状态。

#### 5.6.2 触发条件与后果

触发条件也和 5.5 类似：

1. UART 回调或 DMA 处理已经把数据计入 `rx_fifo`。
2. 某次 `BSP_UART_Read()` 被唤醒时，`kfifo_len(&device->rx_fifo) > buf_size`。
3. 本次读取没有读空 FIFO。
4. 下一次读取前没有新的 UART RX 事件到来。

可能后果是 UART 数据处理延后，或者字节流协议在拆包边界上出现间歇性卡顿。当前是否会在实车触发，要看具体 UART 设备的发送频率、单包长度、读取线程周期和 `buf_size` 设置；因此它更适合作为同类边界风险记录，不应和 CAN TX 多生产者问题放在同一优先级处理。

#### 5.6.3 建议修改

建议直接沿用 5.5 的修法，不再单独设计一套逻辑：

1. `BSP_UART_Read()` 进入时先检查 `kfifo_len(&device->rx_fifo)`。
2. FIFO 已有数据时直接读，不等待事件。
3. 只有 FIFO 为空时，才调用 `tx_event_flags_get(..., TX_OR_CLEAR, ...)` 等待新 RX 事件。
4. 被事件唤醒后再次检查 FIFO，再按 `buf_size` 读取。

如果暂时不改行为，也可以先加诊断计数：统计 `BSP_UART_Read()` 中 `avail > buf_size` 的次数，用实测判断当前 UART 链路是否真的发生过“读一次没读空”的情况。

### 5.7 KFIFO-UART-002：UART DMA RX 满载时的位置同步边界

严重程度：Low

当前判断：这是 UART DMA RX 满载时的边界风险，不是当前实车的主要问题。正常稳定串口流下，只要读取任务能及时消费，`rx_fifo` 不接近满，就不会进入这个分支。

UART DMA 模式下，`rx_buf` 同时承担两个角色：一方面它是 DMA 的环形接收缓冲区，硬件会持续写入；另一方面它又被 `kfifo_init()` 作为软件 FIFO 的底层存储区。DMA 回调通过 `curr_pos` 和 `last_buf_pos` 计算本次新增数据量 `new_len`，再根据 `rx_fifo` 剩余空间裁剪 `new_len`。

风险点出现在 FIFO 空间不足时：代码会把 `new_len` 裁剪到可写空间。如果裁剪后仍有空间，就推进 `rx_fifo.in` 并把 `last_buf_pos` 更新到当前 DMA 位置；如果已经完全没有空间，则不会更新 `last_buf_pos`。但 DMA 硬件本身不会因为软件 FIFO 满了就停止写入，它可能已经继续覆盖 `rx_buf` 中的旧数据。

因此，在极端满载场景下，可能出现三者不一致：

- DMA 实际已经写到了新的环形位置。
- `last_buf_pos` 仍停留在旧位置。
- 软件 FIFO 逻辑上认为旧数据还没有被读走。

这可能导致后续再次计算 DMA 增量时重复统计某段数据，或者任务侧按 FIFO 计数读取时，读到的内存内容已经被 DMA 后续数据覆盖。表现上可能是 UART 字节流偶发乱包、重复片段或丢片段。

不过这个问题需要 UART 接收速度长期高于任务消费速度，直到 `rx_fifo` 被灌满。当前项目里的 SBUS、WT606、裁判系统等 UART 数据大多是稳定周期流，并且有固定帧头、校验或离线保护，实车正常运行时触发概率较低。因此本文只把它作为低优先级边界风险记录，不单独展开修改方案。
