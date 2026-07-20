# bsp_can_task.c/h — CAN 收发后台任务

`board/bsp/CAN/bsp_can_task.{c,h}`

## 前置阅读

本文件实现的是 CAN 总线的**后台收发任务**——两条 ThreadX 线程分别负责接收分发和发送排队。但它建立在上层 CAN 设备抽象（设备注册、中断回调、多总线管理）之上，那部分逻辑不在本文件里。

> 如果你还没读过 [[02_code_twin/board/bsp/CAN/bsp_can]]，建议先去看那边，了解 `Can_Device` 结构体、设备注册接口 `can_device_register()` 以及 `CAN_Bus_Manager` 的总线管理。读完后再回来这里看 RX Task 和 TX Task 怎么消费这些数据。

## 一句话

两个 ThreadX 线程，**循环体里唯一做的事就是 `tx_semaphore_get` 阻塞等信号量**，被唤醒后从 kfifo 取数据处理。不主动轮询，不主动定时，纯被动响应。

## 信号量

```c
TX_SEMAPHORE g_can_rx_sem;  // 中断 put → RX Task get
TX_SEMAPHORE g_can_tx_sem;  // 用户 put → TX Task get
```

两个信号量初始计数都是 0。Task 创建后立即阻塞在 `tx_semaphore_get(..., TX_WAIT_FOREVER)` 上，什么都不做，直到有人 put。

- **RX 侧**：CAN 接收中断 put `g_can_rx_sem`，唤醒 RX Task
- **TX 侧**：调用方（电机任务等）通过 `BSP_CAN_Send()` put `g_can_tx_sem`，唤醒 TX Task

信号量计数特性：连续触发多次 put，计数累加，Task 逐个 get 消费。不会丢通知。

## RX Task — 接收数据流

### 中断侧：怎么 put 的

CAN 硬件 FIFO 收到帧后触发 HAL 回调 `HAL_CAN_RxFifo0MsgPendingCallback`，最终调到 `can_f4_rx_callback()`。中断回调只做三件事：**取硬件 FIFO → 打包成 `BSP_CanMsg_t` → kfifo_put 入 rx_fifo**，全部取完后 `tx_semaphore_put` 通知一次：

```c
static void can_f4_rx_callback(CAN_HandleTypeDef *hcan, uint32_t RxFifo)
{
    CAN_Bus_Manager *bus = can_bus_find((void *)hcan, true);
    if (bus == NULL) return;

    bool         has_msg = false;
    BSP_CanMsg_t msg     = {.hcan = NULL};

    while (HAL_CAN_GetRxFifoFillLevel(hcan, RxFifo) > 0)      // 1. 取空硬件 FIFO
    {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t             rx_data[8];
        if (HAL_CAN_GetRxMessage(hcan, RxFifo, &rx_header, rx_data) != HAL_OK) continue;
        msg.id  = rx_header.StdId;                            // 2. 打包成 BSP_CanMsg_t
        msg.len = rx_header.DLC;
        if (msg.len > 8) msg.len = 8;
        memcpy(msg.data, rx_data, msg.len);
        if (kfifo_put(&bus->rx_fifo, &msg)) has_msg = true;   // 3. 入软件 rx_fifo
    }
    if (has_msg) tx_semaphore_put(&g_can_rx_sem);             // 4. 通知 RX Task（只 put 一次）
}
```

关键细节：中断用 `while` 循环取空硬件 FIFO（最多 3 级深度），逐帧 `kfifo_put` 进软件 rx_fifo，但 **`tx_semaphore_put` 只在最后调一次**。这样即使硬件 FIFO 里积了 3 帧，RX Task 也只被唤醒一次，但它内部的 `while (kfifo_get(...))` 会把 3 帧全部取出。

中断为什么能安全地写 kfifo：rx_fifo 是严格 SPSC（中断唯一生产者，RX Task 唯一消费者），无锁安全。详见 [[01_extracted/algorithm/kfifo-design#SPSC 架构]]。

`BSP_CanMsg_t` 结构体定义和 `can_bus_find` 的总线查找逻辑见 [[02_code_twin/board/bsp/CAN/bsp_can]]。

### Task 侧：怎么 get 的

RX Task 被信号量唤醒后，遍历所有总线，逐条 `kfifo_get` 取出消息，通过 `can_dev_find` 查设备表找到对应的 `Can_Device`，调它的 `rx_callback` 把数据分发出去：

```c
while (1)
{
    tx_semaphore_get(&g_can_rx_sem, TX_WAIT_FOREVER);         // 阻塞等中断通知

    for (int i = 0; i < BSP_CAN_BUS_NUM; i++)
    {
        if (!g_can_bus[i].initialized) continue;

        BSP_CanMsg_t msg;
        while (kfifo_get(&g_can_bus[i].rx_fifo, &msg))        // 逐条取
        {
            Can_Device *dev = can_dev_find(&g_can_bus[i], msg.id);
            if (dev != NULL && dev->rx_callback != NULL)
                dev->rx_callback(dev, msg.data, msg.len);     // 分发到设备回调
        }
    }
}
```

`can_dev_find` 按 `msg.id` 在 `devices[8]` 数组里线性查找 `rx_id` 匹配的设备。设备注册和查找表结构见 [[02_code_twin/board/bsp/CAN/bsp_can#设备查找表]]。

`rx_callback` 是函数指针，设备注册时填入自己的解析函数。例如大疆电机注册时填 `dji_can_rx_callback`，回调里把 data 拆成编码器值/转速/电流/温度写入电机 measure，然后更新离线心跳。详见 [[02_code_twin/modules/MOTOR/DJI/motor_dji#CAN 接收回调]]。

### 完整数据流

```
CAN 硬件 FIFO
  ↓  HAL_CAN_GetRxMessage（中断上下文）
BSP_CanMsg_t 打包
  ↓  kfifo_put → rx_fifo
tx_semaphore_put(g_can_rx_sem)
  ↓  信号量唤醒
RX Task: tx_semaphore_get
  ↓  kfifo_get ← rx_fifo
can_dev_find(msg.id) → Can_Device
  ↓
dev->rx_callback(dev, data, len) → 设备解析（电机/遥控器/板间通信...）
```

## TX Task — 发送数据流

### 谁往 tx_fifo 里放的

调用方（电机任务、板间通信、超级电容等）调 `BSP_CAN_Send()` 发送数据。这个函数在调用方的线程上下文执行，做的事和中断类似：**打包成 `BSP_CanMsg_t` → kfifo_put 入 tx_fifo → tx_semaphore_put 通知 TX Task**：

```c
int BSP_CAN_Send(Can_Device *device, const uint8_t *data, uint8_t len)
{
    CAN_Bus_Manager *bus = (CAN_Bus_Manager *)device->_bus;
    BSP_CanMsg_t msg = {.hcan = bus->hcan, .id = device->tx_id, .len = len};
    memcpy(msg.data, data, len);

    /* tx_fifo 满时丢弃最老消息，保证最新数据总能入队 */
    if (!kfifo_put(&bus->tx_fifo, &msg))
    {
        BSP_CanMsg_t dummy;
        kfifo_get(&bus->tx_fifo, &dummy);   // 丢老的
        kfifo_put(&bus->tx_fifo, &msg);     // 塞新的
    }
    tx_semaphore_put(&g_can_tx_sem);         // 通知 TX Task
    return 0;
}
```

`BSP_CAN_Send()` 和 `BSP_CAN_SendMessage()` 的完整签名和参数校验见 [[02_code_twin/board/bsp/CAN/bsp_can#数据流：发送]]。

> **多生产者注意**：tx_fifo 的生产者不唯一——电机任务、板间通信、超级电容等都会调 `BSP_CAN_Send` 往同一条总线的 tx_fifo 写。kfifo 的无锁安全性依赖 SPSC 前提，多生产者并发写可能丢数据。代码中没有加锁保护，当前依赖各任务遵循统一定时调度，从高优先级往低优先级依次运行，避免实际并发。

### Task 侧：怎么 get 的

TX Task 被唤醒后，遍历所有总线，逐条 `kfifo_get` 取出消息，调 HAL 发送：

```c
while (1)
{
    tx_semaphore_get(&g_can_tx_sem, TX_WAIT_FOREVER);         // 阻塞等调用方通知

    for (int i = 0; i < BSP_CAN_BUS_NUM; i++)
    {
        if (!g_can_bus[i].initialized) continue;

        BSP_CanMsg_t msg;
        while (kfifo_get(&g_can_bus[i].tx_fifo, &msg))
        {
            CAN_TxHeaderTypeDef TxHeader = {
                .StdId = msg.id, .DLC = msg.len,
                .RTR = CAN_RTR_DATA, .IDE = CAN_ID_STD
            };
            uint32_t mailbox;
            // 邮箱满则 sleep(1) 等待硬件腾出位置
            while (HAL_CAN_AddTxMessage(msg.hcan, &TxHeader, msg.data, &mailbox) != HAL_OK)
                tx_thread_sleep(1);
        }
    }
}
```

邮箱满时 `tx_thread_sleep(1)` 让出 CPU 等 1 个 tick，硬件发送完一帧后邮箱空出位置再继续。F407 有 3 个发送邮箱，正常控制周期下不会频繁阻塞。

### 完整数据流

```
调用方线程（电机任务/板间通信/...）
  ↓  BSP_CAN_Send(device, data, len)
BSP_CanMsg_t 打包
  ↓  kfifo_put → tx_fifo（满则丢最老）
tx_semaphore_put(g_can_tx_sem)
  ↓  信号量唤醒
TX Task: tx_semaphore_get
  ↓  kfifo_get ← tx_fifo
HAL_CAN_AddTxMessage → 硬件发送邮箱 → CAN 总线
```

## 任务参数

| 参数 | 值 | 说明 |
|------|-----|------|
| RX 优先级 | 3 | 较高，及时处理接收 |
| TX 优先级 | 4 | 略低于 RX |
| 栈大小 | 1024B | `APPS_STACK_SECTION` 放 CCMRAM/DTCM |
| 时间片 | TX_NO_TIME_SLICE | 不抢占同优先级 |

在 `BSP_Init()` 阶段调用 `BSP_CAN_TaskInit()` 创建信号量和线程。
