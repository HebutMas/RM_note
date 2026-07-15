# bsp_can_task.c/h - CAN 收发后台任务

## 文件位置

`mas_embedded_threadx/board/bsp/CAN/bsp_can_task.c` / `bsp_can_task.h`

## 作用

创建两个后台线程：RX Task 和 TX Task。RX Task 从 kfifo 取出接收消息分发给设备回调，TX Task 从 kfifo 取出发送消息调用 HAL 发送。两者都由信号量驱动，无消息时休眠不占 CPU。

必须在 `BSP_CAN_Device_Init` 之前调用 `BSP_CAN_TaskInit`。

---

## 信号量

```c
TX_SEMAPHORE g_can_rx_sem;   // 接收信号量
TX_SEMAPHORE g_can_tx_sem;   // 发送信号量
```

| 信号量 | 生产者 | 消费者 |
|--------|--------|--------|
| `g_can_rx_sem` | HAL 中断回调（ISR 上下文） | CAN RX Task |
| `g_can_tx_sem` | `BSP_CAN_Send` / `BSP_CAN_SendMessage`（任务上下文） | CAN TX Task |

计数信号量：每放一次 sem 计数加 1，每 get 一次减 1。中断里批量 drain 后只放一次（避免频繁唤醒），用户每次发送都放一次。

---

## RX Task — 接收分发

```c
static void can_rx_task_entry(ULONG arg)
{
    while (1)
    {
        tx_semaphore_get(&g_can_rx_sem, TX_WAIT_FOREVER);

        for (int i = 0; i < BSP_CAN_BUS_NUM; i++)
        {
            if (!g_can_bus[i].initialized) continue;

            BSP_CanMsg_t msg;
            while (kfifo_get(&g_can_bus[i].rx_fifo, &msg))
            {
                Can_Device *dev = can_dev_find(&g_can_bus[i], msg.id);
                if (dev != NULL && dev->rx_callback != NULL)
                    dev->rx_callback(dev, msg.data, msg.len);
            }
        }
    }
}
```

执行逻辑：

1. `tx_semaphore_get` 阻塞等待，无消息时休眠
2. 被唤醒后遍历所有 CAN 总线
3. 每条总线用 `kfifo_get` 排空 `rx_fifo`
4. 对每条消息，用 `can_dev_find` 在设备查找表中按 `msg.id` 匹配 `rx_id`
5. 找到设备后调用它的 `rx_callback`

`rx_callback` 在 **RX Task 上下文中执行**，不是中断上下文。所以回调里可以做较重的处理（解析协议、更新状态），但禁止长时间阻塞（会卡住后续消息的分发）。

---

## TX Task — 发送执行

```c
static void can_tx_task_entry(ULONG arg)
{
    while (1)
    {
        tx_semaphore_get(&g_can_tx_sem, TX_WAIT_FOREVER);

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
                while (HAL_CAN_AddTxMessage(msg.hcan, &TxHeader, msg.data, &mailbox) != HAL_OK)
                    tx_thread_sleep(1);
            }
        }
    }
}
```

执行逻辑：

1. `tx_semaphore_get` 阻塞等待，无发送请求时休眠
2. 被唤醒后遍历所有 CAN 总线
3. 每条总线用 `kfifo_get` 排空 `tx_fifo`
4. 对每条消息调用 `HAL_CAN_AddTxMessage` 发送
5. 如果硬件邮箱满（3 个邮箱都被占用），`HAL_CAN_AddTxMessage` 返回非 OK，`tx_thread_sleep(1)` 放弃一个时间片后重试

---

## 任务优先级

```c
#define BSP_CAN_RX_TASK_PRIORITY 3   // RX 优先级更高
#define BSP_CAN_TX_TASK_PRIORITY 4   // TX 优先级较低
```

ThreadX 中数值越小优先级越高。RX Task 优先于 TX Task——接收比发送紧急，丢接收帧意味着丢失反馈数据（电机角度、IMU 等），丢发送帧只是延迟一拍控制指令。

---

## BSP_CAN_TaskInit — 初始化

```c
void BSP_CAN_TaskInit(void)
{
    tx_semaphore_create(&g_can_rx_sem, "can_rx_sem", 0);
    tx_semaphore_create(&g_can_tx_sem, "can_tx_sem", 0);

    tx_thread_create(&g_can_rx_thread, "CAN RX Task", can_rx_task_entry, 0,
                     g_can_rx_stack, BSP_CAN_TASK_STACK_SIZE,
                     BSP_CAN_RX_TASK_PRIORITY, BSP_CAN_RX_TASK_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&g_can_tx_thread, "CAN TX Task", can_tx_task_entry, 0,
                     g_can_tx_stack, BSP_CAN_TASK_STACK_SIZE,
                     BSP_CAN_TX_TASK_PRIORITY, BSP_CAN_TX_TASK_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START);
}
```

在 `tx_application_define()`（ThreadX 初始化函数）中调用。创建两个信号量（初始计数 0）和两个线程（自动启动）。

- `TX_NO_TIME_SLICE`：不使用时间片轮转，任务由信号量驱动
- `TX_AUTO_START`：线程创建后自动开始运行
- 栈大小 1024 字节，放在 `APPS_STACK_SECTION` 指定的内存段中

---

## 完整收发时序

```
接收:
  1. CAN 总线收到帧 → 硬件 FIFO0/1（3级深度）
  2. 硬件触发中断 → HAL_CAN_RxFifo0MsgPendingCallback
  3. ISR: while drain 硬件 FIFO → kfifo_put 到 rx_fifo（8级深度）
  4. ISR: tx_semaphore_put(&g_can_rx_sem)
  5. RX Task 被唤醒: kfifo_get → can_dev_find → rx_callback

发送:
  1. 用户任务: BSP_CAN_Send → kfifo_put 到 tx_fifo（8级深度）
  2. 用户任务: tx_semaphore_put(&g_can_tx_sem)
  3. TX Task 被唤醒: kfifo_get → HAL_CAN_AddTxMessage
  4. 如果硬件邮箱满: tx_thread_sleep(1) 重试
  5. 硬件发送邮箱（3级深度）→ CAN 总线
```

三层缓冲：硬件 FIFO（3级）→ kfifo（8级）→ 回调/发送。中断只负责把数据从硬件搬到 kfifo，不做协议解析；任务负责分发和发送。中断执行时间极短，保证实时性。
