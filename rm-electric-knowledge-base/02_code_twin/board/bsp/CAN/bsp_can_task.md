# bsp_can_task.c/h — CAN 收发后台任务

`board/bsp/CAN/bsp_can_task.{c,h}`

## 一句话

两个 ThreadX 线程（RX Task + TX Task），各自阻塞在信号量上，被中断唤醒后从 kfifo 取数据处理。

## 信号量

```c
TX_SEMAPHORE g_can_rx_sem;  // 中断 put → RX Task get
TX_SEMAPHORE g_can_tx_sem;  // 用户 put → TX Task get
```

信号量计数特性：中断连续触发多次，semaphore 计数累加，Task 逐个消费。不会丢通知。

## RX Task

```c
while (1) {
    tx_semaphore_get(&g_can_rx_sem, TX_WAIT_FOREVER);  // 阻塞等通知

    for (每条总线) {
        while (kfifo_get(&bus->rx_fifo, &msg)) {       // 逐条取
            Can_Device *dev = can_dev_find(bus, msg.id);
            if (dev && dev->rx_callback)
                dev->rx_callback(dev, msg.data, len);  // 分发到设备回调
        }
    }
}
```

kfifo 的 SPSC 保证此处的线程安全：中断写、RX Task 读，详见 [[01_extracted/algorithm/kfifo-design#SPSC 架构]]。

## TX Task

```c
while (1) {
    tx_semaphore_get(&g_can_tx_sem, TX_WAIT_FOREVER);

    for (每条总线) {
        while (kfifo_get(&bus->tx_fifo, &msg)) {
            HAL_CAN_AddTxMessage(msg.hcan, &txheader, msg.data, &mailbox);
            // 邮箱满则 sleep(1) 等待
        }
    }
}
```

## 任务参数

| 参数 | 值 | 说明 |
|------|-----|------|
| RX 优先级 | 3 | 较高，及时处理接收 |
| TX 优先级 | 4 | 略低于 RX |
| 栈大小 | 1024B | `APPS_STACK_SECTION` 放 CCMRAM/DTCM |
| 时间片 | TX_NO_TIME_SLICE | 不抢占同优先级 |

初始化在 [[03_moc/Robot-Init-Walkthrough#全流程]] 的 `BSP_Init()` 阶段调用。
