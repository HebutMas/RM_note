# bsp_can.c/h — CAN 设备抽象与总线管理

`board/bsp/CAN/bsp_can.{c,h}` + `bsp_can_task.{c,h}`

## 一句话

每条 CAN 总线一个 `CAN_Bus_Manager`，管理设备查找表 + 收发 kfifo。中断把数据塞进 rx_fifo 并发信号量，RX Task 取出后按 ID 查表分发给设备回调。TX Task 同理从 tx_fifo 取消息发送。

## 硬件背景

F407 bxCAN 硬件资源（参考手册第 24 章，第 607 页起）：详见 [[01_extracted/hardware/can-filter]]，或直接打开 [[00_raw/hardware/STM32F407中文手册(完全版) 高清完整.pdf#page=607|手册 P.607]]。

- 双 CAN（CAN1 主 / CAN2 从），共享 512 字节 SRAM
- 比特率高达 1 Mb/s，本项目配置：APB1=42MHz, Prescaler=3, BS1=10TQ, BS2=3TQ → 42M/(3×14) = **1 Mbps**
- 3 个发送邮箱（硬件管理优先级）
- 2 个接收 FIFO，各 3 级深度
- 28 个过滤器组（CAN1: 0-13, CAN2: 14-27）

### 软件设备挂载限制

```c
#define BSP_CAN_BUS_NUM        2    // F407 双 CAN
#define BSP_CAN_MAX_DEV_PER_BUS 8   // 每条总线最多 8 个软件设备
#define BSP_CAN_RX_FIFO_SIZE    8   // 软件 rx_fifo 容量
#define BSP_CAN_TX_FIFO_SIZE    8   // 软件 tx_fifo 容量
```

每条总线用 `can_dev_slot_t devices[8]` 数组做查找表，按 `rx_id` 线性查找。8 个设备够用（电机 4-8 台 + 超电 + 板间通信）。

## 架构：TX/RX 任务 + 设备回调

```
中断: HAL_CAN_RxFifo0MsgPendingCallback
  → can_f4_rx_callback()
    → kfifo_put(&bus->rx_fifo, &msg)     ← 数据入队
    → tx_semaphore_put(&g_can_rx_sem)    ← 通知 RX Task

RX Task (阻塞在信号量上):
  → tx_semaphore_get(&g_can_rx_sem)
  → kfifo_get(&bus->rx_fifo, &msg)
  → can_dev_find(bus, msg.id)            ← 查找表 O(n)
  → dev->rx_callback(dev, msg.data, len) ← 调用设备回调

TX Task (阻塞在信号量上):
  → tx_semaphore_get(&g_can_tx_sem)
  → kfifo_get(&bus->tx_fifo, &msg)
  → HAL_CAN_AddTxMessage(msg.hcan, &txheader, msg.data, &mailbox)
    邮箱满则 tx_thread_sleep(1) 等待
```

中断只做 kfifo 入队 + 发信号量（极快），数据处理在任务里完成。

rx_fifo 是严格 SPSC：中断（唯一生产者）和 RX Task（唯一消费者）。kfifo 无锁安全，详见 [[01_extracted/algorithm/kfifo-design#SPSC 架构]]。

## 发送端多生产者问题

> **注意**：tx_fifo 的生产者不唯一。电机任务、板间通信、超级电容等都会调 `BSP_CAN_Send` / `BSP_CAN_SendMessage` 往同一条总线的 tx_fifo 写。kfifo 的无锁安全性依赖 SPSC 前提，多生产者并发写可能丢数据。代码中没有加锁保护，当前依赖各任务的调度时序避免实际并发，但这是一个潜在隐患。

## 设备查找表（替代链表）

CAN 模块用**数组查找表**而非链表。每条总线最多 8 个设备，用 `rx_id` 做键线性查找。设备少时数组比链表更 cache 友好。链表模式见 [[01_extracted/algorithm/data-structure-linked-list]]（Offline/Motor 模块用链表）。

## F407 过滤器配置

每个设备注册时分配一个过滤器 Bank，精确匹配该设备的 `rx_id`。配置原理和寄存器映射详见 [[01_extracted/hardware/can-filter]]。

## 初始化：`BSP_CAN_Device_Init()`

```
1. 参数检查 + ID 范围检查 (0~2047, 即 11 位标准 ID)
2. can_bus_find() — 找到对应总线
   首次使用时初始化: memset + kfifo_init(rx_fifo) + kfifo_init(tx_fifo)
   + HAL_CAN_ActivateNotification(两个 FIFO 的中断) + HAL_CAN_Start
3. ID 冲突检查 — can_dev_find() 查重
4. BSP_MEM_ALLOC_WAIT — 从内存池分配 Can_Device
5. can_config_filter() — 配置硬件过滤器
6. can_dev_insert() — 插入查找表
```

内存分配的 NULL 检查和链表注册模式见 [[01_extracted/algorithm/data-structure-linked-list#注册 = 分配 + 填充 + 头插]]。

## 发送：`BSP_CAN_Send()` / `BSP_CAN_SendMessage()`

```c
// BSP_CAN_Send: 通过设备指针发送
BSP_CanMsg_t msg = {.hcan = bus->hcan, .id = device->tx_id, .len = len};
memcpy(msg.data, data, len);
kfifo_put(&bus->tx_fifo, &msg);     // 入队
tx_semaphore_put(&g_can_tx_sem);    // 通知 TX Task

// BSP_CAN_SendMessage: 直接传消息结构体（用于电机等需要自定义 CAN 帧的场景）
// 同样入 tx_fifo + 发信号量
```

不直接调 HAL 发送，而是入队后让 TX Task 统一发。调用方不阻塞。

tx_fifo 满时丢弃最老消息（`kfifo_get` 取出丢弃 + `kfifo_put` 写入新的），保证最新数据总能入队。

## 接收中断回调

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    can_f4_rx_callback(hcan, CAN_RX_FIFO0);
}

static void can_f4_rx_callback(CAN_HandleTypeDef *hcan, uint32_t RxFifo) {
    while (HAL_CAN_GetRxFifoFillLevel(hcan, RxFifo) > 0) {
        HAL_CAN_GetRxMessage(hcan, RxFifo, &rx_header, rx_data);
        msg.id = rx_header.StdId;
        msg.len = rx_header.DLC;
        memcpy(msg.data, rx_data, msg.len);
        kfifo_put(&bus->rx_fifo, &msg);  // 入队
    }
    tx_semaphore_put(&g_can_rx_sem);      // 通知 RX Task
}
```

while 循环排空硬件 FIFO（最多 3 级），全部入软件 rx_fifo，然后发一次信号量。两个 FIFO（FIFO0/FIFO1）共用同一个回调逻辑。

## 信号量 vs 事件组

CAN 用两个全局信号量（`g_can_rx_sem` / `g_can_tx_sem`），不是事件组。因为 RX/TX Task 各只有一个，信号量的计数特性正好匹配：多次中断多次 put，Task 逐个 get。

## 任务初始化：`BSP_CAN_TaskInit()`

创建两个信号量 + 两个线程（RX 优先级 3，TX 优先级 4），必须在 `BSP_CAN_Device_Init()` 之前调用。见 [[03_moc/Robot-Init-Walkthrough#全流程]] 中 `BSP_CAN_TaskInit` 的位置。
