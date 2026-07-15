# bsp_can.c/h - CAN 硬件抽象层

## 文件位置

`mas_embedded_threadx/board/bsp/CAN/bsp_can.c` / `bsp_can.h`

## 作用

对上提供 `BSP_CAN_Device_Init`（注册设备）和 `BSP_CAN_Send`（发送数据）两个接口，对下封装 STM32 HAL 库的 CAN 配置、中断回调、过滤器设置。内部用 kfifo 缓冲 + 信号量驱动实现异步收发。

---

## 数据结构

### Can_Device — 一个 CAN 设备

```c
struct Can_Device
{
    uint32_t             tx_id;             /* 发送 ID */
    uint32_t             rx_id;             /* 接收 ID */
    uint8_t              filter_bank_index; /* 过滤器索引 */
    BSP_CAN_RxCallback_t rx_callback;       /* 接收回调 */
    void                *user_arg;          /* 用户参数 */
    void                *_bus;              /* 指向所属 CAN_Bus_Manager */
};
```

每个 CAN 设备对应一个物理外设（如一个电机、一个遥控器）。注册时指定 tx_id 和 rx_id，收到匹配 rx_id 的消息时调用 `rx_callback`。

### CAN_Bus_Manager — 一条 CAN 总线

```c
typedef struct
{
    CAN_HandleTypeDef *hcan;
    can_dev_slot_t devices[BSP_CAN_MAX_DEV_PER_BUS];  /* 设备查找表（数组，不是链表） */
    struct kfifo   rx_fifo;                           /* 接收 FIFO */
    BSP_CanMsg_t   rx_fifo_buf[BSP_CAN_RX_FIFO_SIZE]; /* 接收 FIFO 底层缓冲区 */
    struct kfifo   tx_fifo;                           /* 发送 FIFO */
    BSP_CanMsg_t   tx_fifo_buf[BSP_CAN_TX_FIFO_SIZE]; /* 发送 FIFO 底层缓冲区 */
    bool           initialized;
} CAN_Bus_Manager;
```

- `devices` 是数组查找表（旧版用链表，新版改为数组，遍历更简单、内存更确定）
- `rx_fifo` / `tx_fifo` 使用 kfifo 无锁环形缓冲区，见 [[01_extracted/kfifo-design]]
- 全局变量 `g_can_bus[BSP_CAN_BUS_NUM]`（F407 为 2 条总线）

### BSP_CanMsg_t — CAN 消息帧

```c
typedef struct
{
    CAN_HandleTypeDef *hcan;
    uint32_t id;
    uint8_t  data[8];
    uint8_t  len;
} BSP_CanMsg_t;
```

---

## 过滤器配置

```c
static uint8_t g_can1_filter_idx;
static uint8_t g_can2_filter_idx = 14;
```

CAN1 从 bank 0 开始递增，CAN2 从 bank 14 开始递增。28 个 filter bank 平均分配给双 CAN。

```c
static bool can_config_filter(CAN_HandleTypeDef *hcan, uint32_t rx_id, uint8_t *out_bank_index)
{
    uint8_t filter_bank;
    if (hcan->Instance == CAN1)
    {
        if (g_can1_filter_idx >= 14) return false;
        filter_bank = g_can1_filter_idx++;
    }
    else if (hcan->Instance == CAN2)
    {
        if (g_can2_filter_idx >= 28) return false;
        filter_bank = g_can2_filter_idx++;
    }
    else return false;

    CAN_FilterTypeDef cfg = {
        .FilterMode           = CAN_FILTERMODE_IDMASK,
        .FilterScale          = CAN_FILTERSCALE_16BIT,
        .FilterFIFOAssignment = (rx_id & 1) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1,
        .SlaveStartFilterBank = 14,
        .FilterIdLow          = rx_id << 5,
        .FilterMaskIdLow      = 0x7FF << 5,
        .FilterIdHigh         = rx_id << 5,
        .FilterMaskIdHigh     = 0x7FF << 5,
        .FilterBank           = filter_bank,
        .FilterActivation     = CAN_FILTER_ENABLE,
    };
    if (HAL_CAN_ConfigFilter(hcan, &cfg) != HAL_OK)
    {
        if (hcan->Instance == CAN1) g_can1_filter_idx--;
        else g_can2_filter_idx--;
        return false;
    }
    if (out_bank_index) *out_bank_index = filter_bank;
    return true;
}
```

- `FilterIdLow = rx_id << 5`：标准 ID 左移 5 位对齐到寄存器的 STID 位段
- `FilterMaskIdLow = 0x7FF << 5`：11 位全 1 掩码，要求 ID 完全匹配
- `(rx_id & 1)`：奇偶分配 FIFO0/FIFO1，分散压力
- 配置失败时回退 filter_idx，防止编号泄漏

详细的过滤器原理见 [[01_extracted/stm32-can-filter]]。

---

## BSP_CAN_Device_Init — 注册一个 CAN 设备

```c
Can_Device *BSP_CAN_Device_Init(Can_Device_Init_Config_s *config)
```

执行流程：

1. **参数检查**：config、hcan、rx_id/tx_id 范围
2. **查找总线**：`can_bus_find()` 在 `g_can_bus[]` 中找已绑定的总线，或拿一个空槽
3. **首次初始化总线**：如果是新总线，memset 清零、绑定 hcan、初始化 rx_fifo 和 tx_fifo（kfifo_init）、激活中断、启动 CAN
4. **检查 ID 冲突**：同一总线不允许重复 rx_id
5. **分配设备内存**：`BSP_MEM_ALLOC_WAIT`（ThreadX 内存池分配）
6. **配置过滤器**：调用 `can_config_filter()`，给这个设备的 rx_id 配一个硬件过滤器
7. **插入设备查找表**：`can_dev_insert(bus, slot, rx_id, dev)`

返回 `Can_Device *`，后续用这个指针发送数据和接收回调。

---

## BSP_CAN_Send — 发送数据

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
        kfifo_get(&bus->tx_fifo, &dummy);
        kfifo_put(&bus->tx_fifo, &msg);
    }
    tx_semaphore_put(&g_can_tx_sem);
    return 0;
}
```

发送不直接调 HAL，而是把消息塞进 `tx_fifo`，然后释放信号量唤醒 TX Task。TX Task 在 [[02_code_twin/board/bsp/CAN/bsp_can_task]] 中。

TX kfifo 满时**丢弃最老消息**——电机控制场景下，最新的控制指令比旧的更重要。

---

## HAL 接收回调 — 中断上下文

```c
static void can_f4_rx_callback(CAN_HandleTypeDef *hcan, uint32_t RxFifo)
{
    CAN_Bus_Manager *bus = can_bus_find((void *)hcan, true);
    if (bus == NULL) return;

    bool         has_msg = false;
    BSP_CanMsg_t msg     = {.hcan = NULL};

    while (HAL_CAN_GetRxFifoFillLevel(hcan, RxFifo) > 0)
    {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t             rx_data[8];
        if (HAL_CAN_GetRxMessage(hcan, RxFifo, &rx_header, rx_data) != HAL_OK) continue;
        msg.id  = rx_header.StdId;
        msg.len = rx_header.DLC;
        if (msg.len > 8) msg.len = 8;
        memcpy(msg.data, rx_data, msg.len);
        if (kfifo_put(&bus->rx_fifo, &msg)) has_msg = true;
    }
    if (has_msg) tx_semaphore_put(&g_can_rx_sem);
}
```

中断里**一次性排空硬件 FIFO**（while 循环 drain），每条消息 `kfifo_put` 到 `rx_fifo`。排空后只放一次信号量（批量通知），避免频繁唤醒任务。

`HAL_CAN_RxFifo0MsgPendingCallback` 和 `HAL_CAN_RxFifo1MsgPendingCallback` 分别对应 FIFO0 和 FIFO1 的中断入口，都调 `can_f4_rx_callback`。

---

## 数据流总结

```
接收链路:
  硬件 CAN RX → 硬件 FIFO0/1 (3级) → 中断回调 drain → kfifo rx_fifo (8级)
    → 信号量唤醒 RX Task → can_dev_find 匹配 rx_id → 调用 rx_callback

发送链路:
  用户任务 → BSP_CAN_Send → kfifo tx_fifo (8级) → 信号量唤醒 TX Task
    → HAL_CAN_AddTxMessage → 硬件发送邮箱 (3级) → CAN 总线
```

- kfifo 原理见 [[01_extracted/kfifo-design]]
- RX/TX Task 的实现见 [[02_code_twin/board/bsp/CAN/bsp_can_task]]
