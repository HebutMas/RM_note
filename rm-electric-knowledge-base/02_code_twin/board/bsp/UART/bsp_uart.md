# bsp_uart.c/h — UART 设备抽象

`board/bsp/UART/bsp_uart.{c,h}`

## 一句话

封装 STM32 HAL 的 UART 收发功能，用静态数组管理设备，支持阻塞/中断/DMA 三种模式，DMA 模式用 kfifo 做环形缓冲区。

## 设备数量

```c
#if defined(STM32H723xx)
#define BSP_UART_DEVICE_NUM 6   // H723 有 6 个串口可用
#elif defined(STM32F407xx)
#define BSP_UART_DEVICE_NUM 3   // F407 有 3 个串口可用
#endif
```

## UART_Device 结构体

```c
typedef struct UART_Device {
    UART_HandleTypeDef  *huart;           // UART 句柄
    struct kfifo         rx_fifo;         // 接收 FIFO（IT/DMA 模式）
    uint8_t             *rx_buf;          // DMA/IT 硬件接收缓冲区
    uint32_t             buf_size;        // 硬件缓冲区大小
    volatile uint32_t    last_buf_pos;    // 上次 DMA 处理位置（仅 DMA 模式）
    uint32_t             expected_rx_len; // 期望帧长，0 = 不定长
    TX_EVENT_FLAGS_GROUP event_flags;     // 事件标志组（RX/TX）
    UART_Mode            rx_mode;         // 接收模式
    UART_Mode            tx_mode;         // 发送模式
    uint8_t              initialed;       // 是否已初始化
} UART_Device;
```

## 设备管理：静态数组（不 malloc）

与 CAN 不同，UART 的数组直接存结构体（不是指针）：

```c
static UART_Device uart_device_table[BSP_UART_DEVICE_NUM];
```

分配时不 malloc，直接用数组中的结构体：

```c
int slot = Find_Free_Slot();  // 遍历找 initialed == 0 的位置
UART_Device *dev = &uart_device_table[slot];  // 取地址，不 malloc
dev->initialed = 1;
```

**与 CAN 的区别**：CAN 的数组存指针（`Can_Device *`），设备从内存池 malloc；UART 的数组直接存结构体，不额外 malloc。因为串口数量更少更固定（3~6 个），结构体不大，直接放数组更简单。两种方式的对比见 [[01_extracted/algorithm/array-vs-linked-list]]。

查找设备时按 `huart` 匹配（CAN 按 `rx_id` 匹配）：

```c
static UART_Device *Find_Device(UART_HandleTypeDef *huart) {
    for (int i = 0; i < BSP_UART_DEVICE_NUM; ++i)
        if (uart_device_table[i].initialed && uart_device_table[i].huart == huart)
            return &uart_device_table[i];
    return NULL;
}
```

HAL 回调里只有 `huart`，通过它反查找到对应的 `UART_Device`。

## 三种模式

| 模式 | 接收 | 发送 | 用途 |
|------|------|------|------|
| `UART_MODE_BLOCKING` | `HAL_UARTEx_ReceiveToIdle` 阻塞 | `HAL_UART_Transmit` 阻塞 | 简单场景，调试用 |
| `UART_MODE_IT` | `HAL_UARTEx_ReceiveToIdle_IT` 中断 | `HAL_UART_Transmit_IT` 中断 | 低速串口 |
| `UART_MODE_DMA` | `HAL_UARTEx_ReceiveToIdle_DMA` DMA | `HAL_UART_Transmit_DMA` DMA | 高速串口（遥控器/图传） |

## 初始化：`BSP_UART_Device_Init()`

```c
UART_Device_init_config config = {
    .huart           = &huart5,
    .rx_buf          = rx_buf,         // DMA 接收缓冲区
    .rx_buf_size     = 64,             // 必须是 2 的幂
    .expected_rx_len = 18,             // DT7 帧长 18 字节
    .rx_mode         = UART_MODE_DMA,
    .tx_mode         = UART_MODE_BLOCKING,
};
dt7_uart = BSP_UART_Device_Init(&config);
```

初始化流程：

1. 参数检查 + 重复注册检查（`Find_Device`）
2. `Find_Free_Slot` 找空槽位
3. `rx_buf_size` 必须是 2 的幂（kfifo 要求，位运算取余）
4. `kfifo_init` 初始化接收 FIFO（IT/DMA 模式）
5. `tx_event_flags_create` 创建事件标志组
6. 启动首次接收（`start_rx_it` 或 `start_rx_dma`）

> **rx_buf_size 必须是 2 的幂**：kfifo 用 `& (size - 1)` 做取余，只有 2 的幂减 1 后才是全 1 掩码。详见 [[01_extracted/algorithm/kfifo-design#位运算取余]]。

## 数据流：接收（DMA 模式）

### 第一步：DMA 硬件接收

`HAL_UARTEx_ReceiveToIdle_DMA` 启动 DMA 接收，硬件自动把串口数据写入 `rx_buf`。DMA 支持空闲中断（IDLE），收到一帧后触发 `HAL_UARTEx_RxEventCallback`。

### 第二步：中断回调更新 kfifo 写索引

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    UART_Device *dev = Find_Device(huart);

    if (dev->rx_mode == UART_MODE_DMA) {
        // DMA 环形缓冲区：通过 NDTR 计算 DMA 当前写入位置
        size_t ndtr     = __HAL_DMA_GET_COUNTER(huart->hdmarx);
        size_t curr_pos = dev->buf_size - ndtr;
        size_t last_pos = dev->last_buf_pos;

        // 计算新增数据长度（处理环形回卷）
        size_t new_len;
        if (curr_pos >= last_pos)
            new_len = curr_pos - last_pos;
        else
            new_len = dev->buf_size - last_pos + curr_pos;

        // 更新 kfifo 写索引
        dev->rx_fifo.in += new_len;
        __DMB();  // 内存屏障，确保 CPU 看到 DMA 写入的数据
        dev->last_buf_pos = curr_pos;

        // 通知等待的读取任务
        tx_event_flags_set(&dev->event_flags, BSP_UART_EVENT_RX, TX_OR);
    }
}
```

**关键点**：

- DMA 模式下 `rx_buf` 同时是 DMA 硬件缓冲区和 kfifo 的数据区，kfifo 的 `in` 索引直接跟踪 DMA 写入位置
- `__HAL_DMA_GET_COUNTER` 读取 DMA 剩余传输计数，反算出 DMA 写到了哪里
- `__DMB()` 内存屏障确保 CPU 读到的数据是 DMA 已写入的（CPU cache 和 DMA 不一致性），详见 [[01_extracted/algorithm/kfifo-design#内存屏障]]
- `curr_pos` 回卷时（DMA 写到缓冲区末尾从头开始）用环形计算处理

### 第三步：读取任务从 kfifo 取数据

```c
int BSP_UART_Read(UART_Device *device, uint8_t *buf, uint32_t buf_size,
                  uint32_t *rx_len, uint32_t timeout) {
    // 等待 RX 事件
    tx_event_flags_get(&device->event_flags, BSP_UART_EVENT_RX,
                       TX_OR_CLEAR, &actual_flags, timeout);

    // 从 kfifo 读出数据
    uint32_t actual = kfifo_out(&device->rx_fifo, buf, to_read);
    *rx_len = actual;
    return actual;
}
```

读取任务阻塞在事件标志组上，中断发事件后唤醒，从 kfifo 取出数据。

**与 CAN 的区别**：CAN 用信号量（计数型，多次中断多次 get）；UART 用事件标志组（位掩码型，RX 和 TX 各占 1 bit）。因为 UART 的 RX 和 TX 事件共用一个 `event_flags`，用位区分。

## 数据流：接收（IT 模式）

IT 模式更简单，HAL 每次从 `rx_buf[0]` 开始写：

```c
if (dev->rx_mode == UART_MODE_IT) {
    dev->rx_fifo.in  = Size;   // 直接设置写索引 = 本次接收长度
    dev->rx_fifo.out = 0;      // 重置读索引
    __DMB();
    tx_event_flags_set(&dev->event_flags, BSP_UART_EVENT_RX, TX_OR);
    start_rx_it(dev);          // 重新启动 IT 接收
}
```

IT 模式每次接收覆盖 `rx_buf`，不是环形的。kfifo 的 `in` 直接设为 `Size`，`out` 清零，读取方一次性取完。

## 数据流：发送

```c
int BSP_UART_Send(UART_Device *device, uint8_t *data, uint32_t len, uint32_t timeout) {
    switch (device->tx_mode) {
    case UART_MODE_BLOCKING:
        HAL_UART_Transmit(device->huart, data, len, tmo);
        return len;
    case UART_MODE_DMA:
        BSP_CACHE_CLEAN(data, len);  // DMA 读前刷新 cache
        HAL_UART_Transmit_DMA(device->huart, data, len);
        break;
    }
    // 等待 TX 完成事件
    tx_event_flags_get(&device->event_flags, BSP_UART_EVENT_TX,
                       TX_OR_CLEAR, &actual_flags, timeout);
    return len;
}
```

DMA 发送前 `BSP_CACHE_CLEAN` 刷新 cache，确保 DMA 读到的是 CPU 最新写入的数据。

## 被谁调用

### 遥控器（SBUS/DT7）

SBUS 和 DT7 遥控器解码任务通过 `BSP_UART_Read` 阻塞等待数据，收到后解析通道值。详见 [[02_code_twin/modules/REMOTE/SBUS/sbus]] 和 [[02_code_twin/modules/REMOTE/DT7/dt7]]。

### 图传（VT02/VT03）

图传通信也通过 UART，使用 DMA 模式。

### 裁判系统

裁判系统通过 UART6 接收，数据量大，使用 DMA 模式。

## 错误处理

```c
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    HAL_UART_Abort_IT(huart);  // 出错时中止并重启接收
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart) {
    UART_Device *dev = Find_Device(huart);
    if (dev->rx_mode == UART_MODE_IT) start_rx_it(dev);
    else if (dev->rx_mode == UART_MODE_DMA) start_rx_dma(dev);
}
```

UART 出错（如帧错误、噪声、溢出）时 HAL 调 `ErrorCallback`，中止当前接收后在 `AbortCpltCallback` 中重新启动。保证接收不会因错误永久停止。
