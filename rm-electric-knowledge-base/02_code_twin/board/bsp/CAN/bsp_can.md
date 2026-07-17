# bsp_can.c/h — CAN 设备抽象与总线管理

`board/bsp/CAN/bsp_can.{c,h}` + `bsp_can_task.{c,h}`

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

每条总线用 `can_dev_slot_t devices[8]` 数组做查找表，按 `rx_id` 线性查找。数组存 `rx_id` + `Can_Device *` 指针，设备结构体从内存池 malloc 后填入空槽位。这种"数组槽位 + 指针"的分配方式与 Motor/Offline 模块的链表方式不同，详见 [[01_extracted/algorithm/array-vs-linked-list]]。

### 总线容量分析

1 Mbps 下的标准数据帧（11bit ID + 8 字节数据）位长：

| 段 | 位数 |
|----|------|
| SOF + 仲裁段 + 控制段 | 19 |
| 数据段（8 字节） | 64 |
| CRC + ACK + EOF + 帧间隔 | 28 |
| **不含填充合计** | **111 bit** |

CAN 位填充规则：连续 5 个相同电平 bit 后插入 1 个反相 bit。固定格式段不超过 5 个连续同电平，真正触发填充的是 64 bit 数据段。电机数据（编码器值、电流、温度）不会全 0 或全 1，典型填充 3~5 次，帧长约 114~116 bit。最坏情况（数据全 0/全 1）帧长 127 bit。

| 帧长 | 最大帧率 |
|------|---------|
| 111 bit（无填充） | 9009 帧/s |
| 115 bit（典型） | 8696 帧/s |
| 127 bit（最坏） | 7874 帧/s |

**实际场景**（6 个电机，2ms 控制周期）：

| 来源                                | 帧率           |
| --------------------------------- | ------------ |
| 1个控制帧4 个 M3508 发送（共用 0x200/0x1FF） | 500 帧/s      |
| 4 个 M3508 反馈（1kHz）                | 4000 帧/s     |
| 1 个 GM6020 反馈                     | 1000 帧/s     |
| 1 个 M2006 反馈                      | 1000 帧/s     |
| 2 个 控制帧（6020+2006）                | 1000 帧/s     |
| **合计**                            | **7500 帧/s** |

7500 帧/s × 115 bit ≈ 862,500 bit/s，总线负载率约 86%。实际跑得动，因为电机反馈是周期性的（非突发），CAN 硬件仲裁保证高优先级帧不丢，且实际帧长不会都是最坏情况。


## BSP_CanMsg_t — CAN 消息结构体

kfifo 里存的不是裸字节，而是一个个完整的 `BSP_CanMsg_t` 结构体：

```c
typedef struct {
    CAN_HandleTypeDef *hcan;   // 哪条 CAN 总线
    uint32_t           id;     // 帧 ID（标准帧 11bit）
    uint8_t            data[8]; // 数据载荷（最多 8 字节）
    uint8_t            len;    // 数据长度（DLC）
} BSP_CanMsg_t;
```

**为什么需要这个结构体**：硬件中断只给你一个 `CAN_RxHeaderTypeDef` 和 8 字节 data，但下游消费者（RX Task）需要知道"这是哪条总线来的""帧 ID 是多少"才能查表分发。`BSP_CanMsg_t` 把这些信息打包成一个消息，一次性入队/出队。

kfifo 初始化时指定元素大小为 `sizeof(BSP_CanMsg_t)`：

```c
kfifo_init(&bus->rx_fifo, bus->rx_fifo_buf, BSP_CAN_RX_FIFO_SIZE, sizeof(BSP_CanMsg_t));
```

每次 `kfifo_put` / `kfifo_get` 操作的是一个完整的 `BSP_CanMsg_t`，不是单个字节。kfifo 的位运算取余保证不会读到半个消息。详见 [[01_extracted/algorithm/kfifo-design#SPSC 架构]]。

## Can_Device — 设备抽象

每个挂在 CAN 总线上的设备（电机、遥控器、板间通信等）注册一个 `Can_Device`：

```c
typedef struct {
    CAN_HandleTypeDef *hcan;       // 总线句柄
    uint32_t           tx_id;      // 发送帧 ID
    uint32_t           rx_id;      // 接收帧 ID（用于查找表匹配）
    void (*rx_callback)(Can_Device *dev, const uint8_t *data, uint8_t len);  // 接收回调
    void  *user_arg;               // 调用方上下文（如电机指针）
    CAN_Bus_Manager  *_bus;        // 所属总线管理器
} Can_Device;
```

`rx_callback` 是函数指针：设备注册时填入自己的解析函数。CAN RX Task 收到数据后通过这个回调把数据交给设备处理。函数指针原理见 [[01_extracted/algorithm/function-pointer-pattern]]。

`user_arg` 是回调上下文：注册时存入（比如电机指针），回调时取回，这样回调函数知道该把数据写进哪个实例。大疆电机注册示例见 [[02_code_twin/modules/MOTOR/DJI/motor_dji#CAN 接收回调]]。

## 设备查找表

CAN 模块用**数组槽位 + 指针**管理设备。数组 `devices[8]` 是静态分配的，每个槽位存 `rx_id` + `Can_Device *` 指针：

```c
typedef struct {
    uint32_t    rx_id;  // 接收帧 ID，用于匹配
    Can_Device *dev;    // 设备指针，NULL = 空闲槽位
} can_dev_slot_t;
```

查找时遍历数组比 `rx_id`：

```c
static inline Can_Device *can_dev_find(CAN_Bus_Manager *bus, uint32_t rx_id) {
    for (int i = 0; i < BSP_CAN_MAX_DEV_PER_BUS; i++) {
        if (bus->devices[i].rx_id == rx_id) return bus->devices[i].dev;
    }
    return NULL;
}
```

分配时空槽位管理 + 内存池 malloc：

```c
static inline int can_dev_find_empty(CAN_Bus_Manager *bus) {
    for (int i = 0; i < BSP_CAN_MAX_DEV_PER_BUS; i++) {
        if (bus->devices[i].dev == NULL) return i;  // 找空槽位
    }
    return -1;
}
// BSP_MEM_ALLOC_WAIT(dev, sizeof(Can_Device), ...);  // 从内存池分配
// can_dev_insert(bus, slot, rx_id, dev);             // 填入槽位
```

数组连续访问比链表指针跳转更 cache 友好，设备少（≤8）时实际查找更快。与 Motor/Offline 模块链表方式的对比见 [[01_extracted/algorithm/array-vs-linked-list]]。

## 初始化：`BSP_CAN_TaskInit()`

在 `BSP_Init()` 阶段调用。创建两个信号量 + 两个线程：

| 线程 | 优先级 | 职责 |
|------|--------|------|
| CAN RX Task | 3 | 从 rx_fifo 取消息，分发给设备回调 |
| CAN TX Task | 4 | 从 tx_fifo 取消息，调 HAL 发送 |

必须在 `BSP_CAN_Device_Init()` 之前调用，因为设备注册时需要总线已初始化（kfifo + 中断已启动）。

详见 [[02_code_twin/board/bsp/CAN/bsp_can_task]]。

## 初始化：`BSP_CAN_Device_Init()`

设备注册流程：

1. `can_bus_find()` — 找到对应总线，首次使用时初始化 kfifo + 启动 HAL CAN
2. ID 冲突检查 — `can_dev_find()` 查重
3. `BSP_MEM_ALLOC_WAIT` — 从内存池分配 `Can_Device`
4. `can_config_filter()` — 配置硬件过滤器（精确匹配 `rx_id`），原理见 [[01_extracted/hardware/can-filter]]
5. `can_dev_insert()` — 插入查找表
6. 返回 `Can_Device *`，调用方拿到后设置 `rx_callback` 和 `user_arg`

内存分配和注册模式见 [[01_extracted/algorithm/data-structure-linked-list#注册 = 分配 + 填充 + 头插]]。

## 数据流：接收

### 第一步：中断收到 CAN 帧

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    can_f4_rx_callback(hcan, CAN_RX_FIFO0);
}

static void can_f4_rx_callback(CAN_HandleTypeDef *hcan, uint32_t RxFifo) {
    while (HAL_CAN_GetRxFifoFillLevel(hcan, RxFifo) > 0) {
        HAL_CAN_GetRxMessage(hcan, RxFifo, &rx_header, rx_data);
        msg.id  = rx_header.StdId;
        msg.len = rx_header.DLC;
        memcpy(msg.data, rx_data, msg.len);
        kfifo_put(&bus->rx_fifo, &msg);    // 消息入队
    }
    tx_semaphore_put(&g_can_rx_sem);        // 通知 RX Task
}
```

中断只做三件事：取硬件 FIFO → 打包成 `BSP_CanMsg_t` → 入软件 rx_fifo + 发信号量。while 循环排空硬件 FIFO（最多 3 级），全部入队后发一次信号量。

**为什么不直接在中断里解析数据**：中断要尽量短。解析数据（拆字节、写电机 measure、更新心跳）是重活，放到任务里做。中断只负责搬运。

rx_fifo 是严格 SPSC：中断（唯一生产者）和 RX Task（唯一消费者），kfifo 无锁安全。

### 第二步：RX Task 取出消息并分发

```c
static void can_rx_task_entry(ULONG arg) {
    while (1) {
        tx_semaphore_get(&g_can_rx_sem, TX_WAIT_FOREVER);  // 阻塞等信号量

        for (int i = 0; i < BSP_CAN_BUS_NUM; i++) {
            if (!g_can_bus[i].initialized) continue;

            BSP_CanMsg_t msg;
            while (kfifo_get(&g_can_bus[i].rx_fifo, &msg)) {     // 逐条取出
                Can_Device *dev = can_dev_find(&g_can_bus[i], msg.id);  // 查表
                if (dev != NULL && dev->rx_callback != NULL)
                    dev->rx_callback(dev, msg.data, msg.len);     // 回调分发
            }
        }
    }
}
```

RX Task 阻塞在信号量上。中断发信号量后唤醒，遍历所有总线的 rx_fifo，逐条 `kfifo_get` 取出 `BSP_CanMsg_t`：

1. `msg.id` → `can_dev_find()` 查找表找到对应设备
2. `msg.data` 和 `msg.len` → 传给设备的 `rx_callback`
3. `msg.hcan` → RX Task 不需要（已知是哪条总线），但 TX Task 发送时需要知道用哪个 hcan

**回调链路**：`rx_callback(dev, msg.data, msg.len)` → 比如大疆电机的 `dji_can_rx_callback`，把 data 拆成编码器值/转速/电流/温度写入电机 measure，然后更新离线心跳。详见 [[02_code_twin/modules/MOTOR/DJI/motor_dji#CAN 接收回调]]。

## 数据流：发送

### `BSP_CAN_Send()` / `BSP_CAN_SendMessage()`

调用方（电机任务、板间通信等）不直接调 HAL 发送，而是把消息塞进 tx_fifo：

```c
BSP_CanMsg_t msg = {.hcan = bus->hcan, .id = device->tx_id, .len = len};
memcpy(msg.data, data, len);
kfifo_put(&bus->tx_fifo, &msg);      // 入队
tx_semaphore_put(&g_can_tx_sem);     // 通知 TX Task
```

`BSP_CAN_SendMessage` 同理，用于电机等需要自定义完整 CAN 帧的场景。

tx_fifo 满时丢弃最老消息（`kfifo_get` 取出丢弃 + `kfifo_put` 写入新的），保证最新数据总能入队。

### TX Task 统一发送

```c
static void can_tx_task_entry(ULONG arg) {
    while (1) {
        tx_semaphore_get(&g_can_tx_sem, TX_WAIT_FOREVER);

        for (int i = 0; i < BSP_CAN_BUS_NUM; i++) {
            if (!g_can_bus[i].initialized) continue;

            BSP_CanMsg_t msg;
            while (kfifo_get(&g_can_bus[i].tx_fifo, &msg)) {
                CAN_TxHeaderTypeDef TxHeader = {
                    .StdId = msg.id, .DLC = msg.len,
                    .RTR = CAN_RTR_DATA, .IDE = CAN_ID_STD
                };
                uint32_t mailbox;
                while (HAL_CAN_AddTxMessage(msg.hcan, &TxHeader, msg.data, &mailbox) != HAL_OK)
                    tx_thread_sleep(1);  // 邮箱满等 1 tick
            }
        }
    }
}
```

TX Task 从 tx_fifo 取出 `BSP_CanMsg_t`，用 `msg.hcan` 确定总线、`msg.id` 填帧头、`msg.data` 填数据，调 HAL 发送。3 个硬件邮箱满时 `tx_thread_sleep(1)` 等待（非阻塞，让出 CPU）。

## 发送端多生产者问题

> **注意**：tx_fifo 的生产者不唯一。电机任务、板间通信、超级电容等都会调 `BSP_CAN_Send` / `BSP_CAN_SendMessage` 往同一条总线的 tx_fifo 写。kfifo 的无锁安全性依赖 SPSC 前提，多生产者并发写可能丢数据。代码中没有加锁保护，当前依赖各任务的触发都是统一遵循定时调度, 调度后从高优先级往低优先级依次运行的时序, 避免实际并发

## F407 过滤器配置

每个设备注册时分配一个过滤器 Bank，精确匹配该设备的 `rx_id`。配置原理和寄存器映射详见 [[01_extracted/hardware/can-filter]]。

## 信号量 vs 事件组

CAN 用两个全局信号量（`g_can_rx_sem` / `g_can_tx_sem`），不是事件组。因为 RX/TX Task 各只有一个，信号量的计数特性正好匹配：多次中断多次 put，Task 逐个 get。
