# bsp_can.c/h — CAN 设备抽象与总线管理

`board/bsp/CAN/bsp_can.{c,h}`

## 硬件背景

F407 bxCAN 硬件资源（参考手册第 24 章，第 607 页起）：详见 [[01_extracted/hardware/can-filter]]，或直接打开 [[00_raw/hardware/STM32F407中文手册(完全版) 高清完整.pdf#page=607|手册 P.607]]。

- 双 CAN（CAN1 主 / CAN2 从），共享 512 字节 SRAM
- 比特率高达 1 Mb/s，本项目配置：APB1=42MHz, Prescaler=3, BS1=10TQ, BS2=3TQ → 42M/(3×14) = **1 Mbps**
- 3 个发送邮箱（硬件管理优先级）
- 2 个接收 FIFO，各 3 级深度
- 28 个过滤器组（CAN1: 0-13, CAN2: 14-27）

### 软件设备挂载限制

```c
#define BSP_CAN_BUS_NUM        2    // F407 双 CAN (H723: 3)
#define BSP_CAN_MAX_DEV_PER_BUS 8   // 每条总线最多 8 个软件设备
#define BSP_CAN_RX_FIFO_SIZE    8   // 软件 rx_fifo 容量
#define BSP_CAN_TX_FIFO_SIZE    8   // 软件 tx_fifo 容量
```

## CAN_Bus_Manager — 总线管理器

每条 CAN 总线对应一个 `CAN_Bus_Manager`，全局静态数组 `g_can_bus[BSP_CAN_BUS_NUM]`：

```c
typedef struct {
    CAN_HandleTypeDef *hcan;                                    // 硬件句柄
    can_dev_slot_t     devices[BSP_CAN_MAX_DEV_PER_BUS];        // 设备查找表
    struct kfifo       rx_fifo;                                 // 接收 FIFO
    BSP_CanMsg_t       rx_fifo_buf[BSP_CAN_RX_FIFO_SIZE];       // 接收 FIFO 缓冲区
    struct kfifo       tx_fifo;                                 // 发送 FIFO
    BSP_CanMsg_t       tx_fifo_buf[BSP_CAN_TX_FIFO_SIZE];       // 发送 FIFO 缓冲区
    bool               initialized;                             // 是否初始化
} CAN_Bus_Manager;
```

`CAN_Bus_Manager` 是一条 CAN 总线的完整软件抽象，管理三样东西：

| 管理对象 | 字段 | 说明 |
|---------|------|------|
| 硬件 | `hcan` | 指向 `hcan1` / `hcan2`，区分物理总线 |
| 设备表 | `devices[8]` | 数组槽位，按 `rx_id` 查找设备，见下方 Can_Device |
| 数据通道 | `rx_fifo` / `tx_fifo` | kfifo 软件缓冲，收发各一套，元素是 `BSP_CanMsg_t` |

`rx_fifo` 是严格 SPSC：中断（唯一生产者）put，RX Task（唯一消费者）get。`tx_fifo` 的生产者不唯一，依赖定时调度避免并发。kfifo 原理见 [[01_extracted/algorithm/kfifo-design#SPSC 架构]]。

### 设备查找表

`devices[8]` 是数组槽位 + 指针的管理方式：

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
```

数组连续访问比链表指针跳转更 cache 友好，设备少（≤8）时实际查找更快。与 Motor/Offline 模块链表方式的对比见 [[01_extracted/algorithm/array-vs-linked-list]]。

## Can_Device — 设备抽象

每个挂在 CAN 总线上的设备（电机、遥控器、板间通信等）注册一个 `Can_Device`，从内存池 malloc 后填入 Bus Manager 的空槽位：

```c
struct Can_Device {
    uint32_t             tx_id;             // 发送帧 ID
    uint32_t             rx_id;             // 接收帧 ID（用于查找表匹配）
    uint8_t              filter_bank_index; // 硬件过滤器索引
    BSP_CAN_RxCallback_t rx_callback;       // 接收回调（函数指针）
    void                *user_arg;          // 用户参数（如电机指针）
    void                *_bus;              // 指向所属 CAN_Bus_Manager
};
```

`rx_callback` 是函数指针：设备注册时填入自己的解析函数。CAN RX Task 收到数据后通过这个回调把数据交给设备处理。函数指针原理见 [[01_extracted/algorithm/function-pointer-pattern]]。

`user_arg` 是回调上下文：注册时存入（比如电机指针），回调时取回，这样回调函数知道该把数据写进哪个实例。大疆电机注册示例见 [[02_code_twin/modules/MOTOR/DJI/motor_dji#CAN 接收回调]]。

`_bus` 反向指向所属的 `CAN_Bus_Manager`，这样 `BSP_CAN_Send()` 只需要 `Can_Device *` 就能找到总线，不用额外传 hcan。

## BSP_CanMsg_t — CAN 消息结构体

kfifo 里存的不是裸字节，而是一个个完整的 `BSP_CanMsg_t`：

```c
typedef struct {
    CAN_HandleTypeDef *hcan;    // 哪条 CAN 总线
    uint32_t           id;      // 帧 ID（标准帧 11bit）
    uint8_t            data[8]; // 数据载荷（最多 8 字节）
    uint8_t            len;     // 数据长度（DLC）
} BSP_CanMsg_t;
```

**为什么需要这个结构体**：硬件中断只给你一个 `CAN_RxHeaderTypeDef` 和 8 字节 data，但下游消费者（RX Task）需要知道"这是哪条总线来的""帧 ID 是多少"才能查表分发。`BSP_CanMsg_t` 把这些信息打包成一个消息，一次性入队/出队。

kfifo 初始化时指定元素大小为 `sizeof(BSP_CanMsg_t)`：

```c
kfifo_init(&bus->rx_fifo, bus->rx_fifo_buf, BSP_CAN_RX_FIFO_SIZE, sizeof(BSP_CanMsg_t));
```

每次 `kfifo_put` / `kfifo_get` 操作的是一个完整的 `BSP_CanMsg_t`，不是单个字节。kfifo 的位运算取余保证不会读到半个消息。

## 初始化：`BSP_CAN_TaskInit()`

在 `BSP_Init()` 阶段调用。创建两个信号量 + 两个线程（RX Task 优先级 3，TX Task 优先级 4），负责从 kfifo 取消息并分发/发送。

必须在 `BSP_CAN_Device_Init()` 之前调用，因为设备注册时需要总线已初始化（kfifo + 中断已启动）。

任务详情见 [[02_code_twin/board/bsp/CAN/bsp_can_task]]。

## 初始化：`BSP_CAN_Device_Init()`

设备注册流程：

1. `can_bus_find()` — 遍历 `g_can_bus[]` 找到对应 hcan 的 Bus Manager；首次使用时初始化 kfifo + 启动 HAL CAN
2. ID 冲突检查 — `can_dev_find()` 查重
3. `can_dev_find_empty()` — 找空槽位
4. `BSP_MEM_ALLOC_WAIT` — 从内存池分配 `Can_Device`
5. `can_config_filter()` — 配置硬件过滤器（精确匹配 `rx_id`），原理见 [[01_extracted/hardware/can-filter]]
6. `can_dev_insert()` — 插入查找表
7. 返回 `Can_Device *`，调用方拿到后设置 `rx_callback` 和 `user_arg`

内存分配和注册模式见 [[01_extracted/algorithm/data-structure-linked-list#注册 = 分配 + 填充 + 头插]]。

## 数据流：接收

中断收到 CAN 帧 → 打包成 `BSP_CanMsg_t` → `kfifo_put` 入 rx_fifo → `tx_semaphore_put` 通知 RX Task → RX Task `kfifo_get` 取出 → `can_dev_find` 查表 → `rx_callback` 分发到设备。

中断只做三件事：取硬件 FIFO → 打包 → 入软件 rx_fifo + 发信号量。解析数据（拆字节、写电机 measure、更新心跳）放到 RX Task 里做。详见 [[02_code_twin/board/bsp/CAN/bsp_can_task]]。

回调链路示例：`rx_callback(dev, msg.data, msg.len)` → 大疆电机的 `dji_can_rx_callback`，把 data 拆成编码器值/转速/电流/温度写入电机 measure，然后更新离线心跳。详见 [[02_code_twin/modules/MOTOR/DJI/motor_dji#CAN 接收回调]]。

## 数据流：发送

调用方（电机任务、板间通信等）调 `BSP_CAN_Send()` / `BSP_CAN_SendMessage()` → 打包成 `BSP_CanMsg_t` → `kfifo_put` 入 tx_fifo → `tx_semaphore_put` 通知 TX Task → TX Task `kfifo_get` 取出 → 调 HAL 发送。

tx_fifo 满时丢弃最老消息，保证最新数据总能入队。TX Task 邮箱满时 `tx_thread_sleep(1)` 等待。详见 [[02_code_twin/board/bsp/CAN/bsp_can_task]]。

## 发送端多生产者问题

> **注意**：tx_fifo 的生产者不唯一。电机任务、板间通信、超级电容等都会调 `BSP_CAN_Send` / `BSP_CAN_SendMessage` 往同一条总线的 tx_fifo 写。kfifo 的无锁安全性依赖 SPSC 前提，多生产者并发写可能丢数据。代码中没有加锁保护，当前依赖各任务的触发都是统一遵循定时调度，调度后从高优先级往低优先级依次运行的时序，避免实际并发。

## F407 过滤器配置

每个设备注册时分配一个过滤器 Bank，精确匹配该设备的 `rx_id`。配置原理和寄存器映射详见 [[01_extracted/hardware/can-filter]]。

## 信号量 vs 事件组

CAN 用两个全局信号量（`g_can_rx_sem` / `g_can_tx_sem`），不是事件组。因为 RX/TX Task 各只有一个，信号量的计数特性正好匹配：多次中断多次 put，Task 逐个 get。

## 总线容量分析

1 Mbps 下的标准数据帧（11bit ID + 8 字节数据）位长：

| 段                     | 位数          |
| --------------------- | ----------- |
| SOF + 仲裁段 + 控制段       | 19          |
| 数据段（8 字节）             | 64          |
| CRC + ACK + EOF + 帧间隔 | 28          |
| **不含填充合计**            | **111 bit** |

CAN 位填充规则：连续 5 个相同电平 bit 后插入 1 个反相 bit。固定格式段不超过 5 个连续同电平，真正触发填充的是 64 bit 数据段。电机数据（编码器值、电流、温度）不会全 0 或全 1，典型填充 3~5 次，帧长约 114~116 bit。最坏情况（数据全 0/全 1）帧长 127 bit。

| 帧长           | 最大帧率     |
| ------------ | -------- |
| 111 bit（无填充） | 9009 帧/s |
| 115 bit（典型）  | 8696 帧/s |
| 127 bit（最坏）  | 7874 帧/s |

**实际场景**（6 个电机，2ms 控制周期）：

| 来源 | 帧率 |
|------|------|
| 4 个 M3508 发送（共用 0x200/0x1FF） | 500 帧/s |
| 4 个 M3508 反馈（1kHz） | 4000 帧/s |
| 1 个 GM6020 反馈 | 1000 帧/s |
| 1 个 M2006 反馈 | 1000 帧/s |
| 2 个控制帧（6020+2006） | 1000 帧/s |
| **合计** | **7500 帧/s** |

7500 帧/s × 115 bit ≈ 862,500 bit/s，总线负载率约 86%。实际跑得动，因为电机反馈是周期性的（非突发），CAN 硬件仲裁保证高优先级帧不丢，且实际帧长不会都是最坏情况。
