# BSP 与模块教程

## 目录

1. [架构概览](#1-架构概览)
2. [BSP 层详解](#2-bsp-层详解)
3. [模块层详解](#3-模块层详解)
4. [模块开发流程（6步法）](#4-模块开发流程6步法)
5. [各模块深度分析](#5-各模块深度分析)
6. [算法库详解](#6-算法库详解)
7. [跨平台适配指南](#7-跨平台适配指南)

---

## 1. 架构概览

### 1.1 分层架构

```
┌──────────────────────────────────────────────┐
│  APP 层  (apps/sentry/gimbal_board/)          │
│  机器人业务逻辑：控制循环、模式切换             │
├──────────────────────────────────────────────┤
│  MODULE 层  (modules/)                        │
│  功能模块：传感器、电机、通信、算法              │
├──────────────────────────────────────────────┤
│  BSP 层  (board/bsp/)                         │
│  硬件抽象：CAN、UART、SPI、I2C、PWM、GPIO      │
├──────────────────────────────────────────────┤
│  HAL + RTOS                                    │
│  STM32 HAL Drivers + ThreadX + CherryUSB       │
└──────────────────────────────────────────────┘
```

### 1.2 初始化顺序

```
Robot_Init()
├── 1. UTILS_Init()       # 日志系统（ulog）
├── 2. BSP_Init()         # 硬件抽象层初始化
│   ├── DWT（微秒延时）
│   ├── USB CDC ACM（虚拟串口）
│   ├── LED + BEEP
│   └── CAN 任务（收发线程）
├── 3. MODULE_Init()      # 模块初始化（条件编译）
│   ├── OFFLINE → REMOTE → BMI088 → INS
│   ├── REFEREE → WT606 → SuperCap
│   └── MOTOR → VISION → BOARDCOMM
└── 4. APP_Init()         # 应用层初始化
    └── robot_control_init()  # 创建主控制任务
```

---

## 2. BSP 层详解

### 2.1 BSP 层设计原则

- **统一接口**：所有外设通过相同的 API 模式操作
- **平台无关**：上层代码不依赖具体的 MCU 型号
- **RTOS 集成**：所有阻塞操作使用 ThreadX 同步原语
- **ISR 友好**：关键路径可在中断上下文中安全调用

### 2.2 DWT —— 高精度计时器

**文件**：`board/bsp/DWT/bsp_dwt.c`

DWT（Data Watchpoint and Trace）是 Cortex-M 内核内置的硬件循环计数器。

```c
// 初始化（传入 CPU 频率 MHz）
BSP_DWT_Init(480);  // 480 MHz → H7
BSP_DWT_Init(168);  // 168 MHz → F4

// 微秒级延时（中断安全）
BSP_DWT_Delay(0.1);    // 延时 0.1 ms

// 增量时间测量（用于 PID 控制循环）
float dt = BSP_DWT_GetDeltaT(&last_time);  // 返回秒

// 绝对时间
float t_s  = BSP_DWT_GetTimeline_s();
float t_ms = BSP_DWT_GetTimeline_ms();
float t_us = BSP_DWT_GetTimeline_us();

// 性能测量宏
BSP_DWT_MEASURE_START();
// ... 被测代码 ...
BSP_DWT_MEASURE_END("my_function");  // 自动打印耗时
```

**内部机制**：DWT 使用 64 位变量跟踪溢出，避免 32 位计数器在 480 MHz 下约 8.9 秒就会溢出的问题。

### 2.3 CAN —— 双任务管道架构

**文件**：`board/bsp/CAN/bsp_can.c`, `bsp_can_task.c`

CAN 驱动是本项目最复杂的 BSP 驱动，采用 **双任务管道** 模型。

#### 架构图

```
┌─────────────────────────────────────────────────┐
│  CAN_Bus_Manager (每个 CAN 外设)                   │
│  ├── HAL handle (hfdcan / hcan)                  │
│  ├── Can_Device 槽位表 (最多 8 个设备)              │
│  ├── tx_fifo (kfifo)  ← 发送队列                   │
│  └── rx_fifo (kfifo)  ← 接收队列                   │
├─────────────────────────────────────────────────┤
│  Can_Device (每个逻辑 CAN ID)                      │
│  ├── tx_id / rx_id   (CAN 标识符)                  │
│  ├── rx_callback     (接收回调函数)                 │
│  └── owner           (所属模块)                    │
└─────────────────────────────────────────────────┘
```

#### 设备初始化

```c
BSP_Can_Init_Config_s config = {
    .can_handle = BSP_CAN_HANDLE2,
    .tx_id = 0x200,
    .rx_id = 0x1FF,
    .rx_callback = my_can_callback,
};

Can_Device *dev;
BSP_CAN_Device_Init(&dev, &config);
```

#### 发送

```c
BSP_CanMsg_t msg = {
    .id = dev->tx_id,
    .len = 8,
    .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08},
};
BSP_CAN_Send(dev, &msg);
```

**FIFO 满策略**：发送 FIFO 满时丢弃**最旧**消息，保证最新数据优先发送（对实时控制至关重要）。

#### 跨平台支持

```c
#if defined(STM32H723xx)
    // FDCAN API（H7）
    FDCAN_TxHeaderTypeDef TxHeader = { ... };
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &TxHeader, data);
#elif defined(STM32F407xx)
    // 经典 CAN API（F4）
    CAN_TxHeaderTypeDef TxHeader = { ... };
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &tx_mailbox);
#endif
```

### 2.4 UART —— DMA + 空闲中断

**文件**：`board/bsp/UART/bsp_uart.c`

#### 三种工作模式

| 模式 | API 前缀 | 适用场景 |
|------|---------|---------|
| 阻塞 | `BSP_UART_Transmit` | 调试输出、简单交互 |
| 中断 (IT) | `BSP_UART_Transmit_IT` | 小数据量、低延迟 |
| DMA | `BSP_UART_Transmit_DMA` | 大量数据传输 |

#### DMA 接收机制

使用 HAL 的 `HAL_UARTEx_ReceiveToIdle_DMA`，结合空闲线检测：

```
UART RX 线空闲超过 1 字节时间
  → HAL_UARTEx_RxEventCallback(huart, received_size)
    → kfifo_put(data, size)           ← 数据入队
    → BSP_CACHE_INVALIDATE(buf, size)  ← H7 需要清理缓存
    → tx_event_flags_set(RX_EVENT)     ← 通知等待任务
```

#### 事件标志同步模式

```c
// 发送端
BSP_UART_Transmit_DMA(&dev, data, len);
tx_event_flags_get(&dev->event_flags, BSP_UART_EVENT_TX,
                   TX_OR_CLEAR, &actual, timeout);

// 接收端：在 HAL 回调中自动设置 BSP_UART_EVENT_RX
```

### 2.5 SPI —— 总线互斥 + 级联操作

**文件**：`board/bsp/SPI/bsp_spi.c`

#### 设备管理

```
SPI_Bus (每个 SPI 外设)
├── tx_mutex (优先级继承)
├── event_flags (传输完成)
└── device_list (链表)
    ├── Device 0 (BMI088 加速度计)
    ├── Device 1 (BMI088 陀螺仪)
    └── ...
```

#### 传输接口

```c
// 基本操作（四种模式）
BSP_SPI_Transmit(bus, data, len, timeout);      // 仅发送
BSP_SPI_Receive(bus, data, len, timeout);        // 仅接收
BSP_SPI_TransmitReceive(bus, tx, rx, len, ...);  // 全双工

// 级联操作（两次传输之间不释放 CS）
BSP_SPI_TransAndTrans(bus, data1, len1, data2, len2, timeout);
BSP_SPI_ReceAndRece(bus, rx1, len1, rx2, len2, timeout);
```

级联操作对于需要"写寄存器地址 → 读数据"的传感器（如 BMI088）至关重要。

#### ISR 安全性

```c
if (tx_thread_identify() != NULL) {
    tx_mutex_get(&bus->bus_mutex, TX_WAIT_FOREVER);  // 仅任务上下文
}
spi_cs_low(dev);     // 拉低片选
transfer_once(...);  // 实际传输（阻塞/IT/DMA）
spi_cs_high(dev);    // 拉高片选
if (tx_thread_identify() != NULL) {
    tx_mutex_put(&bus->bus_mutex);
}
```

### 2.6 I2C —— 与 SPI 相同的模式

**文件**：`board/bsp/I2C/bsp_i2c.c`

I2C 驱动与 SPI 几乎相同的架构：
- 总线互斥量（`TX_INHERIT`）
- 事件标志同步
- 设备链表管理
- 四种操作统一到 `transfer_once()` 中
- 支持内存地址读写（寄存器操作）

```c
// 内存读写（带寄存器地址）
BSP_I2C_MemWrite(bus, dev_addr, mem_addr, mem_addr_size, data, len, timeout);
BSP_I2C_MemRead(bus, dev_addr, mem_addr, mem_addr_size, data, len, timeout);
```

### 2.7 PWM —— 简易输出控制

**文件**：`board/bsp/PWM/bsp_pwm.c`

```c
BSP_PWM_Start(dev);     // 启动 PWM 输出
BSP_PWM_Stop(dev);      // 停止 PWM 输出
BSP_PWM_Set_Dutyx10(dev, 500);  // 50.0% 占空比（500 = 50.0% × 10）
```

`dutyx10` 表示万分之一占空比（0–1000），内部通过 `PWM_Convert_Dutyx10_To_Pulse()` 转换为定时器脉冲值。

### 2.8 GPIO —— 外部中断管理

```c
// 注册回调
BSP_GPIO_RegisterCallback(GPIO_PIN_0, my_callback);

// HAL 中断处理自动分派
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    BSP_GPIO_HandleEXTI(GPIO_Pin);
}
```

### 2.9 FLASH —— 双平台完全不同的实现

| 平台 | 存储介质 | 容量 | 接口 |
|------|---------|------|------|
| STM32F407 | 内部 Flash (扇区 11) | 128 KB | 内部总线 |
| STM32H723 | 外部 W25Q64 NOR Flash | 8 MB | OctoSPI (QSPI) |

```c
BSP_FLASH_Write_Buffer(addr, data, len);
BSP_FLASH_Read_Buffer(addr, data, len);
```

上层代码无需关心底层是内部还是外部 Flash。

### 2.10 LED —— 双平台不同驱动方式

| 平台 | 实现方式 | 硬件 |
|------|---------|------|
| H7 | WS2812 智能 LED (SPI6) | 全彩可编程 |
| F4 | 普通 RGB LED (PWM × 3) | TIM5 CH1/CH2/CH3 |

```c
BSP_LED_SetColor(0xFF, 0x00, 0x00, 0xFF);  // 红色，全亮
```

### 2.11 USB CDC ACM —— 虚拟串口

**文件**：`board/bsp/USB/usbd_cdc_acm_user.c`

基于 CherryUSB 协议栈实现虚拟 COM 端口。

```c
// 发送数据（阻塞，带超时）
uint32_t cdc_acm_send(uint8_t busid, const uint8_t *data, uint32_t len, uint32_t timeout);

// 接收数据（阻塞）
uint32_t cdc_acm_recv(uint8_t busid, uint8_t *data, uint32_t len, uint32_t timeout);
```

**关键实现细节**：
- H7 上使用 USB OTG HS（高速，批量端点 512 字节）
- F4 上使用 USB OTG FS（全速，批量端点 64 字节）
- 接收使用 kfifo + DWC2 的 4 字节对齐缓存管理

### 2.12 Cache 一致性处理（H7 专有）

Cortex-M7 有数据和指令缓存，DMA 传输需要手动维护：

```c
// bsp_def.h
#if defined(STM32H723xx)
    #define BSP_IS_DMA_INACCESSIBLE(addr)  /* DTCM/ITCM 地址不能用 DMA */
    #define BSP_CACHE_CLEAN(addr, len)     /* TX DMA 前：写回脏数据 */
    #define BSP_CACHE_INVALIDATE(addr, len) /* RX DMA 后：使缓存无效 */

    // 内存放置
    #define APPS_STACK_SECTION  __attribute__((section(".dtcmram"))) // 栈：DTCM（不可 DMA）
    #define BUFFER_SECTION      __attribute__((section(".RAM_D1")))  // DMA 缓冲：D1 AXI SRAM
#else
    // F4 无缓存，所有宏定义为空
    #define BSP_CACHE_CLEAN(addr, len)
    #define BSP_CACHE_INVALIDATE(addr, len)
    #define APPS_STACK_SECTION  __attribute__((section(".ccmram")))  // CCM 零等待 RAM
    #define BUFFER_SECTION      __attribute__((section(".ram")))
#endif
```

---

## 3. 模块层详解

### 3.1 模块设计模式

每个模块遵循统一的"初始化 → 获取数据 → 后台任务"模式：

```c
// 模块头文件模式
#ifndef MODULE_XXX_H
#define MODULE_XXX_H

// 1. 默认参数（可被 CMake 覆盖）
#ifndef XXX_TASK_STACK_SIZE
#define XXX_TASK_STACK_SIZE 1024
#endif
#ifndef XXX_TASK_PRIORITY
#define XXX_TASK_PRIORITY 10
#endif

// 2. 初始化函数
void Module_XXX_Init(void);

// 3. 数据获取接口
const XXX_Data_t *Module_XXX_Get(void);

#endif
```

### 3.2 注册/发现模式（MOTOR 模块）

MOTOR 模块使用 **链表注册模式**：

```c
// 基类
typedef struct {
    Motor_Type_e motor_type;
    void *hal_handle;
    // ... 公共字段
} Motor_Base;

// 电机通过链表注册
void Motor_Register(Motor_Base *motor);

// 统一更新所有电机
void Motor_UpdateAll(void);
```

### 3.3 观察者模式（离线检测）

OFFLINE 模块使用 **观察者模式**。设备注册超时时间，离线检测线程定时检查：

```c
// 注册一个需要监控的设备
Offline_Device_Register(&dev, timeout_ms, beep_pattern);

// 设备定期"签到"
Offline_Device_Heartbeat(&dev);

// 检测线程（100 Hz）检查所有设备是否超时
// 超时则触发 LED 闪烁 / 蜂鸣器报警
```

---

## 4. 模块开发流程（6步法）

参考 `modules/MODULES.MD`，以下是添加新模块的完整流程。

### 步骤 1：创建模块代码

```
modules/
└── NEW_MODULE/
    ├── module_new_module.h
    └── module_new_module.c
```

**命名约定**：
- 目录名：全大写（如 `BMI088`, `NEW_MODULE`）
- 文件名：`module_<小写>.h` 和 `module_<小写>.c`
- 初始化函数：`Module_<CamelCase>_Init()`
- CMake 宏：`MODULE_<大写>`

```c
// module_new_module.h
#ifndef MODULE_NEW_MODULE_H
#define MODULE_NEW_MODULE_H

#include "tx_api.h"

// 默认参数（CMake 可覆盖）
#ifndef NEW_MODULE_TASK_STACK_SIZE
#define NEW_MODULE_TASK_STACK_SIZE 1024
#endif
#ifndef NEW_MODULE_TASK_PRIORITY
#define NEW_MODULE_TASK_PRIORITY 10
#endif

void Module_NewModule_Init(void);

#endif
```

```c
// module_new_module.c
#include "module_new_module.h"
#include "bsp_def.h"

#define LOG_LVL LOG_LVL_INFO
#define LOG_TAG "NewModule"
#include "ulog_def.h"

// 后台任务栈和线程
static TX_THREAD new_module_thread;
APPS_STACK_SECTION static uint8_t new_module_stack[NEW_MODULE_TASK_STACK_SIZE];

static void new_module_task(ULONG input)
{
    while (1) {
        // 模块业务逻辑
        tx_thread_sleep(10);  // 100 Hz
    }
}

void Module_NewModule_Init(void)
{
    tx_thread_create(&new_module_thread, "NewModule",
        new_module_task, 0,
        new_module_stack, NEW_MODULE_TASK_STACK_SIZE,
        NEW_MODULE_TASK_PRIORITY, NEW_MODULE_TASK_PRIORITY,
        TX_NO_TIME_SLICE, TX_AUTO_START);
    LOG_I("New Module Init OK");
}
```

### 步骤 2：注册 CMake 模块

在 `modules/CMakeLists.txt` 中添加：

```cmake
if(MODULE_NEW_MODULE)
    list(APPEND _module_sources
        NEW_MODULE/module_new_module.c
    )
    list(APPEND _module_includes
        ${CMAKE_CURRENT_LIST_DIR}/NEW_MODULE
    )
endif()
```

### 步骤 3：添加条件初始化

在 `modules/module_init.c` 中添加：

```c
#if MODULE_NEW_MODULE
#include "module_new_module.h"
#endif

// 在 MODULE_Init() 函数中：
#if MODULE_NEW_MODULE
    Module_NewModule_Init();
#endif
```

### 步骤 4：注册默认参数

在 `modules/module_config.cmake` 中添加：

```cmake
# NEW_MODULE 默认参数
set(NEW_MODULE_TASK_STACK_SIZE 1024)
set(NEW_MODULE_TASK_PRIORITY   10)
set(NEW_MODULE_OFFLINE_ENABLE  1)
```

### 步骤 5：添加宏导出

在 `apps/generate_headers.cmake` 中添加参数导出：

```cmake
_gen_cmakedefine(NEW_MODULE_TASK_STACK_SIZE)
_gen_cmakedefine(NEW_MODULE_TASK_PRIORITY)
_gen_cmakedefine(NEW_MODULE_OFFLINE_ENABLE)
```

### 步骤 6：在机器人配置中使能

在 `apps/<robot>/robot.cmake` 中将模块加入相应板型的模块列表：

```cmake
set(MODULES_GIMBAL  OFFLINE REMOTE BMI088 INS MOTOR VISION BOARDCOMM NEW_MODULE)
set(MODULES_CHASSIS OFFLINE BMI088 INS REFEREE MOTOR BOARDCOMM NEW_MODULE)
```

---

## 5. 各模块深度分析

### 5.1 INS —— 惯性导航系统

**文件**：`modules/INS/`

核心是**四元数扩展卡尔曼滤波器 (QuaternionEKF)**：

```c
// 获取姿态数据
const Ins_t *ins = Module_INS_get();

// Ins_t 结构体包含：
// - q[4]：四元数 [w, x, y, z]
// - euler_rad[3]：欧拉角（roll, pitch, yaw）, 单位弧度
// - accel[3]：加速度计读数
// - gyro[3]：陀螺仪读数
// - YawTotalAngle_rad：yaw 轴累积角度
```

**数据流**：
```
BMI088 (SPI) → INS Task (1000 Hz)
  ├── 读取加速度计 + 陀螺仪
  ├── QuaternionEKF 更新
  └── 更新 Ins_t 结构体（供其他模块读取）
```

1000 Hz 的运行频率保证了姿态估计的高带宽，适合快速运动的云台控制。

### 5.2 BMI088 —— IMU 传感器

**文件**：`modules/BMI088/`

```c
// 初始化
Module_BMI088_init();

// 读取数据（在 INS 任务中调用）
BMI088_Read_Accel(accel_data);  // ±24g 量程
BMI088_Read_Gyro(gyro_data);    // ±2000dps 量程
```

**温度控制**：可选的温度 PID 控制（通过 `BMI088_TEMP_ENABLE` 启用），通过 PWM 驱动加热电阻保持传感器温度恒定（减小温漂）。

**陀螺仪标定**：上电时自动采集静止数据，计算零偏并保存到 Flash。

### 5.3 MOTOR —— 统一电机控制

**文件**：`modules/MOTOR/`

#### 支持的电机关型

| 类型 | 接口 | 控制模式 | 适用场景 |
|------|------|---------|---------|
| DJI (M3508, M2006, GM6020) | CAN | 电流/速度/位置 | 驱动轮、云台、拨弹 |
| DAMIAO (DM4310, DM4340) | CAN | MIT 模式 | 高性能云台 |
| Servo | PWM | 角度 | 辅助机构 |
| ZDT (步进) | PWM + DIR | 位置/速度 | 精确位置控制 |

#### 控制架构

```
Motor_Base（虚基类）
├── motor_dji      ← CAN 电机
├── motor_damiao   ← 达妙 CAN 电机
├── motor_servo    ← PWM 舵机
└── motor_zdt      ← 步进电机

Motor_UpdateAll() → 遍历链表 → 每个电机执行 control + send
```

#### 电机的 LQR 控制流

```
参考值 (ref)  ──→ [LQR 计算] ──→ 扭矩输出 ──→ CAN 发送
                    ↑                  │
            状态反馈 (角度+角速度)         ↓
                    │              扭矩→电流转换
              编码器+陀螺仪           │
                                 限幅 + CAN 命令发送
```

#### 前馈支持

```c
motor->base.controller.feedforward_torque = 0.1f;  // Nm
```

前馈扭矩直接加到控制输出上，适合补偿已知扰动（如重力力矩）。

### 5.4 VISION —— 视觉通信

**文件**：`modules/VISION/`

通过 USB CDC ACM 虚拟串口与视觉计算机通信：

```c
// 发送（向视觉计算机发送机器人状态）
Module_Vision_Send(&send_packet, TX_NO_WAIT);

// 接收（获取目标检测结果）
ReceivePacket *pkt = Module_Vision_Receive();
// pkt 包含：目标位置、置信度等
```

**数据帧格式**：
- 帧头：`0xBB` / `0x5B`
- 包含四元数姿态、模式等信息
- 接收任务在 USB RX 事件上持续等待

### 5.5 BOARDCOMM —— 板间通信

**文件**：`modules/BOARDCOMM/`

```
云台板                           底盘板
┌─────────┐   CAN (8字节)   ┌─────────┐
│ Gimbal  │ ──────────────→ │ Chassis │
│ Board   │  vx, vy, wz,    │ Board   │
│         │  offset_angle,  │         │
│         │  chassis_mode   │         │
│         │ ←────────────── │         │
│         │  robot_color,   │         │
│         │  game_progress, │         │
│         │  hp, bullets,   │         │
│         │  shooter_heat   │         │
└─────────┘                 └─────────┘
```

使用 `#pragma pack(1)` 确保结构体恰好 8 字节：

```c
// apps/sentry/sentry_def.h
#pragma pack(1)
typedef struct {
    int8_t  vx, vy, wz;
    int16_t offset_angle;
    uint8_t chassis_mode;
    int8_t  reserved[2];
} GimbalToChassis_cmd_t;
_Static_assert(sizeof(GimbalToChassis_cmd_t) == 8, "...");
#pragma pack()
```

### 5.6 REFEREE —— 裁判系统

**文件**：`modules/REFEREE/`

解析 RoboMaster 官方裁判系统协议：
- 比赛阶段、血量、弹药剩余、枪管热量
- 通过 UART 接收，提供 `Module_Referee_Get()` 接口

### 5.7 REMOTE —— 遥控器接收

**文件**：`modules/REMOTE/`

支持的接收机协议：
- **SBUS**（默认）：DJI 遥控器标准协议
- **DT7**：DJI DT7 遥控器
- **VT02/VT03**：图传遥控数据

两个独立的接收线程：遥控数据（高优先级）和图传数据。

---

## 6. 算法库详解

### 6.1 PID 控制器

**文件**：`modules/algorithm/pid.c`

位置式 PID，支持 8 种可选改进：

```c
typedef enum {
    PID_IMPROVEMENT_NONE             = 0,
    PID_IMPROVEMENT_INTEGRAL_LIMIT   = (1 << 0),  // 积分限幅
    PID_IMPROVEMENT_DIFF_FIRST       = (1 << 1),  // 微分先行
    PID_IMPROVEMENT_TRAPEZOID_INT    = (1 << 2),  // 梯形积分
    PID_IMPROVEMENT_PROPORTION_FIRST = (1 << 3),  // 比例先行
    PID_IMPROVEMENT_OUTPUT_FILTER    = (1 << 4),  // 输出滤波
    PID_IMPROVEMENT_VARIABLE_INT     = (1 << 5),  // 变速积分
    PID_IMPROVEMENT_DIFF_FILTER      = (1 << 6),  // 微分滤波
    PID_IMPROVEMENT_TRAVERSE_DETECT  = (1 << 7),  // 堵转检测
} PID_Improvement_e;

// 使用示例
PidInstance pid;
PidInitConfig config = {
    .kp = 10.0f, .ki = 0.5f, .kd = 0.1f,
    .max_out = 100.0f, .integral_limit = 50.0f,
    .improve = PID_IMPROVEMENT_INTEGRAL_LIMIT | PID_IMPROVEMENT_OUTPUT_FILTER,
};
PID_Init(&pid, &config);

float output = PID_Calculate(&pid, setpoint, measurement);
```

**关键特点**：
- 使用 DWT 自动计算 `dt`，无需手动传入
- 写时复制初始化（`memcpy` 后清零运行时状态）
- 支持零交叉积分分离

### 6.2 LQR 控制器

**文件**：`modules/algorithm/lqr.c`

```c
// 初始化
LQR_Init_Config_s lqr_cfg = {
    .K = {5.47f, 0.56f},  // 增益矩阵
    .state_dim = 2,       // 2 = 角度 + 角速度
};
LQRInstance lqr;
LQRInit(&lqr, &lqr_cfg);

// 计算输出
float torque = LQRCalculate(&lqr, angle_rad, angular_velocity_radps, ref_rad);
// 公式：torque = -K[0]*(angle - ref) - K[1]*velocity
```

详细调参方法参见 `lqr.py使用文档.md`。

### 6.3 user_lib —— 通用工具

```c
// 斜坡发生器（平稳过渡）
float RampCalc(RampInstance *ramp, float target, float rate);

// 一阶低通滤波器
float LowPassFilter_Update(LowPassFilter *lpf, float input);

// 自定义快速 sqrt
float Sqrt(float x);

// 单位转换
float DEGREE_2_RAD(float deg);
float RAD_2_DEGREE(float rad);
```

### 6.4 CRC —— RoboMaster 协议 CRC

```c
// RM 协议的 CRC8 和 CRC16
uint8_t CRC8_Calculate(uint8_t *data, uint32_t len);
uint16_t CRC16_Calculate(uint8_t *data, uint32_t len);
```

### 6.5 chassis_type —— 底盘运动学

```c
// 舵轮逆运动学
void Chassis_Swerve_Calc(Chassis_State *state, Wheel_Output *wheels);
// 输入：vx, vy, wz（机器人坐标系速度）
// 输出：每个轮子的驱动速度 + 转向角度
```

---

## 7. 跨平台适配指南

### 7.1 添加新 MCU 平台的步骤

1. 在 `board/` 下创建新板级目录（如 `board/new_board/`）
2. 复制并修改 `CMakeLists.txt`：设置正确的 `THREADX_ARCH`、C 宏、启动文件
3. 创建工具链文件 `cmake/gcc-arm-none-eabi.cmake`
4. 创建 CubeMX 集成（或手动编写 HAL 初始化代码）
5. 在 `bsp_def.h` 中添加新平台的 `#elif defined(NEW_MCU)` 块
6. 在 BSP 各驱动中处理新平台的特性差异

### 7.2 本项目中两个平台的差异点

| 差异点 | H7 处理 | F4 处理 |
|--------|---------|---------|
| CAN | FDCAN | 经典 CAN |
| USB | OTG HS (480 Mbps) | OTG FS (12 Mbps) |
| SPI | 通用 SPI | 通用 SPI（无差异） |
| Flash | 外部 QSPI W25Q64 | 内部 Flash 扇区 |
| LED | WS2812 (SPI6) | RGB PWM |
| 电源 | 需要使能 24V/5V | 无需额外电源控制 |
| Cache | 需要手动维护 | 无缓存 |
| 线程栈 | DTCM (128 KB) | CCMRAM (64 KB) |
| DMA 缓冲 | RAM_D1 (320 KB) | 通用 RAM (128 KB) |

### 7.3 条件编译模式

```c
#if defined(STM32H723xx)
    // H7 特定代码
#elif defined(STM32F407xx)
    // F4 特定代码
#else
    #error "Unsupported MCU"
#endif
```

### 7.4 内存布局注意事项

- **DMA 缓冲区不能放在 DTCM/ITCM 中**（H7）或 CCMRAM 中（F4）
- 使用 `BUFFER_SECTION` 宏将 DMA 缓冲放在 DMA 可访问的 RAM 区域
- 线程栈使用 `APPS_STACK_SECTION` 放在零等待 RAM 中
- H7 的 DTCM 有 128 KB，F4 的 CCMRAM 只有 64 KB，注意栈总量不超过限制

### 7.5 移植 checklist

- [ ] 工具链文件设置正确的 `-mcpu` / `-mfpu` / `-mfloat-abi`
- [ ] 链接器脚本描述正确的内存布局
- [ ] `tx_initialize_low_level.S` 中 CPU 频率正确
- [ ] `bsp_dwt.c` 中 DWT 频率参数正确
- [ ] 所有外设的 HAL 头文件路径正确
- [ ] CubeMX `.ioc` 或等效初始化代码就位
- [ ] 串口/CAN/SPI/I2C/PWM 引脚配置正确
- [ ] Flash 驱动选择正确的介质类型
- [ ] LED/蜂鸣器驱动适配硬件
- [ ] 在 `bsp_init.c` 中处理平台特定初始化
