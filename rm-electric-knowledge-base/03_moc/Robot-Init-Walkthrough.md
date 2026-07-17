# 应用初始化全流程

> 从 `Robot_Init()` 开始，按调用顺序导航到 02 层各文件。

## 全流程

### 1. Robot_Init()

源文件：`robot/robot_init.c`

| 步骤 | 说明 |
|------|------|
| `tx_byte_pool_create()` | 20KB ThreadX 字节内存池，后续所有 `BSP_MEM_ALLOC_WAIT` 依赖此池 |
| `UTILS_Init()` | ulog 日志初始化 |
| `BSP_Init()` | → 第 2 步 |
| `MODULE_Init()` | → 第 3 步 |
| `APP_Init()` | → 第 4 步，[[02_code_twin/apps/app_init]] |

### 2. BSP_Init()

源文件：`board/bsp/bsp_init.c`（共享硬件抽象层）

`board/bsp/` 是共享硬件抽象层，`board/dji_c/`（F407）和 `board/damiao_h7/`（H723）是两个板子的 CubeMX 工程。BSP 代码用 `#if defined(STM32F407xx)` / `#if defined(STM32H723xx)` 条件编译区分两套硬件。

| 步骤                      | 02 层链接                                      |     |
| ----------------------- | ------------------------------------------- | --- |
| `BSP_DWT_Init(168/480)` | [[02_code_twin/board/bsp/DWT/bsp_dwt]]      |     |
| `cdc_acm_init()`        | USB 虚拟串口                                    |     |
| `BSP_LED_Init()`        | [[02_code_twin/board/bsp/LED/bsp_led]]      |     |
| `BSP_BEEP_Init()`       | [[02_code_twin/board/bsp/BEEP/bsp_beep]]    |     |
| `BSP_CAN_TaskInit()`    | [[02_code_twin/board/bsp/CAN/bsp_can_task]] |     |

### 3. MODULE_Init()

源文件：`modules/module_init.c`

| 步骤                        | 02 层链接                                                                                                                                         |
| ------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| `Module_Offline_init()`   | [[02_code_twin/modules/OFFLINE/module_offline-c]]                                                                                              |
| `Module_Remote_init()`    | [[02_code_twin/modules/REMOTE/module_remote]] — SBUS: [[02_code_twin/modules/REMOTE/SBUS/sbus]] / DT7: [[02_code_twin/modules/REMOTE/DT7/dt7]] |
| `Module_BMI088_init()`    | IMU                                                                                                                                            |
| `Module_INS_Init()`       | 姿态解算                                                                                                                                           |
| `Module_Referee_Init()`   | 裁判系统                                                                                                                                           |
| `Module_WT606_Init()`     | 陀螺仪                                                                                                                                            |
| `Module_SuperCap_Init()`  | 超级电容                                                                                                                                           |
| `Module_Motor_Init()`     | [[02_code_twin/modules/MOTOR/motor_base]] — 2ms 循环，链表调度                                                                                        |
| `Module_Vision_Init()`    | [[02_code_twin/modules/VISION/module_vision]]                                                                                                  |
| `Module_BoardComm_Init()` | 板间通信                                                                                                                                           |

### 4. APP_Init()

源文件：`apps/app_init.c` → [[02_code_twin/apps/app_init]]

`APP_Init()` 本身只有一行 `robot_control_init()`，是一个转发点。`robot_control_init()` 的实现在各兵种各板型目录下，由 CMake 根据 `ROBOT` 和 `BOARD` 变量决定编译哪个版本。

跳转链路：

```
Robot_Init()           robot/robot_init.c         ← 第 1 步，调用 APP_Init()
  └─ APP_Init()        apps/app_init.c            ← 第 4 步，转发
       └─ robot_control_init()
            │  编译时由 config.cmake 的 ROBOT/BOARD 决定链接哪个 .c 文件
            │
            └─ 哨兵云台板: apps/sentry/gimbal_board/robot_control.c
                 → [[02_code_twin/apps/sentry/gimbal_board/robot_control]]
```

当前以哨兵云台板为例：[[02_code_twin/apps/sentry/gimbal_board/robot_control]]

## MODULE_Init() 调用顺序

顺序固定，前一步是后一步的前置条件：

| 调用 | 为什么必须在前面 |
|------|----------------|
| `tx_byte_pool_create()` 最先 | 后续所有 `BSP_MEM_ALLOC_WAIT` 依赖此内存池 |
| `BSP_DWT_Init()` 在 Offline 之前 | Offline 依赖 `BSP_DWT_GetTimeline_ms()` 获取时间戳 |
| `Module_Offline_init()` 第一个 | 后续模块调 `Module_Offline_register()` 依赖 `g_initialized`，链表注册见 [[01_extracted/algorithm/data-structure-linked-list#注册 = 分配 + 填充 + 头插]] |
| `BSP_CAN_TaskInit()` 在 Motor 之前 | 电机初始化调 `BSP_CAN_Device_Init()` 依赖 CAN 总线已启动 |
| `Module_Motor_Init()` 在 APP_Init 之前 | `robot_control_init()` 注册电机时挂入链表，motor task 需已就绪 |
