# 应用初始化全流程

> 从 `Robot_Init()` 开始，按调用顺序导航到 02 层各文件。

## 全流程

```
Robot_Init()                        ← robot/robot_init.c
  │
  ├─ tx_byte_pool_create()          ← 20KB ThreadX 字节内存池
  │
  ├─ UTILS_Init()                   ← ulog 日志初始化
  │
  ├─ BSP_Init()                     ← board/bsp/bsp_init.c (共享硬件抽象层)
  │    ├─ BSP_DWT_Init(168/480)     → [[02_code_twin/board/bsp/DWT/bsp_dwt]]
  │    ├─ cdc_acm_init()            ← USB 虚拟串口
  │    ├─ BSP_LED_Init()            → [[02_code_twin/board/bsp/LED/bsp_led]]
  │    ├─ BSP_BEEP_Init()           → [[02_code_twin/board/bsp/BEEP/bsp_beep]]
  │    └─ BSP_CAN_TaskInit()        → [[02_code_twin/board/bsp/CAN/bsp_can_task]]
  │
  ├─ MODULE_Init()                  ← modules/module_init.c
  │    ├─ Module_Offline_init()     → [[02_code_twin/modules/OFFLINE/module_offline-c]]
  │    ├─ Module_Remote_init()      ← SBUS/DT7 + VT02/VT03
  │    ├─ Module_BMI088_init()      ← IMU
  │    ├─ Module_INS_Init()         ← 姿态解算
  │    ├─ Module_Referee_Init()     ← 裁判系统
  │    ├─ Module_WT606_Init()       ← 陀螺仪
  │    ├─ Module_SuperCap_Init()    ← 超级电容
  │    ├─ Module_Motor_Init()       ← 2ms 循环, 链表调度
  │    ├─ Module_Vision_Init()      ← 视觉通信
  │    └─ Module_BoardComm_Init()   ← 板间通信
  │
  └─ APP_Init()                     ← apps/app_init.c
       └─ robot_control_init()      ← apps/<robot>/single_board/robot_control.c
```

`board/bsp/` 是共享硬件抽象层，`board/dji_c/`（F407）和 `board/damiao_h7/`（H723）是两个板子的 CubeMX 工程。BSP 代码用 `#if defined(STM32F407xx)` / `#if defined(STM32H723xx)` 条件编译区分两套硬件。

## MODULE_Init() 调用顺序

顺序固定，前一步是后一步的前置条件：

| 调用 | 为什么必须在前面 |
|------|-----------------|
| `tx_byte_pool_create()` 最先 | 后续所有 `BSP_MEM_ALLOC_WAIT` 依赖此内存池 |
| `BSP_DWT_Init()` 在 Offline 之前 | Offline 依赖 `BSP_DWT_GetTimeline_ms()` 获取时间戳 |
| `Module_Offline_init()` 第一个 | 后续模块调 `Module_Offline_register()` 依赖 `g_initialized`，链表注册见 [[01_extracted/algorithm/data-structure-linked-list#注册 = 分配 + 填充 + 头插]] |
| `BSP_CAN_TaskInit()` 在 Motor 之前 | 电机初始化调 `BSP_CAN_Device_Init()` 依赖 CAN 总线已启动 |
| `Module_Motor_Init()` 在 APP_Init 之前 | `robot_control_init()` 注册电机时挂入链表，motor task 需已就绪 |

## APP_Init()

`robot_control_init()` 的具体内容取决于 `config.cmake` 中的 `ROBOT` 变量，负责电机注册、遥控映射、控制线程创建。详见各 `apps/<robot>/` 目录。
