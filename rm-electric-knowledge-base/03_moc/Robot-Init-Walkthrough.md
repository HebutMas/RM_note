# 应用初始化全流程

> 从 `main()` 到 `Robot_Init()`，按调用顺序导航到 02 层各文件。

## 入口：从 main() 到 Robot_Init()

启动入口在 CubeMX 自动生成的 `board/dji_c/Src/main.c`（F407）或 `board/damiao_h7/Core/Src/main.c`（H723）。`main()` 做两件事：

```c
// main.c
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    // ... 大量 MX_xxx_Init()，全是 CubeMX 生成的外设初始化
    MX_USB_OTG_FS_PCD_Init();

    tx_kernel_enter();   // ← 启动 ThreadX 调度器
    while (1) {}
}
```

`tx_kernel_enter()` 是 ThreadX 的内核入口，内部会调用 `tx_application_define()`——这是 ThreadX 规范要求用户实现的函数，在 `tx_api.h` 中声明：

```c
// tx_api.h
VOID tx_application_define(VOID *first_unused_memory);
```

调度器在初始化阶段回调这个函数，用户代码就在这里：

```c
// main.c — USER CODE 区域
void tx_application_define(void *first_unused_memory) {
    Robot_Init();
}
```

所以 `Robot_Init()` 在调度器正式开始线程调度之前执行——此时还没有线程切换，所有初始化都是顺序执行的。

`Robot_Init()` 在 `robot/robot_init.c` 中：

```c
void Robot_Init(void) {
    tx_byte_pool_create(&tx_app_byte_pool, ...);   // 20KB 内存池
    UTILS_Init();
    BSP_Init();
    MODULE_Init();
    APP_Init();
}
```

## 全流程目录

```
Robot_Init()
├── tx_byte_pool_create()              20KB ThreadX 内存池
├── UTILS_Init()
│   └── ulog_init()                    日志初始化
├── BSP_Init()
│   ├── BSP_DWT_Init(168/480)          DWT 周期计数器
│   ├── PA12 拔插 + cdc_acm_init()     USB 虚拟串口
│   ├── BSP_LED_Init()                 LED
│   ├── BSP_BEEP_Init()                蜂鸣器
│   └── BSP_CAN_TaskInit()             CAN 总线任务
├── MODULE_Init()                      条件编译，按 ROBOT/BOARD 启用
│   ├── Module_Offline_init()          离线检测
│   ├── Module_Remote_init()           遥控器（SBUS/DT7）
│   ├── Module_BMI088_init()           IMU 驱动
│   ├── Module_INS_Init()              姿态解算
│   ├── Module_WT606_Init()            陀螺仪
│   ├── Module_Motor_Init()            电机任务（线程创建，链表此时为空）
│   │   └── tx_thread_create → motor_task_entry
│   │       └── 2ms: Motor_UpdateAll → PowerControl_Update → Motor_DJI_Flush
│   ├── Module_Vision_Init()           视觉通信
│   │   ├── Module_Offline_register()
│   │   └── tx_thread_create → vision_thread_entry
│   └── Module_BoardComm_Init()        板间通信
└── APP_Init()
    └── robot_control_init()           CMake 按 ROBOT/BOARD 选编译哪个
        ├─ ROBOT=sentry → [[03_moc/Sentry-Init]]
        │   ├─ 云台板: gimbal+shoot+vision+boardcomm
        │   └─ 底盘板: chassis+referee
        └─ ROBOT=infantry3 → [[03_moc/Infantry3-Init]]
            └─ 单板: gimbal+shoot+chassis, 无板间通信
```

## 各层初始化的详细展开

### UTILS_Init()

`utils/utils_init.c`

```c
void UTILS_Init(void) {
    ulog_init();
}
```

日志系统初始化，后续所有模块的 `LOG_I` / `LOG_E` 依赖此函数。

> 详见 [[../../02_code_twin/utils/ulog]]。

### BSP_Init()

`board/bsp/bsp_init.c`

`board/bsp/` 是共享硬件抽象层，`board/dji_c/`（F407）和 `board/damiao_h7/`（H723）是两个板子的 CubeMX 工程。BSP 代码用 `#if defined(STM32F407xx)` / `#if defined(STM32H723xx)` 条件编译区分两套硬件。

| 函数                      | 02 层链接                                      |
| ----------------------- | ------------------------------------------- |
| `BSP_DWT_Init(168/480)` | [[02_code_twin/board/bsp/DWT/bsp_dwt]]      |
| `cdc_acm_init()`        | [[02_code_twin/board/bsp/USB/bsp_usb_cdc]]  |
| `BSP_LED_Init()`        | [[02_code_twin/board/bsp/LED/bsp_led]]      |
| `BSP_BEEP_Init()`       | [[02_code_twin/board/bsp/BEEP/bsp_beep]]    |
| `BSP_CAN_TaskInit()`    | [[02_code_twin/board/bsp/CAN/bsp_can_task]] |
| 日志系统（ulog）已就绪       | [[../../02_code_twin/utils/ulog]]（UTILS_Init 中初始化） |

其中 USB 虚拟串口初始化前有一段 PA12 拔插复位逻辑（模拟 USB 拔插强制主机重新枚举），详见 [[02_code_twin/board/bsp/USB/bsp_usb_cdc]]。

### MODULE_Init()

`modules/module_init.c`

#### 条件编译机制

每个模块都被 `#if MODULE_XXX` 包裹，`MODULE_XXX` 是 CMake 生成的 C 宏，在 `module_config.h` 中定义。CMake 根据 `config.cmake` 中的 `ROBOT` 和 `BOARD` 变量决定哪些模块启用，详见 [[02_code_twin/apps/config-cmake]]。

以哨兵云台板（`ROBOT=sentry BOARD=gimbal`）为例，启用的模块：

| 函数                        | 02 层链接                                                                                                                                         | 哨兵云台板 |
| ------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- | :---: |
| `Module_Offline_init()`   | [[02_code_twin/modules/OFFLINE/module_offline-c]]                                                                                              |  启用   |
| `Module_Remote_init()`    | [[02_code_twin/modules/REMOTE/module_remote]] — SBUS: [[02_code_twin/modules/REMOTE/SBUS/sbus]] / DT7: [[02_code_twin/modules/REMOTE/DT7/dt7]] |  启用   |
| `Module_BMI088_init()`    | [[02_code_twin/modules/BMI088/module_bmi088]]                                                                                                  |  启用   |
| `Module_INS_Init()`       | [[02_code_twin/modules/INS/module_ins]]                                                                                                          |  启用   |
| `Module_Referee_Init()`   | 裁判系统                                                                                                                                           |  未启用  |
| `Module_WT606_Init()`     | 陀螺仪                                                                                                                                            |  启用   |
| `Module_SuperCap_Init()`  | 超级电容                                                                                                                                           |  未启用  |
| `Module_Motor_Init()`     | [[02_code_twin/modules/MOTOR/motor_base]]                                                                                                      |  启用   |
| `Module_Vision_Init()`    | [[02_code_twin/modules/VISION/module_vision]]                                                                                                  |  启用   |
| `Module_BoardComm_Init()` | 板间通信                                                                                                                                           |  启用   |

#### Module_Motor_Init() — 线程先建，电机后注册

`Module_Motor_Init()` 只做一件事：创建 motor 线程并启动 2ms 循环：

```c
void Module_Motor_Init(void) {
    tx_thread_create(&motor_thread, "motor", motor_task_entry, ...);
}

static void motor_task_entry(ULONG thread_input) {
    while (1) {
        Motor_UpdateAll();        // 遍历电机链表，逐个调用 ControlAndSend
        PowerControl_Update();
        Motor_DJI_Flush();        // 大疆电机批量发送
        tx_thread_sleep(2);      // 2ms 周期
    }
}
```

但此时电机链表是空的——`Motor_UpdateAll()` 遍历空链表直接返回，什么也不做。电机注册发生在后面的 APP_Init 阶段，`gimbal_init()` 和 `shoot_init()` 通过 `Motor_DJI_Init()` 将电机挂入链表。注册完成后，下一个 2ms 周期 `Motor_UpdateAll()` 就能遍历到电机并开始控制。

为什么要这样设计？因为 `Module_Motor_Init()` 是模块层（所有兵种共用），而电机注册是应用层（不同兵种注册不同电机）。模块层先建好线程和链表框架，应用层往里面填电机，详见 [[02_code_twin/modules/MOTOR/motor_base]]。

#### Module_Vision_Init()

```c
void Module_Vision_Init(void) {
    offline_dev = Module_Offline_register(&offlineconfig);  // 注册离线检测
    tx_thread_create(&vision_thread, "vision_thread", ...); // 创建接收线程
}
```

创建独立的视觉接收线程，阻塞等待 USB 数据。详见 [[02_code_twin/modules/VISION/module_vision]]。

### APP_Init()

`apps/app_init.c` → [[02_code_twin/apps/app_init]]

`APP_Init()` 只有一行 `robot_control_init()`，是一个转发点。`robot_control_init()` 的实现在各兵种各板型目录下，由 CMake 根据 `ROBOT` 和 `BOARD` 变量决定编译哪个版本：

```
APP_Init()              apps/app_init.c
  └─ robot_control_init()
       │  CMake 按 ROBOT/BOARD 选编译哪个 .c
       │
       ├─ ROBOT=sentry → 哨兵（双板：云台板 + 底盘板）
       │    → [[03_moc/Sentry-Init]]
       │
       └─ ROBOT=infantry3 → 步兵3号（单板）
            → [[03_moc/Infantry3-Init]]
```

哨兵和步兵3号的完整初始化流程（电机注册、2ms 循环、子模块链接）分别展开在：

- **哨兵**：[[03_moc/Sentry-Init]] — 双板架构，云台板管 gimbal+shoot+视觉+板间通信，底盘板管 chassis+裁判系统回传
- **步兵3号**：[[03_moc/Infantry3-Init]] — 单板架构，gimbal+shoot+chassis 全在一块板，无板间通信

至此电机链表填充完毕，motor task 的 2ms 循环开始真正控制电机，robot_control_task 的 2ms 循环开始执行控制逻辑。

## 初始化时序依赖

初始化顺序固定，前一步是后一步的前置条件：

| 依赖 | 原因 |
|------|------|
| `tx_byte_pool_create()` 最先 | 后续所有 `BSP_MEM_ALLOC_WAIT` 依赖此内存池 |
| `BSP_DWT_Init()` 在 Offline 之前 | Offline 依赖 `BSP_DWT_GetTimeline_ms()` 获取时间戳 |
| `Module_Offline_init()` 在其他模块之前 | 后续模块调 `Module_Offline_register()` 依赖 `g_initialized` |
| `BSP_CAN_TaskInit()` 在 Motor 之前 | 电机初始化调 `BSP_CAN_Device_Init()` 依赖 CAN 总线已启动 |
| `Module_Motor_Init()` 在 APP_Init 之前 | 线程和链表框架先建好，APP_Init 再往里注册电机 |
