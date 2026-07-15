# 应用初始化全流程

> 从 `main()` 调用 `Robot_Init()` 开始，到 `robot_control_init()` 结束，程序到底干了什么。
> 本文是阅读入口，按调用顺序逐层展开，并链接到 02 层各文件的源码注释映射。

---

## 阅读顺序

先看 `robot/robot_init.c` 中的 `Robot_Init()`——它是整个应用侧的唯一入口。函数体只有四行调用，但每一行背后都拖着一整条初始化链。

整条链路是严格顺序的：内存池 → 工具 → BSP → 模块 → 应用。前一步失败，后一步不会执行。

---

## 全流程概览

```
Robot_Init()                          ← robot/robot_init.c
  │
  ├─ tx_byte_pool_create()            ← 20KB ThreadX 字节内存池
  │    所有后续 BSP_MEM_ALLOC_WAIT 都从这里分配
  │
  ├─ UTILS_Init()                     ← utils/utils_init.c
  │    └─ ulog_init()                 ← 日志系统初始化
  │
  ├─ BSP_Init()                       ← board/bsp/bsp_init.c
  │    ├─ BSP_DWT_Init(168 / 480)    ← DWT 周期计数器, 提供微秒级延时和时间轴
  │    ├─ cdc_acm_init()              ← USB 虚拟串口 (日志输出通道)
  │    ├─ BSP_LED_Init()              ← LED (绿/红/黑 三态)
  │    ├─ BSP_BEEP_Init()             ← 蜂鸣器
  │    └─ BSP_CAN_TaskInit()          ← CAN 总线初始化 + CAN 接收任务
  │
  ├─ MODULE_Init()                    ← modules/module_init.c  ★核心
  │    ├─ Module_Offline_init()       ← 离线检测模块 (链表 + 看门狗 + 蜂鸣/LED 状态机)
  │    ├─ Module_Remote_init()        ← 遥控器模块 (SBUS/DT7 + VT02/VT03)
  │    ├─ Module_BMI088_init()        ← IMU
  │    ├─ Module_INS_Init()           ← 姿态解算
  │    ├─ Module_Referee_Init()       ← 裁判系统
  │    ├─ Module_WT606_Init()         ← 陀螺仪
  │    ├─ Module_SuperCap_Init()      ← 超级电容
  │    ├─ Module_Motor_Init()         ← 电机控制模块 (2ms 循环 + 链表调度)
  │    ├─ Module_Vision_Init()        ← 视觉通信
  │    └─ Module_BoardComm_Init()     ← 板间通信
  │
  └─ APP_Init()                       ← apps/app_init.c
       └─ robot_control_init()        ← 机器人具体逻辑 (电机注册 + 遥控映射 + 控制线程)
```

---

## 跳转深度

从 `Robot_Init()` 出发，跟随每一层调用逐级深入。缩进表示调用层级，箭头表示链接到 02 层的哪篇笔记：

```
robot/robot_init.c — Robot_Init()
  │
  ├─ tx_byte_pool_create(&tx_app_byte_pool, ..., 20*1024)
  │    创建 ThreadX 字节内存池, 后续所有 BSP_MEM_ALLOC_WAIT 从此分配
  │
  ├─ UTILS_Init()
  │    └─ ulog_init()
  │
  ├─ BSP_Init()
  │    ├─ BSP_DWT_Init(168)          ← F407: 168MHz, H723: 480MHz
  │    ├─ cdc_acm_init(...)
  │    ├─ BSP_LED_Init()
  │    ├─ BSP_BEEP_Init()
  │    └─ BSP_CAN_TaskInit()
  │
  ├─ MODULE_Init()
  │    │
  │    ├─ Module_Offline_init()       ← 【展开见下文】
  │    ├─ Module_Remote_init()        ← 【展开见下文】
  │    ├─ ...
  │    ├─ Module_Motor_Init()         ← 【展开见下文】
  │    └─ ...
  │
  └─ APP_Init()
       └─ robot_control_init()        ← 各机器人自己的 init, 参见 apps/<robot>/
```

---

## MODULE_Init() 展开

`module_init.c` 的函数体是一串 `#if MODULE_XXX` 条件编译调用，顺序固定：

```
Module_Offline_init()    ← 必须第一个！后续模块注册离线设备依赖它
Module_Remote_init()     ← 遥控器
Module_BMI088_init()     ← IMU
Module_INS_Init()        ← 姿态解算
Module_Referee_Init()    ← 裁判系统
Module_WT606_Init()      ← 陀螺仪
Module_SuperCap_Init()   ← 超级电容
Module_Motor_Init()      ← 电机 (必须在所有电机注册之前)
Module_Vision_Init()     ← 视觉
Module_BoardComm_Init()  ← 板间通信
```

> **关键顺序约束**：`Module_Offline_init()` 必须最先调用。因为 Remote、Motor 等后续模块在初始化时会调用 `Module_Offline_register()` 注册自己的离线检测设备，而 `register` 内部会检查 `g_initialized` 标志——如果 Offline 模块未初始化，注册直接返回 NULL。

---

### Offline 模块展开

> 源文件：`modules/OFFLINE/module_offline.{h,c}`
> 02 层映射：[[02_code_twin/modules/OFFLINE/module_offline-h]] / [[02_code_twin/modules/OFFLINE/module_offline-c]]

#### 一句话

离线检测模块维护一个设备链表，每个设备有自己的超时时间。一个 10ms 周期的检测任务遍历链表，超时的设备标记为离线，并根据配置驱动蜂鸣器和 LED 报警。

#### 数据结构

```c
struct Offline_Device {
    Offline_Device *next;       // 单向链表指针
    char            name[32];   // 设备名称 (日志用)
    uint32_t        timeout_ms; // 超时阈值
    uint8_t         beep_times; // 蜂鸣次数 (0=静默故障, >0=严重故障)
    bool            is_offline; // 当前状态
    uint32_t        last_time;  // 上次心跳时间戳 (ms, 来自 DWT)
    uint8_t         enable;     // 检测开关
};
```

链表头指针 `g_device_list` 是模块内静态变量，新设备头插。

#### 初始化：`Module_Offline_init()`

```
Module_Offline_init()
  │
  ├─ g_device_list = NULL               ← 清空链表
  ├─ BSP_BEEP_Start()                   ← 启动蜂鸣器硬件
  ├─ BSP_LED_Show(LED_Green)            ← 初始绿灯 (全在线)
  │
  ├─ tx_thread_create(offline_detect)   ← 创建检测任务
  │    栈: 1024B, 优先级: 6, 周期: 10ms (tx_thread_sleep(10))
  │
  └─ MX_IWDG_Init()                     ← 独立看门狗 (调试器暂停时冻结)
       __HAL_DBGMCU_FREEZE_IWDG()
```

#### 注册：`Module_Offline_register()`

任何模块（Remote、Motor、Vision 等）都可以注册自己的离线设备：

```
Module_Offline_register(&cfg)
  │
  ├─ 检查 g_initialized (必须先调 init)
  │
  ├─ BSP_MEM_ALLOC_WAIT(dev, sizeof(Offline_Device))
  │    ← 从 tx_app_byte_pool (20KB) 分配内存
  │
  ├─ memset(dev, 0, ...)
  ├─ strncpy(dev->name, cfg->name, 31)
  ├─ dev->timeout_ms = cfg->timeout_ms
  ├─ dev->beep_times = cfg->beep_times
  ├─ dev->enable     = cfg->enable
  ├─ dev->last_time  = BSP_DWT_GetTimeline_ms()   ← 初始心跳=当前时间
  │
  └─ 头插链表: dev->next = g_device_list; g_device_list = dev
```

#### 心跳更新：`Module_Offline_device_update()`

```c
void Module_Offline_device_update(Offline_Device *dev) {
    dev->last_time = BSP_DWT_GetTimeline_ms();
}
```

一行代码。各模块在收到有效数据时调用（如 CAN 接收回调、UART 解码成功、PS2 轮询成功），更新时间戳。

#### 检测任务：`offline_detect_task_entry()` (10ms 周期)

```
while (1) {
  ┌─ HAL_IWDG_Refresh()                    ← 喂看门狗 (每次循环)
  │
  ├─ now = BSP_DWT_GetTimeline_ms()
  ├─ 遍历 g_device_list:
  │    elapsed = now - dev->last_time
  │    if (elapsed > dev->timeout_ms)
  │        dev->is_offline = true
  │        if (dev->beep_times > 0)  → 严重故障, 记录蜂鸣次数
  │        else                       → 静默故障
  │    else
  │        dev->is_offline = false
  │
  └─ 蜂鸣/LED 状态机:
       严重故障: 2s 大周期, 按 beep_times 闪烁 (100ms 红+响 / 100ms 黑+停)
       静默故障: 红灯常亮, 蜂鸣关闭
       全在线:   绿灯常亮

  tx_thread_sleep(10)                      ← 10ms 周期
}
```

#### 蜂鸣/LED 状态机详解

状态机维护三个变量：

| 变量 | 作用 |
|------|------|
| `beep_period_start_time` | 当前 2s 大周期起始时刻 |
| `beep_cycle_start_time` | 当前闪烁子周期起始时刻 |
| `remaining_beep_cycles` | 当前大周期内剩余闪烁次数 |

时序：

```
|←──────────── 2s 大周期 ────────────→|
| 100ms  100ms  100ms 100ms           |
| 红+响  黑+停  红+响  黑+停  ...  绿  |
|← beep_times 次 →|    剩余时间绿灯    |
```

- `beep_times = 3`（如电机离线）：响 3 次，然后绿灯到周期结束
- `beep_times = 0`（如遥控器静默离线）：红灯常亮，不响
- 多个设备同时离线：取 `beep_times` 最小的那个为准（最紧急的优先报警）

#### 离线状态查询：`Module_Offline_get_device_status()`

```c
uint8_t Module_Offline_get_device_status(Offline_Device *dev) {
    if (dev == NULL) return STATE_OFFLINE;  // NULL 指针 = 离线
    return dev->is_offline ? STATE_OFFLINE : STATE_ONLINE;
}
```

各模块在控制循环中调用此函数，离线时清零输出。

#### 与其他模块的交互

```
Module_Offline_init()          ← MODULE_Init() 第一个调用
       ↑
       │ 注册
       ├─← Module_Remote_init()      → 注册 "sbus" / "dt7" 设备 (50ms 超时)
       ├─← Motor_DM_Init()           → 注册 "dm4310" 等设备 (200ms 超时)
       ├─← Motor_LK_Init()           → 注册 "mf9025_l" 等设备
       ├─← Module_BMI088_init()      → 注册 IMU 设备
       └─← ...
       ↑
       │ 心跳更新 (在各自的数据接收回调中)
       ├─← dm_can_rx_callback()      → Module_Offline_device_update(motor->base.offline_dev)
       ├─← remote_sbus_decode()      → Module_Offline_device_update(offline)
       └─← ...
       ↑
       │ 状态查询 (在各自的控制循环中)
       ├─← dm_ControlAndSend()       → Module_Offline_get_device_status(base->offline_dev)
       └─← robot_control_task()      → Module_PS2_IsOnline() → 内部查离线状态
```

---

### Remote 模块展开

> 源文件：`modules/REMOTE/module_remote.{h,c}` + `SBUS/sbus.{h,c}` + `DT7/dt7.{h,c}`
> 02 层映射：[[02_code_twin/modules/REMOTE/module_remote-h]] / [[02_code_twin/modules/REMOTE/module_remote-c]]
>           [[02_code_twin/modules/REMOTE/SBUS/sbus-c]] / [[02_code_twin/modules/REMOTE/DT7/dt7-c]]

> **TODO**：待 02 层 SBUS / DT7 code twin 完成后补充展开内容。

#### 一句话

Remote 模块用编译期宏 `REMOTE_SOURCE` 选择 SBUS 或 DT7，用 `REMOTE_VT_SOURCE` 选择 VT02 或 VT03。初始化时创建两个 ThreadX 任务（遥控器解码 + 图传解码），各自阻塞在 `BSP_UART_Read()` 上等 DMA 数据，解码后直接写入全局 `g_remote_data`。

#### 初始化：`Module_Remote_init()`

```
Module_Remote_init()
  │
  ├─ memset(&g_remote_data, 0, ...)
  │
  ├─ #if (REMOTE_SOURCE == 1)
  │    remote_sbus_init(&g_rc_offline)
  │      ├─ REMOTE_UART.Init.BaudRate = 100000
  │      ├─ HAL_UART_Init(&REMOTE_UART)
  │      ├─ Module_Offline_register("sbus", 50ms)
  │      └─ BSP_UART_Device_Init(DMA, expected_rx_len=25)
  │
  ├─ #elif (REMOTE_SOURCE == 2)
  │    remote_dt7_init(&g_rc_offline)
  │      ├─ REMOTE_UART.Init.BaudRate = 100000
  │      ├─ HAL_UART_Init(&REMOTE_UART)
  │      ├─ Module_Offline_register("dt7", 50ms)
  │      └─ BSP_UART_Device_Init(DMA, expected_rx_len=18)
  │
  ├─ #if (REMOTE_VT_SOURCE == 1/2)
  │    remote_vt02_init() / remote_vt03_init()
  │
  ├─ tx_thread_create("Remote Ctrl", remote_ctrl_task_entry)
  │    栈: 1024B, 优先级: 9
  │
  └─ tx_thread_create("Remote VT", vt_task_entry)
       栈: 1024B, 优先级: 8
```

#### 解码任务

```
remote_ctrl_task_entry():
  while (1) {
    #if (REMOTE_SOURCE == 1)
      remote_sbus_decode(&g_remote_data, g_rc_offline)
        ├─ BSP_UART_Read(sbus_uart, buf, 64, &rx_len, TX_WAIT_FOREVER)
        ├─ for 循环扫描帧头 0x0F + 帧尾 0x00
        ├─ 解码 ch1-4 (零偏 + 死区) + ch5-16 (原始值)
        └─ Module_Offline_device_update(offline)
    #elif (REMOTE_SOURCE == 2)
      remote_dt7_decode(&g_remote_data, g_rc_offline)
        ├─ BSP_UART_Read(dt7_uart, buf, 64, &rx_len, TX_WAIT_FOREVER)
        ├─ for 循环 + 通道边界检查 (364..1684)
        ├─ 解码 ch1-4 + sw1/sw2 + 鼠标/键盘/滚轮
        └─ Module_Offline_device_update(offline)
  }
```

> **TODO**：待 02 层 SBUS / DT7 code twin 完成后，在此补充解码逻辑的逐行映射链接。

---

### Motor 模块展开

> 源文件：`modules/MOTOR/module_motor.{h,c}` + `motor_base.{h,c}` + `DAMIAO/motor_damiao.{h,c}` + `DJI/motor_dji.{h,c}`
> 02 层映射：[[02_code_twin/modules/MOTOR/module_motor-c]] / [[02_code_twin/modules/MOTOR/motor_base-c]]
>           [[02_code_twin/modules/MOTOR/DAMIAO/motor_damiao-c]] / [[02_code_twin/modules/MOTOR/DJI/motor_dji-c]]

> **TODO**：待 02 层 Motor code twin 完成后补充展开内容。

#### 一句话

Motor 模块维护一个 `Motor_Base` 链表，每个电机实例注册时挂入。一个 2ms 周期的 motor task 遍历链表，调用每个电机的 `ControlAndSend` 回调完成 PID 计算和 CAN 发送。达妙电机每台独立发帧；大疆电机多台共用一帧（分组发送），所以在 `Motor_UpdateAll()` 之后还要调一次 `Motor_DJI_Flush()`。

#### 初始化：`Module_Motor_Init()`

```
Module_Motor_Init()
  │
  └─ tx_thread_create("motor", motor_task_entry)
       栈: MOTOR_TASK_STACK_SIZE, 优先级: MOTOR_TASK_PRIORITY
       周期: 2ms (tx_thread_sleep(2))
```

注意：`Module_Motor_Init()` 只创建调度任务，**不注册任何电机**。电机注册在各机器人的 `robot_control_init()` 中完成。

#### Motor Task (2ms 循环)

```
motor_task_entry():
  while (1) {
    Motor_UpdateAll()       ← 遍历链表, 调每个电机的 ControlAndSend
    PowerControl_Update()   ← 功率控制
    Motor_DJI_Flush()       ← 批量发送大疆电机 CAN 帧
    tx_thread_sleep(2)      ← 2ms 周期
  }
```

#### Motor_Base 抽象基类

```c
struct Motor_Base {
    Motor_Base       *next;              // 链表指针
    Motor_Type_e      type;              // 电机型号
    Motor_Transport_e transport;         // 传输层 (CAN/UART/PWM)
    Motor_Info_s      info;              // 基本信息 (减速比, 力矩常数)
    Motor_Setting_s   setting;           // 控制设置 (闭环类型, 使能, 反转)
    Motor_Controller_s controller;       // 控制器 (PID/LQR, ref, output)
    Motor_Measure_s   measure;           // 测量数据 (速度, 角度, 力矩)
    Offline_Device   *offline_dev;       // 离线检测设备
    void             *transport_dev;     // 传输层设备 (Can_Device *)
    void (*ControlAndSend)(Motor_Base *); // 回调: PID计算 + CAN发送
};
```

#### 电机注册流程（以达妙为例）

```
Motor_DM_Init(&config, DM_MIT_MODE)
  │
  ├─ BSP_MEM_ALLOC_WAIT(motor, sizeof(DM_Motor_t))   ← 从内存池分配
  ├─ memset(motor, 0, ...)
  ├─ motor->base.type      = config.motor_init_info.motor_type
  ├─ motor->base.setting   = config.setting_init_config
  ├─ motor->mode_type      = DM_MIT_MODE
  ├─ motor->params         = dm_get_params(type)      ← 查参数表
  │
  ├─ 校验 Master ID > CAN ID
  │
  ├─ BSP_CAN_Device_Init(&config.transport_config.can) ← 注册 CAN 设备
  ├─ can_dev->rx_callback = dm_can_rx_callback         ← 设置接收回调
  ├─ can_dev->user_arg    = motor                      ← 回调上下文
  │
  ├─ PIDInit(&motor->base.controller.speed_PID, ...)   ← 初始化 PID
  ├─ PIDInit(&motor->base.controller.angle_PID, ...)
  │
  ├─ motor->base.offline_dev = Module_Offline_register(...)  ← 注册离线检测
  │
  ├─ Motor_DM_Cmd(motor, DM_CMD_CLEAR_ERROR)           ← 清除错误
  ├─ Motor_DM_Cmd(motor, DM_CMD_MOTOR_START)           ← 使能电机
  │
  ├─ motor->base.ControlAndSend = dm_ControlAndSend    ← 设置回调
  └─ Motor_Register(&motor->base)                      ← 挂入全局链表
```

#### ControlAndSend 回调对比

| 维度 | 达妙 `dm_ControlAndSend` | 大疆 `dji_control` |
|------|--------------------------|---------------------|
| PID 计算 | `CalculatePIDOutput()` → 力矩 | `CalculatePIDOutput()` → 力矩 → 电流值 |
| CAN 发送 | 立即发送（每台电机一帧） | 只填充 `sender_assignment` 缓冲区 |
| 批量发送 | 不需要 | `Motor_DJI_Flush()` 统一发送 |
| MIT 模式 | `mit_ctrl(pos, vel, kp, kd, torque)` | 不适用 |
| 速度模式 | `speed_ctrl(vel)` — 4 字节 float | 不适用 |
| 离线处理 | 发零值 MIT 帧 | 清零缓冲区对应位置 |

> **TODO**：待 02 层 Motor code twin 完成后，在此补充 PID 计算流程和 CAN 帧结构的逐行映射链接。

---

## APP_Init() 展开

```
APP_Init()
  └─ robot_control_init()    ← apps/<robot>/single_board/robot_control.c
       ├─ 各机器人自己的电机注册 (Motor_DM_Init / Motor_DJI_Init / ...)
       ├─ 各机器人自己的遥控映射
       └─ tx_thread_create("robot_control", robot_control_task)
            周期: 2ms, 优先级: 通常 30
```

`robot_control_init()` 的具体内容取决于 `config.cmake` 中选择的 `ROBOT` 变量。例如 `ROBOT=infantry3` 会调用 `apps/infantry3/single_board/robot_control.c` 中的实现。

---

## 初始化顺序约束总结

| 约束 | 原因 |
|------|------|
| `tx_byte_pool_create` 必须最先 | 后续所有 `BSP_MEM_ALLOC_WAIT` 依赖此内存池 |
| `BSP_DWT_Init` 必须在 Offline 之前 | Offline 依赖 `BSP_DWT_GetTimeline_ms()` 获取时间戳 |
| `Module_Offline_init` 必须在所有其他模块之前 | 其他模块初始化时调 `Module_Offline_register()` 依赖 `g_initialized` |
| `Module_Motor_Init` 必须在 `robot_control_init` 之前 | 后者注册电机时调 `Motor_Register()` 挂入链表, motor task 需已就绪 |
| `BSP_CAN_TaskInit` 必须在 Motor 之前 | 电机初始化时调 `BSP_CAN_Device_Init()` 依赖 CAN 总线已启动 |
