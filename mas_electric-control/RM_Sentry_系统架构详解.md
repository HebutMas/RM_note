# 哨兵系统架构详解 —— 双板通信、RTOS 与模块集成

## 目录

1. [系统架构总览：双板分工](#1-系统架构总览双板分工)
2. [五层初始化流程](#2-五层初始化流程)
3. [ThreadX RTOS 任务架构](#3-threadx-rtos-任务架构)
4. [板间 CAN 通信协议](#4-板间-can-通信协议)
5. [传感器与姿态估计 (INS)](#5-传感器与姿态估计-ins)
6. [视觉系统通信](#6-视觉系统通信)
7. [遥控器协议映射](#7-遥控器协议映射)
8. [云台自动搜索三态机](#8-云台自动搜索三态机)
9. [安全保护机制](#9-安全保护机制)
10. [CMake 条件编译机制](#10-cmake-条件编译机制)
11. [裁判系统协议透传](#11-裁判系统协议透传)
12. [BSP 硬件抽象层](#12-bsp-硬件抽象层)
13. [概念关系图谱](#13-概念关系图谱)

---

## 1. 系统架构总览：双板分工

### 1.1 为什么需要双板

哨兵机器人需要同时控制 **11 个电机**（8 底盘 + 3 云台/发射）+ 运行姿态估计 + 与视觉计算机通信 + 接收裁判系统数据。单块 MCU 的 CAN 总线带宽、CPU 算力、引脚资源都不足以承载全部任务。

因此采用 **双板架构**：

```
┌─────────────────────────────────────────────────────────────┐
│                        哨兵机器人                            │
│                                                             │
│  ┌───────────────────────┐    ┌───────────────────────┐     │
│  │      云台板 (Gimbal)    │    │     底盘板 (Chassis)    │     │
│  │      STM32H723         │    │      STM32H723         │     │
│  │                        │    │                        │     │
│  │  • 小 YAW (GM6020)     │    │  • 驱动轮 ×4 (M3508)   │     │
│  │  • 大 YAW (GM6020)     │    │  • 转向轮 ×4 (GM6020)  │     │
│  │  • PITCH (DM4310)      │    │  • 功率控制            │     │
│  │  • 摩擦轮 ×2 (M3508)   │    │  • 裁判系统接收         │     │
│  │  • 拨盘 (M2006)        │    │  • 底盘运动学解算       │     │
│  │  • IMU (BMI088)        │    │  • IMU (BMI088)        │     │
│  │  • WT606 陀螺仪        │    │  • INS 姿态估计         │     │
│  │  • INS 姿态估计         │    │                        │     │
│  │  • 遥控器接收 (SBUS)    │    │                        │     │
│  │  • 视觉通信 (VCP)       │◄───┤  CAN 总线 (500kbps)    │     │
│  │                        │    │                        │     │
│  └───────────────────────┘    └───────────────────────┘     │
│           │                            │                     │
│           │ CAN1 (电机总线)             │ CAN2 (底盘电机)     │
│           ▼                            ▼                     │
│    GM6020×2, DM4310,          M3508×4 (驱动轮)               │
│    M3508×2, M2006             GM6020×4 (转向轮)              │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 职责划分

| 维度 | 云台板 (Gimbal) | 底盘板 (Chassis) |
|:------|:----------------|:------------------|
| **电机控制** | 小/大 YAW + PITCH + 摩擦轮×2 + 拨盘 (6电机) | 驱动轮×4 + 转向轮×4 (8电机) |
| **传感器** | BMI088 + WT606 | BMI088 |
| **姿态估计** | INS (四元数 EKF) | INS (四元数 EKF) |
| **通信接收** | 遥控器 (SBUS), 视觉计算机 (USB VCP) | 裁判系统 (UART) |
| **通信发送** | 底盘指令 → CAN → 底盘板 | 裁判数据 → CAN → 云台板 |
| **功率控制** | 无（由底盘板执行） | 有（PowerControl_Update） |
| **控制频率** | 500Hz (2ms 周期) | 500Hz (2ms 周期) |

### 1.3 模块启用对比

```c
// apps/sentry/robot.cmake:6-7
set(MODULES_GIMBAL   OFFLINE REMOTE BMI088 INS WT606 MOTOR VISION BOARDCOMM)
set(MODULES_CHASSIS  OFFLINE BMI088 INS REFEREE MOTOR BOARDCOMM)
```

| 模块 | 云台板 | 底盘板 | 功能 |
|:------|:------:|:------:|:-----|
| OFFLINE | ✓ | ✓ | 设备离线检测 + 蜂鸣 |
| REMOTE | ✓ | ✗ | SBUS/DT7 遥控器解码 |
| BMI088 | ✓ | ✓ | IMU 加速度计+陀螺仪 |
| INS | ✓ | ✓ | 姿态估计 (四元数 EKF) |
| WT606 | ✓ | ✗ | 大 YAW 陀螺仪模块 |
| MOTOR | ✓ | ✓ | 电机控制 (DJI/DM/Servo) |
| VISION | ✓ | ✗ | 视觉计算机 VCP 通信 |
| BOARDCOMM | ✓ | ✓ | 板间 CAN 通信 |
| REFEREE | ✗ | ✓ | 裁判系统协议解析 |

---

## 2. 五层初始化流程

### 2.1 从启动到主循环

```
上电 → HAL_Init() → tx_kernel_enter() → tx_application_define()
                                              │
                                              ▼
                                       Robot_Init()
                                         │
                          ┌──────────────┼──────────────┐
                          ▼              ▼              ▼              ▼
                    UTILS_Init()   BSP_Init()   MODULE_Init()   APP_Init()
                     (日志系统)    (硬件抽象)   (功能模块)     (业务逻辑)
```

### 2.2 Robot_Init() 源码

```c
// robot/robot_init.c:27-43
void Robot_Init(void)
{
    // ① 创建 20KB 内存池 (ThreadX byte pool)
    tx_byte_pool_create(&tx_app_byte_pool, "Tx App memory pool",
                        tx_byte_pool_buffer, 20 * 1024);

    UTILS_Init();   // ② SEGGER RTT 日志系统
    BSP_Init();     // ③ 硬件初始化 (DWT/CAN/UART/SPI/GPIO/USB)
    MODULE_Init();  // ④ 条件初始化各模块
    APP_Init();     // ⑤ 创建主控制任务 (robot_control_task)
}
```

### 2.3 MODULE_Init() 条件初始化

```c
// modules/module_init.c:47-81
void MODULE_Init(void)
{
#if MODULE_OFFLINE
    Module_Offline_init();     // 离线检测任务 (优先级 6)
#endif
#if MODULE_REMOTE
    Module_Remote_init();      // 遥控器解码任务 (优先级 9)
#endif
#if MODULE_BMI088
    Module_BMI088_init();      // IMU 传感器 SPI 驱动
#endif
#if MODULE_INS
    Module_INS_Init();         // 姿态估计任务 (优先级 7)
#endif
#if MODULE_REFEREE
    Module_Referee_Init();     // 裁判系统 UART 解析 (优先级 10)
#endif
#if MODULE_WT606
    Module_WT606_Init();       // WT606 陀螺仪 UART 驱动
#endif
#if MODULE_MOTOR
    Module_Motor_Init();       // 电机 CAN 收发任务 (优先级 12)
#endif
#if MODULE_VISION
    Module_Vision_Init();      // 视觉 VCP 通信任务 (优先级 10)
#endif
#if MODULE_BOARDCOMM
    Module_BoardComm_Init();   // 板间 CAN 通信
#endif
}
```

### 2.4 初始化顺序的关键约束

| 顺序  | 模块      | 被依赖方       | 原因                                            |
| :-- | :------ | :--------- | :-------------------------------------------- |
| 1   | OFFLINE | 所有模块       | 其他模块注册离线设备前需要 OFFLINE 就绪                      |
| 2   | REMOTE  | APP        | APP 层读取遥控器通道值前需要解码就绪                          |
| 3   | BMI088  | INS        | INS 姿态估计依赖 BMI088 的陀螺仪+加速度数据                  |
| 4   | INS     | APP (云台控制) | `gimbal_func` 需要 `ins->YawTotalAngle_rad` 等数据 |
| 5   | MOTOR   | APP        | APP 层创建电机实例前需要 MOTOR 模块初始化                    |

---

## 3. ThreadX RTOS 任务架构

### 3.1 任务总览

```
优先级 (数值越小越高)
  │
  0 ─ ThreadX 内核 (timer, idle)
  6 ─ OFFLINE 检测任务      (1Hz, 检查所有注册设备心跳)
  7 ─ INS 姿态估计任务        (1kHz, 四元数 EKF 更新)
  8 ─ WT606 / VT 图传任务     (串口接收解析)
  9 ─ REMOTE 遥控器解码任务   (SBUS → 标准化通道值)
 10 ─ VISION 通信任务 / REFEREE 裁判系统任务
 12 ─ MOTOR CAN 收发任务      (最高吞吐, 最低延迟)
 30 ─ robot_control_task      (500Hz 主控制循环)
```

### 3.2 主控制任务 (robot_control_task)

```c
// apps/sentry/gimbal_board/robot_control.c:34-65
static void robot_control_task(ULONG thread_input)
{
    while (1)
    {
        RemoteControlSet(&chassis_cmd, &shoot_cmd, &gimbal_cmd);  // ① 遥控器 → 指令
        send_packet.mode = 1 - chassis_upload_data.robot_color;   // ② 视觉发送
        send_packet.q[0..3] = ins->q[0..3];                        //    四元数
        Module_Vision_Send(&send_packet, TX_NO_WAIT);
        receive_packet = Module_Vision_Receive();                  // ③ 视觉接收

        gimbal_func(&gimbal_cmd, &yaw_ecd);                        // ④ 云台控制
        shoot_func(&shoot_cmd);                                    // ⑤ 发射控制

        chassis_send_cmd.vx = (int8_t)(chassis_cmd.vx * 10.0f);   // ⑥ 板间发送
        Module_BoardComm_Send((uint8_t *)&chassis_send_cmd, 8);

        tx_thread_sleep(2);  // 2ms → 500Hz
    }
}
```

**时序图**：

```
0ms                 1ms                 2ms
├───────────────────┼───────────────────┤
│ RemoteControlSet  │                   │
│ Vision Send/Recv  │                   │
│ gimbal_func       │                   │
│ shoot_func        │                   │
│ BoardComm_Send    │                   │
│                   │ tx_thread_sleep(2)│
│                   │ (让出 CPU)        │
└───────────────────┴───────────────────┴──→ 下一周期
```

### 3.3 任务栈与内存

每个 ThreadX 任务有独立的 **1024 字节栈空间**（`TX_THREAD` 栈），通过 `APPS_STACK_SECTION` 宏放入特定内存段。主应用层使用 20KB 字节池（`tx_app_byte_pool`）进行动态内存分配。

---

## 4. 板间 CAN 通信协议

### 4.1 物理层

| 参数 | 值 |
|:------|:----|
| 总线 | CAN2 (`BSP_CAN_HANDLE2`) |
| 波特率 | 1 Mbps |
| 云台→底盘 ID | `0x310` |
| 底盘→云台 ID | `0x311` |
| 数据长度 | 8 字节（固定） |
| 发送频率 | 500Hz（每个主循环一次） |

### 4.2 云台→底盘 (ID 0x310)：底盘控制指令

```c
// apps/sentry/sentry_def.h:87-96
#pragma pack(1)
typedef struct {
    int8_t  vx;           // [0]  X方向速度 (-10 ~ +10)
    int8_t  vy;           // [1]  Y方向速度 (-10 ~ +10)
    int8_t  wz;           // [2]  旋转速度    (-10 ~ +10)
    int16_t offset_angle; // [3-4] YAW 偏移角度 (编码器差值 0~8191)
    uint8_t chassis_mode; // [5]  底盘模式 (chassis_mode_e)
    int8_t  reserved[2];  // [6-7] 保留 (填充至 8 字节)
} GimbalToChassis_cmd_t;
_Static_assert(sizeof(GimbalToChassis_cmd_t) == 8, "...");
```

**速度量纲映射**：
- 发送端：`vx_int8 = (int8_t)(vx_float * 10.0f)` — 遥控杆量 (-1.0~+1.0) 映射到 int8 (-10~+10)
- 接收端：`vx_mps = vx_int8 / 10.0f * CHASSIS_MAX_SPEED_MPS` — 还原为实际速度 (m/s)

### 4.3 底盘→云台 (ID 0x311)：裁判系统数据回传

```c
// apps/sentry/sentry_def.h:98-106
typedef struct {
    uint8_t  robot_color;      // [0]  机器人颜色 (0=红, 1=蓝)
    uint8_t  game_progress;    // [1]  比赛阶段 (0=未开始, 4=比赛中)
    uint16_t current_hp;       // [2-3] 当前血量
    uint16_t bullet_allow;     // [4-5] 17mm 弹丸剩余量
    uint16_t shooter_heat_pct; // [6-7] 枪管热量百分比
} ChassisToGimbal_referee_t;
_Static_assert(sizeof(ChassisToGimbal_referee_t) == 8, "...");
```

### 4.4 通信流程

```
云台板                                        底盘板
──────                                       ──────
每 2ms:                                      每 2ms:
  ① 读取遥控器 → 生成 chassis_cmd             ① 接收 GimbalToChassis_cmd
  ② gimbal_func → 获取 yaw_ecd               ② 解码速度 + offset_angle
  ③ CalcOffsetAngle(yaw_ecd)                 ③ chassis_func (运动学解算)
  ④ 打包 → CAN ID 0x310 ──────────→          ④ handle_referee_data
  ⑤                            ←── CAN ID 0x311 ← 打包裁判数据
  ⑥ 接收 ChassisToGimbal_referee             ⑤ BoardComm_Send
     (用于 send_packet.mode, 视觉等)
```

### 4.5 板间断连保护

```c
// apps/sentry/chassis_board/robot_control.c:29-36
if (Module_BoardComm_Get_Offline_State() == STATE_OFFLINE) {
    // 云台板断连 → 底盘进入无力模式
    chassis_cmd.vx = 0; chassis_cmd.vy = 0; chassis_cmd.wz = 0;
    chassis_cmd.chassis_mode = chassis_zero_force;
}
```

---

## 5. 传感器与姿态估计 (INS)

### 5.1 传感器硬件一览

| 传感器 | 接口 | 数据 | 用途 |
|:--------|:-----|:-----|:-----|
| BMI088 | SPI | 3 轴加速度 + 3 轴陀螺仪 | INS 姿态估计原始输入 |
| WT606 | UART1 | YAW 角度 + Z 轴角速度 | 大云台 YAW 独立反馈源 |

### 5.2 INS 数据结构

```c
// modules/INS/module_ins.h:14-41
typedef struct {
    float q[4];                // 四元数 (w, x, y, z)
    float MotionAccel_b[3];    // 机体坐标系加速度
    float MotionAccel_n[3];    // 绝对系加速度
    float AccelLPF;            // 加速度低通滤波系数
    float xn[3], yn[3], zn[3]; // 机体坐标系在绝对系中的投影向量
    float euler_angle[3];      // 欧拉角 (roll, pitch, yaw) — 度
    float euler_rad[3];        // 欧拉角 — 弧度
    float YawTotalAngle_rad;   // 连续 YAW 角度 (支持多圈累计)
    int32_t YawRoundCount;     // YAW 圈数计数器
    float dt;                  // 采样时间间隔
    uint8_t initialized;       // 初始化完成标志
} Ins_t;
```

### 5.3 传感器反馈源的使用

不同电机根据需求使用不同的反馈源：

| 电机 | 角度反馈源 | 角速度反馈源 | 原因 |
|:-----|:-----------|:-------------|:-----|
| 小 YAW | `ins->YawTotalAngle_rad` | `bmi088_dev->gyro[2]` | BMI088 Z 轴陀螺 1kHz+ 高带宽 |
| 大 YAW | `wt606_device->YawTotalAngle_rad` | `wt606_device->gyro_rps[2]` | WT606 独立陀螺仪 (避免机械耦合) |
| PITCH | `ins->euler_rad[1]` | `bmi088_dev->gyro[0]` | BMI088 X 轴陀螺 |

```c
// gimbal_func.c:67-68 (小 YAW 外部反馈配置)
.other_angle_feedback_ptr = &ins->YawTotalAngle_rad,   // IMU 角度
.other_speed_feedback_ptr = &bmi088_dev->gyro[2],       // 陀螺仪角速度
.setting_init_config = {
    .angle_feedback_source = 1,  // 1 = 使用外部反馈 (非电机编码器)
    .speed_feedback_source = 1,  // 1 = 使用外部反馈
},
```

### 5.4 为什么使用外部传感器反馈而非电机编码器

| 维度 | 电机编码器 (ECD) | 外部 IMU |
|:------|:-----------------|:---------|
| 更新频率 | ~500Hz (CAN 帧率) | 1kHz+ (SPI 直读) |
| 测量对象 | 电机轴角度 | 负载实际姿态 |
| 机械误差 | 包含齿轮间隙 | 直接反映负载位置 |
| 适用场景 | 速度环 (驱动轮) | 角度环 (云台) |

---

## 6. 视觉系统通信

### 6.1 物理层

视觉计算机（NUC/Jetson）通过 **USB 虚拟串口 (CDC ACM)** 与云台板通信。

### 6.2 发送包 (MCU → 上位机)：四元数 + 模式

```c
// modules/VISION/module_vision.h:28-34
#pragma pack(1)
typedef struct {
    uint8_t header;  // 帧头 0xAA
    uint8_t mode;    // 敌方颜色 (0=红方, 1=蓝方)
    float   q[4];    // 四元数 (w, x, y, z) — 相机当前姿态
    uint8_t tail;    // 帧尾 0x55
} SendPacket;  // 总计 3 + 16 + 1 = 20 字节
```

mode 的计算：`mode = 1 - robot_color`（1=攻击红色，0=攻击蓝色 — 取决于敌方颜色）

### 6.3 接收包 (上位机 → MCU)：目标检测结果

```c
// modules/VISION/module_vision.h:37-48
typedef struct {
    uint8_t header;        // 帧头 0xBB
    uint8_t found;         // 是否检测到目标
    uint8_t fire_advice;   // 开火建议 (0=不开火, 1=开火)
    float   target_yaw;    // 目标偏航角 (rad)
    float   target_pitch;  // 目标俯仰角 (rad)
    float   vx;            // 目标 X 速度
    float   vy;            // 目标 Y 速度
    uint8_t nav_state;     // 导航状态
    uint8_t tail;          // 帧尾 0x5B
} ReceivePacket;  // 总计 3 + 1 + 1 + 12 + 1 + 1 = 19 字节
```

### 6.4 通信流程

```
云台板 (MCU)                             视觉计算机 (上位机)
───────────                              ──────────────────
每 2ms:
  ① 填充 SendPacket:
     - q[0..3] = ins->q (当前姿态四元数)
     - mode    = 敌方颜色
  ② Module_Vision_Send() ────────────→  ③ 接收四元数 → 更新相机姿态
                                      ④ 运行目标检测算法
                                      ⑤ 计算 target_yaw/pitch
  ⑦ Module_Vision_Receive() ←────────  ⑥ 发送 ReceivePacket
  ⑧ gimbal_func 使用 target_yaw/pitch
     驱动云台指向目标
```

### 6.5 帧同步机制

视觉模块内部维护状态机：
- **WAIT_HEADER**：等待 0xBB 帧头
- **WAIT_DATA**：接收固定长度数据
- **WAIT_TAIL**：校验 0x5B 帧尾

仅当帧头+帧尾均正确时，才认为接收到一个有效包。

---

## 7. 遥控器协议映射

### 7.1 支持的遥控器

| 协议 | 接口 | 说明 |
|:------|:-----|:-----|
| SBUS | UART3 | 标准 SBUS (100kbps, 8E2)，16 通道 |
| DT7 | UART3 | DJI DT7 遥控器 (115200bps) |
| VT02/VT03 | UART6 | 图传遥控器 |

通过 `REMOTE_SOURCE` 和 `REMOTE_VT_SOURCE` CMake 变量选择。

### 7.2 SBUS 通道值约定

```c
#define SBUS_CHX_BIAS  ((uint16_t)1024)   // 中位
#define SBUS_CHX_UP    ((uint16_t)240)    // 上拨
#define SBUS_CHX_DOWN  ((uint16_t)1807)   // 下拨
```

### 7.3 通道功能映射 (Sentry 云台板)

| 通道 | 类型 | 功能 | 逻辑 |
|:-----|:-----|:-----|:-----|
| CH1 | 摇杆 | VY 速度 | 前推 = 前进, 后拉 = 后退 |
| CH2 | 摇杆 | VX 速度 | 左推 = 左移, 右推 = 右移 |
| CH3 | 摇杆 | Pitch 微调 | 每次累计 ±0.001 rad |
| CH4 | 摇杆 | Yaw 微调 | 每次累计 ±0.001 rad |
| CH5 | 三段开关 | 发射模式 | UP=关机 / MID=开机不转摩擦轮 / DOWN=开机+摩擦轮 |
| CH6 | 三段开关 | 控制模式 | UP=陀螺仪 / MID=自动 / DOWN=无力 |
| CH7 | 三段开关 | 发射类型 | UP=单发 / MID=停止 / DOWN=连发 |
| CH8 | 三段开关 | 底盘模式 | UP=跟随云台 / MID=自旋 / DOWN=反向自旋 |

### 7.4 遥控器离线保护

```c
// robot_func.c:28-103
uint8_t state = Module_Remote_get_offline_status();

if (state & 0x01) {
    // RC 在线 → 正常处理
} else {
    // RC 离线 → 全部进入无力模式
    gimbal_mode = gimbal_zero_force;
    chassis_mode = chassis_zero_force;
    shoot_mode = shoot_off;
    memset(Chassis_Ctrl, 0, sizeof(*Chassis_Ctrl));
}
```

---

## 8. 云台自动搜索三态机

当视觉未发现目标时，云台进入自动搜索模式。搜索由 `auto_search` 字段控制，实现三态切换：

### 8.1 状态定义

| auto_search 值 | 状态 | 行为 | 触发条件 |
|:---------------|:-----|:-----|:---------|
| **0** | 锁定目标 | 大小 YAW 跟随视觉 `target_yaw/pitch` | 视觉系统检测到目标 |
| **1** | 主动搜索 | 小 YAW 匀速旋转 (1.2 rad/s) + Pitch 正弦摆动 (0.8Hz, ±0.3rad) | 视觉丢失目标 |
| **2** | 朝向正前方 | 小 YAW 归零 (朝向 `YawRoundCount * 360°`)，Pitch 归水平 | 特殊复位指令 |

### 8.2 状态切换逻辑

```
视觉检测到目标     →  auto_search = 0: 锁定模式
  记录 auto_yaw_offset = 当前目标 yaw
  重置 search_pitch_time = 0

视觉丢失目标       →  auto_search = 1: 搜索模式
  小 YAW 持续旋转:  search_yaw_angle  += 1.2 * 0.002 (rad/周期)
  Pitch 正弦摆动:   pitch_ref = -|0.3 * sin(2π * 0.8 * t)|
  实时更新 auto_yaw_offset = ins->YawTotalAngle_rad
```

### 8.3 源码实现

```c
// gimbal_func.c:199-225
if (gimbal_cmd->auto_search == 0) {
    // 目标已找到 → 精确跟踪
    Motor_DJI_SetRef(big_yaw_motor, big_yaw_offset);
    Motor_DJI_SetRef(small_yaw_motor, gimbal_cmd->yaw);
    Motor_DM_SetRef(pitch_motor, gimbal_cmd->pitch);
    search_yaw_angle  = gimbal_cmd->yaw;
    search_pitch_time = 0.0f;
    auto_yaw_offset   = gimbal_cmd->yaw;      // 记录当前偏移
}
else if (gimbal_cmd->auto_search == 1) {
    // 自动搜索 → 扫描
    search_yaw_angle += 1.2f * 0.002f;         // 1.2 rad/s 旋转
    search_pitch_time += 0.002f;
    float pitch_ref = 0.3f * sinf(2π * 0.8f * search_pitch_time);
    pitch_ref = fabsf(pitch_ref);              // 仅向下摆动
    Motor_DJI_SetRef(small_yaw_motor, search_yaw_angle);
    Motor_DM_SetRef(pitch_motor, -pitch_ref);
    auto_yaw_offset = ins->YawTotalAngle_rad;  // 实时更新
}
```

### 8.4 搜索轨迹可视化

```
PITCH
  │   ╱╲    ╱╲    ╱╲    ╱╲    正弦摆动, 仅有向下分量 (fabsf)
  │  ╱  ╲  ╱  ╲  ╱  ╲  ╱  ╲
  │ ╱    ╲╱    ╲╱    ╲╱    ╲
  └──────────────────────────────→ YAW
       匀速旋转 (1.2 rad/s)
```

---

## 9. 安全保护机制

哨兵系统实现了**三层安全保护**：

### 9.1 第一层：设备离线检测 (OFFLINE)

每个电机/传感器注册时创建离线检测器：

```c
// 电机注册时附带离线配置
.offline_init_config = {
    .name       = "6020_small",  // 设备名称 (蜂鸣器区分用)
    .timeout_ms = 100,           // 100ms 无心跳 → 判定离线
    .beep_times = 3,             // 离线时蜂鸣 3 声 (听声辨位)
    .enable     = 1,             // 启用离线管理
},
```

**工作原理**：
1. 每周期 `Module_Offline_device_update(dev)` 更新心跳时间戳
2. OFFLINE 任务 (1Hz) 检查所有注册设备：当前时间 - 心跳时间 > timeout_ms → 判定离线
3. 离线后触发蜂鸣器（不同 `beep_times` 区分不同设备）
4. 应用层通过 `Module_Offline_get_device_status(dev)` 检查状态 → 离线则停止该设备控制

### 9.2 第二层：PID 堵转保护

```c
// pid.c:107-121
// 判断条件: |ref - measure| / |ref| > 95% 且持续 500 个周期 (1秒)
if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f) {
    pid->ERRORHandler.ERRORCount++;
}
if (pid->ERRORHandler.ERRORCount > 500) {
    pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    return 0;  // 堵转 → 输出清零, 保护电机
}
```

### 9.3 第三层：板间断连保护

| 保护 | 检测方式 | 响应 |
|:------|:---------|:-----|
| 底盘板感知云台板离线 | `Module_BoardComm_Get_Offline_State()` | 进入 `chassis_zero_force`，所有电机停止 |
| 云台板感知底盘板离线 | 同 BOARDCOMM | 不发送新底盘指令 |
| 遥控器离线 | `Remote_get_offline_status()` | 所有控制模式切换为 `zero_force` |
| 视觉离线 | `Vision_Get_offline_state()` | 自动进入搜索模式 (auto_search=1) |

---

## 10. CMake 条件编译机制

### 10.1 生成链路

```
apps/config.cmake              定义 ROBOT + BOARD
      │
      ▼
apps/sentry/robot.cmake        声明 MODULES_GIMBAL / MODULES_CHASSIS 列表
      │
      ▼
config.cmake 中的 foreach      将模块名展开为 MODULE_XXX=0/1
      │
      ▼
generate_headers.cmake         写入 #define MODULE_XXX 1 → module_config.h
      │
      ▼
modules/module_init.c          #if MODULE_XXX 守卫初始化调用
modules/CMakeLists.txt         if(MODULE_XXX) 条件编译源文件
```

### 10.2 切换机器人/板型

只需要修改 `apps/config.cmake` 中的两行：

```cmake
set(ROBOT "sentry")  # 切换到: hero / infantry3 / engineer / drone ...
set(BOARD "gimbal")  # 切换到: single / gimbal / chassis
```

### 10.3 Sentry 模块参数覆盖

```cmake
# apps/sentry/robot.cmake — Sentry 特定参数覆盖默认值
set(OFFLINE_BEEP_ENABLE  0)           # 哨兵禁音 (避免暴露位置)
set(REMOTE_UART          huart3)      # 使用 UART3 接收 SBUS
set(REMOTE_VT_UART       huart6)      # UART6 接图传
set(REMOTE_SOURCE        1)           # 1 = SBUS
set(REMOTE_VT_SOURCE     0)           # 0 = 无图传遥控
set(BOARDCOMM_CAN        BSP_CAN_HANDLE2)  # CAN2 用于板间通信
```

---

## 11. 裁判系统协议透传

### 11.1 数据流

```
裁判系统 (PM01/图传模块)
      │ UART (115200bps)
      ▼
底盘板 REFEREE 模块
      │ 解析 CMD_ID_ROBOT_STATUS, CMD_ID_POWER_HEAT_DATA 等
      ▼
handle_referee_data()
      │ 打包为 ChassisToGimbal_referee_t (8 字节)
      ▼
CAN ID 0x311 → 云台板
```

### 11.2 裁判数据打包

```c
// chassis_board/robot_func/robot_func.c:5-26
void handle_referee_data(ChassisToGimbal_referee_t *referee_data)
{
    const robot_status_t    *robot_status    = Module_Referee_Get_cmd_data(CMD_ID_ROBOT_STATUS);
    const allowed_bullet_t  *allowed_bullet  = Module_Referee_Get_cmd_data(CMD_ID_ALLOWED_BULLET);
    const game_status_t     *game_status     = Module_Referee_Get_cmd_data(CMD_ID_GAME_STATUS);
    const power_heat_data_t *power_heat_data = Module_Referee_Get_cmd_data(CMD_ID_POWER_HEAT_DATA);

    // 红蓝识别: ID 1-7=红方, 101-107=蓝方
    referee_data->robot_color = (robot_status->robot_id >= 100) ? 1 : 0;
    referee_data->bullet_allow     = max(0, allowed_bullet->bullet_17mm_allowed);
    referee_data->current_hp       = robot_status->current_hp;
    referee_data->game_progress    = game_status->type_progress.game_progress;
    referee_data->shooter_heat_pct = power_heat_data->shooter_17mm_heat;
}
```

### 11.3 各字段用途

| 字段 | 用途 |
|:------|:-----|
| `robot_color` | 决定 `send_packet.mode`（攻击目标颜色） |
| `game_progress` | 比赛阶段 (4=比赛中, 用于 automode 状态判断) |
| `current_hp` | 血量信息 (可触发撤退逻辑) |
| `bullet_allow` | 剩余弹丸 (决定是否可发射) |
| `shooter_heat_pct` | 枪管热量 (发射机构冷却判断) |

---

## 12. BSP 硬件抽象层

### 12.1 BSP 层职责

BSP (Board Support Package) 将 STM32 HAL 封装为统一接口，使上层模块与具体芯片/板型解耦。

### 12.2 核心组件

| 组件 | 说明 | 关键接口 |
|:------|:-----|:---------|
| **DWT** | ARM Cortex-M 调试定时器 (数据观察点) | `BSP_DWT_GetDeltaT()` — 精确周期计数, 零中断开销 |
| **CAN** | CAN 总线收发封装 | `BSP_CAN_HANDLE1` (电机总线), `BSP_CAN_HANDLE2` (板间通信) |
| **UART** | 串口封装 (支持 DMA) | `huart1` (WT606), `huart3` (SBUS), `huart6` (图传/裁判) |
| **SPI** | SPI 接口 | BMI088 传感器通信 |
| **GPIO** | 通用 IO | LED, 蜂鸣器, 电源控制 |
| **USB CDC ACM** | 虚拟串口 | 视觉计算机通信 + SEGGER RTT 日志输出 |

### 12.3 DWT 精确计时

PID/LQR 的微分和积分项依赖精确时间间隔。DWT 使用 ARM Cortex-M 内置的 32 位周期计数器：

```c
// 优势:
// - 精度: 1 CPU 周期 (~5.9ns @ 168MHz)
// - 零开销: 不需要定时器中断
// - 无累积误差: 直接读硬件计数器
pid->dt = BSP_DWT_GetDeltaT(&pid->DWT_CNT);
```

### 12.4 CAN 总线分配

```
CAN1 (BSP_CAN_HANDLE1) — 电机总线:
  ├── 小 YAW (GM6020, ID:1)
  ├── PITCH (DM4310, ID:0x23)
  ├── 摩擦轮×2 (M3508, ID:1/2)
  ├── 拨盘 (M2006, ID:3)
  └── 转向轮×4 (GM6020, ID:1/2/3/4)

CAN2 (BSP_CAN_HANDLE2) — 底盘 + 板间:
  ├── 驱动轮×4 (M3508, ID:1/2/3/4)
  ├── 大 YAW (GM6020, ID:1)
  ├── 板间通信 (ID:0x310/0x311)
  └── 超电模块 (可选)
```

---

## 13. 概念关系图谱

### 13.1 系统架构全景图

```
┌────────────────────────────────────────────────────────────────┐
│                         RM_Sentry 知识体系                       │
│                                                                │
│  ┌─────────────┐  ┌─────────────┐  ┌──────────────────────┐   │
│  │ 控制算法详解  │  │ 功率控制详解  │  │   系统架构详解 (本文)   │   │
│  │ PID+LQR     │  │ Power Control│  │   架构+通信+RTOS      │   │
│  └──────┬──────┘  └──────┬──────┘  └───────────┬──────────┘   │
│         │                │                      │               │
│         ▼                ▼                      ▼               │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                    app/sentry/ 应用层                     │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │   │
│  │  │ gimbal_func  │  │ chassis_func │  │ robot_func   │   │   │
│  │  │ (云台控制)    │  │ (底盘控制)    │  │ (遥控器+裁判)  │   │   │
│  │  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘   │   │
│  └─────────┼─────────────────┼─────────────────┼───────────┘   │
│            │                 │                 │                │
│  ┌─────────▼─────────────────▼─────────────────▼───────────┐   │
│  │                  MODULE 模块层                           │   │
│  │  INS | BMI088 | WT606 | VISION | BOARDCOMM | REFEREE   │   │
│  │  MOTOR(DJI/DM/Servo) | OFFLINE | REMOTE | POWER       │   │
│  └─────────────────────────┬───────────────────────────────┘   │
│                            │                                    │
│  ┌─────────────────────────▼───────────────────────────────┐   │
│  │          BSP 层 (CAN / UART / SPI / GPIO / DWT)          │   │
│  └─────────────────────────┬───────────────────────────────┘   │
│                            │                                    │
│  ┌─────────────────────────▼───────────────────────────────┐   │
│  │       ThreadX RTOS + STM32 HAL + CMSIS-DSP               │   │
│  └──────────────────────────────────────────────────────────┘   │
└────────────────────────────────────────────────────────────────┘
```

### 13.2 数据流汇总

| 数据 | 来源 | 消费方 | 传输方式 | 频率 |
|:------|:-----|:-------|:---------|:-----|
| 四元数姿态 | INS (云台板) | 视觉计算机 | USB VCP | 500Hz |
| 目标角度 | 视觉计算机 | gimbal_func | USB VCP | 500Hz |
| 遥控器通道 | SBUS/UART3 | robot_func | 函数调用 | 500Hz |
| 底盘指令 | 云台板 | 底盘板 | CAN2 (ID 0x310) | 500Hz |
| 裁判数据 | 底盘板 REFEREE | 云台板 | CAN2 (ID 0x311) | 500Hz |
| 电机指令 | MOTOR 模块 | DJI/DM 电调 | CAN1/CAN2 | 500Hz |
| BMI088 数据 | SPI (DMA) | INS 任务 | 内存共享 | 1kHz |
| 离线状态 | OFFLINE 任务 | 各控制函数 | 函数调用 | 1Hz |

### 13.3 关键设计决策

| 决策 | 选择 | 原因 |
|:------|:-----|:-----|
| 双板 vs 单板 | 双板 | 11 电机 + 多传感器 → 单板 CAN 带宽不足 |
| CAN2 用于板间 vs CAN1 | CAN2 (独立总线) | 避免板间通信与电机控制争夺 CAN 带宽 |
| 外部 IMU vs 电机编码器 | 外部 IMU (角度环) | 1kHz+ 高带宽, 无齿轮间隙误差 |
| ThreadX vs FreeRTOS | ThreadX | 商业级安全认证, 更小的中断延迟 |
| LQR 为主 vs PID 为主 | LQR 为主 | 多状态反馈, 数学保证稳定性 |

---

## 相关笔记

- [[RM_Sentry_控制算法详解]] — PID 七大优化、LQR 一维/二维控制器、调参方法
- [[RM_Sentry_功率控制详解]] — 功率模型、混合权重分配、缓冲能量管理
- [[RM_ThreadX_RTOS教程]] — ThreadX 实时操作系统基础知识
- [[RM_BSP与模块教程]] — BSP 硬件抽象层与模块化设计
- [[RM_CMake教程]] — CMake 构建系统与条件编译
- [[MAS_Vision_学习指南]] — 视觉系统目标检测与通信协议
- [[RM_Socket编程详解]] — 串口/USB 通信协议基础

---

*文档基于 `mas_embedded_threadx/apps/sentry/` 及 `modules/` 源码编写*
*作者: laladuduqq (2807523947@qq.com)*
