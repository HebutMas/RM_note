# 哨兵应用层信号流图

> 以 2ms 控制周期为单位，展示信号从输入到输出经过的完整路径。哨兵是双板架构，云台板和底盘板各自独立运行。

## 哨兵云台板信号流

### 输入信号

| 信号源 | 来源 | 数据形式 |
|--------|------|---------|
| SBUS 遥控器 | UART5 DMA → SBUS 解码 → `g_remote_data.channels[0..15]` | 16 通道：CH1-4 摇杆（带符号），CH5-8 开关（原始值） |
| 视觉接收 | USB 虚拟串口 → `Module_Vision_Receive()` → `ReceivePacket *` | 目标偏移量 yaw/pitch + auto_search 标志 |
| IMU 姿态 | BMI088 → INS 解算 → `ins->YawTotalAngle_rad` / `ins->euler_rad[1]` / `ins->q[0..3]` | 云台 yaw/pitch 角度 + 四元数 |
| WT606 陀螺仪 | WT606 → `wt606_device->YawTotalAngle_rad` / `gyro_rps[2]` | 大 yaw 轴角度 + 角速度 |
| 板间通信接收 | CAN → `ChassisToGimbal_referee_t` | robot_color / hp / heat / bullet |
| 电机反馈 | CAN → 电机 measure | small_yaw/big_yaw 编码器值，pitch 角度 |

### 信号流

```
┌─────────────────────────────────────────────────────────────────────┐
│ 输入层                                                               │
│                                                                     │
│  SBUS 遥控器          视觉接收            IMU/WT606      板间通信     │
│  channels[0..15]      ReceivePacket      ins / wt606     referee_t  │
└──────┬──────────────────┬──────────────────┬──────────────┬─────────┘
       │                  │                  │              │
       ▼                  │                  │              │
┌──────────────────────────┐                 │              │
│ RemoteControlSet()       │                 │              │
│                          │                 │              │
│ CH1/CH2 → chassis_cmd    │                 │              │
│   .vx / .vy (归一化)     │                 │              │
│ CH3/CH4 → gimbal_cmd     │                 │              │
│   .yaw / .pitch (累加)   │                 │              │
│ CH6      → gimbal_mode   │                 │              │
│ CH8      → chassis_mode  │                 │              │
│ CH5/CH7  → shoot_cmd     │                 │              │
│   .shoot_mode/friction/  │                 │              │
│   load_mode              │                 │              │
└──────┬───────────────────┘                 │              │
       │                                     │              │
       │  ┌──────────────────────────────────┘              │
       │  │                                               │
       │  │  ┌─────────────────────────────────────────────┘
       │  │  │
       ▼  ▼  ▼
┌──────────────────────────────────────────────────────────────────────┐
│ 视觉收发                                                              │
│                                                                      │
│  SendPacket:                                                         │
│    mode = 1 - referee.robot_color  ← 板间通信取反                     │
│    q[0..3] = ins->q[0..3]          ← IMU 四元数                       │
│    → Module_Vision_Send()          → USB 发给上位机                   │
│                                                                      │
│  ReceivePacket:                                                      │
│    ← Module_Vision_Receive()        ← 上位机发来的目标偏移             │
│    → gimbal_cmd.yaw / .pitch / .auto_search  覆盖遥控器值              │
└──────────────────────────┬───────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────────────┐
│ gimbal_func(&gimbal_cmd, &yaw_ecd)                                   │
│                                                                      │
│  离线检查：3 电机全在线才控制                                         │
│                                                                      │
│  pitch 重力前馈：                                                     │
│    ins->euler_rad[1] → ff_gravity = K*angle + gamma                  │
│    → pitch_motor->base.controller.feedforward_torque                 │
│                                                                      │
│  大 yaw 跟随小 yaw：                                                  │
│    big_yaw_offset = wt606.YawTotalAngle_rad                         │
│                   + (small_yaw.ecd - ALIGN_ECD) * (2π/8192)          │
│                                                                      │
│  按模式设目标：                                                       │
│    gyro_mode:                                                         │
│      small_yaw.SetRef(cmd.yaw + auto_offset)  ← 遥控器/视觉          │
│      big_yaw.SetRef(big_yaw_offset)           ← 跟随小 yaw           │
│      pitch.SetRef(cmd.pitch * DEG_2_RAD)                             │
│    auto_mode (search=0):                                              │
│      small_yaw.SetRef(cmd.yaw)                ← 视觉目标              │
│      big_yaw.SetRef(big_yaw_offset)                                   │
│      pitch.SetRef(cmd.pitch)                                          │
│    auto_mode (search=1):                                              │
│      small_yaw.SetRef(累积旋转角度)            ← 自动搜索              │
│      pitch.SetRef(正弦摆动)                                           │
│                                                                      │
│  yaw_ecd = big_yaw.measure.ecd  ← 反馈给底盘跟随                      │
│                                                                      │
│  输出 → motor 链表（LQR 算 control.output）                          │
└──────────────────────────┬───────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────────────┐
│ shoot_func(&shoot_cmd)                                               │
│                                                                      │
│  离线检查：3 电机全在线才控制                                         │
│                                                                      │
│  shoot_on + friction_on:                                              │
│    friction_l.SetRef(+6800 RPM)  ← 定速旋转                          │
│    friction_r.SetRef(-6800 RPM)                                      │
│    load_stop      → loader.SetRef(0)                                  │
│    load_1_bullet  → loader.SetRef(-4000 RPM)                         │
│    load_burstfire → loader.SetRef(-8000 RPM)                         │
│                                                                      │
│  输出 → motor 链表（LQR 速度环）                                     │
└──────────────────────────┬───────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────────────┐
│ 板间通信发送                                                          │
│                                                                      │
│  GimbalToChassis_cmd_t:                                              │
│    vx = (int8_t)(chassis_cmd.vx * 10)    ← 归一化→压缩 int8          │
│    vy = (int8_t)(chassis_cmd.vy * 10)                                │
│    wz = (int8_t)(chassis_cmd.wz * 10)                                │
│    offset_angle = CalcOffsetAngle(yaw_ecd) ← 编码器偏差              │
│    chassis_mode = chassis_cmd.chassis_mode                           │
│    → Module_BoardComm_Send() → CAN → 底盘板                          │
└──────────────────────────────────────────────────────────────────────┘
```

### 输出信号

| 输出 | 目标 | 经过 |
|------|------|------|
| small_yaw 控制值 | GM6020 CAN1 | LQR 角度环 → `Motor_DJI_SetRef` → motor 链表 → `Motor_DJI_Flush` |
| big_yaw 控制值 | GM6020 CAN2 | 同上 |
| pitch 控制值 | DM4310 CAN1 | LQR 角度环 + 重力前馈 → `Motor_DM_ControlAndSend`（直接发 CAN，不走 Flush） |
| friction_l/r 控制值 | M3508 CAN1 | LQR 速度环 → motor 链表 → Flush |
| loader 控制值 | M2006 CAN1 | 同上 |
| 底盘命令 | CAN 板间通信 | `Module_BoardComm_Send` → 底盘板 |
| 视觉数据 | USB 虚拟串口 | `Module_Vision_Send` → 上位机 |

---

## 哨兵底盘板信号流

### 输入信号

| 信号源 | 来源 | 数据形式 |
|--------|------|---------|
| 板间通信接收 | CAN → `GimbalToChassis_cmd_t` | vx/vy/wz（int8 压缩）+ offset_angle + chassis_mode |
| 裁判系统 | UART → `Module_Referee_Get_cmd_data()` | robot_id / hp / heat / bullet / game_progress |
| 电机反馈 | CAN → 电机 measure | 8 个电机的编码器/转速/电流 |

### 信号流

```
┌─────────────────────────────────────────────────────────────────────┐
│ 输入层                                                               │
│                                                                     │
│  板间通信接收              裁判系统                                   │
│  GimbalToChassis_cmd_t    robot_status / game_status / ...          │
└──────┬──────────────────────────┬───────────────────────────────────┘
       │                          │
       ▼                          │
┌──────────────────────────────────┐                                   │
│ 板间通信解码                      │                                   │
│                                  │                                   │
│ 离线？                           │                                   │
│  ├─ 是 → 全部归零, 安全停车      │                                   │
│  └─ 否 → 解码:                   │                                   │
│         vx = recv.vx / 10 * MAX  │                                   │
│         vy = recv.vy / 10 * MAX  │                                   │
│         wz = recv.wz / 10 * MAX  │                                   │
│         offset_angle =           │                                   │
│           recv.offset_angle      │                                   │
│           * 360 / 8191           │                                   │
│         chassis_mode = recv.mode │                                   │
└──────┬───────────────────────────┘                                   │
       │                                                               │
       │                    ┌──────────────────────────────────────────┘
       │                    │
       ▼                    ▼
┌──────────────────────────────────────────────────────────────────────┐
│ handle_referee_data(&chassis_send_data)                              │
│                                                                      │
│  robot_id >= 100 → robot_color = 1 (蓝)                              │
│  robot_id <  100 → robot_color = 0 (红)                              │
│  bullet_allow / current_hp / game_progress / shooter_heat_pct        │
│  → 打包成 ChassisToGimbal_referee_t                                  │
│  → Module_BoardComm_Send() → CAN → 云台板                            │
└──────────────────────────────────────────────────────────────────────┘
       │
       ▼
┌──────────────────────────────────────────────────────────────────────┐
│ chassis_func(&chassis_cmd)                                           │
│                                                                      │
│  离线检查：8 电机全在线才控制                                        │
│                                                                      │
│  按 chassis_mode 设 wz:                                              │
│    rotate          → wz = 3                                         │
│    rotate_reverse  → wz = -8                                        │
│    follow_gimbal   → PID(offset_angle, 0) → wz                      │
│    automode        → 比赛中才动, 否则归零                            │
│                                                                      │
│  云台系→底盘系坐标变换:                                               │
│    θ = offset_angle * DEG_2_RAD                                      │
│    chassis_vx = vx*cosθ - vy*sinθ                                   │
│    chassis_vy = vx*sinθ + vy*cosθ                                   │
│                                                                      │
│  逆运动学: Chassis_Swerve_Calc(motors, config, vx, vy, wz)          │
│    → 4 个 M3508 驱动轮目标转速 (ref)                                 │
│    → 4 个 GM6020 舵向轮目标角度 (ref)                                │
│                                                                      │
│  输出 → motor 链表（LQR 控制 → 功率控制 → Flush）                    │
└──────────────────────────────────────────────────────────────────────┘
```

### 输出信号

| 输出 | 目标 | 经过 |
|------|------|------|
| 4×M3508 驱动轮控制值 | CAN2 | LQR 速度环 → 功率控制 → `Motor_DJI_Flush` |
| 4×GM6020 舵向轮控制值 | CAN1 | LQR 角度环 → 功率控制 → Flush |
| 裁判系统数据 | CAN 板间通信 | `Module_BoardComm_Send` → 云台板 |

### 双板协作关系

```
云台板                                    底盘板
  │                                         │
  │  GimbalToChassis_cmd_t                  │
  │  (vx/vy/wz/mode/angle)                  │
  ├──────────────CAN──────────────────────→ │ 解码 → chassis_func → 8 电机
  │                                         │
  │                                         │  ChassisToGimbal_referee_t
  │                                         │  (color/hp/heat/bullet)
  │ ←─────────────CAN───────────────────────┤  handle_referee_data
  │                                         │
  │  mode = 1 - robot_color                 │
  │  → SendPacket → USB → 上位机            │
```
