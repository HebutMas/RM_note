# 步兵3号应用层信号流图

> 以 2ms 控制周期为单位，展示信号从输入到输出经过的完整路径。步兵3号是单板架构，所有功能在一块板子上，无板间通信。

## 输入信号

| 信号源 | 来源 | 数据形式 |
|--------|------|---------|
| DT7 遥控器 | UART5 DMA → DT7 解码 → `g_remote_data` | sw1/sw2 开关 + CH1-4 摇杆 + wheel 滚轮 |
| 视觉接收 | USB 虚拟串口 → `Module_Vision_Receive()` → `ReceivePacket *` | 目标偏移量 yaw/pitch |
| 裁判系统 | UART → `Module_Referee_Get_cmd_data()` | robot_id（用于判断红蓝方） |
| IMU 姿态 | BMI088 → INS 解算 → `ins->YawTotalAngle_rad` / `ins->euler_rad[1]` / `ins->q[0..3]` | 云台 yaw/pitch 角度 + 四元数 |
| 电机反馈 | CAN → 电机 measure | yaw 编码器值，各电机转速/电流 |

## 信号流

```
┌─────────────────────────────────────────────────────────────────────┐
│ 输入层                                                               │
│                                                                     │
│  DT7 遥控器            视觉接收            IMU            裁判系统   │
│  sw1/sw2/CH1-4/wheel   ReceivePacket      ins             robot_id  │
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
│ sw2      → chassis_mode  │                 │              │
│   (上:自旋 中:跟随 下:反)│                 │              │
│ sw1      → gimbal_mode   │                 │              │
│   + shoot_cmd            │                 │              │
│   (上:摩擦轮 中:拨盘 下:停)│                │              │
│ wheel    → load_mode     │                 │              │
│   (0:停 正:反 负:连发)   │                 │              │
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
│    robot_id >= 100 → mode = 0 (蓝方)        ← 直接读裁判系统         │
│    robot_id <  100 → mode = 1 (红方)                                 │
│    q[0..3] = ins->q[0..3]                    ← IMU 四元数             │
│    → Module_Vision_Send()                    → USB 发给上位机         │
│                                                                      │
│  ReceivePacket:                                                      │
│    ← Module_Vision_Receive()                ← 上位机发来的目标偏移     │
│    → gimbal_cmd.yaw / .pitch                ← 覆盖遥控器值            │
│    (步兵3号 gimbal_auto_mode 未实现，接收数据暂不使用)                 │
└──────────────────────────┬───────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────────────┐
│ gimbal_func(&gimbal_cmd, &yaw_ecd)                                   │
│                                                                      │
│  离线检查：2 电机全在线才控制                                         │
│                                                                      │
│  按模式设目标：                                                       │
│    gyro_mode:                                                         │
│      yaw.SetRef(cmd.yaw * DEG_2_RAD)          ← 遥控器微调            │
│      pitch.SetRef(cmd.pitch * DEG_2_RAD)                             │
│    auto_mode:                                                         │
│      (未实现, break)                                                  │
│    zero_force:                                                        │
│      Stop                                                             │
│                                                                      │
│  yaw_ecd = yaw_motor.measure.ecd  ← 反馈给底盘跟随                    │
│                                                                      │
│  输出 → motor 链表（LQR 角度环算 control.output）                    │
└──────────────────────────┬───────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────────────┐
│ shoot_func(&shoot_cmd)                                               │
│                                                                      │
│  离线检查：3 电机全在线才控制                                         │
│                                                                      │
│  shoot_on + friction_on:                                              │
│    friction_l.SetRef(-6800 RPM)  ← 定速旋转 (与哨兵方向相反)         │
│    friction_r.SetRef(+6800 RPM)                                      │
│    load_stop      → loader.SetRef(0)                                  │
│    load_1_bullet  → loader.SetRef(-4000 RPM)                         │
│    load_burstfire → loader.SetRef(-8000 RPM)                         │
│  shoot_on + friction_off:                                             │
│    friction 归零, loader 按滚轮                                       │
│                                                                      │
│  输出 → motor 链表（LQR 速度环）                                     │
└──────────────────────────┬───────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────────────────┐
│ chassis_func(&chassis_cmd)                                           │
│                                                                      │
│  补全 chassis_cmd:                                                    │
│    offset_angle = CalcOffsetAngle(yaw_ecd) * 360 / 8191  ← 编码器→角度│
│    vx = vx * MAX_SPEED  ← 归一化→m/s                                 │
│    vy = vy * MAX_SPEED                                               │
│    wz = wz * MAX_SPEED                                               │
│                                                                      │
│  离线检查：4 电机全在线才控制                                         │
│                                                                      │
│  按 chassis_mode 设 wz:                                              │
│    rotate          → wz = 3                                         │
│    rotate_reverse  → wz = -8                                        │
│    follow_gimbal   → PID(offset_angle, 0) → wz                      │
│                                                                      │
│  云台系→底盘系坐标变换:                                               │
│    θ = offset_angle * DEG_2_RAD                                      │
│    chassis_vx = vx*cosθ - vy*sinθ                                   │
│    chassis_vy = vx*sinθ + vy*cosθ                                   │
│                                                                      │
│  逆运动学: Chassis_Mecanum_Calc(motors, config, vx, vy, wz)         │
│    → 4 个 M3508 目标转速 (ref)                                       │
│                                                                      │
│  输出 → motor 链表（LQR 速度环 → 功率控制 → Flush）                  │
└──────────────────────────────────────────────────────────────────────┘
```

## 输出信号

| 输出 | 目标 | 经过 |
|------|------|------|
| yaw 控制值 | GM6020 CAN1 | LQR 角度环 → `Motor_DJI_SetRef` → motor 链表 → `Motor_DJI_Flush` |
| pitch 控制值 | DM4310 CAN2 | LQR 角度环 → `Motor_DM_ControlAndSend`（直接发 CAN，不走 Flush） |
| friction_l/r 控制值 | M3508 CAN2 | LQR 速度环 → motor 链表 → Flush |
| loader 控制值 | M2006 CAN1 | 同上 |
| 4×底盘轮控制值 | M3508 CAN1 | LQR 速度环 → 功率控制 → Flush |
| 视觉数据 | USB 虚拟串口 | `Module_Vision_Send` → 上位机 |

## 与哨兵信号流的对比

两套架构的核心差异在于**下盘系统信号的读取方式**和**底盘控制的部署位置**。

### 裁判系统与视觉 mode 来源

| 架构 | 方式 |
|------|------|
| **单板**（步兵3号） | 本板直接读裁判系统，取 `robot_id` 作为视觉 mode 来源 |
| **双板**（哨兵） | 底盘板读取裁判系统后，打包通过板间 CAN 回传云台板；`robot_color` 取反后用于视觉 mode |

### 底盘控制

| 架构 | 方式 |
|------|------|
| **单板** | 同一块板直接调用 `chassis_func`，麦轮 4×M3508，`Chassis_Mecanum_Calc` |
| **双板** | 云台板打包控制指令通过板间 CAN 发送，底盘板接收后解码调用 `chassis_func`，舵轮 4×M3508 + 4×GM6020，`Chassis_Swerve_Calc` |
