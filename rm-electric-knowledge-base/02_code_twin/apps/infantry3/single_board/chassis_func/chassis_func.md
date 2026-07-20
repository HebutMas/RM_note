# chassis_func — 步兵3号底盘控制

`apps/infantry3/single_board/chassis/chassis_func.c`

## 一句话

4 电机麦轮底盘：4 个 M3508，麦轮逆运动学分解底盘速度。带功率控制。

## 初始化：chassis_init

### 电机注册

| 电机     | 型号    | CAN  | tx_id | `motor_reverse_flag` | 角色    | 减速比  |
| ------ | ----- | ---- | ----- | -------------------- | ----- | ---- |
| [0] LF | M3508 | CAN1 | 1     | 0                    | DRIVE | 16:1 |
| [1] LB | M3508 | CAN1 | 2     | 0                    | DRIVE | 16:1 |
| [2] RB | M3508 | CAN1 | 3     | **1**                | DRIVE | 16:1 |
| [3] RF | M3508 | CAN1 | 4     | **1**                | DRIVE | 16:1 |

全部注册到功率控制（DRIVE 角色），功率上限 120W。与哨兵底盘的区别：4 个驱动轮（哨兵 4 驱动 + 4 舵向 = 8 个），用麦轮逆运动学（哨兵用舵轮）。

### 麦轮几何配置

```c
static const Chassis_Diff_Config_s chassis_diff_config = {
    .decele_ratio = 16.0f,
    .wheel_base_x = 0.5f,   // 前后轮距
    .wheel_base_y = 0.3f,   // 左右轮距
    .wheel_radius = 0.075f,  // 轮子半径
};
```

### 控制器配置

4 个 M3508 全部相同配置：

| 配置项 | 值 | 含义 |
|--------|-----|------|
| `loop_type` | `SPEED_LOOP` | 速度闭环（无位置环） |
| `algorithm_type` | `CONTROL_LQR` | LQR 控制 |
| `state_dim` | 1 | 单状态（仅角速度） |
| `K[0]` | 0.008 | 角速度增益 |
| `feedback_reverse_flag` | 0 | 反馈不取反（用电机自身编码器） |
| `angle_feedback_source` | 0 | 电机自身角度反馈 |
| `speed_feedback_source` | 0 | 电机自身速度反馈 |

> `state_dim=1` 的 LQR 退化为纯比例控制：`output = -K0 * (measure - ref)`，即 `output = K0 * (ref - measure)`。M3508 只需要控速度，不需要位置环，所以用单状态 LQR 足够。

---

## 电机取反逻辑

### 为什么 `motor_reverse_flag` 是 `[0, 0, 1, 1]`

麦轮底盘的 4 个电机左右对称安装。左侧两个电机（LF、LB）和右侧两个电机（RB、RF）的安装方向相反——电机正转时，左侧轮子往前滚，右侧轮子往后滚（或反过来，取决于安装）。

```
         Front (前)
     ┌───────────┐
     │ 0       3 │   0: 左前 LF  reverse=0
     │           │   1: 左后 LB  reverse=0
     │ 1       2 │   2: 右后 RB  reverse=1  ← 安装方向相反
     └───────────┘   3: 右前 RF  reverse=1  ← 安装方向相反
         Back  (后)
```

`motor_reverse_flag` 的作用点在 `CalculateLQROutput()` 中：

```c
ref = motor->base.controller.ref;
if (motor->base.setting.motor_reverse_flag == 1) ref *= -1;  // ← 右侧两个电机在这里取反
```

运动学算出 `wheel_speed[4]` 后，`Motor_DJI_SetRef` 把值写入 `controller.ref`。右侧电机（[2]、[3]）的 ref 在进入 LQR 计算前被乘 -1。

**效果**：运动学层面不需要关心单个电机的安装方向——`Chassis_Mecanum_Calc` 统一算出 4 个轮速，"哪侧需要翻转"由电机层自己处理。抽象出黑盒子，明确 wrapper 的运动逻辑"。

### `motor_reverse_flag` vs `feedback_reverse_flag`

| 对比项  | `motor_reverse_flag`    | `feedback_reverse_flag`    |
| ---- | ----------------------- | -------------------------- |
| 改的是  | ref（目标/执行侧）             | measure（传感器反馈侧）            |
| 代码位置 | `CalculateLQROutput` 开头 | `CalculateLQROutput` 中段    |
| 底盘用途 | 标定电机安装方向 `[0,0,1,1]`    | 全 0（用电机自身编码器，方向一致）         |
| 云台用途 | 一般为 0                   | P 轴用 INS 时设 1（INS 方向与电机相反） |

底盘 4 个电机的 `feedback_reverse_flag` 全是 0，因为它们用电机自身编码器做速度反馈，反馈方向和电机执行方向天然一致。只有在用外部传感器（如 INS、外部编码器）做反馈时，才可能出现反馈方向与电机方向相反的情况——这时候才需要 `feedback_reverse_flag`。

> 详见 [[02_code_twin/apps/infantry3/single_board/gimbal_func/gimbal_func#pitch 的 feedback_reverse_flag = 1：把反馈掰到电机轴]] 中关于 P 轴 `feedback_reverse_flag = 1` 的推导。

### 完整的取反链路（底盘 M3508）

```
Chassis_Mecanum_Calc
  │
  ├── wheel_speed[i] = (运动学公式)         ← 底盘坐标系下的轮速
  │
  └── Motor_DJI_SetRef(motors[i], wheel_speed[i])
        │
        └── controller.ref = wheel_speed[i]
              │
              └── dji_control → CalculateLQROutput:
                    │
                    ├── if (motor_reverse_flag == 1) ref *= -1   ← 右侧电机取反
                    │
                    ├── rad_speed = motor->base.measure.speed_rad (feedback_reverse_flag=0, 不取反)
                    │
                    ├── output = -K0 * (rad_speed - ref)          ← LQR 计算
                    │           = K0 * (ref - rad_speed)
                    │
                    ├── torque = output + feedforward_torque     ← 前馈（当前为 0）
                    │
                    └── output → currentToInteger → CAN 报文      ← 力矩映射到电流
```

> 注意：`motor_reverse_flag` 取反 ref 后，LQR 算出的 output（力矩）方向也跟着翻了。最终 CAN 发送的电流值符号反了，电机实际转向就反了。**效果等同于把电机整体坐标系取反**。

---

## 舵轮的 `drct_factor`（哨兵底盘，对比参考）

舵轮底盘有一个麦轮没有的优化——**避免转 180 度**。这个逻辑在 `Chassis_Swerve_Calc` 中，通过 `drct_factor` 实现：

```c
float diff = target_abs_angle - current_single;  // 目标角度与当前角度的差
AngleLoop_f(&diff, TWO_PI);

if (diff > HALF_PI)          // 差超过 +90°
{
    target_steer_rad = target_abs_angle - PI;  // 转向角减 180°
    drct_factor = -1;                           // 驱动方向翻转
}
else if (diff < -HALF_PI)    // 差超过 -90°
{
    target_steer_rad = target_abs_angle + PI;  // 转向角加 180°
    drct_factor = -1;                           // 驱动方向翻转
}
else                          // 差在 ±90° 以内
{
    target_steer_rad = target_abs_angle;       // 正常转向
    // drct_factor 保持 1
}

Motor_DJI_SetRef(motors[i], target_speed_rad * (float)drct_factor);
```

**原理**：舵轮需要转到目标角度才能产生正确的驱动力方向。如果目标角度和当前角度差超过 90 度，与其让舵向电机转将近 180 度，不如：

1. 把目标转向角翻转 180 度（转到反方向）
2. 同时把驱动电机速度取反（`drct_factor = -1`）

这样轮子滚动的物理方向和原来一样，但舵向电机只需要转不到 90 度。

### `drct_factor` 与 `motor_reverse_flag` 的区别

两者都在 `SetRef` 的值上叠加取反，但层面完全不同：

| | `drct_factor` | `motor_reverse_flag` |
|---|---|---|
| 代码位置 | `chassis_type.c`（运动学层） | `motor_dji.c`（电机驱动层） |
| 触发条件 | 舵轮目标角与当前角差 > 90° | 电机安装方向与坐标系相反 |
| 是否动态 | 是，每个控制周期可能变化 | 否，初始化时固定 |
| 影响哪个电机 | 驱动轮（M3508）的 ref | 所有支持该 flag 的电机 |
| 叠加关系 | `ref = target_speed * drct_factor` | `ref *= -1`（在 LQR 计算前） |

两者可以同时作用于同一个电机：运动学层先乘 `drct_factor`，电机驱动层再根据 `motor_reverse_flag` 决定是否取反。两者不冲突，因为它们解决的是不同层面的问题。

> 麦轮底盘没有 `drct_factor`，因为麦轮的辊子方向是固定的，不存在"转向"问题。`Chassis_Mecanum_Calc` 直接算出 4 个轮速，写入 ref 即可。

---

## 2ms 循环：chassis_func

逻辑与哨兵底盘相同：离线检查 → 模式选择 wz → 坐标变换 → 逆运动学。

### 离线检查

4 个电机全部在线才允许控制；任何一个离线则全部 Stop。

### 底盘模式

| 模式 | wz 来源 | 说明 |
|------|---------|------|
| `chassis_zero_force` | — | 全部 Stop |
| `chassis_rotate` | 固定 3 | 正向自旋 |
| `chassis_rotate_reverse` | 固定 -8 | 反向自旋 |
| `chassis_follow_gimbal_yaw` | 跟随 PID 输出 | 跟随云台 yaw |

> 与哨兵相比缺少 `chassis_automode`（步兵没有导航）。

### 坐标变换

```c
float total_angle_rad = chassis_cmd->offset_angle * DEGREE_2_RAD;
cos_theta = arm_cos_f32(total_angle_rad);
sin_theta = arm_sin_f32(total_angle_rad);
chassis_vx = cmd.vx * cos_theta - cmd.vy * sin_theta;
chassis_vy = cmd.vx * sin_theta + cmd.vy * cos_theta;
```

`offset_angle` 是云台和底盘的相对角度差。遥控器输入的速度是云台坐标系下的（"前进"= 云台指向方向），需要旋转到底盘坐标系。

### 逆运动学

`Chassis_Mecanum_Calc(motors, config, vx, vy, vw)` 把底盘速度向量分解为 4 个轮子的目标速度（rad/s），直接写入 `Motor_DJI_SetRef`。算法实现在 `modules/algorithm/chassis_type.c`，公式推导见 [[01_extracted/algorithm/chassis-kinematics]]。

```
         Front
     ┌───────────┐
     │ ╲       ╱ │    LF(0): ╲ 辊子 +45°
     │   ╲   ╱   │    RF(3): ╱ 辊子 -45°
     │   ╱   ╲   │    LB(1): ╱ 辊子 -45°
     │ ╱       ╲ │    RB(2): ╲ 辊子 +45°
     └───────────┘
         Back

ω_LF = (+vx - vy - ω·L) / R × decele_ratio    [辊子 +45°]
ω_RF = (+vx + vy + ω·L) / R × decele_ratio    [辊子 -45°]
ω_LB = (+vx + vy - ω·L) / R × decele_ratio    [辊子 -45°]
ω_RB = (+vx - vy + ω·L) / R × decele_ratio    [辊子 +45°]

其中 L = (wheel_base_x + wheel_base_y) / 2
```

| | 步兵3号 | 哨兵 |
|---|---|---|
| 逆运动学 | `Chassis_Mecanum_Calc`（麦轮） | `Chassis_Swerve_Calc`（舵轮） |
| 电机数 | 4 | 8 |
| 舵向轮 | 无 | 4×GM6020 |
| drct_factor | 无 | 有（避免转 180°） |

---

## ref → CAN 报文的完整链路

以 motors[0]（LF, `motor_reverse_flag=0`）为例，一个控制周期内从 ref 到 CAN 报文的完整流程：

```
chassis_func()
  │
  └── Chassis_Mecanum_Calc → Motor_DJI_SetRef(motors[0], wheel_speed[0])
        │
        └── controller.ref = wheel_speed[0]    ← 目标速度写入
              │
              └── motor_task_entry (ThreadX, 2ms 周期)
                    │
                    ├── Motor_UpdateAll()       ← 遍历全局电机链表
                    │     │
                    │     └── dji_control(base)
                    │           │
                    │           ├── 离线检查 → 离线则清零缓冲区 + 清 PID 积分
                    │           │
                    │           ├── CalculateLQROutput(motor)
                    │           │     │
                    │           │     ├── ref = controller.ref
                    │           │     ├── if (motor_reverse_flag==1) ref *= -1   ← motors[0] 不取反
                    │           │     ├── rad_speed = measure.speed_rad           ← feedback_reverse_flag=0, 不取反
                    │           │     └── output = -K0 * (rad_speed - ref)
                    │           │               = 0.008 * (ref - rad_speed)       ← LQR 输出（力矩 Nm）
                    │           │
                    │           ├── torque = output + feedforward_torque          ← 前馈为 0
                    │           ├── VAL_LIMIT(torque, -5.32, 5.32)               ← max_torque 限幅
                    │           │
                    │           ├── currentToInteger(-20, 20, -16384, 16384,
                    │           │     torque / (torque_constant * gear_ratio))   ← 力矩→电流→整数
                    │           │     = torque / (0.016 * 16)
                    │           │
                    │           └── 填充 sender_assignment[group].data[2*num]    ← 写入 CAN 缓冲区（不发送）
                    │
                    └── Motor_DJI_Flush()        ← 遍历所有分组，一次性发送
                          │
                          └── BSP_CAN_SendMessage(&sender_assignment[i])
```

### 力矩到电流的映射

M3508 的映射参数：

| 参数 | 值 | 来源 |
|------|-----|------|
| 物理电流范围 | ±20 A | [[01_extracted/motor/motor_params#M3508]] |
| 整数范围 | ±16384 | CAN 协议 |
| `torque_constant` | 0.016 N·m/A | 初始化配置 |
| `gear_ratio` | 16 | 初始化配置（注意：M3508 实际减速比 3591/187≈19.2，这里用 16 可能是轮组等效减速比） |
| `max_torque` | 5.32 N·m | 初始化配置（限幅用） |

> `torque_constant * gear_ratio = 0.016 * 16 = 0.256`，这个值把力矩（Nm）换算成电流（A），再由 `currentToInteger` 映射到 CAN 整数值。参数来源见 [[01_extracted/motor/motor_params#特征参数]]。

---

## 链接

- 麦轮运动学公式：[[01_extracted/algorithm/chassis-kinematics]]（待建）
- 云台反馈取反：[[02_code_twin/apps/infantry3/single_board/gimbal_func/gimbal_func#pitch 的 feedback_reverse_flag = 1：把反馈掰到电机轴]]
- DJI 电机驱动：[[02_code_twin/modules/MOTOR/DJI/motor_dji]]
- 电机基类：[[02_code_twin/modules/MOTOR/motor_base]]
- 功率控制：[[02_code_twin/modules/MOTOR/POWER_CONTROL/power_control]]
- 应用层调用：[[02_code_twin/apps/infantry3/single_board/robot_control]]
- 哨兵对照：[[02_code_twin/apps/sentry/chassis_board/chassis_func/chassis_func]]
