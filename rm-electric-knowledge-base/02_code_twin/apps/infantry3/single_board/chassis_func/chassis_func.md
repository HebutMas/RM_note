# chassis_func — 步兵3号底盘控制

`apps/infantry3/single_board/chassis/chassis_func.c`

## 一句话

4 电机麦轮底盘：4 个 M3508，调用 `Chassis_Mecanum_Calc` 逆运动学分解底盘速度。带功率控制。

---

## 初始化：chassis_init

### 电机注册

| 电机     | 型号    | CAN  | tx_id | `motor_reverse_flag` | 角色    | 减速比  |
| ------ | ----- | ---- | ----- | -------------------- | ----- | ---- |
| [0] LF | M3508 | CAN1 | 1     | 0                    | DRIVE | 16:1 |
| [1] LB | M3508 | CAN1 | 2     | 0                    | DRIVE | 16:1 |
| [2] RB | M3508 | CAN1 | 3     | **1**                | DRIVE | 16:1 |
| [3] RF | M3508 | CAN1 | 4     | **1**                | DRIVE | 16:1 |

全部注册到功率控制（DRIVE 角色），功率上限 120W。

- DJI 电机驱动：[[02_code_twin/modules/MOTOR/DJI/motor_dji]]

### 几何配置

```c
static const Chassis_Diff_Config_s chassis_diff_config = {
    .decele_ratio = 16.0f,
    .wheel_base_x = 0.5f,   // 前后轮距 (m)
    .wheel_base_y = 0.3f,   // 左右轮距 (m)
    .wheel_radius = 0.075f,  // 轮子半径 (m)
};
```

`Chassis_Diff_Config_s` 定义见 [[02_code_twin/modules/algorithm/chassis_type]]。

### 控制器配置

4 个 M3508 全部相同配置：

| 配置项                     | 值             | 含义              |
| ----------------------- | ------------- | --------------- |
| `loop_type`             | `SPEED_LOOP`  | 速度闭环（无位置环）      |
| `algorithm_type`        | `CONTROL_LQR` | LQR 控制          |
| `state_dim`             | 1             | 单状态（仅角速度）       |
| `K[0]`                  | 0.008         | 角速度增益           |
| `feedback_reverse_flag` | 0             | 反馈不取反（用电机自身编码器） |
| `angle_feedback_source` | 0             | 电机自身角度反馈        |
| `speed_feedback_source` | 0             | 电机自身速度反馈        |

> `state_dim=1` 的 LQR 退化为纯比例控制：`output = -K0 * (measure - ref)` = `K0 * (ref - measure)`。

---

## 电机取反

### 为什么 `motor_reverse_flag` 是 `[0, 0, 1, 1]`

麦轮底盘的 4 个电机左右对称安装，左侧和右侧电机的安装方向相反。

**如果不加取反，同样给正速度指令时：**

```
         Front
     ┌───────────┐
     │ ↻      ↺ │   LF 顺时针 (↻)    RF 逆时针 (↺)
     │           │   ← 方向相反 →
     │ ↻      ↺ │   LB 顺时针 (↻)    RB 逆时针 (↺)
     └───────────┘
```

左侧电机正转 = 顺时针，右侧电机正转 = 逆时针 → 左右轮子反转，底盘不会前进。

**加上 `reverse=[0,0,1,1]` 后：**

```
         Front
     ┌───────────┐
     │ ↻      ↻ │   LF 顺时针 (↻)    RF 顺时针 (↻)
     │           │   ← 方向一致 →
     │ ↻      ↻ │   LB 顺时针 (↻)    RB 顺时针 (↻)
     └───────────┘
         Back
```

右侧电机的 ref 被取反，正速度指令变为逆时针 → 实际转动方向与左侧一致，四轮同向旋转 → 前进。

`motor_reverse_flag` 在 `CalculateLQROutput()` 中将 ref 取反，使运动学层不需要关心单个电机的安装方向。完整取反机制见 [[02_code_twin/modules/MOTOR/motor_base#坐标系与取反机制]]。

---

## 2ms 循环：chassis_func

### 离线检查

4 个电机全部在线才允许控制；任何一个离线则全部 Stop。

### 底盘模式

| 模式 | wz 来源 | 说明 |
|------|---------|------|
| `chassis_zero_force` | — | 全部 Stop |
| `chassis_rotate` | 固定 3 | 正向自旋 |
| `chassis_rotate_reverse` | 固定 -8 | 反向自旋 |
| `chassis_follow_gimbal_yaw` | 跟随 PID 输出 | 跟随云台 yaw |

### 坐标变换

```c
float total_angle_rad = chassis_cmd->offset_angle * DEGREE_2_RAD;
cos_theta = arm_cos_f32(total_angle_rad);
sin_theta = arm_sin_f32(total_angle_rad);
chassis_vx = cmd.vx * cos_theta - cmd.vy * sin_theta;
chassis_vy = cmd.vx * sin_theta + cmd.vy * cos_theta;
```

遥控器输入的速度是云台坐标系下的（"前进"= 云台指向方向），需要旋转到底盘坐标系。`offset_angle` 是云台和底盘的相对角度差。

### 逆运动学

```c
Chassis_Mecanum_Calc(motors, config, vx, vy, vw);
```

把底盘速度向量分解为 4 个轮子的目标速度（rad/s），直接写入 `Motor_DJI_SetRef`。实现在 `modules/algorithm/chassis_type.c`，详见 [[02_code_twin/modules/algorithm/chassis_type]]。

---
