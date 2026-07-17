# chassis_func — 哨兵底盘控制

`apps/sentry/chassis_board/chassis/chassis_func.c`

## 一句话

8 电机舵轮底盘：4 个 M3508 驱动轮 + 4 个 GM6020 舵向轮。逆运动学将底盘速度向量分解为 4 个轮子的驱动转速和转向角度。带功率控制。

## 初始化：chassis_init

### 电机注册

| 电机 | 型号 | CAN | 角色 | 功率控制 | 说明 |
|------|------|-----|------|---------|------|
| [0-3] | M3508 | CAN2 ID=1-4 | DRIVE | k1=0.132, k2=3.47, k3=1 | 驱动轮，减速比 16:1 |
| [4-7] | GM6020 | CAN1 ID=1-4 | STEER | k1=0.005, k2=12.98, k3=1 | 舵向轮，无减速 |

全部注册到功率控制，功率上限 120W。详见 [[02_code_twin/modules/MOTOR/POWER_CONTROL/power_control]]。

### 舵轮几何配置

```c
static const Chassis_Swerve_Config_s chassis_swerve_config = {
    .align_rad = {4458, 1010, 7177, 3756},  // 4 个轮子的机械零点（编码器值×弧度系数）
    .decele_ratio = 16.0f,   // 驱动电机减速比
    .radius_wheel_m = 0.12f, // 舵轮投影点到几何中心距离
    .wheel_r = 0.5f,         // 轮子半径
};
```

### 跟随 PID

```c
PID_Init_Config_s config = {.Kp = 0.1, .Ki = 0, .Kd = 0.001, ...};
```

底盘跟随云台时，用 PID 以编码器偏差为输入，输出旋转角速度 `chassis_wz`。

## 2ms 循环：chassis_func

```c
void chassis_func(Chassis_Ctrl_Cmd_t *chassis_cmd) {
    // 1. 8 个电机全部在线才控制
    // 2. 按模式设定 wz
    // 3. 云台系 → 底盘系坐标变换
    // 4. 逆运动学：Chassis_Swerve_Calc → 设置 8 个电机目标
}
```

### 底盘模式

| 模式 | wz 来源 | 说明 |
|------|---------|------|
| `chassis_zero_force` | — | 全部 Stop |
| `chassis_rotate` | 固定 3 | 正向自旋 |
| `chassis_rotate_reverse` | 固定 -8 | 反向自旋 |
| `chassis_follow_gimbal_yaw` | 跟随 PID 输出 | 跟随云台 yaw |
| `chassis_automode` | 0（比赛中才动） | 自主导航 |

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

`Chassis_Swerve_Calc(motors, config, vx, vy, vw)` 把底盘速度向量分解为 4 个轮子的驱动转速（写入 motors[0-3] 的 ref）和转向角度（写入 motors[4-7] 的 ref）。算法实现在 `modules/algorithm/chassis_type.c`。

## 链接

- 电机：[[02_code_twin/modules/MOTOR/motor_base]] / [[02_code_twin/modules/MOTOR/DJI/motor_dji]]
- 功率控制：[[02_code_twin/modules/MOTOR/POWER_CONTROL/power_control]]
- 应用层调用：[[02_code_twin/apps/sentry/chassis_board/robot_control]]
