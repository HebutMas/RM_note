# gimbal_func — 哨兵云台控制

`apps/sentry/gimbal_board/gimbal_func/gimbal_func.c`

## 一句话

3 个电机的云台控制：大 yaw（GM6020）跟随小 yaw 编码器，小 yaw（GM6020）接收遥控器/视觉目标，pitch（DM4310）带重力前馈。支持陀螺模式、自动搜索模式、零力模式。

## 初始化：gimbal_init

```c
ins = Module_INS_get();           // IMU 姿态
bmi088_dev = Module_BMI088_get_device();  // BMI088 原始数据（陀螺仪）
wt606_device = Module_WT606_Get();        // WT606 陀螺仪（大 yaw 用）
```

注册 3 个电机：

| 电机 | 型号 | CAN | 反馈源 | 控制方式 | 说明 |
|------|------|-----|--------|---------|------|
| small_yaw | GM6020 | CAN1 ID=1 | INS.YawTotalAngle + BMI088.gyro[2] | LQR 角度环 | 小 yaw，接收遥控器/视觉目标 |
| big_yaw | GM6020 | CAN2 ID=1 | WT606.YawTotalAngle + WT606.gyro[2] | LQR 角度环 | 大 yaw，跟随小 yaw 编码器 |
| pitch | DM4310 | CAN1 tx=0x23 rx=0x206 | INS.euler_rad[1] + BMI088.gyro[0] | LQR 角度环 | pitch，带重力前馈 |

大 yaw 和小 yaw 的区别：
- **小 yaw** 是操作手直接控制的轴，目标来自遥控器/视觉
- **大 yaw** 不直接接收目标，它的目标是跟随小 yaw 的编码器位置——`big_yaw_offset = WT606角度 + 小yaw编码器偏差`

详见电机注册和 LQR 参数见 [[02_code_twin/modules/MOTOR/motor_base]] / [[02_code_twin/modules/MOTOR/DJI/motor_dji]] / [[02_code_twin/modules/MOTOR/DAMIAO/motor_damiao]]。

## 2ms 循环：gimbal_func

```c
void gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd) {
    // 1. 离线检查：3 个电机全部在线才控制
    // 2. pitch 重力前馈
    Gimbal_PitchFeedback(pitch_motor, ins->euler_rad[1]);
    // 3. 大 yaw 跟随小 yaw
    big_yaw_offset = wt606_device->YawTotalAngle_rad
                   + (small_yaw_motor->measure.ecd - SMALL_YAW_ALIGN_ECD) * (2π/8192);
    // 4. 按模式设置目标
    switch (gimbal_cmd->gimbal_mode) { ... }
    // 5. 反馈 yaw 编码器值给底盘跟随
    *yaw_ecd = big_yaw_motor->measure.ecd;
}
```

### 模式

| 模式 | small_yaw 目标 | big_yaw 目标 | pitch 目标 | 说明 |
|------|---------------|-------------|-----------|------|
| `gimbal_zero_force` | Stop | Stop | Stop | 停转 |
| `gimbal_gyro_mode` | `cmd->yaw + auto_yaw_offset` | `big_yaw_offset` | `cmd->pitch` | 遥控器手动控制 |
| `gimbal_auto_mode` (search=0) | `cmd->yaw` | `big_yaw_offset` | `cmd->pitch` | 视觉目标跟踪 |
| `gimbal_auto_mode` (search=1) | 持续旋转搜索 | `big_yaw_offset` | 正弦摆动 | 自动搜索目标 |
| `gimbal_auto_mode` (search=2) | `INS.YawRoundCount × 360°` | `big_yaw_offset` | 0 | 回正 |

### pitch 重力前馈

```c
static void Gimbal_PitchFeedback(DM_Motor_t *motor, float cur_angle) {
    float ff_gravity = GRAVITY_K_PITCH * cur_angle + GRAVITY_GAMMA;
    pitch_motor->base.controller.feedforward_torque = ff_gravity;
}
```

根据当前 pitch 角度计算重力扭矩，写入前馈字段。LQR 输出会自动叠加前馈：`torque = LQR输出 + feedforward_torque`。这样 LQR 只需要补偿动态部分，不用对抗重力。

## 链接

- 电机基类：[[02_code_twin/modules/MOTOR/motor_base]]
- 大疆电机：[[02_code_twin/modules/MOTOR/DJI/motor_dji]]
- 达妙电机：[[02_code_twin/modules/MOTOR/DAMIAO/motor_damiao]]
- 遥控器输入：[[02_code_twin/apps/sentry/gimbal_board/robot_func]]
- 应用层调用：[[02_code_twin/apps/sentry/gimbal_board/robot_control]]
