# gimbal_func — 步兵3号云台控制

`apps/infantry3/single_board/gimbal_func/gimbal_func.c`

## 一句话

2 个电机的云台控制：1 个 GM6020(yaw) + 1 个 DM4310(pitch)，LQR 角度环。比哨兵简单——没有大小 yaw 协调，没有自动搜索。

## 初始化：gimbal_init

| 电机 | 型号 | CAN | 反馈源 | 控制方式 |
|------|------|-----|--------|---------|
| yaw | GM6020 | CAN1 ID=1 | INS.YawTotalAngle + BMI088.gyro[2] | LQR 角度环 |
| pitch | DM4310 | CAN2 tx=0x01 rx=0xF1 | INS.euler_rad[1] + BMI088.gyro[0] | LQR 角度环 |

与哨兵的区别：只有 1 个 yaw 电机（哨兵有大 yaw + 小 yaw 两个），没有 WT606 陀螺仪。

## 2ms 循环：gimbal_func

```c
void gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd) {
    // 1. 离线检查
    // 2. 按模式设置目标
    switch (gimbal_cmd->gimbal_mode) {
        case gimbal_zero_force: Stop; break;
        case gimbal_gyro_mode:
            Motor_DJI_SetRef(yaw_motor, cmd->yaw * DEGREE_2_RAD);
            Motor_DM_SetRef(pitch_motor, cmd->pitch * DEGREE_2_RAD);
            break;
        case gimbal_auto_mode: break;  // 步兵3号未实现自动模式
    }
    // 3. 反馈 yaw 编码器
    *yaw_ecd = yaw_motor->measure.ecd;
}
```

与哨兵的区别：

| | 步兵3号 | 哨兵 |
|---|---|---|
| yaw 电机数 | 1（GM6020） | 2（大 yaw + 小 yaw，协调控制） |
| pitch 重力前馈 | 无 | 有（`Gimbal_PitchFeedback`） |
| 自动搜索 | 未实现 | 有（持续旋转 + 正弦摆动） |
| yaw 反馈源 | INS | 小 yaw 用 INS，大 yaw 用 WT606 |

## 链接

- 电机：[[02_code_twin/modules/MOTOR/motor_base]] / [[02_code_twin/modules/MOTOR/DJI/motor_dji]] / [[02_code_twin/modules/MOTOR/DAMIAO/motor_damiao]]
- 遥控器输入：[[02_code_twin/apps/infantry3/single_board/robot_func/robot_func]]
- 应用层调用：[[02_code_twin/apps/infantry3/single_board/robot_control]]
