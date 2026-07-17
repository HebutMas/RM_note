# robot_func — 哨兵云台板遥控器输入

`apps/sentry/gimbal_board/robot_func/robot_func.c`

## 一句话

读取 SBUS 遥控器通道，拆分成底盘/云台/发射三个命令结构体。SBUS 8 通道：CH1-4 摇杆，CH5-8 开关。

## RemoteControlSet

```c
void RemoteControlSet(Chassis_Ctrl_Cmd_t *Chassis_Ctrl,
                      Shoot_Ctrl_Cmd_t *Shoot_Ctrl,
                      Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl);
```

### 通道映射

| 通道 | 用途 | 类型 |
|------|------|------|
| CH1 | 底盘左右（vy） | 连续量，归一化到 -1.0~+1.0 |
| CH2 | 底盘前后（vx） | 连续量 |
| CH3 | pitch 微调 | 连续量，×0.001 累加 |
| CH4 | yaw 微调 | 连续量，×0.001 累加 |
| CH5 | 发射控制 | 三档开关 |
| CH6 | 云台模式 | 三档开关 |
| CH7 | 拨盘控制 | 三档开关 |
| CH8 | 底盘模式 | 三档开关 |

### CH6（云台模式）

| 位置 | 模式 | 说明 |
|------|------|------|
| 上 | `gimbal_gyro_mode` | 手动控制，CH3/CH4 微调 pitch/yaw |
| 中 | `gimbal_auto_mode` | 自动模式，底盘也切 `chassis_automode` |
| 下 | `gimbal_zero_force` | 停转，底盘/发射全部关闭 |

### CH8（底盘模式）

| 位置 | 模式 | 说明 |
|------|------|------|
| 上 | `chassis_follow_gimbal_yaw` | 跟随云台 |
| 中 | `chassis_rotate` | 自旋 |
| 下 | `chassis_rotate_reverse` | 反向自旋 |

### CH5 + CH7（发射控制）

| CH5 位置 | CH7 位置 | friction | load |
|---------|---------|----------|------|
| 上 | — | off | stop |
| 中 | — | off | — |
| 下 | 上 | on | `load_1_bullet`（单发） |
| 下 | 中 | on | `load_stop`（待命） |
| 下 | 下 | on | `load_burstfire`（连发） |

### 遥控器离线

全部归零：`gimbal_zero_force` / `chassis_zero_force` / `shoot_off`。

### CalcOffsetAngle

```c
int16_t CalcOffsetAngle(float getyawangle) {
    // yaw 编码器差值，归一化到 [-4095, 4095]
    offset_ecd = getyawangle - YAW_CHASSIS_ALIGN_ECD;
    // 处理跨 0/8191 边界
    while (offset_ecd > 4095) offset_ecd -= 8191;
    while (offset_ecd < -4095) offset_ecd += 8191;
    return (int16_t)offset_ecd;
}
```

计算云台 yaw 编码器与底盘对齐位置的偏差，用于底盘跟随云台。详见 [[02_code_twin/apps/sentry/gimbal_board/robot_control]] 的板间通信部分。

## 链接

- 遥控器协议：[[01_extracted/remote/remote_protocol]]
- SBUS 解码：[[02_code_twin/modules/REMOTE/SBUS/sbus]]
- 应用层调用：[[02_code_twin/apps/sentry/gimbal_board/robot_control]]
