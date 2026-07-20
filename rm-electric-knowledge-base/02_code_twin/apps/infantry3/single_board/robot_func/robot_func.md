# robot_func — 步兵3号遥控器输入

`apps/infantry3/single_board/robot_func/robot_func.c`

## 一句话

读取 DT7 遥控器通道，拆分成底盘/云台/发射三个命令结构体。DT7 用 sw1/sw2 两个三档开关 + 摇杆 + 滚轮。

## RemoteControlSet

### 通道映射

| 通道 | 用途 | 类型 |
|------|------|------|
| CH1 | 底盘左右（vy） | 连续量，归一化 |
| CH2 | 底盘前后（vx） | 连续量，归一化 |
| CH3 | yaw 微调 | 连续量，×0.001 累加 |
| CH4 | pitch 微调 | 连续量，×0.001 累加 |
| sw1 | 云台/发射模式 | 三档开关 |
| sw2 | 底盘模式 | 三档开关 |
| wheel | 拨盘控制 | 滚轮（正/零/负） |

与哨兵的区别：步兵3号用 **DT7** 遥控器（sw1/sw2 + 摇杆 + 滚轮），哨兵用 **SBUS** 遥控器（CH1-8 四摇杆 + 四开关）。详见 [[01_extracted/remote/遥控器协议]]。

### sw2（底盘模式）

| 位置 | 模式 |
|------|------|
| 上 | `chassis_rotate`（自旋） |
| 中 | `chassis_follow_gimbal_yaw`（跟随云台） |
| 下 | `chassis_rotate_reverse`（反向自旋） |

### sw1（云台/发射模式）

| 位置 | 云台 | 发射 |
|------|------|------|
| 上 | — | `shoot_on` + `friction_on`（摩擦轮转，拨盘停） |
| 中 | `gimbal_gyro_mode`（CH3/CH4 微调） | `shoot_on` + `friction_off`（拨盘按滚轮） |
| 下 | `gimbal_zero_force`（停转） | `shoot_off`（全部关闭） |

### 滚轮（拨盘）

| 滚轮方向 | load 模式 |
|---------|----------|
| 0 | `load_stop` |
| 正 | `load_reverse`（反转） |
| 负 | `load_burstfire`（连发） |

### 遥控器离线

全部归零。

### CalcOffsetAngle

与哨兵相同，计算 yaw 编码器偏差用于底盘跟随。

## 链接

- 遥控器协议：[[01_extracted/remote/遥控器协议]]
- DT7 解码：[[02_code_twin/modules/REMOTE/DT7/dt7]]
- 哨兵对照（SBUS）：[[02_code_twin/apps/sentry/gimbal_board/robot_func/robot_func]]
- 应用层调用：[[02_code_twin/apps/infantry3/single_board/robot_control]]
