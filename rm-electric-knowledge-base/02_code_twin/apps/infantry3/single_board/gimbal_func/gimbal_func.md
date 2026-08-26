# gimbal_func — 步兵3号云台控制

`apps/infantry3/single_board/gimbal_func/gimbal_func.c`

## 一句话

2 个电机的云台控制：1 个 GM6020(yaw) + 1 个 DM4310(pitch)，LQR 角度环。比哨兵简单——没有大小 yaw 协调，没有自动搜索。

## 初始化：gimbal_init

### 电机注册总览

| 电机 | 型号 | CAN | 角度反馈 | 速度反馈 | `feedback_reverse_flag` | `motor_reverse_flag` | 控制方式 |
|------|------|-----|---------|---------|------|------|--------|
| yaw | GM6020 | CAN1 ID=1 | `ins->YawTotalAngle_rad` | `bmi088_dev->gyro[2]` | 0 | 0 | LQR 角度环 |
| pitch | DM4310 | CAN2 tx=0x01 rx=0xF1 | `ins->euler_rad[1]` | `bmi088_dev->gyro[0]` | **1** | **1** | LQR 角度环 |

> ⚠️ 2026-08-26 实测更正：pitch 的 `motor_reverse_flag` 由 0 → **1**（migratef1x 已加上）。实测依据：遥控上推左摇杆 → `gimbal_cmd->pitch` 增加 → **云台低头**；若不取反 ref，上推会变成抬头。

两个电机的 `angle_feedback_source = 1`、`speed_feedback_source = 1`，即角度和速度都走外部反馈（INS + BMI088），不用电机自身编码器。

与哨兵的区别：只有 1 个 yaw 电机（哨兵有大 yaw + 小 yaw 两个），没有 WT606 陀螺仪。

---

### BMI088 轴映射：为什么 pitch 用 gyro[0] 而不是 gyro[1]

> BMI088 芯片在 C 板上的物理方向（Pin1 位置、长边方向、坐标系推导）详见 [[01_extracted/hardware/bmi088-orientation]]。芯片轴到 C 板轴的映射表详见 [[02_code_twin/modules/BMI088/module_bmi088#轴映射]]。

这是最容易踩坑的地方。BMI088 输出 `gyro[0/1/2]` 对应 chip X/Y/Z，但芯片在 C 板上的物理安装方向导致索引含义与直觉不符：

| 索引 | 芯片轴 | 实际物理含义 | 对应的 INS 欧拉角 |
|------|--------|------------|-----------------|
| `gyro[0]` | chip X | **pitch 角速度** | `euler_rad[1]` |
| `gyro[1]` | chip Y | **roll 角速度** | `euler_rad[0]` |
| `gyro[2]` | chip Z | **yaw 角速度** | `euler_rad[2]` |

> 注意：`gyro[0]` 是 pitch，但 `euler_angle[0]` 是 roll。**gyro 数组和 euler 数组的索引含义不同**，选反馈源时不能想当然地用相同下标。

源码中的注释也印证了这一点：
```c
.other_speed_feedback_ptr = &bmi088_dev->gyro[0], // c板的pitch轴角速度，根据实际选择对应角速度
```

yaw 电机用 `gyro[2]`（chip Z = yaw），pitch 电机用 `gyro[0]`（chip X = pitch）。如果错误地用了 `gyro[1]`，那反馈的是 roll 角速度，云台会失控。

---

### pitch 的两个取反：feedback 掰反馈、ref 掰指令

pitch 电机（DM4310）的安装方向与 INS pitch 轴方向相反：

```
INS 坐标系（前 X、左 Y、上 Z）：
    pitch 正方向 = 低头（重力补偿后得到）
    （INS 欧拉角 euler_rad[1] 在低头时增大——实测下因为俯仰重力关系"向下为增大"）

DM4310 电机坐标系：
    电机 total_angle 正方向 = 抬头（枪口向上）
    （电机编码器向上转时 total_angle 增大——实测）

→ 因此：电机的"抬头" 对应 INS 的 "低头减小"，电机轴与 INS pitch 轴相反。
```

`feedback_reverse_flag = 1` 的作用是在 `CalculatePIDOutput` / `CalculateLQROutput` 中把外部反馈取反：

```c
// motor_dji.c / motor_damiao.c 中
pid_measure = *other_angle_feedback_ptr;   // 读 INS pitch
if (feedback_reverse_flag == 1)            // pitch 电机 = 1
    pid_measure *= -1;                     // ← 反转，对齐电机轴
```

**同时 `motor_reverse_flag = 1` 也要取反 ref**（migratef1x 2026-08-26 实测后确认）：

```c
// motor_dji.c / motor_damiao.c 中
pid_ref = motor->base.controller.ref;
if (motor->base.setting.motor_reverse_flag == 1) pid_ref *= -1;   // 现在的 pitch 电机 = 1
```

原因：`gimbal_cmd->pitch` 来自遥控器。**实测后确定：遥控器往上推 → pitch ref 值增加 → 云台低头**（即"上推=低头"）。而电机轴正方向是"抬头"（total_angle 向上增大）。所以只有两个取反都加，才能让：

- feedback 取反 → 把 INS 的"低头为正"掰成"抬头为正"（对齐电机轴正向）；
- ref 取反 → 把遥控的"上推=低头"掰成"上推时电机目标角减小"（对齐电机轴负向）。

**两个取反必须同时成立**，缺少任何一个，LQR 都变成正反馈发散。

> ⚠️ 早期笔记曾写 `motor_reverse_flag=0`，那是旧状态。migratef1x 已实测改为 `=1`。**注意**：此处与底盘不同——底盘 `[1,1,0,0]` 是为了处理左右对称安装，见 [[02_code_twin/modules/MOTOR/motor_base#motor_reverse_flag -- 参考端取反]]。

pitch 电机的完整配置：

| flag | 值 | 含义 |
|------|-----|------|
| `feedback_reverse_flag` | **1** | INS 和电机方向相反，把反馈取反到电机轴 |
| `motor_reverse_flag` | **1** | 遥控"上推=低头"，把 ref 取反到电机轴 |
| `angle_feedback_source` | 1 | 用 INS 欧拉角 |
| `speed_feedback_source` | 1 | 用 BMI088 陀螺仪 |

> 两个反转 flag 的定义和区别详见 [[02_code_twin/modules/MOTOR/motor_base#坐标系与取反机制]]。

---

### 视觉协议的坐标系

视觉回传的消息和命令遵循 **前 X、左 Y、上 Z** 的标准坐标系，与 INS 的导航系定义一致。这意味着视觉给出的 yaw/pitch 目标值可以直接作为 ref 下发，不需要额外的坐标变换。

但需要注意：视觉的 yaw 是绝对角度（相对于初始朝向），而 `gimbal_cmd->yaw` 在 gyro 模式下是增量控制（遥控器给出的是角速度积分）。自动模式（`gimbal_auto_mode`）如果接入视觉，需要把视觉绝对角度转成和 `ins->YawTotalAngle_rad` 同一个参考系。

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

- BMI088 物理轴方向（实物验证）：[[01_extracted/hardware/bmi088-orientation]]
- BMI088 代码轴映射：[[02_code_twin/modules/BMI088/module_bmi088#轴映射]]
- INS 导航系定义与右手系规则：[[02_code_twin/modules/INS/module_ins#坐标系与轴映射]]
- 电机取反机制（权威来源）：[[02_code_twin/modules/MOTOR/motor_base#坐标系与取反机制]]
- 电机：[[02_code_twin/modules/MOTOR/motor_base]] / [[02_code_twin/modules/MOTOR/DJI/motor_dji]] / [[02_code_twin/modules/MOTOR/DAMIAO/motor_damiao]]
- 遥控器输入：[[02_code_twin/apps/infantry3/single_board/robot_func/robot_func]]
- 应用层调用：[[02_code_twin/apps/infantry3/single_board/robot_control]]
