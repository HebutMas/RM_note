# gimbal_func — 步兵3号云台控制

`apps/infantry3/single_board/gimbal_func/gimbal_func.c`

## 一句话

2 个电机的云台控制：1 个 GM6020(yaw) + 1 个 DM4310(pitch)，LQR 角度环。比哨兵简单——没有大小 yaw 协调，没有自动搜索。

## 初始化：gimbal_init

### 电机注册总览

| 电机 | 型号 | CAN | 角度反馈 | 速度反馈 | `feedback_reverse_flag` | `motor_reverse_flag` | 控制方式 |
|------|------|-----|---------|---------|------|------|--------|
| yaw | GM6020 | CAN1 ID=1 | `ins->YawTotalAngle_rad` | `bmi088_dev->gyro[2]` | 0 | 0 | LQR 角度环 |
| pitch | DM4310 | CAN2 tx=0x01 rx=0xF1 | `ins->euler_rad[1]` | `bmi088_dev->gyro[0]` | **1** | 0 | LQR 角度环 |

两个电机的 `angle_feedback_source = 1`、`speed_feedback_source = 1`，即角度和速度都走外部反馈（INS + BMI088），不用电机自身编码器。

与哨兵的区别：只有 1 个 yaw 电机（哨兵有大 yaw + 小 yaw 两个），没有 WT606 陀螺仪。

---

### BMI088 轴映射：为什么 pitch 用 gyro[0] 而不是 gyro[1]

> 芯片轴 → C 板坐标系（前 X、左 Y、上 Z）的完整推导和实物照片参考见 [[01_extracted/hardware/bmi088-datasheet#芯片轴 → C 板坐标系映射]]。

这是最容易踩坑的地方。BMI088 芯片寄存器按 X/Y/Z 输出，但芯片在 C 板上的物理安装方向导致芯片坐标轴和板子坐标轴不是一一对应的：

```
    BMI088 芯片轴          C 板坐标轴（你的图片标注）
    ─────────────          ──────────────────────
    chip X  ──→  pitch  ←── board Y / euler[1]
    chip Y  ──→  roll   ←── board X / euler[0]
    chip Z  ──→  yaw    ←── board Z / euler[2]
```

因此 `bmi088_dev->gyro[3]` 的实际含义是：

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

### pitch 的 `feedback_reverse_flag = 1`：把反馈掰到电机轴

pitch 电机（DM4310）的安装方向与 INS pitch 轴方向相反：

```
INS 坐标系（前 X、左 Y、上 Z）：
    pitch 正方向 = 抬头（枪口向上）

DM4310 电机坐标系：
    电机正方向 = 低头（枪口向下）
    （因为电机安装方向和 INS 相反）
```

如果不做任何处理，INS 反馈的 pitch 角度增大（抬头），但电机认为自己在减小 → 正反馈 → 发散。

`feedback_reverse_flag = 1` 的作用是在 `CalculateLQROutput` 中把外部反馈取反：

```c
// motor_dji.c / motor_damiao.c 中
pid_measure = *other_angle_feedback_ptr;   // 读 INS pitch
if (feedback_reverse_flag == 1)            // pitch 电机 = 1
    pid_measure *= -1;                     // ← 反转，对齐电机轴
```

角度反馈和速度反馈都走这条路径，都被取反。**效果是把 INS 坐标系掰到电机坐标系**，LQR 在电机坐标系内闭环。

---

### 为什么不对 ref 取反

注意 pitch 电机的 `motor_reverse_flag` 没有设置（默认 0），即 ref 不取反。这与底盘电机不同——底盘的 `motor_reverse_flag` 是为了处理安装方向，而 pitch 这里通过另一种方式解决。

原因在于 **ref 的符号在更上层就已经对了**。`gimbal_func` 中：

```c
Motor_DM_SetRef(pitch_motor, gimbal_cmd->pitch * DEGREE_2_RAD);
```

`gimbal_cmd->pitch` 来自遥控器/视觉输入。遥控器的 pitch 通道方向已经按照"推杆=低头"来设计，和电机的正方向一致。所以 ref 进入 LQR 时符号已经正确，不需要再取反。

总结三个 flag 的分工：

| flag | 作用 | pitch 电机 | yaw 电机 | 底盘 M3508 |
|------|------|-----------|---------|-----------|
| `feedback_reverse_flag` | 反转外部反馈（角度+速度） | **1**（INS 和电机方向相反） | 0 | 0（用电机自身编码器） |
| `motor_reverse_flag` | 反转 ref | 0（上层已处理符号） | 0 | [0,0,1,1]（左右安装对称） |
| `angle/speed_feedback_source` | 选反馈来源 | 1（INS） | 1（INS） | 0（电机编码器） |

> **设计思路**：`feedback_reverse_flag` 处理传感器和电机之间的坐标系差异，`motor_reverse_flag` 处理电机安装方向，两者在不同环节独立工作。步兵3号 pitch 只需要前者，不需要后者。

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

- BMI088 轴映射：[[02_code_twin/modules/BMI088/module_bmi088#轴映射]]
- INS 欧拉角定义：[[02_code_twin/modules/INS/module_ins#坐标系与轴映射]]
- 电机取反逻辑：[[02_code_twin/apps/infantry3/single_board/chassis_func/chassis_func#电机取反逻辑]]
- 电机：[[02_code_twin/modules/MOTOR/motor_base]] / [[02_code_twin/modules/MOTOR/DJI/motor_dji]] / [[02_code_twin/modules/MOTOR/DAMIAO/motor_damiao]]
- 遥控器输入：[[02_code_twin/apps/infantry3/single_board/robot_func/robot_func]]
- 应用层调用：[[02_code_twin/apps/infantry3/single_board/robot_control]]
