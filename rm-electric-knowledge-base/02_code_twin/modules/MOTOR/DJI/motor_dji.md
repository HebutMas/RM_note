# motor_dji.c/h — 大疆电机驱动

`modules/MOTOR/DJI/motor_dji.{c,h}`

## 一句话

继承 `Motor_Base`，实现大疆电机（GM6020/M3508/M2006）的 PID/LQR 控制和 CAN 通信。4 个电机共用 1 帧 CAN 报文，先填静态缓冲区，`Motor_DJI_Flush()` 批量发送。

## 为什么大疆需要 Flush

大疆 CAN 协议：**1 帧报文（8 字节）控制 4 个电机**，每个电机占 2 字节电流值。

| 帧 ID | 控制电机 | 每电机占 |
|-------|---------|---------|
| 0x200 | M3508/M2006 ID 1~4 | 2 字节 |
| 0x1FF | M3508/M2006 ID 5~8 | 2 字节 |
| 0x1FF | GM6020 ID 1~4（电压） | 2 字节 |
| 0x2FF | GM6020 ID 5~7（电压） | 2 字节 |
| 0x1FE | GM6020 ID 1~4（电流） | 2 字节 |
| 0x2FE | GM6020 ID 5~7（电流） | 2 字节 |

协议详见 [[01_extracted/motor/motor_params#CAN 通信协议]]。

如果每个电机单独发，4 个电机要发 4 帧 CAN，但它们共用同一个帧 ID。**后发的会覆盖先发的数据**——因为同一个帧 ID 只有一个 8 字节缓冲区。

解决方案：`dji_control` 只把自己的电流值填入静态缓冲区 `sender_assignment[group]` 的对应位置，不实际发送。等所有电机都计算完毕，`Motor_DJI_Flush()` 一次性发送所有有数据的帧。

```c
// motor_task_entry 中的调用顺序
Motor_UpdateAll();       // 每个电机调 dji_control，填缓冲区
Motor_DJI_Flush();       // 批量发送所有缓冲区
```

## SenderGrouping — 发送分组设计

### 数据结构

```c
static BSP_CanMsg_t sender_assignment[MOTOR_SENDER_SIZE] = {
    [0] = { .hcan = BSP_CAN_HANDLE1, .id = 0x1ff, .len = 8, .data = {0} },
    [1] = { .hcan = BSP_CAN_HANDLE1, .id = 0x200, .len = 8, .data = {0} },
    [2] = { .hcan = BSP_CAN_HANDLE1, .id = 0x2ff, .len = 8, .data = {0} },
    [3] = { .hcan = BSP_CAN_HANDLE1, .id = 0x1fe, .len = 8, .data = {0} },
    [4] = { .hcan = BSP_CAN_HANDLE1, .id = 0x2fe, .len = 8, .data = {0} },
    [5] = { .hcan = BSP_CAN_HANDLE2, .id = 0x1ff, .len = 8, .data = {0} },
    // ... CAN2、CAN3 同理
};
```

F407 有 2 路 CAN（`MOTOR_SENDER_SIZE=10`），H723 有 3 路 CAN（`MOTOR_SENDER_SIZE=15`）。每路 CAN 有 5 个帧 ID（0x1FF/0x200/0x2FF/0x1FE/0x2FE），共 5×2=10 或 5×3=15 个分组。

### 分组计算逻辑

```c
static UINT MotorSenderGrouping(DJI_Motor_t *motor, Can_Device_Init_Config_s *config) {
    uint8_t motor_id = config->tx_id - 1;  // ID 1~8 → 0~7
    uint8_t motor_send_num;                 // 组内序号 0~3
    uint8_t motor_grouping;                 // sender_assignment 数组下标

    switch (motor->base.info.motor_type) {
    case M3508:
    case M2006:
        if (motor_id < 4) {
            motor_send_num = motor_id;           // 0~3
            motor_grouping = CAN1的0x1FF组;     // 对应 sender_assignment[1]
        } else {
            motor_send_num = motor_id - 4;       // 0~3
            motor_grouping = CAN1的0x200组;     // 对应 sender_assignment[0]
        }
        break;
    case GM6020_CURRENT:
        // 类似，但帧 ID 为 0x1FE/0x2FE
        break;
    }
    // ...
}
```

分组规则：

| 电机类型 | ID 范围 | 帧 ID | 数据位置 |
|---------|---------|-------|---------|
| M3508/M2006 | ID 1~4 | 0x200 | `sender_assignment[group].data[2*num]` 和 `[2*num+1]` |
| M3508/M2006 | ID 5~8 | 0x1FF | 同上 |
| GM6020（电流） | ID 1~4 | 0x1FE | 同上 |
| GM6020（电流） | ID 5~7 | 0x2FE | 同上 |
| GM6020（电压） | ID 1~4 | 0x1FF | 同上 |
| GM6020（电压） | ID 5~7 | 0x2FF | 同上 |

CAN 总线选择（CAN1/CAN2/CAN3）通过 `BSP_CAN_IS_HANDLE1/2/3` 宏判断，决定 `motor_grouping` 的基数（CAN1=0, CAN2=5, CAN3=10）。

## 潜在冲突

### 帧 ID 冲突

| 冲突场景 | 帧 ID | 原因 |
|---------|-------|------|
| M3508 ID 5~8 vs GM6020 ID 1~4（电压模式） | 0x1FF | 两者共用同一帧 ID，同时使用会互相覆盖数据 |
| M2006 ID 1~4 vs M3508 ID 1~4 | 0x200 | C610 和 C620 电调共用帧 ID，同一总线上同 ID 的 M2006 和 M3508 会冲突 |
| GM6020 电压 vs GM6020 电流 | 0x1FF vs 0x1FE | 不冲突（帧 ID 不同），但同总线上同 ID 的电压模式和电流模式 6020 会收到错误的控制帧 |

> **规避方法**：
> 1. 同一 CAN 总线上，M3508/M2006 的 ID 不要重叠（如 M3508 用 1~4，M2006 用 5~8）
> 2. GM6020 用电流模式时，不要在该总线上放电压模式的 6020
> 3. 不同 CAN 总线可以复用 ID（CAN1 和 CAN2 互不干扰）

### 反馈帧冲突

反馈帧 ID = `0x200 + 电调 ID`（M3508/M2006）或 `0x204 + 驱动器 ID`（GM6020）。同一总线上不同类型的电机如果 ID 相同，反馈帧也会冲突。

## 力矩→电流→整数值映射

```c
// dji_control 中
switch (motor->base.info.motor_type) {
case GM6020_CURRENT:
    motor->base.controller.output =
        currentToInteger(-3.0f, 3.0f, -16384, 16384,
            torque / (torque_constant * gear_ratio));
    break;
case M3508:
    motor->base.controller.output =
        currentToInteger(-20.0f, 20.0f, -16384, 16384,
            torque / (torque_constant * gear_ratio));
    break;
case M2006:
    motor->base.controller.output =
        currentToInteger(-10.0f, 10.0f, -10000, 10000,
            torque / (torque_constant * gear_ratio));
    break;
}
```

映射过程：PID/LQR 输出力矩 → 除以（转矩常数 × 减速比）= 物理电流 → `currentToInteger` 线性映射到整数范围。

| 电机 | 物理电流范围 | 整数范围 | 转矩常数 | 减速比 |
|------|------------|---------|---------|--------|
| GM6020 | ±3 A | ±16384 | 0.741 N·m/A | 1 |
| M3508 | ±20 A | ±16384 | 0.3 N·m/A | 3591/187 |
| M2006 | ±10 A | ±10000 | 0.18 N·m/A | 36 |

参数来源见 [[01_extracted/motor/motor_params#特征参数]]。

## 填充发送缓冲区

```c
int16_t out = (int16_t)base->controller.output;
sender_assignment[group].data[2 * num]     = (uint8_t)(out >> 8);     // 高 8 位
sender_assignment[group].data[2 * num + 1] = (uint8_t)(out & 0x00ff); // 低 8 位
```

大端存储：高字节在前。这是因为大疆电调期望大端格式。

## Flush 发送

```c
void Motor_DJI_Flush(void) {
    for (size_t i = 0; i < MOTOR_SENDER_SIZE; ++i) {
        if (sender_enable_flag[i]) {
            BSP_CAN_SendMessage(&sender_assignment[i]);
        }
    }
}
```

只发送有电机注册的分组（`sender_enable_flag[i]=1`），避免发送空帧。

## CAN 接收回调

```c
static void dji_can_rx_callback(Can_Device *dev, const uint8_t *data, uint8_t len) {
    DJI_Motor_t *motor = (DJI_Motor_t *)dev->user_arg;

    motor->measure.last_ecd = motor->measure.ecd;
    motor->measure.ecd = ((uint16_t)data[0] << 8) | data[1];  // 0~8191
    motor->measure.speed_rpm = (int16_t)(data[2] << 8 | data[3]);
    motor->base.measure.speed_rad = motor->measure.speed_rpm * RPM_2_RAD_PER_SEC;
    motor->measure.real_current = (int16_t)(data[4] << 8 | data[5]);
    motor->measure.temperature = data[6];

    // 多圈角度计算
    int16_t delta_ecd = motor->measure.ecd - motor->measure.last_ecd;
    if (delta_ecd > 4096)        // 正向溢出（从 8191 跳到 0）
        motor->measure.total_round--;
    else if (delta_ecd < -4096)  // 反向溢出（从 0 跳到 8191）
        motor->measure.total_round++;

    motor->base.measure.total_angle =
        (float)motor->measure.total_round * (2.0f * PI) + motor->base.measure.single_round_angle;

    Module_Offline_device_update(motor->base.offline_dev);  // 更新心跳
}
```

### 多圈角度计算

编码器值 0~8191 对应 0~360°（`ECD_ANGLE_COEF_DJI_RAD = 2π/8192`）。跨过 0/8191 边界时检测溢出：

- `delta_ecd > 4096`（半圈）：实际是从 8191→0 反转，`total_round--`
- `delta_ecd < -4096`：实际是从 0→8191 正转，`total_round++`

总角度 = 圈数 × 2π + 单圈角度。无符号减法不怕溢出的原理见 [[01_extracted/algorithm/computer-basics#无符号整数回卷]]。

反馈帧格式详见 [[01_extracted/motor/motor_params#反馈帧]]。

## 离线/禁用处理

```c
if (Module_Offline_get_device_status(base->offline_dev) == STATE_OFFLINE || base->setting.enableflag == 0) {
    // 清零发送缓冲区对应位置
    sender_assignment[group].data[2 * num] = 0;
    sender_assignment[group].data[2 * num + 1] = 0;
    // 清零 PID 积分项
    base->controller.speed_PID.Iout = 0;
    base->controller.angle_PID.Iout = 0;
    return;
}
```

离线或禁用时：填零（不发力矩）、清零 PID 积分项（防止积分饱和）。
