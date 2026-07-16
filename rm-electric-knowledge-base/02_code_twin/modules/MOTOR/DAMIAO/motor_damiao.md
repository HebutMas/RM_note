# motor_damiao.c/h — 达妙电机驱动

`modules/MOTOR/DAMIAO/motor_damiao.{c,h}`

## 一句话

继承 `Motor_Base`，实现达妙电机（DM4310/DM6220/DM8009 等）的 PID/LQR 控制和 CAN 通信。每个电机独立 1 帧 CAN，`dm_ControlAndSend` 回调里直接发送，不需要 Flush。

## 为什么达妙不需要 Flush

达妙 CAN 协议：**1 帧报文控制 1 个电机**，帧 ID = `CAN_ID + 模式偏移`。

| 模式 | 帧 ID 偏移 | DLC | 数据内容 |
|------|-----------|-----|---------|
| MIT 模式 | +0x000 | 8 | p+v+Kp+Kd+t 按位打包 |
| 位置速度模式 | +0x100 | 8 | p(float) + v(float) |
| 速度模式 | +0x200 | 4 | v(float) |
| 力位混控 | +0x300 | 8 | p(float) + v(uint16) + i(uint16) |

每个电机的帧 ID 不同（由 `CAN_ID` 决定），不会互相覆盖。所以 `dm_ControlAndSend` 里直接调 `BSP_CAN_SendMessage` 发送，不需要缓冲区。

```c
// dm_ControlAndSend 中
switch (motor->mode_type) {
case DM_MIT_MODE:
    mit_ctrl(motor, 0, 0, 0, 0, base->controller.output);  // 直接发送
    break;
case DM_SPD_MODE:
    speed_ctrl(motor, base->controller.ref);  // 直接发送
    break;
}
```

### 与大疆的对比

| | 大疆电机 | 达妙电机 |
|---|---|---|
| 协议帧 | 1 帧控制 4 个电机 | 1 帧控制 1 个电机 |
| 帧 ID | 按类型固定（0x200/0x1FF/...） | CAN_ID + 模式偏移，每个电机不同 |
| 发送方式 | 先填 `sender_assignment` 静态缓冲区，`Flush` 批量发 | `ControlAndSend` 回调里直接 `BSP_CAN_SendMessage` |
| 冲突风险 | 同帧 ID 的电机会互相覆盖 | 无（帧 ID 各不相同） |
| API | `Motor_DJI_Flush()` 在 `UpdateAll` 后调用 | 无需 Flush |

## 参数表 dm_motor_params

```c
static const DM_Motor_Params_t dm_motor_params[] = {
    {-12.5f, 12.5f, -30.0f, 30.0f, 0.0f, 500.0f, 0.0f, 5.0f, -10.0f, 10.0f}, /* DM4310 */
};

static const DM_Motor_Params_t *dm_get_params(Motor_Type_e type) {
    switch (type) {
    case DM4310:
        return &dm_motor_params[0];
    default:
        return &dm_motor_params[0];  // 暂时全部用 DM4310 参数
    }
}
```

参数结构体：

```c
typedef struct {
    float p_min, p_max;    // 位置范围（PMAX）
    float v_min, v_max;    // 速度范围（VMAX）
    float kp_min, kp_max;  // Kp 范围（固定 0~500）
    float kd_min, kd_max;  // Kd 范围（固定 0~5）
    float t_min, t_max;    // 力矩范围（TMAX）
} DM_Motor_Params_t;
```

> **参数来源**：PMAX/VMAX/TMAX 是调试助手中设置的"控制幅值"参数，不同型号默认值不同。代码中目前只有 DM4310 的参数（PMAX=±12.5, VMAX=±30, TMAX=±10）。**实际使用前必须用调试助手读取电机内的设定值**，确保代码参数与电机一致，否则控制命令会发生等比例缩放。详见 [[01_extracted/motor/motor_params#MIT 模式参数获取]]。

## MIT 模式控制帧打包

```c
static void mit_ctrl(DM_Motor_t *motor, float pos, float vel, float kp, float kd, float torq) {
    const DM_Motor_Params_t *param = motor->params;

    uint16_t pos_tmp = float_to_uint(pos, param->p_min, param->p_max, 16);
    uint16_t vel_tmp = float_to_uint(vel, param->v_min, param->v_max, 12);
    uint16_t kp_tmp  = float_to_uint(kp, param->kp_min, param->kp_max, 12);
    uint16_t kd_tmp  = float_to_uint(kd, param->kd_min, param->kd_max, 12);
    uint16_t tor_tmp = float_to_uint(torq, param->t_min, param->t_max, 12);

    msg.data[0] = (pos_tmp >> 8);
    msg.data[1] = pos_tmp;
    msg.data[2] = (vel_tmp >> 4);
    msg.data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    msg.data[4] = kp_tmp;
    msg.data[5] = (kd_tmp >> 4);
    msg.data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    msg.data[7] = tor_tmp;

    BSP_CAN_SendMessage(&msg);
}
```

### float→uint 线性映射

```c
int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
```

将浮点物理量线性映射到定点整数：`x` 在 `[x_min, x_max]` 范围内映射到 `[0, 2^bits - 1]`。

示例：DM4310 力矩 t_ff=5.0 N·m，TMAX=10，则 `tor_tmp = (5.0 - (-10.0)) * (2^12-1) / 20.0 = 3071`。

映射原理详见 [[01_extracted/motor/motor_params#float↔uint 线性映射]]。

### 位打包布局

```
Byte 0-1:  pos[15:8] | pos[7:0]              (16 bit)
Byte 2-3:  vel[11:4] | vel[3:0] kp[11:8]     (12+4 bit)
Byte 4:    kp[7:0]                           (8 bit, kp 共 12 bit)
Byte 5-6:  kd[11:4] | kd[3:0] tor[11:8]      (12+4 bit)
Byte 7:    tor[7:0]                          (8 bit, tor 共 12 bit)
合计: 16 + 12 + 12 + 12 + 12 = 64 bit = 8 字节
```

协议详见 [[01_extracted/motor/motor_params#MIT 模式控制帧]]。

## MIT 模式的特殊用法

项目中达妙电机常用 MIT 模式发力矩（Kp=Kd=0，只给 t_ff）：

```c
// dm_ControlAndSend 中 MIT 模式
case DM_MIT_MODE:
    mit_ctrl(motor, 0, 0, 0, 0, base->controller.output);
    break;
```

pos=0, vel=0, kp=0, kd=0，只有 torq 是 PID/LQR 的输出。电机内部不做位置/速度闭环，完全由外部软件 PID 闭环计算力矩。

> 这种用法相当于把达妙电机当作力矩源，由 MCU 侧的 PID/LQR 控制器做闭环。与大疆电机（直接发电流值）的区别在于：达妙 MIT 模式发的是力矩（N·m），大疆发的是电流（A）。

## CAN 接收回调与 uint→float 解包

```c
static void dm_can_rx_callback(Can_Device *dev, const uint8_t *data, uint8_t len) {
    DM_Motor_t *motor = (DM_Motor_t *)dev->user_arg;
    const DM_Motor_Params_t *param = motor->params;

    motor->measure.id = data[0] & 0x0F;
    uint8_t error_code = (data[0] >> 4) & 0x0F;

    uint16_t tmp;
    tmp = (uint16_t)((data[1] << 8) | data[2]);
    motor->base.measure.single_round_angle = uint_to_float(tmp, param->p_min, param->p_max, 16);

    tmp = (uint16_t)((data[3] << 4) | data[4] >> 4);
    motor->base.measure.speed_rad = uint_to_float(tmp, param->v_min, param->v_max, 12);

    tmp = (uint16_t)(((data[4] & 0x0f) << 8) | data[5]);
    motor->measure.torque = uint_to_float(tmp, param->t_min, param->t_max, 12);

    motor->measure.T_Mos = (float)data[6];
    motor->measure.T_Rotor = (float)data[7];
}
```

反馈帧使用与发送相同的 PMAX/VMAX/TMAX 做反向映射。如果代码中的参数与电机内设定的不一致，反馈值也会错误。

反馈帧格式详见 [[01_extracted/motor/motor_params#反馈帧]]，错误码见 [[01_extracted/motor/motor_params#错误码]]。

## 多圈角度计算

```c
float current_angle = motor->base.measure.single_round_angle;
float diff = current_angle - motor->measure.last_single_round_angle;
float range = param->p_max - param->p_min;  // e.g. 25.0 for ±12.5
float half_range = range * 0.5f;            // 12.5

if (diff < -half_range) {
    diff += range;          // 正向溢出
    motor->measure.total_round++;
} else if (diff > half_range) {
    diff -= range;          // 反向溢出
    motor->measure.total_round--;
}

motor->base.measure.total_angle += diff;
```

与大疆的编码器溢出检测不同：大疆用 0~8191 整数判断（半圈=4096），达妙用 PMAX 范围判断（半圈=range/2）。原理相同：检测跳变超过半圈时修正圈数。

## 模式切换

达妙电机支持 4 种模式，初始化时选择：

```c
DM_Motor_t *Motor_DM_Init(Motor_Init_Config_s *config, uint32_t DM_Mode_type);
```

| 模式 | 宏定义 | 发送函数 | 数据格式 |
|------|--------|---------|---------|
| MIT | `DM_MIT_MODE (0x000)` | `mit_ctrl()` | 16bit pos + 12bit v/kp/kd/t |
| 位置速度 | `DM_POS_MODE (0x100)` | `pos_speed_ctrl()` | float pos + float vel |
| 速度 | `DM_SPD_MODE (0x200)` | `speed_ctrl()` | float vel (4字节) |
| 力位混控 | `DM_PSI_MODE (0x300)` | `psi_ctrl()` | float pos + uint16 vel + uint16 cur |

模式偏移加到 CAN_ID 上作为帧 ID。切换模式只需改变 `DM_Mode_type` 参数，帧 ID 自动变化。

## 命令帧

```c
void Motor_DM_Cmd(DM_Motor_t *motor, DMMotor_Mode_e cmd) {
    msg.data[0~6] = 0xFF;  // 前 7 字节固定 0xFF
    msg.data[7] = cmd;     // 命令字节

    BSP_CAN_SendMessage(&msg);
    BSP_DWT_Delay(0.0002f);  // 200us 间隔
}
```

| 命令 | 字节值 | 功能 |
|------|--------|------|
| 启动 | 0xFC | 使能电机 |
| 停止 | 0xFD | 失能电机 |
| 保存零点 | 0xFE | 当前位置设为零点 |
| 清除错误 | 0xFB | 清除错误码 |

初始化时自动发送清除错误 + 使能命令。命令帧详见 [[01_extracted/motor/motor_params#命令帧]]。

## 离线/禁用处理

```c
if (Module_Offline_get_device_status(base->offline_dev) == STATE_OFFLINE || base->setting.enableflag == 0) {
    // 清零 PID 积分项
    base->controller.speed_PID.Iout = 0;
    base->controller.angle_PID.Iout = 0;
    // 发零值帧（保持通信，避免电机因超时失能）
    switch (motor->mode_type) {
    case DM_MIT_MODE:
        mit_ctrl(motor, 0, 0, 0, 0, 0);  // 力矩=0
        break;
    case DM_POS_MODE:
        pos_speed_ctrl(motor, 0, 0);
        break;
    case DM_SPD_MODE:
        speed_ctrl(motor, 0);
        break;
    }
    return;
}
```

与大疆不同：达妙电机离线/禁用时**仍然发送零值帧**，而不是完全不发。因为达妙电机有超时失能机制（一段时间收不到命令自动失能），需要保持通信。
