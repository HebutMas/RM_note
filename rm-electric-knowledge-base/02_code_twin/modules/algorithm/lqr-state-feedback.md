# 算法层 LQR 模块文档

## 概述

基于线性二次型调节器 (Linear Quadratic Regulator) 的全状态反馈控制模块，

## 架构设计

### 数据结构

#### LQRErrorType — 错误类型枚举

```c
typedef enum
{
    LQR_ERROR_NONE          = 0, /* 正常 */
    LQR_MOTOR_BLOCKED_ERROR = 1, /* 电机堵转 */
} LQRErrorType;
```

#### LQR_Init_Config_s — 初始化配置

```c
typedef struct
{
    float   K[2];      /* 增益矩阵 [dim1] 或 [dim1, dim2] */
    uint8_t state_dim; /* 状态维度: 1=角速度控制, 2=角度+角速度 */
    float   max_out;   /* 输出限幅 (Nm), 对标电机 max_torque */
} LQR_Init_Config_s;
```

#### LQRInstance — 运行实例

| 字段 | 类型 | 说明 |
|------|------|------|
| `K[2]` | `float` | 增益矩阵 |
| `state_dim` | `uint8_t` | 状态维度 (1 或 2) |
| `ref` | `float` | 当前设定值 (rad 或 rad/s) |
| `measure` | `float` | 当前测量值 (rad 或 rad/s) |
| `output` | `float` | 计算输出 (Nm) |
| `max_out` | `float` | 输出限幅 |
| `blocked_count` | `uint16_t` | 堵转连续计数 |
| `errortype` | `LQRErrorType` | 当前错误类型 |

### 代码框架

```
内部辅助函数:
└── LQR_ErrorHandle()    — 电机堵转检测 + 保护

公开 API:
├── LQRInit()            — 初始化实例
└── LQRCalculate()       — 计算 LQR 输出
```

## API 接口说明

```c
void LQRInit(LQRInstance *lqr, LQR_Init_Config_s *config);
```

```c
float LQRCalculate(LQRInstance *lqr,float rad_degree,float rad_angular_velocity,float rad_ref);
```

## 使用示例

```c
#include "lqr.h"

static LQRInstance g_motor_lqr;

void motor_init(void)
{
    LQR_Init_Config_s cfg = {
        .state_dim = 2,              // 角度 + 角速度双状态
        .K         = {15.5f, 0.45f}, // 仿真/实调得到的增益
        .max_out   = 3.0f,           // max_torque ≈ 3 Nm
    };
    LQRInit(&g_motor_lqr, &cfg);
}

float motor_control(float measure_angle, float measure_speed, float target_angle)
{
    float torque = LQRCalculate(&g_motor_lqr, measure_angle, measure_speed, target_angle);
    return torque;
}
```

## 注意事项

1. **弧度制**: `rad_degree` 和 `rad_ref` 使用弧度 (rad)，`rad_angular_velocity` 使用弧度/秒 (rad/s)。

2. **维度选择**:
   - `state_dim = 1`: 仅控制角速度，`rad_ref` 视为目标速度。
   - `state_dim = 2`: 控制角度位置，`rad_ref` 视为目标角度，角速度作为阻尼项。


## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-02-06 | 初始版本 |
| v1.1 | 2026-05 | 重构数据结构，增加电机堵转保护 |
