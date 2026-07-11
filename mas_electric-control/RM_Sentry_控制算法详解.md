# 哨兵控制算法详解 —— PID、LQR 与系统实战

## 目录

1. [控制算法总览](#1-控制算法总览)
2. [PID 控制器深度解析](#2-pid-控制器深度解析)
3. [LQR 线性二次型调节器](#3-lqr-线性二次型调节器)
4. [PID vs LQR：何时用哪个](#4-pid-vs-lqr何时用哪个)
5. [哨兵实战：云台控制](#5-哨兵实战云台控制)
6. [哨兵实战：底盘控制](#6-哨兵实战底盘控制)
7. [哨兵实战：发射机构控制](#7-哨兵实战发射机构控制)
8. [LQR 设计工具 (Python)](#8-lqr-设计工具-python)
9. [增益调参与稳定性分析](#9-增益调参与稳定性分析)
10. [嵌入式实现细节与优化](#10-嵌入式实现细节与优化)
11. [故障检测与保护](#11-故障检测与保护)
12. [开源社区方案拓展](#12-开源社区方案拓展)

---

## 1. 控制算法总览

### 1.1 哨兵机器人的控制需求

哨兵机器人是一个多自由度机电系统，需要同时控制：

```
┌───────────────────────────────────────────────┐
│                  哨兵控制系统                    │
│                                               │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐       │
│  │ 云台控制  │  │ 底盘控制  │  │ 发射控制  │       │
│  │ (3轴)    │  │ (8电机)  │  │ (3电机)  │       │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘       │
│       │              │              │           │
│  YAW (大+小)    Swerve 4WD   摩擦轮×2 + 拨盘   │
│  PITCH          DRIVE×4                        │
│                 STEER×4                        │
└───────────────────────────────────────────────┘
```

### 1.2 算法选型总表

| 控制轴 | 电机 | 环类型 | 算法 | 状态维度 | 原因 |
|--------|------|--------|------|---------|------|
| 大 YAW | GM6020 | 角度环 | LQR | 2 维 | 大惯量, 需抑制超调 |
| 小 YAW | GM6020 | 角度环 | LQR | 2 维 | 需要快速响应目标 |
| PITCH | DM4310 | 角度环 | LQR | 2 维 | 高精度位置控制 |
| 驱动轮×4 | M3508 | 速度环 | LQR | 1 维 | 简洁高效, 稳态好 |
| 转向轮×4 | GM6020 | 角度环 | LQR | 2 维 | 精确角度控制 |
| 摩擦轮×2 | M3508 | 速度环 | LQR | 1 维 | 恒速控制 |
| 拨盘 | M2006 | 速度环 | LQR | 1 维 | 速度控制 |
| 底盘跟随 | - | - | PID | 1 维 | 简单的误差跟随 |

### 1.3 为什么不只用 PID

传统的 RoboMaster 方案几乎都使用 PID，但本项目选择了 **以 LQR 为主，PID 为辅** 的混合策略：

| 场景    | PID 的问题      | LQR 的优势         |
| ----- | ------------ | --------------- |
| 大惯量云台 | 调参困难, 易超调/震荡 | K 矩阵系统性解算, 天生稳定 |
| 多轴耦合  | 各轴独立调参, 忽略耦合 | 可扩展为 MIMO 控制    |
| 扭矩受限  | 容易饱和, 无预测能力  | Q/R 权重直观反映约束    |
| 不同负载  | 增益需重新手工调试    | 仅需更新模型参数        |

---

## 2. PID 控制器深度解析

### 2.1 位置式 PID 基础

![PID 控制器框图](https://upload.wikimedia.org/wikipedia/commons/4/43/PID_en.svg)
*▲ 经典 PID 反馈控制框图 — P(比例)、I(积分)、D(微分) 三条并行支路 (图片来源: Wikipedia)*

本项目实现了 **位置式 PID**（Positional PID），基本公式：

```
Output = Kp × Err  +  Ki × Σ(Err × dt)  +  Kd × (Err - Last_Err) / dt
          └─比例项─┘   └───积分项─────────┘   └──────微分项──────────┘
```

### 2.2 PID 数据结构

```c
// pid.h:52-93
typedef struct {
    // --- 配置参数 ---
    float Kp;            // 比例增益
    float Ki;            // 积分增益
    float Kd;            // 微分增益
    float MaxOut;        // 输出限幅
    float DeadBand;      // 死区宽度

    // --- 优化参数 ---
    PID_Improvement_e Improve;        // 启用的优化环节 (位掩码)
    float IntegralLimit;             // 积分限幅
    float CoefA, CoefB;              // 变速积分参数
    float Output_LPF_RC;             // 输出滤波器时间常数
    float Derivative_LPF_RC;         // 微分滤波器时间常数

    // --- 运行状态 ---
    float Measure, Last_Measure;     // 测量值
    float Err, Last_Err;             // 误差
    float Pout, Iout, Dout, ITerm;   // 各环节输出
    float Output, Last_Output;       // 最终输出
    float Ref;                       // 设定值
    uint32_t DWT_CNT;                // 时间戳 (DWT 定时器)
    float dt;                        // 两次计算的时间间隔

    PID_ErrorHandler_t ERRORHandler; // 故障检测
} PIDInstance;
```

### 2.3 七大优化环节

本项目 PID 实现了 **7 种优化环节**，通过位掩码 `Improve` 按需启用：

```c
// pid.h:25-36
typedef enum {
    PID_IMPROVE_NONE                = 0x00, // 无优化
    PID_Integral_Limit              = 0x01, // 积分限幅
    PID_Derivative_On_Measurement   = 0x02, // 微分先行
    PID_Trapezoid_Intergral         = 0x04, // 梯形积分
    PID_Proportional_On_Measurement = 0x08, // 比例先行 (保留)
    PID_OutputFilter                = 0x10, // 输出滤波
    PID_ChangingIntegrationRate     = 0x20, // 变速积分
    PID_DerivativeFilter            = 0x40, // 微分滤波
    PID_ErrorHandle                 = 0x80, // 堵转检测
} PID_Improvement_e;
```

#### ① 积分限幅 (`PID_Integral_Limit = 0x01`)

防止积分饱和 (Integral Windup)：

```c
static void f_Integral_Limit(PIDInstance *pid) {
    float temp_Iout   = pid->Iout + pid->ITerm;       // 累积后积分
    float temp_Output = pid->Pout + pid->Iout + pid->Dout;

    if (abs(temp_Output) > pid->MaxOut) {
        if (pid->Err * pid->Iout > 0) {
            pid->ITerm = 0;   // 输出饱和且积分在累积 → 停止积分
        }
    }
    // 硬限幅到 ±IntegralLimit
    if (temp_Iout > pid->IntegralLimit) { pid->ITerm = 0; pid->Iout = pid->IntegralLimit; }
    if (temp_Iout < -pid->IntegralLimit) { pid->ITerm = 0; pid->Iout = -pid->IntegralLimit; }
}
```

#### ② 微分先行 (`PID_Derivative_On_Measurement = 0x02`)

避免设定值跳变引起的微分冲击：

```c
// 传统: Dout = Kd × (Err - Last_Err) / dt   ← 设定值跳变时 Dout 巨大
// 优化: Dout = Kd × (Last_Measure - Measure) / dt  ← 只看反馈变化
static void f_Derivative_On_Measurement(PIDInstance *pid) {
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}
```

#### ③ 梯形积分 (`PID_Trapezoid_Intergral = 0x04`)

用梯形面积替代矩形面积，提高积分精度：

```c
static void f_Trapezoid_Intergral(PIDInstance *pid) {
    // 梯形面积 = (上底 + 下底) × 高 / 2
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2.0f) * pid->dt;
}
```

传统矩形积分 vs 梯形积分：

```
矩形积分:                        梯形积分:
Err │  ┌─┐                       Err │  ┌─┐
    │  │ │ 面积 = Err × dt          │  │╱│ 面积 = (Err1+Err2)/2 × dt
    │  │ │                          │  │ │
    └──┴─┴──→ t                     └──┴─┴──→ t
```

#### ④ 变速积分 (`PID_ChangingIntegrationRate = 0x20`)

误差大时减弱积分，误差小时加强积分——防止大误差阶段的积分过冲：

```c
static void f_Changing_Integration_Rate(PIDInstance *pid) {
    if (pid->Err * pid->Iout > 0) {  // 积分在累积
        if (abs(pid->Err) <= pid->CoefB)
            return;                  // |err| ≤ B: 全积分
        if (abs(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - abs(pid->Err) + pid->CoefB) / pid->CoefA;
        else                         // |err| > A+B: 关闭积分
            pid->ITerm = 0;
    }
}
```

#### ⑤ 微分滤波 (`PID_DerivativeFilter = 0x40`)

对微分项做一阶低通滤波，抑制高频噪声：

```c
static void f_Derivative_Filter(PIDInstance *pid) {
    // 一阶低通: y[k] = α·x[k] + (1-α)·y[k-1],  α = dt/(RC+dt)
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt)
              + pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}
```

#### ⑥ 输出滤波 (`PID_OutputFilter = 0x10`)

对最终输出做低通滤波，平滑电机指令：

```c
static void f_Output_Filter(PIDInstance *pid) {
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt)
                + pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}
```

#### ⑦ 堵转检测 (`PID_ErrorHandle = 0x80`)

检测电机堵转并自动停止输出：

```c
static void f_PID_ErrorHandle(PIDInstance *pid) {
    if (fabsf(pid->Output) < pid->MaxOut * 0.001f
        || fabsf(pid->Ref) < 0.0001f) return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f) {
        pid->ERRORHandler.ERRORCount++;  // 误差 > 95% 设定值, 计数
    } else {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500) {  // 500 个周期 (1s @ 2ms)
        pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    }
}
```

### 2.4 PID 计算主流程

```c
float PIDCalculate(PIDInstance *pid, float measure, float ref) {
    // 1. 堵转检测
    if (pid->Improve & PID_ErrorHandle) f_PID_ErrorHandle(pid);

    // 2. 时间间隔 (DWT 周期计数器)
    pid->dt = BSP_DWT_GetDeltaT(&pid->DWT_CNT);

    // 3. 计算基本 PID
    pid->Measure = measure;
    pid->Ref     = ref;
    pid->Err     = pid->Ref - pid->Measure;

    if (abs(pid->Err) > pid->DeadBand) {
        pid->Pout  = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout  = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

        // 4. 按位掩码依次执行启用的优化
        if (pid->Improve & PID_Trapezoid_Intergral)       f_Trapezoid_Intergral(pid);
        if (pid->Improve & PID_ChangingIntegrationRate)   f_Changing_Integration_Rate(pid);
        if (pid->Improve & PID_Derivative_On_Measurement) f_Derivative_On_Measurement(pid);
        if (pid->Improve & PID_DerivativeFilter)          f_Derivative_Filter(pid);
        if (pid->Improve & PID_Integral_Limit)            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;
        pid->Output = pid->Pout + pid->Iout + pid->Dout;

        if (pid->Improve & PID_OutputFilter) f_Output_Filter(pid);
        f_Output_Limit(pid);  // 始终执行输出限幅
    } else {
        pid->Output = 0;  // 死区内 → 输出清零
        pid->ITerm  = 0;
    }

    // 5. 保存状态
    pid->Last_Measure = pid->Measure;
    pid->Last_Err     = pid->Err;
    pid->Last_Dout    = pid->Dout;
    pid->Last_Output  = pid->Output;

    // 6. 堵转保护
    if (pid->ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR) {
        return 0;  // 输出清零, 保护电机
    }
    return pid->Output;
}
```

### 2.5 PID 哨兵实战：底盘跟随

底盘跟随云台 (Chassis Follow Gimbal Yaw) 是少数使用 PID 的场景——需求简单，不需要复杂建模：

```c
// chassis_func.c:28-29
PID_Init_Config_s config = {
    .MaxOut = 5,           // 最大输出 5 (角速度)
    .IntegralLimit = 0.01, // 积分限幅很小 (稳态微调)
    .DeadBand = 10,        // 死区 10° (避免抖动)
    .Kp = 0.1,             // 小比例增益 (平滑跟随)
    .Ki = 0,               // 无积分 (跟随不需要零误差)
    .Kd = 0.001,           // 小微分 (抑制震荡)
    .Improve = 0x01,       // 仅启用积分限幅
};
PIDInit(&chassis_follow_pid, &config);

// 运行中:
PIDCalculate(&chassis_follow_pid, chassis_cmd->offset_angle, 0);
chassis_wz = chassis_follow_pid.Output;
```

---

## 3. LQR 线性二次型调节器

### 3.1 理论基础

**LQR (Linear Quadratic Regulator)** 是一种最优控制方法。给定线性系统：

```
ẋ = A·x + B·u
```

LQR 求解反馈增益矩阵 **K**，使得控制律 **u = -K·x** 最小化代价函数：

```
J = ∫ (xᵀ·Q·x + uᵀ·R·u) dt

其中:
  Q = 状态权重矩阵 (我们希望状态收敛到 0)
  R = 控制权重矩阵 (我们希望控制量尽量小)
```

**核心优势**：不需要反复手工调 Kp/Ki/Kd，只需设置 Q 和 R 的权重，算法自动解算最优增益。

### 3.2 一维 LQR：速度环 (M3508 驱动轮)

对于纯速度控制，系统可简化为一阶：

```
系统方程:  ẋ = τ / J        (J = 等效转动惯量)
状态:      x = [ω]          (1维, 仅角速度)
LQR 输出:  u = -K[0] × (ω - ω_ref)
```

```c
// chassis_func.c:43-46
.controller_init_config = {.lqr_init = {
    .K = {0.008f},       // 单一增益
    .state_dim = 1,      // 一维状态
}},
.setting_init_config = {
    .loop_type = SPEED_LOOP,        // 速度环
    .algorithm_type = CONTROL_LQR,  // LQR 算法
},
```

**LQR 计算代码 (`lqr.c`)**：

```c
float LQRCalculate(LQRInstance *lqr, float rad_degree,
                   float rad_angular_velocity, float rad_ref) {
    if (lqr->state_dim == 1) {
        // 一维: Output = -K[0] × (ω - ω_ref)
        lqr->measure = rad_angular_velocity;
        lqr->ref     = rad_ref;
        float err    = lqr->measure - lqr->ref;
        lqr->output  = -(lqr->K[0] * err);
    }
    return lqr->output;
}
```

### 3.3 二维 LQR：角度+角速度 (GM6020 云台/转向)

对于角度控制，需要同时反馈位置和速度：

```
系统方程:
  θ̇ = ω
  ω̇ = τ / J

状态向量: x = [θ, ω]ᵀ         (2维)
控制律:   u = -K[0]·θ - K[1]·ω = -(K[0]·err_angle + K[1]·ω)
```

```c
// gimbal_func.c:70-72 (小 YAW)
.lqr_init = {
    .K = {5.47f, 0.56f},   // [Kθ, Kω]
    .state_dim = 2,         // 二维状态
},

// gimbal_func.c:107-109 (大 YAW)
.lqr_init = {
    .K = {22.36f, 3.15f},  // 注意：大 YAW 的 K 远大于小 YAW
    .state_dim = 2,
},

// gimbal_func.c:146-149 (PITCH)
.lqr_init = {
    .K = {38.73f, 2.85f},  // PITCH 负载大，增益最高
    .state_dim = 2,
},
```

**LQR 计算代码 (二维)**：

```c
// lqr.c:30-34
float LQRCalculate(LQRInstance *lqr, float rad_degree,
                   float rad_angular_velocity, float rad_ref) {
    // ... state_dim == 2
    lqr->measure = rad_degree;
    lqr->ref     = rad_ref;
    float err    = lqr->measure - lqr->ref;
    lqr->output  = -(lqr->K[0] * err) - lqr->K[1] * rad_angular_velocity;
    //              ^^^^^^^^^^^^^^^^^^    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //              位置误差反馈            速度阻尼反馈
    return lqr->output;
}
```

**物理含义**：`-K[0]·err` 像弹簧（位置偏离越大回复力越大），`-K[1]·ω` 像阻尼器（速度越快阻力越大）。两者配合实现快速且无超调的收敛。

### 3.4 外部传感器反馈

LQR 的一大特色是支持**外部传感器作为反馈源**，不依赖电机自带编码器：

```c
// gimbal_func.c:67-68 (小 YAW)
.other_angle_feedback_ptr = &ins->YawTotalAngle_rad,  // IMU 的 YAW 角度
.other_speed_feedback_ptr = &bmi088_dev->gyro[2],      // BMI088 Z 轴陀螺仪角速度

.setting_init_config = {
    .angle_feedback_source = 1,  // 1 = 使用外部反馈 (非电机编码器)
    .speed_feedback_source = 1,  // 1 = 使用外部反馈
},
```

这使得 LQR 可以利用 IMU 的高带宽角速度数据（1kHz+）来改善动态响应，而不依赖电机编码器的低速更新。

---

## 4. PID vs LQR：何时用哪个

### 4.1 决策流程图

```
需要控制的系统
│
├── 模型已知且近似线性？
│   ├── 是 → LQR (系统性地设计, 自动保证稳定性)
│   └── 否 → PID
│
├── 需要零稳态误差？
│   ├── 是 → PID (有积分器) 或 LQR + 积分增广
│   └── 否 → LQR 或 P/PD
│
├── 多输入多输出 (MIMO)？
│   ├── 是 → LQR (天然支持多变量)
│   └── 否 → PID 或 LQR 均可
│
└── 约束重要 (扭矩/电流)？
    ├── 是 → LQR (通过 R 权重间接约束)
    └── 否 → 均可
```

### 4.2 对比总结

| 维度 | PID | LQR |
|------|-----|-----|
| **需要模型** | 否 | 是 (A, B 矩阵) |
| **调参方式** | 手工试凑 Kp, Ki, Kd | 设置 Q, R → 数学解算 K |
| **稳定性保证** | 经验性 (可能震荡) | 数学保证 (若系统可控) |
| **鲁棒性** | 好 (对模型不敏感) | 对模型误差较敏感 |
| **计算复杂度** | O(1), 极低 | O(n), 低 (n=状态维度) |
| **积分器** | 内置 | 需额外增广 |
| **嵌入式可行性** | 极简 | 简洁 (仅矩阵乘法) |
| **调参直觉** | 直觉强 (P=刚度, D=阻尼) | 需通过仿真建立直觉 |

### 4.3 本项目的选择逻辑

```
哨兵各控制轴的选择:

云台 YAW (大惯量)      → LQR 2维 ✓  (需要系统性的阻尼设计)
云台 PITCH (高精度)    → LQR 2维 ✓  (模型精确, 最优控制有意义)
驱动轮速度             → LQR 1维 ✓  (一阶纯惯量, LQR = P 控制器)
转向轮角度             → LQR 2维 ✓  (同云台, 需要阻尼)
摩擦轮速度             → LQR 1维 ✓  (同驱动轮)
拨盘速度               → LQR 1维 ✓  (同驱动轮)
底盘跟随云台           → PID 1维 ✓  (需求简单, 经验性调参足够)
缓冲能量管理           → PID 1维 ✓  (模型不明确, PID 更合适)
```

---

## 5. 哨兵实战：云台控制

### 5.1 云台机械架构

![RoboMaster S1 云台结构](https://www.manualslib.com/manual/1615507/Dji-Robomaster-S1.html?page=17)
*▲ RoboMaster S1 云台 — Yaw 轴电机(底座旋转) + Pitch 轴电机(俯仰) (图片来源: DJI 说明书)*

哨兵云台采用 **双层 YAW + 单 PITCH** 架构：

```
┌──────────────────────────────────────┐
│              底盘 (BASE)              │
│  ┌────────────────────────────────┐  │
│  │     大云台 YAW (GM6020)         │  │
│  │     角度范围: 0° ~ 360°         │  │
│  │  ┌──────────────────────────┐  │  │
│  │  │  小云台 YAW (GM6020)      │  │  │
│  │  │  角度范围: -90° ~ +90°    │  │  │
│  │  │  ┌────────────────────┐  │  │  │
│  │  │  │  PITCH (DM4310)    │  │  │  │
│  │  │  │  范围: -20° ~ +40° │  │  │  │
│  │  │  │  [发射机构]         │  │  │  │
│  │  │  └────────────────────┘  │  │  │
│  │  └──────────────────────────┘  │  │
│  └────────────────────────────────┘  │
└──────────────────────────────────────┘
```

### 5.2 大小 YAW 协调控制

这是哨兵云台最精妙的设计——大 YAW 跟随小 YAW，实现快速定位 + 大范围覆盖：

```c
// gimbal_func.c:177-178
// 大 YAW 的目标 = 小 YAW 在大坐标系中的实际位置
big_yaw_offset = wt606_device->YawTotalAngle_rad       // 大 YAW 当前角度
               + ((float)(small_yaw_motor->measure.ecd  // 小 YAW 编码器
                   - SMALL_YAW_ALIGN_ECD)               // 对齐偏置
                   * (2.0f * PI / 8192.0f));            // ECD → rad

// 控制输出:
Motor_DJI_SetRef(big_yaw_motor, big_yaw_offset);     // 大 YAW 跟随
Motor_DJI_SetRef(small_yaw_motor, gimbal_cmd->yaw);  // 小 YAW 定位
```

**工作原理**：

```
初始状态:  大 YAW = 0°, 小 YAW = 0° (对齐状态)
         ┌──────────┐
         │ 大 + 小   │ → 枪口指向 0°
         └──────────┘

目标出现 @ +120°:
  ① 小 YAW 电机旋转至 +60° (小云台最大偏角)
  ② 大 YAW 检测到 offset → 开始跟随旋转至 +60°
  ③ 小 YAW 可以在 ±60° 内继续微调

最终效果: 枪口指向 120°, 小 YAW 仍有余量做精细调整
         大 YAW = 60°, 小 YAW = 60° (相对大云台)
```

### 5.3 自动搜索模式

当视觉未发现目标时，云台进入自动搜索：

```c
// gimbal_func.c:211-225
case gimbal_auto_mode:
    if (gimbal_cmd->auto_search == 1) {
        // YAW: 匀速旋转 (1.2 rad/s)
        search_yaw_angle += SEARCH_YAW_SPEED_RPS * SEARCH_TASK_PERIOD_S;
        Motor_DJI_SetRef(small_yaw_motor, search_yaw_angle);

        // PITCH: 正弦摆动 (幅值 0.3 rad, 频率 0.8 Hz)
        search_pitch_time += SEARCH_TASK_PERIOD_S;
        float search_pitch_ref = SEARCH_PITCH_AMPLITUDE
                               * sinf(2.0f * PI * SEARCH_PITCH_FREQ
                                      * search_pitch_time);
        Motor_DM_SetRef(pitch_motor, -fabsf(search_pitch_ref));
    }
```

**搜索轨迹可视化**：

```
PITCH
  │   ╱╲    ╱╲    ╱╲    ╱╲    正弦摆动, 仅有向下分量 (fabsf)
  │  ╱  ╲  ╱  ╲  ╱  ╲  ╱  ╲
  │ ╱    ╲╱    ╲╱    ╲╱    ╲
  └──────────────────────────────→ YAW
       匀速旋转 (1.2 rad/s)

覆盖区域: 螺旋形逐渐扩展的扫描区域
```

### 5.4 全手动控制 (陀螺仪模式)

```c
// gimbal_func.c:185-194
case gimbal_gyro_mode:
    // 大 YAW 跟随小 YAW (保持协调)
    Motor_DJI_SetRef(big_yaw_motor, big_yaw_offset);
    // 小 YAW 跟随遥控器输入 (累积角度)
    Motor_DJI_SetRef(small_yaw_motor,
                     (gimbal_cmd->yaw * DEGREE_2_RAD) + auto_yaw_offset);
    // PITCH 跟随遥控器输入
    Motor_DM_SetRef(pitch_motor, gimbal_cmd->pitch * DEGREE_2_RAD);
```

---

## 6. 哨兵实战：底盘控制

### 6.1 Swerve 舵轮底盘运动学

![哨兵导航系统](https://raw.githubusercontent.com/suihan21/mid-360-NEXTE_Sentry_Nav/main/doc/sentry_navigation.png)
*▲ 哨兵机器人导航系统 — 搭载 MID-360 激光雷达 (图片来源: NEXTE Sentry Nav)*

哨兵采用 **Swerve Drive**（舵轮）底盘，每个轮子模块包含独立的驱动和转向：

```c
// chassis_func.c:19-24
static const Chassis_Swerve_Config_s chassis_swerve_config = {
    .align_rad = {              // 各轮子的对齐角度 (rad)
        4458 * 0.000767f,       // LF: 前左
        1010 * 0.000767f,       // LB: 后左
        7177 * 0.000767f,       // RB: 后右
        3756 * 0.000767f,       // RF: 前右
    },
    .decele_ratio   = 16.0f,   // 减速比
    .radius_wheel_m = 0.12f,   // 轮子半径 12cm
    .wheel_r        = 0.5f,    // 轮子到底盘中心距离 50cm
};
```

### 6.2 坐标系转换

云台系速度 → 底盘系速度的旋转映射：

```c
// chassis_func.c:260-265
// θ = 云台与底盘的夹角 (offset_angle)
float total_angle_rad = chassis_cmd->offset_angle * DEGREE_2_RAD;
cos_theta = arm_cos_f32(total_angle_rad);
sin_theta = arm_sin_f32(total_angle_rad);

// 2D 旋转矩阵
chassis_vx = chassis_cmd->vx * cos_theta - chassis_cmd->vy * sin_theta;
chassis_vy = chassis_cmd->vx * sin_theta + chassis_cmd->vy * cos_theta;
```

### 6.3 底盘模式切换

```c
typedef enum {
    chassis_zero_force = 0,     // 无力模式 (安全停止)
    chassis_follow_gimbal_yaw,  // 跟随云台 (底盘自旋对齐云台方向)
    chassis_rotate,             // 自旋
    chassis_rotate_reverse,     // 反向自旋
    chassis_automode,           // 自动模式 (视觉/导航控制)
} chassis_mode_e;
```

### 6.4 离线保护

所有电机都注册了离线检测——当 CAN 通信超时或裁判系统断连时，自动进入安全模式：

```c
// chassis_func.c:195-202
if (!Module_Offline_get_device_status(chassis_motors[0]->base.offline_dev) &&
    !Module_Offline_get_device_status(chassis_motors[1]->base.offline_dev) &&
    // ... 所有 8 个电机均在线 ...
    !Module_Offline_get_device_status(chassis_motors[7]->base.offline_dev))
{
    // 正常控制
} else {
    // 任一电机离线 → 全部停止
    Motor_DJI_Stop(all_motors);
}
```

---

## 7. 哨兵实战：发射机构控制

### 7.1 发射机构组成

![RoboMaster AI Robot 发射机构](https://www.manualslib.com/manual/1896323/Icra-Dji-Robomaster-Ai-Challenge-Ai-Robot-2019.html?page=13)
*▲ RoboMaster AI Robot 发射机构 — 两个反向旋转摩擦轮 + 17mm 弹丸 (图片来源: DJI 说明书)*

```
┌─────────────────────────────────────┐
│           发射机构 (Shoot)            │
│                                     │
│  ┌─────────┐    ┌─────────┐         │
│  │ 左摩擦轮  │    │ 右摩擦轮  │        │
│  │ M3508    │    │ M3508    │        │
│  │ 6800 RPM │    │ -6800 RPM│        │
│  └────┬─────┘    └────┬─────┘        │
│       │               │              │
│       └───────┬───────┘              │
│               ▼                      │
│         ┌──────────┐                 │
│         │   弹丸    │ → 发射!         │
│         └──────────┘                 │
│               ▲                      │
│         ┌──────────┐                 │
│         │   拨盘    │                 │
│         │ M2006     │                 │
│         │ 4000~8000 │                 │
│         │ RPM       │                 │
│         └──────────┘                 │
└─────────────────────────────────────┘
```

### 7.2 摩擦轮控制 (LQR 速度环)

```c
// shoot_func.c:30-33
.controller_init_config = {.lqr_init = {
    .K = {0.0011f},      // 低增益 → 平滑加速
    .state_dim = 1,
}},
.setting_init_config = {
    .loop_type = SPEED_LOOP,
    .algorithm_type = CONTROL_LQR,
},
```

发射时设定速度 6800 RPM → 转换为 rad/s → LQR 锁定该速度。

### 7.3 拨盘控制 (三种发射模式)

```c
// shoot_func.c:128-142
switch (shoot_cmd->load_mode) {
case load_stop:
    Motor_DJI_SetRef(loader, 0);                      // 停止
    break;
case load_1_bullet:
    Motor_DJI_SetRef(loader, -4000 * RPM_2_RAD_PER_SEC);  // 单发
    break;
case load_burstfire:
    Motor_DJI_SetRef(loader, -8000 * RPM_2_RAD_PER_SEC);  // 连发
    break;
}
```

### 7.4 热量管理

裁判系统通过 `shooter_17mm_heat` 回传枪管热量，发射机构需要据此决定是否允许发射。本系统将其透传到云台板：

```c
// robot_func.c (chassis):10-26
referee_data->shooter_heat_pct = power_heat_data->shooter_17mm_heat;
```

---

## 8. LQR 设计工具 (Python)

### 8.1 工具概述

`lqr.py` 是本项目配套的 LQR 增益设计工具，位于 `rm_embedded/sentry/lqr.py`。它使用 `scipy` 解算 Riccati 方程，自动计算最优增益矩阵 K。

### 8.2 系统建模

```python
# lqr.py:7-22
J_yaw = 0.2         # YAW 轴转动惯量 (kg·m²)
J_pitch = 0.04      # PITCH 轴转动惯量
R_yaw = 1           # 控制权重
R_pitch = 1

# 状态空间模型: ẋ = Ax + Bu
# x = [θ, ω]ᵀ

# YAW 轴
A_yaw = np.array([[0, 1],
                  [0, 0]])          # θ̇ = ω,  ω̇ = τ/J
B_yaw = np.array([[0],
                  [1/J_yaw]])

# PITCH 轴 (同理)
A_pitch = np.array([[0, 1],
                    [0, 0]])
B_pitch = np.array([[0],
                    [1/J_pitch]])
```

### 8.3 Riccati 方程求解

```python
# lqr.py:28-69
def calculate_lqr(A, B, Q, R, tau_max, axis_name):
    # 解算代数 Riccati 方程: AᵀS + SA - SBR⁻¹BᵀS + Q = 0
    S = solve_continuous_are(A, B, Q, R)

    # 最优反馈增益: K = R⁻¹BᵀS
    K = np.linalg.inv(np.array([[R]])) @ B.T @ S

    # 验证闭环稳定性
    A_cl = A - B @ K
    eigenvalues = np.linalg.eigvals(A_cl)
    stability = np.all(np.real(eigenvalues) < 0)

    # 仿真验证
    t = np.linspace(0, 5, 500)
    def dynamics(x, t):
        return (A - B @ K) @ x
    x0 = np.array([0.5, 0.0])
    x = odeint(dynamics, x0, t)

    return {'K': K, 'eigenvalues': eigenvalues, ...}
```

### 8.4 Q/R 权重调参

```python
# YAW 轴: 重视位置精度, 允许适度控制量
Q_yaw = np.array([[100, 0],   # Q[0,0]=100: 位置误差权重
                  [0, 1]])    # Q[1,1]=1:   速度误差权重

# PITCH 轴: 极高位置精度要求, 更激进
Q_pitch = np.array([[1000, 0],  # Q[0,0]=1000: 极强的位置约束
                    [0, 10]])   # Q[1,1]=10:   中等速度约束
```

**Q 矩阵的物理含义**：

| 参数 | 增大效果 |
|------|---------|
| Q[0,0] (位置权重) | 更快收敛, 更小的稳态误差, 但更大的控制量 |
| Q[1,1] (速度权重) | 更强的速度阻尼, 更平滑的响应 |
| R (控制权重) | 更保守的控制, 保护电机, 但响应变慢 |

### 8.5 输出分析

```python
print(f"K 矩阵: {result_yaw['K']}")        # K = [Kθ, Kω]
print(f"闭环特征值: {result_eigenvalues}")   # 负实部 = 稳定
print(f"稳定时间: {result_settling_time}s") # 到 5% 误差的时间
print(f"最大扭矩: {result_max_tau} Nm")     # 仿真中的最大扭矩
print(f"性能指标 (S 迹): {result_performance}") # 越小越好
```

### 8.6 从 Python 到 C 的工作流

```
Python (离线设计)                  C (嵌入式运行)
─────────────────                  ────────────────
1. 测量 J (转动惯量)      ──┐
2. 设定 Q, R 权重         ──┤     const float K[2] = {
3. solve_continuous_are()  ──┤─→     Kθ,  // 来自 Python
4. 验证 stability          ──┤      Kω   // 来自 Python
5. 仿真 settling_time      ──┘     };
6. 检查 max_tau 不超限
                            LQRCalculate(&lqr, angle, vel, ref);
```

---

## 9. 增益调参与稳定性分析

### 9.1 调参通用原则

```
步骤 1: 确定系统模型
  □ 测量或估算转动惯量 J
  □ 确认电机最大扭矩和扭矩常数
  □ 建立状态空间方程

步骤 2: 初始权重选择
  □ R = 1 (归一化)
  □ Q = diag([1, 0.1]) (适度位置权重, 低速度权重)
  □ 仿真, 观察响应

步骤 3: 迭代优化
  □ 响应太慢 → 增大 Q[0,0]
  □ 超调太大 → 增大 Q[1,1] (增加阻尼)
  □ 扭矩超限 → 增大 R
  □ 震荡不收敛 → 检查模型精度, 可能 J 测量有误

步骤 4: 嵌入式验证
  □ 将 K 写入 MCU
  □ 实测阶跃响应
  □ 调整 Q/R 直到满意
```

### 9.2 哨兵各轴 K 值解读

```
            K[0] (位置)   K[1] (速度)   | 等效特性
大 YAW      22.36         3.15          | 强刚度 + 中等阻尼 → 快速跟随, 无超调
小 YAW       5.47         0.56          | 柔顺跟随 + 轻阻尼 → 平滑运动
PITCH       38.73         2.85          | 极强刚度 + 强阻尼 → 高精度定位
M3508 驱动   0.008         (1维)        | 纯速度环, 柔和平滑
M3508 摩擦   0.0011        (1维)        | 极柔顺, 缓慢加速 (保护机械)
M2006 拨盘   0.005         (1维)        | 中等速度环
```

**观察**：
- 大 YAW 的 K[0] (22.36) 远大于小 YAW (5.47) → 大云台需要更快地跟随小云台的偏移
- PITCH 的 K[0] (38.73) 最大 → DM4310 扭矩大 (10Nm)，可承受更高的位置增益
- 摩擦轮的 K (0.0011) 最小 → 摩擦轮是纯惯量负载，低增益即可

### 9.3 稳定性检查清单

```
□ 闭环特征值全部在左半平面 (Re(λ) < 0)
□ 控制量不超出电机最大扭矩 (考虑 80% 安全余量)
□ 阶跃响应无震荡或超调 < 5%
□ 对不同初始条件的响应一致收敛
□ 电机实际输出不饱和 (避免积分 windup)
□ 电机运行温度正常 (大扭矩长时间运行不发热)
```

---

## 10. 嵌入式实现细节与优化

### 10.1 ARM CMSIS-DSP 加速

PID 实现使用了 ARM 的 CMSIS-DSP 库：

```c
// pid.h:16
#include "arm_math.h"

// 实际使用的 CMSIS 函数:
// - arm_cos_f32()   余弦 (用于坐标变换)
// - arm_sin_f32()   正弦 (用于坐标变换)
// - arm_math.h 提供的 abs/fabsf 等
```

### 10.2 DWT 精确计时

PID/LQR 的微分和积分项依赖精确的时间间隔。本项目使用 ARM Cortex-M 的 **DWT (Data Watchpoint and Trace)** 周期计数器：

```c
// pid.c:162
pid->dt = BSP_DWT_GetDeltaT(&pid->DWT_CNT);

// DWT 优势:
// - 精度: 1 CPU 周期 (~5.9ns @ 168MHz)
// - 零开销: 不需要定时器中断
// - 无累积误差: 直接读硬件计数器
```

### 10.3 死区 (Dead Band)

防止在设定值附近的微小抖动引发电机高频震荡：

```c
// pid.c:170
if (abs(pid->Err) > pid->DeadBand) {
    // 死区外 → 正常计算 PID
} else {
    pid->Output = 0;   // 死区内 → 输出清零, 积分清零
    pid->ITerm  = 0;
}
```

死区值的选取：

| 控制对象 | DeadBand | 理由 |
|---------|----------|------|
| 底盘跟随 | 10° | 避免车身微小震动导致底盘不断调整 |
| 云台角度 | ~0.5°~1° | 需要较高精度, 但避免电机微震 |
| 速度环 | ~1~5 RPM | 微小速度波动可接受 |

### 10.4 电机反向标志

不同电机的安装方向可能不同，通过 `motor_reverse_flag` 统一处理：

```c
// chassis_func.c:64, 75, 86, 99
chassis_motor_config.setting_init_config.motor_reverse_flag = 0; // 正向
chassis_motor_config.setting_init_config.motor_reverse_flag = 1; // 反向
```

---

## 11. 故障检测与保护

### 11.1 电机离线检测

每个电机注册时创建离线检测器：

```c
.offline_init_config = {
    .timeout_ms = 100,   // 100ms 内无 CAN 回复 → 判定离线
    .enable     = 1,     // 启用离线管理
    .beep_times = N,     // 离线时蜂鸣次数 (用于听声辨位)
},
```

### 11.2 PID 堵转保护

```c
// pid.c:107-121
// 判断条件: |ref - measure| / |ref| > 95% 且持续 500 周期
if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f) {
    pid->ERRORHandler.ERRORCount++;
}
if (pid->ERRORHandler.ERRORCount > 500) {
    pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
}

// 堵转后: 输出清零, 保护电机
if (pid->ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR) {
    return 0;
}
```

### 11.3 板间通讯丢失保护

```c
// robot_control.c (chassis):29-36
if (Module_BoardComm_Get_Offline_State() == STATE_OFFLINE) {
    // 云台板断连 → 底盘进入安全模式
    chassis_cmd.vx = 0;
    chassis_cmd.vy = 0;
    chassis_cmd.wz = 0;
    chassis_cmd.chassis_mode = chassis_zero_force;
}
```

---

## 12. 开源社区方案拓展

### 12.1 ZJU-HelloWorld 的 PID 扩展

浙大的方案对 PID 做了额外增强：

- **前馈控制 (Feedforward)**：根据目标速度和加速度预测所需的控制量，减轻 PID 负担
- **模型更新 (updateWheelModel)**：实时更新轮子模型参数 (摩擦力、转动惯量)，用于前馈
- **摩擦补偿**：根据速度方向叠加偏置力矩，补偿静摩擦

### 12.2 PID/LQR/MPC 对比研究

| 算法 | 计算复杂度 | 需要模型 | 约束处理 | 嵌入式可行性 | 适用场景 |
|------|-----------|---------|---------|-------------|---------|
| PID | 极低 | 否 | 需额外限幅 | 极易 | 简单系统, 经验调参 |
| LQR | 低 | 是 (线性) | 权重间接约束 | 容易 | 多状态系统, 最优控制 |
| MPC | 高 | 是 | 内置 | 困难 | 有严格约束的系统 |

### 12.3 多传感器融合的前沿方案

部分强队在 LQR 基础上做了以下增强：

- **Kalman Filter + LQR (LQG)**：用 Kalman 滤波器估计全部状态 (消除测量噪声), 再送入 LQR 控制
- **自适应 LQR**：根据负载变化 (如弹丸数量减少) 在线更新系统模型
- **增益调度 (Gain Scheduling)**：在不同工况 (低速/高速/静止) 使用不同的 K 矩阵

---

## 附录 A：关键参数速查表

### PID 配置

| 参数 | 底盘跟随 | 缓冲能量 | 说明 |
|------|---------|---------|------|
| Kp | 0.1 | 5 | 比例增益 |
| Ki | 0 | 0 | 积分增益 |
| Kd | 0.001 | 0 | 微分增益 |
| MaxOut | 5 | 50 | 输出上限 |
| DeadBand | 10 | 5 | 死区 |
| Improve | 0x01 | 0x01 | 积分限幅 |

### LQR 增益 (K 矩阵)

| 控制轴 | K[0] (位置) | K[1] (速度) | state_dim |
|--------|------------|------------|-----------|
| 大 YAW | 22.36 | 3.15 | 2 |
| 小 YAW | 5.47 | 0.56 | 2 |
| PITCH | 38.73 | 2.85 | 2 |
| 驱动轮 | 0.008 | - | 1 |
| 转向轮 | 2.23 | 0.20 | 2 |
| 摩擦轮 | 0.0011 | - | 1 |
| 拨盘 | 0.005 | - | 1 |

### 自动搜索参数

| 参数 | 值 | 说明 |
|------|-----|------|
| SEARCH_YAW_SPEED_RPS | 1.2 rad/s | YAW 搜索旋转速度 |
| SEARCH_PITCH_AMPLITUDE | 0.3 rad | PITCH 正弦幅值 |
| SEARCH_PITCH_FREQ | 0.8 Hz | PITCH 正弦频率 |

---

## 附录 B：Python LQR 设计速查

```python
import numpy as np
from scipy.linalg import solve_continuous_are

# 1. 系统建模
J = 0.2                    # 转动惯量
A = np.array([[0, 1], [0, 0]])
B = np.array([[0], [1/J]])

# 2. 设置权重
Q = np.array([[100, 0], [0, 1]])   # 重视位置
R = np.array([[1]])                # 控制代价

# 3. 解算
S = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ S
print(f"K = {K}")  # → 写入 C 代码

# 4. 验证
A_cl = A - B @ K
eigvals = np.linalg.eigvals(A_cl)
print(f"稳定: {np.all(np.real(eigvals) < 0)}")
```

---

---

## 相关笔记

- [[RM_Sentry_系统架构详解]] — 双板架构、RTOS、板间通信、传感器集成
- [[RM_Sentry_功率控制详解]] — 功率模型、混合权重分配、缓冲能量管理
- [[RM_LQR_lqr.py数学推导]] — LQR Riccati 方程解算的数学细节
- [[RM_LQR_lqr.py使用文档]] — `lqr.py` 工具的使用方法
- [[RM_ThreadX_RTOS教程]] — ThreadX 实时操作系统基础

---

*文档基于 `mas_embedded_threadx/modules/algorithm/` (PID+LQR) 及 `apps/sentry/` 实战代码编写*
*LQR 设计工具: `rm_embedded/sentry/lqr.py`*
*开源参考: ZJU-HelloWorld, Taproot (UW ARUW), GDUT DynamicX*
