# 哨兵功率控制详解 —— 基于 mas_embedded_threadx

## 目录

1. [功率控制概述](#1-功率控制概述)
2. [RoboMaster 功率规则与裁判系统](#2-robomaster-功率规则与裁判系统)
3. [功率模型：从物理到数学](#3-功率模型从物理到数学)
4. [功率控制架构设计](#4-功率控制架构设计)
5. [功率估算：损耗模型详解](#5-功率估算损耗模型详解)
6. [功率分配：混合权重算法](#6-功率分配混合权重算法)
7. [缓冲能量：超级电容管理](#7-缓冲能量超级电容管理)
8. [舵向轮功率隔离](#8-舵向轮功率隔离)
9. [完整调用流程](#9-完整调用流程)
10. [哨兵实战参数解析](#10-哨兵实战参数解析)
11. [开源方案对比与拓展](#11-开源方案对比与拓展)
12. [调参与验证指南](#12-调参与验证指南)

---

## 1. 功率控制概述

### 1.1 什么是功率控制

**功率控制 (Power Control)** 是哨兵机器人在 RoboMaster 比赛中生存的根本保障。裁判系统为每台机器人设定了严格的功率上限（如 120W），一旦超功率运行，将面临扣血甚至直接死亡的惩罚。

功率控制系统的核心任务：

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  遥控/视觉指令    │ →  │  控制器(PID/LQR) │ →  │  功率控制模块    │ → 电机输出
│  (期望速度/位置)  │    │  (期望扭矩)      │    │  (限制后扭矩)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### 1.2 为什么哨兵需要功率控制

哨兵机器人配备 **8 个电机**（4 个 M3508 驱动轮 + 4 个 GM6020 转向轮），极限工况下瞬时功率可达 200W+。

![M3508 驱动电机](https://rm-static.djicdn.com/robomasters/public/img/m3508-01.76b56e4.jpg)
*▲ M3508 直流无刷减速电机（驱动轮/摩擦轮）*

![GM6020 转向电机](https://rm-static.djicdn.com/robomasters/public/img/pc-img-1.a3418b7.png)
*▲ GM6020 直流无刷电机（云台/转向轮）*

没有功率控制：

- **比赛违规**：裁判系统检测到超功率 → 扣血 / 直接死亡
- **硬件损坏**：持续的过载运行会烧毁电机驱动和电源模块
- **电池过放**：电池电压急剧下降，导致系统不稳定

### 1.3 本模块的核心设计理念

本项目的功率控制模块由 `laladuduqq` 设计，具有以下特点：

| 特性 | 说明 |
|------|------|
| **基于物理模型** | 建立了包含机械损耗、铜损、待机功耗的完整功率模型 |
| **角色分类** | 区分驱动轮 (DRIVE) 和转向轮 (STEER)，分别分配功率预算 |
| **混合权重分配** | 结合"按功率比例分配"和"按误差分配"两种策略，平滑过渡 |
| **悬空轮保护** | 识别打滑/悬空轮，防止其抢占接地轮功率 |
| **缓冲能量协同** | 通过裁判系统反馈的缓冲能量，动态调整可用功率上限 |

---

## 2. RoboMaster 功率规则与裁判系统

### 2.1 裁判系统功率检测

裁判系统通过图传模块实时监测每台机器人的功率消耗：

![裁判系统电源管理模块 PM01](https://rm-static.djicdn.com/tem/17348/5a0305fa041ad1592469731622716447.jpg)
*▲ 裁判系统电源管理模块 (PM01) — 实时测量底盘功率消耗*

```
裁判系统数据链：
  主控模块 → 电源管理模块(测功率) → 图传 → 裁判系统服务器 → 客户端显示

机器人端可获取：
  CMD_ID_POWER_HEAT_DATA  → shooter_17mm_heat (枪管热量)
  CMD_ID_GAME_STATUS      → game_progress (比赛阶段)
  CMD_ID_ROBOT_STATUS     → current_hp (当前血量)
```

### 2.2 功率上限规则

| 机器人类型 | 底盘功率上限 | 备注 |
|-----------|-------------|------|
| 步兵 (Infantry) | 80W | 17mm 弹丸 |
| 英雄 (Hero) | 160W | 42mm 弹丸 |
| 哨兵 (Sentry) | 120W | 全自动 |
| 工程 (Engineer) | 120W | 无发射机构 |
| 无人机 (Drone) | 100W | 空中单位 |

### 2.3 超功率惩罚机制

```
超功率程度          惩罚
──────────────────────────────────
< 5% 且 < 1秒      无惩罚（容忍窗口）
5% ~ 20%           缓冲能量快速消耗
> 20% 或持续       缓冲能量耗尽 → 扣血
缓冲能量 = 0        立即死亡
```

---

## 3. 功率模型：从物理到数学

### 3.1 电机功率的物理本质

一个直流无刷电机的总功率由以下分量构成：

```
P_total = P_mech + P_copper + P_friction + P_static

其中：
  P_mech    = τ × ω       机械输出功率（有用功）
  P_copper  = R × I²      铜损（绕线电阻发热）
  P_friction = b × |ω|    机械摩擦损耗
  P_static  = constant    待机功耗（编码器、MCU 等）
```

### 3.2 本项目采用的损耗模型

代码中定义的功率估算公式（`power_control.h:24`）：

```
P = τ × Ω + k1 × |Ω| + k2 × τ² + k3
```

**各参数物理含义**：

| 参数 | 物理含义 | 单位 | 典型值范围 |
|------|---------|------|-----------|
| `τ × Ω` | 机械输出功率 | W | 驱动轮 0~80W |
| `k1` | 速度相关损耗系数（摩擦+风阻） | W·s/rad | 0.005 ~ 0.15 |
| `k2` | 扭矩相关损耗系数（铜损） | W/(N²·m²) | 1 ~ 15 |
| `k3` | 静态功耗偏置 | W | 0.5 ~ 3 |

### 3.3 为什么使用 τ 而不是电流 I

```
电流 I 与扭矩 τ 的关系：
  τ = Kt × I × gear_ratio

其中 Kt = 扭矩常数 (Nm/A)

使用 τ 的好处：
  1. 与电机类型解耦（M3508/M2006/GM6020 统一公式）
  2. 直接反映机械输出
  3. 方便做扭矩限制
```

### 3.4 功率模型可视化

```
功率组成示意（以 M3508 @ 中等负载为例）：

P_total = 45W
├── P_mech    = 30W  ████████████████████████████████  (~67%)
├── P_copper  =  8W  ██████████                        (~18%)
├── P_friction=  5W  ██████                            (~11%)
└── P_static  =  2W  ██                                (~4%)
```

---

## 4. 功率控制架构设计

### 4.1 模块总览

```
power_control.h / power_control.c

对外接口:
  PowerControl_Register()    ← 注册电机
  PowerControl_SetLimit()    ← 设置功率上限
  PowerControl_Update()      ← 每周期执行功率分配

内部函数:
  dji_output_to_torque()     ← DJI 电机电流 → 扭矩
  dji_set_torque()           ← 扭矩 → DJI 电机电流
  limit_group()              ← 核心：对一组电机执行功率分配
```

### 4.2 数据结构

```c
/* 功率控制角色 */
typedef enum {
    PC_ROLE_NONE  = 0,   // 不参与功率控制
    PC_ROLE_DRIVE = 1,   // 驱动轮（消耗功率）
    PC_ROLE_STEER = 2,   // 舵向轮（消耗功率）
} PowerCtrl_Role_e;

/* 损耗模型参数 */
typedef struct {
    float k1;   // 速度相关损耗系数
    float k2;   // 扭矩相关损耗系数
    float k3;   // 静态功耗偏置
} PowerCtrl_Param_t;

/* 内部节点 */
typedef struct {
    Motor_Base       *motor;          // 电机基类指针
    PowerCtrl_Role_e  role;           // 角色
    PowerCtrl_Param_t param;          // 损耗参数
    float             raw_power;      // 原始估算功率
    float             assigned_power; // 分配后功率
} PC_Node_t;
```

### 4.3 架构图

```
┌─────────────────────────────────────────────────────┐
│                   PowerControl_Update()              │
│                                                     │
│  ① 计算可用功率 (power_limit + buffer_adjustment)    │
│                         │                           │
│                         ▼                           │
│  ② 按角色分组: DRIVE 组 / STEER 组                   │
│                         │                           │
│              ┌──────────┴──────────┐                │
│              ▼                     ▼                │
│  ③ STEER 组分配             ④ DRIVE 组分配           │
│     (总功率 × STEER_RATIO)     (剩余功率)            │
│              │                     │                │
│              └──────────┬──────────┘                │
│                         ▼                           │
│  ⑤ 写入各电机的 controller.output                    │
│                                                     │
└─────────────────────────────────────────────────────┘
```

---

## 5. 功率估算：损耗模型详解

### 5.1 估算公式的代码实现

```c
// power_control.c:114-115
/* P = τ·Ω + k1·|Ω| + k2·τ² + k3 */
cmdPow[i] = tau[i] * omega[i]
          + k1 * ABS(omega[i])
          + k2 * tau[i] * tau[i]
          + k3;
```

### 5.2 DJI 电机电流 ↔ 扭矩转换

由于 DJI 电调使用整型电流值通信，需要建立映射关系：

```c
// power_control.c:36-51
static float dji_output_to_torque(const Motor_Base *m)
{
    float Kt_gear = m->info.torque_constant * m->info.gear_ratio;

    switch (m->info.motor_type) {
    case M3508:
        // 电流范围: -20A ~ +20A → 整数值: -16384 ~ +16384
        return IntegerToCurrent(-20.0f, 20.0f, -16384, 16384,
                                (int16_t)m->controller.output);
    case M2006:
        return IntegerToCurrent(-10.0f, 10.0f, -10000, 10000,
                                (int16_t)m->controller.output);
    case GM6020_CURRENT:
        return IntegerToCurrent(-3.0f,  3.0f,  -16384, 16384,
                                (int16_t)m->controller.output);
    }
}
```

### 5.3 扭矩到电流的反向映射

功率限制后需要将新的扭矩值写回电机输出：

```c
// power_control.c:54-80
static void dji_set_torque(Motor_Base *m, float torque_nm)
{
    float Kt_gear = m->info.torque_constant * m->info.gear_ratio;
    float current_A = torque_nm / Kt_gear;

    switch (m->info.motor_type) {
    case M3508:
        m->controller.output = currentToInteger(-20.0f, 20.0f,
                                                 -16384, 16384, current_A);
        break;
    // ... 其他电机类型
    }
}
```

### 5.4 k1, k2, k3 的物理意义与实测

**调参方法（来自 ZJU-HelloWorld 的开源经验）**：

| 步骤 | 操作 | 目的 |
|------|------|------|
| 1 | 设定 `power_limit = 60W` 恒功率 | 建立基准 |
| 2 | 电机静止，读取功率 | 得到 `k3`（静态功耗） |
| 3 | 堵转电机，调整 `k2` | 使估算功率 = 60W（仅铜损） |
| 4 | 低速行车，调整 `k1` | 补偿低速摩擦损耗 |
| 5 | 高速行车，微调 `k1` | 补偿高速风阻损耗 |
| 6 | 综合测试 | 目标：实际功率曲线接近水平（±5W） |

---

## 6. 功率分配：混合权重算法

### 6.1 分配策略总览

```
功率超限时的分配流程:

Step 1: 收集所有正功率电机（消耗功率者）
        → sum_pos = Σ(正功率), sum_err = Σ(速度误差)

Step 2: 计算可用分配功率
        → allocatable = sum_neg + max_power
          (再生功率回收 + 功率上限)

Step 3: 计算"误差置信度" confidence
        → 误差大 → 按误差比例分配（急加速/减速阶段）
        → 误差小 → 按功率比例分配（稳态巡航阶段）

Step 4: 对每个正功率电机
        → 混合权重 w = confidence × w_err + (1-confidence) × w_prop
        → 分配功率 p_alloc = w × allocatable

Step 5: 反解扭矩（解二次方程）
        → k2·τ² + Ω·τ + (k1|Ω| + k3 - p_alloc) = 0
```

### 6.2 误差置信度计算

```c
// power_control.c:138-145
float confidence;
if (sum_err >= ERROR_THRESHOLD)        // sum_err ≥ 20 rad/s
    confidence = 1.0f;                 // 完全按误差分配
else if (sum_err > PROP_THRESHOLD)     // 15 < sum_err < 20
    confidence = (sum_err - PROP_THRESHOLD)
               / (ERROR_THRESHOLD - PROP_THRESHOLD);
else                                   // sum_err ≤ 15
    confidence = 0.0f;                 // 完全按功率比例分配
```

**设计原理**：

```
confidence = 0 (按功率比例)    confidence = 1 (按误差比例)
        │                              │
稳态巡航: 所有电机速度已接近目标    急加速: 某些电机远未达到目标速度
→ 保持当前功率分配比例最合理         → 优先给误差大的电机分配功率
→ 避免不必要的重新分配抖动           → 确保机动性响应
```

### 6.3 悬空轮保护

当机器人在崎岖地形行驶时，某个轮子可能悬空或打滑。此时该轮的速度误差为零（已接近目标转速），但实际不消耗功率——不应抢占接地轮的功率。

```c
// power_control.c:161-163
/* 悬空轮保护: 速度已接近目标的电机不抢占接地轮功率 */
if (errRad[i] < IDLE_WHEEL_ERR_RAD && p_alloc > cmdPow[i])
    p_alloc = cmdPow[i];  // 限制为原始需求功率
```

参数 `IDLE_WHEEL_ERR_RAD = 10.47 rad/s`（≈ 100 RPM）：当速度误差小于此阈值时，认定为悬空/打滑。

### 6.4 扭矩反解：二次方程求解

**问题**：已知分配功率 `p_alloc`，求对应的扭矩 `τ` 使得 `P(τ) = p_alloc`。

```
P(τ) = k2·τ² + Ω·τ + (k1·|Ω| + k3) = p_alloc

→ k2·τ² + Ω·τ + (k1·|Ω| + k3 - p_alloc) = 0

a = k2,  b = Ω,  c = k1·|Ω| + k3 - p_alloc
```

```c
if (a_coef < 1e-8f) {
    // k2 ≈ 0: 线性方程 → τ = (p - k1·|Ω| - k3) / Ω
    tau_new = (p_alloc - k1 * ABS(Omega) - k3) / Omega;
} else {
    // 二次方程 → 选择与当前扭矩同号的根
    float delta = b*b - 4*a*c;
    float tau1 = (-b + sqrt(delta)) / (2*a);
    float tau2 = (-b - sqrt(delta)) / (2*a);
    tau_new = (current_output >= 0) ? tau1 : tau2;
}

// 限幅到电机最大扭矩
tau_new = CLAMP(tau_new, -max_torque, max_torque);
```

---

## 7. 缓冲能量：超级电容管理

![超级电容阵列实物](https://raw.githubusercontent.com/hkustenterprize/RM2023-SuperCapacitor/master/image/img_cap_array_top.jpg)
*▲ 开源超电方案 — 超级电容阵列 PCB (HKUST ENTERPRIZE)*

![超电控制器主板](https://raw.githubusercontent.com/hkustenterprize/RM2023-SuperCapacitor/master/image/img_controller.jpg)
*▲ 超电控制器主板实物*

### 7.1 缓冲能量机制

裁判系统为每台机器人提供一个 **缓冲能量** (Buffer Energy) 参数，初始值为 60J。当机器人超功率时，裁判系统不会立即惩罚，而是先消耗缓冲能量。

```
功率消耗模式:
──────────────────────────────────────────────────
普通模式 (功率 ≤ 上限):
  → 缓冲能量缓慢恢复至 60J

超功率模式 (功率 > 上限):
  → 缓冲能量快速消耗
  → 消耗速率 = (实际功率 - 上限功率) × 时间

缓冲能量耗尽 (= 0):
  → 立即开始扣血
  → 直至死亡
```

### 7.2 PID 管理缓冲能量

本项目通过 PID 控制缓冲能量的消耗速度：

```c
// power_control.c:248-256
if (s_use_buffer) {
    if (s_buffer_pid.Kp == 0.0f) {
        // 惰性初始化: Kp=5, DeadBand=5, MaxOut=50
        PID_Init_Config_s cfg = {
            .MaxOut = 50, .DeadBand = 5, .Kp = 5, .Improve = 0x01
        };
        PIDInit(&s_buffer_pid, &cfg);
    }
    // 目标: 保持缓冲能量在 30J 以上
    PIDCalculate(&s_buffer_pid, s_buffer_energy, 30.0f);
    effective -= s_buffer_pid.Output;
}
```

**缓冲能量充足时** (`s_buffer_energy >> 30`): PID 输出可提高有效功率上限，允许短时爆发
**缓冲能量不足时** (`s_buffer_energy < 30`): PID 输出降低有效功率上限，保护缓冲能量

### 7.3 哨兵实战参数

```c
// chassis_func.c:186
PowerControl_SetLimit(120,   // 功率上限 120W
                       60,   // 缓冲能量 60J (初始值)
                       0);   // use_buffer = 0 (不使用缓冲能量)
```

哨兵选择 **不使用缓冲能量**（`use_buffer = 0`），始终保持功率在 120W 以下。这是因为哨兵作为全自动单位，更需要稳定可预测的行为。

---

## 8. 舵向轮功率隔离

### 8.1 为什么要隔离

舵向轮 (GM6020 × 4) 与驱动轮 (M3508 × 4) 的工作特性完全不同：

| 特性 | 驱动轮 (M3508) | 转向轮 (GM6020) |
|------|---------------|----------------|
| 功率消耗 | 持续高负载 | 间歇低负载 |
| 控制类型 | 速度环 (SPEED_LOOP) | 角度环 (ANGLE_LOOP) |
| 典型功率 | 20~80W | 2~10W |
| 对机动性影响 | 直接决定底盘速度 | 影响转向精度 |

### 8.2 功率分配策略

```c
// power_control.c:260-274
if (n_steer == 0) {
    // 无转向轮 → 全部功率给驱动
    limit_group(drives, n_drive, effective, NULL);
} else {
    // 先分配转向轮 (最多占 80%)
    float steer_sum = 0.0f;
    limit_group(steers, n_steer, effective * STEER_RATIO, &steer_sum);

    // 实际使用的转向功率 (可能少于分配值)
    float steer_actual = (steer_sum < effective * STEER_RATIO)
                       ? steer_sum : effective * STEER_RATIO;

    // 剩余全部分配给驱动轮
    float drive_limit = effective - steer_actual;
    limit_group(drives, n_drive, drive_limit, NULL);
}
```

`STEER_RATIO = 0.8f`：转向轮最多占 80% 功率预算。这是基于哨兵经验的安全值——转向轮实际消耗通常远小于此值，剩余功率自动回归驱动轮。

---

## 9. 完整调用流程

### 9.1 初始化阶段

```c
// ========== chassis_func.c:26-188 ==========

void chassis_init(void) {
    // 步骤 1: 定义各电机的损耗模型参数
    PowerCtrl_Param_t power_config = {
        .k1 = 0.132,     // 驱动轮 k1
        .k2 = 3.47,      // 驱动轮 k2
        .k3 = 1          // 驱动轮 k3
    };

    PowerCtrl_Param_t gm6020_power_config = {
        .k1 = 0.005,     // 转向轮 k1 (远小于驱动轮)
        .k2 = 12.98,     // 转向轮 k2 (铜损较大)
        .k3 = 1          // 转向轮 k3
    };

    // 步骤 2: 初始化电机 + 注册到功率控制
    chassis_motors[0] = Motor_DJI_Init(&chassis_motor_config);
    PowerControl_Register(&chassis_motors[0]->base,
                          PC_ROLE_DRIVE, power_config);
    // ... 重复 8 次

    // 步骤 3: 设置功率上限
    PowerControl_SetLimit(120, 60, 0);
}
```

### 9.2 运行阶段

```c
// ========== 每 2ms 执行一次的功率控制循环 ==========

// 1. 各控制器计算期望扭矩 (LQR/PID)
//    → motor->controller.output = 期望电流值

// 2. 执行功率分配
PowerControl_Update();
//    → 内部读取 output → 估算功率 → 再分配 → 写回 output

// 3. 刷新 CAN 总线 (将 output 发送给电调)
Motor_DJI_Flush(chassis_motors, 8);
```

### 9.3 时序图

```
时间 →
│
├─ 0ms:  encoder update (读取编码器反馈)
├─ 1ms:  LQR/PID calculate (计算期望扭矩)
├─ 1ms:  PowerControl_Update (功率分配)
├─ 1ms:  Motor_DJI_Flush (CAN 发送)
├─ 2ms:  tx_thread_sleep(2) (任务挂起)
│
└─ 2ms:  下一周期开始
```

---

## 10. 哨兵实战参数解析

### 10.1 驱动轮 (M3508 × 4)

```c
Motor_Init_Config_s chassis_motor_config = {
    .controller_init_config = {.lqr_init = {
        .K = {0.008f},          // LQR 速度环增益
        .state_dim = 1,         // 一阶 LQR (仅速度)
    }},
    .setting_init_config = {
        .loop_type = SPEED_LOOP,         // 速度环
        .algorithm_type = CONTROL_LQR,   // LQR 算法
    },
    .motor_init_info = {
        .motor_type = M3508,
        .gear_ratio = 16,               // 减速比 16:1
        .max_torque = 5.32,             // 最大扭矩 5.32 Nm
        .torque_constant = 0.016,       // 扭矩常数 0.016 Nm/A
    },
};

PowerCtrl_Param_t power_config = {
    .k1 = 0.132,    // 速度损耗: 驱动轮转速高, k1 较大
    .k2 = 3.47,     // 铜损系数: M3508 绕线电阻适中
    .k3 = 1         // 静态功耗 ~1W
};
```

### 10.2 转向轮 (GM6020 × 4)

```c
Motor_Init_Config_s gm6020_motor_config = {
    .controller_init_config = {.lqr_init = {
        .K = {2.23f, 0.2f},     // LQR 角度+速度双状态反馈
        .state_dim = 2,          // 二阶 LQR
    }},
    .setting_init_config = {
        .loop_type = ANGLE_LOOP,         // 角度环
        .algorithm_type = CONTROL_LQR,   // LQR 算法
    },
    .motor_init_info = {
        .motor_type = GM6020_CURRENT,
        .gear_ratio = 1,                // 直驱
        .max_torque = 2.23,             // 最大扭矩 2.23 Nm
        .torque_constant = 0.741,       // 扭矩常数 0.741 Nm/A
    },
};

PowerCtrl_Param_t gm6020_power_config = {
    .k1 = 0.005,    // 速度损耗: 转向轮转速低, k1 很小
    .k2 = 12.98,    // 铜损系数: GM6020 电流模式铜损较大
    .k3 = 1         // 静态功耗 ~1W
};
```

### 10.3 参数对比分析

```
               k1          k2        k3
              (摩擦)     (铜损)   (静态)
M3508 驱动轮   0.132      3.47      1.0
GM6020 转向轮  0.005      12.98     1.0

k1 差异: 驱动轮 >> 转向轮 → 驱动轮高速旋转, 摩擦/风阻大
k2 差异: 转向轮 >> 驱动轮 → GM6020 电流模式铜损远大于 M3508
k3 相同: 静态功耗相当 (~1W)
```

---

## 11. 开源方案对比与拓展

### 11.1 本方案 vs ZJU-HelloWorld 方案

| 维度 | 本方案 (mas_embedded_threadx) | ZJU-HelloWorld |
|------|------------------------------|----------------|
| **功率模型** | P = τΩ + k1\|Ω\| + k2τ² + k3 | P = k1·Iω + k2·I² + k3·\|ω\| + p_bias |
| **变量** | 扭矩 τ | 电流 I |
| **分配策略** | 混合权重 (误差+比例) | 等比例缩放 (Limit_K) |
| **角色分类** | DRIVE / STEER | 无分类 |
| **缓冲能量** | PID 管理 (可选) | 超电板单独管理 |
| **悬空保护** | ✓ 有 | ✗ 无 |
| **复杂度** | 中等 | 较低 |

### 11.2 广东工业大学 DynamicX — 超电管理

GDUT 的方案在传统功率限制之上增加了超级电容硬件管理：

```
状态机:
  Normal    → 恒功率充电, 剩余功率驱动底盘
  OverPower → 全功率充电至 26V, 超电放电提供 ~200W 爆发
  NoCharge  → 全部功率供底盘
  Halt      → 完全断开 (保护模式)
```

**优化策略**：
- 电容不满时预留 15% 功率充电
- 电容电量 >10% 时进入超功率模式（200W 输出）
- 不充电时全功率供底盘

这是一种**硬件+软件协同**的高级功率管理方案，适合对短时爆发力有极高需求的机器人（如工程、英雄）。

### 11.3 Taproot (UW ARUW) — 闭环功率限制

华盛顿大学的 Taproot 框架采用了以下独特设计：

- **闭环反馈**：不仅预测功率，还通过裁判系统回读实际功率做闭环修正
- **Round-Robin 调度**：类似 FRC 的 WPILib 架构
- **C++20 模板**：编译期类型安全，零开销抽象

### 11.4 各方案适用场景

| 方案          | 适用场景    | 优势           | 劣势       |
| ----------- | ------- | ------------ | -------- |
| **本方案**     | 舵轮底盘哨兵  | 悬空保护, 角色分类精细 | 仅 DJI 电机 |
| **ZJU-HW**  | 麦克纳姆轮底盘 | 简洁高效, 社区文档丰富 | 无角色分类    |
| **GDUT**    | 有超电的机器人 | 爆发力强, 硬件协同   | 需要额外硬件   |
| **Taproot** | 跨平台研究   | 闭环修正, 架构优雅   | 学习曲线陡    |

---

## 12. 调参与验证指南

### 12.1 k1/k2/k3 标准调参流程

```
Step 1: 准备工作
  □ 确认裁判系统正常运行
  □ 设置 power_limit = 60W
  □ 设置 use_buffer = 0 (关闭缓冲能量)
  □ 记录裁判系统上报的实时功率

Step 2: 测 k3 (静态功耗)
  □ 所有电机停止 (chassis_zero_force 模式)
  □ 记录裁判系统功率读数 → k3 = 静态功率 / 电机数量

Step 3: 测 k2 (铜损系数)
  □ 机械堵转所有驱动轮 (确保不会转动)
  □ 施加小扭矩指令 (10%~30% 最大扭矩)
  □ 调整 k2 使估算功率等于裁判系统读数
  □ 公式: P = k2·τ² + k3 → k2 = (P - k3) / τ²

Step 4: 测 k1 (摩擦损耗)
  □ 悬空轮子，设定中等速度 (如 5 rad/s)
  □ 调整 k1 使估算功率等于实测功率
  □ 公式: P = k1·|Ω| + k3 → k1 = (P - k3) / |Ω|

Step 5: 综合验证
  □ 正常行驶
  □ 目标: 估算功率与实际功率偏差 < ±5W
  □ 如偏差较大，微调 k1 (高速段) 或 k2 (高扭矩段)
```

### 12.2 常见问题排查

| 现象         | 可能原因                 | 解决方案                    |
| ---------- | -------------------- | ----------------------- |
| 估算功率总是偏低   | k1/k2 偏小             | 增加 k1 或 k2              |
| 估算功率总是偏高   | k1/k2 偏大             | 减小 k1 或 k2              |
| 某个轮子被过度限功率 | 悬空保护误触发              | 调整 `IDLE_WHEEL_ERR_RAD` |
| 转向不灵敏      | STEER_RATIO 过小       | 提高 `STEER_RATIO`        |
| 直线行驶功率波动大  | k1 需要微调              | 在高速段进行单独调参              |
| 缓冲能量耗尽快    | use_buffer=1 且 Kp 过大 | 减小缓冲 PID 的 Kp           |

### 12.3 验证清单

```
□ 裁判系统功率读数连续 30s < power_limit
□ 急加速不超功率 (缓冲能量不被消耗)
□ 急转弯不超功率
□ 单轮悬空场景功率正常分配
□ 长时间运行 (5min+) 功率稳定
□ 发射机构工作时底盘不受影响 (哨兵/步兵)
□ 功率曲线接近水平线 (波动 < ±5W)
```

---

## 附录 A：关键参数速查表

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `POWER_CTRL_MAX_MOTORS` | 8 | 最大注册电机数 |
| `STEER_RATIO` | 0.8 | 舵向轮最大功率占比 |
| `ERROR_THRESHOLD` | 20.0 rad/s | 完全按误差分配阈值 |
| `PROP_THRESHOLD` | 15.0 rad/s | 完全按比例分配阈值 |
| `IDLE_WHEEL_ERR_RAD` | 10.47 rad/s | 悬空轮判定阈值 |
| `s_power_limit` | 120.0 W | 哨兵功率上限 |
| `s_buffer_energy` | 60.0 J | 初始缓冲能量 |

## 附录 B：电机参数速查表

| 电机型号 | 额定扭矩 | 扭矩常数 Kt | 用途 |
|---------|---------|------------|------|
| M3508 | 5.32 Nm | 0.016 Nm/A | 驱动轮/摩擦轮 |
| GM6020 | 2.23 Nm | 0.741 Nm/A | 云台/转向轮 |
| M2006 | 1.80 Nm | 0.005 Nm/A | 拨盘/装载 |
| DM4310 | 10.0 Nm | 0.093 Nm/A | Pitch 轴 |

---

---

## 相关笔记

- [[RM_Sentry_系统架构详解]] — 双板架构、RTOS、板间通信、传感器集成
- [[RM_Sentry_控制算法详解]] — PID 七大优化、LQR 一维/二维控制器、调参方法
- [[RM_ThreadX_RTOS教程]] — ThreadX 实时操作系统基础

---

*文档基于 `mas_embedded_threadx/modules/MOTOR/POWER_CONTROL/` 源码编写*
*开源方案参考: ZJU-HelloWorld, GDUT DynamicX, UW ARUW/Taproot*
