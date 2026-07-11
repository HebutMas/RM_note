# 串联轮腿步兵下位机控制系统设计文档

> 基于 `mas_embedded_threadx` 架构，面向 RoboMaster 2025-2026 赛季串联轮腿步兵机器人。

---

## 目录

1. [系统概述](#1-系统概述)
2. [硬件架构](#2-硬件架构)
3. [软件架构：基于 mas_embedded_threadx 的适配设计](#3-软件架构基于-mas_embedded_threadx-的适配设计)
4. [核心算法一：五连杆运动学](#4-核心算法一五连杆运动学)
5. [核心算法二：雅可比矩阵与 VMC](#5-核心算法二雅可比矩阵与-vmc)
6. [核心算法三：LQR 平衡控制](#6-核心算法三lqr-平衡控制)
7. [模块设计](#7-模块设计)
8. [CAN 通信架构](#8-can-通信架构)
9. [状态机与安全保护](#9-状态机与安全保护)
10. [调试与参数整定](#10-调试与参数整定)

---

## 1. 系统概述

### 1.1 什么是串联轮腿步兵

串联轮腿步兵（Serial Wheel-Legged Infantry）是 RoboMaster 2025-2026 赛季的技术热点。相比传统四麦克纳姆轮底盘，轮腿构型增加了：

| 特性 | 传统麦轮步兵 | 串联轮腿步兵 |
|------|------------|------------|
| 越障能力 | 仅平坦地面 | **可爬台阶、过隧道** |
| 姿态控制 | 无（底盘固定） | **主动调节高度和倾角** |
| 抗冲击 | 刚性连接 | **腿机构缓冲减震** |
| 跌倒恢复 | 无法 | **可自主复位** |
| 控制复杂度 | 低 | **高（LQR+VMC+运动学）** |

### 1.2 系统拓扑

```
┌─────────────────────────────────────────────────────────┐
│                    上位机 (Jetson Orin / NUC)             │
│              视觉检测 · 目标跟踪 · 弹道解算 · 导航         │
└──────────┬──────────────────────────────────┬───────────┘
           │ USB CDC (虚拟串口)               │ 串口 (DBUS遥控器)
           ▼                                  ▼
┌─────────────────────────────────────────────────────────┐
│                  下位机 (双板架构)                        │
│                                                         │
│  ┌──────────────────┐      CAN       ┌───────────────┐ │
│  │   云台板 (Gimbal)  │ ←──────────→ │  底盘板 (Chassis)│ │
│  │   STM32H723/F407 │  板间通信      │  STM32H723    │ │
│  │                   │              │               │ │
│  │ · 遥控器解析       │              │ · LQR 平衡控制  │ │
│  │ · 云台控制 (LQR)   │              │ · VMC 力分配   │ │
│  │ · 射击控制         │              │ · 五连杆运动学  │ │
│  │ · 视觉通信         │              │ · 腿长+姿态控制 │ │
│  │ · INS 姿态解算     │              │ · 轮毂电机控制  │ │
│  └──────────────────┘              └───────────────┘ │
│                                        │               │
└────────────────────────────────────────┼───────────────┘
                                         │ CAN 总线
                    ┌────────────────────┼────────────────────┐
                    ▼                    ▼                    ▼
             ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
             │ 关节电机 ×4  │    │ 轮毂电机 ×2  │    │  云台电机 ×2 │
             │ DM-J8009     │    │ MF9025/3508 │    │ GM6020/4310 │
             │ (MIT 模式)    │    │ (转矩闭环)   │    │ (电流环)    │
             └─────────────┘    └─────────────┘    └─────────────┘
```

### 1.3 设计原则

1. **复用 mas_embedded_threadx 架构**：保持 5 层分层、条件编译、模块化设计
2. **双板分离**：云台板和底盘板独立运行，通过 CAN 板间通信
3. **实时性优先**：LQR 控制和 VMC 计算运行在最高优先级任务中
4. **安全兜底**：离地检测、关节限位、过流保护、跌倒检测全覆盖

---

## 2. 硬件架构

### 2.1 主控选型

| 板卡 | 芯片 | 主频 | 用途 |
|------|------|------|------|
| 底盘板 | STM32H723VG (DM-02) | 480 MHz | 平衡控制 + 运动学 + VMC |
| 云台板 | STM32F407IG (C 板) | 168 MHz | INS + 遥控 + 云台 + 射击 + 视觉通信 |

**为什么底盘板需要 H723？**
- 五连杆运动学 + 雅可比矩阵 + LQR 6 状态 → 大量浮点运算
- 1 kHz 控制频率 → 每次迭代仅 1ms 预算
- Cortex-M7 双精度 FPU + 480MHz 是必要配置

### 2.2 执行器选型

| 组件 | 型号 | 数量 | 通信 | 控制模式 | 关键参数 |
|------|------|------|------|---------|---------|
| 关节电机 | DM-J8009P-2EC | 4（左右各2） | CAN | **MIT 模式** | 峰值扭矩 9 Nm |
| 轮毂电机 | 瓴控 MF9025V2 | 2 | CAN | **转矩闭环** | 额定扭矩 2.5 Nm |
| 云台 Yaw | GM6020 | 1 | CAN | 电流环 | 0.741 Nm/A |
| 云台 Pitch | DM4310 | 1 | CAN | MIT/电流 | 10:1 减速 |
| 拨弹电机 | M2006 | 1 | CAN | 速度环 | 36:1 减速 |
| 摩擦轮电机 | M3508 ×2 | 2 | CAN | 速度环 | — |

### 2.3 传感器选型

| 传感器 | 型号 | 接口 | 用途 |
|--------|------|------|------|
| IMU | BMI088 | SPI | 姿态解算（底盘+云台各一个） |
| 关节编码器 | DM-J8009 内置 | CAN 反馈 | 关节角度（用于运动学） |
| 轮毂编码器 | MF9025 内置 | CAN 反馈 | 轮速（用于里程计） |
| 云台编码器 | GM6020/4310 内置 | CAN 反馈 | 云台角度 |

### 2.4 机械结构

```
                    机身 (Body)
                   ┌──────────┐
                   │  IMU     │
                   │  电池     │
                   │  云台     │
                   └────┬─────┘
               ┌────────┴────────┐
         左腿  │                  │  右腿
      ┌───────┴───────┐  ┌───────┴───────┐
      │  关节电机1     │  │  关节电机3     │
      │  (DM-J8009)   │  │  (DM-J8009)   │
      │    ↓ L1       │  │    ↓ L1       │
      │   连杆1       │  │   连杆1       │
      │    ↓ L2       │  │    ↓ L2       │
      │  关节电机2     │  │  关节电机4     │
      │  (DM-J8009)   │  │  (DM-J8009)   │
      │    ↓          │  │    ↓          │
      │  轮毂电机     │  │  轮毂电机     │
      │  (MF9025)     │  │  (MF9025)     │
      └───────────────┘  └───────────────┘

      L1 = 0.15m (上连杆), L2 = 0.15m (下连杆)
      虚拟腿长 L₀: 0.18~0.30m (可调)
```

### 2.5 CAN 总线拓扑

```
底盘板 CAN1 (1 Mbps)
├── 0x201: 左腿关节电机1 (DM-J8009)
├── 0x202: 左腿关节电机2 (DM-J8009)
├── 0x203: 右腿关节电机3 (DM-J8009)
├── 0x204: 右腿关节电机4 (DM-J8009)
├── 0x205: 左轮毂电机 (MF9025)
└── 0x206: 右轮毂电机 (MF9025)

底盘板 CAN2 (1 Mbps)
├── 云台板板间通信
└── 超级电容模块

云台板 CAN1 (1 Mbps)
├── 0x207: 云台 Yaw (GM6020)
├── 0x208: 云台 Pitch (DM4310)
├── 0x209: 拨弹电机 (M2006)
└── 0x20A: 摩擦轮电机1+2 (M3508)
```

---

## 3. 软件架构：基于 mas_embedded_threadx 的适配设计

### 3.1 分层架构复用

完全复用 mas_embedded_threadx 的 5 层架构，新增轮腿相关模块：

```
APP 层 (apps/infantry_leg/)
├── gimbal_board/robot_control.c     ← 云台板主循环（保留）
├── chassis_board/robot_control.c    ← 底盘板主循环（重写）
├── chassis_board/balance_control.c  ← 🆕 LQR 平衡控制
├── chassis_board/leg_control.c      ← 🆕 腿长+姿态控制
└── chassis_board/chassis_func.c     ← 运动学解算（重写）

ROBOT 层 (robot/)
└── robot_init.c                     ← 不变

MODULE 层 (modules/)
├── BMI088/ INS/ REMOTE/ VISION/ BOARDCOMM/  ← 保留
├── MOTOR/                           ← 🆕 添加 DM 电机 MIT 模式
├── LEG/                             ← 🆕 五连杆运动学 + VMC
│   ├── module_leg.h
│   ├── module_leg.c                 ← 运动学 + 雅可比
│   └── module_vmc.c                 ← VMC 力分配
├── BALANCE/                         ← 🆕 LQR 平衡控制
│   └── module_balance.c
└── algorithm/
    ├── pid.c/h                      ← 保留
    ├── lqr.c/h                      ← 🆕 扩展：支持 6 状态 + 增益调度
    └── kinematics.c/h               ← 🆕 五连杆正/逆运动学

BSP 层 (board/bsp/)
└── 保留全部现有驱动

RTOS + HAL
└── ThreadX + STM32 HAL
```

### 3.2 CMake 配置

```cmake
# apps/config.cmake
set(ROBOT "infantry_leg")  # 🆕 串联轮腿步兵
set(BOARD "chassis")       # 当前编译底盘板（云台板用 gimbal）

# apps/infantry_leg/robot.cmake
set(MODULES_GIMBAL   OFFLINE REMOTE BMI088 INS MOTOR VISION BOARDCOMM)
set(MODULES_CHASSIS  OFFLINE BMI088 INS MOTOR LEG BALANCE BOARDCOMM)
```

### 3.3 底盘板任务设计

```
优先级 3:  CAN RX Task           ← CAN 接收（最高优先级，不丢帧）
优先级 4:  CAN TX Task           ← CAN 发送
优先级 6:  Offline Detect        ← 100 Hz 看门狗 + 离线检测
优先级 7:  INS Task              ← 1000 Hz IMU 姿态解算
优先级 8:  Balance Control Task  ← 🆕 1000 Hz LQR + VMC
优先级 10: Leg Control Task      ← 🆕 500 Hz 腿长控制
优先级 12: Motor Task            ← 500 Hz 电机数据刷新
优先级 30: Robot Control Task    ← 500 Hz 主控制循环
```

**关键设计决策**：Balance Control Task 以 1000 Hz 运行在优先级 8，高于所有其他控制任务。这是因为：
- 轮腿系统是**本质不稳定系统**（倒立摆）
- 控制延迟每增加 1ms，稳定裕度显著下降
- LQR+VMC 计算量约 100-200 µs @ 480MHz，1ms 预算充足

### 3.4 任务数据流

```
INS Task (1000 Hz)
  │ BMI088 gyro + acc
  │ QuaternionEKF → q[4], euler[3]
  │ 写入全局 ins_data
  ▼
Balance Control Task (1000 Hz)
  │ 读取: ins_data, wheel_speed, joint_angles, remote_cmd
  │ 1. 正运动学 → L₀, φ₀, θ, θ̇
  │ 2. LQR 计算 → F₀ (虚拟推力), Tp (髋力矩)
  │ 3. VMC 映射 → τ_joint1, τ_joint2 (每腿)
  │ 4. CAN 发送 → 关节电机 + 轮毂电机
  ▼
CAN TX Task (信号量驱动)
  │ 从发送 FIFO 取出 CAN 帧
  │ HAL_FDCAN_AddMessageToTxFifoQ()
  ▼
关节电机 + 轮毂电机 执行力矩指令
```

### 3.5 云台板（不变）

云台板的架构与原始 sentry 架构几乎相同，变化点：
- 板间通信协议扩展：增加底盘状态反馈（腿长、倾角、离地标志）
- 云台增加**底盘姿态补偿**：当底盘倾斜时，云台反向补偿保持瞄准稳定

---

## 4. 核心算法一：五连杆运动学

### 4.1 机构描述

每条腿是一个**平面五连杆并联机构**：

```
A ──── B ──── C (末端 = 轮轴)
│ L1    │ L2
│       │
D ──── E
  L4    L3
  
固定基座: AD (安装在机身上，间距 L₅)
驱动关节: A (θ₁), D (θ₄)  ← 两个 DM 电机
被动关节: B (θ₂), E (θ₃)
末端: C (轮轴位置)
```

### 4.2 正运动学（已知 θ₁, θ₄ → 求 C 点坐标）

**目标**：从两个驱动电机的编码器角度，求解轮轴在腿坐标系中的位置、虚拟腿长和倾角。

**步骤**：

```
1. 已知量：θ₁, θ₄（编码器读数）
   已知杆长：L₁, L₂, L₃, L₄, L₅（机械参数）

2. 建立闭环矢量方程：
   B = A + L₁·[cos(θ₁), sin(θ₁)]ᵀ
   E = D + L₄·[cos(θ₄), sin(θ₄)]ᵀ
   
   约束：|B - E| = 固定值 → 解出 θ₂, θ₃

3. 利用余弦定理求 θ₂：
   BE² = L₁² + L₄² - 2·L₁·L₄·cos(θ₁ - θ₄) + L₅² - 2·L₅·(L₁·cos(θ₁) - L₄·cos(θ₄))
   
   设 ∠ABE = α：
   cos(α) = (L₁² + BE² - (L₄-L₅项)²) / (2·L₁·BE)
   
   θ₂ = atan2(Ey-By, Ex-Bx) ± α  ← 二义性：凸构型取 "+"

4. 末端坐标：
   Cx = L₁·cos(θ₁) + L₂·cos(θ₂)
   Cy = L₁·sin(θ₁) + L₂·sin(θ₂)

5. 虚拟腿参数：
   L₀ = √((Cx - L₅/2)² + Cy²)     ← 虚拟腿长
   φ₀ = atan2(Cy, Cx - L₅/2)       ← 虚拟腿倾角
```

**构型选择**：
五连杆通常有**凸构型（CONVEX）**和**凹构型（CONCAVE）**两种解。步兵轮腿使用凸构型（膝盖向外的"人腿"姿态），关节限位和状态连续性保证不会跳到凹构型。

### 4.3 逆运动学（已知目标 L₀, φ₀ → 求 θ₁, θ₄）

**目标**：给定目标虚拟腿长和倾角，反求两个驱动电机的角度。

```
1. 目标末端位置：
   Cx_target = (L₅/2) + L₀·cos(φ₀)
   Cy_target = L₀·sin(φ₀)

2. 利用余弦定理求出 θ₁：
   AC² = (Cx - Ax)² + (Cy - Ay)²
   cos(∠CAB) = (L₁² + AC² - L₂²) / (2·L₁·AC)
   θ₁ = atan2(Cy-Ay, Cx-Ax) ± ∠CAB  ← ± 取决于构型

3. 同理求 θ₄：
   DC² = (Cx - Dx)² + (Cy - Dy)²
   cos(∠CDE) = (L₄² + DC² - L₃²) / (2·L₄·DC)
   θ₄ = atan2(Cy-Dy, Cx-Dx) ∓ ∠CDE

4. 四种模式（P/N 组合）：
   根据 θ₂、θ₃ 的符号分为 PP、PN、NP、NN
   保持与当前模式一致，避免跨分支跳变
```

### 4.4 速度级正运动学

```
[dCx/dt, dCy/dt]ᵀ = J_v · [dθ₁/dt, dθ₄/dt]ᵀ

其中 J_v 是 2×2 的速度雅可比矩阵，
由末端位置对各关节角度的偏导组成。
```

### 4.5 实现代码骨架

```c
// modules/LEG/module_leg.h
typedef struct {
    float L1, L2, L3, L4, L5;   // 杆长 (m)
    float theta1, theta4;        // 驱动角 (rad)
    float theta2, theta3;        // 被动角 (rad)
    float Cx, Cy;                // 末端坐标 (m)
    float L0, phi0;              // 虚拟腿长 (m), 倾角 (rad)
    float dL0, dphi0;            // 虚拟腿变化率
    float Jacobian[2][2];        // 速度雅可比
    int8_t mode;                 // 构型: CONVEX=0, CONCAVE=1
} LegKinematics;

// 正运动学
void Leg_ForwardKinematics(LegKinematics *leg,
                           float theta1, float dtheta1,
                           float theta4, float dtheta4);

// 逆运动学
void Leg_InverseKinematics(LegKinematics *leg,
                           float L0_target, float phi0_target,
                           float *theta1_out, float *theta4_out);

// 计算雅可比
void Leg_ComputeJacobian(LegKinematics *leg);
```

---

## 5. 核心算法二：雅可比矩阵与 VMC

### 5.1 VMC 的核心思想

**虚拟模型控制（Virtual Model Control）**将复杂的五连杆机构，通过雅可比矩阵"降维"等效为一条**可变长度的虚拟弹簧-阻尼腿**。

```
真实的物理系统：
  2 个 DM 电机 → 五连杆 → 轮轴位置

VMC 的虚拟等效：
  1 条虚拟腿（长 L₀, 倾角 φ₀）→ 沿腿方向的推力 F₀
                             → 绕中心轴的力矩 Tp
```

### 5.2 虚功原理

根据**虚功原理**（Principle of Virtual Work）：

```
关节空间做的功 = 操作空间做的功

τᵀ · δq = Fᵀ · δx

其中：
  τ = [τ₁, τ₂]ᵀ          ← 关节力矩
  δq = [δθ₁, δθ₂]ᵀ       ← 关节虚位移
  F = [F₀, Tp]ᵀ          ← 虚拟力（推力 + 力矩）
  δx = [δL₀, δφ₀]ᵀ       ← 操作空间虚位移
```

由 `δx = J · δq` 和 `τ = Jᵀ · F` 得出核心公式：

```
[τ₁]        [F₀]
[τ₂] = Jᵀ · [Tp]
```

### 5.3 雅可比矩阵的推导

雅可比矩阵 J 是 2×2 的矩阵，将关节速度映射到操作空间速度：

```
[dL₀/dt]   [j11  j12]   [dθ₁/dt]
[dφ₀/dt] = [j21  j22] · [dθ₂/dt]
```

**J 各元素的物理意义**：
- j11：θ₁ 变化对虚拟腿长 L₀ 的影响
- j12：θ₂ 变化对虚拟腿长 L₀ 的影响
- j21：θ₁ 变化对虚拟腿倾角 φ₀ 的影响
- j22：θ₂ 变化对虚拟腿倾角 φ₀ 的影响

**计算**：J 可以通过运动学方程的偏导数解析计算，也可以数值差分：

```c
void Leg_ComputeJacobian(LegKinematics *leg) {
    float eps = 1e-6f;
    float L0_orig = leg->L0, phi0_orig = leg->phi0;
    
    // 扰动 θ₁
    Leg_ForwardKinematics(leg, leg->theta1 + eps, 0, leg->theta4, 0);
    float dL0_dt1 = (leg->L0 - L0_orig) / eps;
    float dphi0_dt1 = (leg->phi0 - phi0_orig) / eps;
    
    // 恢复 + 扰动 θ₄
    Leg_ForwardKinematics(leg, leg->theta1, 0, leg->theta4 + eps, 0);
    float dL0_dt4 = (leg->L0 - L0_orig) / eps;
    float dphi0_dt4 = (leg->phi0 - phi0_orig) / eps;
    
    // 恢复
    Leg_ForwardKinematics(leg, leg->theta1, 0, leg->theta4, 0);
    
    // 填充雅可比
    leg->Jacobian[0][0] = dL0_dt1;   leg->Jacobian[0][1] = dL0_dt4;
    leg->Jacobian[1][0] = dphi0_dt1; leg->Jacobian[1][1] = dphi0_dt4;
}
```

### 5.4 VMC 力分配实现

```c
// modules/LEG/module_vmc.c
void VMC_ComputeTorques(LegKinematics *leg,
                        float F0,      // 虚拟推力 (LQR 输出, N)
                        float Tp,      // 髋力矩 (LQR 输出, Nm)
                        float *tau1_out, float *tau2_out)
{
    // 1. 更新正运动学（从当前关节角度计算 L₀, φ₀）
    Leg_ForwardKinematics(leg, leg->theta1, leg->dtheta1,
                               leg->theta4, leg->dtheta4);
    
    // 2. 计算雅可比矩阵
    Leg_ComputeJacobian(leg);
    
    // 3. 虚功方程: τ = Jᵀ · F
    float J_T[2][2] = {
        {leg->Jacobian[0][0], leg->Jacobian[1][0]},  // Jᵀ
        {leg->Jacobian[0][1], leg->Jacobian[1][1]}
    };
    
    *tau1_out = J_T[0][0] * F0 + J_T[0][1] * Tp;
    *tau2_out = J_T[1][0] * F0 + J_T[1][1] * Tp;
    
    // 4. 扭矩限幅
    *tau1_out = VAL_LIMIT(*tau1_out, -MAX_JOINT_TORQUE, MAX_JOINT_TORQUE);
    *tau2_out = VAL_LIMIT(*tau2_out, -MAX_JOINT_TORQUE, MAX_JOINT_TORQUE);
}
```

### 5.5 物理直观

```
情况 1: 纯推力 F₀ > 0 (向上支撑)，Tp = 0
  → τ₁ ∝ +F₀, τ₂ ∝ -F₀（两个电机"对推"，如剪刀夹紧）
  → 腿变短 → 产生向上的支撑力

情况 2: 纯力矩 Tp > 0 (向前倾斜力矩)，F₀ = 0
  → τ₁ ∝ +Tp, τ₂ ∝ +Tp（两个电机同向发力）
  → 机身向前倾斜 → 补偿前倾

情况 3: 组合 (F₀ > 0, Tp > 0)
  → τ₁ = Jᵀ₁₁·F₀ + Jᵀ₁₂·Tp
  → τ₂ = Jᵀ₂₁·F₀ + Jᵀ₂₂·Tp
  → 同时提供支撑力 + 姿态力矩
```

---

## 6. 核心算法三：LQR 平衡控制

### 6.1 系统建模：轮腿倒立摆

将机器人在纵向平面（sagittal plane）中简化为**轮式倒立摆（WIP）**模型：

```
状态向量 (6 维):
  x = [θ,  θ̇,  x,  ẋ,  φ,  φ̇]ᵀ
  
  θ:  腿相对于竖直方向的偏角 (rad)
  θ̇: 腿偏角的角速度 (rad/s)
  x:  轮子水平位移 (m)
  ẋ:  轮子水平速度 (m/s)
  φ:  机身俯仰角 (rad)
  φ̇: 机身俯仰角速度 (rad/s)
```

**符号约定**（右腿为例）：
- 机身前倾 → φ > 0 → 需要腿向前蹬（增大 θ）
- 腿前摆 → θ > 0
- 轮子前滚 → x 增大

### 6.2 线性化状态空间模型

牛顿-欧拉力学建模后在平衡点线性化：

```
ẋ = A·x + B·u

A = [0,    1,    0,    0,    0,    0  ]   B = [0,       0   ]
    [a21,  a22,  0,    0,    a25,  a26]       [b21,     b22]
    [0,    0,    0,    1,    0,    0  ]       [0,       0   ]
    [a41,  a42,  0,    0,    a45,  a46]       [b41,     b42]
    [0,    0,    0,    0,    0,    1  ]       [0,       0   ]
    [a61,  a62,  0,    0,    a65,  a66]       [b61,     b62]

控制输入:
  u = [T_wheel, T_hip]ᵀ
  T_wheel: 轮毂电机扭矩 (Nm)
  T_hip:   髋关节等效扭矩 (Nm)
```

**A、B 矩阵的推导依赖于**：
- 机身质量 M_body、腿质量 M_leg、轮质量 M_wheel
- 机身转动惯量 I_body
- 当前虚拟腿长 L₀（A/B 矩阵随腿长变化！）
- 重力加速度 g

### 6.3 LQR 求解

**目标**：找到反馈增益矩阵 K（2×6），使控制律 `u = -K·x` 最小化：

```
J = ∫(xᵀ·Q·x + uᵀ·R·u) dt

Q = diag(q1, q2, q3, q4, q5, q6)  ← 6×6 状态权重
R = diag(r1, r2)                   ← 2×2 控制权重

求解代数 Riccati 方程: AᵀP + PA - PBR⁻¹BᵀP + Q = 0
最优增益: K = R⁻¹BᵀP
```

**离线的 LQR 计算流程**（MATLAB/Python）：

```matlab
% 1. 定义物理参数
M_body = 18;    % kg
M_leg  = 2;     % kg
M_wheel = 0.5;  % kg
I_body = 0.8;   % kg*m^2
g = 9.81;

% 2. 对一系列腿长 {0.18, 0.19, ..., 0.30} 分别计算 A, B

% 3. 对每个腿长求解 LQR
Q = diag([500, 10, 100, 10, 800, 20]);  % 调参
R = diag([0.1, 1]);

[K, S, e] = lqr(A, B, Q, R);

% 4. 多项式拟合：K(L₀) = a·L₀³ + b·L₀² + c·L₀ + d
for i = 1:2
    for j = 1:6
        p = polyfit(L0_range, K_vals(i,j,:), 3);
        K_poly(i,j,:) = p;
    end
end
```

### 6.4 增益调度

由于腿长 L₀ 变化，A/B 矩阵随之变化，K 也必须随腿长变化。利用三次多项式拟合实现**增益调度（Gain Scheduling）**：

```c
// 运行时根据当前腿长计算 K 矩阵
void Balance_GetGains(float L0, float K[2][6]) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 6; j++) {
            float *c = K_poly_coeffs[i][j];  // 预存的系数
            K[i][j] = c[0]*L0*L0*L0 + c[1]*L0*L0 + c[2]*L0 + c[3];
        }
    }
}
```

### 6.5 LQR 平衡控制实现

```c
// modules/BALANCE/module_balance.c
void Balance_Control(float L0_left, float phi0_left,
                     float L0_right, float phi0_right,
                     float wheel_speed_left, float wheel_speed_right,
                     const Ins_t *ins,
                     const RemoteCmd_t *remote,
                     BalanceOutput_t *output)
{
    // 1. 构建状态向量
    float theta     = (phi0_left + phi0_right) / 2.0f;  // 平均腿倾角
    float d_theta   = ins->gyro[1];                      // pitch 角速度
    float x         = (wheel_pos_left + wheel_pos_right) / 2.0f;
    float dx        = (wheel_speed_left + wheel_speed_right) / 2.0f;
    float phi       = ins->euler_rad[1];                 // 机身俯仰角
    float d_phi     = ins->gyro[1];                      // 俯仰角速度
    
    // 2. 增益调度
    float K[2][6];
    float L0_avg = (L0_left + L0_right) / 2.0f;
    Balance_GetGains(L0_avg, K);
    
    // 3. 计算控制量
    float T_wheel = -(K[0][0]*theta + K[0][1]*d_theta + K[0][2]*x
                    + K[0][3]*dx    + K[0][4]*phi    + K[0][5]*d_phi);
    float T_hip   = -(K[1][0]*theta + K[1][1]*d_theta + K[1][2]*x
                    + K[1][3]*dx    + K[1][4]*phi    + K[1][5]*d_phi);
    
    // 4. 叠加遥控器指令
    T_wheel += remote->chassis_vx * VX_TO_TORQUE_GAIN;
    
    // 5. 转向力矩（差速）
    float steer_torque = PD_Steering(remote->chassis_wz, ins->gyro[2]);
    
    // 6. 输出分配
    output->wheel_torque_left  = T_wheel - steer_torque;
    output->wheel_torque_right = T_wheel + steer_torque;
    
    // 7. VMC 力分配
    VMC_ComputeTorques(&left_leg,  T_hip * 0.5f, T_hip, 
                       &output->tau_left_1,  &output->tau_left_2);
    VMC_ComputeTorques(&right_leg, T_hip * 0.5f, T_hip,
                       &output->tau_right_1, &output->tau_right_2);
}
```

### 6.6 转向控制

```
差速转向：左右轮力矩不对称产生偏航力矩

  T_left  = T_wheel - K_steer * (wz_target - wz_measured)
  T_right = T_wheel + K_steer * (wz_target - wz_measured)
  
  K_steer 通过 PD 控制器实现
```

### 6.7 双腿协调（防劈叉）

```
左右腿角度差：δθ = θ_left - θ_right

补偿力矩：Tp_corr = Kp_leg * δθ + Kd_leg * d(δθ)/dt

补偿力矩符号相反施加到左右腿：
  Tp_left  += Tp_corr
  Tp_right -= Tp_corr
```

### 6.8 离心力补偿

转弯时离心力使机身外倾，通过调整双腿虚拟推力差来补偿：

```
F0_left  = F0 + K_roll * (roll - roll_target)
F0_right = F0 - K_roll * (roll - roll_target)
```

### 6.9 离地检测与空中姿态

```c
// 计算驱动轮的支持力（通过动力学方程或力传感器）
float F_N_left  = compute_normal_force(&left_leg, motor_torques);
float F_N_right = compute_normal_force(&right_leg, motor_torques);

bool airborne = (F_N_left < 20.0f) && (F_N_right < 20.0f);

if (airborne) {
    // 空中：仅保持腿竖直，不施加平衡控制
    // 轮毂力矩归零
    output->wheel_torque_left  = 0;
    output->wheel_torque_right = 0;
    // 保持当前腿长
    output->F0 = gravity_compensation;
    // 保持腿竖直 (通过关节位置控制)
}
```

---

## 7. 模块设计

### 7.1 新增模块：LEG

```c
// modules/LEG/module_leg.h
#ifndef MODULE_LEG_H
#define MODULE_LEG_H

#include "tx_api.h"

// 五连杆参数（可从 CMake 覆盖）
#ifndef LEG_L1
#define LEG_L1 0.15f  // 上连杆长度 (m)
#endif
#ifndef LEG_L4
#define LEG_L4 0.15f  // 上连杆长度 (m)
#endif
// ... L2, L3, L5 ...

// 每条腿的运行时状态
typedef struct {
    float theta1, theta4;        // 驱动关节角度 (rad)
    float dtheta1, dtheta4;      // 驱动关节角速度 (rad/s)
    float Cx, Cy;                // 末端坐标 (m)
    float L0, phi0;              // 虚拟腿长 + 倾角
    float Jacobian[2][2];        // 雅可比矩阵
    int8_t mode;                 // CONVEX=0, CONCAVE=1
} LegState;

// API
void Leg_ForwardKinematics(LegState *leg, float t1, float dt1, float t4, float dt4);
void Leg_InverseKinematics(LegState *leg, float L0_tgt, float phi0_tgt,
                           float *t1_out, float *t4_out);
void Leg_ComputeJacobian(LegState *leg);
void VMC_ComputeTorques(LegState *leg, float F0, float Tp,
                        float *tau1_out, float *tau2_out);

void Module_Leg_Init(void);
LegState *Module_Leg_GetLeft(void);
LegState *Module_Leg_GetRight(void);

#endif
```

### 7.2 新增模块：BALANCE

```c
// modules/BALANCE/module_balance.h
#ifndef MODULE_BALANCE_H
#define MODULE_BALANCE_H

// LQR 参数
#define BALANCE_STATE_DIM    6    // [θ, θ̇, x, ẋ, φ, φ̇]
#define BALANCE_CONTROL_DIM  2    // [T_wheel, T_hip]

// 增益多项式的系数（离线计算，硬编码）
typedef struct {
    float K_coeffs[2][6][4];   // [控制][状态][次数] 系数
    float K_steer_p, K_steer_d;
    float K_leg_sync_p, K_leg_sync_d;
    float K_roll_comp;
    float max_wheel_torque;
    float max_joint_torque;
} BalanceConfig;

typedef struct {
    float wheel_torque_left, wheel_torque_right;
    float tau_left_1, tau_left_2;    // 左腿关节力矩
    float tau_right_1, tau_right_2;  // 右腿关节力矩
} BalanceOutput;

void Balance_GetGains(float L0, float K[2][6]);
void Balance_Control(float L0_l, float phi0_l, float L0_r, float phi0_r,
                     float v_l, float v_r,
                     const Ins_t *ins, const RemoteCmd_t *rc,
                     BalanceOutput_t *out);
void Module_Balance_Init(void);

#endif
```

### 7.3 修改模块：MOTOR（添加 DM-MIT 支持）

```c
// modules/MOTOR/module_motor.h — 新增内容

// DM 电机 MIT 模式控制
typedef struct {
    uint16_t can_id;         // CAN ID
    float    position;       // 编码器位置 (rad)
    float    velocity;       // 编码器速度 (rad/s)
    float    torque;         // 力矩指令 (Nm)
    float    kp_mit;         // MIT 模式 Kp
    float    kd_mit;         // MIT 模式 Kd
    float    max_torque;     // 最大力矩
} DM_Motor;

void DM_Motor_SetMIT(DM_Motor *motor, float pos_des, float vel_des, float torque_ff);
void DM_Motor_SendCAN(DM_Motor *motor);
```

**MIT 模式协议**（达妙电机标准）：
```
CAN 数据帧 (8 字节):
  [pos_des(2B)] [vel_des(2B)] [kp(2B)] [kd(2B)] [torque_ff(2B, 实际只有12位)]

  pos_des: float × 1000 → int16
  vel_des: float × 1000 → int16
  kp:      float × 100  → int16
  kd:      float × 100  → int16
  torque:  float × 1000 → int12 (0-4095)
```

### 7.4 模块初始化

```c
// modules/module_init.c — 新增内容
#if MODULE_LEG
#include "module_leg.h"
#endif
#if MODULE_BALANCE
#include "module_balance.h"
#endif

void MODULE_Init(void) {
    // ... 原有模块 ...
#if MODULE_LEG
    Module_Leg_Init();
#endif
#if MODULE_BALANCE
    Module_Balance_Init();
#endif
}
```

---

## 8. CAN 通信架构

### 8.1 发送周期

| 数据 | 周期 | CAN ID | 优先级 |
|------|------|--------|--------|
| 关节电机 MIT 指令 | 1 ms | 0x201-0x204 | 高 |
| 轮毂电机转矩指令 | 1 ms | 0x205-0x206 | 高 |
| 云台电机指令 | 2 ms | 0x207-0x20A | 中 |
| 板间通信 | 5 ms | 0x2FF, 0x2FE | 低 |

### 8.2 板间通信协议（扩展）

```c
// 底盘 → 云台
typedef struct {
    int8_t  vx, vy, wz;            // 遥控速度
    int16_t offset_angle;          // yaw 偏移
    uint8_t chassis_mode;          // 底盘模式
    // 🆕 轮腿扩展
    float   L0_left, L0_right;     // 虚拟腿长
    float   body_pitch, body_roll; // 机身为云台姿态补偿
    uint8_t airborne;              // 离地标志
    int8_t  reserved[1];
} ChassisToGimbal_Leg_t;  // 必须 ≤ 8 字节或拆分为多帧
```

### 8.3 云台姿态补偿

当底盘倾斜时，云台需要反向补偿保持枪管指向目标：

```c
// 云台板 gimbal_func.c
void Gimbal_ApplyChassisCompensation(Gimbal_Ctrl_Cmd_t *cmd,
                                     float body_pitch, float body_roll)
{
    // 底盘前倾 body_pitch 度 → 云台需要后仰 body_pitch 度
    // 底盘右倾 body_roll 度  → 云台需要左倾 body_roll 度
    cmd->pitch -= body_pitch;
    
    // Yaw 也需要补偿（底盘横滚会改变 yaw 的投影）
    cmd->yaw -= body_roll * sin(cmd->pitch * DEGREE_2_RAD);
}
```

---

## 9. 状态机与安全保护

### 9.1 底盘状态机

```
                上电
                 │
                 ▼
         ┌─────────────┐
         │   INIT       │  初始化 + 自检
         │  关节找零点   │
         └──────┬──────┘
                │ 自检通过
                ▼
         ┌─────────────┐
         │   IDLE       │  就绪，等待遥控器指令
         └──────┬──────┘
                │ 遥控器拨到启用档
                ▼
         ┌─────────────┐
    ┌───→│  BALANCE    │←──┐ 平衡运行（正常状态）
    │    │  LQR+VMC    │   │
    │    └──────┬──────┘   │
    │           │           │
    │    离地检测│           │ 着陆检测
    │           ▼           │
    │    ┌─────────────┐   │
    │    │  AIRBORNE   │───┘ 空中状态（跳跃/飞坡）
    │    │  保持姿态    │
    │    └─────────────┘
    │
    │    跌倒检测（俯仰角 > 60°）
    │           │
    │           ▼
    │    ┌─────────────┐
    │    │  FALLEN     │  跌倒
    │    │  电机泄力    │
    │    └──────┬──────┘
    │           │ 自主复位
    │           ▼
    │    ┌─────────────┐
    └────│  RECOVERY   │  恢复（缓慢站起来）
         └─────────────┘
```

### 9.2 安全保护

```c
// 1. 关节限位
if (theta1 < THETA1_MIN || theta1 > THETA1_MAX) {
    emergency_stop = true;
}

// 2. 过流保护
if (abs(motor_current) > MAX_CURRENT) {
    motor_torque_limit *= 0.5f;  // 半扭矩运行
    overcurrent_count++;
    if (overcurrent_count > 100) emergency_stop = true;
}

// 3. 跌倒检测
float pitch = ins->euler_rad[1] * RAD_2_DEGREE;
if (abs(pitch) > 60.0f) {
    state = FALLEN;
    Balance_EmergencyStop();
}

// 4. 通信超时
if (boardcomm_last_rx_time > 500ms) {
    // 云台停转，底盘保持当前状态
    gimbal_cmd->mode = gimbal_zero_force;
}

// 5. 离地超时（防止卡在空中）
if (airborne && airborne_duration > AIRBORNE_TIMEOUT) {
    // 降低腿长到底，准备硬着陆
    Leg_SetLength(MIN_LEG_LENGTH);
}
```

---

## 10. 调试与参数整定

### 10.1 调试阶段

```
第一阶段：板凳模型
  固定腿长（用机械限位），只调试轮毂 LQR
  目标：轮子能稳定平衡不倒
  控制：仅 T_wheel，T_hip = 0
  状态：4 状态 [θ, θ̇, x, ẋ]
  时间：1-2 天

第二阶段：一阶倒立摆
  关节电机 MIT 模式打开，允许腿角度变化
  目标：轮毂+关节联合保持平衡
  控制：T_wheel + T_hip
  状态：6 状态 [θ, θ̇, x, ẋ, φ, φ̇]
  时间：2-3 天

第三阶段：可变腿长
  允许腿长动态变化
  目标：平衡的同时可控制高度
  控制：增益调度 LQR
  时间：1-2 天

第四阶段：运动控制
  平衡状态下响应遥控器指令
  目标：前进后退转弯不倒下
  时间：2-3 天

第五阶段：越障+跳跃
  离地检测 + 空中姿态
  目标：台阶攀爬、飞坡着陆
  时间：3-5 天
```

### 10.2 极性验证

**这是调试中最容易出错的环节！**

```c
// 测试 1: 轮毂极性
// 机器人前倾（φ > 0）→ 轮子应该前滚（T_wheel > 0）
// 如果轮子后滚 → 极性反了 → 取反

// 测试 2: 关节极性
// 只开 lqr[0]（倾角项），注释掉其他
// 机器人前倾 → 腿应该向前蹬（θ 增大）
// 如果腿向后蹬 → 极性反了 → 取反

// 测试 3: MIT 位移/速度极性
// 只开 lqr[4]（机身俯仰角项）
// 机器人前倾 → 腿应该向后蹬（拉回机身）
// MIT 模式：pos_des 应该是减小还是增大？→ 查电机手册

// 测试 4: 左右腿坐标系
// 左腿和右腿的坐标系可能相反
// 分别测试：推动左腿、右腿 → 观察各自的编码器变化方向
```

### 10.3 MATLAB 离线计算辅助

```matlab
% 1. 计算 A, B 矩阵
syms M_body M_leg M_wheel I_body L0 g real
% ... 牛顿-欧拉推导 ...

% 2. 对腿长范围计算 K 矩阵
L0_range = 0.18:0.01:0.30;
K_all = zeros(2, 6, length(L0_range));

for i = 1:length(L0_range)
    [A, B] = compute_AB(L0_range(i), params);
    K_all(:,:,i) = lqr(A, B, Q, R);
end

% 3. 多项式拟合
K_coeffs = zeros(2, 6, 4);  % 三次多项式
for ctrl = 1:2
    for state = 1:6
        K_coeffs(ctrl, state, :) = polyfit(L0_range, ...
            squeeze(K_all(ctrl, state, :))', 3);
    end
end

% 4. 导出为 C 数组
fprintf('float K_coeffs[2][6][4] = {\n');
% ... 自动生成 C 代码 ...
fprintf('};\n');
```

### 10.4 实时调参技巧

```c
// 在遥控器上绑定参数调节通道
// CH5: Q 的 q1（倾角权重）缩放因子，0.5x ~ 2.0x
// CH6: Q 的 q5（俯仰角权重）缩放因子，0.5x ~ 2.0x

float q1_scale  = (remote->ch5 + 1.0f) / 2.0f * 3.0f + 0.5f;  // 0.5 ~ 3.5
float q5_scale  = (remote->ch6 + 1.0f) / 2.0f * 3.0f + 0.5f;

// 用缩放后的 Q 重新计算 K（在线做三次多项式求值成本极低）
// 或者预计算多组 K 表，根据 scale 查表插值
```

### 10.5 常见问题排查

| 问题 | 可能原因 | 排查方法 |
|------|---------|---------|
| 机器人前后剧烈震荡 | LQR K 值过大 | 减小 Q 的 q1（倾角权重） |
| 机器人慢慢倒下 | LQR K 值过小 | 增大 Q 的 q1 |
| 原地抖动 | 陀螺仪噪声过大 | 增大 LPF 系数，检查 IMU 安装 |
| 转弯时向外倾倒 | Roll 补偿不够 | 增大 `K_roll_comp` |
| 腿突然向上弹起 | 关节电机 MIT 位移项极性反了 | 检查 ×(-1) |
| 腿越走越短/越长 | 位置环无积分项或积分饱和 | 添加抗积分饱和 |
| 上电腿不动 | 关节电机零点未设置 | 跑零点标定程序 |
| 离地后落地姿态错乱 | 空中姿态保持未生效 | 检查离地检测阈值 |

---

## 11. 高级功能算法拓展

> 📘 本章内容综合了偏置并联轮腿（郑派成毕业设计，2026）中的通用算法成果，这些算法**不限于特定腿机构**，同样适用于串联轮腿步兵。详见 [[RM_Chassis_偏置并联轮腿学习指南]]。

### 11.1 增量 MPC 鲁棒性补偿

#### 背景问题

LQR 理论上有良好的鲁棒性裕量（增益裕度 > 6dB，相位裕度 > 60°），但引入 **Kalman 滤波器后这些裕量会失效**。而状态观测器（如 Kalman 滤波器）在实际系统中不可避免。完整 MPC 需要在线求解二次规划 → STM32 无法实时计算。

#### 增量 MPC 方案

核心思路：构造增量式状态空间模型，将 QP 问题转化为**直接求导求解析解**。

```
步骤 1：构造增量模型
    Δx(k) = x(k) - x(k-1)
    Δu(k) = u(k) - u(k-1)
    增广状态 z(k) = [Δx(k); y(k)]

步骤 2：预测域 N_p 步状态预测
    Z_future = F · z(k) + Φ · ΔU

步骤 3：代价函数
    J = (Fz + ΦΔU)ᵀ·Q_bar·(Fz + ΦΔU) + ΔUᵀ·R_bar·ΔU

步骤 4：求导求极值（解析解，无需 QP！）
    ΔU* = -(ΦᵀQ_barΦ + R_bar)⁻¹ · ΦᵀQ_bar · F · z(k)
```

**优点**：解析表达式中的矩阵在 MATLAB 中离线预计算，STM32 只需做矩阵-向量乘法。

**代价**：预测补偿效果比完整 MPC 有所折扣，但能提供足够的鲁棒性。

#### 实际部署

在实物中，将 MPC 补偿矩阵的第二行（髋关节力矩补偿）**反向叠加**到 LQR 的髋关节力矩输出：

```c
// LQR 输出
u_lqr = -K * (x - x_des);
// MPC 补偿（注意第二行反号）
u_mpc = compute_mpc_compensation(x, l);
u_final.tau_wheel = u_lqr.tau_wheel + u_mpc.tau_wheel;
u_final.tau_hip   = u_lqr.tau_hip   - u_mpc.tau_hip;  // 反号 → 削弱轮电机期望
```

这样做的物理含义是：**减小轮向电机的期望力矩**，避免轮电机物理极限导致的模型偏差。

#### 补偿矩阵的腿长拟合

MPC 补偿矩阵的系数同样依赖腿长 `l`，但用**双指数函数**（而非 LQR 的三次多项式）拟合：

```
K_mpc(l) = a · exp(b · l) + c · exp(d · l)
```

### 11.2 龙伯格（Luenberger）观测器优化二阶微分

#### 问题

支撑力估计和碰撞力估计都需要 `θ̈` 和 `l̈`（摆杆角度和腿长的二阶导数）。直接做差分 + 低通滤波会产生**大量尖峰噪声**，严重影响力估计的可靠性。

#### Luenberger 观测器方案

假定二阶微分为 0（即 `d³θ/dt³ = 0`, `d³l/dt³ = 0`），构建离散观测器：

```
状态向量:   x_θ = [θ, θ̇, θ̈]ᵀ   或   x_l = [l, l̇, l̈]ᵀ

预测步:     x̂⁻(k+1) = A · x̂(k)
            (A = [1  Δt  Δt²/2; 0  1  Δt; 0  0  1])

更新步:     x̂(k+1) = x̂⁻(k+1) + L · (y_meas(k) - C · x̂⁻(k+1))
            (C = [1 0 0], 仅测量一阶量 θ 或 l)
```

其中观测器增益 `L = [l₁, l₂, l₃]ᵀ` 通过极点配置或离散 Riccati 方程设计。

```c
// 伪代码实现
typedef struct {
    float x[3];       // [θ, θ̇, θ̈] 或 [l, l̇, l̈]
    float L[3];       // 观测器增益
    float dt;
} LuenbergerObserver;

float luenberger_update(LuenbergerObserver *obs, float measurement) {
    // 预测步
    float x_pred[3];
    x_pred[0] = obs->x[0] + obs->x[1] * obs->dt + 0.5f * obs->x[2] * obs->dt * obs->dt;
    x_pred[1] = obs->x[1] + obs->x[2] * obs->dt;
    x_pred[2] = obs->x[2];  // 假定二阶微分为常数

    // 更新步
    float error = measurement - x_pred[0];
    obs->x[0] = x_pred[0] + obs->L[0] * error;
    obs->x[1] = x_pred[1] + obs->L[1] * error;
    obs->x[2] = x_pred[2] + obs->L[2] * error;

    return obs->x[2];  // 返回平滑后的二阶微分
}
```

#### 实测效果

差分+低通滤波方案 vs Luenberger 观测器方案（论文图 4.2）：
- 差分+LPF：**大量尖峰噪声**，噪声幅度可达真实值的 2-3 倍
- Luenberger：**曲线平滑很多**，尖峰噪声显著减少，与真实值更接近

### 11.3 PSO 粒子群优化 LQR 参数

#### 问题

LQR 的 Q 和 R 矩阵通常靠手动调试，过程不科学且效率低下。以 6 状态 2 输入的轮腿系统为例，Q 矩阵有 6 个对角元素需要调试，参数空间大。

#### PSO 优化方案

```
搜索空间：  仅优化 R 矩阵（控制惩罚），Q 保持固定
适应度：    0.8 × |轮向力矩峰值| + 0.2 × |机体俯仰角峰值|
            （8:2 加权 —— 左项保证鲁棒性，右项保证平稳性）

评估平台：  Simscape 仿真（参数与实物一致）
            运行全流程（前进+越障），记录力矩和姿态数据

PSO 配置：  30 个粒子，100 次迭代
```

#### 实现框架

```python
# Python/Simulink 联合仿真伪代码
import numpy as np
from pyswarm import pso

def fitness(R_params):
    # 1. 将 R_params 填入 Simscape 模型
    set_model_R_matrix(R_params)
    # 2. 运行仿真
    sim_out = sim('wheel_leg_model', 'StopTime', '10')
    # 3. 提取指标
    torque_peak = np.max(np.abs(sim_out.torque_wheel))
    pitch_peak = np.max(np.abs(sim_out.pitch_angle))
    # 4. 加权适应度（越小越好）
    return 0.8 * torque_peak + 0.2 * pitch_peak

# PSO 搜索
lb = [0.1, 0.1, 0.1, 0.1]    # R 下界
ub = [100, 100, 100, 100]     # R 上界
optimal_R, _ = pso(fitness, lb, ub, swarmsize=30, maxiter=100)
```

#### 典型优化结果

| 指标 | 手动调参（原 R） | PSO 优化后 |
|------|----------------|-----------|
| R 对角元 | diag(10, 10, 10, 10) | 约 diag(3.85, 5.97, 11.03, 7.28) |
| 轮向峰值力矩 | ~2.25 N·m | ~1.45 N·m（↓35%） |
| 力矩尖峰频率 | 较多 | 显著减少 |

> ⚠️ **注意**：PSO 不能完全替代手动调参。实际使用中 PS0 提供初始值，再根据实车表现微调。

### 11.4 主动柔顺控制

#### 问题

位移控制模式下撞墙或顶到障碍物时，位移误差持续累加 → 积分饱和 → 系统发散。

#### 解决方案

估计机体所受外力 → 调整位移期望主动"退让"：

```
外力估计: F_ext = [基于轮向力矩 + 机体加速度的力平衡推导]

柔顺律:   Δx_desired = K_compliance · F_ext

实际位移期望: x_desired_final = x_desired_original - Δx_desired
```

效果等价于在机器人与环境之间加入了**虚拟弹簧-阻尼**：

```
F_ext = k_virtual · Δx + d_virtual · Δẋ
```

```c
// 主动柔顺控制伪代码
float estimate_external_force(void) {
    // 从轮向力矩和加速度估计外力
    // 简化：忽略减速箱力矩噪声
    float accel_x = get_body_accel_x();
    return mass_body * accel_x;  // F = ma 近似
}

void compliance_control(float *x_desired) {
    float F_ext = estimate_external_force();
    // 当外力超过死区时，主动退让位移期望
    if (fabsf(F_ext) > DEADBAND_FORCE) {
        float dx = K_COMPLIANCE * F_ext;
        dx = CLAMP(dx, -MAX_COMPLIANCE_DX, MAX_COMPLIANCE_DX);
        x_desired[2] -= dx;  // 修改位移期望
    }
}
```

### 11.5 运动估计与打滑补偿

#### 问题

不能直接取左右轮电机角速度平均作为机体速度：
1. 轮电机**定子随腿旋转** → 需要补偿定子角速度
2. 轮子**打滑** → 编码器速度不等于真实对地速度
3. 几何关系需要正确转换（摆杆角度影响水平位移分量）

#### 速度解算

```
步骤 1：轮子对地角速度
    ω_ground = ω_rotor_to_stator + ω_stator_to_leg + ω_leg_to_body + ω_body_to_ground

步骤 2：轮向水平速度（两种方法）
    方法 A（轮速法）：v_wheel = ω_ground · r
    方法 B（几何法）：v_body = [由 θ, l, θ̇, l̇ 的几何关系推导]

步骤 3：融合
    v_body_meas = (v_wheel_left + v_wheel_right) / 4
                + (v_body_geo_left + v_body_geo_right) / 4
    对速度积分得位移
```

#### 打滑补偿 Kalman 滤波器

```
状态向量:    x = [v_x, bias_accel]ᵀ
测量向量:    z = [v_encoder, a_accel]ᵀ

状态转移:    v_x(k+1) = v_x(k) + a_accel(k) · Δt + w_v
            bias(k+1) = bias(k) + w_bias

测量模型:    v_encoder(k) = v_x(k) + slip_noise    ← 打滑引起噪声
            a_accel(k) = a_true(k) + bias(k) + noise
```

**关键**：加速度计**不会打滑** → Kalman 滤波器根据加速度计数据自动抑制打滑引起的编码器速度噪声。

### 11.6 大风车式翻倒自救（需硬件支持）

> ⚠️ **注意**：此功能需要腿机构能 **360° 全周旋转**。串联轮腿的线缆和结构可能不支持，需确认硬件设计。

#### 核心原理

利用腿部全周旋转能力，通过"别地"（block against ground）使翻倒的机器人恢复站立。

#### 四步流程

**Step 1：腿部旋转速度环**

```c
// 重力补偿 + PD 速度控制
float tau_gravity = MASS_LEG * G * d_cm * cosf(angle_leg + phase_offset);
float tau_pd = KP_ROTATION * (omega_desired - omega_actual);
float tau_leg = tau_pd + tau_gravity;
```

**Step 2：腿部姿态纠正**

判断 `θ_leg` 是否在异常区间 → 控制腿部向后摆到同步位置。

**Step 3：机体翻转处理**

- 情况 1（向后翻）：两条腿同步逆时针旋转 → 用腿"别"地 → 机体恢复水平
- 情况 2（向前翻）：两条腿同步顺时针旋转 → 别地 → 进入姿态纠正

**Step 4：丝滑起身**

```c
// 先闭环到同步位置
control_theta_to(THETA_SYNC);
// 再闭环到平衡位置附近
control_theta_to(THETA_BALANCE_NEAR);
// 最后启动 LQR 控制器（放大死区范围 → 丝滑过渡）
lqr_enable_with_large_deadband();
```

### 11.7 撞击式越障与碰撞力估计

#### 碰撞力估计

对驱动轮做水平方向受力分析，利用机体加速度和轮向力矩估计碰撞力：

```
F_collision = [由 F = ma + τ_w/r + 几何变换推导]
```

实际中考虑轮电机减速箱噪声，需对力矩项做滤波处理。

#### 越障策略（碰撞-收腿-自起）

```
状态机：
  NORMAL ──(碰撞力>阈值)──→ DETECTED
  DETECTED ──→ 伸高腿长（趴在障碍高处）
           ──→ 停止 LQR（暂停平衡控制）
           ──→ 收腿到障碍高处
           ──→ 重新自起（恢复 LQR）
           ──→ NORMAL

实测结果：200mm 台阶，通过率 ≈ 100%
```

```c
// 碰撞越障状态机伪代码
enum ObstacleState {
    OS_NORMAL,
    OS_LEG_EXTEND,     // 伸腿趴在障碍上
    OS_LQR_STOP,       // 暂停平衡
    OS_LEG_RETRACT,    // 收腿
    OS_REBALANCE,      // 重新平衡
};

if (collision_force > THRESHOLD_COLLISION && state == OS_NORMAL) {
    state = OS_LEG_EXTEND;
    target_leg_length = OBSTACLE_HEIGHT + MARGIN;
}
// ... 各状态处理
```

### 11.8 跳跃过障模型

#### 适用场景

障碍高度超过碰撞越障极限（~200mm）时，使用跳跃方式。需配置**气弹簧**辅助。

#### 基于机械能守恒的跳跃模型

**起跳阶段**（沿腿长方向推力从 `l_min` 推到 `l_max`）：

```
½ · m · v_y² = ∫_{l_min}^{l_max} (F_thrust - mg) · dl

起跳速度:      v_y = √[2/m · ∫(F_thrust - mg) · dl]
起跳时间:      t_thrust = [通过 v_y, F_thrust 推导]
```

**空中阶段**（斜抛运动）：

```
最大高度:      H_max = v_y² / (2g)
达到最高点:    t_peak = v_y / g
```

**收腿阶段**：

```
收腿推力:      F_retract = [通过收腿长度 + 允许时间约束求解]
```

**落地阶段**：

```
落地时间:      t_fall = √[2 · (H_max - h_obstacle) / g]
```

#### 姿态优化

问题：空中保持摆杆 `θ` 竖直 → 产生反作用转矩 → 机体翻转。

解决：**空中保持机体水平**而非摆杆竖直 + 落地前主动伸腿缓冲。

#### 实际限制

- 跳跃高度依赖气弹簧性能
- 电机无法同时满足高转速 + 高转矩 → 跳跃高度和收腿长度一般低于理论计算值
- 含气弹簧实测：最高 **420mm** 跳跃；优化后稳定跳跃：**200mm**

---

## 附录 A：参考资源

| 资源                                                                                      | 说明                                  |
| --------------------------------------------------------------------------------------- | ----------------------------------- |
| [WilliamGwok/RP_Balance](https://github.com/WilliamGwok/RP_Balance)                     | RM2024 五连杆轮足机器人，含 VMC+LQR MATLAB 代码 |
| [中国石油大学 SPR 轮腿开源](https://bbs.robomaster.com/article/1884251)                           | RM2026 轮腿代码开源                       |
| [zeeklog 轮腿调试系列](https://zeeklog.com/lun-tui-ji-qi-ren-dai-ma-diao-shi-bu-chong-4)      | 完整硬件→代码→极性调试全流程                     |
| [PennState RoboX 电控指南](https://github.com/PennState-RoboX/RobomasterControlHandbook)    | 电控组入门手册                             |
| 《RoboMaster竞赛步兵机器人设计》(清华大学出版社, 2025)                                                    | 第7章：轮腿式平衡机器人专项                      |
| [TAROS 2025: LQR+SMC for Wheel-Legged Robot](https://core.ac.uk/download/656119578.pdf) | LQR+滑模控制融合算法                        |

## 附录 B：新增文件清单

```
mas_embedded_threadx/
├── apps/
│   └── infantry_leg/                    # 🆕 新机器人
│       ├── robot.cmake                  # 模块列表 + 参数
│       ├── chassis_board/
│       │   ├── robot_control.c          # 底盘主循环
│       │   ├── balance_control.c        # LQR 平衡控制
│       │   ├── leg_control.c            # 腿长+姿态控制
│       │   └── chassis_func.c           # 腿部运动学（重写）
│       └── gimbal_board/                # 云台板（从 sentry 复用）
│           ├── robot_control.c
│           ├── gimbal_func.c            # + 底盘姿态补偿
│           └── shoot_func.c
├── modules/
│   ├── LEG/                             # 🆕 五连杆+VMC
│   │   ├── module_leg.h
│   │   ├── module_leg.c
│   │   └── module_vmc.c
│   ├── BALANCE/                         # 🆕 LQR 平衡
│   │   ├── module_balance.h
│   │   └── module_balance.c
│   ├── MOTOR/                           # 扩展
│   │   ├── motor_dm.c                   # 🆕 DM 电机 MIT 模式
│   │   └── motor_base.h                 # 扩展 Motor_Type_e
│   └── algorithm/
│       ├── lqr.c/h                      # 扩展：6 状态 + 增益调度
│       └── kinematics.c/h               # 🆕 五连杆运动学
└── board/
    └── bsp/                             # 保持不变
```


---

## 相关笔记

- [[RM_Chassis_串联轮腿数学推导与仿真]] — 五连杆运动学完整推导 + WIP 拉格朗日动力学 + Python/MATLAB 仿真
- [[RM_Chassis_偏置并联轮腿学习指南]] — 偏置并联轮腿（郑派成毕业设计）：LQR+MPC 级联、大风车自救、跳跃越障、PSO 参数优化、Luenberger 观测器
- [[RM_LQR_lqr.py数学推导]] — LQR 理论数学基础
- [[RM_Chassis_底盘轮系运动学解算教程]] — 传统轮式底盘运动学
