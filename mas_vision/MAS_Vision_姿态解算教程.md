# 姿态解算教程 —— 基于 QuaternionEKF 的 IMU 姿态估计

> 📘 **数据流向**：IMU 四元数经 STM32 → 串口 → Serial 线程 → SPSCQueue(5000) → Armor 线程进行时间插值，详见 [[MAS_Vision_多线程架构学习指南]]
>
> 📘 **下游应用**：本文解算出的 IMU 姿态（欧拉角）是云台一般重力补偿中 ${}^{C}O$ 旋转矩阵的来源 → [[RM_Gimbal_重力补偿与加速度补偿详解]]

## 目录

1. [姿态解算概述](#1-姿态解算概述)
2. [坐标系与旋转表示](#2-坐标系与旋转表示)
3. [四元数数学基础](#3-四元数数学基础)
4. [传感器：BMI088 IMU](#4-传感器bmi088-imu)
5. [姿态解算方法演进](#5-姿态解算方法演进)
6. [QuaternionEKF：本项目的核心算法](#6-quaternionekf本项目的核心算法)
7. [传感器标定](#7-传感器标定)
8. [运动加速度分离](#8-运动加速度分离)
9. [数据流全链路](#9-数据流全链路)
10. [控制回路中的姿态使用](#10-控制回路中的姿态使用)
11. [调参指南](#11-调参指南)
12. [故障排查](#12-故障排查)

---

## 1. 姿态解算概述

### 1.1 什么是姿态解算

**姿态解算**（Attitude Estimation）是指通过惯性传感器（陀螺仪 + 加速度计）的数据，实时计算出物体在三维空间中的朝向（姿态）。

在哨兵机器人中，姿态解算的输出直接驱动云台控制——告诉电机"现在指向哪个方向"以及"应该指向哪个方向"。

### 1.2 为什么需要姿态解算

哨兵云台需要在没有外部参考的情况下：
- 知道自己相对于水平面的倾斜角度（pitch / roll）
- 跟踪自己的旋转角度（yaw），以便在视觉目标出现时快速对准
- 将自身姿态发送给视觉计算机，辅助目标检测和坐标变换

电机自带的编码器只能测量**相对旋转角度**，无法提供相对于大地的绝对姿态。姿态解算填补了这一空白。

### 1.3 系统的输入与输出

```
输入：
  三轴陀螺仪 (gyro_x, gyro_y, gyro_z)  ← 角速度, °/s
  三轴加速度计 (acc_x, acc_y, acc_z)   ← 加速度, m/s²

输出：
  四元数 q[4] = [w, x, y, z]           ← 表示姿态
  欧拉角 [roll, pitch, yaw]            ← 直观的角度
  运动加速度 [ax, ay, az]              ← 去除重力后的纯运动加速度
  体轴向量 [xn, yn, zn]                ← 机体坐标轴在大地系中的方向
```

---

## 2. 坐标系与旋转表示

### 2.1 两个基本坐标系

**导航坐标系（n-frame / Earth Frame）**：

![飞行器姿态角 — Roll/Pitch/Yaw](https://upload.wikimedia.org/wikipedia/commons/0/04/Flight_dynamics_with_text_ortho.svg)
*▲ 飞行器姿态角定义：Roll(滚转)、Pitch(俯仰)、Yaw(偏航) (图片来源: Wikipedia)*
```
Z ↑ (指向天空，与重力方向相反)
X → (水平面内，指向初始朝向)
Y → (水平面内，按右手定则确定)
```

这是"绝对"参考系。重力矢量在此系中恒为 `[0, 0, 9.81]`。

**机体坐标系（b-frame / Body Frame）**：
```
X 轴 — 指向机器人前方
Y 轴 — 指向机器人右侧
Z 轴 — 指向机器人上方
```

这是"相对"参考系，固定在机器人身上。陀螺仪和加速度计测量的是**机体坐标系**中的量。

### 2.2 旋转矩阵（方向余弦矩阵 DCM）

旋转矩阵 R 将机体坐标系的向量变换到导航坐标系：

```
v_n = R · v_b
```

R 是一个 3×3 的正交矩阵，列向量是机体坐标轴在导航系中的表示：
- 第 1 列 = x_b（机体前方在导航系中的方向）
- 第 2 列 = y_b（机体右方在导航系中的方向）
- 第 3 列 = z_b（机体上方在导航系中的方向）

### 2.3 欧拉角

本项目使用 ZYX 内旋（Intrinsic Rotation）顺序：

```
初始：机体与导航系对齐
1. 绕 Z 轴旋转 Yaw   (偏航, ψ)
2. 绕 Y 轴旋转 Pitch (俯仰, θ)
3. 绕 X 轴旋转 Roll  (横滚, φ)
```

**欧拉角的优缺点**：
- ✅ 直观，易于理解和调试
- ❌ 万向节死锁（Pitch = ±90° 时 Yaw 和 Roll 退化）
- ❌ 三角运算多，不适合作为滤波器状态

这就是为什么姿态解算的核心用**四元数**而非欧拉角。

---

## 3. 四元数数学基础

![欧拉角旋转序列动画](https://upload.wikimedia.org/wikipedia/commons/8/85/Euler2a.gif)
*▲ 欧拉角三次旋转序列 (Z-Y-X) 的动画演示 (图片来源: Wikipedia)*

### 3.1 什么是四元数

四元数 q = [w, x, y, z] = w + xi + yj + zk 是一个超复数，可以紧凑地表示三维旋转。

```
w = cos(θ/2)          ← 标量部分，编码旋转角度
[x, y, z] = axis · sin(θ/2)  ← 矢量部分，编码旋转轴
```

其中 θ 是旋转角度，axis 是单位旋转轴。

### 3.2 四元数基本运算

**归一化**（确保表示有效旋转）：
```
|q| = sqrt(w² + x² + y² + z²)
q_normalized = q / |q|
```

**从四元数求旋转矩阵（本项目实现）**：
```c
// BodyFrameToEarthFrame: v_n = R(q) · v_b
v_n[0] = 2*((0.5 - q2² - q3²)*v0 + (q1*q2 - q0*q3)*v1 + (q1*q3 + q0*q2)*v2)
v_n[1] = 2*((q1*q2 + q0*q3)*v0 + (0.5 - q1² - q3²)*v1 + (q2*q3 - q0*q1)*v2)
v_n[2] = 2*((q1*q3 - q0*q2)*v0 + (q2*q3 + q0*q1)*v1 + (0.5 - q1² - q2²)*v2)
```

**四元数乘法**（组合两次旋转）：
```
q_a ⊗ q_b = [w_a*w_b - x_a*x_b - y_a*y_b - z_a*z_b,
             w_a*x_b + x_a*w_b + y_a*z_b - z_a*y_b,
             w_a*y_b - x_a*z_b + y_a*w_b + z_a*x_b,
             w_a*z_b + x_a*y_b - y_a*x_b + z_a*w_b]
```

先做 q_b 旋转，再做 q_a 旋转。

### 3.3 四元数微分方程（陀螺仪更新）

四元数与角速度的关系由以下微分方程描述：

```
dq/dt = 0.5 · Ω(ω) · q

其中 Ω(ω) = [ 0   -ωx  -ωy  -ωz ]
            [ ωx   0    ωz  -ωy ]
            [ ωy  -ωz   0    ωx ]
            [ ωz   ωy  -ωx   0  ]
```

**离散化**（一阶近似，本项目使用的方法）：
```
q_{k+1} = (I + 0.5 · Ω(ω·dt)) · q_k
```

展开即为：
```c
half_dt = 0.5 * dt;
q0_new = q0 - half_dt*(ωx*q1 + ωy*q2 + ωz*q3)
q1_new = q1 + half_dt*(ωx*q0 + ωz*q2 - ωy*q3)
q2_new = q2 + half_dt*(ωy*q0 - ωz*q1 + ωx*q3)
q3_new = q3 + half_dt*(ωz*q0 + ωy*q1 - ωx*q2)
```

这是状态转移矩阵 **F** 的核心。

### 3.4 四元数 → 欧拉角（本项目实现）

```c
Yaw   = atan2( 2*(q0*q3 + q1*q2),   2*(q0² + q1²) - 1 ) * 180/π
Pitch = atan2( 2*(q0*q1 + q2*q3),   2*(q0² + q3²) - 1 ) * 180/π
Roll  = asin(   -2*(q1*q3 - q0*q2) ) * 180/π
```

**为什么用 atan2 而不是 asin 求 Pitch？**
atan2 可以区分四个象限，给出完整的 [-180°, 180°] 范围。asin 只能返回 [-90°, 90°]。对于大角度俯仰运动，atan2 更有优势。

### 3.5 Yaw 角度解缠绕

由于 yaw 来自于 atan2，其输出在 [-180°, 180°] 之间跳变。对于需要连续旋转的场合（如云台自动搜索模式），必须"解缠绕"（unwrapping）：

```c
// YawTotalAngle：累积的绝对角度
if (Yaw - YawAngleLast > 180)  YawRoundCount--;      // 正向穿越 ±180°
else if (Yaw - YawAngleLast < -180) YawRoundCount++;  // 反向穿越
YawTotalAngle = 360.0f * YawRoundCount + Yaw;         // 累积角度
```

这使得 `YawTotalAngle` 可以在 ±∞ 范围内连续变化，没有 ±180° 界限。

---

## 4. 传感器：BMI088 IMU

![Bosch BMI088 IMU](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)
*▲ Bosch BMI088 — 6 轴惯性测量单元 (陀螺仪 + 加速度计)*

### 4.1 BMI088 简介

BMI088 是 Bosch 的一款高性能 6 轴惯性测量单元（IMU）：
- **加速度计**：±6g 量程，16 位精度
- **陀螺仪**：±2000°/s 量程，16 位精度
- **接口**：SPI（加速度计和陀螺仪独立片选）
- **输出速率**：加速度计 800 Hz，陀螺仪 1000 Hz

### 4.2 数据读取流程

```c
// INS 任务中每个周期（约 1000 Hz）：
Module_BMI088_get_accel();   // SPI 读取加速度计
Module_BMI088_get_gyro();    // SPI 读取陀螺仪

// 校准后的值
acc[i]  = (raw_acc  - acc_offset)  * acc_scale;   // m/s²
gyro[i] = (raw_gyro - gyro_offset) * gyro_scale;  // dps（度/秒）
```

### 4.3 IMU 安装误差校正

实际安装时，IMU 芯片可能与机体坐标系不完全对齐。`IMU_Param_Correction()` 函数通过一个可配置的旋转矩阵补偿这个偏差：

```c
// 校正矩阵 = ScaleFactor · R(安装偏角)
corrected_gyro = R_correction · (scale · raw_gyro)
corrected_acc  = R_correction · (scale · raw_acc)
```

R_correction 用 Yaw/Pitch/Roll 三个校正角（ZYX 欧拉角）构造。默认校正角为 [0, 0, 0]，即不做校正。

---

## 5. 姿态解算方法演进

![Tait-Bryan 角 Z-Y-X 序列](https://upload.wikimedia.org/wikipedia/commons/5/53/Taitbrianzyx.svg)
*▲ Tait-Bryan 角度 (Z-Y-X) 内旋序列 — Yaw→Pitch→Roll (图片来源: Wikipedia)*

### 5.1 纯陀螺仪积分

```c
// 最简单的姿态更新
q_new = q_old ⊗ [1, ½·ωx·dt, ½·ωy·dt, ½·ωz·dt]
```

**缺点**：陀螺仪有零偏（bias），积分会随时间累积误差。1°/s 的零偏意味着每秒累积 1° 的漂移，1 分钟后就是 60°。

### 5.2 互补滤波（Complementary Filter）

```c
// Mahony 互补滤波 (开源飞控常用)
q = q ⊗ [1, ½·ωx·dt, ½·ωy·dt, ½·ωz·dt]           // 陀螺仪预测
error = accel_measured × accel_predicted(q)          // 加速度计误差
q = q ⊗ correction(error, Kp, Ki)                    // PI 校正
```

**优点**：简单、计算快
**缺点**：没有最优性保证，大动态运动下加速度计修正不可靠

### 5.3 扩展卡尔曼滤波（EKF）

EKF 将姿态估计建模为**最优状态估计问题**：
- 状态：四元数 + 陀螺仪零偏
- 预测：陀螺仪积分
- 更新：加速度计观测重力方向
- 优势：自动加权（Kalman 增益动态调整信任度）

本项目使用 EKF 方法，并且加入了**自适应卡方检验**来判断加速度计数据的可靠性。

---

## 6. QuaternionEKF：本项目的核心算法

![Kalman 滤波基本概念](https://wiki.stg.apps.ctlt.ubc.ca/images/e/e9/Kalman-filter-basic-concept.png)
*▲ Kalman 滤波器的 Predict-Update 循环 — 先预测状态，再用测量修正 (图片来源: UBC Wiki)*

### 6.1 状态向量

```
x = [q0, q1, q2, q3,  gyro_bias_x,  gyro_bias_y]^T
     ←── 四元数 ──→   ←── 陀螺仪零偏 ──→
```

共 **6 个状态**。

**重要设计选择**：
- `gyro_bias_z` 不在状态中 —— 被强制为 0
- **原因**：没有磁力计时，yaw 轴绝对朝向不可观测（重力矢量只能提供 roll/pitch 信息）
- 这意味着 yaw 角完全依赖陀螺仪积分，会随时间缓慢漂移

### 6.2 预测步骤（时间更新）

**状态转移矩阵 F（6×6）**：

```
F = [F_quat(4×4)    F_bias(4×2)]
    [0_(2×4)        I_(2×2)    ]
```

**F_quat**（四元数运动学线性化）：
```c
half_gx_dt = 0.5 * (gyro_x - bias_x) * dt
half_gy_dt = 0.5 * (gyro_y - bias_y) * dt
half_gz_dt = 0.5 * (gyro_z - 0) * dt       // z 偏置固定为 0

F[0][0]=1   F[0][1]=-hxdt   F[0][2]=-hydt   F[0][3]=-hzdt
F[1][0]=hxdt F[1][1]=1     F[1][2]=hzdt    F[1][3]=-hydt
F[2][0]=hydt F[2][1]=-hzdt  F[2][2]=1      F[2][3]=hxdt
F[3][0]=hzdt F[3][1]=hydt   F[3][2]=-hxdt  F[3][3]=1
```

**F_bias**（陀螺仪零偏对四元数的影响）：
```c
F[0][4] =  q1*dt/2    F[0][5] =  q2*dt/2    // ∂q/∂bias_x, ∂q/∂bias_y
F[1][4] = -q0*dt/2    F[1][5] =  q3*dt/2
F[2][4] = -q3*dt/2    F[2][5] = -q0*dt/2
F[3][4] =  q2*dt/2    F[3][5] = -q1*dt/2
```

**过程噪声 Q**：
```c
Q = diag(Q1*dt, Q1*dt, Q1*dt, Q1*dt, Q2*dt, Q2*dt)
// Q1 = 10.0     — quaternion 过程噪声
// Q2 = 0.001    — gyro bias 随机游走噪声
```

Q1 较大（10.0）说明系统预期运动是比较剧烈的。Q2 很小（0.001）说明预期陀螺仪零偏变化非常缓慢。

### 6.3 更新步骤（量测更新）

**量测模型**：加速度计归一化后 = 重力方向在机体坐标系中的投影

```
z = acc_measured / |acc_measured|      ← 实际测量（3维）
h(x) = predicted_gravity(q)            ← 从四元数预测的重力方向（3维）
```

**预测的重力方向**（从四元数计算）：
```c
h[0] = 2*(q1*q3 - q0*q2)       // 预测的 x 轴重力分量
h[1] = 2*(q0*q1 + q2*q3)       // 预测的 y 轴重力分量
h[2] = q0² - q1² - q2² + q3²   // 预测的 z 轴重力分量
```

这实际上是 `[0,0,1]`（导航系中的重力方向，即 Z 轴）经过 R(q) 的逆变换（转到机体系）。

**量测雅可比 H（3×6）**：
```c
H[0][0]=-2*q2  H[0][1]= 2*q3  H[0][2]=-2*q0  H[0][3]= 2*q1  H[0][4..5]=0
H[1][0]= 2*q1  H[1][1]= 2*q0  H[1][2]= 2*q3  H[1][3]= 2*q2  H[1][4..5]=0
H[2][0]= 2*q0  H[2][1]=-2*q1  H[2][2]=-2*q2  H[2][3]= 2*q3  H[2][4..5]=0
```

**量测噪声 R**：
```c
R = diag(1e6, 1e6, 1e6)   // R = 1,000,000
```

**R = 1,000,000 意味着什么？**

这是一个极大的值，表示 EKF **极不信任加速度计**。这看似荒谬，但实际上是一种**防御性设计**：

- 在理想的静态条件下，加速度计确实是完美的重力传感器
- 但在机器人实际运动中，加速度计测量的 = 重力 + 运动加速度 + 振动
- 运动加速度和振动是"噪声"（就姿态估计而言），而它们远大于传感器的电子噪声
- R=1e6 告诉滤波器："除非有非常充分的证据，否则不要相信加速度计的修正"

这个信任度由卡方检验动态调整。

### 6.4 自适应卡方检验（Chi-Square Gating）

这是本实现最有特色的部分——**根据统计检验动态决定是否接受加速度计量测**。

**步骤 1：计算归一化新息平方（NIS）**

```c
// 新息（Innovation）：测量值与预测值之差
r = z - h(x_pred)

// 新息协方差
S = H · P_pred · H^T + R

// 马氏距离（Mahalanobis Distance）
chi2 = r^T · S^(-1) · r
```

`chi2` 衡量的是"实际的新息有多不符合预期"。理论上，如果滤波器工作正常，`chi2` 应服从自由度为 3 的卡方分布。

**步骤 2：与阈值比较**

```c
threshold = 1e-8    // 卡方检验阈值

if (chi2 < 0.5 * threshold):
    // 滤波器已收敛，数据可信
    converged = true
    apply_measurement_update()

elif (chi2 > threshold AND converged):
    // 可能发生了干扰（运动加速度过大）
    if (stable_flag):
        error_count++
        if (error_count > 50):
            // 连续 50 次异常 → 滤波器可能发散了，强制更新
            reset_and_update()
    else:
        error_count = 0
    // 跳过本次量测更新（仅预测）
    skip_measurement_update()

else:
    // 正常情况：应用增益缩放
    AdaptiveGainScale = (threshold - chi2) / (0.9 * threshold)
    apply_scaled_update()
```

**步骤 3：自适应增益缩放**

当 `chi2` 在可接受范围内但偏大时，按比例缩小 Kalman 增益：

```
K_scaled = K * AdaptiveGainScale
```

这实现了"软切换"——不完全拒绝也不完全接受，而是根据数据质量动态调整。

### 6.5 静止检测

系统通过运动检测来判断当前是否适合使用加速度计修正：

```c
stable = (|gyro|      < 0.3 rad/s)    // 角速度小
      && (| |acc|-g | < 0.2 m/s²)     // 加速度幅值接近重力
      && (horiz_acc   < 0.15 m/s²)    // 水平加速度小
```

静止时，加速度计确实在测量重力，滤波器可以在此时更积极地使用加速度计数据校正姿态。

### 6.6 陀螺仪零偏估计

零偏是状态的一部分，在 EKF 的更新步骤中与四元数同时修正：

```c
// 零偏更新受方向余弦衰减
OrientationCosine = abs(h[0]) 或 abs(h[1])  // 该轴与重力方向的对齐度

// 仅在与重力方向有对齐全的轴上更新零偏
K_bias *= OrientationCosine / (π/2)
```

这个设计的直觉是：当某个陀螺仪轴与重力方向平行时（如 X 轴指向天顶），该轴的零偏不可观测（因为重力不会在该轴上产生约束），因此减少该轴的零偏更新量。

**零偏约束**：
- 每次更新的最大变化限制为 `1e-2 * dt`（防止突变）
- Z 轴零偏**强制为 0**（yaw 不可观测）
- 零偏方差 P[4][4] 和 P[5][5] 每周期除以 fading factor `λ = 0.9996`（防止过度收敛）

### 6.7 通用卡尔曼滤波框架

本项目建立了一个通用的**线性卡尔曼滤波框架**（`kalman_filter.c`），包含 5 个标准方程：

```
eq1:  x̂⁻   = F · x̂             （状态预测）
eq2:  P⁻   = F·P·F^T + Q       （协方差预测）
eq3:  K    = P⁻·H^T/(H·P⁻·H^T+R)（Kalman 增益计算）
eq4:  x̂    = x̂⁻ + K·(z - H·x̂⁻)  （状态更新）
eq5:  P    = (I - K·H)·P⁻       （协方差更新）
```

QuaternionEKF 通过**用户函数钩子**（`User_Func`）替换标准方程中的步骤：
- eq3 和 eq4 被替换为包含卡方检验的自定义逻辑
- 其他方程使用标准 KF 公式

**协方差 P 的 fading 机制**：

```c
P[4][4] /= 0.9996   // 每周期略增零偏的不确定性
P[5][5] /= 0.9996
// 限制 P[4][4], P[5][5] ≤ 10000
```

这防止了零偏估计"睡觉"——如果不做 fading，P 会收敛到极小值，导致 Kalman 增益趋近于零，零偏永远不再更新。

### 6.8 QuaternionEKF 完整执行流程

```
每个周期（约 1 ms）：

1. 读取 BMI088 数据（gyro + acc）
   ├── 应用校准值（零偏、比例因子）
   └── IMU_Param_Correction() 安装误差校正

2. 减去陀螺仪零偏（来自状态）
   gx = gyro_x - bias_x
   gy = gyro_y - bias_y
   gz = gyro_z - 0          // Z 偏置固定为 0

3. 构造 F 矩阵（基于当前 gyro 和 dt）
   └── IMU_QuaternionEKF_F_Linearization_P_Fading()

4. eq1: 状态预测 x⁻ = F · x

5. eq2: 协方差预测 P⁻ = F · P · F^T + Q

6. 计算量测向量 z = acc / |acc|

7. 构造 H 矩阵（量测雅可比）
   └── IMU_QuaternionEKF_H_Linearization()

8. 卡方检验 → 决定是否跳过量测更新

9. 如果接受更新：
   ├── eq3: 计算 Kalman 增益 K（带自适应缩放）
   ├── eq4: 状态更新 x = x + K*(z - h(x))
   ├── eq5: 协方差更新 P = (I - KH)*P
   └── 强制归一化四元数

10. 更新欧拉角、体轴向量
    ├── QuaternionToEuler() → Yaw/Pitch/Roll
    ├── YawTotalAngle 解缠绕
    ├── BodyFrameToEarthFrame([1,0,0]) → xn
    ├── BodyFrameToEarthFrame([0,1,0]) → yn
    └── BodyFrameToEarthFrame([0,0,1]) → zn
```

---

## 7. 传感器标定

### 7.1 上电自动标定

系统上电后自动进行陀螺仪和加速度计的简易标定：

**陀螺仪零偏标定**：
```c
// 静止状态下采集 6000 个样本
for (i = 0; i < 6000; i++) {
    sum_gyro += gyro_raw;
}
gyro_offset = sum_gyro / 6000;   // 平均值即为零偏
```

**加速度计比例因子标定**：
```c
// 同样静止状态下采集 6000 个样本
avg_acc_magnitude = mean(|acc|) / 6000;

// 以 9.81 m/s² 为标准进行标定
acc_scale = 9.81 / avg_acc_magnitude;
```

**有效性检查**：
- `|avg_acc - 9.81| < 0.5`：加速度计幅值接近重力，说明静止
- `max_gyro - min_gyro < 0.01`：陀螺仪波动小，说明静止
- 标定结果存入 Flash，带魔数头 0xAA 防误读

### 7.2 安装误差校正（在线）

```c
// 在 module_ins.c 的初始化中可配置
ins_param.angle[0] = 0.0f;  // 安装 Yaw 校正角
ins_param.angle[1] = 0.0f;  // 安装 Pitch 校正角
ins_param.angle[2] = 0.0f;  // 安装 Roll 校正角
```

如果需要校正安装偏角，修改这三个值（单位：度）即可。`IMU_Param_Correction()` 自动将校正矩阵作用于 gyro 和 acc 数据。

---

## 8. 运动加速度分离

### 8.1 问题

加速度计测量的是**总加速度** = 重力 + 运动加速度 + 振动。

云台控制需要纯运动加速度来做前馈补偿（如预测负载扰动）。姿态估计需要纯重力来做姿态修正。两者都是需要的，但混在一起。

### 8.2 分离方法

```c
// 1. 从四元数获取重力方向在机体系中的投影
EarthFrameToBodyFrame([0, 0, 9.81], gravity_bframe, q);

// 2. 加速度计读数减去重力分量
MotionAccel_b[i] = acc_measured[i] - gravity_bframe[i];

// 3. 一阶低通滤波（滤除高频振动）
MotionAccel_b[i] = MotionAccel_b[i] * LPF/(LPF+dt) 
                 + acc_diff[i] * dt/(LPF+dt);
// LPF = 0.0085 (低通滤波时间常数)

// 4. 转回导航系
BodyFrameToEarthFrame(MotionAccel_b, MotionAccel_n, q);
```

**LPF = 0.0085 的含义**：这是一个非常小的值，对应约 8.5 ms 的时间常数。它让运动加速度的估计"跟上"真实值需要大约 8.5 ms，同时有效抑制高频振动。

### 8.3 使用场景

- `MotionAccel_n`（导航系）：可用于底盘的加速度前馈控制
- `MotionAccel_b`（机体系）：可用于检测碰撞或急停

---

## 9. 数据流全链路

### 9.1 完整时序

```
硬件:
  BMI088 通过 SPI 产生数据
    加速度计 @ 800 Hz
    陀螺仪   @ 1000 Hz

ThreadX 任务:
  INS Task (优先级 7, 1024 B 栈)
    while(1) {
        // 1. 测量 dt
        dt = BSP_DWT_GetDeltaT(&last_time);  // 通常约 0.001 s
        
        // 2. 读取传感器
        BMI088_get_accel();    // SPI 读取
        BMI088_get_gyro();     // SPI 读取
        
        // 3. 校正
        IMU_Param_Correction();
        
        // 4. EKF 更新
        QuaternionEKF_Update(gx, gy, gz, ax, ay, az, dt);
        
        // 5. 写入全局 Ins_t 结构体
        ins->q[0..3] = QEKF_INS.q[0..3];
        ins->euler_rad[0..2] = ...;
        
        tx_thread_sleep(1);  // 约 1ms 休眠 → 约 1000 Hz
    }

  Robot Control Task (优先级 30, 1024 B 栈)
    while(1) {
        // 读取最新姿态
        pitch = ins->euler_rad[1];     // 用于 pitch 控制
        yaw   = ins->YawTotalAngle_rad; // 用于 yaw 控制
        
        // 发送给视觉
        vision_send.q[0..3] = ins->q[0..3];  // 让视觉做坐标变换
        
        // 控制计算
        gimbal_func();   // 用姿态做反馈
        shoot_func();    // 射击控制
        boardcomm_send(); // 发给底盘
        
        tx_thread_sleep(2);  // 2ms → 500 Hz
    }
```

### 9.2 频率设计分析

```
INS 任务:   ~1000 Hz (传感器数据率上限)
控制任务:    500 Hz  (控制带宽需求)
```

INS 以比控制高一倍的频率运行，确保控制循环每次读取的都是"最新且已更新"的姿态数据，而不是上一周期的旧数据。

### 9.3 数据共享（无锁）

```c
// INS 任务写入
const static Ins_t *ins = NULL;    // 全局指针（初始化时赋值）

// 控制任务读取
yaw = ins->YawTotalAngle_rad;      // 直接读取

// 视觉发送
send_packet.q[0] = ins->q[0];      // 直接读取四元数
```

这里没有用互斥量保护 `Ins_t` 结构体——因为它是 POD 类型，单次 float 读取在 Cortex-M7 上是原子的（对齐访问）。

---

## 10. 控制回路中的姿态使用

### 10.1 小 Yaw（精确瞄准）

```c
// 角度反馈来源：INS 累积 Yaw 角（解缠绕）
other_angle_feedback_ptr = &ins->YawTotalAngle_rad;

// 速度反馈来源：BMI088 陀螺仪 Z 轴
other_speed_feedback_ptr  = &bmi088_dev->gyro[2];  // dps → 内部转 rad/s

// LQR 控制
torque = -K1*(YawTotalAngle - ref) - K2*gyro_z;
// K = [5.47, 0.56]
```

### 10.2 Pitch（俯仰控制）

```c
// 角度反馈来源：INS 欧拉角 Pitch
other_angle_feedback_ptr = &ins->euler_rad[1];

// 速度反馈来源：BMI088 陀螺仪 X 轴
other_speed_feedback_ptr  = &bmi088_dev->gyro[0];

// note: pitch 有 feedback_reverse_flag = 1（取反）
// LQR 控制
torque = -K1*(euler_pitch - ref) - K2*gyro_x;
// K = [38.73, 2.85]
```

### 10.3 大 Yaw（粗跟踪）

```c
// 角度反馈来源：WT606 外置航向模块
other_angle_feedback_ptr = &wt606_device->YawTotalAngle_rad;

// 速度反馈来源：WT606 陀螺仪 Z 轴
other_speed_feedback_ptr  = &wt606_device->gyro_rps[2];

// LQR 控制
torque = -K1*(wt606_yaw - ref) - K2*wt606_gyro_z;
// K = [22.36, 3.15]
```

### 10.4 视觉目标跟踪时的坐标变换

```
视觉计算机:
  1. 接收四元数 q[0..3]（本机姿态）
  2. 检测目标在图像中的位置
  3. 用四元数将目标坐标从相机坐标系转到世界坐标系
  4. 计算目标相对于云台的 yaw/pitch 角度
  5. 发送 target_yaw, target_pitch 回云台

云台:
  gimbal_cmd->yaw = target_yaw
  gimbal_cmd->pitch = target_pitch
  → LQR 追踪
```

---

## 11. 调参指南

### 11.1 关键参数速查表

| 参数 | 位置 | 默认值 | 作用 |
|------|------|--------|------|
| Q1 | QuaternionEKF | 10.0 | 过程噪声（四元数） |
| Q2 | QuaternionEKF | 0.001 | 过程噪声（陀螺仪零偏） |
| R | QuaternionEKF | 1,000,000 | 量测噪声（加速度计） |
| λ (lambda) | QuaternionEKF | 0.9996 | P 矩阵 fading factor |
| Chi2Threshold | QuaternionEKF | 1e-8 | 卡方检验阈值 |
| AccelLPF | module_ins.c | 0.0085 | 运动加速度低通 |
| 稳定角速度阈值 | QuaternionEKF | 0.3 rad/s | 判断"静止" |
| 稳定加速度阈值 | QuaternionEKF | 0.2 m/s² | 判断"静止" |
| 稳定水平加速度 | QuaternionEKF | 0.15 m/s² | 判断"静止" |

### 11.2 常见调参场景

**问题：姿态漂移太快（yaw 几分钟就漂了 10°）**

→ 陀螺仪零偏估计不工作。检查：
1. 零偏 fading factor λ 是否太小（Q2 太小会导致零偏不更新）→ 增大 Q2
2. Z 轴零偏被强制设为 0，这是正确的（yaw 没有绝对参考，漂移是正常的）
3. 考虑是否需要外部航向参考（磁力计、WT606 等）

**问题：快速运动时姿态估计很差（欧拉角乱跳）**

→ 加速度计在动态下错误参与修正。检查：
1. Chi2Threshold 是否太大（1e-8 → 试 1e-7）
2. R 是否太小（1e6 → 试 5e6 或 1e7）
3. 稳定检测阈值是否太宽松

**问题：roll/pitch 估计有明显延迟**

→ EKF 响应太慢。检查：
1. Q1 是否太小（10 → 试 50 或 100）
2. INS 任务是否在以 ~1000 Hz 运行（检查 dt 的值）

**问题：静止时 roll/pitch 有 ±2° 的波动**

→ 加速度计噪声过大或者零偏估计不准。检查：
1. 陀螺仪标定数据是否有效
2. R 是否太大（1e6 → 试 5e5，在静态下给加速度计更多信任）
3. 振动是否过大（检查机械安装）

**问题：卡方检验过多跳过量测更新（filter 不收敛）**

→ Chi2Threshold 设置过严。检查：
1. 增大 Chi2Threshold（1e-8 → 1e-6）
2. 增大 R（降低对加速度计的期望精度）

### 11.3 调试方法

在 EKF 更新函数中添加日志输出：

```c
// 监控卡方检验
LOG_D("chi2=%.6f threshold=%.6f converged=%d", chi2, threshold, converged);

// 监控欧拉角
LOG_D("YPR: %.2f, %.2f, %.2f", yaw, pitch, roll);

// 监控陀螺仪零偏
LOG_D("bias: %.6f, %.6f", bias_x, bias_y);

// 监控四元数（验证归一化）
float norm = q0*q0 + q1*q1 + q2*q2 + q3*q3;
LOG_D("q_norm=%.4f", norm);  // 应该接近 1.0
```

---

## 12. 故障排查

### Q1: 上电后 roll/pitch 立即大幅偏差

检查：
1. 加速度计标定是否成功（Flash 中是否有有效标定数据）
2. 初始四元数计算：`InitQuaternion()` 是否正确读取了重力方向
3. BMI088 数据是否正常工作（SPI 通信是否正常，可用 WHO_AM_I 寄存器验证）

### Q2: pitch 角快速运动时失控

这是**万向节死锁附近的表现**。哨兵正常工作时 pitch 不会接近 90°，但如果发生：
- 检查机械限位是否失效
- 检查 pitch 零点是否设置正确（`PITCH_HORIZON_ANGLE`）

### Q3: 振动环境下姿态抖动严重

1. 增大 AccelLPF（0.0085 → 0.02）—— 更强的低通滤波
2. 增大 R（1e6 → 5e6）—— 更不信任加速度计
3. 检查机械减震措施（IMU 应安装在减震结构上）

### Q4: yaw 角持续漂移

**这是正常的**——没有磁力计时，yaw 绝对朝向不可观测。解决方法：
- 在哨兵上这不是严重问题，因为 yaw 只需相对跟踪目标
- 底盘通过 WT606 外置模块获取绝对航向
- 如果必须抑制 yaw 漂移，需添加磁力计或视觉 odometry

### Q5: EKF 的 dt 值不稳定

dt 通过 DWT 周期计数器测量。如果 dt 波动大（如 ±50%）：
- 检查是否有更高优先级的 ISR 长时间占用 CPU
- 检查 INS 任务优先级（7）是否被更低优先级任务干扰（ThreadX 会抢占，但检查是否有临界区过长）
