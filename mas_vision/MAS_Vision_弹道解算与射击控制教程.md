# 弹道解算与射击控制教程 —— 扣下扳机

> 📘 **专题深入**：多线程架构、SPSCQueue 无锁队列、线程间数据流详见 [[MAS_Vision_多线程架构学习指南]]（已从本文独立为专题）
> 📘 **上游依赖**：目标状态由 [[MAS_Vision_位姿解算与目标跟踪教程]] 的 EKF 提供
> 📘 **下游输出**：yaw/pitch/fire_advice 经 [[MAS_Vision_多线程架构学习指南|Armor 线程]] → Serial 线程 → STM32

## 目录

1. [弹道模型](#1-弹道模型)
2. [弹道迭代收敛](#2-弹道迭代收敛)
3. [三种瞄准模式](#3-三种瞄准模式)
4. [射击决策](#4-射击决策)
5. [配置参数速查](#5-配置参数速查)

---

## 1. 弹道模型

### 1.1 物理模型：无阻力弹道

本项目使用**简化的无空气阻力弹道模型**。对于 17mm 塑料弹丸（质量轻、初速 28 m/s），在 0–8 米的有效射程内，空气阻力影响较小（< 0.5° 的角度偏差），简化模型足够。

```
物理设定:
  v₀ = 28 m/s    子弹初速度
  g  = 9.7833    重力加速度（武汉/深圳地区值，非标准 9.81）
  d  = 水平距离    (m)
  h  = 竖直高度差  (m)  正 = 目标比枪口高
  
未知量:
  θ  = 发射俯仰角 (待求解)
  t  = 飞行时间   (待求解)
```

### 1.2 运动方程

```
水平方向（匀速）:  d = v₀·cos(θ)·t
竖直方向（匀加速）: h = v₀·sin(θ)·t - ½g·t²
```

### 1.3 求解过程

从水平方程解出 `t = d/(v₀·cos(θ))`，代入竖直方程：

```
h = d·tan(θ) - g·d²/(2·v₀²·cos²(θ))

利用恒等式 1/cos²(θ) = 1 + tan²(θ):

h = d·tan(θ) - g·d²·(1+tan²(θ))/(2·v₀²)

令 T = tan(θ), 整理为关于 T 的二次方程:

(g·d²)/(2·v₀²) · T² - d·T + (g·d²)/(2·v₀²) + h = 0
```

**系数识别**：
```
a = g·d²/(2·v₀²)
b = -d
c = a + h

判别式: Δ = b² - 4ac = d² - 4a·(a+h)

如果 Δ < 0: 目标不可达（太远）
如果 Δ ≥ 0: T = (-b ± √Δ)/(2a)  →  θ = atan(T)
```

### 1.4 弹道选择

当有两个解时，选择**低抛弹道**（飞行时间更短的那个）：

```
T₁ → θ₁ = atan(T₁) → t₁ = d/(v₀·cos(θ₁))
T₂ → θ₂ = atan(T₂) → t₂ = d/(v₀·cos(θ₂))

选择 min(t₁, t₂)
```

**为什么选飞行时间短的？**
- 子弹到达更快 → 目标运动更少 → 预测更准
- 低抛弹道的弹道更平坦 → 对测距误差不敏感

### 1.5 迭代收敛

子弹在飞行期间目标会继续运动。简单的"打当前点"不够——需要在目标未来位置和飞行时间之间迭代：

```cpp
// 弹道迭代（最多 10 次）
double fly_time = 0.001;  // 初始猜测
for (int iter = 0; iter < 10; iter++) {
    // 1. 预测目标在 fly_time 后的位置
    TargetState future = target.predict(timestamp + fly_time);
    
    // 2. 计算到未来位置的弹道
    Trajectory traj(future.position, bullet_speed);
    
    // 3. 收敛检查
    if (abs(traj.fly_time - fly_time) < 0.001) break;
    
    fly_time = traj.fly_time;
}
```

### 1.6 完整输出

```cpp
struct Trajectory {
    double pitch;      // 发射俯仰角 (rad)
    double fly_time;   // 飞行时间 (s)
    bool unsolvable;   // 目标不可达
};
```

---

## 2. 弹道迭代收敛

### 2.1 为什么需要迭代

弹道求解需要知道**目标位置**来计算飞行时间，但飞行时间内目标会运动到**新位置**。这是一个循环依赖：

```
目标位置 ──→ 弹道求解 ──→ 飞行时间
    ↑                        │
    └──────── 预测 ──────────┘
```

需要迭代打破这个循环，让飞行时间与目标未来位置自洽。

### 2.2 不动点迭代原理

令 `t_f` 为子弹飞行时间，`f(t_f)` 为「预测目标在 `t_f` 后位置 → 弹道求解 → 新飞行时间」的复合函数。我们求解的是不动点方程：

```
t_f = f(t_f)
```

**什么条件下迭代收敛？**

`f` 的 Lipschitz 常数约等于 `|∂f/∂t_f| ≈ |∂(弹道时间)/∂(目标位置) · ∂(目标位置)/∂t_f|`

- `|∂(目标位置)/∂t_f| = v_target`（目标速度，≤ 5 m/s）
- `|∂(弹道时间)/∂(目标位置)| = 1/v_bullet`（约 1/25 s/m）

因此 `|∂f/∂t_f| ≈ 5/25 = 0.2 < 1` → **Banach 不动点定理保证线性收敛**，每轮误差缩小约 5 倍。

### 2.3 源码实现

```cpp
// 弹道迭代（最多 10 次，实践 3-5 次即收敛）
double fly_time = 0.001;  // 初始猜测：子弹几乎瞬间到达
for (int iter = 0; iter < 10; iter++) {
    // 1. 预测目标在 fly_time 后的位置
    TargetState future = target.predict(timestamp + fly_time);
    //    predict() 内部: x = x0 + vx * fly_time (恒速模型)

    // 2. 计算到未来位置的弹道
    Trajectory traj(future.position, bullet_speed);

    // 3. 收敛检查
    if (abs(traj.fly_time - fly_time) < 0.001) break;  // < 1ms 收敛

    fly_time = traj.fly_time;
}
```

### 2.4 收敛性分析

| 目标距离 | 典型飞行时间 | 典型迭代次数 | 原因 |
|----------|-------------|-------------|------|
| 1m | ~40ms | 1-2 | 飞行时间极短，目标位移可忽略 |
| 3m | ~120ms | 2-3 | 目标运动引入修正 |
| 5m | ~200ms | 3-4 | 需要更精确的预测 |
| 8m | ~320ms | 4-5 | 长飞行时间，运动预测关键 |

### 2.5 为什么初始猜测是 0.001 而非 0

除以零保护——`predict(timestamp + 0)` 与弹道求解之间如果 `fly_time = 0`，会导致除零。0.001s 足够小而不影响收敛方向。

### 2.6 不收敛的情况

```cpp
// 10 次迭代还不收敛 → 强制使用最后一次结果
// 可能原因：
// 1. 目标速度接近子弹速度（极端情况）
// 2. 目标在视野边缘，预测可能越界
// 3. 数值震荡（极罕见，因为 |∂f/∂t_f| ≈ 0.2 远小于 1）
```

---

## 3. 三种瞄准模式

### 2.1 模式选择逻辑

```cpp
double omega = abs(x[7]);  // 目标角速度

if (omega >= spinning_threshold_high_)  // ≥ 10 rad/s
    mode = HERO_CENTER;                   // 打中心
else if (omega <= spinning_threshold_low_)  // ≤ 2 rad/s
    mode = TRACK;                            // 跟踪单板
else  // 2 < omega < 10
    mode = COMING;                           // 来板预判

// 英雄模式强制打中心
if (hero_mode_ && omega >= spinning_threshold_low_)
    mode = HERO_CENTER;
```

### 2.2 TRACK 模式（低速，`omega ≤ 2 rad/s`）

**策略**：锁定并跟踪当前可见的装甲板。

```
适用场景: 静止或慢速旋转的目标（步兵正常运动）
瞄准点:   当前可见装甲板的几何中心
开火条件: 始终允许（只要瞄准偏差在容差内）
```

**装甲板锁定**：通过 `lock_id_` 保持锁定连续性——即使多块装甲板可见，也只跟踪之前锁定的那块，避免频繁切换瞄准点。

### 2.3 COMING 模式（中速，`2 < omega < 10 rad/s`）

**策略**：选择正在"靠近"射手的装甲板进行预判射击。

**判断逻辑**：
```
围绕旋转中心旋转的装甲板:
  → 计算每块装甲板的线速度方向
  → 当装甲板与射手方向的角度在减小 → "来板"（正在靠近）
  → 当角度在增大 → "去板"（正在远离）

选择来板 → 预判其到达最近点的时间 → 提前发射
```

**参数**：
- `comming_angle = 60°`：来板判定角度阈值
- `leaving_angle = 30°`：去板判定角度阈值
- 前哨站专用：来板 70°，去板 30°

**为什么打来板？**
来板正在靠近射手方向，子弹到达时它正好在最佳射击位置。去打板则"追不上"。

### 2.4 HERO_CENTER 模式（高速，`omega ≥ 10 rad/s`）

**策略**：直接瞄准旋转中心，持续发射。

```
适用场景: "小陀螺"——高速旋转的机器人
瞄准点:   旋转中心 (x[0], x[2], x[4])

为什么打中心:
  - 高速旋转时，单块装甲板只在视野中停留极短时间
  - 无法在装甲板可见的时间内完成瞄准 + 发射
  - 改为瞄准旋转中心，持续发射
  - 子弹在某个时刻会命中经过该位置的装甲板

开火条件:
  需要同时满足：
    1. 至少有一块装甲板的 |delta_angle| < fire_window_angle
       → "有装甲板正在飞向子弹路径"
    2. 近距离（<3m）: window = 60°, 远距离（≥3m）: window = 20°
       → 近距离窗口更大（角速度的影响更大）
```

### 2.5 瞄准模式总结

| 模式 | 触发 | 瞄准点 | 开火 | 典型目标 |
|------|------|--------|------|---------|
| TRACK | `ω ≤ 2` | 当前装甲板中心 | 始终 | 步兵正常运动 |
| COMING | `2 < ω < 10` | 来板预判位置 | 始终 | 步兵中速旋转 |
| HERO_CENTER | `ω ≥ 10` | 旋转中心 | 装甲板在窗口内 | 小陀螺 |

---

## 4. 射击决策

### 3.1 射击角度计算

```cpp
// Yaw 目标角度（带连续跟踪）
double target_yaw = atan2(target_y, target_x) + yaw_offset;
// 增量式更新：避免 yaw 在 ±180° 跳变时的不连续
target_yaw = last_target_yaw_ + normalize(target_yaw - last_target_yaw_);

// Pitch 目标角度
double target_pitch = -trajectory.pitch - pitch_offset;
// 负号：pitch 向上为正（枪口上抬），弹道求解的 pitch 是发射角
```

### 3.2 开火判定

```cpp
bool should_fire = false;

if (aim_point_valid && fire_allowed) {
    double yaw_err   = abs(gimbal_yaw   - target_yaw);
    double pitch_err = abs(gimbal_pitch - target_pitch);
    
    double yaw_tol   = (distance < 3.0) ? 1.0° : 2.0°;
    double pitch_tol = (distance < 3.0) ? 1.0° : 2.0°;
    
    should_fire = (yaw_err < yaw_tol) && (pitch_err < pitch_tol);
}
```

**容差设计**：
- 近距离（< 3m）：1° 容差 → 3 米处约 5 cm 偏差
- 远距离（≥ 3m）：2° 容差 → 5 米处约 17 cm 偏差 → 装甲板仍在散布圆内
- 容差不是越严越好——太严会错过射击窗口

### 3.3 配置参数速查

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `bullet_speed` | 25 m/s | 子弹初速 |
| `yaw_offset` | 0° | yaw 修正角 |
| `pitch_offset` | 0° | pitch 修正角 |
| `spinning_threshold_low` | 2 rad/s | 低速/中速分界 |
| `spinning_threshold_high` | 10 rad/s | 中速/高速分界 |
| `yaw_tolerance_near` | 1° | 近距离 yaw 容差 |
| `yaw_tolerance_far` | 2° | 远距离 yaw 容差 |
| `pitch_tolerance_near` | 1° | 近距离 pitch 容差 |
| `pitch_tolerance_far` | 2° | 远距离 pitch 容差 |
| `fire_delay_time` | 0.02 s | 发弹延迟补偿 |

---

## 5. 配置参数速查

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `bullet_speed` | 25 m/s | 子弹初速 |
| `yaw_offset` | 0° | yaw 修正角（补偿枪管安装偏差） |
| `pitch_offset` | 0° | pitch 修正角（补偿枪管安装偏差） |
| `spinning_threshold_low` | 2 rad/s | 低速/中速分界 |
| `spinning_threshold_high` | 10 rad/s | 中速/高速分界 |
| `yaw_tolerance_near` | 1° | 近距离 yaw 容差 |
| `yaw_tolerance_far` | 2° | 远距离 yaw 容差 |
| `pitch_tolerance_near` | 1° | 近距离 pitch 容差 |
| `pitch_tolerance_far` | 2° | 远距离 pitch 容差 |
| `fire_delay_time` | 0.02 s | 发弹延迟补偿 |

> 📘 **相关笔记**：[[MAS_Vision_多线程架构学习指南]] · [[MAS_Vision_位姿解算与目标跟踪教程]] · [[MAS_Vision_系统集成标定与调试教程]]
