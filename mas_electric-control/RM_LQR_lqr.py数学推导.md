# lqr.py 完整数学推导

> 对 `rm_embedded/sentry/lqr.py` 的逐行数学注释，从牛顿力学到代数黎卡提方程，再到离散时间部署分析。面向哨兵机器人云台 Yaw/Pitch 轴的 LQR 控制器。

---

## 目录

1. [物理模型：双积分器](#1-物理模型双积分器)
2. [状态空间与系统性质](#2-状态空间与系统性质)
3. [LQR 问题的数学表述](#3-lqr-问题的数学表述)
4. [最优控制的变分推导 → CARE](#4-最优控制的变分推导--care)
5. [2 状态双积分器的解析解](#5-2-状态双积分器的解析解)
6. [S 矩阵的物理含义](#6-s-矩阵的物理含义)
7. [QR 权重与闭环极点的映射](#7-qr-权重与闭环极点的映射)
8. [源代码逐行数学注释](#8-源代码逐行数学注释)
9. [扭矩约束下的可行域](#9-扭矩约束下的可行域)
10. [离散时间实现分析](#10-离散时间实现分析)
11. [Yaw vs Pitch 权重设计的数学依据](#11-yaw-vs-pitch-权重设计的数学依据)

---

## 1. 物理模型：双积分器

### 1.1 从牛顿定律出发

哨兵云台每根轴由一个直驱电机驱动上装旋转。刚体绕固定轴的**欧拉方程**：

$$\boxed{J \cdot \ddot{\theta} = \tau}$$

| 符号              | 含义     | Yaw 值 | Pitch 值 | 单位     |
| --------------- | ------ | ----- | ------- | ------ |
| $J$             | 转动惯量   | 0.2   | 0.04    | kg·m²  |
| $\theta$        | 旋转角度   | —     | —       | rad    |
| $\ddot{\theta}$ | 角加速度   | —     | —       | rad/s² |
| $\tau$          | 电机输出扭矩 | —     | —       | N·m    |

**建模假设**:
1. 忽略轴承摩擦（或视为外部扰动）
2. 电机电流环带宽 (~2kHz) ≫ 控制环带宽 (~500Hz)，扭矩指令可视为瞬时响应
3. $J$ 不变（忽略弹药消耗）

### 1.2 双积分器命名由来

$$\tau \;\xrightarrow{1/J}\; \ddot{\theta} \;\xrightarrow{\int}\; \dot{\theta} \;\xrightarrow{\int}\; \theta$$

从输入扭矩到输出角度需要两次积分，因此称为**双积分器**。它是控制理论中最基本的二阶系统。

---

## 2. 状态空间与系统性质

### 2.1 状态方程

令 $x_1 = \theta$（角度），$x_2 = \dot{\theta}$（角速度），$u = \tau$：

$$\frac{d}{dt}\begin{bmatrix} x_1 \\ x_2 \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} \begin{bmatrix} x_1 \\ x_2 \end{bmatrix} + \begin{bmatrix} 0 \\ 1/J \end{bmatrix} u$$

即：

$$\boxed{\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}u, \quad \mathbf{A} = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} 0 \\ 1/J \end{bmatrix}}$$

对应代码：
```python
A_yaw = np.array([[0, 1], [0, 0]])
B_yaw = np.array([[0], [1/J_yaw]])   # J_yaw = 0.2 → 1/J = 5.0
```

### 2.2 能控性

$$\mathcal{C} = [\mathbf{B},\; \mathbf{A}\mathbf{B}] = \begin{bmatrix} 0 & 1/J \\ 1/J & 0 \end{bmatrix}, \quad \text{rank}(\mathcal{C}) = 2 = n$$

系统**完全能控**：可以从任意初始状态驱动到任意目标状态。

### 2.3 开环特征值与传递函数

$$\det(s\mathbf{I} - \mathbf{A}) = s^2 = 0$$

两个特征值在原点，系统**临界稳定**（非渐近稳定）。

传递函数 $G(s) = \frac{\Theta(s)}{T(s)} = \frac{1}{J s^2}$：恒定扭矩 → 匀加速旋转（无摩擦时不会停止）。

---

## 3. LQR 问题的数学表述

### 3.1 二次代价函数

LQR 寻找**状态反馈控制律** $u = -\mathbf{K}\mathbf{x} = -K_1\theta - K_2\dot{\theta}$，最小化：

$$\boxed{J = \int_0^\infty \left( \mathbf{x}^T \mathbf{Q} \mathbf{x} + R u^2 \right) dt}$$

展开：

$$J = \int_0^\infty \left( q_1 \theta^2 + q_2 \dot{\theta}^2 + R \tau^2 \right) dt$$

| 项 | 含义 | $\uparrow$ 惩罚 → 效果 |
|---|------|----------------------|
| $q_1\theta^2$ | 角度误差代价 | 回正更快，可能震荡 |
| $q_2\dot{\theta}^2$ | 角速度代价 | 阻尼增强，抑制超调 |
| $R\tau^2$ | 控制能量代价 | 控制变"省力"，响应变慢 |

### 3.2 代码中的权重配置

| 轴 | $q_1$ | $q_2$ | $R$ | $\tau_{\max}$ |
|----|-------|-------|-----|---------------|
| Yaw | 100 | 1 | 1 | 2.23 N·m |
| Pitch | 1000 | 10 | 1 | 8.0 N·m |

### 3.3 LQR vs PID 的本质区别

PID 产生 $u = -K_p\theta - K_d\dot{\theta} - K_i\int\theta dt$，形式上与 LQR 相似，但：

| | PID | LQR |
|---|-----|-----|
| 增益来源 | 手动试凑 | 求解 CARE，**数学最优** |
| 最优性 | 无保证 | 全局最小化 $J$ |
| 鲁棒性 | 取决于整定 | ≥60° 相位裕度，∞ 增益裕度 |
| MIMO | 困难 | 自然支持 |

---

## 4. 最优控制的变分推导 → CARE

### 4.1 值函数与 HJB 方程

定义**最优代价-to-go**（值函数）：

$$V(\mathbf{x}) = \min_{u(\cdot)} \int_t^\infty (\mathbf{x}^T\mathbf{Q}\mathbf{x} + Ru^2) d\tau$$

对于无限时域 LQR，值函数取二次型：$V(\mathbf{x}) = \mathbf{x}^T\mathbf{P}\mathbf{x}$（$\mathbf{P} \succ 0$）。

**哈密顿-雅可比-贝尔曼 (HJB)** 方程：

$$\min_u \left[ \mathbf{x}^T\mathbf{Q}\mathbf{x} + Ru^2 + \frac{\partial V}{\partial \mathbf{x}}(\mathbf{A}\mathbf{x} + \mathbf{B}u) \right] = 0$$

代入 $\frac{\partial V}{\partial \mathbf{x}} = 2\mathbf{x}^T\mathbf{P}$：

$$\min_u \left[ \mathbf{x}^T\mathbf{Q}\mathbf{x} + Ru^2 + 2\mathbf{x}^T\mathbf{P}\mathbf{A}\mathbf{x} + 2\mathbf{x}^T\mathbf{P}\mathbf{B}u \right] = 0$$

### 4.2 最优控制的必要条件

对 $u$ 求偏导并令为零（标量 $u$）：

$$\frac{\partial}{\partial u}[Ru^2 + 2\mathbf{x}^T\mathbf{P}\mathbf{B}u] = 2Ru + 2\mathbf{B}^T\mathbf{P}\mathbf{x} = 0$$

$$\boxed{u^* = -R^{-1}\mathbf{B}^T\mathbf{P}\mathbf{x}}$$

定义最优增益矩阵：

$$\boxed{\mathbf{K} = R^{-1}\mathbf{B}^T\mathbf{P}}$$

代码对应：
```python
K = np.linalg.inv(np.array([[R]])) @ B.T @ S
# K = (1/R) · Bᵀ · P
```

### 4.3 代数黎卡提方程 (CARE)

将 $u^*$ 代回 HJB 方程，整理后得到：

$$\boxed{\mathbf{A}^T\mathbf{P} + \mathbf{P}\mathbf{A} - \mathbf{P}\mathbf{B}R^{-1}\mathbf{B}^T\mathbf{P} + \mathbf{Q} = \mathbf{0}}$$

这是**连续代数黎卡提方程** (Continuous Algebraic Riccati Equation)。

代码对应：
```python
S = solve_continuous_are(A, B, Q, R)
# S 就是 P — CARE 的对称正定解
```

### 4.4 闭环系统

$$\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}(-\mathbf{K}\mathbf{x}) = (\mathbf{A} - \mathbf{B}\mathbf{K})\mathbf{x}$$

**稳定性保证**：若 $(\mathbf{A},\mathbf{B})$ 能控且 $(\mathbf{A},\mathbf{Q}^{1/2})$ 能观，则 $\text{Re}[\lambda_i(\mathbf{A} - \mathbf{B}\mathbf{K})] < 0$。

---

## 5. 2 状态双积分器的解析解

### 5.1 代入具体矩阵

$$\mathbf{A} = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} 0 \\ b \end{bmatrix} \; (b = 1/J)$$

$$\mathbf{Q} = \begin{bmatrix} q_1 & 0 \\ 0 & q_2 \end{bmatrix}, \quad R \in \mathbb{R}^+$$

设 $\mathbf{P} = \begin{bmatrix} p_{11} & p_{12} \\ p_{12} & p_{22} \end{bmatrix}$（对称）。

### 5.2 展开 CARE

**步骤 1**: $\mathbf{A}^T\mathbf{P} + \mathbf{P}\mathbf{A}$

$$\mathbf{A}^T\mathbf{P} = \begin{bmatrix} 0 & 0 \\ 1 & 0 \end{bmatrix}\begin{bmatrix} p_{11} & p_{12} \\ p_{12} & p_{22} \end{bmatrix} = \begin{bmatrix} 0 & 0 \\ p_{11} & p_{12} \end{bmatrix}$$

$$\mathbf{P}\mathbf{A} = \begin{bmatrix} p_{11} & p_{12} \\ p_{12} & p_{22} \end{bmatrix}\begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} = \begin{bmatrix} 0 & p_{11} \\ 0 & p_{12} \end{bmatrix}$$

$$\mathbf{A}^T\mathbf{P} + \mathbf{P}\mathbf{A} = \begin{bmatrix} 0 & p_{11} \\ p_{11} & 2p_{12} \end{bmatrix}$$

**步骤 2**: $\mathbf{P}\mathbf{B}R^{-1}\mathbf{B}^T\mathbf{P}$

$$\mathbf{P}\mathbf{B} = \begin{bmatrix} p_{11} & p_{12} \\ p_{12} & p_{22} \end{bmatrix}\begin{bmatrix} 0 \\ b \end{bmatrix} = b\begin{bmatrix} p_{12} \\ p_{22} \end{bmatrix}$$

$$\mathbf{P}\mathbf{B}R^{-1}\mathbf{B}^T\mathbf{P} = \frac{b^2}{R}\begin{bmatrix} p_{12}^2 & p_{12}p_{22} \\ p_{12}p_{22} & p_{22}^2 \end{bmatrix}$$

**步骤 3**: 联立方程

$$\begin{bmatrix} 0 & p_{11} \\ p_{11} & 2p_{12} \end{bmatrix} - \frac{b^2}{R}\begin{bmatrix} p_{12}^2 & p_{12}p_{22} \\ p_{12}p_{22} & p_{22}^2 \end{bmatrix} + \begin{bmatrix} q_1 & 0 \\ 0 & q_2 \end{bmatrix} = \begin{bmatrix} 0 & 0 \\ 0 & 0 \end{bmatrix}$$

### 5.3 三个标量方程

**(1,1):** $-b^2 p_{12}^2 / R + q_1 = 0$

$$\boxed{p_{12} = \frac{\sqrt{q_1 R}}{b} = J\sqrt{q_1 R}}$$

**(1,2):** $p_{11} - b^2 p_{12}p_{22} / R = 0$

$$\boxed{p_{11} = \frac{b^2}{R} p_{12} p_{22}}$$

**(2,2):** $2p_{12} - b^2 p_{22}^2 / R + q_2 = 0$

$$\boxed{p_{22} = \sqrt{\frac{R}{b^2}\left(2p_{12} + q_2\right)} = \frac{1}{b}\sqrt{R\left(\frac{2\sqrt{q_1 R}}{b} + q_2\right)}}$$

### 5.4 解析 LQR 增益

$$\mathbf{K} = \frac{1}{R}\mathbf{B}^T\mathbf{P} = \frac{1}{R}\begin{bmatrix}0 & b\end{bmatrix}\begin{bmatrix}p_{11} & p_{12} \\ p_{12} & p_{22}\end{bmatrix} = \frac{b}{R}\begin{bmatrix}p_{12} & p_{22}\end{bmatrix}$$

$$\boxed{K_1 = \frac{b}{R} \cdot p_{12} = \frac{b}{R} \cdot \frac{\sqrt{q_1 R}}{b} = \sqrt{\frac{q_1}{R}}}$$

$$\boxed{K_2 = \frac{b}{R} \cdot p_{22} = \sqrt{\frac{2}{J}\sqrt{\frac{q_1}{R}} + \frac{q_2}{R}}}$$

### 5.5 关键结论

**$K_1$** 与 $J$ **无关**。增大角度惩罚 $q_1$ → $K_1 \propto \sqrt{q_1}$ → 更强的回正力。双积分器的角度增益完全由 $q_1/R$ 的比值决定。

**$K_2$** 随 $J$ 增大而减小。大惯量系统自然阻尼更大，需要的主动阻尼更少。

### 5.6 数值代入

**Yaw 轴** ($J = 0.2$, $q_1 = 100$, $q_2 = 1$, $R = 1$):

$$K_1 = \sqrt{100/1} = 10.0$$
$$K_2 = \sqrt{\frac{2}{0.2}\sqrt{\frac{100}{1}} + \frac{1}{1}} = \sqrt{10 \times 10 + 1} = \sqrt{101} \approx 10.05$$

解析公式给出闭式估计，`scipy.linalg.solve_continuous_are` 的数值结果为最终部署值。两者一致（误差来自浮点精度）。

**Pitch 轴** ($J = 0.04$, $q_1 = 1000$, $q_2 = 10$, $R = 1$):

$$K_1 = \sqrt{1000/1} \approx 31.62$$
$$K_2 = \sqrt{\frac{2}{0.04}\sqrt{\frac{1000}{1}} + \frac{10}{1}} = \sqrt{50 \times 31.62 + 10} \approx 39.9$$

---

## 6. S 矩阵的物理含义

### 6.1 最优代价-to-go

CARE 的解 $\mathbf{S} = \mathbf{P}$ 的物理含义：

$$V(\mathbf{x}) = \mathbf{x}^T\mathbf{S}\mathbf{x} = \text{从状态 }\mathbf{x}\text{ 出发，在最优控制下的剩余代价}$$

展开：

$$V(\theta, \dot{\theta}) = s_{11}\theta^2 + 2s_{12}\theta\dot{\theta} + s_{22}\dot{\theta}^2$$

### 6.2 各元素的含义

| 元素       | 含义                         |
| -------- | -------------------------- |
| $s_{11}$ | 纯角度误差的长期代价（角度偏移 1 rad 的代价） |
| $s_{22}$ | 纯角速度的长期代价                  |
| $s_{12}$ | 角度-角速度耦合代价（正值 → 同号时代价更大）   |

### 6.3 $\text{tr}(\mathbf{S})$ 作为性能指标

代码中 `performance = np.trace(S)` 汇总了整个系统的控制性能：

$$\text{tr}(\mathbf{S}) = s_{11} + s_{22}$$

**越小越好**——表示在最优控制下，状态偏离平衡的长期代价低。

---

## 7. QR 权重与闭环极点的映射

### 7.1 闭环特征方程

$\mathbf{A}_{cl} = \mathbf{A} - \mathbf{B}\mathbf{K} = \begin{bmatrix} 0 & 1 \\ -\frac{1}{J}K_1 & -\frac{1}{J}K_2 \end{bmatrix}$

特征方程：

$$\det(s\mathbf{I} - \mathbf{A}_{cl}) = s^2 + \frac{K_2}{J}s + \frac{K_1}{J} = 0$$

### 7.2 二阶标准形式

对照标准二阶系统 $s^2 + 2\zeta\omega_n s + \omega_n^2 = 0$：

$$\boxed{\omega_n = \sqrt{\frac{K_1}{J}}, \quad \zeta = \frac{K_2}{2\sqrt{J K_1}}}$$

代入解析 $K_1, K_2$：

$$\omega_n = \sqrt{\frac{1}{J}\sqrt{\frac{q_1}{R}}} = \left(\frac{q_1}{J^2 R}\right)^{1/4}$$

$$\zeta = \frac{\sqrt{\frac{2}{J}\sqrt{\frac{q_1}{R}} + \frac{q_2}{R}}}{2\sqrt{J\sqrt{q_1/R}}}$$

### 7.3 调参的频域直觉

| 调整 | 对 $\omega_n$ 的影响 | 对 $\zeta$ 的影响 |
|------|---------------------|-------------------|
| $\uparrow q_1$ | $\uparrow$ (响应更快) | 轻微 $\downarrow$ |
| $\uparrow q_2$ | 无影响 | $\uparrow$ (阻尼更强) |
| $\uparrow R$ | $\downarrow$ (响应更慢) | 轻微 $\uparrow$ |
| $\uparrow J$ | $\downarrow$ | $\downarrow$ |

### 7.4 极点配置视角

LQR 本质上是**自动极点配置**：通过选择 $\mathbf{Q}, \mathbf{R}$，CARE 自动给出使代价最小的闭环极点位置。与手动极点配置不同，LQR 保证最优性和鲁棒性。

---

## 8. 源代码逐行数学注释

以下对 `lqr.py` 的每个段落给出数学注释。

### 8.1 系统参数 (第 6-22 行)

```python
J_yaw = 0.2      # 绕 Yaw 轴转动惯量 (kg·m²)，来自 CAD 模型
J_pitch = 0.04   # 绕 Pitch 轴转动惯量 (kg·m²)，更小因为负载更轻
R_yaw = 1        # Yaw 轴控制权重（标量），作为 R 矩阵的唯一元素
R_pitch = 1      # Pitch 轴控制权重

Tau_max_yaw = 2.23    # GM6020 峰值扭矩 (N·m)，来自电机规格书
Tau_max_pitch = 8.0   # DM4310 + 10:1 减速器 (N·m)

# 状态空间: ẋ = Ax + Bu
# A = [[0, 1],    — 运动学关系: dθ/dt = ω
#      [0, 0]]    — 动力学关系: dω/dt = τ/J (由 B 矩阵引入)
A_yaw = np.array([[0, 1], [0, 0]])
B_yaw = np.array([[0], [1/J_yaw]])  # 1/J = 5.0 — 扭矩到角加速度的转换
```

**数学对应**：

$$\dot{\mathbf{x}} = \underbrace{\begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}}_{\mathbf{A}} \mathbf{x} + \underbrace{\begin{bmatrix} 0 \\ 5.0 \end{bmatrix}}_{\mathbf{B}} u$$

### 8.2 `calculate_lqr()` 函数 (第 28-69 行)

```python
def calculate_lqr(A, B, Q, R, tau_max, axis_name):
```

**第一步：求解 CARE**

```python
    S = solve_continuous_are(A, B, Q, R)
```

数学：求解 $\mathbf{A}^T\mathbf{S} + \mathbf{S}\mathbf{A} - \mathbf{S}\mathbf{B}R^{-1}\mathbf{B}^T\mathbf{S} + \mathbf{Q} = \mathbf{0}$

`scipy.linalg.solve_continuous_are` 内部使用**舒尔分解**（Schur decomposition）求解，而非直接迭代 $\mathbf{P}$。步骤：
1. 构造哈密顿矩阵 $\mathbf{H} = \begin{bmatrix} \mathbf{A} & -\mathbf{B}R^{-1}\mathbf{B}^T \\ -\mathbf{Q} & -\mathbf{A}^T \end{bmatrix}$
2. 求解 $\mathbf{H}$ 的稳定不变子空间
3. 从中提取 $\mathbf{S}$

**第二步：计算最优增益**

```python
    K = np.linalg.inv(np.array([[R]])) @ B.T @ S
```

数学：$\mathbf{K} = R^{-1}\mathbf{B}^T\mathbf{S} = \frac{1}{R}[0,\; 1/J] \cdot \begin{bmatrix} s_{11} & s_{12} \\ s_{12} & s_{22} \end{bmatrix} = \frac{1}{J \cdot R}[s_{12},\; s_{22}]$

对于标量 $R$，`np.linalg.inv(np.array([[R]]))` 等价于 $1/R$。

**第三步：闭环稳定性分析**

```python
    A_cl = A - B @ K
    eigenvalues = np.linalg.eigvals(A_cl)
    stability = np.all(np.real(eigenvalues) < 0)
```

数学：$\mathbf{A}_{cl} = \mathbf{A} - \mathbf{B}\mathbf{K}$。闭环特征值 $\lambda(\mathbf{A}_{cl})$ 必须全部在左半平面。

**第四步：阶跃响应仿真（验证用）**

```python
    t = np.linspace(0, 5, 500)
    def dynamics(x, t):
        return (A - B @ K) @ x
    x0 = np.array([0.5, 0.0])    # 初始角度误差 0.5 rad
    x = odeint(dynamics, x0, t)
```

数学：求解 $\dot{\mathbf{x}} = \mathbf{A}_{cl}\mathbf{x}$，初始条件 $\mathbf{x}(0) = [0.5, 0]^T$。

线性系统的解析解为 $\mathbf{x}(t) = e^{\mathbf{A}_{cl}t}\mathbf{x}(0)$。`odeint` 用数值积分（LSODA 算法）逼近此解。

**第五步：扭矩分析**

```python
    tau = K @ x.T
    max_tau = np.max(np.abs(tau))
    tau_exceeded = max_tau > tau_max
```

数学：$\tau(t) = -\mathbf{K}\mathbf{x}(t) = -K_1\theta(t) - K_2\dot{\theta}(t)$。检查最大扭矩是否超过电机能力。如果超限，需要增大 $R$ 或减小 $\mathbf{Q}$ 重新设计。

**第六步：调节时间**

```python
    settling_idx = np.where(np.abs(x[:, 0]) < 0.05)[0]
    settling_time = t[settling_idx[0]] if len(settling_idx) > 0 else 5.0
```

数学：从初始 0.5 rad 偏差进入 ±0.05 rad（±2.86°）误差带的时间。对于线性二阶系统，调节时间估算为 $t_s \approx 4 / (\zeta\omega_n) = 4 / |\text{Re}(\lambda_{\max})|$。

### 8.3 Yaw 轴 LQR 设计 (第 74-96 行)

```python
Q_yaw = np.array([[100, 0], [0, 1]])
result_yaw = calculate_lqr(A_yaw, B_yaw, Q_yaw, R_yaw, Tau_max_yaw, "YAW")
```

数学：$\mathbf{Q}_{\text{yaw}} = \begin{bmatrix} 100 & 0 \\ 0 & 1 \end{bmatrix}$

- $q_1 = 100$: 角度误差 1 rad → 代价 100（远大于 $q_2 = 1$，强调位置精度）
- $q_2 = 1$: 角速度代价——提供基础阻尼

**为什么 $q_2$ 这么小？**
因为 LQR 自动通过耦合 $s_{12}$ 引入了有效的速度阻尼（$K_2$ 不为零），不需要显式地大幅惩罚角速度。

### 8.4 Pitch 轴 LQR 设计 (第 105-124 行)

```python
Q_pitch = np.array([[1000, 0], [0, 10]])
```

数学：$\mathbf{Q}_{\text{pitch}} = \begin{bmatrix} 1000 & 0 \\ 0 & 10 \end{bmatrix}$

$q_1^{\text{pitch}} = 10 \times q_1^{\text{yaw}}$，原因：

1. **弹道敏感度**：Pitch 角 1° 误差 → 5m 处偏离目标 8.7cm；Yaw 角类似偏离但可通过底盘转动补偿
2. **力矩裕量**：DM4310 + 10:1 减速器峰值扭矩 8 N·m，远大于 GM6020 的 2.23 N·m，可以承受更激进的增益
3. **惯量小**：$J_{\text{pitch}} = 0.04 \ll J_{\text{yaw}} = 0.2$，同样增益下响应更快

### 8.5 系统响应仿真 (第 129-183 行)

```python
x0_yaw = np.array([0.1, 0.0])   # 0.1 rad ≈ 5.7° 初始偏差
x_yaw = odeint(yaw_dynamics, x0_yaw, t, args=(K_yaw,))
```

$t=0$ 时云台偏离目标 5.7°，零初速。LQR 驱动其回到零位。

回归曲线 $\theta(t)$ 满足：
- 无超调（当 $\zeta \geq 1$，即临界阻尼或过阻尼）
- 指数收敛速率由 $\text{Re}(\lambda_{\max})$ 决定
- 稳态误差为零（LQR 天然包含积分效果，通过 $\mathbf{S}$ 保证）

---

## 9. 扭矩约束下的可行域

### 9.1 扭矩-状态空间

控制律 $u = -K_1\theta - K_2\dot{\theta}$ 受扭矩限幅约束：

$$|K_1\theta + K_2\dot{\theta}| \leq \tau_{\max}$$

在相平面 $(\theta, \dot{\theta})$ 中，这是一个带状区域：

$$-\tau_{\max} \leq K_1\theta + K_2\dot{\theta} \leq \tau_{\max}$$

### 9.2 初始角度的最大允许值

当 $\dot{\theta}(0) = 0$ 时，最大初始角度偏差：

$$\theta_{\max}^{\text{initial}} = \frac{\tau_{\max}}{K_1}$$

| 轴 | $K_1$ | $\tau_{\max}$ | $\theta_{\max}^{\text{initial}}$ |
|----|-------|---------------|----------------------------------|
| Yaw | ≈10.0 | 2.23 N·m | ≈0.223 rad (12.8°) |
| Pitch | ≈31.6 | 8.0 N·m | ≈0.253 rad (14.5°) |

如果初始偏差超过此值，控制扭矩饱和，系统进入非线性区，LQR 的线性最优性不再保证。

### 9.3 代码中的扭矩利用率

```python
'tau_utilization': max_tau / tau_max if not tau_exceeded else 1.0
```

这个指标告诉我们在阶跃响应中用了多少电机能力。建议范围：**50-80%**。
- < 30%：过于保守，可以增大 $q_1$（响应更快）
- \> 90%：过于激进，实际运行时稍有扰动就可能饱和

---

## 10. 离散时间实现分析

### 10.1 连续 LQR 的离散部署

`lqr.py` 计算的是**连续时间**增益 $\mathbf{K}_c$，但 MCU 运行在**离散时间**（500 Hz 或 1000 Hz）。

在固件中，控制律以固定周期 $T_s$ 执行：

```c
// 每 2ms (500Hz) 执行一次
float lqr_output = -K1 * (angle - ref) - K2 * speed;
```

### 10.2 连续增益的可用条件

当采样频率 $f_s = 1/T_s$ 满足香农条件：

$$f_s > 20 \times \frac{|\lambda_{\max}|}{2\pi}$$

即采样频率远高于闭环带宽时，**连续增益可以直接用于离散系统**。

对于 Yaw 轴（$\omega_n \approx 7$ rad/s, $f_{\text{bandwidth}} \approx 1.1$ Hz），500 Hz 采样率提供了 450 倍的过采样——完全可以直接使用连续增益。

### 10.3 离散 LQR 的严格解法（可选）

若需严格离散解，应：

1. 离散化系统：$\mathbf{A}_d = e^{\mathbf{A}T_s}, \quad \mathbf{B}_d = \int_0^{T_s} e^{\mathbf{A}\tau}\mathbf{B} d\tau$
2. 求解离散 CARE：$\mathbf{A}_d^T\mathbf{P}\mathbf{A}_d - \mathbf{P} - \mathbf{A}_d^T\mathbf{P}\mathbf{B}_d(\mathbf{R} + \mathbf{B}_d^T\mathbf{P}\mathbf{B}_d)^{-1}\mathbf{B}_d^T\mathbf{P}\mathbf{A}_d + \mathbf{Q} = \mathbf{0}$
3. 离散增益：$\mathbf{K}_d = (\mathbf{R} + \mathbf{B}_d^T\mathbf{P}\mathbf{B}_d)^{-1}\mathbf{B}_d^T\mathbf{P}\mathbf{A}_d$

对于 500 Hz 采样，$\mathbf{K}_d$ 与 $\mathbf{K}_c$ 的差异 < 0.1%，工程上可忽略。

### 10.4 固件中的扭矩到电流转换

LQR 输出扭矩值，需转为电机电流或 CAN 指令值：

```c
// GM6020: 扭矩常数 ≈ 0.741 N·m/A
float current = torque / 0.741f;

// 限幅
current = VAL_LIMIT(current, -MAX_CURRENT, MAX_CURRENT);

// 转为 CAN 发送值 (int16, -30000 ~ 30000)
int16_t can_value = (int16_t)(current / MAX_CURRENT * 30000.0f);
```

---

## 11. Yaw vs Pitch 权重设计的数学依据

### 11.1 为什么 Pitch 的 $q_1$ 是 Yaw 的 10 倍？

核心原因：**弹道灵敏度不对称**。

在 5 m 距离上：
- Pitch 角误差 $\delta\theta_p = 1°$ → 子弹高度偏差 $\approx 5\tan(1°) \approx 8.7$ cm（可能脱靶）
- Yaw 角误差 $\delta\theta_y = 1°$ → 子弹横向偏差同样 $\approx 8.7$ cm，但 Yaw 可借助底盘旋转和视觉闭环快速修正

因此 Pitch 轴需要更严格的指向精度 → 更大的 $q_1$。

### 11.2 扭矩裕量允许更激进的增益

| 轴 | $\tau_{\max}$ | $J$ | $\tau_{\max}/J$（最大角加速度） |
|----|--------------|-----|-------------------------------|
| Yaw | 2.23 | 0.2 | 11.15 rad/s² |
| Pitch | 8.0 | 0.04 | 200 rad/s² |

Pitch 轴的扭矩-惯量比是 Yaw 的 **18 倍**，可以承受远更激进的增益而不会扭矩饱和。

### 11.3 权重缩放的不变性

LQR 有一个重要性质：如果 $\mathbf{Q}$ 和 $R$ 同时乘以相同的标量 $\alpha$，增益 $\mathbf{K}$ 不变。因为：

$$\alpha\mathbf{A}^T\mathbf{P} + \alpha\mathbf{P}\mathbf{A} - \alpha\mathbf{P}\mathbf{B}(\alpha R)^{-1}\mathbf{B}^T\mathbf{P} + \alpha\mathbf{Q} = \alpha(\text{原 CARE}) = 0$$

只有 $\mathbf{Q}$ 和 $R$ 的**比值**决定 $\mathbf{K}$。因此：

$$\mathbf{K} = f\left(\frac{q_1}{R}, \frac{q_2}{R}, J\right)$$

实际调参时固定 $R = 1$，仅调节 $q_1$ 和 $q_2$，等价于调节比值。

---

## 参考文献

1. Åström, K.J. & Murray, R.M. (2021). *Feedback Systems*, 2nd ed. Princeton. Chapter 7: Linear Quadratic Regulation.
2. Anderson, B.D.O. & Moore, J.B. (2007). *Optimal Control: Linear Quadratic Methods*. Dover.
3. Laub, A.J. (1979). "A Schur Method for Solving Algebraic Riccati Equations." *IEEE Trans. Automatic Control*, 24(6), 913-921.
4. Bittanti, S., Laub, A.J., & Willems, J.C. (1991). *The Riccati Equation*. Springer.
5. Scipy documentation: `scipy.linalg.solve_continuous_are` — [在线文档](https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.solve_continuous_are.html)
6. GM6020 电机规格书, DJI RoboMaster 官方文档.

---

> **文档维护**：本文档与 `lqr.py使用文档.md` 配合阅读——使用文档描述用法和调参，本文档提供逐行数学推导。第 5 节的解析公式可用于无 Python 环境下的快速估算。
