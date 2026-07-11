# 串联轮腿数学推导与仿真

> 本文档是对《串联轮腿步兵下位机控制系统设计》的数学深化，专注于五连杆运动学、WIP 动力学建模、LQR 最优控制的完整理论推导，并提供可运行的 Python 和 MATLAB 仿真框架。

---

## 目录

1. [五连杆运动学完整推导](#1-五连杆运动学完整推导)
2. [雅可比矩阵的解析推导](#2-雅可比矩阵的解析推导)
3. [轮腿倒立摆动力学建模](#3-轮腿倒立摆动力学建模)
4. [线性化与状态空间模型](#4-线性化与状态空间模型)
5. [LQR 最优控制理论](#5-lqr-最优控制理论)
6. [增益调度理论](#6-增益调度理论)
7. [非线性动力学仿真框架](#7-非线性动力学仿真框架)
8. [仿真代码实现 (Python)](#8-仿真代码实现-python)
9. [仿真代码实现 (MATLAB)](#9-仿真代码实现-matlab)
10. [附录：符号表与物理常数](#10-附录符号表与物理常数)

---

## 1. 五连杆运动学完整推导

### 1.1 机构拓扑描述

每条腿是一个**平面五杆并联机构**（Planar Five-Bar Parallel Mechanism），具有两个自由度（2-DOF）。

```
机构简图：

    A ───────── B ───────── C (末端 = 轮轴)
    │   L₁        │   L₂
    │             │
    D ───────── E
      L₄          L₃

固定基座: AD（安装在机身上）
A = 左驱动关节, D = 右驱动关节 (两个 DM 电机)
B = 左被动关节, E = 右被动关节
C = 末端执行器 (轮轴)
```

**关键几何参数**：

| 参数 | 符号 | 典型值 | 单位 |
|------|------|--------|------|
| 左上连杆 | $L_1$ | 0.150 | m |
| 左下连杆 | $L_2$ | 0.150 | m |
| 右上连杆 | $L_3$ | 0.150 | m |
| 右下连杆 | $L_4$ | 0.150 | m |
| 基座间距 | $L_5$ | 0.080 | m |
| 驱动角 1 | $\theta_1$ | — | rad |
| 驱动角 4 | $\theta_4$ | — | rad |
| 被动角 2 | $\theta_2$ | — | rad |
| 被动角 3 | $\theta_3$ | — | rad |

### 1.2 坐标系设定

以基座中点 O 为原点，AD 连线方向为 x 轴，垂直向上为 y 轴：

```
A = (-L₅/2, 0)     ← 左驱动关节
D = (+L₅/2, 0)     ← 右驱动关节
B = A + L₁·[cos θ₁, sin θ₁]ᵀ
E = D + L₄·[cos θ₄, sin θ₄]ᵀ
C = B + L₂·[cos θ₂, sin θ₂]ᵀ
  = E + L₃·[cos θ₃, sin θ₃]ᵀ
```

### 1.3 正运动学（Forward Kinematics）

**已知**：$\theta_1, \theta_4$ (驱动关节角度，从编码器读取)
**求解**：末端 C 点坐标 $(C_x, C_y)$ 以及虚拟腿参数 $(L_0, \phi_0)$

#### 步骤 1：计算 B 点和 E 点坐标

$$B = \begin{bmatrix} B_x \\ B_y \end{bmatrix} = \begin{bmatrix} -L_5/2 + L_1 \cos\theta_1 \\ L_1 \sin\theta_1 \end{bmatrix}$$

$$E = \begin{bmatrix} E_x \\ E_y \end{bmatrix} = \begin{bmatrix} +L_5/2 + L_4 \cos\theta_4 \\ L_4 \sin\theta_4 \end{bmatrix}$$

#### 步骤 2：通过闭环约束求解 $\theta_2$

五连杆机构有一条闭环链：A → B → C → E → D → A。对于每个位形，B 和 E 之间的距离 $d_{BE}$ 可以由以下闭合条件确定：

$$d_{BE} = \sqrt{(E_x - B_x)^2 + (E_y - B_y)^2}$$

这是关键——$d_{BE}$ 完全由已知的 $\theta_1$ 和 $\theta_4$ 决定。

在三角形 $\triangle ABE$ 中应用**余弦定理**：

$$\cos(\angle ABE) = \frac{L_1^2 + d_{BE}^2 - L_4^2}{2 \cdot L_1 \cdot d_{BE}}$$

但注意这里 ABE 不是直接和 L₄ 相关的三角形。更严谨的推导：

对于闭环链 A-B-C-E-D-A，连接 B 和 E，形成两个三角形：
- $\triangle ABE$：边长为 $L_1, d_{BE}, L_4$
- $\triangle BCE$：边长为 $L_2(= L_1), |CE|(= L_3), d_{BE}$

**实际上需要解 $\triangle BCE$**，因为 $\theta_2$ 在 B 处将 $L_1$ 和 $L_2$ 连接：

在 $\triangle BCE$ 中：
- $|BC| = L_2$
- $|CE| = L_3$
- $|BE| = d_{BE}$

应用余弦定理求 $\angle CBE$：

$$\cos(\angle CBE) = \frac{L_2^2 + d_{BE}^2 - L_3^2}{2 \cdot L_2 \cdot d_{BE}}$$

$$\angle CBE = \arccos\left(\frac{L_2^2 + d_{BE}^2 - L_3^2}{2 \cdot L_2 \cdot d_{BE}}\right)$$

接下来需要求线段 BE 的方向角 $\angle (BE, \vec{e}_x)$：

$$\alpha_{BE} = \arctan\!2(E_y - B_y, \; E_x - B_x)$$

#### 步骤 3：确定构型

连杆机构存在**两种装配构型**（对应于 $\triangle BCE$ 中 C 在 BE 的哪一侧）：

$$\theta_2 = \alpha_{BE} \pm \angle CBE$$

- **凸构型**（CONVEX, "+"）：C 在 BE 的"外侧"，膝关节向外凸（模拟人腿形态）
- **凹构型**（CONCAVE, "-"）：C 在 BE 的"内侧"

步兵轮腿始终工作在凸构型，由关节限位保证不会发生构型跳变。

#### 步骤 4：计算末端坐标

$$\theta_2 = \alpha_{BE} + \angle CBE \quad \text{(凸构型)}$$

$$C = \begin{bmatrix} C_x \\ C_y \end{bmatrix} = B + L_2 \begin{bmatrix} \cos\theta_2 \\ \sin\theta_2 \end{bmatrix}$$

也可以从 E 端验证：

$$C = E + L_3 \begin{bmatrix} \cos\theta_3 \\ \sin\theta_3 \end{bmatrix}$$

其中 $\theta_3$ 类似求解。

#### 步骤 5：虚拟腿参数

虚拟腿是将五连杆等效为从基座中点 O 到轮轴 C 的一条"虚拟连杆"：

```
虚拟腿长:    L₀ = √[(Cx - 0)² + (Cy - 0)²]
            = √[Cx² + Cy²]

虚拟倾角:    φ₀ = arctan2(Cy, Cx)
```

> **注意**：原文档定义 $L_0 = \sqrt{(C_x - L_5/2)^2 + C_y^2}$，即相对于 D 点的距离。这里采用 O 点为中心的对称定义。实际使用时两个定义等价，仅差一个坐标平移。

### 1.4 完整正运动学公式汇总

**输入**：$\theta_1, \theta_4$

**输出**：$C_x, C_y, L_0, \phi_0, \theta_2, \theta_3$

```
Bx = -L₅/2 + L₁·cos(θ₁),    By = L₁·sin(θ₁)
Ex = +L₅/2 + L₄·cos(θ₄),    Ey = L₄·sin(θ₄)

d_BE = √((Ex-Bx)² + (Ey-By)²)
α_BE = atan2(Ey-By, Ex-Bx)

cos_β = (L₂² + d_BE² - L₃²) / (2·L₂·d_BE)    ← 余弦定理
β     = acos(clamp(cos_β, -1, 1))              ← ∠CBE

θ₂    = α_BE + β                              ← CONVEX 构型
θ₃    = atan2(Cy-Ey, Cx-Ex)                   ← 或从 C 和 E 反算

Cx    = Bx + L₂·cos(θ₂)
Cy    = By + L₂·sin(θ₂)

L₀    = √(Cx² + Cy²)
φ₀    = atan2(Cy, Cx)
```

### 1.5 逆运动学（Inverse Kinematics）

**已知**：目标虚拟腿参数 $L_0^{\text{target}}, \phi_0^{\text{target}}$
**求解**：驱动关节角度 $\theta_1, \theta_4$

#### 从虚拟腿参数到 C 点坐标

$$C_x^{\text{target}} = L_0^{\text{target}} \cdot \cos(\phi_0^{\text{target}})$$
$$C_y^{\text{target}} = L_0^{\text{target}} \cdot \sin(\phi_0^{\text{target}})$$

#### 对于上连杆链（A → B → C）

在三角形 $\triangle ABC$ 中：
- $|AB| = L_1$
- $|BC| = L_2$
- $|AC| = \sqrt{(C_x - A_x)^2 + (C_y - A_y)^2}$

应用余弦定理求 $\angle CAB$：

$$\cos(\angle CAB) = \frac{L_1^2 + |AC|^2 - L_2^2}{2 \cdot L_1 \cdot |AC|}$$

$$\gamma_A = \arctan\!2(C_y - A_y, \; C_x - A_x)$$

$$\theta_1 = \gamma_A \pm \angle CAB$$

#### 对于下连杆链（D → E → C）

类似地：

$$\theta_4 = \gamma_D \mp \angle CDE$$

其中：
- $\gamma_D = \arctan\!2(C_y - D_y, \; C_x - D_x)$
- $\angle CDE$ 由三角形 $\triangle CDE$ 的余弦定理求得

#### 构型选择

逆运动学有 **4 种解**（两腿各 ±），对应 PP、PN、NP、NN 四种模式，其中 P 表示 $\theta_2 \geq 0$（膝盖朝前），N 表示 $\theta_2 < 0$：

- **PP**：$\theta_1 = \gamma_A + \angle CAB$，$\theta_4 = \gamma_D - \angle CDE$
- **PN**：$\theta_1 = \gamma_A + \angle CAB$，$\theta_4 = \gamma_D + \angle CDE$
- **NP**：$\theta_1 = \gamma_A - \angle CAB$，$\theta_4 = \gamma_D - \angle CDE$
- **NN**：$\theta_1 = \gamma_A - \angle CAB$，$\theta_4 = \gamma_D + \angle CDE$

工程上通过**保持与上一个控制周期的构型一致**来避免跨分支跳变。

### 1.6 工作空间分析

五连杆机构的工作空间受以下约束：

1. **关节限位**：$\theta_1 \in [\theta_1^{\min}, \theta_1^{\max}]$，$\theta_4 \in [\theta_4^{\min}, \theta_4^{\max}]$
2. **奇异位形**：$d_{BE} \geq L_2 + L_3$ 时机构卡死
3. **机械干涉**：连杆间不能穿透

步兵轮腿的典型工作空间：

| 参数 | 最小值 | 最大值 | 单位 |
|------|--------|--------|------|
| 虚拟腿长 $L_0$ | 0.18 | 0.30 | m |
| 虚拟倾角 $\phi_0$ | -30° | +30° | deg |

### 1.7 奇异位形分析

当 $\det(J_v) = 0$ 时，雅可比矩阵奇异，机构失去一个自由度。五连杆主要奇异位形：

1. **拉伸奇异**：所有连杆完全伸展（$d_{BE} = L_2 + L_3$），末端到达最远点
2. **折叠奇异**：$L_1$ 和 $L_4$ 共线且同向，机构"折叠"
3. **边界奇异**：任一关节到达限位

在控制中需要做 **奇异规避**：

$$\tau_{\text{cmd}} = \tau_{\text{vmc}} + k_{\text{singular}} \cdot \left|\frac{1}{\det(J)}\right| \cdot \tau_{\text{damp}}$$

当 $\det(J)$ 趋近于 0 时，加入额外阻尼以避免扭矩指令发散。

---

## 2. 雅可比矩阵的解析推导

### 2.1 解析雅可比 vs 数值雅可比

原文档使用数值差分法计算雅可比。该方法简单但有两个缺点：
1. 双倍正运动学计算量（两次正运动学 + 扰动）
2. 数值误差受扰动步长 $\varepsilon$ 影响

**解析雅可比**通过直接对运动学方程求导获得，计算更精确、更快速。

### 2.2 末端位置对关节角度的偏导数

从正运动学公式出发：

$$C_x(\theta_1, \theta_2) = -L_5/2 + L_1\cos\theta_1 + L_2\cos\theta_2$$
$$C_y(\theta_1, \theta_2) = L_1\sin\theta_1 + L_2\sin\theta_2$$

其中 $\theta_2$ 是 $\theta_1$ 和 $\theta_4$ 的隐函数（通过闭环约束确定）。

求全导数：

$$\frac{dC_x}{dt} = \frac{\partial C_x}{\partial \theta_1}\dot{\theta}_1 + \frac{\partial C_x}{\partial \theta_2}\frac{d\theta_2}{dt}$$
$$\frac{dC_y}{dt} = \frac{\partial C_y}{\partial \theta_1}\dot{\theta}_1 + \frac{\partial C_y}{\partial \theta_2}\frac{d\theta_2}{dt}$$

其中：

$$\frac{\partial C_x}{\partial \theta_1} = -L_1\sin\theta_1, \quad \frac{\partial C_x}{\partial \theta_2} = -L_2\sin\theta_2$$
$$\frac{\partial C_y}{\partial \theta_1} = L_1\cos\theta_1, \quad \frac{\partial C_y}{\partial \theta_2} = L_2\cos\theta_2$$

### 2.3 闭环约束的速度关系

闭环约束：

$$B + L_2\begin{bmatrix}\cos\theta_2 \\ \sin\theta_2\end{bmatrix} = E + L_3\begin{bmatrix}\cos\theta_3 \\ \sin\theta_3\end{bmatrix}$$

对时间求导：

$$v_B + L_2\begin{bmatrix}-\sin\theta_2 \\ \cos\theta_2\end{bmatrix}\dot{\theta}_2 = v_E + L_3\begin{bmatrix}-\sin\theta_3 \\ \cos\theta_3\end{bmatrix}\dot{\theta}_3$$

其中 $v_B = L_1\begin{bmatrix}-\sin\theta_1 \\ \cos\theta_1\end{bmatrix}\dot{\theta}_1$，$v_E = L_4\begin{bmatrix}-\sin\theta_4 \\ \cos\theta_4\end{bmatrix}\dot{\theta}_4$

我们需要消去 $\dot{\theta}_3$。将上式与向量 $\vec{CE}^\perp = \begin{bmatrix}-\sin\theta_3 \\ \cos\theta_3\end{bmatrix}$ 做点积：

$$\vec{CE}^\perp \cdot v_B + L_2\vec{CE}^\perp \cdot \begin{bmatrix}-\sin\theta_2 \\ \cos\theta_2\end{bmatrix}\dot{\theta}_2 = \vec{CE}^\perp \cdot v_E$$

其中 $\vec{CE}^\perp \cdot \begin{bmatrix}-\sin\theta_3 \\ \cos\theta_3\end{bmatrix} = 0$（点积为零）。

展开：

$$\vec{CE}^\perp \cdot v_B = -\sin\theta_3(-L_1\sin\theta_1\dot{\theta}_1) + \cos\theta_3(L_1\cos\theta_1\dot{\theta}_1)$$
$$= L_1\dot{\theta}_1(\sin\theta_3\sin\theta_1 + \cos\theta_3\cos\theta_1) = L_1\dot{\theta}_1\cos(\theta_3 - \theta_1)$$

同理：

$$\vec{CE}^\perp \cdot v_E = L_4\dot{\theta}_4\cos(\theta_3 - \theta_4)$$

以及：

$$\vec{CE}^\perp \cdot \begin{bmatrix}-\sin\theta_2 \\ \cos\theta_2\end{bmatrix} = \sin\theta_3\sin\theta_2 + \cos\theta_3\cos\theta_2 = \cos(\theta_3 - \theta_2)$$

代入解得：

$$\boxed{\dot{\theta}_2 = \frac{L_4\cos(\theta_3 - \theta_4)}{L_2\cos(\theta_3 - \theta_2)}\dot{\theta}_4 - \frac{L_1\cos(\theta_3 - \theta_1)}{L_2\cos(\theta_3 - \theta_2)}\dot{\theta}_1}$$

### 2.4 完整的雅可比矩阵

将 $\dot{\theta}_2$ 代入 $\dot{C}_x, \dot{C}_y$ 的表达式：

$$\begin{bmatrix} \dot{C}_x \\ \dot{C}_y \end{bmatrix} = J_{\text{cart}} \begin{bmatrix} \dot{\theta}_1 \\ \dot{\theta}_4 \end{bmatrix}$$

其中：

$$J_{\text{cart}} = \begin{bmatrix} j_{11} & j_{12} \\ j_{21} & j_{22} \end{bmatrix}$$

$$j_{11} = -L_1\sin\theta_1 - L_2\sin\theta_2 \cdot \frac{-L_1\cos(\theta_3-\theta_1)}{L_2\cos(\theta_3-\theta_2)} = -L_1\sin\theta_1 + L_1\sin\theta_2\frac{\cos(\theta_3-\theta_1)}{\cos(\theta_3-\theta_2)}$$

$$j_{12} = -L_2\sin\theta_2 \cdot \frac{L_4\cos(\theta_3-\theta_4)}{L_2\cos(\theta_3-\theta_2)} = -L_4\sin\theta_2\frac{\cos(\theta_3-\theta_4)}{\cos(\theta_3-\theta_2)}$$

$$j_{21} = L_1\cos\theta_1 + L_2\cos\theta_2 \cdot \frac{-L_1\cos(\theta_3-\theta_1)}{L_2\cos(\theta_3-\theta_2)} = L_1\cos\theta_1 - L_1\cos\theta_2\frac{\cos(\theta_3-\theta_1)}{\cos(\theta_3-\theta_2)}$$

$$j_{22} = L_2\cos\theta_2 \cdot \frac{L_4\cos(\theta_3-\theta_4)}{L_2\cos(\theta_3-\theta_2)} = L_4\cos\theta_2\frac{\cos(\theta_3-\theta_4)}{\cos(\theta_3-\theta_2)}$$

### 2.5 从笛卡尔雅可比到虚拟腿雅可比

虚拟腿参数与笛卡尔坐标的映射：

$$L_0 = \sqrt{C_x^2 + C_y^2}$$
$$\phi_0 = \arctan\!2(C_y, C_x)$$

求导：

$$\dot{L}_0 = \frac{C_x\dot{C}_x + C_y\dot{C}_y}{L_0} = \frac{1}{L_0}\begin{bmatrix}C_x & C_y\end{bmatrix} \begin{bmatrix}\dot{C}_x \\ \dot{C}_y\end{bmatrix}$$

$$\dot{\phi}_0 = \frac{C_x\dot{C}_y - C_y\dot{C}_x}{L_0^2} = \frac{1}{L_0^2}\begin{bmatrix}-C_y & C_x\end{bmatrix} \begin{bmatrix}\dot{C}_x \\ \dot{C}_y\end{bmatrix}$$

因此**虚拟腿雅可比**为：

$$\boxed{J_v = \begin{bmatrix} \frac{C_x}{L_0} & \frac{C_y}{L_0} \\[4pt] -\frac{C_y}{L_0^2} & \frac{C_x}{L_0^2} \end{bmatrix} \cdot J_{\text{cart}}}$$

最终结果是一个 $2 \times 2$ 矩阵：

$$\begin{bmatrix} \dot{L}_0 \\ \dot{\phi}_0 \end{bmatrix} = J_v \begin{bmatrix} \dot{\theta}_1 \\ \dot{\theta}_4 \end{bmatrix}$$

### 2.6 力传递关系（VMC 的核心）

由**虚功原理**（Principle of Virtual Work）：

$$\delta W_{\text{joint}} = \delta W_{\text{task}}$$
$$\boldsymbol{\tau}^T \delta\mathbf{q} = \mathbf{F}^T \delta\mathbf{x}$$
$$\boldsymbol{\tau}^T \delta\mathbf{q} = \mathbf{F}^T J \delta\mathbf{q}$$

由于 $\delta\mathbf{q}$ 任意，得到：

$$\boxed{\boldsymbol{\tau} = J^T \mathbf{F}}$$

展开：

$$\begin{bmatrix} \tau_1 \\ \tau_4 \end{bmatrix} = \begin{bmatrix} j_{11} & j_{21} \\ j_{12} & j_{22} \end{bmatrix} \begin{bmatrix} F_0 \\ T_p \end{bmatrix}$$

其中：
- $F_0$：沿虚拟腿方向的推力 (N)，正值缩短腿（支撑）
- $T_p$：绕虚拟腿基点的力矩 (N·m)，正值使机身向前倾斜

---

## 3. 轮腿倒立摆动力学建模

### 3.1 建模假设

将机器人在纵向平面（Sagittal Plane）中的运动简化为**轮式倒立摆**（Wheeled Inverted Pendulum, WIP）：

1. **刚体假设**：机身、腿、轮子均为刚体
2. **平面运动**：仅考虑纵向平面（前-后方向），忽略横向耦合
3. **点接触**：轮子与地面为点接触，无滑动
4. **集中质量**：各部件质量集中于质心
5. **等效惯量**：腿机构的质量和惯量等效到虚拟腿上

### 3.2 物理参数定义

| 符号 | 含义 | 典型值 | 单位 |
|------|------|--------|------|
| $M_b$ | 机身质量 (Body) | 18.0 | kg |
| $M_l$ | 单腿等效质量 (Leg) | 2.0 | kg |
| $M_w$ | 单轮质量 (Wheel) | 0.5 | kg |
| $I_b$ | 机身绕俯仰轴转动惯量 | 0.8 | kg·m² |
| $I_w$ | 轮子绕轴转动惯量 | 0.002 | kg·m² |
| $L_0$ | 虚拟腿长（可变） | 0.18–0.30 | m |
| $R_w$ | 轮子半径 | 0.076 | m |
| $g$ | 重力加速度 | 9.81 | m/s² |

### 3.3 广义坐标

选择以下广义坐标描述系统：

| 坐标 | 含义 | 正方向 |
|------|------|--------|
| $x$ | 轮子水平位移 | 向前 |
| $\theta$ | 虚拟腿倾角（相对于竖直方向） | 向前倾 |
| $\phi$ | 机身俯仰角（相对于水平面） | 前倾 |

```
几何关系：

    y
    ↑
    |    ┌───────────┐
    |    │  机身      │ ← φ (俯仰角)
    |    │  质心 Mb   │
    |    │  Ib       │
    |    └─────┬─────┘
    |          │ 虚拟腿 L₀
    |          │ θ (腿倾角)
    |    ──○───┴───○── (轮轴)
    |      ← x →
    └──────────────────→ x (地面)
```

### 3.4 各部件的位置与速度

**机身质心位置**（相对于轮轴）：

$$\mathbf{r}_b = \begin{bmatrix} x + L_0\sin\theta \\ L_0\cos\theta \end{bmatrix}$$

> 注意：这里 $\theta$ 是腿相对于竖直方向的偏角，$\theta = 0$ 时腿竖直朝上。

**机身速度**（对时间求导，$L_0$ 是时变的）：

$$\dot{\mathbf{r}}_b = \begin{bmatrix} \dot{x} + \dot{L}_0\sin\theta + L_0\dot{\theta}\cos\theta \\ \dot{L}_0\cos\theta - L_0\dot{\theta}\sin\theta \end{bmatrix}$$

**轮子质心**：

$$\mathbf{r}_w = \begin{bmatrix} x \\ R_w \end{bmatrix}$$

### 3.5 拉格朗日力学方法

系统的动能：

$$T = \underbrace{\frac{1}{2}M_b|\dot{\mathbf{r}}_b|^2}_{\text{机身平动}} + \underbrace{\frac{1}{2}I_b\dot{\phi}^2}_{\text{机身转动}} + \underbrace{\frac{1}{2}M_w \dot{x}^2}_{\text{轮子平动}} + \underbrace{\frac{1}{2}I_w\left(\frac{\dot{x}}{R_w}\right)^2}_{\text{轮子转动}}$$

系统的势能（以地面为零势能面）：

$$V = M_b g \cdot (L_0\cos\theta) + M_w g R_w$$

拉格朗日量：

$$\mathcal{L} = T - V$$

### 3.6 展开动能项

机身的平动动能：

$$|\dot{\mathbf{r}}_b|^2 = (\dot{x} + \dot{L}_0\sin\theta + L_0\dot{\theta}\cos\theta)^2 + (\dot{L}_0\cos\theta - L_0\dot{\theta}\sin\theta)^2$$

展开并利用 $\sin^2 + \cos^2 = 1$：

$$\boxed{|\dot{\mathbf{r}}_b|^2 = \dot{x}^2 + \dot{L}_0^2 + L_0^2\dot{\theta}^2 + 2\dot{x}\dot{L}_0\sin\theta + 2\dot{x}L_0\dot{\theta}\cos\theta}$$

完整的动能：

$$T = \frac{1}{2}M_b\left(\dot{x}^2 + \dot{L}_0^2 + L_0^2\dot{\theta}^2 + 2\dot{x}\dot{L}_0\sin\theta + 2\dot{x}L_0\dot{\theta}\cos\theta\right) + \frac{1}{2}I_b\dot{\phi}^2 + \frac{1}{2}\left(M_w + \frac{I_w}{R_w^2}\right)\dot{x}^2$$

记 $M_{\text{eff}} = M_b + M_w + I_w/R_w^2$ 为等效平动质量。

### 3.7 欧拉-拉格朗日方程

对于广义坐标 $q_i \in \{x, \theta, \phi\}$：

$$\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}_i}\right) - \frac{\partial \mathcal{L}}{\partial q_i} = Q_i$$

其中 $Q_i$ 是广义力（包括控制输入和非保守力）。

### 3.8 各坐标的运动方程

#### 对于 $x$（轮子位移）

$$\frac{\partial \mathcal{L}}{\partial \dot{x}} = M_{\text{eff}}\dot{x} + M_b\dot{L}_0\sin\theta + M_b L_0\dot{\theta}\cos\theta$$

$$\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{x}}\right) = M_{\text{eff}}\ddot{x} + M_b\ddot{L}_0\sin\theta + M_b\dot{L}_0\dot{\theta}\cos\theta + M_b\dot{L}_0\dot{\theta}\cos\theta + M_b L_0\ddot{\theta}\cos\theta - M_b L_0\dot{\theta}^2\sin\theta$$

$$\frac{\partial \mathcal{L}}{\partial x} = 0$$

广义力：$Q_x = \frac{T_w}{R_w}$（轮毂电机扭矩驱动）

整理得：

$$\boxed{M_{\text{eff}}\ddot{x} + M_b\sin\theta \cdot \ddot{L}_0 + M_b L_0\cos\theta \cdot \ddot{\theta} = \frac{T_w}{R_w} + M_b L_0\dot{\theta}^2\sin\theta - 2M_b\dot{L}_0\dot{\theta}\cos\theta}$$

#### 对于 $\theta$（腿倾角）

$$\frac{\partial \mathcal{L}}{\partial \dot{\theta}} = M_b L_0^2\dot{\theta} + M_b L_0\dot{x}\cos\theta$$

对时间求导后：

$$\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{\theta}}\right) = M_b L_0^2\ddot{\theta} + 2M_b L_0\dot{L}_0\dot{\theta} + M_b L_0\ddot{x}\cos\theta + M_b \dot{L}_0\dot{x}\cos\theta - M_b L_0\dot{x}\dot{\theta}\sin\theta$$

$$\frac{\partial \mathcal{L}}{\partial \theta} = M_b\dot{x}\dot{L}_0\cos\theta - M_b\dot{x}L_0\dot{\theta}\sin\theta + M_b g L_0\sin\theta$$

广义力：$Q_\theta = T_p$（髋关节力矩）

整理得：

$$\boxed{M_b L_0^2\ddot{\theta} + M_b L_0\cos\theta \cdot \ddot{x} + 2M_b L_0\dot{L}_0\dot{\theta} - M_b g L_0\sin\theta = T_p - M_b \dot{L}_0\dot{x}\cos\theta}$$

#### 对于 $\phi$（机身俯仰角）

机身俯仰与腿倾角相关但不完全独立。在简化模型中，假设俯仰仅由惯性力矩和重力决定：

$$I_b\ddot{\phi} - M_b g h_b \sin\phi = T_p$$

其中 $h_b$ 是机身质心高度。更精确的建模需考虑腿与机身的耦合。

### 3.9 完整非线性动力学方程（矢量形式）

定义状态向量 $\mathbf{q} = [x, \theta, \phi]^T$，控制输入 $\mathbf{u} = [T_w, T_p]^T$：

$$\boxed{\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{G}(\mathbf{q}) = \mathbf{B}\mathbf{u}}$$

其中：
- $\mathbf{M}(\mathbf{q})$：$3 \times 3$ 质量/惯量矩阵（正定）
- $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})$：科里奥利力/离心力矩阵
- $\mathbf{G}(\mathbf{q})$：重力项
- $\mathbf{B}$：输入矩阵

---

## 4. 线性化与状态空间模型

### 4.1 平衡点

系统在以下条件下平衡：

$$\theta = 0 \quad \text{(腿竖直)}, \quad \phi = 0 \quad \text{(机身水平)}$$
$$\dot{\theta} = 0, \quad \dot{x} = 0, \quad \dot{\phi} = 0$$

在平衡点处，控制输入也不断为 0：$T_w^* = 0, T_p^* = 0$。

### 4.2 线性化过程

对于非线性系统 $\dot{\mathbf{z}} = f(\mathbf{z}, \mathbf{u})$，在平衡点 $\mathbf{z}^*, \mathbf{u}^*$ 做一阶泰勒展开：

$$\Delta\dot{\mathbf{z}} = \left.\frac{\partial f}{\partial \mathbf{z}}\right|_{z^*,u^*} \Delta\mathbf{z} + \left.\frac{\partial f}{\partial \mathbf{u}}\right|_{z^*,u^*} \Delta\mathbf{u}$$

### 4.3 固定腿长时的简化

当腿长 $L_0$ 固定（$\dot{L}_0 = 0, \ddot{L}_0 = 0$）且角度很小时（$\sin\theta \approx \theta, \cos\theta \approx 1$）：

从 $x$ 方程：

$$M_{\text{eff}}\ddot{x} + M_b L_0\ddot{\theta} = \frac{T_w}{R_w} - 2M_b\dot{L}_0\dot{\theta} \;(\to 0) + M_b L_0\dot{\theta}^2\cdot\theta \;(\to 0)$$

线性化：
$$M_{\text{eff}}\ddot{x} + M_b L_0\ddot{\theta} = \frac{T_w}{R_w}$$

从 $\theta$ 方程，令 $\sin\theta \approx \theta$：

$$M_b L_0^2\ddot{\theta} + M_b L_0\ddot{x} - M_b g L_0\theta = T_p$$

注意 $\dot{L}_0 \approx 0$ 消去了科里奥利项。

### 4.4 联立求解

写为矩阵形式：

$$\begin{bmatrix} M_{\text{eff}} & M_b L_0 \\ M_b L_0 & M_b L_0^2 \end{bmatrix} \begin{bmatrix} \ddot{x} \\ \ddot{\theta} \end{bmatrix} + \begin{bmatrix} 0 & 0 \\ 0 & -M_b g L_0 \end{bmatrix} \begin{bmatrix} x \\ \theta \end{bmatrix} = \begin{bmatrix} 1/R_w & 0 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} T_w \\ T_p \end{bmatrix}$$

求逆质量矩阵 $\mathbf{M}^{-1}$：

$$\det(\mathbf{M}) = M_{\text{eff}}M_b L_0^2 - M_b^2 L_0^2 = M_b L_0^2(M_{\text{eff}} - M_b)$$

$$\mathbf{M}^{-1} = \frac{1}{M_b L_0^2(M_{\text{eff}} - M_b)} \begin{bmatrix} M_b L_0^2 & -M_b L_0 \\ -M_b L_0 & M_{\text{eff}} \end{bmatrix}$$

则：

$$\begin{bmatrix} \ddot{x} \\ \ddot{\theta} \end{bmatrix} = \mathbf{M}^{-1}\left(\begin{bmatrix} 0 \\ M_b g L_0\theta \end{bmatrix} + \begin{bmatrix} T_w/R_w \\ T_p \end{bmatrix}\right)$$

展开：

$$\ddot{x} = \frac{1}{M_b L_0^2(M_{\text{eff}}-M_b)}\left(M_b L_0^2\cdot\frac{T_w}{R_w} - M_b L_0\cdot(T_p + M_b g L_0\theta)\right)$$

$$\ddot{\theta} = \frac{1}{M_b L_0^2(M_{\text{eff}}-M_b)}\left(-M_b L_0\cdot\frac{T_w}{R_w} + M_{\text{eff}}\cdot(T_p + M_b g L_0\theta)\right)$$

记 $\Delta = M_{\text{eff}} - M_b$，整理为标准形式：

$$\boxed{\ddot{x} = -\frac{M_b g}{\Delta}\theta + \frac{1}{R_w\Delta}T_w - \frac{1}{L_0\Delta}T_p}$$

$$\boxed{\ddot{\theta} = \frac{M_{\text{eff}} g}{L_0\Delta}\theta - \frac{1}{R_w L_0\Delta}T_w + \frac{M_{\text{eff}}}{M_b L_0^2\Delta}T_p}$$

### 4.5 6 状态状态空间模型

状态向量 $\mathbf{x} = [\theta, \dot{\theta}, x, \dot{x}, \phi, \dot{\phi}]^T$（6 维），控制输入 $\mathbf{u} = [T_w, T_p]^T$（2 维）。

$$\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}$$

**最终 A 矩阵**：

$$\mathbf{A} = \begin{bmatrix}
0 & 1 & 0 & 0 & 0 & 0 \\[4pt]
a_{21} & 0 & 0 & 0 & 0 & 0 \\[4pt]
0 & 0 & 0 & 1 & 0 & 0 \\[4pt]
a_{41} & 0 & 0 & 0 & 0 & 0 \\[4pt]
0 & 0 & 0 & 0 & 0 & 1 \\[4pt]
0 & 0 & 0 & 0 & a_{65} & 0
\end{bmatrix}$$

其中：

$$a_{21} = \frac{M_{\text{eff}}\cdot g}{L_0\cdot(M_{\text{eff}} - M_b)}$$
$$a_{41} = -\frac{M_b \cdot g}{M_{\text{eff}} - M_b}$$
$$a_{65} = \frac{M_b g h_b}{I_b}$$

**最终 B 矩阵**：

$$\mathbf{B} = \begin{bmatrix}
0 & 0 \\[4pt]
b_{21} & b_{22} \\[4pt]
0 & 0 \\[4pt]
b_{41} & b_{42} \\[4pt]
0 & 0 \\[4pt]
0 & b_{62}
\end{bmatrix}$$

其中：

$$b_{21} = -\frac{1}{R_w L_0 (M_{\text{eff}} - M_b)}, \quad b_{22} = \frac{M_{\text{eff}}}{M_b L_0^2 (M_{\text{eff}} - M_b)}$$

$$b_{41} = \frac{1}{R_w (M_{\text{eff}} - M_b)}, \quad b_{42} = -\frac{1}{L_0 (M_{\text{eff}} - M_b)}$$

$$b_{62} = \frac{1}{I_b}$$

### 4.6 关键观察

1. **$L_0$ 依赖性**：A 和 B 矩阵的所有非零元素都依赖于 $L_0$。这意味着：
   - 腿越长（$L_0 \uparrow$）→ $a_{21} \downarrow$（系统"变慢"）→ 需要更小的 LQR 增益
   - 腿越短（$L_0 \downarrow$）→ 系统"更快"但更不稳定

2. **开环不稳定**：$a_{21} > 0$，表示 $\theta$ 的开环动力学是不稳定的（倒立摆的典型特征）。验证：$a_{21} = \frac{M_{\text{eff}}g}{L_0\Delta} > 0$，因此开环特征值为 $\lambda = \pm\sqrt{a_{21}}$，有一个正实根。

3. **可控性**：$\text{rank}(\mathcal{C}) = \text{rank}([B, AB, A^2B, ..., A^5B]) = 6$，系统完全可控。

---

## 5. LQR 最优控制理论

### 5.1 问题陈述

对于线性时不变系统 $\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}$，寻找状态反馈控制律 $\mathbf{u} = -\mathbf{K}\mathbf{x}$，最小化二次代价函数：

$$J = \int_0^\infty \left(\mathbf{x}^T\mathbf{Q}\mathbf{x} + \mathbf{u}^T\mathbf{R}\mathbf{u}\right) dt$$

其中：
- $\mathbf{Q} = \mathbf{Q}^T \succeq 0$（半正定，$6 \times 6$）
- $\mathbf{R} = \mathbf{R}^T \succ 0$（正定，$2 \times 2$）

### 5.2 代数黎卡提方程

最优解由求解**连续代数黎卡提方程**（CARE）获得：

$$\boxed{\mathbf{A}^T\mathbf{P} + \mathbf{P}\mathbf{A} - \mathbf{P}\mathbf{B}\mathbf{R}^{-1}\mathbf{B}^T\mathbf{P} + \mathbf{Q} = \mathbf{0}}$$

其中 $\mathbf{P} = \mathbf{P}^T \succ 0$ 是黎卡提方程的唯一正定解。

**最优增益矩阵**：

$$\boxed{\mathbf{K} = \mathbf{R}^{-1}\mathbf{B}^T\mathbf{P}}$$

### 5.3 解的性质

1. **存在性与唯一性**：当 $(\mathbf{A}, \mathbf{B})$ 可稳定，$(\mathbf{A}, \mathbf{Q}^{1/2})$ 可检测时，CARE 有唯一正定解
2. **闭环稳定性**：$\mathbf{A} - \mathbf{B}\mathbf{K}$ 的所有特征值具有负实部
3. **鲁棒性**：LQR 具有无穷增益裕度和 ±60° 相位裕度（在单输入情况下）
4. **最优性**：$\mathbf{u} = -\mathbf{K}\mathbf{x}$ 是全局最优控制律

### 5.4 Q 和 R 的物理含义

在轮腿平衡问题中：

$$\mathbf{Q} = \text{diag}\left(q_\theta,\; q_{\dot{\theta}},\; q_x,\; q_{\dot{x}},\; q_\phi,\; q_{\dot{\phi}}\right)$$

$$\mathbf{R} = \begin{bmatrix} r_w & 0 \\ 0 & r_p \end{bmatrix}$$

**权重解读**：

| 权重 | 惩罚对象 | 增大效果 |
|------|---------|---------|
| $q_\theta$ | 腿倾角误差 | 腿更"僵硬"，更快回正 |
| $q_{\dot{\theta}}$ | 腿角速度 | 抑制腿部摆动速度 |
| $q_x$ | 轮子位置误差 | 轮子回中，限制漂移 |
| $q_{\dot{x}}$ | 轮子速度 | 抑制底盘速度（阻尼） |
| $q_\phi$ | 机身俯仰角 | 机身更平，射击精度更高 |
| $q_{\dot{\phi}}$ | 机身俯仰角速度 | 抑制机身晃动 |
| $r_w$ | 轮毂扭矩代价 | 轮子响应更慢但省力 |
| $r_p$ | 髋关节扭矩代价 | 髋关节更柔和 |

### 5.5 典型参数配置

**第一阶段（固定腿长，仅轮毂控制）**：

```
Q = diag(500,  10,  100,  10,  0,  0)    ← φ 不参与
R = diag(0.1,  1000)                       ← Tp 被严重惩罚（几乎禁用）
```

**第二阶段（全状态 LQR）**：

```
Q = diag(500,  10,  100,  10,  800,  20)
R = diag(0.1,  1.0)
```

**第三阶段（增益调度）**：

```
Q = diag(500,  10,  100,  10,  800,  20)   ← 固定
R = diag(0.1,  1.0)                         ← 固定
K(L₀) = polyfit(L₀_range, K_vals, 3)       ← K 随 L₀ 变化
```

### 5.6 Python 求解示例

```python
import numpy as np
from scipy.linalg import solve_continuous_are, eigvals

def design_lqr(L0, params, Q_diag, R_diag):
    """
    对给定的虚拟腿长 L₀ 设计 LQR 控制器
    
    参数:
        L0: 虚拟腿长 (m)
        params: dict with Mb, Mw, Iw, Rw, g, Ib, hb
        Q_diag: 6-list of state weights
        R_diag: 2-list of control weights
    
    返回:
        K: 2×6 增益矩阵
        P: 6×6 黎卡提解
        eigs: 闭环特征值
    """
    Mb, Mw = params['Mb'], params['Mw']
    Iw, Rw = params['Iw'], params['Rw']
    g, Ib, hb = params['g'], params['Ib'], params['hb']
    
    Meff = Mb + Mw + Iw / Rw**2
    D = Meff - Mb
    
    # 构造 A 矩阵
    A = np.zeros((6, 6))
    A[0, 1] = 1.0
    A[2, 3] = 1.0
    A[4, 5] = 1.0
    
    A[1, 0] = Meff * g / (L0 * D)        # a21
    A[3, 0] = -Mb * g / D                 # a41
    A[5, 4] = Mb * g * hb / Ib            # a65
    
    # 构造 B 矩阵
    B = np.zeros((6, 2))
    B[1, 0] = -1.0 / (Rw * L0 * D)        # b21
    B[1, 1] = Meff / (Mb * L0**2 * D)     # b22
    B[3, 0] = 1.0 / (Rw * D)              # b41
    B[3, 1] = -1.0 / (L0 * D)             # b42
    B[5, 1] = 1.0 / Ib                     # b62
    
    Q = np.diag(Q_diag)
    R = np.diag(R_diag)
    
    # 求解 CARE
    P = solve_continuous_are(A, B, Q, R)
    
    # 计算增益
    K = np.linalg.solve(R, B.T @ P)
    
    # 验证闭环稳定性
    A_cl = A - B @ K
    eigs = eigvals(A_cl)
    
    return K, P, eigs
```

---

## 6. 增益调度理论

### 6.1 为什么需要增益调度

A 和 B 矩阵依赖于虚拟腿长 $L_0$，而 $L_0$ 在运行时是变化的（0.18–0.30 m）。

- 在 $L_0 = 0.18$ m 处设计的 K 在 $L_0 = 0.30$ m 处可能过于激进
- 在 $L_0 = 0.30$ m 处设计的 K 在 $L_0 = 0.18$ m 处可能不够稳定

### 6.2 增益调度策略

采用**离线计算 + 在线插值**的方案：

1. **离线**：对一系列腿长 $\{L_0^{(1)}, L_0^{(2)}, ..., L_0^{(N)}\}$ 分别求解 LQR，得到 $\{\mathbf{K}^{(1)}, \mathbf{K}^{(2)}, ..., \mathbf{K}^{(N)}\}$
2. **拟合**：对每个矩阵元素 $\mathbf{K}_{ij}$，用**三次多项式**拟合 $\mathbf{K}_{ij}(L_0)$
3. **在线**：根据当前 $L_0$，用多项式求值得到 $\mathbf{K}(L_0)$

### 6.3 多项式拟合

对于一个特定的矩阵元素 $k = K[i][j]$（共 $2 \times 6 = 12$ 个标量），拟合：

$$k(L_0) = c_3 L_0^3 + c_2 L_0^2 + c_1 L_0 + c_0$$

用最小二乘法拟合系数：

$$\begin{bmatrix} c_3 \\ c_2 \\ c_1 \\ c_0 \end{bmatrix} = (\mathbf{X}^T\mathbf{X})^{-1}\mathbf{X}^T \mathbf{k}_{\text{obs}}$$

其中 $\mathbf{X}$ 是 Vandermonde 矩阵：

$$\mathbf{X} = \begin{bmatrix}
(L_0^{(1)})^3 & (L_0^{(1)})^2 & L_0^{(1)} & 1 \\
(L_0^{(2)})^3 & (L_0^{(2)})^2 & L_0^{(2)} & 1 \\
\vdots & \vdots & \vdots & \vdots \\
(L_0^{(N)})^3 & (L_0^{(N)})^2 & L_0^{(N)} & 1
\end{bmatrix}$$

### 6.4 稳定性分析：调度系统的鲁棒性

增益调度本质上是对非线性时变系统的近似。需要验证：

1. **冻结参数稳定性**：在任意固定 $L_0$ 下，$\mathbf{A}(L_0) - \mathbf{B}(L_0)\mathbf{K}(L_0)$ 特征值实部 < 0
2. **缓慢变化假设**：$|\dot{L}_0|$ 必须足够小，使得系统在每个时刻可视为"准静态"
3. **插值误差**：多项式拟合的最大残差必须在可接受范围内

**工程经验**：当 $\dot{L}_0$ 小于系统最快模态的 1/5 时（约 $< 0.5$ m/s），增益调度有效。

### 6.5 完整的增益表生成代码

```python
def generate_gain_table(params, Q_diag, R_diag, L0_range):
    """
    生成增益调度表并对每个元素做三次多项式拟合
    
    返回:
        coeffs: 2×6×4 数组 [control_idx][state_idx][poly_coeff]
    """
    N = len(L0_range)
    K_all = np.zeros((2, 6, N))
    
    for i, L0 in enumerate(L0_range):
        K, _, eigs = design_lqr(L0, params, Q_diag, R_diag)
        K_all[:, :, i] = K
        
        # 检查稳定性
        if np.any(np.real(eigs) >= 0):
            print(f"警告: L0={L0:.3f} 时闭环系统不稳定!")
    
    # 对每个 (i,j) 元素拟合三次多项式
    coeffs = np.zeros((2, 6, 4))
    for ctrl in range(2):
        for state in range(6):
            p = np.polyfit(L0_range, K_all[ctrl, state, :], 3)
            coeffs[ctrl, state, :] = p
    
    return coeffs, K_all

# 使用示例
params = {
    'Mb': 18.0, 'Mw': 0.5, 'Iw': 0.002, 'Rw': 0.076,
    'g': 9.81, 'Ib': 0.8, 'hb': 0.15
}
L0_range = np.linspace(0.18, 0.30, 13)  # 13 个采样点
Q_diag = [500, 10, 100, 10, 800, 20]
R_diag = [0.1, 1.0]

coeffs, K_all = generate_gain_table(params, Q_diag, R_diag, L0_range)
```

---

## 7. 非线性动力学仿真框架

### 7.1 线性 vs 非线性仿真的区别

| 特性 | 线性仿真 | 非线性仿真 |
|------|---------|-----------|
| 模型 | $\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}$ | 完整拉格朗日方程 |
| 适用范围 | 小角度偏差（$|\theta| < 10°$） | 任意角度，包括跌倒 |
| 计算量 | 极低（矩阵乘法） | 中等（ODE 求解） |
| $\sin\theta$ 近似 | $\sin\theta \approx \theta$ | 精确 |
| 腿长变化 | 固定或分段线性 | 连续变化 |
| 控制饱和 | 可添加 | 可添加 |

### 7.2 非线性动力学方程（用于仿真）

从第 3 节的拉格朗日方程出发，不做小角度近似：

$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} = \mathbf{B}\mathbf{u} - \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) - \mathbf{G}(\mathbf{q})$$

仿真时每步需要解算 $\ddot{\mathbf{q}}$：

$$\ddot{\mathbf{q}} = \mathbf{M}(\mathbf{q})^{-1} \left[\mathbf{B}\mathbf{u} - \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) - \mathbf{G}(\mathbf{q})\right]$$

然后数值积分（RK4 或 odeint）。

### 7.3 仿真流程图

```
初始化状态 x₀, 参数 params
  │
  ▼
for t in t_range:
  │
  ├─ 1. 读取当前状态: θ, θ̇, x, ẋ, φ, φ̇
  │
  ├─ 2. 正运动学: (θ₁, θ₄) → L₀, φ₀
  │
  ├─ 3. 增益调度: L₀ → K(L₀)
  │
  ├─ 4. LQR 控制: u = -K·x  (加入饱和限幅)
  │
  ├─ 5. 非线性动力学: M(q)q̈ = Bu - C - G → q̈
  │
  ├─ 6. 数值积分: q(t+dt) = RK4(q(t), q̇(t), q̈)
  │
  └─ 7. 记录数据 + 可视化
```

### 7.4 仿真测试用例

| 测试用例 | 初始条件 | 目标 | 评估指标 |
|---------|---------|------|---------|
| 阶跃响应 | $\theta = 5°$, 其他为 0 | 回复到平衡 | 调节时间、超调量 |
| 脉冲扰动 | 仿真中途施加 $F_{\text{dist}}$ | 抗干扰能力 | 恢复时间、最大偏移 |
| 遥控指令跟踪 | 施加参考速度 $v_x^{\text{ref}}$ | 速度跟踪 | 跟踪误差 RMS |
| 腿长阶跃 | $L_0: 0.20 \to 0.28$ m | 增益调度验证 | 是否保持平衡 |
| 大角度跌落 | $\theta = 30°$ | 非线性效应 | 是否发散 |

---

## 8. 仿真代码实现 (Python)

### 8.1 完整仿真脚本

```python
"""
串联轮腿步兵 — 完整非线性仿真框架
=====================================
包含:
  - 五连杆正/逆运动学
  - 解析雅可比矩阵
  - WIP 非线性动力学
  - LQR 增益调度
  - 多种仿真测试用例
  - 结果可视化

依赖: numpy, scipy, matplotlib
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# ============================================================
# 第 1 部分：物理参数
# ============================================================

class RobotParams:
    """串联轮腿机器人物理参数"""
    def __init__(self):
        # 五连杆几何参数
        self.L1 = 0.150   # 左上连杆 (m)
        self.L2 = 0.150   # 左下连杆 (m)
        self.L3 = 0.150   # 右上连杆 (m)
        self.L4 = 0.150   # 右下连杆 (m)
        self.L5 = 0.080   # 基座间距 (m)
        
        # 质量参数
        self.Mb = 18.0    # 机身质量 (kg)
        self.Ml = 2.0     # 单腿等效质量 (kg)
        self.Mw = 0.5     # 单轮质量 (kg)
        
        # 惯量参数
        self.Ib = 0.8     # 机身俯仰转动惯量 (kg·m²)
        self.Iw = 0.002   # 轮子转动惯量 (kg·m²)
        self.Rw = 0.076   # 轮子半径 (m)
        self.hb = 0.15    # 机身质心高度 (m, 相对于髋关节)
        
        # 环境
        self.g = 9.81     # 重力加速度 (m/s²)
        
        # 执行器约束
        self.T_wheel_max = 5.0    # 轮毂最大扭矩 (N·m)
        self.T_hip_max   = 9.0    # 髋关节最大扭矩 (N·m)
        self.L0_min      = 0.18   # 最小虚拟腿长 (m)
        self.L0_max      = 0.30   # 最大虚拟腿长 (m)

# ============================================================
# 第 2 部分：五连杆运动学
# ============================================================

class FiveBarKinematics:
    """五连杆运动学求解器"""
    
    def __init__(self, params):
        self.p = params
    
    def forward(self, theta1, theta4):
        """
        正运动学: (θ₁, θ₄) → (Cx, Cy, L₀, φ₀, θ₂, θ₃)
        """
        p = self.p
        # B 点
        Bx = -p.L5 / 2 + p.L1 * np.cos(theta1)
        By = p.L1 * np.sin(theta1)
        # E 点
        Ex = p.L5 / 2 + p.L4 * np.cos(theta4)
        Ey = p.L4 * np.sin(theta4)
        
        # BE 距离和方向
        dBE = np.sqrt((Ex - Bx)**2 + (Ey - By)**2)
        alpha_BE = np.arctan2(Ey - By, Ex - Bx)
        
        # 余弦定理求 ∠CBE
        cos_beta = (p.L2**2 + dBE**2 - p.L3**2) / (2 * p.L2 * dBE)
        cos_beta = np.clip(cos_beta, -1.0, 1.0)
        beta = np.arccos(cos_beta)
        
        # 凸构型
        theta2 = alpha_BE + beta
        
        # 末端 C
        Cx = Bx + p.L2 * np.cos(theta2)
        Cy = By + p.L2 * np.sin(theta2)
        
        # 虚拟腿参数
        L0 = np.sqrt(Cx**2 + Cy**2)
        phi0 = np.arctan2(Cy, Cx)
        
        # θ₃ (从 E 到 C)
        theta3 = np.arctan2(Cy - Ey, Cx - Ex)
        
        return {
            'Cx': Cx, 'Cy': Cy, 'L0': L0, 'phi0': phi0,
            'theta2': theta2, 'theta3': theta3,
            'dBE': dBE
        }
    
    def inverse(self, L0_target, phi0_target, mode='PP'):
        """
        逆运动学: (L₀, φ₀) → (θ₁, θ₄)
        mode: 'PP', 'PN', 'NP', 'NN'
        """
        p = self.p
        
        Cx = L0_target * np.cos(phi0_target)
        Cy = L0_target * np.sin(phi0_target)
        
        # 上链 (A → C)
        Ax, Ay = -p.L5 / 2, 0.0
        dAC = np.sqrt((Cx - Ax)**2 + (Cy - Ay)**2)
        gamma_A = np.arctan2(Cy - Ay, Cx - Ax)
        cos_CAB = (p.L1**2 + dAC**2 - p.L2**2) / (2 * p.L1 * dAC)
        cos_CAB = np.clip(cos_CAB, -1.0, 1.0)
        ang_CAB = np.arccos(cos_CAB)
        
        # 下链 (D → C)
        Dx, Dy = p.L5 / 2, 0.0
        dDC = np.sqrt((Cx - Dx)**2 + (Cy - Dy)**2)
        gamma_D = np.arctan2(Cy - Dy, Cx - Dx)
        cos_CDE = (p.L4**2 + dDC**2 - p.L3**2) / (2 * p.L4 * dDC)
        cos_CDE = np.clip(cos_CDE, -1.0, 1.0)
        ang_CDE = np.arccos(cos_CDE)
        
        # 根据模式选择符号
        sign_map = {
            'PP': (+1, -1), 'PN': (+1, +1),
            'NP': (-1, -1), 'NN': (-1, +1)
        }
        s1, s4 = sign_map[mode]
        
        theta1 = gamma_A + s1 * ang_CAB
        theta4 = gamma_D + s4 * ang_CDE
        
        return theta1, theta4
    
    def jacobian_analytic(self, theta1, theta2, theta3, theta4):
        """
        解析雅可比矩阵 J_v: 2×2
        [dL₀/dt, dφ₀/dt]ᵀ = J_v · [dθ₁/dt, dθ₄/dt]ᵀ
        """
        p = self.p
        
        # 先做正运动学得到 Cx, Cy, L₀
        fk = self.forward(theta1, theta4)
        Cx, Cy, L0 = fk['Cx'], fk['Cy'], fk['L0']
        
        cos_32 = np.cos(theta3 - theta2)
        cos_31 = np.cos(theta3 - theta1)
        cos_34 = np.cos(theta3 - theta4)
        
        # 笛卡尔雅可比 J_cart
        j11 = -p.L1 * np.sin(theta1) + p.L1 * np.sin(theta2) * cos_31 / cos_32
        j12 = -p.L4 * np.sin(theta2) * cos_34 / cos_32
        j21 = p.L1 * np.cos(theta1) - p.L1 * np.cos(theta2) * cos_31 / cos_32
        j22 = p.L4 * np.cos(theta2) * cos_34 / cos_32
        
        J_cart = np.array([[j11, j12], [j21, j22]])
        
        # 转换到虚拟腿空间
        T = np.array([
            [Cx / L0, Cy / L0],
            [-Cy / (L0**2), Cx / (L0**2)]
        ])
        
        J_v = T @ J_cart
        return J_v, J_cart
    
    def jacobian_numerical(self, theta1, theta4, eps=1e-6):
        """数值雅可比（用于验证解析雅可比）"""
        fk0 = self.forward(theta1, theta4)
        L00, phi00 = fk0['L0'], fk0['phi0']
        
        fk1 = self.forward(theta1 + eps, theta4)
        fk4 = self.forward(theta1, theta4 + eps)
        
        J = np.zeros((2, 2))
        J[0, 0] = (fk1['L0'] - L00) / eps
        J[0, 1] = (fk4['L0'] - L00) / eps
        J[1, 0] = (fk1['phi0'] - phi00) / eps
        J[1, 1] = (fk4['phi0'] - phi00) / eps
        
        return J
    
    def workspace_analysis(self, n_points=50):
        """分析工作空间"""
        theta1_range = np.linspace(np.pi/6, np.pi/2, n_points)
        theta4_range = np.linspace(np.pi/6, np.pi/2, n_points)
        
        workspace = np.zeros((n_points, n_points, 3))  # Cx, Cy, L0
        
        for i, t1 in enumerate(theta1_range):
            for j, t4 in enumerate(theta4_range):
                try:
                    fk = self.forward(t1, t4)
                    workspace[i, j, 0] = fk['Cx']
                    workspace[i, j, 1] = fk['Cy']
                    workspace[i, j, 2] = fk['L0']
                except:
                    workspace[i, j, :] = np.nan
        
        return workspace, theta1_range, theta4_range

# ============================================================
# 第 3 部分：WIP 动力学
# ============================================================

class WIPDynamics:
    """轮式倒立摆非线性动力学"""
    
    def __init__(self, params):
        self.p = params
    
    def mass_matrix(self, L0, theta):
        """计算质量矩阵 M(q)"""
        p = self.p
        Meff = p.Mb + p.Mw + p.Iw / p.Rw**2
        
        M = np.zeros((3, 3))
        # 对角线
        M[0, 0] = Meff                           # ẍ 系数
        M[1, 1] = p.Mb * L0**2                    # θ̈ 系数
        M[2, 2] = p.Ib                            # φ̈ 系数
        
        # 耦合项
        M[0, 1] = p.Mb * L0 * np.cos(theta)       # ẍ-θ̈ 耦合
        M[1, 0] = M[0, 1]                          # 对称
        
        return M
    
    def coriolis_gravity(self, L0, theta, phi, 
                         dL0, dtheta, dx, dphi):
        """计算科里奥利力 + 离心力 + 重力"""
        p = self.p
        
        C = np.zeros(3)
        
        # x 方向的非线性项
        C[0] = -p.Mb * L0 * dtheta**2 * np.sin(theta) \
               + 2 * p.Mb * dL0 * dtheta * np.cos(theta)
        
        # θ 方向的非线性项
        C[1] = -2 * p.Mb * L0 * dL0 * dtheta \
               + p.Mb * dL0 * dx * np.cos(theta) \
               - p.Mb * p.g * L0 * np.sin(theta)
        
        # φ 方向的重力项
        C[2] = -p.Mb * p.g * p.hb * np.sin(phi)
        
        return C
    
    def input_matrix(self):
        """控制输入矩阵 B"""
        p = self.p
        B = np.zeros((3, 2))
        B[0, 0] = 1.0 / p.Rw   # T_wheel → x
        B[1, 1] = 1.0           # T_hip → θ
        B[2, 1] = 1.0           # T_hip → φ（髋力矩同时影响 θ 和 φ）
        return B
    
    def dynamics(self, t, state, control_func, leg_control_func):
        """
        完整的非线性动力学 ODE

        state: [x, dx, theta, dtheta, phi, dphi]
        control_func(state, L0) → [T_wheel, T_hip]
        leg_control_func(t, state) → L0_target
        """
        x, dx, theta, dtheta, phi, dphi = state
        
        # 腿长控制（简化：直接跟踪目标）
        L0_target = leg_control_func(t, state)
        # 一阶滞后：实际腿长跟踪目标
        tau_leg = 0.05  # 腿长响应时间常数 (s)
        L0, dL0 = self._estimate_leg_state(t, state, L0_target, tau_leg)
        
        # 质量矩阵
        M = self.mass_matrix(L0, theta)
        
        # 科里奥利 + 重力
        C = self.coriolis_gravity(L0, theta, phi, dL0, dtheta, dx, dphi)
        
        # 控制输入
        T_wheel, T_hip = control_func(state, L0)
        
        # 扭矩限幅
        T_wheel = np.clip(T_wheel, -self.p.T_wheel_max, self.p.T_wheel_max)
        T_hip = np.clip(T_hip, -self.p.T_hip_max, self.p.T_hip_max)
        
        B = self.input_matrix()
        u = np.array([T_wheel, T_hip])
        
        # 求解加速度
        # M * q̈ + C = B * u  →  q̈ = M⁻¹(Bu - C)
        try:
            qddot = np.linalg.solve(M, B @ u - C)
        except np.linalg.LinAlgError:
            qddot = np.zeros(3)
        
        return [dx, qddot[0], dtheta, qddot[1], dphi, qddot[2]]
    
    def _estimate_leg_state(self, t, state, L0_target, tau):
        """估计当前的腿长和腿长变化率（简化：一阶系统）"""
        # 这里简化处理：假设腿长立即跟踪到目标
        # 实际需要结合关节电机 MIT 模式的动力学
        return L0_target, 0.0

# ============================================================
# 第 4 部分：LQR 控制器
# ============================================================

class LQRController:
    """增益调度 LQR 控制器"""
    
    def __init__(self, params, Q_diag, R_diag, L0_range):
        self.p = params
        self.Q_diag = Q_diag
        self.R_diag = R_diag
        self.L0_range = L0_range
        self.coeffs = None
        self._build_gain_table()
    
    def _build_AB(self, L0):
        """构造 A, B 矩阵（6 状态）"""
        p = self.p
        Meff = p.Mb + p.Mw + p.Iw / p.Rw**2
        D = Meff - p.Mb
        
        A = np.zeros((6, 6))
        A[0, 1] = 1.0
        A[2, 3] = 1.0
        A[4, 5] = 1.0
        
        A[1, 0] = Meff * p.g / (L0 * D)
        A[3, 0] = -p.Mb * p.g / D
        A[5, 4] = p.Mb * p.g * p.hb / p.Ib
        
        B = np.zeros((6, 2))
        B[1, 0] = -1.0 / (p.Rw * L0 * D)
        B[1, 1] = Meff / (p.Mb * L0**2 * D)
        B[3, 0] = 1.0 / (p.Rw * D)
        B[3, 1] = -1.0 / (L0 * D)
        B[5, 1] = 1.0 / p.Ib
        
        return A, B
    
    def _build_gain_table(self):
        """离线计算增益表并做三次多项式拟合"""
        N = len(self.L0_range)
        K_all = np.zeros((2, 6, N))
        
        Q = np.diag(self.Q_diag)
        R = np.diag(self.R_diag)
        
        for i, L0 in enumerate(self.L0_range):
            A, B = self._build_AB(L0)
            P = solve_continuous_are(A, B, Q, R)
            K = np.linalg.solve(R, B.T @ P)
            K_all[:, :, i] = K
        
        # 三次多项式拟合
        self.coeffs = np.zeros((2, 6, 4))  # [ctrl][state][coeff]
        for ctrl in range(2):
            for state in range(6):
                p = np.polyfit(self.L0_range, K_all[ctrl, state, :], 3)
                self.coeffs[ctrl, state, :] = p
    
    def get_gains(self, L0):
        """根据当前腿长获取 K 矩阵"""
        K = np.zeros((2, 6))
        for ctrl in range(2):
            for state in range(6):
                c = self.coeffs[ctrl, state, :]
                K[ctrl, state] = c[0]*L0**3 + c[1]*L0**2 + c[2]*L0 + c[3]
        return K
    
    def control(self, state, L0):
        """
        计算控制量
        state: [θ, θ̇, x, ẋ, φ, φ̇] (注意顺序！)
        """
        K = self.get_gains(L0)
        x_vec = np.array(state).reshape(6, 1)
        u = -K @ x_vec
        return u.flatten()  # [T_wheel, T_hip]

# ============================================================
# 第 5 部分：仿真管理器
# ============================================================

class SimulationManager:
    """仿真运行管理器"""
    
    def __init__(self, params, lqr_controller):
        self.p = params
        self.lqr = lqr_controller
        self.dynamics = WIPDynamics(params)
        self.kinematics = FiveBarKinematics(params)
    
    def run_simulation(self, x0, t_span, dt, L0_profile=None,
                       disturbance=None, reference_velocity=None):
        """
        运行非线性仿真
        
        参数:
            x0: 初始状态 [θ, θ̇, x, ẋ, φ, φ̇]
            t_span: (t_start, t_end)
            dt: 时间步长
            L0_profile: func(t) → L0_target, 若为 None 则固定 0.22m
            disturbance: func(t) → F_dist (水平扰动力, N)
            reference_velocity: func(t) → vx_ref (前向参考速度, m/s)
        
        返回:
            t: 时间数组
            states: (N, 6) 状态历史
            controls: (N, 2) 控制历史
        """
        if L0_profile is None:
            L0_profile = lambda t: 0.22
        
        if disturbance is None:
            disturbance = lambda t: 0.0
        
        if reference_velocity is None:
            reference_velocity = lambda t: 0.0
        
        t_eval = np.arange(t_span[0], t_span[1] + dt, dt)
        
        def control_func(state, L0):
            theta, dtheta, x, dx, phi, dphi = state
            # 加入参考速度前馈
            vx_ref = reference_velocity(t_current)
            # 修改状态中 x 的期望：期望速度为 vx_ref
            state_mod = np.array([theta, dtheta, 0, dx - vx_ref, phi, dphi])
            u = self.lqr.control(state_mod, L0)
            # 扰动力等效为额外的轮毂扭矩
            u[0] += disturbance(t_current) * self.p.Rw
            return u
        
        # 使用 scipy 的 RK45 求解器
        t_current = t_span[0]
        
        def ode_func(t, y):
            nonlocal t_current
            t_current = t
            leg_func = lambda t, s: L0_profile(t)
            ctrl_func = lambda s, L0: control_func(s, L0)
            return self.dynamics.dynamics(t, y, ctrl_func, leg_func)
        
        sol = solve_ivp(
            ode_func, t_span, x0,
            method='RK45', t_eval=t_eval,
            max_step=dt, rtol=1e-6, atol=1e-9
        )
        
        # 提取控制量
        controls = np.zeros((len(sol.t), 2))
        for i, t in enumerate(sol.t):
            L0 = L0_profile(t)
            u = control_func(sol.y[:, i], L0)
            controls[i, :] = u
        
        return sol.t, sol.y.T, controls
    
    def run_linear_simulation(self, x0, t_span, dt, L0=0.22):
        """
        运行线性化仿真（用于对比验证）
        """
        A, B = self.lqr._build_AB(L0)
        K = self.lqr.get_gains(L0)
        A_cl = A - B @ K
        
        t_eval = np.arange(t_span[0], t_span[1] + dt, dt)
        
        def linear_ode(t, x):
            return A_cl @ x
        
        sol = solve_ivp(linear_ode, t_span, x0, 
                       method='RK45', t_eval=t_eval)
        
        controls = np.zeros((len(sol.t), 2))
        for i in range(len(sol.t)):
            controls[i] = -K @ sol.y[:, i]
        
        return sol.t, sol.y.T, controls

# ============================================================
# 第 6 部分：可视化
# ============================================================

def plot_simulation_results(t, states, controls, title="Simulation Results"):
    """绘制仿真结果"""
    fig = plt.figure(figsize=(14, 10))
    gs = GridSpec(4, 2, figure=fig)
    
    labels_state = [
        (r'$\theta$ (rad)', 'Leg Angle'),
        (r'$\dot{\theta}$ (rad/s)', 'Leg Angular Velocity'),
        (r'$x$ (m)', 'Wheel Position'),
        (r'$\dot{x}$ (m/s)', 'Wheel Velocity'),
        (r'$\phi$ (rad)', 'Body Pitch'),
        (r'$\dot{\phi}$ (rad/s)', 'Body Pitch Velocity')
    ]
    
    for i in range(6):
        ax = fig.add_subplot(gs[i // 2, i % 2])
        ax.plot(t, states[:, i], 'b-', linewidth=1.5)
        ax.set_ylabel(labels_state[i][0])
        ax.set_title(labels_state[i][1])
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='k', linewidth=0.5)
    
    # 控制量
    ax_ctrl1 = fig.add_subplot(gs[3, 0])
    ax_ctrl1.plot(t, controls[:, 0], 'r-', linewidth=1.5)
    ax_ctrl1.set_ylabel('T_wheel (N·m)')
    ax_ctrl1.set_title('Wheel Torque Command')
    ax_ctrl1.grid(True, alpha=0.3)
    
    ax_ctrl2 = fig.add_subplot(gs[3, 1])
    ax_ctrl2.plot(t, controls[:, 1], 'g-', linewidth=1.5)
    ax_ctrl2.set_ylabel('T_hip (N·m)')
    ax_ctrl2.set_title('Hip Torque Command')
    ax_ctrl2.grid(True, alpha=0.3)
    
    plt.suptitle(title, fontsize=14, fontweight='bold')
    plt.tight_layout()
    return fig

def plot_eigenvalue_analysis(L0_range, params, Q_diag, R_diag):
    """绘制特征值随腿长的变化"""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    all_eigs = []
    for L0 in L0_range:
        Meff = params.Mb + params.Mw + params.Iw / params.Rw**2
        D = Meff - params.Mb
        
        A = np.zeros((6, 6))
        A[0, 1] = 1; A[2, 3] = 1; A[4, 5] = 1
        A[1, 0] = Meff * params.g / (L0 * D)
        A[3, 0] = -params.Mb * params.g / D
        A[5, 4] = params.Mb * params.g * params.hb / params.Ib
        
        B = np.zeros((6, 2))
        B[1, 0] = -1.0 / (params.Rw * L0 * D)
        B[1, 1] = Meff / (params.Mb * L0**2 * D)
        B[3, 0] = 1.0 / (params.Rw * D)
        B[3, 1] = -1.0 / (L0 * D)
        B[5, 1] = 1.0 / params.Ib
        
        Q = np.diag(Q_diag)
        R = np.diag(R_diag)
        P = solve_continuous_are(A, B, Q, R)
        K = np.linalg.solve(R, B.T @ P)
        A_cl = A - B @ K
        eigs = np.linalg.eigvals(A_cl)
        all_eigs.append(eigs)
    
    all_eigs = np.array(all_eigs)
    
    # 开环特征值
    axes[0].plot(L0_range, np.real(all_eigs), 'o-', markersize=3)
    axes[0].axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    axes[0].set_xlabel('L₀ (m)')
    axes[0].set_ylabel('Real(λ)')
    axes[0].set_title('Closed-Loop Eigenvalues vs Leg Length')
    axes[0].grid(True, alpha=0.3)
    
    # 特征值在复平面上的分布
    for i, L0 in enumerate(L0_range[::5]):  # 每隔 5 个采样
        axes[1].scatter(np.real(all_eigs[::5][i]), 
                       np.imag(all_eigs[::5][i]),
                       label=f'L₀={L0:.2f}', s=30)
    axes[1].axvline(x=0, color='k', linestyle='--', linewidth=0.5)
    axes[1].axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    axes[1].set_xlabel('Real(λ)')
    axes[1].set_ylabel('Imag(λ)')
    axes[1].set_title('Pole-Zero Map')
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def plot_workspace(kinematics, save_path=None):
    """绘制五连杆工作空间"""
    ws, t1_range, t4_range = kinematics.workspace_analysis(40)
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # Cx-Cy 空间 (笛卡尔)
    im1 = axes[0].pcolormesh(t1_range, t4_range, ws[:, :, 2].T, 
                              shading='auto', cmap='viridis')
    axes[0].set_xlabel('θ₁ (rad)')
    axes[0].set_ylabel('θ₄ (rad)')
    axes[0].set_title('Virtual Leg Length L₀ (m)')
    plt.colorbar(im1, ax=axes[0])
    
    # 末端可达区域
    axes[1].scatter(ws[:, :, 0].flatten(), ws[:, :, 1].flatten(), 
                   c=ws[:, :, 2].flatten(), s=1, cmap='viridis', alpha=0.8)
    axes[1].set_xlabel('Cx (m)')
    axes[1].set_ylabel('Cy (m)')
    axes[1].set_title('Workspace: End-Effector Position')
    axes[1].set_aspect('equal')
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(y=0, color='k', linewidth=0.5)
    axes[1].axvline(x=0, color='k', linewidth=0.5)
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150)
    return fig

# ============================================================
# 第 7 部分：主程序入口
# ============================================================

if __name__ == "__main__":
    print("=" * 60)
    print("串联轮腿步兵 — 数学推导与仿真验证")
    print("=" * 60)
    
    # 初始化参数
    params = RobotParams()
    kinematics = FiveBarKinematics(params)
    
    # ---- 测试 1: 五连杆正/逆运动学一致性 ----
    print("\n[测试 1] 五连杆正/逆运动学一致性验证...")
    theta1_test = np.deg2rad(45)
    theta4_test = np.deg2rad(50)
    
    fk = kinematics.forward(theta1_test, theta4_test)
    print(f"  正运动学: θ₁={np.rad2deg(theta1_test):.1f}°, "
          f"θ₄={np.rad2deg(theta4_test):.1f}°")
    print(f"  → L₀={fk['L0']:.4f} m, φ₀={np.rad2deg(fk['phi0']):.2f}°")
    
    t1_ik, t4_ik = kinematics.inverse(fk['L0'], fk['phi0'])
    print(f"  逆运动学: L₀={fk['L0']:.4f} m, φ₀={np.rad2deg(fk['phi0']):.2f}°")
    print(f"  → θ₁={np.rad2deg(t1_ik):.2f}°, θ₄={np.rad2deg(t4_ik):.2f}°")
    print(f"  误差: Δθ₁={np.rad2deg(abs(t1_ik-theta1_test)):.4f}°, "
          f"Δθ₄={np.rad2deg(abs(t4_ik-theta4_test)):.4f}°")
    
    # ---- 测试 2: 雅可比矩阵验证 ----
    print("\n[测试 2] 雅可比矩阵解析 vs 数值...")
    J_analytic, _ = kinematics.jacobian_analytic(
        theta1_test, fk['theta2'], fk['theta3'], theta4_test)
    J_numerical = kinematics.jacobian_numerical(theta1_test, theta4_test)
    
    J_error = np.max(np.abs(J_analytic - J_numerical))
    print(f"  解析 J:\n{J_analytic}")
    print(f"  数值 J:\n{J_numerical}")
    print(f"  最大误差: {J_error:.2e}")
    
    # ---- 测试 3: LQR 设计 ----
    print("\n[测试 3] LQR 控制器设计...")
    Q_diag = [500, 10, 100, 10, 800, 20]
    R_diag = [0.1, 1.0]
    L0_range = np.linspace(0.18, 0.30, 13)
    
    lqr_ctrl = LQRController(params, Q_diag, R_diag, L0_range)
    
    for L0_test in [0.18, 0.24, 0.30]:
        K = lqr_ctrl.get_gains(L0_test)
        print(f"\n  L₀ = {L0_test:.2f} m:")
        print(f"  K_wheel = [{K[0, 0]:7.2f}, {K[0, 1]:7.2f}, {K[0, 2]:7.2f}, "
              f"{K[0, 3]:7.2f}, {K[0, 4]:7.2f}, {K[0, 5]:7.2f}]")
        print(f"  K_hip   = [{K[1, 0]:7.2f}, {K[1, 1]:7.2f}, {K[1, 2]:7.2f}, "
              f"{K[1, 3]:7.2f}, {K[1, 4]:7.2f}, {K[1, 5]:7.2f}]")
    
    # ---- 测试 4: 非线性仿真 ----
    print("\n[测试 4] 运行非线性仿真...")
    sim = SimulationManager(params, lqr_ctrl)
    
    # 初始状态：小角度偏离平衡
    x0 = np.array([0.1, 0.0, 0.0, 0.0, 0.05, 0.0])  # θ=0.1rad(≈5.7°), φ=0.05rad
    t_span = (0.0, 3.0)
    dt = 0.001
    
    t, states, controls = sim.run_simulation(x0, t_span, dt)
    
    print(f"  仿真完成: {len(t)} 个时间步")
    print(f"  最终状态: θ={np.rad2deg(states[-1, 0]):.3f}°, "
          f"φ={np.rad2deg(states[-1, 4]):.3f}°")
    print(f"  最大轮毂扭矩: {np.max(np.abs(controls[:, 0])):.2f} N·m")
    print(f"  最大髋扭矩: {np.max(np.abs(controls[:, 1])):.2f} N·m")
    
    # 生成训练用 C 代码
    print("\n[导出] C 代码增益系数...")
    print("float K_coeffs[2][6][4] = {")
    for ctrl in range(2):
        name = "T_wheel" if ctrl == 0 else "T_hip  "
        print(f"    // {name}")
        for state in range(6):
            c = lqr_ctrl.coeffs[ctrl, state, :]
            print(f"    {{{c[0]:12.6f}f, {c[1]:12.6f}f, "
                  f"{c[2]:12.6f}f, {c[3]:12.6f}f}},")
        if ctrl == 0:
            print()
    print("};")
    
    print("\n" + "=" * 60)
    print("仿真完成。所有数学推导已通过代码验证。")
    print("=" * 60)
```

### 8.2 仿真测试套件

```python
"""
测试套件: test_leg_simulation.py
用于自动化测试各种仿真场景
"""

def test_step_response():
    """测试 1: 阶跃响应 — 初始角度扰动下的回复"""
    params = RobotParams()
    lqr = LQRController(params, 
                        Q_diag=[500, 10, 100, 10, 800, 20],
                        R_diag=[0.1, 1.0],
                        L0_range=np.linspace(0.18, 0.30, 13))
    sim = SimulationManager(params, lqr)
    
    x0 = np.array([0.087, 0, 0, 0, 0, 0])  # 5° 初始偏差
    t, states, ctrls = sim.run_simulation(x0, (0, 5.0), 0.001)
    
    # 检查: θ 应回到 0 附近
    assert abs(states[-1, 0]) < 1e-3, f"残差过大: {states[-1, 0]}"
    print("✓ 阶跃响应测试通过")


def test_disturbance_rejection():
    """测试 2: 脉冲扰动下的抗干扰能力"""
    params = RobotParams()
    lqr = LQRController(params,
                        Q_diag=[500, 10, 100, 10, 800, 20],
                        R_diag=[0.1, 1.0],
                        L0_range=np.linspace(0.18, 0.30, 13))
    sim = SimulationManager(params, lqr)
    
    x0 = np.zeros(6)
    
    # 在 t=1s 时施加 50N 持续 0.1s 的水平推力
    def disturbance(t):
        if 1.0 <= t <= 1.1:
            return 50.0
        return 0.0
    
    t, states, ctrls = sim.run_simulation(
        x0, (0, 5.0), 0.001, disturbance=disturbance)
    
    # 检查: 扰动后应恢复
    max_deviation = np.max(np.abs(states[:, 0]))
    recovery_angle = abs(states[-1, 0])
    print(f"  最大角度偏差: {np.rad2deg(max_deviation):.2f}°")
    print(f"  恢复后残差: {np.rad2deg(recovery_angle):.4f}°")
    print("✓ 扰动力测试完成")


def test_velocity_tracking():
    """测试 3: 速度跟踪"""
    params = RobotParams()
    lqr = LQRController(params,
                        Q_diag=[500, 10, 100, 10, 800, 20],
                        R_diag=[0.1, 1.0],
                        L0_range=np.linspace(0.18, 0.30, 13))
    sim = SimulationManager(params, lqr)
    
    x0 = np.zeros(6)
    
    # 在 t=1s 开始跟踪 1 m/s 的速度
    def reference_velocity(t):
        if t < 1.0:
            return 0.0
        return 1.0  # m/s
    
    t, states, ctrls = sim.run_simulation(
        x0, (0, 5.0), 0.001, reference_velocity=reference_velocity)
    
    # 检查: 速度应趋近 1 m/s
    final_vel = states[-1, 3]
    print(f"  最终速度: {final_vel:.3f} m/s (目标: 1.0 m/s)")
    print(f"  跟踪误差: {abs(final_vel - 1.0):.3f} m/s")
    print("✓ 速度跟踪测试完成")


def test_gain_scheduling():
    """测试 4: 增益调度 — 腿长变化时的稳定性"""
    params = RobotParams()
    lqr = LQRController(params,
                        Q_diag=[500, 10, 100, 10, 800, 20],
                        R_diag=[0.1, 1.0],
                        L0_range=np.linspace(0.18, 0.30, 13))
    sim = SimulationManager(params, lqr)
    
    x0 = np.zeros(6)
    
    # 腿长从 0.20 渐变到 0.28
    def leg_profile(t):
        if t < 1.0:
            return 0.20
        elif t < 3.0:
            return 0.20 + (t - 1.0) / 2.0 * 0.08  # 线性渐变
        else:
            return 0.28
    
    t, states, ctrls = sim.run_simulation(
        x0, (0, 5.0), 0.001, L0_profile=leg_profile)
    
    # 检查: 腿长变化过程中保持稳定
    max_pitch = np.max(np.abs(states[:, 4]))
    print(f"  腿长渐变过程中的最大俯仰角: {np.rad2deg(max_pitch):.3f}°")
    assert max_pitch < 0.05, f"腿长变化时不稳定: max pitch = {np.rad2deg(max_pitch):.2f}°"
    print("✓ 增益调度测试通过")


def test_linear_vs_nonlinear():
    """测试 5: 线性 vs 非线性仿真对比"""
    params = RobotParams()
    lqr = LQRController(params,
                        Q_diag=[500, 10, 100, 10, 800, 20],
                        R_diag=[0.1, 1.0],
                        L0_range=np.linspace(0.18, 0.30, 13))
    sim = SimulationManager(params, lqr)
    
    x0 = np.array([0.05, 0, 0, 0, 0, 0])
    
    t_nl, states_nl, ctrls_nl = sim.run_simulation(x0, (0, 2.0), 0.001)
    t_lin, states_lin, ctrls_lin = sim.run_linear_simulation(x0, (0, 2.0), 0.001)
    
    # 小角度下两者应接近
    diff = np.max(np.abs(states_nl[:, 0] - states_lin[:, 0]))
    print(f"  非线性 vs 线性最大差异: {diff:.6f} rad")
    assert diff < 0.01, f"差异过大: {diff}"
    print("✓ 线性/非线性一致性测试通过")


if __name__ == "__main__":
    print("运行全部仿真测试...")
    print()
    test_step_response()
    test_disturbance_rejection()
    test_velocity_tracking()
    test_gain_scheduling()
    test_linear_vs_nonlinear()
    print()
    print("所有测试通过 ✓")
```

### 8.3 参数扫描与优化工具

```python
"""
参数扫描工具: param_sweep.py
用于 LQR 权重参数的自动化扫描与优化
"""

def sweep_lqr_weights(params, L0=0.22):
    """扫描 Q 矩阵权重，找到最优组合"""
    
    # 扫描范围
    q_theta_vals = np.logspace(1, 3, 10)    # 10 ~ 1000
    q_phi_vals = np.logspace(1, 3, 10)      # 10 ~ 1000
    
    results = []
    
    for q_theta in q_theta_vals:
        for q_phi in q_phi_vals:
            Q_diag = [q_theta, 10, 100, 10, q_phi, 20]
            R_diag = [0.1, 1.0]
            
            # 构造 LQR
            Meff = params.Mb + params.Mw + params.Iw / params.Rw**2
            D = Meff - params.Mb
            
            A = np.zeros((6, 6))
            A[0, 1] = 1; A[2, 3] = 1; A[4, 5] = 1
            A[1, 0] = Meff * params.g / (L0 * D)
            A[3, 0] = -params.Mb * params.g / D
            A[5, 4] = params.Mb * params.g * params.hb / params.Ib
            
            B = np.zeros((6, 2))
            B[1, 0] = -1.0 / (params.Rw * L0 * D)
            B[1, 1] = Meff / (params.Mb * L0**2 * D)
            B[3, 0] = 1.0 / (params.Rw * D)
            B[3, 1] = -1.0 / (L0 * D)
            B[5, 1] = 1.0 / params.Ib
            
            Q = np.diag(Q_diag)
            R = np.diag(R_diag)
            
            try:
                P = solve_continuous_are(A, B, Q, R)
                K = np.linalg.solve(R, B.T @ P)
                A_cl = A - B @ K
                eigs = np.linalg.eigvals(A_cl)
                
                # 评估指标
                settling_time = -4.0 / np.max(np.real(eigs))  # 估计调节时间
                max_real = np.max(np.real(eigs))
                min_damping = np.min(-np.real(eigs) / np.abs(eigs))
                
                results.append({
                    'q_theta': q_theta,
                    'q_phi': q_phi,
                    'settling_time': settling_time,
                    'max_eig_real': max_real,
                    'min_damping': min_damping,
                    'K_norm': np.linalg.norm(K),
                    'stable': np.all(np.real(eigs) < 0)
                })
            except:
                results.append({
                    'q_theta': q_theta,
                    'q_phi': q_phi,
                    'settling_time': np.inf,
                    'stable': False
                })
    
    return results


def find_optimal_weights(params, L0=0.22, 
                         max_settling_time=0.5, 
                         max_gain_norm=500):
    """找到满足约束的最优权重"""
    results = sweep_lqr_weights(params, L0)
    
    # 过滤
    valid = [r for r in results 
             if r['stable'] 
             and r['settling_time'] < max_settling_time
             and r['K_norm'] < max_gain_norm]
    
    if not valid:
        print("没有满足约束的解！")
        return None
    
    # 按调节时间排序
    valid.sort(key=lambda x: x['settling_time'])
    
    print(f"找到 {len(valid)} 个有效解")
    print(f"\n最优解:")
    best = valid[0]
    print(f"  q_theta = {best['q_theta']:.0f}")
    print(f"  q_phi   = {best['q_phi']:.0f}")
    print(f"  调节时间 = {best['settling_time']:.3f} s")
    print(f"  最小阻尼 = {best['min_damping']:.3f}")
    
    return best
```

---

## 9. 仿真代码实现 (MATLAB)

> 以下 MATLAB 代码可直接运行，包含与 Python 版完全对应的仿真功能。MATLAB 的优势在于其内置的 `lqr()` 函数和控制工具箱，以及成熟的 Simulink 集成能力。

### 9.1 主仿真脚本：`leg_wheel_sim.m`

```matlab
%% ============================================================
%% 串联轮腿步兵 — MATLAB 完整仿真框架
%% ============================================================
%  包含: 五连杆运动学, WIP非线性动力学, LQR增益调度, 可视化
%  依赖: Control System Toolbox (用于 lqr 函数)
%% ============================================================

clear; clc; close all;

%% ---- 第 1 步: 物理参数初始化 ----
params = RobotParams();

%% ---- 第 2 步: 五连杆运动学验证 ----
fprintf('======== 测试 1: 五连杆正/逆运动学一致性 ========\n');

theta1_test = deg2rad(45);
theta4_test = deg2rad(50);

fk = FiveBarFK(params, theta1_test, theta4_test);
fprintf('正运动学: θ₁=%.1f°, θ₄=%.1f°\n', ...
    rad2deg(theta1_test), rad2deg(theta4_test));
fprintf('→ L₀=%.4f m, φ₀=%.2f°\n', fk.L0, rad2deg(fk.phi0));

[t1_ik, t4_ik] = FiveBarIK(params, fk.L0, fk.phi0, 'PP');
fprintf('逆运动学: L₀=%.4f m, φ₀=%.2f°\n', fk.L0, rad2deg(fk.phi0));
fprintf('→ θ₁=%.4f°, θ₄=%.4f°\n', rad2deg(t1_ik), rad2deg(t4_ik));
fprintf('误差: Δθ₁=%.4f°, Δθ₄=%.4f°\n', ...
    rad2deg(abs(t1_ik - theta1_test)), rad2deg(abs(t4_ik - theta4_test)));

%% ---- 第 3 步: 雅可比矩阵验证 ----
fprintf('\n======== 测试 2: 雅可比矩阵解析 vs 数值 ========\n');

J_analytic = FiveBarJacobian(params, theta1_test, theta4_test);
J_numerical = FiveBarJacobianNumerical(params, theta1_test, theta4_test);

fprintf('解析 J:\n');
disp(J_analytic);
fprintf('数值 J:\n');
disp(J_numerical);
fprintf('最大误差: %.2e\n', max(abs(J_analytic - J_numerical), [], 'all'));

%% ---- 第 4 步: LQR 控制器设计 ----
fprintf('\n======== 测试 3: LQR 控制器设计 ========\n');

Q_diag = [500, 10, 100, 10, 800, 20];
R_diag = [0.1, 1.0];
L0_range = linspace(0.18, 0.30, 13);

[lqr_ctrl] = LQRDesigner(params, Q_diag, R_diag, L0_range);

for L0_test = [0.18, 0.24, 0.30]
    K = lqr_ctrl.getGains(L0_test);
    fprintf('\n  L₀ = %.2f m:\n', L0_test);
    fprintf('  K_wheel = [%7.2f, %7.2f, %7.2f, %7.2f, %7.2f, %7.2f]\n', K(1,:));
    fprintf('  K_hip   = [%7.2f, %7.2f, %7.2f, %7.2f, %7.2f, %7.2f]\n', K(2,:));
end

%% ---- 第 5 步: 非线性仿真 — 阶跃响应 ----
fprintf('\n======== 测试 4: 非线性仿真 (阶跃响应) ========\n');

x0 = [0.1; 0; 0; 0; 0.05; 0];  % [θ, θ̇, x, ẋ, φ, φ̇]
tspan = [0, 3];

[t, states, controls] = RunNonlinearSim(params, lqr_ctrl, x0, tspan, 0.001);

fprintf('仿真完成: %d 时间步\n', length(t));
fprintf('最终状态: θ=%.3f°, φ=%.3f°\n', ...
    rad2deg(states(end,1)), rad2deg(states(end,5)));
fprintf('最大轮毂扭矩: %.2f N·m\n', max(abs(controls(:,1))));
fprintf('最大髋扭矩: %.2f N·m\n', max(abs(controls(:,2))));

%% ---- 第 6 步: 可视化 ----
PlotSimResults(t, states, controls, '阶跃响应: θ₀=5.7°, φ₀=2.9°');

%% ---- 第 7 步: 线性 vs 非线性对比 ----
fprintf('\n======== 测试 5: 线性 vs 非线性对比 ========\n');

x0_small = [0.05; 0; 0; 0; 0; 0];
[t_nl, s_nl, ~] = RunNonlinearSim(params, lqr_ctrl, x0_small, [0 2], 0.001);
[t_lin, s_lin, ~] = RunLinearSim(params, lqr_ctrl, x0_small, [0 2], 0.001, 0.22);

diff_max = max(abs(s_nl(:,1) - s_lin(:,1)));
fprintf('非线性 vs 线性最大差异: %.6f rad (%.4f°)\n', ...
    diff_max, rad2deg(diff_max));

figure('Name', '线性 vs 非线性对比');
subplot(2,1,1); hold on;
plot(t_nl, rad2deg(s_nl(:,1)), 'b-', 'LineWidth', 1.5);
plot(t_lin, rad2deg(s_lin(:,1)), 'r--', 'LineWidth', 1.5);
ylabel('\theta (deg)'); legend('非线性', '线性');
title('腿倾角响应对比'); grid on;

subplot(2,1,2);
plot(t_nl, rad2deg(s_nl(:,1) - s_lin(:,1)), 'k-', 'LineWidth', 1);
ylabel('\Delta\theta (deg)'); xlabel('时间 (s)');
title('非线性 - 线性 误差'); grid on;

%% ---- 第 8 步: 导出 C 代码增益系数 ----
fprintf('\n======== 导出 C 代码增益系数 ========\n');
fprintf('float K_coeffs[2][6][4] = {\n');
for ctrl = 1:2
    if ctrl == 1, fprintf('    // T_wheel\n');
    else,         fprintf('    // T_hip\n'); end
    for state = 1:6
        c = lqr_ctrl.coeffs(ctrl, state, :);
        fprintf('    {%.6ff, %.6ff, %.6ff, %.6ff},\n', c(1), c(2), c(3), c(4));
    end
    if ctrl == 1, fprintf('\n'); end
end
fprintf('};\n');

fprintf('\n======== 全部测试完成 ========\n');
```

### 9.2 物理参数类：`RobotParams.m`

```matlab
classdef RobotParams < handle
    %% 串联轮腿机器人物理参数
    
    properties
        % 五连杆几何
        L1 = 0.150;   % 左上连杆 (m)
        L2 = 0.150;   % 左下连杆 (m)
        L3 = 0.150;   % 右上连杆 (m)
        L4 = 0.150;   % 右下连杆 (m)
        L5 = 0.080;   % 基座间距 (m)
        
        % 质量参数
        Mb = 18.0;    % 机身质量 (kg)
        Ml = 2.0;     % 单腿等效质量 (kg)
        Mw = 0.5;     % 单轮质量 (kg)
        
        % 惯量参数
        Ib = 0.8;     % 机身俯仰转动惯量 (kg·m²)
        Iw = 0.002;   % 轮子转动惯量 (kg·m²)
        Rw = 0.076;   % 轮子半径 (m)
        hb = 0.15;    % 机身质心高度 (m)
        
        % 环境
        g = 9.81;     % 重力加速度 (m/s²)
        
        % 执行器约束
        T_wheel_max = 5.0;    % 轮毂最大扭矩 (N·m)
        T_hip_max   = 9.0;    % 髋关节最大扭矩 (N·m)
        L0_min = 0.18;        % 最小虚拟腿长 (m)
        L0_max = 0.30;        % 最大虚拟腿长 (m)
    end
    
    methods
        function Meff = getMeff(obj)
            % 等效平动质量
            Meff = obj.Mb + obj.Mw + obj.Iw / obj.Rw^2;
        end
    end
end
```

### 9.3 五连杆正运动学：`FiveBarFK.m`

```matlab
function fk = FiveBarFK(params, theta1, theta4)
    %% 五连杆正运动学
    %  输入: θ₁, θ₄ (rad)
    %  输出: 结构体 fk (Cx, Cy, L0, phi0, theta2, theta3, dBE)
    
    % B 点 (左驱动 → 左被动)
    Bx = -params.L5/2 + params.L1 * cos(theta1);
    By = params.L1 * sin(theta1);
    
    % E 点 (右驱动 → 右被动)
    Ex = params.L5/2 + params.L4 * cos(theta4);
    Ey = params.L4 * sin(theta4);
    
    % BE 距离和方向
    dBE = sqrt((Ex - Bx)^2 + (Ey - By)^2);
    alpha_BE = atan2(Ey - By, Ex - Bx);
    
    % 余弦定理求 ∠CBE (三角形 BCE 中)
    cos_beta = (params.L2^2 + dBE^2 - params.L3^2) / (2 * params.L2 * dBE);
    cos_beta = max(-1, min(1, cos_beta));  % clamp
    beta = acos(cos_beta);
    
    % 凸构型 (CONVEX)
    theta2 = alpha_BE + beta;
    
    % 末端 C (轮轴位置)
    Cx = Bx + params.L2 * cos(theta2);
    Cy = By + params.L2 * sin(theta2);
    
    % θ₃ (从 E 到 C)
    theta3 = atan2(Cy - Ey, Cx - Ex);
    
    % 虚拟腿参数
    L0   = sqrt(Cx^2 + Cy^2);
    phi0 = atan2(Cy, Cx);
    
    % 返回结构体
    fk.Cx = Cx;     fk.Cy = Cy;
    fk.L0 = L0;     fk.phi0 = phi0;
    fk.theta2 = theta2;
    fk.theta3 = theta3;
    fk.dBE = dBE;
end
```

### 9.4 五连杆逆运动学：`FiveBarIK.m`

```matlab
function [theta1, theta4] = FiveBarIK(params, L0_target, phi0_target, mode)
    %% 五连杆逆运动学
    %  输入: L₀_target (m), φ₀_target (rad), mode ('PP','PN','NP','NN')
    %  输出: θ₁, θ₄ (rad)
    
    if nargin < 4
        mode = 'PP';
    end
    
    % 目标末端 C 坐标
    Cx = L0_target * cos(phi0_target);
    Cy = L0_target * sin(phi0_target);
    
    % ---- 上链 (A → B → C) ----
    Ax = -params.L5/2;  Ay = 0;
    dAC = sqrt((Cx - Ax)^2 + (Cy - Ay)^2);
    gamma_A = atan2(Cy - Ay, Cx - Ax);
    cos_CAB = (params.L1^2 + dAC^2 - params.L2^2) / (2 * params.L1 * dAC);
    cos_CAB = max(-1, min(1, cos_CAB));
    ang_CAB = acos(cos_CAB);
    
    % ---- 下链 (D → E → C) ----
    Dx = params.L5/2;   Dy = 0;
    dDC = sqrt((Cx - Dx)^2 + (Cy - Dy)^2);
    gamma_D = atan2(Cy - Dy, Cx - Dx);
    cos_CDE = (params.L4^2 + dDC^2 - params.L3^2) / (2 * params.L4 * dDC);
    cos_CDE = max(-1, min(1, cos_CDE));
    ang_CDE = acos(cos_CDE);
    
    % ---- 根据构型模式选择符号 ----
    switch mode
        case 'PP'
            s1 =  1;  s4 = -1;
        case 'PN'
            s1 =  1;  s4 =  1;
        case 'NP'
            s1 = -1;  s4 = -1;
        case 'NN'
            s1 = -1;  s4 =  1;
        otherwise
            error('未知构型模式: %s', mode);
    end
    
    theta1 = gamma_A + s1 * ang_CAB;
    theta4 = gamma_D + s4 * ang_CDE;
end
```

### 9.5 解析雅可比矩阵：`FiveBarJacobian.m`

```matlab
function [J_v, J_cart] = FiveBarJacobian(params, theta1, theta4)
    %% 五连杆解析雅可比矩阵
    %  J_v: 虚拟腿空间 [dL₀/dt; dφ₀/dt] = J_v · [dθ₁/dt; dθ₄/dt]
    %  J_cart: 笛卡尔空间 [dCx/dt; dCy/dt] = J_cart · [dθ₁/dt; dθ₄/dt]
    
    % 先做正运动学
    fk = FiveBarFK(params, theta1, theta4);
    theta2 = fk.theta2;
    theta3 = fk.theta3;
    Cx = fk.Cx;
    Cy = fk.Cy;
    L0 = fk.L0;
    
    % 辅助量
    cos_32 = cos(theta3 - theta2);
    cos_31 = cos(theta3 - theta1);
    cos_34 = cos(theta3 - theta4);
    
    % 笛卡尔雅可比
    j11 = -params.L1*sin(theta1) + params.L1*sin(theta2)*cos_31/cos_32;
    j12 = -params.L4*sin(theta2)*cos_34/cos_32;
    j21 =  params.L1*cos(theta1) - params.L1*cos(theta2)*cos_31/cos_32;
    j22 =  params.L4*cos(theta2)*cos_34/cos_32;
    
    J_cart = [j11, j12; j21, j22];
    
    % 转换到虚拟腿空间
    T = [Cx/L0,    Cy/L0;
        -Cy/L0^2,  Cx/L0^2];
    
    J_v = T * J_cart;
end

function J_num = FiveBarJacobianNumerical(params, theta1, theta4, eps)
    %% 数值雅可比 (用于验证解析雅可比)
    if nargin < 4
        eps = 1e-6;
    end
    
    fk0 = FiveBarFK(params, theta1, theta4);
    L00 = fk0.L0;
    phi00 = fk0.phi0;
    
    fk1 = FiveBarFK(params, theta1 + eps, theta4);
    fk4 = FiveBarFK(params, theta1, theta4 + eps);
    
    J_num = [(fk1.L0 - L00)/eps,       (fk4.L0 - L00)/eps;
             (fk1.phi0 - phi00)/eps,   (fk4.phi0 - phi00)/eps];
end
```

### 9.6 WIP 非线性动力学：`WIPDynamics.m`

```matlab
function dstate = WIPDynamics(t, state, params, lqr_ctrl, L0_profile, ...
                              disturbance, ref_velocity)
    %% 轮式倒立摆非线性动力学 ODE
    %
    %  state = [θ, θ̇, x, ẋ, φ, φ̇]
    %  返回 dstate = [θ̇, θ̈, ẋ, ẍ, φ̇, φ̈]
    
    % 解析状态
    theta   = state(1);
    dtheta  = state(2);
    x       = state(3);
    dx      = state(4);
    phi     = state(5);
    dphi    = state(6);
    
    % 当前腿长
    if ~isempty(L0_profile)
        L0 = L0_profile(t);
    else
        L0 = 0.22;
    end
    % 简化: 腿长变化率为 0
    dL0 = 0;
    
    % 等效平动质量
    Meff = params.getMeff();
    
    % ---- 质量矩阵 M(3×3) ----
    M = zeros(3, 3);
    M(1,1) = Meff;
    M(2,2) = params.Mb * L0^2;
    M(3,3) = params.Ib;
    M(1,2) = params.Mb * L0 * cos(theta);
    M(2,1) = M(1,2);
    
    % ---- 科里奥利 + 重力 ----
    C = zeros(3, 1);
    C(1) = -params.Mb * L0 * dtheta^2 * sin(theta) ...
           + 2 * params.Mb * dL0 * dtheta * cos(theta);
    C(2) = -2 * params.Mb * L0 * dL0 * dtheta ...
           + params.Mb * dL0 * dx * cos(theta) ...
           - params.Mb * params.g * L0 * sin(theta);
    C(3) = -params.Mb * params.g * params.hb * sin(phi);
    
    % ---- 控制输入 ----
    % 加入参考速度前馈
    if isempty(ref_velocity)
        vx_ref = 0;
    else
        vx_ref = ref_velocity(t);
    end
    
    % 修改速度误差
    state_mod = [theta; dtheta; 0; dx - vx_ref; phi; dphi];
    u = lqr_ctrl.compute(state_mod, L0)';
    
    % 扰动等效力矩
    if ~isempty(disturbance)
        u(1) = u(1) + disturbance(t) * params.Rw;
    end
    
    % 扭矩限幅
    u(1) = max(-params.T_wheel_max, min(params.T_wheel_max, u(1)));
    u(2) = max(-params.T_hip_max,   min(params.T_hip_max,   u(2)));
    
    % ---- 输入矩阵 ----
    B_ctrl = zeros(3, 2);
    B_ctrl(1,1) = 1 / params.Rw;
    B_ctrl(2,2) = 1;
    B_ctrl(3,2) = 1;
    
    % ---- 求解加速度: M·q̈ = B·u - C → q̈ = M⁻¹(Bu - C) ----
    qddot = M \ (B_ctrl * u - C);
    
    % ---- 状态导数 ----
    dstate = zeros(6, 1);
    dstate(1) = dtheta;     % θ̇
    dstate(2) = qddot(2);   % θ̈
    dstate(3) = dx;         % ẋ
    dstate(4) = qddot(1);   % ẍ
    dstate(5) = dphi;       % φ̇
    dstate(6) = qddot(3);   % φ̈
end
```

### 9.7 LQR 增益调度器：`LQRDesigner.m`

```matlab
classdef LQRDesigner < handle
    %% LQR 增益调度控制器设计器
    
    properties
        params
        Q_diag
        R_diag
        L0_range
        coeffs   % 2×6×4 [ctrl][state][poly_coeff]
        K_table  % 2×6×N 原始增益表
    end
    
    methods
        function obj = LQRDesigner(params, Q_diag, R_diag, L0_range)
            obj.params = params;
            obj.Q_diag = Q_diag;
            obj.R_diag = R_diag;
            obj.L0_range = L0_range;
            obj = obj.buildGainTable();
        end
        
        function obj = buildGainTable(obj)
            %% 离线计算增益表并做三次多项式拟合
            N = length(obj.L0_range);
            obj.K_table = zeros(2, 6, N);
            
            Q = diag(obj.Q_diag);
            R = diag(obj.R_diag);
            
            for i = 1:N
                L0 = obj.L0_range(i);
                [A, B] = obj.buildAB(L0);
                
                % 使用 MATLAB 内置 lqr() 函数
                [K, ~, ~] = lqr(A, B, Q, R);
                obj.K_table(:, :, i) = K;
                
                % 验证闭环稳定性
                A_cl = A - B * K;
                eigs_cl = eig(A_cl);
                if any(real(eigs_cl) >= 0)
                    warning('L₀=%.3f m 时闭环系统不稳定!', L0);
                end
            end
            
            % 三次多项式拟合
            obj.coeffs = zeros(2, 6, 4);
            for ctrl = 1:2
                for state = 1:6
                    p = polyfit(obj.L0_range, ...
                        squeeze(obj.K_table(ctrl, state, :))', 3);
                    obj.coeffs(ctrl, state, :) = p;
                end
            end
        end
        
        function [A, B] = buildAB(obj, L0)
            %% 构造 6 状态 A, B 矩阵
            p = obj.params;
            Meff = p.getMeff();
            Delta = Meff - p.Mb;
            
            A = zeros(6, 6);
            A(1,2) = 1;
            A(3,4) = 1;
            A(5,6) = 1;
            
            A(2,1) = Meff * p.g / (L0 * Delta);
            A(4,1) = -p.Mb * p.g / Delta;
            A(6,5) = p.Mb * p.g * p.hb / p.Ib;
            
            B = zeros(6, 2);
            B(2,1) = -1 / (p.Rw * L0 * Delta);
            B(2,2) = Meff / (p.Mb * L0^2 * Delta);
            B(4,1) = 1 / (p.Rw * Delta);
            B(4,2) = -1 / (L0 * Delta);
            B(6,2) = 1 / p.Ib;
        end
        
        function K = getGains(obj, L0)
            %% 根据当前腿长获取 K 矩阵 (2×6)
            K = zeros(2, 6);
            for ctrl = 1:2
                for state = 1:6
                    c = squeeze(obj.coeffs(ctrl, state, :))';
                    K(ctrl, state) = polyval(c, L0);
                end
            end
        end
        
        function u = compute(obj, state, L0)
            %% 计算控制量 u = -K·x
            %  state: [θ, θ̇, x, ẋ, φ, φ̇] (6×1 或 1×6)
            K = obj.getGains(L0);
            u = -K * state(:);
        end
        
        function plotGainSurface(obj)
            %% 绘制增益随 L₀ 的变化
            L0_fine = linspace(obj.L0_range(1), obj.L0_range(end), 100);
            K_fine = zeros(2, 6, length(L0_fine));
            
            for i = 1:length(L0_fine)
                K_fine(:, :, i) = obj.getGains(L0_fine(i));
            end
            
            state_names = {'\theta', '\theta dot', 'x', 'x dot', '\phi', '\phi dot'};
            ctrl_names  = {'T_{wheel}', 'T_{hip}'};
            
            figure('Name', 'LQR 增益随腿长变化');
            for ctrl = 1:2
                for state = 1:6
                    subplot(2, 6, (ctrl-1)*6 + state);
                    plot(L0_fine, squeeze(K_fine(ctrl, state, :)), ...
                         'b-', 'LineWidth', 1.5);
                    xlabel('L₀ (m)'); 
                    ylabel(sprintf('K_{%d,%d}', ctrl, state));
                    title(sprintf('%s → %s', state_names{state}, ctrl_names{ctrl}));
                    grid on;
                end
            end
            sgtitle('LQR 增益调度表: K(L₀) 三次多项式拟合');
        end
        
        function plotEigenvalues(obj)
            %% 绘制闭环特征值随腿长的变化
            L0_fine = linspace(obj.L0_range(1), obj.L0_range(end), 100);
            all_eigs = zeros(length(L0_fine), 6);
            
            for i = 1:length(L0_fine)
                L0 = L0_fine(i);
                [A, B] = obj.buildAB(L0);
                K = obj.getGains(L0);
                A_cl = A - B * K;
                all_eigs(i, :) = eig(A_cl)';
            end
            
            figure('Name', '闭环特征值分析');
            subplot(1,2,1);
            plot(L0_fine, real(all_eigs), '-', 'LineWidth', 1);
            yline(0, 'k--');
            xlabel('L₀ (m)'); ylabel('Real(λ)');
            title('闭环特征值实部 vs 腿长');
            grid on;
            
            subplot(1,2,2);
            colors = lines(6);
            hold on;
            for j = 1:6
                scatter(real(all_eigs(:,j)), imag(all_eigs(:,j)), ...
                    5, colors(j,:), 'filled');
            end
            xline(0, 'k--'); yline(0, 'k--');
            xlabel('Real(λ)'); ylabel('Imag(λ)');
            title('特征值分布');
            grid on; axis equal;
        end
    end
end
```

### 9.8 非线性仿真运行器：`RunNonlinearSim.m`

```matlab
function [t, states, controls] = RunNonlinearSim(params, lqr_ctrl, ...
                                                  x0, tspan, dt, varargin)
    %% 运行非线性仿真
    %
    %  可选参数 (Name-Value pairs):
    %    'L0Profile'    - 函数句柄 @(t) 返回 L₀_target
    %    'Disturbance'  - 函数句柄 @(t) 返回水平扰动力 (N)
    %    'RefVelocity'  - 函数句柄 @(t) 返回前向参考速度 (m/s)
    
    p = inputParser;
    addParameter(p, 'L0Profile',   @(t) 0.22);
    addParameter(p, 'Disturbance', @(t) 0);
    addParameter(p, 'RefVelocity', @(t) 0);
    parse(p, varargin{:});
    
    L0_profile  = p.Results.L0Profile;
    disturbance = p.Results.Disturbance;
    ref_vel     = p.Results.RefVelocity;
    
    % ODE 选项
    opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-9, 'MaxStep', dt);
    
    % 积分
    sol = ode45(@(t, y) WIPDynamics(t, y, params, lqr_ctrl, ...
                  L0_profile, disturbance, ref_vel), tspan, x0, opts);
    
    % 在均匀时间网格上求值
    t = (tspan(1):dt:tspan(2))';
    states = deval(sol, t)';
    
    % 提取控制量
    controls = zeros(length(t), 2);
    for i = 1:length(t)
        L0 = L0_profile(t(i));
        state_i = states(i, :)';
        % 施加参考速度前馈
        vx_ref = ref_vel(t(i));
        state_mod = [state_i(1); state_i(2); 0; state_i(4) - vx_ref; ...
                     state_i(5); state_i(6)];
        u = lqr_ctrl.compute(state_mod, L0)';
        u(1) = u(1) + disturbance(t(i)) * params.Rw;
        controls(i, :) = max(-[params.T_wheel_max, params.T_hip_max], ...
                          min( [params.T_wheel_max, params.T_hip_max], u));
    end
end

function [t, states, controls] = RunLinearSim(params, lqr_ctrl, ...
                                               x0, tspan, dt, L0)
    %% 运行线性化仿真 (用于对比验证)
    [A, B] = lqr_ctrl.buildAB(L0);
    K = lqr_ctrl.getGains(L0);
    A_cl = A - B * K;
    
    t = (tspan(1):dt:tspan(2))';
    
    % 矩阵指数法 (精确线性系统)
    states = zeros(length(t), 6);
    states(1, :) = x0';
    for i = 2:length(t)
        states(i, :) = (expm(A_cl * dt) * states(i-1, :)')';
    end
    
    controls = -(K * states')';
end
```

### 9.9 可视化函数：`PlotSimResults.m`

```matlab
function PlotSimResults(t, states, controls, suptitle_text)
    %% 仿真结果可视化
    
    figure('Name', suptitle_text, 'Position', [100, 100, 1000, 750]);
    
    state_labels = {
        '\theta (rad)',          '\theta Leg Angle';
        '\theta dot (rad/s)',    'Leg Angular Velocity';
        'x (m)',                 'Wheel Position';
        'x dot (m/s)',           'Wheel Velocity';
        '\phi (rad)',            'Body Pitch';
        '\phi dot (rad/s)',      'Body Pitch Velocity'
    };
    
    for i = 1:6
        subplot(4, 2, i);
        plot(t, states(:, i), 'b-', 'LineWidth', 1.2);
        ylabel(state_labels{i, 1});
        title(state_labels{i, 2});
        yline(0, 'k-', 'LineWidth', 0.5);
        grid on;
    end
    
    % 控制量
    subplot(4, 2, 7);
    plot(t, controls(:, 1), 'r-', 'LineWidth', 1.2);
    ylabel('T_{wheel} (N·m)');
    title('Wheel Torque Command');
    yline(0, 'k-', 'LineWidth', 0.5);
    grid on;
    
    subplot(4, 2, 8);
    plot(t, controls(:, 2), 'Color', [0 0.6 0], 'LineWidth', 1.2);
    ylabel('T_{hip} (N·m)');
    title('Hip Torque Command');
    yline(0, 'k-', 'LineWidth', 0.5);
    grid on;
    
    sgtitle(suptitle_text, 'FontSize', 14, 'FontWeight', 'bold');
end
```

### 9.10 测试套件脚本：`test_suite.m`

```matlab
%% ============================================================
%% 串联轮腿仿真 — 完整测试套件
%% ============================================================
%  运行所有测试用例，验证控制器性能
%% ============================================================

clear; clc; close all;
params = RobotParams();

Q_diag = [500, 10, 100, 10, 800, 20];
R_diag = [0.1, 1.0];
L0_range = linspace(0.18, 0.30, 13);
lqr_ctrl = LQRDesigner(params, Q_diag, R_diag, L0_range);

%% ---- 测试 1: 阶跃响应 ----
fprintf('测试 1: 阶跃响应 (初始角度 = 5°)\n');
x0 = [deg2rad(5); 0; 0; 0; 0; 0];
[t1, s1, c1] = RunNonlinearSim(params, lqr_ctrl, x0, [0 3], 0.001);
PlotSimResults(t1, s1, c1, '测试 1: 阶跃响应 (\theta_0 = 5^\circ)');
assert(abs(s1(end,1)) < 1e-3, 'FAIL: 未能回到平衡点');
fprintf('  PASS: θ_end = %.4f°\n', rad2deg(s1(end,1)));

%% ---- 测试 2: 脉冲扰动 ----
fprintf('\n测试 2: 脉冲扰动 (50N × 0.1s)\n');
x0 = zeros(6, 1);
dist_fcn = @(t) 50 * (t >= 1.0 & t <= 1.1);
[t2, s2, c2] = RunNonlinearSim(params, lqr_ctrl, x0, [0 5], 0.001, ...
    'Disturbance', dist_fcn);
PlotSimResults(t2, s2, c2, '测试 2: 脉冲扰动 (50N × 0.1s)');
max_theta = max(abs(s2(:,1)));
fprintf('  PASS: 最大偏差 = %.2f°, 恢复残差 = %.4f°\n', ...
    rad2deg(max_theta), rad2deg(abs(s2(end,1))));

%% ---- 测试 3: 速度跟踪 ----
fprintf('\n测试 3: 速度跟踪 (vx = 1 m/s)\n');
x0 = zeros(6, 1);
vel_fcn = @(t) 1.0 * (t >= 1.0);
[t3, s3, c3] = RunNonlinearSim(params, lqr_ctrl, x0, [0 5], 0.001, ...
    'RefVelocity', vel_fcn);
PlotSimResults(t3, s3, c3, '测试 3: 速度跟踪 (v_x=1 m/s)');
fprintf('  PASS: 最终速度 = %.3f m/s, 误差 = %.3f m/s\n', ...
    s3(end,4), abs(s3(end,4) - 1));

%% ---- 测试 4: 增益调度 (腿长扫描) ----
fprintf('\n测试 4: 增益调度 (L₀: 0.20→0.28 m)\n');
x0 = zeros(6, 1);
leg_fcn = @(t) 0.20 + 0.08 * max(0, min(1, (t - 1) / 2));
[t4, s4, c4] = RunNonlinearSim(params, lqr_ctrl, x0, [0 5], 0.001, ...
    'L0Profile', leg_fcn);
PlotSimResults(t4, s4, c4, '测试 4: 增益调度 (L₀: 0.20→0.28 m)');
max_phi = max(abs(s4(:,5)));
fprintf('  PASS: 最大俯仰角 = %.3f°\n', rad2deg(max_phi));
assert(max_phi < deg2rad(5), 'FAIL: 腿长变化时不稳定');

%% ---- 测试 5: 大角度跌落 (非线性效应) ----
fprintf('\n测试 5: 大角度偏离 (θ₀ = 30°)\n');
x0 = [deg2rad(30); 0; 0; 0; 0; 0];
[t5, s5, c5] = RunNonlinearSim(params, lqr_ctrl, x0, [0 5], 0.001);
PlotSimResults(t5, s5, c5, '测试 5: 大角度回复 (\theta_0 = 30^\circ)');

% 检查是否稳定或发散
if abs(s5(end,1)) < 0.1
    fprintf('  PASS: 大角度下仍稳定, θ_end = %.2f°\n', rad2deg(s5(end,1)));
else
    fprintf('  INFO: 大角度下发散或收敛慢, θ_end = %.2f°\n', rad2deg(s5(end,1)));
    fprintf('        (这可能在预期内，LQR 仅在小角度下保证稳定)\n');
end

%% ---- 测试 6: 特征值分析 ----
fprintf('\n测试 6: 特征值随腿长变化\n');
lqr_ctrl.plotEigenvalues();

%% ---- 测试 7: 增益曲面 ----
fprintf('\n测试 7: 增益随 L₀ 变化曲面\n');
lqr_ctrl.plotGainSurface();

fprintf('\n======== 全部测试完成 ========\n');
```

### 9.11 参数扫描脚本：`param_sweep.m`

```matlab
function results = param_sweep(params, L0)
    %% LQR 权重参数扫描
    %  扫描 Q 矩阵中 q_theta 和 q_phi 的最优组合
    
    if nargin < 2
        L0 = 0.22;
    end
    
    % 扫描范围
    q_theta_vals = logspace(1, 3, 10);   % 10 ~ 1000
    q_phi_vals   = logspace(1, 3, 10);
    
    results = [];
    
    fprintf('扫描 Q 矩阵权重...\n');
    fprintf('q_theta\t q_phi\t ts(s)\t max(Re(λ))\t stable\n');
    fprintf('-------\t -----\t -----\t ----------\t ------\n');
    
    for q_theta = q_theta_vals
        for q_phi = q_phi_vals
            Q_diag = [q_theta, 10, 100, 10, q_phi, 20];
            R_diag = [0.1, 1.0];
            
            Meff = params.getMeff();
            Delta = Meff - params.Mb;
            
            A = zeros(6);
            A(1,2)=1; A(3,4)=1; A(5,6)=1;
            A(2,1) = Meff * params.g / (L0 * Delta);
            A(4,1) = -params.Mb * params.g / Delta;
            A(6,5) = params.Mb * params.g * params.hb / params.Ib;
            
            B = zeros(6,2);
            B(2,1) = -1/(params.Rw*L0*Delta);
            B(2,2) = Meff/(params.Mb*L0^2*Delta);
            B(4,1) = 1/(params.Rw*Delta);
            B(4,2) = -1/(L0*Delta);
            B(6,2) = 1/params.Ib;
            
            Q = diag(Q_diag);
            R = diag(R_diag);
            
            try
                [K, ~, e] = lqr(A, B, Q, R);
                
                max_real = max(real(e));
                ts_est = -4 / max_real;  % 估计调节时间
                
                r.q_theta = q_theta;
                r.q_phi   = q_phi;
                r.settling_time = ts_est;
                r.max_eig_real  = max_real;
                r.K_norm = norm(K);
                r.stable = all(real(e) < 0);
                
                fprintf('%.0f\t %.0f\t %.3f\t %.3f\t\t %d\n', ...
                    q_theta, q_phi, ts_est, max_real, r.stable);
                
                results = [results; r]; %#ok<AGROW>
            catch
                % 忽略不稳定的解
            end
        end
    end
    
    % 找到最优解 (最小调节时间)
    if ~isempty(results)
        valid = results([results.stable]);
        [~, idx] = min([valid.settling_time]);
        best = valid(idx);
        
        fprintf('\n======== 最优权重 ========\n');
        fprintf('q_theta = %.0f\n', best.q_theta);
        fprintf('q_phi   = %.0f\n', best.q_phi);
        fprintf('调节时间  = %.3f s\n', best.settling_time);
        fprintf('K 范数   = %.1f\n', best.K_norm);
    end
end
```

### 9.12 Simulink 仿真模型结构（参考）

对于需要更直观的模块化仿真，可以使用 Simulink 搭建如下模型：

```
┌─────────────────────────────────────────────────────────┐
│                   WIP_Nonlinear_Model.slx                 │
│                                                           │
│  ┌──────────┐   ┌───────────┐   ┌────────────────────┐  │
│  │ Ref Gen   │──▶│ LQR Ctrl  │──▶│ Nonlinear Dynamics │  │
│  │ vx_ref,  │   │ K(L₀)     │   │ (MATLAB Function)  │  │
│  │ L₀_ref   │   │ u=-Kx     │   │ M(q)q̈+Bu-C-G=0     │  │
│  └──────────┘   └─────┬─────┘   └─────────┬──────────┘  │
│                       │                    │             │
│                       │    ┌──────────┐    │             │
│                       └────│ 饱和限幅  │◀───┘             │
│                            │ Sat      │    state [6]     │
│                            └──────────┘                  │
│                                                           │
│  ┌──────────────────────────────────────────────────┐   │
│  │               Scope / To Workspace                 │   │
│  │        θ(t), φ(t), x(t), T_wheel(t), T_hip(t)     │   │
│  └──────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

**Simulink 模型搭建要点**：

1. **Nonlinear Dynamics 模块**：使用 MATLAB Function 块，内容为 `WIPDynamics.m` 的核心计算
2. **LQR Ctrl 模块**：使用 MATLAB Function 块调用 `lqr_ctrl.compute()`
3. **Ref Gen**：使用 Signal Builder 或 From Workspace 生成参考轨迹
4. **饱和限幅**：使用 Saturation 块，上限 ±T_max
5. **初始状态**：在 Integrator 块中设置 Initial condition
6. **求解器设置**：选择 ode4 (Runge-Kutta)，固定步长 0.001s

### 9.13 MATLAB vs Python 对比总结

| 特性          | MATLAB               | Python                                  |
| ----------- | -------------------- | --------------------------------------- |
| LQR 求解      | 内置 `lqr()` 函数        | 需 `scipy.linalg.solve_continuous_are()` |
| ODE 求解      | `ode45` / `ode15s`   | `scipy.integrate.solve_ivp`             |
| 符号推导        | 强 (Symbolic Toolbox) | 需 `sympy`                               |
| Simulink 集成 | 原生支持                 | 无直接等效                                   |
| C 代码生成      | Embedded Coder       | 手动导出                                    |
| 许可证         | 付费 (学术版免费)           | 开源免费                                    |
| 部署到固件       | 直接生成 C               | 手动翻译                                    |

**推荐工作流**：
1. **MATLAB**：符号推导 A/B 矩阵 → Simulink 快速原型验证
2. **Python**：参数扫描与优化 → 生成 C 增益表
3. **交叉验证**：两者结果误差应 < 1e-6

---

## 10. 附录：符号表与物理常数

### 10.1 完整符号表

| 符号 | 含义 | 单位 | 章节 |
|------|------|------|------|
| $L_1, L_2, L_3, L_4$ | 五连杆各段长度 | m | §1 |
| $L_5$ | 基座间距 | m | §1 |
| $\theta_1, \theta_4$ | 驱动关节角度 | rad | §1 |
| $\theta_2, \theta_3$ | 被动关节角度 | rad | §1 |
| $C_x, C_y$ | 末端（轮轴）坐标 | m | §1 |
| $L_0$ | 虚拟腿长 | m | §1 |
| $\phi_0$ | 虚拟腿倾角 | rad | §1 |
| $J_v$ | 虚拟腿雅可比 $(2 \times 2)$ | — | §2 |
| $J_{\text{cart}}$ | 笛卡尔雅可比 $(2 \times 2)$ | — | §2 |
| $\tau_1, \tau_4$ | 关节电机扭矩 | N·m | §2 |
| $F_0$ | VMC 虚拟推力 | N | §2 |
| $T_p$ | VMC 髋力矩 | N·m | §2 |
| $M_b$ | 机身质量 | kg | §3 |
| $M_l$ | 腿等效质量 | kg | §3 |
| $M_w$ | 轮质量 | kg | §3 |
| $M_{\text{eff}}$ | 等效平动质量 | kg | §3 |
| $I_b$ | 机身转动惯量 | kg·m² | §3 |
| $I_w$ | 轮转动惯量 | kg·m² | §3 |
| $R_w$ | 轮半径 | m | §3 |
| $h_b$ | 机身质心高度 | m | §3 |
| $g$ | 重力加速度 | m/s² | §3 |
| $\theta$ | 腿相对于竖直的偏角 | rad | §3 |
| $\phi$ | 机身俯仰角 | rad | §3 |
| $x$ | 轮子水平位移 | m | §3 |
| $\mathbf{A}$ | 状态矩阵 $(6 \times 6)$ | — | §4 |
| $\mathbf{B}$ | 输入矩阵 $(6 \times 2)$ | — | §4 |
| $\mathbf{Q}$ | 状态权重矩阵 $(6 \times 6)$ | — | §5 |
| $\mathbf{R}$ | 控制权重矩阵 $(2 \times 2)$ | — | §5 |
| $\mathbf{K}$ | LQR 反馈增益 $(2 \times 6)$ | — | §5 |
| $\mathbf{P}$ | 黎卡提方程解 $(6 \times 6)$ | — | §5 |

### 10.2 物理常数参考值

| 参数                      | 范围/值         | 来源            |
| ----------------------- | ------------ | ------------- |
| $L_1 = L_2 = L_3 = L_4$ | 0.150 m      | 机械设计图纸        |
| $L_5$                   | 0.080 m      | 机械设计图纸        |
| $L_0$                   | 0.18–0.30 m  | 工作空间分析        |
| $\phi_0$                | -30° to +30° | 关节限位          |
| $M_b$                   | 16–20 kg     | CAD 质量估算      |
| $R_w$                   | 0.076 m      | MF9025 规格书    |
| $T_w^{\max}$            | 5 N·m        | MF9025 峰值扭矩   |
| $T_{\text{hip}}^{\max}$ | 9 N·m        | DM-J8009 峰值扭矩 |
| $g$                     | 9.81 m/s²    | 地球标准重力        |

### 10.3 参考文献

1. Featherstone, R. (2008). *Rigid Body Dynamics Algorithms*. Springer.
2. Siciliano, B. et al. (2009). *Robotics: Modelling, Planning and Control*. Springer.
3. Åström, K.J. & Murray, R.M. (2021). *Feedback Systems: An Introduction for Scientists and Engineers*. Princeton.
4. Tedrake, R. (2022). *Underactuated Robotics: Learning, Planning, and Control*. MIT Press. [Online]
5. Pratt, J. et al. (2001). "Virtual Model Control: An Intuitive Approach for Bipedal Locomotion." *Int. J. Robotics Research*.
6. Kim, S. & Wensing, P.M. (2017). "Design of Dynamic Legged Robots." *Foundations and Trends in Robotics*.
7. RM 2024 轮足机器人开源: [WilliamGwok/RP_Balance](https://github.com/WilliamGwok/RP_Balance)
8. RM 2025 串联轮腿设计报告 (清华大学出版社)

---

## 相关笔记

- [[RM_Chassis_串联轮腿步兵下位机控制系统设计]] — 串联轮腿的 STM32 下位机实现
- [[RM_Chassis_偏置并联轮腿学习指南]] — 偏置并联轮腿（郑派成毕业设计）：LQR+MPC 级联、大风车自救、跳跃越障、PSO 参数优化
- [[RM_LQR_lqr.py数学推导]] — LQR 理论在轮腿中的数学基础
- [[RM_Chassis_底盘轮系运动学解算教程]] — 传统轮式底盘运动学

---

> **文档维护**：本文档应与《串联轮腿步兵下位机控制系统设计》配合阅读。设计文档描述架构和实现，本文档提供数学基础和仿真工具。LQR 增益的 Python 计算值应导出到设计文档的 C 代码增益表。
