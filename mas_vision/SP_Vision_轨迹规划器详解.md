# SP_Vision 轨迹规划器详解

> 📘 **母文档**：[[SP_Vision_学习指南]]（先读母文档建立全局框架，本篇深化其 §6.4 与 §7 的轨迹理论）
> 📘 **上游**：规划器的输入（整车状态、`target.predict(t)`）来自 [[SP_Vision_整车估计EKF详解]]
> 📘 **相关**：打符的 `mpc_aim` 也走这套规划器，见 [[SP_Vision_打符与全向感知详解]]

本篇聚焦 sp_vision_25 的**核心创新**——把自瞄决策从"经验主义分段 if-else"重构为"轨迹优化问题"。核心是理解"**四条轨迹**"、"**为什么 MPC 是求解器而非控制器**"、"**提前减速和提前开火如何从一次 QP 里自然涌现**"。

---

## 目录

1. [轨迹视角：把自瞄重新定义](#1-轨迹视角把自瞄重新定义)
2. [Trajectory 数据结构与时域](#2-trajectory-数据结构与时域)
3. [plan() 完整流程](#3-plan-完整流程)
4. [get_trajectory：生成参考轨迹](#4-get_trajectory生成参考轨迹)
5. [TinyMPC 的优化问题：二重积分器 + 盒约束](#5-tinympc-的优化问题二重积分器--盒约束)
6. [ADMM 求解器内部](#6-admm-求解器内部)
7. [提前减速：约束下的隐式涌现](#7-提前减速约束下的隐式涌现)
8. [提前开火决策](#8-提前开火决策)
9. [弹道解算与预测时间](#9-弹道解算与预测时间)
10. [为什么是求解器而非闭环控制器](#10-为什么是求解器而非闭环控制器)
11. [概念关系图谱](#11-概念关系图谱)
12. [相关笔记](#12-相关笔记)

---

## 1. 轨迹视角：把自瞄重新定义

传统自瞄决策的痛点：不同兵种、不同敌方状态（平移/低速小陀螺/高速小陀螺）需要不同"行为"，每种都要单独写代码 + 设计判断条件，参数多、难调、难维护。而且自瞄的输入更像**三角波**（云台要来回追旋转的装甲板），没有理论把它和控制指标联系起来。

sp_vision_25 的重构：把自瞄从"瞄准一个点"改为"**让云台跟随一条 yaw(t) 曲线**"。定义四条轨迹（只讨论 yaw，pitch 同理）：

| 轨迹 | 定义 | 关系 |
|:----|:----|:----|
| **目标轨迹** $yaw_{target}(t)$ | 各时刻最近装甲板相对云台的方位角 | 由整车模型算 |
| **射击轨迹** $yaw_{shoot}(t)$ | 目标轨迹提前"子弹飞行时间"后 | $yaw_{shoot}(t) = yaw_{target}(t + t_{fly})$ |
| **云台轨迹** $yaw_{gimbal}(t)$ | 云台实际到达的角度 | 由控制器执行决定 |
| **规划后轨迹** | 规划器优化输出、供云台跟随的可行轨迹 | TinyMPC 求解 |

**命中条件**：$t$ 时刻 $yaw_{gimbal}(t)$ 与 $yaw_{shoot}(t)$ 误差在半个装甲板内，则此刻射出的子弹命中。若两条轨迹**完全重合**，则任意时刻射出的子弹都命中——"子弹发射如水流"。

**好自瞄的量化目标**：

$$\text{命中率} \le \text{重合度} = \frac{\text{重合段}}{\text{重合段} + \text{偏离段}}$$

重合度越高 → 射击窗口占比越高 → DPS 越高 → 击杀越快，同时命中率上限越高。于是"自瞄好坏"被统一为一个可优化目标：**最大化云台轨迹与射击轨迹的重合度**，受"云台最大加速度"（= 电机最大扭矩 / 云台惯量）制约。

> **一句话速记**：把自瞄看成"让云台轨迹贴合射击轨迹（目标轨迹提前 t_fly）"，重合度越高击杀越快命中越高，重合度受云台最大加速度制约——这就是轨迹规划器要优化的目标。

---

## 2. Trajectory 数据结构与时域

轨迹的代码表示（`planner.hpp:13-17`）：

```cpp
constexpr double DT = 0.01;               // 10ms 步长
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2; // = 100
using Trajectory = Eigen::Matrix<double, 4, HORIZON>;   // 4×100
```

- **4 行** = [yaw, yaw_vel, pitch, pitch_vel]（位置 + 速度前馈）
- **100 列** = 时域采样，DT=10ms
- **时域范围**：col(0) ↔ t = −50·DT = −0.5s，**col(50)（HALF_HORIZON）↔ t = 0（当前时刻）**，col(99) ↔ t = +0.49s

即规划器同时看**过去 0.5s 到未来 0.49s** 的一整段轨迹，第 50 列是"现在"。输出的 `Plan` 结构（`planner.hpp:19-31`）含 `control/fire` 两标志和 yaw/yaw_vel/yaw_acc、pitch/pitch_vel/pitch_acc 六个量（位置 + 一阶 + 二阶前馈）。

> **一句话速记**：轨迹是 4×100 矩阵（yaw/yaw_vel/pitch/pitch_vel × 时域），DT=10ms，第 50 列是当前时刻，规划器一次看过去 0.5s 到未来 0.49s。

---

## 3. plan() 完整流程

`planner.cpp:27-94` 是核心。分五步：

**① 弹速兜底**（`planner.cpp:29-32`）：`bullet_speed < 10 || > 25` 时强制 22 m/s，防异常弹速污染弹道解算。

**② 预测 fly_time，把目标轨迹平移成射击轨迹**（`planner.cpp:34-45`）：

```cpp
// 遍历所有虚拟装甲板，选水平距离最近的
min_dist = min over armor of xyza.head<2>().norm();
tools::Trajectory bullet_traj(bullet_speed, min_dist, xyz.z());   // 算子弹飞行时间
target.predict(bullet_traj.fly_time);    // ★ 把整个目标运动平移 fly_time
```

`target.predict(fly_time)` 让 EKF 状态向前推演 `fly_time`，得到"子弹到达时刻"的目标位置——这正是射击轨迹 $yaw_{shoot}(t) = yaw_{target}(t + t_{fly})$ 的代码实现。

**③ 生成参考轨迹**（`planner.cpp:48-56`）：`yaw0 = aim(target, bullet_speed)(0)`（当前时刻瞄准 yaw，作相对基准）；`traj = get_trajectory(...)` 生成 4×100 参考（见 §4）。

**④ 解两个独立 TinyMPC**（`planner.cpp:58-71`）：yaw、pitch 各一个 solver：

```cpp
// yaw
x0 << traj(0,0), traj(1,0);                    // 初始状态 = 参考轨迹首列
tiny_set_x0(yaw_solver_, x0);
yaw_solver_->work->Xref = traj.block(0,0,2,HORIZON);   // 参考 = yaw、yaw_vel 两行
tiny_solve(yaw_solver_);
// pitch 同理，Xref = traj.block(2,0,2,HORIZON)
```

**⑤ 取第 50 列组装 Plan**（`planner.cpp:73-85`）：

```cpp
plan.control  = true;
plan.target_yaw = limit_rad(traj(0, HALF_HORIZON) + yaw0);        // 射击轨迹当前值
plan.yaw     = limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);  // MPC 规划值(加回 yaw0)
plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);            // 速度前馈
plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);            // 加速度前馈
// pitch 同理（pitch 存绝对值，无需加基准）
```

注意 yaw 用相对基准 `yaw0` 存储（避免绕环），输出时加回。

**外层重载**（`planner.cpp:96-108`）在调用上面的 `plan` 前先按角速度加一层延迟预测：

```cpp
delay_time = |ekf_x()[7]| > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;
target->predict(now + delay_time);   // 补偿全链路延迟(见 §9.2)
```

`ekf_x()[7]` 是目标角速度 vyaw。高低速用不同延迟参数（虽然默认都是 0.015s）。

> **一句话速记**：plan() = 弹速兜底 → 用 fly_time 把目标轨迹 predict 成射击轨迹 → get_trajectory 生成 4×100 参考 → yaw/pitch 各解一个 TinyMPC → 取第 50 列（当前时刻）组装成带速度/加速度前馈的 Plan。

---

## 4. get_trajectory：生成参考轨迹

`planner.cpp:179-203` 生成 4×100 参考轨迹。核心是**用 EKF 反复 predict 采样未来位置 + 中心差分算速度前馈**：

```cpp
target.predict(-DT * (HALF_HORIZON + 1));       // 先回退到 t = −51·DT
yaw_pitch_last = aim(target, bullet_speed);      // t = −51·DT 的期望角
target.predict(DT);                              // 前进到 t = −50·DT
yaw_pitch = aim(target, bullet_speed);           // t = −50·DT
for (int i = 0; i < HORIZON; i++) {              // 逐列填 100 列
  target.predict(DT);
  yaw_pitch_next = aim(target, bullet_speed);    // t = (i−50+1)·DT
  yaw_vel   = limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2*DT);   // 中心差分
  pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2*DT);
  traj.col(i) << limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;
  yaw_pitch_last = yaw_pitch;
  yaw_pitch = yaw_pitch_next;
}
```

三个细节：

1. **中心差分算速度**：$v_i = \frac{p_{i+1} - p_{i-1}}{2\,DT}$，比前向差分更准（二阶精度），作为速度前馈。
2. **yaw 差分用 limit_rad 包裹**：yaw 过 ±π 时会跳变（如 179° → −179° 实际只差 2°），`limit_rad` 消除这个绕环假象。
3. **yaw 位置存相对增量** `limit_rad(yaw_pitch(0) − yaw0)`：以当前瞄准 yaw 为基准存相对值，同样为避免绕环；pitch 直接存绝对值（pitch 不会绕环）。

`aim()`（`planner.cpp:156-177`）单点求期望角：由 EKF 预测的目标位置做弹道解算，返回 `{limit_rad(azim + yaw_offset_), −bullet_traj.pitch − pitch_offset_}`，含机械安装角补偿。

> **一句话速记**：get_trajectory 让 EKF 从 −0.5s 到 +0.49s 逐 10ms predict 采样期望角，用中心差分算速度前馈；yaw 全程用 limit_rad + 相对基准避免过 ±π 绕环。

---

## 5. TinyMPC 的优化问题：二重积分器 + 盒约束

### 5.1 云台运动学建模为二重积分器

规划器把云台每个轴建模为**二重积分器**（角度←角速度←角加速度）。状态 $x = [\theta, \dot\theta]$（角度、角速度），控制 $u = [\ddot\theta]$（角加速度）。离散化（`planner.cpp:110-131`，pitch 结构相同）：

```cpp
Eigen::MatrixXd A{{1, DT}, {0, 1}};   // 状态转移
Eigen::MatrixXd B{{0}, {DT}};         // 输入矩阵
Eigen::VectorXd f{{0, 0}};            // 仿射项 = 0
```

即离散动力学：

$$\begin{bmatrix}\theta_{k+1}\\\dot\theta_{k+1}\end{bmatrix} = \underbrace{\begin{bmatrix}1 & DT\\0 & 1\end{bmatrix}}_{A}\begin{bmatrix}\theta_k\\\dot\theta_k\end{bmatrix} + \underbrace{\begin{bmatrix}0\\DT\end{bmatrix}}_{B}\ddot\theta_k$$

这是最简单的"加速度积分成速度、速度积分成角度"模型。

### 5.2 优化问题

$$\min_{u}\ \sum_{k=0}^{N-1}\Big[(x_k - x_k^{ref})^T Q (x_k - x_k^{ref}) + u_k^T R u_k\Big]$$

$$\text{s.t.}\quad x_{k+1} = A x_k + B u_k,\qquad -a_{max} \le u_k \le a_{max}$$

- $x_k^{ref}$ = §4 生成的参考轨迹（射击轨迹）
- $Q$ 惩罚"偏离射击轨迹"，$R$ 惩罚"控制量（加速度）"
- $u$ 盒约束 $\pm a_{max}$ = **云台最大加速度**（关键！见 §7）

setup（`planner.cpp:128-130`）：

```cpp
tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);
yaw_solver_->settings->max_iter = 10;   // 覆盖默认 1000
```

nx=2、nu=1、N=HORIZON=100、rho=1.0。`tiny_setup` 内部把 rho 加到 Q/R 对角（`tiny_api.cpp:107-108`），并做 Riccati 递推预计算稳态增益 Kinf、末端代价 Pinf（`:244-318`，迭代到 `|Kinf−Ktp1|<1e-5` 收敛）。

### 5.3 权重配置

`configs/standard4.yaml:83-92`：

```yaml
max_yaw_acc: 50        # 云台最大角加速度（rad/s²）→ u 盒约束
Q_yaw: [9e6, 0]        # [角度误差权重, 角速度误差权重]
R_yaw: [1]             # 加速度代价权重
max_pitch_acc: 100
Q_pitch: [9e6, 0]
R_pitch: [1]
```

**Q = [9e6, 0]** 的含义：角度误差权重极大（9×10⁶）、角速度误差权重 0、R=1。即**强跟踪角度、几乎不惩罚速度、弱惩罚控制**——策略是"不惜代价贴合射击轨迹的角度，速度随便"。这让重合度最大化，同时约束靠 u 盒约束兜底。

> **一句话速记**：云台建模为二重积分器 x=[角度,角速度] u=[角加速度]，MPC 最小化"偏离射击轨迹"的加权和，Q=[9e6,0] 强跟角度、R=1 弱惩罚控制、u 盒约束 = 云台最大加速度。

---

## 6. ADMM 求解器内部

TinyMPC 用 **ADMM**（交替方向乘子法）求解带约束的 QP。ADMM 的思路：把"无约束 LQR"和"约束投影"拆成两个子问题交替迭代，直到一致。

### 6.1 主迭代循环

`admm.cpp:312-382`，每次迭代五步：

```
1. backward_pass_grad   Riccati 反向传播更新线性项 d、p          admm.cpp:13-20
2. forward_pass         LQR 反馈 u = −Kinf·x − d，滚动 x_{k+1}    admm.cpp:25-32
3. update_slack         投影松弛变量到盒约束（见下）              admm.cpp:91-98
4. update_dual          对偶变量更新 g += x−v; y += u−z           admm.cpp:184-187
5. update_linear_cost   用 Xref/slack/dual 重算线性代价 q、r      admm.cpp:214-247
```

**第 3 步的盒约束投影**（`admm.cpp:96-98`）就是把控制量裁剪到 ±a_max：

```cpp
znew = u_max.cwiseMin(u_min.cwiseMax(u + y));   // clamp(u+y, u_min, u_max)
```

ADMM 的精髓：forward_pass 解的是**无约束**最优（LQR，快），update_slack 把解**投影**回约束集，update_dual 用乘子逼两者一致。反复迭代，约束解和无约束解收敛到同一点。

### 6.2 收敛判据与参数

- **max_iter = 10**（`planner.cpp:130`，覆盖默认 1000）——实时性要求，10 次够用（<1ms）。
- **收敛判据**（`admm.cpp:253-271`）检查原始/对偶残差最大绝对值 < 容差（默认 `1e-3`）：

```cpp
primal_residual_input = (u - znew).cwiseAbs().maxCoeff();     // 原始残差
dual_residual_input   = (z - znew).cwiseAbs().maxCoeff()*rho; // 对偶残差
```

- **rho = 1.0 固定**：自适应 rho 逻辑存在（`admm.cpp:331-357`）但 `adaptive_rho=0` 关闭（注释"currently supports only quadrotor system"）。
- 收敛后写 `solution->x = vnew, u = znew`；但注意 **plan() 读的是 `work->x`/`work->u`（forward_pass 结果），不是 solution**（`planner.cpp:75-85`）。

> **一句话速记**：ADMM 把带约束 QP 拆成"无约束 LQR（forward_pass）+ 盒约束投影（update_slack）+ 对偶逼近（update_dual）"交替迭代；max_iter=10、rho=1.0 固定，<1ms 出解。

---

## 7. 提前减速：约束下的隐式涌现

小陀螺换板时，目标/射击轨迹在切换点**不连续**（下一块装甲板从另一个角度冒出来），硬跟随会超调或滞后。传统做法要显式写"预判换板 → 提前减速"逻辑。

**本项目里提前减速是隐式涌现的**——代码里没有任何"减速"逻辑。关键在 u 盒约束（`planner.cpp:124-128`）：

```cpp
x_min = Constant(2, HORIZON, -1e17);          // 状态无约束
x_max = Constant(2, HORIZON,  1e17);
u_min = Constant(1, HORIZON-1, -max_yaw_acc); // 控制(加速度)盒约束 ±max_yaw_acc
u_max = Constant(1, HORIZON-1,  max_yaw_acc);
tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);
```

**为什么会自然提前减速**：MPC 在整个 100 步时域内最小化"偏离射击轨迹"，同时受加速度上限约束。当它看到未来某列参考轨迹要突变（换板），单靠瞬间加速跟不上（会违反 ±max_acc）。求解器为了在约束内尽可能贴合，只能**从更早的时刻开始分配减速加速度**，提前偏离当前轨迹、平滑过渡到下一块。这是"有限时域 + 加速度约束"下的最优解自然产物，不需要写任何 if 判断。

这正是轨迹视角的威力：把"提前减速"这个原本需要经验调参的行为，变成加速度盒约束下的数学最优。

> **一句话速记**：提前减速不是写出来的，是"100 步时域内贴合突变参考 + 加速度盒约束 ±max_acc"这个优化问题的隐式最优解——求解器为了不违反加速度上限又要跟上突变，只能提前开始过渡。

---

## 8. 提前开火决策

从开火命令到子弹出膛有延迟 $t_{fire}$（摩擦轮掉速）。只看当前误差判断开火不严谨。规划器的做法：**查 $t_{fire}$ 时刻射击轨迹与规划轨迹是否重合**（`planner.cpp:87-93`）：

```cpp
auto shoot_offset_ = 2;   // 硬编码，非配置项 → +2·DT = +20ms
plan.fire = std::hypot(
    traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
    traj(2, HALF_HORIZON + shoot_offset_) - pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)
) < fire_thresh_;
```

含义：检查第 `50 + 2 = 52` 列（当前 +20ms ≈ 子弹出膛时刻），比较**射击参考轨迹**（traj，第 0 行 yaw、第 2 行 pitch）与 **MPC 规划轨迹**（solver work->x）在该列的 yaw/pitch 二维欧氏距离，小于 `fire_thresh_`（standard4 为 0.003 rad）则开火。

即"**20ms 后云台规划位置是否落在射击轨迹容差内**"。这一步取代了传统的所有分段开火参数（近距离容差/远距离容差/高低速阈值……），一个 `fire_thresh_` 搞定。云台偏离射击轨迹时自动停火，也顺带减少弹丸浪费。

> ⚠️ **Voter 与开火无关**：`auto_aim::Voter`（`voter.cpp`）是 Color×ArmorName×ArmorType 的三维投票计数器，用于多帧稳定分类结果。它与 `plan.fire` 完全无关（开火只由上面的轨迹误差决定）。且当前 auto_aim 主流程未实例化它，属旧/独立模块。

> **一句话速记**：开火 = "查 t_fire（当前+20ms）时刻射击轨迹与 MPC 规划轨迹的 yaw/pitch 欧氏误差 < fire_thresh"，一个阈值取代所有分段开火参数；Voter 是分类投票器，与开火无关。

---

## 9. 弹道解算与预测时间

### 9.1 斜抛弹道（tools/trajectory.cpp）

无空气阻力斜抛，`g = 9.7833`（`trajectory.cpp:7`）。给定弹速 $v_0$、水平距离 $d$、高度差 $h$，求发射 pitch 和飞行时间。对 $\tan\theta$ 建立一元二次方程：

$$\frac{g d^2}{2v_0^2}\tan^2\theta - d\tan\theta + \left(\frac{g d^2}{2v_0^2} + h\right) = 0$$

```cpp
a = g*d*d / (2*v0*v0);   b = -d;   c = a + h;
delta = b*b - 4*a*c;
if (delta < 0) { unsolvable = true; return; }        // 打不到（超出射程）
tan_pitch_1 = (-b + sqrt(delta)) / (2*a);            // 两解
tan_pitch_2 = (-b - sqrt(delta)) / (2*a);
t_1 = d / (v0*cos(pitch_1));   t_2 = d / (v0*cos(pitch_2));
pitch = (t_1 < t_2) ? pitch_1 : pitch_2;             // 取飞行时间短者（低伸弹道）
fly_time = min(t_1, t_2);
```

两解对应"低伸"和"高抛"两种弹道，取**飞行时间更短**者（直射更稳、受扰动小）。`delta < 0` 时目标超出射程，`unsolvable=true`，plan 据此抛异常（`planner.cpp:174`）。

### 9.2 预测时间偏移

射击轨迹提前于目标轨迹的时间，除 $t_{fly}$ 外还要叠加全链路延迟：

| 延迟来源 | 处理 |
|:----|:----|
| 图像处理（神经网络推理） | 可直接算 |
| 图像传输、下位机通信、下位机控制 | 打包成一个调参量（约 15ms） |

代码里体现为 `high_speed_delay_time_` / `low_speed_delay_time_`（standard4 都是 0.015s），在 plan 外层重载 `target->predict(now + delay_time)` 补偿（`planner.cpp:100-103`）。**不需要单独考虑开火延迟**——射击轨迹上任意时刻发射都命中，开火延迟已被"查 t_fire 时刻误差"的机制吸收。

> **一句话速记**：弹道是无阻力斜抛（g=9.7833），解 tanθ 一元二次取飞行时间短者；预测时间 = t_fly + 全链路延迟（图像处理可算、其余约 15ms 调参），predict 提前量把目标轨迹平移成射击轨迹。

---

## 10. 为什么是求解器而非闭环控制器

这是理解本设计最关键的一点。TinyMPC 在这里的定位是**离线式轨迹整形器**，不是控制器：

1. **输入不含云台实际状态**：`plan(Target, bullet_speed)` 的输入只有 EKF 预测的目标运动（`target.armor_xyza_list()`、`target.predict()`）和弹速，**没有云台传感器反馈的实际 yaw/pitch**。
2. **初始状态取自参考轨迹自身**：MPC 的 `x0 << traj(0,0), traj(1,0)`（`planner.cpp:59-60`），即"期望轨迹的起点"，而非云台当前位置。
3. **输出交下位机闭环**：`plan.yaw/yaw_vel/yaw_acc` 经 `gimbal.send(...)`（`standard_mpc.cpp:80-82`）下发，云台的实际闭环由下位机的**计算力矩控制**（PID + 动力学模型 + 前馈）完成。

所以 MPC 只做一件事：**在期望的射击轨迹上，求解一条"云台加速度能力内可跟随"的平滑轨迹**（含速度/加速度前馈）。作者实测过用 MPC 直接闭环（转发力矩给电机），小陀螺时效果反而更差——因为闭环控制对模型误差和延迟敏感，而"轨迹整形 + 下位机闭环"分工更鲁棒。

```
EKF目标运动 ──▶ [TinyMPC 求解器] ──规划轨迹+前馈──▶ [下位机计算力矩控制] ──▶ 云台电机
             不含云台状态          (开环整形)         云台实际状态在这里闭环
```

> **一句话速记**：MPC 输入只有 EKF 预测目标（不含云台实际状态）、初始状态取参考轨迹起点、输出交下位机闭环——它是"在射击轨迹上求一条可跟随平滑轨迹"的开环整形器，实际闭环由下位机计算力矩控制完成。

---

## 11. 概念关系图谱

### 11.1 规划器数据流

```
       EKF 整车状态
            │
            ▼
   ① 弹速兜底 → ② 算 fly_time，target.predict(fly_time) ── 目标轨迹平移成射击轨迹
            │
            ▼
   ③ get_trajectory: EKF 逐 10ms predict 采样 −0.5s~+0.49s
      + 中心差分算速度前馈 → 4×100 参考轨迹 Xref
            │
            ├──────────────┬──────────────┐
            ▼              ▼               │
      ④ yaw TinyMPC   pitch TinyMPC        │  二重积分器 x=[角度,角速度] u=[角加速度]
         (ADMM ×10)     (ADMM ×10)         │  Q=[9e6,0] R=[1] u∈±max_acc
            │              │               │
            ▼              ▼               ▼
   ⑤ 取第50列(当前) → Plan{control, yaw/vel/acc, pitch/vel/acc, fire}
            │                                    │
            │                          ⑧ fire = 查第52列(+20ms)
            ▼                             射击轨迹 vs 规划轨迹误差 < fire_thresh
   gimbal.send → 下位机计算力矩控制 → 云台电机（实际闭环在此）
```

### 11.2 三个"隐式涌现"

轨迹视角的核心美感在于——传统要显式写的逻辑，都变成了优化问题的自然结果：

| 传统做法 | 本项目 | 涌现机制 |
|:----|:----|:----|
| 分段判断"该跟随还是瞄中心" | Xref 直接是射击轨迹，MPC 贴合它 | 目标函数 min ‖x−Xref‖ |
| 显式"预判换板 → 提前减速" | 无减速代码 | 加速度盒约束 + 有限时域最优 |
| 分段开火参数（近/远/高低速容差） | 一个 fire_thresh | 查 t_fire 时刻两轨迹误差 |

### 11.3 核心概念对照

| 易混概念 | 区别 |
|:----|:----|
| **目标轨迹 vs 射击轨迹** | 射击轨迹 = 目标轨迹提前 t_fly；`yaw_shoot(t)=yaw_target(t+t_fly)`，代码用 target.predict(fly_time) |
| **射击轨迹 vs 规划后轨迹** | 射击轨迹是"想命中该去的角"(Xref)；规划轨迹是"云台加速度能力内可跟随的"(MPC 输出 work->x) |
| **MPC 求解器 vs 闭环控制器** | 本项目 MPC 输入不含云台状态、输出交下位机闭环；不是直接控电机（实测闭环更差） |
| **提前减速：显式 vs 隐式** | 本项目无减速代码，是加速度盒约束下有限时域最优的隐式产物 |
| **提前开火 vs 当前误差开火** | 查 t_fire(+20ms) 时刻误差，不是查当前误差；吸收了开火延迟 |
| **Q=[9e6,0] 的含义** | 角度误差权重极大、角速度权重 0、R=1：强跟角度、弱惩罚控制 |
| **work->x vs solution->x** | plan() 读 work->x(forward_pass 结果)，非 solution->x |
| **Voter vs 开火** | Voter 是分类投票器，与 plan.fire 无关；开火只由轨迹误差决定 |
| **低伸 vs 高抛弹道** | tanθ 一元二次两解，取飞行时间短者(低伸，更稳) |
| **隐式搜索(TinyMPC) vs 显式搜索(五次多项式)** | TinyMPC 通用、国赛上场；五次多项式确定性高、仅仿真验证 |

### 11.4 全局一句话速记

> 轨迹规划器把自瞄重构为"让云台轨迹贴合射击轨迹"：用 fly_time 把目标轨迹 predict 成射击轨迹(Xref)，云台建模为二重积分器，TinyMPC(ADMM×10,<1ms) 在"角度强跟踪 + 加速度盒约束"下把 Xref 整形成可跟随的规划轨迹——提前减速隐式涌现、开火退化为"查 t_fire 时刻两轨迹重合"；MPC 只是开环整形器，实际闭环交下位机计算力矩控制。

---

## 12. 相关笔记

- [[SP_Vision_学习指南]] — 母文档，全局框架（本篇深化 §6.4 与 §7 轨迹理论）
- [[SP_Vision_整车估计EKF详解]] — 规划器的输入（整车状态、`target.predict(t)` 平移）由 EKF 提供
- [[SP_Vision_装甲板识别与解算详解]] — 感知链的起点，最终喂给 EKF 再喂给规划器
- [[SP_Vision_打符与全向感知详解]] — 打符 `mpc_aim` 复用这套轨迹规划器
- [[SP_Vision_IO与多线程架构详解]] — 规划结果经 `gimbal.send` 下发（含速度/加速度前馈），standard_mpc 每 10ms 规划一次
- [[MAS_Vision_弹道解算与射击控制教程]] — 本战队弹道/开火的另一种（分段）思路，可对照轨迹视角
- [[RM_C程序员_C++速通指南]] — Eigen 矩阵、模板等本篇技法
- 关键参考：方俊杰 Linear Modelled Top Detector（轨迹视角）、TinyMPC（ICRA 2024）、Modern Robotics（计算力矩控制）