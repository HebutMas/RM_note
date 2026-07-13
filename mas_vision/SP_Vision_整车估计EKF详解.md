# SP_Vision 整车估计 EKF 详解

> 📘 **母文档**：[[SP_Vision_学习指南]]（先读母文档建立全局框架，本篇深化其 §6.3）
> 📘 **上游**：本篇 EKF 的观测输入（`Armor` 世界坐标）来自 [[SP_Vision_装甲板识别与解算详解]]
> 📘 **下游**：EKF 输出的整车状态（旋转中心 + 角速度）进入 [[SP_Vision_轨迹规划器详解]]，用于预测射击轨迹
> 📘 **共用**：同一个通用 EKF 类也被打符复用，见 [[SP_Vision_打符与全向感知详解]] §5

本篇聚焦自瞄流水线的**目标状态估计级**：把一连串装甲板世界坐标观测，融合成"敌方整车绕竖直轴旋转"的运动模型。核心问题是"**为什么估整车而非单块装甲板**"、"**11 维状态的雅可比怎么来**"、"**如何用自适应噪声和卡方检验保证鲁棒**"。

---

## 目录

1. [核心思想：估整车，不估单板](#1-核心思想估整车不估单板)
2. [通用 EKF 工具类：注入式设计](#2-通用-ekf-工具类注入式设计)
3. [11 维状态向量与初始化](#3-11-维状态向量与初始化)
4. [状态转移 f 与雅可比 F](#4-状态转移-f-与雅可比-f)
5. [观测方程 h 与雅可比 H](#5-观测方程-h-与雅可比-h)
6. [自适应观测噪声 R](#6-自适应观测噪声-r)
7. [数据关联：看到哪块更新哪块](#7-数据关联看到哪块更新哪块)
8. [Tracker 状态机与鲁棒性护栏](#8-tracker-状态机与鲁棒性护栏)
9. [NIS 卡方检验](#9-nis-卡方检验)
10. [概念关系图谱](#10-概念关系图谱)
11. [相关笔记](#11-相关笔记)

---

## 1. 核心思想：估整车，不估单板

朴素做法是"看到一块装甲板就追一块"。但 RM 场景有两个致命问题：

1. **小陀螺换板**：敌方旋转时装甲板轮替进出视野，单板跟踪会在换板瞬间"目标丢失/重生"，位置估计剧烈跳变。
2. **打提前量**：要预测"下一块即将转出来的装甲板"在哪，单板模型根本没有这个信息。

sp_vision 的解法（源自陈君 rm_vision）：把敌方机器人建模为**绕竖直轴旋转的刚体，N 块装甲板均匀分布在圆周上**。EKF 估计的不是某块装甲板，而是这个刚体的**旋转中心 (x, y, z)、角速度 w、当前相位 a、半径 r**。

带来三个好处：
- **换板连续**：无论看到哪块装甲板，都用来更新"同一个旋转中心"，只是相位偏移不同（第 id 块相位 = `a + id·2π/N`）。中心估计因此平滑不跳。
- **能预测不可见板**：知道旋转中心和角速度，任意时刻任意一块装甲板的位置都能算出——这正是打提前量的基础。
- **一套状态覆盖不对称车**：真实整车装甲板常有长短轴（半径不等）和高低差，用 `r`/`r+l` 两组半径、`z`/`z+h` 两组高度交替描述相邻装甲板。

> **一句话速记**：EKF 估的是"旋转中心 + 角速度 + 半径 + 相位"的整车模型，换板只是切相位偏移 `id·2π/N`，因此中心估计连续、能预测不可见板、能打提前量。

---

## 2. 通用 EKF 工具类：注入式设计

`tools/extended_kalman_filter.hpp/.cpp` 是一个**不含任何整车知识**的通用 EKF，整车模型、打符模型都复用它。解耦靠"注入"实现。

### 2.1 注入了什么

关键设计：**非线性函数用 `std::function` 闭包注入，雅可比矩阵作参数传入**（由上层算好）。

| 注入项 | 类型 | 默认 | 出处 |
|:----|:----|:----|:----|
| `x_add(a, b)` | 闭包（构造时注入） | `a + b` | `extended_kalman_filter.hpp:21-22, 48` |
| `f(x)` | 闭包（predict 参数） | `F·x`（线性） | `.hpp:26-28`、`.cpp:23-26` |
| `h(x)` | 闭包（update 参数） | `H·x`（线性） | `.hpp:35-39`、`.cpp:37-42` |
| `z_subtract(a, b)` | 闭包（update 参数） | `a − b` | `.hpp:32-33` |
| `F`、`H` | `const MatrixXd&` 参数 | 无（每次传入） | 每次 predict/update |

**为什么 x_add / z_subtract 要能自定义**：状态里有角度（yaw、pitch、朝向角），角度做加减必须归一化到 (−π, π]，否则会出现 359° + 2° = 361° 的错误。整车 EKF 注入的 `x_add` 对第 6 维（yaw）做 `limit_rad`，`z_subtract` 对 yaw/pitch/朝向角做 `limit_rad`。这是"角度安全"的 EKF——通用类留出接口，具体模型填角度语义。

### 2.2 五个核心公式与实现

标准 EKF 的预测-更新循环，全部在 `extended_kalman_filter.cpp`：

| 步骤 | 公式 | 实现 |
|:----|:----|:----|
| 协方差先验 | $P^- = FPF^T + Q$ | `.cpp:32` |
| 状态先验 | $\hat{x}^- = f(\hat{x})$ | `.cpp:33` |
| 卡尔曼增益 | $K = P^- H^T(HP^-H^T + R)^{-1}$ | `.cpp:50` |
| 状态后验 | $\hat{x} = x\_add(\hat{x}^-,\ K \cdot z\_subtract(z,\ h(\hat{x}^-)))$ | `.cpp:56` |
| 协方差后验 | $P = (I-KH)P^-(I-KH)^T + KRK^T$ | `.cpp:54` |

**Joseph 形式协方差更新**（`.cpp:54`）是数值稳定关键：

```cpp
P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();
```

比朴素的 `P = (I − KH)P` 计算量大，但**保证协方差矩阵恒正定对称**（浮点误差不会让它失去正定性导致滤波发散）。注释引用 rlabbe 的 "Stable Computation of the Posterior Covariance"。

初始化接口 `ExtendedKalmanFilter(x0, P0, x_add)`（`.hpp:19-22`），成员 `I = Identity(x0.rows())`（`.cpp:10`），公有 `x`、`P` 供外部读写。

> **一句话速记**：通用 EKF 类用 `std::function` 注入 f/h 与角度安全的 x_add/z_subtract，雅可比 F/H 由上层传入；协方差更新用 Joseph 形式保正定，因此整车和打符能共用同一个类。

---

## 3. 11 维状态向量与初始化

### 3.1 状态定义（target.cpp:34-39）

```
x = [ x, vx, y, vy, z, vz, a, w, r, l, h ]
      0   1  2   3  4   5  6  7  8  9  10
```

| 下标 | 符号 | 含义 |
|:----|:----|:----|
| 0,1 | x, vx | 旋转中心 X 坐标与速度 |
| 2,3 | y, vy | 旋转中心 Y 坐标与速度 |
| 4,5 | z, vz | 基准装甲板高度与速度 |
| 6 | a (yaw) | 基准装甲板法向朝向角（整车相位，过 limit_rad） |
| 7 | w (vyaw) | 整车绕竖直轴角速度 |
| 8 | r | 短轴半径 r1（收敛判据要求 0.05~0.5m） |
| 9 | l | 半径差 r2 − r1（长短轴现象，仅 4 板车 id=1/3 用） |
| 10 | h | 高度差 z2 − z1（双半径装甲板高度偏移） |

`l`、`h` 的存在是为了描述**不对称整车**：多数 4 板车的相邻装甲板半径、高度不完全相等（长短轴 + 高低差），于是用 `r`/`r+l` 两组半径和 `z`/`z+h` 两组高度交替表示（详见 §5 的 `use_l_h` 切换）。2 板平衡步兵、3 板前哨站不用 l/h。

### 3.2 初始化：用首帧观测反推

首次检测到目标时，`tracker.cpp:231-264 set_target` 按兵种选半径、板数、初始协方差，`target.cpp:24-49` 构造函数用首帧观测反推旋转中心：

```cpp
// target.cpp:30-32 由观测装甲板 + 假定半径反推旋转中心
center_x = xyz[0] + r * cos(ypr[0]);   // 装甲板在圆周上，中心在其法向反方向 r 处
center_y = xyz[1] + r * sin(ypr[0]);
center_z = xyz[2];
// target.cpp:39 初始 11 维（速度、l、h 全 0，yaw = 观测朝向角）
x0 = {center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0};
```

各兵种初始化参数（`tracker.cpp:238-261`）：

| 兵种 | radius | armor_num | P0 对角（节选） |
|:----|:----|:----|:----|
| 平衡步兵（big 且 3/4/5 号） | 0.2 | 2 | `{1,64,...,0.4,100,1,1,1}` |
| 前哨站 | 0.2765 | 3 | `{...,0.4,100,1e-4,0,0}` |
| 基地 | 0.3205 | 3 | `{...,0.4,100,1e-4,0,0}` |
| 普通（步兵/英雄） | 0.2 | 4 | `{1,64,...,0.4,100,1,1,1}` |

注意速度维初始方差大（64、100），因为初速度完全未知；前哨站/基地的 r 维方差极小（1e-4）且 l/h 为 0，因为它们半径固定、板对称。

> **一句话速记**：首帧用"装甲板位置 + 假定半径沿法向反推"得旋转中心，速度与 l/h 初始化为 0、方差给大；按兵种设板数（2/3/4）和半径。

---

## 4. 状态转移 f 与雅可比 F

### 4.1 匀速模型

整车用**常速度模型**（constant velocity）：旋转中心 x/y/z 匀速平移、相位 a 匀速旋转，半径 r/l/h 不变。状态转移雅可比 F 是 11×11（`target.cpp:79-91`）：

```
      x  vx  y  vy  z  vz  a   w   r  l  h
x   [ 1  dt                              ]   ← x  += vx·dt
vx  [    1                               ]
y   [       1  dt                        ]   ← y  += vy·dt
vy  [          1                         ]
z   [             1  dt                  ]   ← z  += vz·dt
vz  [                1                   ]
a   [                   1  dt            ]   ← a  += w·dt（相位匀速旋转）
w   [                      1             ]
r   [                         1          ]   ← r/l/h 保持不变
l   [                            1       ]
h   [                               1    ]
```

四组"位置-速度对" (x,vx)、(y,vy)、(z,vz)、(a,w) 都是 `F(i, i+1) = dt`；r、l、h 三维对角为 1、无速度项。

### 4.2 f 闭包：唯一的非线性来自 yaw 归一化

严格说这个模型对状态是线性的，但 yaw 需要归一化，所以仍走非线性 f（`target.cpp:125-129`）：

```cpp
auto f = [&](const Eigen::VectorXd & x) {
  Eigen::VectorXd x_prior = F * x;
  x_prior[6] = tools::limit_rad(x_prior[6]);   // 唯一的非线性：yaw 归一化到 (−π, π]
  return x_prior;
};
```

`limit_rad` 把角度限制在 (−π, π]（`math_tools.cpp:8-13`）。加上注入的 `x_add` 也对第 6 维做 limit_rad，双重保证 EKF 内部角度不会累积绕环。

### 4.3 过程噪声 Q：分段白噪声模型

Q 用 Piecewise White Noise Model（`target.cpp:94-122`），把"加速度是白噪声"的假设离散化。系数（`target.cpp:104-106`）：

$$a = \frac{dt^4}{4},\quad b = \frac{dt^3}{2},\quad c = dt^2$$

对每个位置-速度对填入 $\sigma^2 \begin{bmatrix} a & b \\ b & c \end{bmatrix}$ 分块。噪声强度按目标类型区分：

| 目标 | 加速度方差 v1 | 角加速度方差 v2 | 出处 |
|:----|:----|:----|:----|
| 普通目标 | 100 | 400 | `target.cpp:100-103` |
| 前哨站 | 10 | 0.1 | `target.cpp:97-99` |

前哨站噪声给得极小，因为它转速恒定、几乎不加速——低过程噪声让滤波更信任模型、更平滑。r、l、h 三维过程噪声全 0（半径不该随机漂移）。

> **一句话速记**：整车是常速度模型，F 里四组位置-速度对各配 `F(i,i+1)=dt`、r/l/h 不变；唯一非线性是 yaw 的 limit_rad；Q 用分段白噪声、前哨站噪声调到极小以获得平滑。

---

## 5. 观测方程 h 与雅可比 H

### 5.1 从整车状态到装甲板观测

观测量是**某一块装甲板的球坐标 + 朝向角**，4 维：`z = {yaw, pitch, distance, 朝向角}`（`target.cpp:220`）。观测函数 h 把整车状态映射成第 id 块装甲板的这 4 个量（`target.cpp:202-207`）：

```cpp
auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
  Eigen::VectorXd xyz = h_armor_xyz(x, id);        // 整车状态 → 第 id 块 xyz
  Eigen::VectorXd ypd = tools::xyz2ypd(xyz);        // xyz → 球坐标
  auto angle = tools::limit_rad(x[6] + id * 2*CV_PI / armor_num_);   // 该块朝向角
  return {ypd[0], ypd[1], ypd[2], angle};          // yaw, pitch, distance, 朝向角
};
```

第 id 块装甲板中心的世界坐标（`h_armor_xyz`，`target.cpp:267-277`）：

```cpp
angle = limit_rad(x[6] + id * 2π / armor_num);          // 该块相位
use_l_h = (armor_num == 4) && (id == 1 || id == 3);      // 长短轴切换条件
r = use_l_h ? x[8] + x[9] : x[8];                        // 短轴 r 或长轴 r+l
armor_x = x[0] - r * cos(angle);                         // 中心沿相位反方向偏 r
armor_y = x[2] - r * sin(angle);
armor_z = use_l_h ? x[4] + x[10] : x[4];                 // 高度 z 或 z+h
```

**长短轴切换 `use_l_h`**：只有 4 板车、且 id 为 1 或 3（相邻两组）时才用长轴 `r+l` 和高度 `z+h`，其余用短轴 `r`、高度 `z`。这样一套状态就覆盖了 4 板车"两长两短、两高两低"的不对称结构。

球坐标转换 `xyz2ypd`（`math_tools.cpp:103-110`）：

$$\text{yaw} = \operatorname{atan2}(y, x),\quad \text{pitch} = \operatorname{atan2}(z, \sqrt{x^2+y^2}),\quad \text{distance} = \sqrt{x^2+y^2+z^2}$$

`armor_num` 由兵种决定（`tracker.cpp`）：平衡步兵 2、前哨站/基地 3、普通 4。

### 5.2 观测雅可比 H：解析链式法则

H 是 4×11，**解析计算**（非数值差分），用两级链式相乘（`target.cpp:280-317 h_jacobian`）：

$$H = \underbrace{H_{ypda}}_{4\times4} \cdot \underbrace{H_{xyza}}_{4\times11}$$

**第一级** $H_{xyza}$（整车状态 → 装甲板 [x, y, z, angle]，`target.cpp:296-303`）对 §5.1 的 `armor_x/y/z/angle` 求偏导：

```
dx_da = r·sin(angle);   dy_da = −r·cos(angle);     // 对相位 a=x[6]
dx_dr = −cos(angle);    dy_dr = −sin(angle);       // 对半径 r=x[8]
dx_dl = use_l_h ? −cos(angle) : 0;                  // 对 l=x[9]（仅长轴块非零）
dy_dl = use_l_h ? −sin(angle) : 0;
dz_dh = use_l_h ? 1.0 : 0;                          // 对 h=x[10]
// 4 行分别是 armor_x / armor_y / armor_z / angle 对 11 维状态的偏导
// angle 行只有对 x[6] 的偏导 = 1
```

**第二级** $H_{ypda}$：把"装甲板 xyz → 球坐标 ypd"的雅可比 `xyz2ypd_jacobian`（解析式，`math_tools.cpp:112-137`）扩展成 4×4（第 4 行/列对 angle 直通 = 1，`target.cpp:308-313`）。

用解析雅可比而非数值差分，既快又准（避免差分步长选择问题）。

### 5.3 残差归一化

`z_subtract` 对观测残差的角度分量做 limit_rad（`target.cpp:210-216`）：yaw（c[0]）、pitch（c[1]）、朝向角（c[3]）都归一化，distance（c[2]）不处理（它不是角度）。这与 §2.1 的角度安全设计呼应。

> **一句话速记**：h 把整车状态映射成第 id 块装甲板的 (yaw, pitch, distance, 朝向角)，中心沿相位反方向偏 r、4 板车 id=1/3 用长轴 r+l 和高度 z+h；H 用解析链式 `H_ypda·H_xyza` 算，残差里三个角度分量都做 limit_rad。

---

## 6. 自适应观测噪声 R

这是精度的关键设计：**几何越差的观测，自动降权**（`target.cpp:191-199`）。

```cpp
center_yaw  = atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);   // 中心视线方向
delta_angle = limit_rad(armor.ypr_in_world[0] - center_yaw);        // 装甲板法向与视线的夹角
R_dig = { 4e-3,                                    // yaw   固定
          4e-3,                                    // pitch 固定
          log(|delta_angle| + 1) + 1,              // distance 随朝向夹角增大
          log(|distance| + 1) / 200 + 9e-2 };      // 朝向角 随距离增大
```

四个观测量的噪声策略：

| 观测量 | R 值 | 物理直觉 |
|:----|:----|:----|
| yaw | 4e-3（固定） | 方位角观测始终可靠 |
| pitch | 4e-3（固定） | 俯仰角观测始终可靠 |
| distance | `log(|delta_angle|+1) + 1` | 装甲板越侧对相机（朝向夹角大），深度估计越不准 → 降权 |
| 朝向角 | `log(|distance|+1)/200 + 9e-2` | 目标越远，朝向角分辨率越低 → 降权 |

**为什么用 log**：夹角/距离增大时噪声单调上升但增速放缓，避免远处/侧对目标的 R 爆炸导致该观测被完全忽略。被注释掉的旧版是固定值 `{4e-3, 4e-3, 1, 9e-2}`（`target.cpp:191`），自适应版是精度改进。

> **一句话速记**：yaw/pitch 噪声固定 4e-3；distance 随"装甲板法向与视线夹角"增大（侧对时深度不准）、朝向角随距离增大（远处分辨率低），都用 log 平滑增长——几何差的观测自动降权。

---

## 7. 数据关联：看到哪块更新哪块

一帧可能看到多块装甲板，EKF 每次只用一块更新。要决定"这块观测对应整车的第几号相位"（`target.cpp:138-185`）：

1. **生成全部虚拟装甲板**（`armor_xyza_list`，`target.cpp:229-239`）：对 i = 0..armor_num−1，各算一块的预测 xyz 和朝向角（相位 `x[6] + i·2π/N`）。
2. **按距离升序排序**（`target.cpp:150-156`），取 distance 最小的前 3 块作候选（`target.cpp:158-169`，近的观测更可靠）。
3. **关联度量**（`target.cpp:162-163`）= 朝向角误差 + yaw 误差：

```cpp
angle_error = |limit_rad(armor.ypr_in_world[0] - xyza[3])|      // 观测朝向 vs 预测朝向
            + |limit_rad(armor.ypd_in_world[0] - ypd[0])|;      // 观测 yaw  vs 预测 yaw
```

取 `angle_error` 最小的 id。
4. **副作用**（`target.cpp:171-182`）：id≠0 标 `jumped`；id≠上次 id 标 `is_switch_`、`switch_count_++`；然后用该 id 的 h/H 调 `update_ypda` 做一次 EKF 更新。

整车模型的妙处在这里体现：**无论关联到哪块，更新的都是同一个旋转中心**，只是用不同相位偏移的 h/H。所以换板对中心估计是平滑的。

> **一句话速记**：生成所有虚拟板 → 取最近 3 块 → 按"朝向角误差 + yaw 误差"最小关联到某个 id → 用该 id 的 h/H 更新同一旋转中心，换板只换相位。

---

## 8. Tracker 状态机与鲁棒性护栏

### 8.1 五状态机

Tracker 用字符串表示状态（无 enum，`tracker.cpp:180-229`）：

```
                    found
        lost ──────────────▶ detecting ──连续 min_detect_count 帧 found──▶ tracking
         ▲                      │ 未found                                    │ 未found
         │                      ▼                                            ▼
         └──────────────────── lost                                     temp_lost
         ▲                                                                   │
         │◀── temp_lost_count 超 max_temp_lost_count ────────────────────────┤
         │                                                          found    │
         │                                              tracking ◀───────────┘
      switching（全向感知/主相机出现更高优先级目标，待收敛后切入）
```

| 转移 | 条件 | 出处 |
|:----|:----|:----|
| lost → detecting | found，detect_count=1 | `tracker.cpp:182-187` |
| detecting → tracking | 连续 found 且 detect_count ≥ min_detect_count | `:189-192` |
| detecting → lost | 未 found | `:193-196` |
| tracking → temp_lost | 未 found，temp_lost_count=1 | `:199-204` |
| temp_lost → tracking | found | `:216-217` |
| temp_lost → lost | temp_lost_count 超 max_temp_lost_count | `:218-227` |
| switching → detecting | found | `:206-208` |
| switching → lost | temp_lost_count > 200 | `:209-212` |

参数（`configs/*.yaml`，读于 `tracker.cpp:23-26`）：

| 参数 | 典型值 | 含义 |
|:----|:----|:----|
| min_detect_count | 5 | detecting→tracking 所需连续帧 |
| max_temp_lost_count | 15（sentry 25） | temp_lost→lost 容忍帧 |
| outpost_max_temp_lost_count | 75（uav 150） | 前哨站专用容忍帧 |

**detecting 与 temp_lost 的分工**：detecting 是"刚看到、还没确认"的门槛（连续 5 帧才升为 tracking，防误识别），temp_lost 是"跟踪中短暂丢失"的缓冲（容忍 15 帧再降为 lost，防眨眼丢帧）。

**前哨站特判**（`tracker.cpp:220-224`）：temp_lost 时若目标是前哨站，把 `max_temp_lost_count` 换成 `outpost_max_temp_lost_count`（75/150）——前哨站转速稳定、被遮挡时间可预测，容忍更久不轻易丢。

### 8.2 三道强制转 lost 护栏

除状态机正常流转外，三个条件直接强制 lost：

1. **相机断帧**：`state ≠ lost && dt > 0.1s` → lost（`tracker.cpp:38-41, 112-115`）。图像间隔过大说明采集异常，估计已不可信。
2. **物理发散** `diverged()`（`target.cpp:241-250`）：半径超出物理区间即发散。

```cpp
r_ok = x[8] > 0.05 && x[8] < 0.5;                        // 短轴 r ∈ (0.05, 0.5)m
l_ok = x[8]+x[9] > 0.05 && x[8]+x[9] < 0.5;              // 长轴 r+l ∈ (0.05, 0.5)m
if (r_ok && l_ok) return false;  // 都在区间才不发散
```

半径是最强的物理先验——真实装甲板旋转半径就在 5cm~50cm。EKF 若把半径估到区间外，一定是错的，直接丢。

3. **NIS 卡方失败率** ≥ 40%（见 §9）。

> **一句话速记**：五状态机（lost/detecting/tracking/temp_lost/switching）用 min_detect_count 防误识别、max_temp_lost_count 防眨眼丢帧、前哨站放宽容忍；另有三道护栏——断帧 dt>0.1s、半径出 (0.05,0.5)m、NIS 失败率≥40% 强制转 lost。

---

## 9. NIS 卡方检验

NIS（Normalized Innovation Squared，归一化新息平方）是判断"滤波是否还相信自己"的统计量（`extended_kalman_filter.cpp:58-89` + `tracker.cpp:82-90`）。

### 9.1 计算

```cpp
residual = z_subtract(z, h(x));           // 更新后的观测残差（新息）
S = H * P * Hᵀ + R;                        // 新息协方差
nis = residualᵀ · S⁻¹ · residual;         // 归一化新息平方（标量）
```

直觉：残差本身有量纲、大小取决于观测精度；用新息协方差 S 归一化后，NIS 服从卡方分布。若模型与观测吻合，NIS 应落在卡方分布的合理区间；若 NIS 持续偏大，说明预测和观测系统性不符——模型可能已经追错目标。

### 9.2 阈值与触发

阈值 `nis_threshold = 0.711`（`.cpp:65-67`，注释称自由度 4、95% 置信）。单帧 `nis > 0.711` 记一次失败，push 进容量 100 的滑窗 `recent_nis_failures`（`.cpp:74-78`）。

触发 lost（`tracker.cpp:83-90`）：

```cpp
if (近 100 帧 NIS 失败数 >= 0.4 * 100)    // 失败率 ≥ 40%
  state_ = "lost";
```

**为什么用滑窗失败率而非单帧**：单帧 NIS 偶尔超阈值是正常噪声，不能一超就丢；但近 100 帧里有 40 帧都不吻合，说明是系统性错误（追错了目标或模型发散），此时才丢。这是"统计意义上的丢失判定"，比单帧阈值鲁棒得多。

代码里也算了 NEES（用状态误差而非观测残差），但 tracker 只用 NIS 触发 lost，NEES 仅记录供调试（`.cpp:63, 70`）。

> **一句话速记**：NIS = 残差经新息协方差 S 归一化的卡方量，衡量"预测与观测吻合度"；单帧超 0.711 记一次失败，近 100 帧失败率 ≥ 40% 才判 lost——用统计滑窗而非单帧，避免噪声误判。

---

## 10. 概念关系图谱

### 10.1 EKF 一帧的完整流转

```
                        ┌─────────── 通用 EKF 类（tools/extended_kalman_filter）───────────┐
                        │  注入: f, h, F, H, x_add(yaw limit), z_subtract(角度 limit)      │
                        └──────────────────────────────────────────────────────────────┘
   Armor 世界坐标(观测)                    ▲                              │
        │                                 │                              ▼
        ▼                          ┌──────┴──────┐              ┌────────────────┐
   数据关联(§7)                    │ predict(§4) │              │  update(§5,§6) │
   生成虚拟板→取最近3块            │ x⁻=f(x)     │              │ K=P⁻Hᵀ(HP⁻Hᵀ+R)⁻¹│
   朝向+yaw误差最小→id             │ P⁻=FPFᵀ+Q   │              │ 自适应 R(§6)    │
        │                          │ 前哨站钳位w  │              │ Joseph 更新 P   │
        ▼                          └─────────────┘              └────────────────┘
   用该 id 的 h/H 更新                                                   │
   同一旋转中心                                                          ▼
        │                                            ┌──────── 鲁棒护栏(§8,§9)────────┐
        └───────────────────────────────────────────▶ 断帧dt>0.1 / diverged半径 / NIS≥40%│
                                                     └──────────────────────────────────┘
                                                                        │
   状态机(§8): lost→detecting→tracking→temp_lost                        ▼
   输出 Target(旋转中心+w) ──▶ 轨迹规划器预测射击轨迹
```

### 10.2 前哨站特判汇总

前哨站（转速恒定 ~2.51 rad/s）在多处特殊处理：

| 特判 | 值 | 出处 |
|:----|:----|:----|
| 转速钳位 | 收敛后 \|w\|>2 时钳到 ±2.51 rad/s | `target.cpp:131-133` |
| 过程噪声 | v1=10, v2=0.1（远小于普通的 100/400） | `target.cpp:97-99` |
| 收敛门槛 | update_count > 10（普通 >3） | `target.cpp:254, 259` |
| 丢失容忍 | outpost_max_temp_lost_count = 75/150 | `tracker.cpp:220-224` |
| 板数/半径 | armor_num=3, radius=0.2765 | `tracker.cpp:250` |

> ⚠️ 注意：母文档 §6.3 写前哨站钳位 "2.512 rad/s"，**代码实际是 2.51**（`target.cpp:133`）。前哨站真实转速约 0.4 r/s ≈ 2.513 rad/s，代码取了近似值 2.51。

### 10.3 核心概念对照

| 易混概念 | 区别 |
|:----|:----|
| **单块装甲板 vs 整车模型** | EKF 不估单板，估旋转中心+角速度+半径+相位；换板只换相位偏移 id·2π/N |
| **f 的非线性 vs 线性** | 整车常速度模型本质线性(F·x)，唯一非线性是 yaw 的 limit_rad |
| **r vs r+l / z vs z+h** | 4 板车 id=1/3 用长轴 r+l、高度 z+h，其余用短轴 r、高度 z（描述不对称整车） |
| **解析 H vs 数值 H** | 本项目解析链式 H_ypda·H_xyza，比数值差分快且准 |
| **固定 R vs 自适应 R** | yaw/pitch 固定；distance 随朝向夹角、朝向角随距离用 log 增长，几何差的降权 |
| **detecting vs temp_lost** | detecting=刚看到未确认(防误识别)；temp_lost=跟踪中短暂丢(防眨眼) |
| **diverged vs NIS 触发 lost** | diverged=物理先验(半径区间)；NIS=统计先验(近100帧40%失败率) |
| **单帧 NIS vs 滑窗失败率** | 单帧超阈值是正常噪声；近100帧40%失败才判系统性错误→lost |
| **Joseph 形式 vs 朴素更新** | Joseph 计算量大但保协方差正定对称，防滤波发散 |

### 10.4 全局一句话速记

> 把敌方建成"绕竖直轴旋转的整车"，EKF 估**旋转中心+角速度+半径+相位**（11 维），换板只切相位偏移 id·2π/N；通用 EKF 类靠 std::function 注入 f/h 与角度安全运算、Joseph 形式保正定；观测是某块装甲板的球坐标+朝向角，H 解析链式算，R 自适应降权几何差的观测；数据关联取最近板按角度误差匹配；鲁棒性靠状态机 + 断帧/半径发散/NIS 卡方三道护栏。

---

## 11. 相关笔记

- [[SP_Vision_学习指南]] — 母文档，全局框架（本篇深化 §6.3）
- [[SP_Vision_装甲板识别与解算详解]] — 本篇 EKF 的观测输入（Armor 世界坐标）由它产出，含 yaw 优化细节
- [[SP_Vision_轨迹规划器详解]] — EKF 输出的整车状态经 `target.predict(t)` 平移成射击轨迹，是规划器的输入
- [[SP_Vision_打符与全向感知详解]] — 打符的 Big/SmallTarget 复用同一个通用 EKF 类，可对照定轴旋转 vs 整车两种模型
- [[SP_Vision_IO与多线程架构详解]] — EKF 观测所需的云台姿态由 `imu_at` 按图像时间戳 slerp 插值提供
- [[MAS_Vision_位姿解算与目标跟踪教程]] — 本战队自研的目标跟踪，可与整车 EKF 对照
- [[RM_C程序员_C++速通指南]] — `std::function` 注入、Eigen 矩阵运算等本篇技法
