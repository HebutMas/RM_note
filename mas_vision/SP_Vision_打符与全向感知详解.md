# SP_Vision 打符与全向感知详解

> 📘 **母文档**：[[SP_Vision_学习指南]]（先读母文档建立全局框架，本篇深化其 §6.5）
> 📘 **共用组件**：正弦拟合与旋转模型都建立在通用 EKF 之上，见 [[SP_Vision_整车估计EKF详解]]（同一个 `tools::ExtendedKalmanFilter` 类）
> 📘 **相关**：[[SP_Vision_装甲板识别与解算详解]]（打符解算同样用 IPPE PnP，可对照）、[[SP_Vision_轨迹规划器详解]]（打符 mpc_aim 也走轨迹规划）

本篇聚焦两个"非自瞄主线"的功能组：**打符（auto_buff）**——识别能量机关、解算 R 标、预测扇叶正弦转速；**全向感知（omniperception）**——多相机广域搜敌、给云台转向指引。核心难点是"**大符变速旋转如何预测**"与"**多相机像素坐标如何换算成云台角度**"。

---

## 目录

1. [打符全局：从识别到开火的五级流水](#1-打符全局从识别到开火的五级流水)
2. [识别：YOLO11_BUFF 与 R 标精定位](#2-识别yolo11_buff-与-r-标精定位)
3. [扇叶归类：定 target 与五等分排序](#3-扇叶归类定-target-与五等分排序)
4. [位姿解算：5 点 IPPE 与定轴旋转模型](#4-位姿解算5-点-ippe-与定轴旋转模型)
5. [大符正弦模型：EKF 内嵌 vs RANSAC 拟合](#5-大符正弦模型ekf-内嵌-vs-ransac-拟合)
6. [小符与开火决策](#6-小符与开火决策)
7. [全向感知：多相机并行架构](#7-全向感知多相机并行架构)
8. [Decider：方位换算、过滤与优先级](#8-decider方位换算过滤与优先级)
9. [概念关系图谱](#9-概念关系图谱)
10. [相关笔记](#10-相关笔记)

---

## 1. 打符全局：从识别到开火的五级流水

打符（击打能量机关）与自瞄共用相机与云台，但目标运动模型完全不同：能量机关是**绕固定 R 标旋转的五扇叶风车**，只需击打当前"激活"的那一片。流水线五级：

```
YOLO11_BUFF 识别 ──▶ 扇叶归类(定target+排序) ──▶ IPPE 解算 ──▶ 正弦转速预测 ──▶ 弹道+开火
 buff/r 两类           PowerRune                  buff→world         Big/SmallTarget    buff_aimer
 每扇叶6关键点          5等分角                    R标球坐标定圆心      spd=a·sin(wt+φ)+b
```

主程序有两套调用入口（`src/auto_buff_debug.cpp:72`、`src/auto_buff_debug_mpc.cpp:73`）：前者用 `aimer.aim()` 直接发角度命令，后者用 `aimer.mpc_aim()` 走轨迹规划器（与自瞄同一套 MPC）。两者默认都用 `SmallTarget`（`auto_buff_debug.cpp:47-48`，`BigTarget` 被注释，需要时手动切换）。

> **一句话速记**：打符 = "识别扇叶+R标 → 定当前激活扇叶 → IPPE 解出定轴旋转模型 → 预测正弦转速 → 提前量开火"，与自瞄的区别在目标模型（定轴单叶旋转 vs 整车平移+多板）。

---

## 2. 识别：YOLO11_BUFF 与 R 标精定位

### 2.1 模型输出结构

打符用独立的 YOLO11_BUFF 模型（`tasks/auto_buff/yolo11_buff.hpp`），只有**两类**：

```cpp
const std::vector<std::string> class_names = {"buff", "r"};  // yolo11_buff.hpp:13
const int NUM_POINTS = 6;                                    // yolo11_buff.hpp:40 每目标6关键点
```

模型输出布局 `[15, 8400]`，其中 `15 = 4(框 xywh) + 1(置信度) + 6×?`（`yolo11_buff.cpp:65-66` 注释；实际关键点解析取前若干个）。置信度阈值 `0.7`、IoU 阈值 `0.4`（`yolo11_buff.cpp:3-4`），输入 640×640，OpenVINO CPU 推理（`:14-19`），NMS 于 `:103`。

关键点里 **第 5 个点 `kpt[4]` 被当作扇叶中心**（`buff_detector.cpp:105` 构造 `FanBlade(result.kpt, result.kpt[4], _light)`）。

`buff_detector` 有三个入口：`detect_24`（多框 NMS，`:89`）、`detect`（取最高置信度单框，`:125`，调试主程序实际调用）、`detect_debug`（`:162`）。丢失容忍阈值 `LOSE_MAX=20`（`buff_detector.hpp:12`）。

### 2.2 R 标（旋转中心）两步定位

R 标是能量机关的旋转圆心，是所有解算的原点。模型虽有 `r` 类，但为提高精度用"**几何外推初值 + 二值化轮廓精定位**"两步（`buff_detector.cpp:27-77` `get_r_center()`）：

**第一步 · 几何外推初值**（`:40-45`）：R 标在扇叶中心的延长线上。取扇叶第 5、6 关键点方向外推：

```cpp
point5 = points[4];                         // 扇叶中心
point6 = points[5];
r_center_t += (point6 - point5) * 1.4 + point5;   // 沿 5→6 方向外推 1.4 倍
// 多扇叶时取均值
```

**第二步 · 二值化精定位**（`:9-25 handle_img` + `:51-75`）：
1. 灰度 → 阈值 100 二值化 → 5×5 矩形核膨胀（`handle_img`）
2. 以 `radius = norm(points[2]-center)*0.8` 画圆形 mask，与二值图 `bitwise_and` 把搜索限定在 R 标应在的环带（`:51-54`）
3. `findContours(RETR_EXTERNAL)`，对每个轮廓 `minAreaRect`，用评价量选最优（`:60-75`）：

```cpp
ratio = 长宽比 + norm(rect.center - r_center_t) / (radius/3);   // 越接近正方形且越靠近外推初值越优
```

取 `ratio` 最小者的 `rotated_rect.center` 作为精定位 R 中心。

**为什么两步**：模型直接回归的 R 标框有像素抖动，而 R 标在图像上是个近似圆形亮块，用几何约束（在扇叶延长线上）圈定范围 + 形状约束（长宽比接近 1）精定位，比纯模型输出稳定得多。

> **一句话速记**：R 标 = "扇叶中心延长线外推 1.4 倍得初值 → 圆环 mask 内找最接近正方形且离初值最近的轮廓精定位"，几何+形状双约束抗抖动。

---

## 3. 扇叶归类：定 target 与五等分排序

识别出若干扇叶后，需要回答两个问题：**哪一片是当前要打的激活扇叶（target）**？以及**如何把五片扇叶按固定角度顺序排列**？这两件事在 `PowerRune` 构造函数完成（`buff_type.cpp:20-94`）。

### 3.1 定 target：追新亮起的那片

能量机关规则下，"激活"的扇叶会新亮起，是要击打的目标。分三种情况（`buff_type.cpp:27-71`）：

| 情况 | 判据 | 定 target 策略 | 行号 |
|:----|:----|:----|:----|
| 单扇叶 | 只识别到 1 片 | 直接设为 target | `:27` |
| 无新亮起 | 当前亮数 == 上一帧亮数 | 取与上一帧 target **中心距离最近**者（追踪连续性） | `:29-42` |
| 有新亮起 | 当前亮数 == 上一帧 + 1 | **最大最小距离准则**：对每片当前扇叶算它到所有旧亮扇叶的最小距离，取该最小距离**最大**者（离已有扇叶最远 = 新亮那片） | `:44-65` |
| 其他 | 亮数异常跳变 | 报错 `unsolvable_ = true` | `:67-71` |

定为 target 后 `std::iter_swap(ts.begin(), ...)` 把它放到列表首位（`fanblades[0]` 恒为 target，`buff_type.hpp:59`）。

"最大最小距离"的直觉：新亮扇叶不会和已经亮着的扇叶重叠，所以它离所有旧扇叶都比较远——在"到旧扇叶的最近距离"这个指标上，新扇叶的值最大。

### 3.2 五等分角排序

能量机关五片扇叶均匀分布，间隔 `2π/5 = 72°`。以 target 为角度基准，把其余扇叶按相对角排到五个槽位（`buff_type.cpp:75-93`）：

```cpp
target_angles = {0, 2π/5, 4π/5, 6π/5, 8π/5};       // 五等分槽位
for (每片扇叶) rel_angle = atan_angle(center) - target_angle;  // 相对 target 的角度(负则+2π)
sort(扇叶, by rel_angle 升序);
for (每个槽位 target_angles[i])
  若存在扇叶 |rel_angle - target_angles[i]| < π/5 → 填实扇叶
  否则 → 填 FanBlade(_unlight) 占位;
```

`atan_angle` = 扇叶中心相对 R 标的 `atan2`，归一化到 [0, 2π]（`buff_type.cpp:96-101`）。容差 `π/5`（半个槽距）保证匹配唯一。没识别到的扇叶用 `_unlight` 占位，保持五槽结构完整。

枚举 `FanBlade_type { _target, _unlight, _light }`（`buff_type.hpp:18`）。

> **一句话速记**：定 target = "新亮扇叶离所有旧扇叶最远（最大最小距离）"；排序 = "以 target 为 0°，其余按相对角填进 {0,72°,144°,216°,288°} 五槽，缺的用 _unlight 占位"。

---

## 4. 位姿解算：5 点 IPPE 与定轴旋转模型

### 4.1 物点模型与 IPPE

打符解算在 `buff_solver.cpp:55-110`。物点定义 7 个点（buff 自身坐标系，单位 m，`buff_solver.hpp:54-58`），但**实际 PnP 只用 target 扇叶的 4 个关键点 + R 标共 5 点**（`buff_solver.cpp:72-80`）：

```cpp
image_points = target().points;     // 扇叶 4 关键点
image_points.push_back(r_center);   // + R 标中心
// 取前 4 对做 IPPE（与自瞄同一求解器）
cv::solvePnP(OBJECT_POINTS_FOURTH, image_points_fourth, K, D, rvec, tvec, false, SOLVEPNP_IPPE);
```

与自瞄一样用 `SOLVEPNP_IPPE`（共面点专用）。`OBJECT_POINTS` 里的 5 扇叶旋转外推、`compute_rotated_points` 当前被注释未启用（`buff_solver.cpp:44`）。

### 4.2 坐标变换链 buff→camera→gimbal→world

与自瞄的 camera→gimbal→world 完全同构（`buff_solver.cpp:82-109`）：

```cpp
R_buff2gimbal = R_camera2gimbal_ * R_buff2camera;                     // :96
xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;  // :97
R_buff2world  = R_gimbal2world_ * R_buff2gimbal;                      // :101
```

`R_gimbal2world_` 同样由电控发来的 IMU 四元数构造（`buff_solver.cpp:49-53`，`R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_`）。这个四元数依然是按图像时间戳插值得到的那一帧姿态（对齐算法见 [[SP_Vision_IO与多线程架构详解]]）。输出 R 标的世界坐标 `xyz_in_world`、球坐标 `ypd_in_world`、扇叶中心 `blade_xyz_in_world`、欧拉角 `ypr_in_world`。

### 4.3 定轴旋转模型（与自瞄整车模型的本质区别）

打符目标是"绕 R 标定轴旋转的单叶"，模型形式与自瞄的整车模型完全不同（`point_buff2world`，`buff_target.cpp:24-39`）：

```cpp
// R 标位置用球坐标 (R_dis, R_pitch, R_yaw) = (x[3], x[2], x[0]) 定圆心
// 扇叶由单一 roll 角旋转：R_buff2world = rotation_matrix(yaw=x[4], pitch=0, roll=x[5])
```

关键点：
- **圆心用球坐标**（R_yaw/R_pitch/R_dis），不像自瞄用笛卡尔 xyz
- **pitch 固定为 0**，只有一个 roll 角 `x[5]` 表示扇叶转到哪，由转速驱动
- **半径固定 0.7m**：瞄准点恒取 buff 系 `(0, 0, 0.7)`（`buff_target.cpp:281,339` 等多处），不作为状态估计

| | buff Target | 自瞄 Target |
|:----|:----|:----|
| 运动 | 定轴旋转（圆心不动） | 平移 + 旋转（整车） |
| 圆心 | R 标球坐标 (R_yaw,R_pitch,R_dis) | 笛卡尔 (x,y,z) + 速度 |
| 旋转 | 单一 roll 角 x[5] | 相位 a + 相邻板 id·2π/N |
| 半径 | 固定 0.7m（不估计） | 状态 r/r+l（估计，长短轴） |
| 转速 | 正弦(大符)/匀速(小符) | 匀速 w |
| 共用 | 同一个 `tools::ExtendedKalmanFilter` 通用 EKF 类 | |

> **一句话速记**：打符解算 = "5 点 IPPE（4 扇叶点 + R 标）→ buff→world 变换 → 定轴旋转模型（R 标球坐标定圆心、单 roll 角转、半径固定 0.7m）"，与自瞄共用 EKF 类但模型不同。

---

## 5. 大符正弦模型：EKF 内嵌 vs RANSAC 拟合

大符是打符最难的部分——转速不是常数，而是官方规定的正弦规律。项目并存**两套**预测实现。

### 5.1 官方正弦转速公式

$$\text{spd} = a\sin(\omega t + \varphi) + (2.09 - a)$$

即 `b = 2.09 - a`（`buff_target.cpp:454`，注释 `:481`）。官方约束（`buff_target.cpp:483-484`、`buff_predict.hpp:149-150`）：

| 参数 | 范围 | 初值 |
|:----|:----|:----|
| $a$（幅值） | [0.780, 1.045] | 0.9125 |
| $\omega$（角频率） | [1.884, 2.000] | 1.942 |
| $\varphi$（相位） | 任意 | 0 |
| spd 初值 | — | 1.1775（= 2.09 − 0.9125） |

发散判据把范围放宽 1.5 倍（`buff_target.cpp:394-400`）：`a > 1.045×1.5 或 a < 0.78/1.5 或 w > 2.0×1.5 或 w < 1.884/1.5` 判发散。

### 5.2 方案一 · EKF 内嵌正弦（BigTarget，10 维状态）

把正弦参数 a/w/φ 直接塞进状态向量估计（`buff_target.cpp:468-484`）：

```
x = [R_yaw, v_R_yaw, R_pitch, R_dis, yaw, roll(角度), spd, a, w, fi]   // 10 维
      x[0]   x[1]     x[2]    x[3]  x[4]  x[5]        x[6] x[7] x[8] x[9]
```

**预测用角度积分闭式解**（`buff_target.cpp:447-456` 非线性 f）：roll 是转速的时间积分，正弦项能闭式积分：

```cpp
// roll(t) = roll0 + ∫spd dt = roll0 + clockwise·[-a/w·cos(wt+φ) + a/w·cos(w·t0+φ) + (2.09-a)·dt]
x_prior[5] = limit_rad(roll + clockwise*(-a/w*cos(w*t+fi) + a/w*cos(w*lasttime+fi) + (2.09-a)*dt));
x_prior[6] = a*sin(w*t+fi) + 2.09 - a;   // spd 直接代公式
```

$\int a\sin(\omega t+\varphi)\,dt = -\frac{a}{\omega}\cos(\omega t+\varphi)$，所以角度增量有解析式，不需数值积分。状态转移雅可比 `A_` 的 spd 行含对 a/w/φ 的偏导 `[sin(wt+φ)-1, t·a·cos(wt+φ), a·cos(wt+φ)]`（`buff_target.cpp:419`）。

观测分两段：观测 1（`H1` 4×10）测 R 标 [R_yaw, R_pitch, R_dis, roll]（`:578-604`）；观测 2 用链式雅可比 `h_jacobian`（3×10，`:662-712`）测扇叶 [B_yaw, B_pitch, B_dis]（`:613-642`）。

### 5.3 方案二 · RANSAC 正弦拟合（ransac_sine_fitter）

思路不同：EKF 只估出**当前瞬时 spd**（`x[6]`），把 spd 时间序列喂给一个独立拟合器，反解出 a/w/φ（`tools/ransac_sine_fitter.cpp`）。

**数据流**（`buff_target.cpp:645-650`）：

```cpp
if (ekf_.x[6] < 2.1 && ekf_.x[6] >= 0) spd_fitter_.add_data(nowtime, ekf_.x[6]);  // 喂 EKF 估的 spd
spd_fitter_.fit();
fit_spd_ = sine_function(nowtime, A, omega, phi, C);   // 用拟合曲线值替代 EKF 原始 spd 做预测
```

**固定 ω 后线性最小二乘（SVD）**是关键技巧。正弦 `a·sin(ωt+φ)+b` 对 a/φ 是**非线性**的，但一旦固定 ω，用和差化积展开就变**线性**：

$$a\sin(\omega t + \varphi) + C = \underbrace{a\cos\varphi}_{A_1}\sin(\omega t) + \underbrace{a\sin\varphi}_{A_2}\cos(\omega t) + C$$

于是对 [A₁, A₂, C] 是线性最小二乘（`ransac_sine_fitter.cpp:68-89`）：

```cpp
X = [sin(ωt), cos(ωt), 1];                              // 设计矩阵
params = X.bdcSvd(ComputeThinU | ComputeThinV).solve(Y);  // SVD 解线性最小二乘
A   = sqrt(A1*A1 + A2*A2);                               // :47-52 换算回幅值
phi = atan2(A2, A1);                                    // 换算回相位
```

**RANSAC 外层**（`ransac_sine_fitter.cpp:35-62`）：ω 未知，所以每次迭代从 [1.884, 2.000] 随机采一个 ω + 随机取 3 点样本 → 固定 ω 解线性最小二乘 → 用 `|y - (A·sin(ωt+φ)+C)| < 0.5`（threshold）统计内点数（`:91-103`）→ 内点最多的解作为 best。迭代 `max_iterations=100`（`buff_target.cpp:357` 构造 `RansacSineFitter(100, 0.5, 1.884, 2.000)`）。数据超 150 点 `pop_front`（`:65`）、间隔 >5s 清空（`:23`）。

### 5.4 两方案对比

| | EKF 内嵌正弦（BigTarget 10维） | RANSAC 拟合（ransac_sine_fitter） |
|:----|:----|:----|
| a/w/φ 来源 | 直接进状态向量，EKF 递推估计 | EKF 只估瞬时 spd，拟合器反解 |
| 数学 | 非线性 EKF（雅可比含 a/w/φ 偏导） | 固定 ω 线性最小二乘 + RANSAC 搜 ω |
| 抗噪 | 依赖 Q/R 调参 | RANSAC 对离群点鲁棒 |
| 预测 | 角度积分闭式解 | 用拟合曲线值 fit_spd_ 替代 |

实现上两者串联：EKF 提供平滑的瞬时 spd，RANSAC 从 spd 历史反解出稳定的正弦参数，`fit_spd_` 再回灌给预测（`buff_target.cpp:406`）。

> **一句话速记**：大符转速 spd=a·sin(ωt+φ)+(2.09−a)，两套并存——EKF 把 a/w/φ 当状态用角度积分闭式解预测；RANSAC 让 EKF 只估瞬时 spd，再"随机采 ω + 固定 ω 线性最小二乘 + 内点投票"反解出稳定 a/w/φ。

---

## 6. 小符与开火决策

### 6.1 小符：匀速旋转

小符转速恒定，模型退化（`SmallTarget`，7 维状态，`buff_target.cpp:134-152`）：

```
x = [R_yaw, v_R_yaw, R_pitch, R_dis, yaw, roll, spd]   // 7 维，无 a/w/φ
const double SMALL_W = CV_PI / 3;                       // buff_target.hpp:88 固定角速度 π/3
```

roll 匀速传播 `roll += spd·dt`（A_ 的 roll 行 `..., 1.0, dt`，`:96-102`），spd 初值 `SMALL_W · clockwise`（`:150-152`）。发散判据 `|x[6]| > SMALL_W + π/18 或 < SMALL_W − π/18`（`:84`），即转速偏离 π/3 超过 10° 判发散。

### 6.2 开火决策与预测（buff_aimer）

参数从 YAML 读入（`buff_aimer.cpp:12-15`）：`yaw_offset_`/`pitch_offset_`（度→rad）、`fire_gap_time_`、`predict_time_`。

**双次弹道迭代**（`get_send_angle`，`buff_aimer.cpp:150-199`）——因为提前量本身依赖弹丸飞行时间，需迭代收敛：

```cpp
predict(predict_time);              // ① 先按预测提前量算瞄准点
trajectory0 = 弹道解算;             // ② 得飞行时间 fly_time0
predict(trajectory0.fly_time);      // ③ 再按飞行时间重算瞄准点
trajectory1 = 弹道解算;             // ④ 得更准的角度
if (时间误差 > 0.01) 判为未命中;    // :189-193
yaw = trajectory1.yaw + yaw_offset;  pitch = ... + pitch_offset;  // :196-197
```

**开火间隔控制**（`buff_aimer.cpp:61-64`）：`delta_time(now, last_fire_t_) > fire_gap_time_` 才允许 shoot=true，避免连发打到同一片。切换扇叶瞬间不开火（`:58-60`）。切换扇叶触发条件：`mistake_count_ > 3` 或角度跳变 `> 5°`（`:41-53`）。

弹速兜底：`bullet_speed < 10` 设为 24（`buff_aimer.cpp:28,78-81`）。

**实测配置**（各机型 `configs/`）：

| 机型 | fire_gap_time | predict_time |
|:----|:----|:----|
| standard4 / ascento | 0.520 s | 0.100 s |
| standard3 | 0.700 s | 0.120 s |
| uav | 0.600 s | 0.090 s |

> **一句话速记**：小符固定 π/3 匀速；开火用"双次弹道迭代"求提前量（提前量依赖飞行时间需迭代）、按 fire_gap_time 控制间隔、切叶瞬间停火。

---

## 7. 全向感知：多相机并行架构

全向感知（`tasks/omniperception/`）解决"主相机视野窄、跟丢后找不到敌人"的问题：用多路广角相机 360° 搜敌，给云台转向指引。

### 7.1 四路 USB 相机并行推理

架构（`perceptron.hpp` / `perceptron.cpp`）：

```
usbcam1 ──┐  各自独立 YOLO ──┐
usbcam2 ──┤  各自独立 thread ├──▶ ThreadSafeQueue<DetectionResult> ──▶ Decider
usbcam3 ──┤  parallel_infer  │       (容量 10)                          排序/过滤
usbcam4 ──┘                  ┘
```

- 每路相机建独立 `auto_aim::YOLO` 实例（`yolo_parallel1_..4_`，`perceptron.cpp:19-22`）
- 每路起一个 `std::thread` 跑 `parallel_infer`（`perceptron.cpp:26-29`）
- 每路循环：`cam->read` → `yolo->detect` → 非空则 `decider_.delta_angle(armors, device_name)` 换算角度 → 封装 `DetectionResult{armors, ts, delta_yaw, delta_pitch}` push 入队（`perceptron.cpp:66-104`）
- 线程安全队列 `tools::ThreadSafeQueue<DetectionResult>`（容量 10，`perceptron.cpp:16`），`stop_flag_ + condition_ + mutex` 控制停止

`DetectionResult` 结构（`detection.hpp:12-30`）：armors 列表 + timestamp + delta_yaw/delta_pitch（rad）。取用 `get_detection_queue()` 非阻塞把队列全部弹出成 vector（`perceptron.cpp:51-63`）。

### 7.2 相机方位命名

USB 相机的方位由 sharpness 值决定 device_name（`io/usbcamera/usbcamera.cpp`）：`sharpness_==2 → "left"`（`:110`）、`sharpness_==3 → "right"`（`:115`）。哨兵多线程用 `video0..6` 四路（`src/sentry_multithread.cpp:51-54`），后置相机是独立的 `io::Camera`（`src/sentry.cpp:48`，用 `configs/camera.yaml`）。

> **一句话速记**：全向感知 = "4 路 USB 相机各跑独立 YOLO 线程 → 结果进容量 10 的线程安全队列 → Decider 消费"，用并行推理换取 360° 视野。

---

## 8. Decider：方位换算、过滤与优先级

Decider（`tasks/omniperception/decider.cpp`）是全向感知的大脑，把"某路相机第几号像素框"翻译成"云台该转多少度"。

### 8.1 像素中心 → 相对云台角度

核心是 `delta_angle()`（`decider.cpp:109-130`）。归一化中心 `center_norm ∈ [0,1]`，按相机安装方位加偏置：

```cpp
// left 相机（装在 +62° 方位）：
delta_yaw   = 62 + new_fov_h/2 - center_norm.x * new_fov_h;   // :114
delta_pitch = center_norm.y * new_fov_v - new_fov_v/2;        // :115
// right 相机（−62°）：
delta_yaw   = -62 + new_fov_h/2 - center_norm.x * new_fov_h;  // :120
// back 相机（+170°，FOV 硬编码 54.2°/44.5°）：
delta_yaw   = 170 + 54.2/2 - center_norm.x * 54.2;            // :126
delta_pitch = center_norm.y * 44.5 - 44.5/2;                  // :127
```

原理：`安装方位角 + 该像素在相机 FOV 内的偏移`。`new_fov_h/new_fov_v` 来自 YAML（`decider.cpp:20-21`，如 sentry.yaml 为 27°/40.9°）。目标在图像中心（center_norm.x=0.5）时偏移为 0，只剩安装方位角。

### 8.2 过滤与优先级

**过滤**（`armor_filter`，`decider.cpp:132-153`）：去掉非敌方色（`:136`）、5 号（25 赛季无 5 号，`:139`）、前哨站（`:143-144`）、无敌列表内的（`:147-150`，无敌名单由 ROS2 `subscribe_enemy_status` 传入）。

**优先级排序**（`set_priority` + `sort`，`decider.cpp:155-188`）：按 YAML 的 `mode_` 选优先级表（`decider.hpp:65-85`，如 mode1 中 3/4 号=first、1 号=second）。先对每个 DetectionResult 内的 armors 按 priority 排序，再对整个队列按队首 armor 优先级排序。

### 8.3 用途：只指引不开火

Decider **不做精确弹道**，输出的 `io::Command` 里 `shoot` 恒为 false（`decider.cpp:27-107`）。两个用途：

**① 跟丢时转向指引**（哨兵单线程 `src/sentry.cpp:90-91`）：

```cpp
if (tracker.state() == "lost")
  command = decider.decide(yolo, gimbal_pos, usbcam1, usbcam2, back_camera);  // 全向搜敌
else
  command = aimer.aim(...);   // 自瞄接管
```

`decide` 内把 `delta_angle[0]/57.3 + gimbal_pos[0]` 作为绝对 yaw 目标（`decider.cpp:61-63`），让云台转向敌人所在方位。多线程版同理（`sentry_multithread.cpp:87-105`）。

**② 给导航提供目标坐标**（`get_target_info`，`decider.cpp:190-206`）：在 armors 中找与当前 target 同名者，返回 `{xyz_in_gimbal[0], xyz_in_gimbal[1], 1, name+1}`（`name+1` 避免枚举 0 值歧义），经 `ros2.publish` 发给导航（`sentry.cpp:101-103`）。

> **一句话速记**：Decider = "像素中心 + 相机安装方位角(left+62°/right−62°/back+170°) → 相对云台偏航俯仰 → 过滤非敌/无敌/前哨站 → 按 mode 优先级排序"，只给云台转向指引和导航目标坐标，自己不开火。

---

## 9. 概念关系图谱

### 9.1 打符数据流总图

```
                              电控 IMU 四元数 q
                                    │ set_R_gimbal2world
                                    ▼
相机 ──img──▶ YOLO11_BUFF ──▶ get_r_center ──▶ PowerRune ──▶ Solver(5点IPPE)
              buff/r 两类     几何外推+二值化   定target        buff→world
              每叶6关键点      精定位R标        5等分排序           │
                                                                  ▼
                                            ┌──── SmallTarget(7维,匀速π/3) ────┐
                                            │                                  ▼
                                            └──── BigTarget(10维) ──▶ 正弦转速 ──▶ buff_aimer
                                                  EKF内嵌 / RANSAC拟合            双次弹道迭代
                                                  spd=a·sin(ωt+φ)+(2.09−a)       fire_gap控制
```

### 9.2 全向感知数据流

```
usbcam1~4 ─▶ 4×独立YOLO线程 ─▶ 队列 ─▶ Decider ─┬─ tracker.lost时 ─▶ 云台转向指引(不开火)
back_camera┘  parallel_infer   (10)   方位换算    └─ ros2.publish ──▶ 导航目标坐标
                                              过滤+优先级
```

### 9.3 核心概念对照

| 易混概念 | 区别 |
|:----|:----|
| **buff Target vs 自瞄 Target** | buff=定轴旋转(R标球坐标定圆心+单roll角+半径固定0.7m)；自瞄=整车平移+多板绕轴(笛卡尔圆心+相位+估计半径)。共用同一 EKF 类 |
| **大符 vs 小符** | 大符 spd=a·sin(ωt+φ)+(2.09−a) 变速(10维含a/w/φ)；小符固定 SMALL_W=π/3 匀速(7维) |
| **EKF内嵌正弦 vs RANSAC拟合** | 内嵌:a/w/φ 进状态用角度积分闭式解；RANSAC:EKF只估瞬时spd,固定ω线性最小二乘+内点投票反解 |
| **固定ω 前/后** | 固定前:a·sin(ωt+φ)对a/φ非线性；固定后:和差化积成 A₁sin+A₂cos+C 线性,SVD可解 |
| **R标模型输出 vs 精定位** | 模型直接回归的 r 框有抖动；用扇叶延长线外推初值+圆环mask内找正方形轮廓精定位 |
| **buff_predict.hpp vs buff_target.cpp** | 前者旧版预测器(1/5维)；后者当前用(7/10维含球坐标+RANSAC)。主程序用后者 |
| **Decider vs 自瞄** | Decider只指引转向(shoot恒false)、多相机广域;自瞄精确弹道+开火、主相机 |
| **delta_angle 方位偏置** | left+62°/right−62°/back+170°,叠加像素在FOV内偏移 |

### 9.4 全局一句话速记

> **打符** = "YOLO 出扇叶+R标 → 几何+二值化精定位R标 → 追新亮扇叶定target、五等分排序 → 5点IPPE解出定轴旋转模型 → 大符用 spd=a·sin(ωt+φ)+(2.09−a)（EKF内嵌角度积分 或 RANSAC固定ω线性拟合）、小符匀速π/3 → 双次弹道迭代求提前量开火"。**全向感知** = "多相机并行YOLO → 按安装方位换算相对云台角 → 过滤+优先级 → 跟丢时指引云台转向、给导航送目标"，只指引不开火。

---

## 10. 相关笔记

- [[SP_Vision_学习指南]] — 母文档，全局框架（本篇深化 §6.5）
- [[SP_Vision_整车估计EKF详解]] — 打符与自瞄共用同一个通用 EKF 类，可对照两种目标模型的建模差异
- [[SP_Vision_装甲板识别与解算详解]] — 打符解算同样用 IPPE PnP + camera→gimbal→world 变换链
- [[SP_Vision_轨迹规划器详解]] — 打符 `mpc_aim` 走与自瞄相同的轨迹规划器
- [[SP_Vision_IO与多线程架构详解]] — `set_R_gimbal2world` 用的四元数由 `imu_at` 按时间戳 slerp 插值而来；全向感知的线程安全队列与多线程模型
- [[MAS_Vision_学习指南]] — 本战队自研视觉体系，可对照打符/搜敌的不同实现
- [[RM_C程序员_C++速通指南]] — `std::thread`、`std::function`、`shared_ptr`、SVD 等本篇涉及的 C++/Eigen 技法
