# 位姿解算与目标跟踪教程 —— 知道敌人在哪

> 📘 **运行上下文**：位姿解算与目标跟踪在 Armor 线程中执行，线程间通信依赖 SPSCQueue，详见 [[MAS_Vision_多线程架构学习指南]]

## 目录

1. [PnP 位姿解算](#1-pnp-位姿解算)
2. [完整的坐标变换链路](#2-完整的坐标变换链路)
3. [扩展卡尔曼滤波基础](#3-扩展卡尔曼滤波基础)
4. [11 维旋转中心 EKF 模型](#4-11-维旋转中心-ekf-模型)
5. [自适应测量噪声与卡方门控](#5-自适应测量噪声与卡方门控)
6. [跟踪状态机](#6-跟踪状态机)
7. [发散检测与恢复](#7-发散检测与恢复)
8. [完整跟踪循环](#8-完整跟踪循环)

---

## 1. PnP 位姿解算

### 1.1 PnP 问题

**Perspective-n-Point**：已知 N 个 3D 空间点及其在图像上的 2D 投影，求解相机相对于这组 3D 点的位姿（旋转 + 平移）。

```
输入:
  3D 点: 装甲板的 4 个角点在世界坐标（物体坐标）
  2D 点: 装甲板的 4 个角点在图像中的像素坐标
  相机内参: K (焦距 fx,fy, 光心 cx,cy)
  畸变系数: D (k1,k2,p1,p2,k3)

输出:
  rvec: 旋转向量（Rodrigues 形式）
  tvec: 平移向量（装甲板中心在相机坐标系中的位置）
```

### 1.2 IPPE 方法的优势

本项目使用 `cv::SOLVEPNP_IPPE`（Infinitesimal Plane-based Pose Estimation）：

```cpp
cv::solvePnP(
    ARMOR_3D_POINTS,     // 4 个共面的 3D 点
    armor.points,        // 4 个检测到的 2D 角点
    camera_matrix_,      // 3×3 相机内参矩阵
    distort_coeffs_,     // 1×5 畸变系数
    rvec, tvec,          // 输出：旋转向量、平移向量
    false,               // 不使用外参猜测值
    cv::SOLVEPNP_IPPE    // IPPE 方法
);
```

| 方法 | 适用场景 | 精度 | 本项目选择原因 |
|------|---------|------|-------------|
| ITERATIVE | 通用 | 高（需好初值） | 需要初值猜测 |
| EPNP | 通用 | 中等 | 不利用共面先验 |
| **IPPE** | **平面目标** | **高** | **装甲板是平面，返回两个解，重投影选择** |

**IPPE 原理简述**：
1. 计算单应矩阵 H（平面到平面的映射）
2. 从 H 分解出两个可能的 (R, t) —— 平面的两个面
3. 通过重投影误差选择正确的那个
4. 迭代优化，使用 `SOLVEPNP_IPPE` 标志自动完成

### 1.3 角点顺序的重要性

```cpp
// 3D 模型点（物体坐标系）             // 2D 图像点
BIG_ARMOR_POINTS = {                  armor.points = {
    (0,  W/2,  H/2),  // 0:左上         left.top,     // 0:左上
    (0, -W/2,  H/2),  // 1:右上         right.top,    // 1:右上
    (0, -W/2, -H/2),  // 2:右下         right.bottom, // 2:右下
    (0,  W/2, -H/2)   // 3:左下         left.bottom   // 3:左下
};                                      };
```

**3D 点和 2D 点必须一一对应**，这是 solvePnP 正确工作的前提。顺序错误会导致位姿解算完全错误。

### 1.4 旋转向量 → 旋转矩阵

```cpp
cv::Mat rmat;  // 3×3
cv::Rodrigues(rvec, rmat);
Eigen::Matrix3d R_armor2camera;
cv::cv2eigen(rmat, R_armor2camera);
```

Rodrigues 公式：`R = I + sin(θ)·[r]× + (1-cos(θ))·[r]×²`，其中 `θ = ||rvec||`, `r = rvec/θ`

### 1.5 Yaw 优化

IPPE 返回的 yaw 角可能不是最优的。`optimize_yaw()` 在 140° 范围内搜索，找到使重投影误差最小的 yaw：

```cpp
float optimize_yaw(Armor &armor, cv::Mat &rvec) {
    float best_yaw = extract_yaw(rvec);
    float best_error = reprojection_error(armor, rvec);
    
    // 在 ±70° 范围内搜索
    for (float dyaw = -70; dyaw <= 70; dyaw += step) {
        float test_yaw = best_yaw + dyaw;
        set_yaw(rvec, test_yaw);
        float error = reprojection_error(armor, rvec);
        if (error < best_error) {
            best_error = error;
            best_yaw = test_yaw;
        }
    }
    return best_yaw;
}
```

这是必要的——PnP 解算中 yaw 轴通常比其他轴的约束更弱（因为装甲板是近似对称的），额外的优化可以显著提高 yaw 精度。

---

## 2. 完整的坐标变换链路

详见 `MAS_Vision_视觉系统坐标变换教程.md`，这里仅列出关键公式：

```
Step 1: PnP → xyz_in_camera
Step 2: 手眼标定 → xyz_in_gimbal = R_camera2gimbal * xyz_in_camera + t_camera2gimbal
Step 3: 相似变换 → xyz_in_world = R_gimbal2world * xyz_in_gimbal
Step 4: 球坐标 → ypd_in_world = xyz2ypd(xyz_in_world)
```

**重要**：世界坐标系和云台坐标系原点重合，所以只有旋转没有平移。

---

## 3. 扩展卡尔曼滤波基础

### 3.1 为什么是 EKF

标准 KF 假设系统是**线性的**。但我们的观测函数（从旋转中心状态映射到装甲板 YPD 坐标）是**非线性**的：

```
h(x) = ypd_of_armor_on_rotating_robot(x)  ← 非线性
```

EKF 通过对非线性函数进行**一阶泰勒展开**（雅可比矩阵）来近似：

```
h(x) ≈ h(x̂) + H·(x - x̂)
其中 H = ∂h/∂x | x=x̂   （雅可比矩阵）
```

### 3.2 EKF 五方程

```
预测：
  x̂⁻ = f(x̂)            状态预测（非线性）
  P⁻  = F·P·Fᵀ + Q     协方差预测（线性近似）

更新：
  K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹   卡尔曼增益
  x̂ = x̂⁻ + K·(z - h(x̂⁻))      状态更新
  P  = (I - K·H)·P⁻             协方差更新（Joseph 形式保证正定性）
```

### 3.3 卡方检验原理

**归一化新息平方（NIS）**：

```
nis = (z - h(x̂⁻))ᵀ · (H·P⁻·Hᵀ + R)⁻¹ · (z - h(x̂⁻))
```

理论：如果滤波器工作正常，nis 应服从自由度为 `dim(z)` 的卡方分布。

对于 4 维观测（yaw, pitch, distance, armor_yaw），95% 置信度阈值为 **0.711**（单边检验）。

```
nis < 0.711 → 测量值符合预期，正常更新
nis > 0.711 → 测量值异常（可能是错误检测），拒绝更新
```

---

## 4. 11 维旋转中心 EKF 模型

### 4.1 为什么不直接跟踪装甲板位置

装甲板附着在旋转的机器人上。如果直接跟踪单块装甲板的 YPD 坐标：
- 当装甲板转出视野时，跟踪中断
- 另一块装甲板转入视野时，需要重新初始化
- 同一机器人上不同装甲板的 YPD 差异巨大，无法关联

**解决方案：跟踪机器人的旋转中心**。
- 旋转中心是固定的（不随机器人旋转而移动）
- 所有装甲板都围绕旋转中心运动
- 任何装甲板的观测都可以用来更新旋转中心的状态
- 装甲板编号（id）决定了它在旋转几何中的位置

### 4.2 11 维状态向量

```
x = [center_x, vx, center_y, vy, center_z, vz, yaw, omega, r, l, h]ᵀ
     下标 0,1     2,3     4,5     6,7   8    9   10
```

| 索引 | 符号 | 物理含义 | 运动模型 |
|------|------|---------|---------|
| 0 | `center_x` | 旋转中心 X 坐标（世界系） | 恒速 |
| 1 | `vx` | 旋转中心 X 速度 | — |
| 2 | `center_y` | 旋转中心 Y 坐标 | 恒速 |
| 3 | `vy` | 旋转中心 Y 速度 | — |
| 4 | `center_z` | 旋转中心 Z 坐标 | 恒速 |
| 5 | `vz` | 旋转中心 Z 速度 | — |
| 6 | `yaw` | 机器人自身朝向角 | 恒速 |
| 7 | `omega` | 绕垂直轴的角速度 | — |
| 8 | `r` | 基座半径（中心到小装甲板距离） | 恒等 |
| 9 | `l` | 额外半径（大装甲板的延伸） | 恒等 |
| 10 | `h` | 侧装甲板高度偏移 | 恒等 |

### 4.3 状态转移矩阵 F（11×11）

```
F = 块对角矩阵:

[1 dt]              ← (x, vx)
[0  1]
      [1 dt]        ← (y, vy)
      [0  1]
           [1 dt]   ← (z, vz)
           [0  1]
                [1 dt]  ← (yaw, omega)
                [0  1]
                     [1]  ← r (恒等)
                        [1]  ← l (恒等)
                           [1]  ← h (恒等)
```

这是一个**解耦的恒速模型**：
- 位置和角度以恒定速度变化（一阶模型）
- 几何参数（r, l, h）视为常数

### 4.4 过程噪声 Q

**平移通道**（x, y, z）：
```
Q_xyzv = [a·v1,  b·v1]
         [b·v1,  c·v1]

a = dt⁴/4,  b = dt³/2,  c = dt²
v1 = 100 (普通目标), 10 (前哨站)
```

**旋转通道**（yaw, omega）：
```
Q_yaw_w = [a·v2,  b·v2]
          [b·v2,  c·v2]

v2 = 400 (普通目标), 0.1 (前哨站)
```

**物理含义**：
- `v1 = 100` → 假设目标线加速度不确定性 σ ≈ √100 = 10 m/s²
- `v2 = 400` → 假设目标角加速度不确定性 σ ≈ √400 = 20 rad/s²
- 前哨站的值更小 → 其运动更可预测

**为什么前哨站的 v2 = 0.1 而普通目标 = 400？**
前哨站匀速旋转（固定角速度），转速变化极小。普通目标（步兵/英雄）可以突然加速旋转（"小陀螺"）。

### 4.5 观测函数 h(x, armor_id)

从旋转中心状态计算第 `id` 号装甲板的 YPD：

```cpp
// 4 装甲板目标（最普遍）
angle = x[6] + id * 2π/4;                   // 装甲板当前朝向
use_lh = (id == 1 || id == 3);               // id=1,3 是大装甲板
r_eff = use_lh ? x[8] + x[9] : x[8];        // 大小装甲板半径不同
armor_x = x[0] - r_eff * cos(angle);          // 装甲板的世界 X
armor_y = x[2] - r_eff * sin(angle);          // 装甲板的世界 Y
armor_z = x[4] + (use_lh ? x[10] : 0);       // 侧板有高度偏移

// 转球坐标
h(x, id) = [atan2(armor_y, armor_x),          // yaw (观测)
            atan2(armor_z, √(armor_x²+armor_y²)), // pitch (观测)
            √(armor_x²+armor_y²+armor_z²),    // distance (观测)
            angle]                              // armor_yaw (观测)
```

**装甲板几何布局**：
```
       id=1 (大板, 前面)
           ↑
id=0 ←───[中心]───→ id=2
  (小板,右)    │      (小板,左)
           ↓
       id=3 (大板, 后面)

id=1,3: r_eff = r + l, z = cz + h  (前后大板)
id=0,2: r_eff = r,       z = cz     (左右小板)
```

### 4.6 观测雅可比 H（4×11）

通过**链式法则**计算：`H = H_ypd_xyz(3×3) · H_armor_xyza(4×11)`

**H_armor_xyza（装甲板位置对状态的偏导）**：
```cpp
// 以 4 装甲板目标为例:
dx/d(cx)   = 1        dx/d(yaw)  = r_eff·sin(angle)   dx/d(r) = -cos(angle)
dy/d(cy)   = 1        dy/d(yaw)  = -r_eff·cos(angle)  dy/d(r) = -sin(angle)
dz/d(cz)   = 1        dz/d(yaw)  = 0                   dz/d(h) = (use_lh ? 1 : 0)
d(angle)/d(yaw) = 1
// 其他偏导为 0
```

**H_ypd_xyz（球坐标对笛卡尔坐标的偏导）**：见 `MAS_Vision_视觉系统坐标变换教程.md` 第 7 节。

### 4.7 初始状态设置

```cpp
void Target::set_target(const Armor &armor, int armor_id) {
    // 从第一次观测反推旋转中心
    yaw = armor.ypr_in_world[0];  // 装甲板的朝向 = 机器人朝向 + armor_id 偏移
    // 反推机器人朝向
    robot_yaw = yaw - armor_id * 2π/N;
    
    // 反推旋转中心位置
    center_x = armor_x + r * cos(robot_yaw);
    center_y = armor_y + r * sin(robot_yaw);
    center_z = armor_z;
    
    // 速度全部初始化为 0
    vx = vy = vz = omega = 0;
    
    // P0 对角线
    P0 = diag([1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1]);
    //         ↑ 位置不确定度小  ↑ 速度不确定度大  ↑ 几何参数不确定度小
}
```

---

## 5. 自适应测量噪声与卡方门控

### 5.1 自适应 R 矩阵

观测噪声不是恒定的——它取决于**观测几何**：

```cpp
double center_yaw = atan2(armor_y, armor_x);       // 指向装甲板中心的方向
double delta_angle = |armor_yaw - center_yaw|;       // 装甲板偏离中心的角度

R = diag(
    0.004,                                         // yaw 观测噪声 — 恒定
    0.004,                                         // pitch 观测噪声 — 恒定
    log(|delta_angle| + 1) + 1,                     // ❗距离噪声 — 随视角增大
    log(distance + 1) / 200 + 0.09                  // ❗朝向噪声 — 随距离增大
);
```

**设计原理**：
- **距离噪声随视角增大**：从侧面看装甲板时（delta_angle 大），PnP 测距精度下降
- **朝向噪声随距离增大**：远距离装甲板的角度观测精度下降
- 这是将**物理直觉编码到 R 矩阵中**——EKF 自动降低不可靠观测的权重

### 5.2 NIS 滑动窗口监控

```cpp
// 每次 update 后记录
bool fail = (nis > 0.711);  // 4 自由度, 95% 置信度
recent_nis_failures.push_back(fail ? 1 : 0);
if (recent_nis_failures.size() > 100)
    recent_nis_failures.pop_front();

// 计算失败率
double fail_rate = sum(failures) / failures.size();

// 失败率 > 40% → 目标模型可能发散
if (fail_rate > 0.4)
    diverged = true;
```

### 5.3 装甲板匹配

当一帧检测到多个装甲板时，需要决定哪个装甲板属于当前跟踪的目标：

```cpp
double score = distance_weight + angle_weight * (yaw_error + pos_yaw_error) * 10.0;

// 旋转越快，角度权重越大
if (omega > 2.0)
    angle_weight *= 2.0;  // 高速旋转时优先角度匹配
```

角度权重在高速旋转时加倍——因为此时位置变化快，角度（装甲板编号）是更可靠的匹配依据。

---

## 6. 跟踪状态机

### 6.1 状态定义

```cpp
enum class TrackState {
    LOST,        // 没有目标
    DETECTING,   // 发现候选，确认中
    TRACKING,    // 稳定跟踪
    TEMP_LOST    // 短暂丢失（用预测维持）
};
```

### 6.2 状态转移

```
                首次检测到
    LOST ──────────────────→ DETECTING
     ↑                           │
     │                   连续 N 帧检测到
     │                    (min_detect_count = 5)
     │                           │
     │                           ▼
     │                       TRACKING ←──────────┐
     │                        │    ↑              │
     │               连续 K 帧  │    │ 重新检测到   │
     │               丢失       │    │              │
     │              (max=3)    ▼    │              │
     │                     TEMP_LOST ──────────────┘
     │                           │
     └───────────────────────────┘
          超过 max_temp_lost_count 帧未恢复
```

### 6.3 各状态的策略

| 状态 | 含义 | EKF 行为 | 输出 |
|------|------|---------|------|
| LOST | 无目标 | 不运行 | 无 |
| DETECTING | 确认中 | 预测 + 更新 | 输出，但不发送射击指令 |
| TRACKING | 稳定跟踪 | 预测 + 更新 + NIS 门控 | 全部输出 |
| TEMP_LOST | 短暂丢失 | **仅预测**（无更新） | 输出（用预测维持） |

**为什么需要 TEMP_LOST？**
- 装甲板可能被短暂遮挡（自身结构、其他机器人）
- 检测可能有偶发漏检（光线变化、运动模糊）
- 如果一丢失就回到 LOST，频繁的重新确认会错过射击窗口

### 6.4 关键参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `min_detect_count` | 5 | 进入 TRACKING 所需的连续检测帧数 |
| `max_temp_lost_count` | 3 | TEMP_LOST 的最大帧数 |
| `outpost_max_temp_lost_count` | 5 | 前哨站专用（运动慢，可等更久） |

---

## 7. 发散检测与恢复

### 7.1 几何发散检测

```cpp
bool diverged() {
    double r   = x[8];       // 基座半径
    double r_l = x[8] + x[9]; // 大装甲板半径
    
    // 半径超出物理合理范围
    if (r   < 0.05 || r   > 0.5) return true;
    if (r_l < 0.05 || r_l > 0.5) return true;
    
    return false;
}
```

真实机器人的旋转半径在 0.2–0.35m 之间。半径偏离到 0.05 或 0.5 意味着 EKF 已经收敛到错误的状态。

### 7.2 NIS 发散检测

```
nis_fail_rate > 40% (100 帧滑窗) → 发散
```

### 7.3 发散恢复

```cpp
if (diverged()) {
    state = LOST;         // 回到 LOST 状态
    target.reset();       // 重置 EKF
    // 等待下一次检测重新初始化
}
```

### 7.4 收敛检测

```cpp
bool converged() {
    // 普通目标: update_count > 3 且未发散
    // 前哨站: update_count > 10 且未发散（更保守）
    return update_count > threshold && !diverged();
}
```

收敛后启用前哨站的角速度约束（`|omega|` 限制到 2.51 rad/s）。

---

## 8. 完整跟踪循环

### 8.1 主流程

```cpp
optional<Target> ArmorTrack::track(vector<Armor> &armors, time_point timestamp) {
    // 1. 计算 dt
    double dt = delta_time(last_timestamp_, timestamp);
    if (dt > 0.1) {
        state_ = LOST;  // 间隔 > 100ms → 数据中断，重新开始
        return nullopt;
    }
    
    // 2. 按优先级排序装甲板
    sort(armors, [](auto &a, auto &b) {
        // 优先级: 1>2>3>4>5 > sentry > outpost > base
        // 同优先级: 离图像中心近的优先
        return priority(a) > priority(b) 
            || (priority(a) == priority(b) && center_dist(a) < center_dist(b));
    });
    
    // 3. 状态机处理
    switch (state_) {
    case LOST:
        set_target(first_valid_armor);  // 尝试初始化新目标
        break;
    case DETECTING:
    case TRACKING:
    case TEMP_LOST:
        update_target(armors, timestamp);  // EKF 预测 + 更新
        break;
    }
    
    // 4. 状态转移
    state_machine(armors);
    
    // 5. 发散检查
    if (target_.diverged() || nis_fail_rate > 0.4) {
        state_ = LOST;
        target_.reset();
        return nullopt;
    }
    
    // 6. 仅 TRACKING/TEMP_LOST 时返回结果
    if (state_ == TRACKING || state_ == TEMP_LOST)
        return target_.get_state();
    
    return nullopt;
}
```

### 8.2 update_target 细节

```cpp
void update_target(vector<Armor> &armors, timestamp) {
    // 1. EKF 预测（向前预测到当前时间戳）
    target_.predict(timestamp);
    
    // 2. 收集匹配目标名称和类型的装甲板
    vector<Armor*> matched;
    for (auto &armor : armors) {
        if (armor.number == target_name && armor.type == target_type)
            matched.push_back(&armor);
    }
    
    // 3. 对每个匹配的装甲板
    for (auto *armor : matched) {
        // PnP 位姿解算
        armor_pose_.GetArmorPose(*armor);
        
        // EKF 更新
        target_.update(*armor);
    }
    
    detected_this_frame = !matched.empty();
}
```

### 8.3 性能要求

```
单帧跟踪时间: < 2ms
  predict:  < 0.1ms
  update × N: < 0.3ms/armor
  state_machine: < 0.01ms

总计（检测 + 跟踪 + 射击）: < 10ms
目标帧率: 60 FPS (16.7ms 预算)
```
