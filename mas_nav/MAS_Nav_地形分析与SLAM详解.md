# MAS Nav — 地形分析与 2D SLAM 详解

> 📘 **所属**：[[MAS_Nav_学习指南]] §7.4-7.5 | **定位**：感知管道（障碍物检测 + 建图）
> 📘 **下游**：[[MAS_Nav_Nav2导航栈详解]] — 地形输出作为 costmap 输入

## 概述

地形分析是导航栈的"眼睛"，负责将 3D 点云转化为 2D 障碍物信息。MAS Nav 采用**两级地形分析**设计：

| 需求 | 近场 (terrain_analysis) | 远场 (terrain_analysis_ext) |
|------|------------------------|---------------------------|
| 用途 | 局部避障 | 全局路径规划 |
| 精度 | 5cm 体素 | 10cm 体素 |
| 更新 | 1s 衰减 | 4s 衰减 |
| 覆盖 | ~1m² (400 cells) | ~82m² (1681 cells) |

**核心创新：分位数地面估计 + IntensityVoxelLayer**

传统方法用固定高度阈值判定障碍物——但比赛场地有坡道、草地、台阶，固定阈值不可行。MAS Nav 使用**分位数地面估计**：对每个栅格取 Z 坐标的某个分位数作为"地面"，点高于地面的高度（intensity）传给代价地图做连续判定。

### 地形 → SLAM 数据流

```
terrain_analysis_ext → /terrain_map_ext → pointcloud_to_laserscan → /obstacle_scan → slam_toolbox → /map
```

---

## 7.4.2 算法详解

```
输入：/sensor_scan (PointCloud2 in sensor frame)

步骤1：体素降采样
  将空间划分为 voxel_size × voxel_size 的平面栅格
  每个栅格收集所有落入的点

步骤2：地面高度估计
  对每个栅格：
    按 Z 坐标排序
    取 quantileZ 分位数作为地面高度
    quantileZ=0.25 表示：取倒数第25%的点高度（假设75%以上是障碍物）
    quantileZ=0.10 表示：取倒数第10%的点高度（更保守，更多点被认为是地面）

步骤3：障碍物判定
  障碍物条件：Z > 地面高度 + minObstacleHeight
  输出：PointCloud2，intensity = (Z - 地面高度) = 障碍物离地高度

步骤4：动态清除
  每个障碍物记录创建时间
  超过 decayTime 秒未更新的障碍物自动清除
  这处理了动态物体（行人、其他机器人）的消失

步骤5：IntensityVoxelLayer 消费
  代价地图插件读取 intensity 值：
    intensity < 阈值 → 可通行（低矮障碍物/草地）
    intensity > 阈值 → 不可通行（高障碍物/墙壁）
```

## 7.4.3 关键参数详解

| 参数 | 近场值 | 远场值 | 含义 |
|------|--------|--------|------|
| `scanVoxelSize` | 0.05m | 0.1m | 体素边长：越小越精细但点数越多 |
| `quantileZ` | 0.25 | 0.1 | 地面分位点：越小越保守（更多标记地面） |
| `decayTime` | 1.0s | 4.0s | 障碍物保留时间 |
| `minObstacleHeight` | 0.05m | 0.05m | 离地超过此高度才算障碍物 |
| `vehicleHeight` | 1.5m | — | 可通行的最大高度 |

**quantileZ 的选择哲学：**
- `0.25`（近场）：假设至少 25% 的点在地面上，适用于平地
- `0.1`（远场）：更保守，假设只有 10% 的点在地面上，适用于未知地形
- 理想情况下，参数应随场地类型动态调整

---


## 7.5.1 数据来源

```
terrain_analysis_ext → /terrain_map_ext (PointCloud2)
    ↓
pointcloud_to_laserscan → /obstacle_scan (LaserScan)
    ↓
slam_toolbox → /map (OccupancyGrid)
```

**为什么需要 pointcloud_to_laserscan？** slam_toolbox 的标准输入是 LaserScan 消息，需要将 3D 点云投影到 2D 平面：

```yaml
pointcloud_to_laserscan:
  min_height: -0.5    # 低于 -0.5m 的点忽略（地面以下）
  max_height: 1.0     # 高于 1.0m 的点忽略（天花板/悬挂物）
  range_min: 0.1      # 0.1m 以内忽略（盲区）
  range_max: 30.0     # 30m 以外忽略（太远不可靠）
  angle_increment: 0.004363  # ~0.25° 角分辨率
  use_inf: true       # 无回波区域标记为无穷远
```

## 7.5.2 工作模式

```yaml
slam_toolbox:
  mode: "mapping"  # 可选: "mapping" | "localization"
```

- **mapping 模式**：同时建图 + 定位（比赛中使用，因为地图未知）
- **localization 模式**：加载已有地图，只定位不更新（如果提供预建地图）

## 7.5.3 回环检测

```
条件：
  机器运动距离 > minimum_travel_distance (0.1m)
  且 heading 变化 > minimum_travel_heading (0.1 rad)
  
检测逻辑：
  在 loop_search_maximum_distance (10m) 内搜索历史扫描
  若匹配度 > loop_match_minimum_chain_size (5)
  则发布 map → odom 的 TF 修正
```

回环检测修正的是 `map → odom` 的 TF，这会跳跃式修正整个轨迹和地图。

---

## 深度拓展 1：分位数地面估计的统计理论

### 为什么中位数（quantile=0.5）是最优的？

对于每个体素栅格，点的 Z 坐标分布 `{z₁, z₂, ..., zₙ}` 由两部分构成：

```
z_i = z_ground + ε_i      如果点 i 在地面上
z_i = z_ground + h_i + ε_i 如果点 i 在障碍物上 (h_i > 0)
```

其中 `ε_i ~ N(0, σ²)` 是传感器噪声。如果障碍物高度 hi > 0，则障碍物点会"推高"Z 分布。

**分位数选择的统计学含义**：

```
Q(p) = 使 P(Z ≤ Q(p)) ≥ p 的最小值

p = 0.25: 25% 分位数 → 假设 ≤25% 的点在地面上
p = 0.50: 中位数 → p=0.5 是位置参数的最稳健估计(L1损失):
          median = argmin_μ Σ|z_i - μ|
          对离群值(障碍物点)不敏感
```

**最优分位数 p\* 的推导**：

设地面点比例 α ∈ [0,1]，障碍物高度分布 h ~ H（h > 0）。分位数估计量 Q(p) 的偏差：

```
Bias(Q(p)) = { 0                              if p ≤ α
             { E[Q(p) - z_ground] > 0         if p > α
```

当 `p ≤ α`（分位数小于地面点比例）：**零偏差**——因为 Q(p) 一定取自地面点。
当 `p > α`（分位数大于地面点比例）：**正偏差**——Q(p) 取到了障碍物点。

**因此 p\* = α（地面点比例）是最优选择。** 但这取决于环境：
- 平地（α ≈ 0.8）：选择 p = 0.25（保守）→ 不会把地面当障碍物
- 斜坡（α ≈ 0.5）：选择 p = 0.25 → 可能把斜坡下部分当障碍物
- 密集障碍物（α ≈ 0.2）：选择 p = 0.1 → 避免把障碍物当"新地面"

### 为什么近场 p=0.25 而远场 p=0.1

**近场**：机器人周边 1m²，大概率是已知平地 → α 大 → 可以设较高的分位数（0.25），减少地面低估。

**远场**：未知区域，可能有斜坡/台阶 → α 未知 → 设较低的分位数（0.1），宁可多标记一些地面（漏检障碍物）也不能把地面当障碍物（否则路径规划会找不到路）。漏检的障碍物会在机器人靠近时被近场高精度栅格发现。

---

## 深度拓展 2：PointCloud → LaserScan 投影的数学

### 3D → 2D 圆柱投影

```
3D 点 (x, y, z) in sensor frame
    ↓
range = √(x² + y²)           ← 水平距离（忽略 Z）
angle = atan2(y, x)           ← 水平方位角
    ↓
if z ∈ [min_height, max_height]:     ← 高度过滤
  LaserScan[bin(angle)] = range
```

**最小-最大角的扇区划分**：

```
bin(angle) = round((angle - angle_min) / angle_increment)
angle_increment = 0.004363 rad ≈ 0.25°
→ 360°/0.25° = 1440 bins
```

**"Infinite" 处理 (`use_inf: true`)**：如果某个角度 bin 内没有点（因为该方向没有障碍物返回激光），该 bin 的值设为 `inf`。slam_toolbox 将 `inf` 理解为"该方向有无限自由空间"——这对 Bayeisan 占据栅格更新至关重要：

```
log_odds_update(cell, inf_beam):
  P(occupied|inf_beam) → 0    ← 该方向是自由空间
  log_odds → -∞ (or minimum)
```

---

## 深度拓展 3：slam_toolbox 的因子图优化与回环检测

### 因子图表示

slam_toolbox 使用**位姿图（Pose Graph）**表示 SLAM 问题：

```
顶点：   机器人位姿 x₁, x₂, ..., xₙ ∈ SE(2)
边：     两类约束
  (a) 里程计边：x_i → x_{i+1}（来自 Point-LIO）
  (b) 回环边：  x_i → x_j（j >> i，来自激光扫描匹配）
```

### 回环检测的几何一致性

当机器人回到之前访问过的位置时：

```
1. 当前扫描 scan_current → 转为 LaserScan
2. 在半径为 loop_search_radius (10m) 内搜索历史扫描
3. 对候选历史扫描，用 ICP/相关性匹配计算变换
4. 若匹配响应 > loop_match_minimum_chain_size (=5)：
    添加回环边：x_current → x_history
```

**为什么需要链式匹配（chain size ≥ 5）而非单次匹配？**

单次扫描匹配可能因为对称环境（如走廊两端外观相同）产生误匹配。需要**连续 5 次扫描都匹配到相应的历史扫描**——这要求机器人在空间和时间上都连续地回到之前的位置。

### 位姿图优化的数学

添加回环边后，求解最大后验（MAP）位姿：

```
x* = argmin_x Σ_{odom edges} e_odomᵀ·Ω_odom·e_odom
              + Σ_{loop edges} e_loopᵀ·Ω_loop·e_loop

e_odom = x_j - (x_i ⊕ odom_ij)      ← 里程计残差
e_loop = x_j - (x_i ⊕ loop_ij)      ← 回环残差
Ω = 信息矩阵（逆协方差）
```

这是一个**非线性最小二乘**问题，用稀疏 Levenberg-Marquardt 或 Gauss-Newton 求解。结果是一个"强制一致"的轨迹——里程计的累积漂移被回环边"拉回"到正确位置。

**为什么 TF 跳跃是正确的**：回环闭合后，`map → odom` 的 TF 发生跳变。这不是 bug——`map` 帧被修正为全局一致的参考系，`odom` 帧仍保持平滑。下游节点通过 tf2 自动获取修正后的位姿。

---

> 📘 **相关笔记**：[[MAS_Nav_学习指南]] · [[MAS_Nav_Nav2导航栈详解]] · [[MAS_Nav_Point-LIO详解]]
