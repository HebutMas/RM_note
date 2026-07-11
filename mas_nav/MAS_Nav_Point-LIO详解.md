# MAS Nav — Point-LIO 点级激光-惯性里程计详解

> 📘 **所属**：[[MAS_Nav_学习指南]] §7.2 | **定位**：导航系统核心算法
> 📘 **前置**：建议先阅读 [[MAS_Vision_姿态解算教程]] 了解 EKF 基础

## 概述

Point-LIO 是 MAS Nav 系统最核心的算法模块，实现**逐点处理**的激光-惯性里程计。与传统的"先积累一帧再处理"（如 FAST-LIO）不同，Point-LIO 每收到一个 LiDAR 点就立即用 IEKF 更新状态，将延迟从 ~50ms 降至 <5ms。

### 与 FAST-LIO 的核心差异

| 维度 | Point-LIO | FAST-LIO |
|------|-----------|----------|
| 处理粒度 | **逐点**（每收到一个点立即处理） | **逐帧**（累积一帧后批量处理） |
| 数据结构 | iVox（增量体素） | ikd-Tree（增量 kd 树） |
| 状态估计 | 双 IEKF（input/output 两种模式） | 单 IEKF |
| IMU 处理 | 预积分预测 + IEKF 更新 | 预积分预测 |
| 延迟 | **<5ms** | ~50ms |
| 地图管理 | 增量添加（MapIncremental） | 全量重建 |

---

## 7.2.1 数学模型总览

Point-LIO 使用 **误差状态迭代卡尔曼滤波器（Error-State IEKF）**，系统状态为：

```
状态向量 x (24维 - input 模式)：
  [pos(3), rot(3), vel(3), bg(3), ba(3), gravity(3)]
  
状态向量 x (30维 - output 模式)：
  [pos(3), rot(3), vel(3), omg(3), acc(3), bg(3), ba(3), gravity(3)]

其中：
  pos:  IMU 在世界坐标系中的位置
  rot:  IMU 的 SO(3) 旋转矩阵
  vel:  IMU 的速度
  omg:  IMU 的角速度（output 模式）
  acc:  IMU 的加速度（output 模式）
  bg:   陀螺仪零偏 (3维)
  ba:   加速度计零偏 (3维)
  gravity: 重力向量 (3维)
```

## 7.2.2 两种 IEKF 模式

源码定义了两种 IEKF 实例，通过 `use_imu_as_input` 参数切换：

**模式 1：use_imu_as_input = true (kf_input)**
- IMU 测量值直接作为 `input_ikfom` 输入
- 状态包含 24 维：位置、旋转、速度、bg、ba、重力
- **运动模型 `get_f_input`：**
  ```cpp
  res(pos) = vel;                      // 位置导数 = 速度
  res(rot) = gyro - bg;               // 旋转变化率 = 角速度 - 零偏
  res(vel) = rot*(acc-ba) + gravity;   // 加速度 = R*(a_raw - ba) + g
  res(bg) = res(ba) = res(gravity) = 0;
  ```
  雅可比矩阵 `df_dx_input` 描述了状态误差的传播：
  ```cpp
  d(pos)/d(vel) = I₃
  d(vel)/d(rot) = -R·[acc-ba]×    (向量的反对称矩阵)
  d(vel)/d(ba) = -R
  d(vel)/d(gravity) = I₃
  d(rot)/d(bg) = -I₃
  ```

**模式 2：use_imu_as_input = false (kf_output，默认)**
- IMU 不作为输入，而是作为额外的测量更新
- 状态包含 30 维（多了 omg 和 acc）
- IMU 测量通过 `h_model_IMU_output` 作为观测方程

## 7.2.3 IMU 初始化（静态对准）

```cpp
// IMU_Processing.cpp: IMU_init()
// 收集 MAX_INI_COUNT 帧 IMU 数据，计算均值
mean_acc += (cur_acc - mean_acc) / N;  // 滑动平均加速度
mean_gyr += (cur_gyr - mean_gyr) / N;  // 滑动平均角速度
// 达到阈值后：
//   重力方向 = -mean_acc / |mean_acc| * G
//   陀螺零偏 = mean_gyr
```

**Set_init 中的重力对齐：**
```cpp
// 计算当前估计重力与预设重力（如 [0,0,-9.81]）之间的旋转
hat_grav = skew(gravity_ground_truth);
align_angle = hat_grav * tmp_gravity / |...| * acos(align_cos);
rot_init = Exp(align_angle);  // SO(3) 指数映射
```

## 7.2.4 点云预处理

```cpp
// laserMapping.cpp 主循环中：
// 1. IMU 处理：去畸变 + 滤波
p_imu->Process(Measures, feats_undistort);

// 2. 体素降采样
downSizeFilterSurf.setLeafSize(filter_size_surf_min, ...);
downSizeFilterSurf.filter(*feats_down_body);

// 3. 按时间排序（每个点的 curvature 字段储存时间戳）
sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);

// 4. 时间压缩：将相邻时间的点分组
time_seq = time_compressing<int>(feats_down_body);
```

**curvature 字段的妙用：** PCL 的 `PointXYZI` 结构有 `curvature` 字段，Point-LIO 用它储存每个点的实际采集时间戳（毫秒级），避免了额外的数据结构设计。

## 7.2.5 地图初始化

```cpp
// 收集 init_map_size 个点后：
if (init_feats_world->size() >= init_map_size) {
    if (enable_prior_pcd) {
        // 加载预录 PCD 地图
        auto map_cloud = loadPointcloudFromPcd(prior_pcd_map_path);
        ivox_->AddPoints(map_cloud->points);
    } else {
        // 用初始帧的点构建地图
        ivox_->AddPoints(init_feats_world->points);
    }
    init_map = true;
}
```

**enable_prior_pcd 的意义：** 比赛前可以手动驾驶机器人走一圈录制 PCD，比赛时加载作为先验地图，加速收敛。

## 7.2.6 IEKF 逐点更新（核心算法）

```cpp
// 伪代码：对每个时间组 k
for (k = 0; k < time_seq.size(); k++) {
    // 步骤1：时间推进，检查是否有新的 IMU 数据
    while (时间戳_imu最近 < 时间戳_当前点) {
        dt = 时间戳_imu最近 - 时间戳_上一次预测;
        kf_output.predict(dt, Q_output);  // 协方差传播
        从 imu_deque 取出下一个 IMU;
    }
    
    // 步骤2：状态预测
    dt = 时间戳_当前点 - 时间戳_上一次预测;
    kf_output.predict(dt, Q_output);      // 状态前向积分
    
    // 步骤3：观测更新（点到面残差）
    kf_output.update_iterated_dyn_share_modified();
    // 内部调用 h_model_output()：
    //   - 将当前点从 body 帧变换到 world 帧
    //   - 在 iVox 中搜索最近邻的 5 个点
    //   - 拟合平面，计算"点到面"距离作为残差
    //   - 构建观测雅可比 H
    //   - 执行 IEKF 更新
}
```

## 7.2.7 点到面残差观测模型（h_model_output 源码分析）

这是 Point-LIO 精度最关键的保证：

```cpp
void h_model_output(state_output &s, ...) {
    for (每个点 j) {
        // 1. 将点从 body 帧变换到 world 帧
        pointBodyToWorld(&point_body_j, &point_world_j);
        
        // 2. iVox 最近邻搜索（5个最近点）
        ivox_->GetClosestPoint(point_world_j, points_near, NUM_MATCH_POINTS);
        
        // 3. 如果找到了5个最近邻，拟合平面
        if (esti_plane(pabcd, points_near, plane_thr)) {
            // pabcd = 平面方程系数 [a,b,c,d]，其中 ax+by+cz+d=0
            
            // 4. 计算点到平面的距离
            pd2 = |a*p_w.x + b*p_w.y + c*p_w.z + d|;
            
            // 5. 质量检查：距离需要在合理范围内
            // (p_norm > match_s * pd2 * pd2) 确保点不能太远
            if (p_norm > match_s * pd2 * pd2) {
                point_selected_surf[idx + j + 1] = true;  // 标记为有效
            }
        }
    }
    
    // 6. 对每个有效点，构建观测雅可比
    // 当外参估计开启时 (extrinsic_est_en=true):
    ekfom_data.h_x(m) = [norm_vec, A, B, C];
    // norm_vec = 平面法向量 (观测的平移部分)
    // A = p_imu_cross * C  (观测的旋转部分)
    // B = p_cross * R_LI^T * C  (观测的外参部分)
    // C = R^T * norm_vec  (法向量在 body 帧的表达)
    
    // 残差 = -法向量·点坐标 - d  (= 负的点到面距离)
    ekfom_data.z(m) = -norm_vec·p_world - d;
}
```

**为什么使用"点到面"而非"点到点"残差？**
- 点到面残差在切平面方向有约束，但法向量方向不做约束
- 这自然处理了 LiDAR 点在同一平面上的情况（如墙壁）
- 比 ICP 的点到点距离收敛更快、更稳定

## 7.2.8 IMU 饱和检测（h_model_IMU_output）

```cpp
// 如果 IMU 读数达到量程的 99%，认为饱和
if (fabs(angvel_avr(0)) >= 0.99 * satu_gyro) {
    ekfom_data.satu_check[0] = true;   // 标记 X 轴角速度饱和
    ekfom_data.z_IMU(0) = 0.0;         // 该轴残差置零（不参与更新）
}
```

**为什么需要饱和检测？** 机器人快速旋转时（如碰撞反弹），陀螺仪可能超出量程（通常 ±2000°/s）。如果强行用饱和数据做更新，会导致状态估计发散。

## 7.2.9 增量地图更新（MapIncremental）

```cpp
void MapIncremental() {
    for (每个配准后的点) {
        // 计算该点所在地图体素的中心
        center = (point_world / filter_size_map_min).floor() * filter_size_map_min;
        
        // 检查是否已有近邻点在该体素内
        bool need_add = true;
        for (每个最近邻点) {
            if (|近邻点 - 中心| < 0.5 * filter_size_map_min) {
                need_add = false;  // 体素内已有点，不重复添加
                break;
            }
        }
        if (need_add) points_to_add.push_back(point_world);
    }
    ivox_->AddPoints(points_to_add);  // 批量添加到 iVox
}
```

**体素去重的意义：** 控制地图密度，防止反复扫描同一区域时地图无限增长。

## 7.2.10 关键调参指南

| 参数                     | 默认值  | 增大效果    | 减小效果      |
| ---------------------- | ---- | ------- | --------- |
| `point_filter_num`     | 4    | 更快、更稀疏  | 更慢、更密     |
| `ivox_grid_resolution` | 0.2m | 更快、更粗糙  | 更慢、更精细    |
| `filter_size_surf`     | 0.2m | 特征更稀疏   | 特征更密集     |
| `gyr_cov`              | 0.01 | 更信任陀螺   | 更信任 LiDAR |
| `acc_cov`              | 0.1  | 更信任加速度计 | 更信任 LiDAR |
| `plane_thr`            | —    | 更多平面通过  | 更严格的平面筛选  |

---

## 深度拓展 1：IEKF 在 SO(3) 上的李代数切空间

### 为什么不能直接在 SO(3) 上做线性卡尔曼滤波

标准卡尔曼滤波要求状态空间是**向量空间**（可加性）。但旋转矩阵属于 SO(3) 群——**流形而非向量空间**。两个旋转矩阵相加没有几何意义。

**误差状态 IEKF（Error-State IEKF）** 的解决方案：

```
名义状态 x̂（在 SO(3) 上旋转矩阵 R̂）：      只在预测阶段用非线性运动模型更新
误差状态 δx（在 so(3) 李代数上）：           在 EKF 框架中做线性高斯更新
总状态 x = x̂ ⊕ δx（通过指数映射"拉回"流形）：  更新后重新投影到 SO(3)
```

### 李代数 so(3) 的切空间结构

SO(3) 在单位矩阵 I 处的切空间是 so(3)（3×3 反对称矩阵空间）：

```
so(3) = { [φ]× ∈ R³ˣ³ | [φ]×ᵀ = -[φ]× }

其中 [φ]× = [  0   -φz   φy ]
            [ φz    0   -φx ]
            [-φy   φx    0  ]
```

**指数映射 Exp: so(3) → SO(3)**：

```
Exp([φ]×) = I + sin(||φ||)/||φ|| · [φ]× + (1-cos(||φ||))/(||φ||²) · [φ]×²
         = Rodrigues(φ)
```

**对数映射 Log: SO(3) → so(3)**（逆映射）：

```
Log(R) = θ/(2·sin(θ)) · (R - Rᵀ)，其中 θ = acos((trace(R)-1)/2)
```

### IEKF 中的误差状态传播

定义误差旋转：`R = R̂ · Exp([δθ]×)`，其中 δθ ∈ R³ 是角度误差向量。

误差状态的运动学（线性化后）：

```
δθ̇ = -[ω̂-bg]× · δθ - δbg - ng     ← 角速度误差传播
δv̇ = -R̂·[acc-ba]× · δθ - R̂·δba - na + δg  ← 速度误差传播
δṗ = δv                            ← 位置误差传播
δḃg = n_bg, δḃa = n_ba             ← 零偏随机游走
δġ = 0                              ← 重力不变
```

这给出了误差状态的**线性**动力学——从而可以用标准 EKF 方程。

### 为什么迭代（Iterated）？

标准 EKF 在预测点 x̂⁻ 处做一次线性化。如果预测不准，线性化点偏差大会导致 EKF 发散。**IEKF** 在每次观测更新时**重新线性化**：

```
for iter = 1 to MAX_ITER:
    // 1. 在最新估计点 x̂⁺ 处重新计算观测雅可比 H
    H = ∂h/∂x | x=x̂⁺
    
    // 2. 重新计算卡尔曼增益
    K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
    
    // 3. 更新
    x̂⁺ = x̂⁻ ⊕ K·(z - h(x̂⁺))
    
    // 4. 收敛检查
    if ||δx|| < ε: break
```

每次迭代将线性化点向真实状态"拉近"，在大初始误差下也能收敛。

---

## 深度拓展 2：过程噪声 Q 的物理建模

### 连续时间噪声模型

状态驱动的噪声来自两个方面：

**1. IMU 测量噪声**（传感器白噪声）：

```
陀螺仪测量：    ω_meas = ω_true + bg + ng,    ng ~ N(0, σg²·I₃)
加速度计测量：   a_meas = a_true + ba + na,    na ~ N(0, σa²·I₃)
```

其中 `ng` 和 `na` 是**高斯白噪声**。σg² 和 σa² 是陀螺仪和加速度计的噪声密度（来自 BMI088 数据手册）。

**2. 零偏随机游走**（低频漂移）：

```
ḃg = n_bg,    n_bg ~ N(0, σbg²·I₃)
ḃa = n_ba,    n_ba ~ N(0, σba²·I₃)
```

### 离散化 Q 矩阵

从连续时间噪声密度到离散时间协方差的推导：

对于位置-速度通道：

```
P_pos = σa² · [dt³/3   dt²/2]
               [dt²/2      dt]
               
P_vel = P_pos  ← 加速度噪声 → 速度和位置的不确定性
```

对于旋转通道（gyro 噪声 → 姿态角协方差）：

```
P_rot = σg² · dt · I₃  ← 角速度噪声线性累积
P_bg  = σbg² · dt · I₃  ← 零偏随机游走
```

**默认参数选择依据：**

| 参数 | 默认值 | 物理依据 |
|------|--------|----------|
| `gyr_cov = 0.01` | σg²·dt ≈ (0.1 rad/s/√Hz)² × 0.001s | BMI088 ARW (Angle Random Walk) ≈ 0.15°/√h ≈ 0.014 rad/s/√Hz |
| `acc_cov = 0.1` | σa²·dt ≈ (0.3 m/s²/√Hz)² × 0.001s | BMI088 VRW (Velocity Random Walk) ≈ 0.03 m/s/√h ≈ 0.28 m/s²/√Hz |
| 零偏噪声 | ~10⁻⁴量级 | 零偏不稳定度 × √dt |

**为什么 `gyr_cov` < `acc_cov`？** 陀螺仪的角速度测量比加速度计的线加速度测量更可靠——加速度计包含大量振动噪声和运动加速度，陀螺仪的输出本质上更"干净"。这也解释了为什么 ESKF 对陀螺仪积分更信任，对加速度计修正更谨慎。

---

## 深度拓展 3：iVox 与 ikd-Tree 的算法复杂度对比

### 数据结构本质

**iVox（Incremental Voxel）**：

```
固定分辨率的 3D 栅格哈希表
Key = floor(point / resolution)  → 体素索引
Value = 体素内点列表（容量有限，通常 ≤ 20 个点/体素）

插入复杂度：  O(1) 哈希查找 + O(1) 追加
查询复杂度：  O(1) 哈希查找 + O(k) 体素内搜索（k 为邻近体素数）
删除复杂度：  不支持（不删除，通过 MapIncremental 体素去重）
内存：        O(N_voxels × points_per_voxel)
```

**ikd-Tree（Incremental KD-Tree）**：

```
自适应 kd-tree，带惰性删除和部分重建
插入复杂度：  O(log n) 平均，O(n) 最坏（树退化）
查询复杂度：  O(log n) 平均
删除复杂度：  O(log n) 惰性标记 + 定期批量重建
内存：        O(n × (3D坐标 + 树指针))
```

### 为什么 Point-LIO 选择 iVox

| 场景 | iVox | ikd-Tree | 赢家 |
|------|------|----------|------|
| 逐点插入（200k pts/s） | O(1) | O(log n) × 200k | **iVox** |
| 最近邻查询（5 nearest） | O(1+5×27) | O(5 log n) | 接近 |
| 地图增长后插入退化 | 无退化（哈希表） | 树深度增长 → 插入变慢 | **iVox** |
| 内存效率 | 更低（体素去重） | 更高（每点存树节点） | **iVox** |
| 下采样查询 | 天然 | 需遍历 | **iVox** |
| 精确 k-NN（任意 k） | 受限（体素边界） | 精确 | ikd-Tree |

关键洞察：**Point-LIO 每帧处理 200,000 个点**。ikd-Tree 的 O(log n) 插入在 200k ops/s 下会产生显著的开销，而 iVox 的 O(1) 哈希插入是常数时间的。牺牲的是精确 k-NN（iVox 的体素边界可能漏掉跨体素的近邻），但这个精度损失被逐点更新的高频补偿了。

### iVox 最近邻搜索的边界效应

iVox 在查询点所在体素及相邻 26 个体素（3×3×3 邻域）中搜索最近邻。如果真正的最近邻在 2 个体素之外（由于体素大小与点密度的关系），会**漏检**。这个误差的概率：

```
P(漏检) ≈ exp(-λ·voxel_size³)  ← λ 为单位体积点数

当 λ·voxel_size³ > 5 时（平均每个体素 > 5 个点），P(漏检) < 0.7%
```

Point-LIO 的默认体素大小 0.2m 在室内场景中满足此条件。

---

## 深度拓展 4：逐点 vs 逐帧收敛性分析

### 信息获取率

逐帧 LIO（FAST-LIO）：每 50ms 获取一帧（约 10,000 个点），做一次批量 IEKF 更新。

逐点 LIO（Point-LIO）：每 5μs 获取一个点，立即更新。

**信息矩阵（Fisher Information）的累积**：

逐帧：
```
I_frame = Σⱼ H_jᵀ·R⁻¹·H_j   (j 遍历一帧内所有点)
→ 每 50ms 一次大的协方差缩减
```

逐点：
```
I_point = Σₖ H_kᵀ·R⁻¹·H_k    (k 遍历最近处理的点)
→ 每个点都微调协方差
```

**等效性证明**：在测量噪声独立且高斯的前提下，逐点处理和逐帧处理的 Fisher 信息累积是**等价的**（线性系统的叠加原理）：

```
I_frame = I_point₁ + I_point₂ + ... + I_point₁₀₀₀₀ = Σ I_point
```

**为什么逐点好？** 不是信息量更多，而是**信息使用更及时**——状态估计在每一帧内部的 50ms 中持续改善，而非等到帧末。这对**快速机动**（如 90°/s 旋转）至关重要：在帧末才更新意味着 50ms 前的位置偏差已累积了 ~4.5° 的角度误差，而逐点更新将这个误差分散在时域中。

### 收敛速率与 Cramér-Rao 下界

Cramér-Rao 下界给出了估计误差协方差的下限：

```
P ≥ I⁻¹(θ) = (Fisher_Information)⁻¹

在 LIO 中：P ≥ (Σ H_jᵀ·R⁻¹·H_j)⁻¹
```

Point-LIO 以更快的速率逼近 CRLB：每一点都朝下界逼近一小步，而非等一帧后跳一大步。这在**退化场景**（长廊、均匀墙壁）中体现为更平滑的轨迹估计。

---

> 📘 **相关笔记**：[[MAS_Nav_学习指南]] · [[MAS_Vision_姿态解算教程]] · [[MAS_Vision_多线程架构学习指南]]
