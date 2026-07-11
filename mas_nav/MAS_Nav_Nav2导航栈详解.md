# MAS Nav — Nav2 导航栈详解

> 📘 **所属**：[[MAS_Nav_学习指南]] §7.6 | **定位**：路径规划 + 局部控制 + 代价地图
> 📘 **包含**：行为树、MPPI 采样最优控制器、GoalApproachController、IntensityVoxelLayer

## 概述

Nav2 是 ROS 2 的标准导航框架。MAS Nav 在其基础上做了三项关键定制：
1. **MPPI 控制器**（替代默认 DWB）：基于采样的最优控制，1000 条轨迹并行评估
2. **IntensityVoxelLayer**（替代二值 ObstacleLayer）：利用障碍物高度信息区分草地与墙壁
3. **GoalApproachController**（包装器）：在 MPPI 外做终端精度保证

### 整体控制流

```
                   ┌──────────────┐
   目标位姿 ──────>│  BT Navigator │
                   └──────┬───────┘
                          │ action: NavigateToPose
          ┌───────────────┼───────────────┐
          v               v               v
   ┌──────────┐   ┌────────────┐   ┌──────────────┐
   │ Planner  │   │ Controller │   │ Goal Checker │
   │ (Theta*) │   │ (MPPI+GA)  │   │              │
   └────┬─────┘   └─────┬──────┘   └──────────────┘
        │               │
        │ path          │ /cmd_vel
        v               v
   ┌──────────┐   ┌──────────────────┐
   │ Smoother │   │ Velocity Smoother│
   └──────────┘   └────────┬─────────┘
                           │ /cmd_vel_smoothed
                           v
                    ┌─────────────┐
                    │ 电机控制器   │
                    └─────────────┘
```

---

## 7.6.2 行为树详解（源码级）

```xml
<!-- navigate_to_pose_w_replanning_and_recovery.xml -->
<root BTCPP_format="3">
  <BehaviorTree ID="MainTree">
    <!-- 外层：恢复节点，最多重试10次 -->
    <RecoveryNode name="NavigateRecovery" number_of_retries="10">
      
      <!-- 正常导航部分 -->
      <PipelineSequence name="NavigateWithReplanning">
        <!-- 步骤1：以3Hz频率重复计算全局路径 -->
        <RateController hz="3.0">
          <RecoveryNode name="ComputePathToPose" number_of_retries="1">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context"/>
          </RecoveryNode>
        </RateController>
        
        <!-- 步骤2：跟随路径（最多重试10次） -->
        <RecoveryNode name="FollowPath" number_of_retries="10">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context"/>
        </RecoveryNode>
      </PipelineSequence>
      
      <!-- 恢复行为：当正常导航失败时触发 -->
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>          <!-- 目标更新了？重新来 -->
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree"/>
          </Sequence>
          <BackUp backup_dist="1.0" backup_speed="1.0"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

**行为树节点说明：**

| 节点类型 | 作用 |
|----------|------|
| `RecoveryNode` | 先执行子节点1（正常行为），失败后执行子节点2（恢复行为） |
| `PipelineSequence` | 并行序列：子节点可以并行 tick，但必须按顺序完成 |
| `RateController` | 限制子节点的执行频率（3Hz 重规划） |
| `ReactiveFallback` | 每次 tick 都重新选择子节点（而非记忆上次） |
| `RoundRobin` | 轮转执行子节点（每次失败换下一个） |

**恢复策略链：**
1. 清除局部代价地图 → 清除全局代价地图（清掉可能过时的障碍物）
2. 后退 1m（如果用 BackUpFreeSpace 插件，会检测后退方向是否安全）
3. 重新规划 → 重新跟随
4. 重复最多 10 次

## 7.6.3 MPPI 控制器详解

MPPI 是一种基于采样的最优控制器。核心思想：

**不求解复杂的优化问题，而是随机生成大量候选轨迹，选出最好的。**

```
MPPI 算法流程（40Hz 执行）：

步骤1：采样 1000 条轨迹
  每条轨迹 = 40 步控制序列 (v, ω)
  每步 0.05s → 总时域 2s
  从以当前最优控制为均值、temperature 参数控制方差的高斯分布中采样

步骤2：前向仿真每条轨迹
  使用运动模型（差分驱动）模拟轨迹：
    x_{t+1} = x_t + v·cos(θ)·dt
    y_{t+1} = y_t + v·sin(θ)·dt
    θ_{t+1} = θ_t + ω·dt

步骤3：对每条轨迹打分（加权评判函数）
  总代价 = Σ weight_i × critic_i(轨迹)
  
  GoalCritic:      代价 = distance_to_goal^power × weight
                   越接近目标，代价越低
  PathAngleCritic: 代价 = |轨迹朝向 - 路径朝向| × weight
                   越对齐路径方向，代价越低
  ObstaclesCritic: 代价 = cost_in_costmap × critical_weight (如果在膨胀区)
                   越远离障碍物，代价越低
  PathFollowCritic:代价 = distance_to_path × weight
                   越靠近路径，代价越低
  PreferForwardCritic: 代价 = backward_penalty
                   鼓励前进而非后退

步骤4：加权平均
  用 softmax（temperature 参数）对代价做指数加权：
    权重 ∝ exp(-代价 / temperature)
    temperature 高 → 更多探索（平均更均匀）
    temperature 低 → 更多利用（最优点权重极高）
  
  最优控制 = Σ(weight_i × 控制序列_i) / Σ weight_i

步骤5：执行第一步
  只执行最优控制序列的第一个 (v, ω)，下一个周期重新规划
```

**关键参数调优指南：**

| 参数 | 作用 | 增大效果 | 减小效果 |
|------|------|----------|----------|
| `batch_size` (1000) | 采样轨迹数 | 更优解，更慢 | 更快，可能更差 |
| `time_steps` (40) | 时域长度 | 更远见，更慢 | 更快，短视 |
| `temperature` (2.0) | 探索温度 | 更平滑，更多探索 | 更激进，更利用 |
| `model_dt` (0.05) | 仿真步长 | 更粗糙 | 更精细 |
| `GoalCritic.weight` (10.0) | 目标权重 | 更快趋近目标 | 更关注其他因素 |
| `ObstaclesCritic.critical_weight` (50.0) | 碰撞惩罚 | 更谨慎避障 | 更激进 |

## 7.6.4 GoalApproachController 源码分析

```cpp
geometry_msgs::msg::TwistStamped
GoalApproachController::computeVelocityCommands(...) {
    // 1. 先让内部控制器（MPPI）给出候选控制
    auto cmd = inner_controller_->computeVelocityCommands(pose, velocity, goal_checker);
    
    // 2. 计算到目标的距离
    double dx = goal_.pose.position.x - pose.pose.position.x;
    double dy = goal_.pose.position.y - pose.pose.position.y;
    double dist = std::hypot(dx, dy);
    
    // 3. 根据距离选择模式
    if (dist < direct_approach_distance_) {
        // 模式 C：直驱模式 (< 0.5m，默认)
        // 绕过 MPPI，直接朝目标点做比例控制
        double target_speed = min(approach_velocity_, dist * direct_approach_kp_);
        cmd.twist.linear.x = target_speed * (dx / dist);
        cmd.twist.linear.y = target_speed * (dy / dist);
        cmd.twist.angular.z = 0.0;
        
    } else if (dist < approach_distance_) {
        // 模式 B：逼近模式 (< 1.5m，默认)
        // 限制线速度不超过 approach_velocity_
        double speed = hypot(cmd.twist.linear.x, cmd.twist.linear.y);
        if (speed > approach_velocity_) {
            double scale = approach_velocity_ / speed;
            cmd.twist.linear.x *= scale;
            cmd.twist.linear.y *= scale;
            cmd.twist.angular.z *= scale;  // 角速度同步缩放
        }
    }
    // 模式 A：距离 > approach_distance_，透传 MPPI 输出
    
    return cmd;
}
```

**设计精妙之处：**
- 远处的全局优化交给 MPPI（采样+评判）
- 逼近时减速防止冲过目标
- 极近距离直接做 P 控制（MPPI 的采样在近距离意义不大）

## 7.6.5 IntensityVoxelLayer（自定义代价地图插件）

标准 Nav2 的 ObstacleLayer 做二值判定：有点=障碍物，没点=自由空间。但 robomaster 场地上有很多低矮物体（草地、小坡），二值判定会导致大量误报。

**IntensityVoxelLayer 的创新：**

```
标准 ObstacleLayer：
  if (point_count > threshold) → 标记为障碍物

IntensityVoxelLayer：
  if (max_intensity > obstacle_threshold) → 标记为障碍物
  // intensity 来自 terrain_analysis 的输出
  // intensity = 障碍物离地高度
  // 草地 = 低 intensity → 可通行
  // 墙壁 = 高 intensity → 不可通行
```

**订阅的话题：**
- `local_costmap`：订阅 `/terrain_map`（近场 5cm 体素）
- `global_costmap`：订阅 `/terrain_map_ext`（远场 10cm 体素）

---

## 深度拓展 1：MPPI 路径积分控制理论

### 从最优控制到路径积分

MPPI（Model Predictive Path Integral Control）的理论基础是**随机最优控制的路径积分形式**。

考虑受控随机微分方程：

```
dx = (f(x) + G(x)·u)·dt + σ·B·dw
```

其中 `w` 是维纳过程，`σ·B·dw` 是控制通道的噪声。控制目标是最小化轨迹代价：

```
J = E[ φ(x_T) + ∫₀ᵀ L(x_t, u_t) dt ]
```

**关键数学事实**：当控制噪声与动力学耦合时（`G(x) = σ·B`），最优控制可以通过**路径积分**求解，无需迭代优化。最优控制是采样轨迹代价的指数加权平均：

```
u* = ∫ u·p(u)·exp(-S(u)/λ) du / ∫ p(u)·exp(-S(u)/λ) du
```

其中 `S(u)` 是控制序列的累积代价（总代价），`λ` = temperature。这是 MPPI 不需要反向传播/梯度下降的数学根源——**最优控制是采样的期望值**。

### 为什么 temperature 控制探索-利用权衡

设 `λ` 为 temperature 参数。采样轨迹 `{τ_i}`，每条轨迹代价 `S(τ_i)`。

```
权重 w_i = exp(-S(τ_i) / λ) / Σ exp(-S(τ_j) / λ)    ← Softmax!
最优控制 = Σ w_i · u_i
```

**temperature 的极限行为：**

| λ → 0 | λ → ∞ |
|-------|-------|
| `exp(-S/ε) ≈ 0 for all but min(S)` | `exp(-S/∞) ≈ 1 for all` |
| 只有最优轨迹有非零权重 | 所有轨迹等权重 |
| 贪婪利用 — 可能陷入局部最优 | 纯探索 — 输出是平均 |
| 运动抖、不光滑 | 运动光滑但保守 |
| 推荐 λ = 2.0（平衡点） | |

**λ 与代价尺度的关系**：如果目标在 10m 外（GoalCritic 贡献 > 100），λ = 2.0 会"淹没"其他代价项（PathFollow, Obstacles）。此时可以：

```
方案 A：减小 λ（更贪婪）→ 但增加振荡风险
方案 B：对 GoalCritic 做归一化（除以 max_distance）
方案 C：增加 ObstaclesCritic.critical_weight → 强制考虑碰撞
```

### 轨迹采样数的统计分析

batch_size = 1000 的意义：

```
估计误差 ∝ 1/√(batch_size)
→ 1000 条轨迹：相对误差 ~1/√1000 ≈ 3.2%
→ 100 条轨迹： 相对误差 ~1/√100 ≈ 10%
→ 10000 条轨迹：相对误差 ~1/√10000 ≈ 1%（收益递减）

1000 是工程上的 sweet spot：40Hz × 1000 trajectories × 40 steps = 1.6M 仿真步/秒
```

---

## 深度拓展 2：Theta* vs A* 启发式对比

### A* 的基础

```
f(n) = g(n) + h(n)
g(n) = 从起点到 n 的实际代价
h(n) = 从 n 到目标的启发式估计代价

要求 h(n) ≤ h*(n)（可接受启发式）→ 保证最优解
```

**2D 栅格上的标准 A* 只能生成 8 邻域路径**——路径是锯齿状的，真实代价高于直线路径。

### Theta* 的改进：LOS（Line of Sight）检查

```
A* 对每个邻居 n' 更新：
  g(n') = min( g(n') , g(n) + cost(n → n') )

Theta* 额外检查：
  if LOS(parent(n), n'):              ← 可以从祖父直接跳到当前节点？
    g(n') = min( g(n') , g(parent(n)) + cost(parent(n) → n') )
    parent(n') = parent(n)            ← 跳过中间节点
```

**LOS 的几何含义**：如果从祖父节点到当前节点是直线且无障碍物，路径可以直接"拉直"——消除了 A* 的锯齿。

**复杂度影响**：

| | A* (8-连通) | Theta* |
|---|---|---|
| 展开的节点数 | O(N) | O(N) + O(N·log N) LOS 检查 |
| LOS 检查 | 0 | 每步 O(N_grid) 光栅化 |
| 路径长度 | ~1.08× 最优（锯齿化） | ~1.01× 最优（拉直后） |
| 转弯数 | 多（每格一个转弯） | 少（只在障碍物附近转弯） |

**在比赛场地中**：场地主要是矩形区域 + 少量障碍物。Theta* 的 LOS 检查大多数时候成功，路径基本是直线，大幅减少了转弯次数和跟随误差。

---

## 深度拓展 3：行为树执行语义

### Tick 机制

行为树不是事件驱动的，而是**轮询驱动**（ticked）。每 10ms（100Hz），从根节点开始递归 tick：

```
tick(node):
  status = node.execute()
  return {SUCCESS, FAILURE, RUNNING}
```

**三种返回值的语义**：

| 返回值 | 含义 | 父节点行为 |
|--------|------|-----------|
| SUCCESS | 节点完成目标 | Sequence → 下一个兄弟；Fallback → 停止 |
| FAILURE | 节点放弃 | Sequence → 停止返回 FAILURE；Fallback → 下一个兄弟 |
| RUNNING | 节点仍在执行 | 所有父节点向上传递 RUNNING，下次 tick 继续 |

### RecoveryNode 的执行语义

```
RecoveryNode(children=[Navigate, Recover], retries=10):
  failure_count = 0
  while failure_count < retries:
    status = tick(Navigate)
    if status == SUCCESS: return SUCCESS
    if status == FAILURE:
      tick(Recover)           ← 执行恢复行为
      failure_count += 1
  return FAILURE
```

**关键**：恢复节点记忆失败计数。每次恢复（清除代价地图、后退）后，导航重用更新后的地图重新规划。

### PipelineSequence vs ReactiveSequence

Nav2 行为树中关键的设计选择：

```
ReactiveSequence：每次 tick 都从头检查所有子节点
  → 适合需要快速响应变化的场景（如发现障碍物立刻重规划）

PipelineSequence：子节点可并行 tick 但必须按序完成
  → 适合有严格执行顺序的场景（先规划，再跟随）
```

**为什么使用 RateController(3Hz) 包裹重规划？**

每 333ms 重新计算全局路径——太频繁浪费 CPU（MPPI 局部控制会处理细微偏差），太稀疏无法适应动态变化。3Hz 是 40Hz MPPI 控制的 ~1/13，平衡了响应性（>3Hz 足够跟踪变化）和效率（不每帧重算）。

---

## 深度拓展 4：代价地图的数学解释

### 膨胀层的概率含义

Nav2 的 InflationLayer 在障碍物周围创建"膨胀区"。这不是简单地在障碍物外加缓冲区——膨胀代价是**距离的衰减函数**：

```
cost(d) = lethal_cost · exp(-α·d)   (exponential decay)
        或 = lethal_cost · (1 - d/inflation_radius)   (linear decay)
```

**概率解释**：膨胀区编码了"机器人可能在这个区域"的不确定性。距离障碍物 d 处的代价 ∝ P(碰撞 | 距离 d)。

### IntensityVoxelLayer 的信息论优势

标准 ObstacleLayer 做硬二值判定，等价于假设：
```
P(障碍物 | 点云) = { 1 if count > threshold, 0 otherwise }
```
这是**最大化后验（MAP）的硬估计**，丢弃了不确定性信息。

IntensityVoxelLayer 保持连续值：
```
cost(intensity) = f(intensity / max_obstacle_height)
```
保留了**全概率分布**的信息——intensity = 0.1（低草地）给出低代价（可通过），intensity = 0.9（墙壁）给出高代价（不可通过）。这在代价地图层实现了**软判定**而非硬判定，减少了误报对路径规划的影响。

---

> 📘 **相关笔记**：[[MAS_Nav_学习指南]] · [[MAS_Nav_地形分析与SLAM详解]] · [[MAS_Nav_决策与UDP通信]]
