# MAS Nav 学习指南（详注版）

> RoboMaster 哨兵机器人 ROS 2 Humble 全栈自主导航系统 — 逐行源码级深度解析

---

## 目录

1. [项目概述](#1-项目概述)
2. [快速上手](#2-快速上手)
3. [系统架构](#3-系统架构)
4. [分阶段启动流程](#4-分阶段启动流程)
5. [包地图](#5-包地图)
6. [坐标系树](#6-坐标系树)
7. [核心子系统详解](#7-核心子系统详解)
   - 7.1 [Livox MID-360 驱动](#71-livox-mid-360-激光雷达驱动)
   - 7.2 [Point-LIO](#72-point-lio点级激光-惯性里程计) → [[MAS_Nav_Point-LIO详解|专题]]
   - 7.3 [帧适配器](#73-帧适配器)
   - 7.4 [地形分析与 2D SLAM](#74-地形分析与-2d-slam) → [[MAS_Nav_地形分析与SLAM详解|专题]]
   - 7.5 [Nav2 导航栈](#75-nav2-导航栈) → [[MAS_Nav_Nav2导航栈详解|专题]]
   - 7.6 [帧适配器](#73-帧适配器)
   - 7.7 [决策与 UDP 通信](#76-决策与-udp-通信) → [[MAS_Nav_决策与UDP通信|专题]]
8. [配置系统](#8-配置系统)
9. [关键设计模式](#9-关键设计模式)
10. [常见操作](#10-常见操作)
11. [仓库结构](#11-仓库结构)
12. [学习路线建议](#12-学习路线建议)

---

## 1. 项目概述

### 1.1 什么是 MAS Nav

MAS Nav 是一套面向 **RoboMaster 机甲大师赛** 哨兵机器人的 **ROS 2 Humble** 全栈自主导航系统。它的核心任务是在比赛场地内实现完全自主的：

- **定位**：知道自己在哪里（厘米级精度）
- **建图**：知道周围环境长什么样
- **规划**：知道怎么走到目标点
- **控制**：精确执行规划的速度指令
- **决策**：根据比赛状态切换策略

### 1.2 系统能力全景

| 能力模块 | 技术方案 | 性能指标 |
|----------|----------|----------|
| 激光-惯性里程计 (LIO) | Point-LIO (iVox) / FAST-LIO (ikd-Tree) | ~50Hz 位姿输出，逐点处理延迟 <5ms |
| 近场地形分析 | 5cm 体素栅格 + 分位数地面估计 | ~1m² 覆盖，1s 衰减时间 |
| 远场地形分析 | 10cm 体素栅格 + 分位数地面估计 | ~82m² 覆盖，4s 衰减时间 |
| 2D SLAM | slam_toolbox 在线占据栅格建图 | 5cm 分辨率，支持回环检测 |
| 全局规划 | Theta* 规划器 | 运动学可行路径 |
| 局部控制 | MPPI 采样最优控制器 | 1000 轨迹/次，40 步时域 (2s) |
| 终端逼近 | GoalApproachController | <1m 减速至 0.3m/s，<0.2m 直驱 |
| 任务决策 | Python FSM | 4 状态：standby/attack/restore/outpost |
| 通信 | UDP 二进制帧协议 | 500μs 发送间隔，1s 看门狗超时 |

### 1.3 技术栈

```
硬件层：  Livox MID-360 (40m 范围, 360°×59° FOV, 内置 IMU @200Hz)
操作系统： Ubuntu 22.04 + ROS 2 Humble
中间件：  Eclipse Cyclone DDS (rmw_cyclonedds_cpp)
容器化：  Docker (osrf/ros:humble-desktop-full)
构建系统： colcon + CMake (C++17)
语言：    C++ (核心算法) + Python (决策层) + YAML (配置) + XML (行为树)
库依赖：  PCL, Eigen3, tf2, nav2, slam_toolbox, Livox-SDK2, small_gicp
```

---

## 2. 快速上手

### 2.1 环境要求

| 组件 | 最低要求 | 推荐配置 |
|------|----------|----------|
| 操作系统 | Ubuntu 22.04 | Ubuntu 22.04 + RT 内核 |
| Docker | 20.10+ | 24.0+ |
| 内存 | 8 GB | 16 GB+ |
| CPU | 4 核 | 8 核+ |
| GPU | 不需要 | 不需要（纯 CPU 计算） |
| LiDAR | Livox MID-360 | Livox MID-360 |

### 2.2 Docker 镜像构建详解

**Dockerfile 解析（逐层说明）：**

```dockerfile
# 第1层：基础镜像 — ROS 2 Humble 桌面完整版
FROM osrf/ros:humble-desktop-full

# 第2层：配置中科大 APT 镜像源（国内加速）
RUN sed -i 's@//.*archive.ubuntu.com@//mirrors.ustc.edu.cn@g' /etc/apt/sources.list

# 第3层：配置 ROS 2 中科大镜像源
RUN curl -sSL https://mirrors.ustc.edu.cn/rosdistro/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# 第4层：安装系统依赖
# - Cyclone DDS (rmw_cyclonedds_cpp)：替代默认 FastRTPS，更低延迟
# - clang + clangd：C++ 编译器与 LSP 服务器
# - PCL：点云处理库
# - GTSAM + Ceres：非线性优化库（SLAM 备用）
# - libomp：OpenMP 并行加速
RUN apt update && apt install -y \
    cmake clang clangd ros-humble-rmw-cyclonedds-cpp \
    ros-humble-pcl-ros ros-humble-pcl-conversions \
    libomp-dev libceres-dev ros-humble-gtsam ...

# 第5层：编译 small_gicp（广义 ICP 库）
# 从 dependency/ 目录复制源码，编译安装到系统路径
COPY dependency/small_gicp /tmp/small_gicp
RUN cd /tmp/small_gicp && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && make install

# 第6层：环境变量
ENV ROS_DOMAIN_ID=0                        # ROS 域 ID（单机=0）
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # 强制使用 Cyclone DDS
```

### 2.3 三种 Compose 文件的差异

#### docker-compose.yml（跨平台默认版）

```yaml
services:
  mas_nav:
    build: .
    network_mode: "host"       # 主机网络模式，ROS 节点直接通信
    environment:
      - DISPLAY=${DISPLAY}     # X11 转发（GUI 应用）
    volumes:
      - .:/home/ros2_ws/src    # 源码挂载（开发模式）
```

#### docker_compose_linux.yml（生产环境）

额外配置：
- `restart: always` — 容器崩溃自动重启
- `privileged: true` — 硬件访问权限

#### docker_compose_windows.yml（WSL2 开发环境）

额外配置：
- `DISPLAY=host.docker.internal:0.0` — WSL2 特殊的 X11 转发地址
- 网络模式为 `bridge` 而非 `host`（WSL2 兼容性）

### 2.4 colcon 编译详解

```bash
# 步骤1：安装 ROS 依赖（自动解析 package.xml）
# --from-paths src：从 src 目录递归找包
# --ignore-src：只安装系统级依赖，不重装已有包
# -y：跳过确认
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# 步骤2：colcon 编译
colcon build \
  --symlink-install \          # 软链接安装（修改 Python/配置不需要重新编译）
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \          # Release 优化
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \  # 生成 compile_commands.json（clangd 需要）
  --parallel-workers 4         # 并行编译数（根据 CPU 核心数调整）

# 步骤3：source 环境（加载 ROS 包路径）
source install/setup.bash
```

### 2.5 常用调试命令

```bash
# === TF 调试 ===
# 查看完整 TF 树
ros2 run rqt_tf_tree rqt_tf_tree
# 查看两个坐标系之间的实时变换
ros2 run tf2_ros tf2_echo map base_link

# === 话题调试 ===
# 列出所有话题
ros2 topic list
# 话题发布频率
ros2 topic hz /aft_mapped_to_init
# 话题内容预览（--no-arr 跳过数组内容）
ros2 topic echo /odometry --no-arr
# 话题带宽监控
ros2 topic bw /livox/lidar

# === 节点调试 ===
ros2 node list           # 列出所有节点
ros2 node info /point_lio # 查看节点详情（订阅/发布/服务）

# === 参数调试 ===
ros2 param list /controller_server        # 列出参数
ros2 param get /controller_server FollowPath.temperature  # 读取参数
ros2 param set /controller_server FollowPath.temperature 3.0  # 动态修改（需节点支持）

# === 服务调用 ===
# 清除代价地图
ros2 service call /local_costmap/clear_entirely_local_costmap \
  nav2_msgs/srv/ClearEntireCostmap

# === 录制与回放 ===
ros2 bag record -a -o my_session    # 录制所有话题
ros2 bag play my_session            # 回放
```

---

## 3. 系统架构

### 3.1 完整数据流（逐级详解）

系统的数据流共分 **8 个处理层级**，每个层级都是对上一级数据的加工：

```
层级 1：物理传感器
    [Livox MID-360]
    物理输出：40 线激光点云 @ 200,000 pts/s + 内置 IMU @ 200Hz
    电信号 → UDP 以太网帧 → livox_ros_driver2 接收

层级 2：驱动层 (livox_ros_driver2)
    输入：  UDP 以太网帧（Livox 私有协议）
    处理：  解析帧头/帧尾，提取点坐标+反射率+时间戳
    输出：  /livox/lidar (CustomMsg) — 原始点云（Livox 格式）
           /livox/imu (Imu) — 内置 IMU @ 200Hz
           注意：CustomMsg 每个点包含 xyz + reflectivity + tag + offset_time

层级 3：里程计层 (Point-LIO)
    输入：  /livox/lidar + /livox/imu
    处理：  见 §7.2 详解（IEKF + iVox 最近邻 + 点到面残差）
    输出：  /aft_mapped_to_init (Odometry, lidar_odom 帧)
           /cloud_registered (PointCloud2, 世界帧配准点云)
           /path (Path, 轨迹历史)

层级 4：帧适配层 (loam_interface + sensor_scan_generation)
    输入：  /aft_mapped_to_init + /cloud_registered
    处理：  帧变换（lidar_odom → odom → base_footprint）
    输出：  /lidar_odometry (odom 帧里程计)
           /odometry (base_footprint 帧里程计)
           /sensor_scan (传感器帧点云)

层级 5：感知层（并行三个管道）
    管道 A - 地形分析：
        输入： /sensor_scan
        近场： terrain_analysis → /terrain_map (5cm 体素)
        远场： terrain_analysis_ext → /terrain_map_ext (10cm 体素)
    管道 B - 2D SLAM：
        输入： /terrain_map_ext → pointcloud_to_laserscan → /obstacle_scan
        处理： slam_toolbox → /map (2D 占据栅格地图)
    管道 C - 云台补偿：
        输入： /cmd_vel → fake_vel_transform → 速度帧变换

层级 6：代价地图层 (Nav2 Costmaps)
    输入：  /terrain_map + /terrain_map_ext + /map
    处理：  IntensityVoxelLayer（自定义）+ InflationLayer（标准）
    输出：  local_costmap (5m 滚动窗口) + global_costmap (20m 固定)

层级 7：导航层 (Nav2 Stack)
    输入：  两个 costmap + 目标位姿
    处理：  Theta* (全局路径) → SimpleSmoother (平滑) → MPPI (局部控制)
           → GoalApproachController (终端包装)
    输出：  /cmd_vel (geometry_msgs/Twist)

层级 8：决策层 (rm_decision)
    输入：  /referee_data (裁判系统) + Nav2 action 状态
    处理：  Python FSM (standby→attack→restore→outpost)
    输出：  NavigateThroughPoses action 目标
```

### 3.2 数据流中的关键话题汇总

| 话题名 | 消息类型 | 发布者 | 订阅者 | 频率 | 用途 |
|--------|----------|--------|--------|------|------|
| `/livox/lidar` | CustomMsg | livox_ros_driver2 | Point-LIO | ~50Hz | 原始点云 |
| `/livox/imu` | Imu | livox_ros_driver2 | Point-LIO | 200Hz | IMU 数据 |
| `/aft_mapped_to_init` | Odometry | Point-LIO | loam_interface | ~50Hz | LIO 里程计 |
| `/cloud_registered` | PointCloud2 | Point-LIO | sensor_scan_generation | ~10Hz | 配准点云 |
| `/lidar_odometry` | Odometry | loam_interface | sensor_scan_generation | ~50Hz | odom 帧里程计 |
| `/odometry` | Odometry | sensor_scan_generation | Nav2 | ~50Hz | base_footprint 帧里程计 |
| `/terrain_map` | PointCloud2 | terrain_analysis | local_costmap | ~10Hz | 近场障碍物 |
| `/terrain_map_ext` | PointCloud2 | terrain_analysis_ext | global_costmap, pcl2laser | ~5Hz | 远场障碍物 |
| `/obstacle_scan` | LaserScan | pointcloud_to_laserscan | slam_toolbox | ~5Hz | SLAM 扫描 |
| `/map` | OccupancyGrid | slam_toolbox | global_costmap | ~1Hz | 2D 栅格地图 |
| `/cmd_vel` | Twist | Nav2 | fake_vel_transform, ros2_comm | 40Hz | 速度指令 |
| `/referee_data` | RefereeData | ros2_comm | rm_decision | ~10Hz | 裁判数据 |

### 3.3 包间依赖关系（编译期 vs 运行期）

**编译期依赖：**
```
sentry_bringup ──依赖──> 所有其他 sentry_* 包
sentry_utils/ros2_comm ──依赖──> 自定义消息包 (RefereeData)
sentry_utils/pb_nav2_plugins ──依赖──> nav2_costmap_2d
sentry_location/Point-LIO ──依赖──> Eigen3, PCL, Livox-SDK2
```

**运行期依赖（话题通信）：**
- Point-LIO 必须在 livox_ros_driver2 之后启动
- terrain_analysis 必须在 sensor_scan_generation 之后启动
- Nav2 所有节点必须在两个 costmap 激活后启动
- rm_decision 必须在 Nav2 就绪后启动

**关键点：** 编译期大部分包独立，但运行期有严格的启动顺序（通过 TimerAction 保证）。

---

## 4. 分阶段启动流程

### 4.1 为什么需要分阶段启动

ROS 2 的节点启动是并发的。如果所有节点同时启动，会出现以下问题：

1. **TF 不存在**：传感器节点需要 TF 树来发布带 frame_id 的数据，如果 `robot_state_publisher` 还没启动，TF 树就不完整
2. **数据流断裂**：terrain_analysis 需要 sensor_scan_generation 的输出，如果 sensor_scan_generation 还没启动，terrain_analysis 收不到数据
3. **Nav2 崩溃**：Nav2 启动时检查 costmap 数据，如果 costmap 为空会进入错误状态
4. **Lifecycle 节点死锁**：Nav2 采用 lifecycle 管理，节点间存在状态依赖

### 4.2 TimerAction 源码分析

`bringup_pointlio_map.launch.py` 中的实际启动时序（与 CLAUDE.md 文档表保持一致但更精确）：

```python
# 源码中的实际延迟值：
# phase 1（TF 基础）:
add_delayed(static_map_to_odom_node, 0.1)      # map→odom 静态 TF
add_delayed(robot_state_publisher_node, 0.5)    # URDF TF 树

# phase 2（传感器 + LIO + SLAM）:
add_delayed(livox_driver_node, 1.0)             # LiDAR 驱动
add_delayed(point_lio_node, 1.5)                # Point-LIO
add_delayed(slam_toolbox_node, 2.0)             # 2D SLAM
add_delayed(map_saver_node, 2.5)                # 地图保存服务
add_delayed(map_lifecycle_manager_node, 3.0)    # 地图生命周期管理

# phase 3（处理节点）:
add_delayed(loam_interface_node, 3.5)           # LIO→odom 帧适配
add_delayed(sensor_scan_generation_node, 4.0)   # 传感器帧输出
add_delayed(start_terrain_analysis_cmd, 4.5)    # 近场地形
add_delayed(start_terrain_analysis_ext_cmd, 5.0)# 远场地形
add_delayed(start_pointcloud_to_laserscan_node, 5.5)  # PCL→LaserScan
add_delayed(fake_vel_transform_node, 6.0)       # 云台速度补偿

# phase 4（Nav2 导航栈）:
add_delayed(nav2_bringup_group, 7.0)            # 6个Nav2节点 + lifecycle_manager

# phase 5（可视化 + 决策）:
add_delayed(rviz_node, 8.5)                     # RViz 可视化
add_delayed(rm_decision_node, 9.0)              # FSM 决策
add_delayed(ros2_comm, 9.5)                     # UDP 通信
```

### 4.3 各阶段内节点的启动依赖

```
Phase 1：TF 基础设施
  static_map_to_odom_node:
    包：tf2_ros
    功能：发布 map→odom 的静态变换（identity）
    参数：0 0 0 0 0 0 map odom

  robot_state_publisher_node:
    包：robot_state_publisher
    功能：读取 URDF，发布 base_footprint→base_link→livox_link 的 TF
    为什么第二个启动：需要 URDF 文件存在

Phase 2：传感器与核心算法
  livox_driver_node:
    依赖 TF 树（需要知道 LiDAR 安装位置来填充 frame_id）
    参数来自 mid360_config.json（IP、端口）
    注意：使用 respawn=use_respawn，崩溃后自动重启

  point_lio_node:
    依赖：livox_driver_node 开始发布数据
    可执行文件：pointlio_mapping
    参数：nav_config_pointlio.yaml 中的 point_lio 段

  slam_toolbox_node:
    依赖：pointcloud_to_laserscan（但它在 Phase 3！）
    → 这是"渴望启动"：尽早启动并等待数据到来
    可执行文件：sync_slam_toolbox_node（同步模式）

Phase 3：数据处理管道
  为什么 loam_interface 和 sensor_scan_generation 需要 wait：
    loam_interface 需要 /aft_mapped_to_init 有数据
    且需要 base_link→livox_frame 的 TF 存在才能做帧变换

Phase 4：Nav2 全部节点
  nav2_bringup_group 使用 GroupAction + PushRosNamespace
  内部包含 6 个 lifecycle 节点 + 1 个 lifecycle_manager
  所有节点共享同一份 configured_params（RewriteYaml 处理后）

Phase 5：决策与 UI
  rm_decision 最后启动：需要 Nav2 action server 全部就绪
  启动参数：--csv-attack, --csv-restore, --csv-outpost
```

### 4.4 Lifecycle 节点管理

Nav2 使用 **ROS 2 Managed/Lifecycle Nodes**。每个节点有 4 个状态：

```
Unconfigured → Configuring → Inactive → Activating → Active
                                     ↑               ↓
                                     ← Deactivating ←
```

Lifecycle Manager 负责按顺序激活所有节点：
- 先激活 costmap 相关节点
- 再激活 planner → smoother → controller
- 最后激活 behavior_server → bt_navigator

---

## 5. 包地图

### 5.1 完整包详解

#### sentry_bringup（中央调度）

```
sentry_bringup/
├── launch/
│   ├── bringup_pointlio_map.launch.py    # 主要启动文件 (459 行)
│   └── bringup_fastlio_map.launch.py     # FAST-LIO 版本
├── configs/
│   ├── nav_config_pointlio.yaml          # 统一配置 (684 行，核心！)
│   └── mid360_config.json               # LiDAR 网络配置
├── map/simulation/                       # 预建 SLAM 地图
│   ├── RMUC_2024.pgm + RMUC_2024.yaml
│   └── RMUL_2024.pgm + RMUL_2024.yaml
├── behavior_trees/
│   ├── navigate_to_pose_w_replanning_and_recovery.xml
│   └── navigate_through_poses_w_replanning_and_recovery.xml
└── rviz/
    └── sentry.rviz                        # 预配置 RViz 布局
```

**关键设计：** `sentry_bringup` 本身不含编译代码，是纯配置+启动包。这符合 ROS 2 最佳实践：将调度逻辑与算法实现分离。

#### sentry_description（机器人模型）

URDF 定义了机器人的运动学链：

```xml
<!-- base_footprint → base_link (z=0.1m) -->
<joint name="base_footprint_joint" type="fixed">
  <parent link="base_footprint"/>
  <child link="base_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<!-- base_link → livox_link (LiDAR 安装位姿) -->
<joint name="livox_joint" type="fixed">
  <parent link="base_link"/>
  <child link="livox_link"/>
  <origin xyz="0.09 0.03 0.11" rpy="0.3572 0 1.5708"/>
</joint>

<!-- base_link → lakibeam_frame (前向激光) -->
<joint name="lakibeam_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lakibeam_frame"/>
  <origin xyz="-0.17 0.01 0.17" rpy="0 0 0"/>
</joint>
```

**为什么 LiDAR 的 rpy 是 (0.3572, 0, 1.5708)？**
- `0.3572 rad ≈ 20.5°`：LiDAR 的俯仰安装角
- `1.5708 rad = 90°`：LiDAR 的偏航安装角（MID-360 横向安装）

#### livox_ros_driver2（LiDAR 驱动）

**架构层次：**
```
硬件层 (MID-360)
  ↔ UDP (点云数据端口 60000, IMU 端口 60001)
通信层 (comm/comm.cpp)
  → ldq.cpp (无锁队列缓冲)
设备层 (lds_lidar.cpp)
  → lddc.cpp (点云数据聚合)
ROS 接口层 (driver_node.cpp)
  → 发布 /livox/lidar (CustomMsg)
  → 发布 /livox/imu (Imu)
  → 发布 /livox/lidar/pointcloud (PointCloud2)
```

**CustomMsg vs PointCloud2：**
- CustomMsg 保留 Livox 原生信息（tag, offset_time），供 LIO 使用
- PointCloud2 是标准 ROS 格式，供通用 PCL 工具使用

#### Point-LIO（点级 LIO，默认）

**与 FAST-LIO 的核心差异：**

| 维度 | Point-LIO | FAST-LIO |
|------|-----------|----------|
| 处理粒度 | **逐点**（每收到一个点立即处理） | **逐帧**（累积一帧后批量处理） |
| 数据结构 | iVox（增量体素） | ikd-Tree（增量 kd 树） |
| 状态估计 | 双 IEKF（input/output 两种模式） | 单 IEKF |
| IMU 处理 | 预积分预测 + IEKF 更新 | 预积分预测 |
| 延迟 | **<5ms** | ~50ms |
| 地图管理 | 增量添加（MapIncremental） | 全量重建 |

#### sentry_utils 包组

**设计哲学：** 每个 sentry_utils 子包只做一件事，通过 ROS 话题松耦合。

### 5.2 包的 CMake 依赖链

大多数 sentry_utils 包的 CMakeLists.txt 结构相似：

```cmake
cmake_minimum_required(VERSION 3.8)
project(terrain_analysis)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)

add_executable(terrainAnalysis src/terrain_analysis.cpp)
target_include_directories(terrainAnalysis PRIVATE include)
ament_target_dependencies(terrainAnalysis rclcpp sensor_msgs PCL)

install(TARGETS terrainAnalysis DESTINATION lib/${PROJECT_NAME})
ament_package()
```

---

## 6. 坐标系树

### 6.1 完整 TF 树

```
map (世界固定帧)
  │
  │  StaticTransformBroadcaster: identity
  │  发布者: static_map_to_odom_node
  │  修正者: slam_toolbox（回环检测时覆盖此 TF）
  v
odom (漂移帧，长距离有累积误差)
  │
  │  loam_interface: /lidar_odometry
  │  输入：Point-LIO 的 /aft_mapped_to_init (lidar_odom 帧)
  │  处理：TF 查询 base_link→livox_frame，计算 odom→base_link
  │
  │  sensor_scan_generation: /odometry
  │  输入：/lidar_odometry
  │  处理：从 lidar_odom 帧变换到 base_footprint 帧
  v
base_footprint (机器人在地面的投影)
  │
  │  robot_state_publisher: URDF fixed joint (z=0.1m)
  v
base_link (机器人本体坐标系原点)
  │
  │  fake_vel_transform: 动态 TF
  │  补偿云台旋转，输出 fake_base_link
  v
fake_base_link (云台补偿后的虚拟基座)
  │
  ├── livox_link
  │   固定偏移: xyz=(0.09, 0.03, 0.11), rpy=(0.3572, 0, 1.5708)
  │   用途: LiDAR 安装点
  │
  ├── lakibeam_frame
  │   固定偏移: xyz=(-0.17, 0.01, 0.17)
  │   用途: 前向辅助传感器
  │
  └── chassis_link
      用途: 底盘本体
```

### 6.2 为什么需要 fake_base_link

RoboMaster 哨兵机器人的云台（gimbal）可以独立旋转。这意味着：

1. LiDAR 安装在云台上（固定），但云台相对底盘旋转
2. Nav2 的路径跟随假设 `base_link` 朝向与机器人运动方向一致
3. 如果直接用 `base_link`，云台旋转会导致 Nav2 认为机器人朝向变了

**fake_vel_transform 的解决方案：**

```
真正的 base_link 朝向可能被云台旋转影响
        ↓
fake_vel_transform 发布 fake_base_link 的 TF
        ↓
fake_base_link 始终指向机器人运动方向（不变）
        ↓
Nav2 使用 fake_base_link 作为 robot_base_frame
```

实际机制：
1. 监听 `/cmd_vel`，根据速度方向计算运动朝向
2. 补偿云台旋转角度，计算 `base_link → fake_base_link` 的 TF
3. 将 `/cmd_vel` 从 fake_base_link 帧变换回 base_link 帧发给底盘

### 6.3 TF 变换的链式计算

当 Nav2 想知道机器人在 map 中的位置时：

```
map → odom (slam_toolbox 动态更新)
  ×
odom → base_footprint (Point-LIO → loam_interface → sensor_scan_generation)
  ×
base_footprint → base_link (URDF)
  ×
base_link → fake_base_link (fake_vel_transform)
```

中间件 tf2 会缓存这些变换并自动完成链式查找（最长 10 秒缓存）。

---

## 7. 核心子系统详解

### 7.1 Livox MID-360 激光雷达驱动

#### 7.1.1 MID-360 硬件特性

| 参数 | 值 |
|------|-----|
| 测距范围 | 0.1m - 40m (@80% 反射率) |
| FOV | 360° 水平 × 59° 垂直 (-22° ~ +37°) |
| 点频 | 200,000 pts/s |
| 内置 IMU | 200Hz (陀螺仪 + 加速度计) |
| 通信方式 | 以太网 (UDP) |
| 供电 | 9-18V DC |
| 重量 | ~300g |

#### 7.1.2 配置文件 MID360_config.json 详解

```json
{
  "lidar_configs": [{
    "ip": "192.168.1.136",
    "pcl_data_port": 60000,
    "imu_data_port": 60001,
    "multi_topic": 1
  }],
  "xfer_format": 0
}
```

- `multi_topic: 1`：启用多话题模式，IMU 数据通过独立话题发布（否则混在点云中）
- `xfer_format: 0`：使用 Livox 自定义格式（CustomMsg），而非 PCL PointXYZ
  - `0 = Livox custom` → 包含 tag、offset_time 等 LIO 需要的信息
  - `1 = PCL PointXYZ` → 仅 xyz，信息丢失

#### 7.1.3 CustomMsg 消息结构

```cpp
// msg/CustomPoint.msg
float32 x, y, z           // 点坐标（LiDAR 坐标系）
float32 reflectivity       // 反射率 (0-255)
uint8 tag                  // 点的标签信息
uint8 line                 // 激光线号
float64 offset_time        // 相对于帧头的时间偏移（秒）

// msg/CustomMsg.msg
std_msgs/Header header     // 时间戳 + 帧 ID
uint64 timebase            // 帧的时间基准
uint32 point_num           // 点数
CustomPoint[] points       // 点数组
```

**offset_time 的重要性：** MID-360 内部 40 条激光线不是同时发射的，而是按顺序扫描。一个完整帧（0.02s）内，第一个点和最后一个点的实际采集时间差约 20ms。offset_time 记录每个点相对于帧头的偏差，LIO 用它来做运动补偿（去畸变）。

#### 7.1.4 通信架构

```
┌─────────────────────────────────────┐
│          MID-360 硬件              │
│  UDP Socket (点云:60000, IMU:60001) │
└──────────┬──────────────────────────┘
           │ 以太网
           v
┌─────────────────────────────────────┐
│  livox_ros_driver2                  │
│  ┌─────────────────────────────┐    │
│  │ comm/comm.cpp               │    │
│  │ (UDP recvfrom, 非阻塞I/O)   │    │
│  └──────────┬──────────────────┘    │
│             v                       │
│  ┌─────────────────────────────┐    │
│  │ comm/ldq.cpp                │    │
│  │ (无锁队列, 生产者-消费者)    │    │
│  └──────────┬──────────────────┘    │
│             v                       │
│  ┌─────────────────────────────┐    │
│  │ lds_lidar.cpp + lddc.cpp   │    │
│  │ (设备管理 + 数据聚合)       │    │
│  └──────────┬──────────────────┘    │
│             v                       │
│  ┌─────────────────────────────┐    │
│  │ driver_node.cpp             │    │
│  │ (ROS 2 发布器)              │    │
│  └─────────────────────────────┘    │
└─────────────────────────────────────┘
```

**为什么使用无锁队列？** 点云数据量极大（200,000 pts/s），锁竞争会导致帧丢失。无锁队列使用 CAS（Compare-and-Swap）原子操作，生产者和消费者可以并行运行，保证数据不丢失。

---

### 7.2 Point-LIO：点级激光-惯性里程计

> 📘 **专题详解**：[[MAS_Nav_Point-LIO详解]] — IEKF 数学模型、点到面残差观测、iVox 体素结构、IMU 饱和检测、增量地图更新、完整调参指南

Point-LIO 是导航系统最核心的算法模块，使用**误差状态迭代卡尔曼滤波器（Error-State IEKF）**实现逐点激光-惯性里程计。核心创新是**每收到一个 LiDAR 点立即用 IEKF 更新状态**，延迟仅 <5ms（vs FAST-LIO 的 ~50ms）。状态向量 24-30 维，包括位置、旋转、速度、IMU 零偏和重力向量。观测模型使用**点到面残差**（优于传统的点到点 ICP），通过 iVox 增量体素进行最近邻搜索。

### 7.3 帧适配器

#### 7.3.1 loam_interface

**功能：** 将 Point-LIO 的 `aft_mapped_to_init`（lidar_odom 帧）转换为标准 ROS 约定的 `odom` 帧。

**关键代码逻辑：**
```cpp
// 订阅 /aft_mapped_to_init (lidar_odom 帧的里程计)
// 订阅 /cloud_registered (lidar_odom 帧的配准点云)

// 初始化：查找 base_link → lidar_frame 的 TF
auto tf_stamped = tf_buffer_->lookupTransform(
    base_frame_, lidar_frame_, msg->header.stamp, 0.5s);
// 这个 TF 来自 URDF（固定值）

// 里程计转换：
// lidar_odom → lidar 的变换 + odom → lidar_odom 的初值
// = odom → 当前 LiDAR 位置的完整变换
tf_odom_to_lidar = tf_odom_to_lidar_odom_ * tf_lidar_odom_to_lidar;

// 发布 /lidar_odometry (odom 帧)
// 发布 /registered_scan (odom 帧)
```

**为什么要这个适配层？** Point-LIO 使用自己的帧约定（lidar_odom 帧），与 Nav2 期望的 `odom` 帧不兼容。loam_interface 是解耦层，使我们可以替换 LIO 算法而不影响下游。

#### 7.3.2 sensor_scan_generation

**功能：** 将 odom 帧的数据进一步转换到 base_footprint 帧，生成 Nav2 直接可用的里程计和传感器扫描。

```
/lidar_odometry (odom 帧) ──>  /odometry (base_footprint 帧)
/registered_scan (odom 帧) ──>  /sensor_scan (livox_frame 帧)
```

Nav2 期望 `/odometry` 在 `base_footprint` 帧下发布，所以这个适配是必要的。

---

### 7.4 地形分析与 2D SLAM

> 📘 **专题详解**：[[MAS_Nav_地形分析与SLAM详解]] — 两级体素栅格、分位数地面估计、动态障碍物清除、slam_toolbox 回环检测

两级地形分析是导航的"眼睛"：近场（5cm 体素，1s 衰减）用于局部避障，远场（10cm 体素，4s 衰减）用于全局规划。核心创新是**分位数地面估计 + IntensityVoxelLayer**——用体素内 Z 坐标的分位数估计地面高度，障碍物离地高度（intensity）传给代价地图做连续判定（而非传统的二值障碍物判定），有效区分草地（可通行）与墙壁（不可通行）。SLAM 使用 slam_toolbox 的 mapping 模式，输入来自 terrain_analysis_ext → pointcloud_to_laserscan 转换。

### 7.5 Nav2 导航栈

> 📘 **专题详解**：[[MAS_Nav_Nav2导航栈详解]] — 行为树 XML 源码分析、MPPI 采样最优控制、GoalApproachController 三模式切换、IntensityVoxelLayer 自定义插件

Nav2 采用 Theta* 全局规划 + MPPI 局部控制 + 行为树恢复策略。MPPI 的核心思想是**采样 1000 条轨迹，用 softmax 加权选出最优**（非求解优化问题），在 2s 时域内评估 Goal/PathAngle/Obstacles/PathFollow 等多维代价。GoalApproachController 包装器根据距离动态切换模式（透传→减速→直驱 P 控制），保证终端精度。

### 7.6 决策与 UDP 通信

> 📘 **专题详解**：[[MAS_Nav_决策与UDP通信]] — Python FSM 状态转换、CSV 航点管理、BasicNavigator 封装、UDP 二进制帧协议、看门狗安全机制

高层决策使用 Python FSM（4 状态：standby/attack/restore/outpost），通过 CSV 文件管理航点，比赛策略迭代只需修改 CSV 无需重编译。UDP 桥使用自定义二进制帧协议（每帧 ~15 字节），配合 1s 看门狗确保通信中断时自动急停。

---

## 8. 配置系统

### 8.1 统一配置文件详解

`nav_config_pointlio.yaml` 是所有运行时参数的单一来源（684 行，约 40 个参数组的完整配置）。

**参数加载机制：**

```python
# bringup_pointlio_map.launch.py
nav_config_file = 'configs/nav_config_pointlio.yaml'

# RewrittenYaml 处理模板替换 + 命名空间隔离
configured_params = ParameterFile(
    RewrittenYaml(
        source_file=nav_config_file,
        root_key=namespace,           # '' (空=无命名空间)
        param_rewrites={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'bt_navigator...default_nav_to_pose_bt_xml': nav_to_pose_bt,
            'bt_navigator...default_nav_through_poses_bt_xml': nav_through_poses_bt,
        },
        convert_types=True
    ),
    allow_substs=True
)

# 每个节点启动时读取自己对应的 YAML 段
Node(
    package='point_lio',
    parameters=[configured_params],   # 自动提取 point_lio: 段
)
```

### 8.2 完整参数组（全 40+ 段）

每个参数组对应一个 ROS 2 节点。以下为关键参数组：

```
nav_config_pointlio.yaml 结构：
├── livox_ros_driver2          # LiDAR 网络+格式配置
├── point_lio                  # LIO 滤波器+IMU+外参
├── loam_interface             # 帧名称配置
├── sensor_scan_generation     # 扫描周期+帧配置
├── terrain_analysis           # 近场体素+地面估计
├── terrain_analysis_ext       # 远场体素+地面估计
├── pointcloud_to_laserscan    # 高度/强度滤波器
├── fake_vel_transform         # 速度帧变换
├── slam_toolbox               # SLAM 模式+回环检测
├── controller_server          # MPPI全参数+GoalApproachCtrl
│   ├── GoalApproachController # 逼近距离+速度
│   └── FollowPath             # MPPI运动模型+所有critics
├── planner_server             # Theta* 规划器
├── smoother_server            # 路径平滑器
├── local_costmap              # 局部代价地图+IntensityVoxel
├── global_costmap             # 全局代价地图+IntensityVoxel
├── behavior_server            # 行为插件列表
├── velocity_smoother          # 速度+加速度限制
├── bt_navigator               # 行为树文件+服务器超时
├── waypoint_follower          # 航点跟踪器
├── pb_teleop_twist_joy        # 手柄映射
└── ros2_comm                  # UDP端口+看门狗
```

---

## 9. 关键设计模式

### 9.1 分阶段启动 (Staged Startup)

**解决的问题：** ROS 2 并发启动导致的竞态条件。
**实现方式：** TimerAction 在不同延迟后启动不同组的节点。
**优势：** 消除 "node X 需要 node Y 先启动" 的隐式依赖问题。

### 9.2 强度感知代价地图 (IntensityVoxelLayer)

**解决的问题：** 二值障碍物判定无法区分草地（可通行）和墙壁（不可通行）。
**实现方式：** 用 terrain_analysis 输出的障碍物离地高度作为强度值，代价地图根据强度阈值判定。
**优势：** 大幅减少误报，提高复杂地形下的通行能力。

### 9.3 两级感知回路 (Two-Stage Terrain Analysis)

**解决的问题：** 近处需要高精度快速更新，远处需要大范围慢更新。
**实现方式：** 两个独立节点，不同分辨率+衰减时间，各自输出到对应 costmap。
**优势：** 解耦快速/慢速感知，各自独立调优。

### 9.4 点级 LIO (Point-by-Point LIO)

**解决的问题：** 传统帧级 LIO 的延迟（一帧 50ms）。
**实现方式：** 每收到一个点立即处理，不等一帧结束。
**优势：** 延迟降低一个数量级（50ms → ~5ms），快速机动更稳定。

### 9.5 控制器的包装器模式 (Controller Wrapper)

**解决的问题：** MPPI 在近距离精度不足，PID 在远距离优化不够。
**实现方式：** GoalApproachController 包裹 MPPI，根据距离动态切换模式。
**优势：** 全局优化 + 终端精度，优雅的多模式切换。

### 9.6 统一配置 (Unified Config)

**解决的问题：** 参数分散在多个文件中，版本管理和调试困难。
**实现方式：** 一个 YAML 文件包含所有参数，launch 文件通过 RewrittenYaml 分发给各节点。
**优势：** 参数版本化简单，不会出现两个文件参数不一致的情况。

### 9.7 Cyclone DDS (高性能中间件)

**解决的问题：** 默认 FastRTPS 的延迟和 CPU 占用较高。
**实现方式：** 在 Dockerfile 中设置 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`。
**优势：** 更低延迟，更低 CPU 占用，更适合实时系统。

### 9.8 UDP 桥接 (UDP Bridge)

**解决的问题：** Docker 容器内外 ROS 网络互通复杂。
**实现方式：** 自定义二进制帧协议（帧头 0xAA/0xBB + 数据 + 帧尾 0x5A/0x5B）。
**优势：** 极简协议，最小带宽，无 ROS 依赖。

### 9.9 Python FSM 任务控制

**解决的问题：** 比赛策略需要频繁调整，C++ 重编译周期长。
**实现方式：** 高层决策用 Python 实现，算法用 C++，通过 ROS Action 解耦。
**优势：** 策略迭代只需修改 Python 和 CSV，无需重新编译。

---

## 10. 常见操作

### 10.1 调试 Point-LIO

```bash
# 检查点云是否正常到达
ros2 topic hz /livox/lidar          # 应该 ~50Hz
ros2 topic echo /livox/lidar --no-arr  # 看点数量

# 检查 LIO 是否在正常工作
ros2 topic hz /aft_mapped_to_init   # 应该 ~50Hz
ros2 topic echo /aft_mapped_to_init --no-arr  # 看位置是否在移动

# 检查配准后的点云
ros2 topic hz /cloud_registered
# 在 RViz 中可视化 /cloud_registered，应该看到对齐的地图

# 检查 IMU 数据
ros2 topic hz /livox/imu           # 应该 ~200Hz
ros2 topic echo /livox/imu --no-arr

# 常见问题排查：
# 问题：/aft_mapped_to_init 频率为 0
# → 检查 /livox/lidar 是否有数据
# → 检查 LiDAR IP 配置 (mid360_config.json)
# → 检查 Docker 网络模式（是否使用 host 模式）

# 问题：里程计漂移很大
# → 降低 gyr_cov（更信任陀螺仪）
# → 降低 acc_cov（更信任加速度计）
# → 检查 IMU 初始化是否充分（机器人需静止几秒）
```

### 10.2 MPPI 调参流程

```bash
# 1. 先确认基础导航能工作
ros2 launch sentry_bringup bringup_pointlio_map.launch.py
# 在 RViz 中点击 "2D Pose Estimate" 设置初始位姿
# 点击 "Nav2 Goal" 设置目标点
# 观察机器人是否能大致走向目标

# 2. 诊断具体问题
# 问题A：机器人走不到目标（收敛慢）
# → 增大 controller_server.FollowPath.critics.GoalCritic.weight (10 → 20)
# → 增大 controller_server.FollowPath.critics.GoalCritic.power (2.0 → 3.0)

# 问题B：机器人撞障碍物
# → 增大 controller_server.FollowPath.critics.ObstaclesCritic.critical_weight (50 → 100)
# → 增大 local_costmap.inflation_layer.inflation_radius (0.3 → 0.5)

# 问题C：机器人运动抖/seesaw
# → 增大 controller_server.FollowPath.temperature (2.0 → 3.0)
# → 增大 controller_server.FollowPath.lambda (0.01 → 0.05)

# 问题D：路径偏离（不走直线）
# → 增大 controller_server.FollowPath.critics.PathFollowCritic.weight (2 → 5)

# 问题E：冲过目标点
# → 增大 controller_server.GoalApproachController.approach_distance (1.0 → 1.5)
# → 减小 controller_server.GoalApproachController.approach_velocity (0.3 → 0.2)
```

### 10.3 地形分析调参

```bash
# 问题A：粗糙地面被标记为障碍物（太多红色在 costmap）
# → 增大 terrain_analysis.minObstacleHeight (0.05 → 0.08)
# → 减小 terrain_analysis.quantileZ (0.25 → 0.15)，更多点被认为是地面

# 问题B：真实障碍物漏检（机器人撞墙）
# → 减小 terrain_analysis.minObstacleHeight (0.05 → 0.03)
# → 增大 terrain_analysis.maxRelZ (0.3 → 0.5)

# 问题C：障碍物消失太慢（机器人绕过障碍物后仍绕行）
# → 减小 terrain_analysis.decayTime (1.0 → 0.5)

# 问题D：动态障碍物出现太慢
# → 增大 terrain_analysis.scanVoxelSize (0.05 → 0.08)，更快的体素填充
```

### 10.4 添加新航点集

```bash
# 1. 创建 CSV 文件
cat > /home/ros2_ws/my_waypoints.csv << EOF
id, pose_x, pose_y, pose_z, rot_x, rot_y, rot_z, rot_w
1, 1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0
2, 3.0, 4.0, 0.0, 0.0, 0.0, 0.707, 0.707
3, 5.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0
EOF

# 2. 修改 launch 文件中的 CSV 路径
# 编辑 bringup_pointlio_map.launch.py:
# PATH_ATTACK = "/home/ros2_ws/my_waypoints.csv"

# 3. 或使用 RViz 航点编辑器
ros2 launch waypoint_editor waypoint_editor.launch.py
# 在 RViz 中点击 map 添加航点 → 导出 CSV

# 4. 重启 launch
```

### 10.5 保存和加载 SLAM 地图

```bash
# 保存当前地图
ros2 run nav2_map_server map_saver_cli -f my_map
# 生成 my_map.pgm (图像) + my_map.yaml (元数据)

# 加载地图（localization 模式）
# 修改 nav_config_pointlio.yaml 中：
# slam_toolbox.mode: "localization"
# 并将地图文件放入 sentry_bringup/map/

# 或通过服务调用加载
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \
  "{map_url: '/home/ros2_ws/my_map.yaml'}"
```

### 10.6 添加新的 Nav2 插件

```cpp
// 以自定义 Controller 为例：

// 1. 创建包 sentry_utils/my_controller/
// 2. 实现类继承 nav2_core::Controller
class MyController : public nav2_core::Controller {
public:
    void configure(...) override;
    void setPlan(const nav_msgs::msg::Path& path) override;
    geometry_msgs::msg::TwistStamped computeVelocityCommands(...) override;
};

// 3. 导出插件（plugins.xml）
<library path="my_controller">
  <class type="my_controller::MyController" base_class_type="nav2_core::Controller">
    <description>My custom controller</description>
  </class>
</library>

// 4. 在 nav_config_pointlio.yaml 中注册
controller_server:
  ros__parameters:
    controller_plugin_ids: ["GoalApproachController"]
    GoalApproachController:
      inner_controller: "MyController"  # 替换内部控制器

// 5. 确保 CMakeLists.txt 将 plugins.xml 安装到正确位置
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})
```

---

## 11. 仓库结构

```
mas_nav/
├── Dockerfile                 # Docker 镜像构建文件
├── docker-compose.yml         # 跨平台 Compose
├── docker_compose_linux.yml   # Linux 生产环境 Compose
├── docker_compose_windows.yml # WSL2 开发环境 Compose
├── clean_up.sh               # 清理脚本 (killall ROS2 + restart daemon)
├── dependency/
│   └── small_gicp/           # 第三方 GICP 库 (header-only, 用于重定位)
│
├── sentry_bringup/           # ⭐ 中央调度包 (无编译代码)
│   ├── launch/
│   │   ├── bringup_pointlio_map.launch.py    # 主启动文件 (459行)
│   │   └── bringup_fastlio_map.launch.py     # FAST-LIO 版本
│   ├── configs/
│   │   ├── nav_config_pointlio.yaml          # 统一配置 (684行, 核心)
│   │   ├── nav_config_fastlio.yaml           # FAST-LIO 配置
│   │   └── mid360_config.json                # LiDAR 网络配置
│   ├── map/simulation/        # 比赛场地预建地图 (.pgm + .yaml)
│   ├── pcd/simulation/        # 预录点云数据
│   ├── behavior_trees/        # BT XML 行为树文件
│   └── rviz/                  # RViz 预配置文件
│
├── sentry_description/        # 机器人 URDF 模型 + STL 网格
│   ├── urdf/sentry_description.urdf
│   └── meshes/mid360.STL, LakiBeam.STL, base_link.STL
│
├── sentry_hardware/
│   └── livox_ros_driver2/     # Livox 官方 ROS2 驱动 v1.1.0
│       ├── src/               # 驱动核心 (driver_node, lds, comm, callbacks)
│       ├── config/            # LiDAR JSON 配置
│       └── msg/               # 自定义消息 (CustomMsg, CustomPoint)
│
├── sentry_location/
│   ├── Point-LIO/             # ⭐ 点级 LIO (默认)
│   │   ├── src/               # laserMapping, Estimator, IMU_Processing, li_initialization
│   │   └── include/ivox/      # iVox 增量体素结构
│   └── FAST_LIO/              # 遗留: ikd-Tree LIO
│       ├── src/laserMapping.cpp    # 主节点 (1429行)
│       └── include/ikd-Tree/       # ikd 树 + IKFoM 工具包
│
└── sentry_utils/              # 12 个工具包
    ├── loam_interface/        # LIO → 标准 odom 帧适配
    ├── sensor_scan_generation/# 传感器帧扫描 + 底盘里程计
    ├── terrain_analysis/      # 近场地形分析 (5cm 体素)
    ├── terrain_analysis_ext/  # 远场地形分析 (10cm 体素)
    ├── pointcloud_to_laserscan/ # PointCloud2 → LaserScan
    ├── fake_vel_transform/    # 云台旋转速度补偿
    ├── goal_approach_controller/ # Nav2 控制器包装器
    ├── pb_omni_pid_pursuit_controller/ # PID 全向控制器
    ├── pb_nav2_plugins/       # IntensityVoxelLayer + BackUpFreeSpace
    ├── pb_teleop_twist_joy/   # Xbox 手柄遥控
    ├── ros2_comm/             # UDP 裁判系统桥接
    ├── rm_decision/           # Python FSM 决策 (auto_fsm, robot_navigator, callback_msg)
    └── waypoint_editor/       # RViz 航点编辑插件
```

---

## 12. 学习路线建议

### 第 1 阶段：环境与基础（第 1-3 天）

**目标：** 能编译、启动系统，理解数据流。

1. **Docker 构建**：阅读 Dockerfile 的各层依赖，理解为什么选择 Cyclone DDS
2. **colcon 编译**：理解 `--symlink-install`、`--packages-up-to` 的作用
3. **启动系统**：在模拟数据下启动 bringup，观察日志输出
4. **TF 树浏览**：`ros2 run rqt_tf_tree rqt_tf_tree`，直观看到坐标系层级
5. **话题监控**：`ros2 topic list && ros2 topic hz /livox/lidar /aft_mapped_to_init /cmd_vel`

### 第 2 阶段：定位与感知（第 4-8 天）

**目标：** 理解 LIO 和地形分析的数学原理。

6. **读懂 Point-LIO 的 IEKF**：
   - 先理解标准 EKF 的 predict-update 循环
   - 再理解 Point-LIO 如何在其中嵌入点到面残差
   - 读 `h_model_output()` 函数直到理解每一步
7. **理解 IMU 预积分**：`IMU_Processing.cpp` 中的中点积分法
8. **理解地形分析**：
   - 从 `terrain_analysis.cpp` 入手理解栅格化+分位数
   - 理解 intensity 作为障碍物高度的设计
9. **跟踪一个点**：在脑海中模拟一个 LiDAR 点从驱动 → LIO → 帧适配 → 地形分析的完整路径

### 第 3 阶段：导航与决策（第 9-14 天）

**目标：** 理解导航栈和决策逻辑。

10. **阅读行为树 XML**：理解 Nav2 的正常导航 + 恢复策略
11. **理解 costmap**：理解 local/global 两层 costmap 的配合
12. **MPPI 论文**：搜索 "Model Predictive Path Integral Control"，理解采样最优控制
13. **GoalApproachController**：读源码，理解三模式切换（透传/减速/直驱）
14. **Socket 编程基础**：先读 [[RM_Socket编程详解]]，理解 socket()/bind()/sendto()/recvfrom()、非阻塞 I/O、网络字节序等底层 API，再读 [[MAS_Nav_决策与UDP通信]] 中的 UDP 桥实现
15. **rm_decision**：读 `auto_fsm.py`，理解状态机+航点管理

### 第 4 阶段：系统集成（第 15-21 天）

**目标：** 能独立调试、修改、扩展系统。

15. **修改启动文件**：尝试添加一个自定义 log 节点到启动序列
16. **修改行为树**：在恢复序列中添加自定义行为
17. **调参实战**：逐个修改 MPPI、地形、LIO 参数，观察对导航的影响
18. **录制+回放**：录制真实或仿真数据，离线分析各节点性能
19. **阅读全部源码**：按数据流顺序通读全部 17 个包的源码

---

## 参考资料

| 文件 | 位置 | 说明 |
|------|------|------|
| CLAUDE.md | `mas_nav/CLAUDE.md` | 英文版开发指南 |
| SKILL.md | `mas_nav/.claude/skills/mas-nav/SKILL.md` | 技能快速参考 |
| architecture.md | `mas_nav/.claude/skills/mas-nav/references/` | 逐包架构详解 |
| navigation-config.md | `mas_nav/.claude/skills/mas-nav/references/` | YAML 配置完整参考 |
| nav_config_pointlio.yaml | `mas_nav/sentry_bringup/configs/` | 运行时配置 |
| bringup_pointlio_map.launch.py | `mas_nav/sentry_bringup/launch/` | 启动文件 |
| RM_Socket编程详解 | `RM_note/RM_Socket编程详解.md` | Socket API + UDP 通信底层基础 |
| Nav2 官方文档 | https://navigation.ros.org/ | Nav2 框架文档 |
| Point-LIO 论文 | Point-based LiDAR-Inertial Odometry (arXiv) | 论文 |
| Cyclone DDS | https://cyclonedds.io/ | DDS 中间件 |

---

> **提示：** 如果在 AI 辅助编程环境中，使用 `mas-nav` 技能可以让 AI 自动加载本项目的完整上下文。
