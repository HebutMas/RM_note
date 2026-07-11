# 系统集成、标定与调试教程

> 📘 **专题深入**：多线程架构、SPSCQueue 无锁队列原理、g_shutdown 退出机制，详见 [[MAS_Vision_多线程架构学习指南]]

## 目录

1. [相机内参标定](#1-相机内参标定)
2. [手眼标定](#2-手眼标定)
3. [IMU 安装角标定](#3-imu-安装角标定)
4. [ROS2 UDP 通信](#4-ros2-udp-通信)
5. [可视化调试系统](#5-可视化调试系统)
6. [配置管理](#6-配置管理)
7. [性能调优](#7-性能调优)
8. [常见故障排查](#8-常见故障排查)

---

## 1. 相机内参标定

### 1.1 相机模型

**针孔相机模型**将 3D 世界点映射到 2D 图像点：

```
s · [u, v, 1]ᵀ = K · [R|t] · [X, Y, Z, 1]ᵀ

内参矩阵 K:
    [fx   0   cx]
K = [ 0  fy   cy]
    [ 0   0    1]

fx, fy: 焦距（像素单位）—— 镜头将物理距离映射到像素数
cx, cy: 光心（主点）—— 光轴与成像平面的交点
```

**畸变模型**（径向 + 切向）：

```
x_distorted = x·(1 + k₁r² + k₂r⁴ + k₃r⁶) + 2p₁xy + p₂(r² + 2x²)
y_distorted = y·(1 + k₁r² + k₂r⁴ + k₃r⁶) + p₁(r² + 2y²) + 2p₂xy

其中 r² = x² + y²
D = [k₁, k₂, p₁, p₂, k₃]
```

### 1.2 标定原理

1. 拍摄多张不同角度、不同位置的棋盘格
2. `cv::findCirclesGrid()` 提取圆心的 2D 像素坐标（本项目使用圆形标定板）
3. 已知棋盘格的物理尺寸 → 3D 世界坐标
4. `cv::calibrateCamera()` 求解 K 和 D

### 1.3 操作步骤

```bash
./base calibrate_camera
```

1. 准备圆形标定板（`pattern_cols × pattern_rows` 个圆点）
2. 将标定板放在相机前方
3. 缓慢移动标定板，覆盖图像的所有区域（四角、中心、不同距离）
4. 按 **S** 键保存当前帧（系统自动检测圆心）
5. 至少采集 **10 帧**不同姿态的标定板
6. 按 **ESC** 完成标定

### 1.4 标定质量标准

```
重投影误差 < 0.5 像素 → 优秀
重投影误差 < 1.0 像素 → 可用
重投影误差 > 1.5 像素 → 重新标定
```

**提高标定质量**：
- 标定板覆盖图像四角（畸变最大的地方）
- 包含近/中/远距离的标定板
- 标定板与相机平面成角度（不要总是平行）
- 避免运动模糊（标定板静止时才按 S）

### 1.5 标定结果

写入 `config/hikcamera.yaml`：

```yaml
calibration:
  camera_matrix:
    - [fx,  0, cx]
    - [ 0, fy, cy]
    - [ 0,  0,  1]
  distort_coeffs: [k1, k2, p1, p2, k3]
  reprojection_error: 0.2468
```

### 1.6 参数含义速查

| 参数 | 典型值（海康相机 1440×1080） | 含义 |
|------|--------------------------|------|
| fx | ~2000 | 水平焦距（像素） |
| fy | ~2000 | 垂直焦距（通常 fx ≈ fy） |
| cx | ~720 | 光心水平位置（≈ 宽度/2） |
| cy | ~540 | 光心垂直位置（≈ 高度/2） |
| k1 | ~-0.3 | 径向畸变（正=枕形，负=桶形） |
| k2 | ~0.1 | 高阶径向畸变 |

---

## 2. 手眼标定

### 2.1 问题陈述

相机安装在云台上。当云台旋转时，相机跟着旋转。我们需要知道**相机在云台坐标系中的精确位置和朝向**。

```
手眼标定求解：
  X: 相机 → 云台 的变换 (R_camera2gimbal, t_camera2gimbal)

已知:
  A: 云台的运动（从 IMU 四元数推算）
  B: 相机的运动（从拍摄标定板推算）
```

### 2.2 核心方程：AX = XB

```
A · X = X · B

A: 云台在两次采样之间的旋转（从 IMU 四元数获取）
B: 相机在两次采样之间的旋转（从标定板的 PnP 姿态变化获取）
X: 相机在云台坐标系中的位姿（未知，待求解）
```

**直观理解**：
> 你移动你的头（云台），同时用眼睛（相机）观察一个固定的物体。
> 头部运动（A）与眼睛中物体的运动（B）之间的差异，
> 揭示了眼睛在头部中的位置（X）。

### 2.3 操作步骤

```bash
./base calibrate_handeye
```

1. 在相机前方固定放置标定板（AprilTag 或棋盘格，或通用的圆形标定板）
2. **旋转云台到多个不同角度**（yaw 和 pitch 各 3–5 个角度，总共 6–15 个姿态）
3. 每个角度稳定后按 **S** 保存
   - 系统自动记录：标定板在相机中的位姿 + IMU 四元数
4. 按 **C** 清除所有数据重新开始
5. 按 **Q** 计算并写入结果

### 2.4 标定结果

写入 `config/hikcamera.yaml`：

```yaml
handeye_calibration:
  R_camera2gimbal: [r11, r12, r13, r21, r22, r23, r31, r32, r33]
  t_camera2gimbal: [tx, ty, tz]  # 米
```

### 2.5 物理含义

- `t_camera2gimbal`: 相机光心在云台坐标系中的位置（米）
  - tx: 相机在云台前方（水平方向）
  - ty: 相机在云台左侧（水平方向）
  - tz: 相机在云台上方（垂直方向）

### 2.6 提高标定质量

- **云台运动要足够大**：每次旋转至少 15°，太小则 A 矩阵近似单位阵，AX=XB 退化
- **标定板要足够大**：在图像中至少占 1/4 面积，保证 PnP 精度
- **覆盖多个轴**：不仅 yaw 要动，pitch 也要动（对 tz 方向敏感）
- **数据量**：至少 6 组，推荐 10–15 组
- **光照稳定**：标定期间不要改变光照条件

---

## 3. IMU 安装角标定

### 3.1 R_gimbal2imubody 的含义

这个参数描述 IMU 芯片相对于云台坐标系的安装姿态。它是一个 3×3 的旋转矩阵。

```yaml
# config/hikcamera.yaml
handeye_calibration:
  R_gimbal2imubody: [0, 1, 0,  -1, 0, 0,  0, 0, 1]
  # 展开为 3×3 矩阵（行优先）：
  # [ 0,  1,  0]   第一行 = X_gimbal 在 IMU 系中的方向
  # [-1,  0,  0]   第二行 = Y_gimbal 在 IMU 系中的方向
  # [ 0,  0,  1]   第三行 = Z_gimbal 在 IMU 系中的方向
```

### 3.2 手动标定方法

1. 将云台 pitch 归零，使枪管水平
2. 读取 IMU 的 roll 和 pitch 角度（从日志/调试界面）
3. 如果 roll ≠ 0 而云台实际是水平的 → IMU 绕 X 轴有安装偏差
4. 同理，pitch 偏差 → IMU 绕 Y 轴有安装偏差
5. 根据偏差角构建旋转矩阵

### 3.3 自动标定方法（理论，当前版本未实现）

1. 将云台分别绕 yaw 和 pitch 旋转已知角度
2. 记录 IMU 四元数的变化
3. IMU 测量到的"旋转轴"与云台实际的旋转轴之间的差异 → R_gimbal2imubody

### 3.4 验证方法

```cpp
// 验证：当云台 pitch = +30° 时
// 1. 读取 IMU 四元数
// 2. R_gimbal2world = R_gimbal2imubodyᵀ * R_imubody2world * R_gimbal2imubody
// 3. 从 R_gimbal2world 提取 pitch 角
// 4. 应该 ≈ 30°
```

---

## 4. ROS2 UDP 通信

### 4.1 架构

```
Vision系统 (Jetson/NUC)                Docker (ROS2)
┌──────────────┐     UDP:12344      ┌──────────────┐
│  Ros2_comm   │ ─────────────────→ │  ros2 node   │
│              │   裁判系统数据       │              │
│              │                    │              │
│              │     UDP:12345      │              │
│              │ ←───────────────── │              │
│              │   导航指令          │              │
└──────────────┘                    └──────────────┘
```

### 4.2 协议格式

**Send（Vision → ROS2）**：裁判系统数据转发

```cpp
struct ROS2_SEND_PACKET {
    uint16_t projectile_allowance_17mm;  // 17mm 弹丸剩余
    uint8_t  power_management_shooter_output; // 射击功率管理
    uint16_t current_hp;                 // 当前血量
    uint16_t outpost_HP;                 // 前哨站血量
    uint16_t base_HP;                    // 基地血量
    uint8_t  game_progess;               // 比赛阶段
};

// 帧格式: [0xAA, data_len, ROS2_SEND_PACKET, 0x5A]
```

**Recv（ROS2 → Vision）**：导航指令

```cpp
struct ROS2_RECV_PACKET {
    float   vx;          // 导航速度 X
    float   vy;          // 导航速度 Y
    uint8_t nav_state;   // 导航状态
};

// 帧格式: [0xBB, data_len, ROS2_RECV_PACKET, 0x5B]
```

### 4.3 配置

```yaml
# config/ros2.yaml
ros2:
  enabled: true
  server_ip: "127.0.0.1"
  server_port: 8888
  local_port: 8889
```

### 4.4 为什么用 UDP

| 特性 | TCP | UDP（本项目） |
|------|-----|-------------|
| 可靠性 | 保证送达 | 可能丢包 |
| 延迟 | 不确定（重传） | **确定（无重传）** |
| 实时性 | 差 | **好** |
| 适用场景 | 文件传输 | **实时控制数据** |

裁判系统数据和导航指令每秒更新多次，丢失一帧影响极小。等待 TCP 重传的延迟代价远大于偶尔丢一帧数据的代价。

### 4.5 线程实现

```
ROS2 Thread (100 Hz):
  ├── 初始化: 创建非阻塞 UDP socket
  ├── 每 10ms:
  │   ├── 读取 UserSerial 的最新裁判数据
  │   ├── sendto() 发送到 ROS2 Docker
  │   ├── recvfrom() 非阻塞接收导航指令
  │   └── 收到导航指令 → 转发到 UserSerial.sendNav()
  └── 连接断开: 每 2 秒重试初始化
```

---

## 5. 可视化调试系统

### 5.1 SDL2 显示架构

```cpp
// 每个调试窗口有独立的 SPSCQueue<DisplayTask>(2)
// 窗口名称 → SPSCQueue 的映射

// 提交显示任务（在任何线程中调用，无锁）
display_add("detect", frame_with_overlay);
display_add("binary", binary_image);
display_add("track", tracked_armors_image);

// Display 线程独立渲染所有窗口
```

### 5.2 DisplayTask 数据

```cpp
struct DisplayTask {
    cv::Mat image;                     // 源图像
    vector<DisplayText>  texts;        // 文字叠加
    vector<DisplayPoint> points;       // 2D 点
    vector<DisplayLine>  lines;        // 线段
};
```

### 5.3 渲染特性

- **30 FPS 限制**：`display_add()` 拒绝间隔 < 33ms 的更新（降低 CPU 占用）
- **文字缓存**：按字体大小/内容/颜色缓存纹理（最多 50 个），避免重复渲染
- **线段加粗**：通过多条平行线模拟粗线效果

### 5.4 调试 Plotter（UDP 数据绘图）

```yaml
# 启用 plotter
plotter_enable: true
```

Plotter 通过 UDP 将实时数据（yaw, pitch, distance, nis 等）发送到 9870 端口，由独立的 Python 接收端绘制实时曲线。

---

## 6. 配置管理

### 6.1 配置文件全景

| 文件 | 内容 |
|------|------|
| `config/hikcamera.yaml` | 海康相机参数、内参、畸变、手眼标定结果 |
| `config/auto_aim.yaml` | 检测/跟踪/射击的全部参数 |
| `config/serial_config.yaml` | 串口配置（端口、波特率等） |
| `config/usbcamera_config.yaml` | USB 相机配置 |
| `config/ros2.yaml` | ROS2 UDP 通信配置 |

### 6.2 auto_aim.yaml 关键参数

```yaml
auto_aim:
  armor_detector:
    binary_thres: 90           # 二值化阈值
    detect_color: "RED"        # 目标颜色
    lights_params: { ... }     # 灯条过滤参数
    armors_params: { ... }     # 装甲板过滤参数
    number_params:
      model_path: "model/lenet.onnx"
      classifier_threshold: 0.7
      ignore_classes: ["negative"]
    
  armor_track:
    min_detect_count: 5        # 确认跟踪的连续帧数
    max_temp_lost_count: 3     # 临时丢失最大值
    
  armor_shoot:
    bullet_speed: 25.0         # 子弹初速 (m/s)
    yaw_offset: 0.0            # yaw 偏差修正 (度)
    pitch_offset: 0.0          # pitch 偏差修正 (度)
    spinning_threshold_low: 2.0    # TRACK/COMING 分界 (rad/s)
    spinning_threshold_high: 10.0  # COMING/HERO_CENTER 分界
    yaw_tolerance_near: 1.0    # 近距离 yaw 容差 (度)
    yaw_tolerance_far: 2.0
    pitch_tolerance_near: 1.0
    pitch_tolerance_far: 2.0
    fire_delay_time: 0.02      # 发弹延迟 (s)
```

### 6.3 配置热加载建议

当前版本不支持热加载（修改 YAML 后需重启）。建议开发时：
1. 在 YAML 中修改参数
2. 重启 `./base`
3. 观察效果

**可考虑改进**：添加 SIGHUP 信号处理，收到信号后重新加载 YAML。

---

## 7. 性能调优

> 📘 **原理参考**：跳帧策略、SPSCQueue 内存占用、缓存行对齐等底层原理详见 [[MAS_Vision_多线程架构学习指南]]

### 7.1 系统启动检查清单

```
□ 相机: 60 FPS 稳定, 曝光时间合适
□ 串口: 250Hz 发送, IMU 四元数稳定接收
□ 检测: < 5ms/帧, 灯条和装甲板正确检出
□ 跟踪: EKF 收敛, NIS < 阈值
□ 射击: 弹道解算正常, 三种模式正确切换
□ 显示: SDL2 窗口流畅, 不卡顿
□ CPU: 所有线程 < 80% 总占用
```

### 7.2 各阶段的健康指标

| 指标 | 健康范围 | 需要关注 | 严重 |
|------|---------|---------|------|
| detect time | < 5ms | 5–10ms | > 10ms |
| track time | < 2ms | 2–5ms | > 5ms |
| shoot time | < 1ms | 1–3ms | > 3ms |
| NIS | < 0.5 | 0.5–0.7 | > 0.711（95%） |
| nis_fail_rate | < 20% | 20–40% | > 40% |
| IMU data age | < 50ms | 50–200ms | > 200ms |
| serial interval | 3.5–4.5ms | 4.5–5.5ms | > 6ms |

### 7.3 CPU 优化建议

1. **降低相机分辨率**：1440×1080 → 1280×720 → 640×480
   - 分辨率减半 → 检测速度约 4x
   - 代价：远距离小装甲板可能检不到

2. **降低帧率**：60 FPS → 30 FPS
   - 在 `camera_thread` 中主动丢弃一半帧
   - 代价：跟踪延迟增加 16ms

3. **二值化阈值调优**：`binary_thres` 从 90 → 120
   - 减少候选灯条数量 → 配对更快
   - 代价：暗环境可能漏检

### 7.4 内存监控

```cpp
// 关键内存消费者
SPSCQueue<CameraFrame>(2):  1440×1080×3B ×2 = ~9.3 MB
SPSCQueue<Quat>(5000):      24B × 5000     = ~120 KB
cv::Mat 预分配:              ~5 MB
总计:                        ~15 MB

// 在 Jetson Orin (8GB) 上绰绰有余
```

---

## 8. 常见故障排查

### 8.1 相机相关

| 问题 | 诊断 | 解决 |
|------|------|------|
| 相机打不开 | 检查 USB 连接 | `lsusb` 确认设备；检查 MVS SDK 版本 |
| 图像太暗 | 查看 `exposuretime` | 增大曝光时间（4000→8000） |
| 图像过曝 | LED 灯条成大块白斑 | 减小曝光时间，降低 gain |
| 丢帧 | 检查 `MV_SPSCQueue.getFrame()` 超时 | 减小分辨率或帧率；检查 USB 带宽 |

### 8.2 串口相关

| 问题 | 诊断 | 解决 |
|------|------|------|
| 串口打不开 | 检查设备路径 | `ls /dev/mascdc`；确认 STM32 已上电 |
| 数据帧错误 | 大量 CRC/帧尾不匹配 | 检查波特率（115200）；检查接线 |
| IMU 数据过期 | `isQuaternionValid()` 返回 false | 检查 STM32 是否在发送；检查串口 RX 线 |
| 串口发送卡住 | 看 send 频率 | 降低 send 频率（250→100Hz）或增大 buffer |

### 8.3 检测相关

| 问题 | 诊断 | 解决 |
|------|------|------|
| 灯条检不到 | 查看 binary_ 图像 | 降低 `binary_thres`；增大曝光 |
| 灯条太多（噪声） | 查看 findLights 数量 | 提高 `binary_thres`；降低 `max_lightbar_area` |
| 装甲板配对错误 | 查看装甲板的编号标签 | 调节 `max_side_ratio`、`max_rectangular_error` |
| 数字总是识别为 negative | 查看 number_roi | 检查 ONNX 模型路径；增大 `classifier_threshold` |
| 角点位置不准 | 对比原始和优化后的角点 | 检查 LightCornerCorrector 的梯度搜索参数 |

### 8.4 跟踪相关

| 问题 | 诊断 | 解决 |
|------|------|------|
| EKF 不收敛 | 查看 NIS 时间曲线 | 增大 P0（初始不确定度更大） |
| NIS 失败率持续 > 40% | 查看 `recent_nis_failures` | 检查坐标变换链；检查 PnP 输出 |
| 跟踪经常进入 TEMP_LOST | 查看连续丢失帧数 | 增大 `max_temp_lost_count` |
| 从 TEMP_LOST 恢复慢 | 查看重新检测延迟 | 减小 `min_detect_count` |

### 8.5 射击相关

| 问题 | 诊断 | 解决 |
|------|------|------|
| 弹道无解（unsolvable） | 检查 distance 是否过大 | 确认目标在有效射程内 |
| 射击角度跳动 | 查看 target_yaw 的时间曲线 | 检查 yaw 不连续处理；增大容差 |
| 模式频繁切换 | 查看 omega 的时间曲线 | 增大模式切换的迟滞 |
| 不开火 | 检查 yaw/pitch 偏差 | 增大容差；检查 `fire_advice` 逻辑 |

---

## 附录：调试命令速查

```bash
# 启动正常模式
./base

# 相机标定
./base calibrate_camera

# 手眼标定
./base calibrate_handeye

# 查看日志
tail -f logs/mas_vision.log

# 查看实时数据（Plotter）
python3 tools/plotter.py --port 9870

# 检查串口设备
ls -la /dev/mascdc
stty -F /dev/mascdc 115200

# 检查相机设备
lsusb | grep Hikvision
```
