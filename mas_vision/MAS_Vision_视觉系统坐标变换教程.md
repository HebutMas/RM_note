# 视觉系统坐标变换教程 —— 四元数、IMU 与多坐标系变换

> 📘 **运行上下文**：坐标变换在 Armor 线程中执行，IMU 四元数通过 Serial 线程的 SPSCQueue(5000) 传入，经 Slerp 插值后用于坐标系变换，详见 [[MAS_Vision_多线程架构学习指南]]
>
> 📘 **下游应用**：本文的旋转矩阵/坐标系链方法同样用于云台重力补偿（把大地重力逐级转到云台各轴系）→ [[RM_Gimbal_重力补偿与加速度补偿详解]]

## 目录

1. [为什么需要坐标变换](#1-为什么需要坐标变换)
2. [五坐标系体系](#2-五坐标系体系)
3. [四元数在视觉系统中的使用](#3-四元数在视觉系统中的使用)
4. [IMU 数据流：从 STM32 到视觉系统](#4-imu-数据流从-stm32-到视觉系统)
5. [SLERP：四元数时间插值](#5-slerp四元数时间插值)
6. [相似变换：IMU 姿态到云台姿态](#6-相似变换imu-姿态到云台姿态)
7. [YPD 球坐标系与 Jacobian](#7-ypd-球坐标系与-jacobian)
8. [完整坐标变换链路](#8-完整坐标变换链路)
9. [嵌入式端的姿态解算（姿态解算教程整合）](#9-嵌入式端的姿态解算)
10. [调参与故障排查](#10-调参与故障排查)

---

## 1. 为什么需要坐标变换

### 1.1 问题

在 RoboMaster 哨兵系统中，视觉计算机需要回答一个看似简单的问题：

> **"目标装甲板在哪个方向？我应该让云台转多少度才能对准它？"**

但这个问题的答案涉及到多个参考系的叠加：

```
相机的镜头 → 安装在云台上 → 云台在旋转 → 机器人在地面上晃动
    ↑              ↑              ↑              ↑
 像素坐标      手眼标定偏差     IMU 姿态变化    地面不平整
```

### 1.2 单一坐标系为什么不够

假设我们只有一个"相机坐标系"：
- 相机看到装甲板在图像坐标 `(320, 240)` 处
- 但此时机器人正在向右倾斜 10°
- 如果直接根据像素位置计算云台角度，会忽略这 10° 的倾斜
- 结果：子弹偏左 10°——在 5 米距离上偏差约 **87 cm**

解决方案：建立多个坐标系，通过严格的数学变换将"相机看到的"映射到"大地固定的"。

---

## 2. 五坐标系体系

![2D 旋转变换推导](https://upload.wikimedia.org/wikipedia/commons/b/b7/Visual_Derivation_of_Equations_For_Rotation_In_2D.svg)
*▲ 2D 旋转变换的几何推导 — 坐标变换的数学基础 (图片来源: Wikipedia)*

本项目定义了五个坐标系（`auto_aim/armor_track/README.MD`）：

```
装甲板{A} ──solvePnP──→ 相机{C} ──手眼标定──→ 云台{G} ──相似变换──→ 世界{W}
                                                              ↑
                                                    IMU体{I} ─┘
```

### 2.1 各坐标系详解

| 坐标系 | 符号 | 原点 | X 轴 | Y 轴 | Z 轴 | 特点 |
|--------|------|------|------|------|------|------|
| **装甲板** {A} | Armor | 装甲板几何中心 | 右 | 下 | 垂直板面向外 | 固定在装甲板上，随目标运动 |
| **相机** {C} | Camera | 相机光心 | 图像右方向 | 图像下方向 | 沿光轴向前 | OpenCV 标准约定 |
| **云台** {G} | Gimbal | 云台旋转中心 | 指向目标 | 左 | 上 | 随云台旋转，Yaw/Pitch 零点处指向前方 |
| **IMU 体** {I} | IMU Body | IMU 芯片中心 | 芯片 X 轴 | 芯片 Y 轴 | 芯片 Z 轴 | 固定在 IMU 硬件上 |
| **世界** {W} | World | 与云台原点重合 | 前 | 左 | **上（重力反向）** | 惯性系，不随机器人运动 |

### 2.2 关键约定

- **世界坐标系 Z 轴永远指向天空** —— 重力在世界坐标系中是恒定的 `[0, 0, -g]`
- **世界坐标系与云台坐标系原点重合** —— 云台到世界的变换只有旋转，没有平移
- **装甲板坐标系原点在几何中心** —— 四个角点对称分布，简化 PnP

### 2.3 为什么世界坐标系 Z 轴向上

这是最关键的设计决策。原因：
1. **弹道解算必须在惯性系中进行** —— 重力在世界系中是常量
2. **云台的 pitch 角以水平面为基准** —— pitch=0 时枪管水平
3. **IMU 的加速度计测量重力** —— 重力在世界系中是 `[0, 0, -g]`，在机体系中的投影就是姿态信息

---

## 3. 四元数在视觉系统中的使用

> 📘 **四元数基础**（定义、运算、微分方程）详见 [[MAS_Vision_姿态解算教程]] §2-3。本节聚焦四元数在视觉端的具体用法，不重复基础内容。

### 3.1 为什么视觉端必须用四元数

视觉系统从串口收到的是四元数，不是欧拉角。原因：

| 需求 | 欧拉角能胜任？ | 四元数 |
|------|:---:|--------|
| 250Hz 插值（帧曝光时刻 vs IMU 采样时刻不同步） | ❌ 欧拉角线性插值导致角速度不均匀 | ✅ SLERP 恒角速度 |
| 无万向节锁（云台可能大角度运动） | ❌ pitch→±90° 时退化 | ✅ 4 参数覆盖所有旋转 |
| 组合多次旋转（安装偏差 + IMU 旋转 + 手眼标定） | ❌ 欧拉角乘法有奇异性 | ✅ 四元数乘法封闭 |

### 3.2 四元数 → 旋转矩阵（视觉端实现）

本项目实现在 `armor_pose.cpp` 中（通过 Eigen 的 `q.toRotationMatrix()`）：

```
R = [1-2(y²+z²)    2(xy-zw)     2(xz+yw) ]
    [2(xy+zw)      1-2(x²+z²)    2(yz-xw) ]
    [2(xz-yw)      2(yz+xw)     1-2(x²+y²)]
```

### 3.3 四元数 → 欧拉角（视觉端实现）

本项目实现在 `math_tools.cpp` 的 `eulers()` 函数中，支持任意轴序。常用的是 Z-Y-X（Yaw-Pitch-Roll）：

```cpp
// 从旋转矩阵提取 (Z-Y-X intrinsic = Yaw-Pitch-Roll)
R = q.toRotationMatrix();
yaw   = atan2(R(1,0), R(0,0));     // 绕 Z 轴
pitch = atan2(-R(2,0), sqrt(R(2,1)²+R(2,2)²)); // 绕 Y 轴
roll  = atan2(R(2,1), R(2,2));     // 绕 X 轴
```

### 3.4 四元数微分方程（与 INS 的关系）

四元数随时间的变化率与角速度的关系：

```
dq/dt = 0.5 · q ⊗ ω_quat

其中 ω_quat = [0, ωx, ωy, ωz]（角速度表示为纯四元数）

离散化（一阶近似）：
q(t+dt) = q(t) + 0.5·dt·q(t) ⊗ ω_quat
```

这是嵌入式端 INS EKF 预测步骤的基础方程。

---

## 4. IMU 数据流：从 STM32 到视觉系统

### 4.1 通信协议

STM32 通过串口以 **250 Hz** 向视觉计算机发送二进制数据包：

```
ReceivePacket (STM32 → Vision):  19 字节
┌──────┬──────┬──────────────────────────┬──────┐
│ 0xAA │ mode │ q[4] (w,x,y,z float32×4)│ 0x5A │
│  1B  │  1B  │          16B             │  1B  │
└──────┴──────┴──────────────────────────┴──────┘
```

**四元数字节序**：`w, x, y, z` 各 4 字节 float，小端序。

### 4.2 数据接收

```cpp
// user_serial.cpp 接收逻辑
if (rx_buffer 中检测到 0xAA...0x5A 帧) {
    Eigen::Quaterniond q(packet.q[0], packet.q[1], packet.q[2], packet.q[3]);
    q.normalize();  // 确保单位四元数
    
    data_queue_.try_push({q, receive_timestamp});
    has_received_data_ = true;
}
```

### 4.3 数据缓冲

四元数存入 `SPSCQueue<QuaternionWithTimestamp>(5000)`：
- 容量 5000 → 250 Hz 下可缓冲 20 秒的数据
- 大容量是因为视觉处理和 IMU 采样可能不同步
- 队列满时丢弃最旧数据（生产者非阻塞 `try_push`）

---

## 5. SLERP：四元数时间插值

### 5.1 问题：时戳不同步

```
时间线：
  IMU₁ (t=10.000)    相机曝光(t=10.003)    IMU₂ (t=10.004)
       │                    │                    │
       └────────────────────┼────────────────────┘
                            │
                    需要 t=10.003 时刻的姿态！
```

相机帧有一个曝光时刻，IMU 数据有另一个采样时刻。用 IMU 采样时刻的姿态代替曝光时刻的姿态会引入误差——特别是在云台快速旋转时。

### 5.2 滑动窗口设计

```cpp
// 双元素滑动窗口
QuaternionWithTimestamp data_ahead_;   // 较早的采样 (t < target)
QuaternionWithTimestamp data_behind_;  // 较新的采样 (t >= target)
SPSCQueue<QuaternionWithTimestamp> data_queue_(5000);
```

### 5.3 窗口推进逻辑

```cpp
UserSerial::q(steady_clock::time_point target_t)
{
    // 1. 推进窗口直到 behind 的时间超过 target
    while (data_behind_.timestamp < target_t) {
        data_ahead_ = data_behind_;
        auto* ptr = data_queue_.front();
        if (ptr == nullptr) break;  // 队列空，无法继续推进
        data_behind_ = *ptr;
        data_queue_.pop();
    }
    
    // 2. 边界情况处理
    if (data_ahead_.timestamp == epoch)  // 首次调用，只有一个样本
        return data_behind_.quaternion;
    
    // 3. 计算插值系数 k
    double dt_ab = t_behind - t_ahead;
    if (dt_ab <= 1e-9)  // 时戳相同，跳过插值
        return q_b.normalized();
    
    double k = (target_t - t_ahead) / dt_ab;
    k = clamp(k, 0.0, 1.0);  // 防止外插
    
    // 4. 最短路径修正
    if (q_a.dot(q_b) < 0)
        q_b.coeffs() = -q_b.coeffs();
    
    // 5. 执行 SLERP
    return q_a.slerp(k, q_b).normalized();
}
```

### 5.4 SLERP vs Lerp

| 方法 | 公式 | 角速度 | 适用场景 |
|------|------|--------|---------|
| Lerp | `(1-k)·q1 + k·q2`（再归一化） | **非均匀**：两端慢、中间快 | 仅小角度近似可接受 |
| SLERP | `sin((1-k)θ)/sin(θ)·q1 + sin(kθ)/sin(θ)·q2` | **恒定** | 所有旋转，本项目使用 |

**恒定角速度至关重要**：如果插值角速度不均匀，视觉输出的 yaw/pitch 会出现周期性波动，导致云台"抽搐"。

### 5.5 最短路径修正

四元数 q 和 -q 表示相同的旋转，但它们在四维球面上位于对跖点。如果直接对 q 和 -q 做 SLERP，会走球面上的"远路"（弧长 > π）：

```cpp
// 点积 < 0 → 两个四元数之间的夹角 > 90°（在四维球面上）
// → 翻转其中一个，使弧长 < π
if (q_a.dot(q_b) < 0)
    q_b = -q_b;
```

### 5.6 数据有效性检查

```cpp
bool isQuaternionValid() {
    if (!has_received_data_) return false;
    if (age > 1000ms) return false;       // 数据过期
    if (data_queue_.wasEmpty()) return false;  // 队列空
    return true;
}
```

如果数据无效（串口断连、上电初始化等），视觉系统发送零角度，云台保持不动。

---

## 6. 相似变换：IMU 姿态到云台姿态

### 6.1 问题陈述

```
已知：R_imubody2world（IMU 芯片 → 世界）  ← IMU 直接测量
已知：R_gimbal2imubody（云台 → IMU 芯片）  ← 安装偏差（固定，手动测量）

求解：R_gimbal2world（云台 → 世界）        ← 云台控制需要！
```

### 6.2 数学推导

IMU 芯片安装在云台上，两者之间有一个固定的旋转偏差（安装角度）。设：
- vᵍ：云台坐标系中的向量
- vⁱ：IMU 坐标系中的向量
- vʷ：世界坐标系中的向量

```
vⁱ = R_gimbal2imubody · vᵍ     (云台 → IMU)
vʷ = R_imubody2world · vⁱ     (IMU → 世界)

代入：
vʷ = R_imubody2world · R_gimbal2imubody · vᵍ

但我们想要的是 vʷ = R_gimbal2world · vᵍ 的形式，所以：
R_gimbal2world = R_imubody2world · R_gimbal2imubody ... ？

等等，这不对。IMU 测量的是 IMU 体 → 世界的旋转，我们需要的是云台 → 世界的旋转。
通过相似变换：
vʷ = (R_gimbal2imubody)⁻¹ · R_imubody2world · R_gimbal2imubody · vᵍ
```

### 6.3 源码实现

```cpp
// armor_pose.cpp
void ArmorPose::set_R_gimbal2world(const Eigen::Quaterniond &q)
{
    Eigen::Matrix3d R_imubody2world = q.toRotationMatrix();
    R_gimbal2world_ = R_gimbal2imubody_.transpose() 
                    * R_imubody2world 
                    * R_gimbal2imubody_;
}
```

### 6.4 直观解释

```
假设 IMU 芯片被"歪着"安装在云台上（比如旋转了 30°）。

1. R_gimbal2imubody:   先把云台的坐标轴"转歪"，对齐到 IMU 的坐标轴
2. R_imubody2world:    在 IMU 的视角下，施加 IMU 测量到的旋转
3. R_gimbal2imubodyᵀ   把结果"转正"回云台的坐标轴

等价于：把 IMU 测量到的旋转"投影"到云台的旋转轴上。
```

### 6.5 实际例子

```
R_gimbal2imubody = 绕 X 轴旋转 90°
  → IMU 芯片被侧着装（X 轴不变，Y/Z 交换）

R_imubody2world = 绕 Z 轴旋转 30°
  → 机器人整体旋转了 30°

R_gimbal2world = R_gimbal2imubodyᵀ · Rz(30°) · R_gimbal2imubody
  = Rx(-90°) · Rz(30°) · Rx(90°)
  → 30° 的旋转被"投影"为绕云台 Y 轴的旋转
```

### 6.6 R_gimbal2imubody 的标定

目前手动测量并填入 YAML：

```yaml
# config/hikcamera.yaml
handeye_calibration:
  R_gimbal2imubody: [0, 1, 0,   -1, 0, 0,   0, 0, 1]
  # 这是一个 3×3 矩阵按行展开
  # 第一行: [0, 1, 0]  → X_gimbal → Y_imu
  # 第二行: [-1, 0, 0] → Y_gimbal → -X_imu  
  # 第三行: [0, 0, 1]  → Z_gimbal → Z_imu
```

精确值可通过类似手眼标定的方法自动标定：旋转云台到不同角度，记录 IMU 读数变化，求解旋转偏差。

---

## 7. YPD 球坐标系与 Jacobian

### 7.1 为什么需要 YPD

经过坐标变换后，我们得到了装甲板在世界坐标系中的 `(x, y, z)`。但云台和 EKF 需要的不是笛卡尔坐标，而是：

```
Yaw (偏航角):   水平方向的方位角
Pitch (俯仰角): 相对于水平面的仰角
Distance:       直线距离
```

### 7.2 xyz2ypd

```cpp
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d &xyz) {
    auto x = xyz[0], y = xyz[1], z = xyz[2];
    auto yaw      = std::atan2(y, x);
    auto pitch    = std::atan2(z, std::sqrt(x*x + y*y));
    auto distance = std::sqrt(x*x + y*y + z*z);
    return {yaw, pitch, distance};
}
```

**为什么 pitch 是 `atan2(z, sqrt(x²+y²))` 而不是 `atan2(z, x)`？**

因为俯仰角描述的是目标相对于**水平面**的高度。`sqrt(x²+y²)` 是水平距离。如果目标正上方（x=0, y=0, z=5），pitch 应该是 90°（正上方），`atan2(5, 0)` 会出错而 `atan2(5, 0+)` = 90°。

### 7.3 ypd2xyz（逆变换）

```cpp
Eigen::Vector3d ypd2xyz(const Eigen::Vector3d &ypd) {
    auto yaw = ypd[0], pitch = ypd[1], distance = ypd[2];
    auto x = distance * std::cos(pitch) * std::cos(yaw);
    auto y = distance * std::cos(pitch) * std::sin(yaw);
    auto z = distance * std::sin(pitch);
    return {x, y, z};
}
```

### 7.4 Jacobian 矩阵（EKF 线性化所需）

**xyz2ypd 的 Jacobian（3×3）**：

```
∂(yaw, pitch, distance) / ∂(x, y, z) =

∂yaw/∂x = -y/(x²+y²)         ∂yaw/∂y = x/(x²+y²)          ∂yaw/∂z = 0
∂pitch/∂x = -xz/(d²·r_xy)    ∂pitch/∂y = -yz/(d²·r_xy)    ∂pitch/∂z = r_xy/d²
∂dist/∂x = x/d               ∂dist/∂y = y/d               ∂dist/∂z = z/d

其中 d = √(x²+y²+z²), r_xy = √(x²+y²)
```

这个 Jacobian 在 EKF 的更新步骤中用于将"观测到了某个 yaw/pitch/distance"的信息映射回状态空间的修正量。

### 7.5 为什么 EKF 在 YPD 空间做

| 特性 | XYZ-EKF | YPD-EKF（本项目） |
|------|---------|-------------------|
| 观测模型 | 复杂（需要相机投影模型） | **近似线性**（直接测量 yaw/pitch） |
| 测量噪声 | 各向异性（远处一个像素 = 更大距离） | **对角阵**（yaw/pitch 噪声恒定） |
| 数值稳定性 | 一般 | **好** |

---

## 8. 完整坐标变换链路

### 8.1 从像素到世界（正向）

```cpp
// armor_pose.cpp — GetArmorPose()

// Step 1: PnP → 装甲板在相机坐标系
cv::solvePnP(ARMOR_3D_POINTS, armor.points, K, D, rvec, tvec, false, SOLVEPNP_IPPE);
xyz_in_camera = tvec;  // 平移向量 = 装甲板中心在相机系中的坐标

// Step 2: 手眼标定 → 装甲板在云台坐标系
xyz_in_gimbal = R_camera2gimbal * xyz_in_camera + t_camera2gimbal;

// Step 3: 相似变换 → 装甲板在世界坐标系
xyz_in_world = R_gimbal2world * xyz_in_gimbal;  // 只有旋转，云台=世界原点

// Step 4: 旋转姿态提取
R_armor2camera = Rodrigues(rvec);
R_armor2gimbal = R_camera2gimbal * R_armor2camera;
R_armor2world  = R_gimbal2world * R_armor2gimbal;

// Step 5: 球坐标转换
ypr_in_world = eulers(R_armor2world, 2, 1, 0);  // Yaw-Pitch-Roll
ypd_in_world = xyz2ypd(xyz_in_world);            // Yaw-Pitch-Distance
```

### 8.2 数据流总图

```
┌──────────┐  SPI @ 1000Hz   ┌──────────┐  UART @ 250Hz   ┌──────────────┐
│ BMI088   │ ───────────────→ │  STM32   │ ───────────────→ │ Vision (C++) │
│ IMU      │   原始传感器数据   │ INS EKF  │   q[w,x,y,z]    │              │
└──────────┘                  └──────────┘                  └──────────────┘
                                                                │
                          ┌─────────────────────────────────────┘
                          │
                          ▼
              ┌──────────────────────┐
              │ serial.q(frame_time) │  ← SLERP 插值
              │ → q_at_exposure      │
              └──────────────────────┘
                          │
                          ▼
              ┌──────────────────────────────────┐
              │ R_gimbal2world =                 │
              │   R_gimbal2imubodyᵀ              │
              │   · R_imubody2world              │
              │   · R_gimbal2imubody             │
              └──────────────────────────────────┘
                          │
          ┌───────────────┼───────────────┐
          ▼               ▼               ▼
    xyz_in_world    ypr_in_world    ypd_in_world
          │               │               │
          ▼               ▼               ▼
      EKF 观测        调试显示        弹道解算输入
```

---

## 9. 嵌入式端的姿态解算

> 本节整合了 `MAS_Vision_姿态解算教程.md` 的核心内容，聚焦于视觉系统开发者需要了解的嵌入式 INS 知识。

### 9.1 STM32 端的 INS 系统架构

```
BMI088 (SPI)
  ├── 加速度计 @ 800 Hz → acc[3] m/s²
  └── 陀螺仪   @ 1000 Hz → gyro[3] dps
         │
         ▼
    IMU_Param_Correction()  ← 安装误差校正矩阵
         │
         ▼
    QuaternionEKF (6 状态)
    ├── 状态: [q0,q1,q2,q3, bias_x, bias_y]
    ├── 预测: 陀螺仪积分 (1000 Hz)
    ├── 更新: 加速度计观测重力方向
    └── 卡方检验: 动态拒绝异常加速度
         │
         ▼
    Ins_t 结构体 (全局共享)
    ├── q[4] 四元数
    ├── euler_rad[3] 欧拉角
    ├── YawTotalAngle_rad 解缠绕 yaw
    └── MotionAccel 运动加速度
```

### 9.2 EKF 核心参数

| 参数 | 值 | 含义 |
|------|-----|------|
| 状态维度 | 6 | [q0,q1,q2,q3, bias_x, bias_y] |
| 过程噪声 Q1 | 10.0 | 四元数过程噪声（大 = 信任陀螺仪积分少） |
| 过程噪声 Q2 | 0.001 | 陀螺仪零偏随机游走 |
| 量测噪声 R | 1,000,000 | 加速度计量测噪声（极大 = 极不信任加速度计） |
| 卡方阈值 | 1e-8 | 新息门控阈值 |
| 运行频率 | ~1000 Hz | `tx_thread_sleep(1)` |

### 9.3 关键参数 R 的设计逻辑

**R = 1,000,000** 意味着 EKF 极不信任加速度计。这是因为：
- 在运动中 `acc_measured = gravity + motion_accel + vibration`
- motion_accel 和 vibration 对姿态估计而言是"噪声"
- R 极大 → Kalman 增益极小 → 量测更新几乎不改变状态
- **本质上是"大多数时候只靠陀螺仪积分，仅在确认真静止时才用加速度计修正"**

### 9.4 自适应卡方检验

```
if (chi² < 0.5 * threshold):
    converged = true → 正常更新
elif (chi² > threshold AND converged AND stable):
    error_count++
    if (error_count > 50):  → 滤波器可能发散，强制重置
    else: → 跳过本次量测更新
else:
    AdaptiveGainScale = (threshold - chi²) / (0.9 * threshold)
    K_scaled = K * AdaptiveGainScale  → 软切换
```

### 9.5 静止检测

```c
stable = (|gyro|      < 0.3 rad/s)    // 角速度小
      && (| |acc|-g | < 0.2 m/s²)     // 加速度幅值 ≈ 重力
      && (horiz_acc   < 0.15 m/s²)    // 水平加速度小
```

静止时 EKF 可以更积极地使用加速度计修正 roll/pitch。

### 9.6 Yaw 漂移

**z 轴陀螺仪零偏被强制为 0**——因为没有磁力计，yaw 绝对朝向不可观测。这意味着：
- Roll/Pitch：有重力矢量作为绝对参考，长期稳定
- Yaw：完全依赖陀螺仪积分，**会持续漂移**（典型 ~1°/min）
- 云台 yaw 控制实际上不需要绝对朝向，只需相对跟踪目标

### 9.7 视觉系统需要关注的 INS 特性

1. **q[0..3] 是 w,x,y,z 顺序** —— 与 Eigen 构造函数 `Quaterniond(w,x,y,z)` 一致
2. **四元数表示 IMU 体 → 世界** —— 视觉端需通过相似变换转为云台 → 世界
3. **INS 更新率 1000 Hz >> 视觉帧率 60 Hz** —— 说明 SLERP 插值是必要的
4. **加速度计修正被大幅抑制** —— yaw 漂移是无法通过 INS 自身校正的

---

## 10. 调参与故障排查

### 10.1 常见问题

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| 云台 yaw 缓慢漂移 | INS yaw 无绝对参考 | 正常现象，视觉跟踪可补偿；如需绝对朝向需添加磁力计 |
| 视觉输出 yaw/pitch 抖动 | SLERP 未生效，直接用最近的四元数 | 检查 `serial.q(timestamp)` 是否正确传入了帧时间戳 |
| pitch 估计有常值偏差 | `R_gimbal2imubody` 不准确 | 重新标定 IMU 安装角度 |
| 快速旋转时视觉输出滞后 | SLERP 窗口落后于真实时间 | 检查串口数据是否及时到达，增大 `data_queue_` 容量 |
| 静止时输出有小幅波动 | 加速度计噪声 + EKF 修正 | 正常（~0.5°），可通过增大 INS 端的 R 值减少 |

### 10.2 验证坐标变换正确性的方法

```cpp
// 测试：在已知姿态下验证变换
// 1. 云台 pitch 上仰 30°，读取 IMU 四元数
// 2. 手动计算：pitch 应该 ≈ 30°
// 3. 与实际 IMU 输出对比

// 测试相似变换：
// 1. 设置 R_gimbal2imubody = I（假设无安装偏差）
// 2. R_gimbal2world 应该 = q.toRotationMatrix()
// 3. 验证 R_gimbal2world 提取的 yaw/pitch/roll 与 IMU 直接输出一致
```

### 10.3 调试日志

```cpp
// 打印四元数（验证归一化）
LOG_D("q: [%.4f, %.4f, %.4f, %.4f], norm=%.4f", 
      q.w(), q.x(), q.y(), q.z(), q.norm());

// 打印 SLERP 插值系数
LOG_D("SLERP: k=%.3f, q_ahead(t=%.3f), q_behind(t=%.3f)", 
      k, t_ahead, t_behind);

// 打印坐标变换结果
LOG_D("xyz_in_world: [%.3f, %.3f, %.3f], ypd: [%.2f°, %.2f°, %.3fm]",
      x, y, z, yaw*180/PI, pitch*180/PI, dist);
```
