# MAS Vision 多线程架构 — 学习指南

> **目标**：深入理解 mas_vision 的 6 线程模型、无锁通信机制、SPSCQueue 内部原理，以及线程间的数据流动路径。
> **前置**：建议先阅读 [[MAS_Vision_学习指南]] 第1-3周内容，了解项目整体架构。
> **代码位置**：`threads/` (5个线程模块) + `apps/main.cpp` (线程启动) + `rm_utils/SPSCQueue/` (无锁队列)

---

## 目录

## 相关笔记

- [[MAS_Vision_学习指南]] — 八周学习主路线，多线程在第7周
- [[MAS_Vision_弹道解算与射击控制教程]] — 弹道模型 + 三种瞄准模式 + 射击决策
- [[MAS_Vision_系统集成标定与调试教程]] — 线程实现细节、调试方法、性能指标
- [[MAS_Vision_位姿解算与目标跟踪教程]] — Armor 线程中运行的跟踪算法
- [[MAS_Vision_装甲板检测教程]] — Armor 线程中运行的检测算法
- [[MAS_Vision_视觉系统坐标变换教程]] — Armor 线程中的坐标变换链路
- [[MAS_Vision_姿态解算教程]] — IMU 四元数来源（Serial 线程接收）

---

- [1. 概述：为什么需要 6 个线程](#1-概述为什么需要-6-个线程)
- [2. 项目结构](#2-项目结构)
- [3. 核心数据结构](#3-核心数据结构)
  - [3.1 CameraFrame — 跨线程的图像载体](#31-cameraframe--跨线程的图像载体)
  - [3.2 SPSCQueue — 无锁环形缓冲区](#32-spscqueue--无锁环形缓冲区)
  - [3.3 QuaternionWithTimestamp — IMU 数据时间戳绑定](#33-quaternionwithtimestamp--imu-数据时间戳绑定)
  - [3.4 串口协议包（SendPacket / ReceivePacket）](#34-串口协议包sendpacket--receivepacket)
- [4. 线程详解](#4-线程详解)
  - [4.1 Camera 线程 — 海康相机采集](#41-camera-线程--海康相机采集)
  - [4.2 USBCamera 线程 — USB 相机采集](#42-usbcamera-线程--usb-相机采集)
  - [4.3 Serial 线程 — 250Hz 串口收发](#43-serial-线程--250hz-串口收发)
  - [4.4 Armor 线程 — 视觉主流水线](#44-armor-线程--视觉主流水线)
  - [4.5 ROS2 线程 — UDP 桥接 Docker](#45-ros2-线程--udp-桥接-docker)
  - [4.6 Display — 主线程中的 SDL2 显示](#46-display--主线程中的-sdl2-显示)
- [5. 线程间数据流全景](#5-线程间数据流全景)
- [6. SPSCQueue 深度解析](#6-spscqueue-深度解析)
  - [6.1 为什么 capacity 要 +1](#61-为什么-capacity-要-1)
  - [6.2 Cache Line 对齐与 False Sharing 防护](#62-cache-line-对齐与-false-sharing-防护)
  - [6.3 内存序（Memory Order）分析](#63-内存序memory-order分析)
  - [6.4 为什么相机队列用 capacity=2](#64-为什么相机队列用-capacity2)
- [7. 线程同步机制](#7-线程同步机制)
  - [7.1 g_shutdown — 无锁退出信号](#71-g_shutdown--无锁退出信号)
  - [7.2 UserSerial 单例 — 串口共享访问](#72-userserial-单例--串口共享访问)
  - [7.3 为什么整个系统没有互斥锁](#73-为什么整个系统没有互斥锁)
- [8. 关键设计模式总结](#8-关键设计模式总结)

---

## 1. 概述：为什么需要 6 个线程

本项目是 **实时视觉 + 控制** 系统，每帧图像需在十几毫秒内完成检测→位姿→跟踪→弹道→发送的全流程。如果把所有任务放在一个线程里：

```
❌ 单线程模式：
   getImage(阻塞等图) → detect(10ms) → track(2ms) → shoot(1ms) → send(4ms)
   → 返回时已过去 ~17ms，下一帧可能已经丢失
   → 串口收发与图像处理互相阻塞，无法保证 250Hz 的发送频率
```

多线程设计的核心思想是 **解耦**：每个硬件 I/O 操作独占一个线程，计算密集型任务独占一个线程，互不阻塞。

```
✅ 多线程模式：
   Camera Thread:     [等图] → [push] → [等图] → [push] → ...
   Serial Thread:     [收] [收] [收] [发] [收] [收] [收] [发] ... (250Hz 定时发送)
   Armor Thread:      [pop] → [detect → track → shoot → sendVision] → [pop] → ...
   ROS2 Thread:       [收裁判数据] → [发UDP] → [收UDP] → [发nav] → ...
```

| 线程 | 频率特征 | 瓶颈类型 |
|------|----------|----------|
| Camera | 由相机帧率决定（~180fps） | I/O 密集型 |
| USBCamera | 由 USB 帧率决定（~30fps） | I/O 密集型 |
| Serial | 发送 250Hz 精确周期，接收连续轮询 | I/O 密集型 |
| Armor | 无固定频率，有帧就处理 | 计算密集型 |
| ROS2 | ~100Hz（10ms 间隔） | 网络 I/O |
| Display | 有帧就刷新 | 渲染 I/O |

---

## 2. 项目结构

```
threads/
├── armor_thread/
│   ├── armor_thread.hpp          # 函数声明：接收两个 SPSCQueue
│   └── armor_thread.cpp          # 主视觉流水线：detect → track → shoot
├── camera_thread/
│   ├── camera_thread.hpp          # 函数声明：接收 config_path + buffer
│   └── camera_thread.cpp          # HikCamera 阻塞采集 → try_push 到队列
├── serial_thread/
│   ├── serial_thread.hpp          # 函数声明：无参数 (单例获取)
│   └── serial_thread.cpp          # sendPacketToSerial (250Hz) + receiveSerial
├── ros2_thread/
│   ├── ros2_thread.hpp            # 函数声明：接收 UserSerial 引用
│   └── ros2_thread.cpp            # UDP 收发：裁判数据上报 + 导航指令下发
└── usbcamera_thread/
    ├── usbcamera_thread.hpp       # 函数声明：接收 buffer
    └── usbcamera_thread.cpp       # USB 相机采集 → try_push 到队列
```

所有线程函数都声明在 `namespace threads` 中，且都引用同一个 `extern std::atomic<bool> g_shutdown`（定义在 `apps/main.cpp`）。

---

## 3. 核心数据结构

### 3.1 CameraFrame — 跨线程的图像载体

```cpp
// hardware/common_def.hpp
struct CameraFrame
{
    cv::Mat                               frame;     // OpenCV 图像矩阵
    std::chrono::steady_clock::time_point timestamp; // 采集时刻的精确时间戳
};
```

**为什么需要 timestamp？** 相机的曝光时刻与 Armor 线程处理时刻之间有延迟（几毫秒到十几毫秒）。IMU 四元数在持续更新——必须在曝光时刻的 IMU 姿态下做坐标变换，而不是处理时刻的姿态。因此 timestamp 关联了图像与 IMU 数据的精确时间对齐。

### 3.2 SPSCQueue — 无锁环形缓冲区

```cpp
// rm_utils/SPSCQueue/SPSCQueue.h (Erik Rigtorp)
template <typename T>
class SPSCQueue {
    // 核心成员：
    size_t capacity_;          // 实际容量 = 用户请求 + 1（一个空位用于判满）
    T*     slots_;             // 环形缓冲区数组

    alignas(64) std::atomic<size_t> writeIdx_;   // 生产者写指针 (cache line 对齐)
    alignas(64) size_t readIdxCache_;             // 生产者缓存的读指针
    alignas(64) std::atomic<size_t> readIdx_;     // 消费者读指针 (cache line 对齐)
    alignas(64) size_t writeIdxCache_;            // 消费者缓存的写指针

    // 关键接口：
    bool try_push(T&& v);      // 非阻塞写入（生产者调用）
    T*   front();              // 非阻塞读取（消费者调用，返回指针或 nullptr）
    void pop();                // 消费完成后移除元素
};
```

**SPSC 的含义**：Single Producer, Single Consumer（单生产者单消费者）。这是无锁队列能实现的最简情形——不需要 CAS 循环，只需要两个原子变量的 store/load。

深度解析见 [第6节](#6-spscqueue-深度解析)。

### 3.3 QuaternionWithTimestamp — IMU 数据时间戳绑定

```cpp
// hardware/serial/serial_types.hpp
struct QuaternionWithTimestamp
{
    Eigen::Quaterniond                    quaternion; // IMU 姿态四元数 (wxyz)
    std::chrono::steady_clock::time_point timestamp;  // 串口收到该数据的时间点
};
```

Serial 线程每收到一个有效的 IMU 数据包，就将四元数 + 时间戳打包推入 `data_queue_`（容量 5000）。Armor 线程通过 `UserSerial::q(frame_timestamp)` 从队列中取出时间轴两侧的两帧四元数，进行球面线性插值（Slerp）。

**为什么容量是 5000？** 串口以 ~250Hz 发送四元数，即每 4ms 一帧。5000 × 4ms = 20 秒的历史窗口。即使 Armor 线程短暂卡顿，也有足够的历史数据用于插值。

### 3.4 串口协议包（SendPacket / ReceivePacket）

```cpp
// 接收包（STM32 → Vision）
struct ReceivePacket {
    uint8_t header;       // 0xAA
    uint8_t mode;         // VisionMode (0=RED, 1=BLUE, ...)
    float   q[4];         // 四元数 wxyz
    RefereePacket referee_data; // 裁判系统数据 (SENTRY 宏控制)
    uint8_t tail;         // 0x5A
};

// 发送包（Vision → STM32）
struct SendPacket {
    uint8_t header;       // 0xBB
    uint8_t found;        // 是否检测到目标
    uint8_t fire_advice;  // 开火建议
    float   target_yaw;   // 目标 yaw 角 (rad)
    float   target_pitch; // 目标 pitch 角 (rad)
    float   vx;           // X 方向速度 (ROS2 导航)
    float   vy;           // Y 方向速度 (ROS2 导航)
    uint8_t nav_state;    // 导航状态码
    uint8_t tail;         // 0x5B
};
```

SendPacket 由两个线程协作填充：
- **Armor 线程**：写入 `found`, `fire_advice`, `target_yaw`, `target_pitch`
- **ROS2 线程**：写入 `vx`, `vy`, `nav_state`
- **Serial 线程**：定时将整个结构体通过串口发出

---

## 4. 线程详解

### 4.1 Camera 线程 — 海康相机采集

```cpp
void camera_thread_func(
    const std::string& config_path,
    size_t buffer_size,
    std::shared_ptr<rigtorp::SPSCQueue<CameraFrame>> buffer)
{
    hikcamera::HikCamera cam(config_path);
    if (!cam.openCamera()) return;  // 相机打开失败则线程直接退出

    while (!g_shutdown) {
        CameraFrame data = cam.getImage();   // 阻塞等待海康 SDK 返回图像
        if (!data.frame.empty()) {
            buffer->try_push(std::move(data)); // 非阻塞推入队列
        } else {
            std::this_thread::sleep_for(5ms);  // 获取失败则短暂休眠
        }
    }
    cam.closeCamera();
}
```

**关键设计点：**

| 设计 | 原因 |
|------|------|
| `getImage()` 阻塞等待 | 海康 SDK 内部有 DMA 环形缓冲，`getImage()` 等待有图时才返回——不空转 CPU |
| `try_push` 而非 `push` | 如果 Armor 线程处理太慢导致队列满，`try_push` 直接丢弃旧帧（返回 false）而不是阻塞相机线程。这保证了相机线程永远不被下游拖慢 |
| `std::move` 语义 | 将 `cv::Mat` 的所有权移入队列，避免图像数据拷贝（cv::Mat 内部是引用计数智能指针） |
| 休眠策略 | 只在获取失败时休眠 5ms，正常情况全速运行 |

### 4.2 USBCamera 线程 — USB 相机采集

```cpp
void usb_camera_thread_func(
    size_t buffer_size,
    std::shared_ptr<rigtorp::SPSCQueue<CameraFrame>> buffer)
{
    std::string device_path = UsbCameraConfigManager::getDevicePath();
    auto cam = std::make_shared<usbcamera::UsbCamera>(device_path);
    if (!cam->open()) return;

    while (!g_shutdown) {
        CameraFrame data = cam->captureImage();
        if (!data.frame.empty()) {
            buffer->try_push(std::move(data));
        }
        std::this_thread::sleep_for(10ms);  // 每帧固定休眠(通过配置的FPS控制)
    }
    cam->close();
}
```

与 HikCamera 线程的差异：

| 差异 | HikCamera | USBCamera |
|------|-----------|-----------|
| 采集方式 | SDK DMA 阻塞等待 | 主动 capture + 固定休眠 |
| 帧率 | ~180fps（工业相机） | ~30fps（USB 摄像头） |
| 用途 | 主自瞄 | 辅助视角（可选，由 YAML enable 控制） |
| 休眠策略 | 仅在失败时休眠 | 每帧固定休眠 10ms |

**注意：** USB 相机的图像在 Armor 线程中只做检测，**不做**坐标变换/跟踪/射击——仅用于辅助观察，写入了 `processed = true` 但无 IMU 处理和射击输出。

### 4.3 Serial 线程 — 250Hz 串口收发

```cpp
void serial_thread_func()
{
    UserSerial& serial = UserSerial::getInstance();

    auto last_send_time = std::chrono::steady_clock::now();
    const auto send_interval = std::chrono::milliseconds(4); // 250Hz = 4ms

    while (!g_shutdown) {
        auto current_time = std::chrono::steady_clock::now();

        // 发送：严格 250Hz 周期
        if (current_time - last_send_time >= send_interval) {
            serial.sendPacketToSerial();   // 将 send_packet_ 写入串口
            last_send_time = current_time;
        }

        // 接收：每次循环都尝试
        serial.receiveSerial();            // 读取串口缓冲区 → 解析包 → 推入 data_queue_

        std::this_thread::sleep_for(1ms);  // 高频轮询
    }
    serial.closeSerial();
}
```

**为什么发送和接收用不同策略？**

- **发送 (250Hz 定时)**：STM32 的控制循环以 250Hz 运行，视觉端需要同步这个频率。4ms 间隔是精确的死线——早发浪费带宽，晚发导致控制延迟。
- **接收 (连续轮询)**：串口数据是异步到达的，无法预测何时有新数据。每 1ms 尝试读一次，保证 IMU 数据的延迟最小化（最坏情况 1ms + 串口传输时间）。

**UserSerial 内部的线程安全：**

```
Serial Thread (唯一写者)：
  receiveSerial() → 解析 ReceivePacket → data_queue_.try_push() → QUATERNION QUEUE
  sendPacketToSerial() → serial_port->write(&send_packet_, ...)

Armor Thread (读者)：
  serial.q(timestamp) → data_queue_.front()/pop() → Slerp 插值

ROS2 Thread (读者 + 部分写者)：
  serial.getRefereeData() ← 读取 referee_data_ (串口线程写入)
  serial.sendNav() → 修改 send_packet_.vx/vy/nav_state (与 Armor 线程并发写 send_packet_)
```

**潜在问题：ROS2 线程和 Armor 线程都可能写 `send_packet_`，但写的是不同字段**（Armor 写 yaw/pitch/found/fire，ROS2 写 vx/vy/nav_state），利用结构体字段级别的天然隔离避免数据竞争。这不是严格意义上的线程安全，但在 x86/ARM 平台上由于字段不跨 cache line，实践中是安全的。

### 4.4 Armor 线程 — 视觉主流水线

```cpp
void armor_thread_func(
    std::shared_ptr<SPSCQueue<CameraFrame>> hikcamerabuffer,
    std::shared_ptr<SPSCQueue<CameraFrame>> usbcamera_buffer)
{
    auto_aim::ArmorDetector detector("config/auto_aim.yaml");
    auto_aim::ArmorTrack    tracker("config/auto_aim.yaml", "config/hikcamera.yaml");
    auto_aim::ArmorShoot    shooter("config/auto_aim.yaml");
    UserSerial& serial = UserSerial::getInstance();
    VisionMode  last_mode = AUTO_AIM_RED;

    while (!g_shutdown) {
        bool processed = false;

        VisionMode current_mode = serial.getVisionMode();
        // 模式变化时切换识别颜色
        if (current_mode != last_mode) {
            detector.ArmorDetector_Set_Color(
                current_mode == AUTO_AIM_RED ? RED : BLUE);
            last_mode = current_mode;
        }

        // 非红蓝自瞄模式则跳过
        if (current_mode != AUTO_AIM_RED && current_mode != AUTO_AIM_BLUE) {
            std::this_thread::sleep_for(100ms);
            continue;
        }

        // === 处理海康相机帧 ===
        if (auto* f = hikcamerabuffer->front()) {
            CameraFrame frame = std::move(*f);
            hikcamerabuffer->pop();

            if (!frame.frame.empty()) {
                // Step 1: IMU 数据有效性检查
                if (!serial.isQuaternionValid()) {
                    serial.sendVision(0, 0, 0, 0); // 发送零角度，保持云台不动
                    processed = true;
                    continue;
                }

                // Step 2: 装甲板检测
                auto armors = detector.ArmorDetect(frame.frame, "HikCamera");

                // Step 3: 坐标变换 (IMU 四元数 → 世界坐标系旋转)
                Eigen::Quaterniond pos = serial.q(frame.timestamp);
                tracker.armor_pose().set_R_gimbal2world(pos);

                // Step 4: 目标跟踪 (FSM + EKF)
                auto target = tracker.track(armors, frame.timestamp,
                    "HikCamera_track", frame.frame);

                // Step 5: 弹道解算 + 射击决策
                auto shoot_result = shooter.shoot(target, frame.timestamp,
                    tracker.armor_pose().Get_R_gimbal2world(),
                    frame.frame, "HikCamera_shoot");

                // Step 6: 发送控制指令
                serial.sendVision(shoot_result.target_yaw,
                    shoot_result.target_pitch,
                    shoot_result.found, shoot_result.fire_advice);

                processed = true;
            }
        }

        // === 处理 USB 相机帧（仅检测，不跟踪不射击）===
        if (usbcamera_buffer) {
            if (auto* f = usbcamera_buffer->front()) {
                CameraFrame frame = std::move(*f);
                usbcamera_buffer->pop();
                if (!frame.frame.empty()) {
                    auto armors = detector.ArmorDetect(frame.frame, "USB Camera");
                    processed = true;
                }
            }
        }

        // 无帧处理时短暂休眠
        if (!processed) {
            std::this_thread::sleep_for(2ms);
        }
    }
}
```

**主流水线每一步的作用：**

```
┌──────────────────────────────────────────────────────────────────┐
│  1. front()/pop()  — 从 SPSCQueue 取最新帧（非阻塞）                │
│  2. isQuaternionValid() — 检查 IMU 数据是否新鲜（<1秒）              │
│  3. ArmorDetect()  — 二值化 → 灯条提取 → 配对 → ONNX 数字识别       │
│  4. serial.q(t)    — Slerp 插值得到曝光时刻的精确 IMU 姿态            │
│  5. set_R_gimbal2world() — 相似变换：R_gimbal2imubodyᵀ·R_imubody2world·R_gimbal2imubody │
│  6. tracker.track() — FSM 状态机 + EKF 滤波 → 稳定目标               │
│  7. shooter.shoot() — 选择瞄准模式 → 弹道解算 → yaw/pitch + 开火建议  │
│  8. sendVision()    — 写入 send_packet_，等待串口线程发出             │
└──────────────────────────────────────────────────────────────────┘
```

**SPSCQueue 的 front() 返回 nullptr 意味着队列为空**——相机还没推入新帧。此时 `processed` 保持 false，循环休眠 2ms 后重试。

### 4.5 ROS2 线程 — UDP 桥接 Docker

```cpp
void ros2_thread_func(UserSerial& user_serial)
{
    Ros2_comm ros2_comm;

    bool is_initialized = false;
    bool was_connected = false;
    auto last_init_attempt = std::chrono::steady_clock::now();

    while (!g_shutdown) {
        // 懒初始化：每 2 秒重试一次 UDP socket 绑定
        if (!is_initialized) {
            // 2秒间隔重试
            if (elapsed >= 2000ms) {
                if (ros2_comm.init() == 0) is_initialized = true;
            }
        }

        if (is_initialized) {
            // 发送方向：裁判系统数据 → UDP → Docker
            RefereePacket referee_data = user_serial.getRefereeData();
            ROS2_SEND_PACKET send_pkt;
            std::memcpy(&send_pkt, &referee_data, sizeof(send_pkt));
            ros2_comm.send(send_pkt);

            // 接收方向：Docker → UDP → 导航指令
            ROS2_RECV_PACKET recv_pkt;
            if (ros2_comm.recv(recv_pkt) > 0) {
                user_serial.sendNav(recv_pkt.vx, recv_pkt.vy, recv_pkt.nav_state);
                if (!was_connected) {
                    MAS_LOG_INFO("Receiving data from ROS2 (Link is UP)");
                    was_connected = true;
                }
            }
        }

        std::this_thread::sleep_for(10ms); // ~100Hz
    }
}
```

**数据流向：**

```
Armor Thread → sendVision() ─┐
                              ├→ send_packet_ ──→ Serial Thread → STM32
ROS2 Thread  → sendNav() ────┘

Serial Thread → referee_data_ ──→ ROS2 Thread → UDP → Docker ROS2
Docker ROS2  → UDP → ROS2 Thread → sendNav() → send_packet_ → STM32
```

### 4.6 Display — 主线程中的 SDL2 显示

虽然 Display 也是一个独立线程（由 `rm_utils::Display::getInstance().start(pool)` 提交到线程池），但它的实现不直接属于 `threads/` 目录。它在 `main.cpp` 的主流程中启动：

```cpp
rm_utils::Display::getInstance().start(pool);
```

Display 内部为每个**窗口名**（如 `"HikCamera_track"`, `"HikCamera_shoot"` 等）维护一个独立的 SPSCQueue。任何线程调用 `display.display_add(window_name, img, texts, ...)` 即可线程安全地提交渲染帧。

---

## 5. 线程间数据流全景

```
                              ┌──────────────┐
                              │  HikCamera   │ (SDK DMA 环形缓冲)
                              │  硬件传感器    │
                              └──────┬───────┘
                                     │ getImage() 阻塞
                              ┌──────▼───────┐
                              │ Camera Thread │
                              │ try_push()    │
                              └──────┬───────┘
                                     │ SPSCQueue<CameraFrame>(2)
                              ┌──────▼──────────────────────┐
                              │                             │
                     ┌────────▼────────┐           ┌───────▼────────┐
                     │ Armor Thread    │           │ USB Camera      │
                     │                 │           │ Thread          │
                     │ ArmorDetect()   │◄──────────│ (可选, YAML控制) │
                     │ ArmorTrack()    │ SPSCQueue │ try_push()      │
                     │ ArmorShoot()    │ (2)       └────────────────┘
                     │                 │
                     │ UserSerial::    │
                     │  q(timestamp)───┼──── Slerp 插值
                     │  sendVision()   │
                     └────────┬────────┘
                              │
              ┌───────────────┼───────────────┐
              │               │               │
     ┌────────▼────────┐     │      ┌────────▼────────┐
     │ Serial Thread   │     │      │  ROS2 Thread     │
     │                 │     │      │  (可选, YAML控制)  │
     │ sendPacketTo    │     │      │                  │
     │ Serial() 250Hz──┼─────┘      │ getRefereeData() │
     │                 │            │  ──UDP──► Docker  │
     │ receiveSerial() │            │  ◄──UDP── Docker  │
     │  → data_queue_  │            │ sendNav()         │
     └────────┬────────┘            └────────┬─────────┘
              │                              │
              │         send_packet_         │
              │  ┌─────────────────────┐     │
              └──► yaw/pitch/found/fire│◄────┘
                 │ vx/vy/nav_state     │
                 └─────────┬───────────┘
                           │ serial_port->write()
                    ┌──────▼──────┐
                    │    STM32    │
                    │  (下位机)    │
                    └─────────────┘
```

**关键时间约束：**

| 路径 | 延迟要求 | 原因 |
|------|----------|------|
| 相机曝光 → Armor 收到帧 | < 5ms | SPSCQueue(2) 只缓冲最新帧 |
| Armor 检测 → sendVision | < 10ms | 视觉处理必须在一帧内完成 |
| sendVision → 串口发出 | < 4ms | Serial 线程 250Hz 保证 |
| 串口收到 IMU → Armor 可用 | < 1ms | receiveSerial 每 1ms 轮询 |
| 总 Pipeline 延迟 | < 20ms | 从曝光到 STM32 收到角度指令 |

---

## 6. SPSCQueue 深度解析

### 6.1 为什么 capacity 要 +1

环形缓冲区用两个指针 `writeIdx` 和 `readIdx` 判断空/满。如果不留空位，则 `writeIdx == readIdx` 无法区分"全空"和"全满"。因此实际分配的 slot 数 = capacity + 1，其中一个不存数据，只用于区分。

```
   [slot0] [slot1] [slot2] [slot3] [slot4]  ← 用户请求 capacity=4，实际分配 5

   空状态: writeIdx == readIdx
   满状态: (writeIdx + 1) % capacity_ == readIdxCache_
```

### 6.2 Cache Line 对齐与 False Sharing 防护

```cpp
// 内存布局（简化）
| slots_* | allocator | PADDING... |
| writeIdx_ (64B aligned) | readIdxCache_ (64B aligned) |
| readIdx_  (64B aligned) | writeIdxCache_ (64B aligned) |
```

四个索引变量各自独占一个 64 字节的 cache line：

- **`writeIdx_`** — 生产者更新，消费者只读
- **`readIdxCache_`** — 生产者缓存消费者的读指针，避免每次都跨核心读
- **`readIdx_`** — 消费者更新，生产者只读
- **`writeIdxCache_`** — 消费者缓存生产者的写指针

**如果没有 cache line 对齐会怎样？**

```
❌ False Sharing 场景：
   Core0 (生产者) 写 writeIdx_  ──→ 该 cache line 在 Core1 上被标记为 Invalid
   Core1 (消费者) 读 readIdxCache_ (同一 cache line) ──→ 被迫从内存重新加载
   → 即使两个变量没有直接依赖，硬件缓存一致性协议也会产生大量总线流量
```

解决方案：`alignas(64)` 保证每个索引独占一条 cache line（x86 的 cache line 大小是 64 字节）。相邻的 `writeIdx_` 和 `readIdxCache_` 之间存在至少 48 字节的 padding。

**`kPadding` 的作用**是防止 `slots_` 数组与第一个索引变量挤在同一条 cache line 中——在数组前额外分配 `kPadding` 个 T 元素的空间。

### 6.3 内存序（Memory Order）分析

SPSCQueue 使用**三种** C++ 内存序，精确平衡了正确性和性能：

| 操作 | 内存序 | 含义 |
|------|--------|------|
| 读自己的指针 (relaxed) | `memory_order_relaxed` | 同一核心上，store/load 顺序天然保证，不需要同步 |
| 读对方的指针 (acquire) | `memory_order_acquire` | 保证读到对方最新的 store，且之后的所有读写不会被重排到此之前 |
| 更新自己的指针 (release) | `memory_order_release` | 保证之前的所有读写不会被重排到此之后，对方 acquire 能看到 |

**put 路径（生产者视角）：**

```cpp
bool try_push(T&& v) {
    auto writeIdx = writeIdx_.load(relaxed);      // (1) 读自己的写指针
    auto next = writeIdx + 1;
    if (next == capacity_) next = 0;

    if (next == readIdxCache_) {                   // (2) 检查是否满
        readIdxCache_ = readIdx_.load(acquire);    // (3) 队列满时才去读对方的读指针
        if (next == readIdxCache_) return false;   // 真的满了
    }

    new (&slots_[writeIdx + kPadding]) T(v);       // (4) 构造对象
    writeIdx_.store(next, release);                // (5) 发布——消费者可见
    return true;
}
```

**关键优化：`readIdxCache_` 缓存**

绝大多数情况下队列不满，生产者只需要读自己的 `writeIdx_`(relaxed) 和缓存的 `readIdxCache_`——这两条都在自己的 cache line 上，**无需跨核心通信**。只有 `next == readIdxCache_` 时才真正去 `acquire` 读取消费者的 `readIdx_`。

**get 路径（消费者视角）：**

```cpp
T* front() {
    auto readIdx = readIdx_.load(relaxed);          // (1) 读自己的读指针
    if (readIdx == writeIdxCache_) {                // (2) 检查是否空
        writeIdxCache_ = writeIdx_.load(acquire);   // (3) 队列空时才去读对方的写指针
        if (writeIdxCache_ == readIdx) return nullptr; // 真的空了
    }
    return &slots_[readIdx + kPadding];             // (4) 返回指针
}

void pop() {
    auto readIdx = readIdx_.load(relaxed);
    slots_[readIdx + kPadding].~T();                // 析构对象
    auto next = readIdx + 1;
    if (next == capacity_) next = 0;
    readIdx_.store(next, release);                  // 发布——生产者可见
}
```

**为什么不用 `memory_order_seq_cst`（默认）？**

`seq_cst` 保证全局统一的操作顺序，但需要**全总线锁**或等效的硬件机制。在 SPSC 场景中，线程之间只需要生产者和消费者两两同步，`acquire/release` 对提供了足够的 ordering 保证，且不产生不必要的硬件开销。

### 6.4 为什么相机队列用 capacity=2

```cpp
auto camera_buffer = std::make_shared<SPSCQueue<CameraFrame>>(2);
```

| 容量 | 行为 |
|------|------|
| 1 | 生产者必须等消费者 pop 后才能放入新帧 → 相当于**同步管道**，失去了解耦作用 |
| 2（当前） | 一帧在队列中被处理，一帧可以排队。如果 Armor 线程略慢，下一帧已就绪。如果再慢，`try_push` 失败→**自动丢帧**，始终处理最新帧 |
| >2 | 消费者可能处理积压的旧帧，引入延迟——视觉系统应处理最新图像，而非历史图像 |

capacity=2 实现了 **"latest-frame buffer"**——Armor 线程永远处理相机最新产出的帧，中间跳过的帧自动丢弃。

---

## 7. 线程同步机制

### 7.1 g_shutdown — 无锁退出信号

```cpp
// apps/main.cpp
std::atomic<bool> g_shutdown{false};

void signal_handler(int signum) {
    g_shutdown = true;  // SIGINT (Ctrl+C) 或 SIGTERM
}
```

所有线程的主循环都以 `while (!g_shutdown)` 作为条件。`std::atomic<bool>` 保证：
- 信号的写入对所有线程**立即可见**（即使信号处理函数在其他上下文执行）
- 不需要互斥锁——原子变量的 store/load 本身就是线程安全的

退出流程：
1. 主线程接收到 `g_shutdown == true`
2. `Display::shutdown()` — 关闭 SDL2 窗口
3. 逐一对每个 `std::shared_future<void>` 调用 `.get()` — **阻塞等待线程退出**
4. 每个线程的局部析构函数执行清资源：`cam.closeCamera()`, `serial.closeSerial()`, `ros2_comm.close()`

### 7.2 UserSerial 单例 — 串口共享访问

```cpp
static UserSerial& getInstance(const std::string& config_path = "...")
{
    static UserSerial instance(config_path);  // C++11 保证线程安全的静态初始化
    return instance;
}
```

C++11 起，函数内 `static` 局部变量的初始化是线程安全的——编译器自动插入保护。`UserSerial` 是唯一通过单例共享的资源（Display 也是单例），其他通信全部通过 SPSCQueue。

### 7.3 为什么整个系统没有互斥锁

| 同步需求 | 解决方案 | 机制 |
|----------|----------|------|
| 相机 → Armor 帧传递 | SPSCQueue | 无锁原子变量 |
| Serial → Armor IMU 传递 | SPSCQueue(5000) | 无锁原子变量 |
| Armor/ROS2 → Serial 发送 | `send_packet_` 字段隔离 | 不同线程写不同字段 |
| 多线程退出协调 | `std::atomic<bool>` | 原子标志 |
| Serial 单例初始化 | C++11 函数内 static | 编译器保证 |
| 日志写入 | spdlog | 库内部异步处理 |

**无锁设计的优势：**
- **确定性延迟**：无锁操作的时间是常数，不会被互斥锁的上下文切换打乱
- **避免优先级反转**：相机线程（实时 I/O）不会因为等 Armor 线程释放锁而阻塞
- **简化代码**：没有 lock/unlock 配对，没有死锁风险

---

## 8. 关键设计模式总结

| 模式 | 应用 | 效果 |
|------|------|------|
| **SPSC 无锁队列** | 所有跨线程数据传递 | 零阻塞，确定性延迟 |
| **Latest-frame buffer** | 相机队列 capacity=2 | 始终处理最新帧，自动跳帧 |
| **单例模式** | UserSerial, Display | 唯一硬件资源的访问点 |
| **自由函数线程** | 所有 `*_thread_func` | 无线程类生命周期管理，参数即依赖 |
| **原子标志退出** | `g_shutdown` | 优雅关闭，无锁协调 |
| **懒初始化 + 重试** | ROS2 线程 UDP socket | 容错启动，2 秒间隔重试 |
| **字段级隔离** | `send_packet_` | 多写者写不同字段，无锁并发 |
| **时间戳 + Slerp** | IMU 四元数插值 | 精确时间对齐，消除 IMU 与相机的时域偏差 |
| **可选子系统** | USB 相机 / ROS2 | YAML enable 控制，代码级解耦 |
| **线程池提交** | `BS::thread_pool` | 统一线程生命周期管理 |

---

> **建议的学习路径：**
> 1. 先从 `apps/main.cpp` 理解线程的创建和启动参数
> 2. 通读 `camera_thread.cpp` 和 `serial_thread.cpp`——它们是两个最简单的线程
> 3. 理解 SPSCQueue 的 `try_push`/`front`/`pop` 语义（不需要读完 SPSCQueue.h 的全部实现）
> 4. 再读 `armor_thread.cpp` 的完整流水线——它是整个系统的中枢
> 5. 最后读 `ros2_thread.cpp` ——它是可选的数据转发层
> 6. 画一遍线程间数据流图，标注每条边的数据结构
