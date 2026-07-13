# SP_Vision IO 与多线程架构详解

> 📘 **母文档**：[[SP_Vision_学习指南]]（先读母文档建立全局框架，本篇深化其 §5.2 / §5.3 / §5.4）
> 📘 **服务对象**：本篇的四元数插值为 [[SP_Vision_装甲板识别与解算详解]] 的 `set_R_gimbal2world` 与 [[SP_Vision_整车估计EKF详解]] 的观测提供云台姿态
> 📘 **相关**：[[SP_Vision_轨迹规划器详解]] 的规划结果经本篇的通信协议下发；[[SP_Vision_打符与全向感知详解]] 的多相机也用本篇的线程安全队列

本篇聚焦支撑整个自瞄的**基础设施**：硬件抽象层（相机、下位机、IMU）与多线程架构。核心难点是"**图像和姿态如何跨异步时间轴对齐**"、"**收发协议每个字节是什么**"、"**多线程如何在 150fps 采集下保持 500Hz 高频输出**"。

---

## 目录

1. [图像-四元数对齐：imu_at 的 slerp 插值](#1-图像-四元数对齐imu_at-的-slerp-插值)
2. [为什么队列要 5000 帧、为什么补偿 −1ms](#2-为什么队列要-5000-帧为什么补偿-1ms)
3. [三套 IMU 源的对齐算法](#3-三套-imu-源的对齐算法)
4. [通信协议：下行（电控→视觉）](#4-通信协议下行电控视觉)
5. [通信协议：上行（视觉→电控）](#5-通信协议上行视觉电控)
6. [定点传输、CRC 与坏帧防护](#6-定点传输crc-与坏帧防护)
7. [相机抽象与时间戳](#7-相机抽象与时间戳)
8. [多线程模型：三线程解耦](#8-多线程模型三线程解耦)
9. [OpenVINO 异步推理与队列策略](#9-openvino-异步推理与队列策略)
10. [概念关系图谱](#10-概念关系图谱)
11. [相关笔记](#11-相关笔记)

---

## 1. 图像-四元数对齐：imu_at 的 slerp 插值

### 1.1 问题

相机曝光瞬间打的时间戳 `t`，要找**那一刻云台的姿态**（用于把装甲板坐标从相机系转到世界系）。但 IMU 姿态是异步离散到达的——CAN/串口以自己的频率推送四元数，时间戳几乎永远不等于 `t`。必须在 `t` 两侧的两帧姿态之间插值。

### 1.2 imu_at 完整实现（CAN 版，cboard.cpp:22-45）

```cpp
Eigen::Quaterniond CBoard::imu_at(steady_clock::time_point timestamp) {
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;
  while (true) {
    queue_.pop(data_behind_);                        // 阻塞式取新帧
    if (data_behind_.timestamp > timestamp) break;   // 直到夹住 t
    data_ahead_ = data_behind_;
  }
  Eigen::Quaterniond q_a = data_ahead_.q.normalized();   // t 之前最近一帧
  Eigen::Quaterniond q_b = data_behind_.q.normalized();  // t 之后最近一帧
  auto k = (timestamp - data_ahead_.timestamp) / (data_behind_.timestamp - data_ahead_.timestamp);
  return q_a.slerp(k, q_b).normalized();             // 球面线性插值
}
```

三个要点：

1. **双游标夹逼**：`data_ahead_`（≤t）和 `data_behind_`（>t）是跨调用持久的成员（`cboard.hpp:60-61`），保留上次夹逼的残留状态。循环 `pop` 新帧直到 `data_behind_` 越过 `t`，则两游标恰好夹住 `t`。
2. **slerp 球面插值**：$k = \frac{t - t_a}{t_b - t_a}$ 是插值比例，`q_a.slerp(k, q_b)` 在两个四元数间做球面线性插值。为什么用 slerp 而非线性插值：四元数在单位球面上，线性插值会离开球面（模长变化 = 旋转速度失真），slerp 沿球面测地线插值，角速度恒定，才是物理正确的姿态插值。插值前后都 `.normalized()`。
3. **阻塞式等帧**：`queue_.pop` 走 `ThreadSafeQueue::pop`，队列空时 `wait` 阻塞（`thread_safe_queue.hpp:39-52`）。所以若请求的 `t` 还没有更晚的 IMU 帧到达，这个调用会**阻塞等下一帧**——保证一定能夹住 `t`，代价是主循环节奏受 IMU 帧率牵制。

> **一句话速记**：imu_at 用两个持久游标不断 pop 新姿态帧夹住图像时间戳 t，再用 slerp（球面线性插值，物理正确的姿态插值）算出 t 时刻姿态；阻塞式等帧保证一定夹住，代价是受 IMU 帧率牵制。

---

## 2. 为什么队列要 5000 帧、为什么补偿 −1ms

### 2.1 IMU 队列容量 5000

`cboard.cpp:12` 构造 `queue_(5000)`。为什么这么大：imu_at 要用**历史**姿态夹逼图像时间戳，而图像从曝光到处理有延迟，请求的 `t` 往往是"过去某一刻"。队列必须保留足够长的历史四元数，才能回溯到那一刻。5000 帧在千 Hz 级 IMU 下约覆盖数秒历史，足够。

还有个初始化顺序陷阱（`cboard.hpp:58` 注释"必须在 can_ 之前初始化，否则存在死锁的可能"）：`queue_` 声明在 `can_` 之前，因为 CAN 接收回调会 push 队列，而"callback 的运行会早于构造函数完成"（`cboard.cpp:14`），队列必须先就绪。构造尾部先 `pop` 两帧等首帧到达（日志 "Waiting for q..." → "Opened."）。

> ⚠️ `ThreadSafeQueue` 默认 `PopWhenFull=false`（`thread_safe_queue.hpp:12`）：队列满时**丢弃新帧**而非弹出旧帧。IMU 队列 5000 足够大，正常不会满。

### 2.2 时间对齐补偿 −1ms

几乎所有主循环都用 `cboard.imu_at(t - 1ms)`（`standard.cpp:59`、`mt_standard.cpp:88`、`sentry.cpp:67`、`uav.cpp:69` 等十余处）。

原因：相机 `read` 打的 `steady_clock` 时间戳 `t` 是"**取到图之后**"的时刻，而真正的成像瞬间（曝光中点）比这早一个固定量（曝光 + 传输延迟约 1ms）。所以请求比 `t` 早 1ms 的姿态，才对应真实成像瞬间。这是经验性固定偏差补偿。标定工具 `handeye_test.cpp:58` 用可调的 `imu_at(t - 1ms * delay)` 扫描确认这个值。离线/标定路径（`capture.cpp:38`、`cboard_test.cpp:35`）不补偿，直接 `imu_at(t)`。

> **一句话速记**：队列 5000 帧是为了回溯历史姿态夹逼过去的图像时间戳；−1ms 补偿是因为相机时间戳打在"取到图后"、比真实成像瞬间晚约 1ms（曝光+传输），标定实验确定。

---

## 3. 三套 IMU 源的对齐算法

项目支持三种姿态来源，对齐算法**并非完全一致**：

| 源 | 函数 | 队列 | 算法 | −1ms |
|:----|:----|:----|:----|:----|
| CAN（CBoard） | `imu_at(t)` | 5000 | 双游标 while-pop 夹逼 + slerp | ✅ |
| DM_IMU（串口） | `imu_at(t)` | 5000 | 与 CBoard **逐行一致** | ✅ |
| 串口云台（Gimbal） | `q(t)` | 1000 | 每轮 pop+front 夹逼，可外推 | ❌ |

CAN 与 DM_IMU 的 `imu_at` 实现完全相同（`dm_imu.cpp:106-129` 对照 `cboard.cpp:22-45`）。

串口云台的 `q(t)`（`gimbal.cpp:64-78`）算法不同：

```cpp
while (true) {
  auto [q_a, t_a] = queue_.pop();      // 取一帧
  auto [q_b, t_b] = queue_.front();    // 看下一帧（不弹出）
  auto k = delta_time(t_a, t) / delta_time(t_a, t_b);
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
  if (t < t_a) return q_c;                          // t 早于队首 → 外推返回
  if (!(t_a < t && t <= t_b)) continue;             // 没夹住 → 继续
  return q_c;                                        // 夹住 → 返回
}
```

差异：同时 `pop`(q_a) + `front`(q_b) 看两帧、用 `tools::delta_time` 算比值、夹逼条件 `t_a < t && t <= t_b`、且对 `t < t_a`（早于队首）直接外推返回。队列容量 1000（比 CAN/DM 的 5000 小）。调用方（`standard_mpc.cpp:99` 等）用 `gimbal.q(t)`，**无 −1ms 补偿**。

> **一句话速记**：CAN 和 DM_IMU 的 imu_at 逐行一致（5000 队列 + −1ms）；串口云台 q(t) 算法不同（1000 队列、pop+front 夹逼、可对早于队首的 t 外推、无 −1ms）。

---

## 4. 通信协议：下行（电控→视觉）

电控给视觉发**姿态四元数 + 档位 + 弹速**。CAN 与串口两套载体。

### 4.1 CAN 下行（cboard.cpp:68-102）

分两个 CAN ID：

**`quaternion_canid_`**（姿态，`cboard.cpp:72-83`）：8 字节 = 4 个 int16（x,y,z,w 各 /1e4）：

```cpp
auto x = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e4;   // 高低字节拼 int16 再 /1e4
// y, z, w 同理
queue_.push({{w, x, y, z}, timestamp});   // Eigen::Quaterniond(w,x,y,z)
```

**`bullet_speed_canid_`**（弹速+档位，`cboard.cpp:86-90`）：`bullet_speed`（int16/1e2，m/s）、`mode`（档位）、`shoot_mode`、`ft_angle`（无人机专有）。

### 4.2 串口下行 GimbalToVision（gimbal.hpp:17-29）

```cpp
struct __attribute__((packed)) GimbalToVision {
  uint8_t  head[2] = {'S', 'P'};   // 帧头
  uint8_t  mode;                   // 0空闲 1自瞄 2小符 3大符
  float    q[4];                   // 姿态四元数 wxyz
  float    yaw, yaw_vel;
  float    pitch, pitch_vel;
  float    bullet_speed;
  uint16_t bullet_count;           // 子弹累计发送次数
  uint16_t crc16;
};
static_assert(sizeof(GimbalToVision) <= 64);
```

串口版用 float 直传（不定点），字段比 CAN 更全（含 yaw/pitch 及其速度）。解析后 `queue_.push({q, t})`（`gimbal.cpp:165-166`），档位等写入 `state_`。

`mode` 档位是视觉切换功能组的依据：main 进程读 mode 决定跑自瞄还是打符（对应母文档 §3.1 "一个 main 按档位选功能组"）。

> **一句话速记**：下行 = 姿态四元数 + 档位 mode + 弹速 + 子弹数；CAN 用两个 ID（姿态定点 int16/1e4、弹速档位），串口用 GimbalToVision 结构体（float 直传、字段更全）。

---

## 5. 通信协议：上行（视觉→电控）

视觉给电控发**开火/控制标志 + yaw/pitch 目标**。CAN 与串口的关键差异在**前馈量**。

### 5.1 CAN 上行（cboard.cpp:47-66）

8 字节，只发 control/shoot 两 bool + yaw/pitch + horizon_distance：

```cpp
frame.data[0] = command.control ? 1 : 0;
frame.data[1] = command.shoot ? 1 : 0;
frame.data[2] = (int16_t)(command.yaw * 1e4) >> 8;    // yaw 高字节
frame.data[3] = (int16_t)(command.yaw * 1e4);          // yaw 低字节
frame.data[4-5] = pitch 定点;
frame.data[6-7] = horizon_distance 定点（无人机专有）;
```

`io::Command` 结构（`command.hpp:6-13`）：`bool control; bool shoot; double yaw; double pitch; double horizon_distance;`。**CAN 版不发速度/加速度前馈**。

### 5.2 串口上行 VisionToGimbal（gimbal.hpp:33-44）

```cpp
struct __attribute__((packed)) VisionToGimbal {
  uint8_t  head[2] = {'S', 'P'};
  uint8_t  mode;                  // 0不控制 1控云台不开火 2控云台且开火
  float    yaw, yaw_vel, yaw_acc;      // ★ 位置 + 一阶 + 二阶前馈
  float    pitch, pitch_vel, pitch_acc;
  uint16_t crc16;
};
```

发送时 `mode = control ? (fire ? 2 : 1) : 0`（`gimbal.cpp:103`）。**串口版比 CAN 版多了 yaw/pitch 的速度、加速度前馈**——这正是 [[SP_Vision_轨迹规划器详解]] 里 MPC 输出的二阶量，供下位机做计算力矩控制（PID + 前馈）。

⚠️ 澄清一个母文档 §5.4 容易误读的点：**只有串口协议传速度/加速度前馈，CAN 协议不传**。走 MPC 轨迹规划的路径（`standard_mpc.cpp`）用的是串口 Gimbal，所以能发前馈。

> **一句话速记**：上行 = control/shoot + yaw/pitch；关键差异在前馈——CAN 只发角度（定点 int16/1e4），串口 VisionToGimbal 多发 yaw/pitch 的速度+加速度前馈（供下位机计算力矩控制），MPC 路径走串口。

---

## 6. 定点传输、CRC 与坏帧防护

### 6.1 定点传输（CAN）

CAN 每帧仅 8 字节，浮点角度要压缩成 int16。做法：乘 1e4 转 int16（保 4 位小数）、高低字节拆分（`cboard.cpp:54-59`）：

```cpp
frame.data[2] = (int16_t)(command.yaw * 1e4) >> 8;   // 高 8 位
frame.data[3] = (int16_t)(command.yaw * 1e4);         // 低 8 位（隐式截断）
// 接收侧还原：(int16_t)(data[0] << 8 | data[1]) / 1e4
```

弹速用 1e2（保 2 位小数，`cboard.cpp:87`）。串口用 float 不需定点。

### 6.2 CRC 校验

| 载体 | CRC 策略 |
|:----|:----|
| CAN | **硬件 CRC，软件不做**（send/callback 全程无 CRC 调用，靠 CAN 控制器硬件校验） |
| 串口 | 帧头 `'S','P'` + **CRC16 查表法**（DJI 标准表） |

串口 CRC16（`crc.cpp`）：`CRC16_TABLE[256]`、`CRC16_INIT=0xffff`（`crc.cpp:23-46`）。发送侧对除 crc16 字段外的全部字节算校验填入（`gimbal.cpp:89-90`）；接收侧先判 2 字节帧头 `'S'/'P'`（`gimbal.cpp:143-148`），再 `check_crc16`（末两字节 `(data[len-1]<<8)|data[len-2]` 比对，`crc.cpp:85-89`）。DM_IMU 也用 CRC16 逐 16 字节校验 3 个子帧。

### 6.3 坏帧防护：四元数模长校验

CAN 收到姿态后校验单位模长（`cboard.cpp:78-81`）：

```cpp
if (std::abs(x*x + y*y + z*z + w*w - 1) > 1e-2) {   // |q|² 偏离 1 超过 0.01
  logger()->warn("Invalid q: ...");
  return;   // 丢弃该帧，不入队
}
```

合法四元数模长必为 1，偏离说明传输出错，直接丢。（串口侧靠 CRC16 保证，无此模长校验。）

> **一句话速记**：CAN 因帧短用定点（角度×1e4 拆高低字节、弹速×1e2）+ 硬件 CRC + 四元数模长校验(|q|²−1>1e-2 丢弃)；串口用 float 直传 + 帧头 SP + CRC16 查表法（DJI 表）。

---

## 7. 相机抽象与时间戳

### 7.1 多态基类（有一个例外）

工业相机走多态抽象：

```
CameraBase (纯虚 read(Mat&, time_point&))    camera.hpp:11-16
    ├── HikRobot   (海康)                     hikrobot.hpp:16
    └── MindVision (迈德威视)
Camera (门面, 持 unique_ptr<CameraBase>)      camera.hpp:18-26
    └── 工厂按 yaml camera_name 选派生类       camera.cpp:11-32
```

⚠️ **例外**：`USBCamera`（全向感知用）**不继承 `CameraBase`**，也不走工厂，是个独立类（`usbcamera.hpp:13` 无继承），只是碰巧提供了同签名的 `read(img, t)`，靠鸭子类型在模板/泛型代码里通用。所以母文档 §5.2 里"相机多态、hikrobot/usbcamera 派生"的表述不够精确——真正的多态派生只有 HikRobot/MindVision，USBCamera 是同接口的独立类。

### 7.2 时间戳打点

关键：时间戳要在**取到图的瞬间立刻**打，越靠近曝光越好。`HikRobot` 在采集线程取到图、图像转换**之前**就 `steady_clock::now()`（`hikrobot.cpp:118`），USBCamera 在 `cap_.read` 之后打（`usbcamera.cpp:149`）。这个时间戳随图像一路传下去，供 imu_at 对齐。

### 7.3 帧率与断线重连

- HikRobot 帧率 `MV_CC_SetFrameRate(handle_, 150)`（`hikrobot.cpp:90`）= **150 fps**。
- 断线重连守护线程**每 100ms 检查**（`hikrobot.cpp:22-30`）：若未在采集，`capture_stop → reset_usb（libusb_reset_device 复位 USB）→ capture_start`。USBCamera 同样 100ms 检查、最多重开 20 次；SocketCAN 也是 100ms 守护重连。

> **一句话速记**：工业相机走 CameraBase 多态（HikRobot/MindVision），USBCamera 是同接口独立类（不继承）；时间戳在取图瞬间立刻打（越近曝光越好）；150fps，守护线程每 100ms 检查断线并 libusb 复位重连。

---

## 8. 多线程模型：三线程解耦

以多线程主程序 `mt_standard.cpp` 为例，把"采集 / 推理 / 决策发送"拆成不同频率的线程：

```
[相机采集线程]         [detect_thread]           [主线程]                [CommandGener 线程]
 HikRobot 内部          mt_standard.cpp:64-75     mt_standard.cpp:77-133   commandgener.cpp:37-70
 sleep 1ms 循环         camera.read →             detector.debug_pop()     sleep 2ms → ~500Hz
 150 fps                detector.push(异步推理)    (等推理结果)              aim → shoot
 queue_(1) 只留最新                                imu_at(t-1ms) 对齐        → cboard.send
                        MultiThreadDetector       solver → tracker
                        queue_ 容量 16            → commandgener.push
```

各线程职责与频率：

| 线程 | 频率 | 职责 | 队列 |
|:----|:----|:----|:----|
| 相机采集（HikRobot 内部） | 150 fps | 取图、打时间戳 | 输出 queue_(1) 只留最新帧 |
| detect_thread | 跟采集 | `camera.read` → `detector.push`（提交异步推理） | MultiThreadDetector queue_(16) |
| 主线程 | 跟推理 | `debug_pop` 取结果 → imu_at 对齐 → solver → tracker → `commandgener.push` | — |
| CommandGener | ~500 Hz（sleep 2ms） | `aim` + `shoot` → `cboard.send` | latest_ 单槽 |

**为什么这样拆**：

1. **相机队列容量 1**（`hikrobot.cpp:12` `queue_(1)`）：永远只留最新帧，天然丢旧帧保低延迟——视觉不怕丢几帧，怕的是处理过时的帧。
2. **500Hz 高频发指令**：CommandGener 独立 500Hz 循环，即使视觉帧率只有 ~150Hz，云台指令仍以 500Hz 高频输出，中间空档由 EKF 预测填补（发的是预测的当前位置）。这解耦了"视觉帧率"和"控制指令频率"。
3. **200ms 时效窗**（`commandgener.cpp:44`）：

```cpp
if (latest_ && delta_time(now, latest_->t) < 0.2)   // 最新目标距今 < 200ms
  input = latest_;
else
  input = std::nullopt;   // 陈旧目标丢弃，不发指令
```

`latest_` 是单槽 `optional`（`push` 时整体覆盖，只留最新），超 200ms 视为陈旧丢弃，防止目标丢失后仍用旧数据误发指令。`horizon_distance` 由目标 EKF 状态 `sqrt(x²+z²)` 算出（`commandgener.cpp:52-56`）。

单线程版（`sentry.cpp:65-104`）则在一个 while 循环里串行 read→imu_at→detect→track→aim→send，不拆线程（简单但吞吐低）。

> **一句话速记**：三线程解耦——采集 150fps(队列1只留最新)、detect 异步提交推理(队列16)、CommandGener 独立 500Hz 发指令(视觉帧率与控制频率解耦、空档由 EKF 预测填补)、200ms 时效窗丢陈旧目标。

---

## 9. OpenVINO 异步推理与队列策略

多线程识别器 `MultiThreadDetector` 用 OpenVINO **异步推理**让采集与推理重叠（`mt_detector.cpp`）：

**push 端**（`mt_detector.cpp:43-63`）：letterbox 预处理 → 创建 `infer_request` → `set_input_tensor` → **`start_async()`**（非阻塞启动推理）→ 立刻把 `{img, t, infer_request}` 入队返回。

**pop 端**（`mt_detector.cpp:65-80`）：从队列取出 → **`wait()`**（等这个请求推理完成）→ 才做后处理 `postprocess`。

关键是编译模型用 **THROUGHPUT** 模式（`mt_detector.cpp:37-38`）+ 每次 `create_infer_request()` 多请求并发：

```cpp
compiled_model_ = core_.compile_model(
  model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT));
```

于是 push 端持续提交、OpenVINO 后台并行跑多个 infer_request、pop 端等最早的完成——采集/提交的 CPU 时间和推理的 GPU/NPU 时间**重叠**，吞吐提升。queue_(16) 缓冲在途请求。输入固定 NHWC u8 640×640，BGR→RGB、归一化由 PrePostProcessor 内建（`mt_detector.cpp:22-34`，见 [[SP_Vision_装甲板识别与解算详解]] §3.2 图内预处理）。

> ⚠️ 对比：单帧同步推理是"提交 → 阻塞等结果 → 处理下一帧"，采集和推理串行；异步是"提交后立刻采下一帧，推理在后台跑"，两者重叠。THROUGHPUT 模式配多请求是异步的前提。

> **一句话速记**：MultiThreadDetector 用 OpenVINO 异步（push 端 start_async 不阻塞、pop 端 wait 再后处理）+ THROUGHPUT 模式多请求并发，让采集的 CPU 时间和推理的加速器时间重叠，queue(16) 缓冲在途请求。

---

## 10. 概念关系图谱

### 10.1 一帧数据的时间线与线程流转

```
 相机采集线程(150fps)          detect线程            主线程                CommandGener(500Hz)
 ─────────────────           ──────────          ──────────            ──────────────────
 曝光成像                                                               (独立循环)
   │ steady_clock::now()=t
   ▼ (queue容量1,留最新)
 图入队 ──────────────▶ camera.read(img,t)
                          detector.push(img,t)
                          start_async()提交 ─────▶ debug_pop() wait()推理完
                          (queue容量16)             │
                                                    ▼ imu_at(t−1ms) ── slerp插值
                                       IMU队列(5000)──┘ 夹逼t两侧姿态
                                                    ▼
                                       solver.set_R_gimbal2world(q)
                                       solver → tracker(EKF)
                                                    │
                                                    ▼ commandgener.push
                                            (latest_单槽,200ms时效窗) ───▶ aim+shoot
                                                                          cboard/gimbal.send
                                                                          (CAN定点/串口float+前馈)
```

### 10.2 三套 IMU 源对照

| | CAN (CBoard) | DM_IMU | 串口云台 (Gimbal) |
|:----|:----|:----|:----|
| 函数名 | imu_at(t) | imu_at(t) | q(t) |
| 队列容量 | 5000 | 5000 | 1000 |
| 夹逼方式 | while-pop 双游标 | 同 CAN | 每轮 pop+front |
| 早于队首 | 阻塞等 | 阻塞等 | 外推返回 |
| −1ms 补偿 | ✅ | ✅ | ❌ |
| 插值 | slerp | slerp | slerp |

### 10.3 收发协议对照

| | CAN | 串口 (Gimbal) |
|:----|:----|:----|
| 下行结构 | 两个 CAN ID（姿态/弹速） | GimbalToVision |
| 上行结构 | 8 字节裸帧 | VisionToGimbal |
| 数据格式 | 定点 int16（角度×1e4、弹速×1e2） | float 直传 |
| 上行前馈 | ❌ 仅角度 | ✅ 速度 + 加速度前馈 |
| CRC | 硬件 CRC（软件不做） | 帧头 SP + CRC16 查表 |
| 坏帧防护 | 四元数模长校验 | CRC16 |
| MPC 路径 | — | ✅（standard_mpc 走串口发前馈） |

### 10.4 核心概念对照

| 易混概念 | 区别 |
|:----|:----|
| **slerp vs 线性插值** | 四元数在单位球面上，线性插值会离开球面(旋转失真)；slerp 沿测地线、角速度恒定，物理正确 |
| **data_ahead_ vs data_behind_** | 夹逼图像时间戳 t 的前后两帧姿态；跨调用持久保存残留状态 |
| **−1ms 补偿** | 相机时间戳打在"取图后"，比真实成像瞬间晚约 1ms(曝光+传输)，故请求早 1ms 的姿态 |
| **CAN imu_at vs 串口 q(t)** | 前者 5000 队列 while-pop、有 −1ms；后者 1000 队列 pop+front、可外推、无 −1ms |
| **CAN 定点 vs 串口 float** | CAN 帧短需定点(×1e4 拆字节)；串口用 float，且多发速度/加速度前馈 |
| **CAN 硬件 CRC vs 串口 CRC16** | CAN 靠控制器硬件校验(软件不做)；串口软件查表 CRC16 + 帧头 SP |
| **CameraBase 派生 vs USBCamera** | HikRobot/MindVision 是真多态派生；USBCamera 是同接口独立类(不继承) |
| **相机队列 1 vs 检测队列 16** | 相机队列 1 只留最新帧(丢旧保低延迟)；检测队列 16 缓冲在途异步推理请求 |
| **150fps 视觉 vs 500Hz 指令** | CommandGener 独立 500Hz 发指令，视觉帧率与控制频率解耦，空档由 EKF 预测填补 |
| **同步 vs 异步推理** | 同步:提交→阻塞等→处理下帧(串行)；异步:start_async 后立刻采下帧、推理后台跑(重叠)+THROUGHPUT 多请求 |
| **200ms 时效窗** | 最新目标距今 >200ms 视为陈旧丢弃，防目标丢失后用旧数据误发指令 |

### 10.5 全局一句话速记

> IO 层用 imu_at 的 slerp 插值把异步姿态对齐到图像时间戳(双游标夹逼、5000 队列回溯历史、−1ms 补成像延迟)；收发协议 CAN 走定点+硬件 CRC+模长校验、串口走 float+CRC16 且多发速度/加速度前馈(供 MPC 计算力矩控制)；相机 CameraBase 多态(150fps、100ms 守护重连)；多线程三线程解耦(采集队列1、异步推理队列16、CommandGener 500Hz+200ms 时效窗)，视觉帧率与控制频率解耦、空档由 EKF 预测填补。

---

## 11. 相关笔记

- [[SP_Vision_学习指南]] — 母文档，全局框架（本篇深化 §5.2/5.3/5.4）
- [[SP_Vision_装甲板识别与解算详解]] — `set_R_gimbal2world` 用本篇 imu_at 插值的四元数；§3.2 的 OpenVINO 图内预处理与本篇异步推理配套
- [[SP_Vision_整车估计EKF详解]] — EKF 观测所需的云台姿态由本篇提供；CommandGener 的 500Hz 空档由 EKF 预测填补
- [[SP_Vision_轨迹规划器详解]] — MPC 输出的速度/加速度前馈经本篇串口 VisionToGimbal 下发
- [[SP_Vision_打符与全向感知详解]] — 全向感知的多相机并行用本篇的线程安全队列；USBCamera 是本篇的同接口独立类
- [[RM_Socket编程详解]] — 底层通信与串口/CAN 基础
- [[RM_C程序员_C++速通指南]] — `std::thread`、`unique_ptr`、多态、`std::optional`、`__attribute__((packed))` 等本篇技法