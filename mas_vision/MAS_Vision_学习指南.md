# MAS Vision 视觉系统 — 八周学习指南

> **目标读者**：具备 C++ 基础，希望深入理解 RoboMaster 自动瞄准视觉系统的开发者。
> **学习周期**：8 周，每周约 15-20 小时。
> **核心理念**：先理解原理，再读代码，最后动手修改。每一章都从"为什么需要这个"开始。
>
> 💡 **从 C 语言转过来的？** 先看 [[RM_C程序员_C++速通指南]] — 用本项目的真实代码解释 C++ 语法，半天看完就能读懂代码。
>
> 🔬 **想对照其他战队方案？** 看 [[SP_Vision_学习指南]] — 同济 SuperPower 25 赛季自瞄开源，最大亮点是"轨迹视角下的自瞄理论"（用 TinyMPC 轨迹规划器替代分段决策），可与本项目的设计选择对照。

---

## 目录

- [第1周：环境搭建与项目骨骼](#第1周环境搭建与项目骨骼)
- [第2周：核心依赖库深入](#第2周核心依赖库深入)
- [第3周：硬件驱动层 — 相机的眼睛与串口的神经](#第3周硬件驱动层--相机的眼睛与串口的神经)
- [第4周：IMU 四元数与坐标变换 — 空间的魔法](#第4周imu-四元数与坐标变换--空间的魔法)
- [第5周：装甲板检测 — 从像素到目标](#第5周装甲板检测--从像素到目标)
- [第6周：位姿解算与目标跟踪 — 知道敌人在哪](#第6周-位姿解算与目标跟踪--知道敌人在哪)
- [第7周：弹道解算与多线程架构 — 扣下扳机](#第7周弹道解算与多线程架构--扣下扳机)
- [第8周：系统集成、标定与调试](#第8周系统集成标定与调试)

---

## 第1周：环境搭建与项目骨骼

### 1.1 为什么选择 C++17

本项目处理的是**实时视觉+控制**任务。每一帧图像需要在十几毫秒内完成检测、位姿解算、跟踪、弹道解算并发送控制指令。C++ 是唯一能在嵌入式边缘计算设备（Jetson Orin / NUC）上同时满足**零开销抽象**和**确定性延迟**的语言。

C++17 相比 C++14 带来的关键特性在本项目中的使用：

| 特性                   | 使用位置                      | 为什么用                    |
| -------------------- | ------------------------- | ----------------------- |
| `std::optional<T>`   | `ArmorTrack::track()` 返回值 | 明确表达"可能有目标，也可能没有"，替代空指针 |
| `std::shared_ptr<T>` | main.cpp 中 buffer 传递      | 多个线程共享 SPSCQueue 的所有权   |
| `std::atomic<bool>`  | `g_shutdown`              | 无锁的多线程退出信号              |
| `if constexpr`       | 模板代码                      | 编译期分支，消除运行时开销           |
| `constexpr` 常量       | `armor_types.hpp` 中装甲板尺寸  | 编译期计算，嵌入代码段             |
| structured bindings  | 解包返回元组                    | 代码可读性                   |
| `std::chrono`        | 全局时间戳                     | 高精度计时，类型安全              |

**任务清单：**
- [x] 在 Jetson Orin / NUC 上完成 README.MD 中的依赖安装
- [x] 编译并运行 `./base`，确认串口、相机、显示正常
- [ ] 阅读 `apps/main.cpp`，画出程序的启动流程图

### 1.2 CMake 构建系统的设计思路

打开根目录 `CMakeLists.txt`，你会发现它非常简洁。核心设计：

```cmake
# 自动发现所有子目录的 CMakeLists.txt
file(GLOB_RECURSE CMAKE_FILES LIST_DIRECTORIES false "*/CMakeLists.txt")
foreach(CMAKE_FILE ${CMAKE_FILES})
    add_subdirectory(${DIR})
endforeach()

# 链接所有模块为静态库
target_link_libraries(${PROJECT_NAME} PRIVATE
    rm_utils hikcamera serial usbcamera hardware_common threads auto_aim
)
```

**为什么用静态库而不是动态库？**
1. 部署简单：一个可执行文件 + 配置文件即可，不需要管理 `.so` 路径
2. 链接时优化（LTO）：编译器可以跨模块内联和优化
3. 避免 ABI 兼容性问题

**为什么用 `GLOB_RECURSE`？**
- 添加新模块时不需要修改根 CMakeLists.txt，只需在子目录创建 `CMakeLists.txt`
- 缺点：新增文件后需要重新运行 `cmake ..` 才能被识别

**任务清单：**
- [ ] 理解每个子目录的 `CMakeLists.txt` 的依赖关系
- [ ] 尝试在 `auto_aim/` 下新建一个子目录，创建自己的 `CMakeLists.txt`，验证自动发现

### 1.3 项目模块地图

在深入代码之前，先在脑中建立一个"地图"：

```
mas_vision/
├── apps/              应用程序入口 + 标定工具
│   └── main.cpp       ← 一切的起点
├── auto_aim/          核心视觉算法（最重要！）
│   ├── armor_detector/   装甲板检测
│   ├── armor_track/      位姿解算 + 目标跟踪
│   └── armor_shoot/      弹道解算 + 火控
├── hardware/          硬件抽象层
│   ├── hikcamera/        海康工业相机驱动
│   ├── serial/           串口通信（与STM32）
│   └── usbcamera/        USB相机驱动
├── threads/           多线程调度
│   ├── camera_thread/    相机采集线程
│   ├── serial_thread/    串口收发线程
│   ├── armor_thread/     视觉处理主线程
│   └── ros2_thread/      ROS2 UDP通信线程
├── rm_utils/          工具库
│   ├── algorithm/        数学工具、EKF、弹道、正弦拟合
│   ├── display/          SDL2 可视化
│   ├── mas_log/          日志系统
│   ├── plotter/          UDP 数据绘图
│   ├── ros2_comm/        ROS2 UDP 桥接
│   ├── SPSCQueue/        无锁队列
│   └── thread_pool/      线程池
└── config/            运行时 YAML 配置
```

**记忆技巧**：数据从 `hardware` 流入，经过 `threads` 调度，在 `auto_aim` 处理，通过 `hardware/serial` 流出。`rm_utils` 为所有层提供工具支撑。

**任务清单：**
- [ ] 手绘（或工具绘制）上述模块的数据流图
- [ ] 在代码中找到每个模块的"入口函数"（通常是头文件中的主要类或函数）

---

## 第2周：核心依赖库深入

### 2.1 OpenCV — 不只是 `cv::imread`

本项目使用 OpenCV 的核心功能远超基本的图像读写：

| 功能 | 使用的 API | 位置 |
|------|-----------|------|
| 图像采集 | `cv::VideoCapture` | `usbcamera/` |
| 颜色空间转换 | `cv::cvtColor` (BGR→Gray, BGR→HSV) | `armor_detector/` |
| 二值化 | `cv::threshold` | `armor_detector/` |
| 轮廓提取 | `cv::findContours` | `armor_detector/` |
| 旋转矩形 | `cv::RotatedRect` | 灯条建模 |
| PnP 解算 | `cv::solvePnP` (IPPE 方法) | 位姿解算 |
| DNN 推理 | `cv::dnn::readNetFromONNX` | 数字识别 |
| 矩阵运算 | `cv::Mat`, `cv::cv2eigen` | 与 Eigen 互操作 |

**关键概念：`cv::solvePnP` 的 IPPE 方法**

本项目使用 `cv::SOLVEPNP_IPPE`（Infinitesimal Plane-based Pose Estimation）而非传统的 `ITERATIVE` 或 `EPNP`。原因：
- 装甲板是一个**平面**（4个角点共面），IPPE 专为平面目标设计
- IPPE 返回两个可能的解（平面有两个面），然后通过重投影误差选择正确的一个
- 精度高于 EPNP，速度满足实时要求

**任务清单：**
- [ ] 写一个独立的小程序：读取图片，提取红色区域的轮廓，画出旋转矩形
- [ ] 用硬编码的 3D 点和 2D 点测试 `cv::solvePnP`，理解输入输出
- [ ] 阅读 `armor_types.hpp` 中的 `BIG_ARMOR_POINTS` 和 `SMALL_ARMOR_POINTS`，理解 3D 模型点的定义方式

### 2.2 Eigen3 — 线性代数的 C++ 表达

本项目大规模使用 Eigen3 进行 3D 数学运算。与直接写 `double[9]` 或手写矩阵乘法相比，Eigen 的优势：
- **表达式模板**：`R * t + v` 编译为单次循环，不产生临时变量
- **类型安全**：`Eigen::Vector3d` 和 `Eigen::Quaterniond` 是不同的类型，编译器阻止错误赋值
- **与 OpenCV 互操作**：`cv::cv2eigen` / `cv::eigen2cv`

**本项目中最常用的 Eigen 类型和操作：**

```cpp
// 旋转表示
Eigen::Matrix3d    R;      // 3×3 旋转矩阵
Eigen::Quaterniond q;      // 四元数 (w, x, y, z)
Eigen::Vector3d    euler;  // 欧拉角 (yaw, pitch, roll)
Eigen::Vector3d    xyz;    // 3D 位置向量
Eigen::Vector4d    xyza;   // 3D 位置 + yaw 角度

// 旋转矩阵 ↔ 四元数
Eigen::Matrix3d R = q.toRotationMatrix();
Eigen::Quaterniond q(R);

// 坐标变换: P_world = R * P_gimbal
Eigen::Vector3d p_world = R_gimbal2world * p_gimbal;

// 相似变换: R_gimbal2world = Rᵀ · R_imu · R
R_gimbal2world = R_gimbal2imubody.transpose() * R_imubody2world * R_gimbal2imubody;
```

**为什么用四元数而不是欧拉角存储姿态？**
1. 万向节锁：欧拉角在 pitch=±90° 时丢失一个自由度
2. 插值：四元数可以 Slerp（球面线性插值），欧拉角直接插值会产生非均匀旋转
3. 数值稳定性：四元数只有 4 个数，旋转矩阵有 9 个数且需要保持正交性

**任务清单：**
- [ ] 在独立程序中用 Eigen 实现：绕 Z 轴旋转 45°，再绕 Y 轴旋转 30°，用矩阵乘法和四元数乘法分别计算
- [ ] 验证：用 `eulers(R, 2, 1, 0)` 提取欧拉角，检查是否与输入一致
- [ ] 理解 `rm_utils/algorithm/math_tools.hpp` 中的 `xyz2ypd` 和 `ypd2xyz` 函数

### 2.3 其他关键库速览

**yaml-cpp** — 所有配置文件的解析器：
```cpp
YAML::Node config = YAML::LoadFile("config/auto_aim.yaml");
int threshold = config["armor_detector"]["binary_threshold"].as<int>();
```

> 💡 **补充笔记：YAML 与在本项目中的使用**
> 
> **YAML**（YAML Ain't Markup Language）是一种**人类可读的数据序列化格式**。用缩进表示层级（只能用空格，不能用 Tab），无需花括号和逗号，原生支持 `#` 注释。
> 
> ### 本项目 5 个 YAML 配置文件一览
> 
> ```
> config/
> ├── auto_aim.yaml          ← 视觉算法核心参数（检测器、跟踪器、射击器）
> ├── hikcamera.yaml         ← 海康相机设置 + 标定数据（内参、外参、手眼矩阵）
> ├── serial_config.yaml     ← 串口通信参数（端口、波特率、数据位）
> ├── usbcamera_config.yaml  ← USB 相机备选方案参数
> └── ros2.yaml              ← ROS2 UDP 桥接开关
> ```
> 
> 构建时 CMake 自动创建 `build/config -> ../config` 符号链接，运行时从 build 目录通过相对路径加载。
> 
> ### 各文件实际结构与用途
> 
> **① `auto_aim.yaml`** — 视觉管线的心脏配置：
> ```yaml
> auto_aim:
>   armor_detector:
>     debug: false
>     binary_thres: 90              # 二值化阈值（增大→更严格，减少检出的噪点）
>     detect_color: RED             # 检测敌方颜色
>     lights_params:
>       min_lightbar_ratio: 1.5     # 灯条最小长宽比
>       max_angle_error: 45         # 灯条最大倾斜角度（度）
>     armors_params:
>       min_armor_ratio: 1          # 装甲板最小长宽比
>       max_rectangular_error: 25   # 配对时角度容差（度）
>     number_params:
>       model_path: "../auto_aim/armor_detector/model/lenet.onnx"
>       classifier_threshold: 0.7   # 数字识别置信度阈值
>       ignore_classes: ["negative"]
>   armor_track:
>     min_detect_count: 5           # 连续检测到 N 帧才进入 tracking
>     max_temp_lost_count: 25       # 短暂丢失最多维持 N 帧
>   armor_shoot:
>     yaw_offset: 0.5               # 偏航机械零位补偿（度）
>     bullet_speed: 24.0            # 子弹初速（m/s）
>     spinning_threshold_low: 2.0   # 转速低于此→直接跟板
>     spinning_threshold_high: 10.0 # 转速高于此→强制打旋转中心
>     yaw_tolerance_near: 2.0       # 3m 内射击容差（度）
> ```
> 这是调参最频繁的文件——检测灵敏度、跟踪稳定性、射击时机全在这里。
> 
> **② `hikcamera.yaml`** — 相机硬件 + 标定数据：
> ```yaml
> camera:
>   exposuretime: 6000.0            # 曝光时间（μs），越小越能冻结运动
>   gain: 16.0                      # 模拟增益
> calibration:
>   camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]  # 3×3 内参展平为 9 元素数组
>   distort_coeffs: [k1, k2, p1, p2, k3]            # 5 个畸变系数
> handeye_calibration:
>   R_gimbal2imubody: [0,1,0, -1,0,0, 0,0,1]       # 云台→IMU 安装偏差（3×3 展平）
>   R_camera2gimbal: [...]                           # 相机→云台 旋转矩阵（手眼标定结果）
>   t_camera2gimbal: [0.115, -0.001, 0.050]          # 相机→云台 平移向量（米）
> ```
> **YAML 在本文件中的关键用法**：矩阵以**展平数组**形式存储（`[r11,r12,r13, r21,r22,r23, r31,r32,r33]`），代码中用 `config["calibration"]["camera_matrix"].as<std::vector<double>>()` 读出后用 `cv::Mat(3,3, CV_64F, data.data()).clone()` 重建为矩阵。
> 
> **③ `serial_config.yaml`** — 串口参数：
> ```yaml
> serial:
>   port: "/dev/mascdc"            # 串口设备路径
>   baudrate: 115200               # 波特率
>   timeout: 2                     # 超时（秒）
>   parity: "none"                 # 校验位
> ```
> 
> **④ `usbcamera_config.yaml`** — USB 相机备选：
> ```yaml
> usbcamera:
>   enable: false                  # 开关（与海康相机二选一）
>   device_path: /dev/video0
>   width: 1280 / height: 720 / fps: 30
>   auto_exposure: 1               # 0=自动, 1=手动
> ```
> 
> **⑤ `ros2.yaml`** — ROS2 桥接：
> ```yaml
> ros2:
>   enabled: true                  # 开关
>   server_ip: "127.0.0.1"        # Docker 内 ROS2 节点地址
>   server_port: 8888
> ```
> 
> ### yaml-cpp 在本项目中的加载模式
> 
> ```cpp
> // 1. 加载文件
> YAML::Node cfg = YAML::LoadFile("config/auto_aim.yaml");
> 
> // 2. 按路径取值——用 [] 逐层深入
> auto detector  = cfg["auto_aim"]["armor_detector"];
> int  thres     = detector["binary_thres"].as<int>();
> auto color_str = detector["detect_color"].as<std::string>();
> 
> // 3. 数组值——存为 std::vector
> auto mat_flat = cfg["calibration"]["camera_matrix"].as<std::vector<double>>();
> cv::Mat K(3, 3, CV_64F, mat_flat.data());  // 重建为 OpenCV 矩阵
> 
> // 4. 列表值（ignore_classes）
> auto ignored = detector["number_params"]["ignore_classes"];
> for (auto&& cls : ignored)
>     skip_labels.insert(cls.as<std::string>());
> ```
> 
> ### 为什么不用 JSON？
> 
> | 对比维度 | YAML | JSON |
> |---------|------|------|
> | 注释 | ✅ `# 这是注释` | ❌ 不支持 |
> | 字符串引号 | 可省略 | 必须加 |
> | 嵌套表示 | 缩进（视觉层次清晰） | `{}` 嵌套（深层时括号堆叠） |
> | 数组 | `- item` 逐行 | `["item"]` |
> | 本项目适合度 | **调参场景频繁改参数，注释记录原因** | 适合机器间通信，不适合人类频繁编辑 |
> 
> 配置文件中像 `binary_thres: 90` 这样的魔法数字，如果没有旁边的 `# 二值化阈值` 注释，两周后调参者自己都会忘记含义——这就是 YAML 注释的价值。

**spdlog** — 异步日志系统（`rm_utils/mas_log/`）：
```cpp
MAS_LOG_INFO("Armor detected: {}", armor_number);
MAS_LOG_WARN("Serial timeout, reconnecting...");
MAS_LOG_ERROR("Camera initialization failed: {}", e.what());
```

**SDL2** — 显示窗口（`rm_utils/display/`）：
- 每个线程通过 `display_add()` 以无锁方式提交帧
- 支持文字、点、线、多边形的实时叠加

**任务清单：**
- [ ] 阅读 `rm_utils/mas_log/mas_log.hpp`，理解日志系统的设计
- [ ] 阅读 `rm_utils/display/display.hpp`，理解 `display_add()` 的参数和用法

---

## 第3周：硬件驱动层 — 相机的"眼睛"与串口的"神经"

### 3.1 海康工业相机 SDK 驱动

海康机器人（Hikrobot）的 MVS SDK 提供了 C 语言接口。本项目封装在 `hardware/hikcamera/`。

**为什么不用 OpenCV 直接打开海康相机？**
- OpenCV 的 `VideoCapture` 不支持海康相机的专有协议（GigE Vision / USB3 Vision）
- MVS SDK 提供更精细的控制：曝光时间（微秒级）、模拟增益、Gamma、触发模式
- 工业相机需要处理断线重连、丢帧、格式转换（BayerRG→BGR）

**核心流程：**
```
HikCamera::openCamera()
    → MV_CC_Initialize()        初始化 SDK
    → MV_CC_EnumDevices()       枚举设备
    → MV_CC_CreateHandle()      创建设备句柄
    → MV_CC_OpenDevice()        打开设备
    → MV_CC_SetPixelFormat()    设置像素格式 (BayerRG8 → BGR8)
    → MV_CC_SetExposureTime()   设置曝光
    → MV_CC_SetGain()           设置增益
    → MV_CC_StartGrabbing()     开始抓流

HikCamera::getImage()
    → MV_CC_GetImageBuffer()    阻塞获取一帧
    → 将 BayerRG → BGR 转换
    → 封装为 CameraFrame {cv::Mat, timestamp}
```

**任务清单：**
- [ ] 阅读 `hardware/hikcamera/hikcamera.hpp` 和 `.cpp`，画出 HikCamera 类的状态机
- [ ] 修改 `config/hikcamera.yaml` 中的 `exposure_time`（如从 4000 改为 2000），观察图像亮度变化
- [ ] 理解 `CameraFrame` 结构体的设计（为什么包含 `cv::Mat` 和 `std::chrono::time_point`）

### 3.2 串口通信 — 与 STM32 的二进制对话

串口是整个系统的"神经系统"：接收 IMU 姿态数据和控制模式，发送瞄准角度和开火指令。

**为什么用二进制协议而不是 JSON/Protobuf？**
- STM32 端解析 JSON 开销太大，二进制可直接 memcpy 到结构体
- 带宽和延迟：一个完整的 ReceivePacket 只有 ~20 字节，JSON 至少要 200 字节
- 确定性：二进制协议的解析时间是常数

**协议详解（`hardware/serial/serial_types.hpp`）：**

```
接收包 (STM32 → Vision):
┌──────┬──────┬──────────────────────────┬──────┐
│ 0xAA │ mode │ q[4](w,x,y,z float32×4) │ 0x5A │
│  1B  │  1B  │          16B             │  1B  │
└──────┴──────┴──────────────────────────┴──────┘

发送包 (Vision → STM32):
┌──────┬──────┬───────────┬──────────┬───────────┬────────┬────────┬─────────┬──────┐
│ 0xBB │found │fire_advice│target_yaw│target_pitch│  vx    │   vy   │nav_state│ 0x5B │
│  1B  │  1B  │    1B     │ f32×4B   │  f32×4B   │ f32×4B │ f32×4B │   1B    │  1B  │
└──────┴──────┴───────────┴──────────┴───────────┴────────┴────────┴─────────┴──────┘
```

**`__attribute__((packed))` 的作用：**
防止编译器在结构体成员之间插入填充字节。没有此属性，`ReceivePacket` 的大小可能从 19 变为 24 字节，导致与 STM32 的通信错位。

**UserSerial 单例模式的设计理由：**
```
系统中只有一根串口线连接 STM32
    ↓
全局只需要一个 UserSerial 实例
    ↓
使用单例模式，避免:
  - 多次打开同一串口 → 设备忙错误
  - 多处持有不同实例 → 状态不一致
  - 线程间传递串口对象 → 生命周期管理复杂
```

**四元数时间插值算法（UserSerial::q(timestamp)）：**
```
核心问题：相机曝光时刻与IMU采样时刻不同步
解决方案：
  1. Serial线程持续存储 [quaternion, timestamp] 到 SPSCQueue(5000)
  2. Armor线程拿到一帧图像后，用图像的 timestamp 调用 serial.q(frame_timestamp)
  3. 在队列中找到 frame_timestamp 前后的两个四元数
  4. 线性插值（或 Slerp）得到相机曝光时刻的准确姿态

时间线：
  IMU₁(t1)         相机曝光(t_frame)        IMU₂(t2)
    │                   │                      │
    └───────────────────┼──────────────────────┘
                        │
              q_interp = lerp(q1, q2, (t_frame-t1)/(t2-t1))
```

**💎 硬知识：源码中的实际插值算法——SLERP + 双元素滑动窗口**

文档中描述的"线性插值"是一个简化说法。实际代码使用的是 **SLERP（球面线性插值）**，配合一个精巧的双元素滑动窗口：

```
算法数据结构（user_serial.cpp）：
  data_ahead_  = {q1, t1}   ← 较早的采样（时间戳 < 目标时间戳）
  data_behind_ = {q2, t2}   ← 较新的采样（时间戳 ≥ 目标时间戳）
  data_queue_  = SPSCQueue<QuaternionWithTimestamp>(5000)
```

**滑动窗口推动逻辑：**
```cpp
// 当目标时间戳超过当前窗口的"新端"时，窗口向前滑动
while (data_behind_.timestamp < target_timestamp) {
    data_ahead_  = data_behind_;          // 窗口前移半步
    ptr = data_queue_.front();             // 从队列取下一个元素
    if (ptr == nullptr) break;            // 队列为空，无法继续前移
    data_behind_ = *ptr;                  // 新元素成为窗口的"新端"
    data_queue_.pop();
}
```

**SLERP 执行细节：**
```cpp
// 1. 计算插值系数 k ∈ [0, 1]
double k = (target_time - t_ahead) / (t_behind - t_ahead);
k = std::max(0.0, std::min(1.0, k));  // 钳制——防止外插抖动

// 2. 最短路径修正：如果两个四元数点积为负，翻转其中一个
//    q 和 -q 表示同一个旋转，但 SLERP 走的是球面上的弧
//    点积为负意味着当前弧长 > π（走的"远路"），翻转后弧长 < π（"近路"）
if (q_ahead.dot(q_behind) < 0)
    q_behind.coeffs() = -q_behind.coeffs();

// 3. 执行 SLERP 并归一化
return q_ahead.slerp(k, q_behind).normalized();
```

**为什么是 SLERP 而不是简单线性插值（Lerp）？**

| 方法 | 公式 | 角速度 | 问题 |
|------|------|--------|------|
| Lerp | `(1-k)·q1 + k·q2`（再归一化） | 非均匀 | 在两端的角速度慢，中间快 |
| SLERP | `sin((1-k)·θ)/sin(θ)·q1 + sin(k·θ)/sin(θ)·q2` | **恒定** | 计算量稍大，但角速度均匀 |

对旋转进行插值时，**恒定的角速度**至关重要——如果插值角速度不均匀，视觉系统的 yaw/pitch 输出会出现周期性波动，导致云台"抽搐"。

**边界情况处理：**
1. **首次调用**（data_ahead_ 未初始化）：直接返回 data_behind_.quaternion，不做插值
2. **目标时间戳在窗口之外**：k 被钳制到 [0,1]，返回边界值
3. **时间戳间隔过短**（< 1ns）：返回归一化的 q_behind，防止除零
4. **串口重连**：清空整个 data_queue_，丢弃所有旧数据

**任务清单：**
- [ ] 阅读 `hardware/serial/user_serial.hpp` 和 `.cpp`，画出 `receiveSerial()` 和 `sendPacketToSerial()` 的流程图
- [ ] 理解 `SPSCQueue<QuaternionWithTimestamp>` 如何保证线程安全
- [ ] 用示波器或逻辑分析仪观察串口 TX/RX 的实际波形（如果有条件）

### 3.3 SPSCQueue — 无锁队列的精髓

项目中最关键的并发原语是 `rigtorp::SPSCQueue<T>`。

**为什么是 SPSC 而不是 MPMC？**
- 相机线程是唯一的 Producer，视觉线程是唯一的 Consumer → 单生产者单消费者
- SPSC 可以实现**完全无锁**（无 CAS 循环），延迟确定且极低
- MPMC 队列需要使用锁或复杂的无锁算法，延迟不可预测

**为什么 Capacity=2？**
```
Capacity=2 的深意：
  Producer写入 slot[0] → Consumer读取 slot[0]
  Producer写入 slot[1] → Consumer读取 slot[1]
  Producer写入 slot[0] → Consumer读取 slot[0]（覆盖）

效果：Consumer 总是拿到最新的一帧，旧帧被自动丢弃
→ 这是"最新帧优先"策略，而非"全部帧都要处理"
→ 视觉算法跟不上相机帧率时，自动跳帧而不是累积延迟
```

**💎 硬知识：SPSCQueue 的内存序（Memory Order）分析**

这是整个项目中最硬核的并发知识点。SPSCQueue 之所以能实现"完全无锁"，靠的是对 CPU 缓存一致性和内存序的精确控制。

**缓存行（Cache Line）对齐——防止伪共享：**

```cpp
// 源码中的实际布局（SPSCQueue.h 第 254-257 行）
alignas(kCacheLineSize) std::atomic<size_t> writeIdx_ = {0};   // Cache Line 0
alignas(kCacheLineSize) size_t readIdxCache_          = 0;     // Cache Line 1
alignas(kCacheLineSize) std::atomic<size_t> readIdx_  = {0};   // Cache Line 2
alignas(kCacheLineSize) size_t writeIdxCache_         = 0;     // Cache Line 3
```

每个字段独占一个 64 字节的缓存行。如果 `writeIdx_` 和 `readIdx_` 在同一个缓存行中，Producer 写 `writeIdx_` 会导致 Consumer 的 `readIdx_` 缓存行被无效化（反之亦然），即使它们访问的是不同的变量。这就是**伪共享（False Sharing）**——两个核心在争抢同一个缓存行的所有权，即使它们操作的是不同数据。

**为什么需要本地缓存（Cached Index）？**

Producer 侧的 `readIdxCache_` 是 `readIdx_` 的本地副本。关键洞察：

```
Producer 在每次 push 时只需要知道"队列是否满了"
→ 如果上一次检查时队列没满，而且 Consumer 不可能"反读"（readIdx 只增不减）
→ 那么只要 readIdxCache_ != nextWriteIdx，队列就一定没满
→ 不需要每次 push 都去读 Remote CPU 缓存中的 readIdx_！
```

只有当日 `readIdxCache_ == nextWriteIdx`（本地缓存说满了），才需要做一次 `memory_order_acquire` 的原子加载，去验证 Consumer 是否真的已经读走了数据。

**内存序的精确语义——以 `try_emplace` 为例：**

```cpp
// 步骤 1: relaxed 读取 writeIdx（只有 Producer 自己写它，无需同步）
auto const writeIdx = writeIdx_.load(std::memory_order_relaxed);

// 步骤 2: 计算 nextWriteIdx...

// 步骤 3: 仅在本地缓存显示"可能满了"时才做 acquire 加载
if (nextWriteIdx == readIdxCache_) {
    readIdxCache_ = readIdx_.load(std::memory_order_acquire);  // ← 关键！
    // acquire 保证：在此之后的读写不会被重排到此之前
    // 作用：如果读到 Consumer 更新后的 readIdx_，
    //       那么 Consumer 在更新 readIdx_ 之前对 slot 的析构（pop）一定对 Producer 可见
    if (nextWriteIdx == readIdxCache_) return false;  // 真的满了
}

// 步骤 4: placement-new 构造对象到 slot
new (&slots_[writeIdx + kPadding]) T(std::forward<Args>(args)...);

// 步骤 5: release 发布 writeIdx
writeIdx_.store(nextWriteIdx, std::memory_order_release);
// release 保证：在此之前的所有写入（placement-new）不会被重排到此之后
// 作用：Consumer 通过 acquire 读到新的 writeIdx_ 时，
//       一定也能看到完整的 T 对象
```

**Consumer 侧的对称设计：**

```cpp
// front() 中：
auto const readIdx = readIdx_.load(std::memory_order_relaxed);
if (readIdx == writeIdxCache_) {
    writeIdxCache_ = writeIdx_.load(std::memory_order_acquire);  // 仅在"可能空"时同步
    // acquire 保证：能看到 Producer release-store 之前写入的完整对象
}

// pop() 中：
slots_[readIdx + kPadding].~T();  // 先析构
readIdx_.store(nextReadIdx, std::memory_order_release);  // 再发布
// release 保证：析构在 Producer 看到新的 readIdx_ 之前完成
```

**内存序配对总结：**

```
Producer 侧:                           Consumer 侧:
writeIdx_.store(release)  ───对应──→  writeIdx_.load(acquire)  [在 front() 中]
readIdx_.load(acquire)    ←──对应───  readIdx_.store(release)   [在 pop() 中]
```

这是经典的 **Release-Acquire 配对**，构成了 happens-before 关系。没有使用更重的 `memory_order_seq_cst`（顺序一致性），因为 SPSC 场景不需要全局顺序。

**性能影响：**
- 热路径（队列非空非满）：每个 push/pop 只做 1 次 relaxed load + 1 次 release store（共约 2-3ns）
- 冷路径（队列边界）：多做 1 次 acquire load（跨核心通信，约 50-100ns）
- 对比 `std::mutex`：无竞争时约 20ns，有竞争时可达微秒级

**任务清单：**
- [ ] 阅读 `rm_utils/SPSCQueue/SPSCQueue.h`，理解 `writeIdx` 和 `readIdx` 的原子操作
- [ ] 在纸上模拟：Producer 写 5 帧，Consumer 读 3 帧，capacity=2 时的内存布局变化
- [ ] 🆕 对比 `memory_order_relaxed`、`acquire`、`release` 的差异：写一个小程序，用不同内存序测试原子变量的行为

---

## 第4周：IMU 四元数与坐标变换 — 空间的魔法

> 这是整个系统中最需要数学基础的部分。如果你能彻底理解这一章，后面的检测、跟踪、弹道都将水到渠成。

### 4.1 为什么需要多个坐标系

在现实世界中，装甲板的位置由很多因素共同决定：
- 相机看到了装甲板在图像中的位置（像素坐标）
- 相机安装在云台上，云台在旋转（相机→云台的偏移）
- 机器人在运动，地面不平（IMU 的姿态在变化）
- 子弹飞行受重力影响（需要知道目标的世界坐标）

**单一坐标系无法处理所有这些变换！** 因此我们定义了 5 个坐标系（详见 `auto_aim/armor_track/README.MD`）：

```
装甲板{A} ──P_n_P──> 相机{C} ──手眼标定──> 云台{G} ──相似变换──> 世界{W}
                                                              ↑
                                                    IMU体{I} ─┘
```

**关键洞察：世界坐标系是"稳定的"**
- 世界坐标系的 Z 轴始终竖直向上（重力方向）
- 不管机器人怎么晃、云台怎么转，世界坐标系不变
- 弹道解算必须在世界坐标系中进行（因为重力在世界坐标系中是恒定向下的）

### 4.2 四元数的直观理解

很多教程用复数来引入四元数，但这里给一个更直接的工程视角：

**旋转矩阵** = 9 个数，有 6 个约束（正交性），实际自由度只有 3
**欧拉角** = 3 个数，但有万向节锁
**四元数** = 4 个数，有 1 个约束（单位长度），实际自由度 3 —— 完美的 3 DOF 表示

```cpp
// 四元数 q = (w, x, y, z) 表示绕轴 (x,y,z) 旋转 2·acos(w) 弧度
Eigen::Quaterniond q;
q = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ());  // 绕Z轴旋转45°
// q = (cos(π/8), 0, 0, sin(π/8)) = (0.924, 0, 0, 0.383)

// 四元数旋转向量的正确姿势: v' = q * v * q.conjugate()
Eigen::Vector3d v(1, 0, 0);
Eigen::Vector3d v_rotated = q * v;  // Eigen 重载了 * 操作符！
```

### 4.3 相似变换的数学推导（核心中的核心）

这是本项目最精妙的一处设计。问题是：

> **已知**：IMU 直接测量的是 `R_imubody2world`（IMU 芯片相对于世界的姿态）
> **需要**：`R_gimbal2world`（云台相对于世界的姿态）
> **困难**：云台和 IMU 之间有固定的安装偏差 `R_gimbal2imubody`

**公式：`R_gimbal2world = R_gimbal2imubodyᵀ · R_imubody2world · R_gimbal2imubody`**

**为什么叫"相似变换"？** 这不是矩阵相似（`P⁻¹AP`），而是在李群 SO(3) 上的共轭操作：

```
步骤拆解：
1. R_gimbal2imubody:    把云台的坐标轴旋转变到 IMU 的坐标轴上
                         （例如云台的X轴→IMU的Y轴方向）
2. R_imubody2world:     在 IMU 坐标系下，施加 IMU 测量到的世界旋转
3. R_gimbal2imubodyᵀ:   把旋转后的结果转回云台坐标系

直观理解：
  假设你在一个旋转的房间里（IMU 在旋转），
  你想知道门把手（云台）在世界中的方向。
  
  1. 先测量门把手在房间中的固定位置（R_gimbal2imubody）
  2. 房间旋转了（R_imubody2world）
  3. 门把手跟着房间一起旋转，你需要把这个旋转投影到门把手的轴上
  
  这就是相似变换做的事情。
```

**任务清单：**
- [ ] 在纸上推导：给定 `R_imubody2world` = 绕 Z 轴转 30°，`R_gimbal2imubody` = 绕 X 轴转 90°，计算 `R_gimbal2world`
- [ ] 阅读 `ArmorPose::set_R_gimbal2world()` 的实现，与推导对比
- [ ] 思考：如果 IMU 安装在云台正上方，`R_gimbal2imubody` 是什么？如果是倾斜 45° 安装呢？

### 4.4 YPD 球坐标系 — 从 XYZ 到射击角度

经过坐标变换后，我们得到了装甲板在世界坐标系中的 `(x, y, z)`。但云台需要的是**偏航角和俯仰角**：

```
Yaw (偏航角):   φ = atan2(y, x)           — 水平方向的方位角
Pitch (俯仰角): θ = atan2(z, √(x²+y²))    — 相对于水平面的仰角
Distance:       d = √(x²+y²+z²)            — 到目标的直线距离
```

**为什么不是 `atan2(z, x)`？**
因为俯仰角描述的是目标相对于**水平面**的高度，而不是在 XZ 平面内的角度。如果目标正上方（x=0, y=0, z=5），正确的 pitch 应该是 90°（正上方），而不是 `atan2(5, 0)` 的未定义行为。

**YPDA 状态空间（EKF 中使用的 4 维状态）：**
```
x = [yaw, pitch, distance, angle]
       ↑      ↑       ↑        ↑
     方位角  俯仰角   距离   装甲板自身旋转角度
```

**为什么 EKF 用 YPD 而不是 XYZ？**
- YPD 是**观测空间**的自然表示：相机测角精度高，测距精度低
- XYZ 是等方向性的，而视觉测量的不确定性是角度的（远处一个像素跨度更大）
- 在 YPD 空间中，测量噪声矩阵 R 更接近对角阵（各维度独立）

**任务清单：**
- [ ] 手动计算几个点的 YPD：`(1,0,0)`、`(1,1,0)`、`(1,1,1)`
- [ ] 阅读 `rm_utils/algorithm/math_tools.cpp` 中的 `xyz2ypd()` 和 `ypd2xyz()`
- [ ] 理解雅可比矩阵的作用：为 EKF 的更新步骤提供线性化

---

## 第5周：装甲板检测 — 从像素到目标

### 5.1 检测流水线全景

```
输入: BGR 图像 (1440×1080, 60fps)
  │
  ├─ Step 1: 颜色空间变换 BGR → Gray
  │
  ├─ Step 2: 二值化 (threshold → 提取亮区域 = 灯条)
  │
  ├─ Step 3: findContours → 筛选轮廓 (面积、长宽比、实心率)
  │     └─ 得到: vector<LightBar>
  │
  ├─ Step 4: 灯条配对 (几何约束)
  │     ├─ 两个灯条颜色相同
  │     ├─ 角度差 < max_angle_error (通常 ~12°)
  │     ├─ 高度比 < max_height_ratio (通常 ~1.5)
  │     └─ 装甲板宽高比在合理范围
  │     └─ 得到: vector<Armor>
  │
  ├─ Step 5: 提取数字 ROI → ONNX 推理 → 分类
  │     └─ 过滤置信度 < 阈值的候选
  │
  └─ 输出: vector<Armor> (包含 4 个角点、分类结果、置信度)
```

### 5.2 LightBar — 灯条的数学建模

装甲板由**两个 LED 灯条**组成。灯条在图像中呈现为细长的亮矩形。

```cpp
struct LightBar {
    cv::Point2f center, top, bottom;  // 中心、上端点、下端点
    double angle;                      // 灯条倾斜角度
    double length, width, ratio;       // 几何属性
    cv::RotatedRect rotated_rect;      // 旋转矩形
    std::vector<cv::Point2f> points;   // [top, bottom]
};
```

**灯条筛选的几何约束（为什么这样设计）：**

| 约束 | 意义 | 反例 |
|------|------|------|
| `min_aspect_ratio > 1.5` | 灯条是细长的 | 方形光斑不是灯条 |
| `min_solidity > 0.4` | 灯条区域填充率 | 镂空的轮廓不是灯条 |
| `angle_error (与垂线的偏差)` | 灯条近似竖直 | 横着的亮条可能是地面反光 |

### 5.3 Armor — 装甲板的结构

一对灯条组成一个装甲板候选：

```cpp
struct Armor {
    LightBar left, right;               // 左右灯条
    std::vector<cv::Point2f> points;    // 4个角点: 左上→右上→右下→左下
    //                                    (此顺序对应 solvePnP 的 object_points)
    cv::Point2f center;                 // 装甲板中心
    ArmorType type;                     // BIG / SMALL
    std::string number;                 // 识别出的数字 "1"~"5", "sentry"...
    float confidence;                   // 分类置信度
};
```

**角点顺序的重要性：**
```cpp
// 3D模型点 (world coordinates)             2D图像点 (image coordinates)
BIG_ARMOR_POINTS = {                        armor.points = {
    (0,  W/2,  H/2),  // 左上                  left.top,     // 左上
    (0, -W/2,  H/2),  // 右上                  right.top,    // 右上
    (0, -W/2, -H/2),  // 右下                  right.bottom, // 右下
    (0,  W/2, -H/2)   // 左下                  left.bottom   // 左下
};                                         };
```

**3D 点和 2D 点必须一一对应**，这是 `solvePnP` 能正确工作的前提。装甲板坐标系的原点定义在装甲板中心，X 轴垂直于装甲板平面（指向外）。

### 5.4 ONNX 数字识别

系统使用预训练的 LeNet/MLP 模型（ONNX 格式），通过 OpenCV DNN 模块推理：

```cpp
// 关键步骤
cv::dnn::Net net = cv::dnn::readNetFromONNX("model/lenet.onnx");
cv::Mat blob = cv::dnn::blobFromImage(number_img, 1.0/255.0, Size(28,28));
net.setInput(blob);
cv::Mat output = net.forward();  // 输出各类别的概率
```

**为什么用 LeNet 而不是更复杂的网络？**
- 数字图像极小（~28×28），大网络会过拟合
- 推理速度：LeNet < 1ms，ResNet > 10ms——60fps 下每帧只有 16ms 总预算
- OpenCV DNN 对 ONNX 的支持在简单模型上稳定，复杂模型可能有算子兼容问题

**💎 硬知识：LightCornerCorrector 的梯度搜索算法**

这是整个检测流水线中最精巧的细节优化。`solvePnP` 对输入角点的精度极其敏感——1 像素的角点误差可能在 5 米外产生 5-10cm 的位置误差。`LightCornerCorrector` 通过**局部梯度搜索**将角点精度提升到亚像素级别。

**算法流程（`light_corner_corrector.cpp` 第 115-180 行）：**

```
Step 1: 局部二值化（大津法）
  在灯条 ROI 上用 cv::THRESH_OTSU 重新二值化
  → 不同于全局的固定阈值（binary_thres_=90），大津法自动适应局部亮度
  → 找出 ROI 内最大的轮廓 → 更新灯条的旋转矩形

Step 2: 梯度搜索——找灯条端点
  对每个端点方向（top/bottom）：
    - 生成垂直于灯条轴的搜索方向 perp_axis
    - 在灯条半宽度范围内采样 (2*width/2 + 1) 条搜索线
    - 每条线从灯条长度的 40% 位置开始，搜索到 60% 位置
      （搜索范围只在灯条中心区域，避开已经可能准确的端点）
    - 步长 = 0.5 像素（亚像素精度！）

Step 3: 亮度跳变检测
  沿搜索线移动，每一步：
    prev_val = 上一个像素的灰度值（双线性插值）
    cur_val  = 当前像素的灰度值（双线性插值）
    diff = prev_val - cur_val
    
    寻找 diff 最大且 prev_val > ROI 灰度均值的点
    → "灯条内部是亮的 → 边缘外是暗的"：所以 prev_val - cur_val 最大处就是灯条边界
    → prev_val > mean_val 确保我们确实在灯条内部（而非背景噪声）

Step 4: 聚合所有搜索线的候选点
  多条搜索线会找到多个候选角点
  → 取平均位置作为最终角点坐标

Step 5: 降级方案
  如果找不到满足条件的梯度跳变点：
  → 回退到几何端点：center + axis * length * 0.5
  （即从中心沿轴线方向走到端点——纯几何估算）
```

**为什么搜索范围是 40%-60% 而不是整个灯条长度？**
- 灯条的几何中心已经由 `cv::minAreaRect` 给出，相对准确
- 搜索从中心附近开始向外，可以追踪灯条的边界
- 如果从端点开始搜索，端点本身就是我们要修正的目标，会引入偏差

**颜色验证机制（`findLights()` 第 166-204 行）：**

在灯条轴线上沿 `top→bottom` 方向均匀采样 10 个点：
```cpp
// 红色目标：累加 (R - B) 差值
diff_sum += (pixel[2] - pixel[0]);  // BGR 格式：[B=0, G=1, R=2]

// 蓝色目标：累加 (B - R) 差值
diff_sum += (pixel[0] - pixel[2]);

// 如果 diff_sum <= 0，丢弃该灯条（颜色特征不符）
```

这是一种轻量级的颜色验证——它不是对整个图像做颜色分割，而是对已经通过几何筛选的候选灯条做颜色确认。这样做远比全图颜色分割高效。

**任务清单：**
- [ ] 运行 `./base`，观察 debug 窗口中的检测结果（灯条框、装甲板框、数字标签）
- [ ] 修改 `config/auto_aim.yaml` 中的 `binary_threshold`（±20），观察对检测的影响
- [ ] 阅读 `ArmorDetector::findLights()` 和 `findArmors()` 的实现
- [ ] 🆕 阅读 `LightCornerCorrector::lightbar_points_corrector()` 的完整实现，画出梯度搜索的流程图
- [ ] 🆕 思考：如果灯条被部分遮挡（如被其他机器人挡住一半），梯度搜索是否会失效？有什么改进方案？

---

## 第6周：位姿解算与目标跟踪 — 知道敌人在哪

### 6.1 PnP 位姿解算

PnP（Perspective-n-Point）问题：已知 3D 空间中的 N 个点和它们在图像上的 2D 投影，求相机相对于这组 3D 点的位姿。

本项目使用 **IPPE**（平面目标）方法，因为装甲板的 4 个角点共面。

```
算法核心(cv::solvePnP IPPE):
输入: 4个3D点、4个2D点、相机内参K、畸变系数D
输出: rvec(旋转向量), tvec(平移向量)

流程:
1. 计算单应矩阵 H (平面到平面的映射)
2. 从 H 分解出两个可能的 (R, t)  — 平面的两个面
3. 通过重投影误差选择正确的那个
4. 返回 rvec, tvec

关键: tvec 就是装甲板中心在相机坐标系下的 3D 坐标！
```

**为什么返回的是 rvec（旋转向量）而不是旋转矩阵？**
旋转向量只有 3 个参数（轴×角度），而旋转矩阵有 9 个参数带 6 个约束。OpenCV 选择返回更紧凑的表示，后续通过 `cv::Rodrigues(rvec, R)` 转为矩阵。

### 6.2 完整的坐标变换链

```cpp
// Step 1: PnP → 装甲板在相机坐标系
cv::solvePnP(armor_points, armor.points, K, D, rvec, tvec, false, SOLVEPNP_IPPE);
cv::cv2eigen(tvec, xyz_in_camera);           // 位置
Eigen::Matrix3d R_armor2camera = Rodrigues(rvec); // 姿态

// Step 2: 手眼标定 → 装甲板在云台坐标系
xyz_in_gimbal = R_camera2gimbal * xyz_in_camera + t_camera2gimbal;
//                ↑ 相机到云台的旋转    ↑ 相机原点在云台坐标系下的位置

// Step 3: 相似变换 → 装甲板在世界坐标系
xyz_in_world = R_gimbal2world * xyz_in_gimbal;
//              ↑ 通过 IMU 四元数计算

// Step 4: 提取欧拉角（Z-Y-X顺序）
ypr_in_world = eulers(R_armor2world, 2, 1, 0);  // yaw, pitch, roll

// Step 5: 转换为射击用的球坐标
ypd_in_world = xyz2ypd(xyz_in_world);  // yaw, pitch, distance
```

**任务清单：**
- [ ] 阅读 `ArmorPose::GetArmorPose()` 的完整实现
- [ ] 理解 `optimize_yaw()` 的作用（为什么 solvePnP 返回的 yaw 可能不准确，如何通过重投影误差搜索最优 yaw）
- [ ] 计算一个实际例子：给定相机内参、装甲板图像坐标，估算 3D 位置

### 6.3 扩展卡尔曼滤波（EKF）

标准卡尔曼滤波（KF）假设系统是**线性的**。但我们的观测函数（从 YPD 映射回 XYZ 再映射到图像）是非线性的。因此使用**扩展**卡尔曼滤波，通过对非线性函数进行一阶泰勒展开（雅可比矩阵）来近似。

**本项目 EKF 的四维状态：**
```
x = [yaw, pitch, distance, angle]
```

**预测步骤（运动模型）：**
```
x̂ₖ₊₁ = F · xₖ          (常速度模型)
Pₖ₊₁ = F · Pₖ · Fᵀ + Q  (协方差传播)
```

**更新步骤（观测模型）：**
```
K = P · Hᵀ · (H · P · Hᵀ + R)⁻¹   (卡尔曼增益)
x = x + K · (z - h(x))             (状态更新)
P = (I - K · H) · P                (协方差更新)
```

**关键设计决策 — 为什么在 YPD 空间做 EKF？**

| 特性 | XYZ-EKF | YPD-EKF（本项目） |
|------|---------|-------------------|
| 状态维度 | 6 (x,y,z,vx,vy,vz) | 4 (yaw,pitch,dist,angle) |
| 观测模型 | 非线性，需要相机投影 | 近似线性，直接测量 |
| 测量噪声 | 各向异性（远处更大） | 对角阵（yaw/pitch 噪声恒定） |
| 运动模型 | 线性 | 近似线性 |
| 数值稳定性 | 好 | 好 |

**NIS 卡方检验（chi-squared gating）：**
```cpp
nis = (z - h(x))ᵀ · (H·P·Hᵀ + R)⁻¹ · (z - h(x))
if (nis > chi2_threshold(4, 0.95))  // 4自由度, 95%置信度
    reject_measurement();            // 拒绝这个野值
```

这是防止错误检测破坏跟踪的关键机制。当装甲板检测错误（如把灯条配对错误）时，NIS 会急剧增大，EKF 拒绝这个测量值，维持在预测状态。

**任务清单：**
- [ ] 阅读 `rm_utils/algorithm/extended_kalman_filter.hpp`，理解 `predict()` 和 `update()` 的参数含义
- [ ] 阅读 `Target::update_ypda()` 的实现，理解 `h_armor_xyz()` 和 `h_jacobian()` 如何计算
- [ ] 用 Python/Matlab 模拟一个 YPD-EKF：给定真实轨迹 + 噪声观测，观察滤波效果

### 💎 硬知识：源码中的真实 EKF——11 维旋转中心模型

> **重要纠正**：上面描述的 4 维 YPDA-EKF 是简化的教学版本。源码中实际使用的是**11 维状态向量**，跟踪的是**机器人旋转中心**而非装甲板的 YPDA 坐标。这是两个完全不同的建模思路！

**为什么跟踪旋转中心而不是直接跟踪装甲板位置？**

装甲板附着在旋转的机器人上，在 YPDA 空间中的运动是高度非线性的（周期性旋转 + 平移）。如果直接跟踪装甲板位置：
- 当一块板转出视野、另一块板转入时，需要切换跟踪目标
- 同一机器人上不同装甲板的 YPDA 坐标差异巨大，但它们的**旋转中心**是同一个点

→ **跟踪旋转中心** 可以整合同一机器人上所有装甲板的观测信息！

**11 维状态向量（`armor_target.cpp` 第 48-89 行）：**

```
x = [center_x, vx, center_y, vy, center_z, vz, yaw, omega, r, l, h]
      ↑         ↑    ↑         ↑    ↑         ↑    ↑    ↑     ↑  ↑  ↑
      |         |    |         |    |         |    |    |     |  |  |
     机器人中心位置+速度(x)      位置+速度(y)      位置+速度(z)   朝向  角速度 半径 延伸 高度偏移
```

| 分量 | 索引 | 含义 | 运动模型 |
|------|------|------|---------|
| `center_x, vx` | 0, 1 | 旋转中心 X 坐标 + 速度 | 恒速模型 (CV) |
| `center_y, vy` | 2, 3 | 旋转中心 Y 坐标 + 速度 | 恒速模型 (CV) |
| `center_z, vz` | 4, 5 | 旋转中心 Z 坐标 + 速度 | 恒速模型 (CV) |
| `yaw` | 6 | 机器人自身的朝向角 | 恒速模型 (CV) |
| `omega` | 7 | 绕垂直轴的旋转角速度 (rad/s) | 恒速模型 (CV) |
| `r` | 8 | 旋转半径（旋转中心到装甲板的距离）| 随机游走 (I) |
| `l` | 9 | 侧装甲板的额外半径延伸 (用于 4 板机器人) | 随机游走 (I) |
| `h` | 10 | 四板机器人的垂直高度偏移 (侧板上下高差) | 带微小噪声的随机游走 |

**状态转移矩阵 F（11×11）——离散恒速模型：**

```
F = 块对角矩阵:
  [1 dt]  用于 (x,vx), (y,vy), (z,vz)
  [0  1]
  
  [1 dt]  用于 (yaw, omega)
  [0  1]
  
  [1]     用于 r, l  (恒等——半径不会自己变化)
  [1]     用于 h      (恒等——高度偏移不会自己变化)
```

**过程噪声 Q——离散白噪声加速度模型：**

```cpp
// 平移过程噪声强度
double v1 = (目标类型 == "outpost") ? 10 : 100;    // 前哨站慢速，普通目标可快速机动
// 旋转过程噪声强度  
double v2 = (目标类型 == "outpost") ? 0.1 : 400;   // 前哨站匀速旋转，普通目标可变速

// 离散 CV 噪声系数
a = dt⁴/4    // 位置-位置噪声
b = dt³/2    // 位置-速度交叉协方差
c = dt²      // 速度-速度噪声

// 例如 x 通道的 Q 块：
Q_xyzv = [a*v1, b*v1]
         [b*v1, c*v1]
         
// yaw 通道的 Q 块：
Q_yaw_w = [a*v2, b*v2]
          [b*v2, c*v2]
```

**物理含义**：`v1 = 100` 意味着我们假设目标机器人的加速度不确定性 σᵤ = √100 = 10 m/s²。对于前哨站（outpost），`v1 = 10` 反映其几乎静止不动（σᵤ = √10 ≈ 3.16 m/s²）。

**观测函数 h(x, armor_id)——从旋转中心到装甲板 YPDA：**

这是最精妙的设计。给定旋转中心状态 `x` 和装甲板编号 `id`，计算装甲板的世界坐标：

```
// 4 块装甲板的机器人（最普遍情况）
angle    = wrap(x[6] + id * 2π/4)           // 装甲板当前朝向 = 机器人朝向 + 装甲板偏移
use_lh   = (id == 1 || id == 3)              // 侧板有额外延伸
r        = use_lh ? x[8] + x[9] : x[8]       // 侧板半径更长

armor_x  = x[0] - r * cos(angle)             // 装甲板的世界X坐标
armor_y  = x[2] - r * sin(angle)             // 装甲板的世界Y坐标
armor_z  = x[4] + (use_lh ? x[10] : 0)       // 侧板有高度偏移

// 然后转换为球坐标
h(x, id) = [atan2(armor_y, armor_x),         // yaw (观测)
            atan2(armor_z, √(armor_x²+armor_y²)),  // pitch (观测)
            √(armor_x²+armor_y²+armor_z²),    // distance (观测)
            wrap(x[6] + id*2π/4)]             // armor_yaw (观测)
```

**雅可比矩阵 H 的链式求导：**

```
H(4×11) = H_armor_ypda(4×4) * H_armor_xyza(4×11)
           ↑                  ↑
      YPD对armor_xyz的导数    armor_xyz对状态的导数
```

`H_armor_xyza` 中关键项的推导：

```
d(armor_x)/d(yaw)  =  r * sin(angle)      // 因为 armor_x = cx - r·cos(angle)
d(armor_y)/d(yaw)  = -r * cos(angle)      // 因为 armor_y = cy - r·sin(angle)
d(armor_x)/d(r)    = -cos(angle)
d(armor_y)/d(r)    = -sin(angle)
d(armor_x)/d(cx)   = 1
d(armor_y)/d(cy)   = 1
d(armor_z)/d(cz)   = 1
```

这些偏导数将非线性观测函数线性化，使 EKF 能够将"观测到了装甲板在某个 yaw/pitch 方向"的信息反推回"机器人中心在哪里、朝向几何"的状态更新。

**自适应观测噪声 R——随观测几何变化：**

```cpp
double center_yaw = atan2(armor_y, armor_x);
double delta_angle = fabs(wrap(yaw_obs - center_yaw));  // 装甲板与中心方向的夹角

R = diag(
    0.004,                                        // yaw 观测噪声 (恒定)
    0.004,                                        // pitch 观测噪声 (恒定)
    log(|delta_angle| + 1) + 1,                   // ❗距离观测噪声——随视角增大
    log(distance + 1)/200 + 0.09                  // ❗装甲板朝向观测噪声——随距离增大
);
```

**关键洞察**：当从正前方看装甲板时（delta_angle ≈ 0），P3P 测距最准确（R[2] ≈ 1.0）。当从侧面看时（delta_angle 大），装甲板在图像中更窄，测距精度下降（R[2] 增大），EKF 自动降低距离观测的权重。

**NIS 卡方门控——拒绝错误检测：**

```cpp
nis = residual^T * S^(-1) * residual         // 归一化创新平方
if (nis > chi2_threshold(4, 0.95))          // 4自由度, 95%置信度阈值 = 9.488
    reject_measurement();

// 滑动窗口监控：如果过去 100 次更新中有 >40% 失败 NIS 门控
// → 目标模型发散 → 重置为 LOST 状态
```

**前哨站（Outpost）的特殊处理：**

前哨站有 3 块装甲板、3 层高度：
```
angle = x[6] + id * 2π/3          // 3 块板等角分布
az    = x[4] + (id-1) * 0.102     // 每层高度差 0.102m
```

一旦收敛且 `|omega| > 2 rad/s`，角速度被钳制到 ±2.51 rad/s（前哨站的理论最大转速约为 1 转/2.5 秒）。

**总结对比：教学版 vs 源码版 EKF**

| 维度 | 教学简化版 | 源码真实版 |
|------|-----------|-----------|
| 状态维度 | 4 (yaw, pitch, dist, angle) | **11** (旋转中心 + 速度 + 朝向 + 半径 + …) |
| 跟踪目标 | 单块装甲板的球坐标 | **机器人的旋转中心** |
| 装甲板切换 | 需要手动切换跟踪对象 | 自动——同一中心可观测所有板 |
| 观测函数 | 近似线性 (YPDA) | 非线性（旋转几何→YPDA） |
| 雅可比 | 简单 (xyz2ypd) | 链式法则 (旋转几何 × xyz2ypd) |
| 测量噪声 | 固定对角阵 | **自适应**（视角、距离相关） |

### 6.4 跟踪状态机

`ArmorTrack` 使用有限状态机（FSM）来管理跟踪生命周期：

```
                    ┌──────────────────────────────┐
                    │                              │
        ┌─── 连续N帧检测到 ──→  detecting ── 再检测到M帧 ──→ tracking ──┐
        │                           │                          │         │
      lost  ←── 连续K帧丢失 ── temp_lost ←── 连续L帧丢失 ──────────┘         │
        │                           │                          │         │
        └───────────────────────────┴──────────────────────────┘         │
                                                                         │
                        初始化 / 重置 ──────────────────────────────────┘
```

**每个状态的含义和策略：**

| 状态 | 含义 | 策略 |
|------|------|------|
| `lost` | 没有目标 | 累加检测计数，达到阈值后进入 detecting |
| `detecting` | 发现候选，确认中 | 要求连续多帧检测到同一目标才确认，过滤假阳性 |
| `tracking` | 稳定跟踪 | EKF 预测 + 更新 + NIS 门控，持续输出射击角度 |
| `temp_lost` | 短暂丢失 | 用 EKF 纯预测维持一段时间的输出，等待目标重新出现 |

**为什么需要 temp_lost 状态？**
- 装甲板可能被遮挡（机器人自身结构、其他机器人）
- 检测可能有偶发的漏检（光线变化、运动模糊）
- 如果一丢失就回到 lost，会导致频繁的检测确认过程，可能错过射击窗口

**任务清单：**
- [ ] 阅读 `ArmorTrack::track()` 和 `state_machine()` 的实现
- [ ] 画出 FSM 的状态转移图，标注每个转移的条件
- [ ] 修改 `min_detect_count` 和 `max_temp_lost_count`，观察对跟踪稳定性的影响

---

## 第7周：弹道解算与多线程架构 — 扣下扳机

### 7.1 弹道模型

本项目使用简化的弹道模型（无空气阻力）：

```
物理模型:
  水平方向: d = v₀ · cos(θ) · t
  竖直方向: h = v₀ · sin(θ) · t - ½g·t²

其中:
  v₀ = 子弹初速度 (28 m/s)
  d  = 目标水平距离 (m)
  h  = 目标竖直高度 (相对于枪口, m)
  g  = 重力加速度 (9.81 m/s²)
  θ  = 发射俯仰角 (待求解)
  t  = 飞行时间 (待求解)

联立求解:
  从水平方程:  t = d / (v₀·cos(θ))
  代入竖直方程: h = d·tan(θ) - g·d²/(2·v₀²·cos²(θ))
  
  利用 1/cos²(θ) = 1 + tan²(θ):
  h = d·tan(θ) - g·d²·(1+tan²(θ))/(2·v₀²)
  
  这是一个关于 tan(θ) 的二次方程！
```

**实际代码中通常用数值方法**（迭代或查表），因为二次方程的解析解在边界情况（如 d=0）下不稳定。

### 7.2 三种瞄准模式

`ArmorShoot` 根据目标旋转速度选择瞄准策略：

| 模式 | 触发条件 | 瞄准点 | 适用场景 |
|------|---------|--------|---------|
| `TRACK` | 转速 < low_threshold | 当前可见装甲板的中心 | 静止或低速旋转目标 |
| `COMING` | low < 转速 < high | 正在靠近射手的装甲板 | 中等转速，"来板"最优射击时机 |
| `HERO_CENTER` | 转速 > high | 旋转中心（而非装甲板表面） | 高速旋转（小陀螺），打旋转圆心 |

**为什么打旋转中心？**
```
高速旋转时，单块装甲板只在视野中停留极短时间
→ 无法在装甲板可见的时间内完成瞄准+发射
→ 改为瞄准旋转中心，持续发射
→ 子弹在某个时刻会命中经过该位置的装甲板
```

**COMING 模式的判断逻辑：**
```
装甲板围绕旋转中心匀速旋转
→ 计算每块装甲板的线速度方向
→ 选择正在"靠近"射手的装甲板（距离在减小）
→ 预判其到达最近点的时间，提前发射
```

**任务清单：**
- [ ] 阅读 `Trajectory` 的构造函数实现，理解弹道解算的数学
- [ ] 阅读 `ArmorShoot::chooseAimPoint()`，理解三种模式的切换逻辑
- [ ] 在 Python 中画出不同距离下所需的 pitch 角度曲线

### 7.3 多线程架构全景

```
main.cpp (主线程)
  │
  ├─ 创建 BS::thread_pool(6)
  │
  ├─ pool.detach_task( camera_thread_func )     ─┐
  ├─ pool.detach_task( usb_camera_thread_func )  ─┤ 生产者
  ├─ pool.detach_task( serial_thread_func )      ─┤ IO
  ├─ pool.detach_task( armor_thread_func )       ─┤ 消费者/处理器
  └─ pool.detach_task( ros2_thread_func )        ─┘ 辅助通信
```

**线程间数据流（完整版）：**

```
  ┌─────────────┐     SPSCQueue(2)     ┌──────────────┐
  │ HikCamera   │ ───────────────────→ │              │
  │ Thread      │     CameraFrame      │              │
  └─────────────┘                      │              │
                                       │   Armor      │    UserSerial     ┌──────────┐
  ┌─────────────┐     SPSCQueue(2)     │   Thread     │ ───────────────→ │  STM32   │
  │ UsbCamera   │ ───────────────────→ │              │   sendVision()   └──────────┘
  │ Thread      │                      │              │
  └─────────────┘                      │  detect()    │
                                       │  track()     │    sendNav()     ┌──────────┐
  ┌─────────────┐     UserSerial       │  shoot()     │ ←─────────────── │  ROS2    │
  │ Serial      │ ←────────── send ─── │              │                  │  Docker  │
  │ Thread      │ ────────── recv ───→ │  serial.q()  │                  └──────────┘
  └─────────────┘    quaternion        └──────────────┘
       │                                      │
       │ 250Hz send cycle                     │ display_add()
       │ continuous receive                   ↓
       ▼                               ┌──────────────┐
  ┌──────────┐                         │  Display     │
  │  STM32   │                         │  (SDL2)      │
  └──────────┘                         └──────────────┘
```

**任务清单：**
- [ ] 画出所有线程的启动顺序和依赖关系
- [ ] 理解 `g_shutdown` 如何实现优雅退出
- [ ] 理解 `CameraFrame` 的 `std::move` 语义：为什么用移动而不是拷贝

### 7.4 线程安全的层次设计

本项目使用**分层并发策略**，避免全局锁：

```
Layer 1: SPSCQueue — 无锁的点对点通信
  HikCamera → Armor: CameraFrame 的传递
  Serial内部: QuaternionWithTimestamp 的缓冲

Layer 2: 单例 — 天然的互斥（只有一个实例）
  UserSerial: 串口硬件的唯一访问入口
  Display: 显示窗口的唯一所有者

Layer 3: atomic — 无锁的共享状态
  g_shutdown: 全局退出信号
  SPSCQueue 内部的 writeIdx/readIdx

Layer 4: 线程池 — 避免线程创建/销毁开销
  BS::thread_pool: 6 个工作线程常驻
```

**为什么不用 `std::mutex`？**
- 互斥锁在低竞争时有 ~50ns 开销，高竞争时可达微秒级
- 视觉管线中每帧处理时间只有 ~15ms，不能浪费在锁等待上
- SPSCQueue 的无锁实现利用了 CPU 的原子指令（`std::atomic`），延迟在纳秒级

**任务清单：**
- [ ] 阅读 `SPSCQueue.h` 的 `try_push()` 和 `try_pop()` 实现
- [ ] 思考：如果相机帧率是 60fps，视觉处理是 30fps，SPSCQueue(2) 会丢失多少帧？这合理吗？

---

## 第8周：系统集成、标定与调试

### 8.1 相机标定

相机标定确定**内参矩阵 K** 和**畸变系数 D**：

```
K = [fx   0   cx]       fx, fy: 焦距（像素单位）
    [ 0  fy   cy]       cx, cy: 光心（主点）
    [ 0   0    1]

D = [k1, k2, p1, p2, k3]   径向畸变 + 切向畸变
```

**标定原理：**
- 拍摄多张不同角度、不同位置的棋盘格图片
- `cv::findChessboardCorners()` 提取角点的 2D 像素坐标
- 已知棋盘格的物理尺寸 → 3D 世界坐标
- `cv::calibrateCamera()` 求解 K 和 D

```bash
./base calibrate_camera
# 按照提示移动棋盘格，系统自动采集并计算
```

**任务清单：**
- [ ] 打印一张棋盘格（9×6 内角点），完成相机标定
- [ ] 验证标定结果：重投影误差应 < 0.5 像素
- [ ] 理解 fx, fy, cx, cy 的物理含义

### 8.2 手眼标定

手眼标定确定相机相对于云台的**外参**：

```
手眼标定的核心方程（AX = XB）：
  A: 云台的运动（记录的角度变化）
  B: 相机的运动（通过拍摄标定板计算）
  X: 待求解的 相机→云台 的变换矩阵（R_camera2gimbal, t_camera2gimbal）

直观理解：
  你移动你的头（云台），同时用眼睛（相机）观察一个固定的物体。
  通过头部的运动和眼睛中物体的运动，可以推算出眼睛在头部的位置。
```

```bash
./base calibrate_handeye
# 在相机前方放置 AprilTag 或棋盘格
# 旋转云台到多个不同角度，每个角度拍摄一张
# 系统自动计算外参
```

**`R_gimbal2imubody` 的手动填入：**
这个参数描述云台到 IMU 的安装姿态，目前是手动测量并填入 `config/hikcamera.yaml`。精确值可以通过类似手眼标定的方法自动标定（将云台旋转不同角度，记录 IMU 读数变化），但项目当前版本使用手动填入。

**任务清单：**
- [ ] 完成手眼标定，将结果填入 `config/hikcamera.yaml`
- [ ] 验证标定结果：在 `debug` 模式下查看重投影是否准确对齐
- [ ] 理解为什么 `t_camera2gimbal` 是"相机原点在云台坐标系下的坐标"

### 8.3 ROS2 UDP 通信

ROS2 线程通过 UDP 与 Docker 中的 ROS2 节点通信：

```
Vision系统 (Jetson/NUC)                Docker (ROS2)
┌──────────────┐     UDP:12344      ┌──────────────┐
│  Ros2_comm   │ ─────────────────→ │  ros2 node   │
│              │  裁判系统数据        │              │
│              │                    │              │
│              │     UDP:12345      │              │
│              │ ←───────────────── │              │
│              │  导航指令           │              │
└──────────────┘                    └──────────────┘
```

**为什么用 UDP 而不是 TCP？**
- 实时性 > 可靠性：丢失一帧裁判系统数据比等待 TCP 重传更好
- Docker 网络栈的 TCP 有额外开销
- 裁判系统数据和导航指令的更新频率高，丢失一帧影响很小

**任务清单：**
- [ ] 阅读 `rm_utils/ros2_comm/ros2_comm.hpp` 和 `.cpp`
- [ ] 理解 `ROS2_SEND_PACKET` 和 `ROS2_RECV_PACKET` 的数据结构
- [ ] 如果不需要 ROS2，在 `config/ros2.yaml` 中设置 `enable: false`

### 8.4 性能优化策略

**1. 内存池化（已在代码中实现）：**
```cpp
// ArmorDetector 在构造函数中预分配图像缓存
cv::Mat gray_;     // 重复使用，避免每帧分配
cv::Mat binary_;   // 同上
```

**2. 跳帧策略：**
SPSCQueue(2) 天然实现跳帧：视觉处理慢于相机采集时，自动丢弃旧帧，处理最新帧。

**3. 编译优化：**
```bash
cmake .. -DCMAKE_BUILD_TYPE=Release  # Release模式
cmake .. -DCMAKE_CXX_FLAGS="-march=native"  # 针对本地CPU优化指令集
```

**4. 性能瓶颈定位：**
在代码中使用 `std::chrono` 测量每个阶段的耗时：
```cpp
auto start = std::chrono::steady_clock::now();
auto armors = detector.ArmorDetect(frame);
auto detect_time = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start).count();
MAS_LOG_DEBUG("Detection: {}ms, found {} armors", detect_time, armors.size());
```

**任务清单：**
- [ ] 在 `armor_thread_func` 中添加各阶段耗时统计
- [ ] 画出耗时饼图：detect%, pose%, track%, shoot% 各占多少
- [ ] 尝试降低相机分辨率（如 1280×720→640×480），观察帧率和检测精度的变化

### 8.5 调试工具箱

| 工具 | 用途 | 使用方法 |
|------|------|---------|
| SDL2 Display | 可视化检测/跟踪结果 | 设置 `debug: true` |
| Plotter (UDP) | 绘制实时数据曲线 | 设置 `plotter_enable: true`，接收端监听 9870 端口 |
| MAS_LOG | 日志输出 | 查看 `logs/mas_vision.log`，修改 `spdlog::level` 控制详细度 |
| fps_map_ | FPS 统计 | 在 debug 模式下自动打印各模块帧率 |

**任务清单：**
- [ ] 配置 Plotter 接收端，绘制 yaw/pitch 角度的时间曲线
- [ ] 在 `auto_aim.yaml` 中开启 debug，观察检测和跟踪的中间结果
- [ ] 分析日志文件，确认各模块的帧率和异常信息

---

> 📘 **专题深入**：多线程架构、SPSCQueue 无锁队列原理、线程间数据流详见 [[MAS_Vision_多线程架构学习指南]]

---

## 附录 A：项目核心文件索引

| 如果你想理解... | 首先阅读...                                                        |
| --------- | -------------------------------------------------------------- |
| 程序入口和线程启动 | `apps/main.cpp`                                                |
| 装甲板数据结构   | `auto_aim/armor_types.hpp`                                     |
| 灯条检测算法    | `auto_aim/armor_detector/armor_detector.cpp`                   |
| 装甲板配对算法   | `auto_aim/armor_detector/armor_detector.cpp` → `findArmors()`  |
| ONNX 数字识别 | `auto_aim/armor_detector/number_classifier.cpp`                |
| PnP 位姿解算  | `auto_aim/armor_track/armor_pose.cpp` → `GetArmorPose()`       |
| 坐标变换      | `auto_aim/armor_track/armor_pose.cpp` → `set_R_gimbal2world()` |
| EKF 滤波器   | `rm_utils/algorithm/extended_kalman_filter.hpp`                |
| 目标跟踪 FSM  | `auto_aim/armor_track/armor_track.cpp` → `state_machine()`     |
| 弹道解算      | `rm_utils/algorithm/trajectory.cpp`                            |
| 三种瞄准模式    | `auto_aim/armor_shoot/armor_shoot.cpp` → `chooseAimPoint()`    |
| 串口协议      | `hardware/serial/serial_types.hpp`                             |
| 四元数插值     | `hardware/serial/user_serial.cpp` → `q(timestamp)`             |
| 无锁队列      | `rm_utils/SPSCQueue/SPSCQueue.h`                               |
| 视觉主循环     | `threads/armor_thread/armor_thread.cpp`                        |
| 串口收发循环    | `threads/serial_thread/serial_thread.cpp`                      |

## 附录 B：推荐学习顺序（如果时间有限）

1. **先读懂数据流**：`main.cpp` → `armor_thread.cpp`（知道数据怎么流动）
2. **再理解数据结构**：`armor_types.hpp` → `serial_types.hpp`（知道数据长什么样）
3. **深入检测**：`armor_detector.cpp`（知道怎么找到装甲板）
4. **攻克坐标变换**：`armor_pose.cpp` + README.MD（知道空间怎么算）
5. **掌握跟踪**：`armor_track.cpp` → `extended_kalman_filter.hpp`（知道怎么稳定跟踪）
6. **搞定射击**：`armor_shoot.cpp` → `trajectory.cpp`（知道怎么打中）
7. **理解并发**：`SPSCQueue.h` → `user_serial.cpp`（知道为什么这么快）

---

> **最后的建议**：这个项目是多人协作的 RoboMaster 视觉方案，代码中有大量注释和来自同济大学、中南大学等队伍的开源参考。遇到不理解的地方，先看注释，再看参考项目的原始代码，最后动手修改参数看效果。**"改一行代码，运行一次，观察一次"永远比"读十遍文档"学得快。**

---

## 附录 C：ROS2 通信与启动参数详解

### C.1 main 函数的命令行参数

`main(int argc, char *argv[])` 接收 0 或 1 个参数（`argv[1]`），对应三种模式：

| 命令 | 行为 |
|------|------|
| `./base`（无参数） | 正常模式：启动完整视觉流水线（6线程池 + 相机 + 串口 + 装甲板 + ROS2） |
| `./base calibrate_camera` | 相机内参标定：对称圆点标定板，按 `S` 保存图像（≥10张），按 `ESC` 计算并写入 YAML |
| `./base calibrate_handeye` | 手眼标定：需先完成相机标定，按 `S` 保存帧数据（≥3组），按 `Q` 计算外参矩阵 |

未知命令会打印帮助信息并退出。

### C.2 Docker ↔ 宿主机 UDP 端口映射

配置位于 `config/ros2.yaml`：

```yaml
ros2:
  enabled: true
  server_ip: "127.0.0.1"     # Docker 容器 IP（host 网络模式即本机回环）
  server_port: 8888          # Docker 端监听端口（mas_vision 发往这里）
  local_port: 8889           # 宿主机绑定端口（接收 Docker 的回复）
```

### C.3 为什么需要两个端口？

核心原因：**同一 `IP:Port` 在 Linux 内核中只能被一个 socket 绑定**。Docker 使用 host 网络模式时，容器与宿主机共享网络栈，两进程不能同时 `bind(127.0.0.1:8888)`。

UDP 是无连接协议，双方都需要被动接收数据（不是客户端-服务器模型，而是**对等通信**），因此必须各绑一个端口。一个端口的设计只能用 TCP（`connect()` 后 fd 独立），但 TCP 的握手开销不适合高频实时通信场景。

### C.4 ROS2 桥交换的数据

数据流：`STM32 ←→ mas_vision ←→ Docker ROS2 节点 ←→ 决策/导航`

**方向一：视觉 → Docker（上报）**

`ROS2_SEND_PACKET` 直接 `memcpy` 自串口收到的 `RefereePacket`，原封不动转发：

| 字段 | 用途 |
|------|------|
| `projectile_allowance_17mm` | 剩余弹丸量 — 判断是否还能开火 |
| `power_management_shooter_output` | 摩擦轮功率状态 — 射击就绪/热量限制 |
| `current_hp` | 自身血量 — 决策继续攻击还是撤退 |
| `outpost_HP` | 前哨站血量 — 战况评估 |
| `base_HP` | 基地血量 — 防守优先级判断 |
| `game_progess` | 比赛进程阶段 |

**方向二：Docker → 视觉（控制指令）**

`ROS2_RECV_PACKET` 从 ROS2 导航栈接收，写入 `SendPacket` 通过串口下发 STM32：

| 字段 | 用途 |
|------|------|
| `vx` | 底盘 X 方向目标速度 (m/s) |
| `vy` | 底盘 Y 方向目标速度 (m/s) |
| `nav_state` | 导航状态码（巡航/到达/避障/停止等） |

设计意图：视觉端专注**自瞄射击**，导航规划（路径搜索、避障、全局定位）交给 Docker 中的 ROS2 `nav2` 生态处理，两者通过 UDP 桥各司其职。
