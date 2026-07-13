# SP_Vision 学习指南（同济 SuperPower 25 赛季自瞄开源）

> 📘 **项目定位**：本笔记解析 [同济大学 SuperPower 战队 25 赛季自瞄开源项目 sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25)。这是一个**学习参考项目**，与本战队自研的 MAS_Vision 代码是两套独立体系。
> 📘 **最大亮点**：提出"轨迹视角下的自瞄理论"，用**自瞄轨迹规划器**替代传统分段决策逻辑，代码更简洁、调参更有章法、效果更好（14 rad/s、2m、300HP 靶机击杀约 10s）。
> 📘 **相关笔记**：[[MAS_Vision_学习指南]]（本战队自研视觉，可对照阅读）
> 📘 **专题分册**（后续深化）：[[SP_Vision_装甲板识别与解算详解]]、[[SP_Vision_整车估计EKF详解]]、[[SP_Vision_轨迹规划器详解]]、[[SP_Vision_打符与全向感知详解]]、[[SP_Vision_IO与多线程架构详解]]

---

## 目录

1. [概述](#1-概述)
2. [项目结构](#2-项目结构)
3. [软件架构与分层设计](#3-软件架构与分层设计)
4. [核心数据结构](#4-核心数据结构)
5. [自瞄完整数据流](#5-自瞄完整数据流)
6. [五大模块核心知识点](#6-五大模块核心知识点)
7. [轨迹视角下的自瞄理论](#7-轨迹视角下的自瞄理论)
8. [配置参数速查](#8-配置参数速查)
9. [概念关系图谱](#9-概念关系图谱)
10. [相关笔记](#10-相关笔记)

---

## 1. 概述

### 1.1 自瞄是什么

自瞄 = **针对移动装甲板目标的自动瞄准 + 自动火控软件**。操作手切到自瞄档后，自瞄接管云台控制权（预测敌方运动轨迹 + 弹道解算，驱动云台追踪）与发射机构控制权（判断开火时机）。目标是**短击杀时间、高命中率**。

### 1.2 自瞄的模块划分

各队自瞄经过多年演化，已收敛到一套通用的模块划分：

| 模块                     | 角色  | 职责                    | 运行位置          |
| :--------------------- | :-- | :-------------------- | :------------ |
| 装甲板识别器 detector        | 感知  | 从图像得到装甲板四点像素坐标 + 图案类别 | 小电脑（算力高）      |
| 目标状态估计器 tracker/target | 感知  | 由装甲板观测估计敌方整车运动状态      | 小电脑           |
| 决策器 planner/aimer      | 决策  | 预测轨迹，算最佳瞄准位置 + 开火时机   | 小电脑           |
| 控制器                    | 执行  | 驱动云台电机与发射机构           | 嵌入式单片机（实时、稳定） |

**为什么是分布式软件**：识别/估计/决策算法复杂、吃算力，跑在小电脑（NUC）；控制器要求硬实时和稳定，跑在单片机（C 板 STM32F407）。这种划分同时解耦了**视觉组**和**电控组**的职责分工。本项目 = 视觉组的全部代码 + 与电控的通信协议。

### 1.3 本项目相对往年的改进

| 版本               | 语言     | 帧率       | 关键特征                                                          |
| :--------------- | :----- | :------- | :------------------------------------------------------------ |
| sp_vision_23     | Python | ~60 FPS  | 跑通识别→射击完整流程                                                   |
| sp_vision_24     | C++ 重写 | ~100 FPS | 参数文件、日志、离线检测/重启、类 rosbag 录制（时间戳+视频+四元数）、各模块独立测试 → "赛前排查、赛后重现" |
| **sp_vision_25** | C++    | —        | ① 识别器换成**神经网络四点检测**（提高召回）；② 决策器换成**基于轨迹优化的规划器**（替代经验主义分段逻辑）   |

### 1.4 运行环境（3.1）

| 项       | 配置                               |
| :------ | :------------------------------- |
| 操作系统    | Ubuntu 22.04                     |
| 运算平台    | NUC12WSKI7（i7-1260P，16GB）        |
| 相机 / 镜头 | 海康 MV-CS016-10UC / 官方 6mm        |
| 下位机     | RoboMaster 开发板 C 型（STM32F407）    |
| IMU     | C 板内置 BMI088（或达妙 DM IMU）         |
| 通信      | USB2CAN（旧）、MicroUSB 虚拟串口（新）      |
| 辅助工具    | NoMachine（远程桌面）、PlotJuggler（曲线图） |

**依赖库**：MindVision/HikRobot SDK、OpenVINO（神经网络推理）、Ceres（优化）、OpenCV、Eigen、fmt、spdlog、yaml-cpp、nlohmann-json。

### 1.5 一句话速记

> sp_vision_25 = "神经网络四点识别 + EKF 整车估计 + **轨迹规划器决策**" 的分布式自瞄，视觉跑在 NUC、控制跑在 C 板，最大创新是把决策从"分段 if-else"升级为"轨迹优化"。

---

## 2. 项目结构

### 2.1 目录树

```
sp_vision_25
├── assets/          // demo 素材、神经网络权重
├── calibration/     // 标定程序:相机内参、手眼标定、数据采集
├── configs/         // 每台机器人的 YAML 配置(standard3/4、sentry、uav...)
├── io/              // 【硬件抽象层】相机、下位机、IMU、串口/CAN、ROS2 接口
├── src/             // 【应用层】各兵种 main 函数(standard/sentry/uav...)
├── tasks/           // 【功能层】
│   ├── auto_aim/        // 自瞄:detector→tracker/target→planner/aimer→shooter
│   │   ├── multithread/ // 多线程版识别器与命令生成器
│   │   ├── planner/     // 轨迹规划器 + tinympc 求解库
│   │   └── yolos/       // yolo11 / yolov8 / yolov5 三种模型实现
│   ├── auto_buff/       // 打符:能量机关识别、解算、正弦预测
│   └── omniperception/  // 全向感知(多相机/大视野目标发现)
├── tests/           // 每个模块的独立测试程序(赛前排查、赛后重现)
├── tools/           // 【工具层】EKF、弹道、CRC、数学、日志、绘图、录制、线程队列
├── CMakeLists.txt
├── buff_layout.xml / mpc_layout.xml   // PlotJuggler 布局
└── readme.md        // 项目文档(含轨迹理论详解)
```

### 2.2 关键文件速查

| 文件                                         | 作用                         |
| :----------------------------------------- | :------------------------- |
| `tasks/auto_aim/detector.*`                | 传统灯条识别器                    |
| `tasks/auto_aim/yolo*.* + yolos/`          | 神经网络四点检测器（YOLO11/v8/v5）    |
| `tasks/auto_aim/classifier.*`              | 数字分类器（ONNX + softmax）      |
| `tasks/auto_aim/solver.*`                  | PnP 位姿解算 + yaw 优化          |
| `tasks/auto_aim/tracker.* target.*`        | EKF 整车状态估计                 |
| `tasks/auto_aim/planner/planner.*`         | 轨迹规划器（决策核心）                |
| `tasks/auto_aim/aimer.* shooter.* voter.*` | 瞄准 / 开火 / 投票               |
| `tools/extended_kalman_filter.*`           | 通用 EKF（std::function 注入模型） |
| `tools/trajectory.*`                       | 弹道解算（子弹飞行时间 t_fly）         |
| `tools/math_tools.*`                       | 四元数/欧拉角/球坐标/角度限幅           |
| `io/camera.* hikrobot/ usbcamera/`         | 相机驱动（多态基类）                 |
| `io/cboard.* command.hpp`                  | 下位机通信协议                    |

### 2.3 测试程序体系（本项目工程化亮点）

`tests/` 下每个模块都有独立可执行程序，支持"录制视频 + 时间戳 + 四元数"离线回放，实现**赛前问题排查、赛后问题重现**。典型：`auto_aim_test`（录制视频跑自瞄）、`detector_video_test`（离线测识别）、`planner_test_offline`（离线测规划器）、`gimbal_response_test`（测云台响应）。这是 sp_vision_24 起沉淀的可靠性工程能力。

---

## 3. 软件架构与分层设计

### 3.1 为什么需要"视觉框架"

一个兵种往往有多个功能（步兵要自瞄 + 打符），但**相机只能被一个进程打开**——若自瞄、打符各是独立程序，会互相抢占相机导致失败。

sp_vision 的解法：**一个 main 进程 + 多个"功能组"**。main 负责从相机取图，根据电控发来的档位信号（自瞄档 / 打符档）选择对应功能组执行。功能组由类/函数组成（识别器、解算器、预测器等）。不同兵种 = 不同的 main 逻辑（`src/standard.cpp`、`src/sentry.cpp`、`src/uav.cpp` 等）。

### 3.2 三层架构

```
┌─────────────────────────────────────────────┐
│  应用层 src/     不同兵种的 main:            │
│                  standard / sentry / uav     │  ← 组织流程、按档位选功能组
├─────────────────────────────────────────────┤
│  功能层 tasks/   auto_aim / auto_buff /       │  ← 各功能组(识别→估计→决策)
│                  omniperception              │
├─────────────────────────────────────────────┤
│  工具层 tools/   EKF / 弹道 / 数学 / 日志 /   │  ← 复用组件,减少重复造轮子
│                  CRC / 录制 / 线程队列        │
├─────────────────────────────────────────────┤
│  硬件抽象层 io/  相机 / 下位机 / IMU /        │  ← 屏蔽硬件差异,多态切换
│                  串口 / CAN / ROS2           │
└─────────────────────────────────────────────┘
```

**设计原则**：应用层只管"业务编排"，功能层实现"算法逻辑"，工具层提供"通用能力"，硬件层"屏蔽差异"。上层依赖下层，下层不知道上层——这让功能层的算法（如识别器）可以脱离真实硬件、在离线视频上测试。

### 3.3 一句话速记

> 一个 main 进程独占相机、按电控档位切功能组；四层（应用/功能/工具/硬件抽象）自上而下依赖，算法层与硬件解耦，因此能"录像离线复现"。

---

## 4. 核心数据结构

整条自瞄流水线围绕三个核心结构体流动：**Lightbar（灯条）→ Armor（装甲板）→ Target（整车状态）**。理解它们的字段就理解了数据在系统里如何逐级抽象。

### 4.1 Lightbar 灯条（`armor.hpp:67-78`）

传统识别的中间产物。一根灯条 = 一个旋转矩形抽象成的几何体。

| 字段 | 含义 |
|:----|:----|
| `center` / `top` / `bottom` | 中心、顶部中点、底部中点（角点按 y 升序取均值） |
| `top2bottom` | top→bottom 向量 |
| `angle` / `angle_error` | 主轴倾角 / 与竖直（π/2）的偏差（几何过滤用） |
| `length` / `width` / `ratio` | 长 / 宽 / 长宽比 |
| `color` | red/blue/extinguish/purple |

### 4.2 Armor 装甲板（`armor.hpp:80-121`）— 全系统枢纽

**关键设计**：无论传统灯条配对还是 YOLO 关键点检测，最终都归一到 `points`（四角点，固定顺序 **[左上, 右上, 右下, 左下]**）。下游 PnP 因此完全不关心识别方式。

| 字段                                | 含义                                                  |
| :-------------------------------- | :-------------------------------------------------- |
| `points`                          | **四角点，固定顺时针顺序**（识别路径的统一出口）                          |
| `center`                          | 四点均值中心。⚠️ 注释警告：不是对角线交点，不能当真实中心                      |
| `left` / `right`                  | 左右灯条（仅传统路径有效）                                       |
| `type`                            | big / small                                         |
| `name`                            | one~five / sentry / outpost / base / not_armor（9 类） |
| `priority`                        | 打击优先级（1~5）                                          |
| `class_id` / `box` / `confidence` | 神经网络原始类别 / 检测框 / 置信度                                |
| `pattern`                         | 装甲板图案 ROI（送数字分类器）                                   |
| `duplicated`                      | 共用灯条去重标记                                            |
| `xyz_in_gimbal` / `xyz_in_world`  | 云台系 / 世界系位置（m）                                      |
| `ypr_in_gimbal` / `ypr_in_world`  | 云台系 / 世界系欧拉角（yaw-pitch-roll，rad）                    |
| `ypd_in_world`                    | 世界系球坐标（yaw, pitch, distance）                        |
| `yaw_raw`                         | yaw 优化前的原始 PnP 值（保留对比）                              |

**枚举与映射**：`armor_properties`（`armor.hpp:52-64`）是 YOLO11 的 38 类 id → (Color, ArmorName, ArmorType) 的映射表，覆盖 蓝/红/熄灭/紫 四色 × 各兵种 × 大小装甲。

### 4.3 Target 整车状态 — 11 维状态向量

**核心思想**：不追踪单块装甲板，而是把敌方机器人看成"绕竖直轴旋转的刚体，N 块装甲板均匀分布在圆周上"，EKF 估计的是**旋转中心 + 角速度**。（详见 §6.2）

状态向量 `x = [x, vx, y, vy, z, vz, a, w, r, l, h]`（`target.cpp:34-39`）：

| 下标  | 符号       | 含义                               |
| :-- | :------- | :------------------------------- |
| 0,1 | x, vx    | 旋转中心 X 坐标与速度                     |
| 2,3 | y, vy    | 旋转中心 Y 坐标与速度                     |
| 4,5 | z, vz    | 基准装甲板高度与速度                       |
| 6   | a (yaw)  | 基准装甲板法向朝向角（整车相位，过 `limit_rad`）   |
| 7   | w (vyaw) | 整车绕竖直轴角速度                        |
| 8   | r        | 主半径 r1（收敛判据要求 0.05~0.5m）         |
| 9   | l        | 半径差 r2−r1（长短轴现象，仅 4 板车 id=1/3 用） |
| 10  | h        | 高度差 z2−z1（双半径装甲板高度偏移）            |

**为什么用 l、h**：多数整车的装甲板不完全对称（长短轴 + 高低差），用两组半径 `r`/`r+l` 和两组高度 `z`/`z+h` 交替描述相邻装甲板。`armor_num`（2/3/4）决定分布——4 板车用 l/h，3 板前哨站、2 板平衡步兵不用。

### 4.4 数据抽象链一句话速记

> 图像 →（识别）→ **Lightbar** 配对/YOLO 四点 → **Armor**（四点+类别，统一出口）→（PnP+yaw优化）→ Armor 的世界坐标 →（EKF）→ **Target**（旋转中心+角速度）→（规划器）→ 云台命令。

---

## 5. 自瞄完整数据流

### 5.1 单帧流水线

```
camera.read(img, t)                    // 相机取图 + steady_clock 时间戳 t
   ↓
cboard.imu_at(t - 1ms)  →  q           // 按时间戳插值取云台姿态四元数(见 5.3)
   ↓
solver.set_R_gimbal2world(q)           // 用 q 构造 gimbal→world 旋转
   ↓
detector.detect(img)  →  [Armor...]    // 识别:四点 + 类别(传统 or YOLO)
   ↓
solver.solve(armor)                    // 每块 PnP 解算 + yaw 优化 → world 坐标
   ↓
tracker.track(armors, t)  →  Target    // EKF 整车估计
   ↓
aimer/planner  →  Command{yaw,pitch,shoot}  // 轨迹规划 + 开火决策
   ↓
cboard.send(command)                   // CAN/串口下发给电控
```

### 5.2 多线程版（mt_standard）的三线程解耦

```
[capture_thread]        [detect_thread]              [主线程]                [CommandGener 线程]
 相机采集(150fps) ──▶ camera.read→detector.push ──▶ debug_pop(等推理)      500Hz 独立循环
 队列容量1只留最新     OpenVINO 异步推理重叠         imu_at 对齐→solver       aim→shoot→send
                     MultiThreadDetector队列16       tracker→commandgener.push  → cboard.send
```

**为什么这样拆**：① 相机队列容量 1 → 永远只留最新帧，天然丢旧帧保低延迟；② OpenVINO 异步推理让采集与推理重叠，提吞吐；③ 决策/发送剥离成 500Hz 高频线程，即使视觉帧率只有 ~150Hz，云台指令仍高频输出，中间由 EKF 预测填补；④ CommandGener 有 200ms 时效窗（`commandgener.cpp:44`），丢弃陈旧目标不误发指令。

### 5.3 图像-四元数对齐（关键难点）

**问题**：相机曝光瞬间打的时间戳 `t`，要找那一刻云台的姿态。但 IMU/CAN 姿态是异步离散到达的，时间戳不会正好等于 `t`。

**解法**（`cboard.cpp:22-45` `imu_at`）：用两个游标 `data_ahead_`/`data_behind_`，不断从队列 pop 直到 `data_behind_.timestamp > t`，夹出 `t` 两侧的两帧姿态，再做**球面线性插值 slerp**：

```cpp
auto k = t_ac / t_ab;                              // 插值比例 (t-t_a)/(t_b-t_a)
Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();  // 四元数球面插值
```

要点：① **阻塞式**——后一帧没到就卡在 `queue_.pop` 等待，保证一定夹住 `t`，所以 IMU 队列容量必须大（5000 帧）以回溯历史；② 调用方统一用 `t - 1ms` 补偿"时间戳晚于真实曝光"的固定偏差；③ 三套 IMU 源（CAN / 串口云台 / DM_IMU）对齐算法一致。

### 5.4 通信协议要点

| 方向    | 载体                                           | 内容                                                |
| :---- | :------------------------------------------- | :------------------------------------------------ |
| 电控→视觉 | CAN `quaternion_canid` / 串口 `GimbalToVision` | 姿态四元数、mode 档位、弹速、bullet_count                     |
| 视觉→电控 | CAN `send_canid` / 串口 `VisionToGimbal`       | control/shoot 标志、yaw/pitch（**+速度/加速度前馈**，供计算力矩控制） |
|       |                                              |                                                   |

- **定点传输**：CAN 帧里角度 `(int16_t)(值 * 1e4)` 拆高低字节，保 4 位小数。
- **CRC**：CAN 靠硬件 CRC（软件不做）；串口帧头 `'S','P'` + CRC16 校验（查表法，DJI 标准表）。
- **坏帧防护**：四元数模长校验 `|q|²−1 > 1e-2` 丢弃（`cboard.cpp:78`）。
- **视觉→电控发二阶量**（位置+速度+加速度）正是 §7.3 "把轨迹速度/加速度作前馈" 的落地。

### 5.5 关键时间约束速查

| 项             | 值                  | 出处                    |
| :------------ | :----------------- | :-------------------- |
| 相机帧率          | 150 fps            | `hikrobot.cpp:90`     |
| 决策/发送频率       | ~500 Hz（sleep 2ms） | `commandgener.cpp:68` |
| 目标数据时效窗       | 200 ms 超时丢弃        | `commandgener.cpp:44` |
| 时间对齐补偿        | 统一 −1ms            | standard/mt/sentry    |
| IMU 队列缓存      | 5000 帧             | `cboard.cpp:12`       |
| 相机/CAN 断线重连检查 | 每 100ms            | `hikrobot.cpp:23`     |

---

## 6. 五大模块核心知识点

### 6.1 装甲板识别器（detector / yolo / classifier）

**两条路径，统一出口**。无论走哪条，最终都产出 `Armor.points`（四角点，[左上,右上,右下,左下]）+ `name`/`type`。

**传统路径**（`detector.cpp:33-120`）：
1. 灰度化 → 二值化 `threshold`（阈值 `threshold_`，`:41`）→ `findContours`（RETR_EXTERNAL）
2. 每个轮廓 `minAreaRect` → `Lightbar`，几何过滤（角度误差、长宽比、长度）
3. `get_color` 沿轮廓累加 R/B 通道判色 → 左到右排序 → 同色两两配对成 `Armor`
4. 沿灯条延长 1.125 倍取 `pattern` ROI（`:295`，几何常数 = 0.5×126mm/56mm）→ 数字分类器
5. `get_type` 判大小装甲（ratio >3 大、<2.5 小，边界看兵种）
6. **去重**（`:87-115`）：检测共用灯条 → 保留 pattern 面积小/置信度大者

**神经网络路径**（以 YOLO11 `yolo11.cpp:56-176`）：letterbox 到 640×640 → OpenVINO 推理 → 转置输出逐行解码 `[xywh + 类别分 + 四点]` → `NMSBoxes` → `sort_keypoints` 重排为固定四点序 → `class_id` 查 `armor_properties` 表得颜色/名称/类型。

**三种模型对比**：

| 模型     | 输入   | 类别数 | 特点                                       |
| :----- | :--- | :-- | :--------------------------------------- |
| YOLO11 | 640² | 38  | anchor-free，端到端出颜色+编号+四点                 |
| YOLOv8 | 416² | 2   | NN 只出颜色+框+四点，**仍调传统分类器**得数字              |
| YOLOv5 | 640² | 13  | 颜色/数字独立头，objectness 过 sigmoid；可选传统二次矫正角点 |

**OpenVINO 图内预处理**（`yolo11.cpp:45-48`）：用 `PrePostProcessor` 把归一化 + BGR→RGB 编入模型，CPU 只喂 u8 letterbox 图，`LATENCY` 模式优化单帧延迟。

**数字分类器**（`classifier.cpp:41-58`）：32×32 灰度 letterbox → ONNX 前向 → 数值稳定 softmax。输出 9 类 `ArmorName`。

> **一句话速记**：识别 = "灯条配对(传统) 或 YOLO 四点检测(NN)" 都归一到四角点 + 类别；YOLOv8 是"NN 定位 + 传统分类"混合，YOLOv5 可用传统灯条二次矫正角点提高 PnP 精度。

### 6.2 位姿解算器（solver：PnP + yaw 优化）

**3D 物点模型**（`solver.cpp:12-25`）：装甲板放自身坐标系 x=0 平面（法向为 x 轴），四点按 [左上,右上,右下,左下]，y=半宽（大 230/小 135mm）、z=**半灯条长（56mm，不是装甲板物理高度）**——因为 PnP 匹配的是灯条端点。

**PnP**（`solver.cpp:61-63`）：`cv::solvePnP(..., SOLVEPNP_IPPE)`。IPPE 专为共面 4 点设计，快且稳，但对绕竖直轴的 **yaw 存在法向翻转歧义**——这正是要额外做 yaw 优化的原因。

**坐标变换链**（相机→云台→世界）：
```cpp
xyz_in_gimbal = R_camera2gimbal * xyz_in_camera + t_camera2gimbal;   // solver.cpp:67
xyz_in_world  = R_gimbal2world * xyz_in_gimbal;                      // :68
```
其中 `R_gimbal2world = R_gimbal2imubody^T · R_imubody2imuabs · R_gimbal2imubody`（`:48-52`，把 IMU 姿态补偿到云台安装系）。

**yaw 优化（核心亮点，`solver.cpp:196-218`）**：平面 PnP 的 yaw 噪声大，但距离/pitch 可靠。于是**固定 xyz、假定 pitch=15°**（装甲板后仰 15° 安装，前哨站 −15°），只在 ±70° 范围内以 1° 步进**暴力搜索 yaw**，取重投影误差最小者：
```cpp
for (int i = 0; i < SEARCH_RANGE; i++) {        // 140 次搜索
  double yaw = tools::limit_rad(yaw0 + i * CV_PI/180.0);
  auto error = armor_reprojection_error(armor, yaw, (i-70)*CV_PI/180.0);
  if (error < min_error) { min_error = error; best_yaw = yaw; }
}
```
> 平衡步兵（大装甲的 three/four/five）不做 yaw 优化——其 pitch=15° 假设不成立。原始 PnP yaw 保留在 `yaw_raw`。

> **一句话速记**：共面装甲板用 IPPE 解 PnP，但 yaw 不可靠；固定 xyz + 假定 pitch=15°，在 ±70° 内暴力搜索让重投影误差最小的 yaw——这是全套解算精度的命门。

### 6.3 整车状态估计器（tracker / target / EKF）

**核心思想**：不追踪单块装甲板，而是把敌方机器人建模成"绕竖直轴旋转的刚体，N 块装甲板均匀分布在圆周上"，EKF 估计**旋转中心 + 角速度**（11 维状态，见 §4.3）。好处：① 小陀螺时装甲板轮替出现，中心估计连续不跳变；② 能预测暂不可见的装甲板位置（打提前量）；③ 用 `id·2π/N` 相位 + 长短轴一套状态覆盖不对称整车。

**EKF 五公式与实现对应**：

| 步骤    | 公式                                    | 实现                                       |
| :---- | :------------------------------------ | :--------------------------------------- |
| 状态先验  | $\hat{x}^-=f(\hat{x})$                | `target.cpp:125-129`（仅对 yaw 做 limit_rad） |
| 协方差先验 | $P^-=FPF^T+Q$                         | `ekf.cpp:32`                             |
| 卡尔曼增益 | $K=P^-H^T(HP^-H^T+R)^{-1}$            | `ekf.cpp:50`                             |
| 状态后验  | $\hat{x}=\hat{x}^-+K(z-h(\hat{x}^-))$ | `ekf.cpp:56`                             |
| 协方差后验 | $P=(I-KH)P^-(I-KH)^T+KRK^T$           | `ekf.cpp:54`（Joseph 形式保正定）               |

**通用 EKF 工具类设计**（`extended_kalman_filter.hpp`）：不含任何整车知识，用 `std::function` 注入非线性 `f`/`h`、雅可比 `F`/`H`，以及**角度安全的状态加法 `x_add` 和残差 `z_subtract`**（整车对 yaw/pitch/angle 做 limit_rad）。这让同一个 EKF 类可用于任意维度——打符也复用它。

**观测方程**（`target.cpp:267-278`）：整车状态 → 第 id 块装甲板 xyz → 球坐标 (yaw,pitch,distance) + 朝向角：
```cpp
angle = limit_rad(x[6] + id*2π/armor_num);
r = (armor_num==4 && (id==1||id==3)) ? x[8]+x[9] : x[8];  // 长短轴切换
armor_x = x[0] - r*cos(angle);  armor_y = x[2] - r*sin(angle);
```

**自适应观测噪声 R**（`target.cpp:194-196`）：yaw/pitch 固定 4e-3；**distance/angle 随几何动态调**——朝向夹角越大 angle 越不可信，距离越远越不可信。这是精度关键：几何差的观测自动降权。

**数据关联**（`target.cpp:138-185`）：生成全部虚拟装甲板 → 按距离取最近 3 块 → 关联度量 = 朝向角误差 + yaw 角误差最小者 → 用该 id 的 h/H 做 EKF 更新。整车模型天然处理换板：无论看到哪块，都更新同一旋转中心，只换相位偏移。

**tracker 状态机**（`tracker.cpp:180-229`）：

```
 lost ──found──▶ detecting ──连续 min_detect_count 帧──▶ tracking
   ▲               │丢失                                  │丢失
   └───────────────┘                                temp_lost ──超 max_temp_lost_count──▶ lost
                                                     (前哨站用更大的 outpost 阈值)
   switching(全向感知目标待进主相机)
```

额外强制转 lost：相机断帧 dt>0.1s；`target.diverged()`（半径出 [0.05,0.5]m）；近窗口 NIS 卡方失败率 ≥40%（`tracker.cpp:83-90`）。**前哨站特判**：转速恒定 ~2.51 rad/s（真实 ~0.4r/s≈2.513，代码取近似 2.51），收敛后强制钳位（`target.cpp:131-133`）。详见 [[SP_Vision_整车估计EKF详解]] §10.2。

> **一句话速记**：EKF 估的不是装甲板而是"旋转中心+角速度"的整车模型，换板只是切相位；用 Joseph 形式 + 自适应 R + NIS 卡方检验 + 物理半径区间保证鲁棒，通用 EKF 类靠 std::function 注入角度安全运算与打符共用。

### 6.4 决策器 · 轨迹规划器（planner + tinympc）

> 这是 §7 理论的代码落地。主链路用 **Planner**（`src/standard_mpc.cpp`），旧的 aimer/shooter 分段方案并行保留。

**轨迹的代码表示**（`planner.hpp:13-17`）：`Trajectory = Matrix<4, 100>`，4 行 = [yaw, yaw_vel, pitch, pitch_vel]，100 列 = 时域 [−0.5s, +0.5s]，DT=10ms，第 50 列 = 当前时刻。

**plan() 完整流程**（`planner.cpp:27-94`）：
1. 弹速兜底（<10 或 >25 强制 22 m/s）
2. **预测 fly_time**：`tools::Trajectory` 算子弹飞行时间 → `target.predict(fly_time)`，把目标轨迹平移成**射击轨迹**
3. `get_trajectory` 生成 4×100 参考轨迹（中心差分算速度前馈，yaw 存相对增量避免绕环）
4. yaw / pitch 各解一个 TinyMPC，`Xref` = 射击轨迹
5. 取**第 50 列（当前时刻）**组装 `Plan`：位置来自 MPC 解（云台轨迹），带速度/加速度前馈

**TinyMPC 求解的优化问题**（约束 LQR / 盒约束 QP）：
```
min Σ ||x_k − Xref_k||²_Q + ||u_k||²_R
s.t. x_{k+1} = A x_k + B u_k          A=[[1,DT],[0,1]], B=[[0],[DT]]  (二重积分器)
     u_min ≤ u_k ≤ u_max              = ±max_yaw_acc  ← 云台最大加速度约束
```
状态 x=[角度, 角速度]，控制 u=[角加速度]。**提前减速是隐式产生的**：求解器为满足加速度上限、又要贴合突变的参考轨迹，只能提前偏离开始过渡（`planner.cpp:126-127` 的 `u_min/u_max`）。用 ADMM 求解，`max_iter=10`，<1ms。

**为何 MPC 是求解器而非闭环控制器**：它输入只有 EKF 预测的目标运动、**不含云台实际状态**，输出一条"云台能力内可跟随"的轨迹（含前馈）交下位机闭环。实测直接当闭环控制器转发力矩，小陀螺时反而更差。定位是"离线式轨迹整形器"。

**开火决策**（`planner.cpp:87-92`）：查 `HALF_HORIZON + shoot_offset_`（当前 + 20ms ≈ t_fire）列，比较射击轨迹与 MPC 规划轨迹的 yaw/pitch 欧氏误差 < `fire_thresh_` → 开火。即"20ms 后云台是否落在射击轨迹容差内"，无需任何分段参数。

**Voter**（`voter.cpp`）：与开火**无关**，是 (Color×ArmorName×ArmorType) 三维投票计数器，多帧投票稳定分类结果、消除单帧误识别。

**弹道解算**（`trajectory.cpp:9-31`）：斜抛无阻力，g=9.7833。化为 tanθ 的一元二次 `(g·d²/2v0²)tan²θ − d·tanθ + (g·d²/2v0²+h)=0`，两解取飞行时间更短者（直射优于高抛）。

> **一句话速记**：plan() 先用 fly_time 把目标轨迹平移成射击轨迹，TinyMPC 在"云台最大加速度"盒约束下把它整形成可跟随的云台轨迹（提前减速自然涌现），开火退化为"t_fire 后两轨迹是否重合"，一次 QP 同时搞定瞄准+前馈+开火。

### 6.5 打符 · 全向感知（auto_buff / omniperception）

**打符流程**（`auto_buff/`）：
1. **识别**（`buff_detector.cpp`）：YOLO11_BUFF 出 `{buff, r}` 类，每扇叶 6 关键点。R 标（旋转中心）不由模型直接给，而是用扇叶关键点几何外推 + 二值化找圆形轮廓精定位（`:42,60-75`）
2. **扇叶归类**（`buff_type.cpp`）：多亮扇叶中定 target（新亮起的那片），按 5 等分角 `2π/5` 排序
3. **位姿解算**（`buff_solver.cpp`）：5 关键点 + R 标做 IPPE PnP → buff→camera→gimbal→world
4. **旋转预测**：见下

**大符正弦模型**（核心）：官方转速公式 **spd = a·sin(w·t+φ) + (2.09−a)**（即 b=2.09−a），约束 a∈[0.780,1.045]、w∈[1.884,2.000]。两套实现并存：
- **EKF 内嵌正弦**（`BigTarget`，10 维含 a/w/φ）：预测用角度积分闭式解
- **RANSAC 正弦拟合**（`ransac_sine_fitter.cpp`）：EKF 估出的 spd 喂入拟合器，固定 ω 后线性最小二乘（SVD）解 `A1·sin(ωt)+A2·cos(ωt)+C`，再换算 A=√(A1²+A2²)、φ=atan2(A2,A1)；预测时用拟合值替代 EKF 原始值

**小符**（`SmallTarget`）：匀速旋转，固定 SMALL_W=π/3，无 a/w/φ。

**buff Target vs 自瞄 Target**：buff 是"R 标球坐标 + 单一 roll 角 + 转速正弦参数"的定轴旋转模型（半径固定 0.7m）；自瞄是"平移 + 多装甲板绕轴旋转"的整车模型。二者都基于同一个通用 EKF、都用 ypd 球坐标观测。

**全向感知**（`omniperception/`）：4 路 USB 相机 + 后置相机广域搜敌，各自独立 YOLO 并行推理，结果入线程安全队列。`Decider` 按相机安装方位（left/right ±62°、back +170°）把归一化中心换算成相对云台的偏航/俯仰角，过滤非敌色/无敌装甲，按优先级排序。用途：给云台**转向指引**（跟丢时"四处搜敌"，见 `sentry.cpp:90`）+ 给自瞄提供切换目标坐标，自己不做精确弹道。

> **一句话速记**：打符 = "YOLO扇叶+几何找R标 → IPPE解算 → 大符用 spd=a·sin(wt)+(2.09−a) 正弦模型(EKF内嵌或RANSAC拟合)、小符匀速π/3"；全向感知 = 多相机广域搜敌给云台转向指引，跟丢时接管。

---

## 7. 轨迹视角下的自瞄理论

> 这是 sp_vision_25 的核心创新，也是整个项目最值得学习的部分。传统自瞄决策靠经验主义的分段 if-else，本项目把决策问题重构为**轨迹优化问题**。

### 7.1 痛点:传统决策器为什么难

**决策器实现繁琐**：不同兵种、不同敌方状态（平移 / 低速小陀螺 / 高速小陀螺）需要不同的"自瞄行为"。例如英雄只瞄旋转中心、按时间误差开火；步兵平移/低速时持续跟随装甲板、按位置误差开火，高速时又退化为瞄中心。每种行为都要单独写代码 + 设计判断条件 → 维护成本高、参数多、调试负担重。

**控制调参困难**：传统控制指标输入是阶跃/斜坡，而自瞄输入更像**三角波**，且周期/幅值随敌方状态变化。没有理论把二者联系起来，调参靠经验猜测，且不知道云台控制效果的上限在哪。

### 7.2 核心概念:四条轨迹

把自瞄从"瞄准一个点"重新理解为"跟随一条 yaw 随时间变化的曲线"（pitch 同理，只讨论 yaw）：

| 轨迹 | 定义 | 公式 |
|:----|:----|:----|
| **目标轨迹** $yaw_{target}(t)$ | 各时刻距云台最近装甲板相对云台的方位角 | 由整车匀速模型算任意 t 的装甲板位置 |
| **射击轨迹** $yaw_{shoot}(t)$ | 目标轨迹提前"子弹飞行时间 $t_{fly}$"后的轨迹 | $yaw_{shoot}(t)=yaw_{target}(t+t_{fly})$ |
| **云台轨迹** $yaw_{gimbal}(t)$ | 各时刻云台相对初始位置的实际夹角 | 由控制器执行结果决定 |
| **规划后轨迹** | 轨迹规划器优化输出、供云台跟随的可行轨迹 | 见 §7.4 |

**命中条件**：$t$ 时刻 $yaw_{gimbal}(t)$ 与 $yaw_{shoot}(t)$ 误差在半个装甲板内，则此刻射出的子弹命中。若两条轨迹**完全重合**，则任意时刻射出的子弹都命中——"子弹发射如水流一样"。

### 7.3 什么是"好自瞄":重合度决定一切

击杀时间是金标准：

$$\text{击杀时间} = \frac{\text{血量}}{\text{DPS}}, \quad \text{DPS} = \text{单位时间射击窗口占比} \times \text{射频} \times \text{单发伤害}$$

$$\text{命中率} \le \text{重合度} = \frac{\text{重合射击轨迹段}}{\text{重合段} + \text{偏离段}}$$

**结论**：云台轨迹与射击轨迹**重合度越高**，射击窗口占比越高、DPS 越高、击杀时间越短，同时命中率上限越高。于是自瞄好坏被统一为一个可量化目标——最大化轨迹重合度。

重合度受**云台控制能力**制约：

$$\text{云台最大加速度} = \frac{\text{云台电机最大扭矩}}{\text{云台惯量}}$$

（扭矩由电机厂家给出，惯量由系统辨识得到。）为降低控制难度，本项目三管齐下：① 轨迹规划器按云台最大加速度优化射击轨迹使其可跟随；② 计算轨迹的速度/加速度作**前馈**发给电控；③ 采用**计算力矩控制**（结合 PID + 动力学模型，比双环 PID 好调）。

### 7.4 轨迹规划器:两个关键策略

**① 提前减速策略（解决轨迹突变）**

小陀螺时装甲板切换 → 目标/射击轨迹在切换点**不连续**，瞬时速度/加速度未定义，硬跟随会超调或滞后。对策：若未来一段时间装甲板会切换，就**提前减速**向下一块装甲板过渡，使规划后轨迹加速度 ≤ 云台最大加速度。

两种实现方案：

| 方案 | 原理 | 特点 | 实战 |
|:----|:----|:----|:----|
| **隐式搜索** | 以规划轨迹加速度序列为变量，代价=重合度尽可能高，约束=不超云台最大加速度 → 转成 QP，调 **TinyMPC** 求解（<1ms） | 通用；MPC 仅作求解器找可行轨迹解，**不含云台实际状态**（非闭环控制器） | ✅ 国赛上场 |
| **显式搜索** | 以过渡时间为变量，切换点前后起终点定死，用**五次多项式**生成过渡段，调时长直到满足加速度限制 | 确定性高、"跟随段"不参与优化与射击轨迹重合、高转速更稳、无需第三方库 | 仅仿真验证 |

> 注意：这里 MPC 是**求解器**而非闭环控制器。作者试过用 MPC 直接闭环（转发力矩给电机），自身小陀螺时效果反不如下位机方案。

**② 提前开火决策（解决开火延迟）**

从开火命令到子弹出膛（摩擦轮掉速）有延迟 $t_{fire}$。只看当前位置误差判断开火不严谨。轨迹规划器生成一段轨迹序列，**查询 $t_{fire}$ 时刻**射击轨迹与规划轨迹的误差，判断经过 $t_{fire}$ 后子弹是否应出膛。前提是开火延迟波动小（对机械/电控要求高）。

这也顺带给出了命中率保障：云台偏离射击轨迹段（不可避免）时**停火**，减少弹丸浪费。

### 7.5 预测时间偏移量

射击轨迹提前于目标轨迹的时间称"预测时间"，除 $t_{fly}$ 外还需叠加各环节延迟：

| 延迟来源 | 说明 | 可否直接算 |
|:----|:----|:----|
| 图像传输延迟 | 时间戳=接收完成时刻，还要传到小电脑 | 打包进调参 |
| 图像处理延迟 | 神经网络推理耗时不可忽略 | ✅ 可直接算 |
| 下位机通信延迟 | 传决策命令给下位机 | 打包进调参 |
| 下位机控制延迟 | 控云台到目标位置 | 打包进调参 |

实现上除图像处理延迟直接计算外，其余延迟总和作为一个调试参数（约 **15ms**），靠拍慢动作、比击杀时间来调。**不需要考虑开火延迟**——射击轨迹上任意时间发射均命中。

### 7.6 战绩与未来方向

25 赛季国赛：3、4 号步兵最高命中率 39.6%（正常发挥 ≥30%），2m/300HP 靶机 7rad/s 约 8s、14rad/s 约 10s 击杀。未来：轨迹规划器推广到英雄/哨兵/无人机；引入轮式里程计实现"边跑边打"。

### 7.7 一句话速记

> 把自瞄看成"让云台轨迹贴合射击轨迹"，重合度越高击杀越快命中越高；轨迹规划器用"提前减速"处理小陀螺换板突变（TinyMPC 当 QP 求解器 or 五次多项式），用"提前开火（查 $t_{fire}$ 时刻误差）"处理开火延迟，一套框架取代所有分段 if-else。

---

## 8. 配置参数速查

每台机器人一份 YAML（`configs/standard4.yaml` 等），main 启动时加载。分组速查：

**识别（神经网络 + 传统）**

| 参数 | 示例 | 含义 |
|:----|:----|:----|
| `yolo_name` | yolov5 | 用哪个模型（yolo11/yolov8/yolov5） |
| `classify_model` | tiny_resnet.onnx | 数字分类器（v8/传统路径用） |
| `device` | GPU | OpenVINO 推理设备（CPU/GPU） |
| `min_confidence` | 0.8 | 置信度阈值 |
| `use_traditional` | true | YOLOv5 后是否做传统灯条二次矫正角点 |
| `use_roi` / `roi` | false | 是否裁 ROI 及其区域 |
| `threshold` | 150 | 传统法二值化阈值 |
| `max_angle_error` `*_lightbar_ratio` `*_armor_ratio` `max_side_ratio` `max_rectangular_error` | — | 灯条/装甲板几何过滤阈值 |

**估计（tracker）**

| 参数 | 示例 | 含义 |
|:----|:----|:----|
| `min_detect_count` | 5 | detecting→tracking 所需连续检测帧 |
| `max_temp_lost_count` | 15 | temp_lost→lost 的最大容忍帧 |
| `outpost_max_temp_lost_count` | 75 | 前哨站专用（转速稳定，容忍更久） |

**瞄准 / 开火（aimer / shooter / planner）**

| 参数 | 示例 | 含义 |
|:----|:----|:----|
| `yaw_offset` `pitch_offset` | -2 / 0 | 机械安装角补偿（degree） |
| `decision_speed` | 8 rad/s | 高低速小陀螺行为切换阈值 |
| `high/low_speed_delay_time` | 0.015 s | 预测时间偏移（§7.5 的 ~15ms） |
| `first/second_tolerance` | 3 / 2 | 近/远距离射击容差（degree） |
| `judge_distance` | 2 m | 远近判断阈值 |
| `auto_fire` | true | 是否由自瞄接管射击 |
| `fire_thresh` | 0.003 | 开火决策的轨迹误差阈值（rad） |
| `max_yaw_acc` / `max_pitch_acc` | 50 / 100 | 云台最大加速度（喂给 TinyMPC 的约束） |
| `Q_yaw` `R_yaw` | [9e6,0] / [1] | TinyMPC 代价权重（状态 / 输入） |

**标定 / 通信 / 硬件**

| 参数 | 含义 |
|:----|:----|
| `camera_matrix` `distort_coeffs` | 相机内参 K + 畸变系数（PnP 用） |
| `R_camera2gimbal` `t_camera2gimbal` | 手眼标定：相机→云台外参 |
| `R_gimbal2imubody` | 云台→IMU 安装外参 |
| `quaternion_canid` `bullet_speed_canid` `send_canid` | CAN 报文 ID（收姿态四元数、弹速；发命令） |
| `com_port` | 云台串口设备名 |
| `exposure_ms` `gain` | 相机曝光/增益 |

**打符（buff）**

| 参数 | 示例 | 含义 |
|:----|:----|:----|
| `model` | yolo11_buff_int8.xml | 打符检测模型 |
| `fire_gap_time` | 0.520 s | 开火间隔 |
| `predict_time` | 0.100 s | 打符预测提前量 |

---

## 9. 概念关系图谱

### 9.1 模块依赖与数据流总图

```
                          ┌─────────────────────────────────────────┐
                          │  电控 C 板 (STM32F407)                    │
                          │  姿态四元数 q / 弹速 / 档位  ──┐           │
                          └────────────────────────────────┼──────────┘
                                                    CAN/串口 │  ▲ yaw/pitch/shoot
                                                            ▼  │ (+速度/加速度前馈)
   相机 ──img+t──▶ [识别 detector/yolo] ──Armor四点+类别──▶ [解算 solver]
                                                              │ PnP+yaw优化
                                                              ▼ world坐标
                                            imu_at(t-1ms)插值取q → set_R_gimbal2world
                                                              │
                                                              ▼
                                             [估计 tracker/target/EKF] ──Target(旋转中心+w)──┐
                                                                                            ▼
   全向感知 omni ──跟丢时转向指引──────────────────────────────▶ [决策 planner/TinyMPC]
                                                              射击轨迹→整形→云台轨迹+开火
```

### 9.2 核心概念对照表

| 易混概念 | 区别 |
|:----|:----|
| **目标轨迹 vs 射击轨迹** | 射击轨迹 = 目标轨迹提前 t_fly；`yaw_shoot(t)=yaw_target(t+t_fly)` |
| **射击轨迹 vs 云台轨迹** | 射击轨迹是"想让子弹命中该去的角"，云台轨迹是"云台实际/规划能到的角"；二者重合度决定命中率上限 |
| **传统识别 vs YOLO 识别** | 传统:二值化+灯条配对(可解释、无需GPU)；YOLO:端到端四点检测(高召回)。二者归一到同一 `Armor.points` |
| **PnP yaw vs 优化后 yaw** | IPPE 平面 PnP 的 yaw 有法向翻转歧义、噪声大 → 固定 xyz 假定 pitch=15° 暴力搜索修正 |
| **单块装甲板 vs 整车模型** | EKF 不估单板，估旋转中心+角速度；换板只是换相位偏移 id·2π/N |
| **MPC 求解器 vs 闭环控制器** | 本项目 MPC 只输入目标运动(不含云台状态)，输出可跟随轨迹交下位机闭环；不是直接控电机 |
| **Planner vs aimer/shooter** | Planner=轨迹优化(新,亮点,一次QP出瞄准+开火)；aimer/shooter=分段决策(旧,并行保留) |
| **大符 vs 小符** | 大符 spd=a·sin(wt)+(2.09−a) 变速(RANSAC/EKF拟合)；小符匀速 π/3 |
| **buff Target vs 自瞄 Target** | buff=定轴旋转(R标球坐标+roll+正弦转速)；自瞄=平移+多板绕轴。共用通用 EKF |
| **imu_at 的 slerp** | 相机时间戳与姿态帧不对齐 → 夹出前后两帧四元数球面插值，阻塞式等帧 |

### 9.3 全局一句话速记

> **感知**（识别四点 → PnP+yaw优化 → world坐标）→ **估计**（EKF整车模型，估旋转中心+角速度）→ **决策**（射击轨迹经TinyMPC在加速度约束下整形成云台轨迹，t_fire后重合则开火）→ **通信**（发yaw/pitch+前馈给C板闭环）。图像-四元数靠时间戳slerp对齐，多线程用队列解耦、500Hz高频发指令由EKF预测填补。

---

## 10. 相关笔记

### 专题分册（已成册）

本母文档建立了全局框架，五个专题分册各自深化一个模块，均带 `文件名:行号` 源码引用：

- [[SP_Vision_装甲板识别与解算详解]] — 深化 §6.1/6.2：传统灯条几何/去重、YOLO 三模型解码细节、PnP+yaw 优化重投影推导
- [[SP_Vision_整车估计EKF详解]] — 深化 §6.3：通用 EKF 注入式设计、11 维状态雅可比逐项推导、自适应 R、NIS 卡方检验、tracker 状态机与三道护栏
- [[SP_Vision_轨迹规划器详解]] — 深化 §6.4+§7：四轨迹理论、二重积分器建模、TinyMPC/ADMM 求解、提前减速隐式涌现、开火决策、弹道解算
- [[SP_Vision_打符与全向感知详解]] — 深化 §6.5：R 标两步定位、扇叶归类、大符正弦 EKF 内嵌 vs RANSAC 拟合、定轴旋转模型、多相机 Decider 架构
- [[SP_Vision_IO与多线程架构详解]] — 深化 §5.2/5.3/5.4：imu_at slerp 对齐、三套 IMU 源对比、收发协议逐字段、相机多态、三线程模型、OpenVINO 异步推理

### 对照参考

- [[MAS_Vision_学习指南]] — 本战队自研视觉体系，可与 sp_vision 的设计选择对照
- [[MAS_Vision_装甲板检测教程]]、[[MAS_Vision_位姿解算与目标跟踪教程]] — 同类模块的不同实现
- [[MAS_Vision_弹道解算与射击控制教程]] — 弹道/开火决策的另一种思路

### 关联通用知识

- [[RM_Socket编程详解]]、[[RM_ThreadX_RTOS教程]] — 通信与实时性基础
- [[RM_C程序员_C++速通指南]] — std::function 注入、模板、多态等本项目大量使用的 C++ 技法

### 外部资源

- 项目仓库：https://github.com/TongjiSuperPower/sp_vision_25
- 往年：sp_vision_23（Python）、sp_vision_24（C++ 重写）
- 关键参考：陈君 rm_vision（整车估计）、方俊杰 Linear Modelled Top Detector（轨迹视角）、TinyMPC（ICRA 2024）、Modern Robotics（计算力矩控制）
