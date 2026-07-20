# RoboMaster 嵌入式知识库

> 对应 Hub 项目 `mas_embedded_threadx`（路径 `E:\STM32project\rm_2027\mas_embedded_threadx`）的数字孪生级知识库。

---

## 四层架构

```
00_raw/          原始训练资料（只读）
                  PDF 手册、参考代码，不修改，作为知识链的最终出处

01_extracted/    提炼资料
                  从原始资料中萃取的结构化知识（CMake 语法、CAN 过滤器、kfifo 原理等）
                  代码孪生层遇到新概念时链接回这里

02_code_twin/    代码数字孪生
                  每个源文件对应一篇 MD 笔记，镜像项目目录结构
                  摘取关键代码片段 + 逐段解析，首次遇到某语法时链接回 01 层

03_moc/          多维度入口
                  全流程索引，按构建/执行顺序组织，链接到 02 层各文件
```

```
04_notes/        开发经验（独立于四层架构）
                  环境踩坑、变更日志
```

**引用规则**：第 N 层只能向下引用第 N-1 层，不可反向。所有跨文件引用使用 `[[wikilink]]` 语法，路径基于仓库根目录。

---

## 阅读入口

| 入口 | 文件 | 看什么 |
|------|------|--------|
| 构建流程 | [[03_moc/CMake-build-walkthrough]] | CMake 配置→编译→烧录全流程 |
| 运行时初始化 | [[03_moc/Robot-Init-Walkthrough]] | main()→ThreadX→Robot_Init()→各模块初始化 |
| 哨兵初始化详解 | [[03_moc/Sentry-Init]] | 哨兵双板（云台板+底盘板）各自展开 |
| 步兵3号初始化详解 | [[03_moc/Infantry3-Init]] | 步兵3号单板展开 |
| 踩坑记录 | [[04_notes/env_pitfalls]] | 环境配置与编译问题 |
| 变更日志 | [[04_notes/changelog]] | 知识库每次更新记录 |

---

## 02 层覆盖范围

### 已完成

| 目录 | 文件 | 对应源码 |
|------|------|---------|
| `board/bsp/CAN/` | bsp_can, bsp_can_task | CAN 硬件抽象 + 收发后台任务 |
| `board/bsp/DWT/` | bsp_dwt | DWT 计时器（GetDeltaT/Delay） |
| `board/bsp/USB/` | bsp_usb_cdc | CherryUSB CDC 虚拟串口 |
| `board/bsp/UART/` | bsp_uart | 串口驱动 |
| `board/bsp/PWM/` | bsp_pwm | PWM 底层抽象 |
| `board/bsp/BEEP/` | bsp_beep | 蜂鸣器（音调/音量） |
| `board/bsp/LED/` | bsp_led | LED |
| `modules/MOTOR/` | motor_base, motor_dji, motor_damiao, power_control | 电机抽象 + 大疆 + 达妙 + 功率控制 |
| `modules/REMOTE/` | module_remote, sbus, dt7 | 遥控器（SBUS/DT7） |
| `modules/VISION/` | module_vision | 视觉通信（USB CDC） |
| `modules/OFFLINE/` | module_offline-c, module_offline-h | 离线检测 |
| `apps/` | app_init, config-cmake, CMakeLists-txt, generate_headers-cmake | 应用层 CMake + 转发层 |
| `apps/sentry/gimbal_board/` | robot_control, gimbal_func/, shoot_func/, robot_func/ | 哨兵云台板全部应用层 |
| `apps/sentry/chassis_board/` | robot_control, chassis_func/, robot_func/ | 哨兵底盘板全部应用层 |
| `apps/sentry/` | robot-cmake, signal-flow | 哨兵 CMake 对比 + 信号流图 |
| `apps/infantry3/single_board/` | robot_control, gimbal_func/, shoot_func/, chassis_func/, robot_func/ | 步兵3号全部应用层 |
| `apps/infantry3/` | signal-flow | 步兵3号信号流图 |
| CMake 相关 | board/dji_c/ 下 4 个, modules/, robot/, utils/, build | 构建系统全链路 |

### 尚未覆盖

| 源码模块 | 备注 |
|---------|------|
| `modules/INS/` | 姿态解算（四元数 EKF 黑盒） |
| `modules/BMI088/` | SPI 驱动 + 温控 + 标定 |
| `modules/BOARD_COMM/` | 板间 CAN 通信 |
| `modules/REFEREE/` | 裁判系统协议解析 |
| `modules/BOOTLOADER/` | IAP bootloader |
| `modules/SUPERCAP/` | 超级电容 |
| `utils/` | user_lib.c/h（数学工具、链表等） |

---

## 关键设计约定

1. **BSP 层只讲硬件接口**，不展开具体外设应用（如蜂鸣器具体调用放 `bsp_beep.md`，不在 `bsp_pwm.md` 里讲）
2. **模块层讲协议封装 + 线程管理**，不讲"在整个系统中的位置"
3. **应用层讲控制逻辑**，以具体兵种/板型为单位
4. **数据结构跟着模块走**，不放在 BSP 层（如 SendPacket/ReceivePacket 在 module_vision 层）
5. **CMake 文档统一用哨兵云台板示例**（ROBOT=sentry BOARD=gimbal）
6. **步兵3号与哨兵互相对照**，用对比表突出差异
7. **templates 目录不写笔记**
8. **源码加注释时直接改源码文件**，同时更新对应 02 层笔记

---

## 已标注的 Bug

| Bug | 位置 | 说明 |
|-----|------|------|
| `dji_output_to_torque` | `motor_dji.c` | 函数名是 torque 但只返回电流值（IntegerToCurrent 结果），漏乘 Kt_gear |
| PowerControl 改 output 未更新 sender_assignment | `power_control.c` | 功率控制修改了 output 但没写回 sender_assignment 缓冲区，Flush 发的是旧值 |

---

## 源码项目结构速览

```
mas_embedded_threadx/
├── board/
│   ├── dji_c/          # 大疆 C 板（STM32F407）
│   └── damiao_h7/      # 达妙 H7 板（STM32H723）
├── modules/
│   ├── BMI088/         # IMU 驱动（SPI + 温控 + 标定）
│   ├── INS/            # 姿态解算（四元数 EKF）
│   ├── MOTOR/          # 电机抽象 + 大疆 + 达妙 + 功率控制
│   ├── REMOTE/         # 遥控器（SBUS / DT7）
│   ├── VISION/         # 视觉通信（USB CDC）
│   ├── OFFLINE/        # 离线检测
│   ├── BOARD_COMM/     # 板间通信
│   ├── REFEREE/        # 裁判系统
│   └── ...
├── apps/
│   ├── app_init.c      # 转发到各兵种的 robot_control_init()
│   ├── sentry/
│   │   ├── gimbal_board/   # 哨兵云台板
│   │   └── chassis_board/  # 哨兵底盘板
│   └── infantry3/
│       └── single_board/   # 步兵3号单板
├── utils/
│   └── user_lib.c/h   # 数学工具、链表等
└── CMakeLists.txt
```

---

## Obsidian 使用方式

### 安装

1. 下载安装 Obsidian（windows 版安装包后续会放到 `00_raw/` 下）
2. 打开 Obsidian，选择 **打开文件夹作为仓库**（Open folder as vault）
3. 选择 `rm-electric-knowledge-base/` 文件夹

### 必改设置

打开后进入 **设置（Ctrl+,）**：

| 设置项 | 路径 | 改成 | 原因 |
|--------|------|------|------|
| 内部链接类型 | 文件与链接 → 内部链接格式 | **基于仓库根目录的绝对路径** | 项目中有多个同名 `CMakeLists.txt`，用绝对路径避免歧义 |
| 附件默认位置 | 文件与链接 → 附件默认位置 | **当前文件夹** | 图片等附件和笔记放一起 |

### 如何阅读

1. 从 `03_moc/CMake-build-walkthrough.md` 开始，它是全流程入口
2. 按阅读顺序点击 `[[wikilink]]` 逐层深入到 02 层代码孪生笔记
3. 02 层笔记中首次遇到某个语法时，点击链接跳转到 01 层看讲解
4. `Ctrl+G` 查看知识图谱，`Ctrl+Shift+左键` 在新标签页打开链接

### 如何添加笔记

- 代码孪生笔记：复制 `02_code_twin/_templates/code-twin-template.md`，按结构填写
- 新笔记中引用其他笔记用 `[[路径/文件名]]` 或 `[[路径/文件名#标题]]` 跳转到具体段落
- 链接放在代码块外面（Obsidian 不渲染代码块内的 wikilink）

---

## 下一步待办

1. **INS 层展开**：BMI088 数据手册要点 + module_ins.c 黑盒化（输入：陀螺仪/加速度计原始数据，输出：四元数/欧拉角/运动加速度），算法部分当黑盒只关注输入输出
2. `_index.md` 更新为完整目录树
3. 继续补齐未覆盖模块（BOARD_COMM / REFEREE / utils 等）
