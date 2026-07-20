# module_ins.c/h — 姿态解算模块

`modules/INS/module_ins.{c,h}` / `QuaternionEKF.{c,h}`

## 一句话

从 BMI088 读取加速度和角速度，经四元数 EKF 融合后输出姿态（四元数 / 欧拉角 / 运动加速度），同时承担 BMI088 温控。EKF 算法本身当黑盒处理，只关注输入输出。

## 对外接口

| 函数 | 作用 | 返回 |
|------|------|------|
| `Module_INS_Init()` | 初始化 EKF + 创建 INS 线程 | void |
| `Module_INS_get()` | 获取姿态数据指针 | `const Ins_t *`，未初始化返回 NULL |

整个模块对外只暴露这两个函数。所有姿态数据通过 `Ins_t *` 指针读取，外部只读不可写。

## 黑盒模型

```
输入（来自 BMI088）                    输出（Ins_t 结构体）
┌───────────────────┐                ┌────────────────────────┐
│ gyro[3]  (dps)    │                │ q[4]         四元数(wxyz) │
│ acc[3]   (m/s²)   │──→ [EKF 黑盒] ──→│ euler_angle[3] 欧拉角(°) │
│ temp     (℃)     │                │ euler_rad[3]  欧拉角(rad)│
│                   │                │ YawTotalAngle_rad  yaw圈数│
│ dt (DWT计时)      │                │ MotionAccel_b[3] 机体系  │
└───────────────────┘                │ MotionAccel_n[3] 导航系  │
                                     └────────────────────────┘
```

**为什么需要两种传感器**：陀螺仪积分可以得到姿态，但会漂移；加速度计测量重力方向，可以修正 roll/pitch 但无法修正 yaw（重力方向不随 yaw 变化）。EKF 用陀螺仪做预测、加速度计做校正，融合两者的优点。

**为什么不用磁力计**：机器人车架上有大量铁质结构和电机，磁场干扰严重，磁力计数据不可靠。因此纯 6 轴融合，**yaw 角会缓慢漂移**。实际使用中靠云台编码器或视觉修正 yaw，不依赖 IMU 的绝对 yaw。

## EKF 黑盒

`QuaternionEKF.{c,h}` — 作者 Wang Hongxi，版本 V1.2.0

### 输入

```c
void IMU_QuaternionEKF_Update(float gx, float gy, float gz,    // 陀螺仪三轴 dps
                               float ax, float ay, float az,    // 加速度计三轴 m/s²
                               float dt);                       // 时间间隔秒
```

### 输出（全局变量 `QEKF_INS`）

| 字段 | 含义 |
|------|------|
| `QEKF_INS.q[4]` | 四元数估计值 (w, x, y, z) |
| `QEKF_INS.Roll` / `Pitch` / `Yaw` | 欧拉角（度） |
| `QEKF_INS.YawTotalAngle` | yaw 累计角度（度，含圈数） |
| `QEKF_INS.YawRoundCount` | yaw 圈数计数 |
| `QEKF_INS.GyroBias[3]` | 陀螺仪零偏估计值 |
| `QEKF_INS.ConvergeFlag` | 收敛标志（0=未收敛, 1=已收敛） |

### 初始化参数

```c
IMU_QuaternionEKF_Init(init_quaternion,
    10.0f,       // Q1: 四元数更新过程噪声
    0.001f,      // Q2: 陀螺仪零偏过程噪声
    1000000.0f,  // R:  加速度计量测噪声
    0.9996f,     // lambda: 渐消因子
    0.0f);       // lpf: 低通滤波系数
```

### 内部机制（不展开，仅列出）

EKF 内部包含：状态预测（四元数微分方程）、协方差预测、加速度计观测更新、卡方检验异常值剔除、自适应渐消因子。这些细节不在本文展开，需要时直接看 `QuaternionEKF.c` 源码。

### 初始四元数

`InitQuaternion()` 在初始化时读 100 次加速度计取平均，利用重力方向计算初始 roll 和 pitch（yaw 置 0）：

```
静止时加速度计测到的 = 重力向量
→ 归一化后与导航系重力 (0,0,1) 求夹角和旋转轴
→ 轴角公式转四元数
```

## INS 线程

### 创建

```c
// Module_INS_Init() 中
tx_thread_create(&g_ins_task, "INS Task", ins_task_entry, ...,
    INS_TASK_PRIORITY,  // 优先级 7（较高）
    TX_AUTO_START);     // 自动启动
```

### 每周期执行流程

```
ins_task_entry() — 每个线程 tick 循环一次
  │
  ├── 1. dt = BSP_DWT_GetDeltaT()        获取精确时间间隔
  ├── 2. Module_BMI088_get_accel()        读加速度
  ├── 3. Module_BMI088_get_gyro()         读角速度
  ├── 4. IMU_Param_Correction()           安装误差修正（标度因子+旋转矩阵）
  ├── 5. IMU_QuaternionEKF_Update()       EKF 更新（黑盒）
  ├── 6. 复制 q → ins.q                   四元数输出
  ├── 7. BodyFrameToEarthFrame()          计算基向量 xn/yn/zn
  ├── 8. 运动加速度 = acc - gravity_b      去重力分量 + 低通滤波
  ├── 9. 欧拉角/圈数输出                   从 QEKF_INS 复制
  ├── 10. Module_BMI088_get_temp()        读温度
  ├── 11. Module_BMI088_temp_ctrl()       温控 PID 执行
  └── tx_thread_sleep(1)                  休眠一个 tick
```

> 步骤 10-11 是 BMI088 温控，挂在 INS 线程里因为它是周期性任务且需要 IMU 数据。详见 [[02_code_twin/modules/BMI088/module_bmi088#温控 PID]]。

## 安装误差修正

`IMU_Param_Correction()` 对陀螺仪和加速度计数据做两步修正：

1. **标度因子**：`gyro[i] *= scale[i]`，修正传感器灵敏度误差
2. **旋转矩阵**：根据 `IMU_Param.Yaw/Pitch/Roll` 构建方向余弦矩阵，将数据从 IMU 安装坐标系转换到机体坐标系

> `IMU_Param` 默认 scale=1.0、角度=0（无修正）。如果 IMU 安装有倾斜或旋转，在这里配置安装角度。

## Ins_t 结构体

```c
typedef struct {
    float q[4];                // 四元数 (w, x, y, z)
    float MotionAccel_b[3];   // 机体系运动加速度（去重力）
    float MotionAccel_n[3];   // 导航系运动加速度
    float AccelLPF;           // 加速度低通滤波系数
    float xn[3], yn[3], zn[3];// 机体系基向量在导航系的表示
    float euler_angle[3];     // roll, pitch, yaw（度）
    float euler_rad[3];       // roll, pitch, yaw（弧度）
    float YawTotalAngle_rad;  // yaw 累计角度（弧度，含圈数）
    int32_t YawRoundCount;    // yaw 圈数
    float dt;                 // 采样间隔
    uint32_t dwt_cnt;         // DWT 计数器
    uint8_t initialized;      // 初始化标志
} Ins_t;
```

应用层通过 `Module_INS_get()` 拿到 `const Ins_t *`，直接读字段。

## 被谁调用

| 调用者 | 用什么字段 | 链接 |
|--------|-----------|------|
| `Robot_Init()` | `Module_INS_Init()` 一次性初始化 | [[03_moc/Robot-Init-Walkthrough#Module_Init()]] |
| 哨兵云台 `gimbal_func` | `ins->YawTotalAngle_rad`（yaw 反馈）、`ins->euler_rad[1]`（pitch 反馈）、`ins->q[4]`（发视觉） | [[02_code_twin/apps/sentry/gimbal_board/gimbal_func/gimbal_func]] |
| 哨兵云台 `robot_control` | `ins->q[4]` 填入视觉 SendPacket | [[02_code_twin/apps/sentry/gimbal_board/robot_control]] |
| 步兵3号 `gimbal_func` | 同哨兵，yaw/pitch 反馈 + 视觉四元数 | [[02_code_twin/apps/infantry3/single_board/gimbal_func/gimbal_func]] |
| 步兵3号 `robot_control` | `ins->q[4]` 填入视觉 SendPacket | [[02_code_twin/apps/infantry3/single_board/robot_control]] |

## 与 BMI088 的耦合

INS 线程直接调用 `Module_BMI088_get_accel/gyro/temp` + `temp_ctrl`，两个模块在代码层面是耦合的——没有 BMI088，INS 无法独立工作。

> **两种方案对比**：
> - **C 板 + BMI088**（本项目）：自己读原始数据 → 自己跑 EKF 解算 → 自己做温控。代码量大但完全可控。
> - **成品 IMU（如 WT606）**：固件内部已做好解算，通信协议直接输出四元数/欧拉角。代码量小但无法调整算法。
>
> 当前先保持耦合，没有更好的拆分方案。
