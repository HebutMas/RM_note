# module_wt606.c/h — WT606 串口 IMU 模块

`modules/WT606/module_wt606.{c,h}`

## 一句话

通过 UART 读取 WT606 / WT901 系列 IMU，直接获得加速度、角速度、欧拉角和四元数（传感器固件内部已解算），无需外部 EKF。

## 与 INS 模块的对比

| 特性 | INS（BMI088 + EKF） | WT606（串口 IMU） |
|------|:---:|:---:|
| 传感器 | BMI088（SPI） | WT606/WT901 内置（UART） |
| 姿态解算 | 代码内 EKF | 传感器固件内部解算 |
| 输出 | 原始 acc/gyro → EKF → 姿态 | 直接输出欧拉角/四元数 |
| 精度 | 可调算法参数 | 固件固定，不可调 |
| 接口 | SPI（高速） | UART（921600 bps） |
| 温控 | 有（PID 加热） | 无 |
| 代码量 | 大 | 小 |

## 对外接口

| 函数 | 作用 | 返回 |
|------|------|------|
| `Module_WT606_Init()` | 初始化 UART + 创建 WT606 接收线程 | void |
| `Module_WT606_Get()` | 获取 IMU 数据指针 | `const Module_WT606_Device_t *` |
| `Module_WT606_Get_offline_state()` | 获取离线状态 | `uint8_t` |

## 数据结构

```c
typedef struct {
    float   acc[3];               // 加速度 (m/s²)
    float   gyro_dps[3];          // 角速度 (°/s)
    float   gyro_rps[3];          // 角速度 (rad/s)
    float   Euler_degree[3];      // 欧拉角 (°) — roll / pitch / yaw
    float   Euler_rad[3];         // 欧拉角 (rad)
    float   quat[4];              // 四元数 [w, x, y, z]
    float   temperature;          // 温度 (°C)
    float   YawTotalAngle_degree; // yaw 连续总角度 (°)
    float   YawTotalAngle_rad;    // yaw 连续总角度 (rad)
    int32_t YawRoundCount;        // yaw 过零圈数
    // ...
} Module_WT606_Device_t;
```

## 通信协议

每帧 11 字节，UART 921600 bps：

```
0x55 | TYPE | DATA1L | DATA1H | DATA2L | DATA2H | DATA3L | DATA3H | DATA4L | DATA4H | SUM
```

| TYPE | 含义 | 数据内容 |
|------|------|---------|
| 0x51 | 加速度 | Ax, Ay, Az, 温度 |
| 0x52 | 角速度 | Wx, Wy, Wz, 电压 |
| 0x53 | 欧拉角 | Roll, Pitch, Yaw, 版本号 |
| 0x59 | 四元数 | Q0(w), Q1(x), Q2(y), Q3(z) |

**校验和**：`SUM = (0x55 + TYPE + DATA1L + ... + DATA4H) & 0xFF`

**数据转换**：`raw = (int16_t)((int16_t)DATAH << 8 | DATAL)` → `物理值 = raw / 32768 × 量程`

- 加速度：量程 ±16g，`ACC_SCALE = 16 × 9.8 / 32768 ≈ 0.004785`
- 角速度：量程 ±2000°/s，`GYRO_SCALE = 2000 / 32768 ≈ 0.06104`
- 欧拉角：量程 ±180°，`ANGLE_SCALE = 180 / 32768 ≈ 0.005493`
- 四元数：`QUAT_SCALE = 1 / 32768`

## yaw 过零累计

WT606 输出的 yaw 范围是 [-180°, 180°]，越过 ±180° 边界时会跳变。模块内部通过比较当前帧与上一帧的差值判断过零方向，累计 `YawRoundCount`：

```c
delta_yaw = current - last;
if (delta_yaw > 180)  YawRoundCount--;  // 从 -180 跳到 +179（逆时针过零）
if (delta_yaw < -180) YawRoundCount++;  // 从 +179 跳到 -180（顺时针过零）
YawTotalAngle = yaw + 360 × YawRoundCount;
```

## 初始化流程

```c
void Module_WT606_Init(void) {
    WT606_UART.Init.BaudRate = 921600;
    HAL_UART_Init(&WT606_UART);               // 重设 UART 波特率

    wt606_device.uart_dev = BSP_UART_Device_Init(&config);  // 注册 UART 设备
    wt606_device.offline_dev = Module_Offline_register(&config);  // 注册离线检测

    tx_thread_create(&wt606_thread, "wt606", wt606_task_entry, ...);  // 创建接收线程
}
```

## 线程

接收线程阻塞等待 UART 数据，扫描帧头 `0x55`，校验和通过后按 TYPE 分发解析。

## 被谁调用

| 调用者 | 用途 |
|--------|------|
| `Robot_Init()` 的 `MODULE_Init()` | 一次性初始化 |
| 应用层控制代码 | 通过 `Module_WT606_Get()` 读姿态数据 |

> WT606 和 INS 是**替代关系**而非协同关系。项目中根据板型选择使用哪个：WT606 用于不需要高精度姿态的场合（如底盘），INS（BMI088+EKF）用于需要精确姿态控制的场合（如云台）。
