# BMI088 IMU 数据手册提炼

> 出处：BMI088 Data sheet (BST-BMI088-DS000-19, Rev 1.9, Jan 2024)，[[00_raw/hardware/BMI088_Datasheet.pdf|BMI088 Datasheet PDF]]
> 结合 [[01_extracted/hardware/c-board-resources#BMI088 六轴惯性测量单元|C 板硬件资源]] 和项目源码 `modules/BMI088/`

## 一句话

BMI088 是 Bosch 的 6 轴 IMU（3 轴加速度计 + 3 轴陀螺仪），**加速度计和陀螺仪是两个独立芯片**，各有独立 CS、独立 Chip ID、独立 I2C 地址。软件上要当两个 SPI 设备操作。

---

## 接口选择 — 加速度计的 I2C→SPI 切换坑

> 这是 BMI088 最经典的坑。出处：[[00_raw/hardware/BMI088_Datasheet.pdf#page=13|数据手册第 13 页]] 和 [[00_raw/hardware/BMI088_Datasheet.pdf#page=45|第 45 页]]。

**陀螺仪**和**加速度计**的接口选择机制完全不同：

|              | 陀螺仪 (Gyro)                    | 加速度计 (Accel)                                                                       |
| ------------ | ----------------------------- | ---------------------------------------------------------------------------------- |
| **上电默认接口**   | 由 PS 引脚决定（GND=SPI, VDDIO=I2C） | **永远是 I2C**，不管 PS 引脚                                                               |
| **如何切到 SPI** | PS 拉低即可，上电就是 SPI              | 需要检测到 **CSB1 引脚的上升沿**才切换                                                           |
| **切换方法**     | 硬件接线决定，无需软件干预                 | 必须做一次 **dummy SPI 读操作**（如读 ACC_CHIP_ID），返回值无效，但会触发 CSB1 上升沿，此后加速度计锁定在 SPI 模式直到下次断电 |

> 出处：[[00_raw/hardware/BMI088_Datasheet.pdf#page=13|数据手册第 13 页]] 原文——
> "The accelerometer part starts in I2C mode. It will stay in I2C mode until it detects a rising edge on the CSB1 pin (chip select of the accelerometer), on which the accelerometer part switches to SPI mode and stays in this mode until the next power-up-reset."
> "To change the accelerometer to SPI mode in the initialization phase, the user could perform a dummy SPI read operation, e.g. of register ACC_CHIP_ID (the obtained value will be invalid)."
>
> **中文翻译**：
> "加速度计部分以 I2C 模式启动。它将保持 I2C 模式，直到检测到 CSB1 引脚（加速度计片选）上的上升沿，此时加速度计切换到 SPI 模式，并保持该模式直到下一次上电复位。"
> "要在初始化阶段将加速度计切换到 SPI 模式，用户可以执行一次 dummy SPI 读操作，例如读取 ACC_CHIP_ID 寄存器（读到的值无效）。"

**项目源码中的对应**：`module_bmi088.c` 初始化时连续两次读 `BMI088_ACC_CHIP_ID`，第一次读到的是无效值（触发 I2C→SPI 切换），等 300ms 后再读一次验证 Chip ID = 0x1E：

```c
// 第一次读：dummy read，触发加速度计从 I2C 切到 SPI
_bmi088_read_reg(bmi088_device.acc_device, BMI088_ACC_CHIP_ID, &whoami, 1, 1, TX_NO_WAIT);
// whoami 此时无效，丢弃

BSP_DWT_Delay(BMI088_COM_WAIT_SENSOR_TIME);  // 等 300ms

// 第二次读：此时已是 SPI 模式，读到正确的 Chip ID
_bmi088_read_reg(bmi088_device.acc_device, BMI088_ACC_CHIP_ID, &whoami, 1, 1, TX_NO_WAIT);
// whoami 应为 0x1E
```

> **这就是你记忆中"得先拿 I2C 读一句才能转成 SPI"的特性**——严格来说不是"拿 I2C 读"，而是加速度计上电默认在 I2C 模式，需要一次 SPI 读取操作（拉低再释放 CSB1 产生上升沿）才能切换到 SPI。这次读到的数据是无效的。

### PS 引脚

> 出处：[[00_raw/hardware/BMI088_Datasheet.pdf#page=52|数据手册第 52 页]] Pin Description

| Pin | 名称 | SPI 模式 | I2C 模式 |
|-----|------|---------|---------|
| 7 | PS | GND | VDDIO |

PS 引脚**只控制陀螺仪**的接口选择。加速度计不受 PS 引脚影响，上电永远是 I2C，靠 CSB1 上升沿切 SPI。

---

## SPI 读写协议

> 出处：[[00_raw/hardware/BMI088_Datasheet.pdf#page=45|数据手册第 6.1 章（第 45-48 页）]]

SPI 最大 10MHz，兼容 CPOL=0/CPHA=0 和 CPOL=1/CPHA=1 两种模式（自动检测）。C 板用 SPI1（PB3/PA7/PB4）。

### 读写时序差异

> 出处：[[00_raw/hardware/BMI088_Datasheet.pdf#page=45|数据手册第 45-46 页]]（6.1 章节开头）和 [[00_raw/hardware/BMI088_Datasheet.pdf#page=48|第 48 页]]（6.1.2 章节）

| | 陀螺仪 (Gyro) | 加速度计 (Accel) |
|---|---|---|
| **读操作前导** | 1 字节（寄存器地址） | **2 字节**（寄存器地址 + 1 字节 dummy） |
| **dummy 字节内容** | 无 | 不可预测，必须丢弃 |
| **写操作** | 正常 | 正常（无 dummy） |

> 出处：[[00_raw/hardware/BMI088_Datasheet.pdf#page=48|数据手册第 48 页]] 原文——
> "In case of read operations of the accelerometer part, the requested data is not sent immediately, but instead first a dummy byte is sent, and after this dummy byte the actual requested register content is transmitted."
> "For example, to read the accelerometer values in SPI mode, the user has to read 7 bytes, starting from address 0x12 (ACC data). From these bytes the user must discard the first byte and finds the acceleration information in byte #2 – #7."
>
> **中文翻译**：
> "在读操作中，加速度计部分不会立即发送请求的数据，而是先发送一个 dummy 字节，在这个 dummy 字节之后才传输实际的寄存器内容。"
> "例如，要在 SPI 模式下读取加速度值，用户必须从地址 0x12 开始读取 7 个字节。其中第一个字节必须丢弃，加速度信息在第 2~7 字节中（对应地址 0x12~0x17 的内容）。"

**数据帧格式**：

```
写操作:  [CS↓] [reg & 0x7F] [data...] [CS↑]

读操作（陀螺仪）:  [CS↓] [0x80 | reg] [data...] [CS↑]
读操作（加速度计）: [CS↓] [0x80 | reg] [dummy] [data...] [CS↑]
                              ↑ 多 1 字节 dummy，内容无效必须丢弃
```

源码中 `_bmi088_read_reg()` 用 `is_accel` 参数区分：`dummy_bytes = is_accel ? 2 : 1`。

### 写操作间隔

> 出处：[[00_raw/hardware/BMI088_Datasheet.pdf#page=45|数据手册第 45 页]]

两次连续写操作之间必须等待至少 **2μs**（正常模式）或 **1000μs**（suspend 模式）。

---

## 两个独立设备

> 出处：[[00_raw/hardware/BMI088_Datasheet.pdf#page=24|数据手册第 24 页]] "Register Map" 和 [[00_raw/hardware/BMI088_Datasheet.pdf#page=48|第 48 页]] I2C 地址

| | 加速度计 (Accel) | 陀螺仪 (Gyro) |
|---|---|---|
| **Chip ID** | 0x1E | 0x0F |
| **I2C 地址** | 0x18 (SDO1=GND) / 0x19 (SDO1=VDDIO) | 0x68 (SDO2=GND) / 0x69 (SDO2=VDDIO) |
| **C 板 CS 引脚** | PA4 (CSB1) | PB0 (CSB2) |
| **量程** | ±3 / ±6 / ±12 / ±24 g | ±125 / ±250 / ±500 / ±1000 / ±2000 dps |
| **项目配置** | ±6g，800Hz ODR | ±2000 dps，1000Hz ODR / 116Hz BW |
| **数据寄存器** | 0x12~0x17 (X/Y/Z 各 2 字节) | 0x02~0x07 (X/Y/Z 各 2 字节) |
| **温度寄存器** | 0x22~0x23（在 accel 上读） | — |
| **上电后状态** | **suspend 模式**，需手动唤醒 | normal 模式 |

> 出处：数据手册第 13 页——
> "After the POR the gyroscope is in normal mode, while the accelerometer is in suspend mode."
>
> **中文翻译**："上电复位后，陀螺仪处于正常模式，而加速度计处于挂起模式。"

---

## 数据获取

### 加速度计

> 寄存器：`ACC_XOUT_L(0x12)` 起连续 6 字节 → X_L, X_H, Y_L, Y_H, Z_L, Z_H
> 出处：[[00_raw/hardware/BMI088_Datasheet.pdf#page=28|数据手册第 28 页]] 寄存器表，源码 `module_bmi088.c → Module_BMI088_get_accel()`

```c
uint8_t buf[6];
_bmi088_read_reg(acc_device, BMI088_ACCEL_XOUT_L, buf, 6, 1, TX_WAIT_FOREVER);

for (uint8_t i = 0; i < 3; i++) {
    int16_t raw = (int16_t)((buf[2*i+1] << 8) | buf[2*i]);  // 大端拼接
    float   val = BMI088_ACCEL_6G_SEN * (float)raw;          // 乘灵敏度 → m/s²
}
```

**灵敏度**（出处：[[00_raw/hardware/BMI088_Datasheet.pdf#page=8|数据手册第 8 页]] + 源码 `BMI088_reg.h`）：

| 量程 | 灵敏度常数 (m/s²/LSB) | 等效 (LSB/g) |
|------|----------------------|-------------|
| ±3g | 0.000897 | 10920 |
| ±6g | 0.001794 | 5460 |
| ±12g | 0.003589 | 2730 |
| ±24g | 0.007178 | 1365 |

> 灵敏度常数已经把 g 换算进去了，直接 `raw × 灵敏度` 就是 m/s²，不用再乘 9.81。项目配置 ±6g。

### 陀螺仪

> 寄存器：`GYRO_X_L(0x02)` 起连续 6 字节
> 出处：[[00_raw/hardware/BMI088_Datasheet.pdf#page=37|数据手册第 37 页]] 寄存器表，源码 `module_bmi088.c → Module_BMI088_get_gyro()`

```c
uint8_t buf[6];
_bmi088_read_reg(gyro_device, BMI088_GYRO_X_L, buf, 6, 0, TX_WAIT_FOREVER);

for (uint8_t i = 0; i < 3; i++) {
    int16_t raw = (int16_t)((buf[2*i+1] << 8) | buf[2*i]);
    float   val = BMI088_GYRO_2000_SEN * (float)raw;   // → dps
}
```

**灵敏度**（出处：源码 `BMI088_reg.h`）：

| 量程 | 灵敏度常数 (dps/LSB) | 等效 (LSB/°/s) |
|------|---------------------|----------------|
| ±2000 dps | 0.001065 | 16.384 |
| ±1000 dps | 0.000533 | 32.768 |
| ±500 dps | 0.000266 | 65.536 |
| ±250 dps | 0.000133 | 131.072 |
| ±125 dps | 0.000067 | 262.144 |

> 项目配置 ±2000 dps。陀螺仪输出单位是 **dps**（度/秒），不是 rad/s。

### 温度

> 寄存器：`TEMP_M(0x22)` + `TEMP_L(0x23)`，在**加速度计设备**上读
> 出处：[[00_raw/hardware/BMI088_Datasheet.pdf#page=30|数据手册第 30 页]] 寄存器表，源码 `module_bmi088.c → Module_BMI088_get_temp()`

```c
uint8_t buf[2];
_bmi088_read_reg(acc_device, BMI088_TEMP_M, buf, 2, 1, TX_WAIT_FOREVER);

int16_t raw = (((int16_t)buf[0] << 3) | (buf[1] >> 5));  // 11-bit
if (raw > 1023) raw -= 2048;                              // 补码转有符号

temperature = (float)raw * 0.125f + 23.0f;  // 0.125℃/LSB，偏移23℃
```

> 温度用于温控 PID（TIM10_CH1 加热），保持 BMI088 恒温（默认 35℃），减少温漂。出处：[[01_extracted/hardware/c-board-resources#陀螺仪温控 PWM|C 板资源清单]]

---

## 初始化序列

> 出处：[[00_raw/hardware/BMI088_Datasheet.pdf#page=13|数据手册第 13 页]] "Quick Start Guide"，源码 `module_bmi088.c` 初始化表

### 加速度计上电流程

> 出处：数据手册第 13 页原文——
> "a. Power up the sensor  
> b. Wait 1 ms  
> c. Enter normal mode by writing '4' to ACC_PWR_CTRL  
> d. Wait for 450 microseconds"
>
> **中文翻译**：
> a. 给传感器上电
> b. 等待 1 ms
> c. 向 ACC_PWR_CTRL 写入 4，进入正常模式
> d. 等待 450 微秒

### 寄存器初始化表

**加速度计**（源码 `BMI088_Accel_Init_Table`）：

| 序号 | 寄存器 | 值 | 含义 |
|------|--------|---|------|
| 1 | `ACC_PWR_CTRL (0x7D)` | 0x04 | 加速度计上电（acc_en=1） |
| 2 | `ACC_PWR_CONF (0x7C)` | 0x00 | 正常模式（非 suspend） |
| 3 | `ACC_CONF (0x40)` | 0x8B | normal 模式 + 800Hz ODR + must-set bit |
| 4 | `ACC_RANGE (0x41)` | 0x01 | ±6g |
| 5 | `INT1_IO_CTRL (0x53)` | 0x0A | INT1 推挽 + 低电平有效 |
| 6 | `INT_MAP_DATA (0x58)` | 0x04 | DRDY 中断映射到 INT1 |

**陀螺仪**（源码 `BMI088_Gyro_Init_Table`）：

| 序号 | 寄存器 | 值 | 含义 |
|------|--------|---|------|
| 1 | `GYRO_RANGE (0x0F)` | 0x00 | ±2000 dps |
| 2 | `GYRO_BANDWIDTH (0x10)` | 0x82 | 1000Hz ODR / 116Hz BW + must-set bit |
| 3 | `GYRO_LPM1 (0x11)` | 0x00 | 正常模式 |
| 4 | `GYRO_CTRL (0x15)` | 0x80 | DRDY 使能 |
| 5 | `INT3_INT4_IO_CONF (0x16)` | 0x00 | INT3 推挽 + 低电平有效 |
| 6 | `INT3_INT4_IO_MAP (0x18)` | 0x01 | DRDY 映射到 INT3 |

> 每条写完后回读校验，不匹配则记录错误码。出处：源码 `_bmi088_write_init_table()`

---

## 软复位

> 出处：[[00_raw/hardware/BMI088_Datasheet.pdf#page=18|数据手册第 18 页]] "4.8 Soft-Reset"

两个芯片各自独立软复位，写 0xB6 到对应寄存器：

| | 寄存器 | 延时 |
|---|---|---|
| 加速度计 | `ACC_SOFTRESET (0x7E)` | 1 ms |
| 陀螺仪 | `GYRO_SOFTRESET (0x14)` | 30 ms |

> 出处：数据手册第 18 页原文——
> "The soft-reset performs a fundamental reset to the device, which is largely equivalent to a power cycle."
>
> **中文翻译**："软复位对设备执行一次彻底的复位，基本等同于重新上电。"
>
> 软复位后所有寄存器恢复默认值，需重新走初始化表。注意加速度计软复位后会回到 I2C 模式，需要再做一次 dummy SPI 读操作。

---

## C 板硬件连接

> 详见 [[01_extracted/hardware/c-board-resources#BMI088 六轴惯性测量单元|C 板资源清单]]。出处：C 板用户手册第 14 页

| 功能 | 引脚 | 说明 |
|------|------|------|
| SPI1_CLK | PB3 | 共用时钟 |
| SPI1_MOSI | PA7 | 共用 MOSI |
| SPI1_MISO | PB4 | 共用 MISO |
| CSB1 (Accel) | PA4 | 加速度计片选 |
| CSB2 (Gyro) | PB0 | 陀螺仪片选 |
| INT1_Accel | PC4 | 加速度计数据就绪中断 |
| INT1_Gyro | PC5 | 陀螺仪数据就绪中断 |
| 温控加热 | PF6 (TIM10_CH1) | PWM 加热，建议比板载温度高 15~20℃ |

> BMI088 引脚对应：CSB1=Pin14（加速度计片选），CSB2=Pin5（陀螺仪片选），PS=Pin7（协议选择，C 板接 GND 走 SPI）。出处：数据手册第 52 页 Pin Description

---

## 关键参数速查

> 出处：[[00_raw/hardware/BMI088_Datasheet.pdf#page=7|数据手册第 7-8 页]] Electrical Specification + Bosch 官方页面

| 参数 | 加速度计 | 陀螺仪 |
|------|---------|--------|
| 分辨率 | 16 bit | 16 bit |
| 量程 | ±3/±6/±12/±24 g | ±125/±250/±500/±1000/±2000 dps |
| ODR | 12.5~1600 Hz | 100~2000 Hz |
| 通信接口 | SPI (max 10MHz) / I2C (0x18/0x19) | SPI (max 10MHz) / I2C (0x68/0x69) |
| Chip ID | 0x1E | 0x0F |
| 零偏 | ±20 mg | ±1 dps |
| 零漂温系数 | ±0.2 mg/K | ±0.015 dps/K |
| 工作温度 | -40~+85℃ | -40~+85℃ |
| 供电 | 2.4~3.6V | 2.4~3.6V |
| 工作电流 | 150 μA | 5 mA |

---

## 传感轴方向定义

> 出处：[[00_raw/hardware/BMI088_Datasheet.pdf#page=56|数据手册第 56 页]] 8.3 Sensing axes orientation

### 芯片坐标系（BMI088 数据手册定义）

Bosch 标准坐标系，以芯片 Top View 为参考：

```
       Y+ (短边方向)
        ↑
        |
   Pin1 ●──→ X+ (长边方向)
   (底左角)
        Z+ = 垂直芯片表面向外（指向观察者）
```

- **Pin 1 标记**：芯片一角有黑色圆点或凹口标识
- **Pin 1 在左下角时**：X+ 沿芯片长边向右，Y+ 沿芯片短边向上，Z+ 垂直芯片表面向外
- 加速度计：沿 X/Y/Z 正方向加速时输出正数
- 陀螺仪：绕 X/Y/Z 正方向按右手定则旋转时输出正数（如绕 Z+ 逆时针旋转 → 正 yaw rate）

### C 板实物上的轴方向

> 参考：[[00_raw/hardware/RoboMaster 开发板 C 型位号图.pdf|C 板位号图]]，[[00_raw/hardware/RoboMaster 开发板 C 型陀螺仪FPC小板位号图.pdf|FPC 小板位号图]]

BMI088 焊接在独立的 FPC 柔性小板上（见 [[01_extracted/hardware/c-board-resources#BMI088 六轴惯性测量单元]]），FPC 小板通过 14-pin 金手指插座竖直插入 C 板左上角。

**实物识别步骤**：

1. **找到 FPC 小板**：C 板左上角的白色插座，竖直插着一块狭长的 FPC 小板
2. **找到 BMI088 芯片**：FPC 小板下端（远离插座的一端），方形芯片 U1，丝印朝上
3. **找到 Pin 1**：芯片一角有黑色圆点标记，对应 FPC 小板上该角有丝印圆点
4. **确定轴方向**（芯片 Top View，Pin1 在左下角时）：
   - **X+**：沿芯片长边向右（在 C 板上指向**板左侧**）
   - **Y+**：沿芯片短边向上（在 C 板上指向**板顶部方向**）
   - **Z+**：垂直芯片表面向上（在 C 板上指向**板正面外侧**）

```
        C 板实物示意图（俯视）
    ┌─────────────────────────────┐
    │  ┌──────┐                   │
    │  │FPC   │ ← 插入方向        │
    │  │小板  │    ↓              │
    │  │ ┌──┐ │                   │
    │  │ │U1│ │  BMI088           │
    │  │ │● │ │  ●=Pin1           │
    │  │ └──┘ │  X→左  Y→上       │
    │  └──────┘  Z→外             │
    │◄── X+                        │
    │                              │
    │         C 板主体              │
    └─────────────────────────────┘
          ↑
         Y+
```

### C 板 INS 坐标系

> 参考：C 板实物照片标注。INS 导航系定义为**前 X、左 Y、上 Z**。

| 轴 | 物理量 | INS euler 索引 | 板子方向 |
|----|--------|---------------|---------|
| X | roll | `euler_angle[0]` / `euler_rad[0]` | 向前 |
| Y | pitch | `euler_angle[1]` / `euler_rad[1]` | 向左 |
| Z | yaw | `euler_angle[2]` / `euler_rad[2]` | 向上（竖直） |

### 芯片轴 → C 板坐标系映射

BMI088 芯片焊接在 FPC 柔性小板上，FPC 小板竖直插入 C 板左上角。芯片安装方向导致芯片坐标轴和 C 板坐标轴不是一一对齐的：

```
BMI088 芯片（Top View，Pin1 在左下角）：
       Y+ (短边)
        ↑
   Pin1 ●──→ X+ (长边)

FPC 小板插入 C 板左上角后的映射关系：
    chip X (长边) ──→ C 板左方 ──→ board Y 轴 ──→ pitch
    chip Y (短边) ──→ C 板前方 ──→ board X 轴 ──→ roll
    chip Z (垂直芯片) ──→ C 板上方 ──→ board Z 轴 ──→ yaw
```

| 芯片轴 | 寄存器索引 | 板子物理方向 | INS 物理量 |
|--------|-----------|-------------|-----------|
| X | `gyro[0]` / `acc[0]` | 向左 | **pitch** |
| Y | `gyro[1]` / `acc[1]` | 向前 | **roll** |
| Z | `gyro[2]` / `acc[2]` | 向上 | **yaw** |

> 这是理解云台反馈源选择的关键：**`gyro[0]` 是 pitch 角速度，`gyro[1]` 是 roll 角速度**。`gyro` 数组的索引和 `euler` 数组的索引含义不同（`gyro[0]=pitch` 但 `euler[0]=roll`），选反馈源时必须按物理含义选，不能用相同下标。详见 [[02_code_twin/apps/infantry3/single_board/gimbal_func/gimbal_func#BMI088-轴映射]] 和 [[02_code_twin/modules/INS/module_ins#坐标系与轴映射]]。

以哨兵云台为例（代码验证）：

| 芯片轴 | 寄存器索引 | 代码用途 | 对应机器人轴 |
|--------|-----------|---------|-------------|
| X | `gyro[0]` / `acc[0]` | Pitch 角速度反馈（`gimbal_func.c`） | Pitch 轴 |
| Y | `gyro[1]` / `acc[1]` | 未在云台控制中直接使用 | （侧向） |
| Z | `gyro[2]` / `acc[2]` | Yaw 角速度反馈（`gimbal_func.c`） | Yaw 轴 |

> `IMU_Param_Correction()` 中的 `IMU_Param.Yaw/Pitch/Roll` 用于修正安装偏差，当前代码全部硬编码为 0（即默认芯片坐标系 = C 板坐标系）。如果实际安装有旋转或翻转，需要在这里配置安装角。
