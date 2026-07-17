# BMI088 IMU 数据手册提炼

> 出处：Bosch BMI088 Datasheet（bst-bmi088-ds001），结合 [[01_extracted/hardware/c-board-resources#BMI088 六轴惯性测量单元|C 板硬件资源]] 和项目源码 `modules/BMI088/`。

## 一句话

BMI088 是 Bosch 的 6 轴 IMU（3 轴加速度计 + 3 轴陀螺仪），**加速度计和陀螺仪是两个独立芯片**，各自有独立的 SPI 从机（独立的 CS 引脚），软件上要当两个设备操作。

> 注意：BMI088 同时支持 SPI 和 I2C 两种接口，但**加速度计和陀螺仪的 I2C 地址不同**（加速度计 0x18，陀螺仪 0x68）。C 板走的是 SPI1，不涉及 I2C。选哪个接口由上电时 CSB/SDO 引脚电平决定：CSB 拉低 → SPI 模式，CSB 拉高 → I2C 模式。C 板硬件直接把 CS 接了 GPIO，走 SPI。

---

## 两个独立设备

| | 加速度计 (Accel) | 陀螺仪 (Gyro) |
|---|---|---|
| **Chip ID** | 0x1E | 0x0F |
| **I2C 地址** | 0x18 | 0x68 |
| **C 板 CS 引脚** | PA4 | PB0 |
| **SPI 读时序** | 多 1 字节 dummy（共 2 字节前导） | 1 字节前导 |
| **量程** | ±3 / ±6 / ±12 / ±24 g | ±125 / ±250 / ±500 / ±1000 / ±2000 dps |
| **项目配置** | ±6g，800Hz ODR | ±2000 dps，1000Hz ODR / 116Hz BW |
| **数据寄存器** | 0x12~0x17 (X/Y/Z 各 2 字节) | 0x02~0x07 (X/Y/Z 各 2 字节) |
| **温度寄存器** | 0x22~0x23（在 accel 设备上读） | — |

> SPI 读时序差异是 BMI088 最容易踩的坑：加速度计读操作需要在寄存器地址后**多发 1 个 dummy 字节**，陀螺仪不需要。源码中 `_bmi088_read_reg()` 用 `is_accel` 参数区分，`dummy_bytes = is_accel ? 2 : 1`。

---

## SPI 读写协议

BMI088 SPI 最大 10MHz，C 板用 SPI1（PB3/PA7/PB4）。

```c
// 写：地址最高位清零
uint8_t addr = reg & 0x7F;   // BMI088_SPI_WRITE_CODE

// 读：地址最高位置 1
uint8_t addr = 0x80 | reg;   // BMI088_SPI_READ_CODE
```

时序：

```
写操作:  [CS↓] [reg & 0x7F] [data...] [CS↑]
读操作:  [CS↓] [0x80 | reg] [dummy*] [data...] [CS↑]
                              ↑ accel 多 1 字节 dummy，gyro 不需要
```

---

## 数据获取

### 加速度计

**寄存器**：`ACC_XOUT_L(0x12)` 起连续 6 字节 → X_L, X_H, Y_L, Y_H, Z_L, Z_H

```c
// 源码：module_bmi088.c → Module_BMI088_get_accel()
uint8_t buf[6];
_bmi088_read_reg(acc_device, BMI088_ACCEL_XOUT_L, buf, 6, 1, TX_WAIT_FOREVER);

for (uint8_t i = 0; i < 3; i++) {
    int16_t raw = (int16_t)((buf[2*i+1] << 8) | buf[2*i]);  // 大端拼接
    float   val = BMI088_ACCEL_6G_SEN * (float)raw;          // 乘灵敏度 → m/s²
    bmi088_device.acc[i] = val;
}
```

**灵敏度（LSB → m/s²）**：

| 量程 | 灵敏度常数 | 含义 |
|------|----------|------|
| ±3g | `BMI088_ACCEL_3G_SEN` = 0.000897 | 1 LSB ≈ 0.897 mg |
| ±6g | `BMI088_ACCEL_6G_SEN` = 0.001794 | 1 LSB ≈ 1.794 mg |
| ±12g | `BMI088_ACCEL_12G_SEN` = 0.003589 | 1 LSB ≈ 3.589 mg |
| ±24g | `BMI088_ACCEL_24G_SEN` = 0.007178 | 1 LSB ≈ 7.178 mg |

> 灵敏度常数已经是 m/s²/LSB（把 g 换算进去了），直接 `raw × 灵敏度` 就是 m/s²，不用再乘 9.81。项目配置 ±6g。

### 陀螺仪

**寄存器**：`GYRO_X_L(0x02)` 起连续 6 字节 → X_L, X_H, Y_L, Y_H, Z_L, Z_H

```c
// 源码：module_bmi088.c → Module_BMI088_get_gyro()
uint8_t buf[6];
_bmi088_read_reg(gyro_device, BMI088_GYRO_X_L, buf, 6, 0, TX_WAIT_FOREVER);

for (uint8_t i = 0; i < 3; i++) {
    int16_t raw = (int16_t)((buf[2*i+1] << 8) | buf[2*i]);
    float   val = BMI088_GYRO_2000_SEN * (float)raw;   // 乘灵敏度 → dps
    bmi088_device.gyro[i] = val;
}
```

**灵敏度（LSB → dps）**：

| 量程 | 灵敏度常数 | 含义 |
|------|----------|------|
| ±2000 dps | `BMI088_GYRO_2000_SEN` = 0.001065 | 1 LSB ≈ 0.001065 dps |
| ±1000 dps | `BMI088_GYRO_1000_SEN` = 0.000533 | — |
| ±500 dps | `BMI088_GYRO_500_SEN` = 0.000266 | — |
| ±250 dps | `BMI088_GYRO_250_SEN` = 0.000133 | — |
| ±125 dps | `BMI088_GYRO_125_SEN` = 0.000067 | — |

> 项目配置 ±2000 dps。陀螺仪输出单位是 **dps**（度/秒），不是 rad/s，后续 INS 模块使用时需自行转换。

### 温度

**寄存器**：`TEMP_M(0x22)` + `TEMP_L(0x23)`，在**加速度计设备**上读

```c
// 源码：module_bmi088.c → Module_BMI088_get_temp()
uint8_t buf[2];
_bmi088_read_reg(acc_device, BMI088_TEMP_M, buf, 2, 1, TX_WAIT_FOREVER);

int16_t raw = (((int16_t)buf[0] << 3) | (buf[1] >> 5));  // 11-bit，高3位+低5位
if (raw > 1023) raw -= 2048;                              // 补码转有符号

bmi088_device.temperature = (float)raw * 0.125f + 23.0f;  // 0.125℃/LSB，偏移23℃
```

> 温度用于温控 PID（TIM10_CH1 加热），保持 BMI088 在恒温（默认 35℃）工作，减少温漂。

---

## 初始化序列

加速度计和陀螺仪各自有一个初始化表，按顺序写寄存器并回读校验：

### 加速度计初始化表

| 序号 | 寄存器 | 值 | 含义 |
|------|--------|---|------|
| 1 | `ACC_PWR_CTRL (0x7D)` | 0x04 | 加速度计上电 |
| 2 | `ACC_PWR_CONF (0x7C)` | 0x00 | 正常模式（非 suspend） |
| 3 | `ACC_CONF (0x40)` | 0x80 \| 0x0B | normal 模式 + 800Hz ODR + must-set bit |
| 4 | `ACC_RANGE (0x41)` | 0x01 | ±6g |
| 5 | `INT1_IO_CTRL (0x53)` | 0x0A | INT1 推挽 + 低电平有效 |
| 6 | `INT_MAP_DATA (0x58)` | 0x04 | DRDY 中断映射到 INT1 |

### 陀螺仪初始化表

| 序号 | 寄存器 | 值 | 含义 |
|------|--------|---|------|
| 1 | `GYRO_RANGE (0x0F)` | 0x00 | ±2000 dps |
| 2 | `GYRO_BANDWIDTH (0x10)` | 0x82 | 1000Hz ODR / 116Hz BW + must-set bit |
| 3 | `GYRO_LPM1 (0x11)` | 0x00 | 正常模式 |
| 4 | `GYRO_CTRL (0x15)` | 0x80 | DRDY 使能 |
| 5 | `INT3_INT4_IO_CONF (0x16)` | 0x00 | INT3 推挽 + 低电平有效 |
| 6 | `INT3_INT4_IO_MAP (0x18)` | 0x01 | DRDY 映射到 INT3 |

> 每条写完后回读校验，不匹配则记录错误码。源码用 `_bmi088_write_init_table()` 统一处理。

---

## C 板硬件连接

> 详见 [[01_extracted/hardware/c-board-resources#BMI088 六轴惯性测量单元|C 板资源清单]]

BMI088 通过 SPI1 与 STM32 通信，加速度计和陀螺仪各有独立 CS：

| 功能 | 引脚 |
|------|------|
| SPI1_CLK | PB3 |
| SPI1_MOSI | PA7 |
| SPI1_MISO | PB4 |
| CS1_Accel | PA4 |
| CS1_Gyro | PB0 |
| INT1_Accel | PC4 |
| INT1_Gyro | PC5 |
| 温控加热 | PF6 (TIM10_CH1) |

SPI 最大 10MHz。温控用 PWM 加热电路，建议加热温度比板载正常工作温度高 15~20℃。

---

## 关键参数速查

| 参数 | 加速度计 | 陀螺仪 |
|------|---------|--------|
| 分辨率 | 16 bit | 16 bit |
| 量程 | ±3/±6/±12/±24 g | ±125/±250/±500/±1000/±2000 dps |
| ODR | 12.5~1600 Hz | 100~2000 Hz |
| 滤波带宽 | normal/OSR2/OSR4 | ODR/Filter 组合 |
| 通信接口 | SPI (max 10MHz) / I2C (0x18) | SPI (max 10MHz) / I2C (0x68) |
| Chip ID | 0x1E | 0x0F |
| 工作温度 | -40~+85℃ | -40~+85℃ |
| 供电 | 2.4~3.6V | 2.4~3.6V |

---

## 软复位

两个芯片各自独立软复位，写 0xB6 到对应寄存器：

```c
// 加速度计软复位：写 0xB6 到 ACC_SOFTRESET (0x7E)，等 1ms
// 陀螺仪软复位：写 0xB6 到 GYRO_SOFTRESET (0x14)，等 30ms
```

> 软复位后所有寄存器恢复默认值，需重新走初始化表。
