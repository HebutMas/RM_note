# module_bmi088.c/h — BMI088 驱动封装

`modules/BMI088/module_bmi088.{c,h}` / `module_bmi088_cali.c`

## 一句话

封装 BMI088 两个 SPI 设备（加速度计 + 陀螺仪）的初始化、数据读取、温控和标定，对外提供 `Bmi088_device_t` 指针。芯片本身的数据手册参数（寄存器地址、灵敏度、I2C→SPI 切换原理、dummy byte 机制）见 [[01_extracted/hardware/bmi088-datasheet]]，本文不重复。

## 对外接口

| 函数 | 作用 | 返回 |
|------|------|------|
| `Module_BMI088_init()` | 初始化两个 SPI 设备 + 传感器 + 温控 PID + 加载标定 | void |
| `Module_BMI088_get_device()` | 获取设备指针（含 acc/gyro/temperature） | `Bmi088_device_t *`，未初始化返回 NULL |
| `Module_BMI088_get_accel()` | 读加速度 → `dev->acc[3]`（m/s²） | 0=成功 |
| `Module_BMI088_get_gyro()` | 读角速度 → `dev->gyro[3]`（dps） | 0=成功 |
| `Module_BMI088_get_temp()` | 读温度 → `dev->temperature`（℃） | 0=成功 |
| `Module_BMI088_temp_ctrl()` | 温控 PID 执行一次（需 `BMI088_TEMP_ENABLE=1`） | void |
| `Module_BMI088_calibrate_bmi088_offset()` | 陀螺仪零偏标定 + Flash 保存 | 0=成功 |

> 数据读取的寄存器地址、大端拼接、灵敏度换算详见 [[01_extracted/hardware/bmi088-datasheet#数据获取]]。本文只讲代码组织。

## Bmi088_device_t 结构体

```c
typedef struct {
    uint8_t              initialized;
    SPI_Device          *gyro_device;       /* 陀螺仪 SPI 设备（CSB2） */
    SPI_Device          *acc_device;        /* 加速度计 SPI 设备（CSB1） */
    PWM_Device          *bmi088_pwm;        /* 温控 PWM（TIM10_CH1） */
    PIDInstance          pid_temp;          /* 温控 PID 实例 */
    float                acc[3];            /* 加速度（m/s²） */
    float                gyro[3];           /* 角速度（dps） */
    float                temperature;       /* 温度（℃） */
    BMI088_Cali_Offset_t BMI088_Cali_Offset;/* 标定参数 */
    BMI088_ERORR_CODE_e  BMI088_ERORR_CODE; /* 错误码 */
} Bmi088_device_t;
```

两个 `SPI_Device *` 指针——加速度计和陀螺仪各有独立的 SPI 设备句柄，对应各自的 CS 引脚。这是 BMI088 两个独立芯片在代码上的直接体现。

## 轴映射

> 芯片坐标系与 C 板坐标系（前 X、左 Y、上 Z）的对应关系详见 [[01_extracted/hardware/bmi088-datasheet#芯片轴 → C 板坐标系映射]]。

BMI088 寄存器按 X/Y/Z 顺序输出，`get_gyro` / `get_accel` 直接存入 `gyro[3]` / `acc[3]`。芯片在 C 板上的安装方向导致芯片轴和板子轴不对齐：

BMI088 芯片寄存器按 X/Y/Z 顺序输出，`get_gyro` / `get_accel` 直接按这个顺序存入 `gyro[3]` / `acc[3]`。但芯片在 C 板上的物理安装方向导致**芯片坐标轴和板子坐标轴不是一一对应的**：

```
    BMI088 芯片轴          C 板坐标轴
    ─────────────          ──────────
    chip X  ──→  pitch  ←── board Y
    chip Y  ──→  roll   ←── board X
    chip Z  ──→  yaw    ←── board Z（一致）
```

| 索引 | 芯片轴 | 实际物理含义 | 对应的 INS 欧拉角 |
|------|--------|------------|-----------------|
| `gyro[0]` / `acc[0]` | chip X | **pitch** | `euler_angle[1]` |
| `gyro[1]` / `acc[1]` | chip Y | **roll** | `euler_angle[0]` |
| `gyro[2]` / `acc[2]` | chip Z | **yaw** | `euler_angle[2]` |

> **gyro 数组和 euler 数组的索引含义不同**：`gyro[0]` 是 pitch 但 `euler_angle[0]` 是 roll。在云台控制中选择反馈源时，必须按物理含义选，不能用相同下标。详见 [[02_code_twin/apps/infantry3/single_board/gimbal_func/gimbal_func#BMI088-轴映射]]。

> EKF 内部不关心这个映射——它把三轴数据当作一个整体做四元数旋转，输入输出的索引含义是一致的。只有**从 BMI088 单独取某一轴**做速度反馈时，才需要知道这个对应关系。

## SPI 读写封装

> dummy byte 的原理（加速度计读多 1 字节、陀螺仪不需要）见 [[01_extracted/hardware/bmi088-datasheet#SPI 读写协议]]。

```c
// 写：地址最高位清零
static inline uint8_t _bmi088_write_reg(SPI_Device *device, uint8_t reg, ...) {
    uint8_t addr = reg & 0x7F;  // BMI088_SPI_WRITE_CODE
    return BSP_SPI_TransAndTrans(device, &addr, 1, data, len, timeout);
}

// 读：地址最高位置 1，加速度计多 1 字节 dummy
static inline uint8_t _bmi088_read_reg(SPI_Device *device, uint8_t reg, ...) {
    uint8_t dummy_bytes = is_accel ? 2 : 1;  // ← 关键：accel=2, gyro=1
    tx[0] = 0x80 | reg;                       // BMI088_SPI_READ_CODE
    BSP_SPI_TransReceive(device, tx, rx, len + dummy_bytes, timeout);
    memcpy(data, rx + dummy_bytes, len);      // 跳过 dummy 字节
    return status;
}
```

`is_accel` 参数贯穿整个驱动——初始化表回读、数据读取、Chip ID 验证都靠它区分 dummy byte 数量。这是 BMI088 两个芯片 SPI 时序差异在代码中的唯一处理点。

## 初始化流程

`Module_BMI088_init()` 按以下顺序执行：

### 第一步：BSP 设备初始化

```c
// F407: SPI1 + 两个 CS(PA4=acc, PB0=gyro) + TIM10_CH1(温控PWM)
// H723: SPI2 + 两个 CS(PC0=acc, PC3=gyro) + TIM3_CH4(温控PWM)
bmi088_device.acc_device  = BSP_SPI_Device_Init(&acc_cfg);
bmi088_device.gyro_device = BSP_SPI_Device_Init(&gyro_cfg);
bmi088_device.bmi088_pwm  = BSP_PWM_Device_Init(&pwm_cfg);
```

两个芯片共用同一条 SPI 总线（同一组 SCK/MOSI/MISO），只是 CS 引脚不同。

### 第二步：加速度计初始化 `bmi088_acc_init()`

1. **dummy read**（触发 I2C→SPI 切换）：读 `ACC_CHIP_ID`，值无效，丢弃。原理见 [[01_extracted/hardware/bmi088-datasheet#接口选择 — 加速度计的 I2C→SPI 切换坑]]
2. **软复位**：写 `0xB6` 到 `ACC_SOFTRESET(0x7E)`，等 300ms
3. **验证 Chip ID**：再读 `ACC_CHIP_ID`，应为 `0x1E`（源码中此处注释掉了校验，但保留了读取）
4. **写初始化表**：6 条寄存器配置，每条写后回读校验。配置内容见 [[01_extracted/hardware/bmi088-datasheet#初始化序列]]

> 第 1 步和第 3 步之间隔了一次软复位 + 300ms 等待。软复位后加速度计会回到 I2C 模式，但 dummy read 产生的 CSB1 上升沿已经把它切到了 SPI，软复位不会撤销这个切换（SPI 模式锁定到下次断电）。

### 第三步：陀螺仪初始化 `bmi088_gyro_init()`

1. **软复位**：写 `0xB6` 到 `GYRO_SOFTRESET(0x14)`，等 30ms（比加速度计的 1ms 长）
2. **验证 Chip ID**：读 `GYRO_CHIP_ID`，应为 `0x0F`，不匹配则置 `BMI088_NO_SENSOR` 错误码
3. **写初始化表**：6 条寄存器配置 + 回读校验。配置内容见 [[01_extracted/hardware/bmi088-datasheet#初始化序列]]

> 陀螺仪不需要 dummy read——它由 PS 引脚决定接口，C 板 PS 接 GND，上电就是 SPI。

### 第四步：温度 PID + PWM 启动

```c
PID_Init_Config_s pid_cfg = {
    .MaxOut = 1000, .Kp = 20, .Ki = 0, .Kd = 0, .Improve = 0x01,
};
PIDInit(&bmi088_device.pid_temp, &pid_cfg);
BSP_PWM_Start(bmi088_device.bmi088_pwm, NULL, 0);
```

### 第五步：加载标定数据

```c
BSP_FLASH_Read_Buffer(flash_buf, sizeof(flash_buf));
if (flash_buf[last_byte] != 0xAA) {
    // 无有效标定 → 执行标定
    Module_BMI088_calibrate_bmi088_offset(&bmi088_device);
} else {
    // 有标定 → 从 Flash 加载
    memcpy(&bmi088_device.BMI088_Cali_Offset, flash_buf, sizeof(BMI088_Cali_Offset_t));
}
```

Flash 最后一字节 `0xAA` 是标定有效标志。首次上电没有标定数据，自动执行标定流程。

## 温控 PID

> 硬件：TIM10_CH1（F407）控制 BMI088 加热电路。详见 [[01_extracted/hardware/c-board-resources#陀螺仪温控 PWM]]。

```c
void Module_BMI088_temp_ctrl(void) {
#if BMI088_TEMP_ENABLE
    PIDCalculate(&bmi088_device.pid_temp, bmi088_device.temperature, BMI088_TEMP_SET);
    float duty = bmi088_device.pid_temp.Output;
    CLAMP(duty, 0.0f, 1000.0f);
    BSP_PWM_SetDutyCycle(bmi088_device.bmi088_pwm, (uint16_t)duty);
#endif
}
```

- 目标温度 `BMI088_TEMP_SET` = 35℃（`module_bmi088.h` 中定义）
- PID 参数：Kp=20, Ki=0, Kd=0（纯比例控制）
- 输出限幅 0~1000，对应 PWM 占空比
- `BMI088_TEMP_ENABLE` 默认为 0，需要在 `module_config.h` 中开启
- 由 INS 线程每个周期调用一次（`ins_task_entry` 中 `Module_BMI088_get_temp()` → `Module_BMI088_temp_ctrl()`）

## 标定流程

`module_bmi088_cali.c` — `Module_BMI088_calibrate_bmi088_offset()`

### 目的

测量陀螺仪静止时的零偏（GyroOffset）和重力模长（gNorm），保存到 Flash，后续 `get_gyro` / `get_accel` 读取时自动减去零偏 / 乘缩放因子。

### 流程

```
黄灯亮（标定中）
  ↓
采集 6000 组有效数据（超时 10s 保护）
  ├── 每组数据合理性校验：
  │   gNorm 在 5~15 m/s² 之间
  │   三轴陀螺仪绝对值 < 5 dps
  │   无 NaN/Inf
  ├── 波动检测：
  │   gNorm 极差 < 0.5 m/s²
  │   三轴陀螺仪极差 < 0.15 dps
  │   超过 → 判定设备移动 → 丢弃全部 → 重新采集
  └── 喂狗（IWDG）防复位
  ↓
计算平均值 → gNorm, GyroOffset[3]
  ↓
结果校验：
  |gNorm - 9.81| < 0.5
  |GyroOffset[i]| < 0.01
  波动达标
  不达标 → do-while 重采
  ↓
计算 AccelScale = 9.81 / gNorm
  ↓
写入 Flash（末尾 0xAA 标志）
  ↓
超时或异常 → 使用默认值（硬编码零偏 + gNorm=9.675）
```

### 标定参数如何影响数据读取

```c
// get_accel — 如果已标定，乘 AccelScale 修正
float val = BMI088_ACCEL_6G_SEN * (float)raw;
bmi088_device.acc[i] = Calibrated ? val * AccelScale : val;

// get_gyro — 如果已标定，减去 GyroOffset
float val = BMI088_GYRO_2000_SEN * (float)raw;
bmi088_device.gyro[i] = Calibrated ? val - GyroOffset[i] : val;
```

> `AccelScale = 9.81 / gNorm`：标定时测到的重力模长不恰好是 9.81（有安装误差、传感器误差），用缩放因子把实测值校正到标准重力。

## 被谁调用

| 调用者 | 调用什么 | 链接 |
|--------|---------|------|
| `Module_Init()` | `Module_BMI088_init()` 一次性初始化 | [[03_moc/Robot-Init-Walkthrough#Module_Init()]] |
| INS 线程 | 每周期 `get_accel()` + `get_gyro()` + `get_temp()` + `temp_ctrl()` | [[02_code_twin/modules/INS/module_ins]]（待写） |
