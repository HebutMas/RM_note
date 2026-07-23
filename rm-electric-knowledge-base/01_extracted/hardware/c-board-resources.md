# C 板硬件资源清单

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf|C板手册]]
> 主控芯片：STM32F407
> 输入电压：8V~28V，待机电流 0.01A @DC 24V，重量 38g，尺寸 60×41×16.3mm，工作温度 0~55℃

---

## 对外通信接口

### CAN 总线接口

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=15|P.13]]，附表 [[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=19|P.17]]

开发板集成 2 路 CAN，最大支持 1Mbps。

| 接口 | 端口类型 | 线序 | 对应 IO |
|------|---------|------|---------|
| CAN1 | 2-pin | Pin1=CANL, Pin2=CANH                                         \|(左黑又红) | CAN1_TX=PD1, CAN1_RX=PD0 |
| CAN2 | 4-pin | Pin1=5V, Pin2=GND, Pin3=CANH, Pin4=CANL    \|(右边的左红右黑) | CAN2_TX=PB6, CAN2_RX=PB5 |

### UART 接口

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=13|P.11]]，附表 [[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=19|P.17]]

开发板集成 2 路 UART，映射到 STM32 的 UART1 与 UART6。

| 外壳丝印 | STM32 串口 | 端口类型 | 线序 | 对应 IO |
|---------|-----------|---------|------|---------|
| UART1（4-pin） | **USART1** | 4-pin 卧式 | Pin1=RXD, Pin2=TXD, Pin3=GND, Pin4=5V | UART1_TX=PA9, UART1_RX=PB7 |
| UART2（3-pin） | **USART6** | 3-pin 卧式 | Pin1=GND, Pin2=TXD, Pin3=RXD | UART6_TX=PG14, UART6_RX=PG9 |

> **注意**：外壳丝印与 STM32 实际串口不对应。外壳丝印 UART1 对应 STM32 的 USART6，外壳丝印 UART2 对应 STM32 的 USART1。
>
> UART6 接口线序与裁判系统电源模块一致，与电源模块通信时需将 TX 与 RX 线序交叉。
>
> UART 接口只支持 3.3V 和 5V 电平，与 RS485/RS232 通信需外置电平转换芯片。

### DBUS 接口

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=14|P.12]]，附表 [[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=19|P.17]]

1 路 DBUS 遥控器接收接口，与 PWM 接口共用连接器。DBUS 信号经反相电路后连接到 STM32 的 UART3_RX（PC11），波特率一般设置为 100kbps。

---

## 调试与下载接口

### SWD 调试接口

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=10|P.8]]

4-pin 接口，用于程序下载和调试（J-link / ST-link）。

| Pin | 信号 |
|-----|------|
| 1 | SWDIO |
| 2 | SWCLK |
| 3 | GND |
| 4 | 3.3V |

SWD 下载线：线长 100mm，A=SWDIO，B=SWCLK，C=GND，D=3.3V。

### micro USB 接口（虚拟串口 / DFU）

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=10|P.8]]

USB 全速接口（FS, 12Mbps），可用于 USB 通信或 DFU 模式下载固件。USB 供电仅供给 VCC_5V，不支持 PWM 外设接口供电。

| 功能 | 对应 IO |
|------|---------|
| USB_DM | PA11 |
| USB_DP | PA12 |
| USB_OTG | PA10 |

> 通过 USB 可实现虚拟串口功能。DFU 模式需配置 BOOT0=1, BOOT1=0 后复位进入。

---

## IO 资源

### 可配置 I/O 接口

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=12|P.10]]，附表 [[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=19|P.17]]

8-pin 牛角座（2.54mm 间距），支持硬件 IIC（I2C2）与 SPI（SPI2），支持 3.3V ~~或 5V 通信设备（5V 需焊接 R210 并去除 R209）~~。

| Pin | 信号 | 对应 IO |
|-----|------|---------|
| 1 | SPI2_CS | PB12 |
| 2 | GND | — |
| 3 | SPI2_CLK | PB13 |
| 4 | 3.3V | — |
| 5 | SPI2_MOSI | PB15 |
| 6 | I2C2_SCL | PF1 |
| 7 | SPI2_MISO | PB14 |
| 8 | I2C2_SDA | PF0 |

### PWM 接口

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=14|P.12]]，附表 [[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=19|P.18]]

7 路 PWM 输出，用于 5V 舵机或 PWM 驱动模块，总最大电流 5A。与 DBUS 共用 24-pin 连接器。

| PWM 通道 | 对应 IO |
|---------|---------|
| TIM1_CH1 | PE9 |
| TIM1_CH2 | PE11 |
| TIM1_CH3 | PE13 |
| TIM1_CH4 | PE14 |
| TIM8_CH1 | PC6 |
| TIM8_CH2 | PI6 |
| TIM8_CH3 | PI7 |

### 用户自定义按键

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=11|P.9]]，附表 [[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=19|P.17]]

按键按下时 PA0 为低电平。对应 IO：PA0。

### 5V 接口（激光器）

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=9|P.7]]，附表 [[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=19|P.17]]

可控 5V 电源接口，可外接 RoboMaster 红点激光器。开关控制 IO 为 PC8（TIM3_CH3），可通过 PWM 调节亮度。

---

## 板载硬件资源

### BMI088 六轴惯性测量单元

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=16|P.14]]，附表 [[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=19|P.18]]

抗震 BMI088，SPI 通信（SPI1），最大 10MHz。

| 功能 | 对应 IO |
|------|---------|
| SPI1_CLK | PB3 |
| SPI1_MOSI | PA7 |
| SPI1_MISO | PB4 |
| CS1_Accel | PA4 |
| CS1_Gyro | PB0 |
| INT1_Accel | PC4 |
| INT1_Gyro | PC5 |

### 陀螺仪温控 PWM

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=16|P.14]]

TIM10_CH1（PF6）控制陀螺仪加热电路，实现恒温处理。Heat_Power 为 5V，TIM10_CH1 高电平时加热功率 0.58W，建议加热温度比电路板正常工作温度高 15~20℃。

### IST8310 磁力计

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=18|P.16]]，附表 [[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=19|P.18]]

三轴磁力计 IST8310，IIC 通信（I2C3），最大 400kHz，默认 IIC 地址 0x0E。

| 功能 | 对应 IO |
|------|---------|
| I2C3_SCL | PA8 |
| I2C3_SDA | PC9 |
| RSTN_IST8310 | PG6 |
| DRDY_IST8310 | PG3 |

### RGB LED

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=7|P.6]]，附表 [[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=19|P.17]]

共阳极 RGB LED，IO 输出高电平点亮，可通过 PWM 调节亮度。

| 颜色 | 对应 IO |
|------|---------|
| LED_R（红） | PH12 |
| LED_G（绿） | PH11 |
| LED_B（蓝） | PH10 |

### 蜂鸣器

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=16|P.14]]，附表 [[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=19|P.18]]

贴片式无源蜂鸣器，PWM 驱动，额定频率 4000Hz，可通过调节 PWM 频率改变音调。对应定时器通道：TIM4_CH3（PD14）。

### 电压检测

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=16|P.14]]，附表 [[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=19|P.18]]

检测输入电压 VCC_BAT，分压后连接到 STM32 ADC（PF10）。

### 数字摄像头 FPC 接口

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=16|P.14]]，附表 [[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=19|P.18]]

18-pin FPC 接口，支持 DCMI，可连接 8 位 CMOS 摄像头模块。

---

## IO 对照总表

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=19|P.17]]

| 功能 | 网络名 | 对应 IO |
|------|--------|---------|
| LED 红 | LED_R | PH12 |
| LED 绿 | LED_G | PH11 |
| LED 蓝 | LED_B | PH10 |
| 5V 接口 | TIM3_CH3 | PC8 |
| USB | USB_DM | PA11 |
| USB | USB_DP | PA12 |
| USB | USB_OTG | PA10 |
| 按键 | KEY | PA0 |
| 可配置 IO | I2C2_SCL | PF1 |
| 可配置 IO | I2C2_SDA | PF0 |
| 可配置 IO | SPI2_CS | PB12 |
| 可配置 IO | SPI2_CLK | PB13 |
| 可配置 IO | SPI2_MISO | PB14 |
| 可配置 IO | SPI2_MOSI | PB15 |
| UART（3-pin） | UART6_TX | PG14 |
| UART（3-pin） | UART6_RX | PG9 |
| UART（4-pin） | UART1_TX | PA9 |
| UART（4-pin） | UART1_RX | PB7 |
| CAN1 | CAN1_TX | PD1 |
| CAN1 | CAN1_RX | PD0 |
| CAN2 | CAN2_TX | PB6 |
| CAN2 | CAN2_RX | PB5 |
| PWM | TIM1_CH1 | PE9 |
| PWM | TIM1_CH2 | PE11 |
| PWM | TIM1_CH3 | PE13 |
| PWM | TIM1_CH4 | PE14 |
| PWM | TIM8_CH1 | PC6 |
| PWM | TIM8_CH2 | PI6 |
| PWM | TIM8_CH3 | PI7 |
| DBUS | UART3_RX | PC11 |
| 蜂鸣器 | TIM4_CH3 | PD14 |
| 电压检测 | ADC_BAT | PF10 |
| IMU 温控 | TIM10_CH1 | PF6 |
| BMI088 | INT1_Accel | PC4 |
| BMI088 | INT1_Gyro | PC5 |
| BMI088 | CS1_Accel | PA4 |
| BMI088 | CS1_Gyro | PB0 |
| BMI088 | SPI1_CLK | PB3 |
| BMI088 | SPI1_MOSI | PA7 |
| BMI088 | SPI1_MISO | PB4 |
| 磁力计 | RSTN_IST8310 | PG6 |
| 磁力计 | DRDY_IST8310 | PG3 |
| 磁力计 | I2C3_SCL | PA8 |
| 磁力计 | I2C3_SDA | PC9 |
| 摄像头 | I2C1_SCL | PB8 |
| 摄像头 | I2C1_SDA | PB9 |
| 摄像头 | PCLK_OUT | PA6 |
| 摄像头 | DCMI_HREF | PH8 |
| 摄像头 | DCMI_VSYNC | PI5 |
| 摄像头 | DCMI_D0 | PH9 |
| 摄像头 | DCMI_D1 | PC7 |
| 摄像头 | DCMI_D2 | PE0 |
| 摄像头 | DCMI_D3 | PE1 |
| 摄像头 | DCMI_D4 | PE4 |
| 摄像头 | DCMI_D5 | PI4 |
| 摄像头 | DCMI_D6 | PE5 |
| 摄像头 | DCMI_D7 | PE6 |

---

## 电源框图

> 出处：[[00_raw/hardware/RoboMaster  开发板 C 型用户手册.pdf#page=7|P.5]]

| 电源网络 | 来源 | 用途 | 最大电流 |
|---------|------|------|---------|
| VCC_5V_M | 24V → 5V (TPS54540) | 7 路 PWM 舵机接口 | 5A |
| VCC_5V | 24V → 5V (SY8510) | 板载器件供电、下一级电源输入 | 1A |
| VCC_3V3 | 5V → 3.3V (SY8089) | 板载器件供电 | 1A |

输入防护：XT30 接口，防反接 + 缓启动，输入超过 28V 时后级关断（过压保护）。
