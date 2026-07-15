# SBUS协议详解

## 一、简介

SBUS（Serial Bus，串行总线）是由日本 **Futaba（双叶）** 公司开发的串行通信协议，主要用于遥控模型（航模、无人机、车模）和飞行控制系统。通过单根信号线传输多达 16 个通道的数据，大幅简化了传统 PWM 一通道一根线的布线复杂度。

SBUS 有两个版本：
- **SBUS 1**：原始版本，单向通信，接收机不能回传反馈
- **SBUS 2**：增加了双向通信能力，允许接收机从兼容设备（陀螺仪、传感器等）获取状态反馈

> **注意**：SBUS 和 DBUS（大疆 RoboMaster 用的协议）名字相似但不同。DBUS 是大疆在 SBUS 基础上的修改版，波特率同为 100kbps 但帧结构和数据内容不同。

## 二、串口配置

| 参数 | 值 |
|------|-----|
| 波特率 | 100000 bps（100K） |
| 数据位 | 8 位（STM32 中需选 9 位，因为偶校验占用 1 位） |
| 奇偶校验 | 偶校验（EVEN） |
| 停止位 | 2 位 |
| 流控 | 无 |
| 帧长度 | 25 字节 |

即 **8E2** 串口格式。在 STM32 HAL 库中配置为：`WordLength = UART_WORDLENGTH_9B`, `Parity = UART_PARITY_EVEN`, `StopBits = UART_STOPBITS_2`。

## 三、协议帧格式

```
[startbyte] [data1][data2]...[data22] [flags] [endbyte]
    1字节         22字节            1字节    1字节   = 25字节
```

| 字段 | 字节偏移 | 长度 | 说明 |
|------|---------|------|------|
| startbyte | 0 | 1 | 帧起始字节，固定值 **0x0F** |
| data1~data22 | 1~22 | 22 | 16 个通道数据，每个通道 11 bit（22×8 = 16×11 = 176 bit） |
| flags | 23 | 1 | 标志字节（信号丢失、帧丢失等） |
| endbyte | 24 | 1 | 帧结束字节，固定值 **0x00** |

### flags 标志位说明

| bit | 说明 |
|-----|------|
| bit 0 | 数据丢失标志（failsafe）：1 = 信号丢失 |
| bit 1 | 帧丢失标志：1 = 帧丢失 |
| bit 2 | 信号丢失（signal loss） |
| bit 3 | SBUS 2 模式激活（仅 SBUS 2） |
| bit 4 | 无效帧 |

flags = 0 表示控制器与接收器保持连接；flags != 0 表示异常状态，飞控应执行失控保护。

### 通道数据范围

- 每个通道 11 bit，数值范围 0~2047
- 实际有效范围约 **282~1722**，中值约 **1002**
- 不同遥控器可能有细微差异

## 四、通道数据解析

16 个通道每个 11 bit，紧凑排列在 22 字节中。解析方式如下：

```
ch1  = (data2的低3位 << 8)  | data1
ch2  = (data3的低6位 << 5)  | (data2的高5位)
ch3  = (data5的低1位 << 10) | (data4 << 2) | (data3的高2位)
ch4  = (data6的低4位 << 7)  | (data5的高7位)
ch5  = (data7的低7位 << 4)  | (data6的高4位)
ch6  = (data8的低2位 << 9)  | (data7的高1位 << 1) ... (需3字节)
...
```

通用解析代码：

```c
// SBUS 通道解析
void sbus_decode(uint8_t *frame, uint16_t *channels) {
    // frame[0] = 0x0F (startbyte)
    // frame[24] = 0x00 (endbyte)
    channels[0]  = ((frame[1]      | frame[2]  << 8)                  ) & 0x07FF;
    channels[1]  = ((frame[2]  >> 3| frame[3]  << 5)                  ) & 0x07FF;
    channels[2]  = ((frame[3]  >> 6| frame[4]  << 2 | frame[5]  << 10)) & 0x07FF;
    channels[3]  = ((frame[5]  >> 1| frame[6]  << 7)                  ) & 0x07FF;
    channels[4]  = ((frame[6]  >> 4| frame[7]  << 4)                  ) & 0x07FF;
    channels[5]  = ((frame[7]  >> 7| frame[8]  << 1 | frame[9]  << 9 )) & 0x07FF;
    channels[6]  = ((frame[9]  >> 2| frame[10] << 6)                  ) & 0x07FF;
    channels[7]  = ((frame[10] >> 5| frame[11] << 3)                  ) & 0x07FF;
    channels[8]  = ((frame[11]      | frame[12] << 8)                  ) & 0x07FF;
    channels[9]  = ((frame[12] >> 3| frame[13] << 5)                  ) & 0x07FF;
    channels[10] = ((frame[13] >> 6| frame[14] << 2 | frame[15] << 10)) & 0x07FF;
    channels[11] = ((frame[15] >> 1| frame[16] << 7)                  ) & 0x07FF;
    channels[12] = ((frame[16] >> 4| frame[17] << 4)                  ) & 0x07FF;
    channels[13] = ((frame[17] >> 7| frame[18] << 1 | frame[19] << 9 )) & 0x07FF;
    channels[14] = ((frame[19] >> 2| frame[20] << 6)                  ) & 0x07FF;
    channels[15] = ((frame[20] >> 5| frame[21] << 3)                  ) & 0x07FF;
}
```

## 五、信号取反问题

SBUS 信号电平与标准 UART 相反（反相逻辑电平），连接到 MCU 时必须做硬件取反。软件取反无效。

### 硬件取反电路

使用一个 NPN 三极管（如 S8050）和两个电阻实现反相：

```
SBUS信号 → 1K电阻 → 基极
                    ├─ 3.3V
                    │
                   发射极
                    │
                   集电极 → 10K电阻 → 3.3V
                    │
                    └──→ MCU UART RX
                    │
                   GND
```

> 也常用 74HC04 或 SN74LVC1G04 等逻辑反相器芯片。

## 六、发送间隔

| 模式 | 间隔 | 说明 |
|------|------|------|
| 高速模式 | 每 7ms 一帧 | 两帧间隔需超过 3ms 才会被接收 |
| 普通模式 | 每 14ms 一帧 | 标准模式 |

## 七、SBUS vs iBUS vs DBUS 对比

| 特性 | SBUS | iBUS | DBUS |
|------|------|------|------|
| 提出方 | Futaba（双叶） | FlySky（富斯） | DJI（大疆） |
| 波特率 | 100000 bps | 115200 bps | 100000 bps |
| 串口格式 | 8E2 | 8N1 | 8E1 |
| 帧长度 | 25 字节 | 32 字节 | 18 字节 |
| 通道数 | 16 | 14 | 4 摇杆 + 2 开关 |
| 通道分辨率 | 11 bit (0~2047) | 16 bit (0~65535) | 11 bit (364~1684) |
| 信号极性 | 反相 | 正常 | 反相 |
| 额外数据 | flags 标志 | 校验和 | 鼠标 + 键盘 |
| 双向 | SBUS 2 支持双向 | 支持遥测回传 | 单向 |
| 帧间隔 | 7ms / 14ms | ~7ms | 14ms |

## 八、FlySky（富斯）遥控器与 SBUS

### FlySky 支持的协议

FlySky 遥控器（如 FS-i6、FS-i6X）使用 **AFHDS 2A**（第二代自动跳频数字系统）无线协议，接收机可输出多种信号格式：

| 接收机型号 | PWM | PPM | iBUS | SBUS |
|-----------|:---:|:---:|:----:|:----:|
| FS-iA6B | 6ch | - | ✓ | - |
| FS-iA10B | 10ch | - | ✓ | - |
| FS-A8S | - | - | ✓ | ✓ |
| FS-X6B | 6ch | ✓ | ✓ | ✓ |

> **注意**：FS-i6 标准版不支持 SBUS 输出，需要搭配支持 SBUS 的接收机（如 FS-A8S、FS-X6B）。

### FlySky iBUS 协议（富斯自有协议）

iBUS 是 FlySky 自主开发的串行通信协议，与 SBUS 并行存在：

- **波特率**：115200 bps
- **格式**：8N1（标准 TTL 电平，无需取反）
- **帧长度**：32 字节
- **帧结构**：
  - 字节 0~1：帧头（0x20 0x40）
  - 字节 2~29：14 个通道数据，每个 2 字节（小端），范围 1000~2000（中值 1500）
  - 字节 30~31：校验和（0xFFFF 减去前 30 字节之和的低 16 位）
- **优势**：无需硬件取反，标准 TTL 电平，可直接连接 MCU UART

```c
// iBUS 解析示例
void ibus_decode(uint8_t *frame, uint16_t *channels) {
    // 检查帧头
    if (frame[0] != 0x20 || frame[1] != 0x40) return;
    // 解析14个通道
    for (int i = 0; i < 14; i++) {
        channels[i] = frame[2 + i*2] | (frame[3 + i*2] << 8);
    }
    // 校验和验证
    uint16_t checksum = 0xFFFF;
    for (int i = 0; i < 30; i++) checksum -= frame[i];
    uint16_t rx_checksum = frame[30] | (frame[31] << 8);
    // if (checksum != rx_checksum) return; // 校验失败
}
```

## 九、参考来源

- [S.BUS协议 | 接收机与飞控之间的通信协议 - 与非网](https://www.eefocus.com/article/1797810.html)
- [通信中常用的SBUS协议详解 - 华为云社区](https://bbs.huaweicloud.com/blogs/357019)
- [富斯遥控器/接收机的 PWM/PPM/iBUS/SBUS 通道设置](https://icode.best/i/06017056130118)
- [FlySky 官网 - AFHDS3 协议介绍](https://www.flysky-cn.com/flysky-protocol)
- [FlySky FS-i6X 用户手册](https://www.flysky-cn.com/fsi6x)
- [RoboMaster 遥控器接收机用户手册 V1.0](https://www.robomaster.com/zh-CN/products/components/detail/122)
- [DJI DT7-DR16 遥控系统维护手册 V1.7 - RoboMaster 社区](https://bbs.robomaster.com/article/9629)
