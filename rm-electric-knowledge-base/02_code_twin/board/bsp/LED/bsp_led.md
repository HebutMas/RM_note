# bsp_led.c/h — RGB LED 驱动

`board/bsp/LED/bsp_led.{c,h}`

## 为什么在 board/bsp 而不是板子目录

`board/bsp/` 是共享硬件抽象层，两块板子（`board/dji_c/` F407 和 `board/damiao_h7/` H723）共用同一份 BSP 代码。CubeMX 生成的引脚配置和时钟在各板子目录里，BSP 层用 `#if defined(STM32F407xx)` / `#if defined(STM32H723xx)` 条件编译区分两套硬件。

硬件引脚详见 [[01_extracted/hardware/c-board-resources#RGB LED]]。

## API

```c
void BSP_LED_Init(void);            // 初始化 PWM/SPI 设备
void BSP_LED_Show(uint32_t aRGB);   // 显示颜色，ARGB 格式
```

## 32 位 ARGB 颜色编码

入参 `aRGB` 是 `uint32_t`（4 字节 = 32 位），格式为 `0xAARRGGBB`：

```
  AA        RR        GG        BB
  透明度    红        绿        蓝
  8 bit    8 bit     8 bit     8 bit
```

| 通道 | 字节 | 位掩码 | 含义 |
|------|------|--------|------|
| Alpha (A) | 最高字节 [31:24] | `0xFF000000` | 亮度缩放，0=全灭，0xFF=满亮度 |
| Red (R) | [23:16] | `0x00FF0000` | 红色分量，0~255 |
| Green (G) | [15:8] | `0x0000FF00` | 绿色分量，0~255 |
| Blue (B) | [7:0] | `0x000000FF` | 蓝色分量，0~255 |

颜色宏定义：

```c
#define LED_Black  0xFF000000   // A=FF, R=00, G=00, B=00 → 全灭
#define LED_Red    0xFFFF0000   // A=FF, R=FF, G=00, B=00 → 满亮度红
#define LED_Green  0xFF00FF00   // A=FF, R=00, G=FF, B=00 → 满亮度绿
#define LED_Blue   0xFF0000FF   // A=FF, R=00, G=00, B=FF → 满亮度蓝
```

> **为什么用 32 位而不是 24 位（RGB）**：多了一个 Alpha 通道做亮度缩放。`BSP_LED_Show` 内部用 alpha 对 RGB 三通道做等比衰减：`red = (red * alpha) / 255`。alpha=0xFF 时不衰减（满亮度），alpha=0x80 时亮度减半。这样一个参数同时控制颜色和亮度，调用方不需要单独调 PWM 占空比。

> **为什么用 `uint32_t` 而不是 4 个 `uint8_t`**：单个 32 位整数传参更简洁，颜色可以用宏直接定义（如 `0xFFFF0000`），不用结构体初始化。内存中一个 `uint32_t` 就是 4 字节，和 4 个 `uint8_t` 一样大。

## F407 实现：三路 PWM

### 初始化

F407 板载共阳极 RGB LED（红=PH12、绿=PH11、蓝=PH10），挂在 TIM5 的 CH3/CH2/CH1。初始化时各注册一个 PWM 设备：

```c
PWM_Init_Config config = {
    .Channel = TIM_CHANNEL_3, .dutyx10 = 0,
    .htim = &htim5, .Mode = PWM_MODE_BLOCKING
};
pwm_r = BSP_PWM_Device_Init(&config);
config.Channel = TIM_CHANNEL_2;
pwm_g = BSP_PWM_Device_Init(&config);
config.Channel = TIM_CHANNEL_1;
pwm_b = BSP_PWM_Device_Init(&config);
```

PWM 设备抽象见 [[02_code_twin/board/bsp/PWM/bsp_pwm]]。初始化后调 `BSP_PWM_Start` 启动三路 PWM 输出。

### 显示颜色

`BSP_LED_Show` 执行流程：

**第一步：从 32 位整数中提取 4 个通道**

```c
alpha = (aRGB & 0xFF000000) >> 24;   // 取最高字节
red   = (aRGB & 0x00FF0000) >> 16;   // 取第 2 字节
green = (aRGB & 0x0000FF00) >> 8;    // 取第 3 字节
blue  = (aRGB & 0x000000FF) >> 0;    // 取最低字节
```

位掩码 `&` 提取对应字节，右移 `>>` 移到最低 8 位。这是位操作基础，详见 [[01_extracted/algorithm/computer-basics#位操作]]。

**第二步：应用 Alpha 亮度缩放**

```c
if (alpha != 0xFF) {
    red   = (red * alpha) / 255;
    green = (green * alpha) / 255;
    blue  = (blue * alpha) / 255;
}
```

alpha=0xFF 时跳过（满亮度），否则按比例衰减。比如 alpha=0x80（128），red=255 → `255*128/255 = 128`，亮度减半。

**第三步：0-255 颜色值 → 0-1000 PWM 占空比**

```c
uint16_t red_duty = (red * 1000) / 255;
BSP_PWM_SetDutyCycle(pwm_r, red_duty);
```

`BSP_PWM_SetDutyCycle` 接受 0-1000 的占空比（对应 0.0%-100.0%），内部自动换算成 Pulse 值写 CCR 寄存器。为什么用 `SetDutyCycle` 而不是 `SetPulse`：LED 关心"亮度比例"而非绝对寄存器值，换板子 ARR 变了代码不用改。详见 [[02_code_twin/board/bsp/PWM/bsp_pwm#占空比的两种设置方式]]。

> **共阳极 LED**：占空比 100% = 该通道全亮（PWM 输出低电平驱动 LED 导通）。具体极性由 CubeMX 配置的 `TIM_OCPOLARITY` 决定，BSP 层不关心。

## H723 实现：WS2812 SPI 驱动

H723 开发板用 WS2812 灯珠（单线协议），通过 SPI6 驱动。初始化注册 SPI 设备，`BSP_LED_Show` 把 24 bit 颜色编码为 24 字节 SPI 数据。

### 编码方式

WS2812 每个 bit 用一个 SPI 字节表示：

| bit 值 | SPI 字节 | 含义 |
|--------|---------|------|
| 0 | `0xC0` | 高电平时间短 |
| 1 | `0xF0` | 高电平时间长 |

颜色数据按 GRB 顺序（不是 RGB），每个通道 8 bit，共 24 bit = 24 字节 SPI 数据：

```c
for (int i = 0; i < 8; i++) {
    buf[7 - i]  = (((green >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
    buf[15 - i] = (((red >> i)   & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
    buf[23 - i] = (((blue >> i)  & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
}
```

逐 bit 取出 green/red/blue 的每一位，根据 0/1 选择对应的 SPI 编码字节。高位先发（`7-i` 确保 MSB first）。

## 被谁调用

### 初始化：BSP_Init()

`BSP_LED_Init()` 在 `BSP_Init()` 阶段调用，初始化 PWM 或 SPI 设备。调用位置见 [[03_moc/Robot-Init-Walkthrough#2-bsp-init]]。

### 运行时：Offline 模块

Offline 检测任务通过 `BSP_LED_Show()` 切换 LED 颜色表示系统状态：绿色=全在线，红色=有设备离线。详见 [[02_code_twin/modules/OFFLINE/module_offline-c]]。
