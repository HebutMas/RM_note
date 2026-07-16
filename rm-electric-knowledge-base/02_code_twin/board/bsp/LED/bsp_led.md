# bsp_led.c/h — RGB LED 驱动

`board/bsp/LED/bsp_led.{c,h}`

## 为什么在 board/bsp 而不是板子目录

`board/bsp/` 是共享硬件抽象层，两块板子（`board/dji_c/` F407 和 `board/damiao_h7/` H723）共用同一份 BSP 代码。CubeMX 生成的引脚配置和时钟在各板子目录里，BSP 层用 `#if defined()` 条件编译区分差异。

## 硬件背景

C 板（F407）板载共阳极 RGB LED，红=PH12、绿=PH11、蓝=PH10。H723 开发板改用 WS2812 灯珠，通过 SPI 驱动。硬件引脚详见 [[01_extracted/hardware/c-board-resources#RGB LED]]。

## API

```c
void BSP_LED_Init(void);       // 初始化 PWM/SPI 设备
void BSP_LED_Show(uint32_t aRGB);  // 显示颜色，ARGB 格式
```

颜色宏定义：

```c
#define LED_Black  0xFF000000   // 全灭
#define LED_Green  0xFF00FF00
#define LED_Red    0xFFFF0000
```

## F407 实现

三个通道各注册一个 PWM 设备（TIM5 CH1/2/3），通过占空比控制亮度。PWM 设备抽象见 [[02_code_twin/board/bsp/PWM/bsp_pwm]]：

```c
PWM_Init_Config config = {.htim = &htim5, .Channel = TIM_CHANNEL_3, .dutyx10 = 0, .Mode = PWM_MODE_BLOCKING};
pwm_r = BSP_PWM_Device_Init(&config);
```

`BSP_LED_Show` 把 0-255 的颜色值转换为 0-1000 的 PWM 占空比：

```c
uint16_t red_duty = (red * 1000) / 255;
BSP_PWM_SetDutyCycle(pwm_r, red_duty);
```

alpha 通道做亮度缩放：`red = (red * alpha) / 255`，alpha=0 全灭，alpha=255 满亮度。

## H723 实现

WS2812 是单线协议灯珠，每个 bit 用 SPI 传送一个字节：`0xC0` 表示 0 码，`0xF0` 表示 1 码。24 个 bit（GRB 顺序）编码为 24 字节的 SPI 数据。

## 被 Offline 模块调用

Offline 检测任务通过 `BSP_LED_Show()` 切换 LED 颜色表示系统状态：绿色=全在线，红色=有设备离线。详见 [[02_code_twin/modules/OFFLINE/module_offline-c]]。
