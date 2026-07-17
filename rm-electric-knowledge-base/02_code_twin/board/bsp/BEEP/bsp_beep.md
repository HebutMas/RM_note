# bsp_beep.c/h — 蜂鸣器驱动

`board/bsp/BEEP/bsp_beep.{c,h}`

## 一句话

无源蜂鸣器的 PWM 驱动封装，PSC 控制音调，Pulse 控制音量。被 Offline 模块调用做设备掉线报警。

## 硬件

C 板（F407）板载无源蜂鸣器，挂 TIM4_CH3（PD14），CubeMX 配置 PSC=20, ARR=999（有效周期 1000）。H723 用 TIM12_CH2。详见 [[01_extracted/hardware/c-board-resources#蜂鸣器]]。

无源蜂鸣器需要持续 PWM 信号才能发声：**频率决定音调，占空比决定音量**。

## API

```c
void BSP_BEEP_Init();                          // 初始化 PWM 设备
void BSP_BEEP_Start();                         // 开始发声
void BSP_BEEP_Stop();                          // 停止发声
void BSP_BEEP_Set(uint16_t psc, uint16_t pwm); // 设置音调(psc)和音量(pwm)
```

## 初始化

```c
static PWM_Device *beep_pwm_dev = NULL;

void BSP_BEEP_Init() {
#if defined(STM32H723xx)
    PWM_Init_Config pwm_config = {.htim = &htim12, .Channel = TIM_CHANNEL_2, .dutyx10 = 0, .Mode = PWM_MODE_BLOCKING};
#elif defined(STM32F407xx)
    PWM_Init_Config pwm_config = {.htim = &htim4, .Channel = TIM_CHANNEL_3, .dutyx10 = 0, .Mode = PWM_MODE_BLOCKING};
#endif
    beep_pwm_dev = BSP_PWM_Device_Init(&pwm_config);
    BSP_BEEP_Stop();  // 默认关闭
}
```

蜂鸣器就是一个 PWM 设备，初始化时 `dutyx10 = 0`（静音），底层 PWM 抽象见 [[02_code_twin/board/bsp/PWM/bsp_pwm]]。

## BSP_BEEP_Set：音调和音量

```c
void BSP_BEEP_Set(uint16_t psc, uint16_t pwm) {
    if (beep_pwm_dev == NULL) return;
    BSP_PWM_SetPSC(beep_pwm_dev, psc);     // 写 PSC 寄存器 → 控制频率（音调）
    BSP_PWM_SetPulse(beep_pwm_dev, pwm);   // 写 CCR 寄存器 → 控制占空比（音量）
}
```

F407 的 ARR 有效值 = 1000（PSC=20, ARR=999），所以 **pulse 值刚好是占空比的 10 倍**：

| pulse 值 | 占空比 | 效果 |
|----------|--------|------|
| 0 | 0% | 静音 |
| 100 | 10% | 正常音量 |
| 500 | 50% | 很响 |
| 1000 | 100% | 最响 |

用 `BSP_PWM_SetPulse` 而不是 `BSP_PWM_SetDutyCycle`，因为 ARR 固定 1000，直接按 Pulse 来更直观。两种 API 的区别见 [[02_code_twin/board/bsp/PWM/bsp_pwm]]。

## 实际调用过程：Offline 模块报警

蜂鸣器唯一的使用者是 Offline 模块：

```c
// module_offline.h
#if defined(STM32H723xx)
#define OFFLINE_BEEP_TUNE_VALUE 100   // psc
#define OFFLINE_BEEP_CTRL_VALUE 100   // pulse
#elif defined(STM32F407xx)
#define OFFLINE_BEEP_TUNE_VALUE 50    // psc
#define OFFLINE_BEEP_CTRL_VALUE 100   // pulse
#endif
```

```c
// module_offline.c — 检测任务中
// 有设备掉线时：
BSP_BEEP_Set(OFFLINE_BEEP_TUNE_VALUE, OFFLINE_BEEP_CTRL_VALUE);  // psc=50, pulse=100
// 全部在线时：
BSP_BEEP_Set(0, 0);  // 静音
// 启动时：
BSP_BEEP_Start();
```

F407 实际调用 `BSP_BEEP_Set(50, 100)`：

- **psc=50** → PWM 频率 = 84MHz / (50+1) / (999+1) ≈ 1647Hz，控制音调
- **pulse=100** → 占空比 = 100/1000 = 10%，控制音量（比较小的音量）

详见 [[02_code_twin/modules/OFFLINE/module_offline-c]]。

## 为什么用 SetPulse 而不是 SetDutyCycle

蜂鸣器的 ARR 固定 1000，pulse 值直接等于占空比的千分比。用 `SetPulse(100)` 表示"10% 音量"比用 `SetDutyCycle(100)`（0-1000 刻度）更直观——因为 Offline 模块传入的参数本来就是直接写 CCR 的值，不需要百分比换算这一步。

对比 LED 用 `SetDutyCycle`：LED 的 ARR=65535，而且需求是"50% 亮度"这种比例概念，用百分比更自然。详见 [[02_code_twin/board/bsp/LED/bsp_led]]。
