# bsp_beep.c/h — 蜂鸣器驱动

`board/bsp/BEEP/bsp_beep.{c,h}`

## 为什么在 board/bsp

`board/bsp/` 是共享硬件抽象层，F407 和 H723 共用。CubeMX 配置在 `board/dji_c/` 和 `board/damiao_h7/` 各自目录里，BSP 代码用 `#if defined()` 区分两套硬件。

## 硬件背景

C 板（F407）板载无源蜂鸣器，PWM 驱动，额定频率 4000Hz，对应 TIM4_CH3（PD14）。H723 用 TIM12 CH2。详见 [[01_extracted/hardware/c-board-resources#蜂鸣器]]。

无源蜂鸣器需要持续 PWM 信号才能发声，频率决定音调，占空比决定音量。

## API

```c
void BSP_BEEP_Init();                   // 初始化 PWM 设备
void BSP_BEEP_Start();                  // 开始发声
void BSP_BEEP_Stop();                   // 停止发声
void BSP_BEEP_Set(uint16_t psc, uint16_t pwm);  // 设置音调(psc)和音量(pwm)
```

## 实现

蜂鸣器就是一个 PWM 设备，PWM 抽象见 [[02_code_twin/board/bsp/PWM/bsp_pwm]]。`BSP_BEEP_Set` 直接操作定时器的 PSC（预分频器，控制频率）和 Pulse（比较值，控制占空比）：

```c
static PWM_Device *beep_pwm_dev = NULL;

void BSP_BEEP_Set(uint16_t psc, uint16_t pwm) {
    if (beep_pwm_dev == NULL) return;       // NULL 安全检查
    BSP_PWM_SetPSC(beep_pwm_dev, psc);      // 频率 = 定时器时钟 / (psc+1) / (arr+1)
    BSP_PWM_SetPulse(beep_pwm_dev, pwm);    // 占空比 = pulse / (arr+1)
}
```

`beep_pwm_dev` 初始化前为 `NULL`，`BSP_BEEP_Set` 检查后才操作，避免空指针 HardFault。这个模式见 [[01_extracted/algorithm/data-structure-linked-list#NULL 是什么]]。

F407 用 `htim4 CH3`，H723 用 `htim12 CH2`，条件编译区分。

## 被 Offline 模块调用

Offline 检测任务通过 `BSP_BEEP_Start()` / `BSP_BEEP_Stop()` 控制蜂鸣器报警，`BSP_BEEP_Set()` 设置音调。详见 [[02_code_twin/modules/OFFLINE/module_offline-c]]。
