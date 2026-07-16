# bsp_pwm.c/h — PWM 设备抽象

`board/bsp/PWM/bsp_pwm.{c,h}`

## 一句话

封装 STM32 HAL 的定时器 PWM 功能，用链表管理多个 PWM 设备，提供统一的占空比/频率设置接口。LED 和蜂鸣器都基于它。

## 为什么不直接调 HAL

一个定时器可以有多路通道，LED 需要 3 路（RGB），蜂鸣器需要 1 路。直接调 HAL 要每次传 `htim` 和 `Channel`，容易出错。封装成 `PWM_Device` 后，每个设备一个指针，后续只操作这个指针。

## 设备结构

```c
typedef struct PWM_Device {
    PWM_Device *next;          // 链表指针
    TIM_HandleTypeDef *htim;   // 定时器句柄
    uint32_t Channel;          // 通道
    uint32_t Period;           // 自动重装载值（ARR）
    uint32_t dutyx10;          // 占空比 0-1000（对应 0.0%-100.0%）
    PWM_Mode Mode;             // BLOCKING / IT / DMA
    TX_EVENT_FLAGS_GROUP event_flags;  // 完成事件
} PWM_Device;
```

链表管理，和 [[01_extracted/algorithm/data-structure-linked-list]] 同一套模式。

## 初始化

```c
PWM_Device *BSP_PWM_Device_Init(PWM_Init_Config *config) {
    BSP_MEM_ALLOC_WAIT(dev, sizeof(PWM_Device), TX_NO_WAIT);
    dev->htim    = config->htim;
    dev->Channel = config->Channel;
    dev->Period  = htim->Init.Period;   // 从 CubeMX 配置读取
    dev->dutyx10 = config->dutyx10;
    // 头插链表
    dev->next = devices_list;
    devices_list = dev;
}
```

`Period` 从 CubeMX 生成的 `htim->Init.Period` 读取，不需要手动维护。

## 占空比的两种设置方式

PWM 占空比 = CCR / (ARR + 1)。`__HAL_TIM_SET_COMPARE` 直接写 CCR 寄存器。BSP 提供两个 API，区别在于入参的含义不同：

### 方式一：`BSP_PWM_SetDutyCycle(dev, duty)` — 按百分比

```c
uint8_t BSP_PWM_SetDutyCycle(PWM_Device *dev, uint16_t duty) {
    if (duty > 1000) duty = 1000;       // 限制范围 0-1000
    dev->dutyx10 = duty;
    pulse = (dev->Period * duty) / 1000; // 0-1000 → 0-ARR
    __HAL_TIM_SET_COMPARE(dev->htim, dev->Channel, pulse);
}
```

入参 `duty` 范围 0-1000，对应 0.0%-100.0%，精度 0.1%。内部通过 `PWM_Convert_Dutyx10_To_Pulse()` 换算成 Pulse 值再写 CCR。

调用方不需要知道 ARR 是多少，只管"我要 50% 亮度"就传 500。

### 方式二：`BSP_PWM_SetPulse(dev, pulse)` — 直接写 Pulse 值

```c
uint8_t BSP_PWM_SetPulse(PWM_Device *dev, uint32_t pulse) {
    if (pulse > dev->Period) pulse = dev->Period;  // 不超过 ARR
    dev->dutyx10 = (pulse * 1000) / dev->Period;   // 反算 dutyx10 保持一致
    __HAL_TIM_SET_COMPARE(dev->htim, dev->Channel, pulse);
}
```

入参 `pulse` 直接就是要写入 CCR 的值。调用方需要自己算好 Pulse = ARR × 占空比。

两个函数都会同步更新 `dutyx10` 字段，保持状态一致。

## 蜂鸣器为什么用 SetPulse，LED 为什么用 SetDutyCycle

### LED：`BSP_PWM_SetDutyCycle` — 关心"亮度比例"

F407 的 RGB LED 挂在 TIM5，ARR=65535。LED 的需求是"红色亮 50%"，这是一个**比例**概念，不关心底层 ARR 具体是多少：

```c
// bsp_led.c
uint16_t red_duty = (red * 1000) / 255;   // 0-255 颜色值 → 0-1000 占空比
BSP_PWM_SetDutyCycle(pwm_r, red_duty);    // 传百分比，内部自动换算
```

如果哪天换了板子 ARR 变了，LED 代码不用改——还是传 500 表示 50% 亮度。

### 蜂鸣器：`BSP_PWM_SetPulse` — 关心"绝对 Pulse 值"

F407 蜂鸣器挂在 TIM4，ARR=999（即 `Period=1000-1`）。蜂鸣器的 `BSP_BEEP_Set(psc, pwm)` 由 Offline 模块调用，传入的 `pwm` 参数是**直接控制音量的 Pulse 值**（0-999），不是百分比：

```c
// bsp_beep.c
void BSP_BEEP_Set(uint16_t psc, uint16_t pwm) {
    BSP_PWM_SetPSC(beep_pwm_dev, psc);      // 设置频率
    BSP_PWM_SetPulse(beep_pwm_dev, pwm);    // 直接写 Pulse，不做百分比换算
}
```

原因是蜂鸣器音量控制不需要 0.1% 精度，Offline 模块直接算好了 Pulse 值传进来。用 `SetPulse` 跳过了 `duty → pulse` 的换算，少一层间接。

如果用 `SetDutyCycle` 也行（`pwm * 1000 / ARR` 换算一下），但多此一举——蜂鸣器的 ARR 固定 999，Offline 模块直接按 Pulse 来更直观。

### 对比总结

| | LED (`SetDutyCycle`) | 蜂鸣器 (`SetPulse`) |
|---|---|---|
| 入参含义 | 占空比 0-1000（0.0%-100.0%） | Pulse 值 0-ARR |
| 是否需要知道 ARR | 不需要 | 需要 |
| 适用场景 | 需要跨平台移植、按比例控制 | 固定硬件、直接控制寄存器值 |
| 定时器/ARR | TIM5, ARR=65535 | TIM4, ARR=999 |

## 频率设置

```c
uint8_t BSP_PWM_SetPSC(PWM_Device *dev, uint32_t psc) {
    __HAL_TIM_SET_PRESCALER(dev->htim, psc);
}
```

直接写 PSC 寄存器。PWM 频率 = 定时器时钟 / (PSC+1) / (ARR+1)。

被 [[02_code_twin/board/bsp/BEEP/bsp_beep]] 调用控制蜂鸣器音调。

## 三种模式

| 模式 | 函数 | 用途 |
|------|------|------|
| `PWM_MODE_BLOCKING` | `HAL_TIM_PWM_Start` | 简单输出，LED/蜂鸣器用 |
| `PWM_MODE_IT` | `HAL_TIM_PWM_Start_IT` | 中断回调，H723 的 WS2812 用 |
| `PWM_MODE_DMA` | `HAL_TIM_PWM_Start_DMA` | DMA 传输，批量数据 |

DMA 模式下用 `TX_EVENT_FLAGS_GROUP` 做完成同步：`HAL_TIM_PWM_PulseFinishedCallback` 遍历链表找到对应设备，`tx_event_flags_set` 通知等待方。
