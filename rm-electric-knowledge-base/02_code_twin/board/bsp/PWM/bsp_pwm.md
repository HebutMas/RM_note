# bsp_pwm.c/h — PWM 设备抽象

`board/bsp/PWM/bsp_pwm.{c,h}`

## 一句话

封装 STM32 HAL 的定时器 PWM 功能，用链表管理多个 PWM 设备，提供统一的占空比/频率设置接口。

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
    dev->next = devices_list;           // 头插链表
    devices_list = dev;
}
```

`Period` 从 CubeMX 生成的 `htim->Init.Period` 读取，不需要手动维护。

## 占空比的两种设置方式

PWM 占空比 = CCR / (ARR + 1)。`__HAL_TIM_SET_COMPARE` 直接写 CCR 寄存器。BSP 提供两个 API，区别在于入参的含义不同：

### `BSP_PWM_SetDutyCycle(dev, duty)` — 按百分比

```c
uint8_t BSP_PWM_SetDutyCycle(PWM_Device *dev, uint16_t duty) {
    if (duty > 1000) duty = 1000;       // 限制范围 0-1000
    dev->dutyx10 = duty;
    pulse = (dev->Period * duty) / 1000; // 0-1000 → 0-ARR
    __HAL_TIM_SET_COMPARE(dev->htim, dev->Channel, pulse);
}
```

入参 `duty` 范围 0-1000，对应 0.0%-100.0%，精度 0.1%。调用方不需要知道 ARR 是多少，只管"我要 50% 亮度"就传 500。

### `BSP_PWM_SetPulse(dev, pulse)` — 直接写 Pulse 值

```c
uint8_t BSP_PWM_SetPulse(PWM_Device *dev, uint32_t pulse) {
    if (pulse > dev->Period) pulse = dev->Period;  // 不超过 ARR
    dev->dutyx10 = (pulse * 1000) / dev->Period;   // 反算 dutyx10 保持一致
    __HAL_TIM_SET_COMPARE(dev->htim, dev->Channel, pulse);
}
```

入参 `pulse` 直接就是要写入 CCR 的值。调用方需要自己知道 ARR 是多少。

两个函数都会同步更新 `dutyx10` 字段，保持状态一致。

### 谁用哪个

| 调用方 | 用哪个 | 原因 |
|--------|--------|------|
| [[02_code_twin/board/bsp/LED/bsp_led\|LED]] | `SetDutyCycle` | 关心"亮度比例"，不关心 ARR 具体值 |
| [[02_code_twin/board/bsp/BEEP/bsp_beep\|蜂鸣器]] | `SetPulse` | ARR 固定 1000，Offline 模块传的参数本来就是 Pulse 值 |

## 频率设置

```c
uint8_t BSP_PWM_SetPSC(PWM_Device *dev, uint32_t psc) {
    __HAL_TIM_SET_PRESCALER(dev->htim, psc);
}
```

直接写 PSC 寄存器。PWM 频率 = 定时器时钟 / (PSC+1) / (ARR+1)。

## 三种模式

| 模式 | 函数 | 用途 |
|------|------|------|
| `PWM_MODE_BLOCKING` | `HAL_TIM_PWM_Start` | 简单输出，LED/蜂鸣器用 |
| `PWM_MODE_IT` | `HAL_TIM_PWM_Start_IT` | 中断回调，H723 的 WS2812 用 |
| `PWM_MODE_DMA` | `HAL_TIM_PWM_Start_DMA` | DMA 传输，批量数据 |

DMA 模式下用 `TX_EVENT_FLAGS_GROUP` 做完成同步：`HAL_TIM_PWM_PulseFinishedCallback` 遍历链表找到对应设备，`tx_event_flags_set` 通知等待方。
