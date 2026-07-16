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

## 占空比设置

```c
uint8_t BSP_PWM_SetDutyCycle(PWM_Device *dev, uint16_t duty) {
    dev->dutyx10 = duty;
    pulse = (dev->Period * duty) / 1000;   // 0-1000 → 0-ARR
    __HAL_TIM_SET_COMPARE(dev->htim, dev->Channel, pulse);
}
```

`duty` 范围 0-1000（不是 0-100），精度到 0.1%。`__HAL_TIM_SET_COMPARE` 直接写 CCR 寄存器，立即生效。

被 [[02_code_twin/board/bsp/LED/bsp_led]] 调用控制 RGB 亮度。

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
