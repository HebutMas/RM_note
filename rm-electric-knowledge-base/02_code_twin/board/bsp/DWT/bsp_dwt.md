# bsp_dwt.c/h — DWT 周期计数器

`board/bsp/DWT/bsp_dwt.{c,h}`

## 一句话

利用 Cortex-M 内核自带的 DWT（Data Watchpoint and Trace）单元的 `CYCCNT` 寄存器，提供微秒级精确延时和全局时间轴。不占用任何定时器外设。

## 为什么用它

- `HAL_Delay()` 依赖 SysTick，但 SysTick 被 ThreadX 占用了（系统节拍），RTOS 初始化后不能再用
- `tx_thread_sleep()` 最小单位是 1 个 tick（通常 1ms），精度不够
- DWT 是 CPU 内核寄存器，不占外设，不受中断开关影响，可以在临界区使用

## 初始化

```c
void BSP_DWT_Init(uint32_t CPU_Freq_mHz) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // 使能 DWT
    DWT->CYCCNT = 0;                                  // 清零计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              // 开始计数
    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
}
```

C 板（F407）传 168，A 板（H723）传 480。在 `BSP_Init()` 中调用，见 [[03_moc/Robot-Init-Walkthrough#全流程]]。

## 延时：BSP_DWT_Delay

```c
void BSP_DWT_Delay(float Delay) {
    uint32_t tickstart = DWT->CYCCNT;
    uint32_t wait_cycles = (uint32_t)(Delay * CPU_FREQ_Hz);
    while ((DWT->CYCCNT - tickstart) < wait_cycles) {
        __asm__ volatile("" ::: "memory");  // 防止编译器优化掉空循环
    }
}
```

参数单位是**秒**。`Delay * CPU_FREQ_Hz` 把秒换算成时钟周期数，忙等直到 CYCCNT 走够周期数。

- `BSP_DWT_Delay(0.000005f)` = 5 微秒（PS2 软件 SPI 半时钟周期）
- `__asm__ volatile("" ::: "memory")` 是编译器屏障，防止 `-O2` 优化把 while 循环删掉

## 时间轴：BSP_DWT_GetTimeline_ms

```c
float BSP_DWT_GetTimeline_ms(void) {
    DWT_SysTimeUpdate();
    return SysTime.s * 1000.0f + SysTime.ms + SysTime.us * 0.001f;
}
```

`CYCCNT` 是 32 位，168MHz 下约 25.5 秒溢出一次。`DWT_CNT_Update()` 检测溢出（当前值 < 上次值），用 `CYCCNT_RountCount` 记录溢出次数，拼成 64 位计数器，再换算成 s/ms/us。

被 [[02_code_twin/modules/OFFLINE/module_offline-c]] 的检测任务调用获取时间戳，判断设备是否超时。

## 性能测量宏

```c
BSP_DWT_MEASURE_START();
// ... 要测量的代码 ...
BSP_DWT_MEASURE_END("function_name");
```

输出 `[PERF] function_name: 1234 cycles, 7.35 us`，用于性能分析。
