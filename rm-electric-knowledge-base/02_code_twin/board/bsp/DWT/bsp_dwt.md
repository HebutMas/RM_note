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

初始化时还会初始化溢出计数器 `CYCCNT_RountCount = 0` 和 `CYCCNT_LAST = DWT->CYCCNT`（用于溢出检测）。

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

> 该函数不受中断开关影响，可以在临界区和关闭中断时使用。禁止在 RTOS 初始化完成之前或中断临界区使用 `HAL_Delay()` / `tx_thread_sleep()`，应使用本函数。

## GetDeltaT：两次调用之间的时间间隔

```c
float BSP_DWT_GetDeltaT(uint32_t *cnt_last) {
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;
    DWT_CNT_Update();
    return dt;
}
```

### 核心要点：`cnt_last` 是外部传入的参数

`cnt_last` 是一个 **`uint32_t *` 指针**，由调用方自己维护，**不是 DWT 模块内部的全局变量**。DWT 内部有一个 `CYCCNT_LAST` 用于溢出检测，但 `GetDeltaT` 用的不是它——而是用调用方传入的 `cnt_last`。

这意味着：

- **每个调用方各自维护自己的时间戳**，互不干扰
- 多个模块同时调用 `GetDeltaT`，各自拿到的是自己两次调用之间的间隔
- 如果 `cnt_last` 是 DWT 内部全局变量，那一个模块调用后会覆盖其他模块的时间戳，导致其他模块下次调用算出的 dt 是错的

### 使用示例

#### 示例 1：PID 模块（结构体成员变量）

PID 控制器在自己的结构体里维护 `DWT_CNT` 字段：

```c
// pid.h
typedef struct {
    float Kp, Ki, Kd;
    float dt;           // 两次计算的时间间隔（秒）
    uint32_t DWT_CNT;   // 调用方自己维护的时间戳
} PID_TypeDef;

// pid.c
float PID_Calculate(PID_TypeDef *pid, float set, float fdb) {
    pid->dt = BSP_DWT_GetDeltaT(&pid->DWT_CNT);  // 传入自己的 cnt
    // 用 pid->dt 做积分和微分...
}
```

每个 PID 实例有自己的 `DWT_CNT`，多个 PID 互不干扰。

#### 示例 2：INS 模块（结构体成员变量）

```c
// module_ins.h
typedef struct {
    float dt;
    uint32_t dwt_cnt;   // 自己的时间戳
} INS_t;

// module_ins.c
void INS_Update(INS_t *ins) {
    ins->dt = BSP_DWT_GetDeltaT(&ins->dwt_cnt);
    // 用 ins->dt 做姿态解算...
}
```

#### 示例 3：局部静态变量（参考 BMI088 校准模块）

如果不需要多实例，可以用 `static` 局部变量：

```c
void BMI088_Cali_Task(void) {
    static uint32_t dt_cnt = 0;                    // 局部静态变量，函数内维护
    float elapsed = BSP_DWT_GetDeltaT(&dt_cnt);    // 每次调用算出距上次的间隔
    // 累加 elapsed 做超时判断...
}
```

`static` 保证变量在函数调用间保持值，但只在当前函数内可见。

### 为什么用 `uint32_t` 减法不怕溢出

`cnt_now - *cnt_last` 是两个 32 位无符号数的减法。即使 `cnt_now` 溢出回卷了（比 `*cnt_last` 小），无符号减法的结果仍然正确——这是补码运算的天然特性，详见 [[01_extracted/algorithm/computer-basics#无符号整数回卷]]。

168MHz 下 CYCCNT 约 25.5 秒溢出一次，只要两次调用间隔不超过 25.5 秒，`GetDeltaT` 的结果就是正确的。

## 时间轴：BSP_DWT_GetTimeline_ms

```c
float BSP_DWT_GetTimeline_ms(void) {
    DWT_SysTimeUpdate();
    return SysTime.s * 1000.0f + SysTime.ms + SysTime.us * 0.001f;
}
```

`CYCCNT` 是 32 位，168MHz 下约 25.5 秒溢出一次。`DWT_CNT_Update()` 检测溢出（当前值 < 上次值），用 `CYCCNT_RountCount` 记录溢出次数，拼成 64 位计数器，再换算成 s/ms/us。

`DWT_CNT_Update()` 内部有简单的重入保护（`dwt_lock_state` 标志），防止 `GetDeltaT` 和 `DWT_SysTimeUpdate` 嵌套调用时重复递增溢出计数。

被 [[02_code_twin/modules/OFFLINE/module_offline-c]] 的检测任务调用获取时间戳，判断设备是否超时。

## 性能测量宏

```c
BSP_DWT_MEASURE_START();
// ... 要测量的代码 ...
BSP_DWT_MEASURE_END("function_name");
```

输出 `[PERF] function_name: 1234 cycles, 7.35 us`，用于性能分析。F407 和 H723 分别硬编码了各自的时钟频率（168MHz / 480MHz）。
