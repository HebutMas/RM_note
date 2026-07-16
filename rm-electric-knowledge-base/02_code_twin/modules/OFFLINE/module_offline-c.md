# module_offline.c — 离线检测模块实现

`modules/OFFLINE/module_offline.c`

## 内部变量

```c
static Offline_Device *g_device_list = NULL;  // 链表头
static bool g_initialized = false;            // 幂等标志
static uint8_t g_silent_error = 0;            // 静默故障标志
```

`g_device_list` 是链表头指针，初始 `NULL` 表示空链表。`g_initialized` 防止重复初始化。链表操作见 [[01_extracted/algorithm/data-structure-linked-list]]。

## 检测任务（10ms 周期）

```
offline_detect_task_entry()
  while (1):
    HAL_IWDG_Refresh()                    // 喂看门狗
    now = BSP_DWT_GetTimeline_ms()
    遍历 g_device_list:
      elapsed = now - dev->last_time
      if (elapsed > dev->timeout_ms)
          dev->is_offline = true
      else
          dev->is_offline = false

    聚合故障等级 → 驱动蜂鸣/LED 状态机

    tx_thread_sleep(10)                   // 10ms 周期
```

### 聚合故障等级

遍历完所有设备后，取所有离线设备中 `beep_times` 的最小值作为 `request_beep_times`。时间戳来自 DWT 计数器，见 [[02_code_twin/board/bsp/DWT/bsp_dwt]]：

- 取最小值是因为蜂鸣次数少 = 更紧急（如电机 `beep_times=3` 比超级电容 `beep_times=1` 优先级低，所以 1 次的先响）
- `g_silent_error`：如果有 `beep_times > 0` 的设备离线，清零；如果只有 `beep_times == 0` 的设备离线，置 1

效果：
- 严重故障（`beep_times > 0`）：蜂鸣器按次数闪烁响，LED 红
- 静默故障（`beep_times == 0`）：LED 红灯常亮，蜂鸣器不响
- 全在线：LED 绿灯

### 蜂鸣/LED 状态机

2 秒大周期，内部按 `request_beep_times` 分配：

```
|←────────────── 2s 大周期 ──────────────→|
| 亮100ms | 灭100ms | 亮100ms | 灭100ms | 绿灯 |
|← beep_times 次闪烁 →|    剩余时间绿灯      |
```

- `remaining_beep_cycles` 从 `request_beep_times` 递减，每个子周期（100ms）减一
- 闪烁阶段：`BSP_BEEP_Start()` + `BSP_LED_Show(LED_Red)`
- 间隔阶段：`BSP_BEEP_Stop()` + `BSP_LED_Show(LED_Black)`
- 剩余时间：`BSP_LED_Show(LED_Green)`，蜂鸣器关

LED 驱动见 [[02_code_twin/board/bsp/LED/bsp_led]]，蜂鸣器驱动见 [[02_code_twin/board/bsp/BEEP/bsp_beep]]。

## Module_Offline_init()

```c
if (g_initialized) return;    // 幂等
g_device_list = NULL;
g_initialized = true;

BSP_BEEP_Start();
BSP_LED_Show(LED_Green);      // 初始绿灯

tx_thread_create(..., offline_detect_task_entry, ...);  // 创建检测任务

MX_IWDG_Init();               // 独立看门狗
__HAL_DBGMCU_FREEZE_IWDG();   // 调试器暂停时冻结看门狗
```

必须在所有其他模块之前调用，因为后续模块注册设备依赖 `g_initialized`。在 `MODULE_Init()` 中最先调用。

看门狗的作用：如果检测任务卡死（比如死循环），看门狗超时后复位 MCU。调试器暂停时冻结看门狗，防止断点调试时触发复位。

## Module_Offline_register()

```c
if (!g_initialized) return NULL;   // 必须先 init

BSP_MEM_ALLOC_WAIT(dev, sizeof(Offline_Device), TX_NO_WAIT);
if (dev == NULL) return NULL;      // 内存池不够

// 填充字段
memset(dev, 0, sizeof(*dev));
dev->timeout_ms = cfg->timeout_ms;
dev->beep_times = cfg->beep_times;
dev->last_time  = BSP_DWT_GetTimeline_ms();

// 头插链表
```

内存分配 + 头插链表的完整模式见 [[01_extracted/algorithm/data-structure-linked-list#注册 = 分配 + 填充 + 头插]]。`BSP_MEM_ALLOC_WAIT` 底层是 `tx_byte_allocate`，从 `Robot_Init()` 创建的 20KB 内存池分配。返回 `NULL` 的含义见 [[01_extracted/algorithm/data-structure-linked-list#NULL 是什么]]。

## Module_Offline_device_update()

```c
void Module_Offline_device_update(Offline_Device *dev) {
    dev->last_time = BSP_DWT_GetTimeline_ms();
}
```

一行代码，但设计意义重要：每个设备**独立维护自己的心跳时间戳**。各模块在收到有效数据时调用（CAN 接收回调、UART 解码成功等），只更新自己的设备，不影响其他设备。检测任务统一扫描所有时间戳做超时判断。

## Module_Offline_get_device_status()

```c
uint8_t Module_Offline_get_device_status(Offline_Device *dev) {
    if (dev == NULL) return STATE_OFFLINE;
    return dev->is_offline ? STATE_OFFLINE : STATE_ONLINE;
}
```

`dev == NULL` 返回离线——如果注册失败返回了 `NULL`，后续查询状态不会空指针崩溃，而是安全地返回"离线"。这是防御性编程的典型做法。NULL 的含义见 [[01_extracted/algorithm/data-structure-linked-list#NULL 是什么]]，注册失败返回 NULL 的场景见 [[01_extracted/algorithm/data-structure-linked-list#注册 = 分配 + 填充 + 头插]]。
