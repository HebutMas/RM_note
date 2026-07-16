# module_offline.h — 离线检测模块接口

`modules/OFFLINE/module_offline.h`

## 一句话

声明离线检测模块的对外接口：初始化、注册设备、更新心跳、查询状态。

## 宏定义

```c
#define OFFLINE_TASK_STACK_SIZE 1024
#define OFFLINE_TASK_PRIORITY  6
```

检测任务栈 1024 字节，优先级 6（较低，不与控制循环抢 CPU）。

```c
#if defined(STM32F407xx)
    #define OFFLINE_BEEP_PSC  9     // 预分频，4000Hz 附近
    #define OFFLINE_BEEP_PWM  500   // 50% 占空比
#elif defined(STM32H723xx)
    #define OFFLINE_BEEP_PSC  17
    #define OFFLINE_BEEP_PWM  1000
#endif
```

蜂鸣器参数按主频区分，F407 为 168MHz，H723 为 480MHz，预分频值不同才能得到相近的音调。蜂鸣器硬件见 [[02_code_twin/board/bsp/BEEP/bsp_beep]]。

## 状态枚举

```c
typedef enum {
    STATE_ONLINE  = 0,
    STATE_OFFLINE = 1,
} Offline_State_e;
```

## 注册配置

```c
typedef struct {
    char     name[32];       // 设备名称，日志用
    uint32_t timeout_ms;     // 超时阈值（ms）
    uint8_t  beep_times;     // 蜂鸣次数：0=静默故障（红灯不响），>0=严重故障（响蜂鸣）
    uint8_t  enable;         // 是否启用检测
} Offline_Init_config_t;
```

`beep_times` 是核心设计：电机离线 `beep_times=3`（响 3 次，严重），遥控器离线 `beep_times=0`（红灯常亮不响，静默）。

## 离线设备结构体

```c
typedef struct Offline_Device {
    struct Offline_Device *next;     // 链表指针
    char     name[32];
    uint32_t timeout_ms;
    uint8_t  beep_times;
    uint8_t  is_offline;
    uint32_t last_time;              // 上次心跳时间戳（ms，来自 DWT）
    uint8_t  enable;
} Offline_Device;
```

链表节点结构，`next` 指向下一个设备。链表基础知识见 [[01_extracted/algorithm/data-structure-linked-list]]。

## API

```c
void            Module_Offline_init(void);
Offline_Device *Module_Offline_register(Offline_Init_config_t *config);
void            Module_Offline_device_update(Offline_Device *dev);
uint8_t         Module_Offline_get_device_status(Offline_Device *dev);
void            Module_Offline_device_enable(Offline_Device *dev);
void            Module_Offline_device_disable(Offline_Device *dev);
```

实现解析见 [[02_code_twin/modules/OFFLINE/module_offline-c]]，初始化全流程见 [[03_moc/Robot-Init-Walkthrough]]。
