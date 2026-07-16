# module_remote.c/h — 遥控器模块层

`modules/REMOTE/module_remote.{c,h}`

## 一句话

用编译期宏选择 SBUS 或 DT7，创建两个 ThreadX 任务分别解码遥控器和图传数据，解码后直接写入全局 `g_remote_data`。

## 统一数据结构：`Remote_Data_t`

```c
typedef struct {
    int16_t       channels[16];   // ch[0..3]: 摇杆, ch[4..15]: 扩展通道
    dt7_custom_t  dt7;            // 拨杆 + 滚轮 (DT7 专属)
    vt03_custom_t vt03;           // VT03 自定义数据
    mouse_state_t mouse;          // 鼠标
    keyboard_state_t keyboard;    // 键盘
} Remote_Data_t;
```

不同遥控器子模块写入不同字段：SBUS 只填 `channels`，DT7 填 `channels` + `dt7` + `mouse` + `keyboard`。应用层通过 `Module_Remote_get_data()` 获取统一指针，不需要关心是哪种遥控器。

## 宏定义

```c
#define REMOTE_SOURCE 1       // 0=none, 1=sbus, 2=dt7
#define REMOTE_VT_SOURCE 0    // 0=none, 1=vt02, 2=vt03
#define REMOTE_DEAD_ZONE 10   // 摇杆死区
```

这些宏在 `module_remote.h` 中有默认值，但可以被 `module_config.h`（由 `generate_headers.cmake` 生成）覆盖。CMake → `generate_headers.cmake` → `module_config.h` → C 代码，这条链路让构建系统控制运行时配置。

通道值范围宏也在这个文件里：

```c
#define SBUS_CHX_BIAS 1024       // SBUS 中值
#define DT7_CH_VALUE_OFFSET 1024 // DT7 中值（和 SBUS 相同）
#define DT7_CH_VALUE_MIN 364     // DT7 最小值
#define DT7_CH_VALUE_MAX 1684    // DT7 最大值
```

数值来源见 [[01_extracted/remote/remote_protocol#数值范围]]。

## 初始化：`Module_Remote_init()`

```c
memset(&g_remote_data, 0, sizeof(g_remote_data));

#if (REMOTE_SOURCE == 1)
    remote_sbus_init(&g_rc_offline);    // SBUS 初始化 + 离线注册
#elif (REMOTE_SOURCE == 2)
    remote_dt7_init(&g_rc_offline);     // DT7 初始化 + 离线注册
#endif

#if (REMOTE_VT_SOURCE == 1)
    remote_vt02_init(&g_vt_offline);
#elif (REMOTE_VT_SOURCE == 2)
    remote_vt03_init(&g_vt_offline);
#endif

// 创建两个任务
tx_thread_create(&g_remote_ctrl_task, "Remote Ctrl", remote_ctrl_task_entry, ...);
tx_thread_create(&g_vt_task, "Remote VT", vt_task_entry, ...);
```

子模块的 `init` 函数各自配置 UART、注册离线设备，模块层只负责调度。SBUS 初始化见 [[02_code_twin/modules/REMOTE/SBUS/sbus#初始化]]，DT7 见 [[02_code_twin/modules/REMOTE/DT7/dt7#初始化]]。

## 解码任务

```c
static void remote_ctrl_task_entry(ULONG arg) {
    while (1) {
#if (REMOTE_SOURCE == 1)
        remote_sbus_decode(&g_remote_data, g_rc_offline);
#elif (REMOTE_SOURCE == 2)
        remote_dt7_decode(&g_remote_data, g_rc_offline);
#endif
    }
}
```

任务阻塞在 `BSP_UART_Read(TX_WAIT_FOREVER)` 上，不消耗 CPU。收到数据后解码写入全局变量，然后再次阻塞等待。

## 离线状态查询

```c
uint8_t Module_Remote_get_offline_status(void) {
    uint8_t rc = Module_Offline_get_device_status(g_rc_offline);
    uint8_t vt = Module_Offline_get_device_status(g_vt_offline);
    return (rc == STATE_ONLINE ? 1 : 0) | (vt == STATE_ONLINE ? 2 : 0);
}
```

返回值 bit0 = 遥控器在线，bit1 = 图传在线。`Module_Offline_get_device_status` 的 NULL 安全检查见 [[02_code_twin/modules/OFFLINE/module_offline-c#Module_Offline_get_device_status()]]——如果 `g_rc_offline` 或 `g_vt_offline` 为 NULL（注册失败），返回 `STATE_OFFLINE`。

## 与 03 层的链接

初始化调用顺序见 [[03_moc/Robot-Init-Walkthrough#MODULE_Init() 调用顺序]]，Remote 在 Offline 之后初始化。
