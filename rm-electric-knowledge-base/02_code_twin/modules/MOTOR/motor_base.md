# motor_base.c/h — 电机基类与多态分发

`modules/MOTOR/motor_base.{c,h}`

## 一句话

定义 `Motor_Base` 抽象基类，用链表管理所有电机实例，`Motor_UpdateAll()` 遍历链表调用每个电机注册的 `ControlAndSend` 回调函数，实现 C 语言多态。

## 基类结构体

```c
struct Motor_Base {
    Motor_Base       *next;           // 链表指针（头插法）
    Motor_Type_e      type;           // 电机类型枚举
    Motor_Transport_e transport;      // 传输层：CAN / UART / PWM

    Motor_Info_s       info;          // 电机参数（类型、减速比、转矩常数、最大力矩）
    Motor_Setting_s    setting;       // 控制设置（闭环类型、使能、反转标志、反馈来源）
    Motor_Controller_s controller;    // 控制器（PID/LQR、参考值、输出）
    Motor_Measure_s    measure;       // 测量数据（速度、角度、力矩）
    Offline_Device    *offline_dev;   // 离线检测设备

    void *transport_dev;              // 传输设备指针（CAN_Device* / UART_Device* / PWM_Device*）

    void (*ControlAndSend)(Motor_Base *motor);  // 控制回调函数指针
};
```

字段说明：

| 字段 | 作用 | 谁填充 |
|------|------|--------|
| `next` | 链表指针，`Motor_Register` 头插时设置 | `Motor_Register` |
| `type` | 电机类型（GM6020/M3508/M2006/DM4310/...） | 子类 Init |
| `transport` | 传输方式（CAN/UART/PWM） | 子类 Init |
| `info` | 电机参数，来自 [[01_extracted/motor/motor_params]] | 子类 Init（从 config 复制） |
| `setting` | 闭环类型、使能标志、反转/反馈来源 | 子类 Init（从 config 复制） |
| `controller` | PID/LQR 实例、参考值 ref、输出 output | 子类 Init + 运行时更新 |
| `measure` | 速度/角度/力矩反馈 | CAN 接收回调 |
| `offline_dev` | 离线检测设备 | 子类 Init（调用 `Module_Offline_register`） |
| `transport_dev` | CAN/UART/PWM 设备指针 | 子类 Init（调用 `BSP_CAN_Device_Init`） |
| `ControlAndSend` | 控制+发送回调 | 子类 Init（注册自己的实现） |

函数指针和 `MOTOR_GET_DERIVED` 宏的原理详见 [[01_extracted/algorithm/function-pointer-pattern]]。

## 链表注册

```c
static Motor_Base *g_motor_list = NULL;

void Motor_Register(Motor_Base *motor) {
    if (motor == NULL) return;
    motor->next = g_motor_list;    // 头插法
    g_motor_list = motor;
}
```

头插法，最新注册的电机在链表头部。链表基础见 [[01_extracted/algorithm/data-structure-linked-list#头插法]]。

## 遍历调用

```c
void Motor_UpdateAll(void) {
    for (Motor_Base *m = g_motor_list; m; m = m->next) {
        if (m->ControlAndSend)
            m->ControlAndSend(m);
    }
}
```

遍历链表，对每个电机调用它自己注册的 `ControlAndSend`。大疆电机走 `dji_control`，达妙电机走 `dm_ControlAndSend`，调用者不需要区分。

## Motor 任务调度

```c
// module_motor.c
static void motor_task_entry(ULONG thread_input) {
    while (1) {
        Motor_UpdateAll();       // 1. 遍历所有电机，计算控制量
        PowerControl_Update();   // 2. 功率控制
        Motor_DJI_Flush();       // 3. 大疆电机批量发送 CAN 帧
        tx_thread_sleep(2);     // 4. 2ms 周期
    }
}
```

> **关键**：`Motor_UpdateAll()` 之后紧接 `Motor_DJI_Flush()`。大疆电机的 `dji_control` 只填缓冲区不发送，需要 `Flush` 批量发；达妙电机的 `dm_ControlAndSend` 在回调里直接发送，不需要 Flush。原因详见 [[02_code_twin/modules/MOTOR/DJI/motor_dji]] 和 [[02_code_twin/modules/MOTOR/DAMIAO/motor_damiao]]。

## 派生类继承方式

```c
// DJI_Motor_t — 大疆电机
typedef struct {
    Motor_Base    base;         // [必须首字段] 公共基类
    DJI_Measure_s measure;      // DJI 特有：编码器值、RPM、电流、温度
    uint8_t       sender_group; // DJI 特有：CAN 发送分组
    uint8_t       message_num;  // DJI 特有：组内序号 0-3
} DJI_Motor_t;

// DM_Motor_t — 达妙电机
typedef struct {
    Motor_Base               base;    // [必须首字段] 公共基类
    DM_Motor_Measure_s       measure; // 达妙特有：温度、错误码
    uint32_t                 mode_type; // 达妙特有：MIT/POS/SPD/PSI 模式
    const DM_Motor_Params_t *params;  // 达妙特有：PMAX/VMAX/TMAX 参数表
} DM_Motor_t;
```

`base` 必须是第一个字段，这样 `MOTOR_GET_DERIVED(base_ptr, DJI_Motor_t)` 才能正确工作（指针地址相同，直接强转）。

## Motor_Init_Config_s — 统一初始化配置

```c
typedef struct {
    Motor_Controller_Init_s controller_init_config;  // PID/LQR 参数
    Motor_Setting_s         setting_init_config;     // 闭环类型、使能
    Motor_Info_s            motor_init_info;         // 电机类型、减速比、转矩常数
    Offline_Init_config_t   offline_init_config;     // 离线检测
    Motor_Transport_e       transport;               // CAN / UART / PWM
    union {
        Can_Device_Init_Config_s can;
        UART_Device_init_config  uart;
        PWM_Init_Config          pwm;
    } transport_config;  // 传输层配置（联合体，按 transport 选择）
} Motor_Init_Config_s;
```

联合体 `transport_config` 按 `transport` 字段选择对应配置，CAN 电机填 `.can`，UART 电机填 `.uart`。目前所有电机都用 CAN。
