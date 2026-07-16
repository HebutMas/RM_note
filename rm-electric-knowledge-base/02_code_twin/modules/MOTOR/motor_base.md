# motor_base.c/h — 电机基类与多态分发

`modules/MOTOR/motor_base.{c,h}` + `module_motor.{c,h}`

## 入口：`Module_Motor_Init()`

`Module_Motor_Init()` 在 [[03_moc/Robot-Init-Walkthrough#3-module-init]] 中调用，位于 `MODULE_Init()` 阶段。它的前置条件：

- `tx_byte_pool_create()` 已执行 → 后续 `BSP_MEM_ALLOC_WAIT` 有内存池可用
- `BSP_CAN_TaskInit()` 已执行 → CAN 总线已启动，设备注册时 kfifo 和中断就绪
- `Module_Offline_init()` 已执行 → 电机注册时可以调 `Module_Offline_register()`

这个函数只做一件事：创建 motor task 线程：

```c
void Module_Motor_Init(void) {
    tx_thread_create(&motor_thread, "motor", motor_task_entry, 0,
                     motor_thread_stack, MOTOR_TASK_STACK_SIZE,
                     MOTOR_TASK_PRIORITY, MOTOR_TASK_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START);
}
```

线程创建后立即开始运行 `motor_task_entry`，但此时链表 `g_motor_list` 还是空的（还没注册电机），`Motor_UpdateAll` 遍历空链表什么都不做。直到 APP 层调 `robot_control_init()` 注册电机后，链表才有内容。

> **为什么 motor task 可以在电机注册之前启动**：`Motor_UpdateAll` 遍历空链表是安全的（`for (m = NULL; m; m = m->next)` 直接跳过）。线程每 2ms 跑一次空循环，等 APP 层注册电机后自然开始工作。

## 基类结构体：`Motor_Base`

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

    void *transport_dev;              // 传输设备指针（Can_Device*）

    void (*ControlAndSend)(Motor_Base *motor);  // 控制回调函数指针
};
```

| 字段 | 作用 | 谁填充 |
|------|------|--------|
| `next` | 链表指针，`Motor_Register` 头插时设置 | `Motor_Register` |
| `type` | 电机类型（GM6020/M3508/DM4310/...） | 子类 Init |
| `transport` | 传输方式（目前都是 CAN） | 子类 Init |
| `info` | 电机参数，来自 [[01_extracted/motor/motor_params]] | 子类 Init（从 config 复制） |
| `setting` | 闭环类型、使能标志、反转/反馈来源 | 子类 Init + 运行时修改 |
| `controller` | PID/LQR 实例、参考值 ref、输出 output | 子类 Init + 运行时更新 |
| `measure` | 速度/角度/力矩反馈 | CAN 接收回调写入 |
| `offline_dev` | 离线检测设备 | 子类 Init |
| `transport_dev` | CAN 设备指针 | 子类 Init（调 `BSP_CAN_Device_Init`） |
| `ControlAndSend` | 控制+发送回调 | 子类 Init（注册自己的实现） |

## 派生类继承方式

`Motor_Base` 是基类，大疆和达妙各定义自己的派生类，**`base` 必须是第一个字段**：

```c
// motor_dji.h — 大疆电机
typedef struct {
    Motor_Base    base;         // [必须首字段] 公共基类
    DJI_Measure_s measure;      // DJI 特有：编码器值、RPM、电流、温度
    uint8_t       sender_group; // DJI 特有：CAN 发送分组
    uint8_t       message_num;  // DJI 特有：组内序号 0-3
} DJI_Motor_t;

// motor_damiao.h — 达妙电机
typedef struct {
    Motor_Base               base;      // [必须首字段] 公共基类
    DM_Motor_Measure_s       measure;   // 达妙特有：温度、错误码
    uint32_t                 mode_type; // 达妙特有：MIT/POS/SPD/PSI 模式
    const DM_Motor_Params_t *params;    // 达妙特有：PMAX/VMAX/TMAX 参数表
} DM_Motor_t;
```

> `base` 在第一个字段，使得 `&motor->base` 和 `motor` 地址相同。`MOTOR_GET_DERIVED` 宏利用这一点把基类指针强转回派生类指针。原理见 [[01_extracted/algorithm/function-pointer-pattern#基类指针还原派生类]]。

## 子类初始化：注册到链表

大疆电机初始化（详见 [[02_code_twin/modules/MOTOR/DJI/motor_dji]]）：

```c
DJI_Motor_t *Motor_DJI_Init(Motor_Init_Config_s *config) {
    BSP_MEM_ALLOC_WAIT(motor, sizeof(DJI_Motor_t), TX_NO_WAIT);
    memset(motor, 0, sizeof(DJI_Motor_t));

    // 填充基类字段
    motor->base.type      = config->motor_init_info.motor_type;
    motor->base.transport = MOTOR_TRANSPORT_CAN;
    motor->base.info      = config->motor_init_info;
    motor->base.setting   = config->setting_init_config;

    // 注册 CAN 设备（拿到 transport_dev）
    Can_Device *can_dev = BSP_CAN_Device_Init(&config->transport_config.can);
    motor->base.transport_dev = can_dev;
    can_dev->rx_callback = dji_can_rx_callback;
    can_dev->user_arg    = motor;

    // 初始化 PID/LQR、注册离线检测...

    // 关键：注册回调函数 + 加入链表
    motor->base.ControlAndSend = dji_control;
    Motor_Register(&motor->base);
    return motor;
}
```

达妙电机同理（详见 [[02_code_twin/modules/MOTOR/DAMIAO/motor_damiao]]），注册的是 `dm_ControlAndSend`。

这些 Init 函数在 APP 层的 `robot_control_init()` 中被调用（如 `gimbal_init()` 里调 `Motor_DJI_Init` 和 `Motor_DM_Init`）。

## 链表注册：`Motor_Register()`

```c
static Motor_Base *g_motor_list = NULL;

void Motor_Register(Motor_Base *motor) {
    if (motor == NULL) return;
    motor->next = g_motor_list;    // 头插法
    g_motor_list = motor;
}
```

头插法，最新注册的电机在链表头部。链表基础见 [[01_extracted/algorithm/data-structure-linked-list#头插法]]。

## 遍历调用：`Motor_UpdateAll()`

```c
void Motor_UpdateAll(void) {
    for (Motor_Base *m = g_motor_list; m; m = m->next) {
        if (m->ControlAndSend)
            m->ControlAndSend(m);
    }
}
```

遍历链表，对每个电机调用它注册的 `ControlAndSend`。**调用者不需要知道电机类型**：

- 大疆电机 → 走 `dji_control`（详见 [[02_code_twin/modules/MOTOR/DJI/motor_dji]]）：计算 PID/LQR → 力矩转电流 → 填入静态发送缓冲区（不实际发送）
- 达妙电机 → 走 `dm_ControlAndSend`（详见 [[02_code_twin/modules/MOTOR/DAMIAO/motor_damiao]]）：计算 PID/LQR → MIT 模式打包 → 直接 `BSP_CAN_SendMessage` 发送

函数指针多态原理见 [[01_extracted/algorithm/function-pointer-pattern]]。

## 运行时：`motor_task_entry` 2ms 循环

```c
static void motor_task_entry(ULONG thread_input) {
    while (1) {
        Motor_UpdateAll();       // 遍历所有电机，计算控制量
        PowerControl_Update();   // 功率控制（限制总功率）
        Motor_DJI_Flush();       // 大疆电机批量发送 CAN 帧
        tx_thread_sleep(2);     // 2ms 周期
    }
}
```

执行顺序：

1. `Motor_UpdateAll()` — 遍历链表，每个电机调自己的 `ControlAndSend`。大疆只填缓冲区，达妙直接发
2. `PowerControl_Update()` — 功率限制，调整电机输出
3. `Motor_DJI_Flush()` — 大疆电机需要批量发：因为 1 帧 CAN 控制 4 个电机，不能逐个发，必须等所有电机都算完再一次性发。详见 [[02_code_twin/modules/MOTOR/DJI/motor_dji#为什么大疆需要 Flush]]
4. `tx_thread_sleep(2)` — 让出 CPU，2ms 后再次执行（ThreadX tick = 1ms）

> **为什么大疆和达妙发送方式不同**：大疆协议 1 帧 8 字节塞 4 个电机电流值，同一帧 ID 的电机如果逐个发会互相覆盖；达妙协议 1 帧只控制 1 个电机，帧 ID 各不相同，可以直接发。对比详见 [[02_code_twin/modules/MOTOR/DJI/motor_dji]] 和 [[02_code_twin/modules/MOTOR/DAMIAO/motor_damiao]]。

## 前置条件：为什么 APP 层能注册电机

APP 层的 `robot_control_init()` 调用 `gimbal_init()` / `chassis_init()` 等函数，里面调 `Motor_DJI_Init` / `Motor_DM_Init` 注册电机。这之所以能工作，是因为：

| 前置条件 | 在哪里完成 | 链接 |
|---------|-----------|------|
| 内存池可用 | `tx_byte_pool_create()` 在 `Robot_Init()` 最先调用 | [[03_moc/Robot-Init-Walkthrough#1-robot-init]] |
| CAN 总线已启动 | `BSP_CAN_TaskInit()` 在 `BSP_Init()` 中调用 | [[03_moc/Robot-Init-Walkthrough#2-bsp-init]] |
| 离线检测已初始化 | `Module_Offline_init()` 在 `MODULE_Init()` 第一个调用 | [[02_code_twin/modules/OFFLINE/module_offline-c]] |
| motor task 已启动 | `Module_Motor_Init()` 在 `MODULE_Init()` 中调用 | 本文上方 |

调用顺序见 [[03_moc/Robot-Init-Walkthrough#module-init-调用顺序]]。

## `Motor_Init_Config_s` — 统一初始化配置

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

联合体 `transport_config` 按 `transport` 字段选择对应配置。目前所有电机都用 CAN，填 `.transport_config.can`（包含 `hcan`、`tx_id`、`rx_id` 等）。

电机参数（`motor_init_info` 中的转矩常数、减速比、最大力矩）来自 [[01_extracted/motor/motor_params#特征参数]]。
