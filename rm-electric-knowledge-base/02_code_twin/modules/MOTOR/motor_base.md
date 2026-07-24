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

> `base` 在第一个字段，使得 `&motor->base` 和 `motor` 地址相同。`MOTOR_GET_DERIVED` 宏利用这一点把基类指针强转回派生类指针。原理见 [[01_extracted/algorithm/函数指针#基类指针还原派生类]]。

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

函数指针多态原理见 [[01_extracted/algorithm/函数指针]]。

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
## 坐标系与取反机制

> 本节是电机反转逻辑的**权威来源**。应用层各场景（云台、底盘）遇到反转问题时，应链接回本节。本节不涉及具体场景。
>
> 外部反馈源实际方向可以参考
>  [[01_extracted/hardware/bmi088-orientation]]（BMI088 物理轴）和 [[02_code_twin/modules/INS/module_ins]]（INS 导航系）。

### 为什么需要取反

从传感器读数到电机输出，中间经过多层坐标系，每一层都可能出现方向不一致：

    ↓ 物理安装方向
传感器坐标系 (BMI088 gyro/acc)
    ↓ feedback_reverse_flag (测量端取反)
电机轴坐标系
    ↓ motor_reverse_flag (参考端取反)
控制目标坐标系 (遥控器/**视觉**输入的坐标系,底盘要求的输入坐标系)

两个取反标志解决不同层面的方向问题，**独立配置、互不干扰**。

### `feedback_reverse_flag` — 测量端取反

**作用**：把外部传感器反馈转到电机轴方向,形成负反馈。

**代码位置**：`CalculateLQROutput()` 中段，读取 `measure` 后判断：

```c
// 伪代码示意
rad_speed = motor->base.measure.speed_rad;
if (feedback_reverse_flag == 1)
    rad_speed *= -1;  // 反馈取反
```

**适用场景**：电机使用外部传感器（如 INS、BMI088）做反馈时，传感器的正方向与电机正方向不一致。

**典型例子**：pitch 电机的 `feedback_reverse_flag = 1`。INS 的 pitch 正方向是"低头"，但 DM4310 电机正方向(和电机安装方向有关)是"抬头"——如果不取反，反馈会变成正反馈，系统发散。具体场景见云台控制模块。

### `motor_reverse_flag` — 参考端取反

**作用**：把已经对齐到电机轴的控制目标（ref）进一步对齐到控制目标坐标系。

**代码位置**：`CalculateLQROutput()` 开头，读取 `ref` 后判断：

```c
// 伪代码示意
ref = motor->base.controller.ref;
if (motor_reverse_flag == 1)
    ref *= -1;  // 参考取反
```

**适用场景**：电机物理安装方向与运动学/控制目标坐标系相反。常见于对称安装的电机（如麦轮底盘左右两侧）。

**典型例子**：
1.上个例子，pitch轴的电机坐标系向右,也就是说pitch的正方向是   **抬头** ，而视觉的坐标系是前x， **左y** ，上z。pitch正方向是低头，所以需要对于ref设定取反。将电机坐标系反转到控制目标坐标系。
2.底盘右侧两个 M3508 的 `motor_reverse_flag = 1`。麦轮底盘左右电机安装方向相反，运动学算出的 wheel_speed 是"底盘坐标系下的轮速"，右侧电机需要取反才能让轮子实际转对方向。具体场景见底盘控制模块。

### 两者的区别

| 对比项 | `feedback_reverse_flag` | `motor_reverse_flag` |
|--------|------------------------|---------------------|
| 改的是 | **measure**（传感器反馈值） | **ref**（控制目标值） |
| 作用环节 | 传感器系 → 电机系 | 电机系 → 控制目标系 |
| 触发原因 | 传感器方向与电机方向相反 | 电机安装方向与坐标系相反 |
| 底盘典型值 | 全 0（用电机自身编码器） | `[0,0,1,1]`（左右对称） |
| 云台典型值 | pitch=1（INS 方向与电机相反） | 全 0（上层已处理符号） |

### 完整取反链路

以云台 pitch 电机为例，一个控制周期内完整的数据流：

```
INS 输出 euler_rad[1] (pitch 角度)
    │
    ├── 读入 LQR 作为 other_angle_feedback_ptr
    │
    ├── feedback_reverse_flag == 1 ?
    │     └─ YES → measure = -euler_rad[1]   ← 测量取反，掰到电机轴
    │
    ├── 读入 other_speed_feedback_ptr = bmi088_dev->gyro[0]
    │     └─ 同上，feedback_reverse_flag 同时影响角度和速度反馈
    │
    ├── CalculateLQROutput:
    │     ├── ref = controller.ref
    │     ├── motor_reverse_flag == 1 ? (pitch 这里不取反)
    │     ├── output = -K * (measure - ref)   ← LQR 计算
    │     └── output → torque → CAN 报文
```

### 关于 `angle_feedback_source` 和 `speed_feedback_source`

除了两个取反 flag，还有两个选择反馈来源的配置：

| 字段 | 0 | 1 |
|------|---|----|
| `angle_feedback_source` | 电机自身编码器 | INS 欧拉角 (`euler_rad`) |
| `speed_feedback_source` | 电机自身转速 | BMI088 陀螺仪 (`gyro`) |

当 `feedback_source = 1` 时，使用外部传感器做反馈，此时 `feedback_reverse_flag` 才有意义（自身编码器方向天然与电机一致，不需要取反）。

### 与 INS/BMI088 的关系

- `feedback_reverse_flag` 涉及的传感器反馈来自 BMI088/INS，详见 [[02_code_twin/modules/INS/module_ins]] 和 [[01_extracted/hardware/bmi088-orientation]]
- 电机自身编码器不存在方向问题，不需要取反
- 本文（motor_base）只定义取反机制，**各应用层具体场景**（哪个电机为什么需要取反）在各自文件中说明