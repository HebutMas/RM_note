# 函数指针与回调模式

> 本文档以 `motor_dji.c` 的真实代码为例，讲解 C 语言中"结构体内嵌函数指针 + 链表遍历"的多态实现。

---

## 函数指针基础

### 声明

```c
// 声明一个函数指针变量：参数为 Motor_Base*，返回 void
void (*ControlAndSend)(Motor_Base *motor);

// 声明一个函数指针类型（用 typedef）
typedef void (*ControlAndSend_t)(Motor_Base *motor);
```

`(*ControlAndSend)` 的括号不能省——去掉就变成返回 `void*` 的函数声明了。

### 赋值与调用

```c
// 赋值：把函数地址赋给函数指针
ControlAndSend = dji_control;         // 大疆电机的控制函数
ControlAndSend = dm_ControlAndSend;   // 达妙电机的控制函数

// 调用：通过函数指针间接调用
ControlAndSend(motor);  // 等价于 dji_control(motor)
```

函数名就是函数的地址，`dji_control` 和 `&dji_control` 等价。

## 结构体内嵌函数指针：C 语言的多态

### C++ 的做法

C++ 用虚函数实现多态，编译器自动维护虚函数表：

```cpp
class MotorBase {
    virtual void ControlAndSend() = 0;  // 纯虚函数
};
class DJIMotor : public MotorBase {
    void ControlAndSend() override { /* 大疆控制逻辑 */ }
};
class DMMotor : public MotorBase {
    void ControlAndSend() override { /* 达妙控制逻辑 */ }
};
```

### C 语言的做法

C 没有继承语法，用**结构体首字段嵌套 + 函数指针**模拟：

```c
// motor_base.h — 基类
struct Motor_Base {
    Motor_Base       *next;           // 链表指针
    Motor_Type_e      type;           // 电机类型
    Motor_Info_s      info;           // 电机参数
    Motor_Controller_s controller;    // 控制器
    Motor_Measure_s   measure;        // 测量数据
    Offline_Device    *offline_dev;   // 离线检测
    void *transport_dev;              // 传输设备

    void (*ControlAndSend)(Motor_Base *motor);  // 相当于"纯虚函数"
};
```

`ControlAndSend` 是个函数指针，基类不实现它，留给子类注册。

## 大疆电机的实际代码

### 子类注册自己的实现

`motor_dji.c` 初始化时把 `dji_control` 注册进去：

```c
// motor_dji.c
DJI_Motor_t *Motor_DJI_Init(Motor_Init_Config_s *config) {
    DJI_Motor_t *motor = NULL;
    BSP_MEM_ALLOC_WAIT(motor, sizeof(DJI_Motor_t), TX_NO_WAIT);
    memset(motor, 0, sizeof(DJI_Motor_t));

    // 初始化基类字段
    motor->base.type      = config->motor_init_info.motor_type;
    motor->base.transport = MOTOR_TRANSPORT_CAN;
    motor->base.info      = config->motor_init_info;
    motor->base.setting   = config->setting_init_config;

    // ... 注册 CAN 设备、初始化 PID/LQR、注册离线检测 ...

    // 关键：把自己的控制函数注册到基类的函数指针
    motor->base.ControlAndSend = dji_control;

    // 注册到全局链表
    Motor_Register(&motor->base);
    return motor;
}
```

达妙电机同理，注册的是 `dm_ControlAndSend`：

```c
// motor_damiao.c
DM_Motor_t *Motor_DM_Init(Motor_Init_Config_s *config, uint32_t DM_Mode_type) {
    // ... 同样的初始化流程 ...
    motor->base.ControlAndSend = dm_ControlAndSend;  // 注册达妙控制函数
    Motor_Register(&motor->base);
    return motor;
}
```

### 遍历调用（多态分发）

```c
// motor_base.c
static Motor_Base *g_motor_list = NULL;

void Motor_Register(Motor_Base *motor) {
    if (motor == NULL) return;
    motor->next = g_motor_list;    // 头插法
    g_motor_list = motor;
}

void Motor_UpdateAll(void) {
    for (Motor_Base *m = g_motor_list; m; m = m->next) {
        if (m->ControlAndSend)
            m->ControlAndSend(m);  // 多态：大疆走 dji_control，达妙走 dm_ControlAndSend
    }
}
```

`Motor_UpdateAll` 遍历链表，对每个电机调用它自己注册的 `ControlAndSend`。**调用者完全不需要知道电机类型**——函数指针自动分发到正确的实现。这就是 C 语言的运行时多态。

链表头插法详见 [[01_extracted/algorithm/data-structure-linked-list#头插法]]。

## 基类指针还原派生类

### 问题

`Motor_UpdateAll` 拿到的是 `Motor_Base *` 指针。但 `dji_control` 需要访问 `DJI_Motor_t` 的特有字段：

```c
typedef struct {
    Motor_Base    base;         // 第一个字段
    DJI_Measure_s measure;      // DJI 特有：编码器值、RPM、温度
    uint8_t       sender_group; // DJI 特有：CAN 发送分组
    uint8_t       message_num;  // DJI 特有：组内序号
} DJI_Motor_t;
```

### 解决：MOTOR_GET_DERIVED 宏

```c
// motor_base.h
#define MOTOR_GET_DERIVED(base_ptr, derived_type) ((derived_type *)(base_ptr))
```

就是个强制类型转换。能直接转是因为 C 语言保证**结构体首字段地址 = 结构体地址**：

```c
// motor_dji.c 中使用
static void dji_control(Motor_Base *base) {
    DJI_Motor_t *motor = MOTOR_GET_DERIVED(base, DJI_Motor_t);

    // 现在可以访问派生类特有字段
    uint8_t group = motor->sender_group;
    uint8_t num   = motor->message_num;
    int16_t ecd   = motor->measure.ecd;
    // ...
}
```

> `base` 是 `DJI_Motor_t` 的第一个字段，所以 `&motor->base` 和 `motor` 指向同一地址，强转不需要偏移。C++ 编译器自动处理这个偏移，C 语言靠手动保证首字段位置。

## 同一套模式的其他应用

项目中多处使用相同的"结构体内嵌函数指针"模式：

| 模块 | 结构体 | 函数指针 | 用途 |
|------|--------|---------|------|
| Motor | `Motor_Base` | `ControlAndSend` | 电机控制+发送，大疆/达妙各注册实现 |
| CAN BSP | `Can_Device` | `rx_callback` | CAN 接收数据时回调，电机/遥控器各注册解析函数 |

CAN 接收回调示例——大疆电机初始化时注册：

```c
// motor_dji.c
can_dev->rx_callback = dji_can_rx_callback;  // 注册接收回调
can_dev->user_arg = motor;                    // 回调时拿回 motor 指针

// CAN RX Task 中触发
if (dev->rx_callback)
    dev->rx_callback(dev, data, len);  // 调用 dji_can_rx_callback
```

`user_arg` 是回调上下文：注册时存入 `motor` 指针，回调时取回，这样 `dji_can_rx_callback` 知道该把数据写进哪个电机实例。

具体实现见 [[02_code_twin/modules/MOTOR/motor_base]] 和 [[02_code_twin/board/bsp/CAN/bsp_can]]。
