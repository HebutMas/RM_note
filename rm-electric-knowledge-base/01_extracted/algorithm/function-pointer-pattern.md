# 函数指针与回调模式

> 本文档讲解 C 语言中函数指针的基础语法，以及项目中"基类 + 函数指针 + 链表遍历"的多态实现模式。

---

## 函数指针基础

### 声明与赋值

```c
// 声明一个函数指针类型：参数为 Motor_Base*，返回 void
void (*ControlAndSend)(Motor_Base *motor);

// 赋值：将具体函数的地址赋给函数指针
ControlAndSend = dji_control;    // 大疆电机的控制函数
ControlAndSend = dm_ControlAndSend;  // 达妙电机的控制函数

// 调用：通过函数指针调用函数
ControlAndSend(motor);  // 等价于 dji_control(motor) 或 dm_ControlAndSend(motor)
```

### 为什么用函数指针

C 语言没有 C++ 的虚函数表和继承机制。但通过结构体内嵌函数指针，可以实现类似的多态：

```cpp
// C++ 的做法
class MotorBase {
    virtual void ControlAndSend() = 0;  // 纯虚函数
};
class DJIMotor : public MotorBase {
    void ControlAndSend() override { /* 大疆控制逻辑 */ }
};
```

```c
// C 语言的做法：结构体内嵌函数指针
struct Motor_Base {
    void (*ControlAndSend)(Motor_Base *motor);  // "虚函数"
};
// 大疆电机初始化时把自己的函数注册进去
motor->base.ControlAndSend = dji_control;
```

## 项目中的回调模式

### 基类定义函数指针

```c
// motor_base.h
struct Motor_Base {
    Motor_Base       *next;           // 链表指针
    Motor_Type_e      type;           // 电机类型
    Motor_Info_s      info;           // 电机参数
    Motor_Controller_s controller;    // 控制器（PID/LQR）
    Motor_Measure_s   measure;        // 测量数据
    Offline_Device    *offline_dev;   // 离线检测
    void *transport_dev;              // 传输设备（CAN/UART/PWM）

    void (*ControlAndSend)(Motor_Base *motor);  // 回调函数指针
};
```

`ControlAndSend` 是核心回调：每个电机注册自己的控制+发送实现，基类不需要知道具体是哪种电机。

### 子类注册自己的实现

```c
// motor_dji.c — 大疆电机初始化
DJI_Motor_t *Motor_DJI_Init(Motor_Init_Config_s *config) {
    // ... 分配内存、初始化字段 ...
    motor->base.ControlAndSend = dji_control;   // 注册大疆控制函数
    Motor_Register(&motor->base);               // 加入全局链表
}

// motor_damiao.c — 达妙电机初始化
DM_Motor_t *Motor_DM_Init(Motor_Init_Config_s *config, uint32_t mode) {
    // ... 分配内存、初始化字段 ...
    motor->base.ControlAndSend = dm_ControlAndSend;  // 注册达妙控制函数
    Motor_Register(&motor->base);
}
```

### 遍历调用（多态分发）

```c
// motor_base.c
static Motor_Base *g_motor_list = NULL;

void Motor_Register(Motor_Base *motor) {
    motor->next = g_motor_list;    // 头插法
    g_motor_list = motor;
}

void Motor_UpdateAll(void) {
    for (Motor_Base *m = g_motor_list; m; m = m->next) {
        if (m->ControlAndSend)
            m->ControlAndSend(m);  // 多态调用：大疆走 dji_control，达妙走 dm_ControlAndSend
    }
}
```

`Motor_UpdateAll` 遍历链表，对每个电机调用它自己注册的 `ControlAndSend`。调用者不需要知道电机类型，函数指针自动分发到正确的实现。

链表头插法详见 [[01_extracted/algorithm/data-structure-linked-list#头插法]]。

## MOTOR_GET_DERIVED 宏：基类指针还原派生类

### 问题

`Motor_UpdateAll` 遍历时拿到的是 `Motor_Base *` 指针。但 `dji_control` 需要访问 `DJI_Motor_t` 的特有字段（如 `sender_group`、`measure.ecd`），需要从基类指针还原出派生类指针。

### 解决

```c
// motor_base.h
#define MOTOR_GET_DERIVED(base_ptr, derived_type) ((derived_type *)(base_ptr))
```

这个宏依赖一个 C 语言特性：**派生类的第一个字段必须是基类**（即 `base` 是 `DJI_Motor_t` 的首字段），这样基类指针和派生类指针的地址相同，可以直接强转。

```c
// DJI_Motor_t 的结构
typedef struct {
    Motor_Base    base;         // [必须首字段] 公共基类
    DJI_Measure_s measure;      // DJI 特有数据
    uint8_t       sender_group; // DJI 特有字段
    uint8_t       message_num;
} DJI_Motor_t;
```

```c
// 使用
static void dji_control(Motor_Base *base) {
    DJI_Motor_t *motor = MOTOR_GET_DERIVED(base, DJI_Motor_t);
    // 现在可以访问 motor->sender_group, motor->measure.ecd 等
}
```

> 这是 C 语言实现继承的经典手法：结构体首字段为基类，指针强转即可在基类和派生类之间切换。C++ 编译器自动处理这个偏移，C 语言需要手动保证 `base` 是第一个字段。

## 其他回调函数指针

项目中还使用了类似的回调模式：

| 模块 | 函数指针 | 用途 |
|------|---------|------|
| CAN BSP | `can_dev->rx_callback` | CAN 接收到数据时回调，电机/遥控器各注册自己的解析函数 |
| CAN BSP | `can_dev->user_arg` | 回调时的上下文指针（指向电机实例） |
| Motor | `ControlAndSend` | 电机控制+发送，大疆/达妙各注册自己的实现 |
| PWM BSP | `event_flags` | DMA 完成事件通知（非函数指针，但同属回调机制） |

CAN 接收回调示例：

```c
// 大疆电机初始化时注册
can_dev->rx_callback = dji_can_rx_callback;
can_dev->user_arg = motor;  // 回调时拿回 motor 指针

// CAN RX Task 中触发回调
if (dev->rx_callback)
    dev->rx_callback(dev, data, len);  // 调用 dji_can_rx_callback
```

## 与观察者模式的区别

- **回调函数指针**：一对一关系，一个事件触发一个回调
- **观察者模式**：一对多关系，一个事件通知多个订阅者（项目中 Offline 模块的链表遍历更接近观察者模式）

> 观察者订阅模式详见参考笔记：4.25 学习笔记中提到的"哨兵代码"设计模式。
