# 函数指针与回调模式

> 本文档讲解 C 语言中函数指针的基础语法，以及"基类 + 函数指针 + 链表遍历"的多态实现模式。不绑定具体模块。

---

## 函数指针基础

### 声明与赋值

```c
// 声明一个函数指针：参数为 Device*，返回 void
void (*handler)(Device *dev);

// 赋值：将具体函数的地址赋给函数指针
handler = uart_handler;    // UART 设备的处理函数
handler = spi_handler;     // SPI 设备的处理函数

// 调用：通过函数指针调用函数
handler(dev);  // 等价于 uart_handler(dev) 或 spi_handler(dev)
```

### 为什么用函数指针

C 语言没有 C++ 的虚函数表和继承机制。但通过结构体内嵌函数指针，可以实现类似的多态：

```cpp
// C++ 的做法
class Device {
    virtual void handler() = 0;  // 纯虚函数
};
class UARTDevice : public Device {
    void handler() override { /* UART 处理逻辑 */ }
};
```

```c
// C 语言的做法：结构体内嵌函数指针
struct Device {
    void (*handler)(Device *dev);  // "虚函数"
};
// UART 设备初始化时把自己的函数注册进去
dev->handler = uart_handler;
```

## 回调模式

### 基类定义函数指针

```c
struct Device {
    Device *next;           // 链表指针
    uint8_t type;           // 设备类型
    void  *config;          // 设备配置
    void (*handler)(Device *dev);  // 回调函数指针
};
```

`handler` 是核心回调：每个设备注册自己的处理实现，基类不需要知道具体是哪种设备。

### 子类注册自己的实现

```c
// UART 设备初始化
UART_Device *UART_Device_Init(Config *cfg) {
    // ... 分配内存、初始化字段 ...
    dev->base.handler = uart_handler;    // 注册 UART 处理函数
    Device_Register(&dev->base);         // 加入全局链表
}

// SPI 设备初始化
SPI_Device *SPI_Device_Init(Config *cfg) {
    // ... 分配内存、初始化字段 ...
    dev->base.handler = spi_handler;     // 注册 SPI 处理函数
    Device_Register(&dev->base);
}
```

### 遍历调用（多态分发）

```c
static Device *g_device_list = NULL;

void Device_Register(Device *dev) {
    dev->next = g_device_list;    // 头插法
    g_device_list = dev;
}

void Device_UpdateAll(void) {
    for (Device *d = g_device_list; d; d = d->next) {
        if (d->handler)
            d->handler(d);  // 多态调用：UART 走 uart_handler，SPI 走 spi_handler
    }
}
```

`Device_UpdateAll` 遍历链表，对每个设备调用它自己注册的 `handler`。调用者不需要知道设备类型，函数指针自动分发到正确的实现。

链表头插法详见 [[01_extracted/algorithm/data-structure-linked-list#头插法]]。

## 基类指针还原派生类

### 问题

`Device_UpdateAll` 遍历时拿到的是 `Device *` 指针。但 `uart_handler` 需要访问 `UART_Device` 的特有字段（如波特率、缓冲区），需要从基类指针还原出派生类指针。

### 解决

```c
#define GET_DERIVED(base_ptr, derived_type) ((derived_type *)(base_ptr))
```

这个宏依赖一个 C 语言特性：**派生类的第一个字段必须是基类**（即 `base` 是 `UART_Device` 的首字段），这样基类指针和派生类指针的地址相同，可以直接强转。

```c
// UART_Device 的结构
typedef struct {
    Device base;           // [必须首字段] 公共基类
    uint32_t baudrate;     // UART 特有
    uint8_t *rx_buf;       // UART 特有
} UART_Device;
```

```c
// 使用
static void uart_handler(Device *base) {
    UART_Device *dev = GET_DERIVED(base, UART_Device);
    // 现在可以访问 dev->baudrate, dev->rx_buf 等
}
```

> 这是 C 语言实现继承的经典手法：结构体首字段为基类，指针强转即可在基类和派生类之间切换。C++ 编译器自动处理这个偏移，C 语言需要手动保证 `base` 是第一个字段。

## 项目中的实际应用

这套"基类 + 函数指针 + 链表遍历 + GET_DERIVED"模式在项目多个模块中使用：

| 模块 | 基类 | 回调函数指针 | 用途 |
|------|------|------------|------|
| Motor | `Motor_Base` | `ControlAndSend` | 电机控制+发送，大疆/达妙各注册实现 |
| CAN BSP | `Can_Device` | `rx_callback` | CAN 接收回调，各设备注册解析函数 |
| Offline | `Offline_Device` | — | 离线检测链表遍历（无回调，直接查链表） |

具体实现见各模块的 02 层 code twin 文档。
