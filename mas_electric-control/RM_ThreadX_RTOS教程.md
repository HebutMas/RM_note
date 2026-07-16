# ThreadX RTOS 使用教程

## 目录

1. [ThreadX 简介](#1-threadx-简介)
2. [核心概念](#2-核心概念)
3. [线程管理](#3-线程管理)
4. [同步与通信机制](#4-同步与通信机制)
5. [内存管理](#5-内存管理)
6. [定时器](#6-定时器)
7. [中断处理](#7-中断处理)
8. [本项目中的 ThreadX 配置](#8-本项目中的-threadx-配置)
9. [实际案例分析](#9-实际案例分析)
10. [ThreadX 在 Cortex-M 上的移植要点](#10-threadx-在-cortex-m-上的移植要点)

---

## 1. ThreadX 简介

### 1.1 什么是 ThreadX

ThreadX 是由 Microsoft（原 Express Logic）开发的**硬实时操作系统内核**（RTOS），专为嵌入式应用设计。它是一个**抢占式多线程**内核，具有极小的代码体积和确定性的执行时间。

### 1.2 核心特性

| 特性 | 说明 |
|------|------|
| 抢占式调度 | 高优先级线程可立即抢占低优先级线程 |
| 优先级级联 | 支持 32–1024 个优先级 |
| 抢占阈值 | 防止同优先级组内不必要的抢占 |
| 优先级继承 | 自动解决优先级反转问题 |
| 零中断延迟 | 临界区极短，中断几乎零延迟 |
| 时间片轮转 | 同优先级线程可公平分享 CPU |
| 微内核设计 | picokernel 架构，代码极小 |

### 1.3 基本目录结构

```
threadx/
├── common/                    # 通用内核源码
│   ├── tx_initialize_low_level.S  ← 汇编：启动、SysTick、向量表
│   ├── tx_thread_create.c         ← 线程创建
│   ├── tx_thread_schedule.c       ← 调度器
│   ├── tx_mutex_create.c          ← 互斥量
│   ├── tx_semaphore_create.c      ← 信号量
│   ├── tx_queue_create.c          ← 消息队列
│   ├── tx_event_flags_create.c    ← 事件标志
│   ├── tx_timer_create.c          ← 定时器
│   └── tx_byte_pool_create.c      ← 字节池
├── ports/
│   ├── cortex_m4/                 ← Cortex-M4 移植
│   └── cortex_m7/                 ← Cortex-M7 移植
│       ├── inc/tx_port.h          ← 移植宏和类型定义
│       └── src/                   ← 架构特定汇编
└── tx_user.h                      ← 用户配置文件
```

---

## 2. 核心概念

### 2.1 线程（Thread）

线程是 ThreadX 中的基本执行单元。每个线程有自己的：
- **栈空间**（存储局部变量、调用链）
- **优先级**（0 = 最高，31 = 最低，本项目配置为 32 级）
- **抢占阈值**（低于此优先级的线程不能抢占当前线程）
- **时间片**（同优先级线程轮流执行的滴答数）

### 2.2 调度策略

ThreadX 使用**位图优先级调度**：
1. 内核维护一个优先级就绪位图（32 位或更多）
2. 使用 `RBIT` + `CLZ` 指令（ARM 硬件指令）在**一个周期内**找到最高优先级的就绪线程
3. 总是运行优先级最高的就绪线程

```
高优先级 0  ──→ Thread A（运行中）
优先级 5   ──→ Thread B（就绪）
优先级 10  ──→ Thread C（就绪）
优先级 30  ──→ Thread D（挂起，等待信号量）
```

### 2.3 线程状态转换

```
                  +---------+
         +------→ |  EXECUTING | ←------+
         |        +---------+          |
         |           |    ↑             |
    (被抢占)    (阻塞API) | (信号到来)  |
         |           ↓    |             |
         |     +---------+ |      +---------+
         +-----| SUSPENDED|-+     |  READY   |
               +---------+  ←----+---------+
                              (调度器选择)
```

### 2.4 系统滴答（System Tick）

- 配置为 **1000 Hz**（`TX_TIMER_TICKS_PER_SECOND = 1000`）
- 即每 **1 毫秒**产生一次 SysTick 中断
- 所有时间相关的 API（`tx_thread_sleep(N)` 中的 `N`）的单位是 tick

```c
tx_thread_sleep(2);    // 休眠 2 毫秒 → 500 Hz 循环
tx_thread_sleep(1);    // 休眠 1 毫秒 → 1000 Hz 循环
tx_thread_sleep(10);   // 休眠 10 毫秒 → 100 Hz 循环
```

---

## 3. 线程管理

### 3.1 创建线程

```c
UINT tx_thread_create(
    TX_THREAD *thread_ptr,       // 线程控制块指针
    CHAR *name_ptr,              // 线程名称（用于调试）
    VOID (*entry_function)(ULONG), // 入口函数
    ULONG entry_input,           // 入口参数
    VOID *stack_start,           // 栈起始地址
    ULONG stack_size,            // 栈大小（字节）
    UINT priority,               // 优先级（0 最高）
    UINT preempt_threshold,      // 抢占阈值
    ULONG time_slice,            // 时间片（TX_NO_TIME_SLICE = 无限）
    UINT auto_start              // TX_AUTO_START 或 TX_DONT_START
);
```

**本项目的典型用法：**

```c
static TX_THREAD                  robot_control_thread;
APPS_STACK_SECTION static uint8_t robot_control_thread_stack[1024];

tx_thread_create(
    &robot_control_thread,           // 控制块
    "Robot Control",                 // 名称
    robot_control_task,              // 入口函数
    0,                               // 输入参数
    robot_control_thread_stack,      // 栈
    1024,                            // 栈大小
    30,                              // 优先级（最低）
    30,                              // 抢占阈值 = 优先级（不启用抢占阈值）
    TX_NO_TIME_SLICE,                // 无限时间片
    TX_AUTO_START                    // 立即启动
);
```

### 3.2 栈分配策略

**静态分配**（本项目采用）：

```c
// 栈放在 DTCM（H7）或 CCMRAM（F4）中——零等待访问
APPS_STACK_SECTION static uint8_t my_thread_stack[1024];
```

**动态分配**（通过字节池）：

```c
UCHAR *stack_ptr;
tx_byte_allocate(&byte_pool, (VOID**)&stack_ptr, 1024, TX_NO_WAIT);
tx_thread_create(&thread, "name", entry, 0, stack_ptr, 1024, 5, 5,
                 TX_NO_TIME_SLICE, TX_AUTO_START);
```

### 3.3 线程控制 API

```c
// 休眠当前线程（仅在当前线程中调用）
UINT tx_thread_sleep(ULONG timer_ticks);

// 挂起/恢复线程
UINT tx_thread_suspend(TX_THREAD *thread_ptr);
UINT tx_thread_resume(TX_THREAD *thread_ptr);

// 终止线程
UINT tx_thread_terminate(TX_THREAD *thread_ptr);

// 获取当前线程指针
TX_THREAD *tx_thread_identify(void);  // 返回 NULL 表示在 ISR 中

// 优先级变更
UINT tx_thread_priority_change(TX_THREAD *thread_ptr, UINT new_priority, UINT *old_priority);

// 获取线程信息
UINT tx_thread_info_get(TX_THREAD *thread_ptr, CHAR **name, UINT *state,
    ULONG *run_count, UINT *priority, ...);
```

### 3.4 线程模式总结

本项目中所有线程都采用相同的模式：

```c
static void my_task(ULONG thread_input)
{
    while (1)
    {
        // 1. 等待事件（信号量 / 事件标志 / 队列 / 定时睡眠）
        tx_semaphore_get(&sem, TX_WAIT_FOREVER);

        // 2. 处理数据
        process_data();

        // 3. （可选）发送/发布结果
    }
}
```

关键点：
- 所有栈统一为 **1024 字节**（除 CherryUSB 为 2048 字节）
- 所有线程创建时带有 `TX_AUTO_START` 标志
- 不使用时间片（`TX_NO_TIME_SLICE`）
- 抢占阈值 = 优先级（不启用抢占阈值特性）

---

## 4. 同步与通信机制

### 4.1 信号量（Semaphore）—— 计数型同步

**适合场景**：ISR→任务通知、资源计数、生产者-消费者模型

```c
// 创建（初始计数 = 0）
TX_SEMAPHORE sem;
tx_semaphore_create(&sem, "my_sem", 0);

// 在任务中等待
tx_semaphore_get(&sem, TX_WAIT_FOREVER);  // 阻塞直到信号量可用

// 在 ISR 中释放
tx_semaphore_put(&sem);  // ISR 安全
```

**本项目使用场景**：CAN 接收/发送任务

```
FDCAN ISR → kfifo_put(msg) → tx_semaphore_put(&g_can_rx_sem)
                                               ↓
can_rx_task → tx_semaphore_get(&g_can_rx_sem, TX_WAIT_FOREVER)
           → kfifo_get(msg) → 处理
```

### 4.2 互斥量（Mutex）—— 互斥访问 + 优先级继承

**适合场景**：保护共享资源（外设总线、全局数据）

```c
// 创建（带优先级继承）
TX_MUTEX bus_mutex;
tx_mutex_create(&bus_mutex, "spi_mtx", TX_INHERIT);

// 在任务中获取（不能在 ISR 中调用！）
tx_mutex_get(&bus_mutex, TX_WAIT_FOREVER);
// ... 临界区 ...
tx_mutex_put(&bus_mutex);
```

**ISR 安全检查模式**（本项目 SPI/I2C 驱动中使用）：

```c
if (tx_thread_identify() != NULL)  // 当前不在 ISR 中
{
    tx_mutex_get(&bus->bus_mutex, TX_WAIT_FOREVER);
}
// 执行传输（无论是否在 ISR 中）
// ...
if (tx_thread_identify() != NULL)
{
    tx_mutex_put(&bus->bus_mutex);
}
```

### 4.3 事件标志（Event Flags）—— 多事件同步

**适合场景**：等待多个独立事件中的任意一个或多个

```c
// 创建
TX_EVENT_FLAGS_GROUP event_flags;
tx_event_flags_create(&event_flags, "uart_evt");

// 在 ISR 中设置事件
tx_event_flags_set(&event_flags, 0x01, TX_OR);  // ISR 安全

// 在任务中等待
ULONG actual_flags;
tx_event_flags_get(&event_flags, 0x01 | 0x02, TX_OR_CLEAR,
                   &actual_flags, TX_WAIT_FOREVER);
```

**重要**：`TX_OR` vs `TX_AND`：
- `TX_OR`：任一事件触发即唤醒
- `TX_AND`：所有事件都触发才唤醒

**本项目使用场景**：UART / SPI / I2C / PWM / USB 的 DMA/中断完成通知

```c
// UART 驱动中：
#define BSP_UART_EVENT_RX ((ULONG)0x01)
#define BSP_UART_EVENT_TX ((ULONG)0x02)

// HAL 回调中：
tx_event_flags_set(&dev->event_flags, BSP_UART_EVENT_RX, TX_OR);

// 任务中：
tx_event_flags_get(&dev->event_flags, BSP_UART_EVENT_RX,
                   TX_OR_CLEAR, &actual, timeout);
```

### 4.4 消息队列（Queue）—— 线程间数据传递

**适合场景**：线程间传递数据（比全局缓冲区 + 信号量更安全）

```c
// 创建（每个消息 = sizeof(ULONG) 对齐）
TX_QUEUE queue;
tx_queue_create(&queue, "my_queue", TX_1_ULONG,  // 每消息 1 个 ULONG
                queue_buffer, sizeof(queue_buffer));

// 发送
ULONG msg = 42;
tx_queue_send(&queue, &msg, TX_NO_WAIT);  // 可在 ISR 中调用

// 接收
ULONG received;
tx_queue_receive(&queue, &received, TX_WAIT_FOREVER);
```

**本项目使用**：CherryUSB OSAL 线程删除队列

### 4.5 同步机制对比

| 机制 | 方向 | ISR 安全 | 数据携带 | 本项目使用场景 |
|------|------|---------|---------|-------------|
| 信号量 | ISR→Task | ✅ `put` | 计数 | CAN 消息通知 |
| 互斥量 | Task→Task | ❌ | 无 | SPI/I2C 总线保护 |
| 事件标志 | ISR→Task | ✅ `set` | 位掩码 | UART/SPI/I2C/PWM 完成 |
| 消息队列 | Task↔Task | ✅ `send` | 数据 | USB 线程管理 |

> 💡 **一个容易忽视的差异——丢一次通知会怎样**：信号量是**计数**的，中途丢一次 `put`（如硬件事件被打断、回调没触发）计数会永久少 1，最终死锁，因此需要在异常事件里主动补 `put`；事件标志是**位状态**，丢一次 `set` 无影响，下次 `set` 即恢复就绪。这个区别在 USB 总线复位的自愈行为上表现得很典型，实战案例见 [[RM_USB_CDC虚拟串口调试实录]] §3。

---

## 5. 内存管理

### 5.1 字节池（Byte Pool）

ThreadX 的字节池是一种**可变大小**的内存分配器，类似 `malloc()`，但具有确定性行为。

```c
// 创建 20KB 字节池
#define TX_APP_MEM_POOL_SIZE (20 * 1024)
static UCHAR tx_byte_pool_buffer[TX_APP_MEM_POOL_SIZE];
TX_BYTE_POOL tx_app_byte_pool;

tx_byte_pool_create(&tx_app_byte_pool, "Tx App memory pool",
                    tx_byte_pool_buffer, TX_APP_MEM_POOL_SIZE);

// 分配内存
VOID *ptr;
UINT status = tx_byte_allocate(&tx_app_byte_pool, &ptr, size, wait_option);

// 释放内存
tx_byte_release(ptr);
```

**重要**：字节池分配采用**首次适应（first-fit）** 算法。需要 `TX_BYTE_POOL` 的大小至少为池总大小的约 1/8 用于管理开销。

### 5.2 块池（Block Pool）—— 固定大小内存块

```c
TX_BLOCK_POOL pool;
tx_block_pool_create(&pool, "my_pool", block_size,
                     pool_buffer, pool_size);
tx_block_allocate(&pool, &ptr, TX_WAIT_FOREVER);
tx_block_release(ptr);
```

本项目未使用块池，但了解它与字节池的区别很重要：块池更快（O(1)），但没有内部碎片。

### 5.3 本项目的内存分配宏

```c
// bsp_def.h 中的包装宏
#define BSP_MEM_ALLOC_WAIT(ptr, size, wait_option)
#define BSP_MEM_FREE(ptr)

// 使用示例
Can_Device *dev;
BSP_MEM_ALLOC_WAIT(dev, sizeof(Can_Device), TX_WAIT_FOREVER);
// ...
BSP_MEM_FREE(dev);  // 自动将 dev 置 NULL
```

---

## 6. 定时器

### 6.1 应用定时器

```c
// 创建定时器
TX_TIMER my_timer;
tx_timer_create(&my_timer, "my_timer", my_timer_callback, 0,
                 initial_ticks, reschedule_ticks, TX_AUTO_ACTIVATE);

// 修改定时器
tx_timer_change(&my_timer, new_initial, new_reschedule);

// 停用/激活
tx_timer_deactivate(&my_timer);
tx_timer_activate(&my_timer);
```

### 6.2 本项目配置：ISR 内处理定时器

本项目定义了 `TX_TIMER_PROCESS_IN_ISR`：

```c
// tx_user.h
#define TX_TIMER_PROCESS_IN_ISR
```

这意味着：
- **不创建**独立的定时器线程
- 所有定时器超时在 SysTick ISR 中处理
- **优点**：节省 ~1KB 栈空间 + 消除定时器线程上下文切换
- **限制**：定时器回调中只能使用 ISR 安全的 API

### 6.3 本项目的"软件定时器"替代方案

对于周期性任务，本项目使用 `tx_thread_sleep()` 而非定时器回调：

```c
// 500 Hz 周期任务（机器人控制循环）
while (1) {
    do_control();
    tx_thread_sleep(2);  // 2 ms = 500 Hz
}

// 100 Hz 周期任务（离线检测）
while (1) {
    check_offline();
    tx_thread_sleep(10);  // 10 ms = 100 Hz
}
```

---

## 7. 中断处理

### 7.1 ISR 中可安全调用的 API

| API | 类别 | 说明 |
|-----|------|------|
| `tx_semaphore_put()` | 信号量 | ISR→任务通知 |
| `tx_event_flags_set()` | 事件标志 | ISR 中设置事件 |
| `tx_queue_send()` | 队列 | 向队列发送消息 |
| `tx_queue_receive()` | 队列 | 非阻塞模式（`TX_NO_WAIT`） |
| `tx_thread_resume()` | 线程 | 恢复挂起的线程 |

**ISR 中绝对不能调用的 API**：
- `tx_thread_sleep()` — 只能在线程中使用
- `tx_mutex_get()` / `tx_mutex_put()` — 必须在任务上下文中
- `tx_byte_allocate()` / `tx_byte_release()` — 可能在 ISR 中不确定
- 任何带 `TX_WAIT_FOREVER` 或非零超时的 API

### 7.2 中断优先级配置

本项目在 `tx_initialize_low_level.S` 中配置中断优先级：

```
SysTick:   优先级 0x40（中低）  ← 驱动 RTOS 滴答
PendSV:    优先级 0xFF（最低）  ← 上下文切换
SVC:       优先级 0xFF（最低）  ← 未使用（项目未启用 txe_* 包装）
```

**关键规则**：PendSV 必须是**最低**优先级（0xFF），这样才能在所有其他 ISR 完成后才进行上下文切换。

### 7.3 ISR 与任务的典型协作模式

```
中断发生
  ↓
ISR 执行（短小快速）
  ├── 读取外设数据
  ├── kfifo_put() 放入环形缓冲
  └── tx_semaphore_put() / tx_event_flags_set()  ← 仅通知
  ↓
ISR 返回 → PendSV → 上下文切换到等待中的任务
  ↓
任务继续执行
  ├── kfifo_get() 从环形缓冲取数据
  └── 处理数据（可以慢）
```

---

## 8. 本项目中的 ThreadX 配置

### 8.1 tx_user.h 关键配置

```c
#define TX_MAX_PRIORITIES              32    // 32 个优先级
#define TX_TIMER_TICKS_PER_SECOND      1000  // 1 kHz 滴答
#define TX_TIMER_PROCESS_IN_ISR              // ISR 内处理定时器
#define TX_REACTIVATE_INLINE                 // 内联定时器重新激活
#define TX_DISABLE_STACK_FILLING             // 禁用 0xEF 栈填充
#define TX_DISABLE_PREEMPTION_THRESHOLD      // 禁用抢占阈值
#define TX_DISABLE_NOTIFY_CALLBACKS          // 禁用通知回调
#define TX_INLINE_THREAD_RESUME_SUSPEND      // 内联线程恢复/挂起
```

这是**性能优先**的配置，禁用了所有不必要的特性以减少代码体积和 CPU 开销。

### 8.2 本项目的线程优先级分配

```
优先级 3:  CAN RX Task         ← 最高优先级（CAN 延迟敏感）
优先级 4:  CAN TX Task
优先级 6:  Offline Detect       ← 100 Hz 看门狗
优先级 7:  INS Task             ← 1000 Hz 姿态估计
优先级 8:  Remote VT / wt606    ← 串口接收
优先级 9:  Remote Ctrl          ← SBUS 遥控器解析
优先级 10: Vision / Referee / USB OSAL
优先级 12: Motor                ← 电机控制（必须在 INS 之后）
优先级 30: Robot Control        ← 最低优先级（主控制循环）
```

**设计原则**：
- I/O 任务（CAN、UART）优先级高，减少数据延迟
- 传感器处理（INS）在控制输出（Motor）之前
- 主控制循环优先级最低，使用最新数据计算

### 8.3 启动流程

```
硬件上电
  → Reset_Handler（汇编）
    → SystemInit()（时钟配置）
    → main()
      → HAL_Init()（HAL 库初始化）
      → SystemClock_Config()（设置 480MHz / 168MHz）
      → 外设初始化（MX_GPIO_Init, MX_USART_Init, ...）
      → tx_kernel_enter()   ← 永不返回！
          → _tx_initialize_low_level()（汇编：SysTick、向量表）
          → tx_application_define()（C 代码）
              → Robot_Init()（本项目入口）
```

---

## 9. 实际案例分析

### 9.1 CAN 通信架构（完整案例）

**数据结构**：
```c
// 两个信号量
TX_SEMAPHORE g_can_rx_sem;  // 接收信号量
TX_SEMAPHORE g_can_tx_sem;  // 发送信号量

// 环形缓冲（内核 FIFO）
Kfifo rx_fifo;  // 接收缓冲
Kfifo tx_fifo;  // 发送缓冲
```

**接收路径**：
```
硬件 FDCAN → FIFO 中断
  → HAL_FDCAN_RxFifo0Callback()（ISR 上下文）
    → 从硬件读所有消息
    → kfifo_put(&bus->rx_fifo, msg)  ← 快速入队
    → tx_semaphore_put(&g_can_rx_sem) ← 通知接收任务
  → ISR 返回

can_rx_task（优先级 3，任务上下文）：
  while (1) {
    tx_semaphore_get(&g_can_rx_sem, TX_WAIT_FOREVER);
    while (kfifo_get(&bus->rx_fifo, &msg)) {
      // 查找匹配的 CAN 设备
      dev = can_dev_find(bus, msg.id);
      // 调用应用程序回调
      dev->rx_callback(dev, msg.data, msg.len);
    }
  }
```

**发送路径**：
```
应用程序调用 BSP_CAN_Send()（任务上下文）
  → kfifo_put(&bus->tx_fifo, msg)
    → 如果满了，丢弃最旧消息（确保最新数据优先）
  → tx_semaphore_put(&g_can_tx_sem)

can_tx_task（优先级 4，任务上下文）：
  while (1) {
    tx_semaphore_get(&g_can_tx_sem, TX_WAIT_FOREVER);
    while (kfifo_get(&bus->tx_fifo, &msg)) {
      // 重试直到硬件 FIFO 接受
      while (HAL_FDCAN_AddMessageToTxFifoQ(...) != HAL_OK) ;
    }
  }
```

### 9.2 UART 接收架构

```c
// 使用事件标志同步
TX_EVENT_FLAGS_GROUP event_flags;

// HAL 回调（ISR 上下文）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // Size 是本次收到的新数据量
    BSP_CACHE_INVALIDATE(rx_buffer, Size);  // H7 需要
    kfifo_put(&dev->rx_fifo, rx_buffer, Size);
    tx_event_flags_set(&dev->event_flags, BSP_UART_EVENT_RX, TX_OR);
}

// 接收任务（任务上下文）
ULONG actual;
tx_event_flags_get(&dev->event_flags, BSP_UART_EVENT_RX,
                   TX_OR_CLEAR, &actual, TX_WAIT_FOREVER);
// 处理收到的数据...
```

### 9.3 线程间数据共享（无锁方案）

```c
// 视觉模块向主控制循环提供数据
// 使用 kfifo（无锁环形缓冲）而非互斥量

// 发送端（USB 任务）
static SendPacket send_packet;
Module_Vision_Send(&send_packet, TX_NO_WAIT);

// 接收端（主控制循环）
receive_packet = Module_Vision_Receive();  // 返回最新数据包指针
// 注意：这里返回的是内部缓冲区的指针，使用后应立即处理
```

---

## 10. ThreadX 在 Cortex-M 上的移植要点

### 10.1 FPU 上下文保存

Cortex-M4/M7 有硬件浮点单元（FPU）。ThreadX 在上下文切换时需要保存/恢复 FPU 寄存器：

```c
// tx_port.h
#ifdef TX_ENABLE_FPU_SUPPORT
// 在线程创建时初始化 FPU 上下文
// 在 tx_thread_context_save.S 中：VSTM S16-S31
// 在 tx_thread_context_restore.S 中：VLDM S16-S31
#endif
```

本项目通过 `TX_ENABLE_FPU_SUPPORT` 启用此特性。

### 10.2 PendSV 调度器触发

Cortex-M 的 PendSV（可挂起的系统调用）异常用于触发上下文切换：

```c
// tx_port.h 中的宏
#define TX_PORT_SAVE_CONTEXT()
    // 通过设置 PendSV 挂起位来触发调度器
    *((volatile ULONG *) 0xE000ED04) = 0x10000000;
```

### 10.3 临界区保护

本项目使用 **PRIMASK**（`CPSID i` / `CPSIE i`）禁用中断来保护临界区，而非 BASEPRI 方法。（`TX_NOT_INTERRUPTABLE` 已定义）

### 10.4 SysTick 重载值计算

```asm
; tx_initialize_low_level.S (Cortex-M7, 480 MHz)
    LDR r0, =480000000                ; CPU 频率
    LDR r1, =1000                     ; TX_TIMER_TICKS_PER_SECOND
    UDIV r0, r0, r1                   ; 480000000 / 1000 = 480000
    LDR r1, =SysTick_LOAD_RELOAD_Msk  ; 0xFFFFFF
    SUB r0, r1, r0                    ; 480000 - 1 = 479999
    ; ...
    LDR r1, =0xE000E014               ; SysTick 重载寄存器
    STR r0, [r1]                      ; 加载重载值
```

计算出 SysTick 每毫秒触发一次：`480000000 / 1000 = 480000` → 重载值 `479999`。

### 10.5 向量表偏移

```asm
; 将向量表地址写入 VTOR
LDR r0, =g_pfnVectors     ; 向量表符号
LDR r1, =0xE000ED08        ; VTOR 寄存器地址
STR r0, [r1]               ; 设置向量表
```

这在 STM32H7 上尤其重要，因为应用程序可能从 Flash 运行，而默认向量表指向 bootloader 区域。
