# 动态设备管理：数组槽位 vs 链表

> 本文档对比项目中三种动态设备管理方式：纯数组槽位（UART BSP）、数组槽位 + 指针（CAN BSP）和链表头插法（Motor/Offline 模块）。

---

## 三种方式对比

| | 纯数组槽位（UART BSP） | 数组槽位 + 指针（CAN BSP） | 链表头插法（Motor/Offline） |
|---|---|---|---|
| **存储** | 静态数组直接存结构体 `UART_Device table[N]` | 静态数组存指针 `can_dev_slot_t devices[8]` | 每个实例从内存池 malloc，`next` 指针串联 |
| **malloc** | **不 malloc**，直接用数组里的结构体 | 设备结构体从内存池 malloc，指针存入槽位 | 整个结构体从内存池 malloc |
| **查找** | 遍历数组比 `huart` | 遍历数组比 `rx_id` | 遍历链表比 `rx_id` |
| **cache** | 最友好——结构体连续存放 | 较友好——数组连续，但设备结构体分散 | 不友好——指针跳转 |
| **空槽位管理** | `initialed == 0` 表示空 | `dev == NULL` 表示空 | 不需要，malloc 即分配 |
| **上限** | 编译期固定（F407: 3, H723: 6） | 编译期固定 8 | 受内存池大小限制 |
| **删除** | 置 `initialed = 0` | 置 `dev = NULL` | 需遍历链表找前驱节点 |
| **适用场景** | 设备数量最少且完全固定（就那几个串口） | 设备数量少且固定（CAN 总线 ≤8） | 设备数量灵活，初始化时一次性注册 |

---

## 纯数组槽位（UART BSP）

### 数据结构

```c
// 数组直接存结构体，不是指针
static UART_Device uart_device_table[BSP_UART_DEVICE_NUM];  // F407: 3, H723: 6
```

每个 `UART_Device` 包含 UART 句柄、kfifo、DMA 缓冲区指针、事件标志组等，整个结构体直接放在数组里。

### 分配过程

```c
// 1. 找空槽位（initialed == 0 表示空闲）
int slot = Find_Free_Slot();  // 遍历数组找 initialed == 0 的位置
if (slot < 0) return NULL;    // 满了

// 2. 直接取地址，不 malloc
UART_Device *dev = &uart_device_table[slot];
dev->initialed = 1;
```

**关键点**：不 malloc，直接 `&uart_device_table[slot]` 取地址。结构体在编译期就已经分配好内存了，运行时只是标记"这个槽位被占了"。

### 查找过程

```c
static UART_Device *Find_Device(UART_HandleTypeDef *huart) {
    for (int i = 0; i < BSP_UART_DEVICE_NUM; ++i)
        if (uart_device_table[i].initialed && uart_device_table[i].huart == huart)
            return &uart_device_table[i];
    return NULL;
}
```

HAL 回调里只有 `huart` 句柄，通过它反查找到对应的 `UART_Device`。

### 为什么 UART 用纯数组、连指针都不存

1. **数量最少且完全固定**：F407 就 3 个串口、H723 就 6 个，芯片决定了上限，不可能变
2. **结构体不大**：`UART_Device` 大小可接受，直接放数组不浪费
3. **零动态分配**：连 malloc 都不调，最简单也最可靠，不存在内存池耗尽的问题
4. **查找频率中等**：每次中断回调查一次，不像 CAN 那样每帧都查上千次，但数组连续存放也够快

> 具体实现见 [[02_code_twin/board/bsp/UART/bsp_uart]]

---

## 数组槽位 + 指针（CAN BSP）

### 数据结构

```c
// 槽位：存 rx_id + 设备指针
typedef struct {
    uint32_t    rx_id;  // 接收帧 ID，0 = 空闲槽位
    Can_Device *dev;    // 设备指针，NULL = 空闲
} can_dev_slot_t;

// 总线管理器内嵌数组
typedef struct {
    CAN_HandleTypeDef *hcan;
    can_dev_slot_t devices[BSP_CAN_MAX_DEV_PER_BUS];  // 8 个槽位
    // ...
} CAN_Bus_Manager;
```

### 分配过程

```c
// 1. 找空槽位
int slot = can_dev_find_empty(bus);  // 遍历数组找 dev == NULL 的位置
if (slot < 0) return NULL;           // 满了

// 2. 从内存池分配设备结构体
Can_Device *dev;
BSP_MEM_ALLOC_WAIT(dev, sizeof(Can_Device), TX_NO_WAIT);

// 3. 填充并插入槽位
can_dev_insert(bus, slot, config->rx_id, dev);
// bus->devices[slot].rx_id = rx_id;
// bus->devices[slot].dev   = dev;
```

**关键点**：数组 `devices[8]` 是静态分配的（编译期固定），但每个槽位指向的 `Can_Device` 是动态分配的（内存池 malloc）。两步分离：先找空槽位，再 malloc 设备结构体填进去。

### 查找过程

```c
Can_Device *can_dev_find(CAN_Bus_Manager *bus, uint32_t rx_id) {
    for (int i = 0; i < BSP_CAN_MAX_DEV_PER_BUS; i++) {
        if (bus->devices[i].rx_id == rx_id)
            return bus->devices[i].dev;
    }
    return NULL;
}
```

**与 UART 纯数组的区别**：UART 查找返回的是数组内的结构体地址（`&table[i]`），CAN 查找返回的是数组中存的指针（`devices[i].dev`），指针指向的 `Can_Device` 在堆上。多一次指针跳转，但因为数组本身连续，遍历比 ID 的部分仍然是 cache 友好的。

### 为什么 CAN 用数组+指针而不是纯数组

1. **Can_Device 结构体较大**：包含发送缓冲区、回调函数指针、配置参数等，直接放数组会占太多空间
2. **设备数量固定但比 UART 多**：每条总线最多 8 个，编译期知道上限
3. **查找频率最高**：每收到一帧 CAN 数据都要查一次，1 kHz × N 个设备 = 每秒上千次，数组连续访问 cache 友好

> 具体实现见 [[02_code_twin/board/bsp/CAN/bsp_can]]

---

## 链表头插法（Motor/Offline 模块）

### 数据结构

```c
struct Motor_Base {
    Motor_Base *next;     // 链表指针
    // ... 其他字段
};
static Motor_Base *g_motor_list = NULL;
```

整个结构体从内存池 malloc，`next` 指针串联。

### 分配过程

```c
void Motor_Register(Motor_Base *motor) {
    motor->next = g_motor_list;   // 头插
    g_motor_list = motor;
}
```

不需要空槽位管理，malloc 即分配，头插即注册。

### 为什么 Motor/Offline 用链表而不是数组

1. **设备数量灵活**：不同机器人配置的电机数量不同（步兵 4~8 个，哨兵 3~6 个），不方便固定数组大小
2. **查找频率低**：`Motor_UpdateAll` 每个 2ms 周期遍历一次链表，不像 CAN 每帧都要查找
3. **多态需要**：链表存的是基类指针 `Motor_Base *`，遍历时通过函数指针多态分发，不需要按 ID 查找

---

## 三者的本质区别

```
纯数组（UART）          数组+指针（CAN）         链表（Motor）
┌─────────┐            ┌─────────┐            ┌─────────┐
│Device[0]│            │slot[0] ─┼─→ malloc   │ Node A  │
│Device[1]│            │slot[1] ─┼─→ malloc   │   ↓     │
│Device[2]│            │slot[2]  │  NULL      │ Node B  │
└─────────┘            └─────────┘            │   ↓     │
  结构体在数组里          指针在数组里            │ Node C  │
  零 malloc              设备 malloc             │   ↓     │
  cache 最友好           cache 较友好            │ NULL    │
                                                └─────────┘
                                                  全在堆里
                                                  cache 不友好
```

| 维度 | 纯数组（UART） | 数组+指针（CAN） | 链表（Motor） |
|------|--------------|----------------|-------------|
| 内存位置 | 全在 .bss 段（连续） | 数组在 .bss，设备在堆（分散） | 全在堆（分散） |
| malloc 次数 | 0 | N 次（每个设备一次） | N 次（每个节点一次） |
| 查找方式 | 遍历比 `huart` | 遍历比 `rx_id` | 遍历比 `rx_id` |
| 查找返回 | `&table[i]`（数组内地址） | `devices[i].dev`（堆地址） | `node`（堆地址） |
| 指针跳转 | 0 次 | 1 次（数组→堆） | 每步 1 次（节点→节点） |
| 删除 | `initialed = 0` | `dev = NULL` | 遍历找前驱 |
