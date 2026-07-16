# 动态设备管理：数组槽位 vs 链表

> 本文档对比项目中两种动态设备管理方式：数组槽位 + 指针（CAN BSP）和链表头插法（Motor/Offline 模块）。

---

## 两种方式对比

| | 数组槽位 + 指针（CAN BSP） | 链表头插法（Motor/Offline） |
|---|---|---|
| 存储 | 静态数组 `devices[N]`，每个槽位存 `rx_id` + `Can_Device *` | 每个实例从内存池 `malloc`，`next` 指针串联 |
| 分配 | `Can_Device` 从内存池 malloc，指针存入空槽位 | 整个结构体从内存池 malloc，头插链表 |
| 查找 | 遍历数组比 `rx_id`，数组连续访问 cache 友好 | 遍历链表比 `rx_id`，指针跳转 cache 不友好 |
| 空槽位管理 | `dev == NULL` 表示空槽，`Find_Free_Slot` 遍历找空位 | 不需要空槽位管理，malloc 即分配 |
| 上限 | 编译期固定 `N` 个槽位 | 受内存池大小限制 |
| 删除 | 置 `dev = NULL` 即可回收槽位 | 需要遍历链表找到前驱节点 |
| 适用场景 | 设备数量少且固定（CAN 总线 ≤8） | 设备数量灵活，初始化时一次性注册 |

## 数组槽位分配方式（CAN BSP）

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

**与链表的区别**：

- 数组在内存中连续，CPU 预取和 cache line 友好，8 次比较可能只需 1~2 次 cache miss
- 链表节点分散在堆里，每次 `m = m->next` 跳转都可能 cache miss
- 设备少（≤8）时，数组遍历的实际速度比链表快

### 为什么 CAN 用数组而不是链表

1. **设备数量固定且少**：每条 CAN 总线最多 8 个设备，编译期就知道上限
2. **查找频率高**：每收到一帧 CAN 数据都要 `can_dev_find` 查一次 ID，1 kHz × N 个设备 = 每秒上千次查找，cache 友好性很重要
3. **不需要动态增删**：设备在初始化时注册，运行时不会增删

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

## UART 的数组分配方式

UART 模块也用数组槽位，但和 CAN 略有不同——数组里直接存结构体（不是指针）：

```c
static UART_Device uart_device_table[BSP_UART_DEVICE_NUM];  // F407: 3, H723: 6
```

分配时不 malloc，直接用数组中的结构体：

```c
int slot = Find_Free_Slot();        // 找 initialed == 0 的位置
UART_Device *dev = &uart_device_table[slot];  // 直接取地址，不 malloc
dev->initialed = 1;
```

**与 CAN 的区别**：CAN 的数组存指针（`Can_Device *`），设备结构体从内存池 malloc；UART 的数组直接存结构体（`UART_Device`），不额外 malloc。UART 这样做是因为设备数量更少更固定（3~6 个串口），结构体不大，直接放数组里更简单。

具体实现见 [[02_code_twin/board/bsp/CAN/bsp_can]] 和 [[02_code_twin/board/bsp/UART/bsp_uart]]。
