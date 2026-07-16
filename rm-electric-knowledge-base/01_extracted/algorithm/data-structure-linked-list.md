# 链表基础知识

> 本篇解释项目中反复使用的链表数据结构：为什么用它、怎么注册、怎么遍历。

---

## 为什么用链表不用数组

项目中很多模块需要在运行时动态注册设备（电机、离线检测设备、CAN 设备等），数量不确定。数组要预先确定大小，大了浪费内存，小了不够用。链表每个节点按需分配，用 `BSP_MEM_ALLOC_WAIT`（底层是 ThreadX 的 `tx_byte_allocate`）从内存池拿一块内存，挂到链表上就行。

## 节点结构

每个链表节点的第一个字段是指向下一个节点的指针：

```c
struct Offline_Device {
    Offline_Device *next;   // 指向下一个节点
    char name[32];
    uint32_t timeout_ms;
    // ... 其他字段
};
```

`next` 为 `NULL` 表示这是链表最后一个节点。

## NULL 是什么

`NULL` 在 C 语言中就是 `(void *)0`，一个值为 0 的指针，表示"不指向任何有效内存"。

两种常见用法：

1. **链表尾部标记**：`next == NULL` 说明后面没有更多节点了，遍历到此停止
2. **安全检查**：函数返回 `NULL` 通常表示失败（内存不够、未初始化等）。调用方必须检查返回值是否为 `NULL`，否则解引用空指针会导致 HardFault

```c
Offline_Device *dev = Module_Offline_register(&cfg);
if (dev == NULL) {
    // 注册失败，不能使用 dev
    return;
}
// 注册成功，可以安全使用 dev
```

## 头插法

新节点总是插到链表头部，时间复杂度 O(1)，不需要遍历找尾部：

```c
dev->next = g_device_list;   // 新节点指向原来的头部
g_device_list = dev;          // 头指针更新为新节点
```

图示：

```
注册前:  A -> B -> NULL
注册后:  NEW -> A -> B -> NULL
```

## 判空

链表为空就是头指针为 `NULL`：

```c
if (g_device_list == NULL) {
    // 没有任何设备注册
}
```

## 遍历

从头部开始，沿着 `next` 逐个访问，直到遇到 `NULL`：

```c
for (Offline_Device *dev = g_device_list; dev != NULL; dev = dev->next) {
    // 处理每个 dev
}
```

## 注册 = 分配 + 填充 + 头插

完整流程：

```c
// 1. 从内存池分配
BSP_MEM_ALLOC_WAIT(dev, sizeof(Offline_Device), TX_NO_WAIT);
if (dev == NULL) return NULL;  // 内存不够

// 2. 清零 + 填充字段
memset(dev, 0, sizeof(*dev));
dev->timeout_ms = cfg->timeout_ms;
dev->last_time  = now;   // 初始心跳

// 3. 头插
dev->next = g_device_list;
g_device_list = dev;
```

项目中 `Module_Offline_register()`、`Motor_Register()`、`BSP_CAN_Device_Init()` 都是这个模式。

## 参考视频

链表与动态内存分配：https://www.bilibili.com/video/BV1M7w7zQEdm?t=1005.9
