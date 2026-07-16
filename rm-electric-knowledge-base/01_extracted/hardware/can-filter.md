# STM32F407 CAN 过滤器配置规则

> 提炼自 STM32F407 中文手册第 24 章（bxCAN），第 607-620 页、640-644 页。
> 手册：[[00_raw/hardware/STM32F407中文手册(完全版) 高清完整.pdf|STM32F407中文手册(完全版)]]

---

## 为什么需要过滤器

CAN 总线是广播式通信——总线上所有消息都会到达每个节点。如果不加筛选，CPU 要处理大量无关消息。bxCAN 控制器内置了硬件过滤器，在消息进入软件之前就把不需要的消息丢弃，节省 CPU 资源。

---

## 28 个筛选器组（Filter Bank）

STM32F407 有双 CAN（CAN1 主、CAN2 从），两者**共享 28 个筛选器组**（编号 0-27）。

通过 `CAN_FMR` 寄存器的 `CAN2SB[5:0]` 字段设置分界点：

```
筛选器组编号:  0  1  2  ...  CAN2SB-1 | CAN2SB  ...  27
分配给:        CAN1（主）              | CAN2（从）
```

本项目设 `SlaveStartFilterBank = 14`，即平均分配：CAN1 用 bank 0-13，CAN2 用 bank 14-27。这样防止一方占用过多导致另一方无法正常接收。

---

## 每个筛选器组的配置维度

每个筛选器组有两个 32 位寄存器（`CAN_FxR0` 和 `CAN_FxR1`），可配置三个维度：

### 1. 尺度（Scale）：32 位 or 16 位

通过 `CAN_FS1R` 寄存器的 `FSCx` 位配置：

| FSCx | 尺度 | 效果 |
|------|------|------|
| 1 | 32 位 | 一个筛选器组 = 1 个 32 位筛选器 |
| 0 | 16 位 | 一个筛选器组 = 2 个 16 位筛选器 |

### 2. 模式（Mode）：掩码 or 列表

通过 `CAN_FM1R` 寄存器的 `FBMx` 位配置：

| FBMx | 模式 | 效果 |
|------|------|------|
| 0 | 掩码模式（ID Mask） | `FxR0` = 期望 ID，`FxR1` = 掩码。掩码为 1 的位必须匹配，为 0 的位忽略 |
| 1 | 列表模式（ID List） | `FxR0` 和 `FxR1` 各存一个精确 ID，传入 ID 必须完全等于其中之一 |

### 3. FIFO 分配

通过 `CAN_FFA1R` 寄存器的 `FFAx` 位配置：

| FFAx | 分配到 |
|------|--------|
| 0 | FIFO 0 |
| 1 | FIFO 1 |

---

## 16 位尺度下的位映射

本项目使用 16 位尺度 + 掩码模式。在 16 位尺度下，一个筛选器组的两个寄存器被拆成 4 个 16 位字段：

```
CAN_FxR1:  [15:0] = 第一个筛选器的 ID     [31:16] = 第一个筛选器的掩码
CAN_FxR2:  [15:0] = 第二个筛选器的 ID     [31:16] = 第二个筛选器的掩码
```

每个 16 位字段的位映射（标准 ID 场景）：

```
bit:  15  14  13  12  11  10   9   8   7   6   5   4   3   2   1   0
      STID[10:3]                STID[2:0]  RTR  IDE  EXID[17:15]
```

- `STID[10:0]`：11 位标准 ID（CAN 标准 ID 最大 0x7FF = 2047）
- `RTR`：远程请求帧标志
- `IDE`：标识符扩展标志（0 = 标准 ID，1 = 扩展 ID）
- `EXID[17:15]`：扩展 ID 的高 3 位

### 为什么 `FilterIdLow = rx_id << 5`

标准 ID 放在 `STID[10:0]`，即 bit[15:5]。所以需要把 `rx_id` 左移 5 位，让它对齐到正确位置：

```
rx_id = 0x201 = 0b 0010 0000 0001

左移 5 位后:
bit:  15  14  13  12  11  10   9   8   7   6   5   4   3   2   1   0
      0   0   1   0   0   0    0   0   0   0   1   0   0   0   0   0
      └─── STID[10:3] ────────┘ └ STID[2:0] ┘ RTR IDE EXID[17:15]
                                └─ 低位全是 0（标准 ID、数据帧）
```

### 为什么 `FilterMaskIdLow = 0x7FF << 5`

掩码 `0x7FF` 对应 11 位全 1，左移 5 位后 `STID[10:0]` 的每一位都标记为"必须匹配"。低位（RTR、IDE、EXID）对应的掩码位为 0，表示"忽略"。

效果：**只接收 STID 完全等于 `rx_id` 的标准数据帧**。

---

## HAL 库封装

STM32 HAL 库把上述寄存器操作封装成 `CAN_FilterTypeDef` 结构体和 `HAL_CAN_ConfigFilter()` 函数：

```c
CAN_FilterTypeDef cfg = {
    .FilterMode           = CAN_FILTERMODE_IDMASK,    // 掩码模式
    .FilterScale          = CAN_FILTERSCALE_16BIT,    // 16 位尺度（2 个筛选器）
    .FilterFIFOAssignment = (rx_id & 1) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1,  // 奇偶分配 FIFO
    .SlaveStartFilterBank = 14,                       // CAN1/CAN2 分界
    .FilterIdLow          = rx_id << 5,               // 期望 ID（左移对齐）
    .FilterMaskIdLow      = 0x7FF << 5,               // 掩码：11 位全匹配
    .FilterIdHigh         = rx_id << 5,               // 第二个筛选器也配同样的 ID
    .FilterMaskIdHigh     = 0x7FF << 5,
    .FilterBank           = filter_bank,              // 使用哪个 bank
    .FilterActivation     = CAN_FILTER_ENABLE,        // 激活
};
HAL_CAN_ConfigFilter(hcan, &cfg);
```

### 奇偶分配 FIFO 的原因

`(rx_id & 1)` 根据接收 ID 的奇偶性分配到 FIFO0 或 FIFO1。这样消息被分散到两个 FIFO（各 3 级深度），减少单个 FIFO 溢出的概率。两个 FIFO 都有中断回调，RX 任务会同时处理两个 FIFO 的数据。

---

## 配置步骤总结

1. 确定该设备挂在 CAN1 还是 CAN2 上
2. 分配一个空闲的 filter bank（CAN1: 0-13，CAN2: 14-27）
3. 选择尺度（本项目统一用 16 位）
4. 选择模式（本项目统一用掩码模式，精确匹配单个 ID）
5. 设置 `FilterId = rx_id << 5`，`FilterMask = 0x7FF << 5`
6. 根据 `rx_id` 奇偶分配 FIFO
7. 激活筛选器
