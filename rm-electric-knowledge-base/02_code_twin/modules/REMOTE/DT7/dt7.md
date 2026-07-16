# dt7.c/h — DT7 遥控器驱动

`modules/REMOTE/DT7/dt7.{c,h}`

## 一句话

阻塞在 UART DMA 上等 18 字节数据，通过通道值范围校验帧边界，解码 4 个摇杆通道 + 2 个拨杆开关 + 鼠标/键盘/滚轮，写入全局 `Remote_Data_t`。

## 条件编译

```c
#if defined(REMOTE_SOURCE) && (REMOTE_SOURCE == 2)
```

与 SBUS 互斥，编译期选择。

## 初始化：`remote_dt7_init()`

和 SBUS 几乎一模一样，区别：

| 参数 | SBUS | DT7 |
|------|------|-----|
| 帧长 | 25 字节 | 18 字节 |
| 离线名称 | `"sbus"` | `"dt7"` |
| 波特率 | 100000 | 100000（相同） |
| UART 设备 | 同一套 `BSP_UART_Device_Init` | 同一套 |

串口参数 100kbps + 偶校验 + 1 停止位（SBUS 是 2 停止位），协议详见 [[01_extracted/remote/remote_protocol#DBUS（大疆 DT7+DR16）]]。

离线检测注册见 [[02_code_twin/modules/OFFLINE/module_offline-c#Module_Offline_register()]]。

## 解码：`remote_dt7_decode()`

```c
BSP_UART_Read(dt7_uart, buf, sizeof(buf), &rx_len, TX_WAIT_FOREVER);
```

### 帧边界校验（与 SBUS 的关键区别）

SBUS 有固定帧头 `0x0F` 和帧尾 `0x00`，DT7 **没有固定帧头**。改用通道值范围校验：

```c
int16_t ch1 = (buf[i] | buf[i + 1] << 8) & 0x07FF;
// ... 解码 ch1-4
if (ch1 < DT7_CH_VALUE_MIN || ch1 > DT7_CH_VALUE_MAX ||
    ch2 < DT7_CH_VALUE_MIN || ch2 > DT7_CH_VALUE_MAX || ...)
    continue;  // 超范围 → 不是有效帧，跳过
```

`DT7_CH_VALUE_MIN (364)` / `DT7_CH_VALUE_MAX (1684)` 来自 [[01_extracted/remote/remote_protocol#数值范围]]。4 个通道值全部在合法范围内才认为找到帧。

### 通道解码

和 SBUS 完全相同的 11bit 位运算，因为 DBUS 和 SBUS 的通道编码方式相同 [[01_extracted/remote/remote_protocol#字节→位解析]]：

```c
int16_t ch1 = (buf[i] | buf[i + 1] << 8) & 0x07FF;
int16_t ch2 = (buf[i + 1] >> 3 | buf[i + 2] << 5) & 0x07FF;
int16_t ch3 = (buf[i + 2] >> 6 | buf[i + 3] << 2 | buf[i + 4] << 10) & 0x07FF;
int16_t ch4 = (buf[i + 4] >> 1 | buf[i + 5] << 7) & 0x07FF;
```

注意 SBUS 的 buf 索引从 `i+1` 开始（跳过帧头 `0x0F`），DT7 从 `i` 开始（无帧头）。

### 死区映射逻辑（核心）

DT7 通道分为两类，与 SBUS 相同的处理策略：

**CH1~CH4（摇杆通道）—— 减偏移 + 死区过滤**

```c
// 第一步：范围校验（在帧边界校验阶段已完成）
if (ch1 < DT7_CH_VALUE_MIN || ch1 > DT7_CH_VALUE_MAX) continue;

// 第二步：减去中值偏移
data->channels[0] = ch1 - DT7_CH_VALUE_OFFSET;  // 1024 → 0

// 第三步：死区过滤
if (abs(data->channels[0]) <= REMOTE_DEAD_ZONE) data->channels[0] = 0;  // ±10 以内归零
```

- `DT7_CH_VALUE_MIN (364)` / `DT7_CH_VALUE_MAX (1684)`：有效范围
- `DT7_CH_VALUE_OFFSET (1024)`：中值偏移，等于 SBUS 的 `SBUS_CHX_BIAS`
- `REMOTE_DEAD_ZONE (10)`：死区阈值

减偏移后的值范围：`364-1024 = -660` ~ `1684-1024 = +660`。

通道与物理操控的对应关系见 [[01_extracted/remote/remote_protocol#通道映射（DBUS / DT7）]]。

**拨杆开关 S1/S2 —— 离散量，不减偏移、不做死区**

```c
data->dt7.sw1 = ((buf[i + 5] >> 4) & 0x000C) >> 2;  // 1=上, 3=中, 2=下
data->dt7.sw2 = (buf[i + 5] >> 4) & 0x0003;
```

拨杆是 2bit 离散值，只有三个固定值，不需要死区。

### 拨杆开关

```c
data->dt7.sw1 = ((buf[i + 5] >> 4) & 0x000C) >> 2;  // bit 44-45
data->dt7.sw2 = (buf[i + 5] >> 4) & 0x0003;          // bit 46-47
```

2bit 开关值：1=上，3=中，2=下。位偏移来自 [[01_extracted/remote/remote_protocol#18 字节帧结构]]。

### 鼠标/键盘（条件编译）

```c
#if (REMOTE_VT_SOURCE == 0)
    data->mouse.mouse_x = (int16_t)(buf[i + 6] | (buf[i + 7] << 8));
    // ... 鼠标 Y/Z/左键/右键
    data->keyboard.key_code = buf[i + 14] | (buf[i + 15] << 8);
#endif
```

只有 `REMOTE_VT_SOURCE == 0`（无外部图传）时才解析键鼠，因为接了 VT02/VT03 图传后键鼠数据从图传通道获取。

按键位映射见 [[01_extracted/remote/remote_protocol#按键位映射（偏移 112，长度 16bit）]]。

### 滚轮

```c
int16_t wheel = (buf[i + 16] | buf[i + 17] << 8) - DT7_CH_VALUE_OFFSET;
if (abs(wheel) <= REMOTE_DEAD_ZONE * 10) wheel = 0;  // 死区放大 10 倍
data->dt7.wheel = wheel;
```

滚轮死区是摇杆的 10 倍（`REMOTE_DEAD_ZONE * 10 = 100`），因为滚轮精度低、噪声大。

### 离线检测更新

```c
if (offline) Module_Offline_device_update(offline);
```

只要帧校验通过就更新心跳，不像 SBUS 还检查 flags 字节。因为 DT7 没有 flags 标志位。

## 数据写入

```c
data->channels[0..3]  // 摇杆，已去零偏 + 死区
data->dt7.sw1/sw2/wheel  // 拨杆 + 滚轮
data->mouse.*          // 鼠标（无外部图传时）
data->keyboard.*       // 键盘（无外部图传时）
```

DT7 填充 `channels[0..3]` + `dt7` + `mouse` + `keyboard`，比 SBUS 多了开关和键鼠数据。统一数据结构见 [[02_code_twin/modules/REMOTE/module_remote]]。
