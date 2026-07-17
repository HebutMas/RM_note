# sbus.c/h — SBUS 遥控器驱动

`modules/REMOTE/SBUS/sbus.{c,h}`

## 一句话

阻塞在 UART DMA 上等 25 字节数据，扫描帧头 `0x0F` + 帧尾 `0x00`，解码 16 个 11bit 通道，写入全局 `Remote_Data_t`。

## 条件编译

```c
#if defined(REMOTE_SOURCE) && (REMOTE_SOURCE == 1)
```

`REMOTE_SOURCE` 在 `module_remote.h` 中默认定义为 1，可被 `module_config.h`（由 `generate_headers.cmake` 生成）覆盖。编译期选择 SBUS 或 DT7，互斥。

## 初始化：`remote_sbus_init()`

```c
REMOTE_UART.Init.BaudRate = 100000;    // SBUS 固定 100kbps
HAL_UART_Init(&REMOTE_UART);           // 串口参数: 8E2 (偶校验, 2停止位)

// 注册离线检测 (50ms 超时, 静默故障)
Offline_Init_config_t offline_cfg = {
    .name = "sbus", .timeout_ms = 50, .beep_times = 0, .enable = 1,
};

// UART 设备初始化 (DMA 接收, 期望帧长 25 字节)
UART_Device_init_config uart_cfg = {
    .huart = &REMOTE_UART,
    .expected_rx_len = SBUS_FRAME_SIZE,  // 25
    .rx_buf = sbus_rx_buf,
    .rx_mode = UART_MODE_DMA,
};
```

`REMOTE_UART` 默认是 `huart5`，在 `module_remote.h` 中定义，可被 `module_config.h` 覆盖。对应板载 DBUS 接口，详见 [[01_extracted/hardware/c-board-resources#DBUS 接口]]。串口底层驱动见 [[02_code_twin/board/bsp/UART/bsp_uart]]。

串口参数 100kbps + 偶校验 + 2 停止位，协议详见 [[01_extracted/remote/remote_protocol#SBUS（Futaba）]]。

离线检测 `beep_times = 0` 表示静默故障（红灯常亮不响蜂鸣），注册流程见 [[02_code_twin/modules/OFFLINE/module_offline-c#Module_Offline_register()]]。

## 解码：`remote_sbus_decode()`

```c
BSP_UART_Read(sbus_uart, buf, sizeof(buf), &rx_len, TX_WAIT_FOREVER);
```

阻塞等待 DMA 数据。收到后扫描缓冲区找帧：

```c
for (uint32_t i = 0; i + SBUS_FRAME_SIZE - 1 < rx_len; i++) {
    if (buf[i] != SBUS_HEADER) continue;      // 0x0F
    if (buf[i + 24] != SBUS_TAIL) continue;   // 0x00
    // 解码...
}
```

### 通道解码

每个通道 11bit，低位在前，紧凑排列。以 ch0 为例：

```c
int16_t ch1 = ((int16_t)buf[i + 1] >> 0 | ((int16_t)buf[i + 2] << 8)) & 0x07FF;
```

位运算对应协议帧结构 [[01_extracted/remote/remote_protocol#通道→字节位映射]]：CH0 占用 buf[1] 全部 8bit + buf[2] 低 3bit，`& 0x07FF` 取低 11 位。

ch2 跨 3 个字节（因为 11×3=33，偏移 33bit 落在 buf[3] 的第 2 位开始）：

```c
int16_t ch3 = ((int16_t)buf[i + 3] >> 6 | ((int16_t)buf[i + 4] << 2) | (int16_t)buf[i + 5] << 10) & 0x07FF;
```

### 死区映射逻辑（核心）

SBUS 通道分为两类，解码处理方式不同：

**CH1~CH4（摇杆通道）—— 减偏移 + 死区过滤**

```c
// 第一步：范围校验，超出范围视为无效，重置为中值
if (ch1 < SBUS_CHX_UP || ch1 > SBUS_CHX_DOWN) ch1 = SBUS_CHX_BIAS;

// 第二步：减去中值偏移，摇杆中位变为 0
ch1 -= SBUS_CHX_BIAS;                              // 1024 → 0

// 第三步：死区过滤，消除摇杆中心抖动
if (abs(ch1) <= REMOTE_DEAD_ZONE) ch1 = 0;         // ±10 以内归零
```

为什么需要死区：摇杆机械结构存在中心抖动，松手时原始值不会精确回到 1024，可能在 1015~1033 之间漂移。如果不做死区，松手后底盘/云台会缓慢漂移。

- `SBUS_CHX_UP (240)` / `SBUS_CHX_DOWN (1807)`：有效范围边界，超出视为无效
- `SBUS_CHX_BIAS (1024)`：中值偏移，减去后摇杆中位为 0，上推为正，下拉为负
- `REMOTE_DEAD_ZONE (10)`：死区阈值，绝对值 ≤10 时归零

减偏移后的值范围：`240-1024 = -784` ~ `1807-1024 = +783`，使用时除以 `SBUS_CHX_DOWN - SBUS_CHX_BIAS`（783）归一化到 -1.0~+1.0。

通道与物理操控的对应关系见 [[01_extracted/remote/remote_protocol#通道映射（SBUS / 通用遥控器）]]。

**CH5~CH16（开关/旋钮通道）—— 原始值，不减偏移、不做死区**

```c
// 直接写入原始值，不做任何处理
data->channels[4] = (int16_t)(... ) & 0x07FF;  // CH5
data->channels[5] = (int16_t)(... ) & 0x07FF;  // CH6
```

这些通道用于三档拨杆开关，判断逻辑是：

```c
if (ch == SBUS_CHX_UP)        // 240 → 上档
else if (ch == SBUS_CHX_BIAS)  // 1024 → 中档
else if (ch == SBUS_CHX_DOWN)  // 1807 → 下档
```

因为拨杆只有三个固定位置，不需要死区过滤。如果做死区反而会导致中档（1024±10）被归零而误判。

> **注意**：`Module_Remote_get_channel(ch)` 返回的 CH1~CH4 是已减偏移+死区的带符号值，CH5~CH16 是原始值。应用层调用时需区分处理。

### 扩展通道

CH5~CH16 同样用 11bit 解码，但不做零偏和死区处理（原始值直接写入 `data->channels[4..15]`），原因见上方[死区映射逻辑](#死区映射逻辑（核心））。

### 离线检测更新

```c
if (buf[i + 23] == 0x00) {
    Module_Offline_device_update(offline);
}
```

字节 23 是 flags 标志位 [[01_extracted/remote/remote_protocol#flags 标志位（字节 23）]]。`0x00` 表示无帧丢失、无失控保护，此时更新心跳。如果 bit4 或 bit5 置位（失控保护/帧丢失），不更新心跳，触发离线超时。

## 数据写入

直接写入 `Remote_Data_t *data`（实际是 `module_remote.c` 中的全局 `g_remote_data`）：

```c
data->channels[0] = ch1;  // 已去零偏 + 死区
data->channels[1] = ch2;
data->channels[2] = ch3;
data->channels[3] = ch4;
```

SBUS 只填 `channels[0..15]`，不填 `dt7`/`mouse`/`keyboard`（那些是 DT7 专属）。

统一数据结构定义见 [[02_code_twin/modules/REMOTE/module_remote]]。
