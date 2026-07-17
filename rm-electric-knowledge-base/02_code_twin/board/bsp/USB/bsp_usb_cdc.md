# bsp_usb_cdc — USB 虚拟串口（CDC ACM）

`board/bsp/USB/usbd_cdc_acm_user.{c,h}`

## 一句话

基于 CherryUSB 协议栈的 USB CDC ACM 设备，将 STM32 的 USB OTG 外设模拟成串口，供上位机通过 USB 线收发数据。当前唯一用途是视觉模块（MCU ↔ 上位机/迷你主机）通信。

## 为什么用它

- **不需要额外串口芯片**：C 板（F407）和 A 板（H723）都有 USB OTG 接口，直接插 USB 线就能通信
- **比 UART 快**：USB FS 12Mbps / HS 480Mbps，远超 UART 的几 Mbps
- **上位机免驱**：CDC ACM 是标准 USB 设备类，Windows/Linux/macOS 自带驱动，插上就是 COM 口
- **不占 UART 资源**：UART 口留给调试和遥控器，视觉通信走 USB 独立通道

## 协议栈分层

```
应用层      module_vision.c       Module_Vision_Send / Module_Vision_Receive
              │
用户封装层    usbd_cdc_acm_user.c   cdc_acm_init / cdc_acm_send / cdc_acm_recv
              │
CherryUSB    usbd_cdc_acm.c        CDC ACM 类驱动（标准协议处理）
              │
             usbd_core.c           USB 设备核心（枚举/端点管理）
              │
HAL/LL       usb_otg.c             DWC2 控制器寄存器操作
              │
硬件          USB OTG FS (F407) / HS (H723)
```

我们只写用户封装层 `usbd_cdc_acm_user.c`，下面的 CherryUSB 协议栈是开源库，不展开。

## 初始化：cdc_acm_init

```c
void cdc_acm_init(uint8_t busid, uintptr_t reg_base)
```

在 [[02_code_twin/board/bsp/DWT/bsp_dwt|BSP_Init]] 中调用，按板子区分 USB 外设：

| 板子 | reg_base | USB 模式 | 最大包长 |
|------|----------|----------|---------|
| C 板（F407） | `USB_OTG_FS_PERIPH_BASE` | Full-Speed 12Mbps | 64 字节 |
| A 板（H723） | `USB_OTG_HS_PERIPH_BASE` | High-Speed 480Mbps | 512 字节 |

初始化做了 4 件事：

1. **kfifo 初始化** — `cdc_rx_fifo` 用 1024 字节缓冲区，接收到的 USB 数据先存这里
2. **事件标志组创建** — `usb_event_flags`，RX/TX 两个标志位，用于 ThreadX 任务同步
3. **描述符注册** — VID/PID 均为 `0xFFFF`，设备类 `0xEF/0x02/0x01`（IAD，复合设备），2 个接口（CDC ACM 标准需要通信接口 + 数据接口）
4. **端点注册 + 启动** — 注册 IN/OUT 两个批量端点，`usbd_initialize` 启动 USB 设备

### 端点地址

| 端点 | 地址 | 方向 | 用途 |
|------|------|------|------|
| CDC_IN_EP | `0x81` | MCU → Host | 发送数据 |
| CDC_OUT_EP | `0x02` | Host → MCU | 接收数据 |
| CDC_INT_EP | `0x83` | MCU → Host | 通知（波特率等控制信号） |

> 端点地址的 bit7 是方向位：1 = IN（设备到主机），0 = OUT（主机到设备）。`0x81` = EP1 IN，`0x02` = EP2 OUT。

### USB 枚举时的 PA12 复位

```c
// bsp_init.c
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);  // 拉低 D+
BSP_DWT_Delay(0.1);                                      // 保持 100ms
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);    // 释放
BSP_DWT_Delay(0.1);
cdc_acm_init(0, USB_OTG_FS_PERIPH_BASE);
```

PA12 是 USB DP（D+）线。先拉低再释放，模拟 USB 拔插动作，强制主机重新枚举设备。否则如果 MCU 复位但 USB 线没拔，主机可能不会重新识别设备。

## 发送：cdc_acm_send

```c
int cdc_acm_send(const uint8_t *data, uint32_t len, uint32_t timeout)
```

| 参数 | 说明 |
|------|------|
| `data` | 发送数据指针 |
| `len` | 字节数，**不能超过 `CDC_MAX_MPS`**（FS=64 / HS=512） |
| `timeout` | 等待上次发送完成的超时（ThreadX tick） |
| 返回值 | 成功返回 `len`，失败返回 `-1` |

### 发送流程

1. **等 TX 就绪** — `tx_event_flags_get` 等待 `BSP_USB_EVENT_TX` 标志（`TX_OR_CLEAR` 取出后自动清除）
2. **拷贝到发送缓冲区** — `memcpy(cdc_write_buffer, data, len)`，因为 USB DMA 要求缓冲区在特定内存区域（`USB_NOCACHE_RAM_SECTION` + `USB_MEM_ALIGNX`）
3. **启动 IN 端点写** — `usbd_ep_start_write` 把数据丢给 USB 控制器，函数立即返回
4. **等下次调用时阻塞** — 下次调用 `cdc_acm_send` 时，步骤 1 会阻塞直到上次发送完成

### ZLP（零长度包）

```c
void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes)
        usbd_ep_start_write(busid, CDC_IN_EP, NULL, 0);  // 发 ZLP
    else
        tx_event_flags_set(&usb_event_flags, BSP_USB_EVENT_TX, TX_OR_CLEAR);
}
```

USB 批量传输的规则：**如果最后一次发送的数据长度正好是最大包长的整数倍，主机不知道数据是否结束**。此时需要发一个 0 字节的 ZLP（Zero-Length Packet）告诉主机"数据发完了"。只有 ZLP 发完后才置 TX 标志，允许下一次发送。

## 接收：cdc_acm_recv

```c
int cdc_acm_recv(uint8_t *data, uint32_t buf_size, uint32_t *rx_len, uint32_t timeout)
```

| 参数 | 说明 |
|------|------|
| `data` | 接收缓冲区 |
| `buf_size` | 缓冲区大小 |
| `rx_len` | 输出实际接收字节数 |
| `timeout` | 等待数据超时（可传 `TX_WAIT_FOREVER`） |
| 返回值 | 成功返回实际字节数，超时返回 `-1` |

### 接收数据流

```
USB 主机发送数据
    │
    ▼
usbd_cdc_acm_bulk_out()          ← CherryUSB 中断回调
    │
    ├── kfifo_in(&cdc_rx_fifo, ...)   数据写入 kfifo
    ├── tx_event_flags_set(RX)        置 RX 标志
    └── usbd_ep_start_read()          重新启动 OUT 端点接收下一包
    │
    ▼
cdc_acm_recv()                   ← 应用层调用
    │
    ├── tx_event_flags_get(RX)        等待 RX 标志
    └── kfifo_out(&cdc_rx_fifo, ...)  从 kfifo 读出数据
```

关键设计：**中断回调只写 kfifo + 置标志，不做协议解析**。应用层在自己的线程里调用 `cdc_acm_recv` 取数据，实现生产者-消费者解耦。kfifo 是 SPSC（单生产者单消费者）无锁环形缓冲区，详见 [[01_extracted/algorithm/kfifo-design]]。

### kfifo 缓冲区的必要性

USB 收到一包数据时，应用层不一定正在等待接收。kfifo 起到蓄水池作用：

- USB 端点一次最多收 64 字节（FS）或 512 字节（HS）
- kfifo 容量 1024 字节，能暂存多包数据
- 应用层一次 `cdc_acm_recv` 可以读出全部积攒的数据

## 实际使用：视觉模块

当前虚拟串口的唯一使用者是 `modules/VISION/module_vision.c`。

### 发送（MCU → 上位机）

```c
// module_vision.c
void Module_Vision_Send(SendPacket *packet, uint32_t timeout) {
    packet->header = SEND_HEADER;  // 0xAA
    packet->tail   = SEND_TAIL;    // 0x5A
    cdc_acm_send((uint8_t *)packet, sizeof(SendPacket), timeout);
}
```

`SendPacket` 是 `#pragma pack(1)` 紧凑结构体，包含帧头 + 模式 + 四元数 + 帧尾，直接以二进制发送。robot_control 中用 `TX_NO_WAIT` 调用——发不出去就算了，不阻塞控制循环。

### 接收（上位机 → MCU）

```c
// module_vision.c — vision_thread_entry
while (1) {
    int ret = cdc_acm_recv(buf, sizeof(buf), &rx_len, TX_WAIT_FOREVER);
    if (ret <= 0) continue;

    for (uint32_t i = 0; i + sizeof(ReceivePacket) <= rx_len; i++) {
        if (buf[i] == RECV_HEADER && buf[i + sizeof(ReceivePacket) - 1] == RECV_TAIL) {
            memcpy(&rx_packet, &buf[i], sizeof(ReceivePacket));
            Module_Offline_device_update(offline_dev);
            break;
        }
    }
}
```

接收线程**永久阻塞**等待 USB 数据（`TX_WAIT_FOREVER`），收到后在缓冲区里扫描帧头 `0xBB` + 帧尾 `0x5B`，匹配后 `memcpy` 取出整包。这是一个简单的**帧头帧尾定界协议**，没有 CRC 校验——视觉数据丢了下一帧还能补，不需要可靠性保证。

### 数据包结构

| 方向 | 结构体 | 帧头 | 载荷 | 帧尾 | 总大小 |
|------|--------|------|------|------|--------|
| MCU → 上位机 | `SendPacket` | `0xAA` | mode + q[4]（四元数） | `0x5A` | 1+1+16+1 = 19 字节 |
| 上位机 → MCU | `ReceivePacket` | `0xBB` | found + fire_advice + yaw + pitch + vx + vy + nav_state | `0x5B` | 1+1+1+4+4+4+4+1+1 = 21 字节 |

`#pragma pack(1)` 取消结构体对齐填充，保证内存布局和通信字节流完全一致。

## 内存布局

```c
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_read_buffer[CDC_RX_FIFO_SIZE];   // kfifo 底层缓冲（1024B）
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_write_buffer[CDC_MAX_MPS];        // 发送缓冲（64B FS / 512B HS）
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_out_ep_buffer[CDC_MAX_MPS];       // OUT 端点接收缓冲
```

| 宏 | F407 (FS) | H723 (HS) | 说明 |
|----|-----------|-----------|------|
| `USB_NOCACHE_RAM_SECTION` | `.ram` 段 | `.RAM_D2` 段 | 放到非 Cache 内存区，USB DMA 直接访问 |
| `USB_MEM_ALIGNX` | 4 字节对齐 | 4 字节对齐 | DMA 对齐要求 |
| `CDC_MAX_MPS` | 64 | 512 | USB 批量端点最大包长 |

> H723 的 Cache 是 USB 的大敌。如果缓冲区在普通 D-Cache 区域，CPU 写入后 DMA 读到的是旧数据。`.RAM_D2` 段指向 D2 域的 SRAM，这块内存不被 Cache 缓存，DMA 和 CPU 看到的数据一致。F407 没有 D-Cache，用 `.ram` 段只是统一宏定义，没有实际效果。详见 [[01_extracted/hardware/c-board-resources]]。

## 与 UART 的对比

| 特性 | USB CDC（虚拟串口） | [[02_code_twin/board/bsp/UART/bsp_uart\|UART]] |
|------|---------------------|-------|
| 物理接口 | USB（PA11/PA12） | UART TX/RX 引脚 |
| 速率 | FS 12Mbps / HS 480Mbps | 通常 ≤ 1Mbps |
| 上位机接口 | COM 口（免驱） | COM 口（需 USB-TTL 芯片） |
| 缓冲机制 | kfifo + 事件标志组 | DMA 环形缓冲区 + 事件标志组 |
| 同步方式 | ThreadX 事件标志组 | ThreadX 事件标志组 |
| DMA | USB 控制器自带 DMA | UART DMA 外设 |
| 多设备 | 一个 USB 口一个 CDC 设备 | 多个 UART 外设各自独立 |

两者都用了"中断写缓冲 + 事件标志通知 + 应用层取数据"的相同模式，只是底层传输机制不同。

## 前置条件

| 依赖 | 说明 |
|------|------|
| [[02_code_twin/board/bsp/DWT/bsp_dwt\|DWT]] | `cdc_acm_init` 之前需要 DWT 已初始化（BSP_Init 顺序保证） |
| CherryUSB 协议栈 | `CherryUSB/` 目录，CMake 中链接 |
| ThreadX | 事件标志组 `tx_event_flags_create/get/set` |
| [[01_extracted/algorithm/kfifo-design\|kfifo]] | 接收缓冲区实现 |
| USB OTG 外设 | F407 用 FS，H723 用 HS，CubeMX 配置 |
