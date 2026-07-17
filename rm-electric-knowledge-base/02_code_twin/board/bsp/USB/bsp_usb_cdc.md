# bsp_usb_cdc — USB 虚拟串口（CDC ACM）

`board/bsp/USB/usbd_cdc_acm_user.{c,h}`

## 一句话

基于 CherryUSB 协议栈把 STM32 的 USB OTG 模拟成串口，提供 3 个阻塞式收发接口。

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

我们只写用户封装层 `usbd_cdc_acm_user.c`，下面的 CherryUSB 协议栈是开源库，不展开。模块层封装（帧头帧尾、接收线程、数据结构）见 [[02_code_twin/modules/VISION/module_vision]]。

## 接口

| 函数 | 签名 | 作用 |
|------|------|------|
| `cdc_acm_init` | `(uint8_t busid, uintptr_t reg_base)` | 初始化 USB 设备、注册端点、创建 kfifo 和事件标志组 |
| `cdc_acm_send` | `(const uint8_t *data, uint32_t len, uint32_t timeout) → int` | 发送数据，等上次 TX 完成后启动 IN 端点写 |
| `cdc_acm_recv` | `(uint8_t *data, uint32_t buf_size, uint32_t *rx_len, uint32_t timeout) → int` | 接收数据，等 RX 标志后从 kfifo 取数据 |

## 初始化：cdc_acm_init

在 [[02_code_twin/board/bsp/DWT/bsp_dwt|BSP_Init]] 中调用，按板子区分 USB 外设：

```c
// bsp_init.c
BSP_DWT_Init(168);                                       // DWT 先初始化
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);   // PA12 拉低（USB D+）
BSP_DWT_Delay(0.1);                                      // 保持 100ms
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);     // 释放
BSP_DWT_Delay(0.1);
cdc_acm_init(0, USB_OTG_FS_PERIPH_BASE);                 // F407
```

PA12 是 USB D+ 线。拉低再释放 = 模拟 USB 拔插，强制主机重新枚举。

`cdc_acm_init` 内部做了 4 件事：

```c
void cdc_acm_init(uint8_t busid, uintptr_t reg_base) {
    kfifo_init(&cdc_rx_fifo, cdc_read_buffer, CDC_RX_FIFO_SIZE, 1);  // 1. 接收 kfifo
    tx_event_flags_create(&usb_event_flags, "usb_evt");               // 2. 事件标志组
    usbd_desc_register(busid, &cdc_descriptor);                       // 3. 描述符
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf0)); //   通信接口
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf1)); //   数据接口
    usbd_add_endpoint(busid, &cdc_out_ep);                            //   OUT 端点
    usbd_add_endpoint(busid, &cdc_in_ep);                             //   IN 端点
    usbd_initialize(busid, reg_base, usbd_event_handler);            // 4. 启动 USB
}
```

| 板子 | reg_base | USB 模式 | 最大包长 `CDC_MAX_MPS` |
|------|----------|----------|------------------------|
| C 板（F407） | `USB_OTG_FS_PERIPH_BASE` | Full-Speed 12Mbps | 64 字节 |
| A 板（H723） | `USB_OTG_HS_PERIPH_BASE` | High-Speed 480Mbps | 512 字节 |

端点地址：

| 端点 | 地址 | 方向 | 用途 |
|------|------|------|------|
| CDC_IN_EP | `0x81` | MCU → Host | 发送数据 |
| CDC_OUT_EP | `0x02` | Host → MCU | 接收数据 |
| CDC_INT_EP | `0x83` | MCU → Host | 通知（波特率等控制信号） |

枚举成功后 `USBD_EVENT_CONFIGURED` 事件置 TX 标志 + 启动第一次 OUT 端点读。

## 发送：cdc_acm_send

```c
int cdc_acm_send(const uint8_t *data, uint32_t len, uint32_t timeout) {
    // 1. 等上次发送完成
    tx_event_flags_get(&usb_event_flags, BSP_USB_EVENT_TX, TX_OR_CLEAR, &actual_flags, timeout);
    // 2. 拷贝到 USB 专用缓冲区（NOCACHE + 对齐）
    memcpy(cdc_write_buffer, data, len);
    // 3. 启动 IN 端点写（异步，函数立即返回）
    usbd_ep_start_write(0, CDC_IN_EP, cdc_write_buffer, len);
    return len;
}
```

为什么不能直接 `usbd_ep_start_write(data, ...)`？因为 USB DMA 要求缓冲区在非 Cache 内存区且对齐。用户传入的 `data` 可能在任意内存位置，必须先 `memcpy` 到 `cdc_write_buffer`（`USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX`）。

### ZLP（零长度包）

```c
void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes) {
    if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes)
        usbd_ep_start_write(busid, CDC_IN_EP, NULL, 0);  // 发 ZLP
    else
        tx_event_flags_set(&usb_event_flags, BSP_USB_EVENT_TX, TX_OR_CLEAR);
}
```

USB 批量传输规则：如果发送数据长度是最大包长的整数倍，主机不知道数据是否结束，需要补发 0 字节 ZLP。只有 ZLP 完成后才置 TX 标志。

## 接收：cdc_acm_recv

```
USB 主机发数据
    │ (USB 中断)
    ▼
usbd_cdc_acm_bulk_out(busid, ep, nbytes)
    ├── kfifo_in(&cdc_rx_fifo, cdc_out_ep_buffer, nbytes)   写入 kfifo
    ├── tx_event_flags_set(&usb_event_flags, RX, TX_OR)      置 RX 标志
    └── usbd_ep_start_read(busid, CDC_OUT_EP, ...)           重新启动接收
    │ (应用层线程)
    ▼
cdc_acm_recv(buf, 64, &rx_len, TX_WAIT_FOREVER)
    ├── tx_event_flags_get(&usb_event_flags, RX, TX_OR_CLEAR)  等 RX 标志
    └── kfifo_out(&cdc_rx_fifo, buf, to_read)                  从 kfifo 取数据
```

中断回调只做 kfifo 写入 + 置标志，不做协议解析。kfifo 是 SPSC 无锁环形缓冲区，详见 [[01_extracted/algorithm/kfifo-design]]。

## 内存布局

```c
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_read_buffer[1024];         // kfifo 底层缓冲
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_write_buffer[CDC_MAX_MPS]; // 发送缓冲
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_out_ep_buffer[CDC_MAX_MPS];// OUT 端点接收缓冲
```

| 宏 | F407 (FS) | H723 (HS) | 作用 |
|----|-----------|-----------|------|
| `USB_NOCACHE_RAM_SECTION` | `.ram` 段 | `.RAM_D2` 段 | 非 Cache 内存区，USB DMA 直接访问 |
| `USB_MEM_ALIGNX` | 4 字节对齐 | 4 字节对齐 | DMA 对齐要求 |

H723 有 D-Cache，如果缓冲区在普通 Cache 区域，CPU 写入后 DMA 读到的是旧数据。`.RAM_D2` 段指向 D2 域 SRAM，不被 Cache 缓存。F407 无 D-Cache，`.ram` 段只是统一宏定义。详见 [[01_extracted/hardware/c-board-resources]]。

## 链接

- 模块层封装：[[02_code_twin/modules/VISION/module_vision]]
- kfifo 原理：[[01_extracted/algorithm/kfifo-design]]
- 硬件资源：[[01_extracted/hardware/c-board-resources]]
