# bsp_usb_cdc — USB 虚拟串口（CDC ACM）

`board/bsp/USB/usbd_cdc_acm_user.{c,h}`

## 一句话

基于 CherryUSB 协议栈把 STM32 的 USB OTG 模拟成串口，当前唯一用途是视觉模块（MCU ↔ 迷你主机）通信。

## 在哪里调用：完整调用链

从应用层到硬件，虚拟串口被三层封装：

```
robot_control.c          ── 应用层，每 2ms 循环里填数据 + 发送 + 取接收
    │
    ├── Module_Vision_Send(&send_packet, TX_NO_WAIT)
    └── Module_Vision_Receive()
    │
module_vision.c          ── 模块层，加帧头帧尾 + 线程阻塞接收
    │
    ├── cdc_acm_send((uint8_t*)packet, sizeof(SendPacket), timeout)
    └── cdc_acm_recv(buf, sizeof(buf), &rx_len, TX_WAIT_FOREVER)
    │
usbd_cdc_acm_user.c      ── BSP层，事件标志同步 + kfifo缓冲 + USB端点读写
    │
    ├── usbd_ep_start_write(busid, CDC_IN_EP, ...)
    └── usbd_ep_start_read(busid, CDC_OUT_EP, ...)
    │
CherryUSB / HAL          ── 协议栈 + 寄存器操作（不展开）
```

### 应用层怎么调（robot_control.c）

```c
// robot_control.c — robot_control_task，2ms 循环
send_packet.mode = 1 - chassis_upload_data.robot_color;  // 0=红方, 1=蓝方
send_packet.q[0] = ins->q[0];  // 四元数 w
send_packet.q[1] = ins->q[1];  // x
send_packet.q[2] = ins->q[2];  // y
send_packet.q[3] = ins->q[3];  // z
Module_Vision_Send(&send_packet, TX_NO_WAIT);     // 发给上位机
receive_packet = Module_Vision_Receive();          // 取上位机回传
```

发送用 `TX_NO_WAIT`——发不出去就跳过，不阻塞控制循环。接收是**非阻塞取最新值**——`Module_Vision_Receive` 只返回内部 `rx_packet` 的指针，真正的阻塞接收在视觉模块自己的线程里。

### 模块层怎么封装（module_vision.c）

模块层做了两件 BSP 层不管的事：**加协议帧头帧尾**和**独立线程阻塞接收**。

发送封装：

```c
void Module_Vision_Send(SendPacket *packet, uint32_t timeout) {
    packet->header = SEND_HEADER;  // 0xAA
    packet->tail   = SEND_TAIL;    // 0x5A
    cdc_acm_send((uint8_t *)packet, sizeof(SendPacket), timeout);
}
```

接收封装——单独开一个线程，永久阻塞等 USB 数据：

```c
static void vision_thread_entry(ULONG arg) {
    uint8_t buf[64];
    uint32_t rx_len;
    while (1) {
        int ret = cdc_acm_recv(buf, sizeof(buf), &rx_len, TX_WAIT_FOREVER);
        if (ret <= 0) continue;
        // 在 buf 里扫描帧头 0xBB + 帧尾 0x5B
        for (uint32_t i = 0; i + sizeof(ReceivePacket) <= rx_len; i++) {
            if (buf[i] == RECV_HEADER && buf[i + sizeof(ReceivePacket) - 1] == RECV_TAIL) {
                memcpy(&rx_packet, &buf[i], sizeof(ReceivePacket));
                Module_Offline_device_update(offline_dev);
                break;
            }
        }
    }
}
```

为什么接收要单独开线程？因为 `cdc_acm_recv` 会阻塞等数据，而 `robot_control_task` 是 2ms 控制循环不能卡住。视觉线程 `TX_WAIT_FOREVER` 阻塞等数据，收到后写入 `static rx_packet`，控制循环通过 `Module_Vision_Receive()` 直接取指针，不需要等待。

## 封装成了什么样的函数

BSP 层只暴露 3 个函数：

| 函数 | 签名 | 作用 |
|------|------|------|
| `cdc_acm_init` | `(uint8_t busid, uintptr_t reg_base)` | 初始化 USB 设备、注册端点、创建 kfifo 和事件标志组 |
| `cdc_acm_send` | `(const uint8_t *data, uint32_t len, uint32_t timeout) → int` | 发送数据，等上次 TX 完成后启动 IN 端点写 |
| `cdc_acm_recv` | `(uint8_t *data, uint32_t buf_size, uint32_t *rx_len, uint32_t timeout) → int` | 接收数据，等 RX 标志后从 kfifo 取数据 |

模块层在 BSP 层之上又封装了一层，加上了协议和线程管理：

| 函数 | 封装了什么 |
|------|-----------|
| `Module_Vision_Init()` | 注册离线检测 + 创建接收线程 |
| `Module_Vision_Send(packet, timeout)` | 填帧头帧尾 + 调 `cdc_acm_send` |
| `Module_Vision_Receive()` | 返回 `rx_packet` 指针（非阻塞） |

## 怎么样初始化的

初始化分两条线，在 `BSP_Init` 和 `MODULE_Init` 中分别完成：

### 第一条线：BSP 层初始化（USB 硬件 + 协议栈）

```c
// bsp_init.c — BSP_Init()
BSP_DWT_Init(168);                                       // ① DWT 先初始化（后面 Delay 要用）
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);   // ② PA12 拉低
BSP_DWT_Delay(0.1);                                      //   保持 100ms
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);     // ③ 释放
BSP_DWT_Delay(0.1);                                      //   再等 100ms
cdc_acm_init(0, USB_OTG_FS_PERIPH_BASE);                 // ④ 初始化 CDC
```

PA12 是 USB D+ 线。拉低再释放 = 模拟 USB 拔插，强制主机重新枚举。否则 MCU 复位后 USB 线没拔，主机不会重新识别。

`cdc_acm_init` 内部做了 4 件事：

```c
void cdc_acm_init(uint8_t busid, uintptr_t reg_base) {
    kfifo_init(&cdc_rx_fifo, cdc_read_buffer, CDC_RX_FIFO_SIZE, 1);  // 1. 接收 kfifo
    tx_event_flags_create(&usb_event_flags, "usb_evt");               // 2. 事件标志组

    usbd_desc_register(busid, &cdc_descriptor);                       // 3. 描述符（VID/PID/字符串）
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf0)); //   通信接口
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf1)); //   数据接口
    usbd_add_endpoint(busid, &cdc_out_ep);                            //   OUT 端点（接收）
    usbd_add_endpoint(busid, &cdc_in_ep);                             //   IN 端点（发送）

    usbd_initialize(busid, reg_base, usbd_event_handler);            // 4. 启动 USB 设备
}
```

| 步骤 | 做了什么 | 为什么 |
|------|---------|--------|
| kfifo 初始化 | 1024 字节环形缓冲区 | USB 收到的数据暂存，应用层异步取 |
| 事件标志组 | RX/TX 两个标志 | USB 中断 ↔ 应用层任务同步 |
| 描述符注册 | VID=0xFFFF, PID=0xFFFF, 2 个接口 | CDC ACM 标准需要通信+数据两个接口 |
| 启动 USB | `usbd_initialize` | 触发枚举，主机识别为 COM 口 |

枚举成功后，`usbd_event_handler` 收到 `USBD_EVENT_CONFIGURED` 事件，置 TX 标志 + 启动第一次 OUT 端点读：

```c
case USBD_EVENT_CONFIGURED:
    tx_event_flags_set(&usb_event_flags, BSP_USB_EVENT_TX, TX_OR);  // TX 就绪
    usbd_ep_start_read(busid, CDC_OUT_EP, cdc_out_ep_buffer, CDC_MAX_MPS);  // 开始接收
    break;
```

板子差异：

| 板子 | reg_base | USB 模式 | 最大包长 `CDC_MAX_MPS` |
|------|----------|----------|------------------------|
| C 板（F407） | `USB_OTG_FS_PERIPH_BASE` | Full-Speed 12Mbps | 64 字节 |
| A 板（H723） | `USB_OTG_HS_PERIPH_BASE` | High-Speed 480Mbps | 512 字节 |

### 第二条线：模块层初始化（视觉任务 + 离线检测）

```c
// module_init.c — MODULE_Init()
Module_Vision_Init();
```

```c
// module_vision.c
void Module_Vision_Init(void) {
    // 1. 注册离线检测（100ms 超时，掉线响 10 声）
    Offline_Init_config_t offlineconfig = {
        .name = "minipc", .beep_times = 10,
        .enable = VISION_OFFLINE_ENABLE, .timeout_ms = 100
    };
    offline_dev = Module_Offline_register(&offlineconfig);

    // 2. 创建接收线程（阻塞等 USB 数据）
    tx_thread_create(&vision_thread, "vision_thread", vision_thread_entry, ...);
}
```

两条线的时序：`BSP_Init`（USB 硬件就绪）→ `MODULE_Init`（视觉线程启动，开始等数据）。

## 我们要传什么样的数据

两个 `#pragma pack(1)` 紧凑结构体，二进制直接收发：

### 发送：SendPacket（MCU → 上位机）

```c
#pragma pack(1)
typedef struct {
    uint8_t header;   /* 帧头 0xAA */
    uint8_t mode;     /* 0=红方, 1=蓝方（告诉上位机自家颜色） */
    float   q[4];     /* 四元数 w,x,y,z（IMU 姿态） */
    uint8_t tail;     /* 帧尾 0x5A */
} SendPacket;         /* 共 19 字节 */
#pragma pack()
```

robot_control 里怎么填：

| 字段 | 填什么 | 来源 |
|------|--------|------|
| `header` | `0xAA` | `Module_Vision_Send` 自动填 |
| `mode` | `1 - robot_color` | 板间通信收到的颜色（0红1蓝，取反给上位机） |
| `q[0..3]` | IMU 四元数 | `ins->q[0..3]`，来自 INS 模块 |
| `tail` | `0x5A` | `Module_Vision_Send` 自动填 |

上位机拿到四元数后，可以重建机器人的姿态旋转矩阵，用于视觉目标的坐标变换。

### 接收：ReceivePacket（上位机 → MCU）

```c
#pragma pack(1)
typedef struct {
    uint8_t header;        /* 帧头 0xBB */
    uint8_t found;         /* 是否找到目标 */
    uint8_t fire_advice;   /* 开火建议 */
    float   target_yaw;    /* 目标偏航角 (rad) */
    float   target_pitch;   /* 目标俯仰角 (rad) */
    float   vx;            /* 目标速度 x (m/s) */
    float   vy;            /* 目标速度 y (m/s) */
    uint8_t nav_state;     /* 导航状态 */
    uint8_t tail;          /* 帧尾 0x5B */
} ReceivePacket;           /* 共 21 字节 */
#pragma pack()
```

| 字段 | 含义 | 用途 |
|------|------|------|
| `found` | 上位机是否识别到装甲板 | 云台决定是否跟踪 |
| `fire_advice` | 上位机建议是否开火 | 发射机构控制 |
| `target_yaw/pitch` | 目标在机器人坐标系下的角度 | 直接给云台 PID 做目标值 |
| `vx/vy` | 目标线速度 | 预瞄/前馈补偿 |
| `nav_state` | 导航状态码 | 自主导航模式切换 |

### 为什么要 `#pragma pack(1)`

不加 `#pragma pack(1)`，编译器会在 `uint8_t` 后面插填充字节对齐到 4 字节边界。比如 `header`(1B) + `mode`(1B) 后面会有 2 字节填充，然后才是 `float q[4]`。这样结构体内存布局和通信字节流不一致，`memcpy` 取出来的数据就错了。

`#pragma pack(1)` 取消所有填充，结构体每个字段紧挨着，内存大小 = 各字段大小之和，和 USB 传输的字节流一一对应。

### 协议格式总结

| 方向 | 帧头 | 载荷 | 帧尾 | 总大小 | 定界方式 |
|------|------|------|------|--------|---------|
| MCU → 上位机 | `0xAA` | mode + q[4] | `0x5A` | 19B | 帧头+帧尾 |
| 上位机 → MCU | `0xBB` | found + fire + yaw + pitch + vx + vy + nav | `0x5B` | 21B | 帧头+帧尾 |

没有 CRC 校验——视觉数据是连续流，丢一帧等下一帧即可，不需要可靠性保证。

## 发送机制

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

USB 批量传输规则：如果发送的数据长度是最大包长的整数倍，主机不知道数据是否结束。需要补发一个 0 字节 ZLP 告诉主机"发完了"。只有 ZLP 完成后才置 TX 标志。

实际场景：`SendPacket` 19 字节，FS 最大包长 64 字节，19 % 64 ≠ 0，不会触发 ZLP。但如果一次发 64 字节（FS）或 512 字节（HS），就会触发。

## 接收机制

```
USB 主机发数据
    │
    ▼ (USB 中断)
usbd_cdc_acm_bulk_out(busid, ep, nbytes)
    ├── kfifo_in(&cdc_rx_fifo, cdc_out_ep_buffer, nbytes)   写入 kfifo
    ├── tx_event_flags_set(&usb_event_flags, RX, TX_OR)      置 RX 标志
    └── usbd_ep_start_read(busid, CDC_OUT_EP, ...)           重新启动接收
    │
    ▼ (视觉线程)
cdc_acm_recv(buf, 64, &rx_len, TX_WAIT_FOREVER)
    ├── tx_event_flags_get(&usb_event_flags, RX, TX_OR_CLEAR)  等 RX 标志
    └── kfifo_out(&cdc_rx_fifo, buf, to_read)                  从 kfifo 取数据
```

中断回调只做 kfifo 写入 + 置标志，不做协议解析。kfifo 是 SPSC 无锁环形缓冲区，详见 [[01_extracted/algorithm/kfifo-design]]。

## 内存布局

```c
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_read_buffer[1024];   // kfifo 底层缓冲
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_write_buffer[CDC_MAX_MPS];  // 发送缓冲
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_out_ep_buffer[CDC_MAX_MPS]; // OUT 端点接收缓冲
```

| 宏 | F407 (FS) | H723 (HS) | 作用 |
|----|-----------|-----------|------|
| `USB_NOCACHE_RAM_SECTION` | `.ram` 段 | `.RAM_D2` 段 | 非 Cache 内存区，USB DMA 直接访问 |
| `USB_MEM_ALIGNX` | 4 字节对齐 | 4 字节对齐 | DMA 对齐要求 |

H723 有 D-Cache，如果缓冲区在普通 Cache 区域，CPU 写入后 DMA 读到的是旧数据。`.RAM_D2` 段指向 D2 域 SRAM，不被 Cache 缓存。F407 无 D-Cache，`.ram` 段只是统一宏定义。详见 [[01_extracted/hardware/c-board-resources]]。

## 前置条件

| 依赖 | 说明 |
|------|------|
| [[02_code_twin/board/bsp/DWT/bsp_dwt\|DWT]] | 初始化前需要 DWT（PA12 复位用 `BSP_DWT_Delay`） |
| CherryUSB 协议栈 | `CherryUSB/` 目录，CMake 链接 |
| ThreadX | 事件标志组 + 视觉接收线程 |
| [[01_extracted/algorithm/kfifo-design\|kfifo]] | 接收缓冲区 |
| USB OTG 外设 | F407 FS / H723 HS，CubeMX 配置 |
