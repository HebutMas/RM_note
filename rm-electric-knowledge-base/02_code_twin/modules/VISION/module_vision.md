# module_vision.c/h — 视觉通信模块

`modules/VISION/module_vision.{c,h}`

## 通信包结构

两个 `#pragma pack(1)` 紧凑结构体，二进制直接收发。先看清楚发什么、收什么，后面的收发逻辑才有意义。

### SendPacket（MCU → 上位机）

```c
#pragma pack(1)
typedef struct {
    uint8_t header;   /* 帧头 0xAA */
    uint8_t mode;     /* 模式 */
    float   q[4];     /* 四元数 w,x,y,z */
    uint8_t tail;     /* 帧尾 0x5A */
} SendPacket;         /* 共 19 字节 */
#pragma pack()
```

| 字段 | 字节 | 谁来填 | 用途 |
|------|------|--------|------|
| `header` | 1 | `Module_Vision_Send` 自动填 `0xAA` | 帧定界 |
| `mode` | 1 | 应用层填 | 传给上位机的模式标记 |
| `q[0..3]` | 16 | 应用层从 INS 模块取 `ins->q[0..3]` | IMU 姿态四元数，上位机做坐标变换 |
| `tail` | 1 | `Module_Vision_Send` 自动填 `0x5A` | 帧定界 |

### ReceivePacket（上位机 → MCU）

```c
#pragma pack(1)
typedef struct {
    uint8_t header;         /* 帧头 0xBB */
    uint8_t found;          /* 是否找到目标 */
    uint8_t fire_advice;    /* 开火建议 */
    float   target_yaw;     /* 目标偏航角 (rad) */
    float   target_pitch;   /* 目标俯仰角 (rad) */
    float   vx;             /* 目标速度 x (m/s) */
    float   vy;             /* 目标速度 y (m/s) */
    uint8_t nav_state;      /* 导航状态 */
    uint8_t tail;           /* 帧尾 0x5B */
} ReceivePacket;            /* 共 21 字节 */
#pragma pack()
```

| 字段 | 字节 | 用途 |
|------|------|------|
| `header` | 1 | 帧定界 |
| `found` | 1 | 上位机是否识别到装甲板 |
| `fire_advice` | 1 | 上位机建议是否开火 |
| `target_yaw` | 4 | 目标偏航角，直接给云台 PID 做目标值 |
| `target_pitch` | 4 | 目标俯仰角，同上 |
| `vx` | 4 | 目标线速度 x，预瞄/前馈补偿 |
| `vy` | 4 | 目标线速度 y |
| `nav_state` | 1 | 导航状态码，模式切换 |
| `tail` | 1 | 帧定界 |

### 为什么要 `#pragma pack(1)`

不加 `#pragma pack(1)`，编译器会在 `uint8_t` 后面插填充字节对齐到 4 字节边界。比如 `header`(1B) 后面会有 3 字节填充，然后才是 `float`。这样 `sizeof(SendPacket)` = 24 而不是 19，`memcpy` 取出来的数据和 USB 字节流对不上。

`#pragma pack(1)` 取消所有填充，结构体内存布局和通信字节流一一对应。

### 为什么没有 CRC

视觉数据是连续流——2ms 一帧，丢一帧等下一帧即可。加 CRC 增加每帧计算开销和通信字节，不值得。帧头帧尾已经能过滤绝大多数错误数据。

## 初始化

两条线，分别在 `BSP_Init` 和 `MODULE_Init` 中完成：

**BSP 层**（USB 硬件就绪）：

```c
// bsp_init.c — BSP_Init()
cdc_acm_init(0, USB_OTG_FS_PERIPH_BASE);   // F407
// cdc_acm_init(0, USB_OTG_HS_PERIPH_BASE); // H723
```

详见 [[02_code_twin/board/bsp/USB/bsp_usb_cdc]]。

**模块层**（视觉线程 + 离线检测）：

```c
// module_init.c — MODULE_Init()
#if MODULE_VISION
    Module_Vision_Init();
#endif
```

```c
// module_vision.c
void Module_Vision_Init(void) {
    // 1. 注册离线检测设备
    Offline_Init_config_t offlineconfig = {
        .name = "minipc", .beep_times = 10,
        .enable = VISION_OFFLINE_ENABLE, .timeout_ms = 100
    };
    offline_dev = Module_Offline_register(&offlineconfig);

    // 2. 创建接收线程
    tx_thread_create(&vision_thread, "vision_thread", vision_thread_entry, ...);
}
```

| 步骤 | 做了什么 | 为什么 |
|------|---------|--------|
| 离线检测注册 | 名字 `"minipc"`，超时 100ms，掉线响 10 声 | 100ms 没收到上位机数据 → 判定离线 |
| 创建接收线程 | 优先级 10，栈 1024 字节 | 独立线程阻塞等 USB 数据，不卡控制循环 |

## 接收线程：逐字节扫描

```c
static void vision_thread_entry(ULONG arg) {
    uint8_t  buf[64];
    uint32_t rx_len;

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
}
```

逐行解析：

1. **`cdc_acm_recv(buf, 64, &rx_len, TX_WAIT_FOREVER)`** — 阻塞等待 USB 数据。底层是 kfifo + 事件标志组，详见 [[02_code_twin/board/bsp/USB/bsp_usb_cdc]]
2. **`for` 循环逐字节扫描** — 一次 recv 可能收到多包粘在一起，或帧头不在 buf[0] 位置，需要逐字节扫描
3. **帧头 `0xBB` + 帧尾 `0x5B` 双重匹配** — 先查帧头，再跳到 `i + sizeof(ReceivePacket) - 1` 查帧尾，两端匹配才认为完整包
4. **`memcpy(&rx_packet, ...)`** — 拷出整包到全局 `rx_packet`，控制循环通过 `Module_Vision_Receive()` 取指针
5. **`Module_Offline_device_update`** — 收到有效数据，刷新离线检测定时器
6. **`break`** — 只取第一个匹配的包，视觉数据是连续流，取最新即可

### 为什么要逐字节扫描

USB 传输不保证一次收到的数据正好是一个完整包。可能出现：

- **半包**：上一包的后半段 + 这一包的前半段在一次 recv 中
- **粘包**：两包数据在一次 recv 中
- **偏移**：帧头不在 buf[0]，前面有残数据

逐字节扫描帧头帧尾能处理所有这些情况。buf 最大 64 字节，开销可忽略。

## 发送：Module_Vision_Send

```c
void Module_Vision_Send(SendPacket *packet, uint32_t timeout) {
    packet->header = SEND_HEADER;  // 0xAA
    packet->tail   = SEND_TAIL;    // 0x5A
    cdc_acm_send((uint8_t *)packet, sizeof(SendPacket), timeout);
}
```

模块层只做两件事：填帧头帧尾，然后透传给 BSP 层。`cdc_acm_send` 内部等上次 TX 完成、拷贝到 NOCACHE 缓冲、启动 IN 端点，详见 [[02_code_twin/board/bsp/USB/bsp_usb_cdc]]。

`timeout` 由调用方传入。robot_control 传 `TX_NO_WAIT`——发不出去就跳过，视觉数据 2ms 一帧，丢一帧无所谓。

## 对外接口

| 函数 | 阻塞？ | 调用者 |
|------|--------|--------|
| `Module_Vision_Init()` | 否 | `MODULE_Init` |
| `Module_Vision_Send(packet, timeout)` | 看 timeout | 应用层 |
| `Module_Vision_Receive()` | 否（返回指针） | 应用层 |
| `Module_Vision_Get_offline_state()` | 否 | 应用层 |

## 链接

- 底层传输：[[02_code_twin/board/bsp/USB/bsp_usb_cdc]]
- 离线检测：[[02_code_twin/modules/OFFLINE/module_offline-c]]
- 应用层调用（哨兵云台板）：[[02_code_twin/apps/sentry/gimbal_board/robot_control]]
