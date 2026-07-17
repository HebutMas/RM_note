# module_vision.c/h — 视觉通信模块

`modules/VISION/module_vision.{c,h}`

## 一句话

封装 USB 虚拟串口的收发逻辑，用帧头帧尾定界协议与上位机（迷你主机）交换视觉数据。MCU 发四元数和颜色，上位机回传目标角度和开火建议。

## 在整个系统中的位置

```
robot_control.c ─── 2ms 循环 ──── 模块_Vision_Send() ──→ cdc_acm_send() ──→ USB IN 端点
                  │                                              ↑
                  └── Module_Vision_Receive() ←── rx_packet ←── 视觉线程 ←── cdc_acm_recv() ←── USB OUT 端点
```

视觉模块内部有**两条执行路径**：
1. **控制循环路径**（robot_control 线程，2ms）：填 `SendPacket` → 调 `Module_Vision_Send` 发出去 → 调 `Module_Vision_Receive` 取最新接收包指针（非阻塞）
2. **接收线程路径**（vision_thread，阻塞）：`cdc_acm_recv` 永久阻塞等 USB 数据 → 帧头帧尾匹配 → `memcpy` 到 `rx_packet`

为什么要分两条路径？因为 `cdc_acm_recv` 会阻塞等数据，而 2ms 控制循环不能卡住。接收线程替控制循环"守着"USB，收到数据写入 `static rx_packet`，控制循环直接取指针，零等待。

## 初始化：Module_Vision_Init

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
        .name = "minipc",
        .beep_times = 10,
        .enable = VISION_OFFLINE_ENABLE,
        .timeout_ms = 100
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

初始化前置条件：`cdc_acm_init` 已在 `BSP_Init` 中完成（USB 硬件就绪），否则 `cdc_acm_recv` 永远收不到数据。

## 接收线程：vision_thread_entry

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
2. **`for` 循环扫描 buf** — 一次 `cdc_acm_recv` 可能收到多包数据粘在一起，或包头不在 buf[0] 位置，需要逐字节扫描
3. **帧头 `0xBB` + 帧尾 `0x5B` 匹配** — 先查帧头，再跳到 `i + sizeof(ReceivePacket) - 1` 查帧尾，两端匹配才认为是一个完整包
4. **`memcpy(&rx_packet, ...)`** — 从 buf 中拷出整包到全局 `rx_packet`，控制循环通过 `Module_Vision_Receive()` 取指针
5. **`Module_Offline_device_update`** — 收到有效数据，刷新离线检测定时器，表示"迷你主机还在线"
6. **`break`** — 只取第一个匹配的包，后面的丢弃。视觉数据是连续流，取最新即可

### 为什么要逐字节扫描

USB 传输不保证一次收到的数据正好是一个完整包。可能出现：

- **半包**：上一包的后半段 + 这一包的前半段在一次 recv 中
- **粘包**：两包数据在一次 recv 中
- **偏移**：帧头不在 buf[0]，前面有残数据

逐字节扫描帧头帧尾能处理所有这些情况。代价是 O(n) 遍历，但 buf 最大 64 字节，开销可忽略。

## 发送：Module_Vision_Send

```c
void Module_Vision_Send(SendPacket *packet, uint32_t timeout) {
    packet->header = SEND_HEADER;  // 0xAA
    packet->tail   = SEND_TAIL;    // 0x5A
    cdc_acm_send((uint8_t *)packet, sizeof(SendPacket), timeout);
}
```

模块层只做两件事：填帧头帧尾，然后透传给 BSP 层。`cdc_acm_send` 内部会等上次 TX 完成、拷贝到 NOCACHE 缓冲、启动 IN 端点，详见 [[02_code_twin/board/bsp/USB/bsp_usb_cdc]]。

`timeout` 由调用方传入，robot_control 传 `TX_NO_WAIT`——发不出去就跳过，不卡控制循环。视觉数据 2ms 一帧，丢一帧无所谓。

## 应用层怎么调（robot_control.c）

```c
// robot_control.c — robot_control_task，2ms 循环
send_packet.mode = 1 - chassis_upload_data.robot_color;  // 颜色取反给上位机
send_packet.q[0] = ins->q[0];  // 四元数 w
send_packet.q[1] = ins->q[1];  // x
send_packet.q[2] = ins->q[2];  // y
send_packet.q[3] = ins->q[3];  // z
Module_Vision_Send(&send_packet, TX_NO_WAIT);      // 发，不等待
receive_packet = Module_Vision_Receive();           // 取，不等待
```

| 操作 | 阻塞？ | 为什么 |
|------|--------|--------|
| `Module_Vision_Send` | 否（`TX_NO_WAIT`） | 发不出去就跳过，2ms 后再发下一帧 |
| `Module_Vision_Receive` | 否（直接返回指针） | 接收线程已经在后台把数据放好了 |

### `mode` 字段为什么取反

```c
send_packet.mode = 1 - chassis_upload_data.robot_color;
// robot_color: 0=红方, 1=蓝方
// mode:        1=红方, 0=蓝方
```

`robot_color` 从底盘板通过板间通信传过来（0=红, 1=蓝）。取反后传给上位机，具体含义取决于上位机的协议约定。

## 数据结构

两个 `#pragma pack(1)` 紧凑结构体，二进制直接收发：

### SendPacket（MCU → 上位机）

```c
#pragma pack(1)
typedef struct {
    uint8_t header;   /* 帧头 0xAA */
    uint8_t mode;     /* 颜色（1=红方, 0=蓝方） */
    float   q[4];     /* 四元数 w,x,y,z */
    uint8_t tail;     /* 帧尾 0x5A */
} SendPacket;         /* 共 19 字节 */
#pragma pack()
```

| 字段 | 字节 | 来源 | 用途 |
|------|------|------|------|
| `header` | 1 | `Module_Vision_Send` 自动填 | 帧定界 |
| `mode` | 1 | `1 - robot_color` | 告诉上位机自家颜色 |
| `q[0..3]` | 16 | `ins->q[0..3]` | IMU 姿态，上位机做坐标变换 |
| `tail` | 1 | `Module_Vision_Send` 自动填 | 帧定界 |

上位机拿到四元数后重建机器人姿态旋转矩阵，把视觉识别到的目标坐标从相机坐标系变换到机器人坐标系。

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

视觉数据是**连续流**——2ms 一帧，丢一帧等下一帧即可。加 CRC 会增加每帧的计算开销和通信字节，不值得。帧头帧尾已经能过滤绝大多数错误数据。

## 协议格式总结

| 方向 | 帧头 | 载荷 | 帧尾 | 总大小 |
|------|------|------|------|--------|
| MCU → 上位机 | `0xAA` | mode(1) + q[4](16) | `0x5A` | 19B |
| 上位机 → MCU | `0xBB` | found(1) + fire(1) + yaw(4) + pitch(4) + vx(4) + vy(4) + nav(1) | `0x5B` | 21B |

## 对外接口汇总

| 函数 | 阻塞？ | 调用者 | 说明 |
|------|--------|--------|------|
| `Module_Vision_Init()` | 否 | `MODULE_Init` | 注册离线检测 + 创建接收线程 |
| `Module_Vision_Send(packet, timeout)` | 看 timeout | `robot_control_task` | 填帧头帧尾 + `cdc_acm_send` |
| `Module_Vision_Receive()` | 否 | `robot_control_task` | 返回 `rx_packet` 指针 |
| `Module_Vision_Get_offline_state()` | 否 | 应用层 | 返回 `STATE_ONLINE` / `STATE_OFFLINE` |

## 前置条件

| 依赖 | 说明 |
|------|------|
| [[02_code_twin/board/bsp/USB/bsp_usb_cdc\|USB CDC]] | 底层传输，`cdc_acm_init` 须已完成 |
| [[02_code_twin/modules/OFFLINE/module_offline-c\|Offline 模块]] | 离线检测，100ms 超时 |
| [[01_extracted/algorithm/kfifo-design\|kfifo]] | USB 接收缓冲（在 BSP 层使用） |
| ThreadX | 接收线程 + 事件标志组（在 BSP 层使用） |
