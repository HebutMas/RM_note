# RM Socket 编程详解

> 以 ros2_comm（宿主机 ↔ Docker UDP 桥）两端源码为教学案例，逐行讲解 Linux Socket API 在 RoboMaster 实战中的用法。
>
> 📘 **前置**：读者应了解 C++ 基础（指针、内存布局、多线程）
> 📘 **案例源码**：
>   - 宿主机端：`vision/mas_vision/rm_utils/ros2_comm/ros2_comm.{hpp,cpp}` — `Ros2_comm` 类
>   - Docker 端：`vision/mas_nav/sentry_utils/ros2_comm/src/ros2_comm.cpp` — `UdpBridgeNode` 类
> 📘 **关联**：[[MAS_Nav_决策与UDP通信]] — 协议格式 + 线程架构

---

## 目录

1. [概述：通信拓扑](#1-概述通信拓扑)
2. [socket() — 创建通信端点](#2-socket--创建通信端点)
3. [sockaddr_in — 地址结构的秘密](#3-sockaddr_in--地址结构的秘密)
4. [bind() — 占领端口](#4-bind--占领端口)
5. [fcntl() + O_NONBLOCK — 非阻塞 I/O](#5-fcntl--ononblock--非阻塞-io)
6. [recvfrom() — 接收数据的完整流程](#6-recvfrom--接收数据的完整流程)
7. [sendto() — 发送数据的完整流程](#7-sendto--发送数据的完整流程)
8. [网络字节序 — htons/htonl 的真相](#8-网络字节序--htonshtonl-的真相)
9. [二进制帧协议设计](#9-二进制帧协议设计)
10. [两种架构模式对比](#10-两种架构模式对比)
11. [线程安全与看门狗](#11-线程安全与看门狗)
12. [完整数据流追踪](#12-完整数据流追踪)
13. [常见坑与最佳实践](#13-常见坑与最佳实践)
14. [相关笔记](#14-相关笔记)

---

## 1. 概述：通信拓扑

### 1.1 两端源码

本项目 UDP 桥接有两端实现，运行在不同进程中：

```
┌─── 宿主机 (Windows/Linux) ───┐         ┌─── Docker 容器 (ROS 2) ───┐
│                               │         │                            │
│  Ros2_comm                    │  UDP    │  UdpBridgeNode             │
│  (mas_vision/rm_utils)       │◄═══════►│  (mas_nav/sentry_utils)   │
│                               │         │                            │
│  发送：裁判数据 (HP,弹药,进度)  │──0xAA──→│  接收 → /referee_data topic │
│                               │         │                            │
│  接收：导航控制指令 (vx,vy)    │←──0xBB──│  发送 ← /cmd_vel topic      │
│                               │         │                            │
│  监听端口：8889               │         │  监听端口：8888             │
│  目标端口：8888               │         │  目标端口：8889             │
└───────────────────────────────┘         └────────────────────────────┘
```

| 特性   | 宿主机 `Ros2_comm`                  | Docker `UdpBridgeNode`                             |
| ---- | -------------------------------- | -------------------------------------------------- |
| 源码位置 | `mas_vision/rm_utils/ros2_comm/` | `mas_nav/sentry_utils/ros2_comm/src/ros2_comm.cpp` |
| 类型   | 普通 C++ 类                         | ROS 2 Node                                         |
| 线程模型 | 同步（调用者控制频率）                      | 后台线程（500μs 自循环）                                    |
| 配置方式 | `loadConfig()` 读取 YAML           | 硬编码常量                                              |
| 发送内容 | `ROS2_SEND_PACKET`（裁判数据）         | `RawControlPacket`（控制指令 vx,vy）                     |
| 接收内容 | `ROS2_RECV_PACKET`（控制指令）         | `RawRefereePacket`（裁判数据）                           |

### 1.2 为什么用 Socket 而非 ROS Topic

ROS 2 的 DDS 无法穿越 Docker 边界（需复杂网络配置），而 UDP Socket 是操作系统原语，Docker 原生支持端口映射。

### 1.3 为什么是 UDP 而非 TCP

控制指令发送频率可达 **2000 Hz**（每 500μs 一帧）。TCP 的确认重传机制会引入不可控延迟。**丢失一帧的代价 < 引入延迟的代价**——实时控制的典型取舍。

---

## 2. socket() — 创建通信端点

两端代码完全相同。

### Ros2_comm（宿主机端）
```cpp
// ros2_comm.cpp — init()
m_sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
if (m_sock_fd < 0) 
{
    MAS_LOG_ERROR("Failed to create UDP socket: {}", strerror(errno));
    return -1;
}
```

### UdpBridgeNode（Docker 端）
```cpp
// ros2_comm.cpp:114
m_sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
if (m_sock_fd < 0)
{
    RCLCPP_ERROR(this->get_logger(), "Failed to create socket: %s", strerror(errno));
    return false;
}
```

### 参数详解

```cpp
int socket(int domain, int type, int protocol);
```

| 参数 | 传入值 | 含义 | 其他可选值 |
|------|--------|------|-----------|
| `domain` | `AF_INET` | IPv4 协议族 | `AF_INET6`, `AF_UNIX` |
| `type` | `SOCK_DGRAM` | 数据报（UDP） | `SOCK_STREAM` (TCP), `SOCK_RAW` |
| `protocol` | `0` | 自动选择 | `IPPROTO_UDP` 显式指定 |

### 返回值

- **成功**：非负整数 → **文件描述符**（fd），后续所有操作都用它
- **失败**：`-1`，`errno` 被设置为具体错误码

### 为什么初始化为 -1

两端构造函数的初始化列表都把 `m_sock_fd` 设为 `-1`：

```cpp
// Ros2_comm — ros2_comm.hpp:86
int m_sock_fd;

// Ros2_comm — ros2_comm.cpp 构造函数
Ros2_comm::Ros2_comm() : m_sock_fd(-1)
{
    m_recv_buffer.reserve(RECV_BUFFER_SIZE);
    memset(&m_server_addr, 0, sizeof(m_server_addr));
}

// UdpBridgeNode — ros2_comm.cpp:51
UdpBridgeNode() : Node("udp_bridge_node"), m_running(false), m_sock_fd(-1)
```

**设计原理**：fd `0`、`1`、`2` 分别被 stdin/stdout/stderr 占用。`-1` 作为"未初始化"标记，避免误关标准 I/O 和对无效 fd 调用 `close()`。

### 关闭时的 -1 检查

```cpp
// Ros2_comm::close()
void Ros2_comm::close()
{
    if (m_sock_fd >= 0)        // ← 依赖 -1 标记
    {
        ::close(m_sock_fd);
        m_sock_fd = -1;
    }
    m_recv_buffer.clear();
}

// UdpBridgeNode::close_udp() — ros2_comm.cpp:149-156
void close_udp()
{
    if (m_sock_fd >= 0)
    {
        close(m_sock_fd);
        m_sock_fd = -1;
    }
}
```

### errno

`errno` 是线程局部存储的全局变量。`strerror(errno)` 将其转为可读字符串：

| errno          | 常见原因                  |
| -------------- | --------------------- |
| `EACCES`       | 无权限（端口 < 1024 需 root） |
| `EAFNOSUPPORT` | 内核不支持指定协议族            |
| `EMFILE`       | 进程文件描述符耗尽（ulimit -n）  |

---

## 3. sockaddr_in — 地址结构的秘密

### 结构体内存布局

```cpp
struct sockaddr_in {
    sa_family_t    sin_family;  // 2 字节：地址族 (AF_INET = 2)
    in_port_t      sin_port;    // 2 字节：端口号 (网络字节序)
    struct in_addr sin_addr;    // 4 字节：IP 地址
    char           sin_zero[8]; // 8 字节：填充（必须为 0）
};
// sizeof(sockaddr_in) == 16 字节
```

```
字节:    0-1          2-3          4-7             8-15
      ┌──────────┬──────────┬──────────────┬────────────────┐
      │sin_family│ sin_port │  sin_addr    │   sin_zero[8]  │
      │ AF_INET  │  8889    │ 127.0.0.1    │   全 0 (填充)  │
      └──────────┴──────────┴──────────────┴────────────────┘
       2 字节      2 字节       4 字节           8 字节
```

### 两端源码对比

**Ros2_comm** — `m_server_addr` 是成员变量，指向 Docker 容器：
```cpp
// ros2_comm.hpp:87
sockaddr_in m_server_addr; // 目标地址 (Docker)

// ros2_comm.cpp — init()
memset(&m_server_addr, 0, sizeof(m_server_addr));
m_server_addr.sin_family = AF_INET;
m_server_addr.sin_port = htons(s_server_port);     // 8888 (Docker 监听端口)
inet_pton(AF_INET, s_server_ip.c_str(), &m_server_addr.sin_addr); // "127.0.0.1"
```

**UdpBridgeNode** — `m_target_addr` 是成员变量，指向宿主机：
```cpp
// ros2_comm.cpp:109
sockaddr_in m_target_addr; // 宿主机地址

// ros2_comm.cpp:140-143
memset(&m_target_addr, 0, sizeof(m_target_addr));
m_target_addr.sin_family = AF_INET;
m_target_addr.sin_port = htons(TARGET_PORT);        // 8889 (宿主机监听端口)
inet_pton(AF_INET, TARGET_IP, &m_target_addr.sin_addr); // "127.0.0.1"
```

### 为什么必须 memset 为 0

```cpp
memset(&m_server_addr, 0, sizeof(m_server_addr));  // ← 必须！
```

`sockaddr_in` 的 `sin_zero[8]` 是 padding 字段。如果不清零：

1. `bind()` 在内核中校验 `sin_zero` → 非零值导致 `EINVAL`
2. 不同内核版本行为不一致（3.x 不检查，4.x+ 严格检查）
3. Linux socket 编程最常见的**不确定行为**来源

### inet_pton — IP 字符串 → 二进制

```cpp
// Ros2_comm: 返回值被检查
if (inet_pton(AF_INET, s_server_ip.c_str(), &m_server_addr.sin_addr) <= 0)
{
    MAS_LOG_ERROR("Invalid server IP: %s", s_server_ip.c_str());
    close();
    return -1;
}

// UdpBridgeNode: 未检查返回值（127.0.0.1 是固定的）
inet_pton(AF_INET, TARGET_IP, &m_target_addr.sin_addr);
```

- `inet_pton` = **p**resentation **to** **n**etwork
- 反函数：`inet_ntop`
- **不要用** `inet_addr()`——返回 `-1` 既可能是 `255.255.255.255` 也可能是错误，有歧义

---

## 4. bind() — 占领端口

### 两端对比

**Ros2_comm** — 绑定宿主机端口 8889：

```cpp
// ros2_comm.cpp — init()
sockaddr_in local_addr;
memset(&local_addr, 0, sizeof(local_addr));
local_addr.sin_family = AF_INET;
local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
local_addr.sin_port = htons(s_local_port);   // 8889 — 从 YAML 读取

if (bind(m_sock_fd, (sockaddr*)&local_addr, sizeof(local_addr)) < 0)
{
    MAS_LOG_ERROR("Failed to bind UDP port {}: {}", s_local_port, strerror(errno));
    close();
    return -1;
}
```

**UdpBridgeNode** — 绑定 Docker 端口 8888：

```cpp
// ros2_comm.cpp:126-137
sockaddr_in local_addr;
memset(&local_addr, 0, sizeof(local_addr));
local_addr.sin_family = AF_INET;
local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
local_addr.sin_port = htons(LOCAL_PORT);      // 8888 — 硬编码

if (bind(m_sock_fd, (sockaddr*)&local_addr, sizeof(local_addr)) < 0)
{
    RCLCPP_ERROR(this->get_logger(), "Failed to bind port %d: %s", LOCAL_PORT, strerror(errno));
    close(m_sock_fd);
    return false;
}
```

### bind() 做了什么

```
没有 bind 的 socket：         bind 后的 socket：
  可以 sendto()                可以 recvfrom() ← 内核投递数据到此端口
  无法 recvfrom()              也可以 sendto()
  （源端口随机分配）
```

### INADDR_ANY 的含义

```
INADDR_ANY = 0.0.0.0

含义：绑定到所有可用网络接口
  127.0.0.1    ← loopback
  192.168.x.x  ← 物理网卡
  172.17.x.x   ← Docker 桥接网络
```

### 端口配置

| 端口 | Ros2_comm (宿主机) | UdpBridgeNode (Docker) |
|------|-------------------|----------------------|
| 本地监听 | **8889** | **8888** |
| 发送目标 | 8888 (Docker) | 8889 (宿主机) |
| 配置方式 | YAML 文件可配 | 硬编码常量 |

### 绑定失败原因

| errno | 含义 | 场景 |
|-------|------|------|
| `EADDRINUSE` | 端口已被占用 | 另一实例已在运行 |
| `EACCES` | 无权限 | 端口 < 1024 需 root |
| `EADDRNOTAVAIL` | 地址不可用 | 绑定了不属于本机的 IP |

---

## 5. fcntl() + O_NONBLOCK — 非阻塞 I/O

两端代码完全相同：

```cpp
// Ros2_comm — init()
int flags = fcntl(m_sock_fd, F_GETFL, 0);
fcntl(m_sock_fd, F_SETFL, flags | O_NONBLOCK);

// UdpBridgeNode — ros2_comm.cpp:122-123
int flags = fcntl(m_sock_fd, F_GETFL, 0);
fcntl(m_sock_fd, F_SETFL, flags | O_NONBLOCK);
```

### 阻塞 vs 非阻塞

```
阻塞模式 (默认)：
  recvfrom() 调用 ──────────────────> 返回
                 挂起（CPU 不消耗）
                 直到数据到达或错误

非阻塞模式 (O_NONBLOCK)：
  recvfrom() 调用 → 立即返回
    ├─ 有数据 → 返回数据
    └─ 无数据 → 返回 -1, errno = EAGAIN/EWOULDBLOCK
```

### 为什么必须非阻塞

在 `UdpBridgeNode` 中，如果 recvfrom 阻塞：

```
阻塞 recvfrom 等裁判数据 → 裁判系统可能几秒不发 → 线程卡住
  → send_to_host() 不执行 → 看门狗不触发 → 机器人失控
```

在 `Ros2_comm` 中，`recv()` 由外部调用者控制节奏——同样不能卡住调用者。

**收发解耦**是实时通信的基本要求。

### 标志位的读-改-写原理

```cpp
int flags = fcntl(m_sock_fd, F_GETFL, 0);        // ① 读取
fcntl(m_sock_fd, F_SETFL, flags | O_NONBLOCK);   // ② 设置（位或）
```

不能直接 `fcntl(fd, F_SETFL, O_NONBLOCK)`——这会**覆盖**所有其他标志（如 `O_RDWR`、`O_CREAT` 等）。位或保留原有标志，只新增 `O_NONBLOCK`。

---

## 6. recvfrom() — 接收数据的完整流程

### 函数签名

```cpp
ssize_t recvfrom(
    int sockfd,
    void *buf,
    size_t len,
    int flags,
    struct sockaddr *src_addr,   // [出参] 发送方地址
    socklen_t *addrlen           // [入出参] 地址结构体大小
);
```

### Ros2_comm::recv() — 返回数据给调用者

```cpp
// ros2_comm.cpp — recv()
int Ros2_comm::recv(ROS2_RECV_PACKET& out_pkt)
{
    if (m_sock_fd < 0) return -1;

    uint8_t temp[RECV_BUFFER_SIZE];          // 栈上缓冲区，1024 字节
    sockaddr_in from_addr;
    socklen_t from_len = sizeof(from_addr);

    int n = recvfrom(m_sock_fd, temp, sizeof(temp), 0,
                     (sockaddr*)&from_addr, &from_len);

    if (n > 0)
    {
        // ① 最小长度检查（至少 header+len+tail = 3 字节）
        if (n < 3) return 0;

        // ② 帧头校验
        if (temp[0] != RECV_FRAME_HEADER) return 0;

        uint8_t data_len = temp[1];
        size_t total_len = 2 + data_len + 1;    // header(1) + len(1) + data + tail(1)

        // ③ 完整性检查
        if (n < static_cast<int>(total_len)) return 0;

        // ④ 帧尾校验
        if (temp[2 + data_len] != RECV_FRAME_TAIL) return 0;

        // ⑤ 长度匹配 + 反序列化
        if (data_len == sizeof(ROS2_RECV_PACKET))
        {
            memcpy(&out_pkt, &temp[2], sizeof(ROS2_RECV_PACKET));
            return static_cast<int>(data_len);  // 返回数据长度
        }
    }
    else if (n < 0)
    {
        // ⑥ 区分"无数据"和"真实错误"
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return 0;    // 非阻塞：无数据
        }
        return -1;       // 真实错误
    }

    return 0;
}
```

### UdpBridgeNode — 接收后发布 ROS 消息

```cpp
// ros2_comm.cpp:197-231
uint8_t buffer[1024];
sockaddr_in from_addr;
socklen_t from_len = sizeof(from_addr);

int n = recvfrom(m_sock_fd, buffer, sizeof(buffer), 0,
                 (sockaddr*)&from_addr, &from_len);

if (n > 0)
{
    if (n >= 3 && buffer[0] == SEND_FRAME_HEADER)
    {
        uint8_t data_len = buffer[1];
        size_t total_len = 2 + data_len + 1;

        if (n >= static_cast<int>(total_len) &&
            buffer[2 + data_len] == SEND_FRAME_TAIL)
        {
            if (data_len == sizeof(RawRefereePacket))
            {
                RawRefereePacket raw;
                memcpy(&raw, &buffer[2], sizeof(raw));

                auto msg = ros2_comm::msg::RefereeData();
                msg.projectile_allowance_17mm = raw.projectile_allowance_17mm;
                msg.power_management_shooter_output = raw.power_management_shooter_output;
                msg.current_hp = raw.current_hp;
                msg.outpost_hp = raw.outpost_HP;
                msg.base_hp = raw.base_HP;
                msg.game_progess = raw.game_progess;

                pub_referee_->publish(msg);
            }
        }
    }
}
```

### 对比

| 方面 | Ros2_comm::recv() | UdpBridgeNode |
|------|------------------|---------------|
| 缓冲区 | `uint8_t temp[1024]` 栈数组 | `uint8_t buffer[1024]` 栈数组 |
| 返回值语义 | `>0`=数据长度, `0`=无数据, `-1`=错误 | 不检查 errno |
| 输出方式 | 出参 `out_pkt` | 发布 ROS `referee_data` topic |
| EAGAIN 处理 | 明确区分并返回 0 | 未显式处理（靠 n<=0 跳过） |

### 返回值的四种情况

| 返回值 | 含义 | 后续动作 |
|--------|------|---------|
| `n > 0` | 收到 n 字节数据 | 解析帧 |
| `n == 0` | UDP 空包（罕见但合法） | 忽略 |
| `n == -1, errno == EAGAIN` | 无数据可读（非阻塞） | 跳过 |
| `n == -1, errno != EAGAIN` | 真实错误 | 记录日志 |

### 为什么缓冲区是 1024 字节

UDP 理论最大 datagram 65507 字节，但本项目帧最大 13 字节，1024 足够。栈数组（非 `malloc`）：
- 固定大小，无动态分配
- 无内存泄漏风险
- 缓存友好（栈内存通常在 L1 cache）
- 典型栈空间 8MB，1024 字节微不足道

---

## 7. sendto() — 发送数据的完整流程

### 函数签名

```cpp
ssize_t sendto(
    int sockfd,
    const void *buf,
    size_t len,
    int flags,
    const struct sockaddr *dest_addr,
    socklen_t addrlen
);
```

### Ros2_comm::send() — **检查返回值**

```cpp
// ros2_comm.cpp — send()
int Ros2_comm::send(const ROS2_SEND_PACKET& pkt)
{
    if (m_sock_fd < 0) return -1;

    // ① 构建帧：HEADER + LEN + DATA + TAIL
    uint8_t frame[sizeof(ROS2_SEND_PACKET) + 3];       // 10 + 3 = 13 字节
    frame[0] = SEND_FRAME_HEADER;                       // 0xAA
    frame[1] = static_cast<uint8_t>(sizeof(ROS2_SEND_PACKET)); // 10
    memcpy(frame + 2, &pkt, sizeof(ROS2_SEND_PACKET)); // 裁判数据
    frame[2 + sizeof(ROS2_SEND_PACKET)] = SEND_FRAME_TAIL;     // 0x5A

    // ② 发送到 Docker
    int n = sendto(m_sock_fd, frame, sizeof(frame), 0,
                   (sockaddr*)&m_server_addr, sizeof(m_server_addr));

    if (n < 0)
    {
        return -1;     // ← 检查了返回值！
    }
    return 0;
}
```

### UdpBridgeNode::send_to_host() — **未检查返回值**

```cpp
// ros2_comm.cpp:172-186
bool send_to_host(const RawControlPacket& pkt)
{
    if (m_sock_fd < 0) return false;

    uint8_t frame[sizeof(RawControlPacket) + 3];        // 9 + 3 = 12 字节
    frame[0] = RECV_FRAME_HEADER;                        // 0xBB
    frame[1] = static_cast<uint8_t>(sizeof(RawControlPacket)); // 9
    memcpy(frame + 2, &pkt, sizeof(RawControlPacket));  // 控制指令
    frame[2 + sizeof(RawControlPacket)] = RECV_FRAME_TAIL;      // 0x5B

    sendto(m_sock_fd, frame, sizeof(frame), 0,
           (sockaddr*)&m_target_addr, sizeof(m_target_addr));
    // ↑ 返回值未赋值给变量——丢弃了

    return true;
}
```

### 帧布局

```
Ros2_comm 发送帧 (13 字节) — 裁判数据，宿主机 → Docker：
  索引:  [0]     [1]      [2..11]          [12]
       ┌──────┬──────┬────────────────┬──────┐
       │ 0xAA │  10  │ROS2_SEND_PACKET│ 0x5A │
       │ 帧头  │ 长度  │  (10 字节)     │ 帧尾  │
       └──────┴──────┴────────────────┴──────┘

UdpBridgeNode 发送帧 (12 字节) — 控制指令，Docker → 宿主机：
  索引:  [0]     [1]      [2..10]            [11]
       ┌──────┬──────┬────────────────┬──────┐
       │ 0xBB │  9   │ RawControlPacket│ 0x5B │
       │ 帧头  │ 长度  │   (9 字节)      │ 帧尾  │
       └──────┴──────┴────────────────┴──────┘
```

### 返回值检查的差异分析

`Ros2_comm::send()` **检查** sendto 返回值，`UdpBridgeNode::send_to_host()` **不检查**。这反映了两端不同的设计约束：

| | Ros2_comm | UdpBridgeNode |
|---|---|---|
| 调用频率 | 取决于外部调用者（~10-100Hz） | 2000Hz（500μs 循环） |
| 丢帧后果 | 裁判数据丢失可接受 | 控制指令丢失可接受（下一帧 500μs 后到） |
| 错误处理 | 返回 -1 给调用者决策 | 无处理（loopback 几乎不丢包） |

---

## 8. 网络字节序 — htons/htonl 的真相

### 大端和小端

```
十六进制 0x12345678 在内存中的两种排列：

大端 (网络字节序)：  低地址 ← 0x12  0x34  0x56  0x78 → 高地址
小端 (x86/ARM)：     低地址 ← 0x78  0x56  0x34  0x12 → 高地址

网络协议标准 = 大端
x86/ARM (RM 硬件) = 小端
```

### 四个关键函数

| 函数 | 全称 | 用途 |
|------|------|------|
| `htons(x)` | **H**ost **to** **N**etwork **S**hort | 16 位（端口号） |
| `htonl(x)` | **H**ost **to** **N**etwork **L**ong | 32 位（IP 地址） |
| `ntohs(x)` | **N**etwork **to** **H**ost **S**hort | 反向 |
| `ntohl(x)` | **N**etwork **to** **H**ost **L**ong | 反向 |

### 项目中的实际用法

两端完全相同：

```cpp
local_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // 0.0.0.0
local_addr.sin_port = htons(s_local_port);        // 8889 或 8888
```

### 端口转换原理

```
端口 8888 = 0x22B8

在 x86（小端）上：
  内存布局:    [0xB8][0x22]  ← 低位在前

htons(0x22B8):
  操作: 交换两个字节
  结果: 0xB822
  内存: [0x22][0xB8]  ← 现在高位在前

发送到网络: 0x22 0xB8
接收端按大端解析: 0x22 << 8 | 0xB8 = 0x22B8 = 8888 ✓
```

**一句话**：数据进出 `sockaddr_in` 时，端口和 IP 必须用 `htons`/`htonl` 转换。

---

## 9. 二进制帧协议设计

### 协议帧格式

```
定界帧: [HEADER(1)] [LEN(1)] [PAYLOAD(N)] [TAIL(1)]

发送帧（宿主机 → Docker，裁判数据）：
  HEADER: 0xAA
  LEN:    sizeof(ROS2_SEND_PACKET) = 10
  DATA:   ROS2_SEND_PACKET (10 字节，packed)
  TAIL:   0x5A
  总长:   1 + 1 + 10 + 1 = 13 字节

接收帧（Docker → 宿主机，控制指令）：
  HEADER: 0xBB
  LEN:    sizeof(ROS2_RECV_PACKET) = 9
  DATA:   ROS2_RECV_PACKET (9 字节，packed)
  TAIL:   0x5B
  总长:   1 + 1 + 9 + 1 = 12 字节
```

### 帧头/帧尾的选择

```
0xAA = 1010 1010  ← 高-低交替（易识别）
0xBB = 1011 1011  ← 与 0xAA 区别足够大
0x5A = 0101 1010  ← 与 0xAA 汉明距离 = 4
0x5B = 0101 1011  ← 与 0x5A 差 1 bit

选择原则：与数据冲突概率低、易于识别、帧头帧尾间汉明距离大
```

### 帧解析校验链（Ros2_comm::recv 为例）

```
帧头 ≠ 0xBB → 丢弃
  ↓
n < 3 → 丢弃（帧过短）
  ↓
n < 2 + data_len + 1 → 丢弃（帧不完整）
  ↓
buffer[2 + data_len] ≠ 0x5B → 丢弃（帧尾不匹配）
  ↓
data_len ≠ sizeof(ROS2_RECV_PACKET) → 丢弃（长度不对）
  ↓
memcpy → 反序列化成功
```

多层校验链：帧头 → 最小长度 → 完整性 → 帧尾 → 数据长度 → 反序列化。每层一个安全网。

### `__attribute__((packed))` 的作用

两端结构体完全对称，都用了 packed：

```cpp
// ros2_comm.hpp:9-17
struct __attribute__((packed)) ROS2_SEND_PACKET
{
    uint16_t projectile_allowance_17mm;  // 2 字节
    uint8_t  power_management_shooter_output; // 1 字节
    uint16_t current_hp;                 // 2 字节
    uint16_t outpost_HP;                 // 2 字节
    uint16_t base_HP;                    // 2 字节
    uint8_t  game_progess;               // 1 字节
};
// 有 packed:   2+1+2+2+2+1 = 10 字节
// 没有 packed: 2+1+1pad+2+2+2+1+1pad = 12 字节（编译器对齐到 uint16_t 边界）

// ros2_comm.hpp:19-24
struct __attribute__((packed)) ROS2_RECV_PACKET
{
    float vx;           // 4 字节
    float vy;           // 4 字节
    uint8_t nav_state;  // 1 字节
};
// 有 packed:   4+4+1 = 9 字节
```

**关键教训**：跨网络传输的结构体必须 `packed` + `sizeof()` 验证。否则编译器 padding 在不同平台下不一致，接收端解析错位，所有字段值乱掉。

### 为什么用 memcpy 而非指针强转

```cpp
// ❌ 危险 — 违反 strict aliasing + 对齐问题
ROS2_RECV_PACKET* pkt = (ROS2_RECV_PACKET*)&temp[2];
// ARM 上 temp[2] 未对齐到 float 边界 → SIGBUS

// ✅ 安全
ROS2_RECV_PACKET pkt;
memcpy(&pkt, &temp[2], sizeof(pkt));
// memcpy 被编译器内联，自动处理对齐
```

---

## 10. 两种架构模式对比

### Ros2_comm：同步 API 模式

```
调用者线程                           Ros2_comm
     │                                   │
     ├─ loadConfig("config.yaml") ──────→│ 读 YAML，设置静态配置
     ├─ init() ─────────────────────────→│ socket() + bind() + fcntl()
     │                                   │
     ├─ send(game_data) ────────────────→│ 构建帧 → sendto → Docker:8888
     │                                   │
     ├─ recv(ctrl_pkt) ─────────────────→│ recvfrom (非阻塞)
     │   ← 返回数据或 0(无数据)           │
     │                                   │
     └─ close() ────────────────────────→│ ::close(fd) + clear buffer
```

- **无内部状态机**：send/recv 是纯函数式调用
- **调用者控制节奏**：外部决定何时收发，无需后台线程
- **返回值语义清晰**：`>0`=数据，`0`=无数据，`-1`=错误
- **配置灵活**：YAML 驱动，端口/IP 可改

### UdpBridgeNode：后台线程模式

```
main 线程:                              comm_thread (后台):
  rclcpp::spin()                          while(m_running) {
    └─ cmd_vel_callback() ← ROS 回调         recvfrom() → 收裁判数据
         ↓                                   ↓
         m_latest_ctrl = ...                send_to_host() → 发控制指令
         m_last_cmd_time = now()             ↓
                                           sleep_for(500μs)
                                         }
```

- **独立线程**：通信与 ROS spin 解耦
- **看门狗**：1 秒无 `/cmd_vel` 自动急停
- **固定频率**：500μs 周期（2000Hz），确保控制指令及时送出
- **硬编码配置**：端口/IP 编译期确定，比赛环境不需要灵活性

### 模式选择指南

| 场景 | 推荐模式 |
|------|---------|
| 需要内部定时循环 | UdpBridgeNode（后台线程） |
| 外部已有主循环，只需 IO | Ros2_comm（同步 API） |
| 需要超时/看门狗 | UdpBridgeNode |
| 需要灵活配置 | Ros2_comm（YAML） |
| 需要 ROS 集成 | UdpBridgeNode |

---

## 11. 线程安全与看门狗

### UdpBridgeNode 的线程模型

```
main 线程 (ROS spin):                   comm_thread (后台):
  cmd_vel_callback()                       while(m_running) {
    lock(m_ctrl_mutex)                       lock(m_ctrl_mutex)
    m_latest_ctrl.vx = ...                   now = this->now()
    m_latest_ctrl.vy = ...                   if elapsed > 1s → 急停
    m_last_cmd_time = now()                  else → 使用 m_latest_ctrl
    unlock                                   send_to_host(pkt)
                                           unlock
                                           sleep(500μs)
```

**共享变量** `m_latest_ctrl` + `m_last_cmd_time` 被两个线程访问，用 `std::mutex` 保护。

### 互斥锁实现

```cpp
// ros2_comm.cpp:101
std::mutex m_ctrl_mutex;

// 写操作 — ros2_comm.cpp:159-169
void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_ctrl_mutex);  // 构造 lock
    m_latest_ctrl.vx = msg->linear.x;
    m_latest_ctrl.vy = msg->linear.y;
    if (msg->linear.x == 0 && msg->linear.y == 0) {
        m_latest_ctrl.nav_state = 0;
    }
    m_latest_ctrl.nav_state = 1;
    m_last_cmd_time = this->now();
}  // 析构 unlock（RAII）

// 读操作 — ros2_comm.cpp:234-260
{
    std::lock_guard<std::mutex> lock(m_ctrl_mutex);
    rclcpp::Time now = this->now();
    double elapsed = (now - m_last_cmd_time).seconds();

    RawControlPacket pkt_to_send;
    if (elapsed > CMD_TIMEOUT_SEC) { /* 急停 */ }
    else { pkt_to_send = m_latest_ctrl; }

    send_to_host(pkt_to_send);  // 在锁内发送 — 临界区包含 I/O
}
```

### 为什么用 lock_guard 而非手动 lock/unlock

```cpp
// ❌ 异常不安全
m_ctrl_mutex.lock();
// 如果抛异常 → 锁永不释放 → 死锁
m_ctrl_mutex.unlock();

// ✅ 异常安全 (RAII)
std::lock_guard<std::mutex> lock(m_ctrl_mutex);
// 无论是否抛异常，lock_guard 析构函数都会 unlock
```

### 看门狗

```cpp
// ros2_comm.cpp:105
static constexpr double CMD_TIMEOUT_SEC = 1.0;

// 超时检查 — ros2_comm.cpp:237-257
rclcpp::Time now = this->now();
double elapsed = (now - m_last_cmd_time).seconds();

if (elapsed > CMD_TIMEOUT_SEC) {
    pkt_to_send.vx = 0.0f;     // 急停
    pkt_to_send.vy = 0.0f;
    pkt_to_send.nav_state = 0;
    if (!timeout_warning_printed) {
        RCLCPP_WARN(this->get_logger(), "/cmd_vel timeout, stopping!");
        timeout_warning_printed = true;
    }
} else {
    pkt_to_send = m_latest_ctrl;
    timeout_warning_printed = false;
}
```

**设计意图**：Nav2 崩溃 → `/cmd_vel` 停止 → 1 秒后自动急停。保护窗口 ≈ 1000ms，机器人最多滑行 3m（以最高 3m/s 计）。

`Ros2_comm` 没有看门狗——它只是 IO 层，超时安全由上层调用者负责。

---

## 12. 完整数据流追踪

### 12.1 接收路径：Docker → 宿主机（控制指令）

```
① Nav2 MPPI 控制器
    │ publish /cmd_vel (40Hz)
    │ Twist: linear.x=1.5, linear.y=0.3
    v
② UdpBridgeNode::cmd_vel_callback()
    │ lock → m_latest_ctrl = {vx:1.5, vy:0.3, nav_state:1} → unlock
    v
③ UdpBridgeNode::comm_thread_loop() — 发送阶段
    │ lock → 构建 RawControlPacket → send_to_host(frame, 12 bytes)
    │ sendto(fd, [0xBB][9][vx,vy,nav_state][0x5B], ...) → 127.0.0.1:8889
    v
④ Linux 内核网络栈 (loopback)
    v
⑤ Ros2_comm::recv()
    │ recvfrom (非阻塞) → 收到 12 字节
    │ 校验: 帧头=0xBB → 帧尾=0x5B → data_len=9=sizeof(ROS2_RECV_PACKET)
    │ memcpy → out_pkt = {vx:1.5, vy:0.3, nav_state:1}
    │ 返回 9 给调用者
    v
⑥ 宿主机上层 → 电机控制器
```

### 12.2 发送路径：宿主机 → Docker（裁判数据）

```
① 宿主机上层
    │ 填充 ROS2_SEND_PACKET
    │ {projectile_allowance:50, current_hp:456, game_progess:4, ...}
    v
② Ros2_comm::send(pkt)
    │ 构建帧: [0xAA][10][10_bytes_data][0x5A]
    │ sendto(fd, frame, 13, ...) → 127.0.0.1:8888
    │ 返回值被检查: n < 0 → return -1
    v
③ Linux 内核网络栈 (loopback)
    v
④ UdpBridgeNode::comm_thread_loop() — 接收阶段
    │ recvfrom → 收到 13 字节
    │ 校验: 帧头=0xAA → 帧尾=0x5A → data_len=10=sizeof(RawRefereePacket)
    │ memcpy → RawRefereePacket raw
    │ 转换 → RefereeData ROS msg
    │ pub_referee_->publish(msg)
    v
⑤ /referee_data topic
    v
⑥ rm_decision 节点
    │ 回调更新 FSM 状态
    │ HP<200 → RESTORE, else → ATTACK
```

### 12.3 时序约束

| 环节 | 周期/延迟 | 备注 |
|------|----------|------|
| Nav2 → /cmd_vel | 25ms (40Hz) | 控制周期 |
| UdpBridgeNode 发送周期 | **500μs (2000Hz)** | 远高于控制周期 |
| UDP loopback 延迟 | <0.1ms | 内核内存拷贝 |
| Ros2_comm recv 调用频率 | 取决于调用者 | 通常 10-100Hz |
| 看门狗触发延迟 | 1000ms | UdpBridgeNode 独有 |

---

## 13. 常见坑与最佳实践

### 坑 1：不 memset sockaddr_in

```cpp
// ❌ sin_zero 是栈上随机数据
sockaddr_in addr;
addr.sin_family = AF_INET;
addr.sin_port = htons(8888);
bind(fd, &addr, sizeof(addr));  // 可能 EINVAL

// ✅ 先归零
sockaddr_in addr;
memset(&addr, 0, sizeof(addr));
addr.sin_family = AF_INET;
addr.sin_port = htons(8888);
```

### 坑 2：忘记网络字节序

```cpp
// ❌ 端口在小端机器上变成 47138
addr.sin_port = 8888;

// ✅ htons 转换
addr.sin_port = htons(8888);
```

### 坑 3：阻塞 recvfrom 卡线程

```cpp
// ❌ 等不到数据就卡死，send 永远不执行
while (running) {
    recvfrom(fd, buf, 1024, 0, ...);
    send_data();
}

// ✅ 非阻塞
fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
while (running) {
    int n = recvfrom(fd, buf, 1024, 0, ...);
    if (n > 0) { /* 处理 */ }
    send_data();
}
```

### 坑 4：未区分 EAGAIN 和真实错误

```cpp
// ❌ 把"无数据"当错误处理
if (n < 0) {
    return -1;  // EAGAIN 也被当作错误
}

// ✅ 区分
if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return 0;  // 非阻塞：无数据
    }
    return -1;     // 真实错误
}
```

`Ros2_comm::recv()` 做了这个区分，`UdpBridgeNode` 未显式做。

### 坑 5：跨网络传输非 packed 结构体

```cpp
// ❌ sizeof 在不同编译器/平台下可能不同
struct Data { uint8_t a; uint16_t b; };  // 可能是 3 或 4
sendto(fd, &d, sizeof(d), ...);

// ✅ packed + sizeof 验证
struct __attribute__((packed)) Data { uint8_t a; uint16_t b; };
assert(sizeof(Data) == 3);
sendto(fd, &d, sizeof(Data), ...);
```

### 最佳实践清单

- [ ] 构造函数 `m_sock_fd = -1`（标记未初始化）
- [ ] `sockaddr_in` 声明后立刻 `memset` 归零
- [ ] 所有端口/IP 用 `htons`/`htonl` 转换
- [ ] 用 `inet_pton` 而非 `inet_addr`
- [ ] UDP socket 设置 `O_NONBLOCK`
- [ ] recvfrom 后区分 `EAGAIN` 和真实错误
- [ ] 跨网络结构体 `__attribute__((packed))` + `sizeof()` 验证
- [ ] 用 `memcpy` 反序列化而非指针强转
- [ ] sendto 返回值要检查（至少记录日志）
- [ ] 共享变量用 `mutex` + `lock_guard`
- [ ] 通信线程加看门狗超时

---

## 14. 相关笔记

- [[MAS_Nav_决策与UDP通信]] — 协议格式 + 线程架构 + FSM 决策
- [[MAS_Nav_学习指南]] — 导航系统全貌（母文档）
- [[MAS_Nav_Nav2导航栈详解]] — /cmd_vel 的生产者
- [[MAS_Vision_学习指南]] — Ros2_comm 所在系统全貌
- [[RM_C程序员_C++速通指南]] — C++ 基础（std::vector, mutex, atomic, RAII）
