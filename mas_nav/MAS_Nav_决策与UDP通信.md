# MAS Nav — 决策状态机与 UDP 通信

> 📘 **所属**：[[MAS_Nav_学习指南]] §7.7-7.8 | **定位**：战术决策 + 上位机通信
> 📘 **底层基础**：[[RM_Socket编程详解]] — socket API、非阻塞 I/O、帧协议、线程安全等底层知识
> 📘 **相关**：UDP 桥与 mas_vision 端通信，详见 [[MAS_Vision_学习指南#附录-C-ROS2-通信与启动参数详解]]

## 第一部分：决策状态机 (rm_decision)

### 架构总览

```
                    ┌──────────────────┐
    /referee_data ─>│   CallBackMsg     │──> 检测比赛状态变化
                    └────────┬─────────┘
                             │
                    ┌────────v─────────┐
                    │  thread_FSM_state │──> 状态更新循环 (1Hz)
                    │  state_update()   │
                    └────────┬─────────┘
                             │ FSM_state
                    ┌────────v─────────┐
                    │   thread_FSM      │──> 任务执行循环 (1Hz)
                    │   mission_*()     │
                    └────────┬─────────┘
                             │ NavigateThroughPoses
                    ┌────────v─────────┐
                    │  BasicNavigator   │──> Nav2 Action 客户端
                    └──────────────────┘
```

## 7.7.2 状态转换逻辑

```python
def state_update(self):
    if self.msg_callback.gamestatus == 0:
        # 比赛未开始/已结束 → 待机
        self.FSM_state = 0  # STANDBY
    elif self.msg_callback.gamestatus == 4:
        # 比赛进行中
        if self.msg_callback.current_robot_hp < 200:
            self.FSM_state = 2  # RESTORE (血低回血)
        else:
            self.FSM_state = 1  # ATTACK (正常攻击)
```

**状态解释：**

| 状态 | ID | 触发条件 | 行为 | 退出条件 |
|------|-----|----------|------|----------|
| STANDBY | 0 | game_progress=0 | 等待（空循环） | game_progress=4 |
| ATTACK | 1 | game_progress=4 且 HP>=200 | 按 attack.csv 航点导航 | HP<200 |
| RESTORE | 2 | HP<200 | 按 restore.csv 航点导航（去回血区） | HP恢复(>=350) → ATTACK |
| OUTPOST | 3 | 预留（前哨站被毁后策略） | 按 outpost.csv 航点导航 | — |

## 7.7.3 航点管理机制

```python
def __force_load_all_csv(self):
    # 启动时强制加载所有3个CSV
    for state_id, csv_path in self.csv_config.items():
        waypoints = self.__parse_csv_file(csv_path)
        self.state_waypoint_cache[state_id] = waypoints

def __activate_state_waypoints(self, state_id):
    # 状态切换时：
    # 1. 强制取消当前导航任务
    self.__force_cancel_current_nav()
    # 2. 加载新状态的航点列表
    waypoints = self.state_waypoint_cache[state_id]
    self.current_goal_dict = {idx+1: data for idx, data in enumerate(waypoints)}
    self.current_goal_path = list(self.current_goal_dict.keys())
```

**CSV 文件格式：**
```csv
id, pose_x, pose_y, pose_z, rot_x, rot_y, rot_z, rot_w
1, 1.5, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0
2, 3.0, 4.5, 0.0, 0.0, 0.0, 0.707, 0.707
```

## 7.7.4 导航执行逻辑

```python
def execute_full_waypoints(self):
    # 1. 如果导航正在运行，不做任何事
    if not self.navigator.isTaskComplete():
        return
    
    # 2. 从航点列表构建 PoseStamped 序列
    pose_list = [self.build_pose(pid, data) 
                 for pid, data in self.current_goal_dict.items()]
    
    # 3. 发送 NavigateThroughPoses action
    self.navigator.goThroughPoses(pose_list)
```

**Action 与 Topic 的区别：**
- `NavigateThroughPoses` 是 Action（有反馈、可取消、有最终结果）
- 相比普通的 `Publish` topic，Action 提供了更丰富的生命周期管理

## 7.7.5 BasicNavigator（Nav2 Action 客户端封装）

`robot_navigator.py` 是一个通用的 Nav2 客户端封装，提供：

- `goThroughPoses(poses)` — 通过多个航点
- `goToPose(pose)` — 导航到单个目标
- `spin(angle)` — 旋转指定角度
- `backup(distance, speed)` — 后退
- `cancelTask()` — 取消当前任务
- `isTaskComplete()` — 检查任务是否完成
- `clearAllCostmaps()` — 清除代价地图
- `lifecycleStartup()` / `lifecycleShutdown()` — 管理 Nav2 生命周期

---

---

## 第二部分：UDP 桥接通信 (ros2_comm)

## 7.8.1 通信架构

```
┌──────────────┐                  ┌──────────────────┐
│ 上位机/裁判系统 │  UDP (8888/8889) │  Docker 容器     │
│              │<════════════════>│  ros2_comm 节点  │
│              │  二进制帧协议    │                  │
└──────────────┘                  └────────┬─────────┘
                                          │ ROS 2 话题
                                   ┌──────┴──────┐
                                   │ /referee_data│──> rm_decision
                                   │ /cmd_vel     │<── Nav2
                                   └─────────────┘
```

## 7.8.2 自定义二进制协议详解

**协议设计原则：** 极小开销（每帧仅 2 字节帧头+1 字节长度+1 字节帧尾），适配实时性要求。

**下行帧（上位机 → ROS2，UDP 8888，裁判数据）：**

```
字节0:     0xAA (SEND_FRAME_HEADER)
字节1:     数据长度 (data_len = sizeof(RawRefereePacket) = 12)
字节2-13:  RawRefereePacket 结构体 (12 字节)
字节14:    0x5A (SEND_FRAME_TAIL)

结构体布局 (__attribute__((packed))，紧凑排列)：
  uint16 projectile_allowance_17mm    (字节2-3)
  uint8  power_management_shooter_output (字节4)
  uint16 current_hp                  (字节5-6)
  uint16 outpost_HP                  (字节7-8)
  uint16 base_HP                     (字节9-10)
  uint8  game_progess                (字节11)
                                     (字节12) padding? → 实际12字节
```

**上行帧（ROS2 → 上位机，UDP 8889，控制指令）：**

```
字节0:     0xBB (RECV_FRAME_HEADER)
字节1:     数据长度 (sizeof(RawControlPacket) = 9)
字节2-10:  RawControlPacket 结构体 (9 字节)
字节11:    0x5B (RECV_FRAME_TAIL)

结构体布局：
  float32 vx       (字节2-5)
  float32 vy       (字节6-9)
  uint8  nav_state (字节10)
```

**为什么用二进制而非 JSON/Protobuf？**
- 二进制每帧 ~15 字节，JSON 需 ~200 字节
- 500μs 发送间隔 → 2000 packets/s → 节省约 370KB/s 带宽
- 二进制解析无字符串操作，延迟更低

## 7.8.3 通信线程实现

```cpp
void comm_thread_loop() {
    while (m_running) {
        // 1. 非阻塞接收 (recvfrom)
        int n = recvfrom(m_sock_fd, buffer, 1024, 0, ...);
        if (n > 0) {
            //   验证帧头 0xAA + 帧尾 0x5A
            if (buffer[0] == 0xAA && buffer[2+data_len] == 0x5A) {
                //   反序列化 RawRefereePacket
                memcpy(&raw, &buffer[2], sizeof(raw));
                //   发布 ROS 2 消息
                pub_referee_->publish(msg);
            }
        }
        
        // 2. 发送控制指令（每 500μs 一次）
        {
            std::lock_guard<std::mutex> lock(m_ctrl_mutex);
            // 看门狗：超过1秒未收到 cmd_vel → 停止
            if (elapsed > CMD_TIMEOUT_SEC) {
                pkt_to_send.vx = pkt_to_send.vy = 0.0f;
                pkt_to_send.nav_state = 0;
            } else {
                pkt_to_send = m_latest_ctrl;
            }
            send_to_host(pkt_to_send);
        }
        
        std::this_thread::sleep_for(500μs);
    }
}
```

## 7.8.4 看门狗安全机制

```cpp
// /cmd_vel 回调中更新时间戳
void cmd_vel_callback(const Twist::SharedPtr msg) {
    m_latest_ctrl.vx = msg->linear.x;
    m_latest_ctrl.vy = msg->linear.y;
    m_latest_ctrl.nav_state = 1;  // 标记为导航激活
    m_last_cmd_time = this->now(); // 更新最后命令时间
}

// 通信线程检查超时
double elapsed = (now - m_last_cmd_time).seconds();
if (elapsed > 1.0) {
    // 1秒内无指令 → 急停（零速度）
    pkt_to_send.vx = pkt_to_send.vy = 0.0f;
}
```

**为什么需要看门狗？**
- Nav2 崩溃/卡死 → 不会再有 cmd_vel 发布
- 如果上位机继续执行最后的非零速度指令，机器人会失控
- 看门狗确保 1s 内无指令则自动停止

---

## 深度拓展 1：FSM 形式化验证

### 状态转移矩阵

将 FSM 状态编码为数字后，状态转移可表示为：

```
状态集 S = {0: STANDBY, 1: ATTACK, 2: RESTORE, 3: OUTPOST}
输入集 I = {game_progress, HP}

转移表 T: S × I → S

         | GP=0    | GP=4,HP≥200 | GP=4,HP<200 | GP=4,HP≥350
STANDBY  | STANDBY | ATTACK      | -           | -
ATTACK   | STANDBY | ATTACK      | RESTORE     | -
RESTORE  | STANDBY | -           | RESTORE     | ATTACK
OUTPOST  | STANDBY | -           | -           | OUTPOST
```

### 可达性分析

**问题**：从任何合法状态出发，是否总能到达某个期望状态？

```
claim: STANDBY → ATTACK 总是可达的
proof: 当 game_progress=4 且 HP≥200 时，state=ATTACK
       从 STANDBY: GP=4, HP=200 → ATTACK ✓
       从 ATTACK:  直接 ✓
       从 RESTORE: GP=4, HP≥350 → ATTACK (HP恢复后)
       从 OUTPOST: 需要人工干预或 GP=0→STANDBY→ATTACK
```

**问题**：是否存在非预期的状态序列？

```
危险序列：ATTACK → RESTORE → ATTACK → RESTORE → ...
触发条件：HP 在 [180, 220] 附近振荡（受到攻击，在回血边缘）
后果：   机器人不断在攻击和回血之间切换，永远无法到达目标
修复：   添加迟滞（hysteresis）：
         ATTACK→RESTORE 需要 HP < 200
         RESTORE→ATTACK 需要 HP ≥ 350（而非 200）
         → 150 的迟滞带防止振荡
```

**当前代码已有迟滞**：RESTORE 退出条件是 `HP≥350`，进入条件是 `HP<200`。150 的迟滞带足以防止边界振荡。

### 死锁检测

```
死锁条件：所有状态迁移都被阻塞
可能原因：game_progress 停留在非法值（非 0 非 4）
当前处理：default 分支不存在 → state 保持上一帧的值
风险：   如果裁判系统数据中断 → 不会自动 STOP
建议：   添加超时检测：超过 5s 未收到裁判数据 → 强制 state=STANDBY
```

---

## 深度拓展 2：看门狗时序的边界条件分析

### 最坏情况延迟

分析 1s 看门狗的**实际保护窗口**：

```
T0:     Nav2 发布最后一个 cmd_vel        (t = 0ms)
T1:     cmd_vel_callback 更新 m_last_cmd_time  (t + ~0.1ms, ROS 消息延迟)
T2:     comm_thread 检查看门狗           (t + ~500μs, 线程周期)
T3:     下一次 comm_thread 检查          (t + ~500μs + 500μs = t + 1ms)
...
T_1s:   1s 超时触发                     (t + 1000ms)

总延迟 = 1000ms（超时）+ 500μs（线程周期）+ 100μs（ROS 消息延迟）
      ≈ 1000.6ms
```

**在这个窗口内机器人的移动距离**：

```
最坏速度 = 3.0 m/s（Nav2 最大限速）
距离 = 3.0 × 1.0006 ≈ 3.0m

结论：即使 Nav2 崩溃，机器人最多再移动 3m 就会自动急停。
     在 5m × 8m 的比赛场地上，3m 通常不会导致出场，但可能撞到障碍物。
```

**改进建议**：将看门狗从 1.0s 缩短到 0.5s（最多移动 1.5m），代价是正常的导航间隙（没有 cmd_vel 的短暂时刻）可能触发误报。需要权衡 Nav2 的控制周期（40Hz = 25ms）与看门狗窗口的关系。

### 看门狗误触发概率

Nav2 MPPI 每 25ms 发布 cmd_vel。如果连续 40 个周期丢失（40 × 25ms = 1000ms），看门狗触发。

```
P(误触发) = P(连续 40 次 cmd_vel 丢失 | Nav2 正常工作)
         = (P(单次丢失))⁴⁰

若 P(单次丢失) = 0.001:  → (0.001)⁴⁰ ≈ 10⁻¹²⁰ → 零
若 P(单次丢失) = 0.01:   → (0.01)⁴⁰ ≈ 10⁻⁸⁰  → 零
若 P(单次丢失) = 0.05:   → (0.05)⁴⁰ ≈ 10⁻⁵³  → 近乎零
```

**结论**：在正常操作下（Nav2 持续发布），1s 看门狗几乎不可能误触发。但如果在重负载下 Nav2 偶尔丢失一帧，连续丢失 40 帧的概率仍然极低。

---

## 深度拓展 3：UDP 帧的丢包与错误分析

### 丢包率的链路计算

```
UDP 帧：15 字节 + IP header (20B) + UDP header (8B) + Ethernet (14B) = 57 字节

发送频率：1/500μs = 2000 packets/s
带宽占用：57 × 8 × 2000 = 912 kbps ≈ 0.11 MB/s

在 127.0.0.1 loopback 上：    丢包率 ≈ 0%（内核内存拷贝，无物理链路）
在 host 模式 Docker 网络上：   丢包率 ≈ 10⁻⁶（内核网络栈）
在物理以太网上：              丢包率 ≈ 10⁻⁴~10⁻⁶（取决于电缆质量）
```

### 帧错误概率

帧格式 `[HEADER, LEN, DATA, TAIL]`，校验仅靠帧头 `0xAA/0xBB` + 帧尾 `0x5A/0x5B`。

```
P(帧错误) = P(随机数据刚好匹配帧格式)
          = P(random_byte == 0xAA) × P(random_byte == 0x5A)
          × P(中间数据长度恰好为 LEN)
          = (1/256)² × (1/256) ≈ 6 × 10⁻⁸

加上 LEN 校验：实际错误概率更低。
```

**但这不是 CRC 级别的保护**。如果帧中间的数据字段出现比特翻转（宇宙射线、内存 bit-flip），当前帧尾检测无法发现。对于比赛场景（短距离、有线/本机通信），这足够。对于关键安全应用，应添加 CRC16。

### 帧同步恢复

当解析器因错误帧失去同步时：

```
当前实现：
  while (buffer.size() >= sizeof(Packet)):
    找 0xAA → 检查 LEN → 检查 0x5A
    如果失败：buffer.erase(buffer.begin())  ← 丢弃 1 字节，重新搜索

恢复时间：
  最坏：buffer 中有 N 个 0xAA 误匹配 → 最多 N × sizeof(Packet) × 500μs 延迟
  平均：1 个线程周期 (500μs) ← 通常在下一个帧就恢复同步
```

---

> 📘 **相关笔记**：[[MAS_Nav_学习指南]] · [[MAS_Nav_Nav2导航栈详解]] · [[MAS_Vision_学习指南]]
