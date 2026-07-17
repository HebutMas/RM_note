# 步兵3号初始化详解

> 从 [[03_moc/Robot-Init-Walkthrough#APP_Init()]] 分支：步兵3号（ROBOT=infantry3）的 APP_Init 展开。

步兵3号是单板架构，云台、发射、底盘全在一块板子上，无板间通信。

## 步兵3号单板（BOARD=single_board）

`apps/infantry3/single_board/robot_control.c` → [[02_code_twin/apps/infantry3/single_board/robot_control]]

### robot_control_init() — 5 步

| 步骤 | 做了什么 |
|------|---------|
| `Module_INS_get()` | 获取 IMU 指针 |
| `gimbal_init()` | 1×GM6020(yaw) + 1×DM4310(pitch) |
| `shoot_init()` | 2×M3508(摩擦轮) + 1×M2006(拨盘) |
| `chassis_init()` | 4×M3508(麦轮) + 功率控制 |
| `tx_thread_create` | 创建 2ms 控制线程 |

### 2ms 循环

`RemoteControlSet` → 视觉收发 → `gimbal_func` → `shoot_func` → `chassis_func`。单板模式，无板间通信。

### mode 填法

步兵3号直接从裁判系统读 `robot_id`：

```c
const robot_status_t *robot_status = Module_Referee_Get_cmd_data(CMD_ID_ROBOT_STATUS);
if (robot_status->robot_id >= 100)
    send_packet.mode = 0;  // 蓝方 → mode=0
else
    send_packet.mode = 1;  // 红方 → mode=1
```

哨兵云台板则通过板间通信从底盘板拿 `robot_color`（底盘板读裁判系统），取反后填入。最终结果一样：1=红方, 0=蓝方。

### 子模块

| 子模块 | 链接 |
|------|------|
| 云台控制 | [[02_code_twin/apps/infantry3/single_board/gimbal_func/gimbal_func]] |
| 发射控制 | [[02_code_twin/apps/infantry3/single_board/shoot_func/shoot_func]] |
| 底盘控制 | [[02_code_twin/apps/infantry3/single_board/chassis_func/chassis_func]] |
| 遥控器输入 | [[02_code_twin/apps/infantry3/single_board/robot_func/robot_func]] |

### 与哨兵的对比

| | 步兵3号单板 | 哨兵 |
|---|---|---|
| 板型 | 单板（gimbal+shoot+chassis 一体） | 双板（云台板 + 底盘板） |
| 遥控器 | DT7（sw1/sw2 + 摇杆 + 滚轮） | SBUS（CH1-8） |
| 视觉 mode 来源 | 直接读裁判系统 `robot_id` | 通过板间 CAN 从底盘板拿 `robot_color` |
| 底盘控制 | 直接调 `chassis_func` | 打包发给底盘板 |
| 底盘类型 | 麦轮（4×M3508） | 舵轮（4×M3508 + 4×GM6020） |
| 云台 | 2 电机（1 yaw + 1 pitch） | 3 电机（大 yaw + 小 yaw + pitch） |
| 板间通信 | 无 | 有（CAN） |
