# 哨兵机器人初始化详解

> 从 [[03_moc/Robot-Init-Walkthrough#APP_Init()]] 分支：哨兵（ROBOT=sentry）的 APP_Init 展开。

哨兵是双板架构，云台板和底盘板各自编译独立的固件，通过 CAN 板间通信协作。

## 哨兵云台板（BOARD=gimbal）

`apps/sentry/gimbal_board/robot_control.c` → [[02_code_twin/apps/sentry/gimbal_board/robot_control]]

### robot_control_init() — 5 步

| 步骤 | 做了什么 | 说明 |
|------|---------|------|
| `Module_INS_get()` | 获取 IMU 指针 | 后续每帧读 `ins->q[0..3]` 四元数 |
| `gimbal_init()` | `Motor_DJI_Init()` ×2 + `Motor_DM_Init()` ×1 | small_yaw(GM6020) + big_yaw(GM6020) + pitch(DM4310) |
| `shoot_init()` | `Motor_DJI_Init()` ×3 | friction_l + friction_r(M3508) + loader(M2006) |
| `Module_BoardComm_RegisterRxBuffer()` | 注册板间通信接收缓冲区 | 底盘板传裁判系统数据过来 |
| `tx_thread_create` | 创建 2ms 控制线程 | `TX_AUTO_START` 自动启动 |

### 2ms 循环

`RemoteControlSet` → `Module_Vision_Send/Receive` → `gimbal_func` → `shoot_func` → 板间通信发送底盘命令。

### 子模块

| 子模块 | 链接 |
|------|------|
| 云台控制 | [[02_code_twin/apps/sentry/gimbal_board/gimbal_func/gimbal_func]] |
| 发射控制 | [[02_code_twin/apps/sentry/gimbal_board/shoot_func/shoot_func]] |
| 遥控器输入 | [[02_code_twin/apps/sentry/gimbal_board/robot_func/robot_func]] |

## 哨兵底盘板（BOARD=chassis）

`apps/sentry/chassis_board/robot_control.c` → [[02_code_twin/apps/sentry/chassis_board/robot_control]]

### robot_control_init() — 3 步

| 步骤 | 做了什么 |
|------|---------|
| `chassis_init()` | `Motor_DJI_Init()` ×8（4×M3508 驱动 + 4×GM6020 舵向）+ 功率控制 |
| `Module_BoardComm_RegisterRxBuffer()` | 注册接收云台板命令 |
| `tx_thread_create` | 创建 2ms 控制线程 |

### 2ms 循环

板间接收（离线安全停车）→ 裁判系统打包回传 → `chassis_func`。

### 子模块

| 子模块 | 链接 |
|------|------|
| 底盘控制 | [[02_code_twin/apps/sentry/chassis_board/chassis_func/chassis_func]] |
| 裁判系统处理 | [[02_code_twin/apps/sentry/chassis_board/robot_func/robot_func]] |

## 板间协作

```
云台板                              底盘板
  │                                   │
  ├─ GimbalToChassis_cmd_t ──CAN──→  解码 → chassis_func → 8 个电机
  │   (vx/vy/wz/mode/angle)           │
  │                                   │
  │←──CAN── ChassisToGimbal_referee_t ── handle_referee_data
  │   (color/hp/heat/bullet)          │
```

云台板是主控：接收遥控器、处理视觉、控制云台和发射，同时把底盘命令打包发给底盘板。底盘板是从控：执行底盘运动，并把裁判系统数据回传给云台板。
