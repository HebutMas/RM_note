# robot_control — 哨兵底盘板主控制

`apps/sentry/chassis_board/robot_control.c`

## 一句话

2ms 循环：接收云台板板间命令 → 裁判系统数据打包回传 → 底盘控制。无遥控器、无视觉、无云台。

## 初始化：robot_control_init

```c
void robot_control_init(void) {
    chassis_init();                    // 注册 8 个电机 + 功率控制
    Module_BoardComm_RegisterRxBuffer(&chassis_recv_cmd, sizeof(GimbalToChassis_cmd_t));  // 注册板间接收
    tx_thread_create(...);             // 创建 2ms 控制线程
}
```

| 步骤 | 做了什么 |
|------|---------|
| `chassis_init()` | 注册 4×M3508 + 4×GM6020 + 功率控制，详见 [[02_code_twin/apps/sentry/chassis_board/chassis_func/chassis_func]] |
| `Module_BoardComm_RegisterRxBuffer` | 注册接收云台板的 `GimbalToChassis_cmd_t`（速度+模式+偏航角） |
| 创建线程 | 优先级 30，栈 1024 字节 |

## 2ms 循环：robot_control_task

```c
while (1) {
    // 1. 板间通信接收
    if (板间通信离线) {
        // 安全停车
        chassis_cmd = {0, 0, 0, 0, chassis_zero_force};
    } else {
        // 解码云台板命令
        chassis_cmd.vx = chassis_recv_cmd.vx / 10.0 * CHASSIS_MAX_SPEED_MPS;
        chassis_cmd.vy = chassis_recv_cmd.vy / 10.0 * CHASSIS_MAX_SPEED_MPS;
        chassis_cmd.wz = chassis_recv_cmd.wz / 10.0 * CHASSIS_MAX_SPEED_MPS;
        chassis_cmd.offset_angle = chassis_recv_cmd.offset_angle * 360.0 / 8191.0;
        chassis_cmd.chassis_mode = chassis_recv_cmd.chassis_mode;
    }

    // 2. 裁判系统数据打包回传云台板
    handle_referee_data(&chassis_send_data);
    Module_BoardComm_Send(&chassis_send_data, sizeof(ChassisToGimbal_referee_t));

    // 3. 底盘控制
    chassis_func(&chassis_cmd);

    tx_thread_sleep(2);
}
```

### 板间通信解码

云台板发送的 `GimbalToChassis_cmd_t` 是压缩格式（8 字节），底盘板解压：

| 字段 | 云台板发送 | 底盘板解码 |
|------|----------|-----------|
| vx/vy/wz | `int8_t(-10~+10)` | `/ 10.0 * CHASSIS_MAX_SPEED_MPS` → m/s |
| offset_angle | 编码器原始值 | `* 360.0 / 8191.0` → 度 |
| chassis_mode | 枚举 | 直接透传 |

详见 [[02_code_twin/apps/sentry/gimbal_board/robot_control]] 的发送部分。

### 安全停车

板间通信离线时（CAN 超时），底盘板自动进入 `chassis_zero_force`，所有电机停转。防止云台板断连后底盘失控。

## 信号流

```
云台板                          底盘板
  │                               │
  ├─ GimbalToChassis_cmd_t ──CAN──→ 解码 → chassis_func → 8 个电机
  │   (vx/vy/wz/mode/angle)       │
  │                               │
  │←──CAN── ChassisToGimbal_referee_t ── handle_referee_data
  │   (color/hp/heat/bullet)      │
```

> 完整信号流图见 [[02_code_twin/apps/sentry/信号流图]]。

## 链接

- 底盘控制：[[02_code_twin/apps/sentry/chassis_board/chassis_func/chassis_func]]
- 裁判系统处理：[[02_code_twin/apps/sentry/chassis_board/robot_func/robot_func]]
- 云台板（对照）：[[02_code_twin/apps/sentry/gimbal_board/robot_control]]
