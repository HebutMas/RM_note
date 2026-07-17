# robot_control.c — 哨兵云台板主控制

`apps/sentry/gimbal_board/robot_control.c`

## 一句话

哨兵云台板的 2ms 控制循环，依次执行：遥控器输入 → 视觉收发 → 云台控制 → 发射控制 → 板间通信。

## 初始化：robot_control_init

```c
void robot_control_init(void) {
    ins = Module_INS_get();                    // 获取 IMU 姿态指针
    gimbal_init();                             // 云台初始化
    shoot_init();                              // 发射机构初始化
    Module_BoardComm_RegisterRxBuffer(&chassis_upload_data, sizeof(ChassisToGimbal_referee_t));
    tx_thread_create(&robot_control_thread, "robot_control_thread", robot_control_task, ...);
}
```

| 步骤 | 做了什么 |
|------|---------|
| `Module_INS_get()` | 拿到 INS 模块的 `Ins_t *ins` 指针，后续每帧读 `ins->q[0..3]` 四元数 |
| `gimbal_init()` / `shoot_init()` | 云台和发射机构的电机注册 + PID 初始化 |
| `Module_BoardComm_RegisterRxBuffer` | 注册板间通信接收缓冲区，底盘板通过 CAN 把裁判系统数据（颜色/血量/热量）传过来 |
| 创建线程 | 优先级 30，栈 1024 字节，`TX_AUTO_START` 自动启动 |

## 2ms 控制循环：robot_control_task

```c
static void robot_control_task(ULONG thread_input) {
    while (1) {
        RemoteControlSet(&chassis_cmd, &shoot_cmd, &gimbal_cmd);     // ① 遥控器输入
        // ② 视觉收发（见下文）
        gimbal_func(&gimbal_cmd, &yaw_ecd);                          // ③ 云台控制
        shoot_func(&shoot_cmd);                                      // ④ 发射控制
        // ⑤ 板间通信（见下文）
        tx_thread_sleep(2);                                          // 2ms 周期
    }
}
```

5 个阶段依次执行，下面展开视觉和板间通信部分。

## 视觉收发

```c
/* 虚拟串口 */
send_packet.mode = 1 - chassis_upload_data.robot_color;
send_packet.q[0] = ins->q[0];
send_packet.q[1] = ins->q[1];
send_packet.q[2] = ins->q[2];
send_packet.q[3] = ins->q[3];
Module_Vision_Send(&send_packet, TX_NO_WAIT);
receive_packet = Module_Vision_Receive();
```

### 发送：填 SendPacket

| 字段 | 填什么 | 来源 |
|------|--------|------|
| `mode` | `1 - chassis_upload_data.robot_color` | 底盘板通过板间通信传过来的裁判系统颜色 |
| `q[0..3]` | `ins->q[0..3]` | INS 模块的 IMU 四元数 |
| `header` / `tail` | `0xAA` / `0x5A` | `Module_Vision_Send` 自动填 |

`Module_Vision_Send` 详见 [[02_code_twin/modules/VISION/module_vision]]，底层 USB 传输详见 [[02_code_twin/board/bsp/USB/bsp_usb_cdc]]。

发送用 `TX_NO_WAIT`——发不出去就跳过，视觉数据 2ms 一帧，丢一帧无所谓。

### `mode` 字段为什么取反

`chassis_upload_data` 是 `ChassisToGimbal_referee_t` 结构体，由底盘板通过板间通信传过来：

```c
// sentry_def.h
typedef struct {
    uint8_t robot_color;      /* 机器人颜色 (0=红,1=蓝) */
    uint8_t game_progress;    /* 比赛阶段 */
    uint16_t current_hp;       /* 当前血量 */
    uint16_t bullet_allow;     /* 可发射17mm弹丸剩余量 */
    uint16_t shooter_heat_pct; /* 枪管热量% */
} ChassisToGimbal_referee_t;
```

`robot_color` 定义：0=红方，1=蓝方（裁判系统原始值）。取反后传给上位机：`mode = 1 - robot_color`，即 1=红方，0=蓝方。具体取反原因取决于上位机协议约定。

### 接收：取 ReceivePacket 指针

```c
receive_packet = Module_Vision_Receive();
```

非阻塞，直接返回内部 `rx_packet` 指针。真正的阻塞接收在视觉模块自己的线程里完成，详见 [[02_code_twin/modules/VISION/module_vision]]。

## 板间通信

```c
/* 板间通讯 */
// 速度比例 (-1.0~+1.0) → 板间 int8 (-10~+10)
chassis_send_cmd.vx           = (int8_t)(chassis_cmd.vx * 10.0f);
chassis_send_cmd.vy           = (int8_t)(chassis_cmd.vy * 10.0f);
chassis_send_cmd.wz           = (int8_t)(chassis_cmd.wz * 10.0f);
chassis_send_cmd.offset_angle = CalcOffsetAngle(yaw_ecd);
chassis_send_cmd.chassis_mode = chassis_cmd.chassis_mode;
Module_BoardComm_Send((uint8_t *)&chassis_send_cmd, sizeof(GimbalToChassis_cmd_t));
```

云台板把底盘控制指令打包发给底盘板：

| 字段 | 转换 | 说明 |
|------|------|------|
| `vx/vy/wz` | `float(-1.0~+1.0) × 10 → int8_t(-10~+10)` | 速度比例压缩为整数，节省通信字节 |
| `offset_angle` | `CalcOffsetAngle(yaw_ecd)` | 云台偏航角换算 |
| `chassis_mode` | 直接透传 | 底盘模式（跟随/小陀螺等） |

`GimbalToChassis_cmd_t` 固定 8 字节，`_Static_assert` 编译期保证大小。
