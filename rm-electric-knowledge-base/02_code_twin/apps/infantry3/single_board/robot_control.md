# robot_control — 步兵3号单板主控制

`apps/infantry3/single_board/robot_control.c`

## 一句话

单板模式，2ms 循环：遥控器输入 → 视觉收发 → 云台 → 发射 → 底盘。所有功能在一块板子上，无板间通信。

## 初始化：robot_control_init

```c
void robot_control_init(void) {
    ins = Module_INS_get();    // IMU 姿态
    gimbal_init();             // 电机: 1×GM6020(yaw) + 1×DM4310(pitch)
    shoot_init();              // 电机: 2×M3508(摩擦轮) + 1×M2006(拨盘)
    chassis_init();            // 电机: 4×M3508(麦轮)
    tx_thread_create(...);     // 创建 2ms 控制线程
}
```

与哨兵云台板的区别：步兵3号是**单板**，gimbal + shoot + chassis 全在一个 `robot_control_init` 里初始化，不需要板间通信注册。

## 2ms 循环：robot_control_task

```c
while (1) {
    RemoteControlSet(&chassis_cmd, &shoot_cmd, &gimbal_cmd);  // ① 遥控器输入
    // ② 视觉收发
    send_packet.mode = (robot_id >= 100) ? 0 : 1;  // 直接读裁判系统
    send_packet.q[0..3] = ins->q[0..3];
    Module_Vision_Send(&send_packet, TX_NO_WAIT);
    receive_packet = Module_Vision_Receive();
    // ③ 云台控制
    gimbal_func(&gimbal_cmd, &yaw_ecd);
    // ④ 发射控制
    shoot_func(&shoot_cmd);
    // ⑤ 底盘控制
    chassis_cmd.offset_angle = CalcOffsetAngle(yaw_ecd) * 360.0 / 8191.0;
    chassis_func(&chassis_cmd);
    tx_thread_sleep(2);
}
```


> 完整信号流图见 [[02_code_twin/apps/infantry3/signal-flow]]。

## 链接

- 云台控制：[[02_code_twin/apps/infantry3/single_board/gimbal_func/gimbal_func]]
- 发射控制：[[02_code_twin/apps/infantry3/single_board/shoot_func/shoot_func]]
- 底盘控制：[[02_code_twin/apps/infantry3/single_board/chassis_func/chassis_func]]
- 遥控器输入：[[02_code_twin/apps/infantry3/single_board/robot_func/robot_func]]
- 视觉模块：[[02_code_twin/modules/VISION/module_vision]]
- 哨兵云台板（对照）：[[02_code_twin/apps/sentry/gimbal_board/robot_control]]
