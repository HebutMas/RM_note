# robot_func — 哨兵底盘板裁判系统处理

`apps/sentry/chassis_board/robot_func/robot_func.c`

## 一句话

从裁判系统读取机器人状态，打包成 `ChassisToGimbal_referee_t` 结构体，通过板间通信回传给云台板。

## handle_referee_data

```c
void handle_referee_data(ChassisToGimbal_referee_t *referee_data) {
    // 从裁判系统获取 4 个数据块
    robot_status    = Module_Referee_Get_cmd_data(CMD_ID_ROBOT_STATUS);
    allowed_bullet  = Module_Referee_Get_cmd_data(CMD_ID_ALLOWED_BULLET);
    game_status     = Module_Referee_Get_cmd_data(CMD_ID_GAME_STATUS);
    power_heat_data = Module_Referee_Get_cmd_data(CMD_ID_POWER_HEAT_DATA);

    // 打包
    referee_data->robot_color      = (robot_id >= 100) ? 1 : 0;  // 0=红 1=蓝
    referee_data->bullet_allow     = allowed_bullet->bullet_17mm_allowed;
    referee_data->current_hp       = robot_status->current_hp;
    referee_data->game_progress    = game_status->type_progress.game_progress;
    referee_data->shooter_heat_pct = power_heat_data->shooter_17mm_heat;
}
```

### robot_color 判定

裁判系统 `robot_id`：红方 1-7，蓝方 101-107。`>= 100` 判蓝方。

这个 `robot_color` 通过板间 CAN 传给云台板后，云台板取反填入 `SendPacket.mode`（1=红方, 0=蓝方），再通过 USB 虚拟串口发给上位机。详见 [[02_code_twin/apps/sentry/gimbal_board/robot_control]]。

### ChassisToGimbal_referee_t 结构体

| 字段 | 来源 | 用途 |
|------|------|------|
| `robot_color` | robot_status.robot_id | 云台板取反后发给上位机 |
| `game_progress` | game_status | 判断比赛阶段（是否在比赛中） |
| `current_hp` | robot_status | 云台板可读取 |
| `bullet_allow` | allowed_bullet | 剩余弹丸数 |
| `shooter_heat_pct` | power_heat_data | 枪管热量百分比 |

## 链接

- 应用层调用：[[02_code_twin/apps/sentry/chassis_board/robot_control]]
- 板间通信：[[02_code_twin/modules/OFFLINE/module_offline-c]]
