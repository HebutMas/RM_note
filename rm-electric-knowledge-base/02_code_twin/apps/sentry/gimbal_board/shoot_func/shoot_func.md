# shoot_func — 哨兵发射控制

`apps/sentry/gimbal_board/shoot_func/shoot_func.c`

## 一句话

3 个大疆电机的发射控制：2 个 M3508 摩擦轮对转 + 1 个 M2006 拨盘。摩擦轮定速旋转，拨盘控制单发/连发。

## 初始化：shoot_init

| 电机 | 型号 | CAN | tx_id | 控制方式 | 说明 |
|------|------|-----|-------|---------|------|
| friction_l | M3508 | CAN1 | 1 | LQR 速度环 | 左摩擦轮，+6800 RPM |
| friction_r | M3508 | CAN1 | 2 | LQR 速度环 | 右摩擦轮，-6800 RPM |
| loader | M2006 | CAN1 | 3 | LQR 速度环 | 拨盘电机 |

摩擦轮减速比 19:1，拨盘减速比 36:1。摩擦轮正反转速相同方向相反，对转挤出弹丸。

## 2ms 循环：shoot_func

```c
void shoot_func(Shoot_Ctrl_Cmd_t *shoot_cmd) {
    // 1. 离线检查：3 个电机全部在线才控制
    // 2. shoot_on？
    //   ├─ friction_on？
    //   │   ├─ 摩擦轮定速: ±6800 RPM × RPM_2_RAD_PER_SEC
    //   │   └─ 拨盘按 load_mode:
    //   │       load_stop       → 0
    //   │       load_1_bullet   → -4000 RPM
    //   │       load_burstfire  → -8000 RPM
    //   └─ friction_off → 全部归零
    // 3. shoot_off → 全部 Stop
}
```

### 摩擦轮

摩擦轮是**速度环控制**，直接设一个固定转速参考值。弹丸在两个对转摩擦轮之间被加速，出口速度由摩擦轮转速决定。`6800 RPM` 是经验值，需实测后调整。

### 拨盘

| 模式 | 转速 | 说明 |
|------|------|------|
| `load_stop` | 0 | 停止 |
| `load_1_bullet` | -4000 RPM | 单发，每次拨一格 |
| `load_burstfire` | -8000 RPM | 连发 |

拨盘负转速表示正向拨弹（方向取决于安装）。

## 链接

- 电机：[[02_code_twin/modules/MOTOR/DJI/motor_dji]]
- 遥控器输入：[[02_code_twin/apps/sentry/gimbal_board/robot_func]]
- 应用层调用：[[02_code_twin/apps/sentry/gimbal_board/robot_control]]
