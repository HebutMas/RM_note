# chassis_func — 步兵3号底盘控制

`apps/infantry3/single_board/chassis/chassis_func.c`

## 一句话

4 电机麦轮底盘：4 个 M3508，麦轮逆运动学分解底盘速度。带功率控制。

## 初始化：chassis_init

| 电机 | 型号 | CAN | tx_id | 角色 | 减速比 |
|------|------|-----|-------|------|--------|
| [0-3] | M3508 | CAN1 | 1-4 | DRIVE | 16:1 |

全部注册到功率控制（DRIVE 角色），功率上限 120W。与哨兵底盘的区别：4 个驱动轮（哨兵 4 驱动 + 4 舵向 = 8 个），用麦轮逆运动学（哨兵用舵轮）。

### 麦轮几何配置

```c
static const Chassis_Diff_Config_s chassis_diff_config = {
    .decele_ratio = 16.0f,
    .wheel_base_x = 0.5f,   // 前后轮距
    .wheel_base_y = 0.3f,   // 左右轮距
    .wheel_radius = 0.075f,  // 轮子半径
};
```

## 2ms 循环：chassis_func

逻辑与哨兵底盘相同：离线检查 → 模式选择 wz → 坐标变换 → 逆运动学。

区别在逆运动学调用：

| | 步兵3号 | 哨兵 |
|---|---|---|
| 逆运动学 | `Chassis_Mecanum_Calc`（麦轮） | `Chassis_Swerve_Calc`（舵轮） |
| 电机数 | 4 | 8 |
| 舵向轮 | 无 | 4×GM6020 |

## 链接

- 哨兵对照：[[02_code_twin/apps/sentry/chassis_board/chassis_func/chassis_func]]
- 功率控制：[[02_code_twin/modules/MOTOR/POWER_CONTROL/power_control]]
- 应用层调用：[[02_code_twin/apps/infantry3/single_board/robot_control]]
