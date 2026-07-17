# shoot_func — 步兵3号发射控制

`apps/infantry3/single_board/shoot_func/shoot_func.c`

## 一句话

3 个大疆电机的发射控制：2 个 M3508 摩擦轮 + 1 个 M2006 拨盘。与哨兵发射控制逻辑相同，CAN 总线和 tx_id 不同。

## 初始化：shoot_init

| 电机 | 型号 | CAN | tx_id | 减速比 |
|------|------|-----|-------|--------|
| friction_l | M3508 | CAN2 | 6 | 19:1 |
| friction_r | M3508 | CAN2 | 8 | 19:1 |
| loader | M2006 | CAN1 | 7 | 36:1 |

与哨兵的区别：摩擦轮在 CAN2（哨兵在 CAN1），tx_id 不同（步兵 6/8 vs 哨兵 1/2）。摩擦轮转向相反（步兵左负右正 vs 哨兵左正右负）。

## 2ms 循环：shoot_func

逻辑与哨兵完全相同，详见 [[02_code_twin/apps/sentry/gimbal_board/shoot_func/shoot_func]]。

## 链接

- 哨兵对照：[[02_code_twin/apps/sentry/gimbal_board/shoot_func/shoot_func]]
- 应用层调用：[[02_code_twin/apps/infantry3/single_board/robot_control]]
