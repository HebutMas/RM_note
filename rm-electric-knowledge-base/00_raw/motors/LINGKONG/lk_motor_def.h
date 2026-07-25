#ifndef __LK_MOTOR_DEF_H__
#define __LK_MOTOR_DEF_H__

/*----------------------------------------------- CAN 总线参数 -------------------------------------------------*/
#define LK_CAN_CMD_BASE_ID  0x140  /* 主机→电机 命令帧基址，实际 ID = 0x140 + motor_id (ID 范围 1~32)           */
#define LK_CAN_REPLY_BASE_ID 0x180 /* 电机→主机 回复帧基址，实际 ID = 0x180 + motor_id (ID 范围 1~32)           */
#define LK_CAN_DLC           8     /* 数据长度固定 8 字节                                                       */                                         
/*----------------------------------------------- 控制命令字节 --------------------------------------------------*/
/* 基本状态控制 */
#define LK_CMD_READ_STATUS1_ERROR 0x9A /* 读取电机状态1和错误标志                                               */
#define LK_CMD_CLEAR_ERROR        0x9B /* 清除电机错误标志                                                       */
#define LK_CMD_READ_STATUS2       0x9C /* 读取电机状态2                                                          */
#define LK_CMD_READ_STATUS3       0x9D /* 读取电机状态3（除 MS 外）                                              */
#define LK_CMD_MOTOR_OFF          0x80 /* 电机关闭 — 清除转动圈数及控制指令，LED 慢闪                            */
#define LK_CMD_MOTOR_RUN          0x88 /* 电机运行 — 切换到开启状态，LED 常亮                                    */
#define LK_CMD_MOTOR_STOP         0x81 /* 电机停止 — 停止但不改变运行状态                                         */
/* 辅助功能 */
#define LK_CMD_BRAKE_CTRL         0x8C /* 抱闸器控制和状态读取                                                    */
/* 闭环控制，一般只用转矩闭环控制，扭矩闭环参数出场已经设置，可读不用调整 */
#define LK_CMD_OPEN_LOOP          0xA0 /* 开环控制（仅 MS）                                                        */
#define LK_CMD_TORQUE_CLOSED_LOOP 0xA1 /* 转矩闭环控制（仅 MF、MH、MG）                                            */
#define LK_CMD_SPEED_CLOSED_LOOP  0xA2 /* 速度闭环控制                                                             */
#define LK_CMD_MULTI_POS_CTRL1    0xA3 /* 多圈位置闭环控制1                                                        */
#define LK_CMD_MULTI_POS_CTRL2    0xA4 /* 多圈位置闭环控制2（可指定 maxSpeed）                                     */
#define LK_CMD_SINGLE_POS_CTRL1   0xA5 /* 单圈位置闭环控制1                                                        */
#define LK_CMD_SINGLE_POS_CTRL2   0xA6 /* 单圈位置闭环控制2（可指定 maxSpeed）                                     */
#define LK_CMD_INCREMENT_POS_CTRL1 0xA7 /* 增量位置闭环控制1                                                       */
#define LK_CMD_INCREMENT_POS_CTRL2 0xA8 /* 增量位置闭环控制2（可指定 maxSpeed）                                    */
/* 编码器与角度，看返回帧就行，这些命令不用 */
#define LK_CMD_READ_ENCODER       0x90 /* 读取编码器数据                                                           */
#define LK_CMD_SET_ZERO_ROM       0x19 /* 设置当前位置到 ROM 作为电机零点（需重新上电生效）                        */
#define LK_CMD_READ_MULTI_ANGLE   0x92 /* 读取多圈角度                                                             */
#define LK_CMD_READ_SINGLE_ANGLE  0x94 /* 读取单圈角度                                                             */
#define LK_CMD_SET_CURRENT_ANGLE  0x95 /* 设置当前位置为任意角度（写 RAM，即时生效，断电失效）                     */
/* 参数读写 */
#define LK_CMD_READ_PARAM         0xC0 /* 读取控制参数                                                             */
#define LK_CMD_WRITE_PARAM        0xC1 /* 写入控制参数（写 RAM，即时生效，断电失效）                               */
/*----------------------------------------------- 电机状态标志 --------------------------------------------------*/
/* 电机运行状态 (DATA[6] of 0x9A) */
#define LK_MOTOR_STATUS_ON        0x00 /* 电机开启                                                                 */
#define LK_MOTOR_STATUS_OFF       0x10 /* 电机关闭                                                                 */
/*----------------------------------------------- 错误标志位 ----------------------------------------------------*/
/* 错误标志位 (DATA[7] of 0x9A, bit 0~7) */
#define LK_ERROR_UNDER_VOLTAGE    (1 << 0) /* 低电压保护                                                              */
#define LK_ERROR_OVER_VOLTAGE     (1 << 1) /* 高压保护                                                                */
#define LK_ERROR_DRIVER_OVER_TEMP (1 << 2) /* 驱动过温                                                                */
#define LK_ERROR_MOTOR_OVER_TEMP  (1 << 3) /* 电机过温                                                                */
#define LK_ERROR_OVER_CURRENT     (1 << 4) /* 电机过流                                                                */
#define LK_ERROR_SHORT_CIRCUIT    (1 << 5) /* 电机短路                                                                */
#define LK_ERROR_STALL            (1 << 6) /* 电机堵转                                                                */
#define LK_ERROR_SIGNAL_LOST      (1 << 7) /* 输入信号丢失超时                                                        */
/*----------------------------------------------- 抱闸器控制 ----------------------------------------------------*/
#define LK_BRAKE_POWER_OFF        0x00 /* 抱闸器断电（刹车启动）                                                   */
#define LK_BRAKE_POWER_ON         0x01 /* 抱闸器通电（刹车释放）                                                   */
#define LK_BRAKE_READ_STATUS      0x10 /* 读取抱闸器状态                                                           */
/*----------------------------------------------- 单圈转动方向 --------------------------------------------------*/
#define LK_SPIN_CW                0x00 /* 顺时针                                                                   */
#define LK_SPIN_CCW               0x01 /* 逆时针                                                                   */
/*----------------------------------------------- 控制参数 ID ---------------------------------------------------*/
/* PID 参数 */
#define LK_PARAM_ANGLE_PID        0x0A /* 角度环 PID: [Kp:uint16][Ki:uint16][Kd:uint16]                           */
#define LK_PARAM_SPEED_PID        0x0B /* 速度环 PID: [Kp:uint16][Ki:uint16][Kd:uint16]                           */
#define LK_PARAM_CURRENT_PID      0x0C /* 电流环 PID: [Kp:uint16][Ki:uint16][Kd:uint16]                           */
/* 限制参数 */
#define LK_PARAM_TORQUE_LIMIT     0x1E /* 最大力矩电流 (int16_t)                                                   */
#define LK_PARAM_SPEED_LIMIT      0x20 /* 最大速度 (int32_t)                                                       */
#define LK_PARAM_ANGLE_LIMIT      0x22 /* 角度限制 (int32_t)                                                        */
#define LK_PARAM_CURRENT_RAMP     0x24 /* 电流斜率 (int32_t)                                                        */
#define LK_PARAM_SPEED_RAMP       0x26 /* 速度斜率 (int32_t)                                                        */
/*----------------------------------------------- 控制参数范围 --------------------------------------------------*/
/* 开环控制 (仅 MS) */
#define LK_OPENLOOP_POWER_MIN     (-850)  /* 开环功率下限                                                            */
#define LK_OPENLOOP_POWER_MAX     850     /* 开环功率上限                                                            */
/* 转矩闭环控制 (仅 MF、MH、MG) */
#define LK_TORQUE_IQ_MIN          (-2048) /* 转矩电流 iqControl 下限                                                  */
#define LK_TORQUE_IQ_MAX          2048    /* 转矩电流 iqControl 上限                                                  */
/* MF 电机实际转矩电流: iqControl 映射到 -16.5A ~ 16.5A */
#define LK_MF_TORQUE_CURRENT_MIN    (-16.5f) /* MF 实际转矩电流下限 (A)                                                */
#define LK_MF_TORQUE_CURRENT_MAX    16.5f    /* MF 实际转矩电流上限 (A)                                                */
#define LK_MF_TORQUE_CURRENT_RES    (33.0f / 4096.0f) /* MF 转矩电流分辨率 (A/LSB)                                       */
/* MG 电机实际转矩电流: iqControl 映射到 -33A ~ 33A */
#define LK_MG_TORQUE_CURRENT_MIN    (-33.0f) /* MG 实际转矩电流下限 (A)                                               */
#define LK_MG_TORQUE_CURRENT_MAX    33.0f    /* MG 实际转矩电流上限 (A)                                               */
#define LK_MG_TORQUE_CURRENT_RES    (66.0f / 4096.0f) /* MG 转矩电流分辨率 (A/LSB)                                       */
/* 速度闭环控制 */
#define LK_SPEED_UNIT             0.01f   /* 速度单位: 0.01 dps/LSB（int32_t）                                       */
/* 角度控制 */
#define LK_ANGLE_UNIT             0.01f   /* 角度单位: 0.01°/LSB                                                      */
#define LK_SINGLE_CIRCLE_DEG      36000   /* 单圈 360° = 36000 (×0.01°)                                              */
/*----------------------------------------------- 电机系列分类 --------------------------------------------------*/
typedef enum {
    LK_SERIES_MF = 0,   /* MF 系列 — 支持转矩闭环、状态3、相电流读取                         */
    LK_SERIES_MH,       /* MH 系列 — 支持转矩闭环、状态3                                    */
    LK_SERIES_MG,       /* MG 系列 — 支持转矩闭环、状态3（双倍电流分辨率）                  */
    LK_SERIES_MS,       /* MS 系列 — 支持开环控制、无状态3                                 */
} lk_motor_series_t;
/*----------------------------------------------- 系列差异对照 -------------------------------------------------*/
/*  特性              │ MF              │ MH           │ MG              │ MS                */
/*  ─────────────────│─────────────────│──────────────│─────────────────│────────────────── */
/*  转矩闭环          │ ✓               │ ✓            │ ✓               │ ✗                 */
/*  开环控制          │ ✗               │ ✗            │ ✗               │ ✓                 */
/*  状态3 (相电流)    │ ✓               │ ✓            │ ✓               │ ✗                 */
/*  转矩电流分辨率    │ (33/4096) A/LSB │ —            │ (66/4096) A/LSB │ —                 */
/*  相电流分辨率      │ (33/4096) A/LSB │ —            │ (66/4096) A/LSB │ —                 */
/*  状态2 中的值      │ 转矩电流 iq     │ 转矩电流 iq  │ 转矩电流 iq     │ 输出功率 power    */
/*  编码器位数(根据电机)│ 14/15/16 bit    │ 14/15/16 bit │ 14/15/16 bit    │ 14/15/16 bit      */
#endif /* __LK_MOTOR_DEF_H__ */
