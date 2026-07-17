# robot.cmake (sentry) — 哨兵差异配置

## 文件位置

`mas_embedded_threadx/apps/sentry/robot.cmake`

## 作用

被 [[02_code_twin/apps/config-cmake]] 通过 `include(${ROBOT}/robot.cmake)` 加载。在 [[02_code_twin/modules/module_config-cmake]] 的默认值基础上，覆盖哨兵机器人的差异配置。

---

## 原始代码

```cmake
# 先加载默认模板，再覆盖差异
include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)
```

先加载默认模板。注意这里又 `include` 了一次 module_config.cmake——虽然 config.cmake 已经加载过一次，但 CMake 的 `include()` 是幂等的（相同文件不会重复执行），所以不会出问题。

### 覆盖模块列表

```cmake
set(MODULES_GIMBAL   OFFLINE REMOTE BMI088 INS WT606 MOTOR VISION BOARDCOMM)
set(MODULES_CHASSIS  OFFLINE BMI088 INS REFEREE MOTOR BOARDCOMM)
```

哨兵有两个板型，各自启用不同模块：

| 模块 | 云台板 (gimbal) | 底盘板 (chassis) | 说明 |
|------|:---:|:---:|------|
| OFFLINE | 1 | 1 | 离线检测 |
| REMOTE | 1 | 0 | 遥控器（只有云台板接遥控器） |
| BMI088 | 1 | 1 | IMU |
| INS | 1 | 1 | 姿态解算 |
| REFEREE | 0 | 1 | 裁判系统（底盘板接裁判系统） |
| SUPERCAP | 0 | 0 | 超级电容（哨兵未启用） |
| WT606 | 1 | 0 | 陀螺仪模块 |
| MOTOR | 1 | 1 | 电机 |
| VISION | 1 | 0 | 视觉（只有云台板接迷你主机） |
| BOARDCOMM | 1 | 1 | 板间通信（云台 ↔ 底盘） |

### 覆盖差异参数

```cmake
# OFFLINE 参数
set(OFFLINE_BEEP_ENABLE     0)    # 关闭蜂鸣器

# REMOTE 参数
set(REMOTE_UART             huart3) # 串口
set(REMOTE_VT_UART          huart6) # 图传串口
set(REMOTE_SOURCE           1)      # 遥控器选择: 0=none, 1=sbus, 2=dt7
set(REMOTE_VT_SOURCE        0)      # 图传选择:   0=none, 1=vt02, 2=vt03

# REFEREE 参数
set(REFEREE_UART            huart6) # 串口选择

# WT606 参数
set(WT606_UART              huart1) # 串口选择

# BOARDCOMM 参数
set(BOARDCOMM_CAN           BSP_CAN_HANDLE2) # CAN 句柄
```

哨兵的主要差异：

| 参数 | 默认值 | 哨兵值 | 说明 |
|------|--------|--------|------|
| `OFFLINE_BEEP_ENABLE` | 1 | 0 | 关闭蜂鸣器 |
| `REMOTE_VT_UART` | huart1 | huart6 | 图传串口改到 huart6 |
| `REMOTE_VT_SOURCE` | 1 | 0 | 不使用图传 |
| `REFEREE_UART` | huart1 | huart6 | 裁判系统串口改到 huart6 |
| `WT606_UART` | huart1 | huart1 | WT606 串口（和默认一致） |
| `BOARDCOMM_CAN` | BSP_CAN_HANDLE2 | BSP_CAN_HANDLE2 | 板间通信 CAN（和默认一致） |

这些值覆盖 module_config.cmake 中的默认值，最终被 [[02_code_twin/apps/generate_headers-cmake]] 翻译成 C 宏。
