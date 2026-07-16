# robot.cmake (infantry3) — 机器人差异配置

## 文件位置

`mas_embedded_threadx/apps/infantry3/robot.cmake`

## 作用

被 [[02_code_twin/apps/config-cmake]] 通过 `include(${ROBOT}/robot.cmake)` 加载。在 [[02_code_twin/modules/module_config-cmake]] 的默认值基础上，覆盖 infantry3 机器人的差异配置。

---

## 原始代码（摘取）

```cmake
include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)
```

先加载默认模板。注意这里又 `include` 了一次 module_config.cmake——虽然 config.cmake 已经加载过一次，但 CMake 的 `include()` 是幂等的（相同文件不会重复执行），所以不会出问题。

### 覆盖模块列表

```cmake
set(MODULES_SINGLE   OFFLINE REMOTE BMI088 INS REFEREE SUPERCAP VISION MOTOR)
#set(MODULES_SINGLE   OFFLINE REMOTE BMI088 INS MOTOR)
```

infantry3 的 single 板比默认多了 `VISION` 模块。注释掉的那行是调试时临时精简模块列表用的。

### 覆盖差异参数

```cmake
set(OFFLINE_BEEP_ENABLE     0)      # 关闭蜂鸣器（默认是 1）
set(REMOTE_UART             huart3)  # 遥控器串口（和默认一致）
set(REMOTE_VT_UART          huart6)  # 图传串口改为 huart6（默认是 huart1）
set(REMOTE_SOURCE           1)       # SBUS 遥控器
set(REMOTE_VT_SOURCE        0)       # 不使用图传（默认是 1）
set(REFEREE_UART            huart6)  # 裁判系统串口改为 huart6（默认是 huart1）
```

infantry3 的主要差异：关掉蜂鸣器、图传串口从 huart1 改到 huart6、裁判系统串口也改成 huart6、不使用图传。这些值会覆盖 module_config.cmake 中的默认值，最终被 [[02_code_twin/apps/generate_headers-cmake]] 翻译成 C 宏。
