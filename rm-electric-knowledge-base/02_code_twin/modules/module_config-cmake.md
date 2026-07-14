# module_config.cmake — 模块默认参数模板

## 文件位置

`mas_embedded_threadx/modules/module_config.cmake`

## 作用

被 [[02_code_twin/apps/config-cmake]] 和 [[02_code_twin/apps/infantry3-robot-cmake]] 通过 `include()` 加载。定义所有模块的默认参数和各板型的默认模块列表。是配置链的"基类"，robot.cmake 在此基础上覆盖差异项。

---

## 原始代码（摘取）

### 各板型的默认模块列表

```cmake
set(MODULES_SINGLE   OFFLINE REMOTE BMI088 INS REFEREE SUPERCAP MOTOR)
set(MODULES_GIMBAL   OFFLINE REMOTE BMI088 INS MOTOR VISION BOARDCOMM)
set(MODULES_CHASSIS  OFFLINE BMI088 INS REFEREE SUPERCAP MOTOR BOARDCOMM)
```

不同板型启用不同模块。比如底盘板（CHASSIS）不需要遥控器（REMOTE）和视觉（VISION），但需要板间通信（BOARDCOMM）。这些值会被 [[02_code_twin/apps/config-cmake]] 中的 `MODULES_${BOARD_UPPER}` 取出，也允许被 robot.cmake 覆盖。

### 默认参数

```cmake
set(OFFLINE_WATCHDOG_ENABLE 1)
set(OFFLINE_BEEP_ENABLE     1)
set(OFFLINE_TASK_STACK_SIZE 1024)
set(OFFLINE_TASK_PRIORITY   6)

set(REMOTE_UART             huart3)
set(REMOTE_VT_UART          huart1)
set(REMOTE_SOURCE           1)
...

set(REFEREE_UART            huart1)
set(REFEREE_TASK_STACK_SIZE 1024)
...

set(SUPERCAP_CAN            BSP_CAN_HANDLE2)
...
```

每个模块有一组参数：任务栈大小、任务优先级、使用的串口/CAN 句柄等。这些值后续被 [[02_code_twin/apps/generate_headers-cmake]] 翻译成 C 宏写入 `module_config.h`，C 代码里直接 `#define` 引用。

**覆盖机制**：robot.cmake 先 `include` 本文件加载默认值，然后直接 `set(变量名 新值)` 覆盖差异项。CMake 的 `set()` 会覆盖当前作用域中已有的变量，所以后加载的 robot.cmake 中同名变量生效。
