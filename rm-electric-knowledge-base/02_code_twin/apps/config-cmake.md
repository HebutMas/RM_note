# config.cmake — 机器人配置入口

## 文件位置

`mas_embedded_threadx/apps/config.cmake`

## 作用

被 [[02_code_twin/cmake/board_common-cmake]] 通过 `include()` 加载。是整个配置链的起点——你在这里选机器人型号和板型，CMake 据此决定编译哪些模块、用什么参数。

---

## 原始代码（摘取）

### 选择机器人 & 板型（CACHE 变量）

```cmake
set(ROBOT "sentry" CACHE STRING "Target robot")  # hero / engineer / infantry3 / ...
set(BOARD "gimbal" CACHE STRING "Board role")    # single / gimbal / chassis
```

- `set(CACHE)` → [[01_extracted/cmake/cmake-basic-syntax#CACHE 变量 - 跨配置持久化]]

改这两个值就能切换编译目标。`CACHE` 变量写进 `CMakeCache.txt`，下次配置时自动读取。

**切换方式：**

```bash
# 方式一：改 config.cmake 默认值
set(ROBOT "infantry3")

# 方式二：命令行覆盖（不修改文件）
cmake -S board/dji_c -B build/dji_c/Debug -DROBOT=infantry3 -DBOARD=single
```

### 可选值校验

```cmake
set_property(CACHE ROBOT PROPERTY STRINGS hero engineer infantry3 infantry4 infantry5 drone sentry darts)
```

- `set_property` → [[01_extracted/cmake/cmake-basic-syntax#set_property(CACHE) - 限制可选值]]

仅用于 CMake GUI 显示下拉菜单，不限制命令行传值。实际校验：

```cmake
if(NOT BOARD MATCHES "^(single|gimbal|chassis)$")
    message(FATAL_ERROR "Unknown BOARD '${BOARD}'")
endif()
```

### 加载默认配置和差异配置

```cmake
include(${CMAKE_CURRENT_LIST_DIR}/../modules/module_config.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/${ROBOT}/robot.cmake)
```

先加载 [[02_code_twin/modules/module_config-cmake]]（默认参数模板），再加载 [[02_code_twin/apps/sentry/robot-cmake]]（**机器人差异配置，覆盖默认值）**。

### 派生变量

```cmake
string(TOUPPER ${ROBOT} ROBOT_UPPER)   # "sentry" → "SENTRY"
string(TOUPPER ${BOARD} BOARD_UPPER)   # "gimbal" → "GIMBAL"

set(SINGLE_BOARD  0)
set(GIMBAL_BOARD  0)
set(CHASSIS_BOARD 0)
set(${BOARD_UPPER}_BOARD 1)             # 等价于 set(GIMBAL_BOARD 1)
```

- `string` → [[01_extracted/cmake/cmake-basic-syntax#string - 字符串操作]]

通过字符串拼接动态设置板型宏——`BOARD=gimbal` 时只有 `GIMBAL_BOARD=1`，其余为 0。

### 模块开关

```cmake
foreach(_m OFFLINE REMOTE BMI088 INS REFEREE SUPERCAP WT606 MOTOR VISION BOARDCOMM)
    set(MODULE_${_m} 0)
endforeach()

set(_enabled ${MODULES_${BOARD_UPPER}})
foreach(_m ${_enabled})
    set(MODULE_${_m} 1)
endforeach()
```

- `foreach` → [[01_extracted/cmake/cmake-basic-syntax#foreach - 循环]]

第一步：所有模块开关初始化为 0（关闭）。第二步：从 `MODULES_GIMBAL` 取出启用的模块列表，把对应的 `MODULE_XXX` 设为 1。

这些 `MODULE_XXX` 变量后续被 `configure_file` 翻译成 C 宏写入 `module_config.h`，又被 [[02_code_twin/modules/CMakeLists-txt]] 中的 `if(MODULE_XXX)` 用于条件编译。

---

## 配置链一览

```
board_common.cmake
  └─ include(config.cmake)
       ├─ include(module_config.cmake)    ← 默认参数
       │    └─ 定义 MODULES_GIMBAL/SINGLE/CHASSIS 的默认值
       │
       └─ include(sentry/robot.cmake)     ← 覆盖差异
            └─ 覆盖 MODULES_XXX、REMOTE_UART 等
```

## 生成的头文件

`config.cmake` 执行完毕后，`board_common.cmake` 调用 `configure_file` 将 CMake 变量写入 C 头文件：

- [[02_code_twin/apps/robot_def-h-in]] → `robot_def.h`：`#define CURRENT_ROBOT ROBOT_SENTRY`
- [[02_code_twin/apps/module_config-h-in]] → `module_config.h`：模块开关和参数宏