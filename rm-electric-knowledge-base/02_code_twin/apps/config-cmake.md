# config.cmake — 机器人配置入口

## 文件位置

`mas_embedded_threadx/apps/config.cmake`

## 作用

被 [[02_code_twin/board/dji_c/CMakeLists-txt]] 通过 `include()` 加载。是整个配置链的起点——你在这里选机器人型号和板型，CMake 据此决定编译哪些模块、用什么参数。

---

## 原始代码（摘取）

### 选择机器人 & 板型

```cmake
set(ROBOT "infantry3")
set(BOARD "single")
```

改这两个值就能切换编译目标。`ROBOT` 决定业务代码目录（`apps/infantry3/`），`BOARD` 决定板型宏（`SINGLE_BOARD=1`）。

### 加载默认配置和差异配置

```cmake
include(${CMAKE_CURRENT_LIST_DIR}/../modules/module_config.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/${ROBOT}/robot.cmake)
```

先加载 [[02_code_twin/modules/module_config-cmake]]（默认参数模板），再加载 [[02_code_twin/apps/infantry3-robot-cmake]]（机器人差异配置，覆盖默认值）。

### 派生变量

```cmake
string(TOUPPER ${ROBOT} ROBOT_UPPER)   # "infantry3" → "INFANTRY3"
string(TOUPPER ${BOARD} BOARD_UPPER)   # "single" → "SINGLE"

set(SINGLE_BOARD  0)
set(GIMBAL_BOARD  0)
set(CHASSIS_BOARD 0)
set(${BOARD_UPPER}_BOARD 1)             # 等价于 set(SINGLE_BOARD 1)
```

- `string` → [[01_extracted/cmake-basic-syntax#string - 字符串操作]]

通过字符串拼接动态设置板型宏——`BOARD=single` 时只有 `SINGLE_BOARD=1`，其余为 0。

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

- `MODULE_INFANTY3` → [[02_code_twin/apps/infantry3-robot-cmake#覆盖模块列表]]
- `foreach` → [[01_extracted/cmake-basic-syntax#foreach - 循环]]

第一步：所有模块开关初始化为 0（关闭）。

第二步：从 `MODULES_SINGLE`（在 [[02_code_twin/modules/module_config-cmake]] 或 [[02_code_twin/apps/infantry3-robot-cmake]] 中定义）取出启用的模块列表，把对应的 `MODULE_XXX` 设为 1。

以 `infantry3 single` 为例，`MODULES_SINGLE` 最终值（被 robot.cmake 覆盖后）是 `OFFLINE REMOTE BMI088 INS REFEREE SUPERCAP VISION MOTOR`，所以这 8 个模块的 `MODULE_XXX` 被设为 1，其余保持 0。

这些 `MODULE_XXX` 变量后续被 [[02_code_twin/apps/generate_headers-cmake]] 翻译成 C 宏写入 `module_config.h`，又被 [[02_code_twin/modules/CMakeLists-txt]] 中的 `if(MODULE_XXX)` 用于条件编译。
