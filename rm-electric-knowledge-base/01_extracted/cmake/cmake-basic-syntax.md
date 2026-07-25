# CMake 基本语法

> 纯语法操作，不涉及编译流程和 target 概念。

---

## set - 设置变量

普通变量只在当前作用域有效，CACHE 变量写进 `CMakeCache.txt` 跨配置保留。

```cmake
set(<var> <value> [CACHE <type> <docstring> [FORCE]])
```

| 参数            | 填什么                         | 什么意思                               |
| ------------- | --------------------------- | ---------------------------------- |
| `<var>`       | `ROBOT`                     | 变量名                                |
| `<value>`     | `"sentry"` 或 `"${OLD} new"` | 值，支持 `"${OLD_VAR} new_value"` 追加语法 |
| `CACHE`       | 固定关键字                       | 可选，声明为缓存变量                         |
| `<type>`      | `STRING` / `BOOL`           | 缓存变量类型（仅 CMake GUI 有用）             |
| `<docstring>` | `"Target robot"`            | 缓存变量描述（仅 CMake GUI 有用）             |
| `FORCE`       | 可选                          | 强制覆盖缓存中的旧值                         |

在项目中的实际展示：

```cmake
# board/dji_c/CMakeLists.txt — 缓存变量，强制覆盖
set(CONFIG_CHERRYUSB_DEVICE ON CACHE BOOL "..." FORCE)

# config.cmake — 变量名拼接
string(TOUPPER ${BOARD} BOARD_UPPER)    # "single" → "SINGLE"
set(${BOARD_UPPER}_BOARD 1)              # 等价于 set(SINGLE_BOARD 1)
```

### 追加语法

用 `${OLD_VAR} new_value` 在已有变量末尾追加内容。常用于累积编译选项和链接选项：

```cmake
# gcc-arm-none-eabi-common.cmake — 多次追加 CMAKE_C_FLAGS
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")       # 追加 MCU 架构标志
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -ffunction-sections")  # 追加通用标志
# 最终 = "-mcpu=cortex-m3 -Wall -ffunction-sections"
```

第一次设时 `CMAKE_C_FLAGS` 为空，但写成追加语法可以保证多次 `set` 时不会覆盖前一次的值。

### CACHE 变量 — 跨配置持久化


```cmake
# apps/config.cmake
set(ROBOT "sentry" CACHE STRING "Target robot")
set(BOARD "gimbal" CACHE STRING "Board role")
```

CACHE 变量以首次配置时的值为准。改了 config.cmake 默认值后，旧 build 目录不会自动更新，需要用 `-D` 覆盖或删 build 目录重新配置：

```bash
cmake -S board/dji_c -B build/dji_c/Debug -DROBOT=infantry3 -DBOARD=single
```

可选值限制仅用于 CMake GUI 显示下拉菜单，不限制命令行传值。实际校验靠：

```cmake
set_property(CACHE ROBOT PROPERTY STRINGS hero engineer infantry3 ...)
if(NOT BOARD MATCHES "^(single|gimbal|chassis)$")
    message(FATAL_ERROR "Unknown BOARD '${BOARD}'")
endif()
```

---

## string - 字符串操作

```cmake
string(TOUPPER <input> <output_var>)
```

| 参数             | 填什么           | 什么意思  |
| -------------- | ------------- | ----- |
| `<input>`      | `"sentry"`    | 输入字符串 |
| `<output_var>` | `ROBOT_UPPER` | 输出变量名 |
|                |               |       |

在项目中的实际展示：

```cmake
# apps/config.cmake — 转换机器人名称为大写，用于拼接宏
string(TOUPPER ${ROBOT} ROBOT_UPPER)   # "sentry" → "SENTRY"
```

---

## if - 条件判断

```cmake
if(<condition>)
    ...
elseif(<condition>)
    ...
else()
    ...
endif()
```

| 写法                             | 何时为真                |
| ------------------------------ | ------------------- |
| `if(MODULE_BMI088)`            | 变量值 = 1 / ON / TRUE |
| `if(NOT DEFINED THREADX_ARCH)` | 变量未定义               |
| `if(EXISTS ${path})`           | 路径存在                |
| `if(BOARD STREQUAL "single")`  | 字符串相等               |
| `if(TARGET CMSISDSP)`          | 目标已定义               |

在项目中依靠cmake变量的值(1/0)条件添加需要编译的.c文件,注意与宏定义的条件编译区别

```cmake
# modules/CMakeLists.txt — 模块开关控制条件编译
if(MODULE_BMI088)
    list(APPEND _module_sources BMI088/module_bmi088.c)
endif()

# robot/CMakeLists.txt — 检查目标是否存在
if(TARGET CMSISDSP)
    target_link_libraries(robot PUBLIC CMSISDSP)
endif()
```

---

## list - 列表操作

CMake 中空格分隔的字符串就是列表。

```cmake
list(APPEND <list> <element>)
list(REMOVE_ITEM <list> <element>)
list(REMOVE_DUPLICATES <list>)
```

| 参数 | 填什么 | 什么意思 |
|------|--------|----------|
| `<list>` | `_sources` | 列表变量名 |
| `<element>` | `a.c` | 要追加/删除的元素 |

在项目中的实际展示：

```cmake
# modules/CMakeLists.txt — 条件累加源文件
set(_module_sources module_init.c algorithm/pid.c)
if(MODULE_BMI088)
    list(APPEND _module_sources BMI088/module_bmi088.c)
endif()

# board_common.cmake — 移除隐式链接库
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
```

---

## foreach - 循环

```cmake
foreach(<var> <item1> <item2> ...)
    ...
endforeach()
```

| 参数 | 填什么 | 什么意思 |
|------|--------|----------|
| `<var>` | `_m` | 循环变量名 |
| `<items>` | `OFFLINE REMOTE MOTOR` | 要遍历的列表 |

在项目中的实际展示：

```cmake
# apps/config.cmake — 初始化所有模块开关为 0
foreach(_m OFFLINE REMOTE BMI088 INS REFEREE SUPERCAP WT606 MOTOR VISION BOARDCOMM)
    set(MODULE_${_m} 0)
endforeach()

# apps/config.cmake — 把启用的模块设为 1
set(_enabled ${MODULES_${BOARD_UPPER}})
foreach(_m ${_enabled})
    set(MODULE_${_m} 1)
endforeach()
```

---

## file - 文件操作

### file(WRITE)

```cmake
file(WRITE <path> <content>)
```

| 参数 | 填什么 | 什么意思 |
|------|--------|----------|
| `<path>` | `${_generated_dir}/robot_def.h` | 输出文件路径 |
| `<content>` | `"#define FOO 1\n"` | 写入的内容 |

```cmake
# 旧版 generate_headers.cmake
file(WRITE ${_generated_dir}/robot_def.h
"#ifndef _ROBOT_DEF_H_
#define _ROBOT_DEF_H_
#define CURRENT_ROBOT   ROBOT_${ROBOT_UPPER}
#endif
")
```

### file(GLOB_RECURSE) - 递归扫描文件

```cmake
file(GLOB_RECURSE <output_var> [CONFIGURE_DEPENDS] <globs>)
```

| 参数                  | 填什么              | 什么意思           |
| ------------------- | ---------------- | -------------- |
| `<output_var>`      | `_robot_sources` | 输出列表变量名        |
| `CONFIGURE_DEPENDS` | 可选               | 每次构建重新检查文件列表变化 |
| `<globs>`           | `${dir}/*.c`     | 匹配模式           |

```cmake
# apps/CMakeLists.txt
file(GLOB_RECURSE _robot_sources CONFIGURE_DEPENDS ${_robot_board_dir}/*.c)
```

---

## get_filename_component - 提取路径组件

```cmake
get_filename_component(<output_var> <path> <mode>)
```

| 参数             | 填什么                               | 什么意思  |
| -------------- | --------------------------------- | ----- |
| `<output_var>` | `MAS_ROOT`                        | 输出变量名 |
| `<path>`       | `${CMAKE_CURRENT_LIST_DIR}/..`    | 输入路径  |
| `<mode>`       | `DIRECTORY` / `NAME` / `ABSOLUTE` | 提取模式  |

| 模式 | 作用 | 示例 |
|------|------|------|
| `DIRECTORY` | 取目录部分 | `E:/a/b/c.txt` → `E:/a/b` |
| `NAME` | 取文件名部分 | `E:/a/b/c.txt` → `c.txt` |
| `ABSOLUTE` | 转绝对路径 | `../a/b` → `E:/project/a/b` |

```cmake
# cmake/board_common.cmake — 计算仓库根目录绝对路径
get_filename_component(MAS_ROOT ${CMAKE_CURRENT_LIST_DIR}/.. ABSOLUTE)
```

---

## function - 自定义函数

```cmake
function(<name> <arg1> <arg2> ...)
    ...
endfunction()
```

| 参数 | 填什么 | 什么意思 |
|------|--------|----------|
| `<name>` | `_gen_cmakedefine` | 函数名 |
| `<args>` | `OUT_VAR NAME VALUE_VAR` | 形参列表 |

```cmake
# 旧版 generate_headers.cmake — 模拟 #cmakedefine 行为
function(_gen_cmakedefine OUT_VAR NAME VALUE_VAR)
    if(DEFINED ${VALUE_VAR}
       AND NOT "${${VALUE_VAR}}" STREQUAL ""
       AND NOT "${${VALUE_VAR}}" MATCHES "^(FALSE|OFF|0|N|IGNORE|NOTFOUND|.*-NOTFOUND)$")
        set(${OUT_VAR} "#define ${NAME} ${${VALUE_VAR}}" PARENT_SCOPE)
    else()
        set(${OUT_VAR} "/* #undef ${NAME} */" PARENT_SCOPE)
    endif()
endfunction()
```

`PARENT_SCOPE` 是因为函数有自己的作用域，`set` 默认只改函数内部变量，加 `PARENT_SCOPE` 才能把结果传回调用者。

---

## include - 粘贴到当前位置

把指定文件的内容**粘贴到当前位置执行**，共享当前作用域，变量直接生效。

```cmake
include(<path>)
```

| 参数       | 填什么                             | 什么意思            |
| -------- | ------------------------------- | --------------- |
| `<path>` | `${MAS_ROOT}/apps/config.cmake` | 要执行的 CMake 文件路径 |

```cmake
# cmake/board_common.cmake — 加载配置链
include(${MAS_ROOT}/apps/config.cmake)
```

```
board_common.cmake
  └─ include(config.cmake)
       ├─ include(module_config.cmake)   ← 默认参数
       └─ include(sentry/robot.cmake)    ← 差异覆盖
```

---

## configure_file - 模板替换生成文件

把 `.h.in` 模板中的 `@VAR@` 替换为 CMake 变量值，生成 `.h` 头文件。

```cmake
configure_file(<input> <output> [@ONLY])
```

| 参数         | 填什么                               | 什么意思                     |
| ---------- | --------------------------------- | ------------------------ |
| `<input>`  | `${MAS_ROOT}/apps/robot_def.h.in` | 模板文件路径，含 `@VAR@` 占位符     |
| `<output>` | `${_generated_dir}/robot_def.h`   | 生成的头文件路径                 |
| `@ONLY`    | 固定关键字                             | 只替换 `@VAR@`，不替换 `${VAR}` |

```cmake
# cmake/board_common.cmake
configure_file(${MAS_ROOT}/apps/robot_def.h.in ${_generated_dir}/robot_def.h @ONLY)
configure_file(${MAS_ROOT}/apps/module_config.h.in ${_generated_dir}/module_config.h @ONLY)
```

模板中支持两种语法：

| 模板语法 | 行为 |
|---------|------|
| `@VAR@` | 无条件替换为 CMake 变量值，变量不存在时报错 |
| `#cmakedefine VAR @VAR@` | 变量存在且非假值时生成 `#define`，否则生成 `/* #undef */` |

```c
// module_config.h.in 模板
#define MODULE_REMOTE @MODULE_REMOTE@
#cmakedefine REMOTE_UART @REMOTE_UART@
```

`MODULE_REMOTE=1`、`REMOTE_UART=huart4` 时生成：

```c
#define MODULE_REMOTE 1
#define REMOTE_UART huart4
```

`#cmakedefine` 判定规则：变量已定义，且值不为 `FALSE` / `OFF` / `0` / `N` / `IGNORE` / `NOTFOUND` 之一时生成 `#define`，否则生成 `/* #undef */`。

---

## CONFIGURE_DEPENDS - 自动检测文件变化

```cmake
file(GLOB_RECURSE <output_var> CONFIGURE_DEPENDS <globs>)
```

不加 `CONFIGURE_DEPENDS` 时，`GLOB_RECURSE` 只在 `cmake -S ...` 配置时扫描一次。之后新增 `.c` 文件，CMake 不会知道，必须手动重新配置。

加上后，每次构建时 Ninja 检查文件列表是否有变化，有变化就自动重新配置。

```cmake
# apps/CMakeLists.txt
file(GLOB_RECURSE _robot_sources CONFIGURE_DEPENDS ${_robot_board_dir}/*.c)
```

---

## message - 打印信息

```cmake
message([<mode>] <text>)
```

| 模式 | 行为 |
|------|------|
| （无） | 普通信息，继续执行 |
| `FATAL_ERROR` | 打印后停止配置 |

```cmake
# board_common.cmake — 打印构建类型
message("Build type: " ${CMAKE_BUILD_TYPE})

# config.cmake — 校验 BOARD 值，非法时停止
if(NOT BOARD MATCHES "^(single|gimbal|chassis)$")
    message(FATAL_ERROR "Unknown BOARD '${BOARD}'")
endif()
```