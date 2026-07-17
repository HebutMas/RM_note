# CMake 基本语法

> 纯语法操作，不涉及编译流程和 target 概念。就像学 C 语言先学变量、循环、判断。

---

## set - 设置变量

```cmake
set(MY_VAR "hello")              # 普通变量，当前作用域有效
set(MY_VAR "val" CACHE STRING "描述" FORCE)  # 缓存变量，写进 CMakeCache.txt，跨配置保留
```

普通变量就像 C 里的局部变量——出了作用域就没了。缓存变量写进 `CMakeCache.txt`，下次配置时还在。

`FORCE` 强制覆盖缓存中的旧值。本项目在设置 CherryUSB 配置项时用了：

```cmake
# board/dji_c/CMakeLists.txt
set(CONFIG_CHERRYUSB_DEVICE ON CACHE BOOL "..." FORCE)
```

### 变量名拼接

```cmake
string(TOUPPER ${BOARD} BOARD_UPPER)   # "single" → "SINGLE"
set(${BOARD_UPPER}_BOARD 1)             # 等价于 set(SINGLE_BOARD 1)
```

`${BOARD_UPPER}` 在 `set` 执行前被替换成 `SINGLE`，所以实际执行的是 `set(SINGLE_BOARD 1)`。

### 覆盖机制

config.cmake 先 `include` module_config.cmake 加载默认值，robot.cmake 再直接 `set` 覆盖：

```cmake
# modules/module_config.cmake（默认值）
set(OFFLINE_BEEP_ENABLE 1)

# apps/sentry/robot.cmake（覆盖）
set(OFFLINE_BEEP_ENABLE 0)
```

CMake 的 `set()` 会覆盖当前作用域中已有的变量，后加载的生效。

---

## string - 字符串操作

```cmake
string(TOUPPER ${ROBOT} ROBOT_UPPER)   # "sentry" → "SENTRY"
string(TOUPPER ${BOARD} BOARD_UPPER)   # "single" → "SINGLE"
```

`string(TOUPPER 输入 输出变量)` 把字符串转大写，结果存入输出变量。后面用 `ROBOT_${ROBOT_UPPER}` 拼接出 `ROBOT_SENTRY`。

---

## if - 条件判断

```cmake
if(MODULE_BMI088)          # 变量值为 1/ON/TRUE 时为真
    ...
elseif(MODULE_REMOTE)
    ...
else()
    ...
endif()

if(EXISTS ${path})              # 路径存在
if(NOT DEFINED THREADX_ARCH)    # 变量未定义
```

本项目 `modules/CMakeLists.txt` 中大量使用 `if(MODULE_XXX)` 做条件编译：

```cmake
if(MODULE_BMI088)
    list(APPEND _module_sources BMI088/module_bmi088.c)
endif()
```

---

## list - 列表操作

CMake 里用空格分隔的字符串就是列表：

```cmake
set(_sources a.c b.c c.c)    # _sources 是一个列表，有三个元素
```

```cmake
list(APPEND _sources d.c)              # 尾部追加 → a.c b.c c.c d.c
list(REMOVE_DUPLICATES _sources)       # 去重
list(REMOVE_ITEM _sources b.c)         # 删除指定元素
```

本项目 `modules/CMakeLists.txt` 大量使用 `list(APPEND ...)` 做条件累加：

```cmake
set(_module_sources module_init.c algorithm/pid.c)  # 基础列表

if(MODULE_BMI088)
    list(APPEND _module_sources BMI088/module_bmi088.c)  # 条件追加
endif()
```

---

## foreach - 循环

```cmake
# apps/config.cmake — 初始化所有模块开关为 0
foreach(_m OFFLINE REMOTE BMI088 INS REFEREE SUPERCAP WT606 MOTOR VISION BOARDCOMM)
    set(MODULE_${_m} 0)
endforeach()
```

等价于手写 10 行 `set(MODULE_OFFLINE 0)`、`set(MODULE_REMOTE 0)`...

```cmake
# 第二个 foreach：把启用的模块设为 1
set(_enabled ${MODULES_${BOARD_UPPER}})   # 取出 MODULES_SINGLE 列表
foreach(_m ${_enabled})
    set(MODULE_${_m} 1)
endforeach()
```

### foreach 遍历文件提取路径

```cmake
# apps/CMakeLists.txt
foreach(_h ${_robot_headers})
    get_filename_component(_dir ${_h} DIRECTORY)  # 取文件所在目录
    list(APPEND _robot_includes ${_dir})           # 加入头文件路径列表
endforeach()
```

---

## file - 文件操作

### file(WRITE) - 写文件

```cmake
# apps/generate_headers.cmake
file(WRITE ${_generated_dir}/robot_def.h
"#ifndef _ROBOT_DEF_H_
#define _ROBOT_DEF_H_
#define CURRENT_ROBOT   ROBOT_${ROBOT_UPPER}
#endif
")
```

直接写一个头文件到磁盘。`${ROBOT_UPPER}` 在配置阶段被替换为 `SENTRY`。

### file(GLOB_RECURSE) - 递归扫描文件

递归扫描 `apps/sentry/gimbal_board/` 下所有子目录的 `.c` 文件，收集到 `_robot_sources` 变量。

**实例:**
```cmake
# apps/CMakeLists.txt
file(GLOB_RECURSE _robot_sources ${_robot_board_dir}/*.c)
```

```c
// 伪代码：file(GLOB_RECURSE _robot_sources ${_robot_board_dir}/*.c)
function collect_files(dir):
    for each item in dir:
        if item是文件夹:
            collect_files(item)              // 递归进入子目录
        else if item以 ".c" 结尾:
            _robot_sources.push_back(item)   // 收集文件路径
```
从 `_robot_board_dir` 开始，像“地毯式搜索”一样遍历它和所有子孙文件夹，把碰到的每个 `.c` 文件的完整路径都存进 `_robot_sources` 这个列表里。

### get_filename_component - 提取路径组件

```cmake
get_filename_component(_dir ${_h} DIRECTORY)  # apps/foo/chassis/bar.h → apps/foo/chassis/
```

---

## function - 自定义函数

```cmake
# apps/generate_headers.cmake
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

模拟 CMake 的 `#cmakedefine` 行为：变量有值就生成 `#define NAME value`，否则生成 `/* #undef NAME */`。

`PARENT_SCOPE` 是因为函数有自己的作用域，`set` 默认只改函数内部变量，加 `PARENT_SCOPE` 才能把结果传回调用者。

---

## include - 粘贴到当前位置

```cmake
# board/dji_c/CMakeLists.txt
include(${CMAKE_SOURCE_DIR}/../../apps/config.cmake)
```

`include()` 把指定文件的内容**粘贴到当前位置执行**，共享当前作用域，变量直接生效。执行完后回到下一行继续。

### include 的展开链

```
板级 CMakeLists.txt
  └─ include(config.cmake)
       ├─ include(module_config.cmake)    ← 加载默认参数
       └─ include(sentry/robot.cmake)  ← 加载差异配置
            └─ include(module_config.cmake)  ← 重复 include，幂等不重复执行
```

