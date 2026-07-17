# generate_headers.cmake — CMake 变量到 C 宏的翻译器

## 文件位置

`mas_embedded_threadx/apps/generate_headers.cmake`

## 作用

被 [[02_code_twin/board/dji_c/CMakeLists-txt]] 通过 `include()` 加载。读取 config.cmake 和 robot.cmake 设置的 CMake 变量，用 `file(WRITE ...)` 生成两个 C 头文件到 `build/dji_c/Debug/generated/` 目录。

---

## 配置链全貌

```
config.cmake                      设置 ROBOT/BOARD/MODULE_XXX 等变量
  ├→ include module_config.cmake    加载默认参数
  └→ include robot.cmake            覆盖差异参数
                                     ↓
generate_headers.cmake           把变量翻译成 C 宏
  ├→ file(WRITE) robot_def.h        生成机器人类型宏
  └→ file(WRITE) module_config.h    生成模块开关和参数宏
                                     ↓
build/dji_c/Debug/generated/     产物目录
  ├→ robot_def.h                    #define CURRENT_ROBOT ROBOT_SENTRY
  └→ module_config.h                #define MODULE_VISION 1 ...
                                     ↓
源代码 #include "module_config.h"   C 代码中使用宏
```

---

## 原始代码（摘取）

### 生成 robot_def.h

```cmake
file(WRITE ${_generated_dir}/robot_def.h
"#ifndef _ROBOT_DEF_H_
#define _ROBOT_DEF_H_

#define ROBOT_HERO      1
#define ROBOT_ENGINEER  2
#define ROBOT_INFANTRY3 3
...
#define ROBOT_SENTRY    7
...
#define CURRENT_ROBOT   ROBOT_${ROBOT_UPPER}

#endif
")
```

- `file` → [[01_extracted/cmake/cmake-basic-syntax#file - 文件操作]]

`file(WRITE 路径 内容)` 直接写文件。`${ROBOT_UPPER}` 在配置阶段被替换为 `SENTRY`，所以最终写入的文件内容是：

```c
#define CURRENT_ROBOT   ROBOT_SENTRY
```

### _gen_cmakedefine 函数

```cmake
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

- `function` → [[01_extracted/cmake/cmake-basic-syntax#function - 自定义函数]]

这个函数模拟了 CMake 的 `#cmakedefine` 行为：如果变量已定义且不是 FALSE/0/空，就生成 `#define NAME value`；否则生成 `/* #undef NAME */`（注释掉）。

用 `PARENT_SCOPE` 是因为函数有自己的作用域，`set` 默认只改函数内部的变量，加 `PARENT_SCOPE` 才能把结果传回调用者。

```cmake
_gen_cmakedefine(_REMOTE_UART        REMOTE_UART        REMOTE_UART)
```

调用后，`_REMOTE_UART` 变量的值要么是 `#define REMOTE_UART huart3`，要么是 `/* #undef REMOTE_UART */`。

### 生成 module_config.h

```cmake
file(WRITE ${_generated_dir}/module_config.h
"...
#define MODULE_OFFLINE   ${MODULE_OFFLINE}
#define MODULE_REMOTE    ${MODULE_REMOTE}
#define MODULE_BMI088    ${MODULE_BMI088}
...
${_REMOTE_UART}
${_REFEREE_UART}
${_WT606_UART}
${_BOARDCOMM_CAN}
...
")
```

以 `sentry gimbal` 为例，`MODULES_GIMBAL` = `OFFLINE REMOTE BMI088 INS WT606 MOTOR VISION BOARDCOMM`，最终生成的文件类似：

```c
#define SINGLE_BOARD  0
#define GIMBAL_BOARD  1
#define CHASSIS_BOARD 0

#define MODULE_OFFLINE   1
#define MODULE_REMOTE    1
#define MODULE_BMI088    1
#define MODULE_INS       1
#define MODULE_REFEREE   0
#define MODULE_SUPERCAP  0
#define MODULE_WT606     1
#define MODULE_MOTOR     1
#define MODULE_VISION    1
#define MODULE_BOARDCOMM 1

#define REMOTE_UART huart3
#define REMOTE_VT_UART huart6
#define WT606_UART huart1
#define BOARDCOMM_CAN BSP_CAN_HANDLE2
/* #undef REFEREE_UART */
/* #undef SUPERCAP_CAN */
...
```

`REFEREE_UART` 和 `SUPERCAP_CAN` 被注释掉——因为 sentry gimbal 不启用 REFEREE 和 SUPERCAP 模块，对应的 UART/CAN 变量虽然在默认模板中定义了，但 `_gen_cmakedefine` 检测到模块未启用时不会生成 `#define`。

这个头文件被 [[02_code_twin/modules/CMakeLists-txt]] 和 [[02_code_twin/apps/CMakeLists-txt]] 通过 `-include module_config.h` 强制注入到每个 `.c` 文件中。
