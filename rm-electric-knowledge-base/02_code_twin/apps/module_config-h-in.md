# module_config.h.in — 模块配置宏模板

## 文件位置

`mas_embedded_threadx/apps/module_config.h.in`

## 作用

`configure_file` 的输入模板。`board_common.cmake` 执行 `configure_file(module_config.h.in module_config.h @ONLY)` 时，将 `@VAR@` 替换为 CMake 变量值，生成 `module_config.h`。

> 本文件替代了旧版 `generate_headers.cmake` 中的 `file(WRITE module_config.h)`。

---

## 原始代码（摘取）

### 板型宏

```c
#define SINGLE_BOARD  @SINGLE_BOARD@
#define GIMBAL_BOARD  @GIMBAL_BOARD@
#define CHASSIS_BOARD @CHASSIS_BOARD@
```

三个板型宏中只有一个为 1，其余为 0。由 `config.cmake` 中的 `set(${BOARD_UPPER}_BOARD 1)` 决定。

### 模块开关

```c
#define MODULE_OFFLINE   @MODULE_OFFLINE@
#define MODULE_REMOTE    @MODULE_REMOTE@
#define MODULE_BMI088    @MODULE_BMI088@
...
```

1 为启用，0 为禁用。在 `config.cmake` 中由 `MODULES_XXX` 列表决定。

### 可选硬件句柄（`#cmakedefine`）

```c
#cmakedefine REMOTE_UART @REMOTE_UART@
#cmakedefine REFEREE_UART @REFEREE_UART@
#cmakedefine SUPERCAP_CAN @SUPERCAP_CAN@
#cmakedefine WT606_UART @WT606_UART@
#cmakedefine BOARDCOMM_CAN @BOARDCOMM_CAN@
```

- `#cmakedefine` → [[01_extracted/cmake/cmake-basic-syntax#模板语法]]

`#cmakedefine` 是条件宏：变量有值且非假时生成 `#define`，否则生成 `/* #undef */`。这样机器人配置中不需要的硬件句柄不会被错误引用。

### 参数宏

```c
#define REMOTE_UART_DEAD_ZONE    @REMOTE_DEAD_ZONE@
#define REMOTE_TASK_STACK_SIZE   @REMOTE_TASK_STACK_SIZE@
#define OFFLINE_WATCHDOG_ENABLE  @OFFLINE_WATCHDOG_ENABLE@
...
```

所有参数走 `@VAR@` 无条件替换，因为这些参数在 `module_config.cmake` 中都有默认值，不会出现未定义的情况。

---

## 模板替换示例

`ROBOT=sentry, BOARD=gimbal` 时生成的头文件：

```c
#define SINGLE_BOARD  0
#define GIMBAL_BOARD  1
#define CHASSIS_BOARD 0

#define MODULE_OFFLINE  1
#define MODULE_REMOTE   1
#define MODULE_BMI088   1
#define MODULE_REFEREE  0
...

#define REMOTE_UART huart3
/* #undef REFEREE_UART */
/* #undef SUPERCAP_CAN */
```

## 调用关系

```
board_common.cmake
  └─ configure_file(module_config.h.in → module_config.h)  ← 生成的头文件
```

生成的 `module_config.h` 位于 `build/<板名>/Debug/generated/module_config.h`，被各模块的 `.c` 文件通过 `-include` 强制包含（在 `target_compile_options` 中设置）。

## 对应的 CMake 变量来源

| 模板中的宏 | 变量来源 |
|-----------|---------|
| `@SINGLE_BOARD@` | `config.cmake` 中 `set(${BOARD_UPPER}_BOARD 1)` |
| `@MODULE_OFFLINE@` | `config.cmake` 中 `set(MODULE_OFFLINE 1)` |
| `@REMOTE_UART@` | `sentry/robot.cmake` 中 `set(REMOTE_UART huart3)` |
| `@REMOTE_TASK_STACK_SIZE@` | `module_config.cmake` 中 `set(REMOTE_TASK_STACK_SIZE 512)` |