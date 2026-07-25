# robot_def.h.in — 机器人类型宏模板

## 文件位置

`mas_embedded_threadx/apps/robot_def.h.in`

## 作用

`configure_file` 的输入模板。`board_common.cmake` 执行 `configure_file(robot_def.h.in robot_def.h @ONLY)` 时，将 `@ROBOT_UPPER@` 替换为 CMake 变量 `ROBOT_UPPER` 的值，生成 `robot_def.h`。

---

## 原始代码

```c
#ifndef _ROBOT_DEF_H_
#define _ROBOT_DEF_H_

/* 机器人类型定义 */
#define ROBOT_HERO      1
#define ROBOT_ENGINEER  2
#define ROBOT_INFANTRY3 3
#define ROBOT_INFANTRY4 4
#define ROBOT_INFANTRY5 5
#define ROBOT_DRONE     6
#define ROBOT_SENTRY    7
#define ROBOT_DARTS     8

/* 当前编译的机器人 — 由 CMake 根据 ROBOT 选项自动生成 */
#define CURRENT_ROBOT   ROBOT_@ROBOT_UPPER@

#define ROBOT_TYPE      CURRENT_ROBOT

#endif // _ROBOT_DEF_H_
```

## 模板替换示例

`ROBOT=infantry3` 时，`ROBOT_UPPER=INFANTRY3`，替换后：

```c
#define CURRENT_ROBOT   ROBOT_INFANTRY3     // = 3
```

`ROBOT=sentry` 时：

```c
#define CURRENT_ROBOT   ROBOT_SENTRY        // = 7
```

## 调用关系

```
board_common.cmake
  └─ configure_file(robot_def.h.in → robot_def.h)  ← 生成的头文件
```

生成的 `robot_def.h` 位于 `build/<板名>/Debug/generated/robot_def.h`，供应用层代码判断当前是哪个机器人。

> 本文件替代了旧版 `generate_headers.cmake` 中的 `file(WRITE robot_def.h)`。详见 [[01_extracted/cmake/cmake-basic-syntax#configure_file - 模板替换生成文件]]。