# app_init.c — 应用层入口

`apps/app_init.c`

## 一句话

`APP_Init()` 是初始化链的最后一站，调用 `robot_control_init()` 启动具体兵种的控制逻辑。

## 源码

```c
void APP_Init(void)
{
    robot_control_init();
    LOG_I("APP init finished");
}
```

## 跳转链路

```
Robot_Init()          robot/robot_init.c
  └─ APP_Init()       apps/app_init.c        ← 你在这里
       └─ robot_control_init()
            │  具体调用哪个 robot_control_init() 取决于 config.cmake 中的 ROBOT/BOARD
            │  编译时通过 CMake 收集对应目录的 .c 文件决定
            │
            └─ 哨兵云台板: apps/sentry/gimbal_board/robot_control.c
                  → [[02_code_twin/apps/sentry/gimbal_board/robot_control]]
```

`app_init.c` 本身不包含任何业务逻辑，它只是一个转发点。`robot_control_init()` 的实现在各兵种各板型的目录下，由 [[02_code_twin/apps/CMakeLists-txt|CMakeLists.txt]] 根据 `ROBOT` 和 `BOARD` 变量自动收集对应的 `.c` 文件编译进来。

| config.cmake 配置 | 编译的 robot_control.c | 02 层链接 |
|---|---|---|
| `ROBOT=sentry BOARD=gimbal` | `apps/sentry/gimbal_board/robot_control.c` | [[02_code_twin/apps/sentry/gimbal_board/robot_control]] |
