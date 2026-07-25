# CMake 构建全流程

> 从 VS Code 按下构建按钮，到 `base.elf` 产出，CMake 到底干了什么。
> 本文是阅读入口，按顺序链接到 02 层各文件的详解。

---

## 阅读顺序

先看 [[02_code_twin/_vscode/tasks-json]]——它定义了构建按钮背后的一切。tasks.json 中的 `cmake -S` 参数决定了源码入口目录，

以c板为例 [[02_code_twin/board/dji_c/CMakeLists-txt]] 

从那里开始看板级 CMakeLists.txt，再跟着 `include` 和 `add_subdirectory` 逐层深入。

整条链路是单向的：tasks.json → CMakePresets → 工具链 → 板级 CMakeLists → `board_common.cmake` → 子模块。

---
## 全流程概览

建议随着笔记看完上两部分回头再来看,就清晰很多了

```
tasks.json
  │  cmake -S board/dji_c -B build/dji_c/Debug --preset Debug -G Ninja
  │  cmake --build build/dji_c/Debug --config Debug -j N
  │  Copy-Item compile_commands.json, base.elf
  ▼
CMakePresets.json          ← 打包工具链路径、构建类型、生成器
  │  toolchainFile → cmake/gcc-arm-none-eabi.cmake
  │  CMAKE_BUILD_TYPE = Debug
  ▼
gcc-arm-none-eabi.cmake    ← 板级工具链文件，只设 MCU 差异参数
  │  set(MCU_TARGET_FLAGS)   ← 板级差异：-mcpu / -mfpu / -mfloat-abi
  │  set(MCU_LINKER_SCRIPT)  ← 板级差异：链接脚本路径
  └→ include(../../cmake/gcc-arm-none-eabi-common.cmake)  ← 公共工具链逻辑
  ▼
CMakeLists.txt (board/dji_c)   ← 项目入口，只保留板级差异
  │  project(base)
  │  set(THREADX_ARCH cortex_m4)
  └→ include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/board_common.cmake)
      │
      ├─ 加载配置链
      │   ├─ include(config.cmake)           → 选机器人、选板型、设模块开关
      │   │   ├─ include(module_config.cmake)  → 默认参数
      │   │   └─ include(sentry/robot.cmake) → 覆盖差异
      │   │
      │   └─ configure_file 生成 C 头文件
      │       ├─ configure_file(robot_def.h.in  → robot_def.h)     ← 模板生成
      │       └─ configure_file(module_config.h.in → module_config.h) ← 模板生成
      │
      ├─ 添加子模块
      │   ├─ add_subdirectory(cmake/stm32cubemx)   → CubeMX 生成代码
      │   ├─ add_subdirectory(../../threadx)       → ThreadX RTOS
      │   ├─ add_subdirectory(../../utils)         → 工具库
      │   ├─ add_subdirectory(../../board/bsp)     → 硬件抽象层
      │   ├─ add_subdirectory(../../robot)         → 机器人初始化
      │   ├─ add_subdirectory(../../modules)       → 功能模块
      │   └─ add_subdirectory(../../apps)          → 应用层
      │
      ├─ 链接全部
      │   └─ target_link_libraries(base ...所有库...)
      │
      └─ 链接选项
          └─ target_link_options(base PRIVATE -flto)
               │
               ▼
          base.elf + compile_commands.json
```

---

