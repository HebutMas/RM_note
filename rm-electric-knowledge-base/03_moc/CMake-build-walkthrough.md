# CMake 构建全流程

> 从 VS Code 按下构建按钮，到 `base.elf` 产出，CMake 到底干了什么。
> 本文是阅读入口，按顺序链接到 02 层各文件的详解。

---

## 阅读顺序

先看 [[02_code_twin/_vscode/tasks-json]]——它定义了构建按钮背后的一切。tasks.json 中的 `cmake -S` 参数决定了源码入口目录，从那里开始看本级 CMakeLists.txt，再跟着 `add_subdirectory` 和 `include` 逐层深入。

整条链路是单向的：tasks.json → CMakePresets → 工具链 → 板级 CMakeLists → 子模块。每一层只向下展开，不跳回来。

---

## 全流程概览

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
gcc-arm-none-eabi.cmake    ← 告诉 CMake 用 ARM 交叉编译器
  │  设置 C/CXX 编译器、MCU flags、优化标志、链接器标志
  ▼
CMakeLists.txt (board/dji_c)   ← 项目入口，整个构建的指挥中心
  │
  ├─ include(config.cmake)           → 配置链（选机器人、选板型、设模块开关）
  │    ├─ include(module_config.cmake)  → 默认参数
  │    └─ include(infantry3/robot.cmake) → 覆盖差异
  │
  ├─ include(generate_headers.cmake) → 把 CMake 变量翻译成 C 宏
  │    ├─ file(WRITE robot_def.h)
  │    └─ file(WRITE module_config.h)
  │
  ├─ project(base) + enable_language(C ASM)
  ├─ add_executable(base)
  │
  ├─ add_subdirectory(cmake/stm32cubemx)   → CubeMX 生成代码
  ├─ add_subdirectory(../../threadx)       → 第三方，暂不展开
  ├─ include(../../CherryUSB/...)          → 第三方，暂不展开
  ├─ add_subdirectory(../../CMSIS-DSP)     → 第三方，暂不展开
  ├─ add_subdirectory(../../utils)         → 工具库
  ├─ add_subdirectory(../../board/bsp)     → 硬件抽象层
  ├─ add_subdirectory(../../robot)         → 机器人初始化
  ├─ add_subdirectory(../../modules)       → 功能模块
  ├─ add_subdirectory(../../apps)          → 应用层
  │
  ├─ target_link_libraries(base ...所有库...)
  └─ target_link_options(base PRIVATE -flto)
       │
       ▼
  base.elf + compile_commands.json
```

---

## 跳转深度

从 tasks.json 出发，跟随 `add_subdirectory` 和 `include` 逐层深入，每一层只在 02 层有一篇笔记。缩进表示深入层级，箭头表示链接到 02 层的哪篇笔记：

```
_vscode/tasks.json
  └→ [[02_code_twin/_vscode/tasks-json]]
      │
      ├─ --preset Debug
      │   └→ board/dji_c/CMakePresets.json
      │       └→ [[02_code_twin/board/dji_c/CMakePresets-json]]
      │           │
      │           └─ toolchainFile
      │               └→ board/dji_c/cmake/gcc-arm-none-eabi.cmake
      │                   └→ [[02_code_twin/board/dji_c/cmake/gcc-arm-none-eabi-cmake]]
      │
      └─ -S board/dji_c
          └→ board/dji_c/CMakeLists.txt
              └→ [[02_code_twin/board/dji_c/CMakeLists-txt]]
                  │
                  ├─ include(config.cmake)
                  │   ├→ [[02_code_twin/apps/config-cmake]]
                  │   │   ├─ include(module_config.cmake)
                  │   │   │   └→ [[02_code_twin/modules/module_config-cmake]]
                  │   │   └─ include(infantry3/robot.cmake)
                  │   │       └→ [[02_code_twin/apps/infantry3/robot-cmake]]
                  │   │
                  │   └─ include(generate_headers.cmake)
                  │       └→ [[02_code_twin/apps/generate_headers-cmake]]
                  │
                  ├─ add_subdirectory(cmake/stm32cubemx)
                  │   └→ [[02_code_twin/board/dji_c/cmake/stm32cubemx/CMakeLists-txt]]
                  │
                  ├─ add_subdirectory(../../utils)
                  │   └→ [[02_code_twin/utils/CMakeLists-txt]]
                  │
                  ├─ add_subdirectory(../../board/bsp)
                  │   └→ [[02_code_twin/board/bsp/CMakeLists-txt]]
                  │
                  ├─ add_subdirectory(../../robot)
                  │   └→ [[02_code_twin/robot/CMakeLists-txt]]
                  │
                  ├─ add_subdirectory(../../modules)
                  │   └→ [[02_code_twin/modules/CMakeLists-txt]]
                  │
                  └─ add_subdirectory(../../apps)
                      └→ [[02_code_twin/apps/CMakeLists-txt]]
```

每一层向下展开后不跳回来。子模块的 CMakeLists.txt 只负责创建自己的 target 和设置属性，不反向操作父级。最终所有 target 在板级 CMakeLists.txt 的 `target_link_libraries` 处汇总。
