# CMakeLists.txt — 板级构建脚本（dji_c）

## 文件位置

`mas_embedded_threadx/board/dji_c/CMakeLists.txt`

## 作用

整个项目的构建入口。[[02_code_twin/_vscode/tasks-json]] 中 `cmake -S ${workspaceFolder}/board/dji_c` 指向的就是这个文件。它定义了项目名称、加载配置系统、集成所有子模块、最终链接出 `base.elf`。

> 本项目没有顶层 CMakeLists.txt，每个板（`dji_c` / `damiao_h7`）各自独立作为 CMake 入口。这样设计是因为不同板的 MCU、工具链参数、链接器脚本完全不同。加载本文件前，[[02_code_twin/board/dji_c/CMakePresets-json]] 已先加载 [[02_code_twin/board/dji_c/cmake/gcc-arm-none-eabi-cmake]] 完成工具链配置。

---

## 原始代码（摘取）

### 项目基础设置

```cmake
cmake_minimum_required(VERSION 3.22)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME base)
set(THREADX_ARCH cortex_m4)
```

-`CMAKE_BUILD_TYPE`→[[02_code_twin/board/dji_c/CMakePresets-json#Debug / Release 预设]]
- `set` → [[01_extracted/cmake-basic-syntax#set - 设置变量]]
- `if` → [[01_extracted/cmake-basic-syntax#if - 条件判断]]

| 设置                        | 含义                             |
| ------------------------- | ------------------------------ |
| `CMAKE_C_STANDARD 11`     | 使用 C11 标准（CubeMX 生成的代码要求 C11+） |
| `CMAKE_PROJECT_NAME base` | 最终产物为 `base.elf`、`base.map`    |
| `THREADX_ARCH cortex_m4`  | 告诉 ThreadX 使用 Cortex-M4 的端口代码  |

### 配置系统加载

```cmake
include(${CMAKE_SOURCE_DIR}/../../apps/config.cmake)

set(_generated_dir ${CMAKE_CURRENT_BINARY_DIR}/generated)
include(${CMAKE_SOURCE_DIR}/../../apps/generate_headers.cmake)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
```

- `include` → [[01_extracted/cmake-basic-syntax#include - 粘贴到当前位置]]

| 语句                                                          | 作用                                                                                                                                                                |
| ----------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `include(config.cmake)`                                     | 加载 [[02_code_twin/apps/config-cmake]]，设置 `ROBOT`/`BOARD` 和 `MODULE_XXX` 条件编译开关                                                                                    |
| `set(_generated_dir ${CMAKE_CURRENT_BINARY_DIR}/generated)` | 定义 `_generated_dir`，指向 [[02_code_twin/_vscode/tasks-json#阶段一：配置（Configure）\|-B 指定的构建目录]] 下的 `generated/` 子目录，下一行 [[02_code_twin/apps/generate_headers-cmake]] 会用到 |
| `include(generate_headers.cmake)`                           | 加载 [[02_code_twin/apps/generate_headers-cmake]]，把 CMake 变量翻译成 C 宏，生成 `robot_def.h` 和 `module_config.h`                                                            |
| `CMAKE_EXPORT_COMPILE_COMMANDS TRUE`                        | 生成 `compile_commands.json`，供 [[02_code_twin/_vscode/settings-json]] 中的 clangd 做代码跳转                                                                               |

### 子模块集成

```cmake
project(${CMAKE_PROJECT_NAME})
enable_language(C ASM)
add_executable(${CMAKE_PROJECT_NAME})

add_subdirectory(cmake/stm32cubemx)
add_subdirectory(../../threadx threadx)
add_subdirectory(../../utils ...)
add_subdirectory(../../board/bsp ...)
add_subdirectory(../../robot ...)
add_subdirectory(../../modules ...)
add_subdirectory(../../apps ...)
add_subdirectory(../../CMSIS-DSP ...)
```

- `project` → [[01_extracted/gcc-cmake-build#三、project - 声明项目名称]]
- `add_executable` → [[01_extracted/gcc-cmake-build#add_executable - 创建可执行文件目标]]
- `add_subdirectory` → [[01_extracted/gcc-cmake-build#add_subdirectory - 进入子目录执行]]

每个 `add_subdirectory` 的两个参数：第一个是**源码路径**（相对 `CMAKE_SOURCE_DIR`），第二个是**构建目录名**（相对 `CMAKE_BINARY_DIR`）。`../../` 向上穿越两级从 `board/dji_c/` 回到项目根目录。

| `add_subdirectory`  | 指向的 CMakeLists.txt                                            |
| ------------------- | ------------------------------------------------------------- |
| `cmake/stm32cubemx` | [[02_code_twin/board/dji_c/cmake/stm32cubemx/CMakeLists-txt]] |
| `../../threadx`     | 第三方库，暂不展开                                                     |
| `../../utils`       | [[02_code_twin/utils/CMakeLists-txt]]                         |
| `../../board/bsp`   | [[02_code_twin/board/bsp/CMakeLists-txt]]                     |
| `../../robot`       | [[02_code_twin/robot/CMakeLists-txt]]                         |
| `../../modules`     | [[02_code_twin/modules/CMakeLists-txt]]                       |
| `../../apps`        | [[02_code_twin/apps/CMakeLists-txt]]                          |
| `../../CMSIS-DSP`   | 第三方库，暂不展开                                                     |
| `../../CherryUSB`   | 第三方库，暂不展开                                                     |


### 最终链接

```cmake
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    azrtos::threadx
    utils
    bsp
    robot
    CMSISDSP
    cherryusb
    modules
    app
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -flto)
```

- `target_link_libraries` → [[01_extracted/gcc-cmake-build#target_link_libraries - 链接哪些库]]
- `target_link_options` → [[01_extracted/gcc-cmake-build#target_link_options - 链接器选项]]

把所有库链接到 `base.elf`。`-flto` 启用链接时优化（LTO），让编译器跨模块边界做内联和死代码消除。

### 依赖链

```
app ─PUBLIC→ modules ─PUBLIC→ bsp ─PUBLIC→ utils ─PUBLIC→ stm32cubemx
 │              │                │              │                  │
 │              │                │              │                  └─ HAL 头文件 + 宏定义
 │              │                │              └─ azrtos::threadx
 │              │                └─ cherryusb
 │              └─ CMSISDSP
 └─ robot
```

因为每层都用 `PUBLIC` 向上传递，`base.elf` 最终能访问所有底层库的头文件和符号。

---

## 构建产物

配置+构建完成后，`build/dji_c/Debug/` 下的产物：

```
build/dji_c/Debug/
├── build.ninja              ← Ninja 构建脚本（配置阶段生成）
├── CMakeCache.txt           ← CMake 缓存（配置阶段生成）
├── compile_commands.json    ← 编译数据库（供 clangd）
├── base.elf                 ← 最终固件
├── base.map                 ← 内存映射
└── generated/
    ├── robot_def.h          ← 自动生成的机器人类型宏
    └── module_config.h      ← 自动生成的模块开关宏
```

> `base.elf` 和 `compile_commands.json` 会被 [[02_code_twin/_vscode/tasks-json]] 阶段五复制到 `build/` 根目录。`compile_commands.json` 供 [[02_code_twin/_vscode/settings-json]] 中的 clangd 读取，`base.elf`（复制后为 `dji_c.elf`）供外部烧录工具加载。
