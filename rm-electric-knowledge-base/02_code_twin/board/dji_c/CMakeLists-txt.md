# CMakeLists.txt — 板级构建脚本（dji_c）

## 文件位置

`mas_embedded_threadx/board/dji_c/CMakeLists.txt`
	
## 作用

整个项目的构建入口。[[02_code_twin/_vscode/tasks-json]] 中Build dji_c所执行的命令其中的部分代码 `cmake -S ${workspaceFolder}/board/dji_c` 指向的就是这个文件。

板级 CMakeLists.txt 只保留**板级差异**（`THREADX_ARCH`、板专用库），**公共构建逻辑**委托给 `cmake/board_common.cmake`。

**注意** - 该文件由cubemx在初次配置项目时会生成一次,但之后的generate code并不会重复覆盖
    - 我们所做的事情,在对这个冗余的cmakelist,合并同类项,提炼出公共配置逻辑,简化代码 


> 每个板各自独立作为 CMake 入口。因为不同板的 MCU、工具链参数、链接器脚本完全不同。加载本文件前，[[02_code_twin/board/dji_c/CMakePresets-json]] 已先加载 [[02_code_twin/board/dji_c/cmake/gcc-arm-none-eabi-cmake]] 完成工具链配置。

---

## 原始代码

```cmake
cmake_minimum_required(VERSION 3.22)
project(base)

# MCU flags / 链接脚本见本目录 cmake/gcc-arm-none-eabi.cmake

# 本板差异
set(THREADX_ARCH cortex_m4)

# 公共构建逻辑
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/board_common.cmake)

# 板级专属 sources / includes / defines / libraries 在此追加
# target_sources(${CMAKE_PROJECT_NAME} PRIVATE ...)
```

- `project` → [[01_extracted/cmake/gcc-cmake-build#三、project - 声明项目名称]]
- `set` → [[01_extracted/cmake/cmake-basic-syntax#set - 设置变量]]
- `include` → [[01_extracted/cmake/cmake-basic-syntax#include - 粘贴到当前位置]]

## 对应讲解

| 行                                      | 含义                                                           |
| -------------------------------------- | ------------------------------------------------------------ |
| `cmake_minimum_required(VERSION 3.22)` | 只写版本号，其他配置委托给 board_common.cmake                             |
| `project(base)`                        | 声明项目名 base，最终产物 `base.elf`                                   |
| `set(THREADX_ARCH cortex_m4)`          | 板级差异：Cortex-M4 架构（f103_c8 是 cortex_m3，damiao_h7 是 cortex_m7） |
| `include(board_common.cmake)`          | 委托公共构建逻辑：加载配置、生成头文件、添加子模块、链接库                                |

公共构建逻辑的详细展开见 [[02_code_twin/cmake/board_common-cmake]]。