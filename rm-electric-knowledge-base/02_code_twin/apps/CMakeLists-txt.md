# CMakeLists.txt — apps 应用层

## 文件位置

`mas_embedded_threadx/apps/CMakeLists.txt`

## 作用

被 [[02_code_twin/board/dji_c/CMakeLists-txt]] 通过 `add_subdirectory(../../apps ...)` 加载。根据 `ROBOT` 和 `BOARD` 变量自动收集业务代码，编译为 OBJECT 库。

---

## 原始代码（摘取）

### 定位业务代码目录

```cmake
set(_robot_base_dir ${CMAKE_CURRENT_SOURCE_DIR}/${ROBOT})
set(_robot_board_dir ${_robot_base_dir}/${BOARD}_board)
```

`ROBOT` 和 `BOARD` 来自 [[02_code_twin/apps/config-cmake]]。以 `infantry3 single` 为例：

```
_robot_base_dir  = apps/infantry3/
_robot_board_dir = apps/infantry3/single_board/
```

### 自动收集源文件

```cmake
set(_robot_sources "")
if(EXISTS ${_robot_board_dir})
    file(GLOB_RECURSE _robot_sources ${_robot_board_dir}/*.c)
endif()
```

`file(GLOB_RECURSE)` 递归扫描 `apps/infantry3/single_board/` 下所有子目录的 `.c` 文件。往这个目录里加新文件不需要改 CMakeLists.txt。详见 [[01_extracted/cmake/cmake-basic-syntax#file(GLOB_RECURSE) - 递归扫描文件]]

### 自动收集头文件路径

```cmake
# 收集所有子目录作为头文件包含路径
# 把 apps/<robot>/ 根目录加入，方便板型间共享 <robot>_def.h 等头文件
set(_robot_includes "")
if(EXISTS ${_robot_base_dir})
    list(APPEND _robot_includes ${_robot_base_dir})
endif()
```

先把 `apps/infantry3/` 根目录加入头文件路径。这样不同板型（single_board / gimbal_board）之间可以共享 `infantry3_def.h` 这类公共头文件。

```cmake
if(EXISTS ${_robot_board_dir})
    file(GLOB_RECURSE _robot_headers ${_robot_board_dir}/*.h)
    list(APPEND _robot_includes ${_robot_board_dir})
    foreach(_h ${_robot_headers})
        get_filename_component(_dir ${_h} DIRECTORY)
        list(APPEND _robot_includes ${_dir})
    endforeach()
    list(REMOVE_DUPLICATES _robot_includes)
endif()
```

- `file(GLOB_RECURSE)` → [[01_extracted/cmake/cmake-basic-syntax#file(GLOB_RECURSE) - 递归扫描文件]]
- `foreach` → [[01_extracted/cmake/cmake-basic-syntax#foreach - 循环]]
- `list` → [[01_extracted/cmake/cmake-basic-syntax#list - 列表操作]]
- `get_filename_component` → [[01_extracted/cmake/cmake-basic-syntax#get_filename_component - 提取路径组件]]

再扫描 `single_board/` 下所有 `.h` 文件，提取每个文件所在目录加入头文件路径，最后 `list(REMOVE_DUPLICATES)` 去重。这样源码里 `#include "chassis.h"` 就能自动找到 `single_board/chassis/chassis.h` 所在的目录。

### 建库

```cmake
add_library(app OBJECT
    app_init.c
    ${_robot_sources}
)

target_compile_options(app PRIVATE -O2 -include module_config.h)

target_include_directories(app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${_robot_includes}
)

target_link_libraries(app PUBLIC stm32cubemx azrtos::threadx utils bsp CMSISDSP modules)
```

- `add_library(OBJECT)` → [[01_extracted/cmake/gcc-cmake-build#add_library(OBJECT) - .o 直接注入 elf]]
- `target_compile_options` → [[01_extracted/cmake/gcc-cmake-build#target_compile_options - 编译器选项]]
- `target_include_directories` → [[01_extracted/cmake/gcc-cmake-build#target_include_directories - 头文件搜索路径]]
- `target_link_libraries` → [[01_extracted/cmake/gcc-cmake-build#target_link_libraries - 链接哪些库]]
- `PRIVATE`/`PUBLIC` 关键字 → [[01_extracted/cmake/gcc-cmake-build#五、作用域与 PRIVATE / PUBLIC / INTERFACE]]

`app_init.c` 是固定文件（所有机器人共用），`${_robot_sources}` 是自动收集的业务代码。和 modules 一样用 `-include module_config.h` 强制注入配置宏。

链接了 `modules`——app 的业务代码调用 modules 提供的接口（电机控制、裁判系统通信等）。`PUBLIC` 传递，`base` 链接 app 后自动获得 modules 及其所有底层依赖。
