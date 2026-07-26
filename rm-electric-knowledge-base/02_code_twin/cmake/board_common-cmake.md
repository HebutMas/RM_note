# board_common.cmake — 板级公共构建逻辑

## 文件位置

`mas_embedded_threadx/cmake/board_common.cmake`

## 作用

将各板 CMakeLists.txt 中的重复逻辑提取为公共组件。板级 CMakeLists.txt 只需设置 `THREADX_ARCH` 等板级差异，然后 `include(board_common.cmake)` 即可。

---

## 原始代码

### 仓库根目录计算

```cmake
get_filename_component(MAS_ROOT ${CMAKE_CURRENT_LIST_DIR}/.. ABSOLUTE)
```

`CMAKE_CURRENT_LIST_DIR` 指向 `cmake/` 目录，`/..` 回到仓库根目录，`ABSOLUTE` 确保是绝对路径。因为 `board_common.cmake` 是被 `include` 进来的，`CMAKE_CURRENT_SOURCE_DIR` 仍然是板目录，不能用相对路径。
 - [[cmake-basic-syntax#get_filename_component - 提取路径组件]]
### 基础设置

```cmake
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
```

### 配置系统加载

```cmake
include(${MAS_ROOT}/apps/config.cmake)

set(_generated_dir ${CMAKE_CURRENT_BINARY_DIR}/generated)
configure_file(${MAS_ROOT}/apps/robot_def.h.in      ${_generated_dir}/robot_def.h      @ONLY)
configure_file(${MAS_ROOT}/apps/module_config.h.in  ${_generated_dir}/module_config.h  @ONLY)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
```
有关configure_file函数和.h.in的知识详见[[cmake-basic-syntax#configure_file - 模板替换生成文件]]
### 创建目标

```cmake
message("Build type: " ${CMAKE_BUILD_TYPE})
enable_language(C ASM)
add_executable(${CMAKE_PROJECT_NAME})
```

### 子模块集成

```cmake
add_subdirectory(cmake/stm32cubemx)
add_subdirectory(${MAS_ROOT}/threadx threadx)
add_subdirectory(${MAS_ROOT}/utils     ${CMAKE_CURRENT_BINARY_DIR}/utils)
add_subdirectory(${MAS_ROOT}/board/bsp ${CMAKE_CURRENT_BINARY_DIR}/bsp)
add_subdirectory(${MAS_ROOT}/robot     ${CMAKE_CURRENT_BINARY_DIR}/robot)
add_subdirectory(${MAS_ROOT}/modules   ${CMAKE_CURRENT_BINARY_DIR}/modules)
add_subdirectory(${MAS_ROOT}/apps      ${CMAKE_CURRENT_BINARY_DIR}/apps)
```

| 子模块                 | 02 层笔记                                                        |
| ------------------- | ------------------------------------------------------------- |
| `cmake/stm32cubemx` | [[02_code_twin/board/dji_c/cmake/stm32cubemx/CMakeLists-txt]] |
| `threadx`           | 第三方库，暂不展开                                                     |
| `utils`             | [[02_code_twin/utils/CMakeLists-txt]]                         |
| `board/bsp`         | [[02_code_twin/board/bsp/CMakeLists-txt]]                     |
| `robot`             | [[02_code_twin/robot/CMakeLists-txt]]                         |
| `modules`           | [[02_code_twin/modules/CMakeLists-txt]]                       |
| `apps`              | [[02_code_twin/apps/CMakeLists-txt]]                          |

### CherryUSB（统一编译，链接器裁剪）

```cmake
set(CONFIG_CHERRYUSB_DEVICE ON CACHE BOOL "Enable CherryUSB device stack" FORCE)
set(CONFIG_CHERRYUSB_DEVICE_CDC_ACM ON CACHE BOOL "Enable CDC ACM class" FORCE)
set(CONFIG_CHERRYUSB_DEVICE_DWC2_ST ON CACHE BOOL "Use DWC2 OTG with STM32 glue" FORCE)
set(CONFIG_CHERRYUSB_OSAL "threadx" CACHE STRING "Use ThreadX OS abstraction layer" FORCE)

include(${MAS_ROOT}/CherryUSB/cherryusb.cmake)
list(REMOVE_DUPLICATES cherryusb_srcs)
list(REMOVE_DUPLICATES cherryusb_incs)

add_library(cherryusb STATIC ${cherryusb_srcs})
target_include_directories(cherryusb PUBLIC
    ${MAS_ROOT}/board/bsp/USB
    ${cherryusb_incs}
)
target_compile_options(cherryusb PRIVATE -O3 -ffast-math -fno-math-errno)  
target_link_libraries(cherryusb PUBLIC stm32cubemx azrtos::threadx utils)
```
注意此处进行了cherryusb的编译优化→ [[01_extracted/cmake/gcc-cmake-build#target_compile_options - 编译器选项]]
### CMSIS-DSP（统一编译，链接器裁剪）

```cmake
set(LOOPUNROLL ON CACHE BOOL "Loop unrolling for max performance" FORCE)
set(DISABLEFLOAT16 ON CACHE BOOL "Disable float16 kernels (not needed on M4/M7)" FORCE)
add_subdirectory(${MAS_ROOT}/CMSIS-DSP ${CMAKE_CURRENT_BINARY_DIR}/cmsisdsp)
target_compile_options(CMSISDSP PRIVATE -O3 -ffast-math -fno-math-errno -flto)
target_include_directories(CMSISDSP PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include)
```

### 最终链接

```cmake
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${_generated_dir})

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

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

---

## 关键设计

### CherryUSB 和 CMSIS-DSP 为什么放公共层

两个库在所有板子中**统一编译**，不区分板型。`-flto` + `--gc-sections` 在链接阶段自动丢弃未使用的代码，不会额外占用 flash。

### 板级差异的传递方式

| 差异 | 传递方式 |
|------|---------|
| MCU 架构 | 板级 `cmake/gcc-arm-none-eabi.cmake` 中 `set(MCU_TARGET_FLAGS)` |
| 链接脚本 | 板级 `cmake/gcc-arm-none-eabi.cmake` 中 `set(MCU_LINKER_SCRIPT)` |
| ThreadX 架构 | 板级 CMakeLists.txt 中 `set(THREADX_ARCH cortex_m4)` |
| 特殊库 | 板级 CMakeLists.txt 中在 `include(board_common.cmake)` 前后追加 |

### CMSIS 头文件路径

```cmake
target_include_directories(CMSISDSP PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include)
```

`CMAKE_CURRENT_SOURCE_DIR` 指向板目录（如 `board/dji_c/`），所以 `Drivers/CMSIS/Include` 解析为 `board/dji_c/Drivers/CMSIS/Include`。各板需要保证该路径下有正确的 CMSIS 头文件。

---

## 依赖链

```
                   ┌─ app
                   │
                   ├─ modules
                   │
                   ├─ cherryusb ──┬─ stm32cubemx
                   │              ├─ azrtos::threadx
                   │              └─ utils
    base ──link───┼─ CMSISDSP ──┬─ Drivers/CMSIS/Include
                   │             │
                   ├─ robot
                   │
                   ├─ bsp
                   │
                   ├─ utils
                   │
                   ├─ azrtos::threadx
                   │
                   └─ stm32cubemx
```