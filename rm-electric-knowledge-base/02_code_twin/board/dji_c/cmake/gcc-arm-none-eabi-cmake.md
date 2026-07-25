# gcc-arm-none-eabi.cmake — 板级 MCU 差异参数

## 文件位置

`mas_embedded_threadx/board/dji_c/cmake/gcc-arm-none-eabi.cmake`

## 作用

被 [[02_code_twin/board/dji_c/CMakePresets-json]] 的 `toolchainFile` 字段引用。在 CMake 配置阶段**最先被加载**（`project()` 之前）。

这个文件只负责定义**板级差异**（MCU 架构标志和链接脚本路径），**公共工具链逻辑**委托给 `cmake/gcc-arm-none-eabi-common.cmake`。

> 工具链文件必须在 `project()` 之前加载。一旦 `project()` 执行完毕，编译器路径就不可更改。

---

## 原始代码

```cmake
# dji_c (STM32F407 / Cortex-M4) MCU 差异参数
set(MCU_TARGET_FLAGS  "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(MCU_LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../STM32F407XX_FLASH.ld")
# 公共工具链逻辑
include(${CMAKE_CURRENT_LIST_DIR}/../../../cmake/gcc-arm-none-eabi-common.cmake)
```

## 职责

| 设置 | 含义 |
|------|------|
| `MCU_TARGET_FLAGS` | 板级差异：目标芯片的 CPU 架构、FPU 类型、浮点 ABI |
| `MCU_LINKER_SCRIPT` | 板级差异：链接器脚本的绝对路径（`CMAKE_CURRENT_LIST_DIR` 指向当前目录） |
| `include(gcc-arm-none-eabi-common.cmake)` | 加载公共工具链逻辑（编译器路径、优化标志、链接器标志等） |

## 各板差异对比

| 板         | 芯片                      | MCU_TARGET_FLAGS                                     | 链接脚本                   |
| --------- | ----------------------- | ---------------------------------------------------- | ---------------------- |
| dji_c     | STM32F407 (Cortex-M4)   | `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard` | `STM32F407XX_FLASH.ld` |
| damiao_h7 | STM32H723 (Cortex-M7)   | `-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard`    | `STM32H723XG_FLASH.ld` |
| f103_c8   | STM32F103C8 (Cortex-M3) | `-mcpu=cortex-m3`                                    | `STM32F103xx_FLASH.ld` |
| f105_rc   | STM32F105RC (Cortex-M3) | `-mcpu=cortex-m3`                                    | `STM32F105xx_FLASH.ld` |

> 公共工具链逻辑（编译器路径、优化标志、链接器公共标志）详见 [[02_code_twin/cmake/gcc-arm-none-eabi-common-cmake]]。