# gcc-arm-none-eabi-common.cmake — 公共 ARM 工具链逻辑

## 文件位置

`mas_embedded_threadx/cmake/gcc-arm-none-eabi-common.cmake`

## 作用

提取各板 `cmake/gcc-arm-none-eabi.cmake` 中的公共逻辑，避免重复。各板工具链文件只需定义 `MCU_TARGET_FLAGS` 和 `MCU_LINKER_SCRIPT` 两个变量，然后 `include` 本文件。

> 本文件必须在 `project()` 之前加载（由板级工具链文件委托）。一旦 `project()` 执行完毕，编译器路径就不可更改。

---

## 原始代码（摘取）

### 入口校验

```cmake
if(NOT DEFINED MCU_TARGET_FLAGS OR NOT DEFINED MCU_LINKER_SCRIPT)
    message(FATAL_ERROR "MCU_TARGET_FLAGS and MCU_LINKER_SCRIPT must be set before including")
endif()
```

- `message(FATAL_ERROR)` → [[01_extracted/cmake/cmake-basic-syntax#message - 打印信息]]

确保板级工具链文件在 `include` 本文件之前已经设好了两个变量。如果忘记设，CMake 配置阶段直接报错，不需要等到编译。

### 编译器设置

```cmake
set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(TOOLCHAIN_PREFIX                arm-none-eabi-)
set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CMAKE_TRY_COMPILE_TARGET_TYPE   STATIC_LIBRARY)
```

| 设置 | 含义 |
|------|------|
| `CMAKE_SYSTEM_NAME = Generic` | 告诉 CMake 这是嵌入式交叉编译，不尝试运行产物 |
| `CMAKE_TRY_COMPILE_TARGET_TYPE = STATIC_LIBRARY` | 跳过链接测试，嵌入式目标没有标准库和链接器脚本 |

### MCU 架构标志

```cmake
set(TARGET_FLAGS "${MCU_TARGET_FLAGS} ")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
```

`MCU_TARGET_FLAGS` 由板级工具链文件提供（如 `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard`）。

### 编译优化标志

```cmake
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
```

| 标志 | 含义 |
|------|------|
| `-ffunction-sections` | 每个函数放入独立段，配合 `--gc-sections` 移除未使用函数 |
| `-fdata-sections` | 每个数据变量放入独立段 |
| `-fstack-usage` | 生成栈使用报告（`.su` 文件），用于分析线程栈大小 |

### 链接器标志

```cmake
set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${MCU_LINKER_SCRIPT}\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
```

| 标志 | 含义 |
|------|------|
| `-T "${MCU_LINKER_SCRIPT}"` | 指定链接器脚本（由板级提供） |
| `--specs=nano.specs` | 使用 newlib-nano 精简 C 库 |
| `-Wl,--gc-sections` | 链接时移除未引用的段，配合 `-ffunction-sections` 使用 |
| `-Wl,--print-memory-usage` | 链接后打印 Flash/RAM 使用量 |
| `TOOLCHAIN_LINK_LIBRARIES = m` | 链接数学库 `libm`，在 stm32cubemx CMakeLists.txt 中被引用 |

### C++ 标志

```cmake
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")
```

---

## 调用方式

```cmake
# board/<板名>/cmake/gcc-arm-none-eabi.cmake
set(MCU_TARGET_FLAGS  "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(MCU_LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../STM32F407XX_FLASH.ld")
include(${CMAKE_CURRENT_LIST_DIR}/../../../cmake/gcc-arm-none-eabi-common.cmake)
```

板级文件只提供 `MCU_TARGET_FLAGS` 和 `MCU_LINKER_SCRIPT`，其余全部由本文件统一处理。各板差异详见 [[02_code_twin/board/dji_c/cmake/gcc-arm-none-eabi-cmake#各板差异对比]]。