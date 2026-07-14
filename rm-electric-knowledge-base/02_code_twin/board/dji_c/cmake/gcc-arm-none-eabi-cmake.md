# gcc-arm-none-eabi.cmake — ARM 交叉编译工具链配置

## 文件位置

`mas_embedded_threadx/board/dji_c/cmake/gcc-arm-none-eabi.cmake`

## 作用

被 [[02_code_twin/board/dji_c/CMakePresets-json]] 的 `toolchainFile` 字段引用。在 CMake 配置阶段**最先被加载**（`project()` 之前），负责告诉 CMake：用哪个编译器、目标芯片是什么架构、编译/链接标志是什么。

> 工具链文件必须在 `project()` 之前加载。一旦 `project()` 执行完毕，编译器路径就不可更改。之后 [[02_code_twin/board/dji_c/CMakeLists-txt]] 才开始定义项目结构。

---

## 原始代码（摘取）

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

| 设置                                               | 含义                                                    |
| ------------------------------------------------ | ----------------------------------------------------- |
| `CMAKE_SYSTEM_NAME = Generic`                    | 告诉 CMake 这是嵌入式交叉编译（不是 Linux/Windows），不尝试运行编译产物        |
| `CMAKE_TRY_COMPILE_TARGET_TYPE = STATIC_LIBRARY` | 跳过链接测试。嵌入式目标没有标准库和链接器脚本，CMake 默认的"编译+链接一个测试程序"会失败     |
| `arm-none-eabi-` 前缀                              | ARM 裸机 GCC 的标准前缀：`arm`=架构、`none`=无操作系统、`eabi`=嵌入式 ABI |

### MCU 架构标志

```cmake
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
```

| 标志 | 含义 |
|------|------|
| `-mcpu=cortex-m4` | 目标核心：Cortex-M4（C板 STM32F407IG） |
| `-mfpu=fpv4-sp-d16` | FPU 类型：单精度浮点单元 |
| `-mfloat-abi=hard` | 硬件浮点 ABI：浮点参数直接通过 FPU 寄存器传递，不走软件模拟 |

> 对比 damiao_h7 板：`-mcpu=cortex-m7 -mfpu=fpv5-d16`（双精度 FPU）。两块板的工具链差异仅在此处。

### 编译优化标志

```cmake
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
```

| 标志                    | 含义                                      |
| --------------------- | --------------------------------------- |
| `-Wall`               | 开启常用警告                                  |
| `-ffunction-sections` | 每个函数放入独立段，配合链接器 `--gc-sections` 移除未使用函数 |
| `-fdata-sections`     | 每个数据变量放入独立段                             |
| `-fstack-usage`       | 生成栈使用报告（`.su` 文件），用于分析线程栈大小是否够用         |
| `-O0 -g3` (Debug)     | 不优化、最大调试信息                              |
| `-Os -g0` (Release)   | 体积优化、去掉调试信息                             |

> CMake 根据 `CMAKE_BUILD_TYPE`（由 [[02_code_twin/board/dji_c/CMakePresets-json]] 的预设设置）自动选择 `CMAKE_C_FLAGS_DEBUG` 或 `CMAKE_C_FLAGS_RELEASE`，拼接到 `CMAKE_C_FLAGS` 后面。

### 链接器标志

```cmake
set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F407XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
```

| 标志                             | 含义                                 |
| ------------------------------ | ---------------------------------- |
| `-T STM32F407XX_FLASH.ld`      | 指定链接器脚本，定义 Flash/RAM 地址和段布局        |
| `--specs=nano.specs`           | 使用 newlib-nano 精简 C 库（更小的代码体积）     |
| `-Wl,-Map=base.map`            | 生成内存映射文件（`base.map`），可查看每个符号的位置和大小 |
| `-Wl,--gc-sections`            | 链接时移除未引用的段，减小固件体积                  |
| `-Wl,--print-memory-usage`     | 链接后打印 Flash/RAM 使用量                |
| `TOOLCHAIN_LINK_LIBRARIES = m` | 链接数学库 `libm`（`sin/cos/sqrt` 等）     |

> `TOOLCHAIN_LINK_LIBRARIES` 会在 [[02_code_twin/board/dji_c/cmake/stm32cubemx/CMakeLists-txt]] 中被引用。

### C++ 标志

```cmake
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")
```

| 标志 | 含义 |
|------|------|
| `-fno-rtti` | 禁用 RTTI（`dynamic_cast`、`typeid`），减小体积 |
| `-fno-exceptions` | 禁用异常处理（`try/catch`），嵌入式环境通常不需要 |
| `-fno-threadsafe-statics` | 禁用局部静态变量的线程安全初始化（由 RTOS 自行管理） |

---

## 与 damiao_h7 的差异

| 参数 | dji_c (STM32F407) | damiao_h7 (STM32H723) |
|------|-------------------|----------------------|
| CPU | Cortex-M4 | Cortex-M7 |
| FPU | `fpv4-sp-d16`（单精度） | `fpv5-d16`（双精度） |
| 频率 | 168 MHz | 480 MHz |
| 链接器脚本 | `STM32F407XX_FLASH.ld` | `STM32H723XG_FLASH.ld` |
