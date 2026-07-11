# CMake 使用教程 —— 以 mas_embedded_threadx 项目为例

## 目录

1. [CMake 基础概念](#1-cmake-基础概念)
2. [项目构建总览](#2-项目构建总览)
3. [工具链配置](#3-工具链配置)
4. [配置系统：选择机器人与板型](#4-配置系统选择机器人与板型)
5. [头文件生成：从 CMake 变量到 C 宏](#5-头文件生成从-cmake-变量到-c-宏)
6. [目标类型：OBJECT / STATIC / INTERFACE](#6-目标类型object--static--interface)
7. [条件编译：MODULE_XXX 机制](#7-条件编译module_xxx-机制)
8. [属性传递：PUBLIC / PRIVATE / INTERFACE](#8-属性传递public--private--interface)
9. [双板型架构：damiao_h7 与 dji_c](#9-双板型架构damiao_h7-与-dji_c)
10. [STM32CubeMX 集成](#10-stm32cubemx-集成)
11. [第三方库集成：ThreadX / CherryUSB / CMSIS-DSP](#11-第三方库集成threadx--cherryusb--cmsis-dsp)
12. [链接器脚本与内存布局](#12-链接器脚本与内存布局)
13. [CMakePresets：预设构建配置](#13-cmakepresets预设构建配置)
14. [实用技巧总结](#14-实用技巧总结)
15. [CMake 嵌套关系深度解析](#15-cmake-嵌套关系深度解析)
16. [可优化方向](#16-可优化方向)

---

## 1. CMake 基础概念

### 1.1 CMake 是什么

CMake 是一个**跨平台的构建系统生成器**。它不直接编译代码，而是根据 `CMakeLists.txt` 中的描述，生成特定构建工具（如 Ninja、Make、Visual Studio）所需的工程文件。

### 1.2 基本工作流

```bash
# 1. 配置阶段 —— 读取 CMakeLists.txt，生成构建文件
cmake -S <源码目录> -B <构建目录> -G <生成器> [选项...]

# 2. 构建阶段 —— 调用实际编译工具链
cmake --build <构建目录> --config <Debug|Release> -j <并行任务数>
```

本项目使用的命令：
```bash
# 配置 + 构建 damiao_h7 开发板
cmake -S board/damiao_h7 -B build/damiao_h7/Debug --preset Debug -G Ninja
cmake --build build/damiao_h7/Debug --config Debug -j $(nproc)
```

### 1.3 核心语法速查

```cmake
# 变量
set(MY_VAR "hello")              # 设置普通变量
set(MY_VAR "val" CACHE STRING "") # 设置缓存变量（跨配置保留）

# 条件
if(MY_VAR)
    # ...
elseif(OTHER_VAR)
    # ...
else()
    # ...
endif()

# 循环
foreach(_item IN ITEMS a b c)
    message(STATUS "${_item}")
endforeach()

# 列表操作
list(APPEND my_list "new_item")
list(REMOVE_DUPLICATES my_list)

# 字符串操作
string(TOUPPER ${ROBOT} ROBOT_UPPER)

# 函数
function(my_func arg1)
    # ...
endfunction()

# 宏（类似函数，但直接文本替换）
macro(my_macro arg1)
    # ...
endmacro()
```

---

## 2. 项目构建总览

### 2.1 构建入口

本项目的 CMake 构建入口**不是**顶层目录，而是各开发板目录：

```
board/damiao_h7/CMakeLists.txt   ← Cortex-M7 构建入口
board/dji_c/CMakeLists.txt       ← Cortex-M4 构建入口
```

这是因为：不同开发板有不同的 MCU、工具链参数、链接器脚本和外设配置。将每个开发板作为独立的 CMake 项目（`project()` 调用）可以干净地隔离差异。

### 2.2 构建流程（以 damiao_h7 为例）

```
board/damiao_h7/CMakeLists.txt
│
├─ project(base C ASM)                     # 唯一的 project() 调用
├─ include(../../apps/config.cmake)         # 加载 ROBOT/BOARD 配置
├─ include(../../apps/generate_headers.cmake) # 生成 C 头文件
│
├─ add_subdirectory(cmake/stm32cubemx)      # STM32 HAL + CubeMX 代码
├─ add_subdirectory(../../threadx)          # ThreadX RTOS
├─ add_subdirectory(../../CherryUSB)        # USB 协议栈（通过 .cmake 脚本）
├─ add_subdirectory(../../utils)            # 工具库（日志、FIFO）
├─ add_subdirectory(../../board/bsp)        # BSP 硬件抽象
├─ add_subdirectory(../../robot)            # 机器人初始化层
├─ add_subdirectory(../../modules)          # 功能模块（条件编译）
├─ add_subdirectory(../../apps)             # 应用层（机器人业务逻辑）
├─ add_subdirectory(../../CMSIS-DSP)        # ARM DSP 数学库
│
└─ target_link_libraries(base.elf ...)      # 链接所有库生成 ELF
```

### 2.3 构建产物

```
build/
├── damiao_h7/Debug/
│   ├── base.elf              ← 最终固件
│   ├── base.map              ← 内存映射
│   ├── compile_commands.json ← clangd 需要的编译数据库
│   └── generated/            ← 自动生成的 C 头文件
│       ├── robot_def.h
│       └── module_config.h
└── compile_commands.json     ← VS Code 任务自动复制到这里
```

---

## 3. 工具链配置

### 3.1 工具链文件

位于 `board/<开发板>/cmake/gcc-arm-none-eabi.cmake`。

工具链文件在 CMake 配置阶段**最先**被加载（通过 `CMAKE_TOOLCHAIN_FILE`），在 `project()` 调用之前。它设定了编译器、链接器和基本编译标志。

### 3.2 关键配置

```cmake
# 交叉编译器前缀
set(CMAKE_C_COMPILER    arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER  arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)

# 跳过链接检查（嵌入式目标没有可用的链接器脚本时无法链接）
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 基本标志
set(COMMON_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb")
set(CMAKE_C_FLAGS   "${COMMON_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS "${COMMON_FLAGS} -Wl,--gc-sections -Wl,--print-memory-usage --specs=nano.specs")
```

### 3.3 两块开发板的工具链差异

| 参数 | damiao_h7 | dji_c |
|------|-----------|-------|
| MCU | STM32H723VG | STM32F407IG |
| CPU 核心 | Cortex-M7 | Cortex-M4 |
| FPU | `fpv5-d16`（双精度） | `fpv4-sp-d16`（单精度） |
| 频率 | 480 MHz | 168 MHz |
| DWT 初始化 | `BSP_DWT_Init(480)` | `BSP_DWT_Init(168)` |

### 3.4 关键标志解释

| 标志 | 作用 |
|------|------|
| `-ffunction-sections` | 每个函数放入独立段，配合 `--gc-sections` 移除未使用函数 |
| `-fdata-sections` | 每个数据变量放入独立段 |
| `-Wl,--gc-sections` | 链接时移除未引用的段，减小固件体积 |
| `-Wl,--print-memory-usage` | 链接后打印 Flash/RAM 使用量 |
| `--specs=nano.specs` | 使用 newlib-nano 精简 C 库 |

---

## 4. 配置系统：选择机器人与板型

### 4.1 核心配置文件

**`apps/config.cmake`** 是整个项目的配置中心：

```cmake
set(ROBOT "sentry")     # 机器人类型
set(BOARD "gimbal")     # 板型
```

支持的机器人：`hero`, `engineer`, `infantry3`, `infantry4`, `infantry5`, `drone`, `sentry`, `darts`, `customcontrol`

支持的板型：`single`（单板整机）, `gimbal`（云台板）, `chassis`（底盘板）

### 4.2 配置传播流程

```
apps/config.cmake
│
├─ include(modules/module_config.cmake)     # 加载所有模块的默认参数
├─ include(apps/${ROBOT}/robot.cmake)       # 加载机器人特定覆盖
│
├─ string(TOUPPER ${ROBOT} ROBOT_UPPER)     # 生成大写版本
├─ string(TOUPPER ${BOARD} BOARD_UPPER)
│
├─ set(${BOARD_UPPER}_BOARD 1)              # 设置板型宏（GIMBAL_BOARD=1）
│
└─ foreach(_m IN ${MODULES_${BOARD_UPPER}}) # 遍历该板型启用的模块列表
      set(MODULE_${_m} 1)                   #   MODULE_BMI088=1, MODULE_INS=1, ...
```

### 4.3 机器人特定覆盖

**`apps/sentry/robot.cmake`** 覆盖了默认配置：

```cmake
# 包含默认值
include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

# 覆盖：哨兵云台板启用这些模块
set(MODULES_GIMBAL   OFFLINE REMOTE BMI088 INS WT606 MOTOR VISION BOARDCOMM)
set(MODULES_CHASSIS  OFFLINE BMI088 INS REFEREE MOTOR BOARDCOMM)

# 覆盖：硬件句柄
set(REMOTE_UART   huart3)         # SBUS 遥控器接在 UART3
set(REMOTE_VT_UART huart6)        # 图传接在 UART6
set(BOARDCOMM_CAN BSP_CAN_HANDLE2) # 板间通信使用 CAN2
```

这种 **"默认值 + 覆盖"** 的两层模式避免了各机器人的重复配置。

---

## 5. 头文件生成：从 CMake 变量到 C 宏

### 5.1 为什么需要头文件生成

C 预处理器无法直接读取 CMake 变量。因此需要 `apps/generate_headers.cmake` 将 CMake 的配置变量"翻译"为 C 的 `#define` 宏。

### 5.2 生成的头文件

**`robot_def.h`** — 机器人类型枚举：
```c
#define CURRENT_ROBOT   ROBOT_SENTRY   // 由 CMake 的 ROBOT=sentry 生成
#define ROBOT_TYPE      CURRENT_ROBOT
```

**`module_config.h`** — 模块开关与参数：
```c
#define MODULE_BMI088    1
#define MODULE_INS       1
#define MODULE_REFEREE   0   // 云台板不需要裁判系统
#define MODULE_SUPERCAP  0
// ...
#define MOTOR_TASK_PRIORITY 12
#define INS_TASK_STACK_SIZE 1024
```

### 5.3 生成机制

`_gen_cmakedefine()` 函数模拟 CMake 内置的 `#cmakedefine` 行为：

```cmake
function(_gen_cmakedefine var)
    if(${var})
        file(APPEND ${_generated_h} "#define ${var} ${${var}}\n")
    else()
        file(APPEND ${_generated_h} "/* #undef ${var} */\n")
    endif()
endfunction()
```

### 5.4 头文件的引入

生成的 `module_config.h` 通过 **`-include module_config.h`** 编译选项在每个源文件之前强制包含：

```cmake
target_compile_options(modules PRIVATE -include module_config.h)
```

这确保了所有 `#if MODULE_XXX` 守卫在任何 `.c` 文件中都能正常工作。

### 5.5 关键知识点：生成头文件的包含路径

生成目录路径的计算：
```cmake
# modules/CMakeLists.txt 中：
get_filename_component(_generated_dir "${CMAKE_CURRENT_BINARY_DIR}/../generated" ABSOLUTE)
target_include_directories(modules PRIVATE ${_generated_dir})
```

这指向 `build/<board>/<config>/generated/`，确保开发和构建代码隔离。

---

## 6. 目标类型：OBJECT / STATIC / INTERFACE

本项目使用了 CMake 三种主要的库目标类型，各有其适用场景。

### 6.1 OBJECT 库 — 项目内部层

```cmake
add_library(utils OBJECT
    utils_init.c
    ulog/SEGGER_RTT.c
    ulog/ulog.c
    # ...
)
```

**特点**：
- 不生成 `.a` 归档文件
- 源文件直接"注入"到最终可执行文件的编译中
- 可用于需要链接时优化（LTO）的场景
- 本项目使用 OBJECT 库的场景：`utils`、`bsp`、`robot`、`modules`、`app`

**为什么用 OBJECT？**因为这些模块之间需要共享大量的预处理宏（尤其是 `module_config.h` 中的 `MODULE_XXX`），而且最终链接时可以通过 `--gc-sections` 移除单个模块中未使用的函数。

### 6.2 STATIC 库 — 第三方代码

```cmake
add_library(threadx STATIC src1.c src2.c ...)
add_library(threadx ALIAS azrtos::threadx)   # 命名空间别名
add_library(CMSISDSP STATIC ...)
add_library(cherryusb STATIC ...)
```

**特点**：
- 生成 `.a` 归档文件后链接到可执行文件
- 适合不需要在每处使用点重新编译的第三方库
- 本项目使用 STATIC 库的场景：`threadx`、`cherryusb`、`CMSISDSP`

### 6.3 INTERFACE 库 — 纯头文件/编译定义

```cmake
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE
    Drivers/STM32H7xx_HAL_Driver/Inc
    Drivers/CMSIS/Include
    # ...
)
target_compile_definitions(stm32cubemx INTERFACE
    USE_HAL_DRIVER
    STM32H723xx
)
```

**特点**：
- 不编译任何源文件，不生成任何产物
- 仅携带**编译选项**（头文件路径、宏定义、链接依赖）
- 本项目用于 `stm32cubemx`：所有 HAL 头文件路径通过它传播

### 6.4 三种类型对比

| 类型 | 生成产物 | include 传播 | 适用场景 |
|------|---------|-------------|---------|
| OBJECT | 无（直接编译到可执行文件） | 通过 | 项目内部层 |
| STATIC | `.a` 归档文件 | 通过 | 第三方库 |
| INTERFACE | 无 | 通过 | 仅头文件/仅编译选项 |
| EXECUTABLE | `.elf` | 仅继承 | 最终固件 |

---

## 7. 条件编译：MODULE_XXX 机制

### 7.1 三级条件编译

```
CMake 变量 ──→ 生成头文件 ──→ C 预处理器
   │                 │               │
   │ (配置时)         │ (生成时)       │ (编译时)
   │                 │               │
 set(MODULE_    #define MODULE_   #if MODULE_
   BMI088 1)      BMI088 1         BMI088
                                  #include "..."
                                  #endif
```

### 7.2 CMake 层级：条件包含源文件

```cmake
# modules/CMakeLists.txt

# 始终编译
list(APPEND _module_sources
    module_init.c
    algorithm/pid.c
    algorithm/lqr.c
    # ...
)

# 条件编译
if(MODULE_BMI088)
    list(APPEND _module_sources
        BMI088/module_bmi088.c
    )
    list(APPEND _module_includes
        ${CMAKE_CURRENT_LIST_DIR}/BMI088
    )
endif()
```

这种 **累加器模式**（先空列表，逐步 `list(APPEND ...)`）使得条件块可以独立添加，无需修改 `add_library()` 调用。

### 7.3 C 层级：条件初始化

```c
// modules/module_init.c
void MODULE_Init(void)
{
#if MODULE_BMI088
    Module_BMI088_init();
#endif
#if MODULE_INS
    Module_INS_Init();
#endif
    // ...
}
```

### 7.4 模块参数的条件默认值

每个模块头文件使用 `#ifndef` 提供编译期默认值：

```c
// modules/BMI088/module_bmi088.h
#ifndef BMI088_TEMP_ENABLE
#define BMI088_TEMP_ENABLE 0
#endif
```

CMake 可以通过 `generate_headers.cmake` 覆盖这些值（在头文件包含之前通过 `-include module_config.h` 注入）。

---

## 8. 属性传递：PUBLIC / PRIVATE / INTERFACE

### 8.1 可传递的属性

| 属性 | 说明 |
|------|------|
| `target_include_directories` | 头文件搜索路径 |
| `target_compile_definitions` | 编译宏定义 |
| `target_compile_options` | 编译选项 |
| `target_link_libraries` | 链接依赖 |

### 8.2 三种传递方式

```cmake
# PUBLIC：自己和依赖自己的目标都能看到
target_link_libraries(bsp PUBLIC stm32cubemx azrtos::threadx)

# PRIVATE：只有自己能用到
target_compile_options(modules PRIVATE -O2 -include module_config.h)

# INTERFACE：自己不使用，仅传递给依赖者
target_include_directories(stm32cubemx INTERFACE Drivers/STM32H7xx_HAL_Driver/Inc)
```

### 8.3 本项目的依赖链

```
app ──PUBLIC──→ modules ──PUBLIC──→ bsp ──PUBLIC──→ utils ──PUBLIC──→ stm32cubemx
 │                │                  │                 │                  │
 │                │                  │                 │                  └── HAL include 路径
 │                │                  │                 └── azrtos::threadx
 │                │                  └── cherryusb
 │                └── CMSISDSP
 └── robot
```

因为每层都用 `PUBLIC` 向上传递依赖，所以 `app` 最终能访问所有底层库的头文件和符号。

### 8.4 关键技巧：-include 强制包含

```cmake
target_compile_options(modules PRIVATE -include module_config.h)
target_compile_options(app PRIVATE -include module_config.h)
```

`-include` 是 GCC 的一个选项，效果等同于在每个源文件开头插入 `#include "module_config.h"`。这确保了所有模块源文件自动获得配置宏，无需逐个手动 `#include`。

---

## 9. 双板型架构：damiao_h7 与 dji_c

### 9.1 共享与差异

```
共享（同一份源代码）：
  apps/        robot/      modules/      board/bsp/      utils/
  CMSIS-DSP/   CherryUSB/  threadx/

每块板独立：
  board/damiao_h7/                board/dji_c/
  ├── CMakeLists.txt              ├── CMakeLists.txt
  ├── Core/                       ├── Core/
  │   └── Src/main.c              │   └── Src/main.c
  ├── Drivers/ (HAL)              ├── Drivers/ (HAL)
  ├── cmake/toolchain.cmake       ├── cmake/toolchain.cmake
  └── STM32H723XG_FLASH.ld       └── STM32F407XX_FLASH.ld
```

### 9.2 如何在共享代码中区分平台

在 BSP 层，通过预处理器宏区分：

```c
// board/bsp/bsp_init.c
void BSP_Init(void)
{
#if defined(STM32H723xx)
    // H7 特有的电源使能
    HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin, GPIO_PIN_RESET);
    BSP_DWT_Init(480);
    cdc_acm_init(0, USB_OTG_HS_PERIPH_BASE);  // 高速 USB
#elif defined(STM32F407xx)
    BSP_DWT_Init(168);
    cdc_acm_init(0, USB_OTG_FS_PERIPH_BASE);  // 全速 USB
#endif
    // 以下为共享的初始化...
}
```

在 CMake 中，通过 `THREADX_ARCH` 变量区分：

```cmake
set(THREADX_ARCH cortex_m7)  # 在 damiao_h7 CMakeLists.txt 中
set(THREADX_ARCH cortex_m4)  # 在 dji_c CMakeLists.txt 中
```

---

## 10. STM32CubeMX 集成

### 10.1 CubeMX 生成代码的结构

STM32CubeMX 为一个 `.ioc` 项目生成以下代码：

```
board/damiao_h7/
├── base.ioc                    ← CubeMX 项目文件
├── Core/
│   ├── Inc/
│   │   └── main.h / gpio.h / dma.h / ...
│   └── Src/
│       ├── main.c              ← main() + 外设初始化
│       ├── gpio.c / dma.c / ...
│       └── startup_stm32h723xx.s ← 启动汇编
└── Drivers/
    ├── STM32H7xx_HAL_Driver/   ← HAL 驱动源码
    └── CMSIS/
        └── Include/            ← CMSIS-Core 头文件
```

### 10.2 CMake 如何集成

```cmake
# board/damiao_h7/cmake/stm32cubemx/CMakeLists.txt

# INTERFACE 库：携带 HAL 头文件路径和编译定义
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_LIST_DIR}/../../Drivers/CMSIS/Include
    ${CMAKE_CURRENT_LIST_DIR}/../../Core/Inc
)
target_compile_definitions(stm32cubemx INTERFACE
    USE_HAL_DRIVER
    STM32H723xx
)

# OBJECT 库：编译 HAL 驱动源码
add_library(STM32_Drivers OBJECT
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    # ... 所有需要的 HAL 模块
)

# CubeMX 生成的启动文件直接加入可执行文件
target_sources(base PRIVATE Core/Src/startup_stm32h723xx.s)
target_sources(base PRIVATE Core/Src/main.c Core/Src/gpio.c ...)
```

### 10.3 HAL 时基的注意事项

CubeMX 默认使用 SysTick 作为 HAL 时基。但因为 ThreadX 拥有 SysTick，本项目将 HAL 时基配置为 TIM23：

```
# base.ioc 中：
NVIC.TimeBase=TIM23
```

---

## 11. 第三方库集成：ThreadX / CherryUSB / CMSIS-DSP

### 11.1 ThreadX 集成

```cmake
# threadx/CMakeLists.txt
set(THREADX_ARCH cortex_m7)  # 由板级 CMakeLists.txt 设置

add_library(threadx STATIC
    ${THREADX_COMMON_SOURCES}            # 通用内核代码
    ports/${THREADX_ARCH}/src/...        # 架构特定的汇编文件
)

# 提供命名空间别名（现代 CMake 风格）
add_library(azrtos::threadx ALIAS threadx)

target_compile_definitions(threadx PUBLIC
    TX_INCLUDE_USER_DEFINE_FILE    # 启用 tx_user.h 用户配置
    TX_ENABLE_FPU_SUPPORT          # 启用 FPU 上下文保存
)
```

### 11.2 CherryUSB 集成

使用 `.cmake` 脚本而非 `CMakeLists.txt`：

```cmake
# board/damiao_h7/CMakeLists.txt
include(../../CherryUSB/cherryusb.cmake)  # 设置 cherryusb_srcs / cherryusb_incs
list(REMOVE_DUPLICATES cherryusb_srcs)     # 去重
add_library(cherryusb STATIC ${cherryusb_srcs})
target_compile_options(cherryusb PRIVATE -O3 -ffast-math)
```

### 11.3 CMSIS-DSP 集成

```cmake
# 覆盖 CMSIS-DSP 的默认优化
set(LOOPUNROLL ON CACHE BOOL "Enable loop unrolling" FORCE)
set(DISABLEFLOAT16 ON CACHE BOOL "Disable float16" FORCE)

add_subdirectory(../../CMSIS-DSP)

# 提供 CMSIS-Core 头文件路径
target_include_directories(CMSISDSP PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include)

# 重写优化级别
target_compile_options(CMSISDSP PRIVATE -O3 -ffast-math -fno-math-errno -flto)
```

---

## 12. 链接器脚本与内存布局

### 12.1 链接器脚本的作用

链接器脚本（`.ld` 文件）告诉链接器：
- 代码和数据应该放在内存的哪个位置
- 各段（`.text`、`.data`、`.bss`、栈、堆）的起始地址和大小
- 哪些段需要从 Flash 复制到 RAM

### 12.2 damiao_h7 内存区域

```
FLASH   (0x08000000) 1024KB  程序代码（只读）
DTCMRAM (0x20000000) 128KB   紧耦合数据 RAM — 线程栈 (.dtcmram)
RAM_D1  (0x24000000) 320KB   AXI SRAM — DMA 缓冲区 (.RAM_D1)
RAM_D2  (0x30000000) 32KB    D2 域 SRAM
RAM_D3  (0x38000000) 16KB    D3 域 SRAM
ITCMRAM (0x00000000) 64KB    紧耦合指令 RAM — 关键代码 (.itcmram)
```

### 12.3 代码中如何使用内存区域

```c
// bsp_def.h 中：
// 线程栈 —— 放到 DTCM（零延迟访问）
#define APPS_STACK_SECTION __attribute__((section(".dtcmram")))

// DMA 缓冲区 —— 放到 RAM_D1（DMA 可访问）
#define BUFFER_SECTION __attribute__((section(".RAM_D1"))) __attribute__((aligned(32)))

// 使用示例：
APPS_STACK_SECTION static uint8_t robot_control_thread_stack[1024];
BUFFER_SECTION static uint8_t spi_dma_buffer[256];
```

### 12.4 Cache 一致性处理

Cortex-M7 有数据和指令缓存。当 DMA 访问内存时，需要手动维护缓存：

```c
// bsp_def.h (仅对 H7 生效)
#define BSP_CACHE_CLEAN(addr, len)      SCB_CleanDCache_by_Addr((uint32_t*)(addr), (int32_t)(len))
#define BSP_CACHE_INVALIDATE(addr, len) SCB_InvalidateDCache_by_Addr((uint32_t*)(addr), (int32_t)(len))

// 使用场景：
// TX DMA 之前：清理缓存（将脏数据写入内存）
BSP_CACHE_CLEAN(tx_buffer, tx_len);

// RX DMA 之后：无效化缓存（丢弃过时数据，重新从内存读取）
BSP_CACHE_INVALIDATE(rx_buffer, rx_len);
```

---

## 13. CMakePresets：预设构建配置

### 13.1 Presets 文件

**`board/damiao_h7/CMakePresets.json`**：

```json
{
    "configurePresets": [
        {
            "name": "default",
            "hidden": true,
            "generator": "Ninja",
            "binaryDir": "${sourceDir}/build/${presetName}",
            "toolchainFile": "${sourceDir}/cmake/gcc-arm-none-eabi.cmake"
        },
        {
            "name": "Debug",
            "inherits": "default",
            "cacheVariables": {
                "CMAKE_BUILD_TYPE": "Debug"
            }
        },
        {
            "name": "Release",
            "inherits": "default",
            "cacheVariables": {
                "CMAKE_BUILD_TYPE": "Release"
            }
        }
    ],
    "buildPresets": [
        { "name": "Debug", "configurePreset": "Debug" },
        { "name": "Release", "configurePreset": "Release" }
    ]
}
```

### 13.2 使用 Presets

```bash
# 使用预设配置（--preset 必须在 -S/-B 之前）
cmake -S board/damiao_h7 -B build/damiao_h7/Debug --preset Debug -G Ninja

# 使用预设构建
cmake --build build/damiao_h7/Debug --config Debug
```

### 13.3 Presets 优势

- 避免每次输入冗长的工具链路径和构建类型选项
- `hidden: true` 的 "default" 预设可以被继承但不会出现在用户选择列表中
- `binaryDir` 使用宏 `${sourceDir}` 自动适配不同开发人员的工作目录

---

## 14. 实用技巧总结

### 14.1 调试 CMake

```bash
# 查看详细输出
cmake -S board/damiao_h7 -B build --preset Debug -G Ninja --trace

# 只查看某条消息
cmake -S board/damiao_h7 -B build --preset Debug -G Ninja --trace-expand 2>&1 | grep "MY_VAR"

# 查看缓存变量
cmake -S board/damiao_h7 -B build -L
```

### 14.2 项目用到的 CMake 高级模式

| 模式                    | 使用位置                     | 说明               |
| --------------------- | ------------------------ | ---------------- |
| `configure_file()`    | 复制 `tx_user.h`           | 生成时文件复制，避免污染源树   |
| `file(GLOB_RECURSE)`  | `apps/CMakeLists.txt`    | 通配收集机器人源码        |
| `list(APPEND ...)`    | `modules/CMakeLists.txt` | 条件累加源文件          |
| `#cmakedefine` 模拟     | `generate_headers.cmake` | 将 CMake 变量转为 C 宏 |
| `CACHE ... FORCE`     | 板级 CMakeLists.txt        | 强制覆盖子项目默认值       |
| Generator expressions | `threadx/CMakeLists.txt` | 编译器 ID 条件标志      |
| `target_sources()`    | CubeMX CMakeLists.txt    | 向已有目标追加源文件       |

### 14.3 常见问题

**Q: 修改了 `apps/config.cmake` 中的 ROBOT/BOARD 后需要做什么？**
A: 重新配置：`cmake -S board/damiao_h7 -B build/damiao_h7/Debug --preset Debug -G Ninja`

**Q: 添加了新的 `.c` 文件但没有被编译？**
A: 检查是否需要更新 CMakeLists.txt。大多数情况下，`apps/` 目录使用 `GLOB_RECURSE` 自动发现，但 `modules/` 需要手动添加。

**Q: 编译错误提示找不到 `module_config.h`？**
A: 确保已运行配置步骤生成该文件。检查 `build/<board>/Debug/generated/` 目录是否存在。

**Q: 如何增加或减少编译优化级别？**
A: 修改对应层的 `target_compile_options`。大部分模块使用 `-O2`，ThreadX/CherryUSB/CMSISDSP 使用 `-O3`。

---

## 15. CMake 嵌套关系深度解析

### 15.1 `include()` vs `add_subdirectory()` —— 本项目最大的设计分歧点

这是理解本项目 CMake 嵌套结构的核心。两种机制行为截然不同：

|                                | `include()`                      | `add_subdirectory()`                   |
| :----------------------------- | :------------------------------- | :------------------------------------- |
| **作用域**                        | 调用者作用域内展开，**变量污染**               | 新建子作用域，变量默认不传回                         |
| **传回变量**                       | 自然共享，无需额外操作                      | 需要 `set(... PARENT_SCOPE)`             |
| **典型用途**                       | 配置文件、模板展开                        | 独立模块、子项目                               |
| **`CMAKE_CURRENT_SOURCE_DIR`** | 不变，仍是调用者的目录                      | 变为子目录路径                                |
| **可跳转**                        | 依赖 `CMAKE_CURRENT_LIST_DIR` 定位自身 | 天然隔离，`${CMAKE_CURRENT_SOURCE_DIR}` 即自己 |

> **一句话**：`include()` = 把另一个文件的内容**粘贴到当前位置**；`add_subdirectory()` = 进入子目录**另起炉灶**。

### 15.2 配置阶段：`include()` 链的展开顺序

以编译 `damiao_h7 + sentry + gimbal` 为例，追踪 `board/damiao_h7/CMakeLists.txt` 中**第 28 行** `include(apps/config.cmake)` 的完整展开：

```
board/damiao_h7/CMakeLists.txt  (line 28)
│  include(${CMAKE_SOURCE_DIR}/../../apps/config.cmake)
│
├─► apps/config.cmake  (line 9)
│   │  include(${CMAKE_CURRENT_LIST_DIR}/../modules/module_config.cmake)
│   │
│   ├─► modules/module_config.cmake   ← 第 1 次加载
│   │   set(MODULES_GIMBAL OFFLINE REMOTE BMI088 INS MOTOR VISION BOARDCOMM)
│   │   set(REMOTE_UART huart3)   ← 默认值
│   │   set(MOTOR_TASK_PRIORITY 12)
│   │   ... (所有模块的默认参数)
│   │
│   ▼ 回到 apps/config.cmake
│
│   (line 12) include(${CMAKE_CURRENT_LIST_DIR}/${ROBOT}/robot.cmake)
│   │                    ↓ ROBOT = "sentry"
│   │  include(apps/sentry/robot.cmake)
│   │
│   ├─► apps/sentry/robot.cmake  (line 3)
│   │   │  include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)
│   │   │
│   │   ├─► modules/module_config.cmake   ← 第 2 次加载 ⚠️
│   │   │   (重新 set 所有默认值)
│   │   │
│   │   ▼ 回到 apps/sentry/robot.cmake
│   │   │
│   │   (覆盖) set(MODULES_GIMBAL OFFLINE REMOTE BMI088 INS WT606 MOTOR VISION BOARDCOMM)
│   │   (覆盖) set(REMOTE_UART huart3)
│   │   (覆盖) set(REMOTE_VT_UART huart6)
│   │   ...
│   │
│   ▼ 回到 apps/config.cmake
│
│   (line 16) string(TOUPPER ${ROBOT} ROBOT_UPPER)   → SENTRY
│   (line 17) string(TOUPPER ${BOARD} BOARD_UPPER)   → GIMBAL
│   (line 25) set(GIMBAL_BOARD 1)
│   (line 28-35) foreach → MODULE_BMI088=1, MODULE_INS=1, ...
│
▼ 回到 board/damiao_h7/CMakeLists.txt
```

**关键发现**：`module_config.cmake` 被 include 了**两次**（一次在 `config.cmake`，一次在 `robot.cmake`）。第二次 include 把默认值全部重置，然后 `robot.cmake` 用 `set()` 覆盖差异项。这是一种**"全量重置 + 覆盖"**模式，优点是逻辑简单，缺点是重复执行。

### 15.3 `add_subdirectory()` 链：模块树的构建

`include()` 链结束后，执行回到 `board/damiao_h7/CMakeLists.txt`，开始 `add_subdirectory()` 阶段：

```
board/damiao_h7/CMakeLists.txt
│
├── add_subdirectory(cmake/stm32cubemx)          ← INTERFACE 库: 携带 HAL 头文件 + 宏定义
│                                            ← OBJECT 库: 编译 HAL 驱动源码
│
├── add_subdirectory(../../threadx threadx)       ← STATIC 库: ThreadX 内核 + arch 端口
│   └── add_subdirectory(common)                 ← threadx/common/CMakeLists.txt
│
├── include(../../CherryUSB/cherryusb.cmake)      ← ⚠️ 用 include 而非 add_subdirectory
│   (暴露 cherryusb_srcs / cherryusb_incs 变量)
│   └── add_library(cherryusb STATIC ${cherryusb_srcs})   ← 手动建库
│
├── add_subdirectory(../../utils ...)             ← OBJECT 库: 日志、FIFO
├── add_subdirectory(../../board/bsp ...)         ← OBJECT 库: GPIO/UART/SPI/CAN/...
├── add_subdirectory(../../robot ...)             ← OBJECT 库: 机器人初始化
├── add_subdirectory(../../modules ...)           ← OBJECT 库: 条件编译模块树
├── add_subdirectory(../../apps ...)              ← OBJECT 库: GLOB_RECURSE 收集业务代码
├── add_subdirectory(../../CMSIS-DSP ...)         ← STATIC 库: ARM DSP 数学库
│
└── target_link_libraries(base.elf
        stm32cubemx azrtos::threadx utils bsp robot CMSISDSP cherryusb modules app)
```

### 15.4 `../../` 路径穿越 —— 代价与收益

本项目没有顶层 `CMakeLists.txt`，入口直接是 `board/damiao_h7/CMakeLists.txt`。所有 `add_subdirectory()` 使用 `../../` 向上穿越两级再进入目标目录：

```
board/damiao_h7/                    ← cmake -S 入口（CMAKE_SOURCE_DIR）
├── CMakeLists.txt ──include──→ ../../apps/config.cmake     → apps/config.cmake
│                   ──add_subdir→ ../../threadx              → threadx/
│                   ──add_subdir→ ../../utils                → utils/
│                   ...
```

**收益**：
- 不需要顶层 CMakeLists.txt，每个板独立入口
- CI 可以直接 `cmake -S board/damiao_h7` 不必指定额外参数

**代价**：
- `add_subdirectory(../../threadx threadx)` 中第一个 `../../threadx` 是**源码路径**（相对 `CMAKE_SOURCE_DIR`），第二个 `threadx` 是**构建目录名**（相对 `CMAKE_BINARY_DIR`）——新人容易混淆
- IDE（如 CLion、VS Code）在 `CMAKE_SOURCE_DIR` ≠ 项目根目录时，路径解析可能出错
- `CMAKE_CURRENT_LIST_DIR` 和 `CMAKE_CURRENT_SOURCE_DIR` 在 `add_subdirectory(../../...)` 后的语义需要仔细理解

### 15.5 嵌套关系的可视化总结

```
                    ┌──────────────────────────────────────┐
                    │     board/damiao_h7/CMakeLists.txt    │
                    │     cmake_minimum_required            │
                    │     project(base)                     │
                    │     add_executable(base)              │
                    └────┬──────┬──────┬──────┬────────────┘
                         │      │      │      │
              ┌──────────┘      │      │      └──────────────┐
              ▼                 ▼      ▼                     ▼
        include() 链    add_subdirectory() 链         target_link_libraries
        ─────────────   ─────────────────────         ─────────────────────
        config.cmake    stm32cubemx (INTERFACE)       base.elf ← 所有库
          ├─ module_    threadx     (STATIC)
          │  config     cherryusb  (STATIC)
          ├─ robot      utils      (OBJECT)
          │  .cmake     bsp        (OBJECT)
          ├─ generate_  robot      (OBJECT)
          │  headers    modules    (OBJECT)         ◄── 条件编译核心
          └─ 变量就绪    apps       (OBJECT)         ◄── GLOB_RECURSE
                        CMSISDSP   (STATIC)

        作用: 配置       作用: 编译                            作用: 链接
        时机: 第28-32行  时机: 第48-83行                       时机: 第115-126行
```

---

## 16. 可优化方向

> 以下建议按**收益/成本比**排序，标注了难度和风险。

### 16.1 低风险快速优化

#### 16.1.1 `module_config.cmake` 被重复 include

**现状**：`apps/config.cmake` 和 `apps/<robot>/robot.cmake` 都 `include(module_config.cmake)`，导致同一文件被加载两次，所有默认参数被设置两遍。

**优化**：在 `module_config.cmake` 开头加 `include_guard(GLOBAL)`（CMake 3.10+）：

```cmake
# modules/module_config.cmake 第 1 行加
include_guard(GLOBAL)
```

**效果**：第二次 `include()` 时直接跳过，不再重复执行。零副作用。

#### 16.1.2 `GLOB_RECURSE` 不自动发现新文件

**现状**：`apps/CMakeLists.txt` 使用 `file(GLOB_RECURSE _robot_sources ${_robot_board_dir}/*.c)` 收集兵种业务代码。新增 `.c` 文件后，CMake 不会自动重新扫描——必须手动重新运行 `cmake` 配置步骤。

**优化**：添加 `CONFIGURE_DEPENDS` 标志（CMake 3.12+）：

```cmake
# apps/CMakeLists.txt 第 10 行
file(GLOB_RECURSE _robot_sources CONFIGURE_DEPENDS ${_robot_board_dir}/*.c)
```

**效果**：每次构建时 CMake 自动检查目录变化，新文件自动加入编译。对嵌入式项目（源文件 < 500 个）性能影响可忽略。

#### 16.1.3 生成头文件目录路径简化

**现状**：`modules/CMakeLists.txt` 第 12-13 行：

```cmake
get_filename_component(_board_binary_dir ${CMAKE_CURRENT_BINARY_DIR} DIRECTORY)
set(_generated_dir ${_board_binary_dir}/generated)
```

这依赖于 `CMAKE_CURRENT_BINARY_DIR` = `build/<board>/<config>/modules`，向上取父目录 = `build/<board>/<config>`。

**优化**：用 `CMAKE_BINARY_DIR`（即板级构建根目录）直接定位：

```cmake
set(_generated_dir ${CMAKE_BINARY_DIR}/generated)
```

**收益**：一行替代两行，消除隐式的目录嵌套假设。

### 16.2 中风险架构优化

#### 16.2.1 CherryUSB 集成方式不一致

**现状**：CherryUSB 通过 `include(cherryusb.cmake)` 暴露 `cherryusb_srcs` / `cherryusb_incs` 变量，然后手动建库。而其他第三方库（ThreadX、CMSIS-DSP）都用 `add_subdirectory()` 自动建库。

```cmake
# 现状（board/damiao_h7/CMakeLists.txt 第 59-69 行）
include(../../CherryUSB/cherryusb.cmake)       # 暴露 cherryusb_srcs, cherryusb_incs
list(REMOVE_DUPLICATES cherryusb_srcs)          # 手动去重
add_library(cherryusb STATIC ${cherryusb_srcs}) # 手动建库
target_include_directories(cherryusb PUBLIC ...)
target_compile_options(cherryusb PRIVATE -O3 ...)
```

**优化**：将 CherryUSB 封装为一个 `CMakeLists.txt`，用 `add_subdirectory()` 接入：

```cmake
# 建议改为：
add_subdirectory(../../CherryUSB cherryusb)
# CherryUSB/CMakeLists.txt 内部建好 cherryusb target 并设好 include/compile_options
```

**收益**：风格统一，变量不再泄漏到板级 CMakeLists.txt 作用域。

#### 16.2.2 依赖关系存在冗余

**现状**：各层的 `target_link_libraries` 存在"超链接"——子模块链接了并非直接需要的依赖：

```cmake
# modules/CMakeLists.txt — 链接了几乎所有底层库
target_link_libraries(modules PUBLIC stm32cubemx azrtos::threadx m CMSISDSP bsp utils cherryusb)

# apps/CMakeLists.txt — 也链接了大量底层库
target_link_libraries(app PUBLIC stm32cubemx azrtos::threadx utils bsp CMSISDSP modules)
```

CMake 的 `PUBLIC` 依赖会传递，所以 `app → modules → utils → stm32cubemx` 这条链中，`app` 不需要显式链接 `utils` 和 `stm32cubemx`。

**优化**：精简依赖链，只声明本层直接使用的依赖：

```cmake
# modules/CMakeLists.txt —— modules 直接调用 bsp 和 utils 的函数
target_link_libraries(modules PUBLIC bsp utils)
# bsp PUBLIC 依赖 stm32cubemx 和 threadx，会自动传递上来

# apps/CMakeLists.txt —— apps 直接调用 modules 和 CMSISDSP
target_link_libraries(app PUBLIC modules CMSISDSP)
```

**风险**：需要逐一确认每层的实际 `#include` 和函数调用关系，改错会导致链接失败。**建议新项目采用，老项目保持现状**。

#### 16.2.3 缺少顶层 `CMakeLists.txt`

**现状**：项目没有顶层 `CMakeLists.txt`，`cmake -S` 直接指向 `board/damiao_h7/`。这意味着不能一键构建所有板型，CI 必须逐个调用。

**优化**：添加顶层 `CMakeLists.txt`，用 `ExternalProject` 驱动多板构建：

```cmake
# 顶层 CMakeLists.txt
include(ExternalProject)
ExternalProject_Add(damiao_h7
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/board/damiao_h7
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -G Ninja
        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/board/damiao_h7/cmake/gcc-arm-none-eabi.cmake
)
ExternalProject_Add(dji_c ...)
```

**收益**：`cmake --build .` 一键构建所有板型。但引入额外复杂度，对于当前双板规模**性价比不高**。建议板型 > 3 块时再做。

### 16.3 优化建议总览

| 编号 | 优化点 | 难度 | 风险 | 建议 |
|:--|:--|:--|:--|:--|
| 16.1.1 | `include_guard(GLOBAL)` | 低 | 无 | ✅ **立即做** |
| 16.1.2 | `GLOB_RECURSE CONFIGURE_DEPENDS` | 低 | 低 | ✅ 下次改代码顺手改 |
| 16.1.3 | 简化 `_generated_dir` 路径 | 低 | 低 | ✅ 顺手改 |
| 16.2.1 | CherryUSB 用 `add_subdirectory()` | 中 | 低 | ✅ 下个重构窗口做 |
| 16.2.2 | 精简 `target_link_libraries` | 中 | 中 | ⏸ 新项目采用 |
| 16.2.3 | 顶层 CMakeLists.txt | 中 | 中 | ⏸ 板型 > 3 时再做 |

---

## 相关笔记

- [[RM_BSP与模块教程]] — BSP 层和模块层的硬件接口详解，与 CMake 条件编译的模块一一对应
- [[RM_Sentry_系统架构详解]] — 哨兵固件的完整架构，配合理解 `ROBOT=sentry` 的构建链路
- [[RM_Chassis_串联轮腿步兵下位机控制系统设计]] — 步兵下位机架构，理解 `ROBOT=infantry3` 的 CMake 配置
