# CMakeLists.txt — CubeMX 生成代码的 CMake 集成

## 文件位置

`mas_embedded_threadx/board/dji_c/cmake/stm32cubemx/CMakeLists.txt`

## 作用

被 [[02_code_twin/board/dji_c/CMakeLists-txt]] 的 `add_subdirectory(cmake/stm32cubemx)` 加载。负责把 STM32CubeMX 自动生成的 HAL 驱动、外设初始化代码、启动文件接入 CMake 构建体系。

> 这个文件由 CubeMX 生成，但可以手动修改。误删后需要重新用 CubeMX 的 Generate Code 恢复，但恢复后的代码可能与原代码有差异。

---

## 原始代码（摘取）

### 宏定义与头文件路径

```cmake
set(MX_Defines_Syms 
    USE_HAL_DRIVER 
    STM32F407xx
    $<$<CONFIG:Debug>:DEBUG>
)

set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Include
)
```

| 宏定义                        | 含义                              |
| -------------------------- | ------------------------------- |
| `USE_HAL_DRIVER`           | 启用 HAL 库（而非 LL 库）               |
| `STM32F407xx`              | 指定芯片型号，HAL 库据此选择正确的寄存器定义        |
| `$<$<CONFIG:Debug>:DEBUG>` | 生成器表达式：仅在 Debug 配置下定义 `DEBUG` 宏 |

头文件路径指向 CubeMX 生成的 `Inc/` 和 `Drivers/` 目录。

### 应用层源文件

```cmake
set(MX_Application_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/can.c
    # ... adc.c, dma.c, i2c, iwdg, rng, rtc, spi, tim, usart, usb_otg ...
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/stm32f4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/stm32f4xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/stm32f4xx_hal_timebase_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../startup_stm32f407xx.s
)
```

这些是 CubeMX 根据 `.ioc` 配置生成的外设初始化代码：

| 文件                                   | 内容                                               |
| ------------------------------------ | ------------------------------------------------ |
| `main.c`                             | `main()` 函数 + 各外设的 `MX_XXX_Init()`               |
| `gpio.c` / `can.c` / `usart.c` / ... | 对应外设的引脚和参数初始化                                    |
| `stm32f4xx_it.c`                     | 中断服务函数                                           |
| `stm32f4xx_hal_msp.c`                | HAL MSP 回调（底层硬件初始化）                              |
| `stm32f4xx_hal_timebase_tim.c`       | HAL 时基使用 TIM 而非 SysTick（因为 SysTick 被 ThreadX 占用） |
| `startup_stm32f407xx.s`              | 启动汇编文件（中断向量表 + 复位处理）                             |

### HAL 驱动源文件

```cmake
set(STM32_Drivers_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    # ... 全部 HAL 驱动源码 ...
)
```

CubeMX 根据 `.ioc` 中启用的外设，只挑选需要的 HAL 驱动文件加入编译。

### 建库：INTERFACE + OBJECT

```cmake
# INTERFACE 库：携带头文件路径和宏定义，不编译任何源文件
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# OBJECT 库：编译 HAL 驱动源码，产物直接注入最终 elf
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)

# 应用层源码直接加入可执行文件
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})
target_link_libraries(${CMAKE_PROJECT_NAME} STM32_Drivers ${TOOLCHAIN_LINK_LIBRARIES})
```

- `add_library(INTERFACE)` → [[01_extracted/cmake/gcc-cmake-build#add_library(INTERFACE) - 只传属性不编译]]
- `target_include_directories` → [[01_extracted/cmake/gcc-cmake-build#target_include_directories - 头文件搜索路径]]
- `target_compile_definitions` → [[01_extracted/cmake/gcc-cmake-build#target_compile_definitions - 预处理宏]]
- `add_library(OBJECT)` → [[01_extracted/cmake/gcc-cmake-build#add_library(OBJECT) - .o 直接注入 elf]]
- `target_sources` → [[01_extracted/cmake/gcc-cmake-build#target_sources - 指定编译哪些源文件]]
- `target_link_libraries` → [[01_extracted/cmake/gcc-cmake-build#target_link_libraries - 链接哪些库]]
- `INTERFACE`/`PUBLIC`/`PRIVATE` 关键字 → [[01_extracted/cmake/gcc-cmake-build#五、作用域与 PRIVATE / PUBLIC / INTERFACE]]

**为什么分两个库？**

| 库 | 类型 | 携带什么 | 为什么这样设计 |
|----|------|----------|---------------|
| `stm32cubemx` | INTERFACE | 头文件路径 + 宏定义 | 需要被所有上层模块继承（HAL 头文件路径必须传递给 bsp/modules/apps） |
| `STM32_Drivers` | OBJECT | HAL 驱动 `.c` 源码 | 只需编译一次，产物注入 elf。`PUBLIC` 依赖 stm32cubemx，自动继承头文件路径 |

应用层源码（`main.c` 等）不建库，直接 `target_sources` 加入 `base` 可执行文件。


---

## CubeMX 重新生成的代码差异

| 文件 | 原代码 | CubeMX 重新生成后 | 适配方案 |
|------|--------|-------------------|----------|
| `app_azure_rtos.c` | `static TX_RT_POOL tx_app_rt_pool;` | 去掉 `static` | 保留不带 static 的版本，引用处加 `extern` |
| `main.c` | 存在 `MX_IWDG_Init();` | 被移除 | 手动添加回去 |
| `stm32f4xx_it.c` | 存在 `OTG_FS_IRQHandler` | 被移除 | 按 CherryUSB 规范处理 USB 中断 |
