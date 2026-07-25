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

- `if` → [[01_extracted/cmake/cmake-basic-syntax#if - 条件判断]]
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

- `set` → [[01_extracted/cmake/cmake-basic-syntax#set - 设置变量]]

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

- `set` → [[01_extracted/cmake/cmake-basic-syntax#set - 设置变量]]

| 标志 | 含义 |
|------|------|
| `-ffunction-sections` | 每个函数放入独立段，配合 `--gc-sections` 移除未使用函数 |
| `-fdata-sections` | 每个数据变量放入独立段 |
| `-fstack-usage` | 生成栈使用报告（`.su` 文件），用于分析线程栈大小 |

这些标志通过 `set(CMAKE_C_FLAGS ...)` 设置全局编译器选项，和 `target_compile_options` 的区别详见 [[01_extracted/cmake/gcc-cmake-build#target_compile_options - 编译器选项]]。

### 链接器标志

```cmake
set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${MCU_LINKER_SCRIPT}\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
```

- `set` → [[01_extracted/cmake/cmake-basic-syntax#set - 设置变量]]
- `target_link_options` → [[01_extracted/cmake/gcc-cmake-build#target_link_options - 链接器选项]]

| 标志                             | 含义                                             |
| ------------------------------ | ---------------------------------------------- |
| `-T "${MCU_LINKER_SCRIPT}"`    | 指定链接器脚本（由板级提供）                                 |
| `--specs=nano.specs`           | 使用 newlib-nano 精简 C 库                          |
| `-Wl,--gc-sections`            | 链接时移除未引用的段，配合 `-ffunction-sections` 使用         |
| `-Wl,--print-memory-usage`     | 链接后打印 Flash/RAM 使用量                            |
| `TOOLCHAIN_LINK_LIBRARIES = m` | 链接数学库 `libm`，在 stm32cubemx CMakeLists.txt 中被引用 |

### C++ 标志

```cmake
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")
```

- `set` → [[01_extracted/cmake/cmake-basic-syntax#set - 设置变量]]

项目里全是 `.c` 文件，没有 C++ 代码。但 CMake 仍然会初始化 C++ 编译器（`project()` 时自动检测），如果不设这些，万一有人误加了 `.cpp` 文件，编译器会默认开启嵌入式用不到的机制：

| 标志 | 含义 | 为什么关掉 |
|------|------|----------|
| `-fno-rtti` | 关闭运行时类型识别（`dynamic_cast`/`typeid`） | 嵌入式没有动态类型需求，RTTI 表会额外占用 flash |
| `-fno-exceptions` | 关闭异常处理（`try`/`catch`/`throw`） | 裸机没有异常展开机制，异常支持代码会显著增大固件 |
| `-fno-threadsafe-statics` | 关闭局部静态变量的线程安全初始化 | 省去锁的开销，不影响 C 代码 |

这些标志对 C 代码无影响，属于兜底设置。`${CMAKE_C_FLAGS}` 追加语法将 C 的编译选项也继承给 C++，保证两套编译器使用相同的架构标志和优化等级。
