# CMakeLists.txt — modules 功能模块层

## 文件位置

`mas_embedded_threadx/modules/CMakeLists.txt`

## 作用

被 [[02_code_twin/board/dji_c/CMakeLists-txt]] 通过 `add_subdirectory(../../modules ...)` 加载。根据 `MODULE_XXX` 变量条件编译各功能模块（电机控制、IMU、裁判系统等）。

---

## 原始代码（摘取）

### 基础源文件（始终编译）

```cmake
set(_module_sources
    module_init.c
    algorithm/pid.c
    algorithm/user_lib.c
    algorithm/crc_rm.c
    algorithm/lqr.c
    algorithm/chassis_type.c
)
```

所有机器人都需要的公共代码：模块初始化入口、PID 控制器、CRC 校验、LQR 控制器、底盘类型判断。

### 条件累加

```cmake
if(MODULE_BMI088)
    list(APPEND _module_sources
        BMI088/module_bmi088.c
        BMI088/module_bmi088_cali.c
    )
    list(APPEND _module_includes ${CMAKE_CURRENT_SOURCE_DIR}/BMI088)
endif()

if(MODULE_MOTOR)
    list(APPEND _module_sources
        MOTOR/module_motor.c
        MOTOR/motor_base.c
        MOTOR/DJI/motor_dji.c
        MOTOR/POWER_CONTROL/power_control.c
        MOTOR/DAMIAO/motor_damiao.c
        MOTOR/SERVO/motor_servo.c
        MOTOR/ZDT_STEPMOTOR/motor_zdt.c
    )
    list(APPEND _module_includes
        ${CMAKE_CURRENT_SOURCE_DIR}/MOTOR
        ...
    )
endif()

if(MODULE_REMOTE)
    list(APPEND _module_sources
        REMOTE/module_remote.c
        REMOTE/module_remote_vt.c
    )
    ...
endif()
```

每个 `if(MODULE_XXX)` 块做两件事：往 `_module_sources` 追加源文件，往 `_module_includes` 追加头文件路径。`MODULE_XXX` 的值来自 [[02_code_twin/apps/config-cmake#模块开关]]

### 生成头文件目录定位

```cmake
get_filename_component(_board_binary_dir ${CMAKE_CURRENT_BINARY_DIR} DIRECTORY)
set(_generated_dir ${_board_binary_dir}/generated)
```

`CMAKE_CURRENT_BINARY_DIR` 是 `build/dji_c/Debug/modules/`，`DIRECTORY` 取上一级得到 `build/dji_c/Debug/`，拼 `/generated` 就是 `module_config.h` 所在目录。

### 建库 + 强制注入配置头文件

```cmake
add_library(modules OBJECT ${_module_sources})

target_compile_options(modules PRIVATE -O2 -include module_config.h)

target_include_directories(modules PUBLIC ${_module_includes})

target_link_libraries(modules PUBLIC stm32cubemx azrtos::threadx m CMSISDSP bsp utils cherryusb)
```


**特别注意的**
- `module_config.h` 是由 [[02_code_twin/board/dji_c/CMakeLists-txt#配置系统加载]] 中 `include(generate_headers.cmake)` 调用 [[02_code_twin/apps/generate_headers-cmake]] 把 CMake 变量自动转成宏定义生成的 `.h` 文件

链接回 01 层：
- `add_library(OBJECT)` → [[01_extracted/cmake/gcc-cmake-build#add_library(OBJECT) - .o 直接注入 elf]]（变量传入的混合写法）
- `target_compile_options` → [[01_extracted/cmake/gcc-cmake-build#target_compile_options - 编译器选项]]
- `target_include_directories` → [[01_extracted/cmake/gcc-cmake-build#target_include_directories - 头文件搜索路径]]
- `target_link_libraries` → [[01_extracted/cmake/gcc-cmake-build#target_link_libraries - 链接哪些库]]
- `PRIVATE`/`PUBLIC` 关键字 → [[01_extracted/cmake/gcc-cmake-build#五、作用域与 PRIVATE / PUBLIC / INTERFACE]]

`-include module_config.h` 等价于在每个 `.c` 文件第一行自动加 `#include "module_config.h"`。这样源代码里的 `#if MODULE_BMI088` 守卫才能正确判断。

链接了 `m`（数学库 `sin/cos`）、`CMSISDSP`（ARM DSP）、`bsp`、`utils`、`cherryusb`。全部 `PUBLIC` 向上传递给 app。
