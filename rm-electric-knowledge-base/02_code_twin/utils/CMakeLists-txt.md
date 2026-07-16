# CMakeLists.txt — utils 工具库

## 文件位置

`mas_embedded_threadx/utils/CMakeLists.txt`

## 作用

被 [[02_code_twin/board/dji_c/CMakeLists-txt]] 通过 `add_subdirectory(../../utils ...)` 加载。编译日志系统（SEGGER RTT + printf）和 FIFO 缓冲区为 OBJECT 库。

---

## 原始代码

```cmake
add_library(utils OBJECT
    utils_init.c
    ulog/SEGGER_RTT.c
    ulog/ulog.c
    ulog/printf.c
    kfifo/kfifo.c
)

target_compile_options(utils PRIVATE -O2)

target_include_directories(utils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/ulog
    ${CMAKE_CURRENT_SOURCE_DIR}/kfifo
)

target_link_libraries(utils PUBLIC stm32cubemx azrtos::threadx)
```

---

## 解析

这是项目中结构最简单的 CMakeLists.txt，也是**简单模式**的模板。后续 bsp、robot 的结构和它完全一致。

- `add_library(OBJECT)` → [[01_extracted/cmake/gcc-cmake-build#add_library(OBJECT) - .o 直接注入 elf]]（创建时直接内联源文件的写法）
- `源文件的两种写法` → [[01_extracted/cmake/gcc-cmake-build#源文件的两种写法]]
- `target_compile_options` → [[01_extracted/cmake/gcc-cmake-build#target_compile_options - 编译器选项]]
- `target_include_directories` → [[01_extracted/cmake/gcc-cmake-build#target_include_directories - 头文件搜索路径]]
- `target_link_libraries` → [[01_extracted/cmake/gcc-cmake-build#target_link_libraries - 链接哪些库]]
- `PRIVATE`/`PUBLIC`/`INTERFACE` 关键字 → [[01_extracted/cmake/gcc-cmake-build#五、作用域与 PRIVATE / PUBLIC / INTERFACE]]

### 关键字选择

- `target_compile_options` 用 `PRIVATE`：`-O2` 只在编译 utils 自己的 `.c` 时生效，上层不继承——每个模块自己决定优化等级。
- `target_include_directories` 用 `PUBLIC`：utils 自己需要（`#include "ulog.h"`），上层也需要（modules 里 `#include "ulog.h"`），所以路径要传上去。
- `target_link_libraries` 用 `PUBLIC`：utils 调用了 HAL 函数（`stm32cubemx`）和 ThreadX API（`azrtos::threadx`），上层链接 utils 后自动获得这些依赖。
