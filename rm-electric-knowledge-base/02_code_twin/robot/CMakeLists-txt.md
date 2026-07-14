# CMakeLists.txt — robot 机器人初始化层

## 文件位置

`mas_embedded_threadx/robot/CMakeLists.txt`

## 作用

被 [[02_code_twin/board/dji_c/CMakeLists-txt]] 通过 `add_subdirectory(../../robot ...)` 加载。只有一个源文件 `robot_init.c`，是整个机器人的初始化入口——调用各模块的 init 函数，创建所有线程，启动调度器。

---

## 原始代码

```cmake
add_library(robot OBJECT
    robot_init.c
)

target_compile_options(robot PRIVATE -O2)

target_include_directories(robot PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(robot PUBLIC stm32cubemx azrtos::threadx utils bsp CMSISDSP modules app)
```

---

## 与 utils 的差异

结构和 [[02_code_twin/utils/CMakeLists-txt]] 完全一致，是项目中最简单的 CMakeLists.txt——单文件 OBJECT 库。

差异点：`target_link_libraries` 链接了几乎所有模块（`app` 和 `modules` 等），因为 `robot_init.c` 要调用各模块的 init 函数。全部 `PUBLIC` 向上传递给 `base`。
