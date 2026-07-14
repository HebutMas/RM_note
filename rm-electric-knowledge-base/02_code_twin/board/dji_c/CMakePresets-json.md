# CMakePresets.json — CMake 预设构建配置

## 文件位置

`mas_embedded_threadx/board/dji_c/CMakePresets.json`

## 作用

被 [[02_code_twin/_vscode/tasks-json]] 中的 `--preset Debug` 读取。把构建类型（Debug/Release）、生成器（Ninja）、工具链文件路径打包成"预设"，避免每次在命令行手敲一长串参数。

---

## 原始代码（摘取）

```json
{
    "version": 3,
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
            "cacheVariables": { "CMAKE_BUILD_TYPE": "Debug" }
        },
        {
            "name": "Release",
            "inherits": "default",
            "cacheVariables": { "CMAKE_BUILD_TYPE": "Release" }
        }
    ],
    "buildPresets": [
        { "name": "Debug", "configurePreset": "Debug" },
        { "name": "Release", "configurePreset": "Release" }
    ]
}
```

---

## 逐段解析

### `${sourceDir}` 变量

CMakePresets 的内置变量，指向 [[02_code_twin/board/dji_c/CMakeLists-txt]] 所在目录（即 `board/dji_c/`）。而 [[02_code_twin/_vscode/tasks-json]] 中的 `${workspaceFolder}` 是 VS Code 的内置变量，指向项目根目录。两者层级不同：

| 变量 | 来源 | 值 |
|------|------|-----|
| `${sourceDir}` | CMakePresets 内置 | `board/dji_c/` |
| `${workspaceFolder}` | VS Code 内置 | `mas_embedded_threadx/` |


### default 预设（hidden，被继承不直接使用）

| 字段              | 值                                            | 含义                                                                                                                                                                                                                                                               |
| --------------- | -------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `hidden: true`  | —                                            | 不出现在用户可选列表中，仅作为基类被继承                                                                                                                                                                                                                                             |
| `generator`     | `Ninja`                                      | 等价于命令行 `-G Ninja`                                                                                                                                                                                                                                                |
| `binaryDir`     | `${sourceDir}/build/${presetName}`           | 构建输出目录。**但实际不生效**——[[02_code_twin/_vscode/tasks-json]] 中 `-B ${workspaceFolder}/build/dji_c/Debug` 覆盖了这个值。命令行 `-B` 的优先级高于 Preset 中的 `binaryDir`。Preset 里的 `binaryDir` 只有在不传 `-B` 时才生效，展开后是 `board/dji_c/build/Debug`（在源码目录下），而实际用的是 `build/dji_c/Debug`（在项目根目录下） |
| `toolchainFile` | `${sourceDir}/cmake/gcc-arm-none-eabi.cmake` | 指向 [[02_code_twin/board/dji_c/cmake/gcc-arm-none-eabi-cmake]]，CMake 在 `project()` 之前加载它                                                                                                                                                                          |

这就是为什么 Preset 中 `binaryDir` 展开为 `board/dji_c/build/Debug`，而 tasks.json 的 `-B` 展开为 `mas_embedded_threadx/build/dji_c/Debug`——`-B` 优先级更高，Preset 的 `binaryDir` 被忽略。
### Debug / Release 预设

继承 `default`，各自只覆盖一个变量 `CMAKE_BUILD_TYPE`。这是 CMake 的内置变量，CMake 底层根据它的值自动选择对应的编译标志：

| 预设        | `CMAKE_BUILD_TYPE` | CMake 底层行为                                     |
| --------- | ------------------ | ---------------------------------------------- |
| `Debug`   | `Debug`            | 自动选用 `CMAKE_C_FLAGS_DEBUG`，最终编译器拿到 `-O0 -g3`   |
| `Release` | `Release`          | 自动选用 `CMAKE_C_FLAGS_RELEASE`，最终编译器拿到 `-Os -g0` |

> 这些 FLAGS 变量在 [[02_code_twin/board/dji_c/cmake/gcc-arm-none-eabi-cmake]] 中定义，但 `CMAKE_BUILD_TYPE` 本身不在这个文件里被显式引用。

### buildPresets

```json
"buildPresets": [
    { "name": "Debug", "configurePreset": "Debug" }
]
```

绑定构建预设到配置预设。实际项目中 [[02_code_twin/_vscode/tasks-json]] 没有使用 `--build-preset`，而是手动传 `--config Debug`，效果等价。

