# settings.json — VS Code 编辑器配置

## 文件位置

`mas_embedded_threadx/.vscode/settings.json`

## 作用

配置 VS Code 工作区级别的设置，主要分三块：CMake 行为控制、ARM 工具链路径、clangd 代码分析引擎。

---

## 原始代码（摘取）

```json
{
    "cmake.configureOnOpen": false,
    "cmake.automaticReconfigure": false,
    "cmake.configureOnEdit": false,
    "cortex-debug.armToolchainPath": "C:\\msys64\\mingw64\\bin",
    "clangd.detectExtensionConflicts": true,
    "clangd.arguments": [
        "--header-insertion=never",
        "--background-index",
        "--clang-tidy",
        "-j=4",
        "--pch-storage=memory",
        "--function-arg-placeholders=false",
        "--compile-commands-dir=build"
        //这里之前还有一行 注释要求填写ARM 工具链路径的,但是实测删除后问题不大,clangd本质上用的系统的环境变量考上的armgnu工具链
    ]
}
```

---

## 逐段解析

### CMake 配置（禁用自动行为）

```json
"cmake.configureOnOpen": false,
"cmake.automaticReconfigure": false,
"cmake.configureOnEdit": false,
```

三个 `false` 禁用了 CMake Tools 插件的所有自动触发。项目使用 [[02_code_twin/_vscode/tasks-json]] 手动控制构建流程，CMake Tools 的自动行为会与之冲突。

### 调试器路径

```json
"cortex-debug.armToolchainPath": "C:\\msys64\\mingw64\\bin",
```

告诉 Cortex-Debug 插件去哪找 `arm-none-eabi-gdb`，指向 MSYS2 的 mingw64/bin。机器相关，换电脑需要改。

### clangd 配置

```json
"clangd.arguments": [
    "--header-insertion=never",
    "--background-index",
    "--clang-tidy",
    "-j=4",
    "--pch-storage=memory",
    "--function-arg-placeholders=false",
    "--compile-commands-dir=build"
]
```

| 参数                                  | 含义                                                      |
| ----------------------------------- | ------------------------------------------------------- |
| `--header-insertion=never`          | 禁止自动补全 `#include`（避免引入不需要的头文件）                          |
| `--background-index`                | 后台索引整个项目，建立符号数据库                                        |
| `--clang-tidy`                      | 启用 Clang-Tidy 静态检查（潜在 bug、代码风格）                         |
| `-j=4`                              | 同时开 4 个分析线程                                             |
| `--pch-storage=memory`              | 预编译头存内存（提升性能，增加内存开销）                                    |
| `--function-arg-placeholders=false` | 补全函数时不生成参数占位符                                           |
| `--compile-commands-dir=build`      | **关键**：告诉 clangd 去 `build/` 目录找 `compile_commands.json` |
|                                     |                                                         |

### `--compile-commands-dir=build` 的完整链路

`compile_commands.json` 从产生到被 clangd 读取，经过了三个文件、两次转手：

**第一步：生成（CMakeLists.txt）**

[[02_code_twin/board/dji_c/CMakeLists-txt]] 中设置了 `CMAKE_EXPORT_COMPILE_COMMANDS TRUE`。CMake 配置阶段（阶段一）执行时，Ninja 生成器会在 `-B` 指定的构建目录下生成 `compile_commands.json`，即 `build/dji_c/Debug/compile_commands.json`。这个文件记录了每个 `.c` 文件的真实编译命令（编译器路径、头文件搜索路径、宏定义等）。

**第二步：搬运（tasks.json）**

[[02_code_twin/_vscode/tasks-json]] 阶段五用 `Copy-Item` 把 `build/dji_c/Debug/compile_commands.json` 强制复制到 `build/compile_commands.json`。之所以需要这一步，是因为 clangd 的 `--compile-commands-dir=build` 只会在 `build/` 目录下找这个文件，而它实际生成在更深的 `build/dji_c/Debug/` 里。

**第三步：读取（settings.json，即本文件）**

clangd 启动时根据 `--compile-commands-dir=build` 找到 `build/compile_commands.json`，解析其中每个 `.c` 文件的编译参数，据此建立符号索引——代码跳转、补全、诊断全部依赖这个索引。

```
[CMakeLists.txt]  CMAKE_EXPORT_COMPILE_COMMANDS TRUE
      ↓ 配置阶段生成
build/dji_c/Debug/compile_commands.json
      ↓ tasks.json 阶段五 Copy-Item 搬运
build/compile_commands.json
      ↓ settings.json 中 clangd --compile-commands-dir=build 读取
clangd 代码跳转 / 补全 / 诊断
```

如果 `compile_commands.json` 不存在或过时（比如修改了 CMakeLists.txt 但没重新构建），clangd 的跳转和补全会失效。
