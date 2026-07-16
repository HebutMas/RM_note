# tasks.json — VS Code 构建任务配置

## 文件位置

`mas_embedded_threadx/.vscode/tasks.json`

## 执行流程

```
Ctrl+Shift+P → Run Task
    ↓
选择构建任务：Build dji_c / Build damiao_h7
    ↓
执行对应平台的一行 Shell 长命令（Windows 用 PowerShell，Linux 用 bash）
```

本文以 `Build dji_c`（Windows / PowerShell）为例，拆解这一行长命令。

---

## 原始代码（摘取）

### inputs：用户交互输入

```json
"inputs": [
    {
        "id": "selectBoard",
        "type": "pickString",
        "options": ["damiao_h7", "dji_c"],
        "default": "dji_c"
    },
    {
        "id": "selectProbe",
        "type": "pickString",
        "options": ["stlink", "daplink", "jlink"],
        "default": "jlink"
    }
]
```

两个下拉选择器，仅在 `Flash board` 任务中使用。运行烧录任务时弹出选择框，选定板型号和调试器后传给 `flash_interactive.bat`。

### Build dji_c 的 PowerShell 命令

```powershell
cmake -S ${workspaceFolder}/board/dji_c -B ${workspaceFolder}/build/dji_c/Debug --preset Debug -G Ninja;
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE };
cmake --build ${workspaceFolder}/build/dji_c/Debug --config Debug -j ${env:NUMBER_OF_PROCESSORS};
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE };
Copy-Item -Path ${workspaceFolder}/build/dji_c/Debug/compile_commands.json -Destination ${workspaceFolder}/build/compile_commands.json -Force;
Copy-Item -Path ${workspaceFolder}/build/dji_c/Debug/base.elf -Destination ${workspaceFolder}/build/dji_c.elf -Force
```

被分号 `;` 隔成 5 个阶段。下面逐阶段拆解。

---

## 五阶段流水线

### 阶段一：配置（Configure）

```powershell
cmake -S ${workspaceFolder}/board/dji_c -B ${workspaceFolder}/build/dji_c/Debug --preset Debug -G Ninja
```

**参数**：

| 参数               | 含义                                                                |
| ---------------- | ----------------------------------------------------------------- |
| `-S`             | Source，源码根目录（[[02_code_twin/board/dji_c/CMakeLists-txt]] 所在位置）    |
| `-B`             | Build，构建产物输出目录（不存在则自动创建）                                          |
| `--preset Debug` | 加载 [[02_code_twin/board/dji_c/CMakePresets-json]] 中名为 `Debug` 的预设 |
| `-G Ninja`       | 指定生成器为 Ninja，生成 `build.ninja` 而非 `Makefile`                       |

**这一步在硬盘上产生了什么**：

```
build/dji_c/Debug/
├── build.ninja        ← 设计图纸（Ninja 读取此文件决定怎么编译）
├── CMakeCache.txt     ← 变量记账本（记录编译器路径、预设参数等，下次配置时复用）
└── CMakeFiles/        ← 依赖扫描中间文件
```

> `build.ninja` 和 `CMakeCache.txt` 是配置阶段的产物，构建阶段读取它们。`CMakeCache.txt` 缓存了编译器绝对路径——这就是为什么修改 `PATH` 后不删 `build` 目录就不会生效。

### 阶段二：配置检查

```powershell
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
```

`$LASTEXITCODE` 是 PowerShell 变量，保存上一条命令的退出码。配置失败则立刻终止，不执行构建。

**为什么必须立刻检查**：`$LASTEXITCODE` 只保留"上一条"命令的返回值，执行下一条命令后会被覆盖。

### 阶段三：构建（Build）

```powershell
cmake --build ${workspaceFolder}/build/dji_c/Debug --config Debug -j ${env:NUMBER_OF_PROCESSORS}
```

**参数**：

| 参数                               | 含义                                                                                                                 |
| -------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| `--build`                        | 触发 CMake 构建模式，后面的路径是 `build.ninja` 所在位置（必须和阶段一的 `-B` 一致）                                                           |
| `--config Debug`                 | 告诉 Ninja 按图纸中的 Debug 页施工（`-O0 -g3`，不优化、带调试信息。优化级别在 [[02_code_twin/board/dji_c/cmake/gcc-arm-none-eabi-cmake]] 中定义） |
| `-j ${env:NUMBER_OF_PROCESSORS}` | 并行编译线程数，运行时替换为 CPU 核心数                                                                                             |

**这一步在硬盘上产生了什么**：

```
build/dji_c/Debug/
├── *.o                ← 编译产物（.c → .o）
├── base.elf           ← 链接产物（.o → .elf），最终固件
├── base.map           ← 内存映射文件
└── compile_commands.json ← 编译数据库（记录每个 .c 的编译命令）
```

> Ninja 读取阶段一生成的 `build.ninja`，比对文件时间戳决定哪些需要重编，然后调用 GCC 并行编译 `.c → .o`，最后链接成 `base.elf`。

**为什么 `-B` 和 `--build` 后面的路径必须一样**：`-B` 是设计师放图纸的仓库地址，`--build` 是施工队长取图纸的地址。写不一样 Ninja 就扑空。

### 阶段四：构建检查

```powershell
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
```

和阶段二同理。构建（编译/链接）失败则不执行后续的文件复制。

### 阶段五：后处理（Post-Process）

```powershell
Copy-Item -Path .../build/dji_c/Debug/compile_commands.json -Destination .../build/compile_commands.json -Force;
Copy-Item -Path .../build/dji_c/Debug/base.elf -Destination .../build/dji_c.elf -Force
```

把两个关键产物从深层目录复制到 `build/` 根目录：

| 源文件                                       | 目标                            | 之后被谁使用                                                                                          |
| ----------------------------------------- | ----------------------------- | ----------------------------------------------------------------------------------------------- |
| `build/dji_c/Debug/compile_commands.json` | `build/compile_commands.json` | [[02_code_twin/_vscode/settings-json]] 中 clangd 配置 `--compile-commands-dir=build`，读取此文件做代码跳转/补全 |
| `build/dji_c/Debug/base.elf`              | `build/dji_c.elf`             | 外部烧录工具（如 Ozone）加载此固件                                                                            |

---

## `-` vs `--` 参数命名规则

| 格式 | 名称 | 示例 | 说明 |
|------|------|------|------|
| `-X` | 短选项 | `-S` `-B` `-G` `-j` | 单字母，配置模式下的选项 |
| `--xxx` | 长选项 | `--build` `--preset` `--config` | 完整单词 |

`--build` 不是一个"动作"，而是一个长选项，它让 CMake 从**配置模式**切换到**构建模式**。

---

## 产物流向图

```
[阶段一] cmake -S ... -B build/dji_c/Debug --preset Debug -G Ninja
         ↓ 生成 build.ninja + CMakeCache.txt

[阶段三] cmake --build build/dji_c/Debug --config Debug -j N
         ↓ Ninja + GCC 编译链接
         ├── compile_commands.json  (在 build/dji_c/Debug/)
         └── base.elf               (在 build/dji_c/Debug/)

[阶段五] Copy-Item
         ├── compile_commands.json  → build/compile_commands.json  → clangd
         └── base.elf               → build/dji_c.elf              → 调试器/烧录
```

---

## Flash board 任务

```json
"args": ["/c", "${workspaceFolder}/.vscode/flash_interactive.bat ${input:selectBoard} ${input:selectProbe}"]
```

调用烧录脚本，传入用户选择的板型号和调试器（来自 inputs 配置）。`presentation.close: true` 表示烧录完成后自动关闭终端。
