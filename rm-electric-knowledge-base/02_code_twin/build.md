# build/ — 构建产物目录

## 位置

`mas_embedded_threadx/build/`

## 说明

这是构建系统的"工地"，所有配置阶段和编译阶段的产物都在这里。由 [[02_code_twin/_vscode/tasks-json]] 的 `-B` 参数指定。目录可随时删除，重新构建会自动生成。

---

## 目录结构

```
build/
├── compile_commands.json   ← 被阶段五搬运到此，供 clangd 读取
├── dji_c.elf               ← 被阶段五搬运到此，供调试器/烧录加载
└── dji_c/
    └── Debug/
        ├── build.ninja              ← Ninja 构建脚本（配置阶段生成）
        ├── CMakeCache.txt           ← CMake 缓存（编译器路径、预设参数等）
        ├── compile_commands.json    ← 编译数据库（原始位置，被搬运到上层）
        ├── base.elf                 ← 最终固件（被搬运到上层并改名 dji_c.elf）
        ├── base.map                 ← 内存映射文件
        ├── libcherryusb.a           ← CherryUSB 静态库
        ├── libthreadx.a             ← ThreadX 静态库
        ├── *.ltrans.su              ← LTO 栈使用报告
        ├── CMakeFiles/              ← CMake 内部缓存（依赖扫描、规则文件等）
        ├── generated/               ← CMake 自动生成的 C 头文件
        │   ├── robot_def.h          ←   机器人类型宏
        │   └── module_config.h      ←   模块开关宏
        ├── apps/                    ← 各子模块的构建中间产物
        ├── bsp/
        ├── modules/
        ├── robot/
        ├── utils/
        ├── threadx/
        ├── cmsisdsp/
        └── cmake/
```

## 关键文件

| 文件 | 产生阶段 | 用途 |
|------|----------|------|
| `build.ninja` | 阶段一（配置） | Ninja 读取此文件决定怎么编译、链接 |
| `CMakeCache.txt` | 阶段一（配置） | 缓存编译器绝对路径，修改 PATH 后需删 build 目录才能刷新 |
| `compile_commands.json` | 阶段三（构建） | 记录每个 `.c` 的编译命令，被搬运到 `build/` 供 clangd 读取 |
| `base.elf` | 阶段三（构建） | 最终固件，被搬运到 `build/` 并改名 `dji_c.elf` |
| `base.map` | 阶段三（构建） | 内存映射，查看每个符号在 Flash/RAM 中的位置和大小 |
| `generated/module_config.h` | 阶段一（配置） | CMake 变量翻译成 C 宏，通过 `-include` 强制注入每个源文件 |
| `generated/robot_def.h` | 阶段一（配置） | 机器人类型枚举宏 |

> `libcherryusb.a`、`libthreadx.a` 是 STATIC 库的产物。其他模块（apps/bsp/modules 等）使用 OBJECT 库，不生成 `.a`，编译产物直接注入 `base.elf`。子目录（apps/、modules/ 等）里主要是 `CMakeFiles/` 和 `cmake_install.cmake`，都是 CMake 内部管理文件，无需关注。

## 上层与下层

```
build/                           ← clangd 和调试器访问的层级
├── compile_commands.json        ← tasks.json 阶段五从下层搬上来
├── dji_c.elf                    ← tasks.json 阶段五从下层搬上来并改名
└── dji_c/Debug/                 ← 实际构建发生的层级
    ├── build.ninja              ← CMake 配置阶段生成
    ├── base.elf                 ← Ninja 构建阶段生成
    ├── compile_commands.json    ← CMake 配置阶段生成
    └── ...
```

[[02_code_twin/_vscode/tasks-json]] 阶段五的 `Copy-Item` 把深层产物"摘"到上层，是为了让 [[02_code_twin/_vscode/settings-json]] 中的 clangd（`--compile-commands-dir=build`）能找到 `compile_commands.json`，外部烧录工具能找到 `dji_c.elf`。
