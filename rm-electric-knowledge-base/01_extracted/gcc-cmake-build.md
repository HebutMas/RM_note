# GCC 与 CMake 构建流程

> 从 `.c` 到 `.elf` 的完整流程，以及 CMake 如何通过 target 命令控制这个流程。
> 前置知识：[[01_extracted/cmake-basic-syntax]]

---

## 一、GCC 编译四步流程

GCC 是一个翻译器，把人写的 `.c` 翻译成芯片能执行的二进制，分四步：

```
main.c →[预处理器]→ main.i →[编译器]→ main.s →[汇编器]→ main.o →[链接器]→ base.elf
         展开 #include        生成汇编    生成机器码     把多个 .o 合并
         展开 -D 宏
```

### 第一步：预处理器

预处理器做两件事：展开 `#include`（复制粘贴）和展开 `-D` 宏（文本替换）。

```c
// main.c
#include "config.h"       // ← config.h 的内容被粘贴到这里
init_motor(MOTOR_3508);   // ← MOTOR_3508 被替换成 1

// 预处理后变成 main.i：
void init_motor(int id);  // ← 来自 config.h
init_motor(1);            // ← MOTOR_3508 替换成 1
```

这一步需要知道：头文件在哪（`-I`）、要替换哪些宏（`-D`）。

### 第二步：编译器

把 `.i` 翻译成汇编 `.s`，受编译选项影响（`-O2` 优化、`-include` 强制包含等）。

### 第三步：汇编器

把 `.s` 翻译成机器码 `.o`。但 `.o` 里有"空位"——调用外部函数时不知道地址，等链接器来填。

### 第四步：链接器

把所有 `.o` 合并，填上空位。如果函数在 `.a` 库里，从库中抽出对应的 `.o` 合并进来。受链接选项影响（`-flto` 链接时优化、`-T` 链接器脚本等）。

### CMake 与 GCC 的桥梁

CMake 的每个 `target_*` 命令，最终就是在往 GCC 命令行上塞参数。以编译 `ulog.c` 为例，CMake 生成的实际命令类似：

```
arm-none-eabi-gcc -c -O2 -I utils/ulog -DSTM32F407xx ulog.c -o ulog.o
                 ↑     ↑    ↑              ↑            ↑
             -c 编译  来自  来自            来自         产物
             不链接  target_  target_compile_  target_compile_
                    include_  definitions     options
                    directories
```

后续每个 `target_*` 命令的讲解中，不再重复翻译 GCC 命令行，只需记住：CMake 命令就是在控制这几步的参数。

---

## 二、头文件、源文件、库的区别

| 概念 | 后缀 | 作用阶段 | 内容 |
|------|------|----------|------|
| 头文件 | `.h` | 预处理阶段 | 声明（函数原型、宏定义、类型定义） |
| 源文件 | `.c` | 编译阶段 | 实现（函数的具体代码） |
| 目标文件 | `.o` | 汇编阶段后 | 机器码，但地址未确定 |
| 静态库 | `.a` | 链接阶段 | 多个 `.o` 打包成的归档文件 |
| 可执行文件 | `.elf` | 链接阶段后 | 完整的、可烧录到芯片的二进制 |

用一个例子说明三者关系：

```c
// motor.h — 头文件（声明）
void init_motor(int id);
```

```c
// motor.c — 源文件（实现）
void init_motor(int id) { set_pwm(id, 1000); }
```

```
编译 motor.c → motor.o → 打包成 libmotor.a
```

```c
// main.c — 使用者
#include "motor.h"     // 编译时：需要 motor.h 知道函数参数类型
init_motor(1);         // 链接时：需要 libmotor.a 里的机器码
```

- **头文件**是"合同"：告诉编译器函数长什么样，编译器据此检查你调用得对不对
- **库**是"交付物"：提供函数的机器码，链接器据此填充调用地址
- **源文件**是"图纸"：既包含声明（通过 `.h` 暴露出去），也包含实现（编译成 `.o`/`.a`）

只给头文件不给库：代码能编译通过，但链接报错 `undefined reference to 'init_motor'`。
只给库不给头文件：你不知道函数参数类型，没法正确调用。

CMake 中的对应关系：`target_sources` 管源文件、`target_include_directories` 管头文件、`target_link_libraries` 管库。

---

## 三、project - 声明项目名称

告诉 CMake "这个项目叫 base，要编译 C 和汇编"。`project()` 是 CMake 的分水岭——之前只能设置变量（`set`）、加载配置脚本（`include`），工具链文件也是在这之前被加载的，CMake 遇到 `project()` 时才触发编译器检测。`project()` 之后才能创建 target（`add_executable` / `add_library`），才能用 `target_*` 命令。

```cmake
project(<name> [VERSION <major.minor.patch.tweak>] [LANGUAGES <lang1> ...])
```

| 参数          | 填什么    | 什么意思                                              |
| ----------- | ------ | ------------------------------------------------- |
| `<name>`    | `base` | 项目名称，决定最终产物叫 `base.elf`                           |
| `VERSION`   | 省略     | 嵌入式项目不需要版本号管理，不写                                  |
| `LANGUAGES` | 省略     | 可以在这里写 `LANGUAGES C ASM`，也可以拆开用 `enable_language` |

```cmake
project(${CMAKE_PROJECT_NAME})
enable_language(C ASM)
```

- `${CMAKE_PROJECT_NAME}` 是前面 `set` 的 `base`
- `enable_language(C ASM)` 紧跟 project，告诉 CMake 要编译 C 和汇编两种语言——启动文件 `startup_stm32f407xx.s` 是汇编写的

`enable_language` 和 `project(... LANGUAGES ...)` 效果一样，`project(name LANGUAGES C ASM)` 等价于 `project(name)` + `enable_language(C ASM)`。项目拆开写是因为 `CMAKE_PROJECT_NAME` 需要在 `project()` 之前用 `set` 设好。

---

## 四、创建目标

`add_library` 和 `add_executable` 创建一个**命名的编译目标**。创建后才能用 `target_*` 命令往上面添加属性——没有目标，target_* 就是无的放矢。

### add_executable - 创建可执行文件目标

创建一个最终要变成 `.elf` 的目标。一个项目只有一个 `add_executable`，因为最终只烧录一个固件。

```cmake
add_executable(<name> [source1 source2 ...])
```

| 参数 | 填什么 | 什么意思 |
|------|--------|----------|
| `<name>` | `${CMAKE_PROJECT_NAME}` | 目标名，即 `base` |
| `source...` | 可选 | 源文件，创建时可以不写，后续用 `target_sources` 补充 |

```cmake
add_executable(${CMAKE_PROJECT_NAME})
```

这里创建时不写源文件，是因为板级 CMakeLists.txt 此时尚未进入任何子目录，源文件分散在各子模块中，后续通过 `add_subdirectory` 进入子目录后由各模块的 `target_sources` 逐步补充。

### add_library(OBJECT) - .o 直接注入 elf

把几个 `.c` 文件编成一个组，编译成 `.o`，不打包成 `.a`，等最终链接 `base.elf` 时直接塞进去。适合项目内部模块——你写了的代码肯定都要用，没必要打包再抽取。

```cmake
add_library(<name> OBJECT <source1> <source2> ...)
```

| 参数          | 填什么                       | 什么意思                                                                        |
| ----------- | ------------------------- | --------------------------------------------------------------------------- |
| `<name>`    | `utils` / `STM32_Drivers` | 目标名，后续 target_* 全用它引用                                                       |
| `OBJECT`    | 固定关键字                     | 编译完不打包成 `.a`，`.o` 散着放                                                       |
| `source...` | `.c` / `.s` 文件            | 不管什么类型，输入永远是源文件不是 `.o`，OBJECT 只控制输出怎么处理。可以在创建时写，也可以不写后续用 `target_sources` 补 |

```cmake
add_library(STM32_Drivers OBJECT)
```

这里创建时不写源文件，源文件列表 `${STM32_Drivers_Src}` 在上面用 `set` 准备好了，下一行用 `target_sources` 补充。

### add_library(STATIC) - 打包成 .a 归档

把源文件编译后打包成 `.a` 归档文件。链接时链接器从归档中按需抽取被引用的 `.o`。适合第三方库——你只用了库的一部分功能，不需要把整个库都塞进固件。

```cmake
add_library(<name> STATIC <source1> <source2> ...)
```

| 参数 | 填什么 | 什么意思 |
|------|--------|----------|
| `<name>` | `cherryusb` | 目标名 |
| `STATIC` | 固定关键字 | 编译后打包成 `.a` 归档 |
| `source...` | `.c` 文件或变量 | 可以是变量（如 `${cherryusb_srcs}`），也可以创建时不写后续补充 |

```cmake
add_library(cherryusb STATIC ${cherryusb_srcs})
```

`cherryusb_srcs` 是上面 `include(../../CherryUSB/cherryusb.cmake)` 执行后暴露的变量，直接传入省去了 `target_sources` 一步。

### add_library(INTERFACE) - 只传属性不编译

创建一个"空壳"目标，不编译任何源文件，只携带头文件路径和宏定义，传给上层。适合"只给头文件不给源码"的场景——CubeMX 生成的 HAL 驱动头文件路径需要被所有上层模块继承，但没有独立的源文件需要以 INTERFACE 形式编译。

```cmake
add_library(<name> INTERFACE)
```

| 参数 | 填什么 | 什么意思 |
|------|--------|----------|
| `<name>` | `stm32cubemx` | 目标名 |
| `INTERFACE` | 固定关键字 | 不编译任何东西，只携带属性传给上层 |

```cmake
add_library(stm32cubemx INTERFACE)
```

stm32cubemx 是 INTERFACE 库，只携带 HAL 头文件路径和宏定义。实际编译的 HAL 驱动源文件由同文件中的另一个 OBJECT 库 `STM32_Drivers` 负责。为什么拆成两个库：头文件路径需要被所有上层模块继承（INTERFACE 传递），而源文件只需编译一次（OBJECT 注入）。

### 源文件的两种写法

创建时直接内联（如 `add_library(utils OBJECT utils_init.c ...)`）和创建后用 `target_sources` 补充（如 `add_library(STM32_Drivers OBJECT)` + `target_sources(...)`）效果一样。选择哪种看代码风格和文件列表是否需要条件拼装。modules 的条件累加模式就是先 `set` 变量再传入 `add_library`，属于第三种混合写法。

### 对比

**add_executable vs add_library**：

| | add_executable | add_library |
|---|---|---|
| 产物 | `.elf` 可执行文件 | `.o` / `.a` / 无（INTERFACE） |
| 数量 | 一个项目只有一个 | 可以有多个 |
| 源文件 | 可以后续 target_sources 补 | 同左 |
| 被谁链接 | 不被链接，是最终产物 | 被 add_executable 或其他 add_library 链接 |

**OBJECT vs STATIC vs INTERFACE**：

| | OBJECT | STATIC | INTERFACE |
|---|---|---|---|
| 编译源文件 | 是 | 是 | 否 |
| 产物 | `.o`，散着放 | `.a` 归档 | 无 |
| 链接时 | `.o` 全部塞进 elf | 从 `.a` 中按需抽取 `.o` | 不参与链接，只传属性 |
| 适合 | 项目内部模块 | 第三方库 | 纯头文件/宏定义 |
| 本项目使用者 | utils, bsp, robot, modules, app, STM32_Drivers | threadx, cherryusb, CMSISDSP | stm32cubemx |

---

## 五、作用域与 PRIVATE / PUBLIC / INTERFACE

### 变量作用域

`add_subdirectory` 进入子目录时会新建一个子作用域——子目录里 `set` 的变量不会传回父目录，但子目录里 `add_library` 创建的 target 会注册到全局注册表，父目录可以用。`include` 不创建新作用域，共享当前作用域，变量直接生效。

### target 属性传递

在变量作用域的基础上，CMake 还有一套 target 属性传递机制。`target_*` 命令中的 PRIVATE / PUBLIC / INTERFACE 控制的是**属性是否向上层传递**。假设依赖链是 `base → app → bsp`：

| 关键字 | 自己编译时用？ | 传给上层？ | 含义 |
|--------|---|---|---|
| `PRIVATE` | 是 | 否 | 只自己用 |
| `PUBLIC` | 是 | 是 | 自己用，也传给上层 |
| `INTERFACE` | 否 | 是 | 只传给上层，自己不用 |

判断口诀：两个都 YES → PUBLIC；只有自己用 → PRIVATE；只传给别人 → INTERFACE。

后续每个 `target_*` 命令的实例中，会在逐行对应里解释为什么这里用哪个关键字。

---

## 六、target_* 命令

`target_*` 命令操作的对象，就是上面 `add_library` / `add_executable` 创建出来的 target。每个命令对应 GCC 流程的一个环节：

| GCC 步骤 | CMake 命令 | 作用 |
|----------|-----------|------|
| 输入 | `target_sources` | 哪些 .c 进入流水线 |
| 预处理 | `target_include_directories` | 头文件搜索路径 |
| 预处理 | `target_compile_definitions` | 预处理宏 |
| 编译 | `target_compile_options` | 编译器选项 |
| 链接 | `target_link_libraries` | 链接哪些库 |
| 链接 | `target_link_options` | 链接器选项 |

### target_sources - 指定编译哪些源文件

给已创建的目标补充源文件。和 `add_library` / `add_executable` 创建时内联源文件效果一样，只是写法不同。

```cmake
target_sources(<target> [PRIVATE|PUBLIC|INTERFACE] <source1> <source2> ...)
```

| 参数 | 填什么 | 什么意思 |
|------|--------|----------|
| `<target>` | `STM32_Drivers` | 给哪个目标补源文件 |
| 关键字 | `PRIVATE` | 这些源文件只有自己需要编译，不传给上层 |
| `source...` | `${STM32_Drivers_Src}` | 源文件列表，可以是变量 |

```cmake
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
```

PRIVATE 因为源文件只有 STM32_Drivers 自己需要编译，不需要传给上层。和 `add_library` 里写源文件效果完全一样——创建时文件列表已确定就直接内联，需要条件拼装或后续补充就用 `target_sources`。

### target_include_directories - 头文件搜索路径

告诉 GCC 编译器，编译这个目标的 `.c` 文件时，如果遇到 `#include "xxx.h"`，去哪些目录找。它管理的是 **GCC 编译阶段**的行为，和 CMake 自己去哪里找 `CMakeLists.txt`（`include` / `add_subdirectory`）完全是两码事——一个管编译器找头文件，一个管 CMake 找脚本。

```cmake
target_include_directories(<target> [PRIVATE|PUBLIC|INTERFACE] <dir1> <dir2> ...)
```

| 参数 | 填什么 | 什么意思 |
|------|--------|----------|
| `<target>` | `stm32cubemx` | 给哪个目标配路径 |
| 关键字 | `INTERFACE` | 路径只传给上层，自己不编译任何东西 |
| `dir...` | `${MX_Include_Dirs}` | 头文件所在目录，可以填一个或多个，空格分隔。`CMAKE_CURRENT_SOURCE_DIR` 是 CMake 内置变量，指向当前 CMakeLists.txt 所在目录 |

```cmake
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
```

INTERFACE 因为 stm32cubemx 是 INTERFACE 库，自己不编译任何源文件，头文件路径只用来传给上层（bsp / modules / apps 都需要找到 HAL 头文件）。如果不用这个命令，上层模块 `#include "main.h"` 就找不到文件，编译器报错 `fatal error: main.h: No such file or directory`。

### target_compile_definitions - 预处理宏

给编译器加 `-D` 宏，等价于在源文件第一行加 `#define`。

```cmake
target_compile_definitions(<target> [PRIVATE|PUBLIC|INTERFACE] <def1> <def2> ...)
```

| 参数 | 填什么 | 什么意思 |
|------|--------|----------|
| `<target>` | `stm32cubemx` | 给哪个目标配宏 |
| 关键字 | `INTERFACE` | 宏只传给上层 |
| `def...` | `USE_HAL_DRIVER` `STM32F407xx` | 宏名。`USE_HAL_DRIVER` 告诉 HAL 库使用 HAL 风格而非 LL 风格，`STM32F407xx` 指定芯片型号让 HAL 选对寄存器定义 |

```cmake
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})
```

INTERFACE 同上，宏定义只传给上层。CMake 推荐用 `target_compile_definitions` 而不是 `set(CMAKE_C_FLAGS ... -D)`，因为前者可以精确控制宏的传递范围。

### target_compile_options - 编译器选项

给编译器传编译选项，如优化等级 `-O2`、强制包含 `-include` 等。

```cmake
target_compile_options(<target> [PRIVATE|PUBLIC|INTERFACE] <option1> <option2> ...)
```

| 参数 | 填什么 | 什么意思 |
|------|--------|----------|
| `<target>` | `utils` | 给哪个目标配编译选项 |
| 关键字 | `PRIVATE` | 选项只自己用，上层不继承 |
| `option...` | `-O2` | 优化等级。`-include module_config.h` 是特殊选项，等价于在每个 `.c` 文件第一行自动加 `#include "module_config.h"`，这样源代码里的 `#if MODULE_BMI088` 守卫才能正确判断 |

```cmake
target_compile_options(utils PRIVATE -O2)
```

PRIVATE 因为 `-O2` 只在编译 utils 自己的 `.c` 时生效，上层不继承——每个模块自己决定优化等级。

### target_link_libraries - 链接哪些库

声明一个目标依赖哪些库。到最终链接 `base.elf` 时，链接器把所有 OBJECT 库的 `.o` 全部塞进来，从 STATIC 库的 `.a` 中按需抽取，INTERFACE 库不参与链接只传属性。

```cmake
target_link_libraries(<target> [PRIVATE|PUBLIC|INTERFACE] <lib1> <lib2> ...)
```

| 参数 | 填什么 | 什么意思 |
|------|--------|----------|
| `<target>` | `STM32_Drivers` / `${CMAKE_PROJECT_NAME}` | 谁要链接这些库 |
| 关键字 | `PUBLIC` 或省略 | 控制依赖是否向上传递。**省略关键字时默认 PRIVATE** |
| `lib...` | 库目标名 | 可以是 OBJECT / STATIC / INTERFACE 库的名称 |

```cmake
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)
```

PUBLIC 因为 STM32_Drivers 自己需要 stm32cubemx 的头文件路径（编译 HAL 驱动源文件时需要找到 HAL 头文件），上层链接 STM32_Drivers 时也需要继承这些路径。

```cmake
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx azrtos::threadx utils bsp robot CMSISDSP cherryusb modules app
)
```

这是最终汇总点，把所有库链接到 `base.elf`。不带关键字，默认 PRIVATE——因为 base.elf 是最终产物，没有上层了。

### target_link_options - 链接器选项

给链接器传选项，如 `-flto` 链接时优化、`-T` 链接器脚本。和 `target_compile_options` 的区别：一个控制编译器（预处理+编译+汇编阶段），一个控制链接器（最终合并阶段）。

```cmake
target_link_options(<target> [PRIVATE|PUBLIC|INTERFACE] <option1> <option2> ...)
```

| 参数 | 填什么 | 什么意思 |
|------|--------|----------|
| `<target>` | `${CMAKE_PROJECT_NAME}` | 给哪个目标配链接选项 |
| 关键字 | `PRIVATE` | 选项只自己用 |
| `option...` | `-flto` | 链接时优化（LTO），让编译器在合并所有 `.o` 后做一次跨文件内联和死代码消除 |

```cmake
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -flto)
```

PRIVATE 因为链接选项只有最终链接 base.elf 时才用。`-flto` 让编译器跨模块边界做优化。

---

## 七、add_subdirectory 与 include 的对比

### add_subdirectory - 进入子目录执行

告诉 CMake 进入指定子目录，执行那个目录里的 CMakeLists.txt。执行完毕后回到当前文件继续。子目录里 `add_library` 创建的 target 会注册到全局注册表（父目录可以用），但子目录里 `set` 的变量不会传回父目录（见第五节作用域）。

```cmake
add_subdirectory(source_dir [binary_dir] [EXCLUDE_FROM_ALL])
```

| 参数 | 填什么 | 什么意思 |
|------|--------|----------|
| `source_dir` | `cmake/stm32cubemx` 或 `../../utils` | 子目录的源码路径，相对当前 CMakeLists.txt |
| `binary_dir` | `...`（省略时自动命名） | 构建产物目录名，相对 `CMAKE_BINARY_DIR`。`../../` 从 `board/dji_c/` 向上穿越两级回到项目根目录 |

```cmake
add_subdirectory(cmake/stm32cubemx)
add_subdirectory(../../utils ...)
```

第一个单参数——子目录在当前目录下，构建目录自动命名。第二个双参数——子目录在项目根目录下（穿越两级），第二个参数指定构建目录名。

### 对比 include

`include` 的语法见 [[01_extracted/cmake-basic-syntax#include - 粘贴到当前位置]]。

| | `include()` | `add_subdirectory()` |
|---|---|---|
| 本质 | 粘贴到当前位置执行 | 进入子目录另起炉灶 |
| 作用域 | 共享当前，变量直接生效 | 新建子作用域，变量不传回 |
| target | 可以定义，但主要不是为了这个 | 子目录的 add_library 注册的 target 全局可见 |
| 用在哪 | config.cmake / generate_headers.cmake（需要变量传回） | utils / modules / apps（只需要注册 target） |

判断规则：需要在当前作用域共享变量 → `include()`；只需要注册一个 target → `add_subdirectory()`。
