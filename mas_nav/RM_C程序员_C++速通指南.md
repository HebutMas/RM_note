# C 程序员 C++ 深入指南 — 基于 MAS Vision 项目实战

> **目标读者**：熟练 C 语言，希望系统理解本项目 C++ 代码的开发者。
> **学习方式**：每个特性都从"这段话 C 怎么写"出发，逐层深入，用项目真实代码解释。
> **配套文档**：[[MAS_Vision_学习指南]] · [[MAS_Nav_学习指南]] · [[RM_Socket编程详解]]
> **项目代码位置**：`vision/mas_vision/`（视觉） + `vision/mas_nav/`（导航）

---

## 目录

- [0. 心态转换：从 C 到 C++](#0-心态转换从-c-到-c)
- [1. 基础语法糖](#1-基础语法糖)
- [2. 引用 — 不只是"安全的指针"](#2-引用--不只是安全的指针)
- [3. 命名空间 — 代码组织的基础设施](#3-命名空间--代码组织的基础设施)
- [4. RAII — C++ 最核心的设计哲学](#4-raii--c-最核心的设计哲学)
- [5. 类与对象 — 深入面向对象](#5-类与对象--深入面向对象)
- [6. 模板 — 编译期的代码生成](#6-模板--编译期的代码生成)
- [7. std::atomic — 无锁并发的基石](#7-stdatomic--无锁并发的基石)
- [8. 智能指针 — 所有权语义的体现](#8-智能指针--所有权语义的体现)
- [9. std::optional — 类型安全的"可能没有值"](#9-stdoptional--类型安全的可能没有值)
- [10. Lambda 表达式 — 函数式编程的入口](#10-lambda-表达式--函数式编程的入口)
- [11. 移动语义与完美转发](#11-移动语义与完美转发)
- [12. std::chrono — 类型安全的时间库](#12-stdchrono--类型安全的时间库)
- [13. enum class — 强类型枚举](#13-enum-class--强类型枚举)
- [14. constexpr — 编译期计算](#14-constexpr--编译期计算)
- [15. 异常处理 — 现代 C++ 的错误管理](#15-异常处理--现代-c-的错误管理)
- [16. 标准库容器 — 数据结构的选择](#16-标准库容器--数据结构的选择)
- [17. std::function — 类型擦除的可调用对象](#17-stdfunction--类型擦除的可调用对象)
- [18. 线程与并发 — 从 pthread 到 std::thread](#18-线程与并发--从-pthread-到-stdthread)
- [19. static_assert — 编译期断言](#19-static_assert--编译期断言)
- [20. placement new — 在已有内存上构造对象](#20-placement-new--在已有内存上构造对象)
- [21. 缓存行与内存布局](#21-缓存行与内存布局)
- [22. C++ 属性 — 编译器的注释](#22-c-属性--编译器的注释)
- [23. 默认成员初始化器](#23-默认成员初始化器)
- [24. static 关键字在 C++ 中的多重身份](#24-static-关键字在-c-中的多重身份)
- [25. 第三方库的 C++ 使用模式](#25-第三方库的-c-使用模式)
- [附录 A：C → C++ 完整对照词典](#附录-ac--c-完整对照词典)
- [附录 B：masvision C++ 特性代码索引](#附录-bmasvision-c-特性代码索引)
- [附录 C：常见编译错误速查](#附录-c常见编译错误速查)

---

## 0. 心态转换：从 C 到 C++

### 不只是"带类的 C"

很多 C 程序员初学 C++ 的直觉是：C++ = C + class。这个模型在实践中很快会碰壁。C++ 的设计哲学和 C 有本质差异：

| 维度   | C                       | C++                               |
| ---- | ----------------------- | --------------------------------- |
| 设计哲学 | 数据和函数分离                 | 数据和行为绑定（封装 + 抽象）                  |
| 资源管理 | 手动 `malloc`/`free`，容易泄漏 | 自动（RAII + 析构函数），编译器保证释放           |
| 错误处理 | 返回值 + `goto cleanup`    | 异常（本项目选择性使用）                      |
| 泛型   | `void*` + 函数指针（丢失类型信息）  | 模板（类型安全，零运行时开销）                   |
| 初始化  | 容易忘记，导致未定义行为            | 构造函数强制初始化                         |
| 编译模型 | 简单：预处理 → 编译 → 链接        | 复杂：模板两阶段编译、ODR、内联展开               |
| 标准库  | 小而精（libc）               | 庞大完整（STL + chrono + thread + ...） |

### C++ 编译器做了什么 C 编译器不做的事

理解这些机制有助于阅读本项目的代码：

1. **名字改编（Name Mangling）**：C++ 编译器将函数名编码为包含参数类型信息的符号。这是函数重载的实现基础。`nm` 命令可以看到：`_ZN8auto_aim9ArmorTrackC1ERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES9_` — 这是 `auto_aim::ArmorTrack::ArmorTrack(std::string const&, std::string const&)`。

2. **自动生成特殊函数**：如果你不写，编译器自动生成默认构造函数、析构函数、拷贝构造函数、拷贝赋值运算符（C++11 后还有移动版本）。用 `= delete` 可以显式禁止。

3. **模板实例化**：模板不是代码——是代码的"蓝图"。编译器在看到 `SPSCQueue<CameraFrame>` 时才为 `CameraFrame` 生成一份实际的机器码。这就是为什么模板必须写在头文件中。

4. **内联展开**：C++ 中 `inline` 不再主要用于内联优化（编译器自己决定），而是用于解决 ODR（One Definition Rule）——让同一个函数定义可以出现在多个翻译单元中。

### 阅读本项目 C++ 代码的六条口诀

1. **看到 `struct X { ... }`** → 不一定是纯数据，里面可能有构造函数、方法
2. **看到 `X x(args)`** → 不是函数调用，是构造对象（等价于 `malloc` + 初始化一步完成）
3. **看到 `x.func()`** → 等价于 `func(&x)`，对象是隐式的第一个参数（`this`）
4. **看到 `auto`** → 类型太长交给编译器推导；类型仍是静态的，不是动态语言
5. **看到 `::`** → 作用域分隔符，类比文件系统路径的 `/`；`A::B` 即 "A 里面的 B"
6. **看到 `std::`** → 标准库，就像 C 中函数名前缀 `pthread_`/`f`/`SDL_` 的作用

---

## 1. 基础语法糖

本章覆盖 C++ 中"一眼就能看懂，但值得理解其设计意图"的语法特性。

### 1.1 `auto` — 自动类型推导

**C 写法**：
```c
struct timespec start, end;
unsigned long long elapsed_ns;
int (*cmp)(const void*, const void*) = compare_func;  // 函数指针类型手写
```

**C++ 写法**：
```cpp
auto start = std::chrono::steady_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
auto left2right = right.center - left.center;   // cv::Point2f
```

**项目实例** — `armor_types.hpp:110`：
```cpp
auto left2right = right.center - left.center;
auto width = cv::norm(left2right);
auto max_lightbar_length = std::max(left.length, right.length);
```

**为什么存在 `auto`？** 三种场景：

1. **类型名太长**：`std::chrono::steady_clock::time_point` → `auto`。30+ 字符的类型名让代码失去可读性。
2. **类型无法手写**：Lambda 表达式有独一无二的编译器生成类型——不借助 `auto` 你根本写不出来变量声明。
3. **泛型编程**：模板代码中返回类型依赖于模板参数，`auto` 让编译器帮你追踪。

**关键理解**：`auto` 不是动态类型。下面三行含义完全不同：

```cpp
auto        x1 = expr;  // 值语义：复制一份（去掉引用和顶层 const）
auto&       x2 = expr;  // 左值引用：绑定到原对象
const auto& x3 = expr;  // 常量左值引用：不复制，不修改
auto&&      x4 = expr;  // 转发引用：根据 expr 是左值还是右值变成 T& 或 T&&
```

项目中最常见的是 `const auto &` 和 `auto &&`。前者的语义是"我只是看看，别复制"，后者出现在模板/转发场景。

### 1.2 基于范围的 `for` 循环

**C 写法**：
```c
for (int i = 0; i < armor_count; i++) {
    Armor* armor = &armors[i];
    process(armor);
}
```

**C++ 写法**：
```cpp
for (const auto &armor : armors) {
    process(armor);  // armor 是 armors[i] 的 const 引用，不拷贝
}
```

**项目实例** — `armor_track.cpp:282,291`：
```cpp
for (const auto &armor : armors) {
    if (armor.number != target_->name || armor.type != target_->armor_type) continue;
    found_count++;
}

for (auto &armor : armors) {       // 注意：这里去掉了 const，因为要修改
    armor_pose_.GetArmorPose(armor);
    target_->update(armor);
}
```

**选择指南**：

| 写法                        | 何时用                    | 效果      |
| ------------------------- | ---------------------- | ------- |
| `for (auto x : c)`        | 基础类型 (`int`, `double`) | 拷贝（无所谓） |
| `for (const auto &x : c)` | 只读遍历                   | 零拷贝，最常用 |
| `for (auto &x : c)`       | 需要修改元素                 | 零拷贝，可修改 |
| `for (auto &&x : c)`      | 泛型代码（模板中）              | 转发引用    |

**C++17 结构化绑定**（项目中暂未使用但值得了解）：

```cpp
std::map<std::string, int> scores = {{"red", 3}, {"blue", 5}};
for (const auto &[name, score] : scores) {  // 同时解包 key 和 value
    fmt::print("{}: {}\n", name, score);
    
}
```

### 1.3 `nullptr` — 真正的空指针

C 的 `NULL` 在 C++ 中存在歧义——它通常被定义为 `0`（整数字面量）或 `0L`（长整型），而非指针类型。这在函数重载时会引发非预期的行为。

```cpp
void f(int x);      // 版本 1
void f(void* p);    // 版本 2

f(0);         // 调用版本 1（0 是 int）
f(NULL);      // C++ 中通常也调用版本 1！（因为 NULL = 0 或 0L）
f(nullptr);   // 一定调用版本 2（nullptr 只能转为指针）
```

**`nullptr` 的类型**是 `std::nullptr_t`：

```cpp
std::nullptr_t np = nullptr;
int* p1 = np;    // OK：隐式转为任意指针类型
int  p2 = np;    // 编译错误！不能隐式转为整数
```

**项目实例** — `mas_log.cpp:16`：
```cpp
std::shared_ptr<spdlog::logger> MasLog::logger_ = nullptr;
```

**判空惯用法**：

```cpp
// C 风格（能编译，但与 C++ 风格不一致）
if (ptr == NULL) { ... }

// C++ 风格
if (ptr == nullptr) { ... }  // 最明确
if (!ptr) { ... }            // 最简洁（利用指针的布尔转换）
if (ptr)  { ... }            // 非空判断
```

项目中 `if (!ptr)` 比 `if (ptr == nullptr)` 更常见——二者完全等价。

### 1.4 `using` 别名 — 现代化 `typedef`

**C 写法**：
```c
typedef unsigned char uint8_t;
typedef void (*callback_t)(int);
```

**C++ 写法**：
```cpp
using opt_t = std::uint8_t;
using callback_t = void(*)(int);
```

`using` 的真正优势在于**模板别名**（`typedef` 做不到）：

```cpp
template<typename T>
using MyQueue = rigtorp::SPSCQueue<T, MyAllocator>;
```

**项目实例** — `display.hpp:88`：
```cpp
typedef bool (*SDL_Event_Callback)(const SDL_Event &event, void *user_data);
// 这里用了 C 风格 typedef；新代码应该用 using
```

### 1.5 统一初始化 `{}` — 一种语法初始化万物

C++11 引入的 `{}` 可以初始化几乎所有东西，且不会发生"窄化转换"（narrowing conversion，即丢失信息的隐式类型转换）：

```cpp
cv::Point2f pt{1.5f, 2.0f};
std::vector<int> v{1, 2, 3, 4};
std::atomic<bool> flag{false};
```

**项目实例** — `main.cpp:24`：
```cpp
std::atomic<bool> g_shutdown{false};
```

**项目实例** — `armor_types.hpp:135-142`：
```cpp
const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
    {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
    {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}
};
```

注意 `vector` 的元素也是用 `{}` 初始化的——嵌套初始化语法。

**窄化转换保护**：

```cpp
int x = 3.14;    // C 风格：静默截断为 3，不警告
int x{3.14};     // C++ 风格：编译错误！double → int 是窄化转换
int x = {3.14};  // 同样编译错误
```

### 1.6 `decltype` — 查询表达式的类型

`decltype(expr)` 给出 `expr` 的编译期类型而不求值。它在泛型编程中不可或缺。

**项目实例** — `SPSCQueue.h:56`：
```cpp
decltype(std::declval<Alloc2 &>().allocate_at_least(size_t{}))
// 查询 "调用 allocate_at_least" 的返回类型
// std::declval<T>() 是一个"假装返回 T"的工具——只在 decltype 上下文中合法
```

`decltype` vs `auto`：
- `auto` 推导变量的类型（需要初始化表达式）
- `decltype` 查询任意表达式的类型（不需要变量声明）

---

## 2. 引用 — 不只是"安全的指针"

### 左值引用 `T&`

左值引用是已存在变量的**别名**。从实现角度看，编译器通常用指针实现引用，但从语义角度，引用就是变量的另一个名字。

| | 指针 `T*` | 左值引用 `T&` |
|---|---|---|
| 声明 | `int *p;` | `int &r = x;`（必须初始化） |
| 使用 | `*p = 5;`（显式解引用） | `r = 5;`（自动解引用） |
| 可为空 | 是（`NULL`/`nullptr`） | 否（绑定到 `nullptr` 是未定义行为） |
| 可重新绑定 | `p = &y;` | 不能（一旦绑定终生绑定） |
| 取地址 | `p` 本身就是地址 | `&r` 获取被引用变量的地址 |
| 有 const 版本 | `const int *`（指向常量的指针） | `const int &`（常量的引用） |

**项目实例** — `armor_detector.hpp:18`：
```cpp
ArmorDetector(const std::string &config_path = "config/hikcamera.yaml");
//            常量引用：不复制字符串（可能很长），也不允许修改
```

**项目实例** — `armor_thread.cpp:76`：
```cpp
Eigen::Quaterniond pos = serial.q(frame.timestamp);
tracker.armor_pose().set_R_gimbal2world(pos);
// armor_pose() 返回 ArmorPose& — 引用允许链式调用和原地修改
```

**经验法则**：
- 只读参数 → `const T &`（零拷贝 + 不可修改）
- 要修改的参数 → `T &`（零拷贝 + 可修改）
- 可选的参数 → `T *`（允许传 `nullptr`）
- 转移所有权 → `T &&`（见第 11 章）
- 返回对象内部数据 → `const T &`（防止外部修改）

### 右值引用 `T&&`

右值引用是 C++11 引入的——它**只绑定到临时对象（右值）**，目的是"窃取"临时对象的资源而非复制。详见第 11 章移动语义。

### `const T&` 的生命周期延长

一个不太为人知的特性：将一个临时对象绑定到 `const T&` 会延长它的生命周期：

```cpp
std::string getName() { return "hello"; }

const std::string &ref = getName();  // 函数返回的临时 string 存活到 ref 离开作用域
// 没有 ref 的话，这行分号后临时对象就析构了
```

这让你可以安全地用 `const T&` 接收函数返回值，避免一次拷贝。

---

## 3. 命名空间 — 代码组织的基础设施

### 问题本质

C 项目中的"人肉命名空间"：
```c
// 每个模块的所有函数都带前缀
rm_utils_ekf_init();
rm_utils_ekf_predict();
auto_aim_armor_detect();
auto_aim_armor_track();
```

这个模式有三个问题：前缀冗长、无法用 `using` 选择性引入、编译器不检查前缀一致性。

### C++ 方案

```cpp
namespace auto_aim {
    void armor_detect();
    void armor_track();
}
```

使用方式：
```cpp
auto_aim::armor_detect();           // 显式限定（最推荐）
using namespace auto_aim;           // 全部引入（不要在头文件中用！）
using auto_aim::armor_detect;       // 只引入一个（推荐）
```

**本项目使用的命名空间一览**：

| 命名空间          | 用途                | 定义位置                           |
| ------------- | ----------------- | ------------------------------ |
| `auto_aim`    | 核心视觉算法（检测、跟踪、射击）  | `armor_types.hpp:9`            |
| `rm_utils`    | 工具库（日志、显示、EKF、绘图） | `extended_kalman_filter.hpp:9` |
| `threads`     | 各线程工作函数           | `armor_thread.hpp`             |
| `rigtorp`     | 第三方 SPSCQueue     | `SPSCQueue.h:42`               |
| `BS`          | 第三方线程池            | `BS_thread_pool.hpp`           |
| `calibration` | 相机标定模块            | `calibrate_camera.hpp`         |
| `serial`      | 第三方串口库            | `serial.h`                     |

**项目实例** — `armor_types.hpp:9-144`：
整个文件内容都在 `namespace auto_aim { ... }` 块内，所有类型（`LightBar`、`Armor`、`ArmorType`）都在这个作用域下。

### 匿名命名空间 — 替代 `static` 全局

```cpp
// C 写法：文件作用域
static int internal_counter = 0;

// C++ 写法（推荐）：匿名命名空间
namespace {
    int internal_counter = 0;
}  // 匿名命名空间内的名字只在当前翻译单元可见
```

匿名命名空间比 `static` 更通用——`static` 不能用于类型定义，匿名命名空间可以。

### 命名空间别名

```cpp
namespace chr = std::chrono;  // 缩写长的命名空间名
auto t = chr::steady_clock::now();
```

### ADL（Argument-Dependent Lookup，实参依赖查找）

C++ 查找函数名时会**同时**在实参类型的命名空间中查找：

```cpp
namespace auto_aim {
    struct Armor { ... };
    void process(const Armor &a) { ... }  // 在 Armor 的命名空间中
}

auto_aim::Armor armor;
process(armor);  // 不需要写 auto_aim::process！编译器在 auto_aim 中找到了
```

这就是为什么 `std::cout << "hello"` 不需要写 `std::operator<<`——编译器通过 ADL 在 `std` 命名空间找到了 `operator<<`。

---

## 4. RAII — C++ 最核心的设计哲学

### 什么是 RAII

**R**esource **A**cquisition **I**s **I**nitialization — 资源获取即初始化。

核心思想：**把资源的生命周期绑定到对象的生命周期。构造函数获取资源，析构函数释放资源。** 无论作用域以何种方式退出（正常 return、异常、break），栈上对象的析构函数都会被调用。

### C 资源管理的三种模式及其痛点

**模式 1：单一资源**
```c
void process_file(const char *path) {
    FILE *f = fopen(path, "r");
    if (f == NULL) return;

    // ... 使用 f ...

    fclose(f);  // 如果中间有 return，这一行永远执行不到 → 泄漏
}
```

**模式 2：多个资源 → `goto cleanup`**
```c
int process() {
    int ret = -1;
    FILE *f = NULL;
    char *buf = NULL;
    pthread_mutex_t *mtx = NULL;

    f = fopen("config.yaml", "r");
    if (!f) goto cleanup;

    buf = malloc(4096);
    if (!buf) goto cleanup;

    mtx = malloc(sizeof(pthread_mutex_t));
    if (!mtx) goto cleanup;
    pthread_mutex_init(mtx, NULL);

    // ... 使用资源 ...
    ret = 0;

cleanup:
    if (mtx) { pthread_mutex_destroy(mtx); free(mtx); }
    if (buf) free(buf);
    if (f) fclose(f);
    return ret;
}
```

这个模式有两个致命问题：(1) 清理顺序容易搞错，(2) 新增资源需要在所有 return 路径和 `cleanup` 标签处都加上释放代码。

**模式 3：更糟 — 遗忘初始化**
```c
LightBar bar;          // 结构体内容随机！
bar.center.x = 100;    // 设了一个字段，其余是垃圾值
process(&bar);         // 未定义行为
```

### C++ RAII 方案

```cpp
void process() {
    std::ifstream f("config.yaml");      // 构造 = 打开文件
    std::vector<char> buffer(4096);      // 构造 = 分配内存
    std::lock_guard<std::mutex> lock(mtx); // 构造 = 加锁

    // ... 使用 f, buffer, lock ...

}  // 不管怎么退出，三个对象的析构函数按构造的逆序自动调用：
   // lock.~lock_guard() → 解锁
   // buffer.~vector()   → 释放内存
   // f.~ifstream()      → 关闭文件
```

### 本项目 RAII 实战

**例 1：SPSCQueue 析构函数清空队列** — `SPSCQueue.h:100-107`：

```cpp
~SPSCQueue() {
    while (front()) {
        pop();              // 析构队列中所有未处理的元素
    }
    // 释放分配的内存
    std::allocator_traits<Allocator>::deallocate(allocator_, slots_, capacity_ + 2 * kPadding);
}
```

**例 2：海康相机自动关闭** — `hikcamera.hpp:18`：
```cpp
~HikCamera();  // 析构函数调用 MV_CC_CloseDevice()、MV_CC_DestroyHandle() 等
```

**例 3：`std::lock_guard` 自动解锁** — 本项目 `display.hpp` 中的 `std::mutex queues_mutex_`：
```cpp
void Display::display_add(...) {
    std::lock_guard<std::mutex> lock(queues_mutex_);  // 加锁
    // ... 操作共享数据 ...
}  // lock 析构 → 自动解锁（即使中间抛异常）
```

**例 4：禁止拷贝/移动来保护资源** — `SPSCQueue.h:110-111`：
```cpp
SPSCQueue(const SPSCQueue &) = delete;            // 禁止拷贝构造
SPSCQueue &operator=(const SPSCQueue &) = delete; // 禁止拷贝赋值
// 没有声明移动构造/赋值 → 同样被禁止
// 原因：SPSCQueue 内部有原子变量和裸指针，拷贝会导致双重释放
```

### RAII 的进阶：Scope Guard 模式

项目中没有使用，但值得了解——用 RAII 执行任意清理代码：

```cpp
// 伪代码：确保函数退出时执行特定操作
void critical_section() {
    auto cleanup = []() { /* 恢复状态 */ };
    // C++ 没有标准 scope guard，但可以用 unique_ptr + 自定义删除器模拟
    auto guard = std::unique_ptr<void, decltype(cleanup)>(static_cast<void*>(0), cleanup);
    // ... 危险操作 ...
}  // guard 析构 → cleanup() 被调用
```

### 记忆口诀

> 看到 `}` → 编译器自动调 `free()`/`fclose()`/`pthread_mutex_unlock()`。你不需要写释放代码——编译器帮你插入了。

---

## 5. 类与对象 — 深入面向对象

### 从 C struct 到 C++ class

C 中数据和操作分离：
```c
// 数据定义
typedef struct {
    cv::Mat gray;
    cv::Mat binary;
    int threshold;
} ArmorDetector;

// 操作定义（函数第一个参数是结构体指针）
void armor_detector_init(ArmorDetector *self, int threshold);
vector_armor armor_detector_detect(ArmorDetector *self, cv::Mat frame);
```

C++ 中数据和操作绑定：
```cpp
class ArmorDetector {
    cv::Mat gray_;
    cv::Mat binary_;
    int threshold_;

public:
    ArmorDetector(int threshold);                  // 构造 = init
    std::vector<Armor> ArmorDetect(cv::Mat frame); // 方法
};
```

**调用对比**：
```cpp
// C: 把对象作为显式参数
armor_detector_detect(&det, frame);

// C++: 对象是隐式的 this 参数
det.ArmorDetect(frame);   // det 对应 this 指针
```

### `struct` vs `class`

|        | `struct`                                  | `class`                               |
| ------ | ----------------------------------------- | ------------------------------------- |
| 默认访问权限 | `public`                                  | `private`                             |
| 默认继承方式 | `public`                                  | `private`                             |
| 本项目惯用法 | 纯数据聚合（`LightBar`, `Armor`, `CameraFrame`） | 有行为的对象（`ArmorDetector`, `ArmorTrack`） |
| C 兼容性  | 与 C struct 二进制兼容（POD）                     | C++ 独有                                |

**项目实例** — `armor_types.hpp:29-58`（struct）vs `armor_track.hpp:16-101`（class）：

`LightBar` 用 `struct`——它主要是数据，虽然有个构造函数。`ArmorTrack` 用 `class`——它有状态机、私有辅助函数、公有接口。

### 构造函数与初始化列表

**为什么需要初始化列表？** C++ 中，对象的成员变量在**进入构造函数体之前**就已经被默认初始化了。在构造函数体内用 `=` 赋值的本质是先默认初始化再赋值——对 `cv::Mat` 这类大对象是很大的浪费。初始化列表直接在构造时完成初始化：

```cpp
Armor::Armor(const LightBar &l, const LightBar &r)
    : left(l), right(r)    // 初始化列表：直接调用 LightBar 的拷贝构造
{
    // 此时 left 和 right 已经构造完毕，直接用
    color = left.color;
    center = (left.center + right.center) / 2;
}
```

**初始化顺序**：成员按**声明顺序**初始化，不是按初始化列表的书写顺序。如果把 `left` 声明在 `right` 之前，初始化列表就按 `left` → `right` 的顺序执行——不管你怎么写。

**项目实例** — `armor_track.cpp:24-26`：
```cpp
ArmorTrack::ArmorTrack(const std::string &track_config_path, const std::string &pose_config_path)
    : armor_pose_(pose_config_path), detect_count_(0), temp_lost_count_(0), state_{"lost"}, pre_state_{"lost"},
      last_timestamp_(std::chrono::steady_clock::now())
```

这里所有成员的初始值一目了然。如果你把初始化放在构造函数体内，阅读者需要读完整个函数体才能确定每个成员的初值。

### 访问控制：接口与实现分离

```cpp
class ArmorTrack {
public:    // 外部依赖的接口（稳定的契约）
    ArmorTrack(const std::string &track_config_path, ...);
    std::optional<Target> track(...);

private:   // 内部实现细节（可以随意重构）
    ArmorPose armor_pose_;
    std::optional<Target> target_;
    int detect_count_;
    void state_machine(bool found);   // 外部不应该调用状态机
    bool set_target(const Armor &, ...);
    bool update_target(...);
};
```

`private` 的魔力：改了 `detect_count_` 的类型或删了 `state_machine`，外部调用者不需要改一行代码。

### 继承与虚函数

本项目继承使用较少但关键：

**项目实例** — `user_serial.hpp` 中引用的 serial 库：
```cpp
class SerialException : public std::exception {
    const char *what() const noexcept override {
        return "Serial error";
    }
};
```

- `public` 继承 = "是一个"（is-a）关系。`SerialException` 是一个 `std::exception`。
- `override` = 显式声明"我在重写基类的虚函数"。如果基类没有匹配的虚函数，编译器会报错——不用 `override` 的话你只是定义了一个新的无关函数。
- `virtual` = 允许子类重写。虚函数通过 `vtable`（虚函数表）实现动态分发——调用 `what()` 时，具体执行哪个版本的代码由对象的实际类型决定，不是由指针/引用的声明类型决定。

### `const` 成员函数

```cpp
class ArmorTrack {
public:
    std::string state() const { return state_; }  // const 承诺：这个函数不修改对象
};
```

`const` 成员函数内部，`this` 的类型是 `const ArmorTrack*`——所有成员变量都是只读的。这让你可以在 `const ArmorTrack&` 参数上调用 `state()`。

### `mutable` — 突破 const 的合法方式

有时 const 成员函数确实需要修改一些"非逻辑状态"的成员（如缓存、统计计数器）：

**项目实例** — `armor_track.hpp:95-97`：
```cpp
mutable std::map<std::string, double> fps_map_;
mutable std::map<std::string, int> count_map_;
mutable std::map<std::string, std::chrono::steady_clock::time_point> last_time_map_;
```

这些是 FPS 统计用的缓存——从逻辑上讲，`showResult()` 不改变跟踪器的"实质状态"。用 `mutable` 让这些变量即使在 `const` 成员函数中也能被修改。

**项目实例** — `armor_shoot.hpp:100`：
```cpp
mutable std::map<std::string, double> fps_map_;
```

同样用于调试统计，不构成对象的"逻辑状态"改变。

### `static` 类成员

类中的 `static` 成员属于**整个类**而非某个对象：

**项目实例** — `mas_log.hpp:48`：
```cpp
static std::shared_ptr<spdlog::logger> logger_;  // 全局唯一的 logger 实例
```

所有 `MasLog` 对象共享同一个 `logger_`。`static` 成员必须在类外定义（通常在 .cpp 文件中）：

```cpp
// mas_log.cpp
std::shared_ptr<spdlog::logger> MasLog::logger_ = nullptr;
```

### 单例模式 — 全局唯一实例

本项目多处使用"懒汉单例"（Meyer's Singleton）：

**项目实例** — `display.hpp:101-105`：
```cpp
static Display &getInstance() {
    static Display instance;   // C++11 保证：多线程下只初始化一次
    return instance;
}
```

**项目实例** — `user_serial.hpp:21-25`：
```cpp
static UserSerial &getInstance(const std::string &config_path = "config/serial_config.yaml") {
    static UserSerial instance(config_path);
    return instance;
}
```

关键设计：
1. **构造函数私有** → 外部不能 `new UserSerial()`，只能通过 `getInstance()` 获取
2. **`= delete` 拷贝/赋值** → 不能复制单例（`UserSerial(const UserSerial&) = delete`）
3. **`static` 局部变量** → C++11 起，多线程并发执行到 `static` 局部变量初始化时，只有一个线程执行初始化，其余等待。这叫做"magic statics"。

线程安全保证（C++11 [stmt.dcl]/4）：如果初始化期间抛异常，变量仍未初始化，下次调用会重试。

---

## 6. 模板 — 编译期的代码生成

模板是 C++ 最强大的特性，也是最让 C 程序员困惑的部分。它的核心概念：**模板不是代码——是编译器用来生成代码的"蓝图"**。

### 函数模板

```cpp
template<typename T>                // T 是"类型参数"——编译时确定
T max(T a, T b) {
    return a > b ? a : b;
}

// 调用时：
int x = max<int>(3, 5);            // 显式指定 T = int
int y = max(3, 5);                 // 编译器从参数推导 T = int（模板实参推导）
```

编译器在看到 `max(3, 5)` 时自动生成 `int max(int a, int b)`。这个过程叫**隐式实例化**。如果你调用 `max(1.5, 2.5)`，编译器生成 `double max(double, double)`。对每种类型，编译器生成一份独立的机器码——和手写一样快。

### 类模板

```cpp
template<typename T, typename Allocator = std::allocator<T>>
class SPSCQueue {
    // T 是什么类型，队列就存什么类型
    void push(const T &item);
    T* front();
};
```

使用：
```cpp
SPSCQueue<CameraFrame> camera_queue(2);   // T = CameraFrame，队列存整帧图像
SPSCQueue<int> int_queue(10);              // T = int
```

**项目实例** — `SPSCQueue.h:45-258`：整个 SPSCQueue 就是一个模板类，第二个参数有默认值（`std::allocator<T>`）。

### 模板的编译模型（为什么模板必须在头文件中）

这是 C 程序员最容易踩的坑。模板不是普通函数——编译器只有在看到完整的模板定义**和**所有模板参数时才能生成代码。所以模板的完整定义必须放在头文件中，不能分开声明（`.h`）和定义（`.cpp`）。

```cpp
// 错误做法：
// foo.hpp
template<typename T> void foo(T x);  // 只有声明

// foo.cpp
template<typename T> void foo(T x) { /* ... */ }  // 定义在 .cpp 中
// 其他 .cpp 文件 include foo.hpp 时看不到定义 → 链接错误！

// 正确做法：全部写在头文件中
// foo.hpp
template<typename T> void foo(T x) { /* ... */ }
```

本项目 SPSCQueue 的完整实现（250+ 行）全在一个 `.h` 文件中，原因即在此。

### 模板参数不止类型

模板参数可以是：
1. **类型参数**：`template<typename T>` — 最常用
2. **非类型参数**：`template<int N>` — 编译时常量
3. **模板模板参数**：`template<template<typename> class Container>` — 少见

项目中没有后两种的典型用法。

### `typename` vs `class`

```cpp
template<typename T>   // ✓ 推荐：T 是任意类型
template<class T>      // ✓ 等价：历史遗留写法
```

两者在模板参数声明中完全等价。但在**依赖名称**前，必须用 `typename` 告诉编译器"这是个类型"：

```cpp
template<typename T>
void foo() {
    typename T::value_type x;  // 必须加 typename：T::value_type 依赖于 T，编译器不知道它是类型还是值
}
```

### 变参模板（Variadic Templates）

**项目实例** — `SPSCQueue.h:113-130`：

```cpp
template <typename... Args>
void emplace(Args &&...args) noexcept(std::is_nothrow_constructible<T, Args &&...>::value) {
    // ...
    new (&slots_[writeIdx + kPadding]) T(std::forward<Args>(args)...);
}
```

- `typename... Args` — 模板参数包：接受零个或多个类型
- `Args &&...args` — 函数参数包：接受零个或多个参数
- `std::forward<Args>(args)...` — 包展开：对每个参数各自完美转发

调用 `emplace(frame)` 展开成：`T(std::forward<CameraFrame>(frame))`
调用 `emplace(a, b, c)` 展开成：`T(std::forward<A>(a), std::forward<B>(b), std::forward<C>(c))`

### SFINAE（Substitution Failure Is Not An Error）

替换失败不是错误。当模板参数替换导致非法代码时，编译器不报错——只是把该重载从候选集中移除。

```cpp
// 只有当 P 可以用于构造 T 时，这个 push 才存在
template <typename P,
          typename = typename std::enable_if<std::is_constructible<T, P &&>::value>::type>
void push(P &&v) noexcept(...) {
    emplace(std::forward<P>(v));
}
```

`std::enable_if<条件>::type` — 条件为 true 时定义为 `void`（重载启用），为 false 时编译失败（重载被移除，但不报错）。

**项目实例** — `SPSCQueue.h:163-168`：这个 `push(P&&)` 只在 `T` 可以从 `P&&` 构造时才存在。否则，只有 `push(const T&)` 可用——保证了类型安全。

### 类型萃取（Type Traits）

`<type_traits>` 头文件提供了一套编译期查询类型属性的工具：

```cpp
std::is_constructible<T, Args&&...>::value     // T 能否从 Args&&... 构造？
std::is_copy_constructible<T>::value            // T 能否拷贝构造？
std::is_nothrow_constructible<T, Args&&...>::value  // 构造是否不抛异常？
std::is_nothrow_destructible<T>::value          // 析构是否不抛异常？
```

**项目实例** — `SPSCQueue.h`：
```cpp
// 用 noexcept 条件依赖于 T 的性质
void push(const T &v) noexcept(std::is_nothrow_copy_constructible<T>::value);

// 用 static_assert 强制编译期检查
static_assert(std::is_constructible<T, Args &&...>::value,
              "T must be constructible with Args&&...");
```

### `if constexpr` — 编译期条件分支

C++17 引入。与普通 `if` 不同，不成立的分支**不会被编译**：

**项目实例** — `SPSCQueue.h:79-92`：
```cpp
if constexpr (has_allocate_at_least<Allocator>::value) {
    // 如果分配器支持 allocate_at_least，用更高效的方式
    auto res = allocator_.allocate_at_least(capacity_ + 2 * kPadding);
    slots_ = res.ptr;
    capacity_ = res.count - 2 * kPadding;
} else {
    // 降级方案
    slots_ = std::allocator_traits<Allocator>::allocate(allocator_, capacity_ + 2 * kPadding);
}
// else 分支在 has_allocate_at_least 为 true 时根本不会被编译——不需要 #ifdef
```

对比 C 预处理器的优势：`if constexpr` 有作用域语义，两个分支都能看到周围的变量，编译器检查两个分支的语法。

### 模板记忆要点

1. 模板 = 编译器代码生成器（类比 C 宏，但类型安全 + 有作用域 + 支持递归）
2. 每次用不同类型实例化，都是独立的一份机器码
3. **定义必须写在头文件中**
4. `typename T` = "某种类型，编译时确定"
5. 变参模板 + 完美转发 = 零开销抽象的主力工具
6. SFINAE + type traits = 编译期的"接口检查"

---

## 7. std::atomic — 无锁并发的基石

### 为什么需要原子操作

多线程并发访问同一个变量时，即使是 `int` 的 `++` 操作也不是原子的：

```cpp
// counter++ 在汇编层面是三步：
// 1. load counter → register
// 2. increment register
// 3. store register → counter
// 线程 A 在执行步骤 2 时，线程 B 可能执行了步骤 1——丢失一次更新
```

`std::atomic<T>` 保证对它的每个操作都是**不可分割的**——其他线程看到的要么是操作前、要么是操作后的完整状态，不存在"操作到一半"的中间态。

### C vs C++ 原子操作

```c
// C11 <stdatomic.h>
atomic_bool g_shutdown = false;
atomic_store(&g_shutdown, true);     // 写
if (atomic_load(&g_shutdown)) ...     // 读
```

```cpp
// C++ <atomic>
std::atomic<bool> g_shutdown{false};
g_shutdown.store(true);              // 显式写
if (g_shutdown.load()) ...           // 显式读

// 或者更自然：
g_shutdown = true;                   // 隐式 store（默认 memory_order_seq_cst）
if (g_shutdown) ...                  // 隐式 load
```

**项目实例** — `main.cpp:24`：
```cpp
std::atomic<bool> g_shutdown{false};

void signal_handler(int signum) {
    g_shutdown = true;   // 信号处理器写 → 所有线程读 → 安全退出
}
```

### SPSCQueue 中大量使用原子操作

SPSCQueue（单生产者单消费者队列）的核心就是用原子操作实现无锁线程通信。

**项目实例** — `SPSCQueue.h`:
```cpp
alignas(kCacheLineSize) std::atomic<size_t> writeIdx_ = {0};
alignas(kCacheLineSize) size_t readIdxCache_          = 0;   // 非原子！只被一个线程读
alignas(kCacheLineSize) std::atomic<size_t> readIdx_  = {0};
alignas(kCacheLineSize) size_t writeIdxCache_         = 0;   // 非原子！只被一个线程读
```

设计精妙之处：
- `writeIdx_` 是原子的——producer 写，consumer 读
- `writeIdxCache_` 是非原子的——只被 consumer 线程读（producer 不碰）
- **缓存行对齐**（`alignas(64)`）：防止相邻变量在同一缓存行上产生伪共享（false sharing）

### 伪共享（False Sharing）详解

现代 CPU 的缓存一致性以**缓存行**（典型 64 字节）为单位。如果两个核心各自频繁写**同一个缓存行上的不同变量**，缓存一致性协议会不断在两个核心间 ping-pong 该缓存行，性能急剧下降。

```cpp
// 没有 alignas：writeIdx_ 和 readIdxCache_ 可能在同一缓存行
// Producer 写 writeIdx_ → Consumer 的缓存行被 invalidate
// Consumer 读 readIdxCache_ → 必须重新从内存加载整个缓存行
// 实际上它们没有任何数据竞争，只是"住得太近"

// 解决方案：每个变量独占一个缓存行
alignas(64) std::atomic<size_t> writeIdx_;
alignas(64) size_t readIdxCache_;
alignas(64) std::atomic<size_t> readIdx_;
alignas(64) size_t writeIdxCache_;
```

### 内存序（Memory Order）深入理解

原子性只是 `atomic` 的一半。另一半是**内存序**——控制编译器/CPU 在原子操作周围的重排行为。

| 内存序 | 开销 | 含义 | 适用场景 |
|--------|------|------|----------|
| `relaxed` | 最低 | 仅保证原子性，无顺序约束 | 单调计数器 |
| `acquire` | 中 | 读操作：后续读写不重排到此之前 | consumer 读数据 |
| `release` | 中 | 写操作：之前读写不重排到此之后 | producer 发布数据 |
| `acq_rel` | 较高 | acquire + release | read-modify-write |
| `seq_cst` | 最高 | 全局顺序一致 | 默认，不需要精细优化的场合 |

**项目实例** — SPSCQueue 的 push 流程：
```cpp
// Producer 发布数据
writeIdx_.store(nextWriteIdx, std::memory_order_release);
// release 语义：placement new 必须在 store 之前对 consumer 可见
// → consumer 看到新 writeIdx_ 时，一定也能看到构造完成的对象

// Consumer 检查数据
writeIdxCache_ = writeIdx_.load(std::memory_order_acquire);
// acquire 语义：读取对象数据的操作不能重排到 load 之前
// → 保证先确认有数据，再读取数据
```

**性能优化思路**：SPSCQueue 用 `relaxed` 做本地缓存更新（减少昂贵的跨核心通信），只在必要时才用 `acquire`/`release` 做同步。这是"读缓存，有疑问再问主存"的策略。

### 本项目的内存序使用总结

```cpp
// 项目中最常见的模式 — 默认 seq_cst，开销最大但最安全
g_shutdown = true;             // 等价于 store(true, seq_cst)

// SPSCQueue 中精细控制 — 追求极致性能
writeIdx_.load(std::memory_order_relaxed);     // 快速检查
readIdx_.load(std::memory_order_acquire);       // 确认读取
writeIdx_.store(..., std::memory_order_release); // 发布写入
```

**口诀**：90% 的情况用默认的 `seq_cst` 足够。只有像 SPSCQueue 这样对延迟极度敏感的热路径才需要手动指定内存序。

---

## 8. 智能指针 — 所有权语义的体现

### 为什么需要智能指针

C 的 `malloc`/`free` 模式的问题：
1. **忘记 `free`** → 内存泄漏
2. **`free` 两次** → 崩溃
3. **`free` 后继续用** → use-after-free，安全漏洞
4. **异常/提前 return** → 跳过了 `free`

智能指针的思想：把堆内存的**所有权**编码到类型中，编译器保证正确释放。

### `std::unique_ptr<T>` — 独占所有权

```cpp
std::unique_ptr<NumberClassifier> classifier;
classifier = std::make_unique<NumberClassifier>(config);
// classifier 是唯一的所有者，不能复制
// 离开作用域自动 delete
```

**项目实例** — `armor_detector.hpp:96`：
```cpp
std::unique_ptr<NumberClassifier> classifier;
```

**项目实例** — `armor_shoot.hpp:96`：
```cpp
std::unique_ptr<rm_utils::Plotter> plotter_;
```

`unique_ptr` 不能拷贝（编译错误），只能移动——所有权唯一。这让你一眼就能判断："这个资源只有一个地方负责释放"。

### `std::shared_ptr<T>` — 共享所有权，引用计数

```cpp
auto camera_buffer = std::make_shared<rigtorp::SPSCQueue<CameraFrame>>(2);
// 多个线程协程同一个 shared_ptr 指向的队列
auto f1 = pool.submit_task([camera_buffer]() {
    camera_buffer->try_push(frame);  // 线程 1 持有 shared_ptr
});
auto f2 = pool.submit_task([camera_buffer]() {
    auto frame = camera_buffer->front();  // 线程 2 也持有
});
// 最后一个持有者销毁时，队列才被释放——不会提前释放
```

**项目实例** — `main.cpp:83`：
```cpp
auto camera_buffer = std::make_shared<rigtorp::SPSCQueue<CameraFrame>>(camera_buffer_size);
// 随后传给多个线程 lambda —— 每个 lambda 捕获自己的 shared_ptr 副本
pool.submit_task([camera_buffer, usb_buffer]() {
    threads::armor_thread_func(camera_buffer, usb_buffer);
});
```

引用计数原理：
- 每次拷贝 `shared_ptr`，内部引用计数 +1
- 每次析构 `shared_ptr`，引用计数 -1
- 计数归零 → 释放资源

开销：引用计数是原子的（线程安全），有额外内存分配（控制块）。

### `std::weak_ptr<T>` — 不增加引用计数的观察者

项目中暂未使用，但在循环引用场景中至关重要：

```cpp
// 场景：A 持有 B 的 shared_ptr，B 持有 A 的 shared_ptr → 引用计数永不归零
struct B;
struct A { std::shared_ptr<B> b; };
struct B { std::shared_ptr<A> a; };  // 互相持有 → 泄漏！

// 解决：B 用 weak_ptr
struct B { std::weak_ptr<A> a; };  // weak_ptr 不影响引用计数
```

### 选择指南

| 场景 | 使用 | 等价 C 模式 |
|------|------|-------------|
| 一个所有者 | `unique_ptr<T>` | `malloc` + 单一 `free` 点 |
| 多个所有者（共享） | `shared_ptr<T>` | 手动引用计数 |
| 观察者（可能失效） | `weak_ptr<T>` | 无 C 等价物 |
| 不拥有所有权 | `T*` 或 `T&` | 普通指针 |
| 数组/缓冲区 | `std::vector<T>` | `malloc(n * sizeof(T))` |

### 特别注意

1. **不要手动 `new`**：用 `std::make_unique` 和 `std::make_shared` 而不是 `new`——可以避免内存泄漏（如果构造函数抛异常）。
2. **不要用 `shared_ptr` 当"万能钥匙"**：如果只有一个明确的所有者，用 `unique_ptr`。`shared_ptr` 有原子引用计数开销。
3. **`shared_ptr` 不是线程安全的**：对 `shared_ptr` 对象本身的读写需要外部同步（但引用计数的增减是原子的）。
4. **不要从 `this` 创建 `shared_ptr`**：会导致多个控制块，各自计数归零时重复释放。如果需要，继承 `std::enable_shared_from_this<T>`。

---

## 9. std::optional — 类型安全的"可能没有值"

### C 的三种"可能没有值"方案

```c
// 方案 1：返回值 + 输出参数
int find_target(Target* out) {
    if (found) { *out = target; return 0; }
    return -1;  // -1 = 没找到
}

// 方案 2：返回 NULL 指针
Target* find_target() {
    if (found) return &target_obj;
    return NULL;  // NULL = 没找到
}

// 方案 3：哨兵值
int find_index() {
    if (found) return index;
    return -1;  // -1 是合法的"没找到"哨兵
}
```

三个方案都有问题：(1) 输出参数不够直观，(2) `NULL` 没有类型信息——你只能靠注释/文档知道 `NULL` 代表什么，(3) 哨兵值可能和合法值冲突（万一下标就是 -1 呢？）。

### C++ 方案

```cpp
std::optional<Target> track(std::vector<Armor> &armors, std::chrono::steady_clock::time_point t) {
    if (!found) return std::nullopt;  // 明确的"没有值"
    return Target{...};               // 有值
}
```

使用：
```cpp
auto maybe_target = track(armors, t);

if (maybe_target) {                      // 隐式 bool 转换：有值吗？
    shoot(*maybe_target, timestamp);     // * 解引用（等价于 .value()）
}

// 或者更安全的方式：
if (maybe_target.has_value()) {
    shoot(maybe_target.value(), timestamp);
}

// 提供默认值：
Target t = maybe_target.value_or(default_target);  // C++23: and_then, or_else, transform
```

**项目实例** — `armor_track.hpp:47-48`：
```cpp
std::optional<Target> track(std::vector<Armor> &armors,
                            std::chrono::steady_clock::time_point t,
                            ...);
```

返回 `std::optional<Target>` 明确表达：调用者必须检查目标是否存在才能使用。

**项目实例** — `armor_shoot.hpp:59`：
```cpp
SendPacket shoot(const std::optional<Target> &target, ...);
```

参数也是 `optional`——传入 `std::nullopt` 表示没有目标。

### `std::optional` vs 指针 vs 异常

| 方案 | 适用场景 |
|------|----------|
| `std::optional<T>` | "可能没有值"是正常情况（如跟踪丢失） |
| 抛异常 | 没有值是异常情况（如配置文件解析失败） |
| 返回指针 | 对象已存在（由别人所有），只需要引用它 |
| `std::expected<T, E>`(C++23) | 可能失败且有错误信息（比 optional 更多信息） |

### 内部原理

`std::optional<T>` 内部维护一块对齐过的内存（大小 = `sizeof(T)` + 一个 `bool` 标志）。当有值时，对象构造在这块内存上（用 placement new）；当清空时（`reset()`），析构函数被调用。

这就是为什么 `std::optional<T>` 的大小比 `T` 略大。

---

## 10. Lambda 表达式 — 函数式编程的入口

### 从函数指针到 Lambda

C 回调模式：
```c
int compare_by_y(const void *a, const void *b) {
    return ((Point*)a)->y - ((Point*)b)->y;
}
qsort(points, n, sizeof(Point), compare_by_y);  // 函数定义在别处
```

C++ Lambda：
```cpp
std::sort(corners.begin(), corners.end(),
    [](const cv::Point2f &a, const cv::Point2f &b) {
        return a.y < b.y;   // 行为定义在使用处
    });
```

### Lambda 语法拆解

```
[捕获列表](参数列表) -> 返回类型 { 函数体 }
        可选     可选（通常 auto 推导）
```

**项目实例** — `armor_types.hpp:43`：
```cpp
std::sort(corners.begin(), corners.end(),
    [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });
```

这里的 `[]` 是空捕获——不需要外部变量。

### 捕获列表详解

捕获列表决定 Lambda 如何访问外部变量：

```cpp
int threshold = 90;
int counter = 0;

// [=] — 按值捕获所有外部变量（复制一份）
auto check = [=](int value) { return value > threshold; };  // threshold 是副本

// [&] — 按引用捕获所有外部变量（共享同一份）
auto increment = [&]() { counter++; };  // 修改外部 counter

// [var] — 按值捕获特定变量
auto check2 = [threshold](int value) { return value > threshold; };

// [&var] — 按引用捕获特定变量
auto increment2 = [&counter]() { counter++; };

// [=, &counter] — 默认按值，counter 例外（按引用）
// [&, threshold] — 默认按引用，threshold 例外（按值）
```

**项目实例** — `main.cpp:86-87`：
```cpp
auto camera_future = pool.submit_task(
    [camera_buffer_size, camera_buffer]() {
        threads::camera_thread_func("config/hikcamera.yaml", camera_buffer_size, camera_buffer);
    });
```

这里 `capture_buffer`（`shared_ptr`）按值捕获——Lambda 持有 `shared_ptr` 的一份副本，引用计数 +1。这保证了队列在 Lambda 执行期间不被释放。

**项目实例** — `armor_track.cpp:71-75`：
```cpp
auto getPriorityScore = [&](const Armor &armor) -> int {
    auto it = std::find(priority_order.begin(), priority_order.end(), armor.number);
    if (it != priority_order.end()) return std::distance(priority_order.begin(), it);
    return 100;
};
```

这里用 `[&]` 按引用捕获——`priority_order` 是栈上的大 `vector`，不需要复制。

### 泛型 Lambda（C++14）

Lambda 参数可以用 `auto`：

```cpp
auto print = [](const auto &x) { fmt::print("{}\n", x); };
print(42);       // 实例化 print<int>
print("hello");  // 实例化 print<const char*>
```

编译器为每种参数类型生成独立的 Lambda 函数体——和模板一样。

### `mutable` Lambda

默认情况下，按值捕获的变量在 Lambda 内是 `const` 的。加 `mutable` 可以修改：

```cpp
int counter = 0;
auto incrementer = [counter]() mutable { return ++counter; };
// counter 是 Lambda 内部的副本，每次调用自增
incrementer();  // 返回 1
incrementer();  // 返回 2
// 外部 counter 仍然是 0
```

### Lambda 的底层实现

Lambda 被编译器翻译成一个**匿名类的对象**：
- 捕获的变量 → 类的成员变量
- Lambda 体 → `operator()` 成员函数
- `[]` → 不带任何成员的类
- `[=]` → 所有捕获变量作为成员，拷贝构造
- `[&]` → 所有捕获变量作为引用成员

这就是为什么 Lambda 有独一无二的类型——每个 Lambda 生成不同的匿名类。也是为什么你**必须**用 `auto` 声明 Lambda 变量。

### 无捕获 Lambda 可以转为函数指针

```cpp
int (*fp)(int) = [](int x) { return x * 2; };  // OK: [] → 函数指针
```

这是因为 C++ 标准规定无捕获的 Lambda 可以隐式转为等价的函数指针。一旦有捕获（`[]` 变 `[=]`/`[&]`），这个转换就不可用了——捕获变量需要存储空间，函数指针不够。

---

## 11. 移动语义与完美转发

### 问题：复制大对象的开销

```cpp
CameraFrame frame = make_frame();  // cv::Mat 内部可能有好几 MB 像素数据
queue.push(frame);                 // 又复制一份！
// 现在内存中有三份相同的像素数据
```

移动语义的解决方案：**"窃取"资源而非复制**。

### 左值与右值

- **左值**：有名字、可以取地址的对象。`frame` 是左值。
- **右值**：临时的、即将销毁的对象。`make_frame()` 的返回值是右值。

```cpp
int x = 4;           // x 是左值，4 是右值
int y = x + 1;       // x+1 的结果是右值（临时值）
int &ref = x;        // 左值引用 → 绑定到左值
int &&rref = x + 1;  // 右值引用 → 绑定到右值
```

### `std::move` — 把左值"标记为"右值

```cpp
CameraFrame frame = make_frame();
queue.push(std::move(frame));   // 不走拷贝构造，走移动构造
// frame 现在处于"已移动"状态——内部 cv::Mat 的数据指针已被窃取
// frame 可以重新赋值，但不能直接使用
```

**`std::move` 本质上是一个 cast**：`static_cast<T&&>(x)`。它不移动任何东西——只是改变了值类别，让编译器选择移动重载。

**项目实例** — `armor_thread.cpp:61`：
```cpp
auto *f = camera_buffer->front();       // 获取队列中元素的指针
CameraFrame frame = std::move(*f);      // 移动数据：cv::Mat 内部指针转移，零拷贝
camera_buffer->pop();                   // 弹出槽位
```

**项目实例** — `usbcamera_thread.cpp` 中的 `try_push`：
```cpp
buffer->try_push(std::move(data));      // 转移所有权到队列，线程间零拷贝传输
```

### 移动构造函数和移动赋值

编译器自动生成的移动操作：逐个成员移动。对于 `cv::Mat`，它的移动构造函数只是转移内部数据指针——极快。

### 完美转发（Perfect Forwarding）

有时你需要把参数原样传给另一个函数——保留它是左值还是右值的信息：

```cpp
template <typename T>
void wrapper(T&& arg) {      // T&& 在这里是"转发引用"（以前叫万能引用）
    target(std::forward<T>(arg));  // 左值→左值引用，右值→右值引用
}
```

- `T&&` 在模板上下文中是**转发引用**：如果传入左值，`T` 推导为 `T&`，`T&&` 坍缩为 `T&`；如果传入右值，`T` 推导为 `T`，`T&&` 就是右值引用。
- `std::forward<T>(arg)` 根据 `T` 决定是返回左值引用还是右值引用。

**项目实例** — `SPSCQueue.h:113-130`：
```cpp
template <typename... Args>
void emplace(Args &&...args) noexcept(...) {
    // ...
    new (&slots_[writeIdx + kPadding]) T(std::forward<Args>(args)...);
    // 完美转发所有参数给 T 的构造函数
}
```

调用 `emplace(frame)` → `T(std::forward<CameraFrame>(frame))` → 如果 frame 是左值则拷贝构造，右值则移动构造。自动选择效率最高的方式。

### 移动语义的使用准则

| 场景 | 做法 |
|------|------|
| 把大对象放入容器 | `vec.push_back(std::move(obj))` |
| 线程间传输数据 | SPSCQueue + `std::move` |
| 返回局部变量 | 直接 return，编译器自动移动（NRVO/RVO） |
| 函数参数要存起来 | 按值传参然后 `std::move` |
| **不要**对 const 对象用 `std::move` | 无效果（const 对象不能被窃取） |
| **不要**移动后继续使用 | 对象处于"有效但未指定"状态 |

### RVO/NRVO（返回值优化）

编译器可以在函数返回时省略拷贝/移动——直接在调用者的栈帧上构造对象：

```cpp
CameraFrame make_frame() {
    CameraFrame f;
    f.image = capture();
    return f;   // 不要写 return std::move(f)！会阻止 NRVO
}
```

规则：**返回局部变量时，不要写 `std::move`**。编译器比你聪明——它会做 NRVO，写了 `std::move` 反而禁用 NRVO 强制走移动构造。

---

## 12. std::chrono — 类型安全的时间库

### C vs C++ 时间处理

```c
// C: 手动单位换算，两个整数的组合，容易出错
struct timespec start, end;
clock_gettime(CLOCK_MONOTONIC, &start);
// ... work ...
clock_gettime(CLOCK_MONOTONIC, &end);
long long ns = (end.tv_sec - start.tv_sec) * 1000000000LL
             + (end.tv_nsec - start.tv_nsec);
// 想要毫秒？再除以 1e6
```

```cpp
// C++: 单位是类型的一部分
auto start = std::chrono::steady_clock::now();
// ... work ...
auto end = std::chrono::steady_clock::now();
auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
// ms 一定是毫秒——编译器保证
```

### chrono 的核心组件

1. **时钟（Clock）**：提供 `now()` 和时间点类型
2. **时间点（time_point）**：某个时钟的特定时刻
3. **时长（duration）**：两个时间点之间的差

**项目实例** — `common_def.hpp:7-11`：
```cpp
struct CameraFrame {
    cv::Mat frame;
    std::chrono::steady_clock::time_point timestamp;  // 类型安全的时间戳
};
```

**项目实例** — `armor_shoot.cpp:446-447`：
```cpp
auto now = std::chrono::steady_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
```

### 两种常用时钟

| 时钟 | 特性 | 本项目用途 |
|------|------|-----------|
| `steady_clock` | 单调递增，不受系统时间调整影响 | 测量时间间隔、帧率统计、时间戳 |
| `system_clock` | 挂钟时间，可被 NTP 调整 | 日志时间戳 |

**永远不要用 `system_clock` 测量时间间隔**——如果 NTP 在校时回拨了一秒，你的测量结果会变成负数。

### duration_cast

`duration_cast` 是显式的单位转换——类似于 `static_cast`：

```cpp
auto dur = end - start;                              // 默认单位是纳秒（取决于实现）
auto ms = duration_cast<milliseconds>(dur);          // 截断到毫秒
auto us = duration_cast<microseconds>(dur);          // 截断到微秒
auto s = duration_cast<duration<double>>(dur);       // 浮点秒
auto fps = 1.0 / s.count();                          // 帧率
```

### C++20 chrono 增强（本项目暂未使用）

C++20 大幅增强了 chrono，加入了日历、时区、更直观的字面量：

```cpp
using namespace std::chrono_literals;
auto t = 100ms + 5s;           // 字面量
auto today = 2024y/January/31; // 日历日期（C++20）
```

---

## 13. enum class — 强类型枚举

### C enum 的三个问题

```c
enum Color { RED, BLUE };       // problem 1: RED leaks into global scope
enum Team  { RED_TEAM, BLUE };  // problem 2: force awkward naming to avoid conflict
int x = RED;                     // problem 3: implicit int conversion, no type safety
```

### C++ 方案

```cpp
enum class AimMode { TRACK, HERO_CENTER, COMING };

if (mode == AimMode::TRACK) { ... }     // 必须加作用域，读写都清晰
int x = AimMode::TRACK;                 // 编译错误！不能隐式转整数
int x = static_cast<int>(AimMode::TRACK); // 必须显式转换
```

**项目实例** — `armor_shoot.hpp:19-24`：
```cpp
enum class AimMode {
    TRACK,        // 跟踪模式
    HERO_CENTER,  // 英雄中心模式
    COMING        // 来板模式
};
```

**项目中两种风格并存** — `armor_types.hpp:11-23`：
```cpp
enum EnemyColor { RED, BLUE, PURPLE };   // C 风格（无 class）
enum ArmorType { BIG, SMALL, UNKNOWN };  // 也是 C 风格
```

这说明本项目是逐步从 C 风格迁移的。新代码应使用 `enum class`。

### `enum class` 的额外能力

```cpp
enum class Priority : uint8_t {  // 指定底层类型：只需要 1 字节
    LOW = 1,
    MEDIUM = 2,
    HIGH = 3
};

// C++20: using enum 一次性引入所有枚举值
using enum AimMode;
if (mode == TRACK) { ... }      // 省去 AimMode:: 前缀
```

---

## 14. constexpr — 编译期计算

### 超越 `#define`

C 的编译时常量：
```c
#define LIGHTBAR_LENGTH 0.056      // 宏：无类型，无作用域，只是文本替换
const double LIGHTBAR_LENGTH = 0.056;  // 运行时常量，占用内存
```

C++ `constexpr`：
```cpp
constexpr double LIGHTBAR_LENGTH = 56e-3;  // 编译期确定，有类型，有作用域
```

**项目实例** — `armor_types.hpp:131-133`：
```cpp
constexpr double BIG_ARMOR_WIDTH  = 0.228;
constexpr double BIG_ARMOR_HEIGHT = 0.055;
constexpr double SMALL_ARMOR_WIDTH  = 0.135;
```

### `constexpr` 函数

函数可以在编译期执行：

```cpp
constexpr double rad2deg(double rad) { return rad * 180.0 / M_PI; }
constexpr double angle_deg = rad2deg(0.785);  // 编译期计算，不产生运行时代码
```

### `if constexpr` — 编译期条件分支

已在第 6 章模板中详述。核心价值：不成立的分支不被编译，实现"编译期的 `#if`"。

**项目实例** — `SPSCQueue.h:79-92`：
```cpp
if constexpr (has_allocate_at_least<Allocator>::value) {
    // 仅当分配器支持时编译
} else {
    // 否则编译这个分支
}
```

### `constexpr` vs `const`

```cpp
const int x = read_from_file();  // const 只保证不修改——值可能是运行时的
constexpr int y = 42;            // constexpr 保证编译期可求值
```

所有 `constexpr` 变量隐含 `const`，但反过来不成立。

### `consteval`（C++20）和 `constinit`（C++20）

本项目暂未使用，简要了解：
- `consteval`：必须在编译期执行（比 `constexpr` 更强制）
- `constinit`：变量必须在编译期初始化（防止 static initialization order fiasco）

---

## 15. 异常处理 — 现代 C++ 的错误管理

### 从错误码到异常

C 的经典错误处理模式：
```c
int process(const char *path) {
    FILE *f = fopen(path, "r");
    if (f == NULL) return -1;          // 错误码一层层向上传递

    int ret = do_something(f);
    fclose(f);
    if (ret < 0) return -2;             // 手动传播错误

    return 0;
}
```

问题：(1) 错误码和正常返回值混在同一个通道中，(2) 调用者可能忽略错误码，(3) 深层函数的错误需要每一层都手动传播。

### C++ 异常机制

```cpp
void process(const std::string &path) {
    std::ifstream f(path);          // 构造失败 → 抛异常
    do_something(f);                // 如果 do_something 失败 → 抛异常
    // f 的析构函数自动关闭文件（RAII）
}
```

抛异常时，栈展开（stack unwinding）——所有栈上对象的析构函数按构造逆序被调用。这就是异常安全的基础：RAII。

### 本项目的异常处理模式

**模式 1：异常作为"最后防线"** — `main.cpp:153-157`：
```cpp
catch (const std::exception &e) {
    g_shutdown = true;
    return 1;
}
```

最外层兜底——任何未处理的异常最终会在这里被捕获，防止整个进程崩溃。但只做最小处理（设置退出标志），不做恢复。

**模式 2：YAML 配置加载失败降级** — `armor_track.cpp:30-45`：
```cpp
try {
    YAML::Node config = YAML::LoadFile(track_config_path);
    debug_ = config["auto_aim"]["armor_track"]["armor_track_debug"].as<bool>(false);
    // ...
    MAS_LOG_INFO("armor_track yaml loaded successfully");
}
catch (const std::exception &e) {
    MAS_LOG_WARN("Failed to load config, using defaults: {}", e.what());
}
```

**模式 3** — `armor_shoot.cpp:23-55`：同上的配置加载降级模式。

这里的模式是：**配置加载是可恢复的错误**——用默认值继续运行远好过让程序崩溃。异常提供了统一的错误传播通道，catch 块完成降级处理。

### `noexcept` 规范

`noexcept` 声明函数不抛异常。编译器可以利用它做优化，标准库也依赖它做异常安全决策：

```cpp
// 如果 T 的移动构造函数是 noexcept，vector 扩容时会移动而不是拷贝
void pop() noexcept {
    static_assert(std::is_nothrow_destructible<T>::value,
                  "T must be nothrow destructible");
    // ...
}
```

**项目实例** — `SPSCQueue.h:198-211`：
```cpp
void pop() noexcept { ... }                    // 析构操作不抛异常

// 条件 noexcept：取决于 T 的性质
void push(const T &v) noexcept(std::is_nothrow_copy_constructible<T>::value) { ... }
```

**项目实例** — `armor_track.hpp`：
```cpp
ArmorPose &armor_pose() noexcept { return armor_pose_; }
std::string state() const { return state_; }  // 没有 noexcept——访问 map 可能抛异常
void showResult(...) const noexcept;           // debug 函数不应抛异常
```

### 本项目为什么不广泛使用异常

1. **实时性要求**：视觉处理管线不能因为异常导致帧丢失。异常展开的时间不可预测。
2. **嵌入式遗风**：RM 比赛机器人代码通常关闭异常（`-fno-exceptions`），本项目继承了这个传统。
3. **选择性使用**：只在"不可恢复"（main 兜底）和"可降级"（配置加载）场景使用异常。核心处理管线用错误码/nullopt 返回。

---

## 16. 标准库容器 — 数据结构的选择

masvision 代码大量使用标准库容器。每个容器有不同的性能特征和适用场景。

### `std::vector<T>` — 最常用的容器

#### 是什么

`vector` 本质是**堆上的动态数组**。从 C 的角度看，它等价于一段 `malloc` 的连续内存 + 自动 `realloc` + 自动 `free`。

```c
// C 等价物（手动管理）
typedef struct { int *data; size_t size; size_t capacity; } IntVec;
IntVec v;
v.data = malloc(8 * sizeof(int));  // capacity = 8
v.size = 0;
// ... 每次插入前检查是否需要 realloc ...
// ... 用完必须 free(v.data) ...
```

```cpp
// C++ vector — 自动管理
std::vector<int> v;           // 容量 0，不分配堆内存
v.reserve(8);                 // 一次分配 8 个 int 的空间
v.push_back(42);              // 插入，size 自动增长
// ... 离开作用域自动释放 ...
```

#### 内部三个指针

```cpp
template<typename T>
class vector {
    T* begin_;   // 第一个元素
    T* end_;     // 最后一个元素的下一个位置
    T* cap_;     // 已分配内存的末尾
};
```

```
堆内存布局（capacity=8, size=5）：

      begin_               end_          cap_
        ↓                    ↓             ↓
   ┌───┬───┬───┬───┬───┬───┬───┬───┐
   │ 0 │ 1 │ 2 │ 3 │ 4 │   │   │   │
   └───┴───┴───┴───┴───┴───┴───┴───┘
   ├─────── size()=5 ──────┤
   ├────────── capacity()=8 ──────────┤
```

所有元素在堆上**连续存储**，意味着 `&v[0]` 可以直接当 C 数组用（`v.data()` 返回首地址）。

#### 本项目三种典型用法

**用法 A：数组 + vector — 地形栅格**

`terrain_analysis/src/terrainAnalysis.cpp:85`（MAS Nav）：
```cpp
std::vector<float> planarPointElev[kPlanarVoxelNum];
//   400 个独立的 vector，每个存储该栅格内点云的 Z 坐标
//   这不是 vector<vector<float>>，而是 C 风格数组，元素是 vector
```

每帧循环的典型操作：
```cpp
// ① 每帧开始清空所有栅格（不释放内存——capacity 复用）
for (int i = 0; i < planarVoxelNum; i++)
    planarPointElev[i].clear();

// ② 按坐标分配到对应栅格
planarPointElev[planarVoxelWidth * indX + indY].push_back(point.z);
//   ↑ 只存 Z 坐标（高度），X,Y 由栅格索引隐含 —— 省 3× 内存

// ③ 排序取分位数作为地面高度（连续内存 → 排序极快）
sort(v.begin(), v.end());
int q = v.size() * quantileZ;
ground_height = v[q];
```

**为什么用 `vector<float>` 而非 `vector<Point>`**：地形分析只需高度做分位数，存完整 Point（x,y,z）浪费 3 倍内存。在 400 个栅格 × 每个数百点的情况下，这个优化省掉数十 MB。

**用法 B：vector + struct — 观测数据缓冲区**

`pb_nav2_plugins/src/layers/intensity_voxel_layer.cpp:121`（MAS Nav）：
```cpp
std::vector<Observation> observations;  // Observation = {原点, 点云, 射线...}
getMarkingObservations(observations);   // 函数填充 vector —— 传感器数量在编译期未知

for (const auto & obs : observations)   // range-for 遍历
    process(obs);
```

**用法 C：全局 extern — 跨文件共享的工作缓冲区**

`Point-LIO/src/Estimator.h:15-23`（MAS Nav）：
```cpp
extern std::vector<V3D> pbody_list;         // 去畸变后的点坐标列表
extern std::vector<float> pointSearchSqDis; // 最近邻距离平方
extern std::vector<M3D> crossmat_list;      // 叉积矩阵缓存
```

每帧 `clear()` + `push_back` 数千个点，但从不触发 realloc——capacity 在首次填满后就保持稳定。

**用法 D：简单的点集合**

`armor_types.hpp:33,66`（MAS Vision）：
```cpp
std::vector<cv::Point2f> points;  // 灯条/装甲板 4 个顶点
```

#### push_back 的底层机制

```cpp
// v.push_back(x) 的执行过程：

if (end_ < cap_) {               // ① 还有空位 → 快速路径（1 次赋值 + 1 次指针自增）
    *end_ = x;
    end_++;
} else {                         // ② 空间不足 → 重新分配（触发 malloc + memcpy + free）
    size_t new_cap = (cap_ - begin_) * 2;  // 典型增长因子 = 2×
    T* new_mem = static_cast<T*>(malloc(new_cap * sizeof(T)));
    memcpy(new_mem, begin_, size() * sizeof(T));  // 拷贝旧数据
    free(begin_);                                // 释放旧内存
    begin_ = new_mem;
    end_ = begin_ + old_size;
    cap_ = begin_ + new_cap;
    *end_++ = x;                 // ③ 插入新元素
}
```

**增长因子 = 2×** 是 GCC/MSVC 的默认选择。这意味着每个元素的插入代价平均分摊为 O(1)，但**某一次 push_back 可能触发 O(n) 的 realloc**。

#### reserve 和 clear 的配合

```cpp
v.reserve(1000);   // 一次分配到位，后续 push_back 不触发 realloc
                   //    → 适合 ros2_comm 中预定 buffer 大小的场景

v.clear();         // size = 0，但 capacity 不变
                   //    → 旧数据还在内存中，但逻辑上不可访问
                   //    → 下次 push_back 直接覆盖，不触发 malloc
```

```
clear() 前：  [12, 5, 8, 3, 7, ?, ?, ?]  size=5, capacity=8
clear() 后：  [12, 5, 8, 3, 7, ?, ?, ?]  size=0, capacity=8
               ↑ 旧数据逻辑上不可访问，但还在内存中
push_back(9)： [9,  5, 8, 3, 7, ?, ?, ?]  size=1, capacity=8
               ↑ 覆盖第一个元素，不触发 malloc
```

这就是 terrain_analysis 每帧 `clear()` + `push_back` 几千个点不触发 realloc 的原因——capacity 在第一帧就填满了，之后永远走快速路径。

#### 常用操作速查

| 操作 | 语法 | 复杂度 | 本项目示例 |
|------|------|--------|-----------|
| 尾插 | `v.push_back(x)` | 分摊 O(1) | `planarPointElev[idx].push_back(...)` |
| 原地尾插 | `v.emplace_back(args...)` | 分摊 O(1) | 直接在 vector 内存中构造对象，少一次拷贝 |
| 预分配 | `v.reserve(n)` | O(n) | `m_recv_buffer.reserve(1024)` |
| 清空 | `v.clear()` | O(1)（不释放内存） | 每帧开始清空栅格 |
| 大小 | `v.size()` | O(1) | `if (v.size() > 0)` |
| 随机访问 | `v[i]` | O(1)（不检查边界） | `v[quantileID]` |
| 安全访问 | `v.at(i)` | O(1)（检查边界） | 调试期使用 |
| 排序 | `sort(v.begin(), v.end())` | O(n log n) | 高度排序取分位数 |
| 获取 C 指针 | `v.data()` | O(1) | `memcpy(dst, v.data(), v.size())` |
| 释放内存 | `v.clear(); v.shrink_to_fit()` | O(n) | 确认不再使用的栅格 |

#### 常见坑

**坑 1：push_back 可能使所有指针/引用/迭代器失效**

```cpp
float& ref = v[3];
v.push_back(9.0f);   // ← 如果触发了 realloc，ref 指向已释放的内存！
// ref = 5.0f;       // ← 未定义行为
```

**正确做法**：先 `reserve`，或 push_back 后再取引用。

**坑 2：`[]` 越界不报错**

```cpp
// terrain_analysis 中的保护性写法
int size = planarPointElev[i].size();
if (size > 0) {
    int q = static_cast<int>(quantileZ * size);
    if (q >= size) q = size - 1;       // ← 手动 clamp 到合法索引
    elev = planarPointElev[i][q];      // 现在安全
}
```

**坑 3：`vector<bool>` 是特例**

`vector<bool>` 是位压缩存储（1 bool = 1 bit），不是真正的 `vector`。`&v[0]` 编译不通过。用 `vector<char>` 或 `deque<bool>` 替代。

**坑 4：返回 vector 不会拷贝（C++11 起）**

```cpp
std::vector<int> make_vec() {
    std::vector<int> v = {1, 2, 3};
    return v;   // ← 不会拷贝！编译器用 RVO 或移动语义
}
```

#### 为什么不用 `new[]` / `malloc`

```cpp
// ❌ 手动管理
float* buf = new float[n];
// ... 用完必须 delete[] buf; 否则泄漏
// ... 中间抛异常 → delete 跳过 → 泄漏

// ✅ vector — RAII 自动管理
std::vector<float> buf(n);  // 离开作用域自动释放，异常安全
```

对于实时系统（如 mas_nav 的 500μs 通信循环），vector 的 RAII 保证在任何退出路径下释放内存。手动 `new`/`delete` 在异常/提前 return/break 下的泄漏风险是比赛场景不可接受的。

### `std::map<K,V>` — 有序键值对

基于红黑树：O(log n) 查找、插入、删除。键按 `<` 排序。

**项目实例** — `armor_track.hpp:95-97`：
```cpp
mutable std::map<std::string, double> fps_map_;
mutable std::map<std::string, int> count_map_;
mutable std::map<std::string, std::chrono::steady_clock::time_point> last_time_map_;
```

这里用 `map` 而非 `unordered_map`，因为键（窗口名）很少（通常 1-3 个），红黑树和哈希表在这种规模下性能差异可忽略，而红黑树能保持键的排序。

**项目实例** — `ekf.hpp:38`：
```cpp
std::map<std::string, double> data;  // 卡方检验统计数据
```

### `std::unordered_map<K,V>` — 无序键值对

基于哈希表：O(1) 平均查找。键不需要是可比较的（不需要 `operator<`），只需要可哈希的。

**项目实例** — `display.hpp:145-148`：
```cpp
std::unordered_map<std::string, std::unique_ptr<rigtorp::SPSCQueue<DisplayTask>>> window_queues_;
std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_update_times_;
std::unordered_map<std::string, SDLContext> sdl_contexts_;
```

窗口名到窗口数据——典型的 key-value 查询场景。用 `unordered_map` 因为不需要排序，且插入/查找频率不高。

### `std::deque<T>` — 双端队列

分段连续数组：头部和尾部插入/删除 O(1)，随机访问 O(1)，遍历不如 `vector` 快（内存不连续）。

**项目实例** — `extended_kalman_filter.hpp:39`：
```cpp
std::deque<int> recent_nis_failures{0};
```

EKF 中用 `deque` 存储最近 N 个 NIS（归一化创新平方）失败记录。原因：需要头部删除（窗口滑动）+ 尾部追加，`vector` 的头部删除是 O(n)，`deque` 是 O(1)。

### `std::string` — 替代 `char*`

`std::string` 管理自己的内存，自动增长，支持 `+` 拼接、`==` 比较。绝不会缓冲区溢出。

**项目实例** — 遍布全项目：
```cpp
std::string state_;     // armor_track.hpp:69
std::string number;     // armor_types.hpp:73
std::string window_name // armor_track.hpp:47
```

与 C `char*` 对照：
```cpp
// C: 手动管理内存
char *concat(const char *a, const char *b) {
    char *result = malloc(strlen(a) + strlen(b) + 1);
    strcpy(result, a);
    strcat(result, b);
    return result;  // 调用者必须 free
}

// C++: string 管理一切
std::string result = std::string(a) + b;  // 自动分配、自动释放
```

### 容器选择决策树

```
需要存储多个元素？
├─ 需要序号索引（"第 i 个"）？
│  └─ std::vector<T> — 默认选择
│     例外：频繁头部插入/删除 → std::deque<T>
├─ 需要按键查找（"key 为 K 的值"）？
│  ├─ 需要排序遍历 → std::map<K,V>
│  └─ 不需要排序 → std::unordered_map<K,V>
└─ 只需要表达"有"或"没有" → std::set<T> 或 std::unordered_set<T>
```

---

## 17. std::function — 类型擦除的可调用对象

### 存储任意可调用对象

C 中只有函数指针——只能指向自由函数或静态函数：
```c
int (*callback)(int);  // 只能指向签名匹配的函数
```

C++ 的 `std::function` 可以存储**任何可调用对象**：函数、Lambda、函数对象（带 `operator()` 的类）、成员函数（通过 `std::bind`）。

```cpp
#include <functional>

std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract;
```

**项目实例** — `extended_kalman_filter.hpp:21-22`：
```cpp
ExtendedKalmanFilter(
    const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add =
        [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) { return a + b; });
```

这里 `x_add` 参数可以是任何签名匹配的可调用对象。默认值是一个 Lambda——但调用者也可以传入自定义的函数对象。

**项目实例** — `ekf.hpp:30-31`：
```cpp
Eigen::VectorXd update(..., 
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
        [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) { return a - b; });
```

### `std::function` 的性能代价

`std::function` 使用**类型擦除**——内部通过虚函数（或类似机制）间接调用。这意味着：
- 有一次间接调用的开销（≈虚函数调用的开销）
- 可能触发堆分配（如果可调用对象较大，无法放入内部小缓冲区）

在性能敏感的热路径（如每帧调用的函数），优先使用模板——模板没有运行时开销：

```cpp
// 热路径：用模板（零开销，但每个类型生成一份代码）
template<typename F>
void process(F &&func) { func(); }

// 冷路径或存储：用 std::function（有开销，但灵活）
void register_callback(std::function<void()> func) { cb_ = std::move(func); }
```

---

## 18. 线程与并发 — 从 pthread 到 std::thread

### C vs C++ 线程

```c
// C: pthread（POSIX，不可移植到 Windows）
#include <pthread.h>
pthread_t thread;
pthread_create(&thread, NULL, thread_func, arg);
pthread_join(thread, NULL);
```

```cpp
// C++: std::thread 跨平台
#include <thread>
std::thread t(thread_func, arg1, arg2);  // 可变参数，自动转发
t.join();  // 等待线程结束
```

### 本项目使用的并发工具

本项目通过 `BS::thread_pool`（第三方线程池）间接使用线程——不直接创建 `std::thread`：

**项目实例** — `main.cpp:63,86-87`：
```cpp
BS::thread_pool pool(6);      // 6 个工作线程

auto camera_future = pool.submit_task(
    [camera_buffer_size, camera_buffer]() {
        threads::camera_thread_func("config/hikcamera.yaml", camera_buffer_size, camera_buffer);
    });
```

`submit_task` 返回 `std::future<void>`——一个表示"将来会有结果"的对象。

### `std::future` / `std::shared_future`

- `std::future<T>`：独占的异步结果句柄。只能 `get()` 一次。
- `std::shared_future<T>`：共享的异步结果句柄。可以多次 `get()`，可以被拷贝。

**项目实例** — `main.cpp:86,91,95,100,106`：
```cpp
auto camera_future = pool.submit_task([...]{ ... });  // future<void> — 不需要返回值，只要等待

std::shared_future<void> usb_camera_future;  // shared — 可能被多个地方等待
std::shared_future<void> ros2_future;
```

**项目实例** — `main.cpp:130-148` — 等待所有线程结束：
```cpp
if (camera_future.valid()) {
    camera_future.get();  // 阻塞直到线程完成
}
```

### `std::this_thread`

`<thread>` 头提供当前线程操作：

**项目实例** — `main.cpp:124`：
```cpp
std::this_thread::sleep_for(std::chrono::milliseconds(100));
```

**项目实例** — `armor_thread.cpp:52`：
```cpp
std::this_thread::sleep_for(std::chrono::milliseconds(100));
```

注意：C 的 `sleep()`/`usleep()` 是 POSIX 函数，在 Windows 上不可用。`std::this_thread::sleep_for` 跨平台且类型安全（chrono 类型）。

### `std::mutex` / `std::lock_guard`

**项目实例** — `display.hpp:150`：
```cpp
std::mutex queues_mutex_;  // 保护 window_queues_ 的并发访问
```

`std::lock_guard` 是 RAII 锁——构造时加锁，析构时解锁。绝不会忘记解锁：

```cpp
void Display::display_add(...) {
    std::lock_guard<std::mutex> lock(queues_mutex_);  // lock
    // ... 安全操作共享数据 ...
}  // unlock（自动）
```

### 线程安全初始化 — `static` 局部变量

C++11 保证：多线程并发执行到 `static` 局部变量初始化时，只有一个线程执行初始化：

```cpp
static UserSerial &getInstance() {
    static UserSerial instance(config_path);  // 多线程安全，只初始化一次
    return instance;
}
```

这比"双重检查锁定"（double-checked locking）模式更简单、更正确。

---

## 19. static_assert — 编译期断言

### 与 `assert()` 的区别

| | `assert()` | `static_assert` |
|---|---|---|
| 时机 | 运行时 | 编译期 |
| 参数 | 运行时表达式 | 编译期常量表达式 |
| 失败 | 程序崩溃 | 编译失败（含错误消息） |
| 开销 | Debug 模式有开销 | 零运行时开销 |

```c
// C: assert 在运行时
assert(ptr != NULL);  // 条件不满足 → 崩溃

// C++: static_assert 在编译时
static_assert(sizeof(int) == 4, "int must be 4 bytes");  // 条件不满足 → 编译错误
```

### 本项目用法

**项目实例** — `SPSCQueue.h`：
```cpp
// 编译期检查模板参数
static_assert(std::is_constructible<T, Args &&...>::value,
              "T must be constructible with Args&&...");

// 编译期检查内存布局
static_assert(alignof(SPSCQueue<T>) == kCacheLineSize, "");
static_assert(sizeof(SPSCQueue<T>) >= 3 * kCacheLineSize, "");

// 编译期检查析构函数
static_assert(std::is_nothrow_destructible<T>::value,
              "T must be nothrow destructible");
```

`static_assert` 配合 type traits 是模板编程的标准组合——在编译期就拒绝不正确的使用，而不是等到运行时崩溃或出现古怪行为。

---

## 20. placement new — 在已有内存上构造对象

### 为什么需要 placement new

C 中，`malloc` 返回一块原始内存，你可以直接往里面写 POD（平凡可拷贝）类型。但在 C++ 中，对象的生命周期**始于构造函数调用**——光分配内存不够，必须调用构造函数。

`new (ptr) T(args...)` = 在 `ptr` 指向的已分配内存上调用 `T` 的构造函数。**不分配新内存。**

### 本项目的应用：SPSCQueue 的环形缓冲区

SPSCQueue 预分配一块大内存，然后在其中按需构造/析构元素：

**项目实例** — `SPSCQueue.h:128`：
```cpp
// 构造（placement new）
new (&slots_[writeIdx + kPadding]) T(std::forward<Args>(args)...);
```

**项目实例** — `SPSCQueue.h:204`：
```cpp
// 析构（手动调用析构函数）
slots_[readIdx + kPadding].~T();
```

完整流程：
```
1. allocate(n * sizeof(T) + padding)     → 分配原始内存
2. new (slot + i) T(...)                 → 构造对象（placement new）
3. use the object                       → 使用
4. (slot + i)->~T()                      → 手动析构
5. deallocate(slots)                     → 释放原始内存
```

这避免了为队列中每个槽位预先构造 `T`（默认为空），而是按需构造——只在元素被 push 时才调用 `T` 的构造函数。

### 关键理解

- `new (addr) T(...)` 不分配内存——只在 addr 处调用构造函数
- `ptr->~T()` 不释放内存——只调用析构函数
- 构造函数和内存分配是两个独立操作；析构函数和内存释放也是
- placement new 是实现 `std::vector`、`std::optional`、`std::variant` 等容器的基础

---

## 21. 缓存行与内存布局

### 伪共享（False Sharing）问题

两个 CPU 核心各自频繁写**不同**变量，但它们恰好在同一缓存行（64 字节），缓存一致性协议会导致两个核心的缓存行不断失效→重载。这两个变量虽然逻辑上无关，但物理上"共享"了缓存行——故称**伪**共享。

### SPSCQueue 的解决方案

**项目实例** — `SPSCQueue.h:237-257`：
```cpp
static constexpr size_t kCacheLineSize = 64;

alignas(kCacheLineSize) std::atomic<size_t> writeIdx_ = {0};       // 缓存行 1
alignas(kCacheLineSize) size_t readIdxCache_          = 0;         // 缓存行 2
alignas(kCacheLineSize) std::atomic<size_t> readIdx_  = {0};       // 缓存行 3
alignas(kCacheLineSize) size_t writeIdxCache_         = 0;         // 缓存行 4
```

每个热点变量独占一个缓存行——即使浪费了一些内存（填充字节），但避免了跨核心的缓存一致性流量，延迟显著降低。

### `alignas` 和 `alignof`

- `alignas(N)`：指定变量的对齐要求（N 必须是 2 的幂）
- `alignof(T)`：查询 T 的默认对齐（类似 C11 的 `_Alignof`）

```cpp
static_assert(alignof(SPSCQueue<T>) == kCacheLineSize, "");
// SPSCQueue 对象按缓存行对齐，避免和相邻对象产生伪共享
```

### C++17 的 `hardware_destructive_interference_size`

标准库提供了可移植的缓存行大小（SPSCQueue 为了兼容性自己定义了 `kCacheLineSize = 64`）：

```cpp
// C++17 推荐写法（本项目未使用）
alignas(std::hardware_destructive_interference_size) std::atomic<size_t> writeIdx_;
```

---

## 22. C++ 属性 — 编译器的注释

属性（Attributes）为编译器提供额外信息，非侵入式，不影响程序的语义。

### `[[nodiscard]]` — "返回值不应被忽略"

**项目实例** — `SPSCQueue.h`：
```cpp
RIGTORP_NODISCARD bool try_push(P &&v) noexcept(...);  // [[nodiscard]]
RIGTORP_NODISCARD T *front() noexcept;
RIGTORP_NODISCARD size_t size() const noexcept;
```

```cpp
queue.try_push(frame);  // 警告：忽略了 try_push 的返回值（可能 push 失败！）
// 应该：
if (!queue.try_push(frame)) { /* 队列满，处理 */ }
```

`SPSCQueue.h:33-40` 通过宏做兼容处理（老编译器可能不支持 `[[nodiscard]]`）：
```cpp
#ifdef __has_cpp_attribute
#if __has_cpp_attribute(nodiscard)
#define RIGTORP_NODISCARD [[nodiscard]]
#endif
#endif
```

### `[[no_unique_address]]` — 优化空类型的存储

**项目实例** — `SPSCQueue.h:246-248`：
```cpp
#if defined(__has_cpp_attribute) && __has_cpp_attribute(no_unique_address)
    Allocator allocator_ [[no_unique_address]];  // 空分配器不占空间
#else
    Allocator allocator_;
#endif
```

`std::allocator<T>` 是无状态（stateless）的——没有成员变量。但按照 C++ 规则，每个对象必须有唯一地址，所以即使空对象也要占 1 字节。`[[no_unique_address]]` 允许编译器让空成员不占空间——和外部变量共享地址。

### `[[maybe_unused]]` — 抑制"未使用"警告

```cpp
[[maybe_unused]] int debug_only = compute_expensive_debug_info();
// 只在非 DEBUG 模式下使用 → 发布编译不会产生"变量未使用"警告
```

---

## 23. 默认成员初始化器

### 语法和意义

C++11 起，可以在类定义中直接给成员变量默认值：

```cpp
class UserSerial {
    std::atomic<bool> isConnected{false};         // 默认值
    std::atomic<bool> has_received_data_{false};  // 默认值
    // 所有构造函数都会使用这些默认值（除非初始化列表显式覆盖）
};
```

**项目实例** — `display.hpp:64-84`：
```cpp
struct SDLContext {
    SDL_Window *window = nullptr;
    SDL_Renderer *renderer = nullptr;
    SDL_Texture *texture = nullptr;
    int width = 0;
    int height = 0;
    bool closed = false;
    static constexpr size_t MAX_TEXT_CACHE = 50;
};
```

**项目实例** — `armor_track.hpp:63-70`：
```cpp
bool debug_ = false;
int min_detect_count_ = 5;
int max_temp_lost_count_ = 3;
// ...
std::string state_{"lost"}, pre_state_{"lost"};
```

### 相比在构造函数中初始化的优势

1. **声明即文档**：默认值就在类型定义旁边，不需要翻到构造函数看
2. **减少重复**：多个构造函数共享同一默认值，不需要每个都写一遍
3. **防止遗漏**：新加成员不会忘记初始化

当所有成员都有默认初始化器时，`= default` 的默认构造函数就够了——不需要手写。

---

## 24. static 关键字在 C++ 中的多重身份

`static` 在 C++ 中有四种不同含义——取决于使用上下文。

### 1. `static` 全局变量/函数（文件作用域）

```cpp
static int internal = 0;     // 只在当前翻译单元可见
static void helper() { ... } // 同上
```

C++ 推荐用**匿名命名空间**替代（因为 static 不能用于类型）：
```cpp
namespace {
    int internal = 0;
    void helper() { ... }
    struct InternalType { ... };  // static 做不到这个
}
```

### 2. `static` 局部变量

```cpp
void count_calls() {
    static int count = 0;  // 函数第一次调用时初始化一次，之后保持值
    count++;
}
```

**C++11 的关键保证**：多线程下，`static` 局部变量的初始化是**线程安全**的——只有一个线程执行初始化，其余等待。这直接催生了 Meyer's Singleton 模式。

**项目实例** — `display.hpp:101-105`：
```cpp
static Display &getInstance() {
    static Display instance;   // 线程安全的懒初始化
    return instance;
}
```

**项目实例** — `user_serial.hpp:21-25`：
```cpp
static UserSerial &getInstance(const std::string &config_path) {
    static UserSerial instance(config_path);
    return instance;
}
```

### 3. `static` 类成员变量

属于**类**而非对象——所有对象共享同一份：

**项目实例** — `mas_log.hpp:48`：
```cpp
static std::shared_ptr<spdlog::logger> logger_;  // 全局唯一的 logger
```

必须在类外定义（通常 .cpp 中）：
```cpp
// mas_log.cpp
std::shared_ptr<spdlog::logger> MasLog::logger_ = nullptr;
```

### 4. `static` 类成员函数

不依赖对象——没有 `this` 指针，只能访问 static 成员：

**项目实例** — `mas_log.hpp:34-35`：
```cpp
static void init(const std::string &log_path, ...);
static std::shared_ptr<spdlog::logger> get_logger() { return logger_; }
```

调用方式：`MasLog::init(...)` — 不需要 `MasLog` 对象。

---

## 25. 第三方库的 C++ 使用模式

masvision 项目重度依赖四个主要 C++ 库。理解它们的使用模式有助于整体理解代码。

### Eigen — 线性代数库

Eigen 是纯头文件库，提供矩阵、向量、四元数等数学类型。核心特性是**表达式模板**——`a + b * c` 这样的表达式在编译期展开成最优代码，不需要创建中间临时对象。

**项目实例** — `armor_types.hpp:77-81`：
```cpp
Eigen::Vector3d xyz_in_gimbal;   // 3D double 向量
Eigen::Vector3d ypr_in_gimbal;   // yaw-pitch-roll
Eigen::Vector3d xyz_in_world;
Eigen::Vector3d ypr_in_world;
Eigen::Vector3d ypd_in_world;    // 球坐标
```

**项目实例** — `armor_shoot.cpp:63-66`：
```cpp
Eigen::Vector3d gimbal_euler = rm_utils::eulers(R_gimbal2world, 2, 1, 0);
double gimbal_yaw = gimbal_euler[0];
double gimbal_pitch = gimbal_euler[1];
```

**项目实例** — `user_serial.hpp:70`：
```cpp
Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);
```

Eigen 与 C++ 标准库的关键区别：
- 不使用 STL 容器——有自己的内存布局和分配策略
- 固定大小对象（`Vector3d`）在栈上分配（无堆开销）
- 动态大小对象（`MatrixXd`）在堆上分配
- 与 OpenCV 交互需要手动转换（`cv::Mat` ↔ `Eigen::Matrix`）

### OpenCV — 计算机视觉库

OpenCV 的 C++ API 大量使用引用计数（`cv::Mat` 是引用计数的智能资源）和 RAII。

**项目实例** — `armor_types.hpp:32-35`：
```cpp
cv::Point2f center, top, bottom, top2bottom;
std::vector<cv::Point2f> points;
cv::RotatedRect rotated_rect;
```

`cv::Mat` 的关键特性：
```cpp
cv::Mat a = b;         // 浅拷贝！a 和 b 共享同一像素数据（引用计数 +1）
cv::Mat c = b.clone(); // 深拷贝：生成独立的像素数据副本
```

OpenCV 大量使用默认参数和 `InputArray`/`OutputArray` 代理类型来提供灵活的接口。

### YAML-cpp — 配置文件解析

本项目的配置管理统一使用 YAML-cpp（除 ros2_comm 用 nlohmann/json）：

**项目实例** — `armor_shoot.cpp:25-28`：
```cpp
YAML::Node config = YAML::LoadFile(config_path);
auto node = config["auto_aim"]["armor_shoot"];
yaw_offset_ = node["yaw_offset"].as<double>(0.0) / 57.3;
```

`as<T>(default_value)` 模式：类型安全 + 缺省值兜底。如果键不存在或类型不匹配，返回默认值而不是抛异常。

YAML-cpp 的 `Node` 类型是动态类型的——`node["key"]` 返回的结果可以是标量、序列或映射，具体由 YAML 内容决定。这与 C++ 的静态类型形成对比。

### nlohmann/json — JSON 操作

用于 Plotter 数据序列化（调试和可视化输出）：

**项目实例** — `armor_shoot.cpp:187-207`：
```cpp
nlohmann::json data;
data["send_packet"]["found"] = 1;
data["send_packet"]["fire_advice"] = fire;
// ...
plotter_->plot(data);
```

nlohmann/json 用运算符重载实现了类似 Python dict 的语法。`data["a"]["b"] = 42` 在 JSON 对象上创建嵌套结构。

### spdlog — 日志库

**项目实例** — `mas_log.hpp:9-20`：
```cpp
#define MAS_LOG_INFO(...) SPDLOG_LOGGER_CALL(rm_utils::MasLog::get_logger(), spdlog::level::info, __VA_ARGS__)
```

spdlog 基于格式化库 `fmt`，支持 Python 风格的格式化语法：
```cpp
MAS_LOG_INFO("Thread pool created with {} threads", pool.get_thread_count());
//           占位符 {} 被后续参数替换 ↑
```

spdlog 是头文件库，线程安全，异步日志可选。本项目中用于所有模块的日志输出。

---

## 附录 A：C → C++ 完整对照词典

| C 语法/概念 | C++ 等价物 | 出现位置 |
|-------------|-----------|---------|
| `malloc` / `free` | `new` / `delete` 或 `make_unique`/`make_shared` | 全项目 |
| `realloc` | `std::vector` 自动扩容 | `armor_types.hpp` |
| `typedef` | `using X = Y` | `BS_thread_pool.hpp` |
| `#define CONSTANT` | `constexpr` | `armor_types.hpp:131` |
| `NULL` | `nullptr` | `mas_log.cpp:16` |
| `void*` 泛型 | `template<typename T>` | `SPSCQueue.h:45` |
| 函数指针 | Lambda 或 `std::function` | `main.cpp`, `ekf.hpp` |
| `struct { ... }` (纯数据) | `class { ... }` (数据+方法) | 全项目 |
| `_Atomic bool` | `std::atomic<bool>` | `main.cpp:24` |
| `clock_gettime()` | `std::chrono::steady_clock` | `armor_shoot.cpp:446` |
| `enum` (全局污染) | `enum class` (作用域限定) | `armor_shoot.hpp:19` |
| `struct foo*` (输出参数) | `foo&` (引用) 或返回值 | 全项目 |
| `goto cleanup` | RAII 析构函数 | `SPSCQueue.h:100` |
| `#if DEBUG` | `if constexpr` 或运行时 `if` | `SPSCQueue.h:79` |
| `strcpy` / `memcpy` | `std::string` / `std::vector` | `auto_aim/` |
| `pthread_create` | `BS::thread_pool` / `std::thread` | `main.cpp:63` |
| `pthread_mutex_lock` | `std::lock_guard<std::mutex>` | `display.hpp:150` |
| `char*` (字符串) | `std::string` | 全项目 |
| 哨兵值 (`-1` = 没找到) | `std::optional<T>` | `armor_track.hpp:47` |
| `assert()` | `static_assert` (编译期) | `SPSCQueue.h:94` |
| 手动引用计数 | `std::shared_ptr<T>` | `main.cpp:83` |
| `memcpy` 转移所有权 | `std::move` + 移动语义 | `armor_thread.cpp:61` |
| `#error` | `static_assert(false, "...")` | — |
| `__attribute__((aligned(N)))` | `alignas(N)` | `SPSCQueue.h:254` |
| 文件作用域 `static` | 匿名 `namespace { }` | — |
| `pthread_once` | `static` 局部变量（C++11 线程安全） | `display.hpp:103` |
| 信号量/条件变量 | `std::condition_variable` | （BS_thread_pool 内部） |
| `volatile` (并发用) | `std::atomic` | `main.cpp:24` |

---

## 附录 B：masvision C++ 特性代码索引

快速定位每个 C++ 特性在项目中的出现位置。

| C++ 特性 | 文件 | 行号/位置 |
|-----------|------|----------|
| `auto` | `armor_types.hpp` | 110-113 |
| `auto` + 移动 | `armor_thread.cpp` | 59-61 |
| range-for | `armor_track.cpp` | 282, 291 |
| `nullptr` | `mas_log.cpp` | 16 |
| `using` 别名 | `display.hpp` | 88 |
| 统一初始化 `{}` | `armor_types.hpp` | 135-142 |
| `decltype` | `SPSCQueue.h` | 56 |
| 左值引用 `T&` | `armor_detector.hpp` | 18 |
| 返回值引用 | `armor_track.hpp` | 31 |
| 命名空间定义 | `armor_types.hpp` | 9, 144 |
| 命名空间使用 | `main.cpp` | 67, 87 |
| RAII 析构 | `SPSCQueue.h` | 100-107 |
| RAII `lock_guard` | `display.hpp` | 150 |
| `= delete` | `SPSCQueue.h` | 110-111 |
| class 定义 | `armor_track.hpp` | 16-101 |
| struct 定义 | `armor_types.hpp` | 29-58 |
| 构造函数 + 初始化列表 | `armor_types.hpp` | 87 |
| 初始化列表（类） | `armor_track.cpp` | 24-26 |
| `const` 成员函数 | `armor_track.hpp` | 37 |
| `mutable` | `armor_track.hpp` | 95-97 |
| `static` 类成员 | `mas_log.hpp` | 48 |
| 单例模式 | `display.hpp` | 101-105 |
| 单例模式 | `user_serial.hpp` | 21-25 |
| 类模板 | `SPSCQueue.h` | 45-258 |
| 变参模板 | `SPSCQueue.h` | 113-115 |
| SFINAE / `enable_if` | `SPSCQueue.h` | 163-168 |
| type traits | `SPSCQueue.h` | 115, 159 |
| `if constexpr` | `SPSCQueue.h` | 79-92 |
| `std::atomic` | `main.cpp` | 24 |
| 内存序 `acquire`/`release` | `SPSCQueue.h` | 126, 129 |
| `alignas` | `SPSCQueue.h` | 254-257 |
| `unique_ptr` | `armor_detector.hpp` | 96 |
| `shared_ptr` | `main.cpp` | 83 |
| `make_shared` | `main.cpp` | 83 |
| `make_unique` | `armor_shoot.cpp` | 47 |
| `std::optional` 返回 | `armor_track.hpp` | 47 |
| `std::optional` 参数 | `armor_shoot.hpp` | 59 |
| `std::nullopt` | `armor_track.cpp` | 164 |
| Lambda `[]` | `armor_types.hpp` | 43 |
| Lambda `[&]` | `armor_track.cpp` | 71, 78 |
| Lambda 按值捕获 | `main.cpp` | 87, 103 |
| `std::move` | `armor_thread.cpp` | 61 |
| `std::forward` | `SPSCQueue.h` | 128 |
| `std::chrono::steady_clock` | `common_def.hpp` | 10 |
| `duration_cast` | `armor_shoot.cpp` | 447 |
| `enum class` | `armor_shoot.hpp` | 19-24 |
| C 风格 `enum` | `armor_types.hpp` | 11-16 |
| `constexpr` 常量 | `armor_types.hpp` | 131-133 |
| `constexpr` + `static` | `SPSCQueue.h` | 237-238 |
| `try`/`catch` | `main.cpp` | 60, 153 |
| `try`/`catch` + 降级 | `armor_track.cpp` | 30-45 |
| `noexcept` | `SPSCQueue.h` | 115 |
| `noexcept` + type trait | `SPSCQueue.h` | 198 |
| `std::vector` | `armor_types.hpp` | 33, 66 |
| `std::map` | `armor_track.hpp` | 95-97 |
| `std::unordered_map` | `display.hpp` | 145-148 |
| `std::deque` | `extended_kalman_filter.hpp` | 39 |
| `std::string` | 全项目 | — |
| `std::function` | `extended_kalman_filter.hpp` | 21-22 |
| `std::thread` (via pool) | `main.cpp` | 63 |
| `std::future` | `main.cpp` | 86, 130-148 |
| `std::shared_future` | `main.cpp` | 91, 106 |
| `std::mutex` | `display.hpp` | 150 |
| `std::this_thread::sleep_for` | `main.cpp` | 124 |
| `static_assert` | `SPSCQueue.h` | 94-95, 116-117 |
| placement new | `SPSCQueue.h` | 128 |
| 手动析构 `~T()` | `SPSCQueue.h` | 204 |
| `[[nodiscard]]` | `SPSCQueue.h` | 33-40 (宏) |
| `[[no_unique_address]]` | `SPSCQueue.h` | 246 |
| 默认成员初始化器 | `display.hpp` | 64-84 |
| 默认成员初始化器 | `user_serial.hpp` | 95, 124 |
| 宏 + `__VA_ARGS__` | `mas_log.hpp` | 9-20 |
| Eigen Vector3d | `armor_types.hpp` | 77-81 |
| Eigen Quaterniond | `user_serial.hpp` | 70 |
| Eigen Matrix3d | `armor_shoot.hpp` | 59 |
| OpenCV Mat/Point2f | `armor_types.hpp` | 32-35 |
| YAML-cpp | `armor_track.cpp` | 33 |
| nlohmann/json | `armor_track.cpp` | 133 |
| spdlog 宏 | `mas_log.hpp` | 9-20 |

---

## 附录 C：常见编译错误速查

| 错误信息 | 常见原因 | 解决思路 |
|----------|----------|----------|
| `undefined reference to 'vtable for X'` | 声明了虚函数但没定义（特别是虚析构函数） | 给第一个非内联虚函数提供定义，或给虚析构加 `= default` |
| `error: 'X' does not name a type` | 缺少头文件或命名空间 | 检查 `#include` 和 `using` |
| `error: no matching function for call to 'push_back'` | 尝试将右值放入期望左值的容器 | 用 `emplace_back` 或 `std::move` |
| `error: use of deleted function` | 试图拷贝不可拷贝的对象 | 改为移动（`std::move`）或传引用 |
| `error: invalid use of incomplete type` | 只做了前向声明就使用类型 | 包含完整的头文件 |
| 模板错误灾难（几百行错误） | 用错误类型实例化模板 | 读第一条错误，跳过后续候选列表 |
| `error: call of overloaded 'f(nullptr)' is ambiguous` | NULL vs nullptr 歧义 | 用 `nullptr` 替代 `NULL` |
| `error: narrowing conversion from 'double' to 'int'` | `{}` 初始化不允许窄化转换 | 用显式 `static_cast` 或改用 `()` 初始化 |
| `error: 'auto' requires an initializer` | `auto` 声明必须有初始化表达式 | 提供初始化或给出显式类型 |
| `warning: ignoring return value of 'try_push'` | 忽略了 `[[nodiscard]]` 的返回值 | 检查并使用返回值 |
| `error: static assertion failed: T must be nothrow destructible` | `static_assert` 触发 | 确保 T 的析构函数是 `noexcept` |
| `ld: duplicate symbol` | 头文件中定义了非内联函数 | 加 `inline` 或移到 .cpp 文件 |

---

> **最后建议**：这份文档应该配合 [[MAS_Vision_学习指南]] 一起阅读——前者讲解"这个项目用了什么 C++ 技术"，后者讲解"这个项目是怎样工作的"。遇到看不懂的 C++ 语法，先查附录 B 确认在代码中的位置，再回到对应章节理解原理。引用 Bjarne Stroustrup 的话："C++ 的设计目标不是让简单的事情更简单，而是让复杂的事情变得可能。"——本项目中的 SPSCQueue 和 EKF 就是最好的例证。
