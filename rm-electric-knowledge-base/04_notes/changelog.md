# 知识库更新日志

## 2026-07-11

- 搭建四层架构目录结构（00_raw / 01_extracted / 02_code_twin / 03_moc / 04_notes）
- 将现有散落资料归档至 `00_raw/` 对应子目录
- 编写代码孪生笔记模板、README
- 提炼电机参数文档（4 种电机 14 个实例，对照 7 份 PDF 标注出处）

## 2026-07-14（一）

- 统一 02 层所有代码孪生笔记的 wikilink 格式：代码块内链接移到代码块下方列表
- 统一标题破折号为短横线 `-`，修复 Obsidian 跳转不匹配问题
- 删除 `launch-json.md`（项目未用 JLink，使用外部 OpenOCD 烧录）
- `04_notes/env_pitfalls.md` 追加第 3 节（build 目录误删）和第 4 节（CubeMX 重新生成代码差异）

## 2026-07-14（二）

- **重写 `01_extracted/gcc-cmake-build.md`**：
  - 每个命令统一格式：在干什么 → 签名 + 参数表 → 项目实例 + 逐行对应
  - add_library 拆成 OBJECT / STATIC / INTERFACE 三个子节，讲完后统一对比
  - 删除 ALIAS 子节、第八节"实战连招"、等价 GCC 命令翻译
  - 新增第五节"作用域与 PRIVATE / PUBLIC / INTERFACE"
  - 01 层不链接到 02 层，项目实例只贴代码不标来源
- **更新 02 层链接锚点**：板级、stm32cubemx、utils、bsp、robot、modules、apps 全部更新
- **精简 02 层**：utils 保留首次出现内容，bsp/robot 精简为"与 utils 的差异"
- **调整 `tasks-json.md` 结构**：五阶段流水线移到 PowerShell 命令正下方

## 2026-07-14（三）

- **清理反向链接**：删除 02 层文件中指向 `04_notes/env_pitfalls` 的 3 处链接
- **修复 em dash**：cmake-basic-syntax.md 三个标题统一为短横线
- **修复格式**：config-cmake.md、modules CMakeLists-txt.md 链接格式
- **重写 03 层**：新建 `CMake-build-walkthrough.md`，只保留阅读顺序、全流程概览、跳转深度图
- **精简 03 层**：删除与 02 层重复的详细拆解内容

## 2026-07-14（四）

- **重写 README**：精简为四层架构说明 + Obsidian 使用方式

## 2026-07-14（五）

- **01 层新增**：
  - `stm32-can-filter.md`：结合 STM32F407 芯片手册提炼 CAN 过滤器配置规则（28 个 filter bank、双 CAN 分配、16 位尺度位映射、`<<5` 位移原因、掩码模式精确匹配）
  - `kfifo-design.md`：kfifo 无锁环形缓冲区设计原理（SPSC 模型、in/out 持续递增、mask 位与、`__DMB()` 内存屏障、回绕拷贝）
- **02 层新增**：
  - `board/bsp/CAN/bsp_can.md`：CAN 硬件抽象层逐段解析（设备注册、过滤器配置、HAL 回调、发送逻辑）
  - `board/bsp/CAN/bsp_can_task.md`：CAN 收发后台任务逐段解析（RX/TX Task、信号量驱动、三层缓冲）
  - `utils/kfifo/kfifo.md`：kfifo 实现逐段解析（init、put/get、内存屏障、覆盖写）
- **00 层整理**：reference-note/bsp/bsp代码.md 为旧版代码注释（链表+事件标志组），当前代码已改为数组查找表+信号量，标注"仅供 filter 配置方法参考"
