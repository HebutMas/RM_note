---
aliases:
  - RM-GitHub教程
  - GitHub使用指南
tags:
  - robomaster
  - github
  - tutorial
  - team/robomaster
created: 2026-06-12
updated: 2026-06-12
status: draft
---


# 🏆 山海机甲 GitHub 使用教程

> **适用对象**：山海机甲战队全体成员（电控 / 视觉 / 运营）
> **前置要求**：一台能上网的电脑、一个 GitHub 账号

---

## 📑 目录

1. [[#为什么用 GitHub|为什么用 GitHub]]
2. [[#环境搭建|环境搭建]]
3. [[#Git-核心概念一张图|Git 核心概念 ⇢ 一张图]]
4. [[#日常开发工作流|日常开发工作流]]
5. [[#分支管理规范|分支管理规范]]
6. [[#pr-描述模板|PR 描述模板]]
7. [[#常见问题排查|常见问题排查]]
8. [[#RoboMaster-项目仓库结构建议|仓库结构建议]]
9. [[#进阶主题|进阶主题]]
   - [[#91-git-lfs--大文件管理|9.1 Git LFS]]
   - [[#92-github-actions--自动构建与测试|9.2 GitHub Actions]]
   - [[#93-gh-cli--命令行操作-github|9.3 gh CLI]]
   - [[#94-submodule--多仓库协同|9.4 Submodule]]
   - [[#95-release--tag--赛季版本管理|9.5 Release & Tag]]
   - [[#96-code-review-最佳实践|9.6 Code Review]]
   - [[#97-github-wiki-与-pages--搭建知识库|9.7 Wiki & Pages]]
   - [[#98-标准协作工作流--完整循环|9.8 标准工作流]]
   - [[#99-git-急救手册--事故恢复技巧|9.9 Git 急救手册]]

> 🔗 **关联笔记**（可在此创建并链接）：
> - `[[RM-代码规范]]` — 战队编码规范
> - `[[RM-硬件版本管理]]` — 机械/硬件图纸版本管理
> - `[[RM-CI-CD配置]]` — 持续集成 / 自动测试
> - `[[RM-常见Bug记录]]` — 踩坑记录与解决方案

---

## 为什么用 GitHub

| 痛点                        | Git/GitHub 的解决方案                           |
| :------------------------ | :----------------------------------------- |
| 代码传来传去，不知道哪个是最新版          | **单一可信源**：GitHub 上的 `main` 分支永远是最新稳定版      |
| "我改了 A，你改了 B，合并时覆盖了对方的代码" | **版本控制**：每次修改都有记录，冲突可追溯可回滚                 |
| 上个赛季的代码找不到了               | **Tag / Release**：每个赛季打标签，历史代码永久存档         |
| 新队员不知道怎么上手                | **README + Wiki**：文档和代码放在一起，新人自学           |
| 不知道谁改了哪行、为什么改             | **Blame + Commit Message**：每一行代码都能追溯到作者和原因 |

> 💡 **一句话**：GitHub 是你们的 **代码保险箱 + 协作中枢 + 知识库**。

---

## 环境搭建

### 1. 注册 GitHub 账号

- 打开 [github.com](https://github.com)，用邮箱注册
- **用户名建议**：`RM-战队缩写-姓名拼音`，如 `RM-Cyan-YuShuCheng`
- 注册后把用户名发给队长，让队长拉你进 [山海机甲代码仓库](https://github.com/HebutMas)）

> ⏳ **待补充**：战队 GitHub Organization 链接

### 2. 安装 Git

| 系统 | 安装方式 |
|:--|:--|
| **Windows** | 下载 [Git for Windows](https://git-scm.com/download/win)，一路默认安装即可 |
| **macOS** | 终端执行 `brew install git` 或下载 [Git for Mac](https://git-scm.com/download/mac) |
| **Linux (Ubuntu)** | `sudo apt install git` |

安装后在终端（PowerShell / CMD / 终端）验证：

```bash
git --version
# 应该输出类似：git version 2.47.0
```

### 3. 配置 Git 身份

这个很重要——**每次提交都会带上你的名字和邮箱**，方便追溯是谁写的代码。

```bash
git config --global user.name "你的GitHub用户名"
git config --global user.email "你的GitHub注册邮箱"
```

### 4. 配置 SSH（免密推送）

```bash
# 1. 生成 SSH 密钥（一路回车即可）
ssh-keygen -t ed25519 -C "你的GitHub注册邮箱"

# 2. 复制公钥
# Windows PowerShell:
Get-Content ~/.ssh/id_ed25519.pub | Set-Clipboard
# macOS / Linux:
cat ~/.ssh/id_ed25519.pub | pbcopy   # macOS
cat ~/.ssh/id_ed25519.pub | xclip    # Linux

# 3. 打开 GitHub → Settings → SSH and GPG keys → New SSH key
#    粘贴公钥，Title 填 "我的笔记本" 或 "实验室主机"
```

### 5. 配置终端代理（重要！）

> **如果你用代理上网，这步必看。** Git 走终端，**不会自动使用系统代理**。不配的话 `git push` / `git clone` 会卡住或超时。

```bash
# 设置代理（端口默认7899，按你代理工具的实际端口改）
git config --global http.proxy http://127.0.0.1:7899
git config --global https.proxy http://127.0.0.1:7899

# 取消代理
git config --global --unset http.proxy
git config --global --unset https.proxy
```

端口在你的代理工具（Clash / V2Ray 等）里找，常见 7890、7899、1080、10809。`git push` 卡在 `Connecting to github.com` 超时就是代理没配。

---

## Git 核心概念 ⇢ 一张图

```
你的电脑（本地）                         GitHub（远程）
┌─────────────────┐                   ┌──────────────┐
│  工作目录        │  git add →       │              │
│  (你改的文件)    │         暂存区    │  远程仓库     │
│                 │  ← git restore   │  (GitHub)     │
│  ↓ git commit   │                   │  ↑ git push  │
│  本地仓库        │                   │  │           │
│  (所有历史版本)  │  ←── git pull ────│  ↓ git fetch │
└─────────────────┘                   └──────────────┘
```

| 概念                    | 一句话解释                 |
| :-------------------- | :-------------------- |
| **仓库 (Repo)**         | 一个项目的"文件夹"，带有完整历史记录   |
| **提交 (Commit)**       | 一次保存操作，相当于游戏的"存档点"    |
| **分支 (Branch)**       | 一条独立的开发线，你可以安全地在上面做实验 |
| **推送 (Push)**         | 把本地代码上传到 GitHub       |
| **拉取 (Pull)**         | 把 GitHub 上的最新代码下载到本地  |
| **合并 (Merge)**        | 把两条分支的改动合到一起          |
| **克隆 (Clone)**        | 把远程仓库完整下载到本地（含全部历史）   |
| **PR (Pull Request)** | "我写好了，请审核并合并到主分支" 的请求 |

---

## 日常开发工作流

### 单人开发（最简单场景）

```bash
# 1. 拉取最新代码（开始工作前必做！）
git pull

# 2. 写代码...写代码...写代码...

# 3. 查看改了什么
git status

# 4. 添加到暂存区
git add .                    # 添加所有改动
# 或
git add src/main.c           # 只添加特定文件

# 5. 提交（写好注释！）
git commit -m "feat: 底盘PID参数整定完成"

# 6. 推送到 GitHub
git push
```

> ⚠️ **写完一个功能就 commit，不要攒一天再提交！** 一个 commit 只做一件事——写完云台 PID 就 commit 一次，写完底盘功率再 commit 一次。这样出问题能精确回滚，Code Review 也看得过来。Commit message 用 `feat:` / `fix:` / `refactor:` 前缀，如 `feat: 云台PID角度环实现`。

### 多人协作 — Fork 模式

战队仓库是唯一的"权威源"，所有人**先 fork 到自己的 GitHub 账号**，日常开发推到自己的 fork 上，测试通过后再 PR 回主仓库。

#### Fork + Clone

```bash
# 1. GitHub 网页上点 Fork，把战队仓库 fork 到自己账号下
# 2. Clone 你自己的 fork（不是战队仓库！）
git clone git@github.com:你的用户名/rm_control.git
cd rm_control
# 3. 添加战队仓库为 upstream
git remote add upstream git@github.com:HebutMas/rm_control.git
```

#### 日常开发 — 推到自己的 fork

```bash
git checkout -b feature/云台PID调参
# 写代码...写完一个功能就 commit...
git add . && git commit -m "feat: 云台PID参数整定"
git push -u origin feature/云台PID调参
```

#### 两人协作 — 互相拉

两个人一起开发时不需要走 PR，直接从对方的 fork 拉：

```bash
git remote add teammate git@github.com:队友用户名/rm_control.git
git fetch teammate
git merge teammate/feature/云台PID调参
```

#### 多人协作 — PR 回主仓库

测试好的分支向战队主仓库提 PR，Review 通过合并后同步 upstream 并删分支。

> 📖 完整分步教程（含同步 upstream、merge vs rebase、冲突解决）见 [[#98-标准协作工作流--完整循环|§9.8 标准协作工作流]]。

---

## 分支管理规范

### 分支命名规则

```text
<type>/<简短描述>

type 可选值：
  feature   → 新功能
  fix       → Bug 修复
  hotfix    → 紧急修复（直接基于 main）
  refactor  → 代码重构
  docs      → 文档更新
  test      → 测试相关
  exp       → 实验性代码
```

**示例**：
- `feature/云台PID控制`
- `fix/串口通信丢帧`
- `hotfix/比赛前裁判系统兼容`
- `exp/新版电机驱动库测试`

### 分支保护规则（由队长在 GitHub 设置）

| 规则                    | 说明             |
| :-------------------- | :------------- |
| `main` 分支不允许直接推送      | 必须通过 PR 合并     |
| PR 至少需要 1 人 Review 通过 | 防止低级错误直接上主分支   |
| 合并前必须通过 CI 检查         | （如果有配置自动构建/测试） |

> ⏳ **待补充**：各战队的分支保护策略可能不同，此处为建议配置。

---

## PR 描述模板

> 完整的协作流程（克隆→分支→提交→PR→合并→收尾）见 [[#98-标准协作工作流--完整循环|§9.8]]。这里只给一份可直接复用的 PR 描述模板。

建议存为仓库的 `.github/PULL_REQUEST_TEMPLATE.md`，创建 PR 时会自动填入：

```markdown
## 描述
给底盘添加功率限制功能，防止超功率扣血。

## 改动内容
- [x] 新增 `power_limit.c` / `power_limit.h`
- [x] 功率超过阈值时等比缩减输出
- [ ] 尚未测试 200W 引擎功率枪场景

## 测试方式
在训练场模式下手动测试，观察裁判系统功率曲线。

## 关联 Issue
Closes #42
```

> 💡 `Closes #42` 写进 PR 描述后，PR 合并时会**自动关闭** #42 号 Issue。也支持 `Fixes` / `Resolves`。

---

## 常见问题排查

### Q1: `git push` 时提示权限不足

```bash
# 原因：你没被加入仓库的 Collaborator 或 Team
# 解决：联系队长，在仓库 Settings → Collaborators 里添加你
```

### Q2: 合并时出现冲突 (Conflict)

冲突文件里会出现这种标记，手动编辑保留正确代码、删掉标记即可：

```text
<<<<<<< HEAD
你的代码
=======
队友的代码
>>>>>>> feature/xxx
```

> 📖 冲突产生的原因、`merge` vs `rebase` 两种同步方式、逐步解决流程 → [[#98-标准协作工作流--完整循环|§9.8]] 第五步。放弃本次合并重来 → [[#99-git-急救手册--事故恢复技巧|§9.9]] 场景 6。

### Q3: 不小心提交了不该提交的文件 / 想撤销提交

- 撤销提交但保留改动：`git reset --soft HEAD~1`
- 补提交漏掉的文件 → [[#99-git-急救手册--事故恢复技巧|§9.9]] 场景 1
- `reset --hard` 后后悔了 → [[#99-git-急救手册--事故恢复技巧|§9.9]] 场景 2（`reflog` 后悔药）

### Q4: 我想回到之前的某个版本看看

```bash
# 查看提交历史
git log --oneline --graph

# 临时切换到某个版本（只读）
git checkout <commit-hash>

# 回到最新版本
git checkout main
```

### Q5: 提交信息写错了

```bash
# 修改最近一次提交信息（还没 push 时）
git commit --amend -m "新的提交信息"
```

> 已经 push 过要改写历史 → 用 `git push --force-with-lease`，注意事项见 [[#99-git-急救手册--事故恢复技巧|§9.9]] 场景 1。

---

## RoboMaster 项目仓库结构建议

```text
rm_control/                     # 电控代码仓库
├── .github/
│   ├── ISSUE_TEMPLATE/         # Issue 模板
│   │   ├── bug_report.md
│   │   └── feature_request.md
│   └── PULL_REQUEST_TEMPLATE.md
├── src/
│   ├── chassis/                # 底盘控制
│   ├── gimbal/                 # 云台控制
│   ├── shooter/                # 发射机构
│   ├── communication/          # 通信模块
│   └── utils/                  # 工具函数
├── tests/                      # 单元测试
├── docs/                       # 文档
├── tools/                      # 调参工具脚本
├── README.md                   # 项目说明
├── .gitignore                  # 忽略文件规则
└── CMakeLists.txt              # 构建配置

rm_mechanical/                  # 机械图纸仓库
├── chassis/
├── gimbal/
├── shooter/
├── README.md
└── .gitignore

rm_vision/                      # 视觉代码仓库
├── models/                     # 模型文件（⚠️ 大文件用 Git LFS）
├── src/
├── configs/
├── README.md
└── .gitignore
```

### 推荐的 `.gitignore` 模板

```gitignore
# 编译产物
build/
cmake-build-*/
*.o
*.obj
*.elf
*.hex
*.bin

# IDE 配置
.vscode/
.idea/
*.swp
*.swo

# 操作系统
.DS_Store
Thumbs.db

# Python
__pycache__/
*.pyc
venv/
.venv/

# Keil / IAR 中间文件
*.uvguix.*
*.scvd
JLinkLog.txt
```

---

## 进阶主题

> 以下为 Git 进阶技巧，按需查阅。每个主题均可独立阅读。

---

### 9.1 Git LFS — 大文件管理

#### 痛点

Git 对二进制大文件的处理极其低效——每次 `git clone` 都会下载**所有版本的完整副本**，一个 200MB 的 `.step` 文件改 10 次，仓库就膨胀到 2GB+。

**Git LFS（Large File Storage）** 的思路：仓库里只存一个"指针文件"（约 130 字节），真正的二进制文件存在 LFS 服务器上，按需下载。

```
普通 Git：每次 clone 拉取全部历史版本 → 仓库越来越慢
Git LFS：只下载当前 checkout 版本的大文件 → 和普通仓库一样快
```

#### 安装与配置

```bash
# 1. 安装 Git LFS（一次性）
# Windows: 下载安装包 https://git-lfs.com
# macOS:   brew install git-lfs
# Ubuntu:  sudo apt install git-lfs

# 2. 初始化（每台机器一次）
git lfs install

# 3. 在仓库中指定要追踪的文件类型
git lfs track "*.step" "*.sldprt" "*.stp"     # 机械图纸
git lfs track "*.pth" "*.onnx" "*.engine"     # 视觉模型
git lfs track "*.bin" "*.hex" "*.elf"         # 固件二进制
git lfs track "*.zip" "*.tar.gz"              # 数据集压缩包

# 4. 提交 .gitattributes（LFS 追踪规则文件）
git add .gitattributes
git commit -m "chore: 配置 Git LFS 追踪规则"
```

#### 日常使用

配置完成后，**使用方式与普通 Git 完全一致**——`add` / `commit` / `push` 照旧，LFS 在后台自动处理。

```bash
git add chassis_v2.step
git commit -m "机械: 底盘二代图纸更新"
git push    # LFS 文件自动上传到 LFS 服务器
```

#### 克隆含 LFS 的仓库

```bash
# 标准克隆（自动下载 LFS 文件）
git clone git@github.com:RM-YourTeam/rm_mechanical.git

# 如果已经克隆了但 LFS 文件没下载
git lfs pull

# 只想下载当前版本（跳过历史大文件）—— 适合只看不编
GIT_LFS_SKIP_SMUDGE=1 git clone git@github.com:RM-YourTeam/rm_mechanical.git
```

#### 注意事项

| 要点 | 说明 |
|:-----|:-----|
| **免费额度** | GitHub 免费提供 1GB 存储 + 1GB/月流量（2024），超出需付费 |
| **早期加入** | 仓库刚开始就配 LFS，不要等到仓库已经很大再迁移 |
| **迁移已有仓库** | 用 `git lfs migrate` 可把历史中的大文件转为 LFS 指针，但操作前先备份 |
| **CI/CD** | GitHub Actions 环境自带 `git-lfs`，checkout 时设置 `lfs: true` 即可 |
| **不适用场景** | 经常修改的小文件（如 `.c` / `.h`）用普通 Git 更好；LFS 适合改一次顶很久的二进制文件 |

---

### 9.2 GitHub Actions — 自动构建与测试

#### 是什么

GitHub Actions 让你在 **push 代码或创建 PR 时自动运行脚本**——自动编译、自动测试、自动格式化检查，不用人手点。对战队来说尤其有价值：没人专门做 CI，靠自动化兜底。

```
你 push 代码 → GitHub Actions 自动触发 → 编译 + 测试 → 结果贴到 PR 上
                                              ↓ 失败
                                         自动 @ 你修
```

#### 核心概念：Workflow 文件

在仓库中创建 `.github/workflows/<名称>.yml`，GitHub 会自动识别并执行。

#### 实战示例 1：自动编译 STM32 项目

```yaml
# .github/workflows/build.yml
name: 编译检查

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main]

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          submodules: recursive

      - name: 安装 ARM 工具链
        run: |
          sudo apt update
          sudo apt install -y gcc-arm-none-eabi cmake

      - name: 编译电控固件
        run: |
          mkdir build && cd build
          cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi.cmake
          make -j$(nproc)

      - name: 上传编译产物
        uses: actions/upload-artifact@v4
        with:
          name: firmware
          path: build/*.elf
```

#### 实战示例 2：自动检查代码格式

```yaml
# .github/workflows/format-check.yml
name: 代码格式检查

on: [pull_request]

jobs:
  clang-format:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: 运行 clang-format 检查
        run: |
          sudo apt install -y clang-format
          find . -name '*.c' -o -name '*.h' | xargs clang-format --dry-run --Werror
          # --dry-run: 不修改文件，只检查
          # --Werror: 格式不合规视为错误，让 CI 变红
```

#### 实战示例 3：自动跑单元测试

```yaml
# .github/workflows/test.yml
name: 单元测试

on:
  push:
    branches: [main]
  pull_request:

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: 编译并运行测试
        run: |
          cd tests
          mkdir build && cd build
          cmake .. && make
          ctest --output-on-failure
```

#### 实用技巧

| 技巧             | 做法                                                             |
| :------------- | :------------------------------------------------------------- |
| **加密密钥**       | 在仓库 Settings → Secrets 中添加，workflow 里用 `${{ secrets.XXX }}` 引用 |
| **自托管 Runner** | 在实验室主机上装 GitHub Actions Runner，可以连接真实硬件做测试                     |
| **定时触发**       | `on: schedule: - cron: "0 2 * * 1"` → 每周一凌晨 2 点自动跑一次           |
| **跳过 CI**      | commit message 中含 `[skip ci]` 则跳过本次触发                          |
| **免费额度**       | 公共仓库无限免费，私有仓库每月 2000 分钟（2024）                                  |

> 💡 **战队最值得做的 CI**：编译检查 + 格式检查。两个 workflow 加起来不到 50 行 YAML，但能拦截 80% 的低级错误（忘提交文件、缩进混乱、编译不过）。

---

### 9.3 gh CLI — 命令行操作 GitHub

`gh` 是 GitHub 官方命令行工具，不用开网页就能建 PR、看 Issue、管 Release。最实用的场景：代码 push 完了终端里一行命令直接建 PR。

```bash
# 安装
winget install GitHub.cli   # Windows
gh auth login               # 登录

# 常用
gh pr create --title "feat: 云台PID调参" --body "描述"  # 创建 PR
gh pr list                                               # 列出 PR
gh pr checkout 12                                        # 切到 #12 的分支
gh pr merge 12 --squash --delete-branch                 # 合并并删分支
gh repo fork HebutMas/rm_control                         # 一键 fork 并 clone
```

> 💡 知道有这个东西就行，用到的时候 `gh --help` 查命令。

---

### 9.4 Submodule — 多仓库协同

#### 场景

战队的电控、视觉、哨兵三个项目都要用同一个 `rm_common` 工具库（CRC 校验、PID 控制器、通信协议解析）。直接复制粘贴 → 三个版本逐渐分化，修 bug 要改三份。

Submodule 解决：**一个仓库引用另一个仓库的特定版本**，共享代码一处维护。

```
rm_control/              rm_common/（独立仓库）
  ├── src/
  ├── lib/
  │   └── rm_common/  ← git submodule，锁定在 commit abc123
  └── ...
```

#### 基本操作

```bash
# 1. 在项目中添加 submodule
cd rm_control
git submodule add git@github.com:RM-YourTeam/rm_common.git lib/rm_common
git commit -m "chore: 添加 rm_common 子模块"

# 2. 克隆含 submodule 的项目
git clone --recursive git@github.com:RM-YourTeam/rm_control.git
# 如果忘了 --recursive：
git submodule update --init --recursive

# 3. 更新 submodule 到最新版本
cd lib/rm_common
git pull origin main
cd ../..
git add lib/rm_common
git commit -m "chore: 更新 rm_common 到最新版"

# 4. 查看当前 submodule 状态
git submodule status
# 输出示例:  abc1234 lib/rm_common (v1.2.0)
```

#### 常用技巧

```bash
# 批量更新所有 submodule
git submodule update --remote --recursive

# 修改 submodule 里的代码并提交（需要在 submodule 内 push，再在主仓库更新引用）
cd lib/rm_common
# 改代码...
git add . && git commit -m "fix: 修复 CRC 查表错误"
git push
cd ../..
git add lib/rm_common
git commit -m "chore: 更新 rm_common（修复 CRC bug）"

# 移除 submodule
git submodule deinit lib/rm_common
git rm lib/rm_common
rm -rf .git/modules/lib/rm_common
```

#### Submodule vs Monorepo 对比

| 维度 | Submodule | Monorepo（全放一个仓库） |
|:-----|:---------|:---------------------|
| **仓库体积** | ✅ 小，各仓库独立 | ❌ 大，clone 慢 |
| **版本控制** | ✅ 精确锁定依赖版本 | ❌ 依赖随主仓库变动 |
| **跨项目修改** | ❌ 需要改两个仓库、两次 PR | ✅ 一次提交覆盖所有 |
| **权限控制** | ✅ 机械组不需要读视觉代码 | ❌ 所有人看到所有代码 |
| **CI/CD** | 各仓库独立配置 | 一次配置覆盖全部 |
| **学习成本** | 🔶 需要理解 submodule 机制 | ✅ 就是普通 Git |

> 💡 **战队建议**：如果不同组（电控/视觉/机械）的代码**各自独立、共享库很少**，用 Submodule 更合适。如果是一个紧密耦合的 ROS 工作空间（所有包互相依赖），直接 Monorepo 更省心。

---

### 9.5 Release & Tag — 赛季版本管理

#### 为什么需要 Tag

赛季过程中你会频繁改动代码。到了比赛日，你需要精确知道**哪一版代码是赛场上跑的**——Tag 就是这个"书签"。

```bash
# 给当前 commit 打标签
git tag -a v2025.1.0 -m "分区赛出战版本"
git push origin v2025.1.0

# 查看所有标签
git tag -n9    # -n9 显示标签注释

# 回到某个标签的代码
git checkout v2025.1.0

# 基于标签创建热修复分支
git checkout -b hotfix/裁判系统兼容 v2025.1.0
```

#### 版本号规范（RM 适配）

```
v<赛季年份>.<主版本号>.<修订号>

v2025.1.0  → 2025 赛季，第一个正式版本
v2025.1.1  → 在 1.0 基础上修了小 bug
v2025.2.0  → 加了新功能（如新增能量机关识别）
v2025.0.0-rc1  → 候选发布版（赛前测试用）
```

| 版本段 | 何时递增 | 示例 |
|:-------|:---------|:-----|
| 赛季年份 | 新赛季 | `v2024.x.x` → `v2025.1.0` |
| 主版本号 | 重大功能变更 / 不兼容的接口改动 | `v2025.1.0` → `v2025.2.0` |
| 修订号 | Bug 修复 / 参数微调 | `v2025.1.0` → `v2025.1.1` |

#### GitHub Release：附带固件二进制

Tag 只是代码层面的标记。**Release** 是 GitHub 提供的发布页面，可以在 Tag 之上附带编译好的 `.hex` / `.bin` 文件，方便电控组直接烧录，不需要拉代码编译。

```bash
# 1. 打 tag 并推送
git tag -a v2025.1.0 -m "分区赛出战版本"
git push origin v2025.1.0

# 2. 去 GitHub 仓库页面 → Releases → Draft a new Release
#    - Tag: v2025.1.0
#    - Title: 分区赛出战版本
#    - 拖入编译好的 rm_control.hex / rm_vision.tar.gz
#    - 勾选 "Set as pre-release"（赛前测试版）或正式 Release（赛后归档版）
```

> 💡 **实战建议**：每次比赛前后打一个 Tag，Release 里附上固件 + 调参配置文件。这样下个赛季接手的人可以一键下载还原。

---

### 9.6 Code Review 最佳实践

#### 为什么要 Review

- **知识传播**：写底盘的同学顺带懂了云台的代码结构，避免知识孤岛
- **Bug 早发现**：Review 阶段修 bug 的成本是测试阶段的 1/10，是比赛翻车的 1/100
- **代码风格统一**：不然三年后仓库里有五种命名风格

#### Review 者：看什么

| 优先级 | 检查项 | 具体问题 |
|:-------|:-------|:---------|
| 🔴 必看 | **逻辑正确性** | 边界条件处理了吗？除零保护有吗？ |
| 🔴 必看 | **安全性（RM 专属）** | 这段代码会不会让电机暴走？功率限制有没有被绕过？ |
| 🟡 建议 | **可读性** | 变量名能一眼看懂吗？关键逻辑有注释吗？ |
| 🟡 建议 | **结构设计** | 硬编码的阈值应该提成 `#define` 吗？重复代码能抽成函数吗？ |
| 🟢 可选 | **性能** | 循环里有没有不必要的重复计算？ |

#### 如何给出好的 Review 意见

```markdown
<!-- ❌ 差的 Review -->
"这里写得不好"
"重写"
"为什么不用 XXX？"

<!-- ✅ 好的 Review -->
"`motor_output` 在 `target > MAX_POWER` 时没有限幅——如果裁判系统传回异常高值，电机会暴走。
建议在第 47 行加一个 `CLAMP(target, 0, MAX_POWER)`。"

<!-- ✅ GitHub 代码建议（可直接一键采纳） -->
```suggestion
    motor_output = CLAMP(target, 0, MAX_POWER);
```
```

**原则**：
- **对事不对人**——"这段逻辑有问题"而不是"你写错了"
- **说明为什么**——不只是"改成 X"，而是"因为 Y 情况会出问题，所以建议 X"
- **区分必须改和建议改**——用 `nit:`（nitpick，小建议）标记非关键意见

#### 被 Review 者：如何接受反馈

- Review 意见是对代码的评价，不是对人的评价
- 不理解为什么这样改 → 直接在 PR 评论里问，Review 者通常很乐意解释
- 有不同意见 → 讨论，不要默默接受然后心生不满
- 每个意见都要有回应：采纳 / 解释为什么不做 / 记录为 follow-up issue

#### RM 专项 Review Checklist

除通用检查项外，RM 项目应额外关注：

```markdown
## RM Code Review Checklist
- [ ] CAN 总线发送频率是否在合理范围内（通常 ≤1kHz）？
- [ ] 中断服务函数是否足够短（<100μs 为宜）？
- [ ] 有 watchdog 喂狗吗？死循环会导致机器人失联吗？
- [ ] 遥控器/裁判系统断连时的默认行为安全吗（停机 vs 保持最后指令）？
- [ ] 浮点运算是否在 FPU 可用时正确使用了硬件加速？
- [ ] 视觉：推理线程和图像采集线程之间是否有正确的锁保护？
```

> 💡 **最低配置**：每个 PR 至少一个人看过才能合入 `main`。不需要很正式——关键是养成习惯。

---

### 9.7 GitHub Wiki 与 Pages — 搭建知识库

#### Wiki vs Pages 选型

| 维度 | GitHub Wiki | GitHub Pages |
|:-----|:-----------|:------------|
| **用途** | 内部技术文档 | 对外展示网站 |
| **编辑方式** | 网页在线编辑 + Git 仓库 | 本地写 Markdown → push 自动构建 |
| **访问控制** | 跟随仓库权限（队友可见） | 默认公开 |
| **适合放什么** | 调参手册、通信协议、硬件接线图、新人入门指南 | 战队主页、赛季总结、开源项目文档 |
| **上手难度** | ✅ 零门槛，直接在网页上写 | 🔶 需要选一个静态网站工具（MkDocs / Docusaurus / Jekyll） |

#### Wiki：内部知识库

```bash
# Wiki 本质上也是一个 Git 仓库，可以本地克隆
git clone git@github.com:RM-YourTeam/rm_control.wiki.git
cd rm_control.wiki

# 写 Markdown 文件 → push → 自动更新 Wiki 页面
echo "# 底盘调参手册" > 底盘调参手册.md
git add . && git commit -m "docs: 初始化底盘调参手册"
git push
```

**Wiki 适合放的内容**：
- 各模块调参手册（PID 参数推荐值、调参步骤）
- 通信协议文档（CAN ID 分配、自定义帧格式）
- 硬件接线规范（引脚分配表、供电要求）
- 新人入门 SOP（环境配置、编译流程、烧录步骤）
- 赛后复盘记录

#### Pages：对外知识库 / 战队主页

推荐使用 **MkDocs + Material 主题**，学习成本极低：

```bash
# 1. 安装
pip install mkdocs mkdocs-material

# 2. 在仓库 docs/ 目录下创建配置
# docs/index.md — 首页
# docs/调参手册/底盘.md — 子页面

# 3. mkdocs.yml
site_name: 山海机甲知识库
theme: material
nav:
  - 首页: index.md
  - 调参手册:
    - 底盘: 调参手册/底盘.md
    - 云台: 调参手册/云台.md

# 4. 本地预览
mkdocs serve   # 打开 http://localhost:8000

# 5. 部署到 GitHub Pages
mkdocs gh-deploy
```

然后在仓库 Settings → Pages 中选择 `gh-pages` 分支即可。

> 💡 **建议分工**：Wiki 放"活文档"（频繁更新，随代码一起迭代），Pages 放"归档内容"（赛季总结、开源发布）。两者不冲突，可以用同一个仓库同时维护。

### 9.8 标准协作工作流 — 完整循环

> 从 `git clone` 到 PR 合并全流程。重点解决：「队友更新了 main，我的 feature 分支怎么办？」

#### 流程图

```text
远程 main  ──●──●──●──────────●──  队友的新提交
             │                  │
             ↓ git clone        │
本地 main  ──●                   │
             │ git checkout -b   │
             ↓                   │
feature/xxx  ●──●──●──●          │
                       │         │
                       │  git fetch + merge/rebase
                       ↓         ↓
feature/xxx  ●──●──●──●──●──  同步完成
                         │
                         ↓ git push + PR
                     合并回 main ✓
```

#### 第一步：克隆仓库

```bash
git clone git@github.com:RM-YourTeam/rm_control.git
cd rm_control
```

#### 第二步：从最新 main 创建功能分支

```bash
git checkout main                #每次写代码前都要有这一步
git pull origin main             # 确保从最新的 main 起步
git checkout -b feature/底盘功率控制
```

#### 第三步：开发 + 小步提交

```bash
# 写代码...写代码...
git status                       # 每次提交前看一眼
git add src/chassis/power_limit.c
git commit -m "feat: 添加功率限制核心逻辑"

# 继续写...
git add src/chassis/power_limit.h
git commit -m "refactor: 提取功率阈值到配置头文件"

# 继续写...
git add tests/test_power_limit.c
git commit -m "test: 添加功率限制单元测试"
```

> 💡 **小步提交的好处**：出问题时 `git bisect` 能精确定位到 10 行代码的范围，而不是 500 行。

#### 第四步：推送到云端（在远程创建分支）

```bash
# 首次推送：-u 让本地分支追踪远程分支
git push -u origin feature/底盘功率控制

# 之后每次 push 只需要：
git push
```

此时打开 GitHub 仓库页面，已经能看到 `feature/底盘功率控制` 分支了。

#### 第五步：处理 main 更新 ⭐（核心环节）

> **场景**：你在 feature 分支上写了一周代码，期间 main 被队友合并了 5 个 PR。
> 你的分支已经落后——**必须在发起 PR 前同步 main 的最新代码**，否则合并时会有大量冲突。

**两种同步方案**：

| 维度       | `git merge origin/main`              | `git rebase origin/main`                    |
| :------- | :----------------------------------- | :------------------------------------------ |
| **原理**   | 把 main 的改动"合并进来"，生成一个 merge commit   | 把你分支的 commit "搬到" main 最新位置，重写历史            |
| **提交历史** | 保留真实时间线（你的 commit + merge commit 交错） | 线性历史，你的 commit 排在 main 最新 commit **之后**     |
| **冲突解决** | 一次性解决所有冲突                            | 每个 commit 逐个解决冲突                            |
| **适用场景** | 多人共用同一分支、想保留完整时间线                    | 个人分支、追求干净线性历史                               |
| **风险**   | ✅ 低，不改变已有 commit                     | ⚠️ 改写 commit hash，**绝对不能** rebase 已被别人使用的分支 |

---

**方案 A：merge（推荐新手首选）**

```bash
# 1. 切到你的 feature 分支
git checkout feature/底盘功率控制

# 2. 拉取远程最新信息
git fetch origin

# 3. 把 origin/main 合并进你的分支
git merge origin/main

# 4. 如果无冲突 → Git 自动生成 merge commit，完成
#    如果有冲突 → 手动解决 → git add → git commit
git add .
git commit -m "merge: 同步 main 最新代码"

# 5. 推送
git push
```

---

**方案 B：rebase（历史更干净，适合个人分支）**

```bash
# 1. 切到你的 feature 分支
git checkout feature/底盘功率控制

# 2. 拉取远程最新信息
git fetch origin

# 3. 把你的 commit 搬到 origin/main 最新位置
git rebase origin/main

# 4. 如果有冲突：
#    → Git 停下来，手动解决冲突 → git add
#    → git rebase --continue      （继续搬下一个 commit）
#    → 重复直到 rebase 完成
#
#    如果想放弃重来：
#    → git rebase --abort         （回到 rebase 之前的状态）

# 5. ⚠️ 因为改写了提交历史，首次推送需要 force：
git push --force-with-lease
#    ^-- 安全版：如果远程有你不知道的新 commit，会拒绝推送而不是默默覆盖
```

---

**merge vs rebase — 决策口诀**：

> **不确定就用 merge**；分支只有你一个人用 → rebase 更干净；分支有别人在用 → 绝对不要 rebase。

#### 第六步：创建 Pull Request

1. 打开 GitHub 仓库 → `Pull requests` → `New pull request`
2. `base` = `main`，`compare` = `feature/底盘功率控制`
3. 填写 PR 描述（可复用 [[#pr-描述模板|PR 描述模板]]）→ `Create pull request`
4. 等待队友 Review → 根据意见修改 → `git push` 自动更新 PR

#### 第七步：收尾清理

```bash
# PR 合并后
git checkout main
git pull origin main                           # 拉取包含你代码的最新 main
git branch -d feature/底盘功率控制               # 删除本地分支

# 删除远程分支（二选一）
git push origin --delete feature/底盘功率控制    # 命令行方式
# 或在 GitHub PR 合并页面勾选 "Delete branch"    # 网页方式（推荐）
```

#### 完整流程速记

```text
clone → checkout -b → 写代码 → 小步 commit → push -u →
  （main 更新了？→ fetch + merge/rebase → push）→
  创建 PR → 合并 → 切回 main pull → 删除旧分支
```

#### 常见失误预防

| 失误                   | 后果                  | 预防                        |
| :------------------- | :------------------ | :------------------------ |
| 直接在 main 上写代码        | 难以隔离，push 时可能被拒     | 第零步就 `git branch` 确认当前分支  |
| push 前没同步 main       | PR 冲突多，CI 可能基于旧代码测试 | PR 页面的 "out-of-date" 警告   |
| rebase 后用了 `--force` | 可能默默覆盖队友的 commit    | 永远只用 `--force-with-lease` |
| 忘记 pull 就开始写         | 基于过时代码开发            | 把 `git pull` 当肌肉记忆，坐下就先拉  |
| 一个 commit 改了几十个文件    | 无法回滚、无法 bisect      | 一个 commit 只做一件事           |

---

### 9.9 Git 急救手册 — 事故恢复技巧

> 以下每个技巧对应一个真实场景。熟练了可以救你一条命。

#### 场景 1：「我 commit 完才发现漏了一个文件」

```bash
# 补充提交（不新建 commit，追加到上一个 commit 里）
git add 漏掉的文件.c
git commit --amend --no-edit   # --no-edit: 不改 commit message

# ⚠️ 如果已经 push 了：
git push --force-with-lease    # 强制推送（安全版，不会覆盖队友的新 commit）
```

#### 场景 2：「我 git reset --hard 之后后悔了」

```bash
# reflog 记录了你所有的 HEAD 移动——这是 Git 的"时光机"
git reflog
# 输出：
# abc1234 HEAD@{0}: reset: moving to HEAD~1
# def5678 HEAD@{1}: commit: 底盘功率限制功能  ← 你想找回的 commit

# 回到那个 commit
git reset --hard def5678     # 或者
git checkout -b 恢复分支 def5678  # 更安全：先建个分支看看
```

> 💡 **reflog 是 Git 最强的后悔药**——只要 commit 过，90 天内都可以找回（除非执行了 `git gc`）。

#### 场景 3：「main 分支上发现一个 bug，不确定是哪个 commit 引入的」

```bash
# git bisect：二分查找定位引入 bug 的 commit
git bisect start
git bisect bad HEAD              # 当前版本有问题
git bisect good v2025.0.0        # 这个版本没问题

# Git 会自动 checkout 到中间的一个 commit
# 你测试一下 → 告诉 Git 结果：
git bisect good    # 这个 commit 没问题 → Git 往更近的方向找
# 或
git bisect bad     # 这个 commit 有问题 → Git 往更远的方向找

# 重复几次后，Git 会告诉你第一个引入 bug 的 commit
# 找到后退出 bisect：
git bisect reset
```

```bash
# 如果 bug 可以用脚本自动检测，一行搞定：
git bisect start HEAD v2025.0.0
git bisect run python test_motor_power.py   # 脚本返回 0 = good，非 0 = bad
```

#### 场景 4：「只需要把另一个分支上的某几个 commit 搬过来」

```bash
# cherry-pick：精确搬运指定 commit
git checkout -b fix/云台

# 方法 A：搬一个 commit
git cherry-pick abc1234

# 方法 B：搬一段连续的 commit（不含 start，含 end）
git cherry-pick abc1234..def5678

# 方法 C：搬一段连续的 commit（含 start）
git cherry-pick abc1234^..def5678

git push -u origin fix/云台

#只挑选特定commit推送到远端
```

#### 场景 5：「写到一半，需要紧急切分支修 bug，又不想 commit 半成品」

```bash
# git stash：暂存当前进度
git stash                    # 暂存所有改动
git stash -u                 # 暂存包括未跟踪的新文件
git stash save "底盘PID调试中"  # 带描述信息

# 切去修 bug...
git checkout main
# 修 bug → commit → push → 切回来
git checkout feature/底盘

# 恢复暂存的进度
git stash pop      # 恢复最近一次 stash 并删除记录
git stash apply    # 恢复 stash 但保留记录（可以多次 apply）

# 查看 stash 列表
git stash list
# stash@{0}: On feature/底盘: 底盘PID调试中
# stash@{1}: On main: 比赛前紧急配置修改
```

#### 场景 6：「merge 到一半冲突太多，想放弃重来」

```bash
git merge --abort       # 放弃本次 merge，回到 merge 前的状态
git rebase --abort      # 放弃本次 rebase
git cherry-pick --abort # 放弃本次 cherry-pick
```

#### 场景 7：「不小心在主分支上直接写了代码」

```bash
# 把改动搬到新分支上
git checkout -b feature/新功能     # 基于当前位置建分支（改动还在）
git checkout main                  # 切回 main
git reset --hard origin/main       # main 回到远程版本（丢弃本地改动）
# 改动现在安全地留在 feature/新功能 分支上了
```

#### 急救命令速查表

| 事故                    | 命令                                                                           |
| :-------------------- | :--------------------------------------------------------------------------- |
| commit 漏文件            | `git commit --amend --no-edit`                                               |
| 误删 commit / reset 后悔  | `git reflog` → `git checkout -b 恢复 <hash>`                                   |
| 找到引入 bug 的 commit     | `git bisect start` → `bisect bad`/`good`                                     |
| 搬指定 commit 到当前分支      | `git cherry-pick <hash>`                                                     |
| 临时切分支不 commit         | `git stash` → 处理完 → `git stash pop`                                          |
| 放弃正在进行的 merge         | `git merge --abort`                                                          |
| 在主分支上误写代码             | `git checkout -b 新分支` → `git checkout main` → `git reset --hard origin/main` |
| 已 push 的错误 commit 要覆盖 | `git commit --amend` → `git push --force-with-lease`                         |

> 💡 **最重要的急救原则**：不确定时**先建分支**（`git checkout -b 试试看`），分支里随便折腾，搞砸了删掉分支就好。`reflog` 是最后的保险。

---

## 📚 推荐资源

- [Git 官方文档](https://git-scm.com/doc)
- [GitHub Skills](https://skills.github.com/) — 交互式学习
- [Learn Git Branching](https://learngitbranching.js.org/?locale=zh_CN) — 可视化学习 Git 分支（强烈推荐）
- [Pro Git 中文版](https://git-scm.com/book/zh/v2) — 最权威的 Git 书籍

---

> 🏁 **开始你的第一步**：打开终端，执行 `git --version`，然后找队长要仓库地址，`git clone` 下来！
>
> 有问题随时在战队群里问，或者在这里补充 `[[RM-常见Bug记录]]`。
