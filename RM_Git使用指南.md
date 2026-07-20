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
updated: 2026-07-20
status: draft
---

# 山海机甲 Git 使用教程

> **适用对象**：山海机甲战队全体成员（电控 / 视觉 / 运营）
> **前置要求**：一台能上网的电脑、一个 GitHub 账号

---

## 目录

1. [[#为什么用 GitHub|为什么用 GitHub]]
2. [[#环境搭建|环境搭建]]
3. [[#Git 核心概念一张图|Git 核心概念一张图]]
4. [[#本地仓库与单人开发|本地仓库与单人开发]]
5. [[#Commit 提交规范|Commit 提交规范]]
6. [[#分支管理规范|分支管理规范]]
7. [[#分支合并：merge vs rebase|分支合并：merge vs rebase]]
8. [[#实用 .gitignore 模板|实用 .gitignore 模板]]
9. [[#多人协作开发流程|多人协作开发流程]]
10. [[#多人协作注意事项|多人协作注意事项]]
11. [[#PR 描述模板|PR 描述模板]]
12. [[#常见问题排查|常见问题排查]]
13. [[#GitHub Wiki 与 Pages|GitHub Wiki 与 Pages]]
14. [[#GitHub Actions — 自动化编译测试|GitHub Actions]]
15. [[#技术储备（未来方向）|技术储备（未来方向）]]

> 🔗 **关联笔记**（知识库 `rm-electric-knowledge-base/` 下）：
> - `[[04_notes/pitfalls]]` — 踩坑记录与解决方案
> - `[[04_notes/env_pitfalls]]` — 环境配置踩坑
> - `[[04_notes/changelog]]` — 变更日志

---

## 为什么用 GitHub

| 痛点 | Git/GitHub 的解决方案 |
|:-----|:-----|
| 代码传来传去，不知道哪个是最新版 | **单一可信源**：GitHub 上的 `main` 分支永远是最新稳定版 |
| "我改了 A，你改了 B，合并时覆盖了对方的代码" | **版本控制**：每次修改都有记录，冲突可追溯可回滚 |
| 上个赛季的代码找不到了 | **Tag / Release**：每个赛季打标签，历史代码永久存档 |
| 新队员不知道怎么上手 | **README + Wiki**：文档和代码放在一起，新人自学 |
| 不知道谁改了哪行、为什么改 | **Blame + Commit Message**：每一行代码都能追溯到作者和原因 |

> **一句话**：GitHub 是你们的 **代码保险箱 + 协作中枢 + 知识库**。

---

## 环境搭建

### 1. 注册 GitHub 账号

- 打开 [github.com](https://github.com)，用邮箱注册
- **用户名建议**：`RM-战队缩写-姓名拼音`，如 `RM-Cyan-YuShuCheng`
- 注册后把用户名发给队长，让队长拉你进 [山海机甲代码仓库](https://github.com/HebutMas)

### 2. 安装 Git

| 系统 | 安装方式 |
|:--|:--|
| **Windows** | 下载 [Git for Windows](https://git-scm.com/download/win)，一路默认安装即可 |
| **macOS** | 终端执行 `brew install git` 或下载 [Git for Mac](https://git-scm.com/download/mac) |
| **Linux (Ubuntu)** | `sudo apt install git` |

安装后在终端验证：

```bash
git --version
# 应该输出类似：git version 2.47.0
```

### 3. 配置 Git 身份

**每次提交都会带上你的名字和邮箱**，方便追溯是谁写的代码。

```bash
git config --global user.name "你的GitHub用户名"
git config --global user.email "你的GitHub注册邮箱"
```

### 4. 配置终端代理（重要！）

> **如果你用代理上网，这步必看。** Git 走终端，**不会自动使用系统代理**。不配的话 `git push` / `git clone` 会卡住或超时。

```bash
# 设置代理（端口按你代理工具的实际端口改）
git config --global http.proxy http://127.0.0.1:7899
git config --global https.proxy http://127.0.0.1:7899

# 取消代理
git config --global --unset http.proxy
git config --global --unset https.proxy
```

端口在你的代理工具（Clash / V2Ray 等）里找，常见 7890、7899、1080、10809。`git push` 卡在 `Connecting to github.com` 超时就是代理没配。

### 5. 安装 GitHub CLI（可选，推荐）

**Git 和 gh 的区别**：Git 是版本控制工具（管 commit/branch/merge），gh 是 GitHub 官方命令行工具（管 PR/Issue/CI）。你用 Git 把代码推上去，用 gh 在终端里直接建 PR、看 CI 结果，不用开浏览器。

```bash
# 安装
winget install GitHub.cli   # Windows
gh auth login               # 安装后登录（会引导你打开浏览器授权）
```

**常用示例**：

```bash
gh pr create --title "feat: 云台PID调参" --body "描述"   # 创建 PR
gh pr list                                                  # 列出 PR
gh pr checkout 12                                           # 切到 #12 的分支
gh pr merge 12 --squash --delete-branch                    # 合并并删分支
gh repo fork HebutMas/rm_control --clone                   # 一键 fork 并 clone
```

> 如果使用ai_agent，可以安装 GitHub CLI 技能（SkillHub: `github`），让 AI 直接帮你操作 PR、Issue 和 CI。

---

## Git 核心概念一张图

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

| 概念 | 一句话解释 |
|:-----|:-----|
| **仓库 (Repo)** | 一个项目的"文件夹"，带有完整历史记录 |
| **提交 (Commit)** | 一次保存操作，相当于游戏的"存档点" |
| **分支 (Branch)** | 一条独立的开发线，你可以安全地在上面做实验 |
| **推送 (Push)** | 把本地代码上传到 GitHub |
| **拉取 (Pull)** | 把 GitHub 上的最新代码下载到本地 |
| **合并 (Merge)** | 把两条分支的改动合到一起 |
| **克隆 (Clone)** | 把远程仓库完整下载到本地（含全部历史） |
| **PR (Pull Request)** | "我写好了，请审核并合并到主分支" 的请求 |

---

## 本地仓库与单人开发

### 最简流程

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

> **写完一个功能就 commit，不要攒一天再提交！** 一个 commit 只做一件事——写完云台 PID 就 commit 一次，写完底盘功率再 commit 一次。这样出问题能精确回滚，Code Review 也看得过来。

### 急救：commit 完才发现漏了一个文件

```bash
# 补充提交（不新建 commit，追加到上一个 commit 里）
git add 漏掉的文件.c
git commit --amend --no-edit   # --no-edit: 不改 commit message

# 如果已经 push 了：
git push --force-with-lease    # 安全版强制推送，不会覆盖队友的新 commit
```

### 急救：提交信息写错了

```bash
# 修改最近一次提交信息（还没 push 时）
git commit --amend -m "新的提交信息"

# 已经 push 过的要改写历史：
git commit --amend -m "新的提交信息"
git push --force-with-lease
```

---

## Commit 提交规范

> **提交规范是纪律，不是建议。** 一眼看 commit log 就能知道每个提交干了什么，这是团队协作的基本要求。

### Commit Message 格式

```text
<type>: <简短描述>
```

| type | 含义 | 示例 |
|:-----|:-----|:-----|
| `feat` | 新功能 | `feat: 云台PID角度环实现` |
| `fix` | Bug 修复 | `fix: 串口通信丢帧问题` |
| `refactor` | 代码重构（不改功能） | `refactor: 提取功率阈值到配置头文件` |
| `docs` | 文档更新 | `docs: 补充CAN通信协议说明` |
| `test` | 测试相关 | `test: 添加功率限制单元测试` |
| `chore` | 构建/工具/配置 | `chore: 配置Git LFS追踪规则` |
| `hotfix` | 紧急修复 | `hotfix: 比赛前裁判系统兼容` |

### 命名原则

- **一句话说清楚改了什么**，不要写"update"、"修改"这种废话
- **中英文均可**，但要和团队风格统一
- **一个 commit 只做一件事**——写完 PID 就 commit，不要把 PID 和底盘功率混在一个 commit 里
- 分支命名也遵循同样的 type 前缀（见[[#分支管理规范|分支管理规范]]）

---

## 分支管理规范

### 仓库分支说明

战队仓库使用两种分支模式：

- `mas_embedded_threadx` 使用 `main + dev` 双主线。日常开发从 `dev` 创建功能分支，完成后向 `dev` 提 PR；`main` 保存稳定版本。
- `RM_note` 只使用 `main`。修改从 `main` 创建功能分支，完成后向 `main` 提 PR。

无论哪种模式，都不要直接在 `main` 或 `dev` 上修改和推送。

### 为什么用分支

> **永远不要直接在 `main` 或 `dev` 上写代码。** 哪怕只改一行字，也要先开分支。分支是你的安全实验场，搞砸了删掉就好，不影响别人。

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

### 创建与切换分支

以下示例默认以 `mas_embedded_threadx` 的 `dev` 为目标分支；操作 `RM_note` 时将目标分支替换为 `main`。

```bash
# 从最新的 dev 创建功能分支
git checkout dev
git pull origin dev
git checkout -b feature/云台PID控制

# 写代码...小步 commit...
git add . && git commit -m "feat: 云台PID角度环实现"
```

### 开发完合并后删除分支

> **开发完、验证完、合并完之后，自己负责删掉自己的 feature 分支。** 不要留一堆废弃分支在仓库里。

```bash
# 合并完成后
git checkout dev                           # 切回 dev
git pull origin dev                        # 拉取包含你代码的最新 dev
git branch -d feature/云台PID控制           # 删除本地分支
git push origin --delete feature/云台PID控制  # 删除远程分支
```

### 分支保护规则（由队长在 GitHub 设置）

| 规则 | 说明 |
|:-----|:-----|
| `main` 分支不允许直接推送 | 必须通过 PR 合并 |
| PR 至少需要 1 人 Review 通过 | 防止低级错误直接上主分支 |
| 合并前必须通过 CI 检查 | （如果有配置自动构建/测试） |

### 急救：不小心在主分支上直接写了代码

```bash
# 把改动搬到新分支上
git checkout -b feature/新功能     # 基于当前位置建分支（改动还在）
git checkout dev                   # 切回 dev
git reset --hard origin/dev        # dev 回到远程版本（丢弃本地改动）
# 改动现在安全地留在 feature/新功能 分支上了
```

### 急救：写到一半要紧急切分支修 bug，又不想 commit 半成品

```bash
# git stash：暂存当前进度
git stash                    # 暂存所有改动
git stash -u                 # 暂存包括未跟踪的新文件
git stash save "底盘PID调试中"  # 带描述信息

# 切去修 bug...
git checkout dev
# 修 bug → commit → push → 切回来
git checkout feature/底盘

# 恢复暂存的进度
git stash pop      # 恢复最近一次 stash 并删除记录
git stash apply    # 恢复 stash 但保留记录（可以多次 apply）

# 查看 stash 列表
git stash list
```

---

## 分支合并：merge vs rebase

> **场景**：你在 feature 分支上写了一周代码，期间 dev 被队友合并了 5 个 PR。你的分支已经落后——**必须在合并前同步 dev 的最新代码**，否则会有大量冲突。

### 两种方案对比

| 维度 | `git merge` | `git rebase` |
|:-----|:-----|:-----|
| **原理** | 把 dev 的改动"合并进来"，生成一个 merge commit | 把你分支的 commit "搬到" dev 最新位置，重写历史 |
| **提交历史** | 保留真实时间线（你的 commit + merge commit 交错） | 线性历史，你的 commit 排在 dev 最新 commit **之后** |
| **冲突解决** | 一次性解决所有冲突 | 每个 commit 逐个解决冲突 |
| **适用场景** | 多人共用同一分支、想保留完整时间线 | 个人分支、追求干净线性历史 |
| **风险** | 低，不改变已有 commit | 改写 commit hash，**绝对不能** rebase 已被别人使用的分支 |

### 方案 A：merge（推荐新手首选）

```bash
git checkout feature/底盘功率控制    # 切到你的 feature 分支
git fetch origin                    # 拉取远程最新信息
git merge origin/dev                # 把 origin/dev 合并进你的分支

# 如果无冲突 → Git 自动生成 merge commit，完成
# 如果有冲突 → 手动解决 → git add → git commit
git add .
git commit -m "merge: 同步 dev 最新代码"
git push
```

### 方案 B：rebase（历史更干净，适合个人分支）

```bash
git checkout feature/底盘功率控制
git fetch origin
git rebase origin/dev               # 把你的 commit 搬到 origin/dev 最新位置

# 如果有冲突：
#   → Git 停下来，手动解决冲突 → git add
#   → git rebase --continue      （继续搬下一个 commit）
#   → 重复直到 rebase 完成
#
# 如果想放弃重来：
#   → git rebase --abort         （回到 rebase 之前的状态）

# 因为改写了提交历史，首次推送需要 force：
git push --force-with-lease
# ^-- 安全版：如果远程有你不知道的新 commit，会拒绝推送而不是默默覆盖
```

### 决策口诀

> **不确定就用 merge**；分支只有你一个人用 → rebase 更干净；分支有别人在用 → **绝对不要 rebase**。

### 急救：merge/rebase 到一半冲突太多，想放弃重来

```bash
git merge --abort       # 放弃本次 merge，回到 merge 前的状态
git rebase --abort      # 放弃本次 rebase
git cherry-pick --abort # 放弃本次 cherry-pick
```

---

## 实用 .gitignore 模板

> 以下模板覆盖嵌入式开发常见产物，直接复制到仓库根目录 `.gitignore` 即可。

```gitignore
# === 编译产物 ===
build/
cmake-build-*/
*.o
*.obj
*.elf
*.hex
*.bin
*.map
*.list

# === Keil / IAR 中间文件 ===
*.uvguix.*
*.scvd
JLinkLog.txt
*.bak
*.dep
*.d
*.crf
*.iex
*.htm
*.lnp
*.sct
*.ini
*.dbgconf
MDK-ARM/DebugConfig/
MDK-ARM/RTE/
MDK-ARM/*.uvguix.*
MDK-ARM/*.bak
MDK-ARM/*.dep

# === CMake ===
CMakeFiles/
CMakeCache.txt
cmake_install.cmake
Makefile

# === IDE 配置 ===
.vscode/
.idea/
*.swp
*.swo
*~

# === 操作系统 ===
.DS_Store
Thumbs.db
desktop.ini

# === Python ===
__pycache__/
*.pyc
venv/
.venv/

# === 调试 / 日志 ===
*.log
*.jdebug
*.jdebug.user
OzoneSettings.jdebug

# === 固件二进制（如需版本管理则移除此项，改用 Git LFS） ===
# *.hex
# *.bin
```

> **原则**：编译产物和 IDE 中间文件一定要忽略，否则每次 `git status` 一堆噪音，而且不同人编译出来的二进制不同，合并时必冲突。源码、配置、文档**不要忽略**。

---

## 多人协作开发流程

战队成员分两种角色，协作流程不同：

### Member（普通成员）— Fork 模式

普通成员没有主仓库的直接写入权限，需要先 fork 到自己的 GitHub 账号下开发，测试通过后 PR 回主仓库。

```bash
# 1. GitHub 网页上点 Fork，把战队仓库 fork 到自己账号下
# 2. Clone 你自己的 fork（不是战队仓库！）
git clone https://github.com/你的用户名/mas_embedded_threadx.git
cd mas_embedded_threadx

# 3. 添加战队仓库为 upstream（用于同步最新代码）
git remote add upstream https://github.com/HebutMas/mas_embedded_threadx.git

# 4. 从最新的 dev 创建功能分支
git checkout dev
git fetch upstream
git merge upstream/dev              # 同步上游 dev 的最新代码
git checkout -b feature/云台PID控制

# 5. 写代码...小步 commit...
git add . && git commit -m "feat: 云台PID角度环实现"

# 6. 推到自己的 fork
git push -u origin feature/云台PID控制

# 7. 去 GitHub 网页上向主仓库发 PR（base: dev, compare: feature/云台PID控制）
```

### Owner（仓库管理员）— 主仓库分支模式

Owner 有主仓库的直接写入权限，**不需要 fork**，但**仍然不能直接在 dev 上写代码**。流程是：在主仓库开 feature 分支 → push → merge 到 dev → 删分支。

```bash
# 1. 确保从最新的 dev 起步
git checkout dev
git pull origin dev

# 2. 创建功能分支
git checkout -b feature/底盘功率控制

# 3. 写代码...小步 commit...
git add . && git commit -m "feat: 添加功率限制核心逻辑"

# 4. 推到主仓库的 feature 分支
git push -u origin feature/底盘功率控制

# 5. 验证通过后，合并到 dev
git checkout dev
git pull origin dev                 # 再拉一次，防止期间有人更新了 dev
git merge feature/底盘功率控制
git push origin dev

# 6. 删除自己的 feature 分支
git branch -d feature/底盘功率控制
git push origin --delete feature/底盘功率控制
```

> **特别强调**：无论你是 member 还是 owner，**对 dev 分支的修改一定要先开 feature 分支，在分支上开发验证，确认没问题后再合并回 dev**。绝对不要直接在 dev 上改代码。

### 两人协作 — 互相拉

两个人一起开发时不需要走 PR，直接从对方的分支拉：

```bash
git remote add teammate https://github.com/队友用户名/mas_embedded_threadx.git
git fetch teammate
git merge teammate/feature/云台PID调参
```

### 同步主线更新

当你在 feature 分支开发期间，目标分支被其他人更新时，合并前需要先同步最新代码。

Member 从原始仓库 `upstream` 同步：

```bash
git switch feature/云台PID控制
git fetch upstream
git merge upstream/dev
# 个人分支也可以使用：git rebase upstream/dev
```

Owner 从主仓库 `origin` 同步：

```bash
git switch feature/云台PID控制
git fetch origin
git merge origin/dev
# 个人分支也可以使用：git rebase origin/dev
```

同步后再推送功能分支。使用 rebase 时，需要执行：

```bash
git push --force-with-lease
```

---

## 多人协作注意事项

### 冲突处理

冲突文件里会出现这种标记，手动编辑保留正确代码、删掉标记即可：

```text
<<<<<<< HEAD
你的代码
=======
队友的代码
>>>>>>> feature/xxx
```

### 急救：已 push 的错误 commit 要覆盖

```bash
git commit --amend -m "修正后的提交信息"    # 或 --no-edit 只改内容
git push --force-with-lease
# ^-- 永远只用 --force-with-lease，不要用 --force
#     前者会在远程有你不知道的新 commit 时拒绝推送，后者会默默覆盖
```

### Code Review 要点

每个 PR 至少一个人看过才能合入 `main`。Review 时重点关注：

| 优先级 | 检查项 |
|:-------|:-------|
| 必看 | **逻辑正确性**：边界条件处理了吗？除零保护有吗？ |
| 必看 | **安全性（RM 专属）**：这段代码会不会让电机暴走？功率限制有没有被绕过？ |
| 建议 | **可读性**：变量名能一眼看懂吗？关键逻辑有注释吗？ |
| 建议 | **结构设计**：硬编码的阈值应该提成 `#define` 吗？ |

**原则**：对事不对人——"这段逻辑有问题"而不是"你写错了"。说明为什么——不只是"改成 X"，而是"因为 Y 情况会出问题，所以建议 X"。

### 常见失误预防

| 失误 | 后果 | 预防 |
|:-----|:-----|:-----|
| 直接在 dev 上写代码 | 难以隔离，push 时可能被拒 | 写代码前先 `git branch` 确认当前分支 |
| push 前没同步 dev | PR 冲突多，CI 可能基于旧代码测试 | PR 页面的 "out-of-date" 警告 |
| rebase 后用了 `--force` | 可能默默覆盖队友的 commit | 永远只用 `--force-with-lease` |
| 忘记 pull 就开始写 | 基于过时代码开发 | 把 `git pull` 当肌肉记忆，坐下就先拉 |
| 一个 commit 改了几十个文件 | 无法回滚、无法 bisect | 一个 commit 只做一件事 |
| 合并完不删分支 | 仓库一堆废弃分支 | 合并后立即删本地+远程分支 |

---

## PR 描述模板

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

> `Closes #42` 写进 PR 描述后，PR 合并时会**自动关闭** #42 号 Issue。也支持 `Fixes` / `Resolves`。

---

## 常见问题排查

### Q1: `git push` 时提示权限不足

原因：你没被加入仓库的 Collaborator 或 Team。解决：联系队长，在仓库 Settings → Collaborators 里添加你。如果是 fork 模式，确认你推的是自己的 fork 而不是上游仓库。

### Q2: 合并时出现冲突 (Conflict)

手动编辑冲突文件，保留正确代码、删掉 `<<<<<<<` / `=======` / `>>>>>>>` 标记，然后 `git add` + `git commit`。冲突太多想放弃重来 → `git merge --abort`。

### Q3: 不小心提交了不该提交的文件 / 想撤销提交

```bash
git reset --soft HEAD~1    # 撤销提交但保留改动
```

补提交漏掉的文件 → `git commit --amend --no-edit`（见[[#本地仓库与单人开发|单人开发]]的急救部分）。

### Q4: 我想回到之前的某个版本看看

```bash
git log --oneline --graph      # 查看提交历史
git checkout <commit-hash>     # 临时切换到某个版本（只读）
git checkout dev               # 回到最新版本
```

### 急救：误删 commit / `reset --hard` 后后悔了

```bash
# reflog 记录了你所有的 HEAD 移动——这是 Git 的"时光机"
git reflog
# 输出：
# abc1234 HEAD@{0}: reset: moving to HEAD~1
# def5678 HEAD@{1}: commit: 底盘功率限制功能  ← 你想找回的 commit

# 回到那个 commit
git reset --hard def5678              # 或者
git checkout -b 恢复分支 def5678       # 更安全：先建个分支看看
```

> **reflog 是 Git 最强的后悔药**——只要 commit 过，90 天内都可以找回（除非执行了 `git gc`）。

### 急救：main/dev 分支上发现一个 bug，不确定是哪个 commit 引入的

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
git bisect reset                  # 退出 bisect
```

### 急救：只需要把另一个分支上的某几个 commit 搬过来

```bash
# cherry-pick：精确搬运指定 commit
git cherry-pick abc1234                    # 搬一个 commit
git cherry-pick abc1234..def5678           # 搬一段连续的 commit（不含 start，含 end）
git cherry-pick abc1234^..def5678          # 搬一段连续的 commit（含 start）
```

### 急救命令速查表

| 事故 | 命令 |
|:-----|:-----|
| commit 漏文件 | `git commit --amend --no-edit` |
| commit message 写错 | `git commit --amend -m "新信息"` |
| 在主分支上误写代码 | `git checkout -b 新分支` → `git checkout dev` → `git reset --hard origin/dev` |
| 写到一半要切分支 | `git stash` → 处理完 → `git stash pop` |
| 误删 commit / reset 后悔 | `git reflog` → `git checkout -b 恢复 <hash>` |
| 找到引入 bug 的 commit | `git bisect start` → `bisect bad`/`good` |
| 搬指定 commit 到当前分支 | `git cherry-pick <hash>` |
| 放弃正在进行的 merge | `git merge --abort` |
| 已 push 的错误 commit 要覆盖 | `git commit --amend` → `git push --force-with-lease` |

> **最重要的急救原则**：不确定时**先建分支**（`git checkout -b 试试看`），分支里随便折腾，搞砸了删掉分支就好。

---

## GitHub Wiki 与 Pages

### 战队代码和文档放在哪

| 内容 | 位置 | 说明 |
|:-----|:-----|:-----|
| 电控代码 | `github.com/HebutMas/mas_embedded_threadx` | 主仓库，含 dev/main 分支 |
| 电控知识库 | `github.com/HebutMas/RM_note` → `rm-electric-knowledge-base/` | 代码解析、踩坑记录、环境配置 |
| 视觉代码 | `github.com/HebutMas/mas_vision` | 视觉算法和模型 |
| 导航代码 | `github.com/HebutMas/mas_nav` | 导航相关 |
| Git 使用指南 | `RM_note` 仓库根目录 | 本文档 |

### Wiki：内部知识库

GitHub Wiki 本质上也是一个 Git 仓库，可以直接在网页上编辑，也可以本地克隆：

```bash
git clone https://github.com/HebutMas/rm_control.wiki.git
cd rm_control.wiki

# 写 Markdown 文件 → push → 自动更新 Wiki 页面
echo "# 底盘调参手册" > 底盘调参手册.md
git add . && git commit -m "docs: 初始化底盘调参手册"
git push
```

**Wiki 适合放**：调参手册、通信协议文档、硬件接线规范、新人入门 SOP、赛后复盘。

### Pages：对外知识库 / 战队主页

推荐使用 **MkDocs + Material 主题**，学习成本极低：

```bash
# 1. 安装
pip install mkdocs mkdocs-material

# 2. 配置 mkdocs.yml
# 3. 本地预览
mkdocs serve   # 打开 http://localhost:8000

# 4. 部署到 GitHub Pages
mkdocs gh-deploy
```

> **建议分工**：Wiki 放"活文档"（频繁更新，随代码一起迭代），Pages 放"归档内容"（赛季总结、开源发布）。两者不冲突，可以用同一个仓库同时维护。

---

## GitHub Actions — 自动化编译测试

GitHub Actions 让你在 **push 代码或创建 PR 时自动运行脚本**——自动编译、自动测试、自动格式化检查，不用人手点。

```
你 push 代码 → GitHub Actions 自动触发 → 编译 + 测试 → 结果贴到 PR 上
                                              ↓ 失败
                                         自动通知你修
```

### 核心概念：Workflow 文件

在仓库中创建 `.github/workflows/<名称>.yml`，GitHub 会自动识别并执行。

### 实战示例：自动编译 STM32 项目

```yaml
# .github/workflows/build.yml
name: 编译检查

on:
  push:
    branches: [main, dev]
  pull_request:
    branches: [main, dev]

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

### 实用技巧

| 技巧 | 做法 |
|:-----|:-----|
| **跳过 CI** | commit message 中含 `[skip ci]` 则跳过本次触发 |
| **免费额度** | 公共仓库无限免费，私有仓库每月 2000 分钟（2024） |
| **自托管 Runner** | 在实验室主机上装 GitHub Actions Runner，可以连接真实硬件做测试 |

> **战队最值得做的 CI**：编译检查。一个 workflow 不到 30 行 YAML，但能拦截 80% 的低级错误（忘提交文件、编译不过）。

---

## 技术储备（未来方向）

> 以下内容战队暂未使用，作为技术储备。当需求出现时可参考启用。

### Git LFS — 大文件管理

Git 对二进制大文件的处理极其低效——每次 `git clone` 都会下载**所有版本的完整副本**。Git LFS 的思路：仓库里只存一个"指针文件"（约 130 字节），真正的二进制文件存在 LFS 服务器上，按需下载。

```bash
# 安装与初始化
git lfs install
# 指定要追踪的文件类型
git lfs track "*.step" "*.sldprt"    # 机械图纸
git lfs track "*.pth" "*.onnx"       # 视觉模型
git add .gitattributes
git commit -m "chore: 配置 Git LFS 追踪规则"
```

适用场景：机械图纸（.step/.sldprt）、视觉模型（.pth/.onnx）、固件二进制（.bin/.hex）。不适用于经常修改的源码文件。

GitHub 免费提供 1GB 存储 + 1GB/月流量（2024），超出需付费。建议仓库刚开始就配 LFS，不要等到仓库已经很大再迁移。

### Release & Tag — 赛季版本管理

赛季过程中你会频繁改动代码。到了比赛日，你需要精确知道**哪一版代码是赛场上跑的**——Tag 就是这个"书签"。

```bash
# 给当前 commit 打标签
git tag -a v2025.1.0 -m "分区赛出战版本"
git push origin v2025.1.0

# 查看所有标签
git tag -n9

# 回到某个标签的代码
git checkout v2025.1.0

# 基于标签创建热修复分支
git checkout -b hotfix/裁判系统兼容 v2025.1.0
```

版本号规范：`v<赛季年份>.<主版本号>.<修订号>`

| 版本段 | 何时递增 | 示例 |
|:-------|:---------|:-----|
| 赛季年份 | 新赛季 | `v2024.x.x` → `v2025.1.0` |
| 主版本号 | 重大功能变更 | `v2025.1.0` → `v2025.2.0` |
| 修订号 | Bug 修复 / 参数微调 | `v2025.1.0` → `v2025.1.1` |

GitHub Release 可以在 Tag 之上附带编译好的 `.hex` / `.bin` 文件，方便直接烧录。**实战建议**：每次比赛前后打一个 Tag，Release 里附上固件 + 调参配置文件。

---

## 推荐资源

- [Git 官方文档](https://git-scm.com/doc)
- [GitHub Skills](https://skills.github.com/) — 交互式学习
- [Learn Git Branching](https://learngitbranching.js.org/?locale=zh_CN) — 可视化学习 Git 分支（强烈推荐）
- [Pro Git 中文版](https://git-scm.com/book/zh/v2) — 最权威的 Git 书籍

---

> **开始你的第一步**：打开终端，执行 `git --version`，然后找队长要仓库地址，`git clone` 下来！
>
> 有问题随时在战队群里问，或者在知识库 `rm-electric-knowledge-base/04_notes/pitfalls.md` 中补充。
