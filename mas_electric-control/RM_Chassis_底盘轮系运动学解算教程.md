# 底盘轮系运动学解算教程 —— 基于 mas_embedded_threadx

## 目录

1. [坐标系与基础约定](#1-坐标系与基础约定)
2. [舵轮底盘 (Swerve Drive)](#2-舵轮底盘-swerve-drive)
3. [麦克纳姆轮底盘 (Mecanum Wheel)](#3-麦克纳姆轮底盘-mecanum-wheel)
4. [全向轮底盘 (Omni Wheel)](#4-全向轮底盘-omni-wheel)
5. [差速轮底盘 (Differential Drive)](#5-差速轮底盘-differential-drive)
6. [里程计：从轮速到位姿](#6-里程计从轮速到位姿)
7. [逆运动学的实际调用流程](#7-逆运动学的实际调用流程)
8. [底盘模式与控制集成](#8-底盘模式与控制集成)
9. [功率限制与电机保护](#9-功率限制与电机保护)
10. [调试与验证](#10-调试与验证)

---

## 1. 坐标系与基础约定

### 1.1 底盘坐标系

本项目采用**机器人中心坐标系**：

```
        Y+ (左)
         ↑
         │
    ┌────┼────┐
    │ 0  │  3 │     0: 左前 (LF)
    │    │    │     1: 左后 (LB)
    │ 1  │  2 │     2: 右后 (RB)
    │    │    │     3: 右前 (RF)
    └────┼────┘
         │
         └────────→ X+ (前)
```

**符号约定**：
| 变量 | 正方向 | 含义 |
|------|--------|------|
| `vx` | +X（前） | 前进速度 (m/s) |
| `vy` | +Y（左） | 左移速度 (m/s) |
| `vw` / `wz` | +逆时针 | 旋转角速度 (rad/s) |

### 1.2 电机编号约定

在本项目的 `Chassis_Swerve_Calc()` 中，8 个电机分为两组：

```
motors[0..3] = 驱动电机 (M3508) — 控制轮子转速
motors[4..7] = 转向电机 (GM6020) — 控制轮子朝向
```

**轮子索引**：`0=LF`, `1=LB`, `2=RB`, `3=RF`

### 1.3 运动学基本概念

**逆运动学 (Inverse Kinematics)**：已知期望的底盘速度 `[vx, vy, vw]`，求每个轮子的转速和转向角。
> 用途：遥控器输入 → 底盘速度指令 → 电机控制指令

**正运动学 (Forward Kinematics)**：已知每个轮子的实际转速和转向角，求底盘实际速度 `[vx, vy, vw]`。
> 用途：电机编码器反馈 → 底盘实际速度 → 里程计

**里程计 (Odometry)**：对底盘速度进行积分，得到机器人在世界坐标系中的位置和姿态 `[x, y, yaw]`。
> 用途：导航、自主移动、位置闭环控制

---

## 2. 舵轮底盘 (Swerve Drive)

![Swerve Drive 运动学模型](https://docs.wpilib.org/en/2022/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html)
*▲ Swerve Drive 运动学模型 — 四个独立转向模块 (图片来源: WPILib)*

### 2.1 机械结构

舵轮底盘由 **4 个可独立转向的驱动轮** 组成。每个轮子模块包含：
- 1 个**驱动电机**（M3508，控制轮子转速，通过减速器驱动轮毂）
- 1 个**转向电机**（GM6020，控制轮子朝向角度）

```
俯视图：                每个轮子模块（侧视）：
  LF ●────────● RF         ┌──────────┐
     │        │            │ GM6020   │ ← 转向电机（控制朝向）
     │  机身  │            │   ↓      │
     │        │            │ M3508    │ ← 驱动电机（控制转速）
  LB ●────────● RB         │   ‖      │
                           │  ◎ 轮子  │
                           └──────────┘
```

**关键几何参数**：
| 参数 | 哨兵值 | 含义 |
|------|--------|------|
| `wheel_r` | 0.5 m | 轮子投影点到几何中心的距离（旋转力臂） |
| `radius_wheel_m` | 0.12 m | 轮子半径 |
| `decele_ratio` | 16:1 | 驱动电机减速比 |
| `align_rad[4]` | 机械零点对齐角 | 编码器零点与物理前向之间的偏差 |

**为什么是 4 个独立舵轮？**
- 每个轮子的朝向和转速可以**独立控制**
- 可以实现任意方向的平移（全向移动）同时独立控制旋转
- 比麦克纳姆轮效率更高（无侧向滑动损失）
- 哨兵使用舵轮是因为需要高精度位置控制和低速平稳性

### 2.2 逆运动学：底盘速度 → 轮子速度+角度

**文件**：`modules/algorithm/chassis_type.c` → `Chassis_Swerve_Calc()`

#### 步骤 1：速度分解

将底盘中心的 `[vx, vy, vw]` 分解为每个轮子位置处的局部速度：

```
a = wheel_r × 0.707  ← 轮子在 X 和 Y 方向的投影距离

各轮子位置处的局部速度（含旋转分量）：

LF (i=0):  vx_local = vx + vw × a     vy_local = vy + vw × a
LB (i=1):  vx_local = vx + vw × a     vy_local = vy - vw × a
RB (i=2):  vx_local = vx - vw × a     vy_local = vy - vw × a
RF (i=3):  vx_local = vx - vw × a     vy_local = vy + vw × a
```

**物理含义**：当底盘逆时针旋转（vw > 0）时：
- 左前轮（LF）：向右前方运动（vx+, vy+）
- 右前轮（RF）：向右后方运动（vx-, vy+）
- 旋转贡献量 = `vw × 力臂`

#### 步骤 2：速度矢量 → 轮速 + 转向角

```
对每个轮子 i：

线速度幅值:   |v| = √(vx_local² + vy_local²)

驱动轮目标转速 (rad/s):
  target_speed_rad = |v| / radius_wheel_m × decele_ratio

转向电机目标角度 (rad):
  vector_rad = atan2(vy_local, vx_local)
  target_abs_angle = align_rad[i] + vector_rad
```

**为什么是 `atan2(vy_local, vx_local)`？**

`atan2(y, x)` 返回的是从 +X 轴逆时针测量的角度。因为 vx_local 和 vy_local 分别是轮子应该向 X 和 Y 方向运动的速度分量，`atan2(vy, vx)` 恰好给出了轮子应该指向的方向角。

#### 步骤 3：最短路径优化

当轮子需要从当前角度转到目标角度时，有两种选择：
- 顺时针转 170°（短路径）
- 逆时针转 190°（长路径）+ 反转驱动方向

```c
// 核心代码 (chassis_type.c 第 82-98 行)
float angle_diff = target_angle - current_angle;

if (angle_diff > PI/2 || angle_diff < -PI/2) {
    // 最优路径 > 90°：反转驱动方向 + 调整目标角 180°
    target_speed_rad = -target_speed_rad;
    target_angle = wrap_pi(target_angle + PI);
}
```

**这个优化非常关键**：
- 避免了 170° 的大角度转动（慢且可能超限）
- 改为转 10° + 反转驱动方向（快且平滑）
- 原理：轮子旋转 180° + 反转驱动方向 = 相同的运动效果

#### 步骤 4：零速保持

```c
if (|v| < 1e-6) {
    // 速度为零时，保持上一个有效角度
    // 避免不必要的角度重置和抖动
    target_speed_rad = 0;
    target_angle = last_valid_angle[i];
}
```

#### 完整逆运动学实现

```c
void Chassis_Swerve_Calc(
    const Chassis_Swerve_Config_s *config,
    float vx, float vy, float vw,           // 输入: 底盘速度
    float *target_speed, float *target_angle, // 输出: 电机指令
    const float *current_angle)              // 当前转向角度
{
    float a = config->wheel_r * 0.707f;
    
    // 每个轮子在 X/Y 方向的投影坐标
    float px[4] = { a,  a, -a, -a};  // LF, LB, RB, RF 的 X 坐标
    float py[4] = { a, -a, -a,  a};  // LF, LB, RB, RF 的 Y 坐标
    
    for (int i = 0; i < 4; i++) {
        // 速度分解
        float vx_local = vx - vw * py[i];  // vx + vw×a (带符号)
        float vy_local = vy + vw * px[i];  // vy + vw×a (带符号)
        
        // 线速度幅值
        float velocity = sqrtf(vx_local*vx_local + vy_local*vy_local);
        
        // 驱动电机转速
        if (velocity > 1e-6f) {
            target_speed[i] = velocity / config->radius_wheel_m * config->decele_ratio;
        } else {
            target_speed[i] = 0;
        }
        
        // 转向电机角度
        if (velocity > 1e-6f) {
            float vector_rad = atan2f(vy_local, vx_local);
            float target = config->align_rad[i] + vector_rad;
            
            // 最短路径优化
            float diff = target - current_angle[i];
            // 角度归一化到 [-PI, PI]
            while (diff > PI)  diff -= 2*PI;
            while (diff < -PI) diff += 2*PI;
            
            if (diff > PI/2 || diff < -PI/2) {
                target_speed[i] = -target_speed[i];
                target = target + PI;
                while (target > PI)  target -= 2*PI;
                while (target < -PI) target += 2*PI;
            }
            target_angle[i] = target;
        }
    }
}
```

### 2.3 正运动学：轮子速度+角度 → 底盘速度

```c
void Chassis_Swerve_Fwd(
    const Chassis_Swerve_Config_s *config,
    const float *wheel_speed, const float *steer_angle, // 输入
    float *vx, float *vy, float *vw)                    // 输出
{
    float a = config->wheel_r * 0.707f;
    float px[4] = { a,  a, -a, -a};
    float py[4] = { a, -a, -a,  a};
    
    float R = config->radius_wheel_m / config->decele_ratio;
    // R = 轮子每弧度转角对应的线位移 (m/rad)
    
    *vx = 0; *vy = 0; *vw = 0;
    
    for (int i = 0; i < 4; i++) {
        float vi = wheel_speed[i] * R;  // 轮子线速度 (m/s)
        float cos_a = cosf(steer_angle[i]);
        float sin_a = sinf(steer_angle[i]);
        
        *vx += vi * cos_a;
        *vy += vi * sin_a;
        *vw += (px[i] * vi * sin_a - py[i] * vi * cos_a) / (config->wheel_r * config->wheel_r);
    }
    
    *vx /= 4.0f;
    *vy /= 4.0f;
    *vw /= 4.0f;
}
```

**vw 推导**：对于旋转运动，轮子 i 贡献的角速度正比于 `(位置 × 速度) / r²`（叉积的标量形式）。4 个轮子的贡献取平均。

### 2.4 舵轮的哨兵实际配置

```c
// apps/sentry/chassis_board/chassis_func.c
Chassis_Swerve_Config_s swerve_config = {
    .wheel_r        = 0.5f,   // 力臂 0.5m
    .radius_wheel_m = 0.12f,  // 轮径 12cm
    .decele_ratio   = 16.0f,  // M3508 减速比 16:1
    .align_rad = {
        4458 * 0.000767f,   // LF: 4458 ticks × 0.000767 rad/tick = 3.419 rad
        1010 * 0.000767f,   // LB: 0.775 rad
        7177 * 0.000767f,   // RB: 5.505 rad
        3756 * 0.000767f    // RF: 2.881 rad
    }
};
```

**`align_rad` 的物理含义**：
- 当转向电机编码器读数为 `align_rad[i]` 时，轮子恰好指向前方
- `0.000767 rad/tick` = `2π / 8192`（GM6020 编码器每圈 8192 个脉冲）
- 每个轮子的机械零点不同（制造公差 + 安装偏差）

---

## 3. 麦克纳姆轮底盘 (Mecanum Wheel)

![麦克纳姆轮力学矢量图](https://raw.githubusercontent.com/shurik179/gm0/master/source/docs/software/images/mecanum-drive/mecanum-drive-force-diagram.png)
*▲ 麦克纳姆轮力学分解 — 四个轮子的力矢量合成 (图片来源: GM0)*

### 3.1 机械结构

麦克纳姆轮是一种特殊设计的轮子，轮毂周围安装了**可自由旋转的小辊子**，辊子轴线与轮毂轴线成 **45°** 夹角。

```
单个麦克纳姆轮（侧视）：
    ┌──────────────────┐
    │   ○  ○  ○  ○  ○  │  ← 小辊子（可自由旋转）
    │  ╱  ╱  ╱  ╱  ╱  │  ← 45° 倾斜
    │ ╱  ╱  ╱  ╱  ╱   │
    │  ○  ○  ○  ○  ○  │
    └──────────────────┘
        轮毂（电机驱动）
```

**四轮布局（俯视）**：

```
       前
  LF ⊗──────⊗ RF      ⊗ = 辊子方向（从上方看）
       │  │            LF/RB: 辊子 ╱ 方向（左旋）
       │  │            RF/LB: 辊子 ╲ 方向（右旋）
  LB ⊗──────⊗ RB
       后
```

**为什么用麦克纳姆轮？**
- 4 个轮子不需要转向机构，结构简单
- 可以实现**全向移动**（前进、横移、斜移、旋转）
- 通过轮速的线性组合实现任意方向的运动
- 适用于平坦地面（RoboMaster 赛场大部分区域）

**为什么舵轮更好（哨兵选用）？**
- 麦克纳姆轮的辊子与地面是**滑动摩擦**（不是纯滚动），效率较低
- 高加速度时辊子打滑 → 运动学失真
- 舵轮的轮子始终与运动方向对齐 → 纯滚动摩擦 → 效率更高

### 3.2 逆运动学：底盘速度 → 轮子转速

**文件**：`modules/algorithm/chassis_type.c` → `Chassis_Mecanum_Calc()`

**核心公式**：

```c
void Chassis_Mecanum_Calc(
    const Chassis_Diff_Config_s *config,
    float vx, float vy, float vw,
    float *target_speed)  // 输出: 4 个轮子转速 (rad/s)
{
    float L = (config->wheel_base_x + config->wheel_base_y) / 2.0f;
    float speed_factor = config->decele_ratio / config->wheel_radius;
    
    target_speed[0] = (+vx - vy - vw * L) * speed_factor;  // LF
    target_speed[1] = (+vx + vy - vw * L) * speed_factor;  // LB
    target_speed[2] = (+vx + vy + vw * L) * speed_factor;  // RB
    target_speed[3] = (+vx - vy + vw * L) * speed_factor;  // RF
}
```

### 3.3 公式推导

麦克纳姆轮的运动学可以通过**辊子速度分解**推导。每个轮子有一个驱动速度（轮毂转速）和一个被动侧滑速度（辊子转速）。

**关键假设**：轮子与地面之间**没有纵向滑动**（辊子自由旋转方向无约束），只有**横向啮合**（垂直于辊子方向有约束）。

对每个轮子 i，其轮毂线速度 `v_i` 与底盘运动的关系为：

```
v_i = (vx + k_i·vy + L_i·vw) / R_eff

其中：
  k_i = ±1（取决于辊子方向：LF/RB 为 -1, RF/LB 为 +1）
  L_i = (wheel_base_x + wheel_base_y) / 2（等效力臂）
  R_eff = wheel_radius / decele_ratio
```

展开为 4 个轮子：

```
  v0 (LF) = (+vx - vy - vw·L) × factor
  v1 (LB) = (+vx + vy - vw·L) × factor
  v2 (RB) = (+vx + vy + vw·L) × factor
  v3 (RF) = (+vx - vy + vw·L) × factor
```

**验证几种典型运动**：

| 运动方向 | vx | vy | vw | LF | LB | RB | RF |
|---------|----|----|-----|-----|-----|-----|-----|
| 前进 | +1 | 0 | 0 | +向前 | +向前 | +向前 | +向前 |
| 左移 | 0 | +1 | 0 | **-向后** | +向前 | +向前 | **-向后** |
| 逆时针旋转 | 0 | 0 | +1 | **-向后** | **-向后** | +向前 | +向前 |

**横移验证**（vy > 0，向左平移）：

```
LF 轮：vx - vy = 0 - 1 = -1 → 向后转 ✓
LB 轮：vx + vy = 0 + 1 = +1 → 向前转 ✓
RB 轮：vx + vy = 0 + 1 = +1 → 向前转 ✓
RF 轮：vx - vy = 0 - 1 = -1 → 向后转 ✓

LF 和 RF 上的辊子产生向左的分力
LB 和 RB 上的辊子也产生向左的分力
→ 合成向左的平动
→ ✓ 横移效果
```

### 3.4 正运动学

```c
void Chassis_Mecanum_Fwd(
    const Chassis_Diff_Config_s *config,
    const float *wheel_speed,   // 4 个轮子实测转速 (rad/s)
    float *vx, float *vy, float *vw)
{
    float L = (config->wheel_base_x + config->wheel_base_y) / 2.0f;
    float R = config->wheel_radius / config->decele_ratio;
    
    *vx = (wheel_speed[0] + wheel_speed[1] + wheel_speed[2] + wheel_speed[3]) * R / 4.0f;
    *vy = (-wheel_speed[0] + wheel_speed[1] - wheel_speed[2] + wheel_speed[3]) * R / 4.0f;
    *vw = (-wheel_speed[0] - wheel_speed[1] + wheel_speed[2] + wheel_speed[3]) * R / (4.0f * L);
}
```

### 3.5 步兵3 实际配置

```c
// apps/infantry3/single_board/chassis_func.c
Chassis_Diff_Config_s mecanum_config = {
    .wheel_base_x = 0.5f,   // 前后轮距 0.5m
    .wheel_base_y = 0.3f,   // 左右轮距 0.3m
    .wheel_radius = 0.075f, // 轮径 7.5cm
    .decele_ratio = 16.0f   // M3508 减速比 16:1
};
```

---

## 4. 全向轮底盘 (Omni Wheel)

### 4.1 机械结构

全向轮与麦克纳姆轮不同——辊子轴线**垂直于**轮毂轴线（而非 45°）。多个双层/交错全向轮配合使用。

```
全向轮（侧视）：
    ┌──────────────────┐
    │  ═══  ═══  ═══  │  ← 小辊子（垂直于轮毂）
    │  ║║║  ║║║  ║║║  │
    │  ═══  ═══  ═══  │
    └──────────────────┘
```

**四轮布局**（常见 X 型布局）：

```
       前
  LF ◎──────◎ RF      每个轮子偏离中心 45°
       ╲  ╱
        ╲╱
        ╱╲
       ╱  ╲
  LB ◎──────◎ RB
       后
```

### 4.2 逆运动学

```c
void Chassis_Omni_Calc(
    const Chassis_Diff_Config_s *config,
    float vx, float vy, float vw,
    float *target_speed)
{
    float a = config->wheel_base_x / 2.0f;  // 前后半轴距
    float b = config->wheel_base_y / 2.0f;  // 左右半轴距
    float rot_diff = vw * (a - b);
    
    float speed_factor = config->decele_ratio / (config->wheel_radius * 0.7071f);
    // 0.7071 = cos(45°) — 轮子力方向与底盘坐标轴的夹角
    
    target_speed[0] = (+vx + vy + rot_diff) * speed_factor;  // LF
    target_speed[1] = (-vx + vy - rot_diff) * speed_factor;  // LB
    target_speed[2] = (-vx - vy + rot_diff) * speed_factor;  // RB
    target_speed[3] = (+vx - vy - rot_diff) * speed_factor;  // RF
}
```

**与麦克纳姆轮的关键区别**：
- 系数中出现了 `√2/2 = 0.7071`（因为轮子力方向偏离坐标轴 45°）
- 旋转项是 `vw × (a - b)` 而非 `vw × (a + b)/2`
- 轮子编号和旋转方向不同（LB 和 RF 为负 vx 贡献）

### 4.3 正运动学

```c
void Chassis_Omni_Fwd(
    const Chassis_Diff_Config_s *config,
    const float *wheel_speed,
    float *vx, float *vy, float *vw)
{
    float R = config->wheel_radius / config->decele_ratio;
    float a = config->wheel_base_x / 2.0f;
    float b = config->wheel_base_y / 2.0f;
    
    *vx = (wheel_speed[0] - wheel_speed[1] - wheel_speed[2] + wheel_speed[3]) * R / (2.0f * 1.414f);
    *vy = (wheel_speed[0] + wheel_speed[1] - wheel_speed[2] - wheel_speed[3]) * R / (2.0f * 1.414f);
    
    if (fabsf(a - b) > 1e-6f)
        *vw = (wheel_speed[0] - wheel_speed[1] - wheel_speed[2] + wheel_speed[3]) * R * 1.414f / (4.0f * (a - b));
    else
        *vw = 0;  // 方形底盘: vw 不可观
}
```

**方形底盘的 vw 退化**：当 `a == b`（前后轴距 = 左右轴距），旋转分量在正运动学中退化（分子为零），此时无法从轮速观测旋转速度。这是全向轮方形布局的数学特性。

---

## 5. 差速轮底盘 (Differential Drive)

![差速驱动运动学模型](https://upload.wikimedia.org/wikipedia/commons/8/8c/Differential_drive_kinematics.svg)
*▲ 差速驱动运动学模型 — ICR、轮距、航向角 (图片来源: Wikipedia)*

### 5.1 机械结构

差速底盘使用 2 个独立驱动的轮子 + 1 个或多个万向从动轮。

```
俯视图：
       前
  ┌──────────────┐
  │              │
  │   机身       │
  │              │
  │ ◎          ◎│  ← 左右驱动轮
  │              │
  │       ○      │  ← 万向从动轮（无动力）
  └──────────────┘
       后
```

本项目目前**未实现**差速底盘的代码，但它在 RoboMaster 中较为常见（工程机器人、部分哨兵方案）。以下是基于本项目架构的扩展设计。

### 5.2 逆运动学

```c
void Chassis_Diff_Calc(
    const Chassis_Diff_Config_s *config,
    float vx, float vw,              // 注意：差速底盘没有 vy（不能横移）
    float *target_speed)
{
    // 差速底盘只有 2 个驱动轮
    float R = config->wheel_radius / config->decele_ratio;
    float track_width = config->wheel_base_y;  // 左右轮距
    
    // 左轮线速度 = 前进速度 - 旋转产生的附加速度
    float v_left  = vx - vw * track_width / 2.0f;
    // 右轮线速度 = 前进速度 + 旋转产生的附加速度
    float v_right = vx + vw * track_width / 2.0f;
    
    target_speed[0] = v_left  / R;  // 左轮转速 (rad/s)
    target_speed[1] = v_right / R;  // 右轮转速 (rad/s)
}
```

**物理含义**：
- 前进：左右轮同速正转
- 原地右转：左轮正转，右轮反转
- 差速底盘的旋转中心可以在轮轴上的任意位置（取决于左右轮速比）

### 5.3 正运动学

```c
void Chassis_Diff_Fwd(
    const Chassis_Diff_Config_s *config,
    const float *wheel_speed,
    float *vx, float *vw)
{
    float R = config->wheel_radius / config->decele_ratio;
    float track_width = config->wheel_base_y;
    
    float v_left  = wheel_speed[0] * R;
    float v_right = wheel_speed[1] * R;
    
    *vx = (v_left + v_right) / 2.0f;
    *vw = (v_right - v_left) / track_width;
}
```

### 5.4 差速底盘的局限性

| 能力 | 舵轮 | 麦克纳姆 | 差速 |
|------|------|---------|------|
| 前进/后退 | ✅ | ✅ | ✅ |
| 原地旋转 | ✅ | ✅ | ✅ |
| 横向平移 | ✅ | ✅ | ❌ |
| 对角移动 | ✅ | ✅ | ❌ |
| 地面适应 | 好（纯滚动） | 差（辊子滑动） | 好（纯滚动） |
| 机构复杂度 | 高（8电机） | 低（4电机） | 最低（2电机） |

---

## 6. 里程计：从轮速到位姿

### 6.1 统一里程计公式

所有底盘类型使用相同的速度→位姿积分方法：

```c
void Chassis_Odom_Update(
    float vx, float vy, float vw,  // 当前时刻的底盘速度（来自正运动学）
    float dt,                       // 时间间隔 (s)
    float *x, float *y, float *yaw) // 位姿（累积更新）
{
    // 将体坐标系速度转换到世界坐标系
    float cos_yaw = cosf(*yaw);
    float sin_yaw = sinf(*yaw);
    
    // 世界坐标系位移
    *x   += (vx * cos_yaw - vy * sin_yaw) * dt;
    *y   += (vx * sin_yaw + vy * cos_yaw) * dt;
    *yaw += vw * dt;
    
    // yaw 归一化到 [-PI, PI]
    while (*yaw >  PI) *yaw -= 2.0f * PI;
    while (*yaw < -PI) *yaw += 2.0f * PI;
}
```

### 6.2 坐标变换解释

```
体坐标系速度 [vx, vy] 是世界坐标系中沿机器人当前朝向的速度。

要将它们映射到世界坐标系：
  dx_world = vx × cos(yaw) - vy × sin(yaw)
  dy_world = vx × sin(yaw) + vy × cos(yaw)

这是标准的 2D 旋转矩阵的逆（或转置）：
  [dx]   [cos(yaw)  -sin(yaw)]   [vx]
  [dy] = [sin(yaw)   cos(yaw)] × [vy]
```

### 6.3 三种底盘的里程计包装

```c
// 舵轮
void Chassis_Swerve_Odom(config, wheel_speed, steer_angle, dt, x, y, yaw) {
    float vx, vy, vw;
    Chassis_Swerve_Fwd(config, wheel_speed, steer_angle, &vx, &vy, &vw);
    Chassis_Odom_Update(vx, vy, vw, dt, x, y, yaw);
}

// 麦克纳姆
void Chassis_Mecanum_Odom(config, wheel_speed, dt, x, y, yaw) {
    float vx, vy, vw;
    Chassis_Mecanum_Fwd(config, wheel_speed, &vx, &vy, &vw);
    Chassis_Odom_Update(vx, vy, vw, dt, x, y, yaw);
}

// 全向轮
void Chassis_Omni_Odom(config, wheel_speed, dt, x, y, yaw) {
    float vx, vy, vw;
    Chassis_Omni_Fwd(config, wheel_speed, &vx, &vy, &vw);
    Chassis_Odom_Update(vx, vy, vw, dt, x, y, yaw);
}
```

### 6.4 里程计误差来源

| 误差源 | 影响 | 缓解方法 |
|--------|------|---------|
| 轮子打滑 | 位移偏小（驱动轮打滑）或偏大（刹车打滑） | 限制加速度、IMU 融合 |
| 轮径磨损 | 位移系统性偏小 | 定期测量轮径，调整 `wheel_radius` |
| 编码器分辨率 | 低速时量化误差 | 使用高分辨率编码器 |
| 机械间隙 | 转向角滞后 | 减小间隙、前馈补偿 |
| 地面不平 | 轮子载荷变化 → 有效轮径变化 | IMU 融合修正 |

---

## 7. 逆运动学的实际调用流程

### 7.1 从遥控器到电机指令（以哨兵为例）

```
遥控器 (SBUS 50Hz)
  │ 摇杆位置：[-1.0, +1.0]
  ▼
云台板 RemoteControlSet()
  │ 映射到 Chassis_Ctrl_Cmd_t
  │ vx = joy_x × CHASSIS_MAX_SPEED_MPS  (3.0 m/s)
  │ vy = joy_y × CHASSIS_MAX_SPEED_MPS
  │ wz = joy_z × CHASSIS_MAX_WZ
  ▼
缩放为 int8 (-10 ~ +10) → CAN 帧 → 底盘板
  │
  ▼
底盘板 robot_control_task (500Hz)
  │ 解码 int8 → float
  │ chassis_vx = (int8_vx / 10.0f) × 3.0f
  │
  │ 旋转矩阵：将云台系速度转到底盘系
  │ chassis_vx_rot = vx×cos(offset) - vy×sin(offset)
  │ chassis_vy_rot = vx×sin(offset) + vy×cos(offset)
  ▼
chassis_func()
  │ 根据 chassis_mode 选择控制模式
  │ chassis_follow_gimbal_yaw: PID 跟踪云台 Yaw
  │ chassis_rotate: wz = 3.0 rad/s
  │ ...
  ▼
Chassis_Swerve_Calc()
  │ 逆运动学 → target_speed[4], target_angle[4]
  ▼
Motor 控制
  │ motors[0..3] → M3508 速度环 (target_speed)
  │ motors[4..7] → GM6020 角度环 (target_angle)
  ▼
CAN 发送 (1ms 周期)
```

### 7.2 云台坐标系到底盘坐标系的旋转

```c
// chassis_func.c 第 264-265 行
// 当云台相对于底盘有偏航偏移时，需要将云台系的
// 速度指令旋转到底盘系执行
float offset_rad = offset_angle * (2.0f * PI / 8192.0f);
float cos_off = cosf(offset_rad);
float sin_off = sinf(offset_rad);

float chassis_vx = cmd->vx * cos_off - cmd->vy * sin_off;
float chassis_vy = cmd->vx * sin_off + cmd->vy * cos_off;
// chassis_wz 不变（旋转是绕 Z 轴的，不受偏航偏移影响）
```

**为什么需要这个旋转？**
- 遥控器的"前进"指令是相对于**云台当前朝向**的
- 但底盘执行是在**底盘坐标系**中的
- 如果云台偏转了 90°，遥控器的"前进"对底盘来说是"左移"
- 旋转矩阵补偿了这个差异

---

## 8. 底盘模式与控制集成

### 8.1 底盘模式枚举

```c
typedef enum {
    chassis_zero_force        = 0,  // 所有电机停止（泄力）
    chassis_follow_gimbal_yaw = 1,  // 跟随云台 Yaw（PID 对准）
    chassis_rotate            = 2,  // 恒速正转
    chassis_rotate_reverse    = 3,  // 恒速反转
    chassis_automode          = 4,  // 自主模式（哨兵专用）
} chassis_mode_e;
```

### 8.2 跟随云台模式（chassis_follow_gimbal_yaw）

```c
// 目标：保持底盘朝向与云台朝向一致
// 使底盘始终"面向敌人"
float yaw_error = gimbal_yaw - chassis_yaw;
float wz_cmd = PID_Calculate(&chassis_follow_pid, 0.0f, yaw_error);
// Kp=0.1, Ki=0, Kd=0.001

// 然后将 wz_cmd 作为旋转分量输入逆运动学
Chassis_Swerve_Calc(&config, cmd->vx, cmd->vy, wz_cmd, ...);
```

### 8.3 旋转模式（chassis_rotate）

```c
// 小陀螺模式：底盘持续旋转，使装甲板难以被击中
float wz = 3.0f;   // 正转 3 rad/s ≈ 0.5 rev/s
// 或
float wz = -8.0f;  // 反转 8 rad/s ≈ 1.3 rev/s（更高转速）

Chassis_Swerve_Calc(&config, 0, 0, wz, ...);
// vx=vy=0，纯旋转
```

---

## 9. 功率限制与电机保护

### 9.1 功率模型

`modules/MOTOR/POWER_CONTROL/power_control.c` 实现功率预算管理：

```c
// 电机功率模型
P = τ × ω + k1 × |ω| + k2 × τ² + k3

τ: 电机输出扭矩 (Nm)
ω: 电机转速 (rad/s)
k1: 库仑摩擦系数
k2: 铜损系数
k3: 静态功耗
```

### 9.2 哨兵的功率分配

```
总功率预算: 120W + 60J 缓冲能量

角色分配:
├── 驱动轮 (4×M3508, PC_ROLE_DRIVE): 
│   获得 (总功率 - 转向已用功率) 的剩余部分
│   参数: k1=0.132, k2=3.47, k3=1
│
└── 转向轮 (4×GM6020, PC_ROLE_STEER):
    获得 min(转向需求, 总功率×0.8) 的部分
    参数: k1=0.005, k2=12.98, k3=1
```

**功率限制的作用**：
- 防止超级电容过放
- 防止单个电机过热
- 优先级分配：转向 > 驱动（保持稳定 > 快速移动）

### 9.3 功率限制对运动学的影响

当功率不足时，电机输出被等比缩放：

```c
if (total_power_request > available_power) {
    float scale = available_power / total_power_request;
    for (int i = 0; i < num_motors; i++) {
        motor[i].torque *= scale;  // 等比降低所有电机扭矩
    }
}
```

这意味着在功率受限时，机器人的实际运动将比指令慢，但运动方向（速度比值）保持不变——因为所有电机被等比缩放。

---

## 10. 调试与验证

### 10.1 舵轮转向角验证

```c
// 发送纯前进指令
Chassis_Swerve_Calc(&config, 1.0f, 0, 0, ...);
// 期望：4 个轮子都指向前方 (target_angle ≈ align_rad[i])
// 验证：肉眼观察轮子朝向，或用角度尺测量

// 发送纯横移指令
Chassis_Swerve_Calc(&config, 0, 1.0f, 0, ...);
// 期望：4 个轮子都指向左侧 (target_angle ≈ align_rad[i] + 90°)
```

### 10.2 麦克纳姆轮极性验证

```c
// 在 chassis_func.c 中添加调试代码
// 逐个发送单一运动指令，观察轮子转动方向

// 前进: 4 个轮子全部前转 ✓
// 左移: LF 和 RF 反转, LB 和 RB 正转 ✓
// 右旋: LF 和 LB 反转, RF 和 RB 正转 ✓
```

### 10.3 里程计精度验证

```
将机器人放在已知距离的直线上：
1. 里程计归零
2. 前进 2 米
3. 检查里程计输出的位移

误差 < 5% → 合格
误差 > 10% → 检查轮径参数、打滑情况、编码器精度
```

### 10.4 常见问题

| 问题 | 可能原因 | 解决 |
|------|---------|------|
| 舵轮转向方向错误 | `align_rad` 符号反了 | 将 `align_rad[i]` 加/减 π |
| 麦克纳姆轮横移时车身旋转 | 辊子方向装反了 | 检查 LF/RF 辊子方向是否相反 |
| 舵轮行驶时轮子抖动 | 最短路径优化阈值太大 | 减小阈值或增加死区 |
| 里程计 yaw 漂移 | 轮子打滑或轮径参数不准 | 减小加速度、IMU 融合修正 |
| 转向电机不停转 | 目标角度一直在变（未收敛） | 检查角度环 PID 参数、机械是否卡死 |


---

> 📘 **轮腿相关**：本文档覆盖传统轮式底盘运动学。如需了解轮腿式底盘的动力学与控制，请参阅 [[RM_Chassis_串联轮腿步兵下位机控制系统设计]] | [[RM_Chassis_串联轮腿数学推导与仿真]] | [[RM_Chassis_偏置并联轮腿学习指南]]
>
> 📘 **下游消费**：本文解算出的底盘速度/角速度/加速度，是云台加速度补偿递推公式的输入 → [[RM_Gimbal_重力补偿与加速度补偿详解]]
