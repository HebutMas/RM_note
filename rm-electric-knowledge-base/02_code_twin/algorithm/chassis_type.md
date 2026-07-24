# chassis_type — 底盘运动学算法层

`modules/algorithm/chassis_type.c` / `modules/algorithm/chassis_type.h`

---

## 一、坐标系与正方向

```
         ▲  X+  (前进 Forward)
         │
         │
 Y+ ◄────┘        Y+ = 左方向 (Left)
                  ω+ = 逆时针 (CCW, 从顶部俯视)
```

| 符号 | 含义 | 正方向 |
|------|------|--------|
| `vx` | 底盘 X 轴线速度 (m/s) | 前进 |
| `vy` | 底盘 Y 轴线速度 (m/s) | 左移 |
| `vw` / `ω` | 底盘旋转角速度 (rad/s) | 逆时针 |

### 轮组编号（俯视图）

```
       Front (前)
   ┌───────────┐
   │ 0       3 │   0: 左前 LF
   │           │   1: 左后 LB
   │ 1       2 │   2: 右后 RB
   └───────────┘   3: 右前 RF
       Back  (后)
```

---

## 二、麦轮（Mecanum）

### 辊子方向

```
         Front
     ┌───────────┐
     │ ╲       ╱ │    LF(0): ╲ 辊子 +45°
     │   ╲   ╱   │    RF(3): ╱ 辊子 -45°
     │   ╱   ╲   │    LB(1): ╱ 辊子 -45°
     │ ╱       ╲ │    RB(2): ╲ 辊子 +45°
     └───────────┘
         Back
```

### 工作原理

辊子与地面接触时，轮子旋转产生的摩擦力方向与辊子轴向垂直。由于辊子倾斜 45°，轮子正向旋转时产生斜向 45° 的推力。

| 轮组 | 辊子角 | 正转产生推力方向 |
|------|--------|-----------------|
| LF (0) | +45° | 前-左 (↖) |
| RF (3) | -45° | 前-右 (↗) |
| LB (1) | -45° | 后-右 (↘) |
| RB (2) | +45° | 后-左 (↙) |

### 逆运动学

```c
void Chassis_Mecanum_Calc(DJI_Motor_t *motors[4], const Chassis_Diff_Config_s *cfg,
                           float vx, float vy, float vw)
{
    const float L = (cfg->wheel_base_x + cfg->wheel_base_y) / 2.0f;
    const float f = cfg->decele_ratio / cfg->wheel_radius;
    float ws[4];
    // LF:+vx -vy -ω·L   RF:+vx +vy +ω·L
    // LB:+vx +vy -ω·L   RB:+vx -vy +ω·L
    ws[0] = (+vx - vy - vw * L) * f;
    ws[3] = (+vx + vy + vw * L) * f;
    ws[1] = (+vx + vy - vw * L) * f;
    ws[2] = (+vx - vy + vw * L) * f;
    for (int i = 0; i < 4; i++) Motor_DJI_SetRef(motors[i], ws[i]);
}
```

公式：`ω_i = (±vx ± vy ± ω·L) / R`，输出乘 `decele_ratio` 得电机轴角速度。

### 正运动学

```c
Chassis_Velocity_s Chassis_Mecanum_Fwd(const DJI_Motor_t *motors[4],
                                        const Chassis_Diff_Config_s *cfg)
{
    const float L = (cfg->wheel_base_x + cfg->wheel_base_y) / 2.0f;
    const float f = cfg->wheel_radius / cfg->decele_ratio;
    float w[4];
    for (int i = 0; i < 4; i++) w[i] = motors[i]->base.measure.speed_rad * f;
    Chassis_Velocity_s vel;
    vel.vx = (+w[0] + w[3] + w[1] + w[2]) / 4.0f;
    vel.vy = (-w[0] + w[3] + w[1] - w[2]) / 4.0f;
    vel.vw = (-w[0] + w[3] - w[1] + w[2]) / (4.0f * L);
    return vel;
}
```

### 里程计

```c
void Chassis_Mecanum_Odom(Chassis_Odom_s *odom, const DJI_Motor_t *motors[4],
                           const Chassis_Diff_Config_s *cfg, float dt)
{
    Chassis_Velocity_s vel = Chassis_Mecanum_Fwd(motors, cfg);
    float c = arm_cos_f32(odom->yaw), s = arm_sin_f32(odom->yaw);
    odom->x   += (vel.vx * c - vel.vy * s) * dt;
    odom->y   += (vel.vx * s + vel.vy * c) * dt;
    odom->yaw += vel.vw * dt;
}
```

### 配置结构体

```c
typedef struct {
    float wheel_base_x;   // 前后轮距 (m)
    float wheel_base_y;   // 左右轮距 (m)
    float wheel_radius;   // 轮子半径 (m), 通常 0.076 (76mm)
    float decele_ratio;   // 减速比 (3508: 19:1)
} Chassis_Diff_Config_s;
```

---

## 三、全向轮（Omni）

### 轮平面布局

```
         Front
     ┌───────────┐
     │ ↗       ↖ │    LF(0): ↗  轮平面 +45°  (施力方向: 前-左)
     │   ╲   ╱   │    RF(3): ↖  轮平面 -45°  (施力方向: 前-右)
     │   ╱   ╲   │    LB(1): ↖  轮平面 +135° (施力方向: 后-左)
     │ ↖       ↗ │    RB(2): ↗  轮平面 -135° (施力方向: 后-右)
     └───────────┘
         Back
```

全向轮只能沿其轮平面方向施力，垂直于轮平面的方向由辊子自由滚动。

### 逆运动学推导

对于位置 `(px, py)` 处、轮平面方向角为 `θ` 的全向轮：

```
ω_i × R = (vx - ω·py) × cos(θ) + (vy + ω·px) × sin(θ)
```

代入四个轮的位置和角度：

```
LF: px=+a, py=+b, θ= +45°  →  ω_LF = (+vx + vy + ω·(a-b)) / (R·√2)
RF: px=+a, py=-b, θ= -45°  →  ω_RF = (+vx - vy - ω·(a-b)) / (R·√2)
LB: px=-a, py=+b, θ=+135°  →  ω_LB = (-vx + vy - ω·(a-b)) / (R·√2)
RB: px=-a, py=-b, θ=-135°  →  ω_RB = (-vx - vy + ω·(a-b)) / (R·√2)
```

其中 `a = wheel_base_x / 2`，`b = wheel_base_y / 2`。

### 正方形底盘特性 (`a == b`)

旋转差分项 `ω·(a-b) = 0`，纯旋转仅由 RF 和 LB 承担，LF 和 RB 自由滚动。

### 代码实现

```c
void Chassis_Omni_Calc(DJI_Motor_t *motors[4], const Chassis_Diff_Config_s *cfg,
                        float vx, float vy, float vw)
{
    const float a  = cfg->wheel_base_x / 2.0f;
    const float b  = cfg->wheel_base_y / 2.0f;
    const float rd = vw * (a - b);
    const float f  = cfg->decele_ratio / (cfg->wheel_radius * SQRT2_2);
    float ws[4];
    ws[0] = (+vx + vy + rd) * f;  // LF
    ws[3] = (+vx - vy - rd) * f;  // RF
    ws[1] = (-vx + vy - rd) * f;  // LB
    ws[2] = (-vx - vy + rd) * f;  // RB
    for (int i = 0; i < 4; i++) Motor_DJI_SetRef(motors[i], ws[i]);
}
```

正运动学和里程计的模式与麦轮类似，不再展开。

### 配置

与麦轮共用 `Chassis_Diff_Config_s`。

---

## 四、舵轮（Swerve）

### 结构

8 电机：`motors[0..3]` 驱动 (M3508) + `motors[4..7]` 转向 (GM6020)，每个轮组独立控制驱动速度和转向角度。

### 运动学推导

核心思路：将底盘中心速度分解到每个轮组位置，计算该处的线速度方向（转向目标）和大小（驱动速度）。

以 LF 为例，其位置在 `(+wheel_r·√2/2, +wheel_r·√2/2)`：

```
local_vx_LF = vx + vw × (wheel_r × 0.707)
local_vy_LF = vy + vw × (wheel_r × 0.707)
```

`vw × wheel_r × 0.707` 是底盘旋转在 LF 位置产生的切向线速度分量。

**局部速度矩阵：**

| 轮组 | `local_vx` | `local_vy` |
|------|-----------|-----------|
| LF (0) | `vx + vw·wheel_r·0.707` | `vy + vw·wheel_r·0.707` |
| LB (1) | `vx + vw·wheel_r·0.707` | `vy - vw·wheel_r·0.707` |
| RB (2) | `vx - vw·wheel_r·0.707` | `vy - vw·wheel_r·0.707` |
| RF (3) | `vx - vw·wheel_r·0.707` | `vy + vw·wheel_r·0.707` |

**局部速度 → 驱动速度：**

```
target_speed = sqrt(local_vx² + local_vy²) / radius_wheel_m × decele_ratio
```

**局部速度 → 转向角度：**

```
vector_rad     = atan2(local_vy, local_vx)
target_abs_rad = align_rad[i] + vector_rad
```

`align_rad[i]` 是机械零点对齐角——当 `vector_rad = 0`（轮组正前方）时，转向电机的编码器读数。

### drct_factor

目标角与当前角差超过 ±90° 时，转向角翻转 180°，驱动速度取反，避免轮组转 180°。

正运动学和里程计的模式与麦轮类似，不再展开。

### 配置结构体

```c
typedef struct {
    float wheel_r;          // 投影点到中心距离 (m)
    float radius_wheel_m;   // 轮子半径 (m)
    float decele_ratio;     // 减速比 (3508: 19:1)
    float align_rad[4];     // 机械零点角度 [LF, LB, RB, RF]
} Chassis_Swerve_Config_s;
```

---

## 五、底盘类型对比

| 特性 | 麦轮 (Mecanum) | 全向轮 (Omni) | 舵轮 (Swerve) |
|------|---------------|--------------|--------------|
| 电机数 | 4 | 4 | 8 |
| 辊子方向 | 与轮平面成 45° | 与轮平面平行 (0°) | — |
| 施力方向 | 与辊子垂直，即与轮平面成 45° | 沿轮平面方向 | 任意方向（转向电机控制） |
| 轮平面朝向 | 全部朝前 (0°) | 指向外侧 45° | 独立可控 |
| 优点 | 结构简单，全向性好 | 结构简单，全向性好 | 抓地力强，精确控制 |
| 缺点 | 辊子易磨损，效率低 | 辊子易磨损，效率低 | 结构复杂，成本高 |

---

## 六、调用关系

```
chassis_func（应用层）
  └── Chassis_Mecanum_Calc / Chassis_Omni_Calc / Chassis_Swerve_Calc
        └── Motor_DJI_SetRef → 电机驱动层
```