# power_control.c/h — 功率控制

`modules/MOTOR/POWER_CONTROL/power_control.{c,h}`

## 一句话

开环估算各电机功率，超限时按混合权重（功率比例 + 速度误差比例）重新分配功率预算，反解扭矩写回 `controller.output`。只管大疆电机，达妙电机不参与。

## 在 2ms 循环中的位置

```c
// module_motor.c — motor_task_entry
while (1) {
    Motor_UpdateAll();        // ① 遍历链表，逐个调 ControlAndSend
    PowerControl_Update();    // ② 功率限制（修改 output）
    Motor_DJI_Flush();        // ③ 大疆电机批量发送
    tx_thread_sleep(2);
}
```

三步的时序关系是理解功率控制的关键：

| 步骤 | 做了什么 | output 的状态 |
|------|---------|--------------|
| `Motor_UpdateAll()` | 遍历所有电机，调 `ControlAndSend` | PID/LQR 算完，output 已写入 |
| `PowerControl_Update()` | 估算功率，超限时修改 output | output 可能被改小 |
| `Motor_DJI_Flush()` | 大疆电机批量发 CAN | output 发出去 |

### 达妙电机不受功率控制

`Motor_UpdateAll()` 遍历链表调 `ControlAndSend`，达妙电机的 `dm_ControlAndSend` 内部直接 `BSP_CAN_SendMessage` 发出去了——**一帧一帧立即发**，不经过 Flush。所以 `PowerControl_Update()` 执行时，达妙电机的 CAN 帧已经发完了，功率控制改不了达妙电机的 output。

大疆电机不同：`dji_control` 只把 output 写入 `sender_assignment` 静态缓冲区，不实际发 CAN。`Motor_DJI_Flush()` 才真正批量发。所以 `PowerControl_Update()` 有机会在 Flush 之前修改大疆电机的 output。

> 这就是为什么 `PowerControl_Update()` 夹在 `Motor_UpdateAll()` 和 `Motor_DJI_Flush()` 之间——它只能拦截还没发出去的大疆电机。

### 功率控制只处理大疆电机

`dji_output_to_torque()` 和 `dji_set_torque()` 的 switch 只覆盖 M3508 / M2006 / GM6020_CURRENT，default 返回 0。达妙电机即使注册了功率控制，估算功率也是 0，不会被限制。

## 功率模型：开环估算

### 公式

```
P = τ·Ω + k1·|Ω| + k2·τ² + k3
```

| 项 | 物理含义 | 说明 |
|----|---------|------|
| `τ·Ω` | 机械输出功率 | 扭矩 × 角速度，纯物理项 |
| `k1·\|Ω\|` | 旋转损耗 | 轴承摩擦、风阻等与转速相关的损耗 |
| `k2·τ²` | 铜损（ Joule 热） | 电流² × 电阻，与扭矩平方成正比（因为 I ∝ τ） |
| `k3` | 固定损耗 | 驱动器静态功耗等常数项 |

### 怎么拿到 τ 和 Ω

先说清楚一个前提：**大疆电机 CAN 反馈的转速和电流都是减速前（电机轴）的**。

CAN 接收回调里直接把反馈 RPM 换算成 rad/s 存入 `speed_rad`，不除减速比：

```c
// motor_dji.c — dji_can_rx_callback
motor->measure.speed_rpm = (int16_t)(data[2] << 8 | data[3]);   // 电机轴 RPM
motor->base.measure.speed_rad = motor->measure.speed_rpm * RPM_2_RAD_PER_SEC;  // 还是电机轴 rad/s
```

PID/LQR 也用这个电机轴转速做反馈，全程不除减速比。

功率控制里需要换算到**输出轴**，因为 k1/k2/k3 损耗系数是按输出轴标定的：

```c
omega[i] = m->measure.speed_rad / m->info.gear_ratio;   // 电机轴 → 输出轴角速度
tau[i]   = dji_output_to_torque(m);                      // output(整数) → 电流(A) → ×Kt×gear_ratio → 输出轴扭矩(Nm)
```

#### Ω：电机轴 → 输出轴

`measure.speed_rad` 是电机轴角速度（CAN 反馈直接换算的），除以 `gear_ratio` 得到输出轴角速度。M3508 减速比 16:1，电机轴转 16 圈输出轴转 1 圈，转速除 16。GM6020 减速比 1:1，不除。

#### τ：output 整数 → 输出轴扭矩

`dji_output_to_torque()` 把 PID 算出来的 `controller.output`（整数电流值）反算回输出轴扭矩：

```c
// power_control.c — dji_output_to_torque
// 1. 整数 → 实际电流(A)，用 currentToInteger 的逆运算 IntegerToCurrent
// 2. 电流 × Kt × gear_ratio = 输出轴扭矩
float Kt_gear = m->info.torque_constant * m->info.gear_ratio;
return IntegerToCurrent(..., (int16_t)m->controller.output) * Kt_gear;
```

`torque_constant` 注释写的是"减速前扭矩常数 (Nm/A)"——电机本身的绕组常数，不含减速比。所以从电流算输出轴扭矩要乘 `Kt × gear_ratio`：先乘 Kt 得电机轴扭矩，再乘减速比得输出轴扭矩。

反过来，`dji_control` 里从扭矩算电流时是除：

```c
// motor_dji.c — dji_control
torque / (torque_constant * gear_ratio)   // 输出轴扭矩 → 电机轴扭矩 → 电流
```

#### 为什么用命令值不是反馈值

注意 τ 用的是 **PID 的输出（命令扭矩）**，不是电机实际反馈的扭矩。CAN 反馈的 `real_current` 存了但没用来算 `torque_nm`——`torque_nm` 直接等于 `output_torque`（PID 命令值）。所以功率估算是**开环**的：根据"我命令电机输出多少扭矩"+"电机当前转速"，估算"这个命令会导致多大功率消耗"。

### 损耗系数

哨兵底盘板的实际参数：

| 电机 | 角色 | k1 | k2 | k3 |
|------|------|-----|-----|-----|
| M3508 ×4 | 驱动轮 (DRIVE) | 0.132 | 3.47 | 1 |
| GM6020 ×4 | 舵向轮 (STEER) | 0.005 | 12.98 | 1 |

这些系数通过 `PowerControl_Register()` 注册，来源是实测标定：给电机施加已知扭矩和转速，测量实际功率，拟合出 k1/k2/k3。

## 功率分配流程

### 1. 计算可用功率

```c
float effective = s_power_limit;   // 裁判系统功率上限，如 120W
if (s_use_buffer) {
    PIDCalculate(&s_buffer_pid, s_buffer_energy, 30.0f);  // 缓冲能量目标 30J
    effective -= s_buffer_pid.Output;   // 缓冲能量低 → 扣减可用功率
}
```

如果启用缓冲能量管理（哨兵当前 `use_buffer=0` 未启用），用 PID 以缓冲能量为反馈，缓冲能量低于 30J 时减小可用功率，防止超功率扣血。

### 2. 按角色分组分配

```c
if (n_steer == 0) {
    limit_group(drives, n_drive, effective, NULL);      // 全给驱动轮
} else {
    limit_group(steers, n_steer, effective * 0.8, &steer_sum);  // 舵向轮先拿 80%
    drive_limit = effective - steer_actual;             // 剩下给驱动轮
    limit_group(drives, n_drive, drive_limit, NULL);
}
```

| 角色 | 占比 | 电机 | 说明 |
|------|------|------|------|
| STEER（舵向轮） | 80% | GM6020 ×4 | 控制轮子转向，功率需求小且稳定 |
| DRIVE（驱动轮） | 剩余 | M3508 ×4 | 控制轮子转速，功率需求大且波动大 |

舵向轮先分配，实际用了多少就从总预算里扣，剩下的全给驱动轮。这样保证转向不被驱动轮抢功率。

### 3. 组内分配：limit_group

`limit_group()` 对一组电机执行功率限制，核心逻辑：

**第一步：估算每个电机的命令功率**

```c
for (uint8_t i = 0; i < N; i++) {
    cmdPow[i] = tau[i] * omega[i] + k1 * |omega[i]| + k2 * tau[i]² + k3;
    sum_cmd += cmdPow[i];
}
```

**第二步：不超限就直接返回**

```c
if (sum_cmd <= max_power) return;   // 总功率没超，保持原始输出
```

**第三步：超限 → 混合权重分配**

超限时需要把每个电机的功率削减到预算内。权重由两部分混合：

| 权重 | 含义 | 计算方式 |
|------|------|---------|
| `w_prop` | 按功率比例 | 该电机功率 / 正功率电机总功率 |
| `w_err` | 按误差比例 | 该电机速度误差 / 总速度误差 |

混合系数 `confidence` 由总速度误差决定：

```
sum_err ≥ 20 rad/s → confidence = 1（完全按误差分配）
sum_err ≤ 15 rad/s → confidence = 0（完全按功率分配）
中间 → 线性插值
```

**为什么用混合权重？**

- **按功率比例**（confidence=0）：功率大的电机削得多，功率小的削得少。简单但不考虑控制需求。
- **按误差比例**（confidence=1）：速度误差大的电机（还没到目标转速）少削减，误差小的（已接近目标）多削减。优先保正在加速的电机。
- **混合**：误差大时偏向按误差分配（保护正在加速的电机），误差小时偏向按功率分配（均匀削减）。

**第四步：悬空轮保护**

```c
if (errRad[i] < IDLE_WHEEL_ERR_RAD && p_alloc > cmdPow[i])
    p_alloc = cmdPow[i];
```

如果某个轮子悬空（速度误差很小 = 轮子已到达目标转速，没接地阻力），不让它拿比原始命令更多的功率，避免悬空轮抢占接地轮的功率预算。

**第五步：反解扭矩**

分配到功率预算 `p_alloc` 后，需要反推出新的扭矩值。把 `p_alloc` 代入功率公式：

```
P = τ·Ω + k1·|Ω| + k2·τ² + k3
→ k2·τ² + Ω·τ + (k1·|Ω| + k3 - p_alloc) = 0
```

这是一个关于 τ 的一元二次方程，用求根公式解：

```c
float delta = b² - 4ac;
float root1 = (-b + √delta) / (2a);
float root2 = (-b - √delta) / (2a);
tau_new = (output >= 0) ? root1 : root2;   // 选与原扭矩同号的根
```

选根的依据：原扭矩正选正根，原扭矩负选负根，保持方向不变。

**第六步：写回 output**

```c
tau_new = CLAMP(tau_new, -max_torque, max_torque);
dji_set_torque(m, tau_new);   // 扭矩 → 电流 → 整数 → 写入 controller.output
```

`dji_set_torque()` 把新扭矩反算成整数电流值写回 `controller.output`，覆盖原来 PID/LQR 的输出。之后 `Motor_DJI_Flush()` 把这个被削减过的 output 发出去。

## 完整数据流

```
Motor_UpdateAll()
  └─ dji_control(motor)
       ├─ PID/LQR 计算 → torque (Nm)
       ├─ torque → currentToInteger → controller.output (整数电流值)
       └─ 写入 sender_assignment 缓冲区（不发送）
            │
            ▼
PowerControl_Update()
  ├─ dji_output_to_torque(m)        output → τ (Nm)，反算回来
  ├─ P = τ·Ω + k1·|Ω| + k2·τ² + k3  估算功率
  ├─ 超限？
  │   ├─ 否 → 不动
  │   └─ 是 → 分配 p_alloc → 解二次方程 → tau_new
  │           └─ dji_set_torque(m, tau_new)  覆盖 controller.output
  │                 │
  │                 ▼  ⚠ 注意：只改了 output，没改 sender_assignment！
  │
  ▼
Motor_DJI_Flush()
  └─ BSP_CAN_SendMessage(&sender_assignment[i])  发送的是缓冲区里的旧值！
```

> **潜在问题**：`PowerControl_Update()` 通过 `dji_set_torque()` 修改了 `controller.output`，但没有重新写入 `sender_assignment` 缓冲区。`Motor_DJI_Flush()` 发送的是 `dji_control()` 在第一步写入的旧值。功率限制实际上没有生效在 CAN 发送的数据上——它改了 `output` 字段，但 Flush 发的是缓冲区。
>
> 这可能是代码的一个 bug，也可能是设计意图（`output` 供其他逻辑读取，实际发送用缓冲区）。需要结合实际测试确认。

## 对外接口

| 函数 | 调用者 | 说明 |
|------|--------|------|
| `PowerControl_Register(motor, role, param)` | `chassis_init` | 注册电机参与功率控制 |
| `PowerControl_SetLimit(power, buffer, use_buffer)` | `chassis_init` | 设置功率上限和缓冲能量 |
| `PowerControl_Update()` | `motor_task_entry` | 2ms 循环中执行功率分配 |

## 前置条件

| 依赖 | 说明 |
|------|------|
| [[02_code_twin/modules/MOTOR/motor_base\|Motor_Base]] | 电机链表和 `controller.output` |
| [[02_code_twin/modules/MOTOR/DJI/motor_dji\|Motor_DJI]] | `dji_output_to_torque` / `dji_set_torque` 只支持大疆电机 |
| 裁判系统 | `PowerControl_SetLimit` 的功率上限来自裁判系统 |
