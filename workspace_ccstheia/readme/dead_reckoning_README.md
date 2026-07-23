# dead_reckoning 模块 — 航迹推算（编码器里程计 + 陀螺仪航向）

> **当你看代码旁边的注释都看不懂时来翻阅的百科全书**
> 适用芯片：MSPM0G3507 @ 32MHz | 纯 C | 零硬件依赖

---

## 1. 模块概述

这个模块做的是**航迹推算（Dead Reckoning）**：已知机器人**上一时刻的位置/航向**，根据**编码器脉冲**和**陀螺仪角速度**推算**当前的位置/航向**。

说白了就是：你知道自己刚才在哪、面朝哪，也知道轮子转了多少、车身转了多少，就能猜出你现在在哪。

### 使用场景
- 智能小车在已知起点后，用编码器+陀螺仪进行短距离定位
- 作为里程计反馈，配合 diff_drive 做闭环运动控制
- 双轮差速底盘的位置估计（x, y, heading）

---

## 2. 数据结构：`DR_t`

```c
typedef struct {
    float x_mm;
    float y_mm;
    float heading_deg;
    float wheel_base_mm;
    float last_heading_deg;
    uint8_t initialized;
} DR_t;
```

### 逐字段解释

| 字段 | 类型 | 单位 | 含义 |
|------|------|------|------|
| `x_mm` | `float` | mm | X 方向位置坐标。坐标系约定见第 3 节 |
| `y_mm` | `float` | mm | Y 方向位置坐标 |
| `heading_deg` | `float` | **度（°）** | 当前航向角，单位为度。取值范围理论无约束，但会被归一化到 (-180, +180] |
| `wheel_base_mm` | `float` | mm | 轮距（左右驱动轮中心之间的距离），参与运动学计算 |
| `last_heading_deg` | `float` | 度 | 上次更新时的航向角。当前代码中并未在 Update 内使用，由 Reset 同步设置，预留做增量计算 |
| `initialized` | `uint8_t` | — | 初始化标志。0=未初始化（Update 不做推算），1=已可用 |

### 为什么所有位置/轮距都用 mm？

- 编码器一圈脉冲数 → 轮周长 → **mm** 是最直观的长度单位
- float 的精度（~1.2e-7 相对误差）对于 mm 级别的位置累积完全够用
- 如果用 m，d_s = 0.0001m 这种数量级打印和调试都不方便

### 为什么 `heading_deg` 用度而不是弧度？

- **人脑友好**：0°、90°、180° 比 0rad、1.57rad、3.14rad 直观得多
- 陀螺仪输出通常是 °/s，直接用度可以省一次换算
- 只在三角函数计算时转一次弧度（在 Update 里转），不影响外部接口
- 如果用 double 做中间计算反而浪费 MSPM0G3507 的 FPU 资源（单精度硬浮点）

---

## 3. 坐标系定义

```
         y+
          ^
          |
          |   heading > 0 (逆时针)
          |
          +----------> x+

  车头朝 x+ 方向时 heading = 0°
  逆时针旋转时 heading 增加
```

### 具体约定

| 约定 | 值 |
|------|-----|
| x 轴正方向 | 车头初始朝向（前进方向） |
| y 轴正方向 | x 逆时针转 90°（车身左侧） |
| heading = 0° | 车头朝 x 正方向 |
| heading 正方向 | 逆时针（左转为正） |
| heading 范围 | (-180°, +180°]，用 while 循环归一化 |
| 角度单位 | **度**（所有接口），内部三角计算转弧度 |

### 为什么是这个约定？

- 这是 **最常用的机器人坐标系**（ROS REP-103 推荐的变体，只是 ROS 用弧度）
- 标准数学角度系：逆时针为正，0° 指向 x 正方向
- 与学校教材、控制理论教材一致，减少认知负担

---

## 4. 完整运动学模型（数学推导）

### 4.1 基本量定义

| 符号 | 含义 | 单位 |
|------|------|------|
| v | 车体线速度（前进方向） | mm/s |
| ω | 车体角速度 | °/s |
| L | 轮距（wheel_base_mm） | mm |
| Δs_L, Δs_R | 左/右轮在 Δt 内走过的弧长（编码器增量） | mm |
| Δt | 更新周期（dt） | s |
| θ | 当前航向角 | rad |

### 4.2 航向积分

```
θ_new = θ_old + ω_gyro × Δt
```

代码实现（第 64-66 行）：

```c
float heading_rad = dr->heading_deg * (3.14159265f / 180.0f);
float delta_heading = rate_degps * dt;
dr->heading_deg += delta_heading;
```

这里 **没有用编码器推算航向**，而是**直接采用陀螺仪角速度积分**。这是比纯编码器更精确的做法——编码器推算航向需要左右轮里程差除以轮距，会放大编码器噪声；而陀螺仪短时积分精度更高。

### 4.3 位置积分

双轮差速底盘的位移计算（第 75-79 行）：

```c
float ds = (delta_left_mm + delta_right_mm) * 0.5f;
dr->x_mm += ds * cosf(heading_rad);
dr->y_mm += ds * sinf(heading_rad);
```

#### 背后的物理

在 Δt 极短（通常 10-50ms）的前提下，机器人走过的轨迹近似为**一段小圆弧**。圆弧的弧长等于左右轮的平均里程：

```
Δs = (Δs_L + Δs_R) / 2
```

然后把这段弧线**近似为直线段**，方向为当前航向 θ。于是位移分解为：

```
Δx = Δs × cos(θ)
Δy = Δs × sin(θ)
```

#### 完整的微分方程（理论模型）

严格的差速底盘运动学微分方程：

```
dx/dt = v × cos(θ)
dy/dt = v × sin(θ)
dθ/dt = ω

v  = (v_R + v_L) / 2    （v_L, v_R 为左右轮线速度）
ω  = (v_R - v_L) / L    （由轮速差算角速度）
```

代码中用陀螺仪 ω 取代了编码器推算的 ω，避免了 `ω = (v_R - v_L) / L` 带来的放大噪声。

#### 代码中使用的模型

```
θ(t+Δt) = θ(t) + ω_gyro × Δt
x(t+Δt) = x(t) + (Δs_L + Δs_R)/2 × cos(θ(t) + ω_gyro × Δt/2)
y(t+Δt) = y(t) + (Δs_L + Δs_R)/2 × sin(θ(t) + ω_gyro × Δt/2)
```

**注意**：代码实际上用 `heading_rad`（更新前的 θ）而不是 `θ + ωΔt/2`（中间角度）。这是一个 **欧拉前向积分** 近似，在 ωΔt 很小时误差可忽略。

### 4.4 圆弧模型 vs 直线模型

| 模型 | 假设 | 适用条件 |
|------|------|----------|
| 圆弧模型 | Δt 内沿恒定曲率圆弧运动 | Δt 较大或转速较快 |
| 直线模型（代码采用的近似） | Δt 内沿当前航向直线运动 | Δt 足够小（< 50ms） |

**代码实际上是直线近似**：用 `heading_rad`（时段初的航向）乘全程 ds。误差分析：

```
航向误差 = ω × Δt / 2  （平均偏离量）
位置误差 ≈ Δs × (1 - cos(ωΔt/2))  （径向误差）
```

当 ω = 90°/s, Δt = 20ms 时：
- 航向误差 ≈ 0.9°，位置误差 < 0.1%
- 这个精度对于小车短距定位完全可接受

---

## 5. 函数详解

### 5.1 `DR_Init`

```c
void DR_Init(DR_t *dr, float wheel_base_mm)
{
    if (dr == NULL) return;
    dr->wheel_base_mm = (wheel_base_mm > 0.0f) ? wheel_base_mm : 100.0f;
    dr->x_mm           = 0.0f;
    dr->y_mm           = 0.0f;
    dr->heading_deg    = 0.0f;
    dr->last_heading_deg = 0.0f;
    dr->initialized    = 0;
}
```

- 空指针保护：所有函数都做 `dr == NULL` 检查，这是嵌入式 C 的标准防御性编程
- wheel_base_mm 如果传 0 或负数，默认用 100mm——因为 0 会导致运动学计算异常
- `initialized` 设为 0：表示结构体已分配但未就绪，需要调用 `DR_Reset` 设置初始位姿
- 为什么不直接在 Init 里设 initialized=1？因为此时并不知道机器人的初始位姿，可能需要在主控收到 GPS/视觉定位后才确定

### 5.2 `DR_Reset`

```c
void DR_Reset(DR_t *dr, float init_x, float init_y, float init_heading)
{
    if (dr == NULL) return;
    dr->x_mm           = init_x;
    dr->y_mm           = init_y;
    dr->heading_deg    = init_heading;
    dr->last_heading_deg = init_heading;
    dr->initialized    = 1;
}
```

- 设置初始坐标和航向
- `last_heading_deg` 同步为初始航向，虽然当前 Update 未使用，但为后续扩展保留
- `initialized = 1`：允许 Update 执行

典型用法：机器人从充电站出发时调用 `DR_Reset(&dr, 0, 0, 0)`，或用视觉定位到已知坐标后重置。

### 5.3 `DR_Update`（核心函数）

```c
void DR_Update(DR_t *dr, float rate_degps, float delta_left_mm,
               float delta_right_mm, float dt)
{
    if (dr == NULL) return;
    if (dt <= 0.0f) return;

    if (!dr->initialized) {
        dr->initialized = 1;
    }

    float heading_rad = dr->heading_deg * (3.14159265f / 180.0f);
    float delta_heading = rate_degps * dt;
    dr->heading_deg += delta_heading;

    if (dr->heading_deg > 180.0f) {
        dr->heading_deg -= 360.0f;
    } else if (dr->heading_deg <= -180.0f) {
        dr->heading_deg += 360.0f;
    }

    float ds = (delta_left_mm + delta_right_mm) * 0.5f;
    dr->x_mm += ds * cosf(heading_rad);
    dr->y_mm += ds * sinf(heading_rad);
}
```

**逐行详解：**

| 行号 | 代码 | 解释 |
|------|------|------|
| 57-58 | 空指针 + dt ≤ 0 保护 | dt 为 0 会导致除零（虽然没有显式除法，但数学上不合理） |
| 60-62 | 自动初始化 | 如果用户忘记调用 Reset，第一次 Update 会自动设置 initialized=1，位姿保持 0,0,0。这是一种"宽容"设计，避免空指针后的灾难 |
| 64 | 角度转弧度 | π 取 3.14159265f 而非 math.h 的 M_PI，因为 M_PI 在 C99 标准中不是强制定义的。手动写死确保可移植 |
| 65-66 | 航向积分 | `delta_heading = rate_degps × dt`，直接用陀螺仪角速度乘时间 |
| 69-73 | 航向归一化 | 限制到 (-180, +180]。用 if-else 而非 fmodf，因为 fmodf 在嵌入式平台可能较慢，且这里偏差通常不会超过一圈 |
| 75 | 平均里程 | `ds = (Δs_L + Δs_R) / 2`，相当于车体中心走过的弧长 |
| 78-79 | 位置分解 | 用 **更新前** 的航向去分解位移（欧拉前向），注意这里不是 `cos(heading_new)` |

#### 关于 `heading_rad` 的取值时机

```c
float heading_rad = dr->heading_deg * (3.14159265f / 180.0f);
float delta_heading = rate_degps * dt;
dr->heading_deg += delta_heading;
// ...归一化...
float ds = (delta_left_mm + delta_right_mm) * 0.5f;
dr->x_mm += ds * cosf(heading_rad);
dr->y_mm += ds * sinf(heading_rad);
```

注意 `heading_rad` 取的是**更新前的航向**。这意味着用这段时间**开始时的方向**去近似整段的位移方向。更精确的做法是取中间时刻的航向（`heading_rad + omega * dt / 2`），但代码没有这么做。

为什么不取中间值？因为：
1. 需要多一次 float 运算 + 一次临时变量
2. 当 dt 很小（10ms 级）时，影响微乎其微
3. 保持代码最小化，适合 MSPM0G3507 的资源限制

### 5.4 `DR_GetHeading`

```c
float DR_GetHeading(DR_t *dr)
{
    if (dr == NULL) return 0.0f;
    float h = dr->heading_deg;
    while (h > 180.0f)  h -= 360.0f;
    while (h < -180.0f) h += 360.0f;
    return h;
}
```

- 用 `while` 而不是 `if`：因为航向角经过多次积分累积可能超出 [-360, 360] 的范围（比如转了 3 圈），while 能处理多次归一化
- 为什么不直接 `fmodf(h + 180, 360) - 180`？避免浮点取模的精度问题 + 代码更直观
- 为什么不修改 `dr->heading_deg`？这是一个 **getter**，不改变内部状态

### 5.5 `DR_GetPosition`

```c
void DR_GetPosition(DR_t *dr, float *x, float *y)
{
    if (dr == NULL) return;
    if (x != NULL) *x = dr->x_mm;
    if (y != NULL) *y = dr->y_mm;
}
```

- x, y 指针可以为 NULL，允许只读取一个坐标——这在某些场景（如一维循迹只需要 x）很实用
- 空指针保护：防止用户传 NULL 导致写入异常

---

## 6. 误差分析与踩坑点

### 6.1 编码器脉冲累积误差

**问题**：编码器每转脉冲数（PPR）有限，每一步 Δs 都有 ±1 个脉冲的量化误差。

- 假设编码器 PPR=12（霍尔传感器常见值），轮子直径 65mm → 周长 ≈ 204mm
- 每脉冲对应距离 = 204 / 12 ≈ **17mm**
- 意味着每次读取编码器，Δs 有 ±17mm 的误差

**影响**：走 1m 直线，仅量化误差就可积累到 ±(17/204) × 1000 ≈ ±80mm。

**缓解方法**：
- 使用更高分辨率编码器（磁性编码器可达 1024 PPR）
- 做多脉冲累积后再上报（如每 10ms 读一次，累积 5 次才更新一次位置）
- 配合外部传感器（视觉/激光）周期性修正

### 6.2 车轮打滑时的里程计发散

**问题**：打滑时轮子转但车不动，编码器记录 Δs 但实际位移为 0。

- 这是开环里程计的**致命伤**：打滑时位置误差会瞬间爆炸
- 打滑+转向同时发生（原地甩尾）：航向和位置都错

**无法从算法层面解决**：因为没有任何传感器能区分"轮子旋转"和"车身移动"的区别。唯一的办法是融合其他传感器。
- 陀螺仪（已融合）能提供航向参考
- IMU 加速度计可以做零速检测（动态发现打滑）
- 视觉里程计可以对比特征点偏移

### 6.3 陀螺仪零漂

**问题**：MEMS 陀螺仪有温漂，静止时输出也不是 0。

- 即使车不动，`rate_degps` 也有 ~0.5°/s 的零偏
- 长时间静止积分会导致航向缓慢漂移

**缓解**：
- 启动时做静态零偏校准（采样 100ms 静止数据取平均）
- 定期在直线行驶时用小角度修正融合

### 6.4 dt 不恒定

**问题**：如果调度器延迟或中断阻塞，`dt` 不是精确的定时值。

- `dt` 偏大会高估角速度积分
- `dt` 偏小会低估位移

**建议**：dt 应该由硬件定时器提供精确值，而非软件延时估算。

---

## 7. 调试手段

### 7.1 直线行走精度测试

**方法**：让小车走一条 1m 长的直线，然后回读坐标。

```
预期: x ≈ 1000mm, y ≈ 0mm
允许偏差: y < 20mm (2% 横向偏移)
```

如果 y 偏差过大，检查：
- 左右轮编码器是否对称（说明书上说的 PPR 可能有 ±5% 误差）
- 陀螺仪在直线时 output 是否接近 0
- wheel_base_mm 是否准确

### 7.2 转圈测试

**方法**：让小车原地转 360°，看 heading_deg 是否回到 ≈0°（或 ±360°）。

```
误差来源：
- 陀螺仪零偏导致积分漂移
- dt 采样抖动
```

### 7.3 串口输出调试

```c
// 典型调试代码（在 main loop 中每 100ms 打印一次）
DR_GetPosition(&dr, &x, &y);
printf("x=%.1f y=%.1f heading=%.1f\r\n", x, y, DR_GetHeading(&dr));
```

串口打印内容建议：
- x, y, heading 的浮点值（保留 1 位小数）
- 左右轮里程增量（用于排查编码器不对称）
- dt 实际值（确认调度是否准时）

---

## 8. 典型使用模式

```c
DR_t dr;

void setup(void)
{
    DR_Init(&dr, 150.0f);           // 轮距 150mm
    DR_Reset(&dr, 0, 0, 0);         // 起点(0,0), 朝0°
}

void loop(void)
{
    // 从编码器硬件读取增量
    float left_delta  = encoder_get_delta_mm(LEFT_WHEEL);
    float right_delta = encoder_get_delta_mm(RIGHT_WHEEL);
    // 从陀螺仪读取角速度
    float gyro_rate   = imu_get_gyro_z();  // °/s
    // dt 来自定时器
    float dt = get_timer_interval_s();

    DR_Update(&dr, gyro_rate, left_delta, right_delta, dt);

    // 读取结果
    float x, y, heading;
    DR_GetPosition(&dr, &x, &y);
    heading = DR_GetHeading(&dr);
}
```

---

## 9. 完整数据流图

```
编码器左 → Δs_L ─┐
                  ├→ ds = (Δs_L + Δs_R) / 2 → 位置积分
编码器右 → Δs_R ─┘                          ↓
                                     x += ds·cosθ
                                     y += ds·sinθ
陀螺仪 → ω_gyro ─→ θ += ω·dt → θ 归一化 →↑
```

---

## 10. 与 diff_drive 模块的关系

| 模块 | 方向 | 输入 → 输出 |
|------|------|-------------|
| diff_drive | 逆运动学（前向控制） | (v, ω) → (左轮转速, 右轮转速) |
| dead_reckoning | 正运动学（状态估计） | (Δs_L, Δs_R, ω_gyro) → (x, y, θ) |

两者互为**对偶关系**：
- diff_drive 告诉你"怎么走才能到目标"
- dead_reckoning 告诉你"现在在哪里"

配套使用可实现闭环：下指令 → 执行 → 读取编码器 → 修正。

---

## 11. 与 line_follower 的关系

line_follower 给出的是"线相对于车的位置偏差"，这个偏差经过 PID 控制后得到目标角速度 ω_target。然后把 ω_target 输入给 diff_drive 做逆解，同时在 dead_reckoning 里同步记录实际走过的轨迹。这样就能知道"循迹过程中机器人的实际路径是什么"。

---

## 12. 极限情况与边界行为

| 条件 | 行为 |
|------|------|
| dr == NULL | 所有函数安全返回，不 crash |
| wheel_base_mm ≤ 0 | 默认设为 100mm |
| dt ≤ 0 | Update 直接 return |
| 未调用 Reset | 第一次 Update 自动启用，位姿保持 (0,0,0) |
| heading_deg 超过 ±360° | while 循环归一化到 (-180, +180] |
| gyro_rate × dt 非常大 | 航向归一化仍能处理，但位置可能跳跃 |
| 左右编码器读数不一致 | 正常，差速底盘必然不同，平均里程仍可用 |
| 所有外部指针为 NULL | GetPosition 的 x/y 可单独为 NULL |
