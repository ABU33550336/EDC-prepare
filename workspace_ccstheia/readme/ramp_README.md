# 斜坡发生器 (Ramp) — 百科全书级解读

> 阅读前提：你正在控制一个电机/舵机/加热器，发现"目标直接给过去"会导致猛冲/抖动/过冲。你需要一个平滑的过渡——这就是 Ramp 做的事。

---

## 一、整体定位

**斜坡发生器（Ramp Generator）** 是一个纯软件的运动规划器。它不关心物理模型、不涉及控制理论——它的工作极其简单：

> 给定一个目标值，每个周期向目标值靠近一步，步长不超过设定值。

```
初始值 = 0，目标 = 100，步长 = 5
周期 1: 5
周期 2: 10
周期 3: 15
...
周期 20: 100（达到目标）
```

### 它解决什么问题？

如果 PID 的目标值直接从 0 阶跃到 1000：
- PID 看到 error = 1000，比例项输出巨大
- 电机瞬间全速运转
- 机械冲击（齿轮撞击、皮带抖动）
- 电流冲击（可能触发过流保护）
- 过冲严重（系统惯性无法快速响应这么大的阶跃）

用 Ramp 后，目标值平滑地从 0 爬到 1000，PID 的误差不会突然变大，系统平稳启动。

### 它不是什么

- 不是滤波器（滤波器改变信号的频率成分，Ramp 只是限制变化率）
- 不是 PID 控制器（不基于误差反馈计算输出）
- 不是规划器（专业的 S-Curve / T-Curve 运动规划器会考虑加速度和加加速度限制）

本模块只实现 **梯形速度曲线中最简单的一种：恒速逼近**（即加速度阶段为 0→步长直接达到最大速度，然后匀速，最后一步直接到位）。

---

## 二、逐代码拆解

### 2.1 头文件 — 结构体定义 (`ramp.h:1-12`)

```c
#ifndef RAMP_H
#define RAMP_H

#include <stdint.h>

typedef struct {
    float current;  //当前输出值
    float step;     //每步变化量
    float min;      //最小值下限
    float max;      //最大值上限
} Ramp_t;
```

#### 逐字段分析

| 字段 | 类型 | 物理意义 | 典型值 |
|------|------|----------|--------|
| `current` | float | 当前时刻的输出值（也是状态值） | 初始值 |
| `step` | float | 每周期最大变化量（必须是正数） | 1~1000 |
| `min` | float | 输出下限（绝对值） | 0 或 -max |
| `max` | float | 输出上限（绝对值） | 根据系统定 |

#### 为什么叫"斜坡"而不是"加速度"？

**关键概念区分**：

| 概念 | 本模块 | 专业运动控制 |
|------|--------|-------------|
| 速度 | step 决定了**速度**（每周期变化量） | 有加加速度限制 |
| 加速度 | 从 0 瞬间到 step — 无限大 | 有限制 |
| 加加速度 (jerk) | 无限制 | 有限制（S-Curve） |

**本模块不是梯形加减速发生器，而是"恒速斜坡"**。真正的梯形加减速有加速段、匀速段、减速段三段，而本模块只有一段——恒速逼近，最后一步直接到位。

如果你需要真正的梯形加减速（加速→匀速→减速），你需要计算：
- 加速距离 = v²/(2a)
- 减速距离 = v²/(2a)
- 当总距离不够时直接进入三角形（无匀速段）

这个模块不做这些——它只是一个简单的限步长逼近器。

---

### 2.2 初始化函数 (`ramp.c:12-25`)

```c
void Ramp_Init(Ramp_t *ramp, float step, float min, float max)
{
    if (ramp == NULL) return;
    if (step < 0.0f) step = -step;
    ramp->step = (step > 0.0f) ? step : 1.0f;
    if (max < min) {
        float tmp = min;
        min = max;
        max = tmp;
    }
    ramp->min = min;
    ramp->max = max;
    ramp->current = (min + max) * 0.5f;
}
```

#### 逐行拆解

**`if (step < 0.0f) step = -step;`**

- 步长不能为负。如果用户传了负步长，自动取绝对值。
- 为什么用户会传负步长？可能是误操作，也可能是 `#define STEP -5` 这样的宏定义。防御性编程。

**`ramp->step = (step > 0.0f) ? step : 1.0f;`**

- 如果步长正好为 0，电机卡死——永远到达不了目标。所以默认为 1。
- 这是合理的：宁可慢（step=1）也不能不动（step=0）。

**`if (max < min) { swap(min, max); }`**

- 如果用户传了 min=100, max=0，自动纠正。
- 这一步非常重要：没有这一步的话，后续所有限幅逻辑都失效（min > max 会导致 current 永远被钳位在错误的范围）。

**`ramp->current = (min + max) * 0.5f;`**

- 初始值设为中点。
- 这是一个有争议的设计选择：为什么不设为 min 或 0？
  - 设为中点：系统启动时从中间开始向目标运动，比较柔和。
  - 设为 min：从最低点开始，适合"默认关闭"的执行器（如阀门）。
  - 设为 max：从最高点开始，适合"默认全开"的执行器。
- 你可以在 Init 后用 `Ramp_Reset` 重新设置初始值。

---

### 2.3 设置步长 (`ramp.c:32-37`)

```c
void Ramp_SetStep(Ramp_t *ramp, float step)
{
    if (ramp == NULL) return;
    if (step < 0.0f) step = -step;
    ramp->step = (step > 0.0f) ? step : 1.0f;
}
```

**解释**：

- 与 Init 中的步长处理逻辑一致。
- 运行时可以动态改变步长——例如启动时大步长（快速起步），接近目标时小步长（精确到位）。

**运行时改变步长的效果**：
```
周期 1-10: step = 10, current 从 0→100
周期 10:   设 step = 1
周期 11-20: 目标 100，但 current 还在 100，所以不动
周期 21:   目标变为 200，每次只走 1
```

---

### 2.4 重置值 (`ramp.c:44-50`)

```c
void Ramp_Reset(Ramp_t *ramp, float value)
{
    if (ramp == NULL) return;
    if (value < ramp->min) value = ramp->min;
    if (value > ramp->max) value = ramp->max;
    ramp->current = value;
}
```

**解释**：

- 将 current 立即设为 value（带限幅）。
- 这相当于"直跳"——突破了斜坡的限制。用于紧急情况（如急停时直接设为 0）。
- **不改变 step/min/max**。

---

### 2.5 核心 Update 函数 (`ramp.c:58-73`)

```c
float Ramp_Update(Ramp_t *ramp, float target)
{
    if (ramp == NULL) return 0.0f;
    if (target < ramp->min) target = ramp->min;
    if (target > ramp->max) target = ramp->max;

    float diff = target - ramp->current;
    if (diff > ramp->step) {
        ramp->current += ramp->step;
    } else if (diff < -ramp->step) {
        ramp->current -= ramp->step;
    } else {
        ramp->current = target;
    }
    return ramp->current;
}
```

#### 逐行拆解

**输入限幅**：
```c
if (target < ramp->min) target = ramp->min;
if (target > ramp->max) target = ramp->max;
```

- 目标值被钳位到 [min, max]。
- 这意味着你永远无法通过 Ramp 输出超过范围的值——即使你传了 10000，它也会被截断到 max。
- 这个限幅同时保护了 ramp->current（因为 current 是每次累加 step，不会超过 target，而 target 已经被限幅了）。

**计算差值**：
```c
float diff = target - ramp->current;
```

- `diff` 是当前位置到目标的距离。
- `diff > 0`：需要正向移动。
- `diff < 0`：需要反向移动。

**三种情况**：

```c
if (diff > ramp->step) {
    ramp->current += ramp->step;           // 情况1：正向远离，走一步
} else if (diff < -ramp->step) {
    ramp->current -= ramp->step;           // 情况2：负向远离，退一步
} else {
    ramp->current = target;                // 情况3：一步之内，直达目标
}
```

- **情况1 和 2**：距离大于步长，每次只走一个步长。这就是"限斜率"的核心逻辑。
- **情况3**：距离小于等于步长，直接跳到位。这是本模块的重要特性——**最后一步可能跳变**。

**最后一步跳变的问题**：

假设 step=10，target=97，current 在 90：
```
diff = 97 - 90 = 7
7 < 10 → 情况3 → current = 97
```
一步从 90 跳到 97，变化量 7，小于 step。没问题。

但如果 target=101，step=10：
```
diff = 101 - 90 = 11
11 > 10 → 情况1 → current = 100
下一次：
diff = 101 - 100 = 1
1 < 10 → 情况3 → current = 101
```
仍然平滑。

真正的问题是：**当 target 频繁变化时，current 永远追不上 target，一直处于情况1/2，形成线性斜坡**。这就是名字"斜坡"的来源。

---

## 三、数学描述

### 3.1 离散时间方程

```
current[0] = init_value (或 mid)

对于每个 k >= 1:
  target' = clamp(target, min, max)
  diff = target' - current[k-1]
  
  if diff > step:
      current[k] = current[k-1] + step
  elif diff < -step:
      current[k] = current[k-1] - step
  else:
      current[k] = target'
```

### 3.2 到达时间计算

如果 target 固定不变，从 init 到 target 需要的周期数：

```
cycles = ceil(|target - init| / step)
```

例如 init=0, target=1000, step=50：
```
cycles = ceil(1000 / 50) = 20
时间 = 20 * T（T 为采样周期）
```

如果 T=10ms：
```
到达时间 = 20 * 10ms = 200ms = 0.2s
```

### 3.3 输出形状

```
current
  ^
  |               __________________ target
  |             /
  |           /
  |         /          ← 线性上升
  |       /
  |     /
  |   /
  | /
  +------------------------→ time
  
  init
```

这是个 **线性斜坡**，不是梯形。没有减速段——在最后一步之前一直是全速（step），最后一步戛然而止。

---

## 四、为什么用 Ramp？

### 4.1 用途一：PID 目标值预处理器（最常见）

```
用户输入目标 → [Ramp] → 平滑目标 → [PID] → 执行器
```

- 用户按一下按钮，目标从 0 阶跃到 500。
- Ramp 把它变成 0→10→20→...→500 的平滑序列。
- PID 的误差不会突变，避免了积分骤增和微分冲击。

### 4.2 用途二：直接作为开环控制（无传感器系统）

对于没有反馈的步进电机或舵机：
```
RAMp → PWM/脉冲
```

通过控制步进电机的脉冲频率从低到高变化，实现软启动。虽然本模块只支持恒速上升（没有加速度控制），但在简单场景下够用。

### 4.3 用途三：信号平滑器

当传感器数值跳变太大时，用 Ramp 代替低通滤波：

```
传感器 RAW → [Ramp] → 平滑值
```

效果：抑制跳变，但保持响应速度（Ramp 的延迟是固定的 T * cycles，而低通滤波的延迟是变化的）。

### 4.4 什么时候不适用

1. **需要真正的 S 曲线**：CNC、3D 打印机、工业机器人——这些需要限制加加速度（jerk）来防止机械振荡。本模块做不到。
2. **位置模式下的步进电机**：目标位置一次到位，需要梯形加减速曲线。本模块会导致最后一步到位时速度突变。
3. **实时性要求极高**：Ramp 本身延迟固定，但如果加上 PID 闭环，总的响应时间 = Ramp 时间 + PID 调节时间，可能太长。

---

## 五、参数整定（步长与限幅）

### 5.1 步长（step）的设置

步长的选择取决于两个因素：

1. **系统允许的最大变化率**：电机最大加速度是多少？
2. **采样周期**：每次调用的时间间隔。

**从物理量计算 step**：

```
step = max_acceleration * T / k
```

其中：
- `max_acceleration`：系统能承受的最大加速度（rpm/s 或 mm/s²）
- `T`：采样周期（s）
- `k`：比例系数（把加速度映射到输出单位）

**举例**：电机 0~1000 RPM 需要 0.5s（即最大加速度 = 2000 RPM/s），T=10ms=0.01s：
```
step = 2000 * 0.01 = 20 RPM/步
```

需要 1000/20 = 50 步 = 500ms 从 0 到 1000。

### 5.2 min/max 限幅的物理意义

- min：执行器的最小安全值（如电机最低 PWM 占空比 5%，太低转不动）
- max：最大安全值（如舵机最大角度 180°）

**注意**：min/max 不仅限幅输出，也限幅目标输入。如果你传 target=-100，但 min=0，target 被截断到 0，永远不会输出负数。

### 5.3 没有直接的"加速度"或"减速度"设置

这是本模块与真正的梯形加减速器的最大区别：

```
真正的梯形曲线：
加速 (0→v_max) → 匀速 (v_max) → 减速 (v_max→0)

本模块：
只有匀速 (v_step)，没有加/减速（或者说加速度无穷大）
```

所以如果 step=50，第一个周期就是跳 50（从 0→50），加速度无穷大。某些机械系统（如大惯量负载）可能无法接受这个冲击。

**改进建议**：如果你需要分级加速，可以自己封装两层：
```c
Ramp_t speed_ramp;   // 控制"速度"（步长）
Ramp_t pos_ramp;     // 控制"位置"

// 每次 Update:
float current_step = Ramp_Update(&speed_ramp, target_step); // 步长逐渐增大
Ramp_SetStep(&pos_ramp, current_step);                      // 步长动态变化
float output = Ramp_Update(&pos_ramp, target_pos);          // 位置逐渐逼近
```

这样实现了"先慢后快再慢"的效果——虽然还不是真正的 S 曲线，但已经比单纯的恒速斜坡前进了一大步。

---

## 六、深度踩坑点

### 6.1 最后一个周期的跳变

如前面分析的，当 `|diff| <= step` 时，current 直接跳到 target。这个跳变量最小 0 最大 step。

对于某些应用（如下降到目标位置）：
```
step=50, target=0, current=30
30 - 0 = 30 <= 50 → current 直接跳到 0
```

变化量 30，但是是 **硬停**——如果负载有惯性，这个硬停会导致抖动。

**解决方案**：在接近目标时自动减小 step（Z 形逼近），但本模块不支持。你可以这样做：
```c
float target = 100.0f;
float remaining = fabsf(target - ramp.current);
if (remaining < 3 * ramp.step) {
    ramp.step = remaining * 0.3f;  // 最后三步减速
}
float out = Ramp_Update(&ramp, target);
```

### 6.2 步长与目标变化率的匹配

如果目标本身在快速变化（比如通过串口接收的遥控信号），可能会出现：
```
target: 0 → 50 → 100 → 80 → 120 (每 100ms 变化一次)
step = 20, T = 10ms

周期 1: current = 0, target = 0   → current = 0
周期 2: current = 0, target = 50  → current = 20 (增加 20)
周期 3: current = 20, target = 50 → current = 40
周期 4: current = 40, target = 50 → current = 50
周期 5: current = 50, target = 100 → current = 70
...
周期 14: current = 120, target = 80 → current = 100 (下降)
...
```

current 的轨迹始终追着 target 跑，形成一个延迟的、平滑的版本。这就是 Ramp 作为"目标平滑器"的工作方式。

**但如果 step 太大**：平滑效果消失，current 几乎跟着 target 一起跳。
**如果 step 太小**：响应太慢，操作有延迟感。

### 6.3 初始值选择不当

`Ramp_Init` 把 `current` 设为 `(min+max)/2`，这不总是合适的：
- 电机启动：应该从 0 开始（current=0），而不是从中间开始。
- 舵机 0°~180°：中点 90°，如果舵机初始在 0°，Init 后 current=90°，舵机会突然跳到 90°。

**解决方案**：Init 后立即 Reset 到正确的初始位置：
```c
Ramp_t ramp;
Ramp_Init(&ramp, 10.0f, 0.0f, 1000.0f);
Ramp_Reset(&ramp, 0.0f);  // 强制从 0 开始
```

### 6.4 步长为0导致死循环（已被防御）

源码中处理了：`step=0 → step=1`。但 step=1 可能导致响应极慢（1000 目标需要 1000 步）。

**现象**：程序还在运行，current 在缓慢变化，但用户感觉系统"卡死了"。

**诊断**：检查 Ramp 的 step 参数是否被意外设为 0。

### 6.5 没有溢出保护

current, step, min, max 都是 float。如果 step 无限累加：
```c
// 用户错误调用：每次改目标前都 SetStep
for (;;) {
    Ramp_SetStep(&ramp, 1e10);  // 设超大 step
    Ramp_Update(&ramp, 1e10);   // 直接跳到目标
}
```
Ramp 不会阻止你——它假设用户不会这样用。

---

## 七、数据范围与溢出分析

### 7.1 变量范围

| 变量 | 最小 | 最大 | 溢出风险 |
|------|------|------|---------|
| current | min | max | 无（被钳位在 [min,max]） |
| step | 1.0f（自动保护） | FLT_MAX | 步长太大会导致 current 跳变，但不会溢出 |
| min | -FLT_MAX | +FLT_MAX | 用户负责设置合理值 |
| max | -FLT_MAX | +FLT_MAX | 同上 |

### 7.2 计算溢出

`diff = target - current`：
- target 和 current 都被限制在 [min, max]，所以 diff 最大值是 `max - min`。
- 如果 max = 1e30, min = -1e30，diff 最大 2e30——float 能表示，但精度已经差到不如用 int 了。

**实际建议**：min/max 不要超过 ±1e6，超出后 float 的精度损失已经很明显。

### 7.3 类型转换风险

```c
float out = Ramp_Update(&ramp, target);
uint16_t pwm = (uint16_t)out;  // 风险：如果 out 是负数或 > 65535
```

**安全做法**：
```c
float out = Ramp_Update(&ramp, target);
if (out > PWM_MAX) out = PWM_MAX;
if (out < 0) out = 0;
uint16_t pwm = (uint16_t)(out + 0.5f);  // 四舍五入
```

---

## 八、调用链和上层模块

### 8.1 PID 前置平滑器

```
main_control_task (10ms)
  ├── Ramp_Update(&ramp, user_target) → smooth_target
  ├── PID_Pos_Update(&pid, smooth_target, feedback) → output
  └── pwm_set(output)
```

这是最常见的用法。当用户按钮改变目标值时，Ramp 缓动目标值，PID 获得平滑的输入。

### 8.2 舵机/电机开环缓启动

```
main_control_task (20ms)
  ├── Ramp_Update(&ramp, desired_speed) → current_speed
  └── pwm_set(current_speed)   // 直接写 PWM，无反馈
```

适用场景：没有编码器的直流电机，或者对精度要求不高的风机/水泵。

### 8.3 与串级 PID 配合

```
control_task (2ms)
  ├── Ramp_Update(&target_ramp, user_angle) → smooth_angle
  ├── PID_Cascade_Update(&csc, smooth_angle, angle_fb, speed_fb)
  └── motor_drive(output)
```

目标角度经过 Ramp 缓动后再喂给串级 PID 的外环，避免大角度阶跃导致内环目标值跳变。

### 8.4 上层模块

在典型的平衡车/循迹小车系统中，调用方可能是：
```
line_follower.c
    └── 根据传感器偏离量计算目标角速度
         └── Ramp_Update → 平滑后的目标角速度
              └── PID_Cascade_Update → PWM
```

或者：
```
remote_control.c
    └── 上位机通过串口发目标速度
         └── Ramp_Update → 缓动目标速度
              └── PID_Inc_Update → 电机控制
```

---

## 九、调试手段

### 9.1 三步验证

**Step 1**：验证 Ramp 是否正常工作

```c
// 固定目标，观察输出
Ramp_Init(&ramp, 10.0f, 0.0f, 500.0f);
Ramp_Reset(&ramp, 0.0f);

for (int i = 0; i < 100; i++) {
    float out = Ramp_Update(&ramp, 500.0f);
    printf("%d,%f\r\n", i, out);
}
```

预期输出：0, 10, 20, ..., 490, 500, 500, ...

**Step 2**：验证限幅

```c
// 目标超出限幅
Ramp_Reset(&ramp, 100.0f);
float out = Ramp_Update(&ramp, 1000.0f);  // max=500
printf("%f\n", out);  // 应该输出 110 (100+10) 而不是 1000
```

**Step 3**：验证目标反转

```c
// 目标从正变负
Ramp_Reset(&ramp, 400.0f);
for (int i = 0; i < 100; i++) {
    float out = Ramp_Update(&ramp, 0.0f);
    printf("%d,%f\r\n", i, out);
}
```

预期输出：400, 390, 380, ..., 10, 0, 0, ...

### 9.2 串口绘图

```c
Ramp_t ramp;
Ramp_Init(&ramp, 20.0f, 0.0f, 1000.0f);
Ramp_Reset(&ramp, 0.0f);

// 模拟目标变化
float targets[] = {0, 500, 500, 1000, 1000, 0, 0};
int target_idx = 0;
int tick = 0;

void loop_10ms(void) {
    if (tick % 50 == 0) {  // 每 500ms 换一次目标
        target_idx = (target_idx + 1) % 7;
    }
    float out = Ramp_Update(&ramp, targets[target_idx]);
    printf("%d,%f,%f\r\n", tick, out, targets[target_idx]);
    tick++;
}
```

在 SerialPlot 中应该看到：斜坡上升→保持→再上升→保持→下降→保持。

### 9.3 实际系统的响应对比

调试 PID + Ramp 系统时，对比两种情况：

**工况 A**: 没有 Ramp，目标直接给 PID
```
PID_Update(pid, 1000, feedback)  // error = 1000 - feedback
```

**工况 B**: Ramp 平滑目标
```
float target = Ramp_Update(&ramp, 1000);
PID_Update(pid, target, feedback)
```

观测指标：

| 指标 | 无 Ramp | 有 Ramp |
|------|---------|---------|
| 启动电流 | 大 | 小 |
| 机械冲击 | 有 | 无 |
| 过冲 | 大 | 小 |
| 到达时间 | 快 | 慢（受 step 限制） |

**如果到达时间比预期长**：增大 step。
**如果启动冲击还是大**：减小 step。
**如果系统在 Ramp 结束后过冲**：PID 参数问题，不是 Ramp 的问题。

---

## 十、完整使用示例

```c
#include "ramp.h"
#include "pid_pos.h"

Ramp_t target_ramp;
PID_Pos_t motor_pid;

void system_init(void)
{
    // Ramp 配置
    Ramp_Init(&target_ramp, 50.0f, 0.0f, 3000.0f);
    Ramp_Reset(&target_ramp, 0.0f);
    
    // PID 配置
    PID_Pos_Init(&motor_pid);
    PID_Pos_SetParam(&motor_pid, 2.0f, 0.1f, 0.2f);
    PID_Pos_SetLimit(&motor_pid, 500.0f, 3000.0f);
}

// 10ms 定时器
void control_loop(void)
{
    static uint16_t user_target = 0;
    
    // 假设某个事件改变了 user_target
    // (例如电位器 ADC 值或串口命令)
    
    // Ramp 平滑目标
    float smooth_target = Ramp_Update(&target_ramp, (float)user_target);
    
    // PID 控制
    float feedback = read_encoder_speed();
    float output = PID_Pos_Update(&motor_pid, smooth_target, feedback);
    
    // 输出
    pwm_set(output);
}

// 急停
void emergency_stop(void)
{
    Ramp_Reset(&target_ramp, 0.0f);
    PID_Pos_Reset(&motor_pid);
    pwm_set(0);
}
```

---

## 十一、如果要做的更好：真正的梯形加减速

本模块只实现了恒速斜坡。真正的梯形加减速器需要三个额外参数：

```
目标位置 Pos
最大速度 Vmax
加速度 Accel
减速度 Decel（通常等于 Accel）
```

计算流程：
1. 计算加速距离: d_acc = Vmax² / (2 * Accel)
2. 计算减速距离: d_dec = Vmax² / (2 * Decel)
3. 计算总距离: d_total = target - current
4. 如果 d_total <= d_acc + d_dec（三角形，没有匀速段）:
   - Vpeak = sqrt(2 * Accel * Decel * d_total / (Accel + Decel))
5. 否则（梯形，有匀速段）:
   - Vpeak = Vmax

如果你需要这种更专业的运动规划，建议使用专门的运动控制库（如 TI 的 Motor Control SDK 中的运动规划器），或者自己写三段式逻辑。

---

## 十二、总结

`Ramp` 模块用仅 73 行 C 代码实现了 **最基本的斜坡发生器**。

**它做了什么**：
- 每周期向目标逼近一步，步长固定
- 输入/输出自动限幅到 [min, max]
- 零硬件依赖，纯算术运算
- 步长为 0 的防卡死保护
- min > max 的自动纠正

**它没做什么**（如果你需要这些功能，得自己封装）：
1. 真正的梯形加减速（加速→匀速→减速三段）
2. 最后的平滑停靠（当前是直接跳到位）
3. 加速度限制（步长直接从 0 到 step，加速度无穷大）
4. 运行时动态自适应步长（如系统误差大时自动增大步长）

**一句话总结**：如果你需要"让任何变化过程都变得平滑可控"，Ramp 就是你需要的最小实现。如果需要专业的运动规划，请找梯形/S 曲线规划器。
