# 串级PID控制器 (PID_Cascade) — 百科全书级解读

> 阅读前提：你已经完全理解了 `PID_Pos` 位置式PID模块。串级PID就是用两个位置式PID串起来——外环的输出是内环的目标值。如果你还不清楚位置式PID的积分分离、死区、限幅的细节，先回去看 pid_pos_README.md。

---

## 一、整体定位

**串级PID** 是两个 PID 控制器 **嵌套** 在一起的结构：

```
外环目标值 → [外环PID] → 内环目标值 → [内环PID] → 控制输出
                  ↑                        ↑
             外环反馈值                内环反馈值
```

这里的外环是 `PID_Pos`，内环也是 `PID_Pos`。

### 它解决什么问题？

**一个问题**：平衡车

如果用单环 PID 控制平衡车角度：
1. 设定目标角度 = 0°（直立）
2. 车子倾斜 10°，单环 PID 输出一个扭矩
3. 轮子加速，车子回正

**问题来了**：如果轮子突然遇到一个小石子（外力扰动），轮子转速变化但车身角度还没变，单环 PID **不会响应**——因为它看到的角度还是 10°，它不知道轮子已经在打滑了。等角度变化了再反应，已经晚了。

**串级的解决方式**：
- 外环（角度环）：目标 0°，反馈 10° → 输出 "目标角速度 -5°/s"
- 内环（速度环）：目标 -5°/s，反馈实际角速度 -2°/s → 输出电机扭矩
- 轮子遇到石子转速变化 → 内环立刻响应，不需要等外环察觉

**核心思想**：内环负责 **抑制内环扰动**，外环负责 **跟踪目标**。内环比外环快 3~10 倍，保证内环先稳定，外环再慢慢调整。

### 什么时候串级反而不好？

1. **内环响应速度不够快**：如果你用同一个 PWM 频率驱动内环，内环无法比外环快，那串级就没意义。必须保证内环采样率是外环的 3~10 倍。
2. **只有一个可测量的量**：如果你只有角度反馈没有角速度反馈，没法做内环。
3. **系统太简单没必要**：单电机恒速控制，单环足够。
4. **增加了调参难度**：6 个参数（外环 Kp/Ki/Kd + 内环 Kp/Ki/Kd）比 3 个难调得多。

---

## 二、逐代码拆解

### 2.1 头文件 — 结构体定义 (`pid_cascade.h:1-11`)

```c
#ifndef PID_CASCADE_H
#define PID_CASCADE_H

#include "pid_pos.h"

typedef struct {
    PID_Pos_t outer;   //外环PID,通常为位置环或速度环
    PID_Pos_t inner;   //内环PID,通常为电流环或速度环
} PID_Cascade_t;
```

#### 结构体分析

整个串级 PID 就是对两个 `PID_Pos_t` 的 **零开销封装**——不增加任何额外字段。

**为什么用 PID_Pos_t 而不是 PID_Inc_t？**

1. 位置式输出绝对值，作为内环的目标值语义清晰（"请把速度/位置维持在 X"）。
2. 如果用增量式，外环输出的是 Δu，累加后才是内环目标——多一层复杂度，而且 Δu 作为目标值没有物理意义。
3. 串级的经典教材（如 Åström 的《Advanced PID Control》）中，内外环都用位置式是最标准的做法。

**内外环的区别**：

| 特性 | outer（外环） | inner（内环） |
|------|-------------|-------------|
| 控制目标 | 主控量（位置/角度） | 辅助量（速度/角速度） |
| 响应速度 | 慢（低频更新） | 快（高频更新） |
| 扰动抑制 | 对目标变化敏感 | 对内部扰动敏感 |
| 参数 | 可能带积分消除静差 | 通常 P 或 PD，不一定要 I |

---

### 2.2 API 函数清单

```c
void PID_Cascade_Init(PID_Cascade_t *csc);
void PID_Cascade_SetOuterParam(PID_Cascade_t *csc, float kp, float ki, float kd);
void PID_Cascade_SetInnerParam(PID_Cascade_t *csc, float kp, float ki, float kd);
void PID_Cascade_Reset(PID_Cascade_t *csc);
float PID_Cascade_Update(PID_Cascade_t *csc, float outer_target,
                         float outer_feedback, float inner_feedback);
```

**API 设计特点**：
- 相比直接操作 `pid_cascade->outer` 和 `pid_cascade->inner`，这些封装函数只是减少了一行代码，但提供了 **统一的接口风格**。
- 没有提供设置限幅/死区/积分分离的封装函数——用户需要直接操作结构体：
  ```c
  PID_Pos_SetLimit(&csc->outer, 100.0f, 500.0f);
  PID_Pos_SetDeadband(&csc->inner, 1.0f);
  ```

---

### 2.3 Init 函数 (`pid_cascade.c:9-14`)

```c
void PID_Cascade_Init(PID_Cascade_t *csc)
{
    if (csc == NULL) return;
    PID_Pos_Init(&csc->outer);
    PID_Pos_Init(&csc->inner);
}
```

**解释**：

- 分别初始化内外环，所有系数/积分/限幅清零。
- Init 后两个 PID 都可以用了，但必须先设置参数。

---

### 2.4 SetParam 函数 (`pid_cascade.c:23-42`)

```c
void PID_Cascade_SetOuterParam(PID_Cascade_t *csc, float kp, float ki,
                                float kd)
{
    if (csc == NULL) return;
    PID_Pos_SetParam(&csc->outer, kp, ki, kd);
}

void PID_Cascade_SetInnerParam(PID_Cascade_t *csc, float kp, float ki,
                                float kd)
{
    if (csc == NULL) return;
    PID_Pos_SetParam(&csc->inner, kp, ki, kd);
}
```

**解释**：

- 单纯的转发，没有任何额外逻辑。
- **典型参数范围**：
  - 外环（平衡车角度→角速度）：Kp=10~100, Ki=0~1, Kd=1~10
  - 内环（角速度→扭矩）：Kp=0.5~5, Ki=0.01~0.5, Kd=0~0.5
  - 内环 Kp 通常小于外环 Kp（因为内环目标值已经是外环的输出，较小）

---

### 2.5 Reset 函数 (`pid_cascade.c:48-53`)

```c
void PID_Cascade_Reset(PID_Cascade_t *csc)
{
    if (csc == NULL) return;
    PID_Pos_Reset(&csc->outer);
    PID_Pos_Reset(&csc->inner);
}
```

**解释**：

- 重置时不改变参数，只清零积分和上次误差。
- 典型场景：停车后重新启动，或者目标发生大幅阶跃。

---

### 2.6 核心 Update 函数 (`pid_cascade.c:63-73`)

```c
float PID_Cascade_Update(PID_Cascade_t *csc, float outer_target,
                          float outer_feedback, float inner_feedback)
{
    if (csc == NULL) return 0.0f;

    float inner_target = PID_Pos_Update(&csc->outer, outer_target,
                                        outer_feedback);
    float output = PID_Pos_Update(&csc->inner, inner_target,
                                  inner_feedback);
    return output;
}
```

**这就是整个串级 PID 的全部逻辑——两行核心代码。**

#### 逐行拆解

**第1行**：
```c
float inner_target = PID_Pos_Update(&csc->outer, outer_target,
                                    outer_feedback);
```

- 外环执行一次位置式PID计算。
- `outer_target`：外环的设定值。例如平衡车的目标角度 = 0°（直立）。
- `outer_feedback`：外环的反馈值。例如当前车身倾斜角度 = 5°。
- `inner_target`：**外环的输出，作为内环的目标值**。例如 "为了回到 0°，我需要车身的角速度为 -10°/s"。

**这里有一个非常重要的理解**：

外环输出的 `inner_target` 是什么物理量？这取决于外环feedback的物理量纲：
- 如果 outer_feedback 是 **角度**（°），inner_target 的单位就是角度相关量。但内环期望的反馈是角速度（°/s），所以外环输出的是一个 "目标角速度"。
- 如果 outer_feedback 是 **角速度**（°/s），内环是电流环，那么外环输出的是一个 "目标电流"。

**简单说：外环的输出和内环的反馈必须量纲一致。**

**第2行**：
```c
float output = PID_Pos_Update(&csc->inner, inner_target,
                              inner_feedback);
```

- 内环执行一次位置式PID计算。
- `inner_target`：上面得到的值（如目标角速度 -10°/s）。
- `inner_feedback`：内环的反馈值（如当前实际角速度 -8°/s，因为有阻力）。
- `output`：最终控制输出（如 PWM 占空比或电流给定）。

**内外环调用频率不同**：

在典型实现中，内环的 Update 频率是外环的 3~10 倍：

```
// 外环 10ms 执行一次
void outer_loop_10ms(void) {
    inner_target = PID_Pos_Update(&csc->outer, target_angle, current_angle);
}

// 内环 2ms 执行一次（5倍频率）
void inner_loop_2ms(void) {
    output = PID_Pos_Update(&csc->inner, inner_target, current_speed);
    pwm_set(output);
}
```

但在这个模块里，**两个环都在一次 Cascade_Update 调用里执行**，这意味着外环和内环的更新频率相同。这其实是"半串级"——真正的串级应该有不同的频率。

**如果想实现不同频率**：
建议不调用 `PID_Cascade_Update`，而是直接在外部分别调用：
```c
// 外环：每 10ms
if (tick % 5 == 0) {
    inner_target = PID_Pos_Update(&csc->outer, angle_target, angle_fb);
}
// 内环：每 2ms
output = PID_Pos_Update(&csc->inner, inner_target, speed_fb);
```

---

## 三、串级PID的数学分析

### 3.1 结构框图

```
                 扰动1(负载变化)        扰动2(摩擦力)
                       ↓                     ↓
target → [外环PID] → inner_target → [内环PID] → [被控对象] → outer_feedback
    ↑                                                         |
    └────────────────── 外环反馈──────────────────────────────┘
                                       内环反馈 ──────────────┘
```

### 3.2 为什么串级有效？

传递函数的角度：

单环系统：
```
G(s) = C(s) * P(s)
```

串级系统：
```
G(s) = C_outer(s) * C_inner(s) * P(s) / (1 + C_inner(s) * P(s))
```

**关键**：内环闭环 `C_inner(s) * P(s) / (1 + C_inner(s) * P(s))` 的带宽远大于外环带宽。所有内环扰动（摩擦力矩波动、电压波动）先被内环抑制，轮不到外环操心。

### 3.3 什么时候内环不需要 I？

内环通常只需要 **PD 控制**（Ki = 0），原因：
1. 内环频率很高，积分时间常数长到没效果
2. 内环的稳态误差可以被外环的积分消除
3. 内环加 I 容易导致内外环相互振荡

但如果你内环控制的量本身有静差（比如电流传感器有偏置），可以加很小的 I。

---

## 四、参数整定指导

**串级 PID 的整定原则：先内后外，由快及慢。**

### Step 0: 确定采样率

- 内环采样率 = 外环采样率 × (3~10)
- 例如电机速度环内环 1ms，位置环外环 5ms~10ms

### Step 1: 整定内环（内环 Kp, Ki, Kd）

1. 断开外环：设外环目标 = 固定值（如目标角速度 0），内环独立运行
2. 给一个阶跃目标值（如目标角速度 100°/s）
3. 按单环 PID 整定法调内环参数
4. 目标：内环响应快、无静差、无振荡

### Step 2: 整定外环（外环 Kp, Ki, Kd）

1. 闭合内外环
2. 给外环一个阶跃目标（如目标角度 10°）
3. 按单环 PID 整定法调外环参数
4. 目标：整个系统的响应符合要求

### Step 3: 微调

- 如果外环响应慢 → 增大外环 Kp
- 如果系统有振荡 → 判断是内环振荡还是外环振荡（频率高的振荡是内环，频率低的是外环）
- 如果内环响应跟不上外环 → 增大内环 Kp 或提高内环采样频率

### 典型值参考（平衡车角度 + 角速度控制）

| 参数 | 外环（角度 → 目标角速度） | 内环（角速度 → PWM） |
|------|------------------------|--------------------|
| Kp | 15.0 ~ 50.0 | 2.0 ~ 8.0 |
| Ki | 0.0 ~ 1.0 | 0.0 ~ 0.5 |
| Kd | 1.0 ~ 10.0 | 0.1 ~ 2.0 |
| output_limit | ±200°/s（取决于电机极限） | PWM_MAX |
| integral_limit | 外环限幅的一半 | 100~500 |

---

## 五、深度踩坑点

### 5.1 内外环的"共振"

如果外环和内环的带宽过于接近，两个环会互相耦合，产生"共振"现象——输出在某个频率上持续振荡，且比例增益越大振荡越严重。

**标志**：无论怎么调参，系统在一定频率上稳定不下来。

**解决方案**：
1. 确保内环比外环快至少 3 倍
2. 外环的带宽设为内环带宽的 1/5 ~ 1/10
3. 外环的 Kd 不要太大（微分项超前可能会拉近内外环相位）

### 5.2 外环输出作为内环目标时的限幅

外环输出（`inner_target`）在传给内环之前，**必须限幅**。

```c
float inner_target = PID_Pos_Update(&csc->outer, outer_target, outer_fb);

// 必须限幅！否则内环可能收到一个不合理的目标值
if (inner_target > MAX_SPEED) inner_target = MAX_SPEED;
if (inner_target < -MAX_SPEED) inner_target = -MAX_SPEED;

float output = PID_Pos_Update(&csc->inner, inner_target, inner_fb);
```

**为什么**：外环的 `output_limit` 可能不是按照内环反馈的物理单位来设置的。例如外环输出 500，但电机最大只能承受 200 的角速度。如果不限幅，内环收到目标 500，但实际只能跑到 200，积分一直累加，产生饱和。

**当前模块的不足**：没有在 Cascade_Update 内部自动限幅 inner_target。用户需要手动处理。

### 5.3 外环死区 + 内环振荡

如果外环设了 deadband，在死区附近外环输出为 0，内环目标突然变为 0，导致内环输出跳变。

**建议**：外环不要设死区，或者死区设得非常小。死区应该设在最终输出上，而不是内外环之间。

### 5.4 内环反馈噪声对外环的影响

内环反馈值有噪声 → 内环输出有噪声 → 被控对象抖动 → 外环反馈也有噪声。

这是一个正反馈环——内环的噪声会通过外环反馈回到外环输入。

**解决**：
1. 内环反馈要滤波（硬件 LPF 或软件滑动平均）
2. 内环 Kd 尽可能小（或者设为 0）
3. 外环的采样频率低于内环（天然低通滤波效果）

### 5.5 积分饱和的双重放大

外环积分饱和 → inner_target 过大 → 内环误差巨大 → 内环积分也饱和 → 输出饱和。

**两个积分在串联**，问题放大。这是串级 PID 最棘手的问题之一。

**缓解方案**：
1. 外环 `integral_limit` 设得小一点
2. 外环启用积分分离（`SetSepThreshold`）
3. 对 inner_target 做限幅

---

## 六、数据范围与溢出分析

### 6.1 内外环数据流

```
outer_target (float, 如 ±180°)
    → outer_feedback (float, 如 ±180°)
    → inner_target (float, 如 ±1000°/s)
    → inner_feedback (float, 如 ±1000°/s)
    → output (float, 最终控制量)
```

### 6.2 溢出风险

**内环目标值 overflow**：

若外环输出 `integral` 累计到非常大（比如 1e6），乘以 Ki 后得到很大的 inner_target，传给内环后：
- 内环的 error 计算为 `inner_target - inner_feedback`，最大值可达 1e6
- 如果内环没有 output_limit，最终输出可能为 1e6

**解决方案**：对外环输出限幅 / 对内环输入限幅，双保险。

---

## 七、调用链

### 7.1 典型系统：平衡车

```
MPU6050 (10ms)
  ├── DMP 读取四元数 → 计算 pitch angle = 外环反馈
  └── 陀螺仪 raw → 滤波 → 角速度 = 内环反馈

控制任务 (2ms 内环, 10ms 外环)
  ├── 每 10ms: outer_target = 0°（直立）
  │   → PID_Pos(outer) → inner_target = 目标角速度
  └── 每 2ms: PID_Pos(inner) → PWM 输出

→ 电机驱动 (TB6612 / DRV8833)
  ├── PWM 占空比
  └── 方向引脚
```

### 7.2 典型系统：循迹小车

```
摄像头 / 灰度传感器 (20ms)
  └── 位置偏差 = 外环反馈

编码器 (5ms)
  └── 左右轮差速 = 内环反馈

控制任务 (5ms)
  ├── 外环 (20ms): 位置偏差 → 目标转速差
  └── 内环 (5ms): 目标转速差 → 左右轮 PWM 差

→ 双路电机驱动
  ├── 左轮: base_speed - diff/2
  └── 右轮: base_speed + diff/2
```

### 7.3 典型系统：四轴无人机

```
姿态估计 (1ms)
  ├── 角度 (roll/pitch/yaw) = 外环反馈
  └── 角速度 (gyro) = 内环反馈

控制:
  ├── 外环 (4ms): 角度 → 目标角速度
  └── 内环 (1ms): 目标角速度 → 电机油门
```

### 7.4 不调用 PID_Cascade_Update 的替代方案

如果内外环频率不同，直接操作 `csc->outer` 和 `csc->inner`：

```c
static float inner_target = 0;

// 10ms 定时器：外环
void timer_10ms(void) {
    inner_target = PID_Pos_Update(&csc->outer, target_angle, angle_fb);
    // 限幅
    if (inner_target > 200) inner_target = 200;
    if (inner_target < -200) inner_target = -200;
}

// 2ms 定时器：内环
void timer_2ms(void) {
    float out = PID_Pos_Update(&csc->inner, inner_target, speed_fb);
    pwm_set(out);
}
```

---

## 八、调试手段

### 8.1 分层调试（最重要）

串级调试的唯一正确方法是：**先单独调试内环，再调试外环**。

**内环调试**：
```c
// 临时固定外环目标
float inner_target = 100.0f;  // 固定目标角速度
float output = PID_Pos_Update(&csc->inner, inner_target, gyro_speed);
pwm_set(output);
// 用串口打出 inner_target, gyro_speed, output
printf("%f,%f,%f\r\n", inner_target, gyro_speed, output);
```

**外环调试**：
```c
// 内环调好后再闭合
float inner_target = PID_Pos_Update(&csc->outer, 0.0f, pitch_angle);
float output = PID_Pos_Update(&csc->inner, inner_target, gyro_speed);
// 四个变量全打
printf("%f,%f,%f,%f,%f\r\n", 
       0.0f, pitch_angle, inner_target, gyro_speed, output);
```

### 8.2 关键观测指标

| 指标 | 内环 | 外环 |
|------|------|------|
| 阶跃响应时间 | < 50ms | < 500ms |
| 超调量 | < 5% | < 10% |
| 抗扰能力 | 负载突变时内环反馈波动 < 10% | 外环反馈波动 < 5% |
| 噪声水平 | 输出抖动 < ±1% | 内环目标值抖动 < ±2% |

### 8.3 常见问题快速诊断

| 现象 | 原因 | 解决 |
|------|------|------|
| 高频振荡（周期 < 10ms） | 内环 Kp 过大 | 减小内环 Kp |
| 中频振荡（10~50ms） | 内环 Kd 过大 | 减小内环 Kd |
| 低频振荡（100ms~1s） | 外环 Kp 过大 | 减小外环 Kp |
| 超调后恢复慢 | 外环 Ki 太大 | 减小外环 Ki |
| 响应太慢，跟不上目标 | 外环 Kp 太小 | 增大外环 Kp |
| 偶尔抖动一下 | 传感器噪声 | 加滤波，减小 Kd |

---

## 九、完整使用示例

```c
#include "pid_cascade.h"

PID_Cascade_t balance_pid;

// 配置参数
void balance_init(void)
{
    PID_Cascade_Init(&balance_pid);
    
    // 外环：角度环 (P)
    PID_Cascade_SetOuterParam(&balance_pid, 25.0f, 0.0f, 2.0f);
    PID_Pos_SetLimit(&balance_pid.outer, 50.0f, 200.0f);
    PID_Pos_SetSepThreshold(&balance_pid.outer, 30.0f);
    
    // 内环：角速度环 (PD)
    PID_Cascade_SetInnerParam(&balance_pid, 3.5f, 0.01f, 0.5f);
    PID_Pos_SetLimit(&balance_pid.inner, 500.0f, 4000.0f);
}

// 2ms 定时器调用
void control_loop_2ms(void)
{
    static int tick = 0;
    tick++;
    
    // 内环反馈：陀螺仪角速度（每次更新）
    float gyro_speed = read_gyro();
    
    // 外环反馈：角度（每 10ms 更新一次）
    // 注意：这里假设角度计算比控制慢
    float pitch_angle = get_pitch_angle();
    
    // 串级计算
    float output = PID_Cascade_Update(&balance_pid,
                                       0.0f,           // 目标角度 0°
                                       pitch_angle,    // 实际角度
                                       gyro_speed);    // 实际角速度
    
    // 限幅输出
    if (output > PWM_MAX) output = PWM_MAX;
    if (output < -PWM_MAX) output = -PWM_MAX;
    
    // 驱动电机
    motor_drive(output);
}
```

---

## 十、总结

`PID_Cascade` 模块用最少的代码（73 行）实现了最核心的串级 PID 控制结构。

**它做了什么**：
- 封装两个 `PID_Pos_t` 为一个结构体
- Update 时自动将外环输出作为内环目标
- 提供统一的初始化/设参/重置接口

**它没做什么**（需要用户自己处理的重要事项）：
1. 内外环不同频率的支持（需要在外部各自调用）
2. inner_target 的限幅（外环输出限制）
3. 内外环相互独立的限幅设置（通过操作子结构体实现）
4. 无扰切换（从开环到闭环的平滑过渡）

**串级的本质**：
- 用更多的参数换更好的抗扰性能
- 快的控制快的（内环），慢的控制慢的（外环）
- 先稳内再稳外
