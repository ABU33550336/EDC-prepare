# 位置式PID控制器 (PID_Pos) — 百科全书级解读

> 阅读前提：如果你连 "P、I、D 分别是什么" 都还不清楚，建议先去B站看一遍"DR_CAN"的PID入门视频再回来。这里只讨论代码实现层面的每一个细节。

---

## 一、整体定位

**位置式PID** 是PID家族里最"原始"的形式——它直接计算出 **当前时刻应该输出的控制量绝对值**。

```
u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt
```

经过离散化（后向差分法）后变成：

```
u[k] = Kp * e[k] + Ki * T * Σe[i] + Kd * (e[k] - e[k-1]) / T
```

但在这个代码里，**Ki和Kd已经包含了采样周期T**，也就是说传入的Ki是 `Ki_raw * T`，Kd是 `Kd_raw / T`。这是因为在嵌入式MCU上，我们假设用户在外层固定周期调用Update函数（典型值1ms~10ms），直接把T揉进系数里，省掉每次运算的乘法。

### 适用场景

- 需要知道"控制量具体是多少"（例如：输出 0~100% 的PWM占空比）
- 被控对象本身没有积分特性（比如电机——电机加电压就转，不加就停，本身就是积分对象，用位置式PID反而容易出问题，但可以用）
- 执行机构接受绝对位置指令（如舵机角度）

---

## 二、逐代码拆解

### 2.1 头文件 — 结构体定义 (`pid_pos.h:1-19`)

```c
#ifndef PID_POS_H
#define PID_POS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float kp;              //比例系数
    float ki;              //积分系数
    float kd;              //微分系数
    float integral;        //积分累计值
    float last_error;      //上次偏差,用于微分计算
    float integral_limit;  //积分限幅,防止积分饱和
    float output_limit;    //输出限幅
    float sep_threshold;   //积分分离阈值,偏差过大时停止积分
    bool  sep_enable;      //积分分离使能
    float deadband;        //死区,偏差小于此值不调节
} PID_Pos_t;
```

#### 逐字段分析

| 字段 | 类型 | 物理意义 | 取值范围(典型) |
|------|------|----------|---------------|
| `kp` | float | 比例增益，放大当前偏差 | 0.1 ~ 100（取决于系统） |
| `ki` | float | 积分增益，放大历史偏差累计 | 0.001 ~ 10（通常很小） |
| `kd` | float | 微分增益，放大偏差变化趋势 | 0.01 ~ 50 |
| `integral` | float | 偏差的累加和（未乘以Ki） | -integral_limit ~ +integral_limit |
| `last_error` | float | 上一周期的 e[k-1] | 同误差范围 |
| `integral_limit` | float | 积分累加上限值 | 视系统而定，通常是输出限幅的0.5~1倍 |
| `output_limit` | float | 总输出上限 | 根据执行机构确定 |
| `sep_threshold` | float | 超过此偏差停止积分 | 典型值为最大允许偏差的60%~80% |
| `sep_enable` | bool | 是否启用积分分离 | true/false |
| `deadband` | float | 死区宽度 | 传感器噪声幅度的 2~3 倍 |

#### 为什么用 float 不用 int？

这个问题"显然"，但必须说清楚：

1. **Ki / Kd 通常小于 1**：例如 Ki = 0.005，如果定义成 int，要么精度全丢，要么放大1000倍再手动缩回来——纯属自找麻烦。
2. **MSPM0G3507 有 FPU**（单精度浮点单元），float 运算速度堪比整数，没有性能损失。
3. **中间计算可能溢出**：e[k] 的范围假设是 ±10000，误差累积 Σe 可以达到几十万，如果乘 Kp=50 再累加，int32_t 随时溢出。float 的 ±1e38 范围完全不用担心。

结论：**有FPU的MCU上，PID参数用float是标准做法，int反而是特例（非常低端的MCU或特殊安全场景）。**

---

### 2.2 初始化函数 (`pid_pos.c:10-23`)

```c
void PID_Pos_Init(PID_Pos_t *pid)
{
    if (pid == NULL) return;
    pid->kp            = 0.0f;
    pid->ki            = 0.0f;
    pid->kd            = 0.0f;
    pid->integral      = 0.0f;
    pid->last_error    = 0.0f;
    pid->integral_limit  = 0.0f;
    pid->output_limit    = 0.0f;
    pid->sep_threshold = 0.0f;
    pid->sep_enable    = false;
    pid->deadband      = 0.0f;
}
```

**解释**：

- 将所有浮点字段清零，布尔字段置 false。
- `integral_limit = 0` 和 `output_limit = 0` 的特殊含义是 **"不限幅"**（见后续Update函数判断逻辑 `> 0.0f` 才限幅）。
- 没有 malloc、没有初始化依赖，纯栈/全局变量均可使用。**调用方必须保证传的指针有效**，否则 NULL 检查保护。

**踩坑点**：Init 之后不能直接 Update，必须 SetParam 设置系数。如果 Kp=0 就 Update，输出永远为0——这 "显然"，但新手经常犯。

---

### 2.3 参数设置函数 (`pid_pos.c:32-52`)

```c
void PID_Pos_SetParam(PID_Pos_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_Pos_SetLimit(PID_Pos_t *pid, float integral_limit,
                      float output_limit)
{
    if (pid == NULL) return;
    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;
}
```

**解释**：

- SetParam 只改系数，不改积分累计值。这意味着 **运行时切换参数不会重置状态**——如果你从一组参数切到另一组，积分和上次误差还在。这有时是好事（平滑过渡），有时是坏事（老的积分在新参数下可能导致振荡）。
- SetLimit 的 `integral_limit` 和 `output_limit` 传入 `<= 0` 表示不限幅。这是一个设计约定，不是标准——为什么这么设计？因为限幅值通常是正数，用 0 或负数表示"不限制"很自然。

**典型配置顺序**：

```
PID_Pos_Init(&pid);
PID_Pos_SetParam(&pid, 1.5f, 0.02f, 0.1f);
PID_Pos_SetLimit(&pid, 100.0f, 200.0f);
PID_Pos_SetSepThreshold(&pid, 50.0f);
PID_Pos_SetDeadband(&pid, 0.5f);
```

---

### 2.4 积分分离配置 (`pid_pos.c:59-69`)

```c
void PID_Pos_SetSepThreshold(PID_Pos_t *pid, float threshold)
{
    if (pid == NULL) return;
    if (threshold > 0.0f) {
        pid->sep_threshold = threshold;
        pid->sep_enable    = true;
    } else {
        pid->sep_threshold = 0.0f;
        pid->sep_enable    = false;
    }
}
```

**解释**：

- 传入正阈值 → 启用积分分离，`sep_enable = true`。
- 传入 <= 0 → 关闭积分分离（即传统PID，不分离）。
- 这里自动管理了 `sep_enable` 字段，调用者不需要手动设置。
- **积分分离的目的**：当误差很大时（比如刚上电目标100，反馈0），积分项快速累加产生严重的超调。停止积分后，比例项单独把系统拉到接近目标，再恢复积分。

---

### 2.5 死区设置 (`pid_pos.c:76-81`)

```c
void PID_Pos_SetDeadband(PID_Pos_t *pid, float deadband)
{
    if (pid == NULL) return;
    if (deadband < 0.0f) deadband = 0.0f;
    pid->deadband = deadband;
}
```

**解释**：

- 死区的意义：当实际值已经在目标附近（误差小于 deadband），认为"已经到位了"，输出 0。如果不设死区，控制器的微小抖动（尤其是微分项对噪声的放大）会让执行机构不停微调，产生"齿槽效应"。
- 负数输入被自动归零（防御性编程）。
- 死区太大会导致稳态误差（永远到不了目标），太小则没有效果。

---

### 2.6 重置函数 (`pid_pos.c:87-92`)

```c
void PID_Pos_Reset(PID_Pos_t *pid)
{
    if (pid == NULL) return;
    pid->integral   = 0.0f;
    pid->last_error = 0.0f;
}
```

**解释**：

- 重置不清除 Kp/Ki/Kd/限幅/死区，因为这些是"配置"，不是"状态"。
- 典型使用场景：目标值发生阶跃变化时（比如从100变到200），如果不Reset，老的积分会拖累新目标下的响应。
- **另一种场景**：系统紧急停止后重新启动，必须 Reset，否则积分累积之前的巨大误差会导致输出瞬间饱和。

---

### 2.7 核心Update函数 (`pid_pos.c:101-137`)

这是整个模块的灵魂，逐行详细拆解。

```c
float PID_Pos_Update(PID_Pos_t *pid, float target, float feedback)
{
    if (pid == NULL) return 0.0f;

    float error = target - feedback;
```

**这一步**：

- 计算当前偏差 `e[k] = target - feedback`
- 注意符号约定：**正误差 = 反馈小于目标，需要正向控制**。如果电机接反，要么换线，要么把 target 和 feedback 对调——绝对不要改代码里的符号，否则读代码的人会疯。

---

```c
    if (pid->deadband > 0.0f && fabsf(error) < pid->deadband) {
        return 0.0f;
    }
```

**解释**：

- 死区判断在前：如果误差在死区内，**直接返回0，跳过所有计算**。
- 这里有一个微妙之处：微分项依赖于 `last_error`，如果直接返回0，`last_error` 不会被更新。这意味着死区期间微分项"冻结"了。这是一种设计选择——另一种做法是更新 `last_error` 但不输出。哪种更好？取决于应用：
  - 当前做法：死区期间微分项不计入，输出停止，执行机构锁定。
  - 替代做法：更新 `last_error = error` 再返回0，这样下次退出死区时微分不会跳变。
- 目前的实现更适合"死区就是不需要任何动作"的场景（如恒温控制）。

---

```c
    if (!pid->sep_enable || fabsf(error) <= pid->sep_threshold) {
        pid->integral += error;
        if (pid->integral_limit > 0.0f) {
            if (pid->integral > pid->integral_limit) {
                pid->integral = pid->integral_limit;
            } else if (pid->integral < -pid->integral_limit) {
                pid->integral = -pid->integral_limit;
            }
        }
    }
```

**这是本模块最复杂的逻辑**，拆开理解：

**条件 `!pid->sep_enable || fabsf(error) <= pid->sep_threshold`**：

- `sep_enable = false`（未启用积分分离）：条件恒成立，每次都积分——传统PI行为。
- `sep_enable = true`：只有当 `|error| <= sep_threshold` 时才积分；如果 `|error| > sep_threshold`，跳过积分。

**积分累加 `pid->integral += error`**：

- 这是 **未乘以 Ki 的原始累加和**。这一点非常重要：结构体里 `integral` 存的是 `Σe[i]`，不是 `Ki * Σe[i]`。
- 为什么这么实现？因为 Ki 可能会在运行时被 SetParam 改变。如果存的是 Ki * Σe，改变 Ki 后积分就会乱。
- 这也是一个常见面试题：**位置式PID的积分累计应该存 Σe 还是 Ki*Σe？** 答案：存 Σe，Ki 在最后计算时乘。

**积分限幅三行**：

```
if (pid->integral > pid->integral_limit)
    pid->integral = pid->integral_limit;
else if (pid->integral < -pid->integral_limit)
    pid->integral = -pid->integral_limit;
```

- 简单粗暴的硬限幅（clamp），没有 anti-windup 的回退算法。
- 当 `integral_limit = 0` 时（Init后的默认值），限幅条件 `> 0.0f` 不成立，所以不限幅——这也是为什么 Init 里要设成 0（表示不限幅）。
- **积分饱和的典型场景**：假设目标100，反馈卡在0（电机堵转），积分持续累加。几秒后积分可能达到几千。此时即使反馈突然恢复正常，积分也需要很长时间才能"退下来"，导致巨大的过冲。限幅就是为了限制这个最大值。

---

```c
    float derivative = error - pid->last_error;
    pid->last_error = error;
```

**解释**：

- 微分项直接用 `e[k] - e[k-1]`，是误差的变化率，不是反馈的变化率。
- 为什么要减 `last_error` 而不是 `feedback`？理论上标准公式用 `e[k] - e[k-1]` 或者 `-(feedback[k] - feedback[k-1])` 等价（因为 e = target - feedback，target 不变时两者相等）。但当 target 变化时，前者会有 **微分冲击**（setpoint kick），target 阶跃变化瞬间微分项产生巨大脉冲。
- 更高级的 PID 会把微分项改为 `-kd * (feedback[k] - feedback[k-1])` 来避免微分冲击。但本模块没有这么做——原因很简单：保持代码简洁，用户自己决定 target 变化频率。如果你需要避免微分冲击，可以把 SetParam 的 kd 设小一点，或者在外部做 target 滤波（如斜坡输入）。

---

```c
    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative;
```

**这就是完整的位置式PID公式**（离散形式）：

```
u[k] = Kp * e[k] + Ki * Σe[i] + Kd * (e[k] - e[k-1])
```

其中：
- `Kp * e[k]`：**比例项**。当前偏差越大，输出越大。
- `Ki * Σe[i]`：**积分项**。消除稳态误差。只要还有一点点正误差，积分就会慢慢累积，直到输出足够大把误差压到 0。
- `Kd * (e[k] - e[k-1])`：**微分项**。预测未来的误差趋势。如果误差正在快速减小（e[k] < e[k-1]），微分项为负，提前减速，减少过冲。

---

```c
    if (pid->output_limit > 0.0f) {
        if (output > pid->output_limit) {
            output = pid->output_limit;
        } else if (output < -pid->output_limit) {
            output = -pid->output_limit;
        }
    }

    return output;
}
```

**输出限幅**：

- 与积分限幅同样的 clamp 模式。
- 输出限幅 **在计算完 PID 之后执行**，不影响积分累加。这意味着：如果输出被限幅了，积分可能还在累加，导致 **积分饱和（integral windup）**。
- 这是位置式 PID 最大的坑：**输出饱和了，积分还在涨**。后续即使误差变小，积分已经很大，需要很长时间才能退回来。
- 改进方案：anti-windup（积分回退/条件积分）——但本模块没做，因为只有一两个 if 的话用户自己可以在外部实现。

---

## 三、完整离散化公式

位置式 PID 的完整公式：

**连续域**：
```
u(t) = Kp * e(t) + Ki * ∫0^t e(τ) dτ + Kd * de(t)/dt
```

**离散化（后向欧拉法，采样周期 T）**：
```
u[k] = Kp * e[k] + Ki * T * Σ(i=0→k) e[i] + (Kd/T) * (e[k] - e[k-1])
```

**本代码中（因为 Ki 和 Kd 参数已经包含了 T 和 1/T）**：
```
u[k] = Kp * e[k] + Ki_coded * Σe[i] + Kd_coded * (e[k] - e[k-1])
```

其中 `Ki_coded = Ki_raw * T`，`Kd_coded = Kd_raw / T`。

**举个数值例子**：

假设：
- T = 10ms = 0.01s
- Kp = 2.0, Ki_raw = 50.0, Kd_raw = 0.1

则传入代码的系数为：
- Kp = 2.0
- Ki = 50.0 * 0.01 = 0.5
- Kd = 0.1 / 0.01 = 10.0

如果 e[k] = 10, Σe = 200, e[k-1] = 8:
```
u[k] = 2*10 + 0.5*200 + 10*(10-8) = 20 + 100 + 20 = 140
```

---

## 四、为什么用位置式PID？

### 位置式 vs 增量式的选择

| 特性 | 位置式 (PID_Pos) | 增量式 (PID_Inc) |
|------|-----------------|-----------------|
| 输出 | 当前周期应输出的**绝对值** | **增量**Δu，需要外部累加 |
| 安全性 | 输出可能突变（积分可能暴涨） | 增量限幅后比较安全 |
| 积分饱和 | 容易发生，需要 extra 处理 | 天然抗积分饱和（Δui = Ki*e, 有限幅） |
| 执行机构失效 | 输出突变 → 机器飞车风险 | 增量限幅 → 相对安全 |
| 调参难度 | 需要同时调3个参数 | Kp/Ki/Kd 含义略有不同 |
| 切换手动/自动 | 需要做"无扰切换"（bumpless） | 天然无扰（增量从0开始） |

**本模块的位置式适用于**：
- 输出直接控制 PWM 占空比的电机驱动器
- 执行机构本身有"记忆"（如舵机角度——给定角度就保持）
- 需要精确知道当前控制输出值（用于监控或日志）

**不适合的情况**：
- 执行机构本身是积分型（如步进电机位置控制——你用位置式输出"位置"，步进电机需要的是"脉冲数"，增量式更自然）
- 安全关键系统（机器人关节，位置式 PID 意外饱和可能导致伤人）

---

## 五、参数整定指导（详细版）

### 5.1 调参顺序

**永远遵循先 P 再 I 再 D 的顺序。**

不遵守这个顺序的结果：
- 先调 I → 系统震荡，你不知道是 I 太大还是 P 太小
- 先调 D → 噪声放大，你以为是系统不稳

### 5.2 Step 1: 调 P（比例增益）

1. 设 Ki = 0, Kd = 0
2. 从很小的 Kp 开始（如 0.1），观察系统响应
3. 逐步增大 Kp（×1.5 ~ ×2），直到系统开始出现**等幅振荡**
4. 记录等幅振荡时的 Kp 值（称为**临界增益 Ku**）和振荡周期 Tu
5. 取 Kp = 0.5 * Ku 作为最终比例增益

**现象判断**：
| 现象 | 原因 |
|------|------|
| 响应太慢，稳态误差大 | Kp 太小 |
| 轻微过冲然后稳定 | Kp 合适 |
| 持续振荡 | Kp 太大（临界 Ku） |
| 剧烈发散振荡 | Kp 过大，立即减小 |

### 5.3 Step 2: 加 I（积分增益）

1. 保持 Kp 为上一步的值
2. 从很小的 Ki 开始（如 Kp 的 1/100 到 1/50）
3. 逐步增大 Ki，观察稳态误差消除速度

**I 的作用**：消除静差。对于有摩擦的系统（如电机带负载）、有重力偏置的系统（如机械臂），必须加 I 才能到达目标。

**判断方法**：
- 稳态误差长期存在但不振荡：Ki 不够大，继续加大
- 低频振荡（周期几秒到几十秒）：Ki 太大，减小
- 超调增大后恢复变慢：Ki 偏大

### 5.4 Step 3: 加 D（微分增益）

1. Kp/Ki 固定
2. 从很小的 Kd 开始（如 Kp 的 1/100）
3. 逐步增大 Kd，观察过冲抑制效果

**D 的副作用**：
- **放大噪声**：传感器每 1LSB 的噪声，乘上 Kd 后直接出现在输出上。如果反馈信号有毛刺，Kd 会把毛刺放大成输出跳变。
- 建议在反馈路径上加一阶低通滤波（LPF），或者在反馈信号采样后用滑动平均。

**判断方法**：
- 过冲减小、稳定时间缩短 → Kd 合适
- 高频抖动（输出快速来回跳动）→ Kd 太大或噪声太大
- 响应变迟钝 → Kd 为负（设错了符号？）

### 5.5 典型值参考（MSPM0G3507 + 直流电机）

| 参数 | 范围（空载） | 范围（带负载） |
|------|------------|-------------|
| Kp | 0.5 ~ 5.0 | 2.0 ~ 20.0 |
| Ki | 0.01 ~ 0.5 | 0.05 ~ 2.0 |
| Kd | 0.05 ~ 2.0 | 0.1 ~ 5.0 |
| integral_limit | 50 ~ 500 | 100 ~ 1000 |
| output_limit | PWM 周期（如 1000） | 同左 |
| sep_threshold | 最大误差的 70% | 同左 |
| deadband | 编码器 1 ~ 3 脉冲 | 同左 |

---

## 六、深度踩坑点

### 6.1 积分饱和 (Integral Windup)

**现象**：系统从停止状态启动，电机堵转或大惯性导致反馈长时间跟不上目标。积分累积到限幅值。当反馈终于追上时，积分仍很大，导致严重过冲，甚至来回震荡数次才稳定。

**本模块的处理**：用 `integral_limit` 截断积分累加。这是最简单的 anti-windup。

**但这样够吗？** 不够。更完善的方案：
1. **条件积分**：输出饱和时停止积分（本模块已经有 `integral_limit`，但输出限幅与积分限幅独立，输出饱和时积分仍在累加）。
2. **积分回退**：当输出饱和时，不仅停止积分，还把积分往回退一点。
3. **外部串联限幅 + 反馈**：限幅器输出值反馈回来，用 `(限幅前 - 限幅后) * Kg` 回退积分。

**对你的建议**：如果遇到积分饱和问题，可以把 `sep_threshold` 也设置上（积分分离），或者把 `integral_limit` 设得比 `output_limit / Ki` 小很多。

### 6.2 微分项放大噪声

微分运算本质上是一个高通滤波器——它会放大高频成分。传感器的量化噪声、电磁干扰都是高频的。

**在本模块中的表现**：
```
derivative = error - pid->last_error;
```
如果 feedback 有 ±1 的随机噪声（编码器抖动），那么 e 也会抖 ±1，derivative 就会在 ±2 范围内跳变，乘以 Kd=10 后输出跳变 ±20。

**解决方案**：
- 在硬件上对反馈信号加 RC 低通滤波
- 在软件上做滑动平均（moving average）
- 或者直接减小 Kd

### 6.3 积分分离的坑

本模块的积分分离逻辑是：
```c
if (!pid->sep_enable || fabsf(error) <= pid->sep_threshold) {
    pid->integral += error;
}
```

**问题**：当 `|error| > sep_threshold` 时，积分停止；当 `|error|` 降到阈值以下时，积分恢复。但此时 `integral` 可能已经是一个很大的值（阈值附近累计的），恢复积分后很快达到限幅。

**改进建议**：积分恢复时做一次清零或衰减——但这属于"智能PID"范围了，本模块作为基础库没做。

### 6.4 死区 + 积分 = 极限环

如果 deadband = 2, Ki > 0：
- 误差在死区（±2）内时输出为 0，但积分还在累加（因为死区判断前就 Return 了，根本没走到积分代码）。
- 等一下——本模块的实现是：**死区内直接 return 0，根本不更新积分和 last_error**。
- 这意味着：如果系统进入死区后又有外力扰动，积分已经"冻结"了，恢复响应时需要重新累加，可能导致来回在死区边界切换（极限环现象）。
- 本模块的规避：死区足够大，或者用户自己协调死区和 Ki。

---

## 七、数据范围与溢出分析

### 7.1 变量范围

| 变量 | 最小 | 最大 | 溢出后果 |
|------|------|------|---------|
| error | -FLT_MAX | +FLT_MAX | 不影响，float 可表示 ±1e38 |
| integral | -integral_limit | +integral_limit | 被限幅，不可能溢出 |
| derivative | -2*error_max | +2*error_max | 安全 |
| output | -output_limit | +output_limit | 被限幅 |

### 7.2 真正的溢出风险

不在 PID 内部，**在外面调用方**：

```c
float pwm = PID_Pos_Update(&pid, target, feedback);
// 如果 pwm 超过定时器的 CCR 值，直接写入会导致 PWM 满占空比或损坏
TIMER->CCR = (uint16_t)pwm;  // 如果 pwm > 65535，类型转换会截断！
```

**正确做法**：
```c
float pwm = PID_Pos_Update(&pid, target, feedback);
if (pwm < 0) pwm = 0;
if (pwm > PWM_PERIOD) pwm = PWM_PERIOD;
TIMER->CCR = (uint16_t)pwm;
```

### 7.3 类型转换风险

结构体中 `float` 赋值给 `int` 做定时器比较值：
- `(uint16_t)(-1.0f)` = 0xFFFF = 65535（负数的 float 到 uint16_t 的转换结果是 UB/MB，某些编译器会给出 0，有些给出 65535）
- 永远先限幅再转换

---

## 八、调用链和上层模块

### 8.1 典型调用层级

```
main() 或 RTOS Task (1ms ~ 10ms 周期)
  └── motor_control_task()
        ├── encoder_read()          → 获取 speed_feedback
        ├── PID_Pos_Update(&pid, speed_target, speed_feedback) → 输出
        └── pwm_set(output)         → 写入定时器
```

### 8.2 串级系统中的位置

`pid_pos` 被 `pid_cascade` 作为 **内环和外环的基础构建块** 使用。

```c
// pid_cascade.h 中的结构体
typedef struct {
    PID_Pos_t outer;   // 外环：位置环
    PID_Pos_t inner;   // 内环：速度环
} PID_Cascade_t;
```

### 8.3 可能的调用者

- **平衡车**：角度环（外环 PID_Pos）→ 速度环（内环 PID_Pos）
- **循迹小车**：位置偏差（外环 PID_Pos）→ 速度差（内环 PID_Pos）
- **四轴无人机**：横滚/俯仰角环（外环 PID_Pos）→ 角速度环（内环 PID_Pos）
- **单电机速度控制**：直接使用 PID_Pos，外循环由 SoC 定时器驱动

---

## 九、调试手段

### 9.1 串口打曲线方法

在 Update 后加上：

```c
float output = PID_Pos_Update(&pid, target, feedback);
// 调试打印
printf("%lu,%f,%f,%f,%f\r\n",
       ticks, target, feedback, error, output);
```

用 `SerialPlot`（推荐）或 `PyQtGraph` 或 `MATLAB` 解析 CSV 数据绘图。

**关键观察点**：
1. **响应上升时间**：从 10% 到 90% 目标值的时间
2. **超调量**：(峰值 - 目标值) / 目标值 × 100%
3. **稳态误差**：稳定后平均值与目标值的差
4. **稳定时间**：进入 ±5% 误差带后不再出去的时间

### 9.2 阶跃响应测试

在 main 函数中写测试代码：

```c
static float test_feedback = 0.0f;
static float test_target = 100.0f;

// 模拟一阶系统响应
void test_system(void) {
    float u = PID_Pos_Update(&pid, test_target, test_feedback);
    test_feedback += 0.1f * (u - test_feedback); // 一阶低通近似被控对象
    printf("%f,%f,%f\r\n", test_target, test_feedback, u);
}
```

在电脑上用 Python 读取串口数据：

```python
import serial, matplotlib.pyplot as plt
ser = serial.Serial('COM3', 115200)
data = [list(map(float, line.split(','))) for line in ser]
target, feedback, output = zip(*data)
plt.plot(target, label='Target')
plt.plot(feedback, label='Feedback')
plt.plot(output, label='Output')
plt.legend(); plt.show()
```

### 9.3 采样周期的影响

**采样周期 T 决定了 PID 的性能边界**：

- **T 太大**（> 50ms）：系统响应慢，微分项失效（变化率在一个周期内已经完成），算法离散化误差大。
- **T 太小**（< 0.1ms）：纯微分项 Kd/T 变得巨大，噪声放大的灾难；积分项 Ki*T 变得极小，消除稳态误差可能需要很长时间。

**经验法则**：
- 采样频率应 >= 10 × 系统闭环带宽
- 对于直流电机速度环：T = 1ms ~ 10ms
- 对于温度控制：T = 100ms ~ 2s
- 对于无人机姿态：T = 1ms ~ 4ms

**不固定的采样周期会怎样？**

如果系统的采样间隔是变化的（比如在多任务 RTOS 中，每次调度的实际间隔不同），那么本模块的 PID 输出会有显著的抖动，因为没有补偿 T 的变化。

**解决方案**：
- 将 T 作为参数传入 Update 函数
- 将 ki 和 kd 的计算改为 `ki * T` 和 `kd / T`
- 本模块没有这样做——为了性能（少做一次除法）和简单。所以你必须保证定时器周期稳定。

---

## 十、完整使用示例

```c
#include "pid_pos.h"

PID_Pos_t motor_speed_pid;

void motor_control_init(void)
{
    PID_Pos_Init(&motor_speed_pid);
    PID_Pos_SetParam(&motor_speed_pid, 2.5f, 0.1f, 0.3f);
    PID_Pos_SetLimit(&motor_speed_pid, 200.0f, 1000.0f);
    PID_Pos_SetSepThreshold(&motor_speed_pid, 300.0f);
    PID_Pos_SetDeadband(&motor_speed_pid, 1.0f);
}

// 在定时器中断中 1ms 调用一次
void motor_control_isr(void)
{
    float speed = encoder_get_speed();       // 获得当前速度
    float output = PID_Pos_Update(&motor_speed_pid, 
                                  500.0f,     // 目标速度
                                  speed);
    pwm_set_duty(output);                     // 设置 PWM
}
```

---

## 十一、与 pid_inc 的选择决策树

```
是否需要知道当前控制量的绝对值？
  ├── 是 → PID_Pos (位置式)
  └── 否 → 是否安全关键系统（如机器人关节）？
            ├── 是 → PID_Inc (增量式更安全)
            └── 否 → 执行机构是否有"记忆"？
                      ├── 是（舵机、阀门）→ PID_Pos
                      └── 否（电机速度）→ 两者都可
```

---

## 十二、总结

`PID_Pos` 实现了一个功能完备的位置式 PID 控制器，包含：
- 积分分离（SepThreshold）
- 死区（Deadband）
- 积分限幅（IntegralLimit）
- 输出限幅（OutputLimit）
- 零硬件依赖，可移植到任何 C 环境

**不足之处**（当前版本没做但你可能会需要）：
1. 无 anti-windup 回退
2. 无微分低通滤波
3. 无采样周期补偿
4. 无前馈项

如果你在当前模块基础上需要这些功能，建议不要直接改这个库，而是在外部封装一层，或者直接看 pid_inc 模块——增量式在某些场景下可以天然避免其中一些问题。
