# 增量式PID控制器 (PID_Inc) — 百科全书级解读

> 阅读前提：你已经理解了位置式PID（pid_pos）的基本原理。增量式是位置式的"变体"，不是替代品，两者有不同的适用场景。

---

## 一、整体定位

**增量式PID** 不直接计算 "当前控制量 u[k] 应该是多少"，而是计算 **"相对于上次的控制量，这次应该增加/减少多少"**：

```
u[k] = u[k-1] + Δu[k]
```

其中 Δu[k] 只与最近三次误差有关：

```
Δu[k] = Kp * (e[k] - e[k-1]) + Ki * e[k] + Kd * (e[k] - 2*e[k-1] + e[k-2])
```

本模块输出的是 **Δu[k]**，不是 u[k]。调用方需要自己维护 u[k] 的累加（或者通过执行机构的积分特性来自动累加——比如步进电机驱动器，每个脉冲就是一次"增量"）。

### 适用场景

- **执行机构本身是积分元件**：步进电机（一个脉冲转一个角度）、伺服电机（速度模式下积分到位置）、位置控制阀门
- **需要无扰切换**：从手动控制切换到自动控制时，增量式天然无扰（Δu 从 0 开始，不会跳变）
- **安全关键系统**：增量限幅保证了每个周期的输出变化是可控的，不会因为积分饱和导致输出瞬间跳到极限
- **通信带宽有限**：输出的是增量，范围较小，可以用更少的字节传输

---

## 二、逐代码拆解

### 2.1 头文件 — 结构体定义 (`pid_inc.h:1-14`)

```c
#ifndef PID_INC_H
#define PID_INC_H

#include <stdint.h>

typedef struct {
    float kp;              //比例系数
    float ki;              //积分系数
    float kd;              //微分系数
    float error[3];        //三次误差历史,k/k-1/k-2
    float output_limit;    //输出限幅
    float integral_limit;  //积分限幅(暂未使用)
} PID_Inc_t;
```

#### 逐字段分析

| 字段 | 类型 | 物理意义 |
|------|------|----------|
| `kp` | float | 比例系数，放大误差变化 (e[k] - e[k-1]) |
| `ki` | float | 积分系数，放大当前误差 e[k] |
| `kd` | float | 微分系数，放大误差加速度 (e[k] - 2e[k-1] + e[k-2]) |
| `error[3]` | float[3] | 环形移位缓冲区，error[0]=当前, [1]=上一次, [2]=上上次 |
| `output_limit` | float | 每周期允许的最大增量绝对值 |
| `integral_limit` | float | 积分项增量的最大绝对值（限幅 Ki * e[k]） |

#### 与位置式结构体的关键差异

1. **没有 `integral`**：位置式存的是 Σe（历史全部误差累加），增量式不需要显式存积分值，因为积分作用隐含在 e[k] 项里。
2. **没有 `last_error` 而用了 `error[3]`**：增量式的微分项需要 e[k-1] 和 e[k-2] 两个历史值来计算二阶差分。
3. **`integral_limit` 标注为 "暂未使用"**：对照 Update 函数，实际上这个字段 **被使用了**（见 2.5），但头文件注释没更新——这是代码与注释不同步的典型例子。
4. **没有 `sep_threshold` 和 `deadband`**：增量式本身对积分饱和有天然免疫力，所以一般不需要积分分离。死区可以在外部做。

---

### 2.2 初始化函数 (`pid_inc.c:9-20`)

```c
void PID_Inc_Init(PID_Inc_t *pid)
{
    if (pid == NULL) return;
    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->output_limit   = 0.0f;
    pid->integral_limit = 0.0f;
}
```

**解释**：

- 所有系数、误差、限幅全部归零。
- Init 之后不能直接 Update——理由同位置式（Kp=0 输出永远是 0）。
- `error[0]/[1]/[2]` 全部清零，意味着启动时前两个周期的微分项会偏小（因为 e[-1] 和 e[-2] 都是 0），这是合理的初始化行为。

---

### 2.3 参数设置函数 (`pid_inc.c:29-49`)

```c
void PID_Inc_SetParam(PID_Inc_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_Inc_SetLimit(PID_Inc_t *pid, float output_limit,
                      float integral_limit)
{
    if (pid == NULL) return;
    pid->output_limit   = output_limit;
    pid->integral_limit = integral_limit;
}
```

**解释**：

- `SetParam` 与位置式完全相同——只改系数，不清零误差。
- `SetLimit` 接收 `output_limit` 和 `integral_limit`。前者限幅 Δu（每周期变化量），后者限幅 Ki*e[k]（积分贡献的增量部分）。
- 限幅值 `<= 0` 都表示不限幅。

---

### 2.4 重置函数 (`pid_inc.c:55-61`)

```c
void PID_Inc_Reset(PID_Inc_t *pid)
{
    if (pid == NULL) return;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
}
```

**解释**：

- 只清误差历史，不清除系数和限幅。
- 重置后的前 2 个 Update 调用，微分项计算不全（因为 e[-1], e[-2] 为 0），这是正常的——你不希望在 Reset 之后第一个周期就输出一个巨大的 Δu。

---

### 2.5 核心 Update 函数 (`pid_inc.c:70-101`)

```c
float PID_Inc_Update(PID_Inc_t *pid, float target, float feedback)
{
    if (pid == NULL) return 0.0f;

    pid->error[2] = pid->error[1];    //历史误差移位,error[0]为最新
    pid->error[1] = pid->error[0];
    pid->error[0] = target - feedback;
```

**误差移位**：

- 经典的环形缓冲区实现，每次 Update 时移位：`e[k-2] ← e[k-1] ← e[k] ← 新误差`。
- `error[0]` 总是最新误差 e[k]，`error[1]` 是 e[k-1]，`error[2]` 是 e[k-2]。
- 这种实现比位置式的单一 `last_error` 多一份历史，是为了后续的二阶差分计算。

---

```c
    float delta_p = pid->kp * (pid->error[0] - pid->error[1]);  //比例增量
    float delta_i = pid->ki * pid->error[0];                     //积分增量
    float delta_d = pid->kd * (pid->error[0] - 2.0f * pid->error[1]
                                + pid->error[2]);                 //微分增量
```

**这是增量式 PID 的核心公式**。

**比例增量 `ΔP = Kp * (e[k] - e[k-1])`**：

- 比例作用取决于误差的变化量。如果误差稳定（e[k] = e[k-1]），比例增量为 0。
- 对比位置式：位置式的比例项是 `Kp * e[k]`——只要还有误差就有输出。增量式的比例项则只"响应变化"。

**积分增量 `ΔI = Ki * e[k]`**：

- 这就是为什么说 "增量式隐含有积分"——因为 ΔI 与 e[k] 成正比，外部累加后就是 `Ki * Σe`。
- 对比位置式：位置式单独维护一个 integral 累加器。增量式每次的 e[k] 被加进去，天然形成了积分效果。
- **关键区别**：位置式的积分是"永久记忆"（除非 Reset），增量式是"有限记忆"（每次的 e[k] 会逐渐被新的 error 替代——不对！仔细想：增量式的积分是通过外部累加实现的，每次的 ΔI 被加到 u[k] 后就不会消失了。所以本质上和位置式的积分是一样的，只是计算方式不同）。

**微分增量 `ΔD = Kd * (e[k] - 2*e[k-1] + e[k-2])`**：

- 这是一个 **二阶差分**，相当于误差的"加速度"。
- 推导：位置式的微分项是 `Kd * (e[k] - e[k-1])`。增量式对位置微分项再做一次差分：
  ```
  ΔD = Kd * [(e[k] - e[k-1]) - (e[k-1] - e[k-2])]
     = Kd * (e[k] - 2*e[k-1] + e[k-2])
  ```
- 所以增量式的微分项实际上是 **误差的二阶差分**，比位置式的"变化率"更进了一阶。这带来了更强的预测能力，但也放大了更多噪声。

---

```c
    if (pid->integral_limit > 0.0f) {
        if (delta_i > pid->integral_limit) {
            delta_i = pid->integral_limit;
        } else if (delta_i < -pid->integral_limit) {
            delta_i = -pid->integral_limit;
        }
    }
```

**解释**：

- 头文件注释说 `integral_limit` 暂未使用（`//暂未使用`），但代码里实际实现了。要么是注释没更新，要么是早期版本没用后面加上了。
- 限幅对象是 `delta_i = Ki * e[k]`（单周期的积分增量），不是累积积分值。
- 这比位置式的积分限幅更"温柔"——位置式限幅的是历史累加和，而这里只限制每个周期新加的积分量。

---

```c
    float output = delta_p + delta_i + delta_d;  //总控制增量

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

- 这里限幅的是 **Δu（增量）**，不是 u（绝对值）。这意味着 **单个周期的输出变化不会超过 output_limit**。
- 这是一个非常重要的安全特性：即使误差突然变得很大，Δu 也会被限幅在安全范围内，系统只会"慢慢"调整，不会飞车。
- 但这也意味着：对于大阶跃响应，增量式 PID 需要多个周期才能到达目标值——**响应速度慢于位置式**。

---

## 三、完整的数学推导

### 3.1 从位置式到增量式

位置式 PID 离散形式：
```
u[k] = Kp * e[k] + Ki * Σe[i] + Kd * (e[k] - e[k-1])
```

上一周期的位置式：
```
u[k-1] = Kp * e[k-1] + Ki * Σe[i-1] + Kd * (e[k-1] - e[k-2])
```

两式相减：
```
Δu[k] = u[k] - u[k-1]
      = Kp*(e[k]-e[k-1]) + Ki*(Σe[i]-Σe[i-1]) + Kd*(e[k]-2e[k-1]+e[k-2])
      = Kp*(e[k]-e[k-1]) + Ki*e[k] + Kd*(e[k]-2e[k-1]+e[k-2])
```

**最终公式**：
```
Δu[k] = Kp * Δe[k] + Ki * e[k] + Kd * Δ²e[k]
```

其中：
- `Δe[k] = e[k] - e[k-1]` — 误差的一阶差分
- `Δ²e[k] = e[k] - 2*e[k-1] + e[k-2]` — 误差的二阶差分

### 3.2 与位置式对比

```
位置式: u[k] = Kp*e[k] + Ki*Σe[i] + Kd*(e[k]-e[k-1])
增量式: Δu[k] = Kp*(e[k]-e[k-1]) + Ki*e[k] + Kd*(e[k]-2e[k-1]+e[k-2])
```

两者在数学上等价（如果位置式的积分从0开始），但在工程上有本质差异：
- 位置式输出 **绝对值**，一个周期就能计算到位
- 增量式输出 **相对变化量**，需要调用方累加多次才能到达目标

### 3.3 数值示例

假设 Kp=2, Ki=0.5, Kd=1，误差序列：

| k | e[k] | ΔP | ΔI | ΔD | Δu | u |
|---|------|----|----|----|----|---|
| 0 | 100 | — | — | — | — | 0 (初始) |
| 1 | 100 | 0 | 50 | — | 50 | 50 |
| 2 | 80 | -40 | 40 | 20 | 20 | 70 |
| 3 | 50 | -60 | 25 | 10 | -25 | 45 |
| 4 | 20 | -60 | 10 | -5 | -55 | -10 |

注意：k=4 时 u 变成 -10，说明输出反向。这个行为在某些系统中可能是不希望的——正反转切换。

---

## 四、为什么用增量式PID？

### 4.1 安全性优势（最重要考量）

位置式 PID 在以下情况可能产生灾难性输出：
1. **积分饱和**：积分累加到极限，输出值很大
2. **突然恢复**：反馈突然跟上，输出仍然很大
3. **执行机构动作**：从 0 到满量程的阶跃

增量式的保障：
- `output_limit` 限制每个周期的最大变化量
- 即使积分很大，每周期也只加一点点
- 系统有时间反应

### 4.2 无扰切换（Bumpless Transfer）

从手动控制切换到自动控制时：
- 位置式：必须先把 PID 的输出"追上"当前手动设定的值（否则会跳变），需要做复杂的无扰切换逻辑
- 增量式：Δu 从 0 开始累加，输出从当前手动值开始平滑变化

### 4.3 执行机构适配

对于以下设备，增量式更自然：
- **步进电机**：每个脉冲对应一个 Δu（电机驱动器接受脉冲数）
- **带积分特性的阀门**：开大一点/关小一点
- **电子油门**：加速踏板增加/减少角度

### 4.4 什么时候增量式不好

1. **输出需要精确绝对值**：比如舵机角度——你必须告诉它"转到 90°"，而不是"再转 1°"。增量式需要外部维护累加值，掉电丢失。
2. **执行机构没有积分特性**：比如电加热——给定 0V 就不加热，给定 5V 就满功率。如果增量式输出 10 就加热 10，停止后冷却。增量式的累加值在这里没有物理意义。
3. **需要快速响应大阶跃**：增量式的限幅会拖慢响应速度，位置式可以直接跳到目标值。

---

## 五、参数整定指导

### 5.1 增量式参数的"含义"变化

同样叫 Kp/Ki/Kd，在增量式中的单位与位置式不同：

| 参数 | 位置式 | 增量式 |
|------|--------|--------|
| Kp | 输出/误差 | 输出变化量/误差变化量 |
| Ki | 输出/(误差×时间) | 输出变化量/误差 |
| Kd | 输出/(误差变化速率) | 输出变化量/误差加速度 |

由于这些不同的量纲，**增量式的参数通常比位置式小很多**。

### 5.2 调参步骤

**Step 1: 确定 output_limit**

这取决于你对系统的"安全性要求"。对于直流电机速度环：
```
output_limit = PWM_MAX_VALUE * 0.01 ~ 0.05
```
即每个周期允许电机改变 1%~5% 的占空比。这样即使误差很大，电机也需要数十个周期的过渡时间。

**Step 2: 调 Kp（比例增量）**

1. Ki = 0, Kd = 0
2. 从极小的 Kp 开始（如 0.01）
3. 观察 Δu 的变化量
4. 增大 Kp 直到系统对误差变化敏感但不振荡

**经验法则**：增量式的 Kp 通常是位置式的 1/10 ~ 1/100。原因是 ΔP = Kp * (e[k] - e[k-1])。如果 e 变化 1，Kp=2，ΔP=2，累加 50 次才到 100。位置式 Kp=2 就直接输出 2*100=200。

**Step 3: 调 Ki（"积分"增量）**

1. 保持 Kp
2. 从 Ki = Kp * 0.1 开始
3. 观察稳态误差消除

**增量式 Ki 的注意事项**：
- Ki 项就是 ΔI = Ki * e[k]，它直接与当前误差成正比。
- Ki 越大，每次累加的积分越多，稳态误差消除越快，但也越容易振荡。

**Step 4: 调 Kd（微分增量）**

1. 保持 Kp/Ki
2. 从 Kd = Kp * 0.01 开始
3. 增量式的 Kd 控制的是 ΔD = Kd * (e[k] - 2e[k-1] + e[k-2])

**这是最敏感的参数**。二阶差分对噪声的放大是位置式（一阶差分）的 2 倍以上。如果反馈有噪声，Kd 稍微大一点输出就会剧烈抖动。

### 5.3 典型值参考（MSPM0G3507 + 直流电机速度环，采样 1ms）

| 参数 | 空载范围 | 带负载范围 |
|------|---------|-----------|
| Kp | 0.05 ~ 0.5 | 0.2 ~ 2.0 |
| Ki | 0.01 ~ 0.1 | 0.05 ~ 0.5 |
| Kd | 0.001 ~ 0.05 | 0.005 ~ 0.2 |
| output_limit | PWM_MAX × 0.02 | PWM_MAX × 0.05 |
| integral_limit | output_limit × 0.5 | output_limit |

---

## 六、深度踩坑点

### 6.1 增量式没有"真积分"的限制

在位置式中，我们可以单独限制积分累加值。在增量式中，`integral` 隐式存在于外部累加的 u 值里，**无法直接限制**。

假设：
- 外部 u 已经累加到 1000（PWM 满量程）
- 此时 error 变为 0，但 ΔI = Ki * 0 = 0，ΔP = 0，ΔD = 0
- u 保持 1000，不会自动下降

这意味着增量式 PID 的 `u` 没有自动衰减机制。如果需要 u 在误差消除后归零，必须：
1. 在外部限幅 u（如限制 0~1000）
2. 或者加上积分衰减（如每次 Update 后 u *= 0.999）

### 6.2 误差移位导致的"启动瞬态"

Reset 后第一个周期：
```
error[2]=0, error[1]=0, error[0]=e
ΔP = Kp * (e - 0) = Kp*e       ← 正确（因为没有历史，假设 e[-1]=0）
ΔI = Ki * e                     ← 正确
ΔD = Kd * (e - 0 + 0) = Kd*e   ← 错误！启动时的二阶差分被放大
```

解决方案：Reset 后不要立即输出 Δu，可以跳过前 2 个周期的微分项，或者手动初始化 error[1]=error[0], error[2]=error[0]。

### 6.3 累计误差（漂移）

增量式的核心缺陷：**u 的误差会累积**。

假设外部累加代码：
```c
static float u = 0;
u += PID_Inc_Update(&pid, target, feedback);
```

如果每次 Update 有 0.1% 的量化误差（float 舍入），10,000 次后误差累积达到 `0.001 * 10000 = 10`。在某些系统中这可能无法接受（如精密定位）。

**解决方案**：
- 定期重置 u 到已知值
- 或者在累加时使用 double（但 MSPM0G3507 是单精度 FPU，double 用软件模拟，很慢）

### 6.4 ΔD 的噪声放大效应

位置式微分：`Kd * (e[k] - e[k-1])` — 噪声放大 1 倍

增量式微分：`Kd * (e[k] - 2*e[k-1] + e[k-2])` — 噪声放大 √(1²+2²+1²) = √6 ≈ 2.45 倍

这就是为什么增量式的 Kd 通常比位置式的小一个数量级。

### 6.5 积分限幅被标注为"暂未使用"

头文件注释说 `integral_limit` 暂未使用，但代码中确实使用了。这可能导致两种情况：
1. 你的同事看头文件时以为积分不限幅，传了很大的 Ki
2. 代码维护者可能在未来某个版本删掉这个字段，导致积分失去限幅

**最佳实践**：依赖行为而不是注释。既然代码实现了，就当作它是有用的。如果发现注释与代码不符，更新注释。

---

## 七、数据范围与溢出分析

### 7.1 变量范围

| 变量 | 最小 | 最大 | 说明 |
|------|------|------|------|
| error[3] 各项 | -FLT_MAX | +FLT_MAX | 合理值 ±1e5 |
| delta_p | -Kp * 2*err_max | +Kp * 2*err_max | 限幅前 |
| delta_i | -Ki * err_max | +Ki * err_max | 被 integral_limit 限幅 |
| delta_d | -Kd * 4*err_max | +Kd * 4*err_max | 二阶差分的最大可能范围 ±4*err_max |
| output | -output_limit | +output_limit | 限幅后 |

### 7.2 外部累加器的溢出

调用方需要关注的是 **u 的累加值**。

```c
float u = 0;  // 全局或静态变量

void timer_isr(void) {
    u += PID_Inc_Update(&pid, target, feedback);
    if (u > MAX_PWM) u = MAX_PWM;
    if (u < 0) u = 0;
    PWM->CCR = (uint16_t)u;
}
```

**问题**：如果系统长时间运行，u 可能因为各种原因（误差未完全消除、量化误差累积）逐渐偏离合理范围。理论上限幅代码可以解决，但如果限幅经常触发，说明系统有静差。

### 7.3 float 精度问题

float 的精度是 ~7 位十进制有效数字。对于 `u += Δu`，如果 u 已经很大（如 10000），而 Δu 很小（如 0.001），则 `u + 0.001` 由于精度限制可能等于 `u`。

```
10000.0f + 0.001f = 10000.0f  // 因为 float 无法表示 10000.001
```

这意味着当 u 很大时，微小的 Δu 会丢失。解决方案：使用定标整数（Q15、Q31）或者定期将 u 缩放到一个更小的范围。

---

## 八、调用链和上层模块

### 8.1 典型调用模式

增量式 PID 的调用方必须自己维护 **累加器和限幅**：

```c
static float current_output = 0.0f;

void control_task_1ms(void)
{
    float speed = encoder_read();
    float delta = PID_Inc_Update(&pid, target_speed, speed);
    
    current_output += delta;
    
    // 输出限幅必须由调用方实现
    if (current_output > PWM_MAX) current_output = PWM_MAX;
    if (current_output < 0) current_output = 0;
    
    pwm_set(current_output);
}
```

### 8.2 上层调用场景

1. **直流电机速度环**：`main_control.c` 的定时器中断调用
2. **舵机位置缓动**：PID_Inc 输出增量，累加成舵机角度
3. **无人机角速度环**：作为级联PID的内环（虽然实际上参数用的是 PID_Pos，但可以替换为 PID_Inc）

### 8.3 如何替换位置式为增量式

如果你有一块用 PID_Pos 跑的位置环代码：

```c
// 原来是位置式
float u = PID_Pos_Update(&pos_pid, target, feedback);
pwm_set(u);

// 改成增量式
static float u = 0;
u += PID_Inc_Update(&inc_pid, target, feedback);
pwm_set(u);
```

但注意参数必须重新整定——增量式的 Kp/Ki/Kd 与位置式的量纲不同，直接套用会出问题。

---

## 九、调试手段

### 9.1 Debug 打印内容

对于增量式，建议打印以下四组数据：

```c
float delta = PID_Inc_Update(&pid, target, feedback);
static float output = 0;
output += delta;

printf("%lu,%f,%f,%f,%f\r\n",
       tick, target, feedback, delta, output);
```

- `target` 和 `feedback`：判断系统状态
- `delta`（Δu）：每个周期的变化量（直接反映 PID 输出意图）
- `output`（u）：累加后的控制量（实际写入硬件的值）

### 9.2 特殊诊断：观察 Δu 的组成

如果要调试每个分量的贡献，需要修改 PID_Inc_Update 函数（或者复制一份并增加 debug 输出）：

```c
float PID_Inc_Update_Debug(PID_Inc_t *pid, float target, float feedback,
                           float *out_dp, float *out_di, float *out_dd)
{
    // ... 正常计算 ...
    *out_dp = delta_p;
    *out_di = delta_i;
    *out_dd = delta_d;
    return output;
}
```

通过观察三个分量的比例，可以直观地看到：
- 比例增量是否主导了响应
- 积分增量是否过大
- 微分增量是否在噪声淹没下

### 9.3 采样周期的影响

与位置式一样，增量式也假设固定的采样周期。但增量式对采样周期变化更敏感，因为：
- 二阶差分 `e[k] - 2e[k-1] + e[k-2]` 对时间间隔变化非常敏感
- 如果采样周期不稳定，二阶差分值会严重失真

**验证方法**：在调试时记录两次调用之间的实际间隔：
```c
static uint32_t last_time = 0;
uint32_t now = get_tick_us();
uint32_t dt = now - last_time;
last_time = now;
if (abs((int32_t)(dt - EXPECTED_DT)) > TOLERANCE) {
    // 采样间隔异常，此时 PID 输出可能不可靠
}
```

---

## 十、完整使用示例

```c
#include "pid_inc.h"

PID_Inc_t motor_pid;
static float motor_output = 0.0f;

void motor_init(void)
{
    PID_Inc_Init(&motor_pid);
    PID_Inc_SetParam(&motor_pid, 0.2f, 0.05f, 0.01f);
    PID_Inc_SetLimit(&motor_pid, 50.0f, 30.0f);
    
    motor_output = 0.0f;
}

void motor_control_1ms(void)
{
    float speed = read_encoder_speed();
    
    // 目标速度 1000 RPM
    float delta = PID_Inc_Update(&motor_pid, 1000.0f, speed);
    
    // 累加 + 限幅
    motor_output += delta;
    if (motor_output > 5000.0f) motor_output = 5000.0f;
    if (motor_output < 0.0f) motor_output = 0.0f;
    
    // 写入 PWM (12位定时器)
    TIMER0->CCR0 = (uint16_t)(motor_output);
}

// 紧急停止函数
void motor_emergency_stop(void)
{
    PID_Inc_Reset(&motor_pid);
    motor_output = 0.0f;
    TIMER0->CCR0 = 0;
}
```

---

## 十一、位置式 vs 增量式的核心区别总结

```
适用场景:
  位置式: 输出绝对值, 执行机构无积分
  增量式: 输出变化量, 执行机构有积分/需要安全性

安全性:
  位置式: 低 (输出可能突变)
  增量式: 高 (变化速率可控)

参数整定难度:
  位置式: 中等 (直观)
  增量式: 较难 (量纲变化, 参数更小)

微分项:
  位置式: 一阶差分 (噪声放大 1x)
  增量式: 二阶差分 (噪声放大 ~2.45x)
```

---

## 十二、总结

`PID_Inc` 增量式 PID 模块用 100 行 C 代码实现了一个 **更安全、更可控** 的 PID 控制器版本。它不适合所有场景，但在需要安全优先、平滑控制、避免积分饱和的系统中是更好的选择。

**关键特性回顾**：
- 输出为 Δu（增量），而非 u（绝对值）
- 通过 output_limit 限制每周期最大变化率
- 天然抗积分饱和（每个分量的增量独立限幅）
- 无扰切换支持
- 二阶差分微分项（更高预测能力，也更多噪声）

**如果你遇到的问题**：
- "系统响应太慢"：增大 output_limit 和 Kp
- "输出一直在抖动"：减小 Kd，检查传感器噪声
- "稳态误差消不掉"：增大 Ki（注意 Ki 太大导致低频振荡）
- "启动时突然跳一下"：Reset 后前两个周期跳过微分项
