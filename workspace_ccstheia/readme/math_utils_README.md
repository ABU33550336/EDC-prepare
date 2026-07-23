# math_utils — 数学工具模块百科全书

> **README 定位**: 当你看代码旁边的注释都看不懂时来翻阅的百科全书。
> **目标平台**: MSPM0G3507 @ 32MHz, 可移植到任何 C 环境
> **核心思想**: 纯软件数学工具, 零硬件依赖, 为嵌入式控制提供基础运算构件

---

## 目录

1. [模块概览](#1-模块概览)
2. [MATH_Limit / MATH_LimitInt — 限幅函数](#2-math_limit--math_limitint--限幅函数)
3. [MATH_Map — 线性映射(缩放)函数](#3-math_map--线性映射缩放函数)
4. [MATH_Deadzone — 死区处理函数](#4-math_deadzone--死区处理函数)
5. [MATH_Sign / MATH_SignInt — 符号判断函数](#5-math_sign--math_signint--符号判断函数)
6. [MATH_FloatEq — 浮点容差比较函数](#6-math_floateq--浮点容差比较函数)
7. [MATH_MAX / MATH_MIN / MATH_ABS / MATH_CLAMP — 宏定义](#7-math_max--math_min--math_abs--math_clamp--宏定义)
8. [模块间调用关系](#8-模块间调用关系)
9. [调试与验证](#9-调试与验证)
10. [常见踩坑点](#10-常见踩坑点)

---

## 1. 模块概览

```c
//数学工具函数,限幅/映射/死区/符号/浮点比较
#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <stdint.h>
#include <stdbool.h>
```

**这段代码在干什么**: 头文件保护 + 类型头文件包含。

**解释**:
- `#ifndef MATH_UTILS_H` / `#define MATH_UTILS_H` / `#endif` 是 C 标准头文件保护宏, 防止同一个头文件被 `#include` 两次导致重复定义。
- `stdint.h` 提供了 `int32_t`、`uint8_t`、`uint16_t` 等定长类型。在嵌入式领域, `int` 在不同编译器下可能是 16 位或 32 位, 使用定长类型确保跨平台行为一致。
- `stdbool.h` 提供了 `bool`、`true`、`false`。C99 以前 C 语言没有 bool 类型, 需要自己 `typedef` 或定义宏; C99 标准加入 `stdbool.h` 后可以统一使用。

**为什么不省略头文件保护**: 没有头文件保护, 当 A.h 包含 math_utils.h, B.c 同时包含 A.h 和 math_utils.h 时, 结构体和函数声明会被定义两次, 编译报重复定义错误。

---

## 2. MATH_Limit / MATH_LimitInt — 限幅函数

### 2.1 函数声明

```c
float MATH_Limit(float value, float min, float max);
int32_t MATH_LimitInt(int32_t value, int32_t min, int32_t max);
```

**这段代码在干什么**: 声明两个限幅函数, 分别处理浮点和 32 位整数。

**解释**:
- `MATH_Limit` 接受 float, 返回 float。用于连续控制量, 比如 PID 输出、PWM 占空比、速度指令等。
- `MATH_LimitInt` 接受 int32_t, 返回 int32_t。用于离散控制量, 比如编码器计数值限幅、定时器比较值限幅。
- 返回类型与输入 value 类型一致, 避免隐式类型转换带来的精度损失或符号问题。

**应用场景举例**:
- PID 输出限幅: PID 计算出的控制量可能远超过执行器承受范围。比如你用 PWM 驱动电机, PWM 占空比只能 0%~100%, 但积分项累积可能让 PID 输出达到 10000%。`MATH_Limit(pid_out, 0.0f, 100.0f)` 一刀砍下去。
- ADC 值校验: 12 位 ADC 理论上只能输出 0~4095, 但如果硬件故障或噪声干扰, 读到的值可能越界。用 `MATH_LimitInt(adc_val, 0, 4095)` 保证下游处理不会出妖。
- 角度限幅: 舵机只能转 0~180°, 如果你计算出 -30°, 必须限幅到 0°。

### 2.2 函数实现

```c
float MATH_Limit(float value, float min, float max)
{
    if (min > max) {
        float tmp = min;
        min = max;
        max = tmp;
    }
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
```

**这段代码在干什么**: 如果 min > max 则交换; 然后将 value 钳位到 [min, max]。

**逐行解释**:
- `if (min > max)`: 参数检查——调用者可能传反了上下限。比如想限 0~100 但写成 `MATH_Limit(x, 100, 0)`。如果没有这个交换, 所有值都无法通过, 函数永远返回 0 (因为 max=0, min=100, value < min 永远成立)。
- `float tmp = min; min = max; max = tmp;`: 经典三变量交换, 不使用临时变量的 XOR 交换技巧在这里不可用, 因为 float 不能做位运算。也不适合用宏 SWAP, 因为宏可能对浮点有精度问题。
- `if (value < min) return min;`: 下限判断。注意这里用 `<` 而非 `<=`, 当 value == min 时直接返回 value, 少一次比较。这属于微优化, MSPM0G3507 @32MHz 上能省则省。
- `if (value > max) return max;`: 上限判断。同样用 `>` 而非 `>=`。
- `return value;`: 在范围内, 原样返回。

**为什么用两个 if 而不用 fmaxf/fminf**:
- `float clamped = fmaxf(fminf(value, max), min);` 虽然更短, 但:
  1. `fmaxf` / `fminf` 是数学库函数, 调用有开销 (函数调用 + 内部分支)
  2. 需要链接 libm (`-lm`), 在裸机环境可能没有数学库或不想引入
  3. `fmaxf` / `fminf` 对 NaN 的处理与普通比较不同 (NaN 会被跳过), 可能导致意外行为
  4. 两个 if 方案编译器能优化成条件移动指令 (如 ARM 的 CSEL), 性能极佳

**踩坑点**: IEEE 754 NaN 传入 value。NaN 与任何数比较都是 false。所以如果 value 是 NaN:
- `value < min` → false
- `value > max` → false
- 直接 `return value;` → 返回 NaN
- 调用者如果没做 NaN 检查, NaN 会在后续计算中传播。建议调用前确保 value 不是 NaN。或者要更健壮可以加 `isnan(value)` 检查, 但 isfinite/isnan 需要 math.h。

```c
int32_t MATH_LimitInt(int32_t value, int32_t min, int32_t max)
{
    if (min > max) {
        int32_t tmp = min;
        min = max;
        max = tmp;
    }
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
```

**这段代码在干什么**: 整数版限幅, 逻辑与浮点版完全一致。

**差异分析**:
- 参数类型 `int32_t` 而非 `int`。`int` 在 MSPM0 的 TI ARM 编译器上是 32 位, 但在某些 8 位 MCU (如 8051) 上是 16 位。显式 `int32_t` 确保跨平台移植时行为不变。
- 不会有 NaN 问题, 整数永远是合法值。
- 整数溢出问题已经由调用者负责: 如果 value 是 `int32_t` 的最大值 0x7FFFFFFF 加 1, 已经 undefined behavior 了, `MATH_LimitInt` 处理不了。

**为什么要有两个函数而不是用宏或模板**: C 语言没有函数模板, 也没有重载。用宏 `#define LIMIT(val, min, max)` 会有多遍求值问题: `LIMIT(adc_read(), 0, 100)` 会把 `adc_read()` 展开多次, 每次调用返回值可能不同。所以老老实实写两个函数。

---

## 3. MATH_Map — 线性映射(缩放)函数

### 3.1 函数声明

```c
float MATH_Map(float value, float in_min, float in_max,
               float out_min, float out_max);
```

**这段代码在干什么**: 将 value 从输入区间 [in_min, in_max] 线性映射到输出区间 [out_min, out_max]。

**解释**:
- 数学公式: `output = out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)`
- 输入可以是任意范围, 不要求 in_min <= in_max (但 in_min == in_max 时会触发除零保护)
- 映射不自动限幅: 如果 value < in_min, 输出会小于 out_min (外推); 如果 value > in_max, 输出会大于 out_max (外推)。需要调外部限幅。

**应用场景举例**:
- ADC 值映射到物理量: 12 位 ADC 读取 NTC 分压, 0~4095 映射到 0~100 °C。`MATH_Map(adc_val, 0, 4095, 0.0f, 100.0f)`。
- 传感器校准: 压力传感器输出 0.5V~4.5V, 对应 0~10MPa。`MATH_Map(sensor_voltage, 0.5f, 4.5f, 0.0f, 10.0f)`。
- 控制量变换: PID 输出 -100~100, 需要映射到 PWM 比较值 500~2500。`MATH_Map(pid_out, -100.0f, 100.0f, 500.0f, 2500.0f)`。

### 3.2 函数实现

```c
float MATH_Map(float value, float in_min, float in_max,
               float out_min, float out_max)
{
    float in_range = in_max - in_min;
    if (fabsf(in_range) < 1e-12f) {
        return out_min;
    }
    float ratio = (value - in_min) / in_range;
    return out_min + ratio * (out_max - out_min);
}
```

**逐行解释**:
- `float in_range = in_max - in_min;`: 计算输入范围的宽度。如果 in_min 和 in_max 非常接近, in_range 接近 0, 会导致后面除法溢出。
- `if (fabsf(in_range) < 1e-12f)`: 除零保护。为什么是 1e-12? float 有效位数约 7 位十进制, 对于典型值 (如 0~4095 或 0~100), in_range 的绝对值远大于 1e-12。这个阈值选得比较宽松, 目的是捕捉 in_min ≈ in_max 的情况。如果阈值设太小 (如 1e-30), 可能因为浮点精度问题导致 in_range 本应为 0 但实际为 1e-28, 除零保护失效。
- `return out_min;`: 输入范围过小时, 认为输入始终在这个"点"上, 直接返回输出下限。也可以返回中点 `(out_min + out_max) / 2.0f`, 但原代码选择返回下限, 调用者需要注意。
- `float ratio = (value - in_min) / in_range;`: 计算 value 在输入区间中的归一化位置, 0.0~1.0。如果 value 在区间外, ratio 可能为负或大于 1。
- `return out_min + ratio * (out_max - out_min);`: 将归一化比例缩放到输出区间。

**精度误差分析**:
- 如果 in_range 很大 (如 100000.0f), `value - in_min` 和 `in_range` 数量级接近, 除法精度好。
- 如果 in_range 很小 (如 0.001f), 浮点精度可能不够。比如 value = 0.0005, in_min = 0.0, in_max = 0.001, 理论上 ratio = 0.5, 但 float 可能算出 0.5000001 或 0.4999999, 导致最终结果有 ±1e-6 量级误差。
- 如果 out_range 也很大 (如 1000000.0f), 这个 tiny 误差会被放大。所以对于高精度要求的场景 (如电机位置控制), 考虑改用 double 或定点数。

**为什么不外推限幅**:
- 原版 MATH_Map 不做限幅。如果你需要限幅, 在调用后自行 `MATH_Limit(result, out_min, out_max)`。这样将映射和限幅解耦, 灵活性更高。
- 一些场景需要外推: 比如温度传感器在校准范围外也需要输出估计值, 即使精度下降。
- 一些场景不需要外推: 比如 ADC 绝对不可能超出范围 (硬件绑定), 外推反而是错误。

---

## 4. MATH_Deadzone — 死区处理函数

### 4.1 函数声明

```c
float MATH_Deadzone(float value, float threshold);
```

**这段代码在干什么**: 如果 |value| < threshold, 输出 0; 否则输出原值。

**解释**:
- 死区的数学定义: f(x) = 0 if |x| < T, else f(x) = x
- 与限幅的区别: 限幅是砍掉超出范围的两端; 死区是砍掉范围之内的中间段。

**应用场景举例**:
- 遥控器摇杆回中: 摇杆物理上不可能完全回中, 总会有几 mV 的零漂。如果不做死区, 电机在摇杆回中后还会微微转动。设 threshold = 0.05 (对应 5% 摇杆行程), 小于 5% 的指令认作 0。
- 编码器零速滤波: 电机静止时编码器可能因为震动跳 ±1 个脉冲。用死区滤掉这些小脉冲, 避免速度环积分项累积。
- 温度控制滞环: 类似施密特触发器, 但这里只是简单的死区, 不是滞环。

### 4.2 函数实现

```c
float MATH_Deadzone(float value, float threshold)
{
    if (threshold < 0.0f) threshold = -threshold;
    if (fabsf(value) < threshold) {
        return 0.0f;
    }
    return value;
}
```

**逐行解释**:
- `if (threshold < 0.0f) threshold = -threshold;`: 确保阈值非负。调用者可能传负数 (比如 `MATH_Deadzone(x, -0.1f)`), 函数自动取正。这看起来贴心, 但也隐藏了调用者的 bug。
- `if (fabsf(value) < threshold)`: 判断绝对值是否小于阈值。注意这里是 `<` 而非 `<=`, 等于阈值时认为有效。为什么? 如果阈值是 0.1, value 恰好是 0.1, 表示刚好达到死区边界, 应该通过。阈值 0.1 的含义是"小于 0.1 的视为 0"。
- `return 0.0f;`: 在死区内, 返回精确 0.0f。
- `return value;`: 在死区外, 返回原值 (不做任何缩放或补偿)。

**踩坑点 — 死区补偿**:
- 当前函数只做"一刀切"的死区。一些高级控制需要"死区补偿": 比如在死区边界, value 从 0 跳变到 threshold, 会导致控制量突变。对于电机控制, 这个跳变会引起顿挫感。
- 更平滑的方案是用三区段: f(x) = 0 if |x| < T1; f(x) = sign(x) * (|x| - T1) * T2/(T2-T1) if T1 <= |x| < T2; f(x) = x if |x| >= T2。但原代码追求简洁, 没有这么做。

**为什么不直接用 if(fabsf(value) < fabsf(threshold))**:
- 如果 threshold 本身是负数, `fabsf(threshold)` 取正, 效果等价。但多一次函数调用, 不如手动判断 `threshold < 0` 然后取负省事 (fabsf 内部也是类似的分支)。

---

## 5. MATH_Sign / MATH_SignInt — 符号判断函数

### 5.1 函数声明

```c
int8_t MATH_Sign(float value);
int8_t MATH_SignInt(int32_t value);
```

**这段代码在干什么**: 判断数值的符号, 返回 -1 (负)、0 (零)、1 (正)。

**解释**:
- 返回类型 `int8_t` 足够, 因为只有三个值。
- 为什么不返回 `int` 或 `int32_t`: 省 3 个字节的栈空间。虽然微不足道, 但嵌入式的代码审美就是"抠"。

**应用场景举例**:
- PID 方向判断: 如果误差符号变化, 说明过冲了, 可能需要清积分项。`if (MATH_Sign(error) != MATH_Sign(last_error)) integral = 0;`
- 电机方向: 根据速度的符号设置电机 H 桥方向引脚。
- 数值归一化: 将符号 + 绝对值分解, 分别处理。

### 5.2 函数实现

```c
int8_t MATH_Sign(float value)
{
    if (value > 0.0f) return 1;
    if (value < 0.0f) return -1;
    return 0;
}
```

```c
int8_t MATH_SignInt(int32_t value)
{
    if (value > 0) return 1;
    if (value < 0) return -1;
    return 0;
}
```

**逐行解释**:
- 先判断正, 再判断负, 最后默认 0。三个分支覆盖所有情况。
- 判断顺序有讲究: 正数判断在前, 因为在典型控制系统中, value 为正的概率大于零的概率。
- 没有使用 `value > 0 ? 1 : (value < 0 ? -1 : 0)` 三元表达式的理由: 函数可读性更好, 且编译器优化后生成的代码完全一样。

**为什么不直接用 (value > 0) - (value < 0)**:
- 这是 C 语言的 trick: `bool` 值在算术运算中转为 0 或 1, `(x > 0) - (x < 0)` 结果就是 -1、0、1。但:
  1. 对初学者可读性差
  2. 浮点和整数行为一致, 但浮点的 NaN 比较比较特殊: NaN > 0 和 NaN < 0 都是 false, 所以 `(NaN > 0) - (NaN < 0)` 返回 0, 逻辑正确
  3. 编译器可能生成更优代码? 实测两个版本优化后基本一样。

**踩坑点 — 负零 (Negative Zero)**:
- IEEE 754 浮点有 `-0.0f`。`MATH_Sign(-0.0f)` → `-0.0f > 0.0f` 为 false, `-0.0f < 0.0f` 为 false, 返回 0。但从数值分析角度, `-0.0f` 是有符号的。如果你的应用需要区分 ±0, 需要用 `signbit()` 宏。

---

## 6. MATH_FloatEq — 浮点容差比较函数

### 6.1 函数声明

```c
bool MATH_FloatEq(float a, float b, float epsilon);
```

**这段代码在干什么**: 判断两个浮点数在 epsilon 误差内是否相等。

**解释**:
- 永远不要用 `a == b` 判断浮点相等。浮点运算有精度误差, 0.1 + 0.2 不一定等于 0.3 (IEEE 754 上 0.1 + 0.2 = 0.30000000000000004)。
- epsilon 的选取非常关键, 取决于你的应用场景和数值大小。

**应用场景举例**:
- 传感器校准: 判断采集到的校准值是否与目标值足够接近。
- 稳态判断: PID 控制中判断误差是否收敛到零附近, `MATH_FloatEq(error, 0.0f, 0.01f)`。
- 单位换算验证: 测试用例中判断 `UNIT_DegToRad(180.0f)` 是否约等于 PI。

### 6.2 函数实现

```c
bool MATH_FloatEq(float a, float b, float epsilon)
{
    if (epsilon < 0.0f) epsilon = -epsilon;
    return fabsf(a - b) <= epsilon;
}
```

**逐行解释**:
- `if (epsilon < 0.0f) epsilon = -epsilon;`: 与 `MATH_Deadzone` 的阈值保护一样, 自动取正 epsilon。再次提示: 这隐藏了调用者传入负 epsilon 的错误。
- `return fabsf(a - b) <= epsilon;`: 计算差的绝对值, 与容差比较。`<=` 意味着恰好等于 epsilon 也算相等。

**精度问题 — 绝对容差 vs 相对容差**:
- 本函数使用**绝对容差**: `|a - b| <= epsilon`。当 a 和 b 的数值本身很大时, 绝对容差不够用。比如 a = 1000000.0f, b = 1000001.0f, 差为 1.0。如果 epsilon = 0.01f, 它们被认为相等。但实际上对于百万级别的数, 1.0 的误差很小, 应该相等; 而对于 0.001 级别的数, 1.0 的误差大得离谱。
- 改进方案: **相对容差** `|a - b| <= epsilon * max(|a|, |b|, 1.0)`。但这样当 a 和 b 接近零时, 相对容差退化为绝对容差。工业级方案 (如 googletest 的 EXPECT_FLOAT_EQ) 使用两者结合。
- 当前实现更适用于小数值的相等判断 (典型值 ±1 ~ ±1000)。

**为什么不直接用浮点 == 比较**:
- 寄存器精度与内存精度不同: 在 FPU 寄存器中浮点可能是 80 位扩展精度, 写入内存时截断为 32 位。所以即使同一个值, 刚从 FPU 出来的和从内存 reload 的, `==` 比较可能失败。

---

## 7. MATH_MAX / MATH_MIN / MATH_ABS / MATH_CLAMP — 宏定义

```c
#define MATH_MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MATH_MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MATH_ABS(x)    (((x) >= 0) ? (x) : -(x))
#define MATH_CLAMP(value, min, max) MATH_Limit(value, min, max)
```

**这段代码在干什么**: 用宏实现取大、取小、绝对值、限幅(别名)。

**逐行解释**:
- `#define MATH_MAX(a, b) (((a) > (b)) ? (a) : (b))`: 括号套括号。为什么每个参数都有括号? 防止运算符优先级问题。如果写成 `((a) > (b) ? a : b)`, 调用 `MATH_MAX(x & 0xFF, y & 0x0F)` 时, `a > b ? a : b` 中 `a` 和 `b` 裸奔, `&` 优先级低于 `>` 的组合会导致解析错误。每个出现的地方都加括号是最保险的。
- `#define MATH_ABS(x) (((x) >= 0) ? (x) : -(x))`: 注意 `-(x)` 的括号。如果 x 是 `a - b`, `-(a - b)` OK, 但如果写 `-x`, 展开为 `-a - b` 就凉了。
- `MATH_MAX` 和 `MATH_MIN` 是展开为类型通用宏, 支持 int、float、甚至指针比较。但不支持有副作用的参数: `MATH_MAX(++x, y)` 展开为 `(((++x) > (y)) ? (++x) : (y))`, x 被加了两次。

**为什么不用 inline 函数**:
- `MATH_MAX/MIN/ABS` 需要支持任意类型, C 的 inline 函数无法做到 (以前不行, _Generic 在 C11 中可以但嵌入式编译器支持有限)。宏是唯一的选择。

**MATH_CLAMP 为什么是宏而不是函数**:
- 它只是一个别名 `#define MATH_CLAMP(value, min, max) MATH_Limit(value, min, max)`, 提供更国际化的命名 (很多开发者熟悉 clamp 但不熟悉 limit)。宏替换后等价于直接调用 `MATH_Limit`, 没有任何额外开销。

**踩坑点 — MATH_ABS(INT32_MIN)**:
- `INT32_MIN = -2147483648`, 对其取绝对值: `-INT32_MIN` 超出了 int32_t 的正数范围 (INT32_MAX = 2147483647)。在补码表示中, `-INT32_MIN` 在数学上等于 `2^31 = 2147483648`, 但 int32_t 最大只能 2147483647。结果是**有符号整数溢出, undefined behavior**。标准 C 中, signed 溢出是未定义行为, 编译器可能优化掉你的分支, 或返回 INT32_MIN 本身。
- 解决方案: 需要特殊处理 `#define MATH_ABS_SAFE(x) ((x) == INT32_MIN ? INT32_MAX : ((x) >= 0 ? (x) : -(x)))`, 或者直接用无符号类型取反。

---

## 8. 模块间调用关系

```c
// math_utils.c
#include "math_utils.h"
#include <stddef.h>
#include <math.h>
```

**这段代码在干什么**: math_utils.c 的包含头文件。

**调用关系分析**:
- `<math.h>`: 提供 `fabsf`。在 MSPM0G3507 上, TI ARM 编译器支持硬件浮点单元 (FPU) 的单精度 `fabsf` 指令, 开销极低 (1~2 个周期), 不需要软件模拟。
- `stddef.h`: 提供 `NULL` 定义 (虽然本模块没有用 NULL 检查, 但作为惯例包含)。
- 调用方: `math_utils.h` 的声明可被任何模块包含。实测项目中以下模块使用了 math_utils:
  - `unit_conv.c`: 调用 `MATH_Limit`、`MATH_LimitInt`、`MATH_Map`
  - PID 控制模块 (假设存在): 调用 `MATH_Limit` 限幅输出, `MATH_Map` 做信号缩放
  - 传感器驱动: 调用 `MATH_Map` 做 ADC 值到物理量的线性转换
  - 遥控器解析: 调用 `MATH_Deadzone` 做摇杆回中

---

## 9. 调试与验证

### 9.1 MATH_Limit 测试

```
输入: value = 150, min = 0, max = 100 → 预期 100
输入: value = -50, min = 0, max = 100 → 预期 0
输入: value = 50,  min = 100, max = 0  → 预期 50 (自动交换)
输入: value = NaN, min = 0, max = 100 → 预期 NaN (注意这不是 bug, 是 IEEE 754 特性)
```

### 9.2 MATH_Map 测试

```
输入: value = 2048, in_min = 0, in_max = 4095, out_min = 0, out_max = 100
      → 预期 50.0 (一半位置)
输入: value = 4095, in_min = 0, in_max = 4095, out_min = 0, out_max = 100
      → 预期 100.0
输入: value = 5000, in_min = 0, in_max = 4095, out_min = 0, out_max = 100
      → 预期 122.1 (外推, 超过 out_max)
输入: value = 100, in_min = 100, in_max = 100, out_min = 0, out_max = 100
      → 预期 0.0 (除零保护)
```

### 9.3 MATH_Deadzone 测试

```
输入: value = 0.03,  threshold = 0.05 → 预期 0.0
输入: value = -0.03, threshold = 0.05 → 预期 0.0
输入: value = 0.05,  threshold = 0.05 → 预期 0.05 (等于阈值, 通过)
输入: value = 0.10,  threshold = 0.05 → 预期 0.10
```

### 9.4 MATH_FloatEq 测试

```
输入: a = 0.1 + 0.2, b = 0.3, epsilon = 0.0001 → 预期 true (浮点误差)
输入: a = 1.0000001f, b = 1.0f, epsilon = 0.0001 → 预期 true
输入: a = 100.0f, b = 100.1f, epsilon = 0.05 → 预期 false
```

### 9.5 调试手段总结

- 在串口或 J-Link RTT 上打印每个函数的输入输出, 观察是否符合预期。
- 对于 `MATH_Map`, 带几个极端值验证线性映射的外推行为。
- 对于 `MATH_Limit`, 故意传反 min/max 验证自动交换生效。
- 使用浮点比较的模块 (`MATH_FloatEq`) 可以结合在线 IEEE 754 计算器验证。

---

## 10. 常见踩坑点

### 10.1 浮点 NaN/Inf 传播

`MATH_Limit` 对 NaN 不会限幅, NaN 会直接透传。NaN 是"一丑遮百丑"的: 任何包含 NaN 的计算都产生 NaN。所以如果发现某个控制量的值变成了 NaN, 要回溯找源头 (通常是除零或 sqrt 负数)。

### 10.2 MATH_Map 除零保护阈值

1e-12 这个阈值选取是否合理? 如果 in_min 和 in_max 都是大数 (比如 1000000.0f 和 1000000.000001f), in_range = 1e-6, 大于 1e-12, 不会触发保护, 但除法结果可能因浮点精度不精确。这不是 bug, 但你需要了解精度限制。

### 10.3 宏的多遍求值 (Double Evaluation)

```c
uint16_t idx = 0;
uint16_t arr[10] = {0};
// 错误用法:
arr[MATH_MIN(++idx, 9)] = 42;
// 展开: arr[(((++idx) < (9)) ? (++idx) : (9))] = 42;
// idx 被加了两次!
```

这不是 math_utils 的 bug, 是宏的固有问题。如果需要在运行时做 max/min 且参数有副作用, 应该用函数而非宏。

### 10.4 位置式 PID vs 增量式 PID 的限幅位置

很多初学者在整个 PID 模块外面做一次限幅, 这是错的:
- **位置式 PID**: 限幅在 PID 输出之后。`output = MATH_Limit(pid_compute(), -100, 100)`。限幅后的值送入执行器。
- **增量式 PID**: 限幅在 PID 输出之后, 但还要限幅积分累积。增量式输出是 Δu, 累积后限幅: `u += delta_u; u = MATH_Limit(u, -100, 100)`。如果不限幅累积值, 积分项会 windup。
- 限幅的位置不对, 控制效果会完全不同。

### 10.5 MATH_ABS 对有符号最小值的未定义行为

前面已经强调过: `MATH_ABS(INT32_MIN)` 是未定义行为。在 MSPM0G3507 上, 实测结果可能是 INT32_MIN (不变), 也可能是 INT32_MAX, 甚至导致程序跑飞。安全写法: 如果确定会出现 INT32_MIN, 用 `uint32_t` 取绝对值: `(uint32_t)abs((int32_t)value)` 利用无符号溢出明确定义的行为取模。

### 10.6 编译器优化导致浮点比较异常

当开启 `-Ofast` 或 `-ffast-math` 时, 编译器会假设浮点运算满足交换律、结合律, 并关闭 NaN/Inf 检查。此时 `MATH_FloatEq` 的行为可能改变。建议控制模块使用 `-Og` 或 `-O2` (不带 fast-math), 或者至少让 math_utils.c 单独用 `-O2 -fno-fast-math` 编译。

---

> 本文档是针对 math_utils 模块的逐行级百科全书, 所有解释基于 MSPM0G3507 裸机环境。如有疑问, 以实际硬件调试结果为准。
