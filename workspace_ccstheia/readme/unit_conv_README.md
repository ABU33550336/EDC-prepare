# unit_conv — 单位换算模块百科全书

> **README 定位**: 当你看代码旁边的注释都看不懂时来翻阅的百科全书。
> **目标平台**: MSPM0G3507 @ 32MHz, 可移植到任何 C 环境
> **核心思想**: 角度/脉宽/占空比单位换算, 依赖 math_utils, 用于舵机控制和信号处理

---

## 目录

1. [模块概览](#1-模块概览)
2. [PI_F 宏定义](#2-pi_f-宏定义)
3. [UNIT_DegToRad / UNIT_RadToDeg — 角度↔弧度](#3-unit_degtorad--unit_radtodeg--角度弧度)
4. [UNIT_PulseToAngle — 舵机脉宽转角度](#4-unit_pulsetoangle--舵机脉宽转角度)
5. [UNIT_AngleToPulse — 角度转舵机脉宽](#5-unit_angletopulse--角度转舵机脉宽)
6. [UNIT_DutyCycleToPercent / UNIT_PercentToDutyCycle — 占空比↔百分比](#6-unit_dutycycletopercent--unit_percenttodutycycle--占空比百分比)
7. [模块间调用关系](#7-模块间调用关系)
8. [调试与验证](#8-调试与验证)
9. [常见踩坑点](#9-常见踩坑点)

---

## 1. 模块概览

```c
//单位转换工具,角度/占空比/脉宽换算
#ifndef UNIT_CONV_H
#define UNIT_CONV_H

#include <stdint.h>

#define PI_F 3.14159265f

float UNIT_DegToRad(float deg);
float UNIT_RadToDeg(float rad);
float UNIT_PulseToAngle(uint16_t pulse_us, uint16_t min_us,
                        uint16_t max_us, float angle_min,
                        float angle_max);
uint16_t UNIT_AngleToPulse(float angle_deg, uint16_t min_us,
                           uint16_t max_us, float angle_min,
                           float angle_max);
float UNIT_DutyCycleToPercent(float duty);
float UNIT_PercentToDutyCycle(float percent);

#endif
```

**这段代码在干什么**: 头文件声明, 定义 6 个公共接口函数 + 圆周率宏。

**设计思路**:
- 函数的输入输出单位一目了然: 函数名就说明了转换方向。比如 `UNIT_DegToRad` 输入度、输出弧度。
- 使用 `uint16_t` 表示脉宽 (微秒), 因为舵机脉宽通常在 500~2500μs 范围, `uint16_t` 足够。
- 使用 `float` 表示角度, 因为舵机角度控制需要连续值 (如 90.5°)。
- 依赖 math_utils (调用 `MATH_Limit`、`MATH_LimitInt`、`MATH_Map`), 模块之间形成了"工具层 -> 应用层"的层次。

---

## 2. PI_F 宏定义

```c
#define PI_F 3.14159265f
```

**这段代码在干什么**: 定义单精度圆周率 π 的近似值。

**为什么是 3.14159265f 而不是 M_PI**:
- `M_PI` 在 `<math.h>` 中定义, 但它是 double 类型 (3.14159265358979323846)。如果用 `M_PI`, 与 float 运算时会触发 double→float 转换, 编译器可能产生警告或额外类型转换指令。
- 3.14159265f 的单精度精度约为 7~8 位有效数字, 对于角度→弧度的换算 (误差约 10⁻⁷ rad), 在嵌入式控制中完全足够。舵机角度控制 0.1° 分辨率才 1.7×10⁻³ rad, 所以这个精度绰绰有余。
- 为什么不直接用 `3.14159265358979f`? float 只有 23 位尾数 (约 7.2 位十进制), 后面再多位在编译时也会被截断, 所以 3.14159265f 刚好填满 float 精度, 再长也没用。

**更精确的 π 值**: `3.14159265358979323846` 是 double 精度。float 版的精确 16 进制是 `0x40490FDB` (即 3.14159265...)。当前宏的值正确。

---

## 3. UNIT_DegToRad / UNIT_RadToDeg — 角度↔弧度

### 3.1 UNIT_DegToRad

```c
float UNIT_DegToRad(float deg)
{
    return deg * (PI_F / 180.0f);
}
```

**这段代码在干什么**: 角度乘以 π/180 得到弧度。

**数学公式**: rad = deg × π / 180

**应用场景**:
- 三角函数计算: MSPM0 的 `sinf()`、`cosf()` 接受弧度参数。如果用户输入的是角度, 先调用此函数转换。
- 电机控制中的电角度: 电机磁场定向控制 (FOC) 中经常在角度和弧度之间转换。
- GPS/导航: 经纬度是角度, 但航向角等计算需要弧度。

**为什么不用 `deg * 0.0174532925f`**: 等价, 但 `PI_F / 180.0f` 可读性更好。编译器在优化阶段会计算出常量 0.0174532925f, 运行时没有除法。

### 3.2 UNIT_RadToDeg

```c
float UNIT_RadToDeg(float rad)
{
    return rad * (180.0f / PI_F);
}
```

**这段代码在干什么**: 弧度乘以 180/π 得到角度。

**应用场景**:
- 反三角函数输出转换: `atan2f(y, x)` 返回弧度, 如果你需要角度, 用此函数。
- 角度显示: 用户界面通常显示角度而非弧度。

**精度分析**:
- 180.0f / PI_F = 57.2957795... (常量)
- 输入 rad = 1.57079633f (π/2) → 输出 89.9999...°
- float 的单向误差约 0.00002°, 对舵机控制 (精度 0.5° 已很高) 完全够用

---

## 4. UNIT_PulseToAngle — 舵机脉宽转角度

### 4.1 什么是舵机脉宽

标准舵机 (如 SG90、MG995) 的控制信号是 PWM, 周期 20ms (50Hz), 脉宽范围通常:
- 0°: 500μs ~ 1000μs (因型号而异)
- 90°: 1500μs (中位)
- 180°: 2000μs ~ 2500μs

所以舵机角度控制的本质是: **将目标角度映射到对应的脉宽值**, 或者反向从接收到的脉宽值解码出角度。

### 4.2 函数实现

```c
float UNIT_PulseToAngle(uint16_t pulse_us, uint16_t min_us,
                        uint16_t max_us, float angle_min,
                        float angle_max)
{
    if (max_us <= min_us) return angle_min;
    uint16_t pulse = MATH_LimitInt((int32_t)pulse_us,
                                   (int32_t)min_us, (int32_t)max_us);
    return MATH_Map((float)pulse, (float)min_us, (float)max_us,
                    angle_min, angle_max);
}
```

**这段代码在干什么**: 将脉宽值 (μs) 线性映射到角度值。

**逐行解释**:
- `if (max_us <= min_us) return angle_min;`: 参数校验。如果最大脉宽 ≤ 最小脉宽 (通常是用户配置错误), 返回最小角度作为安全值。为什么要保护? 因为如果 max_us == min_us, 后续 MATH_Map 中的 in_range = 0, 会触发除零保护返回 out_min (= angle_min)。所以这里提前拦截, 语义更清晰。

  > **注意**: 这里用的是 `<=` 而非 `<`。max_us == min_us 意味着脉宽范围为零, 舵机无法控制, 返回 angle_min 是最安全的行为 (停在最小角度, 不会异常转动)。

- `uint16_t pulse = MATH_LimitInt((int32_t)pulse_us, (int32_t)min_us, (int32_t)max_us);`: 将脉宽限幅到合法范围内。类型转换:
  - `uint16_t` → `int32_t`: 因为 MATH_LimitInt 接受 int32_t。uint16_t 的值范围 0~65535 在 int32_t 中完全可表示, 没有精度损失。
  - 限幅的意义: 如果接收到的脉宽信号异常 (如干扰导致 capture 模块抓到异常值), 限幅防止映射出异常角度 (如 -30° 或 210°), 导致舵机撞击机械限位。
  - 为什么不是 `MATH_Limit((float)pulse_us, ...)`: 因为 pulse_us 是整数, 用整数限幅避免浮点转换开销。

- `return MATH_Map((float)pulse, (float)min_us, (float)max_us, angle_min, angle_max);`: 限幅后的脉宽映射到角度。强制类型转换 `(float)` 是为 MATH_Map 准备的, 因为 MATH_Map 接受 float。

**应用场景**:
- 舵机反馈位置读取: 有些舵机 (如 AX-12A、SR-系列) 能反馈当前脉宽, 用此函数解码为角度。
- 遥控器通道解析: 遥控器接收机输出 PWM 脉宽, 映射到角度值用于控制云台或机械臂。

**角度范围灵活性**:
- 不限于 0°~180°。如果你用连续旋转舵机 (360° 或自定义范围), 传入对应的 angle_min/angle_max 即可。
- 甚至可以用在非角度映射: 比如将油门摇杆的脉宽映射到 0~100% 推力。

---

## 5. UNIT_AngleToPulse — 角度转舵机脉宽

### 5.1 函数实现

```c
uint16_t UNIT_AngleToPulse(float angle_deg, uint16_t min_us,
                            uint16_t max_us, float angle_min,
                            float angle_max)
{
    if (fabsf(angle_max - angle_min) < 1e-12f) {
        return (uint16_t)((min_us + max_us) / 2);
    }
    float angle = MATH_Limit(angle_deg, angle_min, angle_max);
    float pulse = MATH_Map(angle, angle_min, angle_max,
                           (float)min_us, (float)max_us);
    return (uint16_t)(pulse + 0.5f);
}
```

**这段代码在干什么**: 将角度值映射回舵机脉宽 (μs)。

**逐行解释**:
- `if (fabsf(angle_max - angle_min) < 1e-12f)`: 除零保护。如果角度范围过小 (几乎为零), 无法做线性映射, 返回中点脉宽。

  > 为什么这里不直接委托给 MATH_Map 的除零保护? 因为数学上, 角度范围为零意味着只有一个固定角度, 对应脉宽应该是中点。如果返回 MATH_Map 的结果 (即 out_min = min_us), 则脉宽总是 min_us, 不支持 angle_deg 变化。所以这里做了不同处理: 返回中点。

- `float angle = MATH_Limit(angle_deg, angle_min, angle_max);`: 角度限幅。防止设置超出舵机机械范围的角度。例如用户程序计算出 200° 但舵机只支持 0~180°, 限幅到 180°(或 angle_max)。

- `float pulse = MATH_Map(angle, angle_min, angle_max, (float)min_us, (float)max_us);`: 角度→脉宽的线性映射。

- `return (uint16_t)(pulse + 0.5f);`: 四舍五入到整数。因为脉宽是整数微秒。
  - `pulse + 0.5f`: 浮点加 0.5。如果 pulse = 1500.3, 加 0.5 = 1500.8, 截断 = 1500 (正确舍去); 如果 pulse = 1500.7, 加 0.5 = 1501.2, 截断 = 1501 (正确入)。
  - `(uint16_t)`: 浮点到无符号整数的强制转换。这里假设 pulse 在 0~65535 范围内 (由 LIMIT 保证)。
  - 注意: 如果 pulse 是负数 (理论上不会, 因为 angle 限幅在 angle_min~angle_max, 映射后脉宽在 min_us~max_us 之间), `(uint16_t)` 负数会变成很大的正数 (环绕), 那将是个灾难。但 MATH_Limit + MATH_Map 保证了不会发生。

**为什么不直接用 `roundf(pulse)`**:
- `roundf()` 是 math.h 的标准函数, 需要链接 libm。用 `pulse + 0.5f` 截断是嵌入式领域手动实现四舍五入的惯用技巧, 无需额外链接数学库。
- 但 `pulse + 0.5f` 对负数不适用 (应该 `pulse - 0.5f`), 不过本函数 pulse 恒正, 所以没问题。
- 对于 `uint16_t` 截断, `pulse + 0.5f` 的值如果超过 65535 会发生环绕 (典型值 max_us=2500 不会溢出)。

**应用场景**:
- 舵机控制: 主控根据目标角度计算 PWM 比较值。
  ```c
  uint16_t pulse = UNIT_AngleToPulse(target_angle, 500, 2500, 0.0f, 180.0f);
  TIMER_SetCompare(TIM0, 0, pulse);  // 设置 PWM 占空比
  ```
- 机械臂逆运动学: IK 计算出各关节角度值, 转换为脉宽发送给舵机控制板。

---

## 6. UNIT_DutyCycleToPercent / UNIT_PercentToDutyCycle — 占空比↔百分比

### 6.1 UNIT_DutyCycleToPercent

```c
float UNIT_DutyCycleToPercent(float duty)
{
    return MATH_Limit(duty, 0.0f, 1.0f) * 100.0f;
}
```

**这段代码在干什么**: 占空比 (范围 0~1) 转换为百分比 (范围 0~100)。

**逐行解释**:
- `MATH_Limit(duty, 0.0f, 1.0f)`: 限幅保护。如果 duty 因为计算误差出现 -0.001 或 1.001, 限幅后不会输出奇怪的百分比。
- `* 100.0f`: 缩放。为什么不 `duty * 100`? 如果用整数 100, `duty * 100` 中 duty 是 float, 100 是 int, 编译器会把 100 提升为 float 再运算, 结果 float, 效果一样。但写 `100.0f` 更明确, 避免整数提升的歧义。

**为什么先限幅再乘 100**:
- 如果先乘 100 再限幅: `MATH_Limit(duty * 100.0f, 0.0f, 100.0f)`, 结果一样, 但乘了之后数值更大, 浮点精度问题更小。不过原方案更直观。

### 6.2 UNIT_PercentToDutyCycle

```c
float UNIT_PercentToDutyCycle(float percent)
{
    return MATH_Limit(percent, 0.0f, 100.0f) / 100.0f;
}
```

**这段代码在干什么**: 百分比 (范围 0~100) 转换为占空比 (范围 0~1)。

**对称性**: UNIT_DutyCycleToPercent 和 UNIT_PercentToDutyCycle 互为反函数:
- `UNIT_PercentToDutyCycle(UNIT_DutyCycleToPercent(x))` = x (在 [0,1] 范围内)

**应用场景**:
- PWM 占空比设置: 有些高级 API 接受百分比值 (如 `SetMotorPower(75%)`), 内部需要转为 0~1 的占空比写入 TIMER CCR。
- UI 显示: 人机界面显示"当前亮度: 45%", 底层驱动需要 0~1 值。
- 传感器归一化: 将 0~100 的输出范围归一化为 0~1, 方便后续算法处理。

---

## 7. 模块间调用关系

```c
//unit_conv.c
#include "unit_conv.h"
#include <stddef.h>
#include "math_utils.h"
#include <math.h>
```

**这段代码在干什么**: unit_conv.c 的包含文件。

**依赖链分析**:
- `math_utils.h`: 本模块的核心依赖。调用 `MATH_Limit`、`MATH_LimitInt`、`MATH_Map`。
- `<math.h>`: 调用 `fabsf` (在 UNIT_AngleToPulse 中)。注意只有这一个地方用了 math.h。有些人可能会说: 为了一个 `fabsf` 引入整个数学库是不是代价太大? 在 MSPM0G3507 上, `fabsf` 是一个 FPU 指令 (VABS.F32), 不需要浮点库的软实现, 链接时也不会拉入 libm 的其余部分 (因为没用到 sin/cos/sqrt 等)。所以实际影响为零, 但代码可读性提升了。
- `stddef.h`: 提供 NULL, 虽然本模块没用 NULL (但作为惯例包含)。

**本模块被谁调用**:
- **舵机驱动层**: 封装 UNIT_AngleToPulse / UNIT_PulseToAngle, 提供高层次的舵机角度接口。
- **电机驱动**: 使用 UNIT_DutyCycleToPercent 将 PWM 占空比显示为百分比。
- **UI 界面模块**: 使用 UNIT_RadToDeg 将传感器弧度值转为角度显示给用户。
- **传感器驱动**: 某些角度传感器 (如 MPU6050 DMP 输出四元数, 转欧拉角后为弧度), 调用 UNIT_RadToDeg 输出度。

---

## 8. 调试与验证

### 8.1 手工计算验证

**UNIT_DegToRad 测试**:
```
输入: 180.0f
计算: 180.0 * (3.14159265 / 180.0) = 3.14159265f = PI_F
预期: 3.14159265f
```

```
输入: 90.0f
计算: 90.0 * (3.14159265 / 180.0) = 1.570796325f
预期: π/2 ≈ 1.57079633f
```

**UNIT_RadToDeg 测试**:
```
输入: 3.14159265f
计算: 3.14159265 * (180.0 / 3.14159265) = 180.0f
预期: 180.0f
```

```
输入: 1.57079633f
计算: 1.57079633 * (180.0 / 3.14159265) = 90.0f
预期: 90.0f
```

### 8.2 UNIT_PulseToAngle 手工验证

```
min_us=500, max_us=2500, angle_min=0.0, angle_max=180.0

pulse_us=500 → angle = 0.0°
pulse_us=1500 → angle = 90.0° (中点)
pulse_us=2500 → angle = 180.0°

pulse_us=1000 → ratio = (1000-500)/(2500-500) = 500/2000 = 0.25
                 angle = 0 + 0.25 * 180 = 45.0°

pulse_us=2000 → ratio = (2000-500)/2000 = 1500/2000 = 0.75
                 angle = 0 + 0.75 * 180 = 135.0°
```

### 8.3 UNIT_AngleToPulse 手工验证

```
min_us=500, max_us=2500, angle_min=0.0, angle_max=180.0

angle=0° → pulse = 500μs
angle=90° → pulse = 1500μs
angle=180° → pulse = 2500μs

angle=45° → ratio = (45-0)/(180-0) = 0.25
             pulse = 500 + 0.25 * 2000 = 500 + 500 = 1000μs

angle=200° → 先限幅到 180° → pulse = 2500μs
```

### 8.4 双向转换验证

```
angle = 45.0°
pulse = UNIT_AngleToPulse(45.0, 500, 2500, 0, 180) = 1000μs
calc  = UNIT_PulseToAngle(1000, 500, 2500, 0, 180) = 45.0° ✓
```

由于浮点运算的微小误差, 这个往返测试可能得到 44.9999° 或 45.0001°。这是正常的, 用 MATH_FloatEq 判断即可。

### 8.5 占空比验证

```
UNIT_DutyCycleToPercent(0.5f) = 50.0%
UNIT_DutyCycleToPercent(1.0f) = 100.0%
UNIT_DutyCycleToPercent(1.5f) = 100.0% (限幅后)
UNIT_DutyCycleToPercent(-0.1f) = 0.0% (限幅后)

UNIT_PercentToDutyCycle(50.0f) = 0.5f
UNIT_PercentToDutyCycle(100.0f) = 1.0f
UNIT_PercentToDutyCycle(0.0f) = 0.0f
```

### 8.6 在线计算器辅助验证

- 角度↔弧度: 用任意在线单位换算器验证
- 舵机脉宽映射: 在 Excel 或 Python 中画出脉宽-角度曲线, 与函数输出对比
- Python 验证脚本:
```python
import struct

# 模拟 C 的 float 精度
PI_F = 3.14159265

def deg_to_rad(deg):
    return deg * (PI_F / 180.0)

def rad_to_deg(rad):
    return rad * (180.0 / PI_F)

def pulse_to_angle(pulse, min_us, max_us, angle_min, angle_max):
    if max_us <= min_us:
        return angle_min
    pulse = max(min_us, min(pulse, max_us))
    ratio = (pulse - min_us) / (max_us - min_us)
    return angle_min + ratio * (angle_max - angle_min)

def angle_to_pulse(angle, min_us, max_us, angle_min, angle_max):
    if abs(angle_max - angle_min) < 1e-12:
        return (min_us + max_us) // 2
    angle = max(angle_min, min(angle, angle_max))
    ratio = (angle - angle_min) / (angle_max - angle_min)
    return int(min_us + ratio * (max_us - min_us) + 0.5)

# 测试
print(deg_to_rad(180))      # 3.14159265
print(rad_to_deg(3.14159265)) # 180.0
print(pulse_to_angle(1500, 500, 2500, 0, 180))  # 90.0
print(angle_to_pulse(90, 500, 2500, 0, 180))     # 1500
```

---

## 9. 常见踩坑点

### 9.1 脉宽极性混淆 (CRITICAL)

不同舵机品牌的正逻辑和负逻辑:
- **正逻辑 (大多数)**: 脉宽 500μs = 0°, 2500μs = 180° (脉宽与角度成正比)
- **负逻辑 (少数)**: 脉宽 2500μs = 0°, 500μs = 180° (脉宽与角度成反比)

本模块假设正逻辑。如果用负逻辑的舵机, 交换 min_us 和 max_us 即可:
```c
// 负逻辑舵机: 500μs=180°, 2500μs=0°
float angle = UNIT_PulseToAngle(pulse, 500, 2500, 180.0f, 0.0f);
```

但注意, `UNIT_AngleToPulse` 如果 min_us > max_us 会触发 `if (max_us <= min_us) return angle_min`, 返回 angle_min 而不是正确计算。所以对于负逻辑, 你必须传入 min_us=500, max_us=2500, angle_min=180, angle_max=0 (即 min_us 对应 angle_min, 不管数值大小)。

### 9.2 2的幂次角度范围导致映射精度损失

如果 angle_min = 0.0, angle_max = 180.0, 归一化因子是 180。但如果 angle_min = 0.0, angle_max = 360.0:
- 比值范围在 [0, 1] 之间
- MATH_Map 内部: `ratio = (value - in_min) / (in_max - in_min)` = `value / 360.0f`
- 360.0f 不是 2 的幂, 除法用 FPU 的 FDIV, 约 12 周期。而如果用 256 或 512 范围, 可以用位移优化。但角度映射中 180/360 是自然范围, 不可能硬改成 256。

### 9.3 UNIT_AngleToPulse 的浮点→整数截断问题

```c
return (uint16_t)(pulse + 0.5f);
```

如果 pulse 的计算结果是 65535.6 (脉冲值超过 uint16_t 范围), `(uint16_t)` 转换会截断为 65535 (取模 65536), 不是饱和。但在正常使用中:
- min_us, max_us 在 500~2500 范围
- pulse 不可能超过 2500
- 所以不会出现这个问题

但如果调用者传入 max_us = 60000 等异常值, pulse 可能接近 60000, 在 uint16_t 范围内 (最大 65535), 仍然安全。

### 9.4 角度单位和我们的直觉

角度有两种: **度 (degree)** 和 **弧度 (radian)**。本模块两个方向都提供。但要注意:
- `UNIT_PulseToAngle` 的 angle_min/angle_max 的单位是度 (因为函数名说是 angle, 不是 rad)。但在注释中没有明确标注单位。调用者必须按度传入。
- 如果用弧度传入, 结果会差 57.3 倍。

**建议**: 在使用 UNIT_PulseToAngle 和 UNIT_AngleToPulse 时, 始终以度为单位。如果需要弧度映射, 先用 UNIT_RadToDeg 转换再调用。

### 9.5 浮点精度对舵机控制的影响

MSPM0G3507 的 FPU 是单精度。对于 0~180° 映射到 500~2500μs:
- 角度步长: 0.1° → 对应脉宽步长: (2500-500)/180° * 0.1° = 1.11μs
- float 的精度: 尾数 23 位 → 相对精度约 1.19×10⁻⁷, 绝对精度在 2000 量级约为 0.00024μs
- 所以 float 精度远远高于 PWM 定时器的分辨率 (通常 1μs)

结论: 精度完全够。

### 9.6 MATH_LimitInt 的 int32_t 转换

```c
uint16_t pulse = MATH_LimitInt((int32_t)pulse_us, (int32_t)min_us, (int32_t)max_us);
```

这里 `pulse_us` 是 `uint16_t` (0~65535), `min_us` 和 `max_us` 也是 `uint16_t`。转换为 `int32_t` 安全, 因为没有正数超出 int32_t 范围。但如果 `min_us > max_us`, MATH_LimitInt 会交换它们, 所以传入顺序不重要。

一个隐蔽问题: 如果 `min_us` 是 0, `pulse_us` 是 65535, `max_us` 是 2500。限幅后 pulse = 2500。但如果传入了 `min_us=65535, max_us=0`, 交换后 min=0, max=65535, 限幅结果是 65535。这时的 pulse 在 `[0, 65535]` 范围内, 传入 MATH_Map 时转为 (float)pulse, 映射到角度。这会超出正常范围, 但调用者的参数配置错误导致的, 函数本身无法防御。

### 9.7 UNIT_DutyCycleToPercent 的负占空比

在某些三相 PWM 或 H 桥驱动中, 占空比可以是负数 (代表反向旋转)。本模块的 `UNIT_DutyCycleToPercent` 将 -0.5 这样的值限幅到 0.0, 失去了方向信息。

如果系统需要 "双向占空比" (-100%~100%), 你应该:
```c
// 自定义双向映射
float duty_to_percent_bipolar(float duty) {
    return duty * 100.0f;  // 不限幅, 假设 duty 在 [-1, 1]
}
```

但本模块的 `MATH_Limit` 限幅在 [0,1], 说明本模块设计者假设占空比是单极性的 (0~1)。

### 9.8 PI_F 与 math.h 的 M_PI 冲突

如果其他模块 `#include <math.h>`, 且使用了 `M_PI` (double), 而本模块又定义了 `PI_F` (float), 不会冲突, 因为名字不同。但要注意值的一致性:
- M_PI = 3.14159265358979323846 (double)
- PI_F = 3.14159265f (float)

在混合使用时要小心隐式类型转换:
```c
// some_other_module.c
#include <math.h>
#include "unit_conv.h"

double a = M_PI;       // 3.141592653589793
float b = PI_F;        // 3.14159265f
float c = (float)M_PI; // 3.14159265f (转换后与 PI_F 相同)
```

---

> 本文档是针对 unit_conv 模块的逐行级百科全书, 所有解释基于 MSPM0G3507 裸机环境。如有疑问, 以实际硬件调试结果为准。
