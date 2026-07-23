# diff_drive 模块 — 差速驱动运动学（正解与逆解）

> **当你看代码旁边的注释都看不懂时来翻阅的百科全书**
> 适用芯片：MSPM0G3507 @ 32MHz | 纯 C | 零硬件依赖

---

## 1. 模块概述

这个模块实现了差速底盘（Differential Drive）的**运动学正解与逆解**。简单说就是知道轮速算车速度（正解），或者知道目标速度算轮速（逆解）。

**适用范围**：双轮差速底盘，两个驱动轮独立控制，两个随动轮（万向轮）无动力。

### 代码中只实现了逆解

当前代码只提供了 `DiffDrive_WheelSpeeds` 这个**逆运动学函数**——给定车体目标线速度 v 和目标角速度 ω，算左右轮的转速。正解函数没有写，因为在这个项目中"轮速→车体速度"由 `dead_reckoning` 模块完成。

---

## 2. 数据结构：`DiffDrive_t`

```c
typedef struct {
    float wheel_base_m;
    float wheel_radius_m;
} DiffDrive_t;
```

### 逐字段解释

| 字段 | 类型 | 单位 | 含义 |
|------|------|------|------|
| `wheel_base_m` | `float` | **m** | 轮距：左右驱动轮中心之间的距离 |
| `wheel_radius_m` | `float` | **m** | 车轮半径 |

### 为什么这里用 `m` 而 dead_reckoning 用 `mm`？

- diff_drive 输出的轮速是 `rad/s`，而轮速计算的中间变量（线速度 v）在 SI 制下用 m/s 最自然
- 外部 PID 控制器的积分项通常也用 SI 单位
- 反过来 dead_reckoning 用 mm 是因为编码器脉冲增量用 mm 更直观

**不好**：两个模块单位制不统一。使用时要手动把 mm 转成 m（或者在 PID 层统一）。这算设计上的小瑕疵，但函数内部不交叉调用，所以无实际影响。

---

## 3. 运动学模型（数学推导）

### 3.1 变量定义

| 符号 | 含义 | 单位 |
|------|------|------|
| v | 车体中心线速度（前进为正） | m/s |
| ω | 车体角速度（左转为正） | rad/s |
| L | 轮距 (wheel_base_m) | m |
| r | 车轮半径 (wheel_radius_m) | m |
| v_L | 左轮线速度 | m/s |
| v_R | 右轮线速度 | m/s |
| ω_L | 左轮转速 | rad/s |
| ω_R | 右轮转速 | rad/s |

### 3.2 逆运动学公式（代码实现的）

**正问题**：已知 (v, ω) → 求 (ω_L, ω_R)

```
          v_R = v + ω · L/2
          v_L = v - ω · L/2
          
          ω_R = v_R / r
          ω_L = v_L / r
```

合并写成矩阵形式：

```
┌ ω_L ┐     ┌ 1/r   -L/(2r) ┐ ┌ v ┐
└ ω_R ┘  =  └ 1/r    L/(2r)  ┘ └ ω ┘
```

**推导逻辑**：
1. 车体中心线速度 v 是左右轮线速度的平均值：`v = (v_R + v_L) / 2`
2. 车体角速度 ω 是左右轮线速度差除以轮距：`ω = (v_R - v_L) / L`
3. 联立求解 v_R, v_L 得到上面的公式
4. 再除以半径 r 得到 rad/s 的转速

### 3.3 正运动学公式（代码未实现，仅供参考）

**反问题**：已知 (ω_L, ω_R) → 求 (v, ω)

```
v = (ω_L + ω_R) · r / 2
ω = (ω_R - ω_L) · r / L
```

这个函数在这个模块中没有写，因为 `dead_reckoning.c` 的 `DR_Update` 已经通过另一种方式完成了正解（用平均里程+陀螺仪航向）。

---

## 4. 函数详解

### 4.1 `DiffDrive_Init`

```c
void DiffDrive_Init(DiffDrive_t *dd, float wheel_base_m,
                    float wheel_radius_m)
{
    if (dd == NULL) return;
    dd->wheel_base_m   = (wheel_base_m > 0.0f)   ? wheel_base_m   : 0.1f;
    dd->wheel_radius_m = (wheel_radius_m > 0.0f) ? wheel_radius_m : 0.03f;
}
```

- 如果 wheel_base_m 传 0 或负，默认 **0.1m（10cm）**——这是常见小车轮距
- 如果 wheel_radius_m 传 0 或负，默认 **0.03m（3cm）**——典型 65mm 小车车轮半径
- 这种"静默纠错"设计在嵌入式里很常见，但调试时可能导致迷惑——用户可能忘记设置正确的轮距却没发现

### 4.2 `DiffDrive_WheelSpeeds`（核心函数）

```c
void DiffDrive_WheelSpeeds(DiffDrive_t *dd, float linear_v_ms,
                           float angular_v_rads,
                           float *left_speed, float *right_speed)
{
    if (dd == NULL) return;
    if (left_speed != NULL)  *left_speed  = 0.0f;
    if (right_speed != NULL) *right_speed = 0.0f;
    if (dd->wheel_radius_m < 1e-12f) return;
    if (left_speed == NULL || right_speed == NULL) return;

    float half_base = dd->wheel_base_m * 0.5f;
    float v_left  = linear_v_ms - angular_v_rads * half_base;
    float v_right = linear_v_ms + angular_v_rads * half_base;

    *left_speed  = v_left  / dd->wheel_radius_m;
    *right_speed = v_right / dd->wheel_radius_m;
}
```

**逐行详解：**

| 行号 | 代码 | 解释 |
|------|------|------|
| 37-39 | 空指针 + 先置零输出 | 防止输出指针指向未初始化的内存。即使后续计算失败，输出也是安全的 0 |
| 40 | 半径检查 `1e-12f` | 防除零。为什么是 1e-12？因为 float 的精度到 ~1e-38，1e-12 不算特别小但足够在工程上判定"半径几乎为零" |
| 41 | 输出指针为 NULL 时返回 | 防止写入空地址，同时留下一种"只检查不计算"的调用模式 |
| 43 | `half_base = L/2` | 预计算，避免在公式里重复写 `L/2` |
| 45-46 | 逆运动学核心 | `v_left = v - ω·L/2`, `v_right = v + ω·L/2` |
| 49-50 | 线速度→角速度 | `ω = v/r`。这里除以半径是**从线速度到轮轴转速**的关系 |

#### 符号约定

- `linear_v_ms > 0`：前进（车头向前）
- `angular_v_rads > 0`：左转（逆时针，符合 right-hand rule）
- `left_speed > 0`：左轮正转（前进方向）
- 当 `angular_v_rads > 0`（左转）时：
  - 左轮速度 = v - ωL/2 **变小**（内侧轮减速）
  - 右轮速度 = v + ωL/2 **变大**（外侧轮加速）

#### 边界情况分析

```
v = 0, ω = 0:          左轮=0, 右轮=0   (静止)
v > 0, ω = 0:          左轮=右轮=v/r    (直线)
v = 0, ω > 0:          左轮=-ωL/(2r), 右轮=+ωL/(2r)  (原地旋转)
v > 0, ω > 0:          右轮>左轮>0         (左转前进)
v > 0, ω < 0 (太大时):  左轮= v - ω·L/2, 当 ω·L/2 > v 时左轮=负值 (内侧轮反转)
```

**特别注意**：当 `|ω| > 2v/L` 时，内侧轮会**反转**。这是差速底盘的正常现象——急转弯时内侧轮需要倒转。

---

## 5. 逆解结果怎么喂给 PID

### 典型控制链路

```
目标路径
    ↓
PID 控制器  →  (v_target, ω_target)
                    ↓
          DiffDrive_WheelSpeeds
                    ↓
          (ω_left_target, ω_right_target)
                    ↓
          左右轮速度 PID 控制器
                    ↓
          PWM → 电机 → 编码器
                    ↓
            实际 (ω_actual_L, ω_actual_R)
                    ↓
            dead_reckoning (位置估计)
                    ↓
                反馈到路径
```

### 具体代码示例

```c
// 路径规划给出目标速度
float v_target = 0.5f;       // m/s
float w_target = 0.3f;       // rad/s（左转）

// 逆解算轮速
float left_spd, right_spd;
DiffDrive_WheelSpeeds(&dd, v_target, w_target, &left_spd, &right_spd);

// 左右轮速直接设为 PID 目标值
motor_left_pid.setpoint  = left_spd;   // rad/s
motor_right_pid.setpoint = right_spd;  // rad/s
```

这个输出可以直接作为电机 PID 控制器的目标值。因为两个量的单位都是 rad/s，无需额外换算。

---

## 6. 踩坑点

### 6.1 轮距测量不准

**问题**：wheel_base_m 是从左轮中心到右轮中心的距离。如果实际是 150mm 但代码里设了 160mm：

- 计算出的角速度分量会偏差约 6.7%
- 转弯半径会比预期大

**测量方法**：用卡尺量左轮和右轮在底盘上的安装孔距，而不是轮子最外侧的距离。

### 6.2 车轮半径有效半径 vs 理论半径

**问题**：橡胶轮胎充气程度不同导致有效半径变化。

- 理论半径 30mm，受压后有效半径可能只有 28mm
- 导致实际线速度比预期小 ~7%
- 里程计也会相应偏小

**解决方法**：做一次标定——走 10m 直线，看编码器计算的距离，反推出有效半径。

### 6.3 负轮速

**问题**：`left_speed` 或 `right_speed` 可能为负值。如果电机驱动不支持反转，物理上无法实现。

- 有些廉价的 L298N 驱动板支持正反转（PWM+方向引脚），没问题
- 如果只有单向驱动，则需要避免产生负轮速：限制 |ω| ≤ 2v/L

### 6.4 输出饱和

**问题**：计算出的轮速可能超过电机最大转速。

- 需要在调用后做限幅：`*left_speed = fmaxf(fminf(*left_speed, MAX_RPM_RADS), -MAX_RPM_RADS);`
- 不做限幅会导致电机丢步或电流过载

---

## 7. 调试手段

### 7.1 验证逆解正确性

验证方法：给定 v 和 ω，手动计算 ω_L, ω_R，检查符号和幅值是否符合直觉。

```
例如:
v = 0.5 m/s, ω = 0.5 rad/s, L = 0.15m, r = 0.03m

v_left  = 0.5 - 0.5 × 0.075 = 0.4625 m/s
v_right = 0.5 + 0.5 × 0.075 = 0.5375 m/s
ω_L = 0.4625 / 0.03 = 15.42 rad/s
ω_R = 0.5375 / 0.03 = 17.92 rad/s

右轮比左轮快，左转，符合预期。√
```

### 7.2 正解验证（验证逆解的逆运算）

手动把逆解的 ω_L, ω_R 代回正解公式：

```
v' = (15.42 + 17.92) × 0.03 / 2 = 0.5 m/s ✓
ω' = (17.92 - 15.42) × 0.03 / 0.15 = 0.5 rad/s ✓
```

### 7.3 实际走圈测试

让小车按 `(v=0.3, ω=0.3)` 运动：

- 理论转弯半径 R = v/ω = 1.0m
- 实际走完后测量圆弧半径，如果偏差 > 10%，检查轮距和半径参数

---

## 8. 正解与逆解的物理解读

### 逆解（已知 v, ω → 求轮速）：**控制指令下发**

用在"我要往哪走"的阶段。例如：
- PID 控制器算出了 `v=0.5, ω=0.3`
- 你要告诉电机控制器的 PWM 模块"左轮跑多快、右轮跑多快"
- 这就是逆解做的事情

### 正解（已知轮速 → 求 v, ω）：**状态观测**

用在"我现在在哪"的阶段。例如：
- 编码器读到了左右轮实际转速
- 你想知道车体实际的速度和转弯角速度
- 这就是正解做的事情（本模块未实现，但 dead_reckoning 模块做了类似的事）

**两个方向在整个控制回路中必须同时存在**：逆解控制前向路径，正解反馈后向估计。

---

## 9. 完整输入输出总结

```
DiffDrive_WheelSpeeds 调用:
  输入: linear_v_ms (m/s), angular_v_rads (rad/s)
  输出: *left_speed (rad/s), *right_speed (rad/s)

  输入范围:
    线速度: 无理论限制（实际受电机最大速度约束）
    角速度: 无理论限制（实际受电机最大差速约束）
  输出范围:
    无理论限制（取决于 v, ω, L, r 的组合）
  缺陷:
    不做任何限幅，上层需自行 saturate
```

### 典型参数值

| 参数 | 典型值 | 来源 |
|------|--------|------|
| wheel_base_m | 0.10~0.20m | 底盘设计 |
| wheel_radius_m | 0.025~0.04m | 车轮选型 |
| 左轮 PID 目标 | 0~30 rad/s | 由 v, ω 算出 |
| 右轮 PID 目标 | 0~30 rad/s | 由 v, ω 算出 |

---

## 10. 与 dead_reckoning 的关系

| | diff_drive | dead_reckoning |
|--|-----------|---------------|
| 方向 | 前向（控制） | 反向（估计） |
| 输入 | (v, ω) 目标 | (Δs_L, Δs_R, ω_gyro) |
| 输出 | (ω_L, ω_R) 目标 | (x, y, θ) |
| 频率 | 控制 loop 频率 | 编码器中断频率 |
| 关联 | 下指令 | 读反馈 |

闭环示例：

```
设定目标速度 → diff_drive 逆解 → 电机 → 编码器 → dead_reckoning → 修正目标
```
