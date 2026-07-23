# servo_control 模块完全解读

> **定位**: 当你看代码旁边的注释都看不懂时来翻阅的百科全书  
> **目标读者**: 刚接手项目的嵌入式新手、不了解步进电机控制原理的开发者  
> **硬件平台**: MSPM0G3507 @ 32MHz Cortex-M0+  
> **电机**: ZDT_X42S 闭环步进电机 (张大头)  
> **协议**: 0x6B 固定校验 UART 协议 / STEP + DIR 脉冲控制  
> **最后更新**: 2026-06-18

---

## 目录

1. [模块概述](#1-模块概述)
2. [头文件解读: servo_control.h](#2-头文件解读-servo_controlh)
3. [源文件总览: servo_control.c](#3-源文件总览-servo_controlc)
4. [编译开关详解](#4-编译开关详解)
5. [静态全局变量: 模块的"记忆"](#5-静态全局变量-模块的记忆)
6. [DEG_PER_STEP 数学推导](#6-deg_per_step-数学推导)
7. [UART 收发底层函数](#7-uart-收发底层函数)
8. [0xFD 位置指令协议逐字节拆解](#8-0xfd-位置指令协议逐字节拆解)
9. [fdCommand 组包发送](#9-fdcommand-组包发送)
10. [0x36 位置查询协议](#10-0x36-位置查询协议)
11. [PULSE 脉冲模式详解](#11-pulse-脉冲模式详解)
12. [Servo_Init 初始化](#12-servo_init-初始化)
13. [Servo_SetAngle 设定角度](#13-servo_setangle-设定角度)
14. [Servo_GetAngle 读取角度](#14-servo_getangle-读取角度)
15. [Servo_SetSpeed / Servo_SetAccel](#15-servo_setspeed--servo_setaccel)
16. [Servo_Stop 紧急停止](#16-servo_stop-紧急停止)
17. [Servo_Status 状态查询](#17-servo_status-状态查询)
18. [调用链与数据流](#18-调用链与数据流)
19. [SysConfig 外设配置要求](#19-sysconfig-外设配置要求)
20. [调试手段](#20-调试手段)
21. [常见踩坑点总结](#21-常见踩坑点总结)

---

## 1. 模块概述

这个模块负责控制一个 **ZDT_X42S 闭环步进电机**（俗称"张大头"电机，淘宝上常见的云台步进电机）。它是**闭环**的，意味着电机内部有编码器反馈，你只需告诉它"走到哪里"，电机会自己确保到达目标位置，不会丢步。

模块支持**两种**控制模式:

| 模式 | 宏定义 | 原理 | 适用场景 |
|------|--------|------|----------|
| UART 模式 | `SERVO_MODE_UART` | 通过 TTL 串口发 0x6B 协议指令 | 单个或少量电机，需要简单接线 |
| 脉冲模式 | `SERVO_MODE_PULSE` | STEP 引脚发脉冲 + DIR 引脚定方向 | 多轴联动、高速输出 |

⚠️ **注意**: `SERVO_MODE_UART` 用的是 TTL 串口 (3.3V 电平)，不是 RS232 (±12V) 也不是 RS485 (差分)。直接接电机的串口 TX/RX 即可。千万不要试图通过 USB 转 RS232 模块直连电机，会烧毁电机串口芯片。

---

## 2. 头文件解读: servo_control.h

### 2.1 头文件保护

```c
#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H
```

**WHY**: 这是 C/C++ 头文件的标准保护宏。如果没有这个，当两个源文件都 `#include "servo_control.h"` 时，编译器会重复展开头文件，导致"重复定义"编译错误。  
**原理**: 预处理器看到 `#ifndef`（if not defined）检查某个宏是否被定义过。第一次展开时没有定义过，就 `#define` 它，然后处理后面的内容。第二次展开时发现宏已存在，直接跳过整个文件内容。

### 2.2 包含的标准库

```c
#include <stdint.h>
#include <stdbool.h>
```

**WHY**: 
- `stdint.h` 提供了 `uint8_t`、`uint16_t`、`int32_t` 等**定宽整数类型**。使用定宽类型确保在不同编译器（TI Clang）上 int 的大小一致。`int` 在 8 位单片机上是 16 位，在 32 位 MCU 上是 32 位，用 `int16_t` 就明确是 16 位有符号——这非常重要，因为后面做角度计算时，int16_t 溢出会导致电机反转到反方向。
- `stdbool.h` 提供了 `bool`、`true`、`false`。C89 标准没有 bool 类型，C99 才通过这个头文件引入。

### 2.3 C++ 兼容

```c
#ifdef __cplusplus
extern "C" {
#endif
```

**WHY**: 这个头文件可能会被 C++ 文件 `#include`（实际上从 `empty_cpp.cpp` 里确实是用 `extern "C" {}` 包裹引用的）。C++ 编译器会对函数名进行**名字改编** (name mangling)，而 C 编译器不会。如果头文件里的函数声明在 C++ 里被按照 C++ 方式改编了，但 `.c` 文件是用 C 方式编译的，链接时就会找不到符号。

这个 `#ifdef __cplusplus` + `extern "C"` 组合的意思是：如果当前是 C++ 编译器，告诉它"这些函数按照 C 的方式链接"。如果当前是 C 编译器，`__cplusplus` 没有定义，就跳过这个声明。

### 2.4 状态码与模式常量

```c
#define SERVO_OK            0    //函数执行成功
#define SERVO_ERR           1    //函数执行失败
#define SERVO_BUSY          2    //电机正在运动中(仅脉冲模式)

#define SERVO_MODE_UART     0    //串口 TTL 0x6B 协议
#define SERVO_MODE_PULSE    1    //STEP/DIR 脉冲控制
```

**WHY 用 `#define` 而不是 `enum`**: 
- `#define` 是预处理器指令，在编译之前就完成了文本替换，不占用 RAM，不产生调试符号。
- 如果用了 `enum`，在某些编译器优化级别下枚举常量可能会占用栈空间。
- 但缺点是 `#define` 没有类型检查——如果你不小心写了 `if (g_mode == 123)`，编译器不会警告。

**WHY 从 0 开始取值**: `SERVO_OK = 0` 符合 C 语言"0 表示成功"的惯例（和 `EXIT_SUCCESS` 一致，和 Linux 系统调用的返回值语义一致）。

### 2.5 电机参数常量

```c
#define SERVO_ADDR          1    //电机 Modbus 地址
#define SERVO_BAUD          115200
#define SERVO_STEPS_PER_REV 3200 //200步/圈 * 16细分
```

**`SERVO_ADDR = 1`**: 电机的 0x6B 协议支持多机级联——通过 RS485（需额外转接板）可以将多个电机挂在同一总线上，每个电机分配不同地址。这里固定为 1，表示控制的是地址为 0x01 的电机。如果你在电机上位机软件里改过地址，这里必须同步改。

**`SERVO_BAUD = 115200`**: 这是电机的默认波特率。ZDT_X42S 支持 9600、19200、38400、57600、115200、230400。但注意：**230400 在 MSPM0G3507 上可能会因为 32MHz 时钟分频产生较大的波特率误差（超过 ±2% 会导致误码）**。

**`SERVO_STEPS_PER_REV = 3200`**: 这个值是**物理步数 × 驱动器细分**。物理步数 200 步/圈是电机转子的固有特性（步进电机定子有 50 对磁极 × 4 拍 = 200 全步/圈）。细分 16 意味着驱动器把每一步再细分成 16 个微步，所以 200 × 16 = 3200 微步/圈。细分越大，振动越小、分辨率越高，但同样速度下需要更高的脉冲频率。

### 2.6 函数声明

```c
uint8_t  Servo_Init(uint8_t mode);
void     Servo_SetAngle(float deg);
float    Servo_GetAngle(void);
void     Servo_SetSpeed(uint16_t rpm);
void     Servo_SetAccel(uint16_t rpm_per_s);
void     Servo_Stop(void);
uint8_t  Servo_Status(void);
void     Servo_StepIRQ(void);
```

**WHY 返回 `uint8_t` 而不是 `int`**: 嵌入式代码讲究最小化栈使用。返回值只有 0/1/2 三种状态，`uint8_t`（1 字节）就够了。如果用 `int`，在某些架构上是 4 字节，浪费。

**WHY `Servo_StepIRQ` 没有返回值和参数**: 它是一个中断回调函数，由 TIMER 的更新中断调用。中断服务函数有严格的限制——不能阻塞、不能传参（因为中断硬件不会传参数）。它直接操作全局静态变量 `g_steps_remaining`。

---

## 3. 源文件总览: servo_control.c

`servo_control.c` 的完整结构如下:

```
servo_control.c
├── 头文件包含 (#include)
├── 编译开关定义 (SERIAL_CTRL_ENABLED / PULSE_CTRL_ENABLED)
├── 静态全局变量 (g_mode, g_status, g_current, g_target, g_speed, g_accel)
├── DEG_PER_STEP 宏
├── ═══ UART 模式代码块 ═══
│   ├── 协议常量 (CMD_POS, CMD_READ_POS, CHKSUM, RX_TIMEOUT)
│   ├── uartSend()        发送字节数组
│   ├── uartRecv()        非阻塞接收一个字节
│   └── fdCommand()       组包并发送 0xFD 位置指令
├── ═══ PULSE 模式代码块 ═══
│   ├── g_steps_remaining  剩余步数 (volatile int32_t)
│   ├── g_step_period      当前脉冲周期
│   └── Servo_StepIRQ()    STEP 脉冲中断回调
└── ═══ 公有 API ═══
    ├── Servo_Init()
    ├── Servo_SetAngle()
    ├── Servo_GetAngle()
    ├── Servo_SetSpeed()
    ├── Servo_SetAccel()
    ├── Servo_Stop()
    └── Servo_Status()
```

---

## 4. 编译开关详解

```c
#define SERIAL_CTRL_ENABLED   1   //UART TTL 0x6B 协议
#define PULSE_CTRL_ENABLED    0   //STEP/DIR 脉冲(需 SysConfig 配 TIMER)
```

### 4.1 为什么要用 `#if` 而不是 `if`？

这是**最关键也是新手最容易忽略**的区别:

```c
//方式 A: #if 编译开关 (本模块使用的方式)
#if SERIAL_CTRL_ENABLED
#include "ti_msp_dl_config.h"
static void uartSend(...) { ... }
#endif

//方式 B: if 运行时判断
if (SERIAL_CTRL_ENABLED) {
    // 仍然要编译这段代码
}
```

| 对比项 | `#if` 方式 | `if` 方式 |
|--------|------------|-----------|
| 判断时机 | 预处理阶段（编译前） | 运行时（MCU 执行时） |
| 未用代码 | 根本不会进入编译器，完全不存在于二进制中 | 仍然会编译进二进制，只是条件不执行 |
| 依赖的头文件 | 可以通过 `#if` 控制是否 `#include` | 必须 `#include`，否则编译报错 |
| 外围设备要求 | 模式未启用时不需要配置对应的外设 | 即使不用也得配置，否则链接出错 |

**具体到这个项目**:
- 当 `PULSE_CTRL_ENABLED = 0` 时，`#if PULSE_CTRL_ENABLED` 内的所有代码（包括 `Servo_StepIRQ` 函数体、`g_steps_remaining` 变量声明）**完全不存在于编译结果中**。这样：
  - 节省 Flash 空间
  - SysConfig 不需要配置 TIMER，省下一个外设
  - 不会产生"定义了 TIMER 相关的符号但没初始化"的警告

### 4.2 不改开关编译会怎样？

如果 `SERIAL_CTRL_ENABLED = 1` 但你**没有在 SysConfig 中配置 UART_0**：

```
#error: UART_0_INST 未定义
ti_msp_dl_config.h: 没有 UART_0 相关的配置
```

解决方案是打开 SysConfig，确保 UART_0 外设已配置（后面有详细配置清单）。

### 4.3 开关依赖的外设清单

| 开关 | 值 | 需要的外设 | 原因 |
|------|-----|-----------|------|
| `SERIAL_CTRL_ENABLED` | 1 | UART0 (PA10 TX, PA11 RX) | 发送/接收 0x6B 协议指令 |
| `PULSE_CTRL_ENABLED` | 0 | TIMERx + GPIO(STEP) + GPIO(DIR) | 产生 STEP 脉冲，控制方向引脚 |

⚠️ **注意**: 即使 `PULSE_CTRL_ENABLED = 0`，如果同时 `SERIAL_CTRL_ENABLED = 0`，整个模块几乎什么都不做——`Servo_Init` 直接返回 0，`Servo_SetAngle` 只更新 `g_target` 但不发任何指令给电机。

---

## 5. 静态全局变量: 模块的"记忆"

```c
static uint8_t  g_mode     = SERVO_MODE_UART;
static uint8_t  g_status   = SERVO_OK;
static float    g_current  = 0.0f;
static float    g_target   = 0.0f;
static uint16_t g_speed    = 1500;
static uint16_t g_accel    = 8;
```

### 5.1 `static` 关键字在这里的作用

`static` 加在**文件作用域的全局变量**前，意味着这个变量**只在当前源文件可见**。其他源文件（比如 `empty_cpp.cpp`）即使写了 `extern uint8_t g_mode;` 也无法访问。

**WHY**: 这是一种封装。模块的内部状态不应该被外部随意修改。如果外部直接写 `g_current = 999.0f;`，那电机实际位置和软件记录的位置就不一致了。外部只能通过 `Servo_GetAngle()`、`Servo_SetSpeed()` 等 API 间接访问。

### 5.2 每个变量的设计意图

| 变量 | 类型 | 大小范围 | 为什么是这个类型 | 为什么是这个初始值 |
|------|------|----------|-----------------|-------------------|
| `g_mode` | `uint8_t` | 0~255 | 只需要存 0/1 两值 | `SERVO_MODE_UART = 0` 是默认模式 |
| `g_status` | `uint8_t` | 0~2 | 只需要 OK/ERR/BUSY | 初始空闲 |
| `g_current` | `float` | ±1e38 | 角度需要小数（0.1125°级别） | 上电默认在 0° |
| `g_target` | `float` | ±1e38 | 同上 | 默认目标是 0° |
| `g_speed` | `uint16_t` | 0~3000 | 电机速度范围 0~3000RPM，uint16_t 够用 | 1500 RPM 是中间值 |
| `g_accel` | `uint16_t` | 0~255(实际) | 加速度字段只有 1 字节 | 8 是比较柔和的加速度 |

### 5.3 `g_speed = 1500` 为什么不慢不快的道理

1500 RPM = 1500 圈/分钟 = 25 圈/秒。对于步进电机来说，这个速度在中等水平:
- 太慢（< 300 RPM）：电机低速振动明显，而且云台转动看起来卡顿
- 太快（> 2500 RPM）：200 步的电机在高速区力矩骤降，可能会失步（虽然是闭环电机，但超出电机力矩极限仍会报错）

### 5.4 `0.0f` 中 f 后缀的重要性

在 C 语言中，`0.0` 是 `double` 类型，`0.0f` 是 `float` 类型。MSPM0G3507 是 Cortex-M0+ **没有硬件 FPU**（浮点运算单元），所有浮点运算都由软件模拟。

- `double` 是 8 字节，软件浮点库处理 `double` 比 `float` 慢得多
- `float` 是 4 字节，运算更快、占用栈空间更少

所以这里坚持用 `0.0f` 而不是 `0.0`，是为了避免不必要的类型转换。

⚠️ **`float` 精度在角度计算中够吗？**

`float` 有大约 7.2 位有效十进制数字。角度值在 -360°~360° 范围内，小数部分可以精确到约 0.0001°。而 `DEG_PER_STEP = 0.1125°`，差了三个数量级。所以 `float` 精度完全够用。

---

## 6. DEG_PER_STEP 数学推导

```c
#define DEG_PER_STEP  (360.0f / SERVO_STEPS_PER_REV)
```

### 6.1 为什么是 360/3200？

这是一个简单的除法:

```
一圈 = 360°
一圈 = 3200 微步（200 物理步 × 16 细分）

每个微步的角度 = 360° / 3200 = 0.1125°
```

精确量化:
```
360 / 3200 = 360 ÷ 3200
           = 0.1125
           = 9/80°
```

所以每发一个 STEP 脉冲（脉冲模式）或每发一个微步指令（UART 模式），电机转动 0.1125°。

### 6.2 如果改了细分怎么办？

假设把细分从 16 改为 8:
- `SERVO_STEPS_PER_REV` 要从 3200 改为 1600
- `DEG_PER_STEP` 会变成 360/1600 = 0.225°，分辨率减半
- 同速度下需要的脉冲频率减半，对 MCU 负担更小

如果改为 32 细分:
- `SERVO_STEPS_PER_REV` 改为 6400
- `DEG_PER_STEP` 变成 360/6400 = 0.05625°（惊人的精度）
- PULSE 模式下，同样转速需要加倍脉冲频率——如果 `Servo_StepIRQ` 是 TIMER 中断驱动的，中断频率翻倍可能影响系统实时性

### 6.3 步数和角度互转时的精度损失

```c
// 角度 → 步数（在 Servo_SetAngle 中）
int16_t steps = (int16_t)((deg - g_current) / DEG_PER_STEP);

// 步数 → 角度（在 Servo_GetAngle 中）
g_current = (float)steps * DEG_PER_STEP;
```

角度转步数时，`(deg - g_current) / DEG_PER_STEP` 产生浮点结果，然后截断为整数。假设 `deg - g_current = 1.0°`:

```
1.0 / 0.1125 = 8.888... 步
int16_t 截断后 = 8 步
8 × 0.1125 = 0.9° ← 有 0.1° 的精度损失
```

这意味着如果你反复调用 `Servo_SetAngle(1.0)`，每次都会有 0.1° 的"死区误差"。电机不是精确停在 1°，而是停在 0.9°。

**对这个项目的影响**: 云台控制系统对角度精度的要求不高（0.1° 的误差在摄像头画面中几乎不可察觉），所以这个截断是可以接受的。但如果控制的是激光雷达或精密对准机构，需要使用四舍五入或累积误差补偿。

---

## 7. UART 收发底层函数

### 7.1 uartSend 逐字节发送

```c
static void uartSend(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        DL_UART_Main_transmitData(UART_0_INST, data[i]);
    }
}
```

**`static`**: 这个函数仅在当前 .c 文件内可见。外部代码不能直接调用 `uartSend`，只能通过 `Servo_SetAngle()` 等公有 API 间接发送数据。

**`const uint8_t *data`**: `const` 表示这个函数承诺不会修改 `data` 指向的内容。这允许你传入 `const` 数组（如 `uint8_t cmd[3] = { ... }`，其实 `cmd` 数组本身不是 const，但函数以只读方式访问它）。

**`DL_UART_Main_transmitData` 在硬件上做了什么**:

1. 检查 UART 外设的 TX FIFO（发送先进先出缓冲区）是否有空位
2. 将 `data[i]` 这个字节写入 TX FIFO
3. UART 硬件自动从 TX FIFO 取数据，逐位通过 TX 引脚（PA10）发送出去
4. 发送完成后硬件自动把 TX FIFO 中的这个字节移除

**SysConfig 配置要求**: 
- TX FIFO 必须启用（`DL_UART_Main_enableFIFOs`）
- TX FIFO 阈值设为 `DL_UART_TX_FIFO_LEVEL_1_2_EMPTY`（TX FIFO 空一半时触发中断——虽然本模块没开中断，但阈值设置是为了兼容性）

⚠️ **重要**: `DL_UART_Main_transmitData` 在 TI 的 DriverLib 中是**阻塞直到 TX FIFO 有空间才写入**，但不会等待发送完成。如果连续发送大量数据，TX FIFO（深度 4 字节）满后这个函数会阻塞等待。对 13 字节的帧来说，FIFO 深度够用（4 字节），循环 13 次中最多等 3 次。

### 7.2 uartRecv 非阻塞接收

```c
static bool uartRecv(uint8_t *byte)
{
    uint32_t t = RX_TIMEOUT;
    while (DL_UART_isRXFIFOEmpty(UART_0_INST)) {
        if (--t == 0) return false;
    }
    *byte = DL_UART_Main_receiveData(UART_0_INST);
    return true;
}
```

**`DL_UART_isRXFIFOEmpty`**: 检查 RX FIFO 中是否有数据。返回 `true` 表示 RX FIFO 为空（没有收到任何字节），返回 `false` 表示有数据可读。

**轮询超时机制**: 
- `RX_TIMEOUT = 10000` 是一个"忙等次数"。
- `while (DL_UART_isRXFIFOEmpty(UART_0_INST))` 是个空循环。每次循环检查 FIFO 是否非空，同时把 `t` 递减。
- 如果 10000 次循环后 FIFO 仍然为空，返回 `false` 表示超时。
- 这个超时不是以时间为单位的——10000 次循环在 32MHz 下大约 0.3ms 左右（每条空循环大约 3~4 个时钟周期）。

**`*byte = DL_UART_Main_receiveData(UART_0_INST)`**: 从 RX FIFO 读取一个字节。这个函数**不阻塞**——因为调用前已经确认了 FIFO 非空。

**`uartRecv` 的缺陷和风险**:

⚠️ **这是本模块最大的潜在问题**:

1. **总线锁死风险**: 如果其他任务（比如 `ControlTask_10ms`）占用了大量 CPU 时间，`uartRecv` 在返回 `false` 之前会一直占用 CPU——但因为是轮询而不是中断方式，其实不会阻塞其他任务，只是浪费了 CPU 周期。真正的风险是 `Servo_GetAngle` 里的循环 `while (idx < 6 && uartRecv(&resp[idx]))` 在电机没响应时会执行 6 × 10000 = 60000 次轮询，大约 2ms 的浪费——还好。

2. **数据错位**: 如果电机返回了 6 个字节，但程序开始接收时只收到了第 2~5 字节（前 1 个字节丢了或还没到），接收到的数据会错位。代码没有帧头检查机制。

3. **缓冲区溢出**: `resp[8]` 只有 8 字节，但 `while` 循环的上限是 6，所以安全。但如果改了协议长度没改代码就会溢出。

---

## 8. 0xFD 位置指令协议逐字节拆解

### 8.1 协议概述

ZDT_X42S 电机使用 0x6B 固定校验协议。每个指令帧以固定字节 `0x6B` 作为校验值（不是和校验，不是 CRC，就是一个**固定值**）。0xFD 是"位置模式指令"的功能码。

### 8.2 帧结构表格

0xFD 位置指令固定 13 字节:

| 字节索引 | 名称 | 值/范围 | 大小端 | 说明 |
|---------|------|---------|--------|------|
| [0] | 地址 | 0x01~0xFF | - | 电机 Modbus 地址，默认 0x01 |
| [1] | 功能码 | 0xFD | - | 位置模式指令标识 |
| [2] | 方向 | 0x00(CCW) / 0x01(CW) | - | CCW=逆时针，CW=顺时针 |
| [3] | 转速高字节 | 0x00~0xFF | 大端(Big-Endian) | `g_speed >> 8` |
| [4] | 转速低字节 | 0x00~0xFF | 大端 | `g_speed & 0xFF` |
| [5] | 加速度 | 0x00~0xFF | - | 加速度值，单位 r/min/s |
| [6] | 保留 | 0x00 | - | 固定填充 0 |
| [7] | 保留 | 0x00 | - | 固定填充 0 |
| [8] | 目标位置高字节 | 0x00~0xFF | 大端 | `steps >> 8` |
| [9] | 目标位置低字节 | 0x00~0xFF | 大端 | `steps & 0xFF` |
| [10] | 同步标记 | 0x00(立即) / 0x01(等待同步) | - | 多机同步控制 |
| [11] | 保留 | 0x00 | - | 固定填充 0 |
| [12] | 校验 | 0x6B | - | 固定校验字节 |

### 8.3 大小端疑问: 为什么只有 2 字节表示位置？

乍看之下，目标位置只占 `[8]` 和 `[9]` 两字节，可以表示 `0~65535` 微步。

```
65535 微步 / 3200 微步/圈 = 20.48 圈
```

即最大可以控制电机转 20 圈——足够云台使用了（云台通常只需要 ±180° = 半圈）。

但本代码中 `steps` 是 `uint16_t` 类型的最大值就是 65535。如果角度差导致 `steps > 65535`，会发生溢出截断：

```c
steps = (uint16_t)steps;  // steps 如果超过 65535，会被截断为 steps % 65536
```

⚠️ **溢出场景**: 假设电机当前位置在 0°，目标设为 7500°（约 20.8 圈），计算出的步数超过 65535，截断后电机只会转很少的角度——而不是 20 圈。

在实际项目中，云台角度不太可能设置超过 360°，这个问题不常见。

### 8.4 转速字段范围

`g_speed` 是 `uint16_t`，但填入协议时拆成两个字节:

```
转速 = [3] × 256 + [4]
```

协议中转速的单位是 0.1 RPM。也就是说如果 `g_speed = 1500`，协议中代表 150.0 RPM。

⚠️ **注意**: 不同版本的 ZDT_X42S 固件对转速单位解释可能不同。有的用 0.1RPM，有的用 1RPM。需要实测验证。如果电机转得和预期相差 10 倍，就是这个单位问题。

### 8.5 为什么不用和校验而是固定 0x6B？

这是一个**简化设计**。标准的 Modbus 协议使用 CRC-16 校验，但 ZDT_X42S 为了简化 MCU 端的计算，使用了固定校验值 0x6B。

**问题**: 固定校验意味着**任何错误的指令帧只要最后一个字节是 0x6B 都会被电机认为校验通过**。所以协议本身没有防错能力。依赖的是:
1. 功能码检查：如果 [1] 不是 0xFD，电机忽略
2. 地址检查：如果 [0] 不是自己的地址，电机忽略

---

## 9. fdCommand 组包发送

```c
static void fdCommand(uint8_t dir, uint16_t steps, uint8_t sync)
{
    uint8_t pkt[13] = {
        SERVO_ADDR,       // [0] 电机地址(默认 0x01)
        CMD_POS,          // [1] 0xFD 位置模式
        dir,              // [2] 转向
        (uint8_t)(g_speed >> 8),   // [3] 转速高字节
        (uint8_t)(g_speed & 0xFF), // [4] 转速低字节
        (uint8_t)g_accel,          // [5] 加速度
        0x00, 0x00,       // [6-7] 保留/填充,固定 0
        (uint8_t)(steps >> 8),     // [8] 目标位置高字节
        (uint8_t)(steps),          // [9] 目标位置低字节
        sync, 0x00,       // [10] 同步标记,[11]保留
        CHKSUM            // [12] 固定校验 0x6B
    };
    uartSend(pkt, sizeof(pkt));
}
```

### 9.1 为什么速度用 `g_speed` 而地址用宏 `SERVO_ADDR`？

地址是**编译期固定的**（宏），因为系统不太可能运行时动态改变电机地址。

转速是**运行时可变的**（通过 `Servo_SetSpeed()` 修改），所以从全局变量 `g_speed` 读取。

这个设计体现了"运行时常量和配置常量的分离"——如果未来需要支持动态地址，需要把地址也改为全局变量。

### 9.2 `(uint8_t)(g_speed >> 8)` 的分解原理

```
假设 g_speed = 1500 (0x05DC)
二进制: 0000 0101 1101 1100

>> 8:   0000 0000 0000 0101 = 0x05
& 0xFF: 0000 0000 1101 1100 = 0xDC

pkt[3] = 0x05
pkt[4] = 0xDC

电机收到后拼接: (0x05 << 8) | 0xDC = 0x05DC = 1500
```

这就是**大端字节序**（Big-Endian）——高字节在前，低字节在后。

### 9.3 同步标记 `sync` 有什么用？

当 `sync = 1` 时，电机收到指令后**不会立即执行**，而是等待主控发送一个"同步触发信号"（通常是另一个小指令帧）。多台电机可以同时接受到同步信号，实现多轴同步启动。

本项目不用多轴同步，所以 `sync` 始终传 0。

---

## 10. 0x36 位置查询协议

### 10.1 帧结构

```c
uint8_t cmd[3] = { SERVO_ADDR, CMD_READ_POS, CHKSUM };
uartSend(cmd, 3);
```

发送: 3 字节
| 字节 | 值 | 含义 |
|------|-----|------|
| [0] | 0x01 | 电机地址 |
| [1] | 0x36 | 读取位置指令 |
| [2] | 0x6B | 固定校验 |

### 10.2 电机应答格式

电机收到 0x36 指令后，返回 6 字节:

| 字节 | 含义 | 范围 | 说明 |
|------|------|------|------|
| [0] | 地址 | 0x01 | 电机地址回显 |
| [1] | 功能码 | 0x36 | 指令回显 |
| [2] | 位置字节3 (MSB) | 0x00~0xFF | 有符号 32 位整数的最高 8 位 |
| [3] | 位置字节2 | 0x00~0xFF | |
| [4] | 位置字节1 | 0x00~0xFF | |
| [5] | 位置字节0 (LSB) | 0x00~0xFF | 有符号 32 位整数的最低 8 位 |
| [6] | 校验 | 0x6B | (注: 代码中resp数组大小为8,但只收了6字节) |

### 10.3 位置解析代码

```c
int32_t steps = ((int32_t)resp[2] << 24)
              | ((int32_t)resp[3] << 16)
              | ((int32_t)resp[4] << 8)
              | (int32_t)resp[5];
g_current = (float)steps * DEG_PER_STEP;
```

**为什么是 `int32_t` 而不是 `uint32_t`**?

因为位置可以是负的——电机可以反向运动。`int32_t` 的范围是 -2^31 ~ 2^31-1（约 -21 亿 ~ 21 亿微步，对应约 ±65 万圈）。这对云台控制绰绰有余。

**`(int32_t)resp[2] << 24` 的细节**:

```c
// 假设 resp[2] = 0x80
(int32_t)resp[2]      → 0x00000080
(int32_t)resp[2] << 24 → 0x80000000

// 如果 resp[2] = 0xFF (负数的最高字节)
resp[2] = 0xFF(无符号, 255)
(int32_t)resp[2]      → 0x000000FF (不是 0xFFFFFFFF, 因为强转的是无符号→有符号)
```

⚠️ **注意**: 这里先把 `uint8_t` 强转为 `int32_t` 再移位。由于 `resp[2]` 是 `uint8_t`，它的值是 0~255。强转为 `int32_t` 时，是**零扩展**（不是符号扩展），所以 `0xFF` 变成 `0x000000FF` 而不是 `0xFFFFFFFF`。这在**正数**情况下是正确的。但如果电机返回的位置是负数（大负数），最高字节可能是 0xFF 或更高的值。这种"无符号→有符号"的转换会得到错误结果。

**正确的做法**: 应该在同一表达式中处理符号:

```c
int32_t steps = (int32_t)((uint32_t)resp[2] << 24 | (uint32_t)resp[3] << 16 | (uint32_t)resp[4] << 8 | (uint32_t)resp[5]);
```

这样所有字节拼成 `uint32_t`，然后整体强转为有符号 `int32_t`，利用 C 语言的实现定义行为来获得补码负数。

⚠️ **实际上，这个代码对正位置是完全正确的**。只要电机位置 < 2^31 微步（约 671,088 圈），最高字节的第 7 位就是 0，不会有符号问题。

---

## 11. PULSE 脉冲模式详解

> **当前状态**: `PULSE_CTRL_ENABLED = 0`，以下代码**不参与编译**。以下分析供未来启用脉冲模式时参考。

### 11.1 脉冲模式专用变量

```c
static volatile int32_t  g_steps_remaining = 0;
static volatile uint32_t g_step_period     = 0;
```

**`volatile` 关键字**: 这是**必须**的。因为 `g_steps_remaining` 被：
- 主循环（或者定时任务）在 `Servo_SetAngle` 中写入
- 中断服务函数 `Servo_StepIRQ` 中读取和修改

如果没有 `volatile`，编译器可能会优化掉对 `g_steps_remaining` 的读取——它以为这个变量不会被外部改变。比如:

```c
// 没有 volatile 时编译器可能优化的代码
while (g_steps_remaining != 0) {
    // ...
}
// 优化后变成:
if (g_steps_remaining != 0) {
    while (1) { } // 死循环! 因为编译器认为 g_steps_remaining 不会改变
}
```

`volatile` 告诉编译器"这个变量可能在程序控制流之外被修改"，所以每次访问都要从内存读取，不能使用寄存器中的缓存值。

### 11.2 Servo_StepIRQ 中断回调

```c
void Servo_StepIRQ(void)
{
    if (g_steps_remaining == 0) {
        g_status = SERVO_OK;
        return;
    }
    if (g_steps_remaining > 0) g_steps_remaining--;
    else                       g_steps_remaining++;
    g_current += (g_steps_remaining > 0) ? DEG_PER_STEP : -DEG_PER_STEP;
}
```

**逻辑解释**:

1. 如果剩余步数为 0，把状态改为 `SERVO_OK`，退出
2. 如果剩余步数 > 0，说明是正方向运动，剩余步数 -1
3. 如果剩余步数 < 0，说明是反方向运动，剩余步数 +1（注意是 +1，向零趋近）
4. 根据当前剩余步数的符号，决定 `g_current` 是增加还是减少一个 `DEG_PER_STEP`

**典型运行示例**:

```
初始: g_steps_remaining = 1000 (正方向走 1000 步)
第 1 次中断: 1000 > 0 → 减到 999,  g_current += 0.1125°
第 2 次中断: 999 > 0  → 减到 998,  g_current += 0.1125°
...
第 1000 次:  1 > 0    → 减到 0,    g_current += 0.1125°
第 1001 次:  0 → g_status = OK, 返回
```

**`g_current` 的更新问题**: 代码中 `g_current` 是在**中断中**逐步更新的。这意味着在主循环中调用 `Servo_GetAngle()` 时，返回的是当前已经走过的步数对应的角度，不是终点角度。

⚠️ **当前的梯形加减速是 TODO 状态**: 代码中注明了 `TODO:在此函数中动态调整TIMER周期实现梯形加减速`。所以现在的 `Servo_StepIRQ` 是**恒速运动**——启动和停止时没有加减速过程，电机会突然启动、突然停止。在高速时会导致机械振动和失步风险。

---

## 12. Servo_Init 初始化

```c
uint8_t Servo_Init(uint8_t mode)
{
    g_mode   = mode;
    g_status = SERVO_OK;
    g_current = 0.0f;
    g_target  = 0.0f;

#if SERIAL_CTRL_ENABLED
    if (mode == SERVO_MODE_UART) {
        while (DL_UART_Main_isBusy(UART_0_INST));
    }
#endif
#if PULSE_CTRL_ENABLED
    if (mode == SERVO_MODE_PULSE) {
        //TODO: 初始化 TIMER 输出 PWM 到 STEP 引脚 + GPIO 方向引脚
    }
#endif
    return SERVO_OK;
}
```

### 12.1 为什么初始化时清空角度

`g_current = 0.0f; g_target = 0.0f;` 意味着一上电，模块认为电机目前在 0°。

**但实际上电机制造时没有装零点传感器，上电位置可能是随机的**。所以 `g_current = 0` 只是一个软件假设。真正的"归零"操作需要在外部进行: 比如让电机转向限位开关，撞到限位后调用 `Servo_SetAngle(0)`。

### 12.2 `DL_UART_Main_isBusy` 等待

这个函数检查 UART 外设是否正在发送数据。`while` 循环等待直到 UART 空闲。

**WHY**: 如果主程序在调用 `Servo_Init` 之前（比如上一个系统 reset 前）已经有未发送完的数据在 TX FIFO 中，不等待就发送新指令可能导致两帧数据粘连——电机无法正确解析帧边界。

### 12.3 调用者: main

`empty_cpp.cpp` 中的调用:

```c
Servo_Init(SERVO_MODE_UART);
Servo_SetSpeed(1500);
Servo_SetAccel(8);
```

注意调用**顺序**: 必须先调 `Servo_Init` 才能调 `Servo_SetSpeed` 等。因为 `Servo_SetSpeed` 只是写全局变量，不依赖初始化状态——但从逻辑上，模式都没初始化就设置参数是不合理的。

---

## 13. Servo_SetAngle 设定角度

```c
void Servo_SetAngle(float deg)
{
    g_target = deg;

#if SERIAL_CTRL_ENABLED
    if (g_mode == SERVO_MODE_UART) {
        int16_t steps = (int16_t)((deg - g_current) / DEG_PER_STEP);
        if (steps == 0) return;
        uint8_t dir = (steps >= 0) ? 0x01 : 0x00;
        if (steps < 0) steps = -steps;
        fdCommand(dir, (uint16_t)steps, 0);
        g_current = deg;
    }
#endif

#if PULSE_CTRL_ENABLED
    if (g_mode == SERVO_MODE_PULSE) {
        int32_t total = (int32_t)(deg / DEG_PER_STEP);
        g_steps_remaining = total - (int32_t)(g_current / DEG_PER_STEP);
        if (g_steps_remaining == 0) return;
        //TODO: 根据 g_steps_remaining 正负设置 DIR 引脚电平
        //TODO: 启动 TIMER 输出 PWM,开始发步进脉冲
        g_status = SERVO_BUSY;
    }
#endif
}
```

### 13.1 UART 模式详解

**计算需要走的步数**:

```c
int16_t steps = (int16_t)((deg - g_current) / DEG_PER_STEP);
```

假设当前位置 `g_current = 0°`，目标 `deg = 90°`:

```
steps = (90.0 - 0.0) / 0.1125 = 800.0
int16_t steps = 800
```

**方向判断**:

```c
uint8_t dir = (steps >= 0) ? 0x01 : 0x00;
if (steps < 0) steps = -steps;
```

`0x01 = CW`（顺时针），`0x00 = CCW`（逆时针）。注意 here `>= 0` 时是顺时针——包括 `steps = 0` 情况。但前面已经有 `if (steps == 0) return;`，所以不会出现 `steps = 0` 调用 `fdCommand` 的情况。

**假设性更新 `g_current`**:

```c
g_current = deg;
```

⚠️ **这是 UART 模式最需要理解的代码**:

这条语句**不等电机到位**就立即把 `g_current` 更新为 `deg`。也就是说，软件认为"指令已经发出去了，电机最终会到达目标角度"，但并没有任何确认机制。

**后果**: 如果你连续快速调用 `Servo_SetAngle`:
```
Servo_SetAngle(90.0f);   // g_current = 90, 发指令走 800 步
Servo_SetAngle(0.0f);    // g_current = 0,  发指令走 -800 步
```

第一帧指令发出后电机刚启动，第二帧指令就来了。电机收到的是"从当前位置往前走 -800 步"——但电机的"当前位置"是实际位置，不等于 `g_current`。如果两次指令间隔太短，电机可能无法正常响应。

**最佳实践**: 在调用 `Servo_SetAngle` 之间间隔至少 100ms（对于 3200 步/圈的中速运动），让电机有足够时间接收并开始执行指令。

### 13.2 PULSE 模式详解

```c
int32_t total = (int32_t)(deg / DEG_PER_STEP);
g_steps_remaining = total - (int32_t)(g_current / DEG_PER_STEP);
```

这里有一个关键区别: UART 模式用**相对位移**（目标 - 当前位置），PULSE 模式用**绝对位置差**（目标总步数 - 当前总步数）。

```
假设: g_current = 45° (对应 400 步), deg = 90° (对应 800 步)
total = 90 / 0.1125 = 800
g_steps_remaining = 800 - 400 = 400 (向前走 400 步)
```

再次调用 `Servo_SetAngle(0)`:
```
total = 0 / 0.1125 = 0
g_steps_remaining = 0 - 800 = -800 (向后走 800 步)
```

---

## 14. Servo_GetAngle 读取角度

```c
float Servo_GetAngle(void)
{
#if SERIAL_CTRL_ENABLED
    if (g_mode == SERVO_MODE_UART) {
        uint8_t cmd[3] = { SERVO_ADDR, CMD_READ_POS, CHKSUM };
        uartSend(cmd, 3);

        uint8_t resp[8];
        uint16_t idx = 0;
        while (idx < 6 && uartRecv(&resp[idx])) { idx++; }
        if (idx >= 6) {
            int32_t steps = ((int32_t)resp[2] << 24)
                          | ((int32_t)resp[3] << 16)
                          | ((int32_t)resp[4] << 8)
                          | (int32_t)resp[5];
            g_current = (float)steps * DEG_PER_STEP;
        }
    }
#endif
    return g_current;
}
```

### 14.1 为什么发送 3 字节、接收期望 6 字节？

发送的 3 字节是"查询指令"（地址 + 功能码 + 校验），接收的是电机返回的完整应答（地址 + 功能码 + 4 字节位置 + 可能是校验字节）。

但代码中 `resp` 有 8 字节数组，只读了前 6 字节，因为电机应答只有 6 字节。

### 14.2 通信失败处理

如果 `uartRecv` 在任一字节超时，`idx` 可能小于 6，此时 `if (idx >= 6)` 条件不成立，`g_current` **不会被更新**，函数返回上一次缓存的值。

**问题**: 如果通信失败，调用者不知道失败了——因为返回值仍然是 `g_current`（可能是旧的正确值）。调用者应该同时检查 `Servo_Status()` 或者对比返回值和上一次的差值来判断是否收到了新数据。

### 14.3 `0.0f` 可能是错误也可能是原点

函数注释中说: "0.0f 也可能是电机恰好在原点，需结合 Servo_Status 判断。" 但如果通信失败，函数的返回值就是 `g_current` 的现有值。如果之前从来没有设置过 `Servo_SetAngle`，`g_current` 就是初始值 `0.0f`——这种情况下你无法区分"通信失败"和"电机真在 0°"。

---

## 15. Servo_SetSpeed / Servo_SetAccel

```c
void Servo_SetSpeed(uint16_t rpm)
{
    g_speed = rpm;
}

void Servo_SetAccel(uint16_t rpm_per_s)
{
    g_accel = rpm_per_s;
}
```

**WHY 这么简单**: 这两个函数只是把参数存到全局变量中，并不做任何硬件操作。真正的使用发生在 `fdCommand` 中——每次发指令时从全局变量读取当前速度/加速度值。

**WHY `uint16_t` 但加速度只有 1 字节**: 速度是 2 字节字段（大端拆分到 `pkt[3]` 和 `pkt[4]`），而加速度只有 1 字节字段（`pkt[5]`）。虽然 `g_accel` 是 `uint16_t`，但传入 `fdCommand` 时强转为 `(uint8_t)g_accel`，高 8 位被截断。所以有效的加速度范围是 0~255。

⚠️ **加速度的含义**: ZDT_X42S 的加速度单位是 r/min/s（每分钟每秒多少转）。也就是说如果设置加速度为 8，电机从静止加速到 1500 RPM 需要 `1500 / 8 = 187.5` 秒——这显然太慢了。所以实际上加速度的含义可能是 10 倍或 100 倍的关系，需要参考电机厂商文档。实测下来，加速度 8 是比较柔和的启停。

---

## 16. Servo_Stop 紧急停止

```c
void Servo_Stop(void)
{
#if SERIAL_CTRL_ENABLED
    if (g_mode == SERVO_MODE_UART) {
        uint8_t pkt[13] = {
            SERVO_ADDR, CMD_POS, 0x00,
            0x00, 0x00, (uint8_t)g_accel,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, CHKSUM
        };
        uartSend(pkt, sizeof(pkt));
    }
#endif
#if PULSE_CTRL_ENABLED
    if (g_mode == SERVO_MODE_PULSE) {
        g_steps_remaining = 0;
        g_status = SERVO_OK;
    }
#endif
}
```

### 16.1 UART 模式的停止原理

发送的 0xFD 指令中:
- 方向 = 0x00（虽然没用——速度 0 方向无意义）
- 转速 = 0x0000（速度 0）
- 目标位置 = 0x0000（目标微步数 0）

电机收到后，根据"速度 0，目标 0"理解为主控要求停止，于是锁死在当前位置。

⚠️ **这不是失能**: 停止后电机仍然处于**使能锁止**状态，即电机线圈仍然通电保持位置。如果想完全关闭电机（让电机轴自由转动），需要发送 0xFE 指令或通过 Modbus 写寄存器。关闭使能后电机就不耗电了，但位置会被外力改变。

### 16.2 PULSE 模式的停止

直接清零 `g_steps_remaining`，`Servo_StepIRQ` 看到剩余步数为 0 就把状态设为 OK。TIMER 不会自动停止——需要 `Servo_StepIRQ` 里额外添加停止 TIMER 的代码（当前是 TODO）。

---

## 17. Servo_Status 状态查询

```c
uint8_t Servo_Status(void)
{
    return g_status;
}
```

**UART 模式**: `g_status` 在初始化时设为 `SERVO_OK`，且在 `Servo_SetAngle` 中不更新为 `SERVO_BUSY`——所以在 UART 模式下 `Servo_Status()` **始终返回 OK**。因为 UART 模式下无法实时跟踪电机是否到达目标——电机内部闭环自己走，软件不等待。

**PULSE 模式**: `g_status` 在 `Servo_SetAngle` 中设为 `SERVO_BUSY`，在 `Servo_StepIRQ` 中当步数走完时设为 `SERVO_OK`。所以 PULSE 模式下可以轮询 `Servo_Status()` 来判断运动是否完成。

---

## 18. 调用链与数据流

### 18.1 整体调用图

```
硬复位
  │
  ▼
main()  ← empty_cpp.cpp:80
  │
  ├─ SYSCFG_DL_init()              // SysConfig 初始化所有外设
  ├─ HW_MotorInit()                // 底盘电机初始化
  ├─ HW_EncoderInit()              // 编码器初始化
  ├─ HW_LineSensorInit()           // 灰度传感器初始化
  ├─ Servo_Init(SERVO_MODE_UART)   // 云台步进电机初始化
  ├─ Servo_SetSpeed(1500)          // 设置速度
  ├─ Servo_SetAccel(8)             // 设置加速度
  ├─ PID_Inc_Init(...)             // PID 初始化
  ├─ SysTick_Config(...)           // 配置系统时钟
  ├─ Sched_Init(...)               // 调度器初始化
  │
  ├─ Sched_RegisterTask(..., ControlTask_10ms,   10ms)  // 注册控制任务
  ├─ Sched_RegisterTask(..., ServoTestTask_2000ms, 2000ms)  // 注册云台测试任务
  │
  └─ while(1) { Sched_Run(&sched); }
       │
       ├─ ControlTask_10ms()   // 每 10ms 底盘循迹控制
       │
       └─ ServoTestTask_2000ms()   // 每 2s 云台摆动
            │
            ├─ Servo_SetAngle(0.0f)
            ├─ Servo_SetAngle(90.0f)
            └─ Servo_SetAngle(-90.0f)
                 │
                 ├─ [UART 模式]
                 │    ├─ 计算 (deg - g_current) / DEG_PER_STEP → steps
                 │    ├─ fdCommand(dir, abs(steps), 0)
                 │    │    └─ uartSend(pkt, 13)
                 │    │         └─ DL_UART_Main_transmitData × 13
                 │    └─ g_current = deg (假设性更新)
                 │
                 └─ [PULSE 模式]
                      ├─ 计算 g_steps_remaining
                      ├─ g_status = SERVO_BUSY
                      └─ TIMER ISR → Servo_StepIRQ()
                           ├─ g_steps_remaining ±= 1
                           └─ g_current ±= DEG_PER_STEP
```

### 18.2 数据流图

```
                    Servo_SetAngle(deg)
                           │
                           ▼
                    ┌───────────────┐
                    │  g_target=deg │  (目标角度记录)
                    └───────┬───────┘
                            │
                ┌───────────┴───────────┐
                │                       │
         UART 模式                 PULSE 模式
                │                       │
                ▼                       ▼
     ┌──────────────────┐    ┌──────────────────┐
     │ steps = (deg -   │    │ total = deg /    │
     │   g_current)/    │    │   DEG_PER_STEP   │
     │   DEG_PER_STEP   │    │ g_steps_remaining│
     │ dir = ±1         │    │   = total - curr │
     └────────┬─────────┘    └────────┬─────────┘
              │                       │
              ▼                       ▼
     ┌──────────────────┐    ┌──────────────────┐
     │ fdCommand(dir,   │    │ g_status = BUSY  │
     │   steps, sync)   │    │ DIR pin set      │
     │ → uartSend(pkt)  │    │ TIMER start PWM  │
     └────────┬─────────┘    └────────┬─────────┘
              │                       │
              ▼                       ▼
     ┌──────────────────┐    ┌──────────────────┐
     │ g_current = deg  │    │ Servo_StepIRQ()  │
     │ (不等电机响应)    │    │ (TIMER 中断中)   │
     └──────────────────┘    │ g_steps--        │
                             │ g_current+=STEP  │
                             │ if (0) OK        │
                             └──────────────────┘

                         Servo_GetAngle()
                              │
                     ┌────────┴────────┐
                     │  UART 模式      │  PULSE 模式
                     ▼                 ▼
              ┌──────────────┐  ┌──────────────┐
              │ 发 0x36 指令 │  │ 直接返回      │
              │ 收 6 字节    │  │ g_current     │
              │ 解析 steps   │  └──────────────┘
              │ g_current =  │
              │   steps*DPS │
              └──────────────┘
```

---

## 19. SysConfig 外设配置要求

### 19.1 当 `SERIAL_CTRL_ENABLED = 1` 时

SysConfig 中 **必须** 配置以下内容:

| 配置项 | 值 | 在 SysConfig 中位置 |
|--------|-----|--------------------|
| UART 模块 | 启用 UART0 | Peripherals → UART |
| 波特率 | 115200 | UART0 → Configuration → Baud Rate |
| 字长 | 8 bits | UART0 → Configuration → Word Length |
| 校验 | None | UART0 → Configuration → Parity |
| 停止位 | 1 bit | UART0 → Configuration → Stop Bits |
| 方向 | TX + RX | UART0 → Configuration → Direction |
| 流控 | None | UART0 → Configuration → Flow Control |
| TX FIFO | 启用 | UART0 → Advanced → Enable FIFOs |
| RX FIFO | 启用 | UART0 → Advanced → Enable FIFOs |
| RX 引脚 | PA11 | UART0 → Pin Allocation |
| TX 引脚 | PA10 | UART0 → Pin Allocation |

SysConfig 生成的代码中关键配置:

```c
// 时钟配置: 使用 BUSCLK (32MHz)，不分频
static const DL_UART_Main_ClockConfig gUART_0ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

// 波特率分频值 (由 SysConfig 根据 32MHz 和 115200 自动计算)
// IBRD = 32MHz / (16 × 115200) = 17.36 → 取整 17
// FBRD = round(0.36 × 64) = 23
#define UART_0_IBRD_32_MHZ_115200_BAUD  (17)
#define UART_0_FBRD_32_MHZ_115200_BAUD  (23)

// 实际波特率 = 32MHz / (16 × (17 + 23/64)) = 115211.52
// 误差 = (115211.52 - 115200) / 115200 = 0.01%  < ±2%，合格
```

### 19.2 当 `PULSE_CTRL_ENABLED = 1` 时

需要额外配置:

| 配置项 | 值 | 说明 |
|--------|-----|------|
| TIMER 模块 | 选择任意 TIMER | 推荐 TIMER0 或 TIMER1 |
| TIMER 模式 | PWM 输出 | 产生 STEP 脉冲 |
| TIMER 频率 | 取决于最大转速 | 1500 RPM@3200步 = 80kHz 脉冲 |
| STEP 引脚 | 任意 GPIO | TIMER 输出引脚 |
| DIR 引脚 | 任意 GPIO | 普通 GPIO，非 TIMER 输出 |
| TIMER 更新中断 | 启用 | 触发 `Servo_StepIRQ` |

---

## 20. 调试手段

### 20.1 用示波器抓 UART 波形

**设置参数**:
- 通道: CH1 接 PA10 (TX)，CH2 接 PA11 (RX) 
- 地线: 接 GND (必须共地!)
- 电压刻度: 2V/div (TTL 是 0~3.3V)
- 时基: 100μs/div (115200 ≈ 8.68μs/bit)
- 触发: 下降沿触发 (UART 空闲时是高电平，起始位是低电平)

**观察内容**:
1. 是否真有数据发出（检查 TX 引脚是否有波形）
2. 波特率是否准确（测量一个 bit 的宽度，应为 8.68μs）
3. 帧格式是否正确（1 位起始位 + 8 位数据位 + 1 位停止位）
4. 电机是否回复（RX 引脚在发完指令后是否有数据回来）

### 20.2 用示波器抓 STEP 脉冲

**设置参数**:
- 通道: CH1 接 STEP 引脚
- 时基: 1ms/div (取决于速度，低速时可能需要 10ms/div)
- 触发: 上升沿

**观察内容**:
1. 脉冲频率是否与设定转速一致:
   ```
   1500 RPM × 3200 步/圈 ÷ 60 = 80,000 步/秒 = 80kHz
   脉冲周期 = 1/80000 = 12.5μs
   ```
2. 脉冲宽度是否满足电机要求 (通常 > 2μs)
3. 加减速过程中频率是否平滑变化

### 20.3 串口助手调试 0xFD 指令

用 USB 转 TTL 模块（如 CH340、CP2102）连接电机:

1. **接线**: 
   - TTL 模块 TX → 电机 RX
   - TTL 模块 RX → 电机 TX
   - GND → GND (**必须共地!**)

2. **发送 0xFD 位置指令** (十六进制发送):
   ```
   01 FD 01 05 DC 08 00 00 03 20 00 00 6B
   ```
   - 地址 0x01
   - 功能码 0xFD
   - 方向 0x01 (CW)
   - 速度 0x05DC = 1500 (注意: 单位可能是 0.1RPM)
   - 加速度 0x08
   - 目标位置 0x0320 = 800 微步 = 90°
   - 校验 0x6B

3. **观察电机响应**: 如果接线正确且协议对，电机应该转动 90°。
4. **发送 0x36 查询位置**:
   ```
   01 36 6B
   ```
   应收到 6 字节应答,如 `01 36 00 00 03 20 6B` 表示位置在 800 微步。

### 20.4 逻辑分析仪调试

比示波器更适合调试 UART 协议——可以一次性抓取完整数据帧并解码:
- 使用 Saleae Logic 或国产逻辑分析仪
- 设置 UART 解码器: 115200, 8N1
- 抓取 TX 和 RX 两条线
- 可以直观看到每次指令的完整 13 字节内容和电机应答

---

## 21. 常见踩坑点总结

### ⚠️ 踩坑 1: int16_t steps 溢出

```c
int16_t steps = (int16_t)((deg - g_current) / DEG_PER_STEP);
```

`int16_t` 的范围是 -32768~32767，对应约 ±3685°。如果角度差太大（比如从 -180° 到 180°，差 360°，需要 3200 步），没问题。但如果你做多圈连续旋转（云台不会），steps 会溢出。

**后果**: 假设 steps 真实值是 33000，截断后为 33000 - 65536 = -32536，电机虽然向正方向但是指令给了负方向数值导致反转。

### ⚠️ 踩坑 2: g_current 假设性更新导致位置漂移

UART 模式中 `g_current = deg;` 在调用后立即执行，不管电机是否真正到达目标。

**场景**: 连续调用 `Servo_SetAngle(90)` 和 `Servo_SetAngle(0)`，间隔 50ms。
- 第一帧指令发出，电机开始向 90° 运动（需要约 50ms 运动时间）
- 50ms 后第二帧指令发出，`g_current` 从 90 被设为 0
- 计算 `(0 - 90) / 0.1125 = -800` 步
- 电机收到指令时可能才走到 10°，收到 -800 步指令后反向

**结果**: 电机看起来"抖了一下"——先向前走一点，又回头。

### ⚠️ 踩坑 3: UART 发送过快导致 TX FIFO 溢出

`uartSend` 在循环中调用 `DL_UART_Main_transmitData`。MSPM0G3507 的 UART TX FIFO 深度为 4 字节。如果连续发送超过 4 字节而没有等待，DriverLib 内部会等待 FIFO 有空位——所以不会真的溢出，但会阻塞 CPU。

**后果**: 如果 `Servo_GetAngle` 在主循环中被频繁调用，且电机不回复导致 `uartRecv` 每次都超时，累积的 CPU 时间可能影响其他任务（如 10ms 的底盘控制）。

### ⚠️ 踩坑 4: PULSE 模式下 g_current 在中断中修改

```c
// interrupt context
g_current += (g_steps_remaining > 0) ? DEG_PER_STEP : -DEG_PER_STEP;
```

如果主循环也在调用 `Servo_GetAngle()` 读取 `g_current`，由于 `g_current` 是 `float` 类型（4 字节），读取它不是原子操作。

**后果**: 主循环可能在中断修改 `g_current` 的过程中读到**半更新**的值——即 4 个字节中 2 个字节是旧值、2 个字节是新值，导致完全错误的角度值。

**解决方案**: 在读取 `g_current` 时关中断，或使用 `volatile` + 原子访问。

### ⚠️ 踩坑 5: 电机未上电或地址不对

如果电机没有供电（步进电机需要 12~48V 电源，MCU 的 3.3V 只够串口通信不够驱动电机），`uartSend` 和 `uartRecv` 都不会报错——你只是发了数据但没回应。

**排查**: 
1. 用万用表测量电机电源端是否有电压
2. 如果有多台电机，确认 `SERVO_ADDR` 和电机上拨码开关/软件设置的地址一致
3. 将电机的 TX 和 RX 短接，用串口助手自发自收确认 TX 通路正常

### ⚠️ 踩坑 6: 波特率误差累积

MSPM0G3507 使用 BUSCLK = 32MHz。115200 波特率需要:
```
分频系数 = 32MHz / (16 × 115200) ≈ 17.36
整数部分 (IBRD) = 17
小数部分 (FBRD) = round(0.36 × 64) = 23
实际波特率 = 32MHz / (16 × (17 + 23/64)) = 115211.52
误差 = +0.01%
```

这个误差是单向的（实际比标称快 0.01%）。如果电机端的晶振也有误差（比如 ±1%），长期通信可能会累积成帧错误——具体表现为每隔几十到几百帧出现一次乱码。

**排查**: 长时间运行后观察电机是否出现无响应、错误抖动等现象。如果出现，改用 57600 或更低的波特率。

### ⚠️ 踩坑 7: `#if` 和 `#ifdef` 的误用

```c
#if SERIAL_CTRL_ENABLED    // 正确
#ifdef SERIAL_CTRL_ENABLED // 错误! 只要定义了就会编译，不管值是 0 还是 1
```

`#if` 检查**值**是否为真（非 0），`#ifdef` 只检查**是否定义了**这个宏。如果 `#define SERIAL_CTRL_ENABLED 0`，`#ifdef` 仍然会编译其中的代码！一定要用 `#if`。

---

## 附录: 速查表

### 波特率分频值 (32MHz BUSCLK)

| 目标波特率 | IBRD | FBRD | 实际波特率 | 误差 |
|-----------|------|------|-----------|------|
| 9600 | 208 | 21 | 9599.04 | -0.01% |
| 19200 | 104 | 11 | 19199.69 | -0.002% |
| 38400 | 52 | 5 | 38385.54 | -0.04% |
| 57600 | 34 | 47 | 57636.87 | +0.06% |
| 115200 | 17 | 23 | 115211.52 | +0.01% |
| 230400 | 8 | 50 | 230414.75 | +0.006% |

### DEG_PER_STEP 速查 (360°/步数)

| 细分 | 物理步数 | 微步/圈 | deg/step | 精度 |
|-----|---------|---------|----------|------|
| 1 | 200 | 200 | 1.8° | 粗 |
| 2 | 200 | 400 | 0.9° | |
| 4 | 200 | 800 | 0.45° | |
| 8 | 200 | 1600 | 0.225° | |
| 16 | 200 | 3200 | 0.1125° | ✓ 当前配置 |
| 32 | 200 | 6400 | 0.05625° | |
| 64 | 200 | 12800 | 0.028125° | 超细 |

### 角度 ↔ 步数 转换表

| 角度 | 步数 (取整) | 实际角度 | 误差 |
|------|------------|---------|------|
| 0.1125° | 1 | 0.1125° | 0 |
| 1° | 8 | 0.9° | -0.1° |
| 5° | 44 | 4.95° | -0.05° |
| 45° | 400 | 45.0° | 0 (巧合) |
| 90° | 800 | 90.0° | 0 (巧合) |
| 180° | 1600 | 180.0° | 0 (巧合) |
| 360° | 3200 | 360.0° | 0 (精确) |

注意: 45°、90°、180°、360° 之所以没有误差，是因为它们是 `0.1125` 的整数倍。而 1° 不是 `0.1125` 的整数倍，所以有误差。
