# uart_protocol 协议解析模块 — 超详细解读

> **README 定位**: 当你看代码旁边的注释都看不懂时来翻阅的百科全书
>
> **Target**: MSPM0G3507 @ 32MHz, Cortex-M0+, no RTOS
>
> **关键词**: UART 通信协议, 状态机, 帧解析, CRC8, 粘包处理, 裸机

---

## 1. 模块概述

uart_protocol 是一个**基于状态机(UART receive state machine)的逐字节协议解析器**。它负责从 UART 接收到的字节流中识别出完整的、符合预定义格式的数据帧, 并提取帧中的命令和载荷数据。

**为什么需要这个模块?** 串口(UART)是一个面向字节的异步通信接口, 它本身没有帧的概念。当你的 MCU 通过串口和上位机(PC 或另一块板子)通信时, 你需要自己定义"一帧数据从哪里开始、到哪里结束、中间包含什么信息"。这个文件就是做这个事的。

**核心架构:**
```
UART RX 中断(每收到一个字节)
    ↓
Protocol_ParseByte(byte)  →  状态机内部状态转换
    ↓
返回 1 表示收到完整有效帧
    ↓
主循环中调用 Protocol_GetPacket() 取出帧数据
```

> **为什么在中断中只做一个字节的解析?** 因为 UART 中断是非常频繁的(115200bps 下约 86µs 一个字节), 你不可能在一个中断里做复杂的事情。状态机的好处是: 每次中断只处理一个字节, 处理完就走。解析逻辑被"切碎"到每个字节的处理函数里了。

---

## 2. 帧格式定义

### 2.1 逐字节帧格式

自定义协议帧的完整布局如下(数字表示字节序号):

```
+--------+--------+--------+--------+--------+--------+--------+--------+
| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | ...... | Byte N | Byte N+1|
+--------+--------+--------+--------+--------+--------+--------+--------+
| 帧头   | 命令字 | 长度   | 数据 0 | 数据 1 | ...... | 数据 M | 校验   |
| HEAD   | CMD    | LEN    | DATA[0]| DATA[1]| ...... | DATA[M]| CHK    |
+--------+--------+--------+--------+--------+--------+--------+--------+
```

**实际传输顺序** (注意状态机接收顺序):

```
State: IDLE → HEADER → CMD → LEN → DATA(×N) → CHECKSUM → COMPLETE(等待帧尾)
字节:   HEAD     CMD    LEN   D0..Dn    CK      TAIL
```

> 注意代码中的状态机先经过 HEADER → CMD → LEN → DATA → CHECKSUM → COMPLETE(TAIL)。命名有点混: `PARSE_CHECKSUM` 接收的是校验字节, `PARSE_COMPLETE` 接收的是帧尾并做校验对比。

### 2.2 各字段说明

| 字段 | 长度 | 值/范围 | 说明 |
|------|------|---------|------|
| **帧头(HEAD)** | 1 字节 | `config.head_byte` (由用户定义, 如 0xAA) | 帧起始标志, 唯一标识一帧的开始 |
| **命令字(CMD)** | 1 字节 | 0x00~0xFF | 表示这一帧的"意图", 如 0x01=读取温度, 0x02=设置参数 |
| **长度(LEN)** | 1 字节 | 0~64(`PROTOCOL_MAX_PAYLOAD`) | 后面数据载荷的字节数。0 表示没有数据段 |
| **数据(DATA)** | M 字节 (M=LEN) | 任意 | 实际传输的数据内容, 最多 64 字节 |
| **校验(CK)** | 1 字节 | 由算法计算 | 对 CMD+LEN+DATA 计算得到的校验值 |
| **帧尾(TAIL)** | 1 字节 | `config.tail_byte` (由用户定义, 如 0x55) | 帧结束标志, 同时校验通过后最终确认帧有效 |

### 2.3 为什么选择这个帧头?

帧头的选择是一个看似简单但很重要的决策:

- **用 0xAA**: 二进制 `10101010`, 交替的 1/0 模式, 有良好的曼彻斯特编码特性, 不容易被噪声模拟
- **用 0x55**: `01010101`, 和 0xAA 互补
- **用 0x7E**: HDLC 协议的标志字节, 但需要位填充
- **用两个字节帧头(0xAA 0x55)**: 更可靠, 但多一个字节开销

本模块把帧头设计为可配置的, 是一个好的设计选择。通常建议 `head_byte = 0xAA`, `tail_byte = 0x55`, 或者反之。这两个字节在 UART(通常 LSB first)传输时波形接近方波, 容易用示波器识别, 而且在噪声环境中偶然出现的概率较低。

### 2.4 为什么用 XOR/CRC8 而不用累加和?

本模块支持三种校验方式, 加上一个"无校验"选项:

| 校验方式 | 代码值 | 强度 | 开销 | 适用场景 |
|----------|--------|------|------|---------|
| 无校验 | `CHECK_NONE` | ❌ 无 | 0 | 调试阶段, 或通信环境极好 |
| XOR 校验 | `CHECK_XOR` | 低 | 1 cycle/byte | 简单、常用, 能检测奇数个位翻转 |
| CRC8 | `CHECK_CRC8` | 中 | 8 cycles/byte (逐位法) | 工业通信, 可靠性要求高 |

**XOR 校验的弱点**: 如果两个字节的同一比特位同时出错(偶数个位翻转), XOR 检不出。比如 `0x55 XOR 0xAA = 0xFF`, 但如果都变成 `0x54 XOR 0xAB = 0xFF`, 校验和不变, 错误检测失败。

**CRC8 的优势**: CRC 是本原多项式在 GF(2) 上的除法余数, 它的检错能力是确定的:
- 所有单比特错误: 100% 检测
- 所有双比特错误: 100% 检测(如果多项式是本原的)
- 所有奇数位错误: 100% 检测
- 任意突发错误 ≤ 8 位: 100% 检测
- 更长突发错误: 检测率 > 99.6%

对于工厂产线或长距离通信场景, CRC8 是更好的选择。对于板间短距离通信, XOR 足够。

---

## 3. 帧格式表格(ASCII 画帧)

下面用 ASCII 画一个实际的帧例子。假设 `head=0xAA, tail=0x55, CMD=0x03, LEN=0x04, DATA=[0x10,0x20,0x30,0x40]`, 使用 XOR 校验:

```
字节索引:    [0]    [1]    [2]    [3]    [4]    [5]    [6]    [7]
          +------+------+------+------+------+------+------+------+
内容(HEX):| 0xAA | 0x03 | 0x04 | 0x10 | 0x20 | 0x30 | 0x40 | 0x?? |
          +------+------+------+------+------+------+------+------+
字段:      HEAD   CMD    LEN    D[0]   D[1]   D[2]   D[3]   CHK
                                                               
XOR 校验值 = 0x03 ^ 0x04 ^ 0x10 ^ 0x20 ^ 0x30 ^ 0x40 = 0x57
                               
完整帧: AA 03 04 10 20 30 40 57 55
                                         ^^
                                      校验字节  ^^ 帧尾
```

---

## 4. 核心数据结构

### 4.1 `ParseState_t` — 解析状态枚举

```c
typedef enum {
    PARSE_IDLE,       // 空闲, 等待帧头
    PARSE_HEADER,     // 收到帧头
    PARSE_CMD,        // 收到命令字
    PARSE_LEN,        // 收到长度
    PARSE_DATA,       // 正在接收数据载荷
    PARSE_CHECKSUM,   // 收到校验字节
    PARSE_COMPLETE    // 一帧解析完成
} ParseState_t;
```

| 状态 | 含义 | 到达条件 |
|------|------|---------|
| `PARSE_IDLE` | 初始/复位状态, 等待帧头 | 上电/超时/错误/帧结束后复位 |
| `PARSE_HEADER` | 已收到帧头, 等待命令字 | `byte == head_byte` |
| `PARSE_CMD` | 已收到命令字, 等待长度字节 | 上一个状态收到一个字节 |
| `PARSE_LEN` | 已收到长度, 等待数据字节 | 收到长度字节(且长度 > 0) |
| `PARSE_DATA` | 预留, 目前虽定义但未使用 | — |
| `PARSE_CHECKSUM` | 正在接收校验字节 | 数据接收完毕(或长度为 0 时直接由 CMD→CHECKSUM) |
| `PARSE_COMPLETE` | 收到校验字节, 等待帧尾确认 | 收到校验字节 |

> **注意**: 状态定义中有 `PARSE_DATA` 但 `ParseByte` 中没有对应的 case 处理。这是一个预留值, 可能作者计划将来在数据接收过程中需要做额外的处理(比如按协议转义), 但目前被 `PARSE_LEN` 兼用了(状态机在 `PARSE_LEN` 阶段直接接收所有载荷字节, 而不是切到 `PARSE_DATA`)。

---

### 4.2 `CheckType_t` — 校验类型枚举

```c
typedef enum {
    CHECK_NONE,     // 无校验
    CHECK_XOR,      // 异或校验
    CHECK_CRC8,     // CRC8 校验
    CHECK_CRC16     // CRC16 校验
} CheckType_t;
```

枚举值 0, 1, 2, 3, 可以用在 switch-case 中, 也可以作为数组索引。

> **CRC16 状态**: 代码中 CRC16 的 case 里有 `valid = 0;` (标记为无效), 说明 CRC16 尚未实现。这是一个"计划中"的功能, 框架已经搭好了, 只差填充 `crc16_update` 函数。

---

### 4.3 `ProtocolConfig_t` — 协议配置

```c
typedef struct {
    uint8_t  head_byte;      // 帧头字节
    uint8_t  tail_byte;      // 帧尾字节
    CheckType_t check_type;  // 校验类型
    uint16_t timeout_ms;     // 帧超时时间, ms
} ProtocolConfig_t;
```

这四个配置项定义了一种协议的"风格":
- `head_byte` 和 `tail_byte` 决定了帧边界
- `check_type` 决定了可靠性等级
- `timeout_ms` 决定了接收容错: 如果两个字节间隔超过这个时间, 认为当前帧已损坏, 自动复位

> **为什么超时时间很重要?** 考虑场景: 上位机发送了帧头 `0xAA` 后突然断电或线松了。如果没有超时机制, 解析器永远卡在 `PARSE_HEADER` 状态, 之后的任何字节都会被当作命令字处理, 永远无法重新同步。超时后自动复位到 `PARSE_IDLE`, 等下一次 `0xAA` 到来。

---

### 4.4 `Packet_t` — 解析完成的数据包

```c
typedef struct {
    uint8_t  cmd;                           // 命令字
    uint8_t  payload[PROTOCOL_MAX_PAYLOAD]; // 数据载荷
    uint16_t payload_len;                   // 载荷实际长度
    uint8_t  checksum;                      // 接收到的校验值
    uint8_t  valid;                         // 有效性标志
} Packet_t;
```

`valid` 是个关键标志: 不是所有到达 `PARSE_COMPLETE` 状态的数据包都是有效的。如果校验不匹配, 或者帧尾不对, `valid` 被设为 0, 调用端应该检查这个字段后再做处理。

> **为什么不用 `bool` 用 `uint8_t`?** 代码中是 `uint8_t` 而非 C99 的 `bool`, 可能是因为编译器的 C++ 兼容性或 `stdbool.h` 没有包含(但这个文件包含了 `<stdint.h>` 而不是 `<stdbool.h>`)。从功能上没有区别。

---

### 4.5 `Protocol_t` — 解析器运行时状态

```c
typedef struct {
    ProtocolConfig_t config;       // 协议配置
    ParseState_t state;            // 当前解析状态
    Packet_t     packet;           // 正在组装的包
    uint16_t data_index;           // 当前数据索引
    uint8_t  calc_checksum;        // 本地计算的校验值
    uint32_t last_byte_time;       // 上次收到字节时间戳
    uint32_t (*GetTick)(void);     // 获取时间戳的函数指针
} Protocol_t;
```

| 成员 | 用途 |
|------|------|
| `config` | 协议配置, 初始化后不再修改 |
| `state` | 当前状态机所处的状态, 决定下一个字节怎么处理 |
| `packet` | 正在组装的包, 解析完成后可被读取 |
| `data_index` | 已接收的载荷字节数, 也用来判断数据是否收完 |
| `calc_checksum` | 本地根据收到的 CMD+LEN+DATA 计算出的校验值, 在 COMPLETE 阶段与收到的 checksum 对比 |
| `last_byte_time` | 最新收到字节的时刻, 用于超时判断 |
| `GetTick` | 时间戳获取函数, 与 scheduler 共用同一个计数器 |

> **看到 `Sched_t` 的影子了吗?** `Protocol_t` 和 `Sched_t` 都用了 `GetTick` 函数指针。在同一个工程中, 两个模块可以共用同一个 `sys_tick_ms` 计数器, 互不冲突。

---

## 5. 内部函数解析

### 5.1 `crc8_update` — CRC8 单字节更新

```c
static uint8_t crc8_update(uint8_t crc, uint8_t data)
{
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (uint8_t)((crc << 1) ^ 0x31);
        } else {
            crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}
```

这个函数实现的是**逐位 CRC8 算法**(非查表法)。

多项式: `0x31` = `x^8 + x^5 + x^4 + 1` (左移形式, 即 CRC-8-CCITT 或者说是 CRC-8-Dallas/Maxim 使用的多项式)

计算过程:
1. `crc ^= data`: 新数据与当前 CRC 低 8 位异或
2. 循环 8 次(每个 bit 一次):
   - 如果最高位(bit 7)是 1: 左移 1 位然后异或多项式 `0x31`
   - 如果最高位是 0: 只左移 1 位

> **初始值问题**: 代码中调用 `crc8_update` 时初始值用的是 `0xFF` (见 `Protocol_ParseByte` 中 `PARSE_HEADER` 的 case)。这是 CRC-8-CCITT 的常见初始值。如果你把初始值改为 `0x00`, 计算结果会不同, 通信双方必须一致。

**为什么不查表?** 查表法(256 字节约 256 字节)更快但占 ROM。MSPM0G3507 有 256KB Flash, 其实不在乎这 256 字节。但作者选择逐位法可能是为了(1)代码更短、(2)不依赖查表数组(可重入性强)、(3)可读性更好。逐位法在 32MHz 下计算一个字节的 CRC8 约 8 × (3~4) = 24~32 cycles, ~1µs, 完全可接受。

---

## 6. API 逐行拆解

### 6.1 `Protocol_Init` — 初始化协议解析器

```c
void Protocol_Init(Protocol_t *p, const ProtocolConfig_t *cfg,
                   uint32_t (*get_tick)(void))
{
    if (p == NULL || cfg == NULL || get_tick == NULL) return;
    p->config          = *cfg;
    p->GetTick         = get_tick;
    p->state           = PARSE_IDLE;
    p->last_byte_time  = get_tick();
    Protocol_Reset(p);
}
```

4 步:
1. 参数校验: 三个指针缺一不可
2. 复制配置: `p->config = *cfg` (结构体整体赋值, 相当于 memcpy)
3. 初始状态: `PARSE_IDLE`, 等待帧头
4. 记录当前时间为"上次字节时间", 用于超时判断
5. 调用 `Protocol_Reset` 清空其他字段

> **为什么复制整个结构体而不是存指针 `cfg`?** 如果 `cfg` 是栈上的临时变量, `Protocol_Init` 返回后就被释放了。复制到内部保证配置在整个生命周期内有效。当然代价是多占 `sizeof(ProtocolConfig_t) = 4+1+1+2 = 8` 字节的 RAM。

---

### 6.2 `Protocol_ParseByte` — 逐字节解析(状态机核心)

这是整个模块最复杂的函数, 我们拆成几个块来分析。

```c
uint8_t Protocol_ParseByte(Protocol_t *p, uint8_t byte)
{
    if (p == NULL || p->GetTick == NULL) return 0;

    uint32_t now = p->GetTick();
```

**第 1 块(函数入口 + 时间戳获取):** 获取当前时间用于超时检测。`return 0` 表示"尚未收到完整有效帧"。

---

```c
    if (p->state != PARSE_IDLE) {
        uint32_t elapsed = now - p->last_byte_time;
        if (p->config.timeout_ms > 0 && elapsed >= p->config.timeout_ms) {
            Protocol_Reset(p);
            return 0;
        }
    }
    p->last_byte_time = now;
```

**第 2 块(字节间超时检测):**
- 只在非 `PARSE_IDLE` 状态检测超时。`PARSE_IDLE` 状态本来就没事干, 等多久都没关系, 不用计时。
- `timeout_ms > 0` 的判断让用户可以选择不启用超时功能(设为 0)。
- 超时后调用 `Protocol_Reset` 复位到 `PARSE_IDLE`, 丢弃当前所有已收到的字节。
- 更新 `last_byte_time` 为当前时刻。

> **超时保护的典型场景**: 上位机发了 `AA 03` 后卡住了(程序崩溃/线缆故障)。此时解析器卡在 `PARSE_CMD` 状态。500ms 后超时, 自动复位。如果没超时保护, 下一个正确帧的 `0xAA` 会被当作命令字处理, 完全错位。

---

```c
    switch (p->state) {
    case PARSE_IDLE:
        if (byte == p->config.head_byte) p->state = PARSE_HEADER;
        break;
```

**第 3 块(IDLE 状态):** 最简单的状态。当前字节等于帧头就切到 HEADER, 否则忽略。所有与帧头不匹配的字节都被丢弃。

---

```c
    case PARSE_HEADER:
        p->packet.cmd = byte;
        if (p->config.check_type == CHECK_XOR) {
            p->calc_checksum = byte;
        } else if (p->config.check_type == CHECK_CRC8) {
            p->calc_checksum = crc8_update(0xFF, byte);
        }
        p->state = PARSE_CMD;
        break;
```

**第 4 块(HEADER→CMD 转换):**

**关键问题**: 为什么这个状态叫 `PARSE_HEADER` 但实际接收的是命令字?

看状态机的设计: `PARSE_IDLE` 已经消耗了帧头(判断 `byte == head_byte`), 下一个字节到来时状态已经变成了 `PARSE_HEADER`。所以 `PARSE_HEADER` 实际上是"帧头后的第一个字节 = 命令字"。

> **命名误导**: 状态命名与直观理解不完全一致。更合理的命名可能是: `WAIT_HEAD` → `WAIT_CMD` → `WAIT_LEN` → `WAIT_DATA` → `WAIT_CHECK` → `WAIT_TAIL`。

校验初始化:
- XOR: `calc_checksum = byte`。XOR 的初始值就是命令字本身, 后续每收到一个字节做 `^= byte`。
- CRC8: `calc_checksum = crc8_update(0xFF, byte)`。CRC8 初始值通常为 0xFF(CRC-8-CCITT 的初始值), 然后更新命令字节。

---

```c
    case PARSE_CMD:
        p->packet.payload_len = byte;
        p->data_index = 0;
        if (byte == 0) {
            p->state = PARSE_CHECKSUM;
        } else {
            p->state = PARSE_LEN;
        }
        if (p->config.check_type == CHECK_XOR) {
            p->calc_checksum ^= byte;
        } else if (p->config.check_type == CHECK_CRC8) {
            p->calc_checksum = crc8_update(p->calc_checksum, byte);
        }
        break;
```

**第 5 块(CMD→LEN 转换):** 
- 把当前字节存为 `payload_len`。
- 如果长度=0: 没有数据段, 直接跳到 `PARSE_CHECKSUM`。
- 如果长度>0: 进入 `PARSE_LEN`, 开始接收数据。
- 长度字节也要参与校验计算(CMD+LEN+DATA 一起校验)。

---

```c
    case PARSE_LEN:
        if (p->data_index < PROTOCOL_MAX_PAYLOAD) {
            p->packet.payload[p->data_index] = byte;
        }
        p->data_index++;
        if (p->config.check_type == CHECK_XOR) {
            p->calc_checksum ^= byte;
        } else if (p->config.check_type == CHECK_CRC8) {
            p->calc_checksum = crc8_update(p->calc_checksum, byte);
        }
        if (p->data_index >= p->packet.payload_len) {
            p->state = PARSE_CHECKSUM;
        }
        break;
```

**第 6 块(LEN 数据接收):**

**防溢出保护**: `if (p->data_index < PROTOCOL_MAX_PAYLOAD)` 这行是防止恶意帧或错误帧的。如果对端发来 `LEN=255`, 而 `PROTOCOL_MAX_PAYLOAD=64`, 超出的字节被**丢弃**(不存储)但仍累加 `data_index` 和校验值。这样:
1. 缓冲区不会被写爆
2. 校验值仍然正确
3. 当收到的字节数达到 `payload_len` 时正确切换到校验状态

但是有个问题: **payload 被截断了但上层不知道**。如果 `payload_len` > 64, 上层拿到的 `payload_len` 仍然是 255(因为存在 `packet.payload_len` 里), 但实际有效数据只有前 64 字节。上层必须自行检查 `payload_len <= PROTOCOL_MAX_PAYLOAD`。

---

```c
    case PARSE_DATA:
        break;
```

**第 7 块(DATA 预留):** 空的 `case`, 什么都不做。如上所述, 是未来的扩展占位。

---

```c
    case PARSE_CHECKSUM:
        p->packet.checksum = byte;
        p->state = PARSE_COMPLETE;
        break;
```

**第 8 块(CHECKSUM 接收):** 收到校验字节, 存入 `packet.checksum`, 然后状态切到 `PARSE_COMPLETE`。这里还没有做校验, 只是收下这个字节。**校验在 COMPLETE 阶段进行**。

---

```c
    case PARSE_COMPLETE:
        if (byte == p->config.tail_byte) {
            uint8_t valid = 1;
            if (p->config.check_type == CHECK_XOR) {
                if (p->packet.checksum != p->calc_checksum) valid = 0;
            } else if (p->config.check_type == CHECK_CRC8) {
                if (p->packet.checksum != p->calc_checksum) valid = 0;
            } else if (p->config.check_type == CHECK_CRC16) {
                valid = 0;
            }
            p->packet.valid = valid;
            Protocol_Reset(p);
            return valid;
        }
        Protocol_Reset(p);
        return 0;
```

**第 9 块(COMPLETE 帧尾校验 — 决胜时刻):**

这是状态机的最后一步, 也是最关键的一步:
1. 收到帧尾字节: 说明对端认为这一帧已经结束了
2. 校验对比: 把收到的 `packet.checksum` 和本地计算的 `calc_checksum` 对比
3. 标记有效性: 匹配则 `valid = 1`, 否则 `valid = 0`
4. 复位状态机: 调用 `Protocol_Reset` 为下一帧做准备
5. 返回结果: 1 = 完整有效帧, 0 = 无效或仍在解析

**如果帧尾不匹配**: 整帧被认为是无效的, `return 0`, 不设置 `valid`。

> **为什么收到帧尾才校验?** 理论上你可以在收到校验字节时就做校验, 不用等帧尾。但帧尾提供了一层额外的保护: 校验字节正确但帧尾错误说明对端的发送逻辑有问题(或者这根本不是一帧, 是噪声恰好产生了正确的校验)。双重验证降低了误报率。

---

### 6.3 `Protocol_GetPacket` — 获取已解析的包

```c
uint8_t Protocol_GetPacket(Protocol_t *p, Packet_t *out)
{
    if (p == NULL || out == NULL) return 1;
    *out = p->packet;
    return 0;
}
```

简单结构体拷贝。调用者通过这个函数获取解析完成的帧。**注意**: 调用者应该先检查 `out->valid` 再处理数据。

---

### 6.4 `Protocol_Reset` — 复位解析器

```c
void Protocol_Reset(Protocol_t *p)
{
    if (p == NULL) return;
    p->state      = PARSE_IDLE;
    p->data_index = 0;
    p->calc_checksum = 0;
    p->packet.cmd         = 0;
    p->packet.payload_len = 0;
    p->packet.checksum    = 0;
    p->packet.valid       = 0;
}
```

复位所有解析中间状态为默认值, 但不修改 `config` 和 `GetTick`。这个函数在三种情况下被调用:
1. `Protocol_Init` 时: 初始化状态
2. `Protocol_ParseByte` 超时时: 丢弃当前帧
3. `Protocol_ParseByte` 帧尾不匹配时: 丢弃当前帧
4. `Protocol_ParseByte` 帧完成时: 准备接收下一帧

> **为什么不清空 `packet.payload`?** 不清除 payload 的旧数据是为了性能。如果调用者检查 `valid` 标志或 `payload_len`, 旧数据不会造成影响。重复的清零操作浪费 CPU。

---

## 7. 状态机转换图 (ASCII)

```
                        ┌─────────────────────────────────────────────┐
                        │                                             │
                        │         [任何字节 ≠ head]                   │
                        │               ↓                             │
                        │          ┌──────────┐                       │
            ┌───────────┤ 上电/复位 │  IDLE    │                       │
            │           │          └────┬─────┘                       │
            │           │               │ byte == head                │
            │           │               ↓                             │
            │           │          ┌──────────┐                       │
            │           │          │  HEADER  │ ← 这里实际收到的是CMD  │
            │           │          └────┬─────┘                       │
            │           │               │ 存CMD, 初始化校验           │
            │           │               ↓                             │
            │           │          ┌──────────┐                       │
            │           │          │    CMD   │ ← 这里实际收到的是LEN  │
            │           │          └────┬─────┘                       │
            │           │               │                             │
            │           │      ┌────────┴────────┐                    │
            │           │      │ LEN == 0        │ LEN > 0            │
            │           │      ↓                 ↓                    │
            │           │  ┌──────────┐    ┌──────────┐               │
            │           │  │CHECKSUM │    │   LEN    │ ← 收DATA      │
            │           │  └────┬─────┘    └────┬─────┘               │
            │           │       │               │ data_index >= len   │
            │           │       │               ↓                     │
            │           │       │          ┌──────────┐               │
            │           │       │          │CHECKSUM  │               │
            │           │       │          └────┬─────┘               │
            │           │       │               │ 收校验字节          │
            │           │       │               ↓                     │
            │           │       │          ┌──────────┐               │
            │           │       │          │ COMPLETE │               │
            │           │       │          └────┬─────┘               │
            │           │       │               │ byte == tail?       │
            │           │       │     ┌─────────┴─────────┐           │
            │           │       │     │ YES              │ NO        │
            │           │       │     ↓                  ↓            │
            │           │       │  valid=1/0       valid保留0         │
            │           │       │  返回 1           返回 0            │
            │           │       └─────┼───────────────┘               │
            │           │             │                                │
            │           └─────────────┘ (Protocol_Reset回到IDLE)      │
            │                                                          │
            │  [超时检测]: 任何非IDLE状态下字节间隔 > timeout_ms        │
            │             则 Protocol_Reset → 回到 IDLE               │
            └──────────────────────────────────────────────────────────┘
```

---

## 8. 状态转换说明表

| 当前状态 | 输入字节条件 | 动作 | 下一状态 |
|----------|-------------|------|---------|
| IDLE | `byte == head_byte` | 无 | HEADER |
| IDLE | `byte != head_byte` | 丢弃 | IDLE |
| HEADER | (任意) | 存 CMD, 初始化校验 | CMD |
| CMD | (任意) | 存 LEN, 更新校验 | LEN(如果 LEN>0) 或 CHECKSUM(如果 LEN==0) |
| LEN | (任意) | 存 DATA[data_index], 更新校验, data_index++ | LEN(未收完) 或 CHECKSUM(收完) |
| CHECKSUM | (任意) | 存 checksum | COMPLETE |
| COMPLETE | `byte == tail_byte` | 校验对比, 设valid | IDLE (复位) |
| COMPLETE | `byte != tail_byte` | 丢弃 | IDLE (复位) |
| **任意非IDLE** | **超时** | **丢弃所有已收数据** | **IDLE (复位)** |

---

## 9. 典型用法

```c
#include "uart_protocol.h"

// 协议配置
static const ProtocolConfig_t proto_cfg = {
    .head_byte   = 0xAA,
    .tail_byte   = 0x55,
    .check_type  = CHECK_XOR,
    .timeout_ms  = 100,   // 100ms 无数据视为超时
};

static Protocol_t proto;
static Packet_t   packet;

// 获取系统嘀嗒(与调度器共用)
extern volatile uint32_t sys_tick_ms;
uint32_t GetSysTick(void) {
    return sys_tick_ms;
}

// UART RX 中断服务函数
void UART_IRQHandler(void) {
    uint8_t byte;
    // 从 UART 数据寄存器读一个字节
    byte = UART->DR;
    
    // 逐字节送入解析器
    if (Protocol_ParseByte(&proto, byte)) {
        // 收到一帧完整有效的帧!
        // 注意: 中断中不能做复杂处理, 设置标志让主循环处理
        frame_ready = 1;
    }
}

int main(void) {
    Protocol_Init(&proto, &proto_cfg, GetSysTick);
    
    while (1) {
        if (frame_ready) {
            frame_ready = 0;
            Protocol_GetPacket(&proto, &packet);
            if (packet.valid) {
                ProcessCommand(packet.cmd, packet.payload, packet.payload_len);
            }
        }
        // 其他任务...
    }
}
```

---

## 10. 为什么用中断接收而不用 DMA?

MSPM0G3507 是 Cortex-M0+ 内核, DMA 支持情况:

| DMA 通道数 | 说明 |
|-----------|------|
| 最多 4 通道 | 多个外设共享 |
| 用于 ADC 采集、SPI 传输 | 通常优先分配给高速外设 |
| UART RX 使用 DMA 的收益 | 有限 |

**不用 DMA 的原因:**
1. **UART 速率低**: 115200bps 下, 86µs 一个字节, 中断完全可以承受。M0+ @ 32MHz 处理中断~20 cycles(~0.6µs), CPU 占用率 < 1%。
2. **DMA 通道稀缺**: 如果 DMA 通道给了 ADC 和 SPI, 就剩不下给 UART 的了。
3. **协议解析需要逐字节**: 即使 DMA 把数据搬到缓冲区, 最后还是需要 CPU 逐字节跑一遍状态机。DMA 省不了解析的 CPU 时间。
4. **帧长的可变性**: 协议帧长度不固定, DMA 需要知道"收多少字节", 而状态机天然适应变长帧。

如果 UART 速率是 921600bps 或更高(10.8µs/byte), 可以考虑 DMA + 双缓冲, 但这属于进阶优化, 当前场景不需要。

---

## 11. 踩坑点 (必读)

### 11.1 坑 1: 中断中不能做复杂解析

```c
// ❌ 错! 中断中不能做完整的帧处理和响应
void UART_IRQHandler(void) {
    Protocol_ParseByte(&proto, byte);  // ✅ 这个可以(一字节处理很快)
    ProcessCommand(...);               // ❌ 绝对不行, 可能在中断里卡太久
    SendResponse(...);                 // ❌ 发送也是阻塞的
}
```

`Protocol_ParseByte` 本身是很快的(几十个 cycle), 在中断里调用没问题。但**不要**在中断里处理命令、发送响应。正确做法是设一个标志, 在主循环中处理。

> 为什么? 如果 UART IRQ 优先级比 SysTick 高, 你在中断里处理了 5ms 的命令, SysTick 中断被阻塞, 调度器的时间基准就偏了。

### 11.2 坑 2: 粘包/半包处理

**粘包**: 上位机连续发送两帧 `AA 01 02 11 22 CK 55 AA 02 01 33 CK 55`
**半包**: 帧被拆成两次接收 `AA 01 02 11` 和 `22 CK 55`

这个状态机天然能处理这两种情况:
- **粘包**: 第一帧处理完后自动回到 IDLE, 第一个帧尾后的 `0xAA` 会被 IDLE 识别为新帧头
- **半包**: 状态机保持在半包状态等待剩余字节, 不超时就不复位

但是有一个极端情况: **粘包+帧头恰好出现在上一帧的校验字节位置**。假设 `head=0xAA`, 数据段里恰好也有 0xAA, 就会造成误同步。解决方案是用更长的帧头(比如双字节 0xAA 0x55), 或者在数据段中做字节填充(byte stuffing)。当前的设计是把帧头作为可配置项, 用户可以自己选一个数据段中不太可能出现的值。

### 11.3 坑 3: `payload_len` 伪装攻击/错误

如果对端发送 `LEN=255` 但实际只有 10 字节数据就停了:
- 状态机卡在 `PARSE_LEN`, 等待 255 个字节
- 直到超时(如果有配置)才复位
- 这期间所有后续发送的帧都会被当作 DATA 吃掉

**保护措施**: 超时机制。100ms 没收到下一个字节就复位, 不会永久死锁。

另外, 如前面所说, `payload_len > PROTOCOL_MAX_PAYLOAD` 时 payload 被截断但 `payload_len` 不修正, 调用者需要做:

```c
if (packet.payload_len > PROTOCOL_MAX_PAYLOAD) {
    packet.payload_len = PROTOCOL_MAX_PAYLOAD;  // 截断到安全值
}
```

### 11.4 坑 4: `Protocol_GetPacket` 的时序问题

```c
void UART_IRQHandler(void) {
    if (Protocol_ParseByte(&proto, byte)) {
        frame_ready = 1;
    }
}

int main(void) {
    while (1) {
        if (frame_ready) {
            frame_ready = 0;
            Protocol_GetPacket(&proto, &packet);  // 此时可能又来了一帧!
```

如果主循环在处理上一帧时, 中断又解析完成了一帧(极端高速通信), `packet` 会被覆盖。当前设计中 `Protocol_t` 只有一个 `Packet_t` 实例, **不支持多帧缓冲**。如果需要, 需要加一个队列(ring buffer)或双缓冲。

---

## 12. 调试手段

### 12.1 用串口助手发十六进制帧

最直接的调试方法:

1. 打开串口助手 (SSCOM, PuTTY, SecureCRT, 串口猎人等)
2. 选择 HEX 发送模式
3. 手动构造一帧: `AA 03 04 10 20 30 40 57 55`
4. 观察 MCU 是否有响应

可以故意制造错误帧来测试鲁棒性:
- **错误帧头**: `BB 03 04 ...` → 应该被忽略
- **校验错误**: `AA 03 04 10 20 30 40 00 55` → valid 应该为 0
- **超时测试**: 发 `AA 03` 然后等 200ms 再发后面的 → 应该超时复位
- **帧尾错误**: `AA 03 04 10 20 30 40 57 00` → 应该被丢弃

### 12.2 逻辑分析仪抓 UART 波形

用逻辑分析仪抓 TX 和 RX 引脚, 可以看到:
- 是否发了完整帧
- 字节间隔时间
- 帧间隔时间
- 是否有噪声导致错误字节

在 Saleae Logic 或 PulseView 中设置 UART 解码器, 可以直接看到解析结果, 和你的状态机输出对比。

### 12.3 调试打印状态机

在开发阶段, 可以在 `Protocol_ParseByte` 中添加调试输出(仅 DEBUG 版本):

```c
// 调试: 打印每个字节的状态转换
printf("UART: byte=0x%02X, state=%d\n", byte, p->state);
```

输出示例:
```
UART: byte=0xAA, state=0(IDLE)     → 收到帧头, 进入 HEADER
UART: byte=0x03, state=1(HEADER)   → 收到 CMD=03
UART: byte=0x04, state=2(CMD)      → 收到 LEN=04
UART: byte=0x10, state=3(LEN)      → 收到 DATA[0]
UART: byte=0x20, state=3(LEN)      → 收到 DATA[1]
UART: byte=0x30, state=3(LEN)      → 收到 DATA[2]
UART: byte=0x40, state=3(LEN)      → 收到 DATA[3]
UART: byte=0x57, state=5(CHECKSUM) → 收到 CK
UART: byte=0x55, state=6(COMPLETE) → 收到 TAIL, valid=1 返回 1!
```

---

## 13. 扩展可能性

这个协议解析器的设计留了一些扩展点:

| 扩展方向 | 需要改什么 | 难度 |
|---------|-----------|------|
| CRC16 | 实现 `crc16_update`, 在 CHECKSUM 和 COMPLETE 中添加 CRC16 分支 | 低 |
| 双帧头(0xAA 0x55) | 增加一个 `PARSE_HEADER2` 状态 | 低 |
| 字节转义(如 0x7E 用 0x7D 0x5E 转义) | 在 PARSE_LEN 中添加转义检测 | 中 |
| 多帧缓冲(ring buffer) | 增加循环队列存放多个已完成帧 | 中 |
| 自动回复 ACK | 解析完自动发一个确认字节 | 中 |
| 大数据包(>256 字节) | 把 LEN 字段扩展到 2 字节 | 中 |

---

> **最后的话**: 这个 uart_protocol 模块是典型的嵌入式串口通信实践——状态机驱动、逐字节解析、可配置的校验和帧格式。它不追求极致的性能(没有用查表法 CRC 或 DMA), 而是追求**确定性**和**可预测性**: 每个字节的处理时间是固定的, 状态转换是明确的, 异常(超时/粘包/帧尾错误)都被考虑到。理解了这个代码, 你就学会了嵌入式中最常用的协议解析范式——状态机。
