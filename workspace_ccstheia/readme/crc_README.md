# crc — CRC校验模块百科全书

> **README 定位**: 当你看代码旁边的注释都看不懂时来翻阅的百科全书。
> **目标平台**: MSPM0G3507 @ 32MHz, 可移植到任何 C 环境
> **核心思想**: 查表法 CRC8/CRC16, 用于通信帧校验, 零硬件依赖

---

## 目录

1. [模块概览](#1-模块概览)
2. [CRC8 查表 (多项式 0x31)](#2-crc8-查表-多项式-0x31)
3. [CRC16 查表 (多项式 0x8005)](#3-crc16-查表-多项式-0x8005)
4. [CRC8_Compute / CRC8_ComputeWithInit](#4-crc8_compute--crc8_computewithinit)
5. [CRC16_Compute / CRC16_ComputeWithInit](#5-crc16_compute--crc16_computewithinit)
6. [查表法 vs 计算法 深度对比](#6-查表法-vs-计算法-深度对比)
7. [模块间调用关系](#7-模块间调用关系)
8. [调试与验证](#8-调试与验证)
9. [常见踩坑点](#9-常见踩坑点)

---

## 1. 模块概览

```c
//CRC校验算法,CRC8和CRC16
#ifndef CRC_H
#define CRC_H

#include <stdint.h>

uint8_t  CRC8_Compute(const uint8_t *data, uint16_t len);
uint8_t  CRC8_ComputeWithInit(const uint8_t *data, uint16_t len,
                              uint8_t init);
uint16_t CRC16_Compute(const uint8_t *data, uint16_t len);
uint16_t CRC16_ComputeWithInit(const uint8_t *data, uint16_t len,
                               uint16_t init);

#endif
```

**这段代码在干什么**: CRC 校验模块的头文件声明, 提供 4 个接口函数。

**逐行解释**:
- `#include <stdint.h>`: 使用 `uint8_t`、`uint16_t`。CRC 运算涉及大量按位操作, 必须用无符号类型确保右移行为是逻辑移位 (补 0) 而非算术移位 (补符号位)。
- `CRC8_Compute`: 最常用接口, 初始值固定为 0xFF, 调用者只需传入数据和长度。适用于大多数 Modbus-RTU、SMBus、1-Wire 等协议的 CRC8。
- `CRC8_ComputeWithInit`: 允许自定义初始值。为什么需要这个? 当你要分段计算 CRC 时, 前一段的 CRC 结果作为后一段的 init 传入, 实现流式校验。
- 类似的, `CRC16_Compute` 和 `CRC16_ComputeWithInit` 提供相同功能的分段/非分段校验。

**为什么提供两个版本 (有 init / 无 init)**:
- 80% 的调用场景不需要自定义 init, 提供默认版本减少参数输入量。
- 20% 的场景需要分段校验 (比如校验很大一帧数据, 只能分批从 DMA 拿), 这时需要 `WithInit` 版本。
- 单函数用默认参数? C 语言不支持默认参数, 所以提供两个函数。`CRC8_Compute` 内部委托给 `CRC8_ComputeWithInit`, 没有代码重复。

---

## 2. CRC8 查表 (多项式 0x31)

### 2.1 什么是 CRC8

CRC8 是 8 位循环冗余校验, 生成 1 字节的校验值。核心数学是模 2 多项式除法。

**多项式 0x31 的二进制**: `0011 0001` → `x^8 + x^5 + x^4 + 1` (常数项 1 对应 x^0)。

0x31 是包含常数项的 8 位表示 (实际上正常的 CRC8 多项式是 9 位, 最高位 x^8 固定为 1, 但存储时省略最高位, 所以 0x31 实际代表 `1_0011_0001`, 即 x^8 + x^5 + x^4 + x^0)。

**这个多项式在哪用**:
- Dallas/Maxim 1-Wire 协议 (DS18B20 温度传感器)
- SMBus (System Management Bus)
- 某些 RFID/NFC 协议

### 2.2 查表数组定义

```c
//CRC8查表(多项式0x31),预计算256个值以提高运行速度
static const uint8_t CRC8_TABLE[256] = {
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97,
    0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
    ...
};
```

**这段代码在干什么**: 预计算的 CRC8 查表, 共 256 个条目 (索引 0~255)。

**为什么是 256 个条目**: CRC8 的每次运算输入是一个字节 (8 bit), 最多 256 种组合。查表法就是用空间换时间: 预先计算好每个可能的字节组合对应的 CRC 中间结果, 运行时用索引一次查表拿到结果, 避免逐位计算的循环。

**static const 的含义**:
- `static`: 限定该数组的链接作用域为当前编译单元 (即 crc.c 文件), 其他 .c 文件无法通过 `extern` 访问。这是信息隐藏, 防止调用者直接操作查表。
- `const`: 将表放在 ROM/Flash, 而非 RAM。对于 MSPM0G3507, Flash 有 128KB, RAM 只有 32KB。将表放在 Flash 减少 RAM 占用。`const` 在 MSPM0 上链接到 `.rodata` 段, 不占 `.bss` 或 `.data`。
- 大小: 256 × 1 byte = 256 bytes Flash。这是非常小的代价 (0.2% 的 128KB Flash)。

**CRC8_TABLE 是如何生成的**:
由于本模块直接写了硬编码的常量数组, 所以它是用外部工具预先生成的 (如 pycrc、crcmod 等 Python 库)。生成算法如下 (伪代码):

```
for idx = 0 to 255:
    crc = idx
    for bit = 0 to 7:
        if crc & 0x80:
            crc = (crc << 1) ^ 0x31
        else:
            crc = crc << 1
        crc = crc & 0xFF    // 保持 8 位
    CRC8_TABLE[idx] = crc
```

注意: 这个生成过程是逐位计算。运行时查表不再需要内部循环, 每个字节仅需 1 次查表 + 1 次异或, 速度提升约 8 倍。

---

## 3. CRC16 查表 (多项式 0x8005)

### 3.1 什么是 CRC16

CRC16 是 16 位循环冗余校验, 生成 2 字节校验值。

**多项式 0x8005 的二进制**: `1000 0000 0000 0101` → `x^16 + x^15 + x^2 + 1`

0x8005 省略了最高位的 x^16, 实际多项式是 `1_1000_0000_0000_0101`。

**这个多项式在哪用**:
- MODBUS-RTU: 使用多项式 0x8005, 初始值 0xFFFF (不是本模块的 0x0000, 需要注意!)
- USB: 使用多项式 0x8005 (USB 的 CRC16 用于令牌包)
- XMODEM: 使用多项式 0x1021 (不是 0x8005, 注意区分)

### 3.2 查表数组定义

```c
//CRC16查表(多项式0x8005),预计算256个值以提高运行速度
static const uint16_t CRC16_TABLE[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    ...
};
```

**这段代码在干什么**: 预计算的 CRC16 查表, 每个条目 16 位, 共 256 个条目。

**大小计算**: 256 × 2 bytes = 512 bytes Flash。加上 CRC8 的 256 bytes, 总共 768 bytes Flash 用于查表。对于 128KB Flash, 占比 0.6%, 非常划算。

**CRC16_TABLE 的生成算法**:

```
for idx = 0 to 255:
    crc = idx << 8    // CRC16 的查表索引是高 8 位
    for bit = 0 to 7:
        if crc & 0x8000:
            crc = (crc << 1) ^ 0x8005
        else:
            crc = crc << 1
        crc = crc & 0xFFFF
    CRC16_TABLE[idx] = crc
```

注意 CRC16 查表与 CRC8 查表的关键区别:
- CRC8: 索引 = `crc ^ data[i]` (8 位索引)
- CRC16: 索引 = `(crc >> 8) ^ data[i]` (高 8 位异或后做 8 位索引)
- 这是由 CRC16 的位移位数决定的: CRC8 每次移 8 位, 所以整字节做索引; CRC16 每次移 8 位但寄存器是 16 位, 所以高 8 位 xor 后做索引。

---

## 4. CRC8_Compute / CRC8_ComputeWithInit

### 4.1 CRC8_Compute (默认初始值)

```c
uint8_t CRC8_Compute(const uint8_t *data, uint16_t len)
{
    return CRC8_ComputeWithInit(data, len, 0xFF);
}
```

**这段代码在干什么**: CRC8 计算的默认版本, 初始值固定为 0xFF。

**为什么初始值是 0xFF 而不是 0x00**:
- 如果初始值为 0x00, 并且数据开头有一串 0x00, 那么 CRC 值仍为 0x00, 无法检测出开头的零字节丢失。初始值 0xFF 确保 CRC 寄存器初始不全零, 能够检测出前导零的增删。
- 在 1-Wire 协议、SMBus 中, CRC8 初始值均为 0xFF。
- 如果调用者的协议要求初始值 0x00 (如某些自定义协议), 请使用 `CRC8_ComputeWithInit(data, len, 0x00)`。

### 4.2 CRC8_ComputeWithInit (自定义初始值)

```c
uint8_t CRC8_ComputeWithInit(const uint8_t *data, uint16_t len,
                             uint8_t init)
{
    if (data == NULL || len == 0) return init;
    uint8_t crc = init;
    for (uint16_t i = 0; i < len; i++) {
        crc = CRC8_TABLE[crc ^ data[i]];
    }
    return crc;
}
```

**逐行解释**:
- `if (data == NULL || len == 0) return init;`: 健壮性检查。如果传入空指针或零长度, 不处理直接返回初始值。这是防御性编程。需要注意: 调用者如果传 NULL 且 len > 0, 下一步 `data[i]` 会访问空指针, 触发 HardFault。所以 NULL 检查必须配合 len 检查: `len == 0` 是安全出口, 但 `data == NULL && len > 0` 仍然会崩溃。这是权衡: 多一层 `if (data == NULL)` 后立即返回初值, 比再嵌套 `if (len > 0)` 检查更简洁, 尽管不完全安全。
- `uint8_t crc = init;`: 初始化 CRC 寄存器。init 参数让调用者实现流式校验。
- `for (uint16_t i = 0; i < len; i++)`: 遍历每个字节。`uint16_t` 作为循环变量, 最大 65535。本模块支持最长 65535 字节的校验。如果数据更长, 需要外部分包。
- `crc = CRC8_TABLE[crc ^ data[i]];`: 核心查表操作。
  - `crc ^ data[i]`: 当前 CRC 寄存器高/低 8 位与数据字节异或, 作为查表索引 (0~255)。
  - `CRC8_TABLE[...]`: 用索引查表拿到预计算的 CRC 结果。
  - 赋值回 crc, 进入下一轮迭代。
- `return crc;`: 返回最终 CRC8 值。

**CRC8 查表法的时间复杂度**: O(n), 每个字节仅需: 1 次异或 + 1 次内存读取 + 1 次赋值。在 32MHz 的 MSPM0 上, 每字节约 3~5 个 CPU 周期, 512 字节约 1.5K~2.5K 周期 = 46~78 μs, 非常快。

**流式 CRC8 的例子**:

```c
// 分段校验 1024 字节数据帧
uint8_t crc = 0xFF;                         // 初始值
crc = CRC8_ComputeWithInit(part1, 512, crc);  // 处理前半段, crc 作为 init
crc = CRC8_ComputeWithInit(part2, 512, crc);  // 处理后半段
// 最终 crc 等于 CRC8_Compute(full, 1024)
```

---

## 5. CRC16_Compute / CRC16_ComputeWithInit

### 5.1 CRC16_Compute (默认初始值)

```c
uint16_t CRC16_Compute(const uint8_t *data, uint16_t len)
{
    return CRC16_ComputeWithInit(data, len, 0x0000);
}
```

**这段代码在干什么**: CRC16 计算的默认版本, 初始值为 0x0000。

**为什么初始值是 0x0000**: 本模块面向的是初始值为 0x0000 的协议族。如果你需要 MODBUS 协议 (初始值 0xFFFF), 必须调用 `CRC16_ComputeWithInit(data, len, 0xFFFF)`。

### 5.2 CRC16_ComputeWithInit (自定义初始值)

```c
uint16_t CRC16_ComputeWithInit(const uint8_t *data, uint16_t len,
                               uint16_t init)
{
    if (data == NULL || len == 0) return init;
    uint16_t crc = init;
    for (uint16_t i = 0; i < len; i++) {
        uint16_t idx = ((crc >> 8) ^ data[i]) & 0xFF;
        crc = (uint16_t)((crc << 8) ^ CRC16_TABLE[idx]);
    }
    return crc;
}
```

**逐行解释**:
- `uint16_t crc = init;`: CRC 寄存器是 16 位。
- `uint16_t idx = ((crc >> 8) ^ data[i]) & 0xFF;`:
  - `crc >> 8`: 取 CRC 寄存器的高 8 位 (低 8 位在后面处理)。
  - `^ data[i]`: 异或当前数据字节。
  - `& 0xFF`: 截断为 8 位索引, 确保 idx ∈ [0, 255]。这一步可以省略, 因为 uint16_t 按 8 位掩码后隐式截断, 但显式写出来更清晰。
- `crc = (uint16_t)((crc << 8) ^ CRC16_TABLE[idx]);`:
  - `crc << 8`: 左移 8 位, 低 8 位被移到高 8 位, 低 8 位补 0。原 CRC 的低 8 位现在在高 8 位位置, 等待被异或。
  - `^ CRC16_TABLE[idx]`: 异或查表结果。查表结果已经包含了多项式对输入字节的全部"处理", 无需再做逐位移位。
  - `(uint16_t)(...)`: 防止隐式整数提升。C 语言中 `uint16_t` 进行算数运算时会被提升为 `int` (32 位), `crc << 8` 可能是 32 位中间结果, 不截断会导致高位污染。
- `return crc;` 返回最终 CRC16 值。

**CRC16 查表 vs CRC8 查表的性能差异**:
- CRC8: 每字节 1 次查表
- CRC16: 每字节 1 次查表 + 1 次 16 位移位 + 1 次 16 位异或
- 在 MSPM0G3507 上, CRC16 每字节约 6~8 个 CPU 周期, 比 CRC8 多一倍, 但仍然是 O(n) 的优秀性能。

---

## 6. 查表法 vs 计算法 深度对比

### 6.1 计算法 (逐位法, bit-by-bit)

```c
// 计算法 CRC8 示例 (未在模块中使用, 仅用于对比)
uint8_t CRC8_BitByBit(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0xFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}
```

**计算法的特点**:
- 优点: 无查表占用 Flash (省 256 bytes)。适合 Flash 非常紧张的超低端 MCU。
- 缺点: 每个字节需要 8 次内循环, 每字节约 80~120 个 CPU 周期。512 字节约 41K~61K 周期 = 1.3~1.9 ms。（@32MHz）
- 选择依据: 如果 Flash 余量 < 1KB, 用计算法; 否则用查表法。

### 6.2 本模块选择查表法的原因

MSPM0G3507 的 Flash 为 128KB, 代码量通常 < 32KB, 768 bytes 的 CRC 查表完全装得下。查表法以 768 bytes Flash 换取 ~8 倍速度提升, 在 32MHz 上通信帧处理是决定性的。采用查表法是合理的。

### 6.3 半字节查表法

还有一种折中方案: 半字节查表 (4 bit 索引, 16 个条目):
- CRC8: 16 × 1 = 16 bytes
- CRC16: 16 × 2 = 32 bytes
- 速度: 每字节 2 次查表
- 在本模块没有采用, 因为 Flash 够用, 用全字节查表最简单。

---

## 7. 模块间调用关系

```c
//crc.c
#include "crc.h"
#include <stddef.h>
```

**这段代码在干什么**: crc.c 只包含自身头文件和 stddef.h。

**被谁调用** (推测, 基于项目上下文):
- 通信协议模块: 假设有 `uart_protocol.c`, 发送数据帧时计算 CRC 作为帧尾; 接收时校验 CRC, 不匹配则丢弃帧。
- I2C 传感器驱动: 某些传感器 (如 BME280、MPU6050 等) 的数据手册可能提供 CRC 校验, 需要调用 CRC8 校验读到的寄存器值。
- OTA 升级模块: 接收固件包时校验每包数据的 CRC16, 确保传输无错误。
- Bootloader: 校验固件镜像的完整性。

**不依赖任何硬件外设**: CRC 计算是纯软件实现, 不依赖 MCU 的硬件 CRC 模块 (MSPM0 系列部分型号有硬件 CRC, 但本模块没使用, 保证可移植性)。

---

## 8. 调试与验证

### 8.1 使用在线 CRC 计算器验证

CRC 最方便的调试方式是用在线计算器:

1. 打开 https://www.sunshine2k.de/coding/javascript/crc/crc_js.html
2. 设置 CRC8: 多项式 0x31, 初始值 0xFF, 最终异或 0x00, 输入输出不反转
3. 输入测试数据, 比较计算结果

**测试数据 1**: 数据 `0x01` → CRC8 = ?
手动计算: crc = 0xFF, crc ^ 0x01 = 0xFE, CRC8_TABLE[0xFE] = ?
查表: CRC8_TABLE[0xFE] = 查上面数组第 0xFE 项 (索引 254) = 0x... 需要查表或在线计算。

**测试数据 2**: 使用 Python 验证 (推荐)
```python
# pip install crcmod
import crcmod

crc8_fn = crcmod.predefined.mkPredefinedCrcFun('crc-8')
result = crc8_fn(b'\x01\x02\x03')
print(hex(result))

crc16_fn = crcmod.mkCrcFun(0x18005, initCrc=0x0000, xorOut=0x0000)
result = crc16_fn(b'\x01\x02\x03')
print(hex(result))
```

**调试步骤**:
1. 在 PC 上用 Python/crcmod 计算已知数据序列的 CRC
2. 在 MSPM0 上烧录工程, 用同一个数据序列调用 CRC 函数
3. 对比结果, 一致则模块正确

### 8.2 硬件验证方法

如果手头有逻辑分析仪或示波器, 可以抓取通信波形:
1. 发送方发送数据 + CRC
2. 接收方计算 CRC
3. 对比发送和接收的 CRC 值

### 8.3 单元测试用例

```c
// CRC8 测试用例
uint8_t test_data1[] = {0x00};
assert(CRC8_Compute(test_data1, 1) == 0xC4);  // 具体值请通过在线计算器确认

uint8_t test_data2[] = {0x01, 0x02, 0x03};
uint8_t crc8_result = CRC8_Compute(test_data2, 3);

// CRC16 测试用例
uint16_t crc16_result = CRC16_Compute(test_data1, 1);  // 数据 {0x00}
// 在线计算器验证

// 分段校验测试
uint8_t crc = 0xFF;
crc = CRC8_ComputeWithInit(test_data2, 1, crc);   // 第一字节
crc = CRC8_ComputeWithInit(test_data2 + 1, 2, crc); // 后两字节
// crc 应等于 CRC8_Compute(test_data2, 3)
```

**注意**: 上面的断言值 (0xC4) 需要在验证后才能填入, 别直接复制使用。

---

## 9. 常见踩坑点

### 9.1 多项式不互通 (CRITICAL)

CRC 的最核心踩坑点: **不同厂家/协议使用不同多项式、初始值、最终异或值、输入输出反转配置, 结果完全不同**。

| 协议 | CRC 类型 | 多项式 | 初始值 | 最终异或 | 输入反转 | 输出反转 |
|------|---------|--------|--------|----------|---------|---------|
| 本模块 CRC8 | CRC-8/Dallas | 0x31 | 0xFF | 0x00 | 否 | 否 |
| 本模块 CRC16 | CRC-16/IBM | 0x8005 | 0x0000 | 0x0000 | 否 | 否 |
| MODBUS-RTU | CRC-16/MODBUS | 0x8005 | 0xFFFF | 0x0000 | 是 | 是 |
| CRC-16/CCITT | CRC-16/CCITT | 0x1021 | 0x0000 | 0x0000 | 是 | 是 |
| CRC-16/DNP | CRC-16/DNP | 0x3D65 | 0x0000 | 0xFFFF | 是 | 是 |

**问题实例**: 你的设备用 MODBUS 与 PLC 通信。如果直接用本模块的 `CRC16_Compute(data, len)`, PLC 会一直报 CRC 错误。因为 MODBUS 要求:
- 初始值 0xFFFF (不是 0x0000)
- 输入输出位反转 (reflected)
- 最终异或 0x0000

本模块当前不支持输入输出反转, 所以不能直接用于 MODBUS。解决方法:
- 方案一: 修改本模块, 增加反转配置
- 方案二: 在调用前对数据做位反转, 调用后输出再做位反转
- 方案三: 写一个专门用于 MODBUS 的 CRC16 函数

### 9.2 数据指针对齐和字节序

CRC 计算假设数据是逐字节处理的, 不关心字节序。但如果数据中包含多字节整数 (uint16_t, uint32_t), 发送方和接收方的字节序必须一致。

举例: 发送方需要校验一个 uint32_t 值 `0x12345678`, 以大端字节序发送 (`12 34 56 78`)。如果接收方用小端序解释并装入 uint32_t 再转字节数组, 得到 `78 56 34 12`, CRC 必然不匹配。

**正确做法**: 发送和接收双方约定一致的多字节序列化方式 (通常用大端序/网络字节序), 逐字节送入 CRC 计算。

### 9.3 初始值 0x0000 的全零数据问题

如果用 `CRC16_Compute(data, len)` (初始值 0x0000) 校验全零数据:
- CRC16_Compute({0x00, 0x00, 0x00}, 3) 的结果 = 0x0000
- CRC16_Compute({0x00, 0x00}, 3) 的结果 = 0x0000 (注意长度不同但 CRC 相同!)
- 这意味着初始值 0x0000 的 CRC16 无法区分不同长度的全零序列

这就是为什么很多协议使用 0xFFFF 或 0x0001 等非零初始值的原因。

### 9.4 指针为 NULL 但长度不为 0 的崩溃路径

回顾 `CRC8_ComputeWithInit`:

```c
if (data == NULL || len == 0) return init;
```

如果调用者写出这样的代码:
```c
uint8_t *p = NULL;
uint16_t crc = CRC8_ComputeWithInit(p, 10, 0xFF);
```

条件判断: `data == NULL` → true → 直接 `return init` (0xFF), 不会访问 NULL 指针。这是安全的。

但如果调用者写出:
```c
uint8_t *p = NULL;
uint16_t crc = CRC8_ComputeWithInit(p, 0, 0xFF);  // len = 0
```

`data == NULL` → true → 返回 0xFF。但 len = 0 时计算本来就没有意义, 返回 init 是合理的。

如果调用者:
```c
uint8_t buf[4] = {1,2,3,4};
uint16_t crc = CRC8_ComputeWithInit(buf, 0, 0xFF);  // len = 0, buf 有效
```

`data == NULL` → false, `len == 0` → true → 返回 0xFF。数据被忽略, 但 len = 0 时确实没有数据要处理, 合理。

### 9.5 const 限定符与 Flash 定位

CRC8_TABLE 和 CRC16_TABLE 声明为 `static const`, 确保它们链接到 Flash (`.rodata`)。但有一个陷阱: 在默认编译选项下, `const` 变量仍然会占用 RAM 副本 (如果有初始化代码将其从 Flash 复制到 RAM)。MSPM0 的 TI 编译器默认 `.rodata` 不复制到 RAM, 直接从 Flash 寻址。但如果你用某些 ARM-GCC 的默认链接脚本, `.rodata` 可能被放在 RAM 中。

**检查方法**: 查看生成的 `.map` 文件, 搜索 CRC8_TABLE, 确认它在 ROM 区还是 RAM 区。

### 9.6 最终异或 (XOROUT) 未实现

标准 CRC 规范中有一步"最终异或": 计算完成后, CRC 值与一个固定值异或后输出。很多协议 (如 CRC-8/MAXIM) 的最终异或值为 0x00 (等同无操作), 但有些协议 (如 CRC-8/ITU) 最终异或 0x55。

本模块没有实现最终异或, 因为本模块面向的协议不需要。如果需要, 调用者在返回值上自行异或:

```c
uint8_t crc = CRC8_Compute(data, len) ^ 0x55;  // 手动应用 XOROUT
```

### 9.7 输入反转 (Reflected Input/Output)

标准 CRC 有 RefIn 和 RefOut 参数:
- RefIn = True: 每个输入字节在送入计算前按位反转 (LSB first)
- RefOut = True: 最终 CRC 值按位反转后输出

本模块不支持反射。因为本模块面向的不需要反转的协议 (如 Dallas 1-Wire)。如果需要支持 MODBUS 等反转协议, 需要引入位反转:

```c
// 8 位反转查表 (也可以不用查表, 但为了速度)
static const uint8_t REVERSE8_TABLE[256] = { /* ... */ };

// 或者在 CRC 主循环中处理反射:
uint8_t reved = 0;
for (int b = 0; b < 8; b++) {
    reved = (reved << 1) | ((data[i] >> b) & 1);
}
// 然后用 reved 代替 data[i] 参与运算
```

---

> 本文档是针对 crc 模块的逐行级百科全书, 所有解释基于 MSPM0G3507 裸机环境。如有疑问, 以实际硬件调试结果为准。
