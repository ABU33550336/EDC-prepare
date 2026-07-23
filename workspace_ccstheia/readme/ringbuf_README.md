# ringbuf — 环形缓冲区模块百科全书

> **README 定位**: 当你看代码旁边的注释都看不懂时来翻阅的百科全书。
> **目标平台**: MSPM0G3507 @ 32MHz, 可移植到任何 C 环境
> **核心思想**: 无锁 FIFO 环形队列, 用于 UART/SPI 等异步接收, 零动态内存分配

---

## 目录

1. [模块概览](#1-模块概览)
2. [RingBuf_t 结构体详解](#2-ringbuf_t-结构体详解)
3. [RingBuf_Init — 初始化](#3-ringbuf_init--初始化)
4. [RingBuf_IsEmpty / RingBuf_IsFull — 空满判断](#4-ringbuf_isempty--ringbuf_isfull--空满判断)
5. [RingBuf_Count / RingBuf_Free — 计数和剩余空间](#5-ringbuf_count--ringbuf_free--计数和剩余空间)
6. [RingBuf_Put — 单字节写入](#6-ringbuf_put--单字节写入)
7. [RingBuf_Get — 单字节读取](#7-ringbuf_get--单字节读取)
8. [RingBuf_PutArray / RingBuf_GetArray — 批量读写](#8-ringbuf_putarray--ringbuf_getarray--批量读写)
9. [RingBuf_Reset — 重置](#9-ringbuf_reset--重置)
10. [满/空判断方案深度对比 (浪费一元素 vs 计数器法 vs 本模块)](#10-满空判断方案深度对比-浪费一元素-vs-计数器法-vs-本模块)
11. [模块间调用关系](#11-模块间调用关系)
12. [调试与验证](#12-调试与验证)
13. [常见踩坑点](#13-常见踩坑点)

---

## 1. 模块概览

```c
//环形缓冲区,无锁FIFO实现
#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdint.h>
#include <stdbool.h>

// ... (结构体定义 + 函数声明)

#endif
```

**这段代码在干什么**: 头文件保护, 包含标准定长类型头文件。

**解释**:
- 与 math_utils.h 同样的头文件保护策略。
- `stdint.h` + `stdbool.h` 在嵌入式 C 中是标配, 本模块所有函数都返回 `uint8_t` 或 `bool`。
- 函数返回 `uint8_t` 作为状态码: 0 = 成功, 1 = 失败。节省 RAM (相比返回 int), 且语义清晰。

**设计哲学**:
- **零动态内存分配**: 缓冲区由调用者提供外部数组, ringbuf 只持有一个指针。为什么? 嵌入式系统禁止 `malloc`/`free`, 因为: (1) 堆碎片化 — 长时间运行后可用内存碎片, 无法分配大块; (2) 分配时间不确定 — `malloc` 内部可能遍历空闲链表, 在 ISR 中调用可能导致时序违背; (3) MSPM0G3507 RAM 只有 32KB, 堆空间小, 碎片更致命。
- **无锁**: 单生产者 + 单消费者 (SPSC) 场景下, head 和 tail 分别只被一方修改, 不需要互斥锁或关中断。但中断与主循环的共享访问仍需 volatile。

---

## 2. RingBuf_t 结构体详解

```c
typedef struct {
    uint8_t *buffer;    //数据缓冲区指针
    uint16_t size;      //缓冲区容量
    uint16_t head;      //写指针
    uint16_t tail;      //读指针
    bool     full;      //满标志,区分空和满状态
} RingBuf_t;
```

**这段代码在干什么**: 定义环形缓冲区的数据结构。

**逐字段解释**:

- `uint8_t *buffer`: 指向外部数据缓冲区的指针。缓冲区由调用者分配 (可以是全局数组、静态数组或局部数组)。ringbuf 不拥有这个内存, 只使用它。

- `uint16_t size`: 缓冲区的容量 (字节数)。为什么是 `uint16_t` 而不是 `uint32_t` 或 `uint8_t`?
  - MSPM0G3507 RAM 32KB, UART 接收 buffer 通常 64~1024 字节, `uint16_t` (最大 65535) 足够。
  - `uint8_t` 最大 255, 很多场景不够用。
  - `uint32_t` 浪费 RAM (每个 RingBuf_t 多 2 字节), 且 32 位运算在 32MHz Cortex-M0+ 上比 16 位多指令。

- `uint16_t head`: 读指针 (Consumer 端)。指向下一个待读取数据的位置。当 head == tail 且 full == false, 缓冲区为空。
  - **命名注意**: 这里 head 是读指针 (数据流出端), tail 是写指针 (数据流入端)。一些教科书和网络实现用 head 表示写、tail 表示读, 方向相反。本模块的命名来自"队列头部是数据出口", head 指向最早的数据。读代码时千万别被命名搞混了。
  
- `uint16_t tail`: 写指针 (Producer 端)。指向下一个空闲位置, 新数据写入此位置后 tail 递增。
  
- `bool full`: 满标志。当 tail 递增后追上 head, 说明缓冲区已满, full = true。为什么不用 `head == tail` 判断满?
  - 因为 `head == tail` 同时也代表空。没有 full 标志, 空和满无法区分。
  - 标准解决方案有两个: (1) 浪费一个元素, tail + 1 == head 才算满; (2) 用计数器记录元素个数。本模块使用 full 标志法 — 下面第三节「满/空判断方案深度对比」详细分析。

**结构体的内存布局** (假设 32-bit ARM, 默认 4 字节对齐):
- `*buffer`: 4 字节 (指针在 32 位系统上占 4 字节)
- `size`: 2 字节 (`uint16_t`, 2 字节)
- `head`: 2 字节
- `tail`: 2 字节
- `full`: 1 字节 (`bool` 在 C 中实际占 1 字节)
- padding: 1 字节 (结构体对齐到 4 字节边界)
- 总大小: 4 + 2 + 2 + 2 + 1 + 1 = 12 字节

---

## 3. RingBuf_Init — 初始化

```c
uint8_t RingBuf_Init(RingBuf_t *rb, uint8_t *buf, uint16_t size)
{
    if (rb == NULL || buf == NULL || size == 0) return 1;
    rb->buffer = buf;
    rb->size   = size;
    rb->head   = 0;
    rb->tail   = 0;
    rb->full   = false;
    return 0;
}
```

**这段代码在干什么**: 初始化环形缓冲区, 将 head、tail 置零, full 置 false。

**逐行解释**:
- `if (rb == NULL || buf == NULL || size == 0) return 1;`: 三重校验。rb 或 buf 为空指针, 或者 size==0, 直接返回失败码 1。防御性编程的底线: 空指针访问必定 HardFault。
- `rb->buffer = buf;`: 保存外部缓冲区地址。
- `rb->size = size;`: 保存容量。注意: size 是"可用容量", 不包含任何隐藏的预留字节。与"浪费一个元素"方案不同, 本模块的 size 就是实际可存储的数据字节数。
- `rb->head = 0; rb->tail = 0;`: 读写指针归零。起始状态下, 读指针和写指针重合, 缓冲区为空。
- `rb->full = false;`: 满标志初始化。`head == tail && full == false` 表示空。
- `return 0;`: 成功, 返回 0。

**为什么只赋值而不清空 buffer 内容**:
- 初始化时 buffer 中的旧数据不需要清除, 因为 head=tail=0, 数据被"逻辑删除"了。
- 如果清零 buffer, 需要 O(n) 的 memset 开销, 对于大 buffer 不可接受。

**调用示例**:
```c
#define UART_BUF_SIZE 256
static uint8_t uart_rx_buf[UART_BUF_SIZE];  // 静态分配, 不占堆
static RingBuf_t uart_rb;

void uart_init(void) {
    RingBuf_Init(&uart_rb, uart_rx_buf, UART_BUF_SIZE);
}
```

---

## 4. RingBuf_IsEmpty / RingBuf_IsFull — 空满判断

### 4.1 IsEmpty

```c
bool RingBuf_IsEmpty(RingBuf_t *rb)
{
    if (rb == NULL) return true;
    return (rb->head == rb->tail) && !rb->full;
}
```

**这段代码在干什么**: 判断缓冲区是否为空。

**逻辑推导**:
- 空的条件: `head == tail` (读写指针重合) **且** 不是满的。
- 因为满的时候也是 `head == tail` (写指针追上读指针), 但 full == true。
- 所以 `!full` 是区分空和满的关键。

**为什么 NULL 检查返回 true**: rb 为 NULL 时缓冲区不存在, 认为"空"是安全的行为 — 读取操作会因为空而返回失败; 写入操作会因为空指针解引用而崩溃 (但写入前有单独的 rb->full 检查, 不会走到写入 buffer 的步骤, 因为 NULL 指针的 `rb->full` 本身也是非法访问... 这里确实有隐患。见踩坑点 13.2)。

### 4.2 IsFull

```c
bool RingBuf_IsFull(RingBuf_t *rb)
{
    if (rb == NULL) return true;
    return rb->full;
}
```

**这段代码在干什么**: 判断缓冲区是否已满。

**解释**:
- 直接返回 `rb->full` 字段, O(1) 操作, 无需任何计算。
- 对比"浪费一元素"方案: 满的判断是 `(tail + 1) % size == head`, 需要一次取模运算。
- 对比"计数器"方案: 满的判断是 `count == size`, 需要一次比较。
- full 标志法最快: 只读一个 bool。

**为什么 NULL 返回 true**: 同样出于安全考虑, 认为不存在的缓冲区已满, 阻止写入。

---

## 5. RingBuf_Count / RingBuf_Free — 计数和剩余空间

### 5.1 RingBuf_Count

```c
uint16_t RingBuf_Count(RingBuf_t *rb)
{
    if (rb == NULL) return 0;
    if (rb->full) return rb->size;
    if (rb->tail >= rb->head) {
        return rb->tail - rb->head;
    }
    return rb->size - (rb->head - rb->tail);
}
```

**这段代码在干什么**: 计算缓冲区中当前有效数据的字节数。

**三种情况分析**:
1. `if (rb->full) return rb->size;`: 如果缓冲区满, 有效数据 = 总容量。注意这里比较耗费: 每次 Count 都需要检查 full 标志。
2. `if (rb->tail >= rb->head)`: 写指针在读指针之后或相同位置, 即没有发生回绕 (wrap-around)。
   - 没满时 `tail >= head` 且 tail - head 就是有效数据数。
   - 举例: head = 10, tail = 25, Count = 25 - 10 = 15
3. `return rb->size - (rb->head - rb->tail);`: 写指针回绕, 在读指针之前。
   - head - tail: 读指针超过写指针的距离。
   - size - (head - tail): 总容量减去"读指针领先的距离" = 有效数据。
   - 举例: size = 256, head = 200, tail = 50 → Count = 256 - (200 - 50) = 106

**为什么不直接用 `(tail - head + size) % size`**:
- 取模运算在无硬件除法器的 Cortex-M0+ 上非常慢 (需要软件除法库, 约 30~50 周期)。
- 用条件分支替代取模, 仅需 2~3 个分支, 快得多。
- 这是嵌入式代码优化的经典手法: 用分支替除法/取模。

### 5.2 RingBuf_Free

```c
uint16_t RingBuf_Free(RingBuf_t *rb)
{
    if (rb == NULL) return 0;
    return rb->size - RingBuf_Count(rb);
}
```

**这段代码在干什么**: 计算剩余空闲空间。

**逻辑**: size - Count = 空闲字节数。简单直接。

**时间复杂度**: O(1), 委托给 RingBuf_Count。

---

## 6. RingBuf_Put — 单字节写入

```c
uint8_t RingBuf_Put(RingBuf_t *rb, uint8_t data)
{
    if (rb == NULL) return 1;
    if (rb->full) return 1;
    rb->buffer[rb->tail] = data;
    rb->tail++;
    if (rb->tail >= rb->size) rb->tail = 0;
    if (rb->tail == rb->head) rb->full = true;
    return 0;
}
```

**这段代码在干什么**: 向缓冲区写入一个字节。

**逐行解释**:
- `if (rb == NULL) return 1;`: 参数校验。
- `if (rb->full) return 1;`: 缓冲区已满, 拒绝写入, 返回失败。这是"非覆盖模式" — 满时新数据被丢弃, 旧数据保留。

  > **为什么不支持覆盖模式**: 在一些实时场景中, 新数据比旧数据重要 (如传感器最新读数)。但 UART 接收场景下, 丢失旧数据可能导致协议帧不完整。本模块选择非覆盖模式, 因为丢失数据总比损坏数据好。如果调用者需要覆盖模式, 可以在满时先 `RingBuf_Get` 丢弃一个旧数据再 `RingBuf_Put`。

- `rb->buffer[rb->tail] = data;`: 将数据写入当前 tail 位置。
- `rb->tail++;`: 写指针递增。
- `if (rb->tail >= rb->size) rb->tail = 0;`: 回绕处理。如果 tail 超过容量上限, 绕回 0。这里用 `>=` 而非 `==`, 即使 size 为非法值 0 (虽然 Init 已拦截), 也能回绕到 0, 避免越界访问。

  > **回绕操作的替代方案**: 另一种实现是 `rb->tail = (rb->tail + 1) % rb->size;`。但前面说过, 取模在 Cortex-M0+ 上慢。分支法更快。

- `if (rb->tail == rb->head) rb->full = true;`: 写指针递增后追上读指针, 说明缓冲区被填满了, 设置 full = true。注意: 这个判断在回绕之后, 所以回绕后 tail==head 同样能正确设满标志。

**Put 操作的完整性**: Put 操作不是原子的。在单生产者单消费者场景下:
- tail 只被生产者 (中断或主循环写入方) 修改, head 只被消费者 (主循环读取方) 修改
- 如果生产者在 ISR 中, 消费者在主循环中:
  - 生产者可能在 `rb->tail++` 后被中断, 消费者读到新的 tail 值, 然后 `RingBuf_Count` 算出错误的 Count
  - 解决方案: 在生产者 ISR 中, 写入 buffer 和更新 tail 之间必须有内存屏障 (ARM 上的 `__DMB()` 或 `__DSB()`)
  - 对于 MSPM0G3507 的单核 Cortex-M0+, 没有缓存一致性问题, 编译器 barrier (`volatile`) 足够

---

## 7. RingBuf_Get — 单字节读取

```c
uint8_t RingBuf_Get(RingBuf_t *rb, uint8_t *data)
{
    if (rb == NULL || data == NULL) return 1;
    if (RingBuf_IsEmpty(rb)) return 1;
    *data = rb->buffer[rb->head];
    rb->head++;
    if (rb->head >= rb->size) rb->head = 0;
    rb->full = false;
    return 0;
}
```

**这段代码在干什么**: 从缓冲区读取一个字节。

**逐行解释**:
- `if (rb == NULL || data == NULL) return 1;`: 双重参数校验。data 是输出参数, 需要写入, NULL 指针写入导致 HardFault。
- `if (RingBuf_IsEmpty(rb)) return 1;`: 缓冲区空, 无数据可读, 返回失败。注意这里调用了 RingBuf_IsEmpty 而非直接检查条件。这一层函数调用在 ISR 中可能有点贵, 但可读性好。
- `*data = rb->buffer[rb->head];`: 从 head 位置读取一个字节, 存入 data 指向的变量。
- `rb->head++;`: 读指针递增。
- `if (rb->head >= rb->size) rb->head = 0;`: 回绕处理。
- `rb->full = false;`: 一旦读取了一个字节, 缓冲区一定不是满的。因为"满"的条件是 tail 追上 head (tail == head)。读取后 head 移动, 两者不再相等, 所以 full = false。

  > **这里有一个隐含假设**: 即使缓冲区在读取后仍然达到了完全占满的状态 (比如 size=1 的极端情况), Get 操作移除了一个元素, 所以 full 必须清 false。但如果同时有其他线程在 Put, full 的状态可能会在 Get 返回后立即再次变为 true。这没问题, 因为 `rb->full = false` 只是"此刻"的状态。

**Get 的阻塞/非阻塞行为**:
- 本模块的 Get 是**非阻塞**的: 空时立刻返回 1, 不会等待数据。
- 调用者需要轮询 `RingBuf_Count` 或 `!RingBuf_IsEmpty` 来决定是否 Get。
- 如果调用者需要阻塞等待, 应该在外部加信号量或事件标志。

**为什么 Get 不从 `full` 标志推导**:
- 有人可能会问: 既然 head 移动后一定不是满的, 直接设 `full = false` 会不会把真正满的缓冲区错误设为非满?
- 答案: 不会。在 Get 执行的那一刻:
  1. 如果缓冲区是满的 (full == true, tail == head), Get 从 head 读一个字节, head++ 后 head ≠ tail, full = false ✓
  2. 如果缓冲区非满, Get 后 full 本来就应该是 false, 再写一次 false 也没问题 ✓

---

## 8. RingBuf_PutArray / RingBuf_GetArray — 批量读写

### 8.1 RingBuf_PutArray

```c
uint8_t RingBuf_PutArray(RingBuf_t *rb, const uint8_t *src, uint16_t len)
{
    if (rb == NULL || src == NULL) return 1;
    for (uint16_t i = 0; i < len; i++) {
        if (RingBuf_Put(rb, src[i]) != 0) return 1;
    }
    return 0;
}
```

**这段代码在干什么**: 批量写入多个字节。

**优点**:
- 简单, 复用 RingBuf_Put, 不需要重复实现回绕逻辑。

**缺点**:
- 逐个调用 RingBuf_Put, 每次都有函数调用开销 (参数压栈 + 分支 + 返回)。
- 没有利用批量写入优化: 如果数据跨过 tail 回绕边界, 可以一次性 memcpy 两个分段, 但当前实现是逐字节写入。
- 性能瓶颈: len=100 时需要 100 次函数调用, 在 32MHz 上大约 3000~5000 周期 (90~150μs)。对于 ISR 来说可能过大了。建议高频 ISR 中不用 PutArray, 用逐字节 Put 或直接操作内部指针 (需小心)。

**为什么不优化成 memcpy**: 如果要优化, 可以把写入路径分两段:
1. 从 tail 到 size-1 写入 min(len, size-tail) 字节
2. 回绕后从 0 写到 min(len, head) 字节
但当前实现追求简洁而非极致性能。

### 8.2 RingBuf_GetArray

```c
uint16_t RingBuf_GetArray(RingBuf_t *rb, uint8_t *dst, uint16_t len)
{
    if (rb == NULL || dst == NULL) return 0;
    uint16_t read_cnt = 0;
    for (uint16_t i = 0; i < len; i++) {
        if (RingBuf_Get(rb, &dst[i]) != 0) break;
        read_cnt++;
    }
    return read_cnt;
}
```

**这段代码在干什么**: 批量读取多个字节。

**与 PutArray 的区别**:
- 返回值是实际读取的字节数 (可能小于 len), 而不是 0/1 状态。
- 空时自动停止, 不会阻塞等待。
- 调用者根据返回值判断读到了多少数据。

**为什么 PutArray 返回 0/1, 而 GetArray 返回实际数**:
- PutArray 是全有或全无: 只要有一个字节写不进 (缓冲区满), 整个操作失败。因为部分写入会导致协议帧不完整。
- GetArray 是尽力而为: 能读多少读多少, 调用者根据返回值判断读了多少。这在 UART 接收中很常见: 主循环轮询一次, 能读多少处理多少。

---

## 9. RingBuf_Reset — 重置

```c
void RingBuf_Reset(RingBuf_t *rb)
{
    if (rb == NULL) return;
    rb->head = 0;
    rb->tail = 0;
    rb->full = false;
}
```

**这段代码在干什么**: 清空缓冲区, 丢弃所有数据。

**解释**:
- 不需要清空 buffer 内容, 逻辑清空即可。
- 调用场景:
  - 通信协议发现一帧数据错误, 重置缓冲区, 丢弃所有待处理数据。
  - MCU 从低功耗模式唤醒, 不确定数据是否有效, 直接 Reset。
  - 重新配置通信接口时。

**为什么不加 volatile**: 见踩坑点 13.1。

---

## 10. 满/空判断方案深度对比 (浪费一元素 vs 计数器法 vs 本模块)

### 10.1 方案一: 浪费一个元素 (Classic)

```c
// 伪代码
#define RING_SIZE 256
uint8_t buffer[RING_SIZE];  // 但实际只能存 255 字节!
uint16_t head = 0, tail = 0;

bool isEmpty() { return head == tail; }
bool isFull()  { return (tail + 1) % RING_SIZE == head; }
uint16_t count() {
    if (tail >= head) return tail - head;
    return RING_SIZE - (head - tail);
}
```

**优点**:
- 不需要额外的 full 标志, 结构体更小。
- 经典, 网上教程大部分是这个方案。

**缺点**:
- 缓冲区有效容量 = size - 1。如果你要 256 字节缓冲区, 必须定义 257 字节数组。这很反直觉, 初学者经常忘了预留这一个元素。
- `isFull()` 需要取模, 慢 (除非 size 是 2 的幂, 用 `& (size-1)` 替代)。
- `count()` 的最大值是 RING_SIZE - 1, 永远达不到 RING_SIZE。

### 10.2 方案二: 计数器法

```c
typedef struct {
    uint8_t *buffer;
    uint16_t size;
    uint16_t head;
    uint16_t tail;
    uint16_t count;  // 当前元素个数
} RingBuf_t;

bool isEmpty() { return count == 0; }
bool isFull()  { return count == size; }
// Put: buffer[tail] = data; tail = (tail + 1) % size; count++;
// Get: data = buffer[head]; head = (head + 1) % size; count--;
```

**优点**:
- 空/满判断直接基于 count, 逻辑极其清晰。
- `count()` 直接返回 count 字段, O(1) 简单。

**缺点**:
- 多一个 count 字段 (2 bytes)。
- Put 和 Get 都需要修改 count。在生产者-消费者模型中, count 被两端同时修改, 需要原子操作或互斥保护。
- 如果生产者在 ISR 中执行 `count++`, 消费者在主循环检查 `count == 0`, 可能读到 count 的半修改值 (在 8/16 位 MCU 上, count 的递增可能是非原子的)。

### 10.3 方案三: 本模块的 full 标志法

```c
typedef struct {
    uint8_t *buffer;
    uint16_t size;
    uint16_t head;
    uint16_t tail;
    bool     full;
} RingBuf_t;
```

**优点**:
- **有效容量 = size** (不浪费元素)。
- `isFull()` 只需读一个 bool 字段, 极快。
- `isEmpty()` 只需一次比较 + 一个 bool 检查。
- 结构体只比"浪费一元素"多 1 字节 (bool)。

**缺点**:
- `count()` 需要分支处理回绕, 比其他方案略复杂。
- `Put` 或 `Get` 时需要维护 full 标志, 多了赋值操作。

### 10.4 为什么 full 标志法最适合本场景

| 特性 | 浪费一元素 | 计数器法 | full 标志法 (本模块) |
|------|-----------|---------|-------------------|
| 有效容量 | size - 1 | size | size |
| isFull 复杂度 | 取模 | 读 count | 读 bool |
| isEmpty 复杂度 | 读 head==tail | 读 count==0 | head==tail && !full |
| Count 复杂度 | 分支(无取模) | 读字段 O(1) | 分支 |
| 原子性问题 | head/tail 分离, 好 | count 竞态, 差 | head/tail 分离, 好 |
| 额外存储 | 0 (但浪费 1 元素) | +2 bytes | +1 byte (bool) |

本模块选择 full 标志法, 在 UART 中断接收 + 主循环轮询的典型场景中, 生产者和消费者对 head/tail 的修改是分离的, 不需要原子 count 字段, 避免了方案二的竞态问题。同时不浪费元素, 对于 RAM 仅 32KB 的 MSPM0G3507 来说, 每字节都珍惜。

---

## 11. 模块间调用关系

```c
//ringbuf.c
#include "ringbuf.h"
#include <stddef.h>
```

**这段代码在干什么**: 只包含自身头文件和 stddef.h。

**被谁调用**:
- **UART 驱动**: 最典型调用者。UART 中断服务函数中调用 `RingBuf_Put` 保存接收到的字节; 主循环调用 `RingBuf_Get` / `RingBuf_GetArray` 读取数据并解析协议帧。
- **SPI 驱动**: 类似 UART, SPI 从机接收数据时, 将数据存入 ringbuf, 主循环轮询处理。
- **I2C 从机**: I2C 从机接收到的寄存器读写请求可以通过 ringbuf 传递。
- **日志系统**: 不同模块产生的日志字符串放入 ringbuf, 后台任务定期 flush 到 UART 或 Flash。

**典型调用链**:

```
UART_IRQHandler()
  ├── UART 读取 DR 寄存器得到 1 字节
  ├── RingBuf_Put(&uart_rb, byte)    ← 生产者
  └── 退出中断

main_loop()
  ├── RingBuf_Count(&uart_rb)        ← 检查是否有数据
  ├── RingBuf_GetArray(&uart_rb, buf, 128)  ← 消费者
  └── parse_protocol_frame(buf, len) ← 解析协议帧
```

---

## 12. 调试与验证

### 12.1 设置小缓冲区故意测试溢出

调试环形缓冲区最有效的方法:

```c
#define TEST_SIZE 4  // 故意设小
uint8_t test_buf[TEST_SIZE];
RingBuf_t rb;

RingBuf_Init(&rb, test_buf, TEST_SIZE);

// 测试 1: 写入超过容量的数据
RingBuf_Put(&rb, 'A');  // OK
RingBuf_Put(&rb, 'B');  // OK
RingBuf_Put(&rb, 'C');  // OK
RingBuf_Put(&rb, 'D');  // OK
RingBuf_Put(&rb, 'E');  // 满! 返回 1, D 被保留
assert(RingBuf_IsFull(&rb) == true);

// 测试 2: 回绕测试
uint8_t ch;
RingBuf_Get(&rb, &ch);  // 读出 'A', head=1, full=false
RingBuf_Put(&rb, 'E');  // tail=0 (回绕), 写指针追上...
                        // tail(0) == head(1)? 不! head=1,tail=0, 不同
                        // 所以 full=false, 正常写入
RingBuf_Put(&rb, 'F');  // tail=1, tail==head, full=true
assert(RingBuf_IsFull(&rb) == true);

// 测试 3: 空读取
RingBuf_Reset(&rb);
assert(RingBuf_IsEmpty(&rb) == true);
assert(RingBuf_Get(&rb, &ch) == 1);  // 返回 1 (失败)
```

### 12.2 用串口助手 + 断点测试

1. 用串口助手发送固定长度数据包 (如 10 字节)
2. 在 `RingBuf_Put` 和 `RingBuf_Get` 设断点
3. 观察 head、tail、full 的变化是否符合预期
4. 回绕临界点 (tail == head-1 时) 加断点, 验证 full 标志正确设置

### 12.3 使用逻辑分析仪测量 ISR 耗时

如果 UART 波特率很高 (如 115200), 每 86.8μs 产生一次中断。确保 RingBuf_Put 的执行时间远小于这个间隔:
- 调用 RingBuf_Put 约 15~20 CPU 周期 → @32MHz ≈ 0.5~0.6μs
- 远小于 86.8μs, 不会丢数据

### 12.4 常见的自测问题

```c
// 问题 1: 满+空交替循环 65535 次
RingBuf_Init(&rb, test_buf, 16);
for (int i = 0; i < 65535; i++) {
    for (int j = 0; j < 16; j++) RingBuf_Put(&rb, j);
    assert(RingBuf_IsFull(&rb));
    for (int j = 0; j < 16; j++) RingBuf_Get(&rb, &ch);
    assert(RingBuf_IsEmpty(&rb));
}
// 测完后看 head/tail 计数值是否正确

// 问题 2: 分区写入 + 回绕
RingBuf_Init(&rb, test_buf, 8);
// 写入 4 字节, 读取 2 字节, 再写 6 字节 (要回绕)
RingBuf_PutArray(&rb, "ABCD", 4);  // head=0,tail=4
RingBuf_GetArray(&rb, buf, 2);     // head=2,tail=4
RingBuf_PutArray(&rb, "123456", 6); // tail=4→...→tail=2 (回绕), full=true
assert(RingBuf_Count(&rb) == 8);    // 满
```

---

## 13. 常见踩坑点

### 13.1 volatile 缺失 — 中断与主循环共享变量 (CRITICAL)

**问题**: 如果 `RingBuf_t` 的实例在 ISR (中断服务函数) 和 main loop 之间共享, 编译器可能会优化掉对 head/tail 的读取, 因为编译器看不到这些变量在中断中被修改。

**症状**: 主循环中 `RingBuf_IsEmpty(&rb)` 一直返回 true, 即使 ISR 已经写入了数据。

**根因**: C 编译器不知道 ISR 的存在。编译器看到 main loop 中没有修改 head/tail, 就把 `rb->tail` 缓存到寄存器, 不再重新从内存读取。

**解决方案** (当前代码没有做, 是潜在问题):
```c
// 在结构体定义中, head 和 tail 应加 volatile
typedef struct {
    uint8_t *buffer;
    uint16_t size;
    volatile uint16_t head;   // ← 消费者读指针, 被生产者(ISR)修改
    volatile uint16_t tail;   // ← 生产者写指针, 被消费者修改? 不...
    bool     full;
} RingBuf_t;
```

等等, 重新梳理:
- head 是读指针, 由消费者 (通常是 main loop) 修改, 被生产者 (ISR) 读取 (对比 `rb->tail == rb->head`)
- tail 是写指针, 由生产者 (ISR) 修改, 被消费者 (main loop) 读取

所以应该:
- `volatile uint16_t head`: 生产者读它, 但只有消费者写它 — 生产者读到过时的 head 值也不危险 (最多错误地认为满了或没满), 一般没问题。
- `volatile uint16_t tail`: 消费者读它, 只有生产者写它 — 这是最重要的! 没有 volatile, 消费者可能永远看不到 ISR 写入的新 tail 值。

但当前代码没有加 volatile。这是因为:
1. 如果编译器优化等级为 `-O0` 或 `-Og`, 不会缓存变量, 问题不出现。
2. 如果开启 `-O2` 但没有 `-ffast-math` 等激进优化, volatile 缺失会导致上述 bug。

**建议**: 在所有共享变量上加 volatile 是最安全的做法。但要注意, volatile 会阻止编译器优化, 略微增加代码尺寸。这里是一个 design trade-off: 当前模块注释称"无锁FIFO", 实际上没有 volatile 的"无锁"是不完整的。

### 13.2 NULL 指针解引用的隐患

RingBuf_IsEmpty 和 RingBuf_IsFull 对 NULL 做了检查并返回安全值:
```c
if (rb == NULL) return true;
```

但 `RingBuf_Count` 对 NULL 返回 0:
```c
if (rb == NULL) return 0;
```

但最致命的: **RingBuf_Put 在检查 rb->full 之前没有 null 检查 rb->buffer**。事实上, `RingBuf_Put` 中:
```c
if (rb == NULL) return 1;     // 检查了
if (rb->full) return 1;        // 如果 rb 非空, 这里访问 rb->full OK
rb->buffer[rb->tail] = data;  // 如果 rb->buffer 为 NULL, 这里崩溃!
```

如果调用者用 NULL buffer 初始化了 ringbuf (或初始化时传入 NULL 但 RingBuf_Init 返回 1 后继续使用), 在 Put 时写入 `NULL[tail]` 就是解引用空指针。

**改进建议**: 在 Put 和 Get 中增加 `rb->buffer == NULL` 检查, 但会增加代码体积。当前方案假设调用者正确初始化。

### 13.3 size 不是 2 的幂时的回绕开销

当前代码用 `if (tail >= size) tail = 0;` 做回绕, 不依赖 size 是 2 的幂。

但如果 size 是 2 的幂 (如 64、128、256), 可以用位运算优化:
```c
rb->tail  = (rb->tail + 1) & (size - 1);  // 仅当 size 是 2 的幂时成立
rb->head  = (rb->head + 1) & (size - 1);
rb->index = (rb->index) & (size - 1);     // 任意索引回绕
```

当前代码没有要求 size 是 2 的幂, 通用性更好。但位运算版更快。你可以根据需求约束 size 为 2 的幂来优化, 但当前不是。

### 13.4 RingBuf_Count 的分支预测惩罚

RingBuf_Count 中有条件分支:
```c
if (rb->tail >= rb->head) {
    return rb->tail - rb->head;
}
return rb->size - (rb->head - rb->tail);
```

对于写操作远多于读操作的场景 (快速写入、慢速读取), tail 回绕的概率大, 分支预测器可能频繁 miss。MSPM0G3507 的 Cortex-M0+ 没有分支预测器, 每次分支都有固定的 1~2 周期惩罚。这可以接受。

### 13.5 RingBuf_PutArray 中途失败导致部分写入

```c
uint8_t RingBuf_PutArray(RingBuf_t *rb, const uint8_t *src, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        if (RingBuf_Put(rb, src[i]) != 0) return 1;
    }
    return 0;
}
```

如果 10 字节的数据包在写入第 7 字节时缓冲区满了, 函数返回 1, 但前 6 字节已经写入。调用者无法恢复这个状态, 缓冲区里混入了半帧数据。

**处理方式**: 调用者应该在 PutArray 失败后调用 `RingBuf_Reset` 丢弃所有数据, 或者增加协议设计让接收方能够检测并丢弃不完整帧。

### 13.6 16 位 head/tail 溢出

head 和 tail 是 `uint16_t`, 范围 0~65535。在长期运行的系统中, 如果频繁读写, head/tail 会不会溢出?

**答案: 不会**。因为 head 和 tail 在达到 size-1 后回绕到 0 (而非继续递增到 65535)。回绕条件是 `>= size`, 所以 head 和 tail 始终在 [0, size-1] 范围内, 不会溢出。

但如果你把 head/tail 改成不断递增不回绕 (有些方案用无符号整数自然溢出), 需要确保 count() 的计算正确。本模块方案没有这个问题, head/tail 始终在有限范围内。

---

> 本文档是针对 ringbuf 模块的逐行级百科全书, 所有解释基于 MSPM0G3507 裸机环境。如有疑问, 以实际硬件调试结果为准。
