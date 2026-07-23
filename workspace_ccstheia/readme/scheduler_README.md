# scheduler 调度器模块 — 超详细解读

> **README 定位**: 当你看代码旁边的注释都看不懂时来翻阅的百科全书
>
> **Target**: MSPM0G3507 @ 32MHz, Cortex-M0+, no RTOS
>
> **关键词**: 裸机调度器, 协作式, 时间片轮询, SysTick, 任务注册, 周期执行

---

## 1. 模块概述

这个 scheduler 模块实现了一个**裸机协作式任务调度器**。所谓"裸机"就是你写的所有代码跑在一个 `while(1)` 大循环里, 没有操作系统的线程/进程概念; "协作式"意味着任务之间不互相抢占, 每个任务主动让出 CPU (跑完就返回), 调度器没有权限强制打断一个正在执行的任务。

核心流程:
```
SysTick 1ms 中断 → 更新系统计数器(GetTick 返回值++)
主循环 while(1) 中调用 Sched_Run() → 遍历任务池 → 到期任务按顺序执行
```

调度器本身**不创建任务、不分配内存**, 它只负责"什么时候该跑哪个函数"。任务函数必须事先注册到调度器的任务池里, 调度器到点就调它。

> **为什么不用 RTOS?** Cortex-M0+ 没有 MPU, 跑 FreeRTOS 等 RTOS 虽然可行, 但对于简单的传感采集/IO 控制场景, 一个调度器加一个 while(1) 足矣, 省去了任务栈配置、IPC 通信等复杂度, 内存占用也更低。

---

## 2. 核心数据结构

### 2.1 函数指针类型 — `SchedTaskFunc_t`

```c
typedef bool (*SchedTaskFunc_t)(void *context);
```

这是任务的函数签名。参数 `void *context` 是一个万能指针, 你可以传一个结构体指针进去, 或者传 NULL。返回值 `bool` 表示"这个任务还要不要继续调度":
- `true` : 继续调度, 下一个周期还会执行
- `false`: 自动禁用该任务, 相当于一次性任务或异常终止

> 这个设计很有意思: 你没看错, 任务自己决定生死。比如你有一个"上电自检"任务, 执行完返回 false, 它就被自动移出调度队列了(其实只是禁用, 槽位还占着)。

---

### 2.2 任务描述符 — `Sched_Task_t`

```c
typedef struct {
    SchedTaskFunc_t func;       // 任务函数指针
    void           *context;    // 任务上下文参数
    uint32_t       interval;    // 执行间隔, ms
    uint32_t       last_run;    // 上次执行时间戳
    bool           enabled;     // 使能标志
} Sched_Task_t;
```

| 成员 | 类型 | 含义 |
|------|------|------|
| `func` | `SchedTaskFunc_t` | 函数指针, 为 NULL 表示该槽位空闲 |
| `context` | `void*` | 每次调用时传回给 func 的参数, 你可以在外面定义一个结构体, 把多个任务共用的数据打包传进去 |
| `interval` | `uint32_t` | 周期, 单位 ms。比如设 1000 就是每秒执行一次 |
| `last_run` | `uint32_t` | 上次执行时的系统嘀嗒值。Sched_Run 用 `now - last_run >= interval` 判断是否到期 |
| `enabled` | `bool` | false 时该任务被跳过, 但槽位依然被占用, 可以随时通过 Sched_EnableTask 再打开 |

> **核心机制**: `last_run + interval` 的对比方式决定了调度精度。`last_run` 在任务执行后累加 `interval` (而不是设为 `now`), 这是为了避免"漂移"(drift)。后面会详细说。

---

### 2.3 调度器句柄 — `Sched_t`

```c
typedef struct {
    Sched_Task_t *tasks;        // 任务池指针
    uint16_t      capacity;     // 任务池容量
    uint16_t      count;        // 已注册任务数
    uint32_t    (*GetTick)(void); // 获取时间戳的函数指针
} Sched_t;
```

| 成员 | 含义 |
|------|------|
| `tasks` | 指向外部提供的 `Sched_Task_t` 数组, 这个数组由用户在外部定义(静态/全局), 调度器只管用, 不分配也不释放 |
| `capacity` | 数组的元素个数, 最大 65535 |
| `count` | 当前已注册的任务个数, 方便外部查看调度器负载 |
| `GetTick` | 函数指针, 每次 Sched_Run 通过它获取当前系统嘀嗒值 |

> **为什么 `tasks` 是外部传入的指针而不是调度器内部定义?** 因为这样调度器就不需要知道内存分配策略。你可以传一个静态全局数组, 也可以传一个动态分配的堆空间, 甚至可以在不同模块创建多个调度器实例(比如一个高速调度器跑 1ms 任务, 一个低速调度器跑 100ms 任务)。**更多时候, 整个系统只需要一个调度器实例**。

---

### 2.4 为什么用结构体数组而不用链表?

```c
Sched_Task_t task_pool[8];  // 8个槽位, 静态分配
Sched_t      sched;         // 调度器实例
```

这是嵌入式 C 里一个经典的决策。你要存储多个任务的信息, 可以用数组, 也可以用链表:

| 特性 | 数组 (`Sched_Task_t pool[N]`) | 链表 (malloc 或静态池) |
|------|------|------|
| 内存确定性 | ✅ 编译时就确定, 总能通过 | ❌ 碎片或耗尽时分配失败 |
| 查找速度 | O(N), 但 N 很小(通常 < 32) | O(N) |
| 插入/删除 | 直接覆盖, 无碎片 | 需要维护指针 |
| 遍历 | 指针步进, 缓存友好 | 指针跳转, 缓存不友好 |
| 代码复杂度 | 极低 | 较高 |

**结论: 对于这种小规模(< 32 个任务)的场景, 数组永远是正确的选择。** 不仅代码简单、无动态内存问题, 而且每次 `Sched_Run` 遍历时的缓存局部性更好(所有任务在连续内存里)。

---

## 3. API 逐行拆解

### 3.1 `Sched_Init` — 初始化调度器

```c
uint8_t Sched_Init(Sched_t *sched, Sched_Task_t *task_pool,
                   uint16_t capacity,
                   uint32_t (*get_tick)(void))
{
    if (sched == NULL || task_pool == NULL || get_tick == NULL) {
        return 1;
    }
    if (capacity == 0) return 1;
```

**第 1 块(参数校验):** 三个指针参数任意一个为 NULL 都是致命错误, 直接返回 1。`capacity` 为 0 也没意义。

> 为什么返回 `uint8_t`? 0 表示成功, 非 0 表示错误码。这里只用到了 1, 预留了扩展空间。由于 MSPM0G3507 是 32 位 MCU, `uint8_t` 不比 `uint32_t` 快多少, 但语义上更清晰: 错误码只有几种可能性。

---

```c
    sched->tasks    = task_pool;
    sched->capacity = capacity;
    sched->count    = 0;
    sched->GetTick  = get_tick;
```

**第 2 块(赋值):** 把外部传入的参数存到结构体中。注意这里只是存指针, 没有复制数据, 所以 `task_pool` 的生命周期必须比 `sched` 长。

---

```c
    for (uint16_t i = 0; i < capacity; i++) {
        task_pool[i].func     = NULL;
        task_pool[i].context  = NULL;
        task_pool[i].interval = 0;
        task_pool[i].last_run = 0;
        task_pool[i].enabled  = false;
    }

    return 0;
}
```

**第 3 块(清空任务槽):** 遍历整个数组, 把所有字段清零。`func = NULL` 是关键 — `Sched_Run` 和 `Sched_RegisterTask` 都通过判断 `func` 是否为 NULL 来区分"这个槽在用"还是"空闲"。

> **为什么要手动清空?** 如果 `task_pool` 是全局未初始化变量(BSS段), 启动时自动为 0。但如果你复用一个之前用过的池子, 或者 `task_pool` 是栈上的局部变量(强烈不推荐, 因为 Sched_Init 返回后栈空间可能被覆盖), 里面的垃圾数据会让调度器把无效的函数指针当作任务来执行, 直接 HardFault。

---

### 3.2 `Sched_RegisterTask` — 注册任务

```c
uint8_t Sched_RegisterTask(Sched_t *sched, SchedTaskFunc_t func,
                           void *context, uint32_t interval_ms)
{
    if (sched == NULL || func == NULL) return 0xFF;
    if (interval_ms == 0) return 0xFF;
```

**第 1 块(参数校验):** `sched` 和 `func` 为 NULL 时返回 `0xFF`。`interval_ms` 不能为 0, 否则 `now - last_run >= 0` 永远成立, 任务每调用一次 `Sched_Run` 就执行一次。

> **关于 0xFF**: 0xFF 是一个"不可能的有效索引"标记。因为 `capacity` 最大 65535, 但返回类型是 `uint8_t`, 所以合法索引范围 0~254, 0xFF(255) 保留做错误码。如果你的任务池有 256 个以上槽位...那你应该用 `uint16_t` 做索引, 但这个调度器没考虑这种情况。

---

```c
    for (uint16_t i = 0; i < sched->capacity; i++) {
        if (sched->tasks[i].func == NULL) {
            sched->tasks[i].func     = func;
            sched->tasks[i].context  = context;
            sched->tasks[i].interval = interval_ms;
            sched->tasks[i].last_run = sched->GetTick();
            sched->tasks[i].enabled  = true;
            sched->count++;
            return (uint8_t)i;
        }
    }
    return 0xFF;
}
```

**第 2 块(查找空闲槽):** 从索引 0 开始线性查找, 找到第一个 `func == NULL` 的槽位填入任务信息。注意 `last_run` 被设为**当前时刻**而非 0。这意味着任务注册后不会立即执行, 而是要等满一个 `interval` 才会触发。

> **为什么不从上次中断的位置开始找?** 对于 N <= 32 的数组, 线性查找的开销可以忽略。每次从 0 开始逻辑简单, 而且能保证如果任务被注销后, 前面的空位能被优先复用。

---

### 3.3 `Sched_UnregisterTask` — 注销任务

```c
uint8_t Sched_UnregisterTask(Sched_t *sched, uint16_t index)
{
    if (sched == NULL) return 1;
    if (index >= sched->capacity) return 1;
    if (sched->tasks[index].func == NULL) return 1;
```

**第 1 块(校验三重门):**
1. `sched` 不是 NULL
2. `index` 不越界
3. 该槽位确实是注册状态(`func != NULL`)

三重保障, 一个都不能少。尤其第三个检查可以防止你重复注销同一个任务导致 count 为负。

---

```c
    sched->tasks[index].func     = NULL;
    sched->tasks[index].context  = NULL;
    sched->tasks[index].interval = 0;
    sched->tasks[index].last_run = 0;
    sched->tasks[index].enabled  = false;

    if (sched->count > 0) sched->count--;
    return 0;
}
```

**第 2 块(清空槽位):** 把所有字段清零, 标记为空闲。`count` 做保护性减一(防止 BUG 导致 count 变为负数)。

> **不压缩数组**: 注销后槽位空着, 后面的任务不会往前挪。好处是你之前拿到的索引仍然没变 — 但显然注销后这个索引已经无效了, 所以这个好处聊胜于无。真正的原因: 压缩数组需要移动内存, O(N) 开销, 且使索引失效, 不值当。

---

### 3.4 `Sched_EnableTask` — 使能/禁能任务

```c
uint8_t Sched_EnableTask(Sched_t *sched, uint16_t index, bool enable)
{
    if (sched == NULL) return 1;
    if (index >= sched->capacity) return 1;
    if (sched->tasks[index].func == NULL) return 1;

    sched->tasks[index].enabled = enable;
    return 0;
}
```

极简函数。无非是前两个函数校验逻辑的复现 + 一行赋值。**唯一需要注意的是**: 禁用一个任务(`enable = false`)不会清空它的 `func` 指针, 槽位依然是"已占用"状态, 只是 `Sched_Run` 遍历时会跳过它。

> **暂停/恢复语义**: `Sched_EnableTask(sched, i, false)` 等价于暂停任务, `true` 等价于恢复。被暂停的任务再次启用时, 它的 `last_run` 还是暂停前的值, 这意味着如果暂停了 5 秒, 重启后第一个周期可能立即到期(因为 `now - last_run` 已经很大了)。如果你希望暂停后重新计时, 需要手动把 `last_run` 设为 `GetTick()`。

---

### 3.5 `Sched_Run` — 轮询执行到期任务 (核心)

```c
void Sched_Run(Sched_t *sched)
{
    if (sched == NULL || sched->GetTick == NULL) return;

    uint32_t now = sched->GetTick();
```

**第 1 块(快照当前时间):** 在遍历开始前获取一次 `GetTick()`。这样即使遍历过程中发生了中断(比如 SysTick 更新了计数器), 本次调度所有任务的到期判断都基于同一个时间点, 保证一致性。

---

```c
    for (uint16_t i = 0; i < sched->capacity; i++) {
        Sched_Task_t *task = &sched->tasks[i];
        if (!task->enabled || task->func == NULL) continue;
```

**第 2 块(遍历+跳过无效):** 跳过 `enabled = false` 或 `func = NULL` 的槽位。`continue` 是这个循环的减速带 — 大多数槽位可能都是空的, 它们被跳过几乎没有成本。

> **为什么要检查 `func == NULL`?** `enabled` 为 false 的任务也可能 `func` 非空, 但反过来, 空槽位的 `func` 肯定是 NULL。这里两个条件一起检查其实是冗余的(空槽位一定 `enabled = false`), 但这是防御性编程: 万一谁手贱改了内部字段呢?

---

```c
        uint32_t elapsed = now - task->last_run;
        if (elapsed >= task->interval) {
            task->last_run += task->interval;
```

**第 3 块(到期判断 + last_run 累加):** 这是整个调度器最核心的两行代码。

首先是到期判断: `now - last_run >= interval`。当 `last_run = 1000`, `interval = 100`, `now = 1100` 时, `elapsed = 100, 100 >= 100` 成立, 执行。

然后是 **last_run 累加法**: `last_run += interval` 而非 `last_run = now`。

> **为什么用 `+= interval` 而不是 `= now`?**
>
> 假设某个任务每 100ms 执行一次, 用 `= now` 的方式:
> - 第 0ms: last_run = 0, 执行
> - 执行耗时 5ms, last_run = 5
> - 第 105ms: now - last_run = 100, 执行
> - 执行耗时 5ms, last_run = 110
> - 第 210ms: 执行...
>
> 看到了吗? 每次执行都晚了 5ms, 因为 `last_run` 被设成了执行结束的时间。经过 10 个周期, 任务已经偏移了 50ms。这叫 **漂移(drift)**。
>
> 用 `+= interval` 的方式:
> - 第 0ms: last_run = 0, 执行
> - 第 100ms: last_run = 100, 执行
> - 第 200ms: last_run = 200, 执行
>
> 不管任务执行了多久, `last_run` 始终对齐到理论执行时刻的整数倍。这就是**无漂移调度**。

---

```c
            if (now - task->last_run >= task->interval) {
                task->last_run = now;
            }
```

**第 4 块(任务堆积保护):** 这是防崩溃的最后一道防线。试想这样一个场景:

```
任务 T: interval = 100ms
第 0ms: last_run = 0,  正常执行(耗时 150ms, 严重超时)
第 100ms: last_run 累加为 100, 但 now = 250(因为前面超时)
    检查: now - last_run = 150 >= 100, 又要执行!
    last_run 累加: 100 + 100 = 200
    再次检查: now - 200 = 50 < 100, 终止
```

如果没有这段保护代码, 这个任务会在第 250ms 时被连续触发多次, 产生**任务堆积**(task stacking/task overrun)。具体来说:
- 第一次: `last_run = 0` → `0+100=100`, 检查 `250-100=150>=100`, 执行
- 第二次: `last_run = 100` → `100+100=200`, 检查 `250-200=50>=100`? 不成立, 停

但假设超时非常严重(比如被更高优先级的硬阻塞代码卡了 10 秒):
- `last_run` 会从 0 跳到 100, 再到 200, 300... 直到追上 `now`。
- 在追赶过程中, **每个周期任务都会被立即执行一次**, 造成瞬时的 CPU 负载飙升。

这段保护代码的做法是: 如果累加一次后 `now - last_run` 仍然 >= interval, 说明任务已经落后太多了, 干脆放弃追赶, 直接把 `last_run` 设为 `now`, 从零开始。

> **这叫"丢帧"策略**。在传感器采集场景中, 丢几帧数据远比"卡住整个系统连续执行几十次旧任务"要好。比如一个 1kHz 的传感器读取任务, 如果因为某次 I2C 等待卡了 50ms, 你不会想它在 1ms 内连续执行 50 次读操作。

---

```c
            if (!task->func(task->context)) {
                task->enabled = false;
            }
```

**第 5 块(执行 + 自禁用):** 调用任务函数, 如果返回 false, 自动禁用它。

> **执行时序**: 注意任务函数是在 `Sched_Run` 内部被同步调用的。这意味着所有到期任务按**槽位顺序**依次执行。如果一个任务耗时很长(比如 `HAL_Delay(500)`), 排在它后面的所有任务都会被延迟。这是协作式调度最大的缺点, 也是最大的优点(你完全清楚每个时刻在跑什么代码, 没有并发问题)。

---

## 4. 调度器原理深度分析

### 4.1 协作式 vs 抢占式 vs 时间片轮转

这三个概念经常被混淆, 这里说清楚:

| 调度方式 | 典型实现 | 能打断任务吗 | 任务需要主动让出吗 |
|----------|----------|-------------|-------------------|
| **协作式** | 这个 scheduler | ❌ 不行 | ✅ 必须 |
| **时间片轮转** | 裸机 while + 状态机 | ❌ 不行 | ✅ 必须 |
| **抢占式** | FreeRTOS, uC/OS | ✅ 可以 | ❌ 不需要 |
| **混合式** | 大多数 RTOS | ✅ 可以 | ❌ 不需要 |

- **协作式**: 任务不主动返回, CPU 就一直卡在任务里。我们这个调度器就是协作式 — `Sched_Run` 只是检查"该执行哪个任务", 然后直接调用它。如果任务函数里有死循环, 整个系统就冻住了。
- **时间片轮转**: 严格来说, 时间片轮转指每个任务分到一个固定时长的 CPU 时间片, 时间到就强制切换。这需要定时器中断切换上下文, 属于抢占式的一种。**我们这个调度器既不是时间片轮转, 也不是抢占式**。它只是"在合适的时间点调用合适的函数"。

> **更准确的叫法**: 这个调度器应该叫"**定时轮询调度器**(Timer-polled Scheduler)"或"**周期任务调度器**"。

---

### 4.2 时间精度保障

SysTick 配置为 1ms 中断一次:
```
SysTick 计数值 = 系统时钟 / 1000 - 1 = 32,000,000 / 1000 - 1 = 31,999
```

在 MSPM0G3507 上, SysTick 是一个 24 位递减计数器, 载入 `31999` 后, 每经过 1 个时钟周期减 1, 减到 0 触发中断, 然后自动重载。

在中断服务函数中:
```c
void SysTick_Handler(void) {
    sys_tick_count++;
}
```

`Sched_Run` 通过 `GetTick` 获取 `sys_tick_count`。

**时间误差来源分析:**

| 误差来源 | 量级 | 说明 |
|----------|------|------|
| SysTick 中断延迟 | ~12 cycles(~0.375µs) | 中断入栈+跳转, 每次中断固定延迟 |
| `sys_tick_count++` 执行时间 | ~3 cycles(~0.094µs) | 几乎可忽略 |
| `Sched_Run` 中对 `GetTick` 的调用时间 | ~5 cycles(~0.156µs) | 每次调度一次 |
| 任务执行时间导致的下一次调度延迟 | N ms | 这个才是大头 |

调度器本身不引入周期性误差(因为 `last_run += interval` 消除了漂移), 但**单个任务的执行延迟会推迟同一个调度周期内排在后面的所有任务**。如果 A 任务执行了 10ms, B 和 C 都会被推迟 10ms。

---

### 4.3 多个任务同时到期时的执行顺序

假设三个任务都在 `now = 5000` 时到期:

```
任务 A: interval=100, last_run=4900, 索引 0
任务 B: interval=50,  last_run=4950, 索引 1
任务 C: interval=100, last_run=4900, 索引 2
```

`Sched_Run` 从索引 0 到 `capacity-1` 遍历:

1. `i=0`: A 到期(A: 5000-4900=100>=100) → 执行 A
2. `i=1`: B 到期(B: 5000-4950=50>=50) → 执行 B
3. `i=2`: C 到期(C: 5000-4900=100>=100) → 执行 C

**顺序 = 槽位索引顺序**。

如果你的任务有优先级关系(比如"看门狗喂狗必须优先于 LED 闪烁"), 就需要手动把高优先级的任务注册到低索引的槽位, 或者专门分出一个"高优先级轮次"的调度器实例。**调度器本身没有优先级概念**, 这也是"裸机"的体现 — 一切交给用户控制。

---

### 4.4 任务注册时 `last_run = GetTick()` 的意义

```c
sched->tasks[i].last_run = sched->GetTick();  // 设为当前时刻
```

如果注册时设 `last_run = 0`:
- 系统运行了 5000ms 后注册一个 `interval = 1000ms` 的任务
- `elapsed = 5000 - 0 = 5000 >= 1000`, 立即触发
- 这通常不是你想要的行为 — 你希望注册后等 1000ms 再执行

设 `last_run = now` 的效果:
- 注册后 `elapsed = 0`, 不触发
- 必须等满一个 `interval` 才触发
- 这叫"延迟到下一个整周期执行"

---

## 5. SysTick 配置 (MSPM0G3507 平台相关)

虽然标题写"不依赖 DriverLib", 但 SysTick 的控制寄存器是 Cortex-M0+ 内核标配的, 不涉及厂商外设, 所以可移植性依然很好。以下是配置代码:

```c
// 在 system_mspm0g3507.c 或 main.c 中
volatile uint32_t sys_tick_count = 0;

// SysTick 中断服务函数
void SysTick_Handler(void) {
    sys_tick_count++;
}

// 获取系统嘀嗒数的函数, 传给 Sched_Init 的 get_tick
uint32_t GetSysTick(void) {
    return sys_tick_count;
}
```

SysTick 寄存器配置(M0+ 内核编程手册):

```c
#define SYST_CSR  ((volatile uint32_t *)0xE000E010)  // 控制与状态寄存器
#define SYST_RVR  ((volatile uint32_t *)0xE000E014)  // 重载值寄存器
#define SYST_CVR  ((volatile uint32_t *)0xE000E018)  // 当前值寄存器

void SysTick_Init(void) {
    *SYST_RVR = 31999;       // 重载值 = 32000000/1000 - 1
    *SYST_CVR = 0;           // 清空当前值
    *SYST_CSR = 0x07;        // 使能 SysTick, 使能中断, 使用内核时钟(HCLK)
}
```

`*SYST_CSR = 0x07` 的位含义:
- Bit 0: ENABLE — 使能计数器
- Bit 1: TICKINT — 计到 0 时触发 SysTick 异常
- Bit 2: CLKSOURCE — 1 = 使用内核时钟(HCLK = 32MHz), 0 = 使用 HCLK/8

如果你的系统时钟不是 32MHz 了(比如降频到 16MHz 省电), 重载值也要跟着改: `16000000/1000 - 1 = 15999`。

---

## 6. 典型用法

```c
#include "scheduler.h"

#define TASK_POOL_SIZE  8

// 任务池(全局静态数组)
static Sched_Task_t task_pool[TASK_POOL_SIZE];
static Sched_t      sched;

// 系统嘀嗒计数器, 在 SysTick_Handler 中递增
volatile uint32_t sys_tick_ms = 0;

uint32_t GetSysTick(void) {
    return sys_tick_ms;
}

void SysTick_Handler(void) {
    sys_tick_ms++;
}

// 任务函数: 每 1000ms 切换一次 LED
bool led_task(void *ctx) {
    *(uint8_t *)ctx ^= 0x01;  // 翻转 GPIO
    return true;               // 继续调度
}

// 任务函数: 每 10ms 采集一次 ADC
bool adc_task(void *ctx) {
    // 读取 ADC...
    return true;
}

int main(void) {
    SysTick_Init();                      // 配置 SysTick 1ms 中断
    Sched_Init(&sched, task_pool, TASK_POOL_SIZE, GetSysTick);

    uint8_t led_pin = 0;
    Sched_RegisterTask(&sched, led_task, &led_pin, 1000);
    Sched_RegisterTask(&sched, adc_task, NULL, 10);

    while (1) {
        Sched_Run(&sched);              // 必须频繁调用
        // 也可以做其他低优先级轮询
    }
}
```

---

## 7. 踩坑点 (必读)

### 7.1 坑 1: 任务执行时间超过调度周期

```
任务 A: interval = 1ms, 执行时间 = 2ms (超了!)
任务 B: interval = 10ms (排在 A 后面)
```

`Sched_Run` 执行过程:
- 检查 A: 到期 → 执行 A, 花了 2ms
- 检查 B: 已经过去了 2ms, `now` 还是一样的(因为 `GetTick` 在循环前已获取), 所以 B 的到期判断不受影响...吗?
- 不对! 如果 `GetTick` 在 `Sched_Run` 开始时被快照, B 不受影响。如果 B 恰好在 A 执行期间也到期了, 它会在本次轮次的下半段被执行。

但问题在于: **主循环中下一次调用 `Sched_Run` 会延迟**。如果 A 每次跑 2ms, 而 SysTick 每 1ms 中断一次, 那么:
- `Sched_Run` 返回后, `while(1)` 里其他代码也要执行时间
- 下一次 `Sched_Run` 可能在 3ms 后才被调用
- A 的 `last_run` 已经加了 1ms, 但 `now` 已经过去了 3ms, `elapsed = 3ms >= 1ms`, 立马又要执行

结果: **A 几乎每个主循环都会被执行一次, CPU 全部耗在 A 上, B 饿死**。

> **解决方案**: 要么优化 A 的执行时间(用 DMA、用状态机拆分), 要么增大 A 的 interval, 要么把 A 移到单独的调度器实例中(但需要额外的设计)。

### 7.2 坑 2: 任务里调用阻塞函数

```c
bool my_task(void *ctx) {
    HAL_Delay(50);  // 严禁!
    return true;
}
```

在 `Sched_Run` 内部调用 `HAL_Delay` 或任何阻塞等待函数, 都会让整个调度器停滞。因为 `Sched_Run` 是顺序执行的, 一个任务阻塞 → 所有任务阻塞。

> **解决方案**: 把阻塞等待改成状态机轮询。比如等待外设就绪, 不是 `while(!ready);`, 而是在任务里检查状态, "没就绪就返回, 下次再查"。

### 7.3 坑 3: `last_run` 溢出

`uint32_t` 最大 4294967295, 在 1ms 嘀嗒下约 49.7 天会回绕到 0。

```c
// 49.7 天后:
last_run = 0xFFFFFFF0, now = 0x00000010
elapsed = 0x10 - 0xFFFFFFF0 = 0x20 = 32  // 正确!
```

**好消息: 无符号整数的减法在 C 语言中表现为回绕算术, 结果仍然是正确的。** 只要你始终用 `uint32_t` 且差值小于 2^31, 溢出不是问题。

---

## 8. 调试手段

### 8.1 LED 翻转观察任务执行周期

```c
bool debug_task_1ms(void *ctx) {
    static uint8_t toggle = 0;
    toggle ^= 0x01;
    // 接示波器到 GPIO
    return true;
}

bool debug_task_1000ms(void *ctx) {
    // 人眼可见闪烁
    HAL_GPIO_Toggle(LED_GPIO_Port, LED_Pin);
    return true;
}
```

用示波器/逻辑分析仪抓 GPIO 波形, 看高电平宽度就是任务执行时间, 看周期是否精确 1ms/1000ms。如果 1ms 任务的波形忽长忽短, 说明有其他任务干扰了调度。

### 8.2 任务执行时间统计

```c
bool measure_task(void *ctx) {
    uint32_t start = GetSysTick();
    // ... 被测量的代码 ...
    uint32_t elapsed = GetSysTick() - start;
    if (elapsed > max_elapsed) max_elapsed = elapsed;
    return true;
}
```

> **注意**: 最精确的计时方法是用 DWT 周期计数器(Cortex-M3+ 才有)或 TIM 捕获。M0+ 没有 DWT, 但可以用另一个通用定时器做微秒级计时。这里用 `GetTick()` 只能精确到 1ms。

### 8.3 检查任务超时

在 `Sched_Run` 中添加调试代码(只在 DEBUG 模式下编译):

```c
void Sched_Run(Sched_t *sched) {
    // ...
    if (elapsed >= task->interval) {
        // 检测到任务已经落后了
        if (elapsed >= task->interval * 2) {
            // 落后超过一个周期! 可能有问题
        }
        // ...
    }
}
```

---

## 9. 与其他方案的对比

| 方案 | 优点 | 缺点 | 适用场景 |
|------|------|------|---------|
| **本调度器** | 轻量(< 100 行代码)、无动态内存、无漂移 | 无优先级、协作式、任务之间互相影响 | 简单的周期采集+控制, IO 任务 |
| **状态机超级循环** | 极简、完全可控 | 代码组织混乱、周期不精确 | 极简单的顺序流程 |
| **FreeRTOS** | 抢占式、优先级、IPC | 内存占用大、需要配置栈 | 复杂系统、通信协议栈 |
| **定时器中断轮询** | 最精确的周期 | 中断中不能做复杂处理 | 高速采样、PWM 等 |

---

> **最后的话**: 这个调度器很简单, 但它的设计——外部任务池、函数指针、无漂移累加、任务堆积保护——体现了嵌入式 C 中"少即是多"的哲学。没有复杂的链表、没有动态分配、没有优先级继承协议, 但它在 100 行代码内解决了一个核心问题: "我如何在裸机上按周期执行多个函数?"。理解了这个代码, 你就理解了 80% 的裸机调度思想。
