# fsm 模块 — 有限状态机框架（事件驱动型）

> **当你看代码旁边的注释都看不懂时来翻阅的百科全书**
> 适用芯片：MSPM0G3507 @ 32MHz | 纯 C | 零硬件依赖

---

## 1. 模块概述

这是一个**通用的、事件驱动的有限状态机（FSM）框架**。它不绑定任何具体的业务逻辑，只提供：
- 状态（State）定义
- 事件（Event）定义
- 状态转移表（Transition Table）的注册与匹配
- 事件派发（Event Dispatch）机制
- 转移动作回调（Action Callback）

简单说：它是一套**规则引擎**——定义好"什么状态下发生什么事后要去哪个状态并执行什么动作"，然后往里面丢事件即可。

### 使用场景
- 智能小车任务调度：初始化 → 直行 → 转弯 → 避障 → 停止
- 机器人行为控制：空闲 → 循迹 → 充电 → 暂停
- 任何需要明确状态切换逻辑的嵌入式系统

---

## 2. 数据结构

### 2.1 `FSM_State_t` 和 `FSM_Event_t`

```c
typedef uint8_t FSM_State_t;
typedef uint8_t FSM_Event_t;
```

都是 `uint8_t`，取值范围 0-255。用户需要自行定义枚举：

```c
// 用户自定义状态枚举
typedef enum {
    STATE_IDLE     = 0,
    STATE_DRIVE    = 1,
    STATE_TURN     = 2,
    STATE_AVOID    = 3,
    STATE_STOP     = 4,
    STATE_ERROR    = 5,
} AppState_t;

// 用户自定义事件枚举
typedef enum {
    EVENT_START    = 0,
    EVENT_FINISH   = 1,
    EVENT_OBSTACLE = 2,
    EVENT_LOST     = 3,
    EVENT_STOP     = 4,
} AppEvent_t;
```

**为什么不直接用 enum 而用 typedef uint8_t？**

- 枚举在 C 中默认是 `int`（4 字节），在 32MHz 的 MCU 上浪费 RAM 和 ROM
- `uint8_t`（1 字节）足够表示 256 个状态和事件
- 转移表用 `uint8_t` 可以减少一半的存储开销

### 2.2 `FSM_Transition_t`（状态转移表项）

```c
typedef struct {
    FSM_State_t src_state;
    FSM_Event_t event;
    FSM_State_t dst_state;
    void (*action)(void*);
} FSM_Transition_t;
```

| 字段 | 含义 |
|------|------|
| `src_state` | 源状态。只在当前状态等于此值时匹配 |
| `event` | 触发事件。只有此事件到来时才匹配 |
| `dst_state` | 目标状态。匹配后切换到该状态 |
| `action` | 转移动作函数指针。匹配后执行该回调（可以为 NULL） |

**每一行表示一条规则**："如果当前在 src_state，收到 event，就切换到 dst_state，并执行 action"。

这是一个**查表法**实现的状态机——把状态转移逻辑以数据（表）的形式存在 ROM 中，而不是用 switch-case 硬编码。

### 2.3 `FSM_t`（状态机主结构体）

```c
typedef struct {
    FSM_State_t          current_state;
    uint16_t             transition_count;
    const FSM_Transition_t *transitions;
    void                *context;
} FSM_t;
```

| 字段 | 含义 |
|------|------|
| `current_state` | 当前所处状态 |
| `transition_count` | 转移表的条目数 |
| `transitions` | 指向转移表的指针（存储在用户提供的数组中） |
| `context` | **用户上下文指针**。action 回调函数在被调用时会传入此指针 |

---

## 3. 为什么用函数指针表实现？

传统状态机用 switch-case：

```c
switch (state) {
    case STATE_DRIVE:
        if (event == EVENT_OBSTACLE) {
            state = STATE_AVOID;
            action_avoid();
        }
        break;
    // ...
}
```

这种写法的问题：
1. **状态+事件组合多时 switch 巨长**：N 个状态 × M 个事件的组合会使 switch 膨胀到 N×M 行
2. **可维护性差**：增删一个转移需要阅读整个 switch
3. **ROM 利用率低**：switch 编译后是跳转表（lookup table），但编译器无法优化稀疏矩阵

**函数指针表方案**：

```
转移表是一个数组, 每个元素是一条规则。
FSM_Dispatch 只是遍历数组匹配 src_state + event。
```

优点：
- **数据驱动**：转移逻辑是数据而非代码，可以在运行时改变（比如动态加载不同行为）
- **易扩展**：加一条规则就是加一个数组元素
- **ROM 友好**：数据以 const 形式存 ROM，不占 RAM
- **可测试**：可以把转移表提取出来单独测试

缺点：
- 查表是 O(n) 遍历（n = 转移表条目数）。但如果 n < 50，遍历的开销远小于 switch 的维护成本
- 不像 switch 那样可以一眼看出所有转移关系

---

## 4. 函数详解

### 4.1 `FSM_Init`

```c
void FSM_Init(FSM_t *fsm, FSM_State_t init_state,
              const FSM_Transition_t *transitions,
              uint16_t count, void *context)
{
    if (fsm == NULL) return;
    fsm->current_state   = init_state;
    fsm->transitions     = transitions;
    fsm->transition_count = count;
    fsm->context         = context;
}
```

**逐行详解：**

| 代码 | 含义 |
|------|------|
| `fsm->current_state = init_state` | 设置初始状态，比如 STATE_IDLE |
| `fsm->transitions = transitions` | 指向转移表。这个指针**必须**指向一个生命周期 ≥ fsm 的数组（通常定义在全局或 static） |
| `fsm->transition_count = count` | 记录条目数，供 Dispatch 遍历 |
| `fsm->context = context` | 保存上下文指针，后续 action 回调时传回 |

**为什么不在 Init 中拷贝转移表？**

- 转移表通常很大（数十个结构体），拷贝到 FSM_t 内部（需要变长数组或动态内存）不现实
- 转移表在 ROM 中（`const`），本来就不可写
- 状态机框架应该是"轻量"的，不负责管理转移表的生命周期

### 4.2 `FSM_Dispatch`（核心函数）

```c
uint8_t FSM_Dispatch(FSM_t *fsm, FSM_Event_t event)
{
    if (fsm == NULL || fsm->transitions == NULL) return 1;

    for (uint16_t i = 0; i < fsm->transition_count; i++) {
        const FSM_Transition_t *t = &fsm->transitions[i];
        if (t->src_state == fsm->current_state && t->event == event) {
            fsm->current_state = t->dst_state;
            if (t->action != NULL) {
                t->action(fsm->context);
            }
            return 0;
        }
    }
    return 1;
}
```

**逐行详解：**

| 行号 | 代码 | 含义 |
|------|------|------|
| 37 | 参数检查 | fsm 或 transitions 为 NULL 时返回 1（失败） |
| 39 | 遍历转移表 | 遍历所有条目，寻找匹配的 (src_state, event) 对 |
| 40 | 取条目指针 | `t = &fsm->transitions[i]` |
| 41 | 匹配判断 | `t->src_state == current_state && t->event == event` |
| 42 | 状态切换 | 先切换状态，再执行动作。顺序很重要——action 中可以通过 GetCurrentState 看到新状态 |
| 43-44 | 执行动作 | 如果 action 不为 NULL，调用 action(fsm->context) |
| 46 | 返回成功 | return 0 表示转移成功 |
| 49 | 返回失败 | return 1 表示没找到匹配的转移 |

**返回值的含义**：

```
返回值 0: 找到了匹配转移，已执行状态切换和 action
返回值 1: 无匹配转移（状态未变化），或参数无效
```

**典型问题**：为什么切换状态在 action 之前？

假如先执行 action 再切换状态：

```c
if (t->action != NULL) t->action(fsm->context);  // action 中查询到的还是旧状态
fsm->current_state = t->dst_state;                // 然后才切换
```

这样 action 回调中通过 `FSM_GetCurrentState` 看到的是**旧状态**，这通常不是用户期望的。当前代码把切换放在前面，保证 action 在新状态下执行。

### 4.3 `FSM_GetCurrentState`

```c
FSM_State_t FSM_GetCurrentState(FSM_t *fsm)
{
    if (fsm == NULL) return 0;
    return fsm->current_state;
}
```

参数无效时返回 0（STATE_IDLE 通常定义为 0，这个默认值合理）。

---

## 5. 状态转移表设计模式

### 5.1 典型的智能小车状态转移表

```c
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

// 动作函数声明
void action_start_drive(void *ctx);
void action_start_turn(void *ctx);
void action_avoid_obstacle(void *ctx);
void action_stop(void *ctx);
void action_resume(void *ctx);
void action_error_handler(void *ctx);

// 状态枚举
typedef enum {
    ST_IDLE     = 0,
    ST_DRIVE    = 1,
    ST_TURN     = 2,
    ST_AVOID    = 3,
    ST_STOP     = 4,
    ST_ERROR    = 5,
} State_t;

// 事件枚举
typedef enum {
    EV_START       = 0,
    EV_FINISH_LINE = 1,
    EV_OBSTACLE    = 2,
    EV_TURN_DONE   = 3,
    EV_AVOID_DONE  = 4,
    EV_STOP_CMD    = 5,
    EV_LOST_LINE   = 6,
} Event_t;

// 转移表
static const FSM_Transition_t line_follower_transitions[] = {
    // {src_state,     event,            dst_state,    action}
    { ST_IDLE,        EV_START,         ST_DRIVE,      action_start_drive      },
    { ST_DRIVE,       EV_OBSTACLE,      ST_AVOID,      action_avoid_obstacle   },
    { ST_DRIVE,       EV_FINISH_LINE,   ST_TURN,       action_start_turn       },
    { ST_DRIVE,       EV_STOP_CMD,      ST_STOP,       action_stop             },
    { ST_DRIVE,       EV_LOST_LINE,     ST_ERROR,      action_error_handler    },
    { ST_TURN,        EV_TURN_DONE,     ST_DRIVE,      action_resume           },
    { ST_TURN,        EV_OBSTACLE,      ST_AVOID,      action_avoid_obstacle   },
    { ST_AVOID,       EV_AVOID_DONE,    ST_DRIVE,      action_resume           },
    { ST_AVOID,       EV_STOP_CMD,      ST_STOP,       action_stop             },
    { ST_STOP,        EV_START,         ST_DRIVE,      action_start_drive      },
    { ST_ERROR,       EV_START,         ST_IDLE,       NULL                    },
};
```

### 5.2 状态转移图

```
        ┌──────────────┐
        │   ST_IDLE    │◄──── EV_START ──── ST_ERROR
        └───┬──────────┘
            │ EV_START
            ▼
     ┌──────────────┐
     │   ST_DRIVE   │──── EV_FINISH_LINE ──► ST_TURN
     │  (循迹直行)   │──── EV_OBSTACLE ─────► ST_AVOID
     └───┬──────┬───┘──── EV_STOP_CMD ─────► ST_STOP
         │      │        └── EV_LOST_LINE ──► ST_ERROR
         │      │
         │      └── EV_TURN_DONE ◄───────────┘
         │
         └── EV_AVOID_DONE ◄─────────────────┘
```

### 5.3 每个状态的语义

| 状态 | 做什么 | 进入时 action | 退出条件 |
|------|--------|-------------|---------|
| ST_IDLE | 等待启动命令 | — | EV_START |
| ST_DRIVE | 循迹直行（主运行状态） | 启动电机 PID | EV_FINISH_LINE / EV_OBSTACLE / EV_STOP_CMD / EV_LOST_LINE |
| ST_TURN | 转弯（检测到终点线） | 执行预设转弯角度 | EV_TURN_DONE / EV_OBSTACLE |
| ST_AVOID | 避障（检测到障碍） | 执行避障动作（绕行/停止）| EV_AVOID_DONE / EV_STOP_CMD |
| ST_STOP | 停止 | 关电机 | EV_START |
| ST_ERROR | 出错（丢线等） | 报警或掉头 | EV_START（复位） |

---

## 6. 事件驱动 vs 轮询

### 当前实现：**事件驱动**

`FSM_Dispatch` 是一个被动的函数，需要外部调用者传递事件：

```c
// 典型的主循环
while (1) {
    Event_t ev = get_next_event();   // 阻塞等待或轮询获取事件
    FSM_Dispatch(&fsm, ev);          // 事件驱动状态切换
}
```

事件来源：
- 传感器中断（如避障传感器触发 → EV_OBSTACLE）
- 定时器超时（如转弯计时到 → EV_TURN_DONE）
- 串口命令（如收到 's' → EV_START）
- 条件满足（如循迹偏差归零持续 1s → EV_FINISH_LINE）

### 轮询模式（如果需要）

也可以改成轮询模式，在 loop 中持续检查条件：

```c
void poll_and_dispatch(FSM_t *fsm, const uint16_t *sensors)
{
    switch (FSM_GetCurrentState(fsm)) {
        case ST_DRIVE:
            if (obstacle_detected())      FSM_Dispatch(fsm, EV_OBSTACLE);
            else if (line_lost(sensors))   FSM_Dispatch(fsm, EV_LOST_LINE);
            else if (finish_line(sensors)) FSM_Dispatch(fsm, EV_FINISH_LINE);
            break;
        case ST_TURN:
            if (turn_timeout())            FSM_Dispatch(fsm, EV_TURN_DONE);
            break;
        // ...
    }
}
```

**哪个好？** 事件驱动更松耦合，但需要在合适的地方产生事件；轮询更直接但状态机的优势减弱。当前框架两者都支持，取决于调用方式。

---

## 7. context 指针的使用

```c
void action_stop(void *ctx)
{
    MotorContext_t *motor = (MotorContext_t *)ctx;
    motor_set_speed(motor->left_ch, 0);
    motor_set_speed(motor->right_ch, 0);
    printf("[FSM] motors stopped\r\n");
}
```

Context 让 action 回调可以访问到业务数据，而不需要全局变量。这是一种**手动依赖注入**——框架不关心 context 的类型，只是透明地传递。

**典型 context 包含的内容**：
- 电机控制句柄
- PID 参数指针
- 传感器数据结构
- 串口打印句柄
- 调试计数

---

## 8. 踩坑点

### 8.1 状态泄漏（未定义转移）

**问题**：如果在转移表中没有定义 `(current_state, event)` 的组合，`FSM_Dispatch` 返回 1，**状态不变化，action 不执行**。

这可能静默地导致小车"卡在某个状态没反应"。

**例子**：当前在 ST_AVOID，来了个 EV_FINISH_LINE，但转移表里没定义这个组合 → 事件被忽略 → 避障结束后不会恢复循迹。

**防范方法**：
1. 每个状态都**穷举所有可能出现的事件**（用表格对齐）
2. 对处理不了的事件，加一条兜底规则：

```c
// 兜底: 任何未显式定义的事件都转 ERROR 状态
{ ST_DRIVE,  EV_UNKNOWN,  ST_ERROR,  action_unknown_event },
{ ST_TURN,   EV_UNKNOWN,  ST_ERROR,  action_unknown_event },
// ...
```

3. Dispatch 的返回值一定要检查：

```c
if (FSM_Dispatch(&fsm, event) != 0) {
    printf("WARNING: unhandled event %d in state %d\r\n", event, fsm.current_state);
}
```

### 8.2 重复进入同一状态

**问题**：如果连续派发同一个事件两次（如 EV_START 发了两次），第二次 Dispatch 会再次匹配并执行 action。

某些带副作用的 action（如启动电机、打开电磁铁）重复执行可能出问题。

**解决办法**：在 action 开头做幂等检查：

```c
void action_start_drive(void *ctx)
{
    MotorContext_t *m = (MotorContext_t *)ctx;
    if (m->already_running) return;  // 已经运行了, 不再重复启动
    motor_start();
    m->already_running = 1;
}
```

### 8.3 action 中的死循环或阻塞

**问题**：action 函数如果阻塞（如等待电机启动完成），整个 FSM 也会阻塞。

**FSM 的 action 应该轻量**：只做状态切换时的"瞬时动作"（设置标志、启动定时器、打印日志），而不是执行持续的过程。

```c
// 好的 action
void action_start_turn(void *ctx)
{
    TimerContext_t *t = (TimerContext_t *)ctx;
    timer_start(t->turn_timer, 500);  // 启动 500ms 转弯定时器
    printf("[FSM] entering TURN state\r\n");
}

// 坏的 action
void action_start_turn(void *ctx)
{
    delay_ms(500);  // ❌ 阻塞循环，整个系统卡死
    motor_turn(90);
}
```

### 8.4 多任务抢占

**问题**：如果在中断中调用 `FSM_Dispatch`，而主循环中也调用，会产生竞态条件。

```c
// 两个线程同时修改 fsm.current_state
// → 这行在两个线程中同时执行时会导致状态错乱
fsm->current_state = t->dst_state;
```

**解决方案**：
- 所有 FSM 操作都在同一个 task/loop 中执行
- 中断只产生事件标记，在 main loop 中统一 Dispatch

---

## 9. 状态转移表 vs 嵌套 switch-case 性能对比

假设 10 个状态、10 个事件、20 条有效转移：

| 实现方式 | 代码行数 | 执行时间（最坏情况）| 可维护性 |
|---------|---------|-------------------|---------|
| switch-case | ~200 行 | O(1) 编译器跳转表 | 差（耦合） |
| 查表法（线性搜索）| 20 行表 + 20 行代码 | O(n)=20 次比较 | 好 |
| 查表法（哈希优化）| 复杂 | O(1) | 中 |

对于 MSPM0G3507（32MHz），20 次比较 < 1μs，性能可忽略。

---

## 10. 调试手段

### 10.1 打印状态名

```c
static const char *state_names[] = {
    "IDLE", "DRIVE", "TURN", "AVOID", "STOP", "ERROR"
};

const char* state_to_name(FSM_State_t s) {
    if (s < sizeof(state_names)/sizeof(state_names[0]))
        return state_names[s];
    return "UNKNOWN";
}

// 在 Dispatch 的 action 中打印
void fsm_debug_action(void *ctx) {
    FSM_t *fsm = (FSM_t *)ctx;  // context 可以是 fsm 自身
    // 注意: 此时 fsm->current_state 已经是 dst_state
    printf("[FSM] -> %s\r\n", state_to_name(fsm->current_state));
}
```

### 10.2 跟踪所有转移

给转移表加调试包裹：

```c
uint8_t FSM_Dispatch_Debug(FSM_t *fsm, FSM_Event_t event)
{
    FSM_State_t old_state = fsm->current_state;
    uint8_t ret = FSM_Dispatch(fsm, event);
    if (ret == 0) {
        printf("[FSM] %s --[%d]--> %s\r\n",
            state_to_name(old_state), event,
            state_to_name(fsm->current_state));
    } else {
        printf("[FSM] %s --[%d]--> (no transition)\r\n",
            state_to_name(old_state), event);
    }
    return ret;
}
```

### 10.3 集成到串口命令

```c
// 通过串口手动触发事件进行测试
void uart_cmd_handler(char c)
{
    switch (c) {
        case 's': FSM_Dispatch(&fsm, EV_START);       break;
        case 'o': FSM_Dispatch(&fsm, EV_OBSTACLE);    break;
        case 'f': FSM_Dispatch(&fsm, EV_FINISH_LINE); break;
        case 't': FSM_Dispatch(&fsm, EV_STOP_CMD);    break;
        case '?': printf("State: %s\r\n",
                   state_to_name(FSM_GetCurrentState(&fsm))); break;
    }
}
```

---

## 11. 扩展与改进方向

### 11.1 加入状态入口/出口/持续动作

当前 FSM 只有**转移动作**（Transition Action）。更完整的框架应包含：

```
进入状态时: Entry Action (执行一次)
在状态中:   Do Action (持续执行)
离开状态时: Exit Action (执行一次)
```

但这样会使结构体更复杂（每个状态需要 3 个函数指针）。当前的设计选择了最精简的实现。

### 11.2 优先级匹配

当前查表匹配到第一条就返回。如果要支持"默认转移"（fallback）功能，可以把特定规则放在表尾，或者修改 Dispatch 策略。

### 11.3 层次状态机（Hierarchical FSM）

层次状态机允许一个状态包含子状态（如 ST_DRIVE 包含 ST_DRIVE_FORWARD, ST_DRIVE_SLOW 等）。当前框架不支持，需要另外实现。

---

## 12. 完整示例

```c
#include "fsm.h"

enum { ST_IDLE, ST_DRIVE, ST_STOP };
enum { EV_GO, EV_STOP, EV_FINISH };

void on_enter_drive(void *ctx) { printf("start driving\r\n"); }
void on_stop(void *ctx)        { printf("stopped\r\n"); }

static const FSM_Transition_t table[] = {
    { ST_IDLE,  EV_GO,    ST_DRIVE, on_enter_drive },
    { ST_DRIVE, EV_STOP,  ST_STOP,  on_stop         },
    { ST_DRIVE, EV_FINISH,ST_IDLE,  NULL            },
    { ST_STOP,  EV_GO,    ST_DRIVE, on_enter_drive },
};

FSM_t fsm;

int main(void)
{
    FSM_Init(&fsm, ST_IDLE, table, 4, NULL);

    FSM_Dispatch(&fsm, EV_GO);      // IDLE → DRIVE, prints "start driving"
    FSM_Dispatch(&fsm, EV_FINISH);  // DRIVE → IDLE
    FSM_Dispatch(&fsm, EV_STOP);    // IDLE 下 EV_STOP 无匹配, 返回 1
    // 此时状态仍为 IDLE

    return 0;
}
```

---

## 13. 与其他模块的整合

```
传感器/定时器/串口
    │
    ▼  (产生事件)
FSM_Dispatch()
    │
    ▼  (状态切换 + action)
Action 函数
    │
    ├── 控制 diff_drive → 电机
    ├── 读取 line_follower → 判定事件
    └── 打印日志 → 串口
```

FSM 是整个系统的**大脑**——它不直接控制硬件，而是通过 action 回调调度其他模块。这种分层使得每个模块都可以独立测试和替换。
