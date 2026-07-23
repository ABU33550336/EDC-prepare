//任务调度器,定时轮询执行注册的任务
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

typedef bool (*SchedTaskFunc_t)(void *context);  //任务函数类型,返回true表示还需继续调度

typedef struct {
    SchedTaskFunc_t func;       //任务函数指针
    void           *context;    //任务上下文参数
    uint32_t       interval;    //执行间隔,ms
    uint32_t       last_run;    //上次执行时间戳
    bool           enabled;     //使能标志
} Sched_Task_t;

typedef struct {
    Sched_Task_t *tasks;        //任务池指针
    uint16_t      capacity;     //任务池容量
    uint16_t      count;        //已注册任务数
    uint32_t    (*GetTick)(void);  //获取时间戳的函数指针
} Sched_t;

uint8_t Sched_Init(Sched_t *sched, Sched_Task_t *task_pool,
                   uint16_t capacity,
                   uint32_t (*get_tick)(void));                           //初始化调度器
uint8_t Sched_RegisterTask(Sched_t *sched, SchedTaskFunc_t func,
                           void *context, uint32_t interval_ms);          //注册定时任务
uint8_t Sched_UnregisterTask(Sched_t *sched, uint16_t index);             //注销任务
uint8_t Sched_EnableTask(Sched_t *sched, uint16_t index, bool enable);    //使能/禁能任务
void    Sched_Run(Sched_t *sched);                                        //执行一次调度,遍历所有就绪任务

#endif
