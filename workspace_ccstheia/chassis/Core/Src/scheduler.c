//任务调度器模块,基于时间片轮询执行注册任务

#include "scheduler.h"
#include <stddef.h>

/**
 * @brief 初始化调度器
 * @param sched 调度器指针
 * @param task_pool 任务池数组(由外部提供静态/全局空间)
 * @param capacity 任务池容量
 * @param get_tick 获取系统滴答的函数指针
 * @return 0=成功,1=参数无效
 * @note task_pool生命周期必须长于sched
 *        调用者:
 */
uint8_t Sched_Init(Sched_t *sched, Sched_Task_t *task_pool,
                   uint16_t capacity,
                   uint32_t (*get_tick)(void))
{
    if (sched == NULL || task_pool == NULL || get_tick == NULL) {
        return 1;
    }
    if (capacity == 0) return 1;

    sched->tasks    = task_pool;
    sched->capacity = capacity;
    sched->count    = 0;
    sched->GetTick  = get_tick;

    //清空所有任务槽位,避免未初始化数据导致误判
    for (uint16_t i = 0; i < capacity; i++) {
        task_pool[i].func     = NULL;
        task_pool[i].context  = NULL;
        task_pool[i].interval = 0;
        task_pool[i].last_run = 0;
        task_pool[i].enabled  = false;
    }

    return 0;
}

/**
 * @brief 注册一个周期性任务
 * @param sched 调度器指针
 * @param func 任务函数(返回false自动禁用)
 * @param context 任务参数指针
 * @param interval_ms 执行周期,ms
 * @return 任务索引(0~254),0xFF表示失败
 * @note interval_ms不能为0;func返回false时任务被自动禁用
 *        调用者:
 */
uint8_t Sched_RegisterTask(Sched_t *sched, SchedTaskFunc_t func,
                           void *context, uint32_t interval_ms)
{
    if (sched == NULL || func == NULL) return 0xFF;
    if (interval_ms == 0) return 0xFF;

    //查找第一个空闲槽位
    for (uint16_t i = 0; i < sched->capacity; i++) {
        if (sched->tasks[i].func == NULL) {
            sched->tasks[i].func     = func;
            sched->tasks[i].context  = context;
            sched->tasks[i].interval = interval_ms;
            sched->tasks[i].last_run = sched->GetTick();                      //首次运行时戳设为当前时刻
            sched->tasks[i].enabled  = true;
            sched->count++;
            return (uint8_t)i;
        }
    }
    return 0xFF;                                                              //任务池已满
}

/**
 * @brief 注销指定任务
 * @param sched 调度器指针
 * @param index 任务索引
 * @return 0=成功,1=参数无效或槽位已空
 * @note 仅清除槽位,不压缩数组
 *        调用者:
 */
uint8_t Sched_UnregisterTask(Sched_t *sched, uint16_t index)
{
    if (sched == NULL) return 1;
    if (index >= sched->capacity) return 1;
    if (sched->tasks[index].func == NULL) return 1;                           //该槽位未注册

    sched->tasks[index].func     = NULL;
    sched->tasks[index].context  = NULL;
    sched->tasks[index].interval = 0;
    sched->tasks[index].last_run = 0;
    sched->tasks[index].enabled  = false;

    if (sched->count > 0) sched->count--;
    return 0;
}

/**
 * @brief 启用或禁用指定任务
 * @param sched 调度器指针
 * @param index 任务索引
 * @param enable true=启用,false=禁用
 * @return 0=成功,1=参数无效或槽位为空
 * @note 禁用后任务仍占用槽位
 *        调用者:
 */
uint8_t Sched_EnableTask(Sched_t *sched, uint16_t index, bool enable)
{
    if (sched == NULL) return 1;
    if (index >= sched->capacity) return 1;
    if (sched->tasks[index].func == NULL) return 1;                           //空槽位不可操作

    sched->tasks[index].enabled = enable;
    return 0;
}

/**
 * @brief 轮询执行所有到期任务
 * @param sched 调度器指针
 * @note 此函数应在主循环或定时中断中频繁调用;使用累加last_run防止漂移
 *        调用者:
 */
void Sched_Run(Sched_t *sched)
{
    if (sched == NULL || sched->GetTick == NULL) return;

    uint32_t now = sched->GetTick();                                          //当前系统滴答

    for (uint16_t i = 0; i < sched->capacity; i++) {
        Sched_Task_t *task = &sched->tasks[i];
        if (!task->enabled || task->func == NULL) continue;                   //跳过禁用或空槽位

        uint32_t elapsed = now - task->last_run;
        if (elapsed >= task->interval) {
            task->last_run += task->interval;                                 //按周期累加,避免累积误差

            //若错过多个周期,直接对齐到当前时刻,防止追赶式补偿导致任务堆积
            if (now - task->last_run >= task->interval) {
                task->last_run = now;
            }

            //若任务返回false,自动禁用,适用于一次性任务或错误终止
            if (!task->func(task->context)) {
                task->enabled = false;
            }
        }
    }
}
