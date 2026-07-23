//有限状态机模块,基于状态-事件表驱动状态切换

#include "fsm.h"
#include <stddef.h>

/**
 * @brief 初始化状态机实例
 * @param fsm 状态机指针
 * @param init_state 初始状态
 * @param transitions 状态转移表
 * @param count 转移表条目数
 * @param context 用户上下文(传递给action回调)
 * @note 调用前需保证transitions数组生命周期长于fsm
 *        调用者:
 */
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

/**
 * @brief 分发事件,触发状态转移
 * @param fsm 状态机指针
 * @param event 待处理事件
 * @return 0=转移成功,1=无匹配转移或参数无效
 * @note 转移成功时执行action回调(若有)
 *        调用者:
 */
uint8_t FSM_Dispatch(FSM_t *fsm, FSM_Event_t event)
{
    if (fsm == NULL || fsm->transitions == NULL) return 1;

    for (uint16_t i = 0; i < fsm->transition_count; i++) {
        const FSM_Transition_t *t = &fsm->transitions[i];
        if (t->src_state == fsm->current_state && t->event == event) {
            fsm->current_state = t->dst_state;                                //切换到目标状态
            if (t->action != NULL) {
                t->action(fsm->context);                                      //执行转移动作(若配置)
            }
            return 0;
        }
    }
    return 1;                                                                 //未找到匹配转移
}

/**
 * @brief 获取当前状态
 * @param fsm 状态机指针
 * @return 当前状态值(参数无效时返回0)
 * @note
 *        调用者:
 */
FSM_State_t FSM_GetCurrentState(FSM_t *fsm)
{
    if (fsm == NULL) return 0;
    return fsm->current_state;
}
