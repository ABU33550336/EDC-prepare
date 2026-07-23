//有限状态机,事件驱动的状态切换框架
#ifndef FSM_H
#define FSM_H

#include <stdint.h>

typedef uint8_t FSM_State_t;  //状态类型
typedef uint8_t FSM_Event_t;  //事件类型

//状态转移表项:源状态+事件→目标状态+动作
typedef struct {
    FSM_State_t src_state;       //源状态
    FSM_Event_t event;           //触发事件
    FSM_State_t dst_state;       //目标状态
    void (*action)(void*);       //转移时执行的动作函数
} FSM_Transition_t;

//状态机主结构体
typedef struct {
    FSM_State_t          current_state;     //当前所处状态
    uint16_t             transition_count;  //转移表项数
    const FSM_Transition_t *transitions;    //转移表指针
    void                *context;           //用户上下文,传入action函数
} FSM_t;

void    FSM_Init(FSM_t *fsm, FSM_State_t init_state,
                 const FSM_Transition_t *transitions,
                 uint16_t count, void *context);             //初始化状态机,设置初始状态和转移表
uint8_t FSM_Dispatch(FSM_t *fsm, FSM_Event_t event);        //派发事件,触发状态转移,返回是否找到对应转移
FSM_State_t FSM_GetCurrentState(FSM_t *fsm);                 //获取当前状态

#endif
