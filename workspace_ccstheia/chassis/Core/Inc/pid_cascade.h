//串级PID控制器,外环+内环
#ifndef PID_CASCADE_H
#define PID_CASCADE_H

#include "pid_pos.h"

typedef struct {
    PID_Pos_t outer;   //外环PID,通常为位置环或速度环
    PID_Pos_t inner;   //内环PID,通常为电流环或速度环
} PID_Cascade_t;

void PID_Cascade_Init(PID_Cascade_t *csc);                                             //初始化串级PID
void PID_Cascade_SetOuterParam(PID_Cascade_t *csc, float kp, float ki,
                               float kd);                                              //设置外环PID参数
void PID_Cascade_SetInnerParam(PID_Cascade_t *csc, float kp, float ki,
                               float kd);                                              //设置内环PID参数
void PID_Cascade_Reset(PID_Cascade_t *csc);                                            //重置内外环PID状态
float PID_Cascade_Update(PID_Cascade_t *csc, float outer_target,
                         float outer_feedback, float inner_feedback);                  //执行一次串级PID运算

#endif
