//增量式PID控制器
#ifndef PID_INC_H
#define PID_INC_H

#include <stdint.h>

typedef struct {
    float kp;              //比例系数
    float ki;              //积分系数
    float kd;              //微分系数
    float error[3];        //三次误差历史,k/k-1/k-2
    float output_limit;    //输出限幅
    float integral_limit;  //积分限幅(暂未使用)
} PID_Inc_t;

void PID_Inc_Init(PID_Inc_t *pid);                                                //初始化增量PID结构体
void PID_Inc_SetParam(PID_Inc_t *pid, float kp, float ki, float kd);              //设置PID参数
void PID_Inc_SetLimit(PID_Inc_t *pid, float output_limit,
                      float integral_limit);                                       //设置输出和积分限幅
void PID_Inc_Reset(PID_Inc_t *pid);                                                //重置PID状态,清空历史误差
float PID_Inc_Update(PID_Inc_t *pid, float target, float feedback);               //执行一次增量PID运算,返回增量控制量

#endif
