//位置式PID控制器,带积分分离和死区
#ifndef PID_POS_H
#define PID_POS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float kp;              //比例系数
    float ki;              //积分系数
    float kd;              //微分系数
    float integral;        //积分累计值
    float last_error;      //上次偏差,用于微分计算
    float integral_limit;  //积分限幅,防止积分饱和
    float output_limit;    //输出限幅
    float sep_threshold;   //积分分离阈值,偏差过大时停止积分
    bool  sep_enable;      //积分分离使能
    float deadband;        //死区,偏差小于此值不调节
} PID_Pos_t;

void PID_Pos_Init(PID_Pos_t *pid);                                                    //初始化位置式PID结构体
void PID_Pos_SetParam(PID_Pos_t *pid, float kp, float ki, float kd);                 //设置PID参数
void PID_Pos_SetLimit(PID_Pos_t *pid, float integral_limit,
                      float output_limit);                                             //设置积分和输出限幅
void PID_Pos_SetSepThreshold(PID_Pos_t *pid, float threshold);                        //设置积分分离阈值
void PID_Pos_SetDeadband(PID_Pos_t *pid, float deadband);                             //设置死区阈值
void PID_Pos_Reset(PID_Pos_t *pid);                                                   //重置PID状态,清空积分
float PID_Pos_Update(PID_Pos_t *pid, float target, float feedback);                   //执行一次位置式PID运算,返回控制量

#endif
