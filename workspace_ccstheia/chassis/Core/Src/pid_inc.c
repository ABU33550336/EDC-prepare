//增量式PID控制器模块,输出控制增量
#include "pid_inc.h"
#include <stddef.h>

/**
 * @brief 初始化增量式PID控制器,清空误差和限幅
 * @param pid PID控制器实例指针
 */
void PID_Inc_Init(PID_Inc_t *pid)
{
    if (pid == NULL) return;
    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->output_limit   = 0.0f;
    pid->integral_limit = 0.0f;
}

/**
 * @brief 设置增量式PID的比例/积分/微分系数
 * @param pid PID控制器实例指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void PID_Inc_SetParam(PID_Inc_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief 设置增量式PID输出限幅和积分限幅
 * @param pid PID控制器实例指针
 * @param output_limit 输出限幅值(<=0不限幅)
 * @param integral_limit 积分限幅值(<=0不限幅)
 */
void PID_Inc_SetLimit(PID_Inc_t *pid, float output_limit,
                      float integral_limit)
{
    if (pid == NULL) return;
    pid->output_limit   = output_limit;
    pid->integral_limit = integral_limit;
}

/**
 * @brief 复位增量式PID,清空历史误差
 * @param pid PID控制器实例指针
 */
void PID_Inc_Reset(PID_Inc_t *pid)
{
    if (pid == NULL) return;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
}

/**
 * @brief 增量式PID计算,输出当前周期控制增量
 * @param pid PID控制器实例指针
 * @param target 目标值
 * @param feedback 反馈值
 * @return 控制增量输出
 */
float PID_Inc_Update(PID_Inc_t *pid, float target, float feedback)
{
    if (pid == NULL) return 0.0f;

    pid->error[2] = pid->error[1];    //历史误差移位,error[0]为最新
    pid->error[1] = pid->error[0];
    pid->error[0] = target - feedback;

    float delta_p = pid->kp * (pid->error[0] - pid->error[1]);  //比例增量
    float delta_i = pid->ki * pid->error[0];                     //积分增量
    float delta_d = pid->kd * (pid->error[0] - 2.0f * pid->error[1]  //微分增量
                                + pid->error[2]);

    if (pid->integral_limit > 0.0f) {        //积分项限幅,防止积分饱和
        if (delta_i > pid->integral_limit) {
            delta_i = pid->integral_limit;
        } else if (delta_i < -pid->integral_limit) {
            delta_i = -pid->integral_limit;
        }
    }

    float output = delta_p + delta_i + delta_d;  //总控制增量

    if (pid->output_limit > 0.0f) {          //输出限幅,防止单周期变化过大
        if (output > pid->output_limit) {
            output = pid->output_limit;
        } else if (output < -pid->output_limit) {
            output = -pid->output_limit;
        }
    }

    return output;
}
