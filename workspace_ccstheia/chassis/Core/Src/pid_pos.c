//位置式PID控制器模块,含积分分离/死区功能
#include "pid_pos.h"
#include <stddef.h>
#include <math.h>

/**
 * @brief 初始化位置式PID控制器,清空积分/误差/限幅/死区
 * @param pid PID控制器实例指针
 */
void PID_Pos_Init(PID_Pos_t *pid)
{
    if (pid == NULL) return;
    pid->kp            = 0.0f;
    pid->ki            = 0.0f;
    pid->kd            = 0.0f;
    pid->integral      = 0.0f;
    pid->last_error    = 0.0f;
    pid->integral_limit  = 0.0f;
    pid->output_limit    = 0.0f;
    pid->sep_threshold = 0.0f;
    pid->sep_enable    = false;
    pid->deadband      = 0.0f;
}

/**
 * @brief 设置位置式PID的比例/积分/微分系数
 * @param pid PID控制器实例指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void PID_Pos_SetParam(PID_Pos_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief 设置位置式PID积分限幅和输出限幅
 * @param pid PID控制器实例指针
 * @param integral_limit 积分限幅值(<=0不限幅)
 * @param output_limit 输出限幅值(<=0不限幅)
 */
void PID_Pos_SetLimit(PID_Pos_t *pid, float integral_limit,
                      float output_limit)
{
    if (pid == NULL) return;
    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;
}

/**
 * @brief 设置积分分离阈值,误差超过阈值时停止积分
 * @param pid PID控制器实例指针
 * @param threshold 积分分离阈值(<=0关闭分离)
 */
void PID_Pos_SetSepThreshold(PID_Pos_t *pid, float threshold)
{
    if (pid == NULL) return;
    if (threshold > 0.0f) {
        pid->sep_threshold = threshold;
        pid->sep_enable    = true;
    } else {
        pid->sep_threshold = 0.0f;
        pid->sep_enable    = false;
    }
}

/**
 * @brief 设置位置式PID死区,误差小于死区时输出零
 * @param pid PID控制器实例指针
 * @param deadband 死区宽度(负值自动归零)
 */
void PID_Pos_SetDeadband(PID_Pos_t *pid, float deadband)
{
    if (pid == NULL) return;
    if (deadband < 0.0f) deadband = 0.0f;
    pid->deadband = deadband;
}

/**
 * @brief 复位位置式PID,清空积分和上次误差
 * @param pid PID控制器实例指针
 */
void PID_Pos_Reset(PID_Pos_t *pid)
{
    if (pid == NULL) return;
    pid->integral   = 0.0f;
    pid->last_error = 0.0f;
}

/**
 * @brief 位置式PID计算,含积分分离/死区/输出限幅
 * @param pid PID控制器实例指针
 * @param target 目标值
 * @param feedback 反馈值
 * @return 控制输出
 */
float PID_Pos_Update(PID_Pos_t *pid, float target, float feedback)
{
    if (pid == NULL) return 0.0f;

    float error = target - feedback;

    if (pid->deadband > 0.0f && fabsf(error) < pid->deadband) {  //误差在死区内则输出零,防止频繁动作
        return 0.0f;
    }

    if (!pid->sep_enable || fabsf(error) <= pid->sep_threshold) {  //积分分离,大误差时停止积分防超调
        pid->integral += error;              //累积积分项
        if (pid->integral_limit > 0.0f) {    //积分限幅,防止积分饱和
            if (pid->integral > pid->integral_limit) {
                pid->integral = pid->integral_limit;
            } else if (pid->integral < -pid->integral_limit) {
                pid->integral = -pid->integral_limit;
            }
        }
    }

    float derivative = error - pid->last_error;  //微分项(误差变化率)
    pid->last_error = error;

    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative;

    if (pid->output_limit > 0.0f) {          //输出限幅,防止执行机构超限
        if (output > pid->output_limit) {
            output = pid->output_limit;
        } else if (output < -pid->output_limit) {
            output = -pid->output_limit;
        }
    }

    return output;
}
