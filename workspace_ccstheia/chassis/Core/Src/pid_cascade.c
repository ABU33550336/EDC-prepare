//串级PID控制器模块,外环+内环嵌套结构
#include "pid_cascade.h"
#include <stddef.h>

/**
 * @brief 初始化串级PID控制器,内外环均复位
 * @param csc 串级PID实例指针
 */
void PID_Cascade_Init(PID_Cascade_t *csc)
{
    if (csc == NULL) return;
    PID_Pos_Init(&csc->outer);
    PID_Pos_Init(&csc->inner);
}

/**
 * @brief 设置串级PID外环控制器参数
 * @param csc 串级PID实例指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void PID_Cascade_SetOuterParam(PID_Cascade_t *csc, float kp, float ki,
                               float kd)
{
    if (csc == NULL) return;
    PID_Pos_SetParam(&csc->outer, kp, ki, kd);
}

/**
 * @brief 设置串级PID内环控制器参数
 * @param csc 串级PID实例指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void PID_Cascade_SetInnerParam(PID_Cascade_t *csc, float kp, float ki,
                               float kd)
{
    if (csc == NULL) return;
    PID_Pos_SetParam(&csc->inner, kp, ki, kd);
}

/**
 * @brief 复位串级PID,内外环均清空积分和误差
 * @param csc 串级PID实例指针
 */
void PID_Cascade_Reset(PID_Cascade_t *csc)
{
    if (csc == NULL) return;
    PID_Pos_Reset(&csc->outer);
    PID_Pos_Reset(&csc->inner);
}

/**
 * @brief 串级PID更新,外环输出作为内环目标
 * @param csc 串级PID实例指针
 * @param outer_target 外环目标值
 * @param outer_feedback 外环反馈值
 * @param inner_feedback 内环反馈值
 * @return 最终控制输出
 */
float PID_Cascade_Update(PID_Cascade_t *csc, float outer_target,
                         float outer_feedback, float inner_feedback)
{
    if (csc == NULL) return 0.0f;

    float inner_target = PID_Pos_Update(&csc->outer, outer_target,
                                        outer_feedback);
    float output = PID_Pos_Update(&csc->inner, inner_target,
                                  inner_feedback);
    return output;
}
