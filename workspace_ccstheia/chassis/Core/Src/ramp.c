//斜坡函数模块,输出平滑过渡至目标值
#include "ramp.h"
#include <stddef.h>

/**
 * @brief 初始化斜坡发生器,步长取绝对值,初始化位置设为中点
 * @param ramp 斜坡发生器实例指针
 * @param step 每周期最大步长
 * @param min 输出下限
 * @param max 输出上限
 */
void Ramp_Init(Ramp_t *ramp, float step, float min, float max)
{
    if (ramp == NULL) return;
    if (step < 0.0f) step = -step;          //步长取绝对值
    ramp->step = (step > 0.0f) ? step : 1.0f;  //步长为零时默认1,防止卡死
    if (max < min) {
        float tmp = min;
        min = max;
        max = tmp;
    }
    ramp->min = min;
    ramp->max = max;
    ramp->current = (min + max) * 0.5f;
}

/**
 * @brief 设置斜坡发生器的步长
 * @param ramp 斜坡发生器实例指针
 * @param step 每周期最大步长
 */
void Ramp_SetStep(Ramp_t *ramp, float step)
{
    if (ramp == NULL) return;
    if (step < 0.0f) step = -step;          //步长取绝对值
    ramp->step = (step > 0.0f) ? step : 1.0f;  //步长为零时默认1,防止卡死
}

/**
 * @brief 复位斜坡发生器输出到指定值(自动限幅)
 * @param ramp 斜坡发生器实例指针
 * @param value 目标初始值
 */
void Ramp_Reset(Ramp_t *ramp, float value)
{
    if (ramp == NULL) return;
    if (value < ramp->min) value = ramp->min;
    if (value > ramp->max) value = ramp->max;
    ramp->current = value;
}

/**
 * @brief 斜坡更新,每周期向目标值逼近一步
 * @param ramp 斜坡发生器实例指针
 * @param target 目标值
 * @return 当前输出值
 */
float Ramp_Update(Ramp_t *ramp, float target)
{
    if (ramp == NULL) return 0.0f;
    if (target < ramp->min) target = ramp->min;  //限幅目标值到合法范围
    if (target > ramp->max) target = ramp->max;

    float diff = target - ramp->current;          //到目标值的剩余距离
    if (diff > ramp->step) {                      //正向超出步长,只前进一个步长
        ramp->current += ramp->step;
    } else if (diff < -ramp->step) {              //负向超出步长,只后退一个步长
        ramp->current -= ramp->step;
    } else {                                      //在步长范围内,直接到达目标
        ramp->current = target;
    }
    return ramp->current;
}
