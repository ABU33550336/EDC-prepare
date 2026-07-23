//一阶低通滤波器模块,递归型IIR滤波
#include "lpf.h"
#include <stddef.h>

/**
 * @brief 初始化一阶低通滤波器,alpha钳位到[0,1]
 * @param lpf 滤波器实例指针
 * @param alpha 滤波系数(0~1,越大越跟随输入)
 */
void LPF_Init(LPF_t *lpf, float alpha)
{
    if (lpf == NULL) return;
    if (alpha < 0.0f) alpha = 0.0f;          //钳位alpha到合法范围[0,1]
    if (alpha > 1.0f) alpha = 1.0f;
    lpf->alpha = alpha;
    lpf->last_out = 0.0f;                    //初始输出为零
}

/**
 * @brief 设置低通滤波器滤波系数
 * @param lpf 滤波器实例指针
 * @param alpha 滤波系数(0~1,越大越跟随输入)
 */
void LPF_SetAlpha(LPF_t *lpf, float alpha)
{
    if (lpf == NULL) return;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    lpf->alpha = alpha;
}

/**
 * @brief 复位低通滤波器上次输出值
 * @param lpf 滤波器实例指针
 * @param value 初始输出值
 */
void LPF_Reset(LPF_t *lpf, float value)
{
    if (lpf == NULL) return;
    lpf->last_out = value;
}

/**
 * @brief 一阶低通滤波,alpha越高越跟随输入
 * @param lpf 滤波器实例指针
 * @param input 当前输入值
 * @return 滤波后输出值
 */
float LPF_Update(LPF_t *lpf, float input)
{
    if (lpf == NULL) return input;
    float out = lpf->alpha * input + (1.0f - lpf->alpha) * lpf->last_out;
    lpf->last_out = out;
    return out;
}
