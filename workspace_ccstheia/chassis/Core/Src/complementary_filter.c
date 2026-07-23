//互补滤波器模块,融合加速度计与陀螺仪角度
#include "complementary_filter.h"
#include <stddef.h>

/**
 * @brief 初始化互补滤波器,alpha钳位到[0,1],角度和偏置归零
 * @param cf 滤波器实例指针
 * @param alpha 融合系数(越接近1越信任加速度计)
 */
void Complementary_Init(Complementary_t *cf, float alpha)
{
    if (cf == NULL) return;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    cf->alpha = alpha;
    cf->angle = 0.0f;
    cf->bias  = 0.0f;
}

/**
 * @brief 设置互补滤波器融合系数
 * @param cf 滤波器实例指针
 * @param alpha 融合系数(0~1,越接近1越信任加速度计)
 */
void Complementary_SetAlpha(Complementary_t *cf, float alpha)
{
    if (cf == NULL) return;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    cf->alpha = alpha;
}

/**
 * @brief 复位互补滤波器角度和偏置
 * @param cf 滤波器实例指针
 * @param init_angle 初始角度
 */
void Complementary_Reset(Complementary_t *cf, float init_angle)
{
    if (cf == NULL) return;
    cf->angle = init_angle;
    cf->bias  = 0.0f;
}

/**
 * @brief 互补滤波融合加速度计和陀螺仪角度
 * @param cf 滤波器实例指针
 * @param angle_acc 加速度计计算角度
 * @param rate_gyro 陀螺仪角速度
 * @param dt 采样间隔,s
 * @return 融合后角度
 * @note 陀螺仪积分后与加速度计以alpha比例融合,抑制漂移
 *        调用者:
 */
float Complementary_Update(Complementary_t *cf, float angle_acc,
                           float rate_gyro, float dt)
{
    if (cf == NULL) return 0.0f;
    if (dt <= 0.0f) return cf->angle;        //无效采样间隔时不更新,避免积分异常

    float gyro_integ = cf->angle + (rate_gyro - cf->bias) * dt;  //陀螺仪积分,减去偏置校正漂移
    cf->angle = cf->alpha * angle_acc + (1.0f - cf->alpha) * gyro_integ;  //加速度计与陀螺仪加权融合
    return cf->angle;
}
