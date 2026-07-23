//单位转换模块,角度/脉宽/占空比换算
#include "unit_conv.h"
#include <stddef.h>
#include "math_utils.h"
#include <stddef.h>
#include <math.h>

/**
 * @brief 角度转弧度
 * @param deg 角度值
 * @return 弧度值
 */
float UNIT_DegToRad(float deg)
{
    return deg * (PI_F / 180.0f);
}

/**
 * @brief 弧度转角度
 * @param rad 弧度值
 * @return 角度值
 */
float UNIT_RadToDeg(float rad)
{
    return rad * (180.0f / PI_F);
}

/**
 * @brief 舵机脉宽转角度,先限幅再线性映射
 * @param pulse_us 脉宽,us
 * @param min_us 脉宽下限,us
 * @param max_us 脉宽上限,us
 * @param angle_min 角度下限
 * @param angle_max 角度上限
 * @return 角度值
 */
float UNIT_PulseToAngle(uint16_t pulse_us, uint16_t min_us,
                        uint16_t max_us, float angle_min,
                        float angle_max)
{
    if (max_us <= min_us) return angle_min;  //无效脉宽范围,直接返回最小角度
    uint16_t pulse = MATH_LimitInt((int32_t)pulse_us,
                                   (int32_t)min_us, (int32_t)max_us);
    return MATH_Map((float)pulse, (float)min_us, (float)max_us,
                    angle_min, angle_max);
}

/**
 * @brief 角度转舵机脉宽,自动限幅后线性映射并四舍五入
 * @param angle_deg 角度值
 * @param min_us 脉宽下限,us
 * @param max_us 脉宽上限,us
 * @param angle_min 角度下限
 * @param angle_max 角度上限
 * @return 脉宽,us
 */
uint16_t UNIT_AngleToPulse(float angle_deg, uint16_t min_us,
                            uint16_t max_us, float angle_min,
                            float angle_max)
{
    if (fabsf(angle_max - angle_min) < 1e-12f) {    //角度范围过小,返回中点脉宽
        return (uint16_t)((min_us + max_us) / 2);
    }
    float angle = MATH_Limit(angle_deg, angle_min, angle_max);
    float pulse = MATH_Map(angle, angle_min, angle_max,
                           (float)min_us, (float)max_us);
    return (uint16_t)(pulse + 0.5f);
}

/**
 * @brief 占空比(0~1)转百分比(0~100)
 * @param duty 占空比值
 * @return 百分比值
 */
float UNIT_DutyCycleToPercent(float duty)
{
    return MATH_Limit(duty, 0.0f, 1.0f) * 100.0f;
}

/**
 * @brief 百分比(0~100)转占空比(0~1)
 * @param percent 百分比值
 * @return 占空比值
 */
float UNIT_PercentToDutyCycle(float percent)
{
    return MATH_Limit(percent, 0.0f, 100.0f) / 100.0f;
}
