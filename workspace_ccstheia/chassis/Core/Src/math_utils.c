//数学工具模块,数值限幅/重映射/死区/符号判断/浮点容差比较
#include "math_utils.h"
#include <stddef.h>
#include <math.h>

/**
 * @brief 对浮点数进行限幅,当min>max时自动交换
 * @param value 输入值
 * @param min 下限
 * @param max 上限
 * @return 限幅后的值
 */
float MATH_Limit(float value, float min, float max)
{
    if (min > max) {           //确保min<=max,防止限幅反转
        float tmp = min;
        min = max;
        max = tmp;
    }
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief 对整型数进行限幅,当min>max时自动交换
 * @param value 输入值
 * @param min 下限
 * @param max 上限
 * @return 限幅后的值
 */
int32_t MATH_LimitInt(int32_t value, int32_t min, int32_t max)
{
    if (min > max) {
        int32_t tmp = min;
        min = max;
        max = tmp;
    }
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief 线性映射,将value从输入范围映射到输出范围
 * @param value 输入值
 * @param in_min 输入范围下限
 * @param in_max 输入范围上限
 * @param out_min 输出范围下限
 * @param out_max 输出范围上限
 * @return 映射后的值
 * @note 输入范围接近零时直接返回out_min,避免除零
 *        调用者:
 */
float MATH_Map(float value, float in_min, float in_max,
               float out_min, float out_max)
{
    float in_range = in_max - in_min;        //计算输入范围
    if (fabsf(in_range) < 1e-12f) {          //防止除零,输入范围过小时返回下限
        return out_min;
    }
    float ratio = (value - in_min) / in_range;
    return out_min + ratio * (out_max - out_min);
}

/**
 * @brief 死区处理,绝对值小于阈值时输出零
 * @param value 输入值
 * @param threshold 死区阈值(自动取正)
 * @return 处理后的值
 */
float MATH_Deadzone(float value, float threshold)
{
    if (threshold < 0.0f) threshold = -threshold;  //确保死区阈值非负
    if (fabsf(value) < threshold) {
        return 0.0f;
    }
    return value;
}

/**
 * @brief 浮点数符号判断
 * @param value 输入值
 * @return 正数返回1,负数返回-1,零返回0
 */
int8_t MATH_Sign(float value)
{
    if (value > 0.0f) return 1;
    if (value < 0.0f) return -1;
    return 0;
}

/**
 * @brief 整型数符号判断
 * @param value 输入值
 * @return 正数返回1,负数返回-1,零返回0
 */
int8_t MATH_SignInt(int32_t value)
{
    if (value > 0) return 1;
    if (value < 0) return -1;
    return 0;
}

/**
 * @brief 带容差的浮点相等比较
 * @param a 比较值a
 * @param b 比较值b
 * @param epsilon 容差阈值(自动取正)
 * @return 相等返回true,否则false
 */
bool MATH_FloatEq(float a, float b, float epsilon)
{
    if (epsilon < 0.0f) epsilon = -epsilon;  //确保容差非负
    return fabsf(a - b) <= epsilon;
}
