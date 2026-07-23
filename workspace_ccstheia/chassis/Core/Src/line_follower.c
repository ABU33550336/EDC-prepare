//巡线模块,根据传感器值计算偏离量

#include "line_follower.h"
#include <stddef.h>

static const int16_t LINE_POSITIONS[LINE_SENSOR_COUNT] = {
    -3, -1, 1, 3                                                             //4路传感器位置权重,由左至右
};

/**
 * @brief 计算巡线传感器偏离量(归一化百分比)
 * @param sensor_values 各传感器原始值或二值化标志
 * @param threshold 二值化阈值(仅在mode=1时使用)
 * @param mode 加权模式(0:模拟量加权,1:二值化加权)
 * @return 偏离度百分比(-100~+100),负值偏左,正值偏右
 * @note 加权和除以总权重后归一化到±100
 *        调用者:
 */
float LineFollower_GetDeviation(const uint16_t *sensor_values,
                                uint16_t threshold,
                                uint8_t mode)
{
    if (sensor_values == NULL) return 0.0f;

    float weighted_sum = 0.0f;                                                //加权位置总和
    float total_weight = 0.0f;                                                //总权重(归一化分母)

    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        float weight = 0.0f;

        if (mode == 0) {
            weight = (float)sensor_values[i];                                 //模拟量模式:直接用原始值作为权重
        } else {
            weight = (sensor_values[i] >= threshold) ? 1.0f : 0.0f;           //二值化模式:超过阈值视为检测到线
        }

        weighted_sum += (float)LINE_POSITIONS[i] * weight;
        total_weight += weight;
    }

    //防止除零,所有传感器均未检测到线时返回0
    if (total_weight < 0.001f) {
        return 0.0f;
    }

    //归一化到-100~+100,除以3.0是因为位置范围为-3~+3
    float deviation = (weighted_sum / total_weight) / 3.0f * 100.0f;

    //限幅,防止积分饱和或超界输出
    if (deviation > 100.0f) deviation = 100.0f;
    if (deviation < -100.0f) deviation = -100.0f;

    return deviation;
}
