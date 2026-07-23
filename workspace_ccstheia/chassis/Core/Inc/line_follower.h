//巡线传感器偏差计算,用于循迹控制
#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <stdint.h>

#define LINE_SENSOR_COUNT 4   //灰度传感器数量

//根据传感器原始值计算偏离中心线的偏差,正值偏右,负值偏左
float LineFollower_GetDeviation(const uint16_t *sensor_values,
                                uint16_t threshold,
                                uint8_t mode);

#endif
