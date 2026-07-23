//单位转换工具,角度/占空比/脉宽换算
#ifndef UNIT_CONV_H
#define UNIT_CONV_H

#include <stdint.h>

#define PI_F 3.14159265f                                                           //圆周率近似值

float UNIT_DegToRad(float deg);                                                    //角度转弧度
float UNIT_RadToDeg(float rad);                                                    //弧度转角度

float UNIT_PulseToAngle(uint16_t pulse_us, uint16_t min_us,
                        uint16_t max_us, float angle_min,
                        float angle_max);                                           //脉宽映射为角度值,用于舵机

uint16_t UNIT_AngleToPulse(float angle_deg, uint16_t min_us,
                           uint16_t max_us, float angle_min,
                           float angle_max);                                        //角度值转换为脉宽,用于舵机控制

float UNIT_DutyCycleToPercent(float duty);                                          //占空比转换为百分比
float UNIT_PercentToDutyCycle(float percent);                                       //百分比转换为占空比

#endif
