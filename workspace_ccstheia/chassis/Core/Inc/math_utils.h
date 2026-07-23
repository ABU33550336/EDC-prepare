//数学工具函数,限幅/映射/死区/符号/浮点比较
#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <stdint.h>
#include <stdbool.h>

float MATH_Limit(float value, float min, float max);                               //限幅,将value限制在[min,max]
int32_t MATH_LimitInt(int32_t value, int32_t min, int32_t max);                    //整数限幅

float MATH_Map(float value, float in_min, float in_max,
               float out_min, float out_max);                                       //将一个区间值线性映射到另一区间

float MATH_Deadzone(float value, float threshold);                                  //死区处理,小于阈值返回0

int8_t MATH_Sign(float value);                                                      //浮点数符号,-1/0/1
int8_t MATH_SignInt(int32_t value);                                                 //整数符号,-1/0/1

bool MATH_FloatEq(float a, float b, float epsilon);                                 //浮点相等判断,允许误差epsilon

#define MATH_MAX(a, b) (((a) > (b)) ? (a) : (b))                                   //取较大值
#define MATH_MIN(a, b) (((a) < (b)) ? (a) : (b))                                   //取较小值

#define MATH_ABS(x)    (((x) >= 0) ? (x) : -(x))                                   //取绝对值

#define MATH_CLAMP(value, min, max) MATH_Limit(value, min, max)                     //限幅,同MATH_Limit

#endif
