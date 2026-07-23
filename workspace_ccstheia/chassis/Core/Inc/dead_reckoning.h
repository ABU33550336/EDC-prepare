//航位推算,基于编码器+陀螺仪估算机器人位置
#ifndef DEAD_RECKONING_H
#define DEAD_RECKONING_H

#include <stdint.h>

typedef struct {
    float x_mm;              //X方向位置,mm
    float y_mm;              //Y方向位置,mm
    float heading_deg;       //航向角,度
    float wheel_base_mm;     //轮距,mm
    float last_heading_deg;  //上次航向角,用于增量计算
    uint8_t initialized;     //初始化标志
} DR_t;

void  DR_Init(DR_t *dr, float wheel_base_mm);              //初始化航位推算,设置轮距
void  DR_Reset(DR_t *dr, float init_x, float init_y,
               float init_heading);                         //重置位置和航向
void  DR_Update(DR_t *dr, float rate_degps, float delta_left_mm,
                 float delta_right_mm, float dt);            //基于角速度+里程更新位置推算
float DR_GetHeading(DR_t *dr);                              //获取当前航向角
void  DR_GetPosition(DR_t *dr, float *x, float *y);        //获取当前位置

#endif
