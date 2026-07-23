//互补滤波器,融合加速度计和陀螺仪求姿态角
#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include <stdint.h>

typedef struct {
    float angle;    //融合后的角度估计值
    float bias;     //陀螺仪零偏估计
    float alpha;    //互补系数,alpha越大越信任陀螺仪
} Complementary_t;

void Complementary_Init(Complementary_t *cf, float alpha);                 //初始化互补滤波器
void Complementary_SetAlpha(Complementary_t *cf, float alpha);             //更新互补系数
void Complementary_Reset(Complementary_t *cf, float init_angle);           //重置滤波器角度
float Complementary_Update(Complementary_t *cf, float angle_acc,
                           float rate_gyro, float dt);                     //执行一次互补滤波,返回角度估计值

#endif
