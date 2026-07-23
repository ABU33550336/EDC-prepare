//差分驱动模型,线速度角速度与轮速的换算
#ifndef DIFF_DRIVE_H
#define DIFF_DRIVE_H

#include <stdint.h>

typedef struct {
    float wheel_base_m;     //轮距,m
    float wheel_radius_m;   //车轮半径,m
} DiffDrive_t;

void  DiffDrive_Init(DiffDrive_t *dd, float wheel_base_m,
                     float wheel_radius_m);                  //初始化差分驱动模型
void  DiffDrive_WheelSpeeds(DiffDrive_t *dd, float linear_v_ms,
                            float angular_v_rads,
                            float *left_speed,
                            float *right_speed);             //根据目标线速度和角速度计算左右轮速

#endif
