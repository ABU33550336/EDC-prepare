//斜坡发生器,用于缓启动/缓停止,防止突变
#ifndef RAMP_H
#define RAMP_H

#include <stdint.h>

typedef struct {
    float current;  //当前输出值
    float step;     //每步变化量
    float min;      //最小值下限
    float max;      //最大值上限
} Ramp_t;

void Ramp_Init(Ramp_t *ramp, float step, float min, float max);    //初始化斜坡发生器
void Ramp_SetStep(Ramp_t *ramp, float step);                        //设置步长
void Ramp_Reset(Ramp_t *ramp, float value);                         //重置当前值
float Ramp_Update(Ramp_t *ramp, float target);                      //执行一次斜坡逼近,返回当前值

#endif
