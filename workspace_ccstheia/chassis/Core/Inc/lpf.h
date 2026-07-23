//一阶低通滤波器
#ifndef LPF_H
#define LPF_H

#include <stdint.h>

typedef struct {
    float alpha;     //滤波系数,0~1,越小越平滑
    float last_out;  //上一次输出值
} LPF_t;

void LPF_Init(LPF_t *lpf, float alpha);      //初始化低通滤波器
void LPF_SetAlpha(LPF_t *lpf, float alpha);   //更新滤波系数
void LPF_Reset(LPF_t *lpf, float value);      //重置滤波器状态
float LPF_Update(LPF_t *lpf, float input);    //执行一次低通滤波,返回滤波后值

#endif
