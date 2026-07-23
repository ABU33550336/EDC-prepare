//滑动平均滤波器
#ifndef MOVING_AVG_H
#define MOVING_AVG_H

#include <stdint.h>

typedef struct {
    float *buffer;      //环形缓冲区指针
    uint16_t size;      //缓冲区大小
    uint16_t index;     //当前写入位置
    float   sum;        //窗口内数据和,用于快速求平均
    uint16_t count;     //已采样数量,未满时有效
} MovingAvg_t;

uint8_t MovingAvg_Init(MovingAvg_t *ma, float *buf, uint16_t size);   //初始化滑动平均滤波器
void MovingAvg_Reset(MovingAvg_t *ma);                                 //重置滤波器状态
float MovingAvg_Update(MovingAvg_t *ma, float input);                  //输入新数据,返回当前平均值

#endif
