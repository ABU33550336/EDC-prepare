//中值滤波器,去除脉冲噪声
#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#include <stdint.h>

typedef struct {
    float *buffer;    //数据缓冲区
    float *sort_buf;  //排序缓冲区,用于取中值
    uint16_t size;    //窗口大小
    uint16_t index;   //当前写入位置
    uint16_t count;   //已采样数量
} MedianFilter_t;

uint8_t MedianFilter_Init(MedianFilter_t *mf, float *buf,
                          float *sort_buf, uint16_t size);              //初始化中值滤波器
void MedianFilter_Reset(MedianFilter_t *mf);                             //重置滤波器状态
float MedianFilter_Update(MedianFilter_t *mf, float input);              //输入新数据,返回中值

#endif
