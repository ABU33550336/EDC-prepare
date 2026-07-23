//滑动平均滤波器模块,循环缓冲区实现
#include "moving_avg.h"
#include <stddef.h>

/**
 * @brief 初始化滑动平均滤波器,清空缓冲区
 * @param ma 滤波器实例指针
 * @param buf 外部环形缓冲区
 * @param size 窗口大小
 * @return 成功返回0,参数无效返回1
 */
uint8_t MovingAvg_Init(MovingAvg_t *ma, float *buf, uint16_t size)
{
    if (ma == NULL || buf == NULL || size == 0) return 1;
    ma->buffer = buf;
    ma->size   = size;
    ma->index  = 0;
    ma->sum    = 0.0f;
    ma->count  = 0;
    for (uint16_t i = 0; i < size; i++) {
        buf[i] = 0.0f;
    }
    return 0;
}

/**
 * @brief 复位滑动平均滤波器,清空缓冲区和累加和
 * @param ma 滤波器实例指针
 */
void MovingAvg_Reset(MovingAvg_t *ma)
{
    if (ma == NULL || ma->buffer == NULL) return;
    ma->index = 0;
    ma->sum   = 0.0f;
    ma->count = 0;
    for (uint16_t i = 0; i < ma->size; i++) {
        ma->buffer[i] = 0.0f;
    }
}

/**
 * @brief 滑动平均滤波,用新数据替换最旧数据
 * @param ma 滤波器实例指针
 * @param input 新采样值
 * @return 滤波后的平均值
 */
float MovingAvg_Update(MovingAvg_t *ma, float input)
{
    if (ma == NULL || ma->buffer == NULL || ma->size == 0) {
        return input;
    }
    float old = ma->buffer[ma->index];       //取出最旧数据用于更新累加和
    ma->buffer[ma->index] = input;
    ma->sum = ma->sum - old + input;         //减去最旧值加上新值,维持滑动和
    ma->index++;
    if (ma->index >= ma->size) {             //环形缓冲区,回绕到起点
        ma->index = 0;
    }
    if (ma->count < ma->size) {              //窗口未填满时用当前数据量求平均
        ma->count++;
        return ma->sum / (float)ma->count;
    }
    return ma->sum / (float)ma->size;        //窗口填满后按固定窗口大小平均
}
