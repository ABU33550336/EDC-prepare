//中值滤波器模块,插入排序取中值
#include "median_filter.h"
#include <stddef.h>
#include <string.h>

/**
 * @brief 插入排序,用于中值选取(数据量小,就地排序)
 * @param arr 待排序数组
 * @param len 数组长度
 */
static void insertion_sort(float *arr, uint16_t len)
{
    for (uint16_t i = 1; i < len; i++) {
        float key = arr[i];
        int16_t j = (int16_t)i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

/**
 * @brief 初始化中值滤波器,需要外部提供缓冲区和排序缓冲区
 * @param mf 滤波器实例指针
 * @param buf 数据环形缓冲区
 * @param sort_buf 排序用临时缓冲区
 * @param size 窗口大小(至少3)
 * @return 成功返回0,参数无效返回1
 */
uint8_t MedianFilter_Init(MedianFilter_t *mf, float *buf,
                          float *sort_buf, uint16_t size)
{
    if (mf == NULL || buf == NULL || sort_buf == NULL) return 1;
    if (size < 3) return 1;
    mf->buffer   = buf;
    mf->sort_buf = sort_buf;
    mf->size     = size;
    mf->index    = 0;
    mf->count    = 0;
    for (uint16_t i = 0; i < size; i++) {
        buf[i] = 0.0f;
    }
    return 0;
}

/**
 * @brief 复位中值滤波器,清空数据缓冲区
 * @param mf 滤波器实例指针
 */
void MedianFilter_Reset(MedianFilter_t *mf)
{
    if (mf == NULL || mf->buffer == NULL) return;
    mf->index = 0;
    mf->count = 0;
    for (uint16_t i = 0; i < mf->size; i++) {
        mf->buffer[i] = 0.0f;
    }
}

/**
 * @brief 中值滤波更新,窗口未填满时返回均值,填满后返回中值
 * @param mf 滤波器实例指针
 * @param input 新采样值
 * @return 滤波后值
 * @note 取中值前复制数据到排序缓冲区,不破坏原始顺序
 *        调用者:
 */
float MedianFilter_Update(MedianFilter_t *mf, float input)
{
    if (mf == NULL || mf->buffer == NULL || mf->size < 3) {
        return input;
    }
    mf->buffer[mf->index] = input;           //写入环形缓冲区
    mf->index++;
    if (mf->index >= mf->size) {             //到达末尾则回绕
        mf->index = 0;
    }
    if (mf->count < mf->size) {
        mf->count++;
    }
    if (mf->count < mf->size) {              //窗口未填满时求平均,避免中值偏差
        float sum = 0.0f;
        for (uint16_t i = 0; i < mf->count; i++) {
            sum += mf->buffer[i];
        }
        return sum / (float)mf->count;
    }
    memcpy(mf->sort_buf, mf->buffer, mf->size * sizeof(float));  //复制到排序缓冲区,不破坏原始顺序
    insertion_sort(mf->sort_buf, mf->size);
    return mf->sort_buf[mf->size / 2];       //取排序后中间值作为滤波输出
}
