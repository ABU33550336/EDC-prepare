//环形缓冲区模块,提供FIFO数据存取(支持覆盖检测)

#include "ringbuf.h"
#include <stddef.h>

/**
 * @brief 初始化环形缓冲区
 * @param rb 环形缓冲区指针
 * @param buf 存储数据的外部缓冲区
 * @param size 缓冲区大小(字节)
 * @return 0=成功,1=参数无效
 * @note head==tail且full为false表示空,head==tail且full为true表示满
 *        调用者:
 */
uint8_t RingBuf_Init(RingBuf_t *rb, uint8_t *buf, uint16_t size)
{
    if (rb == NULL || buf == NULL || size == 0) return 1;
    rb->buffer = buf;
    rb->size   = size;
    rb->head   = 0;                                                           //读指针指向第一个有效数据
    rb->tail   = 0;                                                           //写指针指向空闲位置
    rb->full   = false;                                                       //初始为空
    return 0;
}

/**
 * @brief 判断缓冲区是否为空
 * @param rb 环形缓冲区指针
 * @return true=空,false=非空
 * @note head==tail且未满即为空
 *        调用者:
 */
bool RingBuf_IsEmpty(RingBuf_t *rb)
{
    if (rb == NULL) return true;
    return (rb->head == rb->tail) && !rb->full;
}

/**
 * @brief 判断缓冲区是否已满
 * @param rb 环形缓冲区指针
 * @return true=满,false=未满
 * @note 使用full标志而非head==tail,因为head==tail也代表空
 *        调用者:
 */
bool RingBuf_IsFull(RingBuf_t *rb)
{
    if (rb == NULL) return true;
    return rb->full;
}

/**
 * @brief 获取缓冲区中有效数据个数
 * @param rb 环形缓冲区指针
 * @return 有效数据字节数
 * @note 区分tail>=head和tail<head两种回绕情况
 *        调用者:
 */
uint16_t RingBuf_Count(RingBuf_t *rb)
{
    if (rb == NULL) return 0;
    if (rb->full) return rb->size;                                            //满时返回容量
    if (rb->tail >= rb->head) {
        return rb->tail - rb->head;                                           //写指针未回绕
    }
    return rb->size - (rb->head - rb->tail);                                  //写指针已回绕
}

/**
 * @brief 获取缓冲区空闲空间
 * @param rb 环形缓冲区指针
 * @return 空闲字节数
 * @note
 *        调用者:
 */
uint16_t RingBuf_Free(RingBuf_t *rb)
{
    if (rb == NULL) return 0;
    return rb->size - RingBuf_Count(rb);
}

/**
 * @brief 向缓冲区写入一个字节
 * @param rb 环形缓冲区指针
 * @param data 待写入字节
 * @return 0=成功,1=缓冲区满或参数无效
 * @note 满时不覆盖旧数据,返回失败
 *        调用者:
 */
uint8_t RingBuf_Put(RingBuf_t *rb, uint8_t data)
{
    if (rb == NULL) return 1;
    if (rb->full) return 1;                                                   //缓冲区满,丢弃数据
    rb->buffer[rb->tail] = data;
    rb->tail++;
    if (rb->tail >= rb->size) rb->tail = 0;                                   //回绕到起始位置
    if (rb->tail == rb->head) rb->full = true;                                //写指针追上读指针,标记满
    return 0;
}

/**
 * @brief 从缓冲区读取一个字节
 * @param rb 环形缓冲区指针
 * @param data 输出读取的字节
 * @return 0=成功,1=缓冲区空或参数无效
 * @note 读取后清除full标志
 *        调用者:
 */
uint8_t RingBuf_Get(RingBuf_t *rb, uint8_t *data)
{
    if (rb == NULL || data == NULL) return 1;
    if (RingBuf_IsEmpty(rb)) return 1;                                        //缓冲区空,无可读数据
    *data = rb->buffer[rb->head];
    rb->head++;
    if (rb->head >= rb->size) rb->head = 0;                                   //回绕到起始位置
    rb->full = false;                                                         //一旦读取,一定未满
    return 0;
}

/**
 * @brief 批量写入多个字节
 * @param rb 环形缓冲区指针
 * @param src 源数据指针
 * @param len 写入长度
 * @return 0=成功,1=空间不足或参数无效
 * @note 中间任一字节写入失败即终止
 *        调用者:
 */
uint8_t RingBuf_PutArray(RingBuf_t *rb, const uint8_t *src,
                         uint16_t len)
{
    if (rb == NULL || src == NULL) return 1;
    for (uint16_t i = 0; i < len; i++) {
        if (RingBuf_Put(rb, src[i]) != 0) return 1;                           //空间不足,中止写入
    }
    return 0;
}

/**
 * @brief 批量读取多个字节
 * @param rb 环形缓冲区指针
 * @param dst 目标缓冲区指针
 * @param len 期望读取长度
 * @return 实际读取字节数(可能小于len)
 * @note 读到空时自动停止,不会阻塞
 *        调用者:
 */
uint16_t RingBuf_GetArray(RingBuf_t *rb, uint8_t *dst, uint16_t len)
{
    if (rb == NULL || dst == NULL) return 0;
    uint16_t read_cnt = 0;
    for (uint16_t i = 0; i < len; i++) {
        if (RingBuf_Get(rb, &dst[i]) != 0) break;                             //缓冲区空时退出
        read_cnt++;
    }
    return read_cnt;
}

/**
 * @brief 重置缓冲区(丢弃所有数据)
 * @param rb 环形缓冲区指针
 * @note 将head和tail归零并清除full标志
 *        调用者:
 */
void RingBuf_Reset(RingBuf_t *rb)
{
    if (rb == NULL) return;
    rb->head = 0;
    rb->tail = 0;
    rb->full = false;
}
