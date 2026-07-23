//环形缓冲区,无锁FIFO实现
#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t *buffer;    //数据缓冲区指针
    uint16_t size;      //缓冲区容量
    uint16_t head;      //写指针
    uint16_t tail;      //读指针
    bool     full;      //满标志,区分空和满状态
} RingBuf_t;

uint8_t  RingBuf_Init(RingBuf_t *rb, uint8_t *buf, uint16_t size);     //初始化环形缓冲区
bool     RingBuf_IsEmpty(RingBuf_t *rb);                                //判断缓冲区是否为空
bool     RingBuf_IsFull(RingBuf_t *rb);                                 //判断缓冲区是否已满
uint16_t RingBuf_Count(RingBuf_t *rb);                                  //获取缓冲区中有效数据个数
uint16_t RingBuf_Free(RingBuf_t *rb);                                   //获取剩余空间
uint8_t  RingBuf_Put(RingBuf_t *rb, uint8_t data);                      //写入一个字节
uint8_t  RingBuf_Get(RingBuf_t *rb, uint8_t *data);                     //读取一个字节
uint8_t  RingBuf_PutArray(RingBuf_t *rb, const uint8_t *src,
                          uint16_t len);                                //批量写入数据
uint16_t RingBuf_GetArray(RingBuf_t *rb, uint8_t *dst, uint16_t len);  //批量读取数据
void     RingBuf_Reset(RingBuf_t *rb);                                  //重置缓冲区

#endif
