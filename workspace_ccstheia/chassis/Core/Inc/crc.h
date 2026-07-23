//CRC校验算法,CRC8和CRC16
#ifndef CRC_H
#define CRC_H

#include <stdint.h>

uint8_t  CRC8_Compute(const uint8_t *data, uint16_t len);                   //计算CRC8
uint8_t  CRC8_ComputeWithInit(const uint8_t *data, uint16_t len,
                              uint8_t init);                                 //计算CRC8,指定初始值
uint16_t CRC16_Compute(const uint8_t *data, uint16_t len);                   //计算CRC16
uint16_t CRC16_ComputeWithInit(const uint8_t *data, uint16_t len,
                               uint16_t init);                               //计算CRC16,指定初始值

#endif
