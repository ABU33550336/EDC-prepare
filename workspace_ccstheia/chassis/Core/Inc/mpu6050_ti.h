#ifndef MPU6050_TI_H
#define MPU6050_TI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t MPU6050_Init(void);
uint8_t MPU6050_ReadGyroDPS(float *x, float *y, float *z);

#ifdef __cplusplus
}
#endif

#endif
