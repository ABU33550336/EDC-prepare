#ifndef HW_ADAPTER_H
#define HW_ADAPTER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HW_OK                   0
#define HW_ERR                  1

#define HW_MOTOR_CH_LEFT        0
#define HW_MOTOR_CH_RIGHT       1

#define HW_ENCODER_CH_LEFT      0
#define HW_ENCODER_CH_RIGHT     1

#define HW_LINE_SENSOR_COUNT    5

uint32_t HW_GetTick(void);
void     HW_TickInc(void);
void     HW_DelayMs(uint32_t ms);

uint8_t  HW_MotorInit(void);
void     HW_SetMotorPWM(int16_t left, int16_t right);

uint8_t  HW_EncoderInit(void);
int32_t  HW_GetEncoderCnt(uint8_t ch);
void     HW_ClearEncoderCnt(uint8_t ch);

uint8_t  HW_LineSensorInit(void);
void     HW_EncDebugReport(void);   //调试:串口打印左右轮正交解码累计脉冲
void     HW_ReadLineSensors(uint16_t *buf);

#ifdef __cplusplus
}
#endif

#endif
