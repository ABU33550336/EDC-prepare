//云台步进电机控制库头文件
//云台步进电机控制库,适配ZDT_X42S闭环步进电机(张大头)
//支持两种模式: UART(0x6B协议) / STEP脉冲
#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_OK            0       //正常
#define SERVO_ERR           1       //错误
#define SERVO_BUSY          2       //忙

#define SERVO_MODE_UART     0       //UART串口控制
#define SERVO_MODE_PULSE    1       //脉冲步进控制

#define SERVO_ADDR          1       //电机UART地址
#define SERVO_BAUD          115200  //UART波特率,bps
#define SERVO_STEPS_PER_REV 3200   //电机每转步数,细分后总步数

//优先复用CLI的UART实例,需在#include"debug_cli.h"之后包含本文件
//若debug_cli.h尚未被包含(包含顺序问题),则退化为默认UART_0_INST
#ifdef CLI_UART_INST
  #define SERVO_UART_INST    CLI_UART_INST
#else
  #define SERVO_UART_INST    UART_0_INST
#endif

uint8_t  Servo_Init(uint8_t mode);
float    Servo_GetAngle(void);
void     Servo_SetSpeed(uint16_t rpm);
void     Servo_SetAccel(uint16_t rpm_per_s);
void     Servo_Stop(void);
uint8_t  Servo_Status(void);
void     Servo_StepIRQ(void);

#ifdef __cplusplus
}
#endif

#endif
