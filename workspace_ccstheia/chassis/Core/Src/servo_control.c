//云台步进电机控制库,UART/STEP脉冲双模式
#include "ti_msp_dl_config.h"
#include "servo_control.h"
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#define SERIAL_CTRL_ENABLED   1     //UART串口控制使能,通过TX/RX指令控制电机
#define PULSE_CTRL_ENABLED    0     //脉冲步进控制使能,通过STEP/DIR脉冲控制电机

static uint8_t  g_mode     = SERVO_MODE_UART;         //当前控制模式,默认UART无需额外接线
static uint8_t  g_status   = SERVO_OK;                //电机运行状态,初始为空闲
static float    g_current  = 0.0f;                    //当前实际角度,deg,与电机内部位置同步
static float    g_target   = 0.0f;                    //目标角度,deg,由Servo_SetAngle设定
static uint16_t g_speed    = 1500;                    //电机转速,rpm,默认值保证稳定运行
static uint16_t g_accel    = 8;                       //加速度,rpm/s,兼顾响应速度与平滑性

#define DEG_PER_STEP  (360.0f / SERVO_STEPS_PER_REV)  //每步对应角度,deg
#define RX_TIMEOUT    50000                           //UART接收超时,循环计数,防死等

#if SERIAL_CTRL_ENABLED

/**
 * @brief 通过UART发送字符串(内部函数)
 * @param str 待发送的字符串
 * @note 逐字符阻塞发送至字符串结束
 *       调用者:
 *       Servo_SetAngle,Servo_GetAngle,Servo_SetSpeed,Servo_SetAccel,Servo_Stop
 */
static void uartSendStr(const char *str)
{
    while (*str) {
        DL_UART_Main_transmitData(SERVO_UART_INST, (uint8_t)*str++);
    }
}

/**
 * @brief 通过UART接收一行字符串(内部函数)
 * @param buf 接收缓冲区
 * @param maxLen 缓冲区最大长度
 * @return true 成功收到一行数据,false 超时或无数据
 * @note 以\r或\n为行结束符,内置超时保护避免死等
 *       调用者:
 *       Servo_GetAngle
 */
static bool uartRecvLine(char *buf, uint16_t maxLen)
{
    uint16_t i = 0;                                    //缓冲区写入索引
    uint32_t t = RX_TIMEOUT;                           //超时计数器,无数据时递减
    while (i < maxLen - 1) {
        if (!DL_UART_isRXFIFOEmpty(SERVO_UART_INST)) {
            uint8_t byte = DL_UART_Main_receiveData(SERVO_UART_INST);
            if (byte == '\r' || byte == '\n') {
                if (i == 0) continue;                   //跳过行首多余的换行符
                break;                                   //收到换行符表示一行结束
            }
            buf[i++] = (char)byte;
            t = RX_TIMEOUT;                             //收到数据后重置超时计数器
        } else if (--t == 0) {
            break;                                       //超时退出,避免无限阻塞
        }
    }
    buf[i] = '\0';
    return (i > 0);
}

#endif //SERIAL_CTRL_ENABLED

#if PULSE_CTRL_ENABLED

static volatile int32_t  g_steps_remaining = 0;        //剩余步数,正数正向负数反向
static volatile uint32_t g_step_period     = 0;        //步进脉冲周期,决定转速

/**
 * @brief 步进脉冲中断处理,每步更新剩余步数与当前位置
 * @note 由定时器PWM中断触发,逐次逼近目标角度
 *       TIM_IRQHandler
 */
void Servo_StepIRQ(void)
{
    if (g_steps_remaining == 0) {
        g_status = SERVO_OK;                            //电机到位,清除忙标志
        return;
    }
    if (g_steps_remaining > 0) g_steps_remaining--;    //正向剩余步数减1
    else                       g_steps_remaining++;    //反向剩余步数加1(绝对值减1)
    g_current += (g_steps_remaining > 0) ? DEG_PER_STEP : -DEG_PER_STEP;
}

#endif //PULSE_CTRL_ENABLED

/**
 * @brief 初始化云台步进电机,选择控制模式并复位内部状态
 * @param mode 控制模式(SERVO_MODE_UART或SERVO_MODE_PULSE)
 * @return SERVO_OK
 * @note 每次调用将当前位置和目标重置为0度
 *       调用者:
 *       main
 */
uint8_t Servo_Init(uint8_t mode)
{
    g_mode   = mode;
    g_status = SERVO_OK;
    g_current = 0.0f;
    g_target  = 0.0f;
    return SERVO_OK;
}

/**
 * @brief 设定云台目标角度
 * @param deg 目标角度,deg(0~360)
 * @note UART模式下发送指令后立即更新g_current;脉冲模式下计算剩余步数启动运动
 *       调用者:
 *       应用层
 */
/**
 * @brief 读取云台当前实际角度
 * @return 当前角度,deg
 * @note UART模式下发送位置查询命令并解析"pos=xxx"响应以更新g_current
 *       调用者:
 *       应用层
 */
float Servo_GetAngle(void)
{
#if SERIAL_CTRL_ENABLED
    if (g_mode == SERVO_MODE_UART) {
        uartSendStr("?\r\n");                           //发送位置查询命令
        char resp[24];
        if (uartRecvLine(resp, sizeof(resp))) {
            if (resp[0] == 'p' && resp[1] == 'o' && resp[2] == 's' && resp[3] == '=') {
                g_current = (float)atof(resp + 4);      //解析"pos=xxx"格式的响应
            }
        }
    }
#endif
    return g_current;
}

/**
 * @brief 设定电机运行转速
 * @param rpm 目标转速,rpm
 * @note UART模式下发送速度指令至电机驱动器
 *       调用者:
 *       应用层
 */
void Servo_SetSpeed(uint16_t rpm)
{
    g_speed = rpm;
#if SERIAL_CTRL_ENABLED
    if (g_mode == SERVO_MODE_UART) {
        char buf[16];
        sprintf(buf, "s%u\r\n", (unsigned)rpm);
        uartSendStr(buf);
    }
#endif
}

/**
 * @brief 设定电机加减速时间
 * @param rpm_per_s 加速度,rpm/s
 * @note UART模式下发送加速度指令至电机驱动器
 *       调用者:
 *       应用层
 */
void Servo_SetAccel(uint16_t rpm_per_s)
{
    g_accel = rpm_per_s;
#if SERIAL_CTRL_ENABLED
    if (g_mode == SERVO_MODE_UART) {
        char buf[16];
        sprintf(buf, "a%u\r\n", (unsigned)rpm_per_s);
        uartSendStr(buf);
    }
#endif
}

/**
 * @brief 急停电机,立即停止运动
 * @note UART模式下发送stop命令;脉冲模式下清零剩余步数并置状态为空闲
 *       应用层
 */
void Servo_Stop(void)
{
#if SERIAL_CTRL_ENABLED
    if (g_mode == SERVO_MODE_UART) {
        uartSendStr("stop\r\n");
    }
#endif
#if PULSE_CTRL_ENABLED
    if (g_mode == SERVO_MODE_PULSE) {
        g_steps_remaining = 0;
        g_status = SERVO_OK;
    }
#endif
}

/**
 * @brief 查询电机当前运行状态
 * @return 状态码(SERVO_OK/SERVO_BUSY/SERVO_ERR)
 * @note 脉冲模式下由Servo_StepIRQ更新状态标志
 *       应用层
 */
uint8_t Servo_Status(void)
{
    return g_status;
}
