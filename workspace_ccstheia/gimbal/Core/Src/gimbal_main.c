//地猛星云台驱动主程序(完全抄STM32 main.c逻辑)
#include "ti_msp_dl_config.h"
#include "Emm_V5.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define PULSES_PER_REV   3200
#define MOTOR_SPEED_RPM  4000
#define MOTOR_ACCEL      180
#define CMD_BUF_SIZE     32

volatile uint32_t g_tick = 0;

static char           g_cmd_buf[CMD_BUF_SIZE];
static volatile uint8_t  g_cmd_idx = 0;
static volatile uint32_t g_frame_tick = 0;

volatile bool rxFrameFlag  = false;
volatile bool rxFrameFlag2 = false;

void DelayMs(uint32_t ms) {
    for (volatile uint32_t d = 0; d < ms * 32000; d++);
}

//同 STM32 Motor_DriveOne
static void Motor_DriveOne(UART_Regs* uart, uint8_t addr,
    float deg, uint16_t speed_rpm, uint32_t* last)
{
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f)    deg += 360.0f;
    uint32_t target = (uint32_t)(deg * PULSES_PER_REV / 360.0f);
    if (target == *last) return;
    uint32_t dist = (target > *last) ? (target - *last) : (*last - target);

    g_emm_uart = uart;
    if (uart == MTR1_UART_INST) rxFrameFlag  = false;
    else                         rxFrameFlag2 = false;
    Emm_V5_Pos_Control(addr, 0, speed_rpm, MOTOR_ACCEL, target, 1, 0);

    uint32_t wait = dist * 60000UL / (speed_rpm * PULSES_PER_REV);
    wait = wait * 3 / 2 + 100;
    DelayMs(wait);
    *last = target;
}

//同 STM32 Motor_GoToAngle2
static void Motor_GoToAngle2(float thetaL, float thetaR, uint16_t speed_rpm)
{
    static uint32_t lastL = 0xFFFFFFFF, lastR = 0xFFFFFFFF;
    Motor_DriveOne(MTR1_UART_INST, 1, thetaL, speed_rpm, &lastL);
    Motor_DriveOne(MTR2_UART_INST, 1, thetaR, speed_rpm, &lastR);
}

void SysTick_Handler(void)
{
    g_tick++;
}

//同 STM32 USART3_IRQHandler,逐字节收
void UART0_IRQHandler(void)
{
    if (!DL_UART_isRXFIFOEmpty(CMD_UART_INST)) {
        uint8_t byte = DL_UART_Main_receiveData(CMD_UART_INST);
        if (g_cmd_idx < CMD_BUF_SIZE - 1)
            g_cmd_buf[g_cmd_idx++] = (char)byte;
        g_frame_tick = g_tick + 50;
    }
}

//同 STM32 USART1_IRQHandler,IDLE→置flag
void UART1_IRQHandler(void)
{
    if (!DL_UART_isRXFIFOEmpty(MTR1_UART_INST)) {
        DL_UART_Main_receiveData(MTR1_UART_INST);
        rxFrameFlag = true;
    }
}

//同 STM32 USART2_IRQHandler
void UART3_IRQHandler(void)
{
    if (!DL_UART_isRXFIFOEmpty(MTR2_UART_INST)) {
        DL_UART_Main_receiveData(MTR2_UART_INST);
        rxFrameFlag2 = true;
    }
}

//同 STM32 main
int main(void)
{
    SYSCFG_DL_init();
    SysTick_Config(CPUCLK_FREQ / 1000);

    //使能中断(关FIFO,逐字节收,同STM32 RXNE)
    DL_UART_Main_disableFIFOs(CMD_UART_INST);
    DL_UART_enableInterrupt(CMD_UART_INST, DL_UART_INTERRUPT_RX);
    NVIC_EnableIRQ(CMD_UART_INST_INT_IRQN);

    DL_UART_enableInterrupt(MTR1_UART_INST, DL_UART_INTERRUPT_RX);
    NVIC_EnableIRQ(MTR1_UART_INST_INT_IRQN);
    DL_UART_enableInterrupt(MTR2_UART_INST, DL_UART_INTERRUPT_RX);
    NVIC_EnableIRQ(MTR2_UART_INST_INT_IRQN);

    DelayMs(500); //同 STM32 HAL_Delay(500)

    //使能电机1 (同 STM32)
    g_emm_uart = MTR1_UART_INST;
    Emm_V5_En_Control(1, 1, 0);
    {
        uint32_t t = g_tick;
        while (!rxFrameFlag && (g_tick - t) < 500);
        rxFrameFlag = false;
    }
    DelayMs(100);

    //使能电机2 (同 STM32)
    g_emm_uart = MTR2_UART_INST;
    Emm_V5_En_Control(1, 1, 0);
    {
        uint32_t t = g_tick;
        while (!rxFrameFlag2 && (g_tick - t) < 500);
        rxFrameFlag2 = false;
    }
    DelayMs(100);

    //同 STM32 主循环
    while (1)
    {
        if (g_frame_tick && g_cmd_idx > 0 && g_tick >= g_frame_tick) {
            g_cmd_buf[g_cmd_idx] = '\0';
            g_cmd_idx   = 0;
            g_frame_tick = 0;
            if (strchr(g_cmd_buf, ',') != NULL) {
                float thetaL = 0.0f, thetaR = 0.0f;
                sscanf(g_cmd_buf, "%f,%f", &thetaL, &thetaR);
                Motor_GoToAngle2(thetaL, thetaR, MOTOR_SPEED_RPM);
            } else {
                float angle = (float)atof(g_cmd_buf);
                Motor_GoToAngle2(angle, angle, MOTOR_SPEED_RPM);
            }
        }
    }
}
