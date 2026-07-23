//主程序,CLI交互与角度转发
#include "ti_msp_dl_config.h"
#include <stdio.h>

extern "C" {
#include "hw_adapter.h"
#include "line_follower.h"
#include "scheduler.h"
#include "debug_cli.h"
#include "speed_ctl.h"
#include "mpu6050_ti.h"
}

volatile uint32_t g_tick;

extern "C" { SpeedCtl_t g_speed_ctl; }

static uint32_t g_gyro_tick = 0;

#define SELF_TEST_DELAY_MS  1500
#define BSL_PINCM           IOMUX_PINCM40
#define CHASSIS_TEST_MS     2000

extern "C" void SysTick_Handler(void)
{
    g_tick++;
    HW_TickInc();
}

int main(void)
{
    SYSCFG_DL_init();

    DL_GPIO_initDigitalOutput(IOMUX_PINCM50);
    DL_GPIO_enableOutput(GPIOB, DL_GPIO_PIN_22);

    DL_GPIO_initDigitalInput(BSL_PINCM);
    IOMUX->SECCFG.PINCM[BSL_PINCM] |= (1 << 12) | (1 << 13);

    SysTick_Config(CPUCLK_FREQ / 1000);

    {   const char *b = "> ";
        for (const char *p = b; *p; p++) {
            DL_UART_Main_transmitData(UART_0_INST, (uint8_t)*p);
            while (DL_UART_Main_isBusy(UART_0_INST));
        }
    }

    const char *cmds[] = {"180,90\r\n", "270,270\r\n", "90,180\r\n", "0,0\r\n"};
    for (int i = 0; i < 4; i++) {
        const char *p = cmds[i];
        while (*p) {
            DL_UART_Main_transmitData(CLI_UART_INST, (uint8_t)*p);
            while (DL_UART_Main_isBusy(CLI_UART_INST));
            p++;
        }
        HW_DelayMs(SELF_TEST_DELAY_MS);
    }

    HW_MotorInit();

    //编码器GPIO输入+下拉
    DL_GPIO_initDigitalInput(IOMUX_PINCM55); IOMUX->SECCFG.PINCM[IOMUX_PINCM55] |= (1<<12);
    DL_GPIO_initDigitalInput(IOMUX_PINCM59); IOMUX->SECCFG.PINCM[IOMUX_PINCM59] |= (1<<12);
    DL_GPIO_initDigitalInput(IOMUX_PINCM48); IOMUX->SECCFG.PINCM[IOMUX_PINCM48] |= (1<<12);
    DL_GPIO_initDigitalInput(IOMUX_PINCM52); IOMUX->SECCFG.PINCM[IOMUX_PINCM52] |= (1<<12);

    HW_SetMotorPWM(300, 300);
    HW_DelayMs(CHASSIS_TEST_MS);
    HW_SetMotorPWM(-300, -300);
    HW_DelayMs(CHASSIS_TEST_MS);
    HW_SetMotorPWM(0, 0);

    uint32_t lastToggle = 0;
    int motor_mode = 0;
    uint32_t btn_tick = g_tick;
    bool btn_last = (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_18) != 0);

    //编码器初始化
    static int enc_a1, enc_b1, enc_a2, enc_b2;
    static int enc_cnt_l, enc_cnt_r;
    enc_a1 = DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_25) ? 1 : 0;
    enc_b1 = DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_26) ? 1 : 0;
    enc_a2 = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_20) ? 1 : 0;
    enc_b2 = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_24) ? 1 : 0;
    int last_l = 0, last_r = 0;

    for (;;) {
        DebugCLI_Poll();

        if (g_tick - btn_tick >= 50) {
            btn_tick = g_tick;
            bool btn_cur = (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_18) != 0);
            if (!btn_cur && btn_last) {
                motor_mode = (motor_mode + 1) % 3;
            }
            btn_last = btn_cur;
        }

        int pwm = 0;
        if      (motor_mode == 1) pwm =  300;
        else if (motor_mode == 2) pwm = -300;
        HW_SetMotorPWM(pwm, pwm);

        //编码器轮询
        static uint32_t enc_tick;
        if (g_tick - enc_tick >= 1) {
            enc_tick = g_tick;
            int a1 = DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_25) ? 1 : 0;
            int b1 = DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_26) ? 1 : 0;
            if (a1 != enc_a1) { enc_cnt_l += (a1 == b1) ? 1 : -1; enc_a1 = a1; }
            if (b1 != enc_b1) { enc_cnt_l += (a1 == b1) ? -1 : 1; enc_b1 = b1; }
            int a2 = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_20) ? 1 : 0;
            int b2 = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_24) ? 1 : 0;
            if (a2 != enc_a2) { enc_cnt_r += (a2 == b2) ? 1 : -1; enc_a2 = a2; }
            if (b2 != enc_b2) { enc_cnt_r += (a2 == b2) ? -1 : 1; enc_b2 = b2; }
        }

        //每500ms打印编码器
        static uint32_t dbg_tick;
        if (g_tick - dbg_tick >= 500) {
            dbg_tick = g_tick;
            int dl = enc_cnt_l - last_l; last_l = enc_cnt_l;
            int dr = enc_cnt_r - last_r; last_r = enc_cnt_r;
            char buf[32];
            int n = sprintf(buf, "L=%d R=%d\r\n", dl, dr);
            for (int i = 0; i < n; i++) {
                DL_UART_Main_transmitData(UART_0_INST, (uint8_t)buf[i]);
                while (DL_UART_Main_isBusy(UART_0_INST));
            }
        }

        if (g_tick - lastToggle >= 500) {
            lastToggle = g_tick;
            DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_22);
        }
    }
}
