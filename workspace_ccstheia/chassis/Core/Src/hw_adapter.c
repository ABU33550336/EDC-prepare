#include "hw_adapter.h"
#include <stddef.h>
#include <stdio.h>
#include "ti_msp_dl_config.h"

//电机PWM周期,单位定时器计数;占空比=比较值/(本值-1)
#define MOTOR_PWM_PERIOD      1600

//v0.6 IOMUX引脚映射
//PWM输出:TIMA1两路
#define IOMUX_PB2             IOMUX_PINCM15  //左轮PWM输出(TIMA1_C0)
#define IOMUX_PB3             IOMUX_PINCM16  //右轮PWM输出(TIMA1_C1)
//方向GPIO:TB6612四路
#define IOMUX_PA13            IOMUX_PINCM35  //左电机方向1(AIN1)
#define IOMUX_PA14            IOMUX_PINCM36  //左电机方向2(AIN2)
#define IOMUX_PA16            IOMUX_PINCM38  //右电机方向1(BIN1)
#define IOMUX_PA17            IOMUX_PINCM39  //右电机方向2(BIN2)
//编码器:左轮A/B,右轮A/B
#define IOMUX_PA25            IOMUX_PINCM55  //左编码器A相(TIMG12_C1)
#define IOMUX_PA26            IOMUX_PINCM59  //左编码器B相(TIMG8_C0)
#define IOMUX_PB20            IOMUX_PINCM48  //右编码器A相(TIMG12_C0)
#define IOMUX_PB24            IOMUX_PINCM52  //右编码器B相(GPIO中断计数)
//巡线传感器
#define IOMUX_PB17            IOMUX_PINCM43  //巡线传感器1
#define IOMUX_PB16            IOMUX_PINCM33  //巡线传感器2
#define IOMUX_PA12            IOMUX_PINCM34  //巡线传感器3
#define IOMUX_PA27            IOMUX_PINCM60  //巡线传感器4(ADC0_A0_0)

//引脚复用功能编号,取自SysConfig生成的ti_msp_dl_config
#define PF_PB2_TIMA1_C0       8
#define PF_PB3_TIMA1_C1       8
#define PF_PA25_TIMG12_C1     4
#define PF_PA26_TIMG8_C0      4
#define PF_PB20_TIMG12_C0     5

//PWM通道索引,TIMA1的CC0/CC1对应左右轮
#define PWM_CH_LEFT            DL_TIMER_CC_0_INDEX
#define PWM_CH_RIGHT           DL_TIMER_CC_1_INDEX

static uint8_t motor_inited   = 0;  //电机PWM已初始化标志
static uint8_t encoder_inited = 0;  //编码器已初始化标志
static uint8_t line_inited    = 0;  //巡线传感器已初始化标志
static volatile uint32_t tick_ms = 0;     //系统毫秒时基,由SysTick累加

//软件正交解码状态,抄参考工程STM32的TIM_EncoderInterfaceConfig(TI12)思路
//每轮维护带符号累加器,在A/B相边沿按相位差增减,等价硬件四倍频判向
//左轮:A=PA25(TIMG12_C1边沿) B=PA26(TIMG8_C0边沿)
//右轮:A=PB20(TIMG12_C0边沿) B=PB24(GPIO上升沿中断)
//HW_GetEncoderCnt返回自上次读取的增量并清零,与参考工程Encoder_Get读CNT后置零一致
static volatile int32_t g_left_enc_cnt;    //左轮带符号累计脉冲,边沿中断里增减
static volatile int32_t g_left_enc_last;    //上次HW_GetEncoderCnt读取值,用于增量返回
static volatile int32_t g_right_enc_cnt;   //右轮带符号累计脉冲
static volatile int32_t g_right_enc_last;   //上次读取值
//边沿采样用的上一相状态,用于软件判向(替代原两定时器计数相减)
static volatile uint8_t  g_left_a_prev;
static volatile uint8_t  g_left_b_prev;
static volatile uint8_t  g_right_a_prev;
static volatile uint8_t  g_right_b_prev;

/**
 * @brief 获取系统毫秒时基
 * @return 系统当前毫秒计数值
 * @note 直接返回tick_ms,供调度器与超时判断使用
 *       调用者:
 *       HW_DelayMs
 *       SpeedCtl_Update
 */
uint32_t HW_GetTick(void)
{
    return tick_ms;
}

/**
 * @brief 毫秒时基累加,供SysTick中断调用
 * @note 每1ms由SysTick_Handler调用一次,递增全局时基
 *       调用者:
 *       SysTick_Handler
 */
void HW_TickInc(void)
{
    tick_ms++;
}

/**
 * @brief 阻塞延时,单位毫秒
 * @param ms 延时时长,单位ms
 * @note 轮询时基实现,调试可用,正式运行避免长延时阻塞主循环
 *       调用者:
 *       main
 */
void HW_DelayMs(uint32_t ms)
{
    uint32_t start = HW_GetTick();
    while ((HW_GetTick() - start) < ms) { }
}

/**
 * @brief 初始化电机PWM与方向GPIO
 * @return HW_OK(0)表示成功,仅首次执行
 * @note 启动TIMA1两路PWM,方向引脚初始低电平(停止态)
 *       调用者:
 *       main
 */
uint8_t HW_MotorInit(void)
{
    if (motor_inited) return HW_OK;

    //PWM输出:TIMA1_C0左轮(PB2),C1右轮(PB3)
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PB2, PF_PB2_TIMA1_C0);
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PB3, PF_PB3_TIMA1_C1);

    //方向GPIO: 直接寄存器配置,确保推挽输出
    //PINCM: PF=GPIO(1), PC=CONNECTED(7), HIZ=DISABLE, INENA=ENABLE
    IOMUX->SECCFG.PINCM[IOMUX_PA13] = 0x00040081U;
    IOMUX->SECCFG.PINCM[IOMUX_PA14] = 0x00040081U;
    IOMUX->SECCFG.PINCM[IOMUX_PA16] = 0x00040081U;
    IOMUX->SECCFG.PINCM[IOMUX_PA17] = 0x00140081U;  //+DRV高驱动

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    delay_cycles(POWER_STARTUP_DELAY);

    //GPIO输出使能(DOESET)
    GPIOA->DOESET31_0 = DL_GPIO_PIN_13 | DL_GPIO_PIN_14 | DL_GPIO_PIN_16 | DL_GPIO_PIN_17;

    DL_TimerA_reset(TIMA1);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_TimerA_enablePower(TIMA1);   //reset会清PWREN,必须在reset之后重新使能
    delay_cycles(POWER_STARTUP_DELAY);
    //TIMA1时钟源选择:CLKSEL复位值为0(无时钟源),必须显式选BUSCLK,否则计数器无时钟输入不递增,PB2/PB3无PWM输出
    DL_TimerA_ClockConfig tima1Clock = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale    = 0
    };
    DL_TimerA_setClockConfig(TIMA1, &tima1Clock);
    //显式开启TIMA1总线时钟,否则计数器不跑,PB2/PB3无PWM输出
    DL_TimerA_enableClock(TIMA1);

    DL_Timer_PWMConfig pwmCfg = {
        .period    = MOTOR_PWM_PERIOD - 1,
        .pwmMode   = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
        .startTimer = DL_TIMER_START
    };
    DL_Timer_initPWMMode(TIMA1, &pwmCfg);

    //CCP输出方向:initPWMMode不配置CCPD,复位默认CCP引脚为输入方向,必须显式设为输出,PB2/PB3才会出PWM波形
    DL_Timer_setCCPDirection(TIMA1, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);

    DL_Timer_setCaptureCompareValue(TIMA1, 0, PWM_CH_LEFT);
    DL_Timer_setCaptureCompareValue(TIMA1, 0, PWM_CH_RIGHT);

    //方向引脚初始低电平(停止),避免上电误转
    DL_GPIO_clearPins(GPIOA,
        DL_GPIO_PIN_13 | DL_GPIO_PIN_14 | DL_GPIO_PIN_16 | DL_GPIO_PIN_17);

    motor_inited = 1;
    return HW_OK;
}

/**
 * @brief 设置左右轮PWM占空比,正负控制正反转
 * @param left 左轮PWM值(-1000~1000),正数正转负数反转
 * @param right 右轮PWM值(-1000~1000),正数正转负数反转
 * @note 先限幅到±1000再映射到定时器比较值;方向由AIN1/AIN2(BIN1/BIN2)高低电平决定
 *       调用者:
 *       SpeedCtl_Update
 *       main
 */
void HW_SetMotorPWM(int16_t left, int16_t right)
{
    if (!motor_inited) HW_MotorInit();

    if (left > 1000) left = 1000;
    if (left < -1000) left = -1000;
    if (right > 1000) right = 1000;
    if (right < -1000) right = -1000;

    uint32_t left_pwm  = (left  >= 0) ? (uint32_t)left  : (uint32_t)(-left);
    uint32_t right_pwm = (right >= 0) ? (uint32_t)right : (uint32_t)(-right);
    left_pwm  = left_pwm  * (MOTOR_PWM_PERIOD - 1) / 1000;
    right_pwm = right_pwm * (MOTOR_PWM_PERIOD - 1) / 1000;

    DL_Timer_setCaptureCompareValue(TIMA1, left_pwm,  PWM_CH_LEFT);
    DL_Timer_setCaptureCompareValue(TIMA1, right_pwm, PWM_CH_RIGHT);

    //左电机方向:PA13=AIN1, PA14=AIN2
    if (left > 0) {
        DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_13);
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_14);
    } else if (left < 0) {
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_13);
        DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_14);
    } else {
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_13);
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_14);
    }

    //右电机方向:PA16=BIN1, PA17=BIN2
    if (right > 0) {
        DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_16);
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_17);
    } else if (right < 0) {
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_16);
        DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_17);
    } else {
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_16);
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_17);
    }
}

/**
 * @brief 正交解码核心:任意一相边沿时按A/B当前电平相位关系判定±1
 * @note 与STM32 TIM_EncoderInterfaceConfig(TI12,Rising,Rising)的四倍频判向等价
 *       调用者:
 *       LeftEnc_Decode
 *       RightEnc_Decode
 */
//左轮:A=PA25, B=PA26
static void LeftEnc_Decode(void)
{
    uint8_t a = (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_25) != 0) ? 1 : 0;
    uint8_t b = (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_26) != 0) ? 1 : 0;
    //A相变化:若B与A同相则正转,否则反转
    if (a != g_left_a_prev) {
        g_left_enc_cnt += (a == b) ? +1 : -1;
    } else if (b != g_left_b_prev) {
        //B相变化:若B与A同相则反转,否则正转
        g_left_enc_cnt += (a == b) ? -1 : +1;
    }
    g_left_a_prev = a;
    g_left_b_prev = b;
}

//右轮:A=PB20, B=PB24
static void RightEnc_Decode(void)
{
    uint8_t a = (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_20) != 0) ? 1 : 0;
    uint8_t b = (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_24) != 0) ? 1 : 0;
    if (a != g_right_a_prev) {
        g_right_enc_cnt += (a == b) ? +1 : -1;
    } else if (b != g_right_b_prev) {
        g_right_enc_cnt += (a == b) ? -1 : +1;
    }
    g_right_a_prev = a;
    g_right_b_prev = b;
}

/**
 * @brief 初始化编码器采集(软件正交解码)
 * @return HW_OK(0)表示成功,仅首次执行
 * @note 左轮A/B接TIMG12_C1与TIMG8_C0,右轮A接TIMG12_C0,B接PB24 GPIO中断
 *       各相均配上升沿捕获/中断,边沿里读当前电平做判向
 *       调用者:
 *       main
 *       HW_GetEncoderCnt
 */
uint8_t HW_EncoderInit(void)
{
    if (encoder_inited) return HW_OK;

    //左编码器:A=PA25(TIMG12_C1) B=PA26(TIMG8_C0)
    DL_GPIO_initPeripheralInputFunction(IOMUX_PA25, PF_PA25_TIMG12_C1);
    DL_GPIO_initPeripheralInputFunction(IOMUX_PA26, PF_PA26_TIMG8_C0);
    //右编码器A:PB20(TIMG12_C0)
    DL_GPIO_initPeripheralInputFunction(IOMUX_PB20, PF_PB20_TIMG12_C0);
    //编码器输入脚加内部下拉,防止浮空中断风暴
    IOMUX->SECCFG.PINCM[IOMUX_PA25] |= (1 << 12);
    IOMUX->SECCFG.PINCM[IOMUX_PA26] |= (1 << 12);
    IOMUX->SECCFG.PINCM[IOMUX_PB20] |= (1 << 12);

    DL_TimerG_ClockConfig capClock = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale    = 0
    };

    //TIMG12:左A(CC1)+右A(CC0),上升沿捕获
    DL_TimerG_enablePower(TIMG12);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_TimerG_reset(TIMG12);
    DL_TimerG_setClockConfig(TIMG12, &capClock);
    DL_TimerG_enableClock(TIMG12);

    DL_TimerG_CaptureConfig capCfg0 = {
        .captureMode  = DL_TIMER_CAPTURE_MODE_EDGE_TIME,
        .period       = 0xFFFF,
        .startTimer   = DL_TIMER_START,
        .edgeCaptMode = DL_TIMER_CAPTURE_EDGE_DETECTION_MODE_RISING,
        .inputChan    = DL_TIMER_INPUT_CHAN_0,
        .inputInvMode = 0
    };
    DL_TimerG_CaptureConfig capCfg1 = {
        .captureMode  = DL_TIMER_CAPTURE_MODE_EDGE_TIME,
        .period       = 0xFFFF,
        .startTimer   = DL_TIMER_START,
        .edgeCaptMode = DL_TIMER_CAPTURE_EDGE_DETECTION_MODE_RISING,
        .inputChan    = DL_TIMER_INPUT_CHAN_1,
        .inputInvMode = 0
    };
    //CC0=右A, CC1=左A
    DL_TimerG_initCaptureMode(TIMG12, &capCfg0);
    DL_TimerG_initCaptureMode(TIMG12, &capCfg1);
    DL_TimerG_enableInterrupt(TIMG12, DL_TIMER_INTERRUPT_CC0_DN_EVENT);
    DL_TimerG_enableInterrupt(TIMG12, DL_TIMER_INTERRUPT_CC1_DN_EVENT);
    NVIC_EnableIRQ(TIMG12_INT_IRQn);

    //TIMG8:左B(CC0),上升沿捕获
    DL_TimerG_enablePower(TIMG8);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_TimerG_reset(TIMG8);
    DL_TimerG_setClockConfig(TIMG8, &capClock);
    DL_TimerG_enableClock(TIMG8);
    DL_TimerG_initCaptureMode(TIMG8, &capCfg0);
    DL_TimerG_enableInterrupt(TIMG8, DL_TIMER_INTERRUPT_CC0_DN_EVENT);
    NVIC_EnableIRQ(TIMG8_INT_IRQn);

    //PB24 GPIO中断:右编码器B相,上升沿计数
    DL_GPIO_initDigitalInput(IOMUX_PB24);
    IOMUX->SECCFG.PINCM[IOMUX_PINCM52] |= (1 << 12) | (1 << 13); //上拉
    DL_GPIO_enablePower(GPIOB);
    delay_cycles(POWER_STARTUP_DELAY);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
    DL_GPIO_setUpperPinsPolarity(GPIOB, DL_GPIO_PIN_24_EDGE_RISE);
    DL_GPIO_enableInterrupt(GPIOB, DL_GPIO_PIN_24);
    DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_24);

    //初始化相位采样(只读当前电平作为首次判向基准)
    g_left_a_prev  = (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_25) != 0) ? 1 : 0;
    g_left_b_prev  = (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_26) != 0) ? 1 : 0;
    g_right_a_prev = (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_20) != 0) ? 1 : 0;
    g_right_b_prev = (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_24) != 0) ? 1 : 0;
    g_left_enc_cnt  = 0;
    g_left_enc_last  = 0;
    g_right_enc_cnt = 0;
    g_right_enc_last = 0;
    encoder_inited = 1;
    return HW_OK;
}

/**
 * @brief 读取指定通道编码器增量脉冲
 * @param ch 通道选择(HW_ENCODER_CH_LEFT/HW_ENCODER_CH_RIGHT)
 * @return 自上次读取以来的增量脉冲(带符号,正为正转)
 * @note 返回增量并清零该增量,与参考工程Encoder_Get读CNT后置零一致
 *       调用者:
 *       SpeedCtl_Update
 */
int32_t HW_GetEncoderCnt(uint8_t ch)
{
    if (!encoder_inited) HW_EncoderInit();

    if (ch == HW_ENCODER_CH_LEFT) {
        int32_t cur = g_left_enc_cnt;
        int32_t delta = cur - g_left_enc_last;
        g_left_enc_last = cur;
        return delta;
    }
    if (ch == HW_ENCODER_CH_RIGHT) {
        int32_t cur = g_right_enc_cnt;
        int32_t delta = cur - g_right_enc_last;
        g_right_enc_last = cur;
        return delta;
    }
    return 0;
}

/**
 * @brief 清零指定通道编码器读取基准
 * @param ch 通道选择(HW_ENCODER_CH_LEFT/HW_ENCODER_CH_RIGHT)
 * @note 仅同步上次读取基准到当前计数值,不清零实时累加器
 *       避免与SpeedCtl的差分逻辑冲突(原实现双份清零是bug根因)
 *       调用者:
 *       SpeedCtl_Update
 */
void HW_ClearEncoderCnt(uint8_t ch)
{
    if (ch == HW_ENCODER_CH_LEFT) {
        g_left_enc_last = g_left_enc_cnt;
    }
    if (ch == HW_ENCODER_CH_RIGHT) {
        g_right_enc_last = g_right_enc_cnt;
    }
}

//TIMG12中断:左A(CC1)+右A(CC0)边沿,读电平做正交判向
void TIMG12_IRQHandler(void)
{
    uint32_t flags = DL_TimerG_getPendingInterrupt(TIMG12);
    if (flags & DL_TIMER_IIDX_CC0_DN) {
        DL_TimerG_clearInterruptStatus(TIMG12, DL_TIMER_INTERRUPT_CC0_DN_EVENT);
        RightEnc_Decode();   //右轮A相边沿
    }
    if (flags & DL_TIMER_IIDX_CC1_DN) {
        DL_TimerG_clearInterruptStatus(TIMG12, DL_TIMER_INTERRUPT_CC1_DN_EVENT);
        LeftEnc_Decode();    //左轮A相边沿
    }
}

//TIMG8中断:左B(CC0)边沿
void TIMG8_IRQHandler(void)
{
    uint32_t flags = DL_TimerG_getPendingInterrupt(TIMG8);
    if (flags & DL_TIMER_IIDX_CC0_DN) {
        DL_TimerG_clearInterruptStatus(TIMG8, DL_TIMER_INTERRUPT_CC0_DN_EVENT);
        LeftEnc_Decode();    //左轮B相边沿
    }
}

//GPIOB中断:PB24上升沿=右编码器B相脉冲
void GPIOB_IRQHandler(void)
{
    if (DL_GPIO_getPendingInterrupt(GPIOB) == DL_GPIO_IIDX_DIO24) {
        DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_24);
        RightEnc_Decode();   //右轮B相边沿
    }
}

//调试:通过CLI_UART打印左右轮正交解码累计脉冲,用于判断电机不动时编码器是否在动
static void EncDebug_SendChar(uint8_t c)
{
    DL_UART_Main_transmitData(CLI_UART_INST, c);
    while (DL_UART_Main_isBusy(CLI_UART_INST));
}

void HW_EncDebugReport(void)
{
    char buf[48];
    int n = sprintf(buf, "ENC L=%ld R=%ld\r\n",
        (long)g_left_enc_cnt, (long)g_right_enc_cnt);
    for (int i = 0; i < n; i++) {
        EncDebug_SendChar((uint8_t)buf[i]);
    }
}

/**
 * @brief 初始化巡线传感器(数字输入+单路ADC)
 * @return HW_OK(0)表示成功,仅首次执行
 * @note 传感器1/2/3为GPIO数字输入,传感器4为ADC0_A0_0模拟量
 *       调用者:
 *       main
 */
uint8_t HW_LineSensorInit(void)
{
    if (line_inited) return HW_OK;
    //巡线引脚PA12/PB17/PB16为GPIO输入,PA27为ADC0_A0_0
    DL_GPIO_initDigitalInput(IOMUX_PA12);
    DL_GPIO_initDigitalInput(IOMUX_PB17);
    DL_GPIO_initDigitalInput(IOMUX_PB16);
    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    delay_cycles(POWER_STARTUP_DELAY);
#if defined(ADC_0_INST)
    DL_ADC12_startConversion(ADC_0_INST);
#endif
    line_inited = 1;
    return HW_OK;
}

/**
 * @brief 读取巡线传感器值
 * @param buf 输出缓冲区,长度需>=HW_LINE_SENSOR_COUNT
 * @note 关中断拷贝防止DMA/ ADC竞争,数字量直接读引脚电平
 *       调用者:
 *       ControlTask
 */
void HW_ReadLineSensors(uint16_t *buf)
{
    if (!line_inited) HW_LineSensorInit();
    if (buf == NULL) return;
    //传感器1=PB17, 2=PB16, 3=PA12, 4=PA27(ADC), 5=未定义填0
    buf[0] = (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_17) != 0) ? 1 : 0;
    buf[1] = (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_16) != 0) ? 1 : 0;
    buf[2] = (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_12) != 0) ? 1 : 0;
#if defined(ADC_0_INST)
    DL_ADC12_enableConversions(ADC_0_INST);
    DL_ADC12_startConversion(ADC_0_INST);
    while (DL_ADC12_isConversionInProgress(ADC_0_INST) == true) { }
    buf[3] = (uint16_t)DL_ADC12_getMemResult(ADC_0_INST, 0);
#else
    buf[3] = 0;
#endif
    buf[4] = 0; //传感器5未定义
}
