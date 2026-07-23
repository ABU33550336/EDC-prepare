//底盘轮速PID控制器,50ms周期编码器反馈速度闭环
//适用于JGB37-520(330RPM@12V)配TB6612驱动,左右均软件正交解码
#ifndef SPEED_CTL_H
#define SPEED_CTL_H

#include <stdint.h>
#include <stdbool.h>

//速度闭环周期,ms
//选50ms而非更短的原因:编码器仅22PPR,300RPM时脉冲间隔≈9ms/个,
//周期再短(<20ms)会出现大量周期内0脉冲的"假零速"导致控制不稳
#define SPEED_CTL_PERIOD_MS   50   //速度闭环周期,ms

//左编码器每转脉冲数
//JGB37-520霍尔码盘=11PPR,软件解码在每个A/B边沿判向,相当于2倍频=22
#define SPEED_CTL_LEFT_CPR    22   //左编码器每转脉冲数(软件解码2倍频)

//右编码器每转脉冲数
//与左轮同源,同样软件解码2倍频,保持左右CPR一致便于统一测速
#define SPEED_CTL_RIGHT_CPR   22   //右编码器每转脉冲数(软件解码2倍频)

//最大目标转速,rpm
//JGB37-520空载额定转速,11.1V锂电池下实际空载≈305RPM
//超出此值虽不会立刻损坏但力矩急剧下降,且编码器采样精度恶化
#define SPEED_CTL_MAX_RPM     330  //最大目标转速,rpm(JGB37-520额定值)

//单通道PID参数(左轮/右轮各一个实例)
typedef struct {
    float kp;             //比例系数,直接放大当前误差,越大响应越快但超调也大
    float ki;             //积分系数,消除静差,越大消除越快但积分饱和风险增加
    float kd;             //微分系数,预测误差趋势提供阻尼,kd=0时退化为PI
    float integral;       //积分累积值,每周期累加ki×err,受integral_limit限幅
    float integral_limit; //积分限幅,防积分饱和导致超调,经验值取output_limit的20%
    float output_limit;   //输出限幅,对应HW_SetMotorPWM的±1000范围
    float prev_err;       //上次误差,用于微分项kd×(err-prev_err)差分
} SpeedCtl_PID_t;

//双通道速度控制器(左右轮独立PID+公共测速状态)
typedef struct {
    SpeedCtl_PID_t left;     //左轮PID参数+中间变量
    SpeedCtl_PID_t right;    //右轮PID参数+中间变量
    float target_rpm_l;      //左轮目标转速,rpm
    float target_rpm_r;      //右轮目标转速,rpm
    float meas_rpm_l;        //左轮实测转速,rpm,每50ms计算一次
    float meas_rpm_r;        //右轮实测转速,rpm
    int32_t last_enc_l;      //左编码器上次采样值,用于计算帧间差分
    int32_t last_enc_r;      //右编码器上次采样值
    int32_t acc_enc_l;       //左编码器周期内脉冲累积,每次测速后清零
    int32_t acc_enc_r;       //右编码器周期内脉冲累积
    uint32_t last_run;       //上次更新时间戳,ms,判断是否到执行周期
    bool enabled;            //使能标志,false时跳过PID只测速,用于开环调参
} SpeedCtl_t;

void SpeedCtl_Init(SpeedCtl_t *sc);
void SpeedCtl_Enable(SpeedCtl_t *sc);
void SpeedCtl_Disable(SpeedCtl_t *sc);
void SpeedCtl_SetTarget(SpeedCtl_t *sc, float left_rpm, float right_rpm);
void SpeedCtl_SetParam(SpeedCtl_t *sc, float kp, float ki, float kd);
void SpeedCtl_Update(SpeedCtl_t *sc);

#endif
