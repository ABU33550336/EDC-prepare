//继电自整定(Ziegler-Nichols),通过等幅振荡自动计算PID参数
#ifndef PID_AUTOTUNE_H
#define PID_AUTOTUNE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float   *input;              //指向被控量(传感器偏差)
    float    setpoint;           //期望值
    float    noiseBand;          //噪声带,小于此值视为噪声
    float    oStep;              //输出台阶幅值
    uint8_t  controlType;        //0=PI, 1=PID
    int8_t   peakType;           //1=峰,-1=谷,0=初始

    float    output;             //当前输出值(修正量)
    float    outputStart;        //起始输出值

    uint32_t peak1;              //上一个峰时刻(ms)
    uint32_t peak2;              //再上一个峰时刻(ms)
    uint32_t lastTime;           //上次采样时刻(ms)
    int      nLookBack;          //回溯窗口采样数
    float    lastInputs[101];    //历史输入环形缓冲

    int      peakCount;          //检测到的峰数量
    float    peaks[10];          //峰/谷值记录
    float    absMax;             //全程最大值
    float    absMin;             //全程最小值
    float    Ku;                 //临界增益
    float    Pu;                 //临界周期(s)
    bool     running;            //是否运行中
} PID_ATune_t;

void  PID_ATune_Init(PID_ATune_t *at, float *input);
int   PID_ATune_Runtime(PID_ATune_t *at, uint32_t tick_ms);
void  PID_ATune_Cancel(PID_ATune_t *at);
float PID_ATune_GetKp(PID_ATune_t *at);
float PID_ATune_GetKi(PID_ATune_t *at);
float PID_ATune_GetKd(PID_ATune_t *at);

#ifdef __cplusplus
}
#endif

#endif
