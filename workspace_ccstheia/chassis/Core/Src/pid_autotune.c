/**
 * @file    pid_autotune.c
 * @brief   继电自整定(Ziegler-Nichols),通过等幅振荡自动计算PID参数
 *          移植自Brett Beauregard PID_AutoTune_v0
 *          原理:输出在起始值±台阶之间切换→系统等幅振荡→
 *               测量临界振幅/周期→Ziegler-Nichols公式→Kp/Ki/Kd
 * @note    依赖PID_ATune_t.input指向外部变量,Runtime每周期读入当前值
 *          output由Runtime直接写入,调用者将其施加到执行器
 */
#include "pid_autotune.h"
#include <math.h>
#include <string.h>

/**
 * @brief  初始化自整定器
 * @param  at     自整定器实例指针
 * @param  input  指向被控量(传感器偏差)的指针
 * @note   调用者:AutoTuneTask_10ms(empty_cpp.cpp)
 *         默认noiseBand=0.5,oStep=30,controlType=1(PID),lookback=10s
 *         调用前需确保input已指向有效变量
 */
void PID_ATune_Init(PID_ATune_t *at, float *input)
{
    memset(at, 0, sizeof(PID_ATune_t));
    at->input       = input;
    at->controlType = 1;           //默认PID
    at->noiseBand   = 0.5f;
    at->oStep       = 30.0f;
    at->nLookBack   = 40;          //10s * 4 (250ms采样)
    at->lastTime    = 0;
    at->running     = false;
}

/**
 * @brief  执行一次自整定运算,应在固定周期(建议10ms)调用
 * @param  at       自整定器实例指针
 * @param  tick_ms  当前系统时刻(ms)
 * @return 0=运行中, 1=完成
 * @note   调用者:AutoTuneTask_10ms(empty_cpp.cpp)
 *         完成时内部会调用FinishUp计算Ku/Pu,读GetKp/GetKi/GetKd
 *         sampleTime按nLookBack自动选择: <25s取250ms, >=25s取value*10ms
 *         首次调用自动设置setpoint=当前input值,output=outputStart+oStep
 */
int PID_ATune_Runtime(PID_ATune_t *at, uint32_t tick_ms)
{
    if (at->peakCount > 9 && at->running) {
        at->running = false;
        //FinishUp
        at->output = at->outputStart;
        at->Ku = 4.0f * (2.0f * at->oStep) / ((at->absMax - at->absMin) * 3.14159f);
        at->Pu = (float)(at->peak1 - at->peak2) / 1000.0f;
        return 1;
    }

    uint32_t now = tick_ms;
    int sampleTime;
    if (at->nLookBack < 100)
        sampleTime = 250;
    else
        sampleTime = at->nLookBack * 10;

    if ((now - at->lastTime) < (uint32_t)sampleTime) return 0;
    at->lastTime = now;

    float refVal = *at->input;

    if (!at->running) {
        at->peakType   = 0;
        at->peakCount  = 0;
        at->absMax     = refVal;
        at->absMin     = refVal;
        at->setpoint   = refVal;
        at->running    = true;
        at->outputStart = at->output;
        at->output      = at->outputStart + at->oStep;
        //初始化历史缓冲全部填当前值,避免误判
        for (int i = 0; i <= at->nLookBack; i++)
            at->lastInputs[i] = refVal;
        return 0;
    }

    //更新全程极值
    if (refVal > at->absMax) at->absMax = refVal;
    if (refVal < at->absMin) at->absMin = refVal;

    //继电控制:输出在起始值±台阶之间切换
    if (refVal > at->setpoint + at->noiseBand)
        at->output = at->outputStart - at->oStep;
    else if (refVal < at->setpoint - at->noiseBand)
        at->output = at->outputStart + at->oStep;

    //峰谷检测
    bool isMax = true, isMin = true;
    for (int i = at->nLookBack - 1; i >= 0; i--) {
        float val = at->lastInputs[i];
        if (isMax) isMax = (refVal > val);
        if (isMin) isMin = (refVal < val);
        at->lastInputs[i + 1] = at->lastInputs[i];
    }
    at->lastInputs[0] = refVal;

    if (at->nLookBack < 9) return 0;  //缓冲未填满,不信任峰谷检测

    if (isMax) {
        if (at->peakType == 0) at->peakType = 1;
        if (at->peakType == -1) {
            at->peakType = 1;
            at->peak2 = at->peak1;
        }
        at->peak1 = now;
        at->peaks[at->peakCount] = refVal;
    } else if (isMin) {
        if (at->peakType == 0) at->peakType = -1;
        if (at->peakType == 1) {
            at->peakType = -1;
            at->peakCount++;
        }
        if (at->peakCount < 10)
            at->peaks[at->peakCount] = refVal;
    }

    //检查是否收敛:连续三次峰谷幅值差<全程振幅的5%
    if (at->peakCount > 2) {
        float avgSep = (fabsf(at->peaks[at->peakCount - 1] - at->peaks[at->peakCount - 2])
                      + fabsf(at->peaks[at->peakCount - 2] - at->peaks[at->peakCount - 3])) / 2.0f;
        if (avgSep < 0.05f * (at->absMax - at->absMin)) {
            at->running = false;
            at->output = at->outputStart;
            at->Ku = 4.0f * (2.0f * at->oStep) / ((at->absMax - at->absMin) * 3.14159f);
            at->Pu = (float)(at->peak1 - at->peak2) / 1000.0f;
            return 1;
        }
    }

    return 0;
}

/**
 * @brief  取消自整定
 * @param  at  自整定器实例指针
 * @note   调用者:CLI stop命令
 *         重置输出到outputStart,恢复运行标志
 */
void PID_ATune_Cancel(PID_ATune_t *at)
{
    at->running = false;
    at->output = at->outputStart;
}

/**
 * @brief  获取比例系数Kp
 * @param  at  已完成的整定器实例
 * @return Kp值, Ziegler-Nichols: PI→0.4*Ku, PID→0.6*Ku
 * @note   调用者:AutoTuneTask_10ms完成时
 */
float PID_ATune_GetKp(PID_ATune_t *at)
{
    return (at->controlType == 1) ? 0.6f * at->Ku : 0.4f * at->Ku;
}

/**
 * @brief  获取积分系数Ki
 * @param  at  已完成的整定器实例
 * @return Ki值, Ziegler-Nichols: PI→0.48*Ku/Pu, PID→1.2*Ku/Pu
 * @note   调用者:AutoTuneTask_10ms完成时
 */
float PID_ATune_GetKi(PID_ATune_t *at)
{
    if (at->Pu < 0.0001f) return 0.0f;  //防除零
    return (at->controlType == 1) ? 1.2f * at->Ku / at->Pu
                                  : 0.48f * at->Ku / at->Pu;
}

/**
 * @brief  获取微分系数Kd
 * @param  at  已完成的整定器实例
 * @return Kd值, PI→0, PID→0.075*Ku*Pu
 * @note   调用者:AutoTuneTask_10ms完成时
 */
float PID_ATune_GetKd(PID_ATune_t *at)
{
    return (at->controlType == 1) ? 0.075f * at->Ku * at->Pu : 0.0f;
}
