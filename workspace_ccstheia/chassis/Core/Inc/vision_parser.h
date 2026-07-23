//视觉模块数据解析,从UART0接收Maixcam协议
#ifndef VISION_PARSER_H
#define VISION_PARSER_H

#include <stdint.h>
#include <stdbool.h>

//像素→角度换算系数,度/像素
//标定方法:云台转已知角度θ,看靶纸偏移Δpx → K_VISION = θ/Δpx
//初始值0.5仅用于代码调试,正式跑车前需重新标定
#define VISION_K_DEG_PER_PX  0.5f
//俯仰方向像素→角度换算系数,度/像素,待标定
//符号约定:offset_y>0 表示目标在视野上方,云台需上仰(pitch+)
#define VISION_K_PITCH_DEG_PER_PX  0.5f

//视觉数据状态
typedef struct {
    float offset_x;       //水平偏移,像素,正=目标在视野右侧(云台需右转)
    float offset_y;       //垂直偏移,像素,正=目标在视野上方(云台需上仰)
    int   target_state;   //0=无 1=云台跟随 2=底盘控制 3=云台+底盘
    bool  updated;        //新数据消费标志,主循环处理完清零
} VisionData_t;

extern VisionData_t g_vision;

//解析一行视觉协议数据
//调用者:DebugCLI_Poll(UART0收到完整行且含逗号时)
void Vision_ParseLine(const char *line);

#endif
