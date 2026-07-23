//视觉模块数据解析,从UART0接收Maixcam协议
//协议格式(双轴): <offset_x>,<offset_y>,<target_state>\r\n
//兼容旧格式(单轴): <offset_x>,<target_state>\r\n  (此时 offset_y 视为 0)
//例: 30,5,1\r\n = 右偏30像素,上偏5像素,云台跟随
//    30,1\r\n    = 右偏30像素,云台跟随(无俯仰)
//    0,0\r\n      = 无目标
#include "vision_parser.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

VisionData_t g_vision = { 0 };  //视觉数据全局实例,初始无目标

/**
 * @brief 解析一行视觉协议数据
 * @param line 以\\0结尾的一行(不含\\r\\n)
 * @note 兼容格式: <offset_x>,<offset_y>,<target_state> 或 <offset_x>,<target_state>
 *       offset_x/offset_y为浮点数(像素),target_state为整数(0-3)
 *       调用者:
 *       DebugCLI_Poll(UART0收到完整行且含逗号时)
 */
void Vision_ParseLine(const char *line)
{
    if (line == NULL || *line == '\0') return;
    //按逗号数区分格式:1个逗号=旧单轴<ox>,<ts>;2个逗号=新双轴<ox>,<oy>,<ts>
    int commas = 0;
    for (const char *p = line; *p; p++) if (*p == ',') commas++;

    //空字段保护:拒绝开头/结尾逗号、连续逗号(如",30,1"/"30,,1"/"30,1,"),
    //这些属于异常帧,直接丢弃,避免产生垃圾数据或静默误解析
    if (line[0] == ',' || line[strlen(line) - 1] == ',') return;
    for (const char *p = line; *p; p++) {
        if (*p == ',' && *(p + 1) == ',') return;  //连续逗号=空字段
    }

    float ox = 0.0f, oy = 0.0f;
    int   ts = 0;
    if (commas == 1) {
        //旧格式 <offset_x>,<target_state>
        if (sscanf(line, "%f,%d", &ox, &ts) != 2) return;
        oy = 0.0f;
    } else if (commas >= 2) {
        //新格式 <offset_x>,<offset_y>,<target_state>
        if (sscanf(line, "%f,%f,%d", &ox, &oy, &ts) != 3) return;
    } else {
        return;  //无逗号,不是视觉协议
    }
    if (ts < 0 || ts > 3) return;      //target_state只定义0-3
    g_vision.offset_x     = ox;
    g_vision.offset_y     = oy;
    g_vision.target_state = ts;
    g_vision.updated      = true;
}
