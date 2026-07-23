//差速驱动模块,底盘运动学正/逆解算

#include "diff_drive.h"
#include <stddef.h>
#include <math.h>

/**
 * @brief 初始化差速驱动模型
 * @param dd 差速驱动对象指针
 * @param wheel_base_m 轮距, m
 * @param wheel_radius_m 车轮半径, m
 * @note 无效参数使用默认值(轮距0.1m,半径0.03m)防止除零
 *        调用者:
 */
void DiffDrive_Init(DiffDrive_t *dd, float wheel_base_m,
                    float wheel_radius_m)
{
    if (dd == NULL) return;
    dd->wheel_base_m   = (wheel_base_m > 0.0f)   ? wheel_base_m   : 0.1f;    //轮距无效时使用默认值
    dd->wheel_radius_m = (wheel_radius_m > 0.0f) ? wheel_radius_m : 0.03f;   //半径无效时使用默认值
}

/**
 * @brief 逆运动学:将底盘速度解算为左右轮转速
 * @param dd 差速驱动对象指针
 * @param linear_v_ms 线速度,m/s(前进为正)
 * @param angular_v_rads 角速度,rad/s(左转为正)
 * @param left_speed 左轮转速输出,rad/s
 * @param right_speed 右轮转速输出,rad/s
 * @note 公式:v_l = (v - w*L/2)/r, v_r = (v + w*L/2)/r
 *        调用者:
 */
void DiffDrive_WheelSpeeds(DiffDrive_t *dd, float linear_v_ms,
                           float angular_v_rads,
                           float *left_speed, float *right_speed)
{
    if (dd == NULL) return;
    if (left_speed != NULL)  *left_speed  = 0.0f;
    if (right_speed != NULL) *right_speed = 0.0f;
    if (dd->wheel_radius_m < 1e-12f) return;                                  //防除零,半径过小时直接返回
    if (left_speed == NULL || right_speed == NULL) return;

    float half_base = dd->wheel_base_m * 0.5f;                                //L/2,用于计算转弯时的左右轮差
    //逆运动学:左轮减速(右转),右轮加速(右转)
    float v_left  = linear_v_ms - angular_v_rads * half_base;                 //左轮线速度 = v - w*L/2
    float v_right = linear_v_ms + angular_v_rads * half_base;                 //右轮线速度 = v + w*L/2

    //线速度转角速度: ω = v/r
    *left_speed  = v_left  / dd->wheel_radius_m;
    *right_speed = v_right / dd->wheel_radius_m;
}
