//航迹推算模块,通过陀螺仪角速度与轮速编码器更新位姿

#include "dead_reckoning.h"
#include <stddef.h>
#include <math.h>

/**
 * @brief 初始化航迹推算实例
 * @param dr 航迹推算对象指针
 * @param wheel_base_mm 轮距,mm(用于运动学计算)
 * @note 轮距设为默认值100mm若传入<=0;初始化后未置位initialized标志
 *        调用者:
 */
void DR_Init(DR_t *dr, float wheel_base_mm)
{
    if (dr == NULL) return;
    dr->wheel_base_mm = (wheel_base_mm > 0.0f) ? wheel_base_mm : 100.0f;     //防止轮距无效导致计算异常
    dr->x_mm           = 0.0f;
    dr->y_mm           = 0.0f;
    dr->heading_deg    = 0.0f;
    dr->last_heading_deg = 0.0f;
    dr->initialized    = 0;                                                   //需调用DR_Reset后才可使用
}

/**
 * @brief 复位航迹推算至指定初始位姿
 * @param dr 航迹推算对象指针
 * @param init_x 初始X坐标,mm
 * @param init_y 初始Y坐标,mm
 * @param init_heading 初始航向角,度(-180~+180)
 * @note 调用后initialized置1,DR_Update方可正常执行
 *        调用者:
 */
void DR_Reset(DR_t *dr, float init_x, float init_y, float init_heading)
{
    if (dr == NULL) return;
    dr->x_mm           = init_x;
    dr->y_mm           = init_y;
    dr->heading_deg    = init_heading;
    dr->last_heading_deg = init_heading;
    dr->initialized    = 1;
}

/**
 * @brief 更新航迹推算(基于陀螺仪角速度+编码器里程)
 * @param dr 航迹推算对象指针
 * @param rate_degps 陀螺仪角速度,°/s
 * @param delta_left_mm 左轮里程增量,mm
 * @param delta_right_mm 右轮里程增量,mm
 * @param dt 时间间隔,s
 * @note 航向由角速度积分得到;位置由左右轮平均里程在航向上分解
 *        调用者:
 */
void DR_Update(DR_t *dr, float rate_degps, float delta_left_mm,
               float delta_right_mm, float dt)
{
    if (dr == NULL) return;
    if (dt <= 0.0f) return;                                                   //时间增量必须为正

    if (!dr->initialized) {
        dr->initialized = 1;                                                  //自动初始化,使用默认位姿(0,0,0)
    }

    float heading_rad = dr->heading_deg * (3.14159265f / 180.0f);             //当前航向转弧度(用于三角计算)
    float delta_heading = rate_degps * dt;                                    //角度增量 = 角速度 × 时间
    dr->heading_deg += delta_heading;

    //将航向约束到(-180,180]范围,避免累积误差导致角度漂移
    if (dr->heading_deg > 180.0f) {
        dr->heading_deg -= 360.0f;
    } else if (dr->heading_deg <= -180.0f) {
        dr->heading_deg += 360.0f;
    }

    float ds = (delta_left_mm + delta_right_mm) * 0.5f;                      //平均里程(圆弧近似为直线段)

    //在航向方向上分解位移
    dr->x_mm += ds * cosf(heading_rad);                                       //X方向增量 = ds × cos(航向)
    dr->y_mm += ds * sinf(heading_rad);                                       //Y方向增量 = ds × sin(航向)
}

/**
 * @brief 获取当前航向角(归一化到-180~+180)
 * @param dr 航迹推算对象指针
 * @return 航向角,度
 * @note while循环保证多次归一化,防止极端偏差
 *        调用者:
 */
float DR_GetHeading(DR_t *dr)
{
    if (dr == NULL) return 0.0f;
    float h = dr->heading_deg;
    //多次归一化,处理因多次积分导致的超界
    while (h > 180.0f)  h -= 360.0f;
    while (h < -180.0f) h += 360.0f;
    return h;
}

/**
 * @brief 获取当前位置坐标
 * @param dr 航迹推算对象指针
 * @param x 输出X坐标指针(NULL则跳过)
 * @param y 输出Y坐标指针(NULL则跳过)
 * @note
 *        调用者:
 */
void DR_GetPosition(DR_t *dr, float *x, float *y)
{
    if (dr == NULL) return;
    if (x != NULL) *x = dr->x_mm;
    if (y != NULL) *y = dr->y_mm;
}
