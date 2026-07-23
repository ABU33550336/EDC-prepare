//ZDT X42S步进电机Emm_V5固件通信协议库(MSPM0移植版)
#ifndef __EMM_V5_H
#define __EMM_V5_H

//=== MSPM0 兼容层 ===
#include "ti_msp_dl_config.h"
#define __IO  volatile
typedef UART_Regs UART_HandleTypeDef;
//===================

extern UART_HandleTypeDef* g_emm_uart;   //当前使用的电机串口,由调用方切换

//Emm_V5.0步进电机控制库
//作者：ZHANGDATOU
//技术支持：张大头闭环步进
//淘宝店铺：https://zhangdatou.taobao.com
//CSDN博客：http s://blog.csdn.net/zhangdatou666
//qq技术群：262438510

#define  ABS(x)      ((x) > 0 ? (x) : -(x))          //绝对值宏

typedef enum {
	S_VBUS  = 5,   //获取总线电压
	S_CBUS  = 6,   //获取总线电流
	S_CPHA  = 7,   //获取相电流
	S_ENCO  = 8,   //获取编码器原始值
	S_CLKC  = 9,   //获取实时脉冲数
	S_ENCL  = 10,  //获取编码器经校准后的锁定值
	S_CLKI  = 11,  //获取脉冲输入数
	S_TPOS  = 12,  //获取轴目标位置
	S_SPOS  = 13,  //获取轴实时设定目标位置
	S_VEL   = 14,  //获取轴实时转速
	S_CPOS  = 15,  //获取轴实时位置
	S_PERR  = 16,  //获取轴位置跟随差
	S_VBAT  = 17,  //获取线圈充电电池电压(Y42)
	S_TEMP  = 18,  //获取轴实时温度(Y42)
	S_FLAG  = 19,  //获取轴状态标志位
	S_OFLAG = 20,  //获取回零状态标志位
	S_OAF   = 21,  //获取报警状态标志位+回零状态标志位(Y42)
	S_PIN   = 22,  //获取引脚状态(Y42)
}SysParams_t;

#define  MMCL_LEN   512                              //多机指令缓冲区长度
extern __IO uint16_t MMCL_count, MMCL_cmd[MMCL_LEN]; //多机指令计数和缓冲区

//后缀带Y42的为Y42系列专用指令,X42系列通用,其他通用
//编码器函数组
//编码器脉冲校准
void Emm_V5_Trig_Encoder_Cal(uint8_t addr);
//复位电机(Y42)
void Emm_V5_Reset_Motor(uint8_t addr);
//将当前脉冲位置清零
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);
//清除堵转保护
void Emm_V5_Reset_Clog_Pro(uint8_t addr);
//恢复电机运行
void Emm_V5_Restore_Motor(uint8_t addr);
//运动控制函数组
//多机指令(Y42)
void Emm_V5_Multi_Motor_Cmd(uint8_t addr);
//电机使能控制
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);
//速度模式控制
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
//位置模式控制
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, bool snF);
//设置快捷位置模式运动参数
void Emm_V5_Set_QPos_Params(uint8_t addr, uint16_t vel, uint8_t acc, uint8_t raF, bool snF);
//快捷位置模式控制
void Emm_V5_QPos_Control(uint8_t addr, int32_t clk);
//电机立即停止运动
void Emm_V5_Stop_Now(uint8_t addr, bool snF);
//触发多机同步开始运动
void Emm_V5_Synchronous_motion(uint8_t addr);
//原点回归函数组
//设置当前堵转位置为原点
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF);
//触发回原点
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
//强制中断并退出回零
void Emm_V5_Origin_Interrupt(uint8_t addr);
//读取回零参数
void Emm_V5_Origin_Read_Params(uint8_t addr);
//修改回零参数
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
//读取撞块回零返回角度(X42S/Y42)
void X_V2_Origin_Read_SL_RP(uint8_t addr);
//修改撞块回零返回角度(X42S/Y42)
void X_V2_Origin_Modify_SL_RP(uint8_t addr, bool svF, uint16_t sl_rp);
//获取系统运行参数函数组
//定时返回信息指令(Y42)
void Emm_V5_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms);
//读取系统参数
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);
//读写参数设置函数组
//修改电机ID地址
void Emm_V5_Modify_Motor_ID(uint8_t addr, bool svF, uint8_t id);
//修改细分值
void Emm_V5_Modify_MicroStep(uint8_t addr, bool svF, uint8_t mstep);
//修改电机方向
void Emm_V5_Modify_PDFlag(uint8_t addr, bool pdf);
//读取选择参数状态(Y42)
void Emm_V5_Read_Opt_Param_Sta(uint8_t addr);
//修改电机类型(Y42)
void Emm_V5_Modify_Motor_Type(uint8_t addr, bool svF, bool mottype);
//修改固件类型(Y42)
void Emm_V5_Modify_Firmware_Type(uint8_t addr, bool svF, bool fwtype);
//修改开环/闭环控制模式(Y42)
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode);
//修改电机运动方向(Y42)
void Emm_V5_Modify_Motor_Dir(uint8_t addr, bool svF, bool dir);
//修改按键锁定功能(Y42)
void Emm_V5_Modify_Lock_Btn(uint8_t addr, bool svF, bool lockbtn);
//修改输入速度值是否缩小10倍显示(Y42)
void Emm_V5_Modify_S_Vel(uint8_t addr, bool svF, bool s_vel);
//修改开环模式电流
void Emm_V5_Modify_OM_ma(uint8_t addr, bool svF, uint16_t om_ma);
//修改闭环模式电流
void Emm_V5_Modify_FOC_mA(uint8_t addr, bool svF, uint16_t foc_mA);
//读取PID参数
void Emm_V5_Read_PID_Params(uint8_t addr);
//修改PID参数
void Emm_V5_Modify_PID_Params(uint8_t addr, bool svF, uint32_t kp, uint32_t ki, uint32_t kd);
//获取DMX512协议参数(Y42)
void Emm_V5_Read_DMX512_Params(uint8_t addr);
//修改DMX512协议参数(Y42)
void Emm_V5_Modify_DMX512_Params(uint8_t addr, bool svF, uint16_t tch, uint8_t nch, uint8_t mode, uint16_t vel, uint16_t acc, uint16_t vel_step, uint32_t pos_step);
//读取位置到达窗口(Y42)
void Emm_V5_Read_Pos_Window(uint8_t addr);
//修改位置到达窗口(Y42)
void Emm_V5_Modify_Pos_Window(uint8_t addr, bool svF, uint16_t prw);
//读取过热过流保护阈值(Y42)
void Emm_V5_Read_Otocp(uint8_t addr);
//修改过热过流保护阈值(Y42)
void Emm_V5_Modify_Otocp(uint8_t addr, bool svF, uint16_t otp, uint16_t ocp, uint16_t time_ms);
//读取心跳保护时间(Y42)
void Emm_V5_Read_Heart_Protect(uint8_t addr);
//修改心跳保护时间(Y42)
void Emm_V5_Modify_Heart_Protect(uint8_t addr, bool svF, uint32_t hp);
//读取积分限幅/微分系数(Y42)
void Emm_V5_Read_Integral_Limit(uint8_t addr);
//修改积分限幅/微分系数(Y42)
void Emm_V5_Modify_Integral_Limit(uint8_t addr, bool svF, uint32_t il);
//获取系统状态信息函数组
//读取系统状态参数
void Emm_V5_Read_System_State_Params(uint8_t addr);
//读取电机配置参数
void Emm_V5_Read_Motor_Conf_Params(uint8_t addr);

//多机指令(多机指令将指令缓存后统一发送)
//编码器函数组(多机指令版本)
//编码器脉冲校准 - 缓存到多机指令缓冲
void Emm_V5_MMCL_Trig_Encoder_Cal(uint8_t addr);
//复位电机 - 缓存到多机指令缓冲
void Emm_V5_MMCL_Reset_Motor(uint8_t addr);
//将当前脉冲位置清零 - 缓存到多机指令缓冲
void Emm_V5_MMCL_Reset_CurPos_To_Zero(uint8_t addr);
//清除堵转保护 - 缓存到多机指令缓冲
void Emm_V5_MMCL_Reset_Clog_Pro(uint8_t addr);
//恢复电机运行 - 缓存到多机指令缓冲
void Emm_V5_MMCL_Restore_Motor(uint8_t addr);
//运动控制函数组(多机指令版本)
//电机使能控制 - 缓存到多机指令缓冲
void Emm_V5_MMCL_En_Control(uint8_t addr, bool state, bool snF);
//速度模式控制 - 缓存到多机指令缓冲
void Emm_V5_MMCL_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
//位置模式控制 - 缓存到多机指令缓冲
void Emm_V5_MMCL_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, bool snF);
//设置快捷位置模式运动参数
void Emm_V5_MMCL_Set_QPos_Params(uint8_t addr, uint16_t vel, uint8_t acc, uint8_t raF, bool snF);
//快捷位置模式控制
void Emm_V5_MMCL_QPos_Control(uint8_t addr, int32_t clk);
//电机立即停止运动 - 缓存到多机指令缓冲
void Emm_V5_MMCL_Stop_Now(uint8_t addr, bool snF);
//触发多机同步开始运动 - 缓存到多机指令缓冲
void Emm_V5_MMCL_Synchronous_motion(uint8_t addr);
//原点回归函数组(多机指令版本)
//设置当前堵转位置为原点 - 缓存到多机指令缓冲
void Emm_V5_MMCL_Origin_Set_O(uint8_t addr, bool svF);
//触发回原点 - 缓存到多机指令缓冲
void Emm_V5_MMCL_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
//强制中断并退出回零 - 缓存到多机指令缓冲
void Emm_V5_MMCL_Origin_Interrupt(uint8_t addr);
//修改回零参数 - 缓存到多机指令缓冲
void Emm_V5_MMCL_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
//读取撞块回零返回角度(X42S/Y42) - 缓存到多机指令缓冲
void X_V2_MMCL_Origin_Read_SL_RP(uint8_t addr);
//修改撞块回零返回角度(X42S/Y42) - 缓存到多机指令缓冲
void X_V2_MMCL_Origin_Modify_SL_RP(uint8_t addr, bool svF, uint16_t sl_rp);
//获取系统运行参数函数组(多机指令版本)
//定时返回信息指令(Y42) - 缓存到多机指令缓冲
void Emm_V5_MMCL_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms);
//读取系统参数 - 缓存到多机指令缓冲
void Emm_V5_MMCL_Read_Sys_Params(uint8_t addr, SysParams_t s);
//读写参数设置函数组(多机指令版本)

#endif
