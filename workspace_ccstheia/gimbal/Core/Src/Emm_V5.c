//ZDT X42S步进电机Emm_V5固件通信协议库(MSPM0移植版)
#include "Emm_V5.h"

//Emm_V5.0步进电机控制库
//作者：ZHANGDATOU
//技术支持：张大头闭环步进
//淘宝店铺：https://zhangdatou.taobao.com
//CSDN博客：http s://blog.csdn.net/zhangdatou666
//qq技术群：262438510

//=== MSPM0 兼容层 ===
#define __IO  volatile

// 阻塞 TX: 逐字节发送,每字节等待完成
#define HAL_UART_Transmit_DMA(uart, buf, len) \
    do { \
        for (uint16_t _emmi = 0; _emmi < (len); _emmi++) { \
            DL_UART_Main_transmitData((uart), ((uint8_t*)(buf))[_emmi]); \
            while (DL_UART_Main_isBusy(uart)); \
        } \
    } while(0)

// 阻塞 TX (含timeout参数,兼容 STM32 huart2 特殊处理)
#define HAL_UART_Transmit(uart, buf, len, timeout) \
    HAL_UART_Transmit_DMA(uart, buf, len)
//====================

__IO uint16_t MMCL_count = 0, MMCL_cmd[MMCL_LEN] = {0}; //多机指令计数和缓冲区

UART_HandleTypeDef* g_emm_uart = MTR1_UART_INST;  //当前电机串口号,main.c可在调用前切换

//编码器函数组
/**
 * @brief 编码器脉冲校准
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 直接发送校准指令到驱动器
 *         Emm_V5_MMCL_Trig_Encoder_Cal
 */
void Emm_V5_Trig_Encoder_Cal(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x06;                        //功能码
  cmd[2] =  0x45;                        //数据
  cmd[3] =  0x6B;                        //校验字节
  
  //发送指令
	HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 4);
}

/**
 * @brief 复位电机(Y42)
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 直接发送复位指令到驱动器
 *         Emm_V5_MMCL_Reset_Motor
 */
void Emm_V5_Reset_Motor(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x08;                        //功能码
  cmd[2] =  0x97;                        //数据
  cmd[3] =  0x6B;                        //校验字节
  
  //发送指令
	HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 4);
}

/**
 * @brief 将当前脉冲位置清零
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 直接发送清零指令到驱动器
 *         Emm_V5_MMCL_Reset_CurPos_To_Zero
 */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x0A;                        //功能码
  cmd[2] =  0x6D;                        //数据
  cmd[3] =  0x6B;                        //校验字节
  
  //发送指令
	HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 4);
}

/**
 * @brief 清除堵转保护
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 直接发送清除堵转保护指令到驱动器
 *         Emm_V5_MMCL_Reset_Clog_Pro
 */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x0E;                        //功能码
  cmd[2] =  0x52;                        //数据
  cmd[3] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 4);
}

/**
 * @brief 恢复电机运行
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 直接发送恢复运行指令到驱动器
 *         Emm_V5_MMCL_Restore_Motor
 */
void Emm_V5_Restore_Motor(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x0F;                        //功能码
  cmd[2] =  0x5F;                        //数据
  cmd[3] =  0x6B;                        //校验字节
  
  //发送指令
	HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 4);
}

//运动控制函数组
/**
 * @brief 多机指令(Y42)
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 将MMCL缓冲区内所有缓存指令打包发送
 *         Emm_V5_MMCL_Multi_Motor_Cmd
 */
void Emm_V5_Multi_Motor_Cmd(uint8_t addr)
{
  uint16_t i = 0, j = 0, len = 0; __IO static uint8_t cmd[MMCL_LEN] = {0};
  
	//缓存指令数量大于0才发送
	if(MMCL_count > 0)
	{
		//计算总帧长:缓存指令数+5字节(地址+功能码+长度2+校验)
		len = MMCL_count + 5;
		
		//组装指令帧
		cmd[0] = addr;                       //地址
		cmd[1] = 0xAA;                       //功能码
		cmd[2] = (uint8_t)(len >> 8);				 //长度高8位
		cmd[3] = (uint8_t)(len); 		 				 //长度低8位
		for(i=0,j=4; i < MMCL_count; i++,j++) { cmd[j] = MMCL_cmd[i]; }
		cmd[j] = 0x6B; ++j;                  //校验字节
		
		//发送指令,发送后清空缓存计数
		HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, j); MMCL_count = 0;
	}
	else
	{
		MMCL_count = 0;
	}
}

/**
 * @brief 电机使能控制
 * @param addr 驱动器地址
 * @param state 使能状态 true为使能电机,false为关闭电机
 * @param snF 多机同步标志 false为不启用,true为启用
 * @return 地址+功能码+运行状态+校验字节
 * @note 控制电机使能/失能,支持多机同步
 *         Emm_V5_MMCL_En_Control
 */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0xF3;                        //功能码
  cmd[2] =  0xAB;                        //数据
  cmd[3] =  (uint8_t)state;              //使能状态
  cmd[4] =  snF;                         //多机同步运动标志
  cmd[5] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 6);
}

/**
 * @brief 速度模式控制
 * @param addr 驱动器地址
 * @param dir 方向 0为CW,非0为CCW
 * @param vel 速度 范围0-5000RPM
 * @param acc 加速度 范围0-255,0为立即启动
 * @param snF 多机同步标志 false为不启用,true为启用
 * @return 地址+功能码+运行状态+校验字节
 * @note 控制电机以指定速度运行
 *         Emm_V5_MMCL_Vel_Control
 */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区

  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0xF6;                        //功能码
  cmd[2] =  dir;                         //方向
  cmd[3] =  (uint8_t)(vel >> 8);         //速度(RPM)高8位
  cmd[4] =  (uint8_t)(vel >> 0);         //速度(RPM)低8位
  cmd[5] =  acc;                         //加速度,0为立即启动
  cmd[6] =  snF;                         //多机同步运动标志
  cmd[7] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 8);
}

/**
 * @brief 位置模式控制
 * @param addr 驱动器地址
 * @param dir 方向 0为CW,非0为CCW
 * @param vel 速度(RPM) 范围0-5000RPM
 * @param acc 加速度 范围0-255,0为立即启动
 * @param clk 脉冲数 范围0-(2^32-1)
 * @param raF 运动标志 0为绝对位置,1为增量运动,2为相对当前位置实时位置运动
 * @param snF 多机同步标志 false为不启用,true为启用
 * @return 地址+功能码+运行状态+校验字节
 * @note 控制电机运动到指定位置
 *         Emm_V5_MMCL_Pos_Control
 */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, bool snF)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区

  //组装指令帧
  cmd[0]  =  addr;                       //地址
  cmd[1]  =  0xFD;                       //功能码
  cmd[2]  =  dir;                        //方向
  cmd[3]  =  (uint8_t)(vel >> 8);        //速度(RPM)高8位
  cmd[4]  =  (uint8_t)(vel >> 0);        //速度(RPM)低8位
  cmd[5]  =  acc;                        //加速度,0为立即启动
  cmd[6]  =  (uint8_t)(clk >> 24);       //脉冲数(bit24-bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);       //脉冲数(bit16-bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);        //脉冲数(bit8-bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);        //脉冲数(bit0-bit7)
  cmd[10] =  raF;                        //绝对/相对标志,false为绝对,true为增量
  cmd[11] =  snF;                        //多机同步运动标志,false为不启用,true为启用
  cmd[12] =  0x6B;                       //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 13);

}

/**
 * @brief 设置快捷位置模式运动参数
 * @param addr 驱动器地址
 * @param vel 速度(RPM) 范围0-5000RPM
 * @param acc 加速度 范围0-255,0为立即启动
 * @param raF 运动标志 0为绝对位置,1为增量运动,2为相对当前位置实时位置运动
 * @param snF 多机同步标志 false为不启用,true为启用
 * @return 地址+功能码+运行状态+校验字节
 * @note 预设置快捷位置参数,后续通过QPos_Control触发运动
 *         Emm_V5_MMCL_Set_QPos_Params
 */
void Emm_V5_Set_QPos_Params(uint8_t addr, uint16_t vel, uint8_t acc, uint8_t raF, bool snF)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区

  //组装指令帧
  cmd[0]  =  addr;                       //地址
  cmd[1]  =  0xF1;                       //功能码
  cmd[2]  =  (uint8_t)(vel >> 8);        //速度(RPM)高8位
  cmd[3]  =  (uint8_t)(vel >> 0);        //速度(RPM)低8位
  cmd[4]  =  acc;                        //加速度,0为立即启动
  cmd[5] =  raF;                         //绝对/相对标志,false为绝对,true为增量
  cmd[6] =  snF;                         //多机同步运动标志,false为不启用,true为启用
  cmd[7] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 8);
}

/**
 * @brief 快捷位置模式控制
 * @param addr 驱动器地址
 * @param clk 脉冲数(带符号),默认16细分+3200正向转一圈,-3200反向转一圈
 * @return 地址+功能码+运行状态+校验字节
 * @note 使用预设参数直接控制电机运动
 *         Emm_V5_MMCL_QPos_Control
 */
void Emm_V5_QPos_Control(uint8_t addr, int32_t clk)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区

  //组装指令帧
  cmd[0]  =  addr;                       //地址
  cmd[1]  =  0xFC;                       //功能码
  cmd[2]  =  (uint8_t)(clk >> 24);       //脉冲数(bit24-bit31)
  cmd[3]  =  (uint8_t)(clk >> 16);       //脉冲数(bit16-bit23)
  cmd[4]  =  (uint8_t)(clk >> 8);        //脉冲数(bit8-bit15)
  cmd[5]  =  (uint8_t)(clk >> 0);        //脉冲数(bit0-bit7)
  cmd[6] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 7);
}

/**
 * @brief 电机立即停止
 * @param addr 驱动器地址
 * @param snF 多机同步标志 false为不启用,true为启用
 * @return 地址+功能码+运行状态+校验字节
 * @note 紧急停止电机运动
 *         Emm_V5_MMCL_Stop_Now
 */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0xFE;                        //功能码
  cmd[2] =  0x98;                        //数据
  cmd[3] =  snF;                         //多机同步运动标志
  cmd[4] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 5);
}

/**
 * @brief 触发多机同步运动
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 所有已缓存多机指令同时执行
 *         Emm_V5_MMCL_Synchronous_motion
 */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0xFF;                        //功能码
  cmd[2] =  0x66;                        //数据
  cmd[3] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 4);
}

//原点回归函数组
/**
 * @brief 设置当前堵转位置为原点
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @return 地址+功能码+运行状态+校验字节
 * @note 将当前位置设为原点,可选择是否保存到驱动器
 *         Emm_V5_MMCL_Origin_Set_O
 */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x93;                        //功能码
  cmd[2] =  0x88;                        //数据
  cmd[3] =  svF;                         //是否存储标志,false为不存储,true为存储
  cmd[4] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 5);
}

/**
 * @brief 触发回原点
 * @param addr 驱动器地址
 * @param o_mode 回零模式 0为堵圈就近找零,1为堵圈方向找零,2为堵圈限位撞块找零,3为堵圈限位开关回零
 * @param snF 多机同步标志 false为不启用,true为启用
 * @return 地址+功能码+运行状态+校验字节
 * @note 触发驱动器执行回原点动作
 *         Emm_V5_MMCL_Origin_Trigger_Return
 */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x9A;                        //功能码
  cmd[2] =  o_mode;                      //回零模式,0为堵圈就近找零,1为堵圈方向找零,2为堵圈限位撞块找零,3为堵圈限位开关回零
  cmd[3] =  snF;                         //多机同步运动标志,false为不启用,true为启用
  cmd[4] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 5);
}

/**
 * @brief 强制中断并退出回零
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 中断正在执行的回零操作
 *         Emm_V5_MMCL_Origin_Interrupt
 */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x9C;                        //功能码
  cmd[2] =  0x48;                        //数据
  cmd[3] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 4);
}

/**
 * @brief 读取回零参数
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 从驱动器读取当前回零参数配置
 *         Emm_V5_MMCL_Origin_Read_Params
 */
void Emm_V5_Origin_Read_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x22;                        //功能码
  cmd[2] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 3);
}

/**
 * @brief 修改回零参数
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param o_mode 回零模式 0为堵圈就近找零,1为堵圈方向找零,2为堵圈限位撞块找零,3为堵圈限位开关回零
 * @param o_dir 回零方向 0为CW,非0为CCW
 * @param o_vel 回零速度,单位RPM(转/分钟)
 * @param o_tm 回零超时时间,单位秒
 * @param sl_vel 限位撞块低速转速,单位RPM(转/分钟)
 * @param sl_ma 限位撞块碰撞电流,单位Ma(毫安)
 * @param sl_ms 限位撞块碰撞时间,单位Ms(毫秒)
 * @param potF 上电自动回零 false为不启用,true为启用
 * @return 地址+功能码+运行状态+校验字节
 * @note 配置回零相关参数
 *         Emm_V5_MMCL_Origin_Modify_Params
 */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  __IO static uint8_t cmd[32] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x4C;                        //功能码
  cmd[2] =  0xAE;                        //数据
  cmd[3] =  svF;                         //是否存储标志,false为不存储,true为存储
  cmd[4] =  o_mode;                      //回零模式,0为堵圈就近找零,1为堵圈方向找零,2为堵圈限位撞块找零,3为堵圈限位开关回零
  cmd[5] =  o_dir;                       //回零方向
  cmd[6]  =  (uint8_t)(o_vel >> 8);      //回零速度(RPM)高8位
  cmd[7]  =  (uint8_t)(o_vel >> 0);      //回零速度(RPM)低8位
  cmd[8]  =  (uint8_t)(o_tm >> 24);      //回零超时时间(bit24-bit31)
  cmd[9]  =  (uint8_t)(o_tm >> 16);      //回零超时时间(bit16-bit23)
  cmd[10] =  (uint8_t)(o_tm >> 8);       //回零超时时间(bit8-bit15)
  cmd[11] =  (uint8_t)(o_tm >> 0);       //回零超时时间(bit0-bit7)
  cmd[12] =  (uint8_t)(sl_vel >> 8);     //限位撞块低速转速(RPM)高8位
  cmd[13] =  (uint8_t)(sl_vel >> 0);     //限位撞块低速转速(RPM)低8位
  cmd[14] =  (uint8_t)(sl_ma >> 8);      //限位撞块碰撞电流(Ma)高8位
  cmd[15] =  (uint8_t)(sl_ma >> 0);      //限位撞块碰撞电流(Ma)低8位
  cmd[16] =  (uint8_t)(sl_ms >> 8);      //限位撞块碰撞时间(Ms)高8位
  cmd[17] =  (uint8_t)(sl_ms >> 0);      //限位撞块碰撞时间(Ms)低8位
  cmd[18] =  potF;                       //上电自动回零,false为不启用,true为启用
  cmd[19] =  0x6B;                       //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 20);
}

/**
 * @brief 读取撞块回零返回角度(X42S/Y42)
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 读取撞块回零后的返回角度
 *         X_V2_MMCL_Origin_Read_SL_RP
 */
void X_V2_Origin_Read_SL_RP(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x3F;                        //功能码
  cmd[2] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 3);
}

/**
 * @brief 修改撞块回零返回角度(X42S/Y42)
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param sl_rp 撞块回零返回角度,单位0.1度,如40表示4.0度
 * @return 地址+功能码+运行状态+校验字节
 * @note 配置撞块回零返回角度
 *         X_V2_MMCL_Origin_Modify_SL_RP
 */
void X_V2_Origin_Modify_SL_RP(uint8_t addr, bool svF, uint16_t sl_rp)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0]  =  addr;                       //地址
  cmd[1]  =  0x5C;                       //功能码
  cmd[2]  =  0xAC;                       //数据
  cmd[3]  =  svF;                        //是否存储标志,false为不存储,true为存储
  cmd[4]  =  (uint8_t)(sl_rp >> 8);			 //撞块回零返回角度,单位0.1度
	cmd[5]  =  (uint8_t)(sl_rp >> 0);
  cmd[6]  =  0x6B;                       //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 7);
}

//获取系统运行参数函数组
/**
 * @brief 定时返回信息指令(Y42)
 * @param addr 驱动器地址
 * @param s 系统运行参数枚举
 * @param time_ms 定时返回时间
 * @return 地址+功能码+运行状态+校验字节
 * @note 设置驱动器定时自动返回指定参数
 *         Emm_V5_MMCL_Auto_Return_Sys_Params_Timed
 */
void Emm_V5_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms)
{
  uint8_t i = 0; __IO static uint8_t cmd[16] = {0}; //指令缓冲区
  
  //组装指令帧
  cmd[i] = addr; ++i;                    //地址

  cmd[i] = 0x11; ++i;                    //功能码

  cmd[i] = 0x18; ++i;                    //数据

  switch(s)                              //信息选择
  {
    case S_VBUS : cmd[i] = 0x24; ++i; break;	 //获取总线电压
		case S_CBUS : cmd[i] = 0x26; ++i; break;	 //获取总线电流
    case S_CPHA : cmd[i] = 0x27; ++i; break;	 //获取相电流
		case S_ENCO : cmd[i] = 0x29; ++i; break;	 //获取编码器原始值
		case S_CLKC : cmd[i] = 0x30; ++i; break;	 //获取实时脉冲数
    case S_ENCL : cmd[i] = 0x31; ++i; break;	 //获取编码器经校准后的锁定值
		case S_CLKI : cmd[i] = 0x32; ++i; break;	 //获取脉冲输入数
    case S_TPOS : cmd[i] = 0x33; ++i; break;	 //获取轴目标位置
    case S_SPOS : cmd[i] = 0x34; ++i; break;	 //获取轴实时设定目标位置
		case S_VEL  : cmd[i] = 0x35; ++i; break;	 //获取轴实时转速
    case S_CPOS : cmd[i] = 0x36; ++i; break;	 //获取轴实时位置
    case S_PERR : cmd[i] = 0x37; ++i; break;	 //获取轴位置跟随差
		case S_VBAT : cmd[i] = 0x38; ++i; break;	 //获取线圈充电电池电压(Y42)
		case S_TEMP : cmd[i] = 0x39; ++i; break;	 //获取轴实时温度(Y42)
    case S_FLAG : cmd[i] = 0x3A; ++i; break;	 //获取轴状态标志位
    case S_OFLAG: cmd[i] = 0x3B; ++i; break;	 //获取回零状态标志位
		case S_OAF  : cmd[i] = 0x3C; ++i; break;	 //获取报警状态标志位+回零状态标志位(Y42)
		case S_PIN  : cmd[i] = 0x3D; ++i; break;	 //获取引脚状态(Y42)
    default: break;
  }
	
	cmd[i] = (uint8_t)(time_ms >> 8);  ++i;	 //定时时间
	cmd[i] = (uint8_t)(time_ms >> 0);  ++i;

  cmd[i] = 0x6B; ++i;                    //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, i);
}

/**
 * @brief 读取系统参数
 * @param addr 驱动器地址
 * @param s 系统运行参数枚举
 * @return 地址+功能码+运行状态+校验字节
 * @note 读取驱动器指定系统参数
 *         Emm_V5_MMCL_Read_Sys_Params
 */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0; __IO static uint8_t cmd[16] = {0};
  
  // װ������
  cmd[i] = addr; ++i;                   // ��ַ

  switch(s)                             // ������
  {
    case S_VBUS : cmd[i] = 0x24; ++i; break;	// ��ȡ���ߵ�ѹ
		case S_CBUS : cmd[i] = 0x26; ++i; break;	// ��ȡ���ߵ���
    case S_CPHA : cmd[i] = 0x27; ++i; break;	// ��ȡ�����
		case S_ENCO : cmd[i] = 0x29; ++i; break;	// ��ȡ������ԭʼֵ
		case S_CLKC : cmd[i] = 0x30; ++i; break;	// ��ȡʵʱ������
    case S_ENCL : cmd[i] = 0x31; ++i; break;	// ��ȡ�������Ի�У׼��ı�����ֵ
		case S_CLKI : cmd[i] = 0x32; ++i; break;	// ��ȡ����������
    case S_TPOS : cmd[i] = 0x33; ++i; break;	// ��ȡ���Ŀ��λ��
    case S_SPOS : cmd[i] = 0x34; ++i; break;	// ��ȡ���ʵʱ�趨��Ŀ��λ��
		case S_VEL  : cmd[i] = 0x35; ++i; break;	// ��ȡ���ʵʱת��
    case S_CPOS : cmd[i] = 0x36; ++i; break;	// ��ȡ���ʵʱλ��
    case S_PERR : cmd[i] = 0x37; ++i; break;	// ��ȡ���λ�����
		case S_VBAT : cmd[i] = 0x38; ++i; break;	// ��ȡ��Ȧ��������ص�ѹ��Y42��
		case S_TEMP : cmd[i] = 0x39; ++i; break;	// ��ȡ���ʵʱ�¶ȣ�Y42��
    case S_FLAG : cmd[i] = 0x3A; ++i; break;	// ��ȡ���״̬��־λ
    case S_OFLAG: cmd[i] = 0x3B; ++i; break;	// ��ȡ����״̬��־λ
		case S_OAF  : cmd[i] = 0x3C; ++i; break;	// ��ȡ���״̬��־λ + ����״̬��־λ��Y42��
		case S_PIN  : cmd[i] = 0x3D; ++i; break;	// ��ȡ����״̬��Y42��
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // У���ֽ�
  
  // ��������
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, i);
}

//读写参数设置函数组
/**
 * @brief 修改电机ID地址
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param id 新ID地址 默认1,可修改为1-255,0为广播地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 修改驱动器的ID地址
 *         Emm_V5_Modify_MicroStep
 */
void Emm_V5_Modify_Motor_ID(uint8_t addr, bool svF, uint8_t id)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0xAE;                        //功能码
  cmd[2] =  0x4B;                        //数据
  cmd[3] =  svF;                         //是否存储标志,false为不存储,true为存储
  cmd[4] =  id;                  				 //新ID地址,默认1,可修改为1-255,0为广播地址
  cmd[5] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 6);
}

/**
 * @brief 修改细分值
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param mstep 细分数 默认16,可修改为1-255,0为256细分
 * @return 地址+功能码+运行状态+校验字节
 * @note 修改驱动器的步进细分数
 *         Emm_V5_Modify_MicroStep
 */
void Emm_V5_Modify_MicroStep(uint8_t addr, bool svF, uint8_t mstep)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x84;                        //功能码
  cmd[2] =  0x8A;                        //数据
  cmd[3] =  svF;                         //是否存储标志,false为不存储,true为存储
  cmd[4] =  mstep;                 	 		 //细分数,默认16,可修改为1-255,0为256细分
  cmd[5] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 6);
}

/**
 * @brief 修改电机方向标志
 * @param addr 驱动器地址
 * @param pdf 方向标志
 * @return 地址+功能码+运行状态+校验字节
 * @note 修改电机运动方向标志
 *         Emm_V5_Modify_PDFlag
 */
void Emm_V5_Modify_PDFlag(uint8_t addr, bool pdf)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x50;                        //功能码
  cmd[2] =  pdf;                 	 			 //方向标志
  cmd[3] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 4);
}

/**
 * @brief 读取选择参数状态(Y42)
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 读取驱动器选择参数状态
 *         Emm_V5_Read_Opt_Param_Sta
 */
void Emm_V5_Read_Opt_Param_Sta(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x1A;                        //功能码
  cmd[2] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 3);
}

/**
 * @brief 修改电机类型(Y42)
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param mottype 电机类型 0为1.8度步进电机,1为0.9度步进电机
 * @return 地址+功能码+运行状态+校验字节
 * @note 修改驱动器适配的电机类型
 *         Emm_V5_Modify_Motor_Type
 */
void Emm_V5_Modify_Motor_Type(uint8_t addr, bool svF, bool mottype)
{
  __IO static uint8_t cmd[16] = {0}; uint8_t MotType = 0; //指令缓冲区,电机类型值
  
	//0.9度电机用25细分,1.8度电机用50细分
	if(mottype) { MotType = 25; } else { MotType = 50; }
	
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0xD7;                        //功能码
  cmd[2] =  0x35;                        //数据
  cmd[3] =  svF;                         //是否存储标志,false为不存储,true为存储
  cmd[4] =  MotType;                 	 	 //电机类型,25为0.9度,50为1.8度
  cmd[5] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 6);
}

/**
 * @brief 修改固件类型(Y42)
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param fwtype 固件类型 0为X固件,1为Emm固件
 * @return 地址+功能码+运行状态+校验字节
 * @note 修改驱动器固件类型
 *         Emm_V5_Modify_Firmware_Type
 */
void Emm_V5_Modify_Firmware_Type(uint8_t addr, bool svF, bool fwtype)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0xD5;                        //功能码
  cmd[2] =  0x69;                        //数据
  cmd[3] =  svF;                         //是否存储标志,false为不存储,true为存储
  cmd[4] =  fwtype;                 	 	 //固件类型,0为X固件,1为Emm固件
  cmd[5] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 6);
}

/**
 * @brief 修改开环/闭环控制模式(Y42)
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param ctrl_mode 控制模式 0为开环模式,1为闭环FOC模式
 * @return 地址+功能码+运行状态+校验字节
 * @note 切换开环或闭环控制模式
 *         Emm_V5_Modify_Ctrl_Mode
 */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x46;                        //功能码
  cmd[2] =  0x69;                        //数据
  cmd[3] =  svF;                         //是否存储标志,false为不存储,true为存储
  cmd[4] =  ctrl_mode;                   //控制模式,0开环,1闭环FOC
  cmd[5] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 6);
}

/**
 * @brief 修改电机运动方向(Y42)
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param dir 运动方向 0为CW(顺时针),1为CCW
 * @return 地址+功能码+运行状态+校验字节
 * @note 修改电机默认运动方向
 *         Emm_V5_Modify_Motor_Dir
 */
void Emm_V5_Modify_Motor_Dir(uint8_t addr, bool svF, bool dir)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0xD4;                        //功能码
  cmd[2] =  0x60;                        //数据
  cmd[3] =  svF;                         //是否存储标志,false为不存储,true为存储
  cmd[4] =  dir;                  			 //运动方向,0为CW(顺时针),1为CCW
  cmd[5] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 6);
}

/**
 * @brief 修改按键锁定功能(Y42)
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param lock 按键锁定功能 0为Disable,1为Enable
 * @return 地址+功能码+运行状态+校验字节
 * @note 启用或禁用驱动器按键功能
 *         Emm_V5_Modify_Lock_Btn
 */
void Emm_V5_Modify_Lock_Btn(uint8_t addr, bool svF, bool lock)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0xD0;                        //功能码
  cmd[2] =  0xB3;                        //数据
  cmd[3] =  svF;                         //是否存储标志,false为不存储,true为存储
  cmd[4] =  lock;                  			 //按键锁定,0为Disable,1为Enable
  cmd[5] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 6);
}

/**
 * @brief 修改输入速度值是否缩小10倍显示(Y42)
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param s_vel 速度值缩小显示 0为Disable,1为Enable
 * @return 地址+功能码+运行状态+校验字节
 * @note 配置速度显示是否缩小10倍
 *         Emm_V5_Modify_S_Vel
 */
void Emm_V5_Modify_S_Vel(uint8_t addr, bool svF, bool s_vel)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x4F;                        //功能码
  cmd[2] =  0x71;                        //数据
  cmd[3] =  svF;                         //是否存储标志,false为不存储,true为存储
  cmd[4] =  s_vel;                  		 //速度值缩小10倍显示,0为Disable,1为Enable
  cmd[5] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 6);
}

/**
 * @brief 修改开环模式电流
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param om_ma 开环模式电流,单位mA
 * @return 地址+功能码+运行状态+校验字节
 * @note 配置开环模式的运行电流
 *         Emm_V5_Modify_OM_mA
 */
void Emm_V5_Modify_OM_mA(uint8_t addr, bool svF, uint16_t om_ma)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x44;                        //功能码
  cmd[2] =  0x33;                        //数据
  cmd[3] =  svF;                         //是否存储标志,false为不存储,true为存储
  cmd[4] =  (uint8_t)(om_ma >> 8);			 //开环模式电流(mA)高8位
	cmd[5] =  (uint8_t)(om_ma >> 0);        //开环模式电流(mA)低8位
  cmd[6] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 7);
}

/**
 * @brief 修改闭环模式电流
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param foc_mA 闭环FOC模式电流,单位mA
 * @return 地址+功能码+运行状态+校验字节
 * @note 配置闭环FOC模式的运行电流
 *         Emm_V5_Modify_FOC_mA
 */
void Emm_V5_Modify_FOC_mA(uint8_t addr, bool svF, uint16_t foc_mA)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x45;                        //功能码
  cmd[2] =  0x66;                        //数据
  cmd[3] =  svF;                         //是否存储标志,false为不存储,true为存储
  cmd[4] =  (uint8_t)(foc_mA >> 8);			 //闭环FOC模式电流(mA)高8位
	cmd[5] =  (uint8_t)(foc_mA >> 0);      //闭环FOC模式电流(mA)低8位
  cmd[6] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 7);
}

/**
 * @brief 读取PID参数
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 从驱动器读取当前PID参数
 *         Emm_V5_Read_PID_Params
 */
void Emm_V5_Read_PID_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x21;                        //功能码
  cmd[2] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 3);
}

/**
 * @brief 修改PID参数
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param kp 比例系数 默认Y42/18000
 * @param ki 积分系数 默认Y42/10
 * @param kd 微分系数 默认Y42/18000
 * @return 地址+功能码+运行状态+校验字节
 * @note 修改驱动器的PID控制参数
 *         Emm_V5_Modify_PID_Params
 */
void Emm_V5_Modify_PID_Params(uint8_t addr, bool svF, uint32_t kp, uint32_t ki, uint32_t kd)
{
  __IO static uint8_t cmd[20] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0]  =  addr;                       //地址
  cmd[1]  =  0x4A;                       //功能码
  cmd[2]  =  0xC3;                       //数据
  cmd[3]  =  svF;                        //是否存储标志,false为不存储,true为存储
  cmd[4]  =  (uint8_t)(kp >> 24);				 //kp高8位
	cmd[5]  =  (uint8_t)(kp >> 16);
	cmd[6]  =  (uint8_t)(kp >> 8);
	cmd[7]  =  (uint8_t)(kp >> 0);         //kp低8位
	cmd[8]  =  (uint8_t)(ki >> 24);				 //ki高8位
	cmd[9]  =  (uint8_t)(ki >> 16);
	cmd[10] =  (uint8_t)(ki >> 8);
	cmd[11] =  (uint8_t)(ki >> 0);         //ki低8位
	cmd[12] =  (uint8_t)(kd >> 24);				 //kd高8位
	cmd[13] =  (uint8_t)(kd >> 16);
	cmd[14] =  (uint8_t)(kd >> 8);
	cmd[15] =  (uint8_t)(kd >> 0);         //kd低8位
  cmd[16] =  0x6B;                       //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 17);
}

/**
 * @brief 获取DMX512协议参数(Y42)
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 读取驱动器DMX512协议配置
 *         Emm_V5_Read_DMX512_Params
 */
void Emm_V5_Read_DMX512_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x49;                        //功能码
	cmd[2] =  0x78;                        //数据
  cmd[3] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 4);
}

/**
 * @brief 修改DMX512协议参数(Y42)
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param tch 总通道数 默认192,需与DMX512控制器总通道数一致
 * @param nch 每轴占用通道数 默认1,1为单通道模式,2为双通道模式
 * @param mode 运动模式 默认1,0为绝对位置模式运动,1为增量实时位置运动
 * @param vel 单通道模式运动速度 默认值1000,单位RPM
 * @param acc 加速度 acc=设定值/8=125,详见5.3.12位置模式控制(Emm)
 * @param vel_step 双通道模式速度步长 默认值10,运动速度为(通道值*10)RPM
 * @param pos_step 双通道模式运动步长 默认值100,旋转角度为(通道值*10.0)度
 * @return 地址+功能码+运行状态+校验字节
 * @note 配置DMX512协议参数
 *         Emm_V5_Modify_DMX512_Params
 */
void Emm_V5_Modify_DMX512_Params(uint8_t addr, bool svF, uint16_t tch, uint8_t nch, uint8_t mode, uint16_t vel, uint16_t acc, uint16_t vel_step, uint32_t pos_step)
{
  __IO static uint8_t cmd[32] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0]  =  addr;                       //地址
  cmd[1]  =  0xD9;                       //功能码
  cmd[2]  =  0x90;                       //数据
  cmd[3]  =  svF;                        //是否存储标志,false为不存储,true为存储
  cmd[4]  =  (uint8_t)(tch >> 8);     	 //总通道数高8位
  cmd[5]  =  (uint8_t)(tch >> 0);        //总通道数低8位
	cmd[6]  =  nch;                        //每轴占用通道数
	cmd[7]  =  mode;                       //运动模式
	cmd[8]  =  (uint8_t)(vel >> 8);     	 //单通道模式运动速度高8位
  cmd[9]  =  (uint8_t)(vel >> 0);        //单通道模式运动速度低8位
	cmd[10] =  (uint8_t)(acc >> 8);     	 //加速度高8位
  cmd[11] =  (uint8_t)(acc >> 0);        //加速度低8位
	cmd[12] =  (uint8_t)(vel_step >> 8);   //双通道速度步长高8位
  cmd[13] =  (uint8_t)(vel_step >> 0);   //双通道速度步长低8位
  cmd[14]  = (uint8_t)(pos_step >> 24);	 //双通道运动步长(bit24-bit31)
  cmd[15]  = (uint8_t)(pos_step >> 16);
  cmd[16] =  (uint8_t)(pos_step >> 8);
  cmd[17] =  (uint8_t)(pos_step >> 0);   //双通道运动步长(bit0-bit7)
  cmd[18] =  0x6B;                       //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 19);
}

/**
 * @brief 获取位置到达窗口(Y42)
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 读取位置到达窗口配置
 *         Emm_V5_Read_Pos_Window
 */
void Emm_V5_Read_Pos_Window(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x41;                        //功能码
  cmd[2] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 3);
}

/**
 * @brief 修改位置到达窗口(Y42)
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param prw 位置到达窗口 默认值8,即0.8度
 * @return 地址+功能码+运行状态+校验字节
 * @note 配置位置到达判定窗口大小
 *         Emm_V5_Modify_Pos_Window
 */
void Emm_V5_Modify_Pos_Window(uint8_t addr, bool svF, uint16_t prw)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0xD1;                        //功能码
  cmd[2] =  0x07;                        //数据
  cmd[3] =  svF;                         //是否存储标志,false为不存储,true为存储
  cmd[4] =  (uint8_t)(prw >> 8);				 //位置到达窗口高8位,默认值8即0.8度
	cmd[5] =  (uint8_t)(prw >> 0);          //位置到达窗口低8位
  cmd[6] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 7);
}

/**
 * @brief 读取过热过流保护阈值(Y42)
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 读取驱动器过热过流保护配置
 *         Emm_V5_Read_Otocp
 */
void Emm_V5_Read_Otocp(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x13;                        //功能码
  cmd[2] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 3);
}

/**
 * @brief 修改过热过流保护阈值(Y42)
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param otp 过热保护阈值 默认100度
 * @param ocp 过流保护阈值 默认6600mA
 * @param time_ms 过热保护时间 默认1000ms
 * @return 地址+功能码+运行状态+校验字节
 * @note 配置过热过流保护参数
 *         Emm_V5_Modify_Otocp
 */
void Emm_V5_Modify_Otocp(uint8_t addr, bool svF, uint16_t otp, uint16_t ocp, uint16_t time_ms)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0]  =  addr;                       //地址
  cmd[1]  =  0xD3;                       //功能码
  cmd[2]  =  0x56;                       //数据
  cmd[3]  =  svF;                        //是否存储标志,false为不存储,true为存储
  cmd[4]  =  (uint8_t)(otp >> 8);				 //过热保护阈值高8位
	cmd[5]  =  (uint8_t)(otp >> 0);         //过热保护阈值低8位
	cmd[6]  =  (uint8_t)(ocp >> 8);				 //过流保护阈值高8位
	cmd[7]  =  (uint8_t)(ocp >> 0);         //过流保护阈值低8位
	cmd[8]  =  (uint8_t)(time_ms >> 8);		 //过热保护时间高8位
	cmd[9]  =  (uint8_t)(time_ms >> 0);     //过热保护时间低8位
  cmd[10] =  0x6B;                       //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 11);
}

/**
 * @brief 读取心跳保护时间(Y42)
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 读取驱动器心跳保护时间配置
 *         Emm_V5_Read_Heart_Protect
 */
void Emm_V5_Read_Heart_Protect(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x16;                        //功能码
  cmd[2] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 3);
}

/**
 * @brief 修改心跳保护时间(Y42)
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param hp 心跳保护时间,单位ms
 * @return 地址+功能码+运行状态+校验字节
 * @note 配置心跳保护超时时间
 *         Emm_V5_Modify_Heart_Protect
 */
void Emm_V5_Modify_Heart_Protect(uint8_t addr, bool svF, uint32_t hp)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0]  =  addr;                       //地址
  cmd[1]  =  0x68;                       //功能码
  cmd[2]  =  0x38;                       //数据
  cmd[3]  =  svF;                        //是否存储标志,false为不存储,true为存储
  cmd[4]  =  (uint8_t)(hp >> 24);				 //心跳保护时间(ms)(bit24-bit31)
	cmd[5]  =  (uint8_t)(hp >> 16);
	cmd[6]  =  (uint8_t)(hp >> 8);
	cmd[7]  =  (uint8_t)(hp >> 0);          //心跳保护时间(ms)(bit0-bit7)
  cmd[8]  =  0x6B;                       //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 9);
}

/**
 * @brief 读取积分限幅/微分系数(Y42)
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 读取驱动器积分限幅和微分系数
 *         Emm_V5_Read_Integral_Limit
 */
void Emm_V5_Read_Integral_Limit(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x23;                        //功能码
  cmd[2] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 3);
}

/**
 * @brief 修改积分限幅/微分系数(Y42)
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param il 积分限幅值 默认值为65535
 * @return 地址+功能码+运行状态+校验字节
 * @note 配置积分限幅或微分系数
 *         Emm_V5_Modify_Integral_Limit
 */
void Emm_V5_Modify_Integral_Limit(uint8_t addr, bool svF, uint32_t il)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0]  =  addr;                       //地址
  cmd[1]  =  0x4B;                       //功能码
  cmd[2]  =  0x57;                       //数据
  cmd[3]  =  svF;                        //是否存储标志,false为不存储,true为存储
  cmd[4]  =  (uint8_t)(il >> 24);				 //积分限幅值(bit24-bit31)
	cmd[5]  =  (uint8_t)(il >> 16);
	cmd[6]  =  (uint8_t)(il >> 8);
	cmd[7]  =  (uint8_t)(il >> 0);          //积分限幅值(bit0-bit7)
  cmd[8]  =  0x6B;                       //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 9);
}

//获取系统状态信息函数组
/**
 * @brief 读取系统状态参数
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 读取驱动器系统状态
 *         Emm_V5_Read_System_State_Params
 */
void Emm_V5_Read_System_State_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x43;                        //功能码
	cmd[2] =  0x7A;                        //数据
  cmd[3] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 4);
}

/**
 * @brief 读取电机配置参数
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 读取驱动器配置参数
 *         Emm_V5_Read_Motor_Conf_Params
 */
void Emm_V5_Read_Motor_Conf_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};     //指令缓冲区
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x42;                        //功能码
	cmd[2] =  0x6C;                        //数据
  cmd[3] =  0x6B;                        //校验字节
  
  //发送指令
  HAL_UART_Transmit_DMA(g_emm_uart, (uint8_t *)cmd, 4);
}

//多机指令(多机指令将指令缓存后统一发送)
//编码器校准函数组
/**
 * @brief 触发编码器校准 - 缓存到多机指令
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 触发编码器校准并缓存到MMCL缓冲区
 *         Emm_V5_MMCL_Trig_Encoder_Cal
 */
void Emm_V5_MMCL_Trig_Encoder_Cal(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x06;                        //功能码
  cmd[2] =  0x45;                        //数据
  cmd[3] =  0x6B;                        //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
 * @brief 复位电机(Y42) - 缓存到多机指令
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 复位电机并缓存到MMCL缓冲区
 *         Emm_V5_MMCL_Reset_Motor
 */
void Emm_V5_MMCL_Reset_Motor(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x08;                        //功能码
  cmd[2] =  0x97;                        //数据
  cmd[3] =  0x6B;                        //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
 * @brief 清除当前位置归零 - 缓存到多机指令
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 清除当前位置归零并缓存到MMCL缓冲区
 *         Emm_V5_MMCL_Reset_CurPos_To_Zero
 */
void Emm_V5_MMCL_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x0A;                        //功能码
  cmd[2] =  0x6D;                        //数据
  cmd[3] =  0x6B;                        //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
 * @brief 清除堵转保护 - 缓存到多机指令
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 清除堵转保护并缓存到MMCL缓冲区
 *         Emm_V5_MMCL_Reset_Clog_Pro
 */
void Emm_V5_MMCL_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x0E;                        //功能码
  cmd[2] =  0x52;                        //数据
  cmd[3] =  0x6B;                        //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
 * @brief 恢复出厂设置 - 缓存到多机指令
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 恢复出厂设置并缓存到MMCL缓冲区
 *         Emm_V5_MMCL_Restore_Motor
 */
void Emm_V5_MMCL_Restore_Motor(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x0F;                        //功能码
  cmd[2] =  0x5F;                        //数据
  cmd[3] =  0x6B;                        //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

//运动控制函数组
/**
 * @brief 使能信号控制 - 缓存到多机指令
 * @param addr 驱动器地址
 * @param state 使能状态 true为使能电机,false为关闭电机
 * @param snF 多机同步标志 false为不启用,true为启用
 * @return 地址+功能码+运行状态+校验字节
 * @note 控制电机使能/关闭并缓存到MMCL缓冲区
 *         Emm_V5_MMCL_En_Control
 */
void Emm_V5_MMCL_En_Control(uint8_t addr, bool state, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0xF3;                        //功能码
  cmd[2] =  0xAB;                        //数据
  cmd[3] =  (uint8_t)state;              //使能状态
  cmd[4] =  snF;                         //多机同步运动标志
  cmd[5] =  0x6B;                        //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < 6; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
 * @brief 速度模式 - 缓存到多机指令
 * @param addr 驱动器地址
 * @param dir 方向 0为CW,非0值为CCW
 * @param vel 速度 范围0-5000RPM
 * @param acc 加速度 范围0-255,注意0为直接启动
 * @param snF 多机同步标志 false为不启用,true为启用
 * @return 地址+功能码+运行状态+校验字节
 * @note 速度模式控制并缓存到MMCL缓冲区
 *         Emm_V5_MMCL_Vel_Control
 */
void Emm_V5_MMCL_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};

  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0xF6;                        //功能码
  cmd[2] =  dir;                         //方向
  cmd[3] =  (uint8_t)(vel >> 8);         //速度(RPM)高8位
  cmd[4] =  (uint8_t)(vel >> 0);         //速度(RPM)低8位
  cmd[5] =  acc;                         //加速度,注意0为直接启动
  cmd[6] =  snF;                         //多机同步运动标志
  cmd[7] =  0x6B;                        //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < 8; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
 * @brief 位置模式 - 缓存到多机指令
 * @param addr 驱动器地址
 * @param dir 方向 0为CW,非0值为CCW
 * @param vel 速度(RPM) 范围0-5000RPM
 * @param acc 加速度 范围0-255,注意0为直接启动
 * @param clk 脉冲数 范围0-(2^32-1)
 * @param raF 运动标志 0为以当前位置到目标位置运动,1为增量运动,2为相对当前实时位置运动
 * @param snF 多机同步标志 false为不启用,true为启用
 * @return 地址+功能码+运行状态+校验字节
 * @note 位置模式控制并缓存到MMCL缓冲区
 *         Emm_V5_MMCL_Pos_Control
 */
void Emm_V5_MMCL_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};

  //组装指令帧
  cmd[0]  =  addr;                       //地址
  cmd[1]  =  0xFD;                       //功能码
  cmd[2]  =  dir;                        //方向
  cmd[3]  =  (uint8_t)(vel >> 8);        //速度(RPM)高8位
  cmd[4]  =  (uint8_t)(vel >> 0);        //速度(RPM)低8位
  cmd[5]  =  acc;                        //加速度,注意0为直接启动
  cmd[6]  =  (uint8_t)(clk >> 24);       //脉冲数(bit24-bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);       //脉冲数(bit16-bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);        //脉冲数(bit8-bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);        //脉冲数(bit0-bit7)
  cmd[10] =  raF;                        //绝对/相对标志,false为绝对运动,true为增量运动
  cmd[11] =  snF;                        //多机同步运动标志,false为不启用,true为启用
  cmd[12] =  0x6B;                       //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < 13; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
 * @brief 设置快速位置模式运动参数
 * @param addr 驱动器地址
 * @param vel 速度(RPM) 范围0-5000RPM
 * @param acc 加速度 范围0-255,注意0为直接启动
 * @param raF 运动标志 0为以当前位置到目标位置运动,1为增量运动,2为相对当前实时位置运动
 * @param snF 多机同步标志 false为不启用,true为启用
 * @return 地址+功能码+运行状态+校验字节
 * @note 设置快速位置模式运动参数并缓存到MMCL缓冲区
 *         Emm_V5_MMCL_Set_QPos_Params
 */
void Emm_V5_MMCL_Set_QPos_Params(uint8_t addr, uint16_t vel, uint8_t acc, uint8_t raF, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};

  //组装指令帧
  cmd[0]  =  addr;                       //地址
  cmd[1]  =  0xF1;                       //功能码
  cmd[2]  =  (uint8_t)(vel >> 8);        //速度(RPM)高8位
  cmd[3]  =  (uint8_t)(vel >> 0);        //速度(RPM)低8位
  cmd[4]  =  acc;                        //加速度,注意0为直接启动
  cmd[5] =  raF;                         //绝对/相对标志,false为绝对运动,true为增量运动
  cmd[6] =  snF;                         //多机同步运动标志,false为不启用,true为启用
  cmd[7] =  0x6B;                        //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < 8; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
 * @brief 快速位置模式
 * @param addr 驱动器地址
 * @param clk 脉冲数(有符号) 默认16细分,+3200正转一圈,-3200反转一圈
 * @return 地址+功能码+运行状态+校验字节
 * @note 快速位置模式控制并缓存到MMCL缓冲区
 *         Emm_V5_MMCL_QPos_Control
 */
void Emm_V5_MMCL_QPos_Control(uint8_t addr, int32_t clk)
{
  uint8_t j = 0, cmd[16] = {0};

  //组装指令帧
  cmd[0]  =  addr;                       //地址
  cmd[1]  =  0xFC;                       //功能码
  cmd[2]  =  (uint8_t)(clk >> 24);       //脉冲数(bit24-bit31)
  cmd[3]  =  (uint8_t)(clk >> 16);       //脉冲数(bit16-bit23)
  cmd[4]  =  (uint8_t)(clk >> 8);        //脉冲数(bit8-bit15)
  cmd[5]  =  (uint8_t)(clk >> 0);        //脉冲数(bit0-bit7)
  cmd[6] =  0x6B;                        //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < 7; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
 * @brief 紧急停止 - 缓存到多机指令
 * @param addr 驱动器地址
 * @param snF 多机同步标志 false为不启用,true为启用
 * @return 地址+功能码+运行状态+校验字节
 * @note 紧急停止并缓存到MMCL缓冲区
 *         Emm_V5_MMCL_Stop_Now
 */
void Emm_V5_MMCL_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0xFE;                        //功能码
  cmd[2] =  0x98;                        //数据
  cmd[3] =  snF;                         //多机同步运动标志
  cmd[4] =  0x6B;                        //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < 5; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
 * @brief 多机同步运动 - 缓存到多机指令
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 触发多机同步运动
 *         Emm_V5_MMCL_Synchronous_motion
 */
void Emm_V5_MMCL_Synchronous_motion(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0xFF;                        //功能码
  cmd[2] =  0x66;                        //数据
  cmd[3] =  0x6B;                        //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

//原点回归函数组
/**
 * @brief 设置当前圈数为原点 - 缓存到多机指令
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @return 地址+功能码+运行状态+校验字节
 * @note 设置当前圈数为原点并缓存到MMCL缓冲区
 *         Emm_V5_MMCL_Origin_Set_O
 */
void Emm_V5_MMCL_Origin_Set_O(uint8_t addr, bool svF)
{
  uint8_t j = 0, cmd[16] = {0};
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x93;                        //功能码
  cmd[2] =  0x88;                        //数据
  cmd[3] =  svF;                         //是否存储标志,false为不存储,true为存储
  cmd[4] =  0x6B;                        //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < 5; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
 * @brief 触发原点回归 - 缓存到多机指令
 * @param addr 驱动器地址
 * @param o_mode 回归模式 0为单圈就近找零,1为单圈负方向找零,2为单圈正限位碰撞找零,3为单圈负限位回归
 * @param snF 多机同步标志 false为不启用,true为启用
 * @return 地址+功能码+运行状态+校验字节
 * @note 触发原点回归并缓存到MMCL缓冲区
 *         Emm_V5_MMCL_Origin_Trigger_Return
 */
void Emm_V5_MMCL_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x9A;                        //功能码
  cmd[2] =  o_mode;                      //回归模式,0为单圈就近找零,1为单圈负方向找零,2为单圈正限位碰撞找零,3为单圈负限位回归
  cmd[3] =  snF;                         //多机同步运动标志,false为不启用,true为启用
  cmd[4] =  0x6B;                        //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < 5; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
 * @brief 强制中断并退出回归 - 缓存到多机指令
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 强制中断原点回归并缓存到MMCL缓冲区
 *         Emm_V5_MMCL_Origin_Interrupt
 */
void Emm_V5_MMCL_Origin_Interrupt(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x9C;                        //功能码
  cmd[2] =  0x48;                        //数据
  cmd[3] =  0x6B;                        //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
 * @brief 修改回归参数 - 缓存到多机指令
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param o_mode 回归模式 0为单圈就近找零,1为单圈负方向找零,2为单圈正限位碰撞找零,3为单圈负限位回归
 * @param o_dir 找零方向 0为CW,非0值为CCW
 * @param o_vel 回归速度,单位RPM(转/分钟)
 * @param o_tm 回归超时时间,单位毫秒
 * @param sl_vel 正限位碰撞低速转速,单位RPM(转/分钟)
 * @param sl_ma 正限位碰撞扭矩,单位Ma(电流)
 * @param sl_ms 正限位碰撞时间,单位Ms(毫秒)
 * @param potF 上电自动找零点 false为不启用,true为启用
 * @return 地址+功能码+运行状态+校验字节
 * @note 修改原点回归参数并缓存到MMCL缓冲区
 *         Emm_V5_MMCL_Origin_Modify_Params
 */
void Emm_V5_MMCL_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t j = 0, cmd[32] = {0};
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x4C;                        //功能码
  cmd[2] =  0xAE;                        //数据
  cmd[3] =  svF;                         //是否存储标志,false为不存储,true为存储
  cmd[4] =  o_mode;                      //回归模式,0为单圈就近找零,1为单圈负方向找零,2为单圈正限位碰撞找零,3为单圈负限位回归
  cmd[5] =  o_dir;                       //找零方向
  cmd[6]  =  (uint8_t)(o_vel >> 8);      //回归速度(RPM)高8位
  cmd[7]  =  (uint8_t)(o_vel >> 0);      //回归速度(RPM)低8位
  cmd[8]  =  (uint8_t)(o_tm >> 24);      //回归超时时间(bit24-bit31)
  cmd[9]  =  (uint8_t)(o_tm >> 16);      //回归超时时间(bit16-bit23)
  cmd[10] =  (uint8_t)(o_tm >> 8);       //回归超时时间(bit8-bit15)
  cmd[11] =  (uint8_t)(o_tm >> 0);       //回归超时时间(bit0-bit7)
  cmd[12] =  (uint8_t)(sl_vel >> 8);     //正限位碰撞低速转速(RPM)高8位
  cmd[13] =  (uint8_t)(sl_vel >> 0);     //正限位碰撞低速转速(RPM)低8位
  cmd[14] =  (uint8_t)(sl_ma >> 8);      //正限位碰撞扭矩(Ma)高8位
  cmd[15] =  (uint8_t)(sl_ma >> 0);      //正限位碰撞扭矩(Ma)低8位
  cmd[16] =  (uint8_t)(sl_ms >> 8);      //正限位碰撞时间(Ms)高8位
  cmd[17] =  (uint8_t)(sl_ms >> 0);      //正限位碰撞时间(Ms)低8位
  cmd[18] =  potF;                       //上电自动找零点,false为不启用,true为启用
  cmd[19] =  0x6B;                       //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < 20; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
 * @brief 读取碰撞回归返回角度(X42S/Y42) - 缓存到多机指令
 * @param addr 驱动器地址
 * @return 地址+功能码+运行状态+校验字节
 * @note 读取碰撞回归返回角度并缓存到MMCL缓冲区
 *         X_V2_MMCL_Origin_Read_SL_RP
 */
void X_V2_MMCL_Origin_Read_SL_RP(uint8_t addr)
{
  uint8_t j = 0; __IO static uint8_t cmd[16] = {0};
  
  //组装指令帧
  cmd[0] =  addr;                        //地址
  cmd[1] =  0x3F;                        //功能码
  cmd[2] =  0x6B;                        //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < 3; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
 * @brief 修改碰撞回归返回角度(X42S/Y42) - 缓存到多机指令
 * @param addr 驱动器地址
 * @param svF 是否存储标志 false为不存储,true为存储
 * @param sl_rp 碰撞回归返回角度,单位0.1度,写入40即4.0度
 * @return 地址+功能码+运行状态+校验字节
 * @note 修改碰撞回归返回角度并缓存到MMCL缓冲区
 *         X_V2_MMCL_Origin_Modify_SL_RP
 */
void X_V2_MMCL_Origin_Modify_SL_RP(uint8_t addr, bool svF, uint16_t sl_rp)
{
  uint8_t j = 0; __IO static uint8_t cmd[16] = {0};
  
  //组装指令帧
  cmd[0]  =  addr;                       //地址
  cmd[1]  =  0x5C;                       //功能码
  cmd[2]  =  0xAC;                       //数据
  cmd[3]  =  svF;                        //是否存储标志,false为不存储,true为存储
  cmd[4]  =  (uint8_t)(sl_rp >> 8);			 //碰撞回归返回角度高8位,单位0.1度
	cmd[5]  =  (uint8_t)(sl_rp >> 0);       //碰撞回归返回角度低8位
  cmd[6]  =  0x6B;                       //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < 7; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

//获取系统参数函数组
/**
 * @brief 定时返回系统信息(Y42)
 * @param addr 驱动器地址
 * @param s 系统参数选择
 * @param time_ms 定时时间
 * @return 地址+功能码+运行状态+校验字节
 * @note 定时返回系统参数并缓存到MMCL缓冲区
 *         Emm_V5_MMCL_Auto_Return_Sys_Params_Timed
 */
void Emm_V5_MMCL_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms)
{
  uint8_t i = 0, j = 0; uint8_t cmd[16] = {0};
  
  //组装指令帧
  cmd[i] = addr; ++i;                    //地址

  cmd[i] = 0x11; ++i;                    //功能码

  cmd[i] = 0x18; ++i;                    //数据

  switch(s)                              //信息类型
  {
    case S_VBUS : cmd[i] = 0x24; ++i; break;	//读取总线电压
		case S_CBUS : cmd[i] = 0x26; ++i; break;	//读取总线电流
    case S_CPHA : cmd[i] = 0x27; ++i; break;	//读取相电流
		case S_ENCO : cmd[i] = 0x29; ++i; break;	//读取编码器原始值
		case S_CLKC : cmd[i] = 0x30; ++i; break;	//读取实时脉冲数
    case S_ENCL : cmd[i] = 0x31; ++i; break;	//读取编码器线性化校准后的计数
		case S_CLKI : cmd[i] = 0x32; ++i; break;	//读取脉冲输入频率
    case S_TPOS : cmd[i] = 0x33; ++i; break;	//读取电机目标位置
    case S_SPOS : cmd[i] = 0x34; ++i; break;	//读取电机实时设定目标位置
		case S_VEL  : cmd[i] = 0x35; ++i; break;	//读取电机实时转速
    case S_CPOS : cmd[i] = 0x36; ++i; break;	//读取电机实时位置
    case S_PERR : cmd[i] = 0x37; ++i; break;	//读取电机位置误差
		case S_VBAT : cmd[i] = 0x38; ++i; break;	//读取线圈备用电池电压(Y42)
		case S_TEMP : cmd[i] = 0x39; ++i; break;	//读取电机实时温度(Y42)
    case S_FLAG : cmd[i] = 0x3A; ++i; break;	//读取电机状态标志位
    case S_OFLAG: cmd[i] = 0x3B; ++i; break;	//读取驱动状态标志位
		case S_OAF  : cmd[i] = 0x3C; ++i; break;	//读取电机状态+驱动状态(Y42)
		case S_PIN  : cmd[i] = 0x3D; ++i; break;	//读取引脚状态(Y42)
    default: break;
  }
	
	cmd[i] = (uint8_t)(time_ms >> 8);  ++i;	//定时时间高8位
	cmd[i] = (uint8_t)(time_ms >> 0);  ++i; //定时时间低8位

  cmd[i] = 0x6B; ++i;                   	//校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < i; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/**
 * @brief 读取系统参数 - 缓存到多机指令
 * @param addr 驱动器地址
 * @param s 系统参数选择
 * @return 地址+功能码+运行状态+校验字节
 * @note 读取系统参数并缓存到MMCL缓冲区
 *         Emm_V5_MMCL_Read_Sys_Params
 */
void Emm_V5_MMCL_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0, j = 0; uint8_t cmd[16] = {0};
  
  //组装指令帧
  cmd[i] = addr; ++i;                    //地址

  switch(s)                              //功能码
  {
    case S_VBUS : cmd[i] = 0x24; ++i; break;	//读取总线电压
		case S_CBUS : cmd[i] = 0x26; ++i; break;	//读取总线电流
    case S_CPHA : cmd[i] = 0x27; ++i; break;	//读取相电流
		case S_ENCO : cmd[i] = 0x29; ++i; break;	//读取编码器原始值
		case S_CLKC : cmd[i] = 0x30; ++i; break;	//读取实时脉冲数
    case S_ENCL : cmd[i] = 0x31; ++i; break;	//读取编码器线性化校准后的计数
		case S_CLKI : cmd[i] = 0x32; ++i; break;	//读取脉冲输入频率
    case S_TPOS : cmd[i] = 0x33; ++i; break;	//读取电机目标位置
    case S_SPOS : cmd[i] = 0x34; ++i; break;	//读取电机实时设定目标位置
		case S_VEL  : cmd[i] = 0x35; ++i; break;	//读取电机实时转速
    case S_CPOS : cmd[i] = 0x36; ++i; break;	//读取电机实时位置
    case S_PERR : cmd[i] = 0x37; ++i; break;	//读取电机位置误差
		case S_VBAT : cmd[i] = 0x38; ++i; break;	//读取线圈备用电池电压(Y42)
		case S_TEMP : cmd[i] = 0x39; ++i; break;	//读取电机实时温度(Y42)
    case S_FLAG : cmd[i] = 0x3A; ++i; break;	//读取电机状态标志位
    case S_OFLAG: cmd[i] = 0x3B; ++i; break;	//读取驱动状态标志位
		case S_OAF  : cmd[i] = 0x3C; ++i; break;	//读取电机状态+驱动状态(Y42)
		case S_PIN  : cmd[i] = 0x3D; ++i; break;	//读取引脚状态(Y42)
    default: break;
  }

  cmd[i] = 0x6B; ++i;                    //校验字节
  
  //缓存当前指令到MMCL缓冲区
  for(j=0; j < i; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}


