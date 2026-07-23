//串口协议解析模块,基于状态机逐字节解析数据帧

#include "uart_protocol.h"
#include <stddef.h>

/**
 * @brief CRC8单字节更新(多项式0x31)
 * @param crc 当前CRC值
 * @param data 输入字节
 * @return 更新后的CRC8
 * @note 标准CRC8-CCITT算法,逐位计算而非查表
 *        调用者:Protocol_ParseByte
 */
static uint8_t crc8_update(uint8_t crc, uint8_t data)
{
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (uint8_t)((crc << 1) ^ 0x31);                               //多项式0x31(左移形式)
        } else {
            crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/**
 * @brief 初始化协议解析器
 * @param p 协议解析器指针
 * @param cfg 协议配置(帧头/帧尾/校验类型/超时)
 * @param get_tick 获取系统滴答函数(用于超时检测)
 * @note 需保证get_tick在整个生命周期内有效
 *        调用者:
 */
void Protocol_Init(Protocol_t *p, const ProtocolConfig_t *cfg,
                   uint32_t (*get_tick)(void))
{
    if (p == NULL || cfg == NULL || get_tick == NULL) return;
    p->config          = *cfg;
    p->GetTick         = get_tick;
    p->state           = PARSE_IDLE;
    p->last_byte_time  = get_tick();                                           //记录当前时间用于首次超时判断
    Protocol_Reset(p);
}

/**
 * @brief 逐字节解析协议帧(状态机驱动)
 * @param p 协议解析器指针
 * @param byte 输入字节
 * @return 1=收到完整有效帧,0=仍在解析或帧无效
 * @note 状态转换:IDLE->HEADER->CMD->LEN->CHECKSUM->COMPLETE;超时自动复位
 *        调用者:
 */
uint8_t Protocol_ParseByte(Protocol_t *p, uint8_t byte)
{
    if (p == NULL || p->GetTick == NULL) return 0;

    uint32_t now = p->GetTick();

    //非空闲状态下检测字节间超时
    if (p->state != PARSE_IDLE) {
        uint32_t elapsed = now - p->last_byte_time;
        if (p->config.timeout_ms > 0 && elapsed >= p->config.timeout_ms) {
            Protocol_Reset(p);                                                //超时,复位状态机等待新帧
            return 0;
        }
    }
    p->last_byte_time = now;

    switch (p->state) {
    case PARSE_IDLE:
        //等待帧头字节
        if (byte == p->config.head_byte) p->state = PARSE_HEADER;
        break;

    case PARSE_HEADER:
        //接收命令字,初始化校验和
        p->packet.cmd = byte;
        if (p->config.check_type == CHECK_XOR) {
            p->calc_checksum = byte;                                          //XOR校验:初始值为首个校验字节
        } else if (p->config.check_type == CHECK_CRC8) {
            p->calc_checksum = crc8_update(0xFF, byte);                       //CRC8:初始值0xFF后更新
        }
        p->state = PARSE_CMD;
        break;

    case PARSE_CMD:
        //接收负载长度,若长度为0则直接跳到校验段
        p->packet.payload_len = byte;
        p->data_index = 0;
        if (byte == 0) {
            p->state = PARSE_CHECKSUM;                                        //无负载,跳过数据段
        } else {
            p->state = PARSE_LEN;
        }
        //更新校验
        if (p->config.check_type == CHECK_XOR) {
            p->calc_checksum ^= byte;
        } else if (p->config.check_type == CHECK_CRC8) {
            p->calc_checksum = crc8_update(p->calc_checksum, byte);
        }
        break;

    case PARSE_LEN:
        //接收负载数据,超出最大长度时仍移指针但不存储(防溢出)
        if (p->data_index < PROTOCOL_MAX_PAYLOAD) {
            p->packet.payload[p->data_index] = byte;
        }
        p->data_index++;
        //更新校验
        if (p->config.check_type == CHECK_XOR) {
            p->calc_checksum ^= byte;
        } else if (p->config.check_type == CHECK_CRC8) {
            p->calc_checksum = crc8_update(p->calc_checksum, byte);
        }
        //负载接收完毕,切换到校验段
        if (p->data_index >= p->packet.payload_len) {
            p->state = PARSE_CHECKSUM;
        }
        break;

    case PARSE_DATA:
        //预留扩展状态
        break;

    case PARSE_CHECKSUM:
        //接收校验字节,标记帧完整
        p->packet.checksum = byte;
        p->state = PARSE_COMPLETE;
        break;

    case PARSE_COMPLETE:
        //等待帧尾,校验通过则返回有效标志
        if (byte == p->config.tail_byte) {
            uint8_t valid = 1;
            if (p->config.check_type == CHECK_XOR) {
                if (p->packet.checksum != p->calc_checksum) valid = 0;        //XOR校验不匹配
            } else if (p->config.check_type == CHECK_CRC8) {
                if (p->packet.checksum != p->calc_checksum) valid = 0;        //CRC8校验不匹配
            } else if (p->config.check_type == CHECK_CRC16) {
                valid = 0;                                                    //CRC16暂未实现
            }
            p->packet.valid = valid;
            Protocol_Reset(p);                                                //复位等待下一帧
            return valid;
        }
        //帧尾不匹配,丢弃整帧
        Protocol_Reset(p);
        return 0;
    }
    return 0;
}

/**
 * @brief 获取最近解析完成的包
 * @param p 协议解析器指针
 * @param out 输出包结构体
 * @return 0=成功,1=参数无效
 * @note 调用后包内容被复制到out,注意处理valid标志
 *        调用者:
 */
uint8_t Protocol_GetPacket(Protocol_t *p, Packet_t *out)
{
    if (p == NULL || out == NULL) return 1;
    *out = p->packet;
    return 0;
}

/**
 * @brief 复位协议解析器到初始状态
 * @param p 协议解析器指针
 * @note 丢弃当前正在解析的帧,清空所有中间变量
 *        调用者:Protocol_Init,Protocol_ParseByte
 */
void Protocol_Reset(Protocol_t *p)
{
    if (p == NULL) return;
    p->state      = PARSE_IDLE;
    p->data_index = 0;
    p->calc_checksum = 0;
    p->packet.cmd         = 0;
    p->packet.payload_len = 0;
    p->packet.checksum    = 0;
    p->packet.valid       = 0;
}
