//UART通信协议解析,支持多种校验
#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stdint.h>

#define PROTOCOL_MAX_PAYLOAD 64   //最大载荷长度,bytes

//协议解析状态枚举
typedef enum {
    PARSE_IDLE,       //空闲,等待帧头
    PARSE_HEADER,     //收到帧头
    PARSE_CMD,        //收到命令字
    PARSE_LEN,        //收到长度
    PARSE_DATA,       //正在接收数据载荷
    PARSE_CHECKSUM,   //收到校验字节
    PARSE_COMPLETE    //一帧解析完成
} ParseState_t;

//校验类型枚举
typedef enum {
    CHECK_NONE,     //无校验
    CHECK_XOR,      //异或校验
    CHECK_CRC8,     //CRC8校验
    CHECK_CRC16     //CRC16校验
} CheckType_t;

//协议配置参数
typedef struct {
    uint8_t  head_byte;      //帧头字节
    uint8_t  tail_byte;      //帧尾字节
    CheckType_t check_type;  //校验类型
    uint16_t timeout_ms;     //帧超时时间,ms
} ProtocolConfig_t;

//解析完成后的数据包
typedef struct {
    uint8_t  cmd;                               //命令字
    uint8_t  payload[PROTOCOL_MAX_PAYLOAD];     //数据载荷
    uint16_t payload_len;                       //载荷实际长度
    uint8_t  checksum;                          //接收到的校验值
    uint8_t  valid;                             //有效性标志
} Packet_t;

//协议解析器运行时状态
typedef struct {
    ProtocolConfig_t config;       //协议配置
    ParseState_t state;            //当前解析状态
    Packet_t     packet;           //正在组装的包
    uint16_t data_index;           //当前数据索引
    uint8_t  calc_checksum;        //本地计算的校验值
    uint32_t last_byte_time;       //上次收到字节时间戳
    uint32_t (*GetTick)(void);     //获取时间戳的函数指针
} Protocol_t;

void    Protocol_Init(Protocol_t *p, const ProtocolConfig_t *cfg,
                      uint32_t (*get_tick)(void));              //初始化协议解析器
uint8_t Protocol_ParseByte(Protocol_t *p, uint8_t byte);        //输入一个字节进行解析
uint8_t Protocol_GetPacket(Protocol_t *p, Packet_t *out);       //获取已解析完成的数据包
void    Protocol_Reset(Protocol_t *p);                           //重置解析器状态

#endif
