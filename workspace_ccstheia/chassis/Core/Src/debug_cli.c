//调试命令行,通过VCOM串口接收ASCII指令控制云台
#include "debug_cli.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "ti_msp_dl_config.h"

#define CLI_BUF_SIZE  32                     //命令行输入缓冲区大小

static char     g_linebuf[CLI_BUF_SIZE];     //行缓冲区
static uint8_t  g_idx = 0;                   //缓冲区写入位置

/**
 * @brief  通过UART0发送一个字符(轮询阻塞)
 * @param  c  待发送字符
 * @note   while (!DL_UART_isBusy)确保前一个字节已移出移位寄存器
 *       调用者:
 *       cli_puts,putint,putfloat,newline,DebugCLI_Poll
 */
void CLI_PutChar(char c)
{
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t)c);
    while (DL_UART_Main_isBusy(UART_0_INST));
}

static void putch(char c)
{
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t)c);
    while (DL_UART_Main_isBusy(UART_0_INST));
}

/**
 * @brief  发送字符串
 * @param  s  以\0结尾的字符串
 * @note   逐个字符调用putch,直到遇到\0
 *       调用者:
 *       putint,exec_cmd,DebugCLI_PrintHelp
 */
static void cli_puts(const char *s)
{
    while (*s) putch(*s++);
}

/**
 * @brief  发送整数(十进制)
 * @param  v  待发送的32位有符号整数
 * @note   调用者:putfloat,exec_cmd
 *         先用sprintf转字符串再调用cli_puts
 */
static void putint(int32_t v)
{
    char buf[16];
    sprintf(buf, "%ld", (long)v);
    cli_puts(buf);
}

/**
 * @brief  发送浮点数(1位小数)
 * @param  v  待发送浮点数
 * @note   拆分为整数和小数部分分别发送,小数取绝对值防负数显示"--"
 *       调用者:
 *       exec_cmd
 */
static void putfloat(float v)
{
    int32_t whole = (int32_t)v;
    int32_t frac  = (int32_t)((v - (float)whole) * 10.0f);
    if (frac < 0) frac = -frac;
    putint(whole);
    putch('.');
    putint(frac);
}

/**
 * @brief  回车换行
 * @note   发送\r\n到VCOM
 *       调用者:
 *       exec_cmd,DebugCLI_Poll,DebugCLI_PrintHelp
 */
static void newline(void)
{
    putch('\r');
    putch('\n');
}

/**
 * @brief  解析并执行一条命令
 * @param  line  以\0结尾的命令行(不含\r\n)
 * @note   支持 help/?/数字(绝对角度),底盘命令已移除(由B21按钮调试)
 *       调用者:
 *       DebugCLI_Poll
 */
static void exec_cmd(const char *line)
{
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return;

    //单轴角度命令(如"60"):两个云台都转到该角度,直接裸发"thetaL,thetaR"到云台口
    //与for循环同底层(DL_UART_Main_transmitData(CLI_UART_INST)),无中间层
    if (*line >= '0' && *line <= '9') {
        char buf[24];
        int n = sprintf(buf, "%s,%s\r\n", line, line);
        for (int i = 0; i < n; i++) {
            DL_UART_Main_transmitData(CLI_UART_INST, (uint8_t)buf[i]);
            while (DL_UART_Main_isBusy(CLI_UART_INST));
        }
        return;
    }

    cli_puts("? "); cli_puts(line); newline();
}

/**
 * @brief  轮询UART0收字节,缓冲并解析命令
 * @note   RX FIFO非空时逐个读取字节,\r\n触发行解析,退格键支持行编辑
 *       调用者:
 *       main(empty_cpp.cpp)
 */
void DebugCLI_Poll(void)
{
    //转发UART1(STM32发来的)到UART0(VCOM)
    while (!DL_UART_isRXFIFOEmpty(CLI_UART_INST)) {
        uint8_t byte = DL_UART_Main_receiveData(CLI_UART_INST);
        DL_UART_Main_transmitData(UART_0_INST, byte);
        while (DL_UART_Main_isBusy(UART_0_INST));
    }

    while (!DL_UART_isRXFIFOEmpty(UART_0_INST)) {
        uint8_t byte = DL_UART_Main_receiveData(UART_0_INST);

        if (byte == '\r' || byte == '\n') {
            newline();
            g_linebuf[g_idx] = '\0';
            g_linebuf[CLI_BUF_SIZE - 1] = '\0';  //长度保护,防越界
            exec_cmd(g_linebuf);                  //单轴角度(如"60")或"thetaL,thetaR"双轴,直接裸发到云台口
            g_idx = 0;
            cli_puts("> ");
        } else if (byte == '\b' || byte == 0x7F) {
            if (g_idx > 0) {
                g_idx--;
                putch('\b');
                putch(' ');
                putch('\b');
            }
        } else if (byte >= ' ' && byte <= '~') {
            if (g_idx < CLI_BUF_SIZE - 1) {
                g_linebuf[g_idx++] = (char)byte;
                putch((char)byte);
            }
        }
    }
}

/**
 * @brief  打印帮助信息
 * @note   仅剩角度命令,底盘调试由B21按钮完成
 *       调用者:
 *       exec_cmd
 */
void DebugCLI_PrintHelp(void)
{
    newline();
    cli_puts("commands:");
    newline();
    cli_puts("90       goto absolute 90 deg (yaw only)");
    newline();
    cli_puts("90,30    goto yaw=90 pitch=30 (dual axis)");
    newline();
    cli_puts("?        query position");
    newline();
    cli_puts("help     this help");
    newline();
    cli_puts("L        lock yaw (compensate chassis rotation)");
    newline();
    cli_puts("U        unlock yaw");
    newline();
    cli_puts("> ");
}
