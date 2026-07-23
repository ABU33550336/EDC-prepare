//调试用ASCII命令行,通过VCOM(UART0)交互式控制云台
#ifndef DEBUG_CLI_H
#define DEBUG_CLI_H

#include <stdbool.h>

void DebugCLI_Poll(void);
void DebugCLI_PrintHelp(void);
void CLI_PutChar(char c);

#endif
