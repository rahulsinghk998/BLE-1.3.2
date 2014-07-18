#ifndef __UART_H__
#define __UART_H__

#include <iocc2540.h>
#include <ioCC254x_bitdef.h>

extern void Uart_Init(unsigned int baudrate);
extern void Uart_Print(char *p, int len);
#endif