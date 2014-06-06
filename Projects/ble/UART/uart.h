#ifndef __UART_H__
#define __UART_H__
extern void Uart_Init(unsigned int baudrate);
extern void Uart_Print(char *p, int len);
#endif