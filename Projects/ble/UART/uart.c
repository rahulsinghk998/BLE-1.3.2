#include <iocc2540.h>
#include "uart.h"

void Uart_Init(void)
{
  CLKCONCMD &= ~(1<<6); // Select system clock to 32M XOSC
  while (CLKCONSTA & (1<<6)) ; 
  CLKCONCMD &= ~((1<<6)|(7<<0)); // Select system clock to 32M XOSC and set speed to 32M
  PERCFG = 0x00; //Configure USART0 for Alternative 1 => Port P0 (PERCFG.U0CFG = 0).
  P0SEL |= (0xf<<2); // set pin 2 3 4 5 to peripheral
  P2DIR &= ~(3<<6); // Set port 0 priority 1st priority: USART 0 2nd priority: USART 1 3rd priority: Timer 1

  U0CSR |= 1<<7; //UART0 set to UART mode
  U0GCR = 9; // Set BAUD_E to 1001
  U0BAUD = 59; // Set BAUD_M to 0011 1011, refer to table 17-1
  UTX0IF = 0; // Clear USART 0 TX interrupt flag
  U0CSR |= 1<<6; // Enable UART0 receiver
}

void Uart_Print(char *p, int len)
{
  unsigned int i;
  for (i = 0; i < len; i++) {
    U0DBUF = *p++; // write data to buffer for sending
    while (!UTX0IF); // wait for send complete
    UTX0IF = 0; // 0: Interrupt not pending
  }
  U0DBUF = 0x0A; // carriage return
  while (!UTX0IF) ;
  UTX0IF = 0;
}