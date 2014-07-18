#include "uart.h"

void Uart_Init(unsigned int baudrate)
{
  PERCFG = 0x00; //Configure USART0 for Alternative 1 => Port P0 (PERCFG.U0CFG = 0).
  P0SEL |= (0xf<<2); // set pin 2 3 4 5 to peripheral
  P2DIR &= ~(3<<6); // Set port 0 priority 1st priority: USART 0 2nd priority: USART 1 3rd priority: Timer 1

  U0CSR |= 1<<7; //UART0 set to UART mode
  switch ( baudrate ) {
    case 115200:
      U0GCR = 11;
      U0BAUD = 216;
      break;
    case 9600:
      U0GCR = 8;
      U0BAUD = 59;
      break;
    default:
      U0GCR = 9;
      U0BAUD = 59;
      break;
  }
  UTX0IF = 0; // Clear USART 0 TX interrupt flag
  U0CSR |= 1<<6; // Enable UART0 receiver
  URX0IE = 1; // USART0 RX interrupt enable
  EA = 1; // Each interrupt source is individually enabled or disabled by setting its corresponding enable bit.
}

void Uart_Print(char *p, int len)
{
  unsigned int i;
  for (i = 0; i < len; i++) {
    U0DBUF = *p++; // write data to buffer for sending
    while (!UTX0IF); // If send complete, UTX0IF will be set to 1
    UTX0IF = 0; //  Interrupt not pending
  }
  U0DBUF = 0x0A; // carriage return
  while (!UTX0IF) ;
  UTX0IF = 0;
}