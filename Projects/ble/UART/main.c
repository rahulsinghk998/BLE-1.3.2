// 

#include "uart.h"
#include "led.h"
#include <string.h>
unsigned char cmd;

void main(void)
{
  Led_Init();
  Uart_Init(9600);
  while(1) {
    switch(cmd) {
    case 'A':
      cmd = 0;
      Uart_Print("LED1_ON", strlen("LED1_ON"));
      Led1_On();
      break;
    case 'B':
      cmd = 0;
      Uart_Print("LED1_OFF", strlen("LED1_OFF"));
      Led1_Off();
      break;
    case 0:
      break;
    default:
      cmd = 0;
      Uart_Print("Please check cmd!", strlen("Please check cmd!"));
      break;
    }      
  }
}

#pragma vector = URX0_VECTOR
__interrupt void URX0_ISR(void)
{
  cmd = U0DBUF;
  URX0IF = 0;
}