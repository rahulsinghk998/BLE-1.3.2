#include "uart.h"

void main(void)
{
  Uart_Init();
  while(1) {
    Uart_Print("Texas Instruments", 17);
  }
}