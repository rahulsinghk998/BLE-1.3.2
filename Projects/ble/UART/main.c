/* Test Project for cB-OLP425 based on Texas Instrument's CC2540 chip
** test the UART IRQ, LED
** I2c with the LIS3DH accelerometer
** 
** 
*/

#include "uart.h"
#include "led.h"
#include "i2c.h"
#include "tmp112.h"
#include <string.h>
#include <stdio.h>
unsigned char cmd;

void main(void)
{
  // Set system clock source to HS XOSC, with no pre-scaling.
  // set CLKCONCMD.OSC and CLKSPD to 0, and then apply 32M
  CLKCONCMD = (CLKCONCMD & ~(CLKCON_OSC | CLKCON_CLKSPD)) | CLKCON_CLKSPD_32M; 
  
  // Wait until clock source has changed
  while (CLKCONSTA & CLKCON_OSC); 
  
  Led_Init();
  i2c_Init();
  Uart_Init(115200);
  bool tempSensorOK = FALSE;
  tempSensorOK = TMP112_open();
  
  if (tempSensorOK == TRUE) {
    Uart_Print("TMP112 OK", strlen("TMP112 OK"));
  }
  else {
    Uart_Print("TMP112 Failed", strlen("TMP112 Failed"));
  }  
  
  uint16 rawTemperature = 0;
  int8 temperature = 0;
  char numStr[10];
  
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
    case 'T':
      cmd = 0;
      readTemperature(&temperature, &rawTemperature);
      //Uart_Print("Current temp:", strlen("Current temp:"));
      sprintf(numStr, "%d", temperature);
      Uart_Print(numStr, 2);
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