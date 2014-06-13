/*
CC2540 
ghostyu
20130204
*/

/*
˵�����̵���ģ�����

*/
#include<ioCC2540.h>
#include"Common.h"
#include"Hal_lcd.h"

/*�̵������õ����Ŷ���*/
#define HAL_RELAY_PIN       4
#define HAL_RELAY_PIN_CTL   P0_4
#define HAL_RELAY_PORT_DIR      P0DIR
#define HAL_RELAY_PORT_SEL      P0SEL
/*�̵���ģ���ʼ��
  ��ʼ���̵����õ���GPIO
*/
void Relay_Init()
{
  /*��ֹ����*/
  HAL_RELAY_PIN_CTL = 0;
  HAL_RELAY_PORT_DIR |= BV(HAL_RELAY_PIN);
  HAL_RELAY_PORT_SEL &= ~BV(HAL_RELAY_PIN);

}


int main()
{
  HalLcd_HW_Init();
  
  HalLcd_HW_WriteLine(HAL_LCD_LINE_1, "      CC2540EM");
  HalLcd_HW_WriteLine(HAL_LCD_LINE_3, "-->RELAY_TEST");
  
  Relay_Init();
  while(1){
    //ѭ���øߺ��õͼ̵������ü̵�����ͣ�Ŀ���
    HAL_RELAY_PIN_CTL = 1;
    HalLcd_HW_WriteLine(HAL_LCD_LINE_4, "HEIGH");
    HalHW_WaitUs(1000000);
    HAL_RELAY_PIN_CTL = 0;
    HalLcd_HW_WriteLine(HAL_LCD_LINE_4, "LOW");
    HalHW_WaitUs(1000000);
  }
  
}