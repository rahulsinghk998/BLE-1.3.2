/*
CC2540 
ghostyu
20130204
*/

/*
˵������������ģ����Գ���
������������⵽����ʱ��GPIO����ߵ�ƽ��������ʧ�󣬱�͵�ƽ
*/
#include<ioCC2540.h>
#include"Common.h"
#include"Hal_lcd.h"

//��������ģ�����õ���GPIO
#define HAL_RELAY_PIN       4
#define HAL_RELAY_PIN_CTL   P0_4

/*��������ģ���ʼ��
  ��ʼ����������ģ���õ���GPIO
*/
void Smoke_Init()
{
  P0DIR &= ~BV(HAL_RELAY_PIN);  //���룬����
  P0SEL &= ~BV(HAL_RELAY_PIN);

}

int main()
{
  HalLcd_HW_Init();
  
  HalLcd_HW_WriteLine(HAL_LCD_LINE_1, "      CC2540EM");
  HalLcd_HW_WriteLine(HAL_LCD_LINE_3, "-->SMOKE_TEST");
  
  Smoke_Init();
  while(1){
    //����⵽����ʱ��ģ������ߵ�ƽ����ʼ����
    if(HAL_RELAY_PIN_CTL==1){
      HalLcd_HW_WriteLine(HAL_LCD_LINE_4, "HEIGH");
    }else{
      HalLcd_HW_WriteLine(HAL_LCD_LINE_4, "LOW");
    }
    //HalHW_WaitUs(500000);
  }
  
}