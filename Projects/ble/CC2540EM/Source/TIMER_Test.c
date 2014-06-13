/*
CC2540EM TIMERs Test
ghostyu
20130204
*/

/*
ʹ�ö�ʱ�������GPIOʵ���е�LED��ʾ��ʱ
�������ʹ��Timer3��Ϊ��ʾ��timer124�Ȳ���������
*/
#include <ioCC2540.h>
#include "Common.h"
#include "hal_lcd.h"

uint32 counter = 0;
uint8  timeout = 0;
void HalTimer3Init()
{
  //T3CTL &= ~0x03;
  T3CTL = 0xAE;	//32��Ƶ���ж�ʹ�ܣ�ģģʽ
  T3CC0 = 0xC8;	// 1Mhz==1us  200us�ж�
  T3CCTL0 = 0x44;//һ��Ҫ���ñȽϵķ�ʽ����Ȼ�����жϵ�!!!,T3CCTL0.IM==1,T3CCTL0.MODE==1
}
/*��ʼ��ʱ*/
void HalTimer3Start()
{
  T3CTL |= 0x10; 
}
/*ֹͣ��ʱ*/
void HalTimer3Stop()
{
  T3CTL &= ~0x10; 
}
/*�ж�ʹ��*/
void HalTimer3EnableInt()
{
  EA = 1;
  T3IE = 1;
  //IEN1 |= BV(3);
}
/*�жϲ�ʹ��*/
void HalTimer3DisableInt()
{
  T3IE = 0;
  //IEN1 &= ~ BV(3);
}


/*********************************LEDs*****************************************/
/*
MAP
LED1--->P1_0
LED2--->P1_1
LED3--->P1_4
LED4--->P0_1
*/
/*LEDs�˿ڶ���*/
#define LED1_SBIT   P1_0
#define LED2_SBIT   P1_1
#define LED3_SBIT   P1_4
#define LED4_SBIT   P0_1
/*LEDs���Զ��壬�ߵ�ƽ��Ч*/
#define LEDs_POLARITY   ACTIVE_HIGH

/*LEDs���غ궨��*/
#define HAL_TURN_OFF_LED1()       st( LED1_SBIT = LEDs_POLARITY (0); )
#define HAL_TURN_OFF_LED2()       st( LED2_SBIT = LEDs_POLARITY (0); )
#define HAL_TURN_OFF_LED3()       st( LED3_SBIT = LEDs_POLARITY (0); )
#define HAL_TURN_OFF_LED4()       st( LED4_SBIT = LEDs_POLARITY (0); )

#define HAL_TURN_ON_LED1()        st( LED1_SBIT = LEDs_POLARITY (1); )
#define HAL_TURN_ON_LED2()        st( LED2_SBIT = LEDs_POLARITY (1); )
#define HAL_TURN_ON_LED3()        st( LED3_SBIT = LEDs_POLARITY (1); )
#define HAL_TURN_ON_LED4()        st( LED4_SBIT = LEDs_POLARITY (1); )

void GPIO_LedsInit()
{
  /*�ȹ�����led����ֹ��ʼ��ʱled��˸*/
  HAL_TURN_OFF_LED1();
  HAL_TURN_OFF_LED2();
  HAL_TURN_OFF_LED3();
  HAL_TURN_OFF_LED4();
  
  P1DIR |= BV(0)|BV(1)|BV(4);
  P0DIR |= BV(1);
  
  P1SEL &= ~(BV(0)|BV(1)|BV(4));
  P0SEL &= ~(BV(1));
}
/*********************************LEDs*****************************************/

int main()
{
  //������ʱ�ӳ�ʼ������ʹ���ڲ���16M������Ϊʱ�ӣ�����������ʱ��1s�ͱ����2s��
  HAL_BOARD_INIT();
  GPIO_LedsInit();

  HalLcd_HW_Init();
  
  HalLcd_HW_WriteLine(HAL_LCD_LINE_1, "      CC2540EM");
  HalLcd_HW_WriteLine(HAL_LCD_LINE_3, "-->TIMER3_TEST");
  
  /*��ʱ����ʼ��*/
  HalTimer3Init();
  HalTimer3EnableInt();
  HalTimer3Start();
  
  while(1){
    HAL_TURN_ON_LED1();
    HalLcd_HW_WriteLine(HAL_LCD_LINE_5, "Green LED:ON");
    while(!timeout);//�ȴ���־λ��λ
    timeout = 0;    //�����־λ���ȴ���ʱ�жϺ�����λ
    HAL_TURN_OFF_LED1();
    
    HAL_TURN_ON_LED2();
    HalLcd_HW_WriteLine(HAL_LCD_LINE_5, "Red LED:ON");
    while(!timeout);
    timeout = 0;
    HAL_TURN_OFF_LED2();
    
    HAL_TURN_ON_LED3();
    HalLcd_HW_WriteLine(HAL_LCD_LINE_5, "Yellow LED:ON");
    while(!timeout);
    timeout = 0;
    HAL_TURN_OFF_LED3();
    
    HAL_TURN_ON_LED4();
    HalLcd_HW_WriteLine(HAL_LCD_LINE_5, "Orange LED:ON");
    while(!timeout);
    timeout = 0;
    HAL_TURN_OFF_LED4();
    
  }
}
/*�жϺ���*/
#pragma vector = T3_VECTOR 
 __interrupt void T3_ISR(void) 
{ 
  //�ж�1000��Ϊ200ms
  if(counter<5000)
    counter++; 
  else{ 
    timeout = 1;  //��־λ��λ
    counter = 0;  //counter���㣬���¿�ʼ����
  } 
} 