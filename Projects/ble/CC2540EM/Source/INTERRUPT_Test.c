/*
CC2540EM Interrupt Test
ghostyu
20130204
*/

/*
ʹ�õװ�İ���S1������led�Ƶ����𣬰���S1ʹ���жϴ����ź�
*/
#include<ioCC2540.h>
#include"Common.h"
#include"hal_lcd.h"
/*
MAP
S1--->P0_1
*/
#define HAL_S1_PIN  1

uint8 S1_Flag = 0;


/*����S1�˿ڳ�ʼ��*/
void S1_INT_Init()
{
  P0SEL |= BV(HAL_S1_PIN);//����
  P0DIR &= ~(BV(HAL_S1_PIN));//����
  
  P0INP &= ~(BV(HAL_S1_PIN));//������������ģʽ
  P2INP |= BV(5);//P0�˿����ó�����
  
  PICTL &= ~(BV(0));//�������ж�
  P0IFG = 0;
}

/*ʹ��S1�ж�*/
void S1_INT_Enable()
{
  EA = 1;//����CPU�ж�
  P0IEN |= BV(HAL_S1_PIN);//����HAL_S1_PIN�����ж�ʹ��
  IEN1|= BV(5); //�������÷ǳ���Ҫ����Ȼ�������жϺ���
}
/*ʧ��S1�ж�*/
void S1_INT_Disable()
{
  P0IEN &= ~(BV(HAL_S1_PIN));
}

/*********************************LEDs*****************************************/
/*LEDs�˿ڶ���*/
#define LED1_SBIT   P1_0
#define LED2_SBIT   P1_1
#define LED3_SBIT   P1_4
/*LEDs���Զ��壬�ߵ�ƽ��Ч*/
#define LEDs_POLARITY   ACTIVE_HIGH

/*LEDs���غ궨��*/
#define HAL_TURN_OFF_LED1()       st( LED1_SBIT = LEDs_POLARITY (0); )
#define HAL_TURN_OFF_LED2()       st( LED2_SBIT = LEDs_POLARITY (0); )
#define HAL_TURN_OFF_LED3()       st( LED3_SBIT = LEDs_POLARITY (0); )

#define HAL_TURN_ON_LED1()        st( LED1_SBIT = LEDs_POLARITY (1); )
#define HAL_TURN_ON_LED2()        st( LED2_SBIT = LEDs_POLARITY (1); )
#define HAL_TURN_ON_LED3()        st( LED3_SBIT = LEDs_POLARITY (1); )

void GPIO_LedsInit()
{
  /*�ȹ�����led����ֹ��ʼ��ʱled��˸*/
  HAL_TURN_OFF_LED1();
  HAL_TURN_OFF_LED2();
  HAL_TURN_OFF_LED3();
  
  P1DIR |= BV(0)|BV(1)|BV(4);
  
  P1SEL &= ~(BV(0)|BV(1)|BV(4));
}
/*********************************LEDs*****************************************/

int main()
{
  //HAL_BOARD_INIT();
  GPIO_LedsInit();

  HalLcd_HW_Init();
  
  HalLcd_HW_WriteLine(HAL_LCD_LINE_1, "      CC2540EM");
  HalLcd_HW_WriteLine(HAL_LCD_LINE_3, "-->INTERRUPT_TEST");
  
  /*S1������ʼ��*/
  S1_INT_Init();
  S1_INT_Enable();
  
  while(1){
    if(S1_Flag==0){
      HAL_TURN_ON_LED1();
      HAL_TURN_ON_LED2();
      HAL_TURN_ON_LED3();
      HalLcd_HW_WriteLine(HAL_LCD_LINE_5, "ALL LED:ON");
    }else{
      HAL_TURN_OFF_LED1();
      HAL_TURN_OFF_LED2();
      HAL_TURN_OFF_LED3();
      HalLcd_HW_WriteLine(HAL_LCD_LINE_5, "ALL LED:OFF");
    }
  }
}


/*�жϺ���*/
#pragma vector = P0INT_VECTOR
__interrupt void P0_ISR(void) 
{ 
  if(P0IFG&BV(HAL_S1_PIN)){         //�����ж� 
    P0IFG &= ~BV(HAL_S1_PIN); 
    S1_Flag = ~S1_Flag;
  } 
}