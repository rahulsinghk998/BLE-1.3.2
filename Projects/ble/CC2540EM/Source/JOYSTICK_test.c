/*
CC2540 
ghostyu
20130204
*/

/*
˵����
���򰴼�ʵ�飬���򰴼�ʹ��adc������
��5�������е�һ������ʱ����P2_0���Ų���һ���жϣ�
Ȼ���ȡADCͨ��6�ĵ�ѹֵ��
ͨ����ͬ�ĵ�ѹֵ���������ĸ�����
*/
#include<ioCC2540.h>
#include"Common.h"
#include"Hal_lcd.h"

void JOYSTICK_ADC_Init();

/*���򰴼���ʹ������GPIO����
  P2_0 �ж�
  P0_6 adת��
*/
/* Joy stick move at P2.0 */
#define HAL_KEY_JOY_MOVE_PORT   P2
#define HAL_KEY_JOY_MOVE_PIN    0

//5�򰴼� ����ͨ��Ϊad-6
#define HAL_KEY_JOY_CHN   6


uint16 JOYSTICK_Value = 0;
uint8 flag=0;

/*���򰴼�GPIO��ʼ���� 
  ��P2_0
*/
void JOYSTICK_Init()
{
  P2SEL |= BV(HAL_KEY_JOY_MOVE_PIN);  //P2_0 ��Ϊ��ͨGPIO
  P2DIR &= ~BV(HAL_KEY_JOY_MOVE_PIN); //P2_0 ��Ϊ����
  
  PICTL &= ~(BV(3));//����PORT2 �������ж�
  P2IFG = 0;  //���Port2�˿��жϱ�־�Ĵ���
}
/*ʹ�����򰴼���GPIO�ж�*/
void JOYSTICK_Enable()
{
  EA = 1;
  P2IEN |= BV(HAL_KEY_JOY_MOVE_PIN);
  IEN2 |= BV(1); //����PORT2 �ж�ʹ��
}
/*���򰴼��õ���ADC���ֳ�ʼ��*/
void JOYSTICK_ADC_Init()
{
  ADCCON3 |= 0x80;//���òο���ѹ������Ϊѡ��AVDD5
  ADCCON3 |= 0x00;//���÷ֱ���Ϊ8λ
  ADCCON1 |= 0x30;//����ADC������ʽ������Ϊ�����������stΪ1ʱ ��ʼADCת��
}
//adc����ͨ��ѡ�񣬴�0~7���ֱ��ӦAIN0 ~ AIN7
//����ͨ�� ÿ�β���ǰ�����������ã�
void JOYSTICK_ADC_SelectChanel(uint8 ch)
{
    ADCCON3 |= ch;//select ainx as input
    //APCFG |= BV(ch); //���Ǳ�������ã�����ADCCFG
}
/*ADC������ֹͣ����
  sΪtrueʱ��ʼ����
  ����ֹͣ����
*/
void JOYSTICK_ADC_Start(bool s)
{
  if(s){
    ADCCON1 |= BV(6);
  }else{
    ADCCON1 &= ~BV(6);
  }
}
/*ADC�Ƿ��������
  �����󷵻ط�0��
*/
uint8 JOYSTICK_ADC_Busy()
{
  return (ADCCON1&0x80);
}
/*��ȡADC����ֵ*/
uint16 JOYSTICK_ADC_Get()
{
  uint16 t = ((uint16)(ADCH)<<8)+ADCL;
  t>>=8;  //8Ϊ�ֱ��ʣ��õ���ֵ����8λ��������256
  return t;
}

int main()
{
  //HAL_BOARD_INIT();
  HalLcd_HW_Init();
  
  JOYSTICK_Init();
  JOYSTICK_Enable();
  
  JOYSTICK_ADC_Init();
  JOYSTICK_ADC_Get();
  
  HalLcd_HW_WriteLine(HAL_LCD_LINE_1, "      CC2540EM");
  HalLcd_HW_WriteLine(HAL_LCD_LINE_3, "-->JOYSTICK_TEST"); 
  
  while(1){
    if(flag ==1){
      /*��ͬ�Ĳ�����ѹ��Ӧ��ͬ�İ���*/
      if ((JOYSTICK_Value >= 2) && (JOYSTICK_Value <= 38))
      {
         HalLcd_HW_WriteLine(HAL_LCD_LINE_5,"UP");
      }
      else if ((JOYSTICK_Value >= 74) && (JOYSTICK_Value <= 88))
      {
        HalLcd_HW_WriteLine(HAL_LCD_LINE_5,"RIGHT");
      }
      else if ((JOYSTICK_Value >= 60) && (JOYSTICK_Value <= 73))
      {
        HalLcd_HW_WriteLine(HAL_LCD_LINE_5,"LEFT");
      }
      else if ((JOYSTICK_Value >= 39) && (JOYSTICK_Value <= 59))
      {
        HalLcd_HW_WriteLine(HAL_LCD_LINE_5,"DOWN");
      }
      else if ((JOYSTICK_Value >= 89) && (JOYSTICK_Value <= 100))
      {
        HalLcd_HW_WriteLine(HAL_LCD_LINE_5,"CENTER");
      }
      flag = 0;
     
    }


  }
}


/*GPIO�жϺ���*/
#pragma vector = P2INT_VECTOR
__interrupt void P2_ISR(void) 
{ 
  if(P2IFG&BV(HAL_KEY_JOY_MOVE_PIN)){         //�����ж� 
    P2IFG &= ~BV(HAL_KEY_JOY_MOVE_PIN); 
    
    //���ñ��εĲ���ͨ��
    JOYSTICK_ADC_SelectChanel(6);
    //��ʼ����
    JOYSTICK_ADC_Start(1);
    //�ȴ�ת������
    while(!JOYSTICK_ADC_Busy());
    //��ȡ�������
    JOYSTICK_Value = JOYSTICK_ADC_Get();
    
    flag = 1;
  } 
}