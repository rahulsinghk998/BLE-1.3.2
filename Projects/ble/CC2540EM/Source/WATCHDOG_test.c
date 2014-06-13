/*
CC2540 
ghostyu
20130204
*/

/*
˵����
���Ź����飬��ʼ�����Ź���λʱ��Ϊ1s����������ʱ����led1����λ�����µ�������ʵ��led��˸��Ч��
*/
#include<ioCC2540.h>
#include"Common.h"
#include"Hal_lcd.h"

/***************************************LED************************************/
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
/******************************************************************************/


#define WATCHDOG_MODE_WATCHDOG          0x08  /*watchdog time mode:watchdog mode*/

#define WATCHDOG_REBOOT_TIME_1S         0x00  /*watchdog reboot time:1s*/
#define WATCHDOG_REBOOT_TIME_0_25S      0x01  /*0.25s*/
#define WATCHDOG_REBOOT_TIME_15_625MS   0x02  /*15.625ms*/
#define WATCHDOG_REBOOT_TIME_1_9MS      0x03  /*1.9ms*/


#define WATCHDOG_FEED_ONE 0x50  /*watchdog feed first bits*/
#define WATCHDOG_FEED_TWO 0xA0  /*watchdog feed second bits*/


/*�򿪿��Ź�������ʼ��Ϊ1S��δι������λ*/
void WatchdogOpen()
{
  /*���ÿ��Ź���ʱ��Ϊ���Ź�ģʽ*/
  WDCTL |= WATCHDOG_MODE_WATCHDOG;
  /*���ÿ��Ź���λʱ��Ϊ1S*/
  WDCTL |= WATCHDOG_REBOOT_TIME_1S;
}

/*ι���������0x50 �����0xA0*/
void WatchdogFeed()
{
  WDCTL |= WATCHDOG_FEED_ONE;
  WDCTL |= WATCHDOG_FEED_TWO;
}

int main()
{
 // HAL_BOARD_INIT();
  GPIO_LedsInit();
  HalLcd_HW_Init();
  
  HalLcd_HW_WriteLine(HAL_LCD_LINE_1, "      CC2540EM");
  HalLcd_HW_WriteLine(HAL_LCD_LINE_3, "-->WATCHDOG_test");
  
  WatchdogOpen();
  WatchdogFeed();
  while(1){
    HAL_TURN_ON_LED1();
  }
}