/*
CC2540 Common Source
ghostyu
20130204
*/

/*
ΪCC2540EM���л���ʵ���ṩ��ͬʹ�õĺ�����
*/
#include<ioCC2540.h>
#include"Common.h"
/**************************************************************************************************
�����ʱ
����Ҫע���ˣ���ʹ��uint32���ͣ�ʱ�佫�ӳ�5��������
��
void HalHW_WaitUs(uint32 microSecs)
 **************************************************************************************************/
void HalHW_WaitUs(uint16 microSecs)
{
  while(microSecs--)
  {
    /* 3 NOPs == 1 usecs */
        asm("NOP");
        asm("NOP");
        asm("NOP");

  }
}

void HalHW_WaitMS(uint16 ms)
{
  while(ms--){
    int i=0;
    for(i=20; i>0; i--)
      HalHW_WaitUs(50);
  }
}
/*�����ʼ��*/
void HAL_BOARD_INIT()
{
  CLKCONCMD &= ~(BV(6));//ʹ��32M �ⲿ����
  while(!(SLEEPSTA & (BV(6))));  //�ȴ�����������ɣ�SLEEPSTA��CLKCONCMD��һ������
  CLKCONCMD &= ~(BV(5)|BV(4)|BV(3)|BV(2)|BV(1)|BV(0));
  SLEEPCMD |= 0x04;       
}


//