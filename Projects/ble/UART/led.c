#include "Led.h"
void Led_Init()
{
  P1SEL &= ~((1<<4)|(1<<7));
  P1DIR |= (1<<4)|(1<<7);
  Led1_Off();
  Led2_Off();
}

void Delay(unsigned int time)
{
  unsigned int i;
  while(time--) {
    for (i = 0; i < 10000; i++);
  }
}