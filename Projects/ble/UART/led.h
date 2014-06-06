#ifndef __LED_H__
#define __LED_H__
#include <iocc2540.h>

#define LED1 P1_7
#define LED2 P1_4
#define Led1_On() LED1 = 0;
#define Led1_Off() LED1 = 1;
#define Led2_On() LED2 = 0;
#define Led2_Off() LED2 = 1;

extern void Led_Init(void);
extern void Delay(unsigned int time);

#endif