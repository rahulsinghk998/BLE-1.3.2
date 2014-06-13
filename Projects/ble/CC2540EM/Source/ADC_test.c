/*
CC2540 
ghostyu
20130204
*/

/*
˵����
P0_7�ܽ���ΪADC�����źţ���ӦAIN7
���г�������Z-Stack�У�д�ıȽϱ�׼���򵥵�adת����������Joystick_test.c��
*/
#include<ioCC2540.h>
#include"Common.h"
#include"Hal_lcd.h"

/* Resolution */
#define HAL_ADC_RESOLUTION_8       0x01
#define HAL_ADC_RESOLUTION_10      0x02
#define HAL_ADC_RESOLUTION_12      0x03
#define HAL_ADC_RESOLUTION_14      0x04

/* Channels */
#define HAL_ADC_CHANNEL_0          0x00
#define HAL_ADC_CHANNEL_1          0x01
#define HAL_ADC_CHANNEL_2          0x02
#define HAL_ADC_CHANNEL_3          0x03
#define HAL_ADC_CHANNEL_4          0x04
#define HAL_ADC_CHANNEL_5          0x05
#define HAL_ADC_CHANNEL_6          0x06
#define HAL_ADC_CHANNEL_7          0x07

#define HAL_ADC_CHN_AIN0    0x00    /* AIN0 */
#define HAL_ADC_CHN_AIN1    0x01    /* AIN1 */
#define HAL_ADC_CHN_AIN2    0x02    /* AIN2 */
#define HAL_ADC_CHN_AIN3    0x03    /* AIN3 */
#define HAL_ADC_CHN_AIN4    0x04    /* AIN4 */
#define HAL_ADC_CHN_AIN5    0x05    /* AIN5 */
#define HAL_ADC_CHN_AIN6    0x06    /* AIN6 */
#define HAL_ADC_CHN_AIN7    0x07    /* AIN7 */
#define HAL_ADC_CHN_A0A1    0x08    /* AIN0,AIN1 */
#define HAL_ADC_CHN_A2A3    0x09    /* AIN2,AIN3 */
#define HAL_ADC_CHN_A4A5    0x0a    /* AIN4,AIN5 */
#define HAL_ADC_CHN_A6A7    0x0b    /* AIN6,AIN7 */
#define HAL_ADC_CHN_GND     0x0c    /* GND */
#define HAL_ADC_CHN_VREF    0x0d    /* Positive voltage reference */
#define HAL_ADC_CHN_TEMP    0x0e    /* Temperature sensor */
#define HAL_ADC_CHN_VDD3    0x0f    /* VDD/3 */
#define HAL_ADC_CHN_BITS    0x0f    /* Bits [3:0] */

#define HAL_ADC_CHANNEL_TEMP       HAL_ADC_CHN_TEMP
#define HAL_ADC_CHANNEL_VDD        HAL_ADC_CHN_VDD3   /* channel VDD divided by 3 */

/* Vdd Limits */
#define HAL_ADC_VDD_LIMIT_0        0x00
#define HAL_ADC_VDD_LIMIT_1        0x01
#define HAL_ADC_VDD_LIMIT_2        0x02
#define HAL_ADC_VDD_LIMIT_3        0x03
#define HAL_ADC_VDD_LIMIT_4        0x04
#define HAL_ADC_VDD_LIMIT_5        0x05
#define HAL_ADC_VDD_LIMIT_6        0x06
#define HAL_ADC_VDD_LIMIT_7        0x07

/* Reference Voltages */
#define HAL_ADC_REF_125V          0x00    /* Internal Reference (1.25V-CC2430)(1.15V-CC2540) */
#define HAL_ADC_REF_AIN7          0x40    /* AIN7 Reference */
#define HAL_ADC_REF_AVDD          0x80    /* AVDD_SOC Pin Reference */
#define HAL_ADC_REF_DIFF          0xc0    /* AIN7,AIN6 Differential Reference */
#define HAL_ADC_REF_BITS          0xc0    /* Bits [7:6] */


#define HAL_ADC_EOC         0x80    /* End of Conversion bit */
#define HAL_ADC_START       0x40    /* Starts Conversion */

#define HAL_ADC_STSEL_EXT   0x00    /* External Trigger */
#define HAL_ADC_STSEL_FULL  0x10    /* Full Speed, No Trigger */
#define HAL_ADC_STSEL_T1C0  0x20    /* Timer1, Channel 0 Compare Event Trigger */
#define HAL_ADC_STSEL_ST    0x30    /* ADCCON1.ST =1 Trigger */

#define HAL_ADC_RAND_NORM   0x00    /* Normal Operation */
#define HAL_ADC_RAND_LFSR   0x04    /* Clock LFSR */
#define HAL_ADC_RAND_SEED   0x08    /* Seed Modulator */
#define HAL_ADC_RAND_STOP   0x0c    /* Stop Random Generator */
#define HAL_ADC_RAND_BITS   0x0c    /* Bits [3:2] */

#define HAL_ADC_DEC_064     0x00    /* Decimate by 64 : 8-bit resolution */
#define HAL_ADC_DEC_128     0x10    /* Decimate by 128 : 10-bit resolution */
#define HAL_ADC_DEC_256     0x20    /* Decimate by 256 : 12-bit resolution */
#define HAL_ADC_DEC_512     0x30    /* Decimate by 512 : 14-bit resolution */
#define HAL_ADC_DEC_BITS    0x30    /* Bits [5:4] */

#define HAL_ADC_STSEL       HAL_ADC_STSEL_ST
#define HAL_ADC_RAND_GEN    HAL_ADC_RAND_STOP
#define HAL_ADC_REF_VOLT    HAL_ADC_REF_AVDD
#define HAL_ADC_DEC_RATE    HAL_ADC_DEC_064
#define HAL_ADC_SCHN        HAL_ADC_CHN_VDD3
#define HAL_ADC_ECHN        HAL_ADC_CHN_GND


static uint8 adcRef;

void HalAdcInit (void)
{
  /*����adת���Ĳο���ѹ
  ����ʹ���˺궨�壬��ʵ�������õ���
  ADCCON3.EREF  ADCCON3�Ĵ����ĸ���λ��������ѹѡ��
  00:ѡ���ڲ��ο���ѹ
  01:AIN7�ŵĵ�ѹ��Ϊ�ο���ѹ
  10:AVDD5�ŵĵ�ѹ��Ϊ�ο���ѹ
  11:AIN6��7�Ĳ�ֵ�ѹ��Ϊ�ο���ѹ
  */
  adcRef = HAL_ADC_REF_VOLT;

}

uint16 HalAdcRead (uint8 channel, uint8 resolution)
{
  int16  reading = 0;


  uint8   i, resbits;
  uint8  adcChannel = 1;

  if (channel < 8)
  {
    for (i=0; i < channel; i++)
    {
      adcChannel <<= 1;
    }
  }

  /* Enable channel */
  /*
  ������Ӧͨ�����õ���ģ�����ţ�ʹ�ܸ�����Ϊģ�����ţ�ע�⣬�ò��������Ǳ���ġ�
  TI��datasheet�����Ҳ���ADCCFG����Ĵ�������ʵ���Ǹ�������
  ���������ֽУ�APCFG��Analog peripheral I/O configuration
  */
  ADCCFG |= adcChannel;

  /* Convert resolution to decimation rate */
  /*���������ã�����Ҫע����
  ע����������ʵ�����Ӱ�����adcֵ�Ĵ���
  ADCH/L�е�ֵ���Ҫ��ȡ��Ч��MSBs�����Ǹ߶���λ��
  �������õ�64λ�����ʣ���8λ�ֱ��ʣ�ADC���еĽ����Ҫ����ADCL��ֻ����ADCH��������ADC����8λ��
  ��������512�����ʣ����յ�ADCֵҪ����ADCL�ĵ���λ��������ADCֵ����2λ��
  */
  switch (resolution)
  {
    case HAL_ADC_RESOLUTION_8:
      resbits = HAL_ADC_DEC_064;
      break;
    case HAL_ADC_RESOLUTION_10:
      resbits = HAL_ADC_DEC_128;
      break;
    case HAL_ADC_RESOLUTION_12:
      resbits = HAL_ADC_DEC_256;
      break;
    case HAL_ADC_RESOLUTION_14:
    default:
      resbits = HAL_ADC_DEC_512;
      break;
  }

  /* writing to this register starts the extra conversion */
  /*�μ�����˵�������ò���ͨ�����������Լ���׼��ѹ*/
  ADCCON3 = channel | resbits | adcRef;

  /* Wait for the conversion to be done */
  /*�ȴ�������������־λΪADCCON1�����λEOC
	��ת��������EOC��1��while����ѭ��
  */
  while (!(ADCCON1 & HAL_ADC_EOC));

  /* Disable channel after done conversion */
  /*ת����ȡ������ͨ����ģ����������*/
  ADCCFG &= (adcChannel ^ 0xFF);

  /* Read the result */
  /*����ADCֵ��ADCH����8+ADCL*/
  reading = (int16) (ADCL);
  reading |= (int16) (ADCH << 8);

  /* Treat small negative as 0 */
  if (reading < 0)
    reading = 0;

  /*�μ�����˵��*/
  switch (resolution)
  {
    case HAL_ADC_RESOLUTION_8:
	  /*64λ�����ʣ�ADC�ĸ�8λΪ��Чֵ�����reading����8λ*/
      reading >>= 8;
      break;
    case HAL_ADC_RESOLUTION_10:
	  /*128λ�����ʣ�ADC�ĸ�10λΪ��Чֵ�����reading����6λ*/
      reading >>= 6;
      break;
    case HAL_ADC_RESOLUTION_12:
	  /*256λ�����ʣ�ADC�ĸ�12λΪ��Чֵ�����reading����4λ*/
      reading >>= 4;
      break;
    case HAL_ADC_RESOLUTION_14:
    default:
	  /*512λ�����ʣ�ADC�ĸ�14λΪ��Чֵ�����reading����2λ*/
      reading >>= 2;
    break;
  }

  return ((uint16)reading);
}

void HalAdcSetReference ( uint8 reference )
{

  adcRef = reference;
}

/*������ѹ���жϹ����Ƿ����ȶ�*/
bool HalAdcCheckVdd(uint8 vdd)
{
  ADCCON3 = 0x0F;
  while (!(ADCCON1 & 0x80));
  return (ADCH > vdd);
}

int main()
{
  uint16 adc;
  uint16 tmp;
  int8 buf[10];
  //HAL_BOARD_INIT();

  HalLcd_HW_Init();
  
  HalLcd_HW_WriteLine(HAL_LCD_LINE_1, "      CC2540EM");
  HalLcd_HW_WriteLine(HAL_LCD_LINE_3, "-->ADC_test");
  
  HalAdcInit();
  while(1){
	/*adcת��*/
    adc=HalAdcRead(HAL_ADC_CHANNEL_7,HAL_ADC_RESOLUTION_8);
	/*��ת���õ����������г��ַ���*/
    IntToStr(buf,adc);
    HalLcd_HW_WriteLine(HAL_LCD_LINE_4, buf);
    HalHW_WaitMS(1000);//��ʱ
  }
}