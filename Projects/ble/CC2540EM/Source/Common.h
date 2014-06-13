/*
CC2540 Common  header
ghostyu
20130204
*/

/*
ΪCC2540EM���л���ʵ���ṩ��ͬʹ�õĺ궨����ߺ���������
*/
#ifndef _COMMON_H_
#define _COMMON_H_

/*
���ͺ궨��
*/
typedef signed   char   int8;
typedef unsigned char   uint8;

typedef signed   short  int16;
typedef unsigned short  uint16;

typedef signed   long   int32;
typedef unsigned long   uint32;

typedef unsigned char   bool;

typedef uint8           halDataAlign_t;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL
#define NULL 0
#endif


/*�������ã�������Ч���ǵ���Ч*/
#define ACTIVE_LOW  !
#define ACTIVE_HIGH  !!
/*λ����*/
#define BV(n) (1<<(n))
/*����궨�壬��x�ɰ����������*/
#define st(x)      do { x } while (__LINE__ == -1)

void HalHW_WaitUs(uint16 microSecs);
void HalHW_WaitMS(uint16 ms);
void HAL_BOARD_INIT();

#endif