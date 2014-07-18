#ifndef __I2C_H__
#define __I2C_H__

#include <iocc2540.h>
#include "hal_types.h"


/*===========================================================================
 * DEFINES
 *=========================================================================*/
#define BIT0  0x01
#define BIT1  0x02
#define BIT2  0x04
#define BIT3  0x08
#define BIT4  0x10
#define BIT5  0x20
#define BIT6  0x40
#define BIT7  0x80

#define SDA   BIT6 // Controls SDA line (pull-up used for logic 1)
#define SCL   BIT5 // Controls SCL line (pull-up used for logic 1)

#define PxSEL P1SEL  // Port selection
#define PxDIR P1DIR  // Port direction
#define Px    P1     // Port input/output
#define PxIN  P1INP  // Port input configuration


/*===========================================================================
 * DEFINITIONS
 *=========================================================================*/
static void _NOP(void){asm("NOP");}

void i2c_Init();

void i2c_start();

void i2c_txByte(unsigned char data);

unsigned char i2c_rxByte(bool ack);

void i2c_ack(void);

void i2c_stop(void);

void i2c_readBlock(unsigned char SlaveAddress, unsigned int numBytes, void* RxData);

void i2c_writeBlock(unsigned char SlaveAddress, unsigned int numBytes, void* TxData);

#endif

