/*---------------------------------------------------------------------------
* Description: I2C implemented use SWI for the cB-OLP425 board.
*
* CC2540 doesn't have dedicated I2C ports, need to implement by software
* Refer to slva302 from ti.com
*      Master
*      CC2540
*  ---------------
* | I2C SCL: P1_5  | -----> [I2C SLAVE SCL]
* | I2C SDA: P1_6  | -----> [I2C SLAVE SDA]
*  ---------------
*---------------------------------------------------------------------------*/

#include "i2c.h"

/*---------------------------------------------------------------------------
* Description of function. Optional verbose description.
*-------------------------------------------------------------------------*/
void i2c_Init()
{
  PxSEL &= ~(SCL | SDA);                    // Set GPIO function
  PxDIR &= ~(SCL | SDA);                    // Set output direction to input
  Px    &= ~(SCL | SDA);                    // SCL & SDA = 0, low when = outputs 

  PxIN  &= ~(SCL | SDA);                    // Set to Pull up or down, see P2INP
  P2INP &= ~0x40;                           // P2INP.PDUP1 = 1, port 1 to pull up
}

/*---------------------------------------------------------------------------
* Set up start condition for I2C
*-------------------------------------------------------------------------*/
void i2c_start()
{
  PxDIR &= ~SCL;                            // Set to input, SCL = 1 via pull-up
  _NOP();                                   // Quick delay
  _NOP();                                   // Quick delay
  PxDIR |= SDA;                             // Set to output, data low [SDA = 0]
  _NOP();                                   // Quick delay
  _NOP();                                   // Quick delay
  PxDIR |= SCL;                             // Set to output, data low [SCL = 0]
}


/*---------------------------------------------------------------------------
* Set up for I2C ack receivement
*-------------------------------------------------------------------------*/
void i2c_wait_ack(void)
{
  PxDIR &= ~SDA;                            // Set to input, SDA = 1 via pull-up    
  PxDIR &= ~SCL;                            // Set to input, SCL = 1 via pull-up
  _NOP();
  _NOP();

  // TBD break if ack is not received within a certain time
  while (PxIN & SDA == SDA)
  {
    _NOP();
  }

  PxDIR |= SCL;                             // Set to output, data low [SCL = 0]  
}

/*---------------------------------------------------------------------------
* Description of function. Optional verbose description.
*-------------------------------------------------------------------------*/
void i2c_txByte(unsigned char data)
{
  unsigned char bits, temp;

  temp = data;
  bits = 0x08;                              // Load I2C bit counter
  while (bits != 0x00)                      // Loop until all bits are shifted
  {
    if (temp & BIT7)                        // Test data bit
      PxDIR &= ~SDA;                        // Set to input, SDA = 1 via pull-up
    else
      PxDIR |= SDA;                         // Set to output, data low [SDA = 0]
    PxDIR &= ~SCL;                          // Set to output, data low [SCL = 0]
    temp = (temp << 1);                     // Shift bits 1 place to the left
    _NOP();                                 // Quick delay
    PxDIR |= SCL;                           // Set to output, data low [SCL = 0]
    _NOP();                                 // Quick delay
    bits = (bits - 1);                      // Loop until 8 bits are sent
  }

  i2c_wait_ack();                           // wait for acknowledge
}

/*---------------------------------------------------------------------------
* Description of function. Optional verbose description.
*-------------------------------------------------------------------------*/
void i2c_ack(void)                          // Set up for I2C acknowledge
{
  PxDIR |= SCL;                             // Set to output, data low [SCL = 0]
  PxDIR |= SDA;                             // Set to output, data low [SDA = 0]

  PxDIR &= ~SCL;                            // Set to input, SCL = 1 via pull-up
  _NOP();                                   // delay to meet I2C spec
  _NOP();                                   //   "        "        "
  PxDIR |= SCL;                             // Set to output, data low [SCL = 0]
  PxDIR &= ~SDA;                            // Set to input, SDA = 1 via pull-up
}

/*---------------------------------------------------------------------------
* Description of function. Optional verbose description.
*-------------------------------------------------------------------------*/
unsigned char i2c_rxByte(bool ack)          // Read 8 bits of I2C data
{
  unsigned char bits, temp = 0;             // I2C bit counter

  PxDIR &= ~SDA;                            // Set to input, SDA = 1 via pull-up  

  bits = 0x08;                              // Load I2C bit counter
  while (bits > 0)                          // Loop until all bits are read
  {
    PxDIR &= ~SCL;                          // Set to input, SDL = 1 via pull-up
    _NOP();                                 // Quick delay
    temp = (temp << 1);                     // Shift bits 1 place to the left
    if (Px & SDA)                           // Check digital input
      temp = (temp + 1);                    // If input is 'H' store a '1'
    _NOP();                                 // Quick delay
    PxDIR |= SCL;                           // Set to output, data low [SCL = 0]
    _NOP();                                 // Quick delay      
    bits = (bits - 1);                      // Decrement I2C bit counter
  }
  if (ack == TRUE)
  {
    i2c_ack();                          // Send acknowledge
  }
  return (temp);                            // Return 8-bit data byte
}   

/*---------------------------------------------------------------------------
* Description of function. Optional verbose description.
*-------------------------------------------------------------------------*/
void i2c_stop(void)                         // Send I2C stop command
{
  PxDIR |= SDA;                             // Set to output, data low [SCA = 0]
  _NOP();                                   // Quick delay
  _NOP();                                   // Quick delay
  PxDIR &= ~SCL;                            // Set to input, SCL = 1 via pull-up
  _NOP();                                   // Quick delay
  _NOP();                                   // Quick delay
  PxDIR &= ~SDA;                            // Set to input, SDA = 1 via pull-up
} 

/*---------------------------------------------------------------------------
* Description of function. Optional verbose description.
*-------------------------------------------------------------------------*/
void i2c_readBlock(unsigned char SlaveAddress,
                     unsigned int numBytes, 
                     void* RxData)
{
  unsigned char* temp;
  temp = (unsigned char *)RxData;           // Initialize array pointer

  i2c_start();                          // Send Start condition
  for (unsigned int i = 0; i < numBytes; i++) {
    i2c_txByte((SlaveAddress << 1) | BIT0); // [ADDR] + R/W bit = 1
    *(temp) = i2c_rxByte(TRUE);     // Read 8 bits of data and send ack
    temp++;                                 // Increment pointer to next element
  }
  i2c_stop();                   // Send Stop condition
}

/*---------------------------------------------------------------------------
* Description of function. Optional verbose description.
*-------------------------------------------------------------------------*/
void i2c_writeBlock(unsigned char SlaveAddress,
                      unsigned int numBytes, void* TxData)
{        
  unsigned char *temp;

  temp = (unsigned char *)TxData;          // Initialize array pointer
  i2c_start();                 // Send Start condition
  i2c_txByte((SlaveAddress << 1) & ~BIT0); // [ADDR] + R/W bit = 0
  for (unsigned int i = 0; i < numBytes; i++) {
    i2c_txByte(*(temp));       // Send data and ack
    temp++;                                // Increment pointer to next element
  }
  i2c_stop();                  // Send Stop condition
}
