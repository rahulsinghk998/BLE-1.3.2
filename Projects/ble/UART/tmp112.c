#include "tmp112.h"
#include "i2c.h"


/*===========================================================================
 * DEFINES
 *=========================================================================*/
#define TMP112_ADDRESS                          (0x49)
#define TMP112_CONVERSION_DELAY_IN_MS           (50)

#define TMP112_PERIODIC_TIMEOUT                 (1 << 4)
#define TMP112_CONVERSION_TIMEOUT               (1 << 5)


/*===========================================================================
 * TYPES
 *=========================================================================*/
typedef enum
{
  TMP112_S_NOT_INITIALIZED = 0,
  TMP112_S_IDLE,
  TMP112_S_WAIT,
  TMP112_S_WAIT_CONVERSION,

} TMP112_State;

/*===========================================================================
 * DECLARATIONS
 *=========================================================================*/
static void write(uint8 reg, uint8 byte1, uint8 byte2);
static void read(uint8 reg, uint8 *pByte1, uint8 *pByte2);



/*===========================================================================
 * DEFINITIONS
 *=========================================================================*/
#define TMP112_TEMPERATURE_REGISTER     0x00
#define TMP112_CONFIGURATION_REGISTER   0x01

bool TMP112_open(void)
{
  uint8 byte1, byte2;
  bool sensorOk = TRUE;
    
  // Write configuration register
  // Byte1: 0110 0000 One-Shot conversion ready, shutdown mode = 0 (continuously conversion state)
  // Byte2: 0000 0000 Conversion rate 0.25Hz, Alert disabled, Normal mode operation
  write(TMP112_CONFIGURATION_REGISTER, 0x60, 0x00);
  read(TMP112_CONFIGURATION_REGISTER, &byte1, &byte2);  
  
  if ((byte1 & 0x7F) == 0x60)
  {
    sensorOk = TRUE;
  }
  else
  {
    sensorOk = FALSE; 
  }
  
  return sensorOk;
}

/*---------------------------------------------------------------------------
* Description of function. Optional verbose description.
*-------------------------------------------------------------------------*/
void readTemperature(int8 *pTemperature, uint16 *pTemperatureRaw)
{
  uint8 byte1, byte2;

  read(TMP112_TEMPERATURE_REGISTER, &byte1, &byte2);
  
  // TESTCODE:
  // -25.0
  //  byte1 = 0xE7;
  //  byte2 = 0x00;

  // -0,25
  //  byte1 = 0xFF;
  //  byte2 = 0xC0;

  *pTemperatureRaw = (uint16)((uint16)byte1 << 8 ) + (uint16)byte2;
  *pTemperature = byte1;
}


/*---------------------------------------------------------------------------
 * Description of function. Optional verbose description.
 *-------------------------------------------------------------------------*/
static void write(uint8 reg, uint8 byte1, uint8 byte2)
{
  i2c_start();                        

  i2c_txByte((TMP112_ADDRESS << 1)  & ~0x01); // [ADDR] + R/W bit = 0 Write operation

  _NOP();                         
  _NOP();                          

  i2c_txByte(reg);                           

  _NOP();                         
  _NOP();                          
  
  i2c_txByte(byte1);
  
  _NOP();                         
  _NOP();                          

  i2c_txByte(byte2);

  i2c_stop(); 
}

/*---------------------------------------------------------------------------
 * Description of function. Optional verbose description.
 *-------------------------------------------------------------------------*/
static void read(uint8 reg, uint8 *pByte1, uint8 *pByte2)
{
  i2c_start();                        
  i2c_txByte((TMP112_ADDRESS << 1)  & ~0x01); // [ADDR] + R/W bit = 0 Write operation

  // Write pointer register in TMP112
  i2c_txByte(reg);                           
  i2c_stop();

  _NOP();                         
  _NOP();  

  i2c_start();
  i2c_txByte((TMP112_ADDRESS << 1) | 0x01); // [ADDR] + R/W bit = 1 Read operation

  // Read temperature register (2 bytes)
  *pByte1 = i2c_rxByte(TRUE);
  *pByte2 = i2c_rxByte(TRUE);

  i2c_stop();  
}


