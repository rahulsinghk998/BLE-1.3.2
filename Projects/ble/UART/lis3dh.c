/*===========================================================================
 * DEFINES
 *=========================================================================*/
#define LIS3DH_ADDRESS          (0x19)

// Registers
#define LIS3DH_STATUS_REG_AUX   (0x07)
#define LIS3DH_OUT_ADC1_L       (0x08)
#define LIS3DH_OUT_ADC1_H       (0x09)
#define LIS3DH_OUT_ADC2_L       (0x0A)
#define LIS3DH_OUT_ADC2_H       (0x0B)
#define LIS3DH_OUT_ADC3_L       (0x0C)
#define LIS3DH_OUT_ADC3_H       (0x0D)
#define LIS3DH_INT_COUNTER_REG  (0x0E)
#define LIS3DH_WHO_AM_I         (0x0F)
#define LIS3DH_TEMP_CONF_REG    (0x1F)
#define LIS3DH_CTRL_REG1        (0x20)
#define LIS3DH_CTRL_REG2        (0x21)
#define LIS3DH_CTRL_REG3        (0x22)
#define LIS3DH_CTRL_REG4        (0x23)
#define LIS3DH_CTRL_REG5        (0x24)
#define LIS3DH_CTRL_REG6        (0x25)
#define LIS3DH_REFERENCE        (0x26)
#define LIS3DH_STATUS_REG       (0x27)
#define LIS3DH_OUT_X_L          (0x28)
#define LIS3DH_OUT_X_H          (0x29)
#define LIS3DH_OUT_Y_L          (0x2A)
#define LIS3DH_OUT_Y_H          (0x2B)
#define LIS3DH_OUT_Z_L          (0x2C)
#define LIS3DH_OUT_Z_H          (0x2D)
#define LIS3DH_FIFO_CTRL_REG    (0x2E)
#define LIS3DH_FIFO_SRC_REG     (0x2F)
#define LIS3DH_INT1_CFG         (0x30)
#define LIS3DH_INT1_SRC         (0x31)
#define LIS3DH_INT1_THS         (0x32)
#define LIS3DH_INT1_DURATION    (0x33)
#define LIS3DH_CLICK_CFG        (0x38)
#define LIS3DH_CLICK_SRC        (0x39)
#define LIS3DH_CLICK_THS        (0x3A)
#define LIS3DH_TIME_LIMIT       (0x3B)
#define LIS3DH_TIME_LATENCY     (0x3C)
#define LIS3DH_TIME_WINDOW      (0x3D)
#define LIS3DH_ACT_THS          (0x3E)
#define LIS3DH_INACT_DUR        (0x3F)

// Interrupt line is connected to P1.3
#define LIS3DH_INT1_PORT   P1
#define LIS3DH_INT1_BIT    BV(3)
#define LIS3DH_INT1_SEL    P1SEL
#define LIS3DH_INT1_DIR    P1DIR


#define LIS3DH_INT1_INTERRUPT_ENABLE        IEN2
#define LIS3DH_INT1_PORT_INTERRUPT_MASK     P1IEN
#define LIS3DH_INT1_INTERRUPT_CONTROL       P1CTL
#define LIS3DH_INT1_INTERRUPT_STATUS_FLAG   P1IFG

#define LIS3DH_INT1_IENBIT   BV(3) 
#define LIS3DH_INT1_ICTLBIT  BV(1) 
#define LIS3DH_PORT1_INT_IENBIT   BV(4) 

#define LIS3DH_INTERRUPT_EVT            (1 << 0)
//#define LIS3DH_WAKEUP_TIMEOUT_EVT       (1 << 1)

#define LIS3DH_CONFIG_REGISTER             (0)
#define LIS3DH_CONFIG_VALUE                (1)
#define LIS3DH_CONFIG_SIZE                 (sizeof(config)/2)      

#define CTRL_REG1_1HZ_LOW_POWER_ENABLED_XYZ_ENABLED (0x1F)
#define CTRL_REG1_10HZ_LOW_POWER_ENABLED_XYZ_ENABLED (0x2F)

#ifndef LIS3DH_DEFAULT_LIS3DH_CTRL_REG1
  #define LIS3DH_DEFAULT_LIS3DH_CTRL_REG1 CTRL_REG1_10HZ_LOW_POWER_ENABLED_XYZ_ENABLED
#endif

#ifndef LIS3DH_DEFAULT_LIS3DH_WAKEUP_THRESHOLD
  #define LIS3DH_DEFAULT_LIS3DH_WAKEUP_THRESHOLD (0x10)
#endif

static void _NOP(void){asm("NOP");}
static void write( uint8 reg, uint8 val);
static uint8 read(uint8 reg);

const uint8 config[][2] =
{
  {LIS3DH_CTRL_REG1, LIS3DH_DEFAULT_LIS3DH_CTRL_REG1},
  {LIS3DH_CTRL_REG2, 0xC1},     // High pass, auto reset on A01
  {LIS3DH_CTRL_REG3, 0xC0},     // AOI1 and Click Interrupt on INT1   
  {LIS3DH_CTRL_REG4, 0x00},     // Full Scale = 2g
  {LIS3DH_CTRL_REG5, 0x08},     // Interrupt latched 
  {LIS3DH_CTRL_REG6, 0x02},     // Interrupt active low   
  {LIS3DH_INT1_DURATION, 0x00}, // Duration = 0
  {LIS3DH_INT1_CFG, 0x2A},      // Enable X and Y and Z    
  {LIS3DH_INT1_THS,  LIS3DH_DEFAULT_LIS3DH_WAKEUP_THRESHOLD}      // Threshold
};


