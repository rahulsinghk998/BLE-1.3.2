/**************************************************************************************************
 *                                        - ioCC2544.h -
 *
 * Header file with definitions for the Texas Instruments CC2544 low-power System-on-Chip: 
 * an 8051-based MCU with 2.4 GHz RF transceiver, USB, and up to 32 kB FLASH.
 *
 * This file supports IAR, Keil and SDCC compilers.
 *
 **************************************************************************************************
 */

#ifndef IOCC2544_H
#define IOCC2544_H

/* ------------------------------------------------------------------------------------------------
 *                                      Compiler Abstraction
 * ------------------------------------------------------------------------------------------------
 */
#ifdef __IAR_SYSTEMS_ICC__
#pragma language=extended
#define SFR(name,addr)   __sfr   __no_init  volatile  unsigned char  name @ addr;
#define SFRBIT(name, addr, bit7, bit6, bit5, bit4, bit3, bit2, bit1, bit0) \
__sfr __no_init volatile union \
{                              \
  unsigned char name;          \
  struct {                     \
    unsigned char bit0 : 1;    \
    unsigned char bit1 : 1;    \
    unsigned char bit2 : 1;    \
    unsigned char bit3 : 1;    \
    unsigned char bit4 : 1;    \
    unsigned char bit5 : 1;    \
    unsigned char bit6 : 1;    \
    unsigned char bit7 : 1;    \
  };                           \
} @ addr;
#define SBIT(name,addr) /* not in use for IAR C Compiler */
#define XREG(addr)       ((unsigned char volatile __xdata *) 0)[addr]
#define PXREG(addr)      ((unsigned char volatile __xdata *) addr)
#define VECT(num,addr)   addr

#elif defined __IAR_SYSTEMS_ASM__
#define SFR(name,addr)   name  DEFINE  addr
SFRBITMACRO MACRO t, addr, bit7 , bit6, bit5, bit4, bit3, bit2, bit1, bit0
t    DEFINE addr
bit7 DEFINE addr.7
bit6 DEFINE addr.6
bit5 DEFINE addr.5
bit4 DEFINE addr.4    ;; NB: do not modify indentation of this macro
bit3 DEFINE addr.3
bit2 DEFINE addr.2
bit1 DEFINE addr.1
bit0 DEFINE addr.0
            ENDM
#define SFRBIT(name, addr, bit7, bit6, bit5, bit4, bit3, bit2, bit1, bit0) \
    SFRBITMACRO <name>, <addr>, <bit7>, <bit6>, <bit5>, <bit4>, <bit3>, <bit2>, <bit1>, <bit0>
#define SBIT(name,addr)  name  DEFINE  addr
#define XREG(addr)       addr
#define PXREG(addr)      addr
#define VECT(num,addr)   addr
/* IAR assembler uses some predefined registers. The following prevents name collisions. */
#define SP   SPx
#define ACC  ACCx
#define B    Bx
#define PSW  PSWx
#define CY   CYx
#define AC   ACx
#define F0   F0x
#define RS1  RS1x
#define RS0  RS0x
#define OV   OVx
#define P    Px

#elif defined __KEIL__
#define SFR(name,addr)   sfr   name  =  addr;
#define SFRBIT(name, addr, bit7, bit6, bit5, bit4, bit3, bit2, bit1, bit0) \
sfr name = addr; \
sbit bit7 = addr^7; \
sbit bit6 = addr^6; \
sbit bit5 = addr^5; \
sbit bit4 = addr^4; \
sbit bit3 = addr^3; \
sbit bit2 = addr^2; \
sbit bit1 = addr^1; \
sbit bit0 = addr^0;
#define SBIT(name,addr)  sbit  name  =  addr;
#define VECT(num,addr)   num

#ifdef __C51__
#define XREG(addr)       ((unsigned char volatile xdata *) 0)[addr]
#define PXREG(addr)      ((unsigned char volatile xdata *) addr)
#elif defined __AX51__ || defined __A51__
#define XREG(addr)  addr
#define PXREG(addr) addr
#else
#error "Unknown Keil compiler."
#endif

#elif defined SDCC
#define SFR(name, addr)        __sfr   __at(addr)                      name;
#define SBIT(name, addr, bit)  __sbit  __at(addr+bit)                  name;
#define SFRBIT(name, addr, bit7, bit6, bit5, bit4, bit3, bit2, bit1, bit0) \
__sfr __at(addr) name; \
__sbit __at(addr+7) bit7; \
__sbit __at(addr+6) bit6; \
__sbit __at(addr+5) bit5; \
__sbit __at(addr+4) bit4; \
__sbit __at(addr+3) bit3; \
__sbit __at(addr+2) bit2; \
__sbit __at(addr+1) bit1; \
__sbit __at(addr+0) bit0;
#define XREG(addr)       ((__xdata volatile unsigned char *) 0)[addr]
#define PXREG(addr)      ((__xdata volatile unsigned char *) addr)
#define VECT(num,addr)   num

#else
#error "Unrecognized compiler."
#endif


/* ------------------------------------------------------------------------------------------------
 *                                        Interrupt Vectors
 * ------------------------------------------------------------------------------------------------
 */
#define  RFERR_VECTOR   VECT(  0, 0x03 )   /*  RF TX FIFO Underflow and RX FIFO Overflow   */
#define  ADC_VECTOR     VECT(  1, 0x0B )   /*  ADC End of Conversion                       */
#define  URX0_VECTOR    VECT(  2, 0x13 )   /*  USART0 RX Complete                          */
#define  URX1_VECTOR    VECT(  3, 0x1B )   /*  USART1 RX Complete                          */
#define  ENC_VECTOR     VECT(  4, 0x23 )   /*  AES Encryption/Decryption Complete          */
#define  ST_VECTOR      VECT(  5, 0x2B )   /*  Sleep Timer Compare                         */
#define  P2INT_VECTOR   VECT(  6, 0x33 )   /*  Port 2 Inputs and USB                       */
#define  UTX0_VECTOR    VECT(  7, 0x3B )   /*  USART0 TX Complete                          */
#define  DMA_VECTOR     VECT(  8, 0x43 )   /*  DMA Transfer Complete                       */
#define  T1_VECTOR      VECT(  9, 0x4B )   /*  Timer 1 (16-bit) Capture/Compare/Overflow   */
#define  T2_VECTOR      VECT( 10, 0x53 )   /*  Timer 2 (MAC Timer)                         */
#define  T3_VECTOR      VECT( 11, 0x5B )   /*  Timer 3 (8-bit) Capture/Compare/Overflow    */
#define  T4_VECTOR      VECT( 12, 0x63 )   /*  Timer 4 (8-bit) Capture/Compare/Overflow    */
#define  P0INT_VECTOR   VECT( 13, 0x6B )   /*  Port 0 Inputs                               */
#define  UTX1_VECTOR    VECT( 14, 0x73 )   /*  USART1 TX Complete                          */
#define  P1INT_VECTOR   VECT( 15, 0x7B )   /*  Port 1 Inputs                               */
#define  RF_VECTOR      VECT( 16, 0x83 )   /*  RF General Interrupts                       */
#define  WDT_VECTOR     VECT( 17, 0x8B )   /*  Watchdog Overflow in Timer Mode             */

/* ------------------------------------------------------------------------------------------------
 *                                     Interrupt Alias
 * ------------------------------------------------------------------------------------------------
 */
#define  USB_VECTOR     P2INT_VECTOR /*  USB Interrupt vector, alias for P2INT_VECTOR        */


/* ------------------------------------------------------------------------------------------------
 *                                      SFR Bit Alias
 * ------------------------------------------------------------------------------------------------
 */
#define  USBIF     P2IF              /*  USB Interrupt Flag                                  */
/*       USBIE     P2IE              ,   not in a bit addressable register                   */

/* ------------------------------------------------------------------------------------------------
 *                                            SFRs
 * ------------------------------------------------------------------------------------------------
 */

/*
 *   SFRs with an address ending with 0 or 8 are bit accessible.
 *   They are defined with the SFRBIT() macro that sets the name of each bit.
 */

/* Port 0                                                                           */
SFRBIT( P0      ,  0x80, P0_7, P0_6, P0_5, P0_4, P0_3, P0_2, P0_1, P0_0 )
SFR(  SP        ,  0x81  )   /*  Stack Pointer                                      */
SFR(  DPL0      ,  0x82  )   /*  Data Pointer 0 Low Byte                            */
SFR(  DPH0      ,  0x83  )   /*  Data Pointer 0 High Byte                           */
SFR(  DPL1      ,  0x84  )   /*  Data Pointer 1 Low Byte                            */
SFR(  DPH1      ,  0x85  )   /*  Data Pointer 1 High Byte                           */
SFR(  U0CSR     ,  0x86  )   /*  USART 0 Control and Status                         */
SFR(  PCON      ,  0x87  )   /*  Power Mode Control                                 */

/* Interrupt Flags                                                                  */
SFRBIT( TCON    ,  0x88, URX1IF, _TCON6, ADCIF, _TCON4, URX0IF, IT1, RFERRIF, IT0 )
SFR(  P0IFG     ,  0x89  )   /*  Port 0 Interrupt Status Flag                       */
SFR(  P1IFG     ,  0x8A  )   /*  Port 1 Interrupt Status Flag                       */
SFR(  P2IFG     ,  0x8B  )   /*  Port 2 Interrupt Status Flag                       */
SFR(  PICTL     ,  0x8C  )   /*  Port Interrupt Control                             */
SFR(  P1IEN     ,  0x8D  )   /*  Port 1 Interrupt Mask                              */
SFR(  _SFR8E    ,  0x8E  )   /*  not used                                           */
SFR(  PINP      ,  0x8F  )   /*  Port Input Mode                                    */

/* Port 1                                                                           */
SFRBIT( P1      ,  0x90, P1_7, P1_6, P1_5, P1_4, P1_3, P1_2, P1_1, P1_0 )
SFR(  RFIRQF1   ,  0x91  )   /*  RF Interrupt Flags MSB                             */
SFR(  DPS       ,  0x92  )   /*  Data Pointer Select                                */
SFR(  MPAGE     ,  0x93  )   /*  Memory Page Select                                 */
SFR(  T2CTRL    ,  0x94  )   /*  Timer2 Control Register                            */
SFR(  ST0       ,  0x95  )   /*  Sleep Timer 0                                      */
SFR(  ST1       ,  0x96  )   /*  Sleep Timer 1                                      */
SFR(  ST2       ,  0x97  )   /*  Sleep Timer 2                                      */

/*  Interrupt Flags 2                                                               */
SFRBIT( S0CON   ,  0x98, _S0CON7, _S0CON6, _S0CON5, _S0CON4, _S0CON3, _S0CON2, ENCIF_1, ENCIF_0 )
SFR(  _SFR99    ,  0x99  )   /*  reserved                                           */
SFR(  IEN2      ,  0x9A  )   /*  Interrupt Enable 2                                 */
SFR(  S1CON     ,  0x9B  )   /*  Interrupt Flags 3                                  */
SFR(  T2CSPCFG  ,  0x9C  )   /*  Timer2 CSP Interface Configuration (legacy name)   */
SFR(  T2EVTCFG  ,  0x9C  )   /*  Timer2 Event Output Configuration                  */
SFR(  SLEEPSTA  ,  0x9D  )   /*  Sleep Status                                       */
SFR(  CLKCONSTA ,  0x9E  )   /*  Clock Control Status                               */
SFR(  FMAP      ,  0x9F  )   /*  Flash Bank Map                                     */

/* Port 2                                                                           */
SFRBIT( P2      ,  0xA0, _P2_7, _P2_6, _P2_5, P2_4, P2_3, P2_2, P2_1, P2_0 )
SFR(  T2IRQF    ,  0xA1  )   /* Timer2 Interrupt Flags                              */
SFR(  T2M0      ,  0xA2  )   /* Timer2 Multiplexed Register 0                       */
SFR(  T2M1      ,  0xA3  )   /* Timer2 Multiplexed Register 1                       */
SFR(  T2MOVF0   ,  0xA4  )   /* Timer2 Multiplexed Overflow Register 0              */
SFR(  T2MOVF1   ,  0xA5  )   /* Timer2 Multiplexed Overflow Register 1              */
SFR(  T2MOVF2   ,  0xA6  )   /* Timer2 Multiplexed Overflow Register 2              */
SFR(  T2IRQM    ,  0xA7  )   /* Timer2 Interrupt Mask                               */

/* Interrupt Enable 0                                                               */
SFRBIT( IEN0    ,  0xA8, EA, _IEN06, STIE, ENCIE, URX1IE, URX0IE, ADCIE, RFERRIE )
SFR(  IP0       ,  0xA9  )   /*  Interrupt Priority 0                               */
SFR(  _SFRAA    ,  0xAA  )   /*  not used                                           */
SFR(  P0IEN     ,  0xAB  )   /*  Port 0 Interrupt Mask                              */
SFR(  P2IEN     ,  0xAC  )   /*  Port 2 Interrupt Mask                              */
SFR(  STLOAD    ,  0xAD  )   /*  Sleep Timer Load Status                            */
SFR(  _SFRAE    ,  0xAE  )   /*  reserved                                           */
SFR(  T1STAT    ,  0xAF  )   /*  Timer 1 Status                                     */

SFR(  _SFRB0    ,  0xB0  )   /*  not used                                           */
SFR(  ENCDI     ,  0xB1  )   /*  Encryption/Decryption Input Data                   */
SFR(  ENCDO     ,  0xB2  )   /*  Encryption/Decryption Output Data                  */
SFR(  ENCCS     ,  0xB3  )   /*  Encryption/Decryption Control and Status           */
SFR(  ADCCON1   ,  0xB4  )   /*  ADC Control 1                                      */
SFR(  _SFRB5    ,  0xB5  )   /*  reserved                                           */
SFR(  _SFRB6    ,  0xB6  )   /*  reserved                                           */
SFR(  _SFRB7    ,  0xB7  )   /*  reserved                                           */

/*  Interrupt Enable 1                                                              */
SFRBIT( IEN1    ,  0xB8, _IEN17, _IEN16, P0IE, T4IE, T3IE, T2IE, T1IE, DMAIE )
SFR(  IP1       ,  0xB9  )   /*  Interrupt Priority 1                               */
SFR(  _SFRBA    ,  0xBA  )   /*  reserved                                           */
SFR(  _SFRBB    ,  0xBB  )   /*  reserved                                           */
SFR(  RNDL      ,  0xBC  )   /*  Random Number Generator Low Byte                   */
SFR(  RNDH      ,  0xBD  )   /*  Random Number Generator High Byte                  */
SFR(  SLEEPCMD  ,  0xBE  )   /*  Sleep Mode Control Command                         */
SFR(  RFERRF    ,  0xBF  )   /*  RF Error Interrupt Flags                           */

/*  Interrupt Flags 4                                                               */
SFRBIT( IRCON   ,  0xC0, STIF, _IRCON6, P0IF, T4IF, T3IF, T2IF, T1IF, DMAIF )
SFR(  U0DBUF    ,  0xC1  )   /*  USART 0 Receive/Transmit Data Buffer               */
SFR(  U0BAUD    ,  0xC2  )   /*  USART 0 Baud Rate Control                          */
SFR(  T2MSEL    ,  0xC3  )   /*  Timer2 Multiplex Select                            */
SFR(  U0UCR     ,  0xC4  )   /*  USART 0 UART Control                               */
SFR(  U0GCR     ,  0xC5  )   /*  USART 0 Generic Control                            */
SFR(  CLKCONCMD ,  0xC6  )   /*  Clock Control Command                              */
SFR(  MEMCTR    ,  0xC7  )   /*  Memory System Control                              */

SFR(  _SFRC8    ,  0xC8  )   /*  not used                                           */
SFR(  WDCTL     ,  0xC9  )   /*  Watchdog Timer Control                             */
SFR(  T3CNT     ,  0xCA  )   /*  Timer 3 Counter                                    */
SFR(  T3CTL     ,  0xCB  )   /*  Timer 3 Control                                    */
SFR(  T3CCTL0   ,  0xCC  )   /*  Timer 3 Channel 0 Capture/Compare Control          */
SFR(  T3CC0     ,  0xCD  )   /*  Timer 3 Channel 0 Capture/Compare Value            */
SFR(  T3CCTL1   ,  0xCE  )   /*  Timer 3 Channel 1 Capture/Compare Control          */
SFR(  T3CC1     ,  0xCF  )   /*  Timer 3 Channel 1 Capture/Compare Value            */

 /*  Program Status Word                                                            */
SFRBIT( PSW     ,  0xD0, CY, AC, F0, RS1, RS0, OV, F1, P )
SFR(  DMAIRQ    ,  0xD1  )   /*  DMA Interrupt Flag                                 */
SFR(  DMA1CFGL  ,  0xD2  )   /*  DMA Channel 1 Configuration Address Low Byte       */
SFR(  DMA1CFGH  ,  0xD3  )   /*  DMA Channel 1 Configuration Address High Byte      */
SFR(  DMA0CFGL  ,  0xD4  )   /*  DMA Channel 0 Configuration Address Low Byte       */
SFR(  DMA0CFGH  ,  0xD5  )   /*  DMA Channel 0 Configuration Address High Byte      */
SFR(  DMAARM    ,  0xD6  )   /*  DMA Channel Arm                                    */
SFR(  DMAREQ    ,  0xD7  )   /*  DMA Channel Start Request and Status               */

/*  Timers 1/3/4 Interrupt Mask/Flag                                                */
SFRBIT( TIMIF   ,  0xD8 , _TIMIF7, T1OVFIM, T4CH1IF, T4CH0IF, T4OVFIF, T3CH1IF, T3CH0IF, T3OVFIF )
SFR(  RFD       ,  0xD9  )   /*  RF Data                                            */
SFR(  T1CC0L    ,  0xDA  )   /*  Timer 1 Channel 0 Capture/Compare Value Low Byte   */
SFR(  T1CC0H    ,  0xDB  )   /*  Timer 1 Channel 0 Capture/Compare Value High Byte  */
SFR(  T1CC1L    ,  0xDC  )   /*  Timer 1 Channel 1 Capture/Compare Value Low Byte   */
SFR(  T1CC1H    ,  0xDD  )   /*  Timer 1 Channel 1 Capture/Compare Value High Byte  */
SFR(  T1CC2L    ,  0xDE  )   /*  Timer 1 Channel 2 Capture/Compare Value Low Byte   */
SFR(  T1CC2H    ,  0xDF  )   /*  Timer 1 Channel 2 Capture/Compare Value High Byte  */

SFR(  ACC       ,  0xE0  )   /*  Accumulator                                        */
SFR(  RFST      ,  0xE1  )   /*  RF Command Strobe                                  */
SFR(  T1CNTL    ,  0xE2  )   /*  Timer 1 Counter Low                                */
SFR(  T1CNTH    ,  0xE3  )   /*  Timer 1 Counter High                               */
SFR(  T1CTL     ,  0xE4  )   /*  Timer 1 Control And Status                         */
SFR(  T1CCTL0   ,  0xE5  )   /*  Timer 1 Channel 0 Capture/Compare Control          */
SFR(  T1CCTL1   ,  0xE6  )   /*  Timer 1 Channel 1 Capture/Compare Control          */
SFR(  T1CCTL2   ,  0xE7  )   /*  Timer 1 Channel 2 Capture/Compare Control          */

/*  Interrupt Flags 5                                                               */
SFRBIT(  IRCON2 ,  0xE8, _IRCON27, _IRCON26, _IRCON25, WDTIF, P1IF, UTX1IF, UTX0IF, P2IF )
SFR(  RFIRQF0   ,  0xE9  )   /*  RF Interrupt Flags LSB                             */
SFR(  T4CNT     ,  0xEA  )   /*  Timer 4 Counter                                    */
SFR(  T4CTL     ,  0xEB  )   /*  Timer 4 Control                                    */
SFR(  T4CCTL0   ,  0xEC  )   /*  Timer 4 Channel 0 Capture/Compare Control          */
SFR(  T4CC0     ,  0xED  )   /*  Timer 4 Channel 0 Capture/Compare Value            */
SFR(  T4CCTL1   ,  0xEE  )   /*  Timer 4 Channel 1 Capture/Compare Control          */
SFR(  T4CC1     ,  0xEF  )   /*  Timer 4 Channel 1 Capture/Compare Value            */

SFR(  B         ,  0xF0  )   /*  B Register                                         */
SFR(  _SFRF1    ,  0xF1  )   /*  reserved                                           */
SFR(  _SFRF2    ,  0xF2  )   /*  reserved                                           */
SFR(  P0SEL0    ,  0xF3  )   /*  Port 0 Function Select                             */
SFR(  P0SEL1    ,  0xF4  )   /*  Port 0 Function Select                             */
SFR(  P1SEL0    ,  0xF5  )   /*  Port 1 Function Select                             */
SFR(  P1SEL1    ,  0xF6  )   /*  Port 1 Function Select                             */
SFR(  PPULL     ,  0xF7  )   /*  Pull Up, pull down                                 */

SFR(  _SFRF8    ,  0xF8  )   /*  reserved                                           */
SFR(  _SFRF9    ,  0xF9  )   /*  reserved                                           */
SFR(  _SFRFA    ,  0xFA  )   /*  reserved                                           */
SFR(  _SFRFB    ,  0xFB  )   /*  reserved                                           */
SFR(  _SFRFC    ,  0xFC  )   /*  reserved                                           */
SFR(  PDIR      ,  0xFD  )   /*  Port Direction                                     */
SFR(  _SFRFE    ,  0xFE  )   /*  reserved                                           */
SFR(  _SFRFF    ,  0xFF  )   /*  reserved                                           */


/* ------------------------------------------------------------------------------------------------
 *                                       Xdata Radio Registers
 * ------------------------------------------------------------------------------------------------
 */
/* RF core ram */
#define RFCORE_RAM      PXREG( 0x6000 )
#define RFCORE_RAM_SZ   384

/* RF core ram register/general memory page */
#define RFCORE_RAM_PAGE PXREG( 0x6000 )
#define RFCORE_RAM_PAGE_SZ 128

/* FIFO direct access */
#define RXFIFO          PXREG( 0x6080 )
#define RXFIFO_SZ       128
#define TXFIFO          PXREG( 0x6100 )
#define TXFIFO_SZ       128

/* Radio Control Registers */
#define FRMCTRL0       XREG(0x6180)
#define RFIRQM0        XREG(0x6181)
#define RFIRQM1        XREG(0x6182)
#define RFERRM         XREG(0x6183)
#define FREQCTRL       XREG(0x6184)
#define FREQTUNE       XREG(0x6185)
#define TXPOWER        XREG(0x6186)
#define TXCTRL         XREG(0x6187)
#define LLESTAT        XREG(0x6188)
#define _XREG6189      XREG(0x6189)
#define SEMAPHORE0     XREG(0x618A)
#define SEMAPHORE1     XREG(0x618B)
#define SEMAPHORE2     XREG(0x618C)
#define RFSTAT         XREG(0x618D)
#define RSSI           XREG(0x618E)
#define RFPSRND        XREG(0x618F)
#define MDMCTRL0       XREG(0x6190)
#define MDMCTRL1       XREG(0x6191)
#define MDMCTRL2       XREG(0x6192)
#define MDMCTRL3       XREG(0x6193)
#define SW_CONF        XREG(0x6194)
#define SW0            XREG(0x6195)
#define SW1            XREG(0x6196)
#define SW2            XREG(0x6197)
#define SW3            XREG(0x6198)
#define FREQEST        XREG(0x6199)
#define RXCTRL         XREG(0x619A)
#define FSCTRL         XREG(0x619B)
#define FSCAL0         XREG(0x619C)
#define FSCAL1         XREG(0x619D)
#define FSCAL2         XREG(0x619E)
#define FSCAL3         XREG(0x619F)
#define LNAGAIN        XREG(0x61A0)
#define AAFGAIN        XREG(0x61A1)
#define ADCTEST0       XREG(0x61A2)
#define ADCTEST1       XREG(0x61A3)
#define ADCTEST2       XREG(0x61A4)
#define MDMTEST0       XREG(0x61A5)
#define MDMTEST1       XREG(0x61A6)
#define ATEST          XREG(0x61A9)
#define RFC_OBS_CTRL0  XREG(0x61AE)
#define RFC_OBS_CTRL1  XREG(0x61AF)
#define RFC_OBS_CTRL2  XREG(0x61B0)
#define LLECTRL        XREG(0x61B1)
#define ACOMPQS        XREG(0x61B6)
#define TXFILTCFG      XREG(0x61BC)
#define RFRND          XREG(0x61BF)

/* FIFO and RF core RAM regsiters */
#define RFRAMCFG       XREG(0x61C0)
#define _XREG61C1      XREG(0x61C1)
#define _XREG61C2      XREG(0x61C2)
#define RFFDMA0        XREG(0x61C3)
#define RFFDMA1        XREG(0x61C4)
#define RFFSTATUS      XREG(0x61C5)
#define RFFCFG         XREG(0x61C6)

#define RFRXFLEN       XREG(0x61C8)
#define RFRXFTHRS      XREG(0x61C9)
#define RFRXFWR        XREG(0x61CA)
#define RFRXFRD        XREG(0x61CB)
#define RFRXFWP        XREG(0x61CC)
#define RFRXFRP        XREG(0x61CD)
#define RFRXFSWP       XREG(0x61CE)
#define RFRXFSRP       XREG(0x61CF)
#define RFTXFLEN       XREG(0x61D0)
#define RFTXFTHRS      XREG(0x61D1)
#define RFTXFWR        XREG(0x61D2)
#define RFTXFRD        XREG(0x61D3)
#define RFTXFWP        XREG(0x61D4)
#define RFTXFRP        XREG(0x61D5)
#define RFTXFSWP       XREG(0x61D6)
#define RFTXFSRP       XREG(0x61D7)

/* Bit Stream Processor Registers */
#define BSP_P0         XREG(0x61E0)
#define BSP_P1         XREG(0x61E1)
#define BSP_P2         XREG(0x61E2)
#define BSP_P3         XREG(0x61E3)
#define BSP_D0         XREG(0x61E4)
#define BSP_D1         XREG(0x61E5)
#define BSP_D2         XREG(0x61E6)
#define BSP_D3         XREG(0x61E7)
#define BSP_W          XREG(0x61E8)
#define BSP_MODE       XREG(0x61E9)
#define BSP_DATA       XREG(0x61EA)

#define SW4            XREG(0x61F8)
#define SW5            XREG(0x61F9)
#define SW6            XREG(0x61FA)
#define SW7            XREG(0x61FB)
#define DC_I_L         XREG(0x61FC)
#define DC_I_H         XREG(0x61FD)
#define DC_Q_L         XREG(0x61FE)
#define DC_Q_H         XREG(0x61FF)

/* ------------------------------------------------------------------------------------------------
 *                                       Xdata USB Registers
 * ------------------------------------------------------------------------------------------------
 */
#define  USBADDR     XREG( 0x6200 )  /*  Function Address                                    */
#define  USBPOW      XREG( 0x6201 )  /*  Power/Control Register                              */
#define  USBIIF      XREG( 0x6202 )  /*  IN Endpoints and EP0 Interrupt Flags                */
#define  _XREG6203   XREG( 0x6203 )  /*  Reserved                                            */
#define  USBOIF      XREG( 0x6204 )  /*  OUT Endpoints Interrupt Flags                       */
#define  _XREG6205   XREG( 0x6205 )  /*  Reserved                                            */
#define  USBCIF      XREG( 0x6206 )  /*  Common USB Interrupt Flags                          */
#define  USBIIE      XREG( 0x6207 )  /*  IN Endpoints and EP0 Interrupt Enable Mask          */
#define  _XREG6208   XREG( 0x6208 )  /*  Reserved                                            */
#define  USBOIE      XREG( 0x6209 )  /*  OUT Endpoints Interrupt Enable Mask                 */
#define  _XREG620A   XREG( 0x620A )  /*  Reserved                                            */
#define  USBCIE      XREG( 0x620B )  /*  Common USB Interrupt Enable Mask                    */
#define  USBFRML     XREG( 0x620C )  /*  Current Frame Number (Low byte)                     */
#define  USBFRMH     XREG( 0x620D )  /*  Current Frame Number (High byte)                    */
#define  USBINDEX    XREG( 0x620E )  /*  Selects current endpoint                            */
#define  USBCTRL     XREG( 0x620F )  /*  USB Control Register                                */
#define  USBMAXI     XREG( 0x6210 )  /*  Max. packet size for IN endpoint                    */
#define  USBCS0      XREG( 0x6211 )  /*  EP0 Control and Status               (USBINDEX = 0) */
#define  USBCSIL     XREG( 0x6211 )  /*  IN EP{1-5} Control and Status Low    (USBINDEX > 0) */
#define  USBCSIH     XREG( 0x6212 )  /*  IN EP{1-5} Control and Status High                  */
#define  USBMAXO     XREG( 0x6213 )  /*  Max. packet size for OUT endpoint                   */
#define  USBCSOL     XREG( 0x6214 )  /*  OUT EP{1-5} Control and Status Low                  */
#define  USBCSOH     XREG( 0x6215 )  /*  OUT EP{1-5} Control and Status High                 */
#define  USBCNT0     XREG( 0x6216 )  /*  Number of received bytes in EP0 FIFO (USBINDEX = 0) */ 
#define  USBCNTL     XREG( 0x6216 )  /*  Number of bytes in OUT FIFO Low      (USBINDEX > 0) */
#define  USBCNTH     XREG( 0x6217 )  /*  Number of bytes in OUT FIFO High                    */
#define  USBF0       XREG( 0x6220 )  /*  Endpoint 0 FIFO                                     */
#define  USBF1       XREG( 0x6222 )  /*  Endpoint 1 FIFO                                     */
#define  USBF2       XREG( 0x6224 )  /*  Endpoint 2 FIFO                                     */
#define  USBF3       XREG( 0x6226 )  /*  Endpoint 3 FIFO                                     */
#define  USBF4       XREG( 0x6228 )  /*  Endpoint 4 FIFO                                     */
#define  USBF5       XREG( 0x622A )  /*  Endpoint 5 FIFO                                     */

/* ------------------------------------------------------------------------------------------------
 *                                       Xdata Registers
 * ------------------------------------------------------------------------------------------------
 */

/* Observability Control */
#define OBSSEL0         XREG( 0x6243 )
#define OBSSEL1         XREG( 0x6244 )
#define OBSSEL2         XREG( 0x6245 )
#define OBSSEL3         XREG( 0x6246 )
#define OBSSEL4         XREG( 0x6247 )
#define OBSSEL5         XREG( 0x6248 )
#define TR0             XREG( 0x624B )  /*  Test Register 0                                     */

/* Chip Identification */
#define CHVER           XREG( 0x6249 )
#define CHIPID          XREG( 0x624A )

/* Debug Interface DMA Write to Flash */
#define DBGDATA         XREG( 0x6260 )

/*  Analog control register */
#define IVCTRL          XREG( 0x6265 )

/* Flash Controller */
#define FCTL            XREG( 0x6270 )
#define FADDRL          XREG( 0x6271 )
#define FADDRH          XREG( 0x6272 )
#define FWDATA          XREG( 0x6273 )
#define _XREG6274       XREG( 0x6274 )
#define _XREG6275       XREG( 0x6275 )

/* Chip Information */
#define CHIPINFO0       XREG( 0x6276 )
#define CHIPINFO1       XREG( 0x6277 )

/* IR Generation Control */
#define IRCTL           XREG( 0x6281 )

/* Clock Loss Detector */
#define CLD             XREG( 0x6290 )

/* Timer 1 Channels (only mapped as XREG) */
#define T1CCTL3         XREG( 0x62A3 )
#define T1CCTL4         XREG( 0x62A4 )
#define T1CC3L          XREG( 0x62AC )
#define T1CC3H          XREG( 0x62AD )
#define T1CC4L          XREG( 0x62AE )
#define T1CC4H          XREG( 0x62AF )
/* Definition which includes channels represented in SFR (additional XREG mapping of SFR) */
#define XX_T1CCTL0      XREG( 0x62A0 )
#define XX_T1CCTL1      XREG( 0x62A1 )
#define XX_T1CCTL2      XREG( 0x62A2 )
#define XX_T1CCTL3      XREG( 0x62A3 )
#define XX_T1CCTL4      XREG( 0x62A4 )
#define XX_T1CC0L       XREG( 0x62A6 )
#define XX_T1CC0H       XREG( 0x62A7 )
#define XX_T1CC1L       XREG( 0x62A8 )
#define XX_T1CC1H       XREG( 0x62A9 )
#define XX_T1CC2L       XREG( 0x62AA )
#define XX_T1CC2H       XREG( 0x62AB )
#define XX_T1CC3L       XREG( 0x62AC )
#define XX_T1CC3H       XREG( 0x62AD )
#define XX_T1CC4L       XREG( 0x62AE )
#define XX_T1CC4H       XREG( 0x62AF )
/* Pointers for array access */
#define P_T1CCTL        PXREG( 0x62A0 )
#define P_T1CC          PXREG( 0x62A6 )

/* Sleep Timer Capture Control */
#define STCC            XREG( 0x62B0 )
#define STCS            XREG( 0x62B1 )
#define STCV0           XREG( 0x62B2 )
#define STCV1           XREG( 0x62B3 )
#define STCV2           XREG( 0x62B4 )


/* ------------------------------------------------------------------------------------------------
 *                                      Xdata Mapped SFRs
 * ------------------------------------------------------------------------------------------------
 */

/*
 *   Most SFRs are also accessible through XDATA address space.  The register definitions for
 *   this type of access are listed below.  The register names are identical to the SFR names
 *   but with the prefix X_ to denote an XDATA register.
 *
 *   Some SFRs are not accessible through XDATA space.  For clarity, entries are included for these
 *   registers.  They have a prefix of _NA to denote "not available."
 *
 *   For register descriptions, refer to the actual SFR declartions elsewhere in this file.
 */

#define X_P0        XREG( 0x7080 )   /*  Port 0. NOTE! Read-only access from XREG.          */
#define _NA_SP      XREG( 0x7081 )
#define _NA_DPL0    XREG( 0x7082 )
#define _NA_DPH0    XREG( 0x7083 )
#define _NA_DPL1    XREG( 0x7084 )
#define _NA_DPH1    XREG( 0x7085 )
#define X_U0CSR     XREG( 0x7086 )   /*  USART 0 Control and Status                         */
#define _NA_PCON    XREG( 0x7087 )

#define _NA_TCON    XREG( 0x7088 )
#define X_P0IFG     XREG( 0x7089 )   /*  Port 0 Interrupt Status Flag                       */
#define X_P1IFG     XREG( 0x708A )   /*  Port 1 Interrupt Status Flag                       */
#define X_P2IFG     XREG( 0x708B )   /*  Port 2 Interrupt Status Flag                       */
#define X_PICTL     XREG( 0x708C )   /*  Port Interrupt Control                             */
#define X_P1IEN     XREG( 0x708D )   /*  Port 1 Interrupt Mask                              */
#define _NA_SFR8E   XREG( 0x708E )
#define X_PINP      XREG( 0x708F )   /*  Port 0 Input Mode                                  */

#define X_P1        XREG( 0x7090 )   /*  Port 1. NOTE! Read-only access from XREG.          */
#define X_RFIRQF1   XREG( 0x7091 )   /*  RF Interrupt Flags MSB                             */
#define _NA_DPS     XREG( 0x7092 )
#define X_MPAGE     XREG( 0x7093 )   /*  Memory Page Select                                 */
#define X_T2CTRL    XREG( 0x7094 )   /*  Timer2 Control Register                            */
#define X_ST0       XREG( 0x7095 )   /*  Sleep Timer 0                                      */
#define X_ST1       XREG( 0x7096 )   /*  Sleep Timer 1                                      */
#define X_ST2       XREG( 0x7097 )   /*  Sleep Timer 2                                      */

#define _NA_S0CON   XREG( 0x7098 )
#define _NA_SFR99   XREG( 0x7099 )
#define _NA_IEN2    XREG( 0x709A )
#define _NA_S1CON   XREG( 0x709B )
#define X_T2CSPCFG  XREG( 0x709C )   /*  Timer2 CSP Interface Configuration (legacy name)   */
#define X_T2EVTCFG  XREG( 0x709C )   /*  Timer2 Event Output Configuration                  */
#define X_SLEEPSTA  XREG( 0x709D )   /*  Sleep Status                                       */
#define X_CLKCONSTA XREG( 0x709E )   /*  Clock Control Status                               */
#define X_FMAP      XREG( 0x709F )   /*  Flash Bank Map                                     */

#define X_P2        XREG( 0x70A0 )   /*  Port 2. NOTE! Read-only access from XREG.          */
#define X_T2IRQF    XREG( 0x70A1 )   /*  Timer2 Interrupt Flags                             */
#define X_T2M0      XREG( 0x70A2 )   /*  Timer2 Multiplexed Register 0                      */
#define X_T2M1      XREG( 0x70A3 )   /*  Timer2 Multiplexed Register 1                      */
#define X_T2MOVF0   XREG( 0x70A4 )   /*  Timer2 Multiplexed Overflow Register 0             */
#define X_T2MOVF1   XREG( 0x70A5 )   /*  Timer2 Multiplexed Overflow Register 1             */
#define X_T2MOVF2   XREG( 0x70A6 )   /*  Timer2 Multiplexed Overflow Register 2             */
#define X_T2IRQM    XREG( 0x70A7 )   /*  Timer2 Interrupt Mask                              */

#define _NA_IEN0    XREG( 0x70A8 )
#define _NA_IP0     XREG( 0x70A9 )
#define _NA_SFRAA   XREG( 0x70AA )
#define X_P0IEN     XREG( 0x70AB )   /*                                                     */
#define X_P2IEN     XREG( 0x70AC )   /*                                                     */
#define X_STLOAD    XREG( 0x70AD )   /*  Sleep Timer Load Status                            */
#define _NA_SFRAE   XREG( 0x70AE )   /*                                                     */
#define X_T1STAT    XREG( 0x70AF )   /*                                                     */

#define _NA_SFRB0   XREG( 0x70A8 )
#define X_ENCDI     XREG( 0x70B1 )   /*  Encryption Input Data                              */
#define X_ENCDO     XREG( 0x70B2 )   /*  Encryption Output Data                             */
#define X_ENCCS     XREG( 0x70B3 )   /*  Encryption Control and Status                      */
#define X_ADCCON1   XREG( 0x70B4 )   /*  ADC Control 1                                      */
#define _NA_SFRB5   XREG( 0x70B5 )
#define _NA_SFRB6   XREG( 0x70B6 )
#define _NA_SFRB7   XREG( 0x70B7 )

#define _NA_IEN1    XREG( 0x70B8 )
#define _NA_IP1     XREG( 0x70B9 )
#define _NA_SFRBA   XREG( 0x70BA )
#define _NA_SFRBB   XREG( 0x70BB )
#define X_RNDL      XREG( 0x70BC )   /*  Random Register Low Byte                           */
#define X_RNDH      XREG( 0x70BD )   /*  Random Register High Byte                          */
#define X_SLEEPCMD  XREG( 0x70BE )   /*  Sleep Mode Command                                 */
#define X_RFERRF    XREG( 0x70BF )   /*  RF Error Interrupt Flags                           */

#define _NA_IRCON   XREG( 0x70C0 )
#define X_U0DBUF    XREG( 0x70C1 )   /*  USART 0 Receive/Transmit Data Buffer               */
#define X_U0BAUD    XREG( 0x70C2 )   /*  USART 0 Baud Rate Control                          */
#define X_T2MSEL    XREG( 0x70C3 )   /*  Timer2 Multiplex Select                            */
#define X_U0UCR     XREG( 0x70C4 )   /*  USART 0 UART Control                               */
#define X_U0GCR     XREG( 0x70C5 )   /*  USART 0 Generic Control                            */
#define X_CLKCONCMD XREG( 0x70C6 )   /*  Clock Control Command                              */
#define X_MEMCTR    XREG( 0x70C7 )   /*  Memory Arbiter Control                             */

#define _NA_SFRC8   XREG( 0x70C8 )
#define X_WDCTL     XREG( 0x70C9 )   /*  Watchdog Timer Control                             */
#define X_T3CNT     XREG( 0x70CA )   /*  Timer 3 Counter                                    */
#define X_T3CTL     XREG( 0x70CB )   /*  Timer 3 Control                                    */
#define X_T3CCTL0   XREG( 0x70CC )   /*  Timer 3 Channel 0 Capture/Compare Control          */
#define X_T3CC0     XREG( 0x70CD )   /*  Timer 3 Channel 0 Capture/Compare Value            */
#define X_T3CCTL1   XREG( 0x70CE )   /*  Timer 3 Channel 1 Capture/Compare Control          */
#define X_T3CC1     XREG( 0x70CF )   /*  Timer 3 Channel 1 Capture/Compare Value            */

#define _NA_PSW     XREG( 0x70D0 )
#define X_DMAIRQ    XREG( 0x70D1 )   /*  DMA Interrupt Flag                                 */
#define X_DMA1CFGL  XREG( 0x70D2 )   /*  DMA Channel 1-4 Configuration Address Low Byte     */
#define X_DMA1CFGH  XREG( 0x70D3 )   /*  DMA Channel 1-4 Configuration Address High Byte    */
#define X_DMA0CFGL  XREG( 0x70D4 )   /*  DMA Channel 0 Configuration Address Low Byte       */
#define X_DMA0CFGH  XREG( 0x70D5 )   /*  DMA Channel 0 Configuration Address High Byte      */
#define X_DMAARM    XREG( 0x70D6 )   /*  DMA Channel Arm                                    */
#define X_DMAREQ    XREG( 0x70D7 )   /*  DMA Channel Start Request and Status               */

#define X_TIMIF     XREG( 0x70D8 )   /*  Timers 1/3/4 Interrupt Mask/Flag                   */
#define X_RFD       XREG( 0x70D9 )   /*  RF Data                                            */
#define X_T1CC0L    XREG( 0x70DA )   /*  Timer 1 Channel 0 Capture/Compare Value Low Byte   */
#define X_T1CC0H    XREG( 0x70DB )   /*  Timer 1 Channel 0 Capture/Compare Value High Byte  */
#define X_T1CC1L    XREG( 0x70DC )   /*  Timer 1 Channel 1 Capture/Compare Value Low Byte   */
#define X_T1CC1H    XREG( 0x70DD )   /*  Timer 1 Channel 1 Capture/Compare Value High Byte  */
#define X_T1CC2L    XREG( 0x70DE )   /*  Timer 1 Channel 2 Capture/Compare Value Low Byte   */
#define X_T1CC2H    XREG( 0x70DF )   /*  Timer 1 Channel 2 Capture/Compare Value High Byte  */

#define _NA_ACC     XREG( 0x70E0 )
#define X_RFST      XREG( 0x70E1 )   /*  RF Strobe                                          */
#define X_T1CNTL    XREG( 0x70E2 )   /*  Timer 1 Counter Low                                */
#define X_T1CNTH    XREG( 0x70E3 )   /*  Timer 1 Counter High                               */
#define X_T1CTL     XREG( 0x70E4 )   /*  Timer 1 Control and Status                         */
#define X_T1CCTL0   XREG( 0x70E5 )   /*  Timer 1 Channel 0 Capture/Compare Control          */
#define X_T1CCTL1   XREG( 0x70E6 )   /*  Timer 1 Channel 1 Capture/Compare Control          */
#define X_T1CCTL2   XREG( 0x70E7 )   /*  Timer 1 Channel 2 Capture/Compare Control          */

#define _NA_IRCON2  XREG( 0x70E8 )
#define X_RFIRQF0   XREG( 0x70E9 )   /*  RF Interrupt Flags MSB                             */
#define X_T4CNT     XREG( 0x70EA )   /*  Timer 4 Counter                                    */
#define X_T4CTL     XREG( 0x70EB )   /*  Timer 4 Control                                    */
#define X_T4CCTL0   XREG( 0x70EC )   /*  Timer 4 Channel 0 Capture/Compare Control          */
#define X_T4CC0     XREG( 0x70ED )   /*  Timer 4 Channel 0 Capture/Compare Value            */
#define X_T4CCTL1   XREG( 0x70EE )   /*  Timer 4 Channel 1 Capture/Compare Control          */
#define X_T4CC1     XREG( 0x70EF )   /*  Timer 4 Channel 1 Capture/Compare Value            */

#define _NA_B       XREG( 0x70F0 )
#define _NA_SFRF1   XREG( 0x70F1 )
#define _NA_SFRF2   XREG( 0x70F2 )
#define X_P0SEL0    XREG( 0x70F3 )   /*  Port 0 Function Select                             */
#define X_P0SEL1    XREG( 0x70F4 )   /*  Port 0 Function Select                             */
#define X_P1SEL0    XREG( 0x70F5 )   /*  Port 1 Function Select                             */
#define X_P1SEL1    XREG( 0x70F6 )   /*  Port 1 Function Select                             */
#define X_PPULL     XREG( 0x70F7 )   /*  Port pull-up/pull-down                             */

#define _NA_SFRF8   XREG( 0x70F8 )
#define _NA_SFRF9   XREG( 0x70F9 )
#define _NA_SFRFA   XREG( 0x70FA )
#define _NA_SFRFB   XREG( 0x70FB )
#define _NA_SFRFC   XREG( 0x70FC )
#define X_PDIR      XREG( 0x70FD )    /*  Port  Direction                                   */
#define _NA_SFRFE   XREG( 0x70FE )
#define _NA_SFRFF   XREG( 0x70FF )

/* ------------------------------------------------------------------------------------------------
 *                                       Flash
 * ------------------------------------------------------------------------------------------------
 */

#define P_INFOPAGE  PXREG( 0x7800 )  /* Pointer to Start of Flash Information Page          */
#define P_XBANK     PXREG( 0x8000 )  /* Pointer to Start of Selectable Flash Bank (XBANK)   */

/* ------------------------------------------------------------------------------------------------
 */

#ifdef __IAR_SYSTEMS_ICC__
#pragma language=default
#endif

/**************************************************************************************************
 */
#endif
