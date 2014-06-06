/*******************************************************************************
  Filename:    t3_free.c

  Description: Runs Timer 3 in free running mode. LED1 (green LED on CC2544Dongle) 
               is toggled by a Timer overflow interrupt. 

*******************************************************************************/

/*******************************************************************************
* INCLUDES
*/
#include <hal_types.h>
// Include Name definitions of individual bits and bit-fields in the CC254x device registers.
#include <ioCC254x_bitdef.h>
// Include device specific file
#if (chip==2541)
#include "ioCC2541.h"
#elif (chip==2543)
#include "ioCC2543.h"
#elif (chip==2544)
#include "ioCC2544.h"
#elif (chip==2545)
#include "ioCC2545.h"
#else
#error "Chip not supported!"
#endif


/*******************************************************************************
* CONSTANTS
*/


/*******************************************************************************
* LOCAL VARIABLES
*/


/*******************************************************************************
* LOCAL FUNCTIONS
*/


/*******************************************************************************
* @fn      t3_isr
*
* @brief   Interrupt handler for Timer 3 overflow interrupts. Toggles LED1.
*          Interrupts from Timer 3 are level triggered, so the module
*          interrupt flag is cleared prior to the CPU interrupt flag.
*
* @param   void
*
* @return  void
*
*******************************************************************************/
#pragma vector = T3_VECTOR
__interrupt void t3_isr(void)
{
    // Clears the module interrupt flag.
    T3OVFIF = 0;

    // Toggles LED1 on SmartRF05EB or CC2544Dongle.
#if (chip==2541 || chip==2543 || chip==2545)
    P1_0 ^= 1;  // Toggle SRF05EB LED1.
#elif (chip==2544)
    P0_2 ^= 1;  // Toggle GREEN LED1 on CC2544Dongle.
#endif

    // Clears the CPU interrupt flag.
    T3IF = 0;
}

/*******************************************************************************
* @fn          main
*
* @brief       LED1 and Timer 3 are both initialized. The rest of the program
*              is executed by interrupts from Timer 3. The interrupts toggle
*              LED1.
*
* @param       void
*
* @return      0
*******************************************************************************/
int main(void)
{
    /***************************************************************************
     * Setup clock & frequency
     * 
     * Configures the Timer tick speed setting, resulting in a Timer tick
     * frequency of 250 kHz.
     */
    CLKCONCMD = (CLKCONCMD & ~CLKCON_TICKSPD) | CLKCON_TICKSPD_250K;
  
  
    /***************************************************************************
     * Setup I/O
     * 
     * Select P1_0 direction to output
     */ 
#if (chip==2541 || chip==2543 || chip==2545)
    // LED1 as GPIO.
    P1SEL &= ~BIT0;            
    // Set LED1 as output.
    P1DIR |= BIT0;
#elif (chip==2544)
    // LED1 (GREEN) as GPIO.
    P0SEL1 &= ~P0SEL1_SELP0_2;
    // Set LED1 (GREEN) as output.
    PDIR |= PDIR_DIRP0_2;
#endif

  
    /***************************************************************************
     * Setup interrupt
     *
     * Enables global interrupts (IEN0.EA = 1) and interrupts from Timer 3
     * (IEN1.T3IE = 1).
     */
    EA = 1;
    T3IE = 1;
  
  
    /***************************************************************************
     * Timer 3 setup 
     *
     * Timer 3 control (T3CTL) configuration:
     * - Prescaler divider value: 128.
     * - Free running mode.
     * - Overflow interrupt enabled.
     * The Timer is also cleared and started.
     */
    T3CTL = T3CTL_DIV_128 | T3CTL_MODE_FREERUN | T3CTL_OVFIM | T3CTL_CLR | T3CTL_START;

    // Infinite loop.
    while (1);

}

/*******************************************************************************
  Copyright 2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
*******************************************************************************/
