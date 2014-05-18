/*******************************************************************************
 System Initialization File

  File Name:
    system_init.c

  Summary:
    System Initialization.

 Important :
 This demo uses an evaluation license which is meant for demonstration only and
 that customers desiring development and production on OPENRTOS must procure a
 suitable license.
 
  Description:
    This file contains source code necessary to initialize the system.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <xc.h>
#include "app.h"
#include "system_config.h"
#include "system/int/sys_int.h"
/* MPLAB Harmony includes. */
#include "peripheral/osc/plib_osc.h"

// ****************************************************************************
// ****************************************************************************
// Section: Configuration Bits
// ****************************************************************************
// ****************************************************************************
/* Set up the Device Configuration */
// DEVCFG3
// USERID = No Setting
#pragma config FMIIEN   = OFF              // Ethernet RMII/MII Enable (MII Enabled)
#pragma config FETHIO   = OFF              // Ethernet I/O Pin Select (Default Ethernet I/O)
#pragma config PGL1WAY  = OFF             // Permission Group Lock One Way Configuration (Allow only one reconfiguration)
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY  = OFF             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = OFF           // USB USBID Selection (Controlled by Port Function)

// DEVCFG2
#pragma config FPLLIDIV = DIV_3         //3 System PLL Input Divider (1x Divider)
#pragma config FPLLRNG  = RANGE_13_26_MHZ // System PLL Input Range (5-10 MHz Input)
#pragma config FPLLICLK = PLL_POSC      // System PLL Input Clock Selection (POSC is input to the System PLL)
#pragma config FPLLMULT = MUL_50       //50 System PLL Multiplier (PLL Multiply by 50)
#pragma config FPLLODIV = DIV_2
#pragma config UPLLFSEL = FREQ_12MHZ    // USB PLL Input Frequency Selection (USB PLL input is 12 MHz)
#pragma config UPLLEN   = OFF              // USB PLL Enable (USB PLL is enabled)

// DEVCFG1
#pragma config FNOSC    = SPLL             // Oscillator Selection Bits (SPLL)
#pragma config DMTINTV  = WIN_127_128    // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN  = OFF            // Secondary Oscillator Enable (Disable SOSC)
#pragma config IESO     = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD  = EC             // Primary Oscillator Configuration (Primary osc enabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FCKSM    = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config WDTPS    = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WDTSPGM  = STOP           // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS   = NORMAL          // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN   = OFF             // Watchdog Timer Enable (WDT Disabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window size is 25%)
// DMTCNT = No Setting
#pragma config FDMTEN   = OFF             // Deadman Timer Enable (Deadman Timer is disabled)

/* DEVCFG0 */
#pragma config EJTAGBEN = NORMAL
#pragma config DBGPER   = PG_ALL
#pragma config FSLEEP   = OFF
#pragma config FECCCON  = OFF_UNLOCKED
#pragma config BOOTISA  = MIPS32
#pragma config TRCEN    = OFF
#pragma config ICESEL   = ICS_PGx2
#pragma config JTAGEN   = OFF
#pragma config DEBUG    = ON

// DEVCP0
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// ****************************************************************************
// ****************************************************************************
// Section: System Initialization
// ****************************************************************************
// ****************************************************************************
/*******************************************************************************
  Function:
    void SYS_Initialize ( SYS_INIT_DATA *data )

  Summary:
    Initializes the board, services, drivers, application and other modules

  Description:
    This routine initializes the board, services, drivers, application and other
    modules as configured at build time.  In a bare-metal environment (where no
    OS is supported), this routine should be called almost immediately after
    entering the "main" routine.

  Precondition:
    The C-language run-time environment and stack must have been initialized.

  Parameters:
    data        - Pointer to the system initialzation data structure containing
                  pointers to the board, system service, and driver
                  initialization routines
  Returns:
    None.

  Example:
    <code>
    SYS_INT_Initialize(NULL);
    </code>

  Remarks:
    Basic System Initialization Sequence:

    1.  Initilize minimal board services and processor-specific items
        (enough to use the board to initialize drivers and services)
    2.  Initialize all supported system services
    3.  Initialize all supported modules
        (libraries, drivers, middleware, and application-level modules)
    4.  Initialize the main (static) application, if present.

    The order in which services and modules are initialized and started may be
    important.

    For a static system (a system not using the ISP's dynamic implementation
    of the initialization and "Tasks" services) this routine is implemented
    for the specific configuration of an application.
 */

void SYS_Initialize ( void * data )
{
    /* Set up cache and wait states for maximum performance. */
    SYS_DEVCON_PerformanceConfig(SYS_CLK_FREQUENCY);
    /* Initialize the BSP */
     BSP_Initialize( );

     SYS_DEVCON_SystemUnlock();
    /* timers use clock PBCLK3, set this to 40MHz */
    PLIB_OSC_PBClockDivisorSet(OSC_ID_0, OSC_PERIPHERAL_BUS_3, 5 );
     /* ports use PBCLK4 */
    PLIB_OSC_PBClockDivisorSet(OSC_ID_0, OSC_PERIPHERAL_BUS_4, 1 );
     
     SYS_DEVCON_SystemLock();
    /* Initialize the Application */
     APP_Initialize ();
    /* Initializethe interrupt system  */
    SYS_INT_Initialize();

     /* set priority for Timer 5 interrupt source */
    /* Same as configMAX_SYSCALL_INTERRUPT_PRIORITY */
    SYS_INT_VectorPrioritySet(INT_VECTOR_T5, INT_PRIORITY_LEVEL3);
    
    /* set sub-priority for Timer 5 interrupt source */
    SYS_INT_VectorSubprioritySet(INT_VECTOR_T5, INT_SUBPRIORITY_LEVEL3);
}


/*******************************************************************************/
/*******************************************************************************
 End of File
*/
