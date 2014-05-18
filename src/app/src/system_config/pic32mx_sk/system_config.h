/*******************************************************************************
  MPLAB Harmony Blinky LEDs Example

  Company:
    Microchip Technology Inc.

  File Name:
    system_config.h

  Summary:
    MPLAB Harmony blinky_leds system configuration

  Description:
    This file contains the MPLAB Harmony blinky_leds SYS_Initialize routine.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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


#ifndef _SYSTEM_CONFIG_H    // Guards against multiple inclusion
#define _SYSTEM_CONFIG_H
/* This is a temporary workaround for an issue with the peripheral library "Exists"
   functions that causes superfluous warnings.  It "nulls" out the definition of
   The PLIB function attribute that causes the warning.  Once that issue has been
   resolved, this definition should be removed. */
#define _PLIB_UNSUPPORTED

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/*  This section Includes other configuration headers necessary to completely
    define this configuration.
*/


// *****************************************************************************
// *****************************************************************************
// Section: General System Configuration
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Application Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: System Services Configuration
// *****************************************************************************
// *****************************************************************************

#define SYS_CLK_FREQUENCY                       (80000000ul)
#define SYS_DEVCON_PIC32MX_MAX_PB_FREQ           40000000L

#endif // _SYS_CONFIG_H
/*******************************************************************************
 End of File
*/
