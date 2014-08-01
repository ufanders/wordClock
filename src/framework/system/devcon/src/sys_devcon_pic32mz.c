/*******************************************************************************
  Device Control System Service Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    sys_devcon.c

  Summary:
    Device Control System Service implementation.

  Description:
    The DEVCON system service provides a simple interface to manage the Device 
    Control module on Microchip microcontrollers. This file Implements the core
    interface routines for the Device Control system service. While building 
    the system service from source, ALWAYS include this file in the build.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "sys_devcon_local.h"
#include "peripheral/int/plib_int.h"
#include "peripheral/pcache/plib_pcache.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Variable Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: SYS DEVCON Client Setup Routines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* Function:
    void SYS_DEVCON_PerformanceConfig( unsigned int sysclk )

  Summary:
    Configures the PFM wait states and prefetch (cache) module for maximum 
    performance.

  Description:
    This function configures the PFM wait states and prefetch (cache) module 
    for maximum performance.

  Remarks:
    None.
*/

void SYS_DEVCON_PerformanceConfig( unsigned int sysclk )
{
    bool int_flag = false;
    bool ecc;

    if (PLIB_PCACHE_ExistsWaitState(PCACHE_ID_0))
    {
        int ws;

        /* TODO: replace register read with plib when available */
        ecc = (((CFGCON & 0x00000030) >> 4) < 2) ? true : false;
        if (sysclk <= (ecc ? 66000000 : 83000000))
            ws = 0;
        else if (sysclk <= (ecc ? 133000000 : 166000000))
            ws = 1;
        else
            ws = 2;

        if (PLIB_INT_IsEnabled (INT_ID_0))
        {
            int_flag = true;
            PLIB_INT_Disable(INT_ID_0);
        }

        PLIB_PCACHE_WaitStateSet(PCACHE_ID_0, ws);

        if (int_flag)
        {
            PLIB_INT_Enable(INT_ID_0);
            int_flag = false;
        }
    }

    if (PLIB_INT_IsEnabled (INT_ID_0))
    {
        int_flag = true;
        PLIB_INT_Disable(INT_ID_0);
    }

    if (PLIB_PCACHE_ExistsPrefetchEnable(PCACHE_ID_0))
    {
        PLIB_PCACHE_PrefetchEnableSet(PCACHE_ID_0, PLIB_PCACHE_PREFETCH_ENABLE_ALL);
    }
    
    if (int_flag)
    {
        PLIB_INT_Enable(INT_ID_0);
        int_flag = false;
    }
}

/*******************************************************************************
 End of File
*/

