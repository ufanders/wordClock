/*******************************************************************************
  Device Control System Service Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    sys_devcon_local.h

  Summary:
    Device Control System Service local declarations and definitions.

  Description:
    This file contains the Device Control System Service local declarations 
    and definitions.
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


#ifndef _SYS_DEVCON_LOCAL_H
#define _SYS_DEVCON_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "system_config.h"
#include "system/devcon/sys_devcon.h"


// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* SYS DEVCON State Machine States

   Summary
    Defines the various states that can be achieved by the module operation.

   Description
    This enumeration defines the various states that can be achieved by the 
    module operation.

   Remarks:
    None.
*/

typedef enum
{
    /* SYS DEVCON state ready */
    SYS_DEVCON_STATE_READY,

    /* SYS DEVCON state busy */
    SYS_DEVCON_STATE_BUSY,

    /* SYS TMR state init */
    SYS_DEVCON_STATE_INIT,

} SYS_DEVCON_STATES;


// *****************************************************************************
/* Device Constrol System Service Hardware Instance Object

  Summary:
    Defines the object required for the maintainence of the hardware.

  Description:
    This defines the object required for the maintainence of the hardware.

  Remarks:
    None.
*/

typedef struct
{
    /* Current state of module */
    SYS_DEVCON_STATES state;

    /* Status of SYS DEVCON module */
    SYS_STATUS status;

} SYS_DEVCON_OBJECT;

extern SYS_DEVCON_OBJECT devconObject;


#endif //#ifndef _SYS_DEVCON_LOCAL_H

/*******************************************************************************
 End of File
*/

