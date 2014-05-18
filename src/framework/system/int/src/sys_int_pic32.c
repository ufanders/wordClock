/*******************************************************************************
  Interrupt System Service

  Company:
    Microchip Technology Inc.

  File Name:
    sys_int_pic32.c

  Summary:
    Interrupt System Service APIs.

  Description:
    This file contains functions related to the Interrupt System Service for PIC32
    devices.
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

#include "system/int/sys_int.h"

// *****************************************************************************
// *****************************************************************************
// Section: Function Definitions
// *****************************************************************************
// *****************************************************************************


/******************************************************************************

  Function:
    void SYS_INT_Initialize ( void )

  Summary:
    Configures and initializes the interrupt sub-system.

  Description:
    This function appropriately configures and initializes the interrupt sub-system
    for the current system design.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Example:
  <code>
  SYS_INT_Initialize();
  </code>

  Remarks:
    This function is not implemented in the System Interrupt library.  It is
    implemented by the board support package (using the processor-specific
    interrupt peripheral library) because it requires knowledge of the specific
    interrupt requirements for each system.
*/

void SYS_INT_Initialize ( void )

{
    /* enable the multi vector */
    PLIB_INT_MultiVectorSelect( INT_ID_0 );
}

// *****************************************************************************
/*  Disable the generation of interrupts to the CPU

  Summary:
     Disables all interrupts

  Description:
     This function disables all interrupts.
  
  Remarks:
    This feature is not supported on all devices.  Refer to the specific device
    data sheet or family reference manual to determine whether this feature is 
    supported.
 */

bool SYS_INT_Disable( void )
{
    bool intStatus;

    /* get the interrupt disable status before disable is called */
    intStatus = PLIB_INT_IsEnabled (INT_ID_0);

    /* disable the interrupts */
    PLIB_INT_Disable(INT_ID_0);
    
    /* return the interrupt status */
    return intStatus;
}


// *****************************************************************************
/* Disable a interrupt from a particular source.

  Summary:
    Disables the interrupt source.

  Description:
    This function disables the interrupt source.
  
  Remarks:
    This feature is not supported on all devices.  Refer to the specific device
    data sheet or family reference manual to determine whether this feature is 
    supported.
 */


bool SYS_INT_SourceDisable ( INT_SOURCE source )
{
    bool intSrcStatus;

    /* get the interrupt status of this source before disable is called */
    intSrcStatus = PLIB_INT_SourceIsEnabled (INT_ID_0 , source);

    /* disable the interrupts */
    PLIB_INT_SourceDisable (INT_ID_0 , source);

    /* return the source status */
    return intSrcStatus;
}
