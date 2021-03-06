/*******************************************************************************
  TMR Peripheral Library Template Implementation

  File Name:
    tmr_CounterAsyncWriteControl_Unsupported.h

  Summary:
    TMR PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : CounterAsyncWriteControl
    and its Variant : Unsupported
    For following APIs :
        PLIB_TMR_CounterAsyncWriteEnable
        PLIB_TMR_CounterAsyncWriteDisable
        PLIB_TMR_ExistsCounterAsyncWriteControl

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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

//DOM-IGNORE-END

#ifndef _TMR_COUNTERASYNCWRITECONTROL_UNSUPPORTED_H
#define _TMR_COUNTERASYNCWRITECONTROL_UNSUPPORTED_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    None.

  MASKs: 
    None.

  POSs: 
    None.

  LENs: 
    None.

*/


//******************************************************************************
/* Function :  TMR_CounterAsyncWriteEnable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_TMR_CounterAsyncWriteEnable 

  Description:
    This template implements the Unsupported variant of the PLIB_TMR_CounterAsyncWriteEnable function.
*/

PLIB_TEMPLATE void TMR_CounterAsyncWriteEnable_Unsupported( TMR_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_TMR_CounterAsyncWriteEnable");
}


//******************************************************************************
/* Function :  TMR_CounterAsyncWriteDisable_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_TMR_CounterAsyncWriteDisable 

  Description:
    This template implements the Unsupported variant of the PLIB_TMR_CounterAsyncWriteDisable function.
*/

PLIB_TEMPLATE void TMR_CounterAsyncWriteDisable_Unsupported( TMR_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_TMR_CounterAsyncWriteDisable");
}


//******************************************************************************
/* Function :  TMR_ExistsCounterAsyncWriteControl_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_TMR_ExistsCounterAsyncWriteControl

  Description:
    This template implements the Unsupported variant of the PLIB_TMR_ExistsCounterAsyncWriteControl function.
*/

PLIB_TEMPLATE bool TMR_ExistsCounterAsyncWriteControl_Unsupported( TMR_MODULE_ID index )
{
    return false;
}


#endif /*_TMR_COUNTERASYNCWRITECONTROL_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

