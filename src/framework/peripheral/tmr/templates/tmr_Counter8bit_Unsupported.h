/*******************************************************************************
  TMR Peripheral Library Template Implementation

  File Name:
    tmr_Counter8bit_Unsupported.h

  Summary:
    TMR PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : Counter8bit
    and its Variant : Unsupported
    For following APIs :
        PLIB_TMR_Counter8BitSet
        PLIB_TMR_Counter8BitGet
        PLIB_TMR_Counter8BitClear
        PLIB_TMR_ExistsCounter8Bit

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

#ifndef _TMR_COUNTER8BIT_UNSUPPORTED_H
#define _TMR_COUNTER8BIT_UNSUPPORTED_H

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
/* Function :  TMR_Counter8BitSet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_TMR_Counter8BitSet 

  Description:
    This template implements the Unsupported variant of the PLIB_TMR_Counter8BitSet function.
*/

PLIB_TEMPLATE void TMR_Counter8BitSet_Unsupported( TMR_MODULE_ID index , uint8_t value )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_TMR_Counter8BitSet");
}


//******************************************************************************
/* Function :  TMR_Counter8BitGet_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_TMR_Counter8BitGet 

  Description:
    This template implements the Unsupported variant of the PLIB_TMR_Counter8BitGet function.
*/

PLIB_TEMPLATE uint8_t TMR_Counter8BitGet_Unsupported( TMR_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_TMR_Counter8BitGet");

    return 0;
}


//******************************************************************************
/* Function :  TMR_Counter8BitClear_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_TMR_Counter8BitClear 

  Description:
    This template implements the Unsupported variant of the PLIB_TMR_Counter8BitClear function.
*/

PLIB_TEMPLATE void TMR_Counter8BitClear_Unsupported( TMR_MODULE_ID index )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_TMR_Counter8BitClear");
}


//******************************************************************************
/* Function :  TMR_ExistsCounter8Bit_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_TMR_ExistsCounter8Bit

  Description:
    This template implements the Unsupported variant of the PLIB_TMR_ExistsCounter8Bit function.
*/

PLIB_TEMPLATE bool TMR_ExistsCounter8Bit_Unsupported( TMR_MODULE_ID index )
{
    return false;
}


#endif /*_TMR_COUNTER8BIT_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

