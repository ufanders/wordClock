/*******************************************************************************
  INT Peripheral Library Template Implementation

  File Name:
    int_VectorPriority_Default.h

  Summary:
    INT PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : VectorPriority
    and its Variant : Default
    For following APIs :
        PLIB_INT_ExistsVectorPriority
        PLIB_INT_VectorPrioritySet
        PLIB_INT_VectorPriorityGet
        PLIB_INT_VectorSubPrioritySet
        PLIB_INT_VectorSubPriorityGet

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

#ifndef _INT_VECTORPRIORITY_DEFAULT_H
#define _INT_VECTORPRIORITY_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    _INT_INT_PRIORITY_CONTROL_0_VREG(index)
    

  MASKs: 
    _INT_INT_PRIORITY_CONTROL_0_MASK(index)
    

  POSs: 
    _INT_INT_PRIORITY_CONTROL_0_POS(index)
    

  LENs: 
    _INT_INT_PRIORITY_CONTROL_0_LEN(index)
    

*/


//******************************************************************************
/* Function :  INT_ExistsVectorPriority_Default

  Summary:
    Implements Default variant of PLIB_INT_ExistsVectorPriority

  Description:
    This template implements the Default variant of the PLIB_INT_ExistsVectorPriority function.
*/

PLIB_TEMPLATE bool INT_ExistsVectorPriority_Default( INT_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  INT_VectorPrioritySet_Default

  Summary:
    Implements Default variant of PLIB_INT_VectorPrioritySet 

  Description:
    This template implements the Default variant of the PLIB_INT_VectorPrioritySet function.

  Note:
    The vector enum encoding is (x * 32) + y, where x is the register number 
    (IPCx) and y is the bit position of the subpriority field LSB. 0b0000_000x_xxxy_yyyy
*/

PLIB_TEMPLATE void INT_VectorPrioritySet_Default( INT_MODULE_ID index , INT_VECTOR vector , INT_PRIORITY_LEVEL priority )
{
    _SFR_CLEAR(_INT_INT_PRIORITY_CONTROL_0_VREG(index) + ((0x10 * ((vector & 0x7E0) >> 5)) >> 2), 0x7 << ((vector & 0x1F) + 2));
    _SFR_SET(_INT_INT_PRIORITY_CONTROL_0_VREG(index) + ((0x10 * ((vector & 0x7E0) >> 5)) >> 2), priority << ((vector & 0x1F) + 2));
}


//******************************************************************************
/* Function :  INT_VectorPriorityGet_Default

  Summary:
    Implements Default variant of PLIB_INT_VectorPriorityGet 

  Description:
    This template implements the Default variant of the PLIB_INT_VectorPriorityGet function.

  Note:
    The vector enum encoding is (x * 32) + y, where x is the register number 
    (IPCx) and y is the bit position of the subpriority field LSB. 0b0000_000x_xxxy_yyyy
*/

PLIB_TEMPLATE INT_PRIORITY_LEVEL INT_VectorPriorityGet_Default( INT_MODULE_ID index , INT_VECTOR vector )
{
    return _SFR_READ(_INT_INT_PRIORITY_CONTROL_0_VREG(index) + ((0x10 * ((vector & 0x7E0) >> 5)) >> 2)) >> ((vector & 0x1F) + 2);
}


//******************************************************************************
/* Function :  INT_VectorSubPrioritySet_Default

  Summary:
    Implements Default variant of PLIB_INT_VectorSubPrioritySet 

  Description:
    This template implements the Default variant of the PLIB_INT_VectorSubPrioritySet function.

  Note:
    The vector enum encoding is (x * 32) + y, where x is the register number 
    (IPCx) and y is the bit position of the subpriority field LSB. 0b0000_000x_xxxy_yyyy
*/

PLIB_TEMPLATE void INT_VectorSubPrioritySet_Default( INT_MODULE_ID index , INT_VECTOR vector , INT_SUBPRIORITY_LEVEL subpriority )
{
    _SFR_CLEAR(_INT_INT_PRIORITY_CONTROL_0_VREG(index) + ((0x10 * ((vector & 0x7E0) >> 5)) >> 2), 0x3 << (vector & 0x1F));
    _SFR_SET(_INT_INT_PRIORITY_CONTROL_0_VREG(index) + ((0x10 * ((vector & 0x7E0) >> 5)) >> 2), subpriority << (vector & 0x1F));
}


//******************************************************************************
/* Function :  INT_VectorSubPriorityGet_Default

  Summary:
    Implements Default variant of PLIB_INT_VectorSubPriorityGet 

  Description:
    This template implements the Default variant of the PLIB_INT_VectorSubPriorityGet function.

  Note:
    The vector enum encoding is (x * 32) + y, where x is the register number 
    (IPCx) and y is the bit position of the subpriority field LSB. 0b0000_000x_xxxy_yyyy
*/

PLIB_TEMPLATE INT_SUBPRIORITY_LEVEL INT_VectorSubPriorityGet_Default( INT_MODULE_ID index , INT_VECTOR vector )
{
    return (_SFR_READ(_INT_INT_PRIORITY_CONTROL_0_VREG(index) + ((0x10 * ((vector & 0x7E0) >> 5)) >> 2)) >> (vector & 0x1F)) & 0x3;
}


#endif /*_INT_VECTORPRIORITY_DEFAULT_H*/

/******************************************************************************
 End of File
*/

