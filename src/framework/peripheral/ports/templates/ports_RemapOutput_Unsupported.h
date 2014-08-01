/*******************************************************************************
  PORTS Peripheral Library Template Implementation

  File Name:
    ports_RemapOutput_Unsupported.h

  Summary:
    PORTS PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : RemapOutput
    and its Variant : Unsupported
    For following APIs :
        PLIB_PORTS_RemapOutput
        PLIB_PORTS_ExistsRemapOutput

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

#ifndef _PORTS_REMAPOUTPUT_UNSUPPORTED_H
#define _PORTS_REMAPOUTPUT_UNSUPPORTED_H

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
/* Function :  PORTS_RemapOutput_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_PORTS_RemapOutput 

  Description:
    This template implements the Unsupported variant of the PLIB_PORTS_RemapOutput function.
*/

PLIB_TEMPLATE void PORTS_RemapOutput_Unsupported( PORTS_MODULE_ID      index , PORTS_REMAP_OUTPUT_FUNCTION outputFunction , PORTS_REMAP_OUTPUT_PIN      remapOutputPin )
{
    PLIB_ASSERT(false, "The device selected does not implement PLIB_PORTS_RemapOutput");
}


//******************************************************************************
/* Function :  PORTS_ExistsRemapOutput_Unsupported

  Summary:
    Implements Unsupported variant of PLIB_PORTS_ExistsRemapOutput

  Description:
    This template implements the Unsupported variant of the PLIB_PORTS_ExistsRemapOutput function.
*/

PLIB_TEMPLATE bool PORTS_ExistsRemapOutput_Unsupported( PORTS_MODULE_ID index )
{
    return false;
}


#endif /*_PORTS_REMAPOUTPUT_UNSUPPORTED_H*/

/******************************************************************************
 End of File
*/

