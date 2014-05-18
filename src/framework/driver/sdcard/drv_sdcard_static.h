/*******************************************************************************
  SAMPLE Driver Interface Declarations for static single instance driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sample_static_single.h

  Summary:
    SAMPLE Driver Interface Declarations for static single instance driver

  Description:
    The SAMPLE device driver provides a simple interface to manage the SAMPLE
    modules on Microchip microcontrollers.  This file defines the interface
    Declarations for the static single-instance variants of the SAMPLE driver
    driver interface routines

  Remarks:
    Static interfaces encorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    
    Static single-open interfaces also eliminate the need for the open handle.
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

#ifndef _DRV_SAMPLE_STATIC_SINGLE_H
#define _DRV_SAMPLE_STATIC_SINGLE_H


// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 0 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_SAMPLE0_Initialize( const SYS_MODULE_INIT * const init);

void DRV_SAMPLE0_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_SAMPLE0_Deinitialize( void );

SYS_STATUS DRV_SAMPLE0_Status( void );

void DRV_SAMPLE0_Tasks ( void );

void DRV_SAMPLE0_Open( const DRV_IO_INTENT intent );

void DRV_SAMPLE0_Close( void );

DRV_SAMPLE_CLIENT_STATUS DRV_SAMPLE0_ClientStatus( void );

unsigned int DRV_SAMPLE0_VersionGet( void );

char* DRV_SAMPLE0_VersionStrGet( void );

// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 1 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_SAMPLE1_Initialize( const SYS_MODULE_INIT * const init);

void DRV_SAMPLE1_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_SAMPLE1_Deinitialize( void );

SYS_STATUS DRV_SAMPLE1_Status( void );

void DRV_SAMPLE1_Tasks ( void );

void DRV_SAMPLE1_Open( const DRV_IO_INTENT intent );

void DRV_SAMPLE1_Close( void );

DRV_SAMPLE_CLIENT_STATUS DRV_SAMPLE1_ClientStatus( void );

unsigned int DRV_SAMPLE1_VersionGet( void );

char* DRV_SAMPLE1_VersionStrGet( void );

#endif // #ifndef _DRV_SAMPLE_STATIC_SINGLE_H

/*******************************************************************************
 End of File
*/
