/*******************************************************************************
  SD Card Driver Interface Declarations for static multi instance driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_SD Card_static_multi.h

  Summary:
    SD Card Driver Interface Declarations for static multi instance driver

  Description:
    The SD Card device driver provides a simple interface to manage the SD Card
    modules on Microchip microcontrollers.  This file defines the interface
    Declarations for the static multi-instance variants of the SD Card driver
    driver interface routines.
    
  Remarks:
    Static multi-instance interfaces encorporate the driver instance number 
    within the names of the routines, eliminating the need for an object ID or 
    object handle.
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

#ifndef _DRV_SDCARD_STATIC_MULTI_H
#define _DRV_SDCARD_STATIC_MULTI_H


// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 0 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_SDCARD0multi_Initialize( const SYS_MODULE_INIT * const init);

void DRV_SDCARD0multi_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_SDCARD0multi_Deinitialize( void );

SYS_STATUS DRV_SDCARD0multi_Status( void );

void DRV_SDCARD0multi_Tasks ( void );

DRV_HANDLE DRV_SDCARD0multi_Open( const DRV_IO_INTENT intent );

void DRV_SDCARD0multi_Close( DRV_HANDLE handle );

DRV_SDCARD_CLIENT_STATUS DRV_SDCARD0multi_ClientStatus( DRV_HANDLE handle );

unsigned int DRV_SDCARD0_VersionGet( void );

char* DRV_SDCARD0_VersionStrGet( void );

// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 1 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_SDCARD1multi_Initialize( const SYS_MODULE_INIT * const init);

void DRV_SDCARD1multi_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_SDCARD1multi_Deinitialize( void );

SYS_STATUS DRV_SDCARD1multi_Status( void );

void DRV_SDCARD1multi_Tasks ( void );

DRV_HANDLE DRV_SDCARD1multi_Open( const DRV_IO_INTENT intent );

void DRV_SDCARD1multi_Close( DRV_HANDLE handle );

DRV_SDCARD_CLIENT_STATUS DRV_SDCARD1multi_ClientStatus( DRV_HANDLE handle );

unsigned int DRV_SDCARD1_VersionGet( void );

char* DRV_SDCARD1_VersionStrGet( void );

#endif // #ifndef _DRV_SDCARD_STATIC_MULTI_H

/*******************************************************************************
 End of File
*/
