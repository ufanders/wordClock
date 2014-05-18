/*******************************************************************************
  SPI Driver Interface Declarations for Static Multi-instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_spi_static_multi.h

  Summary:
    SPI Driver interface declarations for the static multi-instance driver.

  Description:
    The SPI Driver provides a simple interface to manage the SPI
    modules on Microchip microcontrollers.  This file defines the interface
    Declarations for the static multi-instance variants of the SPI driver
    driver interface routines.
    
  Remarks:
    Static multi-instance interfaces incorporate the driver instance number
    within the names of the routines, eliminating the need for an object ID or 
    object handle.
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

#ifndef _DRV_SPI_STATIC_MULTI_H
#define _DRV_SPI_STATIC_MULTI_H


// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 0 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_SPI0multi_Initialize( const SYS_MODULE_INIT * const init);

void DRV_SPI0multi_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_SPI0multi_Deinitialize( void );

SYS_STATUS DRV_SPI0multi_Status( void );

void DRV_SPI0multi_Tasks ( void );

DRV_HANDLE DRV_SPI0multi_Open( const DRV_IO_INTENT intent );

void DRV_SPI0multi_Close( DRV_HANDLE handle );

DRV_SPI_CLIENT_STATUS DRV_SPI0multi_ClientStatus( DRV_HANDLE handle );

void DRV_SPI0multi_CommunicationSetup( DRV_HANDLE handle, const DRV_SPI_COMM_CONFIG * const config );

SPI_DATA_TYPE DRV_SPI0multi_Get( DRV_HANDLE handle );

void DRV_SPI0multi_Put( DRV_HANDLE handle, const SPI_DATA_TYPE buffer );

unsigned int DRV_SPI0multi_GetBuffer( DRV_HANDLE handle, SPI_DATA_TYPE *buffer, const unsigned int numbytes );

unsigned int DRV_SPI0multi_PutBuffer( DRV_HANDLE handle, const SPI_DATA_TYPE *buffer, const unsigned int numbytes );

DRV_SPI_TRANSFER_STATUS DRV_SPI0multi_TransferStatus( DRV_HANDLE handle );

unsigned int DRV_SPI0multi_VersionGet( void );

char * DRV_SPI0multi_VersionStrGet ( void );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 1 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_SPI1multi_Initialize( const SYS_MODULE_INIT * const init);

void DRV_SPI1multi_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_SPI1multi_Deinitialize( void );

SYS_STATUS DRV_SPI1multi_Status( void );

void DRV_SPI1multi_Tasks ( void );

DRV_HANDLE DRV_SPI1multi_Open( const DRV_IO_INTENT intent );

void DRV_SPI1multi_Close( DRV_HANDLE handle );

DRV_SPI_CLIENT_STATUS DRV_SPI1multi_ClientStatus( DRV_HANDLE handle );

void DRV_SPI1multi_CommunicationSetup( DRV_HANDLE handle, const DRV_SPI_COMM_CONFIG * const config );

SPI_DATA_TYPE DRV_SPI1multi_Get( DRV_HANDLE handle );

void DRV_SPI1multi_Put( DRV_HANDLE handle, const SPI_DATA_TYPE buffer );

unsigned int DRV_SPI1multi_GetBuffer( DRV_HANDLE handle, SPI_DATA_TYPE *buffer, const unsigned int numbytes );

unsigned int DRV_SPI1multi_PutBuffer( DRV_HANDLE handle, const SPI_DATA_TYPE *buffer, const unsigned int numbytes );

DRV_SPI_TRANSFER_STATUS DRV_SPI1multi_TransferStatus( DRV_HANDLE handle );

unsigned int DRV_SPI1multi_VersionGet( void );

char * DRV_SPI1multi_VersionStrGet ( void );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 2 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_SPI2multi_Initialize( const SYS_MODULE_INIT * const init);

void DRV_SPI2multi_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_SPI2multi_Deinitialize( void );

SYS_STATUS DRV_SPI2multi_Status( void );

void DRV_SPI2multi_Tasks ( void );

DRV_HANDLE DRV_SPI2multi_Open( const DRV_IO_INTENT intent );

void DRV_SPI2multi_Close( DRV_HANDLE handle );

DRV_SPI_CLIENT_STATUS DRV_SPI2multi_ClientStatus( DRV_HANDLE handle );

void DRV_SPI2multi_CommunicationSetup( DRV_HANDLE handle, const DRV_SPI_COMM_CONFIG * const config );

SPI_DATA_TYPE DRV_SPI2multi_Get( DRV_HANDLE handle );

void DRV_SPI2multi_Put( DRV_HANDLE handle, const SPI_DATA_TYPE buffer );

unsigned int DRV_SPI2multi_GetBuffer( DRV_HANDLE handle, SPI_DATA_TYPE *buffer, const unsigned int numbytes );

unsigned int DRV_SPI2multi_PutBuffer( DRV_HANDLE handle, const SPI_DATA_TYPE *buffer, const unsigned int numbytes );

DRV_SPI_TRANSFER_STATUS DRV_SPI2multi_TransferStatus( DRV_HANDLE handle );

unsigned int DRV_SPI2multi_VersionGet( void );

char * DRV_SPI2multi_VersionStrGet ( void );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 3 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_SPI3multi_Initialize( const SYS_MODULE_INIT * const init);

void DRV_SPI3multi_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_SPI3multi_Deinitialize( void );

SYS_STATUS DRV_SPI3multi_Status( void );

void DRV_SPI3multi_Tasks ( void );

DRV_HANDLE DRV_SPI3multi_Open( const DRV_IO_INTENT intent );

void DRV_SPI3multi_Close( DRV_HANDLE handle );

DRV_SPI_CLIENT_STATUS DRV_SPI3multi_ClientStatus( DRV_HANDLE handle );

void DRV_SPI3multi_CommunicationSetup( DRV_HANDLE handle, const DRV_SPI_COMM_CONFIG * const config );

SPI_DATA_TYPE DRV_SPI3multi_Get( DRV_HANDLE handle );

void DRV_SPI3multi_Put( DRV_HANDLE handle, const SPI_DATA_TYPE buffer );

unsigned int DRV_SPI3multi_GetBuffer( DRV_HANDLE handle, SPI_DATA_TYPE *buffer, const unsigned int numbytes );

unsigned int DRV_SPI3multi_PutBuffer( DRV_HANDLE handle, const SPI_DATA_TYPE *buffer, const unsigned int numbytes );

DRV_SPI_TRANSFER_STATUS DRV_SPI3multi_TransferStatus( DRV_HANDLE handle );

unsigned int DRV_SPI3multi_VersionGet( void );

char * DRV_SPI3multi_VersionStrGet ( void );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 4 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_SPI4multi_Initialize( const SYS_MODULE_INIT * const init);

void DRV_SPI4multi_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_SPI4multi_Deinitialize( void );

SYS_STATUS DRV_SPI4multi_Status( void );

void DRV_SPI4multi_Tasks ( void );

DRV_HANDLE DRV_SPI4multi_Open( const DRV_IO_INTENT intent );

void DRV_SPI4multi_Close( DRV_HANDLE handle );

DRV_SPI_CLIENT_STATUS DRV_SPI4multi_ClientStatus( DRV_HANDLE handle );

void DRV_SPI4multi_CommunicationSetup( DRV_HANDLE handle, const DRV_SPI_COMM_CONFIG * const config );

SPI_DATA_TYPE DRV_SPI4multi_Get( DRV_HANDLE handle );

void DRV_SPI4multi_Put( DRV_HANDLE handle, const SPI_DATA_TYPE buffer );

unsigned int DRV_SPI4multi_GetBuffer( DRV_HANDLE handle, SPI_DATA_TYPE *buffer, const unsigned int numbytes );

unsigned int DRV_SPI4multi_PutBuffer( DRV_HANDLE handle, const SPI_DATA_TYPE *buffer, const unsigned int numbytes );

DRV_SPI_TRANSFER_STATUS DRV_SPI4multi_TransferStatus( DRV_HANDLE handle );

unsigned int DRV_SPI4multi_VersionGet( void );

char * DRV_SPI4multi_VersionStrGet ( void );


#endif // #ifndef _DRV_SPI_STATIC_MULTI_H

/*******************************************************************************
 End of File
*/
