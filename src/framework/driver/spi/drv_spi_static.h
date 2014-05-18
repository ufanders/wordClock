/*******************************************************************************
  SPI Driver Interface Declarations for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_spi_static.h

  Summary:
    SPI Driver interface declarations for the static single instance driver.

  Description:
    The SPI Driver provides a simple interface to manage the SPI modules on 
    Microchip microcontrollers.  This file defines the interface declarations 
    for the static single-instance variants of the SPI driver driver interface 
    functions.

  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the functions, eliminating the need for an object ID or object handle.
    
    Static single-open interfaces also eliminate the need for the open handle.
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

#ifndef _DRV_SPI_STATIC_H
#define _DRV_SPI_STATIC_H


// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 0 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_SPI0_Initialize( const SYS_MODULE_INIT * const init);

void DRV_SPI0_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_SPI0_Deinitialize( void );

SYS_STATUS DRV_SPI0_Status( void );

void DRV_SPI0_Tasks ( void );

void DRV_SPI0_Open( const DRV_IO_INTENT intent );

void DRV_SPI0_Close( void );

DRV_SPI_CLIENT_STATUS DRV_SPI0_ClientStatus( void );

void DRV_SPI0_CommunicationSetup( const DRV_SPI_COMM_CONFIG * const config );

SPI_DATA_TYPE DRV_SPI0_Get( void );

void DRV_SPI0_Put( const SPI_DATA_TYPE buffer );

unsigned int DRV_SPI0_GetBuffer( SPI_DATA_TYPE *buffer, const unsigned int numbytes );

unsigned int DRV_SPI0_PutBuffer( const SPI_DATA_TYPE *buffer, const unsigned int numbytes );

DRV_SPI_TRANSFER_STATUS DRV_SPI0_TransferStatus( void );

unsigned int DRV_SPI0_VersionGet( void );

char * DRV_SPI0_VersionStrGet ( void );

// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 1 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_SPI1_Initialize( const SYS_MODULE_INIT * const init);

void DRV_SPI1_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_SPI1_Deinitialize( void );

SYS_STATUS DRV_SPI1_Status( void );

void DRV_SPI1_Tasks ( void );

void DRV_SPI1_Open( const DRV_IO_INTENT intent );

void DRV_SPI1_Close( void );

DRV_SPI_CLIENT_STATUS DRV_SPI1_ClientStatus( void );

void DRV_SPI1_CommunicationSetup( const DRV_SPI_COMM_CONFIG * const config );

SPI_DATA_TYPE DRV_SPI1_Get( void );

void DRV_SPI1_Put( const SPI_DATA_TYPE buffer );

unsigned int DRV_SPI1_GetBuffer( SPI_DATA_TYPE *buffer, const unsigned int numbytes );

unsigned int DRV_SPI1_PutBuffer( const SPI_DATA_TYPE *buffer, const unsigned int numbytes );

DRV_SPI_TRANSFER_STATUS DRV_SPI1_TransferStatus( void );

unsigned int DRV_SPI1_VersionGet( void );

char * DRV_SPI1_VersionStrGet ( void );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 2 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_SPI2_Initialize( const SYS_MODULE_INIT * const init);

void DRV_SPI2_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_SPI2_Deinitialize( void );

SYS_STATUS DRV_SPI2_Status( void );

void DRV_SPI2_Tasks ( void );

void DRV_SPI2_Open( const DRV_IO_INTENT intent );

void DRV_SPI2_Close( void );

DRV_SPI_CLIENT_STATUS DRV_SPI2_ClientStatus( void );

void DRV_SPI2_CommunicationSetup( const DRV_SPI_COMM_CONFIG * const config );

SPI_DATA_TYPE DRV_SPI2_Get( void );

void DRV_SPI2_Put( const SPI_DATA_TYPE buffer );

unsigned int DRV_SPI2_GetBuffer( SPI_DATA_TYPE *buffer, const unsigned int numbytes );

unsigned int DRV_SPI2_PutBuffer( const SPI_DATA_TYPE *buffer, const unsigned int numbytes );

DRV_SPI_TRANSFER_STATUS DRV_SPI2_TransferStatus( void );

unsigned int DRV_SPI2_VersionGet( void );

char * DRV_SPI2_VersionStrGet ( void );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 3 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_SPI3_Initialize( const SYS_MODULE_INIT * const init);

void DRV_SPI3_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_SPI3_Deinitialize( void );

SYS_STATUS DRV_SPI3_Status( void );

void DRV_SPI3_Tasks ( void );

void DRV_SPI3_Open( const DRV_IO_INTENT intent );

void DRV_SPI3_Close( void );

DRV_SPI_CLIENT_STATUS DRV_SPI3_ClientStatus( void );

void DRV_SPI3_CommunicationSetup( const DRV_SPI_COMM_CONFIG * const config );

SPI_DATA_TYPE DRV_SPI3_Get( void );

void DRV_SPI3_Put( const SPI_DATA_TYPE buffer );

unsigned int DRV_SPI3_GetBuffer( SPI_DATA_TYPE *buffer, const unsigned int numbytes );

unsigned int DRV_SPI3_PutBuffer( const SPI_DATA_TYPE *buffer, const unsigned int numbytes );

DRV_SPI_TRANSFER_STATUS DRV_SPI3_TransferStatus( void );

unsigned int DRV_SPI3_VersionGet( void );

char * DRV_SPI3_VersionStrGet ( void );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 4 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_SPI4_Initialize( const SYS_MODULE_INIT * const init);

void DRV_SPI4_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_SPI4_Deinitialize( void );

SYS_STATUS DRV_SPI4_Status( void );

void DRV_SPI4_Tasks ( void );

void DRV_SPI4_Open( const DRV_IO_INTENT intent );

void DRV_SPI4_Close( void );

DRV_SPI_CLIENT_STATUS DRV_SPI4_ClientStatus( void );

void DRV_SPI4_CommunicationSetup( const DRV_SPI_COMM_CONFIG * const config );

SPI_DATA_TYPE DRV_SPI4_Get( void );

void DRV_SPI4_Put( const SPI_DATA_TYPE buffer );

unsigned int DRV_SPI4_GetBuffer( SPI_DATA_TYPE *buffer, const unsigned int numbytes );

unsigned int DRV_SPI4_PutBuffer( const SPI_DATA_TYPE *buffer, const unsigned int numbytes );

DRV_SPI_TRANSFER_STATUS DRV_SPI4_TransferStatus( void );

unsigned int DRV_SPI4_VersionGet( void );

char * DRV_SPI4_VersionStrGet ( void );


#endif // #ifndef _DRV_SPI_STATIC_H

/*******************************************************************************
 End of File
*/
