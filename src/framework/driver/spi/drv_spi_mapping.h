/*******************************************************************************
  SPI Device Driver Interface Names Mapping

  Company:
    Microchip Technology Inc.

  File Name:
    drv_spi_mapping.h

  Summary:
    SPI device driver interface names mapping.

  Description:
    This header file maps the interface prototypes in drv_spi.h to
    static variants of these routines appropriate for the selcted configuration.
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

#ifndef _DRV_SPI_MAPPING_H
#define _DRV_SPI_MAPPING_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
/* Note:  See the bottom of file for implementation header include files.
*/

#include <stdint.h>
#include <stdbool.h>

#include "system_config.h"
#include "driver/spi/drv_spi_static.h"
#include "driver/spi/drv_spi_static_multi.h"


// *****************************************************************************
// *****************************************************************************
// Section: Build Parameter Checking
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: DRV_SPI_INSTANCES_NUMBER Check

  Summary:
    Checks the DRV_SPI_INSTANCES_NUMBER definition.

  Description:
    If DRV_SPI_INSTANCES_NUMBER is greater than the number of
    SPI instances available on the device, an error is generated.

  Remarks:
    The _SPI_EXISTS is a device-specific value defined by the processor
    headers in the PLIB.

    If the configuration does not specify the number of driver instances to
    allocate it defaults to the number of SPI instances on the device.
*/

#if defined(DRV_SPI_INSTANCES_NUMBER)

    #if( DRV_SPI_INSTANCES_NUMBER > 5 )

        #error "The number of SPI instances configured is more than the available SPIs on the part"

    #endif

#endif


// *****************************************************************************
// *****************************************************************************
// Section: SPI Driver Static API Name Generation
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: _DRV_SPI_MAKE_NAME(name)

  Summary:
    Creates an instance-specific static interface name.

  Description:
    This macro creates the instance-specific name of the specified static interface
    function by inserting the index number into the name.

    Example: DRV_SPI_Initialize becomes DRV_SPI1_Initialize for an index of 1.

  Remarks:
    Multi-client configurations add the word "multi" to the API name, single-
    client configurations do not.

    Example DRV_SPI_Initialize becomes DRV_SPI1multi_Initialize for an index
    of 1.
*/


#if !defined(DRV_SPI_INSTANCES_NUMBER)

    // Static builds use static naming to reduce calling overhead.
    #if !defined(DRV_SPI_CLIENTS_NUMBER)

        // Static Single-Client Interface Name Generation
        #define _DRV_SPI_STATIC_API_SINGLE(index, name)     DRV_SPI ## index ## _ ## name
        #define _DRV_SPI_STATIC_API(index, name)            _DRV_SPI_STATIC_API_SINGLE(index, name)

    #else // ( DRV_SPI_CLIENTS_NUMBER >= 1 )

        // Static Multi-Client Interface Name Generation
        #define _DRV_SPI_STATIC_API_MULTI(index, name)      DRV_SPI ## index ## multi_ ## name
        #define _DRV_SPI_STATIC_API(index, name)            _DRV_SPI_STATIC_API_MULTI(index, name)

    #endif

    // Static naming Macros
    #define _DRV_SPI_MAKE_NAME(name)                        _DRV_SPI_STATIC_API(DRV_SPI_INDEX, name)

#else // (DRV_SPI_CONFIG_BUILD_TYPE == DRV_BUILD_TYPE_DYNAMIC)

    // Dynamic Interface Name Generation
    #define _DRV_SPI_MAKE_NAME(name)                        DRV_SPI_ ## name

#endif


// *****************************************************************************
// *****************************************************************************
// Section: SPI Driver Static API Mapping
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Static Interface Mapping

  Summary:
    Maps the dynamic interface calls to appropriate static interface.

  Description:
    These macros map calls to the dynamic interface function to calls to the
    appropriate instance-specific static interface function when a static build
    (DRV_SPI_INSTANCES_NUMBER is not defined) is configured.
    
    Example: DRV_SPI_Status(DRV_SPI_INDEX_1);
    Becomes: DRV_SPI_Status();
        
  Remarks:
    Static configuration eliminate the need to pass the object parameter.  
    However, the index is "returned" as the object handle (from the 
    "Initialize" routine) to allow code written for the dynamic API to continue
    to build when using a static configuration.
    
    Example:
    
        object = DRV_SPI_Initialize(DRV_SPI_INDEX_1, &initData);
        
    Becomes:
    
        object = ( DRV_SPI1_Initialize(&initData), DRV_SPI_INDEX );
        
    Static single-client configurations also eliminate the client handle 
    parameter.  However, the index (the driver-object index) is "returned" from
    the "Open" routine to allow code written for multi-client drivers to build
    when using a static single-open configuration.
    
    Example:
    
        handle = DRV_SPI_Open(DRV_SPI_INDEX_1, intent);
        
    Becomes:
    
        handle = ( DRV_SPI_Open(intent), DRV_SPI_INDEX );
*/

#if !defined(DRV_SPI_INSTANCES_NUMBER) // static

    #if !defined(DRV_SPI_CLIENTS_NUMBER) // single client

        #define DRV_SPI_Initialize(sysID, inData)           (_DRV_SPI_MAKE_NAME(Initialize)(inData), DRV_SPI_INDEX)

        #define DRV_SPI_Reinitialize(sysObj, inData)        _DRV_SPI_MAKE_NAME(Reinitialize)(inData)

        #define DRV_SPI_Deinitialize(sysObj)                _DRV_SPI_MAKE_NAME(Deinitialize)()

        #define DRV_SPI_Status(sysObj)                      _DRV_SPI_MAKE_NAME(Status)()

        #define DRV_SPI_Tasks(sysObj)                       _DRV_SPI_MAKE_NAME(Tasks)()

        #define DRV_SPI_Open(sysID, intent)                 (_DRV_SPI_MAKE_NAME(Open)(intent), DRV_SPI_INDEX)

        #define DRV_SPI_Close(handle)                       _DRV_SPI_MAKE_NAME(Close)()

        #define DRV_SPI_ClientStatus(handle)                _DRV_SPI_MAKE_NAME(ClientStatus)()

        #define DRV_SPI_CommunicationSetup(handle, config)  _DRV_SPI_MAKE_NAME(CommunicationSetup)(config)

        #define DRV_SPI_Get(handle)                         _DRV_SPI_MAKE_NAME(Get)()

        #define DRV_SPI_Put(handle, buffer)                 _DRV_SPI_MAKE_NAME(Put)(buffer)

        #define DRV_SPI_GetBuffer(handle, buffer, bytes)    _DRV_SPI_MAKE_NAME(GetBuffer)(buffer, bytes)

        #define DRV_SPI_PutBuffer(handle, buffer, bytes)    _DRV_SPI_MAKE_NAME(PutBuffer)(buffer, bytes)

        #define DRV_SPI_TransferStatus(handle)              _DRV_SPI_MAKE_NAME(TransferStatus)()

        #define DRV_SPI_VersionGet(sysID)                   _DRV_SPI_MAKE_NAME(VersionGet)()

        #define DRV_SPI_VersionStrGet(sysID)                _DRV_SPI_MAKE_NAME(VersionStrGet)()

    #else // multi-client

        #define DRV_SPI_Initialize(sysID, inData)           (_DRV_SPI_MAKE_NAME(Initialize)(inData), DRV_SPI_INDEX)

        #define DRV_SPI_Reinitialize(sysObj, inData)        _DRV_SPI_MAKE_NAME(Reinitialize)(inData)

        #define DRV_SPI_Deinitialize(sysObj)                _DRV_SPI_MAKE_NAME(Deinitialize)()

        #define DRV_SPI_Status(sysObj)                      _DRV_SPI_MAKE_NAME(Status)()

        #define DRV_SPI_Tasks(sysObj)                       _DRV_SPI_MAKE_NAME(Tasks)()

        #define DRV_SPI_Open(sysID, intent)                 (_DRV_SPI_MAKE_NAME(Open)(intent))

        #define DRV_SPI_Close(handle)                       _DRV_SPI_MAKE_NAME(Close)(handle)

        #define DRV_SPI_ClientStatus(handle)                _DRV_SPI_MAKE_NAME(ClientStatus)(handle)

        #define DRV_SPI_CommunicationSetup(handle, config)  _DRV_SPI_MAKE_NAME(CommunicationSetup)(handle, config)

        #define DRV_SPI_Get(handle)                         _DRV_SPI_MAKE_NAME(Get)(handle)

        #define DRV_SPI_Put(handle, buffer)                 _DRV_SPI_MAKE_NAME(Put)(handle, buffer)

        #define DRV_SPI_GetBuffer(handle, buffer, bytes)    _DRV_SPI_MAKE_NAME(GetBuffer)(handle, buffer, bytes)

        #define DRV_SPI_PutBuffer(handle, buffer, bytes)    _DRV_SPI_MAKE_NAME(PutBuffer)(handle, buffer, bytes)

        #define DRV_SPI_TransferStatus(handle)              _DRV_SPI_MAKE_NAME(TransferStatus)(handle)

        #define DRV_SPI_VersionGet(sysID)                   _DRV_SPI_MAKE_NAME(VersionGet)(sysID)

        #define DRV_SPI_VersionStrGet(sysID)                _DRV_SPI_MAKE_NAME(VersionStrGet)(sysID)

    #endif

#else // Dynamic Build

    // No Change in the Naming convention

#endif


// *****************************************************************************
// *****************************************************************************
// Section: SPI Driver API Mapping
// *****************************************************************************
// *****************************************************************************

/* This section maps the Specific SPI Driver APIs to the appropriate PLIB APIs */



#endif // #ifndef _DRV_SPI_MAPPING_H

/*******************************************************************************
 End of File
*/

