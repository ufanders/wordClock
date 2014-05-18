/*******************************************************************************
  SDCARD Device Driver interface names mapping

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdcard_mapping.h

  Summary:
    SDCARD Device Driver Interface names mapping

  Description:
    This header file maps the interface prototypes in "drv_sdcard.h" to
    static variants of these routines appropriate for the selcted configuration.
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

#ifndef _DRV_SDCARD_MAPPING_H
#define _DRV_SDCARD_MAPPING_H


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
//#include "driver/sdcard/drv_sdcard_static_single.h"
#include "driver/sdcard/drv_sdcard_static_multi.h"


// *****************************************************************************
// *****************************************************************************
// Section: Build Parameter Checking
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: DRV_SDCARD_INSTANCES_NUMBER Check

  Summary:
    Checks the DRV_SDCARD_INSTANCES_NUMBER definition

  Description:
    If DRV_SDCARD_INSTANCES_NUMBER is greater than the number of
    SDCARD instances available on the part, an error is generated.

  Remarks:
    The _SDCARD_EXISTS is a processor-specific value defined by the processor
    headers in the PLIB.

    If the configuration does not specify the number of driver instances to
    allocate it defaults to then number of SDCARD instances on the part.
*/

#if defined(DRV_SDCARD_INSTANCES_NUMBER)

    #if (DRV_SDCARD_INSTANCES_NUMBER > SDCARD_MAX_LIMIT )

        #error "The number of SDCARD instances configured is more than the available SDCARDs on the part"

    #endif

#endif


// *****************************************************************************
// *****************************************************************************
// Section: SDCARD Driver Static API Name Generation
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: _DRV_SDCARD_MAKE_NAME(name)

  Summary:
    Creates an instance-specific static interface name

  Description:
     This macro creates the instance-specific name of the given static interface
     routine by inserting the index number into the name.

     Example DRV_SDCARD_Initialize becomes DRV_SDCARD1_Initialize for an index of 1.

  Remarks:
    Multi-client configurations add the word "multi" to the API name, single-
    client configurations do not.

    Example DRV_SDCARD_Initialize becomes DRV_SDCARD1multi_Initialize for an index
    of 1.
*/


#if !defined(DRV_SDCARD_INSTANCES_NUMBER)

    // Static builds use static naming to reduce calling overhead.
    #if !defined(DRV_SDCARD_CLIENTS_NUMBER)

        // Static Single-Client Interface Name Generation
        #define _DRV_SDCARD_STATIC_API_SINGLE(index, name)     DRV_SDCARD ## index ## _ ## name
        #define _DRV_SDCARD_STATIC_API(index, name)            _DRV_SDCARD_STATIC_API_SINGLE(index, name)

    #else // ( DRV_SDCARD_CLIENTS_NUMBER >= 1 )

        // Static Multi-Client Interface Name Generation
        #define _DRV_SDCARD_STATIC_API_MULTI(index, name)      DRV_SDCARD ## index ## multi_ ## name
        #define _DRV_SDCARD_STATIC_API(index, name)            _DRV_SDCARD_STATIC_API_MULTI(index, name)

    #endif

    // Static naming Macros
    #define _DRV_SDCARD_MAKE_NAME(name)                        _DRV_SDCARD_STATIC_API(DRV_SDCARD_INDEX, name)

#else // (DRV_SDCARD_CONFIG_BUILD_TYPE == DRV_BUILD_TYPE_DYNAMIC)

    // Dynamic Interface Name Generation
    #define _DRV_SDCARD_MAKE_NAME(name)                         DRV_SDCARD_ ## name

#endif


// *****************************************************************************
// *****************************************************************************
// Section: SDCARD Driver Static API Mapping
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Static Interface Mapping

  Summary:
    Maps the dynamic interface calls to appropriate static interface

  Description:
    These macros map calls to the dynamic interface routines to calls to the 
    appropriate instance-specific static interface routine when a static build
    (DRV_SDCARD_INSTANCES_NUMBER is not defined) is configured.
    
    Example:
    
        DRV_SDCARD_Status(DRV_SDCARD_INDEX_1);
        
    Becomes:
    
        DRV_SDCARD_Status();
        
  Remarks:
    Static configuration eliminate the need to pass the object parameter.  
    However, the index is "returned" as the object handle (from the 
    "Initialize" routine) to allow code written for the dynamic API to continue
    to build when using a static configuration.
    
    Example:
    
        object = DRV_SDCARD_Initialize(DRV_SDCARD_INDEX_1, &initData);
        
    Becomes:
    
        object = ( DRV_SDCARD1_Initialize(&initData), DRV_SDCARD_INDEX );
        
    Static single-client configurations also eliminate the client handle 
    parameter.  However, the index (the driver-object index) is "returned" from
    the "Open" routine to allow code written for multi-client drivers to build
    when using a static single-open configuration.
    
    Example:
    
        handle = DRV_SDCARD_Open(DRV_SDCARD_INDEX_1, intent);
        
    Becomes:
    
        handle = ( DRV_SDCARD_Open(intent), DRV_SDCARD_INDEX );
*/

#if !defined(DRV_SDCARD_INSTANCES_NUMBER) // static

    #if !defined(DRV_SDCARD_CLIENTS_NUMBER) // single client

        #define DRV_SDCARD_Initialize(sysID, inData)          (_DRV_SDCARD_MAKE_NAME(Initialize)(inData), DRV_SDCARD_INDEX)

        #define DRV_SDCARD_Reinitialize(sysObj, inData)        _DRV_SDCARD_MAKE_NAME(Reinitialize)(inData)

        #define DRV_SDCARD_Deinitialize(sysObj)                _DRV_SDCARD_MAKE_NAME(Deinitialize)()

        #define DRV_SDCARD_Status(sysObj)                      _DRV_SDCARD_MAKE_NAME(Status)()

        #define DRV_SDCARD_Tasks(sysObj)                       _DRV_SDCARD_MAKE_NAME(Tasks)()

        #define DRV_SDCARD_Open(sysID, intent)                (_DRV_SDCARD_MAKE_NAME(Open)(intent), DRV_SDCARD_INDEX)

        #define DRV_SDCARD_Close(handle)                       _DRV_SDCARD_MAKE_NAME(Close)()

        #define DRV_SDCARD_ClientStatus(handle)                _DRV_SDCARD_MAKE_NAME(ClientStatus)()

    #else // multi-client

        #define DRV_SDCARD_Initialize(sysID, inData)          (_DRV_SDCARD_MAKE_NAME(Initialize)(inData), DRV_SDCARD_INDEX)

        #define DRV_SDCARD_Reinitialize(sysObj, inData)        _DRV_SDCARD_MAKE_NAME(Reinitialize)(inData)

        #define DRV_SDCARD_Deinitialize(sysObj)                _DRV_SDCARD_MAKE_NAME(Deinitialize)()

        #define DRV_SDCARD_Status(sysObj)                      _DRV_SDCARD_MAKE_NAME(Status)()

        #define DRV_SDCARD_Tasks(sysObj)                       _DRV_SDCARD_MAKE_NAME(Tasks)()

        #define DRV_SDCARD_Open(sysID, intent)                (_DRV_SDCARD_MAKE_NAME(Open)(intent))

        #define DRV_SDCARD_Close(handle)                       _DRV_SDCARD_MAKE_NAME(Close)(handle)

        #define DRV_SDCARD_ClientStatus(handle)                _DRV_SDCARD_MAKE_NAME(ClientStatus)(handle)

        #define DRV_SDCARD_VersionGet(sysID)                   _DRV_SDCARD_MAKE_NAME(VersionGet)()

        #define DRV_SDCARD_VersionStrGet(sysID)                _DRV_SDCARD_MAKE_NAME(VersionStrGet)()

	#endif

#else // Dynamic Build

    // No Change in the Naming convention

#endif


// *****************************************************************************
// *****************************************************************************
// Section: SDCARD Driver API Mapping
// *****************************************************************************
// *****************************************************************************

/* This section maps the Specific SDCARD DRV APIs to the appropriate PLIB APIs */



#endif // #ifndef _DRV_SDCARD_MAPPING_H

/*******************************************************************************
 End of File
*/

