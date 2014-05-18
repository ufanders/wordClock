/*******************************************************************************
  USB Device Driver Mapping File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usb_mapping.h

  Summary:
    USB Device Driver Mapping File

  Description:
    The USB Device Driver mapping file maps the prototypes in "drv_usb.h" to 
    static variants of these routines appropriate for the selected configuration. 
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip  Technology  Inc.   All  rights  reserved.

Microchip licenses to  you  the  right  to  use,  modify,  copy  and  distribute
Software only when embedded on a Microchip  microcontroller  or  digital  signal
controller  that  is  integrated  into  your  product  or  third  party  product
(pursuant to the  sublicense  terms  in  the  accompanying  license  agreement).

You should refer  to  the  license  agreement  accompanying  this  Software  for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
//DOM-IGNORE-END

#ifndef _DRV_USB_MAPPING_H
#define _DRV_USB_MAPPING_H

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
#include "driver/usb/drv_usb_static.h"


// *****************************************************************************
// *****************************************************************************
// Section: USB Driver Static API Name Generation
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: _DRV_USB_MAKE_NAME(name)

  Summary:
    Creates an instance-specific static interface name

  Description:
     This macro creates the instance-specific name of the given static interface
     routine by inserting the index number into the name.

     Example DRV_USB_Initialize becomes DRV_USB1_Initialize for an index of 1.

  Remarks:
*/

#if !defined(DRV_USB_INSTANCES_NUMBER)

    /* If the DRV_USB_INSTANCES_NUMBER macro is not defined, then this
     * means that the static mode is desired. A USB driver instance
     * only supports one client. So a static build would always be
     * static single. */
    #define _DRV_USB_STATIC_API_SINGLE(index, name)     DRV_USB ## index ## _ ## name
    #define _DRV_USB_STATIC_API(index, name)            _DRV_USB_STATIC_API_SINGLE(index, name)
    #define _DRV_USB_MAKE_NAME(name)                    _DRV_USB_STATIC_API(DRV_USB_INDEX, name)

#else 

    /* This means that a dynamic driver is requested */
    #define _DRV_USB_MAKE_NAME(name)                         DRV_USB_ ## name

#endif


// *****************************************************************************
// *****************************************************************************
// Section: USB Driver Static API Mapping
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Static Interface Mapping

  Summary:
    Maps the dynamic interface calls to appropriate static interface

  Description:
    These macros map calls to the dynamic interface routines to calls to the 
    appropriate instance-specific static interface routine when a static build
    (DRV_USB_INSTANCES_NUMBER is not defined) is configured.
    
    Example:
    
        DRV_USB_Status(DRV_USB_INDEX_1);
        
    Becomes:
    
        DRV_USB_Status();
        
  Remarks:
    Static configuration eliminate the need to pass the object parameter.  
    However, the index is "returned" as the object handle (from the 
    "Initialize" routine) to allow code written for the dynamic API to continue
    to build when using a static configuration.
    
    Example:
    
        object = DRV_USB_Initialize(DRV_USB_INDEX_1, &initData);
        
    Becomes:
    
        object = ( DRV_USB1_Initialize(&initData), DRV_USB_INDEX );
        
    Static single-client configurations also eliminate the client handle 
    parameter.  However, the index (the driver-object index) is "returned" from
    the "Open" routine to allow code written for multi-client drivers to build
    when using a static single-open configuration.
    
    Example:
    
        handle = DRV_USB_Open(DRV_USB_INDEX_1, intent);
        
    Becomes:
    
        handle = ( DRV_USB_Open(intent), DRV_USB_INDEX );
*/

#if !defined(DRV_USB_INSTANCES_NUMBER) // static

    #define _DRV_USB_DEVICE_ENDPOINT_ALL 17

    #define DRV_USB_Initialize(sysID, inData)          (_DRV_USB_MAKE_NAME(Initialize)(inData), DRV_USB_INDEX)  
    #define DRV_USB_Deinitialize(sysObj)                _DRV_USB_MAKE_NAME(Deinitialize)()
    #define DRV_USB_Status(sysObj)                      _DRV_USB_MAKE_NAME(Status)()
    #define DRV_USB_Tasks_ISR(sysObj)                   _DRV_USB_MAKE_NAME(Tasks_ISR)()
    #define DRV_USB_Open(sysID, intent)                 _DRV_USB_MAKE_NAME(Open)(intent)
    #define DRV_USB_Close(handle)                       _DRV_USB_MAKE_NAME(Close)(handle)
    #define DRV_USB_ClientStatus(handle)                _DRV_USB_MAKE_NAME(ClientStatus)(handle)
    #define DRV_USB_ClientEventCallBackSet(w, x, y)     _DRV_USB_MAKE_NAME(ClientEventCallBackSet)(w, x, y)
    #define DRV_USB_DEVICE_AddressSet(handle, address)  _DRV_USB_MAKE_NAME(DEVICE_AddressSet)(handle, address)
    #define DRV_USB_DEVICE_CurrentSpeedGet(handle)      _DRV_USB_MAKE_NAME(DEVICE_CurrentSpeedGet)(handle)
    #define DRV_USB_DEVICE_Attach(handle)               _DRV_USB_MAKE_NAME(DEVICE_Attach)(handle)
    #define DRV_USB_DEVICE_Detach(handle)               _DRV_USB_MAKE_NAME(DEVICE_Detach)(handle)
    #define DRV_USB_DEVICE_EndpointEnable(w, x, y, z)   _DRV_USB_MAKE_NAME(DEVICE_EndpointEnable)(w, x, y, z)
    #define DRV_USB_DEVICE_EndpointDisable(w, x)        _DRV_USB_MAKE_NAME(DEVICE_EndpointDisable)(w, x)
    #define DRV_USB_DEVICE_EndpointIsEnabled(w, x)      _DRV_USB_MAKE_NAME(DEVICE_EndpointIsEnabled)(w, x)
    #define DRV_USB_DEVICE_EndpointIsStalled(w, x)      _DRV_USB_MAKE_NAME(DEVICE_EndpointIsStalled)(w, x)
    #define DRV_USB_DEVICE_IRPSubmit(w, x, y)           _DRV_USB_MAKE_NAME(DEVICE_IRPSubmit)(w, x, y)
    #define DRV_USB_DEVICE_IRPCancelAll(w, x)           _DRV_USB_MAKE_NAME(DEVICE_IRPCancelAll)(w, x)
    #define DRV_USB_DEVICE_EndpointStall(w, x)          _DRV_USB_MAKE_NAME(DEVICE_EndpointStall)(w, x)
    #define DRV_USB_DEVICE_EndpointStallClear(w, x)     _DRV_USB_MAKE_NAME(DEVICE_EndpointStallClear)(w, x)
    #define DRV_USB_DEVICE_EndpointDisableAll(w)        _DRV_USB_MAKE_NAME(DEVICE_EndpointDisableAll)(w)

#else // Dynamic Build

    /**************************************************
     * The DRV_USB_DEVICE_EndpointDisableAll() function
     * is really the DRV_USB_DEVICE_EndpointDisable()
     * function called with a special parameter.
     **************************************************/
    #define _DRV_USB_DEVICE_ENDPOINT_ALL 17
    #define DRV_USB_DEVICE_EndpointDisableAll(x)  DRV_USB_DEVICE_EndpointDisable(x, _DRV_USB_DEVICE_ENDPOINT_ALL)

    // No Change in the Naming convention

#endif



#endif
