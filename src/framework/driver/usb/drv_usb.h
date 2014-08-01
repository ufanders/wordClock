/*************************************************************************
  Company:
    Microchip Technology Inc.
    
  File Name:
    drv_usb.h
	
  Summary:
    USB Device Driver Interface File
	
  Description:
    USB Device Driver Interface
    
    The USB device driver provides a simple interface to manage the "USB"
    peripheral. This file defines the interface definitions and prototypes
    for the USB driver.                                                   
  *************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_USB_H
#define _DRV_USB_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files (continued at end of file)
// *****************************************************************************
// *****************************************************************************
/*  This section lists the other files that are included in this file.  However,
    please see the bottom of the file for additional implementation header files
    that are also included
*/

#include <stdint.h>
#include <stdbool.h>
#include "usb/usb_common.h"
#include "usb/usb_chapter_9.h"
#include "driver/driver_common.h"
#include "system/int/sys_int.h"
#include "system/common/sys_module.h"

// *****************************************************************************
// *****************************************************************************
// Section: Inclusion of peripheral library based on the architecture
// *****************************************************************************
// *****************************************************************************

/*DOM-IGNORE-BEGIN*/
#if defined(__PIC32MX__)
#include "peripheral/usb/plib_usb.h"
#define DRV_USB_ENDPOINT_TABLE_ENTRY_SIZE /*DOM-IGNORE-BEGIN*/ 32 /*DOM-IGNORE-END*/
#endif

#if defined(__PIC32MZ__)
#include "peripheral/usbhs/plib_usbhs.h"
#define DRV_USB_ENDPOINT_TABLE_ENTRY_SIZE /*DOM-IGNORE-BEGIN*/ 0 /*DOM-IGNORE-END*/
#endif
/*DOM-IGNORE-END*/


// *****************************************************************************
// *****************************************************************************
// Section: USB Device Driver Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* USB Driver Module Index Numbers

  Summary:
    USB driver index definitions

  Description:
    These constants provide USB driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals
    and should be passed into the DRV_USB_Initialize and DRV_USB_Open
    routines to identify the driver instance in use. These are not
    indicative of the number of modules that are actually supported by the
    microcontroller.
*/

#define DRV_USB_INDEX_0         0
#define DRV_USB_INDEX_1         1

// *****************************************************************************
// *****************************************************************************
// Section: USB Device Driver Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* USB Driver Host Pipe Handle

  Summary:
    Defines the USB Driver Host Pipe handle type

  Description:
    Defines the USB Driver Host Pipe handle type

  Remarks:
    None.
*/
typedef uintptr_t DRV_USB_HOST_PIPE_HANDLE;

// *****************************************************************************
/* USB Driver Host Pipe Handle Invalid

  Summary:
    Defines the USB Driver Host Pipe Invalid handle 

  Description:
    Defines the USB Driver Host Pipe Invalid handle 

  Remarks:
    None.
*/

#define DRV_USB_HOST_PIPE_HANDLE_INVALID ((DRV_USB_HOST_PIPE_HANDLE)(-1))

// *****************************************************************************
/* USB Driver Events Enumeration

  Summary:
    Identifies the different events that the USB Driver provides.

  Description:
    Identifies the different events that the USB Driver provides.

  Remarks:
    None.
*/


typedef enum
{
    /* Bus error occurred and was reported */

    DRV_USB_EVENT_ERROR = 1,

    /* Host has issued a device reset */

    DRV_USB_EVENT_RESET_DETECT,

    /* Resume detected while USB in suspend mode */

    DRV_USB_EVENT_RESUME_DETECT,

    /* Idle detected */
    
    DRV_USB_EVENT_IDLE_DETECT,

    /* Stall handshake has occurred */
    
    DRV_USB_EVENT_STALL,

    /* Either Device received SOF or
     * SOF threshold was reached in the Host
     * mode operation */
   
    DRV_USB_EVENT_SOF_DETECT,

    /* Host got attach */

    DRV_USB_EVENT_HOST_ATTACH,

    /* Host got detach */

    DRV_USB_EVENT_HOST_DETACH,


}DRV_USB_EVENT;

// *****************************************************************************
/* USB Operating Modes Enumeration

  Summary:
    Provides enumeration of operating modes supported by USB.

  Description:
    This data type provides enumeration of the operating modes supported by the
    USB module.

  Remarks:
    PIC18 devices only support DRV_USB_OPMODE_DEVICE.
*/

typedef enum
{
    // None
    DRV_USB_OPMODE_NONE   /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/ ,
    // Device
    DRV_USB_OPMODE_DEVICE /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/ ,
    // Host
    DRV_USB_OPMODE_HOST   /*DOM-IGNORE-BEGIN*/ = 2 /*DOM-IGNORE-END*/ ,
    // OTG
    DRV_USB_OPMODE_OTG    /*DOM-IGNORE-BEGIN*/ = 3 /*DOM-IGNORE-END*/

} DRV_USB_OPMODES;

// *****************************************************************************
/* Type of the USB Event Callback Function

  Summary:
    Type of the USB event callback function

  Description:
    Type of the USB event callback function. The client should register an event
    callback function of this type when it intends to receive events from the USB
    driver. The event callback function is registered using the
    DRV_USB_ClientEventCallBackSet() function. 

  Parameters:
    hClient    - handle to driver client that registered this callback function
    eventType  - Event type
    eventData  - Event relevant data

  Returns:
    None.

  Remarks:
    None.

*/

typedef void (*DRV_USB_EVENT_CALLBACK) ( uintptr_t hClient, DRV_USB_EVENT  eventType,
                                         void * eventData   );

// *****************************************************************************
/* USB Device Driver Initialization Data

  Summary:
    Contains all the data necessary to initialize the USB device

  Description:
    This structure contains all the data necessary to initialize the USB
    driver.

  Remarks:
    A pointer to a structure of this format containing the desired
    initialization data must be passed into the DRV_USB_Initialize routine.
*/

typedef struct
{
    SYS_MODULE_INIT     moduleInit;     // System module initialization
    uint8_t             usbID;          // Identifies peripheral (PLIB-level) ID
    bool                stopInIdle;     // Boolean flag: true -> Stop USB module in Idle Mode
    bool                suspendInSleep; // Boolean flag: true -> Suspend USB in Sleep Mode
    INT_SOURCE          interruptSource;// Interrupt Source for USB module
    USB_SPEED           operationSpeed; // USB Operation speed
    DRV_USB_OPMODES     operationMode;  // USB module operating mode
    void                *endpointTable; // Endpoint Descriptor Table buffer 

} DRV_USB_INIT;

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - System Level
// *****************************************************************************
// *****************************************************************************


/********************************************************************************
  Function:
       SYS_MODULE_OBJ DRV_USB_Initialize( const SYS_MODULE_INDEX        drvIndex,
                                          const SYS_MODULE_INIT * const init    )
    
  Summary:
    Initializes the USB driver
  Description:
    This routine initializes the USB driver, making it ready for clients to
    open and use it.
  Conditions:
    None.
  Input:
    drvIndex -  Ordinal number of driver instance to be initialized
    init -      Pointer to a data structure containing any data necessary to
                initialize the driver. This should be a DRV_USB_INIT
                structure reference typecast to SYS_MODULE_INIT reference.
  Return:
    If successful, returns a valid handle to a driver object. Otherwise, it
    returns SYS_MODULE_OBJ_INVALID. The returned object must be passed as
    argument to DRV_USB_Status routines.
  Example:
    <code>
     // The following code snippet shows an example initialization of the
     // driver. The USB module to be used is USB1.  The module should not
     // automatically suspend when the  microcontroller enters Sleep mode.
     // The module should continue  operation when the module enters Idle
     // mode.  The power state is set to run at full clock speeds. Device
     // Mode operation should be at FULL speed.
    
    DRV_USB_INIT moduleInit;
    
    usbInitData.usbID               = USB_ID_1;
    usbInitData.opMode              = DRV_USB_OPERATION_MODE_DEVICE;
    usbInitData.pingpongMode        = USB_PING_PONG_FULL_PING_PONG;
    usbInitData.stopInIdle          = false;
    usbInitData.suspendInSleep      = false;
    usbInitData.operationSpeed      = USB_SPEED_FULL;
    usbInitData.interruptSource     = INT_SOURCE_USB;
    
    usbInitData.sysModuleInit.powerState = SYS_MODULE_POWER_RUN_FULL ;
    
    // This is how this data structure is passed to the initialize
    // function.
    
    DRV_USB_Initialize(DRV_USB_INDEX_0, (SYS_MODULE_INIT *) &usbInitData);
    
    </code>
  Remarks:
    This routine must be called before any other USB routine is called.
    This routine should only be called once during system initialization
    unless DRV_USB_Deinitialize is called to deinitialize the driver
    instance. If the driver is built for static mode, then the
    initialization data structure is ignored. The static overrides provided
    in system_config.h are considered instead.                                   
  ********************************************************************************/

SYS_MODULE_OBJ DRV_USB_Initialize ( const SYS_MODULE_INDEX        drvIndex,
                                    const SYS_MODULE_INIT * const init    );

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_USB_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the USB driver module

  Description:
    This routine provides the current status of the USB driver module.

  Precondition:
    Function DRV_USB_Initialize must have been called before calling this
    function

  Parameters:
    object  - Driver object handle, returned from the DRV_USB_Initialize routine

  Returns:
    * SYS_STATUS_READY          - Indicates that the driver is ready
    * SYS_STATUS_UNINITIALIZED  - Indicates that the driver has never been 
                                  initialized

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_USB_Initialize
    SYS_STATUS          status;
    DRV_USB_INIT        moduleInit;

    usbInitData.usbID               = USB_ID_1;
    usbInitData.opMode              = DRV_USB_OPERATION_MODE_DEVICE;
    usbInitData.stopInIdle          = false;
    usbInitData.suspendInSleep      = false;
    usbInitData.operationSpeed      = USB_SPEED_FULL;
    usbInitData.interruptSource     = INT_SOURCE_USB;

    usbInitData.sysModuleInit.powerState = SYS_MODULE_POWER_RUN_FULL ;

    // This is how this data structure is passed to the initialize
    // function. 

    object = DRV_USB_Initialize(DRV_USB_INDEX_0, 
                        (SYS_MODULE_INIT *) &usbInitData);

    status = DRV_USB_Status(object);
    if(SYS_STATUS_READY == status)
    {
        // Driver is ready to be opened.
    }

    </code>

  Remarks:
    None.
*/

SYS_STATUS DRV_USB_Status ( SYS_MODULE_OBJ object );

// *****************************************************************************
/* Function:
    void DRV_USB_Tasks( SYS_MODULE_OBJ object )

  Summary:
    Maintains the driver's state machine when the driver is configured for 
    polled mode.

  Description:
    Maintains the driver's state machine when the driver is configured for 
    polled mode. This function should be called from the system tasks routine.

  Precondition:
    The DRV_USB_Initialize routine must have been called for the specified
    USB driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_USB_Initialize)

  Returns:
    None

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_USB_Initialize

    while (true)
    {
        DRV_USB_Tasks(object);

        // Do other tasks
    }
    </code>

  Remarks:
    This routine is normally not called  directly  by  an  application.   It  is
    called by the system's Tasks routine (SYS_Tasks). This function will never 
    block.  

*/

void DRV_USB_Tasks(SYS_MODULE_OBJ object);

// *****************************************************************************
/* Function:
    void DRV_USB_Tasks_ISR( SYS_MODULE_OBJ object )

  Summary:
    Maintains the driver's state machine and implements its ISR

  Description:
    This routine is used to maintain the driver's internal state machine and
    implement its ISR for interrupt-driven implementations.

  Precondition:
    The DRV_USB_Initialize routine must have been called for the specified
    USB driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_USB_Initialize)

  Returns:
    None

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_USB_Initialize

    while (true)
    {
        DRV_USB_Tasks_ISR (object);

        // Do other tasks
    }
    </code>

  Remarks:
    This routine should be called from the USB interrupt service routine. In case
    of multiple USB modules, it should be ensured that the correct USB driver
    system module object is passed to the this routine.
*/

void DRV_USB_Tasks_ISR( SYS_MODULE_OBJ object );

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client Level
// *****************************************************************************
// *****************************************************************************

/******************************************************************************
  Function:
       DRV_HANDLE DRV_USB_Open( const SYS_MODULE_INDEX drvIndex,
                                const DRV_IO_INTENT    intent  )
    
  Summary:
    Opens the specified USB driver instance and returns a handle to it
  Description:
    This routine opens the specified USB driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The intent flag
    should always be
    DRV_IO_INTENT_EXCLUSIVE|DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_NON_BLOCKING.
    Any other setting of the intent flag will return a invalid driver
    handle. A driver instance can only support one client. Trying to open a
    driver that has an existing client will result in an unsuccessful
    function call.
  Conditions:
    Function DRV_USB_Initialize must have been called before calling this
    function.
  Input:
    drvIndex -  Driver instance that client will use
    intent -    Should always be
                DRV_IO_INTENT_EXCLUSIVE|DRV_IO_INTENT_READWRITE|
                DRV_IO_INTENT_NON_BLOCKING.
  Return:
    If successful, valid driver handle for the client to use when
    interfacing with the driver. If an error occurs, the return value is
    DRV_HANDLE_INVALID.
  Example:
    <code>
    // Shows an example of how to open the driver.
    SYS_MODULE_OBJ      object;     // Returned from DRV_USB_Initialize
    SYS_STATUS          status;
    DRV_USB_INIT        moduleInit;
    DRV_HANDLE          handle;
    
    usbInitData.usbID               = USB_ID_1;
    usbInitData.opMode              = DRV_USB_OPERATION_MODE_DEVICE;
    usbInitData.stopInIdle          = false;
    usbInitData.operationSpeed      = USB_SPEED_FULL;
    usbInitData.suspendInSleep      = false;
    usbInitData.interruptSource     = INT_SOURCE_USB;
    
    usbInitData.sysModuleInit.powerState = SYS_MODULE_POWER_RUN_FULL ;
    
    // This is how this data structure is passed to the initialize
    // function.
    
    object = DRV_USB_Initialize(DRV_USB_INDEX_0,
                        (SYS_MODULE_INIT *) &usbInitData);
    
    status = DRV_USB_Status(object);
    if(SYS_STATUS_READY == status)
    {
        // Driver is ready to be opened.
        handle = DRV_USB_Open(DRV_USB_INDEX_0,
                                DRV_IO_INTENT_EXCLUSIVE|
                                DRV_IO_INTENT_READWRITE|
                                DRV_IO_INTENT_NON_BLOCKING);
        SYS_ASSERT(DRV_HANDLE_INVALID != handle,"Bad handle returned");
    }
    
    
    </code>
  Remarks:
    The handle returned is valid until the DRV_USB_Close routine is called.    
  ******************************************************************************/

DRV_HANDLE DRV_USB_Open( const SYS_MODULE_INDEX drvIndex,
                         const DRV_IO_INTENT    intent  );

// *****************************************************************************
/* Function:
    void DRV_USB_Close( DRV_HANDLE handle )

  Summary:
    Closes an opened-instance of the USB driver

  Description:
    This routine closes an opened-instance of the USB driver, invalidating the
    handle.

  Precondition:
    The DRV_USB_Initialize routine must  have  been  called  for  the  specified
    USB driver instance.  DRV_USB_Open must have been called to obtain  a  valid
    opened device handle.

  Parameters:
    handle  - Client's driver handle (returned from DRV_USB_Open)

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_USB_Open

    DRV_USB_Close(handle);
    </code>

  Remarks:
    After calling this routine, the handle passed in "handle" must not  be  used
    with any of the remaining driver routines.  A new handle must be obtained by
    calling  DRV_USB_Open  before  the  caller  may  use   the   driver   again.
*/

void DRV_USB_Close( DRV_HANDLE handle );

/*************************************************************************************
  Function:
       void DRV_USB_ClientEventCallBackSet( DRV_HANDLE             handle          ,
                                             uintptr_t              hReferenceData  ,
                                             DRV_USB_EVENT_CALLBACK myEventCallBack );
  Summary:
    This function sets up the event callback function that is invoked by
    the USB controller driver to notify the a client of USB bus events.
  Description:
    This function sets up the event callback function that is invoked by
    the USB controller driver to notify the client of USB bus events. The
    call back is disabled by not calling this function after a USB_DRV_Open
    or by passing NULL for the myEventCallBack argument. When the callback
    function is called, the hReferenceData argument is returned.
  Conditions:
    None.
  Input:
    handle -           Client's driver handle (returned from DRV_USB_Open)
    hReferenceData -   object (could be a pointer) that is returned with the
                       callback.
    myEventCallBack -  Call back function for all USB events.
  Return:
    None.
  Example:
    <code>
     // Set the client event callback for the Device Layer.
     // The USBDeviceLayerEventHandler() function is the event
     // handler. When this event handler is invoked by the
     // driver, the driver returns back the second argument
     // specified in the below function (which in this case
     // is the device layer data structure). This allows the
     // application firmware to identify, as an example, the
     // device layer object associated with this callback
     //
    
    DRV_USB_ClientEventCallBackSet(myUSBDevice.usbDriverHandle,
                                    (uintptr_t)&myUSBDevice,
                                    USBDeviceLayerEventHandler);
    
    </code>
  Remarks:
    Typical usage of the USB Driver requires a client to register a
    callback.                                                                         
  *************************************************************************************/

void DRV_USB_ClientEventCallBackSet( DRV_HANDLE   handle          ,
                                      uintptr_t    hReferenceData ,
                                      DRV_USB_EVENT_CALLBACK myEventCallBack );

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Device Mode Operation
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void DRV_USB_DEVICE_AddressSet(DRV_HANDLE handle, uint8_t address);

  Summary:
    This function will set the USB module address that is obtained from the host.

  Description:
    This function will set the USB module address  that  is  obtained  from  the
    host in a setup transaction.  The address is obtained from  the  SET_ADDRESS
    command issued by the host.   The  primary  (first)  client  of  the  driver
    uses this function to set the module's USB address after decoding the  setup
    transaction from the host.

  Precondition:
    None.

  Parameters:
    handle       - Client's driver handle (returned from DRV_USB_Open)
    address      - The address of this module on the USB bus

  Returns:
    None.

  Example:
    <code>
    // This function should be called by the first client of the driver,
    // which is typically the device layer. The address to set is obtained
    // from the host during enumeration.

    DRV_USB_DEVICE_AddressSet(deviceLayer, 4);
    </code>

  Remarks:
    None.
*/

void DRV_USB_DEVICE_AddressSet(DRV_HANDLE handle, uint8_t address);

// *****************************************************************************
/* Function:
    USB_SPEED DRV_USB_DEVICE_CurrentSpeedGet(DRV_HANDLE handle);

  Summary:
    This function will return the USB speed at which device is operating.

  Description:
    This function will return the USB speed at which device is operating.

  Precondition:
    Only valid after device is attached to host.

  Parameters:
    handle       - Client's driver handle (returned from DRV_USB_Open)

  Returns:
    Returns a member of the USB_SPEED type.  

  Example:
    <code>
    // Get the current speed.
    
    USB_SPEED deviceSpeed;
    
    deviceSpeed = DRV_USB_DEVICE_CurrentSpeedGet(deviceLayer);
    
    if(deviceLayer == USB_SPEED_HIGH)
    {
        // Possibly adjust buffers for higher throughput.
    } 
    
    </code>

  Remarks:
    None.
*/

USB_SPEED DRV_USB_DEVICE_CurrentSpeedGet(DRV_HANDLE handle);

// *****************************************************************************
/* Function:
    uint16_t DRV_USB_DEVICE_SOFNumberGet(DRV_HANDLE handle);

  Summary:
    This function will return the USB SOF packet number.

  Description:
    This function will return the USB SOF packet number..

  Precondition:
    Only valid after device is attached to host.

  Parameters:
    handle       - Client's driver handle (returned from DRV_USB_Open)

  Returns:
    Returns SOF packet number.

  Example:
    <code>
    // Get the current speed.

    uint16_t sofNumber;

    sofNumber = DRV_USB_DEVICE_SOFNumberGet(DRV_HANDLE handle);

    if(sofNumber > 0)
    {
        // use the packet number.
    }
    else
    {
        // error
    }

    </code>

  Remarks:
    None.
*/

uint16_t DRV_USB_DEVICE_SOFNumberGet(DRV_HANDLE handle);

// *****************************************************************************
/* Function:
    void DRV_USB_DEVICE_Attach(DRV_HANDLE handle);

  Summary:
    This function will attach the device to the USB. 

  Description:
    This function will attach the device to the USB.  It does this  by  enabling
    the pull up resistors on the D+ or D- lines. This function should be called
    when the driver client is ready  to  receive  communication  from  the  host
    (typically after all initialization is complete). 

  Precondition:
    Client handle should be valid.

  Parameters:
    handle       - Client's driver handle (returned from DRV_USB_Open)

  Returns:
    None.

  Example:
    <code>
    // Open the device driver and attach the device to the USB.
    SYS_MODULE_OBJ      object;     // Returned from DRV_USB_Initialize
    SYS_STATUS          status;
    DRV_USB_INIT        moduleInit;
    DRV_HANDLE          handle;

    usbInitData.usbID               = USB_ID_1;
    usbInitData.opMode              = DRV_USB_OPERATION_MODE_DEVICE;
    usbInitData.stopInIdle          = false;
    usbInitData.suspendInSleep      = false;
    usbInitData.operationSpeed      = USB_SPEED_FULL;
    usbInitData.interruptSource     = INT_SOURCE_USB;

    usbInitData.sysModuleInit.powerState = SYS_MODULE_POWER_RUN_FULL ;

    // This is how this data structure is passed to the initialize
    // function. 

    object = DRV_USB_Initialize(DRV_USB_INDEX_0, 
                        (SYS_MODULE_INIT *) &usbInitData);

    status = DRV_USB_Status(object);
    if(SYS_STATUS_READY == status)
    {
        // Driver is ready to be opened.
        handle = DRV_USB_Open(DRV_USB_INDEX_0, 
                                DRV_IO_INTENT_EXCLUSIVE|
                                DRV_IO_INTENT_READWRITE|
                                DRV_IO_INTENT_NON_BLOCKING);
        SYS_ASSERT(DRV_HANDLE_INVALID != handle,"Bad handle returned");

        // Register a callback
        DRV_USB_ClientEventCallBackSet(handle, (uintptr_t)&myDeviceLayer,
                                                MyDeviceLayerEventCallback);

        // Perform any other initialization here and then attach the device
        // to the bus.
        DRV_USB_DEVICE_Attach(handle);
    }

    </code>

  Remarks:
    None.
*/

void DRV_USB_DEVICE_Attach(DRV_HANDLE handle);

// *****************************************************************************
/* Function:
    void DRV_USB_DEVICE_Detach(DRV_HANDLE handle);

  Summary:
    This function will detach the device from the USB. 

  Description:
    This function will detach  the  device  from  the  USB.   It  does  this  by
    disabling the pull up resistors on the D+ or D- lines. This function should
    be called when the application wants to disconnect the device  from  the  bus
    (typically to implement a soft detach or switch  to  host  mode  operation).

  Precondition:
    Client handle should be valid.

  Parameters:
    handle       - Client's driver handle (returned from DRV_USB_Open)

  Returns:
    None.

  Example:
    <code>
    // Detach the device from the USB
    DRV_USB_DEVICE_Detach(handle);

    </code>

  Remarks:
    None.
*/
void DRV_USB_DEVICE_Detach(DRV_HANDLE handle);

/****************************************************************************
  Function:
       USB_ERROR DRV_USB_DEVICE_EndpointEnable(DRV_HANDLE handle,
                                           USB_ENDPOINT endpointAndDirection,
                                           USB_TRANSFER_TYPE transferType,
                                           uint16_t endpointSize);
    
  Summary:
    This function enables a endpoint for the specified direction and
    endpoint size.
  Description:
    This function enables a endpoint for the specified direction and
    endpoint size. The function will enable the endpoint for communication
    in one direction at a time. It must be called twice if the endpoint is
    required to communicate in both the directions, except for control
    endpoints. If the endpoint type is a control endpoint, the endpoint is
    always bi-directional and the function needs to be called only once.
    The size of the endpoint must match the wMaxPacketSize reported in the
    endpoint descriptor for this endpoint. The function does not check if
    the endpoint is already in use. It is the client's responsibility to
    make sure that a endpoint is not accidentally re-used.
  Conditions:
    Client handle should be valid.
  Input:
    handle -                Client's driver handle (returned from
                            DRV_USB_Open)
    endpointAndDirection -  Specifies the endpoint and direction.
    transferType -          Should be USB_TRANSFER_TYPE_CONTROL for control
                            endpoint, USB_TRANSFER_TYPE_BULK for bulk
                            endpoint, USB_TRANSFER_TYPE_INTERRUPT for
                            interrupt endpoint and
                            USB_TRANSFER_TYPE_ISOCHRONOUS for isochronous
                            endpoint.
    endpointSize -          Maximum size (in bytes) of the endpoint as
                            reported in the endpoint descriptor.
  Return:
    USB_ERROR_NONE - The endpoint was successfully enabled.
    USB_ERROR_DEVICE_ENDPOINT_INVALID - If the endpoint that is being
    accessed is out of the valid endpoint defined for this driver instance.
  Example:
    <code>
    // This code snippet shows an example of how to enable endpoint
    // 0 for control transfers. Note that for a control endpoint, the
    // direction parameter is ignored. A control endpoint is always
    // bi-directional. Endpoint size is 64 bytes.
    
    uint8_t ep;
    
    ep = USB_ENDPOINT_AND_DIRECTION(USB_DATA_DIRECTION_DEVICE_TO_HOST, 0);
    
    DRV_USB_DEVICE_EndpointEnable(handle, ep, USB_TRANSFER_TYPE_CONTROL, 64);
    
    // This code snippet shows an example of how to set up a endpoint
    // for BULK IN transfer. For an IN transfer, data moves from device
    // to host. In this example, endpoint 1 is enabled. The maximum
    // packet size is 64.
    
    uint8_t ep;
    
    ep = USB_ENDPOINT_AND_DIRECTION(USB_DATA_DIRECTION_DEVICE_TO_HOST, 1);
    
    DRV_USB_DEVICE_EndpointEnable(handle, ep, USB_TRANSFER_TYPE_BULK, 64);
    
    // If endpoint 1 must also be set up for BULK OUT, then the enable
    // function must be called again, as shown here.
    
    ep = USB_ENDPOINT_AND_DIRECTION(USB_DATA_DIRECTION_HOST_TO_DEVICE, 1);
    
    DRV_USB_DEVICE_EndpointEnable(handle, ep, USB_TRANSFER_TYPE_BULK, 64);
    </code>
  Remarks:
    None.                                                                    
  ****************************************************************************/
USB_ERROR DRV_USB_DEVICE_EndpointEnable(DRV_HANDLE handle, 
                                    USB_ENDPOINT endpointAndDirection, 
                                    USB_TRANSFER_TYPE transferType,
                                    uint16_t endpointSize);

/****************************************************************************
  Function:
       USB_ERROR DRV_USB_DEVICE_EndpointDisable(DRV_HANDLE handle,
                                           USB_ENDPOINT endpointAndDirection)
    
  Summary:
    This function disables an endpoint.
  Description:
    This function disables an endpoint. If the endpoint type is control
    type then both directions are disabled. For non-control endpoints, the
    function disables one direction at a time.
  Conditions:
    Client handle should be valid.
  Input:
    handle -                Client's driver handle (returned from
                            DRV_USB_Open)
    endpointAndDirection -  Specifies the endpoint and direction.
  Return:
    USB_ERROR_NONE - The endpoint was successfully enabled.
    USB_ERROR_DEVICE_ENDPOINT_INVALID - If the endpoint that is being
    accessed is out of the valid endpoint defined for this driver instance.
  Example:
    <code>
    // This code snippet shows an example of how to disable
    // a control endpoint. Note that the direction parameter is ignored.
    // For a control endpoint, both the directions are disabled.
    
    USB_ENDPOINT ep;
    
    ep = USB_ENDPOINT_AND_DIRECTION(USB_DATA_DIRECTION_DEVICE_TO_HOST, 0);
    
    DRV_USB_DEVICE_EndpointDisable(handle, ep );
    
    // This code snippet shows an example of how to disable a BULK IN
    // endpoint
    
    USB_ENDPOINT ep;
    
    ep = USB_ENDPOINT_AND_DIRECTION(USB_DATA_DIRECTION_DEVICE_TO_HOST, 1);
    
    DRV_USB_DEVICE_EndpointDisable(handle, ep );
    
    </code>
  Remarks:
    None.                                                                    
  ****************************************************************************/

USB_ERROR DRV_USB_DEVICE_EndpointDisable(DRV_HANDLE handle, 
                                    USB_ENDPOINT endpointAndDirection);

// *****************************************************************************
/* Function:
    USB_ERROR DRV_USB_DEVICE_EndpointDisableAll(DRV_HANDLE handle) 
  
  Summary:
    This function disables all provisioned non-zero endpoints.

  Description:
    This function disables all provisioned endpoints in both directions. 
  
  Precondition:
    Client handle should be valid.

  Parameters:
    handle                  - Client's driver handle (returned from DRV_USB_Open)

  Returns:
    None.

  Example:
    <code>
    // This code snippet shows an example of how to disable
    // all endpoints. 

    DRV_USB_DEVICE_EndpointDisableAll(handle);
    
    </code>

  Remarks:
    This function is typically called by the USB Device Layer to disable
    all endpoints on detecting bus reset.
*/

USB_ERROR DRV_USB_DEVICE_EndpointDisableAll(DRV_HANDLE handle); 

/****************************************************************************
  Function:
       USB_ERROR DRV_USB_DEVICE_EndpointStall(DRV_HANDLE handle,
                                           USB_ENDPOINT endpointAndDirection)
    
  Summary:
    This function stalls an endpoint in the specified direction.
  Description:
    This function stalls an endpoint in the specified direction.
  Conditions:
    Client handle should be valid.
  Input:
    handle -                Client's driver handle (returned from
                            DRV_USB_Open)
    endpointAndDirection -  Specifies the endpoint and direction.
  Return:
    USB_ERROR_NONE - The endpoint was successfully enabled.
    USB_ERROR_DEVICE_ENDPOINT_INVALID - If the endpoint that is being
    accessed is out of the valid endpoint defined for this driver instance.
  Example:
    <code>
    // This code snippet shows an example of how to stall an endpoint. In
    // this case , endpoint 1 IN direction is stalled.
    
    USB_ENDPOINT ep;
    
    ep = USB_ENDPOINT_AND_DIRECTION(USB_DATA_DIRECTION_DEVICE_TO_HOST, 1);
    
    DRV_USB_DEVICE_EndpointStall(handle, ep);
    
    </code>
  Remarks:
    None.                                                                    
  ****************************************************************************/

USB_ERROR DRV_USB_DEVICE_EndpointStall(DRV_HANDLE handle, 
                                  USB_ENDPOINT endpointAndDirection);

/****************************************************************************
  Function:
       USB_ERROR DRV_USB_DEVICE_EndpointStallClear(DRV_HANDLE handle,
                                           USB_ENDPOINT endpointAndDirection)
    
  Summary:
    This function clears the stall on an endpoint in the specified
    direction.
  Description:
    This function clears the stall on an endpoint in the specified
    direction.
  Conditions:
    Client handle should be valid.
  Input:
    handle -                Client's driver handle (returned from
                            DRV_USB_Open)
    endpointAndDirection -  Specifies the endpoint and direction.
  Return:
    USB_ERROR_NONE - The endpoint was successfully enabled.
    USB_ERROR_DEVICE_ENDPOINT_INVALID - If the endpoint that is being
    accessed is out of the valid endpoint defined for this driver instance.
  Example:
    <code>
    // This code snippet shows an example of how to clear a stall. In this
    // example. the stall on endpoint 1 IN direction is cleared.
    
    USB_ENDPOINT ep;
    
    ep = USB_ENDPOINT_AND_DIRECTION(USB_DATA_DIRECTION_DEVICE_TO_HOST, 1);
    
    DRV_USB_DEVICE_EndpointStallClear(handle, ep);
    
    </code>
  Remarks:
    None.                                                                    
  ****************************************************************************/
USB_ERROR DRV_USB_DEVICE_EndpointStallClear(DRV_HANDLE handle, 
                                       USB_ENDPOINT endpointAndDirection);

/*****************************************************************************
  Function:
      bool DRV_USB_DEVICE_EndpointIsEnabled(DRV_HANDLE handle,
                                           USB_ENDPOINT endpointAndDirection)
    
  Summary:
    This function returns the enable/disable status of the specified
    endpoint and direction.
  Description:
    This function returns the enable/disable status of the specified
    endpoint and direction.
  Conditions:
    Client handle should be valid.
  Input:
    handle -                Client's driver handle (returned from
                            DRV_USB_Open)
    endpointAndDirection -  Specifies the endpoint and direction.
  Return:
    Returns true if the endpoint is enabled, false otherwise.
  Example:
    <code>
    // This code snippet shows an example of how the
    // DRV_USB_DEVICE_EndpointIsEnabled() function can be used to obtain the
    // status of the endpoint 1 and IN direction.
    
    USB_ENDPOINT ep;
    
    ep = USB_ENDPOINT_AND_DIRECTION(USB_DATA_DIRECTION_DEVICE_TO_HOST, 1);
    
    if(DRV_USB_ENDPOINT_STATE_DISABLED ==
                DRV_USB_DEVICE_EndpointIsEnabled(handle, ep))
    {
        // Endpoint is disabled. Enable endpoint.
    
        DRV_USB_DEVICE_EndpointEnable(handle, ep, USB_ENDPOINT_TYPE_BULK, 64);
    
    }
    
    </code>
  Remarks:
    None.                                                                     
  *****************************************************************************/

bool DRV_USB_DEVICE_EndpointIsEnabled(DRV_HANDLE client, 
                                        USB_ENDPOINT endpointAndDirection);

// *****************************************************************************
/* Function:
    bool DRV_USB_DEVICE_EndpointIsStalled(DRV_HANDLE handle, 
                                        USB_ENDPOINT endpointAndDirection) 
  
  Summary:
    This function returns the stall status of the specified endpoint
    and direction.

  Description:
    This function returns the stall status of the specified endpoint
    and direction.
  
  Precondition:
    Client handle should be valid.

  Parameters:
    handle                  - Client's driver handle (returned from DRV_USB_Open)
    endpointAndDirection    - Specifies the endpoint and direction.

  Returns:
    Returns true if endpoint is stalled, false otherwise.

  Example:
    <code>
    // This code snippet shows an example of how the
    // DRV_USB_DEVICE_EndpointIsStalled() function can be used to obtain the
    // stall status of the endpoint 1 and IN direction.

    USB_ENDPOINT ep;

    ep = USB_ENDPOINT_AND_DIRECTION(USB_DATA_DIRECTION_DEVICE_TO_HOST, 1);

    if(true == DRV_USB_DEVICE_EndpointIsStalled (handle, ep))
    {
        // Endpoint stall is enabled. Clear the stall.

        DRV_USB_DEVICE_EndpointStallClear(handle, ep);
        
    }

    </code>

  Remarks:
    None.
*/

bool DRV_USB_DEVICE_EndpointIsStalled(DRV_HANDLE client, USB_ENDPOINT endpoint); 

/***********************************************************************************
  Function:
       USB_ERROR DRV_USB_DEVICE_IRPSubmit(DRV_HANDLE client,
                                            USB_ENDPOINT endpointAndDirection,
                                            USB_DEVICE_IRP * irp);
    
  Summary:
    This function submits a I/O Request Packet (IRP) for processing to the
    USB Driver.
  Description:
    This function submits a I/O Request Packet (IRP) for processing to the
    USB Driver. The IRP allows a client to send and receive data from the
    USB Host. The data will sent or received through the specified
    endpoint. The direction of the data transfer is indicated by the
    direction flag in the endpointAndDirection structure. Submitting an IRP
    arms the endpoint to either send data to or receive data from the host.
    If an IRP is under progress on the endpoint, then the subsequent IRP
    submit operation will result in the IRPs getting queued. The contents
    of the IRP should not be changed until the IRP has been processed.
    
    Particular attention should be paid to the size parameter of IRP. The
    following should be noted:
    
      * The size parameter while sending data to the host can be less than,
        greater than, equal to or be an exact multiple of maximum packet size
        for the endpoint. The maximum packet size for the endpoint determines
        the number of transactions required to process the IRP.
      * If the size parameter while sending data to the host is less than
        maximum packet size, the transfer will complete in one transaction.
      * If the size parameter while sending data to the host is greater
        than maximum packet size, the IRP will be processed in multiple
        transactions.
      * If the size parameter while sending data to the host is equal to or
        an exact multiple of the maximum packet size, the client can optionally
        ask the driver to send a Zero Length packet by specifying the
        USB_DEVICE_IRP_FLAG_DATA_COMPLETE flag as the flag parameter.
      * The size parameter while receiving data from the host must be an
        exact multiple of the maximum packet size of the endpoint. If this is
        not the case, the driver will return a USB_ERROR_IRP_SIZE_INVALID
        result. If while processing the IRP, the driver receives less than
        maximum packet size or a ZLP from the host, the driver considers the
        IRP as processed. The size parameter at this point contains the actual
        amount of data received from the host. The IRP status is returned as
        USB_DEVICE_IRP_STATUS_COMPLETED_SHORT.
      * If a ZLP needs to be sent to host, the IRP size should be specified
        as 0 and flag should be set as USB_DEVICE_IRP_FLAG_DATA_COMPLETE.
      * If the IRP size is an exact multiple of endpoint size, the client
        can request the driver to not send a ZLP, by specifying the
        USB_DEVICE_IRP_FLAG_DATA_PENDING. This flag indicates that there is
        more data pending in this transfer.
      * Specifying a size less than endpoint size along with the
        USB_DEVICE_IRP_FLAG_DATA_PENDING flag will cause the driver to return a
        USB_ERROR_IRP_SIZE_INVALID.
      * If the size id greater than endpoint size, and the flag is
        specified as USB_DEVICE_IRP_FLAG_DATA_PENDING, the driver will send
        multiple of endpoint size data.
  Conditions:
    Client handle should be valid.
  Input:
    handle -                Client's driver handle (returned from
                            DRV_USB_Open)
    endpointAndDirection -  Specifies the endpoint and direction.
    irp -                   Pointer to the USB_DEVICE_IRP structure.
  Return:
    USB_ERROR_NONE - if the IRP was submitted successful.
    USB_ERROR_IRP_SIZE_INVALID - if the size parameter of the IRP is not
    correct. USB_ERROR_IRP_QUEUE_FULL - if the driver IRP queue is full.
  Example:
    <code>
    // The following code snippet shows an example of how to schedule a
    // IRP to send data from device to host. Assume that max packet size
    // is 64 and endpoint is 1. In this example, the transfer is
    // processed as 3 transactions of 64, 64 and 2 bytes.
    
    USB_ENDPOINT ep;
    USB_DEVICE_IRP irp;
    
    ep.direction = USB_ENDPOINT_AND_DIRECTION(USB_DATA_DIRECTION_DEVICE_TO_HOST, 1);
    
    irp.data = myDataBufferToSend;
    irp.size = 130;
    irp.flags = USB_DEVICE_IRP_FLAG_DATA_COMPLETE;
    irp.callback = MyIRPCompletionCallback;
    irp.referenceData = (uintptr_t)&myDeviceLayerObj;
    
    if (DRV_USB_DEVICE_IRPSubmit(handle, ep, irp)
            != USB_ERROR_NONE)
    {
        // This means there was an error.
    }
    else
    {
        // The status of the IRP can be checked.
        while(irp.status != USB_DEVICE_IRP_STATUS_COMPLETED)
        {
            // Wait or run a task routine.
        }
    }
    
    // The following code snippet shows how the client can request
    // the driver to send a ZLP when the size is an exact multiple of
    // end point size.
    
    irp.data = myDataBufferToSend;
    irp.size = 128;
    irp.flags = USB_DEVICE_IRP_FLAG_DATA_COMPLETE;
    irp.callback = MyIRPCompletionCallback;
    irp.referenceData = (uintptr_t)&myDeviceLayerObj;
    
    // Note that while  receiving data from the host, the size should
    // be an exact multiple maximum packet size of the endpoint. In the
    // example below, the DRV_USB_DEVICE_IRPSubmit() function will
    // return a USB_DEVICE_IRP_SUBMIT_RESULT_INVALID_SIZE value.
    
    ep = USB_ENDPOINT_AND_DIRECTION(USB_DATA_DIRECTION_HOST_TO_DEVICE, 1);
    
    irp.data = myDataBufferToSend;
    irp.size = 60; // THIS SIZE IS NOT CORRECT
    irp.flags = USB_DEVICE_IRP_FLAG_DATA_COMPLETE;
    irp.callback = MyIRPCompletionCallback;
    irp.referenceData = (uintptr_t)&myDeviceLayerObj;
    
    </code>
  Remarks:
    None.                                                                           
  ***********************************************************************************/
USB_ERROR DRV_USB_DEVICE_IRPSubmit(DRV_HANDLE client, 
                                         USB_ENDPOINT endpointAndDirection, 
                                         USB_DEVICE_IRP * irp);

/**************************************************************************
  Function:
       void DRV_USB_DEVICE_IRPCancelAll (DRV_HANDLE client,
                                       USB_ENDPOINT endpointAndDirection);
    
  Summary:
    This function cancels all IRPs that are queued and in progress at the
    specified endpoint.
  Description:
    This function cancels all IRPs that are queued and in progress at the
    specified endpoint.
  Conditions:
    Client handle should be valid.
  Input:
    handle -                Client's driver handle (returned from
                            DRV_USB_Open)
    endpointAndDirection -  Specifies the endpoint and direction.
  Return:
    USB_ERROR_NONE - The endpoint was successfully enabled.
    USB_ERROR_DEVICE_ENDPOINT_INVALID - If the endpoint that is being
    accessed is out of the valid endpoint defined for this driver instance.
  Example:
    <code>
    // This code snippet shows an example of how to cancel all IRPs.
    
    void MyIRPCallback(USB_DEVICE_IRP * irp)
    {
        // Check if this is setup command
    
        if(irp->status == USB_DEVICE_IRP_STATUS_SETUP)
        {
            if(IsSetupCommandSupported(irp->data) == false)
            {
                // This means that this setup command is not
                // supported. Stall the endpoint.
    
                DRV_USB_DEVICE_IRPCancelAll(handle, ep);
                DRV_USB_DEVICE_EndpointStall(handle, ep);
            }
         }
     }
    
    
    </code>
  Remarks:
    None.                                                                  
  **************************************************************************/

USB_ERROR DRV_USB_DEVICE_IRPCancelAll (DRV_HANDLE client, 
                                    USB_ENDPOINT endpointAndDirection);
void DRV_USB_HOST_IRPCancel(USB_HOST_IRP * inputIRP);
void DRV_USB_HOST_PipeClose
(
    DRV_USB_HOST_PIPE_HANDLE pipeHandle
);

USB_ERROR DRV_USB_HOST_IRPSubmit(DRV_USB_HOST_PIPE_HANDLE  hPipe,
                                 USB_HOST_IRP * pinputIRP);
void DRV_USB_HOST_IRPCancel(USB_HOST_IRP * pinputIRP);
DRV_USB_HOST_PIPE_HANDLE DRV_USB_HOST_PipeSetup 
(
    DRV_HANDLE client,
    uint8_t deviceAddress, 
    USB_ENDPOINT endpointAndDirection,
    uint8_t hubAddress,
    uint8_t hubPort,
    USB_TRANSFER_TYPE pipeType, 
    uint8_t bInterval, 
    uint16_t wMaxPacketSize,
    USB_SPEED speed
);

void DRV_USB_HOST_BusResetControl(DRV_HANDLE handle, bool control);
bool DRV_USB_HOST_Resume(DRV_HANDLE handle);
bool DRV_USB_HOST_Suspend(DRV_HANDLE handle);
USB_SPEED DRV_USB_HOST_DeviceCurrentSpeedGet(DRV_HANDLE handle);
void DRV_USB_HOST_OperationEnable(DRV_HANDLE handle, bool enable);
bool DRV_USB_HOST_OperationIsEnabled(DRV_HANDLE handle);

// *****************************************************************************
// *****************************************************************************
// Section: Included Files (continued)
// *****************************************************************************
// *****************************************************************************
/*  The file included below maps the interface definitions above to appropriate
    static implementations, depending on build mode.
*/

#include "driver/usb/drv_usb_mapping.h"

#endif
