/*******************************************************************************
  NVM Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_nvm.h

  Summary:
    NVM Driver Interface Definition

  Description:
    The NVM device driver provides a simple interface to manage the NVM modules
    on Microchip microcontrollers. This file defines the interface definition for 
    the NVM driver.
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
#ifndef _DRV_NVM_H
#define _DRV_NVM_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
/* Note:  A file that maps the interface definitions above to appropriate static
          implementations (depending on build mode) is included at the bottom of
          this file.
*/
#include "system/common/sys_common.h"

#include "driver/driver_common.h"

#include "system/common/sys_module.h"

#include "system/int/sys_int.h"

#include "osal/osal.h"

#include "peripheral/nvm/plib_nvm.h"

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Driver NVM Module Index reference

  Summary:
    NVM driver index definitions

  Description:
    These constants provide NVM driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_NVM_Initialize and  DRV_NVM_Open
    routines to identify the driver instance in use.
*/

#define      DRV_NVM_INDEX_0      0
#define      DRV_NVM_INDEX_1      1


// *****************************************************************************
/* NVM Driver Module Index Count

  Summary:
    Number of valid NVM driver indices

  Description:
    This constant identifies NVM driver index definitions.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.
    This value is part-specific.
*/

#define DRV_NVM_INDEX_COUNT     1

// *****************************************************************************
/* NVM Driver Buffer Handle 

  Summary:
    This type defines the NVM Driver Buffer handle.

  Description:
    This type defines the NVM Driver Buffer handle.  

  Remarks:
    None.
*/

typedef uintptr_t DRV_NVM_BUFFER_HANDLE;

// *****************************************************************************
/* NVM Driver Buffer Invalid Handle 

  Summary:
    This value defines the NVM Driver Buffer Invalid handle.

  Description:
    This value defines the NVM Driver Buffer Invalid handle. This value is
    returned by read/write/erase routines when the desired operation could
    not be completed.

  Remarks:
    None.
*/

#define DRV_NVM_BUFFER_HANDLE_INVALID /* DOM-IGNORE-BEGIN */ ((DRV_NVM_BUFFER_HANDLE)-1) /*DOM-IGNORE-END*/

// *****************************************************************************
/* NVM Client Status

  Summary
    Defines the client status

  Description
    Defines the various client status codes.

  Remarks:
    None
*/

typedef enum
{
    /* Up and running, ready to start new operations */
    DRV_NVM_CLIENT_STATUS_READY
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_READY + 0 /*DOM-IGNORE-END*/,

    /* Operation in progress, unable to start a new one */
    DRV_NVM_CLIENT_STATUS_BUSY
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_BUSY      /*DOM-IGNORE-END*/,

    /*Write operation terminated Occurred*/
    DRV_NVM_WRITE_TERMINATED
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_ERROR - 0 /*DOM-IGNORE-END*/,

    /*Erase operation terminated Occurred*/
    DRV_NVM_ERASE_TERMINATED
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_ERROR - 1 /*DOM-IGNORE-END*/,

    /* Low voltage Error */
    DRV_NVM_LOW_VOLTAGE_ERROR
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_ERROR - 2 /*DOM-IGNORE-END*/,

    /* Low voltage Error */
    DRV_NVM_LOW_VOLTAGE_DETECT_ERROR
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_ERROR - 3 /*DOM-IGNORE-END*/,

    /* Client Invalid */
    DRV_NVM_STATUS_INVALID
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_ERROR - 4 /*DOM-IGNORE-END*/

} DRV_NVM_CLIENT_STATUS;


// *****************************************************************************
/* NVM Driver Initialization Data

  Summary:
    Defines the data required to initialize or reinitialize the NVM driver

  Description:
    This data type defines the data required to initialize or reinitialize the
    NVM driver.

  Remarks:
    Not all the init features are available for all the microcontrollers. Please
	refer to the specific device data sheet to determine availbility.
*/

typedef struct
{
    /* System module initialization */
    SYS_MODULE_INIT     moduleInit;

    /* Identifies NVM hardware module (PLIB-level) ID */
    NVM_MODULE_ID       nvmID;

    /* Interrupt Source for Write Interrupt */
    INT_SOURCE          interruptSource;

    
} DRV_NVM_INIT;


/***********************************************************************
  Summary:
    Specifies the status of the buffer for the read, write and erase
    operations.
	
  Description:
    NVM Driver Buffer Status
    
    This type specifies the status of the buffer for the read, write and
    erase operations.
	
  Remarks:
    None.                                                               
  ***********************************************************************/
  
typedef enum
{
    /*Done OK and ready */
    DRV_NVM_BUFFER_COMPLETED          = 0 ,

    /*Scheduled but not started */
    DRV_NVM_BUFFER_QUEUED             = 1,

    /*Currently being in transfer */
    DRV_NVM_BUFFER_IN_PROGRESS        = 2,

    /*Unknown buffer */
    DRV_NVM_BUFFER_ERROR_UNKNOWN      = -1,

} DRV_NVM_BUFFER_STATUS;

// *****************************************************************************
// *****************************************************************************
// Section: NVM Driver Module Interface Routines
// *****************************************************************************
// *****************************************************************************

/*************************************************************************************
  Function:
       SYS_MODULE_OBJ DRV_NVM_Initialize( const SYS_MODULE_INDEX index,
                                              const SYS_MODULE_INIT * const init )
    
  Summary:
    Initializes the NVM instance for the specified driver index
  Description:
    This routine initializes the NVM driver instance for the specified
    driver index, making it ready for clients to open and use it.
  Conditions:
    None.
  Input:
    index -  Identifier for the instance to be initialized also the type of
             memory used
    init -   Pointer to a data structure containing any data necessary to
             initialize the driver.
  Return:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID.
  Example:
    <code>
    // This code snippet shows an example
    // of initializing the NVM Driver.
    
    DRV_NVM_INIT    NVMInitData;
    SYS_MODULE_OBJ  objectHandle;
    
    NVMInitData.moduleInit.value      = SYS_MODULE_POWER_RUN_FULL;
    NVMInitData.moduleId              = NVM_ID_0;
    NVMInitData.flashInterruptSource  = INT_SOURCE_FLASH_CONTROL;
    
    //usage of DRV_NVM_INDEX_0 indicates usage of Flash-related APIs
    objectHandle = DRV_NVM_Initialize(DRV_NVM_INDEX_0, (SYS_MODULE_INIT*)NVMInitData);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>
  Remarks:
    This routine must be called before any other NVM routine is called.
    
    This routine should only be called once during system initialization
    unless DRV_NVM_Deinitialize is called to deinitialize the driver
    instance.
    
    This routine will NEVER block for hardware access. If the operation
    requires time to allow the hardware to reinitialize, it will be
    reported by the DRV_NVM_Status operation. The system must use
    DRV_NVM_Status to find out when the driver is in the ready state.
    
    Build configuration options may be used to statically override options
    in the "init" structure and will take precedence over initialization
    data passed using this routine.                                                   
  *************************************************************************************/

SYS_MODULE_OBJ DRV_NVM_Initialize( const SYS_MODULE_INDEX index,
                                     const SYS_MODULE_INIT * const init);



/*************************************************************************
  Function:
       void DRV_NVM_Deinitialize( SYS_MODULE_OBJ object )
    
  Summary:
    Deinitializes the specified instance of the NVM driver module
  Description:
    Deinitializes the specified instance of the NVM driver module,
    disabling its operation (and any hardware). Invalidates all the
    internal data.
  Conditions:
    Function DRV_NVM_Initialize should have been called before calling
    this function.
  Input:
    object -  Driver object handle, returned from the DRV_NVM_Initialize
              routine
  Return:
    None.
  Example:
    <code>
    // This code snippet shows an example
    // of deinitializing the driver.
    
    SYS_MODULE_OBJ      object;     //  Returned from DRV_NVM_Initialize
    SYS_STATUS          status;
    
    
    DRV_NVM_Deinitialize(object);
    
    status = DRV_NVM_Status(object);
    if (SYS_MODULE_DEINITIALIZED != status)
    {
        // Check again later if you need to know
        // when the driver is deinitialized.
    }
    </code>
  Remarks:
    Once the Initialize operation has been called, the Deinitialize
    operation must be called before the Initialize operation can be called
    again.
    
    This routine will NEVER block waiting for hardware. If the operation
    requires time to allow the hardware to complete, this will be reported
    by the DRV_NVM_Status operation. The system has to use DRV_NVM_Status
    to find out when the module is in the ready state.                    
  *************************************************************************/

void DRV_NVM_Deinitialize( SYS_MODULE_OBJ object);


/*************************************************************************
  Function:
       SYS_STATUS DRV_NVM_Status( SYS_MODULE_OBJ object )
    
  Summary:
    Gets the current status of the NVM driver module.
  Description:
    This routine provides the current status of the NVM driver module.
  Conditions:
    Function DRV_NVM_Initialize should have been called before calling
    this function.
  Input:
    object -  Driver object handle, returned from the DRV_NVM_Initialize
              routine
  Return:
    SYS_STATUS_READY - Indicates that the driver is ready and accept
    requests for new operations.
    
    Note Any value greater than SYS_STATUS_READY is also a normal running
    state in which the driver is ready to accept new operations.
    
    SYS_STATUS_BUSY - Indicates that the driver is busy with a previous
    system level operation and cannot start another
    
    SYS_STATUS_ERROR - Indicates that the driver is in an error state
  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_NVM_Initialize
    SYS_STATUS          NVMStatus;
    
    NVMStatus = DRV_NVM _Status(object);
    else if (SYS_STATUS_ERROR >= NVMStatus)
    {
        // Handle error
    }
    </code>
  Remarks:
    Any value less than SYS_STATUS_ERROR is also an error state.
    
    SYS_MODULE_DEINITIALIZED - Indicates that the driver has been
    deinitialized
    
    This value is less than SYS_STATUS_ERROR
    
    The this operation can be used to determine when any of the driver's
    module level operations has completed.
    
    If the status operation returns SYS_STATUS_BUSY, the a previous
    operation has not yet completed. Once the status operation returns
    SYS_STATUS_READY, any previous operations have completed.
    
    The value of SYS_STATUS_ERROR is negative (-1). Any value less than
    that is also an error state.
    
    This routine will NEVER block waiting for hardware.
    
    If the Status operation returns an error value, the error may be
    cleared by calling the reinitialize operation. If that fails, the
    deinitialize operation will need to be called, followed by the
    initialize operation to return to normal operations.                  
  *************************************************************************/

SYS_STATUS DRV_NVM_Status( SYS_MODULE_OBJ object);


/***************************************************************************
  Function:
       void DRV_NVM_Tasks ( SYS_MODULE_OBJ object );
    
  Summary:
    Maintains the driver's erase and write state machine and implements its
    ISR.
  Description:
    This routine is used to maintain the driver's internal write and erase
    state machine and implement its ISR for interrupt-driven
    implementations.
  Conditions:
    The DRV_NVM_Initialize routine must have been called for the specified
    NVM driver instance.
  Input:
    object -  Object handle for the specified driver instance (returned from
              DRV_NVM_Initialize)
  Return:
    None.
  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_NVM_Initialize
    
    while (true)
    {
        DRV_NVM_Tasks (object);
    
        // Do other tasks
    }
    </code>
  Remarks:
    This routine is normally not called directly by an application. It is
    called by the system's Tasks routine (SYS_Tasks) or by the appropriate
    raw ISR.
    
    This routine may execute in an ISR context and will never block or
    access any resources that may cause it to block.                        
  ***************************************************************************/

void DRV_NVM_Tasks ( SYS_MODULE_OBJ object );

// *****************************************************************************
// *****************************************************************************
// Section: NVM Driver Client Routines
// *****************************************************************************
// *****************************************************************************

/**************************************************************************
  Function:
       DRV_HANDLE DRV_NVM_Open( const SYS_MODULE_INDEX index,
                                  const DRV_IO_INTENT    ioIntent )
    
  Summary:
    Opens the specified timer driver instance and returns a handle to it
  Description:
    This routine opens the specified NVM driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver.
  Conditions:
    Function DRV_NVM_Initialize must have been called before calling this
    function.
  Input:
    drvIndex -  Identifier for the object instance to be opened
    intent -    Zero or more of the values from the enumeration
                DRV_IO_INTENT "ORed" together to indicate the intended use
                of the driver
  Return:
    If successful, the routine returns a valid open-instance handle (a
    number identifying both the caller and the module instance).
    
    If an error occurs, the return value is DRV_HANDLE_INVALID.
  Example:
    <code>
    DRV_HANDLE handle;
    
    handle = DRV_NVM_Open(DRV_NVM_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        // Unable to open the driver
    }
    </code>
  Remarks:
    The handle returned is valid until the DRV_NVM_Close routine is called.
    
    This routine will NEVER block waiting for hardware.
    
    If the DRV_IO_INTENT_BLOCKING is requested and the driver was built
    appropriately to support blocking behavior, then other client-level
    operations may block waiting on hardware until they are complete.
    
    If DRV_IO_INTENT_NON_BLOCKING is requested the driver client can call
    the DRV_NVM_ClientStatus operation to find out when the module is in
    the ready state.
    
    If the requested intent flags are not supported, the routine will
    return DRV_HANDLE_INVALID.                                            
  **************************************************************************/

DRV_HANDLE DRV_NVM_Open( const SYS_MODULE_INDEX index, const DRV_IO_INTENT ioIntent);

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_NVM_Close( DRV_Handle handle )

  Summary:
    Closes an opened-instance of the NVM driver

  Description:
    This routine closes an opened-instance of the NVM driver, invalidating the
    handle.

  Precondition:
    The DRV_NVM_Initialize routine must have been called for the specified
    NVM driver instance.

    DRV_NVM_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_NVM_Open

    DRV_NVM_Close(handle);
    </code>

  Remarks:
    After calling this routine, the handle passed in "handle" must not be used
    with any of the remaining driver routines.  A new handle must be obtained by
    calling DRV_NVM_Open before the caller may use the driver again.

    If DRV_IO_INTENT_BLOCKING was requested and the driver was built
    appropriately to support blocking behavior call may block until the
    operation is complete.

    If DRV_IO_INTENT_NON_BLOCKING request the driver client can call the
    DRV_NVM_Status operation to find out when the module is in
    the ready state (the handle is no longer valid).

    Note:
    Usually there is no need for the driver client to verify that the Close
    operation has completed.
*/

void DRV_NVM_Close( const DRV_HANDLE handle);

/****************************************************************************
  Function:
       DRV_NVM_CLIENT_STATUS DRV_NVM_ClientStatus(DRV_HANDLE handle);
    
  Summary:
    Gets current client-specific status the NVM driver
  Description:
    This routine gets the client-specific status of the NVM driver
    associated with the given handle.
  Conditions:
    The DRV_NVM_Initialize routine must have been called.
    
    DRV_NVM_Open must have been called to obtain a valid opened device
    handle.
  Input:
    handle -  A valid open-instance handle, returned from the driver's open
              routine
  Return:
    A DRV_NVM_CLIENT_STATUS value describing the current status of the
    driver
  Example:
    <code>
    DRV_HANDLE                  handle;         // Returned from DRV_NVM_Open
    DRV_NVM_CLIENT_STATUS     clientStatus;
    
    clientStatus = DRV_NVM_ClientStatus(handle);
    if(DRV_NVM_CLIENT_STATUS_ERROR >= clientStatus)
    {
        // Handle the error
    }
    </code>
  Remarks:
    This routine will not block for hardware access and will immediately
    return the current status.                                              
  ****************************************************************************/

DRV_NVM_CLIENT_STATUS   DRV_NVM_ClientStatus( const DRV_HANDLE handle );

// *****************************************************************************
/* Function:
    DRV_NVM_BUFFER_STATUS   DRV_NVM_BufferStatus(DRV_HANDLE handle, 
                                        const DRV_HANDLE bufferHandle);

  Summary:
    Gets the current status of the buffer.

  Description:
    This routine gets the current status of the buffer. The application must
    use this routine in case a polling based implementation is desired.

  Precondition:
    The DRV_NVM_Initialize routine must have been called.

    DRV_NVM_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    A DRV_NVM_BUFFER_STATUS value describing the current status of the buffer

  Example:
    <code>
    DRV_HANDLE                  handle;         // Returned from DRV_NVM_Open
    DRV_HANDLE                  bufferHandle;
    DRV_NVM_BUFFER_STATUS       status;
 
    status = DRV_NVM_BufferStatus(handle, bufferHandle);
    if(status == DRV_NVM_BUFFER_COMPLETED)
    {
        // Operation Done
    }
    </code>

  Remarks:
    This routine will not block for hardware access and will immediately return
    the current status.
*/

DRV_NVM_BUFFER_STATUS  DRV_NVM_BufferStatus(DRV_HANDLE handle, const DRV_HANDLE bufferHandle);

// *****************************************************************************
/* Function:
    DRV_NVM_BUFFER_HANDLE DRV_NVM_Read( const DRV_HANDLE handle,  uint8_t *targetbuffer,
	                  uint8_t *srcAddress,  const unsigned int numbytes )

  Summary:
    Reads a block of data from the specified address in memory.

  Description:
    This routine reads a block of data from the specified address in memory. 
    It returns a buffer handle that can be queried by the DRV_NVM_BufferStatus()
    function to check for completion of the operation. 

  Precondition:
    The DRV_NVM_Initialize routine must have been called for the specified
    NVM driver instance.

    DRV_NVM_Open must have been called to obtain a valid opened device handle.

    DRV_IO_INTENT_READ or DRV_IO_INTENT_READWRITE must have been specified in
    the DRV_NVM_Open call.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

    targetBuffer - Buffer into which the data read from the NVM instance
                   will be placed.

	srcAddress	 -	The base address in NVM memory from data read should begin
					
    numbytes     - Total number of bytes that need to be read from the module
                   instance (must be equal to or less than the size of the
                   buffer)

  Returns:
    DRV_NVM_BUFFER_HANDLE_INVALID in case the operation was not successful. A
    valid handle otherwise.

  Example:
    <code>
    DRV_HANDLE      handle;    // Returned from DRV_NVM_Open
    char            myBuffer[MY_BUFFER_SIZE];
	char			*srcAddress = NVM_BASE_ADDRESS_TO_READ_FROM;
    unsigned int    count = MY_BUFFER_SIZE;
    DRV_NVM_BUFFER_HANDLE bufferHandle;

	bufferHandle  = DRV_NVM_Read(myNVMHandle, &myBuffer[total], srcAddress, count);
    if(DRV_NVM_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Do error handling here
    }
    
    // Wait until the buffer completes. This should not
    // be a while loop if a part of cooperative multi-tasking 
    // routine. In that case, it should be invoked in task
    // state machine.
    while(DRV_NVM_BufferStatus(bufferHandle) != DRV_NVM_BUFFER_COMPLETED);

    </code>

  Remarks:
    If the DRV_IO_INTENT_BLOCKING flag was given in the call to the
    DRV_NVM_Open routine and the driver was built to support blocking behavior
    the call will block until the operation is complete.

    If the DRV_IO_INTENT_NONBLOCKING flag was given in the call to the
    DRV_NVM_Open routine or the driver was built to only support non-blocking
    behavior the call will return immediately, identifying how many bytes of
    data were actually copied into the client's buffer and the client must call
    DRV_NVM_Read again with adjusted parameters as shown in the example.
*/

DRV_NVM_BUFFER_HANDLE DRV_NVM_Read( const DRV_HANDLE handle,
        uint8_t *targetBuffer, uint8_t *srcAddress,
        const unsigned int numbytes);

// *****************************************************************************
/* Function:
    DRV_NVM_BUFFER_HANDLE DRV_NVM_Write( const DRV_HANDLE handle,  uint8_t *targetbuffer,
	                  uint8_t *srcAddress,  const unsigned int numbytes )

  Summary:
    Write a block of data to a specified address in memory.

  Description:
    This routine writes a block of data to a specified address in memory. 
    It returns a buffer handle that can be queried by the DRV_NVM_BufferStatus()
    function to check for completion of the operation. The contents of the 
    source buffer should not be changed while the operation is in progress.
    The target address should be aligned on a DRV_NVM_ROW_SIZE byte boundary.
    The number of bytes to write should be equal to or should be multiples
    of DRV_NVM_ROW_SIZE.

  Precondition:
    The DRV_NVM_Initialize routine must have been called for the specified
    NVM driver instance.

    DRV_NVM_Open must have been called to obtain a valid opened device handle.

    DRV_IO_INTENT_READ or DRV_IO_INTENT_READWRITE must have been specified in
    the DRV_NVM_Open call.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

    targetBuffer - Buffer in NVM memory where the data from srcAddress 
                   will be placed. Should be aligned on a DRV_NVM_ROW_SIZE
                   byte boundary.

	srcAddress	 -	The source buffer containing data to programmed into NVM

    numbytes     - Total number of bytes that need to be written to the NVM.
                   This should be equal to or a multiple of DRV_NVM_ROW_SIZE.

  Returns:
    DRV_NVM_BUFFER_HANDLE_INVALID in case the operation was not successful. A
    valid handle otherwise.

  Example:
    <code>
    DRV_HANDLE      handle;    // Returned from DRV_NVM_Open
    char            myBuffer[2 * DRV_NVM_ROW_SIZE];
    
    // Destination address should be row aligned.
	char			*destAddress = (char *)NVM_BASE_ADDRESS_TO_WRITE;
    
    unsigned int    count = 2 * MY_BUFFER_SIZE;
    DRV_NVM_BUFFER_HANDLE bufferHandle;

	bufferHandle  = DRV_NVM_Write(myNVMHandle, destAddress, &myBuffer[total], count);
    if(DRV_NVM_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Do error handling here
    }
    
    // Wait until the buffer completes. This should not
    // be a while loop if a part of cooperative multi-tasking 
    // routine. In that case, it should be invoked in task
    // state machine.
    while(DRV_NVM_BufferStatus(bufferHandle) != DRV_NVM_BUFFER_COMPLETED);

    </code>

  Remarks:
    If the DRV_IO_INTENT_BLOCKING flag was given in the call to the
    DRV_NVM_Open routine and the driver was built to support blocking behavior
    the call will block until the operation is complete.

    If the DRV_IO_INTENT_NONBLOCKING flag was given in the call to the
    DRV_NVM_Open routine or the driver was built to only support non-blocking
    behavior the call will return immediately.
*/


DRV_NVM_BUFFER_HANDLE DRV_NVM_Write( DRV_HANDLE handle, uint8_t * destination,
                                        uint8_t * source, const unsigned int nBytes);

/**************************************************************************
  Function:
       DRV_NVM_BUFFER_HANDLE DRV_NVM_Erase( const DRV_HANDLE handle,
                   uint8_t *targetbuffer, const unsigned int numbytes )
    
  Summary:
    Erase the specified number of pages in flash memory.
  Description:
    This routine erases the specified number of pages in Flash memory. It
    returns a buffer handle that can be queried by the
    DRV_NVM_BufferStatus() function to check for completion of the
    operation. The target address should be aligned on a DRV_NVM_PAGE_SIZE
    byte boundary. The number of bytes to write should be equal to or
    should be multiples of DRV_NVM_PAGE_SIZE.
  Conditions:
    The DRV_NVM_Initialize routine must have been called for the specified
    NVM driver instance.
    
    DRV_NVM_Open must have been called to obtain a valid opened device
    handle.
    
    DRV_IO_INTENT_READ or DRV_IO_INTENT_READWRITE must have been specified
    in the DRV_NVM_Open call.
  Input:
    handle -        A valid open-instance handle, returned from the
                    driver's open routine
    targetBuffer -  Start address in NVM memory from where the erase should
                    begin. Should be aligned on a DRV_NVM_PAGE_SIZE byte
                    boundary.
    numbytes -      Total number of pages to be erased expressed in bytes.
                    This should be equal to or a multiple of
                    DRV_NVM_PAGE_SIZE.
  Return:
    DRV_NVM_BUFFER_HANDLE_INVALID in case the operation was not successful.
    A valid handle otherwise.
  Example:
    <code>
    
    // Returned from DRV_NVM_Open
    DRV_HANDLE      handle;
    
    // Destination address should be row aligned.
    char            *destAddress = (char *)NVM_BASE_ADDRESS_TO_ERASE;
    
    unsigned int    count = 2 * DRV_NVM_PAGE_SIZE;
    DRV_NVM_BUFFER_HANDLE bufferHandle;
    
    bufferHandle  = DRV_NVM_Write(myNVMHandle, destAddress, count);
    if(DRV_NVM_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Do error handling here
    }
    
    // Wait until the buffer completes. This should not
    // be a while loop if a part of cooperative multi-tasking
    // routine. In that case, it should be invoked in task
    // state machine.
    while(DRV_NVM_BufferStatus(bufferHandle) != DRV_NVM_BUFFER_COMPLETED);
    
    </code>
  Remarks:
    If the DRV_IO_INTENT_BLOCKING flag was given in the call to the
    DRV_NVM_Open routine and the driver was built to support blocking
    behavior the call will block until the operation is complete.
    
    If the DRV_IO_INTENT_NONBLOCKING flag was given in the call to the
    DRV_NVM_Open routine or the driver was built to only support
    non-blocking behavior the call will return immediately.                
  **************************************************************************/

DRV_NVM_BUFFER_HANDLE DRV_NVM_Erase( const DRV_HANDLE handle,  
                uint8_t *targetbuffer, const unsigned int numbytes );

// ****************************************************************************
// ****************************************************************************
// Section: Included Files (continued)
// ****************************************************************************
// ****************************************************************************
/*  The files included below map the interface definitions above to appropriate
    static implementations, depending on build mode.
*/

#include "driver/nvm/drv_nvm_mapping.h"


#endif // #ifndef _DRV_NVM_H
/*******************************************************************************
 End of File
*/

