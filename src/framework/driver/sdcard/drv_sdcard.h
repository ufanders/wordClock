/*******************************************************************************
  SD Card Device Driver Interface

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdcard.h

  Summary:
    SD Card Device Driver Interface File

  Description:
    The SD Card device driver provides a simple interface to manage the "SD Card"
    peripheral.  This file defines the interface definitions and prototypes for
    the SD Card driver.
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

#ifndef _DRV_SDCARD_H
#define _DRV_SDCARD_H


// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>

#include "peripheral/spi/plib_spi.h"
#include "system/common/sys_module.h"
#include "driver/driver_common.h"
#include "system/clk/sys_clk.h"
#include "driver/spi/drv_spi.h"
#include "peripheral/int/plib_int.h"
#include "peripheral/ports/plib_ports.h"
#include "system/fs/sys_fs_media_manager.h"

// *****************************************************************************
/* SD Card Buffer Handle

  Summary:
    Handle to a buffer added to the process queue.

  Description:
    This handle identifies the a buffer in the queue(read or write).

  Remarks:
*/

typedef uintptr_t DRV_SDCARD_BUFFER_HANDLE;


// *****************************************************************************
/* SD Card Driver Module Index Numbers

  Summary:
    SD Card driver index definitions

  Description:
    These constants provide SD Card driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_SDCARD_Initialize and
    DRV_SDCARD_Open routines to identify the driver instance in use.
*/

#define DRV_SDCARD_INDEX_0         0
#define DRV_SDCARD_INDEX_1         1
#define DRV_SDCARD_INDEX_2         2
#define DRV_SDCARD_INDEX_3         3


// *****************************************************************************
/* SD Card Driver Module Index Count

  Summary:
    Number of valid SD Card driver indices

  Description:
    This constant identifies number of valid SD Card driver indices.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is derived from part-specific header files defined as part of the
    peripheral libraries.
*/

#define DRV_SDCARD_INDEX_COUNT                     DRV_SDCARD_INDEX_MAX


// *****************************************************************************
/* SD Card Driver Maximum allowed limit

  Summary:
 Maximum allowed SD card instances

  Description:
    This constant identifies number of valid SD Card driver indices.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is derived from part-specific header files defined as part of the
    peripheral libraries.
*/

#define SDCARD_MAX_LIMIT                                                2


// *****************************************************************************
/* SD Card Driver Client Status

  Summary:
    Identifies the client-specific status of the SD card driver

  Description:
    This enumeration identifies the client-specific status of the SD card driver.

  Remarks:
    None.
*/

typedef enum
{
    /* Client in an invalid state */
    DRV_SDCARD_CLIENT_STATUS_INVALID
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_ERROR_EXTENDED - 0 /*DOM-IGNORE-END*/,

    /* Unspecified error condition */
    DRV_SDCARD_CLIENT_STATUS_ERROR
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_ERROR - 0 /*DOM-IGNORE-END*/,

   /* Client is not open */
    DRV_SDCARD_CLIENT_STATUS_CLOSED
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_CLOSED + 0 /*DOM-IGNORE-END*/,

    /* An operation is currently in progress */
    DRV_SDCARD_CLIENT_STATUS_BUSY
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_BUSY      /*DOM-IGNORE-END*/,

    /* Up and running, no operations running */
    DRV_SDCARD_CLIENT_STATUS_READY
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_READY + 0 /*DOM-IGNORE-END*/,

    /* SD Card stopped (not running), but ready to accept commands */
    DRV_SDCARD_CLIENT_STATUS_STOPPED
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_READY_EXTENDED + 0 /*DOM-IGNORE-END*/,

} DRV_SDCARD_CLIENT_STATUS;


// *****************************************************************************
/* SD Card Buffer Status

   Summary
    Defines the buffer status

   Description
    Defines the states that an SD Card buffer can be in during its lifetime

   Remarks:
    As buffers may have a limited life span, so too have the associated handles
    and status info. Once a buffer transfer is completed (and possibly
    notified and acknowledged) it will be discarded. The status of a discarded
    buffer is no longer maintained by the driver.

*/

typedef enum
{
    /*Done OK and ready */
    DRV_SDCARD_BUFFER_COMPLETED
        /*DOM-IGNORE-BEGIN*/ = 0 , /*DOM-IGNORE-END*/

    /*Scheduled but not started */
    DRV_SDCARD_BUFFER_QUEUED
        /*DOM-IGNORE-BEGIN*/ = 1, /*DOM-IGNORE-END*/

    /*Currently being in transfer */
    DRV_SDCARD_BUFFER_IN_PROGRESS
        /*DOM-IGNORE-BEGIN*/ = 2, /*DOM-IGNORE-END*/

    /*Unknown buffer */
    DRV_SDCARD_BUFFER_ERROR_UNKNOWN
        /*DOM-IGNORE-BEGIN*/ = -1, /*DOM-IGNORE-END*/

    /*Read data lost: rx process not fast enough */
    DRV_SDCARD_BUFFER_ERROR_RX_OVERFLOW
        /*DOM-IGNORE-BEGIN*/ = -2  /*DOM-IGNORE-END*/

} DRV_SDCARD_BUFFER_STATUS;


// *****************************************************************************
/* System events

  Summary:
    Defines the different system events

  Description:
    This enum defines different system events.

  Remarks:
    None.
*/

typedef enum
{
    /* The media event is SD Card attach */
    SYS_MEDIA_EVENT_SDCARD_ATTACHED,

    /* The media event is SD Card detach */
    SYS_MEDIA_EVENT_SDCARD_DETACHED,

}SYS_MEDIA_EVENT;


// *****************************************************************************
/* SD Card Device Driver Initialization Data

  Summary:
    Contains all the data necessary to initialize the SD Card device

  Description:
    This structure contains all the data necessary to initialize the SD Card
    device.

  Remarks:
    A pointer to a structure of this format containing the desired
    initialization data must be passed into the DRV_SDCARD_Initialize routine.
*/

typedef struct _DRV_SDCARD_INIT
{
    /* System module initialization */
    SYS_MODULE_INIT                     moduleInit;

	/* SPI driver index */
	SYS_MODULE_INDEX    				spiIndex;

    /* Identifies peripheral (PLIB-level) ID */
    SPI_MODULE_ID                       spiId;

    /* SD card communication speed */
    uint32_t                            sdcardSpeedHz;

    /* Card detect pin setting */
    PORTS_CHANNEL                       cardDetectPort;

    PORTS_BIT_POS                       cardDetectBitPosition;

    /* Write protect pin setting */
    PORTS_CHANNEL                       writeProtectPort;

    PORTS_BIT_POS                       writeProtectBitPosition;

    /* Chip select pin setting */
    PORTS_CHANNEL                       chipSelectPort;


    /* Position of the bit in the port selected for chip selection */
    PORTS_BIT_POS                       chipSelectBitPosition;

} DRV_SDCARD_INIT;


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - System Level
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_SDCARD_Initialize ( const SYS_MODULE_INDEX        index,
                                          const SYS_MODULE_INIT * const init )

  Summary:
    Initializes the SD Card driver

  Description:
    This routine initializes the SD Card driver, making it ready for clients to
    open and use it.

  Precondition:
    None.

  Parameters:
    drvIndex        - Index for the driver instance to be initialized

    init            - Pointer to a data structure containing any data necessary
                      to initialize the driver. This pointer may be null if no
                      data is required because static overrides have been
                      provided.

  Returns:
    If successful, returns a valid handle to a driver object.  Otherwise, it
    returns SYS_MODULE_OBJ_INVALID. The returned object must be passed as
    argument to DRV_SDCARD_Reinitialize, DRV_SDCARD_Deinitialize,
    DRV_SDCARD_TaskRead, DRV_SDCARD_TaskWrite and DRV_SDCARD_Status routines.

  Example:
    <code>
    DRV_SDCARD_INIT     init;
    SYS_MODULE_OBJ      objectHandle;

    // Populate the SD Card initialization structure
    init.moduleInit.value   = SYS_MODULE_POWER_RUN_FULL;
    init.baudRate           = 100000;
    init.spiId 				= SPI_ID_1;

    // Do something

    objectHandle = DRV_SDCARD_Initialize(DRV_SDCARD_INDEX_0, (SYS_MODULE_INIT*)&init);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
    This routine must be called before any other SD Card routine is called.

    This routine should only be called once during system initialization
    unless DRV_SDCARD_Deinitialize is called to deinitialize the driver instance.

    This routine will NEVER block for hardware access. If the operation requires
    time to allow the hardware to re-initialize, it will be reported by the
    DRV_SDCARD_Status operation. The system must use DRV_SDCARD_Status to find out
    when the driver is in the ready state.

    Build configuration options may be used to statically override options in the
    "init" structure and will take precedence over initialization data passed
    using this routine.
*/

SYS_MODULE_OBJ DRV_SDCARD_Initialize ( const SYS_MODULE_INDEX        index,
                                       const SYS_MODULE_INIT * const init );


// *****************************************************************************
/* Function:
    void DRV_SDCARD_Reinitialize ( SYS_MODULE_OBJ                object,
                                  const SYS_MODULE_INIT * const init )

  Summary:
    Reinitializes the driver and refreshes any associated hardware settings

  Description:
    This routine reinitializes the driver and refreshes any associated hardware
    settings using the initialization data given, but it will not interrupt any
    ongoing operations.

  Precondition:
    Function DRV_SDCARD_Initialize must have been called before calling this
    routine and a valid SYS_MODULE_OBJ must have been returned.

  Parameters:
    object          - Driver object handle, returned from the DRV_SDCARD_Initialize
                      routine

    init            - Pointer to the initialization data structure

  Returns:
    None

  Example:
    <code>
	DRV_SDCARD_INIT     init;
	SYS_MODULE_OBJ      objectHandle;

	// Populate the SD Card initialization structure
	init.moduleInit.value   = SYS_MODULE_POWER_RUN_FULL;
	init.baudRate           = 100000;
	init.spiId 				= SPI_ID_1;

	// Do something

    </code>

  Remarks:
    This function can be called multiple times to reinitialize the module.

    This operation can be used to refresh any supported hardware registers as
    specified by the initialization data or to change the power state of the
    module.

    This routine will NEVER block for hardware access. If the operation requires
    time to allow the hardware to re-initialize, it will be reported by the
    DRV_SDCARD_Status operation. The system must use DRV_SDCARD_Status to find out
    when the driver is in the ready state.

    Build configuration options may be used to statically override options in the
    "init" structure and will take precedence over initialization data passed
    using this routine.
*/

void DRV_SDCARD_Reinitialize ( SYS_MODULE_OBJ                object,
                               const SYS_MODULE_INIT * const init );


// *****************************************************************************
/* Function:
    void DRV_SDCARD_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the SD Card driver module

  Description:
    Deinitializes the specified instance of the SD Card driver module, disabling
    its operation (and any hardware).  Invalidates all the internal data.

  Precondition:
    Function DRV_SDCARD_Initialize must have been called before calling this
    routine and a valid SYS_MODULE_OBJ must have been returned.

  Parameters:
    object          - Driver object handle, returned from the
					  DRV_SDCARD_Initialize routine.

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     //  Returned from DRV_SDCARD_Initialize
    SYS_STATUS          status;

    DRV_SDCARD_Deinitialize(object);

    status = DRV_SDCARD_Status(object);
    if (SYS_MODULE_UNINITIALIZED == status)
    {
        // Check again later if you need to know
        // when the driver is deinitialized.
    }
    </code>

  Remarks:
    Once the Initialize operation has been called, the Deinitialize operation
    must be called before the Initialize operation can be called again.

    This routine will NEVER block waiting for hardware. If the operation
    requires time to allow the hardware to complete, this will be reported by
    the DRV_SDCARD_Status operation.  The system has to use DRV_SDCARD_Status to find
    out when the module is in the ready state.
*/

void DRV_SDCARD_Deinitialize ( SYS_MODULE_OBJ object );


// *****************************************************************************
/* Function:
    SYS_STATUS DRV_SDCARD_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the SD Card driver module

  Description:
    This routine provides the current status of the SD Card driver module.

  Precondition:
    Function DRV_SDCARD_Initialize must have been called before calling this
    function

  Parameters:
    object          - Driver object handle, returned from the
                      DRV_SDCARD_Initialize routine

  Returns:
    SYS_STATUS_READY          - Indicates that the driver is busy with a
                                previous system level operation and cannot start
                                another

                                Note Any value greater than SYS_STATUS_READY is
                                also a normal running state in which the driver
                                is ready to accept new operations.

    SYS_STATUS_BUSY           - Indicates that the driver is busy with a
                                previous system level operation and cannot start
                                another

    SYS_STATUS_ERROR          - Indicates that the driver is in an error state

                                Note:  Any value less than SYS_STATUS_ERROR is
                                also an error state.

    SYS_MODULE_DEINITIALIZED  - Indicates that the driver has been deinitialized

                                Note:  This value is less than SYS_STATUS_ERROR

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_SDCARD_Initialize
    SYS_STATUS          status;

    status = DRV_SDCARD_Status(object);
    else if (SYS_STATUS_ERROR >= status)
    {
        // Handle error
    }
    </code>

  Remarks:
    The this operation can be used to determine when any of the driver's module
    level operations has completed.

    If the status operation returns SYS_STATUS_BUSY, the a previous operation
    has not yet completed.  Once the status operation returns SYS_STATUS_READY,
    any previous operations have completed.

    The value of SYS_STATUS_ERROR is negative (-1).  Any value less than that is
    also an error state.

    This routine will NEVER block waiting for hardware.

    If the Status operation returns an error value, the error may be cleared by
    calling the reinitialize operation.  If that fails, the deinitialize
    operation will need to be called, followed by the initialize operation to
    return to normal operations.
*/

SYS_STATUS DRV_SDCARD_Status ( SYS_MODULE_OBJ object );


// *****************************************************************************
/* Function:
    void DRV_SDCARD_Tasks ( SYS_MODULE_OBJ object )

  Summary:
    Maintains the driver's state machine

  Description:
    This routine is used to maintain the driver's internal state machine.

  Precondition:
    The DRV_SDCARD_Initialize routine must have been called for the specified
    SDCARD driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_SDCARD_Initialize)

  Returns:
    None

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_SDCARD_Initialize

    while (true)
    {
        DRV_SDCARD_Tasks (object);

        // Do other tasks
    }
    </code>

  Remarks:
    This routine is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks) or by the appropriate raw
    ISR.

    This routine may execute in an ISR context and will never block or access any
    resources that may cause it to block.
*/

void DRV_SDCARD_Tasks ( SYS_MODULE_OBJ object );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client Level
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_SDCARD_Open ( const SYS_MODULE_INDEX drvIndex,
                                const DRV_IO_INTENT    intent )

  Summary:
    Opens the specified SD Card driver instance and returns a handle to it

  Description:
    This routine opens the specified SD Card driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver.

  Precondition:
    Function DRV_SDCARD_Initialize must have been called before calling this
    function.

  Parameters:
    drvIndex    - Identifier for the object instance to be opened

    intent      - Zero or more of the values from the enumeration
                  DRV_IO_INTENT "ORed" together to indicate the intended use
                  of the driver

  Returns:
    If successful, the routine returns a valid open-instance handle (a number
    identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID.

  Example:
    <code>
    DRV_HANDLE  handle;

    handle = DRV_SDCARD_Open ( DRV_SDCARD_INDEX_0, DRV_IO_INTENT_EXCLUSIVE );

    if ( DRV_HANDLE_INVALID == handle )
    {
        // Unable to open the driver
    }
    </code>

  Remarks:
    The handle returned is valid until the DRV_SDCARD_Close routine is called.

    This routine will NEVER block waiting for hardware.

    If the DRV_IO_INTENT_BLOCKING is requested and the driver was built
    appropriately to support blocking behavior, then other client-level
    operations may block waiting on hardware until they are complete.

    If DRV_IO_INTENT_NON_BLOCKING is requested the driver client can call the
    DRV_SDCARD_ClientStatus operation to find out when the module is in the ready
    state.

    If the requested intent flags are not supported, the routine will return
    DRV_HANDLE_INVALID.
*/

DRV_HANDLE DRV_SDCARD_Open ( const SYS_MODULE_INDEX drvIndex,
                            const DRV_IO_INTENT    intent );


// *****************************************************************************
/* Function:
    void DRV_SDCARD_Close ( DRV_HANDLE handle )

  Summary:
    Closes an opened-instance of the SD Card driver

  Description:
    This routine closes an opened-instance of the SD Card driver, invalidating
    the handle.

  Precondition:
    The DRV_SDCARD_Initialize routine must have been called for the specified
    SD Card driver instance.

    DRV_SDCARD_Open must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_SDCARD_Open

    DRV_SDCARD_Close ( handle );
    </code>

  Remarks:
    After calling this routine, the handle passed in "handle" must not be used
    with any of the remaining driver routines.  A new handle must be obtained by
    calling DRV_SDCARD_Open before the caller may use the driver again.

    If DRV_IO_INTENT_BLOCKING was requested and the driver was built
    appropriately to support blocking behavior call may block until the
    operation is complete.

    If DRV_IO_INTENT_NON_BLOCKING request the driver client can call the
    DRV_SDCARD_Status operation to find out when the module is in
    the ready state (the handle is no longer valid).

    Note:
    Usually there is no need for the driver client to verify that the Close
    operation has completed.
*/

void DRV_SDCARD_Close( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
    DRV_SDCARD_CLIENT_STATUS DRV_SDCARD_ClientStatus ( DRV_HANDLE handle )

  Summary:
    Gets current client-specific status the SD Card driver

  Description:
    This routine gets the client-specific status of the SD Card driver associated
    with the given handle.

  Precondition:
    The DRV_SDCARD_Initialize routine must have been called.

    DRV_SDCARD_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    A DRV_SDCARD_CLIENT_STATUS value describing the current status of the driver

  Example:
    <code>
    DRV_HANDLE sdcardHandle;  // Returned from DRV_SDCARD_Open
    DRV_SDCARD_CLIENT_STATUS sdcardClientStatus;

    sdcardClientStatus = DRV_SDCARD_ClientStatus ( sdcardHandle );
    if ( DRV_SDCARD_CLIENT_STATUS_ERROR >= sdcardClientStatus )
    {
        // Handle the error
    }
    </code>

  Remarks:
    This routine will not block for hardware access and will immediately return
    the current status.
*/

DRV_SDCARD_CLIENT_STATUS DRV_SDCARD_ClientStatus ( DRV_HANDLE handle );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - SD Card Operations
// *****************************************************************************
// *****************************************************************************
/* Functions for read/write operations and it's status */

// *****************************************************************************
/* Function:
    SYS_FS_MEDIA_BUFFER_HANDLE DRV_SDCARD_SectorRead ( DRV_HANDLE handle,
    	uint8_t *buffer, uint32_t sector_addr, uint32_t sectorCount )

  Summary:
    Reads data from the sectors of the SD card.

  Description:
	This function reads data from the sectors of the SD card starting from the sector
	address and stores it in the location pointed by 'buffer'.

  Precondition:
    The DRV_SDCARD_Initialize routine must have been called.

    DRV_SDCARD_Open must have been called to obtain a valid opened device
	handle.

  Parameters:
	handle - A valid open-instance handle, returned from the driver's
				open routine

	sector_addr - The address of the sector on the card.

	sectorCount - Number of sectors to be read.

	buffer -  The buffer where the retrieved data will be stored.  If
				buffer is NULL, do not store the data anywhere.

  Returns:
    SYS_FS_MEDIA_BUFFER_HANDLE - Buffer handle.

  Example:
    <code>
    SYS_FS_MEDIA_BUFFER_HANDLE bufferHandle;

    bufferHandle = DRV_SDCARD_SectorRead ( handle,
											,readData //buffer
											, 20 //  Sector
											, 1 // Number of Sectors
											);

    </code>

  Remarks:
    This routine will not block for hardware access and will immediately return
    the current status.
*/

SYS_FS_MEDIA_BUFFER_HANDLE DRV_SDCARD_SectorRead ( DRV_HANDLE handle,
	uint8_t *buffer, uint32_t sector_addr, uint32_t sectorCount );


// *****************************************************************************
/* Function:
    SYS_FS_MEDIA_BUFFER_HANDLE DRV_SDCARD_SectorWrite ( DRV_HANDLE handle,
    	uint32_t sector_addr, uint8_t *buffer, uint32_t sectorCount )

  Summary:
    Writes sector(s) of data to an SD card.

  Description:
	This function writes sector(s) of data (512 bytes) of data from the
	location pointed to by 'buffer' to the specified sector of the SD card.

  Precondition:
    The DRV_SDCARD_Initialize routine must have been called.

    DRV_SDCARD_Open must have been called to obtain a valid opened device
	handle.

  Parameters:
	handle - A valid open-instance handle, returned from the driver's
				open routine

	sector_addr - The address of the sector on the card.

	buffer -  The buffer with the data to write.

  Returns:
    SYS_FS_MEDIA_BUFFER_HANDLE - Buffer handle.

  Example:
    <code>
    SYS_FS_MEDIA_BUFFER_HANDLE bufferHandle;

	bufferHandle = DRV_SDCARD_SectorWrite ( handle,
											,readData //buffer
											, 20 // Sector
											, 1 // Number of Sectors
											);
    </code>

  Remarks:
	The card expects the address field in the command packet to be a byte address.
	The sector_addr value is converted to a byte address by shifting it left nine
	times (multiplying by 512).

*/

SYS_FS_MEDIA_BUFFER_HANDLE DRV_SDCARD_SectorWrite ( DRV_HANDLE handle,
	uint32_t sector_addr, uint8_t *buffer, uint32_t sectorCount );


// *****************************************************************************
/* Function:
    SYS_FS_MEDIA_BUFFER_STATUS DRV_SDCARD_BufferStatusGet ( DRV_HANDLE handle,
					SYS_FS_MEDIA_BUFFER_HANDLE bufferHandle )

  Summary:
    Gets the status of an an SD card operation (read/write)

  Description:
	This function gets the status of an an SD card operation (read/write).
	To be called only after a read or write is scheduled.

  Precondition:
    The DRV_SDCARD_Initialize routine must have been called.

    DRV_SDCARD_Open must have been called to obtain a valid opened device
	handle.

  Parameters:
	handle - A valid open-instance handle, returned from the driver's
				open routine

	bufferHandle - Handle returned by a 'sector write' or a 'sector read'
				function.



  Returns:
    SYS_FS_MEDIA_BUFFER_STATUS - Buffer status.

  Example:
    <code>
    int globalState = 0;
    SYS_FS_MEDIA_BUFFER_HANDLE bufferHandle;

	switch (globalState)
	{
		case 0:
			bufferHandle = SYS_FS_MEDIA_MANAGER_SectorWrite ( handle,
												,readData //buffer
												, 20 // Sector
												, 1 // Number of Sectors
												);
			globalState++;
			break;
		case 1:
			if ( SYS_FS_MEDIA_MANAGER_BufferStatusGet(handle, bufferHandle) ==
						SYS_FS_MEDIA_BUFFER_COMPLETED)
			{
				//Write complete
			}
			break;
    </code>

  Remarks:
	None.
*/

SYS_FS_MEDIA_BUFFER_STATUS DRV_SDCARD_BufferStatusGet ( DRV_HANDLE handle,
					SYS_FS_MEDIA_BUFFER_HANDLE bufferHandle );


// *****************************************************************************
/* Function:
	SYS_FS_MEDIA_STATUS DRV_SDCARD_MediaStatusGet ( DRV_HANDLE handle )

  Summary:
    Gets the status of the device.

  Description:
	This function gets the status of the device ( attached/detached ).

  Precondition:
    The DRV_SDCARD_Initialize routine must have been called.

    DRV_SDCARD_Open must have been called to obtain a valid opened device
	handle.

  Parameters:
	handle - A valid open-instance handle, returned from the driver's
				open routine

  Returns:
    SYS_FS_MEDIA_STATUS - Status of the device.

  Example:
    <code>
    DRV_HANDLE sdcardHandle;  // Returned from DRV_SDCARD_Open

    if( DRV_SDCARD_MediaStatusGet ( handle ) == SYS_FS_MEDIA_ATTACHED )
	{
		//Device is attached
	}

    </code>

  Remarks:
	None.

*/

SYS_FS_MEDIA_STATUS DRV_SDCARD_MediaStatusGet ( DRV_HANDLE handle );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Module Information
// *****************************************************************************
// *****************************************************************************
/* These functions return the information about the SD card module */

// *****************************************************************************
/* Function:
    uint32_t DRV_SDCARD_SectorSizeGet ( DRV_HANDLE handle )

  Summary:
    Gets the sector size of the selected SD card device.

  Description:
	This function gets the card's sector size.

  Precondition:
    The DRV_SDCARD_Initialize routine must have been called.

    DRV_SDCARD_Open must have been called to obtain a valid opened device
	handle.

	SD Card must have been detected and initialized by the task routine.

  Parameters:
	handle - A valid open-instance handle, returned from the driver's
				open routine

  Returns:
	The size of a sector in the selected SD Card.

  Example:
    <code>
    DRV_HANDLE sdcardHandle;  // Returned from DRV_SDCARD_Open
    DRV_SDCARD_CLIENT_STATUS sdcardClientStatus;
	bool status;

    </code>

  Remarks:
    This routine will not block for hardware access and will immediately return
    the current status.
*/

uint32_t DRV_SDCARD_SectorSizeGet ( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
    uint32_t DRV_SDCARD_SectorsCountGet ( DRV_HANDLE handle )

  Summary:
    Gets the number of sectors present in the selected SD card device.

  Description:
	This function gets the number of sectors present in the selected SD card
	device.

  Precondition:
    The DRV_SDCARD_Initialize routine must have been called.

    DRV_SDCARD_Open must have been called to obtain a valid opened device
	handle.

	SD Card must have been detected and initialized by the task routine.

  Parameters:
	handle - A valid open-instance handle, returned from the driver's
				open routine

  Returns:
	The number of a sectors in the selected SD Card.

  Example:
    <code>
    DRV_HANDLE sdcardHandle;  // Returned from DRV_SDCARD_Open
    DRV_SDCARD_CLIENT_STATUS sdcardClientStatus;
	bool status;

    </code>

  Remarks:
    This routine will not block for hardware access and will immediately return
    the current status.
*/

uint32_t DRV_SDCARD_SectorsCountGet ( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
    bool DRV_SDCARD_WriteProtectionIsEnabled ( DRV_HANDLE handle )

  Summary:
    Gets the status of write protection.

  Description:
    This function returns 'true' if the write protection for the card is
    enabled.

  Precondition:
    The DRV_SDCARD_Initialize routine must have been called.

    DRV_SDCARD_Open must have been called to obtain a valid opened device
    handle.

    SD Card must have been detected and initialized by the task routine.

  Parameters:
    handle - A valid open-instance handle, returned from the driver's
                open routine

  Returns:
    true - Write protection is enabled.
    false - Write protection is disabled.

  Example:
    <code>
    DRV_HANDLE sdcardHandle;  // Returned from DRV_SDCARD_Open
    bool writeProtectStatus;

    //Call DRV_SDCARD_Open
    writeProtectStatus = DRV_SDCARD_WriteProtectionIsEnabled(sdcardHandle);

    </code>

  Remarks:
    This routine will not block for hardware access and will immediately return
    the current status.
*/

bool DRV_SDCARD_WriteProtectionIsEnabled ( DRV_HANDLE handle );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Version Information
// *****************************************************************************
// *****************************************************************************
/* These functions return the version information of the SD card driver */

// *****************************************************************************
/* Function:
    unsigned int DRV_SDCARD_VersionGet ( const SYS_MODULE_INDEX drvIndex )

  Summary:
    Gets SD card driver version in numerical format.

  Description:
    This routine gets the SD card driver version. The version is encoded as
    major * 10000 + minor * 100 + patch. The stringized version can be obtained
    using DRV_SDCARD_VersionStrGet()

  Precondition:
    None.

  Parameters:
    drvIndex    - Identifier for the object instance to get the version for.

  Returns:
    Current driver version in numerical format.

  Example:
    <code>
    unsigned int version;

    version = DRV_SDCARD_VersionGet( DRV_SDCARD_INDEX_1 );

    if(version < 110200)
    {
        // Do Something
    }
    </code>

  Remarks:
    None.
*/

unsigned int DRV_SDCARD_VersionGet ( const SYS_MODULE_INDEX drvIndex );


// *****************************************************************************
/* Function:
    char * DRV_SDCARD_VersionStrGet ( const SYS_MODULE_INDEX drvIndex )

  Summary:
    Gets SD card driver version in string format.

  Description:
    This routine gets the SD card driver version. The version is returned as
    major.minor.path[type], where type is optional. The numerical version can
    be obtained using DRV_SDCARD_VersionGet()

  Precondition:
    None.

  Parameters:
    drvIndex    - Identifier for the object instance to get the version for.

  Returns:
    Current SD card driver version in the string format.

  Example:
    <code>
    char *version;

    version = DRV_SDCARD_VersionStrGet( DRV_SDCARD_INDEX_1 );

    printf("%s", version);
    </code>

  Remarks:
    None.
*/

char* DRV_SDCARD_VersionStrGet ( const SYS_MODULE_INDEX drvIndex );


// *****************************************************************************
// *****************************************************************************
// Section: Included Files (continued)
// *****************************************************************************
// *****************************************************************************
/*  The file included below maps the interface definitions above to appropriate
    static implementations, depending on build mode.
*/

#include "driver/sdcard/drv_sdcard_mapping.h"


#endif // #ifndef _DRV_SDCARD_H

/*******************************************************************************
 End of File
*/

