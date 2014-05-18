/*******************************************************************************
  SPI Driver Interface

  Company:
    Microchip Technology Inc.

  File Name:
    drv_spi.h

  Summary:
    SPI device driver interface file.

  Description:
    The SPI  driver provides a simple interface to manage the SPI module.
    This file defines the interface definitions and prototypes for the SPI driver.
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


#ifndef _DRV_SPI_H
#define _DRV_SPI_H


// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "system_config.h"
#include "driver/driver_common.h"			// Common Driver Definitions
#include "peripheral/spi/plib_spi.h"		// SPI PLIB Header
#include "system/common/sys_common.h"      	// Common System Service Definitions
#include "system/common/sys_module.h"      	// Module/Driver Definitions
#include "system/int/sys_int.h"            	// System Interrupt Definitions
#include "system/clk/sys_clk.h"
#include "system/ports/sys_ports.h"
#include "peripheral/dma/plib_dma.h"

// *****************************************************************************
/* SPI Driver Buffer Handle

  Summary:
    Handle identifying a read or write buffer passed to the driver.

  Description:
    A buffer handle value is returned by a call to the DRV_SPI_BufferAddRead()/
    DRV_SPI_BufferAddWriteor DRV_SPI_BufferAddReadWrite() functions. This handle
	is associated with the buffer passed into the function and it allows the
	application to track the completion of the data from (or into) that buffer.
	The buffer handle value returned from the "buffer add" function is returned
	back to the client by the "callback" function registered with the driver.

    The buffer handle assigned to a client request expires when the client has
    been notified of the completion of the buffer transfer (after event handler
    function that notifies the client returns) or after the buffer has been
    retired by the driver if no event handler callback was set.

  Remarks:
    None
*/

typedef uintptr_t DRV_SPI_BUFFER_HANDLE;


// *****************************************************************************
/* SPI Driver Invalid Buffer Handle

  Summary:
    Definition of an invalid buffer handle.

  Description:
    This is the definition of an invalid buffer handle. An invalid buffer handle
    is returned by DRV_SPI_BufferAddRead() and DRV_SPI_BufferAddWrite()
    function if the buffer add request was not successful.

  Remarks:
    None
*/

#define DRV_SPI_BUFFER_HANDLE_INVALID ((DRV_SPI_BUFFER_HANDLE)(-1))


// *****************************************************************************
/* SPI Driver Module Index Numbers

  Summary:
    SPI driver index definitions.

  Description:
    These constants provide the SPI driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_SPI_Initialize and
    DRV_SPI_Open functions to identify the driver instance in use.
*/

#define DRV_SPI_INDEX_0         0
#define DRV_SPI_INDEX_1         1
#define DRV_SPI_INDEX_2         2
#define DRV_SPI_INDEX_3         3
#define DRV_SPI_INDEX_4         4
#define DRV_SPI_INDEX_5         5


// *****************************************************************************
/* SPI Driver Module Index Count

  Summary:
    Number of valid SPI driver indices.

  Description:
    This constant identifies the number of valid SPI driver indices.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is derived from device-specific header files defined as part of the
    peripheral libraries.
*/

#define DRV_SPI_INDEX_COUNT     SPI_NUMBER_OF_MODULES


// *****************************************************************************
/* SPI Clock Mode Selection

  Summary:
    Identifies the various clock modes of the SPI module.

  Description:
    This enumeration identifies the various clock modes of the SPI module.

  Remarks:
    None.
*/

typedef enum
{
    /* SPI Clock Mode 0 - Idle State Low & Sampling on Rising Edge */
    DRV_SPI_CLOCK_MODE_IDLE_LOW_EDGE_RISE
        /*DOM-IGNORE-BEGIN*/  = 0 /*DOM-IGNORE-END*/,

    /* SPI Clock Mode 1 - Idle State Low & Sampling on Falling Edge */
    DRV_SPI_CLOCK_MODE_IDLE_LOW_EDGE_FALL
        /*DOM-IGNORE-BEGIN*/  = 1 /*DOM-IGNORE-END*/,

    /* SPI Clock Mode 2 - Idle State High & Sampling on Falling Edge */
    DRV_SPI_CLOCK_MODE_IDLE_HIGH_EDGE_FALL
        /*DOM-IGNORE-BEGIN*/  = 2 /*DOM-IGNORE-END*/,

    /* SPI Clock Mode 3 - Idle State High & Sampling on Rising Edge */
    DRV_SPI_CLOCK_MODE_IDLE_HIGH_EDGE_RISE
        /*DOM-IGNORE-BEGIN*/  = 3 /*DOM-IGNORE-END*/

} DRV_SPI_CLOCK_MODE;


// *****************************************************************************
/* SPI Buffer Type Selection

  Summary:
    Identifies the various buffer types of the SPI module.

  Description:
    This enumeration identifies the various buffer types of the SPI module.

  Remarks:
    None.
*/

typedef enum
{
    /* SPI Buffer Type Standard */
    DRV_SPI_BUFFER_TYPE_STANDARD
        /*DOM-IGNORE-BEGIN*/  = 0 /*DOM-IGNORE-END*/,

    /* SPI Enhanced Buffer Type */
    DRV_SPI_BUFFER_TYPE_ENHANCED
        /*DOM-IGNORE-BEGIN*/  = 1 /*DOM-IGNORE-END*/

} DRV_SPI_BUFFER_TYPE;


// *****************************************************************************
/* SPI Protocols Enumeration

  Summary:
    Identifies the various protocols of the SPI module.

  Description:
    This enumeration identifies the various protocols of the SPI module.

  Remarks:
    None.
*/

typedef enum
{
    /* SPI Protocol Type Standard */
    DRV_SPI_PROTOCOL_TYPE_STANDARD
        /*DOM-IGNORE-BEGIN*/  = 0 /*DOM-IGNORE-END*/,

    /* SPI Protocol Type Framed */
    DRV_SPI_PROTOCOL_TYPE_FRAMED
        /*DOM-IGNORE-BEGIN*/  = 1 /*DOM-IGNORE-END*/,

    /*SPI Protocol Type Audio*/
    DRV_SPI_PROTOCOL_TYPE_AUDIO
        /*DOM-IGNORE-BEGIN*/  = 2 /*DOM-IGNORE-END*/

} DRV_SPI_PROTOCOL_TYPE;




// *****************************************************************************
/* SPI Driver Buffer Events

   Summary
    Identifies the possible events that can result from a buffer add request.

   Description
    This enumeration identifies the possible events that can result from a
    buffer add request caused by the client calling either the
    DRV_NVM_BufferAddRead or DRV_NVM_BufferAddWrite functions.

   Remarks:
    One of these values is passed in the "event" parameter of the event
    handling callback function that the client registered with the driver by
    calling the DRV_NVM_BufferEventHandlerSet function when a buffer
    transfer request is completed.

*/

typedef enum
{
    /* Buffer is pending to get processed */
    DRV_SPI_BUFFER_EVENT_PENDING,

    /* All data from or to the buffer was transferred successfully. */
    DRV_SPI_BUFFER_EVENT_COMPLETE,

    /* There was an error while processing the buffer transfer request. */
    DRV_SPI_BUFFER_EVENT_ERROR

}DRV_SPI_BUFFER_EVENT;


// *****************************************************************************
/* SPI Usage Modes Enumeration

  Summary:
    Identifies the various usage modes of the SPI module.

  Description:
    This enumeration identifies the various usage modes of the SPI module.

  Remarks:
    None.
*/

typedef enum
{
    /* SPI Mode Master */
    DRV_SPI_MODE_MASTER
        /*DOM-IGNORE-BEGIN*/  = 0 /*DOM-IGNORE-END*/,

    /* SPI Mode Slave */
    DRV_SPI_MODE_SLAVE
        /*DOM-IGNORE-BEGIN*/  = 1 /*DOM-IGNORE-END*/

} DRV_SPI_MODE;


// *****************************************************************************
/* SPI Driver Buffer Event Handler Function Pointer

   Summary
    Pointer to a SPI Driver Buffer Event handler function

   Description
    This data type defines the required function signature for the SPI driver
    buffer event handling callback function. A client must register a pointer
    to a buffer event handling function who's function signature (parameter
    and return value types) match the types specified by this function pointer
    in order to receive buffer related event calls back from the driver.

    The parameters and return values and return value are described here and
    a partial example implementation is provided.

  Parameters:
    event           - Identifies the type of event

    bufferHandle    - Handle identifying the buffer to which the vent relates

	context         - Value identifying the context of the application that
					  registered the event handling function.

  Returns:
    None.

  Example:
    <code>
    void APP_MyBufferEventHandler( DRV_SPI_BUFFER_EVENT event,
                                   DRV_SPI_BUFFER_HANDLE bufferHandle,
                                   uintptr_t context )
    {
        MY_APP_DATA_STRUCT pAppData = (MY_APP_DATA_STRUCT) context;

        switch(event)
        {
            case DRV_SPI_BUFFER_EVENT_COMPLETE:

                // Handle the completed buffer.
                break;

            case DRV_SPI_BUFFER_EVENT_ERROR:
            default:

                // Handle error.
                break;
        }
    }
    </code>

  Remarks:
    If the event is DRV_SPI_BUFFER_EVENT_COMPLETE, it means that the data was
    transferred successfully.

    If the event is DRV_SPI_BUFFER_EVENT_ERROR, it means that the data was
    not transferred successfully.

    The bufferHandle parameter contains the buffer handle of the buffer that
    failed.

    The context parameter contains the a handle to the client context,
    provided at the time the event handling function was  registered using the
    DRV_SPI_BufferEventHandlerSet function.  This context handle value is
    passed back to the client as the "context" parameter.  It can be any value
    necessary to identify the client context or instance (such as a pointer to
    the client's data) instance of the client that made the buffer add request.

    The event handler function executes in an interrupt context when the driver
    is configured for interrupt mode operation. It is recommended of the
    application to not perform process intensive operations with in this
    function.
*/

typedef void ( *DRV_SPI_BUFFER_EVENT_HANDLER )  (DRV_SPI_BUFFER_EVENT event,
        DRV_SPI_BUFFER_HANDLE bufferHandle, uintptr_t context );


// *****************************************************************************
/* SPI Driver Initialization Data

  Summary:
    Defines the data required to initialize or reinitialize the SPI driver

  Description:
    This data type defines the data required to initialize or reinitialize the
    SPI driver. If the driver is built statically, the members of this data
    structure are statically over-ridden by static override definitions in the
    system_config.h file.

  Remarks:
    None.
*/

typedef struct _DRV_SPI_INIT
{
    /* System module initialization */
    SYS_MODULE_INIT                 		moduleInit;

    /* Identifies peripheral (PLIB-level) ID */
    SPI_MODULE_ID                   		spiId;

    /* SPI Usage Mode Type */
    DRV_SPI_MODE                    		spiMode;

    /* SPI Protocol Type */
    DRV_SPI_PROTOCOL_TYPE           		spiProtocolType;

    /* Communication Width */
    SPI_COMMUNICATION_WIDTH         		commWidth;

    /* Baud Rate Value */
    uint32_t                        		baudRate;

    /* SPI Buffer Type */
    DRV_SPI_BUFFER_TYPE             		bufferType;

    /* FIFO TX Interrupt Mode */
    SPI_FIFO_INTERRUPT              		txInterruptMode;

    /* FIFO RX Interrupt Mode */
    SPI_FIFO_INTERRUPT              		rxInterruptMode;

    /* SPI Clock mode */
    DRV_SPI_CLOCK_MODE              		clockMode;

    /* SPI Input Sample Phase Selection */
    SPI_INPUT_SAMPLING_PHASE        		inputSamplePhase;

    /* Transmit/Receive or Transmit Interrupt Source for SPI module */
    INT_SOURCE                      		txInterruptSource;

    /* Receive Interrupt Source for SPI module */
    INT_SOURCE                      		rxInterruptSource;

    /* Error Interrupt Source for SPI module */
    INT_SOURCE                      		errInterruptSource;

	/* This is the buffer queue size. This is the maximum
     number of  transfer requests that driver will queue. For a
     static build of the driver, this is overridden by the
     DRV_SPI_QUEUE_SIZE macro in system_config.h */
    unsigned int                    		queueSize;
#if 0
    /* This is the SPI transmit DMA channel. This is applicable
     * only if DRV_SPI_SUPPORT_TRANSMIT_DMA is defined as true.
     * For a static build of the driver, this is over-ridden by the
     * DRV_SPI_TRANSMIT_DMA_CHANNEL macro in system_config.h */
    DMA_CHANNEL_ID                  		dmaTransmit;

    /* This is the SPI receive DMA channel. This is applicable
     * only if DRV_SPI_SUPPORT_RECEIVE_DMA is defined as true.
     * For a static build of the driver, this is over-ridden by the
     * DRV_SPI_RECEIVE_DMA_CHANNEL macro in system_config.h */
    DMA_CHANNEL_ID                  		dmaReceive;
#endif
    /* This is the SPI transmit DMA channel interrupt. This is
     * applicable only if DRV_SPI_SUPPORT_TRANSMIT_DMA is defined
     * as true. For a static build of the driver, this is over-ridden
     * by the DRV_SPI_INTERRUPT_SOURCE_TRANSMIT_DMA_CHANNEL macro
     * in system_config.h */
    INT_SOURCE                      dmaInterruptTransmit;

    /* This is the SPI receive DMA channel interrupt. This is
     * applicable only if DRV_SPI_SUPPORT_RECEIVE_DMA is defined
     * as true. For a static build of the driver, this is over-ridden
     * by the DRV_SPI_INTERRUPT_SOURCE_RECEIVE_DMA_CHANNEL macro
     * in system_config.h */
    INT_SOURCE                      dmaInterruptReceive;

	/* This is the transmit DMA channel.
	A value of DMA_CHANNEL_NONE indicates DMA is not required for
	TX. For a static build of the driver this is overridden by the
	DRV_I2S_TRANSMIT_DMA_CHANNEL macro in system_config.h. */
	DMA_CHANNEL dmaChannelTransmit;

	/* This is the receive DMA channel.
	A value of DMA_CHANNEL_NONE indicates DMA is not required for
	Rx. For a static build of the driver this is overridden by the
	DRV_I2S_RECEIVE_DMA_CHANNEL macro in system_config.h. */
	DMA_CHANNEL dmaChannelReceive;

	/* This is the transmit DMA channel interrupt. This is
	applicable only if 'dmaChannelTransmit' has a valid channel number.
	This takes the interrupt source number for the corresponding
	DMA channel. */
	INT_SOURCE dmaInterruptTransmitSource;

	/* This is the receive DMA channel interrupt. This is
	applicable only if 'dmaChannelReceive' has a valid channel number.
	This takes the interrupt source number for the corresponding
	DMA channel. */
	INT_SOURCE dmaInterruptReceiveSource;
} DRV_SPI_INIT;


// *****************************************************************************
/* SPI Client Initialization Data

  Summary:
    Defines the data required to initialize an SPI client.

  Description:
    This structure lists the elements needed to set up an SPI client. Used by
    the function DRV_SPI_ClientSetup function. This data type defines the data
	required to initialize an SPI client.

  Remarks:
    None.
*/

typedef struct _DRV_SPI_CLIENT_SETUP
{
    /* Variable specifying the baud.  */
    uint32_t                    		baudRate;

    /* SPI Input Sample Phase Selection */
    SPI_INPUT_SAMPLING_PHASE    		inputSamplePhase;

    /* SPI Clock mode */
    DRV_SPI_CLOCK_MODE          		clockMode;

    /* Set this bit if it has to be logic high to
    assert the chip select */
    bool 					chipSelectLogicLevel;

    /* PORT which the chip select pin belongs to */
    PORTS_CHANNEL 				chipSelectPort;

    /* Bit position in the port */
    PORTS_BIT_POS 				chipSelectBitPos;

} DRV_SPI_CLIENT_SETUP;


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - System Level
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
      SYS_MODULE_OBJ DRV_SPI_Initialize( const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init )

  Summary:
    Initializes the SPI instance for the specified driver index

  Description:
    This routine initializes the SPI driver instance for the specified driver
    index, making it ready for clients to open and use it. The initialization
    data is specified by the 'init' parameter. The initialization may fail if the
    number of driver objects allocated are insufficient or if the specified
    driver instance is already initialized. The driver instance index is
    independent of the SPI module ID. For example, driver instance 0 can be
    assigned to SPI2.  If the driver is built statically, then some of the
    initialization parameters are overridden by configuration macros. Refer to
    the description of the DRV_SPI_INIT data structure for more details on
    which members on this data structure are overridden.

  Precondition:
    None.

  Input:
    index  - Identifier for the instance to be initialized

    init   - Pointer to a data structure containing any data necessary to
             initialize the driver. If this pointer is NULL, the driver
             uses the static initialization override macros for each
             member of the initialization data structure.

  Return:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, returns SYS_MODULE_OBJ_INVALID.

  Example:
    <code>
    DRV_SPI_INIT        init;
    SYS_MODULE_OBJ      objectHandle;

    // Populate the SPI initialization structure
    init.spiId              = SPI_ID_1;
    init.spiMode            = DRV_SPI_MODE_MASTER;
    init.spiProtocolType    = DRV_SPI_PROTOCOL_TYPE_STANDARD;
    init.commWidth          = SPI_COMMUNICATION_8BIT_WIDE;
    init.baudRate           = 5000;
    init.bufferType         = DRV_SPI_BUFFER_TYPE_STANDARD;
    init.inputSamplePhase   = SPI_INPUT_SAMPLING_PHASE_IN_MIDDLE;
    init.clockMode          = DRV_SPI_CLOCK_MODE_IDLE_LOW_EDGE_RISE;
    init.txInterruptSource  = INT_SOURCE_SPI_1_TRANSMIT; // INT_SOURCE_SPI_1_EVENT
    init.rxInterruptSource  = INT_SOURCE_SPI_1_RECEIVE;  // INT_SOURCE_SPI_1_EVENT
    init.errInterruptSource = INT_SOURCE_SPI_1_ERROR;

    objectHandle = DRV_SPI_Initialize(DRV_SPI_INDEX_1, (SYS_MODULE_INIT*)usartInitData);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>
  Remarks:
    This routine must be called before any other SPI routine is called.

    This routine should only be called once during system initialization
    unless DRV_SPI_Deinitialize is called to deinitialize the driver
    instance. This routine will NEVER block for hardware access.
*/

SYS_MODULE_OBJ DRV_SPI_Initialize( const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init );


/*************************************************************************************
  Function:
       SYS_MODULE_OBJ DRV_SPI_Initialize ( const SYS_MODULE_INDEX        index,
                                          const SYS_MODULE_INIT * const init )

  Summary:
    Initializes the SPI driver.

  Description:
    This function initializes the SPI driver, making it ready for clients
    to open and use.

  Conditions:
    None.

  Input:
    drvIndex -  Index for the driver instance to be initialized

  	init -      Pointer to a data structure containing any data necessary to
                initialize the driver. This pointer may be null if no data
                is required because static overrides have been provided.

  Return:
    If successful, a valid handle to a driver object is returned.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID. The returned object must
    be passed as an argument to the DRV_SPI_Reinitialize,
    DRV_SPI_Deinitialize, DRV_SPI_Tasks, and DRV_SPI_Status functions.

  Example:
    <code>
    DRV_SPI_INIT        init;
    SYS_MODULE_OBJ      objectHandle;

    // Populate the SPI initialization structure
    init.spiId              = SPI_ID_1;
    init.spiMode            = DRV_SPI_MODE_MASTER;
    init.spiProtocolType    = DRV_SPI_PROTOCOL_TYPE_STANDARD;
    init.commWidth          = SPI_COMMUNICATION_8BIT_WIDE;
    init.baudRate           = 5000;
    init.bufferType         = DRV_SPI_BUFFER_TYPE_STANDARD;
    init.inputSamplePhase   = SPI_INPUT_SAMPLING_PHASE_IN_MIDDLE;
    init.clockMode          = DRV_SPI_CLOCK_MODE_IDLE_LOW_EDGE_RISE;
    init.txInterruptSource  = INT_SOURCE_SPI_1_TRANSMIT; // INT_SOURCE_SPI_1_EVENT
    init.rxInterruptSource  = INT_SOURCE_SPI_1_RECEIVE;  // INT_SOURCE_SPI_1_EVENT
    init.errInterruptSource = INT_SOURCE_SPI_1_ERROR;

    // Do something

    objectHandle = DRV_SPI_Initialize( DRV_SPI_INDEX_0, (SYS_MODULE_INIT*)&init );
    if( SYS_MODULE_OBJ_INVALID == objectHandle )
    {
        // Handle error
    }
    </code>

  Remarks:
    This function must be called before any other SPI routine is called.

    This function should only be called once during system initialization
    unless DRV_SPI_Deinitialize is called to deinitialize the driver
    instance.

    This function will NEVER block for hardware access. If the operation
    requires time to allow the hardware to initialize, it will be
    reported by the DRV_SPI_Status operation. The system must use
    DRV_SPI_Status to find out when the driver is in the ready state.

    Build configuration options may be used to statically override options
    in the "init" structure and will take precedence over initialization
    data passed using this function.
*/

SYS_MODULE_OBJ DRV_SPI_Initialize ( const SYS_MODULE_INDEX        index,
                                   const SYS_MODULE_INIT * const init );


/*************************************************************************
  Function:
       void DRV_SPI_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    De-initializes the specified instance of the SPI driver module.

  Description:
    De-initializes the specified instance of the SPI driver module,
    disabling its operation (and any hardware) and invalidates all of the
    internal data.

  Conditions:
    Function DRV_SPI_Initialize must have been called before calling this
    routine and a valid SYS_MODULE_OBJ must have been returned.

  Input:
    object -  Driver object handle, returned from DRV_SPI_Initialize

  Return:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     //  Returned from DRV_SPI_Initialize
    SYS_STATUS          status;

    DRV_SPI_Deinitialize ( object );

    status = DRV_SPI_Status( object );
    if( SYS_MODULE_UNINITIALIZED == status )
    {
        // Check again later if you need to know
        // when the driver is deinitialized.
    }
    </code>

  Remarks:
    Once the Initialize operation has been called, the De-initialize
    operation must be called before the Initialize operation can be called
    again.

    This function will NEVER block waiting for hardware. If the operation
    requires time to allow the hardware to complete, this will be reported
    by the DRV_SPI_Status operation. The system has to use DRV_SPI_Status
    to find out when the module is in the ready state.
*/

void DRV_SPI_Deinitialize ( SYS_MODULE_OBJ object );


/**************************************************************************
  Function:
       SYS_STATUS DRV_SPI_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the SPI driver module.

  Description:
    This function provides the current status of the SPI driver module.

  Conditions:
    The DRV_SPI_Initialize function must have been called before calling
    this function.

  Input:
    object -  Driver object handle, returned from DRV_SPI_Initialize

  Return:
    SYS_STATUS_READY - Indicates that the driver is busy with a previous
    system level operation and cannot start another

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_SPI_Initialize
    SYS_STATUS          status;

    status = DRV_SPI_Status( object );
    if( SYS_STATUS_READY != status )
    {
        // Handle error
    }
    </code>

  Remarks:
    Any value greater than SYS_STATUS_READY is also a normal running state
    in which the driver is ready to accept new operations.

    SYS_MODULE_UNINITIALIZED - Indicates that the driver has been
    deinitialized

    This value is less than SYS_STATUS_ERROR.

    This function can be used to determine when any of the driver's module
    level operations has completed.

    If the status operation returns SYS_STATUS_BUSY, the previous operation
    has not yet completed. Once the status operation returns
    SYS_STATUS_READY, any previous operations have completed.

    The value of SYS_STATUS_ERROR is negative (-1). Any value less than
    that is also an error state.

    This function will NEVER block waiting for hardware.

    If the Status operation returns an error value, the error may be
    cleared by calling the reinitialize operation. If that fails, the
    deinitialize operation will need to be called, followed by the
    initialize operation to return to normal operations.
*/

SYS_STATUS DRV_SPI_Status ( SYS_MODULE_OBJ object );


// *****************************************************************************
/* Function:
    void DRV_SPI_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's state machine and implements its ISR.

  Description:
	This routine is used to maintain the driver's internal state
	machine and implement its transmit ISR for interrupt-driven implementations.
	In polling mode, this function should be called from the SYS_Tasks()
	function. In interrupt mode, this function should be called in the transmit
	interrupt service routine of the USART that is associated with this USART
	driver hardware instance.


  Precondition:
    The DRV_SPI_Initialize routine must have been called for the specified
    SPI driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_SPI_Initialize)

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_SPI_Initialize

    while( true )
    {
        DRV_SPI_Tasks ( object );

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks) or by the appropriate raw
    ISR.

    This function may execute in an ISR context and will never block or access any
    resources that may cause it to block.
*/

void DRV_SPI_Tasks ( SYS_MODULE_OBJ object );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client Level
// *****************************************************************************
// *****************************************************************************

/**************************************************************************
  Function:
       DRV_HANDLE DRV_SPI_Open ( const SYS_MODULE_INDEX drvIndex,
                                const DRV_IO_INTENT    intent )

  Summary:
    Opens the specified SPI driver instance and returns a handle to it.
  Description:
    This routine opens the specified USART driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.

    The DRV_IO_INTENT_BLOCKING and DRV_IO_INTENT_NONBLOCKING ioIntent
    options additionally affect the behavior of the DRV_USART_Read() and
    DRV_USART_Write() functions. If the ioIntent is
    DRV_IO_INTENT_NONBLOCKING, then these function will not block even if
    the required amount of data could not be processed. If the ioIntent is
    DRV_IO_INTENT_BLOCKING, these functions will block until the required
    amount of data is processed.

    If ioIntent is DRV_IO_INTENT_READ, the client will only be read from
    the driver. If ioIntent is DRV_IO_INTENT_WRITE, the client will only be
    able to write to the driver. If the ioIntent in
    DRV_IO_INTENT_READWRITE, the client will be able to do both, read and
    write.

    Specifying a DRV_IO_INTENT_EXCLUSIVE will cause the driver to provide
    exclusive access to this client. The driver cannot be opened by any
    other client.
  Conditions:
    The DRV_SPI_Initialize function must have been called before calling
    this function.
  Input:
    drvIndex -  Identifier for the object instance to be opened
    intent -    Zero or more of the values from the enumeration
                DRV_IO_INTENT ORed together to indicate the intended use of
                the driver
  Return:
    If successful, the routine returns a valid open-instance handle (a
    number identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID. An error
    can occur when the following is true:
      * if the number of client objects allocated via
        DRV_USART_CLIENTS_NUMBER is insufficient
      * if the client is trying to open the driver but driver has been
        opened exclusively by another client
      * if the driver hardware instance being opened is not initialized or
        is invalid
  Example:
    <code>
    DRV_HANDLE  handle;

    handle = DRV_SPI_Open( DRV_SPI_INDEX_0, DRV_IO_INTENT_EXCLUSIVE );

    if( DRV_HANDLE_INVALID == handle )
    {
        // Unable to open the driver
    }
    </code>
  Remarks:
    The handle returned is valid until the DRV_USART_Close routine is
    called. This routine will NEVER block waiting for hardware. If the
    requested intent flags are not supported, the routine will return
    DRV_HANDLE_INVALID. This function is thread safe in a RTOS application.
    It should not be called in an ISR.
*/

DRV_HANDLE DRV_SPI_Open ( const SYS_MODULE_INDEX drvIndex,
                         const DRV_IO_INTENT    intent );


// *****************************************************************************
/* Function:
    void DRV_SPI_Close ( DRV_HANDLE handle )

  Summary:
    Closes an opened instance of the SPI driver

  Description:
    This function closes an opened instance of the SPI driver, invalidating the
    handle.

  Precondition:
    The DRV_SPI_Initialize routine must have been called for the specified
    SPI driver instance.

    DRV_SPI_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_SPI_Open

    DRV_SPI_Close ( handle );
    </code>

  Remarks:
	After calling this routine, the handle passed in "handle" must not be used
    with any of the remaining driver routines.  A new handle must be obtained by
    calling DRV_USART_Open before the caller may use the driver again.  This
    function is thread safe in a RTOS application.

    Note:
    Usually there is no need for the driver client to verify that the Close
    operation has completed.
*/

void DRV_SPI_Close ( DRV_HANDLE handle );


/***************************************************************************************
  Function:
       void DRV_SPI_ClientSetup ( DRV_HANDLE                        handle,
                                        const DRV_SPI_CLIENT_SETUP * const clientSetup )

  Summary:
    Initializes a client.

  Description:
    This function sets up the device communication parameters. Using this
    API user can have configurations like chip select and baud rate
    specific to a client. The driver will be able to switch the between
    clients dynamically. 'DRV_SPI_CLIENT_SPECIFIC_CONTROL' must be defined
    in 'system_config.h'. This function is only useful for instance which
    has got multiple slaves.

  Conditions:
    The DRV_SPI_Initialize routine must have been called.

    DRV_SPI_Open must have been called to obtain a valid opened device
    handle.

  Input:
    handle -       A valid open instance handle, returned from the driver's
                   open routine
    clientSetup -  Communication parameters identified by
                   DRV_SPI_CLIENT_SETUP

  Return:
    None

  Example:
    <code>
    DRV_HANDLE      handle;    // Returned from DRV_SPI_Open
    DRV_SPI_CLIENT_SETUP clientSetup;

    // Populate the communication setup configuration structure
    clientSetup.baudRate         = 5000;
    clientSetup.inputSamplePhase = SPI_INPUT_SAMPLING_PHASE_IN_MIDDLE;
    clientSetup.clockMode        = DRV_SPI_CLOCK_MODE_IDLE_LOW_EDGE_RISE;
    clientSetup.callback         = &callBackFunction; // callback registered

    DRV_SPI_ClientSetup ( handle, &clientSetup );
    </code>

  Remarks:
    This is an optional call. It is necessary only two clients using needs
    different configurations.
*/

void DRV_SPI_ClientSetup ( DRV_HANDLE 					handle,
							const DRV_SPI_CLIENT_SETUP * const clientSetup );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client level Read & Write APIs
// *****************************************************************************
// *****************************************************************************
/* These are blocking APIs */

// *****************************************************************************
/* Function:
    size_t DRV_SPI_Read ( DRV_HANDLE handle, void* buffer,
    										size_t noOfBytes )

  Summary:
    Gets/Reads byte(s) from SPI. The function will wait until the specified
    number of bytes is received.

  Description:
    This function gets/reads byte(s) from SPI. The function will wait until
    the specified number of bytes is received.
  Precondition:
    The DRV_SPI_Initialize routine must have been called for the specified
    SPI driver instance.

    DRV_SPI_Open must have been called to obtain a valid opened device handle.

    DRV_IO_INTENT_READ or DRV_IO_INTENT_READWRITE must have been specified in
    the DRV_SPI_Open call.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
	buffer       - Buffer to which the data should be written to.

	noOfBytes 	 - Number of bytes to be received.

  Returns:
	The number of bytes read.

  Example:
    <code>
    DRV_HANDLE      handle;    // Returned from DRV_SPI_Open
    SPI_DATA_TYPE   myBuffer[MY_BUFFER_SIZE];
	unsigned int numberOfBytesRead;

	numberOfBytesRead = DRV_SPI_Read ( handle, myBuffer, 10 );

	if ( numberOfBytesWritten == 0 )
	{
		// Check for the error...
	}

    </code>

  Remarks:
    If the DRV_IO_INTENT_BLOCKING flag was given in the call to the
    DRV_SPI_Open function and the driver was built to support blocking behavior
    the call will block until the operation is complete.

    If the DRV_IO_INTENT_NONBLOCKING flag was given in the call to the
    DRV_SPI_Open function or the driver was built to only support non-blocking
    behavior the call will return immediately.  If data is not available, a zero
    ('0') value will be returned and an underrun error status will be captured.
    To ensure that valid data is returned, the caller must first check the
    return value to DRV_SPI_BufferStatus, as shown in the example.
*/

size_t DRV_SPI_Read ( DRV_HANDLE handle, void* buffer,
										size_t noOfBytes );


// *****************************************************************************
/* Function:
    size_t DRV_SPI_Write ( DRV_HANDLE handle, void* buffer,
    											size_t noOfBytes )

  Summary:
	Writes the data to SPI. The function will wait until all the bytes are
	transmitted.

  Description:
    This function Writes the data to SPI. The function will wait until all
    the bytes are transmitted.

  Precondition:
    The DRV_SPI_Initialize routine must have been called for the specified
    SPI driver instance.

    DRV_SPI_Open must have been called to obtain a valid opened device handle.

    DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have been specified in
    the DRV_SPI_Open call.

  Parameters:
	handle       - A valid open-instance handle, returned from the driver's
		  			open routine
	buffer       - Buffer which contains the data.

	noOfBytes 	 - Number of bytes to be transmitted.

  Returns:
    The number of bytes written.

  Example:
    <code>
	DRV_HANDLE      handle;    // Returned from DRV_SPI_Open
	SPI_DATA_TYPE   myBuffer[MY_BUFFER_SIZE];//fill the data
	unsigned int 	numberOfBytesWritten;

	numberOfBytesWritten = DRV_SPI_Write ( handle, myBuffer, 10 );

	if ( numberOfBytesWritten == 0 )
	{
		// Check the error...
	}
    </code>

  Remarks:
    If the DRV_IO_INTENT_BLOCKING flag was given in the call to the
    DRV_SPI_Open function and the driver was built to support blocking behavior
    the call will block until the operation is complete.

    If the DRV_IO_INTENT_NONBLOCKING flag was given in the call to the
    DRV_SPI_Open function or the driver was built to only support non-blocking
    behavior the call will return immediately.  If the driver was not able to
    accept the data, an overrun error status will be captured.  To ensure that
    the driver is ready to accept data, the caller must first check the
    return value to DRV_SPI_BufferStatus, as shown in the example.
*/

size_t DRV_SPI_Write ( DRV_HANDLE handle, void* buffer,
											size_t noOfBytes );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client level Read & Write APIs
// *****************************************************************************
// *****************************************************************************
/* These are non-blocking APIs. It doesn't wait until the operation gets
   finished. The actual operation will happen it the task routine. The status of
   this operation can be monitored using DRV_SPI_BufferStatus function. In
   polling mode, User must ensure that the code gets time to execute the task
   routine. */

/*******************************************************************************
  Function:
 	DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddRead ( DRV_HANDLE handle, void *rxBuffer,
                                       size_t size )

  Summary:
    Registers a buffer for a read operation. Actual transfer will happen in
    the Task function.

  Description:
    Registers a buffer for a read operation. Actual transfer will happen in
    the Task function. The status of this operation can be monitored using
    DRV_SPI_BufferStatus function.

  Conditions:
    The DRV_SPI_Initialize routine must have been called for the specified
    SPI driver instance.

    DRV_SPI_Open must have been called to obtain a valid opened device
    handle.

    DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have been specified
    in the DRV_SPI_Open call.

  Input:
    handle -    A valid open-instance handle, returned from the driver's
                open routine

  	rxBuffer -  The buffer to which the data should be written to.

  	size -      Number of bytes to be read.

  Return:
    None.

  Example:
    <code>
    DRV_HANDLE      handle;    // Returned from DRV_SPI_Open
    char   myBuffer[MY_BUFFER_SIZE], state = 0;
	DRV_SPI_BUFFER_HANDLE bufferHandle;

    switch ( state )
    {
		case 0:
			// PUT API returns data in any case, it is up to the user to use it
			bufferHandle = DRV_SPI_BufferAddRead( handle, myBuffer, 10 );
			state++;
			break;
		case 1:
			if( DRV_SPI_BUFFER_STATUS_SUCCESS & DRV_SPI_BufferStatus( bufferHandle ) )
			{
				state++;
				// All transmitter data has been sent.
			}
			break;
	}
    </code>

  Remarks:
    If the driver was not able to accept the data, an overrun error status will
    be captured. To ensure that the driver is ready to accept data, the caller
    must first check the return value to DRV_SPI_BufferStatus, as shown in
    the example.
*/

DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddRead ( DRV_HANDLE handle, void *rxBuffer,
                                    size_t size );


/*******************************************************************************
  Function:
 	DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddWrite ( DRV_HANDLE handle, void *txBuffer,
                                       size_t size )

  Summary:
    Registers a buffer for a write operation. Actual transfer will happen
    in the Task function.

  Description:
    Registers a buffer for a write operation. Actual transfer will happen
    in the Task function. The status of this operation can be monitored
    using DRV_SPI_BufferStatus function.

  Conditions:
    The DRV_SPI_Initialize routine must have been called for the specified
    SPI driver instance.

    DRV_SPI_Open must have been called to obtain a valid opened device
    handle.

    DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have been specified
    in the DRV_SPI_Open call.

  Input:
    handle -    A valid open-instance handle, returned from the driver's
                open routine
    txBuffer -  The buffer which hold the data.
    size -      Number of bytes to be read.

  Return:
    None.

  Example:
    <code>
    DRV_HANDLE      handle;    // Returned from DRV_SPI_Open
	char   myBuffer[MY_BUFFER_SIZE], state = 0;
	DRV_SPI_BUFFER_HANDLE bufferHandle;

	switch ( state )
	{
		case 0:
			// PUT API returns data in any case, it is up to the user to use it
			bufferHandle = DRV_SPI_BufferAddWrite( handle, myBuffer, 10 );
			state++;
			break;
		case 1:
			if( DRV_SPI_BUFFER_STATUS_SUCCESS & DRV_SPI_BufferStatus( bufferHandle ) )
			{
				state++;
				// All transmitter data has been sent.
			}
			break;
	}

    </code>

  Remarks:
    None.
*/

DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddWrite ( DRV_HANDLE handle, void *txBuffer,
                                    size_t size );


/*******************************************************************************
  Function:
       DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddWriteRead( DRV_HANDLE handle,
       					void *txBuffer, void *rxBuffer, size_t size )

  Summary:
    Registers a buffer for a read and write operation. Actual transfer will
    happen in the Task function.

  Description:
    Registers a buffer for a read and write operation. Actual transfer will
    happen in the Task function. The status of this operation can be
    monitored using DRV_SPI_BufferStatus function.

  Conditions:
    The DRV_SPI_Initialize routine must have been called for the specified
    SPI driver instance.

    DRV_SPI_Open must have been called to obtain a valid opened device
    handle.

  Input:
    handle -    A valid open-instance handle, returned from the driver's
                open routine
    txBuffer -  The buffer which hold the data.
    rxBuffer -  The buffer to which the data should be written to.
    size -      Number of bytes to be read.
  Returns:
    None.

  Example:
	<code>
	DRV_HANDLE      handle;    // Returned from DRV_SPI_Open
	char   myReadBuffer[MY_BUFFER_SIZE], myWriteBuffer[MY_BUFFER_SIZE], state = 0;
	DRV_SPI_BUFFER_HANDLE bufferHandle;

	switch ( state )
	{
		case 0:
			// PUT API returns data in any case, it is up to the user to use it
			bufferHandle = DRV_SPI_BufferAddWriteRead( handle, myWriteBuffer,
				myReadBuffer, 10 );

			state++;
			break;
		case 1:
			if( DRV_SPI_BUFFER_STATUS_SUCCESS & DRV_SPI_BufferStatus( bufferHandle ) )
			{
				state++;
				// All transmitter data has been sent.
			}
			break;
	}

	</code>

  Remarks:

    If the driver was not able to accept the data, an overrun error status will
    be captured. To ensure that the driver is ready to accept data, the caller
    must first check the return value to DRV_SPI_BufferStatus, as shown in
    the example.
*/

DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddWriteRead ( DRV_HANDLE handle, void *txBuffer,
                                    void *rxBuffer, size_t size );


// *****************************************************************************
/* Function:
    DRV_SPI_BUFFER_STATUS DRV_SPI_BufferStatus ( DRV_SPI_BUFFER_HANDLE bufferHandle )

  Summary:
    Returns the transmitter and receiver transfer status.

  Description:
    This returns the transmitter and receiver transfer status.

  Precondition:
    The DRV_SPI_Initialize routine must have been called for the specified
    SPI driver instance.

    DRV_SPI_Open must have been called to obtain a valid opened device handle.


  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    A DRV_SPI_BUFFER_STATUS value describing the current status of the
    transfer.

  Example:
    <code>
    DRV_HANDLE handle;    // Returned from DRV_SPI_Open

    if( DRV_SPI_BUFFER_STATUS_SUCCESS & DRV_SPI_BufferStatus( handle ) )
    {
        // All transmitter data has been sent.
    }
    </code>

  Remarks:
    The returned status may contain a value with more than one of the bits
    specified in the DRV_SPI_BUFFER_STATUS enumeration set.  The caller
    should perform an AND with the bit of interest and verify if the
    result is non-zero (as shown in the example) to verify the desired status
    bit.
*/

DRV_SPI_BUFFER_EVENT DRV_SPI_BufferStatus ( DRV_SPI_BUFFER_HANDLE bufferHandle );


// *****************************************************************************
/* Function:
    void DRV_SPI_BufferEventHandlerSet (const DRV_HANDLE handle,
                        const DRV_SPI_BUFFER_EVENT_HANDLER eventHandler,
                        const uintptr_t context );

  Summary:
    Allows a client to identify a buffer event handling function for the driver
    to call back when queued buffer transfers have finished.

  Description:
    This function allows a client to identify a buffer event handling function
    for the driver to call back when queued buffer transfers have finished.
    When a client calls either the DRV_SPI_BufferAddRead or
    DRV_SPI_BufferAddWrite function, it is provided with a handle identifying
    the buffer that was added to the driver's buffer queue.  The driver will
    pass this handle back to the client by calling "eventHandler" function when
    the buffer transfer has completed.

    The event handler should be set before the client performs any "buffer add"
    operations that could generate events. The event handler once set, persists
    until the client closes the driver or sets another event handler (which
    could be a "NULL" pointer to indicate no callback).

  Precondition:
    The DRV_SPI_Initialize routine must have been called for the specified
    SPI driver instance.

    DRV_SPI_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

    eventHandler - Pointer to the event handler function.

    context      - The value of parameter will be passed back to the client
                   unchanged, when the eventHandler function is called.  It can
                   be used to identify any client specific data object that
                   identifies the instance of the client module (for example,
                   it may be a pointer to the client module's state structure).

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific state data object.
    MY_APP_OBJ myAppObj;

    uint8_t myBuffer[MY_BUFFER_SIZE];
    DRV_SPI_BUFFER_HANDLE bufferHandle;

    // mySPIHandle is the handle returned
    // by the DRV_SPI_Open function.

    // Client registers an event handler with driver. This is done once

    DRV_SPI_BufferEventHandlerSet( mySPIHandle, APP_SPIBufferEventHandle,
                                     (uintptr_t)&myAppObj );

    bufferHandle = DRV_SPI_BufferAddRead( mySPIhandle,
                                        myBuffer, MY_BUFFER_SIZE );

    if( DRV_SPI_BUFFER_HANDLE_INVALID == bufferHandle )
    {
        // Error handling here
    }

    // Event Processing Technique. Event is received when
    // the buffer is processed.

    void APP_SPIBufferEventHandler( DRV_SPI_BUFFER_EVENT event,
            DRV_SPI_BUFFER_HANDLE handle, uintptr_t context )
    {
        // The context handle was set to an application specific
        // object. It is now retrievable easily in the event handler.
        MY_APP_OBJ myAppObj = (MY_APP_OBJ *) context;

        switch(event)
        {
            case DRV_SPI_BUFFER_EVENT_SUCCESS:

                // This means the data was transferred.
                break;

            case DRV_SPI_BUFFER_EVENT_FAILURE:

                // Error handling here.

                break;

            default:
                break;
        }
    }
    </code>

  Remarks:
    If the client does not want to be notified when the queued buffer transfer
    has completed, it does not need to register a callback.
*/

void DRV_SPI_BufferEventHandlerSet (const DRV_HANDLE handle,
                    const DRV_SPI_BUFFER_EVENT_HANDLER eventHandler,
                    const uintptr_t context );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Version Information
// *****************************************************************************
// *****************************************************************************
/* These functions return the version information of the SPI driver */

// *****************************************************************************
/* Function:
    unsigned int DRV_SPI_VersionGet ( const SYS_MODULE_INDEX drvIndex )

  Summary:
    Gets the SPI driver version in numerical format.

  Description:
    This function gets the SPI driver version. The version is encoded as
    major * 10000 + minor * 100 + patch. The stringed version can be obtained
    using DRV_SPI_VersionStrGet.

  Precondition:
    None.

  Parameters:
    drvIndex    - Identifier for the object instance to get the version for

  Returns:
    Current driver version in numerical format.

  Example:
    <code>
    unsigned int version;

    version = DRV_SPI_VersionGet( DRV_SPI_INDEX_0 );

    if(version < 110200)
    {
        // Do Something
    }
    </code>

  Remarks:
    None.
*/

unsigned int DRV_SPI_VersionGet ( const SYS_MODULE_INDEX drvIndex );


// *****************************************************************************
/* Function:
    char * DRV_SPI_VersionStrGet ( const SYS_MODULE_INDEX drvIndex )

  Summary:
    Gets the SPI driver version in string format.

  Description:
    This function gets the SPI driver version. The version is returned as
    major.minor.path[type], where type is optional. The numerical version can
    be obtained using DRV_SPI_VersionGet.

  Precondition:
    None.

  Parameters:
    drvIndex    - Identifier for the object instance to get the version for

  Returns:
    Current the SPI driver version in the string format.

  Example:
    <code>
    char *version;

    version = DRV_SPI_VersionStrGet( DRV_SPI_INDEX_0 );

    printf("%s", version);
    </code>

  Remarks:
    None.
*/

char * DRV_SPI_VersionStrGet ( const SYS_MODULE_INDEX drvIndex );


// *****************************************************************************
// *****************************************************************************
// Section: Included Files (continued)
// *****************************************************************************
// *****************************************************************************
/*  The file included below maps the interface definitions above to appropriate
    static implementations, depending on build mode.
*/

//#include "driver/spi/drv_spi_mapping.h"


#endif // #ifndef _DRV_SPI_H

/*******************************************************************************
 End of File
*/

