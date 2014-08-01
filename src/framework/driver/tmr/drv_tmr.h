/*******************************************************************************
  Timer Device Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_tmr.h

  Summary:
    Timer device driver interface header.

  Description:
    This header file contains the function prototypes and definitions of the
    data types and constants that make up the interface to the Timer device
    driver.
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

#ifndef _DRV_TMR_H
#define _DRV_TMR_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
/* Note:  A file that maps the interface definitions above to appropriate static
          implementations (depending on build mode) is included at the bottom of
          this file.
*/

#include <stdint.h>
#include <stdbool.h>

#include "system_config.h"
#include "system/common/sys_common.h"      // Common System Service Definitions
#include "driver/driver_common.h"
#include "peripheral/tmr/plib_tmr.h"
#include "system/common/sys_module.h"      // Module/Driver Definitions
#include "system/int/sys_int.h"            // System Interrupt Definitions
#include "system/clk/sys_clk.h"

// *****************************************************************************
// *****************************************************************************
// Section: TMR Driver Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Timer Driver Module Index Numbers

  Summary:
    Timer driver index definitions

  Description:
    These constants provide Timer driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_TMR_Initialize and  DRV_TMR_Open
    functions to identify the driver instance in use.
*/

#define DRV_TMR_INDEX_0         0
#define DRV_TMR_INDEX_1         1
#define DRV_TMR_INDEX_2         2
#define DRV_TMR_INDEX_3         3
#define DRV_TMR_INDEX_4         4
#define DRV_TMR_INDEX_5         5
#define DRV_TMR_INDEX_6         6
#define DRV_TMR_INDEX_7         7


// *****************************************************************************
/* Timer Driver Module Index Count

  Summary:
    Number of valid Timer driver indices.

  Description:
    This constant identifies Timer driver index definitions.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is device-specific.
*/

#define DRV_TMR_INDEX_COUNT     TMR_NUMBER_OF_MODULES


// *****************************************************************************
/* Timer Synchronous Modes

  Summary:
    Identifies the synchronous modes for the Timer driver.

  Description:
    This data type identifies the synchronous modes for the Timer driver.

  Remarks:
    Not all modes are available on all devices.
*/

typedef enum
{
    /* Synchronous Internal Clock Counter */
    DRV_TMR_SYNC_MODE_SYNCHRONOUS_INTERNAL
            /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/,

    /* Synchronous Internal Gated Counter */
    DRV_TMR_SYNC_MODE_SYNCHRONOUS_INTERNAL_GATED
            /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/,

    /* Synchronous External Clock Counter */
    DRV_TMR_SYNC_MODE_SYNCHRONOUS_EXTERNAL_WITHOUT_CLKSYNC
            /*DOM-IGNORE-BEGIN*/ = 2 /*DOM-IGNORE-END*/,

    /* Synchronous External Clock Counter */
    DRV_TMR_SYNC_MODE_SYNCHRONOUS_EXTERNAL_WITH_CLKSYNC
            /*DOM-IGNORE-BEGIN*/ = 3 /*DOM-IGNORE-END*/,

    /* Asynchronous External Clock Counter */
    DRV_TMR_SYNC_MODE_ASYNCHRONOUS_EXTERNAL_WITHOUT_CLKSYNC
            /*DOM-IGNORE-BEGIN*/ = 4 /*DOM-IGNORE-END*/,

    /* Asynchronous External Clock Counter */
    DRV_TMR_SYNC_MODE_ASYNCHRONOUS_EXTERNAL_WITH_CLKSYNC
            /*DOM-IGNORE-BEGIN*/ = 5 /*DOM-IGNORE-END*/,

    /* Timer Idle Mode */
    DRV_TMR_SYNC_MODE_IDLE
            /*DOM-IGNORE-BEGIN*/ = 6 /*DOM-IGNORE-END*/

} DRV_TMR_SYNC_MODE;


// *****************************************************************************
/* Timer Driver Client Status

  Summary:
    Identifies the client-specific status of the Timer driver

  Description:
    This enumeration identifies the client-specific status of the Timer driver.

  Remarks:
    None.
*/

typedef enum
{
    /* Client in an invalid (or unopened) state */
    DRV_TMR_CLIENT_STATUS_INVALID
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_ERROR - 1 /*DOM-IGNORE-END*/,

    /* Unspecified error condition */
    DRV_TMR_CLIENT_STATUS_ERROR
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_ERROR - 0 /*DOM-IGNORE-END*/,

    /* An operation is currently in progress */
    DRV_TMR_CLIENT_STATUS_BUSY
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_BUSY      /*DOM-IGNORE-END*/,

    /* Up and running, no operations running */
    DRV_TMR_CLIENT_STATUS_READY
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_READY + 0 /*DOM-IGNORE-END*/,

    /* Timer stopped (not running), but ready to accept commands */
    DRV_TMR_CLIENT_STATUS_STOPPED
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_READY + 1 /*DOM-IGNORE-END*/,

    /* Timer started and running, processing transactions */
    DRV_TMR_CLIENT_STATUS_STARTED
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_READY + 2 /*DOM-IGNORE-END*/

} DRV_TMR_CLIENT_STATUS;


// *****************************************************************************
/* Timer Driver Initialize Data

  Summary:
    Defines the Timer driver initialization data.

  Description:
    This data type defines data required to initialize or reinitialize the Timer
    driver.

  Remarks:
    Not all initialization features are available on all devices.
*/

typedef struct
{
    /* System module initialization */
    SYS_MODULE_INIT                     moduleInit;

    /* Identifies timer hardware module (PLIB-level) ID */
    TMR_MODULE_ID                       tmrId;

    /* Clock Source select enumeration */
    TMR_CLOCK_SOURCE                    clockSource;

    /* Timer Period Value */
    uint32_t                            timerPeriod;

    /* Prescaler Selection from the processor enumeration */
    TMR_PRESCALE                        prescale;

    /* TMR Source Edge Selection */
    TMR_CLOCK_SOURCE_EDGE               sourceEdge;

    /* Post Scale Selection */
    uint8_t                             postscale;

    /* Timer Sync Mode */
    DRV_TMR_SYNC_MODE                   syncMode;

    /* Setting this bit to 'true' will combine two timer modules.
    Having this bit set we can achieve maximum delay. */
    bool                                combineTimers;

    /* Interrupt Source for TMR module */
    INT_SOURCE                          interruptSource;

} DRV_TMR_INIT;


// *****************************************************************************
/* Timer Driver Callback Function Pointer

  Summary:
    Pointer to a Timer driver callback function data type.

  Description:
    This data type defines a pointer to a Timer driver callback function.

  Remarks:
    Useful only when timer alarm callback support is enabled by defining the
    DRV_TMR_ALARM_ENABLE configuration option.
*/

typedef void (*DRV_TMR_CALLBACK) ( void );


// *****************************************************************************
/* Timer Alarm Type Enumeration

  Summary:
    Timer driver type enumeration.

  Description:
    This data type identifies the Timer alarm type.

  Remarks:
    Not all modes are available on all devices.
*/

typedef enum
{
    /* One Shot Alarm */
    DRV_TMR_ALARM_TYPE_ONE_SHOT  /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/,

    /* Periodic Alarm */
    DRV_TMR_ALARM_TYPE_PERIODIC  /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/

} DRV_TMR_ALARM_TYPE;


// *****************************************************************************
/* TMR Alarm configuration

  Summary:
    Timer module alarm configuration.

  Description:
    This data type controls the configuration of the alarm.

  Remarks:
    Not all modes are available on all devices.
*/

typedef struct
{
    /* Type of the alarm one shot or periodic */
    DRV_TMR_ALARM_TYPE      type;

    /* Alarm callback */
    DRV_TMR_CALLBACK        callback;

} DRV_TMR_ALARM_CONFIG;


// *****************************************************************************
// *****************************************************************************
// Section: Timer Driver Module Interface Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_TMR_Initialize ( const SYS_MODULE_INDEX drvIndex,
                                       const SYS_MODULE_INIT * const init )

  Summary:
    Initializes the Timer driver .

  Description:
    This function initializes the Timer driver, making it ready for clients to
    open and use it.

  Precondition:
    None.

  Parameters:
    drvIndex        - Index for the driver instance to be initialized

    init            - Pointer to a data structure containing any data necessary
                      to initialize the driver. 

  Returns:
    If successful, returns a valid handle to a driver object.  Otherwise, it
    returns SYS_MODULE_OBJ_INVALID. The returned object must be passed as
    argument to  DRV_TMR_Deinitialize, DRV_TMR_Tasks and DRV_TMR_Status functions.

  Example:
    <code>
    DRV_TMR_INIT    init;
    SYS_MODULE_OBJ  objectHandle;

    // Populate the timer initialization structure
    init.moduleInit.value = SYS_MODULE_POWER_RUN_FULL;
    init.tmrId            = TMR_ID_2;
    init.clockSource      = TMR_CLOCK_SOURCE_PERIPHERAL_CLOCK;
    init.prescale         = TMR_PRESCALE_TX_VALUE_256;
    init.syncMode         = DRV_TMR_SYNC_MODE_SYNCHRONOUS_INTERNAL;
    init.interruptSource  = INT_SOURCE_TIMER_2;
    init.timerPeriod      = 0xABCD;

    // Do something

    objectHandle = DRV_TMR_Initialize ( DRV_TMR_INDEX_0, (SYS_MODULE_INIT*)&init );
    if ( SYS_MODULE_OBJ_INVALID == objectHandle )
    {
        // Handle error
    }
    </code>

  Remarks:
    This function must be called before any other Timer function is called.

    This function should only be called once during system initialization
    unless DRV_TMR_Deinitialize is called to deinitialize the driver instance.

    This function will NEVER block for hardware access. If the operation requires
    time to allow the hardware to reinitialize, it will be reported by the
    DRV_TMR_Status operation. The system must use DRV_TMR_Status to find out
    when the driver is in the ready state.

    Build configuration options may be used to statically override options in the
    "init" structure and will take precedence over initialization data passed
    using this function.
*/

SYS_MODULE_OBJ DRV_TMR_Initialize( const SYS_MODULE_INDEX drvIndex,
                                   const SYS_MODULE_INIT * const init );


// *****************************************************************************
/* Function:
    void DRV_TMR_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the Timer driver.

  Description:
    Deinitializes the specified instance of the Timer driver, disabling
    its operation (and any hardware). All internal data is invalidated.

  Precondition:
    The DRV_TMR_Initialize function must have been called before calling this
    function and a valid SYS_MODULE_OBJ must have been returned.

  Parameters:
    object          - Driver object handle, returned from DRV_TMR_Initialize

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     //  Returned from DRV_TMR_Initialize
    SYS_STATUS          status;

    DRV_TMR_Deinitialize ( object );

    status = DRV_TMR_Status ( object );
 
    if ( SYS_MODULE_UNINITIALIZED == status )
    {
        // Check again later if you need to know
        // when the driver is deinitialized.
    }
    </code>

  Remarks:
    Once the Initialize operation has been called, the Deinitialize operation
    must be called before the Initialize operation can be called again.

    This function will NEVER block waiting for hardware. If the operation
    requires time to allow the hardware to complete, this will be reported by
    the DRV_TMR_Status operation.  The system has to use DRV_TMR_Status to find
    out when the module is in the ready state.
*/

void DRV_TMR_Deinitialize( SYS_MODULE_OBJ object );


// *****************************************************************************
/* Function:
    SYS_STATUS DRV_TMR_Status( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the Timer driver.

  Description:
    This function provides the current status of the Timer driver.

  Precondition:
    The DRV_TMR_Initialize function must have been called before calling this
    function.

  Parameters:
    object          - Driver object handle, returned from DRV_TMR_Initialize

  Returns:
    SYS_STATUS_READY          - Indicates that the driver is initialized and ready
    							for operation

                                Note: Any value greater than SYS_STATUS_READY is
                                also a normal running state in which the driver
                                is ready to accept new operations.

    SYS_STATUS_BUSY           - Indicates that the driver is busy with a
                                previous system level operation and cannot start
                                another

    SYS_STATUS_ERROR          - Indicates that the driver is in an error state

                                Note: Any value less than SYS_STATUS_ERROR is
                                also an error state.

    SYS_MODULE_UNINITIALIZED  - Indicates that the driver has been deinitialized

                                Note: This value is less than SYS_STATUS_ERROR.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_TMR_Initialize
    SYS_STATUS          tmrStatus;

    tmrStatus = DRV_TMR_Status(object);
    else if (SYS_STATUS_ERROR >= tmrStatus)
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

    This function will NEVER block waiting for hardware.

    If the Status operation returns an error value, the error may be cleared by
    calling the reinitialize operation.  If that fails, the deinitialize
    operation will need to be called, followed by the initialize operation to
    return to normal operations.
*/

SYS_STATUS DRV_TMR_Status( SYS_MODULE_OBJ object );


// *****************************************************************************
/* Function:
    void DRV_TMR_Tasks( SYS_MODULE_OBJ object )

  Summary:
    Maintains the driver's state machine and implements its ISR.

  Description:
    This function is used to maintain the driver's internal state machine and
    implement its ISR for interrupt-driven implementations.

  Precondition:
    The DRV_TMR_Initialize function must have been called for the specified Timer
    driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_TMR_Initialize)

  Returns:
    None

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_TMR_Initialize

    while (true)
    {
        DRV_TMR_Tasks (object);

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

void DRV_TMR_Tasks( SYS_MODULE_OBJ object );


// *****************************************************************************
// *****************************************************************************
// Section: TMR Driver Client Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_TMR_Open ( const SYS_MODULE_INDEX drvIndex,
                             const DRV_IO_INTENT    intent )

  Summary:
    Opens the specified Timer driver instance and returns a handle to it.

  Description:
    This function opens the specified Timer driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver.

  Precondition:
    The DRV_TMR_Initialize function must have been called before calling this
    function.

  Parameters:
    drvIndex    - Identifier for the object instance to be opened

    intent      - Zero or more of the values from the enumeration
                  DRV_IO_INTENT ORed together to indicate the intended use
                  of the driver

  Returns:
    If successful, the function returns a valid open instance handle (a number
    identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID.

  Example:
    <code>
    DRV_HANDLE  handle;

    handle = DRV_TMR_Open ( DRV_TMR_INDEX_0, DRV_IO_INTENT_EXCLUSIVE );

    if ( DRV_HANDLE_INVALID == handle )
    {
        // Unable to open the driver
    }
    </code>

  Remarks:
    The handle returned is valid until the DRV_TMR_Close function is called.

    This function will NEVER block waiting for hardware.

    If the DRV_IO_INTENT_BLOCKING is requested and the driver was built
    appropriately to support blocking behavior, other client-level
    operations may block waiting on hardware until they are complete.

    If DRV_IO_INTENT_NON_BLOCKING is requested the driver client can call the
    DRV_TMR_ClientStatus operation to find out when the module is in the ready
    state.

    If the requested intent flags are not supported, the function will return
    DRV_HANDLE_INVALID.

    The Timer driver does not support DRV_IO_INTENT_SHARED.  It must be opened
    with DRV_IO_INTENT_EXCLUSIVE in the "intent" parameter.
*/

DRV_HANDLE DRV_TMR_Open ( const SYS_MODULE_INDEX drvIndex, const DRV_IO_INTENT intent );


// *****************************************************************************
/* Function:
    void DRV_TMR_Close ( DRV_HANDLE handle )

  Summary:
    Closes an opened instance of the Timer driver

  Description:
    This function closes an opened instance of the Timer driver, invalidating the
    handle.

  Preconditions:
    The DRV_TMR_Initialize function must have been called for the specified
    Timer driver instance.

    DRV_TMR_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TMR_Open

    DRV_TMR_Close ( handle );
    </code>

  Remarks:
    After calling this function, the handle passed in "handle" must not be used
    with any of the remaining driver functions.  A new handle must be obtained by
    calling DRV_TMR_Open before the caller may use the driver again.

    If DRV_IO_INTENT_BLOCKING was requested and the driver was built
    appropriately to support blocking behavior, the call may block until the
    operation is complete.

    If DRV_IO_INTENT_NON_BLOCKING request the driver client can call the
    DRV_TMR_Status operation to find out when the module is in the ready state
    (the handle is no longer valid).

    Note: Usually there is no need for the driver client to verify that the Close
    operation has completed.
*/

void DRV_TMR_Close ( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
    DRV_TMR_CLIENT_STATUS DRV_TMR_ClientStatus ( DRV_HANDLE handle )

  Summary:
    Gets current client-specific status of the Timer driver.

  Description:
    This function gets the client-specific status of the Timer driver associated
    with the specified handle.

  Preconditions:
    The DRV_TMR_Initialize function must have been called.

    DRV_TMR_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    A DRV_TMR_CLIENT_STATUS value describing the current status of the driver

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TMR_Open
    DRV_TMR_CLIENT_STATUS tmrClientStatus;

    tmrClientStatus = DRV_TMR_ClientStatus ( tmrHandle );

    if ( DRV_TMR_CLIENT_STATUS_ERROR >= tmrClientStatus )
    {
        // Handle the error
    }
    </code>

  Remarks:
    This function will not block for hardware access and will immediately return
    the current status.
*/

DRV_TMR_CLIENT_STATUS DRV_TMR_ClientStatus( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
   void DRV_TMR_Period16BitSet( DRV_HANDLE handle, uint16_t value )

  Summary:
    Updates the 16-bit Timer's period.

  Description:
    This function updates the 16-bit Timer's period.

  Preconditions:
    The DRV_TMR_Initialize function must have been called.

    DRV_TMR_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

    value        - 16-bit Period value

  Returns:
    None.

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TMR_Open

    DRV_TMR_Period16BitSet(handle, 0x1000);
    </code>

  Remarks:
    None.
*/

void DRV_TMR_Period16BitSet( DRV_HANDLE handle, uint16_t value );


// *****************************************************************************
/* Function:
   void DRV_TMR_Period32BitSet( DRV_HANDLE handle, uint32_t value )

  Summary:
    Updates the 32-bit Timer's period.

  Description:
    This function updates the 32-bit Timer's period.

  Preconditions:
    The DRV_TMR_Initialize unction must have been called.

    DRV_TMR_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

    value        - 32-bit Period value

  Returns:
    None.

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TMR_Open

    DRV_TMR_Period32BitSet(handle, 0xFFFFFFFF);
    </code>

  Remarks:
    None.
*/

void DRV_TMR_Period32BitSet( DRV_HANDLE handle, uint32_t value );


// *****************************************************************************
/* Function:
   uint16_t DRV_TMR_Period16BitGet( DRV_HANDLE handle )

  Summary:
    Provides the 16-bit Timer's period

  Description:
    This function provides the 16-bit tTmer's period.

  Precondition:
    The DRV_TMR_Initialize function must have been called.

    DRV_TMR_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    16-bit timer period value

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TMR_Open
    uint16_t value;
    value = DRV_TMR_Period16BitGet(handle);
    </code>

  Remarks:
    None.
*/

uint16_t DRV_TMR_Period16BitGet( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
   uint32_t DRV_TMR_Period32BitGet( DRV_HANDLE handle )

  Summary:
    Provides the 32-bit Timer's period

  Description:
    This function provides the 32-bit Timer's period.

  Preconditions:
    The DRV_TMR_Initialize function must have been called.

    DRV_TMR_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    32-bit Timer period value.

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TMR_Open
    uint32_t value;
    value = DRV_TMR_Period32BitGet(handle);
    </code>

  Remarks:
    None.
*/

uint32_t DRV_TMR_Period32BitGet( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
   void DRV_TMR_Counter16BitSet( DRV_HANDLE handle, uint16_t value )

  Summary:
    Updates the 16-bit Timer's initial count value.

  Description:
    This function updates the 16-bit Timer's initial count value.

  Preconditions:
    The DRV_TMR_Initialize function must have been called.

    DRV_TMR_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

    value        - 16-bit Counter value

  Returns:
    None.

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TMR_Open

    DRV_TMR_Counter16BitSet(handle, 0x1000);
    </code>

  Remarks:
    None.
*/

void DRV_TMR_Counter16BitSet( DRV_HANDLE handle, uint16_t value );


// *****************************************************************************
/* Function:
   void DRV_TMR_Counter32BitSet ( DRV_HANDLE handle, uint32_t value )

  Summary:
    Updates the 32-bit Timer's initial count value.

  Description:
    This function updates the 32-bit Timer's initial count value.

  Precondition:
    The DRV_TMR_Initialize function must have been called.

    DRV_TMR_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

    value        - 32-bit Counter value

  Returns:
    None.

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TMR_Open

    DRV_TMR_Counter32BitSet(handle, 0xFFFFFFFF);
    </code>

  Remarks:
    None.
*/

void DRV_TMR_Counter32BitSet( DRV_HANDLE handle, uint32_t value );


// *****************************************************************************
/* Function:
   uint16_t DRV_TMR_Counter16BitGet ( DRV_HANDLE handle )

  Summary:
    Provides the 16-bit Timer's current counter value.

  Description:
    This function provides the 16-bit Timer's current counter value.

  Preconditions:
    The DRV_TMR_Initialize function must have been called.

    DRV_TMR_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    16-bit Timer's current counter value

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TMR_Open
    uint16_t value;
    value = DRV_TMR_Counter16BitGet ( handle );
    </code>

  Remarks:
    None.
*/

uint16_t DRV_TMR_Counter16BitGet ( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
   uint32_t DRV_TMR_Counter32BitGet( DRV_HANDLE handle )

  Summary:
    Provides the 32-bit Timer's current counter value.

  Description:
    This function provides the 32-bit Timer's current counter value.

  Preconditions:
    The DRV_TMR_Initialize function must have been called.

    DRV_TMR_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    32-bit Timer's current counter value.

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TMR_Open
    uint32_t value;
    value = DRV_TMR_Counter32BitGet(handle);
    </code>

  Remarks:
    None.
*/

uint32_t DRV_TMR_Counter32BitGet ( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
   void DRV_TMR_Start ( DRV_HANDLE handle )

  Summary:
    Starts the Timer counting.

  Description:
    This function starts the Timer counting.

  Preconditions:
    The DRV_TMR_Initialize function must have been called.

    DRV_TMR_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None.

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TMR_Open

    DRV_TMR_Start ( handle );
    </code>

  Remarks:
    None.
*/

void DRV_TMR_Start ( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
   void DRV_TMR_Stop ( DRV_HANDLE handle )

  Summary:
    Stops the Timer from counting.

  Description:
    This function stops the running Timer from counting.

  Preconditions:
    The DRV_TMR_Initialize function must have been called.

    DRV_TMR_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None.

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TMR_Open

    DRV_TMR_Stop ( handle );
    </code>

  Remarks:
    None.
*/

void DRV_TMR_Stop ( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
   uint32_t DRV_TMR_OperatingFrequencyGet( DRV_HANDLE handle )

  Summary:
    Provides the Timer operating frequency.

  Description:
    This function provides the Timer operating frequency.

  Preconditions:
    The DRV_TMR_Initialize function must have been called.

    DRV_TMR_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's
                  open routine

  Returns:
    32-bit value corresponding to the running frequency.

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TMR_Open
    uint32_t   clkFreq;

    clkFreq = DRV_TMR_OperatingFrequencyGet(handle);
    </code>

  Remarks:
    On most processors, the Timer's base frequency is the same as the peripheral
    bus clock.
*/

uint32_t DRV_TMR_OperatingFrequencyGet( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
   uint32_t DRV_TMR_TickFrequencyGet( DRV_HANDLE handle )

  Summary:
    Provides the Timer's counter "tick" frequency.

  Description:
    This function provides the Timer's counter "tick" frequency.

  Preconditions:
    The DRV_TMR_Initialize function must have been called.

    DRV_TMR_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - A valid handle, returned from the DRV_TMR_Open

  Returns:
    Clock frequency value in cycles per second (Hertz).

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TMR_Open
    uint32_t   clkFreq;

    clkFreq = DRV_TMR_TickFrequencyGet(handle);
    </code>

  Remarks:
    None.
*/

uint32_t DRV_TMR_TickFrequencyGet( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
   bool DRV_TMR_ElapsedStatusGetAndClear( DRV_HANDLE handle )

  Summary:
    Provides and clears the Timer's elapse count.

  Description:
    This function provides the Timer's elapse count and resets it to zero (0).

  Preconditions:
    The DRV_TMR_Initialize function must have been called.

    DRV_TMR_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - A valid handle, returned from the DRV_TMR_Open

  Returns:
    - true        - The Timer has elapsed
    - false       - The Timer has not yet elapsed

  Example:
    <code>
    DRV_HANDLE      handle;  // Returned from DRV_TMR_Open
    bool            elapseStatus;

    elapseStatus = DRV_TMR_ElapsedStatusGetAndClear(handle);
    if (elapseStatus > 0)
    {
        // The counter period has elapsed.
    }
    </code>

  Remarks:
    None.
*/

bool DRV_TMR_ElapsedStatusGetAndClear( DRV_HANDLE handle );


// *****************************************************************************
// *****************************************************************************
// Section: Timer Driver Optional Alarm Functions
// *****************************************************************************
// *****************************************************************************
/* These functions provide interface to the alarm functionality of the timer
   driver, it is an optional feature and is available if enabled by defining the
   DRV_TMR_ALARM_ENABLE configuration macro.
*/

// *****************************************************************************
/* Function:
    void DRV_TMR_AlarmSet( DRV_HANDLE handle, const DRV_TMR_ALARM_CONFIG * config)

  Summary:
    Initializes an alarm.

  Description:
    This function initializes an alarm, allowing the client to receive a
    callback from the driver when the counter period elapses.  Alarms can be
    one-shot or periodic.

  Preconditions:
    The DRV_TMR_Initialize function must have been called.

    DRV_TMR_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - A valid handle, returned from DRV_TMR_Open

    config      - Pointer to the alarm configuration structure containing any data
                  necessary to initialize timer alarms

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE              handle;  // Returned from DRV_TMR_Open
    DRV_TMR_ALARM_CONFIG    alarmConfig;

    alarmConfig.type     = DRV_TMR_ALARM_TYPE_PERIODIC; // Periodic Alarm
    alarmConfig.callback = &callBackFunction;           // callback registered

    DRV_TMR_AlarmSet(handle, config);
    </code>

  Remarks:
    None.
*/

void DRV_TMR_AlarmSet( DRV_HANDLE handle, const DRV_TMR_ALARM_CONFIG * config );


// *****************************************************************************
/* Function:
    unsigned int DRV_TMR_AlarmCountGet( DRV_HANDLE handle )

  Summary:
    Provides the number of times the alarm has elapsed since the last time it was
    cleared.

  Description:
    This function provides the number of times the alarm is generated since the last
    time it was cleared.

  Preconditions:
    The DRV_TMR_Initialize function must have been called.

    DRV_TMR_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - A valid handle, returned from DRV_TMR_Open

  Returns:
    Alarm count value.

  Example:
    <code>
    DRV_HANDLE   handle;        // Returned from DRV_TMR_Open
    unsigned int alarmCount;

    alarmCount = DRV_TMR_AlarmCountGet (handle);
    </code>

  Remarks:
    None.
*/

unsigned int DRV_TMR_AlarmCountGet( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
    void DRV_TMR_AlarmCountClear( DRV_HANDLE handle )

  Summary:
    Clears the alarm elapse counter.

  Description:
    This function clears the alarm elapse counter.

  Preconditions:
    The DRV_TMR_Initialize function must have been called.

    DRV_TMR_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - A valid handle, returned from DRV_TMR_Open

  Returns:
    None.

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TMR_Open

    DRV_TMR_AlarmCountClear(handle);
    </code>

  Remarks:
    None.
*/

void DRV_TMR_AlarmCountClear( DRV_HANDLE handle );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Functions - Version Information
// *****************************************************************************
// *****************************************************************************
/* These functions return the version information of the Timer driver */


// *****************************************************************************
/* Function:
    unsigned int DRV_TMR_VersionGet( const SYS_MODULE_INDEX drvIndex )

  Summary:
    Gets the Timer driver version in numerical format.

  Description:
    This function gets the Timer driver version. The version is encoded as
    major * 10000 + minor * 100 + patch. The stringed version can be obtained
    using DRV_TMR_VersionStrGet()

  Precondition:
    None.

  Parameters:
    drvIndex    - Identifier for the object instance to get the version for

  Returns:
    Current driver version in numerical format.

  Example:
    <code>
    unsigned int version;

    version = DRV_TMR_VersionGet( DRV_TMR_INDEX_0 );

    if(version < 110200)
    {
        // Do Something
    }
    </code>

  Remarks:
    None.
*/

unsigned int DRV_TMR_VersionGet( const SYS_MODULE_INDEX drvIndex );


// *****************************************************************************
/* Function:
    char * DRV_TMR_VersionStrGet( const SYS_MODULE_INDEX drvIndex )

  Summary:
    Gets the Timer driver version in string format.

  Description:
    This function gets the Timer driver version. The version is returned as
    major.minor.path[type], where type is optional. The numerical version can
    be obtained using DRV_TMR_VersionGet.

  Precondition:
    None.

  Parameters:
    drvIndex    - Identifier for the object instance to get the version for

  Returns:
    Current Timer driver version in the string format.

  Example:
    <code>
    char *version;

    version = DRV_TMR_VersionStrGet( DRV_TMR_INDEX_0 );

    printf("%s", version);
    </code>

  Remarks:
    None.
*/

char * DRV_TMR_VersionStrGet( const SYS_MODULE_INDEX drvIndex );


// *****************************************************************************
// *****************************************************************************
// Section: Included Files (continued)
// *****************************************************************************
// *****************************************************************************
/*  The file included below maps the interface definitions above to appropriate
    static implementations, depending on build mode.
*/

#include "driver/tmr/drv_tmr_mapping.h"


#endif // #ifndef _DRV_TMR_H

/*******************************************************************************
 End of File
*/

