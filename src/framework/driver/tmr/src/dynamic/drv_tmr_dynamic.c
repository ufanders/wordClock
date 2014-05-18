/*******************************************************************************
  Timer Device Driver Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_tmr_dynamic.c

  Summary:
    Timer device driver dynamic multi-client implementation.

  Description:
    The Timer device driver provides a simple interface to manage the Tmier modules
    on Microchip microcontrollers.  This file implements the core interface
    routines for the Tmier driver.
    While building the driver from source, ALWAYS use this file in the build.
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


// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>
#include "driver/tmr/src/drv_tmr_local.h"
#include "system/clk/sys_clk.h"
#include "osal/osal.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Variables
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Driver Hardware instance objects.

  Summary:
    Defines the hardware instances objects that are available on the device.

  Description:
    This data type defines the hardware instance objects that are available on
    the device, so as to capture the hardware state of the instance.

  Remarks:
    Not all modes are available on all devices.
*/

DRV_TMR_OBJ             gDrvTMRObj[DRV_TMR_INSTANCES_NUMBER] ;


// *****************************************************************************
/* Driver client instance objects

  Summary:
    Defines the client instance objects that are available on the device.

  Description:
    This data type defines the client instance objects that are available on
    the device, so as to capture the Client state of the instance.

  Remarks:
    None
*/

DRV_TMR_CLIENT_OBJ      gDrvTMRClientObj[DRV_TMR_CLIENTS_NUMBER] ;


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Macros
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro:
    _DRV_TMR_ExternalClockSyncEnable(index)

  Summary:
    Enable external clock synchronization.

  Description:
    This macro enables the external clock synchronization.

  Remarks:
    None.
*/

#define _DRV_TMR_ExternalClockSyncEnable(index)     PLIB_TMR_ClockSourceExternalSyncEnable(index)


// *****************************************************************************
/* Macro:
    _DRV_TMR_ExternalClockSyncDisable(index)

  Summary:
    Disables external clock synchronization.

  Description:
    This macro disables the external clock synchronization.

  Remarks:
    None.
*/

#define _DRV_TMR_ExternalClockSyncDisable(index)    PLIB_TMR_ClockSourceExternalSyncDisable(index)


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************


/******************************************************************************
  Function:
    static void _DRV_TMR_SyncModeControl( TMR_MODULE_ID     plibId,
                                          DRV_TMR_SYNC_MODE newMode )

  Summary:
    Sets the new sync mode.

  Description:
    This internal Timer driver function sets the new sync mode in the hardware.

  Remarks:
    None.
*/

static void _DRV_TMR_SyncModeControl( const TMR_MODULE_ID   plibId,
                                      DRV_TMR_SYNC_MODE     newMode )
{
    if ( DRV_TMR_SYNC_MODE_SYNCHRONOUS_INTERNAL == newMode )
    {
        /* Clock Source Internal */
    }
    else if ( DRV_TMR_SYNC_MODE_SYNCHRONOUS_INTERNAL_GATED == newMode )
    {
        /* Clock Source Internal */

        /* Gated Operation Control */
        if( PLIB_TMR_ExistsGatedTimeAccumulation(  plibId  ) )
        {
            PLIB_TMR_GateEnable( plibId );
        }
    }
    else if ( DRV_TMR_SYNC_MODE_SYNCHRONOUS_EXTERNAL_WITHOUT_CLKSYNC == newMode )
    {
        /* Clock Source External */

        /* External Clock Syncronization */
        _DRV_TMR_ExternalClockSyncDisable( plibId );
    }
    else if ( DRV_TMR_SYNC_MODE_SYNCHRONOUS_EXTERNAL_WITH_CLKSYNC == newMode )
    {
        /* Clock Source External */

        /* External Clock Syncronization */
        _DRV_TMR_ExternalClockSyncEnable( plibId );
    }
    else if ( DRV_TMR_SYNC_MODE_ASYNCHRONOUS_EXTERNAL_WITHOUT_CLKSYNC == newMode )
    {
        /* External Clock Syncronization */
        _DRV_TMR_SyncModeAsyncExtWithoutClockSync( plibId );
    }
    else /* ( DRV_TMR_SYNC_MODE_ASYNCHRONOUS_EXTERNAL_WITH_CLKSYNC == newMode ) */
    {
        /* External Clock Syncronization */
        _DRV_TMR_SyncModeAsyncExtWithClockSync( plibId );
    }

} /* _DRV_TMR_SyncModeControl */


/******************************************************************************
  Function:
    static void _DRV_TMR_SetupHardware ( const TMR_MODULE_ID    plibId,
                                         DRV_TMR_OBJ_HANDLE     dObj,
                                         DRV_TMR_INIT           * tmrInit )

  Summary:
    Sets up the hardware from the initialzation structure.

  Description:
    This function sets up the hardware from the initialzation structure.

  Remarks:
    Called.
*/

static void _DRV_TMR_SetupHardware( const TMR_MODULE_ID plibId,
                                    DRV_TMR_OBJ_HANDLE  dObj,
                                    DRV_TMR_INIT        * tmrInit )
{
    TMR_MODULE_ID plibNextId = -1; // avoid compiler warning
    uint16_t tmr16bitPeriod;
    
    /* Initialize the Interrupt Source */
    gDrvTMRObj[dObj].interruptSource = tmrInit->interruptSource;

    /* If we have to reload */
    gDrvTMRObj[dObj].timerPeriod = tmrInit->timerPeriod;

    /* Enable the configured 8bit or 16bit or 32bit timer mode. */
    if( tmrInit->combineTimers == false )
    {
        PLIB_TMR_Mode16BitEnable( plibId );
        PLIB_TMR_Period16BitSet( plibId, tmrInit->timerPeriod );
    }
    else 
    {
        PLIB_TMR_Mode32BitEnable( plibId );
        
        tmr16bitPeriod = ( tmrInit->timerPeriod & 0x0000FFFF );

        /* Update the Timer data value */
        PLIB_TMR_Period16BitSet( plibId, tmr16bitPeriod );
        
        if ( plibId == TMR_ID_2 )
        {
            plibNextId = TMR_ID_3;
        }
        else if ( plibId == TMR_ID_4 )
        {
            plibNextId = TMR_ID_5;
        }
        tmr16bitPeriod = ( ( tmrInit->timerPeriod & 0xFFFF0000 ) >> 16 );
        
        /* Update the Timer data value */
        PLIB_TMR_Period16BitSet( plibNextId, tmr16bitPeriod );
    }

    /* Timer Mode Configuration */
    /* Clock Source Selection - Internal or External */
    if( PLIB_TMR_ExistsClockSource( plibId ) )
    {
        PLIB_TMR_ClockSourceSelect( plibId, tmrInit->clockSource );
    }

    _DRV_TMR_SyncModeControl( plibId, tmrInit->syncMode );

    /* Update the operating mode of the Timer initialized */
    gDrvTMRObj[dObj].syncMode = tmrInit->syncMode;


    /* Prescaler selection */
    if( PLIB_TMR_ExistsPrescale( plibId ) )
    {
        PLIB_TMR_PrescaleSelect( plibId , tmrInit->prescale );
    }

    gDrvTMRObj[dObj].prescale = tmrInit->prescale;

    /* Asynchronous Write Control */
    if( PLIB_TMR_ExistsCounterAsyncWriteControl( plibId ) )
    {
        if( DRV_TMR_ASYNC_WRITE_ENABLE == true )
        {
            PLIB_TMR_CounterAsyncWriteEnable( plibId );
        }
        else if( DRV_TMR_ASYNC_WRITE_ENABLE == false )
        {
            PLIB_TMR_CounterAsyncWriteDisable( plibId );
        }
    }
   
} /* _DRV_TMR_SetupHardware */


// *****************************************************************************
// *****************************************************************************
// Section: Driver Interface Function Definitions
// *****************************************************************************
// *****************************************************************************

/******************************************************************************
  Function:
    SYS_MODULE_OBJ DRV_TMR_Initialize( const SYS_MODULE_INDEX index,
                                       const SYS_MODULE_INIT * const init )

  Summary:
    Initializes hardware and data for the specified instance of the Timer module.

  Description:
    This function initializes hardware for the instance of the Timer module,
    using the specified hardware initialization data.  It also initializes all
    necessary internal data.

  Parameters:
    index           - Identifies the driver instance to be initialized

    init            - Pointer to the data structure containing all data
                      necessary to initialize the hardware. This pointer may
                      be null if no data is required and static initialization
                      values are to be used.

  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID.
*/

SYS_MODULE_OBJ DRV_TMR_Initialize( const SYS_MODULE_INDEX drvIndex,
                                   const SYS_MODULE_INIT  * const init )
{
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;
    DRV_TMR_INIT * tmrInit = NULL;
	
    /* Validate the driver index */
    if ( drvIndex >= DRV_TMR_INDEX_COUNT )
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Assign to the local pointer the init data passed */
    tmrInit = ( DRV_TMR_INIT * ) init;

    /* Allocate the driver object and set the operation flag to be in use */
    dObj = drvIndex;

    /* Check that the object is valid */
    SYS_ASSERT( dObj < DRV_TMR_INSTANCES_NUMBER, "Hardware Object is invalid" );

    /* Object is valid, set it in use */
    gDrvTMRObj[dObj].inUse = true;

    /* Update the Timer Module Index */
    gDrvTMRObj[dObj].tmrId = tmrInit->tmrId;

    /* Setup the Hardware */
    _DRV_TMR_SetupHardware( tmrInit->tmrId, dObj, tmrInit );

    /* Interrupt flag cleared on the safer side */
    _DRV_TMR_InterruptSourceClear( gDrvTMRObj[dObj].interruptSource );

    /* Enable the interrupt source in case of interrupt mode */
    _DRV_TMR_InterruptSourceEnable( gDrvTMRObj[dObj].interruptSource );

    /* Set the current driver state */
    gDrvTMRObj[dObj].status = SYS_STATUS_READY;

    /* Create the required mutexes. */
    //OSAL_ASSERT( (OSAL_MUTEX_Create(&gDrvTMRObj[dObj].clientObjectMutex) == OSAL_RESULT_TRUE),
    //             "Unable to create client object mutex" );

    /* Return the driver handle */
    return (SYS_MODULE_OBJ)dObj;

} /* DRV_TMR_Initialize */


//******************************************************************************
/* Function:
    void DRV_TMR_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specific module instance of the TMR module

  Description:
    Deinitializes the specific module instancedisabling its operation (and any
    hardware for driver modules).  Resets all the internal data structures and
    fields for the specified instance to the default settings.

  Parameters:
    object          - Identifies the Driver Object returned by the Initialize
                      interface

  Returns:
    None
*/

void DRV_TMR_Deinitialize( SYS_MODULE_OBJ object )
{
    DRV_TMR_OBJ_HANDLE dObj = ( DRV_TMR_OBJ_HANDLE ) object;

	gDrvTMRObj[dObj].inUse = false;

    /* Set the Device Status */
    gDrvTMRObj[dObj].status  = SYS_STATUS_UNINITIALIZED;


} /* DRV_TMR_Deinitialize */



/******************************************************************************
  Function:
    SYS_STATUS DRV_TMR_Status( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the hardware instance of the Timer module.

  Description:
    This function Provides the current status of the hardware instance of the
    Timer module.

  Parameters:
    object          - Identifies the Driver Object returned by the Initialize
                      interface

  Returns:
    SYS_STATUS_READY  -  Indicates that any previous module operation for the
                         specified module has completed

    SYS_STATUS_BUSY   -  Indicates that a previous module operation for the
                         specified module has not yet completed

    SYS_STATUS_ERROR  -  Indicates that the specified module is in an error state
*/

SYS_STATUS DRV_TMR_Status( SYS_MODULE_OBJ object )
{
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) object;

    /* Check for the valid driver object passed */
    SYS_ASSERT( dObj < DRV_TMR_INSTANCES_NUMBER, "Driver Object is invalid" );

    /* Return the status associated with the driver handle */
    return( gDrvTMRObj[dObj].status );

} /* DRV_TMR_Status */


/******************************************************************************
  Function:
    void DRV_TMR_Tasks( SYS_MODULE_OBJ object)

  Summary:
    Used to maintain the driver's state machine and implement its ISR.

  Description:
    This function is used to maintain the driver's internal state machine and
    implement its ISR for interrupt-driven implementations.

  Parameters:
    object          - Identifies the driver object returned by the Initialize
                      interface

  Returns:
    None.
*/

void DRV_TMR_Tasks( SYS_MODULE_OBJ object )
{
    DRV_TMR_OBJ_HANDLE dObj = ( DRV_TMR_OBJ_HANDLE ) object;

    /* Check for the valid driver object passed */
    SYS_ASSERT( dObj < DRV_TMR_INSTANCES_NUMBER, "Driver Object is invalid" );

    /* Check if the Timer Interrupt/Status is set */
    if ( true == _DRV_TMR_InterruptSourceStatusGet( gDrvTMRObj[dObj].interruptSource ) )
    {
        /* Update counter values in case of a overflow based Timer */
        _DRV_TMR_PeriodSet( gDrvTMRObj[dObj].tmrId, gDrvTMRObj[dObj].timerPeriod );

        /* Indicate to the application that the Timer has elapsed */
        gDrvTMRObj[dObj].elapseStatus = true;

        /* Check if the alarm is active */
        if ( gDrvTMRObj[dObj].alarmInUse == true )
        {
            /* increment the alarm */

            gDrvTMRObj[dObj].alarmCount++;

            /* callback the application routine */
            if( gDrvTMRObj[dObj].alarmCallback != NULL )
            {
                gDrvTMRObj[dObj].alarmCallback();
            }

            /* Alarm in one shot mode */
            if ( gDrvTMRObj[dObj].alarmPeriodic != true )
            {
                /* first shot is done, deactivate it */
                gDrvTMRObj[dObj].alarmInUse = false;
                gDrvTMRObj[dObj].alarmCount = 0;
                gDrvTMRObj[dObj].alarmCallback = NULL;
            }
        }

        /* Clear Timer Interrupt/Status Flag */
        _DRV_TMR_InterruptSourceClear( gDrvTMRObj[dObj].interruptSource );
    }

} /* DRV_TMR_Tasks */


/******************************************************************************
  Function:
    DRV_HANDLE DRV_TMR_Open( const SYS_MODULE_INDEX index,
                             const DRV_IO_INTENT intent )

  Summary:
    Opens the specific module instance and returns a handle.

  Description:
    This function opens a driver for use by any client module and provides a
    handle that must be provided to any of the other driver operations to
    identify the caller and the instance of the driver/hardware module.

  Parameters:
    index           - Identifier for the instance to be initialized
    ioIntent        - Possible values from DRV_IO_INTENT

  Returns:
    If successful, the routine returns a valid open instance handle (a number
    identifying both the caller and the module instance).
    If an error occurs, the return value is DRV_HANDLE_INVALID.
*/

DRV_HANDLE DRV_TMR_Open ( const SYS_MODULE_INDEX   drvIndex,
                         const DRV_IO_INTENT      ioIntent )
{
    /* Allocate the client object and set the flag as in use */
    uint8_t i = 0;
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;

    /* Validate the driver index */
    if ( ( drvIndex >= DRV_TMR_INDEX_COUNT ) ||
         ( ioIntent != DRV_IO_INTENT_EXCLUSIVE ) )
    {
        return DRV_HANDLE_INVALID;
    }

    /* Setup client operations */
    for ( i = 0; i < DRV_TMR_CLIENTS_NUMBER ; i++ )
    {
        // Return the matching index associated the hardware instance.
        if ( gDrvTMRClientObj[i].inUse != true )
        {
            /* OSAL - Unlock Mutex */
            clientObj = i;

            gDrvTMRClientObj[clientObj].inUse = true;
            gDrvTMRClientObj[clientObj].driverObject = drvIndex;

            /* Update the Client Status */
            gDrvTMRClientObj[clientObj].status = DRV_TMR_CLIENT_STATUS_READY;

            /* Return the client object */
            return ( DRV_HANDLE ) clientObj;
        }
    }
    /* Returns the available client object */
    return NULL;

} /* DRV_TMR_Open */


/******************************************************************************
  Function:
    void DRV_TMR_Close ( DRV_Handle handle )

  Summary:
    Closes an opened instance of a driver.

  Description:
    This function closes an opened instance of a driver, invalidating the specified
    handle.

  Parameters:
    handle       - A valid open instance handle, returned from the driver's
                   open routine

  Returns:
    None.
*/

void DRV_TMR_Close ( DRV_HANDLE handle )
{
    /* Multi client variables are removed from single client builds. */
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    /* OSAL - lock Mutex */
    if (OSAL_MUTEX_Lock(gDrvTMRObj[dObj].clientObjectMutex, OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        /* Free the Client Instance */
        gDrvTMRClientObj[clientObj].inUse = false ;

        /* OSAL - unlock Mutex */
        OSAL_ASSERT( (OSAL_MUTEX_Unlock(gDrvTMRObj[dObj].clientObjectMutex) == OSAL_RESULT_TRUE),
                     "Unable to unlock client object mutex" );
    }

    /* Update the Client Status */
    gDrvTMRClientObj[clientObj].status = DRV_TMR_CLIENT_STATUS_INVALID;

} /* DRV_TMR_Close */


/******************************************************************************
  Function:
    DRV_TMR_CLIENT_STATUS DRV_TMR_ClientStatus ( DRV_HANDLE handle )

  Summary:
    Gets the status of the module instance associated with the handle.

  Description:
    This function gets the status of the module instance associated with the
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    DRV_TMR_CLIENT_STATUS value describing the current status of the driver.
*/

DRV_TMR_CLIENT_STATUS DRV_TMR_ClientStatus ( DRV_HANDLE handle )
{
    /* Multi client variables are removed from single client builds. */
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Return the client status associated with the handle passed */
    return( gDrvTMRClientObj[clientObj].status );

} /* DRV_TMR_ClientStatus */


/******************************************************************************
  Function:
   void DRV_TMR_Period16BitSet( DRV_HANDLE handle, uint16_t value )

  Summary:
    Updates the Timer's 16-bit value.

  Description:
    This function updates the Timer's 16-bit value.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's
                  open routine
    value       - 16-bit counter value

  Returns:
    None.
*/

void DRV_TMR_Period16BitSet ( DRV_HANDLE handle,
                             uint16_t   value )
{
    /* Multi client variables are removed from single client builds. */
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    /* Update the counter values */
    PLIB_TMR_Period16BitSet( gDrvTMRObj[dObj].tmrId, value );

    /* OSAL Mutex Lock */
    /* Client update with the new count value */
    gDrvTMRObj[dObj].timerPeriod = value;

    /* Reset the status information */
    gDrvTMRObj[dObj].elapseStatus = false;

    /* OSAL Mutex Unlock */

} /* DRV_TMR_Period16BitSet */


/******************************************************************************
  Function:
   void DRV_TMR_Period32BitSet( DRV_HANDLE handle, uint32_t value )

  Summary:
    Updates the Timer's 32-bit value.

  Description:
    This function updates the Timer's 32-bit value.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
    countValue   - 32-bit counter value

  Returns:
    None.
*/

void DRV_TMR_Period32BitSet ( DRV_HANDLE handle, uint32_t value )
{
    /* Multi client variables are removed from single client builds. */
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    /* Update the counter values */
    PLIB_TMR_Period32BitSet( gDrvTMRObj[dObj].tmrId, value );

    /* OSAL Mutex Lock */
    /* Client update with the new count value */
    gDrvTMRObj[dObj].timerPeriod = value;

    /* Reset the status information */
    gDrvTMRObj[dObj].elapseStatus = false;
    /* OSAL Mutex Unlock */

} /* DRV_TMR_Period32BitSet */


/******************************************************************************
  Function:
   uint16_t DRV_TMR_Period16BitGet( DRV_HANDLE handle )

  Summary:
    Provides the Timer's 16-bit period value.

  Description:
    This function provides the Timer's 16-bit period value.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    The Timer's 16-bit period value.
*/

uint16_t DRV_TMR_Period16BitGet ( DRV_HANDLE handle )
{
    /* Multi client variables are removed from single client builds. */
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;
    uint16_t periodVal = 0xFFFF;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    /* OSAL Mutex Lock */
	/* get the timer period value and return it */
	periodVal = PLIB_TMR_Period16BitGet( gDrvTMRObj[dObj].tmrId );

	/* OSAL Mutex Unlock */

    return( periodVal );

} /* DRV_TMR_Period16BitGet */


/******************************************************************************
  Function:
   uint32_t DRV_TMR_Period32BitGet ( DRV_HANDLE handle )

  Summary:
    Provides the Timer's 32-bit period value.

  Description:
    This function provides the Timer's 32-bit period value.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    The Timer's 32-bit period value.
*/

uint32_t DRV_TMR_Period32BitGet ( DRV_HANDLE handle )
{
    /* Multi client variables are removed from single client builds. */
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;
    uint32_t periodVal = 0xFFFFFFFF;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    /* OSAL Mutex Lock */
    /* get the timer period value and return it */
    periodVal = PLIB_TMR_Period32BitGet( gDrvTMRObj[dObj].tmrId );

    /* OSAL Mutex Unlock */

    return( periodVal );

} /* DRV_TMR_Period32BitGet */


/******************************************************************************
  Function:
   void DRV_TMR_Counter16BitSet( DRV_HANDLE handle, uint16_t value )

  Summary:
    Updates the Timer's 16-bit value.

  Description:
    This function updates the Timer's 16-bit value.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's
                  open routine
    value       - 16-bit Ccunter value

  Returns:
    None.
*/

void DRV_TMR_Counter16BitSet( DRV_HANDLE handle, uint16_t value )
{
    /* Multi client variables are removed from single client builds. */
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    /* Update the counter values */
    PLIB_TMR_Counter16BitSet( gDrvTMRObj[dObj].tmrId, value );

    /* Reset the status information */
    gDrvTMRObj[dObj].elapseStatus = false;

} /* DRV_TMR_Counter16BitSet */


/******************************************************************************
  Function:
   void DRV_TMR_Counter32BitSet( DRV_HANDLE handle, uint32_t value )

  Summary:
    Updates the Timer's 32-bit value.

  Description:
    This function updates the Timer's 32-bit value.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's
                  open routine
    value       - 32-bit counter value

  Returns:
    None.
*/

void DRV_TMR_Counter32BitSet( DRV_HANDLE handle, uint32_t value )
{
    /* Multi client variables are removed from single client builds. */
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    /* Update the counter values */
    PLIB_TMR_Counter32BitSet( gDrvTMRObj[dObj].tmrId, value );

    /* Reset the status information */
    gDrvTMRObj[dObj].elapseStatus = false;

} /* DRV_TMR_Counter32BitSet */


/******************************************************************************
  Function:
   uint16_t DRV_TMR_Counter16BitGet( DRV_HANDLE handle )

  Summary:
    Provides the 16-bit current counter value.

  Description:
    This function provides the 16-bit current counter value.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    16-bit current counter value.
*/

uint16_t DRV_TMR_Counter16BitGet( DRV_HANDLE handle )
{
    /* Multi client variables are removed from single client builds. */
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;
    uint16_t countVal = 0xFFFF;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    /* get the current counter value and return it */
    countVal = PLIB_TMR_Counter16BitGet( gDrvTMRObj[dObj].tmrId );

    return( countVal );

} /* DRV_TMR_Counter16BitGet */


/******************************************************************************
  Function:
   uint32_t DRV_TMR_Counter32BitGet( DRV_HANDLE handle )

  Summary:
    Provides the 32-bit current counter value.

  Description:
    This function provides the 32-bit current counter value.

  Parameters:
    handle       - A valid open instance handle, returned from the driver's
                   open routine

  Returns:
    32-bit current counter value.
*/

uint32_t DRV_TMR_Counter32BitGet( DRV_HANDLE handle )
{
    /* Multi client variables are removed from single client builds. */
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;
    uint32_t countVal = 0xFFFFFFFF;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    /* get the current counter value and return it */
    countVal = PLIB_TMR_Counter32BitGet( gDrvTMRObj[dObj].tmrId );

    return( countVal );

} /* DRV_TMR_Counter32BitGet */


/******************************************************************************
  Function:
   void DRV_TMR_SyncModeSet( DRV_HANDLE handle,
                             DRV_TMR_SYNC_MODE newMode )

  Summary:
    Changes the Timer Sync mode.

  Description:
    This function changes the Timer Sync mode.

  Parameters:
    handle       - A valid open instance handle, returned from the driver's
                   open routine
    newMode      - Possible values from the enumeration DRV_TMR_SYNC_MODE

  Returns:
    None.
*/

void DRV_TMR_SyncModeSet( DRV_HANDLE          handle,
                          DRV_TMR_SYNC_MODE   newMode )
{
    /* Multi client variables are removed from single client builds. */
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    /* if the new mode requested matches the existing one, simply return */
    if ( newMode == gDrvTMRObj[dObj].syncMode )
        return;

    /* Timer Mode Re Configuration */

    /* Clock Source Selection - Internal or External */
    if ( ( DRV_TMR_SYNC_MODE_SYNCHRONOUS_INTERNAL == newMode ) ||
         ( DRV_TMR_SYNC_MODE_SYNCHRONOUS_INTERNAL_GATED == newMode ) )
    {
        if( PLIB_TMR_ExistsClockSource( gDrvTMRObj[dObj].tmrId ) )
        {
            PLIB_TMR_ClockSourceSelect( gDrvTMRObj[dObj].tmrId , TMR_CLOCK_SOURCE_PERIPHERAL_CLOCK );
        }
    }
    else /* External Clock Input */
    {
        if( PLIB_TMR_ExistsClockSource( gDrvTMRObj[dObj].tmrId ) )
        {
            PLIB_TMR_ClockSourceSelect( gDrvTMRObj[dObj].tmrId, TMR_CLOCK_SOURCE_EXTERNAL_INPUT_PIN );
        }
    }

    /* Update the sync mode */
    _DRV_TMR_SyncModeControl(  gDrvTMRObj[dObj].tmrId, newMode );

    /* Update the sync mode of the Timer initialized */
    gDrvTMRObj[dObj].syncMode = newMode;

} /* DRV_TMR_SyncModeSet */


/******************************************************************************
  Function:
   DRV_TMR_SYNC_MODE DRV_TMR_SyncModeGet( DRV_HANDLE handle )

  Summary:
    Provides the Timer Sync mode.

  Description:
    This function provides the Timer Sync mode.

  Parameters:
    handle       - A valid open instance handle, returned from the driver's
                   open routine

  Returns:
    mode         - Possible values from the enumeration DRV_TMR_SYNC_MODE
*/

DRV_TMR_SYNC_MODE DRV_TMR_SyncModeGet ( DRV_HANDLE handle )
{
    /* Multi client variables are removed from single client builds. */
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    /* rturn the current sync mode */
    return ( gDrvTMRObj[dObj].syncMode );

} /* DRV_TMR_SyncModeGet */


/******************************************************************************
  Function:
    void DRV_TMR_Start ( DRV_HANDLE handle)

  Summary:
    Starts the Timer.

  Description:
    This function starts the Timer.

  Parameters:
    handle       - A valid open instance handle, returned from the driver's
                   open routine

  Returns:
    None.
*/

void DRV_TMR_Start ( DRV_HANDLE handle )
{
    /* Multi client variables are removed from single client builds. */
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    /* Start the Timer */
    if( PLIB_TMR_ExistsEnableControl( gDrvTMRObj[dObj].tmrId ) )
    {
        PLIB_TMR_Start( gDrvTMRObj[dObj].tmrId );
    }

    _DRV_TMR_InterruptSourceClear( gDrvTMRObj[dObj].interruptSource );

    /* Update the Client Status */
    gDrvTMRClientObj[clientObj].status = DRV_TMR_CLIENT_STATUS_STARTED;

} /* DRV_TMR_Start */


/******************************************************************************
  Function:
    void DRV_TMR_Stop( DRV_HANDLE handle)

  Summary:
    Stops the running Timer.

  Description:
    This function stops the running timer.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None.
*/

void DRV_TMR_Stop ( DRV_HANDLE handle )
{
    /* Multi client variables are removed from single client builds. */
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    /* Stop the Timer from running */
    if( PLIB_TMR_ExistsEnableControl( gDrvTMRObj[dObj].tmrId ) )
    {
        PLIB_TMR_Stop( gDrvTMRObj[dObj].tmrId );
    }

    /* Update the Client Status */
    gDrvTMRClientObj[clientObj].status = DRV_TMR_CLIENT_STATUS_STOPPED;

} /* DRV_TMR_Stop */


/******************************************************************************
  Function:
    bool DRV_TMR_ElapsedStatusGetAndClear( DRV_HANDLE handle )

  Summary:
    Returns the Timer count elapse status.

  Description:
    This function returns the Timer count elapse status.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's
                  open routine

  Returns:
    - true            - Timer has elapsed
    - false           - Timer has not elapsed/processing
*/

bool DRV_TMR_ElapsedStatusGetAndClear( DRV_HANDLE handle )
{
    /* Multi client variables are removed from single client builds. */
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;
    bool status = false;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    /* Get the updated Timer elapse status */
    status = gDrvTMRObj[dObj].elapseStatus;

    /* Check if the Timer elapsed */
    if ( status == true )
    {
        /* Reset the status information */
        gDrvTMRObj[dObj].elapseStatus = false;
    }

    /* return the current elapse status */
    return( status );

} /* DRV_TMR_ElapsedStatusGetAndClear */


/******************************************************************************
  Function:
    void DRV_TMR_AlarmSet( DRV_HANDLE handle,
                           const DRV_TMR_ALARM_CONFIG *config)

  Summary:
    Registers an alarm with the client.

  Description:
    This function registers an alarm with the client.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's
                  open routine
    config      - Pointer to the alarm configuration structure containing any
                  data necessary to initialize Timer alarms

  Returns:
    None.
*/

void DRV_TMR_AlarmSet( DRV_HANDLE                 handle,
                       const DRV_TMR_ALARM_CONFIG * config )
{
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    gDrvTMRObj[dObj].alarmInUse = true ;

    /* update the HW Instance with the callback associated with alarm */
    gDrvTMRObj[dObj].alarmCallback = config->callback ;

    if ( config->type == DRV_TMR_ALARM_TYPE_PERIODIC )
    {
        gDrvTMRObj[dObj].alarmPeriodic = true;
    }
    else
    {
        gDrvTMRObj[dObj].alarmPeriodic = false;
    }
    /* Clear the alarm counter */

} /* DRV_TMR_AlarmSet */

/******************************************************************************
  Function:
    uint32_t DRV_TMR_AlarmCountGet( DRV_HANDLE handle )

  Summary:
    Returns the number of times an alarm is generated since the last time it was
    cleared

  Description:
    This function returns the number of times an alarm is generated since the last
    time it was cleared.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's
                  open routine

  Returns:
    Alarm count value.
*/

unsigned int DRV_TMR_AlarmCountGet( DRV_HANDLE handle )
{
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    return gDrvTMRObj[dObj].alarmCount;

} /* DRV_TMR_AlarmCountGet */


/******************************************************************************
  Function:
    void DRV_TMR_AlarmCountClear( DRV_HANDLE handle )

  Summary:
    Clears the alarm counter.

  Description:
    This function clears the alarm counter.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's
                  open routine

  Returns:
    None.
*/

void DRV_TMR_AlarmCountClear( DRV_HANDLE handle )
{
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    /* Clear the Alarm counter */
    gDrvTMRObj[dObj].alarmCount = 0 ;

} /* DRV_TMR_AlarmCountClear */


/******************************************************************************
  Function:
   uint32_t DRV_TMR_OperatingFrequencyGet( DRV_HANDLE handle )

  Summary:
    Provides the Timer running frequency.

  Description:
    This function provides the Timer running frequency.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver
                  open routine
    tmrBaseFreq - Timer peripheral base frequency

  Returns:
    32-bit value corresponding to the Timer running frequency.
*/

uint32_t DRV_TMR_OperatingFrequencyGet ( DRV_HANDLE handle )
{
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;
    uint32_t prescale, tmrBaseFreq ;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    /* Get the Base frequency from the SYS CLK module */
    tmrBaseFreq = SYS_CLK_PeripheralFrequencyGet( CLK_BUS_PERIPHERAL_1 );

    /* PreScale Used */
    if( PLIB_TMR_ExistsPrescale( gDrvTMRObj[dObj].tmrId ) )
    {
        prescale = PLIB_TMR_PrescaleGet( gDrvTMRObj[dObj].tmrId );
    }
    else
    {
        prescale = 1;
    }

    return( tmrBaseFreq / prescale ) ;

} /* DRV_TMR_OperatingFrequencyGet */


/******************************************************************************
  Function:
   uint32_t DRV_TMR_TickFrequencyGet( DRV_HANDLE handle )

  Summary:
    Provides the timer tick frequency.

  Description:
    This function provides the timer tick frequency.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
    tmrBaseFreq - Timer peripheral base frequency

  Returns:
    32-bit value corresponding to the running tick frequency. For example,
    if the tick period is 100 ms, the return value would be 10 Hz.
*/

uint32_t DRV_TMR_TickFrequencyGet ( DRV_HANDLE handle )
{
    DRV_TMR_CLIENT_OBJ_HANDLE clientObj = (DRV_TMR_CLIENT_OBJ_HANDLE) 0;
    DRV_TMR_OBJ_HANDLE dObj = (DRV_TMR_OBJ_HANDLE) 0;
    uint32_t ticksUsed , ticksPerMs , prescale, tmrBaseFreq ;

    /* Get the Client object from the handle passed */
    clientObj = handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj < DRV_TMR_INSTANCES_NUMBER, "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    dObj = gDrvTMRClientObj[clientObj].driverObject;

    /* Get the Base frequency from the SYS CLK module */
    tmrBaseFreq = SYS_CLK_PeripheralFrequencyGet( CLK_BUS_PERIPHERAL_1 );

    /* Get the ticks to be counted for calculating the freq */
    ticksUsed = gDrvTMRObj[dObj].timerPeriod;

    ticksPerMs = ( ( uint32_t ) tmrBaseFreq / 1000 ) ;
    /* PreScale Used */
    if( PLIB_TMR_ExistsPrescale( gDrvTMRObj[dObj].tmrId ) )
    {
        prescale = PLIB_TMR_PrescaleGet( gDrvTMRObj[dObj].tmrId );
    }
    else
    {
        prescale = 1;
    }

    /* Considering the prescale information*/
    ticksPerMs = ticksPerMs / prescale ;

    return ( ( 1000 / (ticksUsed / ticksPerMs) ) ) ;

} /* DRV_TMR_TickFrequencyGet */


//******************************************************************************
/* Function:
    unsigned int DRV_TMR_VersionGet( const SYS_MODULE_INDEX drvIndex )

  Summary:
    Gets the Timer driver version in numerical format.

  Description:
    This function gets the Timer driver version. The version is encoded as
    major * 10000 + minor * 100 + patch. The stringed version can be obtained
    using DRV_TMR_VersionStrGet().

  Parameters:
    None.

  Returns:
    Current driver version in numerical format.
*/

unsigned int DRV_TMR_VersionGet( const SYS_MODULE_INDEX drvIndex )
{
    return( ( _DRV_TMR_VERSION_MAJOR * 10000 ) +
            ( _DRV_TMR_VERSION_MINOR * 100 ) +
            ( _DRV_TMR_VERSION_PATCH ) );

} /* DRV_TMR_VersionGet */


// *****************************************************************************
/* Function:
    char * DRV_TMR_VersionStrGet( const SYS_MODULE_INDEX drvIndex )

  Summary:
    Gets the Timer driver version in string format.

  Description:
    This function gets the Timer driver version. The version is returned as
    major.minor.path[type], where type is optional. The numerical version can
    be obtained using DRV_TMR_VersionGet()

  Parameters:
    None.

  Returns:
    Current Timer driver version in the string format.

  Remarks:
    None.
*/

char * DRV_TMR_VersionStrGet( const SYS_MODULE_INDEX drvIndex )
{
    return _DRV_TMR_VERSION_STR;

} /* DRV_TMR_VersionStrGet */


/*******************************************************************************
End of File
*/

