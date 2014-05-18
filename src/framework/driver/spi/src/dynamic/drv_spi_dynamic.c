/*******************************************************************************
  SPI Device Driver Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_spi_dynamic.c

  Summary:
    SPI Device Driver Dynamic Multiple Client Implementation

  Description:
    The SPI device driver provides a simple interface to manage the SPI
    modules on Microchip microcontrollers.  This file Implements the core
    interface routines for the SPI driver.

    While building the driver from source, ALWAYS use this file in the build.
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


// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include "driver/spi/src/drv_spi_local.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Variables
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Driver Hardware instance objects.

  Summary:
    Defines the hardware instances objects that are available on the part

  Description:
    This data type defines the hardware instance objects that are available on
    the part, so as to capture the hardware state of the instance.

  Remarks:
    Not all modes are available on all micro-controllers.
*/

static DRV_SPI_OBJ             gDrvSPIObj[DRV_SPI_INSTANCES_NUMBER] ;


// *****************************************************************************
/* Driver Client instance objects.

  Summary:
    Defines the Client instances objects that are available on the part

  Description:
    This data type defines the Client instance objects that are available on
    the part, so as to capture the Client state of the instance.

  Remarks:
    None
*/

static DRV_SPI_CLIENT_OBJ      gDrvSPIClientObj [ DRV_SPI_CLIENTS_NUMBER ] ;


// *****************************************************************************
/* Driver data objects.

  Summary:
    Defines the data object.

  Description:
    This data type defines the data objects. This is used to queue the user
    requests for different operations.

  Remarks:
    None
*/

static DRV_SPI_BUFFER_OBJECT	gDrvSPIBufferObj [ DRV_SPI_INSTANCES_NUMBER ][ DRV_SPI_BUFFER_OBJ_SIZE ];


// *****************************************************************************
/* Macro: _DRV_SPI_CLIENT_OBJ(obj,mem)

  Summary:
    Returns the appropriate client member

  Description:
    Either return the static object or returns the indexed dynamic object.
    This macro has variations for dynamic or static driver.
*/

#define _DRV_SPI_CLIENT_OBJ(obj,mem)    obj->mem


// *****************************************************************************
/* Macro: _DRV_SPI_CLIENT_OBJ_GET(obj)

  Summary:
    Returns the appropriate client instance

  Description:
    Either return the static object or returns the indexed dynamic object.
    This macro has variations for dynamic or static driver.
*/

#define _DRV_SPI_CLIENT_OBJ_GET(obj)    &gDrvSPIClientObj[obj]

// *****************************************************************************
/* Macro: _DRV_SPI_CLIENT_OBJ(obj,mem)

  Summary:
    Returns the appropriate client member

  Description:
    Either return the static object or returns the indexed dynamic object.
    This macro has variations for dynamic or static driver.
*/

#define _DRV_SPI_DATA_OBJ(obj,mem)    ((DRV_SPI_BUFFER_OBJECT*)(obj))->mem


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    static void _DRV_SPI_SetupHardware ( const SPI_MODULE_ID    plibId,
                                        DRV_SPI_OBJ_HANDLE     dObj,
                                        DRV_SPI_INIT           * spiInit )

  Summary:
    Sets up the hardware from the initialization structure

  Description:
    This routine sets up the hardware from the initialization structure.

  Remarks:
    None.
*/

static void _DRV_SPI_SetupHardware ( const SPI_MODULE_ID plibId,
                                     DRV_SPI_OBJ *dObj,
                                     DRV_SPI_INIT * spiInit )
{
    /* Initialize the Interrupt Sources */
	dObj->txInterruptSource = spiInit->txInterruptSource;
	dObj->rxInterruptSource = spiInit->rxInterruptSource;
	dObj->errInterruptSource = spiInit->errInterruptSource;

    /* Power state initialization */
    if( _DRV_SPI_POWER_STATE_GET( spiInit->moduleInit.value ) == SYS_MODULE_POWER_IDLE_STOP )
    {
        PLIB_SPI_StopInIdleEnable( plibId  );
    }
    else if( ( _DRV_SPI_POWER_STATE_GET( spiInit->moduleInit.value ) == SYS_MODULE_POWER_IDLE_RUN ) ||
                ( _DRV_SPI_POWER_STATE_GET( spiInit->moduleInit.value ) == SYS_MODULE_POWER_RUN_FULL ) )
    {
        PLIB_SPI_StopInIdleDisable( plibId  );
    }
    else
    {
        if( _DRV_SPI_POWER_STATE_GET( spiInit->moduleInit.sys.powerState ) == SYS_MODULE_POWER_IDLE_STOP )
        {
            PLIB_SPI_StopInIdleEnable( plibId  );
        }
    }
    /* Clock Mode Initialization */
    if( DRV_SPI_CLOCK_MODE_IDLE_LOW_EDGE_RISE == _DRV_SPI_CLOCK_OPERATION_MODE_GET( spiInit->clockMode ) )
    {
        /* Mode 0 - Idle State Low & Sampling on Rising Edge */
        /* Clock Polarity */
        PLIB_SPI_ClockPolaritySelect( plibId ,
            SPI_CLOCK_POLARITY_IDLE_LOW );

        /* Output Data Phase */
        PLIB_SPI_OutputDataPhaseSelect( plibId ,
            SPI_OUTPUT_DATA_PHASE_ON_IDLE_TO_ACTIVE_CLOCK );
    }
    else if( DRV_SPI_CLOCK_MODE_IDLE_LOW_EDGE_FALL == _DRV_SPI_CLOCK_OPERATION_MODE_GET( spiInit->clockMode ) )
    {
        /* Mode 1 - Idle State Low & Sampling on Falling Edge */
        /* Clock Polarity */
        PLIB_SPI_ClockPolaritySelect( plibId ,
            SPI_CLOCK_POLARITY_IDLE_LOW );

        /* Output Data Phase */
        PLIB_SPI_OutputDataPhaseSelect( plibId ,
            SPI_OUTPUT_DATA_PHASE_ON_ACTIVE_TO_IDLE_CLOCK );
    }
    else if( DRV_SPI_CLOCK_MODE_IDLE_HIGH_EDGE_FALL == _DRV_SPI_CLOCK_OPERATION_MODE_GET( spiInit->clockMode ) )
    {
        /* Mode 2 - Idle State High & Sampling on Falling Edge */
        /* Clock Polarity */
        PLIB_SPI_ClockPolaritySelect ( plibId ,
            SPI_CLOCK_POLARITY_IDLE_HIGH );

        /* Output Data Phase */
        PLIB_SPI_OutputDataPhaseSelect ( plibId ,
            SPI_OUTPUT_DATA_PHASE_ON_IDLE_TO_ACTIVE_CLOCK );
    }
    else // ( DRV_SPI_CLOCK_MODE_IDLE_HIGH_EDGE_RISE == _DRV_SPI_CLOCK_OPERATION_MODE_GET( spiInit->clockMode ) )
    {
        /* Mode 3 - Idle State High & Sampling on Rising Edge */
        /* Clock Polarity */
        PLIB_SPI_ClockPolaritySelect ( plibId ,
                        SPI_CLOCK_POLARITY_IDLE_HIGH );

        /* Output Data Phase */
        PLIB_SPI_OutputDataPhaseSelect ( plibId ,
                        SPI_OUTPUT_DATA_PHASE_ON_ACTIVE_TO_IDLE_CLOCK );
    }
    /* Update the clock mode */
    dObj->clockMode = _DRV_SPI_CLOCK_OPERATION_MODE_GET ( spiInit->clockMode );

    /* Input Sample Phase */
    if( PLIB_SPI_ExistsInputSamplePhase ( plibId  ) )
    {
        PLIB_SPI_InputSamplePhaseSelect ( plibId ,
                _DRV_SPI_INPUT_SAMPLE_PHASE_GET( spiInit->inputSamplePhase ) );
        dObj->inputSamplePhase = _DRV_SPI_INPUT_SAMPLE_PHASE_GET ( spiInit->inputSamplePhase );
    }

    /* Usage Mode Master/Slave */
    if( _DRV_SPI_USAGE_MODE_GET( spiInit->spiMode ) == DRV_SPI_MODE_MASTER )
    {
        /* Master Enable */
        PLIB_SPI_MasterEnable ( plibId  );
    }
    else
    {
        /* Slave Enable */
        PLIB_SPI_SlaveEnable ( plibId  );
    }
    dObj->spiMode = _DRV_SPI_USAGE_MODE_GET ( spiInit->spiMode );

    /* Slave Select Handling */
    if( _DRV_SPI_USAGE_MODE_GET ( spiInit->spiMode ) == DRV_SPI_MODE_SLAVE )
    {
        PLIB_SPI_SlaveSelectEnable ( plibId  );
    }
    else
    {
		PLIB_SPI_SlaveSelectDisable ( plibId  );
    }

    /* Communication Width Selection */
    PLIB_SPI_CommunicationWidthSelect ( plibId ,
                _DRV_SPI_COMMUNICATION_WIDTH_GET( spiInit->commWidth ) );
    dObj->commWidth = _DRV_SPI_COMMUNICATION_WIDTH_GET( spiInit->commWidth );

    /* Baudrate selection */
    // Below changed for PIC32 driver unit testing
    PLIB_SPI_BaudRateSet( plibId , SYS_CLK_PeripheralFrequencyGet(CLK_BUS_PERIPHERAL_1),
                _DRV_SPI_BAUD_RATE_VALUE_GET( spiInit->baudRate ) );

    /* Protocol Selection */
    if( _DRV_SPI_PROTOCOL_USAGE_TYPE_GET( spiInit->spiProtocolType ) == DRV_SPI_PROTOCOL_TYPE_FRAMED )
    {
        /* Frame Sync Pulse Direction */
        PLIB_SPI_FrameSyncPulseDirectionSelect( plibId ,
            DRV_SPI_FRAME_SYNC_PULSE_DIRECTION );

        /* Frame Sync Pulse Polarity */
        PLIB_SPI_FrameSyncPulsePolaritySelect( plibId ,
                    DRV_SPI_FRAME_SYNC_PULSE_POLARITY );

        /* Frame Sync Pulse Edge */
        PLIB_SPI_FrameSyncPulseEdgeSelect( plibId ,
                    DRV_SPI_FRAME_SYNC_PULSE_EDGE );

        /* Check if the framed communication is supported by the device */
        PLIB_SPI_FramedCommunicationEnable( plibId  );

        /* ToDo: Other Frame based parameters update, if any */
    }
    else if( _DRV_SPI_PROTOCOL_USAGE_TYPE_GET( spiInit->spiProtocolType ) == DRV_SPI_PROTOCOL_TYPE_AUDIO )
    {
        /* Check if the audio protocol is supported by the device */
        PLIB_SPI_AudioProtocolEnable( plibId  );

        /* ToDo: Other Audio based parameters update */
    }

    dObj->spiProtocolType = _DRV_SPI_PROTOCOL_USAGE_TYPE_GET( spiInit->spiProtocolType );

    /* Buffer type selection */
    if( PLIB_SPI_ExistsFIFOControl( plibId  ) )
    {
        if( _DRV_SPI_BUFFER_USAGE_TYPE_GET( spiInit->bufferType ) == DRV_SPI_BUFFER_TYPE_ENHANCED )
        {
            /* Enhanced Buffer Mode Enable */
            PLIB_SPI_FIFOEnable( plibId  );
        }
        else
        {
            /* Standard Buffer Mode Enable */
            PLIB_SPI_FIFODisable( plibId  );
        }

        dObj->bufferType = _DRV_SPI_BUFFER_USAGE_TYPE_GET( spiInit->bufferType );
    }

    /* Buffer interrupt mode selection */
    if( _DRV_SPI_BUFFER_USAGE_TYPE_GET( spiInit->bufferType ) == DRV_SPI_BUFFER_TYPE_ENHANCED )
    {
        /* Update the transmit buffer interrupt mode */
        _DRV_SPI_TxFIFOInterruptModeSelect( plibId ,
                        _DRV_SPI_TX_FIFO_INTERRUPT_MODE_GET( spiInit->txInterruptMode ) );
        /* Update the receive buffer interrupt mode */
        _DRV_SPI_RxFIFOInterruptModeSelect( plibId ,
                        _DRV_SPI_RX_FIFO_INTERRUPT_MODE_GET( spiInit->rxInterruptMode ) );
    }
} /* _DRV_SPI_SetupHardware */


// *****************************************************************************
// *****************************************************************************
// Section: Driver Interface Function Definitions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_SPI_Initialize ( const SYS_MODULE_INDEX  index,
                                       const SYS_MODULE_INIT * const init )

  Summary:
    Initializes hardware and data for the given instance of the SPI module

  Description:
    This routine initializes hardware for the instance of the SPI module,
    using the hardware initialization given data.  It also initializes all
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
__attribute__((section("driver")))
SYS_MODULE_OBJ DRV_SPI_Initialize ( const SYS_MODULE_INDEX   drvIndex,
                                   const SYS_MODULE_INIT    * const init )
{
    DRV_SPI_INIT * spiInit;
    SPI_MODULE_ID spiId;

	  /* Validate the driver index */
    if ( drvIndex >= DRV_SPI_INDEX_COUNT )
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    DRV_SPI_OBJ *dObj = _DRV_SPI_INSTANCE_GET ( drvIndex );

    /* Assign to the local pointer the init data passed */
    spiInit = ( DRV_SPI_INIT * ) init;

    /* Object is valid, set it in use */
    dObj->inUse = true;

    /* Save the index of the driver. Important to know this
    as we are using reference based accessing */
    dObj->drvIndex = drvIndex;

    /* Update the SPI Module Index */
    dObj->spiId = spiInit->spiId;
	
	/* Speed up accessing, take it to a local variable */
    spiId = dObj->spiId;

    /* Setup the Hardware */
    _DRV_SPI_SetupHardware ( spiId, dObj, spiInit );

    /* Reset the number of clients */
    dObj->numClients = 0;

    /* Reset the locally used variables */
    dObj->lastClientHandle      = DRV_SPI_CLIENTS_NUMBER+1;

    /* Slave Select Handling */
    if( _DRV_SPI_USAGE_MODE_GET(spiInit->spiMode) == DRV_SPI_MODE_SLAVE )
    {
        PLIB_SPI_BufferClear( spiId );
    }

    /* Clear the SPI Overflow Flag */

    PLIB_SPI_ReceiverOverflowClear ( spiId );

    /* Interrupt flag cleared on the safer side */
    _DRV_SPI_InterruptSourceClear( dObj->txInterruptSource );
    _DRV_SPI_InterruptSourceClear( dObj->rxInterruptSource );
    _DRV_SPI_InterruptSourceClear( dObj->errInterruptSource );

    /* Set the current driver state */
    dObj->status = SYS_STATUS_READY;

    /* Enable the SPI module */
    PLIB_SPI_Enable( spiId ) ;

    /* Return the driver handle */
    return( (SYS_MODULE_OBJ) dObj );
} /* DRV_SPI_Initialize */


//******************************************************************************
/* Function:
    void DRV_SPI_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    De-initializes the specific module instance of the SPI module

  Description:
    De-initializes the specific module instance disabling its operation (and any
    hardware for driver modules).  Resets all the internal data structures and
    fields for the specified instance to the default settings.

  Parameters:
    object          - Identifies the Driver Object returned by the Initialize
                      interface

  Returns:
    None
*/
__attribute__((section("driver")))
void DRV_SPI_Deinitialize ( SYS_MODULE_OBJ object )
{
    DRV_SPI_OBJ *dObj = (DRV_SPI_OBJ*) object;
    SPI_MODULE_ID spiId = dObj->spiId;
	size_t iClient;
    /* Interrupt De-Registration */
    _DRV_SPI_InterruptSourceDisable ( dObj->txInterruptSource );
    _DRV_SPI_InterruptSourceDisable ( dObj->rxInterruptSource );
    _DRV_SPI_InterruptSourceDisable ( dObj->errInterruptSource );

    /* Stop/Disable the device if needed */
    if( PLIB_SPI_ExistsEnableControl( spiId ) )
    {
        PLIB_SPI_Disable( spiId );
    }

	for ( iClient = 0; iClient < DRV_SPI_CLIENTS_NUMBER ; iClient++ )
    {
		gDrvSPIClientObj[iClient].inUse = false;
	}

	dObj->numClients = 0;
	dObj->isExclusive = false;
	/* Clear all the pending requests */
	dObj->queueHead = NULL;
    /* Set the Device Status */
    dObj->status = SYS_STATUS_UNINITIALIZED;

    /* Remove the driver usage */
    dObj->inUse = false;
} /* DRV_SPI_Deinitialize */


//******************************************************************************
/* Function:
    SYS_STATUS DRV_SPI_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the hardware instance of the SPI module

  Description:
    This routine Provides the current status of the hardware instance of the
    SPI module.

  Parameters:
    object          - Identifies the Driver Object returned by the Initialize
                      interface

  Returns:
    SYS_STATUS_READY    Indicates that any previous module operation for the
                        specified module has completed

    SYS_STATUS_BUSY     Indicates that a previous module operation for the
                        specified module has not yet completed

    SYS_STATUS_ERROR    Indicates that the specified module is in an error state
*/
__attribute__((section("driver")))
SYS_STATUS DRV_SPI_Status ( SYS_MODULE_OBJ object )
{
    if ( object == SYS_MODULE_OBJ_INVALID )
    {
        //SYS_ASSERT( " Handle is invalid " );
        return SYS_MODULE_OBJ_INVALID;
    }
    DRV_SPI_OBJ *dObj = (DRV_SPI_OBJ*) object;

    /* Return the status associated with the driver handle */
    return ( dObj->status );
} /* DRV_SPI_Status */


//******************************************************************************
/* Function:
    void DRV_SPI_Tasks ( SYS_MODULE_OBJ object )

  Summary:
    Used to maintain the driver's state machine and implement its ISR

  Description:
    This routine is used to maintain the driver's internal state machine and
    implement its ISR for interrupt-driven implementations.

  Parameters:
    object          - Identifies the Driver Object returned by the Initialize
                      interface

  Returns:
    None.
*/

__attribute__((section("driver")))
void DRV_SPI_Tasks ( SYS_MODULE_OBJ object )
{

    DRV_SPI_OBJ             *dObj           = (DRV_SPI_OBJ*)object;
    DRV_SPI_BUFFER_OBJECT   *lBufferObj     = dObj->taskLObj;
    SPI_DATA_TYPE           dummy           = 0xFF;
    SPI_MODULE_ID           spiId           = dObj->spiId;
    DRV_SPI_CLIENT_OBJ      *lClientObj;

    switch ( dObj->task )
    {
        case DRV_SPI_TASK_PROCESS_QUEUE:
            if ( object == SYS_MODULE_OBJ_INVALID )
            {
                //SYS_ASSERT( " Handle is invalid " );
                return;
            }
            /* Pop the first element from the queue */
            if ( ( dObj->queueHead != NULL ) && ( PLIB_SPI_TransmitBufferIsEmpty ( spiId ) ) )
            {
                /* This should be a global variable since the control could
                go out of this function */
                dObj->taskLObj = dObj->queueHead;

                /* Take it to a local variable to avoid multiple referencing and to speed up */
                lBufferObj   = dObj->taskLObj;

                /* Update the queue head */
                dObj->queueHead = lBufferObj->next;

                lClientObj = (DRV_SPI_CLIENT_OBJ *)lBufferObj->clientHandle;

                /* We need not do this all the time but when the request from a
                 new client  */
                _DRV_SPI_CLIENT_SWITCH_CLIENT();

                if ( true != _DRV_SPI_CLIENT_OBJ(lClientObj, chipSelectLogicLevel) )
                {
                    //_DRV_SPI_CHIP_SELECT_CLEAR(_DRV_SPI_CLIENT_OBJ(lClientObj, chipSelectPort),
                    //    _DRV_SPI_CLIENT_OBJ(lClientObj, chipSelectPort));
                }
                else
                {
                    //_DRV_SPI_CHIP_SELECT_SET(_DRV_SPI_CLIENT_OBJ(lClientObj, chipSelectPort),
                    //    _DRV_SPI_CLIENT_OBJ(lClientObj, chipSelectPort));
                }

                /* Update the task state as per the operation */
                dObj->task = lBufferObj->operation + DRV_SPI_TASK_PROCESS_READ_ONLY;

                if ( lBufferObj->operation == DRV_SPI_OP_READ )
                {
                    PLIB_SPI_BufferWrite( spiId , dummy ) ;
                }
                else
                {
                    PLIB_SPI_BufferWrite ( spiId, *lBufferObj->txBuffer++ ) ;
                }
            }
            break;
        case DRV_SPI_TASK_PROCESS_READ_ONLY:
            /* If RX buffer has some existing data to be read, read (receive them from FIFO) */
            while ( lBufferObj->transferSize )
            {
                if ( PLIB_SPI_ReceiverBufferIsFull ( spiId ) )
                {
                    *lBufferObj->rxBuffer++ = PLIB_SPI_BufferRead ( spiId );

                    /* Handle the overflow */
                    if ( lBufferObj->transferSize > 1 )
                    {
                            PLIB_SPI_BufferWrite( spiId, dummy ) ;
                            lBufferObj->transferSize--;
                    }
                    else /* If all transmission is complete */
                    {
                        /* Hold the buffer till the completion of the operation */
                        lBufferObj->inUse = false;
                        lBufferObj->transferSize--;
                        dObj->task = DRV_SPI_TASK_PROCESS_QUEUE;

                        _DRV_SPI_InterruptSourceDisable ( dObj->rxInterruptSource ) ;

                        _DRV_SPI_CLIENT_OBJ(lBufferObj, status) |=
                                DRV_SPI_BUFFER_EVENT_COMPLETE;

                        /* Have a check here because DRV_SPI_ClientSetup function call is optional */
                        if ( _DRV_SPI_CLIENT_OBJ(((DRV_SPI_CLIENT_OBJ *)lBufferObj->clientHandle), callback) != NULL )
                        {
                            /* Give an indication to the higher layer upon successful transmission */
                            _DRV_SPI_CLIENT_OBJ(((DRV_SPI_CLIENT_OBJ *)lBufferObj->clientHandle),
                                callback)( DRV_SPI_BUFFER_EVENT_COMPLETE, (DRV_SPI_BUFFER_HANDLE)lBufferObj, 0x00 );
                        }
                    }
                }
                else
                {
                    /* Do not block in any case */
                    break;
                }
            }
            break;
        case DRV_SPI_TASK_PROCESS_WRITE_ONLY:
            /* Loop till the transmit size, do not block though */
            while ( lBufferObj->transferSize )
            {
                if ( PLIB_SPI_ReceiverBufferIsFull ( spiId ) )
                {
                    dummy = PLIB_SPI_BufferRead ( spiId );

                    /* Handle the overflow */
                    if ( lBufferObj->transferSize > 1 )
                    {
                        PLIB_SPI_BufferWrite ( spiId, *lBufferObj->txBuffer++ );
						lBufferObj->transferSize--;
                    }
                    else
                    {
                        lBufferObj->transferSize--;

                        /* Hold the buffer till the completion of the operation */
                        lBufferObj->inUse = false;

                        dObj->task = DRV_SPI_TASK_PROCESS_QUEUE;

                        _DRV_SPI_InterruptSourceDisable ( dObj->txInterruptSource ) ;

                        _DRV_SPI_CLIENT_OBJ(lBufferObj, status) |=
                            DRV_SPI_BUFFER_EVENT_COMPLETE;

                       /* Have a check here because DRV_SPI_ClientSetup function call is optional */
                        if ( _DRV_SPI_CLIENT_OBJ(((DRV_SPI_CLIENT_OBJ *)lBufferObj->clientHandle), callback) != NULL )
                        {
                            /* Give an indication to the higher layer upon successful transmission */
                            _DRV_SPI_CLIENT_OBJ(((DRV_SPI_CLIENT_OBJ *)lBufferObj->clientHandle), callback)
                                    ( DRV_SPI_BUFFER_EVENT_COMPLETE, (DRV_SPI_BUFFER_HANDLE)lBufferObj, 0x00 );
                        }
                    }
                }
                else
                {
                    /* Do not block in any case */
                    break;
                }
             }
            break;
        case DRV_SPI_TASK_PROCESS_WRITE_READ:
            while( lBufferObj->transferSize )
            {
                if ( PLIB_SPI_ReceiverBufferIsFull ( spiId ) ) 
                {
                    *lBufferObj->rxBuffer++ = PLIB_SPI_BufferRead ( spiId );

                    if ( lBufferObj->transferSize > 1 )
                    {
                        PLIB_SPI_BufferWrite ( spiId, *lBufferObj->txBuffer++ ) ;
                        lBufferObj->transferSize--;
                    }
                    else
                    {
                        dObj->task = DRV_SPI_TASK_PROCESS_QUEUE;
                        lBufferObj->transferSize--;
                        /* Hold the buffer till the completion of the operation */
                        lBufferObj->inUse = false;
                        _DRV_SPI_InterruptSourceDisable ( dObj->txInterruptSource ) ;
                        _DRV_SPI_InterruptSourceDisable ( dObj->rxInterruptSource ) ;

                        _DRV_SPI_CLIENT_OBJ(lBufferObj, status) |=
                            DRV_SPI_BUFFER_EVENT_COMPLETE;

                        /* Have a check because DRV_SPI_ClientSetup function call is optional */
                        if ( _DRV_SPI_CLIENT_OBJ(((DRV_SPI_CLIENT_OBJ *)lBufferObj->clientHandle), callback) != NULL )
                        {
                            /* Give an indication to the higher layer upon successful transmission */
                            _DRV_SPI_CLIENT_OBJ(((DRV_SPI_CLIENT_OBJ *)lBufferObj->clientHandle), callback)
                                    ( DRV_SPI_BUFFER_EVENT_COMPLETE, (DRV_SPI_BUFFER_HANDLE) lBufferObj, 0x00 );
                        }
                    }
                }
                else
                {
                    /* Do not block in any case */
                    break;
                }
            }
            break;
        default:
            break;

        }

    /* This state is encountered when an error interrupt has occurred.
       or an error has occurred during read */
    if ( true == _DRV_SPI_InterruptSourceStatusGet ( dObj->errInterruptSource ) )
    {
        /* Check for the overflow error */
        if ( PLIB_SPI_ReceiverHasOverflowed ( spiId ) ) 
        {
            if ( PLIB_SPI_ExistsReceiverOverflow ( spiId ) )
            {
                PLIB_SPI_ReceiverOverflowClear ( spiId );
            }

            /* Update the transfer status */
            dObj->transferStatus |= DRV_SPI_BUFFER_EVENT_ERROR;
        }

        _DRV_SPI_InterruptSourceClear ( dObj->errInterruptSource );
    }
}


//******************************************************************************
/* Function:
    DRV_HANDLE DRV_SPI_Open ( const SYS_MODULE_INDEX    index,
                             const DRV_IO_INTENT       intent )

  Summary:
    Opens the specific module instance and returns a handle

  Description:
    This routine opens a driver for use by any client module and provides a
    handle that must be provided to any of the other driver operations to
    identify the caller and the instance of the driver/hardware module.

  Parameters:
    index           - Identifier for the instance to be initialized
    ioIntent        - Possible values from the enumeration DRV_IO_INTENT

  Returns:
    If successful, the routine returns a valid open-instance handle (a number
    identifying both the caller and the module instance)
    If an error occurs, the return value is DRV_HANDLE_INVALID
*/
__attribute__((section("driver")))
DRV_HANDLE DRV_SPI_Open ( const SYS_MODULE_INDEX   drvIndex,
                         const DRV_IO_INTENT      ioIntent )
{
    /* Multi client variables are removed from single client builds. */
    DRV_SPI_CLIENT_OBJ  *clientObj      =
                        ( DRV_SPI_CLIENT_OBJ* ) _DRV_SPI_CLIENT_OBJ_GET(drvIndex);
    DRV_SPI_OBJ         *dObj;
    size_t              iClient;

    /* Validate the driver index */
    if( drvIndex >= DRV_SPI_INDEX_COUNT )
    {
        return DRV_HANDLE_INVALID;
    }
    /* Setup client operations */
    /* Find available slot in array of client objects */
    for ( iClient = 0; iClient < DRV_SPI_CLIENTS_NUMBER ; iClient++ )
    {
        if ( !clientObj->inUse )
        {
            /* Increment the client in case of Multi client support, otherwise remove
            the below statement */
            dObj = _DRV_SPI_INSTANCE_GET(drvIndex);

            clientObj->inUse  = true;
            clientObj->driverObject = dObj;
            clientObj->intent = ioIntent;

            /* Check for exclusive access */
            if ( ( dObj->isExclusive == true ) || ( dObj->inUse != true ) ||
                (( dObj->numClients > 0 ) && DRV_IO_ISEXCLUSIVE( ioIntent )) )
            {
                /* Set that the hardware instance is opened in exclusive mode */
                return DRV_HANDLE_INVALID;
            }

            /* Check if max number of clients open */
            if( dObj->numClients >= DRV_SPI_CLIENTS_NUMBER )
            {
                /* Set that the hardware instance is opened with max clients */
                return DRV_HANDLE_INVALID;
            }

            dObj->numClients++;

            /* Update that, the client is opened in exclusive access mode */
            if( DRV_IO_ISEXCLUSIVE( ioIntent ) )
            {
                dObj->isExclusive = true;
            }
            /* To Do: OSAL - Unlock Mutex */

            /* Return the client object */
            return ( DRV_HANDLE ) clientObj;
        }
        clientObj++;
    }
    /* TODO OSAL - Unlock Mutex */
    return  DRV_HANDLE_INVALID ;
} /* DRV_SPI_Open */


//******************************************************************************
/* Function:
    void DRV_SPI_Close ( DRV_HANDLE handle )

  Summary:
    Closes an opened-instance of a driver

  Description:
    This routine closes an opened-instance of a driver, invalidating the given
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None
*/
__attribute__((section("driver")))
void DRV_SPI_Close ( DRV_HANDLE handle )
{
    /* Multi client variables are removed from single client builds. */
    DRV_SPI_CLIENT_OBJ *clientObj = (DRV_SPI_CLIENT_OBJ*)handle;

    /* To Do: OSAL - lock Mutex */

    /* Free the Client Instance */
    clientObj->inUse = false ;
    /* To Do: OSAL - unlock Mutex */
} /* DRV_SPI_Close */


// *****************************************************************************
/* Function:
    void DRV_SPI_ClientSetup ( DRV_HANDLE handle,
                                 const DRV_SPI_CLIENT_SETUP * const config )

  Summary:
    Sets up the device communication parameters

  Description:
    This function sets up the device communication parameters

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

    config       - Communication parameters identified by DRV_SPI_COMM_CONFIG

  Returns:
    None
*/
__attribute__((section("driver")))
void DRV_SPI_ClientSetup ( DRV_HANDLE handle,
                                 const DRV_SPI_CLIENT_SETUP * const config )
{
    /* Multi client variables are removed from single client builds. */
    DRV_SPI_CLIENT_OBJ *clientObj = (DRV_SPI_CLIENT_OBJ*) handle;

    /* Get the Client object from the handle passed */
    /* Get the driver object from the client */
    _DRV_SPI_SAVE_LAST_CLIENT();

    clientObj->baudRate	             = config->baudRate;
    clientObj->inputSamplePhase      = config->inputSamplePhase;
    clientObj->clockMode             = config->clockMode;
    clientObj->chipSelectLogicLevel  = config->chipSelectLogicLevel;
    clientObj->chipSelectPort        = config->chipSelectPort;
    clientObj->chipSelectBitPos      = config->chipSelectBitPos;

} /* DRV_SPI_ClientSetup */


// *****************************************************************************
/* Function:
    static void _DRV_SPI_ClientHardwareSetup ( DRV_HANDLE handle )

  Summary:
    Sets up the device communication parameters

  Description:
    This function sets up the device communication parameters

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

    config       - Communication parameters identified by DRV_SPI_COMM_CONFIG

  Returns:
    None
*/
#if 0 /* Removed temporarily to avoid build warnings. */
__attribute__((section("driver")))
static void _DRV_SPI_ClientHardwareSetup ( DRV_HANDLE handle )
{
    /* Multi client variables are removed from single client builds. */
    DRV_SPI_CLIENT_OBJ  *clientObj  = (DRV_SPI_CLIENT_OBJ *) handle;
    /* Get the driver object from the client */
    DRV_SPI_OBJ         *dObj       = clientObj->driverObject;
    SPI_MODULE_ID       spiId       = dObj->spiId;

    /* Stop/Disable the device */
    PLIB_SPI_Disable( spiId );

    /* Input Sample Phase */
    PLIB_SPI_InputSamplePhaseSelect( spiId,
            _DRV_SPI_CLIENT_OBJ(clientObj, inputSamplePhase) );

    /* Clock Mode Initialization */
    if( DRV_SPI_CLOCK_MODE_IDLE_LOW_EDGE_RISE == _DRV_SPI_CLIENT_OBJ(clientObj, clockMode) )
    {
        /* Mode 0 - Idle State Low & Sampling on Rising Edge */
        /* Clock Polarity */
        PLIB_SPI_ClockPolaritySelect( spiId, SPI_CLOCK_POLARITY_IDLE_LOW );

        /* Output Data Phase */
        PLIB_SPI_OutputDataPhaseSelect( spiId, SPI_OUTPUT_DATA_PHASE_ON_IDLE_TO_ACTIVE_CLOCK );

    }
    else if( DRV_SPI_CLOCK_MODE_IDLE_LOW_EDGE_FALL == _DRV_SPI_CLIENT_OBJ(clientObj, clockMode) )
    {
        /* Mode 1 - Idle State Low & Sampling on Falling Edge */
        /* Clock Polarity */
        PLIB_SPI_ClockPolaritySelect( spiId, SPI_CLOCK_POLARITY_IDLE_LOW );

    /* Output Data Phase */
    PLIB_SPI_OutputDataPhaseSelect( spiId, SPI_OUTPUT_DATA_PHASE_ON_ACTIVE_TO_IDLE_CLOCK );

    }
    else if( DRV_SPI_CLOCK_MODE_IDLE_HIGH_EDGE_FALL == _DRV_SPI_CLIENT_OBJ(clientObj, clockMode) )
    {
        /* Mode 2 - Idle State High & Sampling on Falling Edge */
        /* Clock Polarity */
        if( PLIB_SPI_ExistsClockPolarity( spiId ) )
        {
            PLIB_SPI_ClockPolaritySelect( spiId, SPI_CLOCK_POLARITY_IDLE_HIGH );
        }

        /* Output Data Phase */
        PLIB_SPI_OutputDataPhaseSelect( spiId, SPI_OUTPUT_DATA_PHASE_ON_IDLE_TO_ACTIVE_CLOCK );
    }
    else // ( DRV_SPI_CLOCK_MODE_IDLE_HIGH_EDGE_RISE == config->clockMode )
    {
        /* Mode 3 - Idle State High & Sampling on Rising Edge */

        /* Clock Polarity */
        PLIB_SPI_ClockPolaritySelect( spiId, SPI_CLOCK_POLARITY_IDLE_HIGH );

        /* Output Data Phase */
        PLIB_SPI_OutputDataPhaseSelect( spiId, SPI_OUTPUT_DATA_PHASE_ON_ACTIVE_TO_IDLE_CLOCK );
    }
    /* Baud-rate selection */

    // Below for PIC32
    PLIB_SPI_BaudRateSet( spiId, SYS_CLK_PeripheralFrequencyGet(CLK_BUS_PERIPHERAL_1),
                _DRV_SPI_CLIENT_OBJ(clientObj, baudRate) );

    /* Restart the device */
    PLIB_SPI_Enable( spiId );
} /* _DRV_SPI_ClientHardwareSetup */
#endif


//******************************************************************************
/* Function:
    void DRV_SPI_BufferEventHandlerSet ( const DRV_HANDLE handle,
                    const DRV_SPI_BUFFER_EVENT_HANDLER eventHandler, 
                    const uintptr_t context )

  Summary:
	Adds a callback function for a client.

  Description:
    This routine adds a callback function for a client.

  Parameters:
    handle       	- A valid open-instance handle, returned from the driver's
						open routine.

	eventHandler    - Call back function.

	context		 	-  context.

  Returns:
    None.
*/

void DRV_SPI_BufferEventHandlerSet (const DRV_HANDLE handle,
                    const DRV_SPI_BUFFER_EVENT_HANDLER eventHandler, 
                    const uintptr_t context )
{
	( (DRV_SPI_CLIENT_OBJ *) handle)->callback = eventHandler;

	( (DRV_SPI_CLIENT_OBJ *) handle)->context = context;
}


//******************************************************************************
/* Function:
    DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddRead ( DRV_HANDLE handle, void *rxBuffer,
                                    size_t size )

  Summary:
    Adds a buffer to queue with a read request. Driver will process this
    request in the task routine.

  Description:
    This routine adds a buffer to queue with a read request. Driver will process
    this request in the task routine.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

	rxBuffer	 - Buffer to which the data to be filled.

	size		 - Number of bytes to be read.

  Returns:
    DRV_SPI_BUFFER_HANDLE using which application can track the current status of
    the buffer.
*/

__attribute__((section("driver")))
DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddRead ( DRV_HANDLE handle, void *rxBuffer,
                                    size_t size )
{
    DRV_SPI_BUFFER_OBJECT   *spiDataObj;
    DRV_SPI_OBJ             *dObj = ((DRV_SPI_CLIENT_OBJ *) handle)->driverObject;

    /* Get a slot in the queue */
    spiDataObj = _DRV_SPI_QueueSlotGet ( dObj );

    if ( spiDataObj != NULL )
    {
        /* Fill the data directly to the queue. Set the inUse flag only at the end */
        spiDataObj->clientHandle    = handle;
        spiDataObj->operation       = DRV_SPI_OP_READ;
        spiDataObj->txBuffer        = NULL;
        spiDataObj->rxBuffer        = rxBuffer;
        spiDataObj->transferSize    = size;

        if ( dObj->queueHead == NULL )
        {
            dObj->queueHead = spiDataObj;
        }
        else
        {
            dObj->queueHead->next = spiDataObj;
        }
        _DRV_SPI_InterruptSourceEnable( dObj->rxInterruptSource ) ;

        return (DRV_SPI_BUFFER_HANDLE)spiDataObj;
    }
    return (DRV_SPI_BUFFER_HANDLE)NULL;
}


//******************************************************************************
/* Function:
    DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddWrite ( DRV_HANDLE handle, void *txBuffer,
                                    size_t size )

  Summary:
    Adds a buffer to queue with a write request. Driver will process this
    request in the task routine.

  Description:
    This routine adds a buffer to queue with a read request. Driver will process
    this request in the task routine.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

	rxBuffer	 - Buffer to which the data to be filled.

	size		 - Number of bytes to b read.

  Returns:
    DRV_SPI_BUFFER_HANDLE using which application can track the current status of
    the buffer.
*/
__attribute__((section("driver")))
DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddWrite ( DRV_HANDLE handle, void *txBuffer,
                                    size_t size )
{
    DRV_SPI_BUFFER_OBJECT *spiDataObj;
    DRV_SPI_OBJ *dObj = ((DRV_SPI_CLIENT_OBJ *) handle)->driverObject;

    /* Get a slot in the queue */
    spiDataObj = _DRV_SPI_QueueSlotGet (dObj);

    if ( spiDataObj != NULL )
    {
        /* Fill the data directly to the queue. Set the inUse flag only at the end */
        spiDataObj->clientHandle    = handle;
        spiDataObj->operation       = DRV_SPI_OP_WRITE;
        spiDataObj->txBuffer        = txBuffer;
        spiDataObj->rxBuffer        = NULL;
        spiDataObj->transferSize    = size;

        if ( dObj->queueHead == NULL )
        {
            dObj->queueHead = spiDataObj;
        }
        else
        {
            dObj->queueHead->next = spiDataObj;
        }

        _DRV_SPI_InterruptSourceEnable( dObj->txInterruptSource ) ;

        return (DRV_SPI_BUFFER_HANDLE) spiDataObj;
    }
    return (DRV_SPI_BUFFER_HANDLE)NULL;
}


//******************************************************************************
/* Function:
    DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddWriteRead ( DRV_HANDLE handle, void *txBuffer,
                                            void *rxBuffer, uint32_t size )

  Summary:
    Adds a buffer to queue with a read request. Driver will process this
    request in the task routine.

  Description:
    This routine adds a buffer to queue with a read request. Driver will process
    this request in the task routine.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

	txBuffer 	 - Buffer which has the data.

	rxBuffer	 - Buffer to which the data to be filled.

	size		 - Number of bytes to b read.

  Returns:
    DRV_SPI_BUFFER_HANDLE using which application can track the current status of
    the buffer.
*/
__attribute__((section("driver")))
DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddWriteRead ( DRV_HANDLE handle, void *txBuffer,
                                            void *rxBuffer, size_t size )
{
    DRV_SPI_BUFFER_OBJECT   *spiDataObj;
    DRV_SPI_OBJ             *dObj = ((DRV_SPI_CLIENT_OBJ *) handle)->driverObject;

    /* Get a slot in the queue */
    spiDataObj = _DRV_SPI_QueueSlotGet ( dObj );

    if ( spiDataObj != NULL )
    {
        /* Fill the data directly to the queue. Set the inUse flag only at the end */
        spiDataObj->clientHandle   	= handle;
        spiDataObj->operation           = DRV_SPI_OP_READ_WRITE;
        spiDataObj->txBuffer            = txBuffer;
        spiDataObj->rxBuffer            = rxBuffer;
        spiDataObj->transferSize        = size;

        if ( dObj->queueHead == NULL )
        {
            dObj->queueHead = spiDataObj;
        }
        else
        {
            dObj->queueHead->next = spiDataObj;
        }

        _DRV_SPI_InterruptSourceEnable( dObj->txInterruptSource );
        _DRV_SPI_InterruptSourceEnable( dObj->rxInterruptSource );

        return (DRV_SPI_BUFFER_HANDLE) spiDataObj;
    }
    return (DRV_SPI_BUFFER_HANDLE)NULL;
}


// *****************************************************************************
/* Function:
    DRV_SPI_BUFFER_EVENT DRV_SPI_BufferStatus ( DRV_SPI_BUFFER_HANDLE bufferHandle )

  Summary:
    Returns the transmitter and receiver transfer status

  Description:
    This returns the transmitter and receiver transfer status.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    A DRV_SPI_TRANSFER_STATUS value describing the current status of the
    transfer.
*/
__attribute__((section("driver")))
DRV_SPI_BUFFER_EVENT DRV_SPI_BufferStatus ( DRV_SPI_BUFFER_HANDLE bufferHandle )
{
    /* return the transfer status. This doesn't have any protection */
    return _DRV_SPI_DATA_OBJ(bufferHandle, status);

} /* DRV_SPI_TransferStatus */


// *****************************************************************************
/* Function:
    size_t DRV_SPI_Read ( DRV_HANDLE handle, void* buffer, size_t noOfBytes )

  Summary:
    Reads specified bytes from the SPI module.

  Description:
    This function reads specified bytes from the SPI module.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

	buffer		 - Buffer to which the read data should be written.

	noOfBytes	 - Number of bytes to be read.
  Returns:
    Number of bytes read.
*/
__attribute__((section("driver")))
size_t DRV_SPI_Read ( DRV_HANDLE handle, void* buffer, size_t noOfBytes )
{
    register char       *tempBuffer = buffer, state =0;
    DRV_SPI_OBJ         *dObj       = ((DRV_SPI_CLIENT_OBJ *) handle)->driverObject;
    SPI_MODULE_ID       spiId       = dObj->spiId;
    size_t copyNoOfBytes = noOfBytes;
    
    /* Client is not opened to read */
    if  ( !( ( ( DRV_SPI_CLIENT_OBJ * ) handle )->intent & DRV_IO_INTENT_READ ) )
    {
        return 0;
    }
    /* If there is something in the queue, return. Come back after some time. */
    if ( dObj->queueHead != NULL  )
    {
        return 0;
    }

    /* Block until the operation is complete */
    while ( noOfBytes )
    {
        /* Wait until the transmit buffer is free */
        if ( ( state == 0 )  && ( PLIB_SPI_TransmitBufferIsEmpty ( spiId ) ) )
        {
            PLIB_SPI_BufferWrite( spiId, 0xFF ) ;
            state++;
        }
        /* Wait until the data is received back */
        if ( ( state == 1 ) && ( PLIB_SPI_ReceiverBufferIsFull ( spiId ) ) )
        {
            *tempBuffer++ = PLIB_SPI_BufferRead ( spiId );
            noOfBytes--;
            state = 0;
        }
    }
    
    return copyNoOfBytes;
}


// *****************************************************************************
/* Function:
    size_t DRV_SPI_Write ( DRV_HANDLE handle, void* buffer, size_t noOfBytes )

  Summary:
    Writes specified bytes from the SPI module.

  Description:
    This function writes specified bytes from the SPI module.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

	buffer		 - Buffer to which holds the data.

	noOfBytes	 - Number of bytes to be written.
  Returns:
    Number of bytes written.
*/
__attribute__((section("driver")))
size_t DRV_SPI_Write ( DRV_HANDLE handle, void* buffer, size_t noOfBytes )
{
    register char *tempBuffer       = buffer, dummy, state =0;
    DRV_SPI_OBJ *dObj               = ((DRV_SPI_CLIENT_OBJ *) handle)->driverObject;
    SPI_MODULE_ID spiId             = dObj->spiId;
	size_t copyNoOfBytes = noOfBytes;
	
    /* Client is not opened to write */
    if  ( !( ( ( DRV_SPI_CLIENT_OBJ * ) handle )->intent & DRV_IO_INTENT_WRITE ) )
    {
        return 0;
    }

    /* If there is something in the queue, return. Come back after some time. */
    if ( dObj->queueHead != NULL  )
    {
        return 0;
    }
    /* Block until the operation is complete */
    while ( noOfBytes )
    {
        if ( ( state == 0 ) && ( PLIB_SPI_TransmitBufferIsEmpty ( spiId ) ) )
        {
            PLIB_SPI_BufferWrite ( spiId, *tempBuffer++ );
			state++;
        }
        if ( ( state == 1 ) && ( PLIB_SPI_ReceiverBufferIsFull ( spiId ) ) )
        {
            dummy = PLIB_SPI_BufferRead ( spiId );
			noOfBytes--;
			state = 0;
		}
    }
    
    return copyNoOfBytes;
}


// *****************************************************************************
/* Function:
    DRV_SPI_BUFFER_OBJ* _DRV_SPI_QueueSlotGet ( DRV_SPI_OBJ *dObj )

  Summary:
    Adds an element to the queue.

  Description:
    This API adds an element to the queue.

  Parameters:
    spiDataObj   - Pointer to the structure which holds the data which is to be
    				added to the queue.

  Returns:
    DRV_SPI_BUFFER_HANDLE - Handle, a pointer to the allocated element in the
    						queue.
*/
__attribute__((section("driver")))
DRV_SPI_BUFFER_OBJECT* _DRV_SPI_QueueSlotGet ( DRV_SPI_OBJ *dObj )
{
    int index;
    DRV_SPI_BUFFER_OBJECT *lQueueObj;
    SYS_MODULE_INDEX drvIndex = dObj->drvIndex;

    for ( index=0; index<DRV_SPI_BUFFER_OBJ_SIZE; index++ )
    {
        lQueueObj = &gDrvSPIBufferObj [ drvIndex ][ index ];

        if ( lQueueObj->inUse == false )
        {
            /* This should be the first thing we do after getting a slot */
            lQueueObj->inUse    = true;
            lQueueObj->status   = DRV_SPI_BUFFER_EVENT_PENDING;

            /* Return the pointer reference */
            return lQueueObj;
        }
    }
    return NULL;
}


//******************************************************************************
/* Function:
    unsigned int DRV_SPI_VersionGet( const SYS_MODULE_INDEX drvIndex )

  Summary:
    Gets SPI driver version in numerical format.

  Description:
    This routine gets the SPI driver version. The version is encoded as
    major * 10000 + minor * 100 + patch. The stringized version can be obtained
    using DRV_SPI_VersionStrGet()

  Parameters:
    None.

  Returns:
    Current driver version in numerical format.
*/
__attribute__((section("driver")))
unsigned int DRV_SPI_VersionGet( const SYS_MODULE_INDEX drvIndex )
{
    return( ( _DRV_SPI_VERSION_MAJOR * 10000 ) +
            ( _DRV_SPI_VERSION_MINOR * 100 ) +
            ( _DRV_SPI_VERSION_PATCH ) );

} /* DRV_SPI_VersionGet */


// *****************************************************************************
/* Function:
    char * DRV_SPI_VersionStrGet ( const SYS_MODULE_INDEX drvIndex )

  Summary:
    Gets SPI driver version in string format.

  Description:
    This routine gets the SPI driver version. The version is returned as
    major.minor.path[type], where type is optional. The numerical version can
    be obtained using DRV_SPI_VersionGet()

  Parameters:
    None.

  Returns:
    Current SPI driver version in the string format.

  Remarks:
    None.
*/
__attribute__((section("driver")))
char * DRV_SPI_VersionStrGet( const SYS_MODULE_INDEX drvIndex )
{
    return _DRV_SPI_VERSION_STR;

} /* DRV_SPI_VersionStrGet */

/*******************************************************************************
End of File
*/



