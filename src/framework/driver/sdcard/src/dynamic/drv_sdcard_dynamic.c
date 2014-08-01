/*******************************************************************************
  SD CARD Device Driver Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdcard_dynamic.c

  Summary:
    SD CARD Device Driver Dynamic Implementation

  Description:
    The SD CARD device driver provides a simple interface to manage the SD CARD
    modules on Microchip microcontrollers.  This file Implements the core
    interface routines for the SD CARD driver.

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
#include "system_config.h"
#include "driver/sdcard/src/drv_sdcard_local.h"
#include "driver/spi/src/drv_spi_local.h"
#include "system/ports/sys_ports.h"
//#include "osal/osal.h"
#include "peripheral/spi/plib_spi.h"
char first_time = 1, inLoop =0;
int previousCommand = 8, testVariable1=0; unsigned char testVariable2[10];
// *****************************************************************************
// *****************************************************************************
// Section: File Scope Variables
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* SD Card command table

  Summary:
    Defines the a command table for SD card.

  Description:
    This data structure makes a command table for the SD Card with the command,
    its CRC, expected response and a flag indicating whether the driver expects
    more data or not. This makes the SD card commands easier to handle.

  Remarks:
    The actual response for the command 'CMD_SD_SEND_OP_COND'is R3, but it has
    same number of bytes as R7. So R7 is used in the table.
*/


const DRV_SDCARD_CMD_OBJ gDrvSDCARDCmdTable[] =
{
    /* Command                             crc     response       more data */
    {CMD_VALUE_GO_IDLE_STATE,             0x95,   RESPONSE_R1,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_SEND_OP_COND,               0xF9,   RESPONSE_R1,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_SEND_IF_COND,               0x87,   RESPONSE_R7,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_SEND_CSD,                   0xAF,   RESPONSE_R1,         DRV_SDCARD_GET_MOREDATA},
    {CMD_VALUE_SEND_CID,                   0x1B,   RESPONSE_R1,         DRV_SDCARD_GET_MOREDATA},
    {CMD_VALUE_STOP_TRANSMISSION,          0xC3,   RESPONSE_R1,        DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_SEND_STATUS,                0xAF,   RESPONSE_R2,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_SET_BLOCKLEN,               0xFF,   RESPONSE_R1,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_READ_SINGLE_BLOCK,          0xFF,   RESPONSE_R1,         DRV_SDCARD_GET_MOREDATA},
    {CMD_VALUE_READ_MULTI_BLOCK,           0xFF,   RESPONSE_R1,         DRV_SDCARD_GET_MOREDATA},
    {CMD_VALUE_WRITE_SINGLE_BLOCK,         0xFF,   RESPONSE_R1,         DRV_SDCARD_GET_MOREDATA},
    {CMD_VALUE_WRITE_MULTI_BLOCK,          0xFF,   RESPONSE_R1,         DRV_SDCARD_GET_MOREDATA},
    {CMD_VALUE_TAG_SECTOR_START,           0xFF,   RESPONSE_R1,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_TAG_SECTOR_END,             0xFF,   RESPONSE_R1,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_ERASE,                      0xDF,   RESPONSE_R1b,        DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_APP_CMD,                    0x73,   RESPONSE_R1,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_READ_OCR,                   0x25,   RESPONSE_R7,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_CRC_ON_OFF,                 0x25,   RESPONSE_R1,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_SD_SEND_OP_COND,            0xFF,   RESPONSE_R7,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_SET_WR_BLK_ERASE_COUNT,     0xFF,   RESPONSE_R1,         DRV_SDCARD_GET_NODATA}
};


// *****************************************************************************
/* SD card media functions.

  Summary:
    These functions are used by the 'media manager' to access the SD card.

  Description:
	These functions are used by the 'media manager' to access the SD card. The
	call will be by using a function pointer. So SD card driver must attach these
	functions to the media manager on initialize.

  Remarks:
    None.
*/

const SYS_FS_MEDIA_FUNCTIONS sdcardMediaFunctions =
{
    .mediaStatusGet     = DRV_SDCARD_MediaStatusGet,
    .sectorRead         = DRV_SDCARD_SectorRead,
    .sectorWrite        = DRV_SDCARD_SectorWrite,
    .bufferStatusGet    = DRV_SDCARD_BufferStatusGet,
    .open               = DRV_SDCARD_Open,
    .close              = DRV_SDCARD_Close,
    .tasks              = DRV_SDCARD_Tasks
};


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

DRV_SDCARD_OBJ              gDrvSDCARDObj[DRV_SDCARD_INSTANCES_NUMBER];


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

DRV_SDCARD_CLIENT_OBJ       gDrvSDCARDClientObj[DRV_SDCARD_CLIENTS_NUMBER * DRV_SDCARD_INSTANCES_NUMBER];


// *****************************************************************************
/* Driver buffer objects.

  Summary:
    Transfer objects for the driver queue.

  Description:
    This isntance of the structure is used as transfer objects for the driver
    queue.

  Remarks:
    None
*/

DRV_SDCARD_XFER_OBJECT      gDrvSDCARDTransferObj[DRV_SDCARD_INSTANCES_NUMBER][ DRV_SDCARD_QUEUE_POOL_SIZE ];


// *****************************************************************************
/* Driver queue object

  Summary:
    Variables to handle the queue.

  Description:
    This isntance of the structure holds the variables to handle the queue.

  Remarks:
    None
*/

DRV_SDCARD_QUEUE_OBJECT   	gDrvSDCARDQueueObj[DRV_SDCARD_INSTANCES_NUMBER];


// *****************************************************************************
/* Macro: _DRV_SDCARD_CLIENT_OBJ(obj,mem)

  Summary:
    Returns the appropriate client member

  Description:
    Either return the static object or returns the indexed dynamic object.
    This macro has variations for dynamic or static driver.
*/

#define _DRV_SDCARD_CLIENT_OBJ(obj,mem)    gDrvSDCARDClientObj[obj].mem


// *****************************************************************************
/* Macro: _DRV_SDCARD_CLIENT_OBJ_GET(obj)

  Summary:
    Returns the appropriate client intance

  Description:
    Either return the static object or returns the indexed dynamic object.
    This macro has variations for dynamic or static driver.
*/

#define _DRV_SDCARD_CLIENT_OBJ_GET(obj)    &gDrvSDCARDClientObj[obj]


// *****************************************************************************
/* Macro: _DRV_SDCARD_INDEX_GET(drvIndex)

  Summary:
    Returns the appropriate driver id for the configuration

  Description:
    Either return the statically declared driver id or returns the dynamic index
    passed into the macro. This macro has variations for dynamic or static
    driver

*/

#define _DRV_SDCARD_INDEX_GET(drvIndex)                            (drvIndex)


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    static void _DRV_SDCARD_SetupHardware( const SDCARD_MODULE_ID   plibId,
                                           DRV_SDCARD_INIT          * sdcardInit )

  Summary:
    Sets up the hardware from the initialization structure

  Description:
    This routine sets up the hardware from the initialization structure.

  Remarks:
    Called
*/

static void _DRV_SDCARD_SetupHardware ( const SDCARD_MODULE_ID   plibId,
                                       DRV_SDCARD_INIT          * sdcardInit )
{
    /* USe system ports to set the port bits */
    /* Set the Card detect pin */
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT,
                                    sdcardInit->cardDetectPort,
                                    sdcardInit->cardDetectBitPosition );

    /* Set the Card 'write protection status' pin */
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT,
                                    sdcardInit->writeProtectPort,
                                    sdcardInit->writeProtectBitPosition );

        /* Set the Card 'Chip Select' pin */
    SYS_PORTS_PinDirectionSelect ( PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT,
                                    sdcardInit->chipSelectPort,
                                    sdcardInit->chipSelectBitPosition );

} /* _DRV_SDCARD_SetupHardware */


// *****************************************************************************
// *****************************************************************************
// Section: Driver Interface Function Definitions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_SDCARD_Initialize( const SYS_MODULE_INDEX    drvIndex,
                                          const SYS_MODULE_INIT     * const init )

  Summary:
    Initializes hardware and data for the given instance of the SD CARD module

  Description:
    This routine initializes hardware for the instance of the SD CARD module,
    using the hardware initialization given data.  It also initializes all
    necessary internal data.

  Parameters:
    drvIndex        - Identifies the driver instance to be initialized.

    init            - Pointer to the data structure containing all data
                      necessary to initialize the hardware. This pointer may
                      be null if no data is required and static initialization
                      values are to be used.

  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID.
*/

SYS_MODULE_OBJ DRV_SDCARD_Initialize ( const SYS_MODULE_INDEX    drvIndex,
                                      const SYS_MODULE_INIT     * const init )
{
    DRV_SDCARD_INIT 		*sdcardInit     = NULL;
    DRV_SDCARD_OBJ           	*dObj 		= (DRV_SDCARD_OBJ*)NULL;

    /* Validate the driver index */
    if ( _DRV_SDCARD_INDEX_GET( drvIndex ) >= DRV_SDCARD_INDEX_COUNT )
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Allocate the driver object and set the operation flag to be in use */
    dObj = _DRV_SDCARD_INSTANCE_GET( drvIndex );

    SYS_ASSERT( dObj->inUse == false, "Instance already in use" );

    /* Assign to the local pointer the init data passed */
    sdcardInit = ( DRV_SDCARD_INIT * ) init;

    /* All the state varibles have their default state as value '0'.
    So, if we set the whole instance to zero, we only have to initialize
    the other elesments */
    memset ( dObj, 0, sizeof ( DRV_SDCARD_OBJ ) );

    /* Set that this instance of the driver is in use */
    dObj->inUse = true;

    dObj->drvIndex = drvIndex;

    /* Data to send olny clock pulses */
    dObj->dataToSendClockPulses = 0xFF;

    /* Update the USART PLIB Id */
    dObj->spiId      = (sdcardInit->spiId);

	/* SPI driver index */
	dObj->spiIndex   = sdcardInit->spiIndex;

    /* These pins are I/O pins used by the SD card */
    dObj->cardDetectBitPosition     = sdcardInit->cardDetectBitPosition;
    dObj->cardDetectPort            = sdcardInit->cardDetectPort;
    dObj->chipSelectBitPosition     = sdcardInit->chipSelectBitPosition;
    dObj->chipSelectPort            = sdcardInit->chipSelectPort;
    dObj->writeProtectBitPosition   = sdcardInit->writeProtectBitPosition;
    dObj->writeProtectPort          = sdcardInit->writeProtectPort;
    //dObj->devName                   = sdcardInit->devName;

    /* Speed at which SD card is going to operate at. This should be less than
    the maximum SPI frequency and should be supported by the SD card used */
    dObj->sdcardSpeedHz = sdcardInit->sdcardSpeedHz;

    /* Setup the Hardware */
    _DRV_SDCARD_SetupHardware (  _DRV_SDCARD_PERIPHERAL_ID_GET( sdcardInit->spiId ) ,
                                sdcardInit );

	/* Initialize the SD Card queue */
    dObj->queueHandle = _DRV_SDCARD_QueueInitialize ( drvIndex );

//	SYS_FS_MEDIA_MANAGER_MediaEventHandler ( sdcardInit->devName,
//					SYS_FS_MEDIA_DRIVER_REGISTER,
//					&sdcardMediaFunctions );
    if(DRV_SDCARD_MEDIA_MANAGER_USE)
    {
        SYS_FS_MEDIA_MANAGER_Register((SYS_MODULE_OBJ)drvIndex,  (SYS_MODULE_INDEX)drvIndex, &sdcardMediaFunctions, SYS_FS_MEDIA_TYPE_SD_CARD);
    }

    /* Update the status */
    ( dObj->status ) = SYS_STATUS_READY;

    /* Return the object structure */
    return ( ( SYS_MODULE_OBJ ) drvIndex );

} /* DRV_SDCARD_Initialize */


//******************************************************************************
/* Function:
    void DRV_SDCARD_Reinitialize( SYS_MODULE_OBJ        object,
                                  const SYS_MODULE_INIT * const init )

  Summary:
    Reinitializes and refreshes the hardware for the instance of the SD CARD
    module

  Description:
    This routine reinitializes and refreshes the hardware for the instance
    of the SD CARD module using the hardware initialization given data.
    It does not clear or reinitialize internal data structures

  Parameters:
    object          - Identifies the Driver Object returned by the Initialize
                      interface
    init            - Pointer to the data structure containing any data
                      necessary to initialize the hardware.

  Returns:
    None
*/

void DRV_SDCARD_Reinitialize( SYS_MODULE_OBJ        object ,
                              const SYS_MODULE_INIT * const init )
{
    DRV_SDCARD_INIT 			*sdcardInit = NULL;
    DRV_SDCARD_OBJ           	*dObj 		= ( DRV_SDCARD_OBJ* ) NULL;

    /* Validate the driver object */
    SYS_ASSERT ( object != SYS_MODULE_OBJ_INVALID, "Invalid system object handle" );

    /* Assign to the local pointer the init data passed */
    sdcardInit = ( DRV_SDCARD_INIT * ) init;

    /* get the driver object */
    dObj = ( DRV_SDCARD_OBJ* ) _DRV_SDCARD_INSTANCE_GET( object );

	/* Set the current driver state */
    ( dObj->status ) = SYS_STATUS_UNINITIALIZED;

    /* Setup the Hardware */
    _DRV_SDCARD_SetupHardware(  _DRV_SDCARD_PERIPHERAL_ID_GET ( sdcardInit->spiId ),
                                sdcardInit );
    /* Update the status */
    ( dObj->status ) = SYS_STATUS_READY;

} /* DRV_SDCARD_Reinitialize */


//******************************************************************************
/* Function:
    void DRV_SDCARD_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specific module instance of the SD CARD module

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

void DRV_SDCARD_Deinitialize( SYS_MODULE_OBJ object )
{
    DRV_SDCARD_OBJ                *dObj;

    /* Allocate the driver object and set the operation flag to be in use */
    dObj = ( DRV_SDCARD_OBJ* ) _DRV_SDCARD_INSTANCE_GET ( object );

    /* Set the Device Status */
    dObj->status  = SYS_STATUS_UNINITIALIZED;

    /* Remove the driver usage */
    dObj->inUse  = false;

} /* DRV_SDCARD_Deinitialize */


//******************************************************************************
/* Function:
    SYS_STATUS DRV_SDCARD_Status( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the hardware instance of the SD CARD module

  Description:
    This routine Provides the current status of the hardware instance of the
    SD CARD module.

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

SYS_STATUS DRV_SDCARD_Status( SYS_MODULE_OBJ object )
{
    DRV_SDCARD_OBJ            	*dObj;

    /* Allocate the driver object and set the operation flag to be in use */
    dObj = ( DRV_SDCARD_OBJ* ) _DRV_SDCARD_INSTANCE_GET ( object );

    /* Return the status associated with the driver handle */
    return( dObj->status ) ;

} /* DRV_SDCARD_Status */


//******************************************************************************
/* Function:
    void DRV_SDCARD_Tasks ( SYS_MODULE_OBJ object )

  Summary:
    Used to maintain the driver's state machine and implement

  Description:
    This routine is used to maintain the driver's internal state machine.

  Parameters:
    object          - Identifies the Driver Object returned by the Initialize
                      interface

  Returns:
    None
*/

void DRV_SDCARD_Tasks ( SYS_MODULE_OBJ object )
{
    DRV_SDCARD_OBJ              *dObj;
    DRV_SDCARD_XFER_OBJECT      *tObj;
    DRV_SDCARD_TASK_OPERATIONS  *lObj;
    uint32_t                    preEraseBlockCount = 0;

    /* Get the driver object */
    dObj = ( DRV_SDCARD_OBJ* ) _DRV_SDCARD_INSTANCE_GET ( object );

    /* Assign to a local pointer for faster operation */
    lObj = &dObj->localTaskObj;

    /* Check what state we are in, to decide what to do */
    switch ( dObj->taskState )
    {
        case DRV_SDCARD_TASK_OPEN_SPI:
            /* Open the SPI in exclusive mode. No other tasks will be
            allowed to control the SD card. But the could be multiple cients
            for SD card driver */
            dObj->spiClientHandle = DRV_SPI_Open ( dObj->spiIndex, DRV_IO_INTENT_READWRITE );

            dObj->taskState = DRV_SDCARD_TASK_CHECK_DEVICE;
            break;
        case DRV_SDCARD_TASK_CHECK_DEVICE:
			/* Check whether we are coming here for the first time. We will process the
			Queue otherwise */
            if ( ( dObj->isAttached != 0 ) && ( dObj->isAttachedLastStatus != 0 ) )
            {
                if ( !_DRV_SDCARD_IsQueueEmpty ( (DRV_SDCARD_QUEUE_HANDLE)&gDrvSDCARDQueueObj[dObj->drvIndex] ) )
                {
                    dObj->taskState = DRV_SDCARD_TASK_PROCESS_QUEUE;
                }
            }
			/* Check for defice attach */
            dObj->isAttached = _DRV_SDCARD_MediaDetect ( object );
			/* TODO Add a non- blocking delay here  */
            if ( dObj->isAttached != dObj->isAttachedLastStatus )
            {
                /* We should call a function on device attach and detach */
                if ( dObj->isAttached == DRV_SDCARD_IS_ATTACHED )
                {
                    /* Do initialize in a separate state, it need to get called
                    repeatedly */
                    dObj->taskState = DRV_SDCARD_TASK_MEDIA_INIT;
                }
                else
                {
                    /* State that the device is attached. */
                    dObj->mediaState = SYS_FS_MEDIA_DETACHED;
                }
                /* Save the last status */
                dObj->isAttachedLastStatus = dObj->isAttached;
            }
            /* Stay in the same case if the device is not attached to the system
            . Otherwise check for read/Write operations */
            break;
        case DRV_SDCARD_TASK_MEDIA_INIT:
			/* Update the card details to the internal data structure */
            _DRV_SDCARD_MediaInitialize ( object );

            /* Once the initialization is complete, move to the next stage */
            if ( dObj->mediaInitState == DRV_SDCARD_MEDIA_STATE_INITIALIZATION_COMPLETE )
            {
                /* State that the device is attached. */
                dObj->mediaState = SYS_FS_MEDIA_ATTACHED;

                /* Call the SYS FS Media Manager Event Handler */
//                dObj->disk = SYS_FS_MEDIA_MANAGER_MediaEventHandler ( dObj->devName,
//                        SYS_FS_MEDIA_EVENT_ATTACH, &object );
                dObj->taskState = DRV_SDCARD_TASK_PROCESS_QUEUE;
            }
            break;
         case DRV_SDCARD_TASK_PROCESS_QUEUE:
            /* Get the first in element from the queue */
            tObj = _DRV_SDCARD_ReadFromQueue ( dObj->queueHandle );

            if ( tObj == NULL )
            {
                /* If there are no read queued, check for device attach/detech */
                dObj->taskState = DRV_SDCARD_TASK_CHECK_DEVICE;
            }
            else
            {
                /* Copy input structure into a global instance of the structure,
                for faster local access of the parameters with smaller code size.
                */
                lObj->buffer        = tObj->buffer;
                lObj->readWrite     = tObj->readWrite;
                lObj->sectorAddress = tObj->sectorAddress;
                lObj->sectorCount   = tObj->sectorCount;

                dObj->bufferStatus  = &tObj->status;

				/* Update the buffer status as 'processing in progress' */
                *dObj->bufferStatus = SYS_FS_MEDIA_BUFFER_IN_PROGRESS;

                /* Select the card */
                _DRV_SDCARD_CHIP_SELECT ( dObj->chipSelectPort,
                dObj->chipSelectBitPosition );

                /* Navigate to different cases based on read/write flags */
                if ( lObj->readWrite == DRV_SDCARD_TRANSFER_READ )
                {
                    dObj->taskState = DRV_SDCARD_TASK_PROCESS_READ;
                }
                else
                {
                    dObj->taskState = DRV_SDCARD_TASK_PROCESS_WRITE;
                }
            }
            break;
        case DRV_SDCARD_TASK_PROCESS_READ:

            /* Initiate the write sequence.
            Initialize counter.  Will be used later for block boundary tracking. */
            lObj->blockCounter = _DRV_SDCARD_MEDIA_BLOCK_SIZE;

            /* Check if we are writing only a single block worth of data, or
            multiple blocks worth of data */
            if( lObj->sectorCount <= 1 )
            {
                /* Send command to read single block */
                if(dObj->sdCardType == DRV_SDCARD_MODE_NORMAL)
                {
                    _DRV_SDCARD_CommandSend ( object, DRV_SDCARD_READ_SINGLE_BLOCK,
                            (lObj->sectorAddress << 9) );
                }
                else if(dObj->sdCardType == DRV_SDCARD_MODE_HC)
                {
                    _DRV_SDCARD_CommandSend ( object, DRV_SDCARD_READ_SINGLE_BLOCK,
                            lObj->sectorAddress );
                }
                previousCommand = DRV_SDCARD_READ_SINGLE_BLOCK;
            }
            else
            {
                 /* Send command to read multiple blocks */
                if(dObj->sdCardType == DRV_SDCARD_MODE_NORMAL)
                {
                    _DRV_SDCARD_CommandSend ( object, DRV_SDCARD_READ_MULTI_BLOCK,
                            (lObj->sectorAddress << 9) );
                }
                else if(dObj->sdCardType == DRV_SDCARD_MODE_HC)
                {
                    _DRV_SDCARD_CommandSend ( object, DRV_SDCARD_READ_MULTI_BLOCK,
                            lObj->sectorAddress );
                }
                previousCommand = DRV_SDCARD_READ_MULTI_BLOCK;
            }
            
            if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
            {
                /* We send the command, now wait for response */
                dObj->taskState = DRV_SDCARD_TASK_READ_WAIT_RESPONSE;
            }
            /* Note: _DRV_SDCARD_CommandSend() sends 8 SPI clock cycles after
            getting the response.  This meets the NAC min timing paramemter, so
            we don't need additional clocking here.
            */
            break;
        case DRV_SDCARD_TASK_READ_WAIT_RESPONSE:
            //TODO add time out

             if ( dObj->cmdResponse.response1.byte != 0x00 )
             {
                 /* Perhaps the card isn't initialized or present */
                 dObj->taskState = DRV_SDCARD_TASK_READ_ABORT;
             }
             else
             {
                 dObj->taskState = DRV_SDCARD_TASK_READ_GET_BUSY_STATE;
             }

            break;
        case DRV_SDCARD_TASK_READ_GET_BUSY_STATE:
            dObj->spiBufferHandle = DRV_SPI_BufferAddRead ( dObj->spiClientHandle,
                       &dObj->packetArray[0], 1 );

            dObj->taskState = DRV_SDCARD_TASK_READ_WAIT_START_TOKEN;
            break;
        case DRV_SDCARD_TASK_READ_WAIT_START_TOKEN:
            /* In this case, we have already issued the READ_MULTI_BLOCK command
            to the media, and we need to keep polling the media until it sends
            us the data start token byte.  This could typically take a
            couple/few milliseconds, up to a maximum of 100ms. TODO handle timeout.
            */
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus
                    ( dObj->spiBufferHandle ) )
            {
                if ( dObj->packetArray[0] != DRV_SDCARD_MMC_FLOATING_BUS )
                {
                    if ( dObj->packetArray[0] == DRV_SDCARD_DATA_START_TOKEN )
                    {
                        dObj->taskState = DRV_SDCARD_TASK_READ_NEW_PACKET_READY;
                    }
                    else
                    {
                        dObj->taskState = DRV_SDCARD_TASK_READ_ABORT;
                    }
                }
                else
                {
                    dObj->taskState = DRV_SDCARD_TASK_READ_GET_BUSY_STATE;
                }
            }
            break;
        case DRV_SDCARD_TASK_READ_NEW_PACKET_READY:
            //TODO: handle multiple blocks
            /* Add the buffer for read */
            dObj->spiBufferHandle = DRV_SPI_BufferAddRead ( dObj->spiClientHandle, lObj->buffer,
                    _DRV_SDCARD_MEDIA_BLOCK_SIZE );

            dObj->taskState = DRV_SDCARD_TASK_READ_CHECK_DATA_READ_CMPLTE;
            break;
        case DRV_SDCARD_TASK_READ_CHECK_DATA_READ_CMPLTE:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus
                    ( dObj->spiBufferHandle ) )
            {
                /* Do a dummy read for the CRC bytes */
                dObj->spiBufferHandle = DRV_SPI_BufferAddRead ( dObj->spiClientHandle, dObj->packetArray, 4 );

                if ( (lObj->sectorCount--) > 1 )
                {
                    lObj->buffer += _DRV_SDCARD_MEDIA_BLOCK_SIZE;
                    dObj->taskState = DRV_SDCARD_TASK_READ_BLOCK_CMPLTE_CRC_READ;
                }
                else
                {
                    dObj->taskState = DRV_SDCARD_TASK_READ_CMPLTE_SEND_CLOCKS;
                }
            }
            break;
        case DRV_SDCARD_TASK_READ_BLOCK_CMPLTE_CRC_READ:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus
            ( dObj->spiBufferHandle ) )
            {
                dObj->taskState = DRV_SDCARD_TASK_READ_NEW_PACKET_READY;
            }
            break;
        case DRV_SDCARD_TASK_READ_CMPLTE_SEND_CLOCKS:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus
                    ( dObj->spiBufferHandle ) )
            {
                /* Send 8 clock pulses */
                dObj->spiBufferHandle = DRV_SPI_BufferAddRead ( dObj->spiClientHandle, dObj->mediaInitArray, 1 );

                dObj->taskState = DRV_SDCARD_TASK_READ_PROCESS_NEXT;
            }
            break;
        case DRV_SDCARD_TASK_READ_PROCESS_NEXT:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus
                    ( dObj->spiBufferHandle ) )
            {

				/* Unselect the chip */
                _DRV_SDCARD_CHIP_DESELECT ( dObj->chipSelectPort,
                dObj->chipSelectBitPosition );
		first_time =0;
		/* Check for the next queue entry */
                dObj->taskState = DRV_SDCARD_TASK_READ_STOP_TRANSMISSION;
            }
            break;
        case DRV_SDCARD_TASK_READ_STOP_TRANSMISSION:
            if ( previousCommand == DRV_SDCARD_READ_MULTI_BLOCK )
            {
                if ((!first_time) ||(inLoop))
                {
                    /* Send command to read multiple blocks */
                    _DRV_SDCARD_CommandSend ( object, DRV_SDCARD_STOP_TRANSMISSION,
                    0 );
                    first_time = 1;
                    inLoop = 1;
                    if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
                    {
                        dObj->spiBufferHandle = DRV_SPI_BufferAddRead ( dObj->spiClientHandle, testVariable2, 8 );

                        /* Check for the next queue entry */
                        dObj->taskState = DRV_SDCARD_TASK_READ_STOP_TRANSMIT_CMPLT;

                        inLoop = 0;
                    }
                }
            }
            else
            {
                /* Check for the next queue entry */
                dObj->taskState = DRV_SDCARD_TASK_PROCESS_QUEUE;
                /* Set the status as buffer processing completed */
                lObj->status = DRV_SDCARD_BUFFER_COMPLETED;

                /* Update the buffer status as 'processing in progress' */
                *dObj->bufferStatus = SYS_FS_MEDIA_BUFFER_COMPLETED;

            }
            break;
        case DRV_SDCARD_TASK_READ_STOP_TRANSMIT_CMPLT:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus
                    ( dObj->spiBufferHandle ) )
            {
                /* Check for the next queue entry */
                dObj->taskState = DRV_SDCARD_TASK_PROCESS_QUEUE;

                /* Set the status as buffer processing completed */
                lObj->status = DRV_SDCARD_BUFFER_COMPLETED;

                /* Update the buffer status as 'processing in progress' */
                *dObj->bufferStatus = SYS_FS_MEDIA_BUFFER_COMPLETED;
            }
            break;
        case DRV_SDCARD_TASK_PROCESS_WRITE:
            /* Initiate the write sequence.
            Initialize counter.  Will be used later for block boundary tracking. */
            lObj->blockCounter = _DRV_SDCARD_MEDIA_BLOCK_SIZE;

            /* Check if we are writing only a single block worth of data, or
            multiple blocks worth of data, change this condition later on
			multiple command */
            if( _DRV_SDCARD_MEDIA_BLOCK_SIZE > 0 )
            {
                lObj->command = DRV_SDCARD_WRITE_SINGLE_BLOCK;

                dObj->taskState = DRV_SDCARD_TASK_MODIFY_ADDRESS;
            }
            else
            {
                lObj->command = DRV_SDCARD_WRITE_MULTI_BLOCK;

                /* Compute the number of blocks that we are going to be writing
                in this multi-block write operation. Divide by 512 to get the
                number of blocks to write */
                preEraseBlockCount = ( 512 / _DRV_SDCARD_MEDIA_BLOCK_SIZE );

                /* Always need to erase at least one block */
                if( preEraseBlockCount == 0 )
                {
                    preEraseBlockCount++;
                }

                /* Should send CMD55/ACMD23 to let the media know how many blocks
                it should pre-erase.  This isn't essential, but it allows for faster
                multi-block writes, and probably also reduces flash wear on the media.
                */
                /* Send CMD55 */
                _DRV_SDCARD_CommandSend ( object, DRV_SDCARD_APP_CMD, 0x00 );

		/* We send the command, now wait for response */
                if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
                {
                    /* Check if successful */
                    if( dObj->cmdResponse.response1.byte == 0x00 )
                    {
                        dObj->taskState = DRV_SDCARD_TASK_ERASE_TO_WRITE;
                    }
		}
            }
            break;
        case DRV_SDCARD_TASK_ERASE_TO_WRITE:
            /* Send ACMD23 TODO preEraseBlockCount un initialized */
            _DRV_SDCARD_CommandSend ( object, DRV_SDCARD_SET_WR_BLK_ERASE_COUNT,
                    preEraseBlockCount );

            if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
            {
                    /* We send the command, now wait for response */
                    dObj->taskState = DRV_SDCARD_TASK_MODIFY_ADDRESS;
            }
            break;
        case DRV_SDCARD_TASK_MODIFY_ADDRESS:
            /* The info->dwAddress parameter is the block address. For
            standard capacity SD cards, the card expects a complete byte
            address. To convert the block address into a byte address, we
            multiply by the block size (512). For SDHC (high capacity)
            cards, the card expects a block address already, so no address
            cconversion is needed
            */
            if ( dObj->sdCardType == DRV_SDCARD_MODE_NORMAL )
            {
                lObj->sectorAddress *= _DRV_SDCARD_MEDIA_BLOCK_SIZE;
            }
            dObj->taskState = DRV_SDCARD_TASK_SEND_WRITE_COMMAND;
            break;
        case DRV_SDCARD_TASK_SEND_WRITE_COMMAND:
            /* Send the write single or write multi command, with the LBA or byte
            address (depeding upon SDHC or standard capacity card) */
            _DRV_SDCARD_CommandSend ( object, lObj->command, lObj->sectorAddress );

            if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
            {
                dObj->taskState = DRV_SDCARD_TASK_CHECK_WRITE_STATUS;
            }
            break;
        case DRV_SDCARD_TASK_CHECK_WRITE_STATUS:
            if( dObj->cmdResponse.response1.byte == 0x00 )
            {
                dObj->taskState = DRV_SDCARD_TASK_SEND_WRITE_PACKET;
            }
            else
            {
                dObj->taskState = DRV_SDCARD_TASK_STATE_ERROR;
            }
            break;
        case DRV_SDCARD_TASK_SEND_WRITE_PACKET:
            /* Check if we just finished programming a block, or we are starting
            for the first time.  In this case, we need to send the data start token */
            if( lObj->blockCounter == _DRV_SDCARD_MEDIA_BLOCK_SIZE )
            {
                /* Send the correct data start token, based on the type of write
                we are doing */
                if ( lObj->command == DRV_SDCARD_WRITE_MULTI_BLOCK )
                {
                    dObj->packetArray[ 0 ] = DRV_SDCARD_DATA_START_MULTI_BLOCK_TOKEN;
                }
                else
                {
                    /*Else it must be a single block write */
                    dObj->packetArray[ 0 ] = DRV_SDCARD_DATA_START_TOKEN;
                }
                dObj->spiBufferHandle = DRV_SPI_BufferAddWrite ( dObj->spiClientHandle, dObj->packetArray, 1 );
            }

                /* Check for special command */
            dObj->taskState = DRV_SDCARD_TASK_SEND_RAW_DATA_WRITE;
            break;
        case DRV_SDCARD_TASK_SEND_RAW_DATA_WRITE:
            /* Now send a packet of raw data bytes to the media, over SPI.
            This code directly impacts data thoroughput in a significant way.
            Special care should be used to make sure this code is speed optimized.
            */
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus ( dObj->spiBufferHandle ) )
            {
                dObj->spiBufferHandle = DRV_SPI_BufferAddWrite ( dObj->spiClientHandle, lObj->buffer, 512 );

                dObj->taskState = DRV_SDCARD_TASK_SEND_CRC;
            }
            break;
        case DRV_SDCARD_TASK_SEND_CRC:
            /* Check if we have finshed sending a 512 byte block.  If so,
            need to receive 16-bit CRC, and retrieve the data_response token
            */
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus ( dObj->spiBufferHandle ) )
            {
                lObj->blockCounter = _DRV_SDCARD_MEDIA_BLOCK_SIZE;    //Re-initialize counter

                /* Add code to compute CRC, if using CRC. By default, the media
                doesn't use CRC unless it is enabled manually during the card
                initialization sequence.
                */
                /* Send 16-bit CRC for the data block just sent, TODO.. check this */
                dObj->spiBufferHandle = DRV_SPI_BufferAddWrite ( dObj->spiClientHandle, dObj->mediaInitArray, 2 );

                dObj->taskState = DRV_SDCARD_TASK_WRITE_RESPONSE_GET;
            }
            break;
        case DRV_SDCARD_TASK_WRITE_RESPONSE_GET:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus
                    ( dObj->spiBufferHandle ) )
            {
                dObj->spiBufferHandle = DRV_SPI_BufferAddRead ( dObj->spiClientHandle, dObj->packetArray , 1 );

                dObj->taskState = DRV_SDCARD_TASK_WRITE_RESPONSE_GET_TOKEN_MASK;
            }
            break;
        case DRV_SDCARD_TASK_WRITE_RESPONSE_GET_TOKEN_MASK:
			if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus
            	( dObj->spiBufferHandle ) )
            {
                /* Read response token byte from media, mask out top three don't
                care bits, and check if there was an error */
                if( ( dObj->packetArray [ 0 ] & DRV_SDCARD_WRITE_RESPONSE_TOKEN_MASK )
                        != DRV_SDCARD_DATA_ACCEPTED )
                {
                    /* Something went wrong.  Try and terminate as gracefully as
                    possible, so as allow possible recovery */
                    dObj->taskState = DRV_SDCARD_TASK_WRITE_ABORT;
                }
                /* The media will now send busy token (0x00) bytes until
                it is internally ready again (after the block is successfully
                writen and the card is ready to accept a new block).
                */
                dObj->taskState = DRV_SDCARD_TASK_MEDIA_BUSY;
                }
                break;
	case DRV_SDCARD_TASK_MEDIA_BUSY:
            /* Dummy read to gobble up a byte (ex: to ensure we meet NBR timing parameter  */
            dObj->spiBufferHandle = DRV_SPI_BufferAddRead ( dObj->spiClientHandle, dObj->packetArray, 1 );

            dObj->taskState = DRV_SDCARD_WRITE_DUMMY_READ;
            break;
        case DRV_SDCARD_WRITE_DUMMY_READ:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus
                        ( dObj->spiBufferHandle ) )
            {
                /* Check whether the card is still busy */
                dObj->spiBufferHandle = DRV_SPI_BufferAddRead ( dObj->spiClientHandle, dObj->packetArray, 1 );

                /* Get the response in another state */
                dObj->taskState = DRV_SDCARD_WRITE_CHECK_STILL_BUSY;
            }
            break;
        case DRV_SDCARD_WRITE_CHECK_STILL_BUSY:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus
                        ( dObj->spiBufferHandle ) )
            {
                if ( dObj->packetArray[0] != 0x00 )
                {
                    /* The media is done and is no longer busy.  Go ahead and
                    either send the next packet of data to the media, or the stop
                    token if we are finshed.
                    */
                    if ( lObj->command == DRV_SDCARD_WRITE_MULTI_BLOCK )
                    {
                        dObj->packetArray[0] = DRV_SDCARD_DATA_STOP_TRAN_TOKEN;

                        /* Check whether the card is still busy */
                        dObj->spiBufferHandle = DRV_SPI_BufferAddWrite ( dObj->spiClientHandle,
                            dObj->packetArray, 1 );

                        /* Get the response in another state */
                        dObj->taskState = DRV_SDCARD_WRITE_STOP_TRAN_CMPLT;
                    }
                    else
                    {
                        /* Send eight clock pulses */
                        dObj->spiBufferHandle = DRV_SPI_BufferAddWrite ( dObj->spiClientHandle,
                            dObj->mediaInitArray, 1 );

                        /* Get the response in another state */
                        dObj->taskState = DRV_SDCARD_WRITE_CHECK_8CLOCKS;
                    }
                }
                else
                {
                    /* Still busy */
                    dObj->taskState = DRV_SDCARD_WRITE_DUMMY_READ;
                }
            }
            break;
        case DRV_SDCARD_WRITE_CHECK_STILL_BUSY_SEND_CLOCKS:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus
                        ( dObj->spiBufferHandle ) )
            {
                /* Get the response in another state */
                dObj->taskState = DRV_SDCARD_WRITE_SEND_8CLOCKS;

                /* Send eight clock pulses */
                dObj->spiBufferHandle = DRV_SPI_BufferAddWrite ( dObj->spiClientHandle,
                    dObj->mediaInitArray, 1 );
            }
            break;
        case DRV_SDCARD_WRITE_STOP_TRAN_CMPLT:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus
                        ( dObj->spiBufferHandle ) )
            {
                /* Send eight clock pulses */
                dObj->spiBufferHandle = DRV_SPI_BufferAddWrite ( dObj->spiClientHandle,
                       /* dObj->mediaInitArray*/testVariable2, 10 );

                /* Get the response in another state */
                dObj->taskState = DRV_SDCARD_WRITE_STOP_MULTIPLE;
            }
            break;
        case DRV_SDCARD_WRITE_STOP_MULTIPLE:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus
                        ( dObj->spiBufferHandle ) )
            {
                /* Check whether the card is still busy */
                dObj->spiBufferHandle = DRV_SPI_BufferAddRead ( dObj->spiClientHandle, dObj->packetArray, 1 );

                 /* Get the response in another state */
                dObj->taskState = DRV_SDCARD_WRITE_STOP_MULTIPLE_STATUS;
            }
            break;
         case DRV_SDCARD_WRITE_STOP_MULTIPLE_STATUS:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus
                ( dObj->spiBufferHandle ) )
            {
                /* We already sent the stop transmit token for the multi-block write
                operation.  Now all we need to do, is keep waiting until the card
                signals it is no longer busy.  Card will keep sending 0x00 bytes
                until it is no longer busy.
                */
                if ( dObj->packetArray == 0x00 )
                {
                    dObj->taskState = DRV_SDCARD_WRITE_COMPLETE;
                }
            }
             break;
        case DRV_SDCARD_WRITE_CHECK_8CLOCKS:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus
                ( dObj->spiBufferHandle ) )
            {
                if ( (lObj->sectorCount--) > 1 )
                {
                    lObj->buffer        += 512;
                    ++(lObj->sectorAddress);
                    dObj->taskState = DRV_SDCARD_TASK_PROCESS_WRITE;
                }
                else
                {
                    /* Update the buffer status as 'processing in progress' */
                    *dObj->bufferStatus = SYS_FS_MEDIA_BUFFER_COMPLETED;

                    /* Un select the chip */
                    _DRV_SDCARD_CHIP_DESELECT ( dObj->chipSelectPort,
                    dObj->chipSelectBitPosition );

                    dObj->taskState = DRV_SDCARD_WRITE_COMPLETE;
                }
		}
            break;
        case DRV_SDCARD_WRITE_COMPLETE:
                dObj->taskState = DRV_SDCARD_TASK_PROCESS_QUEUE;
                break;
        /* These are error cases.  */
        case DRV_SDCARD_TASK_READ_ABORT:
        case DRV_SDCARD_TASK_STATE_ERROR:
        case DRV_SDCARD_WRITE_SEND_8CLOCKS:
        case DRV_SDCARD_TASK_WRITE_ABORT:
        case DRV_SDCARD_TASK_READ_DATA_READ_CMPLTE:
            Nop();
            break;
    }
} /* DRV_SDCARD_Tasks */


//******************************************************************************
/* Function:
    DRV_HANDLE DRV_SDCARD_Open( const SYS_MODULE_INDEX  drvIndex,
                                const DRV_IO_INTENT     ioIntent )

  Summary:
    Opens the specific module instance and returns a handle

  Description:
    This routine opens a driver for use by any client module and provides a
    handle that must be provided to any of the other driver operations to
    identify the caller and the instance of the driver/hardware module.

  Parameters:
    drvIndex        - Identifier for the instance to be initialized
    ioIntent        - Possible values from the enumeration DRV_IO_INTENT

  Returns:
    If successful, the routine returns a valid open-instance handle (a number
    identifying both the caller and the module instance)
    If an error occurs, the return value is DRV_HANDLE_INVALID
*/

DRV_HANDLE  DRV_SDCARD_Open ( const SYS_MODULE_INDEX drvIndex,
                              const DRV_IO_INTENT ioIntent )
{

    DRV_SDCARD_CLIENT_OBJ  	*clientObj      =
    						( DRV_SDCARD_CLIENT_OBJ* ) gDrvSDCARDClientObj;
    DRV_SDCARD_OBJ         	*dObj;
    unsigned int 			iClient;
    bool 					foundIt 		= false;

    /* Validate the driver index */
    /* If there is anything specific to the module & needs to be checked, should
     be handled in this section with an || condition.
     May be something like ioIntent test for Exclusive access */
    if ( drvIndex >= DRV_SDCARD_INDEX_COUNT )
    {
        return DRV_HANDLE_INVALID;
    }

    /* Allocate the driver object and set the operation flag to be in use */
    dObj = ( DRV_SDCARD_OBJ* ) _DRV_SDCARD_INSTANCE_GET ( drvIndex );

    if( dObj->status != SYS_STATUS_READY )
    {
        /* The SDCARD module should be ready */
        SYS_ASSERT ( false, "Was the driver initialized?" );
        return DRV_HANDLE_INVALID;
    }

    /* Check for exclusive access */
    if ( ( dObj->numClients > 0 ) && DRV_IO_ISEXCLUSIVE ( ioIntent ) )
    {
		/* Set that the hardware instance is opened in exclusive mode */
		return ( DRV_HANDLE_INVALID ) ;
    }

    /* Setup client operations */
//    if (OSAL_MUTEX_Lock(dObj->mutexSDCARD_clientHandle , OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        /* Find available slot in array of client objects */
        for (iClient = 0; iClient < DRV_SDCARD_CLIENTS_NUMBER ; iClient++)
        {
            if ( !clientObj->inUse )
            {
                clientObj->inUse  = true;
                clientObj->drvIndex = drvIndex;
                foundIt = true;
                break;
            }
            clientObj++;
        }
        /* TODO OSAL - Unlock Mutex */
        //OSAL_ASSERT( (OSAL_MUTEX_Unlock(dObj->mutexSDCARD_clientHandle) == OSAL_RESULT_TRUE),
        //			 "Unable to unlock client handle mutex" );
    }
    if ( !foundIt )
    {
        /* Couldn't find open slot in object array */
        //SYS_ASSERT(false, "Insufficient Clients. Increase the number of clients
        // TODO	via DRV_SDCARD_CLIENTS_NUMBER");

        return  DRV_HANDLE_INVALID ;
    }

    /* Increment the number of clients connected to the hardware instance */
    ( dObj->numClients )++;

    /* Increment the client in case of Multi client support, otherwise remove
     the below statement */
    clientObj->driverObject = ( DRV_SDCARD_OBJ_HANDLE ) dObj;

    /* Check if max number of clients open */
    if( dObj->numClients > DRV_SDCARD_CLIENTS_NUMBER )
    {
        /* Set that the hardware instance is opened with max clients */
        return DRV_HANDLE_INVALID;
    }
    /* Update the Client Status */
    clientObj->status = DRV_SDCARD_CLIENT_STATUS_READY;

    /* Return the client object */
    return ( ( DRV_HANDLE ) iClient );

} /* DRV_SDCARD_Open */


//******************************************************************************
/* Function:
    void DRV_SDCARD_Close( DRV_HANDLE handle )

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

void DRV_SDCARD_Close( DRV_HANDLE handle )
{
    /* Multi client variables are removed from single client builds. */
    DRV_SDCARD_CLIENT_OBJ   *clientObj =
        ( DRV_SDCARD_CLIENT_OBJ* ) _DRV_SDCARD_CLIENT_OBJ_GET ( handle );

    clientObj->inUse = false;

    /* Update the Client Status */
    clientObj->status = DRV_SDCARD_CLIENT_STATUS_INVALID ;

} /* DRV_SDCARD_Close */


//******************************************************************************
/* Function:
    DRV_SDCARD_CLIENT_STATUS DRV_SDCARD_ClientStatus(DRV_HANDLE handle)

  Summary:
    Gets the status of the module instance associated with the handle

  Description:
    This routine gets the status of the module instance associated with the
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    DRV_SDCARD_CLIENT_STATUS value describing the current status of the driver
*/

DRV_SDCARD_CLIENT_STATUS DRV_SDCARD_ClientStatus( DRV_HANDLE handle )
{
    DRV_SDCARD_CLIENT_OBJ       *clientObj =
        ( DRV_SDCARD_CLIENT_OBJ* ) _DRV_SDCARD_CLIENT_OBJ_GET ( handle );

    /* Return the client status associated with the handle passed */
    return ( ( clientObj->status ) ) ;

} /* DRV_SDCARD_ClientStatus */


//******************************************************************************
/* Function:
    bool _DRV_SDCARD_MediaCommandDetect ( SYS_MODULE_OBJ object )

  Summary:
    Determines whether an SD card is present using a command response method.

  Description:
    This routine determines whether an SD card is present using a command
    response method. If it is a Micro SD card(doesn't has a card detect pin)
    calling this API is the only option detect the presence of the card. This API
    could be called directly or DRV_SDCARD_MediaDetect function will call this
    API based on the configuration.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    true - The Card is present.
    false - The Card is not present.
*/

bool _DRV_SDCARD_MediaCommandDetect ( SYS_MODULE_OBJ object )
{
    volatile unsigned char          clearBuffer;

    /* Get the driver object */
    DRV_SDCARD_OBJ *dObj = ( DRV_SDCARD_OBJ* ) _DRV_SDCARD_INSTANCE_GET ( object );


    /* Check what state we are in, to decide what to do */
    switch (dObj->cmdDetectState)
    {
        case DRV_SDCARD_INIT_START_INIT:
            /* If the SPI module is not enabled, then the media has evidently not
            been initialized.  Try to send CMD0 and CMD13 to reset the device and
            get it into SPI mode (if present), and then request the status of
            the media.  If this times out, then the card is presumably not
            physically present */
            PLIB_SPI_BaudRateSet( _DRV_SDCARD_PERIPHERAL_ID_GET( dObj->spiId ),
                            SYS_CLK_PeripheralFrequencyGet( CLK_BUS_PERIPHERAL_1 ),
                              _DRV_SDCARD_SPI_INITIAL_SPEED );

            dObj->cmdDetectState = DRV_SDCARD_INIT_CHECK_FOR_CARD;
            break;

        case DRV_SDCARD_INIT_CHECK_FOR_CARD:
            /* Send CMD0 to reset the media. If the card is physically present,
            then we should get a valid response. Toggle chip select, to make
            media abandon whatever it may have been doing before.  This ensures
            the CMD0 is sent freshly after CS is asserted low, minimizing risk
            of SPI clock pulse master/slave syncronization problems, due to
            possible application noise on the SCK line. */
            _DRV_SDCARD_CHIP_DESELECT ( dObj->chipSelectPort,
                dObj->chipSelectBitPosition );

            /* Send some "extraneous" clock pulses.  If a previous command was
            terminated before it completed normally, the card might not have
            received the required clocking following the transfer. */
            PLIB_SPI_BufferWrite( dObj->spiId, 0xFF );

            /* Wait in the next state till it gets a receve interrupt */
            dObj->cmdDetectState = DRV_SDCARD_INIT_CLEAR_RCV_BUFFER;
            break;
        case DRV_SDCARD_INIT_CLEAR_RCV_BUFFER:
            if ( true == PLIB_SPI_ReceiverBufferIsFull ( dObj->spiId ) )
            {
                /* We send a dummy data to send clock pulses. Clear the receive
                 interrupt caused due to that. */
                clearBuffer = PLIB_SPI_BufferRead ( dObj->spiId );

                /* Assert the chip select pin */
                _DRV_SDCARD_CHIP_SELECT ( dObj->chipSelectPort,
                    dObj->chipSelectBitPosition );

                /* Send CMD0 to software reset the device */
                _DRV_SDCARD_CommandSend ( object, DRV_SDCARD_GO_IDLE_STATE, 0x00 );

                if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
                {
                    /* We send the command, now wait for response */
                    dObj->cmdDetectState = DRV_SDCARD_INIT_WAIT_FOR_R1_RESPONSE;
                }
            }
            break;
        case DRV_SDCARD_INIT_WAIT_FOR_R1_RESPONSE:
            if ( dObj->cmdResponse.response1.byte != CMD_R1_END_BIT_SET )
            {
                /* We send the command, now wait for response */
                dObj->cmdDetectState = DRV_SDCARD_INIT_CHECK_FOR_CARD;

                /* Check if response was invalid (R1 response byte should be
                    = 0x01 after CMD_GO_IDLE_STATE) */
                return false;
            }
            else
            {
                /* Card is presumably present.  The SDI pin should have a
                pull up resistor on it, so the odds of SDI "floating" to 0x01
                after sending CMD0 is very remote, unless the media is genuinely
                present. Therefore, we should try to perform a full card
                initialization sequence now. */
                _DRV_SDCARD_MediaInitialize ( object );

                if ( dObj->mediaInitState == DRV_SDCARD_MEDIA_STATE_INITIALIZATION_COMPLETE )
                {
                    if ( dObj->mediaInformation.errorCode == SYS_FS_MEDIA_NO_ERROR )
                    {
                        /* Now the card is attached to the system. We need not do this
                        every time we come here. Check for detach */
                        dObj->cmdDetectState = DRV_SDCARD_INIT_CHECK_FOR_DETACH;

                        /* if the card was initialized correctly, it means it is
                        present */
                        return true;
                    }
                }
                else
                {
                    //CloseSPIM();
                    return false;
                }
            }
            break;
        case DRV_SDCARD_INIT_CHECK_FOR_DETACH:
            /* The SPI module was already enabled.  This most likely means the
             media is present and has already been initialized.  However, it is
             possible that the user could have unplugged the media, in which case
             it is no longer present.  We should send it a command, to check the
             status. */
            _DRV_SDCARD_CommandSend ( object, DRV_SDCARD_SEND_STATUS, 0x00 );

            /* Change from this state only on completion of command execution */
            if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
            {
                /* For status command SD card will respond with R2 type packet */
                dObj->cmdDetectState = DRV_SDCARD_INIT_WAIT_FOR_R2_RESPONSE;
            }
            break;
        case DRV_SDCARD_INIT_WAIT_FOR_R2_RESPONSE:
            if(( dObj->cmdResponse.response2.word & 0xEC0C ) != 0x0000 )
            {
                /* The card didn't respond with the expected result.  This probably
                means it is no longer present.  We can try to re-initialize it,
                just to be doubly sure. */
                //CloseSPIM();

                /* Can block and take a long time to execute. */
                _DRV_SDCARD_MediaInitialize ( object );

                dObj->cmdDetectState = DRV_SDCARD_INIT_MEDIA_ERROR_CHECK;
            }
            else
            {
                /* Now the card is attached, check for detach on next call */
                dObj->cmdDetectState = DRV_SDCARD_INIT_CHECK_FOR_DETACH;

                /* if the card was initialized correctly, it means it is present */
                return true;
            }
            break;
        case DRV_SDCARD_INIT_MEDIA_ERROR_CHECK:
            if( dObj->mediaInformation.errorCode == SYS_FS_MEDIA_NO_ERROR )
            {
                if ( dObj->mediaInitState == DRV_SDCARD_MEDIA_STATE_INITIALIZATION_COMPLETE )
                {
                    /* Now the card is attached, check for detach on next call */
                    dObj->cmdDetectState = DRV_SDCARD_INIT_CHECK_FOR_DETACH;

                    /* if the card was initialized correctly, it means it is present */
                    return true;
                }
            }
            /* Set to the state which is right after baud rate initialization */
            //TODOdObj->cmdDetectState = DRV_SDCARD_INIT_CHECK_FOR_CARD;
            break;
        default:

            //SYS_ASSERT("Unknown state");
            break;
        }
    /* Should theoretically never execute to here.  All pathways should have
    already returned with the status. */
    return false;

}/* DRV_SDCARD_MediaDetect */


//******************************************************************************
/* Function:
    void _DRV_SDCARD_CommandSend ( DRV_HANDLE handle, DRV_SDCARD_COMMANDS command,
                                uint32_t address )

  Summary:
    Sends a command.

  Description:
    This routine sends a command to the SD card. The response will be updated in
    the driver instance object. The code is written in an event driven method.
    The user is required to call this API multiple times till the status becomes
    'complete'.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's
                   open routine.

    command     - Command to send.

    address     - If there is an address associated with the command. If there
                    is no address associated, then pass '0'.

  Returns:
    None

  Remarks:
    It is expected to call this routine continuously until we get the status as
    'execution successful'. The caller should not execute this function again
    after the execution of the command. It will cause to execute the same command
    again.
*/

void _DRV_SDCARD_CommandSend ( SYS_MODULE_OBJ object, DRV_SDCARD_COMMANDS command,
                                uint32_t address )
{
    DRV_SDCARD_OBJ                  *dObj       =
        ( DRV_SDCARD_OBJ* ) _DRV_SDCARD_INSTANCE_GET( object );
    uint8_t                         endianArray[4];
    volatile uint8_t                dummyRead= 0;
    uint32_t *tempPtr;

    tempPtr = (uint32_t*)endianArray;
    /* TODO- check for SPI_DRIVER open here */
    /* Check what state we are in, to decide what to do */
    if ( DRV_SDCARD_STOP_TRANSMISSION == command )
    {
        dummyRead = 1;
    }
    if (object != 0)
    {
        dummyRead = 0;
    }
    switch ( dObj->cmdState )
    {
        case DRV_SDCARD_CMD_FRAME_PACKET:
            /* Before start sending the command, assert the chip select */
            _DRV_SDCARD_CHIP_SELECT ( dObj->chipSelectPort,
                dObj->chipSelectBitPosition );

            /* SD card follows big-endian format */
            *tempPtr = *(( uint32_t* )&address);

            /* Form the packet */
            dObj->packetArray[0] = ( gDrvSDCARDCmdTable[command].commandCode | DRV_SDCARD_TRANSMIT_SET );
            dObj->packetArray[1] = endianArray[3];
            dObj->packetArray[2] = endianArray[2];
            dObj->packetArray[3] = endianArray[1];
            dObj->packetArray[4] = endianArray[0];
            dObj->packetArray[5] = gDrvSDCARDCmdTable[command].crc;
            /* Dummy data. Only used in case of DRV_SDCARD_STOP_TRANSMISSION */
            dObj->packetArray[6] = 0xFF;

            /* Framing the packet is complete. Now send it to the card */
            dObj->cmdState = DRV_SDCARD_CMD_SEND_PACKET;
            break;
        case DRV_SDCARD_CMD_SEND_PACKET:

            dObj->spiBufferHandle = DRV_SPI_BufferAddWrite ( dObj->spiClientHandle, dObj->packetArray,
                DRV_SDCARD_PACKET_SIZE );

            /* Do an extra read for this command */
            if ( command != DRV_SDCARD_STOP_TRANSMISSION )
            {
                dObj->cmdState = DRV_SDCARD_CMD_CHECK_TRANSFER_COMPLETE;
            }
            else
            {
                /* TODO this logic will not work if BufferAdd returns a buffer
                 Handle */
                dObj->cmdState = DRV_SDCARD_CMD_CHECK_SPL_CASE;
            }
            break;
        case DRV_SDCARD_CMD_CHECK_SPL_CASE:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus ( dObj->spiBufferHandle ) )
            {
                /* Do a dummy read in case of special command */
                dObj->spiBufferHandle = DRV_SPI_BufferAddRead ( dObj->spiClientHandle, &dObj->packetArray[0], 1 );

                dObj->cmdState = DRV_SDCARD_CMD_CHECK_TRANSFER_COMPLETE;
            }
            break;
        case DRV_SDCARD_CMD_CHECK_TRANSFER_COMPLETE:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus ( dObj->spiBufferHandle ) )
            {
                dObj->spiBufferHandle = DRV_SPI_BufferAddRead ( dObj->spiClientHandle,
                        &dObj->cmdResponse.response1.byte, 1 );

                /* Act as per the response type */
                dObj->cmdState = DRV_SDCARD_CMD_CHECK_RESP_TYPE;
            }
            break;
        case DRV_SDCARD_CMD_CHECK_RESP_TYPE:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus ( dObj->spiBufferHandle ) )
            {
                if (dObj->cmdResponse.response1.byte == DRV_SDCARD_MMC_FLOATING_BUS)
                {
                        dObj->cmdState = DRV_SDCARD_CMD_CHECK_TRANSFER_COMPLETE;
                }
                else
                {
                    /* Check if we should read more bytes, depending upon
                     the response type expected */
                    if ( gDrvSDCARDCmdTable[command].responseType == RESPONSE_R1 )
                    {
                        /* Device requires at least 8 clock pulses after the response
                        has been sent, before if can process the next command.
                        CS may be high or low */
                        dObj->spiBufferHandle = DRV_SPI_BufferAddWrite ( dObj->spiClientHandle,
                            &dObj->dataToSendClockPulses, _DRV_SDCARD_SEND_8_CLOCKS );

                        dObj->cmdState = DRV_SDCARD_CMD_EXEC_CHECK_COMPLETION;
                    }
                    else if ( gDrvSDCARDCmdTable[command].responseType == RESPONSE_R2 )
                    {
                        /* We already received the first byte, just make sure it is in the
                        correct location in the struct. */
                        dObj->cmdResponse.response2.byte1 = dObj->cmdResponse.response1.byte;

                        /* Fetch the second byte of the response. */
                        dObj->spiBufferHandle = DRV_SPI_BufferAddRead ( dObj->spiClientHandle, &dObj->cmdResponse.response2.byte1,
                                        1 );
                        dObj->cmdState = DRV_SDCARD_CMD_HANDLE_R2_RESPONSE;
                    }
                    else if ( gDrvSDCARDCmdTable[command].responseType == RESPONSE_R1b )
                    {
                        /* Expected response it of R2 type */
                        dObj->cmdState = DRV_SDCARD_CMD_HANDLE_R1B_RESPONSE;
                    }
                    else if ( gDrvSDCARDCmdTable[command].responseType == RESPONSE_R7 )
                    {
                        /* Fetch the other four bytes of the R3 or R7 response.
                        Note: The SD card argument response field is 32-bit, big endian format.
                        However, the C compiler stores 32-bit values in little endian in RAM.
                        When writing to the bytes, make sure the order it
                        gets stored in is correct. */
                        dObj->spiBufferHandle = DRV_SPI_BufferAddRead ( dObj->spiClientHandle,
                            dObj->packetArray, 4 );

                        /* Expected response it of R7 type */
                        dObj->cmdState = DRV_SDCARD_CMD_HANDLE_R7_RESPONSE;
                    }
                }
            }
            break;
        case DRV_SDCARD_CMD_HANDLE_R1B_RESPONSE:
            /* Keep trying to read from the media, until it signals it is no longer
            busy.  It will continuously send 0x00 bytes until it is not busy.
            A non-zero value means it is ready for the next command.
            The R1b response is received after a CMD12,  CMD_STOP_TRANSMISSION
            command, where the media card may be busy writing its internal buffer
            to the flash memory.  This can typically take a few milliseconds,
            with a recommended maximum timeout of 250ms or longer for SD cards.
            */
            dObj->spiBufferHandle = DRV_SPI_BufferAddRead ( dObj->spiClientHandle,
                        (uint8_t*)&dObj->cmdResponse.response1.byte, 1 );

            dObj->cmdState = DRV_SDCARD_CMD_R1B_READ_BACK;
            break;
	case DRV_SDCARD_CMD_R1B_READ_BACK:
                if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus ( dObj->spiBufferHandle ) )
                {
                        if ( dObj->cmdResponse.response1.byte != 0x00 )
                        {
                                /* Device requires at least 8 clock pulses after the response
                                has been sent, before if can process the next command.
					CS may be high or low */
					dObj->spiBufferHandle = DRV_SPI_BufferAddWrite ( dObj->spiClientHandle,
					    &dObj->dataToSendClockPulses, _DRV_SDCARD_SEND_8_CLOCKS );

					dObj->cmdState = DRV_SDCARD_CMD_EXEC_CHECK_COMPLETION;
				}
			dObj->cmdState = DRV_SDCARD_CMD_R1B_READ_BACK;
			}
            break;
        case DRV_SDCARD_CMD_HANDLE_R2_RESPONSE:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus ( dObj->spiBufferHandle ) )
            {
                /* Device requires at least 8 clock pulses after the response
                has been sent, before if can process the next command.
                CS may be high or low */
                dObj->spiBufferHandle = DRV_SPI_BufferAddWrite ( dObj->spiClientHandle,
                    &dObj->dataToSendClockPulses, _DRV_SDCARD_SEND_8_CLOCKS );

                dObj->cmdState = DRV_SDCARD_CMD_EXEC_CHECK_COMPLETION;
            }
            break;
       case DRV_SDCARD_CMD_HANDLE_R7_RESPONSE:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus( dObj->spiBufferHandle ) )
            {
                /* Handle the endianness */
                dObj->cmdResponse.response7.bytewise.argument.ocrRegisterByte0 =
                    dObj->packetArray[3];
                dObj->cmdResponse.response7.bytewise.argument.ocrRegisterByte1 =
                    dObj->packetArray[2];
                dObj->cmdResponse.response7.bytewise.argument.ocrRegisterByte2 =
                    dObj->packetArray[1];
                dObj->cmdResponse.response7.bytewise.argument.ocrRegisterByte3 =
                    dObj->packetArray[0];

                /* Device requires at least 8 clock pulses after the response
                has been sent, before if can process the next command.
                CS may be high or low */
                dObj->spiBufferHandle = DRV_SPI_BufferAddWrite ( dObj->spiClientHandle,
                    &dObj->dataToSendClockPulses, _DRV_SDCARD_SEND_8_CLOCKS );

                dObj->cmdState = DRV_SDCARD_CMD_EXEC_CHECK_COMPLETION;
            }
            break;
       case DRV_SDCARD_CMD_EXEC_CHECK_COMPLETION:
           if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus( dObj->spiBufferHandle ) )
           {
                /* Command complete. if no more data is expected, de-select the chip */
                if ( DRV_SDCARD_GET_NODATA == gDrvSDCARDCmdTable[command].moredataexpected )
                {
                    _DRV_SDCARD_CHIP_DESELECT ( dObj->chipSelectPort,
                    dObj->chipSelectBitPosition );
                }

                dObj->cmdState = DRV_SDCARD_CMD_CONFIRM_COMPLETE;
           }
           break;
        case DRV_SDCARD_CMD_CONFIRM_COMPLETE:
            /* This code will be the first case statement getting executed on calling
             _DRV_SDCARD_CommandSend function, except first time */
            dObj->cmdState = DRV_SDCARD_CMD_EXEC_IS_COMPLETE;
            break;
        case DRV_SDCARD_CMD_EXEC_IS_COMPLETE:
            /* This code will be the first case statement getting executed on calling
             _DRV_SDCARD_CommandSend function, except first time */
            dObj->cmdState = DRV_SDCARD_CMD_FRAME_PACKET;
            break;
        default:
            //SYS_ASSERT ( "Invalid data" );
            break;
    }
}


//******************************************************************************
/* Function:
    void _DRV_SDCARD_MediaInitialize ( SYS_MODULE_OBJ object )

  Summary:
    Initializes the SD card.

  Description:
    The function updates MEDIA_INFORMATION structure.  The
    errorCode member may contain the following values:
        * MEDIA_NO_ERROR - The media initialized successfully
        * MEDIA_CANNOT_INITIALIZE - Cannot initialize the media.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's
                   open routine.

  Returns:
    None
*/

void _DRV_SDCARD_MediaInitialize ( SYS_MODULE_OBJ object )
{
    /* Get the driver object */
    DRV_SDCARD_OBJ *dObj = ( DRV_SDCARD_OBJ* ) _DRV_SDCARD_INSTANCE_GET ( object );
    uint8_t index, cSizeMultiplier, blockLength;
    uint16_t cSize;
    int i;

    /* Check what state we are in, to decide what to do */
    switch ( dObj->mediaInitState )
    {
        case DRV_SDCARD_MEDIA_STATE_INIT_CHIP_SELECT:
			dObj->mediaInformation.errorCode = SYS_FS_MEDIA_NO_ERROR;
			dObj->mediaInformation.validityFlags.value = 0;
			dObj->discCapacity = 0;
			dObj->sdCardType = DRV_SDCARD_MODE_NORMAL;

			/* Keep the chip select high(not selected) to send clock pulses  */
            _DRV_SDCARD_CHIP_DESELECT ( dObj->chipSelectPort,
                dObj->chipSelectBitPosition );

			/* 400kHz. Initialize SPI port to <= 400kHz */
			_DRV_SDCARD_SET_SPEED ( _DRV_SDCARD_PERIPHERAL_ID_GET( dObj->spiId ),
                SYS_CLK_PeripheralFrequencyGet( CLK_BUS_PERIPHERAL_1 ),
                _DRV_SDCARD_SPI_INITIAL_SPEED );


            /* No conditions here. Move to the next level */
            dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_INIT_DELAY_INITIAL;
            break;
        case DRV_SDCARD_MEDIA_STATE_INIT_DELAY_INITIAL:
            /* Before start sending the command, assert the chip select */
            /* Media wants the longer of: Vdd ramp time, 1 ms fixed delay, or 74+
            clock pulses. According to spec, CS should be high during the 74+ clock
            pulses. In practice it is preferrable to wait much longer than 1ms,
            in case of contact bounce, or incomplete mechanical insertion (by the
            time we start accessing the media).
            */
            for(i = 0; i<= 0xFFFB; i++);

            /* Keep the chip select high(not selected) to send clock pulses  */
            _DRV_SDCARD_CHIP_DESELECT ( dObj->chipSelectPort,
                dObj->chipSelectBitPosition );

            /* TODO Add delay using timer system service */
            dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_INIT_ARRAY_INIT;
            break;
        case DRV_SDCARD_MEDIA_STATE_INIT_ARRAY_INIT:
             /* Initialize the array with all 0xFF */
            for ( index=0; index< MEDIA_INIT_ARRAY_SIZE; index++ )
            {
                dObj->mediaInitArray[ index ] = 0xFF;
            }
            dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_INIT_SEND_CLOCK_PULSES;
            break;
        case DRV_SDCARD_MEDIA_STATE_INIT_SEND_CLOCK_PULSES:
             /* Send at least 74 clock pulses with chip select high */
             dObj->spiBufferHandle = DRV_SPI_BufferAddWrite ( dObj->spiClientHandle, ( uint8_t* ) dObj->mediaInitArray,
                 MEDIA_INIT_ARRAY_SIZE );
             /* Check the status of completion in the next state */
             dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_CHECK_FOR_80_PULSES;
            break;
        case DRV_SDCARD_MEDIA_STATE_CHECK_FOR_80_PULSES:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE == DRV_SPI_BufferStatus ( dObj->spiBufferHandle ) )
            {
                /* Change the state  */
                if ( dObj->mediaInitindex >= _DRV_SDCARD_SEND_80_CLOCKS )
                {
                    dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_TOGGLE_CHIPSELECT;
                }
                else
                {
                    /* Keep sending clocks if it is not 80 yet */
                    dObj->mediaInitindex++;
                    dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_INIT_SEND_CLOCK_PULSES;
                }
            }
            break;
        case DRV_SDCARD_MEDIA_STATE_TOGGLE_CHIPSELECT:
            /* The chip select is already high when we come here for the first time.
            Doing it again becase the control will come here again if the reset is
            not successful first time */
            _DRV_SDCARD_CHIP_DESELECT ( dObj->chipSelectPort,
                dObj->chipSelectBitPosition );

            /* Send eight clock pulses, send 0x0FF */
            dObj->spiBufferHandle = DRV_SPI_BufferAddWrite ( dObj->spiClientHandle, ( uint8_t* )
                &dObj->dataToSendClockPulses, 1 );

            dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_ENABLE_CHIPSELECT;
            break;
        case DRV_SDCARD_MEDIA_STATE_ENABLE_CHIPSELECT:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus ( dObj->spiBufferHandle ) )
            {
                /* This ensures the CMD0 is sent freshly after CS is asserted low,
                minimizing risk of SPI clock pulse master/slave syncronization problems,
                due to possible application noise on the SCK line.
                */
                _DRV_SDCARD_CHIP_SELECT ( dObj->chipSelectPort,
                    dObj->chipSelectBitPosition );

                dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_RESET_DEVICE;
            }
            break;
        case DRV_SDCARD_MEDIA_STATE_RESET_DEVICE:
            /* Send the command (CMD0) to software reset the device  */
            _DRV_SDCARD_CommandSend ( object, DRV_SDCARD_GO_IDLE_STATE, 0x00 );

            /* Change from this state only on completion of command execution */
            if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
            {
                if ( dObj->cmdResponse.response1.byte == CMD_R1_END_BIT_SET )
                {
                    /* For status command SD card will respond with R2 type packet */
                    dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_SPECIFY_VOLTAGE_LVL;
                }
                else
                {
                    /* For status command SD card will respond with R2 type packet */
                    dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_TOGGLE_CHIPSELECT;
                }
            }
            break;
        case DRV_SDCARD_MEDIA_STATE_SPECIFY_VOLTAGE_LVL:
            /* Send CMD8 (SEND_IF_COND) to specify/request the SD card interface
            condition (ex: indicate what voltage the host runs at).
            0x000001AA --> VHS = 0001b = 2.7V to 3.6V.  The 0xAA LSB is the check
            pattern, and is arbitrary, but 0xAA is recommended (good blend of 0's
            and '1's). The SD card has to echo back the check pattern correctly
            however, in the R7 response. If the SD card doesn't support the
            operating voltage range of the host, then it may not respond. If it
            does support the range, it will respond with a type R7 reponse packet
            (6 bytes/48 bits). Additionally, if the SD card is MMC or SD card
            v1.x spec device, then it may respond with invalid command.  If it is
            a v2.0 spec SD card, then it is mandatory that the card respond to CMD8
            */
            _DRV_SDCARD_CommandSend ( object, DRV_SDCARD_SEND_IF_COND, 0x1AA );

            /* Note: CRC value in the table  is set for value "0x1AA", it should
            be changed if a different value is passed. */

            /* Change from this state only on completion of command execution */
            if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
            {
                if ((( dObj->cmdResponse.response7.bytewise.argument.ocrRegister & 0xFFF ) == 0x1AA )
                        &&( false == dObj->cmdResponse.response7.bitwise.bits.illegalCommand ))
                {
                    dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_READ_OCR_REGISTER;
                }
                else
                {
                    dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_READ_NOT_SDHC;
                }
            }
            break;
        case DRV_SDCARD_MEDIA_STATE_READ_OCR_REGISTER:
            /*Send CMD58 (Read OCR [operating conditions register]).  Reponse
            type is R3, which has 5 bytes. Byte 4 = normal R1 response byte,
            Bytes 3-0 are = OCR register value.
            */
            _DRV_SDCARD_CommandSend ( object, DRV_SDCARD_READ_OCR, 0x00 );

            /* Change from this state only on completion of command execution */
            if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
            {
                dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_SIGNAL_APP_CMD;
            }
            break;
        case DRV_SDCARD_MEDIA_STATE_SIGNAL_APP_CMD:
            /* Send CMD55 (lets SD card know that the next command is application
            specific (going to be ACMD41)) */
            _DRV_SDCARD_CommandSend ( object, DRV_SDCARD_APP_CMD, 0x00 );

            /* Change from this state only on completion of command execution */
            if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
            {
                dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_SEND_APP_CMD;
            }
            break;
        case DRV_SDCARD_MEDIA_STATE_SEND_APP_CMD:
            /* TODO handle timeouts */
            /* Send ACMD41.  This is to check if the SD card is finished booting
            up/ready for full frequency and all further commands.  Response is
            R3 type (6 bytes/48 bits, middle four bytes contain potentially useful
            data).
            Note: When sending ACMD41, the HCS bit is bit 30, and must be = 1 to
            tell SD card the host supports SDHC
            */
            _DRV_SDCARD_CommandSend ( object, DRV_SDCARD_SD_SEND_OP_COND, 0x40000000 );

            /* Change from this state only on completion of command execution */
            if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
            {
                if ( dObj->cmdResponse.response1.byte == 0x00 )
                {
                    dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_INIT_COMPLETE;
                }
                else
                {
                    dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_SIGNAL_APP_CMD;
                }
            }
            break;
        case DRV_SDCARD_MEDIA_STATE_INIT_COMPLETE:
            /* Now send CMD58 ( Read OCR register ).  The OCR register contains
            important info we will want to know about the card (ex: standard
            capacity vs. SDHC).
            */
            _DRV_SDCARD_CommandSend ( object, DRV_SDCARD_READ_OCR, 0x00 );

            /* Change from this state only on completion of command execution */
            if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
            {
                /* Now check the CCS bit (OCR bit 30) in the OCR register, which
                is in our response packet. This will tell us if it is a SD high
                capacity (SDHC) or standard capacity device.
                Note: the HCS bit is only valid when the busy bit is also set
                (indicating device ready).
                */
                if ( dObj->cmdResponse.response7.bytewise.argument.ocrRegister & 0x40000000 )
                {
                    dObj->sdCardType = DRV_SDCARD_MODE_HC;
                }
                else
                {
                    dObj->sdCardType = DRV_SDCARD_MODE_NORMAL;
                }

                    /* Card initialization is complete, switch to normal operation */
                    dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_INCR_CLOCK_SPEED;
                }
            break;
        case DRV_SDCARD_MEDIA_STATE_CANNOT_INITIALIZE:
            dObj->mediaInformation.errorCode = SYS_FS_MEDIA_CANNOT_INITIALIZE;
            break;
        case DRV_SDCARD_MEDIA_STATE_READ_NOT_SDHC:
            /* Keep the chip select high(not selected) */
            _DRV_SDCARD_CHIP_DESELECT ( dObj->chipSelectPort,
                dObj->chipSelectBitPosition );
            /* TODO Add 1 milli second delay */

            dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_NOT_SDHC_RECHECK;
            break;
        case DRV_SDCARD_MEDIA_STATE_NOT_SDHC_RECHECK:
            _DRV_SDCARD_CHIP_SELECT ( dObj->chipSelectPort,
                dObj->chipSelectBitPosition );

            /* Send CMD1 to initialize the media */
            _DRV_SDCARD_CommandSend ( object, DRV_SDCARD_SEND_OP_COND, 0x00 );

			 /* Change from this state only on completion of command execution */
            if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
            {
            	dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_NOT_SDHC_RESET;
			}
            break;
        case DRV_SDCARD_MEDIA_STATE_NOT_SDHC_RESET:
            /* Change from this state only on completion of command execution */
            if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
            {
                if ( dObj->cmdResponse.response1.byte == CMD_R1_END_BIT_SET )
                {
                    dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_CANNOT_INITIALIZE;
                }
                else
                {
                    /*Set read/write block length to 512 bytes.
                    Note: commented out since this theoretically isn't necessary,
                    since all cards v1 and v2 are required to support 512 byte
                    block size, and this is supposed to be the default size selected
                    on cards that support other sizes as well. */

                    /* Set read/write block length to 512 bytes */

                    /* TODO handle this . need a different crd to test */
                    dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_INCR_CLOCK_SPEED;
                }
            }
            break;
        case DRV_SDCARD_MEDIA_STATE_INCR_CLOCK_SPEED:
            /* Temporarily keep the card de-selected */
            _DRV_SDCARD_CHIP_DESELECT ( dObj->chipSelectPort,
                dObj->chipSelectBitPosition );

            /* Basic initialization of media is now complete.  The card will now
            use push/pull outputs with fast drivers.  Therefore, we can now increase
            SPI speed to either the maximum of the microcontroller or maximum of
            media, whichever is slower.  MMC media is typically good for at least
            20Mbps SPI speeds. SD cards would typically operate at up to 25Mbps
            or higher SPI speeds.
             */
            _DRV_SDCARD_SET_SPEED ( _DRV_SDCARD_PERIPHERAL_ID_GET( dObj->spiId ),
              SYS_CLK_PeripheralFrequencyGet( CLK_BUS_PERIPHERAL_1 ),
              dObj->sdcardSpeedHz );

            /* Activate the chip select again */
            _DRV_SDCARD_CHIP_SELECT ( dObj->chipSelectPort,
                dObj->chipSelectBitPosition );
            dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_INCR_CLOCK_SPEED_COMMAND;
    case DRV_SDCARD_MEDIA_STATE_INCR_CLOCK_SPEED_COMMAND:
            /* CMD9: Read CSD data structure */
            _DRV_SDCARD_CommandSend ( object, DRV_SDCARD_SEND_CSD, 0x00 );

			 /* Change from this state only on completion of command execution */
            if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
            {
            	dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_READY_TO_READ_CSD;
            }

            break;
        case DRV_SDCARD_MEDIA_STATE_READY_TO_READ_CSD:
            /* Change from this state only on completion of command execution */
            /* TODO handle timeout */
            if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
            {
                if ( dObj->cmdResponse.response1.byte == 0x00 )
                {
                    dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_READ_CSD;
                }
                else
                {
                    dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_CANNOT_INITIALIZE;
                }
            }
            break;
        case DRV_SDCARD_MEDIA_STATE_READ_CSD:
            /* According to the simplified spec, section 7.2.6, the card will respond
            with a standard response token, followed by a data block of 16 bytes
            suffixed with a 16-bit CRC.
            */
            dObj->spiBufferHandle = DRV_SPI_BufferAddRead ( dObj->spiClientHandle, dObj->mediaInitReadCsd,
                                    _DRV_SDCARD_CSD_READ_SIZE );

            dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_PROCESS_CSD;
            break;
        case DRV_SDCARD_MEDIA_STATE_PROCESS_CSD:
            if ( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus ( dObj->spiBufferHandle ) )
            {
                dObj->mediaSectorSize = _DRV_SDCARD_MEDIA_BLOCK_SIZE;
                dObj->mediaInformation.sectorSize = _DRV_SDCARD_MEDIA_BLOCK_SIZE;
                dObj->mediaInformation.validityFlags.bits.sectorSize = true;
                /* Extract some fields from the response for computing the card
                capacity.
                Note: The structure format depends on if it is a CSD V1 or V2 device.
                Therefore, need to first determine version of the specs that the card
                is designed for, before interpreting the individual fields.
                */
                /* Calculate the MDD_SDSPI_finalLBA (see SD card physical layer
                simplified spec 2.0, section 5.3.2).
                In USB mass storage applications, we will need this information
                to correctly respond to SCSI get capacity requests.  Note: method
                of computing MDD_SDSPI_finalLBA TODO depends on CSD structure spec
                version (either v1 or v2).
                */
                if ( dObj->mediaInitReadCsd[0] & _DRV_SDCARD_CHECK_V2_DEVICE )
                {
                    /* Check CSD_STRUCTURE field for v2+ struct device */
                    /* Must be a v2 device (or a reserved higher version, that
                    doesn't currently exist) */
                    /* Extract the C_SIZE field from the response.  It is a 22-bit
                    number in bit position 69:48.  This is different from v1.
                    It spans bytes 7, 8, and 9 of the response.
                    */
                    cSize = ( ( ( uint32_t ) dObj->mediaInitReadCsd[ 7 ] & 0x3F ) << 16 ) |
                                ( ( uint16_t ) dObj->mediaInitReadCsd[ 8 ] << 8 ) |
                                dObj->mediaInitReadCsd[ 9 ];

                    /* -1 on end is correction factor, since capacity = 0 is valid. */
                    dObj->discCapacity = ( ( uint32_t )( cSize + 1 ) * ( uint16_t )( 1024u ) ) - 1;
                    dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_TURN_OFF_CRC;
                }
                else /* Not a V2 device, Must be a V1 device */
                {
                    /* Must be a v1 device. Extract the C_SIZE field from the
                    response.  It is a 12-bit number in bit position 73:62.
                    Although it is only a 12-bit number, it spans bytes 6, 7,
                    and 8, since it isn't byte aligned.
                    */
                    /* Get the bytes in the correct positions */
                    cSize = ( ( uint32_t ) dObj->mediaInitReadCsd[ 6 ] << 16) |
                            ( ( uint16_t ) dObj->mediaInitReadCsd[ 7 ] << 8) |
                            dObj->mediaInitReadCsd[ 8 ];

                    /* Clear all bits that aren't part of the C_SIZE */
                    cSize &= 0x0003FFC0;

                    /* Shift value down, so the 12-bit C_SIZE is properly right
                    justified */
                    cSize = cSize >> 6;

                    /* Extract the C_SIZE_MULT field from the response.  It is a
                    3-bit number in bit position 49:47 */
                    cSizeMultiplier = ( ( uint16_t ) ( ( dObj->mediaInitReadCsd[ 9 ]
                            & 0x03 ) << 1 ) ) | ( ( uint16_t )((dObj->mediaInitReadCsd[ 10 ]
                            & 0x80 ) >> 7 ) );

                    /* Extract the BLOCK_LEN field from the response. It is a
                    4-bit number in bit position 83:80
                    */
                    blockLength = dObj->mediaInitReadCsd[ 5 ] & 0x0F;

                    /* -9 because we report the size in sectors of 512 bytes each */
                    blockLength = 1 << ( blockLength - 9 );

                    /* Calculate the capacity (see SD card physical layer simplified
                    spec 2.0, section 5.3.2). In USB mass storage applications,
                    we will need this information to correctly respond to SCSI get
                    capacity requests (which will cause MDD_SDSPI_ReadCapacity()
                    to get called).
                    */
                    dObj->discCapacity = ( ( uint32_t ) ( cSize + 1 ) * ( uint16_t )
                            ( ( uint16_t )1 << ( cSizeMultiplier + 2 ) ) * blockLength ) - 1;

                    dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_TURN_OFF_CRC;
                    }
            }
            break;
        case DRV_SDCARD_MEDIA_STATE_TURN_OFF_CRC:
				/* Turn off CRC7 if we can, might be an invalid cmd on some
				cards (CMD59).
				Note: POR default for the media is normally with CRC checking
				off in SPI mode anyway, so this is typically redundant.
				*/
				_DRV_SDCARD_CommandSend ( object, DRV_SDCARD_CRC_ON_OFF, 0x00 );

				/* Change from this state only on completion of command execution */
				if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
				{
					dObj->mediaInitState = DRV_SDCARD_MEDIA_SET_BLOCKLEN;
				}
			break;
		case DRV_SDCARD_MEDIA_SET_BLOCKLEN:
				/* Now set the block length to media sector size. It
				should be already set to this. */
				_DRV_SDCARD_CommandSend ( object, DRV_SDCARD_SET_BLOCKLEN,
				    dObj->mediaSectorSize );

				/* Change from this state only on completion of command execution */
				if ( dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE )
				{
					dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_INITIALIZATION_COMPLETE;
				}
                break;
            break;
		case DRV_SDCARD_MEDIA_STATE_INITIALIZATION_COMPLETE:
				/* Coming for the first time */
            	dObj->mediaInitState = DRV_SDCARD_MEDIA_STATE_INIT_CHIP_SELECT;
            break;
    }
}


//******************************************************************************
/* Function:
    DRV_BUFFER_HANDLE DRV_SDCARD_SectorRead ( DRV_HANDLE handle, uint32_t sector_addr, uint8_t *buffer )

  Summary:
    Writes a sector to the SD Card.

  Description:
    This routine writes a sector to the SD card.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None
*/

DRV_SDCARD_BUFFER_HANDLE DRV_SDCARD_SectorWrite ( DRV_HANDLE handle, uint32_t sector_addr,
                                                    uint8_t *buffer, uint32_t sectorCount )
{
    DRV_SDCARD_CLIENT_OBJ   *clientObj =
        ( DRV_SDCARD_CLIENT_OBJ* ) _DRV_SDCARD_CLIENT_OBJ_GET ( handle );

    /* Add to the queue specifying the type as WRITE */
    return _DRV_SDCARD_AddToQueue (  (DRV_SDCARD_QUEUE_HANDLE)&gDrvSDCARDQueueObj[clientObj->drvIndex],
            buffer, DRV_SDCARD_TRANSFER_WRITE, sectorCount, sector_addr );
}

SYS_FS_MEDIA_STATUS DRV_SDCARD_MediaStatusGet ( DRV_HANDLE handle )
{
        DRV_SDCARD_CLIENT_OBJ  	*clientObj      =
    						( DRV_SDCARD_CLIENT_OBJ* ) &gDrvSDCARDClientObj[handle];
        
        DRV_SDCARD_OBJ* dObj = ( DRV_SDCARD_OBJ* )clientObj->driverObject;

	if ( dObj->isAttached == DRV_SDCARD_IS_ATTACHED )
	{
		return SYS_FS_MEDIA_ATTACHED;
	}
	else
	{
		return SYS_FS_MEDIA_DETACHED;
	}
}

uint32_t DRV_SDCARD_SectorSizeGet ( DRV_HANDLE handle );


uint32_t DRV_SDCARD_SectorsCountGet ( DRV_HANDLE handle );

//******************************************************************************
/* Function:
    DRV_BUFFER_HANDLE DRV_SDCARD_SectorRead ( DRV_HANDLE handle, uint32_t sector_addr, uint8_t *buffer )

  Summary:
    Reads a sector from the SD Card.

  Description:
    This routine reads a sector from SD card.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None
*/

DRV_SDCARD_BUFFER_HANDLE DRV_SDCARD_SectorRead ( DRV_HANDLE handle, uint8_t *buffer,
                                    uint32_t sector_addr, uint32_t sectorCount )
{
     DRV_SDCARD_CLIENT_OBJ   *clientObj =
        ( DRV_SDCARD_CLIENT_OBJ* ) _DRV_SDCARD_CLIENT_OBJ_GET ( handle );
    
    /* Add to the queue specifying the type as READ */
    return _DRV_SDCARD_AddToQueue ( (DRV_SDCARD_QUEUE_HANDLE)&gDrvSDCARDQueueObj[clientObj->drvIndex], buffer, DRV_SDCARD_TRANSFER_READ,
            sectorCount, sector_addr );
}


SYS_FS_MEDIA_BUFFER_STATUS DRV_SDCARD_BufferStatusGet ( DRV_HANDLE handle,
					SYS_FS_MEDIA_BUFFER_HANDLE bufferHandle )
{
	return ( ( DRV_SDCARD_XFER_OBJECT* ) bufferHandle )->status;
}

//******************************************************************************
/* Function:
    bool DRV_SDCARD_WriteProtectionIsEnabled ( DRV_HANDLE handle )

  Summary:
    Checks whether write protection is enabled.

  Description:
    This routine checks whether write protection is enabled.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None
*/

bool DRV_SDCARD_WriteProtectionIsEnabled ( DRV_HANDLE handle )
{
    DRV_SDCARD_CLIENT_OBJ       *clientObj  =
        ( DRV_SDCARD_CLIENT_OBJ* )_DRV_SDCARD_CLIENT_OBJ_GET( handle );
    DRV_SDCARD_OBJ              *dObj       =
        ( DRV_SDCARD_OBJ* ) _DRV_SDCARD_INSTANCE_GET( clientObj->driverObject );

    return _DRV_SDCARD_ValidatePin ( dObj->isWriteProtected, dObj->writeProtectPort,
        dObj->writeProtectBitPosition );
}


//******************************************************************************
/* Function:TODO
    bool DRV_SDCARD_WriteProtectionIsEnabled ( DRV_HANDLE handle )

  Summary:
    Checks whether write protection is enabled.

  Description:
    This routine checks whether write protection is enabled.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None
*/

bool _DRV_SDCARD_MediaPinDetect ( SYS_MODULE_OBJ object )
{
    DRV_SDCARD_OBJ              *dObj       =
        ( DRV_SDCARD_OBJ* ) _DRV_SDCARD_INSTANCE_GET ( object );

    /* Check whether detecting from pin is supported */
    _DRV_SDCARD_IS_PIN_ENABLED ( object );

    /*  Active high for card detection, revised design */
#ifdef DRV_SDCARD_CD_LOGIC_ACTIVE_LOW
     return !( _DRV_SDCARD_ValidatePin ( !dObj->isAttached, dObj->cardDetectPort,
        dObj->cardDetectBitPosition ) );
#endif

#ifdef DRV_SDCARD_CD_LOGIC_ACTIVE_HIGH
    return ( _DRV_SDCARD_ValidatePin ( !dObj->isAttached, dObj->cardDetectPort,
        dObj->cardDetectBitPosition ) );
#endif
}


//******************************************************************************
/* Function:
    bool _DRV_SDCARD_ValidatePin ( char previousState, PORTS_CHANNEL  portName,
        PORTS_BIT_POS portBitPosition )
  Summary:
    Validates the state of a pin by doing repeated reading.

  Description:
    This routine validates the state of a pin by doing repeated reading.

  Parameters:
    previousState   - Current state of the pin.
    portName        - PORT name in which the pin belongs to.
    portBitPosition - Bit number in the port.

  Returns:
    None
*/

bool _DRV_SDCARD_ValidatePin ( char previousState, PORTS_CHANNEL  portName,
        PORTS_BIT_POS portBitPosition )
{
    char reCheck = 0, repeatedData = 0;
    bool protectState, oldState =0;

    /* This loop validates the data by reading repeatedly. */
    for ( reCheck = 0; reCheck<10; reCheck++ )
    {
        /* Read from the pin */
        protectState = _DRV_SDCARD_PORT_PIN_READ ( portName, portBitPosition );

        /* If there is a change in the data read, means that it is changed while
         just now. Validate it multiple times to ensure */
        if( protectState != oldState )
        {
            repeatedData = 0;
        }
        else if ( repeatedData > 3 )
        {
            return protectState;
        }
        else
        {
            repeatedData++;
        }

    oldState = protectState;
    }

    /* Control comes here only if the state has changed while we poll the pin,
     but did not stay sufficient iterations to declare the state change. We can
	 check that next time. */
    return previousState;
}


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Queue handling functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    DRV_SDCARD_QUEUE_HANDLE _DRV_SDCARD_QueueInitialize ( const SYS_MODULE_INDEX    drvIndex )

  Summary:
    Initialize the queue.

  Description:
    Intializes the queue with the size of the array a resets the variables.

  Parameters:
    drvIndex: Index of SD card driver opened.

  Returns:
    A handle to the initialized queue.

  Remarks:
    None.
*/

DRV_SDCARD_QUEUE_HANDLE _DRV_SDCARD_QueueInitialize ( const SYS_MODULE_INDEX    drvIndex )
{
    gDrvSDCARDQueueObj[drvIndex].bufferPool =  &gDrvSDCARDTransferObj[drvIndex][0];

    gDrvSDCARDQueueObj[drvIndex].startIndex = 0;

    gDrvSDCARDQueueObj[drvIndex].endIndex = 0;

    gDrvSDCARDQueueObj[drvIndex].size = DRV_SDCARD_QUEUE_POOL_SIZE;

    return ( ( DRV_SDCARD_QUEUE_HANDLE ) &gDrvSDCARDQueueObj[drvIndex].startIndex );
}


// *****************************************************************************
/* Function:
    bool _DRV_SDCARD_AddToQueue( DRV_SDCARD_QUEUE_HANDLE handle, uint8_t *buffer,
                            DRV_SDCARD_TRANSFER_TYPE readWrite,
                            uint32_t sectorSize, uint32_t sector )

  Summary:
    Adding a new element to the queue.

  Description:
    Adding a new element to the queue.

  Parameters:
;or      -   Sector address.
    sectorSize  -   Number of bytes in a sector.
  Returns:
    true - Successfully added to the queue.
    false - Failed to add to the queue. The queue may be full.

  Remarks:
    None.
*/

DRV_SDCARD_BUFFER_HANDLE _DRV_SDCARD_AddToQueue ( DRV_SDCARD_QUEUE_HANDLE handle, uint8_t *buffer,
                            DRV_SDCARD_TRANSFER_TYPE readWrite,
                            uint32_t sectorCount, uint32_t sector )
{

    DRV_SDCARD_QUEUE_OBJECT     *qObj = ( DRV_SDCARD_QUEUE_OBJECT* )handle ;
    unsigned char               index = qObj->endIndex ;

    qObj->bufferPool[ index ].buffer = buffer;
    qObj->bufferPool[ index ].readWrite = readWrite;
    qObj->bufferPool[ index ].sectorAddress = sector;
    qObj->bufferPool[ index ].sectorCount = sectorCount;
    qObj->bufferPool[ index ].status = SYS_FS_MEDIA_BUFFER_QUEUED;

    qObj->endIndex = ( qObj->endIndex + 1 ) % qObj->size;

    if ( qObj->endIndex == qObj->startIndex )
    {
        /* full, overwrite */
        qObj->startIndex = ( qObj->startIndex + 1 ) % qObj->size;
    }
    return ( DRV_SDCARD_BUFFER_HANDLE ) & ( qObj->bufferPool[ index ] );
}


// *****************************************************************************
/* Function:
    DRV_SDCARD_XFER_OBJECT* _DRV_SDCARD_ReadFromQueue ( DRV_SDCARD_QUEUE_HANDLE *handle )

  Summary:
    Gets the top of the queue.

  Description:
    Gets the top of the queue

  Parameters:
    handle -    A queue handle.

  Returns:
    DRV_SDCARD_XFER_OBJECT.

  Remarks:
    None.
*/

DRV_SDCARD_XFER_OBJECT* _DRV_SDCARD_ReadFromQueue ( DRV_SDCARD_QUEUE_HANDLE handle )
{
    DRV_SDCARD_QUEUE_OBJECT *qObj = ( DRV_SDCARD_QUEUE_OBJECT* )handle ;
    DRV_SDCARD_XFER_OBJECT  *tObj;

    /* Check for queue empty */
    if ( qObj->endIndex  == qObj->startIndex )
    {
        return NULL;
    }

    tObj = &qObj->bufferPool[ qObj->startIndex ];

    qObj->startIndex = ( qObj->startIndex + 1 ) % qObj->size;

    return tObj;
}


// *****************************************************************************
/* Function:
    bool _DRV_SDCARD_IsQueueFull ( DRV_SDCARD_QUEUE_HANDLE *handle )

  Summary:
    Checks whether the queue is full.

  Description:
    Checks whether the queue is full.

  Parameters:
    handle -    A queue handle.

  Returns:
    true - Queue is full.
    false - Queue is not full.

  Remarks:
    None.
*/

bool _DRV_SDCARD_IsQueueFull ( DRV_SDCARD_QUEUE_HANDLE handle )
{
    DRV_SDCARD_QUEUE_OBJECT *qObj = ( DRV_SDCARD_QUEUE_OBJECT* )handle ;

    return ( ( ( qObj->endIndex + 1 ) % qObj->size  ) ==
            qObj->startIndex ) ? true:false;
}


// *****************************************************************************
/* Function:
    bool _DRV_SDCARD_IsQueueEmpty ( DRV_SDCARD_QUEUE_HANDLE *handle )

  Summary:
    Checks whether the queue is empty.

  Description:
    Checks whether the queue is empty.

  Parameters:
    handle -    A queue handle.

  Returns:
    true - Queue is empty.
    false - Queue is not empty.

  Remarks:
    None.
*/

bool _DRV_SDCARD_IsQueueEmpty ( DRV_SDCARD_QUEUE_HANDLE handle )
{
    DRV_SDCARD_QUEUE_OBJECT *qObj = ( DRV_SDCARD_QUEUE_OBJECT* )handle ;

    return  ( qObj->endIndex  == qObj->startIndex ) ? true:false;
}


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Software version functions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    unsigned int DRV_SDCARD_VersionGet( const SYS_MODULE_INDEX drvIndex )

  Summary:
    Gets SD card driver version in numerical format.

  Description:
    This routine gets the SD card driver version. The version is encoded as
    major * 10000 + minor * 100 + patch. The stringized version can be obtained
    using DRV_SDCARD_VersionStrGet()

  Parameters:
    None.

  Returns:
    Current driver version in numerical format.
*/

unsigned int DRV_SDCARD_VersionGet( const SYS_MODULE_INDEX drvIndex )
{
    return( ( _DRV_SDCARD_VERSION_MAJOR * 10000 ) +
            ( _DRV_SDCARD_VERSION_MINOR * 100 ) +
            ( _DRV_SDCARD_VERSION_PATCH ) );

} /* DRV_SDCARD_VersionGet */


// *****************************************************************************
/* Function:
    char * DRV_SDCARD_VersionStrGet( const SYS_MODULE_INDEX drvIndex )

  Summary:
    Gets SD card driver version in string format.

  Description:
    This routine gets the SD card driver version. The version is returned as
    major.minor.path[type], where type is optional. The numertical version can
    be obtained using DRV_SDCARD_VersionGet()

  Parameters:
    None.

  Returns:
    Current SD card driver version in the string format.

  Remarks:
    None.
*/

char * DRV_SDCARD_VersionStrGet( const SYS_MODULE_INDEX drvIndex )
{
    return _DRV_SDCARD_VERSION_STR;

} /* DRV_SDCARD_VersionStrGet */




/*******************************************************************************
End of File
*/



