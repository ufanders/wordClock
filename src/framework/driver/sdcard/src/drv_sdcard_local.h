/*******************************************************************************
  SD Card Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdcard_local.h

  Summary:
    SD Card driver local declarations, structures and function prototypes.

  Description:
    This file contains the SD Card driver's local declarations and definitions.
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

#ifndef _DRV_SDCARD_LOCAL_H
#define _DRV_SDCARD_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "driver/sdcard/drv_sdcard.h"
#include "driver/sdcard/src/drv_sdcard_variant_mapping.h"

// *****************************************************************************
// *****************************************************************************
// Section: Version Numbers
// *****************************************************************************
// *****************************************************************************
/* Versioning of the driver */

// *****************************************************************************
/* SD Card Driver Version Macros

  Summary:
    SD Card driver version

  Description:
    These constants provide SD Card driver version information. The driver
    version is
    DRV_SDCARD_VERSION_MAJOR.DRV_SDCARD_VERSION_MINOR.DRV_SDCARD_VERSION_PATCH.
    It is represented in DRV_SDCARD_VERSION as
    MAJOR *10000 + MINOR * 100 + PATCH, so as to allow comparisons.
    It is also represented in string format in DRV_SDCARD_VERSION_STR.
    DRV_SDCARD_TYPE provides the type of the release when the release is alpha
    or beta. The interfaces DRV_SDCARD_VersionGet() and
    DRV_SDCARD_VersionStrGet() provide interfaces to the access the version
    and the version string.

  Remarks:
    Modify the return value of DRV_SDCARD_VersionStrGet and the
    DRV_SDCARD_VERSION_MAJOR, DRV_SDCARD_VERSION_MINOR,
    DRV_SDCARD_VERSION_PATCH and DRV_SDCARD_VERSION_TYPE
*/

#define _DRV_SDCARD_VERSION_MAJOR         0
#define _DRV_SDCARD_VERSION_MINOR         5
#define _DRV_SDCARD_VERSION_PATCH         2
#define _DRV_SDCARD_VERSION_TYPE          "beta"
#define _DRV_SDCARD_VERSION_STR           "0.52 beta"


// *****************************************************************************
// *****************************************************************************
// Section: Helper Macros
// *****************************************************************************
// *****************************************************************************
/* Helper macros for the driver */

// *****************************************************************************
/* SD Card instance address

  Summary:
    Gets the SD card instance address from its index.

  Description:
    This macro gets the SD card instance address from its index.

  Remarks:
    None.
*/


#define _DRV_SDCARD_INSTANCE_GET(object)        &gDrvSDCARDObj[object]


// *****************************************************************************
/* SD Card chip select

  Summary:
    Asserts the SD card chip select pin.

  Description:
    This macro redirects to PORTS system service function to select the SD card
    by making the I/O pin low.

  Remarks:
    None.
*/

#define _DRV_SDCARD_CHIP_SELECT(port,pin)    	SYS_PORTS_PinClear(PORTS_ID_0,port,pin)


// *****************************************************************************
/* SD Card chip de-select

  Summary:
    Makes the SD card chip not selected.

  Description:
    This macro redirects to PORTS system service function to de-select the SD
    card by making the I/O pin high.

  Remarks:
    None.
*/

#define _DRV_SDCARD_CHIP_DESELECT(port,pin) 	SYS_PORTS_PinSet(PORTS_ID_0,port,pin)


// *****************************************************************************
/* SD Card pin read

  Summary:
    Reads a port pin which is used by SD card.

  Description:
    This macro redirects to PORTS system service function to read an SD card
    port pin ( used by SD card ).

  Remarks:
    None.
*/

#define _DRV_SDCARD_PORT_PIN_READ(port,pin)     SYS_PORTS_PinRead(PORTS_ID_0,port,pin)


// *****************************************************************************
/* SD Card speed set

  Summary:
    Sets the SD card speed.

  Description:
    This macro redirects to SPI API to set the speed.

  Remarks:
    None.
*/

#define _DRV_SDCARD_SET_SPEED(id,clk,speed)     PLIB_SPI_BaudRateSet(id,clk,speed);

// *****************************************************************************
// *****************************************************************************
// Section: SD Card constants
// *****************************************************************************
// *****************************************************************************
/* Constants used by SD card driver */

// *****************************************************************************
/* SD Card initial speed

  Summary:
    Initial speed of the SPI communication.

  Description:
    This macro holds the value of the initial communication speed with SD card.
    SD card only work at <=400kHz SPI frequency during initialization.

  Remarks:
    None.
*/

#define _DRV_SDCARD_SPI_INITIAL_SPEED                        400000


// *****************************************************************************
/* Count for 8 clock pulses

  Summary:
    Bytes count which should be passed to DRV_SPI_BufferReadAdd function to send
    8 clock pulses.

  Description:
    This macro holds the bytes count which should be passed to DRV_SPI_BufferReadAdd
    function to send 8 clock pulses. Sending one byte will send 8 clock pulses.


  Remarks:
    None.
*/

#define _DRV_SDCARD_SEND_8_CLOCKS                                            1


// *****************************************************************************
/* Count for 80 clock pulses

  Summary:
    Bytes count which should be passed to DRV_SPI_BufferReadAdd function to send
    80 clock pulses.

  Description:
    This macro holds the bytes count which should be passed to DRV_SPI_BufferReadAdd
    function to send 80 clock pulses. Sending one byte will send 8 clock pulses.


  Remarks:
    None.
*/

#define _DRV_SDCARD_SEND_80_CLOCKS                                           8


// *****************************************************************************
/* No of bytes to be read for SD card CSD.

  Summary:
    Number of bytes to be read to get the SD card CSD.

  Description:
    This macro holds number of bytes to be read to get the SD card CSD.


  Remarks:
    None.
*/

#define _DRV_SDCARD_CSD_READ_SIZE                                            20


// *****************************************************************************
/* SD card V2 device type.

  Summary:
    Holds the value to be checked against to state the device type is V2.

  Description:
    This macro holds value to be checked against to state the device type is V2.

  Remarks:
    None.
*/

#define _DRV_SDCARD_CHECK_V2_DEVICE                                          0xC0


// *****************************************************************************
/* SD card sector size.

  Summary:
    Holds the SD card sector size.

  Description:
    This macro holds the SD card sector size.

  Remarks:
    None.
*/

#define _DRV_SDCARD_MEDIA_BLOCK_SIZE                                         512


// *****************************************************************************
/* SD card transmit bit.

  Summary:
    Holds the SD card transmit bit position.

  Description:
    This macro holds the SD card transmit bit position.

  Remarks:
    None.
*/

#define DRV_SDCARD_TRANSMIT_SET												0x40


// *****************************************************************************
/* Write response token bit mask.

  Summary:
    Bit mask to AND with the write token response byte from the media,

  Description:
    This macro holds the bit mask to AND with the write token response byte from
    the media to clear the don't care bits.

  Remarks:
    None.
*/

#define DRV_SDCARD_WRITE_RESPONSE_TOKEN_MASK                                0x1F


// *****************************************************************************
/* SPI Bus floating

  Summary:
    This macro represents a floating SPI bus condition.

  Description:
   	This macro represents a floating SPI bus condition.

  Remarks:
    None.
*/

#define DRV_SDCARD_MMC_FLOATING_BUS                     0xFF


// *****************************************************************************
/* SD card bad response

  Summary:
    This macro represents a bad SD card response byte.

  Description:
   	This macro represents a bad SD card response byte.

  Remarks:
    None.
*/

#define DRV_SDCARD_MMC_BAD_RESPONSE    DRV_SDCARD_MMC_FLOATING_BUS


// *****************************************************************************
/* SD card start single block

  Summary:
    This macro represents an SD card start single data block token (used for
    single block writes).

  Description:
   	This macro represents an SD card start single data block token (used for
   	single block writes).

  Remarks:
    None.
*/

#define DRV_SDCARD_DATA_START_TOKEN                     0xFE


// *****************************************************************************
/* SD card start multiple blocks

  Summary:
    This macro represents an SD card start multi-block data token (used for
    multi-block writes).

  Description:
   	This macro represents an SD card start multi-block data token (used for
   	multi-block writes).

  Remarks:
    None.
*/

#define DRV_SDCARD_DATA_START_MULTI_BLOCK_TOKEN         0xFC


// *****************************************************************************
/* SD card stop transmission

  Summary:
    This macro represents an SD card stop transmission token.  This is used when
    finishing a multi block write sequence.

  Description:
   	This macro represents an SD card stop transmission token.  This is used when
   	finishing a multi block write sequence.

  Remarks:
    None.
*/

#define DRV_SDCARD_DATA_STOP_TRAN_TOKEN                 0xFD


// *****************************************************************************
/* SD card data accepted token

  Summary:
    This macro represents an SD card data accepted token.

  Description:
   	This macro represents an SD card data accepted token.

  Remarks:
    None.
*/

#define DRV_SDCARD_DATA_ACCEPTED                        0x05


// *****************************************************************************
/* SD card R1 response end bit

  Summary:
    This macro holds a value to check R1 response end bit.

  Description:
   	This macro holds a value to check R1 response end bit.

  Remarks:
    None.
*/

#define CMD_R1_END_BIT_SET          					0x01


// *****************************************************************************
/* SD card initial packet size

  Summary:
    This macro holds SD card initial packet size.

  Description:
   	This macro holds SD card initial packet size.

  Remarks:
    None.
*/

#define DRV_SDCARD_PACKET_SIZE      					6


// *****************************************************************************
/* SD card Media initialization array size

  Summary:
    This macro holds SD card Media initialization array size.

  Description:
   	This macro holds SD card Media initialization array size.

  Remarks:
    None.
*/

#define MEDIA_INIT_ARRAY_SIZE       					10


// *****************************************************************************
// *****************************************************************************
// Section: SD Card Enumerations
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* SD Card Driver read task Status

  Summary:
    Lists different states of the read task of the SD card driver

  Description:
    This enumeration lists different states of the read task of the SD card
    driver

  Remarks:
    None.
*/

typedef enum
{
    /* Initial state of the task */
    DRV_SDCARD_TASK_OPEN_SPI,

    /* A device is attached */
    DRV_SDCARD_TASK_CHECK_DEVICE,

    /* If the card is attached, initialize */
    DRV_SDCARD_TASK_MEDIA_INIT,

    /* Process queue for Read and write */
    DRV_SDCARD_TASK_PROCESS_QUEUE,

	/* Send stop transmission at the end of multiple
	block read command */
    DRV_SDCARD_TASK_READ_STOP_TRANSMISSION,

	/* State that multiple block read is complete */
    DRV_SDCARD_TASK_READ_STOP_TRANSMIT_CMPLT,

	/* Process read */
    DRV_SDCARD_TASK_PROCESS_READ,

    /* Process write */
    DRV_SDCARD_TASK_PROCESS_WRITE,

    /* Wait for response after sending 'READ' command */
    DRV_SDCARD_TASK_READ_WAIT_RESPONSE,

    /* Get the status of the SD card */
    DRV_SDCARD_TASK_READ_GET_BUSY_STATE,

    /* Wait for the start token */
    DRV_SDCARD_TASK_READ_WAIT_START_TOKEN,

    /* We got the start token.  Ready to receive the
    data block now */
    DRV_SDCARD_TASK_READ_NEW_PACKET_READY,

    /* Check for the data read completion */
    DRV_SDCARD_TASK_READ_CHECK_DATA_READ_CMPLTE,

	/* Block complete. next bytes will be crc */
	DRV_SDCARD_TASK_READ_BLOCK_CMPLTE_CRC_READ,

    /* Send 8 clock pulses after the read */
    DRV_SDCARD_TASK_READ_CMPLTE_SEND_CLOCKS,

    /* Fetch the next element from the queue */
    DRV_SDCARD_TASK_READ_PROCESS_NEXT,

    /* Read is complete */
    DRV_SDCARD_TASK_READ_DATA_READ_CMPLTE,

    /* Read aborted, handle it */
     DRV_SDCARD_TASK_READ_ABORT,

    /* Send erase command */
    DRV_SDCARD_TASK_ERASE_TO_WRITE,

    /* Address expected is different for normal
    mode compared to SDHC */
    DRV_SDCARD_TASK_MODIFY_ADDRESS,

    /* Send write command */
    DRV_SDCARD_TASK_SEND_WRITE_COMMAND,

    /* Check the status of the write */
    DRV_SDCARD_TASK_CHECK_WRITE_STATUS,

    /* Send the write packet */
    DRV_SDCARD_TASK_SEND_WRITE_PACKET,

     /* Actual write */
    DRV_SDCARD_TASK_SEND_RAW_DATA_WRITE,

    /* Send the CRC for the data written */
    DRV_SDCARD_TASK_SEND_CRC,

    /* Get response for the previous packet */
    DRV_SDCARD_TASK_WRITE_RESPONSE_GET,

	/* Write response get */
    DRV_SDCARD_TASK_WRITE_RESPONSE_GET_TOKEN_MASK,

    /* Wait for dummy read to complete */
    DRV_SDCARD_WRITE_DUMMY_READ,

    /* Sending 8 clocks is complete? */
    DRV_SDCARD_WRITE_CHECK_8CLOCKS,

    /* Ceck whether the Media is still busy */
    DRV_SDCARD_WRITE_CHECK_STILL_BUSY,

    /* Send clocks if the SD card stays busy */
    DRV_SDCARD_WRITE_CHECK_STILL_BUSY_SEND_CLOCKS,

    /* Eight clocks */
    DRV_SDCARD_WRITE_STOP_MULTIPLE,

	/* Status of the write multiple command */
    DRV_SDCARD_WRITE_STOP_MULTIPLE_STATUS,

    /* Stop transmision command send complete */
    DRV_SDCARD_WRITE_STOP_TRAN_CMPLT,

    /* Send 8 clock pulses */
    DRV_SDCARD_WRITE_SEND_8CLOCKS,

    /* SD card write is complete */
    DRV_SDCARD_WRITE_COMPLETE,

    /* Media busy */
    DRV_SDCARD_TASK_MEDIA_BUSY,

    /* Check the status of the write */
     DRV_SDCARD_TASK_STATE_ERROR,

    /* Something went wrong on write */
     DRV_SDCARD_TASK_WRITE_ABORT

} DRV_SDCARD_TASK_STATES;


// *****************************************************************************
/* SD Card connection states

  Summary:
    Lists SD card physical connection states

  Description:
    This enumeration lists different SD card physical connection states.

  Remarks:
    None.
*/

typedef enum
{
    /* SD Card is attached from the system */
    DRV_SDCARD_IS_DETACHED,

    /* SD Card is attached to the system */
    DRV_SDCARD_IS_ATTACHED

}DRV_SDCARD_ATTACH;



// *****************************************************************************
/* SD Card transfer type

  Summary:
    Specifies whether the SD card transfer is read or write.

  Description:
    This enumeration holds constants to specify whether the SD card transfer
    is read or write.

  Remarks:
    None.
*/

typedef enum
{
    /* SD card transfer is read from the card */
    DRV_SDCARD_TRANSFER_READ,

   /* SD card transfer is write to the card */
    DRV_SDCARD_TRANSFER_WRITE,

}DRV_SDCARD_TRANSFER_TYPE;


// *****************************************************************************
/* SD Card commands

  Summary:
    Lists different commands supported by the SD card.

  Description:
    This enumeration lists different commands supported by the SD card

  Remarks:
    None.
*/

typedef enum
{
    /* Command code to reset the SD card */
    CMD_VALUE_GO_IDLE_STATE = 0,

    /* Command code to initialize the SD card */
    CMD_VALUE_SEND_OP_COND  = 1,

    /* This macro defined the command code to check for sector addressing */
    CMD_VALUE_SEND_IF_COND  = 8,

    /* Command code to get the Card Specific Data */
    CMD_VALUE_SEND_CSD      = 9,

    /* Command code to get the Card Information */
    CMD_VALUE_SEND_CID      = 10,

    /* Command code to stop transmission during a multi-block read */
    CMD_VALUE_STOP_TRANSMISSION = 12,

    /* Command code to get the card status information */
    CMD_VALUE_SEND_STATUS       = 13,

    /* Command code to set the block length of the card */
    CMD_VALUE_SET_BLOCKLEN      = 16,

    /* Command code to read one block from the card */
    CMD_VALUE_READ_SINGLE_BLOCK  = 17,

    /* Command code to read multiple blocks from the card */
    CMD_VALUE_READ_MULTI_BLOCK   = 18,

    /* Command code to tell the media how many blocks to pre-erase
     (for faster multi-block writes to follow)
     Note: This is an "application specific" command.  This tells the media how
     many blocks to pre-erase for the subsequent WRITE_MULTI_BLOCK */
    CMD_VALUE_SET_WR_BLK_ERASE_COUNT =  23,

    /* Command code to write one block to the card */
    CMD_VALUE_WRITE_SINGLE_BLOCK  = 24,

    /* Command code to write multiple blocks to the card */
    CMD_VALUE_WRITE_MULTI_BLOCK   = 25,

    /* Command code to set the address of the start of an erase operation */
    CMD_VALUE_TAG_SECTOR_START    = 32,

    /* Command code to set the address of the end of an erase operation */
    CMD_VALUE_TAG_SECTOR_END      = 33,

    /* Command code to erase all previously selected blocks */
    CMD_VALUE_ERASE              =  38,

    /* Command code to intitialize an SD card and provide the CSD register value.
    Note: this is an "application specific" command (specific to SD cards)
    and must be preceded by CMD_APP_CMD */
    CMD_VALUE_SD_SEND_OP_COND     = 41,

    /* Command code to begin application specific command inputs */
    CMD_VALUE_APP_CMD             = 55,

    /* Command code to get the OCR register information from the card */
    CMD_VALUE_READ_OCR            = 58,

    /* Command code to disable CRC checking */
    CMD_VALUE_CRC_ON_OFF          = 59,

}DRV_SDCARD_COMMAND_VALUE;


// *****************************************************************************
/* SD Card Responses

  Summary:
    Lists different responses to commands by the SD card.

  Description:
    This enumeration lists different command responses by the SD card

  Remarks:
    None.
*/

typedef enum
{
    /* R1 type response */
    RESPONSE_R1,

    /* R1b type response */
    RESPONSE_R1b,

    /* R2 type response */
    RESPONSE_R2,

    /* R3 type response */
    RESPONSE_R3,

    /* R7 type response */
    RESPONSE_R7

}DRV_SDCARD_RESPONSES;


// *****************************************************************************
/* SD Card data response

  Summary:
    Holds the flags to set whether command is expecing data or not.

  Description:
    This enumeration holds the flags to set whether command is expecing data
    or not.

  Remarks:
    None.
*/

typedef enum
{
    /* Data is expected from the SD Card */
    DRV_SDCARD_GET_MOREDATA,

    /* No data is expected from the SD Card */
    DRV_SDCARD_GET_NODATA,

}DRV_SDCARD_DATA_GET;


// *****************************************************************************
/* SYS FS Media information

  Summary:
   TODO - move this to file system common header.

  Description:
    TODO

  Remarks:
    None.
*/

typedef enum
{
    /* No errors */
    SYS_FS_MEDIA_NO_ERROR,

    /* The requested device is not present */
    SYS_FS_MEDIA_DEVICE_NOT_PRESENT,

    /* Cannot initialize media */
    SYS_FS_MEDIA_CANNOT_INITIALIZE

} SYS_FS_MEDIA_ERRORS;


// *****************************************************************************
/* SD Card command indeces

  Summary:
    Lists the indeces of the commands in the command array.

  Description:
    This enumeration lists the indeces of the commands in the command array.
    This useful in retrieving commands from the command array.

  Remarks:
    None.
*/

typedef enum
{
    /* Index of in the CMD_GO_IDLE_STATE command 'Command array' */
    DRV_SDCARD_GO_IDLE_STATE,

    /* Index of in the CMD_SEND_OP_COND command 'Command array' */
    DRV_SDCARD_SEND_OP_COND,

    /* Index of in the CMD_SEND_IF_COND command 'Command array' */
    DRV_SDCARD_SEND_IF_COND,

    /* Index of in the CMD_SEND_CSD command 'Command array' */
    DRV_SDCARD_SEND_CSD,

    /* Index of in the CMD_SEND_CID command 'Command array' */
    DRV_SDCARD_SEND_CID,

    /* Index of in the CMD_STOP_TRANSMISSION command 'Command array' */
    DRV_SDCARD_STOP_TRANSMISSION,

    /* Index of in the CMD_SEND_STATUS command 'Command array' */
    DRV_SDCARD_SEND_STATUS,

    /* Index of in the CMD_SET_BLOCKLEN command 'Command array' */
    DRV_SDCARD_SET_BLOCKLEN,

    /* Index of in the CMD_READ_SINGLE_BLOCK command 'Command array' */
    DRV_SDCARD_READ_SINGLE_BLOCK,

    /* Index of in the CMD_READ_MULTI_BLOCK command 'Command array' */
    DRV_SDCARD_READ_MULTI_BLOCK,

    /* Index of in the CMD_WRITE_SINGLE_BLOCK command 'Command array' */
    DRV_SDCARD_WRITE_SINGLE_BLOCK,

    /* Index of in the CMD_WRITE_MULTI_BLOCK command 'Command array' */
    DRV_SDCARD_WRITE_MULTI_BLOCK,

    /* Index of in the CMD_TAG_SECTOR_START command 'Command array' */
    DRV_SDCARD_TAG_SECTOR_START,

    /* Index of in the CMD_TAG_SECTOR_END command 'Command array' */
    DRV_SDCARD_TAG_SECTOR_END,

    /* Index of in the CMD_ERASE command 'Command array' */
    DRV_SDCARD_ERASE,

    /* Index of in the CMD_APP_CMD command 'Command array' */
    DRV_SDCARD_APP_CMD,

    /* Index of in the CMD_READ_OCR command 'Command array' */
    DRV_SDCARD_READ_OCR,

    /* Index of in the CMD_CRC_ON_OFF command 'Command array' */
    DRV_SDCARD_CRC_ON_OFF,

    /* Index of in the CMD_SD_SEND_OP_COND command 'Command array' */
    DRV_SDCARD_SD_SEND_OP_COND,

    /* Index of in the CMD_SET_WR_BLK_ERASE_COUNT command 'Command array' */
    DRV_SDCARD_SET_WR_BLK_ERASE_COUNT

}DRV_SDCARD_COMMANDS;


// *****************************************************************************
/* SD Card type

  Summary:
    Lists different types of SD cards.

  Description:
    This enumeration lists different types of SD cards.

  Remarks:
    None.
*/

typedef enum
{
    /* Normal SD Card */
    DRV_SDCARD_MODE_NORMAL,

    /* SDHC type Card */
    DRV_SDCARD_MODE_HC

}DRV_SDCARD_TYPE;


// *****************************************************************************
/* SD Card initailization states

  Summary:
    Lists various initialization stages of SD card.

  Description:
    This enumeration lists various initialization stages of SD card.

  Remarks:
    None.
*/

typedef enum
{
    /* Initial stage */
    DRV_SDCARD_INIT_START_INIT,

    /* Start checking for SD card */
    DRV_SDCARD_INIT_CHECK_FOR_CARD,

    /* Clear the receive buffer after sending the clock pulses. Other there will
    be a receive overflow error on the next receive. */
    DRV_SDCARD_INIT_CLEAR_RCV_BUFFER,

    /* Wait for command response type R1 */
    DRV_SDCARD_INIT_WAIT_FOR_R1_RESPONSE,

    /* Once the initialization is complete, check for detach */
    DRV_SDCARD_INIT_CHECK_FOR_DETACH,

    /* Wait for command response type R2 */
    DRV_SDCARD_INIT_WAIT_FOR_R2_RESPONSE,

    /* Initialization error check */
    DRV_SDCARD_INIT_MEDIA_ERROR_CHECK,

}DRV_SDCARD_INIT_STATES;


// *****************************************************************************
/* SD Card command stages

  Summary:
    Lists various stages while sending a command to SD card.

  Description:
    This enumeration lists various stages while sending a command to SD card.

  Remarks:
    None.
*/

typedef enum
{
    /* Initial stage */
    DRV_SDCARD_CMD_FRAME_PACKET,

    /* Send the packet */
    DRV_SDCARD_CMD_SEND_PACKET,
    /* Clear the response as SPI just replies for every
    byte even before the packet is complete */
    DRV_SDCARD_CMD_CLEAR_REPLY,

    /* DRV_SDCARD_STOP_TRANSMISSION command is a special case.
    Handle that */
    DRV_SDCARD_CMD_CHECK_SPL_CASE,

    /* Check whether the command processing is complete */
    DRV_SDCARD_CMD_CHECK_TRANSFER_COMPLETE,

    /* Act as per the expected response type for the command */
    DRV_SDCARD_CMD_CHECK_RESP_TYPE,

    /* Clear the receive buffer after sending the clock pulses.
    Other there will be a receive overflow error on the next
    receive. */
    DRV_SDCARD_CMD_GET_RESPONSE,

    /* Switch to this when the response type is R2 */
    DRV_SDCARD_CMD_HANDLE_R2_RESPONSE,

    /* Switch to this when the response type is R1b */
    DRV_SDCARD_CMD_HANDLE_R1B_RESPONSE,

    /* Switch to this when the response type is R7 */
    DRV_SDCARD_CMD_HANDLE_R7_RESPONSE,

    /* Read back the data until the card is not busy */
    DRV_SDCARD_CMD_R1B_READ_BACK,

    /* Check for command execution completion */
    DRV_SDCARD_CMD_EXEC_CHECK_COMPLETION,

    /* Set the status as conversion complete */
    DRV_SDCARD_CMD_CONFIRM_COMPLETE,

    /* Command execution is complete */
    DRV_SDCARD_CMD_EXEC_IS_COMPLETE,

}DRV_SDCARD_CMD_STATES;


// *****************************************************************************
/* SD Card initialization stages

  Summary:
    Lists various stages while initializing the SD card.

  Description:
    This enumeration lists various stages while initializing the SD card.

  Remarks:
    None.
*/

typedef enum
{
    /* Initial stage, Assert chip select */
    DRV_SDCARD_MEDIA_STATE_INIT_CHIP_SELECT,

    /* Initial delay */
    DRV_SDCARD_MEDIA_STATE_INIT_DELAY_INITIAL,

    /* Initialize the array to send the clock pulses */
    DRV_SDCARD_MEDIA_STATE_INIT_ARRAY_INIT,

    /* Send 80 clock pulses without using a blocking code */
    DRV_SDCARD_MEDIA_STATE_INIT_SEND_CLOCK_PULSES,

    /* Check whether we send 80 clock pulses or not. If not send again */
    DRV_SDCARD_MEDIA_STATE_CHECK_FOR_80_PULSES,

    /* Toggle the chip select to stop any ongoing operations */
    DRV_SDCARD_MEDIA_STATE_TOGGLE_CHIPSELECT,

    /* Make the chip select low (enable) */
    DRV_SDCARD_MEDIA_STATE_ENABLE_CHIPSELECT,

    /* Reset the device by sending a command */
    DRV_SDCARD_MEDIA_STATE_RESET_DEVICE,

    /* Once the device reset is complete */
    DRV_SDCARD_MEDIA_STATE_SPECIFY_VOLTAGE_LVL,

    /* Read OCR(operating conditions register) */
    DRV_SDCARD_MEDIA_STATE_READ_OCR_REGISTER,

    /* Let the SD card know that next command will be an
     app command */
    DRV_SDCARD_MEDIA_STATE_SIGNAL_APP_CMD,

    /* Send app command */
    DRV_SDCARD_MEDIA_STATE_SEND_APP_CMD,

    /* SD card intialization is complete */
    DRV_SDCARD_MEDIA_STATE_INIT_COMPLETE,

    /* SD card cannot be initialized */
    DRV_SDCARD_MEDIA_STATE_CANNOT_INITIALIZE,

    /* Not SDHC , CMD08 is not supported */
    DRV_SDCARD_MEDIA_STATE_READ_NOT_SDHC,

    /* Recheck the card type */
    DRV_SDCARD_MEDIA_STATE_NOT_SDHC_RECHECK,

    /* Recheck by seding reset */
    DRV_SDCARD_MEDIA_STATE_NOT_SDHC_RESET,

    /* Init complete, switch to normal operation speed */
    DRV_SDCARD_MEDIA_STATE_INCR_CLOCK_SPEED,

    DRV_SDCARD_MEDIA_STATE_INCR_CLOCK_SPEED_COMMAND,

    /* Check whether the card is ready to read CSD */
    DRV_SDCARD_MEDIA_STATE_READY_TO_READ_CSD,

    /* Read CSD */
    DRV_SDCARD_MEDIA_STATE_READ_CSD,

    /* after reading, process and update the internal variables */
    DRV_SDCARD_MEDIA_STATE_PROCESS_CSD,

    /* Send command to turn off the CRC */
    DRV_SDCARD_MEDIA_STATE_TURN_OFF_CRC,

	/* Set the block length */
	DRV_SDCARD_MEDIA_SET_BLOCKLEN,

    /* If the last command processed successfully, the initialization
    is complete */
    DRV_SDCARD_MEDIA_STATE_INITIALIZATION_COMPLETE

}DRV_SDCARD_MEDIA_INIT;


// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

typedef uintptr_t 									DRV_SDCARD_CLIENT_OBJ_HANDLE;
typedef uintptr_t 									DRV_SDCARD_OBJ_HANDLE;
typedef uintptr_t 									DRV_SDCARD_QUEUE_HANDLE;


// *****************************************************************************
// *****************************************************************************
// Section: Data Structures
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* SD Card transfer object

  Summary:
    This structure holds the elements of the SD card transfer queue.

  Description:
    This structure holds the elements of the SD card transfer queue. The queue
    will be processed byt the SD card task and the elements to the queue will be
    added from 'sector read' or 'sector write' functons. So this queue must hold
    all the information about the request.

  Remarks:
    None.
*/

typedef struct
{
    /* Buffer to which the data to written or read from */
    uint8_t 					*buffer;

	/* read/write flag */
    DRV_SDCARD_TRANSFER_TYPE 	readWrite;

	/* Status of the operation */
    SYS_FS_MEDIA_BUFFER_STATUS    status;

	/* Sector start address */
    uint32_t 					sectorAddress;

	/* Number of sectors  */
	uint32_t 					sectorCount;

}DRV_SDCARD_XFER_OBJECT;


// *****************************************************************************
/* SD Card queue object

  Summary:
    SD Card queue elements.

  Description:
   	This structure holds the variables necessary to handle the SD card Queue.

  Remarks:
    None.
*/

typedef struct
{
	/* Queue start index */
	unsigned char                	startIndex;

	/* Queue end index */
	unsigned char                   endIndex;

	/* Queue Link to the actual buffers */
	DRV_SDCARD_XFER_OBJECT          *bufferPool;

	/* Size of the queue */
	unsigned char                	size;

}DRV_SDCARD_QUEUE_OBJECT;


// *****************************************************************************
/* SD Card Command data structure

  Summary:
   Holds different commands ard the expected response for those commands.

  Description:
    This enumeration the commands supported by the SD card, its CRC and response
    corresponding to each command.

  Remarks:
    None.
*/

typedef struct
{
    /* Command code */
    DRV_SDCARD_COMMAND_VALUE      commandCode;

    /* CRC value for that command */
    uint8_t      crc;

    /* Response type */
    DRV_SDCARD_RESPONSES    responseType;

    /* Whether more data is expected or not */
    DRV_SDCARD_DATA_GET    moredataexpected;

} DRV_SDCARD_CMD_OBJ;


// *****************************************************************************
/* SD Card Command data structure

  Summary:
   Holds different commands ard the expected response for those commands.

  Description:
    This enumeration the commands supported by the SD card, its CRC and response
    corresponding to each command.

  Remarks:
    None.
*/

typedef struct
{
    uint8_t *buffer;

    DRV_SDCARD_TRANSFER_TYPE readWrite;

    uint32_t sectorSize;

    uint32_t sectorAddress;

    uint32_t sectorCount;

    uint32_t bytesRemaining;

    uint32_t bytesToRead;

    uint16_t blockCounter;

    uint8_t command;

    DRV_SDCARD_BUFFER_STATUS *status;

}DRV_SDCARD_TASK_OPERATIONS;


// *****************************************************************************
/* Sys FS media information

  Summary:
   Sys FS media information.

  Description:
    Sys FS media information.

  Remarks:
    None.
*/

typedef struct
{
    uint8_t    errorCode;
    union
    {
        uint8_t    value;
        struct
        {
            uint8_t    sectorSize  : 1;
            uint8_t    maxLun      : 1;
        }   bits;
    } validityFlags;

    uint16_t    sectorSize;
    uint8_t    maxLun;

} SYS_FS_MEDIA_INFORMATION;


// *****************************************************************************
/* SD Card Command packet

  Summary:
    Different commands ard the expected response for those commands.

  Description:
    This union represents different ways to access an SD card command packet.

  Remarks:
    None.
*/

typedef union
{
    /* This structure allows array-style access of command bytes */
    struct
    {
        uint8_t field[7];
    };

    /* This structure allows byte-wise access of packet command bytes */
    struct
    {
        /* The CRC byte */
        uint8_t crc;

        /* Filler space (since bitwise declarations can't
         cross a uint32_t boundary) */
        uint8_t c32filler[3];

        /* Address byte 0 */
        uint8_t address0;

        /* Address byte 1 */
        uint8_t address1;

        /* Address byte 2 */
        uint8_t address2;

        /* Address byte 3 */
        uint8_t address3;

        /* Command code byte */
        uint8_t cmd;
    };

    /* This structure allows bitwise access to elements of the command bytes */
    struct
    {
        /* Packet end bit */
        uint8_t  endBit:1;

        /* CRC value */
        uint8_t  crc7:7;

        /* Address */
        uint32_t    address;

        /* Command code */
        uint8_t  cmdIndex:6;

        /* Transmit bit */
        uint8_t  transmitBit:1;

        /* Packet start bit */
        uint8_t  startBit:1;
    };
} DRV_SDCARD_CMD_PACKET;


// *****************************************************************************
/* SD Card R1 type response format

  Summary:
    Different ways to access R1 type response packet.

  Description:
    This union represents different ways to access an SD card R1 type response
    packet.

  Remarks:
    None.
*/

typedef union
{
    /* Byte-wise access */
    uint8_t byte;

    /* This structure allows bitwise access of the response */
    struct
    {
        /* Card is in idle state */
        unsigned inIdleState:1;

        /* Erase reset flag */
        unsigned eraseReset:1;

        /* Illegal command flag */
        unsigned illegalCommand:1;

        /* CRC error flag */
        unsigned crcError:1;

        /* Erase sequence error flag */
        unsigned eraseSequenceError:1;

        /* Address error flag */
        unsigned addressError:1;

        /* Parameter flag */
        unsigned parameterError:1;

        /* Unused bit 7 */
        unsigned unusedB7:1;
    };

} DRV_SDCARD_RESPONSE_1;


// *****************************************************************************
/* SD Card R2 type response format

  Summary:
    Different ways to access R2 type response packet.

  Description:
    This union represents different ways to access an SD card R2 type response
    packet.

  Remarks:
    None.
*/

typedef union
{
    /* Get both the bytes */
    uint16_t word;

    struct
    {
        /* Byte-wise access */
        uint8_t      byte0;
        uint8_t      byte1;
    };
    struct
    {
        /* Card is in idle state */
        unsigned inIdleState:1;

        /* Erase reset flag */
        unsigned eraseReset:1;

        /* Illegal command flag */
        unsigned illegalCommand:1;

        /* CRC error flag */
        unsigned crcError:1;

        /* Erase sequence error flag */
        unsigned eraseSequenceError:1;

        /* Address error flag */
        unsigned addressError:1;

        /* Parameter error flag */
        unsigned parameterError:1;

        /* Un-used bit */
        unsigned unusedB7:1;

        /* Card is locked? */
        unsigned cardIsLocked:1;

        /* WP erase skip| lock/unlock command failed */
        unsigned wpEraseSkipLockFail:1;

        /* Error */
        unsigned error:1;

        /* CC error */
        unsigned ccError:1;

        /* Card ECC fail */
        unsigned cardEccFail:1;

        /* WP violation */
        unsigned wpViolation:1;

        /* Erase parameter */
        unsigned eraseParam:1;

        /* out of range or CSD over write */
        unsigned outrangeCsdOverWrite:1;
    };

} DRV_SDCARD_RESPONSE_2;


// *****************************************************************************
/* SD Card R7 and R3 type response format

  Summary:
    Different ways to access R3/R7 type response packet.

  Description:
    This union represents different ways to access an SD card R3/R7 type
    response packet.

  Remarks:
    Note: The SD card argument response field is 32-bit, big endian format.
    However, the C compiler stores 32-bit values in little endian in RAM. When
    writing to the _returnVal/argument bytes, make sure to byte swap the order
    from which it arrived over the SPI from the SD card.
*/

typedef union
{
    struct
    {
        uint8_t byteR1;

        /* OCR register */
        union
        {
            /* OCR register */
            uint32_t ocrRegister;

            /* OCR register in bytes */
            struct
            {
                uint8_t ocrRegisterByte0;
                uint8_t ocrRegisterByte1;
                uint8_t ocrRegisterByte2;
                uint8_t ocrRegisterByte3;
            };
        }argument;

    } bytewise;
    /* This structure allows bitwise access of the response */
    struct
    {
        struct
        {
        /* Card is in idle state */
        unsigned inIdleState:1;

        /* Erase reset flag */
        unsigned eraseReset:1;

        /* Illegal command flag */
        unsigned illegalCommand:1;

        /* CRC error flag */
        unsigned crcError:1;

        /* Erase sequence error flag */
        unsigned eraseSequenceError:1;

        /* Address error flag */
        unsigned addressError:1;

        /* Parameter error flag */
        unsigned parameterError:1;

        /* un-used bit B7 */
        unsigned unusedB7:1;

        }bits;

        /* Read the complete register */
        uint32_t ocrRegister;

    } bitwise;

} DRV_SDCARD_RESPONSE_7;


// *****************************************************************************
/* SD Card Responses

  Summary:
    Different SD card response packets.

  Description:
    This an union of SD card response packets.

  Remarks:
    None.
*/

typedef union
{
    /* SD Card response 1 */
    DRV_SDCARD_RESPONSE_1  response1;

    /* SD Card response 2 */
    DRV_SDCARD_RESPONSE_2  response2;

    /* SD Card response 7 */
    DRV_SDCARD_RESPONSE_7  response7;

}DRV_SDCARD_RESPONSE_PACKETS;


// *****************************************************************************
/* SD Card data register

  Summary:
    A description of the card specific data register.

  Description:
    This union represents different ways to access information in a packet with
    SD card CSD informaiton. For more information on the CSD register, refer
    SD card user's manual.

  Remarks:
    None.
*/

typedef union
{
    struct
    {
        /* Access 32 bit format 1st */
        uint32_t access32_0;

        /* Access 32 bit format 2nd */
        uint32_t access32_1;

        /* Access 32 bit format 3rd */
        uint32_t access32_2;

        /* Access 32 bit format 4th */
        uint32_t access32_3;
    };
    struct
    {
        /* Access as bytes */
        uint8_t byte[16];
    };

    struct
    {
        /* Un used bit */
        unsigned unUsed           :1;

        /* Crc */
        unsigned crc                :7;

        /* Ecc */
        unsigned ecc                :2;

        /* File format */
        unsigned fileFormat        :2;

        /* temporary write protection */
        unsigned tempWriteProtect  :1;

        /* Permanent write protection */
        unsigned permanantWriteProtect :1;

        /* Copy flag */
        unsigned copyFlag           :1;

        /* File format group */
        unsigned fileFormatGroup    :1;

        /* Reserved bits */
        unsigned reserved_1         :5;

        /* Partial blocks for write allowed */
        unsigned writeBlockPartial   :1;

        /* Max. write data block length high */
        unsigned writeBlockLenHigh    :2;

        /* Max. write data block length low */
        unsigned writeBlockLenLow     :2;

        /* write speed factor */
        unsigned writeSpeedFactor     :3;

        /* default Ecc */
        unsigned defaultEcc            :2;

        /* Write protect group enable */
        unsigned writeProtectGrpEnable :1;

        /* Write protect group size */
        unsigned writeProtectGrpSize   :5;

        /* Erase sector size low */
        unsigned eraseSectorSizeLow   :3;

        /* Erase sector size high */
        unsigned eraseSectorSizeHigh   :2;

        /* Sector size */
        unsigned sectorSize        :5;

        /* TODO  erase single block enable??*/
        /* Device size multiplier low */
        unsigned deviceSizeMultiLow      :1;

        /* Device size multiplier high */
        unsigned deviceSizeMultiHigh      :2;

        /* Max. write current @VDD max */
        unsigned vddWriteCurrentMax     :3;

        /* max. write current @VDD min */
        unsigned vddWriteCurrentMin      :3;

        /* max. read current @VDD max */
        unsigned vddReadCurrentMax     :3;

        /* max. read current @VDD min */
        unsigned vddReadCurrentMin     :3;

        /* device size low */
        unsigned deviceSizeLow         :2;

        /* device size high */
        unsigned deviceSizeHigh        :8;

        /* device size upper */
        unsigned deviceSizeUp          :2;

        /* reserved bits */
        unsigned reserved2             :2;

        /* DSR implemented */
        unsigned dsrImplemented        :1;

        /* Read block misalignment */
        unsigned readBlockMissAllign  :1;

        /* Write block misalignment */
        unsigned writeBlockMissAllign :1;

        /* Partial blocks for read allowed */
        unsigned partialBlockRead     :1;

        /* Max. read data block length */
        unsigned readDataBlockMax     :4;

        /* Card command classes low */
        unsigned cardCmdClassesLow    :4;

        /* Card command classes high */
        unsigned cardCmdClassesHigh   :8;

        /* max. data transfer rate */
        unsigned maxDataTrasferRate   :8;

        /* Data read access-time-2 in clock cycles (NSAC*100) */
        unsigned dataReadTime2Clocks   :8;

        /* data read access-time-1 */
        unsigned dataReadTime1Clocks   :8;

        /* Reserved bits */
        unsigned reserved3             :2;

        /* Specification version */
        unsigned specVersion           :4;

        /* CSD structure */
        unsigned csdStructure         :2;
    };

} DRV_SDCARD_CSD;


// *****************************************************************************
/* SD Card information register

  Summary:
    A description of the card specific information register.

  Description:
    This union represents different ways to access information in a packet with
    SD card CID informaiton. For more information on the CSD register, refer
    SD card user's manual.

  Remarks:
    None.
*/

typedef union
{
    struct
    {
        /* Access 32 bit format 1st */
        uint32_t access32_0;

        /* Access 32 bit format 2nd */
        uint32_t access32_1;

        /* Access 32 bit format 3rd */
        uint32_t access32_2;

        /* Access 32 bit format 4th */
        uint32_t access32_3;
    };
    struct
    {
        uint8_t byte[16];
    };
    struct
    {
        /* Unused bit */
        unsigned    notUsed            :1;

        /* Crc */
        unsigned    crc                 :7;

        /* Manufacturing date */
        unsigned    mnufacturingDate    :8;

        /* Product serial number */
        uint32_t       serialNumber;

        /* Product revision */
        unsigned    revision            :8;

        /* Product name */
        char        name[6];

        /* OEM Application ID */
        uint16_t        oemId;

        /* Manufacturer ID */
        unsigned    manufacturerId     :8;
    };
} DRV_SDCARD_CID;


// *****************************************************************************
/* SD Card Driver Hardware Instance Object

  Summary:
    Defines the object required for the maintainence of the hardware instance

  Description:
    This defines the object required for the maintainence of the hardware
    instance. This object exists once per hardware instance of the peripheral.

  Remarks:
    None
*/

typedef struct _DRV_SDCARD_OBJ_STRUCT
{
    /* Holds the SPI module ID linked to the instance of SD Card driver */
    SPI_MODULE_ID                   spiId;

    /* Index of the SPI driver */
    SYS_MODULE_INDEX                spiIndex;

    /* SPI client handle */
    DRV_HANDLE                      spiClientHandle;

    /* This is needed t keep track of the transfers */
    DRV_SPI_BUFFER_HANDLE           spiBufferHandle;

    /* Speed at which SD card communication should happen */
    uint32_t                        sdcardSpeedHz;

    /* Index of the driver */
    SYS_MODULE_INDEX                drvIndex;

    /* The status of the driver */
    SYS_STATUS                      status;

    /* Flag to indicate in use  */
    bool                            inUse;

    /* Flag to indicate that SD Card is used in exclusive access mode */
    bool                            isExclusive;

    /* Number of clients possible with the hardware instance */
    uint8_t                         numClients;

    /* Mutex to protect the client allocation */
    //OSAL_MUTEX_DECLARE(mutexUSART_clientHandle);

    /* Other members to support driver operations */
    /* Sector size of the SD Card */
    uint32_t                        mediaSectorSize;

    /* Number of sectors in the SD card */
    uint32_t                        discCapacity;

    /* Device is attached */
    DRV_SDCARD_ATTACH               isAttached;

    /* Device is attached last status */
    DRV_SDCARD_ATTACH               isAttachedLastStatus;

    /* SD card type, will be updated by initialization function */
    DRV_SDCARD_TYPE                 sdCardType;

    /* Write protect state */
    char                            isWriteProtected;

    /* Card detect pin setting */
    PORTS_CHANNEL                   cardDetectPort;

    /* Position of the bit in the port selected for card detection */
    PORTS_BIT_POS                   cardDetectBitPosition;

    /* Update in initialization */
    DRV_SDCARD_QUEUE_HANDLE         queueHandle;

    /* Write protect pin setting */
    PORTS_CHANNEL                   writeProtectPort;

    /* Position of the bit in the port selected for Write protection */
    PORTS_BIT_POS                   writeProtectBitPosition;

    /* Chip select pin setting */
    PORTS_CHANNEL                   chipSelectPort;

    /* Position of the bit in the port selected for chip selection */
    PORTS_BIT_POS                   chipSelectBitPosition;

    /* This variable holds the current state of the DRV_SDCARD_Task */
    DRV_SDCARD_TASK_STATES          taskState;

    /* Different stanges of initialization */
    DRV_SDCARD_INIT_STATES          cmdDetectState;

    /* The command response will be updated here */
    DRV_SDCARD_RESPONSE_PACKETS     cmdResponse;

    /* Different states in sending a command */
    DRV_SDCARD_CMD_STATES           cmdState;

    /* Different stages in media initialization */
    DRV_SDCARD_MEDIA_INIT           mediaInitState;

    /* Information about the SD card, Will be updated on initialization */
    SYS_FS_MEDIA_INFORMATION        mediaInformation;

    /* Variables used to hadle the operations in the task */
    DRV_SDCARD_TASK_OPERATIONS      localTaskObj;

	/* Status of the operation */
	SYS_FS_MEDIA_BUFFER_STATUS    *bufferStatus;

    /* Packet which is currently being send ( +1 to handle
     DRV_SDCARD_STOP_TRANSMISSION command ) */
    uint8_t                         packetArray [ DRV_SDCARD_PACKET_SIZE +1 ];

    /* Index in the packet array */
    uint8_t                         cmdIndex;

    /* Media index , used by mediaInitialize function */
    uint8_t                         mediaInitArray [ MEDIA_INIT_ARRAY_SIZE ];

    /* Data about the disc */
    uint8_t                         mediaInitReadCsd [ _DRV_SDCARD_CSD_READ_SIZE ];

    /* Keeps track of the CSD byte read */
    uint8_t                         mediaInitindex;

    /* Many times in the execution of commands the driver needs to send clock
    pulses to SD card. To pass by reference to the SPI driver the data need to
    to be it should be in a global variable */
    uint8_t                         dataToSendClockPulses;

	/* The disk number allocated by Media Manager */
    int								disk;

    /* Status of the device */
    SYS_FS_MEDIA_STATUS             mediaState;


} DRV_SDCARD_OBJ;


// *****************************************************************************
/* SD Card Driver Client Object

  Summary:
    Defines the object required for the maintainence of the software clients

  Description:
    This defines the object required for the maintainence of the software
    clients instance. This object exists once per client instance.

  Remarks:
    None
*/

typedef struct _DRV_SDCARD_CLIENT_OBJ_STRUCT
{
    /* Maintains Client Status  */
    DRV_SDCARD_CLIENT_STATUS                       status;

    /* Hardware instance index associated with the client */
    DRV_SDCARD_OBJ_HANDLE                          driverObject;

    /* Save the index while opening the driver */
    SYS_MODULE_INDEX                               drvIndex;
    /* Flag to indicate in use  */
    bool                                           inUse;

} DRV_SDCARD_CLIENT_OBJ;

/*****************************************************************************
 * If the SD card needs to be controlled by media maanger, then declare the
 * following as 1. Othewise, declare as 0.
 *
 *****************************************************************************/
#define DRV_SDCARD_MEDIA_MANAGER_USE                1

// *****************************************************************************
// *****************************************************************************
// Section: Extern data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*  Hardware Objects for the dynamic driver
*/

extern DRV_SDCARD_OBJ            gDrvSDCARDObj[];


// *****************************************************************************
/*  Client Objects for the multi-client driver
*/

extern DRV_SDCARD_CLIENT_OBJ     gDrvSDCARDClientObj[];


// *****************************************************************************
/*  Hardware Objects for the static driver
*/

extern DRV_SDCARD_OBJ            gDrvSDCARDObj0;
extern DRV_SDCARD_OBJ            gDrvSDCARDObj1;
extern DRV_SDCARD_OBJ            gDrvSDCARDObj2;
extern DRV_SDCARD_OBJ            gDrvSDCARDObj3;
extern DRV_SDCARD_OBJ            gDrvSDCARDObj4;
extern DRV_SDCARD_OBJ            gDrvSDCARDObj5;
extern DRV_SDCARD_OBJ            gDrvSDCARDObj6;


// *****************************************************************************
/*  Client Objects for the single-client driver
*/

extern DRV_SDCARD_CLIENT_OBJ     gDrvSDCARDClientObj0;
extern DRV_SDCARD_CLIENT_OBJ     gDrvSDCARDClientObj1;
extern DRV_SDCARD_CLIENT_OBJ     gDrvSDCARDClientObj2;
extern DRV_SDCARD_CLIENT_OBJ     gDrvSDCARDClientObj3;
extern DRV_SDCARD_CLIENT_OBJ     gDrvSDCARDClientObj4;
extern DRV_SDCARD_CLIENT_OBJ     gDrvSDCARDClientObj5;
extern DRV_SDCARD_CLIENT_OBJ     gDrvSDCARDClientObj6;


// *****************************************************************************
// *****************************************************************************
// Section: Function prototypes of local functions
// *****************************************************************************
// *****************************************************************************

bool _DRV_SDCARD_MediaPinDetect ( SYS_MODULE_OBJ object );
bool _DRV_SDCARD_MediaCommandDetect ( SYS_MODULE_OBJ object );
bool _DRV_SDCARD_ValidatePin ( char previousState, PORTS_CHANNEL  portName,
        PORTS_BIT_POS portBitPosition );
void _DRV_SDCARD_CommandSend ( SYS_MODULE_OBJ object, DRV_SDCARD_COMMANDS command,
                                uint32_t address );
void _DRV_SDCARD_MediaInitialize ( SYS_MODULE_OBJ object );

DRV_SDCARD_QUEUE_HANDLE _DRV_SDCARD_QueueInitialize( const SYS_MODULE_INDEX    drvIndex );

DRV_SDCARD_BUFFER_HANDLE _DRV_SDCARD_AddToQueue(  DRV_SDCARD_QUEUE_HANDLE handle, uint8_t *buffer,
                            DRV_SDCARD_TRANSFER_TYPE readWrite,
                            uint32_t sectorCount, uint32_t sector );

bool _DRV_SDCARD_IsQueueFull ( DRV_SDCARD_QUEUE_HANDLE handle );

bool _DRV_SDCARD_IsQueueEmpty ( DRV_SDCARD_QUEUE_HANDLE handle );

DRV_SDCARD_XFER_OBJECT* _DRV_SDCARD_ReadFromQueue ( DRV_SDCARD_QUEUE_HANDLE handle );


#endif //#ifndef _DRV_SDCARD_LOCAL_H

/*******************************************************************************
 End of File
*/

