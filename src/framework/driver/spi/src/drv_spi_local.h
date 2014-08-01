/*******************************************************************************
  SPI Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_spi_local.h

  Summary:
    SPI driver local declarations and definitions.

  Description:
    This file contains the SPI driver's local declarations and definitions.
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

#ifndef _DRV_SPI_LOCAL_H
#define _DRV_SPI_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "driver/spi/drv_spi.h"
#include "driver/spi/src/drv_spi_variant_mapping.h"


// *****************************************************************************
// *****************************************************************************
// Section: Version Numbers
// *****************************************************************************
// *****************************************************************************
/* Versioning of the driver */

// *****************************************************************************
/* SPI Driver Version Macros

  Summary:
    SPI driver version.

  Description:
    These constants provide SPI driver version information. The driver
    version is:
    DRV_SPI_VERSION_MAJOR.DRV_SPI_VERSION_MINOR.DRV_SPI_VERSION_PATCH.
    It is represented in DRV_SPI_VERSION as:
    MAJOR *10000 + MINOR * 100 + PATCH, so as to allow comparisons.
    It is also represented in string format in DRV_SPI_VERSION_STR.
    DRV_SPI_TYPE provides the type of the release when the release is alpha
    or beta. The interfaces DRV_SPI_VersionGet and DRV_SPI_VersionStrGet
    provide interfaces to the access the version and the version string.

  Remarks:
    Modify the return value of DRV_SPI_VersionStrGet and DRV_SPI_VERSION_MAJOR,
    DRV_SPI_VERSION_MINOR, DRV_SPI_VERSION_PATCH, and DRV_SPI_VERSION_TYPE.
*/

#define _DRV_SPI_VERSION_MAJOR         0
#define _DRV_SPI_VERSION_MINOR         6
#define _DRV_SPI_VERSION_PATCH         0
#define _DRV_SPI_VERSION_TYPE          "beta"
#define _DRV_SPI_VERSION_STR           "0.60 beta"


// *****************************************************************************
/* SPI data object size

  Summary
    Specifies driver data object size.

  Description
    This type specifies the driver data object size.

  Remarks:
  	None.
*/

#define DRV_SPI_BUFFER_OBJ_SIZE						10


// *****************************************************************************
/* SPI Driver task states

  Summary
    Lists the different states that SPI task routine can have.

  Description
    This enumeration lists the different states that SPI task routine can have.

  Remarks:
    None.
*/

typedef enum
{
    /* Process queue */
    DRV_SPI_TASK_PROCESS_QUEUE,

    /* SPI task handle read only buffer request */
    DRV_SPI_TASK_PROCESS_READ_ONLY,

    /* SPI task handle write only buffer request */
    DRV_SPI_TASK_PROCESS_WRITE_ONLY,

    /* SPI task handle read and write buffers request */
    DRV_SPI_TASK_PROCESS_WRITE_READ

} DRV_SPI_DATA_OBJECT_TASK;


// *****************************************************************************
/* SPI Driver task states

  Summary
    Lists the SPI operations.

  Description
    This enumeration lists all the SPI operations.

  Remarks:
    None.
*/

typedef enum
{
    /* SPI operation read only buffer request */
    DRV_SPI_OP_READ,

    /* SPI operation write only buffer request */
    DRV_SPI_OP_WRITE,

    /* SPI operation read and write buffers request */
    DRV_SPI_OP_READ_WRITE

} DRV_SPI_OPERATIONS;


// *****************************************************************************
/* SPI instance address

  Summary:
    Gets the SPI instance address from its index.

  Description:
    This macro gets the SPI instance address from its index.

  Remarks:
    None.
*/

#define _DRV_SPI_INSTANCE_GET(object)        &gDrvSPIObj[object]


// *****************************************************************************
/* SPI chip select

  Summary:
    Asserts the SPI chip select pin.

  Description:
    This macro redirects to PORTS system service function to select the SD card
    by making the I/O pin low.

  Remarks:
    None.
*/

#define _DRV_SPI_CHIP_SELECT_CLEAR(port,pin)    	SYS_PORTS_PinClear(PORTS_ID_0,port,pin)


// *****************************************************************************
/* SPI chip de-select

  Summary:
    Makes the SPI slave not selected.

  Description:
    This macro redirects to PORTS system service function to de-select the SPI
    by making the I/O pin high.

  Remarks:
    None.
*/

#define _DRV_SPI_CHIP_SELECT_SET(port,pin) 	SYS_PORTS_PinSet(PORTS_ID_0,port,pin)


// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* SPI Driver Data Object

  Summary:
    Defines the object required for the maintainence of driver operation queue.

  Description:
    This defines the object required for the maintainence of the software
    clients instance. This object exists once per client instance.

  Remarks:
    None.
*/

typedef struct _DRV_SPI_BUFFER_OBJECT
{

    /* A flag states whether the buffer is in use or not */
    bool                                                    inUse;

    /* A reference to the client instance */
    DRV_HANDLE                                              clientHandle;

    /* Specifies the operation */
    DRV_SPI_OPERATIONS                                      operation;

    /* Pointer to transmit buffer */
    uint8_t *                                               txBuffer;

    /* Pointer to receive buffer */
    uint8_t *                                               rxBuffer;

    /* size of buffer for transfer */
    uint32_t                                                transferSize;

    /* Transfer status */
    DRV_SPI_BUFFER_EVENT                                   status;

    /* Pointer to the next element in the queue */
    void                                                    *next;

}DRV_SPI_BUFFER_OBJECT;


// *****************************************************************************
/* SPI Driver Hardware Instance Object

  Summary:
    Defines the object required for the maintenance of the hardware instance.

  Description:
    This defines the object required for the maintenance of the hardware
    instance. This object exists once per hardware instance of the peripheral.

  Remarks:
    None.
*/

typedef struct _DRV_SPI_OBJ
{
    /* The status of the driver */
    SYS_STATUS                                  status;

    /* The peripheral Id associated with the object */
    SPI_MODULE_ID                               spiId;

    /* Save the index of the driver. Important to know this
    as we are using reference based accessing */
    SYS_MODULE_INDEX                            drvIndex;

    /* Flag to indicate in use  */
    bool                                        inUse;

    /* Flag to indicate that SPI is used in exclusive access mode */
    bool                                        isExclusive;

    /* Number of clients possible with the hardware instance */
    uint8_t                                     numClients;

    /* Hardware initialization parameters */
    /* SPI Usage Mode Type */
    DRV_SPI_MODE                                spiMode;

    /* Communication Width */
    SPI_COMMUNICATION_WIDTH                     commWidth;

    /* Protocol Type */
    DRV_SPI_PROTOCOL_TYPE                       spiProtocolType;

    /* Buffer Type */
    DRV_SPI_BUFFER_TYPE                         bufferType;

    /* SPI Clock mode */
    DRV_SPI_CLOCK_MODE                          clockMode;

    /* SPI Input Sample Phase Selection */
    SPI_INPUT_SAMPLING_PHASE                    inputSamplePhase;

    /* SPI Output Data Phase Selection */
    SPI_OUTPUT_DATA_PHASE                       outputDataPhase;

    /* SPI Clock Polarity */
    SPI_CLOCK_POLARITY                          clkPolarity;

    /* transfer status */
    DRV_SPI_BUFFER_EVENT                       	transferStatus;

    /* Transmit/Receive or Transmit Interrupt
    Source for SPI module */
    INT_SOURCE                                  txInterruptSource;

    /* Receive Interrupt Source for
    SPI module */
    INT_SOURCE                                  rxInterruptSource;

    /* Error Interrupt Source for
    SPI module */
    INT_SOURCE                                  errInterruptSource;

    /* Locally used variables */
    /* Buffer length */
    unsigned int                                bufLength;

    /* Pointer to tx buffer */
    uint8_t *                                   txBuffer;

    /* Pointer to tx buffer */
    uint8_t *                                   rxBuffer;

    /* size of buffer for transmission */
    uint32_t                                    txSize;

    /* size of buffer for reception */
    uint32_t                     		rxSize;

    /* State of the task */
    DRV_SPI_DATA_OBJECT_TASK    		task;

    /* Queue head is specific to the instance */
    DRV_SPI_BUFFER_OBJECT                       *queueHead;

    /* Store the last client handle for every task */
    DRV_HANDLE                                  lastClientHandle;

    DRV_SPI_BUFFER_OBJECT                       *taskLObj;

} DRV_SPI_OBJ;


typedef unsigned int DRV_SPI_CLIENT_OBJ_HANDLE;
typedef unsigned int DRV_SPI_OBJ_HANDLE;


// *****************************************************************************
/* SPI Driver Client Object

  Summary:
    Defines the object required for the maintenance of the software clients.

  Description:
    This defines the object required for the maintenance of the software
    clients instance. This object exists once per client instance.

  Remarks:
    None.
*/

typedef struct _DRV_SPI_CLIENT_OBJ
{
    /* Driver Object associated with the client */
    DRV_SPI_OBJ*                                      	driverObject;
    /* Flag to indicate in use  */
    bool                                                inUse;

    /* The intent with which the client was opened */
    DRV_IO_INTENT                                      	intent;

    /* SPI Input Sample Phase Selection */
    SPI_INPUT_SAMPLING_PHASE                            inputSamplePhase;

        /* SPI Clock mode */
    DRV_SPI_CLOCK_MODE                                  clockMode;

    /* Chip Select logic level */
    bool                                                chipSelectLogicLevel;

    /* PORT which the chip select pin belongs to */
    PORTS_CHANNEL 					chipSelectPort;

    /* Bit position in the port */
    PORTS_BIT_POS 					chipSelectBitPos;

    /* Baud Rate Value */
    uint32_t                                            baudRate;

    /* Save the context, will be passed back with the call back */
    uintptr_t                   			context;

    /* SPI callback */
    DRV_SPI_BUFFER_EVENT_HANDLER                        callback;

} DRV_SPI_CLIENT_OBJ;


// *****************************************************************************
// *****************************************************************************
// Section: Extern data Definitions
// *****************************************************************************
// *****************************************************************************



// *****************************************************************************
/*  Hardware Objects for the static driver
*/

extern DRV_SPI_OBJ            gDrvSPIObj0;
extern DRV_SPI_OBJ            gDrvSPIObj1;
extern DRV_SPI_OBJ            gDrvSPIObj2;
extern DRV_SPI_OBJ            gDrvSPIObj3;
extern DRV_SPI_OBJ            gDrvSPIObj4;
extern DRV_SPI_OBJ            gDrvSPIObj5;
extern DRV_SPI_OBJ            gDrvSPIObj6;


// *****************************************************************************
/*  Client Objects for the single-client driver
*/

extern DRV_SPI_CLIENT_OBJ     gDrvSPIClientObj0;
extern DRV_SPI_CLIENT_OBJ     gDrvSPIClientObj1;
extern DRV_SPI_CLIENT_OBJ     gDrvSPIClientObj2;
extern DRV_SPI_CLIENT_OBJ     gDrvSPIClientObj3;
extern DRV_SPI_CLIENT_OBJ     gDrvSPIClientObj4;
extern DRV_SPI_CLIENT_OBJ     gDrvSPIClientObj5;
extern DRV_SPI_CLIENT_OBJ     gDrvSPIClientObj6;

DRV_SPI_BUFFER_OBJECT* _DRV_SPI_QueueSlotGet ( DRV_SPI_OBJ *dObj );
bool _DRV_SPI_IsQueueNotEmpty ( DRV_HANDLE handle );
DRV_SPI_BUFFER_OBJECT* _DRV_SPI_QueuePop ( SYS_MODULE_OBJ object );
//static void _DRV_SPI_ClientHardwareSetup ( DRV_HANDLE handle );
#endif //#ifndef _DRV_SPI_LOCAL_H

/*******************************************************************************
 End of File
*/

