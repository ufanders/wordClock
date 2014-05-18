/*******************************************************************************
  USB Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usb_local.h

  Summary:
    USB driver local declarations and definitions

  Description:
    This file contains the USB driver's local declarations and definitions.
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

#ifndef _DRV_USB_LOCAL_H
#define _DRV_USB_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "system_config.h"
#include "system/debug/sys_debug.h"
#include "driver/usb/drv_usb.h"
#include "driver/usb/usbfs/src/drv_usb_variant_mapping.h"
// *****************************************************************************
// *****************************************************************************
// Section: Version Numbers
// *****************************************************************************
// *****************************************************************************
/* Versioning of the driver */

// *****************************************************************************
/* USB Driver Version Macros

  Summary:
    USB driver version.

  Description:
    These constants provide USB driver version information. The driver
    version is:
    DRV_USB_VERSION_MAJOR.DRV_USB_VERSION_MINOR.DRV_USB_VERSION_PATCH.
    It is represented in DRV_USB_VERSION as:
    MAJOR *10000 + MINOR * 100 + PATCH, so as to allow comparisons.
    It is also represented in string format in DRV_USB_VERSION_STR.
    DRV_USB_TYPE provides the type of the release when the release is alpha
    or beta. The interfaces DRV_USB_VersionGet and DRV_USB_VersionStrGet 
    provide interfaces to the access the version and the version string.

  Remarks:
    Modify the return value of DRV_USB_VersionStrGet and the
    DRV_USB_VERSION_MAJOR, DRV_USB_VERSION_MINOR,
    DRV_USB_VERSION_PATCH, and DRV_USB_VERSION_TYPE.
*/

#define _DRV_USB_VERSION_MAJOR         0
#define _DRV_USB_VERSION_MINOR         2
#define _DRV_USB_VERSION_PATCH         0
#define _DRV_USB_VERSION_TYPE          Alpha
#define _DRV_USB_VERSION_STR           "0.2.0 Alpha"

#define DRV_USB_HOST_IRP_PER_FRAME_NUMBER        5
#define _DRV_USB_SW_EP_NUMBER DRV_USB_HOST_IRP_PER_FRAME_NUMBER

#define DRV_USB_MAX_CONTROL_BANDWIDTH      20
#define DRV_USB_MAX_BANDWIDTH_PER_FRAME    70

#define USB_TRANSFER_TYPE_LOCAL_CONTROL          0
#define USB_TRANSFER_TYPE_LOCAL_INTERRUPT        1
#define USB_TRANSFER_TYPE_LOCAL_BULK             2
#define USB_TRANSFER_TYPE_LOCAL_ISOC             3

// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

/**********************************************
 * BDT entry union type
 *********************************************/

typedef union
{
    uint8_t     byte[8];
    uint16_t    shortWord[4];
    uint32_t    word[2];
}
DRV_USB_BDT_ENTRY;

#define DRV_USB_CLIENTS_NUMBER DRV_USB_INSTANCES_NUMBER

/***************************************************
 * This is an intermediate flag that is set by
 * the driver to indicate that a ZLP should be sent
 ***************************************************/
#define USB_DEVICE_IRP_FLAG_SEND_ZLP 0x80

/***************************************************
 * This object is used by the driver as IRP place
 * holder along with queueing feature.
 ***************************************************/
typedef struct _USB_DEVICE_IRP_LOCAL 
{
    /* Pointer to the data buffer */
    void * data;

    /* Size of the data buffer */
    unsigned int size;

    /* Status of the IRP */
    USB_DEVICE_IRP_STATUS status;

    /* IRP Callback. If this is NULL,
     * then there is no callback generated */
    void (*callback)(struct _USB_DEVICE_IRP * irp);

    /* Request specific flags */
    USB_DEVICE_IRP_FLAG flags;

    /* User data */
    uintptr_t userData;

    /* This points to the next IRP in the queue */
    struct _USB_DEVICE_IRP_LOCAL * next;

    /* This points to the previous IRP in the queue */
    struct _USB_DEVICE_IRP_LOCAL * previous;

    /* Pending bytes in the IRP */
    uint32_t nPendingBytes;
}
USB_DEVICE_IRP_LOCAL;


/************************************************
 * Endpoint state enumeration.
 ************************************************/
typedef enum
{
    DRV_USB_DEVICE_ENDPOINT_STATE_ENABLED = 0x1,
    DRV_USB_DEVICE_ENDPOINT_STATE_STALLED = 0x2
}
DRV_USB_DEVICE_ENDPOINT_STATE;

/************************************************
 * Endpoint data strucutre. This data structure
 * holds the IRP queue and other flags associated
 * with functioning of the endpoint.
 ************************************************/

typedef struct
{
    /* This is the IRP queue for 
     * the endpoint */
    USB_DEVICE_IRP_LOCAL * irpQueue;

    /* Max packet size for the endpoint */
    uint16_t maxPacketSize;

    /* Endpoint type */
    USB_TRANSFER_TYPE endpointType;

    /* Data Toggle */
    USB_BUFFER_DATA01 nextDataToggle;

    /* Ping pong buffer indicator */
    USB_BUFFER_PING_PONG nextPingPong;

    /* Endpoint state bitmap */
    DRV_USB_DEVICE_ENDPOINT_STATE endpointState;

}
DRV_USB_DEVICE_ENDPOINT_OBJ;

/*********************************************
 * These IRP states are used internally by the
 * HCD to track completion of a host IRP. This
 * is not the same as the public IRP status
 *********************************************/
typedef enum
{   
    DRV_USB_HOST_IRP_STATE_SETUP_STAGE,
    DRV_USB_HOST_IRP_STATE_SETUP_TOKEN_SENT,
    DRV_USB_HOST_IRP_STATE_DATA_STAGE,
    DRV_USB_HOST_IRP_STATE_DATA_STAGE_SENT,
    DRV_USB_HOST_IRP_STATE_HANDSHAKE,
    DRV_USB_HOST_IRP_STATE_HANDSHAKE_SENT,
    DRV_USB_HOST_IRP_STATE_COMPLETE,
    DRV_USB_HOST_IRP_STATE_ABORTED
}
DRV_USB_HOST_IRP_STATE; 

/*********************************************
 * This is the local USB Host IRP object
 ********************************************/
typedef struct _USB_HOST_IRP_LOCAL
{
    /* Points to the 8 byte setup command
     * packet in case this is a IRP is 
     * scheduled on a CONTROL pipe. Should
     * be NULL otherwise */
    void * setup;

    /* Pointer to data buffer */
    void * data;
    
    /* Size of the data buffer */
    unsigned int size;
    
    /* Status of the IRP */ 
    USB_HOST_IRP_STATUS status;

    /* Request specific flags */
    USB_HOST_IRP_FLAG flags;

    /* User data */
    uintptr_t userData;

    /* Pointer to function to be called
     * when IRP is terminated. Can be 
     * NULL, in which case the function
     * will not be called. */
    void (*callback)(struct _USB_HOST_IRP * irp);

    /****************************************
     * These members of the IRP should not be
     * modified by client
     ****************************************/

    uint32_t tempSize;
    DRV_USB_HOST_IRP_STATE tempState;
    uint32_t completedBytes;
    uint32_t completedBytesInThisFrame;
    struct _USB_HOST_IRP_LOCAL * next;
    DRV_USB_HOST_PIPE_HANDLE  pipe;

}
USB_HOST_IRP_LOCAL;

/************************************************
 * This is the Host Pipe Object.
 ************************************************/
typedef struct _DRV_USB_HOST_PIPE_OBJ
{
    /* This pipe object is in use */
    bool inUse;

    /* Client that owns this pipe */
    DRV_HANDLE hClient;

    /* The device address */
    uint8_t deviceAddress;
    
    /* USB endpoint and direction */
    USB_ENDPOINT endpointAndDirection;
    
    /* Interval in case this
     * is a Isochronous or
     * an interrupt pipe */
    uint8_t bInterval;
    
    /* USB Endpoint type */
    USB_TRANSFER_TYPE pipeType;

    /* The IRP queue on this pipe */
    USB_HOST_IRP_LOCAL * irpQueueHead;

    /* The data toggle for this pipe*/
    USB_BUFFER_DATA01 dataToggle;

    /* The NAK counter for the IRP
     * being served on the pipe */
    
    uint32_t nakCounter;

    /* Pipe endpoint size*/

    unsigned int endpointSize;

    /* The next pipe */
    struct _DRV_USB_HOST_PIPE_OBJ * next;

    /* Interval counter */

    uint8_t intervalCounter;

    /* Bandwidth */
    uint8_t bwPerTransaction;

    /* Pipe Speed */
    USB_SPEED speed;
}
DRV_USB_HOST_PIPE_OBJ;

/************************************************
 * This is the Host SW EP Object.
 ************************************************/
typedef struct _DRV_USB_HOST_SW_EP
{
    bool tobeDone;
    USB_TRANSFER_TYPE transferType;
    USB_HOST_IRP_LOCAL * pirp;
}
DRV_USB_HOST_SW_EP;

/*********************************************
 * Host Transfer Group. This data structures
 * contains all pipes belonging to one tranfer
 * type.
 *********************************************/

typedef struct _DRV_USB_HOST_TRANSFER_GROUP
{
    /* The first pipe in this transfer 
     * group */
    DRV_USB_HOST_PIPE_OBJ * pipe;
    
    /* The current pipe being serviced
     * in this transfer group */
    DRV_USB_HOST_PIPE_OBJ * currentPipe;
    
    /* The current IRP being serviced
     * in the pipe */
    void * currentIRP;

    /* Total number of pipes in this
     * transfer group */
    int nPipes;
}
DRV_USB_HOST_TRANSFER_GROUP;

/********************************************
 * This enumeration list the possible status
 * valus of a token once it has completed.
 ********************************************/

typedef enum
{
    USB_TRANSACTION_ACK             = 0x2,
    USB_TRANSACTION_NAK             = 0xA,
    USB_TRANSACTION_STALL           = 0xE,
    USB_TRANSACTION_DATA0           = 0x3,
    USB_TRANSACTION_DATA1           = 0xB,
    USB_TRANSACTION_BUS_TIME_OUT    = 0x0,
    USB_TRANSACTION_DATA_ERROR      = 0xF

}
DRV_USB_TRANSACTION_RESULT;

/**********************************************
 * Driver Client Obj
 **********************************************/

typedef struct _DRV_USB_CLIENT_OBJ_STRUCT
{
    /* Indicates that this object is in use */
    bool inUse;

    /* Client data that will be returned at callback */ 
    uintptr_t  hClientArg;

    /* Index of the driver that own this object */
    _DRV_USB_FOR_DYNAMIC(void* ,  hDriver); 

    /* Call back function for this client */
    DRV_USB_EVENT_CALLBACK  pEventCallBack;     
    
    /* PLIB USB Module ID */ 
    _DRV_USB_FOR_DYNAMIC(USB_MODULE_ID, usbID);      

} DRV_USB_CLIENT_OBJ;


/***********************************************
 * Driver object structure. One object per
 * hardware instance
 **********************************************/

typedef struct _DRV_USB_OBJ_STRUCT
{
    /* Indicates this object is in use */
    bool    inUse;

    /* Indicates USB module associated is enabled */
    bool isEnabled;

    /* Status of this driver instance */
    SYS_STATUS status;     

    /* The USB peripheral associated with 
     * the object */
     _DRV_USB_FOR_DYNAMIC(USB_MODULE_ID, usbID);      

    /* Current operating mode of the driver */
    USB_OPMODES operationMode;

    /* Interrupt source for USB module */
    _DRV_USB_FOR_DYNAMIC(INT_SOURCE, interruptSource);    

    /* Number of clients */
    unsigned int nClients;

    /* Client of this driver */
    DRV_USB_CLIENT_OBJ * pDrvUSBClientObj;

    /* Pointer to the BDT table for this
     * particular instance of the USB module */
    DRV_USB_BDT_ENTRY* pBDT;

    /* Pointer to the endpoint table */
    _DRV_USB_FOR_DYNAMIC(DRV_USB_DEVICE_ENDPOINT_OBJ, *endpointTable);

    /* The object is current in an interrupt context */
    bool isInInterruptContext;

    /* Maintains the timer count value for host */
    _DRV_USB_FOR_HOST(uint32_t, timerCount);

    /* Root Hub Port 0 attached device speed */
    _DRV_USB_FOR_HOST(USB_SPEED, deviceSpeed);

    /* Transfer Groups */
    _DRV_USB_FOR_HOST(DRV_USB_HOST_TRANSFER_GROUP, transferGroup[4]);

    /* The SWEPBuffer index */
    _DRV_USB_FOR_HOST(uint8_t, numSWEpEntry);

    /* EP0 TX Ping Pong Tracker */
    _DRV_USB_FOR_HOST(USB_BUFFER_PING_PONG, ep0TxPingPong);
    
    /* EP0 TX Ping Pong Tracker */
    _DRV_USB_FOR_HOST(USB_BUFFER_PING_PONG, ep0RxPingPong);

    /* Placeholder for bandwidth consumed in frame */
    _DRV_USB_FOR_HOST(uint8_t, globalBWConsumed);

    /*
     * Variable used SW Endpoint objects that is used by this
     * HW instances for USB transfer scheduling
     */
    _DRV_USB_FOR_HOST(DRV_USB_HOST_SW_EP, drvUSBHostSWEp
            [_DRV_USB_SW_EP_NUMBER]);

} DRV_USB_OBJ;


/***************************************************
 * This type defintion allows creation of multiple
 * aligned BDT arrays. Clubbing all members together
 * help the linker to reduce memory gaps
 ***************************************************/

typedef struct
{
    DRV_USB_OBJ gDrvUSBObj;
    DRV_USB_CLIENT_OBJ gDrvUSBClientObj;
    DRV_USB_DEVICE_ENDPOINT_OBJ gDrvUSBEndpoints[DRV_USB_ENDPOINTS_NUMBER * 2];

}
DRV_USB_GROUP;


/**************************************
 * Local functions.
 *************************************/

void _DRV_USB_DEVICE_Initialize(DRV_USB_OBJ * drvObj, SYS_MODULE_INDEX index);
void _DRV_USB_DEVICE_Tasks_ISR(DRV_USB_OBJ * hDriver);
void _DRV_USB_HOST_Initialize(DRV_USB_OBJ * const pusbdrvObj,
                              const SYS_MODULE_INDEX index);
void _DRV_USB_HOST_Tasks_ISR(DRV_USB_OBJ * pusbdrvObj);
bool _DRV_USB_HOST_TransferSchedule(DRV_USB_OBJ * pusbdrvObj,
                                    DRV_USB_TRANSACTION_RESULT lastResult,
                                    unsigned int transactionSize,
                                    bool frameExpiry);
void _DRV_USB_SendTokenToAddress
(
    USB_MODULE_ID usbID,
    uint8_t address,
    USB_PID pid,
    uint8_t endpoint,
    bool isLowSpeed
);
bool _DRV_USB_HOST_NonControlIRPProcess
(
    DRV_USB_OBJ * pusbdrvObj,
    USB_HOST_IRP_LOCAL * pirp,
    DRV_USB_TRANSACTION_RESULT lastTransactionResult,
    int lastTransactionsize
);
bool _DRV_USB_HOST_ControlXferProcess
(
    DRV_USB_OBJ * pusbdrvObj,
    USB_HOST_IRP_LOCAL * pirp,
    DRV_USB_TRANSACTION_RESULT deviceResponse,
    unsigned int deviceResponseSize
);
void _DRV_USB_HOST_Calculate_Control_BW
(
    DRV_USB_OBJ * pusbdrvObj,
    DRV_USB_HOST_TRANSFER_GROUP * ptransferGroup,
    USB_HOST_IRP_LOCAL * pcontrolIRP
);

bool _DRV_USB_HOST_Calculate_NonControl_BW
(
    DRV_USB_OBJ * pusbdrvObj,
    DRV_USB_HOST_TRANSFER_GROUP * ptransferGroup,
    USB_HOST_IRP_LOCAL * ptransferIRP,
    USB_TRANSFER_TYPE transferType,
    uint8_t numSWEpEntry
);
void _DRV_USB_HOST_NonControl_Send_Token
(
    USB_HOST_IRP_LOCAL * pirp,
    DRV_USB_OBJ *pusbdrvObj,
    DRV_USB_HOST_PIPE_OBJ *pipe,
    bool isLowSpeed
);
void _DRV_USB_HOST_Control_Send_Token
(
    USB_HOST_IRP_LOCAL * pirp,
    DRV_USB_OBJ *pusbdrvObj,
    DRV_USB_HOST_PIPE_OBJ *pipe,
    uint8_t endpoint,
    uint8_t deviceAddress,
    USB_MODULE_ID usbID,
    bool isLowSpeed
);


#endif
