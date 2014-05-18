/*******************************************************************************
  USB Device Driver Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usb_host.c

  Summary:
    USB Device Driver Implementation

  Description:
    This file implements the Host mode operation of the USB Driver. This file
    should be included in the application if USB Host mode operation is desired.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
//DOM-IGNORE-END

#include "system_config.h"
#include "driver/usb/drv_usb.h"
#include "driver/usb/usbfs/src/drv_usb_local.h"

/*
 * Global Variable used as Pool of pipe objects that is used by all
 * driver instances.
 */
DRV_USB_HOST_PIPE_OBJ gDrvUSBHostPipeObj[DRV_USB_HOST_PIPES_NUMBER];

const unsigned int gDrvUSBFSTableBW[4][11] =
{{3, 3, 3, 4, 4, 5, 7, 0, 0,  0,  0},   //control
{1, 1, 1, 1, 2, 3, 5, 0, 0,  0,  0},    //interrupt
{1, 1, 1, 1, 2, 3, 5, 0, 0,  0,  0},    //bulk
{1, 1, 1, 1, 2, 3, 5, 9, 18, 35, 69}};   //isoc

const unsigned int gDrvUSBLSTableBW[2][4] ={{26, 27, 28,30},     // control
{11, 11, 12, 14}};   // interrupt

// *****************************************************************************
/* Function:
    void _DRV_USB_SendTokenToAddress
    (
        USB_MODULE_ID usbID,
        uint8_t address,
        USB_PID pid,
        uint8_t endpoint,
        bool isLowSpeed
    )

  Summary:
    Dynamic impementation of _DRV_USB_SendTokenToAddress function

  Description:
    Function programs USB token register with required information for
    sending USB token data

  Remarks:
    This is a local function and should not be called directly by the
    application.

*/

void _DRV_USB_SendTokenToAddress
(
    USB_MODULE_ID usbID,
    uint8_t address,
    USB_PID pid,
    uint8_t endpoint,
    bool isLowSpeed
)
{
    /* Created a function call for this because PLIB function are 
     * inline and this function is being called at several locations */

    PLIB_USB_TokenSend(usbID, pid, endpoint, address, isLowSpeed);

}/* end of _DRV_USB_SendTokenToAddress() */

// *****************************************************************************
/* Function:
    void _DRV_USB_HOST_Initialize
    (
        DRV_USB_OBJ * const pdrvObj,
        const SYS_MODULE_INDEX index
    )

  Summary:
    Dynamic impementation of _DRV_USB_HOST_Initialize function when controller
    is in Host mode.

  Description:
    Function performs the following tasks:
      - Informs the USB module with SOF threshold in bit times
      - Enables VBUS power and initializes the module in HOST mode
      - Resets the BDT table data structure with init value
      - Configures EP0 register for the specific USB module
      - Enables the USB attach interrupt

  Remarks:
    This is a local function and should not be called directly by the
    application.
  
*/

void _DRV_USB_HOST_Initialize
(
    DRV_USB_OBJ * const pdrvObj,
    const SYS_MODULE_INDEX index
)
{
    /* Start of local variable */
    uint8_t bdtEntryindex = 0;
    /* End of local variable */

    /* Set the SOF threshold value in bit times */
    PLIB_USB_SOFThresholdSet(pdrvObj->usbID, 0x4A);
    /* Enable the VBUSON bit in the OTGCON register */
    PLIB_USB_OTG_VBusPowerOn(pdrvObj->usbID);
    /* Select the host mode of operation */
    PLIB_USB_OperatingModeSelect(pdrvObj->usbID, USB_OPMODE_HOST);

    /* Clear up the endpoint 0 BDT entries */
    for(bdtEntryindex = 0; bdtEntryindex < 4; bdtEntryindex ++)
    {
        /* A full duplex endpoint has 4
         * entries, 2 per EP direction */
        pdrvObj->pBDT[bdtEntryindex].word[0] = 0;
        pdrvObj->pBDT[bdtEntryindex].word[1] = 0;
    }
    /* Initialize the odd even buffer pointers */
    pdrvObj->ep0TxPingPong = USB_BUFFER_EVEN;
    pdrvObj->ep0RxPingPong = USB_BUFFER_EVEN;

    /* Configure endpoint 0 register. */
    PLIB_USB_EP0HostSetup(pdrvObj->usbID);
    /* Enable the attach interrupt */
    PLIB_USB_InterruptEnable(pdrvObj->usbID, USB_INT_ATTACH);

}/* end of _DRV_USB_HOST_Initialize() */

// *****************************************************************************
/* Function:
    USB_ERROR DRV_USB_HOST_IRPSubmit
    (
        DRV_USB_HOST_PIPE_HANDLE  hPipe,
        USB_HOST_IRP * pinputIRP
    )

  Summary:
    Dynamic impementation of DRV_USB_HOST_IRPSubmit function when controller
    is in Host mode.

  Description:
    Function performs the following tasks:
      - Validates the pipe object for which IRP has been submitted
      - Adds the IRP to the pipe object data structure

  Remarks:
    See drv_usb.h for usage information.
*/

USB_ERROR DRV_USB_HOST_IRPSubmit
(
    DRV_USB_HOST_PIPE_HANDLE  hPipe,
    USB_HOST_IRP * pinputIRP
)
{
  
    /* Start of local variables */
    USB_HOST_IRP_LOCAL * pirpIterator = (USB_HOST_IRP_LOCAL *)NULL;
    bool interruptWasEnabled          = false;
    USB_HOST_IRP_LOCAL * pirp         = (USB_HOST_IRP_LOCAL *)pinputIRP;
    DRV_USB_HOST_PIPE_OBJ * ppipe     = (DRV_USB_HOST_PIPE_OBJ *)hPipe;
    DRV_USB_OBJ * pusbdrvObj          = (DRV_USB_OBJ *)NULL;
    DRV_USB_CLIENT_OBJ * pusbdrvClient= (DRV_USB_CLIENT_OBJ *)(ppipe->hClient);
    /* End of local variables */
    
    /* Check if valid client object the pipe points to */
    if((pusbdrvClient == NULL) || (!pusbdrvClient->inUse))
    {
        SYS_DEBUG(0, "Illegal pipe or client already closed");
        return USB_ERROR_HOST_PIPE_INVALID;
    }
    
    pusbdrvObj = (DRV_USB_OBJ *)pusbdrvClient->hDriver;

    /* Assign owner pipe */
    pirp->pipe = hPipe;
    /* Clear up any temporary state */
    pirp->tempState = 0;

    /* Check the transfer type */
    if(ppipe->pipeType == USB_TRANSFER_TYPE_CONTROL)
    {
        /* Then we setup the IRP for setup stage */
        pirp->tempState = DRV_USB_HOST_IRP_STATE_SETUP_STAGE;
    }

    pirp->status = USB_HOST_IRP_STATUS_PENDING;

    /* Add the IRP to the pipe. We need to disable
     * the SOF interrupt here because the SOF interrupt
     * updates the pipe structure asynchronously. */

    if(!pusbdrvObj->isInInterruptContext)
    {
        // OSAL: Get Mutex
        interruptWasEnabled =
                _DRV_USB_InterruptSourceDisable(pusbdrvObj->interruptSource);
    }
    
    if(ppipe->irpQueueHead == NULL)
    {
        /* This means that there are no
         * IRPs on this pipe. We can add
         * this IRP directly */
        pirp->next = NULL;
        ppipe->irpQueueHead = pirp;
    }
    else
    {
        pirpIterator = ppipe->irpQueueHead;

        /* Find the last IRP in the linked list*/
        while(pirpIterator->next != NULL)
        {
            pirpIterator = pirpIterator->next;
        }

        /* Add the item to the last irp in the linked list */
        pirpIterator->next = pirp;
    }

    /* Initialize the hidden members */
    pirp->next = NULL;
    pirp->completedBytes = 0;
    pirp->tempSize = 0;

    if(!pusbdrvObj->isInInterruptContext)
    {
        if(interruptWasEnabled)
        {
            _DRV_USB_InterruptSourceEnable(pusbdrvObj->interruptSource);
        }
        //OSAL: Return Mutex
    }
    return USB_ERROR_NONE;

}/* end of DRV_USB_HOST_IRPSubmit() */

// *****************************************************************************
/* Function:
    void DRV_USB_HOST_IRPCancel(USB_HOST_IRP * pinputIRP)


  Summary:
    Dynamic impementation of DRV_USB_HOST_IRPCancel function when controller
    is in Host mode.

  Description:
    Function cancels and IRP

  Remarks:
    See drv_usb.h for usage information.
*/

void DRV_USB_HOST_IRPCancel(USB_HOST_IRP * pinputIRP)
{
    /* Start of local variable */
    USB_HOST_IRP_LOCAL * pirp          = (USB_HOST_IRP_LOCAL *) pinputIRP;
    DRV_USB_OBJ * pusbdrvObj           = (DRV_USB_OBJ *)NULL;
    DRV_USB_CLIENT_OBJ * pusbdrvClient = (DRV_USB_CLIENT_OBJ *)NULL;
    DRV_USB_HOST_PIPE_OBJ * ppipe      = (DRV_USB_HOST_PIPE_OBJ *)NULL;
    USB_HOST_IRP_LOCAL * piteratorIRP  = (USB_HOST_IRP_LOCAL *)NULL;
    bool interruptWasEnabled           = false;
    bool irpCancel                     = false;
    /* End of local variable */

    if((pirp == NULL) ||
            (pirp->pipe == DRV_USB_HOST_PIPE_HANDLE_INVALID))
    {
        SYS_DEBUG(0, "Invalid pipe");
    }
    else if(pirp->status <= USB_HOST_IRP_STATUS_COMPLETED_SHORT)
    {
        SYS_DEBUG(0, "IRP is not pending or in progress");
    }
    else
    {
        ppipe = (DRV_USB_HOST_PIPE_OBJ *)pirp->pipe;
        pusbdrvClient = (DRV_USB_CLIENT_OBJ *) ppipe->hClient;
        pusbdrvObj = (DRV_USB_OBJ *) pusbdrvClient->hDriver;

        if(!pusbdrvObj->isInInterruptContext)
        {
            //OSAL: Get Mutex
            interruptWasEnabled =
                _DRV_USB_InterruptSourceDisable(pusbdrvObj->interruptSource);
        }
        
        piteratorIRP = ppipe->irpQueueHead;

	if(piteratorIRP == pirp)
	{
	    /* Scenario - IRP to be cancelled is 1st IRP in the pipe */
	    ppipe->irpQueueHead = piteratorIRP->next;
	    irpCancel = true;
	}
	else
	{
	    /* Scenario - IRP to be cancelled is NOT the 1st IRP in the pipe */
	    while(piteratorIRP != NULL)
	    {
		if(piteratorIRP->next == pirp)
		{
		    /* IRP to be cancelled has been found */
		    piteratorIRP->next = (piteratorIRP->next)->next;
		    irpCancel = true;
		    break;
		}
		piteratorIRP = piteratorIRP->next;
	    }
	}
	/* Check if IRP has been cancelled */
	if(irpCancel == true)
	{
	    if(pirp->status == USB_HOST_IRP_STATUS_IN_PROGRESS)
	    {
		/* If the irp is already in progress then
		 * we set the temporary state. This will get
		 * caught in _DRV_USB_HOST_ControlXferProcess()
		 * and _DRV_USB_HOST_NonControlIRPProcess()
		 * functions. */
		pirp->tempState = DRV_USB_HOST_IRP_STATE_ABORTED;
	    }
	    else
	    {
		pirp->status = USB_HOST_IRP_STATUS_ABORTED;
		if(pirp->callback != NULL)
		{
		    pirp->callback((USB_HOST_IRP *)pirp);
		}
	    }
	}

        if(piteratorIRP == NULL)
        {
            /* Either no IRP at all in the Pipe
             * or particular IRP not found
             */
            SYS_DEBUG(0, "IRP not found");
        }

        if(!pusbdrvObj->isInInterruptContext)
        {
            if(interruptWasEnabled)
            {
                _DRV_USB_InterruptSourceDisable(pusbdrvObj->interruptSource);
            }
            //OSAL: Release Mutex
        }
    }
}/* end of DRV_USB_HOST_IRPCancel() */

// *****************************************************************************
/* Function:
    bool _DRV_USB_HOST_ControlXferProcess
    (
        DRV_USB_OBJ * hDriver,
        USB_HOST_IRP_LOCAL * irp,
        DRV_USB_TRANSACTION_RESULT deviceResponse,
        unsigned int deviceResponseSize
    )

  Summary:
    Dynamic impementation of _DRV_USB_HOST_ControlXferProcess internal function
    when controller is in Host mode.

  Description:
    Function is used to send\recieve CONTROL transfer token, data and status.
    Function returns true if a token was sent at the time when the it exits.

  Remarks:
    This is a local function and should not be called directly by the
    application.

*/

bool _DRV_USB_HOST_ControlXferProcess
(
    DRV_USB_OBJ * pusbdrvObj,
    USB_HOST_IRP_LOCAL * pirp,
    DRV_USB_TRANSACTION_RESULT deviceResponse,
    unsigned int deviceResponseSize
)
{

    /* This function returns true if a token was sent 
     * at the time when the function exits */

    /* Start of local variables */
    DRV_USB_BDT_ENTRY * pBDT        = (DRV_USB_BDT_ENTRY *)NULL;
    DRV_USB_HOST_PIPE_OBJ * pipe    = (DRV_USB_HOST_PIPE_OBJ *)pirp->pipe;
    uint8_t endpoint                = pipe->endpointAndDirection & 0xF;
    uint8_t deviceAddress           = pipe->deviceAddress;
    USB_MODULE_ID usbID             = pusbdrvObj->usbID;
    bool isLowSpeed                 = (pipe->speed == USB_SPEED_LOW) ?
                                       true : false;
    bool tokenSent                  = false;
    bool endIRP                     = false;
    /* End of local variables */

    if(pirp->tempState == DRV_USB_HOST_IRP_STATE_ABORTED)
    {
        /* This means the application aborted this IRP.
         * We just invoke the callback */

	pusbdrvObj->drvUSBHostSWEp[pusbdrvObj->numSWEpEntry].tobeDone = false;
        ((pusbdrvObj->transferGroup
            [pusbdrvObj->drvUSBHostSWEp[pusbdrvObj->numSWEpEntry].transferType])
                .currentPipe)->irpQueueHead =
            (pusbdrvObj->drvUSBHostSWEp[pusbdrvObj->numSWEpEntry].pirp)->next;

        /* Move the current pipe */
        if((pusbdrvObj->transferGroup[USB_TRANSFER_TYPE_CONTROL].currentPipe)
                ->next != NULL)
	{
	    /* This is not the last PIPE */
	    pusbdrvObj->transferGroup[USB_TRANSFER_TYPE_CONTROL].currentPipe =
		pusbdrvObj->transferGroup[USB_TRANSFER_TYPE_CONTROL].
                    currentPipe->next;
	}
	else
	{
	    /* This is the last PIPE. Move to the head pipe */
	    pusbdrvObj->transferGroup[USB_TRANSFER_TYPE_CONTROL].currentPipe =
                    pusbdrvObj->transferGroup[USB_TRANSFER_TYPE_CONTROL].pipe;
	}
        
        pirp->status = USB_HOST_IRP_STATUS_ABORTED;
        if(pirp->callback != NULL)
        {
            pirp->callback((USB_HOST_IRP *)pirp);
        }
        /* Return false indicating that no token 
         * was sent in this frame */
        return(false);
    }

    switch(pirp->tempState)
    {
        case DRV_USB_HOST_IRP_STATE_SETUP_STAGE:

            /* Then send the SETUP Token 
             * Let the irp know that the process
             * has started.
             *  */

            pirp->status = USB_HOST_IRP_STATUS_IN_PROGRESS;
            pirp->completedBytes = 0;

            if((((uint8_t *)(pirp->setup))[0] & 0x80) != 0)
            {
                /* Data stage direction is from device
                 * to host */
                pipe->endpointAndDirection |=
                        (USB_DATA_DIRECTION_DEVICE_TO_HOST << 7);
            }
            else
            {
                /* Data stage direction is from host to
                 * device */
                pipe->endpointAndDirection &= 0xF;
                pipe->endpointAndDirection |=
                        (USB_DATA_DIRECTION_HOST_TO_DEVICE << 7);
            }

            /* Keep track of the transaction */
            pirp->tempState = DRV_USB_HOST_IRP_STATE_SETUP_TOKEN_SENT;

            pBDT = pusbdrvObj->pBDT + 2 + pusbdrvObj->ep0TxPingPong;
            
            /* COnfigure the BDT Entry for setup packet
             * and Data 0 toggle */
            pBDT->shortWord[1]  = 8;
            pBDT->word[1]       = KVA_TO_PA(pirp->setup);
            pBDT->byte[0]       = 0x88;
            
            /* This will cause a Transaction interrupt */
            _DRV_USB_SendTokenToAddress(usbID, deviceAddress,
                    USB_PID_SETUP, endpoint, isLowSpeed);

            tokenSent = true;

            break;

        case DRV_USB_HOST_IRP_STATE_SETUP_TOKEN_SENT:
            /* Check the response */
            if(deviceResponse == USB_TRANSACTION_ACK)
            {
                if((pirp->data == NULL) || 
                  (pirp->size == 0))
                {
                    /* This means that this is a zero data stage
                     * transaction */

                    pirp->tempState = DRV_USB_HOST_IRP_STATE_HANDSHAKE;
                    pipe->nakCounter = 0;
                    break;
                }

                /* Setup the data toggle for the next stage
                 * and go to the next stage directly.
                 * Switch case fall through is intentional. */
                pipe->dataToggle = USB_BUFFER_DATA1;
                pipe->nakCounter = 0;
                pirp->tempState = DRV_USB_HOST_IRP_STATE_DATA_STAGE;
            }
            else
            {
                pirp->tempState = DRV_USB_HOST_IRP_STATE_COMPLETE;
                pirp->status = USB_HOST_IRP_STATUS_ERROR_UNKNOWN;
                endIRP = true;
                break;
            }

        case DRV_USB_HOST_IRP_STATE_DATA_STAGE:
            /* Send the data stage */

            _DRV_USB_HOST_Control_Send_Token(pirp,pusbdrvObj,pipe,endpoint,
                    deviceAddress,usbID,isLowSpeed);
            tokenSent = true;
            break;

        case DRV_USB_HOST_IRP_STATE_DATA_STAGE_SENT:
 
            /* Check the response to data stage here */
            if(deviceResponse == USB_TRANSACTION_NAK)
            {
                /* This means the device is not ready
                 * with the data. Rewind the state to
                 * the previous state. We will try again
                 * on the next SOF*/

                pirp->tempState = DRV_USB_HOST_IRP_STATE_DATA_STAGE;
                pipe->nakCounter ++;
                if(pipe->nakCounter >= DRV_USB_HOST_NAK_LIMIT)
                {
                    pirp->tempState = DRV_USB_HOST_IRP_STATE_COMPLETE;
                    pirp->status = USB_HOST_IRP_STATUS_ERROR_NAK_TIMEOUT;
                    endIRP = true;
                    pipe->nakCounter = 0;
                }
                break;

            }
            else if (deviceResponse == USB_TRANSACTION_STALL)
            {
                pipe->nakCounter = 0;
	        pirp->status = USB_HOST_IRP_STATUS_ERROR_STALL;
	        pirp->tempState =  DRV_USB_HOST_IRP_STATE_COMPLETE;
                endIRP = true;
                break;
            }
            else if ((deviceResponse == USB_TRANSACTION_DATA0 )
                    ||(deviceResponse == USB_TRANSACTION_DATA1)
                    ||(deviceResponse == USB_TRANSACTION_ACK))
            {
                /* Update the IRP with the amount of data
                 * received */
                pipe->nakCounter = 0;
                pirp->completedBytes += deviceResponseSize;
                pirp->completedBytesInThisFrame += deviceResponseSize;

                if((pipe->endpointAndDirection & 0x80) != 0)
                {
                    /* Data is moving from device to host 
                     * Check if the data stage is done */

                    if((deviceResponseSize < pipe->endpointSize)
                            ||(pirp->completedBytes >= pirp->size))
                    {
                        /* A host control read transfer is done
                         * when the host has received the amount
                         * of data that it was looking for or when
                         * the host receives a less than maxPacketSize
                         * data packet. */

                        /* Reset the nak counter for the next 
                         * data transaction. Fall through to the
                         * handshake stage. */
                        
                        pirp->tempState = DRV_USB_HOST_IRP_STATE_HANDSHAKE;
                    }
                    else if((deviceResponseSize == pirp->tempSize) ||
                            (deviceResponseSize == pipe->endpointSize))
                    {
                        if(pirp->tempSize > pirp->completedBytesInThisFrame)
                        {
                            /* More transactions are required in this frame */
                            pipe->dataToggle ^= 0x1;
                            pirp->tempState = DRV_USB_HOST_IRP_STATE_DATA_STAGE;
                            pirp->status = USB_HOST_IRP_STATUS_IN_PROGRESS;
                            _DRV_USB_HOST_Control_Send_Token(pirp,pusbdrvObj,
                                    pipe,endpoint,deviceAddress,usbID,
                                    isLowSpeed);
                            tokenSent = true;
                        }
                        else
                        {
                            /* Whatever planned for this frane has been done */
                            /* Do not Move the IRP Queue head here */
                            /* Do not end the IRP */
                            endIRP = false;
                            tokenSent = false;
                        }
                        break;
                    }
                }
                else 
                {
                    /* Data is moving from host to device */
                    if(pirp->completedBytes >= pirp->size)
                    {
	                 /* The write transfer is complete. Fall through
	                  * to the handshake stage */   
                        pipe->nakCounter = 0;
                        pirp->tempState = DRV_USB_HOST_IRP_STATE_HANDSHAKE;
                    }
                    else if((deviceResponseSize == pirp->tempSize) ||
                            (deviceResponseSize == pipe->endpointSize))
                    {
                        if(pirp->tempSize > pirp->completedBytesInThisFrame)
                        {
                            /* More transactions are required in this frame */
                            pipe->dataToggle ^= 0x1;
                            pirp->tempState = DRV_USB_HOST_IRP_STATE_DATA_STAGE;
                            pirp->status = USB_HOST_IRP_STATUS_IN_PROGRESS;
                            _DRV_USB_HOST_Control_Send_Token(pirp,pusbdrvObj,
                                    pipe,endpoint,deviceAddress,usbID,
                                    isLowSpeed);
                            tokenSent = true;
                        }
                        else
                        {
                            /* Whatever planned for this frane has been done */
                            /* Do not Move the IRP Queue head here */
                            /* Do not end the IRP */
                            endIRP = false;
                            tokenSent = false;
                        }
                        break;
                    }
                }

            }

        case DRV_USB_HOST_IRP_STATE_HANDSHAKE:
            /* Send the Handshake Stage */
            {
                USB_BUFFER_DIRECTION direction;
                USB_BUFFER_PING_PONG pingPong;
                USB_PID pid;

                if((pipe->endpointAndDirection & 0x80) != 0)
                {
                    /* Data is moving from device to host */
                    direction = USB_BUFFER_TX;
                    pingPong = pusbdrvObj->ep0TxPingPong;
                    pid = USB_PID_OUT;
                }
                else 
                {
                    /* Data is moving from host to device */
                    direction = USB_BUFFER_RX;
                    pingPong = pusbdrvObj->ep0RxPingPong;
                    pid = USB_PID_IN;
                }
                
                pBDT = pusbdrvObj->pBDT + (direction << 1) + pingPong;

                /* Configure the BDT Entry for data packet */
                pBDT->shortWord[1]  = 0;
                pBDT->word[1]       = 0;
                pBDT->byte[0]       = 0xC8;

                /* Keep track of the transaction */
                pirp->tempState = DRV_USB_HOST_IRP_STATE_HANDSHAKE_SENT;

                /* This will cause the Transaction interrupt */
                _DRV_USB_SendTokenToAddress(usbID, deviceAddress,
                        pid, endpoint, isLowSpeed);

                tokenSent = true;
                break;
            }

        case DRV_USB_HOST_IRP_STATE_HANDSHAKE_SENT:
            /* Check the response */
            if((USB_TRANSACTION_ACK == deviceResponse)
                    || (USB_TRANSACTION_DATA1 == deviceResponse))
            {
                /* Transfer is complete */
                pirp->tempState = DRV_USB_HOST_IRP_STATE_COMPLETE;
                pirp->status = USB_HOST_IRP_STATUS_COMPLETED;
                if(((pipe->endpointAndDirection & 0x80) != 0)
                        && (pirp->size > pirp->completedBytes))
                {
                    /* While moving data from device to host, if
                     * we received less data from the device than
                     * expected, then indicate a short packet and 
                     * set the irp size to to actual size */ 

                    pirp->status = USB_HOST_IRP_STATUS_COMPLETED_SHORT;
                    pirp->size = pirp->completedBytes;
                }
                endIRP = true;
            }
            else if(USB_TRANSACTION_NAK == deviceResponse)
            {
                pirp->tempState = DRV_USB_HOST_IRP_STATE_HANDSHAKE;
                pipe->nakCounter ++;
                if(pipe->nakCounter > DRV_USB_HOST_NAK_LIMIT)
                {
                    pirp->tempState = DRV_USB_HOST_IRP_STATE_COMPLETE;
                    pirp->status = USB_HOST_IRP_STATUS_ERROR_NAK_TIMEOUT;
                    endIRP = true;
                }
                break;
            }
            else if(USB_TRANSACTION_STALL == deviceResponse)
            {
                pirp->tempState = DRV_USB_HOST_IRP_STATE_COMPLETE;
                pirp->status = USB_HOST_IRP_STATUS_ERROR_STALL;
                endIRP = true;
                break;
            }

        case DRV_USB_HOST_IRP_STATE_COMPLETE:
            /* Remove the irp from the from the SW EP object. */
            endIRP = true;
            break;

        default:
            break;
    }

    if(endIRP == true)
    {
        pusbdrvObj->drvUSBHostSWEp[pusbdrvObj->numSWEpEntry].tobeDone = false;
        /* Move the current IRP */
        ((pusbdrvObj->transferGroup[USB_TRANSFER_TYPE_CONTROL])
                    .currentPipe)->irpQueueHead =
                (pusbdrvObj->drvUSBHostSWEp[pusbdrvObj->numSWEpEntry].pirp)
                ->next;
        /* Move the current pipe */
        if((pusbdrvObj->transferGroup[USB_TRANSFER_TYPE_CONTROL].currentPipe)
                ->next != NULL)
	{
	    /* This is not the last PIPE */
	    pusbdrvObj->transferGroup[USB_TRANSFER_TYPE_CONTROL].currentPipe =
		pusbdrvObj->transferGroup[USB_TRANSFER_TYPE_CONTROL].
                    currentPipe->next;
	}
	else
	{
	    /* This is the last PIPE. Move to the head pipe */
	    pusbdrvObj->transferGroup[USB_TRANSFER_TYPE_CONTROL].currentPipe =
                    pusbdrvObj->transferGroup[USB_TRANSFER_TYPE_CONTROL].pipe;
	}
        /* IRP completed. Call the callback */
        if(pirp->callback != NULL)
        {
            pirp->callback((USB_HOST_IRP *)pirp);
        }
    }

    return(tokenSent);
}/* end of _DRV_USB_HOST_ControlXferProcess() */

// *****************************************************************************
/* Function:
     void DRV_USB_HOST_PipeClose(DRV_USB_HOST_PIPE_HANDLE pipeHandle)

  Summary:
    Dynamic impementation of DRV_USB_HOST_PipeClose client interface function.

  Description:
    This is the dynamic impementation of DRV_USB_HOST_PipeClose client interface
    function. Function performs the following task:
    - Releases the pipe to be closed from the appropriate pipe type list
    - Aborts all the IRPs on that pipe

  Remarks:
    See drv_usb.h for usage information.
*/

void DRV_USB_HOST_PipeClose
(
    DRV_USB_HOST_PIPE_HANDLE pipeHandle
)
{
    /* Start of local variables */
    bool interruptWasEnabled                    = false;
    DRV_USB_OBJ * pusbdrvObj                    = (DRV_USB_OBJ *)NULL;
    DRV_USB_CLIENT_OBJ * pusbdrvClient          = (DRV_USB_CLIENT_OBJ *)NULL;
    USB_HOST_IRP_LOCAL * pirp                   = (USB_HOST_IRP_LOCAL *)NULL;
    DRV_USB_HOST_PIPE_OBJ * ppipe               = (DRV_USB_HOST_PIPE_OBJ *)NULL;
    DRV_USB_HOST_PIPE_OBJ * piteratorPipe       = (DRV_USB_HOST_PIPE_OBJ *)NULL;
    DRV_USB_HOST_TRANSFER_GROUP * ptransferGroup =
            (DRV_USB_HOST_TRANSFER_GROUP *)NULL ;
    /* End of local variables */
    
    /* Make sure we have a valid pipe */
    if(pipeHandle == DRV_USB_HOST_PIPE_HANDLE_INVALID)
    {
        SYS_DEBUG(0, "Invalid pipe handle");
    }
    else
    {
        ppipe = (DRV_USB_HOST_PIPE_OBJ*) pipeHandle;
        /* Make sure tha we are working with a pipe
         * in use */
        if(ppipe->inUse != true)
        {
            SYS_DEBUG(0, "Pipe is not in use");
            return;
        }

        pusbdrvClient = (DRV_USB_CLIENT_OBJ *)ppipe->hClient;
        pusbdrvObj = (DRV_USB_OBJ *)pusbdrvClient->hDriver;

        /* Disable the interrupt */
        if(!pusbdrvObj->isInInterruptContext)
        {
            //OSAL: Get Mutex
            interruptWasEnabled =
                _DRV_USB_InterruptSourceDisable(pusbdrvObj->interruptSource);
        }

        ptransferGroup = &pusbdrvObj->transferGroup[ppipe->pipeType];
    
        if(ptransferGroup->pipe == ppipe)
        {
            /* first pipe in the transfer group
             * needs to be closed */
            ptransferGroup->pipe = ppipe->next;
        }
        else
        {
            /* Remove this pipe from the linked
             * list */
            piteratorPipe = ptransferGroup->pipe;
            while((piteratorPipe != NULL) &&
                    (piteratorPipe->next != ppipe))
            {
                piteratorPipe = piteratorPipe->next;
            }
            /* Make sure we have a valid pipe */
            if(piteratorPipe == NULL)
            {
                SYS_DEBUG(0, "Illegal pipe handle");
                return;
            }
            piteratorPipe->next = ppipe->next;
        }

        if(ptransferGroup->nPipes != 0)
        {
            /* Reduce the count only if its
             * not zero already */
            ptransferGroup->nPipes --;
        }

        /* Now we invoke the call back for each IRP
         * in this pipe and say that it is aborted.
         * If the IRP is in progress, then that IRP
         * will be actually aborted on the next SOF
         * This will allow the USB module to complete
         * any transaction that was in progress. */

        pirp = (USB_HOST_IRP_LOCAL *)ppipe->irpQueueHead;
        while(pirp != NULL)
        {
            pirp->pipe = DRV_USB_HOST_PIPE_HANDLE_INVALID;

            if(pirp->status == USB_HOST_IRP_STATUS_IN_PROGRESS)
            {
                /* If the IRP is in progress, then we set the
                 * temp IRP state. This will be caught in
                 * the _DRV_USB_HOST_NonControlIRPProcess()
                 * and _DRV_USB_HOST_ControlXferProcess()
                 * functions */
                pirp->tempState = DRV_USB_HOST_IRP_STATE_ABORTED;
            }
            else
            {
                /* IRP is pending */
                pirp->status = USB_HOST_IRP_STATUS_ABORTED;
                if(pirp->callback != NULL)
                {
                    pirp->callback((USB_HOST_IRP*)pirp);
                }
            }
            pirp = pirp->next;
        }

        /* Now we return the pipe back to the
         * driver */

        ppipe->inUse = false;

        /* Enable the interrupts */
        if(!pusbdrvObj->isInInterruptContext)
        {
            if(interruptWasEnabled)
            {
                _DRV_USB_InterruptSourceEnable(pusbdrvObj->interruptSource);
            }
            //OSAL: Return Mutex
        }
    }

}/* end of DRV_USB_HOST_PipeClose()*/

// *****************************************************************************
/* Function:
    DRV_USB_HOST_PIPE_HANDLE DRV_USB_HOST_PipeSetup
    (
        DRV_HANDLE handle,
        uint8_t deviceAddress,
        USB_ENDPOINT endpointAndDirection,
        uint8_t hubAddress,
        uint8_t hubPort,
        USB_TRANSFER_TYPE pipeType,
        uint8_t bInterval,
        uint16_t wMaxPacketSize,
        USB_SPEED speed
    )

  Summary:
    Dynamic impementation of DRV_USB_HOST_PipeSetup client interface function.

  Description:
    This is the dynamic impementation of DRV_USB_HOST_PipeSetup client interface
    function. Function performs the following task:
    - Obtains a free pipe index from global pool
    - Updates the pipe data structure with required information
    - Adds the pipe to the specific transfer object queue based on
      transfer type
    - Returns the pipe address on success

  Remarks:
    See drv_usb.h for usage information.
*/

DRV_USB_HOST_PIPE_HANDLE DRV_USB_HOST_PipeSetup 
(
    DRV_HANDLE handle,
    uint8_t deviceAddress, 
    USB_ENDPOINT endpointAndDirection,
    uint8_t hubAddress,
    uint8_t hubPort,
    USB_TRANSFER_TYPE pipeType, 
    uint8_t bInterval, 
    uint16_t wMaxPacketSize,
    USB_SPEED speed
)
{
    /* Start of local variable */
    DRV_USB_HOST_PIPE_OBJ * ppipe          = (DRV_USB_HOST_PIPE_OBJ *)NULL;
    DRV_USB_HOST_PIPE_OBJ * piteratorPipe  = (DRV_USB_HOST_PIPE_OBJ *)NULL;
    DRV_USB_OBJ * pusbdrvObj               = (DRV_USB_OBJ *)NULL;
    DRV_USB_CLIENT_OBJ * pusbdrvClient     = (DRV_USB_CLIENT_OBJ *)handle;
    DRV_USB_HOST_PIPE_HANDLE returnValue   = DRV_USB_HOST_PIPE_HANDLE_INVALID;
    DRV_USB_HOST_TRANSFER_GROUP * ptransferGroup =
            (DRV_USB_HOST_TRANSFER_GROUP *)NULL;
    unsigned int transferTypeLocal         = 0;
    uint8_t pipeCount                      = 0;
    uint8_t bitSetCount                    = 0;
    /* End of local variable */

    /* Check if the handle is valid */
    if((handle == DRV_HANDLE_INVALID) ||
            (pusbdrvClient->inUse == false))
    {
        SYS_DEBUG(0, "Bad Client or client closed");
    }
    // For RTOS lock the mutex
    else
    {
        pusbdrvObj = (DRV_USB_OBJ *)pusbdrvClient->hDriver;
        /* Search for a free pipe object */
        for(pipeCount = 0; pipeCount < DRV_USB_HOST_PIPES_NUMBER; pipeCount++)
        {
            /* Check for free pipe object */

            if(gDrvUSBHostPipeObj[pipeCount].inUse == false)
            {
                /* We found a pipe object that we can use.
                 * Go and grab that one.
                 */
                gDrvUSBHostPipeObj[pipeCount].inUse = true;
                

                ppipe = &gDrvUSBHostPipeObj[pipeCount];

                ppipe->deviceAddress = deviceAddress;
                ppipe->irpQueueHead  = NULL;
                ppipe->bInterval     = bInterval;
                ppipe->speed         = speed;
                ppipe->pipeType      = pipeType;
                ppipe->hClient       = handle;
                ppipe->endpointSize  = wMaxPacketSize;
                ppipe->intervalCounter = bInterval;
                ppipe->dataToggle = USB_BUFFER_DATA0;

                ppipe->endpointAndDirection   = endpointAndDirection;

                switch(ppipe->pipeType)
                {
                    case USB_TRANSFER_TYPE_CONTROL:
                        transferTypeLocal = USB_TRANSFER_TYPE_LOCAL_CONTROL;
                        break;
                    case USB_TRANSFER_TYPE_ISOCHRONOUS:
                        transferTypeLocal = USB_TRANSFER_TYPE_LOCAL_ISOC;
                        break;
                    case USB_TRANSFER_TYPE_BULK:
                        transferTypeLocal = USB_TRANSFER_TYPE_LOCAL_BULK;
                        break;
                    case USB_TRANSFER_TYPE_INTERRUPT:
                        transferTypeLocal = USB_TRANSFER_TYPE_LOCAL_INTERRUPT;
                        break;
                    default:
                        return returnValue;
                }

                if((wMaxPacketSize <= 512) && (wMaxPacketSize != 0))
                {
                    for(bitSetCount = 0; bitSetCount < 10; bitSetCount++)
                    {
                        if((wMaxPacketSize & 0x00000001) == 0x00000001)
                        {
                            break;
                        }
                        wMaxPacketSize = wMaxPacketSize >> 1;
                    }
                }
                else if((wMaxPacketSize != 0))
                {
                    bitSetCount = 10;
                }
                if(bitSetCount != 10)
                {
                    if(ppipe->speed == USB_SPEED_LOW)
                    {
                        if((ppipe->pipeType == USB_TRANSFER_TYPE_INTERRUPT) ||
                                (ppipe->pipeType == USB_TRANSFER_TYPE_CONTROL))
                        {
                            ppipe->bwPerTransaction =
                                    gDrvUSBLSTableBW[transferTypeLocal]
                                    [bitSetCount];
                        }
                        else
                        {
                            /* Only control and interrupt pipe can exist in
                             low speed */
                            gDrvUSBHostPipeObj[pipeCount].inUse = false;
                            return DRV_USB_HOST_PIPE_HANDLE_INVALID;
                        }
                    }
                    else
                    {
                        ppipe->bwPerTransaction =
                            gDrvUSBFSTableBW[transferTypeLocal][bitSetCount];
                    }
                }
                else if(bitSetCount == 10 &&
                        ppipe->pipeType == USB_TRANSFER_TYPE_ISOCHRONOUS &&
                        ppipe->speed == USB_SPEED_FULL)
                {
                    ppipe->bwPerTransaction =
                            gDrvUSBFSTableBW[transferTypeLocal][bitSetCount];
                }
                else
                {
                    /* Error */
                    return returnValue;
                }

                /* This pipe should be added to the
                 * respective transfer group */

                ptransferGroup = &(pusbdrvObj->transferGroup[pipeType]);

                if(ptransferGroup->pipe == NULL)
                {
                    /* This if the first pipe to be setup */
                    ptransferGroup->pipe = ppipe;
                    ptransferGroup->currentPipe = ppipe;
                }
                else
                {
                    /* This is NOT the first pipe. Find
                     * the last pipe in the linked list */
                    piteratorPipe = ptransferGroup->pipe;
                    while(piteratorPipe->next != NULL)
                    {
                        /* This is not the last pipe in
                         * this transfer group */
                        piteratorPipe = piteratorPipe->next;
                    }
                    piteratorPipe->next = ppipe;
                }
                ppipe->next = NULL;
                /* Update the pipe count in the transfer group */
                ptransferGroup->nPipes ++;
                returnValue = (DRV_USB_HOST_PIPE_HANDLE)ppipe;
                /*OSAL - Mutex Unlock */
                break;
            }/* end of pipe object found */

        } /* end of for loop() */
    }

    if(pipeCount == DRV_USB_HOST_PIPES_NUMBER)
    {
        SYS_DEBUG(0, "Could not find a free pipe object");
        /*OSAL - Mutex Unlock */
    }
    /* Return the handle */
    return returnValue;

}/* end of DRV_USB_HOST_PipeSetup() */

// *****************************************************************************
/* Function:
    void _DRV_USB_HOST_Control_Send_Token
    (
        USB_HOST_IRP_LOCAL * pirp,
        DRV_USB_OBJ *pusbdrvObj,
        DRV_USB_HOST_PIPE_OBJ *pipe,
        bool isLowSpeed
    )

  Summary:
    Dynamic impementation of _DRV_USB_HOST_Control_Send_Token internal
    function.

  Description:
    This is the dynamic impementation of _DRV_USB_HOST_Control_Send_Token
    function. Function sends token for CONTROL transfer data phase.

  Remarks:
    This is a local function and should not be called directly by the
    application.

*/

void _DRV_USB_HOST_Control_Send_Token
(
    USB_HOST_IRP_LOCAL * pirp,
    DRV_USB_OBJ *pusbdrvObj,
    DRV_USB_HOST_PIPE_OBJ *pipe,
    uint8_t endpoint,
    uint8_t deviceAddress,
    USB_MODULE_ID usbID,
    bool isLowSpeed
)
{
    /* Start of local variable */
    USB_BUFFER_DIRECTION direction;
    USB_BUFFER_PING_PONG pingPong;
    USB_PID pid;
    unsigned int size;
    DRV_USB_BDT_ENTRY * pBDT        = (DRV_USB_BDT_ENTRY *)NULL;
    /* End of local variable */

    if((pipe->endpointAndDirection & 0x80)  != 0)
    {
	/* Direction is device to host */

	direction = USB_BUFFER_RX;
	pingPong = pusbdrvObj->ep0RxPingPong;
	pid = USB_PID_IN;
    }
    else
    {
	/* Direction is host to device */
	direction = USB_BUFFER_TX;
	pingPong = pusbdrvObj->ep0TxPingPong;
	pid = USB_PID_OUT;
    }
    if((pirp->tempSize - pirp->completedBytesInThisFrame) >= pipe->endpointSize)
    {
	/* This means we have to break
	 * up the transfer into transactions */
	size = pipe->endpointSize;
    }
    else
    {
	/* Data size is less than endpoint size */
	size = pirp->tempSize;
    }

    /* Keep track of the transaction */
    pirp->tempState = DRV_USB_HOST_IRP_STATE_DATA_STAGE_SENT;

    pBDT = pusbdrvObj->pBDT + (direction << 1) + pingPong;

    /* Configure the BDT Entry for data packet */
    pBDT->shortWord[1]  = size;
    pBDT->word[1]       =
	    KVA_TO_PA(pirp->data + pirp->completedBytes);
    pBDT->byte[0]       = 0x88 | (pipe->dataToggle << 6);

    /* This will cause the Transaction interrupt */
    _DRV_USB_SendTokenToAddress(usbID, deviceAddress,
		pid, endpoint, isLowSpeed);

}/* end of _DRV_USB_HOST_Control_Send_Token() */



// *****************************************************************************
/* Function:
    void _DRV_USB_HOST_NonControl_Send_Token
    (
        USB_HOST_IRP_LOCAL * pirp,
        DRV_USB_OBJ *pusbdrvObj,
        DRV_USB_HOST_PIPE_OBJ *pipe,
        bool isLowSpeed
    )
  
  Summary:
    Dynamic impementation of _DRV_USB_HOST_NonControl_Send_Token internal
    function.

  Description:
    This is the dynamic impementation of _DRV_USB_HOST_NonControl_Send_Token
    function. Function sends token for NON CONTROL transfer.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _DRV_USB_HOST_NonControl_Send_Token
(
    USB_HOST_IRP_LOCAL * pirp,
    DRV_USB_OBJ *pusbdrvObj,
    DRV_USB_HOST_PIPE_OBJ *pipe,
    bool isLowSpeed
)
{
    /* Start of local variable */
    int direction = 0;
    int size;
    USB_PID pid;
    USB_MODULE_ID usbID;
    DRV_USB_BDT_ENTRY * pBDT   = (DRV_USB_BDT_ENTRY *)NULL;
    USB_BUFFER_PING_PONG pingpong;
    uint8_t endpoint;
    uint8_t deviceAddress;
    /* End of local variable */


    pBDT        = pusbdrvObj->pBDT;
    usbID       = pusbdrvObj->usbID;
    
    endpoint    = pipe->endpointAndDirection & 0xF;
    deviceAddress   = pipe->deviceAddress;


    pirp->status = USB_HOST_IRP_STATUS_IN_PROGRESS;
    if((pipe->endpointAndDirection & 0x80) != 0)
    {
            /* Data is moving from device to host */
            direction = USB_BUFFER_RX;
            pingpong = pusbdrvObj->ep0RxPingPong;
            pid = USB_PID_IN;
    }
    else
    {
	/* Data is moving from host to device */
	direction = USB_BUFFER_TX;
	pingpong = pusbdrvObj->ep0TxPingPong;
	pid = USB_PID_OUT;
    }

    if((pirp->tempSize - pirp->completedBytesInThisFrame) >= pipe->endpointSize)
    {
	size = pipe->endpointSize;
    }
    else
    {
	size = pirp->tempSize;
    }

    pBDT = pusbdrvObj->pBDT + (direction << 1) + pingpong;

    /*Configure the BDT Entry for data packet */
    pBDT->shortWord[1] = size;
    pBDT->word[1] = KVA_TO_PA(pirp->data + pirp->completedBytes);
    pBDT->byte[0] = 0x88 | (pipe->dataToggle << 6);

    /* This will cause a transaction interrupt */
    _DRV_USB_SendTokenToAddress(usbID, deviceAddress,
	    pid, endpoint, isLowSpeed);
}/* end of _DRV_USB_HOST_NonControl_Send_Token() */

// *****************************************************************************
/* Function:
    bool _DRV_USB_HOST_NonControlIRPProcess
    (
        DRV_USB_OBJ * pusbdrvObj,
        USB_HOST_IRP_LOCAL * pirp, 
        DRV_USB_TRANSACTION_RESULT lastTransactionResult,
        int lastTransactionsize
    )

  Summary:
    Dynamic impementation of _DRV_USB_HOST_NonControlIRPProcess internal
    function.

  Description:
    This is the dynamic impementation of _DRV_USB_HOST_NonControlIRPProcess
    function. Function sends token for NON CONTROL transfer.

  Remarks:
    This is a local function and should not be called directly by the
    application.
    
*/

bool _DRV_USB_HOST_NonControlIRPProcess
(
    DRV_USB_OBJ * pusbdrvObj,
    USB_HOST_IRP_LOCAL * pirp, 
    DRV_USB_TRANSACTION_RESULT lastTransactionResult,
    int lastTransactionsize
)
{
    /* Start of local variable */
    bool endIRP;
    bool isLowSpeed;
    DRV_USB_HOST_PIPE_OBJ   * pipe   = (DRV_USB_HOST_PIPE_OBJ *)NULL;
    bool tokenSent                   = false;
    /* End of local variable */    

    if(pirp->tempState == DRV_USB_HOST_IRP_STATE_ABORTED)
    {
        /* This means that this IRP was aborted
         * by the application while it was in 
         * progress. Terminate it now. */
	pusbdrvObj->drvUSBHostSWEp[pusbdrvObj->numSWEpEntry].tobeDone = false;
        ((pusbdrvObj->transferGroup
                [pusbdrvObj->drvUSBHostSWEp[pusbdrvObj->numSWEpEntry].
                transferType])
                .currentPipe)->irpQueueHead =
            (pusbdrvObj->drvUSBHostSWEp[pusbdrvObj->numSWEpEntry].pirp)->next;

        pirp->status = USB_HOST_IRP_STATUS_ABORTED;
        if(pirp->callback != NULL)
        {
            pirp->callback((USB_HOST_IRP*)pirp);
        }
        return false;
    }

    pipe        = (DRV_USB_HOST_PIPE_OBJ *)pirp->pipe;
    isLowSpeed 	= (pipe->speed == USB_SPEED_LOW) ? true : false;

    if(lastTransactionResult == 0)
    {
        /* This means that a token need to be sent */
        _DRV_USB_HOST_NonControl_Send_Token(pirp,pusbdrvObj,pipe,isLowSpeed);
        if(pipe->pipeType == USB_TRANSFER_TYPE_ISOCHRONOUS)
        {
            /* Remove the irp from the from the SW EP object. */
            pusbdrvObj->drvUSBHostSWEp[pusbdrvObj->numSWEpEntry].tobeDone
                    = false;

            ((DRV_USB_HOST_PIPE_OBJ *)(pusbdrvObj->drvUSBHostSWEp
                    [pusbdrvObj->numSWEpEntry].pirp->pipe))->irpQueueHead =
                    (pusbdrvObj->drvUSBHostSWEp[pusbdrvObj->numSWEpEntry].pirp)
                    ->next;

            if(pirp->callback != NULL)
            {
                pirp->callback((USB_HOST_IRP *)pirp);
            }
            tokenSent = false;
        }
        else
        {
            tokenSent = true;
        }
    }
    else
    {
        /* This means this function was called from the TRNIF
         * interrupt after the token was sent. Check the 
         * transaction result */
        endIRP = false;
        tokenSent = false;
        switch(lastTransactionResult)
        {
            case USB_TRANSACTION_ACK:
            case USB_TRANSACTION_DATA0:
            case USB_TRANSACTION_DATA1:    
                pirp->completedBytes += lastTransactionsize;
                pirp->completedBytesInThisFrame += lastTransactionsize;
                pipe->dataToggle ^= 0x1;
                pipe->nakCounter = 0;
                if((lastTransactionsize < pipe->endpointSize) ||
                        (pirp->size <= pirp->completedBytes))
                {
                    /* We received data less than endpoint size.
                     * So we end the transfer */
                    if(pirp->size < pirp->completedBytes)
                    {
                        pirp->status = USB_HOST_IRP_STATUS_COMPLETED_SHORT;
                    }
                    else
                    {
                        pirp->status = USB_HOST_IRP_STATUS_COMPLETED;
                    }
                    endIRP = true;
                }
                else if((lastTransactionsize == pirp->tempSize) ||
                        (lastTransactionsize == pipe->endpointSize))
                {
                    if(pirp->tempSize > pirp->completedBytesInThisFrame)
                    {
                        /* Some more transactions are required in this frame */
                        pirp->status = USB_HOST_IRP_STATUS_IN_PROGRESS;
                        _DRV_USB_HOST_NonControl_Send_Token(pirp,pusbdrvObj,
                                pipe,isLowSpeed);
                        tokenSent = true;
                    }
                    else
                    {
                        /* Whatever planned for this frane has been done */
                        /* Do not Move the IRP Queue head here */
                        
                        /* Do not end the IRP */
                        endIRP = false;
                        tokenSent = false;
                    }
                }
                break;
            case USB_TRANSACTION_STALL:
                /* The token was stalled. We end the IRP */
                pipe->nakCounter = 0;
                pirp->status = USB_HOST_IRP_STATUS_ERROR_STALL;
                endIRP = true;
                break;
            case USB_TRANSACTION_NAK:
                /* For non - control transfer we dont implement a 
                 * NAK timeout.Do not do anything here */
                endIRP = false;
                break;
            default:
                break;
        }

        if(endIRP == true)
        {
            /* Remove the irp from the from the SW EP object. */
            pusbdrvObj->drvUSBHostSWEp[pusbdrvObj->numSWEpEntry].tobeDone
                    = false;

            ((DRV_USB_HOST_PIPE_OBJ *)(pusbdrvObj->drvUSBHostSWEp
                    [pusbdrvObj->numSWEpEntry].pirp->pipe))->irpQueueHead =
                    (pusbdrvObj->drvUSBHostSWEp[pusbdrvObj->numSWEpEntry].pirp)
                    ->next;
            /* Update the size field with actual size recieved\transmitted */
            pirp->size = pirp->completedBytes;

            if(pirp->callback != NULL)
            {
                pirp->callback((USB_HOST_IRP *)pirp);
            }
            
        }

    }
    return tokenSent;
}/* end of _DRV_USB_HOST_NonControlIRPProcess() */

// *****************************************************************************
/* Function:
    void _DRV_USB_HOST_Calculate_Control_BW
    (
        DRV_USB_OBJ * pusbdrvObj,
        DRV_USB_HOST_TRANSFER_GROUP * ptransferGroup,
        USB_HOST_IRP_LOCAL * pcontrolIRP
    )

  Summary:
    Dynamic impementation of _DRV_USB_HOST_Calculate_Control_BW internal
    function.

  Description:
    This is the dynamic impementation of _DRV_USB_HOST_Calculate_Control_BW
    function. Function performs the following task:
    - Obtains the bandwidth requirement for the transfer based on pipe
      and IRP size
    - Calculates the number of transactions that can be done and updates data
      structure accordingly
    - Packs the IRP for processing it in this frame.

  Remarks:
    This is a local function and should not be called directly by the
    application.

*/

void _DRV_USB_HOST_Calculate_Control_BW
(
    DRV_USB_OBJ * pusbdrvObj,
    DRV_USB_HOST_TRANSFER_GROUP * ptransferGroup,
    USB_HOST_IRP_LOCAL * pcontrolIRP
)
{
    /* Start of local variable */
    unsigned int bwAvailable             = 0;
    unsigned int bwPerTransaction        = 0;
    unsigned int noofTransaction         = 0;
    unsigned int noofPossibleTransaction = 0;
    DRV_USB_HOST_PIPE_OBJ * ppipe        =
        (DRV_USB_HOST_PIPE_OBJ *)pcontrolIRP->pipe;
    /* End of local variable */

    /* Checks the BW available out of the max possible for CONTROL transfer */
    bwAvailable =
        (DRV_USB_MAX_CONTROL_BANDWIDTH - pusbdrvObj->globalBWConsumed);
    bwPerTransaction = ppipe->bwPerTransaction;

    /* Check if atleast 1 transaction is possible */
    if(bwPerTransaction <= bwAvailable)
    {
        /* Atleast 1 transaction is possible */
        noofTransaction = (pcontrolIRP->size - pcontrolIRP->completedBytes)/
            ppipe->endpointSize;
        if(noofTransaction == 0)
        {
            noofTransaction = 1;
        }
        else if((pcontrolIRP->size - pcontrolIRP->completedBytes) %
                (ppipe->endpointSize)
                != 0)
        {
            noofTransaction++;
        }
        noofPossibleTransaction =
            bwAvailable/bwPerTransaction;
        if(noofPossibleTransaction < noofTransaction)
        {
            noofTransaction = noofPossibleTransaction;
        }
        /* Increment the global bandwidth */
        pusbdrvObj->globalBWConsumed =
            pusbdrvObj->globalBWConsumed +
            (noofTransaction * bwPerTransaction);
        pcontrolIRP->tempSize =
            noofTransaction * ppipe->endpointSize;

        /* Based on the above calculation, the size of the data to be
         * transmitted\recieved will be always multiple of endpoint size.
         * But if this size is more than the actual size, then we should
         * consider the actual data size remaining.
         */
        if((pcontrolIRP->size - pcontrolIRP->completedBytes) <
                pcontrolIRP->tempSize)
        {
            pcontrolIRP->tempSize =
                pcontrolIRP->size - pcontrolIRP->completedBytes;
        }

        /* Reset the IRP completed bytes in this frame field */
        pcontrolIRP->completedBytesInThisFrame = 0;

        /* Now that we have an IRP to be processed,
         * update the global data structure
         */
        pusbdrvObj->drvUSBHostSWEp[0].tobeDone = true;
        pusbdrvObj->drvUSBHostSWEp[0].transferType = USB_TRANSFER_TYPE_CONTROL;
        pusbdrvObj->drvUSBHostSWEp[0].pirp = pcontrolIRP;
    }

}/* end of _DRV_USB_HOST_Calculate_Control_BW() */

// *****************************************************************************
/* Function:
    bool _DRV_USB_HOST_Calculate_NonControl_BW
    (
        DRV_USB_OBJ * pusbdrvObj,
        DRV_USB_HOST_TRANSFER_GROUP * ptransferGroup,
        USB_HOST_IRP_LOCAL * ptransferIRP,
        USB_TRANSFER_TYPE transferType,
        uint8_t numSWEpEntry
    )

  Summary:
    Dynamic impementation of _DRV_USB_HOST_Calculate_NonControl_BW internal
    function.

  Description:
    This is the dynamic impementation of _DRV_USB_HOST_Calculate_NonControl_BW
    function. Function performs the following task:
    - Obtains the bandwidth requirement for the transfer based on pipe
      and IRP size
    - Calculates the number of transactions that can be done and updates data
      structure accordingly
    - Packs the IRP for processing it in this frame.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

bool _DRV_USB_HOST_Calculate_NonControl_BW
(
    DRV_USB_OBJ * pusbdrvObj,
    DRV_USB_HOST_TRANSFER_GROUP * ptransferGroup,
    USB_HOST_IRP_LOCAL * ptransferIRP,
    USB_TRANSFER_TYPE transferType,
    uint8_t numSWEpEntry
)
{
    /* Start of local variable */
    unsigned int bwAvailable             = 0;
    unsigned int bwPerTransaction        = 0;
    unsigned int noofTransaction         = 0;
    unsigned int noofPossibleTransaction = 0;
    DRV_USB_HOST_PIPE_OBJ * ppipe        =
        (DRV_USB_HOST_PIPE_OBJ *)ptransferIRP->pipe;
    bool irpPacked                       = false;

    /* End of local variable */

    /* Calculate the BW available in this frame */
    bwAvailable =
        (DRV_USB_MAX_BANDWIDTH_PER_FRAME - pusbdrvObj->globalBWConsumed);
    bwPerTransaction = ppipe->bwPerTransaction;

    /* Check if atleast 1 transaction is possible */
    if(bwPerTransaction <= bwAvailable)
    {
        /* Atleast 1 transaction is possible */
        noofTransaction = (ptransferIRP->size - ptransferIRP->completedBytes)/
            ppipe->endpointSize;
        if(noofTransaction == 0)
        {
            noofTransaction = 1;
        }
        else if((ptransferIRP->size - ptransferIRP->completedBytes) %
                (ppipe->endpointSize)
                != 0)
        {
            noofTransaction++;
        }
        noofPossibleTransaction =
            bwAvailable/bwPerTransaction;
        if(noofPossibleTransaction < noofTransaction)
        {
            noofTransaction = noofPossibleTransaction;
        }
        /* Increment the global bandwidth */
        pusbdrvObj->globalBWConsumed =
            pusbdrvObj->globalBWConsumed +
            (noofTransaction * bwPerTransaction);
        ptransferIRP->tempSize =
            noofTransaction * ppipe->endpointSize;

        /* Based on the above calculation, the size of the data to be
         * transmitted\recieved will be always multiple of endpoint size.
         * But if this size is more than the actual size, then we should
         * consider the actual data size remaining.
         */
        if((ptransferIRP->size - ptransferIRP->completedBytes) <
                ptransferIRP->tempSize)
        {
            ptransferIRP->tempSize =
                ptransferIRP->size - ptransferIRP->completedBytes;
        }

        /* Reset the IRP completed bytes in this frame field */
        ptransferIRP->completedBytesInThisFrame = 0;

        /* Now that we have an IRP to be processed,
         * update the global data structure
         */
        pusbdrvObj->drvUSBHostSWEp[numSWEpEntry].tobeDone = true;
        pusbdrvObj->drvUSBHostSWEp[numSWEpEntry].transferType = transferType;
        pusbdrvObj->drvUSBHostSWEp[numSWEpEntry].pirp = ptransferIRP;
        irpPacked = true;
    }
    return irpPacked;

}/* end of _DRV_USB_HOST_Calculate_Control_BW() */

// *****************************************************************************
/* Function:
    bool _DRV_USB_HOST_TransferSchedule
    (
        DRV_USB_OBJ * pusbdrvObj,
        DRV_USB_TRANSACTION_RESULT lastResult,
        unsigned int transactionSize,
        bool frameExpiry
    )

  Summary:
    Dynamic impementation of _DRV_USB_HOST_TransferSchedule internal function.

  Description:
    This is the dynamic impementation of _DRV_USB_HOST_TransferSchedule
    function. Function performs the following task:
    - Prepares the SW EP buffer with the transfer details that will be
      processed in the frame
    - Processes the SW EP buffer object that can be done in frame

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

bool _DRV_USB_HOST_TransferSchedule
(
    DRV_USB_OBJ * pusbdrvObj,
    DRV_USB_TRANSACTION_RESULT lastResult,
    unsigned int transactionSize,
    bool frameExpiry
)
{
    /* Start of local variable */
    DRV_USB_HOST_TRANSFER_GROUP * ptransferGroup =
        (DRV_USB_HOST_TRANSFER_GROUP *)NULL;
    DRV_USB_HOST_PIPE_OBJ * piteratorPipe        =
        (DRV_USB_HOST_PIPE_OBJ *)NULL;
    USB_HOST_IRP_LOCAL * pcontrolIRP             = (USB_HOST_IRP_LOCAL *)NULL;
    USB_HOST_IRP_LOCAL * pbulkIRP                = (USB_HOST_IRP_LOCAL *)NULL;
    USB_HOST_IRP_LOCAL * pinterruptIRP           = (USB_HOST_IRP_LOCAL *)NULL;
    USB_HOST_IRP_LOCAL * pisochronousIRP         = (USB_HOST_IRP_LOCAL *)NULL;
    uint8_t numSWEpEntry                         = 0;
    uint8_t numIRPProcess                        = 0;
    bool tokenSent                               = false;
    bool irpPacked                               = false;
    /* End of local variable */

    if(pusbdrvObj == NULL)
    {
        /* NULL pointer error */
        SYS_DEBUG(0, "Illegal USB module instance");
        return false;
    }

    if(lastResult == 0)
    {
        if(frameExpiry)
        {
            /*
               Clear all the NON CONTROL IRPs which may be still there from the
               previous frame, if not completed */
            for(numSWEpEntry = 1; numSWEpEntry < _DRV_USB_SW_EP_NUMBER;
                    numSWEpEntry++)
            {
                pusbdrvObj->drvUSBHostSWEp[numSWEpEntry].tobeDone = false;
            }

            pusbdrvObj->globalBWConsumed = 0;
            numSWEpEntry = 0;

            /* Point to CONTROL pipes */
            ptransferGroup =
                &pusbdrvObj->transferGroup[USB_TRANSFER_TYPE_CONTROL];
            /*
             * current pipe initially will be head pipe.
             * after that it will rotate through the 
             * CONTROL pipe linked list
             */

            if(ptransferGroup->pipe != NULL)
            {
                if(pusbdrvObj->drvUSBHostSWEp[0].tobeDone == true)
                {
                    /* Transfer continuation from last frame */
                    ptransferGroup->currentIRP =
                        (ptransferGroup->currentPipe)->irpQueueHead;
                    pcontrolIRP = ptransferGroup->currentIRP;

                    _DRV_USB_HOST_Calculate_Control_BW(pusbdrvObj,
                            ptransferGroup,pcontrolIRP);
                }
                else
                {
                    /* Fresh Transfer starting this frame */
                }
                while(pusbdrvObj->drvUSBHostSWEp[0].tobeDone == false)
                {
                    /* Fresh Transfer starting this frame */
                    ptransferGroup->currentIRP =
                        (ptransferGroup->currentPipe)->irpQueueHead;
                    pcontrolIRP = ptransferGroup->currentIRP;

                    if((pcontrolIRP != NULL) &&
                            (numIRPProcess < DRV_USB_HOST_IRP_PER_FRAME_NUMBER))
                    {
                        _DRV_USB_HOST_Calculate_Control_BW(pusbdrvObj,
                                ptransferGroup,pcontrolIRP);
                    }/* end of if(pcontrolIRP!= NULL) */
                    if(pusbdrvObj->drvUSBHostSWEp[0].tobeDone == false)
                    {
                        /*
                           There was no CONTROL IRP in the last pipe.
                           */
                        if((ptransferGroup->currentPipe)->next != NULL)
                        {
                            /* This is not the last PIPE */
                            ptransferGroup->currentPipe =
                                (ptransferGroup->currentPipe)->next;
                        }
                        else
                        {
                            /* This is the last PIPE. Move to the head pipe */
                            ptransferGroup->currentPipe = ptransferGroup->pipe;
                            break;
                        }
                    }
                }/* end of while() */
                if(pusbdrvObj->drvUSBHostSWEp[0].tobeDone == true)
                {
                    numIRPProcess++;
                }
            }/* end of if(CONTROL PIPE present)*/

            /* Set it to 1 as isochronous may not be there in the frame */
            numSWEpEntry = 1;

            ptransferGroup =
                &pusbdrvObj->transferGroup[USB_TRANSFER_TYPE_ISOCHRONOUS];
            piteratorPipe = ptransferGroup->pipe;

            if(piteratorPipe != NULL)
            {
                /* Decrement the interval counter for all isochronous pipes */
                do
                {
                    if(piteratorPipe->intervalCounter != 0)
                    {
                        piteratorPipe->intervalCounter--;
                    }
                    else
                    {
                        /* Reinitialize the counter on expiry */
                        piteratorPipe->intervalCounter =
                            piteratorPipe->bInterval;
                    }
                    piteratorPipe = piteratorPipe->next;

                } while(piteratorPipe != NULL);


                ptransferGroup->currentIRP =
                    ptransferGroup->currentPipe->irpQueueHead;
                pisochronousIRP = ptransferGroup->currentIRP;
                piteratorPipe = ptransferGroup->currentPipe;

                while((numSWEpEntry < _DRV_USB_SW_EP_NUMBER) &&
                        (numIRPProcess < DRV_USB_HOST_IRP_PER_FRAME_NUMBER))
                {
                    irpPacked = false;
                    /* Check for IRP with an expired interval */
                    if((pisochronousIRP != NULL) &&
                            (piteratorPipe->intervalCounter == 0))
                    {
                        /* Check BW and pack IRP */
                        irpPacked =
                            _DRV_USB_HOST_Calculate_NonControl_BW(pusbdrvObj,
                                    ptransferGroup,pisochronousIRP,
                                    USB_TRANSFER_TYPE_ISOCHRONOUS,numSWEpEntry);
                        if(irpPacked == true)
                        {
                            ptransferGroup->currentPipe = piteratorPipe;
                            /* Increment the IRP counter */
                            numIRPProcess++;
                            /* Increment SWEP index */
                            numSWEpEntry++;
                            /*
                             * IRP has been packed for this frame.
                             * Now move Current Pipe to next pipe, so that
                             * in the next frame the processing will start
                             * from this pipe.
                             */
                            if(ptransferGroup->currentPipe->next != NULL)
                            {
                                ptransferGroup->currentPipe =
                                    ptransferGroup->currentPipe->next;
                            }
                            else
                            {
                                /*
                                 * This is the last pipe in the list
                                 * Rotate back to the head pipe.
                                 */
                                ptransferGroup->currentPipe =
                                    ptransferGroup->pipe;
                            }
                        }/* end of(if IRP PACKED)*/
                    }
                    /*
                     * irpPacked will be false in below scenarios:
                     *     - there is no IRP to be processed in the pipe
                     *     - irp from the pipe cannot be processed due to
                     *       BW availability issue
                     */
                    if(irpPacked == false)
                    {
                        /* Move to the next pipe.
                         * Note: Current pipe has NOT been moved.
                         * It will be moved only if IRP has been packed
                         * from the pipe.
                         */
                        piteratorPipe = piteratorPipe->next;
                        if(piteratorPipe == NULL)
                        {
                            /* End of pipe. Move to head pipe */
                            piteratorPipe = ptransferGroup->pipe;
                            ptransferGroup->currentPipe = piteratorPipe;
                        }
                    }
                    else
                    {
                        /* No need to move ahead here. Already done before */
                        piteratorPipe = ptransferGroup->currentPipe;
                    }
                    if(piteratorPipe == ptransferGroup->pipe)
                    {
                        /* Reached the beginning of the ISOC pipe */
                        break;
                    }
                    pisochronousIRP = piteratorPipe->irpQueueHead;
                }/* end of while() */
            }/* end of if(ISOCHRONOUS PIPE present) */

            ptransferGroup =
                &pusbdrvObj->transferGroup[USB_TRANSFER_TYPE_INTERRUPT];
            piteratorPipe = ptransferGroup->pipe;

            if(piteratorPipe != NULL)
            {
                /* Decrement the interval counter for all isochronous pipes */
                do
                {
                    if(piteratorPipe->intervalCounter != 0)
                    {
                        piteratorPipe->intervalCounter--;
                    }
                    else
                    {
                        /* Reinitialize the counter on expiry */
                        piteratorPipe->intervalCounter =
                            piteratorPipe->bInterval;
                    }
                    piteratorPipe = piteratorPipe->next;

                } while(piteratorPipe != NULL);


                ptransferGroup->currentIRP =
                    ptransferGroup->currentPipe->irpQueueHead;
                pinterruptIRP = ptransferGroup->currentIRP;
                piteratorPipe = ptransferGroup->currentPipe;

                while((numSWEpEntry < _DRV_USB_SW_EP_NUMBER) &&
                        (numIRPProcess < DRV_USB_HOST_IRP_PER_FRAME_NUMBER))
                {
                    irpPacked = false;
                    /* Check for IRP with an expired interval */
                    if((pinterruptIRP != NULL) &&
                            (piteratorPipe->intervalCounter == 0))
                    {
                        /* Check BW and pack IRP */
                        irpPacked =
                            _DRV_USB_HOST_Calculate_NonControl_BW(pusbdrvObj,
                                    ptransferGroup,pinterruptIRP,
                                    USB_TRANSFER_TYPE_INTERRUPT,numSWEpEntry);
                        if(irpPacked == true)
                        {
                            ptransferGroup->currentPipe = piteratorPipe;
                            /* Increment the IRP counter */
                            numIRPProcess++;
                            /* Increment SWEP index */
                            numSWEpEntry++;
                            /*
                             * IRP has been packed for this frame.
                             * Now move Current Pipe to next pipe, so that
                             * in the next frame the processing will start
                             * from this pipe.
                             */
                            if(ptransferGroup->currentPipe->next != NULL)
                            {
                                ptransferGroup->currentPipe =
                                    ptransferGroup->currentPipe->next;
                            }
                            else
                            {
                                /*
                                 * This is the last pipe in the list
                                 * Rotate back to the head pipe.
                                 */
                                ptransferGroup->currentPipe =
                                    ptransferGroup->pipe;
                            }
                        }/* end of(if IRP PACKED)*/
                    }
                    /*
                     * irpPacked will be false in below scenarios:
                     *     - there is no IRP to be processed in the pipe
                     *     - irp from the pipe cannot be processed due to
                     *       BW availability issue
                     */
                    if(irpPacked == false)
                    {
                        /* Move to the next pipe.
                         * Note: Current pipe has NOT been moved.
                         * It will be moved only if IRP has been packed
                         * from the pipe.
                         */
                        piteratorPipe = piteratorPipe->next;
                        if(piteratorPipe == NULL)
                        {
                            /* End of pipe. Move to head pipe */
                            piteratorPipe = ptransferGroup->pipe;
                            ptransferGroup->currentPipe = piteratorPipe;
                        }
                    }
                    else
                    {
                        /* No need to move ahead here. Already done before */
                        piteratorPipe = ptransferGroup->currentPipe;
                    }
                    if(piteratorPipe == ptransferGroup->pipe)
                    {
                        /* Reached the beginning of the ISOC pipe */
                        break;
                    }
                    pinterruptIRP = piteratorPipe->irpQueueHead;
                }/* end of while() */
            }/* end of if(INTERRUPT PIPE present) */



            /* SW EP processing for BULK transfer */
            ptransferGroup = &pusbdrvObj->transferGroup[USB_TRANSFER_TYPE_BULK];
            if(ptransferGroup->pipe != NULL)
            {
                ptransferGroup->currentIRP =
                    ptransferGroup->currentPipe->irpQueueHead;
                pbulkIRP = ptransferGroup->currentIRP;
                piteratorPipe = ptransferGroup->currentPipe;

                while((numSWEpEntry < _DRV_USB_SW_EP_NUMBER) &&
                        (numIRPProcess < DRV_USB_HOST_IRP_PER_FRAME_NUMBER))
                {
                    irpPacked = false;
                    if(pbulkIRP != NULL)
                    {
                        /* Check BW and pack IRP */
                        irpPacked =
                            _DRV_USB_HOST_Calculate_NonControl_BW(pusbdrvObj,
                                    ptransferGroup,pbulkIRP,USB_TRANSFER_TYPE_BULK,
                                    numSWEpEntry);
                        if(irpPacked == true)
                        {
                            ptransferGroup->currentPipe = piteratorPipe;
                            /* Increment the IRP counter */
                            numIRPProcess++;
                            /* Increment SWEP index */
                            numSWEpEntry++;
                            /*
                             * IRP has been packed for this frame.
                             * Now move Current Pipe to next pipe, so that
                             * in the next frame the processing will start
                             * from this pipe.
                             */
                            if(ptransferGroup->currentPipe->next != NULL)
                            {
                                ptransferGroup->currentPipe =
                                    ptransferGroup->currentPipe->next;
                            }
                            else
                            {
                                /*
                                 * This is the last pipe in the list
                                 * Rotate back to the head pipe.
                                 */
                                ptransferGroup->currentPipe =
                                    ptransferGroup->pipe;
                            }
                        }
                    }
                    /*
                     * irpPacked will be false in below scenarios:
                     *     - there is no IRP to be processed in the pipe
                     *     - irp from the pipe cannot be processed due to
                     *       BW availability issue 
                     */
                    if(irpPacked == false)
                    {
                        /* Move to the next pipe. 
                         * Note: Current pipe has NOT been moved.
                         * It will be moved only if IRP has been packed
                         * from the pipe.
                         */
                        piteratorPipe = piteratorPipe->next;
                        if(piteratorPipe == NULL)
                        {
                            /* End of pipe. Move to head pipe */
                            piteratorPipe = ptransferGroup->pipe;
                            ptransferGroup->currentPipe = piteratorPipe;
                        }
                    }
                    else
                    {
                        /* No need to move ahead here. Already done before */
                        piteratorPipe = ptransferGroup->currentPipe;
                    }
                    if(piteratorPipe == ptransferGroup->pipe)
                    {
                        /* Reached the beginning of the BULK pipe */
                        break;
                    }
                    pbulkIRP = piteratorPipe->irpQueueHead;
                }/* end of while() */
            }/* if(BULK PIPE present) */
            numSWEpEntry = 0;
        }/* end of if(FRAME EXPIRY)*/
        else
        {
            if((pusbdrvObj->numSWEpEntry + 1) < _DRV_USB_SW_EP_NUMBER)
            {
                numSWEpEntry = pusbdrvObj->numSWEpEntry + 1;
            }
            else
            {
                return false;
            }
        }

        /* Call appropriate transfer function to process */
        for(;numSWEpEntry < _DRV_USB_SW_EP_NUMBER;
                numSWEpEntry++)
        {
            if((pusbdrvObj->drvUSBHostSWEp[numSWEpEntry].tobeDone) == true)
            {
                if(pusbdrvObj->drvUSBHostSWEp[numSWEpEntry].transferType
                        == USB_TRANSFER_TYPE_CONTROL) 
                {
                    /* Control transfer */
                    pusbdrvObj->numSWEpEntry = numSWEpEntry;
                    tokenSent = _DRV_USB_HOST_ControlXferProcess(
                            pusbdrvObj,
                            pusbdrvObj->drvUSBHostSWEp[numSWEpEntry].pirp,
                            (DRV_USB_TRANSACTION_RESULT) 0,
                            0
                            );

                }
                else if((pusbdrvObj->drvUSBHostSWEp[numSWEpEntry].transferType
                            == USB_TRANSFER_TYPE_ISOCHRONOUS) ||
                        (pusbdrvObj->drvUSBHostSWEp[numSWEpEntry].transferType
                         == USB_TRANSFER_TYPE_INTERRUPT) ||
                        (pusbdrvObj->drvUSBHostSWEp[numSWEpEntry].transferType
                         == USB_TRANSFER_TYPE_BULK))
                {
                    if((pusbdrvObj->drvUSBHostSWEp[numSWEpEntry].tobeDone
                                == true))
                    {
                        pusbdrvObj->numSWEpEntry = numSWEpEntry;
                        tokenSent = _DRV_USB_HOST_NonControlIRPProcess(
                                pusbdrvObj,
                                pusbdrvObj->drvUSBHostSWEp[numSWEpEntry].pirp,
                                (DRV_USB_TRANSACTION_RESULT) 0,
                                0
                                );
                    }
                }
                else
                {
                    SYS_DEBUG(0, "error IRP schedule");
                }
                break;	
            }/* end of if(tobedone)*/
        }/* end of for() loop */
    }/* end of if(last transaction is 0)*/
    else
    {
        if(pusbdrvObj->numSWEpEntry == 0)
        {
            /* The last scheduled in this frame was CONTROL */
            /* Control transfer */
            tokenSent = _DRV_USB_HOST_ControlXferProcess(
                    pusbdrvObj,
                    pusbdrvObj->drvUSBHostSWEp[pusbdrvObj->numSWEpEntry].pirp,
                    (DRV_USB_TRANSACTION_RESULT) lastResult,
                    transactionSize
                    );
        }
        else
        {
            if((pusbdrvObj->drvUSBHostSWEp[pusbdrvObj->numSWEpEntry].tobeDone
                        == true))
            {
                /* Last scheduled was Isochronous, interrupt or BULK transfer */
                tokenSent = _DRV_USB_HOST_NonControlIRPProcess(
                        pusbdrvObj,
                        pusbdrvObj->drvUSBHostSWEp[pusbdrvObj->numSWEpEntry].pirp,
                        (DRV_USB_TRANSACTION_RESULT) lastResult,
                        transactionSize
                        );
            }
        }
    }

    return tokenSent;

}/* end of _DRV_USB_HOST_TransferSchedule()*/

// *****************************************************************************
/* Function:
    void _DRV_USB_HOST_Tasks_ISR(DRV_USB_OBJ * pusbdrvObj)

  Summary:
    Dynamic impementation of _DRV_USB_HOST_Tasks_ISR function.

  Description:
    This is the dynamic impementation of _DRV_USB_HOST_Tasks_ISR
    internal function. Function is an interrupt handler which does
    neccessary processing based on the interrupt.

  Remarks:

*/

void _DRV_USB_HOST_Tasks_ISR(DRV_USB_OBJ * pusbdrvObj)
{
    /* Start of local variables */
    void                    *peventData = NULL;
    DRV_USB_BDT_ENTRY       *pBDTEntry = (DRV_USB_BDT_ENTRY *) NULL;
    DRV_USB_CLIENT_OBJ      * pusbdrvClient = (DRV_USB_CLIENT_OBJ *)NULL;
    bool                    bDoEventCallBack = false;;
    bool		    tokenWasSent = false;
    uint8_t                 lastEndpoint = 0;
    uint8_t                 swEpCount = 0;
    unsigned int            transactionSize = 0;
    USB_MODULE_ID           usbID;
    DRV_USB_EVENT           eventType = 0;
    USB_INTERRUPTS          usbInterrupts = 0;
    USB_INTERRUPTS          enabledUSBInterrupts = 0;
    USB_INTERRUPTS          clearUSBInterrupts = 0;
    USB_PING_PONG_STATE     lastPingPong = 0;
    USB_BUFFER_DIRECTION    lastDirection = 0;
    DRV_USB_TRANSACTION_RESULT  transactionResult = 0;
    /* End of local variables */

    usbID = pusbdrvObj->usbID;

    usbInterrupts = PLIB_USB_InterruptFlagAllGet(usbID);

    enabledUSBInterrupts = PLIB_USB_InterruptEnableGet(usbID);

    /* Check if an error has occurred */
    if (( usbInterrupts & USB_INT_ERROR ) &&
            (enabledUSBInterrupts & USB_INT_ERROR))
    { 
        USB_ERROR_INTERRUPTS errorType;

        eventType = DRV_USB_EVENT_ERROR;
        bDoEventCallBack = true;

        errorType = PLIB_USB_ErrorInterruptFlagAllGet(usbID); 

        /* Clear the errors */
        PLIB_USB_ErrorInterruptFlagClear( usbID, errorType ); 

        /* Clear the error flag */
        PLIB_USB_InterruptFlagClear( usbID, USB_INT_ERROR ); 

        /* Set the event data */

    }


    else if (( usbInterrupts & USB_INT_SOF ) &&
            (enabledUSBInterrupts & USB_INT_SOF))
    { 
        /* SOF threshold reached by Host. Call the
         * transaction scheduler so that we schedule
         * the next set of transfers */

        bDoEventCallBack = true;
        eventType = DRV_USB_EVENT_SOF_DETECT;
        clearUSBInterrupts = USB_INT_SOF;

        /* Call the transfer scheduler */
        _DRV_USB_HOST_TransferSchedule(pusbdrvObj,
                (DRV_USB_TRANSACTION_RESULT)0,
                (unsigned int)0,
                true);
    }

    else if ((usbInterrupts & USB_INT_STALL) &&
            (enabledUSBInterrupts & USB_INT_STALL))
    {
        /* The device stalled the request. At this 
         * point we dont know yet what to do with this
         * interrupt. Clearing the flag for now */
        clearUSBInterrupts = USB_INT_STALL;
    }

    else if((usbInterrupts & USB_INT_RESUME) &&
            (enabledUSBInterrupts & USB_INT_RESUME))
    {
        /* Device sent resume signalling to the
         * host */

        bDoEventCallBack = true;
        eventType = DRV_USB_EVENT_RESUME_DETECT;
        clearUSBInterrupts = USB_INT_RESUME;
    }

    else if ((usbInterrupts & USB_INT_IDLE_DETECT)
            && (enabledUSBInterrupts & USB_INT_IDLE_DETECT))
    {
        bDoEventCallBack = true;
        eventType = DRV_USB_EVENT_IDLE_DETECT;
        clearUSBInterrupts = USB_INT_IDLE_DETECT;
    }

    else if ((usbInterrupts & USB_INT_ATTACH) && 
            (enabledUSBInterrupts & USB_INT_ATTACH))
    {
        /* Host got a device attach */
        bDoEventCallBack = true;
        eventType = DRV_USB_EVENT_HOST_ATTACH;
        clearUSBInterrupts = USB_INT_ATTACH;

        /* The attach interrupt is persistent.
         * Clearing it will not have any effect
         * It must be disabled and enabled
         * again when a detach is received */

        PLIB_USB_InterruptDisable(usbID, USB_INT_ATTACH);

        /* We also clear and enable the detach interrupt */
        PLIB_USB_InterruptFlagClear(usbID, USB_INT_HOST_DETACH);
        PLIB_USB_InterruptEnable(usbID, USB_INT_HOST_DETACH);

        /* Check if the device speed is low speed 
         * and update the driver structure */

        pusbdrvObj->deviceSpeed = USB_SPEED_FULL;
        PLIB_USB_TokenSpeedSelect(usbID, USB_FULLSPEED_TOKENS);
        PLIB_USB_EP0LSDirectConnectDisable(usbID);

        if(!PLIB_USB_JStateIsActive(usbID))
        {
            /* This means that low speed device
             * was attached. All tokens should be
             * executed at low speed. Also enable
             * direct connection to LS speed bit in
             * the EP0 control register.
             * */

            pusbdrvObj->deviceSpeed = USB_SPEED_LOW;
            PLIB_USB_TokenSpeedSelect(usbID, USB_LOWSPEED_TOKENS);
            PLIB_USB_EP0LSDirectConnectEnable(usbID);
        }

        /* Enable the transaction interrupt */
        PLIB_USB_InterruptEnable(usbID, USB_INT_TOKEN_DONE);

    }
    else if((usbInterrupts & USB_INT_HOST_DETACH)
            && (enabledUSBInterrupts & USB_INT_HOST_DETACH))
    {
        /* Host got a device detach */

        bDoEventCallBack = true;
        eventType = DRV_USB_EVENT_HOST_DETACH;
        clearUSBInterrupts = USB_INT_ATTACH;

        /* The detach interrupt is persistent.
         * Clearing it will not have any effect
         * It must be disabled and enabled
         * again when a attach is received */

        PLIB_USB_InterruptDisable(usbID, USB_INT_HOST_DETACH);

        /* We disable the token done interrupt. This will prevent
         * the control transfer process and the non control transfer
         * process from getting a null irp to process. We then clear
         * the interrupt. To clear the token interrupt, the stat
         * register should be read once. */

        PLIB_USB_InterruptDisable(usbID, USB_INT_TOKEN_DONE);
        PLIB_USB_LastTransactionDetailsGet(usbID, &lastDirection,
                &lastPingPong, &lastEndpoint );
        PLIB_USB_InterruptFlagClear(usbID, USB_INT_TOKEN_DONE);

        /* Clear up all the endpoint 0 BDT entries.*/
        pusbdrvObj->pBDT[0].word[0] = 0x0;
        pusbdrvObj->pBDT[0].word[1] = 0x0;
        pusbdrvObj->pBDT[1].word[0] = 0x0;
        pusbdrvObj->pBDT[1].word[1] = 0x0;
        pusbdrvObj->pBDT[2].word[0] = 0x0;
        pusbdrvObj->pBDT[2].word[1] = 0x0;
        pusbdrvObj->pBDT[3].word[0] = 0x0;
        pusbdrvObj->pBDT[3].word[1] = 0x0;

        /* Un-initialize the SWEP data structure */
        for(swEpCount = 0; swEpCount < _DRV_USB_SW_EP_NUMBER; swEpCount++)
        {
            pusbdrvObj->drvUSBHostSWEp[swEpCount].tobeDone = false;
        }

        /* We also clear and enable the attach interrupt */
        PLIB_USB_InterruptFlagClear(usbID, USB_INT_ATTACH);
        PLIB_USB_InterruptEnable(usbID, USB_INT_ATTACH);

    }


    /* Send all the above events to all the clients */
    if(bDoEventCallBack)
    {
        /* Clear the flag for the next time */

        bDoEventCallBack = false;

        /* The driver events are passed only to client 0
         * which is the host layer. We make sure that we
         * have a valid client. */

        if(pusbdrvObj->pDrvUSBClientObj !=
                (DRV_USB_CLIENT_OBJ *)DRV_HANDLE_INVALID)
        {
            /* Check if the client has a valid
             * callback */

            pusbdrvClient = pusbdrvObj->pDrvUSBClientObj;

            if(pusbdrvClient->pEventCallBack != NULL)
            {
                /************************************************
                 * Client call back. Since the HCD can have only
                 * one client, we will not loop through a table
                 * of client. Client 0 is the host layer.
                 ************************************************/

                pusbdrvClient->pEventCallBack(pusbdrvClient->hClientArg,
                        eventType,  peventData);
            }
        }
    }

    /* Clear interrupts flags */
    PLIB_USB_InterruptFlagClear( usbID, clearUSBInterrupts );

    /* Now check if the token was completed */
    if(PLIB_USB_InterruptFlagGet(usbID, USB_INT_TOKEN_DONE) 
            && ((enabledUSBInterrupts & USB_INT_TOKEN_DONE) != 0))
    {
        /* Get the last transaction status */
        PLIB_USB_LastTransactionDetailsGet(usbID, &lastDirection,
                &lastPingPong, &lastEndpoint );

        /* Clear the flag */
        PLIB_USB_InterruptFlagClear(usbID,USB_INT_TOKEN_DONE);

        /* Get the result of the last transaction */

        pBDTEntry = pusbdrvObj->pBDT + (lastDirection << 1) + lastPingPong;
        transactionResult = (pBDTEntry->byte[0] & 0x3C) >> 2;
        transactionSize = pBDTEntry->shortWord[1];

        if(lastDirection == USB_BUFFER_RX)
        {
            /* Update the RX ping pong buffer indicator */
            pusbdrvObj->ep0RxPingPong ^= 0x1;
        }
        else
        {
            /* Update the even ping pong buffer indicator */
            pusbdrvObj->ep0TxPingPong ^= 0x1;
        }

        tokenWasSent = _DRV_USB_HOST_TransferSchedule(pusbdrvObj,
                transactionResult,
                transactionSize, false);
        if(tokenWasSent == false)
        {
            /* No more than 1 IRP when isocronous IRP is present in 1 frame */
            /* Call the transfer scheduler */

            _DRV_USB_HOST_TransferSchedule(pusbdrvObj,
                    (DRV_USB_TRANSACTION_RESULT)0,
                    (unsigned int)0, false);
        }
    }
}/* end of _DRV_USB_HOST_Tasks_ISR() */

// *****************************************************************************
/* Function:
    void DRV_USB_HOST_BusResetControl
    (
        DRV_HANDLE handle,
        bool control
    )

  Summary:
    Dynamic impementation of DRV_USB_HOST_BusResetControl function.

  Description:
    This is the dynamic impementation of DRV_USB_HOST_BusResetControl
    internal function. Function is used to enable or disable USB BUS RESET
    signal.

  Remarks:
    Refer to drv_usb.h for usage information.
*/

void DRV_USB_HOST_BusResetControl(DRV_HANDLE handle, bool control)
{
    /* Start of local variables */
    DRV_USB_CLIENT_OBJ * pusbdrvClient = (DRV_USB_CLIENT_OBJ *)handle;
    DRV_USB_OBJ * pusbdrvObj = (DRV_USB_OBJ *)pusbdrvClient->hDriver;
    /* End of local variables */
    
    /* If control is true, then enable reset */
    if(control == true)
    {
        PLIB_USB_ResetSignalEnable(pusbdrvObj->usbID);
        /* After the rest is activated, we can
         * find out if the J state is active, 
         * which means that the D- line is pulled
         * up and hence this a low speed device.
         */
        if(PLIB_USB_JStateIsActive(pusbdrvObj->usbID))
        {
            pusbdrvObj->deviceSpeed = USB_SPEED_LOW;
        }
        else
        {
            /* Device is full speed */
            pusbdrvObj->deviceSpeed = USB_SPEED_FULL;
        }
    }
    else
    {
        PLIB_USB_ResetSignalDisable(pusbdrvObj->usbID);
    }
}/* end of DRV_USB_HOST_BusResetControl() */

// *****************************************************************************
/* Function:
    USB_SPEED DRV_USB_HOST_DeviceCurrentSpeedGet(DRV_HANDLE handle)

  Summary:
    Dynamic impementation of DRV_USB_HOST_DeviceCurrentSpeedGet function.

  Description:
    This is the dynamic impementation of DRV_USB_HOST_DeviceCurrentSpeedGet
    client interface function. Function returns the usb speed on success.

  Remarks:
    Refer to drv_usb.h for usage information.
*/

USB_SPEED DRV_USB_HOST_DeviceCurrentSpeedGet(DRV_HANDLE handle)
{
    /* Start of local variables */
    DRV_USB_CLIENT_OBJ * pusbdrvClient = (DRV_USB_CLIENT_OBJ *)handle;
    DRV_USB_OBJ * pusbdrvObj = (DRV_USB_OBJ *)NULL;
    USB_SPEED returnValue = USB_SPEED_ERROR;
    /* End of local variables */

    /* Check if the handle is valid or opened */
    if((handle == DRV_HANDLE_INVALID) ||
            (pusbdrvClient->inUse == false))
    {
        SYS_DEBUG(0, "Bad Client or client closed");
    }
    else
    {
        /* Map the USB HW instance from client
         * and get the device speed
         */
        pusbdrvObj = pusbdrvClient->hDriver;
        returnValue = pusbdrvObj->deviceSpeed;
    }
    
    return returnValue;

}/* end of DRV_USB_HOST_DeviceCurrentSpeedGet() */

// *****************************************************************************
/* Function:
    void DRV_USB_HOST_OperationEnable(DRV_HANDLE handle, bool enable)


  Summary:
    Dynamic impementation of DRV_USB_HOST_OperationEnable function.

  Description:
    This is the dynamic impementation of DRV_USB_HOST_OperationEnable client
    interface function. Function enables\disables the USB Host operation as per
    enable value 1 and 0 respectively.

  Remarks:
    Refer to drv_usb.h for usage information.

*/

void DRV_USB_HOST_OperationEnable(DRV_HANDLE handle, bool enable)
{
    /* Start of local variable */
    DRV_USB_CLIENT_OBJ * pusbdrvClient = (DRV_USB_CLIENT_OBJ *)handle;
    DRV_USB_OBJ * pusbdrvObj = (DRV_USB_OBJ *)NULL;
    /* End of local variable */

    /* Check if the handle is valid or opened */
    if((handle == DRV_HANDLE_INVALID) ||
            (pusbdrvClient->inUse == false))
    {
        SYS_DEBUG(0, "Bad Client or client closed");
    }
    else
    {
        pusbdrvObj = pusbdrvClient->hDriver;
        if(true == enable)
        {
            /* Interrupt flag cleared on the safer side */
            SYS_INT_SourceStatusClear(pusbdrvObj->interruptSource);

            if(DRV_USB_INTERRUPT_MODE)
            {
                /* If interrupt mode is set then enable interrupts
                 * Interrupt priority will be set by the application
                 */
                SYS_INT_SourceEnable(pusbdrvObj->interruptSource);
            }

            pusbdrvObj->isEnabled = true;
            /* Enable the USB module */
            PLIB_USB_Enable(pusbdrvObj->usbID);
        }
        else
        {
            SYS_INT_SourceStatusClear(pusbdrvObj->interruptSource);
            pusbdrvObj->isEnabled = false;
            /* Disable the USB module */
            PLIB_USB_Disable(pusbdrvObj->usbID);
        }
    }

}/* end of DRV_USB_HOST_OperationEnable() */

// *****************************************************************************
/* Function:
    void DRV_USB_HOST_OperationEnable(DRV_HANDLE handle, bool enable)

  Summary:
    Dynamic impementation of DRV_USB_HOST_OperationIsEnabled function.

  Description:
    This is the dynamic impementation of DRV_USB_HOST_OperationIsEnabled
    client interface function. Function returns true or false based on
    USB Host module is enabled or not respectively.

  Remarks:
    Refer to drv_usb.h for usage information.

*/

bool DRV_USB_HOST_OperationIsEnabled(DRV_HANDLE handle)
{
    /* Start of local variable */
    DRV_USB_CLIENT_OBJ * pusbdrvClient = (DRV_USB_CLIENT_OBJ *)handle;
    DRV_USB_OBJ * pusbdrvObj = (DRV_USB_OBJ *)NULL;
    bool returnValue = false;
    /* End of local variable */

    /* Check if the handle is valid or opened */
    if((handle == DRV_HANDLE_INVALID) ||
            (pusbdrvClient->inUse == false))
    {
        SYS_DEBUG(0, "Bad Client or client closed");
    }
    else
    {
        pusbdrvObj = pusbdrvClient->hDriver;
        returnValue = pusbdrvObj->isEnabled;
    }

    return returnValue;

}/* end of DRV_USB_HOST_OperationIsEnabled() */
