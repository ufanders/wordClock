/*******************************************************************************
  USB Device Driver Definition

Company:
Microchip Technology Inc.

File Name:
drv_usb_host.c

Summary:
USB Device Driver Implementation

Description:
The USB device driver provides a simple interface to manage the USB
modules on Microchip microcontrollers.  This file Implements the  host mode
interface routines for the USB driver.

While building the driver from source, ALWAYS use this file in the build if 
USB host mode operation is required.
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

#include "driver/usb/drv_usb.h"
#include "driver/usb/src/drv_usb_local.h"

/*****************************************
 * Pool of pipe objects that is used by
 * all driver instances.
 *****************************************/
DRV_USB_HOST_PIPE_OBJ gDrvUSBHostPipeObj[DRV_USB_HOST_PIPES_NUMBER];

#define USB_END_POINT(x) x

void _DRV_USB_SendTokenToAddress(USB_MODULE_ID usbID, uint8_t address,
        USB_PID pid, uint8_t endpoint, bool isLowSpeed)
{
    /* Created a function call for this because PLIB function are 
     * inline and this function is being called at several locations */

    PLIB_USB_TokeSend(usbID, pid, endpoint, address, isLowSpeed);

}

void _DRV_USB_HOST_Initialize(DRV_USB_OBJ * drvObj, SYS_MODULE_INDEX index)
{
    int iEntry;

    PLIB_USB_SOFThresholdSet(drvObj->usbID, 0x4A);

    /* Enable the VBUSON bit in the OTGCON register */

    PLIB_USB_OTG_VBusPowerOn(drvObj->usbID);

    /* Select the host mode of operation */

    PLIB_USB_OperatingModeSelect( drvObj->usbID, USB_OPMODE_HOST);

    /* Enable the attach interrupt */

    PLIB_USB_InterruptEnable(drvObj->usbID, USB_INT_ATTACH);

    /* Clear up the endpoint 0 BDT entries */
    for(iEntry = 0; iEntry < 4; iEntry ++)
    {
        /* A full duplex endpoint has 4
         * entries, 2 for each direction */

        drvObj->pBDT[iEntry].word[0] = 0;
        drvObj->pBDT[iEntry].word[1] = 0;
    }

    /* Clear the odd even buffer pointers */

    drvObj->ep0TxPingPong = USB_BUFFER_EVEN;
    drvObj->ep0RxPingPong = USB_BUFFER_EVEN;

    /* Configure endpoint 0 register. */

    PLIB_USB_EP0HostSetup(drvObj->usbID);
}

USB_ERROR DRV_USB_HOST_IRPSubmit(DRV_USB_HOST_PIPE_HANDLE  hPipe, USB_HOST_IRP * inputIRP)
{
  
    USB_HOST_IRP_LOCAL * irpIterator;
    bool interruptWasEnabled;
    
    USB_HOST_IRP_LOCAL * irp        = (USB_HOST_IRP_LOCAL *)inputIRP;
    DRV_USB_HOST_PIPE_OBJ * pipe    = (DRV_USB_HOST_PIPE_OBJ *)hPipe;
    DRV_USB_CLIENT_OBJ * hClient    = (DRV_USB_CLIENT_OBJ *)(pipe->hClient);
    DRV_USB_OBJ * hDriver           = (DRV_USB_OBJ *)hClient->hDriver;

    /* Assign owner pipe */
    irp->pipe = hPipe;

    /* Clear up any temporary state */
    irp->tempState = 0;

    /* Check the transfer type */
    if(pipe->pipeType == USB_TRANSFER_TYPE_CONTROL)
    {
        /* Then we setup the IRP for setup stage */

        irp->tempState = DRV_USB_HOST_IRP_STATE_SETUP_STAGE;  
    }

    irp->status = USB_HOST_IRP_STATUS_PENDING;

    /* Add the IRP to the pipe. We need to disable
     * the SOF interrupt here because the SOF interrupt
     * updates the pipe structure asynchronously. */

    if(!hDriver->isInInterruptContext)
    {
        interruptWasEnabled = SYS_INT_SourceIsEnabled(hDriver->interruptSource);
        // OSAL: Get Mutex
        _DRV_USB_InterruptSourceDisable(hDriver->interruptSource);
    }
    
    if(pipe->irpQueueHead == NULL)
    {
        /* This means that there are no
         * IRPs on this pipe. We can add
         * this IRP directly */

        irp->next = NULL;
        irp->previous = NULL;
        pipe->irpQueueHead = irp;
    }
    else
    {
        irpIterator = pipe->irpQueueHead;

        /* Find the last IRP in the linked list*/
        while(irpIterator->next != 0)
        {
            irpIterator = irpIterator->next;
        }

        /* Add the item to the last irp in the linked list */
        irpIterator->next = irp;
        irp->previous = irpIterator;
      
    }

    /* Initialize the hidden members */
    irp->next = NULL;
    irp->completedBytes = 0;

   
    if(!hDriver->isInInterruptContext)
    {
        if(interruptWasEnabled)
        {
            _DRV_USB_InterruptSourceEnable(hDriver->interruptSource);
        }
        //OSAL: Return Mutex
    }

    return USB_ERROR_NONE;
}

bool DRV_USB_HOST_TimerStart(DRV_HANDLE client, uint32_t periodInMilliseconds)
{
    /* This function starts a software timer.
     * It returns true if the timer could be
     * started. If the timer countdown function
     * is alreay in progress, then it returns 
     * false
     */

    DRV_USB_CLIENT_OBJ * hClient = (DRV_USB_CLIENT_OBJ *)client; 
    DRV_USB_OBJ * hDriver = (DRV_USB_OBJ *)hClient->hDriver;

    if(hDriver->timerCount > 0)
    {
        /* This means a count is in progress */
        return false;
    }

    hDriver->timerCount = periodInMilliseconds;
    PLIB_USB_OTG_InterruptEnable(hDriver->usbID, USB_OTG_INT_ONE_MS_TIMEOUT);
    return true;

}

bool DRV_USB_HOST_TimerIsComplete(DRV_HANDLE client)
{
    /*If the timer Count is zero, then
     * time count down is complete */

    DRV_USB_CLIENT_OBJ * hClient = (DRV_USB_CLIENT_OBJ *)client; 
    DRV_USB_OBJ * hDriver = (DRV_USB_OBJ *)hClient->hDriver;

    if(hDriver->timerCount == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void DRV_USB_HOST_TimerReset(DRV_HANDLE client)
{
    DRV_USB_CLIENT_OBJ * hClient = (DRV_USB_CLIENT_OBJ *)client; 
    DRV_USB_OBJ * hDriver = (DRV_USB_OBJ *)hClient->hDriver;

    /* Disable the timer interrupt */
    PLIB_USB_OTG_InterruptDisable(hDriver->usbID, USB_OTG_INT_ONE_MS_TIMEOUT);
    hDriver->timerCount = 0;
}


void DRV_USB_HOST_IRPCancel(USB_HOST_IRP * inputIRP)
{
    /* This function cancels an IRP */

    USB_HOST_IRP_LOCAL * irp = (USB_HOST_IRP_LOCAL *) inputIRP;
    DRV_USB_OBJ * hDriver;
    DRV_USB_CLIENT_OBJ * client;
    DRV_USB_HOST_PIPE_OBJ * pipe;
    bool interruptWasEnabled;


    if(irp->pipe == DRV_USB_HOST_PIPE_HANDLE_INVALID)
    {
        SYS_ASSERT(false, "Invalid pipe");
        return;
    }

    if(irp->status <= USB_HOST_IRP_STATUS_COMPLETED_SHORT)
    {
        SYS_ASSERT(false, "IRP is not pending or in progress");
        return;
    }

    pipe = (DRV_USB_HOST_PIPE_OBJ *)irp->pipe;
    client = (DRV_USB_CLIENT_OBJ *) pipe->hClient;
    hDriver = (DRV_USB_OBJ *) client->hDriver;

    if(!hDriver->isInInterruptContext)
    {
        interruptWasEnabled = SYS_INT_SourceIsEnabled(hDriver->interruptSource);
        //OSAL: Get Mutex
        _DRV_USB_InterruptSourceDisable(hDriver->interruptSource);
    }

    if(irp->previous == NULL)
    {
        /* This means this was the first
         * irp in the queue. Update the pipe
         * queue head directly */

        pipe->irpQueueHead = irp->next;
        if(irp->next != NULL)
        {
            irp->next->previous = NULL;
        }
    }
    else
    {
        /* Remove the IRP from the linked
         * list */
        irp->previous->next = irp->next;

        if(irp->next != NULL)
        {
            /* This applies if this is not the last
             * irp in the linked list */
            irp->next->previous = irp->previous;
        }
    }

    if(irp->status == USB_HOST_IRP_STATUS_IN_PROGRESS)
    {
        /* If the irp is already in progress then
         * we set the temporary state. This will get
         * caught in _DRV_USB_HOST_ControlXferProcess()
         * and _DRV_USB_HOST_NonControlIRPProcess() 
         * functions. */

        irp->tempState = DRV_USB_HOST_IRP_STATE_ABORTED;
    }
    else
    {
        irp->status = USB_HOST_IRP_STATUS_ABORTED;
        if(irp->callback != NULL)
        {
            irp->callback((USB_HOST_IRP *)irp);
        }
    }

    if(!hDriver->isInInterruptContext)
    {
        if(interruptWasEnabled)
        {
            _DRV_USB_InterruptSourceDisable(hDriver->interruptSource);
        }
        //OSAL: Release Mutex
    }
}


bool _DRV_USB_HOST_ControlXferProcess(DRV_USB_OBJ * hDriver, 
        USB_HOST_IRP_LOCAL * irp, DRV_USB_TRANSACTION_RESULT deviceResponse,
        unsigned int deviceResponseSize)
{

    /* This function returns true if a token was sent 
     * at the time when the function exits */

    bool tokenSent                  = false;
    bool endIRP                     = false;    
 	DRV_USB_HOST_PIPE_OBJ * pipe    = (DRV_USB_HOST_PIPE_OBJ *)irp->pipe;
    uint8_t endpoint                = pipe->endpointAndDirection & 0xF;
    uint8_t deviceAddress           = pipe->deviceAddress;
    USB_MODULE_ID usbID             = hDriver->usbID;
    bool isLowSpeed                 = (pipe->speed == USB_SPEED_LOW) ? true : false;


    DRV_USB_BDT_ENTRY * pBDT;

    if(irp->tempState == DRV_USB_HOST_IRP_STATE_ABORTED)
    {
        /* This means the application aborted this IRP.
         * We just invoke the callback */

        irp->status = USB_HOST_IRP_STATUS_ABORTED; 
        hDriver->frameIRPTable[0] = NULL;
        if(irp->callback != NULL)
        {
            irp->callback((USB_HOST_IRP *)irp);
        }

        /* Return false indicating that no token 
         * was sent in this frame */

        return(false);
    }

    switch(irp->tempState)
    {
        case DRV_USB_HOST_IRP_STATE_SETUP_STAGE:

            /* Then send the SETUP Token 
             * Let the irp know that the process
             * has started.
             *  */

            irp->status = USB_HOST_IRP_STATUS_IN_PROGRESS;
            irp->completedBytes = 0;

            if((((uint8_t *)(irp->setup))[0] & 0x80) != 0)
            {
                /* Data stage direction is from device
                 * to host */

                pipe->endpointAndDirection |= (USB_DATA_DIRECTION_DEVICE_TO_HOST << 7);
            }
            else
            {
                /* Data stage direction is from host to
                 * device */
                pipe->endpointAndDirection &= 0xF;
                pipe->endpointAndDirection |= (USB_DATA_DIRECTION_HOST_TO_DEVICE << 7);
            }

            /* Keep track of the transaction */
            irp->tempState = DRV_USB_HOST_IRP_STATE_SETUP_TOKEN_SENT;

            pBDT = hDriver->pBDT + 2 + hDriver->ep0TxPingPong;
            
            /* COnfigure the BDT Entry for setup packet
             * and Data 0 toggle */
            pBDT->shortWord[1]  = 8;
            pBDT->word[1]       = KVA_TO_PA(irp->setup);
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
                if((irp->data == NULL) || 
                  (irp->size == 0))
                {
                    /* This means that this is a zero data stage
                     * transaction */

                    irp->tempState = DRV_USB_HOST_IRP_STATE_HANDSHAKE;
                    pipe->nakCounter = 0;
                    break;
                }

                /* Setup the data toggle for the next stage
                 * and go to the next stage directly.
                 * Switch case fall through is intentional. */
                pipe->dataToggle = USB_BUFFER_DATA1;
                pipe->nakCounter = 0;
                irp->tempState = DRV_USB_HOST_IRP_STATE_DATA_STAGE;
            }
            else
            {
                irp->tempState = DRV_USB_HOST_IRP_STATE_COMPLETE;
                irp->status = USB_HOST_IRP_STATUS_ERROR_UNKNOWN;
                endIRP = true;
                break;
            }

        case DRV_USB_HOST_IRP_STATE_DATA_STAGE:
            /* Send the data stage */
            {
                USB_BUFFER_DIRECTION direction;
                USB_BUFFER_PING_PONG pingPong;
                USB_PID pid;
                unsigned int size;

                if((pipe->endpointAndDirection & 0x80)  != 0)
                {
                    /* Direction is device to host */

                    direction = USB_BUFFER_RX;
                    pingPong = hDriver->ep0RxPingPong;
                    pid = USB_PID_IN;
                }
                else
                {
                    /* Direction is host to device */
                    direction = USB_BUFFER_TX;
                    pingPong = hDriver->ep0TxPingPong;
                    pid = USB_PID_OUT;
                }

                if((irp->size - irp->completedBytes)
                        >= pipe->endpointSize)
                {
                    /* This means we have to break
                     * up the transfer into transactions */

                    size = pipe->endpointSize;

                }
                else
                {
                    /* Data size is less than endpoint size */
                    size = irp->size;
                }

                /* Keep track of the transaction */
                irp->tempState = DRV_USB_HOST_IRP_STATE_DATA_STAGE_SENT;

                pBDT = hDriver->pBDT + (2 * direction ) + pingPong;

                /* COnfigure the BDT Entry for data packet */
                pBDT->shortWord[1]  = size;
                pBDT->word[1]       = KVA_TO_PA(irp->data + irp->completedBytes);
                pBDT->byte[0]       = 0x88 | (pipe->dataToggle << 6);

                /* This will cause the Transaction interrupt */
                _DRV_USB_SendTokenToAddress(usbID, deviceAddress,
                        pid, endpoint, isLowSpeed);

                tokenSent = true;
                break;
            }

        case DRV_USB_HOST_IRP_STATE_DATA_STAGE_SENT:
 
            /* Check the response to data stage here */
 
            if(deviceResponse == USB_TRANSACTION_NAK)
            {
                /* This means the device is not ready
                 * with the data. Rewind the state to
                 * the previous state. We will try again
                 * on the next SOF*/

                irp->tempState = DRV_USB_HOST_IRP_STATE_DATA_STAGE;
                pipe->nakCounter ++;
                if(pipe->nakCounter >= DRV_USB_HOST_NAK_LIMIT)
                {
                    irp->tempState = DRV_USB_HOST_IRP_STATE_COMPLETE;
                    irp->status = USB_HOST_IRP_STATUS_ERROR_NAK_TIMEOUT;
                    endIRP = true;
                    pipe->nakCounter = 0;
                }

                break;

            }
            else if (deviceResponse == USB_TRANSACTION_STALL)
            {
                pipe->nakCounter = 0;
	            irp->status = USB_HOST_IRP_STATUS_ERROR_STALL;
	            irp->tempState =  DRV_USB_HOST_IRP_STATE_COMPLETE;
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
                irp->completedBytes += deviceResponseSize;

                if((pipe->endpointAndDirection & 0x80) != 0)
                {
                    /* Data is moving from device to host 
                     * Check if the data stage is done */

                    if((deviceResponseSize < pipe->endpointSize)
                            ||(irp->completedBytes == irp->size)) 
                    {
                        /* A host control read transfer is done
                         * when the host has received the amount
                         * of data that it was looking for or when
                         * the host receives a less than maxPacketSize
                         * data packet. */

                        /* Reset the nak counter for the next 
                         * data transaction. Fall through to the
                         * handshake stage. */
                        
                        irp->tempState = DRV_USB_HOST_IRP_STATE_HANDSHAKE;
                    }
                    else
                    {
                        /* We have requested for more data and
                         * the device has sent only endpoint size
                         * data. More transactions are required.
                         * Each transaction will be triggered 
                         * in a SOF. Update the data toggle. */
						
						pipe->dataToggle ^= 0x1;
                        irp->tempState = DRV_USB_HOST_IRP_STATE_DATA_STAGE;
                        break;
                    }
                }
                else 
                {
                    /* Data is moving from host to device */

                    if(irp->completedBytes >= irp->size)
                    {
	                    /* The write transfer is complete. Fall through
	                     * to the handshake stage */
	                     
                        pipe->nakCounter = 0;
                        irp->tempState = DRV_USB_HOST_IRP_STATE_HANDSHAKE;
                    }
                    else
                    {
	                    /* This means more transactions are needed */
	                    pipe->dataToggle ^= 0x1;
                        irp->tempState = DRV_USB_HOST_IRP_STATE_DATA_STAGE;
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
                    pingPong = hDriver->ep0TxPingPong;
                    pid = USB_PID_OUT;
                }
                else 
                {
                    /* Data is moving from host to device */
                    direction = USB_BUFFER_RX;
                    pingPong = hDriver->ep0RxPingPong;
                    pid = USB_PID_IN;
                }
                
                pBDT = hDriver->pBDT + (2 * direction ) + pingPong;

                /* COnfigure the BDT Entry for data packet */
                pBDT->shortWord[1]  = 0;
                pBDT->word[1]       = 0;
                pBDT->byte[0]       = 0xC8;

                /* Keep track of the transaction */
                irp->tempState = DRV_USB_HOST_IRP_STATE_HANDSHAKE_SENT;

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
                irp->tempState = DRV_USB_HOST_IRP_STATE_COMPLETE;
                irp->status = USB_HOST_IRP_STATUS_COMPLETED;
                if(((pipe->endpointAndDirection & 0x80) != 0)
                        && (irp->size > irp->completedBytes))
                {
                    /* While moving data from device to host, if
                     * we received less data from the device than
                     * expected, then indicate a short packet and 
                     * set the irp size to to actual size */ 

                    irp->status = USB_HOST_IRP_STATUS_COMPLETED_SHORT;
                    irp->size = irp->completedBytes;
                }
                endIRP = true;
            }
            else if(USB_TRANSACTION_NAK == deviceResponse)
            {
                irp->tempState = DRV_USB_HOST_IRP_STATE_HANDSHAKE;
                pipe->nakCounter ++;
                if(pipe->nakCounter > DRV_USB_HOST_NAK_LIMIT)
                {
                    irp->tempState = DRV_USB_HOST_IRP_STATE_COMPLETE;
                    irp->status = USB_HOST_IRP_STATUS_ERROR_NAK_TIMEOUT;
                    endIRP = true;
                }
                break;
            }
            else if(USB_TRANSACTION_STALL == deviceResponse)
            {
                irp->tempState = DRV_USB_HOST_IRP_STATE_COMPLETE;
                irp->status = USB_HOST_IRP_STATUS_ERROR_STALL;
                endIRP = true;
                break;
            }

        case DRV_USB_HOST_IRP_STATE_COMPLETE:
            /* Get the driver irp object back*/
            break;

        default:
            break;
    }

    if(endIRP == true)
    {
        /* This means we need to end the IRP for
         * any reason. Remove the irp from the 
         * from the frame table index. */

        hDriver->frameIRPTable[0] = NULL;
        pipe->irpQueueHead = irp->next;
        if(irp->callback != NULL)
        {
            irp->callback((USB_HOST_IRP *)irp);
        }
   
        
    }

    return(tokenSent);
}

void DRV_USB_HOST_PipeClose
(
    DRV_USB_HOST_PIPE_HANDLE pipeHandle
)
{
    /* This function closes an open pipe */

    bool                        interruptWasEnabled;
    DRV_USB_OBJ                 * hDriver;
    DRV_USB_CLIENT_OBJ          * client;
    USB_HOST_IRP_LOCAL          * irp;
    DRV_USB_HOST_PIPE_OBJ       * pipe;
    DRV_USB_HOST_TRANSFER_GROUP * transferGroup;
    
    /* Make sure we have a valid pipe */
    if(pipeHandle == DRV_USB_HOST_PIPE_HANDLE_INVALID)
    {
        SYS_ASSERT(false, "Invalid pipe handle");
        return;
    }

    pipe = (DRV_USB_HOST_PIPE_OBJ*) pipeHandle;

    /* Make sure tha we are working with a pipe 
     * in use */
    if(pipe->inUse != true)
    {
        SYS_ASSERT(false, "Pipe is not in use");
        return;
    }

    client = (DRV_USB_CLIENT_OBJ *)pipe->hClient;
    hDriver = (DRV_USB_OBJ *)client->hDriver;

    /* Disable the interrupt */

    if(!hDriver->isInInterruptContext)
    {
        //OSAL: Get Mutex
        interruptWasEnabled = SYS_INT_SourceIsEnabled(hDriver->interruptSource);
        _DRV_USB_InterruptSourceDisable(hDriver->interruptSource);
    }

    transferGroup = &hDriver->transferGroup[pipe->pipeType];
    
    if(pipe->previous == NULL)
    {
        /* The previous pipe could be null
         * if this was the first pipe in the
         * transfer group */

        transferGroup->pipe = pipe->next;
        if(pipe->next != NULL)
        {
            pipe->next->previous = NULL;
        }
    }
    else
    {
        /* Remove this pipe from the linked
         * list */

        pipe->previous->next = pipe->next;
        if(pipe->next != NULL)
        {
            pipe->next->previous = pipe->previous;
        }
    }

    if(transferGroup->nPipes != 0)
    {
        /* Reduce the count only if its
         * not zero already */

        transferGroup->nPipes --;
    }

    /* Now we invoke the call back for each IRP
     * in this pipe and say that it is aborted.
     * If the IRP is in progress, then that IRP
     * will be actually aborted on the next SOF
     * This will allow the USB module to complete
     * any transaction that was in progress. */

    irp = (USB_HOST_IRP_LOCAL *)pipe->irpQueueHead;
    while(irp != NULL)
    {
        irp->pipe = DRV_USB_HOST_PIPE_HANDLE_INVALID;

        if(irp->status == USB_HOST_IRP_STATUS_IN_PROGRESS)
        {
            /* If the IRP is in progress, then we set the 
             * temp IRP state. This will be caught in 
             * the _DRV_USB_HOST_NonControlIRPProcess()
             * and _DRV_USB_HOST_ControlXferProcess() 
             * functions */
            irp->tempState = DRV_USB_HOST_IRP_STATE_ABORTED;
        }
        else
        {
            /* IRP is pending */
            irp->status = USB_HOST_IRP_STATUS_ABORTED;
            if(irp->callback != NULL)  
            {
                irp->callback((USB_HOST_IRP*)irp);
            }
        }
        irp = irp->next;
    }

    /* Now we return the pipe back to the 
     * driver */

    pipe->inUse = false;

    /* Enable the interrupts */
    if(!hDriver->isInInterruptContext)
    {
        if(interruptWasEnabled)
        {
            _DRV_USB_InterruptSourceEnable(hDriver->interruptSource);
        }
        //OSAL: Return Mutex
    }

}


DRV_USB_HOST_PIPE_HANDLE DRV_USB_HOST_PipeSetup 
(
    DRV_HANDLE client,
    uint8_t deviceAddress, 
    USB_ENDPOINT endpointAndDirection,
    USB_TRANSFER_TYPE pipeType, 
    uint8_t bInterval, 
    uint16_t wMaxPacketSize,
    USB_SPEED speed
)
{
    int iEntry;
    DRV_USB_HOST_PIPE_OBJ * pipe;
    DRV_USB_HOST_PIPE_OBJ * iteratorPipe;
    DRV_USB_HOST_TRANSFER_GROUP * transferGroup;
    DRV_USB_OBJ * hDriver;
    DRV_USB_CLIENT_OBJ * hClient = (DRV_USB_CLIENT_OBJ *)client;

    hDriver = (DRV_USB_OBJ *)hClient->hDriver;

    /* Search for a free pipe object */

    /* OSAL - mutex lock */
    for(iEntry = 0; iEntry <= DRV_USB_HOST_PIPES_NUMBER; iEntry ++)
    {
        /* Check for free pipe object */

        if(gDrvUSBHostPipeObj[iEntry].inUse == false)
        {
            /* We found a pipe object that we can use */

            pipe = &gDrvUSBHostPipeObj[iEntry];

            pipe->inUse         = true;
            pipe->deviceAddress = deviceAddress;
            pipe->irpQueueHead  = NULL;
            pipe->bInterval     = bInterval;
            pipe->speed         = speed;
            pipe->pipeType      = pipeType;
            pipe->hClient       = client;
            pipe->endpointSize  = wMaxPacketSize;
            pipe->intervalCounter = bInterval;

            pipe->endpointAndDirection   = endpointAndDirection;
            
            /* This pipe should be added to the 
             * respective transfer group */

            transferGroup = &hDriver->transferGroup[pipeType];

            if(transferGroup->pipe == NULL)
            {
                /* This if the first pipe to be setup */

                transferGroup->pipe = pipe;
                transferGroup->currentPipe = pipe;
                pipe->previous = NULL;
            }
            else
            {
                /* This is not the first pipe. Find
                 * the last pipe in the linked list */

                iteratorPipe = transferGroup->pipe;
                while(iteratorPipe->next != NULL)
                {
                    /* This is not the last pipe in 
                     * this transfer group */
                    iteratorPipe = iteratorPipe->next;
                }
                iteratorPipe->next = pipe;
                pipe->previous = iteratorPipe;
            }
            pipe->next = NULL;

            /* Update the pipe count in the transfer 
             * group */

            transferGroup->nPipes ++;
            return((DRV_USB_HOST_PIPE_HANDLE)pipe);
        }/* End of pipe object found */ 
    }
    /*OSAL - Mutex Unlock */

    SYS_ASSERT(false, "Could not find a free pipe object");

    /* Return the handle */
    return(DRV_USB_HOST_PIPE_HANDLE_INVALID);

}

void _DRV_USB_HOST_NonControlIRPProcess
(
    DRV_USB_OBJ * hDriver,
    USB_HOST_IRP_LOCAL * irp, 
    DRV_USB_TRANSACTION_RESULT lastTransactionResult,
    int lastTransactionsize
)
{
    int         direction;
    int			size;
    bool        endIRP;
    uint8_t     endpoint;
    uint8_t     deviceAddress;

    USB_PID                 pid;    
    USB_MODULE_ID           usbID;
    DRV_USB_BDT_ENTRY       * pBDT;
    USB_BUFFER_PING_PONG    pingpong;
    DRV_USB_HOST_PIPE_OBJ   * pipe;

    pBDT        = hDriver->pBDT;
    usbID       = hDriver->usbID;
    

    if(irp->tempState == DRV_USB_HOST_IRP_STATE_ABORTED)
    {
        /* This means that this IRP was aborted
         * by the application while it was in 
         * progress. Terminate it now. */

        irp->status = USB_HOST_IRP_STATUS_ABORTED;
        if(irp->callback != NULL)
        {
            irp->callback((USB_HOST_IRP*)irp);
        }

        return;
    }

    pipe        = (DRV_USB_HOST_PIPE_OBJ *)irp->pipe;
	isLowSpeed 	= (pipe->speed == USB_SPEED_LOW) ? true : false;
    endpoint    = pipe->endpointAndDirection & 0xF;
    deviceAddress   = pipe->deviceAddress;

    if(lastTransactionResult == 0)
    {
        /* This means that a token need to be sent */

        irp->status = USB_HOST_IRP_STATUS_IN_PROGRESS;
        if((pipe->endpointAndDirection & 0x80) != 0)
        {
            /* Data is moving from device to host */
            direction = USB_BUFFER_RX;
            pingpong = hDriver->ep0RxPingPong;
            pid = USB_PID_IN;
        }
        else 
        {
            /* Data is moving from host to device */
            direction = USB_BUFFER_TX;
            pingpong = hDriver->ep0TxPingPong;
            pid = USB_PID_OUT;
        }

        if((irp->size - irp->completedBytes)
                >= pipe->endpointSize)
        {
            size = pipe->endpointSize;
        } 
        else
        {
            size = irp->size;
        }

        pBDT = hDriver->pBDT + (2 * direction) + pingpong;

        /*Configure the BDT Entry for data packet */
        pBDT->shortWord[1] = size;
        pBDT->word[1] = KVA_TO_PA(irp->data + irp->completedBytes);
        pBDT->byte[0] = 0x88 | (pipe->dataToggle << 6);

        /* This will cause a transaction interrupt */
        _DRV_USB_SendTokenToAddress(usbID, deviceAddress,
                pid, endpoint, isLowSpeed);

        
    }
    else
    {
        /* This means this function was called from the TRNIF
         * interrupt after the token was sent. Check the 
         * transaction result */

        endIRP = false;
        switch(lastTransactionResult)
        {
            case USB_TRANSACTION_ACK:
            case USB_TRANSACTION_DATA0:
            case USB_TRANSACTION_DATA1:    
                irp->completedBytes += lastTransactionsize;
                pipe->dataToggle ^= 0x1;
                pipe->nakCounter = 0;
                if((lastTransactionsize < pipe->endpointSize)
                        || (irp->completedBytes >= irp->size))
                {
                    /* We received data less than endpoint size.
                     * So we end the transfer */

                    if(irp->size < irp->completedBytes)
                    {
                        irp->status = USB_HOST_IRP_STATUS_COMPLETED_SHORT;
                    }
                    else
                    {
                        irp->status = USB_HOST_IRP_STATUS_COMPLETED;
                    }
                    endIRP = true;
                }
                break;
            case USB_TRANSACTION_STALL:
                /* The token was stalled. We end the IRP */
                pipe->nakCounter = 0;
                irp->status = USB_HOST_IRP_STATUS_ERROR_STALL;
                endIRP = true;
                break;
            case USB_TRANSACTION_NAK:
                /* For non - control transfer we dont implement a 
                 * NAK timeout. So dont do anything here */
                break;
            default:
                break;
        }

        if(endIRP == true)
        {
            /* This means we need to end the IRP for
             * any reason. Remove the irp from the 
             * from the frame table index. */

            pipe->irpQueueHead = irp->next;
            
            if(irp->callback != NULL)
            {
                irp->callback((USB_HOST_IRP *)irp);
            }
            
        }

    }
}

void _DRV_USB_HOST_TransferSchedule(DRV_USB_OBJ * hDriver)
{
    DRV_USB_HOST_TRANSFER_GROUP * transferGroup;
    DRV_USB_HOST_PIPE_OBJ * iteratorPipe;
    USB_HOST_IRP_LOCAL * controlIRP;
    int iEntry;

    int pipeCount;

    /* The transfer scheduler will check all
     * the pipes and choose the IRP's that 
     * need to be scheduled within the frame
     * and then start the transaction */

    /* The following loop checks each entry in 
     * the frame IRP table. The frame IRP table
     * contains the IRPs that will attempting in
     * this frame. The first entry (at index 0) 
     * is always a control transfer. The rest 
     * of the table is shared between isochronous,
     * interrupt and bulk transfers. */

    /* The non control transfers are always removed
     * from the frame IRP table at the start of a 
     * frame. This allows us to re-schedule the 
     * periodic transfers at proper intervals. If the 
     * IRP is complete then it is removed from pipe
     * irp queue */

    for(iEntry = 1; 
            iEntry < DRV_USB_HOST_IRP_PER_FRAME_NUMBER; 
            iEntry ++)
    {
        hDriver->frameIRPTable[iEntry] = NULL;
    }

    /* Always start at the top of the Frame
     * IRP table. We first search for control
     * transfers */

    hDriver->frameIRPTableIndex = 0;
    transferGroup = &hDriver->transferGroup[USB_TRANSFER_TYPE_CONTROL];

    if((transferGroup->pipe != NULL) &&
            (hDriver->frameIRPTable[0] == NULL))
    {
        controlIRP = NULL;

        /* No control transfer has been scheduled.
         * Check if a control transfer is available
         * in the control transfer group
         */

        /* The currentpipe member of the transfer
         * group structure points to the pipe
         * that needs to checked for availability
         * of a control transfer */

        for(pipeCount = 0; pipeCount < transferGroup->nPipes; pipeCount++)
        {
            iteratorPipe = transferGroup->currentPipe;
            controlIRP = iteratorPipe->irpQueueHead;

            /* We must update the current pipe member to point to the
             * next pipe in the transfer group. If this
             * NULL, then go back to the first pipe */

            transferGroup->currentPipe = iteratorPipe->next;
            if(transferGroup->currentPipe == NULL)
            {
                transferGroup->currentPipe = transferGroup->pipe;
            }

            if (controlIRP != NULL)
            {
                /* We found a control IRP. */
                hDriver->frameIRPTable[0] = controlIRP;
                break;
            }
        }
    }

    /* ************************************************
     * Now we check for interrupt transfers. Interrupt
     * transfers start from 1.
     *************************************************/

    hDriver->frameIRPTableIndex = 1;
    transferGroup = &hDriver->transferGroup[USB_TRANSFER_TYPE_INTERRUPT];
    if(transferGroup->pipe != NULL)
    {
        /* This means that there are one or more 
         * interrupt pipes Update the interval 
         * count on interrupt pipe */

        iteratorPipe = transferGroup->pipe;
        while((hDriver->frameIRPTableIndex 
                    < DRV_USB_HOST_IRP_PER_FRAME_NUMBER) &&
                (iteratorPipe != NULL))
        {
            /* Check for IRP with an expired interval */
            iteratorPipe->intervalCounter --;
            if(iteratorPipe->intervalCounter == 0)
            {
                /* Interval for the pipe is done. Check
                 * if the pipe has a an IRP and add it
                 * to the frame IRP table */

                iteratorPipe->intervalCounter = iteratorPipe->bInterval;
                if(iteratorPipe->irpQueueHead != NULL)
                {
                    /* Schedule the IRP */
                    hDriver->frameIRPTable[hDriver->frameIRPTableIndex] = 
                        iteratorPipe->irpQueueHead;
                    hDriver->frameIRPTableIndex ++;

                }
               
            }
            iteratorPipe = iteratorPipe->next;
        }

    }

    /**********************************************
     * Now check for Bulk Transfers. Bulk transfers
     * follow periodic transfers.
     **********************************************/

    transferGroup = &hDriver->transferGroup[USB_TRANSFER_TYPE_BULK];
    if(transferGroup->pipe != NULL)
    {
        /* This means that there are one or more 
         * bulk pipes. */

        iteratorPipe = transferGroup->pipe;
        while((hDriver->frameIRPTableIndex 
                    < DRV_USB_HOST_IRP_PER_FRAME_NUMBER) &&
                (iteratorPipe != NULL))
        {

            if(iteratorPipe->irpQueueHead != NULL)
            {
                /* Schedule the IRP */
                hDriver->frameIRPTable[hDriver->frameIRPTableIndex] = 
                    iteratorPipe->irpQueueHead;
                hDriver->frameIRPTableIndex ++;

            }

            iteratorPipe = iteratorPipe->next;
        }
    }

    /**************************************************
     * At this point we will have a set of transactions
     * that should be processed in this frame.
     **************************************************/
	/* Reset the frame table index */
	hDriver->frameIRPTableIndex = 0;

    /* Reset the frame IRP table index and start the
     * first transfer in the table */
    if((hDriver->frameIRPTable[0] != NULL))
    {
        /* We are either continuing a control transfer
         * or sending a new one. From this point on
         * the transfers in the frame are triggred from
         * the transaction complete interrupt  */

        _DRV_USB_HOST_ControlXferProcess(hDriver, hDriver->frameIRPTable[0], 
                0/* Not relevant in this call*/,
                0 /* Not relevant in this call*/);

    }
    else
    {
        /* Since we dont have to process control transfers, set
         * the frame table index to 1 to start processing of non
         * control transfers. We stop the moment we come across
         * the first non control transfer to be processed */
        
        if(hDriver->frameIRPTable[1] != NULL)
        {
            /* Then schedule this IRP on the bus */
            _DRV_USB_HOST_NonControlIRPProcess (hDriver, 
                    hDriver->frameIRPTable[1],0,0);

            hDriver->frameIRPTableIndex = 1;
        }

    }
}

void _DRV_USB_HOST_Tasks_ISR(DRV_USB_OBJ * hDriver)
{
    void                   *eventData = NULL;
    bool                    bDoEventCallBack;
	bool					tokenWasSent;
    uint8_t                 lastEndpoint;
    unsigned int            transactionSize;
    USB_MODULE_ID           usbID;
    DRV_USB_EVENT           eventType = 0;
    USB_INTERRUPTS          usbInterrupts;
    USB_INTERRUPTS          enabledUSBInterrupts;
    DRV_USB_BDT_ENTRY       * pBDTEntry;

    USB_INTERRUPTS          clearUSBInterrupts = 0;
    DRV_USB_CLIENT_OBJ      * hClient;
    USB_PING_PONG_STATE     lastPingPong;
    USB_BUFFER_DIRECTION    lastDirection;
    DRV_USB_TRANSACTION_RESULT  transactionResult;

    USB_HOST_IRP_LOCAL      * irp;

    usbID = hDriver->usbID;
    bDoEventCallBack = false;

    usbInterrupts = PLIB_USB_InterruptFlagAllGet(usbID);

    enabledUSBInterrupts = PLIB_USB_InterrupEnableGet(usbID);

    if ((PLIB_USB_OTG_InterruptFlagGet(usbID, USB_OTG_INT_ONE_MS_TIMEOUT))
            && (PLIB_USB_OTG_InterruptIsEnabled(usbID, USB_OTG_INT_ONE_MS_TIMEOUT)))
    {

        /* This means that a 1 msec event has occured */

        PLIB_USB_OTG_InterruptFlagClear(usbID, USB_OTG_INT_ONE_MS_TIMEOUT);

        if(hDriver->timerCount > 0)
        {
            hDriver->timerCount --;
        }
        else
        {
            /* This means that a desired time out is complete*/
            PLIB_USB_OTG_InterruptDisable(usbID, USB_OTG_INT_ONE_MS_TIMEOUT);
        }

    }

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
        _DRV_USB_HOST_TransferSchedule(hDriver);
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

        hDriver->deviceSpeed = USB_SPEED_FULL;
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

            hDriver->deviceSpeed = USB_SPEED_LOW;
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
        hDriver->pBDT[0].word[0] = 0x0;
        hDriver->pBDT[0].word[1] = 0x0;
        hDriver->pBDT[1].word[0] = 0x0;
        hDriver->pBDT[1].word[1] = 0x0;
        hDriver->pBDT[2].word[0] = 0x0;
        hDriver->pBDT[2].word[1] = 0x0;
        hDriver->pBDT[3].word[0] = 0x0;
        hDriver->pBDT[3].word[1] = 0x0;

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

        if(hDriver->pDrvUSBClientObj != (DRV_USB_CLIENT_OBJ *)DRV_HANDLE_INVALID)
        {
            /* Check if the client has a valid
             * callback */

            hClient = hDriver->pDrvUSBClientObj;

            if(hClient->pEventCallBack != NULL)
            {
                /************************************************
                 * Client call back. Since the HCD can have only
                 * one client, we will not loop through a table
                 * of client. Client 0 is the host layer.
                 ************************************************/

                hClient->pEventCallBack(hClient->hClientArg,
                        eventType,  eventData); 
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

        pBDTEntry = hDriver->pBDT + (2 * lastDirection) + lastPingPong;
        transactionResult = (pBDTEntry->byte[0] & 0x3C) >> 2;
        transactionSize = pBDTEntry->shortWord[1];

        if(lastDirection == USB_BUFFER_RX)
        {
            /* Update the RX ping pong buffer indicator */
            hDriver->ep0RxPingPong ^= 0x1;
        }
        else
        {
            /* Update the even ping pong buffer indicator */
            hDriver->ep0TxPingPong ^= 0x1;
        }

		tokenWasSent = false;
        irp = hDriver->frameIRPTable[hDriver->frameIRPTableIndex];

        if(hDriver->frameIRPTableIndex == 0)
        {
            /* We were processing a control transfer */

            tokenWasSent = _DRV_USB_HOST_ControlXferProcess(hDriver,
                                irp, transactionResult, transactionSize);

            if(tokenWasSent == false)
            {
                /* Check if we can send the next IRP as there is nothing
                 * more of control transfers to be done in this frame */

                irp = hDriver->frameIRPTable[1];

                if(irp != NULL)
                {
                    _DRV_USB_HOST_NonControlIRPProcess(hDriver, irp, 0, 0);
                    hDriver->frameIRPTableIndex ++;
                }
            }
        }
        else if(hDriver->frameIRPTableIndex > 0)
        {
            /* We are processing non control transfers. Update the 
             * current non control transfer in progress. */
            
            _DRV_USB_HOST_NonControlIRPProcess(hDriver, irp,
                                 transactionResult, transactionSize);

            if((hDriver->frameIRPTableIndex + 1) 
                    < DRV_USB_HOST_IRP_PER_FRAME_NUMBER)
            {
                irp = hDriver->frameIRPTable[hDriver->frameIRPTableIndex + 1];
                if(irp != NULL)
                {
                    _DRV_USB_HOST_NonControlIRPProcess(hDriver, irp, 0, 0);
                    hDriver->frameIRPTableIndex ++;
                }
            }
        }

    }

}

void DRV_USB_HOST_BusResetControl(DRV_HANDLE client, bool control)
{
    DRV_USB_CLIENT_OBJ * hClient = (DRV_USB_CLIENT_OBJ *)client; 
    DRV_USB_OBJ * hDriver = (DRV_USB_OBJ *)hClient->hDriver;

    /* If control is true, then enable reset */

    if(control == true)
    {
        PLIB_USB_ResetSignalEnable(hDriver->usbID);

        /* After the rest is activated, we can
         * find out if the J state is active, 
         * which means that the D- line is pulled
         * up and hence this a low speed device.
         */

        if(PLIB_USB_JStateIsActive(hDriver->usbID))
        {
            hDriver->deviceSpeed = USB_SPEED_LOW;
        }
        else
        {
            /* Device is full speed */
            hDriver->deviceSpeed = USB_SPEED_FULL;
        }
    }
    else
    {
        PLIB_USB_ResetSignalDisable(hDriver->usbID);
    }
}

void DRV_USB_HOST_StartOfFrameControl(DRV_HANDLE client, bool control)
{
    DRV_USB_CLIENT_OBJ * hClient = (DRV_USB_CLIENT_OBJ *)client; 
    DRV_USB_OBJ * hDriver = (DRV_USB_OBJ *)hClient->hDriver;

    /* If control is true, then enable SOF and
     * the SOF interrupt */

    if(control == true)
    {
        PLIB_USB_SOFEnable(hDriver->usbID);
        PLIB_USB_InterruptEnable(hDriver->usbID, USB_INT_SOF);
    }
    else
    {
        PLIB_USB_SOFDisable(hDriver->usbID);
        PLIB_USB_InterruptDisable(hDriver->usbID, USB_INT_SOF);
    }

}

USB_SPEED DRV_USB_HOST_DeviceCurrentSpeedGet(DRV_HANDLE client)
{
    DRV_USB_CLIENT_OBJ * hClient = (DRV_USB_CLIENT_OBJ *)client; 
    DRV_USB_OBJ * hDriver = (DRV_USB_OBJ *)hClient->hDriver;

    return(hDriver->deviceSpeed);
} 

void DRV_USB_HOST_OperationEnable(DRV_HANDLE hClient, bool enable)
{
    /* This function enables the USB Module
     * and the USB interrupt, if enable is true */

    DRV_USB_OBJ * hDriver = ((DRV_USB_CLIENT_OBJ *)hClient)->hDriver;
    USB_MODULE_ID usbID = hDriver->usbID;

    if(true == enable)
    {
        /* Interrupt flag cleared on the safer side */

        SYS_INT_SourceStatusClear(hDriver->interruptSource);

        if(DRV_USB_INTERRUPT_MODE)
        {
            /* If interrupt mode is set then enable interrupts 
             * Interrupt priority will be set by the application
             */

            SYS_INT_SourceEnable(hDriver->interruptSource);
        }

        /* Enable the USB module */
        PLIB_USB_Enable(usbID);
    }

}
