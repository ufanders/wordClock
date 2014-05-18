/*******************************************************************************
  USB Device Driver Implementation of device mode operation routines

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usb_device.c

  Summary:
    USB Device Driver Dynamic Implementation of device mode operation routines

  Description:
    The USB device driver provides a simple interface to manage the USB
    modules on Microchip microcontrollers.  This file Implements the 
    interface routines for the USB driver when operating in device mode.

    While building the driver from source, ALWAYS use this file in the build if
    device mode operation is required.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip  Technology  Inc.   All  rights  reserved.

Microchip licenses to  you  the  right  to  use,  modify,  copy  and  distribute
Software only when embedded on a Microchip  microcontroller  or  digital  signal
controller  that  is  integrated  into  your  product  or  third  party  product
(pursuant to the  sublicense  terms  in  the  accompanying  license  agreement).

You should refer  to  the  license  agreement  accompanying  this  Software  for
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
#include "driver/usb/usbhs/src/drv_usb_local.h"
#include "system/debug/sys_debug.h"

static uint8_t * usb1baseAddress = (uint8_t *)&USBCSR0;

uint16_t _DRV_USB_ProcessIRPFIFO
(
    DRV_USB_OBJ * hDriver,
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObj,
    uint8_t endpoint,
    uint8_t direction,
    USB_DEVICE_IRP_LOCAL * irp
)
{
    /* This function only loads or unloads the FIFO
     * It does not set the Tx Packet Ready or clear'
     * the RX Packet Ready bit. The function
     * returns the amount of data that was written
     * or read from the FIFO */

    unsigned int count;
    unsigned int offset;
    USBHS_MODULE_ID usbID;
    uint8_t * data;

    data = (uint8_t *)irp->data;
    usbID = hDriver->usbID;

    if(direction == USB_DATA_DIRECTION_DEVICE_TO_HOST)
    {
        /* This means data has to move from device
         * to host. We should write to the FIFO */

        count = (irp->nPendingBytes < endpointObj->maxPacketSize)
                ? irp->nPendingBytes : endpointObj->maxPacketSize;

        offset = (irp->size - irp->nPendingBytes);

        PLIB_USBHS_DeviceEPFIFOLoad(usbID, endpoint, &data[offset], count);

        irp->nPendingBytes -= count;
    }
    else
    {
        /* This means data is moving from host to device */

        count = PLIB_USBHS_DeviceEPFIFOUnload(usbID, endpoint,
                &data[irp->nPendingBytes]);

        irp->nPendingBytes += count;
    }

    return(count);
}

void _DRV_USB_DEVICE_Initialize(DRV_USB_OBJ * drvObj, SYS_MODULE_INDEX index)
{
    USBHS_MODULE_ID usbID;

    usbID = drvObj->usbID;

    /* Disable all endpoint interrupts 
       Enable the reset, the SOF, resume and suspend interrupt */

    PLIB_USBHS_InterruptEnableSet(usbID, 0xF, 0x0, 0x0);

    /* In device mode endpoint 0 FIFO size is always 64.
     * So any FIFO allocation should start from 64. The
     * actual value stored in this variable is 64/8 */

    drvObj->consumedFIFOSize = 8;
}

void DRV_USB_DEVICE_AddressSet(DRV_HANDLE handle, uint8_t address)
{
    /* Set the address */
    DRV_USB_CLIENT_OBJ * client = (DRV_USB_CLIENT_OBJ *)handle;
    DRV_USB_OBJ * hDriver;
    USBHS_MODULE_ID usbID;

    if((DRV_HANDLE_INVALID == handle) || (client == NULL))
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return;
    }

    /* Set the device address */
    hDriver = (DRV_USB_OBJ *)client->hDriver;
    usbID = hDriver->usbID;

    PLIB_USBHS_DeviceAddressSet(usbID, address);
}

USB_SPEED DRV_USB_DEVICE_CurrentSpeedGet(DRV_HANDLE handle)
{
    DRV_USB_CLIENT_OBJ * client = (DRV_USB_CLIENT_OBJ *) handle;
    DRV_USB_OBJ * hDriver;

    if((DRV_HANDLE_INVALID == handle) || (client == NULL))
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
    }

    hDriver = (DRV_USB_OBJ *)client->hDriver;

    /* The current speed in contained in the
     * device speed member of the driver object */

    return(hDriver->deviceSpeed);
}

void DRV_USB_DEVICE_Attach(DRV_HANDLE handle)
{
    DRV_USB_OBJ * hDriver;
    USBHS_MODULE_ID usbID;
    DRV_USB_CLIENT_OBJ * client = (DRV_USB_CLIENT_OBJ *)handle;

    if((DRV_HANDLE_INVALID == handle) || (client == NULL))
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return;
    }

    hDriver = client->hDriver;
    usbID = hDriver->usbID;

    /* Attach at the speed at which the driver
     * was initialized */

    PLIB_USBHS_DeviceAttach(usbID, hDriver->operationSpeed);
        
    return; 
}

void DRV_USB_DEVICE_Detach(DRV_HANDLE handle)
{
    DRV_USB_OBJ * hDriver;
    USBHS_MODULE_ID usbID;
    DRV_USB_CLIENT_OBJ * client = (DRV_USB_CLIENT_OBJ *)handle;

    if((DRV_HANDLE_INVALID == handle) || (client == NULL))
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return;
    }

    hDriver = client->hDriver;
    usbID = hDriver->usbID;

    /* Detach the device */

    PLIB_USBHS_DeviceDetach(usbID);
}


void _DRV_USB_DEVICE_EndpointObjectEnable
(
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObject,
    uint16_t endpointSize,
    USB_TRANSFER_TYPE endpointType
   
)
{
    /* This is a helper function */
   
    endpointObject->irpQueue        = NULL;
    endpointObject->maxPacketSize   = endpointSize;
    endpointObject->endpointType    = endpointType;
    endpointObject->endpointState  |= DRV_USB_DEVICE_ENDPOINT_STATE_ENABLED;
    
}    

uint16_t DRV_USB_DEVICE_SOFNumberGet(DRV_HANDLE handle)
{
    DRV_USB_CLIENT_OBJ * client;
    DRV_USB_OBJ * hDriver;
    USBHS_MODULE_ID usbID;
    
    if(DRV_HANDLE_INVALID == handle)
    {
        SYS_DEBUG(0, "USB Driver: Invalid Handle");
        return 0;
    }

    client = (DRV_USB_CLIENT_OBJ *)handle;
    hDriver = client->hDriver;
    usbID = hDriver->usbID;

    return(0);

    
    
}

USB_ERROR DRV_USB_DEVICE_EndpointEnable
(
    DRV_HANDLE handle, 
    USB_ENDPOINT endpointAndDirection, 
    USB_TRANSFER_TYPE endpointType,
    uint16_t endpointSize
)
{

    int     direction;
    int     fifoSize;
    bool    handshake;
    uint8_t endpoint;
    unsigned int shiftWord;
    bool    interruptWasEnabled = false;

    DRV_USB_OBJ     * hDriver;
    DRV_USB_CLIENT_OBJ * client;
    USBHS_MODULE_ID   usbID;
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObject;

    /* Enable the endpoint */

    endpoint     = endpointAndDirection & 0xF;
    direction    = ((endpointAndDirection & 0x80) != 0);
    
    if(endpoint >= DRV_USB_ENDPOINTS_NUMBER)
    {
        SYS_ASSERT(false,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    client = (DRV_USB_CLIENT_OBJ *)handle;

    if((DRV_HANDLE_INVALID == handle) || (client == NULL))
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return(USB_ERROR_PARAMETER_INVALID);
    }

    hDriver  = (DRV_USB_OBJ *)client->hDriver;
    usbID    = hDriver->usbID;

    /* Get the pointer to the endpoint object */
    
    endpointObject = (hDriver->endpointTable + (2 * endpoint) + 0);

    if(endpoint == 0)
    {

        /* There are two endpoint objects for a control 
         * endpoint */
        
        _DRV_USB_DEVICE_EndpointObjectEnable(endpointObject, 
                                    endpointSize, endpointType);
   
        endpointObject ++;

        _DRV_USB_DEVICE_EndpointObjectEnable(endpointObject, 
                                    endpointSize, endpointType);

        PLIB_USBHS_TxInterruptEnable(usbID, USBHS_TXRXINT_EP0);

        /* EP0 does not require any configuration 
         * It is ready to receive packets when the device is attached */
    }
    else
    {
        /* Non EP0 endpoints require configuration */
        
        handshake = (endpointType == USB_TRANSFER_TYPE_ISOCHRONOUS)
                        ? false : true;

        /* Update the endpoint database */
        endpointObject += direction;

        _DRV_USB_DEVICE_EndpointObjectEnable(endpointObject, 
                                      endpointSize, endpointType);

        if(!hDriver->isInInterruptContext)
        {
            //OSAL: Get mutex
            interruptWasEnabled = _DRV_USB_InterruptSourceDisable(hDriver->interruptSource);
        }

        /* The following code maps the endpoint size to the value 
         * that should be loaded in the FIFOSZ register */

        fifoSize = 0;
        shiftWord = endpointSize;

        while((shiftWord & 0x1) != 1)
        {
            shiftWord = (shiftWord >> 1);
            fifoSize ++;
        }
        fifoSize -= 3;

        if(direction == USB_DATA_DIRECTION_DEVICE_TO_HOST)
        {
            /* Set up the maxpacket size, fifo start address
             * fifosize and enable the interrupt. CLear the data
             * toggle. */
            PLIB_USBHS_DeviceTxEndpointConfigure(usbID,
                    endpoint, endpointSize, hDriver->consumedFIFOSize, fifoSize,
                    endpointType);
        }
        else
        {
            /* Setup the RX endpoint. Disable NYET handshakes. */
            PLIB_USBHS_DeviceRxEndpointConfigure(usbID,
                    endpoint, endpointSize, hDriver->consumedFIFOSize, fifoSize,
                    endpointType);
        }

        hDriver->consumedFIFOSize += (endpointSize/8);

        if(!hDriver->isInInterruptContext)
        {
            if(interruptWasEnabled)
            {
                _DRV_USB_InterruptSourceEnable(hDriver->interruptSource);
            }
            //OSAL: Release Mutex
        }
    }
    return(USB_ERROR_NONE);
}

USB_ERROR DRV_USB_DEVICE_EndpointDisable
(
    DRV_HANDLE handle, 
    USB_ENDPOINT endpointAndDirection
)
{
    /* This routine disables the specified endpoint.
     * It does not check if there is any ongoing 
     * communication on the bus through the endpoint
     */

    uint8_t endpoint;
    int     direction, iEntry;
    DRV_USB_OBJ * hDriver;
    DRV_USB_CLIENT_OBJ * client;
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObject;
    USBHS_MODULE_ID usbID;
    bool interruptWasEnabled = false;
   
    client = (DRV_USB_CLIENT_OBJ *)handle;

    /* Check if the handle is valid */
    if((DRV_HANDLE_INVALID == handle) || (client == NULL))
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return USB_ERROR_PARAMETER_INVALID;
    }

    /* Get the pointer to associated endpoint object table */

    hDriver         = ((DRV_USB_CLIENT_OBJ *)handle)->hDriver;
    usbID           = hDriver->usbID;
    endpointObject  = hDriver->endpointTable;
    endpoint        = endpointAndDirection & 0xF;
    direction       = ((endpointAndDirection & 0x80) != 0);

    if((endpointAndDirection != _DRV_USB_DEVICE_ENDPOINT_ALL)
            && (endpoint >= DRV_USB_ENDPOINTS_NUMBER))
    {
        /* The endpoint number must be either _DRV_USB_DEVICE_ENDPOINT_ALL
         * or a valid endpoint */

        SYS_ASSERT(false,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    if(!hDriver->isInInterruptContext)
    {
        // OSAL: Get Mutex
        interruptWasEnabled = _DRV_USB_InterruptSourceDisable(hDriver->interruptSource);
    }

    /* If the endpointAndDirection is _DRV_USB_DEVICE_ENDPOINT_ALL
     * then this means that the DRV_USB_DEVICE_EndpointDisableAll()
     * function was called */

    if(endpointAndDirection == _DRV_USB_DEVICE_ENDPOINT_ALL)
    {
        /* Reset the consumed FIFO size */
        hDriver->consumedFIFOSize = 8; 

        /* Disable all the interrupts */

        PLIB_USBHS_TxInterruptDisable(usbID, USBHS_TXRXINT_ALL);
        PLIB_USBHS_RxInterruptDisable(usbID, USBHS_TXRXINT_ALL);

        
        for(iEntry = 0; iEntry < DRV_USB_ENDPOINTS_NUMBER; iEntry ++)
        {
            /* Update the endpoint database */
            
            endpointObject->endpointState  &= ~DRV_USB_DEVICE_ENDPOINT_STATE_ENABLED;
            endpointObject ++;
            endpointObject->endpointState  &= ~DRV_USB_DEVICE_ENDPOINT_STATE_ENABLED;
            endpointObject ++;

        }

        /* Restore the index register */
        if(!hDriver->isInInterruptContext)
        {
            if(interruptWasEnabled)
            {
                /* IF the interrupt was enabled when entering the routine
                 * re-enable it now */
                _DRV_USB_InterruptSourceEnable(hDriver->interruptSource);
                //OSAL: Release the mutex
            }
        }
        return (USB_ERROR_NONE);
    }

    /* Setup the endpointObj to point to the correct
     * endpoint object Interrupt is still disabled */

    endpointObject += (endpoint * 2);

    if(endpointObject->endpointType == USB_TRANSFER_TYPE_CONTROL)
    {
        /* Disable a control endpoint and update the
         * endpoint database. */

        endpointObject->endpointState  &= ~DRV_USB_DEVICE_ENDPOINT_STATE_ENABLED;
        endpointObject += 1;
        endpointObject->endpointState  &= ~DRV_USB_DEVICE_ENDPOINT_STATE_ENABLED;
    }
    else
    {
        /* Disable a specific endpoint direction for non
         * control endpoints. Also reduce the hDriver consumedFIFOSize
         * member by the size of the disable endpoint. */
	
        endpoint += direction;
        endpointObject->endpointState  &= ~DRV_USB_DEVICE_ENDPOINT_STATE_ENABLED;
        hDriver->consumedFIFOSize -= (endpointObject->maxPacketSize/8);

        if(direction == USB_DATA_DIRECTION_DEVICE_TO_HOST)
        {
            /* Disable the TX interrupt */
            PLIB_USBHS_TxInterruptDisable(usbID, (1 << endpoint));
        }
        else
        {
            /* Disable the RX interrupt */
            PLIB_USBHS_TxInterruptDisable(usbID, (1 << endpoint));
        }
    }

    /* Release interrupts and mutex */
    if(!hDriver->isInInterruptContext)
    {
        if(interruptWasEnabled)
        {
            /* IF the interrupt was enabled when entering the routine
             * re-enable it now */
            _DRV_USB_InterruptSourceEnable(hDriver->interruptSource);
        }
        //OSAL: Mutex Release
    }
	return(USB_ERROR_NONE);
}

bool DRV_USB_DEVICE_EndpointIsEnabled(DRV_HANDLE handle,
                                        USB_ENDPOINT endpointAndDirection)
{
    /* Return the state of the endpoint */

    DRV_USB_OBJ * hDriver;
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObj;
    DRV_USB_CLIENT_OBJ * client = (DRV_USB_CLIENT_OBJ *)handle;

    uint8_t endpoint = endpointAndDirection & 0xF;
    int direction = ((endpointAndDirection & 0x80) != 0);

    if((DRV_HANDLE_INVALID == handle) || (client == NULL))
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return false;
    }

    hDriver = ((DRV_USB_CLIENT_OBJ *)client)->hDriver;
    endpointObj = hDriver->endpointTable + 
            (2 * endpoint) + direction;
    
    if((endpointObj->endpointState & 
                DRV_USB_DEVICE_ENDPOINT_STATE_ENABLED) != 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool DRV_USB_DEVICE_EndpointIsStalled(DRV_HANDLE handle,
                                        USB_ENDPOINT endpointAndDirection)
{
    /* Return the state of the endpoint */

    DRV_USB_OBJ * hDriver;
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObj;
    uint8_t endpoint = endpointAndDirection & 0xF;
    int direction = ((endpointAndDirection & 0x80) != 0);
    DRV_USB_CLIENT_OBJ * client = (DRV_USB_CLIENT_OBJ *)handle;

    if((DRV_HANDLE_INVALID == handle) || (client == NULL))
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return false;
    }

    hDriver = ((DRV_USB_CLIENT_OBJ *)client)->hDriver;
    endpointObj = hDriver->endpointTable + 
            (2 * endpoint) + direction;
    
    if((endpointObj->endpointState & 
                DRV_USB_DEVICE_ENDPOINT_STATE_STALLED) != 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

USB_ERROR DRV_USB_DEVICE_IRPSubmit
(
    DRV_HANDLE handle,
    USB_ENDPOINT endpointAndDirection, 
    USB_DEVICE_IRP * inputIRP
)
{
    uint8_t endpoint;
    int direction;
    USBHS_MODULE_ID usbID;
    DRV_USB_CLIENT_OBJ * client = (DRV_USB_CLIENT_OBJ *)handle;
    DRV_USB_OBJ * hDriver;
    USB_DEVICE_IRP_LOCAL * irp = (USB_DEVICE_IRP_LOCAL *)inputIRP;
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObj;
    bool interruptWasEnabled = false;
    unsigned int count;
    uint8_t ep0Status;
    uint8_t rxEPStatus;

    int remainder;

    /* Check for a valid client */

    if((DRV_HANDLE_INVALID == handle) || (client == NULL))
    {
        SYS_ASSERT(false, "Invalid handle");
        return USB_ERROR_PARAMETER_INVALID;
    }

    if(irp->status > USB_DEVICE_IRP_STATUS_SETUP)
    {
        /* This means that the IRP is in use */
        SYS_ASSERT(false, "Device IRP is already in use");
        return(USB_ERROR_DEVICE_IRP_IN_USE);
    }

    /* Check for a valid endpoint */
    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0);

    if(endpoint >= DRV_USB_ENDPOINTS_NUMBER)
    {
        SYS_ASSERT(false, "Endpoint is not provisioned for");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    /* Get the driver object, the module ID and the
     * endpoint and direction specific BDT entry and
     * the endpoint object. */

    hDriver     = (DRV_USB_OBJ*)client->hDriver;
    usbID       = hDriver->usbID;
    endpointObj = hDriver->endpointTable + (2 * endpoint) + direction;

    if(!(endpointObj->endpointState & DRV_USB_DEVICE_ENDPOINT_STATE_ENABLED))
    {
        /* This means the endpoint is not enabled */
        return(USB_ERROR_ENDPOINT_NOT_CONFIGURED);
    }

    /* Check the size of the IRP. If the endpoint receives
     * data from the host, then IRP size must be 
     * multiple of maxPacketSize. If the send ZLP flag is 
     * set, then size must be multiple of endpoint size. */

    remainder = irp->size % endpointObj->maxPacketSize;

    if(remainder == 0)
    {
        /* The IRP size is either 0 or a 
         * exact multiple of maxPacketSize */

        if(USB_DATA_DIRECTION_DEVICE_TO_HOST == direction)
        {
            if(((irp->flags & USB_DEVICE_IRP_FLAG_DATA_COMPLETE) 
                    == USB_DEVICE_IRP_FLAG_DATA_COMPLETE) &&
                (irp->size != 0))
            {
                /* This means a ZLP should be sent after the data
                 * is sent */

                irp->flags |= USB_DEVICE_IRP_FLAG_SEND_ZLP;
            }
        }
    }

    /* Now we check if the interrupt context is active. If so
     * the we dont need to get a mutex or disable interrupts.
     * If this were being done in non interrupt context, we, then
     * we would disable the interrupt. In which case we would
     * get the mutex and then disable the interrupt */

    if(hDriver->isInInterruptContext == false)
    {
        //OSAL: Get mutex
        interruptWasEnabled = _DRV_USB_InterruptSourceDisable(hDriver->interruptSource);
    }

    irp->next = NULL;

    /* If the data is moving from device to host
     * then pending bytes is data remaining to be
     * sent to the host. If the data is moving from
     * host to device, nPendingBytes tracks the
     * amount of data received so far */

    if(USB_DATA_DIRECTION_DEVICE_TO_HOST == direction)
    {
        irp->nPendingBytes = irp->size;
    }
    else
    {
        irp->nPendingBytes = 0;
    }

    /* Mark the IRP status as pending */
    irp->status = USB_DEVICE_IRP_STATUS_PENDING;

    /* Get the last object in the endpoint object
     * IRP Queue */
    if(endpointObj->irpQueue == NULL)
    {
        /* Queue is empty */
        endpointObj->irpQueue = irp;
        irp->previous = NULL;

        /* Because this is the first IRP in the queue
         * then we we must arm the endpoint entry in
         * the BDT. */

        irp->status = USB_DEVICE_IRP_STATUS_IN_PROGRESS;
        
        if(endpoint == 0)
        {
            /* Endpoint zero IRPs require special handling */

            if(direction == USB_DATA_DIRECTION_DEVICE_TO_HOST)
            {
                /* Because we allow the client to provide three/two different
                 * IRPs for the three/two stages of the control transfer. we
                 * need to check what stage we are in when we get an IRP. The
                 * endpoint0State is manipulated by both the main thread and
                 * the interrupt thread. */

                /* Check if the endpoint is waiting for a IRP */
                if(hDriver->endpoint0State == DRV_USB_DEVICE_EP0_STATE_WAITING_FOR_TX_DATA_IRP_FROM_CLIENT)
                {
                    /* In this state, the USB module is waiting for transmit
                     * data from the client.  This would be a Control Read
                     * transfer. And client has provided it */

                    count = _DRV_USB_ProcessIRPFIFO(hDriver, endpointObj, (uint8_t)0,
                            (uint8_t)USB_DATA_DIRECTION_DEVICE_TO_HOST, irp);
                    hDriver->endpoint0State = DRV_USB_DEVICE_EP0_STATE_TX_DATA_STAGE_IN_PROGRESS;
                    if(count < endpointObj->maxPacketSize)
                    {
                        /* This means that this is the last packet
                         * in the control read transfer. Set the
                         * transmit ready and the data end bit */
                        PLIB_USBHS_EP0TxPktRdyDataEnd(usbID);
                    }
                    else
                    {
                        /* This is a multi data stage transfer. Dont
                         * set the data end bit. Only set the
                         * transmit ready bit. */
                        PLIB_USBHS_EP0TxPktRdy(usbID);
                    }
                }
                else if(hDriver->endpoint0State == DRV_USB_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_IRP_FROM_CLIENT)
                {
                    /* In this state, the module is waiting for the client to
                     * complete the handshake stage of the control write
                     * transfer. The client needs to let the module know that
                     * it has received the data. */

                    hDriver->endpoint0State = DRV_USB_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_STAGE_FROM_HOST;
                    PLIB_USBHS_EP0RxPktRdyServicedDataEnd(usbID);

                }
            }
            else
            {
                /* Receive direction handling here */

                if(hDriver->endpoint0State == DRV_USB_DEVICE_EP0_STATE_WAITING_FOR_RX_DATA_IRP_FROM_CLIENT)
                {
                    ep0Status = PLIB_USBHS_EP0StatusGet(usbID);
                    
                    /* Check if data is already present in the FIFO */
                    if(ep0Status & USBHS_EP0_RXPKTRDY)
                    {
                        /* This means we already have data in the FIFO
                         * We will need to check if the IRP is completed */

                        count = _DRV_USB_ProcessIRPFIFO(hDriver, endpointObj, 
                                0, USB_DATA_DIRECTION_HOST_TO_DEVICE, irp);

                        /* Check if the irp is completed */
                        if((count < endpointObj->maxPacketSize)
                                || (irp->nPendingBytes >= irp->size))
                        {
                            /* This means we received a short packet
                             * or the amount of data received is the amount
                             * that was expected */

                            if(irp->size < irp->nPendingBytes)
                            {
                                irp->status = USB_DEVICE_IRP_STATUS_COMPLETED_SHORT;
                            }
                            else
                            {
                                irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                            }

                            irp->size = irp->nPendingBytes;

                            endpointObj->irpQueue = irp->next;
                            if(irp->callback)
                            {
                                irp->callback((USB_DEVICE_IRP *)irp);
                            }

                            /* We can now move to the handshake stage. Check
                             * if a TX IRP is available */

                            if((endpointObj + 1)->irpQueue != NULL)
                            {
                                hDriver->endpoint0State = DRV_USB_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_STAGE_FROM_HOST;

                                /* Data stage is complete. Clear the Rx Pkt Ready
                                 * and set the data end bit. */
                                PLIB_USBHS_EP0RxPktRdyServicedDataEnd(usbID);
                            }
                            else
                            {
                                hDriver->endpoint0State = DRV_USB_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_IRP_FROM_CLIENT;
                            }
                        }
                        else
                        {
                            /* We are expecting more data. Let the module know
                             * that we have completed reading the data */
                            PLIB_USBHS_EP0RxPktRdyServiced(usbID);
                        }
                    }
                    else
                    {
                        /* There is no data in the FIFO yet */
                        hDriver->endpoint0State = DRV_USB_DEVICE_EP0_STATE_WAITING_FOR_RX_DATA_STAGE_FROM_HOST;
                    }
                }
                else if(hDriver->endpoint0State == DRV_USB_DEVICE_EP0_STATE_WAITING_FOR_RX_STATUS_IRP_FROM_CLIENT)
                {
                    /* This IRP will receive the status stage ZLP for 
                     * a control read transfer. The ZLP in this case 
                     * comes from the host. */

                    hDriver->endpoint0State = DRV_USB_DEVICE_EP0_STATE_WAITING_FOR_RX_STATUS_STAGE_FROM_HOST;
                    PLIB_USBHS_EP0DataEndSet(usbID);
                }
            }
        }
        else
        {
            /* Non zero endpoint irp */


            if(direction == USB_DATA_DIRECTION_DEVICE_TO_HOST)
            {
                /* Data is moving from device to host. So this
                 * a TX IRP. We dont have to check for the TXxPktRdy
                 * bit because the fact that the queue is empty
                 * means there is nothing in the queue */

               _DRV_USB_ProcessIRPFIFO(hDriver, endpointObj, endpoint, direction, irp);
              
               /* Set the TX packet Ready bit */

               PLIB_USBHS_DeviceTxEndpointPacketReady(usbID, endpoint);

               /* The rest of the IRP processing takes place in ISR */

            }
            else
            {
                /* We have an IRP to process. Lets check if the RxPktRdy
                 * bit is already set on the endpoint. If so we can start
                 * unloading the fifo */

                rxEPStatus = PLIB_USBHS_RxEPStatusGet(usbID, endpoint);

                if(rxEPStatus & USBHS_RXEP_PKTRDY)
                {
                    /* This means the FIFO already contains data */

                    count = _DRV_USB_ProcessIRPFIFO(hDriver, endpointObj, endpoint, direction, irp);
                    
                    /* Clear the Rx Packet Ready Bit */
                    
                    PLIB_USBHS_RxEPStatusClear(usbID, endpoint, USBHS_RXEP_PKTRDY);

                    /* Check if the irp is completed */

                    if((count < endpointObj->maxPacketSize)
                            || (irp->nPendingBytes >= irp->size))
                    {
                        /* This means we received a short packet
                         * or the amount of data received is the amount
                         * that was expected */

                        if(irp->size < irp->nPendingBytes)
                        {
                            irp->status = USB_DEVICE_IRP_STATUS_COMPLETED_SHORT;
                        }
                        else
                        {
                            irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                        }

                        irp->size = irp->nPendingBytes;

                        endpointObj->irpQueue = irp->next;
                        if(irp->callback)
                        {
                            irp->callback((USB_DEVICE_IRP *)irp);
                        }
                    }
                }
            }/* End of non zero RX IRP submit */
        }/* End of non zero IRP submit */
    }
    else
    {
        /* This means we should surf the linked list
         * to get to the last entry . */
        USB_DEVICE_IRP_LOCAL * iterator;
        iterator = endpointObj->irpQueue;
        while(iterator->next != NULL)
        {
            iterator = iterator->next;
        }
        iterator->next = irp;
        irp->previous = iterator;
        irp->status = USB_DEVICE_IRP_STATUS_PENDING;
    }

    if(hDriver->isInInterruptContext == false)
    {
        if(interruptWasEnabled)
        {
            /* Enable the interrupt only if it was enabled */
            _DRV_USB_InterruptSourceEnable(hDriver->interruptSource);
        }
        //OSAL: Return mutex
    }
    return(USB_ERROR_NONE);

}

void _DRV_USB_DEVICE_IRPQueueFlush
(
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObject
)
{
    USB_DEVICE_IRP_LOCAL * iterator;
    
    /* Check if any IRPs are assigned on this endpoint and 
     * abort them */

    if(endpointObject->irpQueue != NULL)
    {
        /* Cancel the IRP and deallocate driver IRP 
         * objects */

        iterator = endpointObject->irpQueue;

        while(iterator != NULL)
        {
            iterator->status = USB_DEVICE_IRP_STATUS_ABORTED;
            if(iterator->callback != NULL)
            {
                iterator->callback((USB_DEVICE_IRP *)iterator);
            }
            iterator = iterator->next;
        }
    }
    
    /* Set the head pointer to NULL */
    endpointObject->irpQueue = NULL;
}

USB_ERROR DRV_USB_DEVICE_IRPCancelAll
(
    DRV_HANDLE handle,
    USB_ENDPOINT endpointAndDirection
)
{
    int     direction;
    uint8_t endpoint;
    DRV_USB_OBJ     * hDriver;
    USBHS_MODULE_ID   usbID;
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObject;
    DRV_USB_CLIENT_OBJ * client;
    bool interruptWasEnabled = false;

    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0);

    if(endpoint >= DRV_USB_ENDPOINTS_NUMBER)
    {
        SYS_ASSERT(false,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    client = (DRV_USB_CLIENT_OBJ *)handle;

    if((DRV_HANDLE_INVALID == handle) || (client == NULL))
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return USB_ERROR_PARAMETER_INVALID;
    }

    hDriver      = (DRV_USB_OBJ*)client->hDriver;
    usbID        = hDriver->usbID;

    /* Get the endpoint object */
    endpointObject = hDriver->endpointTable + (2 * endpoint)
        + direction;

    if(!hDriver->isInInterruptContext)
    {
        //OSAL : Get mutex
        /* Disable the interrupt */
        interruptWasEnabled = _DRV_USB_InterruptSourceDisable(hDriver->interruptSource);
    }

    /* Flush the endpoint */
    _DRV_USB_DEVICE_IRPQueueFlush(endpointObject);

    /* We need to flush the TXFIFO if any of them are
     * loaded. Endpoint zero does not have a direction.
     * For other endpoints we check if the direction
     * is transmit */

    if((endpoint == 0) || 
            ((endpoint != 0) && (direction == USB_DATA_DIRECTION_DEVICE_TO_HOST)))
    {
        if(endpoint == 0)
        {
 //           PLIB_USBHS_Endpoint0FIFOFlush(usbID);
        }  
        else
        {
            PLIB_USBHS_EndpointTxFIFOFlush(usbID, endpoint);
        }
    }

    if(!hDriver->isInInterruptContext)
    {
        if(interruptWasEnabled)
        {
            _DRV_USB_InterruptSourceEnable(hDriver->interruptSource);
        }
        // OSAL: Release Mutex
    }
    return(USB_ERROR_NONE);
}

USB_ERROR DRV_USB_DEVICE_EndpointStall(DRV_HANDLE handle,
                                    USB_ENDPOINT endpointAndDirection)
{
    int     direction;
    bool    interruptWasEnabled = false;
    uint8_t endpoint;
    DRV_USB_OBJ     * hDriver;
    DRV_USB_CLIENT_OBJ * client;
    USBHS_MODULE_ID   usbID;
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObject;

    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0); 

    if(endpoint >= DRV_USB_ENDPOINTS_NUMBER)
    {
        SYS_ASSERT(false,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    client = (DRV_USB_CLIENT_OBJ *)handle;

    if((DRV_HANDLE_INVALID == handle) || (client == NULL))
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return USB_ERROR_PARAMETER_INVALID;
    }

    hDriver      = (DRV_USB_OBJ*)client->hDriver;
    usbID        = hDriver->usbID;

    /* Get the endpoint object */
    endpointObject = hDriver->endpointTable + (2 * endpoint);
    
    if(!hDriver->isInInterruptContext)
    {
        //OSAL: Get Mutex
        /* Disable the interrupt */
        interruptWasEnabled = _DRV_USB_InterruptSourceDisable(hDriver->interruptSource);
    }

    if(endpoint == 0)
    {
        PLIB_USBHS_EP0StallEnable(usbID);

        /* While stalling endpoint 0 we stall both directions */
        endpointObject->endpointState |= DRV_USB_DEVICE_ENDPOINT_STATE_STALLED;
        _DRV_USB_DEVICE_IRPQueueFlush(endpointObject);
        endpointObject ++;
        endpointObject->endpointState |= DRV_USB_DEVICE_ENDPOINT_STATE_STALLED;
        _DRV_USB_DEVICE_IRPQueueFlush(endpointObject);
    }
    else
    {
        /* Stalling a non zero endpoint */
        if(direction == USB_DATA_DIRECTION_DEVICE_TO_HOST)
        {
            /* Stall the TX direction */
            PLIB_USBHS_DeviceTxEndpointStallEnable(usbID, endpoint);
        }
        else
        {
            /* Stall the RX direction */
            PLIB_USBHS_DeviceTxEndpointStallEnable(usbID, endpoint);
        }

        endpointObject += direction;
        endpointObject->endpointState |= DRV_USB_DEVICE_ENDPOINT_STATE_STALLED;
        _DRV_USB_DEVICE_IRPQueueFlush(endpointObject);
    }

    if(!hDriver->isInInterruptContext)
    {
        if(interruptWasEnabled)
        {
            /* Enable the interrupt */
            _DRV_USB_InterruptSourceEnable(hDriver->interruptSource);
        }
        //OSAL: Release Mutex
    }

    return(USB_ERROR_NONE);
}

USB_ERROR DRV_USB_DEVICE_EndpointStallClear(DRV_HANDLE handle,
                                    USB_ENDPOINT endpointAndDirection)
{
    int     direction;
    bool    interruptWasEnabled = false;
    uint8_t endpoint;
    DRV_USB_OBJ     * hDriver;
    DRV_USB_CLIENT_OBJ * client;
    USBHS_MODULE_ID   usbID;
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObject;

    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0);

    if(endpoint >= DRV_USB_ENDPOINTS_NUMBER)
    {
        SYS_ASSERT(false,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    client = (DRV_USB_CLIENT_OBJ *)handle;

    if((DRV_HANDLE_INVALID == handle) || (client == NULL))
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return USB_ERROR_PARAMETER_INVALID;
    }

    hDriver = (DRV_USB_OBJ *)client->hDriver;
    usbID = hDriver->usbID;

    if(endpoint == 0)
    {
        /* For endpoint zero, the STALL is automatically
         * cleared when the STALL is sent to the host */
        return(USB_ERROR_NONE);
    }

    if(!hDriver->isInInterruptContext)
    {
        //OSAL: Mutex Get
        interruptWasEnabled = _DRV_USB_InterruptSourceDisable(hDriver->interruptSource);
    }

    if(direction == USB_DATA_DIRECTION_DEVICE_TO_HOST)
    {
        /* Clear the stall on the TX direction
         * and the stall sent bit */

        PLIB_USBHS_DeviceTxEndpointStallDisable(usbID, endpoint);
    }
    else
    {
        /* Clear the stall on the RX direction 
         * and the stall sent bit */

        PLIB_USBHS_DeviceRxEndpointStallDisable(usbID, endpoint);
    }

    /* Get the endpoint object */
    endpointObject = hDriver->endpointTable + (2 * endpoint)
                        + direction;
    endpointObject->endpointState &= ~DRV_USB_DEVICE_ENDPOINT_STATE_STALLED;

    if(!hDriver->isInInterruptContext)
    {
        if(interruptWasEnabled)
        {
            _DRV_USB_InterruptSourceEnable(hDriver->interruptSource);
        }
        // OSAL: Return Mutex
    }

    return(USB_ERROR_NONE);
}

volatile uint32_t setupCounter = 0;
void _DRV_USB_DEVICE_Tasks_ISR(DRV_USB_OBJ * hDriver)
{
    uint8_t usbInterrupts;
    uint16_t endpointMask;
    uint8_t  iEndpoint;
    uint8_t  ep0Status;
    uint8_t  txEPStatus;
    uint8_t  rxEPStatus;
    uint8_t endpointTXInterrupts;
    uint8_t endpointRXInterrupts;
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObjReceive;
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObjTransmit;
    USBHS_MODULE_ID usbID;
    DRV_USB_CLIENT_OBJ * client;
    USB_DEVICE_IRP_LOCAL * irp;
    uint16_t endpoint0DataStageSize;
    unsigned int endpoint0DataStageDirection;
    
    unsigned int count;
    uint8_t * data;
    
    if(((DRV_HANDLE)(hDriver->pDrvUSBClientObj) == DRV_HANDLE_INVALID)
            || (hDriver->pDrvUSBClientObj == NULL))
    {
        /* We need a valid client */
        SYS_ASSERT(false, "Driver does not have a client");
        return;
    }

    client = (DRV_USB_CLIENT_OBJ *)hDriver->pDrvUSBClientObj;

    if(client->pEventCallBack == NULL)
    {
        /* We need a valid event handler */
        SYS_ASSERT(false, "Driver needs a event handler");
        return;
    }

    usbID = hDriver->usbID;

    /* Reading the interrupt flags will cause the
     * flags to get cleared */

    usbInterrupts = PLIB_USBHS_GenInterruptFlagsGet(usbID);

    if(usbInterrupts & USBHS_GENINT_RESET)
    {
        /* This means that RESET signalling was detected 
         * on the bus. This means the packet that we should
         * get on the bus for EP0 should be a setup packet */

        hDriver->endpoint0State = 
            DRV_USB_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST;


        /* Default speed is full speed */
        hDriver->deviceSpeed = USB_SPEED_FULL;

        if(hDriver->operationSpeed == USB_SPEED_HIGH)
        {
            /* If high speed operation was specified
             and we were able to connect at high speed
             then update the speed.
             */
            if(PLIB_USBHS_HighSpeedIsConnected(usbID))
            {
                /* This means high speed was successful*/
                hDriver->deviceSpeed = USB_SPEED_HIGH;
            }
        }

        client->pEventCallBack(client->hClientArg,
                DRV_USB_EVENT_RESET_DETECT,  NULL);
    }

    if(usbInterrupts & USBHS_GENINT_SUSPEND)
    {
        /* This measn that the bus was SUSPENDED. */

        client->pEventCallBack(client->hClientArg,
                DRV_USB_EVENT_IDLE_DETECT, NULL);
    }

    if(usbInterrupts & USBHS_GENINT_RESUME)
    {
        /* This means that RESUME was detected on the bus. */

        client->pEventCallBack(client->hClientArg,
                DRV_USB_EVENT_RESUME_DETECT, NULL);
    }        

    if(usbInterrupts & USBHS_GENINT_SOF)
    {
        /* This measn that there was a SOF. */

        client->pEventCallBack(client->hClientArg,
                DRV_USB_EVENT_SOF_DETECT, NULL);
    }        

    /* Read the endpoint interrupts */

    endpointTXInterrupts = PLIB_USBHS_TxInterruptFlagsGet(usbID);
    endpointRXInterrupts = PLIB_USBHS_RxInterruptFlagsGet(usbID);

    /* Setup the endpoint objects */

    endpointObjReceive = hDriver->endpointTable; 
    endpointObjTransmit = hDriver->endpointTable + 1;
    
    if(endpointTXInterrupts & USBHS_TXRXINT_EP0)
    {
       /* This means this is EP0 interrupt */

        *(usb1baseAddress + INDEX) = 0;
        ep0Status = PLIB_USBHS_EP0StatusGet(usbID);

        if(ep0Status & USBHS_EP0_SENTSTALL)
        {
            /* This means a stall was sent. Reset the 
             * endpoint 0 state.  Clear the sent stall bit */
            hDriver->endpoint0State = DRV_USB_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST;
            PLIB_USBHS_EP0SentStallClear(usbID);
            endpointObjReceive->endpointState &= ~DRV_USB_DEVICE_ENDPOINT_STATE_STALLED;
            endpointObjTransmit->endpointState &= ~DRV_USB_DEVICE_ENDPOINT_STATE_STALLED;
        }

        else if(ep0Status & USBHS_EP0_SETUPEND)
        {
            /* This means the current control transfer
             * end prematurely. We dont need to end any 
             * transfers. The device layer will manage
             * the premature transfer end. */
            PLIB_USBHS_EP0SetupEndServiced(usbID);
            if(!hDriver->isZeroLengthControlTransfer)
            {
                /* The setup end bit gets set for zero length transfers. If
                 * this was not a zero length transfer and the setup end bit
                 * is set, then we have genuine control transfer abort. */
                hDriver->endpoint0State = DRV_USB_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST;
            }
        }

        /* We have to check for these conditions before we check for the
         endpoint 0 receive packet because these conditions could potentially
         clear up the IRP queue. These condtions do not have any explicit
         status bit associated with them, but the module will generate an
         interrupt anyways*/

        if(hDriver->endpoint0State == DRV_USB_DEVICE_EP0_STATE_TX_DATA_STAGE_IN_PROGRESS)
        {

            irp = endpointObjTransmit->irpQueue;

            if(irp != NULL)
            {
                /* Check if the Transmit IRP is done */
                if(irp->nPendingBytes == 0)
                {
                    /* This means we got done with the IRP */
                    endpointObjTransmit->irpQueue = irp->next;
                    irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;

                    if(irp->callback != NULL)
                    {
                        irp->callback((USB_DEVICE_IRP *)irp);
                    }

                    /* We should check if the status stage IRP is available */

                    if(endpointObjReceive->irpQueue != NULL)
                    {
                        /* This is the status IRP */
                       
                        irp = endpointObjReceive->irpQueue;
                        irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                        irp->size = 0;
                        endpointObjReceive->irpQueue = irp->next;
                        if(irp->callback != NULL)
                        {
                            irp->callback((USB_DEVICE_IRP *)irp);
                        }
                    }
                    
                    else
                    {
                        SYS_ASSERT(false, "Was expecting status IRP");
                    }

                    /* We will just move on to the next control transfer */
                    hDriver->endpoint0State = DRV_USB_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST;
                }
                else
                {
                    /* There is more data in the IRP. 
                     * Continue to process it */

                   count =  _DRV_USB_ProcessIRPFIFO(hDriver,endpointObjTransmit,
                            0, USB_DATA_DIRECTION_DEVICE_TO_HOST, irp);

                   if(count < endpointObjTransmit->maxPacketSize)
                   {
                        /* This means that this is the last packet
                         * in the control read transfer. Set the
                         * transmit ready and the data end bit */
                        *(usb1baseAddress + CSR0L) = 0xA;
                        
                    }
                    else
                    {
                        /* This is a multi data stage transfer. Dont
                         * set the data end bit. Only set the
                         * transmit ready bit. */
                        *(usb1baseAddress + CSR0L) = 0x2;
                    }
                }
            }
        } /* End of checking for TX Data Stage processing */

        else if(hDriver->endpoint0State == DRV_USB_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_STAGE_FROM_HOST)
        {
            /* This means the TX status stage has completed */

            irp = endpointObjTransmit->irpQueue;
            if(irp != NULL)
            {
                irp->size = 0;
                irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                endpointObjTransmit->irpQueue = irp->next;
                if(irp->callback != NULL)
                {
                    irp->callback((USB_DEVICE_IRP *)irp);
                }
                hDriver->endpoint0State = DRV_USB_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST;
            }
        }
    
        else if(ep0Status & USBHS_EP0_RXPKTRDY)
        {
            /* This means we have received data on the
             * endpoint */

            if (hDriver->endpoint0State ==
                    DRV_USB_DEVICE_EP0_STATE_EXPECTING_SETUP_FROM_HOST)
            {
                /* This means we were expecting a SETUP packet
                 * and we got one. Get the endpoint 0 RX endpoint
                 * object and get the IRP at top of the queue */

                irp = endpointObjReceive->irpQueue;

                /* If we have received a setup packet, this means that if a 
                 * stall was scheduled, then it has been sent. So we can 
                 * clear the stall status of the endpoint object */
                endpointObjReceive->endpointState &= ~DRV_USB_DEVICE_ENDPOINT_STATE_STALLED;
                endpointObjTransmit->endpointState &= ~DRV_USB_DEVICE_ENDPOINT_STATE_STALLED;

                if(irp == NULL)
                {
                    /* This means the device layer did not
                     * schedule a IRP to receive a SETUP
                     * packet! */

                    SYS_ASSERT(false,"No IRP to receive setup packet!");
                    Nop();
                    Nop();
                }
                else
                {

                    /* We have an IRP for the setup packet. Transfer
                     * the data from the FIFO to the IRP */

                    if(setupCounter == 10)
                    {
                        Nop();
                    }

                    data = (uint8_t *)irp->data;
                    *((unsigned int *)(data)) = *((unsigned int *)(usb1baseAddress + FIFO0));
                    *((unsigned int *)(data + 4)) = *((unsigned int *)(usb1baseAddress + FIFO0));
                              
                    /* Remember the size and direction of the data stage */
                    endpoint0DataStageSize = *((unsigned short int *)(data + 6));
                    endpoint0DataStageDirection = ((data[0] & 0x80) != 0);

                    if(endpoint0DataStageSize > 0)
                    {
                        /* Set the Serviced RX packet Ready bit which
                         * will then clear the RX packet ready bit. This
                         * let the hardware know that FIFO has been read */
                        hDriver->isZeroLengthControlTransfer = false;
                        
                    }

                    /* Indicate that this is a setup IRP */
                    irp->status = USB_DEVICE_IRP_STATUS_SETUP;
                    irp->size = 8;
   					
					/* Update the endpoint IRP queue. If
                     * irp->next is NULL, then this means
                     * at this point we dont have anything
                     * in the queue */

                    endpointObjReceive->irpQueue = irp->next;
                    if(irp->callback != NULL)
                    {
                        irp->callback((USB_DEVICE_IRP *)irp);
                    }

                    /* Check the size and the direction of the
                     * control transfer */

                    if(endpoint0DataStageSize == 0)
                    {
                        if(!(endpointObjReceive->endpointState & DRV_USB_DEVICE_ENDPOINT_STATE_STALLED))
                        {
                        /* Set a flag to remember that this is a
                         * zero length control transfer */
                         hDriver->isZeroLengthControlTransfer = true;

                        /* This means data was moving from
                         * host to device. The status stage ZLP
                         * will come from device. Service the
                         * TX endpoint on device */

                        if(endpointObjTransmit->irpQueue != NULL)
                        {
                            /* That means we have a IRP to be processed */
                            endpointObjTransmit->irpQueue->status = USB_DEVICE_IRP_STATUS_IN_PROGRESS;
                            hDriver->endpoint0State = DRV_USB_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_STAGE_FROM_HOST;

                            /* Set the data end bit and the serviced packet
                             * ready bit */
                            *(usb1baseAddress + CSR0L) = 0x48;
                           
                        }
                        else
                        {
                            /* If here means we have to wait for an IRP submit
                             * to complete this transfer. The RxPktRdyServiced
                             * and the data end bit is set in the IRP submit
                             * function */
                            hDriver->endpoint0State = DRV_USB_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_IRP_FROM_CLIENT;
                            }
                        }
                        
                    }
                    else
                    {
                        if(!(endpointObjReceive->endpointState & DRV_USB_DEVICE_ENDPOINT_STATE_STALLED))
                        {
                        /* This means that there is a data stage
                        */
                        if(endpoint0DataStageDirection == USB_DATA_DIRECTION_DEVICE_TO_HOST)
                        {
                            /* Clear the Rx packet ready bit */
                            *(usb1baseAddress + CSR0L) = 0x40;

                            /* This means the data stage is moving from device
                             * to host which means a transmit endpoint is used
                             * */

                            if(endpointObjTransmit->irpQueue != NULL)
                            {
                                /* This means we have a TX IRP to process */

                                hDriver->endpoint0State = DRV_USB_DEVICE_EP0_STATE_TX_DATA_STAGE_IN_PROGRESS;
                                endpointObjTransmit->irpQueue->status = USB_DEVICE_IRP_STATUS_IN_PROGRESS;

                                count = _DRV_USB_ProcessIRPFIFO(hDriver,endpointObjTransmit,
                                        0, USB_DATA_DIRECTION_DEVICE_TO_HOST,
                                        endpointObjTransmit->irpQueue);

                                if(count < endpointObjTransmit->maxPacketSize)
                                {
                                    /* This means that this is the last packet
                                     * in the control read transfer. Set the
                                     * transmit ready and the data end bit */
                                    *(usb1baseAddress + CSR0L) = 0xA;
                                }
                                else
                                {
                                    /* This is a multi data stage transfer. Dont
                                     * set the data end bit. Only set the
                                     * transmit ready bit. */
                                    *(usb1baseAddress + CSR0L) = 0x2;
                                }
                            }
                            else
                            {
                                /* This means we dont have a IRP ready */
                                hDriver->endpoint0State =
                                        DRV_USB_DEVICE_EP0_STATE_WAITING_FOR_TX_DATA_IRP_FROM_CLIENT;
                            }
                        }
                        else
                        {
                            /* This means the data stage is moving from host
                             * to device which means a receive endpoint is used
                             * */

                            *(usb1baseAddress + CSR0L) = 0x40;
                            if(endpointObjReceive->irpQueue != NULL)
                            {
                                /* This means we have a RX IRP to process */

                                hDriver->endpoint0State = DRV_USB_DEVICE_EP0_STATE_WAITING_FOR_RX_DATA_STAGE_FROM_HOST;
                                endpointObjReceive->irpQueue->status = USB_DEVICE_IRP_STATUS_IN_PROGRESS;
                            }
                            else
                            {
                                /* This means we dont have a IRP ready */
                                hDriver->endpoint0State = DRV_USB_DEVICE_EP0_STATE_WAITING_FOR_RX_DATA_IRP_FROM_CLIENT;
                                }
                            }
                        }
                    } /* End of data stage available else condition */
                } /* End of valid setup irp avaialble handling */
            } /* End of expecting Setup Packet Handling state */

            else if(hDriver->endpoint0State == DRV_USB_DEVICE_EP0_STATE_WAITING_FOR_RX_DATA_STAGE_FROM_HOST)
            {
                /* This means were expecting some data from the host
                 * and have received it */

                irp = endpointObjReceive->irpQueue;
                if(irp != NULL)
                {
                    /* This means we have a irp to process */
                    count = _DRV_USB_ProcessIRPFIFO(hDriver, endpointObjReceive,
                            0, USB_DATA_DIRECTION_HOST_TO_DEVICE, irp);

                    /* Clear the Rx Packet Ready Bit */
                    *(usb1baseAddress + CSR0L) = 0x40;

                    /* Check if the irp is completed */
                   if((count < endpointObjReceive->maxPacketSize)
                           || (irp->nPendingBytes >= irp->size))
                   {
                       /* This means we received a short packet
                        * or the amount of data received is the amount
                        * that was expected */

                       if(irp->size < irp->nPendingBytes)
                       {
                           irp->status = USB_DEVICE_IRP_STATUS_COMPLETED_SHORT;
                       }
                       else
                       {
                           irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                       }

                       irp->size = irp->nPendingBytes;
                   
                       endpointObjReceive->irpQueue = irp->next;
                       if(irp->callback)
                       {
                           irp->callback((USB_DEVICE_IRP *)irp);
                       }

                       /* We can now move to the handshake stage. Check
                        * if a TX IRP is available */

                       if(endpointObjTransmit->irpQueue != NULL)
                       {
                           hDriver->endpoint0State = DRV_USB_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_STAGE_FROM_HOST;
                           /* Set the data end bit */
                           *(usb1baseAddress + CSR0L) = 0x8;
                       }
                       else
                       {
                           hDriver->endpoint0State = DRV_USB_DEVICE_EP0_STATE_WAITING_FOR_TX_STATUS_IRP_FROM_CLIENT;
                       }
                   }
                }
            }
        }
    }

    /* Here we check if any other endpoint interrupts
     * have occurred. Start the endpoint mask with
     * endpoint 1 as the endpoint 0 is already serviced
     * */

    endpointMask = 0x2;
    for(iEndpoint = 1; iEndpoint < DRV_USB_ENDPOINTS_NUMBER; iEndpoint ++)
    {
        endpointObjReceive = hDriver->endpointTable + (2 * iEndpoint);
        endpointObjTransmit = hDriver->endpointTable + (2 * iEndpoint) + 1;

        if(endpointTXInterrupts & endpointMask)
        {
            /* Check if the endpoint is enabled */

            if(endpointObjTransmit->endpointState & DRV_USB_DEVICE_ENDPOINT_STATE_ENABLED)
            {
                /* This means the endpoint is enabled. We got a TX interrupt 
                 * because a IRP was scheduled */

                txEPStatus = PLIB_USBHS_TxEPStatusGet(usbID, iEndpoint);

                if(txEPStatus & USBHS_TXEP_SENTSTALL)
                {
                    /* This means a stall was sent. Clear this 
                     * the sent stall bit
                     */

                    PLIB_USBHS_TxEPStatusClear(usbID, iEndpoint, USBHS_TXEP_SENTSTALL);
                }

                if(endpointObjTransmit->irpQueue != NULL)
                {
                    /* We have a valid IRP at the head
                     * of the queue */

                    irp = endpointObjTransmit->irpQueue;
                    if(irp->nPendingBytes == 0)
                    {
                        /* We dont have any more bytes to send in this 
                         * IRP. Check if the ZLP flag is set else end 
                         * the IRP */

                        if((irp->flags & USB_DEVICE_IRP_FLAG_SEND_ZLP) != 0)
                        {
                            /* This means we have to send a ZLP. Clear the
                             * flag */

                            irp->flags &= ~USB_DEVICE_IRP_FLAG_SEND_ZLP;
                            PLIB_USBHS_TxEPStatusClear(usbID, iEndpoint, USBHS_TXEP_UNDERRUN);
                            PLIB_USBHS_DeviceTxEndpointPacketReady(usbID, iEndpoint);
                        }
                        else
                        {
                            /* This means this IRP is processed */
                            irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                            endpointObjTransmit->irpQueue = irp->next;
                            if(irp->callback != NULL)
                            {
                                /* Invoke the IRP */
                                irp->callback((USB_DEVICE_IRP *) irp);
                            }

                            if(endpointObjTransmit->irpQueue != NULL)
                            {
                                /* This means we have one more IRP in the
                                 * queue */

                                _DRV_USB_ProcessIRPFIFO(hDriver, 
                                        endpointObjTransmit, iEndpoint, 
                                        USB_DATA_DIRECTION_DEVICE_TO_HOST,
                                        endpointObjTransmit->irpQueue);

                                /* Set the TX packet Ready bit. Clear the
                                 * FIFO underun bit if it is set. */
                                PLIB_USBHS_TxEPStatusClear(usbID, iEndpoint, USBHS_TXEP_UNDERRUN);
                                PLIB_USBHS_DeviceTxEndpointPacketReady(usbID, iEndpoint);

                            }
                        }
                    }/* End of nPending bytes equal to zero */
                    else
                    {
                        /* This means are more pending bytes in this
                         * IRP */

                        _DRV_USB_ProcessIRPFIFO(hDriver, 
                                endpointObjTransmit, iEndpoint, 
                                USB_DATA_DIRECTION_DEVICE_TO_HOST,
                                irp);

                        /* Set the TX packet Ready bit. Clear the
                         * FIFO underun bit if it is set. */
                        PLIB_USBHS_TxEPStatusClear(usbID, iEndpoint, USBHS_TXEP_UNDERRUN);
                        PLIB_USBHS_DeviceTxEndpointPacketReady(usbID, iEndpoint);

                    }

                }
            }
        }/* End of TX Endpoint Interrupt Handling */

        if(endpointRXInterrupts & endpointMask)
        {
            /* Check if the endpoint is enabled */

            if(endpointObjReceive->endpointState & DRV_USB_DEVICE_ENDPOINT_STATE_ENABLED)
            {
                /* This means the endpoint is enabled */

                rxEPStatus = PLIB_USBHS_RxEPStatusGet(usbID, iEndpoint);
                if(rxEPStatus & USBHS_RXEP_SENTSTALL)
                {
                    /* This means a stall was sent. Clear this 
                     * the sent stall bit
                     */

                    PLIB_USBHS_RxEPStatusClear(usbID, iEndpoint, USBHS_RXEP_SENTSTALL);
                }

                if(endpointObjReceive->irpQueue != NULL)
                {
                    /* This means we have a IRP to process */

                    if(rxEPStatus & USBHS_RXEP_PKTRDY)
                    {
                        /* This means the FIFO already contains data */

                        irp = endpointObjReceive->irpQueue;

                        count = _DRV_USB_ProcessIRPFIFO(hDriver, 
                                endpointObjReceive, iEndpoint,
                                USB_DATA_DIRECTION_HOST_TO_DEVICE, irp);

                        /* Clear the Rx Packet Ready Bit */

                        PLIB_USBHS_RxEPStatusClear(usbID, iEndpoint, USBHS_RXEP_PKTRDY);

                        /* Check if the irp is completed */

                        if((count < endpointObjReceive->maxPacketSize)
                                || (irp->nPendingBytes >= irp->size))
                        {
                            /* This means we received a short packet
                             * or the amount of data received is the amount
                             * that was expected */

                            if(irp->size > irp->nPendingBytes)
                            {
                                /* This means that we received less data
                                 * than what was expected. */
                                irp->status = USB_DEVICE_IRP_STATUS_COMPLETED_SHORT;
                            }
                            else
                            {
                                irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                            }

                            irp->size = irp->nPendingBytes;

                            endpointObjReceive->irpQueue = irp->next;
                            
                            if(irp->callback)
                            {
                                irp->callback((USB_DEVICE_IRP *)irp);
                            }
                        }
                    }
                }
            } /* Check of RX endpoint state enabled */
        }/* End of receive interrupt */
        
        endpointMask = endpointMask << 1;
    
    }/* End of endpoint interrupt checking for loop */
}

