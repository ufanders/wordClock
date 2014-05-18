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

#include "driver/usb/drv_usb.h"
#include "driver/usb/src/drv_usb_local.h"

#define DRIVER __attribute__((section("Driver")))

/**************************************
 * The driver isntance group.
 *************************************/
extern DRV_USB_GROUP gDrvUSBGroup;

/**************************************
 * Create some helper macros.
 **************************************/
#define _DRV_USB_OBJ                                &gDrvUSBGroup.gDrvUSBObj
#define _DRV_USB_CLIENT_OBJ                         &gDrvUSBGroup.gDrvUSBClientObj
#define _DRV_USB_ENDPOINT_OBJ(endpoint, direction)  &gDrvUSBGroup.gDrvUSBEndpoints[(2 * endpoint) + direction]


void DRIVER _DRV_USB_DEVICE_Initialize(DRV_USB_OBJ * hDriver, SYS_MODULE_INDEX index)
{
		
    /* Enable the desired mode */

    PLIB_USB_OperatingModeSelect(DRV_USB_PERIPHERAL_ID, USB_OPMODE_DEVICE);

    /* Enable or disable the interrupts */
    PLIB_USB_AllInterruptEnable(DRV_USB_PERIPHERAL_ID, USB_INT_ALL,
                                    USB_ERR_INT_ALL, ~USB_OTG_INT_ALL);

}

void DRIVER _DRV_USB_MAKE_NAME(DEVICE_AddressSet)(DRV_HANDLE handle, uint8_t address)
{
    /* Set the address */

    if(DRV_HANDLE_INVALID == handle)
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return;
    }

    PLIB_USB_DeviceAddressSet( DRV_USB_PERIPHERAL_ID, address );

}

USB_SPEED DRIVER _DRV_USB_MAKE_NAME(DEVICE_CurrentSpeedGet)(DRV_HANDLE handle)
{
    /* Return the speed */
    if(DRV_HANDLE_INVALID == handle)
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
    }

    return USB_SPEED_FULL;
}

void DRIVER _DRV_USB_MAKE_NAME(DEVICE_Attach)(DRV_HANDLE handle)
{
    /* Attach the device */
    if(DRV_HANDLE_INVALID == handle)
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return;
    }

    PLIB_USB_Enable(DRV_USB_PERIPHERAL_ID);
}

void DRIVER _DRV_USB_MAKE_NAME(DEVICE_Detach)(DRV_HANDLE handle)
{
    /* Detach the device */
    if(DRV_HANDLE_INVALID == handle)
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return;
    }

    PLIB_USB_Disable(DRV_USB_PERIPHERAL_ID);

}

void _DRV_USB_DEVICE_EndpointObjectEnable
(
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObject,
    uint16_t endpointSize,
    USB_TRANSFER_TYPE endpointType,
    USB_BUFFER_DATA01 dataToggle
)
{
    /* This is a helper function */
    endpointObject->nextDataToggle  = USB_BUFFER_DATA0;
    endpointObject->irpQueue        = NULL;
    endpointObject->maxPacketSize   = endpointSize;
    endpointObject->nextPingPong    = USB_BUFFER_EVEN;
    endpointObject->endpointType    = endpointType;
    endpointObject->endpointState  |= DRV_USB_DEVICE_ENDPOINT_STATE_ENABLED;
    
}    

USB_ERROR DRIVER _DRV_USB_MAKE_NAME(DEVICE_EndpointEnable)
(
    DRV_HANDLE handle, 
    USB_ENDPOINT endpointAndDirection, 
    USB_TRANSFER_TYPE endpointType,
    uint16_t endpointSize
)
{
    /* This function can be called from
     * from the USB ISR. Because an endpoint
     * can ne owned by one client only, we
     * dont need mutex protection in this 
     * function */

    int     direction;
    int     iEntry;
    bool    handshake;
    uint8_t endpoint;

    DRV_USB_OBJ     * hDriver;
    DRV_USB_BDT_ENTRY * pBDT;
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObject;

    /* Enable the endpoint */

    endpoint     = endpointAndDirection & 0xF;
    direction    = ((endpointAndDirection & 0x80) != 0);
    
    if(endpoint >= DRV_USB_ENDPOINTS_NUMBER)
    {
        SYS_ASSERT(false,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    if(DRV_HANDLE_INVALID == handle)
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return USB_ERROR_PARAMETER_INVALID;
    }

    hDriver = _DRV_USB_OBJ;

    /* The BDT table has four entries per endpoint
     * The following statement points pBDT to the
     * first endpoint specific entry */

    pBDT         = hDriver->pBDT + (endpoint * 4);
    
    /* Get the pointer to the endpoint object */
    
    endpointObject = _DRV_USB_ENDPOINT_OBJ(endpoint, 0);

    if(endpointType == USB_TRANSFER_TYPE_CONTROL)
    {
        /* For a control endpoint enable both
         * directions. Clear up the BDT entries. */

        uint32_t * pBDT32bit = (uint32_t *)pBDT;

        for(iEntry = 0; iEntry < 7; iEntry ++)
        {
            /* A full duplex endpoint has 4
             * entries, 2 for each direction */

            *(pBDT32bit + iEntry) = 0;
        }
        
        /* The following function enables both directions */
           
        PLIB_USB_EPnAttributesSet( DRV_USB_PERIPHERAL_ID, endpoint, 0, true, 0 );
             
        /* Update the endpoint database for both directions */
        
        _DRV_USB_DEVICE_EndpointObjectEnable(endpointObject, 
                                    endpointSize, endpointType, USB_BUFFER_DATA0);

        /* This is when data moves from device to host. For
         * control end points the Data toggle always starts
         * with DATA1.
         */
   
        endpointObject ++;

        _DRV_USB_DEVICE_EndpointObjectEnable(endpointObject, 
                                    endpointSize, endpointType, USB_BUFFER_DATA1);

    }
    else
    {
        /* Clear up the even odd entries for this
         * endpoint direction in the BDT. Each entry
         * has 2 32 bit entries */

        pBDT += (2 * direction);

        /* Clear up the even entry */
        pBDT->word[0] = 0;
        pBDT->word[1] = 0;
        pBDT ++;

        /* Clear up the odd entry */
        pBDT->word[0] = 0;
        pBDT->word[1] = 0;
        
        handshake = (endpointType == USB_TRANSFER_TYPE_ISOCHRONOUS)
                        ? false : true;
                        
        PLIB_USB_EPnAttributesSet(DRV_USB_PERIPHERAL_ID, endpoint, direction, false, handshake);

        /* Update the endpoint database */
        endpointObject += direction;

        _DRV_USB_DEVICE_EndpointObjectEnable(endpointObject, 
                                      endpointSize, endpointType, USB_BUFFER_DATA0);
    }
    
    return(USB_ERROR_NONE);
}

USB_ERROR DRIVER _DRV_USB_MAKE_NAME(DEVICE_EndpointDisable)
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
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObject;

    /* Check if the handle is valid */

    if(DRV_HANDLE_INVALID == handle)
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return USB_ERROR_PARAMETER_INVALID;
    }

    /* If the endpointAndDirection is _DRV_USB_DEVICE_ENDPOINT_ALL
     * then this means that the DRV_USB_DEVICE_EndpointDisableAll()
     * function was called */

    if(endpointAndDirection == _DRV_USB_DEVICE_ENDPOINT_ALL)
    {
        /* Get the pointer to associated endpoint object table */
        endpointObject = _DRV_USB_ENDPOINT_OBJ(0, 0);
        
        for(iEntry = 0; iEntry < DRV_USB_ENDPOINTS_NUMBER; iEntry ++)
        {
            PLIB_USB_EPnAttributesClear(DRV_USB_PERIPHERAL_ID, iEntry);

            /* Update the endpoint database */

            endpointObject->endpointState  &= ~DRV_USB_DEVICE_ENDPOINT_STATE_ENABLED;
            endpointObject ++;
            endpointObject->endpointState  &= ~DRV_USB_DEVICE_ENDPOINT_STATE_ENABLED;
            endpointObject ++;

        }
        
        return (USB_ERROR_NONE);
    }

    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0);

    if(endpoint >= DRV_USB_ENDPOINTS_NUMBER)
    {
        SYS_ASSERT(false,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    /* Setup the endpointObj to point to the correct
     * endpoint object */

    endpointObject = _DRV_USB_ENDPOINT_OBJ(endpoint, direction);
    if(endpointObject->endpointType == USB_TRANSFER_TYPE_CONTROL)
    {
        /* Disable a control endpoint and update the
         * endpoint database. */

        endpointObject = _DRV_USB_ENDPOINT_OBJ(0, 0);
        PLIB_USB_EPnAttributesClear(DRV_USB_PERIPHERAL_ID, endpoint);
        endpointObject->endpointState  &= ~DRV_USB_DEVICE_ENDPOINT_STATE_ENABLED;
        endpointObject += 1;
        endpointObject->endpointState  &= ~DRV_USB_DEVICE_ENDPOINT_STATE_ENABLED;
    }
    else
    {
        /* Disable a specific endpoint direction for non
         * control endpoints */
	
        endpointObject = _DRV_USB_ENDPOINT_OBJ(endpoint, direction);
        PLIB_USB_EPnDirectionDisable(DRV_USB_PERIPHERAL_ID, endpoint, direction);
        endpointObject->endpointState  &= ~DRV_USB_DEVICE_ENDPOINT_STATE_ENABLED;
    }

	return(USB_ERROR_NONE);
}

bool DRIVER _DRV_USB_MAKE_NAME(DEVICE_EndpointIsEnabled)(DRV_HANDLE client,
                                        USB_ENDPOINT endpointAndDirection)
{
    /* Return the state of the endpoint */

    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObj;

    uint8_t endpoint = endpointAndDirection & 0xF;
    int direction = ((endpointAndDirection & 0x80) != 0);

    if(DRV_HANDLE_INVALID == client)
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return false;
    }

    endpointObj = _DRV_USB_ENDPOINT_OBJ(endpoint, direction);
    
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

bool DRIVER _DRV_USB_MAKE_NAME(DEVICE_EndpointIsStalled)(DRV_HANDLE client,
                                        USB_ENDPOINT endpointAndDirection)
{
    /* Return the state of the endpoint */

    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObj;
    uint8_t endpoint = endpointAndDirection & 0xF;
    int direction = ((endpointAndDirection & 0x80) != 0);

    if(DRV_HANDLE_INVALID == client)
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return false;
    }

    endpointObj = _DRV_USB_ENDPOINT_OBJ(endpoint, direction);
    
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

void DRIVER _DRV_USB_DEVICE_EndpointBDTEntryArm
(
    DRV_USB_BDT_ENTRY * pBDT, 
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObj, 
    USB_DEVICE_IRP_LOCAL * irp,
    int direction
)
{
    /* pBDT is the pointer to the ping pong BDT entries
     * for the endpoint and direction  In this driver we
     * dont check for the data toggle while receiving data from
     * the host. The assumption here is that the host is correct */

    /* If the endpoint is stalled, the stall will be cleared */

    uint16_t size;

    DRV_USB_BDT_ENTRY * currentBDTEntry;

    currentBDTEntry = pBDT + endpointObj->nextPingPong;

    /* Calculate the size of the transaction */
    if(USB_DATA_DIRECTION_DEVICE_TO_HOST == direction)
    {
        /* If data is moving from device to host
         * then enable data toggle syncronization */

        currentBDTEntry->byte[0] = 0x08;

        /* Adjust buffer address for the number of
         * bytes sent */
        currentBDTEntry->word[1] = (uint32_t)
            (KVA_TO_PA (irp->data + irp->size - 
                        irp->nPendingBytes));

        if(irp->nPendingBytes == 0)  
        {
            /* This applies when we need to send a ZLP */
            size = 0;
        }
        else
        {
            size = (irp->nPendingBytes > endpointObj->maxPacketSize)
                ? endpointObj->maxPacketSize: irp->nPendingBytes;
        }

        /* Update the pending bytes only if the 
         * data direction is from device to host. The
         * pending bytes for the other direction is 
         * updated in the ISR */

        irp->nPendingBytes -= size;
    }
    else
    {
        /* Data is moving from host to device */
        currentBDTEntry->byte[0] = 0x0;

        /* Adjust the buffer address for the number of bytes
         * received so far */

        currentBDTEntry->word[1] = (uint32_t)
            (KVA_TO_PA (irp->data + irp->nPendingBytes));
        
        size = (irp->size - irp->nPendingBytes > 
                    endpointObj->maxPacketSize) ? endpointObj->maxPacketSize :
                    irp->size - irp->nPendingBytes;

    }

    /* We set up the data toggle. This will be active
     * only if DTS is active. Clear the DATA0/1 and 
     * then set it according to the next data toggle
     * to be used.*/

    currentBDTEntry->byte[0] &= 0xBF;
    currentBDTEntry->byte[0] |= (endpointObj->nextDataToggle << 6);
    
    /* Set the size */
    currentBDTEntry->shortWord[1] = size;
    
    /* Set the UOWN bit */

    currentBDTEntry->byte[0] |= 0x80;

    endpointObj->nextDataToggle ^= 0x1;

}

USB_ERROR DRIVER _DRV_USB_MAKE_NAME(DEVICE_IRPSubmit)
(
    DRV_HANDLE client,
    USB_ENDPOINT endpointAndDirection, 
    USB_DEVICE_IRP * inputIRP
)
{
    uint8_t endpoint;
    int direction;
    DRV_USB_OBJ * hDriver;
    USB_DEVICE_IRP_LOCAL * irp = (USB_DEVICE_IRP_LOCAL *)inputIRP;
    DRV_USB_BDT_ENTRY * pBDT;
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObj;
    bool interruptWasEnabled = false;

    int remainder;

    /* Check for a valid client */

    if(DRV_HANDLE_INVALID == client)
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

    hDriver     = _DRV_USB_OBJ;
    pBDT        = hDriver->pBDT + (endpoint * 4) + (2 * direction);
    endpointObj = _DRV_USB_ENDPOINT_OBJ(endpoint, direction);

    if((endpointObj->endpointState & DRV_USB_DEVICE_ENDPOINT_STATE_ENABLED) == 0)
    {
        /* This means the endpoint is disabled */        
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
        interruptWasEnabled = _DRV_USB_InterruptSourceDisable(DRV_USB_INTERRUPT_SOURCE);
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
        _DRV_USB_DEVICE_EndpointBDTEntryArm(pBDT,endpointObj, 
                irp, direction);
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
            _DRV_USB_InterruptSourceEnable(DRV_USB_INTERRUPT_SOURCE);
        }
        //OSAL: Return mutex
    }
    return(USB_ERROR_NONE);

}

void DRIVER _DRV_USB_DEVICE_IRPQueueFlush
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

USB_ERROR DRIVER _DRV_USB_MAKE_NAME(DEVICE_IRPCancelAll)
(
    DRV_HANDLE client,
    USB_ENDPOINT endpointAndDirection
)
{
    int     direction;
    uint8_t endpoint;
    DRV_USB_OBJ     * hDriver;
    DRV_USB_BDT_ENTRY * pBDT;
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObject;
    bool interruptWasEnabled = false;

    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0);

    if(endpoint >= DRV_USB_ENDPOINTS_NUMBER)
    {
        SYS_ASSERT(false,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    if(DRV_HANDLE_INVALID == client)
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return USB_ERROR_PARAMETER_INVALID;
    }

    hDriver = _DRV_USB_OBJ;

    /* Get the endpoint object */
    endpointObject = _DRV_USB_ENDPOINT_OBJ(endpoint, direction);

    /* Get the BDT entry for this endpoint */
    pBDT = hDriver->pBDT + (4 * endpoint) + (2 * direction);

    if(!hDriver->isInInterruptContext)
    {
        //OSAL : Get mutex
        interruptWasEnabled = _DRV_USB_InterruptSourceDisable(DRV_USB_INTERRUPT_SOURCE);
    }

    /* Get the odd and even endpoint BDT back */
    pBDT->byte[0] = 0x0;
    (pBDT + 1)->byte[0] = 0x0;

    /* Flush the endpoint */
    _DRV_USB_DEVICE_IRPQueueFlush(endpointObject);

    if(!hDriver->isInInterruptContext)
    {
        if(interruptWasEnabled)
        {
            _DRV_USB_InterruptSourceEnable(DRV_USB_INTERRUPT_SOURCE);
        }
        // OSAL: Release Mutex
    }

    return(USB_ERROR_NONE);
}

USB_ERROR DRIVER _DRV_USB_MAKE_NAME(DEVICE_EndpointStall)(DRV_HANDLE client,
                                    USB_ENDPOINT endpointAndDirection)
{
    int     direction;
    bool    interruptWasEnabled = false;
    uint8_t endpoint;
    DRV_USB_OBJ     * hDriver;
    DRV_USB_BDT_ENTRY * pBDT;
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObject;

    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0);

    if(endpoint >= DRV_USB_ENDPOINTS_NUMBER)
    {
        SYS_ASSERT(false,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    if(DRV_HANDLE_INVALID == client)
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return USB_ERROR_PARAMETER_INVALID;
    }

    hDriver = _DRV_USB_OBJ;

    if(!hDriver->isInInterruptContext)
    {
        //OSAL: Get Mutex
        interruptWasEnabled = _DRV_USB_InterruptSourceDisable(DRV_USB_INTERRUPT_SOURCE);
    }
   
    if(endpoint == 0)
    {
        /* For zero endpoint we stall both directions */

        endpointObject = _DRV_USB_ENDPOINT_OBJ(0, 0);
        pBDT = hDriver->pBDT + (endpointObject->nextPingPong);
        
        /* This is the RX direction for EP0. Get the
         * BDT back, stall it, flush all IRPs and then
         * set the endpoint state */
        
        pBDT->byte[0] = 0x0;
        pBDT->byte[0] |= 0x84;
        _DRV_USB_DEVICE_IRPQueueFlush(endpointObject);
        endpointObject->endpointState |= DRV_USB_DEVICE_ENDPOINT_STATE_STALLED;

        /* Now do the same for the TX direction */

        endpointObject = _DRV_USB_ENDPOINT_OBJ(0, 1);
        pBDT = hDriver->pBDT + 2 + (endpointObject->nextPingPong);
        
        /* This is the TX direction for EP0. Get the
         * BDT back, stall it, flush all IRPs and then
         * set the endpoint state */
        
        pBDT->byte[0] = 0x0;
        pBDT->byte[0] |= 0x84;
        _DRV_USB_DEVICE_IRPQueueFlush(endpointObject);
        endpointObject->endpointState |= DRV_USB_DEVICE_ENDPOINT_STATE_STALLED;
    } 
    else
    {
        /* For non zero endpoints we stall the specified direction */

        /* Get the endpoint object */
        endpointObject = _DRV_USB_ENDPOINT_OBJ(endpoint, direction);

        /* Get the BDT entry for this endpoint */
        pBDT = hDriver->pBDT + (4 * endpoint) + (2 * direction)
            + (endpointObject->nextPingPong);
    
        /* Get the endpoint BDT back. Stall the entry.
         * Flush the endpoint and set the object state. */
        pBDT->byte[0] = 0x0;
        pBDT->byte[0] |= 0x84;
        _DRV_USB_DEVICE_IRPQueueFlush(endpointObject);
        endpointObject->endpointState |= DRV_USB_DEVICE_ENDPOINT_STATE_STALLED;

    }
    
    if(!hDriver->isInInterruptContext)
    {
        if(interruptWasEnabled)
        {
            /* Enable the interrupt */
            _DRV_USB_InterruptSourceEnable(DRV_USB_INTERRUPT_SOURCE);
        }
        //OSAL: Release Mutex
    }
    
    return(USB_ERROR_NONE);
}

USB_ERROR DRIVER _DRV_USB_MAKE_NAME(DEVICE_EndpointStallClear)(DRV_HANDLE client,
                                    USB_ENDPOINT endpointAndDirection)
{
    int     direction;
    uint8_t endpoint;
    DRV_USB_OBJ     * hDriver;
    DRV_USB_BDT_ENTRY * pBDT;
    DRV_USB_DEVICE_ENDPOINT_OBJ * endpointObject;

    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0);

    if(endpoint >= DRV_USB_ENDPOINTS_NUMBER)
    {
        SYS_ASSERT(false,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    if(DRV_HANDLE_INVALID == client)
    {
        SYS_ASSERT(false, "Driver Handle is invalid");
        return USB_ERROR_PARAMETER_INVALID;
    }

    hDriver = _DRV_USB_OBJ;

    /* Get the endpoint object */
    endpointObject = _DRV_USB_ENDPOINT_OBJ(endpoint, direction);
    
    /* Get the BDT entry for this endpoint */
    pBDT = hDriver->pBDT + (4 * endpoint) + (2 * direction)
                         + (endpointObject->nextPingPong);

    /* Clear the stall and data toggle on the 
     * endpoint */
    pBDT->byte[0] = 0x0;
    endpointObject->nextDataToggle = USB_BUFFER_DATA0;

    endpointObject->endpointState &= ~DRV_USB_DEVICE_ENDPOINT_STATE_STALLED;

    return(USB_ERROR_NONE);
}

void DRIVER _DRV_USB_DEVICE_Tasks_ISR(DRV_USB_OBJ * hDriver)
{

    int             iEntry;
    void            * returnData;
    bool            bDoEventCallBack;
    bool            processNextIRP;
    unsigned int    errorType;
    DRV_USB_EVENT   eventType = 0;
    USB_INTERRUPTS  usbInterrupts;
    USB_INTERRUPTS  clearUSBInterrupts = 0;

    USB_DEVICE_IRP_LOCAL        * irp;
    DRV_USB_BDT_ENTRY           * currentBDTEntry;
    DRV_USB_BDT_ENTRY           * lastBDTEntry;
    DRV_USB_CLIENT_OBJ          * hClient;
    USB_PING_PONG_STATE         lastPingPong = 0;
    USB_BUFFER_DIRECTION        lastDirection = 0;
    uint8_t                     lastEndpoint = 0;
    DRV_USB_DEVICE_ENDPOINT_OBJ * lastEndpointObj;

    usbInterrupts = PLIB_USB_InterruptFlagAllGet(DRV_USB_PERIPHERAL_ID);
    hDriver = _DRV_USB_OBJ;

    /* Check if an error has occurred */
    if ( PLIB_USB_InterruptFlagGet( DRV_USB_PERIPHERAL_ID, USB_INT_ERROR ) )
    { 
        eventType = DRV_USB_EVENT_ERROR;
        bDoEventCallBack = true;

        errorType = PLIB_USB_ErrorInterruptFlagAllGet(DRV_USB_PERIPHERAL_ID); 

        /* Clear the errors */
        PLIB_USB_ErrorInterruptFlagClear( DRV_USB_PERIPHERAL_ID, errorType ); 

        /* Clear the error flag */
        PLIB_USB_InterruptFlagClear( DRV_USB_PERIPHERAL_ID, USB_INT_ERROR ); 

        /* Set the return value */
        returnData = &errorType;

    }
    else if ( usbInterrupts & USB_INT_DEVICE_RESET )
    { 
        /* Make sure that all BDs are returned 
         * back to the application */


        for(iEntry = 0; iEntry < DRV_USB_ENDPOINTS_NUMBER; iEntry++)
        {
            currentBDTEntry = hDriver->pBDT + (4 * iEntry);
            (currentBDTEntry + 0)->word[0] = 0x0;
            (currentBDTEntry + 1)->word[0] = 0x0;
            (currentBDTEntry + 2)->word[0] = 0x0;
            (currentBDTEntry + 3)->word[0] = 0x0;
        }

        /* Reset all ping pong buffers to even */
        PLIB_USB_PingPongReset(DRV_USB_PERIPHERAL_ID);

        // Reset address to default value (0)
        PLIB_USB_DeviceAddressSet( DRV_USB_PERIPHERAL_ID, 0 );

        /* Valid USB reset seen on bus
         * no event data to send. */

        bDoEventCallBack = true;
        eventType = DRV_USB_EVENT_RESET_DETECT;
        clearUSBInterrupts = USB_INT_DEVICE_RESET;

    }
    else if ( usbInterrupts & USB_INT_SOF ) 
    { 
        /* SOF received by Device or SOF threshold reached by Host
         * no event data to send. */

        bDoEventCallBack = true;
        eventType = DRV_USB_EVENT_SOF_DETECT;
        clearUSBInterrupts = USB_INT_SOF;

    }
    else if ( usbInterrupts & USB_INT_STALL )
    { 
        /* Stall received by Host or sent by Device 
         * Send a bit mask of endpoints which have
         * stalled.*/
		unsigned int iEndpoint;
        bDoEventCallBack = true;
        eventType = DRV_USB_EVENT_STALL;
        clearUSBInterrupts = USB_INT_STALL;
        for ( iEndpoint = 0; iEndpoint <= DRV_USB_ENDPOINTS_NUMBER; iEndpoint++ )
        {
            if ( PLIB_USB_EPnIsStalled(DRV_USB_PERIPHERAL_ID,iEndpoint) )
            {
                PLIB_USB_EPnStallClear(DRV_USB_PERIPHERAL_ID,iEndpoint);
            }
        }
    }
    else if ( usbInterrupts & USB_INT_IDLE_DETECT ) 
    { 
        /* Idle condition detected on bus 
         * no event data to send. */

        bDoEventCallBack = true;
        eventType = DRV_USB_EVENT_IDLE_DETECT;
        clearUSBInterrupts = USB_INT_IDLE_DETECT;

    }
    else if ( usbInterrupts & USB_INT_RESUME ) 
    { 
        /* Resume signalling observed on bus
         * no event data to send */

        bDoEventCallBack = true;
        eventType = DRV_USB_EVENT_RESUME_DETECT;
        clearUSBInterrupts = USB_INT_RESUME;

    }

    PLIB_USB_InterruptFlagClear( DRV_USB_PERIPHERAL_ID, clearUSBInterrupts );

    /* Send all the above events to all the clients */
    if(bDoEventCallBack)
    {
        /* Clear the flag for the next time */

        bDoEventCallBack = false;
        hClient = _DRV_USB_CLIENT_OBJ;

        if(hClient->pEventCallBack != NULL)
        {
            hClient->pEventCallBack(hClient->hClientArg,
                    eventType,  returnData); 
        }

    } /* End of sending error event and other USB bus events
         (non transaction related events) to the clients) */

    while(PLIB_USB_InterruptFlagGet(DRV_USB_PERIPHERAL_ID, USB_INT_TOKEN_DONE))
    {
        Nop();
        Nop();


        /* Get the details of the last transaction */

        PLIB_USB_LastTransactionDetailsGet(DRV_USB_PERIPHERAL_ID, &lastDirection,
                &lastPingPong, &lastEndpoint);

          
        /* Get the associated endpoint object */
        lastEndpointObj = _DRV_USB_ENDPOINT_OBJ(lastEndpoint, lastDirection);

        /* Toggle the next ping pong */
        lastEndpointObj->nextPingPong ^= 1;

        /* Get the first IRP in the queue */
        irp = lastEndpointObj->irpQueue; 

        /* Get the BDT entry for this direction. currentBDTEntry
         * points to the ping pong set. lastBDTEntry points to 
         * the specific ping or pong entry. */

        currentBDTEntry = hDriver->pBDT + (4 * lastEndpoint) + 
            (2 * lastDirection); 

        lastBDTEntry = currentBDTEntry + lastPingPong;

        /* This flag lets us know if the current IRP is done
         * and that the next IRP should be processed */

        processNextIRP = false;
        switch(lastBDTEntry->byte[0] & 0x3C)
        {
            case 0x34 :

                /* This means a setup packet has been received */
               
                irp->status = USB_DEVICE_IRP_STATUS_SETUP;
                irp->size   = lastBDTEntry->word[1];

                /* Reset the data toggle on the TX endpoint to DATA1 
                 * because we received a setup packet. Any packet that
                 * the device transmit on this endpoint must start
                 * with DATA1 toggle. */

                (lastEndpointObj + USB_DATA_DIRECTION_DEVICE_TO_HOST)->nextDataToggle 
                    = USB_BUFFER_DATA1;

                PLIB_USB_PacketTransferEnable(DRV_USB_PERIPHERAL_ID);

                /* We should get the next IRP in the
                 * queue . */

                processNextIRP = true;
                break;

            case 0x4:
                /* We received an OUT token. Check if the size
                 * is less than maxPacketSize. This means the end of
                 * the transfer. If the pending size is 0 then again
                 * we end the transfer */

                irp->nPendingBytes 
                    += lastBDTEntry->shortWord[1];

                if((lastBDTEntry->shortWord[1] < lastEndpointObj->maxPacketSize)
                        || (irp->nPendingBytes >= irp->size))
                {
                    /* We end the transfer because we either got the amount 
                     * of data that we were expecting or we got the a short packet*/

 					/* If we got less data than we were expecting, then
                     * set the IRP status to short else say it is completed */
                    if(irp->nPendingBytes >= irp->size)
                    {
                        irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                    }
                    else
                    {
                        irp->status = USB_DEVICE_IRP_STATUS_COMPLETED_SHORT;
                    }

					/* Update the irp size with received data */
                    irp->size = irp->nPendingBytes;

                   processNextIRP = true;

                }
                else
                {
                    /* We must continue this transfer */
                    _DRV_USB_DEVICE_EndpointBDTEntryArm(currentBDTEntry, 
                            lastEndpointObj, irp, lastDirection);
                }
                break;

            case 0x24:
                /* This means that a IN token was received from
                 * the host */
    
                if(irp->nPendingBytes == 0)
                {
                    if((irp->flags & USB_DEVICE_IRP_FLAG_SEND_ZLP) != 0)
                    {
                        /* This means a ZLP must be sent */
                        irp->flags &= ~USB_DEVICE_IRP_FLAG_SEND_ZLP;
                        _DRV_USB_DEVICE_EndpointBDTEntryArm(currentBDTEntry, 
                                lastEndpointObj, irp, lastDirection);
                    }
                    else
                    {
                        irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                        processNextIRP = true;
                    }
                }
                else
                {
                    /* We must continue this transfer */
                    _DRV_USB_DEVICE_EndpointBDTEntryArm(currentBDTEntry, 
                            lastEndpointObj, irp, lastDirection);
                }

                break;
            default:
                SYS_ASSERT(false, "Unknown TOKEN received from host");
                break;
        }

        /* Reset the BDT status */
        lastBDTEntry->byte[0] = 0;

        if(processNextIRP)
        {
            /* Check the queue and get the next IRP */
 
 			lastEndpointObj->irpQueue = irp->next;

            /* Now do the IRP callback*/

            if(irp->callback != NULL)
            {
                /* Invoke the callback */
                irp->callback((USB_DEVICE_IRP *)irp);
            }

            if(lastEndpointObj->irpQueue != NULL)
            {
                /* This means we have something in the
                 * queue */
                lastEndpointObj->irpQueue->status = USB_DEVICE_IRP_STATUS_IN_PROGRESS;
                _DRV_USB_DEVICE_EndpointBDTEntryArm( currentBDTEntry, lastEndpointObj,
                        lastEndpointObj->irpQueue, lastDirection);
            }

        }
        
        PLIB_USB_InterruptFlagClear(DRV_USB_PERIPHERAL_ID,USB_INT_TOKEN_DONE);
    }
}

