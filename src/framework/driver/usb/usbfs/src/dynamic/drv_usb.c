/*******************************************************************************
  USB Controller Driver Core Routines.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usb.c

  Summary:
    USB Controller Driver Core Routines intended for Dynamic implementation.

  Description:
    The USB Controller driver provides a simple interface to manage the USB
    modules on Microchip microcontrollers.  This file Implements the core
    interface routines to be used both by the client(USB Host or device layer)
    and the system for communicating with USB Contoller driver.
    While building the driver from source, ALWAYS use this file in the build.
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


// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "driver/usb/usbfs/src/drv_usb_local.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/******************************************************
 * Hardware instance, endpoint table and client object
 * lumped together as group to save memory.
 ******************************************************/
DRV_USB_GROUP gDrvUSBGroup[DRV_USB_INSTANCES_NUMBER];

// *****************************************************************************
// *****************************************************************************
// Section: USB Controller Driver Interface Implementations
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
      SYS_MODULE_OBJ DRV_USB_Initialize( const SYS_MODULE_INDEX index,
                                         const SYS_MODULE_INIT * const init )

  Summary:
    Dynamic impementation of DRV_USB_Initialize system interface function.

  Description:
    This is the dynamic impementation of DRV_USB_Initialize system interface
    function. Function performs the following task:
    - Initializes the neccessary USB module as per the instance init data
    - Updates internal data structure for the particular USB instance
    - Returns the USB instance value as a handle to the system
  
  Remarks:
    See drv_usb.h for usage information.
*/

SYS_MODULE_OBJ DRV_USB_Initialize 
(
    const SYS_MODULE_INDEX  drvIndex,
    const SYS_MODULE_INIT * const init
)
{
    /* Start of local variable */
    DRV_USB_OBJ * pusbdrvObj    = (DRV_USB_OBJ *)NULL;
    USB_MODULE_ID  usbID        = USB_NUMBER_OF_MODULES;
    DRV_USB_INIT * pusbInit     = (DRV_USB_INIT *)NULL;
    SYS_MODULE_OBJ returnValue  = SYS_MODULE_OBJ_INVALID;
    /* End of local variable */

    /* Check if the specified driver index is in valid range */
    if(drvIndex >= DRV_USB_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "USB Driver: Invalid driver index");
        returnValue = SYS_MODULE_OBJ_INVALID;
    }

    /* Check if this hardware instance was already initialized */
    else if(gDrvUSBGroup[drvIndex].gDrvUSBObj.inUse == true)
    {
        /* Cannot initialize an object that is
         * already in use. */
        SYS_DEBUG(0, "USB Driver: Hardware Instance already in use");
        returnValue = SYS_MODULE_OBJ_INVALID;
    }
    else
    {
        /* Grab the particular USB instance object*/
        gDrvUSBGroup[drvIndex].gDrvUSBObj.inUse = true;

        /* Assign to the local pointer the init data passed */
        pusbInit   = (DRV_USB_INIT *) init;
        usbID      = pusbInit->usbID;
        pusbdrvObj = &gDrvUSBGroup[drvIndex].gDrvUSBObj;

        /* Populate the driver instance object with required data */
        pusbdrvObj->status  = SYS_STATUS_BUSY;
        pusbdrvObj->usbID   = usbID;
        pusbdrvObj->operationMode  = pusbInit->operationMode;
        pusbdrvObj->pBDT    = (DRV_USB_BDT_ENTRY *)(pusbInit->endpointTable);

        /* Assign the endpoint table */
        pusbdrvObj->endpointTable = &gDrvUSBGroup[drvIndex].gDrvUSBEndpoints[0];

        pusbdrvObj->interruptSource  = pusbInit->interruptSource;

        /* Turn off USB module */
        PLIB_USB_Disable( usbID );

        /* Setup the Hardware */
        if(pusbInit->stopInIdle)
        {
            PLIB_USB_StopInIdleEnable( usbID );
        }
        else
        {
            PLIB_USB_StopInIdleDisable( usbID );
        }

        if(PLIB_USB_ExistsAutomaticSuspend(usbID))
        {
            if(pusbInit->suspendInSleep)
            {
                PLIB_USB_AutoSuspendEnable( usbID );
            }
            else
            {
                PLIB_USB_AutoSuspendDisable( usbID );
            }
        }

        /* Setup the USB Module as per selected mode */
        switch(pusbInit->operationMode)
        {
            case DRV_USB_OPMODE_DEVICE:
                /* Initialize USB Controller for Device mode */
                _DRV_USB_DEVICE_INIT(pusbdrvObj, drvIndex);
                break;
            case DRV_USB_OPMODE_HOST:
                /* Initialize USB Controller for Host mode */
                _DRV_USB_HOST_INIT(pusbdrvObj, drvIndex);
                break;
            case DRV_USB_OPMODE_OTG:
                /* Not implemented at this point of time*/
                break;
            default:
                SYS_DEBUG(0, "What mode are you trying?");
                break;
        }

        /* Clear and enable the interrupts */
        _DRV_USB_InterruptSourceClear(pusbInit->interruptSource);
        _DRV_USB_InterruptSourceEnable(pusbInit->interruptSource);

        /* Assign the BDT table base address */
        PLIB_USB_BDTBaseAddressSet(usbID ,
            (void *)((uint32_t)KVA_TO_PA(pusbdrvObj->pBDT)));

        /* Set number of clients to zero */
        pusbdrvObj->nClients = 0;
        pusbdrvObj->pDrvUSBClientObj = (DRV_USB_CLIENT_OBJ*)DRV_HANDLE_INVALID;

        /* Indicate that the object is ready and in use
         * and return the driver handle */

        pusbdrvObj->status = SYS_STATUS_READY;
        returnValue = drvIndex;
    }

    return (returnValue);

}/* end of DRV_USB_Initialize() */

// *****************************************************************************
/* Function:
    void DRV_USB_Deinitialize( const SYS_MODULE_OBJ object )

  Summary:
    Dynamic impementation of DRV_USB_Deinitialize system interface function.

  Description:
    This is the dynamic impementation of DRV_USB_Deinitialize system interface
    function.

  Remarks:
    See drv_usb.h for usage information.
*/

void DRV_USB_Deinitialize
( 
    const SYS_MODULE_OBJ  object
)
{
    DRV_USB_OBJ * pusbdrvObj = NULL;

    /* Check if USB instance object is valid */
    if((object == SYS_MODULE_OBJ_INVALID) ||
            (object >= DRV_USB_INSTANCES_NUMBER))
    {
        /* Invalid object */
        SYS_DEBUG(0, "USB Driver: Invalid System Module Object");
    }
    else if(gDrvUSBGroup[object].gDrvUSBObj.inUse == false)
    {
        /* Cannot deinitialize an object that is 
         * not in use. */
        SYS_DEBUG(0, "USB Driver: Instance not in use");
    }
    else
    {
        pusbdrvObj = &gDrvUSBGroup[object].gDrvUSBObj;

        /* Release the USB instance object */
        pusbdrvObj->inUse = false;
        
        /* Uninitialize the status*/
        pusbdrvObj->status = SYS_STATUS_UNINITIALIZED;

        /* Set number of clients to zero */
        pusbdrvObj->nClients = 0;

        /* Remove the USB instance to client data structure link */
        pusbdrvObj->pDrvUSBClientObj =
                (DRV_USB_CLIENT_OBJ*)DRV_HANDLE_INVALID;

        /* Clear and disable the interrupts */
        _DRV_USB_InterruptSourceDisable(pusbdrvObj->interruptSource);
        _DRV_USB_InterruptSourceClear(pusbdrvObj->interruptSource);

        /* Turn off USB module */
        PLIB_USB_Disable(pusbdrvObj->usbID);
    }

    return;

} /* end of DRV_USB_Deinitialize() */

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_USB_Status( const SYS_MODULE_OBJ object )

  Summary:
    Dynamic impementation of DRV_USB_Status system interface function.

  Description:
    This is the dynamic impementation of DRV_USB_Status system interface
    function.

  Remarks:
    See drv_usb.h for usage information.
*/

SYS_STATUS DRV_USB_Status
(
    const SYS_MODULE_OBJ object
)
{
    /* Start of local variables */
    SYS_STATUS returnValue = SYS_STATUS_UNINITIALIZED;
    /* End of local variables */

    /* Check if USB instance object is valid */
    if((object == SYS_MODULE_OBJ_INVALID) ||
            (object >= DRV_USB_INSTANCES_NUMBER))
    {
        /* Invalid object */
        SYS_DEBUG(0, "USB Driver: Invalid object");
    }
    else
    {
        returnValue = gDrvUSBGroup[object].gDrvUSBObj.status;
    }

    /* Return the status of the driver object */
    return returnValue;
    
}/* end of DRV_USB_Status() */

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_USB_Open( const SYS_MODULE_INDEX drvIndex,
                             const DRV_IO_INTENT    ioIntent )

  Summary:
    Dynamic impementation of DRV_USB_Open client interface function.

  Description:
    This is the dynamic impementation of DRV_USB_Open client interface
    function.

  Remarks:
    See drv_usb.h for usage information.
*/

DRV_HANDLE DRV_USB_Open
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT    ioIntent 
)
{
    /* Check if the specified driver index is in valid range */
    if(drvIndex >= DRV_USB_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "USB Driver: Bad Driver Index");
    }
    /* Check if USB instance object is ready*/
    else if(gDrvUSBGroup[drvIndex].gDrvUSBObj.status != SYS_STATUS_READY)
    {
        /* The USB module should be ready */
        SYS_DEBUG(0, "USB Driver: Was the driver initialized?");
    }
    else if(ioIntent != (DRV_IO_INTENT_EXCLUSIVE|DRV_IO_INTENT_NONBLOCKING
                |DRV_IO_INTENT_READWRITE))
    {
        /* The driver only supports this mode */
        SYS_DEBUG(0, "USB Driver: IO intent mode not supported");
    }
    else if(gDrvUSBGroup[drvIndex].gDrvUSBObj.nClients > 0)
    {
        /* Driver supports exclusive open only */
        SYS_DEBUG(0, "Driver already opened once. Cannot open again");
    }
    else
    {
        DRV_USB_CLIENT_OBJ * client = &gDrvUSBGroup[drvIndex].gDrvUSBClientObj;

        client->inUse = true;

        /* Client to USB instance mapping */
        client->hDriver = &gDrvUSBGroup[drvIndex].gDrvUSBObj;
        client->pEventCallBack = NULL;

        /* USB instance to client mapping */
        gDrvUSBGroup[drvIndex].gDrvUSBObj.pDrvUSBClientObj = client;

        /* Increment the client number for the specific USB instance*/
        gDrvUSBGroup[drvIndex].gDrvUSBObj.nClients++;

        /* Return the client object address */
        return ((DRV_HANDLE)client);
    }

    /* Return invalid handle */
    return DRV_HANDLE_INVALID;

}/* end of DRV_USB_Open()*/

// *****************************************************************************
/* Function:
    bool DRV_USB_HOST_Resume(DRV_HANDLE handle)

  Summary:
    Dynamic implementation of DRV_USB_HOST_Resume
    client interface function.

  Description:
    This is the dynamic implementation of DRV_USB_HOST_Resume client
    interface function. Function resumes a suspended BUS.

  Remarks:
    See drv_usb.h for usage information.
*/

bool DRV_USB_HOST_Resume
(
    DRV_HANDLE handle
)
{
    /* Start of local variable */
    DRV_USB_CLIENT_OBJ * pusbdrvClient = (DRV_USB_CLIENT_OBJ *)handle;
    DRV_USB_OBJ * pusbdrvObj = (DRV_USB_OBJ *)NULL;
    bool returnValue = false;
    /* End of local variable */
    
    /* Check if the handle is valid */
    if((handle == DRV_HANDLE_INVALID) ||
            (pusbdrvClient->inUse == false))
    {
        SYS_DEBUG(0, "Bad Client or client closed");
    }
    else
    {
        pusbdrvObj = (DRV_USB_OBJ *)pusbdrvClient->hDriver;

        /* Enable the SOF */
        PLIB_USB_SOFEnable(pusbdrvObj->usbID);
        PLIB_USB_InterruptEnable(pusbdrvObj->usbID, USB_INT_SOF);
        returnValue = true;
    }
    return returnValue;

}/* end of DRV_USB_HOST_Resume() */

// *****************************************************************************
/* Function:
    bool DRV_USB_HOST_Suspend(DRV_HANDLE handle)

  Summary:
    Dynamic implementation of DRV_USB_HOST_Suspend
    client interface function.

  Description:
    This is the dynamic implementation of DRV_USB_HOST_Suspend client
    interface function. Function suspends USB BUS.

  Remarks:
    See drv_usb.h for usage information.
*/

bool DRV_USB_HOST_Suspend
(
    DRV_HANDLE handle
)
{
    /* Start of local variable */
    DRV_USB_CLIENT_OBJ * pusbdrvClient = (DRV_USB_CLIENT_OBJ *)handle;
    DRV_USB_OBJ * pusbdrvObj = (DRV_USB_OBJ *)NULL;
    bool returnValue = false;
    /* End of local variable */

    /* Check if the handle is valid */
    if((handle == DRV_HANDLE_INVALID) ||
            (pusbdrvClient->inUse == false))
    {
        SYS_DEBUG(0, "Bad Client or client closed");
    }
    else
    {
        pusbdrvObj = (DRV_USB_OBJ *)pusbdrvClient->hDriver;

        /* Disable the SOF */
        PLIB_USB_SOFDisable(pusbdrvObj->usbID);
        PLIB_USB_InterruptDisable(pusbdrvObj->usbID, USB_INT_SOF);
        returnValue = true;
    }
    return returnValue;

}/* end of DRV_USB_HOST_Suspend() */

// *****************************************************************************
/* Function:
    void DRV_USB_Close( DRV_HANDLE client )

  Summary:
    Dynamic impementation of DRV_USB_Close client interface function.

  Description:
    This is the dynamic impementation of DRV_USB_Close client interface
    function.

  Remarks:
    See drv_usb.h for usage information.
*/

void DRV_USB_Close
(
    DRV_HANDLE handle
)
{
    /* Start of local variable */
    DRV_USB_CLIENT_OBJ * pusbdrvClient = (DRV_USB_CLIENT_OBJ *)NULL;
    DRV_USB_OBJ * pusbdrvObj = (DRV_USB_OBJ *)NULL;
    /* End of local variable */

    /* Check if the handle is valid */
    if(handle == DRV_HANDLE_INVALID)
    {
        SYS_DEBUG(0, "Bad Client Handle");
    }
    else
    {
        pusbdrvClient = (DRV_USB_CLIENT_OBJ *)handle;
        //For RTOS - mutex lock start
        if(pusbdrvClient->inUse)
        {
            pusbdrvClient->inUse = false;
            /* Map local data structure with client to driver pointer */
            pusbdrvObj = (DRV_USB_OBJ *)pusbdrvClient->hDriver;

            /* Remove this client from the driver client table */
            pusbdrvObj->nClients--;
            pusbdrvObj->pDrvUSBClientObj =
                    (DRV_USB_CLIENT_OBJ *)DRV_HANDLE_INVALID;
            pusbdrvClient->pEventCallBack = NULL;
        }
        else
        {
            SYS_DEBUG(0, "Client Handle already closed");
        }
        //For RTOS - mutex lock end
    }
    return;
    
}/* end of DRV_USB_Close() */

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_USB_Tasks_ISR( SYS_MODULE_OBJ object )

  Summary:
    Dynamic impementation of DRV_USB_Tasks_ISR system interface function.

  Description:
    This is the dynamic impementation of DRV_USB_Tasks_ISR system interface
    function.

  Remarks:
    See drv_usb.h for usage information.
*/

void DRV_USB_Tasks_ISR
(
    SYS_MODULE_OBJ object
)
{
    /* Start of local variable */
    DRV_USB_OBJ * pUSBDriver = (DRV_USB_OBJ *)NULL;
    /* End of local varibale */

    pUSBDriver = &gDrvUSBGroup[object].gDrvUSBObj;
    pUSBDriver->isInInterruptContext = true;

	switch(pUSBDriver->operationMode)
	{
            case DRV_USB_OPMODE_DEVICE:
                _DRV_USB_DEVICE_TASKS_ISR(pUSBDriver);
                break;
            case DRV_USB_OPMODE_HOST:
                _DRV_USB_HOST_TASKS_ISR(pUSBDriver);
                break;
            case DRV_USB_OPMODE_OTG:
                break;
            default:
                SYS_DEBUG(0, "What mode are you trying?");
                break;
	}
  
    /* Clear the interrupt */
    _DRV_USB_InterruptSourceClear(pUSBDriver->interruptSource);
    pUSBDriver->isInInterruptContext = false;
    
}/* end of DRV_USB_Tasks_ISR()*/

// *****************************************************************************
/* Function:
    void DRV_USB_ClientEventCallBackSet(DRV_HANDLE   handle,
                                        uintptr_t    hReferenceData,
                                        DRV_USB_EVENT_CALLBACK eventCallBack)

  Summary:
    Dynamic impementation of DRV_USB_ClientEventCallBackSet client
    interface function.

  Description:
    This is the dynamic impementation of DRV_USB_ClientEventCallBackSet
    client interface function.

  Remarks:
    See drv_usb.h for usage information.
*/

void DRV_USB_ClientEventCallBackSet
( 
    DRV_HANDLE   handle,
    uintptr_t    hReferenceData,
    DRV_USB_EVENT_CALLBACK eventCallBack 
)
{
    /* Start of local variables */
    DRV_USB_CLIENT_OBJ * pusbdrvClient = (DRV_USB_CLIENT_OBJ *)handle;
    /* End of local variables */
    
    /* Check if the handle is valid or opened */
    if((handle == DRV_HANDLE_INVALID) ||
            (pusbdrvClient->inUse == false))
    {
        SYS_DEBUG(0, "Bad Client or client closed");
    }
    else
    {
        /* Assign event call back and reference data */
        pusbdrvClient->hClientArg = hReferenceData;
        pusbdrvClient->pEventCallBack = eventCallBack;
    }
   
    return;
    
}/* end of DRV_USB_ClientEventCallBackSet() */

void DRV_USB_Tasks(SYS_MODULE_OBJ object)
{
    /* This driver does not have any non interrupt tasks */
}
