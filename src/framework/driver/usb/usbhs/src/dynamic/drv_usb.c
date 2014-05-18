/*******************************************************************************
  USB Device Driver Core Routines

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usb.c

  Summary:
    USB Device Driver Dynamic Implementation of Core routines

  Description:
    The USB device driver provides a simple interface to manage the USB
    modules on Microchip microcontrollers.  This file Implements the core
    interface routines for the USB driver.

    While building the driver from source, ALWAYS use this file in the build.
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

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include "driver/usb/usbhs/src/drv_usb_local.h"

/************************************
 * Driver instance object
 ***********************************/

DRV_USB_OBJ gDrvUSBObj[DRV_USB_INSTANCES_NUMBER];

/*********************************
 * Driver client object
 *********************************/
DRV_USB_CLIENT_OBJ gDrvUSBClientObj[DRV_USB_CLIENTS_NUMBER];

/*********************************
 * Array of endpoint objects. Two 
 * objects per endpoint 
 ********************************/

DRV_USB_DEVICE_ENDPOINT_OBJ gDrvUSBEndpoints [DRV_USB_INSTANCES_NUMBER]
                                            [DRV_USB_ENDPOINTS_NUMBER * 2];


SYS_MODULE_OBJ DRV_USB_Initialize 
( 
    const SYS_MODULE_INDEX  drvIndex,
    const SYS_MODULE_INIT * const init
)
{
    DRV_USB_OBJ * drvObj;
    USBHS_MODULE_ID  usbID;
    DRV_USB_INIT * usbInit;

    if(gDrvUSBObj[drvIndex].inUse == true)
    {
        /* Cannot initialize an object that is 
         * already in use. */

        SYS_ASSERT(false, "Instance already in use");
        return SYS_MODULE_OBJ_INVALID;
    }

    if( drvIndex >= DRV_USB_INSTANCES_NUMBER)
    {
        SYS_ASSERT(false,"Increase the value of DRV_USB_INSTANCES_NUMBER");
        return SYS_MODULE_OBJ_INVALID;
    }

    usbInit     = (DRV_USB_INIT *) init;
    usbID       = usbInit->usbID;

    drvObj = &gDrvUSBObj[drvIndex]; 

    /* Populate the driver object with
     * the required data */

    drvObj->inUse   = true;
    drvObj->status  = SYS_STATUS_BUSY; 
    drvObj->usbID   = usbID;            
    drvObj->operationMode  = usbInit->operationMode; 
    drvObj->operationSpeed = usbInit->operationSpeed;

    /* Assign the endpoint table */
    drvObj->endpointTable = &gDrvUSBEndpoints[drvIndex][0];
    drvObj->interruptSource  = usbInit->interruptSource;

    /* Set number of clients to zero */
    drvObj->nClients = 0;
    drvObj->pDrvUSBClientObj = (DRV_USB_CLIENT_OBJ*)DRV_HANDLE_INVALID;

    if(drvObj->operationMode == DRV_USB_OPMODE_HOST)
    {
        /* For Host the ID pin needs to be pulle down */
        CNPDFbits.CNPDF3 = 1;
    }

    /* Set the state to indicate that the delay will be started */
    drvObj->state = DRV_USB_TASK_STATE_STARTING_DELAY;
    
    return ((SYS_MODULE_OBJ)drvIndex); 

} /* DRV_USB_Initialize */

void DRV_USB_Tasks(SYS_MODULE_OBJ object)
{
    DRV_USB_OBJ * hDriver; 
    hDriver = &gDrvUSBObj[object];
    USBHS_MODULE_ID usbID = hDriver->usbID;

    if(hDriver->status <= SYS_STATUS_UNINITIALIZED)
    {
        /* Driver is not intialized */
        return;
    }

    /* Check the tasks state and maintain */
    switch(hDriver->state)
    {
        case DRV_USB_TASK_STATE_STARTING_DELAY:

            /* Start the delay here */
            hDriver->timerHandle = SYS_TMR_DelayMS(3000);
            if(hDriver->timerHandle != SYS_TMR_HANDLE_INVALID)
            {
                /* Reset the PHY. This is a workaround
                 * for an errata */
                PLIB_USBHS_SoftResetEnable(usbID);

                /* Delay has started. Move to the next state */
                hDriver->state = DRV_USB_TASK_STATE_WAITING_FOR_DELAY_COMPLETE;
            }

            break;

        case DRV_USB_TASK_STATE_WAITING_FOR_DELAY_COMPLETE:

            /* Check if the delay is complete */
            if(SYS_TMR_DelayStatusGet(hDriver->timerHandle)) 
            {
                /* This means the delay is complete. Clear the Soft Reset  */
                PLIB_USBHS_SoftResetDisable(usbID);

                /* Setup the USB Module based on the selected
                 * mode */

                switch(hDriver->operationMode)
                {
                    case DRV_USB_OPMODE_DEVICE:
                        _DRV_USB_DEVICE_INIT(hDriver, object);
                        break;
                    case DRV_USB_OPMODE_HOST:
                        _DRV_USB_HOST_INIT(hDriver, object);
                        break;
                    case DRV_USB_OPMODE_OTG:
                        break;
                    default:
                        SYS_ASSERT(false, "What mode are you trying?");
                        break;
                }

                /* Clear and enable the interrupts */
                _DRV_USB_InterruptSourceClear(hDriver->interruptSource);
                _DRV_USB_InterruptSourceEnable(hDriver->interruptSource);

                /* Indicate that the object is ready and change the state to running
                 * */

                hDriver->status = SYS_STATUS_READY;
                hDriver->state = DRV_USB_TASK_STATE_RUNNING;
            }

            break;

        case DRV_USB_TASK_STATE_RUNNING:
            /* The module is in a running state. Nothing to do */
            break;
    }
}

void DRV_USB_Deinitialize 
( 
    const SYS_MODULE_INDEX  object
)
{
    DRV_USB_OBJ * drvObj;
    USBHS_MODULE_ID  usbID;
    
    if(object == SYS_MODULE_OBJ_INVALID)
    { 
	    /* Invalid object */
	    
	    SYS_ASSERT(false, "Invalid object");
	    return ;
	} 
    
    if( object >= DRV_USB_INSTANCES_NUMBER)
    {
        SYS_ASSERT(false,"Invalid driver Index");
        return  ;
    }

    if(gDrvUSBObj[object].inUse == false)
    {
        /* Cannot deinitialize an object that is 
         * not already in use. */

        SYS_ASSERT(false, "Instance not in use");
        return  ;
    }

    drvObj = &gDrvUSBObj[object]; 
    usbID  = drvObj->usbID;

    /* Populate the driver object with
     * the required data */

    drvObj->inUse   = false;
    drvObj->status  = SYS_STATUS_UNINITIALIZED; 

    /* Clear and disable the interrupts */
    _DRV_USB_InterruptSourceDisable(drvObj->interruptSource);
    _DRV_USB_InterruptSourceClear(drvObj->interruptSource);

    /* Set number of clients to zero */

    drvObj->nClients = 0;
    drvObj->pDrvUSBClientObj = (DRV_USB_CLIENT_OBJ*)DRV_HANDLE_INVALID;

    return;

} /* DRV_USB_Initialize */

SYS_STATUS DRV_USB_Status ( SYS_MODULE_OBJ object )
{
    if(object == SYS_MODULE_OBJ_INVALID)
    {
        SYS_ASSERT(false, "System Module Object is invalid");
        return SYS_STATUS_ERROR;
    }
    
    /* Return the status of the driver object */
    return(gDrvUSBObj[object].status);
}


DRV_HANDLE DRV_USB_Open
(
    const SYS_MODULE_INDEX iDriver,
    const DRV_IO_INTENT    ioIntent 
)
{
    DRV_USB_CLIENT_OBJ * hClient = gDrvUSBClientObj;
    DRV_USB_OBJ * drvObj;

    /* The iDriver value should be valid. It should be
     * less the number of driver object instances. 
     */

    if(iDriver >= DRV_USB_INSTANCES_NUMBER)
    {
        SYS_ASSERT(false, "Bad Driver Index");
        return DRV_HANDLE_INVALID;
    }

	drvObj = &gDrvUSBObj[iDriver];

    if(drvObj->status != SYS_STATUS_READY)
    {
        /* The USB module should be ready */

        SYS_ASSERT(false, "Was the driver initialized?");
        return DRV_HANDLE_INVALID;
    }

    if(ioIntent != (DRV_IO_INTENT_EXCLUSIVE|DRV_IO_INTENT_NONBLOCKING
                |DRV_IO_INTENT_READWRITE))
    {
        /* The driver only supports this mode */

        SYS_ASSERT(false, "IO intent mode not supported");
        return DRV_HANDLE_INVALID;
    }	

    if(DRV_USB_CLIENTS_NUMBER < DRV_USB_INSTANCES_NUMBER)
    {
        /* There should be atleast one client object per
         * per driver instnace */

        SYS_ASSERT(false, "Insufficient client objects");
        return DRV_HANDLE_INVALID;
    }

    if(drvObj->nClients > 0)
    {
        /* Driver supports exclusive open only */
        SYS_ASSERT(false, "Driver already opened once. Cannot open again");
        return DRV_HANDLE_INVALID;
    }

    /* One to One mapping between client object
     * and driver object */

    hClient = &gDrvUSBClientObj[iDriver];

    /* Fill in driver client data structure */

    /* Remember which USB control driver owns me */
    hClient->hDriver        = drvObj; 
    hClient->usbID          = drvObj->usbID;

    /* Clear prior value */

    hClient->pEventCallBack = NULL;

    /* Store the handle in the driver object
     * client table and update the number of
     * clients*/

    hClient->inUse = true;
    drvObj->pDrvUSBClientObj = hClient;
    drvObj->nClients ++;

    /* Return the client object */

    return ( ( DRV_HANDLE ) hClient );
}

void DRV_USB_Close( DRV_HANDLE client )
{
    DRV_USB_CLIENT_OBJ * hClient;
    DRV_USB_OBJ * hDriver;

    if(client == DRV_HANDLE_INVALID)
    {
        SYS_ASSERT(false, "Bad Client Handle");
        return;
    }

    hClient = (DRV_USB_CLIENT_OBJ *) client;
    
    if(!hClient->inUse)
    {
        SYS_ASSERT(false, "Invalid client handle");
        return;
    }
    
    hDriver = (DRV_USB_OBJ *)hClient->hDriver;

    /* Remove this client from the driver client
     * table */

    hDriver->nClients--;
    hDriver->pDrvUSBClientObj = (DRV_USB_CLIENT_OBJ *)DRV_HANDLE_INVALID;

    /* Give back the client */
    hClient->inUse = false;
    hClient->pEventCallBack = NULL;
}

void DRV_USB_Tasks_ISR( SYS_MODULE_OBJ object )
{
    DRV_USB_OBJ * 	hDriver; 
    hDriver = &gDrvUSBObj[object];
    hDriver->isInInterruptContext = true;

	switch(hDriver->operationMode)
	{
        case DRV_USB_OPMODE_DEVICE:
            _DRV_USB_DEVICE_TASKS_ISR(hDriver);
            break;
        case DRV_USB_OPMODE_HOST:
            _DRV_USB_HOST_TASKS_ISR(hDriver);
            break;
        case DRV_USB_OPMODE_OTG:
            break;
        default:
            SYS_ASSERT(false, "What mode are you trying?");
            break;
	}	
  
    /* Clear the interrupt */
    _DRV_USB_InterruptSourceClear(hDriver->interruptSource);
    hDriver->isInInterruptContext = false;
}

void DRV_USB_ResumeControl(DRV_HANDLE hClient, bool control)
{
    DRV_USB_CLIENT_OBJ * client = (DRV_USB_CLIENT_OBJ *)hClient;
    DRV_USB_OBJ * hDriver;
    USBHS_MODULE_ID usbID;

    if((hClient == DRV_HANDLE_INVALID) || (client == NULL))
    {
        SYS_ASSERT(false, "Invalid client");
    }

    hDriver = (DRV_USB_OBJ *)client->hDriver;
    usbID = hDriver->usbID;

    if(control)
    {
        PLIB_USBHS_ResumeEnable(usbID);
    }
    else
    {
        PLIB_USBHS_ResumeDisable(usbID);
    }
}

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

        PLIB_USBHS_ResumeEnable(pusbdrvObj->usbID);
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

        /* Suspend the bus */
        PLIB_USBHS_SuspendEnable(pusbdrvObj->usbID);
        returnValue = true;
    }
    return returnValue;

}/* end of DRV_USB_HOST_Suspend() */


void DRV_USB_ClientEventCallBackSet
( 
    DRV_HANDLE   client          ,
    uintptr_t    hReferenceData ,
    DRV_USB_EVENT_CALLBACK eventCallBack 
)
{
    DRV_USB_CLIENT_OBJ * hClient;

    if(client == DRV_HANDLE_INVALID)
    {
        SYS_ASSERT(false, "Bad Client Handle");
        return;
    }

    hClient = (DRV_USB_CLIENT_OBJ *) client;
    
    if(!hClient->inUse)
    {
        SYS_ASSERT(false, "Invalid client handle");
        return;
    }

    /* Assign event call back and reference data */
    hClient->hClientArg = hReferenceData;
    hClient->pEventCallBack = eventCallBack;
   
    return;
    
}
