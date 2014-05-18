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

#include "driver/usb/src/drv_usb_local.h"

DRV_USB_GROUP gDrvUSBGroup;

#define DRIVER __attribute__((section("Driver")))

void DRIVER _DRV_USB_MAKE_NAME(Initialize) 
( 
    const SYS_MODULE_INIT * const init
)
{
    DRV_USB_OBJ * drvObj;
    DRV_USB_INIT * usbInit;

    if(gDrvUSBGroup.gDrvUSBObj.inUse == true)
    {
        /* Cannot initialize an object that is
         * already in use. */

        SYS_ASSERT(false, "Instance already in use");
        return;
    }

    usbInit = (DRV_USB_INIT *) init;
    drvObj  = &gDrvUSBGroup.gDrvUSBObj;

    /* Populate the driver object with
     * the required data */

    drvObj->inUse           = true;
    drvObj->status          = SYS_STATUS_BUSY; 
    drvObj->operationMode   = usbInit->operationMode; 
    drvObj->pBDT            = (DRV_USB_BDT_ENTRY *)(usbInit->endpointTable);

    /* Turn off USB module */

    PLIB_USB_Disable( DRV_USB_PERIPHERAL_ID );

    /* Setup the Hardware */

    if ( usbInit->stopInIdle )
    {
        PLIB_USB_StopInIdleEnable( DRV_USB_PERIPHERAL_ID );
    }
    else
    {
        PLIB_USB_StopInIdleDisable( DRV_USB_PERIPHERAL_ID );
    }

    if ( usbInit->suspendInSleep )
    {
        PLIB_USB_AutoSuspendEnable( DRV_USB_PERIPHERAL_ID );
    }
    else
    {
        PLIB_USB_AutoSuspendDisable( DRV_USB_PERIPHERAL_ID );
    }

    /* Setup the USB Module based on the selected
     * mode */

    switch(usbInit->operationMode)
    {
        case USB_OPMODE_DEVICE:
            _DRV_USB_DEVICE_INIT(drvObj, 0);
            break;
        case USB_OPMODE_HOST:
            _DRV_USB_HOST_INIT(drvObj, 0);
            break;
        case USB_OPMODE_OTG:
            break;
        default:
            SYS_ASSERT(false, "What mode are you trying?");
            break;

    }

    /* Clear and enable the interrupts */
    _DRV_USB_InterruptSourceClear(DRV_USB_INTERRUPT_SOURCE);
    _DRV_USB_InterruptSourceEnable(DRV_USB_INTERRUPT_SOURCE);

    /* Assign the BDT address */

    PLIB_USB_BDTBaseAddressSet( DRV_USB_PERIPHERAL_ID , 
            (void *)((uint32_t)KVA_TO_PA(drvObj->pBDT) ));

    /* Set number of clients to zero */

    drvObj->nClients = 0;
    drvObj->status = SYS_STATUS_READY; 

    return; 

} /* DRV_USB_Initialize */

void _DRV_USB_MAKE_NAME(Deinitialize)(void) 
{
    /* Deinitialize the data object */

    DRV_USB_OBJ * drvObj = &gDrvUSBGroup.gDrvUSBObj;

    drvObj->inUse   = false;
    drvObj->status  = SYS_STATUS_UNINITIALIZED; 

    /* Turn off USB module */

    PLIB_USB_Disable( DRV_USB_PERIPHERAL_ID );

    /* Clear and disable the interrupts */
    _DRV_USB_InterruptSourceDisable(DRV_USB_INTERRUPT_SOURCE);
    _DRV_USB_InterruptSourceClear(DRV_USB_INTERRUPT_SOURCE);

    /* Set number of clients to zero */

    drvObj->nClients = 0;

} /* DRV_USB_Initialize */

SYS_STATUS DRIVER _DRV_USB_MAKE_NAME(Status)(void) 
{
    /* Return the status of the driver object */
    return(gDrvUSBGroup.gDrvUSBObj.status);
}


DRV_HANDLE DRIVER _DRV_USB_MAKE_NAME(Open)
(
    const DRV_IO_INTENT    ioIntent 
)
{
    DRV_USB_CLIENT_OBJ * hClient;
    DRV_USB_OBJ * drvObj;

    drvObj = &gDrvUSBGroup.gDrvUSBObj;

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

    if(drvObj->nClients > 0)
    {
        /* Driver supports exclusive open only */
        SYS_ASSERT(false, "Driver already opened once. Cannot open again");
        return DRV_HANDLE_INVALID;
    }

    /* One to One mapping between client object
     * and driver object */

    hClient = &gDrvUSBGroup.gDrvUSBClientObj;

    /* Clear prior value */

    hClient->pEventCallBack = NULL;
    
    /* Let the driver know that it has been opened once */ 
    drvObj->nClients ++;

    /* Return the client object */

    return ( ( DRV_HANDLE ) hClient );

}

void DRIVER _DRV_USB_MAKE_NAME(Close)( DRV_HANDLE handle)
{
    DRV_USB_CLIENT_OBJ * hClient = (DRV_USB_CLIENT_OBJ * )handle;
    DRV_USB_OBJ * hDriver = &gDrvUSBGroup.gDrvUSBObj;

    if((handle == DRV_HANDLE_INVALID) || (hClient == NULL))
    {
        SYS_ASSERT(false, "Bad Client Handle");
        return;
    }
    
    hDriver = &gDrvUSBGroup.gDrvUSBObj;

    /* Remove this client from the driver client
     * table */

    hDriver->nClients--;

    /* Give back the client */
    hClient->pEventCallBack = NULL;
}

void DRIVER _DRV_USB_MAKE_NAME(Tasks_ISR)(void )
{
    DRV_USB_OBJ * hDriver = & gDrvUSBGroup.gDrvUSBObj;
    hDriver->isInInterruptContext = true;

    switch(hDriver->operationMode)
    {
        case USB_OPMODE_DEVICE:
            _DRV_USB_DEVICE_TASKS_ISR(hDriver);
            break;
        case USB_OPMODE_HOST:
            //_DRV_USB_HOST_TASKS_ISR(hDriver);
            break;
        case USB_OPMODE_OTG:
            break;
        default:
            SYS_ASSERT(false, "What mode are you trying?");
            break;
	}	
  
    /* Clear the interrupt */
    _DRV_USB_InterruptSourceClear(DRV_USB_INTERRUPT_SOURCE);
    hDriver->isInInterruptContext = false;
}


void DRIVER _DRV_USB_MAKE_NAME(ClientEventCallBackSet)
( 
    DRV_HANDLE  handle,
    uintptr_t   hReferenceData ,
    DRV_USB_EVENT_CALLBACK eventCallBack 
)
{
    
    DRV_USB_CLIENT_OBJ * hClient = (DRV_USB_CLIENT_OBJ * )handle;

    if((handle == DRV_HANDLE_INVALID) || (hClient == NULL))
    {
        SYS_ASSERT(false, "Bad Client Handle");
        return;
    }
   
    /* Assign event call back and reference data */
    hClient->hClientArg = hReferenceData;
    hClient->pEventCallBack = eventCallBack;
   
    return;
    
}
