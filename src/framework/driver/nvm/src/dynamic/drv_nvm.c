/*******************************************************************************
  NVM Driver Feature Variant for dual buffer support

  Summary:
    NVM Driver Feature Variant for dual buffer support

  Description:
    This file implements the functions which differ based on different parts
    and various implementations of the same feature. The functions are available
    because callback feature of the driver is selected.
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


// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************
#include "system_config.h"
#include "driver/nvm/src/drv_nvm_local.h"
// *****************************************************************************
/* Driver Hardware instance objects.

  Summary:
    Array of hardware instance objects. One object per instance of the
    peripheral.

  Description:
    Array of hardware instance objects. One object per instance of the
    peripheral.

  Remarks:
    Not all modes are available on all micro-controllers.
*/

DRV_NVM_OBJECT        gDrvNVMObj[DRV_NVM_INSTANCES_NUMBER] ;

// *****************************************************************************
/* Driver Client instance objects.

  Summary:
    Array of Client instance objects.

  Description:
    Array of Client instance objects. One object per client.

  Remarks:
    Not all modes are available on all micro-controllers.
 */

DRV_NVM_CLIENT_OBJECT gDrvNVMClientObj[DRV_NVM_CLIENTS_NUMBER];

// *****************************************************************************
/* Driver Buffer Objects.

  Summary:
    Array of buffer objects.

  Description:
    Array of buffer objects. Buffer objects store read, write and
    erase requests from clients.

  Remarks:
    Not all modes are available on all micro-controllers.
 */

DRV_NVM_BUFFER_OBJECT   gDrvNVMBufferObject[DRV_NVM_BUFFER_OBJECT_NUMBER];

#define DRIVER __attribute__((section("driver")))

static void DRIVER _DRV_NVM_SetupHardware
(
 const NVM_MODULE_ID plibId,
 DRV_NVM_OBJECT      *dObj,
 DRV_NVM_INIT        *nvmInit
 )
{
    /* Initialize the Interrupt Source */
    _DRV_NVM_STATIC_INT_SRC( dObj->interruptSource = _DRV_NVM_GET_INT_SRC(nvmInit->interruptSource) );

    /*Set up memory access */
    if( PLIB_NVM_ExistsMemoryModificationControl( _DRV_NVM_PERIPHERAL_ID_GET( plibId) ))
    {
        PLIB_NVM_MemoryModifyInhibit( _DRV_NVM_PERIPHERAL_ID_GET( plibId ) );
    }
} /* _DRV_NVM_SetupHardware */

void DRIVER _DRV_NVM_UnlockSequence
(
 NVM_MODULE_ID               plibId,
 _DRV_NVM_OPERATION_MODE     mode
 )
{
    bool interruptWasEnabled;

    interruptWasEnabled = SYS_INT_Disable();

    PLIB_NVM_MemoryModifyInhibit(plibId);
    PLIB_NVM_MemoryOperationSelect(plibId, mode);
    PLIB_NVM_MemoryModifyEnable(plibId);

    PLIB_NVM_FlashWriteKeySequence( plibId, DRV_NVM_PROGRAM_UNLOCK_KEY1 );
    PLIB_NVM_FlashWriteKeySequence( plibId, DRV_NVM_PROGRAM_UNLOCK_KEY2 );

    PLIB_NVM_FlashWriteStart(plibId);
	while(PLIB_NVM_FlashWriteCycleHasCompleted(plibId));
    if(interruptWasEnabled)
    {
        SYS_INT_Enable();
    }
}

SYS_MODULE_OBJ DRIVER DRV_NVM_Initialize
(
 const SYS_MODULE_INDEX          drvIndex,
 const SYS_MODULE_INIT   *const  init
 )
{
    DRV_NVM_OBJECT  *dObj           = (DRV_NVM_OBJECT*) NULL;
    DRV_NVM_INIT    *nvmInit        = NULL;

    /* Validate the driver index */
    if ( drvIndex > DRV_NVM_INSTANCES_NUMBER )
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    SYS_ASSERT(gDrvNVMObj[drvIndex].inUse == false, "Instance already in use");



    /* Assign to the local pointer the init data passed */
    nvmInit = ( DRV_NVM_INIT * )init;

    dObj = &gDrvNVMObj[drvIndex];

    dObj->inUse = true;

    /* Update the NVM PLIB Id */
    dObj->moduleId      = ( nvmInit->nvmID );

    /* Setup the Hardware */
    _DRV_NVM_SetupHardware( _DRV_NVM_PERIPHERAL_ID_GET( nvmInit->nvmID ),
            dObj,
            nvmInit ) ;

    dObj->numClients    = 0;

    /* Interrupt flag cleared on the safer side */
    _DRV_NVM_InterruptSourceClear( _DRV_NVM_GET_INT_SRC( dObj->interruptSource ));

    /* Enable the interrupt source in case of interrupt mode */
    _DRV_NVM_InterruptSourceEnable( _DRV_NVM_GET_INT_SRC( dObj->interruptSource ) );

    /* Set the current driver state */
    dObj->status = SYS_STATUS_READY;

    /* Return the driver handle */
    return (SYS_MODULE_OBJ)dObj;


} /* DRV_NVM_Initialize */

void DRIVER DRV_NVM_Deinitialize
(
 SYS_MODULE_OBJ object
 )
{
    DRV_NVM_OBJECT        *dObj = (DRV_NVM_OBJECT*) &gDrvNVMObj[object];
    /* Interrupt De-Registration */
    _DRV_NVM_InterruptSourceDisable( _DRV_NVM_GET_INT_SRC( dObj->interruptSource ) );

    /* Set the Device Status */
    dObj->status = SYS_STATUS_UNINITIALIZED;

    /* Remove the driver usage */
    dObj->inUse = false;

} /* DRV_NVM_Deinitialize */

SYS_STATUS DRIVER DRV_NVM_Status( SYS_MODULE_OBJ object  )
{
    /* Return the status associated with the driver handle */
    return(((DRV_NVM_OBJECT *)object)->status );

} /* DRV_NVM_Status */

DRV_HANDLE DRIVER DRV_NVM_Open
(       const SYS_MODULE_INDEX drvIndex,
        const DRV_IO_INTENT ioIntent
        )
{
    /* Multi client variables are removed from single client builds. */
    DRV_NVM_CLIENT_OBJECT    *clientObj      = (DRV_NVM_CLIENT_OBJECT*) gDrvNVMClientObj;
    DRV_NVM_OBJECT           *dObj           = (DRV_NVM_OBJECT*) &gDrvNVMObj[drvIndex];
    unsigned int iClient;

    /* Validate the driver index */
    if ( drvIndex > DRV_NVM_INSTANCES_NUMBER )
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    if(dObj->status != SYS_STATUS_READY)
    {
        /* The NVM module should be ready */
        SYS_ASSERT(false, "Was the driver initialized?");
        return DRV_HANDLE_INVALID;
    }

    /* Check for exclusive access */
    if (((( dObj->numClients ) > 0) && DRV_IO_ISEXCLUSIVE(ioIntent)) )
    {
        /* Set that the hardware instance is opened in exclusive mode */
        return ( DRV_HANDLE_INVALID ) ;
    }

    /* Find available slot in array of client objects */

    for (iClient = 0; iClient < DRV_NVM_CLIENTS_NUMBER ; iClient++)
    {
        if ( !clientObj->inUse )
        {
            /* Found a client object that can be used */
            clientObj->inUse = true;
            clientObj->driverObj = (DRV_NVM_OBJECT*) dObj;
            clientObj->status = DRV_NVM_CLIENT_STATUS_READY;

            /* OSAL - Unlock Mutex */
            return ( (DRV_HANDLE) clientObj);
        }
        clientObj += 1;
    }

    /* OSAL - Unlock Mutex */
    /* Couldn't find open slot in object array */
    SYS_ASSERT(false, "Insufficient Clients. Increase the number of clients via DRV_USB_CLIENTS_NUMBER");
    return  DRV_HANDLE_INVALID ;

} /* DRV_NVM_Open */


void DRV_NVM_Close( const DRV_HANDLE handle)
{
    /* Multi client variables are removed from single client builds. */
    DRV_NVM_CLIENT_OBJECT *clientObj;

    /* Get the Client object from the handle passed */
    clientObj = (DRV_NVM_CLIENT_OBJECT *)handle;

    /* Check for the Client validity */
    SYS_ASSERT( clientObj == (DRV_NVM_CLIENT_OBJECT *)DRV_HANDLE_INVALID, "Invalid Client Object" ) ;

    /* To Do: OSAL - lock Mutex */

    /* Free the Client Instance */
    clientObj->inUse = false;
    clientObj->status = DRV_NVM_STATUS_INVALID;

}/* DRV_NVM_Close */

DRV_NVM_BUFFER_HANDLE DRV_NVM_Read
(
 const DRV_HANDLE hClient,
 uint8_t * destination,
 uint8_t * source,
 const unsigned int nBytes)
{
    int iEntry,i;
    DRV_NVM_BUFFER_OBJECT * bufferObj;

    /* The read buffer function does not need a task routine
     * A buffer object is still used to allow the client to
     * track the status of the read. The function will complete
     * the read and exit from the function. */

    /* OSAL Mutex lock */
    for(iEntry = 0; iEntry < DRV_NVM_BUFFER_OBJECT_NUMBER; iEntry ++)
    {
        /* Search for free buffer object to use */

        if(gDrvNVMBufferObject[iEntry].inUse == false)
        {
            /* This means this object can be used */

            bufferObj = &gDrvNVMBufferObject[iEntry];
            bufferObj->inUse = true;

            /* OSAL Unlock Mutex */

            bufferObj->hClient = hClient;
            bufferObj->size = nBytes;
            bufferObj->nbytes = 0;
            bufferObj->appDataPointer = destination;
            bufferObj->flashMemPointer = source;
            bufferObj->status = DRV_NVM_BUFFER_IN_PROGRESS;

            for(i = 0; i < nBytes; i ++)
            {

                /* Do the actual read here.
                 * TODO: Not using a PLIB call here as the
                 * memory on PIC32 is linear. We may have to
                 * use a PLIB call for other architectures.
                 * Also check impact for Michigan */

                *destination = *source;
                destination ++;
                source ++;

            }

            bufferObj->status = DRV_NVM_BUFFER_COMPLETED;
            bufferObj->nbytes = nBytes;
            bufferObj->inUse = false;
            return ((DRV_NVM_BUFFER_HANDLE)bufferObj);

        }

    }

    /* OSAL Mutex Unlock */

    SYS_ASSERT(false,"Not enough buffer objects. Try increasing DRV_NVM_BUFFER_OBJECT_NUMBER");
    return DRV_NVM_BUFFER_HANDLE_INVALID;

}

DRV_NVM_BUFFER_STATUS DRIVER  DRV_NVM_BufferStatus
(
 DRV_HANDLE handle,
 const DRV_NVM_BUFFER_HANDLE bufferHandle
 )
{
    DRV_NVM_BUFFER_OBJECT * bufferObject = (DRV_NVM_BUFFER_OBJECT*)bufferHandle;
    return bufferObject->status ;
}

void _DRV_NVM_WriteBufferObjProcess(DRV_NVM_OBJECT * dObj, DRV_NVM_BUFFER_OBJECT * bufferObj)
{
    if(bufferObj->isRowAligned)
    {
        /* If the buffer is row aligned then
         * use row programming, else use word
         * programming */


        PLIB_NVM_FlashAddressToModify(dObj->moduleId , _DRV_NVM_KVA_TO_PA((uint32_t)(bufferObj->flashMemPointer)));
        PLIB_NVM_DataBlockSourceAddress ( dObj->moduleId , _DRV_NVM_KVA_TO_PA((uint32_t)(bufferObj->appDataPointer))) ;
        _DRV_NVM_UnlockSequence(dObj->moduleId, _DRV_ROW_PROGRAM_OPERATION);

    }
    else
    {
        /* Word programming */

        PLIB_NVM_FlashAddressToModify(dObj->moduleId , _DRV_NVM_KVA_TO_PA((uint32_t)(bufferObj->flashMemPointer)++) );
        PLIB_NVM_FlashProvideData ( dObj->moduleId , bufferObj->appDataPointer[bufferObj->nbytes++]) ;
        _DRV_NVM_UnlockSequence(dObj->moduleId, _DRV_WORD_PROGRAM_OPERATION);

    }
}

void _DRV_NVM_EraseBufferObjProcess(DRV_NVM_OBJECT * dObj, DRV_NVM_BUFFER_OBJECT * bufferObj)
{
    /* If the buffer should be page aligned
     * Thats how the erase works in real */


    PLIB_NVM_FlashAddressToModify(dObj->moduleId , _DRV_NVM_KVA_TO_PA((uint32_t)(bufferObj->flashMemPointer)));
    _DRV_NVM_UnlockSequence(dObj->moduleId, _DRV_PAGE_ERASE_OPERATION);

}

DRV_NVM_BUFFER_HANDLE DRV_NVM_Write
(
 DRV_HANDLE handle,
 uint8_t * destination,
 uint8_t * source,
 const unsigned int nBytes
 )
{
    int iEntry;
    DRV_NVM_BUFFER_OBJECT * bufferObj;

    DRV_NVM_CLIENT_OBJECT     *clientObj      =       (DRV_NVM_CLIENT_OBJECT*)handle;
    DRV_NVM_OBJECT            *dObj           =       (DRV_NVM_OBJECT *)clientObj->driverObj ;

    /* OSAL Mutex Lock */

    /* Disable the interrupt so that any
     * write operation from an interrupt context
     * does not interfere. */

    _DRV_NVM_InterruptSourceDisable
        ( _DRV_NVM_GET_INT_SRC( dObj->interruptSource ) );

    for(iEntry = 0; iEntry < DRV_NVM_BUFFER_OBJECT_NUMBER; iEntry ++)
    {
        if(gDrvNVMBufferObject[iEntry].inUse == false)
        {
            /* Found a buffer object that can be used */

            bufferObj = &gDrvNVMBufferObject[iEntry];
            bufferObj->inUse = true;
            bufferObj->appDataPointer         =   source;
            bufferObj->flashMemPointer        =   destination;
            bufferObj->size                   =   nBytes;
            bufferObj->nbytes                 =   0;
            bufferObj->status                 =   DRV_NVM_BUFFER_QUEUED;
            bufferObj->flag					  =   DRV_NVM_BUFFER_FLAG_WRITE;

            /* Check if the specified address is row
             * aligned */

            if((uint32_t)(destination) % DRV_NVM_ROW_SIZE == 0)
            {
                bufferObj->isRowAligned = 1;
                bufferObj->noOfRows = (((uint32_t)(nBytes))/DRV_NVM_ROW_SIZE);
            }

            if(dObj->writeEraseQ == NULL)
            {
                dObj->writeEraseQ = bufferObj;
                bufferObj->next = NULL;
                bufferObj->previous = NULL;
                bufferObj->noOfRows--;
                bufferObj->nbytes = bufferObj->nbytes + DRV_NVM_ROW_SIZE;

                /* Because the write Q is NULL,
                 * we can start an operation */

                PLIB_NVM_MemoryModifyInhibit(dObj->moduleId);
                _DRV_NVM_WriteBufferObjProcess(dObj, bufferObj);
            }
            else
            {
                /* This means there is already a buffer queued
                 * up. We add the buffer to the linked list */
                DRV_NVM_BUFFER_OBJECT * iterator;

                iterator = dObj->writeEraseQ;

                /* Find the last object in the queue */
                while(iterator->next != NULL)
                {
                    iterator = iterator->next;
                }

                /* Append the buffer object to the last buffer
                 * object in the queue */

                iterator->next = bufferObj;
                bufferObj->previous = iterator;
                bufferObj->next = NULL;
            }
            _DRV_NVM_InterruptSourceEnable( _DRV_NVM_GET_INT_SRC( dObj->interruptSource ) );
            /*OSAL Mutex UnLock */
            return((DRV_NVM_BUFFER_HANDLE)bufferObj);

        }
    }

    /* Could not find a buffer object */
    _DRV_NVM_InterruptSourceEnable( _DRV_NVM_GET_INT_SRC( dObj->interruptSource ) );
     /*OSAL Mutex UnLock */
    SYS_ASSERT(false,"Could not allocate buffer object. Increase value of DRV_NVM_BUFFER_OBJECT_NUMBER");
    return(DRV_NVM_BUFFER_HANDLE_INVALID);
}



DRV_NVM_BUFFER_HANDLE DRV_NVM_Erase
(
 DRV_HANDLE handle,
 uint8_t * destination,
 const unsigned int nBytes
 )
{
    int iEntry;
    DRV_NVM_BUFFER_OBJECT * bufferObj;

    DRV_NVM_CLIENT_OBJECT     *clientObj      =       (DRV_NVM_CLIENT_OBJECT*)handle;
    DRV_NVM_OBJECT            *dObj           =       (DRV_NVM_OBJECT *)clientObj->driverObj ;

    /* OSAL Mutex Lock */

    /* Disable the interrupt so that any
     * write operation from an interrupt context
     * does not interfere. */

    /* Check if the specified address is page
     * aligned */

    if((uint32_t)(destination) % DRV_NVM_PAGE_SIZE != 0)
    {
        SYS_ASSERT(false,"Specified memory address to erase is not page aligned");
        return(DRV_HANDLE_INVALID);
    }

    _DRV_NVM_InterruptSourceDisable
        ( _DRV_NVM_GET_INT_SRC( dObj->interruptSource ) );

    for(iEntry = 0; iEntry < DRV_NVM_BUFFER_OBJECT_NUMBER; iEntry ++)
    {
        if(gDrvNVMBufferObject[iEntry].inUse == false)
        {
            /* Found a buffer object that can be used */

            bufferObj                       =   &gDrvNVMBufferObject[iEntry];
            bufferObj->inUse                =   true;
            bufferObj->flashMemPointer      =   destination;
            bufferObj->size                 =   nBytes;
            bufferObj->nbytes               =   0;
            bufferObj->status               =   DRV_NVM_BUFFER_QUEUED;
            bufferObj->flag                 =   DRV_NVM_BUFFER_FLAG_ERASE;

            if(dObj->writeEraseQ == NULL)
            {
                dObj->writeEraseQ = bufferObj;
                bufferObj->next = NULL;
                bufferObj->previous = NULL;
                bufferObj->noOfRows--;
                bufferObj->nbytes = bufferObj->nbytes;

                /* Because the write Q/ is NULL,
                 * we can start an operation */

                PLIB_NVM_MemoryModifyInhibit(dObj->moduleId);
                _DRV_NVM_EraseBufferObjProcess(dObj, bufferObj);
            }
            else
            {
                /* This means there is already a buffer queued
                 * up. We add the buffer to the linked list */
                DRV_NVM_BUFFER_OBJECT * iterator;

                iterator = dObj->writeEraseQ;

                /* Find the last object in the queue */
                while(iterator->next != NULL)
                {
                    iterator = iterator->next;
                }

                /* Append the buffer object to the last buffer
                 * object in the queue */

                iterator->next = bufferObj;
                bufferObj->previous = iterator;
                bufferObj->next = NULL;
            }
            _DRV_NVM_InterruptSourceEnable( _DRV_NVM_GET_INT_SRC( dObj->interruptSource ) );
            /*OSAL Mutex UnLock */
            return((DRV_NVM_BUFFER_HANDLE)bufferObj);

        }
    }

    /* Could not find a buffer object */
    _DRV_NVM_InterruptSourceEnable( _DRV_NVM_GET_INT_SRC( dObj->interruptSource ) );
    /*OSAL Mutex UnLock */
    SYS_ASSERT(false,"Could not allocate buffer object. Increase value of DRV_NVM_BUFFER_OBJECT_NUMBER");
    return(DRV_NVM_BUFFER_HANDLE_INVALID);
}


void DRV_NVM_Tasks(SYS_MODULE_OBJ object)
{
    DRV_NVM_OBJECT * dObj = (DRV_NVM_OBJECT *)object;
    DRV_NVM_BUFFER_OBJECT * bufferObj;

    if(SYS_INT_SourceStatusGet(dObj->interruptSource))
    {
        SYS_INT_SourceStatusClear(dObj->interruptSource);

        /* Get the object at the head of the write queue */

        bufferObj = dObj->writeEraseQ;

        if(bufferObj == NULL)
        {
            /* This is fix for an intermittent issue
             * and requires more investigation.*/
            return;
        }

        /* Check if the buffer is complete */

        if(bufferObj->flag == DRV_NVM_BUFFER_FLAG_WRITE)
        {
            if(bufferObj->nbytes >= bufferObj->size)
            {
                bufferObj->status = DRV_NVM_BUFFER_COMPLETED;
                bufferObj->inUse = false;

                /* Get the next object in the queue */

                dObj->writeEraseQ = bufferObj->next;
                if(dObj->writeEraseQ != NULL)
                {
                    bufferObj->noOfRows--;
                    bufferObj->nbytes = bufferObj->nbytes + DRV_NVM_ROW_SIZE;

                    /* Initiate the next write */

                    PLIB_NVM_MemoryModifyInhibit(dObj->moduleId);
                    _DRV_NVM_WriteBufferObjProcess(dObj, bufferObj);
                }
            }
            else
            {
                /* The buffer is not done yet. Continue
                 * processing this buffer */
                bufferObj->noOfRows--;
                bufferObj->nbytes = bufferObj->nbytes + DRV_NVM_ROW_SIZE;
                bufferObj->flashMemPointer += DRV_NVM_ROW_SIZE;
                bufferObj->appDataPointer += DRV_NVM_ROW_SIZE;

                PLIB_NVM_MemoryModifyInhibit(dObj->moduleId);
                _DRV_NVM_WriteBufferObjProcess(dObj, bufferObj);
            }
        }
        else if(bufferObj->flag == DRV_NVM_BUFFER_FLAG_ERASE)
        {
            if(bufferObj->nbytes >= bufferObj->size)
            {
                bufferObj->status = DRV_NVM_BUFFER_COMPLETED;
                bufferObj->inUse = false;

                /* Get the next object in the queue */

                dObj->writeEraseQ = bufferObj->next;
                if(dObj->writeEraseQ != NULL)
                {
                    bufferObj->nbytes = bufferObj->nbytes + DRV_NVM_PAGE_SIZE;

                    /* Initiate the next write */

                    PLIB_NVM_MemoryModifyInhibit(dObj->moduleId);
                    _DRV_NVM_EraseBufferObjProcess(dObj, bufferObj);
                }
            }
            else
            {
                /* The buffer is not done yet. Continue
                 * processing this buffer */
                bufferObj->nbytes = bufferObj->nbytes + DRV_NVM_PAGE_SIZE;
                bufferObj->flashMemPointer += DRV_NVM_PAGE_SIZE;

                PLIB_NVM_MemoryModifyInhibit(dObj->moduleId);
                _DRV_NVM_EraseBufferObjProcess(dObj, bufferObj);
            }
        }
    }
}
