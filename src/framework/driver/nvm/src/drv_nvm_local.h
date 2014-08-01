/*******************************************************************************
  NVM Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_nvm_local.h

  Summary:
    NVM driver local declarations and definitions

  Description:
    This file contains the timer driver's local declarations and definitions.
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

#ifndef _DRV_NVM_LOCAL_H
#define _DRV_NVM_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "driver/nvm/drv_nvm.h"
#include "driver/nvm/src/drv_nvm_variant_mapping.h"


// *****************************************************************************
// *****************************************************************************
// Section: Version Numbers
// *****************************************************************************
// *****************************************************************************
/* Versioning of the driver */

// *****************************************************************************
/* NVM Driver Version Macros

  Summary:
    NVM driver version

  Description:
    These constants provide NVM driver version information. The driver
    version is
    DRV_NVM_VERSION_MAJOR.DRV_NVM_VERSION_MINOR.DRV_NVM_VERSION_PATCH.
    It is represented in DRV_NVM_VERSION as
    MAJOR *10000 + MINOR * 100 + PATCH, so as to allow comparisons.
    It is also represented in string format in DRV_NVM_VERSION_STR.
    DRV_NVM_TYPE provides the type of the release when the release is alpha
    or beta. The interfaces DRV_NVM_VersionGet() and
    DRV_NVM_VersionStrGet() provide interfaces to the access the version
    and the version string.

  Remarks:
    Modify the return value of DRV_NVM_VersionStrGet and the
    DRV_NVM_VERSION_MAJOR, DRV_NVM_VERSION_MINOR,
    DRV_NVM_VERSION_PATCH and DRV_NVM_VERSION_TYPE
*/

#define _DRV_NVM_VERSION_MAJOR         0
#define _DRV_NVM_VERSION_MINOR         5
#define _DRV_NVM_VERSION_PATCH         1
#define _DRV_NVM_VERSION_TYPE          "beta"
#define _DRV_NVM_VERSION_STR           "0.51 beta"


// *****************************************************************************
// *****************************************************************************
// Section: NVM Driver Static Object Generation
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: _DRV_NVM_OBJ_MAKE_NAME(name)

  Summary:
    Creates an instance-specific static object name

  Description:
     This macro creates the instance-specific name of the given static object
     by inserting the index number into the name.

  Remarks:
    This macro does not affect the dynamic objects
*/

#define _DRV_STATIC_OBJ_NAME_B(name,id) name ## id

#define _DRV_STATIC_OBJ_NAME_A(name,id) _DRV_STATIC_OBJ_NAME_B(name,id)

#define _DRV_NVM_OBJ_MAKE_NAME(name)    _DRV_STATIC_OBJ_NAME_A(name, DRV_NVM_CONFIG_INDEX)


// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* NVM IO Types

   Summary
    Defines the IO types that can be serviced by the NVM driver

   Description
    This enumeration defines the IO types that can be serviced by the NVM
    driver.

   Remarks:
    None
*/

typedef enum
{
    /* Write Operation to be performed on the buffer*/
    DRV_NVM_BUFFER_FLAG_WRITE                 /*DOM-IGNORE-BEGIN*/ = 1 << 0/*DOM-IGNORE-END*/,

    /* Read Operation to be performed on the buffer */
    DRV_NVM_BUFFER_FLAG_READ                  /*DOM-IGNORE-BEGIN*/ = 1 << 1/*DOM-IGNORE-END*/,

    /* Erase Operation to be performed on the buffer */
    DRV_NVM_BUFFER_FLAG_ERASE                  /*DOM-IGNORE-BEGIN*/ = 1 << 2/*DOM-IGNORE-END*/,


}DRV_NVM_BUFFER_FLAGS;

typedef struct _DRV_NVM_BUFFER_OBJECT
{
    bool                            inUse;
    uint8_t                        *appDataPointer;
    uint8_t                        *flashMemPointer;
    uint32_t                        size;
    uint32_t                        nbytes;
    DRV_HANDLE                      hClient;
    DRV_NVM_BUFFER_STATUS           status;
	DRV_NVM_BUFFER_FLAGS			flag;
    bool                            isRowAligned;
    bool                            isPageAligned;
    uint8_t                         noOfPages;
    uint8_t                         noOfRows;
    struct _DRV_NVM_BUFFER_OBJECT   * next;
    struct _DRV_NVM_BUFFER_OBJECT   * previous;
}DRV_NVM_BUFFER_OBJECT;



typedef enum
{
	DRV_NVM_BYTE_PENDING,
	DRV_NVM_BYTE_IS_READY,
	DRV_NVM_BYTE_DATA_AVAILABLE,
	DRV_NVM_BYTE_IDLE
}DRV_NVM_BYTE_TRANSFER_STATUS;

typedef enum {

    _DRV_WORD_PROGRAM_OPERATION = 0x1,
    _DRV_ROW_PROGRAM_OPERATION = 0x3,
    _DRV_PAGE_ERASE_OPERATION = 0x4,
    _DRV_FLASH_ERASE_OPERATION = 0x5

} _DRV_NVM_OPERATION_MODE;

// *****************************************************************************
/* NVM Driver Hardware Instance Object

  Summary:
    Defines the object required for the maintainence of the hardware instance.

  Description:
    This defines the object required for the maintainence of the hardware
    instance. This object exists once per hardware instance of the peripheral.

  Remarks:
    None
*/

typedef struct
{
    /* The module index associated with the object*/
    NVM_MODULE_ID                       moduleId;

    /* Object Index */
    SYS_MODULE_INDEX                    objIndex;

    /* The buffer Q for the write operations */
    DRV_NVM_BUFFER_OBJECT               *writeEraseQ;

    /* The status of the driver */
    SYS_STATUS                          status;

    /* Required : Flag to indicate in use  */
    bool                                inUse;

    /* Flag to indicate that SAMPLE is used in exclusive access mode */
    bool                                IsExclusive;

    bool                                isISRRunning;

    /* number of clients connected to the hardware instance */
    uint8_t                             numClients;

    uint8_t				writeStatus;

    uint8_t				readStatus;

    uint8_t				eraseStatus;

    /* Interrupt Source for TX Interrupt */
    INT_SOURCE                  	interruptSource;

}DRV_NVM_OBJECT;

// *****************************************************************************
/* NVM Driver Client Object

  Summary:
    Defines the object required for the maintainence of the software clients

  Description:
    This defines the object required for the maintainence of the software
    clients instance. This object exists once per client instance.

  Remarks:
    None
*/

typedef struct _DRV_NVM_CLIENT_OBJ_STRUCT
{
    /* Status of the client object */
    DRV_NVM_CLIENT_STATUS                                   status;

    /* The hardware instance object associate with the client */
    void                                                   *driverObj;

    /* Status of the client object */
    SYS_STATUS                                              sysStatus;

    /* The intent with which the client was opened */
    DRV_IO_INTENT                                           intent;

    /* Required : Flag to indicate in use  */
    bool                                                    inUse;

} DRV_NVM_CLIENT_OBJECT;


#endif //#ifndef _DRV_NVM_LOCAL_H

/*******************************************************************************
 End of File
*/

