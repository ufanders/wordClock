/*******************************************************************************
  Timer Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_tmr_local.h

  Summary:
    Timer driver local declarations and definitions.

  Description:
    This file contains the Timer driver's local declarations and definitions.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_TMR_LOCAL_H
#define _DRV_TMR_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "system_config.h"
#include "driver/tmr/drv_tmr.h"
#include "driver/tmr/src/drv_tmr_variant_mapping.h"
#include "osal/osal.h"

// *****************************************************************************
// *****************************************************************************
// Section: Version Numbers
// *****************************************************************************
// *****************************************************************************
/* Versioning of the driver */

// *****************************************************************************
/* TMR Driver Version Macros

  Summary:
    Timer driver version.

  Description:
    These constants provide Timer driver version information. The driver
    version is:
    DRV_TMR_VERSION_MAJOR.DRV_TMR_VERSION_MINOR.DRV_TMR_VERSION_PATCH.
    It is represented in DRV_TMR_VERSION as:
    MAJOR *10000 + MINOR * 100 + PATCH, so as to allow comparisons.
    It is also represented in string format in DRV_TMR_VERSION_STR.
    DRV_TMR_TYPE provides the type of the release when the release is alpha
    or beta. The interfaces DRV_TMR_VersionGet() and
    DRV_TMR_VersionStrGet() provide interfaces to the access the version
    and the version string.

  Remarks:
    Modify the return value of DRV_TMR_VersionStrGet and the
    DRV_TMR_VERSION_MAJOR, DRV_TMR_VERSION_MINOR,
    DRV_TMR_VERSION_PATCH and DRV_TMR_VERSION_TYPE.
*/

#define _DRV_TMR_VERSION_MAJOR         0
#define _DRV_TMR_VERSION_MINOR         5
#define _DRV_TMR_VERSION_PATCH         2
#define _DRV_TMR_VERSION_TYPE          'beta'
#define _DRV_TMR_VERSION_STR           "0.52 beta"




// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* TMR Driver Hardware Instance Object

  Summary:
    Defines the object required for the maintainence of the hardware instance.

  Description:
    This defines the object required for the maintainence of the hardware
    instance. This object exists once per hardware instance of the peripheral.

  Remarks:
    None.
*/

typedef struct _DRV_TMR_OBJ_STRUCT
{
    /* Timer Operation Mode */
    DRV_TMR_SYNC_MODE                                       syncMode;

    /* The status of the driver */
    SYS_STATUS                                              status;

    /* Time Period Value */
    uint32_t                                                timerPeriod;

    /* Prescaler Selection from the processor enumneration */
    TMR_PRESCALE                                            prescale;

    /* Client's alarm counter */
    uint32_t                                                alarmCount;

    /* Alarm callback */
    DRV_TMR_CALLBACK                                        alarmCallback;

    /* The array index associated with the object*/
    TMR_MODULE_ID                                           tmrId;

    /* Interrupt Source for TMR Interrupt */
    INT_SOURCE                                              interruptSource;

    /* Timer elapsed or not */
    bool                                                    elapseStatus;

    /* TMR Driver HW instance operational flags */
    bool                                                    inUse;

    /* Alarm in use/ operational Flag */
    bool                                                    alarmInUse;

     /* Do the alarm periodically */
    bool                                                    alarmPeriodic;

    /* OSAL mutex for client object allocation */
    //OSAL_MUTEX_DECLARE(clientObjectMutex);

} DRV_TMR_OBJ;


// To be removed later & the above line be replaced with
// } DRV_TMR_OBJECT, * DRV_TMR_OBJ;
typedef unsigned short int DRV_TMR_OBJ_HANDLE;


// *****************************************************************************
/* TMR Driver Client Object

  Summary:
    Defines the object required for the maintainence of the software clients.

  Description:
    This defines the object required for the maintainence of the software
    clients instance. This object exists once per client instance.

  Remarks:
    None.
*/

typedef struct _DRV_TMR_CLIENT_OBJ_STRUCT
{
    /* Status of the client object */
    DRV_TMR_CLIENT_STATUS                                   status;

    /* Hardware instance index associated with the client */
    DRV_TMR_OBJ_HANDLE                                      driverObject;

    /* TMR Driver client instance in use/operational flag */
    bool                                                    inUse;

} DRV_TMR_CLIENT_OBJ;


// To be removed later & the above line be replaced with
// } DRV_TMR_CLIENT_OBJECT, * DRV_TMR_CLIENT_OBJ;
typedef unsigned short int DRV_TMR_CLIENT_OBJ_HANDLE;


// *****************************************************************************
// *****************************************************************************
// Section: Extern data definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*  Hardware objects for the dynamic driver
*/

extern DRV_TMR_OBJ   gDrvTMRObj[];


// *****************************************************************************
/*  Client Objects for the multi-client driver
*/

extern DRV_TMR_CLIENT_OBJ    gDrvTMRClientObj[];


// *****************************************************************************
/*  Hardware objects for the static driver
*/

extern DRV_TMR_OBJ   gDrvTMRObj0;
extern DRV_TMR_OBJ   gDrvTMRObj1;
extern DRV_TMR_OBJ   gDrvTMRObj2;
extern DRV_TMR_OBJ   gDrvTMRObj3;
extern DRV_TMR_OBJ   gDrvTMRObj4;
extern DRV_TMR_OBJ   gDrvTMRObj5;
extern DRV_TMR_OBJ   gDrvTMRObj6;


// *****************************************************************************
/*  Client objects for the single-client driver
*/

extern DRV_TMR_CLIENT_OBJ    gDrvTMRClientObj0;
extern DRV_TMR_CLIENT_OBJ    gDrvTMRClientObj1;
extern DRV_TMR_CLIENT_OBJ    gDrvTMRClientObj2;
extern DRV_TMR_CLIENT_OBJ    gDrvTMRClientObj3;
extern DRV_TMR_CLIENT_OBJ    gDrvTMRClientObj4;
extern DRV_TMR_CLIENT_OBJ    gDrvTMRClientObj5;
extern DRV_TMR_CLIENT_OBJ    gDrvTMRClientObj6;


// *****************************************************************************
/*  Extern definitions of functions used internally in the timer driver
*/

extern void _DRV_TMR_PeriodSet16BitInternal( TMR_MODULE_ID index, uint16_t countVal );


#endif //#ifndef _DRV_TMR_LOCAL_H

/*******************************************************************************
 End of File
*/

