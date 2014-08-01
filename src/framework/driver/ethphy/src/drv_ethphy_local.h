/*******************************************************************************
  Ethernet PHY Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ethphy_local.h

  Summary:
    Ethernet PHY driver local declarations and definitions.

  Description:
    This file contains the Ethernet PHY driver's local declarations and definitions.
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

#ifndef _DRV_ETHPHY_LOCAL_H
#define _DRV_ETHPHY_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "driver/ethphy/drv_ethphy.h"
#include "driver/ethphy/src/dynamic/drv_extphy.h"

// *****************************************************************************
// *****************************************************************************
// Section: Version Numbers
// *****************************************************************************
// *****************************************************************************
/* Versioning of the driver */

// *****************************************************************************
/* Ethernet PHY Driver Version Macros

  Summary:
    Ethernet PHY driver version.

  Description:
    These constants provide Ethernet PHY driver version information.

    The driver version is:
    DRV_ETHPHY_VERSION_MAJOR.DRV_ETHPHY_VERSION_MINOR.DRV_ETHPHY_VERSION_PATCH.
    It is represented in DRV_ETHPHY_VERSION as:
    MAJOR *10000 + MINOR * 100 + PATCH, so as to allow comparisons.
    It is also represented in string format in DRV_ETHPHY_VERSION_STR.
    DRV_ETHPHY_TYPE provides the type of the release when the release is alpha
    or beta. The interfaces DRV_ETHPHY_VersionGet and DRV_ETHPHY_VersionStrGet
    provide interfaces to the access the version and the version string.

  Remarks:
    Modify the return value of DRV_ETHPHY_VersionStrGet and DRV_ETHPHY_VERSION_MAJOR,
    DRV_ETHPHY_VERSION_MINOR, DRV_ETHPHY_VERSION_PATCH, and DRV_ETHPHY_VERSION_TYPE.
*/

#define _DRV_ETHPHY_VERSION_MAJOR         0
#define _DRV_ETHPHY_VERSION_MINOR         3
#define _DRV_ETHPHY_VERSION_PATCH         0
#define _DRV_ETHPHY_VERSION_TYPE          "Alpha"
#define _DRV_ETHPHY_VERSION_STR           "0.3.0 Alpha"


// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* ETHPHY Driver Hardware Instance Object

  Summary:
    Defines the object required for the maintainence of the hardware instance.

  Description:
    This defines the object required for the maintainence of the hardware
    instance. This object exists once per hardware instance of the peripheral.

  Remarks:
    None.
*/

typedef struct _DRV_ETHPHY_OBJ_STRUCT
{
    bool                  inUse; // True if in use
    uint8_t          numClients; // Number of active clients
    SYS_STATUS           status; // Status of module
    SYS_MODULE_INDEX    iModule; // Module instance number
    ETH_MODULE_ID      ethphyId; // The peripheral Id associated with the object
    ETHPHY_CONFIG_FLAGS configFlags; // ETHPHY MII/RMII configuration flags
    int                 phyAddress; // PHY SMI address
    struct // Structure for pointers to external PHY support functions
    {
        DRV_ETHPHY_MIICONFIGURE   MyPHYMIIConfigure;  // Select MII or RMII data interface
        DRV_ETHPHY_MDIXCONFIGURE  MyPHYMDIXConfigure; // AutoMDIX or Manual MDIX
        DRV_ETHPHY_SMICLOCKGET    MyPHYSMIClockGet;   // Returns PHY's clock speed
    } sExtPHYFunctions;

} DRV_ETHPHY_OBJ;


// *****************************************************************************
/* ETHPHY Driver Client Object

  Summary:
    Defines the object required for the maintainence of the software clients.

  Description:
    This defines the object required for the maintainence of the software
    clients instance. This object exists once per client instance.

  Remarks:
    None.
*/

typedef struct _DRV_ETHPHY_CLIENT_OBJ_STRUCT
{
    bool                       inUse; // True if in use
    DRV_ETHPHY_CLIENT_STATUS  status; // Client Status
    DRV_ETHPHY_OBJ *         hDriver; // Handle of driver that owns the client
    ETH_MODULE_ID           ethphyId; // The peripheral Id associated with the object

} DRV_ETHPHY_CLIENT_OBJ;


// *****************************************************************************
// *****************************************************************************
// Section: Extern data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*  Hardware Objects for the dynamic driver
*/

extern DRV_ETHPHY_OBJ            gDrvETHPHYObj[];


// *****************************************************************************
/*  Client Objects for the multi-client driver
*/

extern DRV_ETHPHY_CLIENT_OBJ     gDrvETHPHYClientObj[];


// *****************************************************************************
/*  Hardware Objects for the static driver
*/

extern DRV_ETHPHY_OBJ            gDrvETHPHYObj0;
extern DRV_ETHPHY_OBJ            gDrvETHPHYObj1;


// *****************************************************************************
/*  Client Objects for the single-client driver
*/

extern DRV_ETHPHY_CLIENT_OBJ     gDrvETHPHYClientObj0;
extern DRV_ETHPHY_CLIENT_OBJ     gDrvETHPHYClientObj1;


#endif //#ifndef _DRV_ETHPHY_LOCAL_H

/*******************************************************************************
 End of File
*/

