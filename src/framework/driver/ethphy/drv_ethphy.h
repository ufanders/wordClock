/**************************************************************************
  Company:
    Microchip Technology Inc.
    
  File Name:
    drv_ethphy.h

  Summary:
    Ethernet ETHPHY Device Driver Interface File

  Description:
    Ethernet ETHPHY Device Driver Interface
    
    The Ethernet ETHPHY device driver provides a simple interface to manage
    an Ethernet ETHPHY peripheral using MIIM (or SMI) interface. This file
    defines the interface definitions and prototypes for the Ethernet
    ETHPHY driver.                                                         
  **************************************************************************/

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

#ifndef _DRV_ETHPHY_H
#define _DRV_ETHPHY_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "system_config.h"

#include "driver/driver_common.h"
#include "peripheral/eth/plib_eth.h"



#include "driver/ethmac/drv_ethernet_flags.h"

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

/*****************************************************************************/
/* SYS_MODULE_OBJ

  Summary:
    Generic handle to any MPLAB Harmony system object.

  Description:
    Generic handle to any MPLAB Harmony system object.

  Remarks:
    None.
*/

#if !defined(_SYS_OBJ_HANDLE_)
    #define _SYS_OBJ_HANDLE_
    typedef uintptr_t SYS_OBJ_HANDLE;
    #define SYS_OBJ_HANDLE_INVALID  ( (SYS_OBJ_HANDLE) -1 )
    #define SYS_OBJ_HANDLE_STATIC   ( (SYS_OBJ_HANDLE)  0 )
#endif//#if !defined(SYS_OBJ_HANDLE)



// *****************************************************************************
/* Ethernet PHY Driver Module Index Numbers

  Summary:
    Ethernet PHY driver index definitions.

  Description:
    These constants provide the Ethernet PHY driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_ETHPHY_Initialize and
    DRV_ETHPHY_Open routines to identify the driver instance in use.
*/

#define DRV_ETHPHY_INDEX_0         0
#define DRV_ETHPHY_INDEX_1         1

// *****************************************************************************
/* Ethernet PHY Driver Module Index Count

  Summary:
    Number of valid Ethernet PHY driver indices.

  Description:
    This constant identifies the number of valid Ethernet PHY driver indices.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is derived from part-specific header files defined as part of the
    peripheral libraries.
*/

#define DRV_ETHPHY_INDEX_COUNT     1


// *****************************************************************************
/* Ethernet PHY Driver Client Status

  Summary:
    Identifies the client-specific status of the Ethernet PHY driver.

  Description:
    This enumeration identifies the client-specific status of the Ethernet PHY driver.

  Remarks:
    None.
*/

typedef enum
{
    /* Unspecified error condition */
    DRV_ETHPHY_CLIENT_STATUS_ERROR   /*DOM-IGNORE-BEGIN*/ = DRV_CLIENT_STATUS_ERROR /*DOM-IGNORE-END*/,

   /* Client is not open */
    DRV_ETHPHY_CLIENT_STATUS_CLOSED  /*DOM-IGNORE-BEGIN*/ = DRV_CLIENT_STATUS_CLOSED /*DOM-IGNORE-END*/,

    /* An operation is currently in progress */
    DRV_ETHPHY_CLIENT_STATUS_BUSY    /*DOM-IGNORE-BEGIN*/ = DRV_CLIENT_STATUS_BUSY /*DOM-IGNORE-END*/,

    /* Up and running, no operations running */
    DRV_ETHPHY_CLIENT_STATUS_READY   /*DOM-IGNORE-BEGIN*/ = DRV_CLIENT_STATUS_READY /*DOM-IGNORE-END*/

} DRV_ETHPHY_CLIENT_STATUS;


// *****************************************************************************
/* Pointer To Function:
    typedef ETH_RESULT_CODE  (* DRV_ETHPHY_MIICONFIGURE) ( DRV_HANDLE handle, ETHPHY_CONFIG_FLAGS cFlags );

  Summary:
    Pointer to function to configure the Ethernet PHY in one of the MII/RMII operation modes.

  Description:
    This type describes a pointer to a function that configures the Ethernet PHY
    in one of the MII/RMII operation modes.

  Precondition:
    Communication to the PHY should have been established.

  Parameters:
    - handle  - Client's driver handle (returned from DRV_ETHPHY_Open)
    - cFlags  - Requested configuration flags: ETH_PHY_CFG_RMII or ETH_PHY_CFG_MII

  Returns:
    - ETH_RES_OK      - if success
    - ETH_RES_CFG_ERR - on failure

  Remarks:
    For some external PHYs, the data interface is set by pins on the device
    or fuses in the host.  In this case, the function will return ETH_RES_OK if
    the cFlags requests the already configured interface; otherwise, ETH_RES_CFG_ERR.
*/

typedef ETH_RESULT_CODE  (* DRV_ETHPHY_MIICONFIGURE) ( DRV_HANDLE handle, ETHPHY_CONFIG_FLAGS cFlags );


// *****************************************************************************
/* Pointer To Function:
    typedef ETH_RESULT_CODE  (* DRV_ETHPHY_MDIXCONFIGURE) ( DRV_HANDLE handle, ETH_OPEN_FLAGS oFlags );

  Summary:
    Pointer to function that configures the MDIX mode for the Ethernet PHY.

  Description:
    This type describes a pointer to a function that configures the MDIX mode for the Ethernet PHY.

  Precondition:
    Communication to the PHY should have been established.

  Parameters:
    - handle  - Client's driver handle (returned from DRV_ETHPHY_Open)
    - oFlags  - Requested open flags: ETH_OPEN_MDIX_AUTO, ETH_OPEN_MDIX_NORM,
                or ETH_OPEN_MDIX_NORM | ETH_OPEN_MDIX_SWAP

  Returns:
    ETH_RES_OK - if success; otherwise, ETH_RES_CFG_ERR.

  Remarks:
    For some external PHYs, the MDIX configuration is set by pins on the
    device.  In this case, the function will return ETH_RES_OK if the cFlags
    requests the already configured interface; otherwise, ETH_RES_CFG_ERR.
*/

typedef ETH_RESULT_CODE  (* DRV_ETHPHY_MDIXCONFIGURE) ( DRV_HANDLE handle, ETH_OPEN_FLAGS oFlags );


// *****************************************************************************
/* Pointer to Function:
    typedef unsigned int  (* DRV_ETHPHY_SMICLOCKGET) ( DRV_HANDLE handle );

  Summary:
    Pointer to a function to return the SMI/MIIM maximum clock speed in Hz of the Ethernet PHY.

  Description:
    This type describes a pointer to a function that returns the SMI/MIIM
    maximum clock speed in Hz of the Ethernet PHY.

  Precondition:
    Communication to the PHY should have been established.

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    The maximum SMI/MIIM clock speed as an unsigned integer.


  Remarks:
    This value is PHY specific. All PHYs are requested to support 2.5 MHz.
*/

typedef unsigned int  (* DRV_ETHPHY_SMICLOCKGET) ( DRV_HANDLE handle );


// *****************************************************************************
/* Ethernet PHY Device Driver Initialization Data

  Summary:
    Contains all the data necessary to initialize the Ethernet PHY device.

  Description:
    This data structure contains all the data necessary to initialize the Ethernet PHY
    device.

  Remarks:
    A pointer to a structure of this format containing the desired
    initialization data must be passed into the DRV_ETHPHY_Initialize routine.
*/

typedef struct
{
    SYS_MODULE_INIT       moduleInit;         // System module initialization
    ETH_MODULE_ID         ethphyId;           // Identifies peripheral (PLIB-level) ID
    DRV_ETHPHY_MIICONFIGURE   MyPHYMIIConfigure;  // Select MII or RMII data interface
    DRV_ETHPHY_MDIXCONFIGURE  MyPHYMDIXConfigure; // AutoMDIX or Manual MDIX
    DRV_ETHPHY_SMICLOCKGET    MyPHYSMIClockGet;   // Returns PHY's clock speed

} DRV_ETHPHY_INIT;


// *****************************************************************************
/* SMI Interface Scan Data Status Enumeration

  Summary:
    Values for the status of the SMI of the Ethernet PHY.

  Description:
    This enumeration lists the status of the SMI of the Ethernet PHY.

  Remarks:
    None.
*/

typedef enum
{
    // SMI interface ready for new scan/read/write
    DRV_ETHPHY_SMI_SCAN_DATA_NOTVALID  /*DOM-IGNORE-BEGIN*/ = SYS_STATUS_ERROR /*DOM-IGNORE-END*/,

    // SMI interface ready for new scan/read/write
    DRV_ETHPHY_SMI_SCAN_DATA_VALID     /*DOM-IGNORE-BEGIN*/ = SYS_STATUS_READY /*DOM-IGNORE-END*/

}  DRV_ETHPHY_SMI_SCAN_DATA_STATUS;


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - System Level
// *****************************************************************************
// *****************************************************************************


/*****************************************************************************************
  Function:
       SYS_OBJ_HANDLE DRV_ETHPHY_Initialize( const SYS_MODULE_INDEX        index,
                                             const SYS_MODULE_INIT * const init )
    
  Summary:
    Initializes the Ethernet PHY driver.

  Description:
    This function initializes the Ethernet PHY driver, making it ready for
    clients to open and use it.

  Precondition:
    None.

  Parameters:
    - drvIndex  - Index for the driver instance to be initialized
    - init      - Pointer to a data structure containing any data necessary to
                  initialize the driver. This pointer may be null if no data
                  is required because static overrides have been provided.

  Returns:
    - a valid handle to a driver object, if successful.
    - SYS_OBJ_HANDLE_INVALID if initialization failed.
    
  Example:
    <code>
    DRV_ETHPHY_INIT    init;
    SYS_OBJ_HANDLE  objectHandle;
    
    // Populate the Ethernet PHY initialization structure
    init.phyId  = ETHPHY_ID_0;
    
    // Populate the Ethernet PHY initialization structure
    init.phyId  = ETHPHY_ID_2;
    init.sExtPHYFunctions.MyPHYMIIConfigure  = SMSC8720_MIIConfigure;
    init.sExtPHYFunctions.MyPHYMDIXConfigure = SMSC8720_MDIXConfigure;
    init.sExtPHYFunctions.MyPHYSMIClockGet   = SMSC8720_SMIClockGet;
    
    // Do something
    
    objectHandle = DRV_ETHPHY_Initialize(DRV_ETHPHY_INDEX_0, (SYS_MODULE_INIT*)&init);
    if (SYS_OBJ_HANDLE_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
     - This function must be called before any other Ethernet PHY routine is
       called.
    -  This function should only be called once during system initialization
       unless DRV_ETHPHY_Deinitialize is called to deinitialize the driver
       instance.                                                                             
    - The returned object must be passed as argument to DRV_ETHPHY_Reinitialize,
      DRV_ETHPHY_Deinitialize, DRV_ETHPHY_Tasks and DRV_ETHPHY_Status routines.

  *****************************************************************************************/

SYS_OBJ_HANDLE DRV_ETHPHY_Initialize ( const SYS_MODULE_INDEX        index,
                                       const SYS_MODULE_INIT * const init );


// *****************************************************************************
/* Function:
    void DRV_ETHPHY_Reinitialize( SYS_OBJ_HANDLE                object,
                                  const SYS_MODULE_INIT * const init )

  Summary:
    Reinitializes the driver and refreshes any associated hardware settings.

  Description:
    This function reinitializes the driver and refreshes any associated hardware
    settings using the initialization data given, but it will not interrupt any
    ongoing operations.

  Precondition:
    The DRV_ETHPHY_Initialize function must have been called before calling this
    routine and a valid SYS_OBJ_HANDLE must have been returned.

  Parameters:
    - object          - Driver object handle, returned from the DRV_ETHPHY_Initialize
                        routine
    - init            - Pointer to the initialization data structure

  Returns:
    None.

  Example:
    <code>
    DRV_ETHPHY_INIT    init;
    SYS_OBJ_HANDLE  objectHandle;

    // Populate the Ethernet PHY initialization structure
    init.phyId  = ETHPHY_ID_2;
    init.sExtPHYFunctions.MyPHYMIIConfigure  = &SMSC8720_MIIConfigure;
    init.sExtPHYFunctions.MyPHYMDIXConfigure = &SMSC8720_MDIXConfigure;
    init.sExtPHYFunctions.MyPHYSMIClockGet   = &SMSC8720_SMIClockGet;

    DRV_ETHPHY_Reinitialize(objectHandle, (SYS_MODULE_INIT*)&init);

    phyStatus = DRV_ETHPHY_Status(objectHandle);
    if (SYS_STATUS_BUSY == phyStatus)
    {
        // Check again later to ensure the driver is ready
    }
    else if (SYS_STATUS_ERROR >= phyStatus)
    {
        // Handle error
    }
    </code>

  Remarks:
    - This function can be called multiple times to reinitialize the module.
    - This operation can be used to refresh any supported hardware registers as
      specified by the initialization data or to change the power state of the
      module.
*/

void DRV_ETHPHY_Reinitialize ( SYS_OBJ_HANDLE               object,
                               const SYS_MODULE_INIT * const init );


// *****************************************************************************
/* Function:
    void DRV_ETHPHY_Deinitialize ( SYS_OBJ_HANDLE object )

  Summary:
    De-initializes the specified instance of the Ethernet PHY driver module.

  Description:
    This function deinitializes the specified instance of the Ethernet PHY driver
    module, disabling its operation (and any hardware) and invalidates all of the
    internal data.

  Precondition:
    The DRV_ETHPHY_Initialize function must have been called before calling this
    routine and a valid SYS_OBJ_HANDLE must have been returned.

  Parameters:
    - object    - Driver object handle, returned from DRV_ETHPHY_Initialize

  Returns:
    None.

  Example:
    <code>
    SYS_OBJ_HANDLE      object;     //  Returned from DRV_ETHPHY_Initialize
    SYS_STATUS          status;

    DRV_ETHPHY_Deinitialize(object);

    status = DRV_ETHPHY_Status(object);
    if (SYS_MODULE_DEINITIALIZED != status)
    {
        // Check again later if you need to know
        // when the driver is deinitialized.
    }
    </code>

  Remarks:
    - Once the Initialize operation has been called, the Deinitialize operation
      must be called before the Initialize operation can be called again.
*/

void DRV_ETHPHY_Deinitialize ( SYS_OBJ_HANDLE object );


/**************************************************************************
  Function:
       SYS_STATUS DRV_ETHPHY_Status ( SYS_OBJ_HANDLE object )
    
  Summary:
    Provides the current status of the Ethernet PHY driver module.

  Description:
    This function provides the current status of the Ethernet PHY driver
    module.

  Precondition:
    The DRV_ETHPHY_Initialize function must have been called before calling
    this function.

  Parameters:
    object -  Driver object handle, returned from DRV_ETHPHY_Initialize

  Returns:
    - SYS_STATUS_READY  - Indicates that any previous module operation for the
                          specified module has completed
    - SYS_STATUS_BUSY   - Indicates that a previous module operation for the
                          specified module has not yet completed
    - SYS_STATUS_ERROR  - Indicates that the specified module is in an error state

  Example:
    <code>
    SYS_OBJ_HANDLE      object;     // Returned from DRV_ETHPHY_Initialize
    SYS_STATUS          status;
    
    status = DRV_ETHPHY_Status(object);
    if (SYS_STATUS_ERROR >= status)
    {
        // Handle error
    }
    </code>

  Remarks:
    - Any value greater than SYS_STATUS_READY is also a normal running state
      in which the driver is ready to accept new operations.
    - SYS_STATUS_BUSY - Indicates that the driver is busy with a previous
      system level operation and cannot start another
    - SYS_STATUS_ERROR - Indicates that the driver is in an error state
    - Any value less than SYS_STATUS_ERROR is also an error state.
    - SYS_MODULE_DEINITIALIZED - Indicates that the driver has been
      deinitialized
    - The this operation can be used to determine when any of the driver's
      module level operations has completed.
    - If the status operation returns SYS_STATUS_BUSY, the a previous
      operation has not yet completed. Once the status operation returns
      SYS_STATUS_READY, any previous operations have completed.
    - The value of SYS_STATUS_ERROR is negative (-1). Any value less than
      that is also an error state.
    - This function will NEVER block waiting for hardware.
    - If the Status operation returns an error value, the error may be
      cleared by calling the reinitialize operation. If that fails, the
      deinitialize operation will need to be called, followed by the
      initialize operation to return to normal operations.                   
  **************************************************************************/

SYS_STATUS DRV_ETHPHY_Status ( SYS_OBJ_HANDLE object );


/***************************************************************************
  Function:
       void DRV_ETHPHY_Tasks( SYS_OBJ_HANDLE object )
    
  Summary:
    Maintains the driver's state machine and implements its ISR.

  Description:
    This function is used to maintain the driver's internal state machine
    and implement its ISR for interrupt-driven implementations.

  Precondition:
    The DRV_ETHPHY_Initialize routine must have been called for the
    specified Ethernet PHY driver instance.

  Parameters:
    - object -  Object handle for the specified driver instance (returned from
                DRV_ETHPHY_Initialize)
  Returns:
    None
  Example:
    <code>
    SYS_OBJ_HANDLE      object;     // Returned from DRV_ETHPHY_Initialize
    
    while (true)
    {
        DRV_ETHPHY_Tasks (object);
    
        // Do other tasks
    }
    </code>

  Remarks:
    - This function is normally not called directly by an application. It is
      called by the system's Tasks routine (SYS_Tasks)
    - This function will never block or access any resources that may cause
      it to block.                        
  ***************************************************************************/

void DRV_ETHPHY_Tasks( SYS_OBJ_HANDLE object );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client Level
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_ETHPHY_Open( const SYS_MODULE_INDEX drvIndex,
                                const DRV_IO_INTENT    intent )

  Summary:
    Opens the specified Ethernet PHY driver instance and returns a handle to it.

  Description:
    This function opens the specified Ethernet PHY driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver.

  Precondition:
    The DRV_ETHPHY_Initialize function must have been called before calling this
    function.

  Parameters:
    - drvIndex    - Identifier for the object instance to be opened
    - intent      - Zero or more of the values from the enumeration
                    DRV_IO_INTENT ORed together to indicate the intended use
                    of the driver

  Returns:
    - valid open-instance handle if successful (a number
      identifying both the caller and the module instance).
    - DRV_HANDLE_INVALID if an error occurs

  Example:
    <code>
    DRV_HANDLE  handle;

    handle = DRV_ETHPHY_Open(DRV_ETHPHY_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        // Unable to open the driver
    }
    </code>

  Remarks:
    - The handle returned is valid until the DRV_ETHPHY_Close routine is called.
    - This function will NEVER block waiting for hardware.
    - If the DRV_IO_INTENT_BLOCKING is requested and the driver was built
      appropriately to support blocking behavior, other client-level
      operations may block waiting on hardware until they are complete.
    - If DRV_IO_INTENT_NON_BLOCKING is requested the driver client can call the
      DRV_ETHPHY_ClientStatus operation to find out when the module is in the ready
      state.
    - If the requested intent flags are not supported, the function will return
      DRV_HANDLE_INVALID.
*/

DRV_HANDLE DRV_ETHPHY_Open( const SYS_MODULE_INDEX drvIndex,
                            const DRV_IO_INTENT    intent  );


// *****************************************************************************
/* Function:
    void DRV_ETHPHY_Close( DRV_HANDLE handle )

  Summary:
    Closes an opened instance of the Ethernet PHY driver.

  Description:
    This function closes an opened instance of the Ethernet PHY driver, invalidating
    the handle.

  Precondition:
    The DRV_ETHPHY_Initialize routine must have been called for the specified
    Ethernet PHY driver instance.

    DRV_ETHPHY_Open must have been called to obtain a valid opened device handle.

  Parameters:
    - handle     - A valid open instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_ETHPHY_Open

    DRV_ETHPHY_Close(handle);
    </code>

  Remarks:
    - After calling this routine, the handle passed in "handle" must not be used
      with any of the remaining driver routines.  A new handle must be obtained by
      calling DRV_ETHPHY_Open before the caller may use the driver again.
    - If DRV_IO_INTENT_BLOCKING was requested and the driver was built
      appropriately to support blocking behavior call may block until the
      operation is complete.
    - If DRV_IO_INTENT_NON_BLOCKING request the driver client can call the
      DRV_ETHPHY_Status operation to find out when the module is in
      the ready state (the handle is no longer valid).
    - Usually there is no need for the driver client to verify that the Close
      operation has completed.
*/

void DRV_ETHPHY_Close( DRV_HANDLE handle );


/**************************************************************************
  Function:
       ETH_RESULT_CODE DRV_ETHPHY_Setup( DRV_HANDLE  handle,
                                         int phyAddress,
                                         ETH_OPEN_FLAGS   oFlags,
                                         ETHPHY_CONFIG_FLAGS cFlags,
                                         ETH_OPEN_FLAGS*  pResFlags )
    
  Summary:
    Initializes Ethernet PHY communication.

  Description:
    This function initializes the Ethernet PHY communication. It tries to
    detect the external Ethernet PHY, to read the capabilities and find a
    match with the requested features. Then, it programs the Ethernet PHY
    accordingly.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid device
      handle.

  Parameters:
    - handle -     A valid open instance handle, returned from the driver's
                   open routine
    - phyAddress   The PHY SMI address
    - oFlags -     The requested open flags
    - cFlags -     Ethernet PHY MII/RMII configuration flags
    - pResFlags -  Address to store the initialization results

  Returns:
    - ETH_RES_OK for success
    - an ETH_RESULT_CODE error code if the set up procedure failed.

  Example:
    <code>
    
    </code>

  Remarks:
    None.
  **************************************************************************/

ETH_RESULT_CODE DRV_ETHPHY_Setup( DRV_HANDLE handle,
                                  int phyAddress,
                                  ETH_OPEN_FLAGS   oFlags,
                                  ETHPHY_CONFIG_FLAGS cFlags,
                                  ETH_OPEN_FLAGS *  pResFlags );


/***************************************************************************
  Function:
       DRV_ETHPHY_CLIENT_STATUS DRV_ETHPHY_ClientStatus( DRV_HANDLE handle )
    
  Summary:
    Gets the current client-specific status the Ethernet PHY driver.

  Description:
    This function gets the client-specific status of the Ethernet PHY
    driver associated with the given handle.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid opened device
      handle.

  Parameters:
    - handle -  A valid instance handle, returned from the driver's open
                routine
  Returns:
    - DRV_ETHPHY_CLIENT_STATUS value describing the current status of the
      driver.

  Example:
    <code>
    DRV_HANDLE phyHandle;  // Returned from DRV_ETHPHY_Open
    DRV_ETHPHY_CLIENT_STATUS phyClientStatus;
    
    phyClientStatus = DRV_ETHPHY_ClientStatus(phyHandle);
    if(DRV_ETHPHY_CLIENT_STATUS_ERROR >= phyClientStatus)
    {
        // Handle the error
    }
    </code>

  Remarks:
    - This function will not block for hardware access and will immediately
      return the current status.                                             
  ***************************************************************************/

DRV_ETHPHY_CLIENT_STATUS DRV_ETHPHY_ClientStatus( DRV_HANDLE handle );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - SMI/MIIM Interface
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void DRV_ETHPHY_SMIReadStart( DRV_HANDLE handle,
                                  unsigned int rIx)
  Summary:
    Initiates a SMI/MIIM read transaction.

  Description:
    This function initiates a SMI/MIIM read transaction for a given PHY address
    and register.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid opened device
      handle.
    - DRV_ETHPHY_Setup must have been called to properly configure the PHY

  Parameters:
    - handle  - Client's driver handle (returned from DRV_ETHPHY_Open)
    - rIx     - PHY register to be accessed

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    None.
*/

void DRV_ETHPHY_SMIReadStart( DRV_HANDLE handle,
                              unsigned int rIx);


// *****************************************************************************
/* Function:
    uint16_t DRV_ETHPHY_SMIReadResultGet( DRV_HANDLE handle )

  Summary:
    Gets the result of the SMI/MIIM register read.

  Description:
    This function gets the result of the SMI/MIIM register read.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid device handle.
    - DRV_ETHPHY_Setup must have been called to properly configure the PHY
    - DRV_ETHPHY_SMIReadStart was called to initiate a SMI/MIIM register read.
    - DRV_ETHPHY_SMIisBusy should return false.

  Parameters:
    - handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    Result of the SMI/MIIM register read previously scheduled.

  Example:
    <code>
    </code>

  Remarks:
    None.
*/

uint16_t DRV_ETHPHY_SMIReadResultGet( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
    void DRV_ETHPHY_SMIWriteStart( DRV_HANDLE handle,
                                  unsigned int rIx,
                                  uint16_t     wData )
  Summary:
    Initiates a SMI/MIIM write transaction.

  Description:
    This function initiates a SMI/MIIM write transaction for a given PHY address
    and register.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid device handle.
    - DRV_ETHPHY_Setup must have been called to properly configure the PHY

  Parameters:
    - handle  - Client's driver handle (returned from DRV_ETHPHY_Open)
    - rIx     - PHY register to be accessed
    - wData   - Data to be written

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    None.
*/

void DRV_ETHPHY_SMIWriteStart( DRV_HANDLE handle,
                              unsigned int rIx,
                              uint16_t     wData );


/**********************************************************************
  Function:
       bool DRV_ETHPHY_SMIisBusy( DRV_HANDLE handle )
    
  Summary:
    Checks the status of the SMI/MIIM interface.

  Description:
    This function returns a boolean 'true' if the SMI/MIIM interface is
    busy with a transaction.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid device
      handle.
    - DRV_ETHPHY_Setup must have been called to properly configure the PHY

  Parameters:
    - handle -  Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
      - true  - if the SMI/MIIM is busy
      - false - if the SMI/MIIM is not busy

  Example:
    <code>
    
    </code>

  Remarks:
    None.
  **********************************************************************/

bool DRV_ETHPHY_SMIisBusy( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
    void DRV_ETHPHY_SMIScanStart( DRV_HANDLE handle,
                                  unsigned int rIx)
  Summary:
    Starts the scan of the SMI/MIIM register.

  Description:
    This function starts the scan of the SMI/MIIM register.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid device
      handle.
    - DRV_ETHPHY_Setup must have been called to properly configure the PHY

  Parameters:
    - handle  - Client's driver handle (returned from DRV_ETHPHY_Open)
    - rIx:    PHY register to be accessed, 0-31

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    None
*/

void DRV_ETHPHY_SMIScanStart( DRV_HANDLE handle,
                              unsigned int rIx);


// *****************************************************************************
/* Function:
    void DRV_ETHPHY_SMIScanStop( DRV_HANDLE handle )

  Summary:
    Stops the scan of the SMI/MIIM register.

  Description:
    This function stops the current scan of the SMI/MIIM register.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid device
      handle.
    - DRV_ETHPHY_Setup must have been called to properly configure the PHY
    - DRV_ETHPHY_SMIScanStart was called to start a scan


  Parameters:
    - handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    None.
*/

void DRV_ETHPHY_SMIScanStop( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
    DRV_ETHPHY_SMI_SCAN_DATA_STATUS DRV_ETHPHY_SMIScanStatusGet( DRV_HANDLE handle )

  Summary:
    Gets the status of the SMI/MIIM scan data.

  Description:
    This function gets the status of the SMI/MIIM scan data.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid device
      handle.
    - DRV_ETHPHY_Setup must have been called to properly configure the PHY
    - DRV_ETHPHY_SMIScanStart() has been called.

  Parameters:
    - handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    - DRV_ETHPHY_SMI_SCAN_DATA_VALID if scan data was obtained
    - DRV_ETHPHY_SMI_SCAN_DATA_NOTVALID if scan data not available

  Example:
    <code>
    </code>

  Remarks:
    None.
*/

DRV_ETHPHY_SMI_SCAN_DATA_STATUS DRV_ETHPHY_SMIScanStatusGet( DRV_HANDLE handle );


/*******************************************************************
  Function:
       uint16_t DRV_ETHPHY_SMIScanDataGet( DRV_HANDLE handle )
    
  Summary:
    Gets the latest SMI/MIIM scan data result.

  Description:
    This function gets the latest SMI/MIIM scan data result.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid device
      handle.
    - DRV_ETHPHY_Setup must have been called to properly configure the PHY
    - DRV_ETHPHY_SMIScanStart() has been called.
    - DRV_ETHPHY_SMIScanStatusGet() should return DRV_ETHPHY_SMI_SCAN_DATA_VALID.

  Parameters:
    - handle -  Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    Current scan result.

  Example:
    <code>
    
    </code>

  Remarks:
    Returned scan data status must have been DRV_ETHPHY_SMI_SCAN_DATA_VALID.    
  *******************************************************************/

uint16_t DRV_ETHPHY_SMIScanDataGet( DRV_HANDLE handle );


/**************************************************************************
  Function:
       void DRV_ETHPHY_SMIClockSet( DRV_HANDLE handle,
                                    uint32_t hostClock,
                                    uint32_t maxSMIClock )
    
  Summary:
    Sets the SMI/MIIM interface clock.

  Description:
    This function sets SMI/MIIM interface clock base on host clock and
    maximum supported SMI/MIIM interface clock speed.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid device
      handle.
    - DRV_ETHPHY_Setup must have been called to properly configure the PHY

  Parameters:
    - handle      -  Client's driver handle (returned from DRV_ETHPHY_Open)
    - hostClock   -  Host clock speed in Hertz
    - maxSMIClock -  Maximum supported SMI/MIIM clock speed in Hertz

  Returns:
    None.

  Example:
    <code>
    
    </code>
  Remarks:
    None.                                                                           
  **************************************************************************/

void DRV_ETHPHY_SMIClockSet( DRV_HANDLE handle,
                             uint32_t hostClock,
                             uint32_t maxSMIClock );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client & Module Level
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void DRV_ETHPHY_RestartNegotiation( DRV_HANDLE handle )

  Summary:
    Restarts auto-negotiation of the Ethernet PHY link.

  Description:
    This function restarts auto-negotiation of the Ethernet PHY link.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid device
      handle.
    - DRV_ETHPHY_Setup must have been called to properly configure the PHY

  Parameters:
    - handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    None.
*/

void DRV_ETHPHY_RestartNegotiation( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
    ETHPHY_CONFIG_FLAGS DRV_ETHPHY_HWConfigFlagsGet( DRV_HANDLE handle )

  Summary:
    Returns the current Ethernet PHY hardware MII/RMII and ALTERNATE/DEFAULT
    configuration flags.

  Description:
    This function returns the current Ethernet PHY hardware MII/RMII and
    ALTERNATE/DEFAULT configuration flags from the Device Configuration Fuse bits.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid device
      handle.
    - DRV_ETHPHY_Setup must have been called to properly configure the PHY

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    ETHPHY_CONFIG_FLAGS enumeration value for Ethernet PHY configuration flag

  Example:
    <code>
    </code>

  Remarks:
    None.
*/

ETHPHY_CONFIG_FLAGS DRV_ETHPHY_HWConfigFlagsGet( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
    ETH_RESULT_CODE DRV_ETHPHY_NegotiationIsComplete( DRV_HANDLE handle, bool waitComplete )

  Summary:
    Returns the results of a previously initiated Ethernet PHY negotiation.

  Description:
   This function returns the results of a previously initiated Ethernet PHY
   negotiation.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid device
      handle.
    - DRV_ETHPHY_Setup must have been called to properly configure the PHY
    - DRV_ETHPHY_RestartNegotiation should have been called.

  Parameters:
    - handle  - Client's driver handle (returned from DRV_ETHPHY_Open)
    - waitComplete - Boolean flag, true if wait for completion is required

  Returns:
    - ETH_RES_OK if negotiation is done
    - ETH_RES_NEGOTIATION_INACTIVE if no negotiation in progress
    - ETH_RES_NEGOTIATION_NOT_STARTED if negotiation not yet started yet
      (means time out if waitComplete was requested)
    - ETH_RES_NEGOTIATION_ACTIVE if negotiation ongoing
     (means time out if waitComplete was requested).

  Example:
    <code>
    </code>

  Remarks:
    - See also DRV_ETHPHY_NegotiationResultGet.
*/

ETH_RESULT_CODE DRV_ETHPHY_NegotiationIsComplete( DRV_HANDLE handle, bool waitComplete );


// *****************************************************************************
/* Function:
    ETH_LINK_STATUS DRV_ETHPHY_NegotiationResultGet( DRV_HANDLE handle,
                                                  ETH_OPEN_FLAGS* pFlags,
                                                  ETH_PAUSE_TYPE* pPauseType )
  Summary:
    Returns the link status after a completed negotiation.

  Description:
    This function returns the link status after a completed negotiation.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid device
      handle.
    - DRV_ETHPHY_Setup must have been called to properly configure the PHY
    - DRV_ETHPHY_RestartNegotiation, and DRV_ETHPHY_NegotiationIsComplete
      should have been called.

  Parameters:
    - handle     - Client's driver handle (returned from DRV_ETHPHY_Open)
    - pFlags     - Address to store the negotiation result
    - pPauseType - Address to store the pause type supported by the link partner

  Returns:
    ETH_LINK_STATUS enumerated value representing the link status after the (completed) negotiation.

  Example:
    <code>
    </code>

  Remarks:
    The returned value for the negotiation flags is valid only if the negotiation was complete.
*/

ETH_LINK_STATUS DRV_ETHPHY_NegotiationResultGet( DRV_HANDLE handle,
                                                 ETH_OPEN_FLAGS* pFlags,
                                                 ETH_PAUSE_TYPE* pPauseType );


/*********************************************************************************
  Function:
       ETH_LINK_STATUS DRV_ETHPHY_LinkStatusGet( DRV_HANDLE handle, bool refresh )
    
  Summary:
    Returns the current link status.

  Description:
    This function returns the current link status.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid device
      handle.
    - DRV_ETHPHY_Setup must have been called to properly configure the PHY
    
  Parameters:
    - handle -   Client's driver handle (returned from DRV_ETHPHY_Open)
    - refresh -  Boolean flag, true to specify that a double read is needed

  Returns:
    ETH_LINK_STATUS enumerated value representing the current link status.

  Example:
    <code>
    
    </code>

  Remarks:
    - This function reads the Ethernet PHY to get current link status If
      refresh is specified then, if the link is down a second read will be
      performed to return the current link status.
  *********************************************************************************/

ETH_LINK_STATUS DRV_ETHPHY_LinkStatusGet( DRV_HANDLE handle, bool refresh );


/**************************************************************************
  Function:
       bool DRV_ETHPHY_Reset( DRV_HANDLE handle, bool waitComplete )
    
  Summary:
    Immediately resets the Ethernet PHY.

  Description:
    This function immediately resets the Ethernet PHY, optionally waiting
    for a reset to complete.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid device
      handle.
    - DRV_ETHPHY_Setup must have been called to properly configure the PHY
    
  Parameters:
    - handle       - Client's driver handle (returned from DRV_ETHPHY_Open)
    - waitComplete - Boolean flag, if true the procedure will wait for reset
                     to complete
  Returns:
    - true - If PHY reset procedure completed (or completion not required)
    - false - reset procedure failed

  Example:
    <code>
    
    </code>

  Remarks:
    None.
  **************************************************************************/

bool DRV_ETHPHY_Reset( DRV_HANDLE handle, bool waitComplete );


// *****************************************************************************
/* Function:
    void DRV_ETHPHY_LinkScanStart( DRV_HANDLE handle )

  Summary:
    Starts an SMI scan of the link status.

  Description:
    This function starts an SMI scan of the link status.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid device
      handle.
    - DRV_ETHPHY_Setup must have been called to properly configure the PHY

  Parameters:
    - handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    - This function starts an SMII scan of the Ethernet PHY link status register. It is
      a more efficient way of having access to the current link status, since the
      normal SMI frame read operation is slow.
*/

void DRV_ETHPHY_LinkScanStart( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
    void DRV_ETHPHY_LinkScanStop( DRV_HANDLE handle )

  Summary:
    Stops the SMI scan of the link status.

  Description:
    This function stops the SMI scan of the ink status.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid device
      handle.
    - DRV_ETHPHY_Setup must have been called to properly configure the PHY

  Parameters:
    - handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    None.
*/

void DRV_ETHPHY_LinkScanStop( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
    ETH_LINK_STATUS DRV_ETHPHY_LinkScanRead( DRV_HANDLE handle )

  Summary:
    Returns current link status.

  Description:
    This function returns the current link status.

  Precondition:
    - The DRV_ETHPHY_Initialize routine must have been called.
    - DRV_ETHPHY_Open must have been called to obtain a valid device
      handle.
    - DRV_ETHPHY_Setup must have been called to properly configure the PHY
    - DRV_ETHPHY_LinkScanStart must be called first.

  Parameters:
    - handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    ETH_LINK_STATUS enumerated value representing the current link status.

  Example:
    <code>
    </code>

  Remarks:
    - See also DRV_ETHPHY_LinkStatusGet.
*/

ETH_LINK_STATUS DRV_ETHPHY_LinkScanRead( DRV_HANDLE handle );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Version Information
// *****************************************************************************
// *****************************************************************************
/* These functions return the version information of the Ethernet PHY driver */

// *****************************************************************************
/* Function:
    unsigned int DRV_ETHPHY_VersionGet ( const SYS_MODULE_INDEX drvIndex )

  Summary:
    Gets the Ethernet PHY driver version in numerical format.

  Description:
    This function gets the Ethernet PHY driver version. The version is encoded as
    major * 10000 + minor * 100 + patch. The stringed version can be obtained
    using DRV_ETHPHY_VersionStrGet()

  Precondition:
    None.

  Parameters:
    - drvIndex    - Identifier for the object instance to get the version for

  Returns:
    Current driver version in numerical format.

  Example:
    <code>
    unsigned int version;

    version = DRV_ETHPHY_VersionGet( DRV_ETHPHY_INDEX_1 );

    if(version < 110200)
    {
        // Do Something
    }
    </code>

  Remarks:
    None.
*/

unsigned int DRV_ETHPHY_VersionGet ( const SYS_MODULE_INDEX drvIndex );


// *****************************************************************************
/* Function:
    char * DRV_ETHPHY_VersionStrGet ( const SYS_MODULE_INDEX drvIndex )

  Summary:
    Gets the Ethernet PHY driver version in string format.

  Description:
    This function gets the Ethernet PHY driver version. The version is returned as
    major.minor.path[type], where type is optional. The numerical version can
    be obtained using DRV_ETHPHY_VersionGet()

  Precondition:
    None.

  Parameters:
    drvIndex    - Identifier for the object instance to get the version for.

  Returns:
    Current Ethernet PHY driver version in the string format.

  Example:
    <code>
    char *version;
    version = DRV_ETHPHY_VersionStrGet( DRV_ETHPHY_INDEX_1 );
    printf("%s", version);
    </code>

  Remarks:
    None.
*/

char * DRV_ETHPHY_VersionStrGet ( const SYS_MODULE_INDEX drvIndex );


#endif // #ifndef _DRV_ETHPHY_H

/*******************************************************************************
 End of File
*/

