/*******************************************************************************
  Device Control System Service Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    sys_devcon.h

  Summary:
    Device Control (DEVCON) System Service interface definition.

  Description:
    This file contains the interface definition for the Device Control System
    Service.  It provides a way to interact with the Device Control subsystem 
    to manage the device control requests supported by the system.
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

#ifndef _SYS_DEVCON_H
#define _SYS_DEVCON_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "system/system.h"

// *****************************************************************************
// *****************************************************************************
// Section: SYS DEVCON Data Types
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* SYS Device Control Module Index Numbers

  Summary:
    Device Control System Service index definitions.

  Description:
    These constants provide Device Control System Service index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

*/

#define SYS_DEVCON_INDEX_0                 0


/************************************************************************
  Summary:
    Identifies a particular registered event instance.
  Description:
    SYS DEVCON Handle
    
    This event handle identifies a registered instance of an event. Every
    time the application that tries to access the parameters with respect
    to a particular event, shall used this event handle to refer to that
    event.
  Remarks:
    None.                                                                
  ************************************************************************/

typedef int8_t SYS_DEVCON_HANDLE;


// *****************************************************************************
/* SYS DEVCON Initialize structure

  Summary:
    Identifies the system device control initialize structure.

  Description:
    This structure identifies the system device control initialize structure.

  Remarks:
    None.
*/

typedef struct
{
    /* System module initialization */
    SYS_MODULE_INIT                 moduleInit;
    
} SYS_DEVCON_INIT;



// *****************************************************************************
// *****************************************************************************
// Section: SYS DEVCON Module Initialization Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ SYS_DEVCON_Initialize( const SYS_MODULE_INDEX index,
                                       const SYS_MODULE_INIT * const init )

  Summary:
    Initializes data for the instance of the Device Control module and opens
    the specific module instance.

  Description:
    This function initializes the instance of the Device Control module,
    using the specified initialization data. It also initializes any
    internal data structures.

  Precondition:
    None.

  Parameters:
    index           - Index for the instance to be initialized

    init            - Pointer to a data structure containing any data necessary
                      to initialize the Device Control module. This pointer may 
                      be null if no data is required because static overrides 
                      have been provided.

  Returns:
    If successful, returns a valid handle to an object.  Otherwise, it
    returns SYS_MODULE_OBJ_INVALID. The returned object must be passed as
    argument to SYS_DEVCON_Reinitialize, SYS_DEVCON_Deinitialize, SYS_DEVCON_Tasks and
    SYS_DEVCON_Status routines.

  Example:
    <code>
    SYS_MODULE_OBJ  objectHandle;
    SYS_DEVCON_INIT    initConfig;

    // Populate the DEVCON initialization structure
    initConfig.moduleInit.value     = SYS_MODULE_POWER_RUN_FULL;

    objectHandle = SYS_DEVCON_Initialize (SYS_DEVCON_INDEX_0, (SYS_MODULE_INIT*)&initConfig);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    <code>

  Remarks:
    This routine should only be called once during system initialization
    unless SYS_DEVCON_Deinitialize is first called to deinitialize the device
    instance before reinitializing it. If the system was already initialized
    it safely returns without causing any disturbance.
*/

SYS_MODULE_OBJ SYS_DEVCON_Initialize( const SYS_MODULE_INDEX index,
                                   const SYS_MODULE_INIT * const init );


// *****************************************************************************
/* Function:
    void SYS_DEVCON_Reinitialize( SYS_MODULE_OBJ object,
                               const SYS_MODULE_INIT * const init )

   Summary:
    Reinitializes and refreshes the hardware for the instance of the Device 
    Control module.

   Description:
    This function reinitializes the instance of the Device Control module using 
    the supplied data. It modifies the internal data structure.

  Precondition:
    The SYS_DEVCON_Initialize function should have been called before calling this 
    function.

  Parameters:
    object          - Identifies the SYS DEVCON Object returned by the Initialize
                      interface
    init            - Pointer to the data structure containing any data
                      necessary to initialize the hardware

   Returns:
    None

   Example:
    <code>
    SYS_MODULE_OBJ  objectHandle;
    SYS_DEVCON_INIT    initConfig;
    SYS_STATUS      devconStatus;

    // Populate the device control initialization structure
    initConfig.moduleInit.value     = SYS_MODULE_POWER_RUN_FULL;

    SYS_DEVCON_Reinitialize (objectHandle, (SYS_MODULE_INIT*)&initConfig);

    devconStatus = SYS_DEVCON_Status (object);
    if (SYS_STATUS_ERROR >= devconStatus)
    {
        // Handle error
    }
    <code>

   Remarks:
    This operation uses the same initialization data structure as the
    SYS_DEVCON_Initialize operation. This operation can be used to change the
    power state of a DEVCON module. This function can be called multiple times
    to reinitialize the module. This operation uses the same initialization
    data structure as the Initialize operation. This operation can also be
    used to refresh the hardware registers as defined by the initialization
    data.
*/

void SYS_DEVCON_Reinitialize( SYS_MODULE_OBJ object,
                           const SYS_MODULE_INIT * const init );


// *****************************************************************************
/* Function:
    void SYS_DEVCON_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specific module instance of the DEVCON module

  Description:
    This function deinitializes the specific module instance disabling its operation 
    (and any hardware for driver modules). Resets all of the internal data
    structures and fields for the specified instance to the default settings.

  Precondition:
    The SYS_DEVCON_Initialize function should have been called before calling this 
    function.

  Parameters:
    object          - SYS DEVCON object handle, returned from SYS_DEVCON_Initialize

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     //  Returned from SYS_DEVCON_Initialize
    SYS_STATUS          status;

    SYS_DEVCON_Deinitialize (object);

    status = SYS_DEVCON_Status (object);
    if (SYS_MODULE_DEINITIALIZED == status)
    {
        // Check again later if you need to know
        // when the SYS DEVCON is deinitialized.
    }
    <code>

  Remarks:
    Once the Initialize operation has been called, the Deinitialize
    operation must be called before the Initialize operation can be called
    again.
*/

void SYS_DEVCON_Deinitialize( SYS_MODULE_OBJ object );


/*******************************************************************************
  Function:
       SYS_STATUS SYS_DEVCON_Status( SYS_MODULE_OBJ object )
    
  Summary:
    Returns status of the specific instance of the Device Control module.

  Description:
    This function returns the status of the specific module instance
    disabling its operation (and any hardware for driver modules).

  Conditions:
    The SYS_DEVCON_Initialize function should have been called before
    calling this function.

  Input:
    object -  SYS DEVCON object handle, returned from SYS_DEVCON_Initialize

  Return:
    SYS_STATUS_READY - Indicates that any previous operations have
    succeeded and the module is ready for additional operations.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from SYS_DEVCON_Initialize
    SYS_STATUS          tmrStatus;
    
    devconStatus = SYS_DEVCON_Status (object);
    else if (SYS_STATUS_ERROR >= devconStatus)
    {
        // Handle error
    }
    </code>

  Remarks:
    Any value greater than SYS_STATUS_READY is also a normal running state in 
    which the driver is ready to accept new operations.
    SYS_STATUS_BUSY - Indicates that the driver is busy with a previous system 
                      level operation and cannot start another
    SYS_STATUS_ERROR - Indicates that the driver is in an error state
    Any value less than SYS_STATUS_ERROR is also an error state.
    SYS_MODULE_DEINITIALIZED - Indicates that the driver has been deinitialized
    This value is less than SYS_STATUS_ERROR.
       Once the Initialize operation has been called, the Deinitialize
       operation must be called before the Initialize operation can be called
       again.

  *****************************************************************************/

SYS_STATUS SYS_DEVCON_Status( SYS_MODULE_OBJ object );


/**************************************************************************
  Function:
       void SYS_DEVCON_Tasks( SYS_MODULE_OBJ object )
    
  Summary:
    Maintains the system Device Control state machine.

  Description:
    This function is used to maintain the system Device Control internal
    state machine.

  Conditions:
    The SYS_DEVCON_Initialize function must have been called for the
    specified DEVCON driver instance.

  Input:
    object -  SYS DEVCON object handle, returned from SYS_DEVCON_Initialize

  Return:
    None

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from SYS_DEVCON_Initialize
    
    while (true)
    {
        SYS_DEVCON_Tasks (object);
    
        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application. It is
    called by the system's Tasks routine (SYS_Tasks) or by the appropriate
    raw ISR.                                                               
  **************************************************************************/

void SYS_DEVCON_Tasks( SYS_MODULE_OBJ object );

    
// *****************************************************************************
// *****************************************************************************
// Section: SYS DEVCON Client Setup Routines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* Function:
    void SYS_DEVCON_PerformanceConfig( unsigned int sysclk )

  Summary:
    Configures the PFM wait states and Prefetch Cache module for maximum 
    performance.

  Description:
    This function configures the PFM wait states and Prefetch Cache module 
    for maximum performance.

  Precondition:
    The SYS_DEVCON_Initialize function should have been called before calling this 
    function.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    SYS_DEVCON_PerformanceConfig(SYS_FREQUENCY);
    <code>

  Remarks:
    None.
*/

void SYS_DEVCON_PerformanceConfig( unsigned int sysclk );


// *****************************************************************************
/* Function:
    void SYS_DEVCON_SystemUnlock( void )

  Summary:
    Performs a system unlock sequence by writing to the SYSKEY register.

  Description:
    Performs a system unlock sequence by writing to the SYSKEY register. A 
    system unlock sequence is required before performing certain actions such
    as changing a clock frequency or IO unlocking.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    SYS_DEVCON_SystemUnlock();
    // Perform some action
    SYS_DEVCON_SystemLock();
    <code>

  Remarks:
    None.
*/

void SYS_DEVCON_SystemUnlock( void );


// *****************************************************************************
/* Function:
    void SYS_DEVCON_SystemLock( void )

  Summary:
    Performs a system lock sequence by writing to the SYSKEY register.

  Description:
    Performs a system lock sequence by writing to the SYSKEY register. A 
    system lock sequence is required after performing the action that required
    a system lock sequence.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    SYS_DEVCON_SystemUnlock();
    // Perform some action
    SYS_DEVCON_SystemLock();
    <code>

  Remarks:
    None.
*/

void SYS_DEVCON_SystemLock( void );


// *****************************************************************************
/* Function:
    void SYS_DEVCON_JTAGEnable( void )

  Summary:
    Enables the JTAG port on the device.

  Description:
    Enables the JTAG port on the device.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    SYS_DEVCON_JTAGEnable();
    <code>

  Remarks:
    None.
*/

void SYS_DEVCON_JTAGEnable( void );


// *****************************************************************************
/* Function:
    void SYS_DEVCON_JTAGDisable( void )

  Summary:
    Disables the JTAG port on the device.

  Description:
    Disables the JTAG port on the device.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    SYS_DEVCON_JTAGDisable();
    <code>

  Remarks:
    None.
*/

void SYS_DEVCON_JTAGDisable( void );


#endif //_SYS_DEVCON_H

/*******************************************************************************
 End of File
*/

