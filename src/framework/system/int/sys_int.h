/*******************************************************************************
  Interrupt System Service Library Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    sys_int.h

  Summary:
    Interrupt System Service.
    
  Description:
    This file contains the interface definition for the Interrupt System
    Service.  It provides a way to interact with the interrupt subsystem to
    manage the occurrence of interrupts for sources supported by the system.
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

#ifndef _SYS_INT_H
#define _SYS_INT_H


// *****************************************************************************
// *****************************************************************************
// Section: Includes
// *****************************************************************************
// *****************************************************************************

#include "system/common/sys_common.h"
#include "system/common/sys_module.h"

#include "peripheral/int/plib_int.h"



// *****************************************************************************
// *****************************************************************************
// Section: type definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Interrupt Tasks Routine Pointer

  Summary:
    Pointer to an interrupt-handling "Tasks" routine.

  Description:
    This data type defines a pointer to an interrupt-handling "Tasks" routine.
    The form of a tasks routine is as follows:
    
       void My_Tasks ( SYS_MODULE_OBJ object );
    
    Where "MyTasks" is the name of the tasks routine and object is a Handle 
    to the module instance.
    
  Remarks:
    "Tasks" is normally defined by a device driver, middleware, or system
    layer.
    
    The term Interrupt Service Routine (ISR) is used for the "raw" ISR code 
    that is either located directly at the interrupt vector address or whose 
    address is loaded from the interrupt vector.  The term "Tasks" routine is 
    used to identify the driver-specific routine that is called by the actual
    ISR to perform the tasks necessary to handle and clear the interrupt.
*/

typedef void (* SYS_INT_TASKS_POINTER) ( SYS_MODULE_OBJ object );


// *****************************************************************************
// *****************************************************************************
// Section: Interrupt System service Interface Routines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Initialization
// *****************************************************************************
// *****************************************************************************
/******************************************************************************

  Function:
    void SYS_INT_Initialize ( void )

  Summary:
    Configures and initializes the interrupt subsystem.

  Description:
    This  function configures and initializes the interrupt subsystem
    appropriately for the current system design.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Example:
  <code>
  // Initialize the interrupt system. This needs to done in the initialization
  // code.
  SYS_INT_Initialize();
  </code>

  Remarks:
    None.
*/
  
void SYS_INT_Initialize ( void );


// *****************************************************************************
// *****************************************************************************
// Section: ISR Management
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_INT_VectorPrioritySet ( INT_VECTOR vector, INT_PRIORITY_LEVEL priority )

  Summary:
    Sets the given interrupt vector to the specified priority.
    
  Description:
    This routine sets the given interrupt vector to the specified priority.
    
  Precondition:
    SYS_INT_Initialize must have been called.

  Parameters:
    vector	- Identifier for the desired interrupt vector
    
    priority	- Priority (if supported)

  Returns:
    None.

  Example:
    <code>
    #define MY_DRIVER_INTERRUPT_VECTOR	INT_VECTOR_T1
    #define MY_DRIVER_ISR_PRIORITY		INT_PRIORITY_LEVEL2
    // Initialize the interrupt system.This needs to done in the initialization
    // code.
    SYS_INT_Initialize();
    // Assign priority to the interrupt vector
    SYS_INT_VectorPrioritySet(MY_DRIVER_INTERRUPT_VECTOR, MY_DRIVER_ISR_PRIORITY);
    </code>

  Remarks:
    This feature is not supported on all devices.  Refer to the specific device 
    data sheet or family reference manual to determine whether this feature is 
    supported.
    
    In the example code, the macros MY_DRIVER_INTERRUPT_VECTOR,
    MY_DRIVER_ISR_PRIORITY would be defined appropriately during configuration.
*/

void SYS_INT_VectorPrioritySet ( INT_VECTOR vector, INT_PRIORITY_LEVEL priority );


/*******************************************************************************
  Function:
    void SYS_INT_VectorSubprioritySet( INT_VECTOR vector,
                                 INT_SUBPRIORITY_LEVEL subpriority )

  Summary:
    Sets the specified interrupt vector to the given sub priority.
    
  Description:
    This function sets the specified interrupt vector to the specified sub-priority.
    
  Precondition:
    SYS_INT_Initialize must have been called.

  Parameters:
    vector	- Identifier for the desired interrupt vector
    
    subpriority	- Subpriority (if supported)

  Returns:
    None.

  Example:
    <code>
    #define MY_DRIVER_INTERRUPT_VECTOR	INT_VECTOR_T1
    #define MY_DRIVER_ISR_PRIORITY		INT_PRIORITY_LEVEL2
    #define MY_DRIVER_ISR_SUB_PRIORITY	INT_SUBPRIORITY_LEVEL1
    // Initialize the interrupt system.This needs to done in the initialization
    // code.
    SYS_INT_Initialize();
    // Assign priority to the interrupt vector
    SYS_INT_VectorPrioritySet(MY_DRIVER_INTERRUPT_VECTOR, MY_DRIVER_ISR_PRIORITY);
    // Assign sub-priority to the interrupt vector
    SYS_INT_VectorSubprioritySet(MY_DRIVER_INTERRUPT_VECTOR, MY_DRIVER_ISR_SUB_PRIORITY);
    </code>

  Remarks:
    This feature is not supported on all devices.  Refer to the specific device
    data sheet or family reference manual to determine whether this feature is 
    supported.
    
    In the example code, the macros MY_DRIVER_INTERRUPT_VECTOR,
    MY_DRIVER_ISR_SUB_PRIORITY would be defined appropriately during configuration.
*/

void SYS_INT_VectorSubprioritySet ( INT_VECTOR vector, INT_SUBPRIORITY_LEVEL subpriority );


/**********************************************************************************
  Function:
      void SYS_INT_DynamicRegister(  INT_SOURCE           source,
                                     SYS_INT_TASKS_POINTER    tasks,
                                     SYS_MODULE_OBJ object )
    
  Summary:
    Registers an Interrupt "Tasks" Routine for the specified interrupt
    source or trap).
  Description:
    This function registers an Interrupt "Tasks" Routine for the specified
    interrupt source or trap).
  Conditions:
    SYS_INT_Initialize must have been called.
  Input:
    source -  Identifier for the desired interrupt source
    tasks -   Pointer to the tasks routine
    object -  Handle to the module instance
  Return:
    None.
  Example:
    <code>
    SYS_INT_Initialize();
    SYS_INT_DynamicRegister(MY_DRIVER_INTERRUPT_SOURCE, DRV_MYDEV_Tasks, MyObject);
    </code>
  Remarks:
    This routine only generates executable code when a driver is configured
    to register dynamically its "Tasks" routine with the system interrupt
    service. However, it should be called even if the ISR-to-source
    association is defined statically at compile time to maintain source
    code compatibility. A device driver normally registers its own ISR from
    its initialization routine.
    
    In the example code, the macros MY_DRIVER_INTERRUPT_SOURCE would be
    defined in the appropriate configuration header, which would be
    included by the driver source file where the "DRV_MYDEV_Tasks" routine
    and the MyParam data would be defined.
    
    It is safe to call this routine without first calling
    SYS_INT_DynamicDeregister, even if a previous ISR has been registered.
    The effect will be that the new ISR supplants the old one.                     
  **********************************************************************************/

void SYS_INT_DynamicRegister(  	INT_SOURCE           source,
                                SYS_INT_TASKS_POINTER    tasks,
                                SYS_MODULE_OBJ object );


/*******************************************************************************
  Function:
    void SYS_INT_DynamicDeregister ( INT_SOURCE source )

  Summary:
    Deregisters the current ISR from the given interrupt source.
    
  Description:
    This function deregisters the current Interrupt Service Routine (ISR), if
    any, from the specified interrupt source.

  Precondition:
    SYS_INT_Initialize must have been called.

  Parameters:
    source          - Identifier for the desired interrupt source

  Returns:
    None.

  Example:
    <code>
    SYS_INT_DynamicDeregister(SYS_INT_TIMER_1);
    </code>
    
  Remarks:
    It is safe to call this routine, even of no ISR has been registered for 
    the given interrupt source.
    
    Calling this routine is optional.  If the system is designed such that the 
    given ISR is expected to always be available once the system has been 
    initialized, this routine does not need to be called.
*/

void SYS_INT_DynamicDeregister ( INT_SOURCE source );


// *****************************************************************************
// *****************************************************************************
// Section: Global Interrupt Management Routines
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_INT_Enable ( void )

  Summary:
    Enables global interrupts to the processor.
    
  Description:
    This function enables interrupts to the processor at the top level, allowing
    any currently enabled source to generate an interrupt.  This function must be
    called before any source will be able to generate an interrupt.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    // Check if global interrupts are enabled
    if(!SYS_INT_IsEnabled())
    {
        // Enable the global interrupts.
        SYS_INT_Enable();
    }
    </code>

  Remarks:
    SYS_INT_Enable is called from the SYS_INT_Initialize() function.
*/

void SYS_INT_Enable ( void );


/*******************************************************************************
  Function:
    bool SYS_INT_Disable ( void )

  Summary:
    Disables interrupts to the processor.
    
  Description:
    This function disables interrupts to the processor at the top level. This
    function can be called to prevent any source from being able to generate an
    interrupt. It returns the global interrupt status before disabling
    the interrupts.

  Precondition:
    SYS_INT_Initialize must have been called.

  Parameters:
    None.

  Returns:
    true	- Global Interrupts are enabled (before the call to disable)
    false	- Global Interrupts are disabled (before the call to disable)

  Example:
    <code>
    // Interrupt enable status
    bool flag;
    // Disable the global interrupts
    flag = SYS_INT_Disable();
    // Do something critical
    // Check if interrupts were disabled/enabled
    if (flag)
    {
        // enable the global interrupts if they were enabled before the 
        // call to SYS_INT_Disable()
        SYS_INT_Enable();
    }
    </code>

  Remarks:
    None.
*/

bool SYS_INT_Disable ( void );


/*******************************************************************************
  Function:
    bool SYS_INT_IsEnabled ( void )

  Summary:
    Identifies if interrupts are currently enabled or disabled at the top level.
    
  Description:
    This function identifies if interrupts are enabled or disabled at the top
    level.

  Precondition:
    SYS_INT_Initialize must have been called.

  Parameters:
    None.

  Returns:
    - true	- If the interrupts are currently enabled
    - false	- If the interrupts are currently disabled

  Example:
    <code>
    // Check if global interrupts are enabled
    if ( SYS_INT_IsEnabled() )
    {
        // Interrupt enable status
        bool flag;
        // Disable the global interrupts.
        flag = SYS_INT_Disable();
    }
    </code>

  Remarks:
    None.
*/

bool SYS_INT_IsEnabled ( void );


// *****************************************************************************
// *****************************************************************************
// Section: Interrupt Source Management Routines
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_INT_SourceEnable ( INT_SOURCE source )

  Summary:
    Enables the specified source to generate interrupts to the processor.
    
  Description:
    This routine enables the specified source to generate interrupts to the 
    processor when events occur.

  Precondition:
    SYS_INT_Initialize must have been called and an ISR must have been
    registered for the source.

  Parameters:
    source	- Identifier for the desired interrupt source

  Returns:
    None.

  Example:
    <code>
    // Initialize the interrupt system. This needs to done in the initialization
    // code.
    SYS_INT_Initialize();
    // Enable the interrupt source
    SYS_INT_SourceEnable(INT_SOURCE_PARALLEL_PORT);
    </code>

  Remarks:
    An Interrupt Service Routine (ISR) for the given interrupt source must be 
    ready to receive the call before the source is enabled.
*/

void SYS_INT_SourceEnable ( INT_SOURCE source );


/*******************************************************************************
  Function:
    bool SYS_INT_SourceDisable ( INT_SOURCE source )

  Summary:
    Disables the specified source from generating interrupts to the processor.
    
  Description:
    This routine disables the given source from generating interrupts the 
    processor when events occur.It returns the interrupt source enable/disable 
	status before disabling the interrupt source.

  Precondition:
    SYS_INT_Initialize must have been called.

  Parameters:
    source	- Identifier for the desired interrupt source.

  Returns:
    - true	- The Interrupt source is enabled (before the call to SYS_INT_SourceDisable)
    - false	- The Interrupt source is disabled (before the call to SYS_INT_SourceDisable)

  Example:
    <code>
    // interrupt source enable/disable status.
	bool flag
	// Initialize the interrupt system.This needs to done in the initialization
    // code.
    SYS_INT_Initialize();
    // Disable the interrupt source
    flag = SYS_INT_SourceDisable(INT_SOURCE_PARALLEL_PORT);
	// before enabling the source check the enable/disable status
	if(flag)
	{
		SYS_INT_SourceEnable(INT_SOURCE_PARALLEL_PORT);
	}
    </code>

  Remarks:
    None.
*/

bool SYS_INT_SourceDisable ( INT_SOURCE source );


/*******************************************************************************
  Function:
    bool SYS_INT_SourceIsEnabled ( INT_SOURCE source )

  Summary:
    Identifies if the specified source is enabled or disabled.
    
  Description:
    This function identifies if the specified source is currently enabled or is
    currently disabled.

  Precondition:
    SYS_INT_Initialize must have been called.

  Parameters:
    source	- Identifier for the desired interrupt source.

  Returns:
    - true	- If the given source is currently enabled.
    - false	- If the given source is currently disabled.

  Example:
    <code>
    // Initialize the interrupt system. This needs to done in the initialization
    // code.
    SYS_INT_Initialize();
    // Check if the required interrupt source is enabled
    if ( SYS_INT_SourceIsEnabled(INT_SOURCE_PARALLEL_PORT))
    {
        // App code
    }
    </code>

  Remarks:
    None.
*/

bool SYS_INT_SourceIsEnabled ( INT_SOURCE source );


/*******************************************************************************
  Function:
    bool SYS_INT_SourceStatusGet ( INT_SOURCE source )

  Summary:
    Determines the status of the specified interrupt source.
    
  Description:
    This function determines the current status of the interrupt source.

  Precondition:
    SYS_INT_Initialize must have been called.

  Parameters:
    source	- Identifier for the desired interrupt source

  Returns:
    - true	- If the given interrupt source is currently set
    - false	- If the given interrupt source is not currently set

  Example:
    <code>
    // Initialize the interrupt system.This needs to done in the initialization
    // code.
    SYS_INT_Initialize();
    // Check if the required interrupt source is set
    if ( SYS_INT_SourceStatusGet(INT_SOURCE_PARALLEL_PORT) )
    {
        // Handle interrupt
    }
    </code>

  Remarks:
    Works even if the interrupt source or interrupts in general have not been
    enabled, so it can be used for polling implementations.
*/

bool SYS_INT_SourceStatusGet ( INT_SOURCE source );


/*******************************************************************************
  Function:
    void SYS_INT_SourceStatusClear ( INT_SOURCE source )

  Summary:
    Clears the interrupt request for the specified interrupt source.
    
  Description:
    This function clears the interrupt request for the specified interrupt source.

  Precondition:
    SYS_INT_Initialize must have been called.

  Parameters:
    source	- Identifier for the desired interrupt source

  Returns:
    None.

  Example:
    <code>
    // Initialize the interrupt system. This needs to done in the initialization
    // code.
    SYS_INT_Initialize();
    // Check if the interrupt source flag is set
    if ( SYS_INT_SourceStatusGet(INT_SOURCE_PARALLEL_PORT) )
    {
        // Clear the interrupt flag
        SYS_INT_SourceStatusClear(INT_SOURCE_PARALLEL_PORT);
    }
    </code>

  Remarks:
    None.
*/

void SYS_INT_SourceStatusClear ( INT_SOURCE source );


/*******************************************************************************
  Function:
    void SYS_INT_SourceStatusSet ( INT_SOURCE source )

  Summary:
    Sets the specified interrupt source.
    
  Description:
    This function sets the specified interrupt source, causing the processor to be
    interrupted if interrupts are enabled, the source has been enabled, and 
    the priority is higher than the current priority.

  Precondition:
    SYS_INT_Initialize must have been called and an ISR must have been
    registered for the source (if interrupts and the source are enabled).

  Parameters:
    source	- Identifier for the desired interrupt source

  Returns:
    None.

  Example:
    <code>
    // Initialize the interrupt system. This needs to done in the initialization
    // code.
    SYS_INT_Initialize();
    // Check if interrupt source flag is set 
    if ( !SYS_INT_SourceStatusGet(INT_SOURCE_PARALLEL_PORT) )
    {
        // Set the interrupt source flag
        SYS_INT_SourceStatusSet(INT_SOURCE_PARALLEL_PORT);
    }
    </code>

  Remarks:
    Not supported for all interrupt sources. Check the specific data sheet for software 
    clear only interrupt sources.
*/

void SYS_INT_SourceStatusSet ( INT_SOURCE source );

#include "system/int/sys_int_mapping.h"

#endif // _SYS_INT_H

