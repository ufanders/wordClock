/*******************************************************************************
  Oscillator Peripheral Library Interface Header

  Company:
    Microchip Technology Inc.

  File Name:
    plib_osc.h

  Summary:
    Defines the Oscillator (OSC) Peripheral Library interface.

  Description:
    This header file contains the function prototypes and definitions of
    the data types and constants that make up the interface to the Oscillator
    (OSC) Peripheral Library for Microchip microcontrollers.  The definitions
    in this file are for the Oscillator module.
*******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END

#ifndef _PLIB_OSC_H
#define _PLIB_OSC_H

// *****************************************************************************
// *****************************************************************************
// Section: Include files
// *****************************************************************************
// *****************************************************************************

#include "peripheral/osc/processor/osc_processor.h"


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - General Setup
// *****************************************************************************
// *****************************************************************************
/*  APIs general configuration of the oscillator module */

//******************************************************************************
/* Function:
    void PLIB_OSC_OnWaitActionSet ( OSC_MODULE_ID index,
    				    OSC_OPERATION_ON_WAIT onWait )

  Summary:
    Selects the operation to be performed when a WAIT instruction is executed.

  Description:
    This function selects the operation to be performed when a WAIT
    instruction is executed.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
    onWait     - Operation to be performed when a WAIT instruction is
                 executed. One of the possible values of OSC_OPERATION_ON_WAIT.

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_OnWaitActionSet(OSC_ID_0, OSC_ON_WAIT_SLEEP);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsOnWaitAction
	in your application to determine whether this feature is available.

        If this API is not called, the device will enter Idle mode on execution of a
        WAIT instruction.
*/

void PLIB_OSC_OnWaitActionSet ( OSC_MODULE_ID index,
								OSC_OPERATION_ON_WAIT onWait );


//******************************************************************************
/* Function:
    OSC_OPERATION_ON_WAIT PLIB_OSC_OnWaitActionGet ( OSC_MODULE_ID index )

  Summary:
    Gets the configured operation to be performed when a WAIT instruction is
    executed.

  Description:
    This function gets the configured operation that is to be performed when a
    WAIT instruction is executed.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    On a WAIT action, one of the possible values of OSC_OPERATION_ON_WAIT.

  Example:
    <code>
    if (PLIB_OSC_OnWaitActionGet(OSC_ID_0) == OSC_ON_WAIT_SLEEP)
    {
		//Do some action
	}
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsOnWaitAction
	in your application to determine whether this feature is available.
*/

OSC_OPERATION_ON_WAIT PLIB_OSC_OnWaitActionGet ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_OSC_StartupTimerHasExpired ( OSC_MODULE_ID index )

  Summary:
    Returns 'true' if the oscillator start-up time-out timer has expired.

  Description:
    This function returns 'true' if the oscillator start-up time-out timer has
    expired.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    - true    - Oscillator start-up time-out timer has expired
    - false   - Oscillator start-up time-out timer is running

  Example:
    <code>
    bool startUpTImer;

    startUpTImer = PLIB_OSC_StartupTimerHasExpired(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
        specific device data sheet to determine availability.
*/

bool PLIB_OSC_StartupTimerHasExpired ( OSC_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Primary Oscillator Setup
// *****************************************************************************
// *****************************************************************************
/*  APIs for configuring Primary Oscillator */

//******************************************************************************
/* Function:
    void PLIB_OSC_PrimaryOscInSleepModeEnable  ( OSC_MODULE_ID index )

  Summary:
    The Primary Oscillator continues to operate during Sleep mode.

  Description:
    This function enables the Primary Oscillator during Sleep mode.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_PrimaryOscInSleepModeEnable(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
        specific device data sheet to determine availability.
 */

void PLIB_OSC_PrimaryOscInSleepModeEnable ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_PrimaryOscInSleepModeDisable ( OSC_MODULE_ID index )

  Summary:
    The Primary Oscillator is disabled during Sleep mode.

  Description:
    This function disables the Primary Oscillator during Sleep mode.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_PrimaryOscInSleepModeDisable(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
        specific device data sheet to determine availability.
 */

void PLIB_OSC_PrimaryOscInSleepModeDisable( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_OSC_PrimaryOscInSleepModeIsEnabled ( OSC_MODULE_ID index )

  Summary:
    Returns 'true' if the Primary Oscillator is disabled during Sleep mode.

  Description:
    This function gets the enable status of the Primary Oscillator during Sleep
    mode.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    - true   -   Primary Oscillator is enabled during Sleep mode
    - false  -   Primary Oscillator is disabled during Sleep mode


  Example:
    <code>
    bool priOscStatus;

    priOscStatus = PLIB_OSC_PrimaryOscInSleepModeIsEnabled(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
        specific device data sheet to determine availability.
 */

bool PLIB_OSC_PrimaryOscInSleepModeIsEnabled( OSC_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Secondary Oscillator Setup
// *****************************************************************************
// *****************************************************************************
/*  APIs for configuring Secondary Oscillator */

//******************************************************************************
/* Function:
    void PLIB_OSC_SecondaryEnable ( OSC_MODULE_ID index )

  Summary:
    Enables the Secondary Oscillator.

  Description:
    This function enables the Secondary Oscillator.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_SecondaryEnable(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsSecondaryEnable
	in your application to determine whether this feature is available.
*/

void PLIB_OSC_SecondaryEnable ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_SecondaryDisable ( OSC_MODULE_ID index );

  Summary:
    Disables the Secondary Oscillator.

  Description:
    This function disables the Secondary Oscillator.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_SecondaryDisable(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsSecondaryEnable
	in your application to determine whether this feature is available.
*/

void PLIB_OSC_SecondaryDisable ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_OSC_SecondaryIsEnabled ( OSC_MODULE_ID index );

  Summary:
    Returns 'true' if the Secondary Oscillator is enabled.

  Description:
    This function returns 'true' if the Secondary Oscillator is enabled.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    - true    - Secondary Oscillator is enabled
    - false   - Secondary Oscillator is disabled

  Example:
    <code>
    bool secOscEnable;

    secOscEnable = PLIB_OSC_SecondaryIsEnabled(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsSecondaryEnable
	in your application to determine whether this feature is available.
*/

bool PLIB_OSC_SecondaryIsEnabled ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_OSC_SecondaryIsReady ( OSC_MODULE_ID index )

  Summary:
    Returns 'true' if the Secondary Oscillator is ready.

  Description:
    This function returns the ready status of the Secondary Oscillator.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    - true    - Indicates that the Secondary Oscillator is running and is stable
    - false   - Secondary Oscillator is either turned off or is still warming up

  Example:
    <code>
    bool secOscReady;

    secOscReady = PLIB_OSC_SecondaryIsReady(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsSecondaryReady
	in your application to determine whether this feature is available.
 */

bool PLIB_OSC_SecondaryIsReady( OSC_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Auxiliary Oscillator Setup
// *****************************************************************************
// *****************************************************************************
/*  APIs for configuring Auxiliary Oscillator */

//******************************************************************************
/* Function:
    void PLIB_OSC_AuxModeSelect ( OSC_MODULE_ID index,
    				  OSC_AUX_MODE auxOscMode )

  Summary:
    Selects the Auxiliary Oscillator mode.

  Description:
    This function selects the Auxiliary Oscillator mode based on the input.

  Precondition:
    None.

  Parameters:
  	index  - Identifies the desired oscillator module
    auxOscMode - One of the possible values of OSC_AUX_MODE

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_AuxModeSelect(OSC_ID_0, OSC_AUX_MODE_HS);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability.
 */

void PLIB_OSC_AuxModeSelect ( OSC_MODULE_ID index,
				OSC_AUX_MODE AuxOscMode );


//******************************************************************************
/* Function:
    OSC_AUX_MODE PLIB_OSC_AuxModeGet ( OSC_MODULE_ID index )

  Summary:
    Gets the selected Auxiliary Oscillator mode.

  Description:
    This function gets the Auxiliary Oscillator mode.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    One of the possible values of OSC_AUX_MODE.


  Example:
    <code>
    OSC_AUX_MODE auxOscMode;

    auxOscMode = PLIB_OSC_AuxModeGet(OSC_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability.
 */

OSC_AUX_MODE PLIB_OSC_AuxModeGet ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_AuxClockSourceSet ( OSC_MODULE_ID index,
    				      OSC_AUX_CLOCK_SOURCE auxClockSource )

  Summary:
    Sets the clock source for the auxiliary clock divisor.

  Description:
    This function sets the clock source for the auxiliary clock divisor.

  Precondition:
    None.

  Parameters:
  	index    - Identifies the desired oscillator module
  	auxClockSource  - Clock source for the auxiliary clock divisor. One of
                          the possible values from OSC_AUX_CLOCK_SOURCE.

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_AuxClockSourceSet(OSC_ID_0, OSC_AUX_CLOCK_PRIMARY);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

void PLIB_OSC_AuxClockSourceSet ( OSC_MODULE_ID index,
				  OSC_AUX_CLOCK_SOURCE auxClockSource );


//******************************************************************************
/* Function:
    OSC_AUX_CLOCK_SOURCE PLIB_OSC_AuxClockSourceGet ( OSC_MODULE_ID index )

  Summary:
    Gets the clock source for the auxiliary clock.

  Description:
    This function gets the clock source for the auxiliary clock.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    One of the possible values from OSC_AUX_CLOCK_SOURCE.

  Example:
    <code>
    OSC_AUX_CLOCK_SOURCE auxClockSource;

    auxClockSource = PLIB_OSC_AuxClockSourceGet(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

OSC_AUX_CLOCK_SOURCE PLIB_OSC_AuxClockSourceGet ( OSC_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Reference Oscillator Setup
// *****************************************************************************
// *****************************************************************************
/*  APIs for configuring Reference Oscillator */

//******************************************************************************
/* Function:
    void PLIB_OSC_ReferenceOscEnable ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc )

  Summary:
    Enables the reference oscillator.

  Description:
    This function enables the reference oscillator.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
	referenceOsc - Identifies the desired reference oscillator

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_ReferenceOscEnable(OSC_ID_0, OSC_REFERENCE_1);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsReferenceOscEnable
	in your application to determine whether this feature is available.

	If the device has a reference clock output enable control, calling this function
	may not give the reference clock output. Use the PLIB_OSC_ReferenceOutputEnable
	function to enable the output.

 */

void PLIB_OSC_ReferenceOscEnable ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc );


//******************************************************************************
/* Function:
    void PLIB_OSC_ReferenceOscDisable ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc )

  Summary:
    Disables the reference oscillator output.

  Description:
    This function disables output from the reference oscillator.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
	referenceOsc - Identifies the desired reference oscillator

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_ReferenceOscDisable(OSC_ID_0, OSC_REFERENCE_1);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsReferenceOscEnable
	in your application to determine whether this feature is available.
 */

void PLIB_OSC_ReferenceOscDisable ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc );


//******************************************************************************
/* Function:
    bool PLIB_OSC_ReferenceOscIsEnabled ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc )

  Summary:
    Gets the enable status of the reference oscillator output.

  Description:
    This function gets the enable status of the reference oscillator output.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
	referenceOsc - Identifies the desired reference oscillator

  Returns:
    - true   -   Reference oscillator is enabled
    - false  -   Reference oscillator is disabled

  Example:
    <code>
    if(PLIB_OSC_ReferenceOscIsEnabled(OSC_ID_0, OSC_REFERENCE_1))
    {
	//Do some action
    }
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsReferenceOscEnable
	in your application to determine whether this feature is available.
 */

bool PLIB_OSC_ReferenceOscIsEnabled ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc );


//******************************************************************************
/* Function:
    void PLIB_OSC_ReferenceOscStopInSleepEnable ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc )

  Summary:
    Configures the reference oscillator to stop operating in Sleep mode.

  Description:
    This function configures the reference oscillator to stop operating in
    Sleep mode.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
	referenceOsc - Identifies the desired reference oscillator

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_ReferenceOscStopInSleepEnable(OSC_ID_0, OSC_REFERENCE_1);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability or use PLIB_OSC_ExistsReferenceOscStopInSleep
    in your application to determine whether this feature is available.

    The default state of the device is for the reference oscillator to be
    enabled in Sleep mode.
 */

void PLIB_OSC_ReferenceOscStopInSleepEnable ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc );


//******************************************************************************
/* Function:
    void PLIB_OSC_ReferenceOscStopInSleepDisable ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc )

  Summary:
    Enables the reference oscillator in Sleep mode.

  Description:
    This function enables the reference oscillator in Sleep mode. The reference
    oscillator continues to run in Sleep mode.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
	referenceOsc - Identifies the desired reference oscillator

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_ReferenceOsctopInSleepDisable(OSC_ID_0, OSC_REFERENCE_1);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability or use PLIB_OSC_ExistsReferenceOscStopInSleep
    in your application to determine whether this feature is available.

    The reference clock output will be stopped in Sleep mode if the base clock
    selected is 'System Clock' or 'Peripheral Clock' regardless of this function.

    The default state of the device is for the reference oscillator to be
    enabled in Sleep mode. Therefore, calling this function is necessary only if the
    PLIB_OSC_ReferenceOscStopInSleepEnable function was previously called, and
    the software wants to enable oscillator operation in Sleep mode.
 */

void PLIB_OSC_ReferenceOscStopInSleepDisable ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc );


//******************************************************************************
/* Function:
    bool PLIB_OSC_ReferenceOscStopInSleepIsEnabled ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc )

  Summary:
    Returns 'true' if the reference oscillator is disabled in Sleep mode.

  Description:
    This function returns 'true' if the reference oscillator is disabled in
    Sleep mode.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
	referenceOsc - Identifies the desired reference oscillator

  Returns:
    - true   -   Reference oscillator is disabled in Sleep mode
    - false  -   Reference oscillator is enabled in Sleep mode

  Example:
    <code>
    bool refOscSleep;

    refOscSleep = PLIB_OSC_ReferenceOscStopInSleepIsEnabled(OSC_ID_0, OSC_REFERENCE_1);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsReferenceOscStopInSleep
	in your application to determine whether this feature is available.
 */

bool PLIB_OSC_ReferenceOscStopInSleepIsEnabled ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc );


//******************************************************************************
/* Function:
    void PLIB_OSC_ReferenceOscStopInIdleEnable ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc )

  Summary:
    Configures the reference oscillator to stop operating in Idle mode.

  Description:
    This function configures the reference oscillator to stop operating in
    Idle mode.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
	referenceOsc - Identifies the desired reference oscillator

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_ReferenceOscStopInIdleEnable(OSC_ID_0, OSC_REFERENCE_1);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability or use PLIB_OSC_ExistsReferenceOscStopInIdleEnable
    in your application to determine whether this feature is available.

    The default state of the device is reference oscillator enabled in Idle
    mode. Therefore, calling this function is necessary only if the
    PLIB_OSC_ReferenceOscStopInIdleDisable function was previously called, and
    the software wants to enable oscillator operation in Sleep mode.
 */

void PLIB_OSC_ReferenceOscStopInIdleEnable ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc );


//******************************************************************************
/* Function:
    void PLIB_OSC_ReferenceOscStopInIdleDisable ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc )

  Summary:
    Enables the reference oscillator in Idle mode.

  Description:
    This function enables the reference oscillator in Idle mode. The reference
    oscillator continues to run in Idle mode.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
	referenceOsc - Identifies the desired reference oscillator

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_ReferenceOsctopInSleepDisable(OSC_ID_0, OSC_REFERENCE_1);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability or use PLIB_OSC_ExistsReferenceOscStopInIdleEnable
    in your application to determine whether this feature is available.

    The default state of the device is the reference oscillator is enabled in
    Idle mode.
 */

void PLIB_OSC_ReferenceOscStopInIdleDisable ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc );


//******************************************************************************
/* Function:
    bool PLIB_OSC_ReferenceOscStopInIdleIsEnabled ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc )

  Summary:
    Returns 'true' if the reference oscillator is disabled in Idle mode.

  Description:
    This function returns 'true' if the reference oscillator is disabled in
    Idle mode.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
	referenceOsc - Identifies the desired reference oscillator

  Returns:
    - true   -   Reference oscillator is disabled in Idle mode
    - false  -   Reference oscillator is enabled in Idle mode

  Example:
    <code>
    bool refOscIdle;

    refOscIdle = PLIB_OSC_ReferenceOscStopInIdleIsEnabled(OSC_ID_0, OSC_REFERENCE_1);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsReferenceOscStopInIdleEnable
	in your application to determine whether this feature is available.
 */

bool PLIB_OSC_ReferenceOscStopInIdleIsEnabled ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc );


//******************************************************************************
/* Function:
    void PLIB_OSC_ReferenceOutputEnable ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc )

  Summary:
    Enables the reference oscillator output.

  Description:
    This function enables the reference oscillator output.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
	referenceOsc - Identifies the desired reference oscillator

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_ReferenceOutputEnable(OSC_ID_0, OSC_REFERENCE_1);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability or use PLIB_OSC_ExistsReferenceOutputEnable
    in your application to determine whether this feature is available.

    The default state of the device is reference oscillator output disabled.
 */

void PLIB_OSC_ReferenceOutputEnable ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc );


//******************************************************************************
/* Function:
    void PLIB_OSC_ReferenceOutputDisable ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc )

  Summary:
    Disables the reference oscillator output.

  Description:
    This function disables the reference oscillator output.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
	referenceOsc - Identifies the desired reference oscillator

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_ReferenceOutputDisable(OSC_ID_0, OSC_REFERENCE_1);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability or use PLIB_OSC_ExistsReferenceOutputEnable
    in your application to determine whether this feature is available.

    The default state of the device is reference oscillator output disabled.
 */

void PLIB_OSC_ReferenceOutputDisable ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc );


//******************************************************************************
/* Function:
    bool PLIB_OSC_ReferenceOutputIsEnabled ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc )

  Summary:
    Returns 'true' if the reference oscillator output is enabled.

  Description:
    This function returns 'true' if the reference oscillator output is enabled.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
	referenceOsc - Identifies the desired reference oscillator

  Returns:
    - true   -   Reference oscillator output is enabled
    - false  -   Reference oscillator output is disabled

  Example:
    <code>
    bool refOscIdle;

    refOscIdle = PLIB_OSC_ReferenceOutputIsEnabled(OSC_ID_0, OSC_REFERENCE_1);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsReferenceOutputEnable
	in your application to determine whether this feature is available.
 */

bool PLIB_OSC_ReferenceOutputIsEnabled ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc );


//******************************************************************************
/* Function:
    bool PLIB_OSC_ReferenceOscSourceChangeIsActive ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc )

  Summary:
    Returns 'true' if a reference oscillator source change request is active.

  Description:
    This function returns 'true' if the reference oscillator source change
    is in progress. The software is not allowed to give a new source change
    request.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
	referenceOsc - Identifies the desired reference oscillator

  Returns:
    - true   -   Reference oscillator change request is active
    - false  -   Reference oscillator change request is not active

  Example:
    <code>
    if (!PLIB_OSC_ReferenceOscSourceChangeIsActive(OSC_ID_0, OSC_REFERENCE_1))
    {
        //Allowed to change the reference clock source
    }
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsReferenceOscChangeActive
	in your application to determine whether this feature is available.
 */

bool PLIB_OSC_ReferenceOscSourceChangeIsActive ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc );


//******************************************************************************
/* Function:
    void PLIB_OSC_ReferenceOscBaseClockSelect ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc,
    						OSC_REF_BASECLOCK refOscBaseClock )

  Summary:
    Sets the base clock for the reference oscillator.

  Description:
    This function sets the base clock for the reference oscillator. There are
    multiple clock sources by which the user can configure the module to output
    to the pin. Users can check the accuracy of the clock by probing the pin.
    Use the PLIB_OSC_ReferenceOscDivisorValueSet function to divide the clock if
    it is a very high value.

  Precondition:
    None.

  Parameters:
    index      	  - Identifies the desired oscillator module
	referenceOsc - Identifies the desired reference oscillator
    refOscBaseClk - One of the possible values of OSC_REF_BASECLOCK

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_ReferenceOscBaseClockSelect(OSC_ID_0, OSC_REFERENCE_1, OSC_REF_BASECLOCK_PBCLK);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsReferenceOscBaseClock
	in your application to determine whether this feature is available.
 */

void PLIB_OSC_ReferenceOscBaseClockSelect ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc, OSC_REF_BASECLOCK refOscBaseClock );


//******************************************************************************
/* Function:
    bool PLIB_OSC_ReferenceOscSwitchIsComplete ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc )

  Summary:
    Returns 'true' if the reference oscillator base clock switching is complete.

  Description:
    This function returns 'true' if the reference oscillator base clock
    switching is complete.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
	referenceOsc - Identifies the desired reference oscillator

  Returns:
    - true   -   Reference clock base clock switching is complete
    - false  -   Reference clock base clock switching is not complete; switching
	         is not started

  Example:
    <code>
    bool refOscIdle;

    refOscIdle = PLIB_OSC_ReferenceOscSwitchIsComplete(OSC_ID_0, OSC_REFERENCE_1);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsReferenceOscChange
	in your application to determine whether this feature is available.
 */

bool PLIB_OSC_ReferenceOscSwitchIsComplete ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc );


//******************************************************************************
/* Function:
    void PLIB_OSC_ReferenceOscDivisorValueSet ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc,
    						OSC_REF_DIVISOR_TYPE refOscDivValue )

  Summary:
    Selects the reference oscillator divisor value.

  Description:
    This function selects the reference oscillator divisor value.

  Precondition:
    None.

  Parameters:
  	index    -       Identifies the desired oscillator module
	referenceOsc -   Identifies the desired reference oscillator
    refOscDivValue - Value for Reference Oscillator Divisor (RODIV) field.
                     If it is 0, then divider is not used.

  Returns:
    None.

  Example:
    <code>
    // Select the clock source.
    PLIB_OSC_ReferenceOscBaseClockSelect(OSC_ID_0, OSC_REFERENCE_1, OSC_REF_BASECLOCK_PBCLK);

    PLIB_OSC_ReferenceOscDivisorValueSet(OSC_ID_0, OSC_REFERENCE_1, 128);

    PLIB_OSC_ReferenceOscEnable(OSC_ID_0, OSC_REFERENCE_1);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsReferenceOscDivisor
	in your application to determine whether this feature is available.

	The value entered may not be the actual divisor. Please refer to the
	specific device data sheet to determine the actual divisor corresponding
        to the value entered.

 */

void PLIB_OSC_ReferenceOscDivisorValueSet ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc,
					    OSC_REF_DIVISOR_TYPE refOscDivValue );


//******************************************************************************
/* Function:
    void PLIB_OSC_ReferenceOscTrimSet ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc,
    					OSC_REF_TRIM_TYPE trimValue )

  Summary:
    Sets the reference oscillator divisor trim value.

  Description:
    This function selects the reference oscillator divisor trim value. The
    value selected divided by OSC_REF_TRIM_MAX_VALUE will be added to
    the oscillator divisor value.

  Precondition:
    None.

  Parameters:
  	index  - Identifies the desired oscillator module
	referenceOsc - Identifies the desired reference oscillator
    trimValue  - Reference oscillator trim value

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_ReferenceOscTrimSet(OSC_ID_0, OSC_REFERENCE_1, 50);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsReferenceOscTrim
	in your application to determine whether this feature is available.
 */

void PLIB_OSC_ReferenceOscTrimSet ( OSC_MODULE_ID index, OSC_REFERENCE referenceOsc,
                                    OSC_REF_TRIM_TYPE trimValue );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Fast RC Oscillator Setup
// *****************************************************************************
// *****************************************************************************
/*  APIs for configuring RC Oscillator */

//******************************************************************************
/* Function:
    void PLIB_OSC_FRCTuningModeSet ( OSC_MODULE_ID index,
                                     OSC_TUNING_MODE tuningMode )

  Summary:
    Selects the FRC oscillator tuning mode.

  Description:
    This function selects the FRC oscillator tuning mode.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
    tuningMode - One of the possible value from OSC_TUNING_MODE.

  Returns:
    None.

  Example:
    <code>

    PLIB_OSC_FRCTuningModeSet(OSC_ID_0, OSC_TUNING_USING_NUMBER);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability or use PLIB_OSC_ExistsFRCTuning
    in your application to determine whether this feature is available.

 */

void PLIB_OSC_FRCTuningModeSet ( OSC_MODULE_ID index,
                                 OSC_TUNING_MODE tuningMode );


//******************************************************************************
/* Function:
    OSC_TUNING_MODE PLIB_OSC_FRCTuningModeGet ( OSC_MODULE_ID index )

  Summary:
    Gets the FRC oscillator tuning mode.

  Description:
    This function gets the FRC oscillator tuning mode.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    One of the possible value from OSC_TUNING_MODE enum.

  Example:
    <code>

    if (OSC_TUNING_USING_NUMBER == PLIB_OSC_FRCTuningModeGet(OSC_ID_0))
    {
        //Do some action
    }
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.

 */

OSC_TUNING_MODE PLIB_OSC_FRCTuningModeGet ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_FRCDivisorSelect ( OSC_MODULE_ID index,
                                     OSC_FRC_DIV divisorFRC )

  Summary:
    Sets the FRC clock divisor to the specified value.

  Description:
    This function sets the FRC clock divisor to the specified value.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
    divisorFRC - One of the possible values from OSC_FRC_DIV

  Returns:
    None.


  Example:
    <code>
    PLIB_OSC_FRCDivisorSelect ( OSC_ID_0, OSC_FRC_DIV_4 );
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsFRCDivisor
	in your application to determine whether this feature is available.
 */

void PLIB_OSC_FRCDivisorSelect( OSC_MODULE_ID index,
				OSC_FRC_DIV divisorFRC );


//******************************************************************************
/* Function:
    uint16_t PLIB_OSC_FRCDivisorGet( OSC_MODULE_ID index )

  Summary:
    Gets the FRC clock divisor.

  Description:
    This function gets the FRC clock divisor. The value returned will be direct
    number and not enum equivalent.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    FRC divisor value.


  Example:
    <code>
    uint16_t divisorFRC;

    divisorFRC = PLIB_OSC_FRCDivisorGet(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsFRCDivisor
	in your application to determine whether this feature is available.
*/

uint16_t PLIB_OSC_FRCDivisorGet( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_FRCDitherEnable ( OSC_MODULE_ID index )

  Summary:
    Enables the FRC Oscillator clock dithering.

  Description:
    This function enables the FRC Oscillator clock dithering.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_FRCDitherEnable(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

void PLIB_OSC_FRCDitherEnable ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_FRCTuningSequenceValueSet ( OSC_MODULE_ID index,
                                              OSC_TUNING_SEQUENCERS oscSeqencer,
                                              OSC_FRC_TUNE_TYPE seqValue )

  Summary:
    Sets the value in the FRC tune sequencer.

  Description:
    This function sets the value in the FRC tune sequencer.

  Precondition:
    None.

  Parameters:
    index       - Identifies the desired oscillator module
    oscSeqencer	- Select the sequencer
    seqValue	- Value to be set in the sequencer. One of the possible
                  values from OSC_FRC_TUNE_TYPE.

  Returns:
    None.

  Example:
    <code>

    PLIB_OSC_FRCTuningModeSet(OSC_ID_0, OSC_TUNING_SEQ_DITHER);

    PLIB_OSC_FRCTuningSelect(OSC_ID_0, 0x01);

    PLIB_OSC_FRCTuningSequenceValueSet(OSC_ID_0, OSC_TUNING_SEQUENCER_1,
    	OSC_FRC_TUNE_MINUS_2_25_PERCENT);

    PLIB_OSC_FRCTuningSequenceValueSet(OSC_ID_0, OSC_TUNING_SEQUENCER_2,
    	OSC_FRC_TUNE_MINUS_1_5_PERCENT);

    PLIB_OSC_FRCTuningSequenceValueSet(OSC_ID_0, OSC_TUNING_SEQUENCER_3,
    	OSC_FRC_TUNE_MINUS_0_375_PERCENT);

    PLIB_OSC_FRCTuningSequenceValueSet(OSC_ID_0, OSC_TUNING_SEQUENCER_4,
    	OSC_FRC_TUNE_PLUS_0_43_PERCENT);

    PLIB_OSC_FRCTuningSequenceValueSet(OSC_ID_0, OSC_TUNING_SEQUENCER_5,
    	OSC_FRC_TUNE_PLUS_1_29_PERCENT);

    PLIB_OSC_FRCTuningSequenceValueSet(OSC_ID_0, OSC_TUNING_SEQUENCER_6,
    	OSC_FRC_TUNE_PLUS_2_54_PERCENT);

    PLIB_OSC_FRCTuningSequenceValueSet(OSC_ID_0, OSC_TUNING_SEQUENCER_7,
    	OSC_FRC_TUNE_MINUS_3_PERCENT);

    //Configure PWM (period, pulse width and turn on the module)

    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

void PLIB_OSC_FRCTuningSequenceValueSet ( OSC_MODULE_ID index,
                                          OSC_TUNING_SEQUENCERS oscSeqencer,
                                          OSC_FRC_TUNE_TYPE seqValue );


//******************************************************************************
/* Function:
    OSC_FRC_TUNE_TYPE PLIB_OSC_FRCTuningSequenceValueGet (
                                            OSC_MODULE_ID index,
                                            OSC_TUNING_SEQUENCERS oscSeqencer )

  Summary:
    Gets the value set in the FRC tune sequencer.

  Description:
    This function gets the value set in the selected FRC tune sequencer.

  Precondition:
    None.

  Parameters:
  	index   - Identifies the desired oscillator module
    oscSeqencer	- Value to be set in the sequencer

  Returns:
    Value in the sequencer specified. One of the possible value from OSC_FRC_TUNE_TYPE.

  Example:
    <code>
    OSC_FRC_TUNE_TYPE seqValue;

    seqValue = PLIB_OSC_FRCTuningSequenceValueGet(OSC_ID_0,
                OSC_TUNING_SEQUENCER_1);

    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability.

 */

OSC_FRC_TUNE_TYPE PLIB_OSC_FRCTuningSequenceValueGet (
                                        OSC_MODULE_ID index,
                                        OSC_TUNING_SEQUENCERS oscSeqencer );


//******************************************************************************
/* Function:
    void PLIB_OSC_FRCTuningSelect ( OSC_MODULE_ID index,
                                    OSC_FRC_TUNE_TYPE tuningValue )

  Summary:
    Sets the FRC tuning value.

  Description:
    This function tunes the FRC oscillator to the value specified. The
    application is supposed to try different values and find the best one.
    If the device has different tuning modes, this function will be used
    differently. See the example provided for the PLIB_OSC_FRCTuningSequenceValueSet
    function for details.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
    tuningValue - Tuning value. One of the possible values from OSC_FRC_TUNE_TYPE.

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_FRCTuningSelect(OSC_ID_0, 0x05);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability or use PLIB_OSC_ExistsFRCTuning
    in your application to determine whether this feature is available.

 */

void PLIB_OSC_FRCTuningSelect ( OSC_MODULE_ID index,
				OSC_FRC_TUNE_TYPE tuningValue );


//******************************************************************************
/* Function:
    void PLIB_OSC_LinearFeedbackShiftRegSet ( OSC_MODULE_ID index,
                                              OSC_LFSR_TYPE linearFbValue )

  Summary:
    Sets the linear feedback shift register.

  Description:
    This function sets the linear feedback shift register. When the FRC oscillator
    tuning mode is configured for pseudo-random number-based dithering, this
    function is used to provide the seed number for the pseudo-random number
    generation algorithm.

  Precondition:
    None.

  Parameters:
  	index      - Identifies the desired oscillator module
    linearFbValue  - The pseudo-random FRC trim value

  Returns:
    None.

  Example:
    <code>
	PLIB_OSC_FRCTuningModeSet(OSC_ID_0, OSC_TUNING_PSEUDO_RANDOM);

    //15 bit Linear Feedback shift register value

    PLIB_OSC_LinearFeedbackShiftRegSet(OSC_ID_0, 0x7F);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability.

 */

void PLIB_OSC_LinearFeedbackShiftRegSet ( OSC_MODULE_ID index,
                                          OSC_LFSR_TYPE linearFbValue );


//******************************************************************************
/* Function:
    OSC_LFSR_TYPE PLIB_OSC_LinearFeedbackShiftRegGet ( OSC_MODULE_ID index )

  Summary:
    Gets the value from linear feedback shift register.

  Description:
    This function gets the value from the linear feedback shift register.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    OSC_LFSR_TYPE	- Linear Feedback shift register

  Example:
    <code>
	OSC_LFSR_TYPE linearFbValue;

    linearFbValue = PLIB_OSC_LinearFeedbackShiftRegGet(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

OSC_LFSR_TYPE PLIB_OSC_LinearFeedbackShiftRegGet ( OSC_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Oscillator Switch Setup
// *****************************************************************************
// *****************************************************************************
/*  APIs for configuring oscillator Switch */

//*************************************************************************
/*  Function:
       void PLIB_OSC_SysClockSelect ( OSC_MODULE_ID index,
                                       OSC_SYS_TYPE newOsc )

  Summary:
    Selects the new oscillator.

  Description:
    This function selects the new oscillator.

  Conditions:
    None.

  Input:
    index -   Identifies the desired oscillator module
    newOsc -  One of the possible values from OSC_SYS_TYPE

  Return:
    None.

  Example:
    <code>

    PLIB_OSC_SysClockSelect(OSC_ID_0, OSC_PRIMARY);

    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability or use PLIB_OSC_ExistsOscSelect
    in your application to determine whether this feature is available.

    This function adds the necessary delay (NOP instructions) after switching
    the oscillator. Therefore, the user need not add any delay as specified
    in the device data sheet.
  *************************************************************************/

void PLIB_OSC_SysClockSelect ( OSC_MODULE_ID index,
		       OSC_SYS_TYPE newOsc );


//******************************************************************************
/* Function:
    void PLIB_OSC_ClockSwitchingAbort ( OSC_MODULE_ID index )

  Summary:
    Aborts an oscillator switch.

  Description:
    This function aborts the oscillator switch to the selection specified by the
    new oscillator selection bits.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_ClockSwitchingAbort(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsOscSwitchInit
	in your application to determine whether this feature is available.
*/

void PLIB_OSC_ClockSwitchingAbort ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_OSC_ClockSwitchingIsComplete( OSC_MODULE_ID index );

  Summary:
    Gets the oscillator switch progress status.

  Description:
    This function gets the status of the oscillator switch progress.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    true    - Oscillator switch is complete
    false   - Oscillator switch is in progress

  Example:
    <code>
    PLIB_OSC_SysClockSelect(OSC_ID_0, OSC_PRIMARY);

    while(!PLIB_OSC_ClockSwitchingIsComplete(OSC_ID_0));

    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability or use PLIB_OSC_ExistsOscSwitchInit
    in your application to determine whether this feature is available.

*/

bool PLIB_OSC_ClockSwitchingIsComplete ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    OSC_SYS_TYPE PLIB_OSC_CurrentSysClockGet ( OSC_MODULE_ID index )

  Summary:
    Gets the current oscillator selected.

  Description:
    This function gets the current oscillator. If the application hasn't changed
    the oscillator selection, this will be same as the oscillator selected
    through the Configuration bits.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    One of the possible values from OSC_SYS_TYPE.

  Example:
    <code>
    OSC_SYS_TYPE oscCurrent;

    oscCurrent = PLIB_OSC_CurrentSysClockGet(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsOscCurrentGet
	in your application to determine whether this feature is available.
*/

OSC_SYS_TYPE PLIB_OSC_CurrentSysClockGet ( OSC_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Doze Setup
// *****************************************************************************
// *****************************************************************************
/*  APIs for Doze Setup. Doze is a method to reduce power
    consumption By reducing the processor clock */

//******************************************************************************
/* Function:
    void PLIB_OSC_DozeModeEnable ( OSC_MODULE_ID index )

  Summary:
    Enables Doze mode.

  Description:
    This function enables Doze mode. This is a method to reduce power
    consumption By reducing the processor clock. Use the PLIB_OSC_DozeRatioSelect
    function to specify the CPU peripheral clock ratio.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    None.

  Example:
    <code>


    PLIB_OSC_DozeModeEnable(OSC_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability.

    This function adds the necessary delay (NOP instructions), which must be
    added immediately before entering Doze mode and immediately after exiting
    Doze mode.
 */

void PLIB_OSC_DozeModeEnable ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_DozeModeDisable ( OSC_MODULE_ID index )

  Summary:
    Disables Doze mode.

  Description:
    This function disables Doze mode. The CPU peripheral clock ratio is set to
    1:1.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_DozeModeDisable(OSC_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability.

    This function adds the necessary delay (NOP instructions), which must be
    added immediately before entering Doze mode and immediately after exiting
    Doze mode.
 */

void PLIB_OSC_DozeModeDisable ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_DozeRatioSelect ( OSC_MODULE_ID index,
                                    OSC_DOZE_RATIO dozeRatio )

  Summary:
    Selects the Doze ratio of the processor clock to the peripheral clock.

  Description:
    This function selects the Doze ratio, which is the ratio between the
    processor clock and the peripheral clock. By default, the ratio between
    the processor clock and the peripheral clock is 1:1.

  Precondition:
    None.

  Parameters:
  	index  - Identifies the desired oscillator module
    dozeRatio   - One of the possible values of OSC_DOZE_RATIO

  Returns:
    None.


  Example:
    <code>
    PLIB_OSC_DozeRatioSelect(OSC_ID_0, OSC_DOZE_RATIO_4);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

void PLIB_OSC_DozeRatioSelect ( OSC_MODULE_ID index,
				OSC_DOZE_RATIO dozeRatio );


//******************************************************************************
/* Function:
    OSC_DOZE_RATIO PLIB_OSC_DozeRatioGet ( OSC_MODULE_ID index )

  Summary:
    Gets the ratio of the processor clock to the peripheral clock.

  Description:
    This function gets the Doze ratio, which is the ratio between the processor
    clock and the peripheral clock.

  Precondition:
    None.

  Parameters:
     index      - Identifies the desired oscillator module

  Returns:
    One of the possible values of OSC_DOZE_RATIO.


  Example:
    <code>
    OSC_DOZE_RATIO dozeRatio;

    dozeRatio = PLIB_OSC_DozeRatioGet(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

OSC_DOZE_RATIO PLIB_OSC_DozeRatioGet ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_DozeRecoverOnInterruptEnable ( OSC_MODULE_ID index )

  Summary:
    Enables recovery from Doze mode upon an interrupt.

  Description:
    This function enables the feature of recovering from Doze mode upon an
    interrupt. Interrupts clear the power-saving Doze mode enable and reset the
    CPU peripheral clock ratio to 1:1.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    None.


  Example:
    <code>
    PLIB_OSC_DozeRecoverOnInterruptEnable(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

void PLIB_OSC_DozeRecoverOnInterruptEnable ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_RecoverDozeOnInterruptDisable ( OSC_MODULE_ID index )

  Summary:
    Disables the recovery from Doze mode upon an interrupt.

  Description:
    This function disables feature of recovering from Doze mode upon an
    interrupt. Interrupts will not affect the power-saving Doze mode enable.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    None.


  Example:
    <code>
    PLIB_OSC_DozeRecoverOnInterruptDisable(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

void PLIB_OSC_DozeRecoverOnInterruptDisable ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_OSC_DozeRecoverOnInterruptIsEnabled ( OSC_MODULE_ID index )

  Summary:
    Returns 'true' if recover from Doze mode on interrupt is enabled.

  Description:
    This function returns 'true' if recover from Doze mode on interrupt is
    enabled.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    - true   -   Recover from Doze mode on interrupt is enabled
    - false  -   Recover from Doze mode on interrupt is disabled

  Example:
    <code>
    bool recoverOnInt;

    recoverOnInt = PLIB_OSC_DozeRecoverOnInterruptIsEnabled(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

bool PLIB_OSC_DozeRecoverOnInterruptIsEnabled ( OSC_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - USB and Display Clock Setup
// *****************************************************************************
// *****************************************************************************
/*  APIs for USB and Display clock */

//******************************************************************************
/* Function:
    void PLIB_OSC_UsbClockSourceSelect ( OSC_MODULE_ID index,
    					 OSC_USBCLOCK_SOURCE usbClock )

  Summary:
    Sets the USB module clock source.

  Description:
    This function sets the USB module clock source.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
    usbClock   - Select the USB module clock source. One of the possible values
		 from OSC_USBCLOCK_SOURCE.
  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_UsbClockSourceSelect(OSC_ID_0, SYS_OSC_USBCLK_FRC);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability or use PLIB_OSC_ExistsUsbClockSource
    in your application to determine whether this feature is available.

    Before placing the USB module in Suspend mode, use this function to enable the
    FRC clock.
*/

void PLIB_OSC_UsbClockSourceSelect ( OSC_MODULE_ID index,
                                     OSC_USBCLOCK_SOURCE usbClock );


//******************************************************************************
/* Function:
    OSC_USBCLOCK_SOURCE PLIB_OSC_UsbClockSourceGet ( OSC_MODULE_ID index )

  Summary:
    Gets the USB module clock source.

  Description:
    This function gets the USB module clock source.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    USB module clock source. One of the possible values from OSC_USBCLOCK_SOURCE.

  Example:
    <code>
    if (SYS_OSC_USBCLK_FRC == PLIB_OSC_UsbClockSourceGet(OSC_ID_0))
    {
	//Do some action
    }
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsUsbClockSource
	in your application to determine whether this feature is available.
*/

OSC_USBCLOCK_SOURCE PLIB_OSC_UsbClockSourceGet ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_GraphicsClockDivisorSet ( OSC_MODULE_ID index,
                                            uint8_t clockDivisor )

  Summary:
    Sets the Graphics module clock divisor.

  Description:
    This function sets the Graphics module clock divisor.

  Precondition:
    None.

  Parameters:
  	index     - Identifies the desired oscillator module
    clockDivisor  - Graphics module clock divisor

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_GraphicsClockDivisorSet(OSC_ID_0, 127);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

void PLIB_OSC_GraphicsClockDivisorSet ( OSC_MODULE_ID index,
					uint8_t clockDivisor );


//******************************************************************************
/* Function:
    uint8_t PLIB_OSC_GraphicsClockDivisorGet ( OSC_MODULE_ID index )

  Summary:
    Gets the Graphics module clock divisor.

  Description:
    This function gets the Graphics module clock divisor.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    Graphics module clock divisor.

  Example:
    <code>
    uint8_t   displayClockDiv;

    displayClockDiv = PLIB_OSC_GraphicsClockDivisorGet(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

uint8_t PLIB_OSC_GraphicsClockDivisorGet ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_GraphicsClockSourceSelect ( OSC_MODULE_ID index,
                                              OSC_GFX_CLOCK gfxClock  )

  Summary:
    Sets the Graphics Controller module clock.

  Description:
    This function sets the clock frequency for the Graphics Controller module.

  Precondition:
    None.

  Parameters:
  	index  - Identifies the desired oscillator module
    gfxClock   - One of the possible values of OSC_GFX_CLOCK

  Returns:
    None.


  Example:
    <code>
    PLIB_OSC_GraphicsClockSourceSelect (OSC_ID_0, OSC_GFX_CLOCK_96M);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

void PLIB_OSC_GraphicsClockSourceSelect ( OSC_MODULE_ID index,
                                          OSC_GFX_CLOCK gfxClock  );


//******************************************************************************
/* Function:
    OSC_GFX_CLOCK PLIB_OSC_GraphicsClockSourceGet ( OSC_MODULE_ID index )

  Summary:
    Gets the Graphics Controller module clock frequency value.

  Description:
    This function gets the clock frequency used for the Graphics Controller
    module.

  Precondition:
    None.

  Parameters:
  	index      - Identifies the desired oscillator module

  Returns:
    None.


  Example:
    <code>
    OSC_GFX_CLOCK gfxClock;

    gfxClock = PLIB_OSC_GraphicsClockSourceGet (OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

OSC_GFX_CLOCK PLIB_OSC_GraphicsClockSourceGet ( OSC_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - PLL Setup
// *****************************************************************************
// *****************************************************************************
/*  APIs for PLL Setup */

//******************************************************************************
/* Function:
    bool PLIB_OSC_PLLIsLocked ( OSC_MODULE_ID index, OSC_PLL_SELECT pllselect )

  Summary:
    Returns 'true' if the selected PLL is locked.

  Description:
    This function returns the status of the PLL lock.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
    pllselect  - Selects the PLL.

  Returns:
    - true    - PLL module is in lock or PLL module start-up timer is satisfied
    - false   - PLL module is out of lock, PLL start-up timer is running, or PLL is
                disabled

  Example:
    <code>
    bool lockPLL_status;

    lockPLL_status = PLIB_OSC_PLLIsLocked(OSC_ID_0, OSC_PLL_SYSTEM);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability or use PLIB_OSC_ExistsPLLLockStatus
    in your application to determine whether this feature is available.

    If the PLL does not stabilize properly during start-up, this function may not
    reflect the actual status of PLL lock, nor does it detect when the PLL
    loses lock during normal operation.
*/

bool PLIB_OSC_PLLIsLocked ( OSC_MODULE_ID index, OSC_PLL_SELECT pllselect );


//******************************************************************************
/* Function:
    void PLIB_OSC_PLLClockLock ( OSC_MODULE_ID index )

  Summary:
    Locks the clock and PLL selections.

  Description:
    This function locks the clock and PLL selections.

  Precondition:
    The Fail-Safe Clock Monitor (FSCM) should be enabled.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_PLLClockLock(OSC_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability or use PLIB_OSC_ExistsPLLClockLock
    in your application to determine whether this feature is available.

    The data given by this API is valid only if clock switching and monitoring
    is enabled. Otherwise Clock and PLL selections are never locked and may
    be modified.
*/

void PLIB_OSC_PLLClockLock ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_PLLClockUnlock ( OSC_MODULE_ID index )

  Summary:
    Unlocks the clock and PLL selections.

  Description:
    This function unlocks the clock and PLL selection so that the clock and PLL
    may be modified.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_PLLClockUnlock(OSC_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability or use PLIB_OSC_ExistsPLLClockLock
    in your application to determine whether this feature is available.

*/

void PLIB_OSC_PLLClockUnlock ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_OSC_PLLClockIsLocked ( OSC_MODULE_ID index )

  Summary:
    Gets the lock status for the clock and PLL selections.

  Description:
    This function gets the lock status for the clock and PLL selections.

  Precondition:
    None.

  Parameters:
    index     - Identifies the desired oscillator module

  Returns:
    - true    - clock and PLL selections are locked
    - false   - clock and PLL selections are not locked

  Example:
    <code>
    bool clockPLL_st;

    clockPLL_st = PLIB_OSC_PLLClockIsLocked(OSC_ID_0);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the specific
    device data sheet to determine availability or use PLIB_OSC_ExistsPLLClockLock
    in your application to determine whether this feature is available.

    If the PLL does not stabilize properly during start-up, this function may not
    reflect the actual status of PLL lock, nor does it detect when the PLL
    loses lock during normal operation.
*/

bool PLIB_OSC_PLLClockIsLocked ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_SysPLLMultiplierSelect ( OSC_MODULE_ID index,
                                        OSC_SYSPLL_MULTIPLIER_TYPE pll_multiplier )

  Summary:
    Sets the PLL multiplier to the specified value.

  Description:
    This function sets the PLL multiplier to the specified value.

  Precondition:
    None.

  Parameters:
  	index      - Identifies the desired oscillator module
    pll_multiplier  - One of the possible values of PB clock divisor
					  of type OSC_PLL_MUL_TYPE

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_SysPLLMultiplierSelect (OSC_ID_0, 0x08);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsSysPLLMultiplier
	in your application to determine whether this feature is available.

	Note: Use PLL Multiplier value directly for the parameter 'pll_multiplier', NOT the
	value of the 'PLLMULT' field. Library behavior is undefined in case of used
	PLL Multiplier value is not supported by the selected device.
	Refer the data sheet of the device for the detail.
 */

void PLIB_OSC_SysPLLMultiplierSelect( OSC_MODULE_ID index,
                                   OSC_SYSPLL_MULTIPLIER_TYPE pll_multiplier );


//******************************************************************************
/* Function:
    OSC_SYSPLL_MULTIPLIER_TYPE PLIB_OSC_SysPLLMultiplierGet ( OSC_MODULE_ID index )

  Summary:
    Gets the PLL multiplier.

  Description:
    This function gets the PLL multiplier.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    One of the possible values of PB clock divisor of type OSC_SYSPLL_MULTIPLIER_TYPE.

  Example:
    <code>
    OSC_SYSPLL_MULTIPLIER_TYPE pll_multiply;

    pll_multiply = PLIB_OSC_SysPLLMultiplierGet(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsSysPLLMultiplier
	in your application to determine whether this feature is available.

	Note: Actual Multiplier value will be returned, NOT the 'PLLMULT' field value.
		  Refer the data sheet of the device for the detail.
 */

OSC_SYSPLL_MULTIPLIER_TYPE PLIB_OSC_SysPLLMultiplierGet( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_SysPLLOutputDivisorSet ( OSC_MODULE_ID index,
                                           OSC_SYSPLL_OUT_DIV PLLOutDiv )

  Summary:
    Sets the output divider for the PLL to the specified value.

  Description:
    This function sets the output divider for the PLL to the specified value.

  Precondition:
    None.

  Parameters:
  	index      - Identifies the desired oscillator module
    PLLOutDiv - One of the possible values from OSC_SYSPLL_OUT_DIV

  Returns:
    None.


  Example:
    <code>
    PLIB_OSC_SysPLLOutputDivisorSelect(OSC_ID_0, OSC_SYSPLL_OUT_DIV_1);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsSysPLLOutputDivisor
	in your application to determine whether this feature is available.
 */

void PLIB_OSC_SysPLLOutputDivisorSet ( OSC_MODULE_ID index,
                                       OSC_SYSPLL_OUT_DIV PLLOutDiv );


//******************************************************************************
/* Function:
    uint16_t PLIB_OSC_SysPLLOutputDivisorGet ( OSC_MODULE_ID index )

  Summary:
    Gets the output divisor for the PLL.

  Description:
    This function gets the output divisor for the System PLL. The value is the
    actual divisor and not the enum value corresponding to it.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    System PLL output divisor value.


  Example:
    <code>
    uint16_t pllOutDiv;

    pllOutDiv = PLIB_OSC_SysPLLOutputDivisorGet(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsSysPLLOutputDivisor
	in your application to determine whether this feature is available.
 */

uint16_t PLIB_OSC_SysPLLOutputDivisorGet ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_SysPLLInputDivisorSet ( OSC_MODULE_ID index,
                                           uint16_t PLLInDiv )

  Summary:
    Sets the input divider for the PLL to the specified value.

  Description:
    This function sets the input divider for the PLL to the specified value.

  Precondition:
    None.

  Parameters:
  	index      - Identifies the desired oscillator module
    PLLInDiv - 	 System PLL input divisor value

  Returns:
    None.


  Example:
    <code>
    PLIB_OSC_SysPLLInputDivisorSet( OSC_ID_0, 3 );
    </code>

  Remarks:
  	Pass the direct divisor instead of the register value equivalent.

	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsSysPLLInputDivisor
	in your application to determine whether this feature is available.
 */

void PLIB_OSC_SysPLLInputDivisorSet ( OSC_MODULE_ID index,
                                       uint16_t PLLInDiv );


//******************************************************************************
/* Function:
    uint16_t PLIB_OSC_SysPLLInputDivisorGet ( OSC_MODULE_ID index )

  Summary:
    Gets the input divisor for the PLL.

  Description:
    This function gets the input divisor for the PLL.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    Gets the System PLL input divisor as number.


  Example:
    <code>
    uint16_t pllInDiv;

    PLLInDiv = PLIB_OSC_SysPLLInputDivisorGet(OSC_ID_0);
    </code>

  Remarks:
  	Gets the direct register value itself instead of the register value
  	equivalent.

	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsSysPLLInputDivisor
	in your application to determine whether this feature is available.
 */

uint16_t PLIB_OSC_SysPLLInputDivisorGet( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_SysPLLInputClockSourceSet ( OSC_MODULE_ID index,
                                           OSC_SYSPLL_IN_CLK_SOURCE PLLInClockSource )

  Summary:
    Sets the input clock source for the PLL module

  Description:
    This function sets the input clock source for the PLL module

  Precondition:
    None.

  Parameters:
  	index      - Identifies the desired oscillator module
    PLLInClockSource - One of the possible values from OSC_SYSPLL_IN_CLK_SOURCE

  Returns:
    None.


  Example:
    <code>
    PLIB_OSC_SysPLLInputClockSourceSet(OSC_ID_0, OSC_SYSPLL_IN_CLK_SOURCE_FRC);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsSysPLLInputClockSource
	in your application to determine whether this feature is available.
 */

void PLIB_OSC_SysPLLInputClockSourceSet ( OSC_MODULE_ID index,
                                       OSC_SYSPLL_IN_CLK_SOURCE PLLInClockSource );


//******************************************************************************
/* Function:
    OSC_SYSPLL_IN_CLK_SOURCE PLIB_OSC_SysPLLInputClockSourceGet ( OSC_MODULE_ID index )

  Summary:
    Returns the input clock source for the PLL module

  Description:
    This function returns the input clock source for the PLL module

  Precondition:
    None.

  Parameters:
  	index      - Identifies the desired oscillator module


  Returns:
    PLLInClockSource - One of the possible values from OSC_SYSPLL_IN_CLK_SOURCE


  Example:
    <code>
	OSC_SYSPLL_IN_CLK_SOURCE PLLInClockSource;
    PLLInClockSource = PLIB_OSC_SysPLLInputClockSourceGet(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsSysPLLInputClockSource
	in your application to determine whether this feature is available.
 */

OSC_SYSPLL_IN_CLK_SOURCE PLIB_OSC_SysPLLInputClockSourceGet ( OSC_MODULE_ID index);


//******************************************************************************
/* Function:
    void PLIB_OSC_SysPLLFrequencyRangeSet ( OSC_MODULE_ID index,
                                           OSC_SYSPLL_FREQ_RANGE PLLFrequencyRange )

  Summary:
    Sets the frequency range for PLL

  Description:
    This function sets the frequency range for PLL

  Precondition:
    None.

  Parameters:
  	index      - Identifies the desired oscillator module
    PLLFrequencyRange - One of the possible values from OSC_SYSPLL_FREQ_RANGE

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_SysPLLFrequencyRangeSet(OSC_ID_0, OSC_SYSPLL_FREQ_RANGE_5M_TO_10M);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsSysPLLFrequencyRange
	in your application to determine whether this feature is available.
 */

void PLIB_OSC_SysPLLFrequencyRangeSet ( OSC_MODULE_ID index,
                                       OSC_SYSPLL_FREQ_RANGE PLLFrequencyRange );


//******************************************************************************
/* Function:
    OSC_SYSPLL_FREQ_RANGE PLIB_OSC_SysPLLFrequencyRangeGet ( OSC_MODULE_ID index )

  Summary:
    Returns the frequency range for PLL

  Description:
    This function returns the frequency range set for PLL

  Precondition:
    None.

  Parameters:
  	index      - Identifies the desired oscillator module

  Returns:
        PLLFrequencyRange - One of the possible values from OSC_SYSPLL_FREQ_RANGE

  Example:
    <code>
	OSC_SYSPLL_FREQ_RANGE PLLFrequencyRange;
    PLLFrequencyRange = PLIB_OSC_SysPLLFrequencyRangeGet(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsSysPLLFrequencyRange
	in your application to determine whether this feature is available.
 */

OSC_SYSPLL_FREQ_RANGE PLIB_OSC_SysPLLFrequencyRangeGet ( OSC_MODULE_ID index );

//******************************************************************************
/* Function:
    void PLIB_OSC_PLLAuxOutputDivisorSet ( OSC_MODULE_ID index,
                                              OSC_AUXPLL_OUT_DIV auxOutputDiv )

  Summary:
    Sets the output divisor for the Auxiliary PLL to the specified value.

  Description:
    This function sets the output divisor for the Auxiliary PLL to the specified
    value.

  Precondition:
    None.

  Parameters:
  	index      	   - Identifies the desired oscillator module
    auxOutputDiv   - One of the possible values from OSC_AUXPLL_OUT_DIV

  Returns:
    None.


  Example:
    <code>
    PLIB_OSC_PLLAuxOutputDivisorSet(OSC_ID_0, OSC_AUXPLL_OUT_DIV_2);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

void PLIB_OSC_PLLAuxOutputDivisorSet( OSC_MODULE_ID index,
                                         OSC_AUXPLL_OUT_DIV auxOutputDiv );


//******************************************************************************
/* Function:
    OSC_AUXPLL_OUT_DIV PLIB_OSC_PLLAuxOutputDivisorGet ( OSC_MODULE_ID index )

  Summary:
    Gets the output divisor for the Auxiliary PLL.

  Description:
    This function gets the output divisor for the Auxiliary PLL.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    One of the possible values from OSC_AUXPLL_OUT_DIV.


  Example:
    <code>
    OSC_AUXPLL_OUT_DIV auxOutputDiv;

    auxOutputDiv = PLIB_OSC_PLLAuxOutputDivisorGet(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

OSC_AUXPLL_OUT_DIV PLIB_OSC_PLLAuxOutputDivisorGet( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_PLLAuxInputDivisorSelect ( OSC_MODULE_ID index,
                                             OSC_AUXPLL_IN_DIV auxPLLInDiv )

  Summary:
    Sets the input divisor for the Auxiliary PLL to the specified value.

  Description:
    This function sets the input divisor for the Auxiliary PLL to the
    specified value.

  Precondition:
    None.

  Parameters:
  	index      	- Identifies the desired oscillator module
    auxPLLInDiv - One of the possible values from OSC_AUXPLL_IN_DIV

  Returns:
    None.


  Example:
    <code>
    PLIB_OSC_PLLAuxInputDivisorSelect(OSC_ID_0, OSC_AUXPLL_IN_DIV_2);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

void PLIB_OSC_PLLAuxInputDivisorSelect( OSC_MODULE_ID index,
					OSC_AUXPLL_IN_DIV auxPLLInDiv );


//******************************************************************************
/* Function:
    OSC_AUXPLL_IN_DIV PLIB_OSC_PLLAuxInputDivisorGet ( OSC_MODULE_ID index )

  Summary:
    Gets the input divisor for the Auxiliary PLL.

  Description:
    This function gets the input divisor for the Auxiliary PLL.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    One of the possible values from OSC_AUXPLL_IN_DIV.


  Example:
    <code>
    OSC_AUXPLL_IN_DIV auxPLLInDiv;

    auxPLLInDiv = PLIB_OSC_PLLAuxInputDivisorGet(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

OSC_AUXPLL_IN_DIV PLIB_OSC_PLLAuxInputDivisorGet( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_PLLAuxClockSourceSelect ( OSC_MODULE_ID index,
                                            OSC_PLLAUX_CLOCK_SOURCE auxPLLClkSrc )

  Summary:
    Sets the Auxiliary PLL clock source.

  Description:
    This function sets the Auxiliary PLL clock source to either the Primary
    oscillator or the Auxiliary oscillator depending on the input.

  Precondition:
    None.

  Parameters:
    index      	 - Identifies the desired oscillator module

    auxPLLClkSrc - The source for the auxiliary clock. One of the possible
    				values from OSC_PLLAUX_CLOCK_SOURCE.

  Returns:
    None.


  Example:
    <code>
    PLIB_OSC_PLLAuxClockSourceSelect(OSC_ID_0, OSC_PLLAUX_CLOCK);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

void PLIB_OSC_PLLAuxClockSourceSelect ( OSC_MODULE_ID index,
					OSC_PLLAUX_CLOCK_SOURCE auxPLLClkSrc );


//******************************************************************************
/* Function:
    OSC_PLLAUX_CLOCK_SOURCE PLIB_OSC_PLLAuxClockSourceGet ( OSC_MODULE_ID index )

  Summary:
    Returns the clock source selected for the Auxiliary PLL.

  Description:
    This function returns the clock source selected for the Auxiliary PLL.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    The source for the auxiliary clock. One of the possible values from
    OSC_PLLAUX_CLOCK_SOURCE.

  Example:
    <code>
    if(OSC_PLLAUX_CLOCK == PLIB_OSC_PLLAuxClockSourceGet(OSC_ID_0))
    {
        //Take some action
    }
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

OSC_PLLAUX_CLOCK_SOURCE PLIB_OSC_PLLAuxClockSourceGet( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    bool PLIB_OSC_PLLAuxIsLocked ( OSC_MODULE_ID index )

  Summary:
    Returns whether the Auxiliary PLL is locked.

  Description:
    This function returns 'true' if the Auxiliary PLL is locked.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    - true   -   Auxiliary PLL is in lock
    - false  -   Auxiliary PLL is not in lock

  Example:
    <code>
    bool lockAuxPLL;

    lockAuxPLL = PLIB_OSC_PLLAuxIsLocked(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

bool PLIB_OSC_PLLAuxIsLocked ( OSC_MODULE_ID index );


//******************************************************************************
/* Function:
    void PLIB_OSC_PLLEnable ( OSC_MODULE_ID index, OSC_PLL_SELECT selectPLL )

  Summary:
    Enables the specified PLL.

  Description:
    This function enables the the specified PLL.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
    selectPLL  - PLL to be enabled. One of the possible values from
                 OSC_PLL_SELECT.

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_PLLEnable(OSC_ID_0, OSC_PLL_SYSTEM);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

void PLIB_OSC_PLLEnable ( OSC_MODULE_ID index, OSC_PLL_SELECT selectPLL );


//******************************************************************************
/* Function:
    void PLIB_OSC_PLLDisable ( OSC_MODULE_ID index,
                               OSC_PLL_SELECT selectPLL )

  Summary:
    Disables the specified PLL.

  Description:
    This function disables the the specified PLL.


  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
    selectPLL  - PLL to be enabled. One of the possible values from OSC_PLL_SELECT.

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_PLLDisable(OSC_ID_0, OSC_PLL_SYSTEM);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

void PLIB_OSC_PLLDisable ( OSC_MODULE_ID index,
                           OSC_PLL_SELECT selectPLL );


//******************************************************************************
/* Function:
    bool PLIB_OSC_PLLIsEnabled ( OSC_MODULE_ID index,
    				 OSC_PLL_SELECT selectPLL )

  Summary:
    Returns 'true' if the specified PLL is enabled.

  Description:
    This function returns 'true' if the specified PLL is enabled.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
    selectPLL  - PLL to be enabled. One of the possible values from OSC_PLL_SELECT.

  Returns:
    - true   -   The specified PLL is enabled
    - false  -   The specified PLL is disabled

  Example:
    <code>
    if (PLIB_OSC_PLLIsEnabled(OSC_ID_0, OSC_PLL_SYSTEM))
    {
        //Do some action
    }
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

bool PLIB_OSC_PLLIsEnabled ( OSC_MODULE_ID index,
                             OSC_PLL_SELECT selectPLL );


//******************************************************************************
/* Function:
    void PLIB_OSC_PLLAuxMultiplierSelect ( OSC_MODULE_ID index,
                                            OSC_AUXPLL_MULTIPLIER auxPllMultiplier )

  Summary:
    Sets the Auxiliary PLL multiplier to the specified value.

  Description:
    This function sets the Auxiliary PLL multiplier to the specified value.

  Precondition:
    None.

  Parameters:
    index      	      - Identifies the desired oscillator module
    auxPllMultiplier  - One of the possible values from OSC_AUXPLL_MULTIPLIER

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_PLLAuxMultiplierSelect (OSC_ID_0, OSC_AUXPLL_MULTIPLIER_4);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

void PLIB_OSC_PLLAuxMultiplierSelect( OSC_MODULE_ID index,
                                      OSC_AUXPLL_MULTIPLIER auxPllMultiplier );


//******************************************************************************
/* Function:
    OSC_AUXPLL_MULTIPLIER PLIB_OSC_PLLAuxMultiplierGet ( OSC_MODULE_ID index )

  Summary:
    Gets the Auxiliary PLL multiplier.

  Description:
    This function gets the Auxiliary PLL multiplier.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    One of the possible values from OSC_PLL_MUL.

  Example:
    <code>
    OSC_AUXPLL_MULTIPLIER auxpllMultiply;

    auxpllMultiply = PLIB_OSC_PLLAuxMultiplierGet(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability.
 */

OSC_AUXPLL_MULTIPLIER PLIB_OSC_PLLAuxMultiplierGet( OSC_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Peripheral Bus Clock Setup
// *****************************************************************************
// *****************************************************************************
/*  APIs for Peripheral Bus clock Setup */

//******************************************************************************
/* Function:
    bool PLIB_OSC_PBClockDivisorIsReady( OSC_MODULE_ID index,
										OSC_PERIPHERAL_BUS peripheralBusNumber )

  Summary:
    Checks whether the peripheral bus clock divisor is ready to be written.

  Description:
    This function checks whether the peripheral bus clock divisor is ready to
    be written.

  Precondition:
    None.

  Parameters:
    index  - Identifies the desired oscillator module
	peripheralBusNumber - Identifies the desired peripheral bus

  Returns:
    - true - Peripheral bus clock divisor can be written
    - false - Peripheral bus clock divisor cannot be written

  Example:
    <code>
    if (PLIB_OSC_PBClockDivisorIsReady(OSC_ID_0, OSC_PERIPHERAL_BUS_1))
    {
        PLIB_OSC_PBClockDivisorSet (OSC_ID_0, OSC_PERIPHERAL_BUS_1,
									0x01);
    }
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsPBClockReady
	in your application to determine whether this feature is available.
 */

bool PLIB_OSC_PBClockDivisorIsReady( OSC_MODULE_ID index,
									OSC_PERIPHERAL_BUS peripheralBusNumber );


//******************************************************************************
/* Function:
    void PLIB_OSC_PBClockDivisorSet( OSC_MODULE_ID index,
									 OSC_PERIPHERAL_BUS peripheralBusNumber,
                                     OSC_PB_CLOCK_DIV_TYPE peripheralBusClkDiv )

  Summary:
    Sets the peripheral bus clock divisor to the specified value.

  Description:
    This function sets the peripheral bus clock divisor to the specified value.

  Precondition:
    Peripheral bus clock divisor should be ready to be written.

  Parameters:
    index  	- Identifies the desired oscillator module
	peripheralBusNumber - Identifies the desired peripheral bus
    peripheralBusClkDiv - One of the possible values of PB clock divisor
						  of type OSC_PB_CLOCK_DIV_TYPE

  Returns:
    None.

  Example:
    <code>
    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, OSC_PERIPHERAL_BUS_1, 0x01);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsPBClockDivisor
	in your application to determine whether this feature is available.

	Note: Use PB Divider value directly for the parameter 'peripheralBusClkDiv', NOT the
	value of the 'PBDIV' field. Library behavior is undefined in case of used
	PB Divider value is not supported by the selected device.
	Refer the data sheet of the device for the detail.
 */

void PLIB_OSC_PBClockDivisorSet( OSC_MODULE_ID index,
								 OSC_PERIPHERAL_BUS peripheralBusNumber,
                                 OSC_PB_CLOCK_DIV_TYPE peripheralBusClkDiv );


//******************************************************************************
/* Function:
    OSC_PB_CLOCK_DIV_TYPE PLIB_OSC_PBClockDivisorGet ( OSC_MODULE_ID index,
										OSC_PERIPHERAL_BUS peripheralBusNumber )

  Summary:
    Gets the peripheral bus clock divisor.

  Description:
    This function gets the peripheral bus clock divisor.

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module
	peripheralBusNumber - Identifies the desired peripheral bus

  Returns:
    One of the possible values of PB clock divisor of type OSC_PB_CLOCK_DIV_TYPE.

  Example:
    <code>
    OSC_PB_CLOCK_DIV_TYPE peripheralBusClkDiv;

    peripheralBusClkDiv = PLIB_OSC_PBClockDivisorGet(OSC_ID_0,
													OSC_PERIPHERAL_BUS_1);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsPBClockDivisor
	in your application to determine whether this feature is available.

	Note: Actual Divisor value will be returned, NOT the 'PBDIV' field value.
		  Refer the data sheet of the device for the detail.
 */

OSC_PB_CLOCK_DIV_TYPE PLIB_OSC_PBClockDivisorGet( OSC_MODULE_ID index,
										OSC_PERIPHERAL_BUS peripheralBusNumber );


//******************************************************************************
/* Function:
    void PLIB_OSC_PBOutputClockEnable( OSC_MODULE_ID index,
										OSC_PERIPHERAL_BUS peripheralBusNumber )

  Summary:
    Enables the peripheral bus output clock

  Description:
    This function enables the peripheral bus output clock

  Precondition:
    None.

  Parameters:
    index  - Identifies the desired oscillator module
	peripheralBusNumber - Identifies the desired peripheral bus

  Returns:
    None

  Example:
    <code>

        PLIB_OSC_PBOutputClockEnable (OSC_ID_0, OSC_PERIPHERAL_BUS_2);

    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsPBClockOutputEnable
	in your application to determine whether this feature is available.
 */

void PLIB_OSC_PBOutputClockEnable( OSC_MODULE_ID index,
									OSC_PERIPHERAL_BUS peripheralBusNumber );


//******************************************************************************
/* Function:
    void PLIB_OSC_PBOutputClockDisable( OSC_MODULE_ID index,
										OSC_PERIPHERAL_BUS peripheralBusNumber )

  Summary:
    Disables the peripheral bus output clock

  Description:
    This function disables the peripheral bus output clock

  Precondition:
    None.

  Parameters:
    index  - Identifies the desired oscillator module
	peripheralBusNumber - Identifies the desired peripheral bus

  Returns:
    None

  Example:
    <code>

        PLIB_OSC_PBOutputClockDisable (OSC_ID_0, OSC_PERIPHERAL_BUS_2);

    </code>

  Remarks:
	The clock for peripheral bus 1 cannot be turned off.

	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsPBClockOutputEnable
	in your application to determine whether this feature is available.
 */

void PLIB_OSC_PBOutputClockDisable( OSC_MODULE_ID index,
									OSC_PERIPHERAL_BUS peripheralBusNumber );


//******************************************************************************
/* Function:
    bool PLIB_OSC_PBOutputClockIsEnabled( OSC_MODULE_ID index,
										OSC_PERIPHERAL_BUS peripheralBusNumber )

  Summary:
    Checks whether the peripheral bus clock output is enabled or not

  Description:
    This function checks whether the peripheral bus clock output is enabled or not

  Precondition:
    None.

  Parameters:
    index  - Identifies the desired oscillator module
	peripheralBusNumber - Identifies the desired peripheral bus

  Returns:
    - true - Peripheral bus clock output is enabled
    - false - Peripheral bus clock output is disabled

  Example:
    <code>
    if (PLIB_OSC_PBOutputClockIsEnabled(OSC_ID_0, OSC_PERIPHERAL_BUS_2))
    {
        PLIB_OSC_PBOutputClockDisable(OSC_ID_0, OSC_PERIPHERAL_BUS_2);
    }
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsPBClockOutputEnable
	in your application to determine whether this feature is available.
 */

bool PLIB_OSC_PBOutputClockIsEnabled( OSC_MODULE_ID index,
									OSC_PERIPHERAL_BUS peripheralBusNumber );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Clock Fail Monitoring
// *****************************************************************************
// *****************************************************************************
/*  APIs for Clock Fail Monitoring */

//*******************************************************************************
/*  Function:
    bool PLIB_OSC_ClockHasFailed ( OSC_MODULE_ID index );

  Summary:
    Returns 'true' if the clock fails.

  Description:
    This function returns 'true' if the clock fails. Monitors the Fail-Safe Clock
    Monitor (FSCM).

  Precondition:
    None.

  Parameters:
    index      - Identifies the desired oscillator module

  Returns:
    - true    - Fail-Safe Clock Monitor (FSCM) detected a clock failure
    - false   - No clock failure has been detected

  Example:
    <code>
    bool clockStatus;
    clockStatus = PLIB_OSC_ClockHasFailed(OSC_ID_0);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the specific
	device data sheet to determine availability or use PLIB_OSC_ExistsClockFail
	in your application to determine whether this feature is available.
*/

bool PLIB_OSC_ClockHasFailed ( OSC_MODULE_ID index );


// *****************************************************************************
// *****************************************************************************
// Section: OSC Peripheral Library Exists API Routines
// *****************************************************************************
// *****************************************************************************
/* The functions below indicate the existence of the features on the device.
*/

//******************************************************************************
/* Function :  PLIB_OSC_ExistsOnWaitAction( OSC_MODULE_ID index )

  Summary:
    Identifies whether the OnWaitAction feature exists on the OSC module

  Description:
    This function identifies whether the OnWaitAction feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_OnWaitActionSet
    - PLIB_OSC_OnWaitActionGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the OnWaitAction feature is supported on the device
    - false  - If the OnWaitAction feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsOnWaitAction( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsSecondaryEnable( OSC_MODULE_ID index )

  Summary:
    Identifies whether the SecondaryEnable feature exists on the OSC module

  Description:
    This function identifies whether the SecondaryEnable feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_SecondaryEnable
    - PLIB_OSC_SecondaryDisable
    - PLIB_OSC_SecondaryIsEnabled

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the SecondaryEnable feature is supported on the device
    - false  - If the SecondaryEnable feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsSecondaryEnable( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsSecondaryReady( OSC_MODULE_ID index )

  Summary:
    Identifies whether the SecondaryReady feature exists on the OSC module

  Description:
    This function identifies whether the SecondaryReady feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_SecondaryIsReady

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the SecondaryReady feature is supported on the device
    - false  - If the SecondaryReady feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsSecondaryReady( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsFRCDivisor( OSC_MODULE_ID index )

  Summary:
    Identifies whether the FRCDivisor feature exists on the OSC module

  Description:
    This function identifies whether the FRCDivisor feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_FRCDivisorSelect
    - PLIB_OSC_FRCDivisorGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the FRCDivisor feature is supported on the device
    - false  - If the FRCDivisor feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsFRCDivisor( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsOscSelect( OSC_MODULE_ID index )

  Summary:
    Identifies whether the OscSelect feature exists on the OSC module

  Description:
    This function identifies whether the OscSelect feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_SysClockSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the OscSelect feature is supported on the device
    - false  - If the OscSelect feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsOscSelect( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsOscSwitchInit( OSC_MODULE_ID index )

  Summary:
    Identifies whether the OscSwitchInit feature exists on the OSC module

  Description:
    This function identifies whether the OscSwitchInit feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_ClockSwitchingAbort
    - PLIB_OSC_ClockSwitchingIsComplete

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the OscSwitchInit feature is supported on the device
    - false  - If the OscSwitchInit feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsOscSwitchInit( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsOscCurrentGet( OSC_MODULE_ID index )

  Summary:
    Identifies whether the OscCurrentGet feature exists on the OSC module

  Description:
    This function identifies whether the OscCurrentGet feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_CurrentSysClockGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the OscCurrentGet feature is supported on the device
    - false  - If the OscCurrentGet feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsOscCurrentGet( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsPBClockDivisor( OSC_MODULE_ID index )

  Summary:
    Identifies whether the PBClockDivisor feature exists on the OSC module

  Description:
    This function identifies whether the PBClockDivisor feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_PBClockDivisorGet
    - PLIB_OSC_PBClockDivisorSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the PBClockDivisor feature is supported on the device
    - false  - If the PBClockDivisor feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsPBClockDivisor( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsPBClockReady( OSC_MODULE_ID index )

  Summary:
    Identifies whether the PBClockReady feature exists on the OSC module

  Description:
    This function identifies whether the PBClockReady feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_PBClockDivisorIsReady

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the PBClockReady feature is supported on the device
    - false  - If the PBClockReady feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsPBClockReady( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsUsbClockSource( OSC_MODULE_ID index )

  Summary:
    Identifies whether the UsbClockSource feature exists on the OSC module

  Description:
    This function identifies whether the UsbClockSource feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_UsbClockSourceSelect
    - PLIB_OSC_UsbClockSourceGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the UsbClockSource feature is supported on the device
    - false  - If the UsbClockSource feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsUsbClockSource( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsPLLLockStatus( OSC_MODULE_ID index )

  Summary:
    Identifies whether the PLLLockStatus feature exists on the OSC module

  Description:
    This function identifies whether the PLLLockStatus feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_PLLIsLocked

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the PLLLockStatus feature is supported on the device
    - false  - If the PLLLockStatus feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsPLLLockStatus( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsPLLClockLock( OSC_MODULE_ID index )

  Summary:
    Identifies whether the PLLClockLock feature exists on the OSC module

  Description:
    This function identifies whether the PLLClockLock feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_PLLClockLock
    - PLIB_OSC_PLLClockUnlock
    - PLIB_OSC_PLLClockIsLocked

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the PLLClockLock feature is supported on the device
    - false  - If the PLLClockLock feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsPLLClockLock( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsSysPLLMultiplier( OSC_MODULE_ID index )

  Summary:
    Identifies whether the PLLMultiplier feature exists on the OSC module

  Description:
    This function identifies whether the PLLMultiplier feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_SysPLLMultiplierSelect
    - PLIB_OSC_SysPLLMultiplierGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the PLLMultiplier feature is supported on the device
    - false  - If the PLLMultiplier feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsSysPLLMultiplier( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsSysPLLOutputDivisor( OSC_MODULE_ID index )

  Summary:
    Identifies whether the PLLOutputDivisor feature exists on the OSC module

  Description:
    This function identifies whether the PLLOutputDivisor feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_SysPLLOutputDivisorSet
    - PLIB_OSC_SysPLLOutputDivisorGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the PLLOutputDivisor feature is supported on the device
    - false  - If the PLLOutputDivisor feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsSysPLLOutputDivisor( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsClockFail( OSC_MODULE_ID index )

  Summary:
    Identifies whether the ClockFail feature exists on the OSC module

  Description:
    This function identifies whether the ClockFail feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_ClockHasFailed

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the ClockFail feature is supported on the device
    - false  - If the ClockFail feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsClockFail( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsFRCTuning( OSC_MODULE_ID index )

  Summary:
    Identifies whether the FRCTuning feature exists on the OSC module

  Description:
    This function identifies whether the FRCTuning feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_FRCTuningSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the FRCTuning feature is supported on the device
    - false  - If the FRCTuning feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsFRCTuning( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsReferenceOscBaseClock( OSC_MODULE_ID index )

  Summary:
    Identifies whether the ReferenceOscBaseClock feature exists on the OSC module

  Description:
    This function identifies whether the ReferenceOscBaseClock feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_ReferenceOscBaseClockSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the ReferenceOscBaseClock feature is supported on the device
    - false  - If the ReferenceOscBaseClock feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsReferenceOscBaseClock( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsReferenceOscChange( OSC_MODULE_ID index )

  Summary:
    Identifies whether the ReferenceOscChange feature exists on the OSC module

  Description:
    This function identifies whether the ReferenceOscChange feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_ReferenceOscSwitchIsComplete

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the ReferenceOscChange feature is supported on the device
    - false  - If the ReferenceOscChange feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsReferenceOscChange( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsReferenceOscChangeActive( OSC_MODULE_ID index )

  Summary:
    Identifies whether the ReferenceOscChangeActive feature exists on the OSC module

  Description:
    This function identifies whether the ReferenceOscChangeActive feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_ReferenceOscSourceChangeIsActive

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the ReferenceOscChangeActive feature is supported on the device
    - false  - If the ReferenceOscChangeActive feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsReferenceOscChangeActive( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsReferenceOscStopInSleep( OSC_MODULE_ID index )

  Summary:
    Identifies whether the ReferenceOscStopInSleep feature exists on the OSC module

  Description:
    This function identifies whether the ReferenceOscStopInSleep feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_ReferenceOscStopInSleepEnable
    - PLIB_OSC_ReferenceOscStopInSleepDisable
    - PLIB_OSC_ReferenceOscStopInSleepIsEnabled

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the ReferenceOscStopInSleep feature is supported on the device
    - false  - If the ReferenceOscStopInSleep feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsReferenceOscStopInSleep( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsReferenceOutputEnable( OSC_MODULE_ID index )

  Summary:
    Identifies whether the ReferenceOutputEnable feature exists on the OSC module

  Description:
    This function identifies whether the ReferenceOutputEnable feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_ReferenceOutputEnable
    - PLIB_OSC_ReferenceOutputDisable
    - PLIB_OSC_ReferenceOutputIsEnabled

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the ReferenceOutputEnable feature is supported on the device
    - false  - If the ReferenceOutputEnable feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsReferenceOutputEnable( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsReferenceOscStopInIdleEnable( OSC_MODULE_ID index )

  Summary:
    Identifies whether the ReferenceOscStopInIdleEnable feature exists on the OSC module

  Description:
    This function identifies whether the ReferenceOscStopInIdleEnable feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_ReferenceOscStopInIdleEnable
    - PLIB_OSC_ReferenceOscStopInIdleDisable
    - PLIB_OSC_ReferenceOscStopInIdleIsEnabled

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the ReferenceOscStopInIdleEnable feature is supported on the device
    - false  - If the ReferenceOscStopInIdleEnable feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsReferenceOscStopInIdleEnable( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsReferenceOscEnable( OSC_MODULE_ID index )

  Summary:
    Identifies whether the ReferenceOscEnable feature exists on the OSC module

  Description:
    This function identifies whether the ReferenceOscEnable feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_ReferenceOscEnable
    - PLIB_OSC_ReferenceOscDisable
    - PLIB_OSC_ReferenceOscIsEnabled

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the ReferenceOscEnable feature is supported on the device
    - false  - If the ReferenceOscEnable feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsReferenceOscEnable( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsReferenceOscDivisor( OSC_MODULE_ID index )

  Summary:
    Identifies whether the ReferenceOscDivisor feature exists on the OSC module

  Description:
    This function identifies whether the ReferenceOscDivisor feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_ReferenceOscDivisorValueSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the ReferenceOscDivisor feature is supported on the device
    - false  - If the ReferenceOscDivisor feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsReferenceOscDivisor( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsReferenceOscTrim( OSC_MODULE_ID index )

  Summary:
    Identifies whether the ReferenceOscTrim feature exists on the OSC module

  Description:
    This function identifies whether the ReferenceOscTrim feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_ReferenceOscTrimSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the ReferenceOscTrim feature is supported on the device
    - false  - If the ReferenceOscTrim feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsReferenceOscTrim( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsPBClockOutputEnable( OSC_MODULE_ID index )

  Summary:
    Identifies whether the PBClockOutputEnable feature exists on the OSC module

  Description:
    This function identifies whether the PBClockOutputEnable feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_PBOutputClockEnable
    - PLIB_OSC_PBOutputClockDisable
    - PLIB_OSC_PBOutputClockIsEnabled

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the PBClockOutputEnable feature is supported on the device
    - false  - If the PBClockOutputEnable feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsPBClockOutputEnable( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsSysPLLInputDivisor( OSC_MODULE_ID index )

  Summary:
    Identifies whether the PLLInputDivisor feature exists on the OSC module

  Description:
    This function identifies whether the PLLInputDivisor feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_SysPLLInputDivisorSet
    - PLIB_OSC_SysPLLInputDivisorGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the PLLInputDivisor feature is supported on the device
    - false  - If the PLLInputDivisor feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsSysPLLInputDivisor( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsSysPLLInputClockSource( OSC_MODULE_ID index )

  Summary:
    Identifies whether the PLLInputClockSource feature exists on the OSC module

  Description:
    This function identifies whether the PLLInputClockSource feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_SysPLLInputClockSourceSet
    - PLIB_OSC_SysPLLInputClockSourceGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the PLLInputClockSource feature is supported on the device
    - false  - If the PLLInputClockSource feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsSysPLLInputClockSource( OSC_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_OSC_ExistsSysPLLFrequencyRange( OSC_MODULE_ID index )

  Summary:
    Identifies whether the PLLFrequencyRange feature exists on the OSC module

  Description:
    This function identifies whether the PLLFrequencyRange feature is available on the OSC module.
    When this function returns true, these functions are supported on the device:
    - PLIB_OSC_SysPLLFrequencyRangeSet
    - PLIB_OSC_SysPLLFrequencyRangeGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - If the PLLFrequencyRange feature is supported on the device
    - false  - If the PLLFrequencyRange feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_OSC_ExistsSysPLLFrequencyRange( OSC_MODULE_ID index );

#endif // _PLIB_OSC_H
/*******************************************************************************
 End of File
*/

