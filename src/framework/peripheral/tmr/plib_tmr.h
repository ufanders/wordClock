/*******************************************************************************
  Timer Peripheral Library Interface Header

  Company:
    Microchip Technology Inc.

  File Name:
    plib_tmr.h

  Summary:
    Timer Peripheral Library interface header.

  Description:
    This header file contains the function prototypes and definitions of
    the data types and constants that make up the interface to the Timer
    Peripheral Library.
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

#ifndef _PLIB_TMR_H
#define _PLIB_TMR_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files (continued at end of file)
// *****************************************************************************
// *****************************************************************************
/*  This section lists the other files that are included in this file.  However,
    please see the bottom of the file for additional implementation header files
    that are also included
*/

#include "peripheral/tmr/processor/tmr_processor.h"

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************
/*  A brief description of a section can be given directly below the section
    banner.
*/


// *****************************************************************************
// *****************************************************************************
// Section: TMR Peripheral Library Interface Routines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* Function:
    void PLIB_TMR_GateEnable(TMR_MODULE_ID index)

  Summary:
    Enables counting controlled by the corresponding gate function.

  Description:
    This function enables counting controlled by the gate function.

  Precondition:
    The timer must be enabled, for this function to take effect.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_GateEnable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsGatedTimeAccumulation in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_GateEnable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_GateDisable(TMR_MODULE_ID index)

  Summary:
    Enables counting regardless of the corresponding timer gate function.

  Description:
    This function enables counting regardless of the gate function.

  Precondition:
    The timer must be enabled for this function to take effect.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_GateDisable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsGatedTimeAccumulation in your application to determine whether
	this feature is available.
*/
void PLIB_TMR_GateDisable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_GatePolaritySelect(TMR_MODULE_ID index, TMR_GATE_POLARITY polarity)

  Summary:
    Selects the timer gate polarity.

  Description:
    This function selects the timer gate polarity.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    polarity        - One of the possible values of TMR_GATE_POLARITY

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_GatePolaritySelect(MY_TMR_INSTANCE, TMR_GATE_POLARITY_ACTIVE_HIGH);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsGatePolarity in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_GatePolaritySelect(TMR_MODULE_ID index, TMR_GATE_POLARITY polarity);


// *****************************************************************************
/* Function:
    void PLIB_TMR_GateToggleModeEnable(TMR_MODULE_ID index)

  Summary:
    Enables the Gate Toggle mode.

  Description:
    This function enables Gate Toggle mode.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_GateToggleModeEnable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsGateToggleMode in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_GateToggleModeEnable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_GateToggleModeDisable(TMR_MODULE_ID index)

  Summary:
    Disables the Gate Toggle mode.

  Description:
    This function disables Gate Toggle mode and the toggle flip flop is cleared.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_GateToggleModeDisable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsGateToggleMode in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_GateToggleModeDisable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_GateSinglePulseModeEnable(TMR_MODULE_ID index)

  Summary:
    Enables Single pulse mode and the controlling corresponding timer gate.

  Description:
    This function enables Single pulse mode and the controlling corresponding
    timer gate.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_GateSinglePulseModeEnable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsGateSinglePulseMode in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_GateSinglePulseModeEnable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_GateSinglePulseModeDisable(TMR_MODULE_ID index)

  Summary:
    Disables Single Pulse mode.

  Description:
    This function disables Single Pulse mode.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_GateSinglePulseModeDisable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsGateSinglePulseMode in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_GateSinglePulseModeDisable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_GateSinglePulseAcquisitionStart(TMR_MODULE_ID index)

  Summary:
    Starts the gate single pulse acquisition.

  Description:
    This function starts the gate single pulse acquisition feature.

  Precondition:
    PLIB_TMR_GateSinglePulseModeEnable should be called before this function.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_GateSinglePulseAcquisitionStart(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsGateSinglePulseAcqusition in your application to determine
	whether this feature is available.
*/

void PLIB_TMR_GateSinglePulseAcquisitionStart(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    bool PLIB_TMR_GateSinglePulseAcquisitionHasCompleted(TMR_MODULE_ID index)

  Summary:
    Returns the gate single pulse acquisition status.

  Description:
    This function returns the gate single pulse acquisition status.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    - true          - Acquisition is waiting for an edge
    - false         - Acquisition has completed or has not been started


  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    bool isComplete;
    isComplete = PLIB_TMR_GateSinglePulseAcquisitionHasCompleted(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsGateSinglePulseAcqusition in your application to determine
	whether this feature is available.
*/

bool PLIB_TMR_GateSinglePulseAcquisitionHasCompleted(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    bool PLIB_TMR_GateCurrentStateGet(TMR_MODULE_ID index)

  Summary:
    Returns the current level of the timer gate.

  Description:
    This function returns the current level of the timer gate. The gate value is
    set for the entire duration of the clock count whether it is a single pulse
    or a full clock cycle.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    - true          - Acquisition has started
    - false         - Acquisition has completed or has not been started

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    bool gatestate = PLIB_TMR_GateCurrentStateGet(MY_TMR_INSTANCE);
    </code>

  Remarks:
    This feature is not affected by PLIB_TMR_GateEnable or PLIB_TMR_GateDisable.

        This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsGateCurrentState in your application to determine whether
	this feature is available.
*/

bool PLIB_TMR_GateCurrentStateGet(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_GateSourceSelect(TMR_MODULE_ID index, TMR_GATE_SOURCE source)

  Summary:
    Selects the timer gate source.

  Description:
    This function selects the timer gate source.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    source          - One of the possible values of TMR_GATE_SOURCE

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_GateSourceSelect(MY_TMR_INSTANCE, TMR_GATE_SOURCE_CMP1_SYNC_OUTPUT);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsGateSource in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_GateSourceSelect(TMR_MODULE_ID index, TMR_GATE_SOURCE source);


// *****************************************************************************
/* Function:
    void PLIB_TMR_ClockSourceSelect(TMR_MODULE_ID index, TMR_CLOCK_SOURCE source)

  Summary:
    Selects the clock source.

  Description:
    This function selects the clock source.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    source          - One of the possible values of TMR_CLOCK_SOURCE

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_ClockSourceSelect(MY_TMR_INSTANCE, TMR_CLOCK_SOURCE_PERIPHERAL_CLOCK);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsGateSource in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_ClockSourceSelect(TMR_MODULE_ID index, TMR_CLOCK_SOURCE source);


// *****************************************************************************
/* Function:
    void PLIB_TMR_TimerOscillatorEnable(TMR_MODULE_ID index)

  Summary:
    Enables the oscillator associated with the Timer module.

  Description:
    This function enables the oscillator associated with the Timer module.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_TimerOscillatorEnable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsTimerOscillator in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_TimerOscillatorEnable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_TimerOscillatorDisable(TMR_MODULE_ID index)

  Summary:
    Disables the oscillator associated with the Timer module.

  Description:
    This function disables the oscillator associated with the Timer module.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_TimerOscillatorDisable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsTimerOscillator in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_TimerOscillatorDisable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_PrescalerEnable(TMR_MODULE_ID index)

  Summary:
    Enables the prescaler assignment to the indexed Timer module.

  Description:
    This function enables the prescaler assignment to the indexed Timer module.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_PrescalerEnable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsPrescalerAssignment in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_PrescalerEnable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_PrescalerDisable(TMR_MODULE_ID index)

  Summary:
    Disables the prescaler assignment to the indexed Timer module.

  Description:
    This function disables the prescaler assignment to the indexed Timer module.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_PrescalerDisable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsPrescalerAssignment in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_PrescalerDisable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_PrescaleSelect(TMR_MODULE_ID index, TMR_PRESCALE prescale)

  Summary:
    Selects the clock prescaler.

  Description:
    This function selects the clock prescaler.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    prescale        - One of the possible values of TMR_PRESCALE

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_PrescaleSelect(MY_TMR_INSTANCE, TMR_PRESCALE_VALUE_8);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsPrescale in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_PrescaleSelect ( TMR_MODULE_ID index, TMR_PRESCALE prescale );


// *****************************************************************************
/* Function:
    uint16_t PLIB_TMR_PrescaleGet ( TMR_MODULE_ID index )

  Summary:
    Gets the prescaler divisor value.

  Description:
    This function gets the prescaler divisor value. The value returned will
    be direct number and not the enum equivlent.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance to be configured

  Returns:
    Prescaler divisor value.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    uint16_t prescale;
    prescale = PLIB_TMR_PrescaleGet (MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsPrescale in your application to determine whether
	this feature is available.
*/

uint16_t PLIB_TMR_PrescaleGet ( TMR_MODULE_ID index );


// *****************************************************************************
/* Function:
    uint16_t PLIB_TMR_PrescaleDivisorGet(TMR_MODULE_ID index, TMR_PRESCALE prescale)

  Summary:
    Gets the prescaler divisor information.

  Description:
    This function gets the prescaler divisor information.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    prescale        - One of the possible values of TMR_PRESCALE

  Returns:
    prescale divisor value

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    uint16_t div = PLIB_TMR_PrescaleDivisorGet (MY_TMR_INSTANCE, TMR_PRESCALE_VALUE_1);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsPrescale in your application to determine whether
	this feature is available.
*/

uint16_t PLIB_TMR_PrescaleDivisorGet(TMR_MODULE_ID index, TMR_PRESCALE prescale);


// *****************************************************************************
/* Function:
    void PLIB_TMR_ClockSourceExternalSyncEnable(TMR_MODULE_ID index)

  Summary:
    Enables the clock synchronization of the external input.

  Description:
    This function enables the clock synchronization of the external input.

  Precondition:
    The timer module must be configured to use the external clock using the
    function PLIB_TMR_ClockSourceSelect with the clock source as
    TMR_CLOCK_SOURCE_EXTERNAL_INPUT_PIN.
    If the function PLIB_TMR_ClockSourceSelect configures the clock with some
    other enumeration, this function will have no effect.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_ClockSourceExternalSyncEnable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsClockSourceSync in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_ClockSourceExternalSyncEnable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_ClockSourceExternalSyncDisable(TMR_MODULE_ID index)

  Summary:
    Disables the clock synchronization of the external input.

  Description:
    This function disables the clock synchronization of the external input.

  Precondition:
    The timer module must be configured to use the external clock using the
    function  PLIB_TMR_ClockSourceSelect with the clock source as
    TMR_CLOCK_SOURCE_EXTERNAL_INPUT_PIN.
    If the function PLIB_TMR_ClockSourceSelect configures the clock with some
    other enumeration, this function will have no effect.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_ClockSourceExternalSyncDisable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsClockSourceSync in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_ClockSourceExternalSyncDisable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_Mode8BitEnable(TMR_MODULE_ID index)

  Summary:
    Enables the Timer module in 8-bit operation mode and disables 16-bit mode.

  Description:
    This function enables the Timer module in 8-bit operation mode and disables
    16-bit mode.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_Mode8BitEnable(MY_TMR_INSTANCE);
    </code>

  Remarks:
    Calling this function disables the operation of Timer module in 16-bit mode.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_TMR_ExistsMode8Bit in your application to determine whether
    this feature is available.

    For PIC18 Timers, this function disables 16-bit mode and enables 8-bit mode.
*/

void PLIB_TMR_Mode8BitEnable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_Mode16BitEnable(TMR_MODULE_ID index)

  Summary:
    Enables the Timer module for 16-bit operation and disables all other modes.

  Description:
    This function enables the Timer module for 16-bit operation and disables all
    other modes.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_Mode16BitEnable(MY_TMR_INSTANCE);
    </code>

  Remarks:
    Calling this function disables the operation of the Timer module 8-bit mode.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_TMR_ExistsMode16Bit in your application to determine whether
    this feature is available.

    For PIC18 Timers, this function disables 8-bit mode and enables 16-bit mode.
    For PIC24/PIC32 TMRs, this function enables 16-bit mode and disables 32-bit
    mode.
*/

void PLIB_TMR_Mode16BitEnable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_Mode32BitEnable(TMR_MODULE_ID index)

  Summary:
    Enables 32-bit operation on the Timer module combination.

  Description:
    This function enables the Timer module and the index +1 Timer module to act
    as one 32-bit timer module and disables all other modes.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_Mode32BitEnable(MY_TMR_INSTANCE);
    </code>

  Remarks:
    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_TMR_ExistsMode32Bit in your application to determine whether
    this feature is available.

    For PIC24/PIC32 Timers, this function enables 32-bit mode and disables
    16-bit mode.
*/

void PLIB_TMR_Mode32BitEnable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_Start(TMR_MODULE_ID index)

  Summary:
    Starts/enables the indexed timer.

  Description:
    This function starts/enables the indexed timer.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_Start(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsEnableControl in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_Start(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_Stop(TMR_MODULE_ID index)

  Summary:
    Stops/disables the indexed timer.

  Description:
    This function stops/disables the indexed timer.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_Stop(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsEnableControl in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_Stop(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_PostscaleSelect(TMR_MODULE_ID index, TMR_POSTSCALE postscale)

  Summary:
    Selects the output clock postscaler.

  Description:
    This function selects the output clock postscaler.

  Conditions:
    None.

  Input:
    index       -  Identifier for the device instance to be configured
    postscale   -  One of the possible values of TMR_POSTSCALE

  Return:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_PostscaleSelect(MY_TMR_INSTANCE, TMR_POSTSCALE_1_TO_1);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsPostscale in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_PostscaleSelect(TMR_MODULE_ID index, TMR_POSTSCALE postscale);


// *****************************************************************************
/* Function:
    TMR_POSTSCALE PLIB_TMR_PostscaleGet(TMR_MODULE_ID index)

  Summary:
    Gets the postscaler divisor information.

  Description:
    This function gets the postscaler divisor information.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    One of the possible values of TMR_POSTSCALE.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    TMR_POSTSCALE postscale;
    postscale = PLIB_TMR_PostscaleGet (MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsPostscale in your application to determine whether
	this feature is available.
*/

TMR_POSTSCALE PLIB_TMR_PostscaleGet(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    uint16_t PLIB_TMR_PostscaleDivisorGet(TMR_MODULE_ID index, TMR_POSTSCALE postscale)

  Summary:
    Gets the postscaler divisor information.

  Description:
    This function gets the postscaler divisor information.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    postscale       - One of the possible values of TMR_POSTSCALE

  Returns:
    Postscaler divisor value.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    uint16_t div = PLIB_TMR_PostscaleDivisorGet (MY_TMR_INSTANCE, TMR_POSTSCALE_1_TO_16);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsPostscale in your application to determine whether
	this feature is available.
*/

uint16_t PLIB_TMR_PostscaleDivisorGet(TMR_MODULE_ID index, TMR_POSTSCALE postscale);


// *****************************************************************************
/* Function:
    void PLIB_TMR_ClockSourceEdgeSelect(TMR_MODULE_ID index, TMR_CLOCK_SOURCE_EDGE source)

  Summary:
    Selects the input clock source edge.

  Description:
    This function selects the input clock source edge, which could either be
    high-to-low (falling edge) or low-to-high (rising edge).

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    source          - One of the possible values of TMR_CLOCK_SOURCE_EDGE

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_ClockSourceEdgeSelect(MY_TMR_INSTANCE, TMR_CLOCK_SOURCE_EDGE_INCREMENT_ON_LOW_TO_HIGH);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsClockSourceEdge in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_ClockSourceEdgeSelect(TMR_MODULE_ID index, TMR_CLOCK_SOURCE_EDGE source);


// *****************************************************************************
/* Function:
    TMR_CLOCK_SOURCE_EDGE PLIB_TMR_ClockSourceEdgeGet(TMR_MODULE_ID index)

  Summary:
    Gets the source edge information.

  Description:
    This function gets the input clock source edge information.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    One of the possible values of TMR_CLOCK_SOURCE_EDGE.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    TMR_CLOCK_SOURCE_EDGE source;
    source = PLIB_TMR_ClockSourceEdgeGet(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsClockSourceEdge in your application to determine whether
	this feature is available.
*/

TMR_CLOCK_SOURCE_EDGE PLIB_TMR_ClockSourceEdgeGet(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_StopInIdleEnable(TMR_MODULE_ID index)

  Summary:
    Discontinues module operation when the device enters Idle mode.

  Description:
    This function discontinues module operation when the device enters Idle mode.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_StopInIdleEnable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsStopInIdleControl in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_StopInIdleEnable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_StopInIdleDisable(TMR_MODULE_ID index)

  Summary:
    Continue module operation when the device enters Idle mode.

  Description:
    This function continues module operation when the device enters Idle mode.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_StopInIdleDisable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsStopInIdleControl in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_StopInIdleDisable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_AssignmentSelect(TMR_MODULE_ID index, TMR_ASSIGNMENT tmrNums)

  Summary:
    Assigns the specified Timer(s) to the selected modules.

  Description:
    This function assigns the specified Timer(s) as the clock source for
    the selected modules.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    tmrNums         - One of the possible values of TMR_ASSIGNMENT

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_AssignmentSelect(MY_TMR_INSTANCE, TMR_ASSIGNMENT_T1_TO_CCP_ALL);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsTimerAssignment in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_AssignmentSelect(TMR_MODULE_ID index, TMR_ASSIGNMENT tmrNums);


// *****************************************************************************
/* Function:
    void PLIB_TMR_OperateInSleepEnable(TMR_MODULE_ID index)

  Summary:
    Enables Timer module operation when the device is in Sleep mode.

  Description:
    This function enables Timer module operation when the device is in Sleep mode.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_OperateInSleepEnable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsOperationInSleep in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_OperateInSleepEnable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_OperateInSleepDisable(TMR_MODULE_ID index)

  Summary:
    Disables Timer module operation when the device is in Sleep mode.

  Description:
    This function disables Timer module operation when the device is in Sleep mode.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_OperateInSleepDisable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsOperationInSleep in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_OperateInSleepDisable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_TriggerEventResetEnable(TMR_MODULE_ID index)

  Summary:
    Enables the special event trigger reset.

  Description:
    This function enables the special event trigger reset.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_TriggerEventResetEnable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsTriggerEventReset in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_TriggerEventResetEnable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_TriggerEventResetDisable(TMR_MODULE_ID index)

  Summary:
    Disables the special event trigger reset.

  Description:
    This function disables the special event trigger reset.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_TriggerEventResetDisable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsTriggerEventReset in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_TriggerEventResetDisable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_SingleShotModeEnable(TMR_MODULE_ID index)

  Summary:
    Enables Single-shot mode.

  Description:
    This function enables Single-shot mode.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_SingleShotModeEnable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsCountMode in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_SingleShotModeEnable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_ContinousCountModeEnable(TMR_MODULE_ID index)

  Summary:
    Enables Continuous Count mode

  Description:
    This function enables Continuous Count mode.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_ContinousCountModeEnable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsCountMode in your application to determine whether
	this feature is available.
*/

void PLIB_TMR_ContinousCountModeEnable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_CounterAsyncWriteEnable(TMR_MODULE_ID index)

  Summary:
    Enables back-to-back writes with legacy asynchronous timer functionality.

  Description:
    This function enables back-to-back writes with legacy asynchronous timer
    functionality .

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_CounterAsyncWriteEnable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsCounterAsyncWriteControl in your application to determine
	whether this feature is available.
*/

void PLIB_TMR_CounterAsyncWriteEnable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_CounterAsyncWriteDisable(TMR_MODULE_ID index)

  Summary:
    Disables the writes to the counter register until the pending write operation
    completes.

  Description:
    This function disables the writes to the counter register until the pending
    write operation completes.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_CounterAsyncWriteDisable(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsCounterAsyncWriteControl in your application to determine
	whether this feature is available.
*/

void PLIB_TMR_CounterAsyncWriteDisable(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    bool PLIB_TMR_CounterAsyncWriteInProgress(TMR_MODULE_ID index)

  Summary:
    Returns the status of the counter write in Asynchronous mode.

  Description:
    This function returns the status of the counter write in Asynchronous mode.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    - true  - Write is in progress
    - false - Write is complete

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    bool inProgress = PLIB_TMR_CounterAsyncWriteInProgress(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsCounterAsyncWriteInProgress in your application to determine
	whether this feature is available.
*/

bool PLIB_TMR_CounterAsyncWriteInProgress(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    bool PLIB_TMR_SystemClockFromTimerIsActive(TMR_MODULE_ID index)

  Summary:
    Gets the system clock status.

  Description:
    This function gets the system clock derivation status. When the function returns
    true, the system and peripheral clocks are sourced from the timer.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    - true            - System/Device Clock is derived from the timer
    - false           - System/Device Clock is from another source

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    uint8_t status = PLIB_TMR_SystemClockFromTimerIsActive(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsSystemClockStatus in your application to determine
	whether this feature is available.
*/

bool PLIB_TMR_SystemClockFromTimerIsActive(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_Counter8BitSet(TMR_MODULE_ID index, uint8_t value)

  Summary:
    Sets the 8-bit timer value.

  Description:
    This function sets the 8-bit timer value.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    value           - 8-bit value to be set

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_Counter8BitSet(MY_TMR_INSTANCE, 0x10);
    </code>

  Remarks:
    When the timer register is written, the timer increment does not occur
    for that cycle. Writes to the timer register in the asynchronous counter
    mode should be avoided.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_TMR_ExistsCounter8Bit in your application to determine whether this
    feature is available.
*/

void PLIB_TMR_Counter8BitSet(TMR_MODULE_ID index, uint8_t value);


// *****************************************************************************
/* Function:
    uint8_t PLIB_TMR_Counter8BitGet(TMR_MODULE_ID index)

  Summary:
    Gets the 8-bit timer value.

  Description:
    This function gets the 8-bit timer value.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    8-bit timer value.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    uint8_t timerValue = PLIB_TMR_Counter8BitGet(MY_TMR_INSTANCE);
    </code>

  Remarks:
    PLIB_TMR_Counter8BitGet does not prevent the timer from incrementing
    during the same instruction cycle.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_TMR_ExistsCounter8Bit in your application to determine whether this
    feature is available.
*/

uint8_t PLIB_TMR_Counter8BitGet(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_Counter8BitClear(TMR_MODULE_ID index)

  Summary:
    Clears the 8-bit timer value.

  Description:
    This function clears the 8-bit timer value.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_Counter8BitClear(MY_TMR_INSTANCE);
    </code>

  Remarks:
    When the timer register is written, the timer increment does not occur
    for that cycle. Writes to the timer register in the asynchronous counter
    mode should be avoided.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_TMR_ExistsCounter8Bit in your application to determine whether this
    feature is available.
*/

void PLIB_TMR_Counter8BitClear(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_Counter16BitSet(TMR_MODULE_ID index, uint16_t value)

  Summary:
    Sets the 16-bit timer value.

  Description:
    This function sets the 16-bit timer value.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    value           - 16-bit value to be set

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_Counter16BitSet(MY_TMR_INSTANCE, 0x100);
    </code>

  Remarks:
    When the timer register is written to, the timer increment does not occur
    for that cycle. Writes to the timer register in the asynchronous counter
    mode should be avoided.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_TMR_ExistsCounter16Bit in your application to determine whether this
    feature is available.
*/

void PLIB_TMR_Counter16BitSet(TMR_MODULE_ID index, uint16_t value);


// *****************************************************************************
/* Function:
    uint16_t PLIB_TMR_Counter16BitGet(TMR_MODULE_ID index)

  Summary:
    Gets the 16-bit timer value.

  Description:
    This function gets the 16-bit timer value.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    16-bit timer value.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    uint16_t timerValue = PLIB_TMR_Counter16BitGet(MY_TMR_INSTANCE);
    </code>

  Remarks:
    PLIB_TMR_Counter16BitGet does not prevent the timer from incrementing
    during the same instruction cycle.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_TMR_ExistsCounter16Bit in your application to determine whether this
    feature is available.
*/

uint16_t PLIB_TMR_Counter16BitGet(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    uint8_t PLIB_TMR_Counter16BitClear(TMR_MODULE_ID index)

  Summary:
    Clears the 16-bit timer value.

  Description:
    This function clears the 16-bit timer value.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_Counter16BitClear(MY_TMR_INSTANCE);
    </code>

  Remarks:
    When the timer register is written, the timer increment does not occur
    for that cycle. Writes to the timer register in the asynchronous counter
    mode should be avoided.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_TMR_ExistsCounter16Bit in your application to determine whether this
    feature is available.
*/

void PLIB_TMR_Counter16BitClear(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_Counter32BitSet(TMR_MODULE_ID index, uint32_t value)

  Summary:
    Sets the 32-bit timer value.

  Description:
    This function sets the 32-bit timer value.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    value           - 32-bit value to be set

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_Counter32BitSet(MY_TMR_INSTANCE, 0x1000000);
    </code>

  Remarks:
    When the timer register is written, the timer increment does not occur
    for that cycle. Writes to the timer register in the asynchronous counter
    mode should be avoided.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_TMR_ExistsCounter32Bit in your application to determine whether this
    feature is available.
*/

void PLIB_TMR_Counter32BitSet(TMR_MODULE_ID index, uint32_t value);


// *****************************************************************************
/* Function:
    uint32_t PLIB_TMR_Counter32BitGet(TMR_MODULE_ID index)

  Summary:
    Gets the 32-bit timer value.

  Description:
    This function gets the 32-bit timer value.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    32 bit timer value.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    uint32_t timerValue = PLIB_TMR_Counter32BitGet(MY_TMR_INSTANCE);
    </code>

  Remarks:
    PLIB_TMR_Counter32BitGet does not prevent the timer from incrementing
    during the same instruction cycle.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_TMR_ExistsCounter32Bit in your application to determine whether this
    feature is available.
*/

uint32_t PLIB_TMR_Counter32BitGet(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    uint8_t PLIB_TMR_Counter32BitClear(TMR_MODULE_ID index)

  Summary:
    Clears the 32-bit timer value.

  Description:
    This function clears the 32-bit timer value.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    PLIB_TMR_Counter32BitClear(MY_TMR_INSTANCE);
    </code>

  Remarks:
    When the timer register is written, the timer increment does not occur
    for that cycle. Writes to the timer register in the asynchronous counter
    mode should be avoided.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_TMR_ExistsCounter32Bit in your application to determine whether this
    feature is available.
*/

void PLIB_TMR_Counter32BitClear(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_Period8BitSet(TMR_MODULE_ID index, uint8_t period)

  Summary:
    Sets the 8-bit period value.

  Description:
    This function sets the 8-bit period value.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    period          - 8-bit period value

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    // where, MY_PERIOD_VALUE is the 8-bit value which needs to be stored in the
    // period register.
    PLIB_TMR_Period8BitSet(MY_TMR_INSTANCE, MY_PERIOD_VALUE);
    </code>

  Remarks:
    The timer period register may be written at any time before the timer is
    started or after the timer is stopped. The timer period register can also be
    written when servicing the interrupt for the timer. When the timer is in
    operation, it is not recommended to write to the period register.

    This feature may not be available on all devices. Please refer to the
    specific device data sheet to determine availability or use
    PLIB_TMR_ExistsPeriod8Bit in your application to determine whether this
    feature is available.
*/

void PLIB_TMR_Period8BitSet(TMR_MODULE_ID index, uint8_t period);


// *****************************************************************************
/* Function:
    uint8_t PLIB_TMR_Period8BitGet(TMR_MODULE_ID index)

  Summary:
    Gets the 8-bit period value.

  Description:
    This function gets the 8 -it period value.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    period          - 8-bit period value

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    uint8_t period = PLIB_TMR_Period8BitGet(MY_TMR_INSTANCE);
    </code>

  Remarks:
    The timer period register may be written at any time before the timer is
    started or after the timer is stopped. The timer period register can also be
    written when servicing the interrupt for the timer. When the timer is in
    operation, it is not recommended to write to the period register.

	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsPeriod8Bit in your application to determine whether this
	feature is available.
*/

uint8_t PLIB_TMR_Period8BitGet(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_Period16BitSet(TMR_MODULE_ID index, uint16_t period)

  Summary:
    Sets the 16-bit period value.

  Description:
    This function sets the 16-bit period value.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    period          - 16-bit period register value

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    // where, MY_PERIOD_VALUE is the 16-bit value which needs to be stored in the
    // period register.
    PLIB_TMR_Period16BitSet(MY_TMR_INSTANCE, MY_PERIOD_VALUE);
    </code>

  Remarks:
    The timer period register may be written at any time before the timer is
    started or after the timer is stopped. The timer period register can also be
    written when servicing the interrupt for the timer. When the timer is in
    operation, it is not recommended to write to the period register.

	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsPeriod16Bit in your application to determine whether this
	feature is available.
*/

void PLIB_TMR_Period16BitSet(TMR_MODULE_ID index, uint16_t period);


// *****************************************************************************
/* Function:
    uint16_t PLIB_TMR_Period16BitGet(TMR_MODULE_ID index)

  Summary:
    Gets the 16-bit period value.

  Description:
    This function gets the 16-bit period value.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    period          - 16-bit period value

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    uint16_t period = PLIB_TMR_Period16BitGet(MY_TMR_INSTANCE);
    </code>

  Remarks:
    The timer period register may be written at any time before the timer is
    started or after the timer is stopped. The timer period register can also be
    written when servicing the interrupt for the timer. When the timer is in
    operation, it is not recommended to write to the period register.

	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsPeriod16Bit in your application to determine whether this
	feature is available.
*/

uint16_t PLIB_TMR_Period16BitGet(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    void PLIB_TMR_Period32BitSet(TMR_MODULE_ID index, uint32_t period)

  Summary:
    Sets the 32-bit period value.

  Description:
    This function sets the 32-bit period value.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured
    period          - 32-bit period register value

  Returns:
    None.

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    // where, MY_PERIOD_VALUE is the 32-bit value which needs to be stored in the
    // period register.
    PLIB_TMR_Period32BitSet(MY_TMR_INSTANCE, MY_PERIOD_VALUE);
    </code>

  Remarks:
    The timer period register may be written at any time before the timer is
    started or after the timer is stopped. The timer period register can also be
    written when servicing the interrupt for the timer. When the timer is in
    operation, it is not recommended to write to the period register.

	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsPeriod32Bit in your application to determine whether this
	feature is available.
*/

void PLIB_TMR_Period32BitSet(TMR_MODULE_ID index, uint32_t period);


// *****************************************************************************
/* Function:
    uint32_t PLIB_TMR_Period32BitGet(TMR_MODULE_ID index)

  Summary:
    Gets the 32-bit period value.

  Description:
    This function gets the 32-bit period value.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    period          - 32-bit period value

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    uint32_t period = PLIB_TMR_Period32BitGet(MY_TMR_INSTANCE);
    </code>

  Remarks:
    The timer period register may be written at any time before the timer is
    started or after the timer is stopped. The timer period register can also be
    written when servicing the interrupt for the timer. When the timer is in
    operation, it is not recommended to write to the period register.

	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsPeriod32Bit in your application to determine whether this
	feature is available.
*/

uint32_t PLIB_TMR_Period32BitGet(TMR_MODULE_ID index);


// *****************************************************************************
/* Function:
    bool PLIB_TMR_IsPeriodMatchBased (TMR_MODULE_ID index)

  Summary:
    Gets the operating mode state of the Timer module based on Period Match or
    Overflow mode.

  Description:
    This function gets the operating mode state of the Timer module based on
    Period Match or Overflow mode.

  Precondition:
    None.

  Parameters:
    index           - Identifier for the device instance to be configured

  Returns:
    - true            - Operation in Period Match mode
    - false           - Operation in Overflow mode

  Example:
    <code>
    // Where MY_TMR_INSTANCE, is the timer instance selected for use by the
    // application developer.
    bool status = PLIB_TMR_IsPeriodMatchBased(MY_TMR_INSTANCE);
    </code>

  Remarks:
	This feature may not be available on all devices. Please refer to the
	specific device data sheet to determine availability or use
	PLIB_TMR_ExistsTimerOperationMode in your application to determine whether
	this feature is available.
*/

bool PLIB_TMR_IsPeriodMatchBased (TMR_MODULE_ID index);


// *****************************************************************************
// *****************************************************************************
// Section: TMR Peripheral Library Exists API Routines
// *****************************************************************************
// *****************************************************************************
/* The functions below indicate the existence of the features on the device.
*/

//******************************************************************************
/* Function :  PLIB_TMR_ExistsGatedTimeAccumulation( TMR_MODULE_ID index )

  Summary:
    Identifies whether the GatedTimeAccumulation feature exists on the Timer module.

  Description:
    This function identifies whether the GatedTimeAccumulation feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_GateEnable
    - PLIB_TMR_GateDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The GatedTimeAccumulation feature is supported on the device
    - false  - The GatedTimeAccumulation feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsGatedTimeAccumulation( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsGatePolarity( TMR_MODULE_ID index )

  Summary:
    Identifies whether the GatePolarity feature exists on the Timer module.

  Description:
    This function identifies whether the GatePolarity feature is available
    on the Timer module. When this function returns true, this function is
    supported on the device:
    - PLIB_TMR_GatePolaritySelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The GatePolarity feature is supported on the device
    - false  - The GatePolarity feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsGatePolarity( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsGateToggleMode( TMR_MODULE_ID index )

  Summary:
    Identifies whether the GateToggleMode feature exists on the Timer module.

  Description:
    This function identifies whether the GateToggleMode feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_GateToggleModeEnable
    - PLIB_TMR_GateToggleModeDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The GateToggleMode feature is supported on the device
    - false  - The GateToggleMode feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsGateToggleMode( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsGateSinglePulseMode( TMR_MODULE_ID index )

  Summary:
    Identifies whether the GateSinglePulseMode feature exists on the Timer module.

  Description:
    This function identifies whether the GateSinglePulseMode feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_GateSinglePulseModeEnable
    - PLIB_TMR_GateSinglePulseModeDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The GateSinglePulseMode feature is supported on the device
    - false  - The GateSinglePulseMode feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsGateSinglePulseMode( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsGateSinglePulseAcqusition( TMR_MODULE_ID index )

  Summary:
    Identifies whether the GateSinglePulseAcquisition feature exists on the Timer module.

  Description:
    This function identifies whether the GateSinglePulseAcquisition feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_GateSinglePulseAcquisitionStart
    - PLIB_TMR_GateSinglePulseAcquisitionHasCompleted

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The GateSinglePulseAcquisition feature is supported on the device
    - false  - The GateSinglePulseAcquisition feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsGateSinglePulseAcqusition( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsGateCurrentState( TMR_MODULE_ID index )

  Summary:
    Identifies whether the GateCurrentState feature exists on the Timer module.

  Description:
    This function identifies whether the GateCurrentState feature is available
    on the Timer module. When this function returns true, this function is
    supported on the device:
    - PLIB_TMR_GateCurrentStateGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The GateCurrentState feature is supported on the device
    - false  - The GateCurrentState feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsGateCurrentState( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsGateSource( TMR_MODULE_ID index )

  Summary:
    Identifies whether the GateSource feature exists on the Timer module.

  Description:
    This function identifies whether the GateSource feature is available
    on the Timer module. When this function returns true, this function is
    supported on the device:
    - PLIB_TMR_GateSourceSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The GateSource feature is supported on the device
    - false  - The GateSource feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsGateSource( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsClockSource( TMR_MODULE_ID index )

  Summary:
    Identifies whether the ClockSource feature exists on the Timer module.

  Description:
    This function identifies whether the ClockSource feature is available
    on the Timer module. When this function returns true, this function is
    supported on the device:
    - PLIB_TMR_ClockSourceSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ClockSource feature is supported on the device
    - false  - The ClockSource feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsClockSource( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsTimerOscillator( TMR_MODULE_ID index )

  Summary:
    Identifies whether the TimerOscillator feature exists on the Timer module.

  Description:
    This function identifies whether the TimerOscillator feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_TimerOscillatorEnable
    - PLIB_TMR_TimerOscillatorDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The TimerOscillator feature is supported on the device
    - false  - The TimerOscillator feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsTimerOscillator( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsPrescale( TMR_MODULE_ID index )

  Summary:
    Identifies whether the Prescale feature exists on the Timer module.

  Description:
    This function identifies whether the Prescale feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_PrescaleSelect
    - PLIB_TMR_PrescaleGet
    - PLIB_TMR_PrescaleDivisorGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Prescale feature is supported on the device
    - false  - The Prescale feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsPrescale( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsClockSourceSync( TMR_MODULE_ID index )

  Summary:
    Identifies whether the ClockSourceSync feature exists on the Timer module.

  Description:
    This function identifies whether the ClockSourceSync feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_ClockSourceExternalSyncEnable
    - PLIB_TMR_ClockSourceExternalSyncDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ClockSourceSync feature is supported on the device
    - false  - The ClockSourceSync feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsClockSourceSync( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsMode8Bit( TMR_MODULE_ID index )

  Summary:
    Identifies whether the Mode8Bit feature exists on the Timer module.

  Description:
    This function identifies whether the Mode8Bit feature is available
    on the Timer module. When this function returns true, this function is
    supported on the device:
    - PLIB_TMR_Mode8BitEnable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Mode8Bit feature is supported on the device
    - false  - The Mode8Bit feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsMode8Bit( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsMode16Bit( TMR_MODULE_ID index )

  Summary:
    Identifies whether the Mode16Bit feature exists on the Timer module.

  Description:
    This function identifies whether the Mode16Bit feature is available
    on the Timer module. When this function returns true, this function is
    supported on the device:
    - PLIB_TMR_Mode16BitEnable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Mode16Bit feature is supported on the device
    - false  - The Mode16Bit feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsMode16Bit( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsMode32Bit( TMR_MODULE_ID index )

  Summary:
    Identifies whether the Mode32Bit feature exists on the Timer module.

  Description:
    This function identifies whether the Mode32Bit feature is available
    on the Timer module. When this function returns true, this function is
    supported on the device:
    - PLIB_TMR_Mode32BitEnable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Mode32Bit feature is supported on the device
    - false  - The Mode32Bit feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsMode32Bit( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsEnableControl( TMR_MODULE_ID index )

  Summary:
    Identifies whether the EnableControl feature exists on the Timer module.

  Description:
    This function identifies whether the EnableControl feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_Start
    - PLIB_TMR_Stop

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The EnableControl feature is supported on the device
    - false  - The EnableControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsEnableControl( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsPostscale( TMR_MODULE_ID index )

  Summary:
    Identifies whether the Postscale feature exists on the Timer module.

  Description:
    This function identifies whether the Postscale feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_PostscaleSelect
    - PLIB_TMR_PostscaleGet
    - PLIB_TMR_PostscaleDivisorGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Postscale feature is supported on the device
    - false  - The Postscale feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsPostscale( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsClockSourceEdge( TMR_MODULE_ID index )

  Summary:
    Identifies whether the ClockSourceEdge feature exists on the Timer module.

  Description:
    This function identifies whether the ClockSourceEdge feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_ClockSourceEdgeSelect
    - PLIB_TMR_ClockSourceEdgeGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ClockSourceEdge feature is supported on the device
    - false  - The ClockSourceEdge feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsClockSourceEdge( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsPrescalerAssignment( TMR_MODULE_ID index )

  Summary:
    Identifies whether the PrescalerControl feature exists on the Timer module.

  Description:
    This function identifies whether the PrescalerControl feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_PrescalerEnable
    - PLIB_TMR_PrescalerDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PrescalerControl feature is supported on the device
    - false  - The PrescalerControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsPrescalerAssignment( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsStopInIdleControl( TMR_MODULE_ID index )

  Summary:
    Identifies whether the StopInIdle feature exists on the Timer module.

  Description:
    This function identifies whether the StopInIdle feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_StopInIdleEnable
    - PLIB_TMR_StopInIdleDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The StopInIdle feature is supported on the device
    - false  - The StopInIdle feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsStopInIdleControl( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsTimerAssignment( TMR_MODULE_ID index )

  Summary:
    Identifies whether the TimerAssignment feature exists on the Timer module.

  Description:
    This function identifies whether the TimerAssignment feature is available
    on the Timer module. When this function returns true, this function is
    supported on the device:
    - PLIB_TMR_AssignmentSelect

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The TimerAssignment feature is supported on the device
    - false  - The TimerAssignment feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsTimerAssignment( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsOperationInSleep( TMR_MODULE_ID index )

  Summary:
    Identifies whether the OperationInSleep feature exists on the Timer module.

  Description:
    This function identifies whether the OperationInSleep feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_OperateInSleepEnable
    - PLIB_TMR_OperateInSleepDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The OperationInSleep feature is supported on the device
    - false  - The OperationInSleep feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsOperationInSleep( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsTriggerEventReset( TMR_MODULE_ID index )

  Summary:
    Identifies whether the TriggerEventReset feature exists on the Timer module.

  Description:
    This function identifies whether the TriggerEventReset feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_TriggerEventResetEnable
    - PLIB_TMR_TriggerEventResetDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The TriggerEventReset feature is supported on the device
    - false  - The TriggerEventReset feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsTriggerEventReset( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsCountMode( TMR_MODULE_ID index )

  Summary:
    Identifies whether the CountMode feature exists on the Timer module.

  Description:
    This function identifies whether the CountMode feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_ContinousCountModeEnable
    - PLIB_TMR_SingleShotModeEnable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CountMode feature is supported on the device
    - false  - The CountMode feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsCountMode( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsCounter8Bit( TMR_MODULE_ID index )

  Summary:
    Identifies whether the Counter8bit feature exists on the Timer module.

  Description:
    This function identifies whether the Counter8bit feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_Counter8BitSet
    - PLIB_TMR_Counter8BitGet
    - PLIB_TMR_Counter8BitClear

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Counter8bit feature is supported on the device
    - false  - The Counter8bit feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsCounter8Bit( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsCounter16Bit( TMR_MODULE_ID index )

  Summary:
    Identifies whether the Counter16Bit feature exists on the Timer module.

  Description:
    This function identifies whether the Counter16Bit feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_Counter16BitSet
    - PLIB_TMR_Counter16BitGet
    - PLIB_TMR_Counter16BitClear

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Counter16Bit feature is supported on the device
    - false  - The Counter16Bit feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsCounter16Bit( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsCounter32Bit( TMR_MODULE_ID index )

  Summary:
    Identifies whether the Counter32Bit feature exists on the Timer module.

  Description:
    This function identifies whether the Counter32Bit feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_Counter32BitSet
    - PLIB_TMR_Counter32BitGet
    - PLIB_TMR_Counter32BitClear

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Counter32Bit feature is supported on the device
    - false  - The Counter32Bit feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsCounter32Bit( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsPeriod8Bit( TMR_MODULE_ID index )

  Summary:
    Identifies whether the Period8Bit feature exists on the Timer module.

  Description:
    This function identifies whether the Period8Bit feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_Period8BitSet
    - PLIB_TMR_Period8BitGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Period8Bit feature is supported on the device
    - false  - The Period8Bit feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsPeriod8Bit( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsPeriod16Bit( TMR_MODULE_ID index )

  Summary:
    Identifies whether the Period16Bit feature exists on the Timer module.

  Description:
    This function identifies whether the Period16Bit feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_Period16BitSet
    - PLIB_TMR_Period16BitGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Period16Bit feature is supported on the device
    - false  - The Period16Bit feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsPeriod16Bit( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsPeriod32Bit( TMR_MODULE_ID index )

  Summary:
    Identifies whether the Period32Bit feature exists on the Timer module.

  Description:
    This function identifies whether the Period32Bit feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_Period32BitSet
    - PLIB_TMR_Period32BitGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Period32Bit feature is supported on the device
    - false  - The Period32Bit feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsPeriod32Bit( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsCounterAsyncWriteControl( TMR_MODULE_ID index )

  Summary:
    Identifies whether the CounterAsyncWriteControl feature exists on the Timer module.

  Description:
    This function identifies whether the CounterAsyncWriteControl feature is available
    on the Timer module. When this function returns true, these functions are
    supported on the device:
    - PLIB_TMR_CounterAsyncWriteEnable
    - PLIB_TMR_CounterAsyncWriteDisable

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CounterAsyncWriteControl feature is supported on the device
    - false  - The CounterAsyncWriteControl feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsCounterAsyncWriteControl( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsCounterAsyncWriteInProgress( TMR_MODULE_ID index )

  Summary:
    Identifies whether the CounterAsyncWriteInProgress feature exists on the Timer module.

  Description:
    This function identifies whether the CounterAsyncWriteInProgress feature is
    available on the Timer module. When this function returns true, this function
    is supported on the device:
    - PLIB_TMR_CounterAsyncWriteInProgress

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The CounterAsyncWriteInProgress feature is supported on the device
    - false  - The CounterAsyncWriteInProgress feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsCounterAsyncWriteInProgress( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsSystemClockStatus( TMR_MODULE_ID index )

  Summary:
    Identifies whether the SystemClockStatus feature exists on the Timer module.

  Description:
    This function identifies whether the SystemClockStatus feature is
    available on the Timer module. When this function returns true, this function
    is supported on the device:
    - PLIB_TMR_SystemClockFromTimerIsActive

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The SystemClockStatus feature is supported on the device
    - false  - The SystemClockStatus feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsSystemClockStatus( TMR_MODULE_ID index );


//******************************************************************************
/* Function :  PLIB_TMR_ExistsTimerOperationMode( TMR_MODULE_ID index )

  Summary:
    Identifies whether the TimerOperationMode feature exists on the Timer module.

  Description:
    This function identifies whether the TimerOperationMode feature is
    available on the Timer module. When this function returns true, this function
    is supported on the device:
    - PLIB_TMR_IsPeriodMatchBased

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The TimerOperationMode feature is supported on the device
    - false  - The TimerOperationMode feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_TMR_ExistsTimerOperationMode( TMR_MODULE_ID index );


#endif // #ifndef _PLIB_TMR_H
/*******************************************************************************
 End of File
*/

