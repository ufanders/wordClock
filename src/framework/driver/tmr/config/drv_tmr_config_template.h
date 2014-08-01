/*******************************************************************************
  Timer Driver Configuration Definitions for the Template Version

  Company:
    Microchip Technology Inc.

  File Name:
    drv_tmr_config_template.h

  Summary:
    Timer Driver configuration definitions for the template version.

  Description:
    These definitions set up the driver for the default mode of operation of
    the driver.

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

#ifndef _DRV_TMR_CONFIG_TEMPLATE_H
#define _DRV_TMR_CONFIG_TEMPLATE_H


#error "This is a configuration template file.  Do not include it directly."


/**************************************************************************
  Summary:
    Sets up the driver functionality in either split or the unified mode.

  Description:
    TMR driver configuration

    This definition sets up the driver functionality in either split or the
    unified mode.

  Remarks:
    None.
  **************************************************************************/

#define DRV_TMR_UNIFIED                             false


// *****************************************************************************
/* Hardware instances support

  Summary:
    Sets up the maximum number of hardware instances that can be supported by
    the dynamic driver.

  Description:
    This definition sets up the maximum number of hardware instances that can be
    supported by the dynamic driver.

  Remarks:
    None
*/

#define DRV_TMR_INSTANCES_NUMBER                    1


// *****************************************************************************
/* Client support

  Summary:
    Selects the maximum number of clients

  Description:
    This definition select the maximum number of clients that the Timer driver can
    support at run time.

  Remarks:
    None.

*/

#define DRV_TMR_CLIENTS_NUMBER                      1


// *****************************************************************************
/* TMR Static Index Selection

  Summary:
    Timer static index selection.

  Description:
    This definition selects the Timer Static Index for the driver object reference.

  Remarks:
    This index is required to make a reference to the driver object.
*/

#define DRV_TMR_INDEX                               DRV_TMR_INDEX_2


// *****************************************************************************
/* TMR Interrupt And Polled Mode Operation Control

  Summary:
    Controls operation of the driver in the interrupt or polled mode.

  Description:
    This macro controls the operation of the driver in the interrupt
    mode of operation. The possible values of this macro are:
    - true  - Select if interrupt mode of timer operation is desired
    - false - Select if polling mode of timer operation is desired
    Not defining this option to true or false will result in a build error.

  Remarks:
    None.
*/

#define DRV_TMR_INTERRUPT_MODE                      true


// *****************************************************************************
/* TMR prescaler assignment configuration

  Summary:
    Controls prescaler assignment of the Timer.

  Description:
    This macro controls the prescaler assignment of the TMR. This macro accepts
    the following values:
    - true  - Configures the Timer for enable prescaler assignment
    - false - Configures the Timer for disable prescaler assignment
    - DRV_CONFIG_NOT_SUPPORTED - When the feature is not supported on the
             instance

  Remarks:
    This feature is not available in all modules/devices. Refer to the specific
    device data sheet for more information.
*/

#define DRV_TMR_PRESCALER_ENABLE                    false


// *****************************************************************************
/* TMR Asynchronous write mode configuration

  Summary:
    Controls Asynchronous Write mode of the Timer.

  Description:
    This macro controls the Asynchronous Write mode of the Timer. This macro
    accepts the following values:
    - true  - Configures the Timer to enable asynchronous write control
    - false - Configures the Timer to disable asynchronous write control
    - DRV_CONFIG_NOT_SUPPORTED - When the feature is not supported on the
             instance.

  Remarks:
    This feature is not available in all modules/devices. Refer to the specific
    device data sheet for more information.
*/

#define DRV_TMR_ASYNC_WRITE_ENABLE                  false


// *****************************************************************************
// *****************************************************************************
// Section: Initialization Overrides
// *****************************************************************************
// *****************************************************************************
/* This section defines the initialization overrides */

// *****************************************************************************
/* TMR Peripheral ID Selection

  Summary:
    Timer peripheral ID selection.

  Description:
    This macro selects the Timer peripheral ID. This is an initialization
    override of the tmrID member of the initialization configuration.

  Remarks:
    Note: Some devices also support TMR_ID_0.
*/

#define DRV_TMR_PERIPHERAL_ID                       TMR_ID_1


// *****************************************************************************
/* TMR Interrupt Source

  Summary:
    Defines the interrupt source of the static driver.

  Description:
    This macro defines the interrupt source of the static driver.

  Remarks:
    Refer to the Interrupt Peripheral Library documentation for more information
    on the INT_SOURCE enumeration.
*/

#define DRV_TMR_INTERRUPT_SOURCE                    INT_SOURCE_TIMER_1


// *****************************************************************************
/* TMR Synchronization mode configuration

  Summary:
    Controls Synchronization mode of the Timer.

  Description:
    This macro controls the Synchronization mode of the Timer. It accepts one of
    the possible values of the enumeration DRV_TMR_SYNC_MODE.

  Remarks:
    If defined in the configuration, this takes priority over the initialization
    option.
*/

#define DRV_TMR_SYNCHRONIZATION_MODE                DRV_TMR_SYNC_MODE_SYNCHRONOUS_INTERNAL


// *****************************************************************************
/* TMR period value

  Summary:
    Timer period value for an overflow or a period match-based Timer.

  Description:
    This macro defines the Timer period value for an overflow or a period
    match-based Timer.

  Remarks:
    If defined in the configuration, this macro takes priority over the
    initialization option. In addition, the limit is the maximum possible Timer
    count width.

    Note: The user is required to provide the count value. The driver will internally
    adjust the value as required for the Timer hardware based on the overflow or
    period match functionality
*/

#define DRV_TMR_TIMER_PERIOD                        0x4567


// *****************************************************************************
/* TMR Clock Source

  Summary:
    Timer module clock source selection.

  Description:
    This macro selects the Timer module clock source based on the enumeration.

  Remarks:
    If defined in the configuration, this takes priority over the
    initialization option.

    Refer to the processor peripheral header for more information.
*/

#define DRV_TMR_CLOCK_SOURCE                        TMR_CLOCK_SOURCE_PERIPHERAL_CLOCK


// *****************************************************************************
/* TMR Prescale

  Summary:
    Timer module prescale value.

  Description:
    This macro defines the Timer module prescale value.

  Remarks:
    If defined in the configuration, this takes priority over the initialization
    option.

    Refer to the processor peripheral header for more information.
*/

#define DRV_TMR_PRESCALE                            TMR_PRESCALE_TX_1_TO_256


// *****************************************************************************
/* TMR Source Edge

  Summary:
    Timer module source edge.

  Description:
    This macro defines the Timer module source edge.

  Remarks:
    If defined in the configuration, this takes priority over the initialization
    option.

    Refer to the processor peripheral header for more information.
*/

#define DRV_TMR_SOURCE_EDGE                         TMR_SOURCE_EDGE_INCREMENT_ON_LOW_TO_HIGH


// *****************************************************************************
/* TMR post scale configuration

  Summary:
    Macro controls post scale of the TMR

  Description:
    This macro controls the post scale of the TMR

  Remarks:
    This feature is not available in all modules/devices. Refer to the specific
    device data sheet for more information.
*/

#define DRV_TMR_POSTSCALE                           TMR_POSTSCALE_1_TO_1


// *****************************************************************************
/* TMR power state configuration

  Summary:
    Controls the power state of the Timer.

  Description:
    This macro controls the power state of the Timer.

  Remarks:
    This feature is not available in all modules/devices. Refer to the specific
    device data sheet for more information.
*/

#define DRV_TMR_POWER_STATE                         SYS_MODULE_POWER_IDLE_STOP



#endif // #ifndef _DRV_TMR_CONFIG_TEMPLATE_H

/*******************************************************************************
 End of File
*/
