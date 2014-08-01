/*******************************************************************************
  Timer Driver Feature Variant Implementations

  Company:
    Microchip Technology Inc.

  File Name:
    drv_tmr_variant_mapping.h

  Summary:
    Timer driver feature variant implementations.

  Description:
    This file implements the functions which differ based on different devices
    and various implementations of the same feature.
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


#ifndef _DRV_TMR_VARIANT_MAPPING_H
#define _DRV_TMR_VARIANT_MAPPING_H



#if defined (DRV_TMR_INTERRUPT_MODE)

    #if(DRV_TMR_INTERRUPT_MODE == true)

        #define _DRV_TMR_InterruptSourceEnable(source)          SYS_INT_SourceEnable( source )
        #define _DRV_TMR_InterruptSourceDisable(source)         SYS_INT_SourceDisable( source )
        #define _DRV_TMR_InterruptSourceClear(source)           SYS_INT_SourceStatusClear( source )

        #define _DRV_TMR_InterruptSourceStatusGet(source)       SYS_INT_SourceStatusGet( source )

    #elif (DRV_TMR_INTERRUPT_MODE == false)

        #define _DRV_TMR_InterruptSourceEnable(source)
        #define _DRV_TMR_InterruptSourceDisable(source)
        #define _DRV_TMR_InterruptSourceClear(source)           SYS_INT_SourceStatusClear( source )

        #define _DRV_TMR_InterruptSourceStatusGet(source)       SYS_INT_SourceStatusGet( source )

    #else

        #error "No Task mode chosen at build, interrupt or polling needs to be selected. "

    #endif

#else

    #error "No Task mode chosen at build, interrupt or polling needs to be selected. "

#endif

#define _DRV_TMR_PeriodSet(index, value)    \
(                                   \
    PLIB_TMR_Period16BitSet( index, (uint16_t)value ) \
)

// *****************************************************************************
/* Macro:
    _DRV_TMR_SyncModeAsyncExtWithoutClockSync

  Summary:
    Disables synchronization with an external clock when the appropriate Timer
    bit-width mode has been configured.

  Description:
    This macro disables synchronization with an external clock when an
    appropriate timer bit-width mode has been configured.

    Note:  External clock synchronization is only appropriate when configured
           in 16-bit mode.

  Remarks:
    None.
*/

#define _DRV_TMR_SyncModeAsyncExtWithoutClockSync(index)  _DRV_TMR_ExternalClockSyncDisable( index )

    
// *****************************************************************************
/* Macro:
    _DRV_TMR_SyncModeAsyncExtWithClockSync

  Summary:
    Enables synchronization with an external clock when the appropriate Timer
    bit-width mode has been configured.

  Description:
    This macro enables synchronization with an external clock when an
    appropriate timer bit-width mode has been configured.
    
    Note:  External clock synchronization is only appropriate when configured
           in 16-bit mode.

  Remarks:
    None.
*/

#define _DRV_TMR_SyncModeAsyncExtWithClockSync(index)     _DRV_TMR_ExternalClockSyncEnable( index )

   
// *****************************************************************************
/* Alarm feature variable declaration macros

  Summary:
    Macros to declare the alarm members of the driver

  Description:
    These macros enable the alarm members presence in the driver objects.

    The configuration option DRV_TMR_ALARM_ENABLE enables them to do so.

  Remarks:
    None
*/

#if defined (DRV_TMR_ALARM_ENABLE)

    #define _DRV_TMR_DECLARE_ALARM_COUNTER(countDeclare)    countDeclare
    #define _DRV_TMR_DECLARE_ALARM_CALLBACK(cbDeclare)      cbDeclare
    #define _DRV_TMR_DECLARE_ALARM_FLAG(declare)            declare

#else // (!DRV_TMR_ALARM_ENABLE)

    #define _DRV_TMR_DECLARE_ALARM_COUNTER(countDeclare)
    #define _DRV_TMR_DECLARE_ALARM_CALLBACK(cbDeclare)
    #define _DRV_TMR_DECLARE_ALARM_FLAG(declare)

#endif


// *****************************************************************************
// *****************************************************************************
// Initializtion Parameter Static Overrides
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* Sync mode Static Configuration Override

  Summary:
    Allows static override of the sync mode.

  Description:
    These macros allow the sync mode to be statically overriden by
    the DRV_TMR_SYNCHRONIZATION_MODE configuration macro, if it is defined.

    _DRV_TMR_SYNC_MODE_GET replaces the value passed in with the value defined by
    the DRV_TMR_SYNCHRONIZATION_MODE configuration option.

   Remarks:
    None.
*/

#if defined(DRV_TMR_SYNCHRONIZATION_MODE)

    #define _DRV_TMR_SYNC_MODE_GET(arg)                     DRV_TMR_SYNCHRONIZATION_MODE

#else

    #define _DRV_TMR_SYNC_MODE_GET(mode)                    (mode)

#endif


// *****************************************************************************
/* TMR period Static Configuration Override

  Summary:
    Allows static override of the Timer period.

  Description:
    These macros allow the Timer period to be statically overriden by
    the DRV_TMR_TIMER_PERIOD configuration macro, if it is defined.

    _DRV_TMR_VALUE_GET replaces the value passed in with the value defined by
    the DRV_TMR_TIMER_PERIOD configuration option.

   Remarks:
    None.
*/

#if defined(DRV_TMR_TIMER_PERIOD)

    #define _DRV_TMR_VALUE_GET(arg)                             DRV_TMR_TIMER_PERIOD

#else

    #define _DRV_TMR_VALUE_GET(arg)                             (arg)

#endif


// *****************************************************************************
/* Source edge Static Configuration Override

  Summary:
    Allows static override of the source edge.

  Description:
    These macros allow the prescale to be statically overriden by
    the DRV_TMR_PRESCALE configuration macro, if it is defined.

    _DRV_TMR_PRESCALE_GET replaces the value passed in with the value defined by
    the DRV_TMR_PRESCALE configuration option.

   Remarks:
    None.
*/

#if defined(DRV_TMR_SOURCE_EDGE)

    #define _DRV_TMR_SOURCE_EDGE_GET(arg)                       DRV_TMR_SOURCE_EDGE

#else

    #define _DRV_TMR_SOURCE_EDGE_GET(arg)                       (arg)

#endif


// *****************************************************************************
/* Postscale Static Configuration Override

  Summary:
    Allows static override of the postscale.

  Description:
    These macros allow the postscale to be statically overriden by
    the DRV_TMR_POSTSCALE configuration macro, if it is defined.

    _DRV_TMR_POSTSCALE_GET replaces the value passed in with the value defined by
    the DRV_TMR_POSTSCALE configuration option.

   Remarks:
    None.
*/

#if defined(DRV_TMR_POSTSCALE)

    #define _DRV_TMR_POSTSCALE_GET(arg)                         DRV_TMR_POSTSCALE

#else

    #define _DRV_TMR_POSTSCALE_GET(arg)                         (arg)

#endif


// *****************************************************************************
/* Power mode Configuration Override

  Summary:
    Allows static override of the Power mode.

  Description:
    These macros allow the Power mode to be statically overriden by
    the DRV_TMR_CONFIG_INIT_POWER_MODE configuration macro, if it is defined.

    _DRV_TMR_POWER_STATE_GET replaces the value passed in with the value defined by
    the DRV_TMR_CONFIG_INIT_POWER_MODE configuration option.

   Remarks:
    None.
*/

#if defined(DRV_TMR_POWER_STATE)

    #define _DRV_TMR_POWER_STATE_GET(arg)                         DRV_TMR_POWER_STATE

#else

    #define _DRV_TMR_POWER_STATE_GET(arg)                         (arg)

#endif


#endif //_DRV_TMR_VARIANT_MAPPING_H

/*******************************************************************************
 End of File
*/

