/*******************************************************************************
  TMR Driver Interface Declarations for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_tmr_static_single.h

  Summary:
    Timer driver interface declarations for the static single instance driver.

  Description:
    The Timer device driver provides a simple interface to manage the Timer
    modules on Microchip microcontrollers. This file defines the interface
    Declarations for the Timer driver.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    
    Static single-open interfaces also eliminate the need for the open handle.
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

#ifndef _DRV_TMR_STATIC_SINGLE_H
#define _DRV_TMR_STATIC_SINGLE_H


// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 0 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_TMR0_Initialize( const SYS_MODULE_INIT * const init);

void DRV_TMR0_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_TMR0_Deinitialize( void );

SYS_STATUS DRV_TMR0_Status( void );

void DRV_TMR0_Tasks ( void );

void DRV_TMR0_Open( const DRV_IO_INTENT intent );

void DRV_TMR0_Close( void );

DRV_TMR_CLIENT_STATUS DRV_TMR0_ClientStatus( void );

void DRV_TMR0_Period8BitSet ( uint8_t value );

void DRV_TMR0_Period16BitSet ( uint16_t value );

void DRV_TMR0_Period32BitSet ( uint32_t value );

uint8_t DRV_TMR0_Period8BitGet ( void );

uint16_t DRV_TMR0_Period16BitGet ( void );

uint32_t DRV_TMR0_Period32BitGet ( void );

void DRV_TMR0_Counter8BitSet ( uint8_t value );

void DRV_TMR0_Counter16BitSet ( uint16_t value );

void DRV_TMR0_Counter32BitSet ( uint32_t value );

uint8_t DRV_TMR0_Counter8BitGet ( void );

uint16_t DRV_TMR0_Counter16BitGet ( void );

uint32_t DRV_TMR0_Counter32BitGet ( void );

void DRV_TMR0_SyncModeSet ( DRV_TMR_SYNC_MODE newMode );

DRV_TMR_SYNC_MODE DRV_TMR0_SyncModeGet ( void );

void DRV_TMR0_Start ( void );

void DRV_TMR0_Stop ( void );

uint32_t DRV_TMR0_OperatingFrequencyGet ( void );

uint32_t DRV_TMR0_TickFrequencyGet ( void );

bool DRV_TMR0_ElapsedStatusGetAndClear( void );

void DRV_TMR0_AlarmSet ( const DRV_TMR_ALARM_CONFIG *config);

unsigned int DRV_TMR0_AlarmCountGet ( void );

void DRV_TMR0_AlarmCountClear ( void );

unsigned int DRV_TMR0_VersionGet( void );

char * DRV_TMR0_VersionStrGet ( void );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 1 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_TMR1_Initialize( const SYS_MODULE_INIT * const init);

void DRV_TMR1_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_TMR1_Deinitialize( void );

SYS_STATUS DRV_TMR1_Status( void );

void DRV_TMR1_Tasks ( void );

void DRV_TMR1_Open( const DRV_IO_INTENT intent );

void DRV_TMR1_Close( void );

DRV_TMR_CLIENT_STATUS DRV_TMR1_ClientStatus( void );

void DRV_TMR1_Period8BitSet ( uint8_t value );

void DRV_TMR1_Period16BitSet ( uint16_t value );

void DRV_TMR1_Period32BitSet ( uint32_t value );

uint8_t DRV_TMR1_Period8BitGet ( void );

uint16_t DRV_TMR1_Period16BitGet ( void );

uint32_t DRV_TMR1_Period32BitGet ( void );

void DRV_TMR1_Counter8BitSet ( uint8_t value );

void DRV_TMR1_Counter16BitSet ( uint16_t value );

void DRV_TMR1_Counter32BitSet ( uint32_t value );

uint8_t DRV_TMR1_Counter8BitGet ( void );

uint16_t DRV_TMR1_Counter16BitGet ( void );

uint32_t DRV_TMR1_Counter32BitGet ( void );

void DRV_TMR1_SyncModeSet ( DRV_TMR_SYNC_MODE newMode );

DRV_TMR_SYNC_MODE DRV_TMR1_SyncModeGet ( void );

void DRV_TMR1_Start ( void );

void DRV_TMR1_Stop ( void );

uint32_t DRV_TMR1_OperatingFrequencyGet ( void );

uint32_t DRV_TMR1_TickFrequencyGet ( void );

bool DRV_TMR1_ElapsedStatusGetAndClear( void );

void DRV_TMR1_AlarmSet ( const DRV_TMR_ALARM_CONFIG *config);

unsigned int DRV_TMR1_AlarmCountGet ( void );

void DRV_TMR1_AlarmCountClear ( void );

unsigned int DRV_TMR1_VersionGet( void );

char * DRV_TMR1_VersionStrGet ( void );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 2 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_TMR2_Initialize( const SYS_MODULE_INIT * const init);

void DRV_TMR2_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_TMR2_Deinitialize( void );

SYS_STATUS DRV_TMR2_Status( void );

void DRV_TMR2_Tasks ( void );

void DRV_TMR2_Open( const DRV_IO_INTENT intent );

void DRV_TMR2_Close( void );

DRV_TMR_CLIENT_STATUS DRV_TMR2_ClientStatus( void );

void DRV_TMR2_Period8BitSet ( uint8_t value );

void DRV_TMR2_Period16BitSet ( uint16_t value );

void DRV_TMR2_Period32BitSet ( uint32_t value );

uint8_t DRV_TMR2_Period8BitGet ( void );

uint16_t DRV_TMR2_Period16BitGet ( void );

uint32_t DRV_TMR2_Period32BitGet ( void );

void DRV_TMR2_Counter8BitSet ( uint8_t value );

void DRV_TMR2_Counter16BitSet ( uint16_t value );

void DRV_TMR2_Counter32BitSet ( uint32_t value );

uint8_t DRV_TMR2_Counter8BitGet ( void );

uint16_t DRV_TMR2_Counter16BitGet ( void );

uint32_t DRV_TMR2_Counter32BitGet ( void );

void DRV_TMR2_SyncModeSet ( DRV_TMR_SYNC_MODE newMode );

DRV_TMR_SYNC_MODE DRV_TMR2_SyncModeGet ( void );

void DRV_TMR2_Start ( void );

void DRV_TMR2_Stop ( void );

uint32_t DRV_TMR2_OperatingFrequencyGet ( void );

uint32_t DRV_TMR2_TickFrequencyGet ( void );

bool DRV_TMR2_ElapsedStatusGetAndClear( void );

void DRV_TMR2_AlarmSet ( const DRV_TMR_ALARM_CONFIG *config);

unsigned int DRV_TMR2_AlarmCountGet ( void );

void DRV_TMR2_AlarmCountClear ( void );

unsigned int DRV_TMR2_VersionGet( void );

char * DRV_TMR2_VersionStrGet ( void );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 3 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_TMR3_Initialize( const SYS_MODULE_INIT * const init);

void DRV_TMR3_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_TMR3_Deinitialize( void );

SYS_STATUS DRV_TMR3_Status( void );

void DRV_TMR3_Tasks ( void );

void DRV_TMR3_Open( const DRV_IO_INTENT intent );

void DRV_TMR3_Close( void );

DRV_TMR_CLIENT_STATUS DRV_TMR3_ClientStatus( void );

void DRV_TMR3_Period8BitSet ( uint8_t value );

void DRV_TMR3_Period16BitSet ( uint16_t value );

void DRV_TMR3_Period32BitSet ( uint32_t value );

uint8_t DRV_TMR3_Period8BitGet ( void );

uint16_t DRV_TMR3_Period16BitGet ( void );

uint32_t DRV_TMR3_Period32BitGet ( void );

void DRV_TMR3_Counter8BitSet ( uint8_t value );

void DRV_TMR3_Counter16BitSet ( uint16_t value );

void DRV_TMR3_Counter32BitSet ( uint32_t value );

uint8_t DRV_TMR3_Counter8BitGet ( void );

uint16_t DRV_TMR3_Counter16BitGet ( void );

uint32_t DRV_TMR3_Counter32BitGet ( void );

void DRV_TMR3_SyncModeSet ( DRV_TMR_SYNC_MODE newMode );

DRV_TMR_SYNC_MODE DRV_TMR3_SyncModeGet ( void );

void DRV_TMR3_Start ( void );

void DRV_TMR3_Stop ( void );

uint32_t DRV_TMR3_OperatingFrequencyGet ( void );

uint32_t DRV_TMR3_TickFrequencyGet ( void );

bool DRV_TMR3_ElapsedStatusGetAndClear( void );

void DRV_TMR3_AlarmSet ( const DRV_TMR_ALARM_CONFIG *config);

unsigned int DRV_TMR3_AlarmCountGet ( void );

void DRV_TMR3_AlarmCountClear ( void );

unsigned int DRV_TMR3_VersionGet( void );

char * DRV_TMR3_VersionStrGet ( void );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 4 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_TMR4_Initialize( const SYS_MODULE_INIT * const init);

void DRV_TMR4_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_TMR4_Deinitialize( void );

SYS_STATUS DRV_TMR4_Status( void );

void DRV_TMR4_Tasks ( void );

void DRV_TMR4_Open( const DRV_IO_INTENT intent );

void DRV_TMR4_Close( void );

DRV_TMR_CLIENT_STATUS DRV_TMR4_ClientStatus( void );

void DRV_TMR4_Period8BitSet ( uint8_t value );

void DRV_TMR4_Period16BitSet ( uint16_t value );

void DRV_TMR4_Period32BitSet ( uint32_t value );

uint8_t DRV_TMR4_Period8BitGet ( void );

uint16_t DRV_TMR4_Period16BitGet ( void );

uint32_t DRV_TMR4_Period32BitGet ( void );

void DRV_TMR4_Counter8BitSet ( uint8_t value );

void DRV_TMR4_Counter16BitSet ( uint16_t value );

void DRV_TMR4_Counter32BitSet ( uint32_t value );

uint8_t DRV_TMR4_Counter8BitGet ( void );

uint16_t DRV_TMR4_Counter16BitGet ( void );

uint32_t DRV_TMR4_Counter32BitGet ( void );

void DRV_TMR4_SyncModeSet ( DRV_TMR_SYNC_MODE newMode );

DRV_TMR_SYNC_MODE DRV_TMR4_SyncModeGet ( void );

void DRV_TMR4_Start ( void );

void DRV_TMR4_Stop ( void );

uint32_t DRV_TMR4_OperatingFrequencyGet ( void );

uint32_t DRV_TMR4_TickFrequencyGet ( void );

bool DRV_TMR4_ElapsedStatusGetAndClear( void );

void DRV_TMR4_AlarmSet ( const DRV_TMR_ALARM_CONFIG *config);

unsigned int DRV_TMR4_AlarmCountGet ( void );

void DRV_TMR4_AlarmCountClear ( void );

unsigned int DRV_TMR4_VersionGet( void );

char * DRV_TMR4_VersionStrGet ( void );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance 5 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_TMR5_Initialize( const SYS_MODULE_INIT * const init);

void DRV_TMR5_Reinitialize( const SYS_MODULE_INIT * const init);

void DRV_TMR5_Deinitialize( void );

SYS_STATUS DRV_TMR5_Status( void );

void DRV_TMR5_Tasks ( void );

void DRV_TMR5_Open( const DRV_IO_INTENT intent );

void DRV_TMR5_Close( void );

DRV_TMR_CLIENT_STATUS DRV_TMR5_ClientStatus( void );

void DRV_TMR5_Period8BitSet ( uint8_t value );

void DRV_TMR5_Period16BitSet ( uint16_t value );

void DRV_TMR5_Period32BitSet ( uint32_t value );

uint8_t DRV_TMR5_Period8BitGet ( void );

uint16_t DRV_TMR5_Period16BitGet ( void );

uint32_t DRV_TMR5_Period32BitGet ( void );

void DRV_TMR5_Counter8BitSet ( uint8_t value );

void DRV_TMR5_Counter16BitSet ( uint16_t value );

void DRV_TMR5_Counter32BitSet ( uint32_t value );

uint8_t DRV_TMR5_Counter8BitGet ( void );

uint16_t DRV_TMR5_Counter16BitGet ( void );

uint32_t DRV_TMR5_Counter32BitGet ( void );

void DRV_TMR5_SyncModeSet ( DRV_TMR_SYNC_MODE newMode );

DRV_TMR_SYNC_MODE DRV_TMR5_SyncModeGet ( void );

void DRV_TMR5_Start ( void );

void DRV_TMR5_Stop ( void );

uint32_t DRV_TMR5_OperatingFrequencyGet ( void );

uint32_t DRV_TMR5_TickFrequencyGet ( void );

bool DRV_TMR5_ElapsedStatusGetAndClear( void );

void DRV_TMR5_AlarmSet ( const DRV_TMR_ALARM_CONFIG *config);

unsigned int DRV_TMR5_AlarmCountGet ( void );

void DRV_TMR5_AlarmCountClear ( void );

unsigned int DRV_TMR5_VersionGet( void );

char * DRV_TMR5_VersionStrGet ( void );


#endif // #ifndef _DRV_TMR_STATIC_SINGLE_H

/*******************************************************************************
 End of File
*/
