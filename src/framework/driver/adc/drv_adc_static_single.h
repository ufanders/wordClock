/*******************************************************************************
  ADC Driver Interface Definition for static single instance driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_adc_static_single.h
     
  Summary:
    ADC Driver interface definition for static single instance driver.

  Description:
    The ADC device driver provides a simple interface to manage the ADC
    modules on Microchip microcontrollers.  This file defines the interface 
    definition for the ADC driver.
    
  Remarks:
    Static interfaces encorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    
    Static single-open interfaces also eliminate the need for the open handle.
    
    Static multi-open interfaces do not eliminat the open handle as it is 
    necessary to identify which client is calling.
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

#ifndef _DRV_ADC_STATIC_SINGLE_H
#define _DRV_ADC_STATIC_SINGLE_H


// *****************************************************************************
// *****************************************************************************
// Section: Interface Prototypes for Instance 0 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_ADC0_Initialize ( const SYS_MODULE_INIT * const init ) ;

void DRV_ADC0_Reinitialize ( const SYS_MODULE_INIT * const init ) ;

void DRV_ADC0_Deinitialize ( void ) ;

SYS_STATUS DRV_ADC0_Status ( void ) ;

void DRV_ADC0_Tasks ( void ) ;

void DRV_ADC0_Open ( DRV_IO_INTENT ioIntent ) ;

void DRV_ADC0_Close ( void ) ;

DRV_ADC_CLIENT_STATUS DRV_ADC0_ClientStatus ( void ) ;

void DRV_ADC0_Start ( void ) ;

void DRV_ADC0_Stop ( void ) ;

void DRV_ADC0_InputsRegister ( uint32_t inputsMask ) ;

bool DRV_ADC0_SamplesAvailable ( void );

unsigned short DRV_ADC0_SamplesRead ( ADC_SAMPLE * buffer, unsigned short bufferSize) ;

unsigned short DRV_ADC0_SamplesReadLatest ( ADC_SAMPLE * buffer, unsigned short bufferSize) ;


// *****************************************************************************
// *****************************************************************************
// Section: Interface Prototypes for Instance 1 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_ADC1_Initialize ( const SYS_MODULE_INIT * const init ) ;

void DRV_ADC1_Reinitialize ( const SYS_MODULE_INIT * const init ) ;

void DRV_ADC1_Deinitialize ( void ) ;

SYS_STATUS DRV_ADC1_Status ( void ) ;

void DRV_ADC1_Tasks ( void ) ;

void DRV_ADC1_Open ( DRV_IO_INTENT ioIntent ) ;

void DRV_ADC1_Close ( void ) ;

DRV_ADC_CLIENT_STATUS DRV_ADC1_ClientStatus ( void ) ;

void DRV_ADC1_Start ( void ) ;

void DRV_ADC1_Stop ( void ) ;

void DRV_ADC1_InputsRegister ( uint32_t inputsMask ) ;

bool DRV_ADC1_SamplesAvailable ( void );

unsigned short DRV_ADC1_SamplesRead ( ADC_SAMPLE * buffer, unsigned short bufferSize) ;

unsigned short DRV_ADC1_SamplesReadLatest ( ADC_SAMPLE * buffer, unsigned short bufferSize) ;


// *****************************************************************************
// *****************************************************************************
// Section: Interface Prototypes for Instance 2 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_ADC2_Initialize ( const SYS_MODULE_INIT * const init ) ;

void DRV_ADC2_Reinitialize ( const SYS_MODULE_INIT * const init ) ;

void DRV_ADC2_Deinitialize ( void ) ;

SYS_STATUS DRV_ADC2_Status ( void ) ;

void DRV_ADC2_Tasks ( void ) ;

void DRV_ADC2_Open ( DRV_IO_INTENT ioIntent ) ;

void DRV_ADC2_Close ( void ) ;

DRV_ADC_CLIENT_STATUS DRV_ADC2_ClientStatus ( void ) ;

void DRV_ADC2_Start ( void ) ;

void DRV_ADC2_Stop ( void ) ;

void DRV_ADC2_InputsRegister ( uint32_t inputsMask ) ;

bool DRV_ADC2_SamplesAvailable ( void );

unsigned short DRV_ADC2_SamplesRead ( ADC_SAMPLE * buffer, unsigned short bufferSize) ;

unsigned short DRV_ADC2_SamplesReadLatest ( ADC_SAMPLE * buffer, unsigned short bufferSize) ;


#endif // #ifndef _DRV_ADC_STATIC_SINGLE_H

/*******************************************************************************
 End of File
*/

