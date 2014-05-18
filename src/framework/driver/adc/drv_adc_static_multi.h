/*******************************************************************************
  ADC Driver Interface Definition for static multi instance driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usart_static_multi.h

  Summary:
    ADC Driver interface definition for the static multi-instance driver.

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

#ifndef _DRV_ADC_STATIC_MULTI_H
#define _DRV_ADC_STATIC_MULTI_H


// *****************************************************************************
// *****************************************************************************
// Section: Interface Prototypes for Instance 0 for the static single open driver
// *****************************************************************************
// *****************************************************************************

void DRV_ADC0multi_Initialize ( const SYS_MODULE_INIT * const init ) ;

void DRV_ADC0multi_Reinitialize ( const SYS_MODULE_INIT * const init ) ;

void DRV_ADC0multi_Deinitialize ( void ) ;

SYS_STATUS DRV_ADC0multi_Status ( void ) ;

void DRV_ADC0multi_Tasks ( void ) ;

DRV_HANDLE DRV_ADC0multi_Open ( DRV_IO_INTENT ioIntent ) ;

void DRV_ADC0multi_Close ( DRV_HANDLE handle ) ;

DRV_ADC_CLIENT_STATUS DRV_ADC0multi_ClientStatus ( DRV_HANDLE handle ) ;

void DRV_ADC0multi_Start ( DRV_HANDLE handle ) ;

void DRV_ADC0multi_Stop ( DRV_HANDLE handle ) ;

void DRV_ADC0multi_InputsRegister ( DRV_HANDLE handle , uint32_t inputsMask ) ;

bool DRV_ADC0multi_SamplesAvailable ( DRV_HANDLE handle );

unsigned short DRV_ADC0multi_SamplesRead ( DRV_HANDLE handle, 
                                           ADC_SAMPLE * buffer,
                                           unsigned short bufferSize) ;

unsigned short DRV_ADC0multi_SamplesReadLatest ( DRV_HANDLE handle, 
                                                 ADC_SAMPLE * buffer,
                                                 unsigned short bufferSize) ;


// *****************************************************************************
// *****************************************************************************
// Section: Prototypes Headers for Instance 1 for the static multi open driver
// *****************************************************************************
// *****************************************************************************

void DRV_ADC1multi_Initialize ( const SYS_MODULE_INIT * const init ) ;

void DRV_ADC1multi_Reinitialize ( const SYS_MODULE_INIT * const init ) ;

void DRV_ADC1multi_Deinitialize ( void ) ;

SYS_STATUS DRV_ADC1multi_Status ( void ) ;

void DRV_ADC1multi_Tasks ( void ) ;

DRV_HANDLE DRV_ADC1multi_Open ( DRV_IO_INTENT ioIntent ) ;

void DRV_ADC1multi_Close ( DRV_HANDLE handle ) ;

DRV_ADC_CLIENT_STATUS DRV_ADC1multi_ClientStatus ( DRV_HANDLE handle ) ;

void DRV_ADC1multi_Start ( DRV_HANDLE handle ) ;

void DRV_ADC1multi_Stop ( DRV_HANDLE handle ) ;

void DRV_ADC1multi_InputsRegister ( DRV_HANDLE handle , uint32_t inputsMask ) ;

bool DRV_ADC1multi_SamplesAvailable ( DRV_HANDLE handle );

unsigned short DRV_ADC1multi_SamplesRead ( DRV_HANDLE handle,
                                           ADC_SAMPLE * buffer,
                                           unsigned short bufferSize) ;

unsigned short DRV_ADC1multi_SamplesReadLatest ( DRV_HANDLE handle,
                                                 ADC_SAMPLE * buffer,
                                                 unsigned short bufferSize) ;


// *****************************************************************************
// *****************************************************************************
// Section: Interface Prototypes for Instance 2 for the static multi open driver
// *****************************************************************************
// *****************************************************************************

void DRV_ADC2multi_Initialize ( const SYS_MODULE_INIT * const init ) ;

void DRV_ADC2multi_Reinitialize ( const SYS_MODULE_INIT * const init ) ;

void DRV_ADC2multi_Deinitialize ( void ) ;

SYS_STATUS DRV_ADC2multi_Status ( void ) ;

void DRV_ADC2multi_Tasks ( void ) ;

DRV_HANDLE DRV_ADC2multi_Open ( DRV_IO_INTENT ioIntent ) ;

void DRV_ADC2multi_Close ( DRV_HANDLE handle ) ;

DRV_ADC_CLIENT_STATUS DRV_ADC2multi_ClientStatus ( DRV_HANDLE handle ) ;

void DRV_ADC2multi_Start ( DRV_HANDLE handle ) ;

void DRV_ADC2multi_Stop ( DRV_HANDLE handle ) ;

void DRV_ADC2multi_InputsRegister ( DRV_HANDLE handle , uint32_t inputsMask ) ;

bool DRV_ADC2multi_SamplesAvailable ( DRV_HANDLE handle );

unsigned short DRV_ADC2multi_SamplesRead ( DRV_HANDLE handle,
                                           ADC_SAMPLE * buffer,
                                           unsigned short bufferSize) ;

unsigned short DRV_ADC2multi_SamplesReadLatest ( DRV_HANDLE handle,
                                                 ADC_SAMPLE * buffer,
                                                 unsigned short bufferSize) ;


#endif // #ifndef _DRV_ADC_STATIC_MULTI_H

/*******************************************************************************
 End of File
*/

