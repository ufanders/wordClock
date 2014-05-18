/*******************************************************************************
  ADC Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_adc_local.h
  
  Summary:
    ADC Driver local data structures.

  Description:
    This file defines the ADC driver local data structures.
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


#ifndef _DRV_ADC_LOCAL_H
#define _DRV_ADC_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "driver/adc/drv_adc.h"
#include "driver/adc/src/drv_adc_variant_mapping.h"


// *****************************************************************************
// *****************************************************************************
// Section: TMR Driver Static Object Generation
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: _DRV_ADC_OBJ_MAKE_NAME(name)

  Summary:
    Creates an instance-specific static object name.

  Description:
     This macro creates the instance-specific name of the given static object
     by inserting the index number into the name.

  Remarks:
    This macro does not affect the dynamic objects.
*/

#define _DRV_STATIC_OBJ_NAME_B(name,id)                 name ## id

#define _DRV_STATIC_OBJ_NAME_A(name,id)                 _DRV_STATIC_OBJ_NAME_B(name,id)

#define _DRV_ADC_OBJ_MAKE_NAME(name)                    _DRV_STATIC_OBJ_NAME_A(name, DRV_ADC_INDEX)


// *****************************************************************************
// *****************************************************************************
// Section: Symbolic Constant Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro:
    _DRV_ADC_NUMBER_OF_SAMPLE_SETS

  Summary:
    Defines the number of sample sets available based on the configuration.

  Description:
    This macro defines the number of sample sets available based on the 
    configuration selected by the macros DRV_ADC_INTERNAL_BUFFER_SIZE and
    DRV_ADC_SAMPLES_PER_INTERRUPT

  Remarks:
    None
*/

#define _DRV_ADC_NUMBER_OF_SAMPLE_SETS          (DRV_ADC_INTERNAL_BUFFER_SIZE/DRV_ADC_SAMPLES_PER_INTERRUPT)


// *****************************************************************************
/* Macro:
    DRV_ADC_RESULT_BUFFER_SIZE

  Summary:
    Defines the result buffer size.

  Description:
    This macro is used to define the result buffer.

  Remarks:
    None.
*/

#define DRV_ADC_RESULT_BUFFER_SIZE              (DRV_ADC_SAMPLES_PER_INTERRUPT * _DRV_ADC_NUMBER_OF_SAMPLE_SETS)


// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* ADC Driver Hardware Instance Object

  Summary:
    Defines the object required for the maintainence of the hardware instance.

  Description:
    This macro defines the object required for the maintainence of the hardware
    instance. This object exists once per hardware instance of the peripheral.

  Remarks:
    None.
*/

typedef struct {

    /* Interrupt Source */
    INT_SOURCE                      interruptSource;

    /* The status of the driver */
    SYS_STATUS                      status ;

    /* The hardware base address/index associated with the device instance */
    ADC_MODULE_ID                   adcId ;

    /* configured number of samples per interrupt */
    ADC_SAMPLES_PER_INTERRUPT       samplesPerInterrupt ;

    /* Inputs Mask */
    uint32_t                        inputsMask ;

    /* Initialization Flags */
    DRV_ADC_INIT_FLAGS              initFlags;

    /* The position where the ADC driver writes the sample */
    uint8_t                         currentSampleSet;

    /* The position where the ADC driver writes the sample next time */
    uint8_t                         nextSampleSet;

    /* ADC Result Buffer */
    ADC_SAMPLE                      resultBuffer[DRV_ADC_RESULT_BUFFER_SIZE];

    /* Array to indicate which clients want which sample set.
       Restricts to max 8 clients right now, can make this optional to increase to 16, 32, 64 */
    uint8_t                         readRequestFlag[_DRV_ADC_NUMBER_OF_SAMPLE_SETS];

    /* Flag to indicate if the sample set Q has looped around */
    uint8_t                         loopedAround;

    /* number of clients connected to the hardware instance */
    uint8_t                         numClients;

    /* Flag to indicate ADC object usage */
    bool                            inUse;

    /* Flag to indicate that ADC is used in exclusive access mode */
    bool                            adcInExclusiveAccess;

    /* Flag to indicate that ADC samples are available */
    bool                            adcSamplesAvailable;

} DRV_ADC_OBJECT ;


// To be removed later & the above line be replaced with
// } DRV_ADC_OBJECT, * DRV_ADC_OBJ;
typedef unsigned short int DRV_ADC_OBJ;


// *****************************************************************************
/* ADC Driver Client Object

  Summary:
    Defines the object required for the maintainence of the software clients.

  Description:
    This defines the object required for the maintainence of the software
    clients instance. This object exists once per client instance.

  Remarks:
    None
*/

typedef struct
{
    /* Status of the client object */
    DRV_ADC_CLIENT_STATUS           status ;

    /* The intent with which the client was opened */
    DRV_IO_INTENT                   intent ;

    /* The index of the hardware instance object associate with the client */
    DRV_ADC_OBJ                     driverObject;

    /* The position where the ADC driver writes the sample */
    uint8_t                         clientSampleSet;

    /* The position from where the client reads the sample */
    uint8_t                         currentReadIndex;

    /* Client Mask */
    uint8_t                         accessMask;

    /* Flag to indicate Client usage */
    bool                            inUse;

} DRV_ADC_CLIENT_OBJECT ;

// To be removed later & the above line be replaced with
// } DRV_ADC_CLIENT_OBJECT, * DRV_ADC_CLIENT_OBJ;
typedef unsigned short int DRV_ADC_CLIENT_OBJ;


// *****************************************************************************
// *****************************************************************************
// Section: Macros
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro:
    _DRV_ADC_SampleSetNextGet(curSampleSet)

  Summary:
    Gives out the next available sample set.

  Description:
    This macro gives out the next available sample set.

  Remarks:
    None
*/

#define _DRV_ADC_SampleSetNextGet(curSampleSet)             \
        ( ( ( curSampleSet + 1 ) < _DRV_ADC_NUMBER_OF_SAMPLE_SETS) ? \
          (   curSampleSet + 1 ) :                          \
          (   0 ) )


// *****************************************************************************
/* Macro:
    _DRV_ADC_SampleSetPreviousGet(curSampleSet)

  Summary:
    Gives out the valid previous sample set.

  Description:
    This macro gives out the valind previous sample set.

  Remarks:
    None
*/

#define _DRV_ADC_SampleSetPreviousGet(curSampleSet)         \
        ( ( ( curSampleSet - 1 ) != -1 ) ?                  \
          (   curSampleSet - 1 ) :                          \
          (   _DRV_ADC_NUMBER_OF_SAMPLE_SETS-1 ) )


// *****************************************************************************
// *****************************************************************************
// Section: Extern data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*  Hardware Objects for the dynamic driver
*/

extern DRV_ADC_OBJECT               gDrvADCObj[];

// *****************************************************************************
/*  Client Objects for the multi-client driver
*/

extern DRV_ADC_CLIENT_OBJECT        gDrvADCClientObj[];

// *****************************************************************************
/*  Hardware Objects for the static driver
*/

extern DRV_ADC_OBJECT               gDrvADCObj0;
extern DRV_ADC_OBJECT               gDrvADCObj1;
extern DRV_ADC_OBJECT               gDrvADCObj2;
extern DRV_ADC_OBJECT               gDrvADCObj3;
extern DRV_ADC_OBJECT               gDrvADCObj4;

// *****************************************************************************
/*  Client Objects for the single-client driver
*/

extern DRV_ADC_CLIENT_OBJECT        gDrvADCClientObj0;
extern DRV_ADC_CLIENT_OBJECT        gDrvADCClientObj1;
extern DRV_ADC_CLIENT_OBJECT        gDrvADCClientObj2;
extern DRV_ADC_CLIENT_OBJECT        gDrvADCClientObj3;
extern DRV_ADC_CLIENT_OBJECT        gDrvADCClientObj4;

// *****************************************************************************
/*  Extern definitions of functions used internally in the ADC driver
*/

extern DRV_ADC_CLIENT_OBJ _DRV_ADC_ClientObjectAllocate (SYS_MODULE_INDEX drvIndex );


#endif //#ifndef _DRV_ADC_LOCAL_H

/*******************************************************************************
 End of File
*/
