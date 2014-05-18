/*******************************************************************************
  SD Card Driver Feature Variant Implementations

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdcard_variant_mapping.h

  Summary:
    SD Card Driver Feature Variant Implementations

  Description:
    This file implements the functions which differ based on different parts
    and various implementations of the same feature.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_SDCARD_VARIANT_MAPPING_H
#define _DRV_SDCARD_VARIANT_MAPPING_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#if !defined(DRV_SDCARD_INSTANCES_NUMBER)

    #if defined(DRV_SDCARD_CLIENTS_NUMBER)
    
        /* Map internal macros and functions to the static multi open variant */
        #include "sdcard/src/static/drv_sdcard_hw_static.h"
        #include "sdcard/src/client_single/drv_sdcard_client_multi.h"
    
    #else
    
        /* Map internal macros and functions to the static single open variant */
        #include "sdcard/src/static/drv_sdcard_hw_static.h"
        #include "sdcard/src/client_single/drv_sdcard_client_single.h"

    #endif
    
#else // (DRV_SDCARD_INSTANCES_NUMBER > 1)

    /* Map internal macros and functions to the dynamic variant */
    //#include "driver/sdcard/src/dynamic/drv_sdcard_hw_dynamic.h"
    //#include "driver/sdcard/src/client_multi/drv_sdcard_client_multi.h"

#endif


// *****************************************************************************
// *****************************************************************************
// Section: SD Card Driver Static Object Generation
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: _DRV_SDCARD_OBJ_MAKE_NAME(name)

  Summary:
    Creates an instance-specific static object name

  Description:
     This macro creates the instance-specific name of the given static object
     by inserting the index number into the name.

  Remarks:
    This macro does not affect the dynamic objects
*/

#define _DRV_STATIC_OBJ_NAME_B(name,id)     name ## id

#define _DRV_STATIC_OBJ_NAME_A(name,id)     _DRV_STATIC_OBJ_NAME_B(name,id)

#define _DRV_SDCARD_OBJ_MAKE_NAME(name)     _DRV_STATIC_OBJ_NAME_A(name, DRV_SDCARD_INDEX)


// *****************************************************************************
// *****************************************************************************
// Section: Feature Variant Mapping
// *****************************************************************************
// *****************************************************************************
/* Some variants are determined by hardware feature existence, some features
   are determined user configuration of the driver, and some variants are
   combination of the two.
*/

// *****************************************************************************
/* PLIB ID Static Configuration Override

  Summary:
    Allows static override of the peripehral library ID

  Description:
    These macros allow the peripheral library ID to be statically overriden by 
    the DRV_SDCARD_PERIPHERAL_ID configuration macro, if it is defined.
    
    _DRV_SDCARD_PERIPHERAL_ID_GET replaces the value passed in with the value 
    defined by the DRV_SDCARD_PERIPHERAL_ID configuration option.
    
    _DRV_SDCARD_STATIC_PLIB_ID removes any statement passed into it from the 
    build if the DRV_SDCARD_PERIPHERAL_ID configuration option is defined.
*/

#if defined(DRV_SDCARD_PERIPHERAL_ID)

    #define _DRV_SDCARD_PERIPHERAL_ID_GET(plibId)      DRV_SDCARD_PERIPHERAL_ID
    #define _DRV_SDCARD_STATIC_PLIB_ID(any)

#else

    #define _DRV_SDCARD_PERIPHERAL_ID_GET(plibId)      plibId
    #define _DRV_SDCARD_STATIC_PLIB_ID(any)            any

#endif


// *****************************************************************************
/* Interrupt Source Static Configuration Override

  Summary:
    Allows static override of the interrupt source

  Description:
    These macros allow the interrupt source to be statically overriden by the 
    DRV_SDCARD_INTERRUPT_SOURCE configuration macro, if it is defined.
    
    _DRV_SDCARD_INT_SRC_GET replaces the value passed in with the value defined 
    by the DRV_SDCARD_INTERRUPT_SOURCE configuration option.
    
    _DRV_SDCARD_STATIC_INT_SRC removes any statement passed into it from the 
    build if the DRV_SDCARD_INTERRUPT_SOURCE configuration option is defined.
*/

#if defined(DRV_SDCARD_INTERRUPT_SOURCE)

    #define _DRV_SDCARD_INT_SRC_GET(source)    DRV_SDCARD_INTERRUPT_SOURCE
    #define _DRV_SDCARD_STATIC_INT_SRC(any)

#else

    #define _DRV_SDCARD_INT_SRC_GET(source)    source
    #define _DRV_SDCARD_STATIC_INT_SRC(any)    any

#endif

#define SDCARD_MODULE_ID       SPI_MODULE_ID



#if defined (DRV_SDCARD_MEDIA_SOFT_DETECT)

    #define _DRV_SDCARD_IS_PIN_ENABLED(a)   SYS_ASSERT ( "Detect pin is not supported" )
    #define _DRV_SDCARD_MediaDetect(a)      _DRV_SDCARD_MediaCommandDetect ( a )
#else
    #define  _DRV_SDCARD_IS_PIN_ENABLED(a)
    #define _DRV_SDCARD_MediaDetect(a)      _DRV_SDCARD_MediaPinDetect(a)
#endif


            
#endif //_DRV_SDCARD_VARIANT_MAPPING_H

/*******************************************************************************
 End of File
*/

