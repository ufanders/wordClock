/*******************************************************************************
  USB Driver Feature Variant Implementations

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usb_variant_mapping.h

  Summary:
    USB Driver Feature Variant Implementations

  Description:
    This file implements the functions which differ based on different parts
    and various implementations of the same feature.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip  Technology  Inc.   All  rights  reserved.

Microchip licenses to  you  the  right  to  use,  modify,  copy  and  distribute
Software only when embedded on a Microchip  microcontroller  or  digital  signal
controller  that  is  integrated  into  your  product  or  third  party  product
(pursuant to the  sublicense  terms  in  the  accompanying  license  agreement).

You should refer  to  the  license  agreement  accompanying  this  Software  for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
//DOM-IGNORE-END

#ifndef _DRV_USB_VARIANT_MAPPING_H
#define _DRV_USB_VARIANT_MAPPING_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#if !defined(DRV_USB_INSTANCES_NUMBER)

    #if defined(DRV_USB_CLIENTS_NUMBER)

    /* Multi client operation in static is not supported */

    #error "This driver does not support static multi-client operation"
    
    #else
    
        /* Map internal macros and functions to the static 
         * single open variant */
        
        #include "driver/usb/drv_usb_static.h"

    #endif

    #define _DRV_USB_IS_STATIC

#else 

    /* This means that dynamic operation is requested */

    #define _DRV_USB_IS_DYNAMIC

#endif

// *****************************************************************************
// *****************************************************************************
// Section: USB Driver Static Object Generation
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: _DRV_USB_OBJ_MAKE_NAME(name)

  Summary:
    Creates an instance-specific static object name.

  Description:
     This macro creates the instance-specific name of the specified static object
     by inserting the index number into the name.

  Remarks:
    This macro does not affect the dynamic objects.
*/

#define _DRV_STATIC_OBJ_NAME_B(name,id)     name ## id
#define _DRV_STATIC_OBJ_NAME_A(name,id)     _DRV_STATIC_OBJ_NAME_B(name,id)
#define _DRV_USB_OBJ_MAKE_NAME(name)        _DRV_STATIC_OBJ_NAME_A(name, DRV_USB_INDEX)

/**********************************************
 * These macros allow variables to be compiled
 * based on dynamic or static buil.
 *********************************************/

#ifdef _DRV_USB_IS_STATIC

    #define _DRV_USB_FOR_DYNAMIC( type, object )
    #define _DRV_USB_FOR_STATIC( type, object )     type object

#endif

#ifdef _DRV_USB_IS_DYNAMIC

    #define _DRV_USB_FOR_DYNAMIC( type, object )    type object
    #define _DRV_USB_FOR_STATIC( type, object )

#endif

// *****************************************************************************
/* Interrupt Source Control

  Summary:
    Macros to enable, disable or clear the interrupt source

  Description:
    These macros enable, disable, or clear the interrupt source.

    The macros get mapped to the respective SYS module APIs if the configuration
    option DRV_USB_INTERRUPT_MODE is set to true.

  Remarks:
    These macros are mandatory.
*/

#ifndef DRV_USB_INTERRUPT_MODE
    #error "Interrupt mode must be defined and must be either true or false"
#endif

#if (DRV_USB_INTERRUPT_MODE == true)

    #define _DRV_USB_InterruptSourceEnable(source)      SYS_INT_SourceEnable( source )
    #define _DRV_USB_InterruptSourceDisable(source)     SYS_INT_SourceDisable( source )
    #define _DRV_USB_InterruptSourceClear(source)       SYS_INT_SourceStatusClear( source )

    #define _DRV_USB_InterruptSourceStatusGet(source)   SYS_INT_SourceStatusGet( source )
    #define _DRV_USB_InterruptSourceStatusSet(source)   SYS_INT_SourceStatusSet( source )
 
#endif

#if (DRV_USB_INTERRUPT_MODE == false)

    #define _DRV_USB_InterruptSourceEnable(source)
    #define _DRV_USB_InterruptSourceDisable(source)
    #define _DRV_USB_InterruptSourceClear(source)       SYS_INT_SourceStatusClear( source )
    #define _DRV_USB_InterruptSourceStatusGet(source)   SYS_INT_SourceStatusGet( source )
    #define DRV_USB_Tasks(x)                            DRV_USB_Tasks_ISR(x)

#endif

/**********************************************
 * Sets up driver mode-specific init routine
 * based on selected support.
 *********************************************/

#ifndef DRV_USB_DEVICE_SUPPORT
    #error "DRV_USB_DEVICE_SUPPORT must be defined and be either true or false"
#endif

#ifndef DRV_USB_HOST_SUPPORT
    #error "DRV_USB_HOST_SUPPORT must be defined and be either true or false"
#endif

#if (DRV_USB_DEVICE_SUPPORT == true)
    #define _DRV_USB_DEVICE_INIT(x, y)      _DRV_USB_DEVICE_Initialize(x , y)
    #define _DRV_USB_DEVICE_TASKS_ISR(x)    _DRV_USB_DEVICE_Tasks_ISR(x)
    #define _DRV_USB_FOR_DEVICE(x, y)       x y
#elif (DRV_USB_DEVICE_SUPPORT == false)
    #define _DRV_USB_DEVICE_INIT(x, y)  
    #define _DRV_USB_DEVICE_TASKS_ISR(x) 
    #define _DRV_USB_FOR_DEVICE(x, y)       
#endif
 
#if (DRV_USB_HOST_SUPPORT == true)
    #define _DRV_USB_HOST_INIT(x, y)    _DRV_USB_HOST_Initialize(x , y)
    #define _DRV_USB_HOST_TASKS_ISR(x)  _DRV_USB_HOST_Tasks_ISR(x)
    #define _DRV_USB_FOR_HOST(x, y)     x y
#elif (DRV_USB_HOST_SUPPORT == false)
    #define _DRV_USB_HOST_INIT(x, y)  
    #define _DRV_USB_HOST_TASKS_ISR(x) 
    #define _DRV_USB_FOR_HOST(x, y) 
#endif

#endif
