/*******************************************************************************
  SAMPLE Driver Configuration Definitions for the template version

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sample_config_template.h

  Summary:
    SAMPLE driver configuration definitions template

  Description:
    These definitions statically define the driver's mode of operation.
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

#ifndef _DRV_SAMPLE_CONFIG_TEMPLATE_H
#define _DRV_SAMPLE_CONFIG_TEMPLATE_H


//#error "This is a configuration template file.  Do not include it directly."


// *****************************************************************************
/* SAMPLE hardware instance configuration

  Summary:
    Selects the maximum number of hardware instances that can be supported by
    the dynamic driver

  Description:
    This definition selects the maximum number of hardware instances that can be
    supported by the dynamic driver. Not defining it means using a static driver.

  Remarks:
    None
*/

#define DRV_SAMPLE_INSTANCES_NUMBER                1


// *****************************************************************************
/* SAMPLE Maximum Number of Clients

  Summary:
    Selects the miximum number of clients

  Description:
    This definition select the maximum number of clients that the SAMPLE driver can
    support at run time. Not defining it means using a single client.

  Remarks:
    None.

*/

#define DRV_SAMPLE_CLIENTS_NUMBER                1


// *****************************************************************************
/* SAMPLE Static Index Selection

  Summary:
    SAMPLE Static Index selection

  Description:
    SAMPLE Static Index selection for the driver object reference

  Remarks:
    This index is required to make a reference to the driver object
*/

#define DRV_SAMPLE_INDEX                                DRV_SAMPLE_INDEX_2


// *****************************************************************************
/* SAMPLE Interrupt And Polled Mode Operation Control

  Summary:
    Macro controls operation of the driver in the interrupt or polled mode

  Description:
    This macro controls the operation of the driver in the interrupt
    mode of operation. The possible values of this macro is

    - true  - Select if interrupt mode of spi operation is desired

    - false - Select if polling mode of spi operation is desired

    Not defining this option to true or false will result in build error.

  Remarks:
    None.
*/

#define DRV_SAMPLE_INTERRUPT_MODE          true


// *****************************************************************************
// *****************************************************************************
// Section: Initialization Overrides
// *****************************************************************************
// *****************************************************************************
/* This section defines the initialization overrides */

// *****************************************************************************
/* SAMPLE Peripheral ID Selection

  Summary:
    Defines an override of the peripheral id.

  Description:
    Defines an override of the peripheral id, using macros

  Remarks:

    Note: Some devices also support SAMPLE_ID_0
*/

#define DRV_SAMPLE_PERIPHERAL_ID                         SAMPLE_ID_1


// *****************************************************************************
/* SAMPLE Interrupt Source

  Summary:
    Defines an override of the interrupt source in case of static driver.

  Description:
    Defines an override of the interrupt source in case of static driver.

  Remarks:
    Refer to the INT PLIB document for more information on INT_SOURCE
    enumeration.

*/

#define DRV_SAMPLE_INTERRUPT_SOURCE                INT_SOURCE_SPI_1


// *****************************************************************************
/* SAMPLE power state configuration

  Summary:
    Defines an override of the power state of the SAMPLE driver.

  Description:
    Defines an override of the power state of the SAMPLE driver.

  Remarks:
    Note: This feature may not be available in the device or the SAMPLE module
    selected.
*/

#define DRV_SAMPLE_POWER_STATE                 SYS_MODULE_POWER_IDLE_STOP


#endif // #ifndef _DRV_SAMPLE_CONFIG_TEMPLATE_H

/*******************************************************************************
 End of File
*/

