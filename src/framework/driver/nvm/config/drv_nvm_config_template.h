/**********************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    drv_NVM_config_template.h

  Summary:
    NVM driver configuration definitions template
    
  Description:
    NVM Driver Configuration Definitions for the template version
    
    These definitions statically define the driver's mode of operation.
  **********************************************************************/

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

#ifndef _DRV_NVM_CONFIG_TEMPLATE_H
#define _DRV_NVM_CONFIG_TEMPLATE_H


#error "This is a configuration template file.  Do not include it directly."


// *****************************************************************************
/* NVM Driver instance configuration

  Summary:
    Selects the maximum number of Driver instances that can be supported by
    the dynamic driver.

  Description:
    This definition selects the maximum number of Driver instances that can be 
    supported by the dynamic driver. In case of this driver, multiple instances
    of the driver could use the same hardware instance. 

  Remarks:
    None
*/

#define DRV_NVM_INSTANCES_NUMBER                        1


// *****************************************************************************
/* NVM maximum number of clients

  Summary:
    Selects the maximum number of clients

  Description:
    This definition selects the maximum number of clients that the NVM driver can
    supported at run time.

  Remarks:
    None.

*/

#define DRV_NVM_CLIENTS_NUMBER                          1


// *****************************************************************************
/* NVM interrupt and polled mode operation control

  Summary:
    Macro specifies operation of the driver to be in the interrupt mode 
    or polled mode

  Description:
    This macro specifies operation of the driver to be in the interrupt mode 
    or polled mode
    
    - true  - Select if interrupt mode of NVM operation is desired
    - false - Select if polling mode of NVM operation is desired
    
    Not defining this option to true or false will result in build error.

  Remarks:
    None.
*/

#define DRV_NVM_INTERRUPT_MODE                          false

// *****************************************************************************
/* NVM Driver maximum number of buffer objects

  Summary:
    Selects the maximum number of buffer objects

  Description:
    This definition selects the maximum number of buffer objects. This indirectly
    also specifies the queue depth. The NVM Driver can queue up 
    DRV_NVM_BUFFER_OBJECT_NUMBER of read/write/erase requests before return a
    DRV_NVM_BUFFER_HANDLE_INVALID due to the queue being full. Increasing this
    number increases the RAM requirement of the driver.

  Remarks:
    None.

*/

#define DRV_NVM_BUFFER_OBJECT_NUMBER                          5

// *****************************************************************************
/* NVM Driver maximum number of buffer objects

  Summary:
    Selects the maximum number of buffer objects

  Description:
    This definition selects the maximum number of buffer objects. This indirectly
    also specifies the queue depth. The NVM Driver can queue up 
    DRV_NVM_BUFFER_OBJECT_NUMBER of read/write/erase requests before return a
    DRV_NVM_BUFFER_HANDLE_INVALID due to the queue being full. Increasing this
    number increases the RAM requirement of the driver.

  Remarks:
    None.

*/

// *****************************************************************************
/* NVM Driver Program Row Size.

  Summary:
    Specifies the NVM Driver Program Row Size in bytes.

  Description:
    This definition specifies the NVM Driver Program Row Size in bytes. This
    parameter is device specific and should be obtained from the device specific
    data sheet. The Program Row Size is the minimum block size that can be 
    programmed in one program operation.

  Remarks:
    None.

*/

#define DRV_NVM_ROW_SIZE    512

// *****************************************************************************
/* NVM Driver Erase Page Size.

  Summary:
    Specifies the NVM Driver Erase Page Size in bytes.

  Description:
    This definition specifies the NVM Driver Erase Page Size in bytes. This
    parameter is device specific and should be obtained from the device specific
    data sheet. The Erase Page Size is the minimum block size that can be 
    erased in one erase operation.

  Remarks:
    None.

*/

#define DRV_NVM_ERASE_SIZE   4096

// *****************************************************************************
/* NVM Driver Program Unlock Key 1

  Summary:
    Specifies the NVM Driver Program Unlock Key 1

  Description:
    This definition specifies the NVM Driver Program Unlock Key 1
    parameter is device specific and should be obtained from the device specific
    data sheet. 

  Remarks:
    None.

*/

#define DRV_NVM_UNLOCK_KEY1 0xAA996655

// *****************************************************************************
/* NVM Driver Program Unlock Key 2

  Summary:
    Specifies the NVM Driver Program Unlock Key 2

  Description:
    This definition specifies the NVM Driver Program Unlock Key 2
    parameter is device specific and should be obtained from the device specific
    data sheet. 

  Remarks:
    None.

*/

#define DRV_NVM_UNLOCK_KEY2 0x556699AA


#endif // #ifndef _DRV_NVM_CONFIG_TEMPLATE_H

/*******************************************************************************
 End of File
*/

