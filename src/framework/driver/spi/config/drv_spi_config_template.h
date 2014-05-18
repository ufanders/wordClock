/*******************************************************************************
  SPI Driver Configuration Definitions for the Template Version

  Company:
    Microchip Technology Inc.

  File Name:
    drv_spi_config_template.h

  Summary:
    SPI Driver configuration definitions template.

  Description:
    These definitions statically define the driver's mode of operation.
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

#ifndef _DRV_SPI_CONFIG_TEMPLATE_H
#define _DRV_SPI_CONFIG_TEMPLATE_H


#error "This is a configuration template file.  Do not include it directly."


// *****************************************************************************
/* SPI hardware instance configuration

  Summary:
    Selects the maximum number of hardware instances that can be supported by
    the dynamic driver
                      .
  Description:
    This definition selects the maximum number of hardware instances that can be 
    supported by the dynamic driver.

  Remarks:
    None.
*/

#define DRV_SPI_INSTANCES_NUMBER                        1


/*********************************************************************
  Summary:
    Selects the maximum number of clients.
  Description:
    SPI maximum number of clients
    
    This definition selects the maximum number of clients that the SPI
    driver can support at run time.
  Remarks:
    None.                                                             
  *********************************************************************/

#define DRV_SPI_CLIENTS_NUMBER                          1


// *****************************************************************************
/* SPI static index selection

  Summary:
    SPI static index selection.

  Description:
    This definition selects the SPI static index for the driver object reference.

  Remarks:
    This index is required to make a reference to the driver object.
*/

#define DRV_SPI_INDEX                                   DRV_SPI_INDEX_2


// *****************************************************************************
/* SPI interrupt and polled mode operation control

  Summary:
    Controls operation of the driver in the interrupt or polled mode.

  Description:
    This macro controls the operation of the driver in the interrupt
    mode of operation. The possible values of this macro are:
    
    - true  - Select if interrupt mode of SPI operation is desired
    - false - Select if polling mode of SPI operation is desired
    
    Not defining this option to true or false will result in build error.

  Remarks:
    None.
*/

#define DRV_SPI_INTERRUPT_MODE                          false


// *****************************************************************************
/* SPI input sample phase selection

  Summary:
    Selects the SPI input sample phase.

  Description:
    This macro selects the SPI input sample phase.

  Remarks:
    None.
*/

#define DRV_SPI_INPUT_SAMPLE_PHASE                      SPI_INPUT_SAMPLING_PHASE_IN_MIDDLE


// *****************************************************************************
/* SPI baud rate clock selection

  Summary:
    Selects the SPI baud rate clock.

  Description:
    This macro selects the SPI baud rate clock.

  Remarks:
    None.
*/

#define DRV_SPI_BAUD_RATE_CLOCK                         SPI_BAUD_RATE_CLOCK_MCLK


// *****************************************************************************
/* SPI frame sync pulse direction selection

  Summary:
    Selects the SPI frame sync pulse direction.

  Description:
    This macro selects the SPI frame sync pulse direction.

  Remarks:
    None.
*/

#define DRV_SPI_FRAME_SYNC_PULSE_DIRECTION              SPI_FRAME_PULSE_DIRECTION_INPUT


// *****************************************************************************
/* SPI frame sync pulse polarity selection

  Summary:
    Selects the SPI frame sync pulse polarity.

  Description:
    This macro selects the SPI frame sync pulse polarity.

  Remarks:
    None.
*/

#define DRV_SPI_FRAME_SYNC_PULSE_POLARITY               SPI_FRAME_PULSE_POLARITY_ACTIVE_HIGH


// *****************************************************************************
/* SPI frame sync pulse edge selection

  Summary:
    Selects the SPI frame sync pulse edge.

  Description:
    This macro selects the SPI frame sync pulse edge.

  Remarks:
    None.
*/

#define DRV_SPI_FRAME_SYNC_PULSE_EDGE                   SPI_FRAME_PULSE_EDGE_PRECEDES_FIRST_CLOCK


// *****************************************************************************
/* SPI frame sync pulse width selection

  Summary:
   Selects the SPI frame sync pulse width.

  Description:
    This macro selects the SPI frame sync pulse width.

  Remarks:
    None.
*/

#define DRV_SPI_FRAME_SYNC_PULSE_WIDTH                  SPI_FRAME_PULSE_WIDTH_TO_ONE_WORD


// *****************************************************************************
/* SPI frame sync pulse count selection

  Summary:
    Selects the SPI frame sync pulse count.

  Description:
    This macro selects the SPI frame sync pulse count.

  Remarks:
    None.
*/

#define DRV_SPI_FRAME_SYNC_PULSE_COUNT                  SPI_FRAME_SYNC_PULSE_16_CHAR


// *****************************************************************************
/* SPI pins remap selection

  Summary:
    Selects the SPI pins remap.

  Description:
    This macro selects the SPI pins remap. The possible values of this macro are:
        - true    - Remap is required
        - false   - Remap is not required

  Remarks:
    None.
*/

#define DRV_SPI_PORTS_REMAP_USAGE                       false


#define DRV_SPI_BUFFER_SIZE                             8


// *****************************************************************************
// *****************************************************************************
// Section: Initialization Overrides
// *****************************************************************************
// *****************************************************************************
/* This section defines the initialization overrides */

/********************************************************************
  Summary:
    SPI peripheral ID selection.
  Description:
    SPI peripheral ID selection
    
    This macro selects the SPI peripheral ID selection. This is an
    initialization override of the spiID member of the initialization
    configuration.
  Remarks:
    Some devices also support SPI_ID_0.                              
  ********************************************************************/

#define DRV_SPI_PERIPHERAL_ID                           SPI_ID_1


// *****************************************************************************
/* SPI transmit/receive or transmit alone interrupt source

  Summary:
    Defines the transmit/receive or transmit alone interrupt source in case of
    static driver.

  Description:
    This macro defines the transmit/receive or transmit alone interrupt source of
    the static driver.

  Remarks:
    Refer to the Interrupt Peripheral Library documentation for more information 
    on the INT_SOURCE enumeration.

    Note: Some devices have a single interrupt source for both transmit
    and receive communication, while some have a dedicated transmit interrupt
    source. These are handled through the DRV_SPI_INTERRUPT_SOURCE configuration
    switch.
*/

#define DRV_SPI_INTERRUPT_SOURCE_TX                     INT_SOURCE_SPI_1_EVENT
// or
// #define DRV_SPI_INTERRUPT_SOURCE_TX                     INT_SOURCE_SPI_1_TRANSMIT


// *****************************************************************************
/* SPI receive interrupt source

  Summary:
   Defines the receive interrupt source in case of static driver.

  Description:
    This macro defines the receive interrupt source of the static driver.

  Remarks:
    Refer to the Interrupt Peripheral Library documentation for more information on 
    INT_SOURCE enumeration.

    Note: This should be specified in case the device does not have a
    dedicated receive interrupt source.
*/

#define DRV_SPI_INTERRUPT_SOURCE_RX                     INT_SOURCE_SPI_1_EVENT
// or
// #define DRV_SPI_INTERRUPT_SOURCE_RX                     INT_SOURCE_SPI_1_RECEIVE


// *****************************************************************************
/* SPI error interrupt source

  Summary:
   Defines the error interrupt source in case of static driver

  Description:
    Macro to define the error interrupt source in case of static driver.

  Remarks:
    Refer to the Interrupt Peripheral Library help for more information on INT_SOURCE
    enumeration
*/

#define DRV_SPI_INTERRUPT_SOURCE_ERROR                  INT_SOURCE_SPI_1_ERROR


// *****************************************************************************
/* SPI power state configuration

  Summary:
    Controls the power state of the SPI driver.

  Description:
    This macro controls the power state of the SPI driver.

  Remarks:
    Note: This feature may not be available in the device or the SPI module
    selected.
*/

#define DRV_SPI_POWER_STATE                             SYS_MODULE_POWER_IDLE_STOP


// *****************************************************************************
/* SPI communication width configuration

  Summary:
    Controls the communication width of the SPI driver.

  Description:
    This macro controls the communication width of the SPI driver.

  Remarks:
    None.
*/

#define DRV_SPI_COMMUNICATION_WIDTH                     SPI_COMMUNICATION_8BIT_WIDE


// *****************************************************************************
/* SPI baud rate value configuration

  Summary:
    Controls the baud rate value of the SPI driver.

  Description:
    This macro controls the baud rate value of the SPI driver.

  Remarks:
    None.
*/

#define DRV_SPI_BAUD_RATE                               8000000


// *****************************************************************************
/* SPI protocol type configuration

  Summary:
    Controls the protocol type of the SPI driver.

  Description:
    This macro controls the protocol type of the SPI driver.

  Remarks:
    None.
*/

#define DRV_SPI_PROTOCOL                                DRV_SPI_PROTOCOL_TYPE_STANDARD


// *****************************************************************************
/* SPI buffer type configuration

  Summary:
    Controls the buffer type of the SPI driver.

  Description:
    This macro controls the buffer type of the SPI driver.

  Remarks:
    None.
*/

#define DRV_SPI_BUFFER_USAGE_TYPE                       DRV_SPI_BUFFER_TYPE_STANDARD


#define DRV_SPI_TX_FIFO_INTERRUPT_MODE                  SPI_FIFO_INTERRUPT_WHEN_TRANSMISSION_IS_COMPLETE

#define DRV_SPI_RX_FIFO_INTERRUPT_MODE                  SPI_FIFO_INTERRUPT_WHEN_RECEIVE_BUFFER_IS_FULL


// *****************************************************************************
/* SPI clock mode configuration

  Summary:
    Controls the clock mode of the SPI driver.

  Description:
    This macro controls the clock mode of the SPI driver.

  Remarks:
    None.
*/

#define DRV_SPI_CLOCK_OPERATION_MODE                    DRV_SPI_CLOCK_MODE_IDLE_LOW_EDGE_RISE


// *****************************************************************************
/* SPI usage mode configuration

  Summary:
    Controls the usage mode of the SPI driver.

  Description:
    This macro controls the usage mode of the SPI driver.

  Remarks:
    None.
*/

#define DRV_SPI_USAGE_MODE                              DRV_SPI_MODE_MASTER


#endif // #ifndef _DRV_SPI_CONFIG_TEMPLATE_H

/*******************************************************************************
 End of File
*/

