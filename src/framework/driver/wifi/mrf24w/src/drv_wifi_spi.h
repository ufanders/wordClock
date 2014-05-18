/*******************************************************************************
  WiFi MAC interface functions

  File Name:
    drv_wifi_spi.h

  Summary:
   WiFi-specific MAC function prototypes called by TCP/IP stack.

  Description:
    The functions in this header file are accessed by the TCP/IP stack via
    function pointers.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright © 2012 released Microchip Technology Inc. All rights reserved.

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


#ifndef _DRV_WIFI_SPI_H
#define _DRV_WIFI_SPI_H

#if defined(TCPIP_IF_MRF24W)

// Using IO  as SPI chip select
#define WF_CS_Init()      {PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, WF_CS_PORT_CHANNEL, WF_CS_BIT_POS);}
#define WF_CS_Assert()    {SYS_PORTS_PinClear            (  PORTS_ID_0, WF_CS_PORT_CHANNEL, WF_CS_BIT_POS );}
#define WF_CS_Deassert()  {SYS_PORTS_PinSet              (  PORTS_ID_0, WF_CS_PORT_CHANNEL, WF_CS_BIT_POS );}

//#define WF_SPI_DMA_ENABLE
#if defined (WF_SPI_DMA_ENABLE)
    #define WF_SPI_DMA_BUF_SIZE 1024
#endif


bool WF_SpiInit(void);
void WF_SpiEnableChipSelect(void);
void WF_SpiDisableChipSelect(void);
void WF_SpiTxRx(uint8_t   *p_txBuf, uint16_t  txLen, uint8_t   *p_rxBuf, uint16_t  rxLen);

#endif /* TCPIP_IF_MRF24W */

#endif /* _DRV_WIFI_SPI_H */

// DOM-IGNORE-END
