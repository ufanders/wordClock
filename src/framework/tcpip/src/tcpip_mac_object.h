/*******************************************************************************
  Multiple MAC Module implementation for Microchip Stack

  Company:
    Microchip Technology Inc.
    
  File Name:
    tcpip_mac_object.h

  Summary:
    
  Description:
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright © 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND,
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
// DOM-IGNORE-END

#ifndef _TCPIP_MAC_OBJECT_H_ 
#define _TCPIP_MAC_OBJECT_H_ 

#include "tcpip/tcpip_mac.h"
#include "tcpip/tcpip_manager.h"
#include "osal/osal.h"


/************************************
 *  the PIC32 MAC parameterized interface implementation
 *************************************/

typedef struct
{
    TCPIP_MAC_RES       (*TCPIP_MAC_Close)(TCPIP_MAC_HANDLE hMac);
    bool                (*TCPIP_MAC_LinkCheck)(TCPIP_MAC_HANDLE hMac);
    TCPIP_MAC_RES       (*TCPIP_MAC_RxFilterHashTableEntrySet)(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_ADDR* DestMACAddr);
    bool 	            (*TCPIP_MAC_PowerMode)(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_POWER_MODE pwrMode);
    TCPIP_MAC_RES       (*TCPIP_MAC_PacketTx)(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PACKET * ptrPacket);
    TCPIP_MAC_PACKET*   (*TCPIP_MAC_PacketRx)(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RES* pRes, const TCPIP_MAC_PACKET_RX_STAT** ppPktStat);
    TCPIP_MAC_RES       (*TCPIP_MAC_Process)(TCPIP_MAC_HANDLE hMac);
    TCPIP_MAC_RES       (*TCPIP_MAC_StatisticsGet)(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RX_STATISTICS* pRxStatistics, TCPIP_MAC_TX_STATISTICS* pTxStatistics);

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    bool                (*TCPIP_MAC_EventMaskSet)(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents, bool enable);
    bool                (*TCPIP_MAC_EventAcknowledge)(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents);
    TCPIP_MAC_EVENT     (*TCPIP_MAC_EventPendingGet)(TCPIP_MAC_HANDLE hMac);
#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)
    OSAL_SEM_HANDLE_TYPE semaphore;     // Semaphore information.
 
}TCPIP_MAC_OBJECT;        // TCPIP MAC object descriptor

typedef struct
{
    const TCPIP_MAC_OBJECT* pObj;           // associated object
                                        // pointer to the object here is intended to allow
                                        // multiple MAC objects of the same type
                                        // to share an unique const object table
    void*               mac_data[0];    // specific MAC object data
}TCPIP_MAC_DCPT; 
    
// supported MAC objects

TCPIP_MAC_RES       DRV_ETHMAC_PIC32MACSetup(TCPIP_MAC_MODULE_CTRL* const stackData, const TCPIP_MODULE_MAC_PIC32INT_CONFIG* initData);
TCPIP_MAC_RES       DRV_ETHMAC_PIC32MACTeardown(const TCPIP_MAC_MODULE_CTRL* const stackData );
TCPIP_MAC_HANDLE    DRV_ETHMAC_PIC32MACOpen( TCPIP_STACK_MODULE macId );
size_t              DRV_ETHMAC_PIC32MACGetConfig(TCPIP_STACK_MODULE modId, void* configBuff, size_t buffSize, size_t* pConfigSize);


TCPIP_MAC_RES       MRF24W_MACInitialize(TCPIP_MAC_MODULE_CTRL* const stackData, const TCPIP_MODULE_MAC_MRF24W_CONFIG* initData);
TCPIP_MAC_RES       MRF24W_MACDeinitialize(const TCPIP_MAC_MODULE_CTRL* const stackData );
TCPIP_MAC_HANDLE    MRF24W_MACOpen( TCPIP_STACK_MODULE macId );
size_t              MRF24W_MACGetConfig(TCPIP_STACK_MODULE modId, void* configBuff, size_t buffSize, size_t* pConfigSize);



#endif  // _TCPIP_MAC_OBJECT_H_ 

