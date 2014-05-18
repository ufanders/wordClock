/*******************************************************************************
  Multiple MAC Module implementation for Microchip Stack

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
File Name:  tcpip_mac_object.c
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

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


#include "tcpip/src/tcpip_private.h"

#include "tcpip/src/tcpip_mac_private.h"
#include "tcpip/src/tcpip_mac_object.h"

/************************************
 *  the PIC32 MAC parameterized interface implementation
 *************************************/


TCPIP_MAC_RES TCPIP_MAC_Close(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->TCPIP_MAC_Close)(hMac);
}
    
bool TCPIP_MAC_LinkCheck(TCPIP_MAC_HANDLE hMac)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->TCPIP_MAC_LinkCheck)(hMac);
}
    
TCPIP_MAC_RES TCPIP_MAC_RxFilterHashTableEntrySet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_ADDR* DestMACAddr)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->TCPIP_MAC_RxFilterHashTableEntrySet)(hMac, DestMACAddr);
}
    

bool TCPIP_MAC_PowerMode(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_POWER_MODE pwrMode)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->TCPIP_MAC_PowerMode)(hMac, pwrMode);
}
    
TCPIP_MAC_RES TCPIP_MAC_PacketTx(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PACKET * ptrPacket)
{
    if (OSAL_SEM_Pend(((const TCPIP_MAC_DCPT*)hMac)->pObj->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
    {
        // SYS_DEBUG message
    }
    TCPIP_MAC_RES res = (*((const TCPIP_MAC_DCPT*)hMac)->pObj->TCPIP_MAC_PacketTx)(hMac, ptrPacket);
    if (OSAL_SEM_Post(((const TCPIP_MAC_DCPT*)hMac)->pObj->semaphore) != OSAL_RESULT_TRUE)
    {
        // SYS_DEBUG message
    }
    return res;
}

TCPIP_MAC_PACKET* TCPIP_MAC_PacketRx(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RES* pRes, const TCPIP_MAC_PACKET_RX_STAT** ppPktStat)
{
    if (OSAL_SEM_Pend(((const TCPIP_MAC_DCPT*)hMac)->pObj->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
    {
        // SYS_DEBUG message
    }
    TCPIP_MAC_PACKET* res = (*((const TCPIP_MAC_DCPT*)hMac)->pObj->TCPIP_MAC_PacketRx)(hMac, pRes, ppPktStat);
    if (OSAL_SEM_Post(((const TCPIP_MAC_DCPT*)hMac)->pObj->semaphore) != OSAL_RESULT_TRUE)
    {
        // SYS_DEBUG message
    }
    return res;
}

TCPIP_MAC_RES TCPIP_MAC_Process(TCPIP_MAC_HANDLE hMac)
{
    if (OSAL_SEM_Pend(((const TCPIP_MAC_DCPT*)hMac)->pObj->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
    {
        // SYS_DEBUG message
    }
    TCPIP_MAC_RES res = (*((const TCPIP_MAC_DCPT*)hMac)->pObj->TCPIP_MAC_Process)(hMac);
    if (OSAL_SEM_Post(((const TCPIP_MAC_DCPT*)hMac)->pObj->semaphore) != OSAL_RESULT_TRUE)
    {
        // SYS_DEBUG message
    }
    return res;
}

TCPIP_MAC_RES TCPIP_MAC_StatisticsGet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RX_STATISTICS* pRxStatistics, TCPIP_MAC_TX_STATISTICS* pTxStatistics)
{
    return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->TCPIP_MAC_StatisticsGet)(hMac, pRxStatistics, pTxStatistics);
}

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

bool TCPIP_MAC_EventMaskSet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents, bool enable)
{
        return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->TCPIP_MAC_EventMaskSet)(hMac, macEvents, enable);
}

bool TCPIP_MAC_EventAcknowledge(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents)
{
        return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->TCPIP_MAC_EventAcknowledge)(hMac, macEvents);
}

TCPIP_MAC_EVENT TCPIP_MAC_EventPendingGet(TCPIP_MAC_HANDLE hMac)
{
        return (*((const TCPIP_MAC_DCPT*)hMac)->pObj->TCPIP_MAC_EventPendingGet)(hMac);
}


#endif  // (TCPIP_STACK_USE_EVENT_NOTIFICATION)




