/*******************************************************************************
  Internet Control Message Protocol (ICMP) Server

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    - Provides "ping" diagnostics
    - Reference: RFC 792
*******************************************************************************/

/*******************************************************************************
File Name:  ICMP.c
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


#if defined(TCPIP_STACK_USE_IPV4)
#if defined(TCPIP_STACK_USE_ICMP_SERVER) || defined(TCPIP_STACK_USE_ICMP_CLIENT)

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_ICMP
#include "tcpip/src/tcpip_notify.h"

// ICMP Header Structure
typedef struct
{
    uint8_t vType;
    uint8_t vCode;
    uint16_t wChecksum;
    uint16_t wIdentifier;
    uint16_t wSequenceNumber;
} ICMP_HEADER;

#define ICMP_TYPE_ECHO_REQUEST      8   // ICMP server is requested echo - type
#define ICMP_CODE_ECHO_REQUEST      0   // ICMP server is requested echo - code


#define ICMP_TYPE_ECHO_REPLY        0   // ICMP client echo reply - type
#define ICMP_CODE_ECHO_REPLY        0   // ICMP client echo reply - code



static int          icmpInitCount = 0;      // ICMP module initialization count

static const void*      icmpMemH = 0;        // memory handle


#if defined(TCPIP_STACK_USE_ICMP_CLIENT)

// Callback function for informing the upper-layer protocols about ICMP events
typedef void (*icmpCallback) (TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data);

static PROTECTED_SINGLE_LIST      icmpRegisteredUsers = { {0} };
//
// ICMP callback registration
typedef struct  _TAG_ICMP_LIST_NODE
{
	struct _TAG_ICMP_LIST_NODE* next;		// next node in list
                                            // makes it valid SGL_LIST_NODE node
    icmpCallback                callback;   // handler to be called for ICMP event
}ICMP_LIST_NODE;


// ICMP Packet Structure
typedef struct
{
	uint8_t vType;
	uint8_t vCode;
	uint16_t wChecksum;
	uint16_t wIdentifier;
	uint16_t wSequenceNumber;
	uint32_t wData;
} ICMP_PACKET;

#endif

// local prototypes
static void _ICMPAckPacket(TCPIP_MAC_PACKET* pkt, const void* ackParam);
static IPV4_PACKET * _ICMPAllocateTxPacketStruct (uint16_t totICMPLen);
static TCPIP_MAC_PKT_ACK_RES _ICMPProcessEchoRequest(TCPIP_NET_IF* pNetIf, TCPIP_MAC_PACKET* pRxPkt, uint32_t destAdd, uint32_t srcAdd);

#if defined(TCPIP_STACK_USE_ICMP_CLIENT)
static void _ICMPNotifyClients(TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data);
#endif  // defined(TCPIP_STACK_USE_ICMP_CLIENT)


bool TCPIP_ICMP_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const ICMP_MODULE_CONFIG* const pIcmpInit)
{
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }

    // stack start up
    if(icmpInitCount == 0)
    {   // first time we're run
        icmpMemH = stackCtrl->memH;
#if defined(TCPIP_STACK_USE_ICMP_CLIENT)
        TCPIP_Helper_ProtectedSingleListInitialize(&icmpRegisteredUsers);
#endif  // defined(TCPIP_STACK_USE_ICMP_CLIENT)
    }

    // interface init
    // postpone packet allocation until the TCPIP_ICMP_EchoRequestSend is called
    //

    icmpInitCount++;
    return true;
}


void TCPIP_ICMP_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down

    // one way or another this interface is going down

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // stack shut down
        if(icmpInitCount > 0)
        {   // we're up and running
            if(--icmpInitCount == 0)
            {   // all closed
                // release resources
#if defined(TCPIP_STACK_USE_ICMP_CLIENT)
                TCPIP_Notification_RemoveAll(&icmpRegisteredUsers, icmpMemH);
#endif  // defined(TCPIP_STACK_USE_ICMP_CLIENT)
                icmpMemH = 0;
            }
        }
    }

}

#if defined(TCPIP_STACK_USE_ICMP_CLIENT)
ICMP_HANDLE TCPIP_ICMP_CallbackRegister (void (*callback)(TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data))
{
    if(icmpMemH)
    {
        ICMP_LIST_NODE* newNode = (ICMP_LIST_NODE*)TCPIP_Notification_Add(&icmpRegisteredUsers, icmpMemH, sizeof(*newNode));
        if(newNode)
        {
            newNode->callback = callback;
            return newNode;
        }
    }

    return 0;
}


bool TCPIP_ICMP_CallbackDeregister(ICMP_HANDLE hIcmp)
{
    if(hIcmp && icmpMemH)
    {
        if(TCPIP_Notification_Remove((SGL_LIST_NODE*)hIcmp, &icmpRegisteredUsers, icmpMemH))
        {
            return true;
        }
    }

    return false;


}
#else
ICMP_HANDLE TCPIP_ICMP_CallbackRegister (void (*callback)(TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data))
{
    return 0;
}

bool TCPIP_ICMP_CallbackDeregister(ICMP_HANDLE hIcmp)
{
    return false;
}

#endif  // defined(TCPIP_STACK_USE_ICMP_CLIENT)



static IPV4_PACKET * _ICMPAllocateTxPacketStruct (uint16_t totICMPLen)
{
    IPV4_PACKET * ptrPacket;

    ptrPacket = (IPV4_PACKET*)TCPIP_PKT_SocketAlloc(sizeof(IPV4_PACKET), totICMPLen, 0, TCPIP_MAC_PKT_FLAG_ICMPV4 | TCPIP_MAC_PKT_FLAG_IPV4 | TCPIP_MAC_PKT_FLAG_TX);

    if (ptrPacket != 0)
    {
        TCPIP_PKT_PacketAcknowledgeSet(&ptrPacket->macPkt, _ICMPAckPacket, 0);
    }

    return ptrPacket;
}

// packet deallocation function
// packet was transmitted by the IP layer
static void _ICMPAckPacket(TCPIP_MAC_PACKET* pkt, const void* ackParam)
{
    TCPIP_PKT_PacketFree(pkt);
}

#if defined (TCPIP_STACK_USE_ICMP_CLIENT)
ICMP_ECHO_RESULT TCPIP_ICMP_EchoRequestSend (TCPIP_NET_HANDLE netH, IPV4_ADDR * targetAddr, uint16_t sequenceNumber, uint16_t identifier)
{
    IPV4_PACKET*    pTxPkt;
    ICMP_PACKET*    pICMPPkt;
    uint32_t        payload = 0x44332211;
    uint16_t        pktSize = sizeof(ICMP_PACKET);
    ICMP_ECHO_RESULT res;


    while(true)
    {
        // allocate TX packet
        pTxPkt = _ICMPAllocateTxPacketStruct(pktSize);
        if(pTxPkt == 0)
        {
            res = ICMP_ECHO_ALLOC_ERROR;
            break;
        }

        pICMPPkt = (ICMP_PACKET*)pTxPkt->macPkt.pTransportLayer;

        pICMPPkt->vType = ICMP_TYPE_ECHO_REQUEST; 
        pICMPPkt->vCode = ICMP_CODE_ECHO_REQUEST;
        pICMPPkt->wChecksum = 0x0000;
        pICMPPkt->wIdentifier = identifier;
        pICMPPkt->wSequenceNumber = sequenceNumber;
        pICMPPkt->wData = payload;
        pICMPPkt->wChecksum = TCPIP_Helper_CalcIPChecksum((uint8_t*)pICMPPkt, pktSize, 0);
        pTxPkt->destAddress.Val = targetAddr->Val;

        pTxPkt->netIfH = TCPIP_IPV4_SelectSourceInterface(netH, targetAddr, &pTxPkt->srcAddress, false);
        if(pTxPkt->netIfH == 0)
        {   // could not find an route
            res = ICMP_ECHO_ROUTE_ERROR;
            break;
        }

        pTxPkt->macPkt.pDSeg->segLen += pktSize;
        TCPIP_IPV4_PacketFormatTx(pTxPkt, IP_PROT_ICMP, pktSize);
        if(!TCPIP_IPV4_PacketTransmit(pTxPkt))
        {
            res = ICMP_ECHO_TRANSMIT_ERROR;
            break;
        }

        res = ICMP_ECHO_OK;
        break;
    }

    if(res < 0 && pTxPkt != 0)
    {
        TCPIP_PKT_PacketFree(&pTxPkt->macPkt);
    }

    return res;
}


#endif

// Processes an ICMP packet and generates an echo reply, if requested
// Note: the srcAdd parameter could be detected from the pNetIf
// However, when support for multiple IP addresses per interface is implemented
// passing the source address explicitely is better
void TCPIP_ICMP_Process(TCPIP_NET_IF* pNetIf, TCPIP_MAC_PACKET* pRxPkt, uint32_t destAdd, uint32_t srcAdd)
{
    ICMP_HEADER            *pRxHdr;
    TCPIP_MAC_DATA_SEGMENT *pSeg;
    TCPIP_UINT16_VAL        checksum;
    TCPIP_MAC_PKT_ACK_RES   ackRes;

    pRxHdr = (ICMP_HEADER*)pRxPkt->pTransportLayer;
    ackRes = TCPIP_MAC_PKT_ACK_RX_OK;

    while(true)
    {

        if(pRxPkt->totTransportLen < sizeof(*pRxHdr))
        {
            ackRes = TCPIP_MAC_PKT_ACK_STRUCT_ERR;
            break;
        }

        // Validate the checksum
        // The checksum data includes the precomputed checksum in the header
        // so a valid packet will always have a checksum of 0x0000
        // 1st segment
        checksum.Val = ~TCPIP_Helper_CalcIPChecksum(pRxPkt->pTransportLayer, pRxPkt->pDSeg->segLen, 0);
        // add the other possible data segments
        for(pSeg = pRxPkt->pDSeg->next; pSeg != 0; pSeg = pSeg->next)
        {
            checksum.Val = ~TCPIP_Helper_CalcIPChecksum(pSeg->segLoad, pSeg->segLen, checksum.Val);
        }

        if((uint16_t)~checksum.Val != 0)
        {
            ackRes = TCPIP_MAC_PKT_ACK_CHKSUM_ERR;
            break;
        }

        if(pRxHdr->vType == ICMP_TYPE_ECHO_REQUEST && pRxHdr->vCode == ICMP_CODE_ECHO_REQUEST)
        {   // echo request
            ackRes = _ICMPProcessEchoRequest(pNetIf, pRxPkt, destAdd, srcAdd);
            break;
        }

#if defined(TCPIP_STACK_USE_ICMP_CLIENT)
        if(pRxHdr->vType == ICMP_TYPE_ECHO_REPLY && pRxHdr->vCode == ICMP_CODE_ECHO_REPLY)
        {   // echo reply; check if our own
            // Get the sequence number and identifier fields
            TCPIP_UINT32_VAL userData;
            IPV4_ADDR remoteIPAddr;

            userData.w[0] = pRxHdr->wIdentifier;
            userData.w[1] = pRxHdr->wSequenceNumber;
            remoteIPAddr.Val = srcAdd;

            // Send a message to the application-level Ping driver that we've received an Echo Reply
            _ICMPNotifyClients(pNetIf, &remoteIPAddr, (void *)userData.v);
            ackRes = TCPIP_MAC_PKT_ACK_RX_OK;
        }
#endif

        break;
    }


    TCPIP_PKT_PacketAcknowledge(pRxPkt, ackRes); 
    return;
}


// echo request
static TCPIP_MAC_PKT_ACK_RES _ICMPProcessEchoRequest(TCPIP_NET_IF* pNetIf, TCPIP_MAC_PACKET* pRxPkt, uint32_t destAdd, uint32_t srcAdd)
{
    ICMP_HEADER            *pTxHdr;
    IPV4_PACKET            *pTxPkt;
    TCPIP_UINT16_VAL        checksum;
    uint8_t                *pSrcData, *pDstData;
    TCPIP_MAC_DATA_SEGMENT *pSrcSeg;

    // allocate TX packet
    pTxPkt = _ICMPAllocateTxPacketStruct(pRxPkt->totTransportLen);
    if(pTxPkt != 0)
    {   // succeeded

        // copy the 1st segment payload
        pSrcSeg = pRxPkt->pDSeg;
        pSrcData = pRxPkt->pTransportLayer;
        pDstData = pTxPkt->macPkt.pTransportLayer;
        while(pSrcSeg)
        {
            memcpy(pDstData, pSrcData, pSrcSeg->segLen);
            pDstData += pSrcSeg->segLen;
            if((pSrcSeg = pSrcSeg->next) != 0)
            {
                pSrcData = pSrcSeg->segLoad;
            }
        }

        // set the correct vType, vCode, and Checksum
        pTxHdr = (ICMP_HEADER*)pTxPkt->macPkt.pTransportLayer;

        pTxHdr->vType = ICMP_TYPE_ECHO_REPLY;
        pTxHdr->vCode = ICMP_CODE_ECHO_REPLY;
        checksum.Val = pTxHdr->wChecksum;
        checksum.v[0] += 8;	// Subtract 0x0800 from the checksum
        if(checksum.v[0] < 8u)
        {
            checksum.v[1]++;
            if(checksum.v[1] == 0u)
            {
                checksum.v[0]++;
            }
        }

        pTxHdr->wChecksum = checksum.Val;
        pTxPkt->srcAddress.Val = destAdd;
        pTxPkt->destAddress.Val = srcAdd;
        pTxPkt->netIfH = pNetIf;

        pTxPkt->macPkt.pDSeg->segLen += pRxPkt->totTransportLen;
        TCPIP_IPV4_PacketFormatTx(pTxPkt, IP_PROT_ICMP, pRxPkt->totTransportLen);
        if(!TCPIP_IPV4_PacketTransmit(pTxPkt))
        {
            TCPIP_PKT_PacketFree(&pTxPkt->macPkt);
        }
    }

    return TCPIP_MAC_PKT_ACK_RX_OK;
}


#if defined(TCPIP_STACK_USE_ICMP_CLIENT)

static void _ICMPNotifyClients(TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data)
{
    ICMP_LIST_NODE* dNode;

    for(dNode = (ICMP_LIST_NODE*)icmpRegisteredUsers.list.head; dNode != 0; dNode = dNode->next)
    {
        (*dNode->callback)(hNetIf, remoteIP, data);
    }

}

#endif //#if defined(TCPIP_STACK_USE_ICMP_CLIENT)

#endif //#if defined(TCPIP_STACK_USE_ICMP_SERVER) || defined(TCPIP_STACK_USE_ICMP_CLIENT)
#endif  // defined(TCPIP_STACK_USE_IPV4)


