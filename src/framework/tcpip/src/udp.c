/*******************************************************************************
  User Datagram Protocol (UDP) Communications Layer

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides unreliable, minimum latency transport of application 
     datagram (packet) oriented data
    -Reference: RFC 768
*******************************************************************************/

/*******************************************************************************
File Name:  udp.c
Copyright 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND,
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
#include "udp_private.h"
#include "osal/osal.h"

#if defined(TCPIP_STACK_USE_UDP)

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_UDP


/****************************************************************************
  Section:
	UDP Global Variables
  ***************************************************************************/
 

// Store an array of information pertaining to each UDP socket
static UDP_SOCKET_DCPT** UDPSocketDcpt = 0; 

static int          nUdpSockets = 0;    // number of sockets in the current UDP configuration
static const void*  udpMemH = 0;        // memory handle
static int          udpInitCount = 0;   // initialization counter

static uint16_t     udpDefTxSize;               // default size of the TX buffer

static SINGLE_LIST  udpPacketPool = { 0 };  // private pool of UDP packets

static uint16_t     udpPacketsInPool = 0;       // number of packets in the pool

static uint16_t     udpPoolPacketSize = 0;      // size of packets




static const UDP_MODULE_CONFIG udpConfigDefault = 
{
	UDP_MAX_SOCKETS,
	UDP_SOCKET_DEFAULT_TX_SIZE,
    UDP_SOCKET_POOL_BUFFERS,
    UDP_SOCKET_POOL_BUFFER_SIZE,
};


static OSAL_SEM_HANDLE_TYPE udpSemaphore = 0;
static OSAL_RESULT udpSemaphoreEnabled = OSAL_RESULT_FALSE;


/****************************************************************************
  Section:
	Function Prototypes
  ***************************************************************************/

static void     _UDPClose(UDP_SOCKET_DCPT* pSkt);
static void     _UDPFreeTxResources(UDP_SOCKET_DCPT* pSkt);
static void     _UDPFreeRxQueue(UDP_SOCKET_DCPT* pSkt);

static void _UDPResetRxPacket(UDP_SOCKET_DCPT* pSkt, TCPIP_MAC_PACKET* pRxPkt)
{
    if((pSkt->pCurrRxPkt = pRxPkt) != 0)
    {
        pSkt->pCurrRxSeg = pRxPkt->pDSeg;
        pSkt->rxSegLen = pRxPkt->pDSeg->segLen - sizeof(UDP_HEADER);
        pSkt->rxTotLen = pRxPkt->totTransportLen - sizeof(UDP_HEADER);
        pSkt->rxCurr = pRxPkt->pTransportLayer + sizeof(UDP_HEADER);
    }
    else
    {
        pSkt->pCurrRxSeg = 0;
        pSkt->rxSegLen = 0;
        pSkt->rxTotLen = 0;
        pSkt->rxCurr = 0;
    }
}


static void _UDPSetNewRxPacket(UDP_SOCKET_DCPT* pSkt, TCPIP_MAC_PACKET* pRxPkt)
{

    if(pSkt->pCurrRxPkt != 0)
    {   // acknowledge the old one
        TCPIP_PKT_PacketAcknowledge(pSkt->pCurrRxPkt, TCPIP_MAC_PKT_ACK_RX_OK);
    }

    _UDPResetRxPacket(pSkt, pRxPkt);

#if defined (TCPIP_STACK_USE_IPV4)
    if(pRxPkt != 0 && pSkt->addType == IP_ADDRESS_TYPE_IPV4)
    {
        if(pSkt->flags.destSet == 0)
        {
            pSkt->destAddress.Val = TCPIP_IPV4_PacketGetSourceAddress(pRxPkt)->Val;
        }
        if(pSkt->flags.srcSet == 0)
        {
            uint32_t newSrcAdd = TCPIP_IPV4_PacketGetDestAddress(pRxPkt)->Val;
            if(pSkt->srcAddress.Val != newSrcAdd)
            {
                pSkt->flags.srcSolved = 0;
            }
            pSkt->srcAddress.Val = newSrcAdd;
            pSkt->flags.srcSet = 1;
        }
        pSkt->pSktNet = (TCPIP_NET_IF*)pRxPkt->pktIf;    // bind it
    }
#endif  // defined (TCPIP_STACK_USE_IPV4)

#if defined (TCPIP_STACK_USE_IPV6)
    if(pRxPkt != 0 && pSkt->addType == IP_ADDRESS_TYPE_IPV6)
    {
        if(pSkt->flags.destSet == 0)
        {
            TCPIP_IPV6_DestAddressSet(pSkt->pV6Pkt, TCPIP_IPV6_PacketGetSourceAddress(pRxPkt));
        }
        if(pSkt->flags.srcSet == 0)
        {
            pSkt->flags.srcSolved = 0;
            pSkt->flags.srcSet = 1;
            TCPIP_IPV6_SourceAddressSet(pSkt->pV6Pkt, TCPIP_IPV6_PacketGetDestAddress(pRxPkt));
        }
        pSkt->pSktNet = (TCPIP_NET_IF*)pRxPkt->pktIf;    // bind it
        pSkt->pV6Pkt->netIfH = (TCPIP_NET_IF*)pRxPkt->pktIf;
        TCPIP_IPV6_PacketIPProtocolSet (pSkt->pV6Pkt);
    }
#endif  // defined (TCPIP_STACK_USE_IPV6)

    // else leave the old source/dest values in place
}

static void _UDPUpdatePacket(UDP_SOCKET_DCPT* pSkt)
{
    TCPIP_MAC_PACKET* pNextPkt = (TCPIP_MAC_PACKET*)TCPIP_Helper_SingleListHeadRemove(&pSkt->rxQueue);
    _UDPSetNewRxPacket(pSkt, pNextPkt);
}


static UDP_SOCKET_DCPT*  _UDPFindMatchingSocket(TCPIP_NET_IF* pPktIf, UDP_HEADER *h, IP_ADDRESS_TYPE addressType);

static void*             _UDPAllocateTxPacket(uint16_t pktSize, uint16_t txBuffSize, TCPIP_MAC_PACKET_FLAGS allocFlags);

#if defined (TCPIP_STACK_USE_IPV4)
static void*            _UDPv4PAllocateSktTxBuffer(UDP_SOCKET_DCPT* pSkt, IP_ADDRESS_TYPE addType, bool update);
static void             _UDPv4TxAckFnc (TCPIP_MAC_PACKET * pPkt, const void * param);
static uint16_t         _UDPv4IsTxPutReady(UDP_SOCKET_DCPT* pSkt);
static uint16_t         _UDPv4Flush(UDP_SOCKET_DCPT* pSkt);
#endif  // defined (TCPIP_STACK_USE_IPV4)

#if defined (TCPIP_STACK_USE_IPV6)
static IPV6_PACKET*     _UDPv6AllocateTxPacketStruct (TCPIP_NET_IF * pNetIf, UDP_SOCKET_DCPT * socket, bool update);
static void             _UDPv6TxAckFnc (void* pkt, bool success, const void * param);
static uint16_t         _UDPv6IsTxPutReady(UDP_SOCKET_DCPT* pSkt, unsigned short count);
static uint16_t         _UDPv6Flush(UDP_SOCKET_DCPT* pSkt);
static void             _UDPv6FreePacket(IPV6_PACKET* pPkt);
static void             _UDPResetHeader(UDP_HEADER * h);
#endif  // defined (TCPIP_STACK_USE_IPV6)

static UDP_PORT         _UDPAllocateEphemeralPort(void);
static bool             _UDPIsAvailablePort(UDP_PORT port);
static bool             _UDPIsSharedPortAvailable(UDP_PORT port, TCPIP_NET_IF* pNet);


static UDP_SOCKET       _UDPOpen(IP_ADDRESS_TYPE addType, UDP_OPEN_TYPE opType, UDP_PORT port, IP_MULTI_ADDRESS* address);

static bool             _UDPSetSourceAddress(UDP_SOCKET_DCPT* pSkt, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* localAddress)
{

    if(localAddress == 0)
    {   // nothing to set
        return false;
    }

    while(pSkt->addType == addType)
    {
#if defined (TCPIP_STACK_USE_IPV6)
        if (addType == IP_ADDRESS_TYPE_IPV6)
        {
            if(pSkt->pV6Pkt != 0)
            {
               TCPIP_IPV6_SourceAddressSet(pSkt->pV6Pkt, &localAddress->v6Add);
               return true;
            }
            break;
        }
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
        if (addType == IP_ADDRESS_TYPE_IPV4)
        {
            pSkt->srcAddress.Val = localAddress->v4Add.Val;
            pSkt->flags.srcSet = 1;
            pSkt->flags.srcSolved = 0;
            return true;
        }
#endif  // defined (TCPIP_STACK_USE_IPV4)

        break;
    }

    return false;

}

static  void _UDPSocketBind(UDP_SOCKET_DCPT* pSkt, TCPIP_NET_IF* pNet, IP_MULTI_ADDRESS* srcAddress)
{
    pSkt->pSktNet = pNet;
#if defined (TCPIP_STACK_USE_IPV6)
    if(pSkt->addType == IP_ADDRESS_TYPE_IPV6)
    {
        if(pSkt->pV6Pkt != 0)
        {
            pSkt->pV6Pkt->netIfH = pNet;
        }
    }
#endif  // defined (TCPIP_STACK_USE_IPV6)

    _UDPSetSourceAddress(pSkt, pSkt->addType, srcAddress);
}



/*static __inline__*/static  void /*__attribute__((always_inline))*/ _UDPSocketTxSet(UDP_SOCKET_DCPT* pSkt,  void* pTxPkt, uint8_t* txBuff, IP_ADDRESS_TYPE addType)
{
    pSkt->txStart = txBuff;
    pSkt->txEnd = txBuff + pSkt->txSize;
    pSkt->txWrite = txBuff;
    pSkt->addType =  addType;
    pSkt->pPkt = pTxPkt;
}

// returns the associated socket descriptor, if such a socket is valid
/*static __inline__*/static  UDP_SOCKET_DCPT* /*__attribute__((always_inline))*/ _UDPSocketDcpt(UDP_SOCKET s)
{
    if(s >= 0 && s < nUdpSockets)
    {
       return UDPSocketDcpt[s];
    }

    return 0;
}

/****************************************************************************
  Section:
	Connection Management Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	void TCPIP_UDP_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const UDP_MODULE_CONFIG* pUdpInit)

  Summary:
	Initializes the UDP module.

  Description:
	Initializes the UDP module.  This function initializes all the UDP 
	sockets to the closed state.

  Precondition:
	If passed as a parameter, the used stack heap should be initialized

  Parameters:
    stackCtrl  - pointer to a Stack initialization structure

    pUdpInit    - pointer to a UDP initialization structure containing:
                    - nSockets:         number of sockets to be created
                    - sktTxBuffSize:    size of the TX buffer
                    - poolBuffers:      number of buffers in the pool; 0 if none
                    - poolBufferSize:   size of the buffers in the pool; all equal    

  Returns:
  	true if success,
    false otherwise
  	
  Remarks:
	This function is called only once per interface
    but it actually performs initialization just once
  ***************************************************************************/

bool TCPIP_UDP_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const UDP_MODULE_CONFIG* pUdpInit)
{
    
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface start up
        return true;    // do not store per interface data
    }
    
    // stack start up
    OSAL_CRIT_Enter(OSAL_CRIT_TYPE_LOW);  // Full System Lock
    if(udpInitCount != 0)
    {   // initialize just once
        udpInitCount++;
        OSAL_CRIT_Leave(OSAL_CRIT_TYPE_LOW);  // Full System Lock
        return true;
    }
    
    if(stackCtrl->memH == 0)
    {
        SYS_ERROR(SYS_ERROR_ERROR, "UDP NULL dynamic allocation handle");
        return false;
    }

    // select default configuration if init data is missing
    if(pUdpInit == 0)
    {
        pUdpInit = &udpConfigDefault;
    }

    UDPSocketDcpt = (UDP_SOCKET_DCPT**)TCPIP_HEAP_Calloc(stackCtrl->memH, pUdpInit->nSockets, sizeof(UDP_SOCKET_DCPT*));
    if(UDPSocketDcpt == 0)
    {
        SYS_ERROR(SYS_ERROR_ERROR, "UDP Dynamic allocation failed");
        return false;
    }

    TCPIP_Helper_SingleListInitialize (&udpPacketPool);
    udpPacketsInPool = pUdpInit->poolBuffers;
    udpPoolPacketSize = (pUdpInit->poolBufferSize < UDP_SOCKET_POOL_BUFFER_MIN_SIZE) ? UDP_SOCKET_POOL_BUFFER_MIN_SIZE: pUdpInit->poolBufferSize;
    // allocate the pool; used for IPv4 only for now!!!
    if(udpPacketsInPool)
    {
        TCPIP_MAC_PACKET*   pPkt;
        int ix;

        for(ix = 0; ix < udpPacketsInPool; ix++)
        {
            pPkt = _UDPAllocateTxPacket(sizeof(UDP_V4_PACKET), udpPoolPacketSize, UDP_SOCKET_POOL_BUFFER_FLAG);
            if(pPkt)
            {
                TCPIP_Helper_SingleListTailAdd(&udpPacketPool, (SGL_LIST_NODE*)pPkt);
            }
            else
            {
                break;
            }

        }

        if(ix != udpPacketsInPool)
        {
            udpPacketsInPool = ix;
            SYS_ERROR(SYS_ERROR_WARN, "UDP Pool allocation failed");
        }
    }

    udpMemH = stackCtrl->memH;
    nUdpSockets = pUdpInit->nSockets;
    udpDefTxSize = pUdpInit->sktTxBuffSize;

    udpInitCount++;
    udpSemaphoreEnabled = OSAL_SEM_Create(&udpSemaphore, OSAL_SEM_TYPE_BINARY, 1, 1);
    udpSemaphore = udpSemaphore;  // Remove a warning
    OSAL_CRIT_Leave(OSAL_CRIT_TYPE_LOW);  // Full System Lock
    return true;
}

/*****************************************************************************
  Function:
	void TCPIP_UDP_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)

  Summary:
	De-Initializes the UDP module.

  Description:
	De-Initializes the UDP module.
    This function initializes each socket to the CLOSED state.
    If dynamic memory was allocated for the UDP sockets, the function
	will deallocate it.

  Precondition:
	TCPIP_UDP_Initialize() should have been called

  Parameters:
	stackCtrl   - pointer to Stack data showing which interface is closing

  Returns:
    None
    
  Remarks:
  ***************************************************************************/
void TCPIP_UDP_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    UDP_SOCKET_DCPT*    pSkt;
    int ix;

    // interface is going down
    for(ix = 0; ix < nUdpSockets; ix++)
    {
        pSkt = UDPSocketDcpt[ix];
        
        if(pSkt && pSkt->pSktNet == stackCtrl->pNetIf)
        {   // close on this specific interface
            if(pSkt->flags.looseNetIf)
            {
                pSkt->pSktNet = 0;  // unbound
            }
            else
            {   // close the socket
                _UDPClose(pSkt);
            }
        }
    }

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // stack shut down
        if(udpInitCount > 0)
        {   // we're up and running

            OSAL_CRIT_Enter(OSAL_CRIT_TYPE_LOW);  // Full System Lock
            if(--udpInitCount == 0)
            {   // all closed
                // release resources
                // just in case there are any not bound sockets
                TCPIP_MAC_PACKET*   pPkt;
                for(ix = 0; ix < nUdpSockets; ix++)
                {
                    pSkt = UDPSocketDcpt[ix];
                    if(pSkt) 
                    {
                        _UDPClose(pSkt);
                    }
                }

                TCPIP_HEAP_Free(udpMemH, UDPSocketDcpt);

                UDPSocketDcpt = 0;

                while((pPkt = (TCPIP_MAC_PACKET*)TCPIP_Helper_SingleListHeadRemove(&udpPacketPool)) != 0)
                {
                    TCPIP_PKT_PacketFree(pPkt);
                }
                udpPacketsInPool = 0;
                udpPoolPacketSize = 0;

                udpMemH = 0;
                nUdpSockets = 0;
            }
            OSAL_CRIT_Leave(OSAL_CRIT_TYPE_LOW);  // Full System Lock

        }
    }
}

UDP_SOCKET TCPIP_UDP_ServerOpen(IP_ADDRESS_TYPE addType, UDP_PORT localPort,  IP_MULTI_ADDRESS* localAddress)
{
    return UDPOpenServerSkt(addType, localPort, localAddress, UDP_OPEN_SERVER);
}

UDP_SOCKET UDPOpenServerSkt(IP_ADDRESS_TYPE addType, UDP_PORT localPort,  IP_MULTI_ADDRESS* localAddress, UDP_OPEN_TYPE opType)
{
    UDP_SOCKET  skt;
    TCPIP_NET_IF* pDefIf = 0;   // default: unbound
   
#if !defined (TCPIP_STACK_USE_IPV6)
   if(addType == IP_ADDRESS_TYPE_IPV6)
   {
       return INVALID_SOCKET;
   } 
   else
   {
       addType = IP_ADDRESS_TYPE_IPV4;
   }
#endif  // defined (TCPIP_STACK_USE_IPV6)
    
#if !defined (TCPIP_STACK_USE_IPV4)
   if(addType == IP_ADDRESS_TYPE_IPV4)
   {
       return INVALID_SOCKET;
   }
   else
   {
       addType = IP_ADDRESS_TYPE_IPV6;
   }
#endif  // defined (TCPIP_STACK_USE_IPV4)

#if defined (TCPIP_STACK_USE_IPV4)
   if(addType == IP_ADDRESS_TYPE_IPV4 && localAddress != 0)
   {
       if(localAddress->v4Add.Val == 0)
       { // ignore
           localAddress = 0;
       }
       else
       {
           pDefIf = TCPIP_STACK_IPAddToNet(&localAddress->v4Add, false);
           if(pDefIf == 0)
           {    // no such interface
               return INVALID_UDP_SOCKET;
           }
       }
   }
#endif  // defined (TCPIP_STACK_USE_IPV4)

#if defined (TCPIP_STACK_USE_IPV6)
   if(addType == IP_ADDRESS_TYPE_IPV6 && localAddress != 0)
   {
       pDefIf = _TCPIPStackIPv6AddToNet(&localAddress->v6Add, IPV6_ADDR_TYPE_UNICAST, false);
       if(pDefIf == 0)
       {    // no such interface
           return INVALID_UDP_SOCKET;
       }
   }
#endif  // defined (TCPIP_STACK_USE_IPV6)

    
    skt = _UDPOpen(addType, opType, localPort, 0);
    if(skt != INVALID_UDP_SOCKET)
    {
        _UDPSocketBind(UDPSocketDcpt[skt], pDefIf, localAddress);
    }

    return skt;
}

UDP_SOCKET TCPIP_UDP_ClientOpen(IP_ADDRESS_TYPE addType, UDP_PORT remotePort, IP_MULTI_ADDRESS* remoteAddress)
{
    return UDPOpenClientSkt(addType, remotePort, remoteAddress, UDP_OPEN_CLIENT);
}

UDP_SOCKET UDPOpenClientSkt(IP_ADDRESS_TYPE addType, UDP_PORT remotePort, IP_MULTI_ADDRESS* remoteAddress, UDP_OPEN_TYPE opType)
{
    UDP_SOCKET  skt;

#if !defined (TCPIP_STACK_USE_IPV6)
    if(addType == IP_ADDRESS_TYPE_IPV6)
    {
        return INVALID_SOCKET; 
    }
    else
    {
        addType = IP_ADDRESS_TYPE_IPV4;
    }
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if !defined (TCPIP_STACK_USE_IPV4)
    if(addType == IP_ADDRESS_TYPE_IPV4)
    {
        return INVALID_SOCKET; 
    }
    else
    {
        addType = IP_ADDRESS_TYPE_IPV6;
    }
#endif  // defined (TCPIP_STACK_USE_IPV4)

    if(addType == IP_ADDRESS_TYPE_ANY)
    {
        return INVALID_SOCKET; 
    }
    
    skt = _UDPOpen(addType, opType, remotePort, remoteAddress);
#if defined (TCPIP_STACK_USE_IPV6)
    if(addType == IP_ADDRESS_TYPE_IPV6 && skt != INVALID_UDP_SOCKET)
    {   // IPv6 clients require explicit binding; use the default interface; user can change the binding
        _UDPSocketBind(UDPSocketDcpt[skt], (TCPIP_NET_IF*)TCPIP_STACK_NetDefaultGet(), 0);
    }
#endif  // !defined (TCPIP_STACK_USE_IPV6)

    return skt;
}



static UDP_SOCKET _UDPOpen(IP_ADDRESS_TYPE addType, UDP_OPEN_TYPE opType, UDP_PORT port, IP_MULTI_ADDRESS* hostAddress)
{
    int        sktIx;
    UDP_SOCKET_DCPT *pSkt;
    UDP_PORT   localPort, remotePort;

    if(UDPSocketDcpt == 0)
    {
        return (UDP_SOCKET)INVALID_UDP_SOCKET;
    }

    if((opType & UDP_OPEN_CLIENT) != 0)
    {
        localPort = 0;
        remotePort = port;
    }
    else
    {
        localPort = port;
        remotePort = 0;
    }

    if(localPort == 0)
    {
        localPort = _UDPAllocateEphemeralPort();
        if(localPort == 0)
        {   // could'nt allocate a new port
            return INVALID_UDP_SOCKET;
        }
    }

    pSkt = (UDP_SOCKET_DCPT*)-1;

    if (udpSemaphoreEnabled == OSAL_RESULT_TRUE)
    {
        // Shared Data Lock
        if (OSAL_SEM_Pend(udpSemaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            // SYS_DEBUG message
        }
    }

    for ( sktIx = 0; sktIx < nUdpSockets; sktIx++ )
    {
        pSkt = UDPSocketDcpt[sktIx];
        if(pSkt == 0)
        {   // found an empty slot
            break;
        }
    }

    if(pSkt != 0)
    {   // all slots taken;
        if (udpSemaphoreEnabled == OSAL_RESULT_TRUE)
        {
            // Shared Data Lock
            if (OSAL_SEM_Post(udpSemaphore) != OSAL_RESULT_TRUE)
            {
                // SYS_DEBUG message
            }
        }
        return (UDP_SOCKET)INVALID_UDP_SOCKET;
    }

    pSkt = (UDP_SOCKET_DCPT*)TCPIP_HEAP_Calloc(udpMemH, 1, sizeof(*pSkt));
    if(pSkt == 0)
    {   // out of memory
        if (udpSemaphoreEnabled == OSAL_RESULT_TRUE)
        {
            // Shared Data Lock
            if (OSAL_SEM_Post(udpSemaphore) != OSAL_RESULT_TRUE)
            {
                // SYS_DEBUG message
            }
        }
        return (UDP_SOCKET)INVALID_UDP_SOCKET;
    }
    // success; store it
    UDPSocketDcpt[sktIx] = pSkt;
    if (udpSemaphoreEnabled == OSAL_RESULT_TRUE)
    {
        // Shared Data Lock
        if (OSAL_SEM_Post(udpSemaphore) != OSAL_RESULT_TRUE)
        {
            // SYS_DEBUG message
        }
    }

    pSkt->sktIx = sktIx;
    pSkt->localPort = localPort;	
    pSkt->remotePort = remotePort;
    pSkt->addType = addType;
    pSkt->txAllocLimit = UDP_SOCKET_DEFAULT_TX_QUEUE_LIMIT; 

    if((opType & UDP_OPEN_TX_SPLIT) != 0)
    {
        pSkt->txSize = 0;
        pSkt->flags.txSplitAlloc = 1;
    }
    else
    {
        pSkt->txSize = udpDefTxSize;
    }

    if((opType & UDP_OPEN_CONFIG_SERVICE) != 0)
    {
        pSkt->flags.stackConfig = 1;
    }

    // For IPv4 we postpone the allocation until the user wants to write something.
    // This allows RX only server sockets, that don't take extra memory.
    // Not possible for IPv6. It will have to rely on TCPIP_UDP_OptionsSet!

#if defined (TCPIP_STACK_USE_IPV6)
    if (addType == IP_ADDRESS_TYPE_IPV6)
    {
        _UDPv6AllocateTxPacketStruct (0, pSkt, true);
        if(pSkt->pPkt == 0)
        {   // out of memory
            _UDPClose(pSkt);
            return INVALID_UDP_SOCKET;
        }
    }
#endif  // defined (TCPIP_STACK_USE_IPV6)


    if((opType & UDP_OPEN_SERVER) != 0)
    {
#if defined (TCPIP_STACK_USE_IPV6)
        if (addType == IP_ADDRESS_TYPE_IPV6)
        {
            TCPIP_IPV6_DestAddressSet(pSkt->pV6Pkt,(IPV6_ADDR *)&IPV6_FIXED_ADDR_ALL_NODES_MULTICAST);
            TCPIP_IPV6_SetRemoteMacAddress(pSkt->pV6Pkt, &IPV6_MULTICAST_MAC_ADDRESS);
        }
#endif  // defined (TCPIP_STACK_USE_IPV6)

        // default non strict connections for server
        pSkt->flags.looseRemPort = pSkt->flags.looseNetIf = 1; 
    }
    else
    {   // UDP_OPEN_CLIENT
        switch(addType)
        {
#if defined (TCPIP_STACK_USE_IPV4)
            case IP_ADDRESS_TYPE_IPV4:
                // hostAddress is a literal IP address.
                if(hostAddress)
                {
                    pSkt->destAddress.Val = hostAddress->v4Add.Val;
                    pSkt->flags.destSet = 1;
                }
                break;
#endif  // defined (TCPIP_STACK_USE_IPV4)

#if defined (TCPIP_STACK_USE_IPV6)
            case IP_ADDRESS_TYPE_IPV6:
                TCPIP_IPV6_DestAddressSet (pSkt->pV6Pkt, &hostAddress->v6Add);
                break;
#endif  // defined (TCPIP_STACK_USE_IPV6)

            default:
                break;
        }
    }

    return (UDP_SOCKET)sktIx;
}



// allocates a packet from the packet heap
static void* _UDPAllocateTxPacket(uint16_t pktSize, uint16_t txBuffSize, TCPIP_MAC_PACKET_FLAGS allocFlags)
{
    allocFlags |= TCPIP_MAC_PKT_FLAG_TX | TCPIP_MAC_PKT_FLAG_UDP;
    
    return TCPIP_PKT_SocketAlloc(pktSize, sizeof(UDP_HEADER), txBuffSize, allocFlags);
}



#if defined (TCPIP_STACK_USE_IPV4)
static void* _UDPv4PAllocateSktTxBuffer(UDP_SOCKET_DCPT* pSkt, IP_ADDRESS_TYPE addType, bool update)
{
    TCPIP_MAC_PACKET_FLAGS allocFlags;
    TCPIP_MAC_PACKET*   pPkt;
    uint8_t*            pTxBuff;
    uint16_t            pktSize;

    if(addType != IP_ADDRESS_TYPE_IPV4)
    {   // IPv4 allocation only
        return 0;
    }

    if(pSkt->txAllocCnt >= pSkt->txAllocLimit)
    {   // cannot try to allocate any more packets
        return 0;
    }

    // allocate IPv4 packet
    pPkt = 0;
    allocFlags = TCPIP_MAC_PKT_FLAG_IPV4;
    if(pSkt->flags.stackConfig != 0)
    {
        allocFlags |= TCPIP_MAC_PKT_FLAG_CONFIG;
    }

    if(pSkt->flags.txSplitAlloc == 0)
    {
        pktSize = sizeof(UDP_V4_PACKET);
        // allocate from pool, if possible
        if(pSkt->flags.usePool != 0 && pSkt->txSize <= udpPoolPacketSize)
        {
            if((pPkt = (TCPIP_MAC_PACKET*)TCPIP_Helper_SingleListHeadRemove(&udpPacketPool)) != 0)
            {
                pPkt->pktFlags = allocFlags | UDP_SOCKET_POOL_BUFFER_FLAG;
                pPkt->next = 0;
                pPkt->pDSeg->segLen = 0;
            }
        }
    }
    else
    {
        allocFlags |= TCPIP_MAC_PKT_FLAG_SPLIT;
        pktSize = sizeof(UDP_V4_PACKET) + sizeof(*((UDP_V4_PACKET*)0)->zcSeg);
    }

    if(pPkt == 0)
    {   // allocate from main packet pool
        pPkt = _UDPAllocateTxPacket(pktSize, pSkt->txSize, allocFlags);
    }

    if(pPkt)
    {
        TCPIP_PKT_PacketAcknowledgeSet(pPkt, _UDPv4TxAckFnc, pSkt);

        if(pSkt->flags.txSplitAlloc)
        {   // link the 2 segments
            TCPIP_MAC_DATA_SEGMENT* pZSeg = ((UDP_V4_PACKET*)pPkt)->zcSeg;
            pPkt->pDSeg->next = pZSeg;
            pZSeg->segFlags = TCPIP_MAC_SEG_FLAG_STATIC;   // embedded in packet itself
            pTxBuff = 0;    // will be set as external payload
        }
        else
        {
            pTxBuff = pPkt->pTransportLayer + sizeof(UDP_HEADER);
        }

        if(update)
        {
            _UDPSocketTxSet(pSkt, pPkt, pTxBuff, addType);
        }
        pSkt->txAllocCnt++;
    }


    return pPkt;
}

static void _UDPv4TxAckFnc (TCPIP_MAC_PACKET * pPkt, const void * param)
{
    UDP_SOCKET_DCPT* pSkt = (UDP_SOCKET_DCPT*)param;

	if(pSkt != 0 && pSkt == _UDPSocketDcpt(pSkt->sktIx))
    {   // apparently still alive and kicking
        if(pSkt->pV4Pkt == (IPV4_PACKET*)pPkt)
        {   // still using this packet;
            ((IPV4_PACKET*)pPkt)->macPkt.pDSeg->segLen = 0;
            if(pSkt->flags.txSplitAlloc)
            {
                ((UDP_V4_PACKET*)pPkt)->zcSeg->segLen = 0;
            }
            else
            {
                pSkt->txWrite = pSkt->txStart;
            }
            return;
        }
    }
    else
    {   // closed module/socket
        pSkt = 0;
    }

    // either closed socket or another packet already allocated
    if((pPkt->pktFlags & UDP_SOCKET_POOL_BUFFER_FLAG) != 0 && udpPacketsInPool != 0)
    {   // re-insert in the pool
        TCPIP_Helper_SingleListTailAdd(&udpPacketPool, (SGL_LIST_NODE*)pPkt);
    }
    else
    {
        TCPIP_PKT_PacketFree(pPkt);
    }

    if(pSkt)
    {
        pSkt->txAllocCnt--;
    }
}

bool TCPIP_UDP_BcastIPV4AddressSet(UDP_SOCKET s, UDP_SOCKET_BCAST_TYPE bcastType, TCPIP_NET_HANDLE hNet)
{
    UDP_SOCKET_DCPT *pSkt;
    IPV4_ADDR       bcastAddress;

    pSkt = _UDPSocketDcpt(s);
    if(pSkt == 0 || pSkt->addType != IP_ADDRESS_TYPE_IPV4)
    {
        return false;
    }

    if(pSkt->flags.bcastForceType != UDP_BCAST_NONE)
    {   // BCAST is already set and cannot be overridden!
        return false;
    }


    switch (bcastType)
    {
        case UDP_BCAST_NETWORK_LIMITED:
            bcastAddress.Val = 0xffffffff;
            break;


        case UDP_BCAST_NETWORK_DIRECTED:
            if((bcastAddress.Val = TCPIP_STACK_NetAddressBcast(hNet)) == 0)
            {   // interface down/unknown?
                return false;
            }
            break;


        default:
            return false;
    }


    // set broadcast address
    pSkt->destAddress.Val = bcastAddress.Val;
    pSkt->flags.destSet = 1;

    return true;
}

static uint16_t _UDPv4IsTxPutReady(UDP_SOCKET_DCPT* pSkt)
{
    if(pSkt->pV4Pkt == 0 || (pSkt->pV4Pkt->macPkt.pktFlags & TCPIP_MAC_PKT_FLAG_QUEUED) != 0)
    {   // packet is in one of the queues; allocate another tx space
        if(_UDPv4PAllocateSktTxBuffer(pSkt, IP_ADDRESS_TYPE_IPV4, true) == 0)
        {   // allocation failed; caller will have to retry later
            return 0;
        }
        // else the new packet data is updated in place
    }

    if(pSkt->flags.txSplitAlloc == 0)
    {
        return pSkt->txEnd - pSkt->txWrite;
    }

    // return max segment possible
    return 1514 - sizeof(TCPIP_MAC_ETHERNET_HEADER) - sizeof(IPV4_HEADER) - sizeof(UDP_HEADER);
}


/*****************************************************************************
  Function:
	void TCPIP_UDP_ProcessIPv4(TCPIP_NET_IF* pNetIf, TCPIP_MAC_PACKET* pRxPkt)

  Summary:
	Handles an incoming UDP packet.
	
  Description:
	This function handles an incoming UDP segment to determine if it is 
	acceptable and should be handed to one of the stack applications for
	processing.

  Precondition:
	TCPIP_UDP_Initialize() has been called.

  Parameters:
    pNetIf  - interface on which the packet arrived
	pRxPkt - packet
	
  Return Values:
    None.
  ***************************************************************************/
void TCPIP_UDP_ProcessIPv4(TCPIP_NET_IF* pNetIf, TCPIP_MAC_PACKET* pRxPkt)
{
    UDP_HEADER*		 pUDPHdr;
    UDP_SOCKET_DCPT* pSkt;
    uint16_t         udpTotLength;
    const IPV4_ADDR* pPktSrcAdd;
    const IPV4_ADDR* pPktDstAdd;

    pUDPHdr = (UDP_HEADER*)pRxPkt->pTransportLayer;
    udpTotLength = TCPIP_Helper_ntohs(pUDPHdr->Length);

    if(udpTotLength < sizeof(UDP_HEADER) || udpTotLength != pRxPkt->totTransportLen)
    {   // discard suspect packet
        TCPIP_PKT_PacketAcknowledge(pRxPkt, TCPIP_MAC_PKT_ACK_STRUCT_ERR);
        return;
    }

    pPktSrcAdd = TCPIP_IPV4_PacketGetSourceAddress(pRxPkt);
    pPktDstAdd = TCPIP_IPV4_PacketGetDestAddress(pRxPkt);
	// See if we need to validate the checksum field (0x0000 is disabled)
#ifdef UDP_USE_RX_CHECKSUM
	if((pUDPHdr->Checksum != 0))
	{
        IPV4_PSEUDO_HEADER  pseudoHdr;
        uint16_t            calcChkSum;
	    // Calculate IP pseudoheader checksum.
	    pseudoHdr.SourceAddress.Val = pPktSrcAdd->Val;
	    pseudoHdr.DestAddress.Val = pPktDstAdd->Val;
	    pseudoHdr.Zero	= 0;
	    pseudoHdr.Protocol = IP_PROT_UDP;
	    pseudoHdr.Length = pUDPHdr->Length;

	    calcChkSum = ~TCPIP_Helper_CalcIPChecksum((uint8_t*)&pseudoHdr, sizeof(pseudoHdr), 0);
        if((pRxPkt->pktFlags & TCPIP_MAC_PKT_FLAG_SPLIT) != 0)
        {
            calcChkSum = TCPIP_Helper_PacketChecksum(pRxPkt, (uint8_t*)pUDPHdr, udpTotLength, calcChkSum);
        }
        else
        {
            calcChkSum = TCPIP_Helper_CalcIPChecksum((uint8_t*)pUDPHdr, udpTotLength, calcChkSum);
        }

        if(calcChkSum != 0)
        {   // discard packet
            TCPIP_PKT_PacketAcknowledge(pRxPkt, TCPIP_MAC_PKT_ACK_CHKSUM_ERR);
            return;
        }
	}
#endif // UDP_USE_RX_CHECKSUM

    pUDPHdr->SourcePort = TCPIP_Helper_ntohs(pUDPHdr->SourcePort);
    pUDPHdr->DestinationPort = TCPIP_Helper_ntohs(pUDPHdr->DestinationPort);
    pUDPHdr->Length = udpTotLength - sizeof(UDP_HEADER);    

    pSkt = _UDPFindMatchingSocket(pNetIf, pUDPHdr, IP_ADDRESS_TYPE_IPV4);
    if(pSkt == 0)
    {
        // If there is no matching socket, There is no one to handle
        // this data.  Discard it.
        TCPIP_PKT_PacketAcknowledge(pRxPkt, TCPIP_MAC_PKT_ACK_PROTO_DEST_ERR);
    }
    else
    {   // valid RX packet
        if(pSkt->pCurrRxPkt == 0)
        {
            _UDPSetNewRxPacket(pSkt, pRxPkt);
        }
        else
        {   // insert this packet in the RX queue
            TCPIP_Helper_SingleListTailAdd(&pSkt->rxQueue, (SGL_LIST_NODE*)pRxPkt);
        }
    }
}

static uint16_t _UDPv4Flush(UDP_SOCKET_DCPT* pSkt)
{
    IPV4_PACKET*        pv4Pkt;
    uint16_t            udpLoadLen, udpTotLen, rootLen;
    UDP_HEADER*         pUDPHdr;
    IPV4_PSEUDO_HEADER  pseudoHdr;
    uint16_t            checksum;
    TCPIP_MAC_DATA_SEGMENT* pZSeg;

    if(pSkt->destAddress.Val == 0)
    {   // don't even bother
        return 0;
    }

    if(pSkt->flags.srcSolved == 0 || pSkt->pSktNet == 0)
    {
        pSkt->pSktNet = (TCPIP_NET_IF*)TCPIP_IPV4_SelectSourceInterface(pSkt->pSktNet, &pSkt->destAddress, &pSkt->srcAddress, pSkt->flags.srcSet != 0);
        if(pSkt->pSktNet == 0)
        {   // cannot find an route?
            return 0;
        }
        pSkt->flags.srcSolved = 1;
    }

    pv4Pkt = pSkt->pV4Pkt;
    pv4Pkt->srcAddress.Val = pSkt->srcAddress.Val;
    pv4Pkt->destAddress.Val = pSkt->destAddress.Val;
    pv4Pkt->netIfH = pSkt->pSktNet;

    // start preparing the UDP header and packet
    pUDPHdr = (UDP_HEADER*)pv4Pkt->macPkt.pTransportLayer;

    // update the current load length
    if(pSkt->flags.txSplitAlloc != 0)
    {
        pZSeg = ((UDP_V4_PACKET*)pv4Pkt)->zcSeg;
        udpLoadLen = pZSeg->segLen;
        rootLen = sizeof(UDP_HEADER);
        // size of the payload should already be set
    }
    else
    {
        udpLoadLen = pSkt->txWrite - pSkt->txStart;
        rootLen = udpLoadLen + sizeof(UDP_HEADER); 
        pZSeg = 0;
    }
    pv4Pkt->macPkt.pDSeg->segLen += rootLen;
    udpTotLen = udpLoadLen + sizeof(UDP_HEADER);

    pUDPHdr->SourcePort = TCPIP_Helper_htons(pSkt->localPort);
    pUDPHdr->DestinationPort = TCPIP_Helper_htons(pSkt->remotePort);
    pUDPHdr->Length = TCPIP_Helper_htons(udpTotLen);
    pUDPHdr->Checksum = 0;

    // add the pseudo header
    pseudoHdr.SourceAddress.Val = pv4Pkt->srcAddress.Val;
    pseudoHdr.DestAddress.Val = pv4Pkt->destAddress.Val;
    pseudoHdr.Zero = 0;
    pseudoHdr.Protocol = IP_PROT_UDP;
    pseudoHdr.Length = pUDPHdr->Length;
    checksum = ~TCPIP_Helper_CalcIPChecksum((uint8_t*)&pseudoHdr, sizeof(pseudoHdr), 0);

    if(pSkt->flags.txSplitAlloc != 0)
    {
        checksum = ~TCPIP_Helper_CalcIPChecksum((uint8_t*)pUDPHdr, sizeof(UDP_HEADER), checksum);
        checksum = ~TCPIP_Helper_CalcIPChecksum(pZSeg->segLoad, udpLoadLen, checksum);
    }
    else
    {   // one contiguous buffer
        checksum = ~TCPIP_Helper_CalcIPChecksum((uint8_t*)pUDPHdr, udpTotLen, checksum);
    }

    pUDPHdr->Checksum = ~checksum;

    // and we're done
    TCPIP_IPV4_PacketFormatTx(pv4Pkt, IP_PROT_UDP, udpTotLen);
    pv4Pkt->macPkt.next = 0;    // single packet
    if(TCPIP_IPV4_PacketTransmit(pv4Pkt))
    {
        return udpLoadLen; 
    }

    return 0;
}

#endif  // defined (TCPIP_STACK_USE_IPV4)

#if defined (TCPIP_STACK_USE_IPV6)
static IPV6_PACKET * _UDPv6AllocateTxPacketStruct (TCPIP_NET_IF * pNetIf, UDP_SOCKET_DCPT * pSkt, bool update)
{
    IPV6_PACKET * pkt;
    uint8_t*     txBuffer;

    if(pSkt->txAllocCnt >= pSkt->txAllocLimit)
    {   // cannot try to allocate any more packets
        return 0;
    }

    pkt = TCPIP_IPV6_TxPacketAllocate (pNetIf, _UDPv6TxAckFnc, pSkt);

    if (pkt == 0)
        return 0;

    if (TCPIP_IPV6_UpperLayerHeaderPut (pkt, NULL, sizeof (UDP_HEADER), IP_PROT_UDP, UDP_CHECKSUM_OFFSET) == NULL)
    {
        TCPIP_IPV6_PacketFree (pkt);
        return 0;
    }


    txBuffer = (uint8_t*)TCPIP_HEAP_Malloc(udpMemH, pSkt->txSize);
    if(txBuffer == 0)
    {
        TCPIP_IPV6_PacketFree (pkt);
        return 0;
    }

    if(update)
    {
        _UDPSocketTxSet(pSkt, pkt, txBuffer, IP_ADDRESS_TYPE_IPV6);
    }

    pkt->clientData = txBuffer;
    return pkt;
}

static void _UDPv6FreePacket(IPV6_PACKET* pkt)
{
    if(pkt->clientData != 0)
    {
        TCPIP_HEAP_Free(udpMemH, pkt->clientData);
        pkt->clientData = 0;
    }
    TCPIP_IPV6_PacketFree ((IPV6_PACKET *)pkt);
}

static void _UDPv6TxAckFnc (void* pkt, bool success, const void * param)
{
    UDP_SOCKET_DCPT* pSkt = (UDP_SOCKET_DCPT*)param;

	if(pSkt != 0 && pSkt == _UDPSocketDcpt(pSkt->sktIx))
    {   // apparently still alive and kicking
        if(pSkt->pV6Pkt == (IPV6_PACKET*)pkt)
        {   // still using this packet;
            _UDPResetHeader(TCPIP_IPV6_UpperLayerHeaderPtrGet((IPV6_PACKET *)pkt));
            pSkt->txWrite = pSkt->txStart;
            return;
        }
    }
    else
    {   // closed module/socket
        pSkt = 0;
    }
    
    _UDPv6FreePacket ((IPV6_PACKET *)pkt);
    if(pSkt)
    {
        pSkt->txAllocCnt--;
    }
}

static uint16_t _UDPv6IsTxPutReady(UDP_SOCKET_DCPT* pSkt, unsigned short count)
{
    IPV6_PACKET * pkt;

    pkt = pSkt->pV6Pkt;
    if (pkt == NULL)
    {
        // This should only happen if the user has made an inappropriate call to an 
        // unopened socket.
        return 0;
    }

    if (pkt->flags.queued)
    {
        // Try to allocate a new transmit packet
        // Try to allocate a new transmit packet
        IPV6_PACKET * tempPtr = _UDPv6AllocateTxPacketStruct(pSkt->pSktNet, pSkt, false);
        if (tempPtr != NULL)
        {
            if (!TCPIP_IPV6_TxPacketStructCopy (tempPtr, pkt))
            {
                _UDPv6FreePacket(tempPtr);
                return 0;
            }

            // store changes
            _UDPSocketTxSet(pSkt, tempPtr, pkt->clientData, IP_ADDRESS_TYPE_IPV6);
            pkt = tempPtr;
            _UDPResetHeader(TCPIP_IPV6_UpperLayerHeaderPtrGet(pkt));
        }
        else
        {
            // We couldn't allocate a new packet.  Return 0 until we can 
            // or until a queued packet can be returned to this node.
            return 0;
        }
    }

    return pSkt->txEnd - pSkt->txWrite;
}

void TCPIP_UDP_ProcessIPv6(TCPIP_MAC_PACKET* pRxPkt, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, uint16_t dataLen, uint16_t headerLen)
{
    UDP_HEADER*         h;
    uint16_t            udpTotLength;
    UDP_SOCKET_DCPT*    pSkt;
    TCPIP_NET_IF*       pNetIf;


    // Retrieve UDP header.
    h = (UDP_HEADER*)pRxPkt->pTransportLayer;
    udpTotLength = TCPIP_Helper_ntohs(h->Length);

    if(dataLen < sizeof(UDP_HEADER) || udpTotLength != dataLen)
    {   // discard suspect packet
        TCPIP_PKT_PacketAcknowledge(pRxPkt, TCPIP_MAC_PKT_ACK_STRUCT_ERR);
        return;
    }


#ifdef UDP_USE_RX_CHECKSUM
    if(h->Checksum != 0)
    {
        IPV6_PSEUDO_HEADER  pseudoHeader;
        uint16_t            calcChkSum;

        // Calculate IP pseudoheader checksum.
        memcpy (&pseudoHeader.SourceAddress, remoteIP, sizeof (IPV6_ADDR));
        memcpy (&pseudoHeader.DestAddress, localIP, sizeof (IPV6_ADDR));
        // Total payload length is the length of data + extension headers
        pseudoHeader.PacketLength = h->Length;
        pseudoHeader.zero1 = 0;
        pseudoHeader.zero2 = 0;
        pseudoHeader.NextHeader = IP_PROT_UDP;

	    calcChkSum = ~TCPIP_Helper_CalcIPChecksum((uint8_t*)&pseudoHeader, sizeof(pseudoHeader), 0);
        if((pRxPkt->pktFlags & TCPIP_MAC_PKT_FLAG_SPLIT) != 0)
        {
            calcChkSum = TCPIP_Helper_PacketChecksum(pRxPkt, (uint8_t*)h, udpTotLength, calcChkSum);
        }
        else
        {
            calcChkSum = TCPIP_Helper_CalcIPChecksum((uint8_t*)h, udpTotLength, calcChkSum);
        }

        if(calcChkSum != 0)
        {   // discard packet
            TCPIP_PKT_PacketAcknowledge(pRxPkt, TCPIP_MAC_PKT_ACK_CHKSUM_ERR);
            return;
        }
	}
#endif // UDP_USE_RX_CHECKSUM



    h->SourcePort        = TCPIP_Helper_ntohs(h->SourcePort);
    h->DestinationPort   = TCPIP_Helper_ntohs(h->DestinationPort);
    h->Length            = udpTotLength - sizeof(UDP_HEADER);

    pNetIf = (TCPIP_NET_IF*)pRxPkt->pktIf;
    pSkt = _UDPFindMatchingSocket(pNetIf, h, IP_ADDRESS_TYPE_IPV6);
    if(pSkt == 0)
    {
        // Send ICMP Destination Unreachable Code 4 (Port unreachable) and discard packet
        TCPIP_IPV6_ErrorSend (pNetIf, pRxPkt, localIP, remoteIP, ICMPV6_ERR_DU_PORT_UNREACHABLE, ICMPV6_ERROR_DEST_UNREACHABLE, 0x00000000, dataLen + headerLen + sizeof (IPV6_HEADER));

        // If there is no matching socket, There is no one to handle
        // this data.  Discard it.
        TCPIP_PKT_PacketAcknowledge(pRxPkt, TCPIP_MAC_PKT_ACK_PROTO_DEST_ERR);
    }
    else
    {   // valid RX packet
        if(pSkt->pCurrRxPkt == 0)
        {
            _UDPSetNewRxPacket(pSkt, pRxPkt);
        }
        else
        {   // insert this packet in the RX queue
            TCPIP_Helper_SingleListTailAdd(&pSkt->rxQueue, (SGL_LIST_NODE*)pRxPkt);
        }
    }

}

static uint16_t _UDPv6Flush(UDP_SOCKET_DCPT* pSkt)
{
    TCPIP_NET_IF*   pSktNet;
    uint16_t        wUDPLength;
    UDP_HEADER*     pUDPHeader;

    pSktNet = (TCPIP_NET_IF*)pSkt->pV6Pkt->netIfH;
	if(pSktNet == 0 || !TCPIP_IPV6_InterfaceIsReady(pSktNet))
    {   // IPv6 client socket requires explicit binding
        return 0;
    }

    wUDPLength = pSkt->txWrite - pSkt->txStart;
    if(!TCPIP_IPV6_PayloadSet (pSkt->pV6Pkt, pSkt->txStart, wUDPLength))
    {
        return 0;
    }


    TCPIP_IPV6_HeaderPut(pSkt->pV6Pkt, IP_PROT_UDP);

    pUDPHeader = (UDP_HEADER *)TCPIP_IPV6_UpperLayerHeaderPtrGet(pSkt->pV6Pkt);

    pUDPHeader->SourcePort = TCPIP_Helper_htons(pSkt->localPort);
    pUDPHeader->DestinationPort = TCPIP_Helper_htons(pSkt->remotePort);
    pUDPHeader->Length = TCPIP_Helper_htons(wUDPLength + sizeof(UDP_HEADER));

    TCPIP_IPV6_Flush (pSkt->pV6Pkt);

    if (!TCPIP_IPV6_IsPacketQueued(pSkt->pV6Pkt))
    {
        TCPIP_IPV6_TransmitPacketStateReset (pSkt->pV6Pkt);
        _UDPResetHeader(pUDPHeader);
        pSkt->txWrite = pSkt->txStart;
    }
    return wUDPLength;
}


static void _UDPResetHeader(UDP_HEADER * h)
{
    if (h)
    {
        h->Checksum = 0x0000;
        h->Length = 0x0000;
    }
}

#endif  // defined (TCPIP_STACK_USE_IPV6)


bool TCPIP_UDP_IsConnected(UDP_SOCKET s)
{
    UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(s);
    return pSkt != 0;
}




void TCPIP_UDP_Close(UDP_SOCKET s)
{
    UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(s);

    if(pSkt)
    {
        _UDPClose(pSkt);
    }
}

static void _UDPClose(UDP_SOCKET_DCPT* pSkt)
{
    _UDPFreeTxResources(pSkt);
    _UDPFreeRxQueue(pSkt);
    if(pSkt->pCurrRxPkt != 0)
    {   // acknowledge the old one
        TCPIP_PKT_PacketAcknowledge(pSkt->pCurrRxPkt, TCPIP_MAC_PKT_ACK_PROTO_DEST_CLOSE);
    }

    UDPSocketDcpt[pSkt->sktIx] = 0;
    TCPIP_HEAP_Free(udpMemH, pSkt);
}

static void _UDPFreeTxResources(UDP_SOCKET_DCPT* pSkt)
{

    switch(pSkt->addType)
    {
#if defined(TCPIP_STACK_USE_IPV6)
        case IP_ADDRESS_TYPE_IPV6:
            
            if(pSkt->pV6Pkt != 0)
            {
                if(pSkt->pV6Pkt->flags.queued == 0)
                {
                    _UDPv6FreePacket(pSkt->pV6Pkt);
                    pSkt->txStart = 0;
                }
            }
            break;
#endif  // defined(TCPIP_STACK_USE_IPV6)

#if defined(TCPIP_STACK_USE_IPV4)
        case IP_ADDRESS_TYPE_IPV4:
            if(pSkt->pV4Pkt)
            {
                TCPIP_MAC_PACKET* pPkt = &pSkt->pV4Pkt->macPkt;
                if((pPkt->pktFlags & TCPIP_MAC_PKT_FLAG_QUEUED) == 0)
                {   
                    if((pPkt->pktFlags & UDP_SOCKET_POOL_BUFFER_FLAG) != 0)
                    {   // re-insert in the pool
                        TCPIP_Helper_SingleListTailAdd(&udpPacketPool, (SGL_LIST_NODE*)pPkt);
                    }
                    else
                    {
                        TCPIP_PKT_PacketFree(pPkt);
                    }
                }
            }
            break;
#endif  // defined (TCPIP_STACK_USE_IPV4)

        default:
            break;
    }


    pSkt->pPkt = 0;
}

// frees the associated socket RX packet queue
static void _UDPFreeRxQueue(UDP_SOCKET_DCPT* pSkt)
{
    TCPIP_MAC_PACKET*   pRxPkt;

    while((pRxPkt = (TCPIP_MAC_PACKET*)TCPIP_Helper_SingleListHeadRemove(&pSkt->rxQueue)) != 0)
    {
        TCPIP_PKT_PacketAcknowledge(pRxPkt, TCPIP_MAC_PKT_ACK_PROTO_DEST_CLOSE);
    }

}

bool TCPIP_UDP_SocketInfoGet(UDP_SOCKET s, UDP_SOCKET_INFO* pInfo)
{
    UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(s);

    if(pSkt == 0 || pInfo == 0)
    {
        return false;
    }


    while(true)
    {
#if defined (TCPIP_STACK_USE_IPV6)
        if (pSkt->addType == IP_ADDRESS_TYPE_IPV6)
        {
            memcpy(&pInfo->remoteIPaddress.v6Add.v, (void*)TCPIP_IPV6_DestAddressGet(pSkt->pV6Pkt), sizeof(IPV6_ADDR));
            memcpy(&pInfo->localIPaddress.v6Add.v, (void*)TCPIP_IPV6_SourceAddressGet(pSkt->pV6Pkt), sizeof(IPV6_ADDR));
            pInfo->addressType = IP_ADDRESS_TYPE_IPV6;
            break;
        }
#endif  // defined (TCPIP_STACK_USE_IPV6)


#if defined (TCPIP_STACK_USE_IPV4)
        if (pSkt->addType == IP_ADDRESS_TYPE_IPV4)
        {
            pInfo->remoteIPaddress.v4Add.Val = pSkt->destAddress.Val;
            pInfo->localIPaddress.v4Add.Val = pSkt->srcAddress.Val;
            pInfo->addressType = IP_ADDRESS_TYPE_IPV4;
        }
#endif  // defined (TCPIP_STACK_USE_IPV4)
        break;
    }

	pInfo->remotePort = pSkt->remotePort;
	pInfo->localPort = pSkt->localPort;

	return true;

}

bool TCPIP_UDP_TxOffsetSet(UDP_SOCKET s, uint16_t wOffset, bool relative)
{
    UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(s);
    
    if(pSkt && pSkt->txStart)
    {
        uint8_t* pNewWrite = relative ? pSkt->txWrite : pSkt->txStart;
        pNewWrite += wOffset;

        if(pSkt->txStart <= pNewWrite && pNewWrite <= pSkt->txEnd)
        {
            pSkt->txWrite = pNewWrite;
            return true;
        }
    }

    return false;
}


void TCPIP_UDP_RxOffsetSet(UDP_SOCKET s, uint16_t wOffset)
{
    UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(s);

    if(pSkt && pSkt->pCurrRxPkt != 0)
    {
        // set the packet segments from the beginning and discard
        _UDPResetRxPacket(pSkt, pSkt->pCurrRxPkt);
        TCPIP_UDP_ArrayGet(s, 0, wOffset);
    }
}




uint16_t TCPIP_UDP_PutIsReady(UDP_SOCKET s)
{
    UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(s);
    if(pSkt == 0)
    { 
        return 0;
    }

#if defined (TCPIP_STACK_USE_IPV6)
    if(pSkt->addType == IP_ADDRESS_TYPE_IPV6)
    {
        return _UDPv6IsTxPutReady(pSkt, pSkt->txSize);
    }
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
    if(pSkt->addType == IP_ADDRESS_TYPE_IPV4)
    {
        return _UDPv4IsTxPutReady(pSkt);
    }
#endif  // defined (TCPIP_STACK_USE_IPV4)

    return 0;   // can happen if it is a server socket and opened with IP_ADDRESS_TYPE_ANY
                // and no client connected to it

}

uint16_t TCPIP_UDP_TxPutIsReady(UDP_SOCKET s, unsigned short count)
{
    return TCPIP_UDP_PutIsReady(s);
}



uint16_t TCPIP_UDP_Put(UDP_SOCKET s, uint8_t v)
{
        UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(s);

        if(pSkt && pSkt->txStart)
        {
            if(pSkt->txWrite < pSkt->txEnd)
            {
                *pSkt->txWrite++ = v;
                return 1;
            }
        }

    return 0;
}

uint16_t TCPIP_UDP_ArrayPut(UDP_SOCKET s, const uint8_t *cData, uint16_t wDataLen)
{
    if(cData != 0 && wDataLen != 0)
    {
        UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(s);

        if(pSkt && pSkt->txStart)
        {
            uint16_t wrSpace = pSkt->txEnd - pSkt->txWrite;
            if(wDataLen > wrSpace)
            {
                wDataLen = wrSpace;
            }

            if(wDataLen)
            {
                memcpy(pSkt->txWrite, cData, wDataLen);
                pSkt->txWrite += wDataLen;
            }

            return wDataLen;
        }
    }

    return 0;
}

const uint8_t* TCPIP_UDP_StringPut(UDP_SOCKET s, const uint8_t *strData)
{
    if(strData)
    {
        UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(s);
        if(pSkt)
        {
            return strData + TCPIP_UDP_ArrayPut(s, strData, strlen((char*)strData));
        }
    }

    return 0;
}



uint16_t TCPIP_UDP_Flush(UDP_SOCKET s)
{
    uint16_t payload;
    UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(s);

    if(pSkt && pSkt->pPkt)
    {
        if(pSkt->flags.txSplitAlloc == 0)
        {
            payload = pSkt->txWrite - pSkt->txStart;
        }
        else
        {
            payload = ((UDP_V4_PACKET*)pSkt->pPkt)->zcSeg->segLen;
        }

        if(payload)
        {
#if defined (TCPIP_STACK_USE_IPV6)
            if(pSkt->addType == IP_ADDRESS_TYPE_IPV6)
            {
                return _UDPv6Flush(pSkt);
            }
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
            if(pSkt->addType == IP_ADDRESS_TYPE_IPV4)
            {
                return _UDPv4Flush(pSkt);
            }
#endif  // defined (TCPIP_STACK_USE_IPV4)
        }

    }
    return 0;
}


uint16_t TCPIP_UDP_TxCountGet(UDP_SOCKET s)
{
    UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(s);

    if(pSkt)
    {
        return pSkt->txWrite - pSkt->txStart;
    }

    return 0;
}


/****************************************************************************
  Section:
	Receive Functions
  ***************************************************************************/

uint16_t TCPIP_UDP_GetIsReady(UDP_SOCKET s)
{
    UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(s);
    
    if(pSkt == 0)
    { 
        return 0;
    }

    if(pSkt->pCurrRxSeg == 0)
    {   // no more data in this packet 
        _UDPUpdatePacket(pSkt);
    }

    return pSkt->pCurrRxSeg == 0 ? 0 : pSkt->rxTotLen;

}

uint16_t TCPIP_UDP_Get(UDP_SOCKET s, uint8_t *v)
{
    return TCPIP_UDP_ArrayGet(s, v, 1);
}


uint16_t TCPIP_UDP_ArrayGet(UDP_SOCKET s, uint8_t *cData, uint16_t reqBytes)
{
    TCPIP_MAC_DATA_SEGMENT *pSeg;
    uint16_t    xtractBytes;
    uint16_t    avlblBytes;

    UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(s);

    if(reqBytes == 0 || pSkt == 0)
    {   // do not advance the current RX packet if no data is requested
        return 0;
    }
    
    if(pSkt->pCurrRxSeg == 0)
    {   // no more data in this packet 
        _UDPUpdatePacket(pSkt);
    }

    avlblBytes = 0;
    while(reqBytes != 0 && (pSeg = pSkt->pCurrRxSeg) != 0 && pSkt->rxTotLen != 0)
    {
        xtractBytes = reqBytes <= pSkt->rxSegLen ? reqBytes : pSkt->rxSegLen;
        if(xtractBytes > pSkt->rxTotLen)
        {
            xtractBytes = pSkt->rxTotLen;
        }

        if(xtractBytes)
        {
            if(cData != 0)
            {
                memcpy(cData, pSkt->rxCurr, xtractBytes);
                cData += xtractBytes;
            }
            // adjust
            reqBytes -= xtractBytes;
            avlblBytes += xtractBytes;

            pSkt->rxTotLen -= xtractBytes;
            pSkt->rxSegLen -= xtractBytes;
            pSkt->rxCurr += xtractBytes;
        }

        if(pSkt->rxSegLen == 0)
        {   // go to the next segment in the packet
            pSeg = pSeg->next;
            if((pSkt->pCurrRxSeg = pSeg) != 0)
            {
                pSkt->rxSegLen = pSeg->segLen;
                pSkt->rxCurr = pSeg->segLoad;
            }
            else
            {
                pSkt->rxSegLen = 0;
                pSkt->rxCurr = 0;
            }
        }
        // else more data in this segment
    }

    if(pSkt->rxTotLen == 0)
    {   // done with this packet
        _UDPUpdatePacket(pSkt);
    }
        
    return avlblBytes;
}

void TCPIP_UDP_Discard(UDP_SOCKET s)
{
    UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(s);

    if(pSkt)
    {
        _UDPUpdatePacket(pSkt);
    }
}


/*****************************************************************************
  Function:
	static UDP_SOCKET_DCPT* _UDPFindMatchingSocket(   TCPIP_NET_IF* pPktIf, UDP_HEADER *h, IP_ADDRESS_TYPE addressType)

  Summary:
	Matches an incoming UDP segment to a currently active socket.
	
  Description:
	This function attempts to match an incoming UDP segment to a currently
	active socket for processing.

  Precondition:
	UDP segment header and IP header have both been retrieved.

  Parameters:
    pPktIf - interface the UDP packet/header belongs to  
	h - The UDP header that was received.
	
  Returns:
  	A UDP_SOCKET_DCPT handle of a matching socket, or 0 when no
  	match could be made.
  ***************************************************************************/
static UDP_SOCKET_DCPT* _UDPFindMatchingSocket( TCPIP_NET_IF* pPktIf, UDP_HEADER *h, IP_ADDRESS_TYPE addressType)
{
    int sktIx;
    UDP_SOCKET_DCPT* partialMatch;
    UDP_SOCKET_DCPT *pSkt;

    partialMatch = 0;

    for(sktIx = 0; sktIx < nUdpSockets; sktIx++)
    {
        pSkt = UDPSocketDcpt[sktIx];
        // This packet is said to be matching with current socket:
        // 1. If its destination port matches with our local port and
        // 2. Packet source IP address matches with previously saved socket remote IP address and
        // 3. Packet source port number matches with previously saved socket remote port number
        if(pSkt && (pSkt->addType == addressType || pSkt->addType == IP_ADDRESS_TYPE_ANY))
        {   // can handle this address type
            if(pSkt->localPort == h->DestinationPort)
            {
                if(pSkt->flags.looseRemPort || (pSkt->remotePort == h->SourcePort))
                {
                    if(pSkt->flags.looseNetIf)
                    {   // not caring about incoming interface
                        partialMatch = pSkt;
                        break;
                    }

#if defined(TCPIP_STACK_USE_IPV6)
                    if(addressType == IP_ADDRESS_TYPE_IPV6)
                    {
                        if(pSkt->pSktNet != 0)
                        {
                            if(TCPIP_IPV6_AddressFind(pSkt->pSktNet, TCPIP_IPV6_DestAddressGet(pSkt->pV6Pkt), IPV6_ADDR_TYPE_UNICAST) != 0)
                            {   // interface match
                                if(pSkt->flags.looseRemPort || pSkt->addType == IP_ADDRESS_TYPE_ANY)
                                {   // the port and IF have to be adjusted
                                    partialMatch = pSkt;
                                    break;
                                }
                                else
                                {   // perfect match : port + interface
                                    pSkt->addType = addressType;
                                    return pSkt;
                                }
                            }
                            // else interface mismatch, continue
                        }
                        else
                        {   // incoming interface not specified and we have remote port match
                            partialMatch = pSkt;
                            break;
                        }
                    }
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
                    if(addressType == IP_ADDRESS_TYPE_IPV4)
                    {
                        if(pSkt->pSktNet == pPktIf)
                        {   // interface match
                            if(pSkt->flags.looseRemPort || pSkt->addType == IP_ADDRESS_TYPE_ANY)
                            {   // the port and IF have to be adjusted
                                pSkt->addType = addressType;
                                partialMatch = pSkt;
                                break;
                            }
                            else
                            {   // perfect match : port + interface
                                return pSkt;
                            }
                        }
                        else if(pSkt->pSktNet == 0)
                        {   // incoming interface not specified and we have remote port match
                            partialMatch = pSkt;
                            break;
                        }
                    }
#endif  // defined (TCPIP_STACK_USE_IPV4)
                }
                else if(pSkt->remotePort == 0)
                {   // not connected yet, can accept requests from any port
                    if(pSkt->flags.looseNetIf)
                    {   // interface does not matter
                        partialMatch = pSkt;
                        break;
                    }
                    else if(pSkt->pSktNet == 0 || pSkt->pSktNet == pPktIf)
                    {   // there could be other server socket waiting
                        partialMatch = pSkt;
                    }
                }
            }
        }
    }

    if(partialMatch == 0)
    {
        return 0;
    }

    // we have a server socket that can handle this packet
    pSkt = partialMatch;
#if defined (TCPIP_STACK_USE_IPV6)
    if (addressType == IP_ADDRESS_TYPE_IPV6)
    {   // lazy allocation does not work for IPv6
        if(pSkt->pV6Pkt == 0)
        {   // could be a server socket opened with IP_ADDRESS_TYPE_ANY
            if(!_UDPv6AllocateTxPacketStruct(pPktIf, pSkt, true))
            {   // failed to allocate memory
                return 0;
            }
        }
    }
#endif  // defined (TCPIP_STACK_USE_IPV6)


    pSkt->addType = addressType;
    pSkt->remotePort = h->SourcePort;
    return pSkt;
}


bool TCPIP_UDP_SocketNetSet(UDP_SOCKET s, TCPIP_NET_HANDLE hNet)
{
    UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(s);
    if(pSkt)
    {
        TCPIP_NET_IF* pIf = _TCPIPStackHandleToNetUp(hNet);
        // don't do any check here;
        // user can clear the assigned interface

        pSkt->pSktNet = pIf;

#if defined (TCPIP_STACK_USE_IPV6)
        if(pSkt->addType == IP_ADDRESS_TYPE_IPV6)
        {
            if(pSkt->pV6Pkt == 0)
            {
                return false;
            }

            pSkt->pV6Pkt->netIfH = pIf;
        }
#endif  // defined (TCPIP_STACK_USE_IPV6)

        return true;
    }
    return false;
}

TCPIP_NET_HANDLE TCPIP_UDP_SocketNetGet(UDP_SOCKET s)
{
    UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(s);
    
    return pSkt?pSkt->pSktNet:0;
}

// sets the source IP address of a packet
bool TCPIP_UDP_SourceIPAddressSet(UDP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* localAddress)
{
    UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(s);

    return pSkt ? _UDPSetSourceAddress(pSkt, addType, localAddress) : false;
}

bool TCPIP_UDP_Bind(UDP_SOCKET s, IP_ADDRESS_TYPE addType, UDP_PORT localPort,  IP_MULTI_ADDRESS* localAddress)
{
    TCPIP_NET_IF* pSktIf;
    UDP_SOCKET_DCPT* pSkt;

    pSkt = _UDPSocketDcpt(s);
    if(pSkt == 0)
    {
        return false;
    }

    if(pSkt->addType != IP_ADDRESS_TYPE_ANY && pSkt->addType != addType)
    {   // address type mismatch
        return false;
    }

    // check for valid address
    pSktIf = 0;

    if(localAddress != 0)
    {
#if defined (TCPIP_STACK_USE_IPV4)
        if (addType == IP_ADDRESS_TYPE_IPV4)
        {
            pSktIf = TCPIP_STACK_IPAddToNet(&localAddress->v4Add, false);
        }
#else
        if (addType == IP_ADDRESS_TYPE_IPV6)
        {
            pSktIf = _TCPIPStackIPv6AddToNet(&localAddress->v6Add, IPV6_ADDR_TYPE_UNICAST, false);
        }
#endif  // defined (TCPIP_STACK_USE_IPV4)

        if(pSktIf == 0)
        {    // no such interface
            return false;
        }
    }

    if(localPort == 0)
    {
        localPort = _UDPAllocateEphemeralPort();
        if(localPort == 0)
        {
            return false;
        }
    }
    else if(localPort != pSkt->localPort && _UDPIsSharedPortAvailable(localPort, pSktIf) == false)
    {
        return false;
    }

    // success; bind
    pSkt->addType = addType;
    _UDPSocketBind(pSkt, pSktIf, localAddress);
    pSkt->localPort = localPort;
    return true;
}

bool TCPIP_UDP_RemoteBind(UDP_SOCKET s, IP_ADDRESS_TYPE addType, UDP_PORT remotePort,  IP_MULTI_ADDRESS* remoteAddress)
{
    UDP_SOCKET_DCPT *pSkt = _UDPSocketDcpt(s);

    if(pSkt != 0)
    {
        if(remoteAddress == 0 || TCPIP_UDP_DestinationIPAddressSet(s, addType, remoteAddress) == true)
        {
            pSkt->remotePort = remotePort;
            return true;
        }
    }

    return false;
}

// Allows setting options to a socket like enable broadcast, Rx/Tx buffer size, etc
bool TCPIP_UDP_OptionsSet(UDP_SOCKET hUDP, UDP_SOCKET_OPTION option, void* optParam)
{
    UDP_SOCKET_DCPT*  pSkt = _UDPSocketDcpt(hUDP);

    if(pSkt)
    {
        switch(option)
        {
            case UDP_OPTION_STRICT_PORT:
                pSkt->flags.looseRemPort = (optParam == 0);
                return true;


            case UDP_OPTION_STRICT_NET:
                pSkt->flags.looseNetIf = (optParam == 0);
                return true;

            case UDP_OPTION_BROADCAST:
                pSkt->flags.bcastForceType = (UDP_SOCKET_BCAST_TYPE)(optParam);
                // else the discrete address set takes effect
                return true;

            case UDP_OPTION_BUFFER_POOL:
#if defined (TCPIP_STACK_USE_IPV6)
                if(pSkt->addType == IP_ADDRESS_TYPE_IPV6)
                {   
                    return false;
                }
#endif  // defined (TCPIP_STACK_USE_IPV6)
                if(pSkt->flags.txSplitAlloc != 0)
                {   // no support for external payload sockets
                    return false;
                }

                if(pSkt->flags.usePool != (optParam != 0))
                {   // changed the buffer type; just release the packet
                    _UDPFreeTxResources(pSkt);
                    pSkt->flags.usePool = (optParam != 0);
                }
                return true;


            case UDP_OPTION_TX_BUFF:
                // just release the packet
                _UDPFreeTxResources(pSkt);
                pSkt->txSize = (uint16_t)(unsigned int)optParam;
                return true;

            case UDP_OPTION_TX_QUEUE_LIMIT:
                pSkt->txAllocLimit = (uint8_t)(unsigned int)optParam;
                return true;

            default:
                break;
        }
    }    

    return false;
}

// Allows getting options to a socket like enable broadcast, Rx/Tx buffer size, etc
bool TCPIP_UDP_OptionsGet(UDP_SOCKET hUDP, UDP_SOCKET_OPTION option, void* optParam)
{
    UDP_SOCKET_DCPT *pSkt = _UDPSocketDcpt(hUDP);

    if(pSkt && optParam)
    {
        switch(option)
        {
            case UDP_OPTION_STRICT_PORT:
                *(bool*)optParam = pSkt->flags.looseRemPort == 0;
                return true;

            case UDP_OPTION_STRICT_NET:
                *(bool*)optParam = pSkt->flags.looseNetIf == 0;
                return true;

            case UDP_OPTION_BROADCAST:
                *(UDP_SOCKET_BCAST_TYPE*)optParam = (UDP_SOCKET_BCAST_TYPE)pSkt->flags.bcastForceType;
                return true;

            case UDP_OPTION_BUFFER_POOL:
                *(bool*)optParam = pSkt->flags.usePool != 0;
                return true;

            case UDP_OPTION_TX_BUFF:
                *(uint16_t*)optParam = pSkt->txSize;
                return true;
                
            case UDP_OPTION_TX_QUEUE_LIMIT:
                *(uint8_t*)optParam = pSkt->txAllocLimit;
                return true;

            default:
                break;
        }
    }    

    return false;
}


bool TCPIP_UDP_DestinationIPAddressSet(UDP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* remoteAddress)
{
    UDP_SOCKET_DCPT *pSkt;


    if(remoteAddress == 0)
    {
        return false;
    }

    pSkt = _UDPSocketDcpt(s);

    while(pSkt != 0 && pSkt->addType == addType)
    {
#if defined (TCPIP_STACK_USE_IPV6)
        if (pSkt->addType == IP_ADDRESS_TYPE_IPV6)
        {
            if(pSkt->pV6Pkt != 0)
            {
                TCPIP_IPV6_DestAddressSet (pSkt->pV6Pkt, &remoteAddress->v6Add);
                return true;
            }
            break;
        }
#endif  // defined (TCPIP_STACK_USE_IPV6)

#if defined (TCPIP_STACK_USE_IPV4)
        if (pSkt->addType == IP_ADDRESS_TYPE_IPV4)
        {
            if(pSkt->flags.bcastForceType != UDP_BCAST_NONE)
            {   // BCAST is already set and cannot be overridden!
                return false;
            }
            pSkt->destAddress.Val = remoteAddress->v4Add.Val;
            pSkt->flags.destSet = 1;
            return true;
        }
#endif  // defined (TCPIP_STACK_USE_IPV4)

        break;
    }

    return false;
}

bool UDPSetSplitPayload(UDP_SOCKET s, void* pLoad, uint16_t loadSize)
{
    UDP_SOCKET_DCPT *pSkt = _UDPSocketDcpt(s);

    if(pSkt && pSkt->flags.txSplitAlloc && pSkt->addType == IP_ADDRESS_TYPE_IPV4)
    {
        if(pSkt->pPkt == 0)
        {
            _UDPv4PAllocateSktTxBuffer(pSkt, pSkt->addType, true);
        }

        if(pSkt->pPkt != 0)
        {
            TCPIP_MAC_DATA_SEGMENT* pZSeg = ((UDP_V4_PACKET*)pSkt->pPkt)->zcSeg;
            pZSeg->segLen = pZSeg->segSize = loadSize;
            pZSeg->segLoad = pLoad;

            return true;
        }
    }

    return false;
}



static UDP_PORT _UDPAllocateEphemeralPort(void)
{
    int      num_ephemeral;
    int      count;
    UDP_PORT next_ephemeral;


    count = num_ephemeral = TCPIP_UDP_LOCAL_PORT_END_NUMBER - TCPIP_UDP_LOCAL_PORT_START_NUMBER + 1;

    next_ephemeral = TCPIP_UDP_LOCAL_PORT_START_NUMBER + (SYS_RANDOM_PseudoGet() % num_ephemeral);

    while(count--)
    {
        if(_UDPIsAvailablePort(next_ephemeral))
        {
            return next_ephemeral;
        }

        if (next_ephemeral == TCPIP_UDP_LOCAL_PORT_END_NUMBER)
        {
            next_ephemeral = TCPIP_UDP_LOCAL_PORT_START_NUMBER;
        }
        else
        {
            next_ephemeral++;
        }
    }

    return 0;   // not found
}

static bool _UDPIsAvailablePort(UDP_PORT port)
{
    int skt;
    UDP_SOCKET_DCPT *pSkt;

    // Find an available socket that matches the specified socket type
    for(skt = 0; skt < nUdpSockets; skt++)
    {
        pSkt = UDPSocketDcpt[skt]; 
        if(pSkt && pSkt->localPort == port)
        {
            return false;
        }
    }

    return true;
}

// checks that a port can be shared by multiple sockets
// the socket have to receive data from different interfaces
static bool _UDPIsSharedPortAvailable(UDP_PORT port, TCPIP_NET_IF* pNet)
{
    int skt;
    UDP_SOCKET_DCPT *pSkt;

    // Find an available socket that matches the specified socket type
    for(skt = 0; skt < nUdpSockets; skt++)
    {
        pSkt = UDPSocketDcpt[skt]; 
        if(pSkt && pSkt->localPort == port)
        {
            if(pSkt->flags.looseNetIf != 0 || pSkt->pSktNet == 0 || pSkt->pSktNet == pNet)
            {
                return false;
            }
        }
    }

    return true;
}

// debug/trace support
//

#if defined (TCPIP_UDP_DEBUG)

int TCPIP_UDP_DebugSktNo(void)
{
    return nUdpSockets;
}

bool TCPIP_UDP_DebugSktInfo(int sktNo, TCPIP_UDP_SKT_DEBUG_INFO* pInfo)
{
    UDP_SOCKET_DCPT* pSkt = _UDPSocketDcpt(sktNo);
    if(pSkt)
    {
        pInfo->addType = pSkt->addType;
        pInfo->remotePort = pSkt->remotePort;
        pInfo->localPort = pSkt->localPort;
        pInfo->rxQueueSize = TCPIP_Helper_SingleListCount(&pSkt->rxQueue);
        pInfo->txSize = pSkt->txEnd - pSkt->txStart;

        return true;
    }


    return false;
}


#endif // defined (TCPIP_UDP_DEBUG)

#endif //#if defined(TCPIP_STACK_USE_UDP)
