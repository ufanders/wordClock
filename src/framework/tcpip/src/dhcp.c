/*******************************************************************************
  Dynamic Host Configuration Protocol (DHCP) Client

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    - Provides automatic IP address, subnet mask, gateway address, 
      DNS server address, and other configuration parameters on DHCP 
      enabled networks.
    - Reference: RFC 2131, 2132
*******************************************************************************/

/*******************************************************************************
File Name:  DHCP.c
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

#define __DHCP_C

#include "tcpip/src/tcpip_private.h"
#include "tcpip/src/dhcp_private.h"

#if !defined(TCPIP_STACK_USE_DHCP_CLIENT)

// provide basic/common functions for DHCP access when disabled
bool TCPIP_DHCP_IsEnabled(TCPIP_NET_HANDLE hNet)
{
    return false;
}

bool TCPIP_DHCP_Disable(TCPIP_NET_HANDLE hNet)
{
    return false;
}

bool TCPIP_DHCP_Enable(TCPIP_NET_HANDLE hNet)
{
    return false;
}

bool TCPIP_DHCP_Renew(TCPIP_NET_HANDLE hNet)
{
    return false;
}


bool TCPIP_DHCP_Request(TCPIP_NET_HANDLE hNet, IPV4_ADDR reqAddress)
{
    return false;
}

#else   // defined(TCPIP_STACK_USE_DHCP_CLIENT)

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_DHCP_CLIENT
#include "tcpip/src/tcpip_notify.h"
#include "system/tmr/sys_tmr.h"

// Unique variables per interface
typedef struct
{
	UDP_SOCKET			hDHCPSocket;	// Handle to DHCP client socket
	SM_DHCP				smState;		// DHCP client state machine variable
    DHCP_OPERATION_TYPE dhcpOp;         // DHCP current operation
	uint32_t			dwTimer;		// Tick timer value used for triggering future events after a certain wait period.
	uint32_t				dwLeaseTime;	// DHCP lease time remaining, in seconds
	uint32_t				dwServerID;		// DHCP Server ID cache
	IPV4_ADDR				tempIPAddress;	// Temporary IP address to use when no DHCP lease
	IPV4_ADDR				tempGateway;	// Temporary gateway to use when no DHCP lease
	IPV4_ADDR				tempMask;		// Temporary mask to use when no DHCP lease
	#if defined(TCPIP_STACK_USE_DNS)
	IPV4_ADDR				tempDNS;		// Temporary primary DNS server
	IPV4_ADDR				tempDNS2;		// Temporary secondary DNS server
	#endif	
    TCPIP_UINT32_VAL           transactionID;  // current transaction ID
	union
	{
	    struct
	    {
	        unsigned char bDHCPEnabled        : 1;		// Whether or not DHCP is currently enabled
	        unsigned char bIsBound            : 1;		// Whether or not DHCP is currently bound
	        unsigned char bOfferReceived      : 1;		// Whether or not an offer has been received
			unsigned char bDHCPServerDetected : 1;	    // Indicates if a DCHP server has been detected
            unsigned char bSetRunFail         : 1;      // if set, report a run fail signal
            unsigned char bIfUp               : 1;      // interface up flag
            unsigned char reserved            : 2;      // not used

	    };
	    uint8_t val;
	} flags;
	// Indicates which DHCP values are currently valid
	union
	{
		struct
		{
			char IPAddress:1;	// Leased IP address is valid
			char Gateway:1;		// Gateway address is valid
			char Mask:1;		// Subnet mask is valid
			char DNS:1;			// Primary DNS is valid
			char DNS2:1;		// Secondary DNS is valid
			char HostName:1;	// Host name is valid (not implemented)
		};
		uint8_t val;
	} validValues;
} DHCP_CLIENT_VARS;

static uint8_t  _DHCPReceive(DHCP_CLIENT_VARS* pClient, TCPIP_NET_IF* pNetIf);
static void     _DHCPSend(DHCP_CLIENT_VARS* pClient, TCPIP_NET_IF* pNetIf, uint8_t messageType, bool bRenewing);
static void     _DHCPNotifyClients(TCPIP_NET_IF* pNetIf, TCPIP_DHCP_EVENT_TYPE evType);
static bool     _DHCPStartOperation(TCPIP_NET_IF* pNetIf, DHCP_OPERATION_REQ opReq, uint32_t reqAddress);

static bool     _DHCPPacketFilter(TCPIP_MAC_PACKET* pRxPkt, const void* param);

static DHCP_CLIENT_VARS*	DHCPClients = 0;    // DHCP client per interface 

static int                  dhcpInitCount = 0;      // DHCP module initialization count

static UDP_PORT             dhcpClientPort;
static UDP_PORT             dhcpServerPort;

static unsigned int         dhcpTmo, newDhcpTmo;

static tcpipAsyncHandle       dhcpTimerHandle = 0;

static IPV4_FILTER_HANDLE   dhcpFilterHandle = 0;       // IPv4 packet filter

static const DHCP_MODULE_CONFIG dhcpConfigDefault = 
{
    DHCP_CLIENT_ENABLED,
    DHCP_TIMEOUT,
    TCPIP_DHCP_CLIENT_PORT,
    TCPIP_DHCP_SERVER_PORT,
};

// DHCP event registration

static const void*      dhcpMemH = 0;        // memory handle

static PROTECTED_SINGLE_LIST      dhcpRegisteredUsers = { {0} };



static void _DHCPClose(TCPIP_NET_IF* pNetIf, bool disable)
{
    DHCP_CLIENT_VARS* pClient = DHCPClients + TCPIP_STACK_NetIxGet(pNetIf);
    
    if(pClient->flags.bDHCPEnabled != 0)
    {
        if(pClient->hDHCPSocket != INVALID_UDP_SOCKET)
        {
            TCPIP_UDP_Close(pClient->hDHCPSocket);
            pClient->hDHCPSocket = INVALID_UDP_SOCKET;
        }
        
		pClient->flags.bIsBound = false;
        pClient->flags.bDHCPServerDetected = false;
        
        if(disable)
        {
            pClient->flags.bDHCPEnabled = false;
            pNetIf->Flags.bIsDHCPEnabled = false;
        }
        else
        {   // let it active
            pClient->smState = SM_DHCP_IDLE;
        }
    }
}

static void _DHCPEnable(TCPIP_NET_IF* pNetIf, bool waitConnUp, DHCP_OPERATION_TYPE opType)
{
    DHCP_CLIENT_VARS* pClient = DHCPClients + TCPIP_STACK_NetIxGet(pNetIf);
    
    if(waitConnUp && TCPIP_STACK_NetworkIsLinked(pNetIf))
    {   // the idea is that if the link is down we'll get an connection up event anyway
        // to wake us up from SM_DHCP_IDLE
        pClient->smState = SM_DHCP_WAIT_LINK;
    }
    else
    {
        pClient->smState = SM_DHCP_IDLE;
    }
    
    pClient->flags.bDHCPEnabled = true;
    pClient->dhcpOp = opType;
    pNetIf->Flags.bIsDHCPEnabled = true;


    if(dhcpFilterHandle == 0)
    {
        dhcpFilterHandle = IPv4RegisterFilter(_DHCPPacketFilter, 0);
        if(dhcpFilterHandle == 0)
        {
            SYS_ERROR(SYS_ERROR_WARN, "DHCP: Failed to register IPv4 filter! \r\n");
        }
    }
}

static void _DHCPCleanup(void)
{
    TCPIP_HEAP_Free(dhcpMemH, DHCPClients);
    DHCPClients = 0;

    TCPIP_Notification_RemoveAll(&dhcpRegisteredUsers, dhcpMemH);
        
    if(dhcpFilterHandle != 0)
    {
        Ipv4DeRegisterFilter(dhcpFilterHandle);
        dhcpFilterHandle = 0;
    }

    if(dhcpTimerHandle)
    {
        _TCPIPStackAsyncHandlerDeRegister(dhcpTimerHandle);
        dhcpTimerHandle = 0;
    }
}

static void _DHCPSetRunFail(DHCP_CLIENT_VARS* pClient, SM_DHCP newState) 
{
    pClient->smState = newState;
    pClient->flags.bSetRunFail = 1;
    pClient->dhcpOp = DHCP_OPER_INIT;       // failure forces a brand new lease acquisition

}

/*****************************************************************************
  Function:
    bool TCPIP_DHCP_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const DHCP_MODULE_CONFIG* pDhcpConfig);

  Summary:
	Resets the DHCP client module for the specified interface.

  Description:
	Resets the DHCP client module, giving up any current lease, knowledge of 
	DHCP servers, etc. for the specified interface.

  Precondition:
	None

  Parameters:
	stackCtrl - pointer to stack structure specifying the interface to initialize

  Returns:
	None

  Remarks:
	This function should be called internally just once per interface 
    by the stack manager.
***************************************************************************/
bool TCPIP_DHCP_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const DHCP_MODULE_CONFIG* pDhcpConfig)
{
    DHCP_CLIENT_VARS*   pClient;
	
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        if(stackCtrl->pNetIf->Flags.bIsDHCPEnabled != 0)
        {
            _DHCPEnable(stackCtrl->pNetIf, true, DHCP_OPER_INIT);
        }
        return true;
    }

    // stack init
    
    // select default configuration if init data is missing
    if(pDhcpConfig == 0)
    {
        pDhcpConfig = &dhcpConfigDefault;
    }

    if(dhcpInitCount == 0)
    {   // first time we're run
        // store the memory allocation handle
        dhcpMemH = stackCtrl->memH;
        
        DHCPClients = (DHCP_CLIENT_VARS*)TCPIP_HEAP_Calloc(dhcpMemH,  stackCtrl->nIfs, sizeof(DHCP_CLIENT_VARS));
        if(DHCPClients == 0)
        {   // failed
            return false;
        }

        dhcpFilterHandle = 0;

        // create the DHCP timer
        dhcpTimerHandle = _TCPIPStackAsyncHandlerRegister(TCPIP_DHCP_Task, 0, DHCP_TASK_TICK_RATE);
        if(dhcpTimerHandle == 0)
        {   // cannot create the DHCP timer
            _DHCPCleanup();
            return false;
        }
        
        // initialize the clients
        int ix;
        for(ix = 0, pClient = DHCPClients; ix < stackCtrl->nIfs; ix++, pClient++)
        {
            pClient->hDHCPSocket = INVALID_UDP_SOCKET;
        }

        TCPIP_Helper_ProtectedSingleListInitialize(&dhcpRegisteredUsers);
    }
            
    pClient = DHCPClients + stackCtrl->netIx;

    // Reset state machine and flags to default values
    pClient->flags.val = 0;

    // update the DHCP parameters
    dhcpTmo = newDhcpTmo = pDhcpConfig->dhcpTmo;
    dhcpClientPort = pDhcpConfig->dhcpCliPort;
    dhcpServerPort = pDhcpConfig->dhcpSrvPort;

    if(stackCtrl->pNetIf->Flags.bIsDHCPEnabled != 0)
    {   // override the pDhcpConfig->dhcpEnable passed with the what the stack manager says
        _DHCPEnable(stackCtrl->pNetIf, true, DHCP_OPER_INIT);
    }

    dhcpInitCount++;


    return true;
}

/*****************************************************************************
  Function:
    bool DHCPDeinit(const TCPIP_STACK_MODULE_CTRL* const stackCtrl);

  Summary:
	Turns off the DHCP client module for the specified interface.

  Description:
	Closes out UDP socket.

  Precondition:
	None

  Parameters:
	stackData - pointer to stack structure specifying the interface to deinitialize

  Returns:
	None

  Remarks:
	This function should be called internally just once per interface 
    by the stack manager.
***************************************************************************/
void TCPIP_DHCP_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down

    // one way or another this interface is going down
    _DHCPClose(stackCtrl->pNetIf, true);

    //  the registered users for this interface are not removed
    //  since this interface is closed there won't be any event generated on it anyway
    //  deallocation will wait for the whole stack to deinit 
    
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // whole stack is going down
        if(dhcpInitCount > 0)
        {   // we're up and running
            if(--dhcpInitCount == 0)
            {   // all closed
                // release resources
                _DHCPCleanup();
                dhcpMemH = 0;
            }
        }
    }

}


/*****************************************************************************
  Function:
	void TCPIP_DHCP_Disable(TCPIP_NET_HANDLE hNet)

  Summary:
	Disables the DHCP Client for the specified interface.

  Description:
	Disables the DHCP client for the specified interface.

  Precondition:
	None

  Parameters:
	pNetIf - Interface to disable the DHCP client on.

  Returns:
	true if success
    false otherwise

  Remarks:
	When the interface continues using its old configuration, it is possible 
	that the lease may expire and the DHCP server provide the IP to another
	client.
    The application should not request the keeping of the old lease
    unless there is no danger of conflict.
***************************************************************************/
bool TCPIP_DHCP_Disable(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(hNet);
    if(pNetIf)
    {
        _DHCPClose(pNetIf, true);
        TCPIP_STACK_AddressServiceEvent(pNetIf, TCPIP_STACK_ADDRESS_SERVICE_DHCPC, TCPIP_STACK_ADDRESS_SERVICE_EVENT_USER_STOP);
        _DHCPNotifyClients(pNetIf, DHCP_EVENT_SERVICE_DISABLED);
        return true;
    }

    return false;
}


bool TCPIP_DHCP_Enable(TCPIP_NET_HANDLE hNet)
{
    return _DHCPStartOperation(_TCPIPStackHandleToNetUp(hNet), DHCP_OP_REQ_ENABLE, 0);
}

// dhcp.h
bool TCPIP_DHCP_Renew(TCPIP_NET_HANDLE hNet)
{
    return _DHCPStartOperation(_TCPIPStackHandleToNetUp(hNet), DHCP_OP_REQ_RENEW, 0);
}


bool TCPIP_DHCP_Request(TCPIP_NET_HANDLE hNet, IPV4_ADDR reqAddress)
{
    return _DHCPStartOperation(_TCPIPStackHandleToNetUp(hNet), DHCP_OP_REQ_REQUEST, reqAddress.Val);
}

static bool _DHCPStartOperation(TCPIP_NET_IF* pNetIf, DHCP_OPERATION_REQ opReq, uint32_t reqAddress)
{
    DHCP_OPERATION_TYPE opType = DHCP_OPER_NONE;
    if(pNetIf == 0)
    {
        return false;
    }

    DHCP_CLIENT_VARS* pClient = DHCPClients + TCPIP_STACK_NetIxGet(pNetIf);

    switch(opReq)
    {
        case DHCP_OP_REQ_ENABLE:
            if(pClient->flags.bDHCPEnabled != 0)
            {   // already enabled
                return true;
            }

            if(TCPIP_STACK_AddressServiceCanStart(pNetIf, TCPIP_STACK_ADDRESS_SERVICE_DHCPC))
            {
                opType = DHCP_OPER_INIT;
            }
            break;

        case DHCP_OP_REQ_RENEW:
            // DHCP should be running and bound
            if(pClient->flags.bDHCPEnabled != 0 && pClient->flags.bIsBound != 0)
            {
                opType = DHCP_OPER_RENEW;
            }
            break;

        case DHCP_OP_REQ_REQUEST:
            // if DHCP not running, then it needs to be enabled
            if(reqAddress != 0)
            {
                if(pClient->flags.bDHCPEnabled != 0)
                {   // depending on the client state
                    if(pClient->smState == SM_DHCP_GET_REQUEST_ACK || 
                       pClient->smState == SM_DHCP_GET_RENEW_ACK || 
                       pClient->smState == SM_DHCP_GET_RENEW_ACK2 || 
                       pClient->smState == SM_DHCP_GET_RENEW_ACK3)
                    {
                       return false;
                    } 
                    // make sure any previous conversation is discarded
                    _DHCPClose(pNetIf, false);
                }
                else if(!TCPIP_STACK_AddressServiceCanStart(pNetIf, TCPIP_STACK_ADDRESS_SERVICE_DHCPC))
                {
                    return false;
                }
                
                pClient->tempIPAddress.Val = reqAddress;
                opType = DHCP_OPER_INIT_REBOOT;
            }

            break;
        
        default:
            break;
    }

    if(opType != DHCP_OPER_NONE)
    {
        _DHCPEnable(pNetIf, true, opType);
        return true;
    }

    return false;
}

/*****************************************************************************
  Function:
	bool TCPIP_DHCP_IsEnabled(CPIP_NET_HANDLE hNet)

  Summary:
	Determins if the DHCP client is enabled on the specified interface.

  Description:
	Determins if the DHCP client is enabled on the specified interface.

  Precondition:
	None

  Parameters:
	 hNet- Interface to query.

  Returns:
	None
***************************************************************************/
bool TCPIP_DHCP_IsEnabled(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(hNet);
    if(pNetIf)
    {
        return pNetIf->Flags.bIsDHCPEnabled != 0;
    }
    return false;
}


/*****************************************************************************
  Function:
	bool TCPIP_DHCP_IsBound(TCPIP_NET_HANDLE hNet)

  Summary:
	Determins if the DHCP client has an IP address lease on the specified 
	interface.

  Description:
	Determins if the DHCP client has an IP address lease on the specified 
	interface.

  Precondition:
	None

  Parameters:
	hNet - Interface to query

  Returns:
	true - DHCP client has obtained an IP address lease (and likely other 
		parameters) and these values are currently being used.
	false - No IP address is currently leased
***************************************************************************/
bool TCPIP_DHCP_IsBound(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(hNet);
    if(pNetIf)
    {
        return (DHCPClients + TCPIP_STACK_NetIxGet(pNetIf))->flags.bIsBound;
    }

    return false;
}


/*****************************************************************************
  Function:
	bool TCPIP_DHCP_IsServerDetected(TCPIP_NET_HANDLE hNet)

  Summary:
	Determins if the DHCP client on the specified interface has seen a DHCP 
	server.

  Description:
	Determins if the DHCP client on the specified interface has seen a DHCP 
	server.
	
  Precondition:
	None

  Parameters:
	hNet- Interface to query.

  Returns:
	true - At least one DHCP server is attached to the specified network 
		interface.
	false - No DHCP servers are currently detected on the specified network 
		interface.
***************************************************************************/
bool TCPIP_DHCP_IsServerDetected(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(hNet);
    if(pNetIf)
    {
        return (DHCPClients + TCPIP_STACK_NetIxGet(pNetIf))->flags.bDHCPServerDetected;
    }
    return false;
}

// adjust the DHCP timeout
// how long to wait before a DHCP request times out, seconds
bool TCPIP_DHCP_RequestTimeoutSet(TCPIP_NET_HANDLE hNet, int tmo)
{   
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(hNet);
    if(pNetIf && tmo)
    {
        newDhcpTmo = tmo;
        return true;
    }
    return false;
}

/*****************************************************************************
  Function:
	void TCPIP_DHCP_Task(void)

  Summary:
	Performs periodic DHCP tasks for all interfaces.

  Description:
	This function performs any periodic tasks requied by the DHCP module, 
	such as sending and receiving messages involved with obtaining and
	maintaining a lease.

  Precondition:
	None

  Parameters:
	None

  Returns:
	None
***************************************************************************/
void TCPIP_DHCP_Task(void)
{
    DHCP_CLIENT_VARS*   pClient;
    bool                udpSuccess;
    int                 netIx, nNets;
    TCPIP_NET_IF*       pNetIf;
    IPV4_ADDR           bcastAdd = {0xffffffff};

    nNets = TCPIP_STACK_NumberOfNetworksGet();
    for(netIx = 0; netIx < nNets; netIx++) 
    {
        pNetIf = (TCPIP_NET_IF*)TCPIP_STACK_IndexToNet (netIx);
        if(!TCPIP_STACK_NetworkIsUp(pNetIf))
        {   // inactive interface
            continue;
        }

        pClient = DHCPClients + TCPIP_STACK_NetIxGet(pNetIf); 

        if(pClient->flags.bDHCPEnabled == false)
        {   // not enabled on this interface
            continue;
        }

        switch(pClient->smState)
        {
            case SM_DHCP_IDLE:
                break;

            case SM_DHCP_WAIT_LINK:
                if(!TCPIP_STACK_NetworkIsLinked(pNetIf))
                {   // no connection yet
                    break;
                }

                pClient->smState = SM_DHCP_GET_SOCKET;
                // and fall through

            case SM_DHCP_GET_SOCKET:
                // Open a socket to send and receive broadcast messages on
                pClient->hDHCPSocket = UDPOpenClientSkt(IP_ADDRESS_TYPE_IPV4, dhcpServerPort, 0, UDP_OPEN_CLIENT | UDP_OPEN_CONFIG_SERVICE);
                if(pClient->hDHCPSocket == INVALID_UDP_SOCKET)
                {
                    break;
                }
                // and bind to the DHCP local port
                udpSuccess = TCPIP_UDP_Bind(pClient->hDHCPSocket, IP_ADDRESS_TYPE_IPV4, dhcpClientPort,  0);
                if(udpSuccess)
                {
                    // bind to this interface
                    TCPIP_UDP_SocketNetSet(pClient->hDHCPSocket, pNetIf);
                    // allow sending broadcast messages
                    udpSuccess = TCPIP_UDP_BcastIPV4AddressSet(pClient->hDHCPSocket, UDP_BCAST_NETWORK_LIMITED, pNetIf);
                }
                if(!udpSuccess)
                {
                    TCPIP_UDP_Close(pClient->hDHCPSocket);
                    pClient->hDHCPSocket = INVALID_UDP_SOCKET;
                    {
                        break;
                    }
                }
                pClient->smState = SM_DHCP_WAIT_OPEN;
                break;

            case SM_DHCP_WAIT_OPEN:
                // since we don't do ARP here anymore we have to wait for the UDP to do it!
                if(!TCPIP_UDP_IsOpened(pClient->hDHCPSocket))
                {
                    break;
                }

                // advance the state machine according to the DHCP operation
                if(pClient->dhcpOp == DHCP_OPER_INIT_REBOOT)
                {
                    pClient->smState = SM_DHCP_SEND_REQUEST;
                }
                else if(pClient->dhcpOp == DHCP_OPER_RENEW)
                {
                    pClient->smState = SM_DHCP_SEND_RENEW;
                }
                else
                {
                    pClient->smState = SM_DHCP_SEND_DISCOVERY;
                }
                break;
              

            case SM_DHCP_SEND_DISCOVERY:
                
                if(pClient->flags.bSetRunFail != 0)
                {
                    pClient->flags.bSetRunFail = 0;
                    TCPIP_STACK_AddressServiceEvent(pNetIf, TCPIP_STACK_ADDRESS_SERVICE_DHCPC, TCPIP_STACK_ADDRESS_SERVICE_EVENT_RUN_FAIL);
                }

                // Assume default IP Lease time of 60 seconds.
                // This should be minimum possible to make sure that if the
                // server did not specify lease time, we try again after this 
                // minimum time.
                pClient->dwLeaseTime = 60;
                pClient->validValues.val = 0x00;
                pClient->flags.bIsBound = false;	
                pClient->flags.bOfferReceived = false;


                // Ensure transmitter is ready to accept data
                if(TCPIP_UDP_TxPutIsReady(pClient->hDHCPSocket, 300) < 300u)
                    break;

                // Ensure that we transmit to the broadcast IP and MAC addresses
                // The UDP Socket remembers who it was last talking to
                TCPIP_UDP_BcastIPV4AddressSet(pClient->hDHCPSocket, UDP_BCAST_NETWORK_LIMITED, pNetIf);

                // Send the DHCP Discover broadcast
                _DHCPSend(pClient, pNetIf, DHCP_DISCOVER_MESSAGE, false);

                _DHCPNotifyClients(pNetIf, DHCP_EVENT_DISCOVER);
                // Start a timer and begin looking for a response
                pClient->dwTimer = SYS_TMR_TickCountGet();
                dhcpTmo = newDhcpTmo;
                pClient->smState = SM_DHCP_GET_OFFER;
                break;

            case SM_DHCP_GET_OFFER:
                // Check to see if a packet has arrived
                if(TCPIP_UDP_GetIsReady(pClient->hDHCPSocket) < 250u)
                {
                    // Go back and transmit a new discovery if we didn't get an offer after 2 seconds
                    if(SYS_TMR_TickCountGet() - pClient->dwTimer >= (dhcpTmo * SYS_TMR_TickPerSecond()))
                    {
                        _DHCPSetRunFail(pClient, SM_DHCP_SEND_DISCOVERY);
                    }
                    break;
                }

                // Let the DHCP server module know that there is a DHCP server 
                // on this network
                pClient->flags.bDHCPServerDetected = true;

                // Check to see if we received an offer
                if(_DHCPReceive(pClient, pNetIf) != DHCP_OFFER_MESSAGE)
                    break;

                pClient->smState = SM_DHCP_SEND_REQUEST;
                // No break

            case SM_DHCP_SEND_REQUEST:
                if(TCPIP_UDP_TxPutIsReady(pClient->hDHCPSocket, 300) < 300u)
                    break;

                // Ensure that we transmit to the broadcast IP and MAC addresses
                // The UDP Socket remembers who it was last talking to, so 
                // we must set this back to the broadcast address since the 
                // current socket values are the unicast addresses of the DHCP 
                // server.
                TCPIP_UDP_BcastIPV4AddressSet(pClient->hDHCPSocket, UDP_BCAST_NETWORK_LIMITED, pNetIf);

                // Send the DHCP request message
                _DHCPSend(pClient, pNetIf, DHCP_REQUEST_MESSAGE, false);

                // Start a timer and begin looking for a response
                pClient->dwTimer = SYS_TMR_TickCountGet();
                dhcpTmo = newDhcpTmo;
                pClient->smState = SM_DHCP_GET_REQUEST_ACK;
                break;

            case SM_DHCP_GET_REQUEST_ACK:
                // Check to see if a packet has arrived
                if(TCPIP_UDP_GetIsReady(pClient->hDHCPSocket) < 250u)
                {
                    // Go back and transmit a new discovery if we didn't get an ACK after 2 seconds
                    if(SYS_TMR_TickCountGet() - pClient->dwTimer >= (dhcpTmo * SYS_TMR_TickPerSecond()))
                    {
                        _DHCPSetRunFail(pClient, SM_DHCP_SEND_DISCOVERY);
                    }
                    break;
                }

                // Check to see if we received an offer
                switch(_DHCPReceive(pClient, pNetIf))
                {
                    case DHCP_ACK_MESSAGE:


                        if(pClient->validValues.IPAddress && pClient->validValues.Mask)
                        {   // having a new address without a valid mask seems weird
                            // or at least we can try to generate it...   
                            IPV4_ADDR oldNetIp;
                            IPV4_ADDR oldNetMask;

                            TCPIP_UDP_Close(pClient->hDHCPSocket);
                            pClient->hDHCPSocket = INVALID_UDP_SOCKET;
                            pClient->dwTimer = SYS_TMR_TickCountGet();
                            pClient->smState = SM_DHCP_BOUND;
                            pClient->flags.bIsBound = true;	

                            oldNetIp.Val = TCPIP_STACK_NetAddressGet(pNetIf);
                            oldNetMask.Val = TCPIP_STACK_NetMaskGet(pNetIf);

                            _TCPIPStackSetConfigAddress(pNetIf, &pClient->tempIPAddress, &pClient->tempMask, false);
                            if(pClient->validValues.Gateway)
                            {
                                TCPIP_STACK_GatewayAddressSet(pNetIf, &pClient->tempGateway);
                            }
#if defined(TCPIP_STACK_USE_DNS)
                            if(pNetIf->Flags.bIsDNSServerAuto != 0)
                            {
                                if(pClient->validValues.DNS)
                                {
                                    TCPIP_STACK_PrimaryDNSAddressSet(pNetIf, &pClient->tempDNS);
                                }
                                if(pClient->validValues.DNS2)
                                {
                                    TCPIP_STACK_SecondaryDNSAddressSet(pNetIf, &pClient->tempDNS2);
                                }
                                else
                                {
                                    IPV4_ADDR zeroAdd = {0};
                                    TCPIP_STACK_SecondaryDNSAddressSet(pNetIf, &zeroAdd);
                                }
                            }
#endif
                            //if(pClient->validValues.HostName)
                            //	TCPIP_STACK_NetBiosNameSet(pNetIf, pClient->tempHostName);
                            _DHCPNotifyClients(pNetIf, DHCP_EVENT_BOUND);
                            if((pClient->tempIPAddress.Val & pClient->tempMask.Val) != (oldNetIp.Val & oldNetMask.Val))
                            {   // changed networks
                                TCPIP_ARP_EntryRemoveNet(pNetIf, &oldNetIp, &oldNetMask, ARP_ENTRY_TYPE_ANY);
                            } 
                        }
                        else
                        {
                            _DHCPSetRunFail(pClient, SM_DHCP_SEND_DISCOVERY);
                        }


                        break;

                    case DHCP_NAK_MESSAGE:
                        _DHCPSetRunFail(pClient, SM_DHCP_SEND_DISCOVERY);
                        break;
                }
                break;

            case SM_DHCP_BOUND:
                if(SYS_TMR_TickCountGet() - pClient->dwTimer < SYS_TMR_TickPerSecond())
                    break;

                // Check to see if our lease is still valid, if so, decrement lease 
                // time
                if(pClient->dwLeaseTime > (pClient->dwLeaseTime>>1))
                {
                    pClient->dwTimer += SYS_TMR_TickPerSecond();
                    pClient->dwLeaseTime--;
                    break;
                }

                _DHCPNotifyClients(pNetIf, DHCP_EVENT_LEASE_EXPIRED);
                // Open a socket to send and receive DHCP messages on
                pClient->hDHCPSocket = UDPOpenClientSkt(IP_ADDRESS_TYPE_IPV4, dhcpServerPort, 0, UDP_OPEN_CLIENT | UDP_OPEN_CONFIG_SERVICE);
                
                if(pClient->hDHCPSocket == INVALID_UDP_SOCKET)
                {
                    break;
                }
                // allow sending broadcast messages
                // and bind to the DHCP local port
                if(!TCPIP_UDP_Bind(pClient->hDHCPSocket, IP_ADDRESS_TYPE_IPV4, dhcpClientPort,  (IP_MULTI_ADDRESS*)&bcastAdd))
                {
                    TCPIP_UDP_Close(pClient->hDHCPSocket);
                    pClient->hDHCPSocket = INVALID_UDP_SOCKET;
                    {
                        break;
                    }
                }
                TCPIP_UDP_SocketNetSet(pClient->hDHCPSocket, pNetIf);            
                pClient->smState = SM_DHCP_SEND_RENEW;
                // No break

            case SM_DHCP_SEND_RENEW:
            case SM_DHCP_SEND_RENEW2:
            case SM_DHCP_SEND_RENEW3:
                if(TCPIP_UDP_TxPutIsReady(pClient->hDHCPSocket, 300) < 300u)
                    break;

                // Send the DHCP request message
                _DHCPSend(pClient, pNetIf, DHCP_REQUEST_MESSAGE, true);
                pClient->flags.bOfferReceived = false;

                // Start a timer and begin looking for a response
                pClient->dwTimer = SYS_TMR_TickCountGet();
                dhcpTmo = newDhcpTmo;
                pClient->smState++;
                break;

            case SM_DHCP_GET_RENEW_ACK:
            case SM_DHCP_GET_RENEW_ACK2:
            case SM_DHCP_GET_RENEW_ACK3:
                // Check to see if a packet has arrived
                if(TCPIP_UDP_GetIsReady(pClient->hDHCPSocket) < 250u)
                {
                    // Go back and transmit a new discovery if we didn't get an ACK after 2 seconds
                    if(SYS_TMR_TickCountGet() - pClient->dwTimer >=  (dhcpTmo * SYS_TMR_TickPerSecond()))
                    {
                        if(++pClient->smState > SM_DHCP_GET_RENEW_ACK3)
                        {
                            _DHCPSetRunFail(pClient, SM_DHCP_SEND_DISCOVERY);
                        }
                    }
                    break;
                }

                // Check to see if we received an offer
                switch(_DHCPReceive(pClient, pNetIf))
                {
                    case DHCP_ACK_MESSAGE:
                        TCPIP_UDP_Close(pClient->hDHCPSocket);
                        pClient->hDHCPSocket = INVALID_UDP_SOCKET;
                        pClient->dwTimer = SYS_TMR_TickCountGet();

                        pClient->smState = SM_DHCP_BOUND;
                        _DHCPNotifyClients(pNetIf, DHCP_EVENT_BOUND);
                        break;

                    case DHCP_NAK_MESSAGE:
                        _DHCPSetRunFail(pClient, SM_DHCP_SEND_DISCOVERY);
                        break;
                }
                break;
        }
    }
}



/*****************************************************************************
Function:
  void _DHCPReceive(DHCP_CLIENT_VARS* pClient, TCPIP_NET_IF* pNetIf)

Description:
  Receives and parses a DHCP message.

Precondition:
  A DHCP message is waiting in the UDP buffer.

Parameters:
  pClient - client descriptor
  pNetIf - interface to use

Returns:
  One of the DCHP_TYPE* contants.
***************************************************************************/
static uint8_t _DHCPReceive(DHCP_CLIENT_VARS* pClient, TCPIP_NET_IF* pNetIf)
{
	/*********************************************************************
	DHCP PACKET FORMAT AS PER RFC 1541

	0                   1                   2                   3
	0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|     op (1)    |   htype (1)   |   hlen (1)    |   hops (1)    |
	+---------------+---------------+---------------+---------------+
	|                            xid (4)                            |
	+-------------------------------+-------------------------------+
	|           secs (2)            |           flags (2)           |
	+-------------------------------+-------------------------------+
	|                          ciaddr  (4)                          |
	+---------------------------------------------------------------+
	|                          yiaddr  (4)                          |
	+---------------------------------------------------------------+
	|                          siaddr  (4)                          |
	+---------------------------------------------------------------+
	|                          giaddr  (4)                          |
	+---------------------------------------------------------------+
	|                                                               |
	|                          chaddr  (16)                         |
	|                                                               |
	|                                                               |
	+---------------------------------------------------------------+
	|                                                               |
	|                          sname   (64)                         |
	+---------------------------------------------------------------+
	|                                                               |
	|                          file    (128)                        |
	+---------------------------------------------------------------+
	|                                                               |
	|                          options (312)                        |
	+---------------------------------------------------------------+

	********************************************************************/
	uint8_t v;
	uint8_t len;
	uint8_t type;
	bool lbDone;
	TCPIP_UINT32_VAL tempServerID;
    TCPIP_UINT32_VAL transactionID;
    TCPIP_UINT32_VAL dwLeaseTime;
    UDP_SOCKET      s;
    bool    dhcpInvalid;
    uint8_t recvBuffer[20]; // temporary receive buffer

	// Assume unknown message until proven otherwise.
	type = DHCP_UNKNOWN_MESSAGE;
    dhcpInvalid = false;

    s = pClient->hDHCPSocket;
	TCPIP_UDP_Get(s, &v);                             // op

	// Make sure this is BOOT_REPLY.
	if ( v != BOOT_REPLY )
	{
        TCPIP_UDP_Discard(s);
        return DHCP_UNKNOWN_MESSAGE;    // type
    }

    // check the transaction ID
    TCPIP_UDP_RxOffsetSet(s, 4);
    TCPIP_UDP_ArrayGet(s, transactionID.v, sizeof(transactionID.v));
    if(transactionID.Val != pClient->transactionID.Val)
    {   // not what we're expecting
        dhcpInvalid = true;
    }
    else
    {
        // Jump to chaddr field (Client Hardware Address -- our MAC address for 
        // Ethernet and WiFi networks) and verify that this message is directed 
        // to us before doing any other processing.
        TCPIP_UDP_RxOffsetSet(s, 28);		// chaddr field is at offset 28 in the UDP packet payload -- see DHCP packet format above
        TCPIP_UDP_ArrayGet(s, recvBuffer, sizeof(pNetIf->netMACAddr));  // get MAC address field
        if(memcmp(recvBuffer, pNetIf->netMACAddr.v, sizeof(pNetIf->netMACAddr)) != 0)
        {
            dhcpInvalid = true;
        }
        else
        {
            // Check to see if this is the first offer.  If it is, record its 
            // yiaddr value ("Your (client) IP address") so that we can REQUEST to 
            // use it later.
            if(!pClient->flags.bOfferReceived)
            {
                TCPIP_UDP_RxOffsetSet(s, 16);
                TCPIP_UDP_ArrayGet(s, (uint8_t*)&pClient->tempIPAddress, sizeof(pClient->tempIPAddress));
                pClient->validValues.IPAddress = 1;
            }

            // Jump to DHCP options (ignore htype, hlen, hops, xid, secs, flags, 
            // ciaddr, siaddr, giaddr, padding part of chaddr, sname, file, magic 
            // cookie fields)
            TCPIP_UDP_RxOffsetSet(s, 240);
        }
    }

    lbDone = false;
    while(dhcpInvalid == false && lbDone == false)
    {
        // Get the Option number
        // Break out eventually in case if this is a malformed 
        // DHCP message, ie: missing DHCP_END_OPTION marker
        if(!TCPIP_UDP_Get(s, &v))
        {
            lbDone = true;
            break;
        }

        switch(v)
        {
            case DHCP_MESSAGE_TYPE:
                TCPIP_UDP_ArrayGet(s, recvBuffer, 2);      // get len and type
                if(recvBuffer[0] != 1)
                {   // Len must be 1
                    dhcpInvalid = true;
                }
                else
                {
                    type = recvBuffer[1];
                    // Throw away the packet if we know we don't need it (ie: another offer when we already have one)
                    if(pClient->flags.bOfferReceived && (type == DHCP_OFFER_MESSAGE))
                    {
                        dhcpInvalid = true;
                    }
                }

                break;

            case DHCP_SUBNET_MASK:
                // get length and the mask itself (size == 4)
                TCPIP_UDP_ArrayGet(s, recvBuffer, 1 + sizeof(pClient->tempMask));
                if(recvBuffer[0] != sizeof(pClient->tempMask))
                {
                    dhcpInvalid = true;
                }
                else if(pClient->flags.bOfferReceived == 0)
                {   // the offered IP mask is needed
                    memcpy(&pClient->tempMask, recvBuffer + 1, sizeof(pClient->tempMask)); 
                    pClient->validValues.Mask = 1;
                }
                // else ignore

                break;

            case DHCP_ROUTER:
                // get length and the router IP address (size == 4)
                TCPIP_UDP_ArrayGet(s, recvBuffer, 1 + sizeof(pClient->tempGateway));
                len = recvBuffer[0];
                if(len < sizeof(pClient->tempGateway))
                {
                    dhcpInvalid = true;
                }
                else
                {
                    if(pClient->flags.bOfferReceived == 0)
                    {   // take 1st offer
                        memcpy(&pClient->tempGateway, recvBuffer + 1, sizeof(pClient->tempGateway)); 
                        pClient->validValues.Gateway = 1;
                    }
                }

                break;

#if defined(TCPIP_STACK_USE_DNS)
            case DHCP_DNS:
                // get length and the DNS IP address (size == 4)
                TCPIP_UDP_ArrayGet(s, recvBuffer, 1 + sizeof(pClient->tempDNS));
                len = recvBuffer[0];
                if(len < sizeof(pClient->tempDNS))
                {
                    dhcpInvalid = true;
                }
                else
                {
                    len -= 4;
                    // Check to see if this is the first offer
                    if(pClient->flags.bOfferReceived == 0)
                    {
                        memcpy(&pClient->tempDNS, recvBuffer + 1, sizeof(pClient->tempDNS)); 
                        pClient->validValues.DNS = 1;
                        if(len >= 4)
                        {   // have a 2nd DNS
                            TCPIP_UDP_ArrayGet(s, (uint8_t*)&pClient->tempDNS2, sizeof(pClient->tempDNS2));
                            pClient->validValues.DNS2 = 1;
                            len -= 4;
                        }
                    }

                }

                break;
#endif

            case DHCP_SERVER_IDENTIFIER:
                // get length and the server ID (size == 4)
                TCPIP_UDP_ArrayGet(s, recvBuffer, 1 + sizeof(tempServerID.Val));
                if(recvBuffer[0] != sizeof(tempServerID.Val))
                {
                    dhcpInvalid = true;
                }
                else
                {   // update the ID
                    tempServerID.v[3] = recvBuffer[1]; 
                    tempServerID.v[2] = recvBuffer[2];
                    tempServerID.v[1] = recvBuffer[3];
                    tempServerID.v[0] = recvBuffer[4];
                }
 
                break;

            case DHCP_END_OPTION:
                lbDone = true;
                break;

            case DHCP_IP_LEASE_TIME:
                // get length and the lease time (size == 4)
                TCPIP_UDP_ArrayGet(s, recvBuffer, 1 + sizeof(pClient->dwLeaseTime));
                if(recvBuffer[0] != sizeof(pClient->dwLeaseTime))
                {
                    dhcpInvalid = true;
                }
                else if(pClient->flags.bOfferReceived == 0)
                {
                    dwLeaseTime.v[3] = recvBuffer[1];
                    dwLeaseTime.v[2] = recvBuffer[2];
                    dwLeaseTime.v[1] = recvBuffer[3];
                    dwLeaseTime.v[0] = recvBuffer[4];

                    // In case if our clock is not as accurate as the remote 
                    // DHCP server's clock, let's treat the lease time as only 
                    // 96.875% of the value given
                    pClient->dwLeaseTime = dwLeaseTime.Val - (dwLeaseTime.Val >> 5);
                }

               break;

            default:
                // Ignore all unsupport tags.
                TCPIP_UDP_Get(s, &len);                // Get option len
                TCPIP_UDP_ArrayGet(s,NULL,len);
                break;
        }
    }

    if(dhcpInvalid == false)
    {
        // If this is an OFFER message, remember current server id.
        if ( type == DHCP_OFFER_MESSAGE )
        {
            pClient->dwServerID = tempServerID.Val;
            pClient->flags.bOfferReceived = true;
        }
        else if(type != DHCP_UNKNOWN_MESSAGE)
        {
            // For other types of messages, make sure that received
            // server id matches with our previous one.
            if ( pClient->dwServerID != tempServerID.Val )
            {
                type = DHCP_UNKNOWN_MESSAGE;
            }
        }
    }
    else
    {
        type = DHCP_UNKNOWN_MESSAGE;
    }

	TCPIP_UDP_Discard(s);                             // We are done with this packet
	return type;

}

/*****************************************************************************
  Function:
        static void _DHCPSend(DHCP_CLIENT_VARS* pClient, TCPIP_NET_IF* pNetIf, uint8_t messageType, bool bRenewing)

  Description:
        Sends a DHCP message.

  Precondition:
        UDP is ready to write a DHCP packet.

  Parameters:
    pClient  - client descriptor
    pNetIf   - interface to use
        messageType - One of the DHCP_TYPE constants
        bRenewing - Whether or not this is a renewal request

  Returns:
        None
 ***************************************************************************/

static void _DHCPSend(DHCP_CLIENT_VARS* pClient, TCPIP_NET_IF* pNetIf, uint8_t messageType, bool bRenewing)
{
    UDP_SOCKET s;
    bool newTransaction;
    IPV4_ADDR zeroIP = {0};
    uint8_t dhcpFields[208];


    memset(dhcpFields, 0, sizeof(dhcpFields));
    *(uint16_t*)(&dhcpFields[202]) = 0x8263; // Load magic cookie as per RFC 1533.
    *(uint16_t*)(&dhcpFields[204]) = 0x6353; // Unaligned so has to be set like this
    // Load message type.
    *(uint16_t*)(&dhcpFields[206]) = DHCP_MESSAGE_TYPE | DHCP_MESSAGE_TYPE_LEN<<8;



    s = pClient->hDHCPSocket;

    newTransaction = (messageType == DHCP_DISCOVER_MESSAGE || (pClient->dhcpOp == DHCP_OPER_INIT_REBOOT && messageType == DHCP_REQUEST_MESSAGE));
    if (newTransaction)
    {
        pClient->transactionID.Val = SYS_RANDOM_PseudoGet(); // generate a new transaction ID
    }

    {
        uint8_t dhcpHeader [12];
        memset(dhcpHeader, 0, sizeof(dhcpHeader));
        dhcpHeader[0] = BOOT_REQUEST;
        dhcpHeader[1] = BOOT_HW_TYPE;
        dhcpHeader[2] = BOOT_LEN_OF_HW_TYPE;
        *(uint32_t*)(&dhcpHeader[4]) =  pClient->transactionID.Val;
        TCPIP_UDP_ArrayPut(s, dhcpHeader, sizeof (dhcpHeader));
    }

    // If this is DHCP REQUEST message, use previously allocated IP address.
    if (pClient->dhcpOp != DHCP_OPER_INIT_REBOOT && ((messageType == DHCP_REQUEST_MESSAGE) && bRenewing))
    {
        TCPIP_UDP_ArrayPut(s, (uint8_t*) & pClient->tempIPAddress, sizeof (pClient->tempIPAddress));
    }
    else
    {
        TCPIP_UDP_ArrayPut(s, dhcpFields, 4);
    }

    // Set yiaddr, siaddr, giaddr as zeros,
    TCPIP_UDP_ArrayPut(s, dhcpFields, 12);

    // Load chaddr - Client hardware address.
    TCPIP_UDP_ArrayPut(s, pNetIf->netMACAddr.v, sizeof (pNetIf->netMACAddr));

    // Set chaddr[6..15], sname and file as zeros.

    TCPIP_UDP_ArrayPut(s, dhcpFields, sizeof (dhcpFields));
    TCPIP_UDP_Put(s, messageType);

    if (newTransaction)
    {
        // Reset offered flag so we know to act upon the next valid offer
        pClient->flags.bOfferReceived = false;
    }

    if (pClient->dhcpOp != DHCP_OPER_INIT_REBOOT)
    { // can use a server ID
        if ((messageType == DHCP_REQUEST_MESSAGE) && !bRenewing)
        {
            // DHCP REQUEST message must include server identifier the first time
            // to identify the server we are talking to.
            // _DHCPReceive() would populate "serverID" when it
            // receives DHCP OFFER message. We will simply use that
            // when we are replying to server.
            // If this is a renwal request, we must not include server id.
            uint8_t dhcpServerId [] = {
                DHCP_SERVER_IDENTIFIER, DHCP_SERVER_IDENTIFIER_LEN,
                ((uint8_t*) (&pClient->dwServerID))[3],
                ((uint8_t*) (&pClient->dwServerID))[2],
                ((uint8_t*) (&pClient->dwServerID))[1],
                ((uint8_t*) (&pClient->dwServerID))[0]
            };
            TCPIP_UDP_ArrayPut(s, dhcpServerId, sizeof (dhcpServerId));
        }
    }

    // Load our interested parameters
    // This is hardcoded list.  If any new parameters are desired,
    // new lines must be added here.
    {
        uint8_t dhcpParams[] = {
            DHCP_PARAM_REQUEST_LIST,
            DHCP_PARAM_REQUEST_LIST_LEN,
            DHCP_SUBNET_MASK,
            DHCP_ROUTER,
            DHCP_DNS,
            DHCP_HOST_NAME,
        };
        TCPIP_UDP_ArrayPut(s, dhcpParams, sizeof (dhcpParams));
    }

    // Add requested IP address to DHCP Request Message
    if (((messageType == DHCP_REQUEST_MESSAGE) && pClient->dhcpOp == DHCP_OPER_INIT_REBOOT) ||
            ((messageType == DHCP_REQUEST_MESSAGE) && !bRenewing) ||
            ((messageType == DHCP_DISCOVER_MESSAGE) && pClient->tempIPAddress.Val))
    {
        uint8_t requestIp [] = {
            DHCP_PARAM_REQUEST_IP_ADDRESS, DHCP_PARAM_REQUEST_IP_ADDRESS_LEN
        };
        TCPIP_UDP_ArrayPut(s, requestIp, sizeof (requestIp));
        TCPIP_UDP_ArrayPut(s, (uint8_t*) & pClient->tempIPAddress, DHCP_PARAM_REQUEST_IP_ADDRESS_LEN);
    }

    // Add any new paramter request here.

    // End of Options.
    TCPIP_UDP_Put(s, DHCP_END_OPTION);

    // Add zero padding to ensure compatibility with old BOOTP relays that discard small packets (<300 UDP octets)
    TCPIP_UDP_ArrayPut(s, dhcpFields, 300 - TCPIP_UDP_TxCountGet(s));

    // Make sure we advertise a 0.0.0.0 IP address so all DHCP servers will respond.  If we have a static IP outside the DHCP server's scope, it may simply ignore discover messages.
    if (pClient->dhcpOp != DHCP_OPER_INIT_REBOOT && !bRenewing)
    {
        TCPIP_UDP_SourceIPAddressSet(s, IP_ADDRESS_TYPE_IPV4, (IP_MULTI_ADDRESS*) & zeroIP);
    }
    TCPIP_UDP_Flush(s);
}

void TCPIP_DHCP_ConnectionHandler(TCPIP_NET_IF* pNetIf, TCPIP_MAC_EVENT connEvent)
{
    DHCP_CLIENT_VARS* pClient = DHCPClients + TCPIP_STACK_NetIxGet(pNetIf);

    if (pClient->flags.bDHCPEnabled != 0)
    {
        if (connEvent & TCPIP_MAC_EV_CONN_LOST)
        {
            // let it wait for the connection
            _DHCPClose(pNetIf, false);
            _TCPIPStackSetConfigAddress(pNetIf, 0, 0, true);
            TCPIP_STACK_AddressServiceEvent(pNetIf, TCPIP_STACK_ADDRESS_SERVICE_DHCPC, TCPIP_STACK_ADDRESS_SERVICE_EVENT_CONN_LOST);
            _DHCPNotifyClients(pNetIf, DHCP_EVENT_CONN_LOST);
        }
        else if (connEvent & TCPIP_MAC_EV_CONN_ESTABLISHED)
        {
            // put it in wait connection mode
            // should be in this state anyway
            // but just in case we've missed the link down event
            _DHCPEnable(pNetIf, true, pClient->flags.bIfUp ? DHCP_OPER_INIT_REBOOT : DHCP_OPER_INIT);
            pClient->flags.bIfUp = 1;
        }
    }

}

// Register an DHCP event handler
// Use hNet == 0 to register on all interfaces available
// Returns a valid handle if the call succeeds,
// or a null handle if the call failed.
// Function has to be called after the DHCP is initialized
// The hParam is passed by the client and will be used by the DHCP when the notification is made.
// It is used for per-thread content or if more modules, for example, share the same handler
// and need a way to differentiate the callback.
TCPIP_DHCP_HANDLE TCPIP_DHCP_HandlerRegister(TCPIP_NET_HANDLE hNet, TCPIP_DHCP_EVENT_HANDLER handler, const void* hParam)
{
    if(dhcpMemH)
    {
        DHCP_LIST_NODE* newNode = (DHCP_LIST_NODE*)TCPIP_Notification_Add(&dhcpRegisteredUsers, dhcpMemH, sizeof(*newNode));
        if(newNode)
        {
            newNode->handler = handler;
            newNode->hParam = hParam;
            newNode->hNet = hNet;
            return newNode;
        }
    }

    return 0;
}

// deregister the event handler
bool TCPIP_DHCP_HandlerDeRegister(TCPIP_DHCP_HANDLE hDhcp)
{
    if(hDhcp && dhcpMemH)
    {
        if(TCPIP_Notification_Remove((SGL_LIST_NODE*)hDhcp, &dhcpRegisteredUsers, dhcpMemH))
        {
            return true;
        }
    }

    return false;
}

static void _DHCPNotifyClients(TCPIP_NET_IF* pNetIf, TCPIP_DHCP_EVENT_TYPE evType)
{
    DHCP_LIST_NODE* dNode;

    for(dNode = (DHCP_LIST_NODE*)dhcpRegisteredUsers.list.head; dNode != 0; dNode = dNode->next)
    {
        if(dNode->hNet == 0 || dNode->hNet == pNetIf)
        {   // trigger event
            (*dNode->handler)(pNetIf, evType, dNode->hParam);
        }
    }
    
}


// enable unicast DHCP packets
// this should be an IPv4 packet
static bool _DHCPPacketFilter(TCPIP_MAC_PACKET* pRxPkt, const void* param)
{
    TCPIP_MAC_ETHERNET_HEADER* macHdr = (TCPIP_MAC_ETHERNET_HEADER*)pRxPkt->pMacLayer;
    const uint8_t* netMacAddr = TCPIP_STACK_NetMACAddressGet((TCPIP_NET_IF*)pRxPkt->pktIf);

    if(netMacAddr)
    {
        if(memcmp(netMacAddr, macHdr->DestMACAddr.v, sizeof(macHdr->DestMACAddr)) == 0)
        {   // unicast to me
            IPV4_HEADER* pHeader = (IPV4_HEADER*)pRxPkt->pNetLayer;
            if(pHeader->Protocol == IP_PROT_UDP)
            {   // UDP packet
                UDP_HEADER* pUDPHdr = (UDP_HEADER*)pRxPkt->pTransportLayer;
                UDP_PORT destPort = TCPIP_Helper_ntohs(pUDPHdr->DestinationPort);
                UDP_PORT srcPort = TCPIP_Helper_ntohs(pUDPHdr->SourcePort);
                if(destPort == dhcpClientPort && srcPort == dhcpServerPort)
                {   // valid DHCP packet
                    return true;
                }
            }
        }
    }

    return false;
}


#endif	//#if defined(TCPIP_STACK_USE_DHCP_CLIENT)


