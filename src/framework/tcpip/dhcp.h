/*******************************************************************************
  Dynamic Host Configuration Protocol (DHCP) Client API Header File

  Company:
    Microchip Technology Inc.

  File Name:
    dhcp.h

  Summary:
    Dynamic Host Configuration Protocol (DHCP) Client API

  Description:
     Provides automatic IP address, subnet mask, gateway address,
     DNS server address, and other configuration parameters on DHCP
     enabled networks.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef __DHCP_H
#define __DHCP_H

// *****************************************************************************
// *****************************************************************************
// Section: Data Types and Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Enumeration: TCPIP_DHCP_EVENT_TYPE

  Summary:
    DHCP Event Type

  Description:
    None
 */
typedef enum
{
    DHCP_EVENT_DISCOVER = 1,     // DHCP cycle started
    DHCP_EVENT_BOUND,            // DHCP lease obtained
    DHCP_EVENT_LEASE_EXPIRED,    // lease expired
    DHCP_EVENT_CONN_LOST,        // connection to the DHCP server lost, reverted to the default IP address
    DHCP_EVENT_SERVICE_DISABLED  // DHCP service disabled, reverted to the default IP address

} TCPIP_DHCP_EVENT_TYPE;


// *****************************************************************************
/*
  Type:
    TCPIP_DHCP_EVENT_HANDLER

  Summary:
    DHCP Event Handler

  Description:
    Prototype of a DHCP event handler. Clients can register a handler with the
    DHCP service. Once an DHCP event occurs the DHCP service will called the
    registered handler.
    The handler has to be short and fast. It is meant for
    setting an event flag, not for lengthy processing!
 */

typedef void    (*TCPIP_DHCP_EVENT_HANDLER)(TCPIP_NET_HANDLE hNet, TCPIP_DHCP_EVENT_TYPE evType, const void* param);


// *****************************************************************************
/*
  Type:
    TCPIP_DHCP_HANDLE

  Summary:
    DHCP Handle

  Description:
    A handle that a client can use after the event handler has been registered.
 */

typedef const void* TCPIP_DHCP_HANDLE;


// *****************************************************************************
/* DHCP Module Configuration

  Summary:
    DHCP Module Configuration run-time parameters

  Description:
    This structure contains the data that's passed to the DHCP module
    at the TCP/IP stack initialization.

 */

typedef struct
{
    bool    dhcpEnable;     // DHCP client enable at stack start-up 
    int     dhcpTmo;        // timeout to wait for a DHCP request, seconds
    int     dhcpCliPort;    // client port for DHCP client transactions
    int     dhcpSrvPort;    // remote server port for DHCP server messages

} DHCP_MODULE_CONFIG;


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - General
// *****************************************************************************
// *****************************************************************************

/*****************************************************************************
  Function:
    void TCPIP_DHCP_Enable(TCPIP_NET_HANDLE hNet)

  Summary:
    Enables the DHCP client for the specified interface.

  Description:
    Enables the DHCP client for the specified interface, if it is disabled.
    If it is already enabled, no action is taken.

  Precondition:
    DHCP module initialized.

  Parameters:
     hNet - Interface to enable the DHCP client on.

  Returns:
    - true	- if successful
    - false	- if unsuccessful
 */
bool TCPIP_DHCP_Enable(TCPIP_NET_HANDLE hNet);


/*****************************************************************************
  Function:
    bool TCPIP_DHCP_Disable(TCPIP_NET_HANDLE hNet)

  Summary:
    Disables the DHCP Client for the specified interface.

  Description:
    Disables the DHCP client for the specified interface.
    If it is already disabled, no action is taken.

  Precondition:
    DHCP module initialized.

  Parameters:
    pNetIf - Interface to disable the DHCP client on.

  Returns:
    - true	- if successful
    - false	- if unsuccessful

  Remarks:
    When the DHCP client is disabled and the interface continues using its old configuration,
    it is possible that the lease may expire and the DHCP server provide the IP address
    to another client.
    The application should not keep the old lease unless it is sure 
    that there is no danger of conflict.
 */
bool TCPIP_DHCP_Disable(TCPIP_NET_HANDLE hNet );


/*****************************************************************************
  Function:
    void TCPIP_DHCP_Renew(TCPIP_NET_HANDLE hNet)

  Summary:
    Renews the DHCP lease for the specified interface.

  Description:
    Tries to contact the server and renew the DHCP lease for the specified interface.
    The interface should have the DHCP enabled and in bound state
    for this call to succeed.

  Precondition:
    DHCP module initialized.
    DHCP enabled, a valid lease.

  Parameters:
     hNet - Interface to renew the DHCP lease on.

  Returns:
    - true	- if successful
    - false - if unsuccessful
 */
bool TCPIP_DHCP_Renew(TCPIP_NET_HANDLE hNet);


/*****************************************************************************
  Function:
    void TCPIP_DHCP_Request(TCPIP_NET_HANDLE hNet, IPV4_ADDR reqAddress)

  Summary:
    Requests the supplied IPv4 address from a DHCP server.

  Description:
    If the DHCP client is not enabled on that interface, this call will first try to enable it.
    If this succeeds or the DHCP client was already enabled, the following steps are taken: 
    The DHCP client probes the DHCP server and requests the supplied IPv4 address
    as a valid lease for the specified interface.
    If the server acknowledges the request, then this is the new IPv4 address
    of the interface.
    If the DHCP server rejects the request, then the whole DHCP
    process is resumed starting with the DHCP Discovery phase.

  Precondition:
    DHCP module initialized.
    DHCP enabled, a valid lease

  Parameters:
     hNet - Interface to renew the DHCP lease on.

  Returns:
    - true	- if successful
    - false - if the supplied IP address is invalid or the DHCP client
			  is in the middle of a transaction


 Remarks:
    The requested IPv4 address should be a previous lease that was granted to the host.
    This call should be used when the host is restarting.

 */
bool TCPIP_DHCP_Request(TCPIP_NET_HANDLE hNet, IPV4_ADDR reqAddress);


/*****************************************************************************
  Function:
    bool TCPIP_DHCP_IsEnabled(CPIP_NET_HANDLE hNet)

  Summary:
    Determines if the DHCP client is enabled on the specified interface.

  Description:
    Returns the current state of the DHCP client on the specified interface.

  Precondition:
    DHCP module initialized.

  Parameters:
    hNet- Interface to query.

  Returns:
    - true	- if the DHCP client service is enabled on the specified interface
    - false	- if the DHCP client service is not enabled on the specified interface
 */
bool TCPIP_DHCP_IsEnabled(TCPIP_NET_HANDLE hNet);


/*****************************************************************************
  Function:
    bool TCPIP_DHCP_IsBound(TCPIP_NET_HANDLE hNet)

  Summary:
    Determines if the DHCP client has an IP address lease on the specified
    interface.

  Description:
    Returns the status of the current IP address lease on the specified
    interface.

  Precondition:
    None

  Parameters:
    hNet - Interface to query

  Returns:
    - true  - DHCP client has obtained an IP address lease (and likely other
              parameters) and these values are currently being used
    - false - No IP address is currently leased
 */
bool TCPIP_DHCP_IsBound(TCPIP_NET_HANDLE hNet);


/*****************************************************************************
  Function:
    bool TCPIP_DHCP_IsServerDetected(TCPIP_NET_HANDLE hNet)

  Summary:
    Query if the DHCP client on the specified interface has been able
    to contact a DHCP server.

  Description:
    Determines if the DHCP client on the specified interface received any reply
    from a DHCP server.

  Precondition:
    DHCP module initialized.

  Parameters:
    hNet- Interface to query.

  Returns:
    - true  - At least one DHCP server is attached to the specified network
              interface
    - false - No DHCP servers are currently detected on the specified network
              interface
 */
 bool TCPIP_DHCP_IsServerDetected(TCPIP_NET_HANDLE hNet);


// *****************************************************************************
/* Function:
    TCPIP_DHCP_RequestTimeoutSet(TCPIP_NET_HANDLE hNet, int tmo)

  Summary:
    This function sets the DHCP client request timeout

  Description:
    This function allows the run time adjustment of the DHCP timeout.
    This value specifies for how long the client has to wait for
    a DHCP server reply.

  Precondition:
    DHCP module initialized.

  Parameters:
    hNet - Interface handle.
    tmo  - Timeout to wait, in seconds

  Returns:
    - true  - if successful
    - false - if a wrong interface handle or time-out value was provided 
    
 */

bool TCPIP_DHCP_RequestTimeoutSet(TCPIP_NET_HANDLE hNet, int tmo);


// *****************************************************************************
/* Function:
    TCPIP_DHCP_HandlerRegister(TCPIP_NET_HANDLE hNet, TCPIP_DHCP_EVENT_HANDLER handler, const void* hParam)

  Summary:
    Registers a DHCP Handler.

  Description:
    This function registers a DHCP event handler.
    The DHCP module will call the registered handler when a
    DHCP event (TCPIP_DHCP_EVENT_TYPE) occurs.

  Precondition:
    DHCP module initialized.

  Parameters:
    hNet    - Interface handle.
              Use hNet == 0 to register on all interfaces available.
    handler - Handler to be called when a DHCP event occurs.
    hParam  - Parameter to be used in the handler call.
              This is user supplied and is not used by the DHCP module.


  Returns:
    Returns a valid handle if the call succeeds, or a null handle if
    the call failed (out of memory, for example).

  Remarks:
    The handler has to be short and fast. It is meant for
    setting an event flag, not for lengthy processing!

    The hParam is passed by the client and will be used by the DHCP when the
    notification is made. It is used for per-thread content or if more modules,
    for example, share the same handler and need a way to differentiate the
    callback.
 */

TCPIP_DHCP_HANDLE      TCPIP_DHCP_HandlerRegister(TCPIP_NET_HANDLE hNet, TCPIP_DHCP_EVENT_HANDLER handler, const void* hParam);

// *****************************************************************************
/* Function:
    TCPIP_DHCP_HandlerDeRegister(TCPIP_DHCP_HANDLE hDhcp)

  Summary:
    Deregisters a previously registered DHCP handler.
    
  Description:
    Deregisters the DHCP event handler.

  Precondition:
    DHCP module initialized.

  Parameters:
    hDhcp   - A handle returned by a previous call to TCPIP_DHCP_HandlerRegister.

  Returns:
    - true	- if the call succeeds
    - false - if no such handler is registered
 */

bool             TCPIP_DHCP_HandlerDeRegister(TCPIP_DHCP_HANDLE hDhcp);


#endif  // __DHCP_H

/******************************************************************************
 End of File
 */
