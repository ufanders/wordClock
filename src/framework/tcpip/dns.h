/*******************************************************************************
  Domain Name System (DNS) Client API Header file
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    dns.h

  Summary:
    DNS definitions and interface file

  Description:
    This source file contains the DNS client module API
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

#ifndef __DNS_H
#define __DNS_H

// *****************************************************************************
// *****************************************************************************
// Section: Data Types and Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*  
  Enumeration:
	TCPIP_DNS_EVENT_TYPE

  Summary:
    This enumeration is used  to notify DNS client applications.

  Description:
    These events are used while notifying to the registered applications.
	
  Remarks:
    None.
*/
typedef enum
{
    TCPIP_DNS_EVENT_QUERY = 1,          // DNS Query sent
    TCPIP_DNS_EVENT_BOUND,              // DNS socket obtained
    TCPIP_DNS_EVENT_CACHE_EXPIRED,      // lease expired
    TCPIP_DNS_EVENT_CONN_LOST,          // connection to the DNS server lost
                                        // reverted to the defaultIP address
    TCPIP_DNS_EVENT_SERVICE_DISABLED,   // DNS service disabled
                                        // reverted to the defaultIP address
    TCPIP_DNS_EVENT_NAME_RESOLVED,      // DNS Name resolved
    TCPIP_DNS_EVENT_NO_ACTION,          // DNS no event generated
}TCPIP_DNS_EVENT_TYPE;

// *****************************************************************************
/* Type:
		TCPIP_DNS_EVENT_HANDLER

  Summary:
    Notification handler that can be called when a specific entry is resolved
	and entry is timed out.

  Description:
    The format of a notification handler registered with the DNS module.
	Once an DNS event occurs the DNS service will be called for the registered handler.
	The parameter member is used as DNS host name and the notification will be specific to the 
	that DNS host name. If the parameter is null, all the registered application will get the 
	notification.
  Remarks:
    If pNetIf == 0 then the notification is called for events on any interface.
*/
typedef void    (*TCPIP_DNS_EVENT_HANDLER)(TCPIP_NET_HANDLE hNet, TCPIP_DNS_EVENT_TYPE evType, const void* param);

// *****************************************************************************
/*
  Type:
    TCPIP_DNS_HANDLE

  Summary:
    DNS Client Handle

  Description:
    A handle that a application needs to use when deregistering a notification handler. 

  Remarks:
    This handle can be used by the application after the event handler has been registered.
*/
typedef const void* TCPIP_DNS_HANDLE;

// *****************************************************************************
/* 
  Enumeration:
	DNS_RESOLVE_TYPE

  Summary:
    DNS query record type.

  Description:
    This enumeration provides the RecordType argument for TCPIP_DNS_Resolve.
	The stack supports DNS_TYPE_A and DNS_TYPE_AAAA.
	
  Remarks:
    None.
*/
typedef enum
{

    DNS_TYPE_A      = 1,        // Indicates an A (standard address) record.
    DNS_TYPE_MX     = 15,       // Indicates an MX (mail exchanger) record.
    DNS_TYPE_AAAA   = 28u       // Indicates a quad-A (IPv6 address) address record.
}DNS_RESOLVE_TYPE;

// *****************************************************************************
/* Enumeration:
    TCPIP_DNS_RESULT

  Summary:
    DNS Client and Server Results (success and failure codes)

  Description:
    Various results for DNS Client and Server definitions.

  Remarks:
    None.
*/
typedef enum
{
    // success codes
    DNS_RES_OK                  = 0,    // operation succeeded
    DNS_RES_PENDING,                    // operation is ongoing


    // failure codes
    DNS_RES_NO_ENTRY            = -1,   // no such entry exists
    DNS_RES_CACHE_FULL          = -2,   // the cache is full and no entry could be
    DNS_RES_OPEN_TMO            = -3,   // DNS client couldn't get a socket
    DNS_RES_SERVER_TMO          = -4,   // DNS server response tmo
	DNS_RES_MEMORY_FAIL         = -5,   // DNS Allocation fail
    DNS_RES_DUPLICATE_ENTRY     = -6,   // DNS add duplicate entry failure
	DNS_RES_NO_SERVICE          = -7,   // DNS service not implemented

    // backward compatibility results
    DNS_RES_NO_INTERFACE        = -10,   // an active/requested interface could not be found
    DNS_RES_BUSY                = -11,   // module is in use by other task; retry later
    DNS_RES_ARP_TMO             = -12,   // ARP tmo
    DNS_RES_NO_IPADDRESS        = -13,   // HAsh entry has no IP value with a busy entry

}TCPIP_DNS_RESULT;


// *****************************************************************************
/* 
  Structure:
    DNS_CLIENT_MODULE_CONFIG

  Summary:
    Provides a place holder for DNS client configuration.

  Description:
    DNS module runtime configuration and initialization parameters. 

  Remarks:
    None.
*/
typedef struct
{
    bool    deleteOldLease;             // Delete old cache if still in place,
    // specific DNS parameters
    size_t  cacheEntries;               // Max number of cache entries 
    uint32_t     entrySolvedTmo;        // Solved entry removed after this tmo if not referenced - seconds
    size_t  IPv4EntriesPerDNSName;      // Number of IPv4 entries per DNS name and Default value is 1.
    IP_ADDRESS_TYPE dnsIpAddressType;   // This parameter can be used to choose for DNS client open. 
	                                    // At present it is default IPv4 address type
    size_t  IPv6EntriesPerDNSName;      // Number of IPv6 address per DNS Name
                                        // Default value is 1 and is used only when IPv6 is enabled

                                
}DNS_CLIENT_MODULE_CONFIG;


// *****************************************************************************
/* 
  Structure:
    TCPIP_DNS_RESOLVED_SERVERIPADDRESS

  Summary:
    DNS module resolved data for both IPv4 and IPv6 .

  Description:
    DNS module uses this structure to collect a IPv4 and IPv6 resolved 
	address.

  Remarks:
    None.
*/
typedef struct
{
    int                         nIPv4Entries;  // how many entries in the ip4Address[] array; if IPv4 is defined
    IPV4_ADDR                   ip4Address;    // provide a symbol for IPv4 entries number;
    int                         nIPv6Entries;  // how many entries in the ip6Address[] array; if IPv6 is defined
    IPV6_ADDR                   ip6Address;    // provide a symbol for IPv6 entries number
}TCPIP_DNS_RESOLVED_SERVERIPADDRESS;


// *****************************************************************************
// *****************************************************************************
// Section: DNS client Routines
// *****************************************************************************
// *****************************************************************************


//****************************************************************************
/*  Function:
    void TCPIP_DNS_Resolve(uint8_t* Hostname, DNS_RESOLVE_TYPE Type)

  Summary:
    Begins resolution of an address.

  Description:
    This function attempts to resolve a host name to an IP address.  When
    called, it starts the DNS state machine.  Call TCPIP_DNS_IsResolved repeatedly
    to determine if the resolution is complete.

    Only one DNS resolution may be executed at a time.  The host-name must
    not be modified in memory until the resolution is complete.

  Precondition:

  Parameters:
    Hostname - A pointer to the null terminated string specifying the
        host for which to resolve an IP.
    RecordType - DNS_TYPE_A or DNS_TYPE_MX depending on what type of
        record resolution is desired.

  Returns:
    DNS_RES_OK on success
    

  Remarks:
    This function requires access to one UDP socket.  If none are available,
    UDP_MAX_SOCKETS may need to be increased.

  */
TCPIP_DNS_RESULT  TCPIP_DNS_Resolve(const char* HostName, DNS_RESOLVE_TYPE Type);


//****************************************************************************
/*  
  Function:
    TCPIP_DNS_RESULT TCPIP_DNS_IsResolved(const char* HostName, void* HostIP)

  Summary:
    Determines if the DNS resolution is complete and provides the IP.

  Description:
    Call this function to determine if the DNS resolution of an address has
    been completed.  If so, the resolved address will be provided in HostIP.

  Precondition:
    TCPIP_DNS_Resolve has been called.

  Parameters:
    Hostname - A pointer to the null terminated string specifying the
        host for which to resolve an IP.
  	HostIP - A pointer to an IPV4_ADDR structure in which to store the
		 resolved IP address once resolution is complete.

  Return Values:
    - DNS_RES_OK - The DNS client has obtained an IP. HostIP will contain the resolved address.
    - DNS_RES_PENDING - The resolution process is still in progress
    - DNS_RES_SERVER_TMO - DNS server timed out
    - DNS_RES_NO_ENTRY - no such entry to be resolved exists

  Remarks:
    None.
*/
TCPIP_DNS_RESULT  TCPIP_DNS_IsResolved(const char* HostName, void* HostIP);

//****************************************************************************
/*  
  Function:
    TCPIP_DNS_RESULT TCPIP_DNS_GetIPv4Address(const char* hostName,int index,IPV4_ADDR* ipv4Addr);

  Summary:
	Get IPV4 address from DNS client cache entry.
	
  Description:
	Call this function to Get IPv4 address if the DNS resolution of an address has
	been completed.  

  Precondition:
	TCPIP_DNS_Resolve and TCPIP_DNS_IsResolved has been called.

  Parameters:
	hostName - A pointer to the null terminated string specifying the
		host for which to resolve an IP.
	index - Get the exact location resolved IP address
    ipv4Addr - Ipv4 address to be collected for Socket open.
		
  Return Values:
  	- DNS_RES_OK - The DNS client has obtained an IPV4 address 
    - DNS_RES_NO_SERVICE - Failure  
    
  Remarks:
    none    
*/
TCPIP_DNS_RESULT TCPIP_DNS_GetIPv4Address(const char* hostName,int index,IPV4_ADDR* ipv4Addr);

//****************************************************************************
/*  Function:
    TCPIP_DNS_RESULT TCPIP_DNS_GetIPv6Address(const char* hostName,int index,IPV6_ADDR* ipv6Addr)

  Summary:
	Get IPV6 address from DNS client cache entry.
	
  Description:
	Call this function to Get IPv6 address if the DNS resolution of an address has
	been completed.  

  Precondition:
	TCPIP_DNS_Resolve and TCPIP_DNS_IsResolved has been called.

  Parameters:
	hostName - A pointer to the null terminated string specifying the
			host for which to resolve an IPv6
	index - Get the exact location resolved IP address
    ipv6Addr - Ipv6 address to be collected for Socket open
		
  Return Values:
  	- DNS_RES_OK - The DNS client has obtained an IPV4 address
    - DNS_RES_NO_SERVICE - Failure
    
  Remarks:
    none    
  */
TCPIP_DNS_RESULT TCPIP_DNS_GetIPv6Address(const char* hostName,int index,IPV6_ADDR* ipv6Addr);

//****************************************************************************
/*  
  Function:
    int TCPIP_DNS_GetNumberOfIPAddresses(const char* hostName,IP_ADDRESS_TYPE type)

  Summary:
	Get the count of resolved Ipv4 and Ipv6 address.
	
  Description:
	Call this function to get total count of Ipv4 or IPv6 address as per IP type.

  Precondition:
	DNSResolve has been called.

  Parameters:
	hostName - A pointer to the null terminated string specifying the
		host for which to resolve an IP
	type - IPv4 or IPv6 type

  Return Values:
  	Number of resolved IPv4 or IPv6 address as per IP address type and hostName
    
    
  Remarks:
    none    
  */
int TCPIP_DNS_GetNumberOfIPAddresses(const char* hostName,IP_ADDRESS_TYPE type);

//****************************************************************************
/*  
  Function:
	bool TCPIP_DNS_IsEnabled(CPIP_NET_HANDLE hNet)

  Summary:
	Determines if the DNS client is enabled on that specified interface.

  Description:
	Returns current state of DNS Client on the specified interface.

  Precondition:
	DNS module initialized

  Parameters:
	hNet - Interface to query.

  Returns:
	- true	- if the DNS client service is enabled on the specified interface
    - false	- if the DNS client service is not enabled on the specified interface
*/
bool TCPIP_DNS_IsEnabled(TCPIP_NET_HANDLE hNet);

//****************************************************************************
/*
  Function:
	bool TCPIP_DNS_Enable(TCPIP_NET_HANDLE hNet)

  Summary:
	Enables the DNS Client for the specified interface.

  Description:
	Enables the DNS Client for the specified interface, if it is disabled.
	If it is already enabled, nothing is done.

  Precondition:
	DNS module initialized

  Parameters:
	hNet - Interface to enable the DNS Client on

  Returns:
	- true	- if successful
    - false	- if unsuccessful
*/
bool TCPIP_DNS_Enable(TCPIP_NET_HANDLE hNet);

//*****************************************************************************
/*
  Function:
	bool TCPIP_DNS_Disable(TCPIP_NET_HANDLE hNet)

  Summary:
	Disables the DNS Client for the specified interface.

  Description:
	Disables the DNS Client for the specified interface.

  Precondition:
	DNS module initialized

  Parameters:
	hNet - Interface to disable the DNS Client off.

  Returns:
    - true	- if successful
    - false	- if unsuccessful

  Remarks:
    When the DNS client is disabled and the interface continues using its old configuration,
    it is possible that the lease may expire and TCPIP_DNS_IsResolved will provide 
	the IP address to another application. The application should not keep the old 
	lease unless it is sure that there is no danger of conflict.
*/
bool TCPIP_DNS_Disable(TCPIP_NET_HANDLE hNet);

// *****************************************************************************
/* 
  Function:
	TCPIP_DNS_HANDLE 
	TCPIP_DNS_HandlerRegister(TCPIP_NET_HANDLE hNet, TCPIP_DNS_EVENT_HANDLER handler, const void* hParam);

  Summary:
    Registers a DNS client Handler.

  Description:
    This function registers a DNS client event handler.
    The DNS client module will call the registered handler when a
    DNS client event (TCPIP_DNS_EVENT_TYPE) occurs.

  Precondition:
    DNS client module initialized.

  Parameters:
    hNet    - Interface handle.
              Use hNet == 0 to register on all interfaces available.
    handler - Handler to be called when a DNS client event occurs.
    hParam  - Parameter to be used in the handler call.
             For DNS client module, it is used to compare domain name. 
             If domain is specified, then the Notification will be only for that host name 
             else  notification will be for all the registered module.

  Returns:
    - Returns a valid handle if the call succeeds 
	- Returns null handle if the call failed (out of memory, for example)

  Remarks:
    The handler has to be short and fast. It is meant for
    setting an event flag, not for lengthy processing!   
 */
TCPIP_DNS_HANDLE TCPIP_DNS_HandlerRegister(TCPIP_NET_HANDLE hNet, TCPIP_DNS_EVENT_HANDLER handler, const void* hParam);

// *****************************************************************************
/* Function:
	bool TCPIP_DNS_HandlerDeRegister(TCPIP_DNS_HANDLE hDns);

  Summary:
    Deregisters a previously registered DNS client handler.
    
  Description:
    Deregisters the DNS client event handler.

  Precondition:
    DNS client module initialized.

  Parameters:
    hDns   - A handle returned by a previous call to TCPIP_DNS_HandlerRegister.

  Returns:
   - true - if the call succeeds
   - false - if no such handler is registered
 */
bool TCPIP_DNS_HandlerDeRegister(TCPIP_DNS_HANDLE hDns);

// *****************************************************************************
/* Function:
   TCPIP_DNS_RESULT 
   TCPIP_DNS_InsertHostName(uint8_t *pDnsHostName, uint16_t transactionId,DNS_RESOLVE_TYPE type,int intfIdx);

  Summary:
    Insert hostname to the DNS Hash entry.

  Description:
    This function is used to insert a entry to the DNS HASH table with a host name, 
	transaction ID, resolve type. In general, this function is called from the 
	TCPIP_DNS_Resolve. These details will be used while sending the DNS query.
    Until the DNS client receives the Response for the query, the entry flag 
	will be in Incomplete state. If there is no response to the query, the 
	entry will be removed  from the table.

  Precondition:
    DNS client module initialized.

  Parameters:
    pDnsHostName - Domain name to be inserted.
    transaction ID - need to be verified  after receiving the response
    type - DNS_TYPE_A or DNS_TYPE_AAAA
    intfIdx - interface index( not yet used). This is set to 0.

  Returns:
    - DNS_RES_OK - If Successful
    - DNS_RES_MEMORY_FAIL - If the dynamic allocation fails for pDnsHostName.
    - DNS_RES_CACHE_FULL - If there is no space in the DNS cache entry

  Remarks:
    None. 
 */
TCPIP_DNS_RESULT TCPIP_DNS_InsertHostName(uint8_t *pDnsHostName, uint16_t transactionId,DNS_RESOLVE_TYPE type,int intfIdx);
#endif
