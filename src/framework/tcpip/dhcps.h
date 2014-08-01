/*******************************************************************************
  Dynamic Host Configuration Protocol (DHCP) Server API Header File

  Company:
    Microchip Technology Inc.

  File Name:
    dhcps.h

  Summary:
    Dynamic Host Configuration Protocol(DHCP) Server APIs.

  Description:
     The DHCP server assigns a free IP address to a requesting 
     client from the range defined in the dhcps_config.h  file.
	 Lease time per IP address is decided as per the TMO configuration 
	 which is defined in dhcps_config.h.
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

#ifndef __DHCPS_H
#define __DHCPS_H

// *****************************************************************************
// *****************************************************************************
// Section: Data Types and Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Structure:
    DHCPS_MODULE_CONFIG

  Summary:
    DHCP Server module runtime and initialization configuration data.

  Description:
    DHCP server configuration and initialization data . Configuration
	is part of tcpip_stack_init.c.
*/
typedef struct
{
    bool    enabled;   				// enable DHCP server
    bool    deleteOldLease;  		// delete old cache if still in place,
    // specific DHCP parameters
    size_t  leaseEntries;   		// max number of lease entries
    uint32_t     entrySolvedTmo; 		// solved entry removed after this tmo in seconds
    char    *startIpAddressRange; 	// IP address string which is the start value for DHCP clients.
} DHCPS_MODULE_CONFIG;

// *****************************************************************************
/*
  Structure:
    DHCPS_MODULE_CONFIG

  Summary:
    DHCP Server module runtime and initialization configuration data.

  Description:
    DHCP server configuration and initialization data. Configuration
	is part of tcpip_stack_init.c.
*/
typedef struct
{
    TCPIP_MAC_ADDR    hwAdd; // Client MAC address
    IPV4_ADDR   ipAddress;   // Leased IP address
    uint32_t    leaseTime;   // Lease period
}TCPIP_DHCPS_LEASE_ENTRY;

// *****************************************************************************
/*
  Enumeration:
    TCPIP_DHCPS_POOL_ENTRY_TYPE

  Summary:
    DHCP server pool types are used to get and remove the leased IP address details.

  Description:
    DHCP_SERVER_POOL_ENTRY_ALL -  Get or Remove all the leased address which includes 
	                              both solved and unsolved entries.
    DHCP_SERVER_POOL_ENTRY_IN_USE - Get or Remove only solved leased IP address.
*/

typedef enum
{
    DHCP_SERVER_POOL_ENTRY_ALL,    // Get or Remove all the Leased address.
    DHCP_SERVER_POOL_ENTRY_IN_USE, // Get or remove only Leased IP address.
}TCPIP_DHCPS_POOL_ENTRY_TYPE;

// *****************************************************************************
/*
  Type:
    TCPIP_DHCPS_LEASE_HANDLE

  Summary:
    DHCP Server Lease Handle 

  Description:
    A handle that server is using to provide the index of a lease entry. 

  Remarks:
    This handle is used by command handler to get the Index of Lease entry.
*/
typedef const void* TCPIP_DHCPS_LEASE_HANDLE;

// *****************************************************************************
// *****************************************************************************
// Section: DHCP Server Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Function:
    bool TCPIP_DHCPS_IsEnabled(TCPIP_NET_HANDLE hNet)

  Summary:
    Determines if the DHCP Server is enabled on the specified interface.

  Description:
    Returns the current state of the DHCP Server on the specified interface.

  Precondition:
    DHCP Server module initialized.

  Parameters:
    hNet- Interface to query.

  Returns:
    - true	- if successful
    - false	- if unsuccessful
*/
bool TCPIP_DHCPS_IsEnabled(TCPIP_NET_HANDLE hNet);

/*****************************************************************************
  Function:
    bool TCPIP_DHCPS_Disable(TCPIP_NET_HANDLE hNet)

  Summary:
    Disables the DHCP Server for the specified interface.

  Description:
    Disables the DHCP Server for the specified interface.
	If it is already disabled, no action is taken.

  Precondition:
    DHCP Server module initialized.

  Parameters:
    hNet - Interface to disable the DHCP Server on.

  Returns:
    - true	- if successful
    - false	- if unsuccessful

  Remarks:
    When the interface continues using its old configuration, it is possible
    that the lease may take sometime to expire. And The communication will 
    be there until it is not expired.Lease time is configured in dhcps_config.h.
 */
bool TCPIP_DHCPS_Disable(TCPIP_NET_HANDLE hNet);

/*****************************************************************************
  Function:
    void TCPIP_DHCPS_Enable(TCPIP_NET_HANDLE hNet)

  Summary:
    Enables the DHCP Server for the specified interface.

  Description:
    Enables the DHCP Server for the specified interface, if it is disabled.
    If it is already enabled, nothing is done.

  Precondition:
    DHCP Server module initialized.

  Parameters:
     hNet - Interface to enable the DHCP Server on.

  Returns:
    - true	- if successful
    - false	- if unsuccessful
*/
bool TCPIP_DHCPS_Enable(TCPIP_NET_HANDLE hNet);

/*****************************************************************************
  Function:
    TCPIP_DHCPS_LEASE_HANDLE  TCPIP_DHCPS_LeaseEntryGet(TCPIP_NET_HANDLE netH, 
		TCPIP_DHCPS_LEASE_ENTRY* pLeaseEntry, TCPIP_DHCPS_LEASE_HANDLE leaseHandle);
		
  Summary:
    Get the lease entry details as per TCPIP_DHCPS_LEASE_HANDLE and per interface.

  Description:
    Returns a lease entry for the  TCPIP_DHCPS_LEASE_HANDLE. if the lease entry is not 
	present for that TCPIP_DHCPS_LEASE_HANDLE, then it will return the next valid lease entry.

  Precondition:
    DHCP Server module initialized.

  Parameters:
    netH - Lease entry for this Interface
	pLeaseEntry - Client lease entry details
	leaseHandle - Lease index 

  Returns:
    - returns a non-zero TCPIP_DHCPS_LEASE_HANDLE to be used in the subsequent calls 
	- returns 0 if end of list or wrong interface, or DHCP server is not running on that interface
*/
TCPIP_DHCPS_LEASE_HANDLE  TCPIP_DHCPS_LeaseEntryGet(TCPIP_NET_HANDLE netH, TCPIP_DHCPS_LEASE_ENTRY* pLeaseEntry, TCPIP_DHCPS_LEASE_HANDLE leaseHandle);

/******************************************************************************
 Function:
    bool TCPIP_DHCPS_RemovePoolEntries(TCPIP_NET_HANDLE netH, TCPIP_DHCPS_POOL_ENTRY_TYPE type);

  Summary:
    Removes all the entries or only used entries of a certain type belonging to a network interface.

  Description:
    This API is used to remove the DHCP server entries from the pool as per TCPIP_DHCPS_POOL_ENTRY_TYPE. 
	
  Precondition:
    DHCP server module should have been initialized

  Parameters:
    hNet    -   Interface handle to use
    type    -   type of entries to remove:
				DHCP_SERVER_POOL_ENTRY_ALL,
				DHCP_SERVER_POOL_ENTRY_IN_USE

  Returns:
    - true	- if successful
    - false	- if unsuccessful

  Remarks:
    None
  ***************************************************************************/

bool TCPIP_DHCPS_RemovePoolEntries(TCPIP_NET_HANDLE netH, TCPIP_DHCPS_POOL_ENTRY_TYPE type);

/******************************************************************************
 Function:
    int TCPIP_DHCPS_GetPoolEntries(TCPIP_NET_HANDLE netH, TCPIP_DHCPS_POOL_ENTRY_TYPE type);

  Summary:
    Get all the entries or only used entries of a certain type belonging to a network interface.

  Description:
    This API is used to get the DHCP server entries from the pool as per TCPIP_DHCPS_POOL_ENTRY_TYPE. 
	
  Precondition:
    DHCP server module should have been initialized

  Parameters:
    hNet    -   Interface handle to use
    type    -   type of entries to remove:
			DHCP_SERVER_POOL_ENTRY_ALL,
            DHCP_SERVER_POOL_ENTRY_IN_USE

  Returns:
    - true	- if successful
    - false	- if unsuccessful

  Remarks:
    None
  ***************************************************************************/

int TCPIP_DHCPS_GetPoolEntries(TCPIP_NET_HANDLE netH, TCPIP_DHCPS_POOL_ENTRY_TYPE type);


#endif // __DHCPS_H

