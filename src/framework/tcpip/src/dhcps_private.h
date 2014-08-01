/*******************************************************************************
  DHCPS Private file for Microchip TCP/IP Stack

  Company:
    Microchip Technology Inc.
    
  File Name:
    dhcps_private.h

  Summary:
    DHCPS Private file for Microchip TCP/IP Stack

  Description:
    This file provides the TCP/IP Stack DHCPS Private file API definitions.
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

#ifndef _DHCPS_PRIVATE_H_ 
#define _DHCPS_PRIVATE_H_

#include "tcpip/src/oahash.h"
#include "tcpip/src/link_list.h"

// Boot file name, null terminated string; "generic" name or null in DHCPDISCOVER, fully qualified
// directory-path name in DHCPOFFER.
#define DHCPS_BOOTFILE_NAME_SIZE 128
// Optional server host name, null terminated string.
#define DHCPS_HOST_NAME_SIZE	 64
// Unused Client Hardware address
#define DHCPS_CLEINT_HW_ADDRESS_SIZE 10

#define DHCPS_UNUSED_BYTES_FOR_TX   (DHCPS_BOOTFILE_NAME_SIZE+DHCPS_HOST_NAME_SIZE+DHCPS_CLEINT_HW_ADDRESS_SIZE)


#define DHCPS_MAX_REPONSE_PACKET_SIZE 300
typedef struct 
{
    IPV4_ADDR	serverIPAddress;	// Interface IP address when DHCP server is enabled
    IPV4_ADDR	serverGateway;	// Interface Gateway address when DHCP server is enabled
    IPV4_ADDR	serverMask;		// Interface NetMask address when DHCP server is enabled
#if defined(TCPIP_STACK_USE_DNS)
    IPV4_ADDR	serverDNS;		// Interface primary DNS server address when DHCP server is enabled
    IPV4_ADDR	serverDNS2;		// Interface secondary DNS server address when DHCP server is enabled
#endif	
}DHCPS_INTERFACE_CONFIG;

typedef struct __attribute__((packed))
{
    // DHCP Message Byte Option PDU
    uint8_t optionType;  // Option type 
    uint8_t optionTypeLen; // Option type len
    uint8_t byteVal;    
}TCPIP_DHCPS_OPTION_BYTE_DATATYPE;

typedef struct __attribute__((packed))
{
    // DHCP Message Byte Option PDU
    uint8_t optionType;  // Option type 
    uint8_t optionTypeLen; // Option type len
    uint16_t shortVal;    
}TCPIP_DHCPS_OPTION_SHORT_DATATYPE;

typedef struct __attribute__((packed))
{
    // DHCP Message Byte Option PDU
    uint8_t optionType;  // Option type 
    uint8_t optionTypeLen; // Option type len
    uint32_t intVal;    
}TCPIP_DHCPS_OPTION_INT_DATATYPE;

typedef struct 
{
    TCPIP_DHCPS_OPTION_BYTE_DATATYPE messageType ; // Message Type
    TCPIP_DHCPS_OPTION_INT_DATATYPE  subnetmaskType; // subnet mask type
    TCPIP_DHCPS_OPTION_INT_DATATYPE  serverIdentifierType;  // server Identifier Type
    TCPIP_DHCPS_OPTION_INT_DATATYPE  routerType;  // router Type
    TCPIP_DHCPS_OPTION_INT_DATATYPE  dnsType;  // DNS type
    TCPIP_DHCPS_OPTION_INT_DATATYPE  ipLeaseTimeType;  // IP lease time type
}TCPIP_DHCPS_OPTION;

/*
Various Definitions for Success and Failure Codes

  Summary:
    None

  Description:
    None

  Remarks:
    None
*/
typedef enum
{
    // success codes
    DHCPS_RES_OK                  = 0,    // operation succeeded
    DHCPS_RES_ENTRY_NEW,                  // operation succeeded and a new entry was added
    DHCPS_RES_ENTRY_EXIST,                // the required entry was already cached
            
    // failure codes
    DHCPS_RES_NO_ENTRY            = -1,   // no such entry exists    
    DHCPS_RES_CACHE_FULL          = -2,   // the cache is full and no entry could be
}DHCPS_RESULT;



typedef enum
{
    DHCP_SERVER_OPEN_SOCKET,
    DHCP_SERVER_LISTEN,
    DHCP_SERVER_IDLE,
}DHCP_SRVR_STAT;


typedef struct
{
    bool            enabled;            // Whether or not the DHCP server is enabled
    UDP_SOCKET      uSkt;               // Socket used by DHCP Server
    IPV4_ADDR		dhcpNextLease;	    // IP Address to provide for next lease
    DHCP_SRVR_STAT  smServer;           // server state machine status    
    tcpipAsyncHandle timerHandle;
    DHCPS_INTERFACE_CONFIG intfAddrsConf;	
    int     netIx;				   // index of the current interface addressed
}DHCP_SRVR_DCPT;    // DHCP server descriptor

    
// DHCP Server cache entry
typedef struct	_TAG_DHCPS_HASH_ENTRY 
{
	OA_HASH_ENTRY				hEntry; 		// hash header;
	uint32_t					Client_Lease_Time;
	uint32_t					pendingTime;
	IPV4_ADDR					ipAddress;	  // the hash key: the IP address
	TCPIP_MAC_ADDR					hwAdd;			// the hardware address
	int							intfIdx;
}DHCPS_HASH_ENTRY;

#define     DHCPS_HASH_PROBE_STEP      1    // step to advance for hash collision
#define     DHCPS_HASH_KEY_SIZE        (sizeof(((DHCPS_HASH_ENTRY*)0)->hwAdd))

// each DHCPS Lease entry consists of
typedef struct
{
    OA_HASH_DCPT*       hashDcpt;       // contiguous space for a hash descriptor  and hash table entries    
    SINGLE_LIST         completeList;   // list of completed, valid entries
    uint32_t		leaseDuartion;
    IPV4_ADDR		dhcpSStartAddress;	    // IP Address to provide for next lease
}DHCPS_HASH_DCPT;

// DHCPS ENTRY flags used in hEntry->flags
// note that only the hEntry->flags.user fields
// should be used! 
typedef enum
{
    DHCPS_FLAG_ENTRY_BUSY         = 0x0001,          // this is used by the hash itself!
    // user flags
    DHCPS_FLAG_ENTRY_INCOMPLETE   = 0x0040,          // entry is not completed yet
    DHCPS_FLAG_ENTRY_COMPLETE     = 0x0080,          // regular entry, complete
                                                   // else it's incomplete
                                                   //
    DHCPS_FLAG_ENTRY_VALID_MASK   = (DHCPS_FLAG_ENTRY_INCOMPLETE | DHCPS_FLAG_ENTRY_COMPLETE )
                                                     
                                                  
}DHCPS_ENTRY_FLAGS;

// DHCPS unaligned key
// the IP address field is unaligned
// in DHCPS 
typedef struct __attribute__((packed))
{
    uint32_t    v;
}DHCPS_UNALIGNED_KEY;

typedef struct
{
    uint8_t *head;      // probably this is the allocated structure
    uint8_t* wrPtr;     // current write pointer, initialized to head when buffer allocated
    uint8_t* endPtr;    // end pointer, initialized to head + len when buffer allocated
}TCPIP_DHCPS_DATA;

int TCPIP_DHCPS_HashMACKeyCompare(OA_HASH_DCPT* pOH, OA_HASH_ENTRY* hEntry, void* key);
int TCPIP_DHCPS_HashIPKeyCompare(OA_HASH_DCPT* pOH, OA_HASH_ENTRY* hEntry, void* key);
void TCPIP_DHCPS_HashIPKeyCopy(OA_HASH_DCPT* pOH, OA_HASH_ENTRY* dstEntry, void* key);
void TCPIP_DHCPS_HashMACKeyCopy(OA_HASH_DCPT* pOH, OA_HASH_ENTRY* dstEntry, void* key);
OA_HASH_ENTRY* TCPIP_DHCPS_HashDeleteEntry(OA_HASH_DCPT* pOH);
size_t TCPIP_DHCPS_MACHashKeyHash(OA_HASH_DCPT* pOH, void* key);
size_t TCPIP_DHCPS_HashProbeHash(OA_HASH_DCPT* pOH, void* key);
size_t TCPIP_DHCPS_IPAddressHashKeyHash(OA_HASH_DCPT* pOH, void* key);






#endif  // _DHCPS_PRIVATE_H_ 


