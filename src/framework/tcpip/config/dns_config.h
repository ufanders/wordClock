/*******************************************************************************
  Domain Name Service (CNS) Configuration file

  Company:
    Microchip Technology Inc.
    
  File Name:
    dns_config.h

  Summary:
    DNS configuration file
    
  Description:
    This file contains the DNS module configuration options
    
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright © 2011 released Microchip Technology Inc.  All rights reserved.

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
#ifndef _DNS_CONFIG_H_
#define _DNS_CONFIG_H_

// For multi-homed hosts, the default DNS Ethernet interface
#define DNS_DEFAULT_IF        "PIC32INT"

// When the DNS Client performs an ARP request for the DNS Server
// this is the elapsed time after which a ARP resolution is considered
// to have timed out
// In seconds
#define DNS_CLIENT_ARP_TMO		(1*60)		

// The DNS Client performs an UDP socket open request
// to communicate with the DNS Server
// This is the elapsed time after which an open request is considered
// to have timed out
// In seconds
#define DNS_CLIENT_OPEN_TMO		(1*60)

// When the DNS Client connected to the DNS Server
// this is the elapsed time after which an the communication is considered
// to have timed failed if there was no reply from the server
// In seconds
#define DNS_CLIENT_SERVER_TMO		(1*60)

// DNS Client task processing rate, in seconds.
// The DNS Client module will process a timer event with this rate
// for processing its own state machine, etc.
#define DNS_CLIENT_TASK_PROCESS_RATE	(2)

// Number of DNS resolver entries 
#define DNS_CLIENT_CACHE_ENTRIES    5

// DNS client Time-outs . if this macro is zero then entries will be deleted w.r.t TTL. 
// If the macro value is other than 0 , then this value will be considered as the final 
// value for the hash entry time-out. Default will be kept as 0.
#define DNS_CLIENT_CACHE_ENTRY_TMO 0

// Maximum and default number of IPv4 answers to be considered 
// while processing DNS response from server for a query.
#define DNS_CLIENT_CACHE_PER_IPV4_ADDRESS       5

// Maximum and default number of IPv6 answers to be considered 
// while processing DNS response from server for a query.
#define DNS_CLIENT_CACHE_PER_IPV6_ADDRESS       1

// This parameter can be used to choose for DNS client open. 
// At present it is default IPv4 address type. 
#define DNS_CLIENT_OPEN_ADDRESS_TYPE       IP_ADDRESS_TYPE_IPV4

// Default TTL time  for a solved entry in the cache
#define DNS_CACHE_DEFAULT_TTL_VAL      (20 * 60)

// Time-out for the a unsolved entry in the cache, in seconds.
// The entry will be removed if the TMO elapsed
// and the entry has not been referenced again
#define DNS_CACHE_UNSOLVED_ENTRY_TMO      (10)

#endif  // _DNS_CONFIG_H_



