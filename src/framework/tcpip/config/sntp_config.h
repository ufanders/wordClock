/*******************************************************************************
  Network Time Protocol (SNTP) Configuration file

  Company:
    Microchip Technology Inc.
    
  File Name:
    sntp_config.h

  Summary:
    SNTP configuration file
    
  Description:
    This file contains the SNTP module configuration options
    
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
#ifndef _SNTP_CONFIG_H_
#define _SNTP_CONFIG_H_


// for multi-homed hosts, the default SNTP interface
#define NTP_DEFAULT_IF        "PIC32INT"

// The default NTP version to use (3 or 4)
#define NTP_VERSION             (4)

// The default connection type to use: IPv4/IPv6
#define NTP_DEFAULT_CONNECTION_TYPE     (IP_ADDRESS_TYPE_IPV4)

// Reference Epoch to use.  (default: 01-Jan-1970 00:00:00)
#define NTP_EPOCH 				(86400ul * (365ul * 70ul + 17ul))

// Defines how long to wait before assuming the query has failed, seconds
#define NTP_REPLY_TIMEOUT		(6ul)

// Number of retries to connect to the server
#define NTP_MAX_RETRIES           (3)

// The maximum acceptable NTP stratum number
// Should be less than 16 (unsynchronized server) 
#define NTP_MAX_STRATUM         (15)

// elapsed time that qualifies a time stamp as stale
// normally it should be correlated with NTP_QUERY_INTERVAL
#define NTP_TIME_STAMP_TMO      (11 * 60)

// These are normally available network time servers.
// The actual IP returned from the pool will vary every
// minute so as to spread the load around stratum 1 timeservers.
// For best accuracy and network overhead you should locate the 
// pool server closest to your geography, but it will still work
// if you use the global pool.ntp.org address or choose the wrong 
// one or ship your embedded device to another geography.
#define NTP_SERVER	"pool.ntp.org"
//#define NTP_SERVER	"europe.pool.ntp.org"
//#define NTP_SERVER	"asia.pool.ntp.org"
//#define NTP_SERVER	"oceania.pool.ntp.org"
//#define NTP_SERVER	"north-america.pool.ntp.org"
//#define NTP_SERVER	"south-america.pool.ntp.org"
//#define NTP_SERVER	"africa.pool.ntp.org"

// Defines how frequently to resynchronize the date/time, seconds (default: 10 minutes)
#define NTP_QUERY_INTERVAL		(10ul*60ul)

// Defines how long to wait to retry an update after a failure, seconds.
// Updates may take up to 6 seconds to fail, so this 14 second delay is actually only an 8-second retry.
#define NTP_FAST_QUERY_INTERVAL	(14ul)






#endif // _SNTP_CONFIG_H_
