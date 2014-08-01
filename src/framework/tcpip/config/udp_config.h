/*******************************************************************************
  User Datagram Protocol (UDP) Configuration file

  Company:
    Microchip Technology Inc.
    
  File Name:
    udp_config.h

  Summary:
    UDP Configuration file
    
  Description:
    This file contains the UDP module configuration options
    
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
#ifndef _UDP_CONFIG_H_
#define _UDP_CONFIG_H_

// Maximum number of UDP sockets that can be opened simultaneously
// These sockets will be created when the module is initialized.
#define UDP_MAX_SOCKETS			(10)

// Default socket TX buffer size.
#define UDP_SOCKET_DEFAULT_TX_SIZE	512

// Calculate and transmit a checksum when sending data.
// Checksum is not mandatory for UDP packets but is highly recommended.
// This will affect the UDP TX performance.
#define UDP_USE_TX_CHECKSUM

// Check incoming packets to have proper checksum.
#define UDP_USE_RX_CHECKSUM

// The maximum number of packets that can be queued
// by an UDP socket at a certain time.
// For sockets that need to transfer a lot of data (Iperf, for example),
// especially on slow connections this limit prevents running out of memory
// because the MAC/PHY transfer cannot keep up with the UDP packet allocation rate
// imposed by the application.
// Adjust depending on the UDP_SOCKET_DEFAULT_TX_SIZE, the connection speed
// and the amount of memory available to the stack.
#define UDP_SOCKET_DEFAULT_TX_QUEUE_LIMIT   3


// Number of buffers in the private UDP pool
// These are preallocated buffers that are to be used
// by UDP sockets only.
// This improves the UDP socket throughput.
// However, this memory is not returned to the stack heap,
// it always belongs to UDP.
// A socket needs to have an option set in order to use the buffers pool
// (see the UDPSetOptions()).
#define UDP_SOCKET_POOL_BUFFERS         4       // use 0 to disable the feature

#define UDP_SOCKET_POOL_BUFFER_SIZE     512     // size of the buffers in the UDP pool
                                                // any UDP socket that is enabled to use the pool
                                                // and has the TX size <= than this size can use 
                                                // a buffer from the pool



#endif  // _UDP_CONFIG_H_
