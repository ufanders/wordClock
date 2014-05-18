/*******************************************************************************
  Dynamic Host Configuration Protocol (DCHP) Configuration file

  Company:
    Microchip Technology Inc.
    
  File Name:
    dhcp_config.h

  Summary:
    DCHP configuration file
    
  Description:
    This file contains the DCHP module configuration options
    
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
#ifndef _DCHP_CONFIG_H_
#define _DCHP_CONFIG_H_


// Defines how long to wait before a DHCP request times out, seconds
#define DHCP_TIMEOUT				(2ul)


// The DHCP task processing rate: number of milliseconds to generate an DHCP tick.
// Used by the DHCP state machine
// The default value is 5ms
#define DHCP_TASK_TICK_RATE        (5)     

// Enable/disable the DHCP client at stack start-up.
// Note: the interface initialization setting in TCPIP_NETWORK_CONFIG takes precedence!
#define DHCP_CLIENT_ENABLED        1




#endif  // _DCHP_CONFIG_H_
