/*******************************************************************************
  DHCP Server manager private API for Microchip TCP/IP Stack

  Company:
    Microchip Technology Inc.
    
  File Name:
    dhcps_manager.h

  Summary:
    DHCP Server manager private API for Microchip TCP/IP Stack

  Description:
    This file provides the TCP/IP Stack DHCP Server Manager Private API definitions.
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

#ifndef _DHCPS_MANAGER_H_
#define _DHCPS_MANAGER_H_


// Returns true if service needed, called by the stack manager
bool TCPIP_DHCPS_TaskIsPending(void);

// Task that checks lease time.
void TCPIP_DHCPS_TaskForLeaseTime(void);


/*****************************************************************************
  Function:
    bool TCPIP_DHCPS_Task(TCPIP_NET_IF* pNetIf)

  Summary:
    Performs periodic DHCP server tasks.

  Description:
    This function performs any periodic tasks required by the DHCP server
    module, such as processing DHCP requests and distributing IP addresses.

  Precondition:
    None

  Parameters:
    pNetIf   - interface

  Returns:
    None
  ***************************************************************************/
bool TCPIP_DHCPS_Task(TCPIP_NET_IF* pNetIf);


/*****************************************************************************
  Function:
    bool TCPIP_DHCPS_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const DHCPS_MODULE_CONFIG* pDhcpConfig);

  Summary:
    Resets the DHCP server module for the specified interface.

  Description:
    Resets the DHCP server module for the specified interface.

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
bool TCPIP_DHCPS_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const DHCPS_MODULE_CONFIG* pDhcpConfig);


/*****************************************************************************
  Function:
    bool TCPIP_DHCPS_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl);

  Summary:
    Turns off the DHCP server module for the specified interface.

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
void TCPIP_DHCPS_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl);

#endif  // _DHCPS_MANAGER_H_


