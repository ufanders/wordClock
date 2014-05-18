/*******************************************************************************
  TCP/IP modules manager file

  Company:
    Microchip Technology Inc.
    
  File Name:
    tcpip_module_manager.h

  Summary:
    Internal TCP/IP stack module manager file
    
  Description:
    This header file contains the function prototypes and definitions of the 
    TCP/IP stack manager services
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

#ifndef __TCPIP_MODULE_MANAGER_H_
#define __TCPIP_MODULE_MANAGER_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>


#if defined(TCPIP_IF_MRF24W)
#include "driver/wifi/mrf24w/drv_wifi.h"
#endif  // defined(TCPIP_IF_MRF24W)

#include "driver/ethmac/drv_ethmac.h"

// definitions
// 


// ************* stack supported modules and their attached functionality **************
// 

// ******* table with TCPIP stack modules **************
// We use directly the functions names (pointers) rather than providing registration functions
// so that we can create this table in const space
//

// initialization function
// if the module has initialization to do, this function
// will be called. It should return a result to indicate
// if the initialization was successful. If not, the
// interface will not be completed.
typedef bool    (*tcpipModuleInitFunc)(const TCPIP_STACK_MODULE_CTRL* const, const void* );

// de-initialization function
// if the module needs to clean up when the module is
// brought down, this function will be called. It should
// return a result to indicate that everything has been
// cleaned up.
typedef void    (*tcpipModuleDeInitFunc)(const TCPIP_STACK_MODULE_CTRL * const);


// descriptor of an TCPIP stack module entry
// module that's part of the stack
// each module has an ID and init/deinit functions
// 
typedef struct
{
    TCPIP_STACK_MODULE       moduleId;           // module identification
    tcpipModuleInitFunc      initFunc;           // initialization function
    tcpipModuleDeInitFunc    deInitFunc;         // deinitialization function
}TCPIP_STACK_MODULE_ENTRY;


static const TCPIP_STACK_MODULE_ENTRY  TCPIP_STACK_MODULE_ENTRY_TBL [] =
{
    //ModuleID                  //InitFunc                                      //DeInitFunc                
#if defined(TCPIP_STACK_USE_IPV4)
    {TCPIP_MODULE_IPV4,          (tcpipModuleInitFunc)TCPIP_IPV4_Initialize,    TCPIP_IPV4_DeInitialize},       // TCPIP_MODULE_IPV4,
#endif
#if defined(TCPIP_STACK_USE_ICMP_CLIENT) || defined(TCPIP_STACK_USE_ICMP_SERVER)
    {TCPIP_MODULE_ICMP,          (tcpipModuleInitFunc)TCPIP_ICMP_Initialize,           TCPIP_ICMP_Deinitialize},              // TCPIP_MODULE_ICMP,
#endif
    {TCPIP_MODULE_ARP,           (tcpipModuleInitFunc)TCPIP_ARP_Initialize,            TCPIP_ARP_Deinitialize},               // TCPIP_MODULE_ARP,
#if defined(TCPIP_STACK_USE_IPV6)
    {TCPIP_MODULE_IPV6,          (tcpipModuleInitFunc)TCPIP_IPV6_Initialize,    TCPIP_IPV6_Deinitialize},       // TCPIP_MODULE_IPV6
    {TCPIP_MODULE_ICMPV6,        (tcpipModuleInitFunc)TCPIP_ICMPV6_Initialize,  TCPIP_ICMPV6_Deinitialize},     // TCPIP_MODULE_ICMPV6
    {TCPIP_MODULE_NDP,           (tcpipModuleInitFunc)TCPIP_NDP_Initialize,     TCPIP_NDP_Deinitialize},        // TCPIP_MODULE_NDP
#endif
#if defined(TCPIP_STACK_USE_UDP)
    {TCPIP_MODULE_UDP,           (tcpipModuleInitFunc)TCPIP_UDP_Initialize,                  TCPIP_UDP_Deinitialize},                     // TCPIP_MODULE_UDP,
#endif
#if defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)
    {TCPIP_MODULE_SSL,           (tcpipModuleInitFunc)TCPIP_SSL_Initialize,                  TCPIP_SSL_Deinitialize},                     // TCPIP_MODULE_SSL,
#endif
#if defined(TCPIP_STACK_USE_TCP)
    {TCPIP_MODULE_TCP,           (tcpipModuleInitFunc)TCPIP_TCP_Initialize,                  TCPIP_TCP_Deinitialize},                     //  TCPIP_MODULE_TCP,
#endif    
#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
    {TCPIP_MODULE_DHCP_CLIENT,   (tcpipModuleInitFunc)TCPIP_DHCP_Initialize,                 TCPIP_DHCP_Deinitialize},                    // TCPIP_MODULE_DHCP_CLIENT,
#endif
#if defined(TCPIP_STACK_USE_DHCP_SERVER)
    {TCPIP_MODULE_DHCP_SERVER,   (tcpipModuleInitFunc)TCPIP_DHCPS_Initialize,           TCPIP_DHCPS_Deinitialize},              // TCPIP_MODULE_DHCP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_ANNOUNCE)
    {TCPIP_MODULE_ANNOUNCE,      (tcpipModuleInitFunc)TCPIP_ANNOUNCE_Init,      TCPIP_ANNOUNCE_DeInit},         // TCPIP_MODULE_ANNOUNCE,
#endif
#if defined(TCPIP_STACK_USE_DNS)
    {TCPIP_MODULE_DNS_CLIENT,    (tcpipModuleInitFunc)TCPIP_DNS_ClientInitialize,            TCPIP_DNS_ClientDeinitialize},               // TCPIP_MODULE_DNS_CLIENT,
#endif
#if defined(TCPIP_STACK_USE_NBNS)
    {TCPIP_MODULE_NBNS,          (tcpipModuleInitFunc)TCPIP_NBNS_Initialize,                 TCPIP_NBNS_Deinitialize},                    // TCPIP_MODULE_NBNS
#endif
#if defined(TCPIP_STACK_USE_SMTP_CLIENT)
    {TCPIP_MODULE_SMTP_CLIENT,   (tcpipModuleInitFunc)TCPIP_SMTP_ClientInitialize,           TCPIP_SMTP_ClientDeinitialize},              // TCPIP_MODULE_SMTP_CLIENT,
#endif
#if defined(TCPIP_STACK_USE_SNTP_CLIENT)
    {TCPIP_MODULE_SNTP,          (tcpipModuleInitFunc)TCPIP_SNTP_Initialize,                 TCPIP_SNTP_Deinitialize},                    // TCPIP_MODULE_SNTP,
#endif
#if defined(TCPIP_STACK_USE_BERKELEY_API)
    {TCPIP_MODULE_BERKELEY,      (tcpipModuleInitFunc)BerkeleySocketInit,       BerkeleySocketDeInit},          // TCPIP_MODULE_BERKELEY,
#endif
#if defined(TCPIP_STACK_USE_HTTP_SERVER)
    {TCPIP_MODULE_HTTP_SERVER,   (tcpipModuleInitFunc)TCPIP_HTTP_Initialize,                 TCPIP_HTTP_Deinitialize},                    // TCPIP_MODULE_HTTP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_TELNET_SERVER)
    {TCPIP_MODULE_TELNET_SERVER, (tcpipModuleInitFunc)TCPIP_TELNET_Initialize,               TCPIP_TELNET_Deinitialize},                  // TCPIP_MODULE_TELNET_SERVER,
#endif
#if defined(TCPIP_STACK_USE_RSA)
    {TCPIP_MODULE_RSA,           (tcpipModuleInitFunc)TCPIP_RSA_Initialize,                  TCPIP_RSA_Deinitialize},                     // TCPIP_MODULE_RSA,
#endif
#if defined(TCPIP_STACK_USE_FTP_SERVER)
    {TCPIP_MODULE_FTP_SERVER,    (tcpipModuleInitFunc)TCPIP_FTP_ServerInit,     TCPIP_FTP_DeInit},        // TCPIP_MODULE_FTP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_SNMP_SERVER)
    {TCPIP_MODULE_SNMP_SERVER,   (tcpipModuleInitFunc)TCPIP_SNMP_Initialize,                 TCPIP_SNMP_Deinitialize},                    // TCPIP_MODULE_SNMP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_DNS_SERVER)
    {TCPIP_MODULE_DNS_SERVER,    (tcpipModuleInitFunc)TCPIP_DNSS_Initialize,            TCPIP_DNSS_Deinitialize},               // TCPIP_MODULE_DNS_SERVER,
#endif
#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    {TCPIP_MODULE_DYNDNS_CLIENT, (tcpipModuleInitFunc)TCPIP_DDNS_Initialize,                 TCPIP_DDNS_Deinitialize},                    // TCPIP_MODULE_DYNDNS_CLIENT,
#endif
#if defined(TCPIP_STACK_USE_REBOOT_SERVER)
    {TCPIP_MODULE_REBOOT_SERVER, (tcpipModuleInitFunc)TCPIP_REBOOT_Initialize,         TCPIP_REBOOT_Deinitialize},            // TCPIP_MODULE_REBOOT_SERVER,
#endif
#if defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)
    {TCPIP_MODULE_ZCLL,          (tcpipModuleInitFunc)TCPIP_ZCLL_Initialize,     TCPIP_ZCLL_Deinitialize},        // TCPIP_MODULE_ZCLL,
#if defined(TCPIP_STACK_USE_ZEROCONF_MDNS_SD)
    {TCPIP_MODULE_MDNS,          (tcpipModuleInitFunc)TCPIP_MDNS_Initialize,           TCPIP_MDNS_Deinitialize},              // TCPIP_MODULE_MDNS,
#endif
#endif
#if defined(TCPIP_STACK_COMMAND_ENABLE)
    {TCPIP_MODULE_TCPIP_COMMAND, (tcpipModuleInitFunc)TCPIP_Commands_Initialize,        TCPIP_Commands_Deinitialize},           //    TCPIP_MODULE_TCPIP_COMMAND
#endif
#if defined(TCPIP_STACK_USE_IPERF)
    {TCPIP_MODULE_TCPIP_IPERF,   (tcpipModuleInitFunc)TCPIP_IPERF_Initialize,   TCPIP_IPERF_Deinitialize},      //    TCPIP_MODULE_TCPIP_IPERF
#endif
        // Add other stack modules here
     
};

// *********** a stack module that exposes an asynchronous handle ****************


typedef struct
{
    tcpipModuleAsyncHandler     asyncHandler;       // attention handler
                                                    // if NULL, the corresponding entry is available
    tcpipModuleAsyncPending     asyncPending;       // returns true if attention needed
                                                    // if NULL, the module does not need its own timing resources
                                                    // but uses timeouts derived from the stack manager timeout
    int16_t                     asyncTmo;           // module required timeout, msec; 
                                                    // the stack manager checks that the module reached its timeout
                                                    // used only when asyncPending == 0
    int16_t                     currTmo;            // current module timeout, msec; maintained by the stack manager
                                                    // used only when asyncPending == 0

}TCPIP_STACK_ASYNC_MODULE_ENTRY;

// number of services in the stack requiring asynchronous/timeout services
// currently the following modules require it:
//  ARP, TCP, Announce, DNS Client, SSL, WiFi MAC, DHCP client, DHCP server, TCPIP commands, iperf
//  IPv6 (5 slots)
// update as needed
//
#if defined(TCPIP_STACK_USE_IPV6)
#define TCPIP_STACK_ASYNC_MODULE_ENTRIES    16
#else
#define TCPIP_STACK_ASYNC_MODULE_ENTRIES    11
#endif  // defined(TCPIP_STACK_USE_IPV6)




// *************** a stack module that exposes a synchronous handle *************


// synchronous event handler
// the stack calls it when there's an MAC event pending
typedef bool    (*tcpipModuleSyncHandler)(TCPIP_NET_IF* pNetIf);



typedef struct
{
    tcpipModuleSyncHandler      syncHandler;        // synchronous handler
}TCPIP_STACK_SYNC_MODULE_ENTRY;

// table containing all the modules having synchronous handlers
// although this table does not contain the module ID
// it is maintained in the same order as TCPIP_STACK_MODULE_ENTRY_TBL!

#define temp_synch(handler)      ((tcpipModuleSyncHandler)handler)

static const TCPIP_STACK_SYNC_MODULE_ENTRY  TCPIP_STACK_MODULE_SYNC_TBL [] =
{
    // syncHandler
#if defined(TCPIP_STACK_USE_DHCP_SERVER)
    {TCPIP_DHCPS_Task},                                   // TCPIP_MODULE_DHCP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_ANNOUNCE)
    {TCPIP_ANNOUNCE_Task},                              // TCPIP_MODULE_ANNOUNCE,
#endif
#if defined(TCPIP_STACK_USE_NBNS)
    {TCPIP_NBNS_Task},                                         // TCPIP_MODULE_NBNS
#endif
#if defined(TCPIP_STACK_USE_DNS)
    {TCPIP_DNS_ClientTask},                        // TCPIP_MODULE_DNS_SERVER,
#endif
#if defined(TCPIP_STACK_USE_DNS_SERVER)
    {TCPIP_DNSS_Task},                        // TCPIP_MODULE_DNS_SERVER,
#endif
#if defined(TCPIP_STACK_USE_SMTP_CLIENT)
    {temp_synch(TCPIP_SMTP_ClientTask)},                       // TCPIP_MODULE_SMTP_CLIENT,
#endif
#if defined(TCPIP_STACK_USE_SNTP_CLIENT)
    {TCPIP_SNTP_Client},                                       // TCPIP_MODULE_SNTP,
#endif
#if defined(TCPIP_STACK_USE_HTTP_SERVER)
    {temp_synch(TCPIP_HTTP_Server)},                           // TCPIP_MODULE_HTTP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_TELNET_SERVER)
    {TCPIP_TELNET_Task},                                       // TCPIP_MODULE_TELNET_SERVER,
#endif
#if defined(TCPIP_STACK_USE_FTP_SERVER)
    {TCPIP_FTP_Server},                                 // TCPIP_MODULE_FTP_SERVER,
#endif
#if defined(TCPIP_STACK_USE_SNMP_SERVER)
    {TCPIP_SNMP_Task},                                         // TCPIP_MODULE_SNMP_SERVER,
#endif

#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    {temp_synch(TCPIP_DDNS_Task)},                             // TCPIP_MODULE_DYNDNS_CLIENT,
#endif
#if defined(TCPIP_STACK_USE_REBOOT_SERVER)
    {TCPIP_REBOOT_Task},                                 // TCPIP_MODULE_REBOOT_SERVER,
#endif
#if defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)
    {TCPIP_ZCLL_Process},                                // TCPIP_MODULE_ZCLL,
#if defined(TCPIP_STACK_USE_ZEROCONF_MDNS_SD)
    {TCPIP_MDNS_Process},                                      // TCPIP_MODULE_MDNS,
#endif
#endif
#if defined(TCPIP_STACK_USE_IPERF)
    {temp_synch(TCPIP_IPERF_Task)},                     // TCPIP_MODULE_TCPIP_IPERF
#endif
    // Add other needed services having synchronous handlers

};

// *************** a stack module that exposes an get configuration function *************


// get configuration function
// each module could have its own function to retrieve the current configuration
typedef size_t (*tcpipModuleGetConfig)(TCPIP_STACK_MODULE modId, void* configBuff, size_t buffSize, size_t* pConfigSize);


typedef struct
{
    TCPIP_STACK_MODULE      moduleId;       // module identification
    tcpipModuleGetConfig    getConfig;      // get Config function
}TCPIP_STACK_GET_CONFIG_MODULE_ENTRY;

// table containing all the modules having get configuration handlers
static const TCPIP_STACK_GET_CONFIG_MODULE_ENTRY  TCPIP_STACK_MODULE_GET_CONFIG_ENTRY_TBL [] =
{
    //moduleId                  //getConfig 
#if defined(TCPIP_IF_PIC32INT)
    {TCPIP_MODULE_MAC_PIC32INT, DRV_ETHMAC_PIC32MACGetConfig},          // TCPIP_MODULE_MAC_PIC32INT
#endif
#if defined(TCPIP_IF_MRF24W)
    {TCPIP_MODULE_MAC_MRF24W,   MRF24W_MACGetConfig},        // TCPIP_MODULE_MAC_MRF24W
#endif

    // Add other modules get config handlers
};

// **************** descriptor of an TCPIP stack MAC entry *************
// (MAC module supported by the stack)
// 

// MAC initialization function
// Returns a result to indicate that the initialization was successful.
// If it fails, the stack won't turn up that interface
// If the operation needs to wait for the hardware, the initialization
// function can return a pending code
typedef TCPIP_MAC_RES    (*tcpipMacInitFunc)(TCPIP_MAC_MODULE_CTRL* const, const void*);

// MAC deinitialization function
// Returns a result to indicate that everything has been
// cleaned up.
typedef TCPIP_MAC_RES    (*tcpipMacDeInitFunc)(const TCPIP_MAC_MODULE_CTRL * const);

// function to open a MAC and get a client handle
typedef TCPIP_MAC_HANDLE    (*tcpipMacOpenFunc)( TCPIP_STACK_MODULE macId );



typedef struct
{
    TCPIP_STACK_MODULE          moduleId;           // MAC module identification
    const char*                 interfaceName;      // corresponding interface name
    tcpipMacInitFunc            initFunc;           // initialization function
    tcpipMacDeInitFunc          deInitFunc;         // deinitialization function
    tcpipMacOpenFunc            openFunc;           // open function
}TCPIP_STACK_MODULE_MAC_ENTRY;

// table with TCPIP stack MAC modules
// We use functions pointers rather than variable pointers so that we can create this table in const
// Also, the name of the functions is fixed rather than providing registration functions
//

static const TCPIP_STACK_MODULE_MAC_ENTRY  TCPIP_STACK_MODULE_MAC_ENTRY_TBL [] =
{
    //ModuleID                  //interfaceName               //InitFunc                                       //DeInitFunc                // openFunc
#if defined(TCPIP_IF_PIC32INT)
    {TCPIP_MODULE_MAC_PIC32INT, TCPIP_STACK_IF_NAME_PIC32INT, (tcpipMacInitFunc)DRV_ETHMAC_PIC32MACSetup,         DRV_ETHMAC_PIC32MACTeardown,       DRV_ETHMAC_PIC32MACOpen},          // TCPIP_MODULE_MAC_PIC32INT
#endif
#if defined(TCPIP_IF_MRF24W)
    {TCPIP_MODULE_MAC_MRF24W,   TCPIP_STACK_IF_NAME_MRF24W,   (tcpipMacInitFunc)MRF24W_MACInitialize,       MRF24W_MACDeinitialize,     MRF24W_MACOpen},        // TCPIP_MODULE_MAC_MRF24W
#endif
};



// Connection event handler definition.
// The stack calls the handler when a new connection event occurs.
// Note that this call will carry only connection events!
typedef void    (*tcpipModuleConnHandler)(TCPIP_NET_IF* pNetIf, TCPIP_MAC_EVENT connEvent);



// Since the modules that need connection notification is
// known to the stack manager no dynamic approach is taken.
// But simply a call table is maintained.
static const tcpipModuleConnHandler  TCPIP_STACK_CONN_EVENT_TBL [] =
{
#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
    TCPIP_DHCP_ConnectionHandler,
#endif // defined(TCPIP_STACK_USE_DHCP_CLIENT)

    // add other needed handlers here
};



#endif //  __TCPIP_MODULE_MANAGER_H_








