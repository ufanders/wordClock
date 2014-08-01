/*******************************************************************************
  MRF24W Driver Event Handler

  File Name:  
    drv_wifi_event_handler.c  
  
  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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


/*==========================================================================*/
/*                                  INCLUDES                                */
/*==========================================================================*/
#include "tcpip/src/tcpip_private.h"

#if defined(TCPIP_IF_MRF24W)

/*==========================================================================*/
/*                                  DEFINES                                 */
/*==========================================================================*/

/*-------------------------------------------*/
/* Connection Manager Event Message Subtypes */
/* (Used in Mgmt Indicate messages)          */
/*-------------------------------------------*/
#define WF_EVENT_CONNECTION_ATTEMPT_STATUS_SUBTYPE   (6)  
#define WF_EVENT_CONNECTION_LOST_SUBTYPE             (7)
#define WF_EVENT_CONNECTION_REESTABLISHED_SUBTYPE    (8)
#define WF_EVENT_KEY_CALCULATION_REQUEST_SUBTYPE     (9)
#define WF_EVENT_SCAN_RESULTS_READY_SUBTYPE          (11)
#define WF_EVENT_SCAN_IE_RESULTS_READY_SUBTYPE       (12)
#define WF_EVENT_SOFT_AP_EVENT_SUBTYPE               (13)
#define WF_EVENT_DISCONNECT_DONE_SUBTYPE             (14)

/* event values for index 2 of WF_CONNECTION_ATTEMPT_STATUS_EVENT_SUBTYPE */
#define CONNECTION_ATTEMPT_SUCCESSFUL    ((uint8_t)1)   /* if not 1 then failed to connect and info field is error code */
#define CONNECTION_ATTEMPT_FAILED        ((uint8_t)2)

/* event values for index 2 of WF_EVENT_CONNECTION_LOST_SUBTYPE */
#define CONNECTION_TEMPORARILY_LOST      ((uint8_t)1)
#define CONNECTION_PERMANENTLY_LOST      ((uint8_t)2)
#define CONNECTION_REESTABLISHED         ((uint8_t)3)   

#define isMgmtRxPending()                ((ASYNC_MGMT_RX_PENDING     & g_asyncTaskFlags) == ASYNC_MGMT_RX_PENDING)
#if (WF_GRATUITOUS_ARP == DRV_WIFI_ENABLED)
#define isAsyncGratArpPending()          ((ASYNC_GRAT_ARP_PENDING    & g_asyncTaskFlags) == ASYNC_GRAT_ARP_PENDING)
#endif
#define isAsyncEasyConfigPending()       ((ASYNC_EASY_CONFIG_PENDING & g_asyncTaskFlags) == ASYNC_EASY_CONFIG_PENDING)
#define isAsyncDhcpPending()             ((ASYNC_DHCP_CONFIG_PENDING & g_asyncTaskFlags) == ASYNC_DHCP_CONFIG_PENDING)
#define isAsyncPowerSavePending()        ((ASYNC_POWER_SAVE_PENDING  & g_asyncTaskFlags) == ASYNC_POWER_SAVE_PENDING)
#define isAsyncWpaConnectPending()       ((ASYNC_WPABUTTON_CONNECT   & g_asyncTaskFlags) == ASYNC_WPABUTTON_CONNECT)

/*==========================================================================*/
/*                                  GLOBALS                                 */
/*==========================================================================*/
bool g_DhcpRenew = false;
volatile uint8_t g_asyncTaskFlags;

DRV_WIFI_MGMT_INDICATE_SOFT_AP_EVENT g_softAPEvent;

tMgmtIndicatePassphraseReady passphraseReady;

/*==========================================================================*/
/*                                  LOCAL FUNCTIONS                         */
/*==========================================================================*/
static void WifiAsyncClearEventPending(uint8_t event);

#if defined(TCPIP_STACK_USE_IPV6)
static void WF_Initialize_IPV6_Multicast_Filter(void);
#endif

#if defined (TCPIP_STACK_USE_DHCP_CLIENT)
    static void RenewDhcp(void);
#endif

/*****************************************************************************
 * FUNCTION: WFProcessMgmtIndicateMsg
 *
 * RETURNS:  error code
 *
 * PARAMS:   None
 *
 *  NOTES:   Processes a management indicate message
 *****************************************************************************/
void WFProcessMgmtIndicateMsg()
{
    tMgmtIndicateHdr  hdr;
    uint8_t buf[6];
    uint8_t event = 0xff;
    uint16_t eventInfo;

    // read mgmt indicate header (2 bytes)
    RawRead(RAW_SCRATCH_ID, MGMT_INDICATE_BASE, sizeof(tMgmtIndicateHdr), (uint8_t *)&hdr);

    /* if not a management indicate then fatal error */
    SYS_ASSERT((hdr.type == WF_MGMT_INDICATE_TYPE), "Invalid Indicate Header" );
        
    /* Determine which event occurred and handle it */
    switch (hdr.subType)
    {
        /*-----------------------------------------------------------------*/        
        case WF_EVENT_CONNECTION_ATTEMPT_STATUS_SUBTYPE:
        /*-----------------------------------------------------------------*/
            RawReadRelative(RAW_SCRATCH_ID, 2, buf); /* read first 2 bytes after header */
            /* if connection attempt successful */
            if (buf[0] == CONNECTION_ATTEMPT_SUCCESSFUL)
            {
                event     = DRV_WIFI_EVENT_CONNECTION_SUCCESSFUL;
                eventInfo = DRV_WIFI_NO_ADDITIONAL_INFO;
                SignalWiFiConnectionChanged(true);
                #if defined(TCPIP_STACK_USE_IPV6)
                    WF_Initialize_IPV6_Multicast_Filter();
                #endif
                #if defined (TCPIP_STACK_USE_DHCP_CLIENT)
                    RenewDhcp();
                #endif
                SetLogicalConnectionState(true);
            }
            /* else connection attempt failed */
            else
            {
                event     = DRV_WIFI_EVENT_CONNECTION_FAILED;
                eventInfo = (uint16_t)(buf[0] << 8 | buf[1]);   /* contains connection failure code */
                SetLogicalConnectionState(false);
            }
            break;
            
        /*-----------------------------------------------------------------*/
        case WF_EVENT_CONNECTION_LOST_SUBTYPE:
        /*-----------------------------------------------------------------*/ 
            /* read next two data bytes in message
               buf[0] -- 1: Connection temporarily lost  2: Connection permanently lost 3: Connection Reestablished 
               buf[1] -- 0: Beacon Timeout  1: Deauth from AP  */
            RawReadRelative(RAW_SCRATCH_ID, 2, buf);

            if (buf[0] == CONNECTION_TEMPORARILY_LOST)
            {
                event     = DRV_WIFI_EVENT_CONNECTION_TEMPORARILY_LOST;
                eventInfo = (uint16_t)buf[1];    /* lost due to beacon timeout or deauth */
                SignalWiFiConnectionChanged(false);
                
                SetLogicalConnectionState(false);
            }    
            else if (buf[0] == CONNECTION_PERMANENTLY_LOST)
            {
                event     = DRV_WIFI_EVENT_CONNECTION_PERMANENTLY_LOST;
                eventInfo = (uint16_t)buf[1];   /* lost due to beacon timeout or deauth */
                SetLogicalConnectionState(false);
                SignalWiFiConnectionChanged(false);                
            }
            else if (buf[0] == CONNECTION_REESTABLISHED)
            {
                event     = DRV_WIFI_EVENT_CONNECTION_REESTABLISHED;
                eventInfo = (uint16_t)buf[1];    /* originally lost due to beacon timeout or deauth */
                #if defined(TCPIP_STACK_USE_DHCP_CLIENT)
                RenewDhcp();
                #endif
                SignalWiFiConnectionChanged(true);  
                
                SetLogicalConnectionState(true);
            }    
            else
            {
                /* invalid parameter in message */
                SYS_ASSERT(false, "");
            }        
            break;
        
        /*-----------------------------------------------------------------*/                    
        case WF_EVENT_SCAN_RESULTS_READY_SUBTYPE:        
        /*-----------------------------------------------------------------*/
            RawReadRelative(RAW_SCRATCH_ID, 1, buf);
            event = DRV_WIFI_EVENT_SCAN_RESULTS_READY;
            eventInfo = (uint16_t)buf[0];          /* number of scan results */
            break;
            
        /*-----------------------------------------------------------------*/
        case WF_EVENT_SCAN_IE_RESULTS_READY_SUBTYPE:
        /*-----------------------------------------------------------------*/
            event = DRV_WIFI_EVENT_IE_RESULTS_READY;
            /* read indexes 2 and 3 containing the 16-bit value of IE bytes */
            RawReadRelative(RAW_SCRATCH_ID, 2, (uint8_t *)&eventInfo);
            eventInfo = TCPIP_Helper_ntohs(eventInfo);     /* fix endianess of 16-bit value */
            break;    
        
        /*-----------------------------------------------------------------*/
        case WF_EVENT_KEY_CALCULATION_REQUEST_SUBTYPE:
        /*-----------------------------------------------------------------*/
            event = DRV_WIFI_EVENT_KEY_CALCULATION_REQUEST;
            RawReadRelative(RAW_SCRATCH_ID, sizeof(tMgmtIndicatePassphraseReady), (uint8_t *)&passphraseReady);
            WifiAsyncSetEventPending(ASYNC_WPABUTTON_CONNECT);
            break;

        /*-----------------------------------------------------------------*/
        case WF_EVENT_SOFT_AP_EVENT_SUBTYPE:    /* Valid only with 3108 or the later module FW version */
        /*-----------------------------------------------------------------*/
            event = DRV_WIFI_EVENT_SOFT_AP;
            RawReadRelative(RAW_SCRATCH_ID, sizeof(DRV_WIFI_MGMT_INDICATE_SOFT_AP_EVENT), (uint8_t *)&g_softAPEvent);
            break;
            
        /*-----------------------------------------------------------------*/
        case WF_EVENT_DISCONNECT_DONE_SUBTYPE:
        /*-----------------------------------------------------------------*/
            event =  DRV_WIFI_EVENT_DISCONNECT_DONE;
            /* set state to no connection */
            SetLogicalConnectionState(false);
            break;
            
        /*-----------------------------------------------------------------*/
        default:
        /*-----------------------------------------------------------------*/
            SYS_ASSERT(false, "");
            break;        
    }

    /* if the application wants to be notified of the event */
    WF_UserEventsSet(event, eventInfo, 1);
}

#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
static void RenewDhcp(void)
{
    g_DhcpRenew = true;
    SetDhcpProgressState();
}    
#endif


/*******************************************************************************
  Function:
    void DRV_WIFI_SoftApEventInfoGet(DRV_WIFI_MGMT_INDICATE_SOFT_AP_EVENT *p_eventInfo);

  Summary:
    Gets the stored Soft AP event info

  Description:
    This function retrieves the additional event info after a Soft AP event has
    occurred.

  Parameters:
    p_event -- pointer to where event info is written.  See
               DRV_WIFI_MGMT_INDICATE_SOFT_AP_EVENT structure.

  Returns:
    None.

  Remarks:
    None.
  *****************************************************************************/
DRV_WIFI_MGMT_INDICATE_SOFT_AP_EVENT * DRV_WIFI_SoftApEventInfoGet(void)
{
    return &g_softAPEvent;
}


// callback function; stack calls this upon DHCP events
void DhcpEventHandler(TCPIP_NET_HANDLE hNet, TCPIP_DHCP_EVENT_TYPE evType, const void* param)
{
    if (evType == DHCP_EVENT_BOUND)
    {
        SignalDHCPSuccessful();
    }
}

void WifiAsyncClearAllEvents(void)
{
    g_asyncTaskFlags = 0x00;
}

// sets an event bit
void WifiAsyncSetEventPending(uint8_t event)
{
    bool intEnabled = SYS_INT_SourceDisable(MRFWB0M_INT_SOURCE);
    
    g_asyncTaskFlags |= event;

    if(intEnabled)
    {
        DRV_WIFI_INT_SourceEnable();
    }
}

static void WifiAsyncClearEventPending(uint8_t event)
{
    bool intEnabled = SYS_INT_SourceDisable(MRFWB0M_INT_SOURCE);

    g_asyncTaskFlags &= ~event;

    if(intEnabled)
    {
        DRV_WIFI_INT_SourceEnable();
    }
}

// called by TCPIP_STACK_Task() periodically to check if WiFi async event has occurred
bool WiFiAsyncTaskPending(void)
{
    if ((GetRxPendingCount() > 0) || (g_asyncTaskFlags > 0))
    {
        return true;
    }
    else 
    {
        return false;
    }
}

void WaitMgmtResponse(void)
{
    if (isMgmtRxPending())
    {
        WifiAsyncClearEventPending(ASYNC_MGMT_RX_PENDING);
        MgmtRxTask();
    }
}



// called by TCPIP_STACK_Task() if any WiFi event is pending
void WiFiAsyncTask(void)
{
    if (isMgmtRxPending())
    {
        WifiAsyncClearEventPending(ASYNC_MGMT_RX_PENDING);
        MgmtRxTask();
    }

    #if (WF_GRATUITOUS_ARP == DRV_WIFI_ENABLED)
    if (isAsyncGratArpPending())
    {
        WifiAsyncClearEventPending(ASYNC_GRAT_ARP_PENDING);
        WiFi_GratuitousArpTask();
    }
    #endif

    if (isAsyncEasyConfigPending())
    {
        WifiAsyncClearEventPending(ASYNC_EASY_CONFIG_PENDING);
        WiFi_EasyConfigTask();
    }

    if (isAsyncDhcpPending())
    {
        WifiAsyncClearEventPending(ASYNC_DHCP_CONFIG_PENDING);
        DhcpConfigTask();
    }

    if (isAsyncPowerSavePending())
    {
        WifiAsyncClearEventPending(ASYNC_POWER_SAVE_PENDING);
        PowerSaveTask();
    }

    if(isAsyncWpaConnectPending())
    {
        WifiAsyncClearEventPending(ASYNC_WPABUTTON_CONNECT);
        WifiAsyncWpaButtonConnectTask();
    }
}

/*****************************************************************************
 * FUNCTION: WF_Initialize_IPV6_Multicast_Filter
 *
 * RETURNS:  None
 *
 * PARAMS:   None
 *
 *  NOTES:   Initialize IPV6 multicast filter which is specific to
 *           the Microchip Demo code.
 *****************************************************************************/
#if defined(TCPIP_STACK_USE_IPV6)
static void WF_Initialize_IPV6_Multicast_Filter(void)
{
    TCPIP_NET_IF* pNetIf;
    TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandleGet("MRF24W");
    uint8_t solicitedNodeMulticastMACAddr[] = {0x33, 0x33, 0xff, 0x00, 0x00, 0x00};
    uint8_t allNodesMulticastMACAddr[] = {0x33, 0x33, 0x00, 0x00, 0x00, 0x01};
    uint8_t i;

    pNetIf =  (TCPIP_NET_IF*)netH;

    for (i =0; i < 3; i++)
    {
        solicitedNodeMulticastMACAddr[3 + i] = pNetIf->netMACAddr.v[3 + i];
    }

    #if (WF_SOFTWARE_MULTICAST_FILTER ==  DRV_WIFI_ENABLED)
        DRV_WIFI_SWMultiCastFilterEnable();
        do {
            DRV_WIFI_SWMULTICAST_CONFIG p_config;

            p_config.filterId = DRV_WIFI_MULTICAST_FILTER_1;
            p_config.action = DRV_WIFI_MULTICAST_USE_FILTERS;
            memcpy((void *)p_config.macBytes, (void *)solicitedNodeMulticastMACAddr, WF_MAC_ADDRESS_LENGTH);
            p_config.macBitMask = 0x3F;
            DRV_WIFI_SWMulticastFilterSet(&p_config);

            p_config.filterId = DRV_WIFI_MULTICAST_FILTER_2;
            p_config.action = DRV_WIFI_MULTICAST_USE_FILTERS;
            memcpy((void *)p_config.macBytes, (void *)allNodesMulticastMACAddr, WF_MAC_ADDRESS_LENGTH);
            p_config.macBitMask = 0x3F;
            DRV_WIFI_SWMulticastFilterSet(&p_config);
        } while (0);

    #else // using HW multicast flters
        DRV_WIFI_HWMulticastFilterSet(DRV_WIFI_MULTICAST_FILTER_1, solicitedNodeMulticastMACAddr);
        DRV_WIFI_HWMulticastFilterSet(DRV_WIFI_MULTICAST_FILTER_2, allNodesMulticastMACAddr);
    #endif 
}
#endif /* WF_Initialize_IPV6_Multicast_Filter */

#endif /* TCPIP_IF_MRF24W */


//DOM-IGNORE-END
