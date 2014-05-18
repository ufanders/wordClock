/*******************************************************************************
  MRF24W Driver

  File Name:  
    drv_easy_config.c  
  
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



#include "tcpip/src/tcpip_private.h"




#if defined(TCPIP_IF_MRF24W)
#include <ctype.h>
#include "../framework/driver/wifi/mrf24w/src/drv_wifi_easy_config.h"

DRV_WIFI_EZ_CONFIG_SCAN_CONTEXT  g_ScanCtx;

t_wfEasyConfigCtx g_easyConfigCtx;
#if (WF_DEFAULT_NETWORK_TYPE == DRV_WIFI_NETWORK_TYPE_SOFT_AP) || (DRV_WIFI_DEFAULT_ADHOC_PRESCAN == DRV_WIFI_ENABLED)
DRV_WIFI_SCAN_RESULT preScanResult[50];      //WF_PRESCAN  May change struct later for memory optimization
#endif

/* Easy Config Private Functions */
static void EasyConfigConnect(void);


static void EasyConfigTimerHandler();

typedef enum
{
    EZ_WAIT_FOR_START = 0,
    EZ_WAIT_FOR_DELAY = 1,
} t_EasyConfigStates;

void WFEasyConfigInit(void)
{
    EZ_CFGCXT.ssid[0] = 0;
    EZ_CFGCXT.security = DRV_WIFI_SECURITY_OPEN;
    EZ_CFGCXT.key[0] = 0;
    EZ_CFGCXT.defaultWepKey = WF_WEP_KEY_INVALID;
    EZ_CFGCXT.type = DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE;
    EZ_CFGCXT.state = EZ_WAIT_FOR_START;
    EZ_CFGCXT.isWifiNeedToConfigure = false;
}


// called from a system async event handler whenever g_easyConfigCtx.isWifiNeedToConfigure is true
void WiFi_EasyConfigTask(void)
{
    switch (g_easyConfigCtx.state)
    {
        case EZ_WAIT_FOR_START:
            g_easyConfigCtx.isWifiNeedToConfigure = false;
            // first thing to do is delay one second so user can see on the web
            // page that disconnect is about occur in process of connecting to
            // designated AP.  So create and start timer.
            g_easyConfigCtx.timer = SYS_TMR_CallbackSingle(1000 * WF_EASY_CONFIG_DELAY_TIME, EasyConfigTimerHandler);

            g_easyConfigCtx.state = EZ_WAIT_FOR_DELAY;
            break;

        case EZ_WAIT_FOR_DELAY:
            g_easyConfigCtx.isWifiNeedToConfigure = false;
            
            // connect to AP that user selected on web page
            EasyConfigConnect();
            g_easyConfigCtx.state = EZ_WAIT_FOR_START;

            break;
    }
}

static void EasyConfigTimerHandler(void) 
{
    //it is SYS_TMR_CallbackSingle, only once, we don't have to stop it
    //SYS_TMR_CallbackStop(g_easyConfigCtx.timer);

    // This function being called from timer interrupt, so don't do any work
    // here, but, schedule the async event handler to call EasyConfigStateMachine
    g_easyConfigCtx.isWifiNeedToConfigure = true;

    WifiAsyncSetEventPending(ASYNC_EASY_CONFIG_PENDING);
}


static void EasyConfigConnect(void)
{
    // ToDo: here we let Adhoc work as Software mode, reset WiFi module,re-initialize it, then connect to AP
    #if 1 //WF_DEFAULT_NETWORK_TYPE == DRV_WIFI_NETWORK_TYPE_SOFT_AP
        // SoftAP: To allow redirection, need to hibernate before changing network type. Module
        //         FW has SoftAP flag and therefore hibernate mode is needed to clear this
        //         indication and allow proper network change. This should work for non-SoftAP,
        //         but these have not been tested yet.
        int ret_init;
        WF_InitForSoftApReDirection_enable();
        do
        {
            ret_init = WF_InitForSoftApReDirection();
        } while(ret_init == TCPIP_MAC_RES_PENDING);
    #else
        DRV_WIFI_Disconnect();    // break the connection supporting the web page

        if (g_easyConfigCtx.ssid)    // if AP has an SSID (not a hidden SSID)
        {
            DRV_WIFI_SsidSet(g_easyConfigCtx.ssid, strlen((char*)g_easyConfigCtx.ssid));
        }

        // Set security supported by designated AP
        EasyConfigSetSecurity();

        DRV_WIFI_ConfigDataSave();

        /* Set wlan mode */
        DRV_WIFI_NetworkTypeSet(EZ_CFGCXT.type);

        #if  (WF_MODULE_CONNECTION_MANAGER == DRV_WIFI_DISABLED)
            DRV_WIFI_ReconnectModeSet(0,                                    // report-only when connection lost (no reconnect)
                                     DRV_WIFI_DO_NOT_ATTEMPT_TO_RECONNECT,  // report-only when deauth received (no reconnect)
                                     40,                                    // set beacon timeout to 40 beacon periods
                                     DRV_WIFI_DO_NOT_ATTEMPT_TO_RECONNECT); // report only when beacon timeout occurs
        #else
            //TCPIP_NET_IF* p_config= (TCPIP_NET_IF*)GetNetworkConfig();
            if (p_wifi_ConfigData->networkType == DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE)
            {
                DRV_WIFI_ReconnectModeSet(DRV_WIFI_RETRY_FOREVER,         // retry forever to connect to WiFi network
                                          DRV_WIFI_ATTEMPT_TO_RECONNECT,  // reconnect on deauth from AP
                                          40,                             // beacon timeout is 40 beacon periods
                                          DRV_WIFI_ATTEMPT_TO_RECONNECT); // reconnect on beacon timeout
            }
        #endif
        /* Kick off connection now... */
        {  //delay a short time
            int i,j;
            for(i=0;i<1000;i++)
            for(j=0;j<1000;j++)
                        ;
        }
        DRV_WIFI_Connect();
        TCPIP_DHCPS_Disable( TCPIP_STACK_NetHandleGet("MRF24W"));
        TCPIP_DHCP_Enable( TCPIP_STACK_NetHandleGet("MRF24W"));
    #endif
}


bool WiFi_EasyConfigTaskPending(void)
{
    return g_easyConfigCtx.isWifiNeedToConfigure;
}



void WFInitScan(void)
{
    SCANCXT.scanState = 0;
    SCANCXT.numScanResults = 0;
    SCANCXT.displayIdx = 0;
}

uint16_t WFStartScan(void)
{
    /* If scan already in progress bail out */
    if (IS_SCAN_IN_PROGRESS(SCANCXT.scanState))
    {
        return DRV_WIFI_ERROR_OPERATION_CANCELLED;
    }
    if (DRV_WIFI_Scan(true) != DRV_WIFI_SUCCESS)
    {
        return DRV_WIFI_ERROR_OPERATION_CANCELLED;
    }
   SCAN_SET_IN_PROGRESS(SCANCXT.scanState);

   return DRV_WIFI_SUCCESS;
}

uint16_t WFRetrieveScanResult(uint8_t Idx, DRV_WIFI_SCAN_RESULT *p_ScanResult)
{
    if (Idx >= SCANCXT.numScanResults)
    {
        return DRV_WIFI_ERROR_INVALID_PARAM;
    }

    DRV_WIFI_ScanGetResult(Idx, p_ScanResult);
    //p_ScanResult->ssid[p_ScanResult->ssidLen] = 0; /* Terminate */

    return DRV_WIFI_SUCCESS;
}

void WFScanEventHandler(uint16_t scanResults)
{
    /* Cache number APs found in scan */
    SCANCXT.numScanResults = scanResults;

    /* Clear the scan in progress */
    SCAN_CLEAR_IN_PROGRESS(SCANCXT.scanState);
    SCAN_SET_VALID(SCANCXT.scanState);
}

void WFDisplayScanMgr(void)
{
    DRV_WIFI_SCAN_RESULT   bssDesc;
    char ssid[32+1];
    char buf[10];
    int i;
    char rssiChan[48];

    if ((SCANCXT.numScanResults == 0)               ||
        (!IS_SCAN_STATE_DISPLAY(SCANCXT.scanState)) ||
        (IS_SCAN_IN_PROGRESS(SCANCXT.scanState))    ||
        (!IS_SCAN_STATE_VALID(SCANCXT.scanState)))
    {
       SCAN_CLEAR_DISPLAY(SCANCXT.scanState); 
       return;
    }

    WFRetrieveScanResult(SCANCXT.displayIdx, &bssDesc);

    memset(ssid, ' ', sizeof(ssid));

    /* Display SSID */
    for(i = 0; i < bssDesc.ssidLen; i++)
    {
        if (!isprint(bssDesc.ssid[i]))
        {
            ssid[i] = '*';
        }
        else
        {
            ssid[i] = bssDesc.ssid[i];
        }


    }
    ssid[32] = 0;

   
    sprintf(buf,"%2d)  ",SCANCXT.displayIdx+1);
    SYS_CONSOLE_MESSAGE(buf);
    SYS_CONSOLE_MESSAGE(ssid);
    
    /* Display SSID  & Channel */
    /* RSSI_MAX : 200, RSSI_MIN : 106 */
    sprintf(rssiChan, " %2u    %u\r\n", bssDesc.rssi, bssDesc.channel);
    SYS_CONSOLE_MESSAGE(rssiChan);

    #if (WF_DEFAULT_NETWORK_TYPE == DRV_WIFI_NETWORK_TYPE_SOFT_AP) || (DRV_WIFI_DEFAULT_ADHOC_PRESCAN == DRV_WIFI_ENABLED)
        preScanResult[SCANCXT.displayIdx]= bssDesc;    // WF_PRESCAN
        if (SCANCXT.displayIdx == sizeof(preScanResult) / sizeof(preScanResult[0]) - 1)
        {
            SCAN_CLEAR_DISPLAY(SCANCXT.scanState);
            SCANCXT.displayIdx = 0;
        }
    #endif

    if (++SCANCXT.displayIdx == SCANCXT.numScanResults)  
    {
        SCAN_CLEAR_DISPLAY(SCANCXT.scanState);
        SCANCXT.displayIdx = 0;
    }
}

#endif /* TCPIP_IF_MRF24W */

//DOM-IGNORE-END
 