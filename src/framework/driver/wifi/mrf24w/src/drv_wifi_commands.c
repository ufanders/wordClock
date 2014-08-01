/*******************************************************************************
  MRF24W commands(based on system commander) implementation

  File Name:  
    drv_wifi_commands.c  
  
  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright � 2012 released Microchip Technology Inc. All rights reserved.

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


//==============================================================================
//                                  Includes
//==============================================================================

#include "tcpip/src/tcpip_private.h"
#include <ctype.h>
#include "drv_wifi_config.h"

#if defined( TCPIP_STACK_COMMANDS_WIFI_ENABLE )

// iwconfig control block
typedef struct
{
    uint8_t powerSaveState;  // power save state
    uint8_t connState;       // connection state
    bool    isIdle;          // true if connState is DRV_WIFI_CSTATE_NOT_CONNECTED
} t_wfIwconfigCb;

//==============================================================================
//                                  Constants
//==============================================================================


//==============================================================================
//                                  Local Globals
//==============================================================================
static t_wfIwconfigCb iwconfigCb;
static bool iwconfigCbInitialized = false;

//==============================================================================
//                                  Local Function Prototypes
//==============================================================================
static bool iwconfigUpdateCb(_CMDIO_DEV_NODE* pCmdIO);
static void iwconfigDisplayStatus(_CMDIO_DEV_NODE* pCmdIO);
static bool iwconfigSetSsid(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static bool iwconfigSetMode(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static bool iwconfigSetChannel(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static bool iwconfigSetPower(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static bool iwconfigSetRTS(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int  CommandIwConfig(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int  CommandIw_readConfig(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int  CommandIw_eraseConfig(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
static int  CommandIw_saveConfig(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);

int Command_nvm_info(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);

 int Command_nvm_readall(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
 int Command_nvm_read(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);

 int Command_nvm_write1(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);
 int Command_nvm_write2(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv);

extern bool wf_update_begin_uart;
bool wf_update_begin__tcp = false;
int Command_iw_update_tcp(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    wf_update_begin__tcp = true;
    WF_UserEventsSet(DRV_WIFI_UPDATE, 0, false);
    return true;
}
int Command_iw_update_uart(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    wf_update_begin_uart = true;
    WF_UserEventsSet(DRV_WIFI_UPDATE, 0, false);
    return true;
}

static void DisplayScanResult(_CMDIO_DEV_NODE* pCmdIO, uint16_t index, DRV_WIFI_SCAN_RESULT *p_scanResult);

// command table (placed here because needs above function prototypes)
static const _SYS_CMD_DCPT    wfCmdTbl[]=
{
    {"iwconfig",     CommandIwConfig,         ": Wifi mode config"},
    {"iw_readconf",  CommandIw_readConfig,    ": read file from storage"},
    {"iw_eraseconf", CommandIw_eraseConfig,   ": erase storage"},
    {"iw_saveconf",  CommandIw_saveConfig,    ": save storage"},
    
    {"nvm_readall",     Command_nvm_readall,        ": nvm_readall"},
    {"iw_update_tcp",  Command_iw_update_tcp,        ": iw_update_tcp"},
    {"iw_update_uart",  Command_iw_update_uart,        ": iw_update_uart"},
 #if 0  //only for test
    {"nvm_read",     Command_nvm_read,        ": nvm_read"},
    {"nvm_write1",     Command_nvm_write1,    ": nvm_write1"},
    {"nvm_write2",     Command_nvm_write2,    ": nvm_write2"},
    {"nvm_info",     Command_nvm_info,        ": nvm_info"},
#endif

};

bool WIFICommandsInit(void)
{
    if (_SYS_COMMAND_ADDGRP(wfCmdTbl, sizeof(wfCmdTbl)/sizeof(*wfCmdTbl), "wifi", ": Wifi commands") == -1) 
    {
        return false;
    }
    
    return true;
}

static int CommandIwConfig(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    uint16_t scanIndex;
    DRV_WIFI_SCAN_RESULT scanResult;

    if (DRV_WIFI_isHibernateEnable())
    {
        SYS_CONSOLE_MESSAGE("The Wi-Fi module is in hibernate mode - command failed.\r\n");
        return false;
    }

    if ( !iwconfigUpdateCb(pCmdIO) )
    {
        SYS_CONSOLE_MESSAGE("iwconfigCb structure set failed\r\n");
        return false;
    }

    // if user only typed in iwconfig with no other parameters
    if (argc == 1)
    {
        iwconfigDisplayStatus(pCmdIO);
        return true;
    }

    if ( (2 <= argc) && (strcmp((char*)argv[1], "ssid") == 0) )
    {
        return iwconfigSetSsid(pCmdIO, argc, argv);
    }
    else if ( (2 <= argc) && (strcmp((char*)argv[1], "mode") == 0) )
    {
       return iwconfigSetMode(pCmdIO, argc, argv);
    }
    else if ( (2 <= argc) && (strcmp((char*)argv[1], "channel") == 0) )
    {
        return iwconfigSetChannel(pCmdIO, argc, argv);
    }
    else if ( (2 <= argc) && (strcmp((char*)argv[1], "power") == 0) )
    {
        return iwconfigSetPower(pCmdIO, argc, argv);
    }
    else if ( (2 <= argc) && (strcmp((char*)argv[1], "rts") == 0) )
    {
        return iwconfigSetRTS(pCmdIO, argc, argv);
    }
    else if ( (2 == argc) && (strcmp((char*)argv[1], "scan") == 0) )
    {
        if (WFStartScan() == DRV_WIFI_SUCCESS)
        {
            SYS_CONSOLE_MESSAGE("Scan started ...\r\n");
            SCAN_SET_DISPLAY(SCANCXT.scanState);
            SCANCXT.displayIdx = 0;
        }
        else
        {
            SYS_CONSOLE_MESSAGE("Scan failed.\r\n");
        }
        return false;
    }
    else if ( (2 <= argc) && (strcmp((char*)argv[1], "scanget") == 0) )
    {
        if (argc != 3)
        {
            SYS_CONSOLE_MESSAGE("Need scan index to display (1 - N)\r\n");
            return false;
        }

        if (g_ScanCtx.numScanResults == 0)
        {
            SYS_CONSOLE_MESSAGE("No scan results to display\r\n");
            return false;
        }

        scanIndex = atoi((char *)argv[2]);
        if ((scanIndex > 0) && (scanIndex <= g_ScanCtx.numScanResults))
        {
            DRV_WIFI_ScanGetResult(scanIndex-1, &scanResult);
            DisplayScanResult(pCmdIO, scanIndex-1, &scanResult);
        }
        else
        {
            SYS_CONSOLE_PRINT("Scan index must be between 1 and %d\r\n", g_ScanCtx.numScanResults);
            return false;
        }
    }
    else    
    {
        SYS_CONSOLE_MESSAGE("Unknown parameter\r\n");
        return false;
    }

    return true;
}


//==============================================================================
//                            Local Function prototype for iwconfig
//==============================================================================


/*****************************************************************************
 * FUNCTION: iwconfigSetCb
 *
 * RETURNS: true or false
 *
 * PARAMS:  None
 *
 * NOTES:   Updates the iwconfigCb structure
 *****************************************************************************/
static bool iwconfigUpdateCb(_CMDIO_DEV_NODE* pCmdIO)
{
    if ( !iwconfigCbInitialized ) // first time call of iwconfigUpdateCb
    {
        memset(&iwconfigCb, 0, sizeof(iwconfigCb));
        iwconfigCbInitialized = true;
    }

    DRV_WIFI_PowerSaveStateGet(&iwconfigCb.powerSaveState);
    if (iwconfigCb.powerSaveState == DRV_WIFI_PS_HIBERNATE)
    {
        SYS_CONSOLE_MESSAGE("WF device hibernated\r\n");
        return false;
    }

    DRV_WIFI_ConnectionStateGet(&iwconfigCb.connState);

    if ((iwconfigCb.connState == DRV_WIFI_CSTATE_NOT_CONNECTED) || (iwconfigCb.connState == DRV_WIFI_CSTATE_CONNECTION_PERMANENTLY_LOST))
    {
        iwconfigCb.isIdle = true;
    }    
    else
    {
        iwconfigCb.isIdle = false;
    }    

    return true;
}

/*****************************************************************************
 * FUNCTION: iwconfigDisplayStatus
 *
 * RETURNS:    None
 *
 * PARAMS:    None
 *
 * NOTES:    Responds to the user invoking iwconfig with no parameters
 *****************************************************************************/
static void iwconfigDisplayStatus(_CMDIO_DEV_NODE* pCmdIO)
{
    uint8_t *p;
    uint8_t tmp;
    uint8_t connectionState;

    union
    {
        struct
        {
            uint8_t List[DRV_WIFI_MAX_CHANNEL_LIST_LENGTH];
            uint8_t Num;
        } Channel;

        uint8_t Domain;

        struct
        {
            uint8_t String[DRV_WIFI_MAX_SSID_LENGTH+1];
            uint8_t Len;
        } Ssid;

        struct
        {
            uint8_t NetworkType;
        } Mode;

        struct
        {
            uint16_t Threshold;
        } Rts;
    } ws; // workspace

    // channel
    DRV_WIFI_ChannelListGet(ws.Channel.List, &ws.Channel.Num);
    SYS_CONSOLE_MESSAGE("\tchannel:  ");
    p = ws.Channel.List;
    tmp = ws.Channel.Num;
    while ( --tmp > 0u )
    {
        SYS_CONSOLE_PRINT("%d,", *p);
        p++;
    }
    SYS_CONSOLE_PRINT("%d\r\n", *p);

    // domain
    DRV_WIFI_RegionalDomainGet(&ws.Domain);
    SYS_CONSOLE_MESSAGE("\tdomain:   ");
    if ( ws.Domain == DRV_WIFI_DOMAIN_FCC )
    {
        SYS_CONSOLE_MESSAGE("FCC");
    }
    else if ( ws.Domain == DRV_WIFI_DOMAIN_ETSI )
    {
        SYS_CONSOLE_MESSAGE("ETSI");
    }
    else if ( ws.Domain == DRV_WIFI_DOMAIN_JAPAN )
    {
        SYS_CONSOLE_MESSAGE("JAPAN or OTHER");
    }
    else
    {
        SYS_CONSOLE_MESSAGE("unknown");
    }
    SYS_CONSOLE_MESSAGE("\r\n");


    // rts
    DRV_WIFI_RtsThresholdGet(&ws.Rts.Threshold);
    SYS_CONSOLE_PRINT("\trts:      %d\r\n", ws.Rts.Threshold);

    // mode
    DRV_WIFI_ConnectionStateGet(&connectionState);
    DRV_WIFI_NetworkTypeGet(&ws.Mode.NetworkType);
    SYS_CONSOLE_MESSAGE("\tmode:     ");
    if (iwconfigCb.isIdle)
    {
        if (iwconfigCb.connState == DRV_WIFI_CSTATE_NOT_CONNECTED)
        {
            SYS_CONSOLE_MESSAGE("Idle");
        }
        else if (iwconfigCb.connState == DRV_WIFI_CSTATE_CONNECTION_PERMANENTLY_LOST)
        {
            SYS_CONSOLE_MESSAGE("Idle (connection permanently lost)");
        }
        else
        {
            SYS_CONSOLE_MESSAGE("idle (?)");
        } 
    }
    else
    {
        DRV_WIFI_NetworkTypeGet(&ws.Mode.NetworkType);
        if (ws.Mode.NetworkType == DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE)
        {
            if (iwconfigCb.connState == DRV_WIFI_CSTATE_CONNECTION_IN_PROGRESS)
            {
                SYS_CONSOLE_MESSAGE("Managed (connection in progress)");
            }
            else if (iwconfigCb.connState == DRV_WIFI_CSTATE_CONNECTED_INFRASTRUCTURE)
            {
                SYS_CONSOLE_MESSAGE("Managed (connected)");
            }
            else if (iwconfigCb.connState == DRV_WIFI_CSTATE_RECONNECTION_IN_PROGRESS)
            {
                SYS_CONSOLE_MESSAGE("Managed (reconnection in progress)");
            }
            else 
            {
                SYS_CONSOLE_MESSAGE("managed (?)");
            }    
        }
        else if (ws.Mode.NetworkType == DRV_WIFI_NETWORK_TYPE_ADHOC)
        {
            if (iwconfigCb.connState == DRV_WIFI_CSTATE_CONNECTION_IN_PROGRESS)
            {
                SYS_CONSOLE_MESSAGE("AdHoc (connection in progress)");
            }
            else if (iwconfigCb.connState == DRV_WIFI_CSTATE_CONNECTED_ADHOC)
            {
                SYS_CONSOLE_MESSAGE("AdHoc (connected)");
            }
            else if (iwconfigCb.connState == DRV_WIFI_CSTATE_RECONNECTION_IN_PROGRESS)
            {
                SYS_CONSOLE_MESSAGE("AdHoc (reconnection in progress)");
            }
            else 
            {
                SYS_CONSOLE_MESSAGE("AdHoc (?)");
            }    
    
            SYS_CONSOLE_MESSAGE("AdHoc");
        }
        else if (ws.Mode.NetworkType == DRV_WIFI_NETWORK_TYPE_SOFT_AP)
        {
            SYS_CONSOLE_MESSAGE("SoftAP");
        }
        else
        {
            SYS_CONSOLE_MESSAGE("Unknown");
        }
    }
    SYS_CONSOLE_MESSAGE("\r\n");

    // ssid
    DRV_WIFI_SsidGet(ws.Ssid.String, &ws.Ssid.Len);
    ws.Ssid.String[ws.Ssid.Len] = '\0';

    SYS_CONSOLE_MESSAGE("\tssid:     ");
    SYS_CONSOLE_MESSAGE((char *)ws.Ssid.String);
    SYS_CONSOLE_MESSAGE("\r\n");

    // power
    switch (iwconfigCb.powerSaveState)
    {
        case DRV_WIFI_PS_PS_POLL_DTIM_ENABLED:
            SYS_CONSOLE_MESSAGE("\tpwrsave:  enabled\r\n");
            SYS_CONSOLE_MESSAGE("\tdtim rx:  enabled\r\n");
            break;
        case DRV_WIFI_PS_PS_POLL_DTIM_DISABLED:
            SYS_CONSOLE_MESSAGE("\tpwrsave:  enabled\r\n");
            SYS_CONSOLE_MESSAGE("\tdtim rx:  disabled\r\n");
            break;
        case DRV_WIFI_PS_OFF:
            SYS_CONSOLE_MESSAGE("\tpwrsave:  disabled\r\n");
            break;
        default:
            SYS_CONSOLE_PRINT("\tpwrsave:  unknown %d\r\n", iwconfigCb.powerSaveState);
            break;
    }
}

static bool iwconfigSetSsid(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    if (argc < 3)
    {
        SYS_CONSOLE_MESSAGE("Missing value for last parameter\r\n");
        return false;
    }

    if (argc > 3)
    {
        SYS_CONSOLE_MESSAGE("SSID may not contain space for this demo\r\n");
        return false;
    }

    DRV_WIFI_SsidSet((uint8_t *)argv[2], strlen((char*)argv[2]));

    return true;
}

static bool iwconfigSetMode(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    uint8_t networkType;

    DRV_WIFI_NetworkTypeGet(&networkType);

    if ( (3 <= argc) && (strcmp((char*)argv[2], "idle") == 0) )
    {
        if ( iwconfigCb.isIdle )
        {
            SYS_CONSOLE_MESSAGE("Already in the idle mode\r\n");
        }
        else
        {
            DRV_WIFI_Disconnect();
        }
    }
    else if ( (3 <= argc) && (strcmp((char*)argv[2], "managed") == 0) )
    {
        if ( iwconfigCb.isIdle )
        {
            DRV_WIFI_NetworkTypeSet(DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE);
            DRV_WIFI_Connect();
        }
        else
        {
            DRV_WIFI_NetworkTypeGet(&networkType);
            if (networkType == DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE)
            {
                SYS_CONSOLE_MESSAGE("Already in the managed mode\r\n");
            }
            else
            {
                DRV_WIFI_Disconnect();
                DRV_WIFI_NetworkTypeSet(DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE);
                DRV_WIFI_Connect();
            }
        }
    }
    else if ( (3 <= argc) && (strcmp((char*)argv[2], "adhoc") == 0) )
    {
        if ( iwconfigCb.isIdle )
        {
            DRV_WIFI_ADHOC_NETWORK_CONTEXT adhocContext;

            DRV_WIFI_NetworkTypeSet(DRV_WIFI_NETWORK_TYPE_ADHOC);
            adhocContext.beaconPeriod = DRV_WIFI_DEFAULT_ADHOC_BEACON_PERIOD;
            adhocContext.hiddenSsid   = DRV_WIFI_DEFAULT_ADHOC_HIDDEN_SSID;
            adhocContext.mode = DRV_WIFI_ADHOC_CONNECT_THEN_START;
            DRV_WIFI_AdhocContextSet(&adhocContext);
            DRV_WIFI_ReconnectModeSet(3,                              // retry N times to start or join AdHoc network
                                DRV_WIFI_DO_NOT_ATTEMPT_TO_RECONNECT, // do not attempt to reconnect on deauth from station
                                40,                                   // beacon timeout is 40 beacon periods
                                DRV_WIFI_ATTEMPT_TO_RECONNECT);       // reconnect on beacon timeout
            DRV_WIFI_Connect();
        }
        else
        {
            DRV_WIFI_NetworkTypeGet(&networkType);
            if (networkType == DRV_WIFI_NETWORK_TYPE_ADHOC)
            {
                SYS_CONSOLE_MESSAGE("Already in the adhoc mode\r\n");
            }
            else
            {
                DRV_WIFI_Disconnect();

                DRV_WIFI_NetworkTypeSet(DRV_WIFI_NETWORK_TYPE_ADHOC);
                DRV_WIFI_Connect();
            }
        }
    }
    else
    {
        SYS_CONSOLE_MESSAGE("Unknown parameter\r\n");
        return false;
    }

    return true;
}

static bool iwconfigSetChannel(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    char *p1, *p2;
    char *p_channelList;
    uint8_t index = 0;
    uint16_t temp;
    uint8_t i;
    uint8_t regionalDomain;

    DRV_WIFI_RegionalDomainGet(&regionalDomain);

    if (argc < 3)
    {
        SYS_CONSOLE_MESSAGE("No channel numbers entered\r\n");
        return false;
    }

    if ( !iwconfigCb.isIdle )
    {
        SYS_CONSOLE_MESSAGE("Channel(s) can only be set in idle mode\r\n");
        return false;
    }

    p_channelList = argv[2];
    p1 = p2 = p_channelList;

    if ( (3 <= argc) && (strcmp((char*)argv[2], "all") == 0) )
    {
        DRV_WIFI_ChannelListSet((uint8_t *)p_channelList, 0); // reset to domain default channel list
        return true;
    }

    do
    {
        if ( (p2 = strchr(p1, (int) ',')) != NULL )
        {
            *p2='\0';
            p2++;
        }

        temp = atoi(p1);
        if( temp == 0)
        {
            SYS_CONSOLE_MESSAGE("Invalid channel\r\n");
            return  false;
        }

        p1 = p2;
        p_channelList[index] = (uint8_t)temp;
        index++;

    } while (  p2 != NULL );

    // Validate channels against current Domain
    #if (regionalDomain == DRV_WIFI_DOMAIN_FCC) || (regionalDomain == DRV_WIFI_DOMAIN_IC)
        for (i=0; i<index; i++)
        {
            if (p_channelList[i] < 1 || p_channelList[i] > 11)
            {
                SYS_CONSOLE_MESSAGE("In valid channel for FCC domain\r\n");
                return false;
            }
        }
    #elif (regionalDomain == DRV_WIFI_DOMAIN_ETSI)
        for (i=0; i<index; i++)
        {
            if (p_channelList[i] < 1 || p_channelList[i] > 13)
            {
                SYS_CONSOLE_PRINT("In valid channel for ETSI domain\r\n");
                return false;
            }
        }
    #elif (regionalDomain == DRV_WIFI_DOMAIN_JAPAN) || (regionalDomain == DRV_WIFI_DOMAIN_OTHER)
        for (i=0; i<index; i++)
        {
            if (p_channelList[i] < 1 || p_channelList[i] > 14)
            {
                SYS_CONSOLE_PRINT("In valid channel for Japan or Other domain\r\n");
                return false;
            }
        }
    #elif (regionalDomain == DRV_WIFI_DOMAIN_SPAIN)
        for (i=0; i<index; i++)
        {
            if (p_channelList[i] < 10 || p_channelList[i] > 11)
            {
                SYS_CONSOLE_PRINT("In valid channel for Spain\r\n");
                return false;
            }
        }
    #elif (regionalDomain == DRV_WIFI_DOMAIN_FRANCE)
        for (i=0; i<index; i++)
        {
            if (p_channelList[i] < 10 || p_channelList[i] > 13)
            {
                SYS_CONSOLE_PRINT("In valid channel for France\r\n");
                return false;
            }
        }
    #elif (regionalDomain == DRV_WIFI_DOMAIN_JAPAN_A)
        for (i=0; i<index; i++)
        {
            if (p_channelList[i] < 14 || p_channelList[i] > 14)
            {
                SYS_CONSOLE_PRINT("In valid channel for Japan_A\r\n");
                return false;
            }
        }
    #elif (regionalDomain == DRV_WIFI_DOMAIN_JAPAN_B)
        for (i=0; i<index; i++)
        {
            if (p_channelList[i] < 1 || p_channelList[i] > 13)
            {
                SYS_CONSOLE_PRINT("In valid channel for Japan_B\r\n");
                return false;
            }
        }
    #endif

    DRV_WIFI_ChannelListSet((uint8_t *)p_channelList, index);

    return true;
}

static bool iwconfigSetPower(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    DRV_WIFI_PS_POLL_CONTEXT psPollContext;

    psPollContext.dtimInterval   = DRV_WIFI_DEFAULT_PS_DTIM_INTERVAL;
    psPollContext.listenInterval = DRV_WIFI_DEFAULT_PS_LISTEN_INTERVAL;
    psPollContext.useDtim        = true;

    if (argc < 3)
    {
        SYS_CONSOLE_MESSAGE("Missing value for last parameter\r\n");
        return false;
    }

    if ( (3 <= argc) && (strcmp((char*)argv[2], "reenable") == 0) )
    {    // reenable power saving
        DRV_WIFI_PsPollEnable(&psPollContext);
    }
    else if ( (3 <= argc) && (strcmp((char*)argv[2], "disable") == 0) )
    {    // disable power saving
        DRV_WIFI_PsPollDisable();
    }
    else if ( (3 <= argc) && (strcmp((char*)argv[2], "unicast") == 0) )
    {    // enable power saving but don't poll for DTIM
        psPollContext.useDtim = false;
        DRV_WIFI_PsPollEnable(&psPollContext);
    }
    else if ( (3 <= argc) && (strcmp((char*)argv[2], "all") == 0) )
    {    // enable power saving and poll for DTIM
        DRV_WIFI_PsPollEnable(&psPollContext);
    }
    else
    {
        SYS_CONSOLE_MESSAGE("Unknown parameter\r\n");
        return false;
    }

    return true;
}

static bool iwconfigSetRTS(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    uint16_t rtsThreshold;

    if (argc < 3)
    {
        SYS_CONSOLE_MESSAGE("Missing value for last parameter\r\n");
        return false;
    }

    rtsThreshold = atoi(argv[2]);
    if(rtsThreshold > 255)
    {
        return  false;
    }

    DRV_WIFI_RtsThresholdSet(rtsThreshold);

    return true;
}

static int CommandIw_eraseConfig(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    DRV_WIFI_ConfigDataErase();
    SYS_CONSOLE_MESSAGE("--Done\r\n");
    return 0;
}

static int CommandIw_saveConfig(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    DRV_WIFI_ConfigDataSave();
    SYS_CONSOLE_MESSAGE("--Done\r\n");
    return 0;
}
static int CommandIw_readConfig(_CMDIO_DEV_NODE* pCmdIO, int argc, char** argv)
{
    OutputDemoHeader();
    return 1;
}

static void DisplayScanResult(_CMDIO_DEV_NODE* pCmdIO, uint16_t index, DRV_WIFI_SCAN_RESULT *p_scanResult)
{
    int i;
    uint32_t rate;

    SYS_CONSOLE_MESSAGE("\r\n======================\r\n");
    SYS_CONSOLE_PRINT("Scan Result  %d\r\n", index+1);

    // ssid
    if(p_scanResult->ssidLen<32)
    {
        p_scanResult->ssid[p_scanResult->ssidLen] = '\0'; // ensure string terminator
        SYS_CONSOLE_PRINT("SSID:     %s\r\n", p_scanResult->ssid);

    }
    else
    {
        SYS_CONSOLE_MESSAGE("SSID:     ");
        for(i=0;i<32;i++) SYS_CONSOLE_PRINT("%c", p_scanResult->ssid[i]);
        SYS_CONSOLE_MESSAGE("\r\n");
    }


    // bssid
    SYS_CONSOLE_PRINT("BSSID:    %02x %02x %02x %02x %02x %02x\r\n",
                          p_scanResult->bssid[0], p_scanResult->bssid[1],
                          p_scanResult->bssid[2], p_scanResult->bssid[3],
                          p_scanResult->bssid[4], p_scanResult->bssid[5]);

    // BSS Type
    SYS_CONSOLE_MESSAGE("BSS Type: ");
    if (p_scanResult->bssType == DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE)
    {
        SYS_CONSOLE_MESSAGE("Infrastructure\r\n");
    }
    else
    {
        SYS_CONSOLE_MESSAGE("AdHoc\r\n");
    }

    // channel
    SYS_CONSOLE_PRINT("Channel:  %d\r\n", p_scanResult->channel);

    // beacon period
    SYS_CONSOLE_PRINT("Beacon:   %d ms\r\n", p_scanResult->beaconPeriod);

    // basic rates
    SYS_CONSOLE_MESSAGE("Rates:    ");
    for (i = 0; i < p_scanResult->numRates; ++i)
    {
        rate = (p_scanResult->basicRateSet[i] & ~0x80) * 500000;
        if ((rate % 1000000) == 0)
        {
            SYS_CONSOLE_PRINT("%d, ", rate / 1000000);
        }
        else
        {
            SYS_CONSOLE_PRINT("%d.5, ", rate / 1000000);
        }
    }
    SYS_CONSOLE_MESSAGE("  (mbps)\r\n");

    // security
    SYS_CONSOLE_MESSAGE("Security: ");
    if ((p_scanResult->apConfig & 0x10) > 0)        // if privacy bit set
    {
        if ((p_scanResult->apConfig & 0x80) > 0)    // if WPA2 bit is set
        {
            SYS_CONSOLE_MESSAGE("WPA2\r\n");
        }
        else if ((p_scanResult->apConfig & 0x40) > 0)    // if WPA bit is set
        {
            SYS_CONSOLE_MESSAGE("WPA\r\n");
        }
        else
        {
            SYS_CONSOLE_MESSAGE("WEP\r\n");
        }
    }
    else
    {
        SYS_CONSOLE_MESSAGE("Open\r\n");
    }

    // preamble
    SYS_CONSOLE_MESSAGE("Preamble: ");
    if ((p_scanResult->apConfig & 0x20) > 0)
    {
        SYS_CONSOLE_MESSAGE("Long\r\n");
    }
    else
    {
         SYS_CONSOLE_MESSAGE("Short\r\n");
    }

    // rssi
    SYS_CONSOLE_PRINT("RSSI:     %d\r\n", p_scanResult->rssi);
}


/*****************************************************************************
 * FUNCTION: HexToBin
 *
 * RETURNS: binary value associated with ASCII hex input value
 *
 * PARAMS:  hexChar -- ASCII hex character
 *
 * NOTES:   Converts an input ascii hex character to its binary value.  Function
 *          does not error check; it assumes only hex characters are passed in.
 *****************************************************************************/
uint8_t HexToBin(uint8_t hexChar)
{
    if ((hexChar >= 'a') && (hexChar <= 'f'))
    {
        return (0x0a + (hexChar - 'a'));
    }
    else if ((hexChar >= 'A') && (hexChar <= 'F'))
    {
        return (0x0a + (hexChar - 'A'));
    }
    else //  ((hexChar >= '0') && (hexChar <= '9'))
    {
        return (0x00 + (hexChar - '0'));
    }

}

/*****************************************************************************
 * FUNCTION: ConvertASCIIHexToBinary
 *
 * RETURNS: true if conversion successful, else false
 *
 * PARAMS:  p_ascii   -- ascii string to be converted
 *          p_binary  -- binary value if conversion successful
 *
 * NOTES:   Converts an input ascii hex string to binary value (up to 32-bit value)
 *****************************************************************************/
static bool ConvertASCIIHexToBinary(char *p_ascii, uint16_t *p_binary)
{
    int8_t  i;
    uint32_t multiplier = 1;

    *p_binary = 0;

    // not allowed to have a string of more than 4 nibbles
    if (strlen((char*)p_ascii) > 8u)
    {
        return false;
    }

    // first, ensure all characters are a hex digit
    for (i = (uint8_t)strlen((char *)p_ascii) - 1; i >= 0 ; --i)
    {
        if (!isxdigit(p_ascii[i]))
        {
            return false;
        }
        *p_binary += multiplier * HexToBin(p_ascii[i]);
        multiplier *= 16;
    }

    return true;
}

bool convertAsciiToHexInPlace(char *p_string, uint8_t expectedHexBinSize )
{

    char  ascii_buffer[3];
    uint8_t  hex_binary_index = 0;
    char  *hex_string_start = p_string;
    uint16_t hex_buffer = 0;

    /* gobble up any hex prefix */
    if ( memcmp(hex_string_start, (const char*) "0x", 2) == 0 )
    {
         hex_string_start+=2;
    }

   if ( strlen( (char *) hex_string_start) != (expectedHexBinSize*2) )
   {
      return false;
   }

    while ( hex_binary_index < expectedHexBinSize )
    {

      memcpy ( ascii_buffer, (const char*) hex_string_start, 2 );
      ascii_buffer[2] = '\0';

      /* convert the hex string to a machine hex value */
      if ( !ConvertASCIIHexToBinary( ascii_buffer,&hex_buffer) )
      {
        return false;
      }

      p_string[hex_binary_index++] = (uint8_t) hex_buffer;
      hex_string_start +=2;
    }

    return true;
}

#endif /* TCPIP_STACK_COMMANDS_WIFI_ENABLE */

//DOM-IGNORE-END