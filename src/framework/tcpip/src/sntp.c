/*******************************************************************************
  Simple Network Time Protocol (SNTP) Client Version 3

  Summary:
    Module for Microchip TCP/IP Stack

  Description:
    -Locates an NTP Server from public site using DNS
    -Requests UTC time using SNTP and updates SNTPTime structure
     periodically, according to NTP_QUERY_INTERVAL value
    -Reference: RFC 1305
*******************************************************************************/

/*******************************************************************************
File Name:  SNTP.c
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

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

#define __SNTP_C

#include "tcpip/src/tcpip_private.h"
#include "system/tmr/sys_tmr.h"

#if defined(TCPIP_STACK_USE_SNTP_CLIENT)

// Defines the structure of an NTP packet
typedef struct
{
    struct
    {
        uint8_t mode            : 3;  // NTP mode
        uint8_t versionNumber   : 3;  // SNTP version number
        uint8_t leapIndicator   : 2;  // Leap second indicator
    } flags;                          // Flags for the packet

    uint8_t stratum;                  // Stratum level of local clock
    int8_t poll;                      // Poll interval
    int8_t precision;                 // Precision (seconds to nearest power of 2)
    uint32_t root_delay;              // Root delay between local machine and server
    uint32_t root_dispersion;         // Root dispersion (maximum error)
    uint32_t ref_identifier;          // Reference clock identifier
    uint32_t ref_ts_secs;             // Reference timestamp (in seconds)
    uint32_t ref_ts_fraq;             // Reference timestamp (fractions)
    uint32_t orig_ts_secs;            // Origination timestamp (in seconds)
    uint32_t orig_ts_fraq;            // Origination timestamp (fractions)
    uint32_t recv_ts_secs;            // Time at which request arrived at sender (seconds)
    uint32_t recv_ts_fraq;            // Time at which request arrived at sender (fractions)
    uint32_t tx_ts_secs;              // Time at which request left sender (seconds)
    uint32_t tx_ts_fraq;              // Time at which request left sender (fractions)

} NTP_PACKET;

// Seconds value obtained by last update
static uint32_t dwSNTPSeconds = 0;

// Tick count of last update
static uint32_t dwLastUpdateTick = 0;

static TCPIP_NET_IF*  pSntpIf = 0;    // we use only one interface for SNTP (for now at least)
static TCPIP_NET_IF*  pSntpDefIf = 0;    // default SNTP interface

static UDP_SOCKET   sntpSocket = INVALID_UDP_SOCKET;    // UDP socket we use


static enum
{
    SM_HOME = 0,
    SM_WAIT_DNS,
    SM_DNS_RESOLVED,
    SM_UDP_IS_OPENED,
    SM_UDP_SEND,
    SM_UDP_RECV,
    SM_SHORT_WAIT,
    SM_WAIT

} sntpState = SM_HOME;

// the server address
static IP_MULTI_ADDRESS  serverIP;
static IP_ADDRESS_TYPE   ntpConnection = NTP_DEFAULT_CONNECTION_TYPE;
static union
{
    struct
    {
        uint32_t    tStampSeconds;
        uint32_t    tStampFraction;
    };
    uint64_t    llStamp;
}ntpTimeStamp;       // last valid time stamp

static uint32_t         ntpLastStampTick;   // time of the last time stamp
static SNTP_RESULT      ntpLastError;

// returns true if the module is idle
// and a new connection can be started or parameters changed
static bool _SntpIsIdle(void)
{
    return (sntpState == SM_HOME || sntpState == SM_SHORT_WAIT || sntpState == SM_WAIT);
}

static void  _SntpInit(TCPIP_NET_IF* pNewIf)
{
    if(pSntpIf != 0)
    {
        if(pNewIf == pSntpIf)
        {   // this interface is going away/re-initialized, etc
            if(sntpSocket != INVALID_UDP_SOCKET)
            {
                TCPIP_UDP_Close(sntpSocket);
                sntpSocket = INVALID_UDP_SOCKET;
            }
            pSntpIf = 0;
            sntpState = SM_HOME;
            ntpTimeStamp.llStamp = 0;
            ntpLastStampTick = 0;
        }
    }
}//_SntpInit


// sntp_manager.h
bool TCPIP_SNTP_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const SNTP_MODULE_CONFIG* pSNTPConfig)
{

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }

    // stack init
    _SntpInit(stackCtrl->pNetIf);

    pSntpDefIf = (TCPIP_NET_IF*)TCPIP_STACK_NetHandleGet(NTP_DEFAULT_IF);

    ntpLastError = SNTP_RES_NTP_SERVER_TMO; 
    sntpState = SM_HOME;
    return true;

}//TCPIP_SNTP_Initialize


// sntp_manager.h
void TCPIP_SNTP_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{

    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN)
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down

    // interface is going down one way or another
    _SntpInit(stackCtrl->pNetIf);

}//TCPIP_SNTP_Deinitialize


// sntp_manager.h
bool TCPIP_SNTP_Client(TCPIP_NET_IF* pNetIf)
{
    NTP_PACKET          pkt;
    uint16_t            w;
    TCPIP_DNS_RESULT          dnsRes;
    static uint32_t     SNTPTimer;


    if(pSntpIf != 0 && pNetIf != pSntpIf)
    {   // not our job
        return false;
    }

    switch(sntpState)
    {
        case SM_HOME:
            sntpSocket = INVALID_UDP_SOCKET;
            pSntpIf = pSntpDefIf;
            if(!TCPIP_STACK_NetworkIsLinked(pSntpIf))
            {
                pSntpIf = _TCPIPStackAnyNetLinked(true);
            }

            TCPIP_DNS_Resolve(NTP_SERVER, ntpConnection ==IP_ADDRESS_TYPE_IPV6 ? DNS_TYPE_AAAA : DNS_TYPE_A);
            sntpState++;
            break;

        case SM_WAIT_DNS:
            dnsRes = TCPIP_DNS_IsResolved(NTP_SERVER, &serverIP);
            if(dnsRes == DNS_RES_PENDING)
            {   // ongoing operation;
                break;
            }
            else if(dnsRes < 0)
            {   // some DNS error occurred; retry after waiting a while
                SNTPTimer = SYS_TMR_TickCountGet();
                sntpState = SM_SHORT_WAIT;
                ntpLastError = SNTP_RES_NTP_DNS_ERR;
            }
            else
            {
                sntpState++;
            }
            break;

        case SM_DNS_RESOLVED:
	    sntpSocket = TCPIP_UDP_ClientOpen(ntpConnection, TCPIP_NTP_SERVER_REMOTE_PORT, (IP_MULTI_ADDRESS*)&serverIP);
            if(sntpSocket != INVALID_UDP_SOCKET)
            {
                TCPIP_UDP_SocketNetSet(sntpSocket, pSntpIf);
                sntpState++;
                SNTPTimer = SYS_TMR_TickCountGet();
            }
            else
            {
                ntpLastError = SNTP_RES_SKT_ERR; 
            }
            break;

        case SM_UDP_IS_OPENED:
            if(TCPIP_UDP_IsOpened(sntpSocket) == true)
            {
                SNTPTimer = SYS_TMR_TickCountGet();
                sntpState = SM_UDP_SEND;
            }
            else if((SYS_TMR_TickCountGet() - SNTPTimer > 1*SYS_TMR_TickPerSecond()))
            {   // failed to open
                TCPIP_UDP_Close(sntpSocket);
                sntpState = SM_DNS_RESOLVED;
                sntpSocket = INVALID_UDP_SOCKET;
                ntpLastError = SNTP_RES_SKT_ERR; 
            }
            break;

        case SM_UDP_SEND:
            // Open up the sending UDP socket
            // Make certain the socket can be written to
            if(!TCPIP_UDP_TxPutIsReady(sntpSocket, sizeof(pkt)))
            {   // Wait no more than 1 sec
                if((SYS_TMR_TickCountGet() - SNTPTimer > 1*SYS_TMR_TickPerSecond()))
                {
                    TCPIP_UDP_Close(sntpSocket);
                    sntpState = SM_DNS_RESOLVED;
                    sntpSocket = INVALID_UDP_SOCKET;
                    ntpLastError = SNTP_RES_SKT_ERR; 
                    break;
                }
            }

            // Success
            // Transmit a time request packet
            memset(&pkt, 0, sizeof(pkt));
            pkt.flags.versionNumber = NTP_VERSION;
            pkt.flags.mode = 3;             // NTP Client
            pkt.orig_ts_secs = TCPIP_Helper_htonl(NTP_EPOCH);
            TCPIP_UDP_ArrayPut(sntpSocket, (uint8_t*) &pkt, sizeof(pkt));
            TCPIP_UDP_Flush(sntpSocket);

            SNTPTimer = SYS_TMR_TickCountGet();
            sntpState = SM_UDP_RECV;
            break;

        case SM_UDP_RECV:
            // Look for a response time packet
            if(!TCPIP_UDP_GetIsReady(sntpSocket))
            {
                if((SYS_TMR_TickCountGet()) - SNTPTimer > NTP_REPLY_TIMEOUT * SYS_TMR_TickPerSecond())
                {
                    // Abort the request and resume
                    TCPIP_UDP_Close(sntpSocket);
                    //SNTPTimer = SYS_TMR_TickCountGet();
                    //sntpState = SM_SHORT_WAIT;
                    sntpState = SM_HOME;
                    sntpSocket = INVALID_UDP_SOCKET;
                    ntpLastError = SNTP_RES_NTP_SERVER_TMO; 
                }
                break;
            }

            // Get the response time packet
            w = TCPIP_UDP_ArrayGet(sntpSocket, (uint8_t*) &pkt, sizeof(pkt));
            TCPIP_UDP_Close(sntpSocket);
            SNTPTimer = SYS_TMR_TickCountGet();

            sntpState = SM_WAIT;
            sntpSocket = INVALID_UDP_SOCKET;

            // sanity packet check
            if(w != sizeof(pkt) || pkt.flags.versionNumber != NTP_VERSION )
            {
                ntpLastError = SNTP_RES_NTP_VERSION_ERR; 
                break;
            }
            if((pkt.tx_ts_secs == 0 && pkt.tx_ts_fraq == 0))
            {
                ntpLastError = SNTP_RES_NTP_TSTAMP_ERR; 
                break;
            }
            if(pkt.stratum == 0 )
            {
                ntpLastError = SNTP_RES_NTP_KOD_ERR; 
                break;
            }
            if(pkt.stratum >= NTP_MAX_STRATUM || pkt.flags.leapIndicator == 3 )
            {
                ntpLastError = SNTP_RES_NTP_SYNC_ERR; 
                break;
            }

            // get the last timestamp
            ntpTimeStamp.tStampSeconds = pkt.tx_ts_secs;
            ntpTimeStamp.tStampFraction = pkt.tx_ts_fraq;
            ntpLastStampTick = SYS_TMR_TickCountGet();

            
            // Set out local time to match the returned time
            dwLastUpdateTick = ntpLastStampTick;
            dwSNTPSeconds = TCPIP_Helper_ntohl(pkt.tx_ts_secs) - NTP_EPOCH;
            // Do rounding.  If the partial seconds is > 0.5 then add 1 to the seconds count.
            if(((uint8_t*)&pkt.tx_ts_fraq)[0] & 0x80)
                dwSNTPSeconds++;

            break;

        case SM_SHORT_WAIT:
            // Attempt to requery the NTP server after a specified NTP_FAST_QUERY_INTERVAL time (ex: 8 seconds) has elapsed.
            if(SYS_TMR_TickCountGet() - SNTPTimer > (NTP_FAST_QUERY_INTERVAL * SYS_TMR_TickPerSecond()))
            {
                sntpState = SM_HOME;
                sntpSocket = INVALID_UDP_SOCKET;
            }
            break;

        case SM_WAIT:
            // Requery the NTP server after a specified NTP_QUERY_INTERVAL time (ex: 10 minutes) has elapsed.
            if(SYS_TMR_TickCountGet() - SNTPTimer > (NTP_QUERY_INTERVAL * SYS_TMR_TickPerSecond()))
            {
                sntpState = SM_HOME;
                sntpSocket = INVALID_UDP_SOCKET;
            }

            break;
    }

    return true;

}//TCPIP_SNTP_Client


// sntp.h
uint32_t TCPIP_SNTP_UTCSecondsGet(void)
{

    uint32_t dwTickDelta;
    uint32_t dwTick;

    // Update the dwSNTPSeconds variable with the number of seconds
    // that has elapsed
    dwTick = SYS_TMR_TickCountGet();
    dwTickDelta = dwTick - dwLastUpdateTick;
    while(dwTickDelta > SYS_TMR_TickPerSecond())
    {
        dwSNTPSeconds++;
        dwTickDelta -= SYS_TMR_TickPerSecond();
    }


    // Save the tick and residual fractional seconds for the next call
    dwLastUpdateTick = dwTick - dwTickDelta;

    return dwSNTPSeconds;

}//TCPIP_SNTP_UTCSecondsGet

SNTP_RESULT TCPIP_SNTP_ConnectionParamSet(TCPIP_NET_HANDLE netH, IP_ADDRESS_TYPE ntpConnType)
{
    if(!_SntpIsIdle())
    {
        return SNTP_RES_BUSY;
    }

    if(netH)
    {
        pSntpDefIf = _TCPIPStackHandleToNet(netH);
    }

    if(ntpConnType != IP_ADDRESS_TYPE_ANY)
    {
        ntpConnection = ntpConnType;
    }

    return SNTP_RES_OK;
}


SNTP_RESULT TCPIP_SNTP_ConnectionInitiate(void)
{
    if(!_SntpIsIdle())
    {
        return SNTP_RES_PROGRESS;
    }

    sntpState = SM_HOME;
    return SNTP_RES_OK;
}

SNTP_RESULT TCPIP_SNTP_TimeStampGet(uint64_t* pTStamp, uint32_t* pLastUpdate)
{
    SNTP_RESULT res;

    if(ntpTimeStamp.llStamp == 0 || ntpLastStampTick == 0 || (SYS_TMR_TickCountGet() - ntpLastStampTick >  NTP_TIME_STAMP_TMO))
    {
        res = SNTP_RES_TSTAMP_STALE;
    }
    else
    {
        res = SNTP_RES_OK;
    }

    if(pTStamp)
    {
        *pTStamp = ntpTimeStamp.llStamp;
    }
    if(pLastUpdate)
    {
        *pLastUpdate = ntpLastStampTick;
    }

    return res;

}

SNTP_RESULT TCPIP_SNTP_LastErrorGet(void)
{
    SNTP_RESULT res = ntpLastError;
    ntpLastError = SNTP_RES_OK;
    return res; 
}

#endif  //if defined(TCPIP_STACK_USE_SNTP_CLIENT)
