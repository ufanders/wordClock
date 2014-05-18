/*******************************************************************************
  MRF24W Driver Medium Access Control (MAC) Layer

  File Name: 
    drv_wifi_mac.c  
  
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


/*
*********************************************************************************************************
*                                           INCLUDES                               
*********************************************************************************************************
*/

#include "tcpip/src/tcpip_private.h"
#include "system/tmr/sys_tmr.h"

#if defined(TCPIP_IF_MRF24W) 

#include "../framework/driver/wifi/mrf24w/src/drv_wifi_easy_config.h"

#if 1  //temporary timer service api
#include "system/tmr/src/sys_tmr_local.h"
extern SYS_TMR_DELAY_OBJECT     sDelayObject [ SYS_TMR_MAX_DELAY_EVENTS ];
void SYS_TMR_CallbackDeregister_wifi ( SYS_TMR_HANDLE handle )
{
    SYS_TMR_DELAY_OBJECT * delayObject;

    /* Check for handle validity */
    if ( SYS_TMR_HANDLE_INVALID == handle )
    {
        return;
    }

    delayObject = &sDelayObject [ handle ];

    /* Change the event status, to stop the event */
    delayObject->status = SYS_TMR_CALLBACK_INACTIVE;
    delayObject->callback = NULL;

    /* Remove the item from the Queue */
    delayObject->inUse = false;
    
} /* SYS_TMR_CallbackDeregister */

#endif


//#define DBG_TX_PRINT(s) SYS_CONSOLE_MESSAGE(s)
#define DBG_TX_PRINT(s)

//#define DBG_RX_PRINT(s) SYS_CONSOLE_MESSAGE(s)
#define DBG_RX_PRINT(s)


#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_MAC_MRF24W
/*
*********************************************************************************************************
*                                           DEFINES                               
*********************************************************************************************************
*/

#define NUM_PREALLOCATED_RX_PACKETS     (4)

#define SNAP_HDR_LENGTH (6)
#define SNAP_VAL        (0xaa)
#define SNAP_CTRL_VAL   (0x03)
#define SNAP_TYPE_VAL   (0x00)

#define SNAP_HEADER_OFFSET         (10)     // offset in t_RxHeader where SNAP header starts
#define ETH_HEADER_START_OFFSET    (16)     // offset in Rx packet where Ethernet header starts
#define WF_RX_PREAMBLE_SIZE         (sizeof(t_wfRxPreamble))
#define WF_TX_PREAMBLE_SIZE         (sizeof(t_wfTxPreamble))
#define ETH_HEADER_SIZE            (14)


//--------------
// RAW States
//--------------

// RAW Init states
enum
{
    R_INIT_BEGIN                = 0,
    R_WAIT_FOR_SCRATCH_UNMOUNT  = 1,
    R_WAIT_FOR_SCRATCH_MOUNT    = 2
} t_rawInitStates;

//---------------
// Rx Data states
//---------------
typedef enum
{
    DATA_RX_IDLE             = 0,
    DATA_RX_WAIT_FOR_TX_IDLE = 1,
    WAIT_FOR_DATA_RX_MOUNT   = 2,
    WAIT_FOR_DATA_RX_UNMOUNT = 3

} t_RxStates;

//---------------
// Tx data states
//---------------
typedef enum
{
    DATA_TX_IDLE                  = 0,
    WAIT_FOR_TX_MEMORY            = 1,
    WAIT_FOR_DATA_TX_MOUNT        = 2,
    WAIT_FOR_DATA_TX_SIGNAL       = 3,
    WAIT_FOR_RX_IDLE              = 4
} t_TxStates;


/*
*********************************************************************************************************
*                                           LOCAL DATA TYPES                               
*********************************************************************************************************
*/

// this structure is present in all Rx packets received from the MRF24WG, at the 
// start of the Rx RAW buffer, starting at index 0.  Payload data immediately follows
// this structure
typedef struct wfRxHeaderStruct
{
    uint8_t     type;               // always WF_DATA_RX_INDICATE_TYPE (3)
    uint8_t     subtype;
    uint16_t    rssi;               // not used
    uint32_t    arrivalTime;        // not used
    uint16_t    dataLength;         // number of bytes of payload which immediately follow this structure
    uint8_t     snapHeader[6];      // SNAP header (always
    uint8_t     destAddress[6];     // destination MAC address  (start of Ethernet header)
    uint8_t     srcMacAddress[6];   // source MAC address
    uint16_t    ethType;            // Ethernet type code
} t_RxHeader;


typedef struct
{
    uint8_t type;
    uint8_t subType;
} t_RxPreamble;

typedef struct
{
    uint8_t           snap[SNAP_HDR_LENGTH];
    TCPIP_MAC_ADDR    DestMACAddr;    // ethernet header presented to stack starts here
    TCPIP_MAC_ADDR    SourceMACAddr;
    TCPIP_UINT16_VAL  Type;
} t_wfRxPreamble;

// used to keep track of rx packets queued for stack
typedef struct fifo_struct
{
    int front;
    int rear;
    TCPIP_MAC_PACKET * items[NUM_PREALLOCATED_RX_PACKETS + 1];
} t_fifo;

typedef struct
{
    uint8_t  reserved[4];
} t_wfTxPreamble;


/*
*********************************************************************************************************
*                                           LOCAL GLOBAL VARIABLES                               
*********************************************************************************************************
*/
static uint8_t  g_rawInitState;                         // state for Raw Init state machine
static uint8_t  g_rxDataState;                          // state for Data Rx state machine
static uint8_t  g_txDataState;                          // state for Data Tx state machine
static SYS_TMR_HANDLE g_DataRxRawMoveTimer  = NULL;   // timer callback for Data Rx raw move complete
static SYS_TMR_HANDLE g_DataTxRawMoveTimer  = NULL;   // timer callback for Data Tx raw move complete
static SYS_TMR_HANDLE g_ScratchRawMoveTimer = NULL;   // timer callback for Scratch raw move complete
static bool g_rxDataRawMoveTimeout;                     // set to true when rx data raw move timeout occurs
static bool g_txDataRawMoveTimeout;                     // set to true when tx data raw move timeout occurs
static bool g_scratchRawMoveTimeout;                    // set to true when scratch raw move timeout occurs
static SINGLE_LIST g_dataTxQueue = { 0 };               // queue of data tx packets waiting to be transmitted from host
static SINGLE_LIST g_dataRxQueue = { 0 };               // queue of data rx packets waiting to be processed by stack from host
static t_fifo  g_rxFifo;                                // FIFO to hold pointers to Rx packets
static bool g_rxPacketPending;
static bool g_txWaitingForRxIdle;


static TCPIP_MAC_PACKET *RxBuffer_packet[NUM_PREALLOCATED_RX_PACKETS]={NULL};

// packet allocation functions as passed by the stack
static _TCPIP_PKT_ALLOC_PTR    g_pktAllocF = 0;
static _TCPIP_PKT_FREE_PTR     g_pktFreeF = 0;
static _TCPIP_PKT_ACK_PTR      g_pktAckF = 0;


/*
*********************************************************************************************************
*                                           LOCAL FUNCTION PROTOTYPES                               
*********************************************************************************************************
*/

static void RawInit(void);
static void RawMoveDataRxTimeoutHandler(void);
static void RawMoveDataTxTimeoutHandler(void);
static void RawMoveScratchTimeoutHandler(void);
static void RxDataCallback(TCPIP_MAC_PACKET * pktHandle, const void* ackParam);
static bool isRxPacketHeaderValid(void);
static TCPIP_MAC_PACKET * GetAvailRxBuf(void);
static void FifoInit(t_fifo *p_fifo);
static bool isFifoEmpty(t_fifo *p_fifo);
static void FifoInsert(t_fifo *p_fifo, TCPIP_MAC_PACKET *p_packet);
static TCPIP_MAC_PACKET * FifoRemove(t_fifo *p_fifo);
static bool isTxStateMachineIdle(void);


#if (WF_GRATUITOUS_ARP == DRV_WIFI_ENABLED)
// needed to move struct definition,prototype, and static here; compiler would
// not see them above when wrapped with #if (WF_GRATUITOUS_ARP == DRV_WIFI_ENABLED)
typedef struct
{
    uint8_t          periodInSeconds;       // period in seconds betweeen each arp
    SystemTickHandle timerHandle;           // handle to timer handle
} t_gratuitousArpContext;

static void GratuitousArpHandler(SYS_TICK curSysTick);

static t_gratuitousArpContext g_gratArpContext;

// Called periodically when gratuitous arp is being used.  Called from timer interrupt, so
// don't do any work here, but signal an async event.  We don't want to send an
// arp from the timer interrupt!
static void GratuitousArpHandler(SYS_TICK curSysTick)
{
    curSysTick = curSysTick;  // avoid compiler warning

    // if connected and gratuitous arp has not been disabled
    if ( (WFisConnected()) && (g_gratArpContext.periodInSeconds > 0) )
    {
        WifiAsyncSetEventPending(ASYNC_GRAT_ARP_PENDING);
    }
}

// called by system async event handler
void WiFi_GratuitousArpTask(void)
{
    TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandleGet("MRF24W");
    if(netH)
    {
        TCPIP_NET_IF* pNetIf =  (TCPIP_NET_IF*)netH;
        TCPIP_ARP_Probe(netH, &pNetIf->netIPAddr, &pNetIf->netIPAddr, ARP_OPERATION_REQ | ARP_OPERATION_PROBE_ONLY);
    }
}

// called during init to create gratuitous ARP timer and callback reference
void InitGratuitousArp(void)
{
    // create timer that will be use for gratuitous arp
}


/*******************************************************************************
  Function:
    void DRV_WIFI_GratuitousArpStart(uint8_t period);

  Summary:
    Starts a periodic gratuitous ARP response

  Description:
    This function starts a gratuitous ARP response to be periodically transmitted.

  Parameters:
    period - period between gratuitous ARP, in seconds

  Returns:
    None.
  *****************************************************************************/
void DRV_WIFI_GratuitousArpStart(uint8_t period)
{
    // configure the arp timer for the desired period
    g_gratArpContext.timerHandle = SYS_TMR_CallbackPeriodic(1000 * period, GratuitousArpHandler);
}

/*******************************************************************************
  Function:
    void DRV_WIFI_GratuitousArpStop(void);

  Summary:
    Stops a periodic gratuitous ARP.

  Description:
    This function stops a gratuitous ARP.

  Precondition:
    WiFi initialization must be complete.

  Parameters:
    None

  Returns:
    None.

  Example:
      <code>
        DRV_WIFI_GratuitousArpStop();
      </code>

  Remarks:
    None.
  *****************************************************************************/
void DRV_WIFI_GratuitousArpStop(void)
{
    SYS_TMR_CallbackDeregister_wifi(g_gratArpContext.timerHandle);
}
#endif // WF_GRATUITOUS_ARP == WF_DRV_ENABLED



#if (WF_SAVE_WPS_CREDENTIALS == DRV_WIFI_ENABLED)

enum
{
    WEP_SHORT_KEY_SIZE  = 5,
    WEP_LONG_KEY_SIZE   = 13
};

enum
{
    SECURITY_NONE,
    SECURITY_OPEN,
    SECURITY_SHARED_KEY40,
    SECURITY_SHARED_KEY104,
    SECURITY_OPEN_KEY40,
    SECURITY_OPEN_KEY104,
    SECURITY_WPA1_PSK_KEY,
    SECURITY_WPA1_PSK_PASS,
    SECURITY_WPA2_PSK_KEY,
    SECURITY_WPA2_PSK_PASS,
    SECURITY_WPAUTO_PSK_KEY,
    SECURITY_WPAUTO_PSK_PASS,
    SECURITY_WPA_ENTERPRISE,
    SECURITY_WPS_PIN,
    SECURITY_WPS_PSB,
};

enum
{
    WEP_KEYIDX_MAX = 4,
    MSK_MAX = 64,
    PIN_MAX = 8,
};

struct sec_wep40
{
    uint8_t key_idx;
    uint8_t key[WEP_KEYIDX_MAX][5];
};

struct sec_wep104
{
    uint8_t key_idx;
    uint8_t key[WEP_KEYIDX_MAX][13];
};

struct sec_wpa_psk
{
    uint8_t key_len;
    uint8_t key[MSK_MAX];
};

struct sec_wps
{
    uint8_t pin[PIN_MAX];
};

union sec_key
{
    struct sec_wep40 wep40;
    struct sec_wep104 wep104;
    struct sec_wpa_psk wpa_psk;
    struct sec_wps wps;
};


static uint8_t ConvAscii2Hex(uint8_t a)
{
    if (a >= '0' && a <= '9')
        return (uint8_t)(a - 48);
    if (a >= 'a' && a <= 'f')
        return (uint8_t)(a - 97 + 10);
    if (a >= 'A' && a <= 'F')
        return (uint8_t)(a - 65 + 10);

    return '?';
}

static void ConvAsciiKey2Hex(uint8_t *key, uint8_t keyLen, uint8_t *hexKey)
{
    uint8_t i;

    for (i = 0; i < keyLen; i += 2)
    {
        hexKey[i / 2] = ConvAscii2Hex(key[i]) << 4;
        hexKey[i / 2] |= ConvAscii2Hex(key[i + 1]);
    }
}

static void ConfigWep(DRV_WIFI_WPS_CREDENTIAL *cred, uint8_t *secType, union sec_key *key)
{
    uint8_t i;
    uint8_t wep_key[WEP_LONG_KEY_SIZE];
    struct sec_wep40 *wep_ctx = (struct sec_wep40 *)key;
    uint8_t *keys = (uint8_t *)wep_ctx + 1;
    uint8_t key_len = 0;

    if (cred->keyLen == WEP_SHORT_KEY_SIZE * 2)
    {
        *secType = DRV_WIFI_SECURITY_WEP_40;
        ConvAsciiKey2Hex(cred->netKey, cred->keyLen, wep_key);
        key_len = cred->keyLen / 2;
    }
    else if (cred->keyLen == WEP_SHORT_KEY_SIZE)
    {
        *secType = DRV_WIFI_SECURITY_WEP_40;
        memcpy(wep_key, cred->netKey, cred->keyLen);
        key_len = cred->keyLen;
    } 
    else if (cred->keyLen == WEP_LONG_KEY_SIZE * 2)
    {
        *secType = DRV_WIFI_SECURITY_WEP_104;
        ConvAsciiKey2Hex(cred->netKey, cred->keyLen, wep_key);
        key_len = cred->keyLen / 2;
    }
    else if (cred->keyLen == WEP_LONG_KEY_SIZE)
    {
        *secType = DRV_WIFI_SECURITY_WEP_104;
        memcpy(wep_key, cred->netKey, cred->keyLen);
        key_len = cred->keyLen;
    } 
    else
    {
        //WF_ASSERT(false);
    }

    for (i = 0; i < 4; i++)
    {
        memcpy(keys + i * key_len, wep_key, key_len);
    }

    wep_ctx->key_idx = cred->keyIdx - 1;
}


void WF_SaveWPSCredentials(void)
{
    DRV_WIFI_WPS_CREDENTIAL cred;
    union sec_key key;
    uint8_t *psk;
    static bool once = false;

    if (!once)
    {
        DRV_WIFI_WPSCredentialsGet(&cred);
        memcpy((void *)(p_wifi_ConfigData->netSSID), (void *)cred.ssid, cred.ssidLen);
        p_wifi_ConfigData->SsidLength  = cred.ssidLen;

        switch (cred.authType)
        {
        case DRV_WIFI_WPS_AUTH_OPEN:
            if (cred.encType == DRV_WIFI_WPS_ENC_NONE)
            {
                p_wifi_ConfigData->SecurityMode = DRV_WIFI_SECURITY_OPEN;
            } 
            else if (cred.encType == DRV_WIFI_WPS_ENC_WEP)
            {
                ConfigWep(&cred, &(p_wifi_ConfigData->SecurityMode), &key);
                if (p_wifi_ConfigData->SecurityMode == DRV_WIFI_SECURITY_WEP_40)
                {
                    memcpy((void *)p_wifi_ConfigData->SecurityKey, (void *)key.wep40.key, WEP_SHORT_KEY_SIZE * 4);
                    p_wifi_ConfigData->SecurityKeyLength = WEP_SHORT_KEY_SIZE * 4;
                } 
                else if (p_wifi_ConfigData->SecurityMode == DRV_WIFI_SECURITY_WEP_104)
                {
                    memcpy((void *)p_wifi_ConfigData->SecurityKey, (void *)key.wep104.key, WEP_LONG_KEY_SIZE * 4);
                    p_wifi_ConfigData->SecurityKeyLength = WEP_LONG_KEY_SIZE * 4;
                } 
                else
                {
                    //WF_ASSERT(false);
                }
            }
            break;

        case DRV_WIFI_WPS_AUTH_SHARED:
            ConfigWep(&cred, &p_wifi_ConfigData->SecurityMode, &key);
            if (p_wifi_ConfigData->SecurityMode == DRV_WIFI_SECURITY_WEP_40)
            {
                memcpy((void *)p_wifi_ConfigData->SecurityKey, (void *)key.wep40.key, WEP_SHORT_KEY_SIZE * 4);
                p_wifi_ConfigData->SecurityKeyLength = WEP_SHORT_KEY_SIZE * 4;
            } 
            else if (p_wifi_ConfigData->SecurityMode == DRV_WIFI_SECURITY_WEP_104)
            {
                memcpy((void *)p_wifi_ConfigData->SecurityKey, (void *)key.wep104.key, WEP_LONG_KEY_SIZE * 4);
                p_wifi_ConfigData->SecurityKeyLength = WEP_LONG_KEY_SIZE * 4;
            } 
            else
            {
                //WF_ASSERT(false);
            }
            break;

        case DRV_WIFI_WPS_AUTH_WPA_PSK:
        case DRV_WIFI_WPS_AUTH_WPA2_PSK:
            psk = (uint8_t *)p_wifi_ConfigData->SecurityKey;;
            memset((void *)psk, 0x00, 64);
            if (cred.keyLen == 64)
            {
                p_wifi_ConfigData->SecurityMode = cred.authType == DRV_WIFI_WPS_AUTH_WPA_PSK ?DRV_WIFI_SECURITY_WPA_WITH_KEY : DRV_WIFI_SECURITY_WPA2_WITH_KEY;
                p_wifi_ConfigData->SecurityKeyLength = 32;
                ConvAsciiKey2Hex(cred.netKey, cred.keyLen, psk);
            } 
            else if (cred.keyLen >= 8 && cred.keyLen < 64)
            {
                p_wifi_ConfigData->SecurityMode= cred.authType == DRV_WIFI_WPS_AUTH_WPA_PSK ?DRV_WIFI_SECURITY_WPA_WITH_PASS_PHRASE : DRV_WIFI_SECURITY_WPA2_WITH_PASS_PHRASE;
                p_wifi_ConfigData->SecurityKeyLength = cred.keyLen;
                if (p_wifi_ConfigData->SecurityKeyLength > 8 && cred.netKey[p_wifi_ConfigData->SecurityKeyLength - 1] == '\0')
                {
                    --p_wifi_ConfigData->SecurityKeyLength;
                }
                memcpy(psk, cred.netKey, p_wifi_ConfigData->SecurityKeyLength);
            }
            break;

        default:
            //WF_ASSERT(false);
            break;
        } // end switch

        DRV_WIFI_ConfigDataSave();
        once = true;
    }
}
#endif /* WF_SAVE_WPS_CREDENTIALS == DRV_WIFI_ENABLED */

 /******************************************************************************
 * Function:        TCPIP_MAC_RES MRF24W_MACInit(const TCPIP_MAC_MODULE_CTRL* const stackData, const TCPIP_MODULE_MAC_MRF24W_CONFIG* initData)
 *
 * PreCondition:    None
 *
 * Input:           pNetIf   - network interface
 *
 * Output:          TCPIP_MAC_RES_OK if initialization succeeded,
 *                  error code otherwise
 *
 * Side Effects:    None
 *
 * Overview:        MRF24W_MACInit sets up the PIC's SPI module and all the
 *                  registers in the MRF24W so that normal operation can
 *                  begin. This function is called repeatedly until init is completed.
 *
 * Note:            When the system is using dynamically registered interrupts,
 *                  the WiFi interrupt handler should be registered here because
 *                  the DRV_WIFI_Init() enables the interrupts! 
 *****************************************************************************/
TCPIP_MAC_RES MRF24W_MACInit(const TCPIP_MAC_MODULE_CTRL* const stackData, const TCPIP_MODULE_MAC_MRF24W_CONFIG* initData)
{
    int retCode;

    // called repeatedly during init, DO NOT add any code here
    retCode = WF_InitStateMachine();

    g_pktAllocF = (_TCPIP_PKT_ALLOC_PTR)stackData->pktAllocF;
    g_pktFreeF = (_TCPIP_PKT_FREE_PTR)stackData->pktFreeF;
    g_pktAckF = (_TCPIP_PKT_ACK_PTR)stackData->pktAckF;

    return retCode;
}

void RawResetInitStateMachine(void)
{
    g_rawInitState = R_INIT_BEGIN;
}

int RawInitStateMachine(void)
{
    int retCode = TCPIP_MAC_RES_PENDING;
    int status;
    uint16_t byteCount;
    
    switch (g_rawInitState)
    {
        //-------------------------------------
        case R_INIT_BEGIN:
        //-------------------------------------
            // create raw move timers
            CreateRawMoveTimers();

            // By default the firmware mounts Scratch to RAW 1 after reset.  If desired,
            // we can read the SysInfo data block from the Scratch.  We are not using this
            // data, so unmount the scratch from this RAW window.
            ScratchUnmount(RAW_ID_1);
            g_rawInitState = R_WAIT_FOR_SCRATCH_UNMOUNT;
            break;

        //-------------------------------------
        case R_WAIT_FOR_SCRATCH_UNMOUNT:
        //-------------------------------------
            // if raw move completed
            if (isRawMoveComplete(RAW_ID_1, &status, &byteCount))
            {
                /* Mount scratch memory, index defaults to 0.  This will stay permanently mounted.   */
                /* If one needs to know, this function returns the number of bytes in scratch memory */
                ScratchMount(RAW_SCRATCH_ID);
                g_rawInitState = R_WAIT_FOR_SCRATCH_MOUNT;
            }
            // else if raw move not completed and it timed out
            else if (status == RM_TIMEOUT)
            {
                retCode = RAW_INIT_SCRATCH_UNMOUNT_FAIL;
            }
            break;

        //-------------------------------------
        case R_WAIT_FOR_SCRATCH_MOUNT:
        //-------------------------------------
            if (isRawMoveComplete(RAW_SCRATCH_ID, &status, &byteCount))
            {
                RawInit();  // complete raw init
                retCode = RAW_INIT_COMPLETE;
            }
            // else if raw move not completed and it timed out
            else if (status == RM_TIMEOUT)
            {
                retCode = RAW_INIT_SCRATCH_MOUNT_FAIL;
            }
            break;
    }

    return retCode;
}

/******************************************************************************
 * Function:        void RawInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Initializes the RAW window states.
 *
 * Note:            None
 *****************************************************************************/
static void RawInit(void)
{
    ClearAllIndexOutofBoundsFlags(); /* no raw indexes have been set past end of raw window */
}


/******************************************************************************
 * Function:        bool MRF24W_MACCheckLink(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          true: If the PHY reports that a link partner is present
 *                        and the link has been up continuously since the last
 *                        call to MRF24W_MACCheckLink()
 *                  false: If the PHY reports no link partner, or the link went
 *                         down momentarily since the last call to MRF24W_MACCheckLink()
 *
 * Side Effects:    None
 *
 * Overview:        Returns the PHSTAT1.LLSTAT bit.
 *
 * Note:            None
 *****************************************************************************/
bool MRF24W_MACCheckLink(void)
{
    return ( WFisConnected() );
}
 
/******************************************************************************
 * Function:        void MRF24W_MACPowerDown(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Placehholder function, not needed for actual operation, but
 *                  may be called by the MAC.
 *
 * Note:           None
 *****************************************************************************/
void MRF24W_MACPowerDown(void)
{
}//end MRF24W_MACPowerDown


/******************************************************************************
 * Function:        void MRF24W_MACPowerUp(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Placehholder function, not needed for actual operation, but
 *                  may be called by the MAC.
 *
 * Note:            None
 *****************************************************************************/
void MRF24W_MACPowerUp(void)
{
}//end MRF24W_MACPowerUp


// creates timers used to check for timeouts on data tx/rx raw moves
void CreateRawMoveTimers(void)
{

}


// called by RawMove()
void StartRawMoveTimer(uint16_t rawId)
{
    uint16_t timeout;

    timeout = SYS_TMR_TickPerSecond() * 4;

    if (rawId == RAW_DATA_RX_ID)
    {
        g_rxDataRawMoveTimeout = false;
        g_DataRxRawMoveTimer = SYS_TMR_CallbackPeriodic(1000 * 4, RawMoveDataRxTimeoutHandler);
    }
    else if (rawId == RAW_DATA_TX_ID)
    {
        g_txDataRawMoveTimeout = false;
        g_DataTxRawMoveTimer = SYS_TMR_CallbackPeriodic(1000 * 4, RawMoveDataTxTimeoutHandler);
    }
    else // must be RAW_SCRATCH_ID (only at init)
    {
        g_scratchRawMoveTimeout = false;
        g_ScratchRawMoveTimer = SYS_TMR_CallbackPeriodic(1000 * 4, RawMoveScratchTimeoutHandler);
    }
}
void StopRawMoveTimer(uint16_t rawId)
{
    if (rawId == RAW_DATA_RX_ID)
    {
       SYS_TMR_CallbackDeregister_wifi(g_DataRxRawMoveTimer);
    }
    else if (rawId == RAW_DATA_TX_ID)
    {
        SYS_TMR_CallbackDeregister_wifi(g_DataTxRawMoveTimer);
    }
    else // must be RAW_SCRATCH_ID (only at init)
    {
        SYS_TMR_CallbackDeregister_wifi(g_ScratchRawMoveTimer);
    }
}

bool isRawMoveTimeout(uint16_t rawId)
{
    if (rawId == RAW_DATA_RX_ID)
    {
        return (g_rxDataRawMoveTimeout);
    }
    else if (rawId == RAW_DATA_TX_ID)
    {
        return (g_txDataRawMoveTimeout);
    }
    else // must be RAW_SCRATCH_ID (only at init)
    {
        return (g_scratchRawMoveTimeout);
    }
}

// called from timer interrupt when timeout occurs
static void RawMoveDataRxTimeoutHandler(void)
{
    g_rxDataRawMoveTimeout = true;
    SignalRxStateMachine();
    StopRawMoveTimer(RAW_DATA_RX_ID);
}


// called from timer interrupt when timeout occurs
static void RawMoveDataTxTimeoutHandler(void)
{
    g_txDataRawMoveTimeout = true;
    SignalTxStateMachine();
    StopRawMoveTimer(RAW_DATA_TX_ID);
}

static void RawMoveScratchTimeoutHandler(void)
{
    g_scratchRawMoveTimeout = true;
    StopRawMoveTimer(RAW_SCRATCH_ID);
}


void InitDataTxStateMachine(void)
{
    g_txDataState = DATA_TX_IDLE;
    g_txWaitingForRxIdle = false;
    
    // create a queue to hold data tx messages that need to be sent to MRF
    TCPIP_Helper_SingleListInitialize(&g_dataTxQueue);
}

void InitDataRxStateMachine(void)
{
    int i;
    
    g_rxDataState = DATA_RX_IDLE;
    g_rxPacketPending = false;

    // create a queue to hold pointers to preallocated Rx packets
    TCPIP_Helper_SingleListInitialize(&g_dataRxQueue);

    for (i = 0; i < NUM_PREALLOCATED_RX_PACKETS; ++i)
    {
        // preallocate Rx buffers to store Rx packets as they come in (1500 bytes data plus header and checksum)
        RxBuffer_packet[i] = g_pktAllocF ? _TCPIP_PKT_ALLOC_BY_PTR(g_pktAllocF, sizeof(TCPIP_MAC_PACKET), 1518, 0) : 0;
        if (RxBuffer_packet[i] != NULL)
        {
            RxBuffer_packet[i]->next = NULL;
            RxBuffer_packet[i]->ackFunc = RxDataCallback;
            RxBuffer_packet[i]->ackParam = NULL;
            RxBuffer_packet[i]->pktFlags = 0;
            TCPIP_Helper_SingleListTailAdd(&g_dataRxQueue, (SGL_LIST_NODE *)RxBuffer_packet[i]);
        }
        else
        {
            SYS_ASSERT(false, "");
        }
    }

    // create a another FIFO to store pointers
    FifoInit(&g_rxFifo);
}

void DeInitDataRxStateMachine(void)
{
    int i;
    if(g_pktFreeF != NULL)
    {
        for (i = 0; i < NUM_PREALLOCATED_RX_PACKETS; ++i)
        {
            if(RxBuffer_packet[i] != NULL)
            {
                _TCPIP_PKT_FREE_BY_PTR(g_pktFreeF, RxBuffer_packet[i]);
                RxBuffer_packet[i] = NULL;
            }
        }
    }
}

static void FifoInit(t_fifo *p_fifo)
{
    memset(p_fifo, 0x00, sizeof(t_fifo));
    p_fifo->front = p_fifo->rear = NUM_PREALLOCATED_RX_PACKETS - 1;
}

static bool isFifoEmpty(t_fifo *p_fifo)
{
    return p_fifo->front == p_fifo->rear;
}

static void FifoInsert(t_fifo *p_fifo, TCPIP_MAC_PACKET *p_packet)
{
    if (p_fifo->rear == NUM_PREALLOCATED_RX_PACKETS - 1)
    {
        p_fifo->rear = 0;
    }
    else
    {
        ++p_fifo->rear;
    }

    p_fifo->items[p_fifo->rear] = p_packet;
}

static TCPIP_MAC_PACKET * FifoRemove(t_fifo *p_fifo)
{
    if (p_fifo->front == NUM_PREALLOCATED_RX_PACKETS - 1)
    {
        p_fifo->front = 0;
    }
    else
    {
        ++p_fifo->front;
    }

    return p_fifo->items[p_fifo->front];
}

// called by stack when done with Rx packet
static void RxDataCallback(TCPIP_MAC_PACKET * pktHandle, const void* ackParam)
{
    ackParam = ackParam;  // ignore warning

    if(pktHandle)
    {
        pktHandle->pktFlags &= ~TCPIP_MAC_PKT_FLAG_QUEUED;
        TCPIP_Helper_SingleListTailAdd(&g_dataRxQueue, (SGL_LIST_NODE *)pktHandle);  // add packet back to free list
    }
    else
    {
        SYS_CONSOLE_MESSAGE("!!!\r\n");
    }
}

// called when Raw move complete event occurs
void WiFi_DataRxTask(void)
{
    int status;
    uint16_t byteCount;
    uint16_t ethPacketLength;  // includes header
    TCPIP_MAC_PACKET *p_packet;
    const TCPIP_MAC_MODULE_CTRL *p_ctrl = GetStackData();
    const uint8_t* p_Mac = TCPIP_STACK_NetAddressMac(TCPIP_STACK_IndexToNet(p_ctrl->netIx));

    DecrementRxPendingCount();
    SYS_INT_SourceDisable(MRFWB0M_INT_SOURCE);

    DBG_RX_PRINT("RX: ");
    switch (g_rxDataState)
    {
        //======================================================================
        case DATA_RX_IDLE:
            DBG_RX_PRINT("0 ");
            if (isTxStateMachineIdle())
            {
                DBG_RX_PRINT("1 ");
                RawMountRxDataBuffer();   // start the raw Rx data mount
                g_rxDataState = WAIT_FOR_DATA_RX_MOUNT;
            }
            else
            {
                DBG_RX_PRINT("2 ");
                // schedule state machine to run again
                SignalRxStateMachine();
                IncrementRxPendingCount();
                g_rxDataState = DATA_RX_WAIT_FOR_TX_IDLE;
            }
            break;

        //======================================================================
        case DATA_RX_WAIT_FOR_TX_IDLE:
            // if tx state machine is idle, or tx state machine is waiting for Rx state machine
            if (isTxStateMachineIdle())
            {
                DBG_RX_PRINT("3 ");
                // then run this (rx) state machine and let tx state machine wait
                RawMountRxDataBuffer();   // start the raw Rx data mount
                g_rxDataState = WAIT_FOR_DATA_RX_MOUNT;
            }
            else
            {
                DBG_RX_PRINT("4 ");
                // schedule this state machine to run again (and recheck if tx is idle)
                SignalRxStateMachine();
                IncrementRxPendingCount();
            }
            break;

        //======================================================================
        case WAIT_FOR_DATA_RX_MOUNT:
            // this should always be true as the event is only triggered, and this
            // function should only be called, after the Raw Move has completed.
            if (isRawMoveComplete(RAW_DATA_RX_ID, &status, &byteCount))
            {
                DBG_RX_PRINT("5 ");
                //SYS_ASSERT((byteCount > 0) && (byteCount <= 1530), "Rx Packet is larger than 1528 bytes for UDP or 1530 bytes for TCP\r\n");
                { // for debugging
                        char buf_t[20];
                        if( !(byteCount > 0) && (byteCount <= 1530))
                        {    
                            sprintf(buf_t,"%d\r\n",byteCount);
                            SYS_CONSOLE_MESSAGE("Rx Packet is larger than 1528 bytes for UDP or 1530 bytes for TCP,  byteCount=");
                            SYS_CONSOLE_MESSAGE(buf_t);
                            
                        }
                }
                // if raw move successfully completed and the Rx packet header is valid
                if ((status == RM_COMPLETE) && (isRxPacketHeaderValid()))
                {
                    DBG_RX_PRINT("6 ");
                    if (g_rxDataRawMoveTimeout)
                    {
                        g_rxDataRawMoveTimeout = false;
                        SYS_CONSOLE_MESSAGE("RMTO\r\n");
                    }

                    // get an Rx buffer structure from queue to copy rx packet to (from MRF to Rx buffer struct)
                    p_packet = GetAvailRxBuf();

                    // if packet structure available
                    if (p_packet != NULL)
                    {
                        DBG_RX_PRINT("7 ");
                        // read Ethernet packet into host buffer
                        // set raw pointer to start of 802.11 payload (start of Ethernet packet)
                        ethPacketLength = byteCount - ETH_HEADER_START_OFFSET;
                        RawRead(RAW_DATA_RX_ID, ETH_HEADER_START_OFFSET, ethPacketLength, p_packet->pDSeg->segLoad);

                        // if we received our own broadcast then throw it away
                        if ( memcmp(&p_packet->pDSeg->segLoad[6], p_Mac, 6) == 0 )
                        {
                            DBG_RX_PRINT("8 ");
                            // put buffer back in free list
                            TCPIP_Helper_SingleListTailAdd(&g_dataRxQueue, (SGL_LIST_NODE *)p_packet); 
                        }
                        // else flag packet as queued and signal stack to process it
                        else
                        {
                            DBG_RX_PRINT("9 ");
                            // mark packet as queued and stuff in timestamp
                            p_packet->pDSeg->segLen = ethPacketLength - ETH_HEADER_SIZE;
                            p_packet->pktFlags |= TCPIP_MAC_PKT_FLAG_QUEUED;
                            p_packet->tStamp = SYS_TMR_TickCountGet();

                            // Note: re-set pMacLayer and pNetLayer; IPv6 changes these pointers inside the packet, so
                            //       when Rx packets are reused this is needed.
                            p_packet->pMacLayer = p_packet->pDSeg->segLoad;
                            p_packet->pNetLayer = p_packet->pMacLayer + sizeof(TCPIP_MAC_ETHERNET_HEADER);

                            // store packet pointer in FIFO and signal stack that rx packet ready to process
                            FifoInsert(&g_rxFifo, p_packet);

                            // notify stack of Rx packet
                           // SYS_CONSOLE_MESSAGE(" NS\r\n");
                            WF_UserEventsSet(TCPIP_EV_RX_DONE, 0, false);
                        }
                    }
                    // else out of pre-queued Rx packets
                    else
                    {
                        SYS_CONSOLE_MESSAGE("Out of Rx -- throw away packet\r\n");
                    }
                }
                // else if raw move timed out (note: invalid, proprietary packet is OK, so does not cause assert)
                else if (status == RM_TIMEOUT)
                {
                    SYS_ASSERT(false, "Should never happen");
                }
                else
                {
                    // We were hitting this case when a proprietary packet was received, so we should
                    // not assert, but simply throw packet away, and not assert as we had been doing.
                    ;
                }

                // begin raw unmount of data rx packet (it's either been buffered for stack or being thrown away)
                DeallocateDataRxBuffer();
                g_rxDataState = WAIT_FOR_DATA_RX_UNMOUNT;
            }
            // else Raw Move not yet complete (still waiting)
            else
            {
                SYS_ASSERT(false, "Should never happen");
            }
            break;

        //======================================================================
        case WAIT_FOR_DATA_RX_UNMOUNT:
            if (isRawMoveComplete(RAW_DATA_RX_ID, &status, &byteCount))
            {
                DBG_RX_PRINT("10 ");
                // if timed out waiting for raw move to complete
                if (g_rxDataRawMoveTimeout)
                {
                    g_rxDataRawMoveTimeout = false;
                    DBG_RX_PRINT("RUTO\r\n");
//                    SYS_ASSERT(false, "Should never happen");
                }
                g_rxDataState = DATA_RX_IDLE;

                // if notified of new rx packet while waiting for Raw move complete
                if (g_rxPacketPending == true)
                {
                    DBG_RX_PRINT("12 ");
                    // kick-start state machine again
                    g_rxPacketPending = false;
                    SignalRxStateMachine();
                    IncrementRxPendingCount();
                }
                // else if tx state machine waiting for rx state machine to go idle
                else if (g_txWaitingForRxIdle == true)
                {
                    DBG_RX_PRINT("11 ");
                    // kick-start Tx state machine so it can finish
                    g_txWaitingForRxIdle = false;
                    SignalTxStateMachine();
                }

            }
            else
            {
                SYS_ASSERT(false, "Should never happen");
            }
            break;
    } // end switch

    DBG_RX_PRINT("\r\n");
    DRV_WIFI_INT_SourceEnable();


}

static bool isRxPacketHeaderValid(void)
{
     t_RxPreamble    wfPreamble;
     uint8_t         snapHdr[6];
     
    // read the data frame internal preamble (type and subtype bytes) to verify that we did, in
    // fact, mount an Rx data packet.  This read auto-increments the raw index to the first
    // actual data byte in the frame.
    //RawGetByte(RAW_DATA_RX_ID, (uint8_t*)&wfPreamble, sizeof(t_RxPreamble));
    RawRead(RAW_DATA_RX_ID, 0, sizeof(t_RxPreamble), (uint8_t*)&wfPreamble);
    SYS_ASSERT((wfPreamble.type == WF_DATA_RX_INDICATE_TYPE), "");  // should never fail!

    // read snap header in Rx packet and verify it; if snap header is not valid
    // this is a proprietary packet that will be thrown away by Rx state machine.
    RawRead(RAW_DATA_RX_ID, SNAP_HEADER_OFFSET, 6, (uint8_t *)&snapHdr);

    // verify that the expected bytes contain the SNAP header; if not, this is a
    // proprietary packet that will be thrown away by Rx state machine.
    if (!(snapHdr[0] == SNAP_VAL        &&
          snapHdr[1] == SNAP_VAL        &&
          snapHdr[2] == SNAP_CTRL_VAL   &&
          snapHdr[3] == SNAP_TYPE_VAL   &&
          snapHdr[4] == SNAP_TYPE_VAL   &&
          snapHdr[5] == SNAP_TYPE_VAL) )
    {
        return false;
    }

    return true;
}

// finds an available Rx packet structure from the list that was allocated and
// queued up at init.
static TCPIP_MAC_PACKET * GetAvailRxBuf(void)
{
    TCPIP_MAC_PACKET *p_packet = NULL;

    // if free list has an available Rx packet buffer
    if (g_dataRxQueue.nNodes > 0)
    {
        p_packet = (TCPIP_MAC_PACKET *)TCPIP_Helper_SingleListHeadRemove(&g_dataRxQueue);
        SYS_ASSERT(p_packet != NULL, "Should never happen");
    }

    return p_packet;
}

bool isDataTxTaskInactive(void)
{
    return (g_txDataState == DATA_TX_IDLE);
}

// retrieve the oldest of the queued Rx packets to deliver to the stack
TCPIP_MAC_PACKET * MRF24W_GetRxPacket(void)
{
    if (!isFifoEmpty(&g_rxFifo))
    {

        return FifoRemove(&g_rxFifo);
    }
    else
    {
        return NULL;  // signals no rx packet to process
    }
}


static uint16_t GetTxPacketLength(TCPIP_MAC_PACKET *p_packet)
{
    uint16_t packetLength;
    TCPIP_MAC_DATA_SEGMENT *p_seg = p_packet->pDSeg;  // point to first segment

    // number of bytes in first segment
    packetLength = p_seg->segLen;

    // loop thru any other segments and add their data length
    while (p_seg->next != NULL)
    {
        p_seg = p_seg->next;
        packetLength += p_seg->segLen;
    }

    // return the packet length plus extra 4 bytes needed for internal header
    return packetLength + WF_TX_DATA_MSG_PREAMBLE_LENGTH;
}


// called by mrf24w_MACTxPacket when stack wants to send a packet
TCPIP_MAC_RES MRF24W_TxPacket(TCPIP_MAC_PACKET *p_packet)
{
    if (DRV_WIFI_isHibernateEnable())
    {
        SYS_CONSOLE_MESSAGE("No data tx when Wi-Fi in hibernate mode.\r\n");
        return TCPIP_MAC_RES_QUEUE_TX_FULL;
    }

    // if we get too many tx packets queued up, start throwing them away
    if (g_dataTxQueue.nNodes >= MAX_TX_QUEUE_COUNT)
    {
        SYS_CONSOLE_PRINT("Max Tx Packets (%d)\r\n", g_dataTxQueue.nNodes);
        return TCPIP_MAC_RES_QUEUE_TX_FULL;
    }

    // queue the Tx Data packet pointer
    TCPIP_Helper_SingleListTailAdd(&g_dataTxQueue, (SGL_LIST_NODE *)p_packet);

    // signal list manager that this packet cannot be reused until the 
    // ack function is called.
    p_packet->pktFlags |= TCPIP_MAC_PKT_FLAG_QUEUED;

    // if Tx Data Task is idle and there is only a single message in the queue then
    // kick-started the Tx Data Task.  If there is more than one message in the queue
    // then we've had at least two calls to this function before the Tx Data Task had
    // a chance to run, but we only want to call it once, not twice.  The task
    // will check for additional tx packets when it finishes the previous one.
    if ((g_txDataState == DATA_TX_IDLE) && (g_dataTxQueue.nNodes == 1))  // node check debug
    {
        WiFi_DataTxTask();
    }

    return TCPIP_MAC_RES_OK;
}

static void PrepTxPacketHeaders(TCPIP_MAC_PACKET *p_packet)
{
    uint8_t  *p_load;
    TCPIP_MAC_ETHERNET_HEADER *p_etherHeader;
    uint16_t type;

    p_etherHeader = (TCPIP_MAC_ETHERNET_HEADER *)(p_packet->pMacLayer);
    type = p_etherHeader->Type;

    // overwrite source address in Ethernet header with SNAP header.  The Ethernet
    // source address starts at index 6 in the Ethernet packet
    p_load = p_packet->pDSeg->segLoad; // start of packet
    
    // overwrite bytes [6] thru [11] in ethernet header to SNAP header
    p_load[6]  = SNAP_VAL;
    p_load[7]  = SNAP_VAL;
    p_load[8]  = SNAP_CTRL_VAL;
    p_load[9]  = SNAP_TYPE_VAL;
    p_load[10] = SNAP_TYPE_VAL;
    p_load[11] = SNAP_TYPE_VAL;

    // overwrite indexes [12], [13] with Ethernet type for protocol being used (already in network order)
    p_load[12]   = (uint8_t)type;      
    p_load[13] = (uint8_t)(type >> 8); 
    
    // now that the packet has been modified for WiFi mac, prepend the internal msg
    // header used by SPI interface.  These four bytes are immediately prior to the
    // p_DSeg pointer (the actual 'secret' buffer is the size of segLoadOffset).
    p_load = p_packet->pDSeg->segLoad - WF_TX_PREAMBLE_SIZE;  // have p_load point to 4 bytes before pDSeg->segLoad

    // write out internal preamble
    p_load[0] = WF_DATA_REQUEST_TYPE;
    p_load[1] = WF_STD_DATA_MSG_SUBTYPE;
    p_load[2] = 1;
    p_load[3] = 0;
}

static bool isTxStateMachineIdle(void)
{
    // tx state machine is logically idle if its state is idle or it is waiting for the rx state machine to go idle
    return (g_txDataState == DATA_TX_IDLE) || (g_txDataState == WAIT_FOR_RX_IDLE); 
}

bool isRxStateMachineIdle(void)
{
    return (g_rxDataState == DATA_RX_IDLE);
}

void SetRxPacketPending(void)
{
    g_rxPacketPending = true;
}

void WiFi_DataTxTask(void)
{
    int status;
    uint16_t byteCount = 0;
    uint8_t *p_segData;
    TCPIP_MAC_DATA_SEGMENT *p_seg;
    static TCPIP_MAC_PACKET *p_packet = NULL;  // packet currently being sent
    static uint16_t packetLength = 0;       // total length of buffer needed on MRF
    bool txStateMachineActive = true;

    SYS_INT_SourceDisable(MRFWB0M_INT_SOURCE);
    DBG_TX_PRINT("TX: ");

    while (txStateMachineActive)
    {
        txStateMachineActive = false;

        switch (g_txDataState)
        {
            //======================================================================
            case DATA_TX_IDLE:
                DBG_TX_PRINT("1 ");
                // if any tx packets in queue
                if (g_dataTxQueue.nNodes > 0)
                {
                    DBG_TX_PRINT("2 ");
                    // point to Tx packet, but do not yet remove it from the queue
                    p_packet = (TCPIP_MAC_PACKET *)g_dataTxQueue.head;
                    SYS_ASSERT(p_packet != NULL, "");
                    // prepend internal WiFi header and modify Ethernet header to SNAP; get packet length
                    PrepTxPacketHeaders(p_packet);
                    packetLength = GetTxPacketLength(p_packet);

                    if (isRxStateMachineIdle())
                    {
                        DBG_TX_PRINT("3 ");
                        // start raw mount if bytes available
                        if (AllocateDataTxBuffer(packetLength))
                        {
                            DBG_TX_PRINT("4 ");
                            g_txDataState = WAIT_FOR_DATA_TX_MOUNT;
                        }
                        // else not enough bytes available on MRF right now
                        else
                        {
                            DBG_TX_PRINT("5 ");
                            // schedule state machine to run again and check if it can proceed
                            SignalTxStateMachine();    
                            g_txDataState = WAIT_FOR_TX_MEMORY;
                        }
                    }
                    // else need to wait for Rx state machine to finish
                    else
                    {
                        DBG_TX_PRINT("6 ");
                        g_txDataState = WAIT_FOR_RX_IDLE;
                        g_txWaitingForRxIdle = true;  // let rx state machine know that tx state machine is waiting for it
                    }
                }
                else
                {
                    SYS_ASSERT(p_packet != NULL, "Should never happen\r\n");
                }
                break;

            //======================================================================
            case WAIT_FOR_RX_IDLE:
                DBG_TX_PRINT("7 ");
                if (isRxStateMachineIdle())  // should always be true
                {
                    // start raw mount if bytes available
                    if (AllocateDataTxBuffer(packetLength))
                    {
                        DBG_TX_PRINT("8 ");
                        g_txDataState = WAIT_FOR_DATA_TX_MOUNT;
                    }
                    // else not enough bytes available on MRF right now
                    else
                    {
                        DBG_TX_PRINT("9 ");
                        // schedule state machine to run again
                        SignalTxStateMachine();
                        g_txDataState = WAIT_FOR_TX_MEMORY;
                    }
                }
                else
                {
                    SYS_ASSERT(false, "Should never happen");
                }
                break;

            //======================================================================
            case WAIT_FOR_TX_MEMORY:
                DBG_TX_PRINT("10 ");
                if (isRxStateMachineIdle())
                {
                    DBG_TX_PRINT("11 ");
                    // attempt to allocate data buffer on MRF (if bytes available then Raw Move has been initiated)
                    if (AllocateDataTxBuffer(packetLength))
                    {
                        DBG_TX_PRINT("12 ");
                        g_txDataState = WAIT_FOR_DATA_TX_MOUNT;
                    }
                    // else not enough bytes available on MRF right now
                    else
                    {
                        DBG_TX_PRINT("13 ");
                        // schedule state machine to run again
                        SignalTxStateMachine();   // should notmake a diffeerence
                    }
                }
                else
                {
                    DBG_TX_PRINT("14 ");
                    g_txDataState = WAIT_FOR_RX_IDLE;
                    g_txWaitingForRxIdle = true;  // let rx state machine know that tx state machine is waiting for it
                }
                break;

            //======================================================================
            case WAIT_FOR_DATA_TX_MOUNT:
                DBG_TX_PRINT("15 ");
                // if tx data buffer successfully mounted (or timed out trying)
                if (isRawMoveComplete(RAW_DATA_TX_ID, &status, &byteCount))
                {
                    DBG_TX_PRINT("16 ");
                    if ((status == RM_COMPLETE) && (byteCount != 0))
                    {
                        DBG_TX_PRINT("17 ");
                        // if, after the raw mount, we didn't get all the bytes we needed
                        if (byteCount < packetLength)
                        {
                            SYS_CONSOLE_MESSAGE("IVB\r\n");
                        }

                        uint16_t curIndex = 0;

                        SYS_ASSERT(g_txDataRawMoveTimeout == false,"Should never happen");
                        // write out the first segment to MRF, including prepended internal header
                        p_seg = p_packet->pDSeg;  // point to first segment
                        p_segData = p_seg->segLoad - WF_TX_PREAMBLE_SIZE;
                        RawWrite(RAW_DATA_TX_ID, 0, p_seg->segLen + WF_TX_PREAMBLE_SIZE, p_segData);
                        curIndex += p_seg->segLen + WF_TX_PREAMBLE_SIZE;

                        // write out any other segments to MRF
                        while (p_seg->next != NULL)
                        {
                            DBG_TX_PRINT("18 ");
                            p_seg = p_seg->next;
                            p_segData = p_seg->segLoad;
                            RawWrite(RAW_DATA_TX_ID, curIndex, p_seg->segLen, p_segData);
                            curIndex += p_seg->segLen;
                        }

                        // Now that full packet copied to MRF24WG, signal it that packet ready to transmit (start raw move)
                        SendRAWDataFrame(packetLength);

                        g_txDataState = WAIT_FOR_DATA_TX_SIGNAL;

                    }
                    else if (byteCount == 0)
                    {
                        SYS_ASSERT(false, "Should never happen");
                    }
                }
                else
                {
                    SYS_ASSERT(false, "Should never happen");
                }
                break;

            //======================================================================
            case WAIT_FOR_DATA_TX_SIGNAL:
                DBG_TX_PRINT("19 ");
                // if raw move complete that signals MRF24WG that there is a tx data
                // packet to transmit
                if (isRawMoveComplete(RAW_DATA_TX_ID, &status, &byteCount))
                {
                    DBG_TX_PRINT("20 ");
                    if (status == RM_COMPLETE)
                    {
                        DBG_TX_PRINT("21 ");
                        if (g_txDataRawMoveTimeout)
                        {
                            g_txDataRawMoveTimeout = false;
                            DBG_TX_PRINT("TUTO\r\n");
                            //SYS_ASSERT(false, "Should never happen\r\n");
                        }

                        // remove tx packet from the list
                        p_packet = (TCPIP_MAC_PACKET *)TCPIP_Helper_SingleListHeadRemove(&g_dataTxQueue);
                        //SYS_CONSOLE_PRINT("R=%d\r\n", g_dataTxQueue.nNodes);

                        // call stack ack function to let it know packet was transmitted
                        if(g_pktAckF)
                        {
                            _TCPIP_PKT_ACK_BY_PTR(g_pktAckF, p_packet, TCPIP_MAC_PKT_ACK_TX_OK);
                        }
                        else
                        {
                            SYS_ASSERT(false, "Should never happen");
                        }

                        // if any pending tx packets
                        if (g_dataTxQueue.nNodes > 0)
                        {
                            DBG_TX_PRINT("22 ");
                            txStateMachineActive = true;
                        }

                        g_txDataState = DATA_TX_IDLE; // ready for next tx data
                    }
                    else
                    {
                        SYS_ASSERT(false, "Should never happen");
                    }
                }
                else
                {
                    SYS_ASSERT(false, "Should never happen");
                }
                break;
        } // end switch

    } // end while

    DRV_WIFI_INT_SourceEnable();

    DBG_TX_PRINT("\r\n");
}


void wifi_release_tx_heap_when_Redirection(void)
{
    TCPIP_MAC_PACKET *p_packet = NULL;
    // remove tx packet from the list
    while(g_dataTxQueue.nNodes > 0)
    {
        p_packet = (TCPIP_MAC_PACKET *)TCPIP_Helper_SingleListHeadRemove(&g_dataTxQueue);
        // call stack ack function to let it know packet was transmitted
        if(g_pktAckF)
        {
            _TCPIP_PKT_ACK_BY_PTR(g_pktAckF, p_packet, TCPIP_MAC_PKT_ACK_TX_OK);
        }
        else
        {
            SYS_ASSERT(false, "Should never happen");
        }
    }  
}

#endif /* TCPIP_IF_MRF24W*/

//DOM-IGNORE-END
