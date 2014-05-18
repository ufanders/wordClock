/*******************************************************************************
  TCP/IP Stack Manager

  Summary:
    Module for Microchip TCP/IP Stack

  Description:
    -Handles internal RX packet pre-processing prior to dispatching
     to upper application layers.
    -Reference: AN833
*******************************************************************************/

/*******************************************************************************
File Name:  tcpip_manager.c
Copyright ©2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED ?AS IS?WITHOUT WARRANTY OF ANY KIND,
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

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_TCPIP_MANAGER

#include "tcpip/src/tcpip_mac_private.h"

#include "tcpip_mac_object.h"

#include "tcpip_module_manager.h"

#define TCPIP_STACK_INIT_MESSAGE   "TCP/IP Stack Initialization "

#include "tcpip_notify.h"
#include "system/tmr/sys_tmr.h"


static TCPIP_NET_IF* tcpipNetIf = 0;       // dynamically allocated

// Main default interfaces
typedef struct
{
    TCPIP_NET_IF* defaultNet;     // default network interface
}TCPIPDefaultIF;


static TCPIPDefaultIF tcpipDefIf = { 0 };

static volatile int    totTcpipEventsCnt = 0;
static volatile int    newTcpipTickAvlbl = 0;

static volatile int    newTcpipErrorEventCnt = 0;
static volatile int    newTcpipStackEventCnt = 0;


static void    _TCPIP_MacEventCB(TCPIP_MAC_EVENT event, const void* hParam);

static uint8_t*    tcpip_heap = 0;          // the actual TCPIP heap

static TCPIP_STACK_MODULE_CTRL  tcpip_stack_ctrl_data = {0};

//
static SYS_TMR_HANDLE    tcpip_stack_tickH = 0;      // tick handle

static uint32_t            stackTaskRate;   // actual task running rate

static TCPIP_STACK_ASYNC_MODULE_ENTRY  TCPIP_STACK_MODULE_ASYNC_TBL [TCPIP_STACK_ASYNC_MODULE_ENTRIES] = { {0} };


static const TCPIP_NETWORK_CONFIG tcpip_def_config = 
{
    TCPIP_NETWORK_DEFAULT_INTERFACE_NAME,
    TCPIP_NETWORK_DEFAULT_HOST_NAME,
    TCPIP_NETWORK_DEFAULT_MAC_ADDR,
    TCPIP_NETWORK_DEFAULT_IP_ADDRESS,
    TCPIP_NETWORK_DEFAULT_IP_MASK,
    TCPIP_NETWORK_DEFAULT_GATEWAY,
    TCPIP_NETWORK_DEFAULT_DNS,
    TCPIP_NETWORK_DEFAULT_SECOND_DNS,
    TCPIP_NETWORK_DEFAULT_POWER_MODE,
    TCPIP_NETWORK_DEFAULT_INTERFACE_FLAGS,
    TCPIP_NETWORK_DEFAULT_IPV6_ADDRESS,
    TCPIP_NETWORK_DEFAULT_IPV6_PREFIX_LENGTH,
    TCPIP_NETWORK_DEFAULT_IPV6_GATEWAY,
};

static void _TCPIP_STACK_TickHandler();        // stack tick handler

static void _TCPIP_ProcessTickEvent(void);
static void ProcessTCPIPMacRxEvents(TCPIP_NET_IF* pNetIf);
static void _TCPIP_ProcessMACErrorEvents(TCPIP_NET_IF* pNetIf, TCPIP_MAC_EVENT activeEvent);

static bool _InitNetConfig(const TCPIP_NETWORK_CONFIG* pUsrConfig, int nNets);
static bool _LoadDefaultConfig(const TCPIP_NETWORK_CONFIG* pUsrConfig, TCPIP_NET_IF* pNetIf);

static const TCPIP_STACK_MODULE_CONFIG* _TCPIP_STACK_FindModuleData(TCPIP_STACK_MODULE moduleId, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules);

static const TCPIP_STACK_MODULE_MAC_ENTRY* _TCPIP_STACK_FindMacModule(TCPIP_STACK_MODULE moduleId);

static void  TCPIP_STACK_BringNetDown(TCPIP_STACK_MODULE_CTRL* stackCtrlData);
static bool  TCPIP_STACK_BringNetUp(TCPIP_STACK_MODULE_CTRL* stackCtrlData, const TCPIP_NETWORK_CONFIG* pNetConf, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules);

static void* _NetConfigStringToBuffer(void** ppDstBuff, void* pSrcBuff, size_t* pDstSize, size_t* pNeedLen, size_t* pActLen);

static TCPIP_MAC_ACTION TCPIP_STACK_StackToMacAction(TCPIP_STACK_ACTION action);

static void TCPIP_STACK_StacktoMacCtrl(TCPIP_MAC_MODULE_CTRL* pMacCtrl, TCPIP_STACK_MODULE_CTRL* stackCtrlData);

static bool TCPIP_STACK_ProcessPending(void);

static void _TCPIPStackProcessTmo(void);

static void _TCPIPStackRxListPurge(TCPIP_NET_IF* pNetIf);

static __inline__ void __attribute__((always_inline)) _TCPIPStackRxListInsert(TCPIP_NET_IF* pNetIf, TCPIP_MAC_PACKET* pRxPkt)
{
    pRxPkt->pktIf = pNetIf;
    TCPIP_Helper_ProtectedSingleListTailAdd(&pNetIf->rxQueue, (SGL_LIST_NODE*)pRxPkt);
}

// Note the TCPIP_EVENT enum is aligned with the TCPIP_MAC_EVENT!
static __inline__ TCPIP_EVENT __attribute__((always_inline)) TCPIP_STACK_Mac2TcpipEvent(TCPIP_MAC_EVENT macEvent)
{
    return (TCPIP_EVENT)macEvent;
}

/*********************************************************************
 * Function:        bool TCPIP_STACK_Initialize(const TCPIP_NETWORK_CONFIG* pNetConf, int nNets, TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules)
 *
 * PreCondition:    None
 *
 * Input:           pNetConf      - pointer to an array of TCPIP_NETWORK_CONFIG to support
 *                  nNets       - number of network configurations in the array
 *                  pModConfig  - pointer to an array of TCPIP_STACK_MODULE_CONFIG
 *                  nModules    - number of modules to initialize
 *
 * Output:          true if Stack and its componets are initialized
 *                  false otherwise
 *
 * Overview:        The function initializes the stack.
 *                  If an error occurs, the SYS_ERROR() is called
 *                  and the function de-initialize itself and will return false.
 *
 * Side Effects:    None
 *
 * Note:            This function must be called before any of the
 *                  stack or its component routines are used.
 *
 *                  New TCPIP_NETWORK_CONFIG types should be added/removed at run time for implementations that support
 *                  dynamic network interface creation.
 *
 ********************************************************************/
bool TCPIP_STACK_Initialize(const TCPIP_NETWORK_CONFIG* pUsrConfig, int nNets, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules)
{
    int                     netIx;
    bool                    initFail;
    TCPIP_HEAP_HANDLE       heapH;
    TCPIP_NET_IF*           pIf;
    TCPIP_MAC_POWER_MODE    powerMode;

    if(tcpipNetIf != 0)
    {   // already up and running
        return true;
    }

    if(pUsrConfig == 0)
    {
        pUsrConfig = & tcpip_def_config;
        nNets = 1;  // TCPIP_NETWORK_INTERFACES_NO; hard coded!
    }
    else if(nNets == 0)
    {   // cannot run with no interface
        return false;
    }

    SYS_CONSOLE_MESSAGE(TCPIP_STACK_INIT_MESSAGE "Started \n\r");


    while(true)
    {
        initFail = false;

        totTcpipEventsCnt = 0;

        newTcpipErrorEventCnt = 0;
        newTcpipStackEventCnt = 0;
        newTcpipTickAvlbl = 0;

        // start stack initialization

        memset(&tcpip_stack_ctrl_data, 0, sizeof(tcpip_stack_ctrl_data));

        tcpip_heap = (uint8_t*)SystemMalloc(TCPIP_STACK_DRAM_SIZE);

        heapH = TCPIP_HEAP_Create(tcpip_heap, TCPIP_STACK_DRAM_SIZE, 0, 0);     // get handle to the heap memory

        if((tcpip_stack_ctrl_data.memH = heapH) == 0)
        {
            SYS_ERROR_PRINT(SYS_ERROR_ERROR, "Heap creation failed, size: %d\r\n", TCPIP_STACK_DRAM_SIZE);
            initFail = true;
            break;
        }

        tcpipNetIf = (TCPIP_NET_IF*)TCPIP_HEAP_Calloc(heapH, nNets, sizeof(TCPIP_NET_IF)); // allocate for each network interface
        if(tcpipNetIf == 0)
        {   // failed
            SYS_ERROR(SYS_ERROR_ERROR, "Network configuration allocation failed\r\n");
            initFail = true;
            break;
        }

        if(TCPIP_PKT_Initialize(heapH) == false)
        {
            SYS_ERROR(SYS_ERROR_ERROR, "TCPIP Packet initialization failed\r\n");
            initFail = true;
            break;
        }

        tcpip_stack_ctrl_data.nIfs = nNets;
        tcpip_stack_ctrl_data.nMdls = nModules;

        if(!_InitNetConfig(pUsrConfig, nNets))
        {
            SYS_ERROR(SYS_ERROR_ERROR, "Network configuration initialization failed\r\n");
            initFail = true;   // failed the initialization
            break;
        }

        tcpipDefIf.defaultNet = 0;          // delete the old default
        // initialize the tmo/async handlers
        memset(TCPIP_STACK_MODULE_ASYNC_TBL, 0x0, sizeof(TCPIP_STACK_MODULE_ASYNC_TBL));
#if defined(TCPIP_IF_MRF24W)
        // statically initialize the WiFi MAC entry
        TCPIP_STACK_MODULE_ASYNC_TBL[0].asyncHandler = WiFiAsyncTask;
        TCPIP_STACK_MODULE_ASYNC_TBL[0].asyncPending = WiFiAsyncTaskPending;
#endif  // defined(TCPIP_IF_MRF24W)

        // start per interface initializing
        tcpip_stack_ctrl_data.stackAction = TCPIP_STACK_ACTION_INIT;

        for(netIx = 0, pIf = tcpipNetIf; netIx < nNets; netIx++, pIf++)
        {
            // get the power mode
#if defined (__C30__)  
            powerMode = TCPIP_MAC_POWER_FULL;
#else
            powerMode = TCPIP_Helper_StringToPowerMode(pUsrConfig->powerMode);
#endif
            if(powerMode == TCPIP_MAC_POWER_NONE || powerMode != TCPIP_MAC_POWER_FULL)
            {   
                SYS_ERROR(SYS_ERROR_ERROR, "Stack Power Mode initialization fail\r\n");
                initFail = true;
                break;
            }

            // set transient data
            tcpip_stack_ctrl_data.powerMode = powerMode;
            tcpip_stack_ctrl_data.pNetIf = pIf;
            tcpip_stack_ctrl_data.netIx = netIx;
            if(!TCPIP_STACK_BringNetUp(&tcpip_stack_ctrl_data, pUsrConfig, pModConfig, nModules))
            {
                initFail = true;
                break;
            }

            // interface success
            // set the default interfaces
            if(tcpipDefIf.defaultNet == 0)
            {
                tcpipDefIf.defaultNet = pIf;    // set as the 1st valid interface
            }
        }

    // initializing the rest of the services
    // (that don't need interface specific initialization)

        if(!initFail)
        {
            tcpip_stack_tickH = SYS_TMR_CallbackPeriodic(TCPIP_STACK_TICK_RATE, _TCPIP_STACK_TickHandler);
            if (tcpip_stack_tickH == SYS_TMR_HANDLE_INVALID)
            {
                SYS_ERROR(SYS_ERROR_ERROR, "Stack tick registration failed\r\n");
                initFail = true;
                break;
            }
            stackTaskRate = TCPIP_STACK_TICK_RATE;
        }

        break;
    }

    // initialization done
    if(!initFail)
    {
        size_t heapLeft;
        SYS_CONSOLE_MESSAGE(TCPIP_STACK_INIT_MESSAGE "Ended - success \n\r");
        // check the amount of heap left
        heapLeft = TCPIP_HEAP_FreeSize(heapH);
        if(heapLeft < TCPIP_STACK_DRAM_RUN_LIMIT)
        {
            SYS_ERROR_PRINT(SYS_ERROR_WARN, "Dynamic memory is low: %d\r\n", heapLeft);
        }
        return true;
    }


    SYS_CONSOLE_MESSAGE(TCPIP_STACK_INIT_MESSAGE "failed - Aborting! \n\r");
    TCPIP_STACK_Deinitialize();
    return false;

}

/*********************************************************************
 * Function:        bool TCPIP_STACK_BringNetUp(TCPIP_NET_IF* pNetIf, const TCPIP_NETWORK_CONFIG* pNetConf, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules)
 *
 * PreCondition:    None
 *
 * Input:           net interface to bring up
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function performs the initialization of a net interface
 *
 * Note:            None
 ********************************************************************/
static bool TCPIP_STACK_BringNetUp(TCPIP_STACK_MODULE_CTRL* stackCtrlData, const TCPIP_NETWORK_CONFIG* pNetConf, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules)
{
    TCPIP_NET_IF*           pNetIf;
    bool                    netUpFail;
    const void*             configData;
    const TCPIP_STACK_MODULE_CONFIG* pConfig;
    TCPIP_MAC_MODULE_CTRL   macCtrl;
    TCPIP_MAC_RES           macInitRes;

    netUpFail = false;
    pNetIf = stackCtrlData->pNetIf;
    // restore the dynamic interface data
    pNetIf->netIfIx = stackCtrlData->netIx;
    pNetIf->Flags.powerMode = stackCtrlData->powerMode;


    while(true)
    {
        // start stack MAC modules initialization
        const TCPIP_STACK_MODULE_MAC_ENTRY*  pMacEntry = _TCPIP_STACK_FindMacModule(pNetIf->macId);

        if(pMacEntry == 0)
        {
            SYS_ERROR_PRINT(SYS_ERROR_ERROR, "%s MAC module not found\r\n", pNetConf->interface);
            netUpFail = 1;
            break;
        }
        // find MAC initialization data
        configData = 0;
        if (pModConfig != 0)
        {
            pConfig = _TCPIP_STACK_FindModuleData(pMacEntry->moduleId, pModConfig, nModules);
            if(pConfig != 0)
            {
                configData = pConfig->configData;
            }
        }
        // init the MAC
        TCPIP_STACK_StacktoMacCtrl(&macCtrl, stackCtrlData);

        do
        {
            macInitRes = (*pMacEntry->initFunc)(&macCtrl, configData);
        }while(macInitRes == TCPIP_MAC_RES_PENDING);

        if( macInitRes != TCPIP_MAC_RES_OK)
        {
            SYS_ERROR_PRINT(SYS_ERROR_ERROR, "%s MAC initialization failed\r\n", pNetConf->interface);
            netUpFail = 1;
            break;
        }
        
        // get the MAC address and MAC processing flags
        memcpy(pNetIf->netMACAddr.v, macCtrl.ifPhyAddress.v, sizeof(pNetIf->netMACAddr));
        pNetIf->Flags.bMacProcessOnEvent = macCtrl.processFlags != TCPIP_MAC_PROCESS_FLAG_NONE;

        // open the MAC
        if( (pNetIf->hIfMac = (*pMacEntry->openFunc)(pNetIf->macId)) == 0)
        {
            SYS_ERROR_PRINT(SYS_ERROR_ERROR, "%s MAC Open failed\r\n", pNetConf->interface);
            netUpFail = 1;
            break;
        }

        // start stack initialization per module
        int modIx;
        const TCPIP_STACK_MODULE_ENTRY*  pEntry = TCPIP_STACK_MODULE_ENTRY_TBL + 0;

        for(modIx = 0; modIx < sizeof(TCPIP_STACK_MODULE_ENTRY_TBL)/sizeof(*TCPIP_STACK_MODULE_ENTRY_TBL); modIx++)
        {
            configData = 0;
            if (pModConfig != 0)
            {
                pConfig = _TCPIP_STACK_FindModuleData(pEntry->moduleId, pModConfig, nModules);
                if(pConfig != 0)
                {
                    configData = pConfig->configData;
                }
            }

            if(!pEntry->initFunc(stackCtrlData, configData))
            {
                SYS_ERROR_PRINT(SYS_ERROR_ERROR, "Module no: %d Initialization failed\r\n", pEntry->moduleId);
                netUpFail = 1;
                break;
            }
            pEntry++;
        }

        if(!netUpFail)
        {
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

            if(!TCPIP_MAC_EventMaskSet(pNetIf->hIfMac, TCPIP_MAC_EV_RX_DONE | TCPIP_MAC_EV_TX_DONE | TCPIP_MAC_EV_RXTX_ERRORS | TCPIP_MAC_EV_CONN_ALL, true))
            {
                SYS_ERROR_PRINT(SYS_ERROR_ERROR, "%s MAC event notification setting failed\r\n", pNetConf->interface);
                netUpFail = 1;
                break;
            }
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

            // completed the MAC initialization
        }

        break;
    }

    if(netUpFail)
    {
        return false;
    }

    if (OSAL_SEM_Create(&((const TCPIP_MAC_DCPT*)pNetIf->hIfMac)->pObj->semaphore, OSAL_SEM_TYPE_BINARY, 1, 1) != OSAL_RESULT_TRUE)
    {
        //SYS_DEBUG message
    }

    pNetIf->Flags.bInterfaceEnabled = true;
    pNetIf->Flags.powerMode = stackCtrlData->powerMode;

    return true;

}

/*********************************************************************
 * Function:        void TCPIP_STACK_NetUp(TCPIP_NET_HANDLE netH, const TCPIP_NETWORK_CONFIG* pUsrConfig)
 *
 * PreCondition:    None
 *
 * Input:           net interface to bring up
 *
 * Output:          true if success
 *                  false if no such network or an error occurred
 *
 * Side Effects:    None
 *
 * Overview:        This function performs the de-initialization of a net interface
 *
 * Note:            None
 ********************************************************************/
bool TCPIP_STACK_NetUp(TCPIP_NET_HANDLE netH, const TCPIP_NETWORK_CONFIG* pUsrConfig)
{
    bool    success;
    TCPIP_MAC_POWER_MODE  powerMode;

    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(netH);

    if(pNetIf)
    {
        if(pNetIf->Flags.bInterfaceEnabled)
        {   // already up
            return true;
        }

        if(pUsrConfig == 0)
        {
            pUsrConfig = & tcpip_def_config;
        }

        // Before we load the default config, we should save what used to be the netIfIx
        // set transient data
        tcpip_stack_ctrl_data.pNetIf = pNetIf;
        tcpip_stack_ctrl_data.netIx = pNetIf->netIfIx;
        tcpip_stack_ctrl_data.stackAction = TCPIP_STACK_ACTION_IF_UP;

        if(!_LoadDefaultConfig(pUsrConfig, pNetIf))
        {
            SYS_ERROR(SYS_ERROR_ERROR, "Default Flash Network configuration load failed\r\n");
            return false;
        }

#if defined (__C30__)  
        powerMode = TCPIP_MAC_POWER_FULL;
#else
        powerMode = TCPIP_Helper_StringToPowerMode(pUsrConfig->powerMode);
#endif
        if(powerMode == TCPIP_MAC_POWER_NONE || powerMode != TCPIP_MAC_POWER_FULL)
        {   
            SYS_ERROR(SYS_ERROR_ERROR, "Stack Power Mode initialization fail\r\n");
            return false;
        }

        tcpip_stack_ctrl_data.powerMode = powerMode;

        success = TCPIP_STACK_BringNetUp(&tcpip_stack_ctrl_data, pUsrConfig, 0, 0);
        if(!success)
        {   // don't let the MAC hanging because of a module failure
            tcpip_stack_ctrl_data.stackAction = TCPIP_STACK_ACTION_IF_DOWN;
            tcpip_stack_ctrl_data.powerMode = TCPIP_MAC_POWER_DOWN;
            TCPIP_STACK_BringNetDown(&tcpip_stack_ctrl_data);
        }
        return success;
    }

    return false;

}
/*********************************************************************
 * Function:        void TCPIP_STACK_Deinitialize(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function performs the de-initialization of the TCPIP stack
 *
 * Note:            None
 ********************************************************************/
void TCPIP_STACK_Deinitialize(void)
{
    int         netIx;
    TCPIP_NET_IF* pIf;

    if(tcpipNetIf == 0)
    {   // already shut down
        return;
    }
    if (tcpip_stack_tickH != SYS_TMR_HANDLE_INVALID)
    {
        SYS_TMR_CallbackStop(tcpip_stack_tickH);
    }

    // set transient data
    tcpip_stack_ctrl_data.stackAction = TCPIP_STACK_ACTION_DEINIT;
    tcpip_stack_ctrl_data.powerMode = TCPIP_MAC_POWER_DOWN;

    for(netIx = 0, pIf = tcpipNetIf; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pIf++)
    {
        tcpip_stack_ctrl_data.pNetIf = pIf;
        tcpip_stack_ctrl_data.netIx = pIf->netIfIx;
        if(pIf->Flags.bInterfaceEnabled)
        {
            TCPIP_STACK_BringNetDown(&tcpip_stack_ctrl_data);
        }
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
        TCPIP_Notification_RemoveAll(&pIf->registeredClients, tcpip_stack_ctrl_data.memH);
#endif
    }

    TCPIP_PKT_Deinitialize();
    TCPIP_HEAP_Free(tcpip_stack_ctrl_data.memH, tcpipNetIf);
    TCPIP_HEAP_Delete(tcpip_stack_ctrl_data.memH);     // destroy the heap
    tcpip_stack_ctrl_data.memH = 0;
    tcpipNetIf = 0;

    SystemFree(tcpip_heap);     // free the allocated memory
    tcpip_heap = 0;

    tcpip_stack_ctrl_data.nIfs = 0;
    tcpip_stack_ctrl_data.nMdls = 0;
}


/*********************************************************************
 * Function:        void TCPIP_STACK_BringNetDown(TCPIP_STACK_MODULE_CTRL* stackCtrlData)
 *
 * PreCondition:    None
 *
 * Input:           net interface to bring down
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function performs the de-initialization of a net interface
 *
 * Note:            None
 ********************************************************************/
static void TCPIP_STACK_BringNetDown(TCPIP_STACK_MODULE_CTRL* stackCtrlData)
{
    TCPIP_MAC_MODULE_CTRL   macCtrl;
    const TCPIP_STACK_MODULE_ENTRY*  pEntry;
    const TCPIP_STACK_MODULE_MAC_ENTRY*  pMacEntry;
    TCPIP_NET_IF* pNetIf;


    // Go to the last entry in the table
    pEntry = TCPIP_STACK_MODULE_ENTRY_TBL + sizeof(TCPIP_STACK_MODULE_ENTRY_TBL)/sizeof(*TCPIP_STACK_MODULE_ENTRY_TBL);
    do
    {
        pEntry--;
        pEntry->deInitFunc(stackCtrlData);
    }
    while (pEntry != TCPIP_STACK_MODULE_ENTRY_TBL);

    pNetIf = stackCtrlData->pNetIf;
    _TCPIPStackRxListPurge(pNetIf);
    if(pNetIf->hIfMac)
    {
        TCPIP_MAC_Close(pNetIf->hIfMac);
        pNetIf->hIfMac = 0;
    }
    // kill the MAC
    TCPIP_STACK_StacktoMacCtrl(&macCtrl, stackCtrlData); 
    pMacEntry = _TCPIP_STACK_FindMacModule(pNetIf->macId);
    (*pMacEntry->deInitFunc)(&macCtrl);

    if (OSAL_SEM_Delete(&((const TCPIP_MAC_DCPT*)pNetIf->hIfMac)->pObj->semaphore) != OSAL_RESULT_TRUE)
    {
        //SYS_DEBUG message
    }

    pNetIf->Flags.bInterfaceEnabled = false;
    pNetIf->Flags.powerMode = stackCtrlData->powerMode;

}

/*********************************************************************
 * Function:        void TCPIP_STACK_NetDown(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    None
 *
 * Input:           net interface to bring down
 *
 * Output:          true if success
 *                  false if no such network
 *
 * Side Effects:    None
 *
 * Overview:        This function performs the de-initialization of a net interface
 *
 * Note:            None
 ********************************************************************/
bool TCPIP_STACK_NetDown(TCPIP_NET_HANDLE netH)
{
    int netIx;
    TCPIP_NET_IF *pIf, *pNewIf;
    TCPIP_NET_IF* pDownIf = _TCPIPStackHandleToNet(netH);

    if(pDownIf)
    {
        if(pDownIf->Flags.bInterfaceEnabled)
        {
            // set transient data
            tcpip_stack_ctrl_data.stackAction = TCPIP_STACK_ACTION_IF_DOWN;
            tcpip_stack_ctrl_data.powerMode = TCPIP_MAC_POWER_DOWN;
            tcpip_stack_ctrl_data.pNetIf = pDownIf;
            tcpip_stack_ctrl_data.netIx = pDownIf->netIfIx;

            if(tcpipDefIf.defaultNet == pDownIf)
            {   // since this interface is going down change the default interface
                pNewIf = 0;
                for(netIx = 0, pIf = tcpipNetIf; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pIf++)
                {
                    if(pIf != pDownIf && pIf->Flags.bInterfaceEnabled)
                    {   // select this one
                        pNewIf = pIf;
                        break;
                    }
                }
                tcpipDefIf.defaultNet = pNewIf;
            }

            TCPIP_STACK_BringNetDown(&tcpip_stack_ctrl_data);
        }
        return true;
    }

    return false;

}


/*********************************************************************
 * Function:        void TCPIP_STACK_Task(void)
 *
 * PreCondition:    TCPIP_STACK_Initialize() is already called.
 *
 * Input:           None
 *
 * Output:          Stack Finite-state Machine (FSM) is executed.
 *
 * Side Effects:    None
 *
 * Note:            This FSM checks for new incoming packets,
 *                  and routes it to appropriate stack components.
 *                  It also performs timed operations.
 *
 *                  This function must be called periodically to
 *                  ensure timely responses.
 ********************************************************************/
void TCPIP_STACK_Task(void)
{
    int                 netIx, modIx;
    TCPIP_NET_IF*       pNetIf;
    TCPIP_MAC_EVENT     activeEvents;
    bool                wasTickEvent;
    const TCPIP_STACK_SYNC_MODULE_ENTRY*   pSyncEntry;

    if(!TCPIP_STACK_ProcessPending())
    {   // process only when events are pending
        return;
    }

    if( newTcpipTickAvlbl)
    {
        wasTickEvent = true;
        _TCPIP_ProcessTickEvent();
    }
    else
    {
        wasTickEvent = false;
    }

    while( totTcpipEventsCnt)
    {   // there are events pending
        totTcpipEventsCnt = 0;

        for(netIx = 0, pNetIf = tcpipNetIf; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pNetIf++)
        {
            if (!pNetIf->Flags.bInterfaceEnabled)
            {
                continue;
            }
            activeEvents =  pNetIf->activeEvents;

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
            TCPIP_MAC_HANDLE activeMac = pNetIf->hIfMac;
            {
                TCPIP_MAC_EVENT macEvents = 0;
                if(pNetIf->Flags.bNewTcpipEventAvlbl)
                {
                    pNetIf->Flags.bNewTcpipEventAvlbl = 0;
                    macEvents = TCPIP_MAC_EventPendingGet(activeMac);
                }
                activeEvents |= macEvents;
            }
#else
            activeEvents |= TCPIP_MAC_EV_RX_DONE|TCPIP_MAC_EV_TX_DONE;    // just fake pending events
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

            // clear processed events
            pNetIf->activeEvents &= ~activeEvents;
            pNetIf->currEvents |= activeEvents;     // store all the processed events

            if(activeEvents)
            {
                if((activeEvents & TCPIP_MAC_EV_RX_DONE) != 0)
                {
                    ProcessTCPIPMacRxEvents(pNetIf);
                    newTcpipStackEventCnt++;
                }

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
                activeEvents &= ~TCPIP_MAC_EV_CONN_ALL; // connection events generated internally so no need to be ack-ed
                if(activeEvents)
                {
                    TCPIP_MAC_EventAcknowledge(activeMac, activeEvents);
                }
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

                if(pNetIf->Flags.bMacProcessOnEvent != 0)
                {   // normal MAC internal processing
                    TCPIP_MAC_Process(pNetIf->hIfMac);
                }

                // process the synchronous handlers
                pSyncEntry = TCPIP_STACK_MODULE_SYNC_TBL + 0;
                for(modIx = 0; modIx < sizeof(TCPIP_STACK_MODULE_SYNC_TBL)/sizeof(*TCPIP_STACK_MODULE_SYNC_TBL); modIx++)
                {
                    (*pSyncEntry->syncHandler)(pNetIf);
                    pSyncEntry++;
                }

            }
        }
    }

    // process connection related events    
    for(netIx = 0, pNetIf = tcpipNetIf; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pNetIf++)
    {
        if (!pNetIf->Flags.bInterfaceEnabled)
        {
            continue;
        }

        activeEvents = pNetIf->currEvents;
        if((activeEvents & TCPIP_MAC_EV_CONN_ALL) != 0)
        {
            for(modIx = 0; modIx < sizeof(TCPIP_STACK_CONN_EVENT_TBL)/sizeof(*TCPIP_STACK_CONN_EVENT_TBL); modIx++)
            {
                (*TCPIP_STACK_CONN_EVENT_TBL[modIx])(pNetIf, activeEvents);
            }
        }

        if((activeEvents & TCPIP_MAC_EV_RXTX_ERRORS) != 0)
        {    // some error has occurred
            _TCPIP_ProcessMACErrorEvents(pNetIf, activeEvents);
        }

        pNetIf->currEvents = 0;
    }

#if defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)
    TCPIP_SSL_Task();
#endif  // defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)

 
    if(wasTickEvent)
    {   // process the asynchronous handlers
        _TCPIPStackProcessTmo();
    }
}


static void _TCPIP_ProcessTickEvent(void)
{
    int     netIx;
    TCPIP_NET_IF* pNetIf;
    bool    linkCurr;

    newTcpipTickAvlbl = 0;

    for(netIx = 0, pNetIf = tcpipNetIf; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pNetIf++)
    {
        if(pNetIf->Flags.bInterfaceEnabled)
        {
            linkCurr = TCPIP_MAC_LinkCheck(pNetIf->hIfMac);     // check link status
            if(pNetIf->exFlags.linkPrev != linkCurr)
            {   // link status changed
                // just call directly the CB, and do not involve the MAC notification mechanism
                _TCPIP_MacEventCB(linkCurr?TCPIP_MAC_EV_CONN_ESTABLISHED:TCPIP_MAC_EV_CONN_LOST, pNetIf);
                pNetIf->exFlags.linkPrev = linkCurr;
            }
        }
    }

}

static void ProcessTCPIPMacRxEvents(TCPIP_NET_IF* pNetIf)
{
    uint16_t frameType;
    TCPIP_MAC_PACKET*       pRxPkt;
    TCPIP_MAC_ETHERNET_HEADER*    pMacHdr;

    // Process as many incomming packets as we can
    while(true)
    {
        // get new packets
        while((pRxPkt = TCPIP_MAC_PacketRx(pNetIf->hIfMac, 0, 0)) != 0)
        {
            _TCPIPStackRxListInsert(pNetIf, pRxPkt);
        }

        if(TCPIP_Helper_ProtectedSingleListIsEmpty(&pNetIf->rxQueue))
        {   // no more processing needed
            return;
        }

        // process the RX queue
        while((pRxPkt = (TCPIP_MAC_PACKET*)TCPIP_Helper_ProtectedSingleListHeadRemove(&pNetIf->rxQueue)))
        {
            pMacHdr = (TCPIP_MAC_ETHERNET_HEADER*)pRxPkt->pMacLayer;
            // get the packet type
            frameType = TCPIP_Helper_ntohs(pMacHdr->Type);

            // Dispatch the packet to the appropriate handler
            switch(frameType)
            {
#if defined (TCPIP_STACK_USE_IPV4)
                case ETHERTYPE_ARP:
                    TCPIP_ARP_Process(pNetIf, pRxPkt);
                    break;

                case ETHERTYPE_IPV4:
                    TCPIP_IPV4_Process(pNetIf, pRxPkt);
                    break;

#endif  // defined (TCPIP_STACK_USE_IPV4)

#if defined(TCPIP_STACK_USE_IPV6)
                case ETHERTYPE_IPV6:
                    TCPIP_IPV6_Process(pNetIf, pRxPkt);
                    break;
#endif  // defined (TCPIP_STACK_USE_IPV6)

                default:
                    // unknown packet type; discard
                    TCPIP_PKT_PacketAcknowledge(pRxPkt, TCPIP_MAC_PKT_ACK_TYPE_ERR); 
                    break;
            }
        }
    }
}


static void    _TCPIP_MacEventCB(TCPIP_MAC_EVENT event, const void* hParam)
{
    TCPIP_NET_IF* pNetIf = (TCPIP_NET_IF*)hParam;

    pNetIf->activeEvents |= event;
    pNetIf->Flags.bNewTcpipEventAvlbl = 1;
    totTcpipEventsCnt++;
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    TCPIP_EVENT   tcpipEvent;
    TCPIP_EVENT_LIST_NODE* tNode;

    tcpipEvent = TCPIP_STACK_Mac2TcpipEvent(event);
    for(tNode = (TCPIP_EVENT_LIST_NODE*)pNetIf->registeredClients.list.head; tNode != 0; tNode = tNode->next)
    {
        if((tNode->evMask & tcpipEvent) != 0 )
        {   // trigger event
            (*tNode->handler)(pNetIf, tcpipEvent, tNode->hParam);
        }
    }

#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

}



/*******************************************************************************
  Function:
    void    _TCPIP_STACK_TickHandler()

  Summary:
    Stack tick handler.

  Description:
    This function is called from within the System Tick ISR.
    It provides the Stack tick processing.
    It will call the notification handler registered with SYS_TMR_CallbackPeriodic


  Precondition:
   System Tick should have been initialized
   and the Stack tick handler should have been registered with the SYS_TMR_CallbackPeriodic.

  Parameters:
    currSysTick   - current system tick value at the time of call

  Returns:
    None

  Remarks:
    None
*****************************************************************************/
static void _TCPIP_STACK_TickHandler()
{
        newTcpipTickAvlbl++;
}





static void _TCPIP_ProcessMACErrorEvents(TCPIP_NET_IF* pNetIf, TCPIP_MAC_EVENT activeEvent)
{
    newTcpipErrorEventCnt++;
}





/*********************************************************************
 * Function:        TCPIP_STACK_NetDefaultGet(void)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Initialize()
 *
 * Input:           None
 *
 * Output:          The default net interface for multi-homed hosts
 *
 * Side Effects:    None
 *
 * Note:            Function to dynamically change the default interface
 *                  will be added.
 ********************************************************************/
TCPIP_NET_HANDLE TCPIP_STACK_NetDefaultGet(void)
{
    return tcpipDefIf.defaultNet;
}

// sets the default interface
// returns true if success,
// false if failed (the old interface does not change)
bool TCPIP_STACK_NetDefaultSet(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNewIf = _TCPIPStackHandleToNetUp(netH);
    if(pNewIf)
    {
        tcpipDefIf.defaultNet = pNewIf;
        return true;
    }

    return false;
}

/*********************************************************************
 * Function:        TCPIP_STACK_IPAddToNet(IPV4_ADDR* pIpAddress, bool useDefault)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Initialize()
 *
 * Input:           pIpAddress - pointer to an IP address
 *
 *                  useDefault - when no interface is found,
 *                               if true: return the default interface
 *                               else return 0;
 *
 * Output:          Resolves a local IP address to a network interface.
 *
 *
 * Side Effects:    None
 *
 * Note:            None
 ********************************************************************/
TCPIP_NET_IF* TCPIP_STACK_IPAddToNet(IPV4_ADDR* pIpAddress, bool useDefault)
{
    TCPIP_NET_IF* pNetIf = 0;

    if(pIpAddress && pIpAddress->Val != 0)
    {
        pNetIf = TCPIP_STACK_NetByAddress(pIpAddress);
    }

    if(pNetIf == 0 && useDefault)
    {
        pNetIf = tcpipDefIf.defaultNet;
    }

    return pNetIf;
}

/*********************************************************************
 * Function:        _TCPIPStackIpAddFromAnyNet(TCPIP_NET_IF* pNetIf, IPV4_ADDR* pIpAddress)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Initialize()
 *
 * Input:           pNetIf  - network interface to check for
 *                            if 0 all interfaces are checked
 *                  pIpAddress - pointer to an IP address
 *
 * Output:          Resolves a local IP address to a network interface
 *                  to which it belongs
 *
 * Side Effects:    None
 *
 * Note:            None
 ********************************************************************/
TCPIP_NET_IF* _TCPIPStackIpAddFromAnyNet(TCPIP_NET_IF* pNetIf, IPV4_ADDR* pIpAddress)
{
    int netIx;
    TCPIP_NET_IF* pIf;

    if(pIpAddress && pIpAddress->Val != 0)
    {
        for(netIx = 0, pIf = tcpipNetIf ; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pIf++)
        {
            if(pNetIf == 0 || pIf == pNetIf)
            {
                if(pIf->Flags.bInterfaceEnabled)
                {
                    if(_TCPIPStackIpAddFromLAN(pIf, pIpAddress))
                    {
                        return pIf;
                    }
                }
            }
        }
    }

    return 0;
}

/*********************************************************************
 * Function:        TCPIP_STACK_MacToNet(TCPIP_MAC_HANDLE hMac)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Initialize()
 *
 * Input:           None
 *
 * Output:          Resolves a MAC Id to an tcpipNetIf entry.
 *
 *
 * Side Effects:    None
 *
 * Note:            The tcpipNetIf entries match 1 to 1 the network interfaces
 *                  A more efficient algorithm to find MAC<->tcpipNetIf entry correspondence
 *                  will be eventually added.
 ********************************************************************/
TCPIP_NET_IF* TCPIP_STACK_MacToNet(TCPIP_MAC_HANDLE hMac)
{
    int netIx;
    TCPIP_NET_IF* pNetIf;

    for(netIx = 0, pNetIf = tcpipNetIf; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pNetIf++)
    {
        if(pNetIf->hIfMac == hMac)
        {
            return pNetIf;
        }
    }


    return 0;
}

/*********************************************************************
 * Function:        TCPIP_STACK_MacToNetIndex(TCPIP_MAC_HANDLE hMac)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Initialize()
 *
 * Input:           None
 *
 * Output:          Resolves a MAC to an tcpipNetIf entry.
 *
 *
 * Side Effects:    None
 *
 * Note:            The tcpipNetIf entries match 1 to 1 the network interfaces
 *                  A more efficient algorithm to find MAC<->tcpipNetIf entry correspondence
 *                  will be eventually added.
 ********************************************************************/
int TCPIP_STACK_MacToNetIndex(TCPIP_MAC_HANDLE hMac)
{
    int netIx;
    TCPIP_NET_IF* pNetIf;

    for(netIx = 0, pNetIf = tcpipNetIf; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pNetIf++)
    {
        if(pNetIf->hIfMac == hMac)
        {
            return netIx;
        }
    }


    return -1;
}

/*********************************************************************
 * Function:        int TCPIP_STACK_NumberOfNetworksGet(void)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Initialize()
 *
 * Input:           None
 *
 * Output:          Number of network interfaces
 *
 * Side Effects:    None
 *
 * Note:            None
 ********************************************************************/
int TCPIP_STACK_NumberOfNetworksGet(void)
{
    return tcpip_stack_ctrl_data.nIfs;
}

/*********************************************************************
 * Function:        TCPIP_STACK_IndexToNet(int netIx)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Initialize()
 *
 * Input:           None
 *
 * Output:          Resolves an index to an tcpipNetIf entry.
 *
 *
 * Side Effects:    None
 *
 * Note:            The tcpipNetIf entries match 1 to 1 the network interfaces
 ********************************************************************/
TCPIP_NET_HANDLE TCPIP_STACK_IndexToNet(int netIx)
{
    if(netIx < tcpip_stack_ctrl_data.nIfs)
    {
        return tcpipNetIf + netIx;
    }

    return 0;
}

int  TCPIP_STACK_NetIndexGet(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(hNet);
    return TCPIP_STACK_NetIxGet(pNetIf);
}

/*********************************************************************
 * Function:        TCPIP_STACK_MACIdToNet(TCPIP_STACK_MODULE macId)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Initialize()
 *
 * Input:           None
 *
 * Output:          Resolves an MAC id to a Net entry.
 *
 *
 * Side Effects:    None
 *
 * Note:            In multi-homed hosts with multiple
 *                  interfaces of the same type,
 *                  the translation might not be unique.
 *                  The first match is returned!
 *
 ********************************************************************/
TCPIP_NET_IF* TCPIP_STACK_MACIdToNet(TCPIP_STACK_MODULE macId)
{
    TCPIP_NET_IF* pNetIf;

    if(macId != TCPIP_MODULE_MAC_NONE)
    {
        int netIx;
        for(netIx = 0, pNetIf = tcpipNetIf; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pNetIf++)
        {
            if(pNetIf->macId == macId)
            {
                return pNetIf;
            }
        }
    }


    return 0;

}

/*********************************************************************
 * Function:        TCPIP_NET_HANDLE TCPIP_STACK_NetHandleGet(const char* interface)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Initialize()
 *
 * Input:           interface - The names specified in tcpip_config.h::TCPIP_NETWORK_CONFIG.
 *
 * Output:          Resolves an interface name to a handle.
 *
 * Side Effects:    None
 *
 * Example:            TCPIP_NET_HANDLE hNet = TCPIP_STACK_NetHandleGet("PIC32INT\r\n");
 *
 * Note:            None
 ********************************************************************/
TCPIP_NET_HANDLE TCPIP_STACK_NetHandleGet(const char* interface)
{
    return TCPIP_STACK_MACIdToNet(TCPIP_STACK_StringToMACId(interface));
}


/*********************************************************************
 * Function:        const char* TCPIP_STACK_NetNameGet(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Initialize()
 *
 * Input:           netH - Interface handle to get the name of.
 *
 * Output:          it returns the name associated to that interface handle
 *                     returns 0 if no such name
 *
 * Side Effects:    None
 *
 * Example:            TCPIP_NET_HANDLE netH = TCPIP_STACK_IndexToNet(0);
 *                     const char* netName = TCPIP_STACK_NetNameGet(netH);
 *
 * Note:            None
 ********************************************************************/
const char* TCPIP_STACK_NetNameGet(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF*  pNetIf = _TCPIPStackHandleToNet(netH);

    return pNetIf?TCPIP_STACK_MACIdToString(pNetIf->macId):0;
}




/*********************************************************************
 * Function:        TCPIP_STACK_NetByAddress(const IPV4_ADDR* pIpAddress)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Initialize()
 *
 * Input:           pointer to an IP address
 *
 * Output:          The network interface pointer to which this ip
 *                  address belongs to.
 *                  NULL if not one of our addresses.
 *
 * Side Effects:    None
 *
 * Note:            A single network interface can support multiple IP addresses.
 *                  For now this feature is not implemented/supported.
 *
 ********************************************************************/
TCPIP_NET_IF* TCPIP_STACK_NetByAddress(const IPV4_ADDR* pIpAddress)
{
    int netIx;
    TCPIP_NET_IF* pIf;

    for(netIx = 0, pIf = tcpipNetIf ; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pIf++)
    {
        if(pIf->Flags.bInterfaceEnabled && pIf->netIPAddr.Val == pIpAddress->Val)
        {
            return pIf;
        }
    }


    return 0;
}

/*********************************************************************
 * Function:        uint32_t TCPIP_STACK_NetAddress(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by TCPIP_STACK_Initialize()
 *
 * Input:           netH - Interface handle to get address of.
 *
 * Output:          The IP address of an interface.
 *
 * Side Effects:    None
 *
 * Example:            TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandleGet("PIC32INT\r\n");
 *                     uint32_t ipAdd = TCPIP_STACK_NetAddress(netH);
 *
 * Note:            None
 ********************************************************************/
uint32_t TCPIP_STACK_NetAddress(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);
    return TCPIP_STACK_NetAddressGet(pNetIf);
}


#if defined(TCPIP_STACK_USE_IPV6)
IPV6_ADDR_HANDLE TCPIP_STACK_NetIPv6AddressGet(TCPIP_NET_HANDLE netH, IPV6_ADDR_TYPE addType, IPV6_ADDR_STRUCT* pAddStruct, IPV6_ADDR_HANDLE addHandle)
{
    IPV6_ADDR_STRUCT * addrNode;
    IPV6_INTERFACE_CONFIG*  pIpv6Config;

    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);
    if(pNetIf == 0)
    {
        return 0;
    }


    // Note: for both unicast and multicast addresses we start from unicast list
    // that's because we need to construct the solicited node multicas address
    // which is not currently stored in its own list!

    pIpv6Config = TCPIP_IPV6_InterfaceConfigGet(pNetIf);
    if(addHandle == 0)
    {   // start iteration through the list
        addrNode = (IPV6_ADDR_STRUCT *)pIpv6Config->listIpv6UnicastAddresses.head;
    }
    else
    {
        addrNode = ((IPV6_ADDR_STRUCT*)addHandle)->next;
    }

    if(addType == IPV6_ADDR_TYPE_UNICAST)
    {
        if(addrNode && pAddStruct)
        {
            memcpy(pAddStruct, addrNode, sizeof(*addrNode));
            pAddStruct->next = pAddStruct->prev = 0;
        }
        return addrNode;
    }

    if(addType == IPV6_ADDR_TYPE_MULTICAST)
    {
        if(addrNode == 0)
        {
            if(addHandle == 0 || ((IPV6_ADDR_STRUCT*)addHandle)->flags.type == IPV6_ADDR_TYPE_UNICAST)
            {   // either the unicast list is empty or finished the unicast list
                addrNode = (IPV6_ADDR_STRUCT *)pIpv6Config->listIpv6MulticastAddresses.head;
            }
        }

        if(addrNode != 0 && addrNode->flags.type == IPV6_ADDR_TYPE_UNICAST)
        {   // do not report the same solicited node address multiple times
            IPV6_ADDR_STRUCT * unicastHead = (IPV6_ADDR_STRUCT *)pIpv6Config->listIpv6UnicastAddresses.head;
            IPV6_ADDR_STRUCT * currAddress = unicastHead;
            while(currAddress != addrNode)
            {
                if(memcmp(addrNode->address.v + sizeof (IPV6_ADDR) - 3, currAddress->address.v + sizeof (IPV6_ADDR) - 3, 3) == 0)
                {   // address match; skip this one
                    addrNode = addrNode->next;
                    if(addrNode == 0)
                    {   // end of list
                        addrNode = (IPV6_ADDR_STRUCT *)pIpv6Config->listIpv6MulticastAddresses.head;
                        break;
                    }
                    else
                    {   // restart traversal
                        currAddress = unicastHead; 
                    }
                }
                else
                {
                    currAddress = currAddress->next;
                }
            }
        }


        if(addrNode && pAddStruct)
        {
            memcpy(pAddStruct, addrNode, sizeof(*addrNode));
            pAddStruct->next = pAddStruct->prev = 0;
            if(addrNode->flags.type == IPV6_ADDR_TYPE_UNICAST)
            {   // construct the solicited node multicast address
                memcpy(pAddStruct->address.v, IPV6_SOLICITED_NODE_MULTICAST.v, sizeof (IPV6_ADDR) - 3);
                pAddStruct->flags.type = IPV6_ADDR_TYPE_MULTICAST;
            }
        }
        return addrNode;
    }


    // no other address type supported
    return 0;
}

// finds an interface that has the IPv6 address
TCPIP_NET_IF* _TCPIPStackIPv6AddToNet(IPV6_ADDR* pIPv6Address, IPV6_ADDR_TYPE addType, bool useDefault)
{
    TCPIP_NET_IF* pNetIf;
    int           netIx;
    TCPIP_NET_IF* pSrchIf = 0;

    if(pIPv6Address != 0)
    {
        for(netIx = 0; netIx < tcpip_stack_ctrl_data.nIfs; netIx++)
        {
            pNetIf = tcpipNetIf + netIx;
            if(TCPIP_IPV6_AddressFind(pNetIf, pIPv6Address, addType) != 0)
            {    // found interface
                pSrchIf = pNetIf;
                break;
            }
        }
    }

    if(pSrchIf == 0 && useDefault)
    {
        pSrchIf = tcpipDefIf.defaultNet;
    }

    return pSrchIf;
}



#endif  // defined(TCPIP_STACK_USE_IPV6)




bool TCPIP_STACK_NetAddressSet(TCPIP_NET_HANDLE netH, IPV4_ADDR* ipAddress, IPV4_ADDR* mask, bool setDefault)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);
    if(pNetIf)
    {
        TCPIP_STACK_NetworkAddressSet(pNetIf, ipAddress, mask, setDefault);
        return true;
    }

    return false;
}
bool TCPIP_STACK_NetAddressGatewaySet(TCPIP_NET_HANDLE netH, IPV4_ADDR* ipAddress)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);
    if(pNetIf)
    {
        TCPIP_STACK_GatewayAddressSet(pNetIf, ipAddress);
        return true;
    }

    return false;
}

uint32_t TCPIP_STACK_NetAddressGateway(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);

    if(pNetIf)
    {
        return pNetIf->netGateway.Val;
    }

    return 0;
}

uint32_t TCPIP_STACK_NetAddressDnsPrimary(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);

    if(pNetIf)
    {
        return pNetIf->PrimaryDNSServer.Val;
    }

    return 0;
}

uint32_t TCPIP_STACK_NetAddressDnsSecond(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);

    if(pNetIf)
    {
        return pNetIf->SecondaryDNSServer.Val;
    }

    return 0;
}

bool TCPIP_STACK_NetAddressDnsPrimarySet(TCPIP_NET_HANDLE netH, IPV4_ADDR* ipAddress)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);
    TCPIP_STACK_PrimaryDNSAddressSet(pNetIf, ipAddress);

    return pNetIf != 0;
}

bool TCPIP_STACK_NetAddressDnsSecondSet(TCPIP_NET_HANDLE netH, IPV4_ADDR* ipAddress)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);
    if(pNetIf)
    {
        TCPIP_STACK_SecondaryDNSAddressSet(pNetIf, ipAddress);
        return true;
    }

    return false;
}

/*********************************************************************
 * Function:        uint32_t TCPIP_STACK_NetMask(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    None
 *
 * Input:           netH - Interface handle to get address of.
 *
 * Output:          if interface enabled then Value of subnet mask
 *                     else 0
 *
 * Example:         TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandleGet("PIC32INT\r\n");
 *                     uint32_t subMask = TCPIP_STACK_NetMask(netH);
 *
 * Note:            None
 ********************************************************************/
uint32_t TCPIP_STACK_NetMask(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);
    if(pNetIf)
    {
        return TCPIP_STACK_NetMaskGet(pNetIf);
    }

    return 0;
}


bool TCPIP_STACK_NetAddressMacSet(TCPIP_NET_HANDLE netH, const TCPIP_MAC_ADDR* pAddr)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);
    if(pNetIf)
    {
        memcpy(pNetIf->netMACAddr.v, pAddr->v, sizeof(pNetIf->netMACAddr));
        return true;
    }

    return false;
}

const char* TCPIP_STACK_NetBIOSName(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);
    if(pNetIf)
    {
        return (const char*)pNetIf->NetBIOSName;
    }

    return 0;

}

bool TCPIP_STACK_NetBiosNameSet(TCPIP_NET_HANDLE netH, const char* biosName)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);
    if(pNetIf)
    {
        memcpy(pNetIf->NetBIOSName, biosName, sizeof(pNetIf->NetBIOSName));
        TCPIP_Helper_FormatNetBIOSName(pNetIf->NetBIOSName);
        return true;
    }

    return false;
}

const uint8_t* TCPIP_STACK_NetAddressMac(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);
    if(pNetIf)
    {
        return TCPIP_STACK_NetMACAddressGet(pNetIf);
    }

    return 0;
}

bool TCPIP_STACK_SetLocalMasksType(TCPIP_NET_HANDLE netH, TCPIP_LOCAL_MASK_TYPE andType, TCPIP_LOCAL_MASK_TYPE orType)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);

    if(pNetIf)
    {
        pNetIf->exFlags.localAndType = andType;
        pNetIf->exFlags.localOrType = orType;
        return true;
    }

    return false;
}

bool TCPIP_STACK_SetLocalMasks(TCPIP_NET_HANDLE netH, uint32_t andMask, uint32_t orMask)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);

    if(pNetIf)
    {
        pNetIf->localAndMask.Val = andMask;
        pNetIf->localOrMask.Val = orMask;
        return true;
    }

    return false;
}

TCPIP_STACK_MODULE  TCPIP_STACK_NetMACIdGet(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);
    if(pNetIf)
    {
        return TCPIP_STACK_NetMACId(pNetIf);
    }

    return TCPIP_MODULE_NONE;
}

bool TCPIP_STACK_NetMACStatisticsGet(TCPIP_NET_HANDLE netH, TCPIP_MAC_RX_STATISTICS* pRxStatistics, TCPIP_MAC_TX_STATISTICS* pTxStatistics)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);
    if(pNetIf)
    {
        TCPIP_MAC_RES res = TCPIP_MAC_StatisticsGet(pNetIf->hIfMac, pRxStatistics, pTxStatistics);
        if(res == TCPIP_MAC_RES_OK)
        {
            return true;
        }
    }

    return false;
}

size_t TCPIP_STACK_ModuleConfigGet(TCPIP_STACK_MODULE modId, void* configBuff, size_t buffSize, size_t* pNeededSize)
{
    if(tcpipNetIf != 0)
    {   // we should be up and running for this

        int modIx;
        const TCPIP_STACK_GET_CONFIG_MODULE_ENTRY*  pCfgEntry = TCPIP_STACK_MODULE_GET_CONFIG_ENTRY_TBL + 0;
        for(modIx = 0; modIx < sizeof(TCPIP_STACK_MODULE_GET_CONFIG_ENTRY_TBL)/sizeof(*TCPIP_STACK_MODULE_GET_CONFIG_ENTRY_TBL); modIx++)
        {
            if(pCfgEntry->moduleId == modId)
            {   // found corresponding module
                return (*pCfgEntry->getConfig)(modId, configBuff, buffSize, pNeededSize);
            }
            pCfgEntry++;
        }
    }

    // not found
    return -1;
}

// all the parameters are returned without checking
// that the interface is enabled or not!
size_t TCPIP_STACK_NetConfigGet(TCPIP_NET_HANDLE netH, void* configStoreBuff, size_t configStoreSize, size_t* pNeededSize)
{
    TCPIP_NET_IF* pNetIf;
    TCPIP_STACK_NET_IF_DCPT* pNetStg;

    if(tcpipNetIf == 0 || (pNetIf = _TCPIPStackHandleToNet(netH))== 0)
    {   // we should be up and running and have a valid IF
        return -1;
    }

    if(pNeededSize == 0 && (configStoreBuff == 0 || configStoreSize == 0 ))
    {   // nothing to do
        return 0;
    }
    // store needed size
    if(pNeededSize)
    {
        *pNeededSize = sizeof(*pNetStg); 
    }

    if(configStoreBuff && configStoreSize >= sizeof(*pNetStg))
    {   // copy all the fields
        pNetStg = (TCPIP_STACK_NET_IF_DCPT*)configStoreBuff;

        // the TCPIP_STACK_NET_IF_DCPT has to be at the very beginning of the pNetIf !!!
        memcpy(pNetStg, &pNetIf->size, sizeof(*pNetStg));
        // update the size field
        pNetStg->size = sizeof(*pNetStg);

        return sizeof(*pNetStg);
    }

    return 0;
}

static void* _NetConfigStringToBuffer(void** ppDstBuff, void* pSrcBuff, size_t* pDstSize, size_t* pNeedLen, size_t* pActLen)
{
    size_t  currLen;

    currLen = strlen(pSrcBuff) + 1;

    *pNeedLen += currLen;
    if(currLen && currLen <= *pDstSize)
    {
        void* pCopy = *ppDstBuff;
        memcpy(*ppDstBuff, pSrcBuff, currLen);
        *ppDstBuff += currLen;
        *pDstSize -= currLen;
        *pActLen += currLen;
        return pCopy;
    }
    else
    {   // stop copying
        *pDstSize = 0;
        return 0;
    }

}

// restores pNetConfig from configBuff
TCPIP_NETWORK_CONFIG*   TCPIP_STACK_NetConfigSet(void* configStoreBuff, void* netConfigBuff, size_t buffSize, size_t* pNeededSize)
{
    TCPIP_NETWORK_CONFIG* pNetConf;            
    TCPIP_STACK_NET_IF_DCPT* pNetStg = (TCPIP_STACK_NET_IF_DCPT*)configStoreBuff;

    if(configStoreBuff == 0 || (pNeededSize == 0 && netConfigBuff == 0 ))
    {   // nothing to do
        return 0;
    }

    // minimum sanity check
    if(pNetStg->size != sizeof(*pNetStg))
    {   // not valid config save?
        return 0;
    }

    if(buffSize < sizeof(*pNetConf))
    {   // not even enough room to start
        return 0;
    }

    char    tempBuff[50 + 1];   // buffer large enough to hold any string in a TCPIP_NETWORK_CONFIG!
    void*   pDstBuff;
    size_t  dstSize;
    size_t  needLen, actualLen;
    
    // create at the very beginning of the buffer
    pNetConf = (TCPIP_NETWORK_CONFIG*)netConfigBuff;
    pDstBuff = pNetConf + 1;    // write area
    dstSize = buffSize - sizeof(*pNetConf);
    needLen = actualLen = 0;
    tempBuff[sizeof(tempBuff) - 1] = '\0';   // always end properly
    
    // get each field
    strncpy(tempBuff, TCPIP_STACK_MACIdToString(pNetStg->macId), sizeof(tempBuff) - 1);
    pNetConf->interface = _NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &needLen, &actualLen);

    strncpy(tempBuff, (char*)pNetStg->NetBIOSName, sizeof(tempBuff) - 1);
    pNetConf->hostName = _NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &needLen, &actualLen);

    TCPIP_Helper_MACAddressToString(&pNetStg->netMACAddr, tempBuff, sizeof(tempBuff) - 1);
    pNetConf->macAddr = _NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &needLen, &actualLen);

    TCPIP_Helper_IPAddressToString(&pNetStg->netIPAddr, tempBuff, sizeof(tempBuff) - 1);
    pNetConf->ipAddr = _NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &needLen, &actualLen);

    TCPIP_Helper_IPAddressToString(&pNetStg->netMask, tempBuff, sizeof(tempBuff) - 1);
    pNetConf->ipMask = _NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &needLen, &actualLen);

    TCPIP_Helper_IPAddressToString(&pNetStg->netGateway, tempBuff, sizeof(tempBuff) - 1);
    pNetConf->gateway = _NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &needLen, &actualLen);

    TCPIP_Helper_IPAddressToString(&pNetStg->PrimaryDNSServer, tempBuff, sizeof(tempBuff) - 1);
    pNetConf->priDNS = _NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &needLen, &actualLen);

    TCPIP_Helper_IPAddressToString(&pNetStg->SecondaryDNSServer, tempBuff, sizeof(tempBuff) - 1);
    pNetConf->secondDNS = _NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &needLen, &actualLen);

    strncpy(tempBuff, TCPIP_Helper_PowerModeToString(pNetStg->Flags.powerMode), sizeof(tempBuff) - 1);
    pNetConf->powerMode = _NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &needLen, &actualLen);

    // set the flags
    pNetConf->startFlags = 0; 
    if(pNetStg->Flags.bIsDHCPEnabled)
    {
        pNetConf->startFlags |= TCPIP_NETWORK_CONFIG_DHCP_CLIENT_ON; 
    }
    if(pNetStg->Flags.bIsDHCPSrvEnabled)
    {
        pNetConf->startFlags |= TCPIP_NETWORK_CONFIG_DHCP_SERVER_ON; 
    }
    if(pNetStg->Flags.bIsZcllEnabled)
    {
        pNetConf->startFlags |= TCPIP_NETWORK_CONFIG_ZCLL_ON; 
    }
#if defined(TCPIP_STACK_USE_IPV6)
    if((pNetStg->startFlags & TCPIP_NETWORK_CONFIG_IPV6_ADDRESS) != 0)
    {
        pNetConf->startFlags |= TCPIP_NETWORK_CONFIG_IPV6_ADDRESS;
        pNetConf->ipv6PrefixLen = pNetStg->ipv6PrefixLen;

        TCPIP_Helper_IPv6AddressToString(&pNetStg->netIPv6Addr, tempBuff, sizeof(tempBuff) - 1);
        pNetConf->ipv6Addr = _NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &needLen, &actualLen);

        TCPIP_Helper_IPv6AddressToString(&pNetStg->netIPv6Gateway, tempBuff, sizeof(tempBuff) - 1);
        pNetConf->ipv6Gateway = _NetConfigStringToBuffer(&pDstBuff, tempBuff, &dstSize, &needLen, &actualLen);
    }

#endif  // defined(TCPIP_STACK_USE_IPV6)

    if(pNeededSize)
    {
        *pNeededSize = needLen + sizeof(*pNetConf);
    }

    if(actualLen == needLen)
    {   // succeeded
        return pNetConf;
    }

    return 0;

}

TCPIP_STACK_MODULE TCPIP_STACK_StringToMACId(const char* str)
{

    if(str)
    {
        int ix;
        const TCPIP_STACK_MODULE_MAC_ENTRY* pEntry = TCPIP_STACK_MODULE_MAC_ENTRY_TBL;

        for(ix = 0; ix < sizeof(TCPIP_STACK_MODULE_MAC_ENTRY_TBL)/sizeof(*TCPIP_STACK_MODULE_MAC_ENTRY_TBL); ix++, pEntry++)
        {
            if(!strcmp(str, pEntry->interfaceName))
            {
                return pEntry->moduleId;
            }
        }
    }

    return TCPIP_MODULE_MAC_NONE;
}

const char* TCPIP_STACK_MACIdToString(TCPIP_STACK_MODULE moduleId)
{
    int ix;
    const TCPIP_STACK_MODULE_MAC_ENTRY* pEntry = TCPIP_STACK_MODULE_MAC_ENTRY_TBL;

    for(ix = 0; ix < sizeof(TCPIP_STACK_MODULE_MAC_ENTRY_TBL)/sizeof(*TCPIP_STACK_MODULE_MAC_ENTRY_TBL); ix++, pEntry++)
    {
        if(pEntry->moduleId == moduleId)
        {
            return pEntry->interfaceName;
        }
    }

    return 0;
}

// detects if an IP address is a local network
bool TCPIP_STACK_IPAddressIsLocalNetwork(TCPIP_NET_IF* pNetIf, IPV4_ADDR destIpAdd)
{
    uint32_t currNetVal, destNetVal;
    uint32_t andMask, orMask;

    if(pNetIf == 0)
    {   // unknown
        return false;
    }

    switch(pNetIf->exFlags.localAndType)
    {
        case TCPIP_LOCAL_MASK_ZERO:
            andMask = 0;
            break;

        case TCPIP_LOCAL_MASK_ONE:
            andMask = 0xffffffff;
            break;

        case TCPIP_LOCAL_MASK_NET:
            andMask = pNetIf->netMask.Val;
            break;

        default:    // TCPIP_LOCAL_MASK_SET
            andMask = pNetIf->localAndMask.Val;
            break;
    }

    switch(pNetIf->exFlags.localOrType)
    {
        case TCPIP_LOCAL_MASK_ZERO:
            orMask = 0;
            break;

        case TCPIP_LOCAL_MASK_ONE:
            orMask = 0xffffffff;
            break;

        case TCPIP_LOCAL_MASK_NET:
            orMask = pNetIf->netMask.Val;
            break;

        default:    // TCPIP_LOCAL_MASK_SET
            orMask = pNetIf->localOrMask.Val;
            break;
    }

    currNetVal  = (pNetIf->netIPAddr.Val & andMask) | orMask;
    destNetVal = (destIpAdd.Val & andMask) | orMask;

    return destNetVal == currNetVal; 
}

/*********************************************************************
 * Function:        TCPIP_STACK_NetAddressBcast(TCPIP_NET_HANDLE netH)
 *
 * PreCondition:    TCPIP stack should have been initialized by
 *                    TCPIP_STACK_Initialize()
 *
 * Input:           interface handle to get address of
 *
 * Output:          The broadcast IP address of an interface.
 *
 *
 * Side Effects:    None
 *
 * Note:           None
 *                 
 ********************************************************************/
uint32_t TCPIP_STACK_NetAddressBcast(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);
    if(pNetIf)
    {
        return (pNetIf->netIPAddr.Val | ~pNetIf->netMask.Val);
    }

    return 0;


}

bool TCPIP_STACK_NetIsUp(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);
    return pNetIf != 0;
}

bool TCPIP_STACK_NetIsLinked(TCPIP_NET_HANDLE netH)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(netH);
    if(pNetIf)
    {
        return TCPIP_STACK_NetworkIsLinked(pNetIf);
    }
    return false;
}

TCPIP_STACK_DNS_SERVICE_TYPE TCPIP_STACK_DNSServiceSelect(TCPIP_NET_IF* pNetIf, TCPIP_NETWORK_CONFIG_FLAGS configFlags)
{
    // clear all the existing DNS address service bits
    pNetIf->Flags.v &= ~TCPIP_STACK_SERVICE_DNS_MASK;

    
#if defined(TCPIP_STACK_USE_DNS)
        if((configFlags & TCPIP_NETWORK_CONFIG_DNS_CLIENT_ON) != 0 )
        { 
            pNetIf->Flags.bIsDnsClientEnabled = 1;
            return TCPIP_STACK_SERVICE_DNSC;
        }
#endif  // defined(TCPIP_STACK_USE_DNS)
    
#if defined(TCPIP_STACK_USE_DNS_SERVER)
        if((configFlags & TCPIP_NETWORK_CONFIG_DNS_SERVER_ON) != 0 )
        { 
            pNetIf->Flags.bIsDnsServerEnabled = 1;
            return TCPIP_STACK_SERVICE_DNSS;
        }
#endif  // defined(TCPIP_STACK_USE_DNS_SERVER)
    // couldn't select an address service
    // use default/static
    return TCPIP_STACK_DNS_SERVICE_NONE;
}


TCPIP_STACK_ADDRESS_SERVICE_TYPE TCPIP_STACK_AddressServiceSelect(TCPIP_NET_IF* pNetIf, TCPIP_NETWORK_CONFIG_FLAGS configFlags)
{
    // clear all the existing address service bits
    pNetIf->Flags.v &= ~TCPIP_STACK_ADDRESS_SERVICE_MASK;

    // Set up the address service on this interface
    // Priority (high to low): DHCPc, ZCLL, DHCPS, static IP address
#if defined(TCPIP_STACK_USE_DHCP_CLIENT)
    if((configFlags & TCPIP_NETWORK_CONFIG_DHCP_CLIENT_ON) != 0 )
    { 
        pNetIf->Flags.bIsDHCPEnabled = 1;
        return TCPIP_STACK_ADDRESS_SERVICE_DHCPC;
    }
#endif  // defined(TCPIP_STACK_USE_DHCP_CLIENT)

#if defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)
    if((configFlags & TCPIP_NETWORK_CONFIG_ZCLL_ON) != 0 )
    { 
        pNetIf->Flags.bIsZcllEnabled = 1;
        return TCPIP_STACK_ADDRESS_SERVICE_ZCLL;
    }
#endif  // defined(TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL)

#if defined(TCPIP_STACK_USE_DHCP_SERVER)
    if((configFlags & TCPIP_NETWORK_CONFIG_DHCP_SERVER_ON) != 0 )
    { 
        pNetIf->Flags.bIsDHCPSrvEnabled = 1;
        return TCPIP_STACK_ADDRESS_SERVICE_DHCPS;
    }
#endif  // defined(TCPIP_STACK_USE_DHCP_SERVER)

    // couldn't select an address service
    // use default/static
    return TCPIP_STACK_ADDRESS_SERVICE_NONE;

}


bool TCPIP_STACK_AddressServiceCanStart(TCPIP_NET_IF* pNetIf, TCPIP_STACK_ADDRESS_SERVICE_TYPE adSvcType)
{
    if(pNetIf)
    {   // enable a different address service only if there's not another one running
        // client has to stop a previos service (DHCP, ZCLL, etc.) in order to start another one
        return (pNetIf->Flags.v & TCPIP_STACK_ADDRESS_SERVICE_MASK) == TCPIP_STACK_ADDRESS_SERVICE_NONE;
    }

    return false;
}

bool TCPIP_STACK_DNSServiceCanStart(TCPIP_NET_IF* pNetIf, TCPIP_STACK_DNS_SERVICE_TYPE adSvcType)
{
    if(pNetIf)
    {   // enable a different address service only if there's not another one running
        // client has to stop a previos service (DHCP, ZCLL, etc.) in order to start another one
        return (pNetIf->Flags.v & TCPIP_STACK_SERVICE_DNS_MASK) != TCPIP_STACK_SERVICE_DNSS;
    }

    return false;
}

TCPIP_STACK_ADDRESS_SERVICE_TYPE _TCPIPStackAddressServiceIsRunning(TCPIP_NET_IF* pNetIf)
{
    return (TCPIP_STACK_ADDRESS_SERVICE_TYPE)(pNetIf->Flags.v & TCPIP_STACK_ADDRESS_SERVICE_MASK);
}

void TCPIP_STACK_AddressServiceEvent(TCPIP_NET_IF* pNetIf, TCPIP_STACK_ADDRESS_SERVICE_TYPE adSvcType,
                                    TCPIP_STACK_ADDRESS_SERVICE_EVENT evType)
{
    typedef bool(*addSvcFnc)(TCPIP_NET_HANDLE hNet);
    addSvcFnc   addFnc;

    if(evType == TCPIP_STACK_ADDRESS_SERVICE_EVENT_CONN_LOST)
    {   // connection loss is considered a temporary event;
        // no need to disable a service
        // since we don't have network connectivity anyway
        return;
    }
    else if(adSvcType == TCPIP_STACK_ADDRESS_SERVICE_DHCPS)
    {   // if DHCP server was stopped/failed
        // we won't start another address service
        // the user will have to take a decision
        return;
    }

    // the DHCPc/ZCLL address service failed/stopped:
    // TCPIP_STACK_ADDRESS_SERVICE_EVENT_RUN_FAIL, TCPIP_STACK_ADDRESS_SERVICE_EVENT_USER_STOP
    //
    // make sure any running service is cleared
    pNetIf->Flags.v &= ~TCPIP_STACK_ADDRESS_SERVICE_MASK;
    _TCPIPStackSetConfig(pNetIf, true);
    addFnc = 0;
    if(adSvcType == TCPIP_STACK_ADDRESS_SERVICE_DHCPC)
    {   // the DHCP client has been stopped or failed
        // if possible we'll select ZCLL
        if((pNetIf->startFlags & TCPIP_NETWORK_CONFIG_ZCLL_ON) != 0)
        {   // OK, we can use ZCLL
            addFnc = TCPIP_ZCLL_Enable;
        }
    }
    // else if (adSvcType == TCPIP_STACK_ADDRESS_SERVICE_ZCLL)
    // we'll select the default IP address


    if(addFnc)
    {
        if((*addFnc)(pNetIf) == true)
        {   // success
            return;
        }
    }

    // no other address service or it couldn't be started
    // select the default/static addresses
    TCPIP_STACK_AddressServiceDefaultSet(pNetIf);
}

/*********************************************************************
 * Function:        bool _InitNetConfig(const TCPIP_NETWORK_CONFIG* pUsrConfig, int nNets)
 *
 * PreCondition:    MPFSInit() is already called.
 *
 * Input:           pUsrConfig  - pointer to user configurations
 *                  nNets       - number of networks configurations provided
 *
 * Output:          Write/Read non-volatile config variables.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 ********************************************************************/
static bool _InitNetConfig(const TCPIP_NETWORK_CONFIG* pUsrConfig, int nNets)
{
    int     ix;
    TCPIP_NET_IF* pNetConfig;


    for(ix =0, pNetConfig = tcpipNetIf; ix < nNets; ix++, pNetConfig++, pUsrConfig++)
    {
        if(!_LoadDefaultConfig(pUsrConfig, pNetConfig))
        {
            SYS_ERROR(SYS_ERROR_ERROR, "Default Flash Network configuration load failed\r\n");
            return false;
        }
    }



    return true;
}


/*********************************************************************
 * Function:        bool _LoadDefaultConfig(const TCPIP_NETWORK_CONFIG* pUsrConfig, TCPIP_NET_IF* pNetIf)
 *
 * PreCondition:    None
 *
 * Input:           pUsrConfig  - pointer to configurations to use
 *                  pNetIf     - network interface to default configure
 *
 * Output:          true if the default configuration sucessfully loaded,
 *                  false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        Loads the default values (flash) for the network configuration
 *
 * Note:            None
 ********************************************************************/
static bool _LoadDefaultConfig(const TCPIP_NETWORK_CONFIG* pUsrConfig, TCPIP_NET_IF* pNetIf)
{
    TCPIP_STACK_ADDRESS_SERVICE_TYPE startAddService;
	TCPIP_STACK_DNS_SERVICE_TYPE	 addDynamicNameService;

    memset(pNetIf, 0, sizeof(*pNetIf));

    if(pUsrConfig->macAddr != 0)
    {
        TCPIP_Helper_StringToMACAddress(pUsrConfig->macAddr, pNetIf->netMACAddr.v);
    }
    else
    {
        memset(pNetIf->netMACAddr.v, 0, sizeof(pNetIf->netMACAddr.v));
    }

    // store the default addresses
    TCPIP_Helper_StringToIPAddress(pUsrConfig->ipAddr, &pNetIf->DefaultIPAddr);
    TCPIP_Helper_StringToIPAddress(pUsrConfig->ipMask, &pNetIf->DefaultMask);

    TCPIP_Helper_StringToIPAddress(pUsrConfig->gateway, &pNetIf->DefaultGateway);
    TCPIP_Helper_StringToIPAddress(pUsrConfig->priDNS, &pNetIf->DefaultDNSServer);
    TCPIP_Helper_StringToIPAddress(pUsrConfig->secondDNS, &pNetIf->DefaultDNSServer2);

    pNetIf->macId = TCPIP_STACK_StringToMACId(pUsrConfig->interface);
    if(pNetIf->macId == TCPIP_MODULE_MAC_NONE)
    {
        return false;   // no such MAC interface
    }

    // Load the NetBIOS Host Name
    memcpy(pNetIf->NetBIOSName, pUsrConfig->hostName, sizeof(tcpipNetIf[0].NetBIOSName));
    TCPIP_Helper_FormatNetBIOSName(pNetIf->NetBIOSName);

    // store start up flags
    pNetIf->startFlags = pUsrConfig->startFlags;
#if defined(TCPIP_STACK_USE_IPV6)
    if((pNetIf->startFlags & TCPIP_NETWORK_CONFIG_IPV6_ADDRESS) != 0)
    {
        pNetIf->ipv6PrefixLen = (uint16_t)pUsrConfig->ipv6PrefixLen;
        if(pUsrConfig->ipv6Addr == 0 || !TCPIP_Helper_StringToIPv6Address (pUsrConfig->ipv6Addr, &pNetIf->netIPv6Addr))
        {   // ignore the static IPv6 address if incorrect
            pNetIf->startFlags &= ~TCPIP_NETWORK_CONFIG_IPV6_ADDRESS;
        }
        else if(pUsrConfig->ipv6Gateway == 0 || !TCPIP_Helper_StringToIPv6Address (pUsrConfig->ipv6Gateway, &pNetIf->netIPv6Gateway))
        {   // ignore the IPv6 gateway if incorrect
            pNetIf->startFlags &= ~TCPIP_NETWORK_CONFIG_IPV6_ADDRESS;
        }
            
    }
#endif  // defined(TCPIP_STACK_USE_IPV6)

    // Set up the address service on this interface
    _TCPIPStackSetConfig(pNetIf, true);
    startAddService = TCPIP_STACK_AddressServiceSelect(pNetIf, pUsrConfig->startFlags);

    if(startAddService == TCPIP_STACK_ADDRESS_SERVICE_NONE)
    {   // couldn't start an address service; use the static values supplied
        TCPIP_STACK_AddressServiceDefaultSet(pNetIf);
    }
    
    if( (startAddService != TCPIP_STACK_ADDRESS_SERVICE_DHCPC))
    {   // will use the default DNS server
        pNetIf->PrimaryDNSServer.Val = pNetIf->DefaultDNSServer.Val;
        pNetIf->SecondaryDNSServer.Val = pNetIf->DefaultDNSServer2.Val;
    }
    else
    {   // The DHCPc will update these
        pNetIf->Flags.bIsDNSServerAuto = 1;
    }

    addDynamicNameService =  TCPIP_STACK_DNSServiceSelect(pNetIf, pUsrConfig->startFlags);

    if(addDynamicNameService == TCPIP_STACK_DNS_SERVICE_NONE)
    {
         pNetIf->Flags.bIsDnsClientEnabled = 1;
    }

    return true;
}

void TCPIP_STACK_AddressServiceDefaultSet(TCPIP_NET_IF* pNetIf)
{
    _TCPIPStackSetConfigAddress(pNetIf, &pNetIf->DefaultIPAddr, &pNetIf->DefaultMask, false);
    pNetIf->netGateway.Val = pNetIf->DefaultGateway.Val;
}


#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

// Stack external event notification support

TCPIP_EVENT TCPIP_STACK_EventsPendingGet(const void* h)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(h);
    if(pNetIf)
    {
        return TCPIP_STACK_Mac2TcpipEvent(TCPIP_MAC_EventPendingGet(pNetIf->hIfMac));
    }
    return TCPIP_EV_NONE;
}

TCPIP_EVENT_HANDLE    TCPIP_STACK_HandlerRegister(TCPIP_NET_HANDLE hNet, TCPIP_EVENT evMask, TCPIP_STACK_EVENT_HANDLER handler, const void* hParam)
{
    TCPIP_EVENT_LIST_NODE* newNode;
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(hNet);

    if(pNetIf && handler)
    {
        newNode = (TCPIP_EVENT_LIST_NODE*)TCPIP_Notification_Add(&pNetIf->registeredClients, tcpip_stack_ctrl_data.memH, sizeof(*newNode));
        if(newNode)
        {
            newNode->handler = handler;
            newNode->hParam = hParam;
            newNode->evMask = evMask;
            newNode->pNetIf = pNetIf;
        }
        return newNode;
    }

    return 0;
}

bool TCPIP_STACK_HandlerDeregister(TCPIP_EVENT_HANDLE hStack)
{
    TCPIP_EVENT_LIST_NODE* pNode = (TCPIP_EVENT_LIST_NODE*)hStack;
    if(pNode)
    {
        if(TCPIP_Notification_Remove((SGL_LIST_NODE*)pNode, &pNode->pNetIf->registeredClients,tcpip_stack_ctrl_data.memH))
        {
            return true;
        }
    }

    return false;

}


#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

static const TCPIP_STACK_MODULE_CONFIG* _TCPIP_STACK_FindModuleData(TCPIP_STACK_MODULE moduleId, const TCPIP_STACK_MODULE_CONFIG* pModConfig, int nModules)
{
    while(nModules--)
    {
        if(pModConfig->moduleId == moduleId)
        {
            return pModConfig;
        }
        pModConfig++;
    }

    return 0;
}

static const TCPIP_STACK_MODULE_MAC_ENTRY* _TCPIP_STACK_FindMacModule(TCPIP_STACK_MODULE moduleId)
{
    int modIx;
    const TCPIP_STACK_MODULE_MAC_ENTRY*  pMacEntry = TCPIP_STACK_MODULE_MAC_ENTRY_TBL + 0;
    for(modIx = 0; modIx < sizeof(TCPIP_STACK_MODULE_MAC_ENTRY_TBL)/sizeof(*TCPIP_STACK_MODULE_MAC_ENTRY_TBL); modIx++)
    {
        if(pMacEntry->moduleId == moduleId)
        {   // found corresponding MAC
            return pMacEntry;
        }
        pMacEntry++;
    }

    return 0;
}


int  TCPIP_STACK_NetIxGet(TCPIP_NET_IF* pNetIf)
{
    if(pNetIf)
    {
        return pNetIf->netIfIx;
    }
    return -1;
}


uint32_t  TCPIP_STACK_NetAddressGet(TCPIP_NET_IF* pNetIf)
{
    if(TCPIP_STACK_NetworkIsUp(pNetIf))
    {
        return _TCPIPStackNetAddress(pNetIf);
    }
    return 0;
}


const IPV6_ADDR* TCPIP_STACK_NetStaticIPv6AddressGet(TCPIP_NET_IF* pNetIf, int* pPrefixLen)
{
#if defined(TCPIP_STACK_USE_IPV6)
    if((pNetIf->startFlags & TCPIP_NETWORK_CONFIG_IPV6_ADDRESS) != 0)
    {
        if(pPrefixLen)
        {
            *pPrefixLen = (int)pNetIf->ipv6PrefixLen;
        }
        return &pNetIf->netIPv6Addr;
    }
#endif  // defined(TCPIP_STACK_USE_IPV6)

    return 0;
}

const IPV6_ADDR* TCPIP_STACK_NetDefaultIPv6GatewayGet(TCPIP_NET_IF* pNetIf)
{
#if defined(TCPIP_STACK_USE_IPV6)
    if((pNetIf->startFlags & TCPIP_NETWORK_CONFIG_IPV6_ADDRESS) != 0)
    {
        return &pNetIf->netIPv6Gateway;
    }
#endif  // defined(TCPIP_STACK_USE_IPV6)

    return 0;
}


void  TCPIP_STACK_NetworkAddressSet(TCPIP_NET_IF* pNetIf, IPV4_ADDR* ipAddress, IPV4_ADDR* mask, bool setDefault)
{
    if(pNetIf)
    {
        if(ipAddress)
        {
            pNetIf->netIPAddr.Val = ipAddress->Val;
            if(setDefault)
            {
                pNetIf->DefaultIPAddr.Val = ipAddress->Val;
            }
        }

        if(mask)
        {
            pNetIf->netMask.Val = mask->Val;
            if(setDefault)
            {
                pNetIf->DefaultMask.Val = mask->Val;
            }
        }
    }
}

void  _TCPIPStackSetConfigAddress(TCPIP_NET_IF* pNetIf, IPV4_ADDR* ipAddress, IPV4_ADDR* mask, bool config)
{
    if(pNetIf)
    {
        if(ipAddress)
        {
            pNetIf->netIPAddr.Val = ipAddress->Val;
        }

        if(mask)
        {
            pNetIf->netMask.Val = mask->Val;
        }
        _TCPIPStackSetConfig(pNetIf, config);
    }
}

uint32_t  TCPIP_STACK_NetMaskGet(TCPIP_NET_IF* pNetIf)
{
    if(TCPIP_STACK_NetworkIsUp(pNetIf))
    {
        return pNetIf->netMask.Val;
    }
    return 0;
}

void  TCPIP_STACK_GatewayAddressSet(TCPIP_NET_IF* pNetIf, IPV4_ADDR* ipAddress)
{
    if(pNetIf)
    {
        pNetIf->netGateway.Val = ipAddress->Val;
    }
}
void  TCPIP_STACK_PrimaryDNSAddressSet(TCPIP_NET_IF* pNetIf, IPV4_ADDR* ipAddress)
{
    if(pNetIf)
    {
        pNetIf->PrimaryDNSServer.Val = ipAddress->Val;
    }
}

void  TCPIP_STACK_SecondaryDNSAddressSet(TCPIP_NET_IF* pNetIf, IPV4_ADDR* ipAddress)
{
    if(pNetIf)
    {
        pNetIf->SecondaryDNSServer.Val = ipAddress->Val;
    }
}


bool  TCPIP_STACK_AddressIsOfNetUp( TCPIP_NET_IF* pNetIf, const IPV4_ADDR* pIpAdd)
{
    if(TCPIP_STACK_NetworkIsUp(pNetIf))
    {
        return TCPIP_STACK_AddressIsOfNet(pNetIf, pIpAdd);
    }
    return false;
}

// detects net-directed bcast
bool  TCPIP_STACK_NetIsBcastAddress(TCPIP_NET_IF* pNetIf, IPV4_ADDR* pIpAdd)
{
    if(TCPIP_STACK_NetworkIsUp(pNetIf))
    {
       return (pIpAdd->Val == (pNetIf->netIPAddr.Val | ~pNetIf->netMask.Val));
    }
    return false;
}

// detects limited or net-directed bcast
bool  TCPIP_STACK_IsBcastAddress(TCPIP_NET_IF* pNetIf, IPV4_ADDR* pIpAdd)
{
    if(TCPIP_STACK_NetworkIsUp(pNetIf))
    {
       return (TCPIP_Helper_IsBcastAddress(pIpAdd) ||  TCPIP_STACK_NetIsBcastAddress(pNetIf, pIpAdd));
    }
    return false;
}

bool TCPIP_STACK_NetworkIsLinked(TCPIP_NET_IF* pNetIf)
{
    if(TCPIP_STACK_NetworkIsUp(pNetIf))
    {
        return pNetIf->exFlags.linkPrev;
    }

    return false;
}

// checks for valid up and linked interface
TCPIP_NET_IF* _TCPIPStackHandleToNetLinked(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(hNet);
    if(pNetIf != 0 && pNetIf->exFlags.linkPrev != 0)
    {
        return pNetIf;
    }
    
    return 0;
}

TCPIP_NET_IF* _TCPIPStackAnyNetLinked(bool useDefault)
{
    int netIx;
    TCPIP_NET_IF* pNetIf = 0;

    if(useDefault)
    {
        pNetIf = _TCPIPStackHandleToNetLinked(tcpipDefIf.defaultNet);
    }

    if(pNetIf == 0)
    {
        TCPIP_NET_IF* pIf;

        for(netIx = 0, pIf = tcpipNetIf ; netIx < tcpip_stack_ctrl_data.nIfs; netIx++, pIf++)
        {
            if(pIf->Flags.bInterfaceEnabled && pIf->exFlags.linkPrev != 0)
            {   // found linked interface
                pNetIf = pIf;
                break;
            }
        }
    }

    return pNetIf;
}






TCPIP_STACK_MODULE  TCPIP_STACK_NetMACId(TCPIP_NET_IF* pNetIf)
{
    return pNetIf?pNetIf->macId:TCPIP_MODULE_MAC_NONE;
}


 TCPIP_MAC_HANDLE  TCPIP_STACK_NetToMAC(TCPIP_NET_IF* pNetIf)
{
    return pNetIf?pNetIf->hIfMac:0;
}



const uint8_t*  TCPIP_STACK_NetMACAddressGet(TCPIP_NET_IF* pNetIf)
{
    if(TCPIP_STACK_NetworkIsUp(pNetIf))
    {
        return pNetIf->netMACAddr.v;
    }

    return 0;
}

static TCPIP_MAC_ACTION TCPIP_STACK_StackToMacAction(TCPIP_STACK_ACTION action)
{   // TCPIP_MAC_ACTION and TCPIP_STACK_ACTION should be kept in sync!
    return (TCPIP_MAC_ACTION)action;
}

static void TCPIP_STACK_StacktoMacCtrl(TCPIP_MAC_MODULE_CTRL* pMacCtrl, TCPIP_STACK_MODULE_CTRL* stackCtrlData)
{
    TCPIP_NET_IF* pNetIf = stackCtrlData->pNetIf;


    pMacCtrl->nIfs = stackCtrlData->nIfs;

    pMacCtrl->mallocF = (TCPIP_MAC_HEAP_MallocF)_TCPIP_HEAP_MALLOC_FNC;
    pMacCtrl->callocF = (TCPIP_MAC_HEAP_CallocF)_TCPIP_HEAP_CALLOC_FNC;
    pMacCtrl->freeF = (TCPIP_MAC_HEAP_FreeF)_TCPIP_HEAP_FREE_FNC;
    pMacCtrl->memH = stackCtrlData->memH;


    pMacCtrl->pktAllocF = (TCPIP_MAC_PKT_AllocF)_TCPIP_PKT_ALLOC_FNC;
    pMacCtrl->pktFreeF = (TCPIP_MAC_PKT_FreeF)_TCPIP_PKT_FREE_FNC;
    pMacCtrl->pktAckF = (TCPIP_MAC_PKT_AckF)_TCPIP_PKT_ACK_FNC;


#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    // Stack can use one handler for all network interfaces, like in this case
    // Each time a notification is received, all interfaces are checked
    // Or, more efficient, use a handler per interface
    pMacCtrl->eventF = _TCPIP_MacEventCB;
    pMacCtrl->eventParam = pNetIf;
#else
    pMacCtrl->eventF = 0;
    pMacCtrl->eventParam = 0;
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

    pMacCtrl->moduleId = pNetIf->macId;
    pMacCtrl->netIx = stackCtrlData->netIx;
    pMacCtrl->macAction = TCPIP_STACK_StackToMacAction(stackCtrlData->stackAction);
    pMacCtrl->powerMode = stackCtrlData->powerMode;

    memcpy(pMacCtrl->ifPhyAddress.v, pNetIf->netMACAddr.v, sizeof(pMacCtrl->ifPhyAddress));
    pMacCtrl->processFlags = TCPIP_MAC_PROCESS_FLAG_NONE;
}

// returns true if the stack needs processing time
static bool TCPIP_STACK_ProcessPending(void)
{
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
#if defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)
    return (newTcpipTickAvlbl != 0 || totTcpipEventsCnt != 0 || TCPIP_SSL_TaskIsPending() == true);
#else
    return (newTcpipTickAvlbl != 0 || totTcpipEventsCnt != 0);
#endif  //defined(TCPIP_STACK_USE_SSL_SERVER) || defined(TCPIP_STACK_USE_SSL_CLIENT)
#else
    // fake pending events
    totTcpipEventsCnt++;
    return true;
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
}

tcpipAsyncHandle _TCPIPStackAsyncHandlerRegister(tcpipModuleAsyncHandler asyncHandler, tcpipModuleAsyncPending asyncPending, int16_t asyncTmoMs)
{
    int ix;
    TCPIP_STACK_ASYNC_MODULE_ENTRY* pAsyncEntry;

    if(asyncHandler != 0)
    {
        for(ix = 0, pAsyncEntry = TCPIP_STACK_MODULE_ASYNC_TBL; ix < sizeof(TCPIP_STACK_MODULE_ASYNC_TBL)/sizeof(*TCPIP_STACK_MODULE_ASYNC_TBL); ix++, pAsyncEntry++)
        {
            if(pAsyncEntry->asyncHandler == 0 || pAsyncEntry->asyncHandler == asyncHandler)
            {   // found empty slot
                pAsyncEntry->asyncHandler = asyncHandler;
                pAsyncEntry->asyncPending = asyncPending;
                pAsyncEntry->asyncTmo = pAsyncEntry->currTmo = asyncTmoMs;
                return pAsyncEntry;
            }
        }
    }

    return 0;
}


bool _TCPIPStackAsyncHandlerSetParams(tcpipAsyncHandle handle, tcpipModuleAsyncPending asyncPending, int16_t asyncTmoMs)
{
    TCPIP_STACK_ASYNC_MODULE_ENTRY* pAsyncEntry = (TCPIP_STACK_ASYNC_MODULE_ENTRY*)handle;
    if(pAsyncEntry->asyncHandler != 0)
    {   // minimim sanity check
        pAsyncEntry->asyncPending = asyncPending;
        pAsyncEntry->asyncTmo = pAsyncEntry->currTmo = asyncTmoMs;
        return true;
    }

    return false;
}

// de-registers a previous registered timeout handler
void _TCPIPStackAsyncHandlerDeRegister(tcpipAsyncHandle handle)
{
    TCPIP_STACK_ASYNC_MODULE_ENTRY* pAsyncEntry = (TCPIP_STACK_ASYNC_MODULE_ENTRY*)handle;

    memset(pAsyncEntry, 0x0, sizeof(*pAsyncEntry));
}

static void _TCPIPStackProcessTmo(void)
{
    int     ix;
    bool    isTmo;
    TCPIP_STACK_ASYNC_MODULE_ENTRY* pAsyncEntry;

    for(ix = 0, pAsyncEntry = TCPIP_STACK_MODULE_ASYNC_TBL; ix < sizeof(TCPIP_STACK_MODULE_ASYNC_TBL)/sizeof(*TCPIP_STACK_MODULE_ASYNC_TBL); ix++, pAsyncEntry++)
    {
        if(pAsyncEntry->asyncHandler == 0)
        {   // unused slot
            continue;
        }

        if(pAsyncEntry->asyncPending == 0)
        {   // stack manager maintained timeout
            isTmo = false;
            if(pAsyncEntry->asyncTmo != 0)
            {
                if((pAsyncEntry->currTmo -= stackTaskRate) <= 0)
                {   // timeout
                    pAsyncEntry->currTmo += pAsyncEntry->asyncTmo;
                    isTmo = true;
                }
            }
        }
        else
        {
            isTmo = (*pAsyncEntry->asyncPending)();
        }

        if(isTmo)
        {   // call the timeout handler
            (*pAsyncEntry->asyncHandler)();
        }
    }
}

// purges the interface queue of TCPIP_MAC_PACKET RX packets
static void _TCPIPStackRxListPurge(TCPIP_NET_IF* pNetIf)
{
    TCPIP_MAC_PACKET *pRxPkt;

    // kill the list
	while((pRxPkt = (TCPIP_MAC_PACKET*)TCPIP_Helper_ProtectedSingleListHeadRemove(&pNetIf->rxQueue)))
    {
        TCPIP_PKT_PacketAcknowledge(pRxPkt, TCPIP_MAC_PKT_ACK_SOURCE_ERR);
    }
}


// inserts a RX packet into the manager RX queue
// this has to be a fully formatted TCPIP_MAC_PACKET
void _TCPIPStackInsertRxPacket(TCPIP_NET_IF* pNetIf, TCPIP_MAC_PACKET* pRxPkt, bool signal)
{
    pRxPkt->pktFlags |= TCPIP_MAC_PKT_FLAG_QUEUED;
    // update the frame length
    pRxPkt->pDSeg->segLen -= sizeof(TCPIP_MAC_ETHERNET_HEADER);
    _TCPIPStackRxListInsert(pNetIf, pRxPkt);

    if(signal)
    {
        _TCPIP_MacEventCB(TCPIP_MAC_EV_RX_DONE, pNetIf);
    }
}


