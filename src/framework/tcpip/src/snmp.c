/*******************************************************************************
  Simple Network Management Protocol (SNMP) Version 1 Agent
  Simple Network Management Protocol (SNMP) Version 2 community based Agent

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides SNMP API for doing stuff
    -Reference: RFC 1157 (for SNMP V1)
                RFC 3416 (for SNMPv2C)
*******************************************************************************/

/*******************************************************************************
File Name:  SNMP.c
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

#define __SNMP_C

#include "tcpip/src/tcpip_private.h"
#if defined(TCPIP_STACK_USE_SNMP_SERVER)
#include "tcpip/src/snmp_private.h"
#include "tcpip/src/snmpv3_private.h"
#include "tcpip/src/common/sys_fs_wrapper.h"
#include "tcpip/snmp.h"
#include "mib.h"


#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
#include "tcpip/snmpv3.h"
#endif

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_SNMP_SERVER

#include "sys_fs_config.h"
#include "system/tmr/sys_tmr.h"

/****************************************************************************
  Section:
	Global Variables
  ***************************************************************************/
static uint16_t SNMPTxOffset;	//Snmp udp buffer tx offset
static uint16_t SNMPRxOffset;

static SNMP_STATUS SNMPStatus;	//MIB file access status

static reqVarErrStatus snmpReqVarErrStatus;


#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
static uint16_t msgSecrtyParamLenOffset;
static int	SNMPV3InitCount=0;
static SNMPV3_STACK_DCPT_STUB* Snmpv3StkStubPtr=0;
#endif 


static const void*  SnmpStackMemH = 0;        // memory handle

static SNMP_STACK_DCPT_STUB*  SnmpStackDcptMemStubPtr=0;

static SNMPV1V2_REQUEST_WHOLEMSG snmpV1V2cIncomingMsgBufPtr;

static TCPIP_SNMP_DCPT* snmpDcpt= 0;

#if defined (TCPIP_STACK_USE_IPV6)
static void SNMP_IPV6_Notify(TCPIP_NET_HANDLE hNet, uint8_t evType, const void * param);
#endif

//ASN format datatype for snmp v1 and v2c
static const SNMP_DATA_TYPE_INFO dataTypeTable[] =
{
    { ASN_INT,           1       }, //INT8_VAL          
    { ASN_INT,           2       }, //INT16_VAL         
    { ASN_INT,           4       }, //INT32_VAL         
    { OCTET_STRING,      0xff    }, //BYTE_ARRAY        
    { OCTET_STRING,      0xff    }, //ASCII_ARRAY       
    { SNMP_IP_ADDR,      4       }, //IPADDRESS        
    { SNMP_COUNTER32,    4       }, //COUNTER32         
    { SNMP_TIME_TICKS,   4       }, //TIME_TICKS_VAL    
    { SNMP_GAUGE32,      4       }, //GAUTE32           
    { ASN_OID,           0xff    }  //OID_VAL           
};

static int snmpInitCount = 0;      // SNMP module initialization count

static int32_t snmpFileDescrptr = SYS_FS_HANDLE_INVALID;
static int32_t snmpTrapFileDescriptr = SYS_FS_HANDLE_INVALID;


/****************************************************************************
  Section:
	Function Prototypes
  ***************************************************************************/

static void TCPIP_SNMP_PutDataArray(UDP_SOCKET socket,SNMP_BUFFER_DATA *snmpPutData);
static uint8_t TCPIP_SNMP_OIDsCountGet(uint16_t pdulen);
static SNMP_ERR_STATUS TCPIP_SNMP_ProcessPDUHeader(PDU_INFO * pduDbPtr, char* community, uint8_t* len);
static bool TCPIP_SNMP_ProcessHeaderGetSet(PDU_INFO * pduDbPtr);
static bool TCPIP_SNMP_ProcessVariables(PDU_INFO * pduDbPtr,char* community, uint8_t len);


static bool TCPIP_SNMP_OIDIsValid(uint8_t* oid, uint8_t* len);
static bool TCPIP_SNMP_CommunityStringIsValid(char* community, uint8_t* len);
static bool TCPIP_SNMP_PDUIsValid(SNMP_ACTION* pdu);
static bool TCPIP_SNMP_DataIsASNNull(void);
static bool TCPIP_SNMP_OIDStringGetByAddr(int32_t fileDescr,OID_INFO* rec, uint8_t* oidString, uint8_t* len);
static void TCPIP_SNMP_MIBRecordRead(int32_t fileDescr,uint32_t h, OID_INFO* rec);
static void TCPIP_SNMP_ErrorStatusSet(uint16_t errorStatusOffset, uint16_t errorIndexOffset,SNMP_ERR_STATUS errorStatus,uint8_t errorIndex,SNMP_BUFFER_DATA *snmpPutTxData);
static bool TCPIP_SNMP_PvtMIBObjIsRequested(uint8_t* OIDValuePtr);
static bool TCPIP_SNMP_MemoryAllocate(const TCPIP_STACK_MODULE_CTRL* const stackCtrl);
static void TCPIP_SNMP_StackDescriptorsInitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl);
static bool TCPIP_SNMP_DefaultUserConfigLoad(void);

#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
  extern  void TCPIP_SNMPv3_Initialize(TCPIP_NET_IF* netIf);
#endif /* TCPIP_STACK_USE_SNMPV3_SERVER */

/****************************************************************************
  ===========================================================================
  Section:
	SNMP v1 and v2c Agent Routines
  ===========================================================================
  ***************************************************************************/

/****************************************************************************
Function:
bool TCPIP_SNMP_Initialize(void)

Summary:
Initialize SNMP module internals.

Description:
    This function initializes the Snmp agent. One udp socket is intialized
    and opened at port 161. Agent will receive and transmit all the snmp
    pdus on this udp socket.

Precondition:
    At least one UDP socket must be available. TCPIP_UDP_Initialize() is already called.

Parameters:
    None

Returns:
    None

Remarks:
    This function is called only once during lifetime of the application.
    One UDP socket will be used.
 ***************************************************************************/
bool TCPIP_SNMP_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl,
              const SNMP_MODULE_CONFIG* snmpData)
{
    TCPIP_SNMP_DCPT *pDcpt;
	
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   
    	// interface restart
        return true;
    }

    if(snmpInitCount == 0)
    {   		
        TCPIP_SNMP_MemoryAllocate(stackCtrl);

        snmpDcpt = (TCPIP_SNMP_DCPT*)TCPIP_HEAP_Calloc(stackCtrl->memH,  stackCtrl->nIfs, sizeof(TCPIP_SNMP_DCPT));
        if(snmpDcpt == 0)
        {	// failed
            return false;
        }


        TCPIP_SNMP_StackDescriptorsInitialize(stackCtrl);	
		
        SNMPStatus.Val = 0;

        if(TCPIP_SNMP_DefaultUserConfigLoad()!= true)
            return false;
   }

    pDcpt = snmpDcpt + stackCtrl->netIx;
    pDcpt->sm = SNMP_HOME;
    pDcpt->skt = INVALID_UDP_SOCKET;
#if defined (TCPIP_STACK_USE_IPV6)
    pDcpt->snmpIPV6Handler = TCPIP_IPV6_HandlerRegister(0, (IPV6_EVENT_HANDLER)SNMP_IPV6_Notify, NULL);
    if (pDcpt->snmpIPV6Handler == NULL)
    {
      return false;
    }
    pDcpt->ipv6EventType = IPV6_EVENT_ADDRESS_REMOVED;
#endif
    snmpInitCount++;
    return true;
}

/****************************************************************************
  Function:
    void TCPIP_SNMP_Deinitialize(void)

  Summary:
    DeInitialize SNMP module internals.

  Description:
  	This function deinitializes the Snmp agent. Closes the UDP socket if all
    interfaces are down.		
  	
  Precondition:
	None.

  Parameters:
	None

  Returns:
	None
    
  Remarks:
	This function may be called many times during lifetime of the application.
 ***************************************************************************/
void TCPIP_SNMP_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    TCPIP_SNMP_DCPT *pDcpt;
    pDcpt = snmpDcpt + stackCtrl->netIx;

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // whole stack is going down
        if(snmpInitCount > 0)
        {   // we're up and running
            if(--snmpInitCount == 0)
            {   // all closed
                // release resources
                if(snmpDcpt != 0)
                {
                   TCPIP_HEAP_Free(stackCtrl->memH, snmpDcpt);
                   snmpDcpt = 0;
                }
            }
            if (pDcpt->skt != INVALID_UDP_SOCKET)
            {
                TCPIP_UDP_Close(pDcpt->skt);
                pDcpt->skt = INVALID_UDP_SOCKET;
            }
#if defined (TCPIP_STACK_USE_IPV6)
            TCPIP_IPV6_HandlerDeregister(pDcpt->snmpIPV6Handler);
#endif
        }
    }
}


static bool TCPIP_SNMP_MemoryAllocate(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    SnmpStackMemH=stackCtrl->memH;

    SnmpStackDcptMemStubPtr = (SNMP_STACK_DCPT_STUB *)TCPIP_HEAP_Malloc(SnmpStackMemH, (sizeof(SNMP_STACK_DCPT_STUB)
	
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER

                                + sizeof(SNMPV3_STACK_DCPT_STUB)
#endif

                                ));
    if(SnmpStackDcptMemStubPtr ==0)
    {
        return false;
    }
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
    Snmpv3StkStubPtr = (SNMPV3_STACK_DCPT_STUB*) (((unsigned long int)(SnmpStackDcptMemStubPtr)) + sizeof(SNMP_STACK_DCPT_STUB));
#endif

    return true;

}



void TCPIP_SNMP_PacketProcStubPtrsGet( SNMP_PROCESSING_MEM_INFO_PTRS * dynMemInfoPtr)
{
    dynMemInfoPtr->snmpHeapMemHandler=SnmpStackMemH;
    dynMemInfoPtr->snmpStkDynMemStubPtr=SnmpStackDcptMemStubPtr;
    dynMemInfoPtr->snmpDcptPtr=snmpDcpt;
}

#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
void TCPIP_SNMPV3_PacketProcStubPtrsGet( SNMPV3_PROCESSING_MEM_INFO_PTRS * dynMemInfoPtr)
{
    dynMemInfoPtr->snmpHeapMemHandler=SnmpStackMemH;
    dynMemInfoPtr->snmpv3StkProcessingDynMemStubPtr=Snmpv3StkStubPtr;
}
#endif


static void TCPIP_SNMP_StackDescriptorsInitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
     SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr;
     SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0;
#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3) && !defined(SNMP_TRAP_DISABLED)
     uint8_t usmTrapIndex=0;
     static const char * const cTrapUserSecurityName[] = SNMPV3_TRAP_USER_SECURITY_NAME_DB;
     static const uint8_t const eTrapMsgProcessModelType[] = SNMPV3_TRAP_MSG_PROCESS_MODEL_DB;
     static const uint8_t const  eTrapSecurityModelType[] = SNMPV3_TRAP_SECURITY_MODEL_TYPE_DB;
     static const uint8_t const  eTrapSecurityLevelType[] = SNMPV3_TRAP_SECURITY_LEVEL_DB;
#endif
    TCPIP_SNMPV3_PacketProcStubPtrsGet(&snmpv3PktProcessingMemPntr);
    snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;


#if defined(SNMP_V1_V2_TRAP_WITH_SNMPV3)&& !defined(SNMP_TRAP_DISABLED)
    //snmv3 global database for trap table
    for(;usmTrapIndex<SNMPV3_USM_MAX_USER;usmTrapIndex++)
    {
        memcpy(snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[usmTrapIndex].userSecurityName,cTrapUserSecurityName[usmTrapIndex]
                                                        ,strlen(cTrapUserSecurityName[usmTrapIndex]));
        snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[usmTrapIndex].messageProcessingModelType = eTrapMsgProcessModelType[usmTrapIndex];
        snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[usmTrapIndex].securityModelType = eTrapSecurityModelType[usmTrapIndex];
        snmpv3EngnDcptMemoryStubPtr->Snmpv3TrapConfigData[usmTrapIndex].securityLevelType = eTrapSecurityLevelType[usmTrapIndex];
    }
    #endif

    snmpv3EngnDcptMemoryStubPtr->SnmpInMsgAuthParamLen=12;
    snmpv3EngnDcptMemoryStubPtr->SnmpInMsgPrivParmLen=8;
    snmpv3EngnDcptMemoryStubPtr->SnmpOutMsgAuthParmLen=12;
    snmpv3EngnDcptMemoryStubPtr->SnmpOutMsgPrivParmLen=8;

#endif 

#if !defined(SNMP_TRAP_DISABLED)
    SnmpStackDcptMemStubPtr->gSendTrapFlag=false;//global flag to send Trap
    SnmpStackDcptMemStubPtr->gOIDCorrespondingSnmpMibID=MICROCHIP;
    SnmpStackDcptMemStubPtr->gGenericTrapNotification=ENTERPRISE_SPECIFIC;
    SnmpStackDcptMemStubPtr->gSpecificTrapNotification=VENDOR_TRAP_DEFAULT; // Vendor specific trap code
#endif

#if defined(SNMP_STACK_USE_V2_TRAP) && !defined(SNMP_TRAP_DISABLED)
    //if gSetTrapSendFlag == false then the last varbind variable for
    //multiple varbind variable pdu structure or if there is only varbind variable send.
    // if gSetTrapSendFlag == true, then v2 trap pdu is expecting more varbind variable.
    SnmpStackDcptMemStubPtr->gSetTrapSendFlag = false;
#endif /* SNMP_STACK_USE_V2_TRAP */

}


/****************************************************************************
Function:
    bool TCPIP_SNMP_DataCopyToProcessBuffer(uint8_t val ,SNMP_BUFFER_DATA *putbuf)

Summary:
    Copies uint8_t data to dynamically allocated memory buffer.

Description:
    The SNMPv1 and v2c stack implementation uses dynamically allocated memory buffer for
    processing of request and response packets. This routine copies the uint8_t data to the
    allocated buffer and updates the offset length couter.

Precondition:
    The SNMPv1 and v2c stack has sucessfully allocated dynamic memory buffer from the Heap

Parameters:
    val: uint8_t value to be written to the buffer
    putbuf: pointer to the dynamically allocated buffer to which the 'val' to be written

Return Values:
    true: if successfully write to the buffer
    false: failure in writing to the buffer
	
  Remarks:
  	This routine is used by the SNMP stack. If required to be used by the application
  	code, valid pointers should be passed to this routine. 
  	
***************************************************************************/
bool TCPIP_SNMP_DataCopyToProcessBuffer(uint8_t val ,SNMP_BUFFER_DATA *putbuf)
{
    if(putbuf->length < SNMP_MAX_MSG_SIZE)
    {
        putbuf->head[putbuf->length++] = (uint8_t)val;
        return true;
    }
    else
    {
        return false;
    }

}

/****************************************************************************
Function:
    bool TCPIP_SNMP_ProcessBufferDataGet(SNMP_BUFFER_DATA getbuf,uint16_t pos)

Summary:
    Reads uint8_t data from dynamically allocated memory buffer.

Description:
    The SNMPstack implementation uses dynamically allocated memory buffer for
    processing of request and response packets. This routine reads the uint8_t data from
    the allocated buffer at the positions (offset) provided.

Precondition:
    The SNMPv1 and v2c stack has sucessfully allocated dynamic memory buffer from the Heap

Parameters:
    getbuf: Structure from where to read the data byte.
    pos: position in the buffer from which the data to be read

Return Values:
    uint8_t: 1 byte value read

Remarks:
    The read position offset is required to be provided every time the routine is called.
    This API do not increment the buffer read offset automatically, everytime it is called.
  	
***************************************************************************/	
uint8_t TCPIP_SNMP_ProcessBufferDataGet(SNMP_BUFFER_DATA getbuf,uint16_t pos)
{
    return (uint8_t)(getbuf.head[pos]);
}


/****************************************************************************
Function:
    bool TCPIP_SNMP_Task(TCPIP_NET_IF* pNetIf)

Summary:
    Polls for every snmp pdu received.

Description:
    Handle incoming SNMP requests as well as any outgoing SNMP
    responses and timeout conditions.

Precondition:
    TCPIP_SNMP_Initialize() is already called.

Parameters:
    None

Return Values:
    true	-	If SNMP module has finished with a state
    false	-	If a state has not been finished.

Remarks:
    None
 ***************************************************************************/
bool TCPIP_SNMP_Task(TCPIP_NET_IF* pNetIf)
{
    char community[SNMP_COMMUNITY_MAX_LEN];
    uint8_t communityLen=0;
    PDU_INFO pduInfoDB; //received pdu information database
    bool lbReturn=true;
    int        netIx;
    void *SnmpBufptr;
    UDP_SOCKET     s;
    SNMP_ERR_STATUS snmpErr =SNMP_NO_ERR;

    if(!pNetIf)
    {
        return false;
    }
    else
    {
        netIx = TCPIP_STACK_NetIxGet(pNetIf);
    }

    s = snmpDcpt[netIx].skt;

    if(pNetIf->netIPAddr.Val == 0)
        return false;
   
#if defined (TCPIP_STACK_USE_IPV6)    
    if(snmpDcpt[pNetIf->netIfIx].ipv6EventType != IPV6_EVENT_ADDRESS_ADDED)
        return false;
#endif
    
    switch(snmpDcpt[netIx].sm)
    {
        case SNMP_HOME:
            // Open a SNMP agent socket
            s = TCPIP_UDP_ServerOpen(IP_ADDRESS_TYPE_ANY, SNMP_AGENT_PORT, 0);
            if(s == INVALID_UDP_SOCKET)
            {
                return false;
            }
            snmpDcpt[netIx].skt = s;
            TCPIP_UDP_SocketNetSet(s, pNetIf);

// intialize SNMP v3 with a proper pNetIf parameters 			
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
            if( SNMPV3InitCount == 0)
            {
                TCPIP_SNMPv3_Initialize(pNetIf);
                SNMPV3InitCount++;
            }
#endif
            snmpDcpt[netIx].sm=SNMP_LISTEN;
            break;

        case SNMP_LISTEN:
            // Do nothing if no data is waiting
            if(!TCPIP_UDP_GetIsReady(s))
            {
                return false;
            }
            // As we process SNMP variables, we will prepare response on-the-fly
            // creating full duplex transfer.
            // Current MAC layer does not support full duplex transfer, so
            // SNMP needs to manage its own full duplex connection.
            // Prepare for full duplex transfer.
            if(!TCPIP_SNMP_PDUProcessDuplexInit(s))
            {
                return false;
            }

            communityLen = 0;	// Suppress C30 warning: 'communityLen' may be used uninitialized in this function
            snmpErr = TCPIP_SNMP_ProcessPDUHeader(&pduInfoDB,community, &communityLen);
            // We received a discovery request, reply when we can
            snmpDcpt[netIx].sm=SNMP_PROCESS;
            //break;
        case SNMP_PROCESS:

            if(pduInfoDB.snmpVersion != SNMP_V3)
            {
                if ( snmpErr != SNMP_NO_ERR)
                {
                    snmpDcpt[netIx].sm = SNMP_PACKET_DISCARD;
                    break;
                }
                if ( !TCPIP_SNMP_ProcessHeaderGetSet(&pduInfoDB))
                {
                    snmpDcpt[netIx].sm = SNMP_PACKET_DISCARD;
                    break;
                }
            }

            // Open MIB file.
            SNMPStatus.Flags.bIsFileOpen = false;

            snmpFileDescrptr= SYS_FS_FileOpen_Wrapper((const char*)SNMP_BIB_FILE_NAME,SYS_FS_FILE_OPEN_READ);
            if(snmpFileDescrptr != SYS_FS_HANDLE_INVALID)
            {
               SNMPStatus.Flags.bIsFileOpen = true;
            }
            if(SNMPStatus.Flags.bIsFileOpen == false)
			{
				snmpDcpt[netIx].sm = SNMP_PACKET_DISCARD;
				break;
			}

            if(pduInfoDB.snmpVersion != SNMP_V3) // if(SNMP_V1, SNMP_V2C)
            {
                SnmpBufptr = TCPIP_HEAP_Calloc(SnmpStackMemH,1,(size_t)SNMP_MAX_MSG_SIZE);
                if(SnmpBufptr)
                {
                    SnmpStackDcptMemStubPtr->outPduBufData.head = (uint8_t *)SnmpBufptr;
                }
                lbReturn = TCPIP_SNMP_ProcessVariables(&pduInfoDB,community, communityLen);
                if(snmpV1V2cIncomingMsgBufPtr.wholeMsgHead != 0)
                {
                    TCPIP_HEAP_Free(SnmpStackMemH, snmpV1V2cIncomingMsgBufPtr.wholeMsgHead);
                    snmpV1V2cIncomingMsgBufPtr.wholeMsgLen=0;
                    snmpV1V2cIncomingMsgBufPtr.wholeMsgHead=0;
                    snmpV1V2cIncomingMsgBufPtr.snmpMsgHead=0;
                }
            }

#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
            else
            {
                lbReturn = TCPIP_SNMPv3_V3MsgDataProcess(&pduInfoDB);
            }
#endif

            if(SNMPStatus.Flags.bIsFileOpen)
            {
				SYS_FS_FileClose(snmpFileDescrptr);
               	snmpFileDescrptr= SYS_FS_HANDLE_INVALID;
            }

            if(lbReturn == false)
            {
                if(SnmpStackDcptMemStubPtr->outPduBufData.head !=0x00)
                {
                    TCPIP_HEAP_Free(SnmpStackMemH, SnmpStackDcptMemStubPtr->outPduBufData.head);
                }
                SnmpStackDcptMemStubPtr->outPduBufData.length = 0;
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
                TCPIP_SNMPv3_DynAllocMemFree();
#endif 
                snmpDcpt[netIx].sm = SNMP_PACKET_DISCARD;
                break;
            }

            if(SnmpStackDcptMemStubPtr->gSendTrapFlag==(uint8_t)false)
            {
                TCPIP_UDP_Flush(s);
            }
            snmpDcpt[netIx].sm=SNMP_PACKET_DISCARD;
            //break;
        case SNMP_PACKET_DISCARD:

            if(snmpV1V2cIncomingMsgBufPtr.wholeMsgHead != 0)
            {
                TCPIP_HEAP_Free(SnmpStackMemH, snmpV1V2cIncomingMsgBufPtr.wholeMsgHead);
                snmpV1V2cIncomingMsgBufPtr.wholeMsgLen=0;
                snmpV1V2cIncomingMsgBufPtr.wholeMsgHead=0;
                snmpV1V2cIncomingMsgBufPtr.snmpMsgHead=0;
            }
            TCPIP_UDP_Discard(s);
            // clear v1/v2c outbuf dynamic buffer data
            if(SnmpStackDcptMemStubPtr->outPduBufData.head !=0x00)
            {
                TCPIP_HEAP_Free(SnmpStackMemH, SnmpStackDcptMemStubPtr->outPduBufData.head);
                SnmpStackDcptMemStubPtr->outPduBufData.head=0x00;
                SnmpStackDcptMemStubPtr->outPduBufData.length = 0;
            }

#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
        TCPIP_SNMPv3_DynAllocMemFree();
#endif
        snmpDcpt[netIx].sm = SNMP_LISTEN;
        break;
    }
    return true;
}


#if !defined(SNMP_TRAP_DISABLED)
static uint32_t  snmpTrapTimer=0;


/****************************************************************************
  Function:
	void TCPIP_SNMP_NotifyPrepare(IP_MULTI_ADDRESS* remoteHost,
                           char* community,
                           uint8_t communityLen,
                           SNMP_ID agentIDVar,
                           uint8_t notificationCode,
                           uint32_t timestamp)

  Summary:
	Collects trap notification info and send ARP to remote host.

  Description:
	This function prepares SNMP module to send SNMP trap notification
	to remote host. It sends ARP request to remote host to learn remote
	host MAC address.
	
  Precondition:
	TCPIP_SNMP_Initialize() is already called.

  Parameters:
	remoteHost  - pointer to remote Host IP address
	community   - Community string to use to notify
	communityLen- Community string length
	agentIDVar  - System ID to use identify this agent
	notificaitonCode - Notification Code to use
	timestamp   - Notification timestamp in 100th of second.

  Returns:
	None
		
  Remarks:
	This is first of series of functions to complete SNMP notification.
 ***************************************************************************/
void TCPIP_SNMP_NotifyPrepare(IP_MULTI_ADDRESS* remoteHost,
                        char* community,
                        uint8_t communityLen,
                        SNMP_ID agentIDVar,
                        uint8_t notificationCode,
                        uint32_t timestamp )
{
    uint8_t intfIdx = 0;
    static IP_MULTI_ADDRESS* remHostIpAddrPtr;
    remHostIpAddrPtr = remoteHost;

    intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_NetDefaultGet())->netIfIx;
    strcpy(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.community, community);
    SnmpStackDcptMemStubPtr->SNMPNotifyInfo.communityLen = communityLen;

    SnmpStackDcptMemStubPtr->SNMPNotifyInfo.agentIDVar = agentIDVar;
    SnmpStackDcptMemStubPtr->SNMPNotifyInfo.notificationCode = notificationCode;

    SnmpStackDcptMemStubPtr->SNMPNotifyInfo.timestamp = timestamp;
    SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket = INVALID_UDP_SOCKET;

}

/****************************************************************************
Function:
    bool TCPIP_SNMP_NotifyIsReady(IP_MULTI_ADDRESS* remoteHost,SNMP_TRAP_IP_ADDRESS_TYPE eTrapMultiAddressType)

Summary:
    Resolves given remoteHost IP address into MAC address.

Description:
    This function resolves given remoteHost IP address into MAC address using
    ARP module. If remoteHost is not aviailable, this function would never
    return true. Application must implement timeout logic to handle
    "remoteHost not avialable" situation.

Precondition:
    TCPIP_SNMP_NotifyPrepare() is already called.

Parameters:
    remoteHost  - Pointer to remote Host IP address

Return Values:
    true	-	If remoteHost IP address is resolved and
                            TCPIP_SNMP_Notify may be called.
    false	-	If remoteHost IP address is not resolved.

Remarks:
    This would fail if there were not UDP socket to open.
 ***************************************************************************/
bool TCPIP_SNMP_NotifyIsReady(IP_MULTI_ADDRESS* remoteHost,SNMP_TRAP_IP_ADDRESS_TYPE eTrapMultiAddressType)
{
    TCPIP_NET_IF *pNetIf;

    IP_MULTI_ADDRESS remoteAddress;
    if(eTrapMultiAddressType == IPV4_SNMP_TRAP)
    {
        remoteAddress.v4Add.Val = remoteHost->v4Add.Val;
    }
    else
    {
        memcpy(&remoteAddress.v6Add,&remoteHost->v6Add,16);
    }
	
    pNetIf = _TCPIPStackHandleToNet(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf);
    if(eTrapMultiAddressType == IPV4_SNMP_TRAP)
    {
        if(pNetIf->netIPAddr.Val == 0)
        {   
            return false;
        }        
    }
#ifdef TCPIP_STACK_USE_IPV6
    if(eTrapMultiAddressType == IPV6_SNMP_TRAP)
    {
        if(TCPIP_SNMP_EventNotifyGet(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf) != IPV6_EVENT_ADDRESS_ADDED)
        {   
            return false;
        }        
    }
#endif      

    if(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket == INVALID_UDP_SOCKET)
    {
        if(eTrapMultiAddressType == IPV4_SNMP_TRAP)
        {
            SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket = TCPIP_UDP_ClientOpen(IP_ADDRESS_TYPE_IPV4,SNMP_NMS_PORT,&remoteAddress);
        }
        else
        {
            SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket = TCPIP_UDP_ClientOpen(IP_ADDRESS_TYPE_IPV6,SNMP_NMS_PORT,&remoteAddress);
        }

        if(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket != INVALID_UDP_SOCKET)
        {
            TCPIP_UDP_SocketNetSet(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket,SnmpStackDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf);
            snmpTrapTimer = SYS_TMR_TickCountGet();

        }
        else
        {
            return false;
        }
    }


    if(TCPIP_UDP_IsOpened(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket)== true)
    {
        snmpTrapTimer = SYS_TMR_TickCountGet();
        return true;
    }
    else
    {
        return false;
    }

    return true;
}

uint32_t TCPIP_SNMP_TrapTimeGet(void)

{
    return snmpTrapTimer;
}

/****************************************************************************
Function:
    uint8_t *TCPIP_SNMP_GenericTrapCodeToTrapOID(uint8_t generic_trap_code,uint8_t *len)
Summary:
    Resolves generic trap code to generic trap OID.

Description:
    This function resolves given generic trap code  to generic trap OID.

Precondition:
    TCPIP_SNMP_NotifyPrepare() is already called.

Parameters:
    generic_trap_code  - SNMP_GENERIC_TRAP_NOTIFICATION_TYPE
    len 	- generic trap OID length

Return Values:
    uint8_t *- TRAP OID

Remarks:
    This would fail if generic_trap_code is not coming under
    SNMP_GENERIC_TRAP_NOTIFICATION_TYPE
 ***************************************************************************/

uint8_t *TCPIP_SNMP_GenericTrapCodeToTrapOID(uint8_t generic_trap_code,uint8_t *len)
{
    static  uint8_t gen_trap_oid[] = {0x2b,6,1,6,3,1,1,5,1};
/*	
	static  uint8_t cold_trap_oid[] = {0x2b,6,1,6,3,1,1,5,1};
    static  uint8_t warm_start_oid = {0x2b,6,1,6,3,1,1,5,2};
    static  uint8_t auth_fail_oid  = {0x2b,6,1,6,3,1,1,5,5};
    static  uint8_t linkdown_oid   = {0x2b,6,1,6,3,1,1,5,3};
    static  uint8_t linkup_oid     = {0x2b,6,1,6,3,1,1,5,4};
*/
    static uint8_t snmptrap_oids[]  = {0x2b,6,1,6,3,1,1,4,1 };

    *len = sizeof(gen_trap_oid);
    switch (generic_trap_code) 
    {
        case COLD_START:
            gen_trap_oid[*len-1] = 1;
            break;
        case WARM_START:
            gen_trap_oid[*len-1] = 2;
            break;
        case LINK_UP:
            gen_trap_oid[*len-1] = 4;
            break;
        case LINK_DOWN:
            gen_trap_oid[*len-1] = 3;
            break;
        case AUTH_FAILURE:
            gen_trap_oid[*len-1] = 5;
            break;
	case ENTERPRISE_SPECIFIC:
            *len = sizeof(snmptrap_oids);
            return snmptrap_oids;
        default:
            return NULL;

    } /* switch (generic_trap_code) */

    return gen_trap_oid;

} /* end getSnmpV2TrapOid() */

/****************************************************************************
Function:
    bool TCPIP_SNMP_Notify(SNMP_ID var,SNMP_VAL val,SNMP_INDEX index)

Summary:
    Creates and Sends TRAP pdu.

Description:
    This function creates SNMP V2 Trap PDU and sends it to previously specified
    remoteHost.

    snmpv1 trap pdu:
   | PDU-type | enterprise | agent-addr | generic-trap | specific-trap |
   | time-stamp | varbind-list |

   The v1 enterprise is mapped directly to SNMPv2TrapOID.0
    SNMP v2 trap pdu:
   version (0 or 1) | community | SNMP-PDU |pdu-type | request-id | error-status
   |err-index |varbinds

   The first two variables (in varbind-list) of snmpv2 are: sysUpTime.0 and
    SNMPv2TrapOID.0

    Generic Trap OID is used as the varbind for authentication failure.

Precondition:
    TCPIP_SNMP_NotifyIsReady() is already called and returned true.

Parameters:
    var     - SNMP var ID that is to be used in notification
    val     - Value of var. Only value of uint8_t, uint16_t or uint32_t can be sent.
    index   - Index of var. If this var is a single,index would be 0, or else
                      if this var Is a sequence, index could be any value
                      from 0 to 127

Return Values:
    true	-	if SNMP notification was successful sent.
                            This does not guarantee that remoteHost recieved it.
    false	-	Notification sent failed.
                        This would fail under following contions:
                        1) Given SNMP_BIB_FILE does not exist in file system
                        2) Given var does not exist.
                        3) Previously given agentID does not exist
                        4) Data type of given var is unknown -
                           possible if file system itself was corrupted.

Remarks:
    This would fail if there were not UDP socket to open.
 ***************************************************************************/
#if defined(SNMP_STACK_USE_V2_TRAP)

bool TCPIP_SNMP_Notify(SNMP_ID var, SNMP_VAL val, SNMP_INDEX index)
{
    char* pCommunity;
    uint8_t len;
    uint8_t OIDValue[SNMP_OID_MAX_LEN];
    uint8_t OIDLen;
    static uint32_t varbindlen = 0;
    uint8_t agentIDLen;
    uint8_t* pOIDValue;
    static uint16_t packetStructLenOffset = 0;
    static uint16_t pduStructLenOffset = 0;
    static uint16_t varBindStructLenOffset = 0;
    static uint16_t varPairStructLenOffset = 0;
    static uint16_t prevOffset = 0;
    uint16_t tempOffset = 0;
    OID_INFO rec;
    SNMP_DATA_TYPE_INFO dataTypeInfo;
    uint8_t	snmptrap_oids[]  = {0x2b,6,1,6,3,1,1,4,1 }; /* len=10 */
    uint8_t	sysUpTime_oids[] = {0x2b,6,1,2,1,1,3}; /* len = 8 */
    TCPIP_UINT16_VAL trapVarBindLen={0};
    int i=0;
    SNMP_BUFFER_DATA *snmpTrapPutData=NULL;
    uint8_t intfIdx;

    intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_NetDefaultGet())->netIfIx;
    if(snmpTrapFileDescriptr == SYS_FS_HANDLE_INVALID)
    {
        snmpTrapFileDescriptr = SYS_FS_FileOpen_Wrapper((const char*)SNMP_BIB_FILE_NAME,SYS_FS_FILE_OPEN_READ);
        if(snmpTrapFileDescriptr == SYS_FS_HANDLE_INVALID)
        {
            TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
            SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket = INVALID_UDP_SOCKET;
            return false;
        }
    }
	
    if((packetStructLenOffset == 0)&&(pduStructLenOffset==0))
    {
        if(!TCPIP_SNMP_PDUProcessDuplexInit(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket))
        {
            TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
            SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket = INVALID_UDP_SOCKET;
            return false;
        }
        SnmpStackDcptMemStubPtr->trapPduOutBufData.head = TCPIP_HEAP_Calloc(SnmpStackMemH,1,(size_t)SNMP_MAX_MSG_SIZE);
        if(!SnmpStackDcptMemStubPtr->trapPduOutBufData.head)
        {
            TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
            SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket = INVALID_UDP_SOCKET;
            return false;
        }
        SnmpStackDcptMemStubPtr->trapPduOutBufData.length = 0;
        snmpTrapPutData = &SnmpStackDcptMemStubPtr->trapPduOutBufData;
        prevOffset =  snmpTrapPutData->length;

        len = SnmpStackDcptMemStubPtr->SNMPNotifyInfo.communityLen;
        pCommunity = SnmpStackDcptMemStubPtr->SNMPNotifyInfo.community;

        TCPIP_SNMP_DataCopyToProcessBuffer(STRUCTURE,snmpTrapPutData);			// First item is packet structure
        TCPIP_SNMP_DataCopyToProcessBuffer(0x82,snmpTrapPutData);
        packetStructLenOffset = snmpTrapPutData->length;
        TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);
        TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);

        // Put SNMP version info.
        TCPIP_SNMP_DataCopyToProcessBuffer(ASN_INT,snmpTrapPutData);				// Int type.
        TCPIP_SNMP_DataCopyToProcessBuffer(1,snmpTrapPutData);					// One byte long value.
        TCPIP_SNMP_DataCopyToProcessBuffer(SNMP_V2C,snmpTrapPutData); 		  // v2

        //len = strlen(community);	// Save community length for later use.
        TCPIP_SNMP_DataCopyToProcessBuffer(OCTET_STRING,snmpTrapPutData); 		// Octet string type.
        TCPIP_SNMP_DataCopyToProcessBuffer(len,snmpTrapPutData);					// community string length
        while( len-- )					// Copy entire string.
            TCPIP_SNMP_DataCopyToProcessBuffer(*(pCommunity++),snmpTrapPutData);

        //TRAP Version type.
        TCPIP_SNMP_DataCopyToProcessBuffer(SNMP_V2_TRAP,snmpTrapPutData);
        TCPIP_SNMP_DataCopyToProcessBuffer(0x82,snmpTrapPutData);
        pduStructLenOffset = snmpTrapPutData->length;
        TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);
        TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);

        //put Request ID for the trapv2 as 1
        TCPIP_SNMP_DataCopyToProcessBuffer(ASN_INT,snmpTrapPutData);	// Int type.
        TCPIP_SNMP_DataCopyToProcessBuffer(4,snmpTrapPutData);		// To simplify logic, always use 4 byte long requestID
        TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);
        TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);
        TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);
        TCPIP_SNMP_DataCopyToProcessBuffer(1,snmpTrapPutData);

        // Put error status.
        TCPIP_SNMP_DataCopyToProcessBuffer(ASN_INT,snmpTrapPutData);				// Int type
        TCPIP_SNMP_DataCopyToProcessBuffer(1,snmpTrapPutData);					// One byte long.
        TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);					// Placeholder.

        // Similarly put error index.
        TCPIP_SNMP_DataCopyToProcessBuffer(ASN_INT,snmpTrapPutData);				// Int type
        TCPIP_SNMP_DataCopyToProcessBuffer(1,snmpTrapPutData);					// One byte long
        TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);					// Placeholder.

        // Variable binding structure header
        TCPIP_SNMP_DataCopyToProcessBuffer(0x30,snmpTrapPutData);
        TCPIP_SNMP_DataCopyToProcessBuffer(0x82,snmpTrapPutData);
        varBindStructLenOffset = snmpTrapPutData->length; //SNMPTxOffset;
        TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);
        TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);

        // Create variable name-pair structure
        TCPIP_SNMP_DataCopyToProcessBuffer(0x30,snmpTrapPutData);
        varPairStructLenOffset = snmpTrapPutData->length; //SNMPTxOffset;
        TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);

        // Set 1st varbind object i,e sysUpTime.0 time stamp for the snmpv2 trap
        // Get complete notification variable OID string.

        TCPIP_SNMP_DataCopyToProcessBuffer(ASN_OID,snmpTrapPutData);
        OIDLen = (uint8_t)sizeof(sysUpTime_oids);
        TCPIP_SNMP_DataCopyToProcessBuffer((uint8_t)(OIDLen)+1,snmpTrapPutData);
        pOIDValue = sysUpTime_oids;
        while( OIDLen-- )
            TCPIP_SNMP_DataCopyToProcessBuffer(*pOIDValue++,snmpTrapPutData);

        //1st varbind	 and this is a scalar object so index = 0
        TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);

        // Time stamp
        TCPIP_SNMP_DataCopyToProcessBuffer(SNMP_TIME_TICKS,snmpTrapPutData);
        TCPIP_SNMP_DataCopyToProcessBuffer(4,snmpTrapPutData);

        TCPIP_UINT32_VAL tStamp;
        tStamp.Val = SnmpStackDcptMemStubPtr->SNMPNotifyInfo.timestamp;
        TCPIP_SNMP_DataCopyToProcessBuffer(tStamp.v[3],snmpTrapPutData);
        TCPIP_SNMP_DataCopyToProcessBuffer(tStamp.v[2],snmpTrapPutData);
        TCPIP_SNMP_DataCopyToProcessBuffer(tStamp.v[1],snmpTrapPutData);
        TCPIP_SNMP_DataCopyToProcessBuffer(tStamp.v[0],snmpTrapPutData);

        tempOffset = snmpTrapPutData->length;
        //set the snmp time varbind trap offset
        snmpTrapPutData->length = varPairStructLenOffset;

        // SNMP time stamp varbind length
        OIDLen = 2							// 1st varbind header
           + (uint8_t)sizeof(sysUpTime_oids)
           + 1						   // index byte
           + 6 ;						// time stamp

        TCPIP_SNMP_DataCopyToProcessBuffer(OIDLen,snmpTrapPutData);
        //set the previous TX offset
        snmpTrapPutData->length = tempOffset;
        varbindlen += OIDLen // varbind length
                  + 2;  // varbind type(30) and length of individual varbind pdu

        // Set 2nd varbind object i,e snmpTrapOID.0 for the snmpv2 trap
        // Get complete notification variable OID string.

        // Create variable name-pair structure
        TCPIP_SNMP_DataCopyToProcessBuffer(0x30,snmpTrapPutData);
        varPairStructLenOffset = snmpTrapPutData->length; //SNMPTxOffset;
        TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);

        // Copy OID string into PDU.
        TCPIP_SNMP_DataCopyToProcessBuffer(ASN_OID,snmpTrapPutData);
        OIDLen = (uint8_t)sizeof(snmptrap_oids);
        TCPIP_SNMP_DataCopyToProcessBuffer((uint8_t)(OIDLen)+1,snmpTrapPutData);

        pOIDValue = snmptrap_oids;
        while( OIDLen-- )
            TCPIP_SNMP_DataCopyToProcessBuffer(*pOIDValue++,snmpTrapPutData);

        //2nd varbind  and this is a scalar object so index = 0
        TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);
        if ( !TCPIP_SNMP_OIDStringFindByID(snmpTrapFileDescriptr,SnmpStackDcptMemStubPtr->SNMPNotifyInfo.trapIDVar, &rec, OIDValue, &OIDLen) )
        {
            SYS_FS_FileClose(snmpTrapFileDescriptr);
            TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
            TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
            snmpTrapFileDescriptr = SYS_FS_HANDLE_INVALID;
            return false;
        }
        TCPIP_SNMP_DataCopyToProcessBuffer(ASN_OID,snmpTrapPutData);
        agentIDLen = OIDLen;
        len  = OIDLen;
        TCPIP_SNMP_DataCopyToProcessBuffer(agentIDLen,snmpTrapPutData);
        for(i=0;i<len;i++)
        {
            TCPIP_SNMP_DataCopyToProcessBuffer(OIDValue[i],snmpTrapPutData);
        }
        tempOffset = snmpTrapPutData->length;
        //set the snmp varbind trap offset
        snmpTrapPutData->length = varPairStructLenOffset;
        // Snmp trap varbind length
        OIDLen = 2					 // Agent ID header bytes
                + (uint8_t)sizeof(snmptrap_oids)
                + 1 					   // index byte
                + 2 					 // header
                + agentIDLen;				 // Agent ID bytes
        TCPIP_SNMP_DataCopyToProcessBuffer(OIDLen,snmpTrapPutData);

        //set the previous TX offset
        snmpTrapPutData->length = tempOffset;
        varbindlen += OIDLen // varbind length
                   + 2;	 // varbind type(30) and length of individual varbind pdu	
    }
    else
    { // collect the last varbind offset value.
        snmpTrapPutData = &SnmpStackDcptMemStubPtr->trapPduOutBufData;
        snmpTrapPutData->length = varPairStructLenOffset;
    }

    // Create variable name-pair structure
    TCPIP_SNMP_DataCopyToProcessBuffer(0x30,snmpTrapPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(0x82,snmpTrapPutData);
    varPairStructLenOffset = snmpTrapPutData->length;
    TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);
    /* to send generic trap */
    if(SnmpStackDcptMemStubPtr->gGenericTrapNotification != ENTERPRISE_SPECIFIC)
    {
        pOIDValue = TCPIP_SNMP_GenericTrapCodeToTrapOID(SnmpStackDcptMemStubPtr->gGenericTrapNotification,&OIDLen);
        if(pOIDValue == NULL)
        {
            SYS_FS_FileClose(snmpTrapFileDescriptr);
            snmpTrapFileDescriptr= SYS_FS_HANDLE_INVALID;
            TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
            TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
            return false;
        }
        // Copy OID string into PDU.
        TCPIP_SNMP_DataCopyToProcessBuffer(ASN_OID,snmpTrapPutData);
        TCPIP_SNMP_DataCopyToProcessBuffer((uint8_t)(OIDLen)+1,snmpTrapPutData);
        while( OIDLen-- )
            TCPIP_SNMP_DataCopyToProcessBuffer(*pOIDValue++,snmpTrapPutData);

        //2nd varbind  and this is a scalar object so index = 0
        TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);
        // for microchip , SnmpStackDcptMemStubPtr->SNMPNotifyInfo.agentIDVar == MICROCHIP
        if ( !TCPIP_SNMP_OIDStringFindByID(snmpTrapFileDescriptr,SnmpStackDcptMemStubPtr->SNMPNotifyInfo.agentIDVar, &rec, OIDValue, &OIDLen) )
        {
            SYS_FS_FileClose(snmpTrapFileDescriptr);
            snmpTrapFileDescriptr= SYS_FS_HANDLE_INVALID;
            TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
            TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
            return false;
        }
        if ( !rec.nodeInfo.Flags.bIsAgentID )
        {
            SYS_FS_FileClose(snmpTrapFileDescriptr);
            snmpTrapFileDescriptr= SYS_FS_HANDLE_INVALID;
            TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
            TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
            return false;
        }

        if(SYS_FS_FileSeek(snmpTrapFileDescriptr,rec.hData, SYS_FS_SEEK_SET) == -1)
        {
            SYS_FS_FileClose(snmpTrapFileDescriptr);
            snmpTrapFileDescriptr= SYS_FS_HANDLE_INVALID;
            TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
            TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
            return false;
        }
        TCPIP_SNMP_DataCopyToProcessBuffer(ASN_OID,snmpTrapPutData);
        if(SYS_FS_FileRead(snmpTrapFileDescriptr, (uint8_t*)&len,1) == -1)
        {
            SYS_FS_FileClose(snmpTrapFileDescriptr);
            snmpTrapFileDescriptr= SYS_FS_HANDLE_INVALID;
            TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
            TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
            return false;
        }

        agentIDLen = len;
        TCPIP_SNMP_DataCopyToProcessBuffer(agentIDLen,snmpTrapPutData);
        while( len-- )
        {
            uint8_t c;
            SYS_FS_FileRead(snmpTrapFileDescriptr,(uint8_t*)&c,1);
            TCPIP_SNMP_DataCopyToProcessBuffer(c,snmpTrapPutData);
        }
        tempOffset = snmpTrapPutData->length;
        //set the snmp varbind trap offset
        snmpTrapPutData->length = varPairStructLenOffset;
        // Snmp trap varbind length
        trapVarBindLen.Val = 2					 // Agent ID header bytes
                + (uint8_t)sizeof(snmptrap_oids)
                + 1 					   // index byte
                + 2 					 // header
                + agentIDLen;				 // Agent ID bytes
        TCPIP_SNMP_DataCopyToProcessBuffer(trapVarBindLen.v[1],snmpTrapPutData);
        TCPIP_SNMP_DataCopyToProcessBuffer(trapVarBindLen.v[0],snmpTrapPutData);
        len = trapVarBindLen.Val;
    }
    else
    {
        // Get complete notification variable OID string.
        if ( !TCPIP_SNMP_OIDStringFindByID(snmpTrapFileDescriptr,var, &rec, OIDValue, &OIDLen) )
        {
            SYS_FS_FileClose(snmpTrapFileDescriptr);
            snmpTrapFileDescriptr= SYS_FS_HANDLE_INVALID;
            TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
            TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
            return false;
        }

        pOIDValue = OIDValue;

        // Copy OID string into packet.
        TCPIP_SNMP_DataCopyToProcessBuffer(ASN_OID,snmpTrapPutData);
        TCPIP_SNMP_DataCopyToProcessBuffer((uint8_t)(OIDLen+1),snmpTrapPutData);
        len = OIDLen;
        while( len-- )
        {
            TCPIP_SNMP_DataCopyToProcessBuffer(*pOIDValue++,snmpTrapPutData);
        }
        TCPIP_SNMP_DataCopyToProcessBuffer(index,snmpTrapPutData);

        // Encode and Copy actual data bytes
        if ( !TCPIP_SNMP_DataTypeInfoGet(rec.dataType, &dataTypeInfo) )
        {
            SYS_FS_FileClose(snmpTrapFileDescriptr);
            snmpTrapFileDescriptr= SYS_FS_HANDLE_INVALID;
            TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
            TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
            return false;
        }
        TCPIP_SNMP_DataCopyToProcessBuffer(dataTypeInfo.asnType,snmpTrapPutData);
         //Modified to Send trap even for  dataTypeInfo.asnType= ASCII_STRING,
        //where dataTypeInfo.asnLen=0xff
        if ( dataTypeInfo.asnLen == 0xff )
        {
            uint8_t *asciiStr= (uint8_t *)val.dword;
            int k=0;
            dataTypeInfo.asnLen=strlen((char *)asciiStr);
            len = dataTypeInfo.asnLen;
            TCPIP_SNMP_DataCopyToProcessBuffer(len,snmpTrapPutData);
            for(k=0;k<len;k++)
            {
                TCPIP_SNMP_DataCopyToProcessBuffer(asciiStr[k],snmpTrapPutData);
            }
        }
        else
        {
            len = dataTypeInfo.asnLen;
            TCPIP_SNMP_DataCopyToProcessBuffer(len,snmpTrapPutData);
            while( len-- )
            {
                TCPIP_SNMP_DataCopyToProcessBuffer(val.v[len],snmpTrapPutData);
            }
        }

        trapVarBindLen.Val = dataTypeInfo.asnLen	// data bytes count
                 + 1					// Length byte
                 + 1					// Data type byte
                 + OIDLen				// OID bytes
                 + 2					// OID header bytes
                 + 1;					// index byte
        tempOffset = snmpTrapPutData->length;
        snmpTrapPutData->length = varPairStructLenOffset;
        TCPIP_SNMP_DataCopyToProcessBuffer(trapVarBindLen.v[1],snmpTrapPutData);
        TCPIP_SNMP_DataCopyToProcessBuffer(trapVarBindLen.v[0],snmpTrapPutData);
    }
    //set the previous TX offset
    snmpTrapPutData->length = tempOffset;
    varPairStructLenOffset = tempOffset;

    varbindlen += trapVarBindLen.Val // length of varbind
                            +4; // varbind type(30) and 0x82 , lenght1 and length2 of individual varbind pdu
    if(SnmpStackDcptMemStubPtr->gSetTrapSendFlag == true)
    {
        SYS_FS_FileClose(snmpTrapFileDescriptr);
        snmpTrapFileDescriptr= SYS_FS_HANDLE_INVALID;
        return true;
    }
    trapVarBindLen.Val = varbindlen;
    snmpTrapPutData->length = varBindStructLenOffset;
    TCPIP_SNMP_DataCopyToProcessBuffer(trapVarBindLen.v[1],snmpTrapPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(trapVarBindLen.v[0],snmpTrapPutData);
    trapVarBindLen.Val = varbindlen
    + 4 				   //  Variable Binding structure header(0x30,0x82,length1,length2)
    + 12;					// req , error and error status for SNMPv2

    snmpTrapPutData->length = pduStructLenOffset;
    TCPIP_SNMP_DataCopyToProcessBuffer(trapVarBindLen.v[1],snmpTrapPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(trapVarBindLen.v[0],snmpTrapPutData);


    trapVarBindLen.Val = trapVarBindLen.Val 						  // PDU struct length
    + 4 							// PDU trap header
    + SnmpStackDcptMemStubPtr->SNMPNotifyInfo.communityLen			 // Community string bytes
    + 2 							// Community header bytes
    + 3;							// SNMP version bytes


    snmpTrapPutData->length = packetStructLenOffset;
    TCPIP_SNMP_DataCopyToProcessBuffer(trapVarBindLen.v[1],snmpTrapPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(trapVarBindLen.v[0],snmpTrapPutData);

    snmpTrapPutData->length = tempOffset;

// after setting all the offset values, initialize all static variables to 0.
    packetStructLenOffset = 0;
    pduStructLenOffset = 0;
    varBindStructLenOffset = 0;
    varPairStructLenOffset = 0;
    prevOffset = 0;
    varbindlen = 0;

    TCPIP_SNMP_PutDataArray(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket,snmpTrapPutData);

    SYS_FS_FileClose(snmpTrapFileDescriptr);
    snmpTrapFileDescriptr= SYS_FS_HANDLE_INVALID;
    TCPIP_UDP_Flush(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
    TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
    TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
    SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket = INVALID_UDP_SOCKET;
    snmpTrapPutData = NULL;
    return true;
}


#else /* SNMP_STACK_USE_V2_TRAP */

/****************************************************************************
Function:
    bool TCPIP_SNMP_Notify(SNMP_ID var,SNMP_VAL val,SNMP_INDEX index)

Summary:
    Creates and Sends TRAP pdu.

Description:
    This function creates SNMP trap PDU and sends it to previously specified
    remoteHost.
    snmpv1 trap pdu:
   | PDU-type | enterprise | agent-addr | generic-trap | specific-trap |
   | time-stamp | varbind-list |

   The v1 enterprise is mapped directly to SNMPv2TrapOID.0

Precondition:
    TCPIP_SNMP_NotifyIsReady() is already called and returned true.

Parameters:
    var     - SNMP var ID that is to be used in notification
    val     - Value of var. Only value of uint8_t, uint16_t or uint32_t can be sent.
    index   - Index of var. If this var is a single,index would be 0, or else
                      if this var Is a sequence, index could be any value
                      from 0 to 127

Return Values:
    true	-	if SNMP notification was successful sent.
                            This does not guarantee that remoteHost recieved it.
    false	-	Notification sent failed.
                        This would fail under following contions:
                        1) Given SNMP_BIB_FILE does not exist in file system
                        2) Given var does not exist.
                        3) Previously given agentID does not exist
                        4) Data type of given var is unknown -
                           possible if file system itself was corrupted.

Remarks:
    This would fail if there were not UDP socket to open.
***************************************************************************/
bool TCPIP_SNMP_Notify(SNMP_ID var, SNMP_VAL val, SNMP_INDEX index)
{
    char* pCommunity;
    uint8_t len;
    uint8_t OIDValue[SNMP_OID_MAX_LEN];
    uint8_t OIDLen;
    uint8_t agentIDLen;
    uint8_t* pOIDValue;
    uint16_t packetStructLenOffset;
    uint16_t pduStructLenOffset;
    uint16_t varBindStructLenOffset;
    uint16_t varPairStructLenOffset;
    uint16_t prevOffset;
    OID_INFO rec;
    SNMP_DATA_TYPE_INFO dataTypeInfo;

    SNMP_BUFFER_DATA *snmpTrapPutData=NULL;

    if(snmpTrapFileDescriptr== SYS_FS_HANDLE_INVALID)
    {
        snmpTrapFileDescriptr = SYS_FS_FileOpen_Wrapper((const char*)SNMP_BIB_FILE_NAME,SYS_FS_FILE_OPEN_READ);
        if(snmpTrapFileDescriptr == SYS_FS_HANDLE_INVALID)
        {
            TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
            return false;
        }
    }

    if(!TCPIP_SNMP_PDUProcessDuplexInit(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket))
    {
        return false;
    }

    len = SnmpStackDcptMemStubPtr->SNMPNotifyInfo.communityLen;
    pCommunity = SnmpStackDcptMemStubPtr->SNMPNotifyInfo.community;
	
    SnmpStackDcptMemStubPtr->trapPduOutBufData.head = TCPIP_HEAP_Calloc(SnmpStackMemH,1,(size_t)SNMP_MAX_MSG_SIZE);
    if(!SnmpStackDcptMemStubPtr->trapPduOutBufData.head)
    {
        TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
        return false;
    }
    SnmpStackDcptMemStubPtr->trapPduOutBufData.length = 0;
    snmpTrapPutData = &SnmpStackDcptMemStubPtr->trapPduOutBufData;
	
    TCPIP_SNMP_DataCopyToProcessBuffer(STRUCTURE,snmpTrapPutData);            // First item is packet structure
    packetStructLenOffset = snmpTrapPutData->length;
    TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);

    // Put SNMP version info.
    TCPIP_SNMP_DataCopyToProcessBuffer(ASN_INT,snmpTrapPutData);              // Int type.
    TCPIP_SNMP_DataCopyToProcessBuffer(1,snmpTrapPutData);                    // One byte long value.

    //Application has to decide which snmp version has to be
    //updated to the notification pdu.
    TCPIP_SNMP_DataCopyToProcessBuffer(SNMP_V1,snmpTrapPutData);              // v1.
    

    //len = strlen(community);  // Save community length for later use.
    TCPIP_SNMP_DataCopyToProcessBuffer(OCTET_STRING,snmpTrapPutData);         // Octet string type.
    TCPIP_SNMP_DataCopyToProcessBuffer(len,snmpTrapPutData);                  // community string length
    while( len-- )                  // Copy entire string.
        TCPIP_SNMP_DataCopyToProcessBuffer(*(pCommunity++),snmpTrapPutData);

    // Put PDU type.  SNMP agent's response is always GET RESPONSE
    TCPIP_SNMP_DataCopyToProcessBuffer(TRAP,snmpTrapPutData);
    pduStructLenOffset = snmpTrapPutData->length;
    TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);

    // Get complete OID string from file snmp.bib.
    if ( !TCPIP_SNMP_OIDStringFindByID(snmpTrapFileDescriptr,SnmpStackDcptMemStubPtr->SNMPNotifyInfo.agentIDVar,
                           &rec, OIDValue, &agentIDLen) )
    {
        SYS_FS_FileClose(snmpTrapFileDescriptr);
        snmpTrapFileDescriptr= SYS_FS_HANDLE_INVALID;
        TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
        TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
        return false;
    }

    if ( !rec.nodeInfo.Flags.bIsAgentID )
    {
        SYS_FS_FileClose(snmpTrapFileDescriptr);
        snmpTrapFileDescriptr= SYS_FS_HANDLE_INVALID;
        TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
        TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
        return false;
    }

    SYS_FS_FileSeek(snmpTrapFileDescriptr, rec.hData, SYS_FS_SEEK_SET);
    TCPIP_SNMP_DataCopyToProcessBuffer(ASN_OID,snmpTrapPutData);
    SYS_FS_FileRead(snmpTrapFileDescriptr,(uint8_t*)&len,1);
    agentIDLen = len;
    TCPIP_SNMP_DataCopyToProcessBuffer(len,snmpTrapPutData);
    while( len-- )
    {
        uint8_t c;
        SYS_FS_FileRead(snmpTrapFileDescriptr,&c,1);
        TCPIP_SNMP_DataCopyToProcessBuffer(c,snmpTrapPutData);
    }

    
   // pNetIf = TCPIP_UDP_SocketNetGet(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
    // This agent's IP address.
    TCPIP_SNMP_DataCopyToProcessBuffer(SNMP_IP_ADDR,snmpTrapPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(4,snmpTrapPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(((TCPIP_NET_IF*)(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf))->netIPAddr.v[0],snmpTrapPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(((TCPIP_NET_IF*)(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf))->netIPAddr.v[1],snmpTrapPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(((TCPIP_NET_IF*)(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf))->netIPAddr.v[2],snmpTrapPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(((TCPIP_NET_IF*)(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf))->netIPAddr.v[3],snmpTrapPutData);

    // Geberic/Enterprise Trap code
     TCPIP_SNMP_DataCopyToProcessBuffer(ASN_INT,snmpTrapPutData);
     TCPIP_SNMP_DataCopyToProcessBuffer(1,snmpTrapPutData);
     TCPIP_SNMP_DataCopyToProcessBuffer(SnmpStackDcptMemStubPtr->gGenericTrapNotification,snmpTrapPutData);

	// Specific Trap code
    TCPIP_SNMP_DataCopyToProcessBuffer(ASN_INT,snmpTrapPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(1,snmpTrapPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.notificationCode,snmpTrapPutData);

    // Time stamp
    TCPIP_SNMP_DataCopyToProcessBuffer(SNMP_TIME_TICKS,snmpTrapPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(4,snmpTrapPutData);
    TCPIP_UINT32_VAL tStamp;
    tStamp.Val = SnmpStackDcptMemStubPtr->SNMPNotifyInfo.timestamp;

    TCPIP_SNMP_DataCopyToProcessBuffer(tStamp.v[3],snmpTrapPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(tStamp.v[2],snmpTrapPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(tStamp.v[1],snmpTrapPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(tStamp.v[0],snmpTrapPutData);

    // Variable binding structure header
    TCPIP_SNMP_DataCopyToProcessBuffer(0x30,snmpTrapPutData);
    varBindStructLenOffset = snmpTrapPutData->length;
    TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);

    // Create variable name-pair structure
    TCPIP_SNMP_DataCopyToProcessBuffer(0x30,snmpTrapPutData);
    varPairStructLenOffset = snmpTrapPutData->length;
    TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpTrapPutData);
	 
    // Get complete notification variable OID string.
    if ( !TCPIP_SNMP_OIDStringFindByID(snmpTrapFileDescriptr,var, &rec, OIDValue, &OIDLen) )
    {
        SYS_FS_FileClose(snmpTrapFileDescriptr);
        snmpTrapFileDescriptr= SYS_FS_HANDLE_INVALID;
        TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
        TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
        return false;
    }

    // Copy OID string into packet.
    TCPIP_SNMP_DataCopyToProcessBuffer(ASN_OID,snmpTrapPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer((uint8_t)(OIDLen+1),snmpTrapPutData);
    len = OIDLen;
    pOIDValue = OIDValue;
    while( len-- )
        TCPIP_SNMP_DataCopyToProcessBuffer(*pOIDValue++,snmpTrapPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(index,snmpTrapPutData);

    // Encode and Copy actual data bytes
    if ( !TCPIP_SNMP_DataTypeInfoGet(rec.dataType, &dataTypeInfo) )
    {
        SYS_FS_FileClose(snmpTrapFileDescriptr);
        snmpTrapFileDescriptr= SYS_FS_HANDLE_INVALID;
        TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
        TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
        return false;
    }

    TCPIP_SNMP_DataCopyToProcessBuffer(dataTypeInfo.asnType,snmpTrapPutData);

    //Modified to Send trap even for  dataTypeInfo.asnType= ASCII_STRING,
    //where dataTypeInfo.asnLen=0xff
    if ( dataTypeInfo.asnLen == 0xff )
    {
        dataTypeInfo.asnLen=0x4;
        val.dword=0;
    }

    len = dataTypeInfo.asnLen;
    TCPIP_SNMP_DataCopyToProcessBuffer(len,snmpTrapPutData);
    while( len-- )
        TCPIP_SNMP_DataCopyToProcessBuffer(val.v[len],snmpTrapPutData);

    len = dataTypeInfo.asnLen           // data bytes count
         + 1                            // Length byte
         + 1                            // Data type byte
         + OIDLen                       // OID bytes
         + 2                            // OID header bytes
         + 1;                           // index byte

    prevOffset = snmpTrapPutData->length;
    snmpTrapPutData->length = varPairStructLenOffset;
    TCPIP_SNMP_DataCopyToProcessBuffer(len,snmpTrapPutData);

    len += 2;                           // Variable Binding structure header
    snmpTrapPutData->length = varBindStructLenOffset;
    TCPIP_SNMP_DataCopyToProcessBuffer(len,snmpTrapPutData);

    len = len
        + 2                             // Var bind struct header
        + 6                             // 6 bytes of timestamp
        + 3                             // 3 bytes of trap code
        + 3                             // 3 bytes of notification code
        + 6                             // 6 bytes of agnent IP address
        + agentIDLen                    // Agent ID bytes
        + 2;                                // Agent ID header bytes
    snmpTrapPutData->length = pduStructLenOffset;
    TCPIP_SNMP_DataCopyToProcessBuffer(len,snmpTrapPutData);

    len = len                           // PDU struct length
        + 2                             // PDU header
        + SnmpStackDcptMemStubPtr->SNMPNotifyInfo.communityLen            // Community string bytes
        + 2                             // Community header bytes
        + 3;                            // SNMP version bytes
    snmpTrapPutData->length = packetStructLenOffset;
    TCPIP_SNMP_DataCopyToProcessBuffer(len,snmpTrapPutData);

    snmpTrapPutData->length = prevOffset;
// after setting all the offset values, initialize all static variables to 0.
    packetStructLenOffset = 0;
    pduStructLenOffset = 0;
    varBindStructLenOffset = 0;
    varPairStructLenOffset = 0;
    prevOffset = 0;

    TCPIP_SNMP_PutDataArray(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket,snmpTrapPutData);
    SYS_FS_FileClose(snmpTrapFileDescriptr);
    snmpTrapFileDescriptr= SYS_FS_HANDLE_INVALID;
    TCPIP_UDP_Flush(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
    TCPIP_UDP_Close(SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket);
    TCPIP_HEAP_Free(SnmpStackMemH,SnmpStackDcptMemStubPtr->trapPduOutBufData.head);
    return true;
}

#endif 

TCPIP_NET_HANDLE TCPIP_SNMP_ClientGetNet(int *netIx,TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(hNet);
    if(*netIx >= TCPIP_STACK_NumberOfNetworksGet())
    {
        return NULL;
    }

    if(pNetIf == NULL)
    {
        pNetIf = (TCPIP_NET_IF*)TCPIP_STACK_NetDefaultGet();
        *netIx = TCPIP_STACK_NetIxGet(pNetIf);
        return pNetIf;
    }
    for( ; *netIx < TCPIP_STACK_NumberOfNetworksGet(); )
    {
        ++pNetIf;
        *netIx= TCPIP_STACK_NetIxGet(pNetIf);

        if(pNetIf != NULL)
        {
            return pNetIf;
        }
        else
        {
            pNetIf = (TCPIP_NET_IF*)TCPIP_STACK_NetDefaultGet();
        }
    }

    return pNetIf;

}
#endif // Code removed when SNMP_TRAP_DISABLED


/****************************************************************************
Function:
    SNMP_ERR_STATUS TCPIP_SNMP_ProcessPDUHeader(PDU_INFO* pduDbPtr,
                                                      char* community, uint8_t* len)

Summary:
    Validates the received udp packet Snmp header.

Description:
    Collects PDU_INFO (SNMP pdu information database),community name,
    community length and length of data payload.
    This function validates the received udp packet for these different
    variables of snmp pdu. The sequence in which these elements
    are received is important. The validation is done for the agent
    processing capabilities and the max UDP packet length as UDP packets
    can not be fragmented.

Precondition:
    TCPIP_UDP_GetIsReady(SNMPAgentSocket) is called in TCPIP_SNMP_Task(),
    it check if there is any packet on SNMP Agent socket,
should return true.

Parameters:
     pduDbPtr  - Pointer to received pdu information database
     community - Pointer to var storing, community string in rxed pdu
     len	   - Pointer to var storing, community string length rxed in pdu

Return Values:
    SNMP_ACTION - Snmp request pdu type.

Remarks:
    The received pdu will be processed only if this routine returns the
    pdu type else the pdu is discarded as not Snmp pdu.
 ***************************************************************************/
static SNMP_ERR_STATUS TCPIP_SNMP_ProcessPDUHeader(PDU_INFO* pduDbPtr, char* community, uint8_t* len)
{
    uint32_t tempLen;
    SNMP_ACTION pdu=0;
    SNMP_ERR_STATUS snmpErr = SNMP_NO_ERR;
    uint8_t snmpMsgBuf[7];/* 0x30,0x81/0x82/length,0xlen,0xlen,0x02,0x01,0x03(Snmp Version)*/
    uint16_t snmpMsgLen;
    uint16_t retlen,tempCntr=0;
    uint8_t* tempPtr;
    uint8_t* inDataPtr;

    uint8_t intfIdx;

#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
    uint8_t extraMemReqdFor16BytesBlocks;

    SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr;
    SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0;
	
    TCPIP_SNMPV3_PacketProcStubPtrsGet(&snmpv3PktProcessingMemPntr);
    snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;
#endif

    TCPIP_NET_IF* pNetIf = (TCPIP_NET_IF*)TCPIP_STACK_NetDefaultGet();
    intfIdx =  pNetIf->netIfIx;

    tempPtr=snmpMsgBuf;
    // set the readFromSnmpBuf flag to true when we are reading from UDP SNMP socket buffer
    snmpDcpt[intfIdx].readFromSnmpBuf = false;

    //Get complete StructureOF var binds info
    *snmpMsgBuf = TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt);
    tempCntr+=1;
	
    if(!IS_STRUCTURE(*snmpMsgBuf))
     return SNMP_WRONG_TYPE;

	 // Retrieve structure length
    retlen= TCPIP_SNMP_LengthIsValid((uint16_t *)&tempLen);
    if ( !retlen )
        return SNMP_WRONG_LENGTH;

    if(retlen == 1)
    {
        tempCntr+=1;
        *(snmpMsgBuf+1)=tempLen;
    }
    else if( retlen== 2)
    {
        tempCntr+=2;
        *(snmpMsgBuf+1)=0x81; //BER encoding
        *(snmpMsgBuf+2)=tempLen&0xFF;
    }
    else if(retlen== 3)
    {
        tempCntr+=3;
        *(snmpMsgBuf+1)=0x82;//BER encoding
        *(snmpMsgBuf+2)=(tempLen>>8)&0xFF;
        *(snmpMsgBuf+3)=tempLen & 0xFF;
    }
    else
    {
        return SNMP_WRONG_LENGTH;
    }
    snmpMsgLen=tempLen;
	

    //Get snmp version info ASN_INT (1 Byte) + Length (1 Byte)+ snmp Version 1 Byte

     if ( !TCPIP_SNMP_VarDataTypeIsValidInteger(&tempLen) )
        return SNMP_WRONG_TYPE;

    snmpMsgBuf[tempCntr]=ASN_INT;
		
    pduDbPtr->snmpVersion= tempLen;
    if ( (tempLen != (uint8_t)SNMP_V1) && ( tempLen != (uint8_t)SNMP_V2C )&&( tempLen != (uint8_t)SNMP_V3 ))
        return SNMP_WRONG_TYPE;

    snmpMsgBuf[tempCntr+1]=0x01;
    snmpMsgBuf[tempCntr+2]=tempLen&0xFF;

#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
    //Valid snmp packet in the SNMP UDP Socket received
    if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V3)
    {
        //Allocate dynamic memory to store the received SNMPv3 Whole Message.
        snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.wholeMsgLen =(snmpMsgLen+tempCntr);
        extraMemReqdFor16BytesBlocks=16-(snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.wholeMsgLen%16);

        snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.wholeMsgHead=(uint8_t*)(TCPIP_HEAP_Calloc(SnmpStackMemH,1,(size_t)snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.wholeMsgLen+extraMemReqdFor16BytesBlocks+16));

        if(snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.wholeMsgHead == NULL)
        {
            return SNMP_GEN_ERR;
        }

        snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.snmpMsgHead=snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.wholeMsgHead+tempCntr+3/*snmp Version info 0x02,0x01,0x03*/;
        snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.snmpMsgLen=snmpMsgLen-3/*snmp Version info 0x02,0x01,0x03*/;


        //copy the WholeMsg structure info and snmp version info to dynamic mem from the UDP buffer.
        //(WholeMsg is required to authenticate the received snmp pdu )//RFC 3414.
        tempCntr=tempCntr+3/*snmp Version info 0x02,0x01,0x03*/;
        inDataPtr=snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.wholeMsgHead;
        while((tempCntr--)!=0)
        {
            *inDataPtr++=*tempPtr++;
        }

        inDataPtr=snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.snmpMsgHead;
        tempCntr=snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf.snmpMsgLen;

        while(tempCntr--)
        {
            *inDataPtr++=TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt);
        }
		
        snmpErr=TCPIP_SNMPv3_MsgProcessingModelProcessPDU(SNMP_REQUEST_PDU);
        snmpErr=TCPIP_SNMPv3_UserSecurityModelProcessPDU(SNMP_REQUEST_PDU);
        snmpErr=TCPIP_SNMPv3_ScopedPDUProcessing(SNMP_REQUEST_PDU);


        //Complete SNMPv3 data payload (Encrypted or as plain text) is received

         //Check if received SNMPv3 message is Authenticated
        if((snmpv3EngnDcptMemoryStubPtr->SnmpSecurityLevel & 0x01)==0x01)
        {
            //Message is authenticated
            if(SNMPv3AuthenticateRxedPduForDataIntegrity(&snmpv3EngnDcptMemoryStubPtr->InPduWholeMsgBuf)
                    != SNMPV3_MSG_AUTH_PASS)
            return SNMP_AUTH_ERROR;
        }

        //Check if received SNMPv3 message is Encrypted.
        if((snmpv3EngnDcptMemoryStubPtr->SnmpSecurityLevel & 0x02)==0x02)
        {
             //Message is encrypted. Decrypt the message for processing
             //user privacy protocol is AES

            if(SNMPv3AESDecryptRxedScopedPdu() != SNMPV3_MSG_PRIV_PASS)
                    return SNMP_PRIVACY_ERROR;
        }
    }

    else
#endif	
    {
        snmpV1V2cIncomingMsgBufPtr.wholeMsgLen=(snmpMsgLen+tempCntr);
        snmpV1V2cIncomingMsgBufPtr.wholeMsgHead=(uint8_t*)(TCPIP_HEAP_Calloc(SnmpStackMemH,1,(size_t)snmpV1V2cIncomingMsgBufPtr.wholeMsgLen+1));

        if(snmpV1V2cIncomingMsgBufPtr.wholeMsgHead == NULL)
        {
                return SNMP_GEN_ERR;
        }

        snmpV1V2cIncomingMsgBufPtr.snmpMsgHead=snmpV1V2cIncomingMsgBufPtr.wholeMsgHead+tempCntr+3/*snmp Version info 0x02,0x01,0x03*/;
        snmpV1V2cIncomingMsgBufPtr.snmpMsgLen = snmpMsgLen-3/*snmp Version info 0x02,0x01,0x03*/;
		

        //copy the WholeMsg structure info and snmp version info to dynamic mem from the UDP buffer.
                        // To copy the SNMP message payload to the message buffer
        tempCntr=tempCntr+3/*snmp Version info 0x02,0x01,0x<snmpversion>*/;
        inDataPtr=snmpV1V2cIncomingMsgBufPtr.wholeMsgHead;


        while((tempCntr--)!=0)
        {
            *inDataPtr++=*tempPtr++;
        }
		
        inDataPtr=snmpV1V2cIncomingMsgBufPtr.snmpMsgHead;
        tempCntr=snmpV1V2cIncomingMsgBufPtr.snmpMsgLen;

        while(tempCntr--)
        {
            *inDataPtr++=TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt);
        }
		
        // set the readFromSnmpBuf flag to true when we are reading from UDP SNMP socket buffer
        snmpDcpt[intfIdx].readFromSnmpBuf = true;
        SNMPRxOffset = 0;
        if((tempLen == (uint8_t)SNMP_V1)||(tempLen == (uint8_t)SNMP_V2C))
        {

            // This function populates response as it processes community string.
            if ( !TCPIP_SNMP_CommunityStringIsValid(community, len) )
                return SNMP_WRONG_VALUE;

        // Fetch and validate pdu type.
            if ( !TCPIP_SNMP_PDUIsValid(&pdu) )
                return SNMP_WRONG_TYPE;

            pduDbPtr->pduType = pdu;

            //Get_Bulk_Request is not defined in SNMP V1, hence discard udp request packet
            if(pduDbPtr->snmpVersion==(uint8_t)SNMP_V1 && pduDbPtr->pduType == GET_BULK_REQUEST)
                    return SNMP_WRONG_TYPE;

            // Ask main application to verify community name against requested pdu type.
            if(TCPIP_SNMP_IsValidCommunity((uint8_t *)community)==(uint8_t)INVALID_COMMUNITY)
                return SNMP_WRONG_VALUE;
        }
    }
    return snmpErr;
}

/****************************************************************************
Function:
    bool SNMPProcessGetSetHeader(PDU_INFO* pduDbPtr)

Summary:
    Validates the received udp packet Get/Set request header.

Description:
    All the variables of snmp pdu request header are validated for their
    data types. Collects request_id for the snmp request pdu. Fetch,validates
    error status,error index and discard as they are need not to be processed
    as received in request pdu. Collects non repeaters and max repeaters
    values in case of Get_Bulk request.

Precondition:
    TCPIP_SNMP_ProcessPDUHeader() is called and returns pdu type and do not returns
    SNMP_ACTION_UNKNOWN

Parameters:
    pduDbPtr  - Pointer to received pdu information database.

Return Values:
    true  - If the received request header is validated and passed.
    false - If rxed request header is not valid.

Remarks:
    The request pdu will be processed only if this routine returns true
 ***************************************************************************/
static bool TCPIP_SNMP_ProcessHeaderGetSet(PDU_INFO* pduDbPtr)
{
    uint32_t tempData;

    // Fetch and save request ID.
    if ( TCPIP_SNMP_VarDataTypeIsValidInteger(&tempData) )
         pduDbPtr->requestID = tempData;
    else
        return false;

    if((pduDbPtr->snmpVersion == (uint8_t)SNMP_V1 || pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C /*|| pduDbPtr->snmpVersion == (uint8_t)SNMP_V3*/) &&(pduDbPtr->pduType != GET_BULK_REQUEST))
    {
        // Fetch and discard error status
        if ( !TCPIP_SNMP_VarDataTypeIsValidInteger(&tempData) )
            return false;

        // Fetch and disacard error index
        return TCPIP_SNMP_VarDataTypeIsValidInteger(&tempData);
    }
    else if((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C /*|| pduDbPtr->snmpVersion == (uint8_t)SNMP_V3*/ )&& pduDbPtr->pduType == GET_BULK_REQUEST )
    {
        // Fetch non-repeaters value
        if ( TCPIP_SNMP_VarDataTypeIsValidInteger(&tempData) )
             pduDbPtr->nonRepeators=tempData&0xFF;
        else
            return false;

        // Fetch Max-repetitions value
        if(TCPIP_SNMP_VarDataTypeIsValidInteger(&tempData))
             pduDbPtr->maxRepetitions=(uint8_t)tempData&0xFF;
        else
            return false;
    }
    else
        return false;
	
    return true;
}


/****************************************************************************
Function:
    bool TCPIP_SNMP_ProcessVariables(PDU_INFO* pduDbPtr,uint8_t* community, uint8_t len)

Summary:
    This routine processes the snmp request and parallely creates the
    response pdu.

Description:
    Once the received pdu is validated as Snmp pdu, it is forwarded for
    processing to this routine. This rotuine handles Get, Get_Next, Get_Bulk,
    Set request and creates appropriate response as Get_Response.
    This routine will decide on whether the request pdu should be processed
    or be discarded.

Precondition:
    The received udp packet is varified as SNMP request.
    TCPIP_SNMP_ProcessPDUHeader() and SNMPProcessGetSetHeader() returns but false.

Parameters:
    pduDbPtr  - Pointer to received pdu information database
community - Pointer to var, storing community string in rxed pdu
    len	   	  - Pointer to var, storing community string length rxed in pdu

Return Values:
    true 	- If the snmp request processing is successful.
    false	- If the processing failed else the processing is not completed.

Remarks:
    None
 ***************************************************************************/
static bool TCPIP_SNMP_ProcessVariables(PDU_INFO* pduDbPtr,char* community, uint8_t len)
{	
    uint8_t getbulkOverFlowFlag = false;
    uint8_t temp =0;
    uint8_t OIDValue[SNMP_OID_MAX_LEN];
    uint8_t OIDLen=0;
    uint8_t varIndex =0;
    uint8_t errorIndex;
    uint8_t communityLen=0,commRetVal=0;
    uint8_t noOfOIDsInReq=0,tempNonRepeators=0,noOfVarToBeInResponse=0;
    uint8_t repeatCntr,varBindCntr;
    uint8_t Getbulk_N=0,Getbulk_M=0,Getbulk_R=0;/*Refer RFC 3416 Section "4.2.3" GetBulkRequest-PDU*/
    uint8_t oidLookUpRet=0;
    uint8_t templen=0;
    uint8_t successor=0;// 'I'th lexicographic successor
    uint8_t *ptemp;
    uint8_t *ptroid;
    uint8_t *rxedCommunityName;
    uint16_t varBindStructOffset=0;
    uint16_t tempTxOffset=0;
    uint16_t oidOffset=0;
    uint16_t prevOffset=0;
    uint16_t packetStructLenOffset=0;
    //uint16_t snmpV3ScopedPduOffset=0;
    uint16_t pduLenOffset=0;
    uint16_t errorStatusOffset=0;
    uint16_t errorIndexOffset=0;    
    uint16_t varStructLenOffset=0;	
    uint16_t prevSnmpRxOffset=0;
    TCPIP_UINT16_VAL varBindingLen={0};
    TCPIP_UINT16_VAL tempLen={0};
    TCPIP_UINT16_VAL varPairLen={0};
    static TCPIP_UINT16_VAL varBindLen={0};
    OID_INFO OIDInfo;
    SNMP_ERR_STATUS errorStatus;
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
    uint8_t agentIDLen;
    OID_INFO rec;
    TCPIP_UINT16_VAL tempByteCntr;
#endif
    bool bSnmpGenError = false;
    
    bool bSnmpSendPkt = false;
    TCPIP_UINT16_VAL bytesAdded2Pdu;
    uint8_t intfIdx;
    SNMP_BUFFER_DATA *snmpPutData=NULL;
    static enum
    {
        SM_PKT_STRUCT_LEN_OFFSET=0u,
        SM_RESPONSE_PDU_LEN_OFFSET,
        SM_ERROR_STATUS_OFFSET,
        SM_ERROR_INDEX_OFFSET,
        SM_FIND_NO_OF_REQUESTED_VARBINDS,
        SM_FIND_NO_OF_RESPONSE_VARBINDS,
        SM_VARBIND_STRUCT_OFFSET,
        SM_VARSTRUCT_LEN_OFFSET,
        SM_POPULATE_REQ_OID,
        SM_FIND_OID_IN_MIB,
        SM_NON_REPETITIONS,
        SM_MAX_REPETITIONS
    }smSnmp=SM_PKT_STRUCT_LEN_OFFSET;

    snmpReqVarErrStatus.noSuchInstanceErr=0x0000;
    snmpReqVarErrStatus.noSuchNameErr=0x0000;
    snmpReqVarErrStatus.noSuchObjectErr=0x0000;
    snmpReqVarErrStatus.endOfMibViewErr=0x0000;

    rxedCommunityName=(uint8_t *)community;
    /* Locate the start offset of the TX PDU */
    //tempTxOffset = _SNMPGetTxOffset();
    tempTxOffset = SNMPTxOffset;
    OIDLen = 0;
    varBindLen.Val=0x0000;
    SnmpStackDcptMemStubPtr->getZeroInstance = false;
    bytesAdded2Pdu.Val=0x00;

    intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_NetDefaultGet())->netIfIx;

    while(1)
    {
        switch(smSnmp)
        {

            // Before each variables are processed, prepare necessary header.

            case SM_PKT_STRUCT_LEN_OFFSET:
                snmpPutData = &SnmpStackDcptMemStubPtr->outPduBufData;

                varPairLen.Val=0x0000;

                TCPIP_SNMP_DataCopyToProcessBuffer(STRUCTURE,snmpPutData);  // first item in snmp packet
                TCPIP_SNMP_DataCopyToProcessBuffer(0x82,snmpPutData);

                // Since we do not know length of structure at this point, use
                // placeholder bytes that will be replaced with actual value.

                packetStructLenOffset = snmpPutData->length;
                TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpPutData);
                TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpPutData);

                // Put SNMP version info - only v1.0 is supported.
                TCPIP_SNMP_DataCopyToProcessBuffer(ASN_INT,snmpPutData);              // Int type.
                TCPIP_SNMP_DataCopyToProcessBuffer(1,snmpPutData);                    // One byte long value.
                TCPIP_SNMP_DataCopyToProcessBuffer(pduDbPtr->snmpVersion,snmpPutData);              // v1.0.

                // Put community string
                communityLen = len;             // Save community length for later use.
                TCPIP_SNMP_DataCopyToProcessBuffer(OCTET_STRING,snmpPutData);         // Octet string type.
                TCPIP_SNMP_DataCopyToProcessBuffer(len,snmpPutData);                  // community string length
                while( len-- )                  // Copy entire string.
                        TCPIP_SNMP_DataCopyToProcessBuffer(*community++,snmpPutData);

                smSnmp++;

			//return false;
            case SM_RESPONSE_PDU_LEN_OFFSET:

                // Put PDU type.  SNMP agent's response is always GET RESPONSE
                TCPIP_SNMP_DataCopyToProcessBuffer(GET_RESPONSE,snmpPutData);

                // Since we don't know length of this response, use placeholders until
                // we know for sure...
                TCPIP_SNMP_DataCopyToProcessBuffer(0x82,snmpPutData);
                pduLenOffset = snmpPutData->length;
                TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpPutData); // Be prepared for 2 byte-long length
                TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpPutData);

                // Put original request back.
                TCPIP_SNMP_DataCopyToProcessBuffer(ASN_INT,snmpPutData);	// Int type.
                TCPIP_SNMP_DataCopyToProcessBuffer(4,snmpPutData);		// To simplify logic, always use 4 byte long requestID
                TCPIP_SNMP_DataCopyToProcessBuffer((pduDbPtr->requestID>>24)&0xFF,snmpPutData); // Start MSB
                TCPIP_SNMP_DataCopyToProcessBuffer((pduDbPtr->requestID>>16)&0xFF,snmpPutData);
                TCPIP_SNMP_DataCopyToProcessBuffer((pduDbPtr->requestID>>8)&0xFF,snmpPutData);
                TCPIP_SNMP_DataCopyToProcessBuffer(pduDbPtr->requestID&0xFF,snmpPutData);

                smSnmp++;

                //return false;
			
            case SM_ERROR_STATUS_OFFSET :

                // Put error status.
                // Since we do not know error status, put place holder until we know it...
                TCPIP_SNMP_DataCopyToProcessBuffer(ASN_INT,snmpPutData);              // Int type
                TCPIP_SNMP_DataCopyToProcessBuffer(1,snmpPutData);                    // One byte long.
                errorStatusOffset = snmpPutData->length;
                TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpPutData);                    // Placeholder.
                smSnmp++;

            case SM_ERROR_INDEX_OFFSET :

                // Similarly put error index.
                TCPIP_SNMP_DataCopyToProcessBuffer(ASN_INT,snmpPutData);              // Int type
                TCPIP_SNMP_DataCopyToProcessBuffer(1,snmpPutData);                    // One byte long
                errorIndexOffset = snmpPutData->length;
                TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpPutData);                    // Placeholder.

                varIndex    = 0;
                errorIndex  = 0;
                errorStatus = SNMP_NO_ERR;

                smSnmp++;

            case SM_FIND_NO_OF_REQUESTED_VARBINDS:

                // Decode variable binding structure
                if ( !TCPIP_SNMP_SNMPStructureIsValid(&varBindingLen.Val) )
                return false;

                //Find number of OIDs/varbinds's data requested in received PDU.
                noOfOIDsInReq=TCPIP_SNMP_OIDsCountGet(varBindingLen.Val);

                smSnmp++;

                //return false;

            case SM_FIND_NO_OF_RESPONSE_VARBINDS:

                //Calulate number of variables to be responded for the received request
                Getbulk_N = noOfOIDsInReq; Getbulk_M=0; Getbulk_R=0;
                if(((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C)||
                        ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V3))) &&
                        (pduDbPtr->pduType == GET_BULK_REQUEST))
                {
                    if((pduDbPtr->nonRepeators) <= noOfOIDsInReq)
                    {
                        Getbulk_N = pduDbPtr->nonRepeators;
                    }

                    Getbulk_M = pduDbPtr->maxRepetitions;

                    if((noOfOIDsInReq - Getbulk_N)>=0u)
                        Getbulk_R = noOfOIDsInReq-Getbulk_N;
                }

                tempNonRepeators = Getbulk_N;

                noOfVarToBeInResponse = Getbulk_N + (Getbulk_M * Getbulk_R);//Refer RFC 3416

                smSnmp++;

                //return false;

            case SM_VARBIND_STRUCT_OFFSET:

                // Put variable binding response structure
                TCPIP_SNMP_DataCopyToProcessBuffer(STRUCTURE,snmpPutData);
                TCPIP_SNMP_DataCopyToProcessBuffer(0x82,snmpPutData);

                // Since we do not know data payload length, put place holder until we know it...
                varBindStructOffset = snmpPutData->length;
                TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpPutData);
                TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpPutData);

                varBindLen.Val = 0;

                smSnmp++;

                    //return false;
			
            case SM_VARSTRUCT_LEN_OFFSET:

                /*	If the getbulk request is received with zero non-repeaters, process
                        variable State Machine jumps to SM_MAX_REPETITIONS. Modify the Rx
                        and Tx offset accordigly. */
                if(Getbulk_N==0u)
                {

                #ifdef TCPIP_STACK_USE_SNMPV3_SERVER
                    if((pduDbPtr->snmpVersion == (uint8_t)SNMP_V3))
                    {
                            // Get complete OID string from snmp.bib file.
                        TCPIP_SNMP_OIDStringFindByID(snmpFileDescrptr,MICROCHIP,&rec, OIDValue, &agentIDLen);

                        if ( rec.nodeInfo.Flags.bIsAgentID )
                        {
                            SYS_FS_FileSeek(snmpFileDescrptr, rec.hData, SYS_FS_SEEK_SET);
                        }
                        TCPIP_SNMP_DataCopyToProcessBuffer(ASN_OID,snmpPutData);

                        SYS_FS_FileRead(snmpFileDescrptr,&len,1);

                        agentIDLen = len;
                        TCPIP_SNMP_DataCopyToProcessBuffer(len,snmpPutData);
                        while( len-- )
                        {
                            uint8_t c;

                            SYS_FS_FileRead(snmpFileDescrptr,&c,1);

                            TCPIP_SNMP_DataCopyToProcessBuffer(c,snmpPutData);
                        }

                    }
                    else
                    #endif
                    {
                        prevSnmpRxOffset=SNMPRxOffset;
                        smSnmp=SM_MAX_REPETITIONS;
                        varStructLenOffset = snmpPutData->length;
                        snmpPutData->length=snmpPutData->length+4;
                        break;
                    }
                }
			
/*
    Need to know what variable we are processing, so that in case
    if there is problem for that variable, we can put it in
    errorIndex location of SNMP packet.
*/
                varIndex++;

                // Decode variable length structure
                temp = TCPIP_SNMP_SNMPStructureIsValid(&tempLen.Val);
                if ( !temp )
                {
                    TCPIP_SNMP_ErrorStatusSet(errorStatusOffset,errorIndexOffset,SNMP_GEN_ERR,varIndex,snmpPutData);
                    bSnmpGenError = true;
                    break;
                }

                varBindingLen.Val -= tempLen.Val;
                varBindingLen.Val -= temp;

                varStructLenOffset = snmpPutData->length;

                if(pduDbPtr->pduType == GET_BULK_REQUEST )
                {
                    snmpPutData->length=snmpPutData->length+4;
                }
                smSnmp++;
                //return false;

            case SM_POPULATE_REQ_OID:

            /* 	Populate received pdu for the requested OIDs and also create the
                response pdu on the go.*/
                // Decode next object
                if ( !TCPIP_SNMP_OIDIsValid(OIDValue, &OIDLen) )
                {
                    TCPIP_SNMP_ErrorStatusSet(errorStatusOffset,errorIndexOffset,SNMP_GEN_ERR,varIndex,snmpPutData);
                    bSnmpGenError = true;
                    break;
                }
                // For Get & Get-Next, value must be NULL.
                if ( pduDbPtr->pduType != (uint8_t)SET_REQUEST )
                {
                    if ( !TCPIP_SNMP_DataIsASNNull() )
                    return false;
                }
                if(pduDbPtr->pduType != GET_BULK_REQUEST )
                {
                    // Prepare response - original variable
                    TCPIP_SNMP_DataCopyToProcessBuffer(ASN_OID,snmpPutData);
                    oidOffset = snmpPutData->length;
                    TCPIP_SNMP_DataCopyToProcessBuffer(OIDLen,snmpPutData);
                    ptemp = OIDValue;
                    temp = OIDLen;
                    while( temp-- )
                    TCPIP_SNMP_DataCopyToProcessBuffer(*ptemp++,snmpPutData);
                }

                /*
                   Match "rxedCommunityName" to "readCommunity" to authorize access
                   to private MIB objects.
                   As we start supporting the secured encrypted community transaction,
                   rxed community string can be an encrypted string which the agent
                   need to decrypt and validate to autohrize access.
                   The agent should respond with encrypted community name.
                */
                if((pduDbPtr->snmpVersion != (uint8_t)SNMP_V3))
                {
                    commRetVal=TCPIP_SNMP_IsValidCommunity(rxedCommunityName);

                    smSnmp=SM_PKT_STRUCT_LEN_OFFSET;	// Start out assuming commRetVal == INVALID_COMMUNITY
                    if(pduDbPtr->pduType == (uint8_t)SET_REQUEST)
                    {
                        if(commRetVal==(uint8_t)WRITE_COMMUNITY)//If SET request, then "community==WRITE_COMMUNITY" is must.
                        {
                                smSnmp=SM_FIND_OID_IN_MIB;
                        }
                    }
                    else
                    {
                        if(commRetVal!=(uint8_t)INVALID_COMMUNITY)//If any GET request, then "community!=INVALID_COMMUNITY" is must (community is WRITE_COMMUNITY or READ_COMMUNITY).
                        {
                                smSnmp=SM_FIND_OID_IN_MIB;
                        }
                    }
                }
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
                else
                {
                    smSnmp=SM_FIND_OID_IN_MIB;
                }
#endif
                //Verify if trying to access the private object
                //Application has to decide on what community name should allowed to
                //read write the private mib objects.

                if(TCPIP_SNMP_PvtMIBObjIsRequested(OIDValue) && (smSnmp==SM_PKT_STRUCT_LEN_OFFSET) )
                {
                    //If private mib object is requested and community do not match,
                    //generate authentication failure TRAP

                    Getbulk_N=0;
                    noOfVarToBeInResponse=0;
                    smSnmp=SM_PKT_STRUCT_LEN_OFFSET;

                    //Searching the requested OID in the MIB database
                    oidLookUpRet = TCPIP_SNMP_OIDFindInMgmtInfoBase(snmpFileDescrptr,pduDbPtr,OIDValue, OIDLen, &OIDInfo);
                    SnmpStackDcptMemStubPtr->gOIDCorrespondingSnmpMibID=OIDInfo.id;

                    //_SNMPSetTxOffset(packetStructLenOffset-2);
                    SNMPTxOffset=packetStructLenOffset-2;
                    SnmpStackDcptMemStubPtr->gSpecificTrapNotification=VENDOR_TRAP_DEFAULT;
                    SnmpStackDcptMemStubPtr->gGenericTrapNotification=AUTH_FAILURE;
                    SnmpStackDcptMemStubPtr->gSendTrapFlag=true;
                }
                /*else
                        smSnmp++;*/

                if(smSnmp==SM_PKT_STRUCT_LEN_OFFSET || smSnmp==SM_VARSTRUCT_LEN_OFFSET)
                        break;

                    //return false;
			
            case SM_FIND_OID_IN_MIB:

                /* Search for the requested OID in the MIB database with the agent.*/

                if(Getbulk_N!= 0u)
                        Getbulk_N--;

                if(Getbulk_N==0u)
                        prevSnmpRxOffset=SNMPRxOffset;

                noOfVarToBeInResponse--;

                //Searching the requested OID in the MIB database
                oidLookUpRet = TCPIP_SNMP_OIDFindInMgmtInfoBase(snmpFileDescrptr,pduDbPtr,OIDValue, OIDLen, &OIDInfo);

                if(oidLookUpRet != (uint8_t)true && (pduDbPtr->pduType != GET_NEXT_REQUEST) &&
                        (pduDbPtr->pduType != GET_BULK_REQUEST))
                {
                    snmpPutData->length = varStructLenOffset;

                    // Put corresponding variable response structure
                    TCPIP_SNMP_DataCopyToProcessBuffer(STRUCTURE,snmpPutData);
                    TCPIP_SNMP_DataCopyToProcessBuffer(0x82,snmpPutData);

                    varStructLenOffset= snmpPutData->length;
                    TCPIP_SNMP_DataCopyToProcessBuffer(0x00,snmpPutData);//Place holder
                    TCPIP_SNMP_DataCopyToProcessBuffer(0x00,snmpPutData);

                    // ASN OID data type
                    templen=OIDLen;
                    ptroid=OIDValue;
                    TCPIP_SNMP_DataCopyToProcessBuffer(ASN_OID,snmpPutData);

                    if(SnmpStackDcptMemStubPtr->appendZeroToOID)
                        TCPIP_SNMP_DataCopyToProcessBuffer(OIDLen+1,snmpPutData);//for appending "0"
                    else
                        TCPIP_SNMP_DataCopyToProcessBuffer(OIDLen,snmpPutData);//do not append "0"

                    //Put OID
                    while( templen-- )
                    TCPIP_SNMP_DataCopyToProcessBuffer(*ptroid++,snmpPutData);

                    if(SnmpStackDcptMemStubPtr->appendZeroToOID)
                    {
                        TCPIP_SNMP_DataCopyToProcessBuffer(0x00,snmpPutData);//Appending '0' to OID in response
                        varPairLen.Val += OIDLen+1+2; //Modify the response length
                    }
                    else
                        varPairLen.Val += OIDLen+2;

                    //update and send the error status and the error index.
                    if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
                    {
                        errorStatus = SNMP_NO_SUCH_NAME;
                        TCPIP_SNMP_ErrorStatusSet(errorStatusOffset,errorIndexOffset,SNMP_NO_SUCH_NAME,varIndex,snmpPutData);
                        TCPIP_SNMP_DataCopyToProcessBuffer(ASN_NULL,snmpPutData);
                        TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpPutData);
                    }
                    else if(((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C)
                                    ||( pduDbPtr->snmpVersion == (uint8_t)SNMP_V3))
                                    && pduDbPtr->pduType != SET_REQUEST)
                    {
                        if(pduDbPtr->pduType == SNMP_GET)
                        {
                            TCPIP_SNMP_DataCopyToProcessBuffer(oidLookUpRet,snmpPutData);
                            TCPIP_SNMP_DataCopyToProcessBuffer(0x00,snmpPutData);
                            if(oidLookUpRet == SNMP_NO_SUCH_OBJ)
                            {
                                snmpReqVarErrStatus.noSuchObjectErr|=(0x0001 << varIndex);
                            }
                            else if(oidLookUpRet == SNMP_NO_SUCH_INSTANCE)
                            {
                                snmpReqVarErrStatus.noSuchInstanceErr|=(0x0001 << varIndex);
                            }
                        }
                    }

                    if(pduDbPtr->snmpVersion !=SNMP_V3)
                    varPairLen.Val +=2 ;

                    varBindLen.Val += 4	// Variable Pair STRUCTURE byte + 1 length byte.
                    + varPairLen.Val;

                    //Now update the place holder for var pair length
                    prevOffset = snmpPutData->length;
                    snmpPutData->length = varStructLenOffset;

                    TCPIP_SNMP_DataCopyToProcessBuffer(varPairLen.v[1],snmpPutData);
                    TCPIP_SNMP_DataCopyToProcessBuffer(varPairLen.v[0],snmpPutData);

                    snmpPutData->length = prevOffset;
                    varPairLen.Val=0x00;

                    //Reset to state machine to access the next oid in request
                    smSnmp=SM_VARSTRUCT_LEN_OFFSET;
                    break;
                }
                smSnmp++;

                    //return false;

            case SM_NON_REPETITIONS:

                /* 	Variables in get,get_next,set and get_bulk ( non repetition variables)
                        of snmp request are processed in this part of the state machine.*/

                //Save SnmpTxOffsetfor future uses.
                prevOffset = snmpPutData->length;
                snmpPutData->length = varStructLenOffset;

                //Start response variable binding with ASN STRUCTURE type.
                TCPIP_SNMP_DataCopyToProcessBuffer(STRUCTURE,snmpPutData);
                TCPIP_SNMP_DataCopyToProcessBuffer(0x82,snmpPutData);

                varStructLenOffset= snmpPutData->length;
                TCPIP_SNMP_DataCopyToProcessBuffer(0x00,snmpPutData); //place holder
                TCPIP_SNMP_DataCopyToProcessBuffer(0x00,snmpPutData);

                TCPIP_SNMP_DataCopyToProcessBuffer(ASN_OID,snmpPutData);

                if(pduDbPtr->pduType == SNMP_SET)
                {
                    templen=OIDLen;
                    ptroid=OIDValue;
                    //to validate the REC ID is present or not

                    if(TCPIP_SNMP_RecordIDValidation(pduDbPtr->snmpVersion,OIDInfo.nodeInfo.Flags.bIsIDPresent,OIDInfo.id,OIDValue,OIDLen) != true)
                    {

                             /*if the variable binding's name specifies a
                         * variable which does not exist and could not ever be
                         * created, then the value of the Response-PDU's error-
                         * status field is set to `noCreation', and the value of its
                         * error-index field is set to the index of the failed
                         * variable binding.
                         */
                        errorStatus = SNMP_NO_CREATION;
                        smSnmp=SM_PKT_STRUCT_LEN_OFFSET;
                        return false;
                    }

                    if(SnmpStackDcptMemStubPtr->appendZeroToOID)
                            TCPIP_SNMP_DataCopyToProcessBuffer(OIDLen+1,snmpPutData);//for appending "0"
                    else
                            TCPIP_SNMP_DataCopyToProcessBuffer(OIDLen,snmpPutData);//do not append "0"

                    //Put OID
                    while( templen-- )
                    TCPIP_SNMP_DataCopyToProcessBuffer(*ptroid++,snmpPutData);

                    if(SnmpStackDcptMemStubPtr->appendZeroToOID)
                        TCPIP_SNMP_DataCopyToProcessBuffer(0x00,snmpPutData);//Appending '0' to OID in response

				//Now process the SET command
                    temp = TCPIP_SNMP_ProcessSetVar(pduDbPtr,&OIDInfo, &errorStatus);

                    if ( errorStatus != SNMP_NO_ERR )
                    {
                        //SET var command failed. Update the error status.
                        TCPIP_SNMP_ErrorStatusSet(errorStatusOffset,
                        errorIndexOffset,
                        errorStatus,
                        varIndex,snmpPutData);
                    }

                    if(SnmpStackDcptMemStubPtr->appendZeroToOID)
                            varPairLen.Val = OIDLen+1 +2   // OID name + header bytes
                    + temp;            // value bytes as put by SetVar
                    else
                            varPairLen.Val = OIDLen+2+temp;

                }
                else if((pduDbPtr->pduType == SNMP_GET)  ||
                        ((pduDbPtr->pduType == SNMP_V2C_GET_BULK) && (oidLookUpRet == true)))
                {
                    //to validate the REC ID is present or not
                    if(TCPIP_SNMP_RecordIDValidation(pduDbPtr->snmpVersion,OIDInfo.nodeInfo.Flags.bIsIDPresent,OIDInfo.id,OIDValue,OIDLen) != true)
                    {
                        smSnmp=SM_PKT_STRUCT_LEN_OFFSET;
                        return false;
                    }
                    templen=OIDLen;
                    ptroid=OIDValue;

                    if(SnmpStackDcptMemStubPtr->appendZeroToOID)
                            TCPIP_SNMP_DataCopyToProcessBuffer(OIDLen+1,snmpPutData);//for appending "0"
                    else
                            TCPIP_SNMP_DataCopyToProcessBuffer(OIDLen,snmpPutData);//do not append "0"

                    //Put OID
                    while( templen-- )
                            TCPIP_SNMP_DataCopyToProcessBuffer(*ptroid++,snmpPutData);

                    if(SnmpStackDcptMemStubPtr->appendZeroToOID)
                    {
                            TCPIP_SNMP_DataCopyToProcessBuffer(0x00,snmpPutData);//Appending '0' to OID in response
                            varPairLen.Val = OIDLen + 2+1;
                    }
                    else
                            varPairLen.Val = OIDLen +2;

                    //Now process the GET command
                    temp=TCPIP_SNMP_ProcessGetVar(&OIDInfo,false,pduDbPtr);

                }
                else if((pduDbPtr->pduType == SNMP_GET_NEXT)||
                        ((pduDbPtr->pduType == SNMP_V2C_GET_BULK) && (oidLookUpRet != true)))
                {
                    temp=TCPIP_SNMP_ProcessGetNextVar(&OIDInfo,pduDbPtr);

                    //If Get Next command failed
                    if(temp==0u)
                    {
                        templen=OIDLen;
                        ptroid=OIDValue;

                        if(SnmpStackDcptMemStubPtr->appendZeroToOID)
                                TCPIP_SNMP_DataCopyToProcessBuffer(OIDLen+1,snmpPutData);//for appending "0"
                        else
                                TCPIP_SNMP_DataCopyToProcessBuffer(OIDLen,snmpPutData);//do not append "0"

                        //Put OID
                        while( templen-- )
                                TCPIP_SNMP_DataCopyToProcessBuffer(*ptroid++,snmpPutData);

                        if(SnmpStackDcptMemStubPtr->appendZeroToOID)
                                TCPIP_SNMP_DataCopyToProcessBuffer(0x00,snmpPutData);//Appending '0' to OID in response
                    }
                }


                /*  If the request command processing is failed, update
                        the error status, index accordingly and response pdu.*/
                if(temp == 0u &&(pduDbPtr->pduType != SNMP_SET))
                {
                    if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
                    {
                        errorStatus = SNMP_NO_SUCH_NAME;
                        TCPIP_SNMP_ErrorStatusSet(errorStatusOffset,errorIndexOffset,SNMP_NO_SUCH_NAME,
                                               varIndex,snmpPutData);
                    }

                    TCPIP_SNMP_DataCopyToProcessBuffer(ASN_NULL,snmpPutData);
                    TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpPutData);

                    if((pduDbPtr->pduType == SNMP_GET_NEXT|| pduDbPtr->pduType == SNMP_V2C_GET_BULK)&&pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C)
                    {
                        snmpPutData->length=snmpPutData->length-2;
                        TCPIP_SNMP_DataCopyToProcessBuffer(SNMP_END_OF_MIB_VIEW,snmpPutData);
                        TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpPutData);
                    }

                    if((pduDbPtr->pduType == SNMP_GET) ||
                            ((pduDbPtr->pduType == SNMP_V2C_GET_BULK) && (oidLookUpRet == true)))
                    {
                            temp = 2;
                    }
                    else if((pduDbPtr->pduType == SNMP_GET_NEXT)||
                            ((pduDbPtr->pduType == SNMP_V2C_GET_BULK) && (oidLookUpRet != true)))
                    {
                         varPairLen.Val = OIDLen+1          // as put by GetNextVar()
                         + 2                // OID header
                         + 2;               // END_OF_MIB_VIEW bytes
                    }


                    /* 	Applications can make use of the below information
                            to find the error status for the given variable and to
                            build the logic arround. */
                    snmpReqVarErrStatus.noSuchNameErr	 |=(0x0001 << varIndex);
                    snmpReqVarErrStatus.noSuchObjectErr	 |=(0x0001 << varIndex);
                    snmpReqVarErrStatus.noSuchInstanceErr|=(0x0001 << varIndex);
                    snmpReqVarErrStatus.endOfMibViewErr	 |=(0x0001 << varIndex);

                }
                else if((pduDbPtr->pduType == SNMP_GET_NEXT)||
                        ((pduDbPtr->pduType == SNMP_V2C_GET_BULK) && (oidLookUpRet != true)))
                {
                    if(SnmpStackDcptMemStubPtr->getZeroInstance)
                        varPairLen.Val += temp+2;
                    else
                        varPairLen.Val = (temp + 2);
                }

                if((pduDbPtr->pduType == SNMP_GET) ||
                        ((pduDbPtr->pduType == SNMP_V2C_GET_BULK) && (oidLookUpRet == true)))
                        varPairLen.Val += temp;

                varBindLen.Val += 4	// Variable Pair STRUCTURE byte + 1 length byte.
                + varPairLen.Val;

                //Update place holder
                prevOffset =  snmpPutData->length;
                snmpPutData->length = varStructLenOffset;
                TCPIP_SNMP_DataCopyToProcessBuffer(varPairLen.v[1],snmpPutData);
                TCPIP_SNMP_DataCopyToProcessBuffer(varPairLen.v[0],snmpPutData);

                snmpPutData->length = prevOffset;
                varStructLenOffset = snmpPutData->length;

                /* 	Decide on the number of Non repetition variables remained to
                        be processed, decide the course of state machine.*/

                if((pduDbPtr->pduType==GET_BULK_REQUEST) &&
                   ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C)||
                    (pduDbPtr->snmpVersion == (uint8_t)SNMP_V3))&&( Getbulk_N == 0u))
                {
                    if((varStructLenOffset - tempTxOffset) >= SNMP_MAX_MSG_SIZE)
                    {
                        getbulkOverFlowFlag = true;
                        break;
                    }
                    else
                    {
                        smSnmp=SM_MAX_REPETITIONS;
                    }
                }
                else
                    smSnmp=SM_VARSTRUCT_LEN_OFFSET;
                varPairLen.Val=0x00;
                SnmpStackDcptMemStubPtr->getZeroInstance = false;

                /* check length*/
                break;

                    //return false;

            case SM_MAX_REPETITIONS:

                /*Process each variable in request as Get_Next for
                  Getbulk_M (Max_repetition) times */
                for(repeatCntr=0;repeatCntr<Getbulk_M;repeatCntr++)
                {
                    SNMPRxOffset=prevSnmpRxOffset;

                    //Process every veriable in the request.
                    for(varBindCntr=0;varBindCntr<Getbulk_R;varBindCntr++)
                    {
                        if(varBindCntr==0u)
                        varIndex=(noOfOIDsInReq-Getbulk_R);
                        varIndex++;
                        if((snmpReqVarErrStatus.endOfMibViewErr >> (tempNonRepeators+varBindCntr+1))&0x0001)
                        {
                            noOfVarToBeInResponse--;
                            temp = TCPIP_SNMP_SNMPStructureIsValid(&tempLen.Val);

                            if(varBindCntr!=Getbulk_R)
                            {
                                SNMPRxOffset=SNMPRxOffset+tempLen.Val;//2+OIDLen+2;
                            }
                            continue;
                        }

                        if(noOfVarToBeInResponse != 0)
                        {
                            noOfVarToBeInResponse--;
                        }
                        varPairLen.Val = 0;
                        prevOffset = snmpPutData->length;
                        snmpPutData->length = varStructLenOffset;
                        if(TCPIP_SNMP_DataCopyToProcessBuffer(STRUCTURE,snmpPutData)!= true)
                        {
                            getbulkOverFlowFlag = true;
                            break;
                        }
                        if(TCPIP_SNMP_DataCopyToProcessBuffer(0x82,snmpPutData)!= true)
                        {
                            getbulkOverFlowFlag = true;
                            break;
                        }
                        varStructLenOffset= snmpPutData->length;
                        if(TCPIP_SNMP_DataCopyToProcessBuffer(0x00,snmpPutData)!= true)
                        {
                            getbulkOverFlowFlag = true;
                            break;
                        }
                        if(TCPIP_SNMP_DataCopyToProcessBuffer(0x00,snmpPutData)!= true)
                        {
                            getbulkOverFlowFlag = true;
                            break;
                        }
                        successor=repeatCntr;
                        // Decode variable length structure
                        temp = TCPIP_SNMP_SNMPStructureIsValid(&tempLen.Val);
                        if ( !temp )
                                break;

                        // Decode next object
                        if ( !TCPIP_SNMP_OIDIsValid(OIDValue, &OIDLen) )
                        {
                            TCPIP_SNMP_ErrorStatusSet(errorStatusOffset,errorIndexOffset,SNMP_GEN_ERR,varIndex,snmpPutData);
                            bSnmpGenError = true;
                            break;
                        }
                        templen=OIDLen;
                        ptroid=OIDValue;

                        // For Get & Get-Next, value must be NULL.
                        if ( pduDbPtr->pduType != (uint8_t)SET_REQUEST )
                            if (!TCPIP_SNMP_DataIsASNNull())
                            break;

                        oidLookUpRet = TCPIP_SNMP_OIDFindInMgmtInfoBase(snmpFileDescrptr,pduDbPtr,OIDValue, OIDLen, &OIDInfo);
                        if(oidLookUpRet == SNMP_END_OF_MIB_VIEW)
                        {
                            temp = TCPIP_SNMP_NextLeafGet(snmpFileDescrptr,&OIDInfo);
                        }
                        if(oidLookUpRet == false)
                        {
                            templen=OIDLen;
                            ptroid=OIDValue;
                            if(TCPIP_SNMP_DataCopyToProcessBuffer(ASN_OID,snmpPutData)!= true)
                            {
                                getbulkOverFlowFlag = true;
                                break;
                            }

                            if(SnmpStackDcptMemStubPtr->appendZeroToOID)
                            {
                                if(TCPIP_SNMP_DataCopyToProcessBuffer(OIDLen+1,snmpPutData)!= true)//for appending "0"
                                {
                                    getbulkOverFlowFlag = true;
                                    break;
                                }
                                OIDLen += 1;
                            }
                            else
                            {
                                if(TCPIP_SNMP_DataCopyToProcessBuffer(OIDLen,snmpPutData)!= true)
                                {
                                    getbulkOverFlowFlag = true;
                                    break;
                                }
                            }

                            //Put OID
                            while( templen-- )
                            {
                                if(TCPIP_SNMP_DataCopyToProcessBuffer(*ptroid++,snmpPutData)!= true)
                                {
                                    getbulkOverFlowFlag = true;
                                    break;
                                }
                            }

                            if(SnmpStackDcptMemStubPtr->appendZeroToOID)
                            {
                                if(TCPIP_SNMP_DataCopyToProcessBuffer(0x00,snmpPutData)!= true)
                                {
                                    getbulkOverFlowFlag = true;
                                    break;
                                }
                            }

                            if(TCPIP_SNMP_DataCopyToProcessBuffer(SNMP_END_OF_MIB_VIEW,snmpPutData)!= true)
                            {
                                getbulkOverFlowFlag = true;
                                break;
                            }
                            if(TCPIP_SNMP_DataCopyToProcessBuffer(0x00,snmpPutData)!= true)
                            {
                                getbulkOverFlowFlag = true;
                                break;
                            }

                            //Start counting total number of bytes in this structure.
                            varPairLen.Val = OIDLen // as put by GetNextVar()
                             +2       // OID header
                             +2;      // endOfMibView bytes

                            snmpReqVarErrStatus.endOfMibViewErr	 |=(0x0001 << varIndex);
                        }
                        else if(temp != 0)//if(oidLookUpRet != SNMP_END_OF_MIB_VIEW)
                        {
                            temp = TCPIP_SNMP_ProcessGetBulkVar(&OIDInfo, &OIDValue[0],&OIDLen,&successor,pduDbPtr);
                        }
                        if ( temp == 0u )
                        {
                            templen=OIDLen;
                            ptroid=OIDValue;
                            if(TCPIP_SNMP_DataCopyToProcessBuffer(ASN_OID,snmpPutData)!=true)
                            {
                                getbulkOverFlowFlag = true;
                                break;
                            }
                            if(TCPIP_SNMP_DataCopyToProcessBuffer(OIDLen,snmpPutData)!= true)
                            {
                                getbulkOverFlowFlag = true;
                                break;
                            }

                            //Put OID
                            while( templen-- )
                            {
                                if(TCPIP_SNMP_DataCopyToProcessBuffer(*ptroid++,snmpPutData)!= true)
                                {
                                    getbulkOverFlowFlag = true;
                                    break;
                                }
                            }

                            /*Do send back the Same OID if get_next is EndOfMibView. Do not
                              append zero to this OID*/

                            if(TCPIP_SNMP_DataCopyToProcessBuffer(SNMP_END_OF_MIB_VIEW,snmpPutData)!=true)
                            {
                                getbulkOverFlowFlag = true;
                                break;
                            }
                            if(TCPIP_SNMP_DataCopyToProcessBuffer(0x00,snmpPutData)!= true)
                            {
                                getbulkOverFlowFlag = true;
                                break;
                            }

                            snmpReqVarErrStatus.endOfMibViewErr	 |=(0x0001 << varIndex);

                            //Start counting total number of bytes in this structure.
                            varPairLen.Val = OIDLen  // as put by GetNextVar()
                                 + 2     // OID header
                                 + 2;    // endOfMibView byte.
                        }
                        else
                        {
                            varPairLen.Val = (temp + 2);        // + OID headerbytes
                        }

                        varBindLen.Val += 4	// Variable Pair STRUCTURE byte + 1 length byte.
                        + varPairLen.Val;

                        prevOffset = snmpPutData->length;
                        snmpPutData->length = varStructLenOffset;
                        if(TCPIP_SNMP_DataCopyToProcessBuffer(varPairLen.v[1],snmpPutData)!=true)
                        {
                            getbulkOverFlowFlag = true;
                            break;
                        }
                        if(TCPIP_SNMP_DataCopyToProcessBuffer(varPairLen.v[0],snmpPutData)!=true)
                        {
                            getbulkOverFlowFlag = true;
                            break;
                        }

                        snmpPutData->length = prevOffset;
                        varStructLenOffset = snmpPutData->length;
                        if((varStructLenOffset - tempTxOffset) > (SNMP_MAX_MSG_SIZE))
                        {
                            getbulkOverFlowFlag = true;
                            break;
                        }

                    }//for(varBindCntr=0;varBindCntr<Getbulk_R;varBindCntr++)

                }//for(repeatCntr=0;repeatCntr<Getbulk_M;repeatCntr++)
			
                break;
            }//end of switch(smSnmp)

            /*If all the variables are processed and the repsonse pdu is updated with
              the number of variable responses ought to be in the response; you are done
              with the request pdu processing. Else continue to processing.*/
            if(Getbulk_N==0u && noOfVarToBeInResponse==0u)
            {
                smSnmp=SM_PKT_STRUCT_LEN_OFFSET;
                break;
            }

	}//end of while(1)		


	// Update the place holders with respective values.
		
	
	/* As per RFC 3416 - GET bULK Response - 4.2.3
	If the size of the message encapsulating the Response-PDU containing the 
	requested number of variable bindings would be greater than either a local
	constraint or the maximum message size of the originator, then the response
	is generated with a lesser number of variable bindings. This lesser number is
	the ordered set of variable bindings with some of the variable bindings at the
	end of the set removed, such that the size of the message encapsulating the
	Response-PDU is approximately equal to but no greater than either a local
	constraint or the maximum message size of the originator. Note that the 
	number of variable bindings removed has no relationship to the values of N, M, or R.*/
	if(getbulkOverFlowFlag && (pduDbPtr->pduType==GET_BULK_REQUEST))
	{
            snmpPutData->length = prevOffset;
            varBindLen.Val = varBindLen.Val ;
            TCPIP_UDP_TxOffsetSet(snmpDcpt[intfIdx].skt, snmpPutData->length, true);
	}

    prevOffset = snmpPutData->length;
    /* GetRequest-PDU (As per RFC 3416 - SECTION - 4.2.1)
    During the process of any OID,variable binding fails due to invalid OID
    or invalid OID type or invalid OID length etc, i,e other than "noSuchObject"
    or " noSuchInstance", then the Response-PDU is re-formatted with the same
    values in its request-id and variable-bindings fields as the received
GetRequest-PDU , with the value of its error-status field set to "genErr", 

    GetNextRequest-PDU (As per RFC 3416 - SECTION - 4.2.2)
    During the process of any OID,variable binding fails due to invalid OID
    or invalid OID type or invalid OID length etc, other than "endOfMibView" ,
    then the Response-PDU is re-formatted with the same values in its request-id and
    variable-bindings fields as the received GetNextRequest-PDU,with the value of
    its error-status field set to "genErr", and the value of its error-index
    field is set to the index of the failed variable binding.

    The generated Response-PDU is then encapsulated into a message. If the size of the resultant
    message is less than or equal to maximum message size of the originator, it is transmitted
    to the originator of the GetNextRequest-PDU.

    Otherwise, an alternate Response-PDU is generated. This alternate Response-PDU is formatted
    with the same values in its request-id field as the received GetNextRequest-PDU, with the value
    of its error-status field set to "tooBig", the value of its error-index field set to zero, and an empty
    variable-bindings field.

    */
    //calculate the number of bytes are the part of RESPONSE PDU
    if(bSnmpGenError)
    {
        if(((prevOffset - tempTxOffset) > SNMP_MAX_MSG_SIZE)
            && (pduDbPtr->pduType!=GET_BULK_REQUEST))
        {
            /* for snmpv2 (or snmpv3) by rfc3416 we return special
            *   tooBig(1) response with empty variable-bindings field.
            * error status  = toobig(1) and error_index set to 0.
            */

            TCPIP_SNMP_ErrorStatusSet(errorStatusOffset,errorIndexOffset,SNMP_TOO_BIG,0,snmpPutData);
            varBindLen.Val =  6 						// Request ID bytes (4+2)
                                            + 3 						// Error status 	(2+1)
                                            + 3;						// Error index		(2+1)
            snmpPutData->length = pduLenOffset;
            TCPIP_SNMP_DataCopyToProcessBuffer(varBindLen.v[1],snmpPutData);
            TCPIP_SNMP_DataCopyToProcessBuffer(varBindLen.v[0],snmpPutData);

            // varBindLen is reused as "packetLen".
            varBindLen.Val = 3						// SNMP Version bytes
                                            + 2 + communityLen		// community string bytes
                                            + 4 					// PDU structure header bytes.
                                            + varBindLen.Val;

            snmpPutData->length  = packetStructLenOffset;
            TCPIP_SNMP_DataCopyToProcessBuffer(varBindLen.v[1],snmpPutData);
            TCPIP_SNMP_DataCopyToProcessBuffer(varBindLen.v[0],snmpPutData);
            snmpPutData->length = varBindStructOffset-2;
            bSnmpSendPkt =  true;

            TCPIP_SNMP_PutDataArray(snmpDcpt[intfIdx].skt,snmpPutData);
            smSnmp = SM_PKT_STRUCT_LEN_OFFSET;
            return true;
        }
    }
    snmpPutData->length = varBindStructOffset;
    TCPIP_SNMP_DataCopyToProcessBuffer(varBindLen.v[1],snmpPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(varBindLen.v[0],snmpPutData);
    snmpPutData->length = prevOffset;


    // varBindLen is reused as "pduLen"
    varBindLen.Val = varBindLen.Val+4       // Variable Binding Strucure length
        + 6                         // Request ID bytes (4+2)
        + 3                         // Error status		(2+1)
        + 3;                        // Error index		(2+1)
    prevOffset = snmpPutData->length;

    snmpPutData->length = pduLenOffset;
    TCPIP_SNMP_DataCopyToProcessBuffer(varBindLen.v[1],snmpPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(varBindLen.v[0],snmpPutData);
    snmpPutData->length = prevOffset;


#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
    if(pduDbPtr->snmpVersion == SNMP_V3)
    {
        prevOffset = snmpPutData->length;

        snmpPutData->length = msgSecrtyParamLenOffset;
        TCPIP_SNMP_DataCopyToProcessBuffer(varBindLen.v[0]+tempByteCntr.v[0],snmpPutData);
        snmpPutData->length = prevOffset;
    }
#endif

    // Update the place holders with respective values.
    if(pduDbPtr->snmpVersion != SNMP_V3)
    {
        // varBindLen is reused as "packetLen".
        varBindLen.Val = 3                      // SNMP Version bytes
                + 2 + communityLen      // community string bytes
                + 4                     // PDU structure header bytes.
                + varBindLen.Val;
    }
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER		
    else
    {
        varBindLen.Val = 3       // SNMP Version bytes
                        + 2      // PDU structure header bytes.
                        + varBindLen.Val
                        +bytesAdded2Pdu.Val;
    }
#endif

    prevOffset = snmpPutData->length;

    snmpPutData->length = packetStructLenOffset;
    TCPIP_SNMP_DataCopyToProcessBuffer(varBindLen.v[1],snmpPutData);
    TCPIP_SNMP_DataCopyToProcessBuffer(varBindLen.v[0],snmpPutData);

    snmpPutData->length = prevOffset;

    TCPIP_SNMP_PutDataArray(snmpDcpt[intfIdx].skt,snmpPutData);

    smSnmp = SM_PKT_STRUCT_LEN_OFFSET;

    return true;
	
}

/****************************************************************************
Function:
    uint8_t TCPIP_SNMP_ProcessGetNextVar(OID_INFO* rec,PDU_INFO* pduDbPtr)

Summary:
    Retrieves next node from the MIB database.

Description:
    This routine reads into the MIB stored with the agent .
    It will search for the first lexicographic successor of the variable
    binding's name in the incoming GetNextRequest-PDU. If found, the
    corresponding variable binding's name and value fields in the Response
    pdu are set to the name and value of the located variable. If the
    lexicographic succesor is not found, the vlaue filed is set to
    "endofMibView" and name field is retained as in request.

Precondition:
    TCPIP_SNMP_ProcessVariables() is called.

Parameters:
    rec - Pointer to SNMP MIB object information for which next node
              to be found

Return Values:
    temp.V[0]- Total number of bytes copied to response packet if succesful.
    false	 - If End of MIB is reached or processing is failure.

Remarks:
    None.
 ***************************************************************************/
uint8_t TCPIP_SNMP_ProcessGetNextVar(OID_INFO* rec,PDU_INFO* pduDbPtr)
{
    TCPIP_UINT16_VAL temp;
    uint8_t putBytes=0;
    OID_INFO indexRec;
    uint8_t *pOIDValue;
    uint8_t OIDValue[SNMP_OID_MAX_LEN];
    uint8_t OIDLen;
    SNMP_INDEX_INFO indexInfo;
    MIB_INFO varNodeInfo;
    SNMP_ID varID;
    uint16_t OIDValOffset=0;
    uint16_t prevOffset;
    static uint8_t varDataType;
    static uint8_t indexBytes;
    uint8_t idLen = 1;
    uint8_t	dummyRead;

    SNMP_BUFFER_DATA *snmpPutData =  NULL;

#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
    SNMPV3MSGDATA	*dynPduBuf=NULL;

    SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr;
    SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0;

    TCPIP_SNMPV3_PacketProcStubPtrsGet(&snmpv3PktProcessingMemPntr);

    snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;
    dynPduBuf = &snmpv3EngnDcptMemoryStubPtr->ScopedPduRespnsBuf;
#endif

    snmpPutData = &SnmpStackDcptMemStubPtr->outPduBufData;
    temp.v[0] = 0;

    // Get next leaf only if this OID is a parent or a simple leaf node.
    if ( rec->nodeInfo.Flags.bIsParent ||
       (!rec->nodeInfo.Flags.bIsParent && !rec->nodeInfo.Flags.bIsSequence) )
    {
        if ( !TCPIP_SNMP_NextLeafGet(snmpFileDescrptr,rec))
            return false;
    }

    // Get complete OID string from oid record.
    if ( !TCPIP_SNMP_OIDStringGetByAddr(snmpFileDescrptr,rec, OIDValue, &OIDLen))
    {
        return false;
    }
    
    //to validate the REC ID is present or not
    // do while loop till find find a valid entry.
    while(1)
    {
        if(TCPIP_SNMP_RecordIDValidation(pduDbPtr->snmpVersion,rec->nodeInfo.Flags.bIsIDPresent,rec->id,OIDValue,OIDLen) != true)
        {
            if(!TCPIP_SNMP_NextLeafGet(snmpFileDescrptr,rec))
                    return false;
            else
            {
                // Get complete OID string from oid record.
                if ( !TCPIP_SNMP_OIDStringGetByAddr(snmpFileDescrptr,rec, OIDValue, &OIDLen))
                {
                    return false;
                }
            }
        }
        else
        {
            break;
        }
    }
    varNodeInfo.Val = 0;
    while(1)
    {
        if(!rec->nodeInfo.Flags.bIsSequence)
            break;
        // Need to fetch index information from MIB and prepare complete OID+
        // index response.
        varNodeInfo.Val = rec->nodeInfo.Val;
    
        // In this version, only 7-bit index is supported.
        SYS_FS_FileRead(snmpFileDescrptr,&dummyRead,1);
        indexBytes = 0;
        SYS_FS_FileRead(snmpFileDescrptr,&indexInfo.Val,1);
        SYS_FS_FileRead(snmpFileDescrptr,&idLen,1);
        if(idLen == 1)
        {
            uint8_t temp;
            SYS_FS_FileRead(snmpFileDescrptr,(uint8_t*)&temp,1);
            indexRec.id = temp & 0xFF;
        }
        else if(idLen == 2)
        {
            uint8_t temp[2];
            SYS_FS_FileRead(snmpFileDescrptr,temp,2);
            indexRec.id = 0;
            indexRec.id = temp[0] & 0xFF;
            indexRec.id <<= 8;
            indexRec.id |= temp[1] & 0xFF;
        }
        indexRec.dataType = 0;
        SYS_FS_FileRead(snmpFileDescrptr,(uint8_t*)&indexRec.dataType,1);
    
        indexRec.index = rec->index;
    
        // Check with application to see if there exists next index
        // for this index id.
        if (!TCPIP_SNMP_NextIndexGet(indexRec.id, &indexRec.index))
        {
            if ( !TCPIP_SNMP_NextLeafGet(snmpFileDescrptr,rec))
                return false;
            
            if (!TCPIP_SNMP_OIDStringGetByAddr(snmpFileDescrptr,rec, OIDValue, &OIDLen))
            {   
                return false;
            }
            if(TCPIP_SNMP_RecordIDValidation(pduDbPtr->snmpVersion,rec->nodeInfo.Flags.bIsIDPresent,rec->id,OIDValue,OIDLen) != true)
            continue;
        }
        else
        {
            break;
        }
    }

    // Copy complete OID string to create response packet.
    pOIDValue = OIDValue;	
    temp.v[0] = OIDLen;
    if(pduDbPtr->snmpVersion != SNMP_V3)
    {
        OIDValOffset = snmpPutData->length;
        //temp.v[0] = OIDLen;
        snmpPutData->length = OIDValOffset+1;
        while( temp.v[0]-- )
            TCPIP_SNMP_DataCopyToProcessBuffer(*pOIDValue++,snmpPutData);
    }
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
    else
    {
        OIDValOffset = dynPduBuf->length;
        //temp.v[0] = OIDLen;
        //dynPduBuf.length = OIDValOffset+1;	// offset for the OID length
        if(TCPIP_SNMPv3_DataCopyToProcessBuff(0,dynPduBuf)!= true)
            return false;
        while( temp.v[0]-- )
        {
            if(TCPIP_SNMPv3_DataCopyToProcessBuff(*pOIDValue++,dynPduBuf) != true)
                return false;
        }
    }
#endif

	//Put OID

    // Start counting number of bytes put - OIDLen is already counted.
    temp.v[0] = OIDLen;

    varDataType = rec->dataType;
    varID = rec->id;

    // If this is a simple OID, handle it as a GetVar command.
    if(!rec->nodeInfo.Flags.bIsSequence)
    {
    	if(pduDbPtr->snmpVersion != SNMP_V3)
        {
            // This is an addition to previously copied OID string.
            // This is index value of '0'.
            TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpPutData);
            temp.v[0]++;

            // Since we added one more byte to previously copied OID
            // string, we need to update OIDLen value.

            prevOffset = snmpPutData->length;
            snmpPutData->length = OIDValOffset;
            TCPIP_SNMP_DataCopyToProcessBuffer(++OIDLen,snmpPutData);
            snmpPutData->length = prevOffset;

            // Now do Get on this simple variable.
            prevOffset = snmpPutData->length;
            putBytes = TCPIP_SNMP_ProcessGetVar(rec, false,pduDbPtr);
            if ( putBytes == 0u )
            {
                snmpPutData->length = prevOffset;
                TCPIP_SNMP_DataCopyToProcessBuffer(ASN_NULL,snmpPutData);
                TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpPutData);
                putBytes = 2;
            }
        }
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
        else
        {
            // This is index value of '0'.
            if(TCPIP_SNMPv3_DataCopyToProcessBuff(0,dynPduBuf) != true)
                return false;
            temp.v[0]++;
            prevOffset = dynPduBuf->length;
            dynPduBuf->length = OIDValOffset;
            // Since we added one more byte to previously copied OID
            // string, we need to update OIDLen value.
            if(TCPIP_SNMPv3_DataCopyToProcessBuff(++OIDLen,dynPduBuf)!= true)
                return false;
            dynPduBuf->length = prevOffset;

            // Now do Get on this simple variable.
            prevOffset = dynPduBuf->length;
            putBytes = TCPIP_SNMP_ProcessGetVar(rec, false,pduDbPtr);
            if(dynPduBuf->length >= dynPduBuf->maxlength)
                    return false;
            if(( putBytes == 0u ) && (pduDbPtr->snmpVersion == SNMP_V3))
            {
                dynPduBuf->length = prevOffset;
                if(TCPIP_SNMPv3_DataCopyToProcessBuff(ASN_NULL,dynPduBuf)!= true)
                        return false;
                if(TCPIP_SNMPv3_DataCopyToProcessBuff(0,dynPduBuf)!= true)
                        return false;
                putBytes = 2;
            }
        }
#endif
        temp.v[0] += putBytes; // TCPIP_SNMP_ProcessGetVar(rec, false,pduDbPtr);

         // Return with total number of bytes copied to response packet.
        return temp.v[0];
    }

    // Index is assumed to be dynamic, and leaf node.
    // mib2bib has already ensured that this was the case.
    indexRec.nodeInfo.Flags.bIsConstant = 0;
    indexRec.nodeInfo.Flags.bIsParent = 0;
    indexRec.nodeInfo.Flags.bIsSequence = 1;

    // Now handle this as simple GetVar.
    // Keep track of number of bytes added to OID.
    indexBytes += TCPIP_SNMP_ProcessGetVar(&indexRec, true,pduDbPtr);

    rec->index = indexRec.index;

    // These are the total number of bytes put so far as a result of this function.
    temp.v[0] += indexBytes;

    // These are the total number of bytes in OID string including index bytes.
    OIDLen += indexBytes;

    if(pduDbPtr->snmpVersion != SNMP_V3)
    {
        // Since we added index bytes to previously copied OID
        // string, we need to update OIDLen value.
        prevOffset = snmpPutData->length ;
            snmpPutData->length = OIDValOffset;

        TCPIP_SNMP_DataCopyToProcessBuffer(OIDLen,snmpPutData);
        snmpPutData->length = prevOffset;

    }
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
    else
    {
        // Since we added index bytes to previously copied OID
        // string, we need to update OIDLen value.
        prevOffset = dynPduBuf->length;
        dynPduBuf->length = OIDValOffset;
        TCPIP_SNMPv3_DataCopyToProcessBuff(OIDLen,dynPduBuf);
        dynPduBuf->length = prevOffset;
    }
#endif

    // Fetch actual value itself.
    // Need to restore original OID value.
    rec->nodeInfo.Val = varNodeInfo.Val;
    rec->id = varID;
    rec->dataType = varDataType;

    temp.v[0] += TCPIP_SNMP_ProcessGetVar(rec, false,pduDbPtr);
    return temp.v[0];
}


/****************************************************************************
Function:
    uint8_t TCPIP_SNMP_ProcessGetBulkVar(OID_INFO* rec, uint8_t* oidValuePtr,
                                               uint8_t* oidLenPtr,uint8_t* successor)

Summary:
    This routine process the SNMPv2c Get Bulk Request.

Description:
    TCPIP_SNMP_ProcessVariables() processes the received snmp request pdu for each of
    the variable binding in the variable binding list to produce a response
    pdu. Depending on the number of the Max_repetitions for every variable
    in the list for which Bulk information is expected, TCPIP_SNMP_ProcessGetBulkVar()
    is executed. It searches for the next lexicographically ordered
    successor for of the OID received in the request. For each of the
    iterations upto max-repetitions, the next leaf node is searched in the
    MIB to that of the leaf node found in the last iteration, for the
    corresponding variable binding.

Precondition:
    TCPIP_SNMP_ProcessVariables() is called.

Parameters:
    rec 		- Pointer to SNMP MIB variable object information OID
    oidValuePtr	- Pointer to new node OID found in MIB
    oidLenPtr	- Oid length
    successor	- 'I'th lexicographic successor to be found value

Return Values:
    false 	  - If no lexicographic successor found
    temp.v[0] - Total number of bytes copied to response packet

Remarks:
    None.
***************************************************************************/
uint8_t TCPIP_SNMP_ProcessGetBulkVar(OID_INFO* rec, uint8_t* oidValuePtr, uint8_t* oidLenPtr,uint8_t* successor,PDU_INFO* pduDbPtr)
{    
    //uint8_t ref;
    uint8_t putBytes,cntr;
    uint8_t OIDLen;
    static uint8_t varDataType;
    static uint8_t indexBytes;
    uint8_t sequenceCnt=0;
    uint8_t sequenceRepeatCnt=0;
    bool lbNextLeaf;
    SNMP_ID varID;
    OID_INFO indexRec;
    SNMP_INDEX_INFO indexInfo;
    MIB_INFO varNodeInfo;
    uint16_t OIDValOffset;
    uint16_t varBindOffset=0;
    uint16_t prevOffset;
    TCPIP_UINT16_VAL temp;
    //static SNMP_VAL v;
    uint8_t idLen=1;
    uint8_t dummyRead;
	
	
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
    SNMPV3MSGDATA	*dynPduBuf=NULL;
    SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr;
    SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0;

    TCPIP_SNMPV3_PacketProcStubPtrsGet(&snmpv3PktProcessingMemPntr);

    snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;
    dynPduBuf = &snmpv3EngnDcptMemoryStubPtr->ScopedPduRespnsBuf;
#endif

    SNMP_BUFFER_DATA *snmpPutData =  NULL;
    snmpPutData = &SnmpStackDcptMemStubPtr->outPduBufData;

    /* intialize the local variables to 0 */
    OIDLen=0;
    sequenceCnt=0;
    sequenceRepeatCnt=0;
    varID=0;
    OIDValOffset=0;
    varBindOffset=0;
    prevOffset=0;
    temp.Val=0;

    lbNextLeaf = false;
    temp.v[0] = 0;
    sequenceRepeatCnt=*successor;

    //Reach to the node for the expected iteration
    for(cntr=0;cntr<=*successor;cntr++)
    {
    // Get next leaf only if this OID is a parent or a simple leaf node.
        if((rec->nodeInfo.Flags.bIsParent)||
        (!rec->nodeInfo.Flags.bIsParent && !rec->nodeInfo.Flags.bIsSequence))
        {	/* to maintain the number of interations */
            sequenceCnt++;
            if(!TCPIP_SNMP_NextLeafGet(snmpFileDescrptr,rec))
                return false;
        }
    }

    /* If request OID is a sequence variable, the below for loop retrives the
    expected instance for the OID. SequenceRepeatCnt starts with "0th instance" and
    increments to Max repeatations. Find the exact indexed OID in the request at first.
    If indexed OID is not available, then go for the next index.
    If the next index is not available , then go to the next leaf.
    */
    for(;sequenceCnt<=sequenceRepeatCnt;sequenceCnt++)
    {
        if(rec->nodeInfo.Flags.bIsSequence)
        {
            TCPIP_SNMP_ExactIndexGet(rec->id,&rec->index);
            if(!TCPIP_SNMP_NextIndexGet(rec->id,&rec->index))
            {
                if(!TCPIP_SNMP_NextLeafGet(snmpFileDescrptr,rec))
                    return false;
            }
        }
        else
        {
            if(!TCPIP_SNMP_NextLeafGet(snmpFileDescrptr,rec))
                return false;
        }
    }

    // Get complete OID string from oid record.
    if(!TCPIP_SNMP_OIDStringGetByAddr(snmpFileDescrptr,rec, oidValuePtr, &OIDLen))
        return false;

    //to validate the REC ID is present or not
    while(1)
    {
        if(TCPIP_SNMP_RecordIDValidation(pduDbPtr->snmpVersion,rec->nodeInfo.Flags.bIsIDPresent,rec->id,oidValuePtr,OIDLen) != true)
        {
            if(!TCPIP_SNMP_NextLeafGet(snmpFileDescrptr,rec))
                    return false;
            else
            {
                    // Get complete OID string from oid record.
                if(!TCPIP_SNMP_OIDStringGetByAddr(snmpFileDescrptr,rec, oidValuePtr, &OIDLen))
                    return false;
            }
        }
        else
        {
            break;
        }
    }
    // get exact index value when it is a sequence variable
    varNodeInfo.Val = 0;
    while(1)
    {
        if(!rec->nodeInfo.Flags.bIsSequence)
            break;
        // Need to fetch index information from MIB and prepare complete OID+
        // index response.
        varNodeInfo.Val = rec->nodeInfo.Val;

        // In this version, only 7-bit index is supported.
        SYS_FS_FileRead(snmpFileDescrptr,&dummyRead,1);
        indexBytes = 0;
        SYS_FS_FileRead(snmpFileDescrptr,&indexInfo.Val,1);
        SYS_FS_FileRead(snmpFileDescrptr,&idLen,1);
        if(idLen == 1)
        {
            uint8_t temp;
    	    SYS_FS_FileRead(snmpFileDescrptr,&temp,1);
    	    indexRec.id = temp & 0xFF;
        }
        else if(idLen == 2)
        {
            uint8_t temp[2];
    	    SYS_FS_FileRead(snmpFileDescrptr,temp,2);
            indexRec.id = 0;
            indexRec.id = temp[0] & 0xFF;
            indexRec.id <<= 8;
            indexRec.id |= temp[1] & 0xFF;
        }
        indexRec.dataType = 0;
        SYS_FS_FileRead(snmpFileDescrptr,(uint8_t*)&indexRec.dataType,1);

        indexRec.index = rec->index;
         // Check with application to see if there exists next index
        // for this index id.
        if (!TCPIP_SNMP_ExactIndexGet(indexRec.id, &indexRec.index))
        {
            if ( !TCPIP_SNMP_NextLeafGet(snmpFileDescrptr,rec))
                return false;
            
            if (!TCPIP_SNMP_OIDStringGetByAddr(snmpFileDescrptr,rec, oidValuePtr,&OIDLen))
            {   
                return false;
            }
            if(TCPIP_SNMP_RecordIDValidation(pduDbPtr->snmpVersion,rec->nodeInfo.Flags.bIsIDPresent,rec->id,oidValuePtr,OIDLen) != true)
            continue;
        }
        else
        {
            break;
        }
    }
	
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
    if(pduDbPtr->snmpVersion == SNMP_V3)
    {
        varBindOffset = dynPduBuf->length;
        if(TCPIP_SNMPv3_DataCopyToProcessBuff(ASN_OID,dynPduBuf) != true)
            return false;

        OIDValOffset = dynPduBuf->length;
        temp.v[0] = OIDLen;
        if(TCPIP_SNMPv3_DataCopyToProcessBuff(0,dynPduBuf) != true)
                return false;
        //Put OID
        while( temp.v[0]-- )
        {
            if(TCPIP_SNMPv3_DataCopyToProcessBuff(*oidValuePtr,dynPduBuf) != true)
                return false;
            oidValuePtr++;
        }
    }
    else
#endif
    {
        varBindOffset = snmpPutData->length;
        if(TCPIP_SNMP_DataCopyToProcessBuffer(ASN_OID,snmpPutData)!= true)
            return false;

        OIDValOffset = snmpPutData->length;
        temp.v[0] = OIDLen;
        snmpPutData->length = OIDValOffset+1;
        //Put OID
        while( temp.v[0]-- )
        {
            if(TCPIP_SNMP_DataCopyToProcessBuffer(*oidValuePtr,snmpPutData)!= true)
                return false;
            oidValuePtr++;
        }
    }
	// Start counting number of bytes put - OIDLen is already counted.
    temp.v[0] =*oidLenPtr= OIDLen;

    varDataType = rec->dataType;
    varID = rec->id;

    // If this is a simple OID, handle it as a GetVar command.
    if (!rec->nodeInfo.Flags.bIsSequence)
    {
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
    	if(pduDbPtr->snmpVersion == SNMP_V3)
    	{
            // This is an addition to previously copied OID string.
            // This is index value of '0'.
             if(TCPIP_SNMPv3_DataCopyToProcessBuff(0,dynPduBuf) != true)
                return false;
            temp.v[0]++;

            // Since we added one more byte to previously copied OID
            // string, we need to update OIDLen value.
            prevOffset = dynPduBuf->length;
            dynPduBuf->length = OIDValOffset;
            if(TCPIP_SNMPv3_DataCopyToProcessBuff(++OIDLen,dynPduBuf) != true)
                return false;
            dynPduBuf->length = prevOffset;


            // Now do Get on this simple variable.
            prevOffset = dynPduBuf->length;
            putBytes = TCPIP_SNMP_ProcessGetVar(rec, false,pduDbPtr);
    	}
	else
#endif
        {
             // This is an addition to previously copied OID string.
            // This is index value of '0'.
            if(TCPIP_SNMP_DataCopyToProcessBuffer(0,snmpPutData)!= true)
                return false;
            temp.v[0]++;

            // Since we added one more byte to previously copied OID
            // string, we need to update OIDLen value.

            prevOffset = snmpPutData->length;
            snmpPutData->length = OIDValOffset;

            if(TCPIP_SNMP_DataCopyToProcessBuffer(++OIDLen,snmpPutData)!= true)
                return false;
            snmpPutData->length = prevOffset;
            // Now do Get on this simple variable.
            prevOffset = snmpPutData->length;
            putBytes = TCPIP_SNMP_ProcessGetVar(rec, false,pduDbPtr);
            if(putBytes == false)
                return false;
        }
        temp.v[0] += putBytes; // TCPIP_SNMP_ProcessGetVar(rec, false,pduDbPtr);

            // Return with total number of bytes copied to response packet.
        return temp.v[0];
    }
    
    // Index is assumed to be dynamic, and leaf node.
    // mib2bib has already ensured that this was the case.
    indexRec.nodeInfo.Flags.bIsConstant = 0;
    indexRec.nodeInfo.Flags.bIsParent = 0;
    indexRec.nodeInfo.Flags.bIsSequence = 1;

    // Now handle this as simple GetVar.
    // Keep track of number of bytes added to OID.
    putBytes = TCPIP_SNMP_ProcessGetVar(&indexRec, true,pduDbPtr);
    if(putBytes == false)
	    return false;
    indexBytes += putBytes;

    rec->index = indexRec.index;

    // These are the total number of bytes put so far as a result of this function.
    temp.v[0] += indexBytes;

    // These are the total number of bytes in OID string including index bytes.
    OIDLen += indexBytes;

    if(pduDbPtr->snmpVersion != SNMP_V3)
    {
        // Since we added index bytes to previously copied OID
        // string, we need to update OIDLen value.
        prevOffset = snmpPutData->length;
        snmpPutData->length = OIDValOffset;
        TCPIP_SNMP_DataCopyToProcessBuffer(OIDLen,snmpPutData);
        snmpPutData->length = prevOffset;
    }
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
    else
    {
        // Since we added index bytes to previously copied OID
        // string, we need to update OIDLen value.
        prevOffset = dynPduBuf->length;
        dynPduBuf->length =OIDValOffset;
        TCPIP_SNMPv3_DataCopyToProcessBuff(OIDLen,dynPduBuf);
        dynPduBuf->length = prevOffset;
    }
#endif
    // Fetch actual value itself.
    // Need to restore original OID value.
    rec->nodeInfo.Val = varNodeInfo.Val;
    rec->id = varID;
    rec->dataType = varDataType;

    temp.v[0] += TCPIP_SNMP_ProcessGetVar(rec, false,pduDbPtr);
	
    return temp.v[0];
}


/****************************************************************************
Function:
    uint8_t TCPIP_SNMP_OIDFindInMgmtInfoBase(int32_t fileDescr,PDU_INFO* pduDbPtr,uint8_t* oid, uint8_t oidLen, OID_INFO* rec)

Summary:
    To search and validate whether the requested OID is in the MIB database.

Description:
    The MIB database is stored with the agent in binary mib format.
    This is the binary mib format:
    <oid, nodeInfo, [id], [SiblingOffset], [DistantSibling], [dataType],
    [dataLen], [data], [{IndexCount, <IndexType>, <Index>, ...>]}, ChildNode
    variable bind name is a dotted string of oid. Every oid is a node in the
    MIB tree and have varied information. This routine on reception of the
    snmp request, will search for every oid in the var name. This routine
    will return information whether the requested var name is part of the
    MIB tree data structre of this agent or not.

Precondition:
    Valid snmp request with valid OID format is received.

Parameters:
	fileDescr - A valid File decriptor which is alread opened.
    pduDbPtr	- Pointer to received snmp  pdu elements information
    oid		- Pointer to the string of OID to be searched
    oidLen		- Oid length
    rec		- Pointer to SNMP MIB variable object information

Return Values:
    true	-   If the complete OID string is found in the mib
    false	-   If complete OID do not match.
            Also different erros returned are
            SNMP_END_OF_MIB_VIEW
            SNMP_NO_SUCH_NAME
            SNMP_NO_SUCH_OBJ
            SNMP_NO_SUCH_INSTANCE
Remarks:
    This routine works for the snmp mib storage format. It uses the file system
    APIs to read,search and collect information from the mib database.
***************************************************************************/
uint8_t TCPIP_SNMP_OIDFindInMgmtInfoBase(int32_t fileDescr,PDU_INFO* pduDbPtr,uint8_t* oid, uint8_t oidLen, OID_INFO* rec)
{
    uint8_t idLen=1;
    uint8_t savedOID,tempSavedOID;
    uint8_t matchedCount;
    uint8_t snmpVer;
    uint8_t snmpReqType;
    uint8_t tempLen;
    uint8_t* reqOidPtr;
    uint8_t comapreOidWithSibling=false;

    TCPIP_UINT16_VAL tempData;
    uint32_t hNode,tempHNode;//    FS hNode;
    bool  bFoundIt=false;

    SnmpStackDcptMemStubPtr->appendZeroToOID=true;

    snmpVer=pduDbPtr->snmpVersion;
    snmpReqType=pduDbPtr->pduType;

    if(!SNMPStatus.Flags.bIsFileOpen )
       return false;
	
    hNode = 0;
    matchedCount = oidLen;

    tempLen=oidLen;
    reqOidPtr=oid;

    while( 1 )
    {
        SYS_FS_FileSeek(fileDescr, hNode, SYS_FS_SEEK_SET);
        rec->hNode = SYS_FS_FileTell(fileDescr); // hNode;
        SYS_FS_FileRead(fileDescr,&savedOID,1);
        if(comapreOidWithSibling==(uint8_t)false)
        {
            tempSavedOID=savedOID;
            tempHNode=hNode;
        }
        SYS_FS_FileRead(fileDescr,&rec->nodeInfo.Val,1);
        if(rec->nodeInfo.Flags.bIsIDPresent)
        {
            SYS_FS_FileRead(fileDescr,&idLen,1);
            if(idLen == 1)
            {
                uint8_t temp;
                SYS_FS_FileRead(fileDescr,&temp,1);
                rec->id = temp & 0xFF;
            }
            else if(idLen == 2)
            {
                uint8_t temp[2];
                SYS_FS_FileRead(fileDescr,temp,2);
                rec->id = 0;
                rec->id = temp[0] & 0xFF;
                rec->id <<= 8;
                rec->id |= temp[1] & 0xFF;
            }
        }
        if(rec->nodeInfo.Flags.bIsSibling)
        {
            SYS_FS_FileRead(fileDescr,&tempData.v[0],1);
            SYS_FS_FileRead(fileDescr,&tempData.v[1],1);
            rec->hSibling = tempData.Val;
        }

        if ( savedOID != *reqOidPtr )
        {
            /*if very first OID byte does not match, it may be because it is
            0, 1 or 2.  In that case declare that there is a match.
            The command processor would detect OID type and continue or reject
            this OID as a valid argument.*/
            if(matchedCount == oidLen)
            {
                bFoundIt =  true;
                break;
            }
			
            if(comapreOidWithSibling==(uint8_t)true && !rec->nodeInfo.Flags.bIsSibling)
            {
                bFoundIt =  false;
                break;
            }

            if ( rec->nodeInfo.Flags.bIsSibling )
            {
                SYS_FS_FileSeek(fileDescr, tempData.Val, SYS_FS_SEEK_SET);
                hNode = SYS_FS_FileTell(fileDescr);
                comapreOidWithSibling=true;
            }
            else
            {
                bFoundIt =  false;
                break;
            }
        }
        else
        {
	        // One more oid byte matched.
            matchedCount--;
            reqOidPtr++;

            // A node is said to be matched if last matched node is a leaf node
            // or all but last OID string is matched and last byte of OID is '0'.
            // i.e. single index.
            if ( !rec->nodeInfo.Flags.bIsParent )
            {
            	// Read and discard Distant Sibling info if there is any.
                if ( rec->nodeInfo.Flags.bIsDistantSibling )
                {	
                    SYS_FS_FileRead(fileDescr,&tempData.v[0],1);
                    SYS_FS_FileRead(fileDescr,&tempData.v[1],1);

                    rec->hSibling = tempData.Val;					
                }

                rec->dataType = 0;

                SYS_FS_FileRead(fileDescr,(uint8_t*)&rec->dataType,1);
                rec->hData = SYS_FS_FileTell(fileDescr);

                if(snmpReqType==SNMP_GET && matchedCount == 0u)
                {
                    SnmpStackDcptMemStubPtr->appendZeroToOID=false;
                    bFoundIt =  false;
                    break;
                }
                else if(snmpReqType==(uint8_t)SNMP_GET
                        && matchedCount == 1u && *reqOidPtr == 0x00u)
                {
                    SnmpStackDcptMemStubPtr->appendZeroToOID=false;
                }
                else if(snmpReqType==SNMP_GET_NEXT && matchedCount == 0u)
                {
                    SnmpStackDcptMemStubPtr->appendZeroToOID=true;
                    SnmpStackDcptMemStubPtr->getZeroInstance=true;
                }
                else if(snmpReqType==(uint8_t)SNMP_V2C_GET_BULK && matchedCount == 1u )
                {
                    SnmpStackDcptMemStubPtr->appendZeroToOID=false;
                }
                bFoundIt =  true;
                break;
            }
            else if(matchedCount == 1u && *reqOidPtr == 0x00u)
            {
                SnmpStackDcptMemStubPtr->appendZeroToOID=false;
                if(rec->nodeInfo.Flags.bIsParent)
                {
                    bFoundIt =  false;
                    break;
                }

            }
            else if(matchedCount == 0u)
            {
                if(rec->nodeInfo.Flags.bIsParent || snmpReqType==SNMP_GET)
                {
                    SnmpStackDcptMemStubPtr->appendZeroToOID=false;
                    bFoundIt =  false;
                    break;
                }
                else
                {
                     bFoundIt =  true;
                     break;
                }
            }
            else
            {
                hNode = SYS_FS_FileTell(fileDescr);
                // Try to match following child node.
                continue;
            }
        }
    }

    if(bFoundIt == true)
    {
    	// Convert index info from OID to regular value format.
      	rec->index = savedOID;

    	/*To Reach To The Next leaf Node */
        savedOID = *reqOidPtr;
    	
    	rec->indexLen = 1;

    	if(matchedCount ==1u)
    	{
            rec->index = *reqOidPtr;
    	}
    	else if(matchedCount == 0u)
    	{
            rec->index = 0;
    	}
    	else if ( matchedCount > 1u || savedOID & 0x80 /*In this version, we only support 7-bit index*/)
        {	
            // Current instnace spans across more than 7-bit.
            rec->indexLen = 0xff;

            if(snmpReqType==SNMP_GET && snmpVer==(uint8_t)SNMP_V1)
            {
                return SNMP_NO_SUCH_NAME;
            }
            else if(snmpReqType==SNMP_GET && snmpVer==(uint8_t)SNMP_V2C)
            {
                if(matchedCount== oidLen) //No OBJECT IDNETIFIER Prefix match
                    return SNMP_NO_SUCH_INSTANCE;
                else
                    return SNMP_NO_SUCH_OBJ;
            }

            return false;
        }
        
        if(SnmpStackDcptMemStubPtr->getZeroInstance)
        {
            rec->index = SNMP_INDEX_INVALID;
        }

    	return true;
    }
    else
    {
    	if(snmpReqType==SNMP_GET)
    	{
            if(snmpVer==(uint8_t)SNMP_V1)
                return SNMP_NO_SUCH_NAME;

            else if(snmpVer==(uint8_t)SNMP_V2C)
            {
                if(matchedCount== oidLen) //No OBJECT IDNETIFIER Prefix match
                    return SNMP_NO_SUCH_INSTANCE;
                else
                    return SNMP_NO_SUCH_OBJ;
            }
    	}
    	else if((snmpReqType==SNMP_GET_NEXT||snmpReqType==SNMP_V2C_GET_BULK) && snmpVer==(uint8_t)SNMP_V2C)
    	{
            return SNMP_END_OF_MIB_VIEW;
    	}    	
    }    
    return false;
}	

/****************************************************************************
Function:
    bool TCPIP_SNMP_NextLeafGet(int32_t fileDescr,OID_INFO* rec)

Summary:
    Searches for the next leaf node in the MIP tree.

Description:
    This routine searches for the next leaf node from the current node.
    The input to this function is the node from where next leaf node
    is to be located. The next leaf node will be a silbing else distant
    sibling or leaf node of next branch, if any present. The input parameter
    var pointer will be updated with the newly found leaf node OID info.

Precondition:
    TCPIP_SNMP_ProcessGetBulkVar() else TCPIP_SNMP_ProcessGetNextVar() is called.

Parameters:
	fileDescr - A valid File decriptor which is alread opened.
    rec		- Pointer to SNMP MIB variable object information

Return Values:
    true	- If next leaf node is found.
    false	- There is no next leaf node.

Remarks:
    None.
***************************************************************************/
bool TCPIP_SNMP_NextLeafGet(int32_t fileDescr, OID_INFO* rec)
{
    TCPIP_UINT16_VAL temp;
    uint8_t idLen=1;

    // If current node is leaf, its next sibling (near or distant) is the next leaf.
    if ( !rec->nodeInfo.Flags.bIsParent )
    {
        // Since this is a leaf node, it must have at least one distant or near
        // sibling to get next sibling.
        if(rec->nodeInfo.Flags.bIsSibling ||
           rec->nodeInfo.Flags.bIsDistantSibling )
        {
            // Reposition at sibling.
            SYS_FS_FileSeek(fileDescr, rec->hSibling, SYS_FS_SEEK_SET);

            // Fetch node related information
        }
        // There is no sibling to this leaf.  This must be the very last node on the tree.
        else
        {
            //--SYS_FS_close();
            return false;
        }
    }

    while( 1 )
    {
        // Remember current offset for this node.
        rec->hNode = SYS_FS_FileTell(fileDescr);

        // Read OID byte.
        SYS_FS_FileRead(fileDescr,&rec->oid,1);
        SYS_FS_FileRead(fileDescr,&rec->nodeInfo.Val,1);
        if ( rec->nodeInfo.Flags.bIsIDPresent )
        {
            SYS_FS_FileRead(fileDescr,&idLen,1);
            if(idLen == 1)
            {
                uint8_t temp;
                SYS_FS_FileRead(fileDescr,&temp,1);
                rec->id = temp & 0xFF;
            }
            else if(idLen == 2)
            {
                uint8_t temp[2];
                SYS_FS_FileRead(fileDescr,temp,2);
                rec->id = 0;
                rec->id = temp[0] & 0xFF;
                rec->id <<= 8;
                rec->id |= temp[1] & 0xFF;
            }
        }
        if ( rec->nodeInfo.Flags.bIsSibling ||
             rec->nodeInfo.Flags.bIsDistantSibling )
        {
            SYS_FS_FileRead(fileDescr,&temp.v[0],1);
            SYS_FS_FileRead(fileDescr,&temp.v[1],1);
            rec->hSibling = temp.Val;
        }
        if ( rec->nodeInfo.Flags.bIsParent )
        {
            continue;
        }
        rec->dataType = 0;
        SYS_FS_FileRead(fileDescr,(uint8_t*)&rec->dataType,1);
        rec->hData = SYS_FS_FileTell(fileDescr);

        // Since we just found next leaf in line, it will always have zero index
        // to it.
        rec->indexLen = 1;
        rec->index = 0;

        if (rec->nodeInfo.Flags.bIsSequence)
        {
            rec->index = SNMP_INDEX_INVALID;
        }

        return true;
    }
    return false;
}


/****************************************************************************
Function:
    bool TCPIP_SNMP_CommunityStringIsValid(char* community, uint8_t* len)

Summary:
    Verifies for the community string datatype and the max
    community name and length, this agent can process.

Description:
    This routine populates and validates the community datatype, community
    name and length from the received snmp request pdu. Community name is
    used for accessing public and private memebrs of the mib.

Precondition:
    TCPIP_SNMP_ProcessPDUHeader() is called.

Parameters:
    community -	Pointer to memory where community string will be stored.
    len		  - Pointer to memory where comunity length gets stored.

Return Values:
    true	- If valid community received.
    false	- If community is not valid.

Remarks:
    None.
***************************************************************************/
static bool TCPIP_SNMP_CommunityStringIsValid(char* community, uint8_t* len)
{
    uint8_t tempData;
    uint8_t tempLen;
    uint8_t intfIdx;
    intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_NetDefaultGet())->netIfIx;
	
    tempData = TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt);
    if ( !IS_OCTET_STRING(tempData) )
        return false;

    tempLen = TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt);
    *len    = tempLen;
    if ( tempLen > SNMP_COMMUNITY_MAX_LEN )
        return false;

    while( tempLen-- )
    {
        tempData = TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt);
        *community++ = tempData;
    }
    *community = '\0';
     return true;
}

/****************************************************************************
Function:
    bool TCPIP_SNMP_VarDataTypeIsValidInteger(uint32_t* val)

Summary:
    Verifies variable datatype as int and retrieves its value.

Description:
    This routine populates and validates the received variable for the
    data type as "ASN_INT" and the data length for max 4 bytes.

Precondition:
    TCPIP_SNMP_ProcessPDUHeader() or SNMPProcessGetSetHeader() is called.

Parameters:
    val - Pointer to memory where int var value will be stored.

ReturnValues:
    true	- If valid integer type and value is received.
    false	- Other than integer data type and value received .

Remarks:
    None.
***************************************************************************/
bool TCPIP_SNMP_VarDataTypeIsValidInteger(uint32_t* val)
{
    TCPIP_UINT32_VAL tempData;
    TCPIP_UINT32_VAL tempLen;
    uint8_t intfIdx;
    intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_NetDefaultGet())->netIfIx;
	
    tempLen.Val = 0;

    // Get variable type
    if ( !IS_ASN_INT(TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt)) )
        return false;

    if ( !TCPIP_SNMP_LengthIsValid(&tempLen.w[0]) )
        return false;

    // Integer length of more than 32-bit is not supported.
    if ( tempLen.Val > 4u )
        return false;

    tempData.Val = 0;
    while( tempLen.v[0]-- )
        tempData.v[tempLen.v[0]] = TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt);

    *val = tempData.Val;

    return true;
}


/****************************************************************************
Function:
    bool TCPIP_SNMP_PDUIsValid(SNMP_ACTION* pdu)

Summary:
    Verifies for the snmp request type.

Description:
    This routine populates and verifies for the received snmp request
    pdu type.

Precondition:
    TCPIP_SNMP_ProcessPDUHeader() is called.

Parameters:
    val - Pointer to memory where received snmp request type is stored.

Return Values:
    true	- If this snmp request can be processed by the agent.
    false	- If the request can not be processed.

Remarks:
    None.
***************************************************************************/
bool TCPIP_SNMP_PDUIsValid(SNMP_ACTION* pdu)
{
    uint8_t tempData;
    uint16_t tempLen;
    uint8_t intfIdx;
    intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_NetDefaultGet())->netIfIx;

    // Fetch pdu data type
    tempData = TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt);
    if ( !IS_AGENT_PDU(tempData) )
        return false;

    *pdu = tempData;


	/* Now fetch pdu length.  We don't need to remember pdu length.
	   Do this to proceed to next pdu element of interest*/	
    return TCPIP_SNMP_LengthIsValid(&tempLen);
}

/****************************************************************************
Function:
    uint8_t TCPIP_SNMP_LengthIsValid(uint16_t* len)

Summary:
    Retrieves the packet length and actual pdu length.

Description:
    Checks current packet and returns total length value as well as
    actual length bytes.We do not support any length byte count of more
    than 2 i.e. total length value must not be more than 16-bit.

Precondition:
    None

Parameters:
    len - Pointer to memory where actual length is stored.

Return Values:
    lengthBytes	- Total length bytes are 0x80 itself plus tempData.

Remarks:
    None.
***************************************************************************/
uint8_t TCPIP_SNMP_LengthIsValid(uint16_t *len)
{
    uint8_t tempData;
    TCPIP_UINT16_VAL tempLen;
    uint8_t lengthBytes;
    uint8_t intfIdx;
    intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_NetDefaultGet())->netIfIx;
    // Initialize length value.
    tempLen.Val = 0;
    lengthBytes = 0;

    tempData = TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt);
    tempLen.v[0] = tempData;
    if ( tempData & 0x80 )
    {
        tempData &= 0x7F;

        // We do not support any length byte count of more than 2
        // i.e. total length value must not be more than 16-bit.
        if ( tempData > 2u )
            return false;

        // Total length bytes are 0x80 itself plus tempData.
        lengthBytes = tempData + 1;

        // Get upto 2 bytes of length value.
        while( tempData-- )
            tempLen.v[tempData] = TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt);
    }
    else
        lengthBytes = 1;

    *len = tempLen.Val;

    return lengthBytes;
}


/****************************************************************************
Function:
    bool TCPIP_SNMP_DataIsASNNull(void)


Summary:
    Verifies the value type as ASN_NULL.

Description:
    For Get,Get_Next,Get_Bulk snmp reuest, the var bind the value data type
    should be ASN_NULL and value field must be NULL and . This routine
    verifies the data type and value fields in the received requests.
    The SET request, the value data type can not be ASN_NULL,
    otherwise the snmp request is not processed.

Precondition:
    None

Parameters:
    None

Returns Values
    true	- If value type is ASN_NULL and value is NULL.
    false	- If data type and value is other than ASN_NULL and NULL resp.

Remarks:
    None.
***************************************************************************/
static bool TCPIP_SNMP_DataIsASNNull(void)
{
    uint8_t a;
    uint8_t intfIdx;
    intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_NetDefaultGet())->netIfIx;
		
    // Fetch and verify that this is NULL data type.
    /*if ( !IS_ASN_NULL(TCPIP_SNMP_ReadByte()) )
        return false;*/

    a=TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt);

    if (!IS_ASN_NULL(a))
        return false;

    // Fetch and verify that length value is zero.
    return (TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt) == 0u );
}


/****************************************************************************
Function:
    bool TCPIP_SNMP_OIDIsValid(uint8_t* oid, uint8_t* len)

Summary:
    Populates OID type, length and oid string from the received pdu.

Description:
    In this routine, OID data type "ASN_OID" is verified in the received pdu.
    If the data type is matched, then only var bind is processed. OID length
    and OID is populated. The max OID length can be 15.

Precondition:
    ProcessVariabels() is called.

Parameters:
    oid - Pointer to memory to store the received OID string
    len	- Pointer to memory to store OID length

Return Values:
    true	- If value type is ASN_OID and oid length not more than 15.
    false	- Otherwise.

Remarks:
    None.
***************************************************************************/
static bool TCPIP_SNMP_OIDIsValid(uint8_t* oid, uint8_t* len)
{
    TCPIP_UINT32_VAL tempLen;
    uint8_t intfIdx;
    intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_NetDefaultGet())->netIfIx;
	
    // Fetch and verify that this is OID.
    if ( !IS_OID(TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt)) )
        return false;

    // Retrieve OID length
    if ( !TCPIP_SNMP_LengthIsValid(&tempLen.w[0]) )
        return false;

    // Make sure that OID length is within our capability.
    if ( tempLen.w[0] > (uint8_t)SNMP_OID_MAX_LEN )
        return false;

    *len = tempLen.v[0];

    while( tempLen.v[0]-- )
    {
        *oid++ = TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt);
    }
    *oid=0xff;
    return true;
}

/****************************************************************************
Function:
    uint8_t TCPIP_SNMP_SNMPStructureIsValid(uint16_t* dataLen)

Summary:
    Decode variable length structure.

Description:
    This routine is used  to verify whether the received varbind is of type
    STRUCTURE and to find out the variable binding structure length.

Precondition:
    TCPIP_SNMP_ProcessPDUHeader() is called.

Parameters:
    datalen	- Pointer to memory to store OID structure length.

Return Values:
    headrbytes	- Variable binding length.
    false		- If variable data structure is not type STRUCTURE.

Remarks:
    None.
***************************************************************************/
uint8_t TCPIP_SNMP_SNMPStructureIsValid(uint16_t* dataLen)
{
    TCPIP_UINT32_VAL tempLen;
    uint8_t headerBytes;
    uint8_t intfIdx;
    intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_NetDefaultGet())->netIfIx;

		
    if ( !IS_STRUCTURE(TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt)) )
        return false;

    // Retrieve structure length
    headerBytes = TCPIP_SNMP_LengthIsValid(&tempLen.w[0]);
    if ( !headerBytes )
        return false;

    headerBytes++;

    // Since we are using UDP as our transport and UDP are not fragmented,
    // this structure length cannot be more than 1500 bytes.
    // As a result, we will only use lower uint16_t of length value.
    *dataLen = tempLen.w[0];

    return headerBytes;
}

/****************************************************************************
Function:
    bool TCPIP_SNMP_PDUProcessDuplexInit(UDP_SOCKET socket)

Summary:
    Prepare for full duplex transfer.

Description:
    As we process SNMP variables, we will prepare response on-the-fly
creating full duplex transfer. SNMP stack manages its software simulated full 
duplex connection. Prepare for full duplex transfer. Set the Tx and Rx 
offset to start of the buffer.

Precondition:
    TCPIP_SNMP_Task() is called.

Parameters:
    socket - An active udp socket for which tx and rx offset to be set.

Returns:
    true if success,
    false otherwise.

Remarks:
    This routine should be called for every new snmp packet received.
***************************************************************************/
bool TCPIP_SNMP_PDUProcessDuplexInit(UDP_SOCKET socket)
{
    // In full duplex transfer, transport protocol must be ready to
    // accept new transmit packet.
    if(TCPIP_UDP_TxPutIsReady(socket,SNMP_MAX_MSG_SIZE))
    {
        // Initialize buffer offsets.
        SNMPRxOffset = 0;
        SNMPTxOffset = 0;
        return true;
    }

    return false;

}


/****************************************************************************
Function:
    void TCPIP_SNMP_PutByte(uint8_t v)

Summary:
    Copy byte to tx buffer.

Description:
    This function writes a single byte to the currently active UDP socket,
    transmit buffer, while incrementing the buffer offset for the next write
    operation.

Precondition:
    TCPIP_SNMP_Task() is called.
    A active udp socket is availabe to tx from.

Parameters:
    None.

Returns:
    None.

Remarks:
    None.
***************************************************************************/
void TCPIP_SNMP_PutByte(UDP_SOCKET socket,uint8_t v)
{
    TCPIP_UDP_Put(socket, v);
    SNMPTxOffset++;
}

static void TCPIP_SNMP_PutDataArray(UDP_SOCKET socket,SNMP_BUFFER_DATA *snmpPutData)
{	
    TCPIP_UDP_ArrayPut(socket,snmpPutData->head,snmpPutData->length);
}

/****************************************************************************
Function:
    uint8_t TCPIP_SNMP_ReadByte(void)

Summary:
    Read byte from snmp udp socket rx buffer.

Description:
    This function reads a single byte from the currently active UDP socket,
    receive buffer, while incrementing the buffer offset from where the next
    byte will be read.

Precondition:
    TCPIP_SNMP_Task() is called.
    A active udp socket is available to read from.

Parameters:
    None

Returns:
    None.

Remarks:
    None.
***************************************************************************/
uint8_t TCPIP_SNMP_ReadByte(UDP_SOCKET skt)
{
    uint8_t v;
    uint8_t intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_NetDefaultGet())->netIfIx;
	
    if(!snmpDcpt[intfIdx].readFromSnmpBuf)
    {
        TCPIP_UDP_RxOffsetSet(skt, SNMPRxOffset++);
        TCPIP_UDP_Get(skt, &v);
    }
    else
    {
        v= (uint8_t)snmpV1V2cIncomingMsgBufPtr.snmpMsgHead[SNMPRxOffset++];
    }
    return v;
}


/****************************************************************************
Function:
    bool TCPIP_SNMP_OIDStringFindByID(int32_t fileDescr,SNMP_ID id, OID_INFO* info,
                                              uint8_t* oidString, uint8_t* len)

Summary:
    Get complete notification variable OID string from snmp.bib using var id.

Description:
    This routine is called when a OID string is required to be searched
    from snmp.bib using agent id. The string is saved with agent.
    TRAP pdu is send with this OID corresponding to the SNMP_ID used
    by the agent application to send the pdu.

Precondition:
    TCPIP_SNMP_Notify() is called.

Parameters:
	fileDescr - A valid File decriptor which is alread opened.
    id			-	System ID to use identify this agent.
    info		-	Pointer to SNMP MIB variable object information
    oidString	-	Pointer to store the string of OID serached
    len			-	Oid length

Return Values:
    true	-	If oid string is found for the variable id in snmp.bib.
    FLASE	-	Otherwise.

Remarks:
    This function is used only when TRAP is enabled.
***************************************************************************/
bool TCPIP_SNMP_OIDStringFindByID(int32_t fileDescr,SNMP_ID id, OID_INFO* info, uint8_t* oidString, uint8_t* len)
{
    uint32_t hCurrent;

    hCurrent = 0;

    while (1)
    {
    	//Read in the Mib record for the oid info
        TCPIP_SNMP_MIBRecordRead(fileDescr,hCurrent, info);

        if ( !info->nodeInfo.Flags.bIsParent )
        {
            if ( info->nodeInfo.Flags.bIsIDPresent )
            {
                if ( info->id == id )
                    return TCPIP_SNMP_OIDStringGetByAddr(fileDescr,info, oidString, len);
            }

            if ( info->nodeInfo.Flags.bIsSibling ||
                 info->nodeInfo.Flags.bIsDistantSibling )
            {
                SYS_FS_FileSeek(fileDescr, info->hSibling, SYS_FS_SEEK_SET);
            }
            else
                break;
        }
        hCurrent = SYS_FS_FileTell(fileDescr);
    }
    return false;
}



/****************************************************************************
Function:
    bool TCPIP_SNMP_OIDStringGetByAddr(int32_t fileDescr,OID_INFO* rec, uint8_t* oidString, uint8_t* len)

Summary:
    Get OID string from snmp.bib using the node address.

Description:
    This routine is called when a OID string is required to be searched
    from snmp.bib using node address.

Precondition:
    None.

Parameters:
	fileDescr - A valid File decriptor which is alread opened.
    rec			-	Pointer to SNMP MIB variable object information
    oidString	-	Pointer to store the string of OID searched
    len			-	Oid length

Return Values:
    true	-	If oid string is found.
    FLASE	-	Otherwise.

Remarks:
    None.
***************************************************************************/
bool TCPIP_SNMP_OIDStringGetByAddr(int32_t fileDescr,OID_INFO* rec, uint8_t* oidString, uint8_t* len)
{
    uint32_t hTarget;
    uint32_t hCurrent;
    uint32_t hNext;
    OID_INFO currentMIB;
    uint8_t index;
    enum { SM_PROBE_SIBLING, SM_PROBE_CHILD } state;

    hCurrent = 0;


    hTarget = rec->hNode;//node address
    state = SM_PROBE_SIBLING;
    index = 0;

    while( 1 )
    {
        TCPIP_SNMP_MIBRecordRead(fileDescr,hCurrent, &currentMIB);

        oidString[index] = currentMIB.oid;

        if ( hTarget == hCurrent )
        {
            *len = ++index;
            return true;
        }

        switch(state)
        {
            case SM_PROBE_SIBLING:
                if ( !currentMIB.nodeInfo.Flags.bIsSibling )
                    state = SM_PROBE_CHILD;
                else
                {
                    hNext = currentMIB.hSibling;

                    SYS_FS_FileSeek(fileDescr, hNext, SYS_FS_SEEK_SET);
                    hNext = SYS_FS_FileTell(fileDescr);

                    if ( hTarget >= hNext )
                    {
                        hCurrent = hNext;
                        break;
                    }
                    else
                        state = SM_PROBE_CHILD;
                }

            case SM_PROBE_CHILD:
                if ( !currentMIB.nodeInfo.Flags.bIsParent )
                    return false;

                index++;

                hCurrent = currentMIB.hChild;
                state = SM_PROBE_SIBLING;
                break;
        }
    }
    return false;
}


/****************************************************************************
Function:
    void TCPIP_SNMP_MIBRecordRead(int32_t fileDescr,uint32_t h, OID_INFO* rec)

Summary:
    Get OID string from snmp.bib using the node address.

Description:
    This routine is called when a OID string is required to be searched
    from snmp.bib using node address.

Precondition:
    TCPIP_SNMP_OIDStringFindByID() or TCPIP_SNMP_OIDStringGetByAddr() is called.

Parameters:
	fileDescr - A valid File decriptor which is alread opened.
    h		-	Node adderess whose oid is to be read.
    rec		-	Pointer to store SNMP MIB variable object information

Returns:
    None.

Remarks:
    None.
***************************************************************************/
static void TCPIP_SNMP_MIBRecordRead(int32_t fileDescr,uint32_t h, OID_INFO* rec)
{
    MIB_INFO nodeInfo;
    TCPIP_UINT16_VAL tempVal;
    uint8_t idLen=1;

    SYS_FS_FileSeek(fileDescr, h, SEEK_SET);

    rec->hNode = h;
    SYS_FS_FileRead(fileDescr,&rec->oid,1);
    SYS_FS_FileRead(fileDescr,&rec->nodeInfo.Val,1);
    nodeInfo = rec->nodeInfo;
    if ( nodeInfo.Flags.bIsIDPresent )
    {
        SYS_FS_FileRead(fileDescr,&idLen,1);
        if(idLen == 1)
        {
            uint8_t temp=0;
            SYS_FS_FileRead(fileDescr,&temp,1);
            rec->id = temp & 0xFF;
        }
        else if(idLen == 2)
        {
            uint8_t temp[2];
            SYS_FS_FileRead(fileDescr,temp,2);
            rec->id = 0;
            rec->id = temp[0] & 0xFF;
            rec->id <<= 8;
            rec->id |= temp[1] & 0xFF;
        }
    }
    if ( nodeInfo.Flags.bIsSibling )
    {
        SYS_FS_FileRead(fileDescr,&tempVal.v[0],1);
        SYS_FS_FileRead(fileDescr,&tempVal.v[1],1);
        rec->hSibling = tempVal.Val;
    }
    if ( nodeInfo.Flags.bIsParent )
    {
       rec->hChild = SYS_FS_FileTell(fileDescr);
    }
    else
    {
        if ( nodeInfo.Flags.bIsDistantSibling )
        {
            SYS_FS_FileRead(fileDescr,&tempVal.v[0],1);
            SYS_FS_FileRead(fileDescr,&tempVal.v[1],1);
            rec->hSibling = tempVal.Val;
        }
        rec->dataType = 0;
        SYS_FS_FileRead(fileDescr,(uint8_t*)&rec->dataType,1);
        rec->hData = SYS_FS_FileTell(fileDescr);
    }
}

bool TCPIP_SNMP_DataTypeInfoGet(SNMP_DATA_TYPE dataType, SNMP_DATA_TYPE_INFO *info )
{
    if ( dataType >= DATA_TYPE_UNKNOWN )
    {
        info->asnType   = 0x00;
        info->asnLen    = 0x00;
        return false;
    }

    info->asnType   = dataTypeTable[dataType].asnType;
    info->asnLen    = dataTypeTable[dataType].asnLen;

    return true;
}

/****************************************************************************
Function:
    uint8_t TCPIP_SNMP_ProcessSetVar(PDU_INFO* pduDbPtr,OID_INFO* rec,
                                       SNMP_ERR_STATUS* errorStatus)

Summary:
    Processes snmp Set request pdu.

Description:
    This routine processes the received snmp set request pdu for the
    variable binding in the request and also creates the response pdu.

Precondition:
    TCPIP_SNMP_ProcessVariables() is called.

Parameters:
pduDbPtr	-   Received pdu information database pointer
rec		  	-   Pointer to SNMP MIB variable object information
errorStatus -   Pointer to update error status info

Return Values:
    copiedBytes	- Number of bytes copied by this routine to the
                              snmp pdu tx buffer.

Remarks:
    None.
***************************************************************************/
uint8_t TCPIP_SNMP_ProcessSetVar(PDU_INFO* pduDbPtr,OID_INFO* rec, SNMP_ERR_STATUS* errorStatus)
{
    uint8_t ref;
    uint8_t temp=0;
    uint8_t dataType=0;
    uint8_t dataLen=0;
    uint8_t copiedBytes=0;
    SNMP_ERR_STATUS errorCode;
    SNMP_DATA_TYPE_INFO actualDataTypeInfo;
    SNMP_VAL dataValue;
    uint8_t intfIdx;
    SNMP_BUFFER_DATA *snmpPutData = NULL;

#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
    SNMPV3MSGDATA	*dynPduBuf=NULL;

    SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr;
    SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0;

    TCPIP_SNMPV3_PacketProcStubPtrsGet(&snmpv3PktProcessingMemPntr);

    snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;
    dynPduBuf = &snmpv3EngnDcptMemoryStubPtr->ScopedPduRespnsBuf;
#endif

    intfIdx = ((TCPIP_NET_IF*)TCPIP_STACK_NetDefaultGet())->netIfIx;
    snmpPutData = &SnmpStackDcptMemStubPtr->outPduBufData;
	
    // Start with no error.
    errorCode = SNMP_NO_ERR;
    copiedBytes = 0;

	

    // Non-leaf, Constant and ReadOnly node cannot be modified
    if(rec->nodeInfo.Flags.bIsParent   ||
       rec->nodeInfo.Flags.bIsConstant ||
       !rec->nodeInfo.Flags.bIsEditable )
    {	
        if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
            errorCode = SNMP_NO_SUCH_NAME;
        else if ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C) ||
                    (pduDbPtr->snmpVersion == (uint8_t)SNMP_V3))
            errorCode = SNMP_NOT_WRITABLE;
    }

    if(pduDbPtr->snmpVersion != (uint8_t)SNMP_V3)
    {
        dataType = TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt);
        TCPIP_SNMP_DataCopyToProcessBuffer(dataType,snmpPutData);
    }
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
    else
    {
        dataType = TCPIP_SNMPv3_ProcessBuffDataGet(snmpv3EngnDcptMemoryStubPtr->ScopedPduRequstBuf,++snmpv3EngnDcptMemoryStubPtr->ScopedPduDataPos);
        TCPIP_SNMPv3_DataCopyToProcessBuff(dataType,dynPduBuf);
    }
#endif
    copiedBytes++;

    // Get data type for this node.
    if ( !TCPIP_SNMP_DataTypeInfoGet(rec->dataType, &actualDataTypeInfo) )
    {
        if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
            errorCode = SNMP_BAD_VALUE;
        else if ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C) ||
                 (pduDbPtr->snmpVersion == (uint8_t) SNMP_V3))
            errorCode = SNMP_WRONG_TYPE;
    }

    // Make sure that received data type is same as what is declared
    // for this node.
    if ( dataType != actualDataTypeInfo.asnType )
    {
        if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
            errorCode = SNMP_BAD_VALUE;
        else if ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C) ||
                        (pduDbPtr->snmpVersion == (uint8_t) SNMP_V3))
            errorCode = SNMP_WRONG_TYPE;
    }

    // Make sure that received data length is within our capability.
    if(pduDbPtr->snmpVersion != (uint8_t) SNMP_V3)
    {
        dataLen = TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt);
        TCPIP_SNMP_DataCopyToProcessBuffer(dataLen,snmpPutData);
    }
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
    else
    {
        dataLen = TCPIP_SNMPv3_ProcessBuffDataGet(snmpv3EngnDcptMemoryStubPtr->ScopedPduRequstBuf,++snmpv3EngnDcptMemoryStubPtr->ScopedPduDataPos);
        TCPIP_SNMPv3_DataCopyToProcessBuff(dataLen,dynPduBuf);
    }
#endif
    copiedBytes++;

    // Only max data length of 127 is supported.
    if ( dataLen > 0x7fu )
    {
        if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
            errorCode = SNMP_BAD_VALUE;
        else if ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C)||
                    (pduDbPtr->snmpVersion == (uint8_t) SNMP_V3))
            errorCode = SNMP_WRONG_LENGTH;
    }

    // If this is a Simple variable and given index is other than '0',
    // it is considered bad value
    if ( !rec->nodeInfo.Flags.bIsSequence && rec->index != 0x00u ){
        errorCode = SNMP_NO_SUCH_NAME;}

    dataValue.dword = 0;
    ref = 0;

    // If data length is within 4 bytes, fetch all at once and pass it
    // to application.
    if ( actualDataTypeInfo.asnLen != 0xff )
    {
        // According to mib def., this data length for this data type/
        // must be less or equal to 4, if not, we don't know what this
        // is.
        if ( dataLen <= 4u )
        {
            // Now that we have verified data length, fetch them all
            // at once and save it in correct place.
            //dataLen--;

            while( dataLen-- )
            {
                if(pduDbPtr->snmpVersion != (uint8_t) SNMP_V3)
                {
                    temp = TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt);
	                // Copy same byte back to create response...
                    TCPIP_SNMP_DataCopyToProcessBuffer(temp,snmpPutData);
                }
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
                else
                {
                    temp = TCPIP_SNMPv3_ProcessBuffDataGet(snmpv3EngnDcptMemoryStubPtr->ScopedPduRequstBuf,++snmpv3EngnDcptMemoryStubPtr->ScopedPduDataPos);
                    TCPIP_SNMPv3_DataCopyToProcessBuff(temp,dynPduBuf);
                }
#endif
                dataValue.v[dataLen] = temp;
                copiedBytes++;
            }


            // Pass it to application.
            if ( errorCode == SNMP_NO_ERR )
            {
                if(!TCPIP_SNMP_VarbindSet(rec->id, rec->index, ref, dataValue))
                {
                    if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
                        errorCode = SNMP_BAD_VALUE;
                    else if ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C) ||
                                (pduDbPtr->snmpVersion == (uint8_t) SNMP_V3))
                        errorCode = SNMP_WRONG_VALUE;
                }
            }
        }
        else
        {
            if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
                errorCode = SNMP_BAD_VALUE;
            else if ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C)||
                    (pduDbPtr->snmpVersion == (uint8_t) SNMP_V3))
            {
                if( rec->nodeInfo.Flags.bIsConstant)
                    errorCode = SNMP_NOT_WRITABLE;
                else
                    errorCode = SNMP_WRONG_LENGTH;
            }
        }
    }
    else
    {
        // This is a multi-byte Set operation.
        // Check with application to see if this many bytes can be
        // written to current variable.      
        if ( !TCPIP_SNMP_IsValidLength(rec->id, dataLen,rec->index) )
        {
            if(pduDbPtr->snmpVersion == (uint8_t)SNMP_V1)
                errorCode = SNMP_BAD_VALUE;
            else if ((pduDbPtr->snmpVersion == (uint8_t)SNMP_V2C)  ||
                    (pduDbPtr->snmpVersion == (uint8_t) SNMP_V3))
            {
                if( rec->nodeInfo.Flags.bIsConstant)
                    errorCode = SNMP_NOT_WRITABLE;
                else
                    errorCode = SNMP_WRONG_LENGTH;
            }
        }
        // Even though there may have been error processing this
        // variable, we still need to reply with original data
        // so at least copy those bytes.
        while( dataLen-- )
        {
            if(pduDbPtr->snmpVersion != (uint8_t) SNMP_V3)
            {
                dataValue.byte = TCPIP_SNMP_ReadByte(snmpDcpt[intfIdx].skt);
                TCPIP_SNMP_DataCopyToProcessBuffer(dataValue.byte,snmpPutData);
            }
            #ifdef TCPIP_STACK_USE_SNMPV3_SERVER
            else
            {
                dataValue.byte = TCPIP_SNMPv3_ProcessBuffDataGet(snmpv3EngnDcptMemoryStubPtr->ScopedPduRequstBuf,++snmpv3EngnDcptMemoryStubPtr->ScopedPduDataPos);
                TCPIP_SNMPv3_DataCopyToProcessBuff(dataValue.byte,dynPduBuf);
            }
            #endif
            copiedBytes++;

            // Ask applicaton to set this variable only if there was
            // no previous error.
            if ( errorCode == SNMP_NO_ERR )
            {
                if ( !TCPIP_SNMP_VarbindSet(rec->id, rec->index, ref++, dataValue) )
                {
                    errorCode = SNMP_BAD_VALUE;
                }
            }
        }
        // Let application know about end of data transfer
        if ( errorCode == SNMP_NO_ERR )
        {
            if(!TCPIP_SNMP_VarbindSet(rec->id, rec->index, (uint16_t)SNMP_END_OF_VAR, dataValue))
            {
                errorCode = SNMP_BAD_VALUE;
            }
        }
    }
    *errorStatus = errorCode;

    return copiedBytes;
}


/****************************************************************************
Function:
    uint8_t TCPIP_SNMP_ProcessGetVar(OID_INFO* rec, bool bAsOID,PDU_INFO* pduDbPtr)

Summary:
    Processes snmp Get request pdu.

Description:
    This routine processes the received snmp Get request pdu for the
    variable binding in the request and also creates the response pdu.

Precondition:
    TCPIP_SNMP_ProcessVariables() is called.

Parameters:
rec		 -   Pointer to SNMP MIB variable object information
bAsOID	 -   Oid flag.

Return Values:
    varLen	- Number of bytes put in response tx pdu
    false	- If any of the elements of the request pdu validation fails.

Remarks:
    None.
***************************************************************************/
uint8_t TCPIP_SNMP_ProcessGetVar(OID_INFO* rec, bool bAsOID,PDU_INFO* pduDbPtr)
{
    uint8_t ref;
    uint8_t temp;
    uint8_t varLen;
    uint8_t dataType;
    uint16_t offset;
    uint16_t prevOffset;
    uint16_t initPutDatalengthOffset=0;
    SNMP_VAL v;
    SNMP_DATA_TYPE_INFO dataTypeInfo;
    SNMP_BUFFER_DATA *snmpPutData = NULL;
	
	 
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	

    SNMPV3MSGDATA	*dynPduBuf=NULL;
    SNMPV3_PROCESSING_MEM_INFO_PTRS snmpv3PktProcessingMemPntr;
    SNMPV3_STACK_DCPT_STUB * snmpv3EngnDcptMemoryStubPtr=0;
						 
    TCPIP_SNMPV3_PacketProcStubPtrsGet(&snmpv3PktProcessingMemPntr);

    snmpv3EngnDcptMemoryStubPtr=snmpv3PktProcessingMemPntr.snmpv3StkProcessingDynMemStubPtr;
    dynPduBuf = &snmpv3EngnDcptMemoryStubPtr->ScopedPduRespnsBuf;
		
#endif
	
    snmpPutData = &SnmpStackDcptMemStubPtr->outPduBufData;
    initPutDatalengthOffset = snmpPutData->length;
    offset = 0;	// Suppress C30 warning: 'offset' may be used uninitialized in this function
    v.dword   = 0;

    // Non-leaf node does not contain any data.
    if ( rec->nodeInfo.Flags.bIsParent )
        return false;

    // If current OID is Simple variable and index is other than .0
    // we don't Get this variable.
    if ( !rec->nodeInfo.Flags.bIsSequence )
    {
        // index of other than '0' is not invalid.
        if ( rec->index > 0u )
        {
            snmpPutData->length = initPutDatalengthOffset;
            return false;
        }
    }

    dataType = rec->dataType;
    if ( !TCPIP_SNMP_DataTypeInfoGet(dataType, &dataTypeInfo) )
    {
        snmpPutData->length = initPutDatalengthOffset;
        return false;
    }

    if ( !bAsOID )
    {
        if(pduDbPtr->snmpVersion != (uint8_t) SNMP_V3)
        {
            if(TCPIP_SNMP_DataCopyToProcessBuffer(dataTypeInfo.asnType,snmpPutData)!=true)
                return false;

            offset = snmpPutData->length;//SNMPTxOffset;
            if(TCPIP_SNMP_DataCopyToProcessBuffer(dataTypeInfo.asnLen,snmpPutData)!=true)
                return false;
        }
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
        else
        {
            if(TCPIP_SNMPv3_DataCopyToProcessBuff(dataTypeInfo.asnType,dynPduBuf) != true)
                return false;
            offset = dynPduBuf->length;
            if(TCPIP_SNMPv3_DataCopyToProcessBuff(dataTypeInfo.asnLen,dynPduBuf)!= true)
                return false;
        }
#endif
    }

    if ( rec->nodeInfo.Flags.bIsConstant )
    {
        uint8_t c;

        SYS_FS_FileSeek(snmpFileDescrptr, rec->hData, SEEK_SET);
        SYS_FS_FileRead(snmpFileDescrptr,&varLen,1);
        temp = varLen;
        while( temp-- )
        {
            SYS_FS_FileRead(snmpFileDescrptr,&c,1);
            if(pduDbPtr->snmpVersion != (uint8_t) SNMP_V3)
            {
                if(TCPIP_SNMP_DataCopyToProcessBuffer(c,snmpPutData)!= true)
                    return false;
            }
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
            else
                if(TCPIP_SNMPv3_DataCopyToProcessBuff(c,dynPduBuf) != true)
                    return false;
#endif	
        }
    }
    else
    {
        ref = SNMP_START_OF_VAR;
        v.dword = 0;
        varLen = 0;

        do
        {
            if ( TCPIP_SNMP_VarbindGet(rec->id, rec->index, &ref, &v) )
            {
                if ( dataTypeInfo.asnLen != 0xff )
                {
                    varLen = dataTypeInfo.asnLen;

                    while( dataTypeInfo.asnLen )
                    {
                        if(pduDbPtr->snmpVersion != (uint8_t) SNMP_V3)
                        {
                            if(TCPIP_SNMP_DataCopyToProcessBuffer(v.v[--dataTypeInfo.asnLen],snmpPutData)!= true)
                                return false;
                        }
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
                        else
                            if(TCPIP_SNMPv3_DataCopyToProcessBuff(v.v[--dataTypeInfo.asnLen],dynPduBuf)!=true)
                                return false;
#endif
                    }
                    break;
                }
                else
                {
                    if(ref == SNMP_END_OF_VAR)
                        break;
                    varLen++;
                    if(pduDbPtr->snmpVersion != (uint8_t) SNMP_V3)
                    {
                        if(TCPIP_SNMP_DataCopyToProcessBuffer(v.v[0],snmpPutData)!= true)
                            return false;
                    }
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER
                    else
                        if(TCPIP_SNMPv3_DataCopyToProcessBuff(v.v[0],dynPduBuf) != true)
                            return false;
#endif
                }
            }
            else
            {
                snmpPutData->length = initPutDatalengthOffset;
            	return false;
            }

        } while( ref != SNMP_END_OF_VAR );
    }

    if ( !bAsOID )
    {
         if(pduDbPtr->snmpVersion != SNMP_V3)
         {
            prevOffset = snmpPutData->length;
            snmpPutData->length = offset;
            TCPIP_SNMP_DataCopyToProcessBuffer(varLen,snmpPutData);
            snmpPutData->length = prevOffset;
         }
#ifdef TCPIP_STACK_USE_SNMPV3_SERVER	
         else
        {
            prevOffset = dynPduBuf->length;
            dynPduBuf->length = offset;
            TCPIP_SNMPv3_DataCopyToProcessBuff(varLen,dynPduBuf);
            dynPduBuf->length = prevOffset;
        }
#endif
        varLen++;
        varLen++;
    }
    return varLen;
}


/****************************************************************************
Function:
    void TCPIP_SNMP_ErrorStatusSet(uint16_t errorStatusOffset,
                       uint16_t errorIndexOffset,
                       SNMP_ERR_STATUS errorStatus,
                       uint8_t errorIndex)
Summary:
    Set snmp error status in the response pdu.

Description:
    This routine processes the received snmp Get request pdu for the
    variable binding in the request and also creates the response pdu.

Precondition:
    TCPIP_SNMP_ProcessVariables() is called.

Parameters:
    errorStatusOffset - Offset to update error status in Response Tx pdu
    errorIndexOffset  - Offset to update error index
    errorStatus		  - Snmp process error to be updated in response.
    errorIndex		  - Index of the request varbind in the var bind list
                                    for which error status is to be updated.

Returns:
    None.

Remarks:
    None.
***************************************************************************/
void TCPIP_SNMP_ErrorStatusSet(uint16_t errorStatusOffset,
                           uint16_t errorIndexOffset,
                           SNMP_ERR_STATUS errorStatus,
                           uint8_t errorIndex,SNMP_BUFFER_DATA *snmpPutTxData)
{
    uint16_t prevOffset;

    prevOffset = snmpPutTxData->length;
    snmpPutTxData->length = errorStatusOffset;
    TCPIP_SNMP_DataCopyToProcessBuffer(errorStatus,snmpPutTxData);

    
    snmpPutTxData->length = errorIndexOffset;
    TCPIP_SNMP_DataCopyToProcessBuffer(errorIndex,snmpPutTxData);

    snmpPutTxData->length = prevOffset;
}


/****************************************************************************
Function:
    uint8_t TCPIP_SNMP_OIDsCountGet(uint16_t pdulen)

Summary:
    Finds number of varbinds in the varbind list received in a pdu.

Description:
    This routine is used to find the number of OIDs requested in the received
    snmp pdu.

Precondition	:
    TCPIP_SNMP_ProcessVariables() is called.

Parameters:
    pdulen		-	Length of snmp pdu request received.

Return Values:
    varCount	-	Number of OIDs found in a pdu request.

Remarks:
    None.
***************************************************************************/
static uint8_t TCPIP_SNMP_OIDsCountGet(uint16_t pdulen)
{
    uint8_t  structureLen;
    uint8_t varCount=0;
    uint16_t prevUDPRxOffset;
    uint16_t varBindLen;
    uint16_t snmpPduLen;
 	
    snmpPduLen=pdulen;

    prevUDPRxOffset=SNMPRxOffset;

    while(snmpPduLen)
    {
        structureLen = TCPIP_SNMP_SNMPStructureIsValid(&varBindLen);
        if(!structureLen)
            return false;
	

        SNMPRxOffset=SNMPRxOffset+varBindLen;
        varCount++;
        snmpPduLen=snmpPduLen
                - structureLen // 1 byte for STRUCTURE identifier + 0x82 or ox81+1+1 byte(s) for varbind length
                - varBindLen;
					//-1 //1 byte for STRUCTURE identifier
					//-1//1 byte for varbind length 
				//	-varBindLen;
    }

    SNMPRxOffset=prevUDPRxOffset;

    return varCount;
}

bool  TCPIP_SNMP_WriteCommunityGet(int index,int len, uint8_t * dest)
{
    int minLen=0;
    int commLen=0;

    if((dest == NULL) || (len==0))
        return false;

    if(len>SNMP_COMMUNITY_MAX_LEN)
        return false;

    if(index >= SNMP_MAX_COMMUNITY_SUPPORT)
        return false;

    commLen = strlen((char*)SnmpStackDcptMemStubPtr->snmpNetConfig.writeCommunity[index]);
    minLen= len<commLen?len:commLen;

    strncpy((char*)dest,(char*)SnmpStackDcptMemStubPtr->snmpNetConfig.writeCommunity[index],minLen);

    return true;
}

bool  TCPIP_SNMP_ReadCommunityGet(int index,int len, uint8_t * dest)
{
    int minLen=0;
    int commLen=0;

    if((dest == NULL) || (len==0))
        return false;

    if(len>SNMP_COMMUNITY_MAX_LEN)
        return false;

    if(index >= SNMP_MAX_COMMUNITY_SUPPORT)
        return false;

    commLen = strlen((char*)SnmpStackDcptMemStubPtr->snmpNetConfig.readCommunity[index]);
    minLen= len<commLen?len:commLen;

    strncpy((char*)dest,(char*)SnmpStackDcptMemStubPtr->snmpNetConfig.readCommunity[index],minLen);

    return true;
}


/****************************************************************************
Function:
    static bool TCPIP_SNMP_DefaultUserConfigLoad(void)

Summary:
    Configure SNMP Default configuration.

Description:
    Initialize SNMP default Communities and other snmp parameters.

Precondition	:
    This is called from TCPIP_SNMP_Initialize().
Parameters:
    void
Return Values:
    bool
Remarks:
    None.
***************************************************************************/

static bool TCPIP_SNMP_DefaultUserConfigLoad(void)
{
   uint8_t i;
   static const char * const cReadCommunities[] = SNMP_READ_COMMUNITIES;
   static const char * const cWriteCommunities[] = SNMP_WRITE_COMMUNITIES;
   const char * strCommunity;

   for(i = 0; i < SNMP_MAX_COMMUNITY_SUPPORT; i++)
   {
       // Get a pointer to the next community string
       strCommunity = cReadCommunities[i];
       if(i >= sizeof(cReadCommunities)/sizeof(cReadCommunities[0]))
           strCommunity = "";

       // Ensure we don't buffer overflow.	If your code gets stuck here,
       // it means your SNMP_COMMUNITY_MAX_LEN definition in tcpip_config.h
       // is either too small or one of your community string lengths
       // (SNMP_READ_COMMUNITIES) are too large.  Fix either.
       if(strlen(strCommunity) >= sizeof(SnmpStackDcptMemStubPtr->snmpNetConfig.readCommunity[0]))
           return false;

// Copy string into tcpipNetIf
       strcpy((char*)SnmpStackDcptMemStubPtr->snmpNetConfig.readCommunity[i], strCommunity);

       // Get a pointer to the next community string
       strCommunity = cWriteCommunities[i];
       if(i >= sizeof(cWriteCommunities)/sizeof(cWriteCommunities[0]))
           strCommunity = "";

       // Ensure we don't buffer overflow.	If your code gets stuck here,
       // it means your SNMP_COMMUNITY_MAX_LEN definition in tcpip_config.h
       // is either too small or one of your community string lengths
       // (SNMP_WRITE_COMMUNITIES) are too large.  Fix either.
       if(strlen(strCommunity) >= sizeof(SnmpStackDcptMemStubPtr->snmpNetConfig.writeCommunity[0]))
           return false;

        // Copy string into tcpipNetIf
        strcpy((char*)SnmpStackDcptMemStubPtr->snmpNetConfig.writeCommunity[i], strCommunity);
    }
    return true;
}
/****************************************************************************
Function:
    bool TCPIP_SNMP_PvtMIBObjIsRequested(uint8_t* OIDValuePtr)

Summary:
    To find whether requested OID is only for private access.

Description:
    This routine is used to find whether requested object belongs
    to the private object group of the mib of agent. If yes, then
    that mib object can be accessed only with private community
    (supported in SNMPv2c).

Precondition	:
    TCPIP_SNMP_ProcessVariables() is called.

Parameters:
    OIDValuePtr	-	Pointer to memory stored with received OID.

Return Values:
    true	-	If the requested object is of private branch of the mib.
    FLASE	-	If the requested object is publically accessible.

Remarks:
    None.
***************************************************************************/
static bool TCPIP_SNMP_PvtMIBObjIsRequested(uint8_t* OIDValuePtr)
{
    uint8_t cnt=0;
    uint8_t pvtObjIdentifier[4]={0x2b,0x06/*dod*/,0x01/*internet*/,0x04/*private*/};

    while(cnt<4u)
    {
        //check whether requested oid is for pvt obj
        if(pvtObjIdentifier[cnt]== OIDValuePtr[cnt])
        {
            cnt++;
        }
        else
        {
            cnt=0;
            return false;
        }
        if(cnt == 0x04u)
            return true;
    }
    return false;

}

/*
* Set the File Descriptor
*/
void TCPIP_SNMP_FileDescrSet(int32_t fileDescr)
{
    snmpFileDescrptr = fileDescr;
}

/*
*  Get The File Descriptor
*/
int32_t TCPIP_SNMP_FileDescrGet(void)
{
    return snmpFileDescrptr;
}

void TCPIP_SNMP_TrapSpecificNotificationGet(uint8_t *specTrap)
{
    SNMP_PROCESSING_MEM_INFO_PTRS snmpPktProcsMemPtrsInfo;
    SNMP_STACK_DCPT_STUB * snmpStkDcptMemStubPtr=0;

    TCPIP_SNMP_PacketProcStubPtrsGet(&snmpPktProcsMemPtrsInfo);

    snmpStkDcptMemStubPtr=snmpPktProcsMemPtrsInfo.snmpStkDynMemStubPtr;
    *specTrap = snmpStkDcptMemStubPtr->gSpecificTrapNotification;
}

void TCPIP_SNMP_TrapSpecificNotificationSet(uint8_t specTrap,uint8_t genTrap, SNMP_ID trapID)
{
    SNMP_PROCESSING_MEM_INFO_PTRS snmpPktProcsMemPtrsInfo;
    SNMP_STACK_DCPT_STUB * snmpStkDcptMemStubPtr=0;

    TCPIP_SNMP_PacketProcStubPtrsGet(&snmpPktProcsMemPtrsInfo);

    snmpStkDcptMemStubPtr=snmpPktProcsMemPtrsInfo.snmpStkDynMemStubPtr;
    snmpStkDcptMemStubPtr->gSpecificTrapNotification = specTrap;
    snmpStkDcptMemStubPtr->gGenericTrapNotification = genTrap;
#if defined(SNMP_STACK_USE_V2_TRAP)   
    snmpStkDcptMemStubPtr->SNMPNotifyInfo.trapIDVar = trapID;
#endif   
}

void TCPIP_SNMP_TrapInterFaceSet(TCPIP_NET_HANDLE netIntf)
{
   SnmpStackDcptMemStubPtr->SNMPNotifyInfo.snmpTrapInf = netIntf;
}
void TCPIP_SNMP_AuthTrapFlagSet(bool sendTrap)
{
   SnmpStackDcptMemStubPtr->gSendTrapFlag =  sendTrap;
}

void TCPIP_SNMP_AuthTrapFlagGet(bool *authTrap)
{
   *authTrap = SnmpStackDcptMemStubPtr->gSendTrapFlag;
}
#if defined(SNMP_STACK_USE_V2_TRAP)
void TCPIP_SNMP_TrapSendFlagSet(bool trapNotify)
{
   SnmpStackDcptMemStubPtr->gSetTrapSendFlag =  trapNotify;
}

void TCPIP_SNMP_TrapSendFlagGet(bool *trapNotify)
{
   *trapNotify = SnmpStackDcptMemStubPtr->gSetTrapSendFlag;
}
#endif
void TCPIP_SNMP_TrapSocketIDGet(UDP_SOCKET *socket)
{
    *socket = SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket;
}

void TCPIP_SNMP_TrapSocketIDSet(UDP_SOCKET socket)
{
    SnmpStackDcptMemStubPtr->SNMPNotifyInfo.socket =  INVALID_UDP_SOCKET;
}

void TCPIP_SNMP_TRAPMibIDGet(uint32_t *mibID)
{
    *mibID = SnmpStackDcptMemStubPtr->gOIDCorrespondingSnmpMibID;
}

void TCPIP_SNMP_MibIDSet(uint32_t mibID)
{
    SnmpStackDcptMemStubPtr->gOIDCorrespondingSnmpMibID = mibID;
}
#if defined (TCPIP_STACK_USE_IPV6)
static void SNMP_IPV6_Notify(TCPIP_NET_HANDLE hNet, uint8_t evType, const void * param)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(hNet);
    snmpDcpt[pNetIf->netIfIx].ipv6EventType = evType;
}

/*
* Return IPv6  Notification 
*/

IPV6_EVENT_TYPE TCPIP_SNMP_EventNotifyGet(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNet(hNet);
    return snmpDcpt[pNetIf->netIfIx].ipv6EventType;
}

#endif


#endif //#if defined(TCPIP_STACK_USE_SNMP_SERVER)

