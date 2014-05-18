/*******************************************************************************
  Domain Name System (DNS) Server dummy

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    - Acts as a DNS server, but gives out the local IP address for all 
      queries to force web browsers to access the board.
    - Reference: RFC 1034 and RFC 1035
*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************
File Name:  dnss.c
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
//DOM-IGNORE-END

#define DNSS_C

#include "tcpip/src/tcpip_private.h"
#include "tcpip/src/dnss_private.h"
#include "tcpip/src/hash_fnv.h"

#if defined(TCPIP_STACK_USE_DNS_SERVER)

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_DNS_SERVER

static const DNSS_MODULE_CONFIG dnssDefaultConfigData = 
{
    true,
    DNSS_REPLY_BOARD_ADDR,
    DNSS_CACHE_MAX_SERVER_ENTRIES,
    DNSS_CACHE_PER_IPV4_ADDRESS,
#ifdef TCPIP_STACK_USE_IPV6
    DNSS_CACHE_PER_IPV6_ADDRESS,
#endif	
};

static DNSS_DCPT gDnsSrvDcpt={0,INVALID_UDP_SOCKET ,DNSS_STATE_START,0};

static void _DNSCopyRXNameToTX(UDP_SOCKET s);
static  TCPIP_DNS_RESULT  _DNSSUpdateHashEntry( DNSS_HASH_ENTRY *dnsSHE,TCPIP_DNSS_CACHE_ENTRY dnssCacheEntry);
static  TCPIP_DNS_RESULT  _DNSSSetHashEntry( DNSS_HASH_ENTRY_FLAGS newFlags,TCPIP_DNSS_CACHE_ENTRY dnssCacheEntry);
static void _DNSSGetRecordType(UDP_SOCKET s,TCPIP_UINT16_VAL *recordType);
static  void _DNSSRemoveCacheEntries(void);
// Server Need to parse the incoming hostname from client . replace Len with dot
static uint8_t hostNameWithDot[DNSS_HOST_NAME_LEN+1]={0};
static uint16_t countWithDot=0;

// Server Need to parse the incoming hostname from client . keep Len and this array will be 
//used while transmitting Name Server response packet
static uint8_t hostNameWithLen[DNSS_HOST_NAME_LEN+1]={0}; 
static uint16_t countWithLen=0;

static uint8_t  dnsSrvRecvByte[64+1]={0};
// DNS server received buffer position
static uint32_t gDnsSrvBytePos=0;


bool TCPIP_DNSS_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const DNSS_MODULE_CONFIG* pDnsSConfig)
{
    DNSS_DCPT		*pDnsSDcpt;
    OA_HASH_DCPT	*hashDcpt;
    size_t			hashMemSize;

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {	// interface restart

        if(stackCtrl->pNetIf->Flags.bIsDNSServerAuto != 0)
        {	// override the pDhcpsConfig->dhcpEnable passed with the what the stack manager says
            TCPIP_DNSS_Enable(stackCtrl->pNetIf);
        }
        return true;
    }

    pDnsSDcpt = &gDnsSrvDcpt;

    if(pDnsSConfig == 0)
    {
        pDnsSConfig =  &dnssDefaultConfigData;
    }
    if(pDnsSDcpt->dnsSrvInitCount==0)
    {
        pDnsSDcpt->memH  = stackCtrl->memH;

        if(pDnsSDcpt->dnssHashDcpt == 0)
        {
            hashMemSize = sizeof(OA_HASH_DCPT) + pDnsSConfig->cacheEntries * sizeof(DNSS_HASH_ENTRY);
            hashDcpt = (OA_HASH_DCPT*)TCPIP_HEAP_Malloc(pDnsSDcpt->memH,hashMemSize);
            if(hashDcpt == 0)
            {	// failed
                return false;
            }

            // populate the entries
            hashDcpt->memBlk = hashDcpt + 1;
            hashDcpt->hParam = hashDcpt;	// store the descriptor it belongs to
            hashDcpt->hEntrySize = sizeof(DNSS_HASH_ENTRY);
            hashDcpt->hEntries = pDnsSConfig->cacheEntries;
            hashDcpt->probeStep = TCPIP_DNSS_HASH_PROBE_STEP;

            hashDcpt->hashF = TCPIP_OAHASH_DNSS_KeyHash;
            hashDcpt->delF = TCPIP_OAHASH_DNSS_EntryDelete;
            hashDcpt->cmpF = TCPIP_OAHASH_DNSS_KeyCompare;
            hashDcpt->cpyF = TCPIP_OAHASH_DNSS_KeyCopy;
#if defined(OA_DOUBLE_HASH_PROBING)
            hashDcpt->probeHash = TCPIP_OAHASH_DNSS_ProbeHash;
#endif  // defined(OA_DOUBLE_HASH_PROBING)

            TCPIP_OAHASH_Initialize(hashDcpt);
            pDnsSDcpt->dnssHashDcpt = hashDcpt;
            pDnsSDcpt->flags.Val = 0;

            pDnsSDcpt->IPv4EntriesPerDNSName= pDnsSConfig->IPv4EntriesPerDNSName;
#ifdef TCPIP_STACK_USE_IPV6
            pDnsSDcpt->IPv6EntriesPerDNSName = pDnsSConfig->IPv6EntriesPerDNSName;
#endif
        }
        pDnsSDcpt->dnsSrvSocket = INVALID_UDP_SOCKET;
        pDnsSDcpt->smState = DNSS_STATE_START;
        pDnsSDcpt->replyWithBoardInfo = pDnsSConfig->replyBoardAddr;
        pDnsSDcpt->dnsSrvInitCount++;


        if(pDnsSDcpt->dnsSTimerHandle == 0)
        {	// once per service
            pDnsSDcpt->dnsSTimerHandle = _TCPIPStackAsyncHandlerRegister(TCPIP_DNSS_CacheTimeTask,0,DNSS_TASK_PROCESS_RATE*1000);
            if(pDnsSDcpt->dnsSTimerHandle)
            {
                pDnsSDcpt->dnsSTimeSeconds = 0;
            }
            else
            {
                return false;
            }
        }
    }

    if(stackCtrl->pNetIf->Flags.bIsDNSServerAuto!= 0)
    {
        TCPIP_DNSS_Enable(stackCtrl->pNetIf);
    }

	
    return true;
}

void TCPIP_DNSS_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    DNSS_DCPT *pDnsSDcpt;

    pDnsSDcpt = &gDnsSrvDcpt;
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {
        if(pDnsSDcpt->dnsSrvInitCount > 0)
        {	// we're up and running

            if(--pDnsSDcpt->dnsSrvInitCount == 0)
            {	// all closed
                // release resources
                if(pDnsSDcpt->dnsSTimerHandle)
                {
                    _TCPIPStackAsyncHandlerDeRegister(pDnsSDcpt->dnsSTimerHandle);
                    pDnsSDcpt->dnsSTimerHandle = 0;
                    pDnsSDcpt->dnsSTickPending = 0;
                    pDnsSDcpt->dnsSTimeSeconds = 0;
                }
                if(pDnsSDcpt->dnsSrvSocket != INVALID_UDP_SOCKET)
                {
                    TCPIP_UDP_Close(pDnsSDcpt->dnsSrvSocket);
                }
            }
        }
        // remove all the cache entries
        _DNSSRemoveCacheEntries();
    }

}

static  void _DNSSRemoveCacheEntries(void)
{
    OA_HASH_ENTRY* pBkt;
    DNSS_HASH_ENTRY *dnsSHE;    
    DNSS_DCPT       *pDnsSDcpt; 
    uint8_t *pMemoryBlock;  
    size_t      bktIx;
        
    pDnsSDcpt = &gDnsSrvDcpt;   

    if(pDnsSDcpt->dnssHashDcpt)
    {
        for(bktIx = 0; bktIx < pDnsSDcpt->dnssHashDcpt->hEntries; bktIx++)
        {
            pBkt = TCPIP_OAHASH_EntryGet(pDnsSDcpt->dnssHashDcpt, bktIx);  
            
            if(pBkt->flags.busy != 0)
            {
                dnsSHE = (DNSS_HASH_ENTRY*)pBkt;
                pMemoryBlock = (uint8_t*)dnsSHE->memblk;
                
                TCPIP_HEAP_Free(pDnsSDcpt->memH,pMemoryBlock);
                dnsSHE->nIPv4Entries = 0;
#if defined(TCPIP_STACK_USE_IPV6)
                dnsSHE->nIPv6Entries = 0;
#endif
                TCPIP_OAHASH_EntryRemove(pDnsSDcpt->dnssHashDcpt,pBkt);
            }
        }
    }
}

TCPIP_DNS_RESULT TCPIP_DNSS_AddressCntGet(int index,uint8_t * hostName,uint8_t * ipCount)
{
    DNSS_HASH_ENTRY* pDnsSHE;
    OA_HASH_ENTRY	*hE;
    OA_HASH_DCPT	*pOH;
    DNSS_DCPT*	pDnsSDcpt;
    TCPIP_NET_IF* pNetIf;

    
    pDnsSDcpt = &gDnsSrvDcpt;
    pOH = pDnsSDcpt->dnssHashDcpt;
    if(hostName == 0)
    {
        return DNS_RES_MEMORY_FAIL;
    }
    if(index >= pOH->hEntries)
    {
        return DNS_RES_NO_SERVICE;
    }
    pNetIf = (TCPIP_NET_IF*)TCPIP_UDP_SocketNetGet(pDnsSDcpt->dnsSrvSocket);

	hE = TCPIP_OAHASH_EntryGet(pOH, index);
	if((hE->flags.busy != 0) && (hE->flags.value & DNSS_FLAG_ENTRY_COMPLETE))
	{
	   pDnsSHE = (DNSS_HASH_ENTRY*)hE;
	   strncpy((char*)hostName,(char*)pDnsSHE->pHostName,strlen((char*)pDnsSHE->pHostName));
	   *ipCount = pDnsSHE->nIPv4Entries;
#if defined(TCPIP_STACK_USE_IPV6)
	    *ipCount += pDnsSHE->nIPv6Entries;
#endif
	    return DNS_RES_OK;
	}
    return DNS_RES_NO_ENTRY;

}

TCPIP_DNS_RESULT TCPIP_DNSS_EntryGet(uint8_t * hostName,IP_ADDRESS_TYPE type,int index,IP_MULTI_ADDRESS* pGetAdd)
{
    OA_HASH_ENTRY* hE;
    DNSS_HASH_ENTRY *dnsSHE;    
    DNSS_DCPT       *pDnsSDcpt; 
    uint8_t *pMemoryBlock; 
#if defined(TCPIP_STACK_USE_IPV6)
	uint8_t i=0;
    uint8_t nullval=0;
#endif
    pDnsSDcpt = &gDnsSrvDcpt;    
    if(hostName == 0)
    {
        return DNS_RES_NO_ENTRY;
    }
    hE = TCPIP_OAHASH_EntryLookup(pDnsSDcpt->dnssHashDcpt, (uint8_t *)hostName);
    if(hE == 0)
    {  
        return DNS_RES_NO_ENTRY;
    }
    dnsSHE = (DNSS_HASH_ENTRY*)hE;
    
    pMemoryBlock = (uint8_t*)dnsSHE->memblk;

    if(type == IP_ADDRESS_TYPE_IPV4)
    {
        if(index >= dnsSHE->nIPv4Entries)
            return DNS_RES_NO_SERVICE;
        dnsSHE->pip4Address = (IPV4_ADDR *)pMemoryBlock;
        if(dnsSHE->pip4Address == 0)
            return DNS_RES_NO_ENTRY;

        if(dnsSHE->pip4Address[index].Val != 0)
        {
            pGetAdd->v4Add.Val = dnsSHE->pip4Address[index].Val;
        }
        else
        {
            return DNS_RES_NO_IPADDRESS;
        }
    }
#if defined(TCPIP_STACK_USE_IPV6)
    if(type == IP_ADDRESS_TYPE_IPV6)
    {
        if(index >= dnsSHE->nIPv6Entries)
            return DNS_RES_NO_SERVICE;
        dnsSHE->pip6Address = (IPV6_ADDR *)(pMemoryBlock+pDnsSDcpt->IPv4EntriesPerDNSName*sizeof(IPV4_ADDR));
        if(dnsSHE->pip6Address == 0)
            return DNS_RES_NO_ENTRY;

        if(memcmp(dnsSHE->pip6Address[i].v,&nullval,sizeof(IPV6_ADDR)) != 0)
        {
            memcpy(pGetAdd->v6Add.v,dnsSHE->pip6Address[i].v,sizeof(IPV6_ADDR));
        }
        else
        {
            return DNS_RES_NO_IPADDRESS;
        }
    }
#endif
    return DNS_RES_OK;
}


TCPIP_DNS_RESULT TCPIP_DNSS_EntryAdd(const char* name, IP_ADDRESS_TYPE type, IP_MULTI_ADDRESS* pAdd,uint32_t validStartTime)
{
    OA_HASH_ENTRY   *hE;
    DNSS_DCPT       *pDnsSDcpt; 
    DNSS_HASH_ENTRY* dnsSHE;
    TCPIP_DNSS_CACHE_ENTRY dnssCacheEntry;
    
    pDnsSDcpt = &gDnsSrvDcpt;
	
    if(name == 0)
    {
        return DNS_RES_NO_ENTRY;
    }

    dnssCacheEntry.sHostNameData = (uint8_t *)name;
    dnssCacheEntry.recordType = type;
    dnssCacheEntry.validStartTime.Val = validStartTime;
    if(dnssCacheEntry.recordType == IP_ADDRESS_TYPE_IPV4)
    {
        dnssCacheEntry.ip4Address.Val = pAdd->v4Add.Val;
    }
#if defined(TCPIP_STACK_USE_IPV6)     
    else if(dnssCacheEntry.recordType == IP_ADDRESS_TYPE_IPV6)
    {
        memcpy(&dnssCacheEntry.ip6Address,&pAdd->v6Add,sizeof(IPV6_ADDR));
    }
#endif	
    else
    {
        return DNS_RES_NO_ENTRY;
    }
    hE = TCPIP_OAHASH_EntryLookup(pDnsSDcpt->dnssHashDcpt, dnssCacheEntry.sHostNameData);
    if(hE != 0)
    {
        dnsSHE = (DNSS_HASH_ENTRY*)hE;
        return _DNSSUpdateHashEntry(dnsSHE, dnssCacheEntry);
    }

    return _DNSSSetHashEntry(DNSS_FLAG_ENTRY_COMPLETE, dnssCacheEntry);

}

static  TCPIP_DNS_RESULT  _DNSSUpdateHashEntry( DNSS_HASH_ENTRY *dnsSHE,TCPIP_DNSS_CACHE_ENTRY dnssCacheEntry)
{
    DNSS_DCPT       *pDnsSDcpt; 
    uint8_t *pMemoryBlock;    
    pDnsSDcpt = &gDnsSrvDcpt;
    uint8_t     i=0;

    pMemoryBlock = (uint8_t*)dnsSHE->memblk;

    if(dnssCacheEntry.recordType == IP_ADDRESS_TYPE_IPV4)
    {
        if(dnsSHE->nIPv4Entries >= pDnsSDcpt->IPv4EntriesPerDNSName)
            return DNS_RES_CACHE_FULL;
        dnsSHE->pip4Address = (IPV4_ADDR *)pMemoryBlock;
        if(dnsSHE->pip4Address == 0)
            return DNS_RES_MEMORY_FAIL;
        for(i=0;i<pDnsSDcpt->IPv4EntriesPerDNSName;i++)
        {
            if(dnsSHE->pip4Address[i].Val == dnssCacheEntry.ip4Address.Val )
            {
                return DNS_RES_DUPLICATE_ENTRY;
            }
        }
        dnsSHE->pip4Address[dnsSHE->nIPv4Entries].Val = 
                            dnssCacheEntry.ip4Address.Val;
        dnsSHE->nIPv4Entries ++ ;
    }
#if defined(TCPIP_STACK_USE_IPV6)
    if(dnssCacheEntry.recordType == IP_ADDRESS_TYPE_IPV6)
    {
        if(dnsSHE->nIPv6Entries >= pDnsSDcpt->IPv6EntriesPerDNSName)
            return DNS_RES_CACHE_FULL;
        dnsSHE->pip6Address = (IPV6_ADDR *)(pMemoryBlock+pDnsSDcpt->IPv4EntriesPerDNSName*sizeof(IPV4_ADDR));
        if(dnsSHE->pip6Address == 0)
            return DNS_RES_MEMORY_FAIL;
        for(i=0;i<pDnsSDcpt->IPv6EntriesPerDNSName;i++)
        {
            if(memcmp(&dnsSHE->pip6Address[i],&dnssCacheEntry.ip6Address,sizeof(IPV6_ADDR)) == 0)
            {
                return DNS_RES_DUPLICATE_ENTRY;
            }
        }
        memcpy( &dnsSHE->pip6Address[dnsSHE->nIPv6Entries],&dnssCacheEntry.ip6Address,sizeof(IPV6_ADDR));
        dnsSHE->nIPv6Entries ++ ;
    }
#endif
    dnsSHE->startTime = dnssCacheEntry.validStartTime;
    
    return DNS_RES_OK;
}

static  TCPIP_DNS_RESULT  _DNSSSetHashEntry( DNSS_HASH_ENTRY_FLAGS newFlags,TCPIP_DNSS_CACHE_ENTRY dnssCacheEntry)
{
    int memoryBlockSize=0;
    uint8_t *pMemoryBlock;    
    DNSS_DCPT       *pDnsSDcpt; 
    OA_HASH_ENTRY* hE;
    DNSS_HASH_ENTRY *dnsSHE;
    uint8_t * hostName;
    

    pDnsSDcpt = &gDnsSrvDcpt;
    
    memoryBlockSize = pDnsSDcpt->IPv4EntriesPerDNSName*sizeof(IPV4_ADDR)
#if defined(TCPIP_STACK_USE_IPV6)
        + pDnsSDcpt->IPv6EntriesPerDNSName*sizeof(IPV6_ADDR)
#endif        
        +strlen((char*)dnssCacheEntry.sHostNameData)+1;

    pMemoryBlock = (uint8_t *)TCPIP_HEAP_Malloc(pDnsSDcpt->memH,memoryBlockSize);
    if(pMemoryBlock == 0)
    {
        return DNS_RES_MEMORY_FAIL;
    }
    memset(pMemoryBlock,0,memoryBlockSize);
    
    hostName = (uint8_t*)(pMemoryBlock+pDnsSDcpt->IPv4EntriesPerDNSName*sizeof(IPV4_ADDR)
#if defined(TCPIP_STACK_USE_IPV6)
        + pDnsSDcpt->IPv6EntriesPerDNSName*sizeof(IPV6_ADDR)
#endif        
    );
// copy HostName first
    memset(hostName,0,strlen((char*)dnssCacheEntry.sHostNameData)+1);
    strncpy((char*)hostName,(char*)dnssCacheEntry.sHostNameData,strlen((char*)dnssCacheEntry.sHostNameData));
    
    hE = TCPIP_OAHASH_EntryLookupOrInsert(pDnsSDcpt->dnssHashDcpt, hostName);
    if(hE == 0)
    {
        TCPIP_HEAP_Free(pDnsSDcpt->memH,pMemoryBlock);
        return DNS_RES_CACHE_FULL;
    }
    dnsSHE = (DNSS_HASH_ENTRY*)hE;
    dnsSHE->hEntry.flags.value &= ~DNSS_FLAG_ENTRY_VALID_MASK;
    dnsSHE->hEntry.flags.value |= newFlags;
    dnsSHE->memblk = pMemoryBlock;
    dnsSHE->recordType = dnssCacheEntry.recordType;
    
    if(dnssCacheEntry.recordType == IP_ADDRESS_TYPE_IPV4)
    {
        if(dnsSHE->nIPv4Entries >= pDnsSDcpt->IPv4EntriesPerDNSName)
            return DNS_RES_CACHE_FULL;
        dnsSHE->pip4Address = (IPV4_ADDR *)pMemoryBlock;
        if(dnsSHE->pip4Address == 0)
            return DNS_RES_MEMORY_FAIL;
        dnsSHE->pip4Address[dnsSHE->nIPv4Entries].Val = 
                            dnssCacheEntry.ip4Address.Val;
        dnsSHE->nIPv4Entries ++ ;
    }
#if defined(TCPIP_STACK_USE_IPV6)
    if(dnssCacheEntry.recordType == IP_ADDRESS_TYPE_IPV6)
    {
        if(dnsSHE->nIPv6Entries >= pDnsSDcpt->IPv6EntriesPerDNSName)
            return DNS_RES_CACHE_FULL;
        dnsSHE->pip6Address = (IPV6_ADDR *)(pMemoryBlock+pDnsSDcpt->IPv4EntriesPerDNSName*sizeof(IPV4_ADDR));
        if(dnsSHE->pip6Address == 0)
            return DNS_RES_MEMORY_FAIL;
        memcpy( &dnsSHE->pip6Address[dnsSHE->nIPv6Entries],&dnssCacheEntry.ip6Address,sizeof(IPV6_ADDR));
        dnsSHE->nIPv6Entries ++ ;
    }
#endif

    dnsSHE->tInsert = SYS_TMR_TickCountGet();
    dnsSHE->startTime = dnssCacheEntry.validStartTime;
    return DNS_RES_OK;
}

TCPIP_DNS_RESULT TCPIP_DNSS_CacheEntryRemove(const char* name, IP_ADDRESS_TYPE type, IP_MULTI_ADDRESS* pAdd)
{
    OA_HASH_ENTRY* hE;
    DNSS_HASH_ENTRY *dnsSHE;    
    DNSS_DCPT       *pDnsSDcpt; 
    uint8_t *pMemoryBlock;  
    int         i=0;
    bool        addrISPresent=false;
    
    pDnsSDcpt = &gDnsSrvDcpt;     
    if(name == 0)
    {
        return DNS_RES_NO_ENTRY;
    }
    hE = TCPIP_OAHASH_EntryLookup(pDnsSDcpt->dnssHashDcpt, (uint8_t *)name);
    if(hE == 0)
    {  
        return DNS_RES_NO_ENTRY;
    }
    dnsSHE = (DNSS_HASH_ENTRY*)hE;
    if(type != dnsSHE->recordType)
    {  
        return DNS_RES_NO_ENTRY;
    }
    pMemoryBlock = (uint8_t*)dnsSHE->memblk;

    if(dnsSHE->recordType == IP_ADDRESS_TYPE_IPV4)
    {
        if(dnsSHE->nIPv4Entries > pDnsSDcpt->IPv4EntriesPerDNSName)
            return DNS_RES_MEMORY_FAIL;
        dnsSHE->pip4Address = (IPV4_ADDR *)pMemoryBlock;
        if(dnsSHE->pip4Address == 0)
            return DNS_RES_MEMORY_FAIL;
        for(i=0;i<pDnsSDcpt->IPv4EntriesPerDNSName;i++)
        {
            if(dnsSHE->pip4Address[i].Val == pAdd->v4Add.Val)
            {
                dnsSHE->nIPv4Entries--;
                dnsSHE->pip4Address[i].Val = 0;
                addrISPresent = true;
                break;
            }
        }
    }
#if defined(TCPIP_STACK_USE_IPV6)
    if(dnsSHE->recordType == IP_ADDRESS_TYPE_IPV6)
    {
        if(dnsSHE->nIPv6Entries > pDnsSDcpt->IPv6EntriesPerDNSName)
            return DNS_RES_MEMORY_FAIL;
        dnsSHE->pip6Address = (IPV6_ADDR *)(pMemoryBlock+pDnsSDcpt->IPv4EntriesPerDNSName*sizeof(IPV4_ADDR));
        if(dnsSHE->pip6Address == 0)
            return DNS_RES_MEMORY_FAIL;
        for(i=0;i<pDnsSDcpt->IPv6EntriesPerDNSName;i++)
        {
            if(memcmp(&dnsSHE->pip6Address[i],&pAdd->v6Add,sizeof(IPV6_ADDR)) == 0)
            {
                dnsSHE->nIPv6Entries--;
                memset(&dnsSHE->pip6Address[i], 0,sizeof(IPV6_ADDR));
                addrISPresent = true;
                break;
            }
        }
    }
#endif

    if(addrISPresent == false)
    {
        return DNS_RES_NO_ENTRY;
    }

   // Free Hash entry and free the allocated memory for this HostName if there
   // is no IPv4 and IPv6 entry
   if(!dnsSHE->nIPv4Entries 
#if defined(TCPIP_STACK_USE_IPV6)
     && !dnsSHE->nIPv6Entries
#endif
    )
    {       
        TCPIP_HEAP_Free(pDnsSDcpt->memH,pMemoryBlock);
        TCPIP_OAHASH_EntryRemove(pDnsSDcpt->dnssHashDcpt,hE);
    }
   return DNS_RES_OK;
    
}

/****************************************************************************
  Function:
    uint8_t TCPIP_DNSS_DataGet(uint16_t pos)

  Summary:
    Reads uint8_t data from globally allocated memory buffer.

  Description:
    Get Dns server buffer incomming query packet details.

  Precondition:
    None

  Parameters:
  	pos: position in the buffer from which the data to be read

  Return Values:
	uint8_t: 1 byte value read

  Remarks:
  	The read position offset is required to be provided every time the routine is called.
  	This API do not increment the buffer read offset automatically, everytime it is called.

***************************************************************************/
uint8_t TCPIP_DNSS_DataGet(uint16_t pos)
{
	return (uint8_t)(dnsSrvRecvByte[pos]);
}

/****************************************************************************
  Function:
    bool TCPIP_DNSS_DataPut(uint8_t * buf,uint16_t pos,uint8_t val)

  Summary:
    write uint8_t data to dynamic allocated memory buffer.

  Description:
    Put Dns server buffer for Outgoing response packet.

  Precondition:
    None

  Parameters:
        buf : write to the buffer
  	pos: position in the buffer from which the data to be read
  	val : write the value to the buffer

  Return Values:
	true or false: 0 or 1

  Remarks:
  	The read position offset is required to be provided every time the routine is called.
  	This API do not increment the buffer read offset automatically, everytime it is called.

***************************************************************************/
bool TCPIP_DNSS_DataPut(uint8_t * buf,uint32_t pos,uint8_t val)
{
    if(buf == 0)
        return false;
	buf[pos] = val;
    return true;
}


/*********************************************************************
 * Function:        void TCPIP_DNSS_Task(TCPIP_NET_IF* pNet)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Sends dummy responses that point to ourself for DNS requests
 *
 * Note:            None
 ********************************************************************/
bool TCPIP_DNSS_Task(TCPIP_NET_IF* pNet)
{
    DNSS_DCPT   *pDnsSrvDcpt;
    UDP_SOCKET  s;
    TCPIP_UINT16_VAL    recordType;
    OA_HASH_ENTRY* hE=NULL;
    DNSS_HASH_ENTRY *dnsSHE = NULL;       // get rid of compiler warning
    uint16_t resAnswerRRs=0;
    uint32_t ttlTime = NULL;              // get rid of compiler warning
    uint8_t *pMemoryBlock = NULL;         // get rid of compiler warning
    uint8_t *txbuf;
    uint8_t  count=0;
    uint32_t   recvLen=0;
    uint32_t   servTxMsgSize=0;
    uint32_t   txBufPos = 0;
    uint8_t    hostNamePos=0;  
#if defined (TCPIP_STACK_USE_IPV6)
    uint8_t     i=0;
    IPV6_INTERFACE_CONFIG*  pIpv6Config;
    IPV6_ADDR_STRUCT * addressPointer;
#endif
    uint16_t     offset=0;
    static uint32_t dnsProcessInterval;
    static TCPIP_UINT16_VAL transactionId;
    
    static struct
    {
        TCPIP_UINT16_VAL wTransactionID;
        TCPIP_UINT16_VAL wFlags;
        TCPIP_UINT16_VAL wQuestions;
        TCPIP_UINT16_VAL wAnswerRRs;
        TCPIP_UINT16_VAL wAuthorityRRs;
        TCPIP_UINT16_VAL wAdditionalRRs;
    } DNSHeader;

    if(pNet == 0 || pNet->Flags.bInterfaceEnabled == 0)
    {
        return false;
    }
     if(pNet->netIPAddr.Val == 0)
        return false;

    // check if DNS Server is enabled on this nterface
    if(!TCPIP_DNSS_IsEnabled(pNet))
    {
        return false;
    }
    
    pDnsSrvDcpt  = &gDnsSrvDcpt;
    s = pDnsSrvDcpt->dnsSrvSocket;
    switch(pDnsSrvDcpt->smState)
    {
        case DNSS_STATE_START:
            // Create a socket to listen on if this is the first time calling this function
            if(s == INVALID_UDP_SOCKET)
            {
                s = TCPIP_UDP_ServerOpen(IP_ADDRESS_TYPE_ANY, TCPIP_DNS_SERVER_PORT, 0);
                if(s == INVALID_UDP_SOCKET)
                    break;
            }
            TCPIP_UDP_SocketNetSet(s, pNet);
            pDnsSrvDcpt->dnsSrvSocket = s;
            pDnsSrvDcpt->smState = DNSS_STATE_WAIT_REQUEST;
            pDnsSrvDcpt->intfIdx = TCPIP_STACK_NetIxGet(pNet);
            pDnsSrvDcpt->flags.bits.DNSServInUse = DNS_SERVER_ENABLE;
            dnsProcessInterval = SYS_TMR_TickCountGet();
            transactionId.Val = 0;
            break;

        case DNSS_STATE_WAIT_REQUEST:

            // See if a DNS query packet has arrived
            recvLen = TCPIP_UDP_GetIsReady(s);
            if(recvLen == 0)
            {
                TCPIP_UDP_Discard(s);
                break;
            }
            if(recvLen > (sizeof(dnsSrvRecvByte)-1))
            {
                TCPIP_UDP_Discard(s);
                break;
            }
            gDnsSrvBytePos = 0;
            // Read DNS header
            TCPIP_UDP_ArrayGet(s, (uint8_t*)dnsSrvRecvByte, recvLen);
            // Assign DNS transaction ID
            DNSHeader.wTransactionID.v[1] = dnsSrvRecvByte[gDnsSrvBytePos++];
            DNSHeader.wTransactionID.v[0] = dnsSrvRecvByte[gDnsSrvBytePos++];
            // To make a response for a valid address and quicker one.
            // this is used to make sure that we should not transmit same address response again and again for sometime.
            // This will help to improve the throughput
            //We are checking the previous transaction id and process the rx packet only when it is a new transaction id
            // comparing to the previous one.
            if(transactionId.Val != DNSHeader.wTransactionID.Val)
            {
                transactionId.Val = DNSHeader.wTransactionID.Val;
            }
            else
            {
            	TCPIP_UDP_Discard(s);
                break;
            }
            // Assign DNS wflags
            DNSHeader.wFlags.v[1] = dnsSrvRecvByte[gDnsSrvBytePos++];
            DNSHeader.wFlags.v[0] = dnsSrvRecvByte[gDnsSrvBytePos++];

            DNSHeader.wQuestions.v[1] = dnsSrvRecvByte[gDnsSrvBytePos++];
            DNSHeader.wQuestions.v[0] = dnsSrvRecvByte[gDnsSrvBytePos++];

            DNSHeader.wAnswerRRs.v[1] = dnsSrvRecvByte[gDnsSrvBytePos++];
            DNSHeader.wAnswerRRs.v[0] = dnsSrvRecvByte[gDnsSrvBytePos++];

            DNSHeader.wAuthorityRRs.v[1] = dnsSrvRecvByte[gDnsSrvBytePos++];
            DNSHeader.wAuthorityRRs.v[0] = dnsSrvRecvByte[gDnsSrvBytePos++];

            DNSHeader.wAdditionalRRs.v[1] = dnsSrvRecvByte[gDnsSrvBytePos++];
            DNSHeader.wAdditionalRRs.v[0] = dnsSrvRecvByte[gDnsSrvBytePos++];

            // Ignore this packet if it isn't a query
            if((DNSHeader.wFlags.Val & 0x8000) == 0x8000u)
            {
                TCPIP_UDP_Discard(s);
                break;
            }
            // Ignore this packet if there are no questions in it
            if(DNSHeader.wQuestions.Val == 0u)
            {            
                TCPIP_UDP_Discard(s);
                break;
            }

            pDnsSrvDcpt->smState = DNSS_STATE_PUT_REQUEST;
            break;

        case DNSS_STATE_PUT_REQUEST:           
            // collect hostname from Client Query Named server packet
            _DNSCopyRXNameToTX(s);   // Copy hostname of first question over to TX packet
            if(strlen((char*)hostNameWithDot) == 0)
            {            	
                pDnsSrvDcpt->smState = DNSS_STATE_WAIT_REQUEST;
                TCPIP_UDP_Discard(s);
                break;
            }

            // Get the Record type
            _DNSSGetRecordType(s,&recordType);
            switch(recordType.Val)
            {
                case DNS_TYPE_A:
#if defined(TCPIP_STACK_USE_IPV6)
                case DNS_TYPE_AAAA:
#endif
                    break;
                default:
                    pDnsSrvDcpt->smState = DNSS_STATE_WAIT_REQUEST;					
                    TCPIP_UDP_Discard(s);
                    return true;
            }
            if(!pDnsSrvDcpt->replyWithBoardInfo)
            {
                hE = TCPIP_OAHASH_EntryLookup(pDnsSrvDcpt->dnssHashDcpt, (uint8_t *)hostNameWithDot);
                if(hE != 0)
                {
                    dnsSHE = (DNSS_HASH_ENTRY*)hE;
                    pMemoryBlock = (uint8_t*)dnsSHE->memblk;
                }
                else 
                {
                    pDnsSrvDcpt->smState = DNSS_STATE_WAIT_REQUEST;
                    TCPIP_UDP_Discard(s);
                    break;
                }
            }
            // update Answer field
            // If the client Query answer is zero, then Response will have all the answers which is present in the cache
            // else if the client query answer count is more than the available answer counts  of the cache, then Answer RRs should 
            // be the value of available entries in the cache , else if only the limited Answer RRs
            if(recordType.Val == DNS_TYPE_A)
            {
                if(pDnsSrvDcpt->replyWithBoardInfo)
                {
                    resAnswerRRs = 1;
                    ttlTime = DNSS_TTL_TIME;
                }
                else if(hE != 0)
                {
                    if((DNSHeader.wAnswerRRs.Val == 0) || (DNSHeader.wAnswerRRs.Val > dnsSHE->nIPv4Entries))
                    {
                    // all the available IPv4 address entries
                        resAnswerRRs = dnsSHE->nIPv4Entries;
                    }                
                    else  // only limited entries
                    {
                        resAnswerRRs = DNSHeader.wAnswerRRs.Val;
                    }
                    // ttl time  w.r.t configured per entry
                    ttlTime = dnsSHE->startTime.Val - ((SYS_TMR_TickCountGet() - dnsSHE->tInsert)/SYS_TMR_TickPerSecond());
                }
                else
                {
                    pDnsSrvDcpt->smState = DNSS_STATE_WAIT_REQUEST;
                    TCPIP_UDP_Discard(s);
                    break;
                }
            }
#if defined(TCPIP_STACK_USE_IPV6)            
            else if(recordType.Val == DNS_TYPE_AAAA)
            {
                if(pDnsSrvDcpt->replyWithBoardInfo)
                {
                    resAnswerRRs = 1;
                    ttlTime = DNSS_TTL_TIME;
                }
                else if(hE != 0)
                {
                    if((DNSHeader.wAnswerRRs.Val == 0) || (DNSHeader.wAnswerRRs.Val > dnsSHE->nIPv6Entries))
                    {
                    // all the available IPv6 address entries                    
                        resAnswerRRs = dnsSHE->nIPv6Entries;
                    }                
                    else  // only limited entries
                    {
                        resAnswerRRs = DNSHeader.wAnswerRRs.Val;
                    }
                     // ttl time  w.r.t configured per entry
                    ttlTime = dnsSHE->startTime.Val - ((SYS_TMR_TickCountGet() - dnsSHE->tInsert)/SYS_TMR_TickPerSecond());
                }
                else
                {
                    pDnsSrvDcpt->smState = DNSS_STATE_WAIT_REQUEST;
                    TCPIP_UDP_Discard(s);
                    break;
                }
            }
#endif                
            offset = 0xC00C; // that is the location at 0x0c ( 12) DNS packet compression RFC 1035
            servTxMsgSize = sizeof(DNSHeader)         // DNS header 
                            + countWithLen+2+2;  // Query hostname + type + class
            if(recordType.Val == DNS_TYPE_A)
            {
                // offset + record type+class+ttl+ip type+size of IP address * number of answers present ih HASH table
                servTxMsgSize += resAnswerRRs*(2+2+2+4+2+sizeof(IPV4_ADDR));
            }
#if defined(TCPIP_STACK_USE_IPV6)            
            else if(recordType.Val == DNS_TYPE_AAAA)
            {
                // offset + record type+class+ttl+ip type+size of IP address * number of answers present ih HASH table
                servTxMsgSize += resAnswerRRs*(2+2+2+4+2+sizeof(IPV6_ADDR));
            }
#endif                        
            // check that we can transmit a DNS response packet
            if(!TCPIP_UDP_TxPutIsReady(s, servTxMsgSize))
            {   
                pDnsSrvDcpt->smState = DNSS_STATE_WAIT_REQUEST;   
                TCPIP_UDP_Discard(s);
                break;
            }
            txbuf = TCPIP_HEAP_Malloc(pDnsSrvDcpt->memH,servTxMsgSize+1 ); // one extra memory 
            if(txbuf == 0)
            {   
                pDnsSrvDcpt->smState = DNSS_STATE_WAIT_REQUEST;  
                TCPIP_UDP_Discard(s);
                break;
            }
            txBufPos = 0;
            // Write DNS Server response packet
            // Transaction ID 
            TCPIP_DNSS_DataPut(txbuf,txBufPos++,DNSHeader.wTransactionID.v[1]);            
            TCPIP_DNSS_DataPut(txbuf,txBufPos++,DNSHeader.wTransactionID.v[0]);
            
            if(DNSHeader.wFlags.Val & 0x0100)
            {
                TCPIP_DNSS_DataPut(txbuf,txBufPos++,0x81); // Message is a response with recursion desired
            }
            else
            {
                TCPIP_DNSS_DataPut(txbuf,txBufPos++,0x80); // Message is a response without recursion desired flag set
            }
            TCPIP_DNSS_DataPut(txbuf,txBufPos++,0x80); // Recursion available

            // Question 
            TCPIP_DNSS_DataPut(txbuf,txBufPos++,DNSHeader.wQuestions.v[1]);            
            TCPIP_DNSS_DataPut(txbuf,txBufPos++,DNSHeader.wQuestions.v[0]);
            
            // Answer
            TCPIP_DNSS_DataPut(txbuf,txBufPos++,resAnswerRRs>>8&0xFF);            
            TCPIP_DNSS_DataPut(txbuf,txBufPos++,resAnswerRRs&0xFF);
            
            // send Authority and Additional RRs as 0 , It wll chnages latter when we support Authentication and Additional DNS info
            TCPIP_DNSS_DataPut(txbuf,txBufPos++,0);            
            TCPIP_DNSS_DataPut(txbuf,txBufPos++,0);
            TCPIP_DNSS_DataPut(txbuf,txBufPos++,0);            
            TCPIP_DNSS_DataPut(txbuf,txBufPos++,0);
            // Prepare all the queries
            for(hostNamePos=0;hostNamePos<countWithLen;hostNamePos++)
            {
                TCPIP_DNSS_DataPut(txbuf,txBufPos++,hostNameWithLen[hostNamePos]);
            }
            // Record Type
            TCPIP_DNSS_DataPut(txbuf,txBufPos++,recordType.v[1]);
            TCPIP_DNSS_DataPut(txbuf,txBufPos++,recordType.v[0]);
            // Class
            TCPIP_DNSS_DataPut(txbuf,txBufPos++,0x00);
            TCPIP_DNSS_DataPut(txbuf,txBufPos++,0x01);
            
            // Prepare Answer for all the answers 
            for(count=0;count <resAnswerRRs;count++)
            {
                // Put Host name Pointer As per RFC1035 DNS compression                
                TCPIP_DNSS_DataPut(txbuf,txBufPos++,offset>>8 &0xFF);                
                TCPIP_DNSS_DataPut(txbuf,txBufPos++,offset&0xFF);

                // Record Type                
                TCPIP_DNSS_DataPut(txbuf,txBufPos++,recordType.v[1]);            
                TCPIP_DNSS_DataPut(txbuf,txBufPos++,recordType.v[0]);
                // Class
                TCPIP_DNSS_DataPut(txbuf,txBufPos++,0x00);            
                TCPIP_DNSS_DataPut(txbuf,txBufPos++,0x01);
                // TTL
                TCPIP_DNSS_DataPut(txbuf,txBufPos++,ttlTime>>24&0xFF);            
                TCPIP_DNSS_DataPut(txbuf,txBufPos++,ttlTime>>16&0xFF);
                TCPIP_DNSS_DataPut(txbuf,txBufPos++,ttlTime>>8&0xFF);            
                TCPIP_DNSS_DataPut(txbuf,txBufPos++,ttlTime&0xFF);
                if(recordType.Val == DNS_TYPE_A)
                {
                    // Length for TYPE_A
                    TCPIP_DNSS_DataPut(txbuf,txBufPos++,0x00);            
                    TCPIP_DNSS_DataPut(txbuf,txBufPos++,0x04);
                    if(hE != 0)
                    {
                        dnsSHE->pip4Address = (IPV4_ADDR *)pMemoryBlock;
                        if(dnsSHE->pip4Address == 0)
                        {
                            pDnsSrvDcpt->smState = DNSS_STATE_WAIT_REQUEST;                
                            TCPIP_UDP_Discard(s);
                            break;
                        }
                        TCPIP_DNSS_DataPut(txbuf,txBufPos++,dnsSHE->pip4Address[count].v[0]);            
                        TCPIP_DNSS_DataPut(txbuf,txBufPos++,dnsSHE->pip4Address[count].v[1]);
                        TCPIP_DNSS_DataPut(txbuf,txBufPos++,dnsSHE->pip4Address[count].v[2]);            
                        TCPIP_DNSS_DataPut(txbuf,txBufPos++,dnsSHE->pip4Address[count].v[3]);
                    }
                    else
                    {
                        TCPIP_DNSS_DataPut(txbuf,txBufPos++,pNet->netIPAddr.v[0]);            
                        TCPIP_DNSS_DataPut(txbuf,txBufPos++,pNet->netIPAddr.v[1]);
                        TCPIP_DNSS_DataPut(txbuf,txBufPos++,pNet->netIPAddr.v[2]);            
                        TCPIP_DNSS_DataPut(txbuf,txBufPos++,pNet->netIPAddr.v[3]);
                    }
                }
#if defined(TCPIP_STACK_USE_IPV6)                
                else if(recordType.Val == DNS_TYPE_AAAA)
                {
                    // Length for TYPE_A
                    TCPIP_DNSS_DataPut(txbuf,txBufPos++,0x00);     // Data Length 16 bytes        
                    TCPIP_DNSS_DataPut(txbuf,txBufPos++,0x10);  // sizeof (IPV6_ADDR)
                    if(hE != 0)
                    {
                        dnsSHE->pip6Address = (IPV6_ADDR *)(pMemoryBlock+pDnsSrvDcpt->IPv4EntriesPerDNSName*sizeof(IPV4_ADDR));
                        if(dnsSHE->pip6Address == 0)
                        {
                            pDnsSrvDcpt->smState = DNSS_STATE_WAIT_REQUEST;                
                            TCPIP_UDP_Discard(s);
                            break;
                        }
                        for(i=0;i<sizeof(IPV6_ADDR);i++)    
                        {
                            TCPIP_DNSS_DataPut(txbuf,txBufPos++,dnsSHE->pip6Address[count].v[i]); 
                        }
                    }
                    else
                    {
                        pIpv6Config = TCPIP_IPV6_InterfaceConfigGet(pNet);                        
                        addressPointer = (IPV6_ADDR_STRUCT *)pIpv6Config->listIpv6UnicastAddresses.head;
                        // only one IPv6 unicast address
                        for(i=0;i<sizeof(IPV6_ADDR);i++)    
                        {
                            TCPIP_DNSS_DataPut(txbuf,txBufPos++,addressPointer->address.v[i]); 
                        }
                    }
                }
#endif                
            }
            // Transmit all the server bytes
            TCPIP_UDP_ArrayPut(s,txbuf,txBufPos);
            TCPIP_UDP_Flush(s);
            if(txbuf !=0)
            {
                TCPIP_HEAP_Free(pDnsSrvDcpt->memH,txbuf); 
            }

            pDnsSrvDcpt->smState = DNSS_STATE_DONE;
            break;
            
         case DNSS_STATE_DONE:
             if(((SYS_TMR_TickCountGet() - dnsProcessInterval)/SYS_TMR_TickPerSecond()) > 1)
             {
                 pDnsSrvDcpt->smState = DNSS_STATE_WAIT_REQUEST;
                 dnsProcessInterval = SYS_TMR_TickCountGet();
             }
            break;
    }
            
    return true;
           
}



/*****************************************************************************
  Function:
	static void _DNSCopyRXNameToTX(UDP_SOCKET s)

  Summary:
	Copies a DNS hostname, possibly including name compression, from the RX 
	packet to the TX packet (without name compression in TX case).
	
  Description:
	None

  Precondition:
	RX pointer is set to currently point to the DNS name to copy

  Parameters:
	None

  Returns:
  	None
  ***************************************************************************/
static void _DNSCopyRXNameToTX(UDP_SOCKET s)
{
    uint16_t w;
    uint8_t i=0,j=0;
    uint8_t len;
    //uint8_t data[64]={0};
    
    countWithDot=0;
    countWithLen=0;
    while(1)
    {
        // Get first byte which will tell us if this is a 16-bit pointer or the
        // length of the first of a series of labels
        //	return;
        i = TCPIP_DNSS_DataGet(gDnsSrvBytePos++);
		
        // Check if this is a pointer, if so, get the remaining 8 bits and seek to the pointer value
        if((i & 0xC0u) == 0xC0u)
        {
            ((uint8_t*)&w)[1] = i & 0x3F;
            w = TCPIP_DNSS_DataGet(gDnsSrvBytePos++);
            gDnsSrvBytePos =  w;
            continue;
        }

        // Write the length byte
        len = i;
        if(countWithLen==0 && countWithDot==0)
        {
            hostNameWithLen[countWithLen++]=len;
        }
        else
        {
            hostNameWithLen[countWithLen++]=len;
            // when it reached the end of hostname , then '.' is not required
            if(len!=0)
                hostNameWithDot[countWithDot++]='.';
        }
		
		
        // Exit if we've reached a zero length label
        if(len == 0u)
        {
            hostNameWithDot[countWithDot] = 0;
            return;
        }

        //UDPGetArray(s,data,len);
        for(j=0;j<len;j++)
        {
            i = TCPIP_DNSS_DataGet(gDnsSrvBytePos++);
        // update the hostNameWithDot with data 
            hostNameWithLen[countWithLen++] = i;
        
        // update the hostNameWithLen with data 
            hostNameWithDot[countWithDot++] = i;
        }

        if((countWithLen > DNSS_HOST_NAME_LEN) || (countWithDot > DNSS_HOST_NAME_LEN))
        {
            return;
        }        
    }
}



void TCPIP_DNSS_CacheTimeTask(void)
{
    DNSS_HASH_ENTRY* pDnsSHE;
    OA_HASH_ENTRY	*hE;
    int 			bktIx=0;
    OA_HASH_DCPT	*pOH;
    DNSS_DCPT*	pDnsSDcpt;
    TCPIP_NET_IF* pNetIf;

    
    pDnsSDcpt = &gDnsSrvDcpt;
    pOH = pDnsSDcpt->dnssHashDcpt;
    gDnsSrvDcpt.dnsSTimeSeconds += DNSS_TASK_PROCESS_RATE;
    if(pDnsSDcpt->dnsSrvSocket == INVALID_UDP_SOCKET)
    {
       return;
    }
    pNetIf = (TCPIP_NET_IF*)TCPIP_UDP_SocketNetGet(pDnsSDcpt->dnsSrvSocket);


// check the lease values and if there is any entry whose lease value exceeds the lease duration remove the lease entries from the HASH.

    for(bktIx = 0; bktIx < pOH->hEntries; bktIx++)
    {
        hE = TCPIP_OAHASH_EntryGet(pOH, bktIx);
    	if((hE->flags.busy != 0) && (hE->flags.value & DNSS_FLAG_ENTRY_COMPLETE))
    	{
            pDnsSHE = (DNSS_HASH_ENTRY*)hE;
            if(((SYS_TMR_TickCountGet() - pDnsSHE->tInsert)/SYS_TMR_TickPerSecond()) > pDnsSHE->startTime.Val )
            {
                pDnsSHE->tInsert = 0;
                TCPIP_OAHASH_EntryRemove(pOH,hE);

                // Free IPv4 and IPv6 address and hostname      
                TCPIP_HEAP_Free(pDnsSDcpt->memH,pDnsSHE->memblk);
                pDnsSHE->nIPv4Entries = 0;
                pDnsSHE->pip4Address = 0;            
#ifdef TCPIP_STACK_USE_IPV6
                pDnsSHE->nIPv6Entries = 0;
                pDnsSHE->pip6Address = 0;
#endif    
            }           
    	}       
    }   
}




bool TCPIP_DNSS_IsEnabled(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(hNet);
    if(pNetIf)
    {
        return pNetIf->Flags.bIsDnsServerEnabled!= 0;
    }
    return false;
}

bool TCPIP_DNSS_Enable(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(hNet);
    DNSS_DCPT* pServer;
    pServer  = &gDnsSrvDcpt;
    if(pNetIf == 0)
    {
        return false;
    }
    
    if(TCPIP_STACK_DNSServiceCanStart(pNetIf, TCPIP_STACK_SERVICE_DNSS))
    {
        if(pServer->flags.bits.DNSServInUse == DNS_SERVER_DISABLE)
        {
            pServer->smState = DNSS_STATE_START;
            pServer->flags.bits.DNSServInUse = DNS_SERVER_ENABLE;
            pNetIf->Flags.bIsDnsServerEnabled = true;
            pServer->dnsSrvSocket = INVALID_UDP_SOCKET;
            return true;
        }
    }
    return false;
}

bool TCPIP_DNSS_Disable(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(hNet);
    DNSS_DCPT* pServer;
    pServer  = &gDnsSrvDcpt;
    if(pNetIf == 0)
    {
        return false;
    }
    if(pServer->flags.bits.DNSServInUse == DNS_SERVER_ENABLE)
    {
        pServer->smState = DNSS_STATE_START;
        pServer->flags.bits.DNSServInUse = DNS_SERVER_DISABLE;
                pNetIf->Flags.bIsDnsServerEnabled = false;
        if(pServer->dnsSrvSocket != INVALID_UDP_SOCKET)
        {
            TCPIP_UDP_Close(pServer->dnsSrvSocket);
        }
    }
    
   // _TCPIPStackAddressServiceEvent(pNetIf, TCPIP_STACK_ADDRESS_SERVICE_DNSS, TCPIP_STACK_ADDRESS_SERVICE_EVENT_USER_STOP);
    return true;
	
}

static void _DNSSGetRecordType(UDP_SOCKET s,TCPIP_UINT16_VAL *recordType)
{
    recordType->v[1] = TCPIP_DNSS_DataGet(gDnsSrvBytePos++);
    recordType->v[0] = TCPIP_DNSS_DataGet(gDnsSrvBytePos++);
}



size_t TCPIP_OAHASH_DNSS_KeyHash(OA_HASH_DCPT* pOH, void* key)
{
    uint8_t    *dnsHostNameKey;
    size_t      hostnameLen=0;

    dnsHostNameKey = (uint8_t *)key;
    hostnameLen = strlen((const char*)dnsHostNameKey);
    return fnv_32_hash(dnsHostNameKey, hostnameLen) % (pOH->hEntries);
}


OA_HASH_ENTRY* TCPIP_OAHASH_DNSS_EntryDelete(OA_HASH_DCPT* pOH)
{
    OA_HASH_ENTRY*  pBkt;
    size_t      bktIx;
    DNSS_HASH_ENTRY  *pE;
    DNSS_DCPT        *pDnssDcpt;

    pDnssDcpt = &gDnsSrvDcpt;
    for(bktIx = 0; bktIx < pOH->hEntries; bktIx++)
    {
        pBkt = TCPIP_OAHASH_EntryGet(pOH, bktIx);		
        if(pBkt->flags.busy != 0)
        {
            pE = (DNSS_HASH_ENTRY*)pBkt;
            
            if(SYS_TMR_TickCountGet() - pE->tInsert > (pE->startTime.Val * SYS_TMR_TickPerSecond()))
            {
                return pBkt;
            }
        }
    }

    return 0;
}


int TCPIP_OAHASH_DNSS_KeyCompare(OA_HASH_DCPT* pOH, OA_HASH_ENTRY* hEntry, void* key)
{
    DNSS_HASH_ENTRY  *pDnssHE;
    uint8_t         *dnsHostNameKey;
    size_t          hostnameLen=0;
    
    pDnssHE =(DNSS_HASH_ENTRY  *)hEntry;
    dnsHostNameKey = (uint8_t *)key;    
    hostnameLen = strlen((const char*)dnsHostNameKey);
    
    return strcmp((const char*)pDnssHE->pHostName,(const char*)dnsHostNameKey);
}

void TCPIP_OAHASH_DNSS_KeyCopy(OA_HASH_DCPT* pOH, OA_HASH_ENTRY* dstEntry, void* key)
{
    uint8_t    *dnsHostNameKey;
    DNSS_HASH_ENTRY  *pDnssHE;
    size_t          hostnameLen=0;

    if(key==NULL) return;
    
    pDnssHE =(DNSS_HASH_ENTRY  *)dstEntry;
    dnsHostNameKey = (uint8_t *)key;
    hostnameLen = strlen((const char*)dnsHostNameKey);

    if(dnsHostNameKey)
        pDnssHE->pHostName = dnsHostNameKey;
}

#if defined(OA_DOUBLE_HASH_PROBING)
size_t TCPIP_OAHASH_DNSS_ProbeHash(OA_HASH_DCPT* pOH, void* key)
{
    uint8_t    *dnsHostNameKey;
    size_t      hostnameLen=0;
    
    dnsHostNameKey = (uint8_t  *)key;
    hostnameLen = strlen(dnsHostNameKey);
    return fnv_32a_hash(dnsHostNameKey, hostnameLen) % (pOH->hEntries);
}
#endif  // defined(OA_DOUBLE_HASH_PROBING)

#else
bool TCPIP_DNSS_IsEnabled(TCPIP_NET_HANDLE hNet){return false;}
bool TCPIP_DNSS_Enable(TCPIP_NET_HANDLE hNet){return false;}
bool TCPIP_DNSS_Disable(TCPIP_NET_HANDLE hNet){return false;}


#endif //#if defined(TCPIP_STACK_USE_DNS_SERVER)
