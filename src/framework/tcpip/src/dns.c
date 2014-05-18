/*******************************************************************************
  Domain Name System (DNS) Client 
  Module for Microchip TCP/IP Stack

  Summary:
    DNS client implementation file
    
  Description:
    This source file contains the functions of the 
    DNS client routines
    
    Provides  hostname to IP address translation
    Reference: RFC 1035
*******************************************************************************/

/*******************************************************************************
File Name:  DNS.c
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

#include "tcpip/src/tcpip_private.h"
#include "tcpip/src/dns_private.h"
#include "tcpip/src/hash_fnv.h"
#include "system/tmr/sys_tmr.h"

#if defined(TCPIP_STACK_USE_DNS)
#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_DNS_CLIENT
#include "tcpip_notify.h"

	
/****************************************************************************
  Section:
	Constants and Global Variables
  ***************************************************************************/

static const DNS_CLIENT_MODULE_CONFIG dnsDefaultConfigData = 
{
    true,
    DNS_CLIENT_CACHE_ENTRIES,
    DNS_CLIENT_CACHE_ENTRY_TMO,
    DNS_CLIENT_CACHE_PER_IPV4_ADDRESS,
#ifdef TCPIP_STACK_USE_IPV6
    DNS_CLIENT_CACHE_PER_IPV6_ADDRESS,
#endif	
    DNS_CLIENT_OPEN_ADDRESS_TYPE,
};

// This structure is used to collect the entries for DNS client use
typedef struct
{
//	DNS_HOSTNAME				sHostNameData;  //Host name size
    int                         nIPv4Entries;     // how many entries in the ip4Address[] array; if IPv4 is defined
    IPV4_ADDR                   ip4Address[DNS_CLIENT_CACHE_PER_IPV4_ADDRESS];    // provide a symbol for IPv4 entries number; 
    TCPIP_UINT32_VAL            ipTTL; //  Minimum TTL for all the IPv4 and Ipv6 address
#ifdef TCPIP_STACK_USE_IPV6
    int                         nIPv6Entries;     // how many entries in the ip6Address[] array; if IPv6 is defined
    IPV6_ADDR                   ip6Address[DNS_CLIENT_CACHE_PER_IPV6_ADDRESS];    // provide a symbol for IPv6 entries number
#endif
}DNS_CACHE_ENTRY;

static PROTECTED_SINGLE_LIST      dnsRegisteredUsers = { {0} };

#if defined (TCPIP_STACK_USE_DHCP_CLIENT)
static TCPIP_DHCP_HANDLE dnsDHCPHandler = NULL;
#endif
static DNS_DCPT gDnsDcpt;

/****************************************************************************
  Section:
	Function Prototypes
  ***************************************************************************/
static void DNS_DhcpNotify(TCPIP_NET_HANDLE hNet, uint8_t evType, const void * param);
static void _DNSNotifyClients(TCPIP_NET_IF* pNetIf, TCPIP_DNS_EVENT_TYPE evType,void *param);
static void _DNSPutString(DNSCLINTBUFFERDATA *putbuf, const char* String);
static void _DNSDiscardName(DNS_DCPT *pDnsDcpt);
static DNS_STATE _DNSRetry(DNS_STATE currState);
//static int DNSHashIPKeyCompare(OA_HASH_DCPT* pOH, OA_HASH_ENTRY* hEntry, void* key);

static  TCPIP_DNS_RESULT  _DNSSetHashEntry(DNS_HASH_ENTRY* dnsHE, DNS_HASH_ENTRY_FLAGS newFlags,DNS_CACHE_ENTRY dnsCacheEntry)
{
    int i =0;
    int memoryBlockSize=0;
    uint8_t *pMemoryBlock;
    
    dnsHE->hEntry.flags.value &= ~DNS_FLAG_ENTRY_VALID_MASK;
    dnsHE->hEntry.flags.value |= newFlags;

    memoryBlockSize = dnsCacheEntry.nIPv4Entries*sizeof(IPV4_ADDR)
#if defined(TCPIP_STACK_USE_IPV6)
        + dnsCacheEntry.nIPv6Entries*sizeof(IPV6_ADDR)
#endif        
        ;
	if(memoryBlockSize == 0)
	{
		return DNS_RES_NO_SERVICE;
	}
    pMemoryBlock = (uint8_t *)TCPIP_HEAP_Malloc(gDnsDcpt.memH,memoryBlockSize);
    if(pMemoryBlock == 0)
    {
        return DNS_RES_MEMORY_FAIL;
    }
    dnsHE->memblk = pMemoryBlock;
    if((dnsCacheEntry.nIPv4Entries != 0)
#if defined(TCPIP_STACK_USE_IPV6)
            && (dnsCacheEntry.nIPv6Entries!=0)
#endif
    )
    {
        dnsHE->recordType = IP_ADDRESS_TYPE_IPV4 | IP_ADDRESS_TYPE_IPV6;
    }
    else if(dnsCacheEntry.nIPv4Entries != 0)
    {
        dnsHE->recordType = IP_ADDRESS_TYPE_IPV4;
    }
#if defined(TCPIP_STACK_USE_IPV6)
    else if(dnsCacheEntry.nIPv6Entries != 0)
    {
        dnsHE->recordType = IP_ADDRESS_TYPE_IPV6;
    }
#endif    
    if((dnsHE->recordType & IP_ADDRESS_TYPE_IPV4) == IP_ADDRESS_TYPE_IPV4)
    {
        dnsHE->pip4Address = (IPV4_ADDR *)pMemoryBlock;
        if(dnsHE->pip4Address == 0)
            return DNS_RES_MEMORY_FAIL;
        dnsHE->nIPv4Entries = dnsCacheEntry.nIPv4Entries;
        for(i=0;i<dnsCacheEntry.nIPv4Entries;i++)
        {
            dnsHE->pip4Address[i].Val = dnsCacheEntry.ip4Address[i].Val;
        }
    }
#if defined(TCPIP_STACK_USE_IPV6)
    if((dnsHE->recordType & IP_ADDRESS_TYPE_IPV6) == IP_ADDRESS_TYPE_IPV6)
    {
        dnsHE->pip6Address = (IPV6_ADDR *)pMemoryBlock+dnsCacheEntry.nIPv4Entries*sizeof(IPV4_ADDR);
        if(dnsHE->pip6Address == 0)
            return DNS_RES_MEMORY_FAIL;
        dnsHE->nIPv6Entries = dnsCacheEntry.nIPv6Entries;
        for(i=0;i<dnsCacheEntry.nIPv6Entries;i++)
        {
            memcpy( &dnsHE->pip6Address[i],&dnsCacheEntry.ip6Address[i],sizeof(IPV6_ADDR));
        }
    }
#endif
    if(dnsCacheEntry.ipTTL.Val != 0)
        dnsHE->ipTTL.Val = dnsCacheEntry.ipTTL.Val;
    else
        dnsHE->ipTTL.Val = DNS_CACHE_DEFAULT_TTL_VAL;
    dnsHE->tInsert = gDnsDcpt.dnsTimeSeconds;
    return DNS_RES_OK;
}

static  void _DNSRemoveCacheEntries(DNS_HASH_DCPT* pDNSHashDcpt)
{
    OA_HASH_ENTRY*  pBkt;
    size_t      bktIx;
    DNS_HASH_ENTRY  *pE;
    DNS_DCPT        *pDnsDcpt;
    
    pDnsDcpt = &gDnsDcpt;
    if(pDNSHashDcpt->hashDcpt)
    {
        for(bktIx = 0; bktIx < pDNSHashDcpt->hashDcpt->hEntries; bktIx++)
        {
            pBkt = TCPIP_OAHASH_EntryGet(pDNSHashDcpt->hashDcpt, bktIx);
            if(pBkt->flags.busy != 0)
            {
                pE = (DNS_HASH_ENTRY*)pBkt;
                if(pDnsDcpt->cacheEntryTMO)
                {
                    if((pDnsDcpt->dnsTimeSeconds-pE->tInsert)/2 > pDnsDcpt->cacheEntryTMO)
                    {
                        TCPIP_HEAP_Free(pDnsDcpt->memH,pE->memblk);
                        pE->nIPv4Entries = 0;
                        pE->pip4Address = 0;
                        pE->ipTTL.Val = 0;
#ifdef TCPIP_STACK_USE_IPV6
                        pE->nIPv6Entries = 0;
                        pE->pip6Address = 0;
#endif
                        TCPIP_OAHASH_EntryRemove(pDNSHashDcpt->hashDcpt,pBkt);
    // Free DNS HOST Name
                        if(pE->pHostName)
                        {
                            TCPIP_HEAP_Free(pDnsDcpt->memH,pE->pHostName);
                            pE->pHostName = 0;
                        }

                    }
                }
                else  // if cacheEntryTMO is zero, then Response TTl will come into picture
                {
                
                    if((pDnsDcpt->dnsTimeSeconds-pE->tInsert)/2 > pE->ipTTL.Val)
                    {
                        TCPIP_HEAP_Free(pDnsDcpt->memH,pE->memblk);
                        pE->nIPv4Entries = 0;
                        pE->pip4Address = 0;
                        pE->ipTTL.Val= 0;
#ifdef TCPIP_STACK_USE_IPV6
                        pE->nIPv6Entries = 0;
                        pE->pip6Address = 0;
#endif
                        TCPIP_OAHASH_EntryRemove(pDNSHashDcpt->hashDcpt,pBkt);
    // Free DNS HOST Name
                        if(pE->pHostName)
                        {
                            TCPIP_HEAP_Free(pDnsDcpt->memH,pE->pHostName);
                            pE->pHostName = 0;
                        }
                    }
                }
            }
        }
    }
}

/*static __inline__*/static  void /*__attribute__((always_inline))*/ _DNSReleaseSocket(DNS_DCPT *dnsDcpt)
{
    if(dnsDcpt->dnsSocket != INVALID_UDP_SOCKET)
    {
        TCPIP_UDP_Close(dnsDcpt->dnsSocket);
        dnsDcpt->dnsSocket = INVALID_UDP_SOCKET;
    }
	//dnsDcpt->smState = DNS_IDLE;
}

/****************************************************************************
  Section:
	Implementation
  ***************************************************************************/


/*****************************************************************************
  Function:
	bool TCPIP_DNS_ClientInitialize(const TCPIP_STACK_MODULE_CTRL* const stackData,
                       const DNS_CLIENT_MODULE_GONFIG* dnsData);

  Summary:
	Initializes the DNS module.
	
  Description:
	This function perform the initialization of the DNS client module.
    It has to be called before any other operation with the DNS client
    is possible.

  Precondition:
	Stack is initialized.

  Parameters:
    stackData - stack initialization data

    dnsData   - DNS client module specific initialization data    

  Return Values:
  	true      - the initialization was performed OK and the module is ready to be used
  	false     - The DNS module initialization failed.
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_DNS_ClientInitialize(const TCPIP_STACK_MODULE_CTRL* const stackData,
                       const DNS_CLIENT_MODULE_CONFIG* dnsData)
{
    DNS_DCPT        *pDnsDcpt;
    OA_HASH_DCPT    *hashDcpt;
    size_t          hashMemSize;
	
    if(stackData->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }

    if(dnsData == 0)
    {
        dnsData =  &dnsDefaultConfigData;
    }
    pDnsDcpt = &gDnsDcpt;
    // stack start up
    if(pDnsDcpt->dnsInitCount == 0)
    {   // initialize just once

        pDnsDcpt->memH = stackData->memH;
        if(pDnsDcpt->dnsCacheDcpt.hashDcpt == 0)
        {
            hashMemSize = sizeof(OA_HASH_DCPT) + dnsData->cacheEntries * sizeof(DNS_HASH_ENTRY);
            hashDcpt = (OA_HASH_DCPT*)TCPIP_HEAP_Malloc(pDnsDcpt->memH,hashMemSize);
            if(hashDcpt == 0)
            {	// failed
                return false;
            }

            // populate the entries
            hashDcpt->memBlk = hashDcpt + 1;
            hashDcpt->hParam = hashDcpt;	// store the descriptor it belongs to
            hashDcpt->hEntrySize = sizeof(DNS_HASH_ENTRY);
            hashDcpt->hEntries = dnsData->cacheEntries;
            hashDcpt->probeStep = DNS_HASH_PROBE_STEP;
            
            hashDcpt->hashF = TCPIP_DNS_OAHASH_KeyHash;
            hashDcpt->delF = TCPIP_DNS_OAHASH_DeleteEntry;
            hashDcpt->cmpF = TCPIP_DNS_OAHASH_KeyCompare;
            hashDcpt->cpyF = TCPIP_DNS_OAHASH_KeyCopy;
#if defined(OA_DOUBLE_HASH_PROBING)
            hashDcpt->probeHash = TCPIP_DNS_OAHASH_ProbeHash;
#endif  // defined(OA_DOUBLE_HASH_PROBING)

            TCPIP_OAHASH_Initialize(hashDcpt);
            pDnsDcpt->dnsCacheDcpt.hashDcpt = hashDcpt;
            pDnsDcpt->smState = DNS_IDLE;
            pDnsDcpt->flags.Val = 0;
            pDnsDcpt->dnsSocket =  INVALID_UDP_SOCKET;
            pDnsDcpt->cacheEntryTMO = dnsData->entrySolvedTmo;
            pDnsDcpt->dnscacheEntries = dnsData->cacheEntries;
            pDnsDcpt->IPv4EntriesPerDNSName= dnsData->IPv4EntriesPerDNSName;
#ifdef TCPIP_STACK_USE_IPV6
            pDnsDcpt->IPv6EntriesPerDNSName = dnsData->IPv6EntriesPerDNSName;
#endif
            pDnsDcpt->dnsIpAddressType = dnsData->dnsIpAddressType;
            pDnsDcpt->vDNSServerIx = 0;
            pDnsDcpt->pDNSNet = (TCPIP_NET_IF*)TCPIP_STACK_NetHandleGet(DNS_DEFAULT_IF);
            if(pDnsDcpt->pDNSNet == 0)
            {
                pDnsDcpt->pDNSNet = (TCPIP_NET_IF*)TCPIP_STACK_NetDefaultGet();
            }
            if(pDnsDcpt->pDNSNet == 0)
            {
                pDnsDcpt->DNSServers[0].v4Add.Val = 0;
                pDnsDcpt->DNSServers[1].v4Add.Val = 0;
            }
            else
            {
                pDnsDcpt->DNSServers[0].v4Add.Val = pDnsDcpt->pDNSNet->DefaultDNSServer.Val;
                pDnsDcpt->DNSServers[1].v4Add.Val = pDnsDcpt->pDNSNet->DefaultDNSServer2.Val;
            }
            pDnsDcpt->dnsTimeSeconds=0;
        }
        TCPIP_Helper_ProtectedSingleListInitialize(&dnsRegisteredUsers);
#if defined (TCPIP_STACK_USE_DHCP_CLIENT)
        dnsDHCPHandler = TCPIP_DHCP_HandlerRegister(0, (TCPIP_DHCP_EVENT_HANDLER)DNS_DhcpNotify, NULL);
        if (dnsDHCPHandler == NULL)
        {
            TCPIP_HEAP_Free(pDnsDcpt->memH, pDnsDcpt->dnsCacheDcpt.hashDcpt);
            return false;
        }
#endif
    }


    if(pDnsDcpt->dnsTimerHandle == 0)
    {  
    // once per service
        pDnsDcpt->dnsTimerHandle = _TCPIPStackAsyncHandlerRegister(TCPIP_DNS_CacheTimeTask, 0, DNS_CLIENT_TASK_PROCESS_RATE*1000);
        if(pDnsDcpt->dnsTimerHandle == 0)
        {
            TCPIP_HEAP_Free(pDnsDcpt->memH, pDnsDcpt->dnsCacheDcpt.hashDcpt);
            return false;
        }        
    }
	
    pDnsDcpt->dnsInitCount++;
    return true;
}

static void _DNSClientDeleteCache(DNS_DCPT* pDnsDcpt)
{
    DNS_HASH_DCPT *pDnsHashDcpt;
    pDnsHashDcpt = &pDnsDcpt->dnsCacheDcpt;
    if(pDnsHashDcpt)
    {
        _DNSRemoveCacheEntries(pDnsHashDcpt);
        TCPIP_HEAP_Free(pDnsDcpt->memH, pDnsHashDcpt->hashDcpt);
        pDnsHashDcpt->hashDcpt = 0;
    }
	
    if( pDnsDcpt->dnsTimerHandle)
    {
       _TCPIPStackAsyncHandlerDeRegister( pDnsDcpt->dnsTimerHandle);
        pDnsDcpt->dnsTimerHandle = 0;
        pDnsDcpt->dnsTimeSeconds = 0;
    }

}


/*****************************************************************************
  Function:
    void TCPIP_DNS_ClientDeinitialize(const TCPIP_STACK_MODULE_CTRL* const stackData);

  Summary:
    De-Initializes the DNS module.
	
  Description:
    This function perform the de-initialization of the DNS client module.
    It is used to release all the resources that are in use by the DNS client.
    
  Precondition:
    Stack is initialized.

  Parameters:
    stackData   - interface to use normally should be a default DNS interface
                
  Return Values:
    None
  	
  Remarks:
    None
  ***************************************************************************/
void TCPIP_DNS_ClientDeinitialize(const TCPIP_STACK_MODULE_CTRL* const stackData)
{
    DNS_DCPT 	   *pDnsDcpt;

    pDnsDcpt = &gDnsDcpt;
    // interface going down
    if(pDnsDcpt->pDNSNet != stackData->pNetIf)
    {   // my interface is shut down
        return;
    }
    TCPIP_Notification_RemoveAll(&dnsRegisteredUsers, pDnsDcpt->memH);
#if defined   ( TCPIP_STACK_USE_DHCP_CLIENT)
    if (dnsDHCPHandler != NULL)
    {
        TCPIP_DHCP_HandlerDeRegister(dnsDHCPHandler);
        dnsDHCPHandler = NULL;
    }
#endif
  
    if(stackData->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // stack shut down
        if(pDnsDcpt->dnsInitCount > 0)
        {   // we're up and running
            if(--pDnsDcpt->dnsInitCount == 0)
            {   // all closed and Release DNS client Hash resources
                _DNSClientDeleteCache(pDnsDcpt);
            }
        }
    }
}

TCPIP_DNS_RESULT TCPIP_DNS_Resolve(const char* hostName, DNS_RESOLVE_TYPE type)
{
    DNS_DCPT*		pDnsDcpt;
    DNS_HASH_DCPT*   pDnsHashDcpt;
    DNS_HASH_ENTRY*	 dnsHashEntry;
    OA_HASH_ENTRY*   hE;
    int             hostNameSize=0;
    DNS_CACHE_ENTRY ResolvedDNSEntry;
    TCPIP_UINT16_VAL transactionId;
    pDnsDcpt = &gDnsDcpt;
    pDnsHashDcpt = &pDnsDcpt->dnsCacheDcpt;
    hostNameSize = strlen(hostName);
    transactionId.Val = 0;
    hE = TCPIP_OAHASH_EntryLookup(pDnsHashDcpt->hashDcpt,(void*) hostName);
    if(hE != 0)
    {
        dnsHashEntry = (DNS_HASH_ENTRY*)hE;
        if(dnsHashEntry->hEntry.flags.value & DNS_FLAG_ENTRY_COMPLETE)
        {
            if((dnsHashEntry->nIPv4Entries>0) && (dnsHashEntry->pip4Address != 0))
            {
                //pDnsDcpt->smState = DNS_DONE;
                pDnsDcpt->flags.bits.AddressType = IP_ADDRESS_TYPE_IPV4;
                pDnsDcpt->flags.bits.AddressValid = true;
            }
#if defined (TCPIP_STACK_USE_IPV6)
            else if((dnsHashEntry->nIPv6Entries>0) && (dnsHashEntry->pip6Address != 0))
            {
                pDnsDcpt->flags.bits.AddressValid = true;
                pDnsDcpt->flags.bits.AddressType = IP_ADDRESS_TYPE_IPV6;
                //pDnsDcpt->smState = DNS_DONE;
            }
#endif
        }
        else
        {
            pDnsDcpt->smState = DNS_GET_RESULT;
        }

    }
    else
    {
        /*
        - check hostname , if this one is part of HASH table. if present , then
        - if
        */

        ResolvedDNSEntry.ipTTL.Val = 0;
        ResolvedDNSEntry.nIPv4Entries = 0;
#ifdef TCPIP_STACK_USE_IPV6            
        ResolvedDNSEntry.nIPv6Entries = 0;
#endif
        if(TCPIP_Helper_StringToIPAddress(hostName, &ResolvedDNSEntry.ip4Address[ResolvedDNSEntry.nIPv4Entries]))
        {
            if(TCPIP_DNS_InsertHostName((uint8_t *)hostName,transactionId.Val,DNS_TYPE_A, 0)!= DNS_RES_OK)
            {
                pDnsDcpt->smState = DNS_FAIL_HASH_NO_ENTRY;
                return DNS_RES_NO_SERVICE;
            }
            hE = TCPIP_OAHASH_EntryLookup(pDnsHashDcpt->hashDcpt,(void*)hostName);
            if(hE == 0)
            {
                return DNS_RES_CACHE_FULL;
            }
            dnsHashEntry = (DNS_HASH_ENTRY*)hE;
            pDnsDcpt->flags.bits.AddressValid = true;
            pDnsDcpt->flags.bits.AddressType = IP_ADDRESS_TYPE_IPV4;
            ResolvedDNSEntry.nIPv4Entries=1;

            _DNSSetHashEntry(dnsHashEntry,DNS_FLAG_ENTRY_COMPLETE,ResolvedDNSEntry);
            return DNS_RES_OK;
        }
#if defined (TCPIP_STACK_USE_IPV6)
        else if (TCPIP_Helper_StringToIPv6Address (hostName, &ResolvedDNSEntry.ip6Address[ResolvedDNSEntry.nIPv6Entries]))
        {
            if(TCPIP_DNS_InsertHostName((uint8_t *)hostName,transactionId.Val,DNS_TYPE_AAAA,0)!= DNS_RES_OK)
            {
                pDnsDcpt->smState = DNS_FAIL_HASH_NO_ENTRY;
                return DNS_RES_NO_SERVICE;
            }
            hE = TCPIP_OAHASH_EntryLookupOrInsert(pDnsHashDcpt->hashDcpt,(void*)hostName);
            if(hE == 0)
            {
                return DNS_RES_CACHE_FULL;
            }
            dnsHashEntry = (DNS_HASH_ENTRY*)hE;
            pDnsDcpt->flags.bits.AddressValid = true;
            pDnsDcpt->flags.bits.AddressType = IP_ADDRESS_TYPE_IPV6;
            //pDnsDcpt->smState = DNS_DONE;
            ResolvedDNSEntry.nIPv6Entries=1;

            _DNSSetHashEntry(dnsHashEntry,DNS_FLAG_ENTRY_COMPLETE,ResolvedDNSEntry);
            return DNS_RES_OK;
        }
#endif
		
        // calculate DNS query transaction ID
        transactionId.Val = (uint16_t)SYS_RANDOM_PseudoGet();
        pDnsDcpt->pDnsHostName = (uint8_t*)hostName;
        // insert DNS hostname to the hash entry with DNS INCOMPLEATE FLAG
        if(TCPIP_DNS_InsertHostName(pDnsDcpt->pDnsHostName,transactionId.Val,type,0)!= DNS_RES_OK)
        {
            pDnsDcpt->smState = DNS_FAIL_HASH_NO_ENTRY;
            return false;
        }
        if(pDnsDcpt->dnsSocket != INVALID_UDP_SOCKET)
        {
            pDnsDcpt->smState = DNS_QUERY;            
            //pDnsDcpt->stateStartTime = SYS_TMR_TickCountGet();
        }
        else
        {
            pDnsDcpt->smState = DNS_START;
        }        
        pDnsDcpt->recordType = type;
        pDnsDcpt->flags.bits.AddressValid = false;           
    }
		
    return DNS_RES_OK;
}

TCPIP_DNS_RESULT TCPIP_DNS_GetIPv4Address(const char* hostName,int index,IPV4_ADDR* ipv4Addr)
{
    DNS_DCPT*		pDnsDcpt;
    DNS_HASH_DCPT*   pDnsHashDcpt;
    DNS_HASH_ENTRY*	 dnsHashEntry;
    OA_HASH_ENTRY*   hE;
    int             hostNameSize=0;
    

    pDnsDcpt = &gDnsDcpt;
    pDnsHashDcpt = &pDnsDcpt->dnsCacheDcpt;
    hostNameSize = strlen(hostName);
    if(ipv4Addr == NULL)
    {
        return DNS_RES_NO_SERVICE;
    }
    hE = TCPIP_OAHASH_EntryLookup(pDnsHashDcpt->hashDcpt,(void*) hostName);
    if(hE != 0)
    {
        if(hE->flags.value & DNS_FLAG_ENTRY_COMPLETE)
        {
            dnsHashEntry = (DNS_HASH_ENTRY*)hE;
            if((dnsHashEntry->recordType & IP_ADDRESS_TYPE_IPV4) == IP_ADDRESS_TYPE_IPV4)
            {
                if(index > dnsHashEntry->nIPv4Entries)
                {
                    return DNS_RES_NO_SERVICE;
                }
                ipv4Addr->Val = dnsHashEntry->pip4Address[index].Val;
            }
        }
    }
    return DNS_RES_OK;
}

#ifdef TCPIP_STACK_USE_IPV6
TCPIP_DNS_RESULT TCPIP_DNS_GetIPv6Address(const char* hostName,int index,IPV6_ADDR* ipv6Addr)
{
    DNS_DCPT*		pDnsDcpt;
    DNS_HASH_DCPT*   pDnsHashDcpt;
    DNS_HASH_ENTRY*	 dnsHashEntry;
    OA_HASH_ENTRY*   hE;
    int             hostNameSize=0;
    

    pDnsDcpt = &gDnsDcpt;
    pDnsHashDcpt = &pDnsDcpt->dnsCacheDcpt;
    hostNameSize = strlen(hostName);
    if(ipv6Addr == NULL)
    {
        return DNS_RES_NO_SERVICE;
    }
    hE = TCPIP_OAHASH_EntryLookup(pDnsHashDcpt->hashDcpt,(void*) hostName);
    if(hE != 0)
    {
        if(hE->flags.value & DNS_FLAG_ENTRY_COMPLETE)
        {
            dnsHashEntry = (DNS_HASH_ENTRY*)hE;
            if((dnsHashEntry->recordType & IP_ADDRESS_TYPE_IPV6) == IP_ADDRESS_TYPE_IPV6)
            {
                if(index > dnsHashEntry->nIPv6Entries)
                {
                    return DNS_RES_NO_SERVICE;
                }
                memcpy (ipv6Addr, dnsHashEntry->pip6Address+index, sizeof (IPV6_ADDR));
            }
        }
    }
    return DNS_RES_OK;
}

#endif

int TCPIP_DNS_GetNumberOfIPAddresses(const char* hostName,IP_ADDRESS_TYPE type)
{
    DNS_DCPT*		pDnsDcpt;
    DNS_HASH_DCPT*   pDnsHashDcpt;
    DNS_HASH_ENTRY*	 dnsHashEntry;
    OA_HASH_ENTRY*   hE;
    int             hostNameSize=0;
    

    pDnsDcpt = &gDnsDcpt;
    pDnsHashDcpt = &pDnsDcpt->dnsCacheDcpt;
    hostNameSize = strlen(hostName);

    hE = TCPIP_OAHASH_EntryLookup(pDnsHashDcpt->hashDcpt,(void*) hostName);
    if(hE != 0)
    {
        if(hE->flags.value & DNS_FLAG_ENTRY_COMPLETE)
        {
            dnsHashEntry = (DNS_HASH_ENTRY*)hE;
            if((dnsHashEntry->recordType & IP_ADDRESS_TYPE_IPV4) == IP_ADDRESS_TYPE_IPV4)
            {
                 return dnsHashEntry->nIPv4Entries;
            }
#ifdef TCPIP_STACK_USE_IPV6            
            else if((dnsHashEntry->recordType & IP_ADDRESS_TYPE_IPV6) == IP_ADDRESS_TYPE_IPV6)
            {
                 return dnsHashEntry->nIPv6Entries;
            }
#endif
        }
    }
    return 0;
}

TCPIP_DNS_RESULT TCPIP_DNS_IsResolved(const char* hostName, void * ipAddr)
{    
    DNS_DCPT*		pDnsDcpt;
    OA_HASH_ENTRY*	hE;
    int             hostNameSize=0;	
    DNS_HASH_ENTRY* pDnsHE;
    TCPIP_DNS_RESOLVED_SERVERIPADDRESS serverIpAddress;

    if(ipAddr == NULL)
    {
        return DNS_RES_NO_SERVICE;
    }
	
    pDnsDcpt = &gDnsDcpt;
    hostNameSize =  strlen(hostName);
    hE = TCPIP_OAHASH_EntryLookup(pDnsDcpt->dnsCacheDcpt.hashDcpt,(void*)hostName);
    if(hE == 0)
    {
        return DNS_RES_NO_ENTRY;
    }
    if((hE->flags.value & DNS_FLAG_ENTRY_INCOMPLETE) == DNS_FLAG_ENTRY_INCOMPLETE)
    {
        if(pDnsDcpt->smState < DNS_DONE)
        {
            return DNS_RES_PENDING;
        }
        // some kind of error
        switch (pDnsDcpt->smState)
        {
            case DNS_FAIL_ARP_TMO:
                return DNS_RES_ARP_TMO; 
        
            case DNS_FAIL_OPEN_TMO:
                return DNS_RES_OPEN_TMO; 
        
            default:    // DNS_FAIL_SERVER_TMO:
                return DNS_RES_SERVER_TMO; 
        }
    }

    if((hE->flags.value & DNS_FLAG_ENTRY_COMPLETE) == DNS_FLAG_ENTRY_COMPLETE)
    {
    	pDnsHE = (DNS_HASH_ENTRY*)hE;

#if defined (TCPIP_STACK_USE_IPV6)
        if (pDnsHE->recordType == IP_ADDRESS_TYPE_IPV6)
        {
            serverIpAddress.nIPv6Entries = TCPIP_DNS_GetNumberOfIPAddresses(hostName,IP_ADDRESS_TYPE_IPV6);
            TCPIP_DNS_GetIPv6Address(hostName,serverIpAddress.nIPv6Entries-1,&serverIpAddress.ip6Address);
            memcpy (ipAddr, &serverIpAddress.ip6Address, sizeof (IPV6_ADDR));
        }
        else
#endif
//  Get the valid Ipv4 address.
        {
            serverIpAddress.nIPv4Entries = TCPIP_DNS_GetNumberOfIPAddresses(hostName,IP_ADDRESS_TYPE_IPV4);
            serverIpAddress.ip4Address.Val = 0;
            TCPIP_DNS_GetIPv4Address(hostName,serverIpAddress.nIPv4Entries-1,&serverIpAddress.ip4Address);
            ((IPV4_ADDR*)ipAddr)->Val = serverIpAddress.ip4Address.Val;
        }
    }
    
    return DNS_RES_OK;
}

TCPIP_DNS_RESULT TCPIP_DNS_InsertHostName(uint8_t *pDnsHostName, uint16_t transactionId,DNS_RESOLVE_TYPE type,int intfIdx)
{
    size_t hostNameSize=0;
    DNS_DCPT*	pDnsDcpt;
    uint8_t*    ptrHostName;
    OA_HASH_ENTRY*  hE;
    DNS_HASH_ENTRY* pDnsHE;

    pDnsDcpt = &gDnsDcpt;
    hostNameSize = strlen((const char*)pDnsHostName);
    ptrHostName = (uint8_t *)TCPIP_HEAP_Calloc(pDnsDcpt->memH,1,hostNameSize+1);
    if(ptrHostName == 0)
    {
        return DNS_RES_MEMORY_FAIL;
    }
    memcpy(ptrHostName,pDnsHostName,hostNameSize);
    ptrHostName[hostNameSize]='\0';
    hE = TCPIP_OAHASH_EntryLookupOrInsert(pDnsDcpt->dnsCacheDcpt.hashDcpt,ptrHostName);
    if(hE == 0)
    {
        return DNS_RES_CACHE_FULL;
    }
    
    hE->flags.value &= ~DNS_FLAG_ENTRY_VALID_MASK;
    hE->flags.value |= DNS_FLAG_ENTRY_INCOMPLETE;

    pDnsHE = (DNS_HASH_ENTRY*)hE;
    pDnsHE->tInsert = pDnsDcpt->dnsTimeSeconds;
    pDnsHE->SentTransactionID.Val = transactionId;
    pDnsHE->netIfIdx = intfIdx;
	pDnsHE->resolve_type = type;
    return DNS_RES_OK;
}

DNS_HASH_ENTRY *_DNSHashEntryFromTransactionId(TCPIP_UINT16_VAL transactionId)
{
    DNS_HASH_ENTRY* pDnsHE;
    DNS_HASH_DCPT  *pDnsHashDcpt;
    OA_HASH_ENTRY   *hE;
    int             bktIx=0;
    OA_HASH_DCPT    *pOH;
    DNS_DCPT       *pDnsDcpt;
    
    pDnsDcpt = &gDnsDcpt;
    pDnsHashDcpt = &pDnsDcpt->dnsCacheDcpt;
    pOH = pDnsHashDcpt->hashDcpt;

    for(bktIx = 0; bktIx < pOH->hEntries; bktIx++)
    {
        hE = TCPIP_OAHASH_EntryGet(pOH, bktIx);
        if(hE->flags.busy != 0)
        {
            pDnsHE = (DNS_HASH_ENTRY*)hE;
            if(pDnsHE->SentTransactionID.Val == transactionId.Val)
            {
                return pDnsHE;
            }
        }
    }

    return 0;
}

void TCPIP_DNS_CacheTimeTask(void)
{
    DNS_HASH_ENTRY  *pDnsHE;
    DNS_HASH_DCPT   *pDnsHashDcpt;
    OA_HASH_ENTRY   *hE;
    int             bktIx=0;
    OA_HASH_DCPT    *pOH;
    DNS_DCPT        *pDnsDcpt;
    TCPIP_NET_IF    *pNetIf;

    
    pDnsDcpt = &gDnsDcpt;
    pDnsHashDcpt = &pDnsDcpt->dnsCacheDcpt;
    pOH = pDnsHashDcpt->hashDcpt;
    if(pDnsDcpt->dnsSocket == INVALID_UDP_SOCKET)
    {
        return;
    }
    pNetIf = (TCPIP_NET_IF*)TCPIP_UDP_SocketNetGet(pDnsDcpt->dnsSocket);
    pDnsDcpt->dnsTimeSeconds += DNS_CLIENT_TASK_PROCESS_RATE;

// check the lease values and if there is any entry whose lease value exceeds the lease duration remove the lease entries from the HASH.

    for(bktIx = 0; bktIx < pOH->hEntries; bktIx++)
    {
        hE = TCPIP_OAHASH_EntryGet(pOH, bktIx);
    	if((hE->flags.busy != 0) && (hE->flags.value & DNS_FLAG_ENTRY_COMPLETE))
    	{
            pDnsHE = (DNS_HASH_ENTRY*)hE;
            // if cacheEntryTMO is not equal to zero, then TTL time is the timeout period. 
            // TTL time may be a  higher value , so it i sbetter to use a cacheEntryTMO
            if(pDnsDcpt->cacheEntryTMO > 0)
            {
                if((pDnsDcpt->dnsTimeSeconds-pDnsHE->tInsert) > pDnsDcpt->cacheEntryTMO)
                {
                    pDnsHE->tInsert = 0;
					pDnsHE->nIPv4Entries = 0;
                    pDnsHE->pip4Address = 0;            
                    pDnsHE->ipTTL.Val = 0;
#ifdef TCPIP_STACK_USE_IPV6
                    pDnsHE->nIPv6Entries = 0;
                    pDnsHE->pip6Address = 0;
#endif
                    // Notify to to the specific Host name that it is expired.
                    _DNSNotifyClients(0,TCPIP_DNS_EVENT_CACHE_EXPIRED,pDnsHE->pHostName);
                    TCPIP_OAHASH_EntryRemove(pOH,hE);
					
                    // free Memory for hostname
                    if(pDnsHE->pHostName)
                    {
                        TCPIP_HEAP_Free(pDnsDcpt->memH,pDnsHE->pHostName);
                        //pDnsHE->hostNameData.size = 0;
                    }
                    // Free IPv4 and IPv6 address       
                    TCPIP_HEAP_Free(pDnsDcpt->memH,pDnsHE->memblk);
                    
                }
            }
            else
            {
                if((pDnsDcpt->dnsTimeSeconds-pDnsHE->tInsert) > pDnsHE->ipTTL.Val)
                {
                    TCPIP_HEAP_Free(pDnsDcpt->memH,pDnsHE->memblk);
                    pDnsHE->nIPv4Entries = 0;
                    pDnsHE->pip4Address = 0;
                    pDnsHE->ipTTL.Val = 0;
#ifdef TCPIP_STACK_USE_IPV6
                    pDnsHE->nIPv6Entries = 0;
                    pDnsHE->pip6Address = 0;
#endif
                    // Notify to to the specific Host name that it is expired.
                    _DNSNotifyClients(0,TCPIP_DNS_EVENT_CACHE_EXPIRED,pDnsHE->pHostName);
                    TCPIP_OAHASH_EntryRemove(pOH,hE);
// Free DNS HOST Name
                    if(pDnsHE->pHostName)
                    {
                        TCPIP_HEAP_Free(pDnsDcpt->memH,pDnsHE->pHostName);
                        pDnsHE->pHostName = 0;
                    }       
                    
                }
            }    			
    	}
        else if((hE->flags.busy != 0) && (hE->flags.value & DNS_FLAG_ENTRY_INCOMPLETE))
        {
            pDnsHE = (DNS_HASH_ENTRY*)hE;
            if((pDnsDcpt->dnsTimeSeconds - pDnsHE->tInsert)>DNS_CACHE_UNSOLVED_ENTRY_TMO)
            {
                pDnsHE->tInsert = 0;
                // Notify to to the specific Host name that it is expired.
                _DNSNotifyClients(0,TCPIP_DNS_EVENT_CACHE_EXPIRED,pDnsHE->pHostName);
                TCPIP_OAHASH_EntryRemove(pOH,hE);
                if(pDnsHE->pHostName)
                {
                    TCPIP_HEAP_Free(pDnsDcpt->memH,pDnsHE->pHostName);
                }
            }
        }
    }
 
}

/****************************************************************************
  Function:
    static bool _DNSCopyDataToProcessBuff(uint8_t val ,DNSCLINTBUFFERDATA *putbuf)
	
  Summary:
  	Copies uint8_t data to dynamically allocated memory buffer.

  Description:
		  	 		 		  	
  Precondition:
   	
  Parameters:
  	val: uint8_t value to be written to the buffer
  	putbuf: pointer to the dynamically allocated buffer to which the 'val' to be written 
  	
  Return Values:
	true: if successfully write to the buffer
	false: failure in writing to the buffer
	
  Remarks:
  	This routine is used by the DNS Client stack. If required to be used by the application
  	code, valid pointers should be passed to this routine. 
  	
***************************************************************************/
static bool _DNSCopyDataToProcessBuff(uint8_t val ,DNSCLINTBUFFERDATA *putbuf)
{
    if(putbuf->length < putbuf->maxlength)
    {
        putbuf->head[putbuf->length] = (uint8_t)val;
        putbuf->length++;
        return true;
    }

    return false;
}

static bool _DNSCopyDataArrayToProcessBuff(uint8_t *val ,DNSCLINTBUFFERDATA *putbuf,int len)
{
    uint8_t pos=0;

    for(pos = 0;pos<len;pos++)
    {
        if(putbuf->length < putbuf->maxlength)
        {
            putbuf->head[putbuf->length] = (uint8_t)(*(val+pos));
            putbuf->length++;
        }
        else
        {
            return false;
        }
    }

    return true;

}

static uint8_t _DNSGetDataFromUDPBuff(uint8_t pos ,DNSCLINTBUFFERDATA *getbuf)
{
    return (uint8_t)(getbuf->head[pos]);
}

#ifdef TCPIP_STACK_USE_IPV6
static uint8_t _DNSGetDataArrayFromUDPBuff(uint8_t pos,DNSCLINTBUFFERDATA *getBuf,int len,uint8_t *destBuf)
{
    int     i =0;
    if((destBuf == 0) || (getBuf->head == 0))
        return 0;
    if(pos>len || getBuf->maxlength < len)
        return 0;

    for(i=0;i<len;i++)
    {
        destBuf[i] = getBuf->head[pos++];
    }
	
    return pos;

}
#endif  // TCPIP_STACK_USE_IPV6

bool TCPIP_DNS_ClientTask(TCPIP_NET_IF* pNewIf)
{
    uint8_t 		i;
    TCPIP_UINT16_VAL	w;
    DNS_HEADER		DNSHeader;
    DNS_ANSWER_HEADER	DNSAnswerHeader;
    DNS_DCPT*		pDnsDcpt;
    OA_HASH_ENTRY*	hE;
    DNS_HASH_ENTRY* 	dnshE;
    int 		replyPktSize=0,getBufferPktSize=0;
    DNS_CACHE_ENTRY 	ResolvedDNSEntry;
    int 		netIx;
    int 		udpPutBytes;
    int 		NoofCachePerDnsAddress=0;
    int 		ipv6Size=0;
    static uint32_t	bktIdx=0;
    int			tempTimeOut=0;

    ipv6Size = sizeof(IPV6_ADDR);

    pDnsDcpt = &gDnsDcpt;

    if(pNewIf == 0 || !TCPIP_STACK_NetworkIsUp(pNewIf))
    {	// try a default interface
        if(TCPIP_STACK_NetworkIsUp(pDnsDcpt->pDNSNet))
        {
            pNewIf = pDnsDcpt->pDNSNet;
        }
        else
        {
            pNewIf = (TCPIP_NET_IF*)TCPIP_STACK_NetDefaultGet();
            if(!TCPIP_STACK_NetworkIsUp(pNewIf))
            {
               pNewIf = 0;
            }
        }
    }
    if((pNewIf == 0) || (pNewIf->netIPAddr.Val == 0))
        return false;

// check if DNS client is enabled in this nterface
    if(!TCPIP_DNS_IsEnabled(pNewIf))
    {
        return false;
    }

    pDnsDcpt->pDNSNet = pNewIf;
    netIx = TCPIP_STACK_NetIndexGet(pDnsDcpt->pDNSNet);

    switch(pDnsDcpt->smState)
    {
        case DNS_IDLE:
           // pDnsDcpt->smState = DNS_START;
            break;	// nothing to do
        case DNS_START:
            pDnsDcpt->smState = _DNSRetry(DNS_START);
            pDnsDcpt->stateStartTime = 0;  // flag the first Open try
            break;
        case DNS_OPEN_SOCKET:
            if(pDnsDcpt->dnsSocket == INVALID_UDP_SOCKET)
            {
                pDnsDcpt->dnsSocket = TCPIP_UDP_ClientOpen(pDnsDcpt->dnsIpAddressType, TCPIP_DNS_CLIENT_PORT, pDnsDcpt->DNSServers + pDnsDcpt->vDNSServerIx);
            }
            if(pDnsDcpt->dnsSocket == INVALID_UDP_SOCKET)
            {
                if(pDnsDcpt->stateStartTime == 0)
                {
                    pDnsDcpt->stateStartTime = SYS_TMR_TickCountGet();
                }
                else if((SYS_TMR_TickCountGet() - pDnsDcpt->stateStartTime)/SYS_TMR_TickPerSecond() >= DNS_CLIENT_OPEN_TMO )
                {
                    pDnsDcpt->smState = DNS_FAIL_OPEN_TMO;
                }
                break;
            }
            else
            {
                TCPIP_UDP_SocketNetSet(pDnsDcpt->dnsSocket, pDnsDcpt->pDNSNet);
            }
            // got a valid UDP socket
            if(TCPIP_UDP_IsOpened(pDnsDcpt->dnsSocket))
            {
                pDnsDcpt->stateStartTime = SYS_TMR_TickCountGet();
            }
            else
            {
                break;
            }
            pDnsDcpt->smState = DNS_QUERY;
            // no break, start sending the query;

        case DNS_QUERY:
            if(bktIdx >= pDnsDcpt->dnsCacheDcpt.hashDcpt->hEntries)
            {
                 bktIdx = 0;
            }

            hE = TCPIP_OAHASH_EntryGet(pDnsDcpt->dnsCacheDcpt.hashDcpt, bktIdx++);
            if((hE->flags.busy != 0) && (hE->flags.value & DNS_FLAG_ENTRY_COMPLETE))
            {
                pDnsDcpt->smState = DNS_GET_RESULT;
                break;
            }
            else if(hE->flags.busy == 0)
            {
                break;
            }

            // Incompleate hash Entry Need to send query for this
            dnshE = (DNS_HASH_ENTRY*)hE;
            udpPutBytes = TCPIP_UDP_TxPutIsReady(pDnsDcpt->dnsSocket, 18 + strlen((const char*)dnshE->pHostName)+1);
            if(! TCPIP_UDP_IsOpened(pDnsDcpt->dnsSocket) || ( udpPutBytes < (18 + strlen((const char*)dnshE->pHostName)+1)))
            {
                if((SYS_TMR_TickCountGet() - pDnsDcpt->stateStartTime)/SYS_TMR_TickPerSecond() >= DNS_CLIENT_OPEN_TMO )
                {
                    pDnsDcpt->smState = DNS_FAIL_OPEN_TMO;
                }
                break;	// wait some more
            }

            if(pDnsDcpt->udpPutBufferData.head == 0)
            {
                pDnsDcpt->udpPutBufferData.head = (uint8_t *)(TCPIP_HEAP_Malloc(pDnsDcpt->memH,udpPutBytes));
                if(pDnsDcpt->udpPutBufferData.head == 0)
                {
                    pDnsDcpt->smState = DNS_FAIL_NO_MEM;
                    return false;
                }
                pDnsDcpt->udpPutBufferData.length = 0;
                pDnsDcpt->udpPutBufferData.maxlength = udpPutBytes;
            }
    // Put DNS query here
            _DNSCopyDataToProcessBuff(dnshE->SentTransactionID.v[1],&pDnsDcpt->udpPutBufferData); // User chosen transaction ID
            _DNSCopyDataToProcessBuff(dnshE->SentTransactionID.v[0],&pDnsDcpt->udpPutBufferData);
            _DNSCopyDataToProcessBuff(0x01,&pDnsDcpt->udpPutBufferData); // Standard query with recursion
            _DNSCopyDataToProcessBuff(0x00,&pDnsDcpt->udpPutBufferData);
            _DNSCopyDataToProcessBuff(0x00,&pDnsDcpt->udpPutBufferData); // 0x0001 questions
            _DNSCopyDataToProcessBuff(0x01,&pDnsDcpt->udpPutBufferData);
            _DNSCopyDataToProcessBuff(0x00,&pDnsDcpt->udpPutBufferData); // 0x0000 answers
            _DNSCopyDataToProcessBuff(0x00,&pDnsDcpt->udpPutBufferData);
            _DNSCopyDataToProcessBuff(0x00,&pDnsDcpt->udpPutBufferData); // 0x0000 name server resource records
            _DNSCopyDataToProcessBuff(0x00,&pDnsDcpt->udpPutBufferData);
            _DNSCopyDataToProcessBuff(0x00,&pDnsDcpt->udpPutBufferData); // 0x0000 additional records
            _DNSCopyDataToProcessBuff(0x00,&pDnsDcpt->udpPutBufferData);
            // Put hostname string to resolve
            _DNSPutString(&pDnsDcpt->udpPutBufferData, (const char*)dnshE->pHostName);
            _DNSCopyDataToProcessBuff(0x00,&pDnsDcpt->udpPutBufferData); // Type: DNS_TYPE_A A (host address) or DNS_TYPE_MX for mail exchange
            _DNSCopyDataToProcessBuff(dnshE->resolve_type,&pDnsDcpt->udpPutBufferData);
            _DNSCopyDataToProcessBuff(0x00,&pDnsDcpt->udpPutBufferData); // Class: IN (Internet)
            _DNSCopyDataToProcessBuff(0x01,&pDnsDcpt->udpPutBufferData);
            // Put complete DNS query packet buffer to the UDP buffer
            TCPIP_UDP_ArrayPut(pDnsDcpt->dnsSocket, pDnsDcpt->udpPutBufferData.head, pDnsDcpt->udpPutBufferData.length);

            TCPIP_UDP_Flush(pDnsDcpt->dnsSocket);
            TCPIP_HEAP_Free(pDnsDcpt->memH,pDnsDcpt->udpPutBufferData.head);
            pDnsDcpt->udpPutBufferData.head = 0;
            pDnsDcpt->stateStartTime = SYS_TMR_TickCountGet();
            pDnsDcpt->smState = DNS_GET_RESULT;

            break;

        case DNS_GET_RESULT:
            replyPktSize = TCPIP_UDP_GetIsReady(pDnsDcpt->dnsSocket);

            if(!replyPktSize)
            {
                tempTimeOut = (SYS_TMR_TickCountGet() - pDnsDcpt->stateStartTime)/SYS_TMR_TickPerSecond();
                if(tempTimeOut >= DNS_CLIENT_SERVER_TMO)
                {
                    pDnsDcpt->smState = DNS_FAIL_SERVER;
                }
                break;
            }
            if(pDnsDcpt->udpGetBufferData.head == 0)
            {
                pDnsDcpt->udpGetBufferData.head = (uint8_t *)(TCPIP_HEAP_Malloc(pDnsDcpt->memH,replyPktSize+1));
                if(pDnsDcpt->udpGetBufferData.head == 0)
                {
                    pDnsDcpt->smState = DNS_FAIL_NO_MEM;
                    return false;
                }
                pDnsDcpt->udpGetBufferData.length = 0;
                pDnsDcpt->udpGetBufferData.maxlength = replyPktSize;
            }
            // Get Compleate Array of DNS Reply bytes
            getBufferPktSize = TCPIP_UDP_ArrayGet(pDnsDcpt->dnsSocket,pDnsDcpt->udpGetBufferData.head,replyPktSize);

// Retrieve the DNS header and de-big-endian it
            DNSHeader.TransactionID.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
            DNSHeader.TransactionID.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
            // find DNS HASH entry from transaction ID
            dnshE = _DNSHashEntryFromTransactionId(DNSHeader.TransactionID);
            // Throw this packet away if it isn't in response to our last query
            if(dnshE == 0)
            {
                TCPIP_UDP_Discard(pDnsDcpt->dnsSocket);
                pDnsDcpt->smState = DNS_QUERY;
                if(pDnsDcpt->udpGetBufferData.head)
                {
                    TCPIP_HEAP_Free(pDnsDcpt->memH,pDnsDcpt->udpGetBufferData.head);
                    pDnsDcpt->udpGetBufferData.head = 0;
                }
                break;
            }
            // If the Entry is already exist , No new Resolved address is required to be added
            if(dnshE->hEntry.flags.value & DNS_FLAG_ENTRY_COMPLETE)
            {
                TCPIP_UDP_Discard(pDnsDcpt->dnsSocket);
                if(pDnsDcpt->udpGetBufferData.head)
                {
                    TCPIP_HEAP_Free(pDnsDcpt->memH,pDnsDcpt->udpGetBufferData.head);
                    pDnsDcpt->udpGetBufferData.head = 0;
                }				
                pDnsDcpt->smState = DNS_QUERY;
                break;
            }
			
            ResolvedDNSEntry.ipTTL.Val = 0;
            ResolvedDNSEntry.nIPv4Entries = 0;
#ifdef TCPIP_STACK_USE_IPV6            
            ResolvedDNSEntry.nIPv6Entries = 0;
#endif

            DNSHeader.Flags.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
            DNSHeader.Flags.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
            DNSHeader.Questions.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
            DNSHeader.Questions.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
            DNSHeader.Answers.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
            DNSHeader.Answers.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
            DNSHeader.AuthoritativeRecords.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
            DNSHeader.AuthoritativeRecords.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
            DNSHeader.AdditionalRecords.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
            DNSHeader.AdditionalRecords.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
// No Such Name Return 

            if(DNSHeader.Flags.v[0]&0x03)
            {
                TCPIP_UDP_Discard(pDnsDcpt->dnsSocket);
                if(pDnsDcpt->udpGetBufferData.head)
                {
                    TCPIP_HEAP_Free(pDnsDcpt->memH,pDnsDcpt->udpGetBufferData.head);
                    pDnsDcpt->udpGetBufferData.head = 0;
                }
                break;
            }

// Remove all questions (queries)
            while(DNSHeader.Questions.Val--)
            {
                _DNSDiscardName(pDnsDcpt);
                w.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData); // Question type
                w.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                w.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData); // Question class
                w.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
            }

    // Scan through answers
            while(DNSHeader.Answers.Val--)
            {
                _DNSDiscardName(pDnsDcpt);					// Throw away response name
                DNSAnswerHeader.ResponseType.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData); // Response type
                DNSAnswerHeader.ResponseType.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                DNSAnswerHeader.ResponseClass.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData); // Response class
                DNSAnswerHeader.ResponseClass.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
        // Time to live
                DNSAnswerHeader.ResponseTTL.v[3] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                DNSAnswerHeader.ResponseTTL.v[2] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                DNSAnswerHeader.ResponseTTL.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                DNSAnswerHeader.ResponseTTL.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
        // Response length
                DNSAnswerHeader.ResponseLen.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                DNSAnswerHeader.ResponseLen.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);

        // Make sure that this is a 4 byte IP address, response type A or MX, class 1
        // Check if this is Type A, MX, or AAAA
                if( DNSAnswerHeader.ResponseClass.Val	== 0x0001u) // Internet class
                {
                    if (DNSAnswerHeader.ResponseType.Val    == 0x0001u &&
                                DNSAnswerHeader.ResponseLen.Val == 0x0004u)
                    {
                        pDnsDcpt->flags.bits.AddressValid = true;
                        pDnsDcpt->flags.bits.AddressType = IP_ADDRESS_TYPE_IPV4;
                        if(ResolvedDNSEntry.nIPv4Entries < pDnsDcpt->IPv4EntriesPerDNSName)
                        {
                            ResolvedDNSEntry.ip4Address[ResolvedDNSEntry.nIPv4Entries].v[0] =
                                    _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                            ResolvedDNSEntry.ip4Address[ResolvedDNSEntry.nIPv4Entries].v[1] =
                                    _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                            ResolvedDNSEntry.ip4Address[ResolvedDNSEntry.nIPv4Entries].v[2] =
                                    _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                            ResolvedDNSEntry.ip4Address[ResolvedDNSEntry.nIPv4Entries].v[3] =
                                    _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                            if((DNSAnswerHeader.ResponseTTL.Val < ResolvedDNSEntry.ipTTL.Val)
                                    ||(ResolvedDNSEntry.ipTTL.Val == 0))
                            {
                                ResolvedDNSEntry.ipTTL.Val = DNSAnswerHeader.ResponseTTL.Val;
                            }
                            ResolvedDNSEntry.nIPv4Entries++;
                        }
                        else
                        {
                            break;
                        }
                    }
                    else if (DNSAnswerHeader.ResponseType.Val == 0x001Cu &&
                                    DNSAnswerHeader.ResponseLen.Val == 0x0010u)
                    {
                        if (pDnsDcpt->recordType != DNS_TYPE_AAAA)
                        {
                            while(DNSAnswerHeader.ResponseLen.Val--)
                            {
                                i = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                            }
                            continue;
                        }
                        pDnsDcpt->flags.bits.AddressValid = true;
                        pDnsDcpt->flags.bits.AddressType = IP_ADDRESS_TYPE_IPV6;
#ifdef TCPIP_STACK_USE_IPV6
                        if(ResolvedDNSEntry.nIPv6Entries < pDnsDcpt->IPv6EntriesPerDNSName)
                        {
                            pDnsDcpt->udpGetBufferData.length =
                                    _DNSGetDataArrayFromUDPBuff(pDnsDcpt->udpGetBufferData.length,&pDnsDcpt->udpGetBufferData,sizeof (IPV6_ADDR),
                                            (uint8_t*)ResolvedDNSEntry.ip6Address[ResolvedDNSEntry.nIPv6Entries].v);

                            if((DNSAnswerHeader.ResponseTTL.Val < ResolvedDNSEntry.ipTTL.Val)
                                    ||(ResolvedDNSEntry.ipTTL.Val == 0))
                            {
                                ResolvedDNSEntry.ipTTL.Val = DNSAnswerHeader.ResponseTTL.Val;
                            }
                            ResolvedDNSEntry.nIPv6Entries++;
                         }
                        else
                        {
                           break;
                        }
#else
                        ipv6Size = sizeof(IPV6_ADDR);
                        while(ipv6Size--)
                        {
                            i = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                        }
#endif
                    }
                    else
                    {
                        while(DNSAnswerHeader.ResponseLen.Val--)
                        {
                            i = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                        }
                    }
                }
                else
                {
                    while(DNSAnswerHeader.ResponseLen.Val--)
                    {
                        i = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                    }
                }

            }

            // Remove all Authoritative Records
            while(DNSHeader.AuthoritativeRecords.Val--)
            {
                // insert the number of Authorative entries if the NoofCachePerDnsAddress is not equal to zero
                if(!NoofCachePerDnsAddress)
                {
                    break;
                }
                _DNSDiscardName(pDnsDcpt);					// Throw away response name
                // Response Type
                DNSAnswerHeader.ResponseType.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData); // Response type
                DNSAnswerHeader.ResponseType.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                // Response Class
                DNSAnswerHeader.ResponseClass.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData); // Response class
                DNSAnswerHeader.ResponseClass.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                //Time to live
                DNSAnswerHeader.ResponseTTL.v[3] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                DNSAnswerHeader.ResponseTTL.v[2] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                DNSAnswerHeader.ResponseTTL.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                DNSAnswerHeader.ResponseTTL.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                // Response length
                DNSAnswerHeader.ResponseLen.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                DNSAnswerHeader.ResponseLen.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);

                // Make sure that this is a 4 byte IP address, response type A or MX, class 1
                // Check if this is Type A
                if( DNSAnswerHeader.ResponseClass.Val	== 0x0001u) // Internet class
                {
                    if (DNSAnswerHeader.ResponseType.Val	== 0x0001u &&
                            DNSAnswerHeader.ResponseLen.Val 	== 0x0004u)
                    {
                        pDnsDcpt->flags.bits.AddressValid = true;
                        pDnsDcpt->flags.bits.AddressType = IP_ADDRESS_TYPE_IPV4;
                        if(ResolvedDNSEntry.nIPv4Entries < pDnsDcpt->IPv4EntriesPerDNSName)
                        {
                            ResolvedDNSEntry.ip4Address[ResolvedDNSEntry.nIPv4Entries].v[0] =
                                    _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                            ResolvedDNSEntry.ip4Address[ResolvedDNSEntry.nIPv4Entries].v[1] =
                                    _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                            ResolvedDNSEntry.ip4Address[ResolvedDNSEntry.nIPv4Entries].v[2] =
                                    _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                            ResolvedDNSEntry.ip4Address[ResolvedDNSEntry.nIPv4Entries].v[3] =
                                    _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                            if((DNSAnswerHeader.ResponseTTL.Val < ResolvedDNSEntry.ipTTL.Val)
                                    ||(ResolvedDNSEntry.ipTTL.Val == 0))
                            {
                                ResolvedDNSEntry.ipTTL.Val = DNSAnswerHeader.ResponseTTL.Val;
                            }
                            ResolvedDNSEntry.nIPv4Entries++;
                        }
                        else
                        {
                            break;
                        }

                    }
                    else if (DNSAnswerHeader.ResponseType.Val == 0x001Cu &&
                                            DNSAnswerHeader.ResponseLen.Val == 0x0010u)
                    {
                        if (pDnsDcpt->recordType != DNS_TYPE_AAAA)
                        {
                            while(DNSAnswerHeader.ResponseLen.Val--)
                            {
                                i = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                            }
                            continue;
                        }
                        pDnsDcpt->flags.bits.AddressValid = true;
                        pDnsDcpt->flags.bits.AddressType = IP_ADDRESS_TYPE_IPV6;
#ifdef TCPIP_STACK_USE_IPV6
                        if(ResolvedDNSEntry.nIPv6Entries < pDnsDcpt->IPv6EntriesPerDNSName)
                        {
                            pDnsDcpt->udpGetBufferData.length =
                                    _DNSGetDataArrayFromUDPBuff(pDnsDcpt->udpGetBufferData.length,&pDnsDcpt->udpGetBufferData,sizeof (IPV6_ADDR),
                                            (uint8_t*)ResolvedDNSEntry.ip6Address[ResolvedDNSEntry.nIPv6Entries].v);
                            if((DNSAnswerHeader.ResponseTTL.Val < ResolvedDNSEntry.ipTTL.Val)
                                    ||(ResolvedDNSEntry.ipTTL.Val == 0))
                            {
                                ResolvedDNSEntry.ipTTL.Val = DNSAnswerHeader.ResponseTTL.Val;
                            }
                            ResolvedDNSEntry.nIPv6Entries++;
                        }
                        else
                        {
                            break;
                        }
#else
                        ipv6Size = sizeof(IPV6_ADDR);
                        while(ipv6Size--)
                        {
                            i = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                        }
#endif
                    }
                    else
                    {
                        while(DNSAnswerHeader.ResponseLen.Val--)
                        {
                            i = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                        }
                    }
                }
                else
                {
                    while(DNSAnswerHeader.ResponseLen.Val--)
                    {
                        i = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                    }
                }
            }

            // Remove all Additional Records
            while(DNSHeader.AdditionalRecords.Val--)
            {
                // insert the number of Authorative entries if the NoofCachePerDnsAddress is not equal to zero
                if(!NoofCachePerDnsAddress)
                {
                    break;
                }
                _DNSDiscardName(pDnsDcpt);					// Throw away response name
                // Response Type
                DNSAnswerHeader.ResponseType.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                DNSAnswerHeader.ResponseType.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                // Response Class
                DNSAnswerHeader.ResponseClass.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                DNSAnswerHeader.ResponseClass.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                //Time to live
                DNSAnswerHeader.ResponseTTL.v[3] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                DNSAnswerHeader.ResponseTTL.v[2] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                DNSAnswerHeader.ResponseTTL.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                DNSAnswerHeader.ResponseTTL.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                // Response length
                DNSAnswerHeader.ResponseLen.v[1] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                DNSAnswerHeader.ResponseLen.v[0] = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);

                // Make sure that this is a 4 byte IP address, response type A or MX, class 1
                // Check if this is Type A
                if( DNSAnswerHeader.ResponseClass.Val	== 0x0001u) // Internet class
                {
                    if (DNSAnswerHeader.ResponseType.Val	== 0x0001u &&
                            DNSAnswerHeader.ResponseLen.Val 	== 0x0004u)
                    {
                        pDnsDcpt->flags.bits.AddressValid = true;
                        pDnsDcpt->flags.bits.AddressType = IP_ADDRESS_TYPE_IPV4;
                        if(ResolvedDNSEntry.nIPv4Entries < pDnsDcpt->IPv4EntriesPerDNSName)
                        {
                            ResolvedDNSEntry.ip4Address[ResolvedDNSEntry.nIPv4Entries].v[0] =
                                    _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                            ResolvedDNSEntry.ip4Address[ResolvedDNSEntry.nIPv4Entries].v[1] =
                                    _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                            ResolvedDNSEntry.ip4Address[ResolvedDNSEntry.nIPv4Entries].v[2] =
                                    _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                            ResolvedDNSEntry.ip4Address[ResolvedDNSEntry.nIPv4Entries].v[3] =
                                    _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                            if((DNSAnswerHeader.ResponseTTL.Val < ResolvedDNSEntry.ipTTL.Val)
                                    ||(ResolvedDNSEntry.ipTTL.Val == 0))
                            {
                                 ResolvedDNSEntry.ipTTL.Val = DNSAnswerHeader.ResponseTTL.Val;
                            }
                            ResolvedDNSEntry.nIPv4Entries++;
                        }
                        else
                        {
                           break;
                        }
                    }
                    else if (DNSAnswerHeader.ResponseType.Val == 0x001Cu &&
                                            DNSAnswerHeader.ResponseLen.Val == 0x0010u)
                    {
                        if (pDnsDcpt->recordType != DNS_TYPE_AAAA)
                        {
                            while(DNSAnswerHeader.ResponseLen.Val--)
                            {
                                i = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                            }
                            continue;
                        }
                        pDnsDcpt->flags.bits.AddressValid = true;
                        pDnsDcpt->flags.bits.AddressType = IP_ADDRESS_TYPE_IPV6;
#ifdef TCPIP_STACK_USE_IPV6
                        if(ResolvedDNSEntry.nIPv6Entries < pDnsDcpt->IPv6EntriesPerDNSName)
                        {
                            pDnsDcpt->udpGetBufferData.length =
                                _DNSGetDataArrayFromUDPBuff(pDnsDcpt->udpGetBufferData.length,&pDnsDcpt->udpGetBufferData,sizeof(IPV6_ADDR),
                                        (uint8_t*)&ResolvedDNSEntry.ip6Address[ResolvedDNSEntry.nIPv6Entries].v);

                            if((DNSAnswerHeader.ResponseTTL.Val < ResolvedDNSEntry.ipTTL.Val)
                                ||(ResolvedDNSEntry.ipTTL.Val == 0))
                            {
                                ResolvedDNSEntry.ipTTL.Val = DNSAnswerHeader.ResponseTTL.Val;
                            }
                            ResolvedDNSEntry.nIPv6Entries++;
                        }
                        else
                        {
                            break;
                        }
#else
                        ipv6Size = sizeof(IPV6_ADDR);
                        while(ipv6Size--)
                        {
                            i = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                        }
#endif
                    }
                    else
                    {
                        while(DNSAnswerHeader.ResponseLen.Val--)
                        {
                            i = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                        }
                    }
                }
                else
                {
                    while(DNSAnswerHeader.ResponseLen.Val--)
                    {
                        i = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
                    }
                }
            }

            if(pDnsDcpt->flags.bits.AddressValid)
            {
                _DNSSetHashEntry(dnshE,DNS_FLAG_ENTRY_COMPLETE,ResolvedDNSEntry);
                _DNSNotifyClients(0,TCPIP_DNS_EVENT_NAME_RESOLVED,dnshE->pHostName);
            }
           
            if(pDnsDcpt->udpGetBufferData.head)
            {
                TCPIP_HEAP_Free(pDnsDcpt->memH,pDnsDcpt->udpGetBufferData.head);
                pDnsDcpt->udpGetBufferData.head = 0;
            }
            TCPIP_UDP_Discard(pDnsDcpt->dnsSocket);
            pDnsDcpt->smState = DNS_QUERY;
            break;	// done
			
        case DNS_FAIL_ARP:
            // see if there is other server we may try
            pDnsDcpt->smState = _DNSRetry(DNS_FAIL_ARP);
            break;

        case DNS_FAIL_SERVER:
            pDnsDcpt->smState = _DNSRetry(DNS_FAIL_SERVER);
            break;
        case DNS_FAIL_ARP_TMO:			// ARP resolution TMO
        case DNS_FAIL_OPEN_TMO:			// Open Socket TMO
        case DNS_FAIL_SERVER_TMO:		// DNS server TMO
            if(pNewIf->PrimaryDNSServer.Val !=0 || pNewIf->SecondaryDNSServer.Val!=0)
            {
                pDnsDcpt->DNSServers[0].v4Add.Val = pNewIf->PrimaryDNSServer.Val;
                pDnsDcpt->DNSServers[1].v4Add.Val = pNewIf->SecondaryDNSServer.Val;
            }
            // Release socket
            _DNSReleaseSocket(pDnsDcpt);
            // if all the server index is tried, then try with socket open with 0th index.
            if(pDnsDcpt->vDNSServerIx == sizeof(pDnsDcpt->DNSServers)/sizeof(IP_MULTI_ADDRESS))
            {
                pDnsDcpt->smState = DNS_START;
            }
            break;
        default:	// DNS_DONE
            // either done or some error state
            break;
    }
    return true;

}

// see if we can perform a retry
static DNS_STATE _DNSRetry(DNS_STATE currState)
{
    DNS_DCPT    *pDnsDcpt=&gDnsDcpt;
    
    // Release socket
    _DNSReleaseSocket(pDnsDcpt);
    
    if(currState == DNS_START)
    {
        pDnsDcpt->vDNSServerIx = 0;
    }
    else
    {
        pDnsDcpt->vDNSServerIx++;
    }
    
    if(pDnsDcpt->dnsIpAddressType == IP_ADDRESS_TYPE_IPV4)
    {
        for( ; pDnsDcpt->vDNSServerIx < sizeof(pDnsDcpt->DNSServers)/sizeof(IP_MULTI_ADDRESS); pDnsDcpt->vDNSServerIx++)
        {   // can try another server if valid address
            if(pDnsDcpt->DNSServers[pDnsDcpt->vDNSServerIx].v4Add.Val != 0)
            {
                return DNS_OPEN_SOCKET;   // new state
            }
        }
    }

    // nothing else to try
    if(currState == DNS_FAIL_ARP)
    {
        return DNS_FAIL_ARP_TMO;
    }

    // default: DNS_FAIL_SERVER
    return DNS_FAIL_SERVER_TMO;
}


/*****************************************************************************
  Function:
	static void _DNSPutString(DNSCLINTBUFFERDATA *putbuf, const char* String)

  Summary:
	Writes a string to the DNS dynamic allocated buffer which will be used while pouplating UDP 
	Buffer.
	
  Description:
	This function writes a string to the DNS dynamic allocated buffer, ensuring that it is
	properly formatted.

  Precondition:
	DNS dynamic allocated buffer .

  Parameters:
	String - the string to write to the DNS dynamic allocated buffer.

  Returns:
  	None
  ***************************************************************************/
static void _DNSPutString(DNSCLINTBUFFERDATA *putbuf, const char* String)
{
    const char *RightPtr;
    uint8_t i;
    uint8_t Len;

    RightPtr = String;

    while(1)
    {
        do
        {
            i = *RightPtr++;
        } while((i != 0x00u) && (i != '.') && (i != '/') && (i != ',') && (i != '>'));

        // Put the length and data
        // Also, skip over the '.' in the input string
        Len = (uint8_t)(RightPtr-String-1);
        _DNSCopyDataToProcessBuff(Len,putbuf);
        _DNSCopyDataArrayToProcessBuff((uint8_t*)String,putbuf,Len);
        String += Len + 1;

        if(i == 0x00u || i == '/' || i == ',' || i == '>')
                break;
    }

	// Put the string null terminator character (zero length label)	
    _DNSCopyDataToProcessBuff(0x00,putbuf);
}


/*****************************************************************************
  Function:
	static void DNSDiscardName(DNS_DCPT *pDnsDcpt)

  Summary:
	Reads a name string or string pointer from the DNS socket and discards it.
	
  Description:
	This function reads a name string from the DNS socket.  Each string 
	consists of a series of labels.  Each label consists of a length prefix 
	byte, followed by the label bytes.  At the end of the string, a zero length 
	label is found as termination.  If name compression is used, this function 
	will automatically detect the pointer and discard it.

  Precondition:
	UDP socket is obtained and ready for reading a DNS name

  Parameters:
	None

  Returns:
  	None
  ***************************************************************************/
static void _DNSDiscardName(DNS_DCPT *pDnsDcpt)
{
    uint8_t i;
    uint32_t len=0;

    while(1)
    {
        // Get first byte which will tell us if this is a 16-bit pointer or the
        // length of the first of a series of labels
        len = _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
        if(!len)
            return;
		
        // Check if this is a pointer, if so, get the remaining 8 bits and return
        if((len & 0xC0u) == 0xC0u)
        {
            _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
            return;
        }

        // Exit once we reach a zero length label
        if(len == 0u)
            return;

        for(i=0;i<len;i++)
        {
            _DNSGetDataFromUDPBuff(pDnsDcpt->udpGetBufferData.length++,&pDnsDcpt->udpGetBufferData);
        }
    }
}


    
size_t TCPIP_DNS_OAHASH_KeyHash(OA_HASH_DCPT* pOH, void* key)
{
    uint8_t    *dnsHostNameKey;
    size_t      hostnameLen=0;

    dnsHostNameKey = (uint8_t *)key;
    hostnameLen = strlen((const char*)dnsHostNameKey);
    return fnv_32_hash(dnsHostNameKey, hostnameLen) % (pOH->hEntries);
}


OA_HASH_ENTRY* TCPIP_DNS_OAHASH_DeleteEntry(OA_HASH_DCPT* pOH)
{
    OA_HASH_ENTRY*  pBkt;
    size_t      bktIx;
    DNS_HASH_ENTRY  *pE;
    DNS_DCPT        *pDnsDcpt;

    pDnsDcpt = &gDnsDcpt;
    for(bktIx = 0; bktIx < pOH->hEntries; bktIx++)
    {
        pBkt = TCPIP_OAHASH_EntryGet(pOH, bktIx);		
        if(pBkt->flags.busy != 0)
        {
            pE = (DNS_HASH_ENTRY*)pBkt;
            if(pDnsDcpt->cacheEntryTMO > 0)
            {
                if((pDnsDcpt->dnsTimeSeconds-pE->tInsert) > pDnsDcpt->cacheEntryTMO)
                {
                    _DNSNotifyClients(0,TCPIP_DNS_EVENT_CACHE_EXPIRED,pE->pHostName);
                    return pBkt;
                }
            }
            else if((pDnsDcpt->dnsTimeSeconds-pE->tInsert) > pE->ipTTL.Val)
            {
                _DNSNotifyClients(0,TCPIP_DNS_EVENT_CACHE_EXPIRED,pE->pHostName);
                return pBkt;
            }
        }
    }

    return 0;
}


int TCPIP_DNS_OAHASH_KeyCompare(OA_HASH_DCPT* pOH, OA_HASH_ENTRY* hEntry, void* key)
{
    DNS_HASH_ENTRY  *pDnsHE;
    uint8_t         *dnsHostNameKey;
    size_t          hostnameLen=0;

  
    pDnsHE =(DNS_HASH_ENTRY  *)hEntry;
    dnsHostNameKey = (uint8_t *)key;    
    hostnameLen = strlen((const char*)dnsHostNameKey);
    
    return strcmp((const char*)pDnsHE->pHostName,(const char*)dnsHostNameKey);
}

void TCPIP_DNS_OAHASH_KeyCopy(OA_HASH_DCPT* pOH, OA_HASH_ENTRY* dstEntry, void* key)
{
    uint8_t    *dnsHostNameKey;
    DNS_HASH_ENTRY  *pDnsHE;
    size_t          hostnameLen=0;

    if(key==NULL) return;
    
    pDnsHE =(DNS_HASH_ENTRY  *)dstEntry;
    dnsHostNameKey = (uint8_t *)key;
    hostnameLen = strlen((const char*)dnsHostNameKey);

    if(dnsHostNameKey)
        pDnsHE->pHostName = dnsHostNameKey;
   // memcpy(pDnsHE->hostNameData.hostname,dnsHostNameKey->hostname,pDnsHE->hostNameData.size);
}

#if defined(OA_DOUBLE_HASH_PROBING)
size_t TCPIP_DNS_OAHASH_ProbeHash(OA_HASH_DCPT* pOH, void* key)
{
    uint8_t    *dnsHostNameKey;
    size_t      hostnameLen=0;
    
    dnsHostNameKey = (uint8_t  *)key;
    hostnameLen = strlen(dnsHostNameKey);
    return fnv_32a_hash(dnsHostNameKey, hostnameLen) % (pOH->hEntries);
}
#endif  // defined(OA_DOUBLE_HASH_PROBING)

// Register an DNS event handler
// Use hNet == 0 to register on all interfaces available
// Returns a valid handle if the call succeeds,
// or a null handle if the call failed.
// Function has to be called after the DNS is initialized
// The hParam is passed by the client and will be used by the DNS when the notification is made.
// It is used for per-thread content or if more modules, for example, share the same handler
// and need a way to differentiate the callback.
TCPIP_DNS_HANDLE TCPIP_DNS_HandlerRegister(TCPIP_NET_HANDLE hNet, TCPIP_DNS_EVENT_HANDLER handler, const void* hParam)
{
    
    if(gDnsDcpt.memH)
    {
        DNS_LIST_NODE* newNode = (DNS_LIST_NODE*)TCPIP_Notification_Add(&dnsRegisteredUsers, gDnsDcpt.memH, sizeof(*newNode));
        if(newNode)
        {
            newNode->handler = handler;
            newNode->hParam = hParam;
            newNode->hNet = hNet;
            return newNode;
        }
    }

    return 0;
}

// deregister the event handler
bool TCPIP_DNS_HandlerDeRegister(TCPIP_DNS_HANDLE hDns)
{
    if(hDns && gDnsDcpt.memH)
    {
        if(TCPIP_Notification_Remove((SGL_LIST_NODE*)hDns, &dnsRegisteredUsers, gDnsDcpt.memH))
        {
            return true;
        }
    }

    return false;
}

static void _DNSNotifyClients(TCPIP_NET_IF* pNetIf, TCPIP_DNS_EVENT_TYPE evType,void *param)
{
    DNS_LIST_NODE* dNode;

    for(dNode = (DNS_LIST_NODE*)dnsRegisteredUsers.list.head; dNode != 0; dNode = dNode->next)
    {
        if(dNode->hNet == 0 || dNode->hNet == pNetIf)
        {   // trigger event
            if(param != 0)
            {
                if(strcmp((char*)param,dNode->hParam)==0)
                {
                    (*dNode->handler)(pNetIf, evType, dNode->hParam);
                }
            }
            else
            {
                (*dNode->handler)(pNetIf, evType, dNode->hParam);
            }
        }
    }    
}

bool TCPIP_DNS_IsEnabled(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(hNet);
    if(pNetIf)
    {
        return pNetIf->Flags.bIsDnsClientEnabled!= 0;
    }
    return false;
}

bool TCPIP_DNS_Enable(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(hNet);
    DNS_DCPT*			pDnsDcpt;
    pDnsDcpt = &gDnsDcpt;
    if(pNetIf == 0)
    {
        return false;
    }
    
    if(TCPIP_STACK_DNSServiceCanStart(pNetIf, TCPIP_STACK_SERVICE_DNSC))
    {
        pDnsDcpt->smState = DNS_START;
        pNetIf->Flags.bIsDnsClientEnabled = true;
        pDnsDcpt->dnsSocket = INVALID_UDP_SOCKET;
    }
    return true;
}

bool TCPIP_DNS_Disable(TCPIP_NET_HANDLE hNet)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(hNet);
    DNS_DCPT*			pDnsDcpt;
    pDnsDcpt = &gDnsDcpt;
    if(pNetIf == 0)
    {
        return false;
    }
    pDnsDcpt->smState = DNS_IDLE;
    pNetIf->Flags.bIsDnsClientEnabled = false;
    if(pDnsDcpt->dnsSocket != INVALID_UDP_SOCKET)
    {
        TCPIP_UDP_Close(pDnsDcpt->dnsSocket);
    }
    
    return true;
	
}

static void DNS_DhcpNotify(TCPIP_NET_HANDLE hNet, uint8_t evType, const void * param)
{
    TCPIP_NET_IF* pNetIf = _TCPIPStackHandleToNetUp(hNet);
    if(gDnsDcpt.memH)
    {
        if(evType == DHCP_EVENT_BOUND )
        {
            gDnsDcpt.DNSServers[0].v4Add.Val = pNetIf->PrimaryDNSServer.Val;
            gDnsDcpt.DNSServers[1].v4Add.Val = pNetIf->SecondaryDNSServer.Val;
        }
        else
        {
            gDnsDcpt.DNSServers[0].v4Add.Val = pNetIf->DefaultDNSServer.Val;
            gDnsDcpt.DNSServers[1].v4Add.Val = pNetIf->DefaultDNSServer2.Val;
        }
       // _DNSReleaseSocket(&gDnsDcpt);
        gDnsDcpt.smState = DNS_START;
        _DNSRetry(gDnsDcpt.smState);

    }
}
#else
bool TCPIP_DNS_IsEnabled(TCPIP_NET_HANDLE hNet){return false;}
bool TCPIP_DNS_Enable(TCPIP_NET_HANDLE hNet){return false;}
bool TCPIP_DNS_Disable(TCPIP_NET_HANDLE hNet){return false;}

TCPIP_DNS_RESULT  TCPIP_DNS_Resolve(const char* HostName, DNS_RESOLVE_TYPE Type)
{
    return DNS_RES_NO_SERVICE; 
}

TCPIP_DNS_RESULT  TCPIP_DNS_IsResolved(const char* HostName, void* HostIP)
{
    return DNS_RES_NO_SERVICE; 
}




#endif	//#if defined(TCPIP_STACK_USE_DNS)
