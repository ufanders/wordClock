
/*******************************************************************************
  Address Resolution Protocol (ARP) Header file

  Company:
    Microchip Technology Inc.

  File Name:
   arp.h

  Summary:
    Address Resolution Protocol (ARP) Header file

  Description:
    This source file contains the ARP module API
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#ifndef __ARP_H_
#define __ARP_H_

#include <stdbool.h>

// *****************************************************************************
// *****************************************************************************
// Section: Data Types and Constants
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************

/*
  Enumeration:
    TCPIP_ARP_RESULT

  Summary:
    ARP Results (success and failure codes)

  Description:
    Various Definitions for Success and Failure Codes

  Remarks:
    None
*/
typedef enum
{
    //DOM-IGNORE-BEGIN
    // Assignments irrelevant to user have been removed.
    //DOM-IGNORE-END

    // success codes
    ARP_RES_OK                  = 0,    // operation succeeded
    ARP_RES_ENTRY_NEW,                  // operation succeeded and a new entry was added
    ARP_RES_ENTRY_SOLVED,               // the required entry is already solved
    ARP_RES_ENTRY_QUEUED,               // the required entry was already queued
    ARP_RES_ENTRY_EXIST,                // the required entry was already cached
    ARP_RES_PERM_QUOTA_EXCEED,          // info: the quota of permanent entries was exceeded
    ARP_RES_PROBE_OK,                   // requested probe sent


    // failure codes
    ARP_RES_NO_ENTRY            = -1,   // no such entry exists
    ARP_RES_CACHE_FULL          = -2,   // the cache is full and no entry could be
                                        // removed to make room
    ARP_RES_TX_FAILED           = -3,   // failed to transmit an ARP message
    ARP_RES_BAD_INDEX           = -4,   // bad query index    
    ARP_RES_BAD_ADDRESS         = -5,   // bad IP address specified 
    ARP_RES_NO_INTERFACE        = -6,   // no such interface exists   
    ARP_RES_BAD_TYPE            = -7,   // no such type is valid/exists   
    ARP_RES_CONFIGURE_ERR       = -8,   // interface is configuring now, no ARP probes
    ARP_RES_PROBE_FAILED        = -9,   // requested probe failed
}TCPIP_ARP_RESULT;


// *****************************************************************************
/*
  Enumeration:
    TCPIP_ARP_ENTRY_TYPE

  Summary:
    Type of ARP entry

  Description:
    List of different ARP cache entries

  Remarks:
    None
*/
typedef enum
{
    ARP_ENTRY_TYPE_INVALID,             // empty entry
    ARP_ENTRY_TYPE_PERMANENT,           // entry valid and permanent
    ARP_ENTRY_TYPE_COMPLETE,            // entry valid
    ARP_ENTRY_TYPE_INCOMPLETE,          // entry not resolved yet
    ARP_ENTRY_TYPE_ANY,                 // any busy entry (PERMANENT|COMPLETE|INCOMPLETE) 
    ARP_ENTRY_TYPE_TOTAL,               // total entries - the number of entries the cache can store 
}TCPIP_ARP_ENTRY_TYPE;

// *****************************************************************************
/*
  Structure:
    TCPIP_ARP_ENTRY_QUERY

  Summary:
    ARP Entry Query

  Description:
    Data structure for an ARP query.
*/
typedef struct
{
    TCPIP_ARP_ENTRY_TYPE  entryType;      // what entry type
    IPV4_ADDR         entryIpAdd;         // the entry IP address
    TCPIP_MAC_ADDR        entryHwAdd;     // the entry hardware address
}TCPIP_ARP_ENTRY_QUERY;


// *****************************************************************************
/*
  Enumeration:
    TCPIP_ARP_EVENT_TYPE

  Summary:
    Events reported by ARP

  Description:
    List of events reported by ARP.

  Remarks:
    Possibly multiple events can be set, where it makes sense.
*/
typedef enum
{
    ARP_EVENT_SOLVED        = 0x01,     // a queued cache entry was solved;                        
    ARP_EVENT_UPDATED       = 0x02,     // an existent cache entry was updated
    ARP_EVENT_PERM_UPDATE   = 0x04,     // an update for an permanent entry was received
                                        // however the permanent entry was not updated    
    
    // error events 
    ARP_EVENT_TMO           = -1 ,     // an entry could not be solved and a tmo occurred                                        
}TCPIP_ARP_EVENT_TYPE;


typedef struct
{
    // specific ARP params
    size_t  cacheEntries;   // cache entries for this interface
    bool    deleteOld;      // delete old cache if still in place,
                            // else don't re-initialize it
    int     entrySolvedTmo; // solved entry removed after this tmo
                            // if not referenced - seconds
    int     entryPendingTmo;// timeout for a pending to be solved entry in the cache, in seconds
    int     entryRetryTmo;  // timeout for resending an ARP request for a pending entry - seconds
                            // 1 sec < tmo < entryPendingTmo
    int     permQuota;      // max percentage of permanent entries allowed in the cache - %
    int     purgeThres;     // purge threshold - %
    int     purgeQuanta;    // no of entries to delete once the threshold is reached
    int     retries;        // no of retries for resolving an entry
    int     gratProbeCount; // no of retries done for a gratuitous ARP request
}ARP_MODULE_CONFIG;





// *****************************************************************************
/*
  Type:
    TCPIP_ARP_EVENT_HANDLER

  Summary:
    Notification handler that can be called when a specific entry is resolved.

  Description:
    The format of a notification handler registered with the ARP module.

  Remarks:
    The param member significance is module dependent.
    It can be an IP address, pointer to some other structure, etc.
    The handler is called when an event of some sort occurs for a particular IP address entry.
    If pNetIf == 0 then the notification is called for events on any interface
*/
typedef void    (*TCPIP_ARP_EVENT_HANDLER)(TCPIP_NET_HANDLE hNet, const IPV4_ADDR* ipAdd, const TCPIP_MAC_ADDR* MACAddr, TCPIP_ARP_EVENT_TYPE evType, const void* param);


// *****************************************************************************
/*
  Type:
    TCPIP_ARP_HANDLE

  Summary:
    ARP Handle

  Description:
    A handle that a client needs to use when  deregistering a notification handler. 

  Remarks:
    This handle can be used by the client after the event handler has been registered.
*/
typedef const void* TCPIP_ARP_HANDLE;

/*
  Enumeration:
    TCPIP_ARP_OPERATION_TYPE

  Summary:
    Type of ARP operation

  Description:
    Operation to be performed by an ARP probe

  Remarks:
    Used for low level functionality, TCPIP_ARP_Probe
*/
typedef enum
{
    // supported ARP operations
    ARP_OPERATION_REQ       = 1,        // ARP request
    ARP_OPERATION_RESP      = 2,        // ARP response
    //
    ARP_OPERATION_MASK      = 0x000f,   // extract ARP operation

    // ARP flags
    ARP_OPERATION_CONFIGURE     = 0x1000,   // stack configuration ARP packet
    ARP_OPERATION_GRATUITOUS    = 0x2000,   // stack gratuitous ARP packet
    ARP_OPERATION_PROBE_ONLY    = 0x4000,   // an ARP probe is sent only once, the target address is not stored 
    //
}TCPIP_ARP_OPERATION_TYPE;



// *****************************************************************************
// *****************************************************************************
// Section: ARP Routines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/*
Function:
    TCPIP_ARP_HANDLE TCPIP_ARP_HandlerRegister(TCPIP_NET_HANDLE hNet, TCPIP_ARP_EVENT_HANDLER handler, const void* hParam)

  Summary:
    Register an ARP resolve handler

  Description:
    This function will register a notification handler with the ARP module.

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet        -   Specifies interface to register on. 
                    Use hNet == 0 to register on all interfaces available.
    handler     -   Handler to be called for event.
    hParam      -   The hParam is passed by the client and will be used by the 
                    ARP when the notification is made. It is used for per-thread 
                    content or if more modules, for example, share the same handler
                    and need a way to differentiate the callback.

  Returns:
    TCPIP_ARP_HANDLE 
        On Success - Returns a valid handle
        On Failure - null handle

  Example:
     None

  Remarks:
    NONE
*/
TCPIP_ARP_HANDLE      TCPIP_ARP_HandlerRegister(TCPIP_NET_HANDLE hNet, TCPIP_ARP_EVENT_HANDLER handler, const void* hParam);

// *****************************************************************************
/*
Function:
    bool TCPIP_ARP_HandlerDeRegister(TCPIP_ARP_HANDLE hArp)

  Summary:
    Deregisters the event handler

  Description:
    Deregisters a previously registered ARP handler

  Precondition:
    ARP module should have been initialized

  Parameters:
    hArp    -   ARP Handle

  Returns:
    bool
        On Success - true
        On Failure - false (If no such handler registered)

  Example:
    None

  Remarks:
    None
*/
bool            TCPIP_ARP_HandlerDeRegister(TCPIP_ARP_HANDLE hArp);


// *****************************************************************************
/*
  Function:
    TCPIP_ARP_RESULT TCPIP_ARP_Resolve(TCPIP_NET_HANDLE hNet, IPV4_ADDR* IPAddr)

  Summary:
    Transmits an ARP request to resolve an IP address.

  Description:
    This function transmits and ARP request to determine the hardware
    address of a given IP address.
    Upon the address resolution it calls the registered handler
    (if available) with the supplied notification parameter (if != 0)

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet   - interface to use
    IPAddr - The IP address to be resolved.  The address must be specified
             in network byte order (big endian).

  Returns:
    An element from the TCPIP_ARP_RESULT enumeration.
    ARP_RES_ENTRY_SOLVED     - if the required entry is already solved
    ARP_RES_ENTRY_QUEUED     - if the required entry was already queued
    ARP_RES_ENTRY_NEW        - if the operation succeeded and a new entry
                               was added (and queued for resolving)
    ARP_RES_CACHE_FULL       - if new entry could not be inserted,
                               the cache was full
    ARP_RES_BAD_ADDRESS      - bad address specified
    ARP_RES_NO_INTERFACE     - no such interface

  Remarks:

    To retrieve the ARP query result, call the TCPIP_ARP_IsResolved() function.
  ***************************************************************************/
TCPIP_ARP_RESULT      TCPIP_ARP_Resolve(TCPIP_NET_HANDLE hNet, IPV4_ADDR* IPAddr);


// *****************************************************************************
/*
  Function:
    bool TCPIP_ARP_IsResolved(TCPIP_NET_HANDLE hNet, IPV4_ADDR* IPAddr, TCPIP_MAC_ADDR* MACAddr)

  Summary:
    Determines if an ARP request has been resolved yet.

  Description:
    This function checks if an ARP request has been resolved yet, and if
    so, stores the resolved MAC address in the pointer provided.

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet   - interface to use
    IPAddr - The IP address to be resolved.  This must match the IP address
             provided to the TCPIP_ARP_Resolve() function call.
    MACAddr - A buffer to store the corresponding MAC address retrieved from
             the ARP query.

  Return Values:
    true - The IP address has been resolved and MACAddr MAC address field
           indicates the response.
    false - The IP address is not yet resolved.  Try calling TCPIP_ARP_IsResolved()
           again at a later time.  If you don't get a response after a
           application specific timeout period, you may want to call
           TCPIP_ARP_Resolve() again to transmit another ARP query (in case if the
           original query or response was lost on the network).  If you never
           receive an ARP response, this may indicate that the IP address
           isn't in use.

  Remarks:
  ***************************************************************************/
bool            TCPIP_ARP_IsResolved(TCPIP_NET_HANDLE hNet, IPV4_ADDR* IPAddr, TCPIP_MAC_ADDR* MACAddr);

/*
  Function:
    TCPIP_ARP_RESULT TCPIP_ARP_Probe(TCPIP_NET_HANDLE hNet, IPV4_ADDR* IPAddr, IPV4_ADDR* srcAddr, TCPIP_ARP_OPERATION_TYPE opType)

  Summary:
    Transmits an ARP probe to resolve an IP address.


  Description:
    This function transmits and ARP probe to determine the hardware
    address of a given IP address.
    The packet will use the type of operation and the source address 
    specified as parameters.

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet    - interface to use
    IPAddr  - The IP address to be resolved.  The address must be specified
              in network byte order (big endian).
    srcAddr - The source address to be used in the ARP packet
    opType  - Operation code to be set in the outgoing ARP packet    

  Returns:
    ARP_RES_ENTRY_SOLVED     - if the required entry is already solved
    ARP_RES_ENTRY_QUEUED     - if the required entry was already queued
    ARP_RES_ENTRY_NEW        - if the operation succeeded and a new entry
                               was added (and queued for resolving)
    ARP_RES_CACHE_FULL       - if new entry could not be inserted,
                               the cache was full
    ARP_RES_BAD_ADDRESS      - bad address specified
    ARP_RES_NO_INTERFACE     - no such interface

  Remarks:

    This function is a more advanced version of TCPIP_ARP_Resolve.
    It allows the caller to specify the operation type and the source address
    of the outgoing ARP packet.
    It also supports the ARP flags defined in TCPIP_ARP_OPERATION_TYPE.

    No check is done for IPAddr to be valid.

    To retrieve the ARP query result, call the TCPIP_ARP_IsResolved() function.
  ***************************************************************************/
TCPIP_ARP_RESULT TCPIP_ARP_Probe(TCPIP_NET_HANDLE hNet
                    , IPV4_ADDR* IPAddr
                    , IPV4_ADDR* srcAddr
                    , TCPIP_ARP_OPERATION_TYPE opType);

// *****************************************************************************
// *****************************************************************************
// Section: cache manipulation routines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/*
Function:
    TCPIP_ARP_RESULT TCPIP_ARP_EntrySet(TCPIP_NET_HANDLE hNet, IPV4_ADDR* ipAdd, TCPIP_MAC_ADDR* hwAdd, bool perm)

  Summary:
    Adds an ARP cache entry for the specified interface.

  Description:
    This function will add an entry to the selected interface cache. 
    The entry can be marked as as permanent (not subject to timeouts
    or updates from the network).
    If cache is full, an entry will be deleted to make room.

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet    -   Interface to use
    ipAdd   -   The IP address
    hwAdd   -   The mapping MAC address for the supplied ipAdd
    perm    -   if true, the entry will be marked as permanent

  Returns:
    On Success - ARP_RES_OK/ARP_RES_ENTRY_EXIST
    ON Failure - An Error 
        (for example, cache is full with permanent entries that cannot be 
        purged or the permanent quota exceeded)  

  Example:
    None

  Remarks:
    None
*/
TCPIP_ARP_RESULT      TCPIP_ARP_EntrySet(TCPIP_NET_HANDLE hNet, IPV4_ADDR* ipAdd, TCPIP_MAC_ADDR* hwAdd, bool perm);


// *****************************************************************************
/*
Function:
    TCPIP_ARP_RESULT TCPIP_ARP_EntryGet(TCPIP_NET_HANDLE hNet, IPV4_ADDR* ipAdd, TCPIP_MAC_ADDR* pHwAdd, bool probe)

  Summary:
    Gets the current mapping for an IP address

  Description:
    If probe == false
        The function behaves identical to TCPIP_ARP_IsResolved():
        If the corresponding MAC address exists in the cache
        it is copied to the user supplied pHwAdd


    If probe == true
        The function behaves identical to TCPIP_ARP_Resolve():
        - If the corresponding MAC address does not exist in the cache
        this function transmits and ARP request.
        Upon the address resolution it calls the registered handler
        (if available) with the supplied notification parameter (if != 0)
        - If the hardware address exists in the cache, the result is written to pHwAdd
        and no network ARP request is sent

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet    -   Interface to use
    ipAdd   -   The IP address to get entries from
    pHwAdd  -   pointer to store the hardware address
    probe   -   boolean to specify if ARP probing is initiated or not

  Returns:
    ARP_RES_ENTRY_SOLVED     - if the required entry is already solved
    ARP_RES_ENTRY_QUEUED     - if the required entry was already queued
    ARP_RES_ENTRY_NEW        - if the operation succeeded and a new entry
                               was added (and queued for resolving)
    ARP_RES_CACHE_FULL       - if new entry could not be inserted,
                               the cache was full
    ARP_RES_BAD_ADDRESS      - bad address specified
    ARP_RES_NO_INTERFACE     - no such interface

  Example:
    None

  Remarks:
    similar to TCPIP_ARP_Resolve() + TCPIP_ARP_IsResolved()
    It avoids a double hash search when the mapping exists.
*/
TCPIP_ARP_RESULT      TCPIP_ARP_EntryGet(TCPIP_NET_HANDLE hNet, IPV4_ADDR* ipAdd, TCPIP_MAC_ADDR* pHwAdd, bool probe);


// *****************************************************************************
/*
Function:
    TCPIP_ARP_RESULT TCPIP_ARP_EntryRemove(TCPIP_NET_HANDLE hNet, IPV4_ADDR* ipAdd);

  Summary:
    Removes the mapping of an address, even a permanent one

  Description:
    This function removes an existent mapping from the selected interface cache.

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet    -   Interface to use
    ipAdd   -   IP Address to remove entries for

  Returns:
    TCPIP_ARP_RESULT
        On Success - ARP_RES_OK
        On Failure - ARP_RES_NO_ENTRY (if no such mapping exists)
                     ARP_RES_NO_INTERFACE (if no such interface exists)

  Example:
    None

  Remarks:
    None
*/
TCPIP_ARP_RESULT      TCPIP_ARP_EntryRemove(TCPIP_NET_HANDLE hNet,  IPV4_ADDR* ipAdd);


// *****************************************************************************
/*
Function:
    TCPIP_ARP_RESULT TCPIP_ARP_EntryRemoveAll(TCPIP_NET_HANDLE hNet)

  Summary:
    Removes all the mapping belonging to an interface.

  Description:
    This function removes all existent mappings from the selected interface cache.

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet    -   network interface handle

  Returns:
    TCPIP_ARP_RESULT
        On Success - ARP_RES_OK
        On Failure - ARP_RES_NO_INTERFACE (if no such interface exists)


  Example:
    None

  Remarks:
    None
*/
TCPIP_ARP_RESULT      TCPIP_ARP_EntryRemoveAll(TCPIP_NET_HANDLE hNet);


// *****************************************************************************
/*
Function:
    TCPIP_ARP_RESULT TCPIP_ARP_EntryRemoveNet(TCPIP_NET_HANDLE hNet, IPV4_ADDR* ipAdd, IPV4_ADDR* mask, TCPIP_ARP_ENTRY_TYPE type)

  Summary:
    Removes all the entries belonging to a network interface.

  Description:
    This function removes all existent mappings belonging to a network interface.
    The performed operation:
    if(entry->type == type and entry->ipAdd & mask == ipAdd & mask)
        then remove entry

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet    -   Interface handle to use
    ipAdd   -   IP address
    mask    -   IP address of mask
    type    -   type of entries to remove:  
        valid types:    ARP_ENTRY_TYPE_PERMANENT
                        ARP_ENTRY_TYPE_COMPLETE
                        ARP_ENTRY_TYPE_INCOMPLETE
                        ARP_ENTRY_TYPE_ANY

  Returns:
    TCPIP_ARP_RESULT
        On Success - ARP_RES_OK
        On Failure - ARP_RES_BAD_TYPE (if no such type exists)
                     ARP_RES_NO_INTERFACE (if no such interface exists)


  Example:
    None

  Remarks:
    None
*/
TCPIP_ARP_RESULT      TCPIP_ARP_EntryRemoveNet(TCPIP_NET_HANDLE hNet, IPV4_ADDR* ipAdd, IPV4_ADDR* mask , TCPIP_ARP_ENTRY_TYPE type);


// *****************************************************************************
/*
Function:
    TCPIP_ARP_RESULT TCPIP_ARP_EntryQuery(TCPIP_NET_HANDLE hNet, size_t index, TCPIP_ARP_ENTRY_QUERY* pArpQuery)

  Summary:
    Queries an ARP cache entry using the index of the cache line.

  Description:
    This function can be used for displaying the cache contents

  Precondition:
    ARP module should have been initialized
    The index has to be a valid one i.e. < then TCPIP_ARP_CacheEntriesNoGet()
    populates the supplied TCPIP_ARP_ENTRY_QUERY query entry

  Parameters:
    hNet        -   Interface handle to use
    index       -   Index to cache
    pArpQuery   -   entry type, IP address, hardware address

  Returns:
    On Success - ARP_RES_OK
    On Failure - ARP_RES_BAD_INDEX (if index is out of range)
                 ARP_RES_NO_INTERFACE (if no such interface exists)

  Example:
    None

  Remarks:
    None
*/
TCPIP_ARP_RESULT      TCPIP_ARP_EntryQuery(TCPIP_NET_HANDLE hNet, size_t index, TCPIP_ARP_ENTRY_QUERY* pArpQuery);


// *****************************************************************************
/*
Function:
    size_t TCPIP_ARP_CacheEntriesNoGet(TCPIP_NET_HANDLE hNet, TCPIP_ARP_ENTRY_TYPE type);

  Summary:
    Used to retrieve the number of entries for a specific interface

  Description:
    The function will return the number of entries of the specified type that are currently in the cache

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet    -   Interface to use
    type    -   Type of ARP entry

  Returns:
    The number of entries of the specified type per interface

  Example:
    None

  Remarks:
    None
*/
size_t          TCPIP_ARP_CacheEntriesNoGet(TCPIP_NET_HANDLE hNet, TCPIP_ARP_ENTRY_TYPE type);


// *****************************************************************************
/*
Function
    TCPIP_ARP_RESULT TCPIP_ARP_CacheThresholdSet(TCPIP_NET_HANDLE hNet, int purgeThres, int purgeEntries);

  Summary:
    Sets the cache threshold for the specified interface in %

  Description:
    This function sets the current value of the cache threshold
    for the selected interface.
    During the ARP operation, once the number of entries
    in the cache is greater than the purge threshold
    a number of purgeEntries (usually one) will be discarded

  Precondition:
    ARP module should have been initialized

  Parameters:
    hNet            -   Interface handle to use
    purgeThres      -   Threshold to start cache purging
    purgeEntries    -   Number of entries to purge

  Returns:
    On Success - ARP_RES_OK
    On Failure - ARP_RES_NO_INTERFACE (if no such interface exists)

  Example:
    None

  Remarks:
    None
*/
TCPIP_ARP_RESULT TCPIP_ARP_CacheThresholdSet(TCPIP_NET_HANDLE hNet, int purgeThres, int purgeEntries);



#endif  // __ARP_H_



