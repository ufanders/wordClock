/*******************************************************************************
  Ethernet MAC Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ethmac_local.h

  Summary:
    Ethernet MAC driver local declarations and definitions.

  Description:
    This file contains the Ethernet MAC driver's local declarations and definitions.
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
//DOM-IGNORE-END

#ifndef _DRV_ETHMAC_LOCAL_H
#define _DRV_ETHMAC_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "tcpip/tcpip.h"
#include "tcpip/src/tcpip_private.h"
#include "tcpip/src/tcpip_mac_private.h"
#include "tcpip/tcpip_mac.h"
#include "tcpip/src/tcpip_mac_object.h"

#include "peripheral/eth/plib_eth.h"
#include "peripheral/eth/src/plib_eth_legacy.h"

#include "peripheral/int/plib_int.h"

#include "driver/ethphy/drv_ethphy.h"
#include "driver/ethmac/drv_ethmac.h"

// *****************************************************************************
// *****************************************************************************
// Section: Version Numbers
// *****************************************************************************
// *****************************************************************************
/* Versioning of the driver */

// *****************************************************************************
/* Ethernet MAC Driver Version Macros

  Summary:
    Ethernet MAC driver version.

  Description:
    These constants provide Ethernet MAC driver version information.
    The driver version is:
    DRV_ETHMAC_VERSION_MAJOR.DRV_ETHMAC_VERSION_MINOR.DRV_ETHMAC_VERSION_PATCH.
    It is represented in DRV_ETHMAC_VERSION asL
    MAJOR *10000 + MINOR * 100 + PATCH, so as to allow comparisons.
    It is also represented in string format in DRV_ETHMAC_VERSION_STR.
    DRV_ETHMAC_TYPE provides the type of the release when the release is alpha
    or beta. The interfaces DRV_ETHMAC_VersionGet and DRV_ETHMAC_VersionStrGet
    provide interfaces to the access the version and the version string.

  Remarks:
    Modify the return value of DRV_ETHMAC_VersionStrGet and the
    DRV_ETHMAC_VERSION_MAJOR, DRV_ETHMAC_VERSION_MINOR,
    DRV_ETHMAC_VERSION_PATCH and DRV_ETHMAC_VERSION_TYPE
*/

#define _DRV_ETHMAC_VERSION_MAJOR         0
#define _DRV_ETHMAC_VERSION_MINOR         3
#define _DRV_ETHMAC_VERSION_PATCH         0
#define _DRV_ETHMAC_VERSION_TYPE          "Alpha"
#define _DRV_ETHMAC_VERSION_STR           "0.3.0 Alpha"


// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* TCPIP Stack Event Descriptor

  Summary:

  Description:

  Remarks:
    None
*/
#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
typedef struct
{
    TCPIP_MAC_EVENT             _TcpEnabledEvents;          // group enabled notification events
    volatile TCPIP_MAC_EVENT    _TcpPendingEvents;          // group notification events that are set, waiting to be re-acknowledged
    ETH_LEGACY_EVENTS           _EthEnabledEvents;          // copy in ETH_LEGACY_EVENTS space
    volatile ETH_LEGACY_EVENTS  _EthPendingEvents;          // copy in ETH_LEGACY_EVENTS space
    TCPIP_MAC_EventF            _TcpNotifyFnc;              // group notification handler
    const void*                 _TcpNotifyParam;            // notification parameter
}DRV_ETHMAC_EVENT_DCPT;   // event descriptor per group

// transmit directly from within ISR
// provides a faster response when running out of
// TX descriptors
// ONLY when defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
//
//#define ETH_PIC32_INT_MAC_ISR_TX


#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)


#define ETH_PIC32_INT_MAC_MIN_RX_SIZE           128     // minimum RX buffer size
                                                        // less than this creates excessive fragmentation
                                                        // Keep it always multiple of 16!

#define ETH_PIC32_INT_MAC_MIN_TX_DESCRIPTORS    4       // minimum number of TX descriptors
                                                        // needed to accomodate zero copy and TCP traffic
                                                        //

// *****************************************************************************
/* PIC32 Embedded MAC Data Structure

  Summary:

  Description:

  Remarks:
    Dynamic data needed for the embedded PIC32 MAC
*/

typedef struct
{
    unsigned int        _macIx;             // index of the MAC, for multiple MAC's support
    unsigned int        _phyIx;             // index of the associated PHY
    SYS_OBJ_HANDLE      hPhySysObject;      // PHY object handle
    SYS_OBJ_HANDLE      hPhyClient;         // PHY handle
    union
    {
        uint16_t        val;
        struct
        {
            uint16_t    _init               : 1;    // the corresponding MAC is initialized
            uint16_t    _open               : 1;    // the corresponding MAC is opened
            uint16_t    _linkPresent        : 1;    // lif connection to the PHY properly detected : on/off
            uint16_t    _linkNegotiation    : 1;    // if an auto-negotiation is in effect : on/off
            uint16_t	_linkPrev           : 1;    // last value of the link status: on/off
            uint16_t	_linkUpDone       : 1;      // the link up sequence done
            // add another flags here
        };
    }                   _macFlags;          // corresponding MAC flags
    uint16_t            _nTxDescriptors;	// number of active TX descriptors
    uint16_t            _nRxDescriptors;	// number of active RX descriptors
    uint16_t            _RxBuffSize;        // size of each RX buffer 16 multiple

    SINGLE_LIST         _TxQueue;           // current TX queue; stores TX queued packets 

    // general stuff
    const void*            _AllocH ;        // allocation handle  
    _TCPIP_HEAP_CALLOC_PTR _callocF;        // allocation functions
    _TCPIP_HEAP_FREE_PTR   _freeF;

    // packet allocation functions
    _TCPIP_PKT_ALLOC_PTR    pktAllocF;
    _TCPIP_PKT_FREE_PTR     pktFreeF;
    _TCPIP_PKT_ACK_PTR      pktAckF;
    

    // timing and link status maintenance
    uint32_t            _linkUpTick;          // tick value when the link up sequence was started
    uint32_t            _linkWaitTick;          // tick value to wait for

    INT_SOURCE     _macIntSrc;             // this MAC interrupt source

#if defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    DRV_ETHMAC_EVENT_DCPT _pic32_ev_group_dcpt;
#if defined(ETH_PIC32_INT_MAC_ISR_TX)
    SINGLE_LIST         _TxAckQueue;        // TX acknowledgement queue; stores TX packets that need to be acknowledged 
#endif  // defined(ETH_PIC32_INT_MAC_ISR_TX)
#endif  // defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)

    // rx descriptor; supports maximum fragmentation
    DRV_ETHMAC_PKT_DCPT         rxPktDcpt[EMAC_RX_FRAGMENTS];

    // descriptor lists
    DRV_ETHMAC_DCPT_LIST _EnetTxFreeList;    // transmit list of free descriptors
    DRV_ETHMAC_DCPT_LIST _EnetTxBusyList;    // transmit list of descriptors passed to the Tx Engine
                                     // the _EnetTxBusyList always ends with an sw owned descriptor (hdr.EOWN=0);

    DRV_ETHMAC_DCPT_LIST _EnetRxFreeList;    // receive list of free descriptors
    DRV_ETHMAC_DCPT_LIST _EnetRxBusyList;    // receive list of descriptors passed to the Rx Engine

    DRV_ETHMAC_DCPT_LIST* _EnetTxFreePtr;    // pointer to the transmit list of free descriptors
    DRV_ETHMAC_DCPT_LIST* _EnetTxBusyPtr;    // pointer to the transmit list of descriptors passed to the Tx Engine

    DRV_ETHMAC_DCPT_LIST* _EnetRxFreePtr;    // pointer to the receive list of free descriptors
    DRV_ETHMAC_DCPT_LIST* _EnetRxBusyPtr;    // pointer to the receive list of descriptors passed to the Rx Engine



    // debug: run time statistics
    TCPIP_MAC_RX_STATISTICS _rxStat;
    TCPIP_MAC_TX_STATISTICS _txStat;


} DRV_ETHMAC_INSTANCE_DATA;


// *****************************************************************************
/* PIC32 Embedded MAC Descriptor

  Summary:

  Description:

  Remarks:
    the embedded PIC32 MAC descriptor
    support for multiple instances
*/

typedef struct
{
    const TCPIP_MAC_OBJECT* pObj;  // safe cast to TCPIP_MAC_DCPT
    DRV_ETHMAC_INSTANCE_DATA     mData;  // specific PIC32 MAC data

} DRV_ETHMAC_INSTANCE_DCPT;


#endif //#ifndef _DRV_ETHMAC_LOCAL_H

/*******************************************************************************
 End of File
*/

