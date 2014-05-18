/*******************************************************************************
  MAC Module Definitions for the Microchip TCP/IP Stack

  Company:
    Microchip Technology Inc.
    
  File Name:
    tcpip_mac.h

  Summary:
    
  Description:
    
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright © 2012 released Microchip Technology Inc.  All rights reserved.

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

#ifndef __TCPIP_MAC_H_
#define __TCPIP_MAC_H_

#include <stdint.h>
#include <stdbool.h>

#include "../driver/ethmac/drv_ethernet_flags.h"

// *****************************************************************************
// *****************************************************************************
// Section: MAC interface definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*  MAC Address

  Summary:
    Definition of a MAC address.

  Description:
    This structure defines the physical MAC address of an interface.

  Remarks:        
    None
*/

typedef struct __attribute__((__packed__))
{
    uint8_t v[6];
} TCPIP_MAC_ADDR;

// *****************************************************************************
/*  MAC Ethernet Header

  Summary:
    Definition of a MAC frame header.

  Description:
    This structure defines the generic Ethernet header
    that starts all the Ethernet frames.

  Remarks:        
    None
*/

typedef struct  __attribute__((aligned(2), packed))
{
    TCPIP_MAC_ADDR      DestMACAddr;
    TCPIP_MAC_ADDR      SourceMACAddr;
    uint16_t            Type;
} TCPIP_MAC_ETHERNET_HEADER;
        
// *****************************************************************************
/*  MAC Initialization Data

  Summary:
    Data that's passed to the MAC at initialization time.

  Description:
    This structure defines the MAC initialization data for the
    PIC32 MAC/Ethernet controller.

  Remarks:        
    - Each supported MAC has its own specific init/configuration data
*/

typedef struct
{
    /*  number of TX descriptors */
    int                 nTxDescriptors;

    /*  size of the corresponding RX buffer */
    int                 rxBuffSize;

    /*  number of RX descriptors */
    int                 nRxDescriptors;

    /*  flags to use for the ETH connection */
    ETH_OPEN_FLAGS      ethFlags;

    /*  PHY configuration  */
    ETHPHY_CONFIG_FLAGS phyFlags;

    /*  PHYs initialization delay (milliseconds) */
    /*  for insuring that the PHY is ready to transmit data. */
    int                 linkInitDelay;

    /* PHY address, as configured on the board. */
    /* All PHYs respond to address 0 */
    int                 phyAddress;

}TCPIP_MODULE_MAC_PIC32INT_CONFIG;

typedef struct
{
}TCPIP_MODULE_MAC_MRF24W_CONFIG;


// *****************************************************************************
/*  MAC Handle

  Summary:
    Handle to a MAC driver.

  Description:
    This data type defines a MAC client handle.
  
  Remarks:        
    None.
*/

typedef const void* TCPIP_MAC_HANDLE;


// *****************************************************************************
/*  MAC Segment Flags

  Summary:
    Flags belonging to MAC data segment.

  Description:
    This enumeration contains the definitions of MAC segment flags:
    segment allocation flags and general purpose flags.
  
  Remarks:        
    - 16 bits only segment flags are supported.
*/


typedef enum
{
    /*  Segment can not be dynamically deallocated. */
    /*  Set when the segment is allocated */
    TCPIP_MAC_SEG_FLAG_STATIC       = 0x0001, 

    /*  If set, it's a TX segment; otherwise, is a RX packet. */
    TCPIP_MAC_SEG_FLAG_TX           = 0x0002, 


    /*  User available segment flags. */
    TCPIP_MAC_SEG_FLAG_USER         = 0x0100, 


}TCPIP_MAC_SEGMENT_FLAGS;


// *****************************************************************************
/*  TCPIP MAC Data Segment

  Summary:
    A data segment that's part of a TX/RX packet.

  Description:
    Structure of a segment buffer transferred with the MAC.
    A MAC TX or RX packet can consist of multiple data segments.
    On TX the MAC has to be able to transmit packets that span
    multiple data segments.
    On RX of a network frame the MAC may have to use multiple segments to 
    construct a packet.
    (For performance reasons, a contiguous MAC packet, with just one segment, if possible, is preferred).
   
  
  Remarks:
    - See notes for the segLoadOffset member.
    - On 32 bit machines the segment payload is allocated so that it is always
      32 bit aligned and its size is 32 bits multiple.
      The segLoadOffset adds to the payload address and insures that the network layer data
      is 32 bit aligned.

*/

typedef struct _tag_MAC_DATA_SEGMENT
{
    /*  Multi-segment support, next segment in the chain. */
    struct _tag_MAC_DATA_SEGMENT* next;      

    /*  Pointer to segment data payload.
        It specifies the address of the 1st byte to be transmitted. */
    uint8_t*                 segLoad;        

    /*  Segment payload size; Number of bytes from this segment
        that has to be transmitted. */
    uint16_t                 segLen;         

    /*  Segment allocated total usable size.
        This does not include the segLoadOffset (see below). */
    uint16_t                 segSize;        

    /*  TCPIP_MAC_SEGMENT_FLAGS segment flags:
        TCPIP_MAC_SEG_FLAG_STATIC and TCPIP_MAC_SEG_FLAG_RX, TCPIP_MAC_SEG_FLAG_RX */
    uint16_t                 segFlags;       

    /*  Offset in bytes between the address pointed by segLoad 
        and the address where the segment buffer starts/was allocated.
        It specifies some available space at the beginning of the
        segment buffer. */

    /*  Note 1. This offset is used as a performance improvement. 
        It allows for the MAC frame to start on an unaligned address 
        but enforces the alignment of the network layer data and 
        improves the IP checksum calculation. 
        The value of this offset is MAC dependent. 
        A typical value for an Ethernet MAC should be 2 bytes 
        (size of the MAC frame is 14 bytes).
        Note 2. Normally only the 1st segment of a packet needs 
        this extra room at the beginning of the segment buffer.
        Note 3. The MAC may make use of this space at the beginning of the
        segment buffer. This is a space reserved for the MAC purposes. 
        Note 4. It is up to the MAC to check that the value of this offset is enforced. */
    /*  PIC32 MAC specific notes: 
            1. The MAC will reject the packet if the load offset is not at least 2 bytes.
            2. The PIC32 MAC uses these 2 bytes to calculate the offset between the segLoad
               and the TCPIP_MAC_PACKET packet it belongs to.
               That means that the TCPIP_MAC_PACKET* and the segLoad cannot be
               more than 64 KB apart! */
    uint16_t                 segLoadOffset;  

    /*  Additional client segment data. Ignored by the MAC driver. */
    uint8_t                  segClientData[4];

    /*  Additional client segment payload; Ignored by the MAC driver. */
    uint8_t                  segClientLoad[0];
}TCPIP_MAC_DATA_SEGMENT;




// *****************************************************************************
/*  MAC Data Packet Definitions */


// *****************************************************************************
/*  Received Packet Status

  Summary:
    Status of a received packet.

  Description:
    This structure contains the status of a received packet.
  
  Remarks:
    - Not all the MACs have hardware support for the received packet status.

*/

typedef struct __attribute__ ((__packed__))
{

    /*  correct checksum filled in */
    unsigned        chksumOk        : 1;

    /*  Packet payload checksum */
    unsigned        pktChecksum     :16;

    /*  Runt packet received */
    unsigned        runtPkt         : 1;

    /*  Unicast, not me packet, */
    unsigned        notMeUcast      : 1;

    /*  Hash table match */
    unsigned        htMatch         : 1;

    /*  Magic packet match */
    unsigned        magicMatch      : 1;

    /*  Pattern match match */
    unsigned        pmMatch         : 1;

    /*  Unicast match */
    unsigned        uMatch          : 1;

    /*  Broadcast match */
    unsigned        bMatch          : 1;

    /*  Multicast match */
    unsigned        mMatch          : 1;

    /*  Received bytes */
    unsigned        rxBytes         :16;

    /*  CRC error in packet */
    unsigned        crcError        : 1;

    /*  Receive length check error */
    unsigned        lenError        : 1;

    /*  Receive length out of range */
    unsigned        lenRange        : 1;

    /*  Receive OK */
    unsigned        rxOk            : 1;

    /*  Multicast packet */
    unsigned        mcast           : 1;

    /*  Broadcast packet */
    unsigned        bcast           : 1;

    /*  Control frame received */
    unsigned        rxCtrl          : 1;

    /*  Received VLAN tagged frame */
    unsigned        rxVLAN          : 1;

}TCPIP_MAC_PACKET_RX_STAT;


// *****************************************************************************
/*  MAC Packet Flags

  Summary:
    Flags belonging to MAC packet.

  Description:
    This enumeration contains the definitions of MAC packet flags:
    packet allocation flags and general purpose flags.
  
  Remarks:
    - 16 bits only packet flags are supported.
*/


typedef enum
{
    /* Packet can not be dynamically deallocated.
       Set when the packet is allocated. */
    TCPIP_MAC_PKT_FLAG_STATIC       = 0x0001,       

    /* If set, it is a TX packet/segment. Otherwise, it is a RX packet.*/
    TCPIP_MAC_PKT_FLAG_TX           = 0x0002,

    /* Packet data spans multiple segments - ZC functionality.
       If not set then the packet has only one data segment.
       It is set by the MAC driver when a RX packet spans multiple
       data segments. */
    TCPIP_MAC_PKT_FLAG_SPLIT        = 0x0004,       


    /* Packet data is queued somewhere, cannot be freed.
       The flag is set by the packet source when the packet is passed
       for further processing to a destination.
       Cleared by the packet destination when the packet processing was completed.*/
    TCPIP_MAC_PKT_FLAG_QUEUED       = 0x0008,       


    /* RX flag, MAC updated. Specifies an unicast packet. */
    TCPIP_MAC_PKT_FLAG_UNICAST      = 0x0010,      

    /* RX flag, MAC updated. Specifies a broadcast packet. */
    TCPIP_MAC_PKT_FLAG_BCAST        = 0x0020,      

    /* RX flag, MAC updated. Specifies an multicast packet. */
    TCPIP_MAC_PKT_FLAG_MCAST        = 0x0040,      

    /* Packet cast mask bits. */
    TCPIP_MAC_PKT_FLAG_CAST_MASK    = 0x0070,      

    /* Packet cast mask. Specifies a packet where the MCAST/BCAST fields
       are not updated by the MAC RX process. */
    TCPIP_MAC_PKT_FLAG_CAST_DISABLED= 0x0000,      

    /* Available user flags. */
    TCPIP_MAC_PKT_FLAG_USER         = 0x0100,      


}TCPIP_MAC_PACKET_FLAGS;


// *****************************************************************************
/*  MAC Packet Acknowledge Result

  Summary:
    List of MAC return codes for a packet acknowledge function.

  Description:
    This enumeration contains the list of MAC codes
    used for a packet acknowledgment.
  
  Remarks:        
    - 16 bits only acknowledge results are supported.
    - Positive codes indicate success.
      Negative codes indicate a failure of some sort.

*/



typedef enum
{
    /* packet result unknown, unspecified */
    TCPIP_MAC_PKT_ACK_NONE              = 0,    

    /* TX success code -  packet was transmitted successfully */
    TCPIP_MAC_PKT_ACK_TX_OK             = 1, 

    /* RX success code -  packet was received/processed successfully */
    TCPIP_MAC_PKT_ACK_RX_OK             = 2,

    /* TX:packet was dropped because the link was down */
    TCPIP_MAC_PKT_ACK_LINK_DOWN         = -1, 

    /* TX:packet was dropped because the network is down */ 
    TCPIP_MAC_PKT_ACK_NET_DOWN          = -2,

    /* TX:packet was dropped because the buffer type is not supported */
    TCPIP_MAC_PKT_ACK_BUFFER_ERR        = -3,

    /* TX:packet was dropped because of an ARP timeout */
    TCPIP_MAC_PKT_ACK_ARP_TMO           = -4, 

    /* TX:packet associated interface is down or non existent*/ 
    TCPIP_MAC_PKT_ACK_ARP_NET_ERR       = -5,   

    /* RX: packet was dropped because the checksum was incorrect */
    TCPIP_MAC_PKT_ACK_CHKSUM_ERR        = -10, 

    /* RX: packet was dropped because of wrong interface source address */
    TCPIP_MAC_PKT_ACK_SOURCE_ERR        = -11,

    /* RX: packet was dropped because the type was unknown  */
    TCPIP_MAC_PKT_ACK_TYPE_ERR          = -12,

    /* RX: internal packet structure error  */
    TCPIP_MAC_PKT_ACK_STRUCT_ERR        = -13,  

    /* RX: the packet protocol couldn't find a destination for it */
    TCPIP_MAC_PKT_ACK_PROTO_DEST_ERR    = -14,

    /* RX: the packet too fragmented  */
    TCPIP_MAC_PKT_ACK_FRAGMENT_ERR      = -15, 

    /* RX: the packet destination is closing */
    TCPIP_MAC_PKT_ACK_PROTO_DEST_CLOSE  = -16,

    /* RX: memory allocation error */
    TCPIP_MAC_PKT_ACK_ALLOC_ERR         = -17,  


}TCPIP_MAC_PKT_ACK_RES;


// *****************************************************************************
/*  MAC Packet forward reference

  Summary:
   Forward reference to a MAC packet.

  Description:
    Forward reference needed in the MAC packet acknowledge
    function.

  
  Remarks:        
    None
*/

typedef struct _tag_TCPIP_MAC_PACKET    TCPIP_MAC_PACKET;



// *****************************************************************************
/*  MAC Acknowledge Function

  Summary:
    Prototype of a MAC packet acknowledge function.

  Description:
    This is the prototype of the function that the MAC calls
    once the TX/RX packet processing is done.

  
  Remarks:        
    None
*/
typedef void    (*TCPIP_MAC_PACKET_ACK_FUNC)(TCPIP_MAC_PACKET* pkt,  const void* param);


// *****************************************************************************
/*  MAC Packet

  Summary:
    A TX/RX packet descriptor.

  Description:
    Structure of a packet transferred with the MAC.
    This is the structure used for both TX and RX.
    See the description of each individual field.

  Remarks:
    - Specific TCP/IP stack implementations might offer packet support functions
      to assist in driver development.
      (For the MCHP TCP/IP stack see tcpip_packet.h)
    - Since the packets may be dynamically allocated, an acknowledge function can result
      in data deallocation (blocking).
      Therefore the acknowledgment function should NOT be called
      from within an interrupt context.

*/
struct _tag_TCPIP_MAC_PACKET
{
    /* Multi-packet/queuing support.
       This field is used for chaining/queuing packets. */
    struct _tag_TCPIP_MAC_PACKET*   next;

    /* Packet acknowledgment function.
       On TX: A stack module owns the packet.
        Once the MAC is done transmitting the packet (success or failure)
        it has to set an appropriate acknowledge result in the ackRes field ( a TCPIP_MAC_PKT_ACK_RES)
        adjust some packet flags (see TCPIP_MAC_PKT_FLAG_QUEUED) and call the packet acknowledge function (ackFunc).
        This call informs the packet owner that the MAC is done with using that packet.
        It is up to the implementation what action the ackFunc takes: reuse, free, discard the packet or if
        some of the above steps are included in the ackFunc itself.
       On RX: The packet is under the MAC control and it's passed to the stack
        when it contains valid data.
        Once the recipient of the packet (IP, ICMP, UDP, TCP, etc.) is done with the packet processing
        it has to set an appropriate acknowledge result in the ackRes field ( a TCPIP_MAC_PKT_ACK_RES)
        adjust some packet flags (see TCPIP_MAC_PKT_FLAG_QUEUED) and call the packet acknowledge function (ackFunc).
        This call informs the packet owner (the MAC) that the processing of the packet is completed.
        It is up to the implementation what action the ackFunc takes: reuse, free, discard the packet or if
        some of the above steps are included in the ackFunc itself. */
    TCPIP_MAC_PACKET_ACK_FUNC       ackFunc;

    /* Associated acknowledgment parameter to be used when ackFunc is called.
       For TX packets: this is a client supplied parameter and
       is not used by the MAC driver.
       For RX: If the MAC owns the RX packet then the MAC driver can use this field
       for further dispatching in the MAC driver owned acknowledgment function. */
    const void*                     ackParam;

    /* Data (root) segment associated to this packet.
       It is up to the design if the root data segment associated with a packet
       is contiguous in memory with the packet itself.
       It can be 0 if the packet has no associated data. */
    TCPIP_MAC_DATA_SEGMENT*         pDSeg;

    /* Pointer to the MAC frame.
       On TX: the sending higher layer protocol updates this field.
       On RX: the MAC driver updates this field before handing over the packet.
       (MCHP TCP/IP stack note: The packet allocation function update this field automatically). */
    uint8_t*                        pMacLayer;

    /* Pointer to the network layer data.
       The MAC driver shouldn't need this field.
       On TX: the sending higher layer protocol updates this field.
       On RX: the MAC driver updates this field before handing over the packet.
       (MCHP TCP/IP stack note: The packet allocation function update this field automatically). */
    uint8_t*                        pNetLayer;

    /* Pointer to the transport layer.
       The MAC driver shouldn't need this field.
       On TX: the sending higher layer protocol updates this field.
       On RX: the MAC driver updates this field before handing over the packet.
       (MCHP TCP/IP stack note: The packet allocation function update this field automatically). */
    uint8_t*                        pTransportLayer;

    /* Total length of the transport layer.
       The MAC driver shouldn't need this field. */
    uint16_t                        totTransportLen;

    /* TCPIP_MAC_PACKET_FLAGS associated packet flags.
       On TX: the sending higher layer protocol updates this field.
       On RX: the MAC driver updates this field before handing over the packet.
       Flags that the MAC driver has to set/clear:
            TCPIP_MAC_PKT_FLAG_RX
               and TCPIP_MAC_PKT_FLAG_UNICAST, TCPIP_MAC_PKT_FLAG_BCAST and TCPIP_MAC_PKT_FLAG_MCAST
            TCPIP_MAC_PKT_FLAG_QUEUED
            TCPIP_MAC_PKT_FLAG_SPLIT */
    uint16_t                        pktFlags;

    /* Time stamp value. Statistics, info.
       On TX: the sending higher layer protocol updates this field.
            The MAC driver shouldn't need this field
       On RX: the MAC driver updates this field before handing over the packet. */
    uint32_t                        tStamp;

    /* The packet interface.
       On TX: the sending higher layer protocol updates this field.
            The MAC driver doesn't use this field.
       On RX: the receiving higher level protocol updates this value.
            The MAC driver doesn't use this field. */
    const void*                     pktIf;

    /* A TCPIP_MAC_PKT_ACK_RES code associated with the packet.
       On TX: The MAC driver sets this field when calling the packet ackFunc.
       On RX: The higher level protocol which is the recipient of the packet
             sets this field when calling the packet ackFunc. */
    int16_t                         ackRes;

    /* Client/padding data; ignored by the MAC driver.
       It can be used by the packet client. */
    uint16_t                        pktClientData;

    /* Additional client packet payload, variable packet data.
       Ignored by the MAC driver. */
    uint32_t                        pktClientLoad[0];
};


// *****************************************************************************
/*  MAC Result Enumeration

  Summary:
    List of return codes from MAC functions.

  Description:
    This is the list of codes that the MAC uses to specify
    the outcome of a MAC function.

  
  Remarks:        
    - Benign operation results - always have positive values.
    - Error codes - always negative values.

*/
typedef enum
{
    /*  operation successful */
    TCPIP_MAC_RES_OK                = 0,   

    /*  operation is pending upon some hardware resource.
        call again to completion */
    TCPIP_MAC_RES_PENDING           = 1,    

    /*  unsupported type */
    TCPIP_MAC_RES_TYPE_ERR          = -1,   

    /*  device is in use */
    TCPIP_MAC_RES_IS_BUSY           = -2,   

    /*  generic initialization failure */
    TCPIP_MAC_RES_INIT_FAIL         = -3,   

    /*  PHY initialization failure */
    TCPIP_MAC_RES_PHY_INIT_FAIL     = -4,   

    /*  Event system initialization failure */
    TCPIP_MAC_RES_EVENT_INIT_FAIL   = -5,   

    /*  unsupported operation */
    TCPIP_MAC_RES_OP_ERR            = -6,   

    /*  memory allocation error */
    TCPIP_MAC_RES_ALLOC_ERR         = -7,   

    /*  already instantiated, initialized error */
    TCPIP_MAC_RES_INSTANCE_ERR      = -8,   

    /*  too fragmented, RX buffer too small */
    TCPIP_MAC_RES_FRAGMENT_ERR      = -9,   

    /*  unsupported/corrupted packet error */
    TCPIP_MAC_RES_PACKET_ERR        = -10,  

    /*  TX queue exceeded the limit */
    TCPIP_MAC_RES_QUEUE_TX_FULL     = -11,  


}TCPIP_MAC_RES;


// *****************************************************************************
// *****************************************************************************
// Section: MAC interface Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* TCP/IP MAC Action

  Summary:
    Network interface action for initialization/deinitialization

  Description:

  Remarks:
    None.

*/

typedef enum
{
    /*  stack is initialized  */
    TCPIP_MAC_ACTION_INIT,         

    /*  stack is reinitialized */
    TCPIP_MAC_ACTION_REINIT,       

    /*  stack is deinitialized */
    TCPIP_MAC_ACTION_DEINIT,       

    /*  interface is brought up  */
    TCPIP_MAC_ACTION_IF_UP,        

    /*  interface is brought down */
    TCPIP_MAC_ACTION_IF_DOWN,      

}TCPIP_MAC_ACTION;

// *****************************************************************************
/* TCP/IP MAC Power Mode

  Summary:
    Supported MAC power mode state.

  Description:
    List of supported MAC power modes.

  Remarks:
    None.

*/

// 
typedef enum
{
    /*  unknown power mode */
    TCPIP_MAC_POWER_NONE,     

    /*  up and running; valid for init/reinit */
    TCPIP_MAC_POWER_FULL,     

    /*  low power mode; valid for init/reinit */
    TCPIP_MAC_POWER_LOW,      

    /*  interface is down */
    TCPIP_MAC_POWER_DOWN,     

}TCPIP_MAC_POWER_MODE;

// *****************************************************************************
/* TCP/IP MAC Event

  Summary:
    Defines the possible MAC event types.

  Description:
    TCP/IP MAC Events Codes.
    
    This enumeration defines all the possible events that can be reported
    by the MAC to the stack.
    
    Depending on the type of the hardware Ethernet/Wi-Fi interface, etc.,
    not all events are possible.

  Remarks:
    None

*/

typedef enum
{
    /*  no event */
    TCPIP_MAC_EV_NONE               = 0x0000,
    
    /*  RX triggered events: A receive packet is pending */
    TCPIP_MAC_EV_RX_PKTPEND         = 0x0001,

    /*  RX triggered events: RX FIFO overflow (system level latency, no descriptors, etc.) */
    TCPIP_MAC_EV_RX_OVFLOW          = 0x0002,

    /*  RX triggered events: no RX descriptor available to receive a new packet */
    TCPIP_MAC_EV_RX_BUFNA           = 0x0004,

    /*  RX triggered events: There's RX data available */
    TCPIP_MAC_EV_RX_ACT             = 0x0008,

    /*  RX triggered events: A packet was successfully received */
    TCPIP_MAC_EV_RX_DONE            = 0x0010,

    /*  RX triggered events: the number of received packets is greater than or equal to than the RX Full Watermark */
    TCPIP_MAC_EV_RX_FWMARK          = 0x0020,

    /*  RX triggered events: the number of received packets is less than or equal to than the RX Empty Watermark */
    TCPIP_MAC_EV_RX_EWMARK          = 0x0040,

    /*  RX triggered events: a bus error encountered during an RX transfer */
    TCPIP_MAC_EV_RX_BUSERR          = 0x0080,

    /*  TX triggered events: A packet was transmitted and its status is available */
    TCPIP_MAC_EV_TX_DONE            = 0x0100,          

    /*  TX triggered events: a TX packet was aborted by the MAC (jumbo/system underrun/excessive defer/late collision/excessive collisions) */
    TCPIP_MAC_EV_TX_ABORT           = 0x0200,

    /*  TX triggered events: a bus error encountered during a TX transfer */
    TCPIP_MAC_EV_TX_BUSERR          = 0x0400,

    /*  Connection triggered events: Connection established */
    TCPIP_MAC_EV_CONN_ESTABLISHED   = 0x0800,

    /*  Connection triggered events: Connection lost */
    TCPIP_MAC_EV_CONN_LOST          = 0x1000,

    /*  Useful Masks: all RX related events */
    TCPIP_MAC_EV_RX_ALL             = (TCPIP_MAC_EV_RX_PKTPEND|TCPIP_MAC_EV_RX_OVFLOW|TCPIP_MAC_EV_RX_BUFNA|TCPIP_MAC_EV_RX_ACT|TCPIP_MAC_EV_RX_DONE|TCPIP_MAC_EV_RX_FWMARK|TCPIP_MAC_EV_RX_EWMARK|TCPIP_MAC_EV_RX_BUSERR),

    /*  Useful Masks: all TX related events */
    TCPIP_MAC_EV_TX_ALL            = (TCPIP_MAC_EV_TX_DONE|TCPIP_MAC_EV_TX_ABORT|TCPIP_MAC_EV_TX_BUSERR),

    /*  abnormal traffic/system events: Action should be taken accordingly by the stack (or the stack user) */
    TCPIP_MAC_EV_RXTX_ERRORS        = (TCPIP_MAC_EV_RX_OVFLOW|TCPIP_MAC_EV_RX_BUFNA|TCPIP_MAC_EV_RX_BUSERR|TCPIP_MAC_EV_TX_ABORT|TCPIP_MAC_EV_TX_BUSERR),

    /*  Mask of all Connection related events */
    TCPIP_MAC_EV_CONN_ALL            = (TCPIP_MAC_EV_CONN_ESTABLISHED|TCPIP_MAC_EV_CONN_LOST)

}TCPIP_MAC_EVENT;



// *****************************************************************************
/*  Handle to a heap

  Summary:
   A handle used for memory allocation functions.

  Description:
    This is the handle that's required for accessing memory allocation functions.
   
  
  Remarks:        
    None
*/
typedef const void* TCPIP_MAC_HEAP_HANDLE;

// *****************************************************************************
/*
  Memory Allocation Function:
    typedef void*   (*TCPIP_MAC_HEAP_MallocF)(TCPIP_MAC_HEAP_HANDLE heapH, size_t nBytes);

  Summary:
    MAC allocation function prototype.

  Description:
    This malloc style function is used by the MAC to allocate memory.


  Parameters:
    - heapH       - heap handle to be used in call
    - nBytes      - number of bytes requested

  Returns:
   - a valid pointer if the allocation request succeeded
   - 0 otherwise


  Remarks:   
   None.

*/
typedef void*   (*TCPIP_MAC_HEAP_MallocF)(TCPIP_MAC_HEAP_HANDLE heapH, size_t nBytes);

// *****************************************************************************
/*
  Memory Allocation Function:
    typedef void*   (*TCPIP_MAC_HEAP_CallocF)(TCPIP_MAC_HEAP_HANDLE heapH, size_t nElems, size_t elemSize);

  Summary:
    MAC allocation function prototype.

  Description:
    This calloc style function is used by the MAC to allocate memory.


  Parameters:
    - heapH       - heap handle to be used in call
    - nElems      - number of identical elements requested
    - elemSize    - size of one element, in bytes

  Returns:
   - a valid pointer if the allocation request succeeded
   - 0 otherwise


  Remarks:   
   None.

*/
typedef void*   (*TCPIP_MAC_HEAP_CallocF)(TCPIP_MAC_HEAP_HANDLE heapH, size_t nElems, size_t elemSize);

// *****************************************************************************
/*
  Memory Allocation Function:
    typedef int     (*TCPIP_MAC_HEAP_FreeF)(TCPIP_MAC_HEAP_HANDLE heapH, const void* pBuff);

  Summary:
    MAC allocation function prototype.

  Description:
    This free style function is used by the MAC to free allocated memory.


  Parameters:
    - heapH       - heap handle to be used in call
    - pBuff       - pointer to memory previously returned by a memory allocation function 

  Returns:
   - a non zero number if the free request succeeded
   - 0 otherwise


  Remarks:   
   None.

*/
typedef int     (*TCPIP_MAC_HEAP_FreeF)(TCPIP_MAC_HEAP_HANDLE heapH, const void* pBuff);


// *****************************************************************************
/*
  Packet Allocation Function:
    typedef TCPIP_MAC_PACKET* (*TCPIP_MAC_PKT_AllocF)(uint16_t pktLen, uint16_t segLoadLen, TCPIP_MAC_PACKET_FLAGS flags);

  Summary:
    MAC packet allocation function prototype.

  Description:
    This function is used by the MAC to allocate a TCPIP_MAC_PACKET packet.


  Parameters:
    - pktLen      - the size of the packet (it will be 32 bits rounded up)
    - segLoadLen  - the payload size for the segment associated to this packet.
                  Payload is always 32 bit aligned.
                  If 0 no segment is created/attached to the packet.
    - flags       - packet flags

  Returns:
   - a valid packet pointer if the allocation request succeeded
   - 0 otherwise


  Remarks:   
   - The returned allocated packet should always have the TCPIP_MAC_ETHERNET_HEADER added to the packet.

*/
typedef TCPIP_MAC_PACKET* (*TCPIP_MAC_PKT_AllocF)(uint16_t pktLen, uint16_t segLoadLen, TCPIP_MAC_PACKET_FLAGS flags);


// *****************************************************************************
/*
  Packet Allocation Function:
    typedef void      (*TCPIP_MAC_PKT_FreeF)(TCPIP_MAC_PACKET* pPkt);

  Summary:
    MAC packet free function prototype.

  Description:
    This function is used by the MAC to free a previously allocated TCPIP_MAC_PACKET packet.


  Parameters:
    - pPkt      - pointer to a previously allocated packet.

  Returns:
   None.


  Remarks:   
    - The function will free a previously allocated packet.
      However packets or segments marked with TCPIP_MAC_PKT_FLAG_STATIC/TCPIP_MAC_SEG_FLAG_STATIC
      are not freed.
    - Also note that this function does not free explicitly the external segment payload.
      A payload that was created contiguously when the segment was created
      will be automatically freed by this function.

*/
typedef void      (*TCPIP_MAC_PKT_FreeF)(TCPIP_MAC_PACKET* pPkt);

// *****************************************************************************
/*
  Packet Acknowledgment Function:
    typedef void      (*TCPIP_MAC_PKT_AckF)(TCPIP_MAC_PACKET* pPkt, TCPIP_MAC_PKT_ACK_RES ackRes);

  Summary:
    MAC packet acknowledge function prototype.

  Description:
    This function is used by the MAC to acknowledge a TCPIP_MAC_PACKET packet
    when the packet processing is completed.


  Parameters:
    - pPkt      - pointer to a valid TCPIP_MAC_PACKET packet.
    - ackRes    - the result of the packet processing

  Returns:
   None.


  Remarks:
    - A TCPIP_MAC_PACKET packet always has an acknowledgment function.
    - This function should clear the TCPIP_MAC_PKT_FLAG_QUEUED flag.
    - The packet's ackRes is updated only if the parameter ackRes != TCPIP_MAC_PKT_ACK_NONE.

*/
typedef void      (*TCPIP_MAC_PKT_AckF)(TCPIP_MAC_PACKET* pPkt, TCPIP_MAC_PKT_ACK_RES ackRes);


// *****************************************************************************
/*
  Event notification Function:
    typedef void    (*TCPIP_MAC_EventF)(TCPIP_MAC_EVENT event, const void* eventParam);

  Summary:
    MAC event notification handler.

  Description:
    This function describes the MAC event notification handler.
    This is a handler specified by the user of the MAC (the TCP/IP stack).
    The stack can use the handler to be notified of MAC events.
    Whenever a notification occurs the passed events have to be eventually processed:
    - Stack should process the TCPIP_EV_RX_PKTPEND/TCPIP_EV_RX_DONE, TCPIP_EV_TX_DONE  events
    - Process the specific (error) condition
    - Acknowledge the events by calling TCPIP_MAC_EventAcknowledge() so that they can be re-enabled.

  Parameters:
    - event        - event that's reported (multiple events can be OR-ed)  
    - eventParam   - user parameter that's used in the notification handler
    
  Returns:
   None


  Remarks:   
    - The notification handler will be called from the ISR which detects the corresponding event.
      The event notification handler has to be kept as short as possible and non-blocking.
      Mainly useful for RTOS integration where this handler will wake-up a thread that waits for a MAC event to occur.
    - The event notification system also enables the user of the TCPIP stack to call into the stack
      for processing only when there are relevant events rather than being forced to periodically call
      from within a loop at unknown moments.
    - Without a notification handler the stack user can still call TCPIP_MAC_EventPendingGet() to see if processing by the stack needed.
    - This is a default way of adding MAC interrupt processing to the TCP/IP stack.
*/
typedef void    (*TCPIP_MAC_EventF)(TCPIP_MAC_EVENT event, const void* eventParam);


// *****************************************************************************
/*  MAC Process Flags

  Summary:
    List of the MAC processing flags.


  Description:
    List of specific MAC processing flags that indicate to the user of the MAC
    (TCP/IP stack) the processing that's expected by the MAC driver.

  
  Remarks:        
    Multiple flags can be "ORed".
*/
typedef enum
{
    /*  the stack never has to call the TCPIP_MAC_Process function */
    TCPIP_MAC_PROCESS_FLAG_NONE     = 0x0000, 

    /*  the stack has to call the TCPIP_MAC_Process after an RX signal */
    TCPIP_MAC_PROCESS_FLAG_RX       = 0x0001, 

    /*  the stack has to call the TCPIP_MAC_Process after an TX signal */
    TCPIP_MAC_PROCESS_FLAG_TX       = 0x0002, 


    /*  the stack has to call the TCPIP_MAC_Process after any type of signal */
    TCPIP_MAC_PROCESS_FLAG_ANY      = 0x0100, 

}TCPIP_MAC_PROCESS_FLAGS;


// *****************************************************************************
/*  MAC Initialization Data

  Summary:
    Data structure that's passed to the MAC at the initialization time.

  Description:
    This is the data structure that the MAC user (TCP/IP stack)
    passes on to the MAC driver at the MAC initialization time.
    It contains all the data needed for the MAC to initialize itself and
    to start processing packets.

  
  Remarks:        
    - Most of the data that's passed in this structure is permanent data.
      It is maintained by the stack for one full session
      i.e., across Initialize() -> DeInitialize() calls.
    - Some fields are module specific though
      (like the memory allocation handle, allocation functions, etc.)
      that could be different from one module to the other.

*/
typedef struct
{
    /*  number of the interfaces supported in this session */
    int     nIfs;         

    /*  malloc type allocation function */
    TCPIP_MAC_HEAP_MallocF  mallocF;

    /*  calloc type allocation function */
    TCPIP_MAC_HEAP_CallocF  callocF;

    /*  free type allocation free function */
    TCPIP_MAC_HEAP_FreeF    freeF;

    /*  handle to be used in the stack allocation service calls */
    TCPIP_MAC_HEAP_HANDLE   memH; 

    /*  packet allocation function */
    TCPIP_MAC_PKT_AllocF    pktAllocF;

    /*  packet free function */
    TCPIP_MAC_PKT_FreeF     pktFreeF;

    /*  packet allocation function */
    TCPIP_MAC_PKT_AckF      pktAckF;

    /*  Event notification function. 
        used by the MAC for event reporting. */
    TCPIP_MAC_EventF        eventF;

    /*  Parameter to be used when the event function is called. */
    const void*             eventParam;    

    /*  Module identifier. 
        Allows multiple channels/ports, etc. MAC support. */
    unsigned int            moduleId;

    /*  index of the current interface */
    int                     netIx;

    /*  current action for the MAC/stack */
    TCPIP_MAC_ACTION        macAction;

    /*  The power mode for this interface to go to. 
        Valid only if stackAction == init/reinit. 
        Ignored for deinitialize operation. */
    TCPIP_MAC_POWER_MODE    powerMode;

    /*  Physical address of the interface. 
        MAC sets this field as part of the initialization function. 
        The stack will use this data as the interface address. */
    TCPIP_MAC_ADDR                ifPhyAddress;

    /*  MAC process flags. 
        MAC sets this field as part of the initialization function
        The stack will use this value to call into  
        the MAC process function after receiving a MAC event. */
    TCPIP_MAC_PROCESS_FLAGS processFlags;
    
}TCPIP_MAC_MODULE_CTRL;

// *****************************************************************************
/*  MAC RX Statistics Data

  Summary:
    MAC receive statistics data gathered at run time.

  Description:
    This structure defines the RX statistics data for the
    MAC  driver.

  Remarks:        
    This statistics are recorded by the driver, not by the hardware.

*/

typedef struct
{
    /*  number of OK RX packets */
    int                 nRxOkPackets;

    /*  number of unacknowledged pending RX buffers in the driver queues. */
    /*  If each incoming packet fits within a RX buffer (the RX buffer is large enough) */
    /*  than this corresponds to the number of unacknowledged pending RX packets. */
    /*  Otherwise the number of packets is less than the pending buffers. */
    
    int                 nRxPendBuffers;

    /*  number of RX packets with errors */
    int                 nRxErrorPackets;

    /*  number of RX fragmentation errors */
    int                 nRxFragmentErrors;

}TCPIP_MAC_RX_STATISTICS;

// *****************************************************************************
/*  MAC TX Statistics Data

  Summary:
    MAC transmit statistics data gathered at run time.

  Description:
    This structure defines the TX statistics data for the
    MAC  driver.

  Remarks:        
    This statistics are recorded by the driver, not by the hardware.

*/

typedef struct
{
    /*  number of OK transmitted packets */
    int                 nTxOkPackets;

    /*  number of unacknowledged pending TX buffers in the driver queues. */
    /*  This is equal with pending TX packets when each packet */
    /*  is contained within a TX buffer. */
    int                 nTxPendBuffers;

    /*  number of packets that could not be transmitted */
    int                 nTxErrorPackets;

    /*  number of times the TX queue was full */
    /*  this may signal that the number of TX descriptors is too small */
    int                 nTxQueueFull;

}TCPIP_MAC_TX_STATISTICS;


// *****************************************************************************
/*
  MAC Initialize function.
    TCPIP_MAC_RES       TCPIP_MAC_Initialize(TCPIP_MAC_MODULE_CTRL* const stackData, const void* const moduleData);

  Summary:
    MAC driver initialization function.

  Description:
    This is the function that initializes the MAC.
    It is called by the stack as a result of one interface going up.


  Precondition:
   None.


  Parameters:
    - stackData   - Stack prepared data.
                    MAC uses it for returning its settings: the host MAC address, processing requirements, etc.

    - moduleData  - Driver specific. Dependent on the MAC type.
                    For PIC32 MAC driver, the TCPIP_MODULE_MAC_PIC32INT_CONFIG is used. 


  Returns:
    - TCPIP_MAC_RES_OK if all processing went on OK.
    - a TCPIP_MAC_RES error code if processing failed for some reason.

 Remarks:
    - If this function fails, the stack won't turn up that interface.
    - If the operation needs to wait for the hardware, the initialization
      function can return a pending code.

*/
TCPIP_MAC_RES       TCPIP_MAC_Initialize(TCPIP_MAC_MODULE_CTRL* const stackData, const void* const moduleData);

// *****************************************************************************
/*  
  MAC De-Initialize function.
    TCPIP_MAC_RES       TCPIP_MAC_Deinitialize(const TCPIP_MAC_MODULE_CTRL * const stackData);

  Summary:
    MAC driver deinitialization function.

  Description:
    This is the function that deinitializes the MAC.
    It is called by the stack as a result of one interface going down.

  Precondition:
   TCPIP_MAC_Initialize should have been called.

  Parameters:
    - stackData   - Stack prepared data.
                    It tells the MAC the current action taken by the stack.


  Returns:
    - TCPIP_MAC_RES_OK if all processing went on OK.
    - a TCPIP_MAC_RES error code if processing failed for some reason.

 Remarks:
    None.

*/
TCPIP_MAC_RES       TCPIP_MAC_Deinitialize(const TCPIP_MAC_MODULE_CTRL * const stackData);


// *****************************************************************************
/*  
  MAC Re-Initialize function.
    TCPIP_MAC_RES       TCPIP_MAC_Reinitialize(const TCPIP_MAC_MODULE_CTRL* const stackData, const void* const moduleData);

  Summary:
    MAC driver re-initialization function.

  Description:
    This is the function that re-initializes the MAC.
    It is called by the stack as a result of the system changing power modes.
    Allows re-initialization of the MAC with different power modes, etc.

  Precondition:
   TCPIP_MAC_Initialize should have been called.

  Parameters:
    - stackData   - Stack prepared data.
                    MAC uses it for returning its settings: the host MAC address, processing requirements, etc.

    - moduleData  - Driver specific. Dependent on the MAC type.
                    For PIC32 MAC driver, the TCPIP_MODULE_MAC_PIC32INT_CONFIG is used. 


  Returns:
    - TCPIP_MAC_RES_OK if all processing went on OK.
    - a TCPIP_MAC_RES error code if processing failed for some reason.

 Remarks:
    - This function is optional and is not currently implemented.

*/
TCPIP_MAC_RES       TCPIP_MAC_Reinitialize(const TCPIP_MAC_MODULE_CTRL* const stackData, const void* const moduleData);


// *****************************************************************************
/*  
  MAC Open function.
    TCPIP_MAC_HANDLE    TCPIP_MAC_Open( unsigned int macId );

  Summary:
    MAC driver open function.

  Description:
    This is the function that opens an instance of the MAC.
    Once a MAC client handle is obtained all the MAC functions can be accessed
    using that handle.

  Precondition:
   TCPIP_MAC_Initialize should have been called.

  Parameters:
    - macId   - module identifier.

  Returns:
    - A valid handle if the open function succeeded.
    - 0 otherwise

 Remarks:
    None.

*/
TCPIP_MAC_HANDLE    TCPIP_MAC_Open( unsigned int macId );


// *****************************************************************************
/*  
  MAC Close function.
    TCPIP_MAC_RES       TCPIP_MAC_Close(TCPIP_MAC_HANDLE hMac);

  Summary:
    MAC driver close function.

  Description:
    This is the function that closes an instance of the MAC.
    All per client data is released and the handle can no longer be used
    after this function is called.

  Precondition:
   TCPIP_MAC_Initialize should have been called.
   TCPIP_MAC_Open should have been called to obtain a valid handle.

  Parameters:
    - hMac   - MAC client handle obtained by a call to TCPIP_MAC_Open.

  Returns:
    - TCPIP_MAC_RES_OK if all processing went on OK.
    - a TCPIP_MAC_RES error code if processing failed for some reason.

 Remarks:
    None.

*/
TCPIP_MAC_RES       TCPIP_MAC_Close(TCPIP_MAC_HANDLE hMac);


// *****************************************************************************
/*  
  MAC Packet transmit function.
    TCPIP_MAC_RES       TCPIP_MAC_PacketTx(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PACKET * ptrPacket);

  Summary:
    MAC driver transmit function.

  Description:
    This is the MAC transmit function.
    Using this function a packet is submitted to the MAC driver for transmission.
       
  Precondition:
   TCPIP_MAC_Initialize should have been called.
   TCPIP_MAC_Open should have been called to obtain a valid handle.

  Parameters:
    - hMac      - MAC client handle obtained by a call to TCPIP_MAC_Open.

    - ptrPacket - pointer to a TCPIP_MAC_PACKET that's completely formatted
                  and ready to be transmitted over the network

  Returns:
    - TCPIP_MAC_RES_OK if all processing went on OK.
    - a TCPIP_MAC_RES error code if processing failed for some reason.

 Remarks:
    - A success code returned from this function signals only that the 
      packet was successfully scheduled for transmission over the interface
      and not that the packet was actually transmitted.
      An event will be triggered when the packet is transmitted.
    - The MAC driver has to support internal queuing.
      Since the TCPIP_MAC_PACKET data structure contains internal queuing members
      the MAC can queue the packet at no expense.
      Therefore a packet is to be rejected only if it's not properly formatted.
      Otherwise it has to be scheduled for transmission in an internal MAC queue.
    - Once the packet is scheduled for transmission the MAC driver has to set
      the TCPIP_MAC_PKT_FLAG_QUEUED flag so that the stack is aware that this
      packet is under processing and cannot be modified.
    - Once the packet is transmitted, the TCPIP_MAC_PKT_FLAG_QUEUED has to be
      cleared, the proper packet acknowledgment result (ackRes) has to be
      set and the packet acknowledgment function (ackFunc) has to be called.
      It is implementation dependent if all these steps are implemented as
      part of the ackFunc itself or as discrete steps.
    - On 32 bit machines the 1st segment payload of a packet is allocated so
      that it is always 32 bit aligned and its size is 32 bits multiple. The
      segLoadOffset adds to the payload address and insures that the network
      layer data is 32 bit aligned.
    - PIC32 MAC driver specific : the driver checks that the
      segLoadOffset >= 2.
      See notes for the segLoadOffset member.
    - The packet is not required to contain the Frame Check Sequence
      (FCS/CRC32) field. The MAC driver/controller will insert that field
      itself, if it's required.
    - The MAC driver is required to support the transmission of multiple
      chained packets.                                                       

*/
TCPIP_MAC_RES       TCPIP_MAC_PacketTx(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PACKET * ptrPacket);

// *****************************************************************************
/*  MAC Packet Receive function
     TCPIP_MAC_PACKET*   TCPIP_MAC_PacketRx (TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RES* pRes, const TCPIP_MAC_PACKET_RX_STAT** ppPktStat);

  Summary:
    A packet is returned if such a pending packet exists.

  Description:
    This is the MAC receive function.
    
    Once a pending packet is available in the MAC driver internal RX queues
    this function will dequeue the packet and hand it over to the
    MAC driver's client - i.e., the stack - for further processing.

    The flags for a RX packet have to be updated by the MAC driver:
        - TCPIP_MAC_PKT_FLAG_RX has to be set
          If the MAC supports it, it should set: 
            - TCPIP_MAC_PKT_FLAG_UNICAST has to be set if that packet is a unicast packet
            - TCPIP_MAC_PKT_FLAG_BCAST has to be set if that packet is a broadcast packet
            - TCPIP_MAC_PKT_FLAG_MCAST has to be set if that packet is a multicast packet
        - TCPIP_MAC_PKT_FLAG_QUEUED has to be set
        - TCPIP_MAC_PKT_FLAG_SPLIT has to be set if the packet has multiple data segments

    Additional information about the packet is available by providing the pRes and
    ppPktStat fields.

  Precondition:
   TCPIP_MAC_Initialize should have been called.
   TCPIP_MAC_Open should have been called to obtain a valid handle.

  Parameters:
    - hMac        - handle identifying the MAC driver client

    - pRes        - optional pointer to an address that will receive an additional
                    result associated with the operation.
                    Can be 0 if not needed.

    - ppPktStat   - optional pointer to an address that will receive the received
                    packet status.
                    Note that this pointer cannot be used once the packet
                    acknowledgment function was called.
                    Can be 0 if not needed.

  Returns:
    - a valid pointer to an available RX packet
    - 0 if no packet pending/available

  Remarks:
    - The MAC driver should dequeue and return to the caller just one single packet, 
      and not multiple chained packets!
    - Once the higher level layers in the stack are done with processing the RX packet,
      they have to call the corresponding packet acknowledgment function
      that tells the owner of that packet that it can resume control of that packet.
    - Once the stack modules are done processing the RX packets and the acknowledge function is called
      it is up to the driver design to reuse the RX packets, or simply return them to the pool
      they were allocated from (assuming that some sort of allocation is implemented).
      This document makes no requirement about how the MAC RX packets are obtained,
      using dynamic or static allocation techniques.
      This is up to the design of the MAC.
    - The MAC driver can use the TCPIP_MAC_Process() for obtaining new RX packets if needed.
    - Not all the MACs have hardware support for the received packet status.
      If the MAC driver cannot supply the TCPIP_MAC_PACKET_RX_STAT info,
      it should set the ppPktStat to 0.

*/
TCPIP_MAC_PACKET*   TCPIP_MAC_PacketRx (TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RES* pRes, const TCPIP_MAC_PACKET_RX_STAT** ppPktStat);

// *****************************************************************************
/*  MAC Process function
     TCPIP_MAC_RES       TCPIP_MAC_Process(TCPIP_MAC_HANDLE hMac);

  Summary:
    MAC periodic processing function.

  Description:
    This is a function that allows for internal processing by the MAC driver.
    It is meant for processing that cannot be done from within ISR.

    Normally this function will be called in response to an TX and/or RX event
    signaled by the driver.
    This is specified by the MAC driver at initialization time using the
    TCPIP_MAC_MODULE_CTRL::processFlags.

    An alternative approach is that the MAC driver uses a system service
    to create a timer signal that will call the TCPIP_MAC_Process on a periodic basis.

  Precondition:
   TCPIP_MAC_Initialize should have been called.
   TCPIP_MAC_Open should have been called to obtain a valid handle.

  Parameters:
    - hMac        - handle identifying the MAC driver client

  Returns:
    - TCPIP_MAC_RES_OK if all processing went on OK
    - a TCPIP_MAC_RES error code if processing failed for some reason

  Remarks:
    - Some of the processing that this function is intended for:
        - the MAC driver can process its pending TX queues
          (although it could do that from within the TX ISR)
        - RX buffers replenishing. If the number of packets in the RX queue
          falls below a specified limit, the MAC driver can use this function
          to allocate some extra RX packets.
          Similarly, if there are too many allocated RX packets, the MAC driver
          can free some of them.

*/
TCPIP_MAC_RES       TCPIP_MAC_Process(TCPIP_MAC_HANDLE hMac);


// *****************************************************************************
/*  MAC Link check function
     bool TCPIP_MAC_LinkCheck(TCPIP_MAC_HANDLE hMac);

  Summary:
    MAC link checking function.

  Description:
    This is a function that returns the current status of the link.

  Precondition:
   TCPIP_MAC_Initialize should have been called.
   TCPIP_MAC_Open should have been called to obtain a valid handle.

  Parameters:
    - hMac        - handle identifying the MAC driver client

  Returns:
    - true  - if the link is up
    - false - if the link is down


  Remarks:
    - The stack will call this function periodically.
      Therefore this function is not required to initiate a new PHY transaction
      and to wait for the result.
      It can just initiate a new PHY transaction and return immediately the result of the
      previous transaction. 

*/
bool                TCPIP_MAC_LinkCheck(TCPIP_MAC_HANDLE hMac);


// *****************************************************************************
/*  MAC Event Mask Set function
     bool TCPIP_MAC_EventMaskSet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents, bool enable);

  Summary:
    MAC events report enable/disable function.

  Description:
    This is a function that enables or disables the events to be reported
    to the MAC client (TCP/IP stack).

     All events that are to be enabled will be added to the notification process.
     All events that are to be disabled will be removed from the notification process.
     The stack (or stack user) has to catch the events that are notified and process them.
     After that the stack should call TCPIP_MAC_EventAcknowledge() so that the events can be re-enabled.

  Precondition:
   TCPIP_MAC_Initialize should have been called.
   TCPIP_MAC_Open should have been called to obtain a valid handle.

  Parameters:
    - hMac        - handle identifying the MAC driver client
    - macEvents   - events that the MAC client wants to add/delete for notification
    - enable      - if true, the events will  be enabled, else disabled

  Returns:
    - true  - if operation succeeded
    - false - if some error occurred and the operation failed

  Example:
    <code>
    TCPIP_MAC_EventMaskSet( hMac, TCPIP_MAC_EV_RX_OVFLOW | TCPIP_MAC_EV_RX_BUFNA, true );
    </code>

  Remarks:
    - Multiple events can be "ORed" together.
    - The event notification system enables the user of the MAC and of the stack to call
      into the stack for processing only when there are relevant events rather than
      being forced to periodically call from within a loop.
    - If the notification events are null the interrupt processing will be disabled.
      Otherwise the event notification will be enabled and the interrupts relating
      to the requested events will be enabled.
    - Note that once an event has been caught by the MAC and reported through the notification handler
      it may be disabled until the TCPIP_MAC_EventAcknowledge is called.

*/
bool                    TCPIP_MAC_EventMaskSet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents, bool enable);

// *****************************************************************************
/*  MAC Event Acknowledge function
     bool TCPIP_MAC_EventAcknowledge(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents);

  Summary:
     This function acknowledges a previously reported MAC event.

  Description:
    This function acknowledges and re-enables processed events.
    Multiple events can be "ORed" together as they are processed together.
    The events acknowledged by this function should be the events that have
    been retrieved from the MAC by calling TCPIP_MAC_EventPendingGet or have
    been passed to the MAC client by using the notification handler.
    Once the events are acknowledged they will be re-enabled.

  Precondition:
   TCPIP_MAC_Initialize should have been called.
   TCPIP_MAC_Open should have been called to obtain a valid handle.

  Parameters:
    - hMac        - handle identifying the MAC driver client
    - macEvents   - MAC events that are acknowledged and re-enabled

  Returns:
    - true  - if events acknowledged
    - false - if no events to be acknowledged

  Example:
    <code>
    TCPIP_MAC_EventAcknowledge( hMac, newMacEvents );
    </code>

  Remarks:
    - All events should be acknowledged, in order to be re-enabled.
    - Some events are fatal errors and should not be acknowledged
      (TCPIP_MAC_EV_RX_BUSERR, TCPIP_MAC_EV_TX_BUSERR).
      Stack re-initialization is needed under such circumstances.
    - Some events are just system/application behavior and they are intended only as simple info
      (TCPIP_MAC_EV_RX_OVFLOW, TCPIP_MAC_EV_RX_BUFNA, TCPIP_MAC_EV_TX_ABORT, TCPIP_MAC_EV_RX_ACT).
    - The TCPIP_MAC_EV_RX_FWMARK and TCPIP_MAC_EV_RX_EWMARK events are part of the normal flow control operation
      (if auto flow control was enabled).
      They should be enabled alternatively, if needed.
    - The events are persistent. They shouldn't be re-enabled unless they have been processed
      and the condition that generated them was removed.
      Re-enabling them immediately without proper processing will have dramatic
      effects on the system performance.

*/
bool                    TCPIP_MAC_EventAcknowledge(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents);

// *****************************************************************************
/*  MAC Event Pending function
     TCPIP_MAC_EVENT TCPIP_MAC_EventPendingGet(TCPIP_MAC_HANDLE hMac);

  Summary:
    Returns the currently pending MAC events.

  Description:
    This function returns the currently MAC pending events.
    Multiple events will be "ORed" together as they accumulate.
    MAC processing is needed whenever a transmission related event is present
    (TCPIP_MAC_EV_RX_PKTPEND, TCPIP_MAC_EV_TX_DONE).
    Other, non critical events, may be passed to an user for informational purposes.
    All events have to be eventually acknowledged if re-enabling is needed.


  Precondition:
   TCPIP_MAC_Initialize should have been called.
   TCPIP_MAC_Open should have been called to obtain a valid handle.

  Parameters:
    - hMac        - handle identifying the MAC driver client

  Returns:
    The currently stack pending events.

  Example:
    <code>
    TCPIP_MAC_EVENT currEvents = TCPIP_MAC_EventPendingGet( hMac);
    </code>

  Remarks:
    - This is the preferred method to get the current pending MAC events.
      Even with a notification handler in place it's better to use this function
      to get the current pending events rather than using the events passed by the
      notification handler which could be stale.
    - The returned value is just a momentary value. The pending events can change any time.

*/
TCPIP_MAC_EVENT         TCPIP_MAC_EventPendingGet(TCPIP_MAC_HANDLE hMac);




// *****************************************************************************
/*  MAC Hash Receive Filter function
     TCPIP_MAC_RES TCPIP_MAC_RxFilterHashTableEntrySet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_ADDR* DestMACAddr);

  Summary:
    Sets the current MAC hash table receive filter.

  Description:
    This function sets the MAC hash table filtering to allow 
    packets sent to DestMACAddr to be received.
    It calculates a CRC-32 using polynomial 0x4C11DB7 over the 6 byte MAC address
    and then, using bits 28:23 of the CRC, will set the appropriate bits in
    the hash table filter registers.

    The function will enable/disable the Hash Table receive filter if needed.

  Precondition:
   TCPIP_MAC_Initialize should have been called.
   TCPIP_MAC_Open should have been called to obtain a valid handle.

  Parameters:
    - hMac        - handle identifying the MAC driver client

    - DestMACAddr - destination MAC address (6 bytes) to allow 
                    through the Hash Table Filter.
                    If DestMACAddr is set to 00-00-00-00-00-00,
                    then the hash table will be cleared of all entries
                    and the filter will be disabled.

  Returns:
    - TCPIP_MAC_RES_OK if all processing went on OK.
    - a TCPIP_MAC_RES error code if function failed for some reason.

  Example:
    <code>
    </code>

  Remarks:
    - There is no way to individually remove destination MAC 
      addresses from the hash table since it is possible to have 
      a hash collision and therefore multiple MAC addresses 
      relying on the same hash table bit.
    - A workaround is to have the stack store each enabled MAC address
      and to perform the comparison at run time.
    - A call to TCPIP_MAC_RxFilterHashTableEntrySet() using a 
      00-00-00-00-00-00 destination MAC address, which will clear 
      the entire hash table and disable the hash table filter.  
      This will allow the receive of all packets, regardless of their destination.

*/
TCPIP_MAC_RES        TCPIP_MAC_RxFilterHashTableEntrySet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_ADDR* DestMACAddr);

// *****************************************************************************
/*  Function:
     TCPIP_MAC_RES TCPIP_MAC_StatisticsGet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RX_STATISTICS* pRxStatistics, TCPIP_MAC_TX_STATISTICS* pTxStatistics);

  Summary:
    Gets the current MAC statistics.

  Description:
    This function will get the current value of the statistic counters
    maintained by the MAC driver.


  Precondition:
   TCPIP_MAC_Initialize should have been called.
   TCPIP_MAC_Open should have been called to obtain a valid handle.

  Parameters:
    - hMac          - handle identifying the MAC driver client

    - pRxStatistics - pointer to a TCPIP_MAC_RX_STATISTICS that will receive the current 
                      RX statistics counters
                      Can be NULL if not needed

    - pTxStatistics - pointer to a TCPIP_MAC_TX_STATISTICS that will receive the current 
                      TX statistics counters
                      Can be NULL if not needed

  Returns:
    - TCPIP_MAC_RES_OK if all processing went on OK.
    - TCPIP_MAC_RES_OP_ERR error code if function not supported by the driver.

  Example:
    <code>
    </code>

  Remarks:
    - The reported values are info only and change dynamically.

*/
TCPIP_MAC_RES        TCPIP_MAC_StatisticsGet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RX_STATISTICS* pRxStatistics, TCPIP_MAC_TX_STATISTICS* pTxStatistics);
    

#endif // __TCPIP_MAC_H_

