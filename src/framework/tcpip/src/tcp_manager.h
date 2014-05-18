/*******************************************************************************
  TCP Manager internal stack API

  Company:
    Microchip Technology Inc.
    
  File Name:
    tcp_manager.h

  Summary:
    
  Description:
*******************************************************************************/
// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END

#ifndef __TCP_MANAGER_H_
#define __TCP_MANAGER_H_


/****************************************************************************
  Section:
    Type Definitions
  ***************************************************************************/


/****************************************************************************
  Section:
    Function Declarations
  ***************************************************************************/

/*****************************************************************************
  Function:
    bool TCPIP_TCP_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackInit, TCP_MODULE_CONFIG* pTcpInit)

  Summary:
    Initializes the TCP module.

  Description:
    Initializes the TCP module.  This function sets up the TCP buffers
    in memory and initializes each socket to the CLOSED state.  If
    insufficient memory was allocated for the TCP sockets, the function
    will call the TCPIPError() error function that can be captured by the debugger
    and will return false.

  Precondition:
    None

  Parameters:
    stackInit   - pointer to stack initialization data; contains heap, interfaces, etc

    pTcpInit    - pointer to a TCP initialization structure containing:
                    - nSockets:         number of sockets to be created
                    - sktTxBuffSize:    default TX buffer size
                    - sktRxBuffSize:    default RX buffer size
  Returns:
    true if initialization succeeded
    false otherwise

  Remarks:
   None
 */
bool TCPIP_TCP_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackInit, const TCP_MODULE_CONFIG* pTcpInit);


/*****************************************************************************
  Function:
    void TCPIP_TCP_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackInit)

  Summary:
    De-Initializes the TCP module.

  Description:
    De-Initializes the TCP module.
    This function initializes each socket to the CLOSED state.
    If dynamic memory was allocated for the TCP sockets, the function
    will deallocate it.

  Precondition:
    TCPIP_TCP_Initialize() should have been called

  Parameters:
    stackInit   - pointer to stack initialization data; contains heap, interfaces, etc
                  and interface that's going down

  Returns:
    None

  Remarks:
 */
void TCPIP_TCP_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackInit);


/*****************************************************************************
  Function:
    bool TCPIP_TCP_ProcessIPv4(TCPIP_NET_IF* pPktIf, NODE_INFO* remote, IPV4_ADDR* localIP, uint16_t len)

  Summary:
    Handles incoming TCP segments.

  Description:
    This function handles incoming TCP segments.  When a segment arrives, it
    is compared to open sockets using a hash of the remote port and IP.
    On a match, the data is passed to _TcpHandleSeg for further processing.

  Precondition:
    TCP is initialized and a TCP segment is ready in the MAC buffer.

  Parameters:
    pPktIf -  interface that has packets
    remote - Remote NODE_INFO structure
    localIP - This stack's IP address (for header checking)
    len - Total length of the waiting TCP segment

  Return Values:
    true - the segment was properly handled.
    false - otherwise
 */
void TCPIP_TCP_ProcessIPv4(TCPIP_NET_IF* pPktIf, TCPIP_MAC_PACKET* pRxPkt);

void TCPIP_TCP_ProcessIPv6(TCPIP_MAC_PACKET* pRxPkt, IPV6_ADDR * localIP, IPV6_ADDR * remoteIP, uint16_t dataLen, uint16_t headerLen);


/*****************************************************************************
  Function:
    void TCPIP_TCP_Tick(void)

  Summary:
    Performs periodic TCP tasks.

  Description:
    This function performs any required periodic TCP tasks.  Each
    socket's state machine is checked, and any elapsed timeout periods
    are handled.

  Precondition:
    TCP is initialized.

  Parameters:
    None

  Returns:
    None
 */
void TCPIP_TCP_Tick(void);

bool TCPIP_TCP_TaskIsPending(void);

bool TCPIP_TCP_SourceIPAddressSet(TCP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* localAddress);

bool TCPIP_TCP_DestinationIPAddressSet(TCP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* remoteAddress);

int     TCPIP_TCP_SocketsNumberGet(void);



// SSL related
void        TCPIP_TCPSSL_MessageTransmit(TCP_SOCKET hTCP);

uint8_t     TCPIP_TCP_SocketSSLIdentifierGet(TCP_SOCKET hTCP);


/*****************************************************************************
  Function:
    void TCPIP_TCPSSL_RecordHeaderPut(TCP_SOCKET hTCP, uint8_t* hdr, bool recDone)

  Summary:
    Writes an SSL record header and sends an SSL record.

  Description:
    This function writes an SSL record header to the pending TCP SSL data,
    then indicates that the data is ready to be sent by moving the txHead
    pointer.

    If the record is complete, set recDone to true.  The sslTxHead
    pointer will be moved forward 5 bytes to leave space for a future
    record header.  If the record is only partially sent, use false and
    to leave the pointer where it is so that more data can be added
    to the record.  Partial records can only be used for the
    SERVER_CERTIFICATE handshake message.

  Precondition:
    TCP is initialized, and hTCP is connected with an active SSL session.

  Parameters:
    hTCP        - TCP connection to write the header and transmit with
    hdr         - Record header (5 bytes) to send or NULL to just
                  move the pointerctx
    recDone     - true if the record is done, false otherwise

  Returns:
    None

  Remarks:
    This function should never be called by an application.  It is used
    only by the SSL module itself.
  ***************************************************************************/
void        TCPIP_TCPSSL_RecordHeaderPut(TCP_SOCKET hTCP, uint8_t* hdr, bool recDone);


/*****************************************************************************
  Function:
    uint16_t TCPIP_TCPSSL_PendingTxSizeGet(TCP_SOCKET hTCP)

  Summary:
    Determines how many bytes are pending for a future SSL record.

  Description:
    This function determines how many bytes are pending for a future SSL
    record.

  Precondition:
    TCP is initialized, and hTCP is connected with an active SSL connection.

  Parameters:
    hTCP        - TCP connection to check

  Returns:
    None
  ***************************************************************************/
uint16_t    TCPIP_TCPSSL_PendingTxSizeGet(TCP_SOCKET hTCP);


/*****************************************************************************
  Function:
    void TCPIP_TCPSSL_HandIncomingToSSL(TCP_SOCKET hTCP)

  Summary:
    Hands newly arrive TCP data to the SSL module for processing.

  Description:
    This function processes incoming TCP data as an SSL record and
    performs any necessary repositioning and decrypting.

  Precondition:
    TCP is initialized, and hTCP is connected with an active SSL session.

  Parameters:
    hTCP        - TCP connection to handle incoming data on

  Returns:
    None

  Remarks:
    This function should never be called by an application.  It is used
    only by the SSL module itself.
  ***************************************************************************/
void        TCPIP_TCPSSL_HandIncomingToSSL(TCP_SOCKET hTCP);


/*****************************************************************************
  Function:
    bool TCPIP_TCPSSL_MessageRequest(TCP_SOCKET hTCP, uint8_t msg)

  Summary:
    Requests an SSL message to be transmitted.

  Description:
    This function is called to request that a specific SSL message be
    transmitted.  This message should only be called by the SSL module.

  Precondition:
    TCP is initialized.

  Parameters:
    hTCP        - TCP connection to use
    msg         - One of the SSL_MESSAGE types to transmit.

  Return Values:
    true        - The message was requested.
    false       - Another message is already pending transmission.
 */
bool        TCPIP_TCPSSL_MessageRequest(TCP_SOCKET hTCP, uint8_t msg);


/*****************************************************************************
  Function:
    void TCPIP_TCPSSL_DecryptPlusMACCalc(TCP_SOCKET hTCP, void * ctx, uint16_t len)

  Summary:
    Decrypts and MACs data arriving via SSL.

  Description:
    This function decrypts data in the TCP buffer and calculates the MAC over
    the data.  All data is left in the exact same location in the TCP buffer.
    It is called to help process incoming SSL records.

  Precondition:
    TCP is initialized, hTCP is connected, and ctx's Sbox is loaded.

  Parameters:
    hTCP        - TCP connection to decrypt in
    ctx         - context to use
    len         - Number of bytes to crypt
    inPlace     - true to write back in place, false to write at end of
                    currently visible data.

  Returns:
    None

  Remarks:
    This function should never be called by an application.  It is used
    only by the SSL module itself.
  ***************************************************************************/
void        TCPIP_TCPSSL_DecryptPlusMACCalc(TCP_SOCKET hTCP, ARCFOUR_CTX* ctx, uint16_t len);


/*****************************************************************************
  Function:
    void TCPIP_TCPSSL_EncryptPlusMACInPlace(TCP_SOCKET hTCP, void* ctx, uint8_t* MACSecret, uint16_t len)

  Summary:
    Encrypts and MACs data in place in the TCP TX buffer.

  Description:
    This function encrypts data in the TCP buffer while calculating a MAC.
    When encryption is finished, the MAC is appended to the buffer and
    the record will be ready to transmit.

  Precondition:
    TCP is initialized, hTCP is connected, and ctx's Sbox is loaded.

  Parameters:
    hTCP        - TCP connection to encrypt in
    ctx         - context to use
    MACSecret   - MAC encryption secret to use
    len         - Number of bytes to crypt

  Returns:
    None

  Remarks:
    This function should never be called by an application.  It is used
    only by the SSL module itself.
  ***************************************************************************/
void        TCPIP_TCPSSL_EncryptPlusMACInPlace(TCP_SOCKET hTCP, ARCFOUR_CTX* ctx, uint8_t* MACSecret, uint16_t len);

// debug/trace support
//

//#define TCPIP_TCP_DEBUG

typedef struct
{
    uint16_t    addType;        // IPv4/IPv6/unknown socket type
    uint16_t    remotePort;     // port no
    uint16_t    localPort;      // port no
    uint16_t    rxSize;         // RX buffer size
    uint16_t    txSize;         // TX buffer size
}TCPIP_TCP_SKT_DEBUG_INFO;

// number of TCP sockets
int     TCPIP_TCP_DebugSktNo(void);

// fills in a debug info structure for the specified socket
bool    TCPIP_TCP_DebugSktInfo(int sktNo, TCPIP_TCP_SKT_DEBUG_INFO* pInfo);



#endif  // __TCP_MANAGER_H_
