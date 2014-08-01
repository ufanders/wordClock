/*******************************************************************************
  UDP Module Definitions for the Microchip TCP/IP Stack

  Company:
    Microchip Technology Inc.
    
  File Name:
    udp.h

  Summary:
    UDP is a standard transport layer protocol described in RFC 768. It provides 
    fast but unreliable data-gram based transfers over networks, and forms the 
    foundation SNTP, SNMP, DNS, and many other protocol standards
    
  Description:
    UDP is a standard transport layer protocol described in RFC 768.
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

#ifndef __UDP_H_
#define __UDP_H_


// Defines a UDP Port Number
typedef uint16_t UDP_PORT;

// Provides a handle to a UDP Socket
typedef int16_t UDP_SOCKET;


#define INVALID_UDP_SOCKET      (-1)		// Indicates a UDP socket that is not valid

// Information about a socket
typedef struct
{
    IP_ADDRESS_TYPE     addressType;
    IP_MULTI_ADDRESS    remoteIPaddress;
    IP_MULTI_ADDRESS    localIPaddress;
	UDP_PORT            remotePort;	// Port number associated with remote node
    UDP_PORT            localPort;  // local port number
} UDP_SOCKET_INFO;


// UDP socket options
typedef enum
{
    UDP_OPTION_STRICT_PORT,         // When connection is done the socket stores the remote host local port.
                                    // If option is enabled the remote host local port is always checked to match the initial one.
                                    // If disabled the remote host local port is not checked. 
                                    // Disabled by default on a socket server. Enabled by default on a client socket.
    UDP_OPTION_STRICT_NET,          // When connection is done the socket stores the network interface the connection occurred on.
                                    // If option is enabled the socket accepts data only from the interface that matches the initial connection.
                                    // If disabled the socket receives data from any remote host regardless of the 
                                    // interface on which the packet arrived.
                                    // Disabled by default on a socket server. Enabled by default on a client socket.
    UDP_OPTION_BROADCAST,           // Enables the Broadcast transmission by the socket
    UDP_OPTION_BUFFER_POOL,         // Enables the socket to use the private UDP buffers pool.
                                    // The size of the TX buffer has to be less than the size of the buffers in the pool.
    UDP_OPTION_TX_BUFF,             // Request different TX buffer size. Has to call TCPIP_UDP_OptionsGet to see the exact space
                                    // allocated.
    UDP_OPTION_TX_QUEUE_LIMIT,      // Sets the maximum number of packets that can be queued/allocated by an UDP socket
                                    // at a certain time.
                                    // For sockets that need to transmit a lot of data (Iperf client, for example),
                                    // especially on slow connections this limit prevents running out of memory
                                    // because the MAC transfer cannot keep up with the UDP packet allocation rate
                                    // imposed by the application.
                                    // Adjust depending on the size of the UDP TX buffer, the connection speed
                                    // and the amount of memory available to the stack.
                                    


}UDP_SOCKET_OPTION;


typedef enum
{
    UDP_BCAST_NONE,                    // no broadcast
    UDP_BCAST_NETWORK_LIMITED,         // network limited broadcast
    UDP_BCAST_NETWORK_DIRECTED,        // network directed broadcast

}UDP_SOCKET_BCAST_TYPE;


// *****************************************************************************
/*
  Structure:
    UDP_MODULE_CONFIG

  Summary:
    UDP module run time configuration/initialization data.

  Description:
    UDP module configuration/initialization
*/
//
typedef struct
{
    uint16_t        nSockets;       // number of sockets to be created
    uint16_t        sktTxBuffSize;  // default size of the socket TX buffer
    uint16_t        poolBuffers;    // number of buffers in the pool; 0 if none
    uint16_t        poolBufferSize; // size of the buffers in the pool; all equal    
}UDP_MODULE_CONFIG;


// *****************************************************************************
// *****************************************************************************
// Section: Function Prototypes
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************

/*
  Function:
	 UDP_SOCKET TCPIP_UDP_ServerOpen(IP_ADDRESS_TYPE addType, UDP_PORT localPort,  IP_MULTI_ADDRESS* localAddress)

  Summary:
	Opens a UDP socket as a server.
	
  Description:
	Provides a unified method for opening UDP server sockets. 

  Precondition:
    UDP is initialized.

  Parameters:
    IP_ADDRESS_TYPE addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4 or IP_ADDRESS_TYPE_IPV6.
    
    UDP_PORT localPort				-	UDP port on which to listen for connections
    
    IP_MULTI_ADDRESS* localAddress	-	Local address to use.
                                        Can be NULL if any incoming interface will do.
	
  Returns:
 	INVALID_SOCKET -  No sockets of the specified type were available to be
                      opened.

    Otherwise -       A UDP_SOCKET handle. Save this handle and use it when
                      calling all other UDP APIs. 
 */
UDP_SOCKET          TCPIP_UDP_ServerOpen(IP_ADDRESS_TYPE addType, UDP_PORT localPort,  IP_MULTI_ADDRESS* localAddress);

// *****************************************************************************

/*
  Function:
	 TCPIP_UDP_ClientOpen(IP_ADDRESS_TYPE addType, UDP_PORT remotePort, IP_MULTI_ADDRESS* remoteAddress)

  Summary:
	Opens a UDP socket as a client.
	
  Description:
	Provides a unified method for opening UDP client sockets. 

  Precondition:
    UDP is initialized.

  Parameters:
    IP_ADDRESS_TYPE addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4 or IP_ADDRESS_TYPE_IPV6.

    UDP_PORT remotePort				-   The remote UDP port to which a connection should be made.
                                        The local port for client sockets will be automatically picked
                                        by the UDP module.
                                        
    IP_MULTI_ADDRESS* remoteAddress	-	The remote address to connect to.
	
  Returns:
 	INVALID_SOCKET -  No sockets of the specified type were available to be
                      opened.

    Otherwise -       A UDP_SOCKET handle. Save this handle and use it when
                      calling all other UDP APIs. 

 Remarks:
    IP_ADDRESS_TYPE_ANY is not supported!
 */
UDP_SOCKET          TCPIP_UDP_ClientOpen(IP_ADDRESS_TYPE addType, UDP_PORT remotePort, IP_MULTI_ADDRESS* remoteAddress);

// *****************************************************************************

/*
  Function:
    bool TCPIP_UDP_Bind(UDP_SOCKET hUDP, IP_ADDRESS_TYPE addType, UDP_PORT localPort,  IP_MULTI_ADDRESS* localAddress);

  Summary:
    Bind a socket to a local address and port.
    This function is meant for client sockets.
    It assigns a specific source address and port for a socket.

  Description:
    Sockets don't need specific binding, it is done automatically by the stack
    However, specific binding can be requested using these functions.
    Works for both client and server sockets.

  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket

  Parameters:
	hUDP			-	The socket to bind.
	addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4.
	localPort		-	The local port to bind to.
	localAddress	-   Local address to use.
	
  Returns:
	true if success
	false otherwise

  Remarks:
    If localAddress == 0 the local address of the socket won't be changed.

    If localPort is 0 the stack will assign a unique local port

  */
bool                TCPIP_UDP_Bind(UDP_SOCKET hUDP, IP_ADDRESS_TYPE addType, UDP_PORT localPort,  IP_MULTI_ADDRESS* localAddress);

// *****************************************************************************

/*
  Function:
    bool TCPIP_UDP_RemoteBind(UDP_SOCKET hUDP, IP_ADDRESS_TYPE addType, UDP_PORT remotePort,  IP_MULTI_ADDRESS* remoteAddress);

  Summary:
    Bind a socket to a remote address
    This function is meant for server sockets.

  Description:
    Sockets don't need specific remote binding, they should accept connections on any incoming interface.
    Thus the binding is done automatically by the stack.
    However, specific binding can be requested using these functions.
    For a server socket it can be used to restrict accepting connections from  a specific remote host.
    For a client socket it will just change the default binding done when the socket was opened.

  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket

  Parameters:
	hUDP			-	The socket to bind.
	addType			-	The type of address being used. Example: IP_ADDRESS_TYPE_IPV4.
	remotePort		-	The remote port to bind to.
	remoteAddress	-   Remote address to use.
	
  Returns:
	true if success
	false otherwise

  Remarks:
    If remoteAddress == 0 the remote address of the socket won't be changed.
    The remote port is always changed, even if remotePort == 0.

    The call should fail if the socket is already bound to an interface
    (a server socket is connected or a client socket already sent the data on an interface).
    Implementation pending.

    
  */
bool                TCPIP_UDP_RemoteBind(UDP_SOCKET hUDP, IP_ADDRESS_TYPE addType, UDP_PORT remotePort,  IP_MULTI_ADDRESS* remoteAddress);


// *****************************************************************************
/*
  Function:
	bool TCPIP_UDP_OptionsSet(UDP_SOCKET hUDP, UDP_SOCKET_OPTION option, void* optParam);

  Summary:
    Allows setting options to a socket like adjust Rx/Tx buffer size, etc

  Description:
    Various options can be set at the socket level.
    This function provides compatibility with BSD implementations.

  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket


  Parameters:
    hUDP		    - socket to set options for	
	option			- specific option to be set	
	optParam		- the option value; this is option dependent	
                        UDP_OPTION_STRICT_PORT  -- boolean enable/disable
                        UDP_OPTION_STRICT_NET   -- boolean enable/disable
                        UDP_OPTION_BROADCAST    -- UDP_SOCKET_BCAST_TYPE
                        UDP_OPTION_BUFFER_POOL  -- boolean enable/disable
                                                   Note: changing the UDP_OPTION_BUFFER_POOL will discard
                                                         the data in the current socket buffer 
                        UDP_OPTION_TX_BUFF      -- 16 bit value in bytes of the TX buffer
                                                   Note: the UDP_OPTION_TX_BUFF will discard
                                                         the data in the current socket buffer 
                        UDP_OPTION_TX_QUEUE_LIMIT -- 8 bit value of the TX queue limit

  Returns:
 	true if success
    false otherwise
  */	
bool                TCPIP_UDP_OptionsSet(UDP_SOCKET hUDP, UDP_SOCKET_OPTION option, void* optParam);


// *****************************************************************************
/*
  Function:
	bool TCPIP_UDP_OptionsGet(UDP_SOCKET hUDP, UDP_SOCKET_OPTION option, void* optParam);

  Summary:
    Allows getting the options for a socket like: current Rx/Tx buffer size, etc

  Description:
    Various options can be retrieved at the socket level.
    This function provides compatibility with BSD implementations.

  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket


  Parameters:
    hUDP		    - socket to get options for	
	option			- specific option to get	
	optParam		- pointer to an area that will receive the option value; this is option dependent
                      the size of the area has to be large enough to accommodate the specific option    
                        UDP_OPTION_STRICT_PORT  -- pointer to boolean
                        UDP_OPTION_STRICT_NET   -- pointer to boolean
                        UDP_OPTION_BROADCAST    -- UDP_SOCKET_BCAST_TYPE
                        UDP_OPTION_BUFFER_POOL  -- pointer to boolean
                        UDP_OPTION_TX_BUFF      -- pointer to a 16 bit value to receive bytes of the TX buffer
                        UDP_OPTION_TX_QUEUE_LIMIT -- pointer to an 8 bit value to receive the TX queue limit

  Returns:
 	true if success
    false otherwise
  */	
bool                TCPIP_UDP_OptionsGet(UDP_SOCKET hUDP, UDP_SOCKET_OPTION option, void* optParam);


// *****************************************************************************

/*
  Function:
	  bool TCPIP_UDP_IsConnected(UDP_SOCKET hUDP)
  
 Summary:
	  Determines if a socket has an established connection.

 Description:
	This function determines if a socket has an established connection to a remote node.  
	Call this function after calling TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()
    to determine when the connection is set up and ready for use.  

 Precondition:
    None

 Parameters:
	hUDP - The socket to check.

 Return Values:
	true - The socket has been opened.
	false - The socket is not currently opened.

 Remarks:
	Since an UDP server or client socket can always send data,
    regardless if there's another remote socket connected,
    this function will return true if the socket is opened.
 */
bool                TCPIP_UDP_IsConnected(UDP_SOCKET hUDP);

// *****************************************************************************

/*
  Function:
	  bool TCPIP_UDP_IsOpened(UDP_SOCKET hUDP)
  
 Summary:
	  Determines if a socket was opened.

 Description:
	This function determines if a socket was opened.  

 Precondition:
    None

 Parameters:
	hUDP - The socket to check.

 Return Values:
	true - The socket has been opened.
	false - The socket is not currently opened.

 Remarks:
	This is a backward compatibility call.

 */
#define             TCPIP_UDP_IsOpened(hUDP)      TCPIP_UDP_IsConnected(hUDP)       


// *****************************************************************************

/*
  Function:
	void TCPIP_UDP_Close(UDP_SOCKET hUDP)

  Summary:
	Closes a UDP socket and frees the handle.
	
  Description:
	Closes a UDP socket and frees the handle.  Call this function to release
	a socket and return it to the pool for use by future communications.

  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket

  Parameters:
	hUDP - The socket handle to be released.

  Returns:
  	None
  	
  Remarks:
    Always close the socket when no longer in use.
    This will free the allocated resources, including the TX buffers.

  */
void                TCPIP_UDP_Close(UDP_SOCKET hUDP);

// *****************************************************************************

/*
  Function:
	bool TCPIP_UDP_SocketInfoGet(UDP_SOCKET hUDP, UDP_SOCKET_INFO* pInfo)

  Summary:
	Returns information about a selected UDP socket.
	
  Description:
    This function will fill a user passed UDP_SOCKET_INFO structure
    with status of the selected socket.
    

  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket
    pInfo - valid address of a UDP_SOCKET_INFO structure

  Parameters:
  	UDP_SOCKET hUDP - Socket to obtain info for.
	UDP_SOCKET_INFO* pInfo - pointer to UDP_SOCKET_INFO to receive socket info

  Returns:
    true - if the call succeeded
    false - if no such socket or invalid pInfo

  */
bool                TCPIP_UDP_SocketInfoGet(UDP_SOCKET hUDP, UDP_SOCKET_INFO* pInfo);

// *****************************************************************************

/*
  Function:
	bool TCPIP_UDP_TxOffsetSet(UDP_SOCKET hUDP, uint16_t wOffset, bool relative)

  Summary:
	Moves the pointer within the TX buffer.
	
  Description:
	This function allows the write location within the TX buffer to be 
	specified. Future calls to TCPIP_UDP_Put, TCPIP_UDP_ArrayPut, TCPIP_UDP_StringPut, etc
    will write data from the indicated location.

  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket

  Parameters:
    hUDP    - UDP socket handle 
	wOffset - Offset in the UDP packet data payload to move the write pointer.
    relative- if true, the wOffset is added to the current write pointer.
              else the wOffset is from the beginning of the UDP buffer

  Returns:
  	true if the offset was a valid one and the write pointer has been moved
    false otherwise
  */
bool                TCPIP_UDP_TxOffsetSet(UDP_SOCKET hUDP, uint16_t wOffset, bool relative);

// *****************************************************************************

/*
  Function:
	uint16_t TCPIP_UDP_TxPutIsReady(UDP_SOCKET hUDP, unsigned short count)

  Summary:
	Determines how many bytes can be written to the UDP socket.
	
  Description:
	This function returns the number of bytes that can be written to the specified UDP
	socket.
  
  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket

  Parameters:
	hUDP  - UDP socket handle
    count - Number of bytes requested

  Returns:
  	The number of bytes that can be written to this socket.

  Remarks:
    The function won't increase the size of the UDP TX buffer.
    If this is needed use TCPIP_UDP_OptionsSet().
    The count variable is not used.

    The function is similar to the TCPIP_UDP_PutIsReady and maintained for backward compatibility.

  */
uint16_t            TCPIP_UDP_TxPutIsReady(UDP_SOCKET hUDP, unsigned short count);

// *****************************************************************************

/*
  Function:
	uint16_t TCPIP_UDP_PutIsReady(UDP_SOCKET hUDP)

  Summary:
	Determines how many bytes can be written to the UDP socket.
	
  Description:
	This function determines how many bytes can be written to the specified UDP
	socket.
  
  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket

  Parameters:
	hUDP  - UDP socket handle

  Returns:
  	The number of bytes that can be written to this socket.

  Remarks:
    None

  */
uint16_t            TCPIP_UDP_PutIsReady(UDP_SOCKET hUDP);

// *****************************************************************************

/*
  Function:
	uint16_t TCPIP_UDP_ArrayPut(UDP_SOCKET hUDP, const uint8_t *cData, uint16_t wDataLen)

  Summary:
	Writes an array of bytes to the UDP socket.
	
  Description:
	This function writes an array of bytes to the UDP socket, 
	while incrementing the socket write pointer.
    TCPIP_UDP_PutIsReady could be used before calling this function
    to verify that there is room in the socket buffer.

  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket
    cData - valid pointer

  Parameters:
	hUDP  - UDP socket handle
	cData - The array to write to the socket.
	wDataLen - Number of bytes from cData to be written.
	
  Returns:
  	The number of bytes successfully placed in the UDP transmit buffer.
    If this value is less than wDataLen, then the buffer became full and the
  	input was truncated.

  */
uint16_t            TCPIP_UDP_ArrayPut(UDP_SOCKET hUDP, const uint8_t *cData, uint16_t wDataLen);

// *****************************************************************************

/*
  Function:
	uint8_t* TCPIP_UDP_StringPut(UDP_SOCKET hUDP, const uint8_t *strData)

  Summary:
	Writes a null-terminated string to the UDP socket.
	
  Description:
	This function writes a null-terminated string to the UDP socket
    while incrementing the socket write pointer.
    TCPIP_UDP_PutIsReady could be used before
    calling this function to verify that there is room in the socket buffer.

  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP    - valid socket
    strData - valid pointer

  Parameters:
    hUDP     - UDP socket handle
	strData  - Pointer to the string to be written to the socket.
	
  Returns:
  	A pointer to the byte following the last byte written.
    Note that this is different than the TCPIP_UDP_ArrayPut functions.
    If this pointer does not dereference to a NULL byte,
    then the buffer became full and the input data was truncated.

  */
const uint8_t*      TCPIP_UDP_StringPut(UDP_SOCKET hUDP, const uint8_t *strData);

// *****************************************************************************

/*
  Function:
	uint16_t TCPIP_UDP_TxCountGet(UDP_SOCKET hUDP)

  Summary:
	Returns the amount of bytes written into the UDP socket.
	
  Description:
	This function returns the amount of bytes written into the UDP socket,
    i.e. the current position of the write pointer. 

  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket

  Parameters:
	hUDP   - UDP socket handle

  Return Values:
  	number of bytes in the socket TX buffer
    0 if the socket is invalid

  */
uint16_t            TCPIP_UDP_TxCountGet(UDP_SOCKET hUDP);

// *****************************************************************************

/*
  Function:
	uint16_t TCPIP_UDP_Flush(UDP_SOCKET hUDP)

  Summary:
	Transmits all pending data in a UDP socket.
	
  Description:
	This function builds a UDP packet with the pending TX data and marks it 
	for transmission over the network interface.
    There is no UDP state machine to send the socket data automatically.
    The UDP socket client must call this function to actually send the data over
    the network.

  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket

  Parameters:
    hUDP   - UDP socket handle
	
  Returns:
  	The number of bytes that currently were in the socket TX buffer
    and have been flushed.
    0 if the packet could not be transmitted:
        - invalid socket
        - invalid remote address
        - no route to the remote host could be found

  Remarks:
	Note that a UDP socket must be flushed in order to
    send data over the network.
    There is no UDP state machine (auto transmit) for UDP sockets.

  */
uint16_t            TCPIP_UDP_Flush(UDP_SOCKET hUDP);

// *****************************************************************************

/*
  Function:
	uint16_t TCPIP_UDP_Put(UDP_SOCKET hUDP, uint8_t v)

  Summary:
	Writes a byte to the UDP socket.
	
  Description:
	This function writes a single byte to the UDP socket, 
	while incrementing the socket write pointer.
    TCPIP_UDP_PutIsReady could be used before calling this function
    to verify that there is room in the socket buffer.

  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket

  Parameters:
    hUDP   - UDP socket handle
	v - The byte to be loaded into the transmit buffer.

  Return Values: The number of bytes successfully written to the socket
  	1 - The byte was successfully written to the socket.
  	0 - The transmit buffer is already full and so the write failed
        or the socket is invalid.
    

  Remarks:
    This function is very inefficient and its use is discouraged.
    A buffered approach (TCPIP_UDP_ArrayPut) is preferred.

  */
uint16_t                TCPIP_UDP_Put(UDP_SOCKET hUDP, uint8_t v);

// *****************************************************************************

/*
  Function:
	uint16_t TCPIP_UDP_GetIsReady(UDP_SOCKET hUDP)

  Summary:
	Determines how many bytes can be read from the UDP socket.
	
  Description:
	This function will return the number of bytes that are available
    in the specified UDP socket RX buffer.
  
  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket

  Parameters:
    hUDP   - UDP socket handle

  Returns:
  	The number of bytes that can be read from this socket.

  Remarks:
    None

  */
uint16_t            TCPIP_UDP_GetIsReady(UDP_SOCKET hUDP);

// *****************************************************************************

/*
  Function:
	void TCPIP_UDP_RxOffsetSet(UDP_SOCKET hUDP, uint16_t wOffset)

  Summary:
	Moves the read pointer within the socket RX buffer.
	
  Description:
	This function allows the user to specify the read location within the socket RX buffer.
    Future calls to TCPIP_UDP_Get and TCPIP_UDP_ArrayGet will read data from
	the indicated location forward.

  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket

  Parameters:
    hUDP    - UDP socket handle
	wOffset - Offset from beginning of UDP packet data payload to place the
		      read pointer.

  Returns:
  	None

  */
void                TCPIP_UDP_RxOffsetSet(UDP_SOCKET hUDP, uint16_t rOffset);

// *****************************************************************************

/*
  Function:
	uint16_t TCPIP_UDP_ArrayGet(UDP_SOCKET hUDP, uint8_t *cData, uint16_t wDataLen)

  Summary:
	Reads an array of bytes from the UDP socket.
	
  Description:
	This function reads an array of bytes from the UDP socket, 
	while adjusting the current read pointer and decrementing
    the remaining bytes available.
    TCPIP_UDP_GetIsReady could be used before calling this function
    to get the number of the available bytes in the socket.

    
  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket

  Parameters:
    hUDP     - UDP socket handle
	cData    - The buffer to receive the bytes being read.
               If NULL, the bytes are simply discarded 
	wDataLen - Number of bytes to be read from the socket.
	
  Returns:
  	The number of bytes successfully read from the UDP buffer.
    If this value is less than wDataLen, then the buffer was emptied
    and no more data is available.

  Remarks:
    For discarding a large number of bytes in the RX buffer the TCPIP_UDP_RxOffsetSet()
    usage will be more efficient.

  */
uint16_t            TCPIP_UDP_ArrayGet(UDP_SOCKET hUDP, uint8_t *cData, uint16_t wDataLen);

// *****************************************************************************

/*
  Function:
	uint16_t TCPIP_UDP_Get(UDP_SOCKET hUDP, uint8_t *v)

  Summary:
	Reads a byte from the UDP socket.
	
  Description:
	This function reads a single byte from the UDP socket, 
	while decrementing the remaining RX buffer length.
    TCPIP_UDP_GetIsReady could be used before calling this function
    to get the number of bytes available in the socket.

  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket

  Parameters:
    hUDP   - socket handle
	v - The buffer to receive the data being read.

  Return Values: The number of bytes successfully read
  	1 - A byte was successfully read
  	0 - No data available in the read buffer or invalid socket

  Remarks:
    This function is very inefficient and its usage is discouraged.
    A buffered approach (TCPIP_UDP_ArrayGet) is preferred.

 */
uint16_t                TCPIP_UDP_Get(UDP_SOCKET hUDP, uint8_t *v);

// *****************************************************************************

/*
  Function:
	void TCPIP_UDP_Discard(UDP_SOCKET hUDP)

  Summary:
	Discards any remaining RX data from a UDP socket.
	
  Description:
	This function discards any remaining received data in the UDP socket.

  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket

  Parameters:
    hUDP   - socket handle
	
  Returns:
  	None

  Remarks:
	If no data is available, this function does nothing.

    When data available, calling it repeatedly will discard
    one pending RX packet at a time.

  */
void                TCPIP_UDP_Discard(UDP_SOCKET hUDP);


// *****************************************************************************

/*
  Function:
	bool TCPIP_UDP_SocketNetSet(UDP_SOCKET hUDP, TCPIP_NET_HANDLE hNet)

  Summary:
	Sets the network interface for an UDP socket
	
  Description:
	This function sets the network interface for an UDP socket

  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket

  Parameters:
	hUDP - The UDP socket
   	hNet - interface handle
	
  Returns:
    true if success
    false otherwise.

  Remarks:
    An invalid hNet can be passed (0) so that the current
    socket network interface selection will be cleared

  */
bool                TCPIP_UDP_SocketNetSet(UDP_SOCKET hUDP, TCPIP_NET_HANDLE hNet);

// *****************************************************************************

/*
  Function:
	TCPIP_NET_HANDLE TCPIP_UDP_SocketNetGet(UDP_SOCKET hUDP)

  Summary:
	Gets the network interface of an UDP socket
	
  Description:
	This function returns the interface handle of an UDP socket

  Precondition:
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP - valid socket

  Parameters:
	hUDP - The UDP socket
	
  Returns:
    Handle of the interface that socket currently uses.

  */
TCPIP_NET_HANDLE    TCPIP_UDP_SocketNetGet(UDP_SOCKET hUDP);

// *****************************************************************************

/*
  Function:
	bool TCPIP_UDP_BcastIPV4AddressSet(UDP_SOCKET hUDP, UDP_SOCKET_BCAST_TYPE bcastType, TCPIP_NET_HANDLE hNet)

  Summary:
	Sets the broadcast IP address of a socket
	Allows an UDP socket to send broadcasts.
	
  Description:
      It sets the broadcast address for the socket


  Precondition:
	UDP initialized
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP  - valid socket

  Parameters:
	hUDP			-	the UDP socket
	bcastType   	-	Type of broadcast
	hNet        	- 	handle of an interface to use for the network directed broadcast
                        Not used for network limited broadcast
	
  Returns:
    true if success
	false otherwise:
        - invalid socket
        - invalid socket address type
        - a broadcast for the specified interface could not be obtained
        - invalid broadcast type specified

  Remarks:
    This function allows changing of the destination IPv4 address dynamically.

    However, the call will fail if the socket was previously set to broadcast
    using the TCPIP_UDP_OptionsSet call.
    TCPIP_UDP_OptionsSet takes precedence.

  */
bool            TCPIP_UDP_BcastIPV4AddressSet(UDP_SOCKET hUDP, UDP_SOCKET_BCAST_TYPE bcastType, TCPIP_NET_HANDLE hNet);

// *****************************************************************************

/*
  Function:
	bool TCPIP_UDP_DestinationIPAddressSet(UDP_SOCKET hUDP, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* remoteAddress)

  Summary:
	Sets the destination IP address of a socket
	
  Description:
      - It sets the IP destination address
        This allows changing the IP destination address dynamically.


  Precondition:
	UDP initialized
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP  - valid socket
    remoteAddress - valid address pointer

  Parameters:
	hUDP			-	the UDP socket
	addType        	-	Type of address: IPv4/IPv6
	remoteAddress   - 	pointer to an address to use 
	
  Returns:
    true if success
	false otherwise:
        - invalid socket
        - invalid socket address type
        - socket is of broadcast type

  Remarks:
    The call will fail if the socket was previously set to broadcast
    using the TCPIP_UDP_OptionsSet call.
    TCPIP_UDP_OptionsSet takes precedence.

    The call will fail if remoteAddress is 0.
    The destination IP address will not be changed.
    
  */
bool    TCPIP_UDP_DestinationIPAddressSet(UDP_SOCKET hUDP, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* remoteAddress);

// *****************************************************************************

/*
  Function:
    bool TCPIP_UDP_SourceIPAddressSet(UDP_SOCKET hUDP, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* localAddress)

  Summary:
	Sets the source IP address of a socket
	
  Description:
      - It sets the IP source address
        This allows changing the source IP address dynamically.


  Precondition:
	UDP initialized
	UDP socket should have been opened with TCPIP_UDP_ServerOpen()/TCPIP_UDP_ClientOpen()().
    hUDP  - valid socket
    localAddress - valid address pointer

  Parameters:
	hUDP			- the UDP socket
	addType        	- Type of address: IPv4/IPv6
	localAddress    - pointer to an address to use 
	
  Returns:
    true if success
	false otherwise:
        - invalid socket
        - invalid socket address type
        - unspecified localAddress


  Remarks:
    The call will fail if localAddress is 0.
    The source IP address will not be changed.
    
  */
bool TCPIP_UDP_SourceIPAddressSet(UDP_SOCKET hUDP, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* localAddress);


#endif  // __UDP_H_


