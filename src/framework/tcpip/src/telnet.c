/*******************************************************************************
  Telnet Server

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides Telnet services on TCP port 23
    -Reference: RFC 854
*******************************************************************************/

/*******************************************************************************
File Name:  Telnet.c
Copyright ?2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND,
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

#define __TELNET_C

#include "tcpip/src/tcpip_private.h"

#if defined(TCPIP_STACK_USE_TELNET_SERVER)


#if !defined(MAX_TELNET_CONNECTIONS)
    // Maximum number of Telnet connections
	#define MAX_TELNET_CONNECTIONS	(2u)
#endif
#if !defined(TELNET_USERNAME)
    // Default Telnet user name
	#define TELNET_USERNAME		"admin"
#endif
#if !defined(TELNET_PASSWORD)
    // Default Telnet password
	#define TELNET_PASSWORD		"microchip"
#endif

#define TELNET_LINE_RETURN  "\r"
#define TELNET_LINE_FEED    "\n"
#define TELNET_LINE_TERM    TELNET_LINE_RETURN TELNET_LINE_FEED  

// limited set of supported telnet commands 
#define TELNET_CMD_IAC          "\xff"
#define TELNET_CMD_DONT         "\xfe"
#define TELNET_CMD_DO           "\xfd"
#define TELNET_CMD_WONT         "\xfc"
#define TELNET_CMD_WILL         "\xfb"

#define TELNET_CMD_IAC_CODE     '\xff'
#define TELNET_CMD_DONT_CODE    '\xfe'
#define TELNET_CMD_DO_CODE      '\xfd'
#define TELNET_CMD_WONT_CODE    '\xfc'
#define TELNET_CMD_WILL_CODE    '\xfb'



// limited set of supported telnet options
#define TELNET_OPT_SUPP_LOCAL_ECHO  "\x2d"      // suppress local echo


// connection display strings
//

// start up message
// 2J is clear screen, 31m is red, 1m is bold
// 0m is clear all attributes
#define TELNET_START_MSG                "\x1b[2J\x1b[31m\x1b[1m" \
                                "Microchip Telnet Server 1.1\x1b[0m\r\n" \
								"Login: "
//
// ask password message
#define TELNET_ASK_PASSWORD_MSG     "Password: " TELNET_CMD_IAC TELNET_CMD_DO TELNET_OPT_SUPP_LOCAL_ECHO        // ask Suppress Local Echo

// Access denied message/ failed logon
#define TELNET_FAIL_LOGON_MSG       TELNET_LINE_TERM "Access denied" TELNET_LINE_TERM TELNET_LINE_TERM      

// internal buffer overflow message
#define TELNET_BUFFER_OVFLOW_MSG    "Too much data. Aborted" TELNET_LINE_TERM

// Successful authentication message/log on OK
#define TELNET_LOGON_OK             TELNET_LINE_TERM "Logged in successfully" TELNET_LINE_TERM TELNET_LINE_TERM 

// welcome message
#define TELNET_WELCOME_MSG          TELNET_LINE_TERM "--- Telnet Console ---" TELNET_LINE_TERM \
                                                     "Type help for commands" TELNET_LINE_TERM ">"

// disconnect message
//#define TELNET_BYE_MSG            TELNET_LINE_TERM TELNET_LINE_TERM "Goodbye!" TELNET_LINE_TERM TELNET_LINE_TERM

// failure to register with the command processor
#define TELNET_FAIL_CMD_REGISTER    "Failed to connect to the command processor. Aborting!" TELNET_LINE_TERM

// buffering defines
#define TELNET_PRINT_BUFF           200     // internal print buffer
#define TELNET_LINE_BUFF            (80 +3) // assembled line buffer for password, authentication, etc
                                            // + extra room for \r\n
#define TELNET_SKT_MESSAGE_SPACE    80      // min space needed in the socket buffer for displaying messages


// machine state
typedef	enum
{
    SM_HOME = 0,
    SM_PRINT_LOGIN,
    SM_GET_LOGIN,
    SM_GET_PASSWORD,
    SM_GET_PASSWORD_BAD_LOGIN,
    SM_AUTHENTICATED,
    SM_CONNECTED
} TELNET_STATE;

typedef enum
{
    TELNET_MSG_LINE_PENDING,        // no line assembled yet
    TELNET_MSG_LINE_DONE,           // line assembled
    TELNET_MSG_LINE_OVFL            // line buffer capacity exceeded
}TELNET_MSG_LINE_RES;   // message line result
    
typedef struct
{
    TCP_SOCKET          telnetSkt;
    TELNET_STATE        telnetState;
    _CMDIO_DEV_NODE*    telnetIO;
}TELNET_DCPT;

static TELNET_DCPT      telnetDcpt[MAX_TELNET_CONNECTIONS];

static int              telnetInitCount = 0;      // TELNET module initialization count


// prototypes
static void _Telnet_MSG(const void* cmdIoParam, const char* str);
static void _Telnet_PRINT(const void* cmdIoParam, const char* format, ...);
static void _Telnet_PUTC(const void* cmdIoParam, char c);
static bool _Telnet_DATA_RDY(const void* cmdIoParam);
static char _Telnet_GETC(const void* cmdIoParam);

static void _Telnet_Deregister(TELNET_DCPT* pDcpt);
static void _Telnet_Close(TELNET_DCPT* pDcpt);

static TELNET_STATE _Telnet_UserCheck(TCP_SOCKET tSocket, TELNET_STATE tState);
static TELNET_STATE _Telnet_LogonCheck(TCP_SOCKET tSocket, TELNET_STATE tState);
static TELNET_MSG_LINE_RES _Telnet_MessageLineCheck(TCP_SOCKET tSkt, char* lineBuffer, int bufferSize, int* readBytes);
static char* _Telnet_CommandsSkip(const char* strMsg);

static const _CMDIO_DEV_API telnetIOApi = 
{
    _Telnet_MSG,
    _Telnet_PRINT,
    _Telnet_PUTC,
    _Telnet_DATA_RDY,
    _Telnet_GETC
};



// implementation
//


bool TCPIP_TELNET_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const TELNET_MODULE_CONFIG* pTelConfig)
{

    int tIx;
    TELNET_DCPT* pDcpt;

    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    // interface restart


    // stack init

    if(telnetInitCount == 0)
    {   // first time we're run
        pDcpt = telnetDcpt;
        for(tIx = 0; tIx < sizeof(telnetDcpt)/sizeof(*telnetDcpt); tIx++, pDcpt++)
        {
            pDcpt->telnetSkt = INVALID_SOCKET;
            pDcpt->telnetState = SM_HOME;
        }

    }

    telnetInitCount++;


    return true;
}


void TCPIP_TELNET_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    int tIx;

    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down

    
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
    {   // whole stack is going down
        if(telnetInitCount > 0)
        {   // we're up and running
            if(--telnetInitCount == 0)
            {   // all closed
                for(tIx = 0; tIx < sizeof(telnetDcpt)/sizeof(*telnetDcpt); tIx++)
                {
                    _Telnet_Close(telnetDcpt + tIx);
                }
            }
        }
    }

}


static void _Telnet_Deregister(TELNET_DCPT* pDcpt)
{
    if (pDcpt->telnetIO != 0)
    {
        _SYS_CMDIO_DELETE(pDcpt->telnetIO);
        pDcpt->telnetIO = 0;
    }

}

static void _Telnet_Close(TELNET_DCPT* pDcpt)
{

    _Telnet_Deregister(pDcpt);

    if( pDcpt->telnetSkt != INVALID_SOCKET)
    {
        TCPIP_TCP_Close(pDcpt->telnetSkt);
        pDcpt->telnetSkt = INVALID_SOCKET;
    }

    pDcpt->telnetState = SM_HOME;

}

// Telnet's PUTC
static void _Telnet_PUTC(const void* cmdIoParam, char c)
{
    TCP_SOCKET tSkt = (TCP_SOCKET)(int)cmdIoParam;
    if(tSkt != INVALID_SOCKET)
    {
        TCPIP_TCP_Put(tSkt, (uint8_t)c);
    }
}

// Telnet's message	
static void _Telnet_MSG(const void* cmdIoParam, const char* str)
{
    TCP_SOCKET tSkt = (TCP_SOCKET)(int)cmdIoParam;
    if(tSkt != INVALID_SOCKET)
    {
        TCPIP_TCP_StringPut(tSkt, (const uint8_t*)str);
    }
}


// Telnet's print
static void _Telnet_PRINT(const void* cmdIoParam, const char* format, ...)
{
    va_list arg_list;
    char buff[TELNET_PRINT_BUFF];

    va_start(arg_list, format);
    vsnprintf(buff, TELNET_PRINT_BUFF, format, arg_list);
    va_end(arg_list);

    _Telnet_MSG(cmdIoParam, buff);
}


// Telnet's data ready
static bool _Telnet_DATA_RDY(const void* cmdIoParam)
{
    TCP_SOCKET tSkt = (TCP_SOCKET)(int)cmdIoParam;
    if(tSkt != INVALID_SOCKET)
    {
        return TCPIP_TCP_GetIsReady(tSkt) != 0;
    }

    return false;
}

// Telnet's getc
static char _Telnet_GETC(const void* cmdIoParam)
{

    TCP_SOCKET tSkt = (TCP_SOCKET)(int)cmdIoParam;
    if(tSkt != INVALID_SOCKET)
    {
        uint8_t bData;
        if(TCPIP_TCP_Get(tSkt, &bData))
        {
            return (char)bData;
        }
    }

    return 0;
}



/*********************************************************************
 * Function:        bool TCPIP_TELNET_Task(TCPIP_NET_IF* pNetIf)
 *
 * PreCondition:    Stack is initialized()
 *
 * Input:           pNetIf  - network interface
 *
 * Output:          true if processing OK, false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        Performs Telnet Server related tasks.  Contains
 *                  the Telnet state machine and state tracking
 *                  variables.
 *
 * Note:            None
 ********************************************************************/
bool TCPIP_TELNET_Task(TCPIP_NET_IF* pNetIf)
{
    int         tIx;
    TELNET_DCPT* pDcpt;
    TCP_SOCKET	tSocket;
    TELNET_STATE tState;


    // Loop through each telnet session and process state changes and TX/RX data
    pDcpt = telnetDcpt;
    for(tIx = 0; tIx < sizeof(telnetDcpt)/sizeof(*telnetDcpt); tIx++, pDcpt++)
    {
        // Load up static state information for this session
        tSocket = pDcpt->telnetSkt;
        tState = pDcpt->telnetState;

        // Reset our state if the remote client disconnected from us
        if(tSocket != INVALID_SOCKET)
        {
            if(TCPIP_TCP_WasReset(tSocket))
            {
                // Deregister IO and free its space
                _Telnet_Deregister(pDcpt);
                tState = SM_PRINT_LOGIN;
            }
        }

        // Handle session state
        switch(tState)
        {
            case SM_HOME:
                // Connect a socket to the remote TCP server
                tSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_ANY, TCPIP_TELNET_SERVER_PORT, 0);

                // Abort operation if no TCP socket could be opened.
                // If this ever happens, you need to update your tcp_config.h
                if(tSocket == INVALID_SOCKET)
                {
                    break;
                }

                pDcpt->telnetSkt = tSocket;
                // Open an SSL listener if SSL server support is enabled
#if defined(TCPIP_STACK_USE_SSL_SERVER)
                TCPIP_TCPSSL_ListenerAdd(tSocket, TCPIP_TELNET_SERVER_SECURE_PORT);
#endif

                tState++;
                break;

            case SM_PRINT_LOGIN:
#if defined(TCPIP_STACK_USE_SSL_SERVER)
                // Reject unsecured connections if TELNET_REJECT_UNSECURED is defined
#if defined(TELNET_REJECT_UNSECURED)
                if(!TCPIP_TCP_SocketIsSecuredBySSL(tSocket))
                {
                    if(TCPIP_TCP_IsConnected(tSocket))
                    {
                        TCPIP_TCP_Close(tSocket);
                        break;
                    }	
                }
#endif

                // Don't attempt to transmit anything if we are still handshaking.
                if(TCPIP_TCPSSL_StillHandshaking(tSocket))
                    break;
#endif

                // Make certain the socket can be written to
                if(TCPIP_TCP_PutIsReady(tSocket) < TELNET_SKT_MESSAGE_SPACE)
                    break;

                // Place the application protocol data into the transmit buffer.
                TCPIP_TCP_StringPut(tSocket, (const uint8_t*)TELNET_START_MSG);

                // Send the packet
                TCPIP_TCP_Flush(tSocket);
                tState++;

            case SM_GET_LOGIN:
                tState = _Telnet_UserCheck(tSocket, tState);
                break;

            case SM_GET_PASSWORD:
            case SM_GET_PASSWORD_BAD_LOGIN:

                tState = _Telnet_LogonCheck(tSocket, tState);
                break;

            case SM_AUTHENTICATED:
                if(TCPIP_TCP_PutIsReady(tSocket) < TELNET_SKT_MESSAGE_SPACE)
                    break;

                TCPIP_TCP_StringPut(tSocket, (const uint8_t*)TELNET_WELCOME_MSG);
                tState++;

                TCPIP_TCP_Flush(tSocket);

                // Register telnet as cmd IO device
                pDcpt->telnetIO = _SYS_CMDIO_ADD(&telnetIOApi, (const void*)(int)tSocket);
                if (pDcpt->telnetIO == 0)
                {
                    TCPIP_TCP_StringPut(tSocket, (const uint8_t*)TELNET_FAIL_CMD_REGISTER);
                    TCPIP_TCP_Disconnect(tSocket);
                    tState = SM_PRINT_LOGIN;
                    break;
                }	

            case SM_CONNECTED:
                // Check if you're disconnected and de-register from the command processor

                break;
        }


        // Save session state back into the static array
        pDcpt->telnetState = tState;
    }

    return true;
}

static TELNET_STATE _Telnet_UserCheck(TCP_SOCKET tSkt, TELNET_STATE tState)
{
    int         avlblBytes;
    bool        userFound;
    char        *lineStr;
    TELNET_MSG_LINE_RES lineRes;


    char    userMessage[TELNET_LINE_BUFF];    // telnet confirmation message

    if(TCPIP_TCP_PutIsReady(tSkt) < TELNET_SKT_MESSAGE_SPACE)
    {   // wait some more
        return tState;
    }

    lineRes = _Telnet_MessageLineCheck(tSkt, userMessage, sizeof(userMessage), &avlblBytes);

    if(lineRes == TELNET_MSG_LINE_PENDING)
    {   // wait some more
        return tState;
    }
    else if(lineRes == TELNET_MSG_LINE_OVFL)
    {
        TCPIP_TCP_StringPut(tSkt, (const uint8_t*)TELNET_BUFFER_OVFLOW_MSG);
        TCPIP_TCP_StringPut(tSkt, (const uint8_t*)TELNET_FAIL_LOGON_MSG);
        TCPIP_TCP_Disconnect(tSkt);
        return SM_PRINT_LOGIN;	
    }

    // TELNET_MSG_LINE_DONE
    // ignore telnet commands/advertisments sent to us by the client
    // we do not support them!
    lineStr = _Telnet_CommandsSkip(userMessage);
    // remove the line termination 
    lineStr = strtok(lineStr, TELNET_LINE_TERM);
    // find the user name
    if(lineStr && strcmp(lineStr, TELNET_USERNAME) == 0)
    {
        userFound = true;
    }
    else
    {
        userFound = false;
    }

    TCPIP_TCP_ArrayGet(tSkt, 0, avlblBytes);  //  throw this line of data away
    // Print the password prompt
    TCPIP_TCP_StringPut(tSkt, (const uint8_t*)TELNET_ASK_PASSWORD_MSG);
    return userFound?SM_GET_PASSWORD:SM_GET_PASSWORD_BAD_LOGIN;

}

static TELNET_STATE _Telnet_LogonCheck(TCP_SOCKET tSkt, TELNET_STATE tState)
{
    int     avlblBytes;
    bool    sktDisconnect, sktOverflow;
    char*   lineStr;
    TELNET_MSG_LINE_RES lineRes;

    char    passMessage[TELNET_LINE_BUFF];    // telnet confirmation message


    if(TCPIP_TCP_PutIsReady(tSkt) < TELNET_SKT_MESSAGE_SPACE)
    {   // wait some more
        return tState;
    }

    lineRes = _Telnet_MessageLineCheck(tSkt, passMessage, sizeof(passMessage), &avlblBytes);

    if(lineRes == TELNET_MSG_LINE_PENDING)
    {   // wait some more
        return tState;
    }

    sktDisconnect = sktOverflow = false;

    if(lineRes == TELNET_MSG_LINE_OVFL)
    {
        sktOverflow = true;
    }
    else
    {   // TELNET_MSG_LINE_DONE
        // ignore telnet commands/advertisments sent to us by the client
        // we do not support them!
        lineStr = _Telnet_CommandsSkip(passMessage);
        // remove the line termination 
        lineStr = strtok(lineStr, TELNET_LINE_TERM);
        if(tState != SM_GET_PASSWORD || strcmp(lineStr, TELNET_PASSWORD) != 0)
        {   // failed
            sktDisconnect = true;
        }
    }

    if(sktOverflow)
    {
        TCPIP_TCP_StringPut(tSkt, (const uint8_t*)TELNET_BUFFER_OVFLOW_MSG);
    }

    if(sktOverflow || sktDisconnect)
    {
        TCPIP_TCP_StringPut(tSkt, (const uint8_t*)TELNET_FAIL_LOGON_MSG);
        TCPIP_TCP_Disconnect(tSkt);
        return SM_PRINT_LOGIN;	
    }

    // success
    TCPIP_TCP_ArrayGet(tSkt, 0, avlblBytes);  //  throw this line of data away
    // Print the authenticated prompt
    TCPIP_TCP_StringPut(tSkt, (const uint8_t*)TELNET_LOGON_OK);
    return SM_AUTHENTICATED;

}



// checks if a complete line is assembled
static TELNET_MSG_LINE_RES _Telnet_MessageLineCheck(TCP_SOCKET tSkt, char* lineBuffer, int bufferSize, int* readBytes)
{
    int     avlblBytes;
    char    *lineTerm;

    avlblBytes = TCPIP_TCP_ArrayPeek(tSkt, (uint8_t*)lineBuffer, bufferSize - 1, 0);

    if(avlblBytes)
    {   // we need at least one terminator character
        // make sure we have a complete line
        lineBuffer[avlblBytes] = 0;
        lineTerm = strstr(lineBuffer, TELNET_LINE_RETURN);
        if(lineTerm == 0)
        {
            lineTerm = strstr(lineBuffer, TELNET_LINE_FEED);
        }

        if(lineTerm != 0)
        {
            *readBytes = avlblBytes;
            return TELNET_MSG_LINE_DONE;
        }

        // no end of line pressed yet
        if(avlblBytes == bufferSize - 1)
        {   // our buffer is full: overflowed
            return TELNET_MSG_LINE_OVFL;
        }
        // else wait some more
    }

    return TELNET_MSG_LINE_PENDING;

}

static char* _Telnet_CommandsSkip(const char* strMsg)
{
    char c;
    while(true)
    {
        if(*strMsg != TELNET_CMD_IAC_CODE)
        {
            break;
        }
        // start of command
        c = *++strMsg;
        if(c == TELNET_CMD_IAC_CODE)
        {   // this is data, not command
            break;
        }
        // valid command sequence
        if(c == TELNET_CMD_DO_CODE || c == TELNET_CMD_DONT_CODE || c == TELNET_CMD_WILL_CODE || c == TELNET_CMD_WONT_CODE)
        {   // skip option character that follows
            strMsg += 2;
        }
        else
        {   // we don't support other commands for now
            break;
        }
    }

    return (char*)strMsg;

}

#endif	//#if defined(TCPIP_STACK_USE_TELNET_SERVER)
