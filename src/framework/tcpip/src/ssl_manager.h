/*******************************************************************************
  SSLv3 Module Manager API

  Company:
    Microchip Technology Inc.
    
  File Name:
    ssl_manager.h

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

#ifndef __SSL_MANAGER_H_
#define __SSL_MANAGER_H_

// Identifier for invalid SSL allocations
#define     SSL_INVALID_ID (0xFFu)

bool        TCPIP_SSL_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackData,
                const SSL_MODULE_CONFIG* sslData);
void        TCPIP_SSL_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackData);


void        TCPIP_SSL_MACBegin(uint8_t* MACSecret, uint32_t seq, uint8_t protocol, uint16_t len);
void        TCPIP_SSL_MACAdd(uint8_t* data, uint16_t len);
void        TCPIP_SSL_MACCalc(uint8_t* MACSecret, uint8_t* result);


void        TCPIP_SSL_PartialRecordStart(TCP_SOCKET hTCP, uint8_t sslStubID, uint8_t txProtocol, uint16_t wLen);
#define     TCPIP_SSL_PartialRecordFlush(a)        TCPIP_TCPSSL_RecordHeaderPut(a, NULL, false);
#define     TCPIP_SSL_PartialRecordFinish(a)       TCPIP_TCPSSL_RecordHeaderPut(a, NULL, true);

void        TCPIP_SSL_Task(void);
bool        TCPIP_SSL_TaskIsPending(void);

uint8_t*    TCPIP_SSL_BufferGet(int netIx);

uint8_t     TCPIP_SSL_StartSession(TCP_SOCKET hTCP, void * buffer, uint8_t supDataType);
void        TCPIP_SSL_Terminate(TCP_SOCKET hTCP, uint8_t sslStubId);
void        TCPIP_SSL_PeriodicTask(TCP_SOCKET hTCP, uint8_t sslStubID);
uint16_t    TCPIP_SSL_RecordReceive(TCP_SOCKET hTCP, uint8_t sslStubID);
void        TCPIP_SSL_HandshakeReceive(TCP_SOCKET hTCP, uint8_t sslStubID);
void        TCPIP_SSL_RecordTransmit(TCP_SOCKET hTCP, uint8_t sslStubID, uint8_t txProtocol);
void        TCPIP_SSL_MessageTransmit(TCP_SOCKET hTCP, uint8_t sslStubID, uint8_t msg);

#endif  // __SSL_MANAGER_H_





