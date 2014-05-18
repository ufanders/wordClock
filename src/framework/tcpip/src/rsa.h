/*******************************************************************************
  RSA Public Key Encryption Library Header

  Company:
    Microchip Technology Inc.
    
  File Name:
    rsa.h

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

#ifndef __RSA_H_
#define __RSA_H_

#define RSA_KEY_WORDS   (SSL_RSA_KEY_SIZE/BIGINT_DATA_SIZE)     // Represents the number of words in a key
#define RSA_PRIME_WORDS (SSL_RSA_KEY_SIZE/BIGINT_DATA_SIZE/2)   // Represents the number of words in an RSA prime

/****************************************************************************
  Section:
    Status Codes
  ***************************************************************************/


// Status response from RSA procedures
typedef enum
{
    RSA_WORKING = 0u,   // RSA is working through a calculation
    RSA_FINISHED_M1,    // RSA decryption has just completed calculation of the M1 CRT value
    RSA_FINISHED_M2,    // RSA decryption has just completed calculation of the M2 CRT value
    RSA_DONE            // The RSA calculation is complete
} RSA_STATUS;

// Indicates the data format for any RSA integer
typedef enum
{
    RSA_BIG_ENDIAN = 0u,    // Data expressed with the most significant byte first
    RSA_LITTLE_ENDIAN       // Data expressed with the least significant byte first
} RSA_DATA_FORMAT;

// Indicates the RSA operation to be completed
typedef enum
{
    RSA_OP_ENCRYPT = 0u,    // This is an encryption procedure
    RSA_OP_DECRYPT          // This is a decryption procedure
} RSA_OP;

typedef struct
{
} RSA_MODULE_CONFIG;

/****************************************************************************
  Section:
    Function Prototypes
  ***************************************************************************/

bool        TCPIP_RSA_UsageBegin(TCPIP_NET_HANDLE netH, RSA_OP op, uint16_t vKeyByteLen);
void        TCPIP_RSA_UsageEnd(TCPIP_NET_HANDLE netH);
void        TCPIP_RSA_DataSet(TCPIP_NET_HANDLE netH, uint8_t* data, uint16_t len, RSA_DATA_FORMAT format);
void        TCPIP_RSA_ResultSet(uint8_t* data, RSA_DATA_FORMAT format);
RSA_STATUS  TCPIP_RSA_StepCalculation(TCPIP_NET_HANDLE netH);
void        TCPIP_RSA_ExponentSet(uint8_t* data, uint8_t len, RSA_DATA_FORMAT format);
void        TCPIP_RSA_ModulusSet(uint8_t* data, RSA_DATA_FORMAT format);

#define TCPIP_RSA_EncryptBegin(netH, a)    TCPIP_RSA_UsageBegin(netH, RSA_OP_ENCRYPT, a)
#define TCPIP_RSA_EncryptEnd(netH)         TCPIP_RSA_UsageEnd(netH)

#define TCPIP_RSA_DecryptBegin(netH)       TCPIP_RSA_UsageBegin(netH, RSA_OP_DECRYPT, SSL_RSA_KEY_SIZE/8)
#define TCPIP_RSA_DecryptEnd(netH)         TCPIP_RSA_UsageEnd(netH)


#endif  // __RSA_H_
