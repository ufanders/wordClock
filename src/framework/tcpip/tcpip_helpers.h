/*******************************************************************************
  Header file for tcpip_stack_helpers

  Company:
    Microchip Technology Inc.
    
  File Name:
    tcpip_helpers.h

  Summary:

  Description:
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

#ifndef __TCPIP_HELPERS_H_
#define __TCPIP_HELPERS_H_

bool	TCPIP_Helper_StringToIPAddress(const char* str, IPV4_ADDR* IPAddress);

bool    TCPIP_Helper_IPAddressToString(const IPV4_ADDR* IPAddress, char* buff, size_t buffSize);


bool    TCPIP_Helper_StringToIPv6Address (const char * str, IPV6_ADDR * addr);

bool    TCPIP_Helper_IPv6AddressToString (const IPV6_ADDR * addr, char* buff, size_t buffSize);


extern __inline__ bool __attribute__((always_inline)) TCPIP_Helper_IsBcastAddress(IPV4_ADDR* IPAddress)
{
    return (IPAddress->Val == 0xFFFFFFFF);
}

extern __inline__ bool __attribute__((always_inline)) TCPIP_Helper_IsMcastAddress(IPV4_ADDR* IPAddress)
{
    return ((IPAddress->v[0] & 0xf0) == 0xE0);
}

bool            TCPIP_Helper_StringToMACAddress(const char* str, uint8_t macAddr[6]);

bool            TCPIP_Helper_MACAddressToString(const TCPIP_MAC_ADDR* macAddr, char* buff, size_t buffSize);

// helpers to convert a host long to a network long
// and reverse
//
// helpers to convert a host short to a network short
// and reverse
//

#if !defined(__PIC32MX__)

extern inline uint32_t __attribute__((always_inline)) TCPIP_Helper_htonl(uint32_t hLong)
{
    return (((hLong & 0x000000ff) << 24) | ((hLong & 0x0000ff00) << 8) | ((hLong & 0x00ff0000) >> 8) | ((hLong & 0xff000000) >> 24));
}

extern inline uint16_t __attribute__((always_inline)) TCPIP_Helper_htons(uint16_t hShort)
{
       return (((hShort) << 8) | ((hShort) >> 8));
}
#else

uint32_t __attribute__((nomips16)) TCPIP_Helper_htonl(uint32_t hLong);

uint16_t __attribute__((nomips16)) TCPIP_Helper_htons(uint16_t hShort);


#endif  // !defined(__PIC32MX__)



uint32_t    TCPIP_Helper_ntohl(uint32_t nLong);
#define     TCPIP_Helper_ntohl(n)   TCPIP_Helper_htonl(n)

uint16_t TCPIP_Helper_ntohs(uint16_t nShort);
#define  TCPIP_Helper_ntohs(n)  TCPIP_Helper_htons(n) 

/*****************************************************************************
  Function:
	void TCPIP_Helper_FormatNetBIOSName(uint8_t Name[])

  Summary:
	Formats a string to a valid NetBIOS name.

  Description:
	This function formats a string to a valid NetBIOS name.  Names will be
	exactly 16 characters, as defined by the NetBIOS spec.  The 16th 
	character will be a 0x00 byte, while the other 15 will be the 
	provided string, padded with spaces as necessary.

  Precondition:
	None

  Parameters:
	Name - the string to format as a NetBIOS name.  This parameter must have
	  at least 16 bytes allocated.

  Returns:
	None
  ***************************************************************************/
void    TCPIP_Helper_FormatNetBIOSName(uint8_t Name[]);

#endif  // __TCPIP_HELPERS_H_

