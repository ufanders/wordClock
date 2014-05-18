/*******************************************************************************
  Simple Network Management Protocol (SNMP) Configuration file

  Company:
    Microchip Technology Inc.
    
  File Name:
    snmp_config.h

  Summary:
    SNMP configuration file

  Description:
    This file contains the SNMP module configuration options
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright © 2011 released Microchip Technology Inc.  All rights reserved.

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
#ifndef _SNMP_CONFIG_H_
#define _SNMP_CONFIG_H_

// SNMP agent information

// For multi-homed hosts, the default SNMP interface
#define MY_DEFAULT_SNMP_IF             "PIC32INT"

// The Microchip mib2bib.jar compiler is used to convert the Microchip MIB script to binary format 
// and it is compatible with the Microchip SNMP agent. which is written in ASCII format.
// Name of the bib file for SNMP is snmp.bib.
#define SNMP_BIB_FILE_NAME      "snmp.bib"

// Maximum length for the OID String. Change this to match your OID string length.
#define SNMP_OID_MAX_LEN         (18)

// The maximum length in octets of an SNMP message which this SNMP agent able to process.
// SNMP MIN and MAX message 484 bytes in size.
// As per RFC 3411 snmpEngineMaxMessageSize and RFC 1157 ( section 4- protocol specification )
// and implementation  supports more than 484 whenever feasible.
#define SNMP_MAX_MSG_SIZE       484

// Update the Non record id OID value which is part of CustomSnmpDemoApp.c file.
// This is the maximum size for gSnmpNonMibRecInfo[] which is the list of static variable Parent 
// OIDs which are not part of mib.h file. This structure is used to restrict access to static 
// variables of SNMPv3 OIDs from SNMPv2c and SNMPv1 version.
// With SNMPv3 all the OIDs accessible but when we are using SNMPv2c version , static variables of the SNMPv3
// cannot be accessible with SNMP version v2c.
// SNMP agent supports both SMIv1 and SMIv2 standard and 
// snmp.mib has been updated with respect to SMIV2 standard and it also includes
// MODULE-IDENTITY ( number 1)after ENTERPRISE-ID.
#define SNMP_MAX_NON_REC_ID_OID  3

// This is the maximum length for community string.
// Application must ensure that this length is observed.
// SNMP module adds one byte extra after SNMP_COMMUNITY_MAX_LEN
// for adding '\0' NULL character.
#define SNMP_COMMUNITY_MAX_LEN      (8u)

// Specifying more strings than SNMP_MAX_COMMUNITY_SUPPORT will result in the
// later strings being ignored (but still wasting program memory).  Specifying
// fewer strings is legal, as long as at least one is present.
#define SNMP_MAX_COMMUNITY_SUPPORT  (3u)

// Maximum length for SNMP Trap community name 
#define SNMP_NOTIFY_COMMUNITY_LEN        (SNMP_COMMUNITY_MAX_LEN)

// Default SNMPv2C community names.  These can be overridden at run time if
// alternate strings are present in external EEPROM or Flash (actual
// strings could be stored by the TCPIP storage service.
// These strings are case sensitive.
// An empty string means disabled (not matchable).
// For application security, these default community names should not be
// used, but should all be disabled to force the end user to select unique
// community names.  These defaults are provided only to make it easier to
// start development.  Specifying more strings than
// SNMP_MAX_COMMUNITY_SUPPORT will result in the later strings being
// ignored (but still wasting program memory).  Specifying fewer strings is
// legal, as long as at least one is present.  A string larger than
// SNMP_COMMUNITY_MAX_LEN bytes will be ignored.
#define SNMP_READ_COMMUNITIES       {"public", "read", ""}

// Default SNMPv2C write community names.  See SNMP_READ_COMMUNITIES for more information.
#define SNMP_WRITE_COMMUNITIES      {"private", "write", "public"}

// Trap information

// This macro will be used to avoid SNMP OID memory buffer corruption
#define SNMP_TRAP_COMMUNITY_MAX_LEN_MEM_USE   (8)

// This is the maximum number of interested receivers who should receive notifications 
// when some interesting event occurs.This macro is used as total size for both IPv4 and IPv6 
// trap receiver address table
#define SNMP_TRAP_TABLE_SIZE         (2)

// Maximum length of trap community name is used while sending notification to the receiver
#define SNMP_TRAP_COMMUNITY_MAX_LEN       (SNMP_TRAP_COMMUNITY_MAX_LEN_MEM_USE+1)

// Comment following line if SNMP TRAP support is needed
//#define SNMP_TRAP_DISABLED


// This macro is used to send SNMP TRAP v2 formatted trap PDUs
// By default SNMP agent is configured to send V2 formatted traps
// Note that the SNMP V2c agent should only send V2 formatted traps
#define SNMP_STACK_USE_V2_TRAP

// This macro is used to enable traps for SNMPv3. Both TRAPv1 and TRAPv2 version TRAP PDUs 
// can be transmitted with SNMPv3.
// Note that iReasoning SNMP manager supports SNMPv3 with TRAPv2.
#define SNMP_V1_V2_TRAP_WITH_SNMPV3



#endif  // _SNMP_CONFIG_H_
