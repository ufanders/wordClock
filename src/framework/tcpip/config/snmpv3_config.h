/*******************************************************************************
  Simple Network Management Protocol (SNMPv3) Configuration file

  Company:
    Microchip Technology Inc.
    
  File Name:
    snmpv3_config.h

  Summary:
    SNMPv3 configuration file
    
  Description:
    This file contains the SNMPv3 module configuration options    
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
#ifndef _SNMPv3_CONFIG_H_
#define _SNMPv3_CONFIG_H_

// Maximum number of SNMPv3 users.
// User Security Model should have at least 1 user. Default is 3.
#define SNMPV3_USM_MAX_USER	3

// Maximum size for SNMPv3 User Security Name length.
#define USER_SECURITY_NAME_LEN (16)

// User security name length for memory validation
#define SNMPV3_USER_SECURITY_NAME_LEN_MEM_USE (USER_SECURITY_NAME_LEN+1)

// SNMPv3 Authentication Localized password key length size
#define AUTH_LOCALIZED_PASSWORD_KEY_LEN	(20)

// SNMPv3 authentication localized Key length for memory validation
#define SNMPV3_AUTH_LOCALIZED_PASSWORD_KEY_LEN_MEM_USE (AUTH_LOCALIZED_PASSWORD_KEY_LEN+1)

// SNMPv3 Privacy Password key length size
#define PRIV_LOCALIZED_PASSWORD_KEY_LEN	(20)

// SNMPv3 privacy key length size for memory validation
#define SNMPV3_PRIV_LOCALIZED_PASSWORD_KEY_LEN_MEM_USE (PRIV_LOCALIZED_PASSWORD_KEY_LEN+1)

// SNMPv3 configuration details for user name , authentication and privacy
// SNMPv3 default user name database. This macro has 3 user names and this is as per SNMPV3_USM_MAX_USER. 
#define SNMPV3_USER_SECURITY_NAME_DB 	{"microchip","SnmpAdmin","root"}

// SNMPv3 default authentication data base. SNMP agent supports MD5, SHA1 algorithm.  
// SNMPV3_HAMC_MD5 supports MD5 authentication which is mapped with user name /"microchip/"
// SNMPV3_HMAC_SHA1 supports SHA1 authentication which is mapped with user name /"SnmpAdmin/"
// SNMPV3_NO_HMAC_AUTH supports no authentication which is mapped to the user name /"root/"
#define SNMPV3_USER_AUTH_TYPE_DB	 {SNMPV3_HAMC_MD5,SNMPV3_HMAC_SHA1,SNMPV3_NO_HMAC_AUTH}

// SNMPv3 default authentication data base for password.   
// /"auth12345/" is the password for SNMPV3_HAMC_MD5 authentication which is mapped to user name /"microchip/"
// /"ChandlerUS/" is the password for SHA1 authentication which is mapped to user name /"SnmpAdmin/"
// No password is required for SNMPV3_NO_HMAC_AUTH which is mapped to the user name /"root/"
#define SNMPV3_USER_AUTH_PASSWD_DB	 {"auth12345","ChandlerUS",""}

// SNMPv3 default privacy data base. AES algorithm is used for SNMPv3 encryption and decryption.  
// SNMPV3_AES_PRIV supports AES algorithm which is mapped with user name /"microchip/". 
// SNMPV3_NO_PRIV is used for No privacy which is mapped with user name /"SnmpAdmin/" and /"root"
#define SNMPV3_USER_PRIV_TYPE_DB	 {SNMPV3_AES_PRIV,SNMPV3_NO_PRIV,SNMPV3_NO_PRIV}

// SNMPv3 default authentication data base for password.   
// /"priv12345/" is the password for SNMPV3_AES_PRIV privacy which is mapped to user name /"microchip/"
// There is no password for SNMPV3_NO_PRIV
#define SNMPV3_USER_PRIV_PASSWD_DB   {"priv12345","",""}

// SNMPv3 Target configuration details for TRAP user name , authentication and privacy
// SNMPv3 default TRAP user name database. This macro has 3 user names and this is as per SNMPV3_USM_MAX_USER.
// This is mapped with SNMPV3_USER_SECURITY_NAME_DB. To transmit the trap, TRAP user name should match to the 
// SNMPV3_USER_SECURITY_NAME_DB user name data base.
#define SNMPV3_TRAP_USER_SECURITY_NAME_DB {"microchip","SnmpAdmin","root"}

// SNMPv3 target configuration with default trap authentication data base. SNMP agent supports MD5, SHA1 algorithm.  
// SNMPV3_MSG_PROCESSING_MODEL - Trap user name SNMPV3_TRAP_USER_SECURITY_NAME_DB supports SNMPv3 message processing model 
#define SNMPV3_TRAP_MSG_PROCESS_MODEL_DB {SNMPV3_MSG_PROCESSING_MODEL,SNMPV3_MSG_PROCESSING_MODEL,SNMPV3_MSG_PROCESSING_MODEL}

// SNMPv3 trap security model database
// SNMPV3_USM_SECURITY_MODEL - Trap user name SNMPV3_TRAP_USER_SECURITY_NAME_DB supports USM security model
#define SNMPV3_TRAP_SECURITY_MODEL_TYPE_DB {SNMPV3_USM_SECURITY_MODEL,SNMPV3_USM_SECURITY_MODEL,SNMPV3_USM_SECURITY_MODEL}

// SNMPv3 trap security level data base
// AUTH_PRIV - /"microchip/" trap user name is used with authentication and privacy privilege  
// AUTH_NO_PRIV - /"SnmpAdmin/" trap  user name with authentication and no privacy supports
// NO_AUTH_NO_PRIV - /"root/" trap user name with with no authentication and no privacy
#define SNMPV3_TRAP_SECURITY_LEVEL_DB {AUTH_PRIV,AUTH_NO_PRIV,NO_AUTH_NO_PRIV}


#endif  // _SNMPv3_CONFIG_H_
