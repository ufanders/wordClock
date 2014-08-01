/***********************************************************************
  Company:
    Microchip Technology Inc.
    
  File Name:
    drv_ethernet_flags.h

  Summary:
    Ethernet driver configuration flags file

  Description:
    Ethernet Drivers Configuration Flags
    
    This file provides the definition of commonly-used configuration
    enumerations for use with the Ethernet PHY and Ethernet MAC Drivers.
  ***********************************************************************/

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

#ifndef _DRV_ETHERNET_FLAGS_H
#define _DRV_ETHERNET_FLAGS_H

// *****************************************************************************
/* Ethernet PHY Configuration Flags

  Summary:
    Defines configuration options for the Ethernet PHY.

  Description:
    This enumeration defines configuration options for the Ethernet PHY.
    Used by: DRV_ETHPHY_MIIConfigure, DRV_ETHPHY_INIT structure, DRV_ETHPHY_Setup,
    Returned by: DRV_ETHPHY_HWConfigFlagsGet
*/

typedef enum  // ETHPHY flags, connection flags (formerly eEthPhyCfgFlags in eth_pic_32_ext_phy.h)
{
    // RMII data interface in configuration fuses.
    ETH_PHY_CFG_RMII        /*DOM-IGNORE-BEGIN*/ = 0x01 /*DOM-IGNORE-END*/ ,

     // MII data interface in configuration fuses.
    ETH_PHY_CFG_MII         /*DOM-IGNORE-BEGIN*/ = 0x00 /*DOM-IGNORE-END*/ ,

    // Configuration fuses is ALT
    ETH_PHY_CFG_ALTERNATE   /*DOM-IGNORE-BEGIN*/ = 0x02 /*DOM-IGNORE-END*/ ,

    // Configuration fuses is DEFAULT
    ETH_PHY_CFG_DEFAULT     /*DOM-IGNORE-BEGIN*/ = 0x00 /*DOM-IGNORE-END*/ ,

    // Use the fuses configuration to detect if you are RMII/MII and ALT/DEFAULT configuration
    ETH_PHY_CFG_AUTO        /*DOM-IGNORE-BEGIN*/ = 0x10 /*DOM-IGNORE-END*/
        // NOTE: - this option does not check the consistency btw the software call and the way the
        //         fuses are configured. If just assumes that the fuses are properly configured.
        //       - option is valid for DRV_ETHPHY_Setup() call only!

} ETHPHY_CONFIG_FLAGS;        // flags for DRV_ETHPHY_Setup() call


/**************************************************************************
  Summary:
    Defines the possible results of Ethernet operations that can succeed or
    fail
  Description:
    Ethernet Operation Result Codes
    
    This enumeration defines the possible results of any of the Ethernet
    library operations that have the possibility of failing. This result
    should be checked to ensure that the operation achieved the desired
    result.                                                                
**************************************************************************/

typedef enum
{
    // Everything ok
    ETH_RES_OK,

    // Ethernet RX, TX, acknowledge packets:

    // No such packet exist
    ETH_RES_NO_PACKET,

    // Packet is queued (not transmitted or received and not processed)
    ETH_RES_PACKET_QUEUED,

    // Ethernet buffers, descriptors errors:

    // Some memory allocation failed
    ETH_RES_OUT_OF_MEMORY,

    // Not enough descriptors available
    ETH_RES_NO_DESCRIPTORS,

    // We don't support user space buffers.
    ETH_RES_USPACE_ERR,

    // The size of the receive buffers too small
    ETH_RES_RX_SIZE_ERR,

    // A received packet spans more buffers/descriptors than supplied
    ETH_RES_RX_PKT_SPLIT_ERR,

    // No negotiation support
    ETH_RES_NEGOTIATION_UNABLE,

    // No negotiation active
    ETH_RES_NEGOTIATION_INACTIVE,

    // Negotiation not started yet
    ETH_RES_NEGOTIATION_NOT_STARTED,

    // Negotiation active
    ETH_RES_NEGOTIATION_ACTIVE,

    // Link down after negotiation, negotiation failed
    ETH_RES_NEGOTIATION_LINKDOWN,

    // PHY errors:

    /*No PHY was detected or it failed to respond to reset command*/
    ETH_RES_DTCT_ERR,

    /******************************************************************
      No match between the capabilities: the PHY supported and the open
      requested ones                                                   
      ******************************************************************/
    ETH_RES_CPBL_ERR,

    // Hardware configuration doesn't match the requested open mode
    ETH_RES_CFG_ERR,

} ETH_RESULT_CODE; /*DOM-IGNORE-BEGIN*/ /* in eth.h, eEthRes;  */ /*DOM-IGNORE-END*/


// *****************************************************************************
/* Ethernet Open Configuration Settings

  Summary:
    Supported open configuration flags for the Ethernet module (EthMACOpen).

  Description:
    This enumeration defines the various configuration options for the Ethernet
    module.  These values can be ORed together to create a configuration mask
    passed to the EthMACOpen routine.

  Remarks:
    When Auto-negotiation is specified:
        - If multiple capability flags are set (ETH_OPEN_FDUPLEX,
          ETH_OPEN_HDUPLEX, ETH_OPEN_100, ETH_OPEN_10 ) they are all advertised
          as this link side capabilities.
        - If no setting is passed, the lowest one is taken, i.e.,
          ETH_OPEN_HDUPLEX and ETH_OPEN_10.
        - Auto-MDIX requires Auto-Negotiation; ETH_OPEN_MDIX_NORM or
          ETH_OPEN_MDIX_SWAP setting irrelevant.

    When No Auto-negotiation is specified:
        - If multiple settings, the highest priority setting is taken, i.e.
          ETH_OPEN_FDUPLEX over ETH_OPEN_HDUPLEX and ETH_OPEN_100 over
          ETH_OPEN_10.
        - If no setting, the lowest setting is taken, i.e., ETH_OPEN_HDUPLEX and
          ETH_OPEN_10.
        - The MDIX is set based on the ETH_OPEN_MDIX_NORM/ETH_OPEN_MDIX_SWAP
          setting.
*/

typedef enum
{
    // Link capabilities flags:

    // Use auto negotiation. set the following flags to specify your choices
    ETH_OPEN_AUTO
        /*DOM-IGNORE-BEGIN*/ = 0x1 /*DOM-IGNORE-END*/,

    // Use full duplex or full duplex negotiation capability needed
    ETH_OPEN_FDUPLEX
        /*DOM-IGNORE-BEGIN*/ = 0x2 /*DOM-IGNORE-END*/,

    // Use half duplex or half duplex negotiation capability needed
    ETH_OPEN_HDUPLEX
        /*DOM-IGNORE-BEGIN*/ = 0x4 /*DOM-IGNORE-END*/,

    // Use 100MBps or 100MBps negotiation capability needed
    ETH_OPEN_100
        /*DOM-IGNORE-BEGIN*/ = 0x8 /*DOM-IGNORE-END*/,

    // Use 10MBps or 10MBps negotiation capability needed
    ETH_OPEN_10
        /*DOM-IGNORE-BEGIN*/ = 0x10 /*DOM-IGNORE-END*/,

    // Allow huge packets RX/TX
    ETH_OPEN_HUGE_PKTS
        /*DOM-IGNORE-BEGIN*/ = 0x20 /*DOM-IGNORE-END*/,

    // Loopbacked at the MAC level
    ETH_OPEN_MAC_LOOPBACK
        /*DOM-IGNORE-BEGIN*/ = 0x40 /*DOM-IGNORE-END*/,

    /*When PHY is loopbacked, negotiation will be disabled!*/
    ETH_OPEN_PHY_LOOPBACK
        /*DOM-IGNORE-BEGIN*/ = 0x80 /*DOM-IGNORE-END*/,

    // Use Auto MDIX
    ETH_OPEN_MDIX_AUTO
        /*DOM-IGNORE-BEGIN*/ = 0x100 /*DOM-IGNORE-END*/,

    // Use normal MDIX when Auto MDIX disabled
    ETH_OPEN_MDIX_NORM
        /*DOM-IGNORE-BEGIN*/ = 0x0 /*DOM-IGNORE-END*/,

    // Use swapped MDIX when Auto MDIX disabled
    ETH_OPEN_MDIX_SWAP
        /*DOM-IGNORE-BEGIN*/ = 0x200 /*DOM-IGNORE-END*/,

    // MII/RMII flags:

    // RMII connection
    ETH_OPEN_RMII
        /*DOM-IGNORE-BEGIN*/ = 0x400 /*DOM-IGNORE-END*/,

    // MII connection
    ETH_OPEN_MII
        /*DOM-IGNORE-BEGIN*/ = 0x000 /*DOM-IGNORE-END*/,


    // All capabilities default
    ETH_OPEN_DEFAULT = (ETH_OPEN_AUTO|ETH_OPEN_FDUPLEX|ETH_OPEN_HDUPLEX|
                        ETH_OPEN_100|ETH_OPEN_10|ETH_OPEN_MDIX_AUTO)

} ETH_OPEN_FLAGS; /*DOM-IGNORE-BEGIN*/ /* in eth.h, eEthOpenFlags;  */ /*DOM-IGNORE-END*/


// *****************************************************************************
/* Ethernet Close Flags

  Summary:
    Defines the possible disable codes of Ethernet controller "DRV_ETHMAC_LegacyClose" call.

  Description:
    This enumeration defines the close capabilities of the Ethernet module.
*/

typedef enum
{
    /*Wait for the current TX/RX operation to finish*/
    ETH_CLOSE_GRACEFUL  /*DOM-IGNORE-BEGIN*/ = 0x1 /*DOM-IGNORE-END*/,

    // Default close options
    ETH_CLOSE_DEFAULT = (0)

} ETH_CLOSE_FLAGS; /*DOM-IGNORE-BEGIN*/ /* in eth.h, eEthCloseFlags;  */ /*DOM-IGNORE-END*/


// *****************************************************************************
/* Ethernet MAC Pause Types

  Summary:
    Defines the possible Ethernet MAC pause types.

  Description:
    This enumeration defines the pause capabilities of the Ethernet MAC.
*/

typedef enum
{
    // No PAUSE capabilities
    ETH_MAC_PAUSE_TYPE_NONE     /*DOM-IGNORE-BEGIN*/ = 0x0 /*DOM-IGNORE-END*/,

    // Supports symmetric PAUSE
    ETH_MAC_PAUSE_TYPE_PAUSE    /*DOM-IGNORE-BEGIN*/ = 0x1 /*DOM-IGNORE-END*/,

    // Supports ASM_DIR
    ETH_MAC_PAUSE_TYPE_ASM_DIR  /*DOM-IGNORE-BEGIN*/ = 0x2 /*DOM-IGNORE-END*/,

    // The previous two values converted to TX/RX capabilities:

    // Enable MAC TX pause support
    ETH_MAC_PAUSE_TYPE_EN_TX    /*DOM-IGNORE-BEGIN*/ = 0x4 /*DOM-IGNORE-END*/,

    // Enable MAC RX pause support
    ETH_MAC_PAUSE_TYPE_EN_RX    /*DOM-IGNORE-BEGIN*/ = 0x8 /*DOM-IGNORE-END*/,

    // All types of pause
    ETH_MAC_PAUSE_ALL       = (ETH_MAC_PAUSE_TYPE_PAUSE|ETH_MAC_PAUSE_TYPE_ASM_DIR|
                           ETH_MAC_PAUSE_TYPE_EN_TX|ETH_MAC_PAUSE_TYPE_EN_RX),

    // All pause capabilities our MAC supports
    ETH_MAC_PAUSE_CPBL_MASK = ETH_MAC_PAUSE_ALL

} ETH_PAUSE_TYPE; /*DOM-IGNORE-BEGIN*/ /* in eth.h, eEthMacPauseType;  */ /*DOM-IGNORE-END*/


// *****************************************************************************
/* Ethernet Link Status Codes

  Summary:
    Defines the possible status flags of Ethernet link.

  Description:
    This enumeration defines the flags describing the status of the Ethernet
    link.

  Remarks:
    Multiple flags can be set.
*/

typedef enum
{
    // No connection to the LinkPartner
    ETH_LINK_ST_DOWN           /*DOM-IGNORE-BEGIN*/ = 0x0 /*DOM-IGNORE-END*/,

    // Link is up
    ETH_LINK_ST_UP             /*DOM-IGNORE-BEGIN*/ = 0x1 /*DOM-IGNORE-END*/,

    // LP non negotiation able
    ETH_LINK_ST_LP_NEG_UNABLE  /*DOM-IGNORE-BEGIN*/ = 0x2 /*DOM-IGNORE-END*/,

    // LP fault during negotiation
    ETH_LINK_ST_REMOTE_FAULT   /*DOM-IGNORE-BEGIN*/ = 0x4 /*DOM-IGNORE-END*/,

    // Parallel Detection Fault encountered (when ETH_LINK_ST_LP_NEG_UNABLE)
    ETH_LINK_ST_PDF            /*DOM-IGNORE-BEGIN*/ = 0x8 /*DOM-IGNORE-END*/,

    // LP supports symmetric pause
    ETH_LINK_ST_LP_PAUSE       /*DOM-IGNORE-BEGIN*/ = 0x10 /*DOM-IGNORE-END*/,

    // LP supports asymmetric TX/RX pause operation
    ETH_LINK_ST_LP_ASM_DIR     /*DOM-IGNORE-BEGIN*/ = 0x20 /*DOM-IGNORE-END*/,

    // LP not there
    ETH_LINK_ST_NEG_TMO        /*DOM-IGNORE-BEGIN*/ = 0x1000 /*DOM-IGNORE-END*/,

    // An unexpected fatal error occurred during the negotiation
    ETH_LINK_ST_NEG_FATAL_ERR  /*DOM-IGNORE-BEGIN*/ = 0x2000 /*DOM-IGNORE-END*/

} ETH_LINK_STATUS; /*DOM-IGNORE-BEGIN*/ /* in eth.h, eEthLinkStat;  */ /*DOM-IGNORE-END*/

#endif//_DRV_ETHERNET_FLAGS_H
