/*******************************************************************************
  External Phy API header file

  Company:
    Microchip Technology Inc.
    
  File Name:
    drv_extphy.h

  Summary:
    External Phy API definitions

  Description:
    This file provides External Phy API definitions.
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

#ifndef _DRV_EXTPHY_H_
#define _DRV_EXTPHY_H_

#include "system_config.h"
#include "driver/driver_common.h"

#include "driver/ethphy/drv_ethphy.h"
#include "driver/ethphy/src/dynamic/drv_extphy.h"
#include "driver/ethphy/src/dynamic/drv_extphy_regs.h"

#include "driver/ethmac/drv_ethernet_flags.h"

#include "peripheral/eth/plib_eth.h"
#include "peripheral/eth/src/plib_eth_legacy.h"

// definitions

/****************************************************************************
 * Function:        DRV_EXTPHY_SMIAddressGet
 *
 * PreCondition:    None
 *
 * Input:           handle - A valid open-instance handle, returned from the driver's open routine
 *
 * Output:          PHY MIIM address
 *
 *
 * Side Effects:    None
 *
 * Overview:        This function returns the address the PHY uses for MIIM transactions
 *
 * Note:            None
 *****************************************************************************/

extern unsigned int     DRV_EXTPHY_SMIAddressGet(DRV_HANDLE handle);


/****************************************************************************
 * Function:        DRV_EXTPHY_SMIClockGet
 *
 * PreCondition:    None
 *
 * Input:           handle - A valid open-instance handle, returned from the driver's open routine
 *
 * Output:          PHY MIIM clock, Hz
 *
 *
 * Side Effects:    None
 *
 * Overview:        This function returns the maximum clock frequency that the PHY can use for the MIIM transactions
 *
 * Note:            None
 *****************************************************************************/
extern unsigned int     DRV_EXTPHY_SMIClockGet(DRV_HANDLE handle);


/****************************************************************************
 * Function:        DRV_EXTPHY_MIIConfigure
 *
 * PreCondition:    - Communication to the PHY should have been established.
 *
 * Input:           handle - A valid open-instance handle, returned from the driver's open routine
 *                  cFlags - the requested configuration flags: ETH_PHY_CFG_RMII/ETH_PHY_CFG_MII
 *
 * Output:          ETH_RES_OK - success,
 *                  an error code otherwise
 *
 *
 * Side Effects:    None
 *
 * Overview:        This function configures the PHY in one of MII/RMII operation modes.
 *
 * Note:            None
 *****************************************************************************/
extern ETH_RESULT_CODE      DRV_EXTPHY_MIIConfigure(DRV_HANDLE handle, ETHPHY_CONFIG_FLAGS cFlags);


/****************************************************************************
 * Function:        DRV_EXTPHY_MDIXConfigure
 *
 * PreCondition:    - Communication to the PHY should have been established.
 *
 * Input:           handle - A valid open-instance handle, returned from the driver's open routine
 *                  oFlags - the requested open flags: ETH_OPEN_MDIX_AUTO, ETH_OPEN_MDIX_NORM/ETH_OPEN_MDIX_SWAP
 *
 * Output:          ETH_RES_OK - success,
 *                  an error code otherwise
 *
 *
 * Side Effects:    None
 *
 * Overview:        This function configures the MDIX mode for the PHY.
 *
 * Note:            None
 *****************************************************************************/
extern ETH_RESULT_CODE      DRV_EXTPHY_MDIXConfigure(DRV_HANDLE handle, ETH_OPEN_FLAGS oFlags);

#ifdef PHY_WOL_ENABLE
/****************************************************************************
 * Function:        DRV_EXTPHY_WOLConfiguration
 *
 * PreCondition:    EthInit and EthPhyInit should have been called.
 *
 * Input:          hClientObj - A valid open-instance handle, returned from the driver's open routine
 *                    bAddr[] -  Source Mac Address , or a Magic Packet MAC address
 *
 * Output:         none
 *
 *
 * Side Effects:    None
 *
 * Overview:       Configure WOL for External PHY with a Source MAC address
 *                 or a 6 byte magic packet mac address.
 *
 * Note:            None
 *****************************************************************************/
extern void  DRV_EXTPHY_WOLConfiguration(DRV_HANDLE hClientObj,unsigned char bAddr[]);
#endif /* PHY_WOL_ENABLE */

#endif  // _DRV_EXTPHY_H_

