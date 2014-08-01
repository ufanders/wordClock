/*******************************************************************************
  National DP83640 PHY API for Microchip TCP/IP Stack
*******************************************************************************/

/*******************************************************************************
File Name:  ETHPIC32ExtPhyDP83640.c
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

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

#include <peripheral/ports.h>
#include "driver/ethphy/src/dynamic/drv_extphy.h"


#include "driver/ethphy/src/dynamic/drv_extphy_dp83640.h"

/****************************************************************************
 *                 interface functions
 ****************************************************************************/


/****************************************************************************
 * Function:        DRV_EXTPHY_MIIConfigure
 *
 * PreCondition:    - Communication to the PHY should have been established.
 *
 * Input:           handle - A valid open-instance handle, returned from the driver's open routine
 *                  cFlags - the requested open flags: ETH_PHY_CFG_RMII/ETH_PHY_CFG_MII
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
ETH_RESULT_CODE DRV_EXTPHY_MIIConfigure(DRV_HANDLE hClientObj, ETHPHY_CONFIG_FLAGS cFlags)
{
    unsigned short  phyReg;

    // RMII_BYPASS is in page 0
    DRV_ETHPHY_SMIWriteStart(hClientObj, PHY_REG_PAGESEL, 0);

    DRV_ETHPHY_SMIReadStart(hClientObj, PHY_REG_RMII_BYPASS);
    phyReg = DRV_ETHPHY_SMIReadResultGet(hClientObj);

    if(cFlags&ETH_PHY_CFG_RMII)
    {
        phyReg |= _RMIIBYPASS_RMII_MODE_MASK;
        phyReg &= ~_RMIIBYPASS_RMII_REV1_0_MASK;      // use RMII 1.2
    }
    else
    {
        phyReg &= ~(_RMIIBYPASS_RMII_MODE_MASK);  // MII
    }

    DRV_ETHPHY_SMIWriteStart(hClientObj, PHY_REG_RMII_BYPASS, phyReg);    // update the RMII and Bypass Register


    return ETH_RES_OK;

}


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
ETH_RESULT_CODE DRV_EXTPHY_MDIXConfigure(DRV_HANDLE hClientObj, ETH_OPEN_FLAGS oFlags)
{
    unsigned short  phyReg;

    // PHY_CTRL is in page 0
    DRV_ETHPHY_SMIWriteStart(hClientObj, PHY_REG_PAGESEL, 0);

    DRV_ETHPHY_SMIReadStart(hClientObj, PHY_REG_PHY_CTRL);
    phyReg = DRV_ETHPHY_SMIReadResultGet(hClientObj);

    if(oFlags&ETH_OPEN_MDIX_AUTO)
    {   // enable Auto-MDIX
        phyReg |= _PHYCTRL_MDIX_EN_MASK;
    }
    else
    {   // no Auto-MDIX
        phyReg &= ~(_PHYCTRL_MDIX_EN_MASK);   // disable Auto-MDIX
           if(oFlags&ETH_OPEN_MDIX_SWAP)
           {
               phyReg |= _PHYCTRL_FORCE_MDIX_MASK;    // swap
           }
           else
           {
               phyReg &= ~(_PHYCTRL_FORCE_MDIX_MASK); // normal
           }
    }

    DRV_ETHPHY_SMIWriteStart(hClientObj, PHY_REG_PHY_CTRL, phyReg);

    return ETH_RES_OK;

}

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
unsigned int DRV_EXTPHY_SMIClockGet(DRV_HANDLE handle)
{
    return 25000000;        //  25 MHz max clock supported
}


