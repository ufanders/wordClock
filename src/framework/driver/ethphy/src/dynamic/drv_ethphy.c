/*******************************************************************************
  Ethernet PHY Device Driver Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ethphy.c

  Summary:
    Ethernet PHY Device Driver Dynamic Implementation

  Description:
    The Ethernet PHY device driver provides a simple interface to manage the
    Ethernet PHY modules on Microchip microcontrollers.  This file Implements
    the core interface routines for the Ethernet PHY driver.

    While building the driver from source, ALWAYS use this file in the build.
*******************************************************************************/

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


// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include "driver/ethphy/src/drv_ethphy_local.h"
#include "system_config.h"
#if !defined (PORT_DEBUG)
#include "driver/ethphy/src/dynamic/drv_ethphy_port_mappings.h"
#endif

#include "system/clk/sys_clk.h"
#include "system/tmr/sys_tmr.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Variables
// *****************************************************************************
// *****************************************************************************

/* This is a temporary workaround for an issue with the TCP/IP stack initialization.
   Once that issue has been resolved, these definitions should be removed. */
//
// resolution/granularity of time base
#define DRV_ETHPHY_TMR_RESOLUTION       128
// increment step per call
#define DRV_ETHPHY_TMR_STEP             1
// local timer counter
static uint32_t _drv_ethphy_tmr_count = 0;
// redirect
#define SYS_TMR_TickCountGet()      DRV_ETHPHY_TMR_TickCountGet() 
static uint32_t DRV_ETHPHY_TMR_TickCountGet(void)
{
    _drv_ethphy_tmr_count += DRV_ETHPHY_TMR_STEP;
    return _drv_ethphy_tmr_count / DRV_ETHPHY_TMR_RESOLUTION;
}



// Local Definitions
//
#define PROT_802_3  0x01    // IEEE 802.3 capability
#define MAC_COMM_CPBL_MASK  (_BMSTAT_BASE10T_HDX_MASK|_BMSTAT_BASE10T_FDX_MASK|_BMSTAT_BASE100TX_HDX_MASK|_BMSTAT_BASE100TX_FDX_MASK)
// all comm capabilities our MAC supports

// *****************************************************************************
/* Function:        mRegIxPhyAddToEMACxMADR

   PreCondition:    rIx a valid PHY register, 0-31
                    phyAdd a valid PHY address, 0-31

   Input:           rIx:    PHY register to be accessed
                    phyAdd: PHY to be accessed

   Output:          None

   Side Effects:    None

   Overview:        This macro converts a register index and PHY address to a EMACxMADR format;

   Note:            None
*/
// From eth_miim_access.c:
#define  mRegIxPhyAddToEMACxMADR(rIx, phyAdd)  ((rIx)<<_EMACxMADR_REGADDR_POSITION)|((phyAdd)<<_EMACxMADR_PHYADDR_POSITION)


// From eth_miim_access.c:
static const short _MIIClockDivisorTable[]=
{
    4, 6, 8, 10, 14, 20, 28, 40, 
#if defined (__PIC32MZ__)
    48, 50,
#endif  // defined (__PIC32MZ__)
};  // divider values for the Host clock

// local inlined functions
//

static ETH_LINK_STATUS _Phy2LinkStat(__BMSTATbits_t phyStat)
{
    ETH_LINK_STATUS  linkStat;

    linkStat = (phyStat.LINK_STAT)?ETH_LINK_ST_UP:ETH_LINK_ST_DOWN;
    if(phyStat.REM_FAULT)
    {
        linkStat|=ETH_LINK_ST_REMOTE_FAULT;
    }

    return linkStat;

} /* _Phy2LinkStat */

static inline unsigned short _PhyReadReg( DRV_HANDLE hClientObj, unsigned int rIx)
{
    DRV_ETHPHY_SMIReadStart(hClientObj, rIx);
    return DRV_ETHPHY_SMIReadResultGet(hClientObj);
} /* _PhyReadReg */


// *****************************************************************************
/* Driver Hardware instance objects.

  Summary:
    Defines the hardware instances objects that are available on the part

  Description:
    This data type defines the hardware instance objects that are available on
    the part, so as to capture the hardware state of the instance.

  Remarks:
    Not all modes are available on all micro-controllers.
*/

DRV_ETHPHY_OBJ              gDrvETHPHYObj[DRV_ETHPHY_INSTANCES_NUMBER];


// *****************************************************************************
/* Driver Client instance objects.

  Summary:
    Defines the Client instances objects that are available on the part

  Description:
    This data type defines the Client instance objects that are available on
    the part, so as to capture the Client state of the instance.

  Remarks:
    None
*/

DRV_ETHPHY_CLIENT_OBJ       gDrvETHPHYClientObj[DRV_ETHPHY_CLIENTS_NUMBER * DRV_ETHPHY_INSTANCES_NUMBER];


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************

/****************************************************************************
 * Function:        _PhyInitIo
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Helper to properly set the ethernet i/o pins to digital pins.
 *
 * Note:            Even when the ethernet device is turned on the analog shared pins have to be configured.
 *****************************************************************************/
#if !defined (PORT_DEBUG)
#if defined (__PIC32MX__)
static void _PhyInitIo(DRV_HANDLE hClientObj)
{
    __DEVCFG3bits_t bcfg3;

    bcfg3=DEVCFG3bits;
    if(bcfg3.FETHIO)
    {   // default setting, both RMII and MII
        PORTSetPinsDigitalOut(_ETH_MDC_PORT, _ETH_MDC_BIT);
        PORTSetPinsDigitalIn(_ETH_MDIO_PORT, _ETH_MDIO_BIT);

        PORTSetPinsDigitalOut(_ETH_TXEN_PORT, _ETH_TXEN_BIT);
        PORTSetPinsDigitalOut(_ETH_TXD0_PORT, _ETH_TXD0_BIT);
        PORTSetPinsDigitalOut(_ETH_TXD1_PORT, _ETH_TXD1_BIT);


        PORTSetPinsDigitalIn(_ETH_RXCLK_PORT, _ETH_RXCLK_BIT);
        PORTSetPinsDigitalIn(_ETH_RXDV_PORT, _ETH_RXDV_BIT);
        PORTSetPinsDigitalIn(_ETH_RXD0_PORT, _ETH_RXD0_BIT);
        PORTSetPinsDigitalIn(_ETH_RXD1_PORT, _ETH_RXD1_BIT);
        PORTSetPinsDigitalIn(_ETH_RXERR_PORT, _ETH_RXERR_BIT);


        if(bcfg3.FMIIEN)
        {   // just MII
            PORTSetPinsDigitalIn(_ETH_TXCLK_PORT, _ETH_TXCLK_BIT);
            PORTSetPinsDigitalOut(_ETH_TXD2_PORT, _ETH_TXD2_BIT);
            PORTSetPinsDigitalOut(_ETH_TXD3_PORT, _ETH_TXD3_BIT);
            PORTSetPinsDigitalOut(_ETH_TXERR_PORT, _ETH_TXERR_BIT);

            PORTSetPinsDigitalIn(_ETH_RXD2_PORT, _ETH_RXD2_BIT);
            PORTSetPinsDigitalIn(_ETH_RXD3_PORT, _ETH_RXD3_BIT);
            PORTSetPinsDigitalIn(_ETH_CRS_PORT, _ETH_CRS_BIT);
            PORTSetPinsDigitalIn(_ETH_COL_PORT, _ETH_COL_BIT);
        }
    }
    else
    {   // alternate setting, both RMII and MII
        PORTSetPinsDigitalOut(_ETH_ALT_MDC_PORT, _ETH_ALT_MDC_BIT);
        PORTSetPinsDigitalIn(_ETH_ALT_MDIO_PORT, _ETH_ALT_MDIO_BIT);

        PORTSetPinsDigitalOut(_ETH_ALT_TXEN_PORT, _ETH_ALT_TXEN_BIT);
        PORTSetPinsDigitalOut(_ETH_ALT_TXD0_PORT, _ETH_ALT_TXD0_BIT);
        PORTSetPinsDigitalOut(_ETH_ALT_TXD1_PORT, _ETH_ALT_TXD1_BIT);


        PORTSetPinsDigitalIn(_ETH_ALT_RXCLK_PORT, _ETH_ALT_RXCLK_BIT);
        PORTSetPinsDigitalIn(_ETH_ALT_RXDV_PORT, _ETH_ALT_RXDV_BIT);
        PORTSetPinsDigitalIn(_ETH_ALT_RXD0_PORT, _ETH_ALT_RXD0_BIT);
        PORTSetPinsDigitalIn(_ETH_ALT_RXD1_PORT, _ETH_ALT_RXD1_BIT);
        PORTSetPinsDigitalIn(_ETH_ALT_RXERR_PORT, _ETH_ALT_RXERR_BIT);


        if(bcfg3.FMIIEN)
        {   // just MII
            PORTSetPinsDigitalIn(_ETH_ALT_TXCLK_PORT, _ETH_ALT_TXCLK_BIT);
            PORTSetPinsDigitalOut(_ETH_ALT_TXD2_PORT, _ETH_ALT_TXD2_BIT);
            PORTSetPinsDigitalOut(_ETH_ALT_TXD3_PORT, _ETH_ALT_TXD3_BIT);
            PORTSetPinsDigitalOut(_ETH_ALT_TXERR_PORT, _ETH_ALT_TXERR_BIT);

            PORTSetPinsDigitalIn(_ETH_ALT_RXD2_PORT, _ETH_ALT_RXD2_BIT);
            PORTSetPinsDigitalIn(_ETH_ALT_RXD3_PORT, _ETH_ALT_RXD3_BIT);
            PORTSetPinsDigitalIn(_ETH_ALT_CRS_PORT, _ETH_ALT_CRS_BIT);
            PORTSetPinsDigitalIn(_ETH_ALT_COL_PORT, _ETH_ALT_COL_BIT);
        }

    }
}
#endif  // defined (__PIC32MX__)
#endif


/****************************************************************************
 * Function:        _PhyDetectReset
 *
 * PreCondition:    DRV_ETHPHY_SMIClockSet() should have been called
 *
 * Input:           handle - valid open-isntance handle, returned from driver's open routine.
 *
 * Output:          true if the detection and the reset of the PHY succeeded,
 *                  false if no PHY detected
 *
 * Side Effects:    None
 *
 * Overview:        This function detects and resets the PHY.
 *
 * Note:            Needs the system running frequency to for the PHY detection
 *****************************************************************************/
static int _PhyDetectReset(DRV_HANDLE hClientObj)
{
    __BMCONbits_t bmcon;
    unsigned int  tWaitReset;

    bmcon.w = _PhyReadReg(hClientObj, PHY_REG_BMCON);        // read the BMCON register

    if(bmcon.RESET)
    {   // that is already suspicios...but give it a chance to clear itself
        tWaitReset = SYS_TMR_TickCountGet() + ((PHY_RESET_CLR_TMO * SYS_TMR_TickPerSecond()) + 999)/1000;
        do
        {
            bmcon.w = _PhyReadReg(hClientObj, PHY_REG_BMCON);
        }while(bmcon.RESET && SYS_TMR_TickCountGet() < tWaitReset); // wait reset self clear

        bmcon.w = _PhyReadReg(hClientObj, PHY_REG_BMCON);
        if(bmcon.RESET)
        {   // tmo clearing the reset
            return 0;
        }
    }//end if(bmcon.RESET)

    // ok, reset bit is low
    // try to see if we can write smth to the PHY
    // we use Loopback and Isolate bits
    DRV_ETHPHY_SMIWriteStart(hClientObj, PHY_REG_BMCON, _BMCON_LOOPBACK_MASK|_BMCON_ISOLATE_MASK);    // write control bits
    bmcon.w = _PhyReadReg(hClientObj, PHY_REG_BMCON);        // read back
    if( (bmcon.LOOPBACK == 0) || (bmcon.ISOLATE == 0) )
    {   // failed to set
        return 0;
    }
    bmcon.w ^=_BMCON_LOOPBACK_MASK|_BMCON_ISOLATE_MASK;
    DRV_ETHPHY_SMIWriteStart(hClientObj, PHY_REG_BMCON, bmcon.w);     // clear bits and write
    bmcon.w = _PhyReadReg(hClientObj, PHY_REG_BMCON);        // read back
    if(bmcon.LOOPBACK || bmcon.ISOLATE)
    {  // failed to clear
        return 0;
    }

    // everything seems to be fine
    return DRV_ETHPHY_Reset(hClientObj,true);

} /* _PhyDetectReset */


DRV_ETHPHY_CLIENT_OBJ * _DRV_ETHPHY_ClientObjectAllocate( SYS_MODULE_INDEX drvIndex )
{
    /* Local Variables */
    uint16_t i, iClientObj;

    for (i = 0; i < DRV_ETHPHY_CLIENTS_NUMBER ; i++)
    {
        iClientObj = ( i * drvIndex );
        // Return the matching index associated the hardware instance.
        if ( gDrvETHPHYClientObj[iClientObj].inUse == false )
        {
            return gDrvETHPHYClientObj + iClientObj;
        }
    }
    return NULL;
} /* _DRV_ETHPHY_ClientObjectAllocate */



// *****************************************************************************
// *****************************************************************************
// Section: Driver Interface Function Definitions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    SYS_OBJ_HANDLE DRV_ETHPHY_Initialize( const SYS_MODULE_INDEX    drvIndex,
                                          const SYS_MODULE_INIT     * const init )

  Summary:
    Initializes hardware and data for the given instance of the ETHPHY module

  Description:
    This routine initializes hardware for the instance of the ETHPHY module,
    using the hardware initialization given data.  It also initializes all
    necessary internal data.

  Parameters:
    drvIndex        - Identifies the driver instance to be initialized
    init            - Pointer to the data structure containing all data
                      necessary to initialize the hardware. This pointer may
                      be null if no data is required and static initialization
                      values are to be used.

  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, it returns SYS_OBJ_HANDLE_INVALID.
*/

SYS_OBJ_HANDLE DRV_ETHPHY_Initialize( const SYS_MODULE_INDEX  iModule,
                                      const SYS_MODULE_INIT   * const init )
{
    DRV_ETHPHY_OBJ * hSysObj;
    DRV_ETHPHY_INIT *ethphyInit = NULL;

    if ( iModule >= DRV_ETHPHY_INSTANCES_NUMBER ||
         gDrvETHPHYObj[iModule].inUse == true )
    {
        return SYS_OBJ_HANDLE_INVALID;
    }

    hSysObj = gDrvETHPHYObj + iModule;

    /* Assign to the local pointer the init data passed */
    ethphyInit = ( DRV_ETHPHY_INIT * ) init;

    hSysObj->inUse = true;      // Set object to be in use
    hSysObj->numClients = 0;
    hSysObj->status = SYS_STATUS_READY; // Set module state
    hSysObj->iModule  = iModule;  // Store driver instance
    hSysObj->ethphyId = ethphyInit->ethphyId; // Store PLIB ID

    // Assign External PHY Service Functions
    hSysObj->sExtPHYFunctions.MyPHYMIIConfigure  = ethphyInit->MyPHYMIIConfigure ;
    hSysObj->sExtPHYFunctions.MyPHYMDIXConfigure = ethphyInit->MyPHYMDIXConfigure;
    hSysObj->sExtPHYFunctions.MyPHYSMIClockGet   = ethphyInit->MyPHYSMIClockGet  ;

    /* Return the driver handle */
    return ( SYS_OBJ_HANDLE )hSysObj ;

} /* DRV_ETHPHY_Initialize */


//******************************************************************************
/* Function:
    void DRV_ETHPHY_Reinitialize( SYS_OBJ_HANDLE        object,
                                  const SYS_MODULE_INIT * const init )

  Summary:
    Reinitializes and refreshes the hardware for the instance of the ETHPHY
    module

  Description:
    This routine reinitializes and refreshes the hardware for the instance
    of the ETHPHY module using the hardware initialization given data.
    It does not clear or reinitialize internal data structures

  Parameters:
    object          - Identifies the Driver Object returned by the Initialize
                      interface
    init            - Pointer to the data structure containing any data
                      necessary to initialize the hardware.

  Returns:
    None
*/

void DRV_ETHPHY_Reinitialize( SYS_OBJ_HANDLE        object ,
                              const SYS_MODULE_INIT * const init )
{
    DRV_ETHPHY_OBJ * hSysObj = (DRV_ETHPHY_OBJ *) object ;
    DRV_ETHPHY_INIT * ethphyInit = (DRV_ETHPHY_INIT *)init;

    /* Check for the valid driver object passed */
    SYS_ASSERT( NULL != hSysObj, "Hardware Object is invalid" );

    hSysObj->numClients = 0;
    hSysObj->status = SYS_STATUS_READY; // Set module state
    hSysObj->ethphyId = ethphyInit->ethphyId; // Store PLIB ID

    // Assign External PHY Service Functions
    hSysObj->sExtPHYFunctions.MyPHYMIIConfigure  = ethphyInit->MyPHYMIIConfigure ;
    hSysObj->sExtPHYFunctions.MyPHYMDIXConfigure = ethphyInit->MyPHYMDIXConfigure;
    hSysObj->sExtPHYFunctions.MyPHYSMIClockGet   = ethphyInit->MyPHYSMIClockGet  ;

} /* DRV_ETHPHY_Reinitialize */


//******************************************************************************
/* Function:
    void DRV_ETHPHY_Deinitialize( SYS_OBJ_HANDLE object )

  Summary:
    Deinitializes the specific module instance of the ETHPHY module

  Description:
    Deinitializes the specific module instancedisabling its operation (and any
    hardware for driver modules).  Resets all the internal data structures and
    fields for the specified instance to the default settings.

  Parameters:
    object          - Identifies the Driver Object returned by the Initialize
                      interface

  Returns:
    None
*/

void DRV_ETHPHY_Deinitialize( SYS_OBJ_HANDLE hSysObj )
{
    DRV_ETHPHY_OBJ * hObj = (DRV_ETHPHY_OBJ *) hSysObj ;

    /* Check for the valid driver object passed */
    SYS_ASSERT( NULL != hObj, "Hardware Object is invalid" );

    /* Set the Device Status */
    hObj->status  = SYS_STATUS_UNINITIALIZED;

    /* Remove the driver usage */
    hObj->inUse  = false;

} /* DRV_ETHPHY_Deinitialize */


//******************************************************************************
/* Function:
    SYS_STATUS DRV_ETHPHY_Status( SYS_OBJ_HANDLE object )

  Summary:
    Provides the current status of the hardware instance of the ETHPHY module

  Description:
    This routine Provides the current status of the hardware instance of the
    ETHPHY module.

  Parameters:
    object          - Identifies the Driver Object returned by the Initialize
                      interface

  Returns:
    SYS_STATUS_READY    Indicates that any previous module operation for the
                        specified module has completed

    SYS_STATUS_BUSY     Indicates that a previous module operation for the
                        specified module has not yet completed

    SYS_STATUS_ERROR    Indicates that the specified module is in an error state
*/

SYS_STATUS DRV_ETHPHY_Status( SYS_OBJ_HANDLE hSysObj )
{
    DRV_ETHPHY_OBJ * hObj = (DRV_ETHPHY_OBJ *) hSysObj ;

    /* Check for the valid driver object passed */
    SYS_ASSERT( NULL != hObj, "Hardware Object is invalid" );

    /* Return the status associated with the driver handle */
    return( hObj->status ) ;

} /* DRV_ETHPHY_Status */


//******************************************************************************
/* Function:
    void DRV_ETHPHY_Tasks( SYS_OBJ_HANDLE object)

  Summary:
    Used to maintain the driver's state machine

  Description:
    This routine is used to maintain the driver's internal state machine.

  Parameters:
    object          - Identifies the Driver Object returned by the Initialize
                      interface

  Returns:
    None
*/

void DRV_ETHPHY_Tasks( SYS_OBJ_HANDLE hSysObj )
{
    DRV_ETHPHY_OBJ * hObj = (DRV_ETHPHY_OBJ *) hSysObj ;

    /* Check for the valid driver object passed */
    SYS_ASSERT( NULL !=  hObj, "Hardware Object is invalid" );

} /* DRV_ETHPHY_Tasks */


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client Level
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    DRV_HANDLE DRV_ETHPHY_Open( const SYS_MODULE_INDEX  drvIndex,
                                const DRV_IO_INTENT     ioIntent )

  Summary:
    Opens the specific module instance and returns a handle

  Description:
    This routine opens a driver for use by any client module and provides a
    handle that must be provided to any of the other driver operations to
    identify the caller and the instance of the driver/hardware module.

  Parameters:
    drvIndex        - Identifier for the instance to be initialized
    ioIntent        - Possible values from the enumeration DRV_IO_INTENT

  Returns:
    If successful, the routine returns a valid open-instance handle (a number
    identifying both the caller and the module instance)
    If an error occurs, the return value is DRV_HANDLE_INVALID
*/

DRV_HANDLE  DRV_ETHPHY_Open ( const SYS_MODULE_INDEX iModule,
                              const DRV_IO_INTENT ioIntent )
{
    /* Multi client variables are removed from single client builds. */
    DRV_ETHPHY_CLIENT_OBJ * hClientObj;

    /* Validate the driver index */
    /* If there is anything specific to the module & needs to be checked, should
       be handled in this section with an || condition.
       May be something like ioIntent test for Exclusive access */
    if ( iModule >= DRV_ETHPHY_INDEX_COUNT )
    {
        return DRV_HANDLE_INVALID;
    }

    /* Setup client operations */

    /* To Do: OSAL - Lock Mutex */

    /* Allocate the client object and set the flag as in use */
    hClientObj = _DRV_ETHPHY_ClientObjectAllocate( iModule ) ;

    hClientObj->inUse    = true;
    hClientObj->hDriver  = gDrvETHPHYObj + iModule;
    hClientObj->ethphyId = hClientObj->ethphyId;

    /* To Do: OSAL - Unlock Mutex */

    /* Update the Client Status */
    hClientObj->status = DRV_ETHPHY_CLIENT_STATUS_READY;

    /* Return the client object */
    return ( ( DRV_HANDLE ) hClientObj );

} /* DRV_ETHPHY_Open */


//******************************************************************************
/* Function:
    void DRV_ETHPHY_Close( DRV_HANDLE handle )

  Summary:
    Closes an opened-instance of a driver

  Description:
    This routine closes an opened-instance of a driver, invalidating the given
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None
*/

void DRV_ETHPHY_Close( DRV_HANDLE handle )
{
    DRV_ETHPHY_CLIENT_OBJ * hClientObj = (DRV_ETHPHY_CLIENT_OBJ *)handle;

    /* Check for the Client validity */
    SYS_ASSERT( NULL != hClientObj, "Invalid Client Object" ) ;

    /* To Do: OSAL - lock Mutex */

    /* Free the Client Instance */
    hClientObj->inUse = false ;

    /* To Do: OSAL - unlock Mutex */

    /* Update the Client Status */
    hClientObj->status = DRV_ETHPHY_CLIENT_STATUS_CLOSED;

} /* DRV_ETHPHY_Close */


// *****************************************************************************
/* Function:
    ETH_RESULT_CODE DRV_ETHPHY_Setup( DRV_HANDLE handle,
                                      int phyAddress,
                                      ETH_OPEN_FLAGS   oFlags,
                                      ETHPHY_CONFIG_FLAGS cFlags,
                                      ETH_OPEN_FLAGS*  pResFlags )

  Summary:
    Sets Up the ETHPHY's communication.

  Description:
    This function sets up the ETHPHY communication. It tries to detect the
    external ETHPHY, to read the capabilties and find a match with the requested
    features.Then it programs the ETHPHY accordingly.

  Precondition:

  Parameters:
    handle   - A valid open-instance handle, returned from the driver's open routine
    oFlags   - the requested open flags
    cFlags   - ETHPHY MII/RMII configuration flags
    pResFlags - address to store the initialization results

  Returns:
    ETH_RES_OK for success, an error code otherwise.
*/
/*DOM-IGNORE-BEGIN
Replaces:
eEthRes __attribute__((weak)) EthPhyInit(eEthOpenFlags oFlags, eEthPhyCfgFlags cFlags, eEthOpenFlags* pResFlags)
  DOM-IGNORE-END*/

ETH_RESULT_CODE __attribute__((weak))
                DRV_ETHPHY_Setup( DRV_HANDLE hDriverClient,
                                  int phyAddress,
                                  ETH_OPEN_FLAGS   oFlags,
                                  ETHPHY_CONFIG_FLAGS cFlags,
                                  ETH_OPEN_FLAGS*  pResFlags )
{
    DRV_ETHPHY_CLIENT_OBJ * hClientObj = (DRV_ETHPHY_CLIENT_OBJ *) hDriverClient;
    DRV_ETHPHY_OBJ * hDriver;
    unsigned short  ctrlReg;
    ETHPHY_CONFIG_FLAGS hwFlags, swFlags;
    unsigned short  phyCpbl, openReqs, matchCpbl;
    ETH_RESULT_CODE res;

    DRV_ETHPHY_MIICONFIGURE   MyPHYMIIConfigure;
    DRV_ETHPHY_MDIXCONFIGURE  MyPHYMDIXConfigure;
    DRV_ETHPHY_SMICLOCKGET    MyPHYSMIClockGet;

    /* Check for the Client validity */
    SYS_ASSERT( NULL != hClientObj, "Invalid Client Object" ) ;
    hDriver = hClientObj->hDriver;

    // the way the hw is configured
    hwFlags = DRV_ETHPHY_HWConfigFlagsGet(hDriverClient);

    if(cFlags&ETH_PHY_CFG_AUTO)
    {
        cFlags=hwFlags;
    }
    else
    {   // some minimal check against the way the hw is configured
        swFlags = cFlags&(ETH_PHY_CFG_RMII|ETH_PHY_CFG_ALTERNATE);

        if((swFlags ^ hwFlags)!=0)
        {   // hw-sw configuration mismatch MII/RMII, ALT/DEF config
            return ETH_RES_CFG_ERR;
        }
    }

    // Set the PHY SMI address
    hDriver->phyAddress = phyAddress;

    if(oFlags&(ETH_OPEN_PHY_LOOPBACK|ETH_OPEN_MAC_LOOPBACK))
    {
        oFlags &= ~ETH_OPEN_AUTO; // no negotiation in loopback mode!
    }

    if(!(oFlags&ETH_OPEN_AUTO))
    {
        oFlags &= ~ETH_OPEN_MDIX_AUTO;        // Auto-MDIX has to be in auto negotiation only
    }

    oFlags |= (cFlags&ETH_PHY_CFG_RMII)?ETH_OPEN_RMII:ETH_OPEN_MII;

#if !defined (PORT_DEBUG)
#if defined (__PIC32MX__)
    _PhyInitIo(hDriverClient);   // init IO pins
#endif
#endif


    // Set SMI clock
    MyPHYSMIClockGet = hDriver->sExtPHYFunctions.MyPHYSMIClockGet;
    if ( MyPHYSMIClockGet == NULL )
    {
        return ETH_RES_CPBL_ERR;
    }
    else
    {
#if defined (__PIC32MZ__)
        DRV_ETHPHY_SMIClockSet(hDriverClient, SYS_CLK_PeripheralFrequencyGet(CLK_BUS_PERIPHERAL_5), (*MyPHYSMIClockGet)(hDriverClient) );
#else
        DRV_ETHPHY_SMIClockSet(hDriverClient, SYS_CLK_SystemFrequencyGet(), (*MyPHYSMIClockGet)(hDriverClient) );
#endif
    }

    // try to detect the PHY and reset it
    if(!_PhyDetectReset(hDriverClient))
    {   // failed to detect the PHY
        return ETH_RES_DTCT_ERR;
    }

    // provide some defaults
    if(!(oFlags&(ETH_OPEN_FDUPLEX|ETH_OPEN_HDUPLEX)))
    {
        oFlags |= ETH_OPEN_HDUPLEX;
    }
    if(!(oFlags&(ETH_OPEN_100|ETH_OPEN_10)))
    {
        oFlags |= ETH_OPEN_10;
    }

    if(oFlags&ETH_OPEN_AUTO)
    {   // advertise auto negotiation
        openReqs = _BMSTAT_AN_ABLE_MASK;

        if(oFlags&ETH_OPEN_100)
        {
            if(oFlags&ETH_OPEN_FDUPLEX)
            {
                openReqs |= _BMSTAT_BASE100TX_FDX_MASK;
            }
            if(oFlags&ETH_OPEN_HDUPLEX)
            {
                openReqs |= _BMSTAT_BASE100TX_HDX_MASK;
            }
        }

        if(oFlags&ETH_OPEN_10)
        {
            if(oFlags&ETH_OPEN_FDUPLEX)
            {
                openReqs |= _BMSTAT_BASE10T_FDX_MASK;
            }
            if(oFlags&ETH_OPEN_HDUPLEX)
            {
                openReqs |= _BMSTAT_BASE10T_HDX_MASK;
            }
        }
    }
    else
    {   // no auto negotiation
        if(oFlags&ETH_OPEN_100)
        {
            openReqs = (oFlags&ETH_OPEN_FDUPLEX)?_BMSTAT_BASE100TX_FDX_MASK:_BMSTAT_BASE100TX_HDX_MASK;
        }
        else
        {
            openReqs = (oFlags&ETH_OPEN_FDUPLEX)?_BMSTAT_BASE10T_FDX_MASK:_BMSTAT_BASE10T_HDX_MASK;
        }
    }

    // try to match the oFlags with the PHY capabilities
    phyCpbl   = _PhyReadReg(hDriverClient, PHY_REG_BMSTAT);
    matchCpbl = (openReqs&(MAC_COMM_CPBL_MASK|_BMSTAT_AN_ABLE_MASK))&phyCpbl; // common features
    if(!(matchCpbl&MAC_COMM_CPBL_MASK))
    {   // no match?
        return ETH_RES_CPBL_ERR;
    }

    // we're ok, we can configure the PHY
    MyPHYMIIConfigure = hDriver->sExtPHYFunctions.MyPHYMIIConfigure;
    if ( MyPHYMIIConfigure == NULL )
    {
        return ETH_RES_CPBL_ERR;
    }
    else
    {
        res = (*MyPHYMIIConfigure)(hDriverClient,cFlags);
        if(res != ETH_RES_OK)
        {
            return res;
        }
    }

    MyPHYMDIXConfigure = hDriver->sExtPHYFunctions.MyPHYMDIXConfigure;
    if ( MyPHYMDIXConfigure == NULL )
    {
        return ETH_RES_CPBL_ERR;
    }
    else
    {
        res = (*MyPHYMDIXConfigure)(hDriverClient,oFlags);
        if(res != ETH_RES_OK)
        {
            return res;
        }
    }

    if(matchCpbl&_BMSTAT_AN_ABLE_MASK)
    {   // ok, we can perform auto negotiation
        unsigned short  anadReg;

        anadReg = (((matchCpbl>>_BMSTAT_NEGOTIATION_POS)<<_ANAD_NEGOTIATION_POS)&_ANAD_NEGOTIATION_MASK)|PROT_802_3;
        if(ETH_MAC_PAUSE_CPBL_MASK & ETH_MAC_PAUSE_TYPE_PAUSE)
        {
            anadReg |= _ANAD_PAUSE_MASK;
        }
        if(ETH_MAC_PAUSE_CPBL_MASK& ETH_MAC_PAUSE_TYPE_ASM_DIR)
        {
            anadReg |= _ANAD_ASM_DIR_MASK;
        }

        DRV_ETHPHY_SMIWriteStart(hDriverClient, PHY_REG_ANAD, anadReg);      // advertise our capabilities

        DRV_ETHPHY_RestartNegotiation(hDriverClient); // restart negotiation and we'll have to wait
    }
    else
    {   // ok, just don't use negotiation

        ctrlReg = 0;
        if(matchCpbl&(_BMSTAT_BASE100TX_HDX_MASK|_BMSTAT_BASE100TX_FDX_MASK))   // set 100Mbps request/capability
        {
            ctrlReg |= _BMCON_SPEED_MASK;
        }

        if(matchCpbl&(_BMSTAT_BASE10T_FDX_MASK|_BMSTAT_BASE100TX_FDX_MASK))
        {
            ctrlReg |= _BMCON_DUPLEX_MASK;
        }

        if(oFlags&ETH_OPEN_PHY_LOOPBACK)
        {
            ctrlReg |= _BMCON_LOOPBACK_MASK;
        }

        DRV_ETHPHY_SMIWriteStart(hDriverClient, PHY_REG_BMCON, ctrlReg); // update the configuration
    }


    // now update the open flags
    // the upper layer needs to know the PHY set-up to further set-up the MAC.

    // clear the capabilities
    oFlags &= ~(ETH_OPEN_AUTO|ETH_OPEN_FDUPLEX|ETH_OPEN_HDUPLEX|ETH_OPEN_100|ETH_OPEN_10);

    if(matchCpbl&_BMSTAT_AN_ABLE_MASK)
    {
        oFlags |= ETH_OPEN_AUTO;
    }
    if(matchCpbl&(_BMSTAT_BASE100TX_HDX_MASK|_BMSTAT_BASE100TX_FDX_MASK))   // set 100Mbps request/capability
    {
        oFlags |= ETH_OPEN_100;
    }
    if(matchCpbl&(_BMSTAT_BASE10T_HDX_MASK|_BMSTAT_BASE10T_FDX_MASK))   // set 10Mbps request/capability
    {
        oFlags |= ETH_OPEN_10;
    }
    if(matchCpbl&(_BMSTAT_BASE10T_FDX_MASK|_BMSTAT_BASE100TX_FDX_MASK))
    {
        oFlags |= ETH_OPEN_FDUPLEX;
    }
    if(matchCpbl&(_BMSTAT_BASE10T_HDX_MASK|_BMSTAT_BASE100TX_HDX_MASK))
    {
        oFlags |= ETH_OPEN_HDUPLEX;
    }

    *pResFlags = oFlags;      // upper layer needs to know the PHY set-up to further set-up the MAC.

    return ETH_RES_OK;

} /* DRV_ETHPHY_Setup */


//******************************************************************************
/* Function:
    DRV_ETHPHY_CLIENT_STATUS DRV_ETHPHY_ClientStatus(DRV_HANDLE handle)

  Summary:
    Gets the status of the module instance associated with the handle

  Description:
    This routine gets the status of the module instance associated with the
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    DRV_ETHPHY_CLIENT_STATUS value describing the current status of the driver
*/
/* DOM-IGNORE-BEGIN
No corresponding routine
   DOM-IGNORE-END */

DRV_ETHPHY_CLIENT_STATUS DRV_ETHPHY_ClientStatus( DRV_HANDLE handle )
{
    DRV_ETHPHY_CLIENT_OBJ * hClientObj = (DRV_ETHPHY_CLIENT_OBJ *) handle;

    /* Check for the Client validity */
    SYS_ASSERT( NULL != hClientObj, "Invalid Client Object" );

    /* Return the client status associated with the handle passed */
    return( hClientObj->status );

} /* DRV_ETHPHY_ClientStatus */


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - SMI/MIIM Interface
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void DRV_ETHPHY_SMIReadStart( DRV_HANDLE handle,
                                  unsigned int rIx);
  Summary:
    Initiates SMI/MIIM read transaction.

  Description:
    Initiates SMI/MIIM read transaction for a given PHY address and register.

  Precondition:
    MyPHYSMIClockSet() called to setup SMI/MIIM clock.

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)
    rIx     - PHY register to be accessed

  Returns:
    None.
*/

void DRV_ETHPHY_SMIReadStart( DRV_HANDLE handle,
                              unsigned int rIx)
{
    DRV_ETHPHY_CLIENT_OBJ * hClientObj = (DRV_ETHPHY_CLIENT_OBJ *) handle;
    ETH_MODULE_ID    ethphyId;  // Assume instance ID for PHY same as Ethernet MAC PLIB

    SYS_ASSERT( NULL != hClientObj, "Bad client handle!" );

    ethphyId = hClientObj->ethphyId;
    while ( PLIB_ETH_MIIMIsBusy(ethphyId) )
    {// wait in case of some previous operation
        //Do nothing.
    }

    PLIB_ETH_PHYAddressSet(ethphyId, hClientObj->hDriver->phyAddress);
    PLIB_ETH_RegisterAddressSet(ethphyId,rIx);
    PLIB_ETH_MIIMReadStart(ethphyId);  // Start read

} /* DRV_ETHPHY_SMIReadStart */


// *****************************************************************************
/* Function:
    uint16_t DRV_ETHPHY_SMIReadResultGet( DRV_HANDLE handle )

  Summary:
    Get result of SMI/MIIM register read.

  Description:
    Get result of SMI/MIIM register read.

  Precondition:
    MyPHYSMIClockSet() called to setup SMI/MIIM clock and
    DRV_ETHPHY_SMIReadStart() called to initiate SMI/MIIM register read.
    DRV_ETHPHY_SMIisBusy() should return false.

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    Result of SMI/MIIM register read previously scheduled.
*/
/* DOM-IGNORE-BEGIN
In eth_miim_access.c:
unsigned short EthMIIMReadResult ( void )
   DOM-IGNORE-END */

uint16_t DRV_ETHPHY_SMIReadResultGet( DRV_HANDLE handle )
{
    DRV_ETHPHY_CLIENT_OBJ * hClientObj = (DRV_ETHPHY_CLIENT_OBJ *) handle;
    ETH_MODULE_ID    ethphyId;  // Assume instance ID for PHY same as Ethernet MAC PLIB

    SYS_ASSERT( NULL != hClientObj, "Bad client handle!" );

    ethphyId = hClientObj->ethphyId;
    while ( PLIB_ETH_MIIMIsBusy(ethphyId) )
    {// wait op complete
        //Do nothing.
    }

    PLIB_ETH_MIIMWriteStart(ethphyId);         // Stop read cycle.
    return PLIB_ETH_MIIMReadDataGet(ethphyId); // The read register

} /* DRV_ETHPHY_SMIReadResultGet */


// *****************************************************************************
/* Function:
    void DRV_ETHPHY_SMIWriteStart( DRV_HANDLE handle,
                                   unsigned int rIx,
                                   uint16_t     wData )
  Summary:
    Initiates SMI/MIIM write transaction.

  Description:
    Initiates SMI/MIIM write transaction for a given PHY address and register.

  Precondition:
    MyPHYSMIClockSet() called to setup SMI/MIIM clock.

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)
    rIx     - PHY register to be accessed
    wData   - Data to be written

  Returns:
    None.
*/

void DRV_ETHPHY_SMIWriteStart( DRV_HANDLE handle,
                               unsigned int rIx,
                               uint16_t     wData )
{
    DRV_ETHPHY_CLIENT_OBJ * hClientObj = (DRV_ETHPHY_CLIENT_OBJ *) handle;
    ETH_MODULE_ID    ethphyId;  // Assume instance ID for PHY same as Ethernet MAC PLIB

    SYS_ASSERT( NULL != hClientObj, "Bad client handle!" );

    ethphyId = hClientObj->ethphyId;
    while ( PLIB_ETH_MIIMIsBusy(ethphyId) )
    {// wait in case of some previous operation
        //Do nothing.
    }

    PLIB_ETH_PHYAddressSet(ethphyId, hClientObj->hDriver->phyAddress);
    PLIB_ETH_RegisterAddressSet(ethphyId,rIx);
    PLIB_ETH_MIIMWriteDataSet(ethphyId,wData);
} /* DRV_ETHPHY_SMIWriteStart */


// *****************************************************************************
/* Function:
    bool DRV_ETHPHY_SMIisBusy( DRV_HANDLE handle )

  Summary:
    Returns boolean true if SMI/MIIM interface is busy with a transaction.

  Description:
    Returns boolean true if SMI/MIIM interface is busy with a transaction.

  Precondition:
    MyPHYSMIClockSet() called to setup SMI/MIIM clock.

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    True - if SMI/MIIM is busy, false otherwise.
*/
/* DOM-IGNORE-BEGIN
New
   DOM-IGNORE-END */

bool DRV_ETHPHY_SMIisBusy( DRV_HANDLE handle )
{
    DRV_ETHPHY_CLIENT_OBJ * hClientObj = (DRV_ETHPHY_CLIENT_OBJ *) handle;
    ETH_MODULE_ID    ethphyId;  // Assume instance ID for PHY same as Ethernet MAC PLIB

    SYS_ASSERT( NULL != hClientObj, "Bad client handle!" );
    ethphyId = hClientObj->ethphyId;
    return PLIB_ETH_MIIMIsBusy(ethphyId);

} /* DRV_ETHPHY_SMIisBusy */


// *****************************************************************************
/* Function:
    void DRV_ETHPHY_SMIScanStart( DRV_HANDLE handle,
                                  unsigned int rIx);
  Summary:
    Start scan of SMI/MIIM register.

  Description:
    Start scan of SMI/MIIM register.

  Precondition:

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)
    rIx:    PHY register to be accessed, 0-31

  Returns:
    None
*/

void DRV_ETHPHY_SMIScanStart( DRV_HANDLE handle,
                              unsigned int rIx)
{
    DRV_ETHPHY_CLIENT_OBJ * hClientObj = (DRV_ETHPHY_CLIENT_OBJ *) handle;
    ETH_MODULE_ID    ethphyId;  // Assume instance ID for PHY same as Ethernet MAC PLIB

    SYS_ASSERT( NULL != hClientObj, "Bad client handle!" );
    ethphyId = hClientObj->ethphyId;
    while ( PLIB_ETH_MIIMIsBusy(ethphyId) )
    {
        //Do nothing.
    }

    PLIB_ETH_PHYAddressSet(ethphyId, hClientObj->hDriver->phyAddress);
    PLIB_ETH_RegisterAddressSet(ethphyId,rIx);
    PLIB_ETH_MIIMScanModeEnable(ethphyId);

} /* DRV_ETHPHY_SMIScanStart */


// *****************************************************************************
/* Function:
    void DRV_ETHPHY_SMIScanStop( DRV_HANDLE handle );

  Summary:
    Stop scan of SMI/MIIM register.

  Description:
    Stop scan of SMI/MIIM register.

  Precondition:

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    None

  Example:
    <code>
    </code>

  Remarks:
    Stops a scan transaction on the SMI interface.
*/
/* DOM-IGNORE-BEGIN
New
   DOM-IGNORE-END */

void DRV_ETHPHY_SMIScanStop( DRV_HANDLE handle )
{
    DRV_ETHPHY_CLIENT_OBJ * hClientObj = (DRV_ETHPHY_CLIENT_OBJ *) handle;
    ETH_MODULE_ID    ethphyId;  // Assume instance ID for PHY same as Ethernet MAC PLIB

    SYS_ASSERT( NULL != hClientObj, "Bad client handle!" );
    ethphyId = hClientObj->ethphyId;
    PLIB_ETH_MIIMScanModeDisable(ethphyId);
}
/* DRV_ETHPHY_SMIScanStop */


// *****************************************************************************
/* Function:
    DRV_ETHPHY_SMI_SCAN_DATA_STATUS DRV_ETHPHY_SMIScanStatusGet( DRV_HANDLE handle )

  Summary:
    Get status of SMI/MIIM scan data.

  Description:
    Get status of SMI/MIIM scan data.

  Precondition:
    DRV_ETHPHY_SMIScanStart() has been called.

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    DRV_ETHPHY_SMI_SCAN_DATA_NOTVALID or DRV_ETHPHY_SMI_SCAN_DATA_VALID

  Example:
    <code>
    </code>

  Remarks:
    None.
*/
/* DOM-IGNORE-BEGIN
New
   DOM-IGNORE-END */

DRV_ETHPHY_SMI_SCAN_DATA_STATUS DRV_ETHPHY_SMIScanStatusGet( DRV_HANDLE handle )
{
    DRV_ETHPHY_CLIENT_OBJ * hClientObj = (DRV_ETHPHY_CLIENT_OBJ *) handle;
    ETH_MODULE_ID    ethphyId;  // Assume instance ID for PHY same as Ethernet MAC PLIB

    SYS_ASSERT( NULL != hClientObj, "Bad client handle!" );
    ethphyId = hClientObj->ethphyId;
    return ( PLIB_ETH_DataNotValid(ethphyId) ? DRV_ETHPHY_SMI_SCAN_DATA_NOTVALID : DRV_ETHPHY_SMI_SCAN_DATA_VALID );

} /* DRV_ETHPHY_SMIScanStatusGet */


// *****************************************************************************
/* Function:
    uint16_t DRV_ETHPHY_SMIScanDataGet( DRV_HANDLE handle )

  Summary:
    Get latest SMI/MIIM scan data result.

  Description:
    Get latest SMI/MIIM scan data result.

  Precondition:
    DRV_ETHPHY_SMIScanStart() has been called.
    DRV_ETHPHY_SMIScanStatusGet() should return DRV_ETHPHY_SMI_SCAN_DATA_VALID.

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    Current scan result.

  Example:
    <code>
    </code>

  Remarks:
    Scan data status must be DRV_ETHPHY_SMI_SCAN_DATA_VALID.
*/
/* DOM-IGNORE-BEGIN
In eth_miim_access.c:
unsigned short EthMIIMScanResult(void)
   DOM-IGNORE-END */

uint16_t DRV_ETHPHY_SMIScanDataGet( DRV_HANDLE handle )
{
    DRV_ETHPHY_CLIENT_OBJ * hClientObj = (DRV_ETHPHY_CLIENT_OBJ *) handle;
    ETH_MODULE_ID    ethphyId;  // Assume instance ID for PHY same as Ethernet MAC PLIB

    SYS_ASSERT( NULL != hClientObj, "Bad client handle!" );

    ethphyId = hClientObj->ethphyId;
    while ( PLIB_ETH_DataNotValid(ethphyId) )
    {// wait until data valid
        //Do Nothing.
    }
    return PLIB_ETH_MIIMReadDataGet(ethphyId); // Return data read

} /* DRV_ETHPHY_SMIScanStatusGet */


// *****************************************************************************
/* Function:
    void DRV_ETHPHY_SMIClockSet( DRV_HANDLE handle,
                                 uint32_t hostClock,
                                 uint32_t maxSMIClock )
  Summary:
    Sets SMI/MIIM interface clock.

  Description:
    Sets SMI/MIIM interface clock base on host clock and maximum supported
    SMI/MIIM interface clock speed.

  Precondition:

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)
    hostClock - Host clock speed in Hz
    maxSMIClock - Maximum supported SMI/MIIM clock speed in Hz

  Returns:
    None.
*/
/* DOM-IGNORE-BEGIN
In eth_miim_access.c:
void EthMIIMConfig(unsigned int hostClock, unsigned int miimClock)
   DOM-IGNORE-END */

void DRV_ETHPHY_SMIClockSet( DRV_HANDLE handle,
                             uint32_t hostClock,
                             uint32_t maxSMIClock )
{
    DRV_ETHPHY_CLIENT_OBJ * hClientObj = (DRV_ETHPHY_CLIENT_OBJ *) handle;
    int  ix;
    ETH_MODULE_ID    ethphyId;  // Assume instance ID for PHY same as Ethernet MAC PLIB

    SYS_ASSERT( NULL != hClientObj, "Bad client handle!" );
    ethphyId = hClientObj->ethphyId;
    PLIB_ETH_MIIMResetEnable(ethphyId); // Issue MIIM reset
    PLIB_ETH_MIIMResetDisable(ethphyId); // Clear MIIM reset

    for(ix=0; ix<sizeof(_MIIClockDivisorTable)/sizeof(*_MIIClockDivisorTable); ix++)
    {
        if(hostClock/_MIIClockDivisorTable[ix]<=maxSMIClock)
        {   // found it
            break;
        }
    }

    if(ix == sizeof(_MIIClockDivisorTable)/sizeof(*_MIIClockDivisorTable))
    {
        ix--;   // max divider; best we can do
    }
    PLIB_ETH_MIIMClockSet(ethphyId,ix+1);  // program the clock

} /* DRV_ETHPHY_SMIClockSet */


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client & Module Level
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void DRV_ETHPHY_RestartNegotiation( DRV_HANDLE handle )

  Summary:
    Restarts autonegotiation of the ETHPHY link.

  Description:
    Restarts autonegotiation of the ETHPHY link.

  Precondition:
    The ETHPHY should have been initialized with proper duplex/speed mode!

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    ETH_RES_OK for success
    ETH_RES_NEGOTIATION_UNABLE if the auto-negotiation is not supported
*/
/* DOM-IGNORE-BEGIN
ETH_RESULT_CODE  __attribute__((weak)) EthPhyRestartNegotiation(void)
   DOM-IGNORE-END */

void DRV_ETHPHY_RestartNegotiation( DRV_HANDLE hClient )
{
    ETH_RESULT_CODE res;
    __BMSTATbits_t  phyCpbl;

    phyCpbl.w = _PhyReadReg(hClient, PHY_REG_BMSTAT);

    if(phyCpbl.AN_ABLE)
    {   // ok, we can perform auto negotiation
        DRV_ETHPHY_SMIWriteStart( hClient,
                                  PHY_REG_BMCON,
                                  _BMCON_AN_ENABLE_MASK|_BMCON_AN_RESTART_MASK );    // restart negotiation and we'll have to wait
        res = ETH_RES_OK;
    }
    else
    {
        res = ETH_RES_NEGOTIATION_UNABLE;     // no negotiation ability!
    }

} /* DRV_ETHPHY_RestartNegotiation */


// *****************************************************************************
/* Function:
    ETHPHY_CONFIG_FLAGS DRV_ETHPHY_HWConfigFlagsGet( DRV_HANDLE handle )

  Summary:
    Returns the current ETHPHY hardware MII/RMII and ALTERNATE/DEFAULT configuration flags

  Description:
    Returns the current ETHPHY hardware MII/RMII and ALTERNATE/DEFAULT configuration flags
    from the Device Configuration Fuse bits.

  Precondition:
    None

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    ETHPHY configuration flag, see ETHPHY_CONFIG_FLAGS enumeration for bit values.
*/
/* DOM-IGNORE-BEGIN
eEthPhyCfgFlags __attribute__((weak)) EthPhyGetHwConfigFlags(void)
   DOM-IGNORE-END */

ETHPHY_CONFIG_FLAGS DRV_ETHPHY_HWConfigFlagsGet( DRV_HANDLE handle )
{
    DRV_ETHPHY_CLIENT_OBJ * hClientObj = (DRV_ETHPHY_CLIENT_OBJ *) handle;
    ETHPHY_CONFIG_FLAGS hwFlags;

    SYS_ASSERT( NULL != hClientObj, "Bad client handle!" );

    // the way the hw is configured
#if defined (__PIC32MX__)
    hwFlags =  (DEVCFG3bits.FMIIEN != 0) ?     ETH_PHY_CFG_MII : ETH_PHY_CFG_RMII;
    hwFlags |= (DEVCFG3bits.FETHIO != 0) ? ETH_PHY_CFG_DEFAULT : ETH_PHY_CFG_ALTERNATE;
#elif defined (__PIC32MZ__)
    // TODO: does PIC32MZ supports multiple ETHIO configurations?
    // TODO: just for 144 pin device, RMII!
    hwFlags = ETH_PHY_CFG_RMII | ETH_PHY_CFG_DEFAULT;
#else
    SYS_ASSERT( false, "Unsupported PIC32 processor!" );
    hwFlags = 0;
#endif  // defined (__PIC32MX__)

    return hwFlags;

} /* DRV_ETHPHY_HWConfigFlagsGet */


// *****************************************************************************
/* Function:
    ETH_RESULT_CODE DRV_ETHPHY_NegotiationIsComplete( DRV_HANDLE handle, bool waitComplete )

  Summary:
    Returns the results of a previously initiated ETHPHY negotiation.

  Description:
    Returns the results of a previously initiated ETHPHY negotiation.

  Precondition:
    DRV_ETHPHY_Setup (and DRV_ETHPHY_RestartNegotiation) should have been called

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)
    waitComplete - boolean flag, true if wait for completion is required

  Returns:
    ETH_RES_OK if negotiation done,
    ETH_RES_NEGOTIATION_INACTIVE if no negotiation in progress
    ETH_RES_NEGOTIATION_NOT_STARTED if negotiation not started yet (means tmo if waitComplete was requested)
    ETH_RES_NEGOTIATION_ACTIVE if negotiation ongoing (means tmo if waitComplete was requested)
*/
/* DOM-IGNORE-BEGIN
ETH_RESULT_CODE  __attribute__((weak)) EthPhyNegotiationComplete(int waitComplete)
   DOM-IGNORE-END */

ETH_RESULT_CODE DRV_ETHPHY_NegotiationIsComplete( DRV_HANDLE hClient, bool waitComplete )
{
    __BMCONbits_t   phyBMCon;
    __BMSTATbits_t  phyStat;
    ETH_RESULT_CODE res;


    phyBMCon.w = _PhyReadReg(hClient, PHY_REG_BMCON);
    if(phyBMCon.AN_ENABLE)
    {   // just protect from an accidental call
        phyBMCon.w = _PhyReadReg(hClient, PHY_REG_BMCON);
        phyStat.w  = _PhyReadReg(hClient, PHY_REG_BMSTAT);

        if(waitComplete)
        {
            unsigned int    tWait;

            if(phyBMCon.AN_RESTART)
            {   // not started yet
                tWait = SYS_TMR_TickCountGet() + ((PHY_NEG_INIT_TMO * SYS_TMR_TickPerSecond()) + 999)/1000;
                do
                {
                    phyBMCon.w=_PhyReadReg(hClient, PHY_REG_BMCON);
                }while(phyBMCon.AN_RESTART && SYS_TMR_TickCountGet() < tWait);      // wait auto negotiation start
            }

            if(!phyBMCon.AN_RESTART)
            {   // ok, started
                tWait = SYS_TMR_TickCountGet() + ((PHY_NEG_DONE_TMO * SYS_TMR_TickPerSecond()) + 999)/1000;
                do
                {
                    phyStat.w=_PhyReadReg(hClient,PHY_REG_BMSTAT);
                }while(phyStat.AN_COMPLETE==0 && SYS_TMR_TickCountGet() < tWait);   // wait auto negotiation done

                phyStat.w=_PhyReadReg(hClient,PHY_REG_BMSTAT);
            }
        }
    }

    if(!phyBMCon.AN_ENABLE)
    {
        res=ETH_RES_NEGOTIATION_INACTIVE;       // no negotiation is taking place!
    }
    else if(phyBMCon.AN_RESTART)
    {
        res=ETH_RES_NEGOTIATION_NOT_STARTED;        // not started yet/tmo
    }
    else
    {
        res= (phyStat.AN_COMPLETE==0)?ETH_RES_NEGOTIATION_ACTIVE:ETH_RES_OK;    // active/tmo/ok
    }

    return res;

} /* DRV_ETHPHY_NegotiationIsComplete */


// *****************************************************************************
/* Function:
    ETH_LINK_STATUS DRV_ETHPHY_NegotiationResultGet( DRV_HANDLE handle,
                                                     ETH_OPEN_FLAGS* pFlags,
                                                     ETH_PAUSE_TYPE* pPauseType )
  Summary:
    Returns the link status after a completed negotiation.

  Description:
    Returns the link status after a completed negotiation.

  Precondition:
    DRV_ETHPHY_Setup, DRV_ETHPHY_RestartNegotiation, and DRV_ETHPHY_NegotiationIsComplete should have been called.

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)
    pFlags     - address to store the negotiation result
    pPauseType - address to store the pause type supported by the link partner

  Returns:
    Link status after the (completed) negotiation, see ETH_LINK_STATUS enumeration.
*/
/* DOM-IGNORE-BEGIN
eEthLinkStat  __attribute__((weak)) EthPhyGetNegotiationResult(eEthOpenFlags* pFlags,
                                                               eEthMacPauseType* pPauseType)
   DOM-IGNORE-END */

ETH_LINK_STATUS DRV_ETHPHY_NegotiationResultGet( DRV_HANDLE hClient,
                                                 ETH_OPEN_FLAGS* pFlags,
                                                 ETH_PAUSE_TYPE* pPauseType )
{
    ETH_LINK_STATUS  linkStat;
    ETH_OPEN_FLAGS   oFlags;
    __BMSTATbits_t   phyStat;
    __ANEXPbits_t    phyExp;
    __ANLPADbits_t   lpAD;
    __ANADbits_t     anadReg;
    ETH_PAUSE_TYPE   pauseType;


    //  should have BMCON.AN_ENABLE==1
    //  wait for it to finish!

    oFlags=0;   // don't know the result yet
    pauseType = ETH_MAC_PAUSE_TYPE_NONE;

    phyStat.w = _PhyReadReg(hClient, PHY_REG_BMSTAT);
    if(phyStat.AN_COMPLETE==0)
    {
        linkStat =( ETH_LINK_ST_DOWN|ETH_LINK_ST_NEG_TMO);
    }
    else if(!phyStat.LINK_STAT)
    {
        linkStat = ETH_LINK_ST_DOWN;
    }
    else
    {   // we're up and running
        int lcl_Pause, lcl_AsmDir, lp_Pause, lp_AsmDir;     // pause capabilities, local and LP

        linkStat = ETH_LINK_ST_UP;

        lcl_Pause  = (ETH_MAC_PAUSE_CPBL_MASK & ETH_MAC_PAUSE_TYPE_PAUSE)?1:0;
        lcl_AsmDir = (ETH_MAC_PAUSE_CPBL_MASK & ETH_MAC_PAUSE_TYPE_ASM_DIR)?1:0;
        lp_Pause   = lp_AsmDir=0;         // in case negotiation fails
        lpAD.w     = _ANAD_BASE10T_MASK;  // lowest priority resolution

        phyExp.w = _PhyReadReg(hClient, PHY_REG_ANEXP);
        if(phyExp.LP_AN_ABLE)
        {   // ok,valid auto negotiation info

            lpAD.w = _PhyReadReg(hClient, PHY_REG_ANLPAD);
            if(lpAD.REM_FAULT)
            {
                linkStat |= ETH_LINK_ST_REMOTE_FAULT;
            }

            if(lpAD.PAUSE)
            {
                linkStat |= ETH_LINK_ST_LP_PAUSE;
                lp_Pause = 1;
            }
            if(lpAD.ASM_DIR)
            {
                linkStat |= ETH_LINK_ST_LP_ASM_DIR;
                lp_AsmDir = 1;
            }

        }
        else
        {
            linkStat |= ETH_LINK_ST_LP_NEG_UNABLE;
            if(phyExp.PDF)
            {
                linkStat |= ETH_LINK_ST_PDF;
            }
        }

        // set the PHY connection params

        anadReg.w  = _PhyReadReg(hClient, PHY_REG_ANAD);       // get our advertised capabilities
        anadReg.w &= lpAD.w;              // get the matching ones
        // get the settings, according to IEEE 802.3 Annex 28B.3 Priority Resolution
        // Note: we don't support 100BaseT4 !

        if(anadReg.w & _ANAD_BASE100TX_FDX_MASK)
        {
            oFlags = (ETH_OPEN_100|ETH_OPEN_FDUPLEX);
        }
        else if(anadReg.w&_ANAD_BASE100TX_MASK)
        {
            oFlags = (ETH_OPEN_100|ETH_OPEN_HDUPLEX);
        }
        else if(anadReg.w&_ANAD_BASE10T_FDX_MASK)
        {
            oFlags = (ETH_OPEN_10|ETH_OPEN_FDUPLEX);
        }
        else if(anadReg.w&_ANAD_BASE10T_MASK)
        {
            oFlags = (ETH_OPEN_10|ETH_OPEN_HDUPLEX);
        }
        else
        {   // this should NOT happen!
            linkStat |= ETH_LINK_ST_NEG_FATAL_ERR;
            linkStat &= ~ETH_LINK_ST_UP;      // make sure we stop...!
        }


        // set the pause type for the MAC
        // according to IEEE Std 802.3-2002 Tables 28B-2, 28B-3
        if(oFlags & ETH_OPEN_FDUPLEX)
        {   // pause type relevant for full duplex only
            if(lp_Pause & (lcl_Pause|(lcl_AsmDir&lp_AsmDir)))
            {
                pauseType = ETH_MAC_PAUSE_TYPE_EN_TX;
            }
            if(lcl_Pause & (lp_Pause | (lcl_AsmDir&lp_AsmDir)))
            {
                pauseType |= ETH_MAC_PAUSE_TYPE_EN_RX;
            }
        }
    }

    if(pFlags)
    {
        *pFlags = oFlags;
    }

    if(pPauseType)
    {
        *pPauseType = pauseType;
    }
    return linkStat;

} /* DRV_ETHPHY_NegotiationResultGet */


// *****************************************************************************
/* Function:
    ETH_LINK_STATUS DRV_ETHPHY_LinkStatusGet( DRV_HANDLE handle, bool refresh )

  Summary:
    Returns current link status.

  Description:
    Returns current link status.

  Precondition:
    DRV_ETHPHY_Setup should have been called.

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)
    refresh - boolean flag, true to specify that a double read is needed.

  Returns:
    Current link status, see ETH_LINK_STATUS enumeration.

  Example:
    <code>
    </code>

  Remarks:
    This function reads the ETHPHY to get current link status
    If refresh is specified then, if the link is down a second read
    will be performed to return the current link status.
*/
/* DOM-IGNORE-BEGIN
eEthLinkStat  __attribute__((weak)) EthPhyGetLinkStatus(int refresh)
   DOM-IGNORE-END */

ETH_LINK_STATUS DRV_ETHPHY_LinkStatusGet( DRV_HANDLE hClient, bool refresh )
{
    __BMSTATbits_t  phyStat;

    // read the link status
    phyStat.w = _PhyReadReg(hClient, PHY_REG_BMSTAT);
    if( phyStat.LINK_STAT==0 && refresh )
    {   // link down could be an old condition. re-read
        phyStat.w = _PhyReadReg(hClient, PHY_REG_BMSTAT);
    }

    return _Phy2LinkStat(phyStat);

} /* DRV_ETHPHY_LinkStatusGet */


// *****************************************************************************
/* Function:
    bool DRV_ETHPHY_Reset( DRV_HANDLE handle, bool waitComplete )

  Summary:
    Immediately resets the ETHPHY, optionally waiting for reset to complete.

  Description:
    Immediately resets the ETHPHY.

  Precondition:
    Communication with the ETHPHY already established

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)
    waitComplete  - boolean flag, if true the procedure will wait for reset to complete

  Returns:
    True  - If PHY reset procedure completed (or completion not required)
    False - Otherwise
*/
/* DOM-IGNORE-BEGIN
int  __attribute__((weak)) EthPhyReset(int waitComplete)
   DOM-IGNORE-END */

bool DRV_ETHPHY_Reset( DRV_HANDLE hClient, bool waitComplete )
{
    DRV_ETHPHY_SMIWriteStart(hClient, PHY_REG_BMCON, _BMCON_RESET_MASK);       // Soft Reset the PHY

    if(waitComplete)
    {   // wait reset self clear
        __BMCONbits_t bmcon;
        unsigned int  tWaitReset;

        tWaitReset = SYS_TMR_TickCountGet() + ((PHY_RESET_CLR_TMO * SYS_TMR_TickPerSecond()) + 999)/1000;
        do
        {
            bmcon.w=_PhyReadReg(hClient, PHY_REG_BMCON);
        }while(bmcon.RESET && SYS_TMR_TickCountGet() < tWaitReset);
        bmcon.w=_PhyReadReg(hClient, PHY_REG_BMCON);
        if(bmcon.RESET)
        {   // tmo clearing the reset
            return false;
        }
    }

    return true;

} /* DRV_ETHPHY_Reset */


// *****************************************************************************
/* Function:
    void DRV_ETHPHY_LinkScanStart( DRV_HANDLE handle )

  Summary:
    Starts SMI scan of link status.

  Description:
    Starts SMI scan of link status.

  Precondition:
    Communication with the ETHPHY already established

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    None.
*/
/* DOM-IGNORE-BEGIN
void __attribute__((weak)) EthPhyScanLinkStart(void)
   DOM-IGNORE-END */

void DRV_ETHPHY_LinkScanStart( DRV_HANDLE hClient )
{
    DRV_ETHPHY_SMIScanStart(hClient, PHY_REG_BMSTAT);
} /* DRV_ETHPHY_LinkScanStart */


// *****************************************************************************
/* Function:
    void DRV_ETHPHY_LinkScanStop( DRV_HANDLE handle )

  Summary:
    Stops SMI scan of link status.

  Description:
    Stops SMI scan of link status.

  Precondition:
    Communication with the ETHPHY already established

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    None.
*/
/* DOM-IGNORE-BEGIN
void __attribute__((weak)) EthPhyScanLinkStop(void)
   DOM-IGNORE-END */

void DRV_ETHPHY_LinkScanStop( DRV_HANDLE hClient )
{
    DRV_ETHPHY_SMIScanStop(hClient);
} /* DRV_ETHPHY_LinkScanStop */


// *****************************************************************************
/* Function:
    ETH_LINK_STATUS DRV_ETHPHY_LinkScanRead( DRV_HANDLE handle )

  Summary:
    Returns current link status.

  Description:
    Returns current link status.

  Precondition:
    DRV_ETHPHY_LinkScanStart must be called first.

  Parameters:
    handle  - Client's driver handle (returned from DRV_ETHPHY_Open)

  Returns:
    Current link status, see ETH_LINK_STATUS enumeration.

  Example:
    <code>
    </code>

  Remarks:
    See also DRV_ETHPHY_LinkStatusGet.
*/
/* DOM-IGNORE-BEGIN
eEthLinkStat __attribute__((weak)) EthPhyScanLinkRead(void)
   DOM-IGNORE-END */

ETH_LINK_STATUS DRV_ETHPHY_LinkScanRead( DRV_HANDLE hClient )
{
    __BMSTATbits_t  phyStat;

    phyStat.w = DRV_ETHPHY_SMIScanDataGet(hClient);

    return _Phy2LinkStat(phyStat);

} /* DRV_ETHPHY_LinkScanRead */


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Version Information
// *****************************************************************************
// *****************************************************************************
/* These functions return the version information of the ETHPHY driver */

//******************************************************************************
/* Function:
    unsigned int DRV_ETHPHY_VersionGet( const SYS_MODULE_INDEX drvIndex )

  Summary:
    Gets ethphy driver version in numerical format.

  Description:
    This routine gets the e driver version. The version is encoded as
    major * 10000 + minor * 100 + patch. The stringized version can be obtained
    using DRV_ETHPHY_VersionStrGet()

  Parameters:
    None.

  Returns:
    Current driver version in numerical format.
*/

unsigned int DRV_ETHPHY_VersionGet( const SYS_MODULE_INDEX drvIndex )
{
    return( ( _DRV_ETHPHY_VERSION_MAJOR * 10000 ) +
            ( _DRV_ETHPHY_VERSION_MINOR * 100 ) +
            ( _DRV_ETHPHY_VERSION_PATCH ) );

} /* DRV_ETHPHY_VersionGet */


// *****************************************************************************
/* Function:
    char * DRV_ETHPHY_VersionStrGet( const SYS_MODULE_INDEX drvIndex )

  Summary:
    Gets ethphy driver version in string format.

  Description:
    This routine gets the ethphy driver version. The version is returned as
    major.minor.path[type], where type is optional. The numertical version can
    be obtained using DRV_ETHPHY_VersionGet()

  Parameters:
    None.

  Returns:
    Current ethphy driver version in the string format.

  Remarks:
    None.
*/

char * DRV_ETHPHY_VersionStrGet( const SYS_MODULE_INDEX drvIndex )
{
    return _DRV_ETHPHY_VERSION_STR;

} /* DRV_ETHPHY_VersionStrGet */


/*******************************************************************************
End of File
*/
