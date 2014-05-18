/*******************************************************************************
 ADC Driver Source File

  Company:    
    Microchip Technology Inc.

  File Name:
    drv_adc.c

  Summary:
    ADC Driver source file.

  Description:
    This file implements the ADC Driver Interface routines. While building the 
    driver from source, ALWAYS use this file in the build.
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
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include "driver/adc/src/drv_adc_local.h"


// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:  
    static void _DRV_ADC_SetupHardware ( const ADC_MODULE_ID plibId,
                                         DRV_ADC_OBJ dObj, DRV_ADC_INIT * adcInit)

  Summary:
    Initializes the hardware registers.

  Description:
    Takes the initialization data from the application (through the DRV_ADC_Initialize
    function) and initializes the hardware registers.

  Remarks:
    None.
*/

static void _DRV_ADC_SetupHardware
(
    /* Function Parameters: (Dynamic arguments are removed from static builds.) */
    _DRV_ADC_DYN_ARG_COMMA( const ADC_MODULE_ID plibId )
    _DRV_ADC_DYN_ARG_COMMA( DRV_ADC_OBJ dObj )
    DRV_ADC_INIT * adcInit
)
{
    /* Get the ADC Interrupt Source */
    _DRV_ADC_STATIC_INT_SRC( _DRV_ADC_OBJ(dObj, interruptSource) = _DRV_ADC_GET_INT_SRC(adcInit->interruptSource) );

    /* Power state initialization */
    _DRV_ADC_PowerState( _DRV_ADC_PERIPHERAL_ID_GET(plibId) , adcInit);

    /* Conversion clock selection */
    _DRV_ADC_ConversionClockSelect( _DRV_ADC_PERIPHERAL_ID_GET( plibId ) ,
                                    _DRV_ADC_CONVERSION_CLOCK( adcInit->conversionClockPrescaler ) ) ;

    /* Conversion clock source selection */
    _DRV_ADC_ConversionClockSource( _DRV_ADC_PERIPHERAL_ID_GET( plibId ) ,
                                    _DRV_ADC_CONVERSION_CLOCK_SOURCE_GET( adcInit->conversionClockSource ) ) ;

    /* Conversion trigger source selection */
    _DRV_ADC_ConversionTriggerSourceSelect( _DRV_ADC_PERIPHERAL_ID_GET( plibId ) ,
                                            _DRV_ADC_CONVERSION_TRIGGER_SOURCE_GET( adcInit->conversionTriggerSource ) ) ;

    /* Conversion trigger source selection */
    _DRV_ADC_SamplesPerInterruptSelect( _DRV_ADC_PERIPHERAL_ID_GET( plibId ) ,
                                        (_DRV_ADC_SAMPLES_PER_INTERRUPT_GET( adcInit->samplesPerInterrupt ) - 1) ) ;
    /* Update the driver object */
    _DRV_ADC_OBJ( dObj , samplesPerInterrupt ) = _DRV_ADC_SAMPLES_PER_INTERRUPT_GET( adcInit->samplesPerInterrupt ) ;

    /* Output data format selection */
    _DRV_ADC_ResultFormatSelect( _DRV_ADC_PERIPHERAL_ID_GET( plibId ) ,
                                 _DRV_ADC_RESULT_FORMAT( adcInit->dataOutputFormat ) ) ;

    /* Check if Auto sampling is supported */
    if ( _DRV_ADC_AUTO_SAMPLING_GET(adcInit->initFlags) & DRV_ADC_AUTO_SAMPLING )
    {
        /* Auto sampling is supported, enable it */
        _DRV_ADC_SampleAutoStartEnable( _DRV_ADC_PERIPHERAL_ID_GET( plibId ) ) ;
        _DRV_ADC_OBJ( dObj , initFlags ) |= DRV_ADC_AUTO_SAMPLING ;
    }

    /* Check if the user has requested for stopping the conversion sequence */
    if ( _DRV_ADC_STOP_CONVERSION_ON_INTERRUPT_GET(adcInit->initFlags) & DRV_ADC_STOP_CONVERSION_ON_INTERRUPT )
    {
        _DRV_ADC_ConversionStopSequenceEnable( _DRV_ADC_PERIPHERAL_ID_GET( plibId ) ) ;
        _DRV_ADC_OBJ( dObj , initFlags ) |= DRV_ADC_STOP_CONVERSION_ON_INTERRUPT ;
    }

    /* Check if the user has requested for Alternating the inputs */
    if ( _DRV_ADC_ALTERNATE_INPUT_SAMPLING_GET(adcInit->initFlags) & DRV_ADC_ALTERNATE_INPUT_SAMPLING )
    {
        _DRV_ADC_ModeAlternateInputSampleEnable( _DRV_ADC_PERIPHERAL_ID_GET( plibId ) ) ;
        _DRV_ADC_OBJ( dObj , initFlags ) |= DRV_ADC_ALTERNATE_INPUT_SAMPLING ;
    }

    /* Voltage reference selection */
    _DRV_ADC_VoltageReferenceSelect( _DRV_ADC_PERIPHERAL_ID_GET( plibId ) ,
                                     _DRV_ADC_VOLTAGE_REF_GET( adcInit->voltageReference ) ) ;

    /* Enable scanning of inputs */
    _DRV_ADC_MuxAInputScanEnable( _DRV_ADC_PERIPHERAL_ID_GET( plibId ) ) ;

    /* Sample acquisition time selection */
    _DRV_ADC_SampleAcqusitionTimeSelect( _DRV_ADC_PERIPHERAL_ID_GET( plibId ) ,
                                         _DRV_ADC_ACQUISITION_TIME_GET( adcInit->acquisitionTime ) ) ;

    /* Select the analog Channel to convert */
    _DRV_ADC_SelectInput( _DRV_ADC_PERIPHERAL_ID_GET( plibId ), _DRV_ADC_ANALOG_INPUT( adcInit->analogInput ) );
    /* Update the driver object */
    _DRV_ADC_OBJ( dObj , inputsMask ) = _DRV_ADC_ANALOG_INPUT( adcInit->analogInput );

} /* _DRV_ADC_SetupHardware */


// *****************************************************************************
// *****************************************************************************
// Section: Interface functions
// *****************************************************************************
// *****************************************************************************


/*******************************************************************************
  Function:
    SYS_MODULE_OBJ DRV_ADC_Initialize ( const SYS_MODULE_INDEX index,
                                        const SYS_MODULE_INIT * const init )

  Summary:
    Initializes hardware and data for the instance of the ADC module.

  Description:
    This function initializes hardware for the instance of the ADC module,
    using the hardware initialization given data.  It also initializes any
    internal data structures.

  Parameters:
    drvId           - Identifier for the instance to be initialized
    init            - Pointer to the data structure containing any data
                      necessary to initialize the hardware. This pointer may
                      be null if no data is required and default
                      initialization is to be used.

  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID.
*/

_DRV_ADC_DYN_RETURN_TYPE (SYS_MODULE_OBJ) _DRV_ADC_MAKE_NAME ( Initialize )
(
    /* Function Parameters: (Dynamic arguments are removed from static builds.) */
    _DRV_ADC_DYN_ARG_COMMA( const SYS_MODULE_INDEX drvIndex )
    const SYS_MODULE_INIT * const init
)
{
    _DRV_ADC_DYN( DRV_ADC_OBJ dObj = (DRV_ADC_OBJ) 0 );
    DRV_ADC_INIT * adcInit;

    /* Validate the driver index */
    if ( _DRV_ADC_INDEX_GET(drvIndex) >= DRV_ADC_INDEX_COUNT )
    {
        _DRV_ADC_DYN_RETURN( SYS_MODULE_OBJ_INVALID );
    }

    /* Validate and assign to the local pointer the pointer passed */
    adcInit = ( DRV_ADC_INIT * ) init ;

    /* Allocate the driver object and set the operation flag to be in use */
    _DRV_ADC_DYN( dObj = _DRV_ADC_OBJ_ALLOCATE(drvIndex) ) ;
    _DRV_ADC_OBJ( dObj , inUse ) = true ;
    _DRV_ADC_OBJ( dObj , currentSampleSet )  = 0 ;
    _DRV_ADC_OBJ( dObj , nextSampleSet )     = 0 ;
    _DRV_ADC_OBJ( dObj , loopedAround )      = false;

    /* Check that the object is valid */
    SYS_ASSERT( _DRV_ADC_ObjectIsValid( dObj ) , "Hardware object is invalid" ) ;

    /* Update the ADC PLIB Id */
    _DRV_ADC_DYN(_DRV_ADC_OBJ( dObj , adcId ) = _DRV_ADC_PERIPHERAL_ID_GET( adcInit->adcId ) );

    /* Setup the Hardware */
    _DRV_ADC_SetupHardware( _DRV_ADC_DYN_PARAM_COMMA( _DRV_ADC_PERIPHERAL_ID_GET( adcInit->adcId ) )
                            _DRV_ADC_DYN_PARAM_COMMA( dObj )
                            adcInit ) ;

    /* Clear the interrupt Source */
    _DRV_ADC_InterruptSourceClear( _DRV_ADC_GET_INT_SRC(_DRV_ADC_OBJ(dObj, interruptSource)) );

    /* Enable the interrupt source in case of interrupt mode */
    _DRV_ADC_InterruptSourceEnable( _DRV_ADC_GET_INT_SRC(_DRV_ADC_OBJ(dObj, interruptSource)) );

    /* Initialization is complete; Set the status as ready */
    _DRV_ADC_OBJ( dObj , status ) = SYS_STATUS_READY ;

    /* Return the object structure */
    _DRV_ADC_DYN_RETURN( (SYS_MODULE_OBJ)dObj );

} /* DRV_ADC_Initialize */


/*******************************************************************************
  Function:
    void DRV_ADC_Reinitialize ( SYS_MODULE_OBJ object,
                                const SYS_MODULE_INIT * const init )

  Summary:
    Reinitializes the hardware for the instance of the ADC module.

  Description:
    This function reinitializes the hardware for the instance of the ADC module
    using the hardware initialization given data. It does not clear or reinitialize 
    internal data structures. Applications should call this function if the module 
    is already initialized and the it wants to change the Initialization. An instance 
    of SYS_MODULE_INIT structure should be filled in by the application and passed to 
    this function.


  Parameters:
    object      - Identifier for the instance to be re-initialized. This will
                  be returned by the Initialize function and will be used for
                  all the system level APIs as a module identifier.
    init        - Pointer to the data structure containing any data
                  necessary to initialize the hardware. This pointer may
                  be null if no data is required and the default
                  initialization is to be used.

  Returns:
    None.
*/

void _DRV_ADC_MAKE_NAME ( Reinitialize )
(
    /* Function Parameters: (Dynamic arguments are removed from static builds.) */
    _DRV_ADC_DYN_ARG_COMMA( SYS_MODULE_OBJ object )
    const SYS_MODULE_INIT * const init
)
{
    _DRV_ADC_DYN( DRV_ADC_OBJ dObj = (DRV_ADC_OBJ) object );
    DRV_ADC_INIT * adcInit = NULL;

    /* Check that the object is valid */
    SYS_ASSERT( _DRV_ADC_ObjectIsValid( dObj ) , "Driver object is invalid" ) ;

    /* Valid init structure is present */
    adcInit = ( DRV_ADC_INIT * ) init ;

    /* Disable the module before initialization */
    _DRV_ADC_DisableInternal( _DRV_ADC_OBJ( dObj , adcId ) );

    /* Set the current driver state */
    _DRV_ADC_OBJ( dObj , status ) = SYS_STATUS_UNINITIALIZED ;

    /* Setup the Hardware */
    _DRV_ADC_SetupHardware( _DRV_ADC_DYN_PARAM_COMMA( _DRV_ADC_PERIPHERAL_ID_GET( adcInit->adcId ) )
                            _DRV_ADC_DYN_PARAM_COMMA( dObj )
                            adcInit ) ;

    /* Enable ADC module */
    _DRV_ADC_EnableInternal( _DRV_ADC_OBJ( dObj , adcId ) );

    _DRV_ADC_MC(_DRV_ADC_OBJ(dObj, numClients) = 0);
    
    /* Initialization is complete; Set the status as ready */
    _DRV_ADC_OBJ( dObj , status ) = SYS_STATUS_READY ;

} /* DRV_ADC_Reinitialize */


/*******************************************************************************
  Function:
    void DRV_ADC_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specific module instance of the ADC module.

  Description:
    Deinitializes the specific module instance disabling its operation (and any
    hardware for driver modules).  Resets all the internal data structures and
    fields for the specified instance to the default settings.

  Parameters:
    object           - Identifier for the instance to be deinitialized

  Returns:
    None.
*/

void _DRV_ADC_MAKE_NAME ( Deinitialize )
(
    /* Function Parameter: (Dynamic arguments are removed from static builds.) */
    _DRV_ADC_DYN_ARG( SYS_MODULE_OBJ object )
)
{
    _DRV_ADC_DYN( DRV_ADC_OBJ dObj = (DRV_ADC_OBJ) object );

    /* Check that the object is valid */
    SYS_ASSERT( _DRV_ADC_ObjectIsValid( dObj ) , "Driver object is invalid" ) ;

    /* Disable the interrupt */
    _DRV_ADC_InterruptSourceDisable( _DRV_ADC_GET_INT_SRC(_DRV_ADC_OBJ(dObj, interruptSource)) ) ;

    /* Set the Device Status */
    _DRV_ADC_OBJ( dObj , status ) = SYS_MODULE_DEINITIALIZED ;

    /* Remove the driver Instance usage */
    _DRV_ADC_OBJ( dObj , inUse ) = false ;

} /* DRV_ADC_Deinitialize */


/*******************************************************************************
  Function:
    SYS_STATUS DRV_ADC_Status ( SYS_MODULE_OBJ object )

  Summary:
    Gets the status of the ADC instance.

  Description:
    Returns the current status of the ADC module.

  Remarks:
    SYS_STATUS_READY	Indicates that any previous module operation for the
                        specified module has completed
    SYS_STATUS_BUSY	Indicates that a previous module operation for the
                        specified module has not yet completed
    SYS_STATUS_ERROR	Indicates that the specified module is in an error state
*/

SYS_STATUS _DRV_ADC_MAKE_NAME ( Status )
(
    /* Function Parameter: (Dynamic arguments are removed from static builds.) */
    _DRV_ADC_DYN_ARG( SYS_MODULE_OBJ object )
)
{
    _DRV_ADC_DYN( DRV_ADC_OBJ dObj = (DRV_ADC_OBJ) object );

    /* Check that the object is valid */
    SYS_ASSERT( _DRV_ADC_ObjectIsValid( dObj ) , "Driver object is invalid" ) ;

    /* Return the status associated with the driver handle */
    return ( _DRV_ADC_OBJ( dObj , status ) ) ;

} /* DRV_ADC_Status */


/*******************************************************************************
  Function:
    DRV_HANDLE DRV_ADC_Open( const SYS_MODULE_INDEX index,
                             const DRV_IO_INTENT intent )

  Summary:
    Opens a new client for the device instance.

  Description:
    Returns a handle of the opened client instance. All client operation APIs
    will require this handle as an argument.
	
  Parameters:
    index           - Identifier for the instance to be initialized
    ioIntent        - Possible values from the enumeration DRV_IO_INTENT

  Remarks:
    If successful, the routine returns a valid open-instance handle (a number
    identifying both the caller and the module instance)
    If an error occurs, the return value is DRV_HANDLE_INVALID
*/

_DRV_ADC_MC_RETURN_TYPE( DRV_HANDLE ) _DRV_ADC_MAKE_NAME ( Open )
(
    /* Function Parameters: (Dynamic arguments are removed from static builds.) */
    _DRV_ADC_DYN_ARG_COMMA( const SYS_MODULE_INDEX drvIndex )
    const DRV_IO_INTENT ioIntent
)
{
    /* Local Variables */
    _DRV_ADC_MC( DRV_ADC_CLIENT_OBJ clientObj = (DRV_ADC_CLIENT_OBJ) 0 );
    _DRV_ADC_DYN( DRV_ADC_OBJ dObj = (DRV_ADC_OBJ) drvIndex );

    /* Validate the driver index */
    if ( _DRV_ADC_INDEX_GET(drvIndex) >= DRV_ADC_INDEX_COUNT )
    {
        _DRV_ADC_MC_RETURN( DRV_HANDLE_INVALID );
    }

    /* Check for exclusive access */
    if (( _DRV_ADC_MC_Test(_DRV_ADC_OBJ( dObj , adcInExclusiveAccess ) == true )) ||
       ( _DRV_ADC_MC_Test(_DRV_ADC_OBJ( dObj , numClients ) > 0) && DRV_IO_ISEXCLUSIVE(ioIntent)))
    {
        /* Set that the hardware instance is opened in exclusive mode */
        _DRV_ADC_MC_RETURN( DRV_HANDLE_INVALID ) ;
    }

    /* Check if max number of clients open */
    if ( _DRV_ADC_MC_Test(_DRV_ADC_OBJ( dObj , numClients ) > DRV_ADC_CLIENTS_NUMBER ))
    {
        /* Set that the hardware instance is opened with max clients */
        _DRV_ADC_MC_RETURN( DRV_HANDLE_INVALID ) ;
    }

    /* To Do: OSAL - Lock Mutex */

    /* Allocate the client object and set the flag as in use */
    _DRV_ADC_MC(clientObj = _DRV_ADC_ClientObjectAllocate(_DRV_ADC_INDEX_GET(drvIndex))) ;
    _DRV_ADC_CLIENT_OBJ( clientObj , inUse ) = true ;
    _DRV_ADC_CLIENT_OBJ( clientObj , clientSampleSet ) = -1 ;
    _DRV_ADC_CLIENT_OBJ( clientObj, currentReadIndex) = 0 ;
    /* To Do: OSAL - Unlock Mutex */

    _DRV_ADC_MC( _DRV_ADC_OBJ(dObj, numClients)++ ) ;

    if ( DRV_IO_ISEXCLUSIVE( ioIntent ) )
    {
        _DRV_ADC_OBJ( dObj , adcInExclusiveAccess ) = true ;
    }

    /* Update the Client Status */
    _DRV_ADC_CLIENT_OBJ( clientObj , status ) = DRV_ADC_CLIENT_STATUS_READY ;

    /* Return the client object */
    _DRV_ADC_MC_RETURN( ( DRV_HANDLE ) clientObj );
    
} /* DRV_ADC_Open */


/*******************************************************************************
  Function:
    void DRV_ADC_Close( DRV_HANDLE handle )

  Summary:
    Removes an opened client.

  Description:
    Closes an opened client of the ADC. Clears the memory used by the client
    and removes the client from the driver.

   Remarks:
    None.
*/

void _DRV_ADC_MAKE_NAME ( Close )
(
    /* Function Parameters: (Multi client arguments are removed from single client builds.) */
    _DRV_ADC_MC_ARG( DRV_HANDLE handle )
)
{
    _DRV_ADC_MC( DRV_ADC_CLIENT_OBJ clientObj = (DRV_ADC_CLIENT_OBJ) 0 );

    /* Get the Client object from the handle passed */
    _DRV_ADC_MC( clientObj = handle ) ;

    /* Check for the Client validity */
    SYS_ASSERT( _DRV_ADC_MC_Test( clientObj < DRV_ADC_CLIENTS_NUMBER)  , "Invalid Client Object" ) ;

    /* To Do: OSAL - lock Mutex */

    /* Delete the Client Instance */
    _DRV_ADC_CLIENT_OBJ( clientObj , inUse ) = false ;

    /* To Do: OSAL - unlock Mutex */

    /* Set the Client Status */
    _DRV_ADC_CLIENT_OBJ( clientObj , status ) = DRV_ADC_CLIENT_STATUS_INVALID ;

} /* DRV_ADC_Close */


/*******************************************************************************
  Function:
    DRV_ADC_CLIENT_STATUS DRV_ADC_ClientStatus( DRV_HANDLE handle )

  Summary:
    Gets the status of the ADC client.

  Description:
    Returns the present status of the client.

  Remarks:
    The status will be specific to ADC and all possible status
    options are listed in DRV_ADC_CLIENT_STATUS enum.
*/

DRV_ADC_CLIENT_STATUS _DRV_ADC_MAKE_NAME ( ClientStatus )
(
    /* Function Parameters: (Multi client arguments are removed from single client builds.) */
    _DRV_ADC_MC_ARG( DRV_HANDLE handle )
)
{
    _DRV_ADC_MC( DRV_ADC_CLIENT_OBJ clientObj = (DRV_ADC_CLIENT_OBJ) 0 );

    /* Get the Client object from the handle passed */
    _DRV_ADC_MC( clientObj = handle ) ;

    /* Check for the Client validity */
    SYS_ASSERT( ( _DRV_ADC_MC_Test( clientObj < DRV_ADC_CLIENTS_NUMBER) ) , "Invalid Client Object" ) ;

    /* Return the client status associated with the handle passed */
    return( _DRV_ADC_CLIENT_OBJ( clientObj , status ) );
    
} /* DRV_ADC_ClientStatus */


/*******************************************************************************
  Function:
    void DRV_ADC_Start( DRV_HANDLE handle )

  Summary:
        Starts the module.

  Description:
        Makes the ADC module enabled (switches ON the module).

  Remarks:
        This should be called once the initialization is complete and ADC is ready
        to operate.
*/

void _DRV_ADC_MAKE_NAME ( Start )
(
    /* Function Parameters: (Multi client arguments are removed from single client builds.) */
    _DRV_ADC_MC_ARG( DRV_HANDLE handle )
)
{
    _DRV_ADC_MC( DRV_ADC_CLIENT_OBJ clientObj = (DRV_ADC_CLIENT_OBJ) 0 );
    _DRV_ADC_DYN( DRV_ADC_OBJ dObj = (DRV_ADC_OBJ) 0 );

    /* Get the Client object from the handle passed */
    _DRV_ADC_MC( clientObj = handle ) ;

    /* Check for the Client validity */
    SYS_ASSERT( ( _DRV_ADC_MC_Test( clientObj < DRV_ADC_CLIENTS_NUMBER) ) , "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    _DRV_ADC_DYN(dObj=_DRV_ADC_CLIENT_OBJ(clientObj,driverObject));

    /* Start the ADC */
    _DRV_ADC_EnableInternal( _DRV_ADC_OBJ( dObj , adcId ) );

    /* trigger conversion in case the device supports only manual */
    _DRV_ADC_ManualStart( _DRV_ADC_OBJ( dObj , adcId ) ) ;
    
    /* Update the Client Status */
    _DRV_ADC_CLIENT_OBJ( clientObj , status ) = DRV_ADC_CLIENT_STATUS_STARTED ;

} /* DRV_ADC_Start */


/*******************************************************************************
  Function:
    void DRV_ADC_Stop( DRV_HANDLE handle )

  Summary:
        Stops the module.

  Description:
        Stops (disables) the ADC module.

  Remarks:
        Applications should call this once it is done with its use of the ADC
        module.
*/

void _DRV_ADC_MAKE_NAME ( Stop )
(
    /* Function Parameters: (Multi client arguments are removed from single client builds.) */
    _DRV_ADC_MC_ARG( DRV_HANDLE handle )
)
{
    _DRV_ADC_MC( DRV_ADC_CLIENT_OBJ clientObj = (DRV_ADC_CLIENT_OBJ) 0 );
    _DRV_ADC_DYN( DRV_ADC_OBJ dObj = (DRV_ADC_OBJ) 0 );

    /* Get the Client object from the handle passed */
    _DRV_ADC_MC( clientObj = handle ) ;

    /* Check for the Client validity */
    SYS_ASSERT( ( _DRV_ADC_MC_Test( clientObj < DRV_ADC_CLIENTS_NUMBER) ) , "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    _DRV_ADC_DYN(dObj=_DRV_ADC_CLIENT_OBJ(clientObj,driverObject)) ;

    /* Stop the ADC from running */
    _DRV_ADC_DisableInternal( _DRV_ADC_OBJ( dObj , adcId ) );

    /* Update the Client Status */
    _DRV_ADC_CLIENT_OBJ( clientObj , status ) = DRV_ADC_CLIENT_STATUS_STOPPED ;

} /* DRV_ADC_Stop */


/*******************************************************************************
  Function:
    void DRV_ADC_InputsRegister ( DRV_HANDLE handle , uint32_t inputsMask )

  Summary:
    Registers an input set with the driver for sampling.

  Description:
    This function registers an input set with the driver for sampling.

  Parameters:
    handle       - Handle of the communication channel as returned by the
                   DRV_ADC_Open function.=
    inputsMask   - Mask bits recognizing the various analog channels

  Returns:
    None
*/

void _DRV_ADC_MAKE_NAME ( InputsRegister )
(
    /* Function Parameters: (Dynamic arguments are removed from static builds.) */
    _DRV_ADC_MC_ARG_COMMA( DRV_HANDLE handle )
    uint32_t inputsMask
)
{
    _DRV_ADC_MC( DRV_ADC_CLIENT_OBJ clientObj = (DRV_ADC_CLIENT_OBJ) 0 );
    _DRV_ADC_DYN( DRV_ADC_OBJ dObj = (DRV_ADC_OBJ) 0 );

    /* Get the Client object from the handle passed */
    _DRV_ADC_MC( clientObj = handle ) ;

    /* Check for the Client validity */
    SYS_ASSERT( ( _DRV_ADC_MC_Test( clientObj < DRV_ADC_CLIENTS_NUMBER) ) , "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    _DRV_ADC_DYN(dObj=_DRV_ADC_CLIENT_OBJ(clientObj,driverObject)) ;

    /* Get the channels to be scanned for conversion */
    /* Activate the required inputs */
    _DRV_ADC_SelectInput( _DRV_ADC_OBJ( dObj , adcId ),
                          inputsMask ) ;
    /* Update the driver object */
    _DRV_ADC_OBJ( dObj , inputsMask ) |= inputsMask;

} /* DRV_ADC_InputsRegister */


/*******************************************************************************
  Function:
    unsigned short DRV_ADC_SamplesRead ( DRV_HANDLE         handle,
                                         ADC_SAMPLE     *buffer,
                                         unsigned short     bufferSize)

  Summary:
    Returns the ADC converted value.

  Description:
    This function is used to return the ADC converted value.

  Parameters:
    Driver handle

  Returns:
    None
*/

unsigned short _DRV_ADC_MAKE_NAME ( SamplesRead )
(
    /* Function Parameters: (Dynamic arguments are removed from static builds.) */
    _DRV_ADC_MC_ARG_COMMA( DRV_HANDLE handle )
    ADC_SAMPLE *buffer ,
    unsigned short bufferSize
)
{
    _DRV_ADC_MC( DRV_ADC_CLIENT_OBJ clientObj = (DRV_ADC_CLIENT_OBJ) 0 );
    _DRV_ADC_DYN( DRV_ADC_OBJ dObj = (DRV_ADC_OBJ) 0 );
    uint8_t i = 0, c = 0, clientSampleSet = 0, nextSampleSet = 0 , currentSampleSet = 0 ;

    /* Get the Client object from the handle passed */
    _DRV_ADC_MC( clientObj = handle ) ;

    /* Check for the Client validity */
    SYS_ASSERT( ( _DRV_ADC_MC_Test( clientObj < DRV_ADC_CLIENTS_NUMBER) ) , "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    _DRV_ADC_DYN(dObj=_DRV_ADC_CLIENT_OBJ(clientObj,driverObject)) ;

    /* if bufferSize is < DRV_ADC_CONFIG_SAMPLES_PER_INTERRUPT , return DRV_ADC_STATUS_BUFFER_TOO_SMALL */
    if(bufferSize < DRV_ADC_SAMPLES_PER_INTERRUPT )
    {
        _DRV_ADC_CLIENT_OBJ(clientObj, status ) = DRV_ADC_CLIENT_STATUS_BUFFER_TOO_SMALL;
        return 0;
    }

    /*  if _DRV_ADC_CLIENT_OBJ(clientObj, operationalFlags) is set for DRV_ADC_OVERFLOW
        return the error DRV_ADC_CLIENT_STATUS_OVERFLOW
     */

    /* Get the read index from the client object and read the bufferSize number
       of samples */

    currentSampleSet = _DRV_ADC_OBJ(dObj, currentSampleSet);

    if ( _DRV_ADC_CLIENT_OBJ(clientObj, clientSampleSet) == (uint8_t)-1 )
    {
        clientSampleSet = _DRV_ADC_CLIENT_OBJ(clientObj, clientSampleSet) =
                    _DRV_ADC_SampleSetNextGet(currentSampleSet) * _DRV_ADC_OBJ(dObj, loopedAround);
    }
    else
    {
        clientSampleSet = _DRV_ADC_CLIENT_OBJ(clientObj, clientSampleSet);
        
        if( _DRV_ADC_OBJ( dObj, readRequestFlag[clientSampleSet] ) & \
            _DRV_ADC_CLIENT_OBJ( clientObj, accessMask ) )
        {
            /* The sample set it is supposed to be reading is not yet
               overwritten, so read it */
        }
        else
        {
            /* The sample set it is supposed to be reading is overwritten
               so go to the tail and grab the oldest sample set. */
            _DRV_ADC_CLIENT_OBJ (clientObj, status ) = DRV_ADC_CLIENT_STATUS_OVERFLOW;
            clientSampleSet = _DRV_ADC_CLIENT_OBJ(clientObj, clientSampleSet) =
                    _DRV_ADC_SampleSetNextGet(currentSampleSet);
        }

    }

    c = _DRV_ADC_CLIENT_OBJ(clientObj, clientSampleSet) * 
        _DRV_ADC_SAMPLES_PER_INTERRUPT_GET( _DRV_ADC_OBJ(dObj, samplesPerInterrupt));

    /* Restrict the buffer size */
    if( bufferSize > DRV_ADC_INTERNAL_BUFFER_SIZE )
    {
        /* The driver can at most return the sample set array - 1, 
           since the task routine might be working on the latest one.  */
        bufferSize = DRV_ADC_INTERNAL_BUFFER_SIZE;
    }

    /* Read the result from the result buffer
       Restrict the buffer size to the SAMPLE SET SIZE */
    for( i =0 ; i < bufferSize ; i++)
    {
        buffer[i] = _DRV_ADC_OBJ(dObj, resultBuffer[c++]);
    }

    /* Set the read request flag for the next sample set */
    /* The next sample set in this case will be clientSampleSet + 1
       (since you are allowing only read of one sample set at a time),
       and not necessary the current sample set
       if you allow multiple sample set reads at time, this logic will change */

    nextSampleSet = _DRV_ADC_SampleSetNextGet(currentSampleSet);
    _DRV_ADC_OBJ( dObj, readRequestFlag[nextSampleSet] ) |=
            _DRV_ADC_CLIENT_OBJ( clientObj, accessMask ) ;

    return i;

} /* DRV_ADC_SamplesRead */


/*******************************************************************************
  Function:
    unsigned short DRV_ADC_SamplesReadLatest ( DRV_HANDLE        handle,
                                               ADC_SAMPLE   *buffer,
                                               unsigned short    bufferSize)

   Summary:
    Returns the ADC converted value.

  Description:
    This function is used to return the ADC converted value.

  Parameters:
    Driver handle.

  Returns:
    None.
*/

unsigned short _DRV_ADC_MAKE_NAME ( SamplesReadLatest )
(
    /* Function Parameters: (Dynamic arguments are removed from static builds.) */
    _DRV_ADC_MC_ARG_COMMA( DRV_HANDLE handle )
    ADC_SAMPLE *buffer ,
    unsigned short bufferSize
)
{
    _DRV_ADC_MC( DRV_ADC_CLIENT_OBJ clientObj = (DRV_ADC_CLIENT_OBJ) 0 );
    _DRV_ADC_DYN( DRV_ADC_OBJ dObj = (DRV_ADC_OBJ) 0 );
    unsigned short i , c ;

    /* Get the Client object from the handle passed */
    _DRV_ADC_MC( clientObj = handle ) ;

    /* Check for the Client validity */
    SYS_ASSERT( ( _DRV_ADC_MC_Test( clientObj < DRV_ADC_CLIENTS_NUMBER) ) , "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    _DRV_ADC_DYN(dObj=_DRV_ADC_CLIENT_OBJ(clientObj,driverObject)) ;

    /* if bufferSize is > DRV_ADC_CONFIG_SAMPLES_PER_INTERRUPT , return DRV_ADC_STATUS_BUFFER_TOO_SMALL */

    /*  if _DRV_ADC_CLIENT_OBJ(clientObj, operationalFlags) is set for DRV_ADC_OVERFLOW
        return the error DRV_ADC_CLIENT_STATUS_OVERFLOW
     */

    if ( bufferSize > DRV_ADC_SAMPLES_PER_INTERRUPT )
    {
        bufferSize = DRV_ADC_SAMPLES_PER_INTERRUPT ;
    }

    /* Get the read index from the client object and read the bufferSize number
       of samples */
    c = _DRV_ADC_OBJ( dObj, currentSampleSet ) *
            _DRV_ADC_SAMPLES_PER_INTERRUPT_GET( _DRV_ADC_OBJ(dObj, samplesPerInterrupt));

    for( i =0 ; i < bufferSize ; i++)
    {
        buffer[i] = _DRV_ADC_OBJ(dObj, resultBuffer[c++]);
    }

    _DRV_ADC_CLIENT_OBJ(clientObj, status ) = DRV_ADC_CLIENT_STATUS_READY;
    
    return i;

} /* DRV_ADC_SamplesReadLatest */


/*******************************************************************************
  Function:
    bool DRV_ADC_SamplesAvailable ( DRV_HANDLE handle )

   Summary:
    Gives an indication about the sample availability.

  Description:
    This function is used to provide the status of sample availability.

  Parameters:
    Client handle

  Returns:
    -true  - Sample is available
    -false - Sample is not available

  Remarks:
    Note: Status is set as unavailable after a call to the function is made.
*/

bool _DRV_ADC_MAKE_NAME ( SamplesAvailable )
(
    /* Function Parameters: (Multi client arguments are removed from single client builds.) */
    _DRV_ADC_MC_ARG( DRV_HANDLE handle )
)
{
    _DRV_ADC_MC( DRV_ADC_CLIENT_OBJ clientObj = (DRV_ADC_CLIENT_OBJ) 0 );
    _DRV_ADC_DYN( DRV_ADC_OBJ dObj = (DRV_ADC_OBJ) 0 );

    /* Get the Client object from the handle passed */
    _DRV_ADC_MC( clientObj = handle ) ;

    /* Check for the Client validity */
    SYS_ASSERT( ( _DRV_ADC_MC_Test( clientObj < DRV_ADC_CLIENTS_NUMBER) ) , "Invalid Client Object" ) ;

    /* Get the driver object from the client */
    _DRV_ADC_DYN(dObj=_DRV_ADC_CLIENT_OBJ(clientObj,driverObject)) ;

    /* Check if the ADC Converted data is available */
    if ( ( _DRV_ADC_OBJ( dObj , adcSamplesAvailable ) == true ) )
    {
        /* Clear the flag for the next iteration */
        _DRV_ADC_OBJ( dObj , adcSamplesAvailable ) =  false ;

        return true ;
    }
    else
    {
        return false ;
    }

} /* DRV_ADC_SamplesAvailable */


/*******************************************************************************
  Function:
    void DRV_ADC_Tasks( SYS_MODULE_OBJ object)

   Summary:
    Used to maintain the driver's state machine and implement its ISR.

  Description:
    This function is used to maintain the driver's internal state machine and
    implement its ISR for interrupt-driven implementations.

  Parameters:
    contextData - Device-specific hardware data

  Returns:
    None.
*/

void _DRV_ADC_MAKE_NAME ( Tasks )
(
    /* Function Parameter: (Dynamic arguments are removed from static builds.) */
    _DRV_ADC_DYN_ARG( SYS_MODULE_OBJ object )
)
{
    _DRV_ADC_DYN( DRV_ADC_OBJ dObj = (DRV_ADC_OBJ) object );
    uint8_t i = 0, writeIndex = 0, currentSampleSet = 0 ;
    ADC_MODULE_ID adcId;
    ADC_SAMPLE sampleBuffer;

    /* Check for the valid driver object passed */
    SYS_ASSERT( _DRV_ADC_ObjectIsValid( dObj ) , "Driver Object is invalid" ) ;

    if ( true == _DRV_ADC_InterruptSourceStatusGet( _DRV_ADC_GET_INT_SRC(_DRV_ADC_OBJ(dObj, interruptSource)) ) )
    {
        if ( _DRV_ADC_OBJ( dObj , nextSampleSet ) < _DRV_ADC_NUMBER_OF_SAMPLE_SETS )
        {
            currentSampleSet = _DRV_ADC_OBJ( dObj , currentSampleSet ) =
                    _DRV_ADC_OBJ( dObj , nextSampleSet );

            _DRV_ADC_OBJ( dObj , nextSampleSet ) += 1;
            _DRV_ADC_OBJ( dObj , loopedAround )  = false ;
        }
        else
        {
            _DRV_ADC_OBJ( dObj , nextSampleSet ) = 0;

            currentSampleSet = _DRV_ADC_OBJ( dObj , currentSampleSet ) =
                    _DRV_ADC_OBJ( dObj , nextSampleSet );

            _DRV_ADC_OBJ( dObj , nextSampleSet ) += 1;
            _DRV_ADC_OBJ( dObj , loopedAround )  = true ;
        }

        /* Setup overflow for all the clients */
        _DRV_ADC_OBJ(dObj, readRequestFlag[currentSampleSet]) = 0;


        writeIndex = currentSampleSet *
                _DRV_ADC_SAMPLES_PER_INTERRUPT_GET( _DRV_ADC_OBJ( dObj, samplesPerInterrupt ) );

        for ( i = 0 ; i < _DRV_ADC_SAMPLES_PER_INTERRUPT_GET( _DRV_ADC_OBJ( dObj, samplesPerInterrupt ) ) ; i++ )
        {
            adcId = _DRV_ADC_OBJ( dObj , adcId );
            sampleBuffer = _DRV_ADC_ReadSample( adcId , i ) ;
            _DRV_ADC_OBJ( dObj , resultBuffer[writeIndex++] ) = sampleBuffer;
        }

        /* Indicate to the application that the Conversion has been completed */
        _DRV_ADC_OBJ( dObj , adcSamplesAvailable ) = true ;

        /* Clear ADC Interrupt/Status Flag */
        _DRV_ADC_InterruptSourceClear( _DRV_ADC_GET_INT_SRC(_DRV_ADC_OBJ(dObj, interruptSource)) ) ;

        /* Manual Sampling/Conversion Start */
        /* trigger conversion in case the device supports manual */
        _DRV_ADC_ManualStart( _DRV_ADC_OBJ( dObj , adcId ) ) ;
    }

} /* DRV_ADC_Tasks */


/*******************************************************************************
End of File
*/
