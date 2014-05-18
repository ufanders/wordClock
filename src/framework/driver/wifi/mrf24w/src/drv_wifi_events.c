/*******************************************************************************
  tcpip MRF24W MAC events implementation

  File Name:  
    drv_wifi_events.c  
  
  Summary:
    
  Description:
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

#include "tcpip/src/tcpip_private.h"

#if defined(TCPIP_IF_MRF24W)

// stack internal notification
typedef struct
{
    bool                    _mrfEnabledEvents;          // group enabled notification events
    volatile TCPIP_MAC_EVENT _mrfPendingEvents;         // group notification events that are set, waiting to be re-acknowledged
    TCPIP_MAC_EventF        _mrfNotifyFnc;              // group notification handler
    const void*             _mrfNotifyParam;            // notification parameter
} MRF24W_EV_GROUP_DCPT;   // event descriptor

/*************************
 * local data
 *************************/

static MRF24W_EV_GROUP_DCPT  _mrfGroupDcpt = 
{
    TCPIP_MAC_EV_NONE, TCPIP_MAC_EV_NONE, 0     
};

static MRF24W_USR_EV_DCPT    _mrfUsrEvent;        // stack user events


/*********************************
 *  local proto
 *  referenced by WFMAc::MACInit() too! 
 ******************************************/
void DRV_WIFI_MRF24W_Tasks_ISR(SYS_MODULE_OBJ index);


/*******************************************************************************
  Function:
    TCPIP_MAC_RES    MRF24W_MACEventInit(TCPIP_MAC_HANDLE hMac,
                                         TCPIP_MAC_EventF eventF,
                                         const void*      eventParam,
                                         int              intPri,
                                         int              intSubPri)

  Summary:
    Initializes the ethernet event notification.

  Description:
     This function initializes the ethernet event notification.
     It performs any resource allocation that may be needed.

  Precondition:
     None.

  Parameters:
    hMac      - parameter identifying the intended MAC  
    intPri     - priority of the TCPIP interrupt events
    intSubPri  - sub-priority of the TCPIP interrupt events
    
  Returns:
    TCPIP_MAC_RES_OK  if initialization succeeded,
    an error code otherwise

  Example:
    <code>
    MRF24W_MACEventInit( hMac, 4, 3 );
    </code>

  Remarks:

    Not multi-threaded safe.
*****************************************************************************/
TCPIP_MAC_RES MRF24W_MACEventInit(TCPIP_MAC_HANDLE hMac,
                                  TCPIP_MAC_EventF eventF,
                                  const void*      eventParam,
                                  int              intPri,
                                  int              intSubPri)
{
    hMac   = hMac;              // avoid compiler warning
    intPri = intPri;            // avoid compiler warning
    intSubPri = intSubPri;      // avoid compiler warning

    _mrfGroupDcpt._mrfNotifyFnc = eventF;     // set new handler
    _mrfGroupDcpt._mrfNotifyParam = eventParam;   
    _mrfGroupDcpt._mrfEnabledEvents = false;
    _mrfGroupDcpt._mrfPendingEvents = 0;

    _mrfUsrEvent.trafficEvents = _mrfUsrEvent.mgmtEvents = 0;
    _mrfUsrEvent.trafficEventInfo = _mrfUsrEvent.mgmtEventInfo =0;

    return TCPIP_MAC_RES_OK;
}

/*******************************************************************************
  Function:
    TCPIP_MAC_RES    MRF24W_MACEventDeInit(TCPIP_MAC_HANDLE hMac )

  Summary:
    De-initializes the ethernet event notification.

  Description:
     This function de-initializes the ethernet event notification.
     It performs any resource clean-up that may be needed.

  Precondition:
     None.

  Parameters:
    hMac      - parameter identifying the intended MAC  
    
  Returns:
    TCPIP_MAC_RES_OK  always

  Example:
    <code>
    MRF24W_MACEventDeInit( hMac );
    </code>

  Remarks:

    Not multi-threaded safe.
*****************************************************************************/
TCPIP_MAC_RES MRF24W_MACEventDeInit(TCPIP_MAC_HANDLE hMac)
{
    SYS_INT_SourceDisable(MRFWB0M_INT_SOURCE);      // stop MRF ints
    SYS_INT_SourceStatusClear(MRFWB0M_INT_SOURCE);

    _mrfGroupDcpt._mrfNotifyFnc = 0;
    _mrfGroupDcpt._mrfEnabledEvents = false;
    _mrfGroupDcpt._mrfPendingEvents = 0;

    return TCPIP_MAC_RES_OK;
}


/*******************************************************************************
  Function:
    bool    MRF24W_MACEventSetMask(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT tcpipEvents, bool enable)

  Summary:
    Adds new events to the list of the enabled ones.

  Description:
     This function sets new enabled events.
     Multiple events can be orr-ed together.
     All events that are set will be added to the notification process. The other events will not ne touched.
     The stack (or stack user) has to catch the events that are notified and process them:
         - The stack should process the TCPIP_MAC_EV_RX_PKTPEND, TCPIP_MAC_EV_TX_DONE transfer events
         - Process the specific condition and acknowledge them calling MRF24W_MACEventAcknowledge() so that they can be re-enabled.

  Precondition:
   TCPIPInit should have been called.

  Parameters:
    hMac      - parameter identifying the intended MAC  
    tcpipEvents - events the user of the stack wants to add for notification
    enable      - boolean to enable/disable the event notification
    
  Returns:
    true  if operation succeeded,
    false code otherwise

  Example:
    <code>
    MRF24W_MACEventSetMask( hMac, TCPIP_MAC_EV_RX_OVFLOW | TCPIP_MAC_EV_RX_BUFNA, true );
    </code>

  Remarks:
    Globally enable/disable all notifications for now.

*****************************************************************************/
bool MRF24W_MACEventSetMask(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents, bool enable)
{
    if(enable)
    {
        _mrfGroupDcpt._mrfEnabledEvents = true;
        SYS_INT_SourceEnable(MRFWB0M_INT_SOURCE);      // start MRF ints
    }
    else
    {
        SYS_INT_SourceDisable(MRFWB0M_INT_SOURCE);      // stop MRF ints
        _mrfGroupDcpt._mrfEnabledEvents = false;
    }

    return true;
}

/*******************************************************************************
  Function:
    bool    MRF24W_MACEventAcknowledge(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT tcpipEvents)

  Summary:
    Acknowledges and re-enables processed events.

  Description:
    This function acknowledges and re-enables processed events.
    Multiple events can be orr-ed together as they are processed together.
    The events acknowledged by this function should be the events that have been retrieved from the stack
    by calling MRF24W_MACEventGetPending() or have been passed to the user by the stack using the notification handler
    and have been processed and have to be re-enabled.


  Precondition:
   TCPIPInit should have been called.

  Parameters:
    hMac      - parameter identifying the intended MAC  
    tcpipEvents - the events that the user processed and need to be re-enabled
    
  Returns:
    true if events acknowledged
    false if no events to be acknowledged
    an error code otherwise

  Example:
    <code>
    MRF24W_MACEventAcknowledge( hMac, stackNewEvents );
    </code>

  Remarks:   
    All events should be acknowledged, in order to be re-enabled.

    For now, the re-enabling is done internally by the MRF processing events code.
*****************************************************************************/
bool MRF24W_MACEventAcknowledge(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents)
{
    if(_mrfGroupDcpt._mrfPendingEvents)
    {
        _mrfGroupDcpt._mrfPendingEvents = 0;
        return true;
    }
    else
    {
        return false;
    }
}


/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT MRF24W_MACEventGetPending(TCPIP_MAC_HANDLE hMac)

  Summary:
    Returns the currently pending events.

  Description:
    This function returns the currently pending events.
    Multiple events can be orr-ed together as they accumulate.
    The stack should be called for processing whenever a stack managed event (TCPIP_MAC_EV_RX_PKTPEND, TCPIP_MAC_EV_TX_DONE) is present.
    The other, non critical events, may not be managed by the stack and passed to an user.
    They will have to be eventually acknowledged if re-enabling is needed.

  Precondition:
   hMac      - parameter identifying the intended MAC  
   TCPIPInit should have been called.

  Parameters:
    
  Returns:
    The currently stack pending events.

  Example:
    <code>
    TCPIP_MAC_EVENT currEvents = MRF24W_MACEventGetPending( hMac);
    </code>

  Remarks:   
    This is the preferred method to get the current pending MAC events.
    The stack maintains a proper image of the events from their occurrence to their acknowledgement.
    
    Even with a notification handler in place it's better to use this function to get the current pending events
    rather than using the events passed by the notification handler which could be stale.
    
    The events are persistent. They shouldn't be re-enabled unless they have been processed and
    the condition that generated them was removed.
    Re-enabling them immediately without proper processing will have dramatic effects on system performance.

    The returned value is just a momentary value. The pending events can change any time.
*****************************************************************************/
TCPIP_MAC_EVENT MRF24W_MACEventGetPending(TCPIP_MAC_HANDLE hMac)
{
    return _mrfGroupDcpt._mrfPendingEvents;
}


/*******************************************************************************
  Function:
    void WF_UserEventsSet(tWFEvents event, uint16_t eventInfo, bool isMgmt);

  Summary:
    Sets the current events from the MRF24WG

  Description:
    This function sets the current event(s) from the MRF24WG

  Parameters:
    event       -- current MRF24W traffic event.
    eventInfo   -- additional event info
                   This is not applicable to all events.
    isMgmt      -- specifies traffic or management event

  Returns:
    None.
  *****************************************************************************/
void WF_UserEventsSet(uint16_t event, uint16_t eventInfo, bool isMgmt)
{
    if(isMgmt)
    {
        _mrfUsrEvent.mgmtEvents = event;
        _mrfGroupDcpt._mrfPendingEvents = event;   // add this line so event function sees event?
        _mrfUsrEvent.mgmtEventInfo = eventInfo;
    }
    else
    {
        _mrfUsrEvent.trafficEvents = event;
        _mrfGroupDcpt._mrfPendingEvents = event;   // add this line so event function sees event?
        _mrfUsrEvent.trafficEventInfo = eventInfo;
    }

    // let app know of event
    if(_mrfGroupDcpt._mrfNotifyFnc)
    {
        (*_mrfGroupDcpt._mrfNotifyFnc)(_mrfGroupDcpt._mrfPendingEvents, _mrfGroupDcpt._mrfNotifyParam);
    }
} 

/*******************************************************************************
  Function:
    uint16_t WF_TrafficEventsGet(uint16_t* pEventInfo);

  Summary:
    Returns the current traffic events from the MRF24WG

  Description:
    This function returns the current traffic events from the MRF24WG

  Precondition:
    TCPIP stack should be initialized.

  Parameters:
    pEventInfo -- address to store additional information about the traffic event.
                  This is not applicable to all events.

  Returns:
          The traffic event that occurred.

  Remarks:
          None.
  *****************************************************************************/
uint16_t WF_TrafficEventsGet(uint16_t* pEventInfo)
{
    uint16_t res = _mrfUsrEvent.trafficEvents;
    if(pEventInfo)
    {
        *pEventInfo = _mrfUsrEvent.trafficEventInfo;
    }
    
    _mrfUsrEvent.trafficEvents = 0;
    _mrfUsrEvent.trafficEventInfo = 0;

    return res;
}

/*******************************************************************************
  Function:
    uint16_t WF_MgmtEventsGet(MRF24W_USR_EV_DCPT *pEventInfo);

  Summary:
    Returns the current management events from the MRF24W

  Description:

  Precondition:
          TCPIP stack should be initialized.

  Parameters:
    pEventInfo -- address to store additional information about the management event.
                  This is not applicable to all events.

  Returns:
          The management event that occurred, or 0 if no mgmt event has occurred

  Remarks:
          None.
  *****************************************************************************/
uint16_t WF_MgmtEventsGet(MRF24W_USR_EV_DCPT *pEventInfo)
{
    uint16_t res;
    // if mgmt event has occurred
    if (_mrfUsrEvent.mgmtEvents > 0)
    {
        // copy info to caller structure
        pEventInfo->mgmtEvents = _mrfUsrEvent.mgmtEvents;
        pEventInfo->mgmtEventInfo =  _mrfUsrEvent.mgmtEventInfo;
        res = pEventInfo->mgmtEvents;
    }
    else
    {
        res = 0;    // signals no mgmt event has occurred
    }

    // clear global structure for next mgmt event
    _mrfUsrEvent.mgmtEvents = 0;
    _mrfUsrEvent.mgmtEventInfo = 0;

    return res;
}


/****************************************************************************
 * Function:        DRV_WIFI_MRF24W_Tasks_ISR
 *
 * PreCondition:    TCPIPInit, MRF24W_MACEventSetMask should have been called.
 *
 * Input:           p - unused
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function processes the Ethernet interrupts and reports the events back to the user.
 *
 * Note:            None
 ******************************************************************************/
void DRV_WIFI_MRF24W_Tasks_ISR(SYS_MODULE_OBJ index)
{
    
    if(_mrfGroupDcpt._mrfEnabledEvents )
    {
        _mrfGroupDcpt._mrfPendingEvents = TCPIP_MAC_EV_RX_PKTPEND|TCPIP_MAC_EV_TX_DONE;
        if(_mrfGroupDcpt._mrfNotifyFnc)
        {
            (*_mrfGroupDcpt._mrfNotifyFnc)(_mrfGroupDcpt._mrfPendingEvents, _mrfGroupDcpt._mrfNotifyParam);  
        }
    }
    
    SYS_INT_SourceDisable(MRFWB0M_INT_SOURCE);        // disable further interrupts
    SYS_INT_SourceStatusClear(MRFWB0M_INT_SOURCE);      // acknowledge the int Controller

    /* invoke MRF handler */
    DRV_WIFI_INT_Handle();
}

#endif  // defined(TCPIP_IF_MRF24W)

//DOM-IGNORE-END
