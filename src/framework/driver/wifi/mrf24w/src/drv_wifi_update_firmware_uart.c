/*******************************************************************************
  MRF24W Driver Medium Access Control (MAC) Layer

  File Name: 
    drv_wifi_upate_firmware_uart.c
  
  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides access to MRF24W WiFi controller
    -Reference: MRF24W Data sheet, IEEE 802.11 Standard
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

/*
*********************************************************************************************************
*                                           INCLUDES                               
*********************************************************************************************************
*/
#include "tcpip/src/tcpip_private.h"
#include "tcpip/tcpip_mac.h"
#include "tcpip/src/tcpip_mac_object.h"

#if defined(TCPIP_IF_MRF24W) 
bool wf_update_begin_uart = false;

#if defined(WF_UPDATE_FIRMWARE_UART)

#define MAX_USER_RESPONSE_LEN   (20u)

#define XMODEM_SOH      0x01u
#define XMODEM_EOT      0x04u
#define XMODEM_ACK      0x06u
#define XMODEM_NAK      0x15u
#define XMODEM_CAN      0x18u
#define XMODEM_BLOCK_LEN 128u


extern void dbgPrintf( const char* format, ... ) ;
extern uint32_t ImageUpdate_Addr;
extern uint32_t ImageUpdate_Checksum;
extern uint32_t ImageUpdate_Size;
extern uint8_t AutoUpdate_Initialize_SM(void);
extern void AutoUpdate_Completed(void);
extern void AutoUpdate_Restore(void);
extern void delay_update(int k);
void dbgPrintf( const char* format, ... )
{
}
static void XMODEM_SendToModule_subAPI(uint8_t *buf)
{
    int i;
    uint8_t buf_module[36];
    buf_module[0]=(ImageUpdate_Addr&0x00FF0000)>>16;
    buf_module[1]=(ImageUpdate_Addr&0x0000FF00)>>8;
    buf_module[2]=(ImageUpdate_Addr&0xFF);
    buf_module[3]=32;
    for(i=0;i<32;i++) buf_module[i+4]=buf[i];
    SendSetParamMsg(PARAM_FLASH_WRITE, buf_module, 36);
    ImageUpdate_Addr += 32;
}


static void XMODEM_SendToModule(uint8_t *xmodm_buf)
{
    int i;
    
    //  1. Calculate checksum
    for(i=0;i<128;i++)
    {
        if((ImageUpdate_Size % 4) == 0) ImageUpdate_Checksum += (uint32_t)xmodm_buf[i]<<24;
        if((ImageUpdate_Size % 4) == 1) ImageUpdate_Checksum += (uint32_t)xmodm_buf[i]<<16;
        if((ImageUpdate_Size % 4) == 2) ImageUpdate_Checksum += (uint32_t)xmodm_buf[i]<<8;
        if((ImageUpdate_Size % 4) == 3) ImageUpdate_Checksum += (uint32_t)xmodm_buf[i];
        ImageUpdate_Size ++;
    }
    // 2. send 128 bytes                
    XMODEM_SendToModule_subAPI(&xmodm_buf[0]);
    XMODEM_SendToModule_subAPI(&xmodm_buf[32]);
    XMODEM_SendToModule_subAPI(&xmodm_buf[64]);
    XMODEM_SendToModule_subAPI(&xmodm_buf[96]);
}
static uint8_t tempData[XMODEM_BLOCK_LEN];

bool    WF_FirmwareUpdate_Uart_24G(void)
{
    enum SM_FIRMWARE_UPDATE
    {
        SM_FIRMWARE_UPDATE_SOH,
        SM_FIRMWARE_UPDATE_BLOCK,
        SM_FIRMWARE_UPDATE_BLOCK_CMP,
        SM_FIRMWARE_UPDATE_DATA,
        SM_FIRMWARE_UPDATE_CHECKSUM,
        SM_FIRMWARE_UPDATE_FINISH,
    } state;

    uint8_t c;
   // MPFS_HANDLE handle;
    bool lbDone;
    uint8_t blockLen=0;
    bool lResult = false;
    uint8_t BlockNumber=0, preBlockNum=0;
    uint8_t checksum=0;
    
    SYS_TICK lastTick;
    SYS_TICK currentTick;
    state = SM_FIRMWARE_UPDATE_SOH;
    lbDone = false;

    TCPIP_NET_HANDLE netH;


    netH = TCPIP_STACK_NetHandleGet("MRF24W");
 #if 0   
    if( BUTTON3_IO == 1u) return false;
    SYS_CONSOLE_MESSAGE("\n\rPress S2 (on Explorer16) to start the update.\n\r");
    while(BUTTON2_IO == 1u);
#else
    if(false == wf_update_begin_uart) return false;
    wf_update_begin_uart = false;
#endif


    delay_update(100);
    while(1 != AutoUpdate_Initialize_SM()){;}
    SYS_CONSOLE_MESSAGE("I am ready, Please transfer firmware patch by XMODEM.\r\n"); 
    lastTick = SYS_TICK_Get();
    do
    {
        currentTick = SYS_TICK_Get();
        if ( currentTick - lastTick >= (SYS_TICK_TicksPerSecondGet()*2) )
        {
            lastTick = SYS_TICK_Get();
            _SYS_CONSOLE_PUTC(XMODEM_NAK); //WriteUART(XMODEM_NAK);
        }

    } while(!_SYS_CONSOLE_DATA_RDY()); //(!DataRdyUART());

    
    while(!lbDone)
    {
        if(_SYS_CONSOLE_DATA_RDY()) //(DataRdyUART())
        {
            c = _SYS_CONSOLE_GETC();//ReadUART();
            lastTick = SYS_TICK_Get();
        }
        else
        {
            // put some timeout to make sure  that we do not wait forever.
             currentTick = SYS_TICK_Get();
            if ( currentTick - lastTick >= (SYS_TICK_TicksPerSecondGet()*10) )
            {
                //if time out, copy old patch image from bank2 to bank1
                SYS_CONSOLE_MESSAGE("timeout, revert begin...\r\n");
                AutoUpdate_Restore();
                SYS_CONSOLE_MESSAGE("revert done\r\n");
                return false;
            }
            continue;
        }
        //dbgPrintf("(%02x) ",c); 
        switch(state)
        {
        case SM_FIRMWARE_UPDATE_SOH:
            if(c == XMODEM_SOH)
            {
                state = SM_FIRMWARE_UPDATE_BLOCK;
                dbgPrintf("\r\n! ");
                checksum = c;
                lResult = true;
            }
            else if ( c == XMODEM_EOT )
            {
                state = SM_FIRMWARE_UPDATE_FINISH;
                
                // todo jw://while(BusyUART());
                _SYS_CONSOLE_PUTC(XMODEM_ACK); //WriteUART(XMODEM_ACK);
                lbDone = true;
            }  
            else
            {
                dbgPrintf("\n!error\n");
                while(1);
            }
            break;
        case SM_FIRMWARE_UPDATE_BLOCK:
            BlockNumber = c;
            dbgPrintf("BLK=%d ",BlockNumber);         
            checksum += c;
            state = SM_FIRMWARE_UPDATE_BLOCK_CMP;
            break;

        case SM_FIRMWARE_UPDATE_BLOCK_CMP:
            dbgPrintf("%d ",c);
            dbgPrintf("@:");
            //Judge: Is it correct ?
            if(c != (BlockNumber ^ 0xFF))
            {
                lResult = false;
                dbgPrintf("\nBLOCK_CMP err: %x,%x\n", c, BlockNumber ^ 0xFF );
            }
            else 
            {
                if((uint8_t)(preBlockNum+1) != BlockNumber)
                {
                    lResult = false;
                    dbgPrintf("\nBLOCK  err %x %x\n",preBlockNum+1,BlockNumber);
                }
            }
            checksum += c;
            blockLen = 0;
            state = SM_FIRMWARE_UPDATE_DATA;
            break;
        case SM_FIRMWARE_UPDATE_DATA:
            // Buffer block data until it is over.
            tempData[blockLen++] = c;
            if ( blockLen == XMODEM_BLOCK_LEN )
            {
                state = SM_FIRMWARE_UPDATE_CHECKSUM;
            }
            checksum += c;
            
            break;
        case SM_FIRMWARE_UPDATE_CHECKSUM:
            dbgPrintf("Checksum=%x=%x ",checksum,c);
            if(checksum != c)
            {
                lResult = false;
                dbgPrintf("\nchecksum  err\n");
            }
            XMODEM_SendToModule(tempData);
            if(lResult == true)
            {
                _SYS_CONSOLE_PUTC(XMODEM_ACK);//WriteUART(XMODEM_ACK);
                preBlockNum++;
            }
            else
            {
                _SYS_CONSOLE_PUTC(XMODEM_NAK);//WriteUART(XMODEM_NAK);
            }
            state = SM_FIRMWARE_UPDATE_SOH;
            break;

        default:
            dbgPrintf("\n!error\n");
            while(1);
            break;
        }

    }
    
    AutoUpdate_Completed();

    return true;
}

#endif

#endif // TCPIP_IF_MRF24W
