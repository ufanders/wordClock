/*******************************************************************************

  Summary:

  Description:

*******************************************************************************/

/*******************************************************************************
File Name:  usart.c
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
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PzUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/

#include <xc.h>

#include "system_config.h"

#if defined (SYS_CONSOLE_ENABLE) || defined (SYS_DEBUG_ENABLE)

#if defined(__PIC32MX__)
#include <p32xxxx.h>
#elif defined(__PIC24F__)
#include <p24fxxxx.h>
#elif defined(__PIC24H__)
#include <p24hxxxx.h>
#elif defined(__dsPIC33F__)
#include <p33fxxxx.h>
#elif defined(__dsPIC33E__)
#include <p33exxxx.h>
#endif


#if defined(__C32__)
#define  USART_REG volatile uint32_t
#define  USART_MODE_CTRL volatile __U1MODEbits_t
#define  USART_STAT_CTRL volatile __U1STAbits_t
#else
#define  USART_REG volatile uint16_t
#define  USART_MODE_CTRL volatile UxMODEBITS
#define  USART_STAT_CTRL volatile UxSTABITS
#endif

#include "system_config.h"
#include "system/clk/src/sys_clk_local.h"

typedef struct __usart_module_ctrl
{
    USART_REG *UxMODE;
    USART_REG *UxSTA;
    USART_REG *UxTXREG;
    USART_REG *UxRXREG;
    USART_REG *UxBRG;
} USART_MODULE_CTRL;

// supported ports
static const USART_MODULE_CTRL usart_module_ctrl[] =
{
    {(USART_REG*)&U1MODE, (USART_REG*)&U1STA, (USART_REG*)&U1TXREG, (USART_REG*)&U1RXREG, (USART_REG*)&U1BRG},
    {(USART_REG*)&U2MODE, (USART_REG*)&U2STA, (USART_REG*)&U2TXREG, (USART_REG*)&U2RXREG, (USART_REG*)&U2BRG},
//  {&U3MODE, &U3STA, &U3TXREG, &U3RXREG, &U3BRG},
//  {&U4MODE, &U4STA, &U4TXREG, &U4RXREG, &U4BRG},
};


// prototypes
bool                USART_INIT(int port_no, unsigned int baud_rate);
_SYS_CDEV_HANDLE    USART_OPEN(int port_no);
void                USART_PRINT(_SYS_CDEV_HANDLE u_handle, const char* str);
bool                USART_DATA_RDY(_SYS_CDEV_HANDLE u_handle);
char                USART_GETC(_SYS_CDEV_HANDLE u_handle);
int                 USART_GETS(_SYS_CDEV_HANDLE u_handle, char* buff, int buff_len);
void                USART_PUTC(_SYS_CDEV_HANDLE u_handle, char c);


// object implementing the USART character device
_SYS_CDEV_OBJ _usart_cdev_obj =
{
    USART_INIT,     // _SYS_CDEV_INIT  init;
    USART_OPEN,     // _SYS_CDEV_OPEN  open;
    USART_PRINT,    // _SYS_CDEV_PRINT print;
    USART_PUTC,     // _SYS_CDEV_PUTC  putc;
    USART_DATA_RDY, // _SYS_CDEV_DATA_RDY  isRdy;
    USART_GETC,     // _SYS_CDEV_GETC      getc;;
    USART_GETS,     // _SYS_CDEV_GETS      gets;
};


// *****************************************************************************
// *****************************************************************************
// Section: USART Functions
// *****************************************************************************
// *****************************************************************************


#define _USART_BAUD_RATE_FACTOR 4

bool USART_INIT(int port_no, unsigned int baud_rate)
{
#if defined (__PIC32MZ__)
#if defined (UART1)
    /* UART1 BRG */
    U1BRG = 0x35;
    /* UART1 RX/TX */
    ANSELDCLR = 0x0400;
    ANSELGCLR = 0x0080;
    ANSELBCLR = 0x0004;
    TRISBCLR = 0x0004;
    LATBCLR = 0x0004;
    RPG7R = 0x01;
    U1RXR = 0x03;

    /* UART2 TX Enable */
    U1MODE = 0x8000;
    U1STAbits.UTXEN = 1;
    U1STAbits.URXEN = 1;
#else

    /* UART2 BRG */
    U2MODEbits.UEN = 0;

    ANSELBCLR = 0xC004;
    ANSELGCLR = 0x0040;
    TRISBCLR = 0x0004;
    LATBCLR = 0x0004;
    RPB14R = 0x2;        // B14 is u2Txd
#if defined (MEBII)
    U2RXR=0x1;
#else
	U2RXR=0x3;           // RXd on pin B15
#endif
    U2BRG = 0x35;

    /* UART2 TX Enable */
    U2STAbits.UTXEN = 1;
    U2STAbits.URXEN = 1;
    U2MODEbits.ON = 1;

#endif

    return true;
#else
    const USART_MODULE_CTRL *pusart_module_ctrl;

    if((unsigned int)port_no >= sizeof(usart_module_ctrl)/sizeof(*usart_module_ctrl))
    {   // invalid/not supported port
        return false;
    }

    pusart_module_ctrl = usart_module_ctrl + port_no;
    *(pusart_module_ctrl->UxBRG) = (((clkObject.systemClock)/(baud_rate)/_USART_BAUD_RATE_FACTOR) - 1);
    *(pusart_module_ctrl->UxMODE) = 0;

    ((USART_MODE_CTRL*)(pusart_module_ctrl->UxMODE))->BRGH = 1;
    *(pusart_module_ctrl->UxSTA) = 0;
    ((USART_MODE_CTRL*)(pusart_module_ctrl->UxMODE))->UARTEN = 1;
    ((USART_MODE_CTRL*)(pusart_module_ctrl->UxMODE))->STSEL = 0;
    ((USART_STAT_CTRL*)(pusart_module_ctrl->UxSTA))->UTXEN = 1;
    #if defined (__C32__)
        ((USART_STAT_CTRL*)(pusart_module_ctrl->UxSTA))->URXEN = 1;
    #endif
    ((USART_STAT_CTRL*)(pusart_module_ctrl->UxSTA))->OERR = 0;

    return true;
#endif
}

_SYS_CDEV_HANDLE USART_OPEN(int port_no)
{
    if((unsigned int)port_no < sizeof(usart_module_ctrl)/sizeof(*usart_module_ctrl))
    {
        return usart_module_ctrl + port_no;
    }
    return 0;
}

void USART_PRINT(_SYS_CDEV_HANDLE u_handle, const char* str)
{
    unsigned char c;
    const USART_MODULE_CTRL* pusart_module_ctrl = (const USART_MODULE_CTRL*)u_handle;

    while((c = *str++))
    {
        while(((USART_STAT_CTRL*)(pusart_module_ctrl->UxSTA))->UTXBF == 1);
        *(pusart_module_ctrl->UxTXREG) = c;
    }
}

bool USART_DATA_RDY(_SYS_CDEV_HANDLE u_handle)
{
    const USART_MODULE_CTRL* pusart_module_ctrl = (const USART_MODULE_CTRL*)u_handle;
    return (((USART_STAT_CTRL*)(pusart_module_ctrl->UxSTA))->URXDA == 1);
}

char USART_GETC(_SYS_CDEV_HANDLE u_handle)
{
    const USART_MODULE_CTRL* pusart_module_ctrl = (const USART_MODULE_CTRL*)u_handle;

    while(((USART_STAT_CTRL*)(pusart_module_ctrl->UxSTA))->URXDA != 1);
    return(*(pusart_module_ctrl->UxRXREG));
}


int USART_GETS(_SYS_CDEV_HANDLE u_handle, char* buff, int buff_len)
{
    int i=0;

    do
    {
        *buff = USART_GETC(u_handle);
    }while((*buff++ != '\n') && (i++ < buff_len));

    return(i);
}

void USART_PUTC(_SYS_CDEV_HANDLE u_handle, char c)
{
    const USART_MODULE_CTRL* pusart_module_ctrl = (const USART_MODULE_CTRL*)u_handle;

    while(((USART_STAT_CTRL*)(pusart_module_ctrl->UxSTA))->UTXBF == 1);
    *(pusart_module_ctrl->UxTXREG) = c;
}




#endif  // defined (SYS_CONSOLE_ENABLE) || defined (SYS_DEBUG_ENABLE)

