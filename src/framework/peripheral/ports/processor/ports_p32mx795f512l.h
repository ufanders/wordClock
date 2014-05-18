/* Created by plibgen $Revision: 1.30 $ */

#ifndef _PORTS_P32MX795F512L_H
#define _PORTS_P32MX795F512L_H

/* Section 1 - Enumerate instances, define constants, VREGs */

#include <xc.h>
#include <stdbool.h>

#include "peripheral/peripheral_common_32bit.h"

/* Default definition used for all API dispatch functions */
#ifndef PLIB_INLINE_API
    #define PLIB_INLINE_API extern inline
#endif

/* Default definition used for all other functions */
#ifndef PLIB_INLINE
    #define PLIB_INLINE extern inline
#endif

typedef enum {

    PORTS_ID_0 = 0,
    PORTS_NUMBER_OF_MODULES

} PORTS_MODULE_ID;

typedef enum {

    PORTS_PIN_MODE_ANALOG = 0,
    PORTS_PIN_MODE_DIGITAL = 1

} PORTS_PIN_MODE;

typedef enum {

    CN8 = 8,
    CN9 = 9,
    CN10 = 10,
    CN11 = 11,
    CN7 = 7,
    CN6 = 6,
    CN5 = 5,
    CN4 = 4,
    CN3 = 3,
    CN2 = 2,
    CN12 = 12,
    CN20 = 20,
    CN21 = 21,
    CN17 = 17,
    CN18 = 18,
    CN1 = 1,
    CN0 = 0,
    CN19 = 19,
    CN13 = 13,
    CN14 = 14,
    CN15 = 15,
    CN16 = 16

} PORTS_CHANGE_NOTICE_PIN;

typedef enum {

    PORTS_ANALOG_PIN_0 = 0,
    PORTS_ANALOG_PIN_1 = 1,
    PORTS_ANALOG_PIN_2 = 2,
    PORTS_ANALOG_PIN_3 = 3,
    PORTS_ANALOG_PIN_4 = 4,
    PORTS_ANALOG_PIN_5 = 5,
    PORTS_ANALOG_PIN_6 = 6,
    PORTS_ANALOG_PIN_7 = 7,
    PORTS_ANALOG_PIN_8 = 8,
    PORTS_ANALOG_PIN_9 = 9,
    PORTS_ANALOG_PIN_10 = 10,
    PORTS_ANALOG_PIN_11 = 11,
    PORTS_ANALOG_PIN_12 = 12,
    PORTS_ANALOG_PIN_13 = 13,
    PORTS_ANALOG_PIN_14 = 14,
    PORTS_ANALOG_PIN_15 = 15

} PORTS_ANALOG_PIN;

typedef enum {

    PORTS_BIT_POS_0 = 0,
    PORTS_BIT_POS_1 = 1,
    PORTS_BIT_POS_2 = 2,
    PORTS_BIT_POS_3 = 3,
    PORTS_BIT_POS_4 = 4,
    PORTS_BIT_POS_5 = 5,
    PORTS_BIT_POS_6 = 6,
    PORTS_BIT_POS_7 = 7,
    PORTS_BIT_POS_8 = 8,
    PORTS_BIT_POS_9 = 9,
    PORTS_BIT_POS_10 = 10,
    PORTS_BIT_POS_11 = 11,
    PORTS_BIT_POS_12 = 12,
    PORTS_BIT_POS_13 = 13,
    PORTS_BIT_POS_14 = 14,
    PORTS_BIT_POS_15 = 15

} PORTS_BIT_POS;

typedef enum {

    PORTS_REMAP_INPUT_FUNCTION_NONE

} PORTS_REMAP_INPUT_FUNCTION;

typedef enum {

    PORTS_REMAP_INPUT_PIN_NONE

} PORTS_REMAP_INPUT_PIN;

typedef enum {

    PORTS_REMAP_OUTPUT_FUNCTION_NONE

} PORTS_REMAP_OUTPUT_FUNCTION;

typedef enum {

    PORTS_REMAP_OUTPUT_PIN_NONE

} PORTS_REMAP_OUTPUT_PIN;

typedef enum {

    PORT_CHANNEL_A = 0x00,
    PORT_CHANNEL_B = 0x01,
    PORT_CHANNEL_C = 0x02,
    PORT_CHANNEL_D = 0x03,
    PORT_CHANNEL_E = 0x04,
    PORT_CHANNEL_F = 0x05,
    PORT_CHANNEL_G = 0x06

} PORTS_CHANNEL;

PLIB_INLINE SFR_TYPE* _PORTS_READ_A_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &PORTA;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_WRITE_A_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &LATA;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_DIRECTION_A_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &TRISA;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_OPEN_DRAIN_A_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &ODCA;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_READ_B_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &PORTB;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_WRITE_B_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &LATB;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_DIRECTION_B_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &TRISB;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_OPEN_DRAIN_B_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &ODCB;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_READ_C_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &PORTC;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_WRITE_C_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &LATC;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_DIRECTION_C_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &TRISC;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_OPEN_DRAIN_C_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &ODCC;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_READ_D_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &PORTD;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_WRITE_D_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &LATD;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_DIRECTION_D_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &TRISD;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_OPEN_DRAIN_D_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &ODCD;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_READ_E_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &PORTE;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_WRITE_E_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &LATE;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_DIRECTION_E_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &TRISE;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_OPEN_DRAIN_E_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &ODCE;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_READ_F_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &PORTF;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_WRITE_F_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &LATF;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_DIRECTION_F_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &TRISF;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_OPEN_DRAIN_F_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &ODCF;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_READ_G_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &PORTG;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_WRITE_G_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &LATG;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_DIRECTION_G_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &TRISG;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_OPEN_DRAIN_G_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &ODCG;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_PIN_MODE_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &AD1PCFG;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_CHANGE_NOTICE_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &CNCON;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_CHANGE_NOTICE_IN_IDLE_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &CNCON;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_PIN_CHANGE_NOTICE_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &CNEN;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PORTS_CHANGE_NOTICE_PULL_UP_VREG(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return &CNPUE;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_A_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_A_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_A_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_A_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_B_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_B_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_B_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_B_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_C_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_C_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_C_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_C_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_D_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_D_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_D_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_D_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_E_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_E_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_E_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_E_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_F_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_F_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_F_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_F_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_G_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_G_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_G_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_G_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_PIN_MODE_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)-1;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_CHANGE_NOTICE_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return _CNCON_ON_MASK;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_CHANGE_NOTICE_IN_IDLE_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return _CNCON_SIDL_MASK;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_PIN_CHANGE_NOTICE_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return _CNEN_CNEN0_MASK;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_CHANGE_NOTICE_PULL_UP_MASK(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return _CNPUE_CNPUE0_MASK;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_A_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_A_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_A_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_A_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_B_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_B_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_B_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_B_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_C_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_C_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_C_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_C_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_D_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_D_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_D_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_D_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_E_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_E_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_E_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_E_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_F_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_F_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_F_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_F_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_G_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_G_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_G_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_G_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_PIN_MODE_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)0;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_CHANGE_NOTICE_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return _CNCON_ON_POSITION;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_CHANGE_NOTICE_IN_IDLE_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return _CNCON_SIDL_POSITION;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_PIN_CHANGE_NOTICE_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return _CNEN_CNEN0_POSITION;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_CHANGE_NOTICE_PULL_UP_POS(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return _CNPUE_CNPUE0_POSITION;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_A_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_A_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_A_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_A_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_B_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_B_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_B_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_B_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_C_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_C_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_C_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_C_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_D_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_D_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_D_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_D_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_E_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_E_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_E_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_E_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_F_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_F_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_F_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_F_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_READ_G_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_WRITE_G_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_DIRECTION_G_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_OPEN_DRAIN_G_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_PIN_MODE_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_CHANGE_NOTICE_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return _CNCON_ON_LENGTH;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_CHANGE_NOTICE_IN_IDLE_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return _CNCON_SIDL_LENGTH;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_PIN_CHANGE_NOTICE_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return _CNEN_CNEN0_LENGTH;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PORTS_CHANGE_NOTICE_PULL_UP_LEN(PORTS_MODULE_ID i)
{
    switch (i) {
        case PORTS_ID_0 :
            return _CNPUE_CNPUE0_LENGTH;
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/ports_RemapInput_Unsupported.h"
#include "../templates/ports_RemapOutput_Unsupported.h"
#include "../templates/ports_PinMode_Non_PPS.h"
#include "../templates/ports_PortsRead_MCU32.h"
#include "../templates/ports_PortsWrite_MCU32.h"
#include "../templates/ports_PortsDirection_MCU32.h"
#include "../templates/ports_PortsOpenDrain_MCU32.h"
#include "../templates/ports_ChangeNotice_Default.h"
#include "../templates/ports_PinChangeNotice_Default.h"
#include "../templates/ports_ChangeNoticeInIdle_Default.h"
#include "../templates/ports_ChangeNoticePullup_Default.h"
#include "../templates/ports_PinModePerPort_Unsupported.h"
#include "../templates/ports_ChangeNoticePullDownPerPort_Unsupported.h"
#include "../templates/ports_ChangeNoticePullUpPerPort_Unsupported.h"
#include "../templates/ports_PinChangeNoticePerPort_Unsupported.h"
#include "../templates/ports_ChangeNoticePerPortTurnOn_Unsupported.h"
#include "../templates/ports_ChangeNoticeInIdlePerPort_Unsupported.h"
#include "../templates/ports_ChangeNoticePerPortStatus_Unsupported.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_RemapInput(PORTS_MODULE_ID index, PORTS_REMAP_INPUT_FUNCTION inputFunction, PORTS_REMAP_INPUT_PIN remapInputPin)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_RemapInput_Unsupported(index, inputFunction, remapInputPin);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PORTS_ExistsRemapInput(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ExistsRemapInput_Unsupported(index);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_RemapOutput(PORTS_MODULE_ID index, PORTS_REMAP_OUTPUT_FUNCTION outputFunction, PORTS_REMAP_OUTPUT_PIN remapOutputPin)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_RemapOutput_Unsupported(index, outputFunction, remapOutputPin);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PORTS_ExistsRemapOutput(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ExistsRemapOutput_Unsupported(index);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_PORTS_PinModeSelect(PORTS_MODULE_ID index, PORTS_ANALOG_PIN pin, PORTS_PIN_MODE mode)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_PinModeSelect_Non_PPS(index, pin, mode);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsPinMode(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ExistsPinMode_Non_PPS(index);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_PORTS_PinGet(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_PinGet_MCU32(index, channel, bitPos);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API PORTS_DATA_TYPE PLIB_PORTS_Read(PORTS_MODULE_ID index, PORTS_CHANNEL channel)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_Read_MCU32(index, channel);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (PORTS_DATA_TYPE)0;
    }
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsPortsRead(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ExistsPortsRead_MCU32(index);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_PORTS_PinWrite(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos, bool value)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_PinWrite_MCU32(index, channel, bitPos, value);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PORTS_PinSet(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_PinSet_MCU32(index, channel, bitPos);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PORTS_PinClear(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_PinClear_MCU32(index, channel, bitPos);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PORTS_PinToggle(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_PinToggle_MCU32(index, channel, bitPos);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PORTS_Write(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_TYPE value)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_Write_MCU32(index, channel, value);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PORTS_Set(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_TYPE value, PORTS_DATA_MASK mask)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_Set_MCU32(index, channel, value, mask);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PORTS_Toggle(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK toggleMask)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_Toggle_MCU32(index, channel, toggleMask);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PORTS_Clear(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK clearMask)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_Clear_MCU32(index, channel, clearMask);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsPortsWrite(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ExistsPortsWrite_MCU32(index);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_PORTS_PinDirectionInputSet(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_PinDirectionInputSet_MCU32(index, channel, bitPos);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PORTS_PinDirectionOutputSet(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_PinDirectionOutputSet_MCU32(index, channel, bitPos);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PORTS_DirectionInputSet(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK mask)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_DirectionInputSet_MCU32(index, channel, mask);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PORTS_DirectionOutputSet(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK mask)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_DirectionOutputSet_MCU32(index, channel, mask);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API PORTS_DATA_MASK PLIB_PORTS_DirectionGet(PORTS_MODULE_ID index, PORTS_CHANNEL channel)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_DirectionGet_MCU32(index, channel);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (PORTS_DATA_MASK)0;
    }
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsPortsDirection(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ExistsPortsDirection_MCU32(index);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_PORTS_PinOpenDrainEnable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_PinOpenDrainEnable_MCU32(index, channel, bitPos);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PORTS_PinOpenDrainDisable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_PinOpenDrainDisable_MCU32(index, channel, bitPos);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PORTS_OpenDrainEnable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK mask)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_OpenDrainEnable_MCU32(index, channel, mask);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PORTS_OpenDrainDisable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK mask)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_OpenDrainDisable_MCU32(index, channel, mask);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsPortsOpenDrain(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ExistsPortsOpenDrain_MCU32(index);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_PORTS_ChangeNoticeEnable(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_ChangeNoticeEnable_Default(index);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PORTS_ChangeNoticeDisable(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_ChangeNoticeDisable_Default(index);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsChangeNotice(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ExistsChangeNotice_Default(index);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_PORTS_PinChangeNoticeEnable(PORTS_MODULE_ID index, PORTS_CHANGE_NOTICE_PIN pinNum)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_PinChangeNoticeEnable_Default(index, pinNum);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PORTS_PinChangeNoticeDisable(PORTS_MODULE_ID index, PORTS_CHANGE_NOTICE_PIN pinNum)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_PinChangeNoticeDisable_Default(index, pinNum);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsPinChangeNotice(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ExistsPinChangeNotice_Default(index);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_PORTS_ChangeNoticeInIdleEnable(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_ChangeNoticeInIdleEnable_Default(index);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PORTS_ChangeNoticeInIdleDisable(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_ChangeNoticeInIdleDisable_Default(index);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsChangeNoticeInIdle(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ExistsChangeNoticeInIdle_Default(index);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_PORTS_ChangeNoticePullUpEnable(PORTS_MODULE_ID index, PORTS_CHANGE_NOTICE_PIN pinNum)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_ChangeNoticePullUpEnable_Default(index, pinNum);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PORTS_ChangeNoticePullUpDisable(PORTS_MODULE_ID index, PORTS_CHANGE_NOTICE_PIN pinNum)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_ChangeNoticePullUpDisable_Default(index, pinNum);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsChangeNoticePullUp(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ExistsChangeNoticePullUp_Default(index);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_PinModePerPortSelect(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos, PORTS_PIN_MODE mode)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_PinModePerPortSelect_Unsupported(index, channel, bitPos, mode);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PORTS_ExistsPinModePerPort(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ExistsPinModePerPort_Unsupported(index);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_ChangeNoticePullDownPerPortEnable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_ChangeNoticePullDownPerPortEnable_Unsupported(index, channel, bitPos);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_ChangeNoticePullDownPerPortDisable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_ChangeNoticePullDownPerPortDisable_Unsupported(index, channel, bitPos);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PORTS_ExistsChangeNoticePullDownPerPort(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ExistsChangeNoticePullDownPerPort_Unsupported(index);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_ChangeNoticePullUpPerPortEnable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_ChangeNoticePullUpPerPortEnable_Unsupported(index, channel, bitPos);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_ChangeNoticePullUpPerPortDisable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_ChangeNoticePullUpPerPortDisable_Unsupported(index, channel, bitPos);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PORTS_ExistsChangeNoticePullUpPerPort(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ExistsChangeNoticePullUpPerPort_Unsupported(index);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_PinChangeNoticePerPortEnable_Unsupported(index, channel, bitPos);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_PinChangeNoticePerPortDisable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_PinChangeNoticePerPortDisable_Unsupported(index, channel, bitPos);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PORTS_ExistsPinChangeNoticePerPort(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ExistsPinChangeNoticePerPort_Unsupported(index);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_ChangeNoticePerPortTurnOn(PORTS_MODULE_ID index, PORTS_CHANNEL channel)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_ChangeNoticePerPortTurnOn_Unsupported(index, channel);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_ChangeNoticePerPortTurnOff(PORTS_MODULE_ID index, PORTS_CHANNEL channel)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_ChangeNoticePerPortTurnOff_Unsupported(index, channel);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PORTS_ExistsChangeNoticePerPortTurnOn(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ExistsChangeNoticePerPortTurnOn_Unsupported(index);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_ChangeNoticeInIdlePerPortEnable(PORTS_MODULE_ID index, PORTS_CHANNEL channel)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_ChangeNoticeInIdlePerPortEnable_Unsupported(index, channel);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_ChangeNoticeInIdlePerPortDisable(PORTS_MODULE_ID index, PORTS_CHANNEL channel)
{
    switch (index) {
        case PORTS_ID_0 :
            PORTS_ChangeNoticeInIdlePerPortDisable_Unsupported(index, channel);
            break;
        case PORTS_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PORTS_ExistsChangeNoticePerPortInIdle(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ExistsChangeNoticePerPortInIdle_Unsupported(index);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PORTS_ChangeNoticePerPortHasOccured(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ChangeNoticePerPortHasOccured_Unsupported(index, channel, bitPos);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PORTS_ExistsChangeNoticePerPortStatus(PORTS_MODULE_ID index)
{
    switch (index) {
        case PORTS_ID_0 :
            return PORTS_ExistsChangeNoticePerPortStatus_Unsupported(index);
        case PORTS_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

#endif
