/*******************************************************************************
  Linked list helper file

  Company:
    Microchip Technology Inc.
    
  File Name:
    link_list.h

  Summary:
    Linked lists manipulation Interface Header
    
  Description:
    This header file contains the function prototypes and definitions of the 
    linked lists manipulation routines
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

#ifndef _LINK_LISTS_H_
#define _LINK_LISTS_H_

#include <stdbool.h>
#include "osal/osal.h"

typedef struct _TAG_SGL_LIST_NODE
{
	struct _TAG_SGL_LIST_NODE*		next;		// next node in list
    void*                           data[0];    // generic payload    
}SGL_LIST_NODE;	// generic linked list node definition


typedef struct
{
	SGL_LIST_NODE*	head;	// list head
	SGL_LIST_NODE*	tail;
    int             nNodes; // number of nodes in the list

}SINGLE_LIST;	// single linked list


/////  single linked lists manipulation ///////////
//


void  TCPIP_Helper_SingleListInitialize(SINGLE_LIST* pL);


extern __inline__ bool __attribute__((always_inline)) TCPIP_Helper_SingleListIsEmpty(SINGLE_LIST* pL)
{
    return pL->head == 0;
}


extern __inline__ int __attribute__((always_inline)) TCPIP_Helper_SingleListCount(SINGLE_LIST* pL)
{
    return pL->nNodes;
}

void  TCPIP_Helper_SingleListHeadAdd(SINGLE_LIST* pL, SGL_LIST_NODE* pN);

void  TCPIP_Helper_SingleListTailAdd(SINGLE_LIST* pL, SGL_LIST_NODE* pN);


// insertion in the middle, not head or tail
void  TCPIP_Helper_SingleListMidAdd(SINGLE_LIST* pL, SGL_LIST_NODE* pN, SGL_LIST_NODE* after);


// removes the head node
SGL_LIST_NODE*  TCPIP_Helper_SingleListHeadRemove(SINGLE_LIST* pL);

// removes the next node (following prev) in the list
// if prev == 0 removed the head
SGL_LIST_NODE*  TCPIP_Helper_SingleListNextRemove(SINGLE_LIST* pL, SGL_LIST_NODE* prev);


// removes a node anywhere in the list
// Note: this is lengthy!
// Use a double linked list if faster operation needed!
SGL_LIST_NODE*  TCPIP_Helper_SingleListNodeRemove(SINGLE_LIST* pL, SGL_LIST_NODE* pN);


void  TCPIP_Helper_SingleListAppend(SINGLE_LIST* pL, SINGLE_LIST* pAList);


extern __inline__ void __attribute__((always_inline)) TCPIP_Helper_SingleListDelete(SINGLE_LIST* pL)
{
	while((TCPIP_Helper_SingleListHeadRemove(pL)));
}

// Singling licked protected list /////
typedef struct
{
    SINGLE_LIST list;
    OSAL_SEM_HANDLE_TYPE semaphore;
} PROTECTED_SINGLE_LIST;

void  TCPIP_Helper_ProtectedSingleListInitialize(PROTECTED_SINGLE_LIST* pL);


extern __inline__ bool __attribute__((always_inline)) TCPIP_Helper_ProtectedSingleListIsEmpty(PROTECTED_SINGLE_LIST* pL)
{
    return TCPIP_Helper_SingleListIsEmpty(&pL->list);
}


extern __inline__ int __attribute__((always_inline)) TCPIP_Helper_ProtectedSingleListCount(PROTECTED_SINGLE_LIST* pL)
{
    return TCPIP_Helper_SingleListCount(&pL->list);
}

void  TCPIP_Helper_ProtectedSingleListHeadAdd(PROTECTED_SINGLE_LIST* pL, SGL_LIST_NODE* pN);

void  TCPIP_Helper_ProtectedSingleListTailAdd(PROTECTED_SINGLE_LIST* pL, SGL_LIST_NODE* pN);


// insertion in the middle, not head or tail
void  TCPIP_Helper_ProtectedSingleListMidAdd(PROTECTED_SINGLE_LIST* pL, SGL_LIST_NODE* pN, SGL_LIST_NODE* after);


// removes the head node
SGL_LIST_NODE*  TCPIP_Helper_ProtectedSingleListHeadRemove(PROTECTED_SINGLE_LIST* pL);

// removes the next node (following prev) in the list
// if prev == 0 removed the head
SGL_LIST_NODE*  TCPIP_Helper_ProtectedSingleListNextRemove(PROTECTED_SINGLE_LIST* pL, SGL_LIST_NODE* prev);


// removes a node anywhere in the list
// Note: this is lengthy!
// Use a double linked list if faster operation needed!
SGL_LIST_NODE*  TCPIP_Helper_ProtectedSingleListNodeRemove(PROTECTED_SINGLE_LIST* pL, SGL_LIST_NODE* pN);


void  TCPIP_Helper_ProtectedSingleListAppend(PROTECTED_SINGLE_LIST* pL, SINGLE_LIST* pAList);


void TCPIP_Helper_ProtectedSingleListDelete(PROTECTED_SINGLE_LIST* pL);



/////  double linked lists manipulation ///////////
//

typedef struct _TAG_DBL_LIST_NODE
{
	struct _TAG_DBL_LIST_NODE*		next;		// next node in list
	struct _TAG_DBL_LIST_NODE*		prev;		// prev node in list
    void*                           data[0];    // generic payload    
}DBL_LIST_NODE;	// generic linked list node definition


typedef struct
{
	DBL_LIST_NODE*	head;	// list head
	DBL_LIST_NODE*	tail;   // list tail;
    int             nNodes; // number of nodes in the list 
}DOUBLE_LIST;	// double linked list


void  TCPIP_Helper_DoubleListInitialize(DOUBLE_LIST* pL);


extern __inline__ bool __attribute__((always_inline)) TCPIP_Helper_DoubleListIsEmpty(DOUBLE_LIST* pL)
{
    return pL->head == 0;
}

extern __inline__ int __attribute__((always_inline)) TCPIP_Helper_DoubleListCount(DOUBLE_LIST* pL)
{
    return pL->nNodes;
}

void  TCPIP_Helper_DoubleListHeadAdd(DOUBLE_LIST* pL, DBL_LIST_NODE* pN);

void  TCPIP_Helper_DoubleListTailAdd(DOUBLE_LIST* pL, DBL_LIST_NODE* pN);

// add node pN in the middle, after existing node "after"
void  TCPIP_Helper_DoubleListMidAdd(DOUBLE_LIST* pL, DBL_LIST_NODE* pN, DBL_LIST_NODE* after);

DBL_LIST_NODE*  TCPIP_Helper_DoubleListHeadRemove(DOUBLE_LIST* pL);

DBL_LIST_NODE*  TCPIP_Helper_DoubleListTailRemove(DOUBLE_LIST* pL);

// remove existing node, neither head, nor tail
void  TCPIP_Helper_DoubleListMidRemove(DOUBLE_LIST* pL, DBL_LIST_NODE* pN);

void  TCPIP_Helper_DoubleListNodeRemove(DOUBLE_LIST* pL, DBL_LIST_NODE* pN);

extern __inline__ void __attribute__((always_inline)) TCPIP_Helper_DoubleListDelete(DOUBLE_LIST* pL)
{
    while((TCPIP_Helper_DoubleListHeadRemove(pL)));
}


typedef struct
{
    DOUBLE_LIST list;
    OSAL_SEM_HANDLE_TYPE semaphore;
}PROTECTED_DOUBLE_LIST;	// double linked list


void  TCPIP_Helper_ProtectedDoubleListInitialize(PROTECTED_DOUBLE_LIST* pL);


extern __inline__ bool __attribute__((always_inline)) TCPIP_Helper_ProtectedDoubleListIsEmpty(PROTECTED_DOUBLE_LIST* pL)
{
    return TCPIP_Helper_DoubleListIsEmpty(&pL->list);
}

extern __inline__ int __attribute__((always_inline)) TCPIP_Helper_ProtectedDoubleListCount(PROTECTED_DOUBLE_LIST* pL)
{
    return TCPIP_Helper_DoubleListCount(&pL->list);
}

void  TCPIP_Helper_ProtectedDoubleListHeadAdd(PROTECTED_DOUBLE_LIST* pL, DBL_LIST_NODE* pN);

void  TCPIP_Helper_ProtectedDoubleListTailAdd(PROTECTED_DOUBLE_LIST* pL, DBL_LIST_NODE* pN);

// add node pN in the middle, after existing node "after"
void  TCPIP_Helper_ProtectedDoubleListMidAdd(PROTECTED_DOUBLE_LIST* pL, DBL_LIST_NODE* pN, DBL_LIST_NODE* after);

DBL_LIST_NODE*  TCPIP_Helper_ProtectedDoubleListHeadRemove(PROTECTED_DOUBLE_LIST* pL);

DBL_LIST_NODE*  TCPIP_Helper_ProtectedDoubleListTailRemove(PROTECTED_DOUBLE_LIST* pL);

// remove existing node, neither head, nor tail
void  TCPIP_Helper_ProtectedDoubleListMidRemove(PROTECTED_DOUBLE_LIST* pL, DBL_LIST_NODE* pN);

void  TCPIP_Helper_ProtectedDoubleListNodeRemove(PROTECTED_DOUBLE_LIST* pL, DBL_LIST_NODE* pN);

void TCPIP_Helper_ProtectedDoubleListDelete(PROTECTED_DOUBLE_LIST* pL);


#endif //  _LINK_LISTS_H_


