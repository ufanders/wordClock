/*******************************************************************************

Company:
    Microchip Technology Inc.

File Name:
    app.c

Summary:
    FreeRTOS and OPENRTOS Basic demo application file

 Important :
 This demo uses an evaluation license which is meant for demonstration only and 
 that customers desiring development and production on OPENRTOS must procure a
 suitable license.
 
 Description:
 This file implements a demo application that blinks the LED's
 on the PIC32 starter kit. The application creates four FreeRTOS tasks and
 one queue. Queue send task writes the data in the queue. queue receive tasks
 receives the data and toggle the LED. ISR block task initializes a timer and
 configures its period to 50ms. ISR block task is waiting on a sempahore which is
 given in timer interrupt. When interrupt occurs, interrupt handler gives the
 semaphore. The task waiting on this semaphore takes it and toggles the LED.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END


// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/* Standard includes. */
#include <stdio.h>
#include <xc.h>
#include "app.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* MPLAB Harmony includes. */
#include "peripheral/tmr/plib_tmr.h"
#include "peripheral/int/plib_int.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************

/* Priorities at which the tasks are created. */
#define	QUEUE_SEND_TASK_PRIORITY        ( tskIDLE_PRIORITY + 1 )
#define QUEUE_RECEIVE_TASK1_PRIORITY	( tskIDLE_PRIORITY + 2 )
#define QUEUE_RECEIVE_TASK2_PRIORITY	( tskIDLE_PRIORITY + 3 )
#define ISR_TASK_PRIORITY               ( tskIDLE_PRIORITY + 3 )

/* The rate at which data is sent to the queue.  The 200ms value is converted
to ticks using the portTICK_RATE_MS constant. */
#define QUEUE_SEND_FREQUENCY_MS		( 200 / portTICK_RATE_MS )

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define QUEUE_LENGTH			( 1 )

/* Values passed to the two tasks just to check the task parameter
functionality. */
#define QUEUE_SEND_PARAMETER		( 0x1111UL )
#define QUEUE_RECEIVE_PARAMETER		( 0x2222UL )

/* The period of the blinky software timer.  The period is specified in ms and
converted to ticks using the portTICK_RATE_MS constant. */
#define BLINKY_TIMER_PERIOD		( 50 / portTICK_RATE_MS )

/* The LED used by the communicating tasks and the blinky timer respectively. */
#define TASK1_LED			( 0 )
#define TASK2_LED			( 1 )
#define ISRTASK_LED                     ( 2 )

/* Misc. */
#define T5PRESCALAR                     ( 6 )
/*-----------------------------------------------------------*/

/*
 * The tasks as described in the comments at the top of this file.
 */
static void QueueReceiveTask1( void *pvParameters );
static void QueueReceiveTask2( void *pvParameters );
static void QueueSendTask( void *pvParameters );
static void ISRBlockTask( void *pvParameters );


/* The timer 5 interrupt handler.  As this interrupt uses the FreeRTOS assembly
entry point the IPL setting in the following function prototype has no effect. */
void __attribute__( (interrupt(ipl3), vector(_TIMER_5_VECTOR))) vT5InterruptWrapper( void );

/*-----------------------------------------------------------*/

/* The queue used by both tasks. */
static xQueueHandle xQueue = NULL;

/*-----------------------------------------------------------*/

/* The semaphore used to signal the ISRBlockTask */
static xSemaphoreHandle xBlockSemaphore;

/*-----------------------------------------------------------*/
/* The structure that is passed into tasks that use the ISRBlockTask() task function.
 The structure lets the task know which LED to toggle, and timer period. */
typedef struct xTASK_PARAMETER {
    uint16_t usLEDNumber;                   /* The number of the LED to toggle. */
    uint16_t timerPeriod;                   /* Timer 5 period */
} APPTaskParameter_t;
/*------------------------------------------------------------*/
/* Create an xTaskParameters_t structure  */
static const APPTaskParameter_t APPTaskParameters = {2 /* Toggle LED */, 31250 /* Timer 5 period. */};


/******************************************************************************
 *
 * APP_Initialize() creates one queue and four tasks. One task that sends the
 * data using FreeRTOS queue to the two tasks that waits for the data in queue.
 * QueueReceiveTask2 priority is higher than the  QueueReceiveTask1 priority.
 * QueueReceiveTask2 receives the data first and then sleeps for specified
 * time. QueueReceiveTask1 receives the next data since QueueReceiveTask2 is
 * not in running state.
 *
 */
void APP_Initialize( void )
{
    /* Create the queue. */
    xQueue = xQueueCreate( QUEUE_LENGTH, sizeof( unsigned long ) );

    if( xQueue != NULL )
    {
        /* Create the three tasks as described in the comments at the top of this file. */
        xTaskCreate(QueueReceiveTask1,                                /* The function that implements the task. */
                    ( signed char * ) "RxTask1",                      /* The text name assigned to the task - for debug only as it is not used by the kernel. */
                    configMINIMAL_STACK_SIZE,                        /* The size of the stack to allocate to the task. */
                    ( void * ) QUEUE_RECEIVE_PARAMETER,              /* The parameter passed to the task - just to check the functionality. */
                    QUEUE_RECEIVE_TASK1_PRIORITY,                    /* The priority assigned to the task. */
                    NULL );					    /* The task handle is not required, so NULL is passed. */
        /* Create the three tasks as described in the comments at the top of this file. */
        xTaskCreate(QueueReceiveTask2,
                    ( signed char * ) "RxTask2",
                    configMINIMAL_STACK_SIZE,
                    ( void * ) QUEUE_RECEIVE_PARAMETER,
                    QUEUE_RECEIVE_TASK2_PRIORITY,
                    NULL );

        xTaskCreate(QueueSendTask,
                    ( signed char * ) "TXtask",
                    configMINIMAL_STACK_SIZE,
                    ( void * ) QUEUE_SEND_PARAMETER,
                    QUEUE_SEND_TASK_PRIORITY,
                    NULL );

        xTaskCreate( ISRBlockTask,
                    ( signed char * ) "ISRtask",
                    configMINIMAL_STACK_SIZE,
                    ( void * ) &APPTaskParameters,
                    ISR_TASK_PRIORITY,
                    NULL );
    }
}

/*-----------------------------------------------------------*/
/*
 * The Queue Send Task:
 * The queue send task is implemented by the QueueSendTask() function in
 * this file.  QueueSendTask() sits in a loop that causes it to repeatedly
 * block for 200 milliseconds, before sending the values 100 and 1000 to the
 * queue that was created within APP_Initialize(). When the queue sends task
 * write the first value (100) to the queue, the high priority task 
 * QueueReceiveTask2() starts running, receives the data and sleeps for 100ms.
 * Now, queue send task writes the second value (1000) to the queue that makes
 * QueueReceiveTask1() running and QueueReceiveTask1() receives the data.
 * Once the values are sent, the task loops
 * back around to block for another 200 milliseconds.
 */
static void QueueSendTask( void *pvParameters )
{
    portTickType xNextWakeTime;
    const unsigned long ulValueToSend1 = 100UL;
    const unsigned long ulValueToSend2 = 1000UL;

    /* Check the task parameter is as expected. */
    configASSERT( ( ( unsigned long ) pvParameters ) == QUEUE_SEND_PARAMETER );

    /* Initialise xNextWakeTime - this only needs to be done once. */
    xNextWakeTime = xTaskGetTickCount();

    for( ;; )
    {
        /* Place this task in the blocked state until it is time to run again.
        The block time is specified in ticks, the constant used converts ticks
        to ms.  While in the Blocked state this task will not consume any CPU
        time. */
        vTaskDelayUntil( &xNextWakeTime, QUEUE_SEND_FREQUENCY_MS );

        /* Send to the queue - causing the queue receive task2 to unblock and
        toggle the LED.  0 is used as the block time so the sending operation
        will not block - it shouldn't need to block as the queue should always
        be empty at this point in the code. */
        xQueueSend( xQueue, &ulValueToSend1, 0U );
        /* Send to the queue - causing the queue receive task1 to unblock and
        toggle the LED.  0 is used as the block time so the sending operation
        will not block - it shouldn't need to block as the queue should always
        be empty at this point in the code. */
        xQueueSend( xQueue, &ulValueToSend2, 0U );
    }
}
/*-----------------------------------------------------------*/
/*
  * The Queue Receive Task:
 * The queue receive task is implemented by the QueueReceiveTask1() function
 * in this file.  QueueReceiveTask1() sits in a loop where it repeatedly
 * blocks on attempts to read data from the queue that was created within
 * APP_Initialize().  When data is received, the task checks the value of the
 * data, and if the value equals the expected 1000, toggles the LED.  The 'block
 * time' parameter passed to the queue receive function specifies that the
 * task should be held in the Blocked state indefinitely to wait for data to
 * be available on the queue.  The queue receive task will only leave the
 * Blocked state when the queue send task writes to the queue.  As the queue
 * send task writes to the queue every 200 milliseconds, the queue receive
 * task leaves the Blocked state every 200 milliseconds, and therefore toggles
 * the LED every 200 milliseconds.
 */
static void QueueReceiveTask1( void *pvParameters )
{
    unsigned long ulReceivedValue;

    /* Check the task parameter is as expected. */
    configASSERT( ( ( unsigned long ) pvParameters ) == QUEUE_RECEIVE_PARAMETER );

    for( ;; )
    {
        /* Wait until something arrives in the queue - this task will block
        indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
        FreeRTOSConfig.h. */
        xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );

        /*  To get here something must have been received from the queue, but
        is it the expected value?  If it is, toggle the LED. */
        if( ulReceivedValue == 1000UL )
        {
            BSP_ToggleLED( TASK1_LED );

            ulReceivedValue = 0U;
        }
    }
}
/*-----------------------------------------------------------*/
/*
  * The Queue Receive Task:
 * The queue receive task is implemented by the QueueReceiveTask2() function
 * in this file.  QueueReceiveTask2() sits in a loop where it repeatedly
 * blocks on attempts to read data from the queue that was created within
 * APP_Initialize().  When data is received, the task checks the value of the
 * data, and if the value equals the expected 100, toggles the LED and sleeps
 * for 100ms.  The 'block time' parameter passed to the queue receive function
 * specifies that the task should be held in the Blocked state indefinitely to
 * wait for data to be available on the queue.  The queue receive task will only
 * leave the Blocked state when the queue send task writes to the queue.  
 * As the queue send task writes to the queue every 200 milliseconds, the queue
 * receive task leaves the Blocked state every 200 milliseconds, and therefore
 * toggles the LED every 200 milliseconds.
 */
static void QueueReceiveTask2( void *pvParameters )
{
unsigned long ulReceivedValue;

    /* Check the task parameter is as expected. */
    configASSERT( ( ( unsigned long ) pvParameters ) == QUEUE_RECEIVE_PARAMETER );

    for( ;; )
    {
        /* Wait until something arrives in the queue - this task will block
        indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
        FreeRTOSConfig.h. */
        xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );

        /*  To get here something must have been received from the queue, but
        is it the expected value?  If it is, toggle the LED. */
        if( ulReceivedValue == 100UL )
        {
            BSP_ToggleLED( TASK2_LED );
            vTaskDelay((portTickType)ulReceivedValue);
            ulReceivedValue = 0U;
        }
    }
}
/*-----------------------------------------------------------*/
/*
 * The Timer ISR Test:
 * This demonstrates triggering an ISR from a peripheral timer. A task is created
 * which blocks on a semaphore. Separately a peripheral timer is set to cause an
 * interrupt every 50ms. The ISR handler (in a separate assembly file) then
 * releases the semaphore which causes the task to unblock and toggle an LED. This
 * sequence tests operation of the ISR and system stack handling.
 *
 */
static void ISRBlockTask( void* pvParameters )
{
  /* local variables marked as volatile so the compiler does not optimize them away */
    APPTaskParameter_t *pxTaskParameter;
   pxTaskParameter = (APPTaskParameter_t *) pvParameters;

   /* Create the semaphore used to signal this task */
    vSemaphoreCreateBinary( xBlockSemaphore );
    /* Set up timer 5 to generate an interrupt every 50 ms */
    PLIB_TMR_Counter16BitClear(TMR_ID_5);
    /* Timer 5 is going to interrupt at 20Hz Hz. (40,000,000 / (64 * 20) */
    PLIB_TMR_PrescaleSelect(TMR_ID_5, T5PRESCALAR);
    PLIB_TMR_Period16BitSet(TMR_ID_5, pxTaskParameter->timerPeriod);
    /* Clear the interrupt as a starting condition. */
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_5);
    /* Enable the interrupt. */
    PLIB_INT_SourceEnable(INT_ID_0,INT_SOURCE_TIMER_5);
    /* Start the timer. */
    PLIB_TMR_Start(TMR_ID_5);

    for( ;; )
    {
        /* block on the binary semaphore given by an ISR */
        xSemaphoreTake( xBlockSemaphore, portMAX_DELAY );

        BSP_ToggleLED( pxTaskParameter->usLEDNumber );

    }
}
/*-----------------------------------------------------------*/

void vT5InterruptHandler( void )
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* This function is the handler for the peripheral timer interrupt.
     The interrupt is initially signalled in a separate assembly file
     which switches to the system stack and then calls this function.
     It gives a semaphore which signals the prvISRBlockTask */
    xSemaphoreGiveFromISR( xBlockSemaphore, &xHigherPriorityTaskWoken );

    /* Clear the interrupt */
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_5);

    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/

