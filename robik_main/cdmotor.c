/******************************************************************************

  Robik - Rubik's cube solver from junk
  =====================================

  cdmotor.c

  Color detector motor controlling module
  Contains CdMotorTask

  18.10.2017

*****************************************************************************/

#include <stdlib.h>
#include <stdbool.h>

#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_exti.h>
#include <misc.h>

#include "tinyprintf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "common.h"
#include "tilter.h"
#include "cdmotor.h"

//#define DEBUG

/* =====================================================================
------------------------ Constants & macros ------------------------- */

#define TASK_STACK_SIZE     90         // Stack size in words
#define TASK_PRIORITY       1

#define CD_INDEX_INT_PRIORITY 12			// one higher than step int

#define MOVE_MAX_WAIT		(5000/portTICK_RATE_MS)

#define ST_RIGHT            -1
#define ST_STOP              0
#define ST_LEFT              1

#define INIT_SPEED           13000

/* =====================================================================
------------------------  Global variables  ------------------------- */

// --- Move sequences

static const int16_t from_rest_to_center[] = { 23,      END_OF_SEQ };
static const int16_t from_center_to_edge[] = { 115, 54, END_OF_SEQ };
static const int16_t from_edge_to_corner[] = { 125, 66, END_OF_SEQ };
static const int16_t from_edge_to_rest[]   = { 370,     END_OF_SEQ };

static const int16_t *seq_tbl[] = {
	cli_seq_tbl,			// test sequence from CLI
    from_rest_to_center,
    from_center_to_edge,
    from_edge_to_corner,
    from_edge_to_rest
};

static int16_t single_seq[] = { 0, END_OF_SEQ };

// --- Stepper motor handling

static SemaphoreHandle_t EndMotionSem = NULL;
static SemaphoreHandle_t IndexSem = NULL;

static int16_t step_dir = ST_STOP;
static int16_t pos_ctr = 0;
static uint8_t acc_idx = 0;
static bool accel = true;
static uint8_t max_acc_idx = 255;
static int16_t slew_cnt;
static uint8_t par_a = 254;
static uint8_t par_b = 60;
static bool slow_speed = false;

// step pattern table, half step
static const uint16_t step_tbl[] = {
			//  PD6 PD5 PD4
			//  I   PB  PA  =>  I   PB  PA
	0x00,   //  0   0   0       on  -   -
	0x10,   //  0   0   1       on  -   +
	0x30,   //  0   1   1       on  +   +
	0x20    //  0   1   0       on  +   -

};

#define STEP_TBL_MASK           0x03
#define STEPPER_STOP            0x70  //  off +   +

#define STEPPER_CONTROL_MASK  	0x00700070

extern const uint16_t AccTable[];		// acceleration table in tilter.c

/* =====================================================================
------------------------ Function prototypes ------------------------ */

static void SearchHomePos( void );
static void SetIndexInterrupt(FunctionalState state);
static void StartStepping(void);
static void StopStepping(void);
void InitTimerInterrupt(void);

/* =====================================================================
Motion task.
--------------------------------------------------------------------- */

static void CdMotorTask(void *pvParameters)
{
   	MOTION_MSG_T msg;
   	const int16_t *seq;
	int16_t len;
    int i;

	StepperOut(STEPPER_STOP,STEPPER_CONTROL_MASK);

	Delay(500);

    // Main loop.
    while(1)
    {
		// Wait for a command (blocks here)
        if (xQueueReceive(CdmQueue, &msg, portMAX_DELAY) == pdPASS)
		{
#ifdef DEBUG
            printf("CdmTask: got cmd: %d:%d\r\n",msg.type,msg.value);
#endif
            seq = NULL;
            
            switch (msg.type)
			{
            case CDMC_SEQ:
                seq = seq_tbl[msg.value];
                break;
                
			case CDMC_INIT:
				// start initializing
#ifdef DEBUG
				printf("CdmTask: start init\r\n");
#endif
				SearchHomePos();
                
                // drive to rest position after init
                single_seq[0] = 370;
                seq = single_seq;
                break;
                
            case CDMC_POS:
                // drive to given position
                single_seq[0] = msg.value;
                seq = single_seq;
                break;
                
			case CDMC_SYNC:
				xSemaphoreGive(MotionSyncSem);
				break;

			case CDMC_COEFF:
				par_a = msg.value;
				break;

			case CDMC_LIMIT:
				par_b = msg.value;
				break;

			case CDMC_SHOW_POS:
				printf("pos=%d\r\n",pos_ctr);
				break;
            }

            if (seq != NULL)
            {
            	// execute the sequence
				for (i = 0; seq[i] != END_OF_SEQ; ++i)
                {
                    // calculate movement direction and length in steps
                    if (pos_ctr > seq[i])
                    {
                        step_dir = ST_LEFT;
                        len = pos_ctr - seq[i];
                    }
                    else
                    {
                        step_dir = ST_RIGHT;
                        len = seq[i] - pos_ctr;
                    }

                    // ramp len is 1/4 of move length or 256 steps, whichever is smaller
                    if (len < 1024)
                        max_acc_idx = len / 4;
                    else
                        max_acc_idx = 255;

                    // par_b limits the maximum velocity
                    if (max_acc_idx > par_b)
                        max_acc_idx = par_b;

                    // length of slewing (driving at max velocity)
                    slew_cnt = len - 2*max_acc_idx - 1;

                    acc_idx = 0;	// start in AccTable[0]
                    accel = true;	// first we are accelerating

                    StartStepping();	// enable stepper interrupt
                    
                    // wait for movement end (blocks here)
                    if (xSemaphoreTake(EndMotionSem, MOVE_MAX_WAIT) == pdPASS)
                    {
                        StopStepping();	// disable stepper interrupt
#ifdef DEBUG
                        printf("stop stepping\r\n");
#endif
                    }
                    else
                    {
                        error_code |= CDM_MOTION_ERROR;
#ifdef DEBUG
                        printf("waiting EndMotionSem failed\r\n");
#endif
                    }
                }
            }
        }
    }
}

/* =====================================================================
Search home position
--------------------------------------------------------------------- */

#define MAX_SEARCH1_TIME	(700/portTICK_RATE_MS)
#define MAX_SEARCH2_TIME	(3000/portTICK_RATE_MS)
#define ADD_STEP_TIME   	(500/portTICK_RATE_MS)

static void SearchHomePos( void )
{
	bool search_again;

	slow_speed = true;	// use slow speed (short ramp)

	do {
		search_again = false;

		// Assume we are right from index: start moving left
		step_dir = ST_LEFT;
        acc_idx = 0;

		// enable index interrupt and start motor
		SetIndexInterrupt(ENABLE);
		StartStepping();

		// wait for index interrupt (blocks here)
		if (xSemaphoreTake(IndexSem, MAX_SEARCH1_TIME) == pdPASS)
		{
			pos_ctr = 310;		// index is at 310 steps from left
#ifdef DEBUG
			printf("index found\r\n");
#endif
		}
		else
		{	// index not found in time, search to right
			step_dir = ST_RIGHT;

			// wait for index interrupt (blocks here)
			if (xSemaphoreTake(IndexSem, MAX_SEARCH2_TIME) == pdPASS)
			{
				// found, step over index and do searching again
				// after this we know we are right from index
				vTaskDelay(ADD_STEP_TIME);
				search_again = true;
			}
			else
				error_code |= CDM_INDEX_ERROR;	// index not found
		}

		// end movement and disable index interrupt
		StopStepping();
		StepperOut(STEPPER_STOP,STEPPER_CONTROL_MASK);
		SetIndexInterrupt(DISABLE);
	}
	while (search_again);

	slow_speed = false;
}

/* =====================================================================
Module initialization function. Create queues and task.
--------------------------------------------------------------------- */

uint32_t CdMotorInit(void)
{
    // Queue for commands
	CdmQueue = xQueueCreate(CDM_QUEUE_SIZE, sizeof(MOTION_MSG_T));
	if (CdmQueue == NULL)
    {
        return(1);
    }

	// Semaphore for end of movement
	EndMotionSem = xSemaphoreCreateBinary();
	if (EndMotionSem == NULL)
    {
        return(1);
    }
    
    // Semaphore for index
	IndexSem = xSemaphoreCreateBinary();
	if (IndexSem == NULL)
    {
        return(1);
    }

    // Create the task.
    if(xTaskCreate(CdMotorTask, "CDM", TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + TASK_PRIORITY, NULL) != pdTRUE)
    {
        return(1);
    }

    // Success.
    return(0);
}

/* =======================================================================
Enable/disable EXTI10 interrupt, from cd motor opto
----------------------------------------------------------------------- */

static void SetIndexInterrupt(FunctionalState state)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

    EXTI_ClearITPendingBit(EXTI_Line10 | EXTI_Line11 | EXTI_Line12 | EXTI_Line13 | EXTI_Line14 | EXTI_Line15 );

	// configure NVIC
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CD_INDEX_INT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = state;
	NVIC_Init(&NVIC_InitStructure);

	// Configure EXTI line
	EXTI_InitStructure.EXTI_Line = EXTI_Line10;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = state;
	EXTI_Init(&EXTI_InitStructure);
}

/* =======================================================================
Start stepping clock, TIM3 CH2
----------------------------------------------------------------------- */

static void StartStepping(void)
{
    uint16_t cur_ctr;

    cur_ctr = TIM_GetCounter(TIM3);
    TIM_SetCompare2(TIM3, cur_ctr + 200);	// first interrupt after 100 us

    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
    TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
}

/* =======================================================================
Stop stepping clock
----------------------------------------------------------------------- */

static void StopStepping(void)
{
    TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
}

/* =======================================================================
Feeds next step to stepper motor. Called from TIM3 interrupt.
----------------------------------------------------------------------- */

void StepColdetMotor(void)
{
	static int16_t cur_step = 0;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint16_t capture;
	uint32_t offset;

	TEST_ON;

    capture = TIM_GetCapture2(TIM3);

    // feed next step to stepper port
    cur_step = (cur_step + step_dir) & STEP_TBL_MASK;
	StepperOut(step_tbl[cur_step],STEPPER_CONTROL_MASK);

    // update position counter
    pos_ctr -= step_dir;

    // calculate next interrupt interval
    offset = ((uint32_t)par_a * (uint32_t)AccTable[acc_idx]) >> 8;
    TIM_SetCompare2(TIM3, capture + offset);

    if (slow_speed)
    {
        // drive with slow speed, short ramp (during index searching)
		if (acc_idx < 20)
			acc_idx++;
    }
    else  // normal operation, use ramps
    {
        // calculate next position in AccTable[]
        if (accel && acc_idx < max_acc_idx)
        {
            // in the middle of acceleration ramp => next position in table
            acc_idx++;
        }
        else if (!accel && acc_idx != 0)
        {
            // in the middle of deceleration ramp => next position in table
            acc_idx--;
        }
        else
        {	// slewing => update slew count
            // if slew end, start decelaration ramp
            if (slew_cnt == 0)
                accel = false;
            else
                slew_cnt--;
        }

        // if slew has ended and deceleration has ended => movement has ended
        if (slew_cnt == 0 && acc_idx == 0)
        {
            step_dir = ST_STOP;
			StepperOut(STEPPER_STOP,STEPPER_CONTROL_MASK);

            xSemaphoreGiveFromISR(EndMotionSem, &xHigherPriorityTaskWoken);
            portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
        }
    }

	TEST_OFF;
}

/* =======================================================================
Index interrupt handler
----------------------------------------------------------------------- */

void EXTI15_10_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Clear possible other EXTI-interrupts
	EXTI_ClearITPendingBit(EXTI_Line11 | EXTI_Line12 | EXTI_Line13 | EXTI_Line14 | EXTI_Line15);

	// Check if EXTI_Line10 is asserted
    if(EXTI_GetITStatus(EXTI_Line10) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line10);

		xSemaphoreGiveFromISR(IndexSem, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    }
}

/* ============================ EOF ====================================== */

