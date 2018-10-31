/******************************************************************************

  Robik - Rubik's cube solver from junk
  =====================================

  twister.c

  Twister controlling module

  20.10.2017

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
#include "twister.h"
#include "pid.h"

//#define DEBUG

/* =====================================================================
------------------------ Constants & macros ------------------------- */

#define TASK_STACK_SIZE     110         // Stack size in words
#define TASK_PRIORITY       1

#define PID_STEP_DELAY      (10/portTICK_RATE_MS)
#define INDEX_MAX_WAIT      (5000/portTICK_RATE_MS)
#define INDEX_INT_PRIORITY  13			// one higher than UART

#define K_PS     200
#define K_IS     150
#define K_DS     0
#define K_PP     250
#define K_IP     0
#define K_DP     80

#define INIT_SPEED			 230

#define INDEX_OFFSET_INIT	 233
#define INDEX_OFFSET_RECALIB 2248

#define ERROR_LIMIT          200

#define SCAN_NONE            0
#define SCAN_EDGE            1
#define SCAN_CORNER          2

#define RECALIB_INTERVAL     40		// quarter turns

/* =====================================================================
------------------------  Global variables  ------------------------- */

// --- Move sequences

static const MOTION_MSG_T rotate1_seq[] = {
		{ TWC_REL_POS, 	2015 },
		{ TWC_WAIT,      500 },
		{ END_OF_SEQ,      0 }
};

static const MOTION_MSG_T rotate2_seq[] = {
		{ TWC_REL_POS, 	4030 },
		{ TWC_WAIT,      800 },
		{ END_OF_SEQ,      0 }
};

static const MOTION_MSG_T rotate3_seq[] = {
		{ TWC_REL_POS, -2015 },
		{ TWC_WAIT,      500 },
		{ END_OF_SEQ,      0 }
};

static const MOTION_MSG_T twist1_seq[] = {
		{ TWC_REL_POS,  2145 },
		{ TWC_WAIT,      600 },
		{ TWC_REL_POS,  -130 },
		{ TWC_WAIT,      150 },
		{ END_OF_SEQ,      0 }
};

static const MOTION_MSG_T twist2_seq[] = {
		{ TWC_REL_POS,  4150 },
		{ TWC_WAIT,     1100 },
		{ TWC_REL_POS,  -120 },
		{ TWC_WAIT,      150 },
		{ END_OF_SEQ,      0 }
};

static const MOTION_MSG_T twist3_seq[] = {
		{ TWC_REL_POS,  6165 },
		{ TWC_WAIT,     1600 },
		{ TWC_REL_POS,  -120 },
		{ TWC_WAIT,      150 },
		{ END_OF_SEQ,      0 }
};

static const MOTION_MSG_T cd_edge_start_seq[] = {
		{ TWC_ABS_POS,   740 },
		{ TWC_WAIT,      900 },
		{ END_OF_SEQ,      0 }
};

static const MOTION_MSG_T cd_corner_start_seq[] = {
		{ TWC_ABS_POS,  7710 },
		{ TWC_WAIT,      510 },
		{ END_OF_SEQ,      0 }
};

static const MOTION_MSG_T cd_next_edge_seq[] = {
		{ TWC_REL_POS,  1980 },
		{ TWC_WAIT,      420 },
		{ END_OF_SEQ,      0 }
};

static const MOTION_MSG_T cd_next_corner_seq[] = {
		{ TWC_REL_POS,  2030 },
		{ TWC_WAIT,      440 },
		{ END_OF_SEQ,      0 }
};

static const MOTION_MSG_T cd_end_seq[] = {
		{ TWC_ABS_POS,     0 },
		{ TWC_WAIT,     2000 },
		{ END_OF_SEQ,      0 }
};

static const MOTION_MSG_T *seq_tbl[] = {
	cli_msg_tbl,					// TWS_TEST, test sequence from CLI
    rotate1_seq,					// TWS_ROT1
    rotate2_seq,					// TWS_ROT2
    rotate3_seq,					// TWS_ROT3
    twist1_seq,						// TWS_TWIST1
    twist2_seq,						// TWS_TWIST2
    twist3_seq,						// TWS_TWIST3
    cd_edge_start_seq,				// TWS_CD_EDGE_START
    cd_corner_start_seq,			// TWS_CD_CORNER_START
    NULL,							// TWS_CD_NEXT       -- handled separately
    cd_end_seq						// TWS_CD_END
};

uint16_t quarter_ctr = 0;
bool tw_need_recalib = false;

// --- Motor handling 

static struct PID_DATA spd_pidData;
static struct PID_DATA pos_pidData;

static int16_t kps = K_PS;
static int16_t kis = K_IS;
static int16_t kds = K_DS;
static int16_t kpp = K_PP;
static int16_t kip = K_IP;
static int16_t kdp = K_DP;

static SemaphoreHandle_t IndexSem = NULL;
static SemaphoreHandle_t EndMotionSem = NULL;

/* =====================================================================
------------------------ Function prototypes ------------------------ */

static void SetMotorSpeed(int16_t cv);
static int SearchHomePos( uint16_t offs );
static void SetIndexInterrupt(FunctionalState state);

/* =====================================================================
Motion task.
--------------------------------------------------------------------- */

static void TwisterTask(void *pvParameters)
{
   	const MOTION_MSG_T *seq = NULL;
   	MOTION_MSG_T msg;
    portTickType WakeTime;
    int16_t sps = 0,cvs,pvs;
    int16_t spp = 0,cvp,pvp;
    bool run_pid = false;
	uint16_t cur_ctr = 0;
	uint16_t prev_ctr = 0;
	uint16_t diff;
	bool prev_ok = false;
    int16_t wait_ctr = 0;
    int8_t idx = 0;
	bool init_ramp_on = false;
    int8_t scan_phase = SCAN_NONE;


    WakeTime = xTaskGetTickCount();
    
    // Main loop.
    while(1)
    {
        // Wait for one PID step delay
        vTaskDelayUntil(&WakeTime,PID_STEP_DELAY);
        
        if (wait_ctr != 0)
        {
        	// Waiting: don't execute sequence or read new commands from queue
           	wait_ctr--;
        }
        else
        {	// Not waiting: if sequence active, execute next step
            if (seq != NULL)
            {
            	// execute next step
				if (seq[idx].type != END_OF_SEQ)
                {
                    switch (seq[idx].type)
                    {
                    case TWC_ABS_POS:
                        spp = seq[idx].value;
                        break;

                    case TWC_REL_POS:
                        spp += seq[idx].value;
                        break;

                    case TWC_WAIT:
                        wait_ctr = seq[idx].value/10;
                        break;
                    }
                    idx++;
                }
                else
                    seq = NULL;		// sequence ended
            }
            else // sequence not active: check if there is a new command in the queue
            {
				// receive a command from queue (does not block here)
				if (xQueueReceive(TwQueue, &msg, 0) == pdPASS)
				{
#ifdef DEBUG
					printf("TwTask: got cmd: %d:%d\r\n",msg.type,msg.value);
#endif
//					printf("q=%d,%c\r\n",quarter_ctr,tw_need_recalib ? 't' : 'f');

					seq = NULL;

					switch (msg.type)
					{
					case TWC_SEQ:
						switch (msg.value)
						{
						case TWS_ROT1:
						case TWS_TWIST1:
							quarter_ctr += 1;
							break;

						case TWS_ROT2:
						case TWS_TWIST2:
							quarter_ctr += 2;
							break;

						case TWS_ROT3:
						case TWS_TWIST3:
							quarter_ctr += 3;
							break;

						case TWS_CD_EDGE_START:
							scan_phase = SCAN_EDGE;
							break;

						case TWS_CD_CORNER_START:
							scan_phase = SCAN_CORNER;
							break;

						case TWS_CD_END:
							scan_phase = SCAN_NONE;
							break;
						}

						if (msg.value == TWS_CD_NEXT)
						{
							if (scan_phase == SCAN_EDGE)
								seq = cd_next_edge_seq;
							else if (scan_phase == SCAN_CORNER)
								seq = cd_next_corner_seq;
						}
						else
							seq = seq_tbl[msg.value];
						idx = 0;
						break;

					case TWC_RECALIB:
						// recalibrate
						if (SearchHomePos(INDEX_OFFSET_RECALIB) == OK)
						{
							spp = 0;
							run_pid = true;
							wait_ctr = 80;			// 800 ms

							quarter_ctr = 0;
						}
						else
							vTaskSuspend(NULL);     // init error: stop
						break;

					case TWC_INIT:
						// initialize
						if (SearchHomePos(INDEX_OFFSET_INIT) == OK)
						{
							spp = INDEX_OFFSET_INIT;
							init_ramp_on = true;
							run_pid = true;
							wait_ctr = 70;			// 700 ms

							quarter_ctr = 0;
						}
						else
							vTaskSuspend(NULL);     // reset error: stop
						break;

					case TWC_ABS_POS:
						spp = msg.value;
						printf("pos=%d, TIM4=%d\r\n",spp,TIM_GetCounter(TIM4));
						break;

					case TWC_REL_POS:
						spp += msg.value;
						printf("pos=%d, TIM4=%d\r\n",spp,TIM_GetCounter(TIM4));
						break;

					case TWC_SYNC:
						xSemaphoreGive(MotionSyncSem);
						break;

					case TWC_KPS:
						kps = msg.value;
						pid_Init(kps, kis, kds, &spd_pidData);
						break;

					case TWC_KIS:
						kis = msg.value;
						pid_Init(kps, kis, kds, &spd_pidData);
						break;

					case TWC_KDS:
						kds = msg.value;
						pid_Init(kps, kis, kds, &spd_pidData);
						break;

					case TWC_KPP:
						kpp = msg.value;
						pid_Init(kpp, kip, kdp, &pos_pidData);
						break;

					case TWC_KIP:
						kip = msg.value;
						pid_Init(kpp, kip, kdp, &pos_pidData);
						break;

					case TWC_KDP:
						kdp = msg.value;
						pid_Init(kpp, kip, kdp, &pos_pidData);
						break;

					case TWC_SHOW_POS:
						printf("spp=%d, TIM4=%d\r\n",spp,TIM_GetCounter(TIM4));
						break;
					}
				}
            }
        }
        
        if (init_ramp_on)
        {
        	spp -= 4;
        	if (spp < 4)
        	{
        		spp = 0;
				init_ramp_on = false;
			}
        }

        if (run_pid)
        {
        	// Run PID controllers for the motor
			cur_ctr = TIM_GetCounter(TIM4);

			// Position PID
			pvp = (int16_t) cur_ctr;	// cast 0...65535 --> -32767...+32767
			cvp = pid_Controller(spp, pvp, &pos_pidData);
			sps = cvp / 5;
			if (sps > 60) sps = 60;
			if (sps < -60) sps = -60;

			if (prev_ok)
			{
				diff = cur_ctr - prev_ctr;

				// Speed PID
				pvs = (int16_t) diff;	// cast 0...65535 --> -32767...+32767
				cvs = pid_Controller(sps, pvs, &spd_pidData);
				SetMotorSpeed(cvs);
			}

			prev_ctr = cur_ctr;
			prev_ok = true;
        }

        /* --- reaclibration removed
        // ask for recalibration asap
        if (quarter_ctr > RECALIB_INTERVAL)
        {
        	tw_need_recalib = true;
        	quarter_ctr = 0;
        }
        */
    }
}

/* =====================================================================
--------------------------------------------------------------------- */

void SetMotorSpeed( int16_t cv )
{
	int16_t pwm_val;

	if (cv < 0)
	{
		cv = -cv;
		GPIO_WriteBit(GPIOE, GPIO_Pin_1, Bit_RESET);	// 1 = CW, 0 = CCW
	}
	else
	{
		GPIO_WriteBit(GPIOE, GPIO_Pin_1, Bit_SET);
	}

    if (cv < 10)
    	pwm_val = 0;
    else
    {
    	pwm_val = cv + 550;
		//pwm_val = (16*cv)/10 + 550;

		if (pwm_val > 980)
		{
			pwm_val = 980;
			pid_Reset_Integrator(&spd_pidData);
		}
	}

	TIM_SetCompare1(TIM2, pwm_val);
}

/* =====================================================================
Search home position
--------------------------------------------------------------------- */

#define MAX_SEARCH1_TIME	(1000/portTICK_RATE_MS)
#define MAX_SEARCH2_TIME	(2500/portTICK_RATE_MS)
#define ADD_STEP_TIME   	(800/portTICK_RATE_MS)

static int SearchHomePos( uint16_t offs )
{
    int rc;
    
    // enable index interrupt and start motor
    SetIndexInterrupt(ENABLE);
    SetMotorSpeed(INIT_SPEED);

    // wait for interrupt (blocks here)
    if (xSemaphoreTake(IndexSem, INDEX_MAX_WAIT) == pdPASS)
    {
        // reset encoder counter and stop
        TIM_SetCounter(TIM4,offs);
        rc = OK;
    }
    else
    {	// error
		error_code |= TW_INDEX_ERROR;	// index not found
        rc = FAIL;
    }

    // stop and disable index interrupt
    SetMotorSpeed(0);
    SetIndexInterrupt(DISABLE);
    
    return rc;
}

/* =====================================================================
Module initialization function. Create queues and tasks.
--------------------------------------------------------------------- */

uint32_t TwisterInit(void)
{
    // Queue for commands
	TwQueue = xQueueCreate(TW_QUEUE_SIZE, sizeof(MOTION_MSG_T));
	if (TwQueue == NULL)
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
    if(xTaskCreate(TwisterTask, "TW", TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + TASK_PRIORITY, NULL) != pdTRUE)
    {
        return(1);
    }

    pid_Init(kps, kis, kds, &spd_pidData);
    pid_Init(kpp, kip, kdp, &pos_pidData);

    // Success.
    return(0);
}

/* =======================================================================
Enable/disable EXTI7 interrupt
----------------------------------------------------------------------- */

void SetIndexInterrupt(FunctionalState state)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

    EXTI_ClearITPendingBit(EXTI_Line5 | EXTI_Line6 | EXTI_Line7 | EXTI_Line8 | EXTI_Line9);

	// configure NVIC
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = INDEX_INT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = state;
	NVIC_Init(&NVIC_InitStructure);

	// Configure EXTI line
	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = state;
	EXTI_Init(&EXTI_InitStructure);
}

/* =======================================================================
Index interrupt handler
----------------------------------------------------------------------- */

void EXTI9_5_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Clear possible other EXTI-interrupts
	EXTI_ClearITPendingBit(EXTI_Line5 | EXTI_Line6 | EXTI_Line8 | EXTI_Line9);

	//Check if EXTI_Line7 is asserted
    if(EXTI_GetITStatus(EXTI_Line7) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line7);

		xSemaphoreGiveFromISR(IndexSem, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    }
}

/* ============================ EOF ====================================== */
