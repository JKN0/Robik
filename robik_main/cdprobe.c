/******************************************************************************

  Robik - Rubik's cube solver from junk
  =====================================

  cdprobe.c

  Color detection module
  Contains CdProbeTask

  22.12.2017

*****************************************************************************/

#include <stdlib.h>
#include <stdbool.h>

#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_adc.h>
#include <misc.h>

#include "tinyprintf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "common.h"
#include "tilter.h"
#include "twister.h"
#include "cdmotor.h"
#include "move.h"
#include "cdprobe.h"

//#define DEBUG

/* =====================================================================
------------------------ Constants & macros ------------------------- */

#define TASK_STACK_SIZE     100         // Stack size in words
#define TASK_PRIORITY       1

#define ADC_INT_PRIORITY	13			// one higher than UART

#define ADC_DELAY			(500/portTICK_RATE_MS)
#define ADC_MAX_WAIT		(50/portTICK_RATE_MS)

#define SAMPLE_COUNT        16

#define MEAS_PREDELAY		(4/portTICK_RATE_MS)

#define MEAS_FAIL    		9999

#define MIN_DIFF    		4

/* =====================================================================
------------------------ Typedefs ----------------------------------- */

typedef struct pwm_t {
    uint16_t r;
    uint16_t g;
    uint16_t b;
} PWM_T;

typedef struct step_t {
	uint16_t limit;
	uint16_t next_gt;
	uint16_t color_gt;
	uint16_t next_lt;
	uint16_t color_lt;
} STEP_T;

/* =====================================================================
------------------------  Global variables  ------------------------- */

static SemaphoreHandle_t AdcSem = NULL;

uint16_t adcvals[SAMPLE_COUNT];

// --- Color detection tables ---

static const PWM_T pwm[] = {
				//  r    g    b
                {  0,   500,   0 },    // 0
                {  900,   0,   0 },    // 1
                {  230,   0, 480 },    // 2
                {    0, 600, 200 },    // 3
                {  200, 700,  50 },    // 4
};

static const STEP_T steps[] = {
					// limit  next_gt  color_gt  next_lt  color_lt
                    {   320,       3,    NONE,        1,    NONE },    // 0
                    {   140,   READY,     ORA,        2,    NONE },    // 1
                    {   170,   READY,     WHT,    READY,     YEL },    // 2
                    {   580,   READY,     RED,        4,    NONE },    // 3
                    {   220,   READY,     BLU,    READY,     GRN },    // 4
};

/* --- old tables
static const PWM_T pwm[] = {
				//  r    g    b
                {  999,   0,   0 },    // 0
                {  500,   0, 400 },    // 1
                {  800,   0,   0 },    // 2
                {  999,   0,   0 },    // 3
//                {  700, 900,   0 },    // 4
                {  500,   0, 900 },    // 4
};

static const STEP_T steps[] = {
					// limit  next_gt  color_gt  next_lt  color_lt
                    {   357,       3,    NONE,        1,    NONE },    // 0
                    {   200,   READY,     YEL,        2,    NONE },    // 1
                    {   150,   READY,     ORA,    READY,     WHT },    // 2
                    {   550,       4,    NONE,    READY,     RED },    // 3
//                    {   283,   READY,     BLU,    READY,     GRN },    // 4
                    {   590,   READY,     GRN,    READY,     BLU },    // 4
};
*/

/* --- very old tables
static const PWM_T pwm[] = {
				//  r    g    b
                {  999,   0,   0 },    // 0
                {  500,   0, 400 },    // 1
                {  500, 130,   0 },    // 2
                {    0, 800, 470 },    // 3
                {  700, 900,   0 },    // 4
};

static const STEP_T steps[] = {
					// limit  next_gt  color_gt  next_lt  color_lt
                    {   220,       3,    NONE,        1,    NONE },    // 0
                    {   200,   READY,     YEL,        2,    NONE },    // 1
                    {   170,   READY,     ORA,    READY,     WHT },    // 2
                    {   485,   READY,     RED,        4,    NONE },    // 3
                    {   220,   READY,     BLU,    READY,     GRN },    // 4
};
*/

extern char *col_names[];

/* =====================================================================
------------------------ Function prototypes ------------------------ */

static uint16_t Measure(void);
static void SetRGB_PWMs(uint16_t r_pwm, uint16_t g_pwm, uint16_t b_pwm);
static void StartAdcSampling(void);
static void StopAdcSampling(void);
static void InitAdcInterrupt(void);

/* =====================================================================
Task main function.
--------------------------------------------------------------------- */

static void CdProbeTask(void *pvParameters)
{
    uint16_t measval, color;
    uint8_t cur_step, next_step;
    int16_t diff;

	// Ensure that xxxxFromISR() are not called before scheduler is started
	// by enabling interrupts not earlier than here
    InitAdcInterrupt();

    // Main loop.
    while(1)
    {
		// Wait for start command (blocks here)
		if (xSemaphoreTake(ColdetStartSem, portMAX_DELAY) == pdPASS)
		{
			if (monitor_flags & MON_COLDET)
				printf("Coldet start:\r\n");

			// loop until color is found, 1...3 loops
			cur_step = 0;
			while (1)
			{
				// Set RGB led to color used in the current step
				SetRGB_PWMs(pwm[cur_step].r, pwm[cur_step].g, pwm[cur_step].b);

			    // Wait for a while for phototransistor voltage to stabilize (RC-circuit)
			    vTaskDelay(MEAS_PREDELAY);

			    // Measure the voltage with ADC
				measval = Measure();

				if (monitor_flags & MON_COLDET)
					printf("  step %d: %d\r\n",cur_step,measval);

				if (measval == MEAS_FAIL || measval > 850)
				{
					// measurement value not valid: exit loop
					color = NONE;
					break;
				}
				else
				{	// Compare to limit and decide next step
					diff = measval - steps[cur_step].limit;

					if (diff > 0)	// measval > limit
					{
						if (diff < MIN_DIFF)
						{
							// too close to limit
							color = NONE;
							break;
						}

						next_step = steps[cur_step].next_gt;
						color = steps[cur_step].color_gt;
					}
					else	// measval < limit
					{
						if (-diff < MIN_DIFF)
						{
							// too close to limit
							color = NONE;
							break;
						}

						next_step = steps[cur_step].next_lt;
						color = steps[cur_step].color_lt;
					}

					// If no next step, we know the color: exit loop
					if (next_step == READY)
						break;

					// next round
					cur_step = next_step;
				}
			}

			if (monitor_flags & MON_COLDET)
				printf("  result: %s\r\n",col_names[color]);

			// Send the detected color
			xQueueSend(CdpResultQueue, &color, portMAX_DELAY);

			// Turn off RGB-led
			SetRGB_PWMs(0,0,0);
		}
    }
}

/* =====================================================================
Task initialization function. Create queues and tasks.
--------------------------------------------------------------------- */

uint32_t CdProbeInit(void)
{

	// create semaphore for color detection start
	ColdetStartSem = xSemaphoreCreateBinary();
	if (ColdetStartSem == NULL)
    {
        return(1);
    }

	// create semaphore for ADC ending
	AdcSem = xSemaphoreCreateBinary();
	if (AdcSem == NULL)
    {
        return(1);
    }

    // Create a queue for receiving detected color values from cdp task
	CdpResultQueue = xQueueCreate(CDP_RES_QUEUE_SIZE, sizeof(uint16_t));

    // Create the color detector task.
    if(xTaskCreate(CdProbeTask, "CDP", TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + TASK_PRIORITY, NULL) != pdTRUE)
    {
        return(1);
    }

    // Success.
    return(0);
}

/* =======================================================================
----------------------------------------------------------------------- */

static void InitAdcInterrupt(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	// configure NVIC: ADC
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ADC_INT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
}

/* =======================================================================
Measure light level. Return value 0...1023 or MEAS_FAIL
----------------------------------------------------------------------- */

static uint16_t Measure( void )
{
    uint32_t sum = 0;
    uint16_t measval;
    int i;

	StartAdcSampling();

	// wait for all samples taken
	if (xSemaphoreTake(AdcSem, ADC_MAX_WAIT) == pdPASS)
	{
		for (i = 0; i < SAMPLE_COUNT; i++)
			sum += adcvals[i];

		measval = sum/(SAMPLE_COUNT*4);		// extra division by 4: 12 bits -> 10 bits
	}
	else
		measval = MEAS_FAIL;

	StopAdcSampling();
	return measval;
}

/* =======================================================================
Light up RGB-led with given PWM-combination
----------------------------------------------------------------------- */

static void SetRGB_PWMs(uint16_t r_pwm, uint16_t g_pwm, uint16_t b_pwm)
{
    TIM_SetCompare2(TIM2,r_pwm);
    TIM_SetCompare3(TIM2,g_pwm);
    TIM_SetCompare4(TIM2,b_pwm);
}

/* =======================================================================
Start ADC sampling by enabling TIM3 CH3 interrupt
----------------------------------------------------------------------- */

static void StartAdcSampling(void)
{
    uint16_t cur_ctr;

    cur_ctr = TIM_GetCounter(TIM3);
    TIM_SetCompare3(TIM3, cur_ctr + 100);   // first interrupt after 50 us

    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
    TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
}

/* =======================================================================
Stop ADC sampling by disabling TIM3 CH3 interrupt 
----------------------------------------------------------------------- */

static void StopAdcSampling(void)
{
    TIM_ITConfig(TIM3, TIM_IT_CC3, DISABLE);
}

/* =======================================================================
Adc interrupt handler. Collects 16 samples to adcvals[]; after that,
sets the semaphore AdcSem.
----------------------------------------------------------------------- */

void ADC1_2_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	static int sample_idx = 0;

	//Check if ADC EOC int is asserted
	if(ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
    {
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);

		adcvals[sample_idx] = ADC_GetConversionValue(ADC1);
		sample_idx++;

		if (sample_idx >= SAMPLE_COUNT)
		{
			sample_idx = 0;

			xSemaphoreGiveFromISR(AdcSem, &xHigherPriorityTaskWoken);
			portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
		}
    }
}

/* =======================================================================
Start ADC. During sampling, this is called from TIM3_IRQHandler at 500 us
intervals. This starts one A/D-conversion; the conversion result is stored 
in adcvals[] by ADC1_2_IRQHandler.
----------------------------------------------------------------------- */

void StartAdc(void)
{
    uint16_t capture = 0;

    capture = TIM_GetCapture3(TIM3);
    TIM_SetCompare3(TIM3, capture + COLDET_SAMPLE_INTERVAL);

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);			// start one sample
}

/* ============================ EOF ====================================== */
