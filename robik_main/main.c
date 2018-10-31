/******************************************************************************

  Robik - Rubik's cube solver from junk
  =====================================

  Main module

  16.9.2017

*****************************************************************************/

#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_adc.h>
#include <misc.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "tinyprintf.h"

#include "common.h"
#include "console.h"
#include "tilter.h"
#include "twister.h"
#include "cdmotor.h"
#include "display.h"
#include "move.h"
#include "cdprobe.h"

extern void InitDevice(void);   // in init.c

/* =====================================================================
------------------------ Constants & macros ------------------------- */

#define SOLV_RX_QUEUE_SIZE      100
#define SOLV_TX_QUEUE_SIZE      20

#define TASK_STACK_SIZE     110         // Stack size in words
#define TASK_PRIORITY       1

#define USART3_INT_PRIORITY 14          // one higher than kernel

#define WHITE_BUTTON_PRESSED    (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) == Bit_RESET)

/* =====================================================================
------------------------  Global variables  ------------------------- */

xQueueHandle SolverRxQueue;
xQueueHandle SolverTxQueue;

uint32_t error_code = 0;
uint32_t monitor_flags = 0;

static char dsp1_buf[17];
static char dsp2_buf[17];
static uint8_t move_ctr = 0;

static char cd_result_buf[13];
#define RES_BUF_INIT  "   -   -    "

                             // tile: 0 1 2 3 4  5 6 7 8
static const uint8_t res_idx_tbl[] = {5,6,1,4,9,10,2,0,8 };
static const char *col_chars = "NROYGBW";
static const char *face_chars_scan = "FLBDRU";      // scan sequence order, not the normal FLBRUD !

extern const char *test_mode_names[];

/* =====================================================================
------------------------ Function prototypes ------------------------ */

void InitTimerInterrupt(void);
void InitUSART3Interrupt(void);
uint32_t MainInit(void);
static bool CubeDetected(void);
static void WaitContinueButton(void);
static void ResetSolver(void);
void DetectColors(void);
uint8_t ExecSolution(void);
void ExerciseMode( void );
void Send2Solver(char *s);
char GetSolverCh(void);
uint8_t CheckSolverComms(void);

/* =====================================================================
Main program
Initialize hw, all modules and start FreRTOS scheduler.
--------------------------------------------------------------------- */

int main(void)
{
    // initialize CPU I/O
    InitDevice();

    // use polling putc before RTOS started
    init_printf(0,putc_cons_poll);

    printf("\r\n*** Robik 1.1 ***\r\n>");
    
    // Initialize modules

    if (MainInit() != 0)
    {
        printf("MainTask init failed\r\n");
        while(1);
    }

    if (ConsoleInit() != 0)
    {
        printf("Console init failed\r\n");
        while(1);
    }

    if (TilterInit() != 0)
    {
        printf("Tilter init failed\r\n");
        while(1);
    }

    if (TwisterInit() != 0)
    {
        printf("Twister init failed\r\n");
        while(1);
    }

    if (CdMotorInit() != 0)
    {
        printf("CdMotor init failed\r\n");
        while(1);
    }

    if (DisplayInit() != 0)
    {
        printf("Display init failed\r\n");
        while(1);
    }

    if (MoveInit() != 0)
    {
        printf("Move init failed\r\n");
        while(1);
    }

    if (CdProbeInit() != 0)
    {
        printf("CdProbe init failed\r\n");
        while(1);
    }

    // Start the tasks
    vTaskStartScheduler();

    while (1);
}

/******************************************************************************
***                       Main task                                         ***
******************************************************************************/

/* =====================================================================
MainTask function.
--------------------------------------------------------------------- */

static void MainTask(void *pvParameters)
{
    uint8_t rc;
    uint8_t prev_tm = TEST_NONE;
    bool exercise = false;

    // Ensure that xxxxFromISR() are not called before scheduler is started
    // by enabling interrupts not earlier than here
    InitTimerInterrupt();
    InitUSART3Interrupt();

    if (WHITE_BUTTON_PRESSED)
        exercise = true;

    Send2Dsp(DSPC_WRITE_ROW1,0," *** ROBIK ***  ");
    Send2Dsp(DSPC_WRITE_ROW2,0,"Initializing... ");

     Delay(600);

     // Initialize motion processes
     Send2Mov(MVC_INIT_MOTION,0,0);

    // Reset solver board (helps debugging)
    ResetSolver();
    Delay(500);

    // Check communications with solver board
    if (CheckSolverComms() == FAIL)
        error_code |= SOLVER_INIT_ERROR;

    // Wait for motion initialization
    if (SyncWithMov() == FAIL)
        error_code |= MAIN_INIT_ERROR;

    if (error_code != 0)
    {
        Send2Dsp(DSPC_WRITE_ROW1,0,"Init error      ");
        sprintf(dsp2_buf,"code %04X      ",error_code);
        Send2Dsp(DSPC_WRITE_ROW2,0,dsp2_buf);

        vTaskSuspend(NULL);
    }

    if (exercise)
        ExerciseMode();         // does not return

    // --- Main loop
    while(1)
    {
    reloop:
        if (test_mode == TEST_NONE)
        {
            Send2Dsp(DSPC_WRITE_ROW1,0," *** ROBIK ***  ");
            Send2Dsp(DSPC_WRITE_ROW2,0," - Set cube -   ");
        }

        // --- Wait here until cube is placed on tilt tray
        while (!CubeDetected())
        {
            if (test_mode == TEST_NONE && prev_tm != TEST_NONE)
            {
                prev_tm = test_mode;
                goto reloop;
            }
            prev_tm = test_mode;
            Delay(20);
        }

        if (test_mode == TEST_CUB)
            printf("cube detected\r\n");

        // Actual color detection/solving is done only when not in any test mode
        if (test_mode == TEST_NONE)
        {
            Send2Dsp(DSPC_WRITE_ROW1,0,"Start color     ");
            Send2Dsp(DSPC_WRITE_ROW2,0,"detection...    ");

            Delay(1000);

            // Move cube from rest to rotation position and sync
            Send2Mov(MVC_FROM_REST,0,0);
            if (SyncWithMov() == FAIL)
                error_code |= MAIN_SYNC_ERROR;

            if (error_code != 0)
            {
                Send2Dsp(DSPC_WRITE_ROW1,0,"Move error      ");
                sprintf(dsp2_buf,"code %04X       ",error_code);
                Send2Dsp(DSPC_WRITE_ROW2,0,dsp2_buf);

                vTaskSuspend(NULL);
            }

            // --- Phase 1: scan colors from cube
            DetectColors();

            if (error_code != 0)
            {
                Send2Dsp(DSPC_WRITE_ROW1,0,"Color detect err");
                sprintf(dsp2_buf,"code %04X       ",error_code);
                Send2Dsp(DSPC_WRITE_ROW2,0,dsp2_buf);

                vTaskSuspend(NULL);
            }

            // --- Phase 2: execute moves that solve the cube
            move_ctr = 0;
            rc = ExecSolution();

            if (rc == 0 && error_code == 0)
            {
                Send2Dsp(DSPC_WRITE_ROW1,0,"Solved!         ");
                sprintf(dsp2_buf,"%2d moves        ",move_ctr);
                Send2Dsp(DSPC_WRITE_ROW2,0,dsp2_buf);
            }
            else
            {
                if (rc == 0)
                    sprintf(dsp1_buf,"Solving error   ");
                else
                    sprintf(dsp1_buf,"Solver error #%d ",rc);

                Send2Dsp(DSPC_WRITE_ROW1,0,dsp1_buf);
                sprintf(dsp2_buf,"code %04X       ",error_code);
                Send2Dsp(DSPC_WRITE_ROW2,0,dsp2_buf);

                // wait for button after solver error
                WaitContinueButton();
                error_code = 0;     // reset error code

                Send2Dsp(DSPC_WRITE_ROW1,0,"Remove cube     ");
                Send2Dsp(DSPC_CLR_ROW2,0,NULL);
            }

            // Move cube from rotation position to tilt tray and sync
            Send2Mov(MVC_TO_REST,0,0);
            if (SyncWithMov() == FAIL)
                error_code |= MAIN_SYNC_ERROR;

            if (error_code != 0)
            {
                Send2Dsp(DSPC_WRITE_ROW1,0,"Move error      ");
                sprintf(dsp2_buf,"code %04X       ",error_code);
                Send2Dsp(DSPC_WRITE_ROW2,0,dsp2_buf);

                vTaskSuspend(NULL);
            }

            // Reset solver board for next round
            ResetSolver();
            Delay(500);

            // Check communications after reset
            if (CheckSolverComms() == FAIL)
                error_code |= SOLVER_INIT_ERROR;

            if (error_code != 0)
            {
                Send2Dsp(DSPC_WRITE_ROW1,0,"Solver reset err");
                sprintf(dsp2_buf,"code %04X       ",error_code);
                Send2Dsp(DSPC_WRITE_ROW2,0,dsp2_buf);

                vTaskSuspend(NULL);
            }

        } // if (test_mode == TEST_NONE)

        // --- Wait here until cube is removed from tilt tray
        while (CubeDetected())
            Delay(20);

        if (test_mode == TEST_CUB)
            printf("cube not detected\r\n");
    }
}

/* =====================================================================
Cube detection. Return true, if cube detected.
--------------------------------------------------------------------- */

static bool CubeDetected(void)
{
    int i;
    bool detect = true;

    // Detector led on
    GPIO_WriteBit(GPIOC, GPIO_Pin_12, Bit_SET);

    for (i = 0; i < 5; i++)
    {
        // Wait for 10 ms
        Delay(10);

        // Read detector state
        detect &= (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == Bit_SET);
    }

    // Detector led off
    GPIO_WriteBit(GPIOC, GPIO_Pin_12, Bit_RESET);
    Delay(2);

    return detect;
}

/* =====================================================================
Wait for continue button (white). Display led on during waiting.
--------------------------------------------------------------------- */

static void WaitContinueButton(void)
{
    dsp_led = true;
    Send2Dsp(DSPC_UPDATE_LED,0,NULL);

    while (!WHITE_BUTTON_PRESSED)
        Delay(10);

    dsp_led = false;
    Send2Dsp(DSPC_UPDATE_LED,0,NULL);
}

/* =====================================================================
Reset solver board
--------------------------------------------------------------------- */

static void ResetSolver(void)
{
    // Drop PD13 for 10 ms without affecting other port bits
    // (stepper control lines)
    GPIOD->BSRR = 0x20000000;
    Delay(10);
    GPIOD->BSRR = 0x00002000;
}
/* =====================================================================
Color detection coordinator.
Command cube movements and color detection, send results to solver.
--------------------------------------------------------------------- */

#define RESP_MAX_WAIT       (1000/portTICK_RATE_MS)
#define NEXT_TILE_MAX_WAIT  (4000/portTICK_RATE_MS)
#define NEXT_FACE_MAX_WAIT  (15000/portTICK_RATE_MS)

void DetectColors(void)
{
    uint8_t face,tile;
    bool start_scan = true;
    int16_t color;

    // to move task: we are starting color scanning
    Send2Mov(MVC_C_START,0,0);

    Send2Dsp(DSPC_CLR_ROW2,0,NULL);

    // loop for all 6 faces
    for (face = 0; face < FACES; face++)
    {
        // to move task: turn next face up
        // at first time this is not needed
        if (!start_scan)
            Send2Mov(MVC_C_NEXT_FACE,0,0);

        start_scan = false;

        // wait until cube movement has been done
        if (SyncWithMov() == FAIL)
            error_code |= COLDET_SYNC_ERROR;

        if (error_code != 0)
            return;

        // reset result buf
        strcpy(cd_result_buf,RES_BUF_INIT);

        // display the face we are scanning now
        sprintf(dsp1_buf,"Scan face %c     ",face_chars_scan[face]);
        Send2Dsp(DSPC_WRITE_ROW1,0,dsp1_buf);
        Send2Dsp(DSPC_WRITE_ADDR,0xC1,cd_result_buf);

        // loop for all 9 tiles of the face
        for (tile = 0; tile < 9; tile++)
        {
            // to move task: position cd probe to next tile, sync
            Send2Mov(MVC_C_NEXT_TILE,0,0);
            if (SyncWithMov() == FAIL)
                error_code |= COLDET_SYNC_ERROR;

            if (error_code != 0)
                return;

            // to cd probe task: resolve current color, wait for result
            xSemaphoreGive(ColdetStartSem);
            if (xQueueReceive(CdpResultQueue, &color, RESP_MAX_WAIT) == pdFAIL)
            {
                error_code |= COLDET_DETECT_ERROR;
                return;
            }

            // store the result to buffer
            cd_result_buf[res_idx_tbl[tile]] = col_chars[color];

            // display the scan progress
            Send2Dsp(DSPC_WRITE_ADDR,0xC1,cd_result_buf);
        }

        // to move task: this is the end of current face
        Send2Mov(MVC_C_END_FACE,0,0);

        // send this face's detection result to solver
        Send2Solver(cd_result_buf);

        if (monitor_flags & MON_SOLVER_COMM)
            printf("Sent to solver: %s\r\n",cd_result_buf);
    }

    // send end-of-scan to solver
    Send2Solver("\r");
}

/* =====================================================================
Solution moves coordinator.
Read moves from solver and command execution of moves.
--------------------------------------------------------------------- */

uint8_t ExecSolution(void)
{
    char ch,face_ch,turns_ch;
    uint8_t face,turns;
    bool got_error_message = false;

    // to move task: we are starting solution execution
    Send2Mov(MVC_S_START,0,0);

    Send2Dsp(DSPC_WRITE_ROW1,0,"Solving...      ");
    Send2Dsp(DSPC_CLR_ROW2,0,NULL);

    while (1)
    {
        // read char from solver serial port (should be a face/error/end)
        ch = GetSolverCh();
        face_ch = ch;

        switch (ch)
        {
        // face names, turn amount will follow
        case 'F': face = FRONT;  break;
        case 'L': face = LEFT;   break;
        case 'B': face = BACK;   break;
        case 'R': face = RIGHT;  break;
        case 'U': face = UP;     break;
        case 'D': face = DOWN;   break;

        case 'E':   // error message from solver, error number will follow
            got_error_message = true;
            break;

        case '\r':  // end marker, all moves done
            if (monitor_flags & MON_SOLVER_COMM)
                printf("From solver: end\r\n");
            return 0;

        default:    // something unknown from solver
            error_code |= SOLVER_ERROR;
            return 0;
        }

        // read char from solver serial port (should be turn amount or error number)
        ch = GetSolverCh();
        turns_ch = ch;

        if (monitor_flags & MON_SOLVER_COMM)
            printf("From solver: %c%c\r\n",face_ch,turns_ch);

        if (!isdigit((int)ch))
        {
            // not digit, not acceptable
            error_code |= SOLVER_ERROR;
            return 0;
        }

        // if error message, return error number
        if (got_error_message)
            return ch & 0x0F;

        turns = ch & 0x0F;
        if (turns < 1 || turns > 3)
        {
            // invalid turn amount
            error_code |= SOLVER_ERROR;
            return 0;
        }

        move_ctr++;

        // display current move
        sprintf(dsp2_buf,"move %d: %c%c",move_ctr,face_ch,turns_ch);
        Send2Dsp(DSPC_WRITE_ROW2,0,dsp2_buf);

        // to move task: execute move
        Send2Mov(MVC_S_MOVE,face,turns);

        // wait for execution
        if (SyncWithMov() == FAIL)
            error_code |= EXEC_SYNC_ERROR;

        if (error_code != 0)
            return 0;
    }
}

/* =====================================================================
Exercise mode. Just moves tilter back and forth endlessly. Exit by
power off.
--------------------------------------------------------------------- */

void ExerciseMode( void )
{
    MOTION_MSG_T msg;

    Send2Dsp(DSPC_WRITE_ROW1,0,"Exercise mode   ");
    Send2Dsp(DSPC_CLR_ROW2,0,NULL);

    while (1)
    {
        // start exercise sequence
        msg.type = TLC_SEQ;
        msg.value = TLS_EXERCISE;
        xQueueSend(TlQueue, &msg, portMAX_DELAY);

        // wait for sequence execution
        msg.type = TLC_SYNC;
        msg.value = 0;
        xQueueSend(TlQueue, &msg, portMAX_DELAY);
        xSemaphoreTake(MotionSyncSem, SYNC_MAX_WAIT);
    }
}

/* =====================================================================
Delay about n ms. Don't use values under 2.
--------------------------------------------------------------------- */

void Delay( int ms )
{
    vTaskDelay(ms/portTICK_RATE_MS);
}

/* =====================================================================
Task initialization function. Create queues and tasks.
--------------------------------------------------------------------- */

uint32_t MainInit(void)
{
    // Create a queue for received characters from solver
    SolverRxQueue = xQueueCreate(SOLV_RX_QUEUE_SIZE, sizeof(char));

    // Create a queue for sending characters to solver
    SolverTxQueue = xQueueCreate(SOLV_TX_QUEUE_SIZE, sizeof(char));

    // Create the main task
    if(xTaskCreate(MainTask, "MAIN", TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + TASK_PRIORITY, NULL) != pdTRUE)
    {
        return(1);
    }

    // Success
    return(0);
}

/* =======================================================================
----------------------------------------------------------------------- */

void InitTimerInterrupt(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the TIM3 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = STEP_INT_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* =======================================================================
TIM3 interrupt handler. Occurs for each step, interval varies.

This is the common IRQ handler for all three channels:
    CH1: Tilter motor stepping
    CH2: Coldet motor stepping
    CH3: ADC sampling
----------------------------------------------------------------------- */

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
        StepTilterMotor();
    }

    if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
        StepColdetMotor();
    }

    if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
        StartAdc();
    }
}

/* =======================================================================
Send string to solver board (USART3)
----------------------------------------------------------------------- */

void Send2Solver(char *s)
{
    while (*s != '\0')
    {
        xQueueSend(SolverTxQueue, s, portMAX_DELAY);
        s++;
    }

    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}

/* =======================================================================
Get char from solver board (USART3)
----------------------------------------------------------------------- */

char GetSolverCh(void)
{
    char ch;

    if (xQueueReceive(SolverRxQueue, &ch, portMAX_DELAY) == pdFAIL)
        return 0;

    return ch & 0x7F;
}

/* =======================================================================
Check communication with solver board
----------------------------------------------------------------------- */

#define SOLVER_MAX_WAIT     (2000/portTICK_RATE_MS)

uint8_t CheckSolverComms(void)
{
    char ch;
    uint8_t state = 0;

    Send2Solver("H\r");

    while (1)
    {
        if (xQueueReceive(SolverRxQueue, &ch, SOLVER_MAX_WAIT) == pdPASS)
        {
            switch (state)
            {
            case 0:  if (ch == 'Y')  state = 1; break;
            case 1:  if (ch == '\r') state = 2; break;
            default:                 state = 0; break;
            }
        }
        else
            return (state == 2) ? OK : FAIL;
    }
}

/* =======================================================================
Init USART3 interrupt
----------------------------------------------------------------------- */

void InitUSART3Interrupt(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    // Enable USART3 IRQ
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART3_INT_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

/* =====================================================================
USART3 interrupt.
--------------------------------------------------------------------- */

void USART3_IRQHandler(void)
{
    char ch;

    // Add received chars to queue
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
         // Receive the character and put to queue
        ch = USART_ReceiveData(USART3);
        if (xQueueSendFromISR(SolverRxQueue, &ch, NULL) != pdPASS)
        {
            // Error. The queue should never be full.
            while(1);
        }
    }

    if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
    {
         // Anything to send?
        if (xQueueReceiveFromISR(SolverTxQueue, &ch, NULL) == pdPASS)
            USART_SendData(USART3, ch); // Transmit the character
        else
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE); // Suppress interrupt when queue empty
    }
}

/* =======================================================================
FreeRTOS hook stubs
----------------------------------------------------------------------- */

void vApplicationMallocFailedHook( void )
{
    taskDISABLE_INTERRUPTS();

    // Light yellow led
    GPIO_WriteBit(GPIOC, GPIO_Pin_10, Bit_SET);

    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{

}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    taskDISABLE_INTERRUPTS();
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
    static uint16_t ctr = 250;
    static bool state = false;

    // blink the green led (PA8) in 1 Hz rate
    ctr--;
    if (ctr == 0)
    {
        ctr = 250;
        state = !state;
        if (state)
            GPIOA->BSRR = 0x01000000;
        else
            GPIOA->BSRR = 0x00000100;
    }
}

/* ============================ EOF ====================================== */
