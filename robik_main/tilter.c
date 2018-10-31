/******************************************************************************

  Robik - Rubik's cube solver from junk
  =====================================

  tilter.c

  Tilter controlling module
  Contains TilterTask

  27.10.2017

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

#define TASK_STACK_SIZE     110         // Stack size in words
#define TASK_PRIORITY       1

#define TL_INDEX_INT_PRIORITY 12        // one higher than step int

#define MOVE_MAX_WAIT       (5000/portTICK_RATE_MS)

#define ST_RIGHT            -1
#define ST_STOP              0
#define ST_LEFT              1

#define STOPPED              0
#define ACCELERATING         1
#define DECELERATING         2
#define SLEWING              3

#define ROT_POS              0
#define TW_POS               1
#define REST_POS             2
#define FAR_RIGHT_POS        3

#define INIT_SPEED           5500

/* =====================================================================
------------------------  Global variables  ------------------------- */

// --- Tilt/move sequences

// rotation position = 170
// twist position = 810
// rest position = 630
// far left (during tilting) = 0
// far right (during tilting) = 1830

static const MOTION_MSG_T tilt_from_rot_to_rot_seq[] = { // rotation position --> rotation position
        { TLC_ACCEL,  145 },    // from 170 -> 1660 steps = 1830
        { TLC_SLEW,   470 },
        { TLC_DECEL,   45 },
        { TLC_SLEW,   100 },
        { TLC_ACCEL,   30 },
        { TLC_SLEW,   640 },
        { TLC_DECEL,   30 },
        { TLC_SLEW,   100 },
        { TLC_DECEL,  100 },
        { TLC_ACCEL, -150 },    // from 1830 <- 1830 steps = 0
        { TLC_SLEW,  -990 },
        { TLC_DECEL,  -70 },
        { TLC_SLEW,  -120 },
        { TLC_ACCEL,  -25 },
        { TLC_SLEW,  -370 },
        { TLC_DECEL, -105 },
        { TLC_ACCEL,   85 },    // from 0 -> 170 steps = 170
        { TLC_DECEL,   85 },
        { END_OF_SEQ,   0 },
};

static const MOTION_MSG_T tilt_from_tw_to_tw_seq[] = {  // twist position --> twist position
        { TLC_ACCEL,  100 },    // from 810 -> 1020 steps = 1830
        { TLC_SLEW,    60 },
        { TLC_ACCEL,   30 },
        { TLC_SLEW,   700 },
        { TLC_DECEL,  130 },
        { TLC_ACCEL, -150 },    // from 1830 <- 1830 steps = 0
        { TLC_SLEW,  -990 },
        { TLC_DECEL,  -70 },
        { TLC_SLEW,  -120 },
        { TLC_ACCEL,  -25 },
        { TLC_SLEW,  -370 },
        { TLC_DECEL, -105 },
        { TLC_ACCEL,  145 },    // from 0 -> 810 steps = 810
        { TLC_SLEW,   520 },
        { TLC_DECEL,  145 },
        { END_OF_SEQ,   0 },
};

static const MOTION_MSG_T tilt_from_rot_to_tw_seq[] = {     // rotation position --> twist position
        { TLC_ACCEL,  145 },    // from 170 -> 1660 steps = 1830
        { TLC_SLEW,   470 },
        { TLC_DECEL,   45 },
        { TLC_SLEW,   100 },
        { TLC_ACCEL,   30 },
        { TLC_SLEW,   640 },
        { TLC_DECEL,   30 },
        { TLC_SLEW,   100 },
        { TLC_DECEL,  100 },
        { TLC_ACCEL, -150 },    // from 1830 <- 1830 steps = 0
        { TLC_SLEW,  -990 },
        { TLC_DECEL,  -70 },
        { TLC_SLEW,  -120 },
        { TLC_ACCEL,  -25 },
        { TLC_SLEW,  -370 },
        { TLC_DECEL, -105 },
        { TLC_ACCEL,  145 },    // from 0 -> 810 steps = 810
        { TLC_SLEW,   520 },
        { TLC_DECEL,  145 },
        { END_OF_SEQ,   0 },
};

static const MOTION_MSG_T tilt_from_tw_to_rot_seq[] = {     // twist position --> rotation position
        { TLC_ACCEL,  100 },    // from 810 -> 1020 steps = 1830
        { TLC_SLEW,    60 },
        { TLC_ACCEL,   30 },
        { TLC_SLEW,   700 },
        { TLC_DECEL,  130 },
        { TLC_ACCEL, -150 },    // from 1830 <- 1830 steps = 0
        { TLC_SLEW,  -990 },
        { TLC_DECEL,  -70 },
        { TLC_SLEW,  -120 },
        { TLC_ACCEL,  -25 },
        { TLC_SLEW,  -370 },
        { TLC_DECEL, -105 },
        { TLC_ACCEL,   85 },    // from 0 -> 170 steps = 170
        { TLC_DECEL,   85 },
        { END_OF_SEQ,   0 },
};

static const MOTION_MSG_T move_from_rest_to_rot_seq[] = {   // rest position --> rotation position, no tilt
        { TLC_ACCEL, -120 },    // from 630 <- 630 steps = 0
        { TLC_SLEW,  -390 },
        { TLC_DECEL, -120 },
        { TLC_ACCEL,   85 },    // from 0 -> 170 steps = 170
        { TLC_DECEL,   85 },
        { END_OF_SEQ,   0 },
};

static const MOTION_MSG_T move_from_tw_to_rest_seq[] = {    // twist position --> rest position (with tilt)
        { TLC_ACCEL,  100 },    // from 810 -> 1020 steps = 1830
        { TLC_SLEW,   160 },
        { TLC_ACCEL,   30 },
        { TLC_SLEW,   600 },
        { TLC_DECEL,  130 },
        { TLC_ACCEL, -150 },    // from 1830 <- 1200 steps = 630
        { TLC_SLEW,  -900 },
        { TLC_DECEL, -150 },
        { END_OF_SEQ, 0   },
};

static const MOTION_MSG_T move_from_index_to_rest_seq[] = {     // index position --> rest position, no tilt
        { TLC_ACCEL,  140 },    // from 30 -> 600 steps = 630
        { TLC_SLEW,   320 },
        { TLC_DECEL,  140 },
        { END_OF_SEQ, 0   },
};

static const MOTION_MSG_T move_from_rot_to_tw_seq[] = {     // rotation position --> twist position, no tilt
        { TLC_ACCEL,  150 },    // from 170 -> 640 steps = 810
        { TLC_SLEW,   340 },
        { TLC_DECEL,  150 },
        { END_OF_SEQ, 0   },
};

static const MOTION_MSG_T move_from_tw_to_rot_seq[] = {     // twist position --> rotation position, no tilt
        { TLC_ACCEL, -150 },    // from 810 <- 640 steps = 170
        { TLC_SLEW,  -340 },
        { TLC_DECEL, -150 },
        { END_OF_SEQ, 0   },
};

static const MOTION_MSG_T move_from_rot_to_rest_seq[] = {   // rotation position --> rest position (with tilt)
        { TLC_ACCEL,  150 },    // from 170 -> 1660 steps = 1830
        { TLC_SLEW,   460 },
        { TLC_DECEL,   50 },
        { TLC_SLEW,   100 },
        { TLC_ACCEL,   30 },
        { TLC_SLEW,   640 },
        { TLC_DECEL,   30 },
        { TLC_SLEW,   100 },
        { TLC_DECEL,  100 },
        { TLC_ACCEL, -150 },    // from 1830 <- 1200 steps = 630
        { TLC_SLEW,  -900 },
        { TLC_DECEL, -150 },
        { END_OF_SEQ, 0   },
};

// following three sequences do the same as tilt_from_tw_to_tw_seq or tilt_from_rot_to_tw_seq
// but in two pieces
// used during twister recalibration: after from_xx_to_fr_seq we know twister is free
static const MOTION_MSG_T tilt_from_tw_to_fr_seq[] = {  // twist position --> far right position
        { TLC_ACCEL,  100 },    // from 810 -> 1020 steps = 1830
        { TLC_SLEW,    60 },
        { TLC_ACCEL,   30 },
        { TLC_SLEW,   700 },
        { TLC_DECEL,  130 },
        { END_OF_SEQ,   0 },
};

static const MOTION_MSG_T tilt_from_rot_to_fr_seq[] = { // rotation position --> far right position
        { TLC_ACCEL,  150 },    // from 170 -> 1660 steps = 1830
        { TLC_SLEW,   460 },
        { TLC_DECEL,   50 },
        { TLC_SLEW,   100 },
        { TLC_ACCEL,   30 },
        { TLC_SLEW,   640 },
        { TLC_DECEL,   30 },
        { TLC_SLEW,   100 },
        { TLC_DECEL,  100 },
        { END_OF_SEQ,   0 },
};

static const MOTION_MSG_T tilt_from_fr_to_tw_seq[] = {  // far right position --> twist position
        { TLC_ACCEL, -150 },    // from 1830 <- 1830 steps = 0
        { TLC_SLEW,  -990 },
        { TLC_DECEL,  -70 },
        { TLC_SLEW,  -120 },
        { TLC_ACCEL,  -40 },
        { TLC_SLEW,  -340 },
        { TLC_DECEL, -120 },
        { TLC_ACCEL,  145 },    // from 0 -> 810 steps = 810
        { TLC_SLEW,   520 },
        { TLC_DECEL,  145 },
        { END_OF_SEQ,   0 },
};

static const MOTION_MSG_T exercise_seq[] = {    // rest position --> rest position
        { TLC_ACCEL, -150 },    // from 630 <- 630 steps = 0
        { TLC_SLEW,  -330 },
        { TLC_DECEL, -150 },
        { TLC_ACCEL,  150 },    // from 0 -> 1830 steps = 1830
        { TLC_SLEW,  1530 },
        { TLC_DECEL,  150 },
        { TLC_ACCEL, -150 },    // from 1830 <- 1200 steps = 630
        { TLC_SLEW,  -900 },
        { TLC_DECEL, -150 },
        { END_OF_SEQ,   0 },
};

uint16_t cur_pos = REST_POS;

// --- Stepper motor handling

static SemaphoreHandle_t EndMotionSem = NULL;
static SemaphoreHandle_t IndexSem = NULL;

static int16_t step_dir = ST_STOP;
static int16_t steps = 0;
static uint16_t acc_idx = 0;
static uint16_t mode = STOPPED;
static uint8_t par_a = 182;
static int16_t pos_ctr = 0;
static bool const_speed = false;

// step pattern table, half step
static const uint16_t step_tbl[] = {
           //  PD3 PD2 PD1 PD0
           //  IB  IA  PB  PA  => IB  IA  B   A
    0x3,   //  0   0   1   1      max max +   +
    0x7,   //  0   1   1   1      max off +   off
    0x2,   //  0   0   1   0      max max +   -
    0xA,   //  1   0   1   0      off max off -
    0x0,   //  0   0   0   0      max max -   -
    0x4,   //  0   1   0   0      max off -   off
    0x1,   //  0   0   0   1      max max -   +
    0x9    //  1   0   0   1      off max off +
};

#define STEP_TBL_MASK           0x07

#define STEPPER_STOP            0x0F        //  off off +   +
#define STEPPER_CONTROL_MASK    0x000F000F

// Constant velocity acceleration/deceleration table for both steppers
// 65535*(sqrt(n+1)-sqrt(n))
const uint16_t AccTable[256] =
{
    0xFFFF,0x6A09,0x515D,0x4498,0x3C6E,0x36A2,0x323E,0x2EC3,0x2BEC,0x298A,0x2783,0x25C0,0x2435,0x22D7,0x219E,0x2084,
    0x1F83,0x1E99,0x1DC2,0x1CFC,0x1C45,0x1B9B,0x1AFC,0x1A67,0x19DC,0x1959,0x18DD,0x1868,0x17FA,0x1791,0x172D,0x16CE,
    0x1673,0x161D,0x15CA,0x157B,0x152F,0x14E7,0x14A1,0x145D,0x141D,0x13DE,0x13A2,0x1368,0x1330,0x12F9,0x12C5,0x1292,
    0x1261,0x1231,0x1203,0x11D6,0x11AA,0x117F,0x1156,0x112E,0x1107,0x10E1,0x10BC,0x1098,0x1074,0x1052,0x1030,0x1010,
    0x0FF0,0x0FD0,0x0FB2,0x0F94,0x0F77,0x0F5A,0x0F3E,0x0F23,0x0F08,0x0EEE,0x0ED4,0x0EBB,0x0EA2,0x0E8A,0x0E72,0x0E5B,
    0x0E44,0x0E2D,0x0E17,0x0E01,0x0DEC,0x0DD7,0x0DC3,0x0DAF,0x0D9B,0x0D87,0x0D74,0x0D61,0x0D4F,0x0D3C,0x0D2A,0x0D19,
    0x0D07,0x0CF6,0x0CE5,0x0CD4,0x0CC4,0x0CB4,0x0CA4,0x0C94,0x0C85,0x0C76,0x0C67,0x0C58,0x0C49,0x0C3B,0x0C2D,0x0C1F,
    0x0C11,0x0C03,0x0BF6,0x0BE8,0x0BDB,0x0BCE,0x0BC2,0x0BB5,0x0BA9,0x0B9C,0x0B90,0x0B84,0x0B78,0x0B6C,0x0B61,0x0B55,
    0x0B4A,0x0B3F,0x0B34,0x0B29,0x0B1E,0x0B13,0x0B09,0x0AFE,0x0AF4,0x0AEA,0x0AE0,0x0AD6,0x0ACC,0x0AC2,0x0AB8,0x0AAF,
    0x0AA5,0x0A9C,0x0A93,0x0A8A,0x0A80,0x0A77,0x0A6F,0x0A66,0x0A5D,0x0A54,0x0A4C,0x0A43,0x0A3B,0x0A32,0x0A2A,0x0A22,
    0x0A1A,0x0A12,0x0A0A,0x0A02,0x09FA,0x09F3,0x09EB,0x09E3,0x09DC,0x09D4,0x09CD,0x09C6,0x09BE,0x09B7,0x09B0,0x09A9,
    0x09A2,0x099B,0x0994,0x098D,0x0986,0x0980,0x0979,0x0972,0x096C,0x0965,0x095F,0x0959,0x0952,0x094C,0x0946,0x093F,
    0x0939,0x0933,0x092D,0x0927,0x0921,0x091B,0x0915,0x090F,0x090A,0x0904,0x08FE,0x08F9,0x08F3,0x08ED,0x08E8,0x08E2,
    0x08DD,0x08D7,0x08D2,0x08CD,0x08C7,0x08C2,0x08BD,0x08B8,0x08B2,0x08AD,0x08A8,0x08A3,0x089E,0x0899,0x0894,0x088F,
    0x088A,0x0886,0x0881,0x087C,0x0877,0x0872,0x086E,0x0869,0x0864,0x0860,0x085B,0x0857,0x0852,0x084E,0x0849,0x0845,
    0x0840,0x083C,0x0838,0x0833,0x082F,0x082B,0x0827,0x0822,0x081E,0x081A,0x0816,0x0812,0x080E,0x080A,0x0805,0x0801
};

/* =====================================================================
------------------------ Function prototypes ------------------------ */

static const MOTION_MSG_T *SelectSeq( int16_t scmd );
static int SearchHomePos( void );
static void SetIndexInterrupt(FunctionalState state);
static void StartStepping(void);
static void StopStepping(void);
void StepTilterMotor(void);

/* =====================================================================
Motion task.
--------------------------------------------------------------------- */

static void TilterTask(void *pvParameters)
{
    const MOTION_MSG_T *seq = NULL;
    MOTION_MSG_T msg;
    int i;

    StepperOut(STEPPER_STOP,STEPPER_CONTROL_MASK);

    // Main loop.
    while(1)
    {
        // Wait for a command (blocks here)
        if (xQueueReceive(TlQueue, &msg, portMAX_DELAY) == pdPASS)
        {
#ifdef DEBUG
            printf("TlTask: got cmd: %d:%d\r\n",msg.type,msg.value);
#endif
            seq = NULL;
            
            switch (msg.type)
            {
            case TLC_SEQ:
                seq = SelectSeq(msg.value);
                break;
                
            case TLC_INIT:
                // start initializing
#ifdef DEBUG
                printf("TlTask: start init\r\n");
#endif
                if (SearchHomePos() == FAIL)
                    vTaskSuspend(NULL);

                pos_ctr = 30;       // index is at 30 steps from left

                // drive to rest position after init
                seq = move_from_index_to_rest_seq;
                cur_pos = REST_POS;
                break;
                
            case TLC_SYNC:
                xSemaphoreGive(MotionSyncSem);
                break;

            case TLC_COEFF:
                par_a = msg.value;
                break;

            case TLC_SHOW_POS:
                printf("pos=%d\r\n",pos_ctr);
                break;
            }
            
            if (seq != NULL)
            {
                // execute the sequence
                for (i = 0; seq[i].type != END_OF_SEQ; ++i)
                {
                    if (seq[i].value < 0)
                    {
                        step_dir = ST_LEFT;
                        steps = -seq[i].value;
                    }
                    else
                    {
                        step_dir = ST_RIGHT;
                        steps = seq[i].value;
                    }
            
                    switch (seq[i].type)
                    {
                    case TLC_ACCEL:
                        if (mode == STOPPED)
                        {
                            acc_idx = 0;
                            StartStepping();
#ifdef DEBUG
                            printf("start stepping\r\n");
#endif
                        }
                        mode = ACCELERATING;
#ifdef DEBUG
                        printf("start accelerate\r\n");
#endif
                        break;

                    case TLC_DECEL:
                        if (mode != STOPPED)
                        {
                            mode = DECELERATING;
#ifdef DEBUG
                            printf("start decelerate\r\n");
#endif
                        }
                        break;

                    case TLC_SLEW:
                        if (mode != STOPPED)
                        {
                            mode = SLEWING;
#ifdef DEBUG
                            printf("start slew\r\n");
#endif
                        }
                        break;

                    case TLC_STOP:
                        if (mode != STOPPED)
                        {
                            mode = DECELERATING;
                            steps = acc_idx;
                        }
                        break;
                    }

                    // wait for movement end (blocks here)
                    if (xSemaphoreTake(EndMotionSem, MOVE_MAX_WAIT) == pdPASS)
                    {
                        // if result is to stop, disable stepper interrupt
                        if (mode == STOPPED)
                        {
                            StopStepping();
#ifdef DEBUG
                            printf("stop stepping\r\n");
#endif
                        }
                    }
                    else
                    {
                        error_code |= TL_MOTION_ERROR;
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
Select a suitable sequence to be executed according to TLC_SEQ command
and current position. NULL = execute nothing.
--------------------------------------------------------------------- */

static const MOTION_MSG_T *SelectSeq( int16_t scmd )
{
    const MOTION_MSG_T *seq = NULL;
    
    switch (scmd)
    {
    case TLS_TEST:
        seq = cli_msg_tbl;
        break;
        
    case TLS_TILT2ROT:
        if (cur_pos == ROT_POS)
            seq = tilt_from_rot_to_rot_seq;
        else if (cur_pos == TW_POS)
        {
            seq = tilt_from_tw_to_rot_seq;
            cur_pos = ROT_POS;
        }
        break;

    case TLS_TILT2TW:
        if (cur_pos == ROT_POS)
        {
            seq = tilt_from_rot_to_tw_seq;
            cur_pos = TW_POS;
        }
        else if (cur_pos == TW_POS)
            seq = tilt_from_tw_to_tw_seq;
        break;

    case TLS_MOVE2ROT:
        if (cur_pos == REST_POS)
        {
            seq = move_from_rest_to_rot_seq;
            cur_pos = ROT_POS;
        }
        else if (cur_pos == TW_POS)
        {
            seq = move_from_tw_to_rot_seq;
            cur_pos = ROT_POS;
        }
        break;

    case TLS_MOVE2REST:
        if (cur_pos == TW_POS)
        {
            seq = move_from_tw_to_rest_seq;
            cur_pos = REST_POS;
        }
        else if (cur_pos == ROT_POS)
        {
            seq = move_from_rot_to_rest_seq;
            cur_pos = REST_POS;
        }
        break;

    case TLS_MOVE2TW:
        if (cur_pos == ROT_POS)
        {
            seq = move_from_rot_to_tw_seq;
            cur_pos = TW_POS;
        }
        //else if (cur_pos == REST_POS)
        //    seq = move_from_rest_to_tw_seq;   // no need for this
        break;

    case TLS_TILT2TW_1:
        if (cur_pos == ROT_POS)
        {
            seq = tilt_from_rot_to_fr_seq;
            cur_pos = FAR_RIGHT_POS;
        }
        else if (cur_pos == TW_POS)
        {
            seq = tilt_from_tw_to_fr_seq;
            cur_pos = FAR_RIGHT_POS;
        }
        break;

    case TLS_TILT2TW_2:
        if (cur_pos == FAR_RIGHT_POS)
        {
            seq = tilt_from_fr_to_tw_seq;
            cur_pos = TW_POS;
        }
        break;

    case TLS_EXERCISE:
        seq = exercise_seq;
        break;
    }
    
    return seq;
}

/* =====================================================================
Search home position
--------------------------------------------------------------------- */

#define STEP_RIGHT_TIME (300/portTICK_RATE_MS)
#define MAX_SEARCH_TIME (7000/portTICK_RATE_MS)

static int SearchHomePos( void )
{
    int rc;

    const_speed = true; // use constant speed during search

    // if opto is dark already, move to right for a short time
    // to get out from the fork
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) == Bit_SET)
    {
        step_dir = ST_RIGHT;
        StartStepping();
        vTaskDelay(STEP_RIGHT_TIME);
        StopStepping();
        StepperOut(STEPPER_STOP,STEPPER_CONTROL_MASK);
    }

    // Now we are right from index: start moving left
    step_dir = ST_LEFT;

    // enable index interrupt and start motor
    SetIndexInterrupt(ENABLE);
    StartStepping();

    // wait for index interrupt (blocks here)
    if (xSemaphoreTake(IndexSem, MAX_SEARCH_TIME) == pdPASS)
    {
        rc = OK;
#ifdef DEBUG
        printf("index found\r\n");
#endif
    }
    else
    {   // index not found in time
        error_code |= TL_INDEX_ERROR;   // index not found
        rc = FAIL;
    }

    // end movement and disable index interrupt
    StopStepping();
    StepperOut(STEPPER_STOP,STEPPER_CONTROL_MASK);
    SetIndexInterrupt(DISABLE);

    const_speed = false;

    return rc;
}

/* =====================================================================
Module initialization function. Create queues and tasks.
--------------------------------------------------------------------- */

uint32_t TilterInit(void)
{
    // Queue for commands
    TlQueue = xQueueCreate(TL_QUEUE_SIZE, sizeof(MOTION_MSG_T));
    if (TlQueue == NULL)
    {
        return(1);
    }

    // create semaphore for end of movement
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
    if(xTaskCreate(TilterTask, "TL", TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + TASK_PRIORITY, NULL) != pdTRUE)
    {
        return(1);
    }
    
    // Success.
    return(0);
}

/* =======================================================================
Enable/disable EXTI4 interrupt, from tilter motor opto
----------------------------------------------------------------------- */

static void SetIndexInterrupt(FunctionalState state)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    EXTI_ClearITPendingBit(EXTI_Line4);

    // configure NVIC
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TL_INDEX_INT_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = state;
    NVIC_Init(&NVIC_InitStructure);

    // Configure EXTI line
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = state;
    EXTI_Init(&EXTI_InitStructure);
}

/* =======================================================================
Start stepping clock, TIM3 CH1
----------------------------------------------------------------------- */

static void StartStepping(void)
{
    uint16_t cur_ctr;

    cur_ctr = TIM_GetCounter(TIM3);
    TIM_SetCompare1(TIM3, cur_ctr + 200);   // first interrupt after 100 us

    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
}

/* =======================================================================
Stop stepping clock
----------------------------------------------------------------------- */

static void StopStepping(void)
{
    TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);
}

/* =======================================================================
Feeds next step to stepper motor.
----------------------------------------------------------------------- */

void StepTilterMotor(void)
{
    static int16_t cur_step = 0;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t capture;
    uint32_t offset;

    capture = TIM_GetCapture1(TIM3);

    // feed next step to stepper port
    cur_step = (cur_step + step_dir) & STEP_TBL_MASK;
    StepperOut(step_tbl[cur_step],STEPPER_CONTROL_MASK);

    // update position counter
    pos_ctr -= step_dir;

    if (const_speed)
    {
        // drive with constant speed (during index searching)
        TIM_SetCompare1(TIM3, capture + INIT_SPEED);
    }
    else  // normal operation
    {
        // calculate next interrupt interval
        offset = ((uint32_t)par_a * (uint32_t)AccTable[acc_idx]) >> 8;

        TIM_SetCompare1(TIM3, capture + offset);

        // calculate next position in AccTable[]
        switch (mode)
        {
        case ACCELERATING:
            // in the middle of acceleration ramp => next position in table
            if (acc_idx < 255)
                acc_idx++;
            break;

        case DECELERATING:
            // in the middle of deceleration ramp => next position in table
            if (acc_idx != 0)
                acc_idx--;
            break;

        case SLEWING:
            // slewing => no change
            break;
        }

        // if all steps taken, movement has ended
        steps--;
        if (steps == 0)
        {
            if (acc_idx == 0)
            {
                mode = STOPPED;
                step_dir = ST_STOP;
                StepperOut(STEPPER_STOP,STEPPER_CONTROL_MASK);
            }

            xSemaphoreGiveFromISR(EndMotionSem, &xHigherPriorityTaskWoken);
            portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/* =======================================================================
Index interrupt handler
----------------------------------------------------------------------- */

void EXTI4_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Check if EXTI_Line4 is asserted
    if(EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line4);

        xSemaphoreGiveFromISR(IndexSem, &xHigherPriorityTaskWoken);
        portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    }
}

/* =======================================================================
Set stepper port output
    PD0...3 = tilter: states = 0x00...0x0F, mask = 0x000F000F
    PD4...6 = coldet: states = 0x00...0x70, mask = 0x00700070
States are written inverted to port, because ULN2003 inverts it again

Done via BSRR, because this must be atomic and re-entrant!
----------------------------------------------------------------------- */

void StepperOut( uint16_t states, uint32_t mask )
{
    GPIOD->BSRR = (((uint32_t) states << 16) | ~((uint32_t) states)) & mask;
}

/* =====================================================================
Send message to tilter task
--------------------------------------------------------------------- */

void Send2Tlm( uint16_t type, int16_t value )
{
    MOTION_MSG_T msg;

    msg.type = type;
    msg.value = value;
    xQueueSend(TlQueue, &msg, portMAX_DELAY);
}

/* ============================ EOF ====================================== */

