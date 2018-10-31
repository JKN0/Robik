/******************************************************************************

  Robik - Rubik's cube solver from junk
  =====================================

  move.c

  Move coordinating module
  Contains MoveTask

  17.12.2017

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
#include "twister.h"
#include "cdmotor.h"
#include "move.h"

//#define DEBUG

/* =====================================================================
------------------------ Constants & macros ------------------------- */

#define TASK_STACK_SIZE     110         // Stack size in words
#define TASK_PRIORITY       1

#define TL_INDEX_INT_PRIORITY 12        // one higher than step int

#define MOVE_MAX_WAIT       (5000/portTICK_RATE_MS)

// destinations
#define DEST_OWN             0
#define DEST_TLM             1
#define DEST_TWM             2
#define DEST_CDM             3

typedef struct move_step_t {
    uint8_t dest;
    uint16_t type;
    uint8_t value;
} MOVE_STEP_T;

/* =====================================================================
------------------------  Global variables  ------------------------- */

// --- Turn sequences during solving; these only tilt/rotate cube, no twist

// each sequence turns one face down
// before and after the sequence, tilt head is in twist position

static const MOVE_STEP_T turn_f_down_seq[] = {    // turn face F down
    { DEST_TLM, TLC_SEQ,    TLS_MOVE2ROT },
    { DEST_TLM, TLC_SYNC,   0            },
    { DEST_OWN, SEQ_SYNC,   1            },
    { DEST_TWM, TWC_SEQ,    TWS_ROT3     },
    { DEST_TWM, TWC_SYNC,   0            },
    { DEST_OWN, SEQ_SYNC,   1            },
    { DEST_TLM, TLC_SEQ,    TLS_TILT2TW  },
    { DEST_TLM, TLC_SYNC,   0            },
    { DEST_OWN, SEQ_SYNC,   1            },
    { DEST_OWN, END_OF_SEQ, 0            },
};

static const MOVE_STEP_T turn_l_down_seq[] = {    // turn face L down
    { DEST_TLM, TLC_SEQ,    TLS_MOVE2ROT },
    { DEST_TLM, TLC_SYNC,   0            },
    { DEST_OWN, SEQ_SYNC,   1            },
    { DEST_TWM, TWC_SEQ,    TWS_ROT2     },
    { DEST_TWM, TWC_SYNC,   0            },
    { DEST_OWN, SEQ_SYNC,   1            },
    { DEST_TLM, TLC_SEQ,    TLS_TILT2TW  },
    { DEST_TLM, TLC_SYNC,   0            },
    { DEST_OWN, SEQ_SYNC,   1            },
    { DEST_OWN, END_OF_SEQ, 0            },
};

static const MOVE_STEP_T turn_b_down_seq[] = {    // turn face B down
    { DEST_TLM, TLC_SEQ,    TLS_MOVE2ROT },
    { DEST_TLM, TLC_SYNC,   0            },
    { DEST_OWN, SEQ_SYNC,   1            },
    { DEST_TWM, TWC_SEQ,    TWS_ROT1     },
    { DEST_TWM, TWC_SYNC,   0            },
    { DEST_OWN, SEQ_SYNC,   1            },
    { DEST_TLM, TLC_SEQ,    TLS_TILT2TW  },
    { DEST_TLM, TLC_SYNC,   0            },
    { DEST_OWN, SEQ_SYNC,   1            },
    { DEST_OWN, END_OF_SEQ, 0            },
};

static const MOVE_STEP_T turn_r_down_seq[] = {    // turn face R down
    { DEST_TLM, TLC_SEQ,    TLS_TILT2TW  },
    { DEST_TLM, TLC_SYNC,   0            },
    { DEST_OWN, SEQ_SYNC,   1            },
    { DEST_OWN, END_OF_SEQ, 0            },
};

static const MOVE_STEP_T turn_u_down_seq[] = {    // turn face U down
    { DEST_TLM, TLC_SEQ,    TLS_TILT2TW  },
    { DEST_TLM, TLC_SEQ,    TLS_TILT2TW  },
    { DEST_TLM, TLC_SYNC,   0            },
    { DEST_OWN, SEQ_SYNC,   1            },
    { DEST_OWN, END_OF_SEQ, 0            },
};

static const MOVE_STEP_T turn_d_down_seq[] = {    // turn face D down (already is)
    { DEST_TLM, TLC_SEQ,    TLS_MOVE2TW  },
    { DEST_TLM, TLC_SYNC,   0            },
    { DEST_OWN, SEQ_SYNC,   1            },
    { DEST_OWN, END_OF_SEQ, 0            },
};

static const MOVE_STEP_T *turn_seq_tbl[] = {
    turn_f_down_seq,
    turn_l_down_seq,
    turn_b_down_seq,
    turn_r_down_seq,
    turn_u_down_seq,
    turn_d_down_seq
};

// --- Turn sequences during color detection; these only tilt/rotate cube, no twist

// start face scanning:
// - CD to center
static const MOVE_STEP_T cd_to_center_norm_seq[] = {
    { DEST_CDM, CDMC_SEQ,   CDS_CENTER        },
    { DEST_CDM, CDMC_SYNC,  0                 },
    { DEST_OWN, SEQ_SYNC,   1                 },
    { DEST_OWN, END_OF_SEQ, 0                 },
};

// start face scanning after face 4:
// - CD to center
static const MOVE_STEP_T cd_to_center4_seq[] = {
    { DEST_TWM, TWC_SEQ,    TWS_ROT3          },
    { DEST_CDM, CDMC_SEQ,   CDS_CENTER        },
    { DEST_TWM, TWC_SYNC,   0                 },
    { DEST_CDM, CDMC_SYNC,  0                 },
    { DEST_OWN, SEQ_SYNC,   2                 },
    { DEST_OWN, END_OF_SEQ, 0                 },
};

// start edge detections:
// - CD to edge
// - TW to edge position
static const MOVE_STEP_T cd_to_edge_seq[] = {
    { DEST_CDM, CDMC_SEQ,   CDS_EDGE          },
    { DEST_TWM, TWC_SEQ,    TWS_CD_EDGE_START },
    { DEST_CDM, CDMC_SYNC,  0                 },
    { DEST_TWM, TWC_SYNC,   0                 },
    { DEST_OWN, SEQ_SYNC,   2                 },
    { DEST_OWN, END_OF_SEQ, 0                 },
};

// start corner detections:
// - CD to corner
// - TW to corner position
static const MOVE_STEP_T cd_to_corner_seq[] = {
    { DEST_CDM, CDMC_SEQ,   CDS_CORNER          },
    { DEST_TWM, TWC_SEQ,    TWS_CD_CORNER_START },
    { DEST_CDM, CDMC_SYNC,  0                   },
    { DEST_TWM, TWC_SYNC,   0                   },
    { DEST_OWN, SEQ_SYNC,   2                   },
    { DEST_OWN, END_OF_SEQ, 0                   },
};

// move to next tile:
// - TW to next position (one quarter turn clockwise)
static const MOVE_STEP_T cd_next_tile_seq[] = {
    { DEST_TWM, TWC_SEQ,    TWS_CD_NEXT       },
    { DEST_TWM, TWC_SYNC,   0                 },
    { DEST_OWN, SEQ_SYNC,   1                 },
    { DEST_OWN, END_OF_SEQ, 0                 },
};

// end one face scanning:
// - CD to rest
// - TW to initial position
static const MOVE_STEP_T cd_end_face_seq[] = {
    { DEST_CDM, CDMC_SEQ,   CDS_REST          },
    { DEST_TWM, TWC_INIT,   0                 },
    { DEST_CDM, CDMC_SYNC,  0                 },
    { DEST_TWM, TWC_SYNC,   0                 },
    { DEST_OWN, SEQ_SYNC,   2                 },
    { DEST_OWN, END_OF_SEQ, 0                 },
};

// move to next face, normal case:
// - TL tilt
static const MOVE_STEP_T cd_next_face_seq[] = {
    { DEST_TLM, TLC_SEQ,    TLS_TILT2ROT      },
    { DEST_TLM, TLC_SYNC,   0                 },
    { DEST_OWN, SEQ_SYNC,   1                 },
    { DEST_OWN, END_OF_SEQ, 0                 },
};

// move to next face, includes rotations:
// - TW rot 1
// - TL tilt
// - TW rot 3
static const MOVE_STEP_T cd_face4_seq[] = {
    { DEST_TWM, TWC_SEQ,    TWS_ROT1          },
    { DEST_TWM, TWC_SYNC,   0                 },
    { DEST_OWN, SEQ_SYNC,   1                 },
    { DEST_TLM, TLC_SEQ,    TLS_TILT2ROT      },
    { DEST_TLM, TLC_SYNC,   0                 },
    { DEST_OWN, SEQ_SYNC,   1                 },
    { DEST_OWN, END_OF_SEQ, 0                 },
};

// --- Other sequences

// initializing sequence   
static const MOVE_STEP_T init_motion_tasks[] = {
    { DEST_CDM, CDMC_INIT,  0 },
    { DEST_CDM, CDMC_SYNC,  0 },
    { DEST_OWN, SEQ_SYNC,   1 },
    { DEST_TLM, TLC_INIT,   0 },
    { DEST_TWM, TWC_INIT,   0 },
    { DEST_TLM, TLC_SYNC,   0 },
    { DEST_TWM, TWC_SYNC,   0 },
    { DEST_OWN, SEQ_SYNC,   2 },
    { DEST_OWN, END_OF_SEQ, 0 },
};

// move cube from rest position to rotation position and sync
static const MOVE_STEP_T from_rest_seq[] = {
        { DEST_TLM, TLC_SEQ,    TLS_MOVE2ROT      },
        { DEST_TLM, TLC_SYNC,   0                 },
        { DEST_OWN, SEQ_SYNC,   1                 },
        { DEST_OWN, END_OF_SEQ, 0                 },
};

// move cube to rest position and sync
static const MOVE_STEP_T to_rest_seq[] = {
        { DEST_TLM, TLC_SEQ,    TLS_MOVE2REST     },
        { DEST_TLM, TLC_SYNC,   0                 },
        { DEST_OWN, SEQ_SYNC,   1                 },
        { DEST_TWM, TWC_INIT,   0                 },    // initialize twister for next cube
        { DEST_OWN, END_OF_SEQ, 0                 },
};

xQueueHandle *dest_queues[] = { NULL, &TlQueue, &TwQueue, &CdmQueue };

uint8_t cube[6] = { FRONT,LEFT,BACK,RIGHT,UP,DOWN };

uint8_t tile_ctr = 0;
uint8_t face_ctr = 0;

/* =====================================================================
------------------------ Function prototypes ------------------------ */

static uint32_t WaitMotionSync( uint8_t count);
static uint32_t ExecuteSeq( const MOVE_STEP_T *seq );
static const MOVE_STEP_T *FindTurnSeq( uint8_t face );
static uint32_t TwRecalib(void);
static uint32_t TwistCube( uint8_t turns );
static void UpdateCubeTable( const MOVE_STEP_T *seq );
static void TiltCubeTable( void );
static void RotateCubeTable( void );

/* =====================================================================
Move task.
--------------------------------------------------------------------- */

static void MoveTask(void *pvParameters)
{
    MOVE_MSG_T msg;
    const MOVE_STEP_T *seq = NULL;
    bool do_twist = false;

    // Main loop.
    while(1)
    {
        // Wait for a command (blocks here)
        if (xQueueReceive(MvQueue, &msg, portMAX_DELAY) == pdPASS)
        {
#ifdef DEBUG
            printf("MvTask: got cmd: %d:%d:%d\r\n",msg.type,msg.face,msg.turns);
#endif
            seq = NULL;
            do_twist = false;
            
            switch (msg.type)
            {
            case MVC_S_START:
                // initialize cube position table
                cube[FRONT] = FRONT;
                cube[LEFT]  = LEFT;
                cube[BACK]  = BACK;
                cube[RIGHT] = RIGHT;
                cube[UP]    = UP;
                cube[DOWN]  = DOWN;
                break;
                
            case MVC_S_MOVE:
                // find sequence to turn the cube to right position
                seq = FindTurnSeq(msg.face);
                do_twist = true;
                
                if(seq != NULL)
                    // update cube position table according to sequence
                    UpdateCubeTable(seq);
                break;
                
            case MVC_C_START:
                face_ctr = 0;
                tile_ctr = 0;
                break;
                
            case MVC_C_NEXT_TILE:
                switch (tile_ctr)
                {
                case 0:  seq = (face_ctr == 3) ? cd_to_center4_seq : cd_to_center_norm_seq; break;
                case 1:  seq = cd_to_edge_seq; break;
                case 5:  seq = cd_to_corner_seq; break;
                default: seq = cd_next_tile_seq; break;
                }
                tile_ctr++;
                break;
                
            case MVC_C_END_FACE:
                seq = cd_end_face_seq;
                break;
                
            case MVC_C_NEXT_FACE:
                tile_ctr = 0;
                face_ctr++;
                seq = (face_ctr == 3) ? cd_face4_seq : cd_next_face_seq;
                break;
                
            case MVC_SYNC:
                xSemaphoreGive(MoveSyncSem);
                break;
                
            case MVC_INIT_MOTION:
                seq = init_motion_tasks;
                break;

            case MVC_FROM_REST:
                seq = from_rest_seq;
                break;

            case MVC_TO_REST:
                seq = to_rest_seq;
                break;
            }
            
            if (seq != NULL)
            {
                // execute the sequence
                // - during solving, sequence contains cube tilts/rotates, twist is done after that
                // - during color detection, sequence contains cube tilts/rotates, no twist is done
                if (ExecuteSeq(seq) == OK)
                {
                    if (do_twist)
                    {
                        if (TwistCube(msg.turns) == FAIL)
                            error_code |= MOV_TW_SYNC_ERROR;
                    }
                }
                else
                    error_code |= MOV_SEQ_SYNC_ERROR;
            }
        }
    }
}

/* =====================================================================
Execute given sequence by sending all messages to motion tasks and
waiting for syncs.
--------------------------------------------------------------------- */

static uint32_t ExecuteSeq( const MOVE_STEP_T *seq )
{
    int i;
    MOTION_MSG_T msg;
    
    for (i = 0; seq[i].type != END_OF_SEQ; ++i)
    {
        if (seq[i].dest == DEST_OWN)
        {
            if (seq[i].type == SEQ_SYNC)
            {
                // wait for sync (blocks here)
                if (WaitMotionSync(seq[i].value) == FAIL)
                {
                    printf("WaitMotionSync(%d) failed\r\n",seq[i].value);
                    return FAIL;
                }
            }
        }
        else // destination == motion process
        {
            // if twister wants recalibration, replace next tilt command
            // with recalibration
            if (tw_need_recalib && seq[i].dest == DEST_TLM &&
                seq[i].type == TLC_SEQ && seq[i].value == TLS_TILT2TW)
            {
                if (TwRecalib() == FAIL)
                    return FAIL;

                tw_need_recalib = false;
            }
            else
            {
                // send the message to appropriate motion task
                msg.type = seq[i].type;
                msg.value = seq[i].value;
                xQueueSend(*dest_queues[seq[i].dest], &msg, portMAX_DELAY);
            }
        }
    }
    
    return OK;
}
/* =====================================================================
Wait until given number of motion tasks (1...3) has set (given) 
the sync semaphore.
--------------------------------------------------------------------- */

static uint32_t WaitMotionSync( uint8_t count)
{
    uint8_t i;

    for (i = 0; i < count; i++)
    {
        if (xSemaphoreTake(MotionSyncSem,SYNC_MAX_WAIT) != pdPASS)
            return FAIL;
    }
    
    return OK;
}

/* =====================================================================
Find move sequence needed to get the given face facing downwards.
Return pointer to sequence.
--------------------------------------------------------------------- */

static const MOVE_STEP_T *FindTurnSeq( uint8_t face )
{
    uint8_t f;
    
    // find where the desired face currently is
    for (f = FRONT; f < FACES; f++)
    {
        if (cube[f] == face)
            break;
    }
    
    if (f >= FACES)
        return NULL;
    
    return turn_seq_tbl[f];
}

/* =====================================================================
Recalibrate twister by sending TLS_TILT2TW sequence in two parts and
twister init between them. Cube will be on tilting bed during twister
initializing itself.
--------------------------------------------------------------------- */

static uint32_t TwRecalib(void)
{
    MOTION_MSG_T msg;

    // first part of TILT2TW
    msg.type = TLC_SEQ;
    msg.value = TLS_TILT2TW_1;
    xQueueSend(TlQueue, &msg, portMAX_DELAY);

    // wait for execution
    msg.type = TLC_SYNC;
    msg.value = 0;
    xQueueSend(TlQueue, &msg, portMAX_DELAY);

    if (WaitMotionSync(1) == FAIL)
        return FAIL;

    // recalibrate twister
    msg.type = TWC_RECALIB;
    msg.value = 0;
    xQueueSend(TwQueue, &msg, portMAX_DELAY);

    // wait for execution
    msg.type = TWC_SYNC;
    msg.value = 0;
    xQueueSend(TwQueue, &msg, portMAX_DELAY);

    if (WaitMotionSync(1) == FAIL)
        return FAIL;

    // second part of TILT2TW
    msg.type = TLC_SEQ;
    msg.value = TLS_TILT2TW_2;
    xQueueSend(TlQueue, &msg, portMAX_DELAY);

    return OK;
}

/* =====================================================================
Twist the DOWN face counter-clockwise.
Parameter 'turns' gives the number of clockwise quarter turns!
Function blocks until twist is done.
--------------------------------------------------------------------- */

static uint32_t TwistCube( uint8_t turns )
{
    MOTION_MSG_T msg;
    
    switch (turns)
    {
    case 1: msg.value = TWS_TWIST3; break;
    case 2: msg.value = TWS_TWIST2; break;
    case 3: msg.value = TWS_TWIST1; break;
    default: return FAIL;
    }
    
    msg.type = TWC_SEQ;
    xQueueSend(TwQueue, &msg, portMAX_DELAY);
    
    msg.type = TWC_SYNC;
    xQueueSend(TwQueue, &msg, portMAX_DELAY);
    
    return WaitMotionSync(1);
}

/* =====================================================================
Execute given sequence on cube[] table, to get it to the same position
as real cube.
--------------------------------------------------------------------- */

static void UpdateCubeTable( const MOVE_STEP_T *seq )
{
    int i;
    
    for (i = 0; seq[i].type != END_OF_SEQ; ++i)
    {
        if (seq[i].dest == DEST_TLM && seq[i].type == TLC_SEQ)
        {   
            // step is addressed to tilter and contains a tilter sequence number;
            // does it actually tilt the cube? (TLS_MOVExxxx don't tilt)
            if (seq[i].value == TLS_TILT2ROT || seq[i].value == TLS_TILT2TW)
            {
                TiltCubeTable();
            }
        }
        else if (seq[i].dest == DEST_TWM && seq[i].type == TWC_SEQ)
        {
            // step is addressed to twister and contains a twister sequence number;
            // does it actually rotate the cube? (there should not exist others)
            switch (seq[i].value)
            {
            case TWS_ROT1:
                RotateCubeTable();
                break;
                
            case TWS_ROT2:
                RotateCubeTable();
                RotateCubeTable();
                break;

            case TWS_ROT3:
                RotateCubeTable();
                RotateCubeTable();
                RotateCubeTable();
                break;
                
            default:
                break;
            }
        }
    }
}

/* =====================================================================
Tilt cube[] table one quarter of turn clockwise.
--------------------------------------------------------------------- */

static void TiltCubeTable( void )
{
    uint8_t tmp;
    
    tmp         = cube[LEFT];
    cube[LEFT]  = cube[DOWN];
    cube[DOWN]  = cube[RIGHT];
    cube[RIGHT] = cube[UP];
    cube[UP]    = tmp;
}

/* =====================================================================
Rotate cube[] table one quarter of turn clockwise.
--------------------------------------------------------------------- */

static void RotateCubeTable( void )
{
    uint8_t tmp;
    
    tmp         = cube[LEFT];
    cube[LEFT]  = cube[FRONT];
    cube[FRONT] = cube[RIGHT];
    cube[RIGHT] = cube[BACK];
    cube[BACK]  = tmp;
}

/* =====================================================================
Module initialization function. Create queues and tasks.
--------------------------------------------------------------------- */

uint32_t MoveInit(void)
{
    // Queue for commands
    MvQueue = xQueueCreate(MV_QUEUE_SIZE, sizeof(MOVE_MSG_T));
    if (MvQueue == NULL)
    {
        return(1);
    }

    // Syncing semaphore between Move->Main
    MoveSyncSem = xSemaphoreCreateBinary();
    if (MoveSyncSem == NULL)
    {
        return(1);
    }

    // Syncing semaphore between Motions->Move
    MotionSyncSem = xSemaphoreCreateCounting(3,0);
    if (MotionSyncSem == NULL)
    {
        return(1);
    }

    // Create the task.
    if(xTaskCreate(MoveTask, "MOV", TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + TASK_PRIORITY, NULL) != pdTRUE)
    {
        return(1);
    }
    
    // Success.
    return(0);
}

/* =====================================================================
Send message to move task
--------------------------------------------------------------------- */

void Send2Mov( uint16_t type, uint8_t face, uint8_t turns )
{
    MOVE_MSG_T msg;

    msg.type = type;
    msg.face = face;
    msg.turns = turns;
    xQueueSend(MvQueue, &msg, portMAX_DELAY);
}

/* =====================================================================
Sync with move task: send sync command and wait for semaphore
--------------------------------------------------------------------- */

uint32_t SyncWithMov( void )
{
    Send2Mov(MVC_SYNC,0,0);
    if (xSemaphoreTake(MoveSyncSem, SYNC_MAX_WAIT) == pdFAIL)
        return FAIL;
    else
        return OK;
}

/* ============================ EOF ====================================== */

