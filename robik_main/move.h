/******************************************************************************

  Robik - Rubik's cube solver from junk
  =====================================

  move.h

  Move module definitions

  17.12.2017

*****************************************************************************/

#ifndef __MOVE_H__
#define __MOVE_H__

#define MVC_S_START         1
#define MVC_S_MOVE          2
#define MVC_C_START         3
#define MVC_C_NEXT_TILE     4
#define MVC_C_END_FACE      5
#define MVC_C_NEXT_FACE     6
#define MVC_SYNC            7
#define MVC_INIT_MOTION     8
#define MVC_FROM_REST       9
#define MVC_TO_REST         10

#define MV_QUEUE_SIZE       5

typedef struct move_msg_t {
    uint16_t type;
    uint8_t face;
    uint8_t turns;
} MOVE_MSG_T;

xQueueHandle MvQueue;
SemaphoreHandle_t MoveSyncSem;

uint32_t MoveInit(void);
void Send2Mov( uint16_t type, uint8_t face, uint8_t turns );
uint32_t SyncWithMov( void );

#endif // __MOVE_H__
