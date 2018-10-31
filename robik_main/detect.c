/******************************************************************************

  Robik - Rubik's cube solver from junk
  =====================================

  detect.c

  Contains function DetectColors()

  26.12.2017

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

#define RESP_MAX_WAIT		(1000/portTICK_RATE_MS)

#define MOVE_MAX_WAIT		(5000/portTICK_RATE_MS)

typedef struct move_step_t {
	uint8_t dest;
	uint16_t type;
	uint8_t value;
} MOVE_STEP_T;

/* =====================================================================
------------------------  Global variables  ------------------------- */

char result_buf[12];
const char *res_buf_init = "   -   -    ";
                      // tile: 0 1 2 3 4  5 6 7 8
const uint8_t res_idx_tbl[] = {5,6,1,4,9,10,2,0,8 };
const char *col_chars[] = "?ROYGBW";
const char *face_chars[] = "FLBRUD";

/* =====================================================================
------------------------ Function prototypes ------------------------ */

static uint32_t WaitMotionSync( uint8_t count);

/* =====================================================================
Color detection coordinator.
--------------------------------------------------------------------- */

void DetectColors(void)
{
    uint8_t face,tile;
    bool start_scan = true;
	int16_t color;
    
    // tell move task we are starting color scanning
    Send2Mov(MVC_C_START,0,0);

    // loop for all 6 faces
    for (face = FRONT; face <= DOWN; face++)
    {
        strcpy(result_buf,res_buf_init);
        
        // display the progress
        sprintf(dsp_buf,"Scan face %c     ",face_chars[face]);
        Send2Dsp(DSPC_WRITE_ROW1,0,dsp_buf);
        sprintf(dsp_buf," %s   ",result_buf);
        Send2Dsp(DSPC_WRITE_ROW2,0,dsp_buf);
        
        // tell move task to turn next face
        // at first time this is not needed
        if (!start_scan)
            Send2Mov(MVC_C_NEXT_FACE,0,0);
        
        start_scan = false;
        
        // loop for all 9 tiles of a face
        for (tile = 0; tile < 9; tile++)
        {
            // tell move task to position coldet probe to next tile and sync
            Send2Mov(MVC_C_NEXT_TILE,0,0);
            Send2Mov(MVC_SYNC,0,0);
            if (xSemaphoreTake(MoveSyncSem, SYNC_MAX_WAIT) == pdFAIL)
                error_code |= COLDET_SYNC_ERROR;

            if (error_code != 0)
                return;

            // tell CD probe task to detect current color and wait for result
       		xSemaphoreGive(ColdetStartSem);
            if (xQueueReceive(CdpResultQueue, &color, RESP_MAX_WAIT) == pdFAIL ||
                color == NONE)
            {
                error_code |= COLDET_DETECT_ERROR;
                return;
            }

            // store the result to buffer
            result_buf[res_idx_tbl[tile]] = col_chars[color];
            
            // show current result
            sprintf(dsp_buf," %s   ",result_buf);
            Send2Dsp(DSPC_WRITE_ROW2,0,dsp_buf);
        }
        
        // tell move task that this is the end of face
        Send2Mov(MVC_C_END_FACE,0,0);
        
        // send this face's detection result to solver
        printf("To solver: '%s'\r\n",result_buf);
    }
    
    printf("To solver: end of scan\r\n");
}
