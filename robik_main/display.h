/******************************************************************************

  Robik - Rubik's cube solver from junk
  =====================================

  display.h

  Display module definitions

  8.10.2017

*****************************************************************************/

#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#define DSPC_WRITE_CUR       	0
#define DSPC_WRITE_ROW1      	1
#define DSPC_WRITE_ROW2      	2
#define DSPC_WRITE_ADDR      	3
#define DSPC_CLR_ROW1			4
#define DSPC_CLR_ROW2			5
#define DSPC_UPDATE_LED			6
#define DSPC_INIT 				7

#define DSP_QUEUE_SIZE	5

typedef struct dsp_msg_t {
    uint16_t cmd;
    uint16_t addr;
    char *buf;
} DSP_MSG_T;

xQueueHandle DspQueue;

extern bool dsp_led;

uint32_t DisplayInit(void);
void Send2Dsp( uint16_t cmd, uint16_t addr, char *buf );

#endif // __DISPLAY_H__
