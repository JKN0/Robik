/******************************************************************************

  Robik - Rubik's cube solver from junk
  =====================================

  cdprobe.c

  Color detection module definitions

  22.12.2017

*****************************************************************************/

#ifndef __CDPROBE_H__
#define __CDPROBE_H__

#define COLDET_SAMPLE_INTERVAL 1000  // = 500 us @ 2 MHz clock

#define NONE 0
#define RED  1
#define ORA  2
#define YEL  3
#define GRN  4
#define BLU  5
#define WHT  6

#define READY 99

#define CDP_RES_QUEUE_SIZE	3

SemaphoreHandle_t ColdetStartSem;
xQueueHandle CdpResultQueue;

uint32_t CdProbeInit(void);

void StartAdc(void);

#endif // __CDPROBE_H__
