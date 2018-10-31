/******************************************************************************

  Robik - Rubik's cube solver from junk
  =====================================

  cdmotor.h

  Color detector motor module definitions

  7.10.2017

*****************************************************************************/

#ifndef __CDMOTOR_H__
#define __CDMOTOR_H__

#define CDMC_SEQ            1       // value = sequence number, one of CDS_XXXX
#define CDMC_INIT           2
#define CDMC_SYNC           3
#define CDMC_POS            4       // value = position in steps
#define CDMC_COEFF          5       // value = par_a
#define CDMC_LIMIT          6       // value = par_b
#define CDMC_SHOW_POS       7

// valid sequence numbers
#define CDS_TEST            0
#define CDS_CENTER          1
#define CDS_EDGE            2
#define CDS_CORNER          3
#define CDS_REST            4

#define CDM_QUEUE_SIZE      5

xQueueHandle CdmQueue;

uint32_t CdMotorInit(void);

void StepColdetMotor(void);

#endif // __CDMOTOR_H__
