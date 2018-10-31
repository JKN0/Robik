/******************************************************************************

  Robik - Rubik's cube solver from junk
  =====================================

  tilter.h

  Tilter module definitions

  27.10.2017

*****************************************************************************/

#ifndef __TILTER_H__
#define __TILTER_H__

#define TLC_ACCEL			1
#define TLC_DECEL			2
#define TLC_SLEW			3
#define TLC_STOP			4
#define TLC_SEQ 			5
#define TLC_INIT			6
#define TLC_SYNC			7
#define TLC_COEFF			8
#define TLC_SHOW_POS	    9

#define TLS_TEST    		0
#define TLS_TILT2ROT		1
#define TLS_TILT2TW		    2
#define TLS_MOVE2ROT		3
#define TLS_MOVE2REST		4
#define TLS_MOVE2TW			5
#define TLS_TILT2TW_1	    6
#define TLS_TILT2TW_2	    7
#define TLS_EXERCISE	    8

#define TL_QUEUE_SIZE		5

xQueueHandle TlQueue;

uint32_t TilterInit(void);

void StepTilterMotor(void);
void StepperOut( uint16_t states, uint32_t mask );
void Send2Tlm( uint16_t type, int16_t value );

#endif // __TILTER_H__
