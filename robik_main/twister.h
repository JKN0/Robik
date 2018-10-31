/******************************************************************************

  Robik - Rubik's cube solver from junk
  =====================================

  twister.h

  Twister module definitions

  20.10.2017

*****************************************************************************/

#ifndef __TWISTER_H__
#define __TWISTER_H__

#define TWC_ABS_POS		    1
#define TWC_REL_POS		    2
#define TWC_WAIT    	    3
#define TWC_SEQ 			4
#define TWC_INIT			5
#define TWC_SYNC			6
#define TWC_KPS			    7
#define TWC_KIS			    8
#define TWC_KDS			    9
#define TWC_KPP			    10
#define TWC_KIP			    11
#define TWC_KDP			    12
#define TWC_SHOW_POS	    13
#define TWC_RECALIB		    14

#define TWS_TEST       		0

// rotate sequences
#define TWS_ROT1       		 1
#define TWS_ROT2   	    	 2
#define TWS_ROT3   		     3
// twist sequences
#define TWS_TWIST1   		 4
#define TWS_TWIST2   		 5
#define TWS_TWIST3   		 6
// color detect sequences
#define TWS_CD_EDGE_START    7
#define TWS_CD_CORNER_START  8
#define TWS_CD_NEXT          9
#define TWS_CD_END           10

#define TW_QUEUE_SIZE		5

xQueueHandle TwQueue;

extern bool tw_need_recalib;

uint32_t TwisterInit(void);

#endif // __TWISTER_H__
