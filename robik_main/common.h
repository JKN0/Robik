/******************************************************************************

  Robik - Rubik's cube solver from junk
  =====================================

  common.h

  Common definitions

  20.9.2017

*****************************************************************************/

#ifndef __COMMON_H__
#define __COMMON_H__

typedef struct motion_msg_t {
	uint16_t type;
	int16_t value;
} MOTION_MSG_T;

#define SYNC_MAX_WAIT		(14000/portTICK_RATE_MS)

#define END_OF_SEQ			0x7FFF
#define SEQ_SYNC			0x7FFE

#define OK                  0
#define FAIL                1

#define STEP_INT_PRIORITY	11			// stepper interrupt priority

// Global error codes
extern uint32_t error_code;

#define TL_INDEX_ERROR      0x00000001
#define TL_MOTION_ERROR     0x00000002
#define TW_INDEX_ERROR      0x00000004
#define TW_MOTION_ERROR     0x00000008
#define CDM_INDEX_ERROR     0x00000010
#define CDM_MOTION_ERROR    0x00000020
#define MOV_SEQ_SYNC_ERROR  0x00000040
#define MOV_TW_SYNC_ERROR   0x00000080
#define MAIN_INIT_ERROR     0x00000100
#define MAIN_SYNC_ERROR     0x00000200
#define COLDET_SYNC_ERROR   0x00000400
#define COLDET_DETECT_ERROR 0x00000800
#define SOLVER_INIT_ERROR   0x00001000
#define SOLVER_ERROR   		0x00002000
#define EXEC_SYNC_ERROR 	0x00004000

extern uint32_t monitor_flags;

#define MON_COLDET          0x00000001
#define MON_SOLVER_COMM     0x00000002
#define MON_DISPLAY         0x00000004

#define TEST_ON 			GPIO_WriteBit(GPIOE, GPIO_Pin_13, Bit_SET)
#define TEST_OFF 			GPIO_WriteBit(GPIOE, GPIO_Pin_13, Bit_RESET)

extern int16_t cli_seq_tbl[];
extern MOTION_MSG_T cli_msg_tbl[];

// Test modes
extern uint8_t test_mode;

#define TEST_NONE			0
#define TEST_TLM			1
#define TEST_TWM			2
#define TEST_CDM			3
#define TEST_DSP			4
#define TEST_MOV			5
#define TEST_CDP			6
#define TEST_CUB			7

#define FACES       6
#define EDGES      12
#define CORNERS     8
#define TILES       9

// Face names
#define FRONT  0
#define LEFT   1
#define BACK   2
#define RIGHT  3
#define UP     4
#define DOWN   5

// Color names
#define NONE   0
#define RED    1
#define ORA    2
#define YEL    3
#define GRN    4
#define BLU    5
#define WHT    6

SemaphoreHandle_t MotionSyncSem;

void Delay( int ms );

#define SLOWDOWN_JUMPER_ON 	(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == Bit_RESET)

#endif // __COMMON_H__
