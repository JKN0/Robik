/******************************************************************************

  Robik - Rubik's cube solver from junk
  =====================================

  console.c

  Contains ConsoleTask

  17.9.2017

*****************************************************************************/


#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>
#include <misc.h>

#include "tinyprintf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "common.h"
#include "console.h"
#include "tilter.h"
#include "twister.h"
#include "cdmotor.h"
#include "display.h"
#include "move.h"
#include "cdprobe.h"

/* =====================================================================
------------------------ Constants & macros ------------------------- */

#define TASK_STACK_SIZE     110          // Stack size in words
#define TASK_PRIORITY       1
#define USART1_INT_PRIORITY 14			// one higher than kernel

#define INBUF_LEN			30

/* =====================================================================
------------------------  Global variables  ------------------------- */

char inbuf[INBUF_LEN];
uint8_t test_mode = TEST_NONE;

const char *test_mode_names[] = { "","TLM","TWM","CDM","DSP","MOV","CDP","CUB" };

static const char *tlm_cmd_chars = "-ADWT";
static const char *twm_cmd_chars = "-ARW";
static const char *face_chars = "flbrud";

static char dsp_buf[17];

int16_t cli_seq_tbl[10];
MOTION_MSG_T cli_msg_tbl[10];

const char *col_names[] = {
    "none",
    "red",
    "orange",
    "yellow",
    "green",
    "blue",
    "white"
};

/* =====================================================================
------------------------ Function prototypes ------------------------ */

void GetLineFromConsole(void);
void InitUSART1Interrupt(void);
int TestNoneCLI( char cmd, int par1, int par2 );
int TestTlmCLI( char cmd, int par1, int par2 );
int TestTwmCLI( char cmd, int par1, int par2 );
int TestCdmCLI( char cmd, int par1, int par2 );
int TestDspCLI( char cmd, int par1, char *buf );
int TestMovCLI( char cmd, int par1, int par2 );
int TestCdpCLI( char cmd, int par1, int par2 );
int TestCubCLI( char cmd );

/* =====================================================================
Task function.
--------------------------------------------------------------------- */

static void ConsoleTask(void *pvParameters)
{
	int par1,par2,rc=0;
	char *p,*buf;

	// Ensure that xxxxFromISR() are not called before scheduler is started
	// by enabling interrupts not earlier than here
	InitUSART1Interrupt();

	// start using putc that writes to send queue
	init_printf(0,putc_cons_queue);

    // Main loop.
    while(1)
    {
		// Wait for a line (blocks here until complete line received)
		GetLineFromConsole();

		if (strlen(inbuf) > 0)
		{
			if (strlen(inbuf) > 1 && inbuf[1] >= 'a')
			{
				if (strncmp(inbuf,"exit",4) == 0)
					test_mode = TEST_NONE;
				else if (strncmp(inbuf,"tlm",3) == 0)
					test_mode = TEST_TLM;
				else if (strncmp(inbuf,"twm",3) == 0)
					test_mode = TEST_TWM;
				else if (strncmp(inbuf,"cdm",3) == 0)
					test_mode = TEST_CDM;
				else if (strncmp(inbuf,"dsp",3) == 0)
					test_mode = TEST_DSP;
				else if (strncmp(inbuf,"mov",3) == 0)
					test_mode = TEST_MOV;
				else if (strncmp(inbuf,"cdp",3) == 0)
					test_mode = TEST_CDP;
				else if (strncmp(inbuf,"cub",3) == 0)
					test_mode = TEST_CUB;
				else
					printf("Unknown command\r\n");

				if (test_mode != TEST_NONE)
				{
					sprintf(dsp_buf,"Test mode %s   ",test_mode_names[test_mode]);
					Send2Dsp(DSPC_WRITE_ROW1,0,dsp_buf);
					Send2Dsp(DSPC_CLR_ROW2,0,NULL);
				}
			}
			else
			{
				par1 = atoi(inbuf+1);

				p = strchr(inbuf,',');
				if (p == NULL)
				{
					par2 = 0;
					buf = "";
				}
				else
				{
					par2 = atoi(p+1);
					buf = p + 1;
				}

				switch (test_mode)
				{
				case TEST_NONE:
					rc = TestNoneCLI(inbuf[0],par1,par2);
					break;

				case TEST_TLM:
					rc = TestTlmCLI(inbuf[0],par1,par2);
					break;

				case TEST_TWM:
					rc = TestTwmCLI(inbuf[0],par1,par2);
					break;

				case TEST_CDM:
					rc = TestCdmCLI(inbuf[0],par1,par2);
					break;

				case TEST_DSP:
					rc = TestDspCLI(inbuf[0],par1,buf);
					break;

				case TEST_MOV:
					rc = TestMovCLI(inbuf[0],par1,par2);
					break;

				case TEST_CDP:
					rc = TestCdpCLI(inbuf[0],par1,par2);
					break;

				case TEST_CUB:
					rc = TestCubCLI(inbuf[0]);
					break;
				}

				if (rc == FAIL)
					printf("Unknown command\r\n");
			}
		}

		printf("%s>",test_mode_names[test_mode]);
    }
}

/* =====================================================================
Task initialization function. Create queues and task.
--------------------------------------------------------------------- */

uint32_t ConsoleInit(void)
{

    // Create a queue for received characters
	ConsoleRxQueue = xQueueCreate(CONS_RX_QUEUE_SIZE, sizeof(char));

    // Create a queue for sending characters
	ConsoleTxQueue = xQueueCreate(CONS_TX_QUEUE_SIZE, sizeof(char));

    // Create the uart task.
    if(xTaskCreate(ConsoleTask, "CONS", TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + TASK_PRIORITY, NULL) != pdTRUE)
    {
        return(1);
    }

    // Success.
    return(0);
}

/* =====================================================================
Get one line of chars from rx queue
--------------------------------------------------------------------- */

void GetLineFromConsole(void)
{
	static uint8_t idx = 0;
	char ch;

    while(1)
    {
		// wait for a char from interrupt service
    	if (xQueueReceive(ConsoleRxQueue, &ch, portMAX_DELAY) == pdPASS)
		{
			ch &= 0x7F;

			if (ch == '\r')
			{
				inbuf[idx] = '\0';
				idx = 0;

				printf("\r\n");		// echo CR

				return;
			}
			else if (ch == '\b' && idx > 0)
			{
				idx--;
				printf("\b \b");		// echo BS
			}
			else if (ch >= ' ')
			{
				ch |= 0x20;		// to lowercase

				if (idx < INBUF_LEN-1)
				{
					inbuf[idx] = ch;
					idx++;

					putc_cons_queue(NULL,ch);	// echo
				}
			}
		}
    }
}

/* =====================================================================
CLI for TEST_NONE mode.
--------------------------------------------------------------------- */

int TestNoneCLI( char cmd, int par1, int par2 )
{
	switch (cmd)
	{
	case 'f':
		printf("free heap: %d bytes\r\n",xPortGetFreeHeapSize());
		return OK;

	case 'e':
		printf("error code: %08X\r\n",error_code);
		return OK;

	case 'm':
		switch (par1)
		{
		case 0:
			monitor_flags = 0;
			printf("All monitors off\r\n");
			break;

		case 1:
			monitor_flags |= MON_COLDET;
			printf("Monitoring color detection\r\n");
			break;

		case 2:
			monitor_flags |= MON_SOLVER_COMM;
			printf("Monitoring solver comm\r\n");
			break;

		case 3:
			monitor_flags |= MON_DISPLAY;
			printf("Monitoring display messages\r\n");
			break;

		default:
			printf("Unknown monitoring\r\n");
			break;
		}
		return OK;

	case 'h':
		printf(	"Test modes: TLM,TWM,CDM,MOV,DSP,CDP,CUB\r\n"
				"Other commands:\r\n"
				"  f  - show free heap\r\n"
				"  e  - show error code\r\n"
				"  m1 - monitor color detection\r\n"
				"  m2 - monitor solver comm\r\n"
				"  m3 - monitor display\r\n"
				"  m0 - monitors off\r\n");
		return OK;
	}

	return FAIL;
}

/* =====================================================================
CLI for TEST_TLM mode.
--------------------------------------------------------------------- */

int TestTlmCLI( char cmd, int par1, int par2 )
{
	MOTION_MSG_T msg;
	int i;

	switch (cmd)
	{
	case 'a':
		cli_msg_tbl[par1].type = TLC_ACCEL;
		cli_msg_tbl[par1].value = par2;
		return OK;

	case 'd':
		cli_msg_tbl[par1].type = TLC_DECEL;
		cli_msg_tbl[par1].value = par2;
		return OK;

	case 'w':
		cli_msg_tbl[par1].type = TLC_SLEW;
		cli_msg_tbl[par1].value = par2;
		return OK;

	case 't':
		cli_msg_tbl[par1].type = TLC_STOP;
		cli_msg_tbl[par1].value = 0;
		return OK;

	case 'e':
		cli_msg_tbl[par1].type = END_OF_SEQ;
		cli_msg_tbl[par1].value = 0;
		return OK;

	case 'x':
		msg.type = TLC_SEQ;
		msg.value = TLS_TEST;
	    xQueueSend(TlQueue, &msg, portMAX_DELAY);
		return OK;

	case 'l':
		for (i = 0; i < 10; ++i)
			if (cli_msg_tbl[i].type == END_OF_SEQ)
			    printf("tbl[%d] = END\r\n",i);
			else
				printf("tbl[%d] = %c %d\r\n",i,tlm_cmd_chars[cli_msg_tbl[i].type],cli_msg_tbl[i].value);
		return OK;

	case 's':
		msg.type = TLC_SEQ;
		msg.value = par1;
	    xQueueSend(TlQueue, &msg, portMAX_DELAY);
		return OK;

	case 'n':
		msg.type = TLC_INIT;
		msg.value = 0;
	    xQueueSend(TlQueue, &msg, portMAX_DELAY);
		return OK;

	case 'y':
		msg.type = TLC_SYNC;
		msg.value = 0;
	    xQueueSend(TlQueue, &msg, portMAX_DELAY);

    	printf("Waiting sync...");
		if (xSemaphoreTake(MotionSyncSem, SYNC_MAX_WAIT) == pdPASS)
			printf("got\r\n");
		else
			printf("error\r\n");
		return OK;

	case 'c':
		msg.type = TLC_COEFF;
		msg.value = par1;
	    xQueueSend(TlQueue, &msg, portMAX_DELAY);
		return OK;

	case 'q':
		msg.type = TLC_SHOW_POS;
		msg.value = 0;
	    xQueueSend(TlQueue, &msg, portMAX_DELAY);
		return OK;

	case 'h':
		printf(	"TLM commands:\r\n"
				"  a<i>,<st> - accel\r\n"
				"  d<i>,<st> - decel\r\n"
				"  w<i>,<st> - slew\r\n"
				"  t<i>      - stop\r\n"
				"  e<i>      - end-of-seq\r\n"
				"  x         - exec test seq\r\n"
				"  l         - list\r\n"
		        "  s<n>      - exec seq n\r\n"
				"  n         - init\r\n"
		        "  y         - sync\r\n"
				"  c<coeff>  - coeff\r\n"
				"  q         - qry pos\r\n");
		return OK;
	}

	return FAIL;
}

/* =====================================================================
CLI for TEST_TWM mode.
--------------------------------------------------------------------- */

int TestTwmCLI( char cmd, int par1, int par2 )
{
	MOTION_MSG_T msg;
	int i;

	switch (cmd)
	{
	case 'r':
		cli_msg_tbl[par1].type = TWC_REL_POS;
		cli_msg_tbl[par1].value = par2;
		return OK;

	case 'a':
		cli_msg_tbl[par1].type = TWC_ABS_POS;
		cli_msg_tbl[par1].value = par2;
		return OK;

	case 'w':
		cli_msg_tbl[par1].type = TWC_WAIT;
		cli_msg_tbl[par1].value = par2;
		return OK;

	case 'e':
		cli_msg_tbl[par1].type = END_OF_SEQ;
		cli_msg_tbl[par1].value = 0;
		return OK;

	case 'x':
		msg.type = TWC_SEQ;
		msg.value = TWS_TEST;
	    xQueueSend(TwQueue, &msg, portMAX_DELAY);
		return OK;

	case 'l':
		for (i = 0; i < 10; ++i)
			if (cli_msg_tbl[i].type == END_OF_SEQ)
			    printf("tbl[%d] = END\r\n",i);
			else
				printf("tbl[%d] = %c %d\r\n",i,twm_cmd_chars[cli_msg_tbl[i].type],cli_msg_tbl[i].value);
		return OK;

	case 'g':
		msg.type = TWC_ABS_POS;
		msg.value = par1;
		xQueueSend(TwQueue, &msg, portMAX_DELAY);
		return OK;

	case 's':
		msg.type = TWC_SEQ;
		msg.value = par1;
	    xQueueSend(TwQueue, &msg, portMAX_DELAY);
		return OK;

	case 'n':
		msg.type = TWC_INIT;
		msg.value = 0;
	    xQueueSend(TwQueue, &msg, portMAX_DELAY);
		return OK;

	case 'c':
		msg.type = TWC_RECALIB;
		msg.value = 0;
	    xQueueSend(TwQueue, &msg, portMAX_DELAY);
		return OK;

	case 'y':
		msg.type = TWC_SYNC;
		msg.value = 0;
	    xQueueSend(TwQueue, &msg, portMAX_DELAY);

    	printf("Waiting sync...");
		if (xSemaphoreTake(MotionSyncSem, SYNC_MAX_WAIT) == pdPASS)
			printf("got\r\n");
		else
			printf("error\r\n");
		return OK;

	case 'p':
		msg.type = (par1 == 1 ? TWC_KPS : TWC_KPP);
		msg.value = par2;
	    xQueueSend(TwQueue, &msg, portMAX_DELAY);
		return OK;

	case 'i':
		msg.type = (par1 == 1 ? TWC_KIS : TWC_KIP);
		msg.value = par2;
	    xQueueSend(TwQueue, &msg, portMAX_DELAY);
		return OK;

	case 'd':
		msg.type = (par1 == 1 ? TWC_KDS : TWC_KDP);
		msg.value = par2;
	    xQueueSend(TwQueue, &msg, portMAX_DELAY);
		return OK;

	case 'q':
		msg.type = TWC_SHOW_POS;
		msg.value = 0;
	    xQueueSend(TwQueue, &msg, portMAX_DELAY);
		return OK;

	case 'h':
		printf(	"TWM commands:\r\n"
				"  r<i>,<pos> - rel pos\r\n"
				"  a<i>,<pos> - abs pos\r\n"
				"  w<i>,<ms>  - wait\r\n"
				"  e<i>       - end-of-seq\r\n"
				"  x          - exec test seq\r\n"
				"  l          - list\r\n"
				"  g<pos>     - go abs pos\r\n"
		        "  s<n>       - exec seq n\r\n"
				"  n          - init\r\n"
				"  c          - recalib\r\n"
		        "  y          - sync\r\n"
				"  p1,<kp>    - kp speed\r\n"
				"  i1,<ki>    - ki speed\r\n"
				"  d1,<kd>    - kd speed\r\n"
				"  p2,<kp>    - kp pos\r\n"
				"  i2,<ki>    - ki pos\r\n"
				"  d2,<kd>    - kd pos\r\n"
				"  q          - qry pos\r\n");
		return OK;
	}

	return FAIL;
}

/* =====================================================================
CLI for TEST_CDM mode.
--------------------------------------------------------------------- */

int TestCdmCLI( char cmd, int par1, int par2 )
{
	MOTION_MSG_T msg;
	int i;

	switch (cmd)
	{
	case 'p':
		cli_seq_tbl[par1] = par2;
		return OK;

	case 'e':
		cli_seq_tbl[par1] = END_OF_SEQ;
		return OK;

	case 'x':
		msg.type = CDMC_SEQ;
		msg.value = CDS_TEST;
	    xQueueSend(CdmQueue, &msg, portMAX_DELAY);
		return OK;

	case 'l':
		for (i = 0; i < 10; ++i)
			if (cli_seq_tbl[i] == END_OF_SEQ)
				printf("tbl[%d] = END\r\n",i);
			else
				printf("tbl[%d] = %d\r\n",i,cli_seq_tbl[i]);
		return OK;

	case 's':
		msg.type = CDMC_SEQ;
		msg.value = par1;
	    xQueueSend(CdmQueue, &msg, portMAX_DELAY);
		return OK;

	case 'n':
		msg.type = CDMC_INIT;
		msg.value = 0;
	    xQueueSend(CdmQueue, &msg, portMAX_DELAY);
		return OK;

	case 'y':
		msg.type = CDMC_SYNC;
		msg.value = 0;
	    xQueueSend(CdmQueue, &msg, portMAX_DELAY);

    	printf("Waiting sync...");
		if (xSemaphoreTake(MotionSyncSem, SYNC_MAX_WAIT) == pdPASS)
			printf("got\r\n");
		else
			printf("error\r\n");
		return OK;

	case 'c':
		msg.type = CDMC_COEFF;
		msg.value = par1;
	    xQueueSend(CdmQueue, &msg, portMAX_DELAY);
		return OK;

	case 'm':
		msg.type = CDMC_LIMIT;
		msg.value = par1;
	    xQueueSend(CdmQueue, &msg, portMAX_DELAY);
		return OK;

	case 'q':
		msg.type = CDMC_SHOW_POS;
		msg.value = 0;
	    xQueueSend(CdmQueue, &msg, portMAX_DELAY);
		return OK;

	case 'h':
		printf(	"CDM commands:\r\n"
				"  p<i>,<pos> - position\r\n"
				"  e<i>       - end-of-seq\r\n"
				"  x          - exec test seq\r\n"
				"  l          - list\r\n"
		        "  s<n>       - exec seq n\r\n"
				"  n          - init\r\n"
		        "  y          - sync\r\n"
				"  c<coeff>   - coeff\r\n"
				"  m<limit>   - limit\r\n"
				"  q          - qry pos\r\n");
		return OK;
	}

	return FAIL;
}

/* =====================================================================
CLI for TEST_DSP mode.
--------------------------------------------------------------------- */

int TestDspCLI( char cmd, int par1, char *buf )
{
	uint16_t dcmd;

	switch (cmd)
	{
	case 'r':
		dcmd = par1 == 1 ? DSPC_WRITE_ROW1 : DSPC_WRITE_ROW2;
		Send2Dsp(dcmd,0,buf);
		return OK;

	case 'w':
		// par1 = addr to write to
		//   0 = write to current pos
		//   128...144 for row 1
		//   192...208 for row 2
		dcmd = par1 == 0 ? DSPC_WRITE_CUR : DSPC_WRITE_ADDR;
		Send2Dsp(dcmd,par1,buf);
		return OK;

	case 'c':
		// par1: 1 = row1, 2 = row2
		dcmd = par1 == 1 ? DSPC_CLR_ROW1 : DSPC_CLR_ROW2;
		Send2Dsp(dcmd,0,NULL);
		return OK;

	case 'l':
		// par1: 1 = led on, 0 = led off
		dsp_led = par1 == 0 ? false : true;
		Send2Dsp(DSPC_UPDATE_LED,0,NULL);
		return OK;

	case 'n':
		Send2Dsp(DSPC_INIT,0,NULL);
		return OK;

	case 'h':
		printf(	"DSP commands:\r\n"
		        "  r<n>,<txt> - write text to row n\r\n"
		        "  w<a>,<txt> - write text to address a\r\n"
		        "  l<st>      - set led to state st\r\n"
		        "  c<n>       - clear row n\r\n"
				"  n          - init\r\n");
		return OK;
	}

	return FAIL;
}

/* =====================================================================
CLI for TEST_MOV mode.
--------------------------------------------------------------------- */

int TestMovCLI( char cmd, int par1, int par2 )
{
	int f;
	uint16_t mcmd;

	switch (cmd)
	{
	case 's':
	    Send2Mov(MVC_S_START,0,0);
		return OK;

	case 'n':
	    Send2Mov(MVC_INIT_MOTION,0,0);
		return OK;

	case 'e':
		mcmd = par1 == 1 ? MVC_FROM_REST : MVC_TO_REST;
	    Send2Mov(mcmd,0,0);
		return OK;

	case 'y':
	    Send2Mov(MVC_SYNC,0,0);

    	printf("Waiting sync...");
		if (xSemaphoreTake(MoveSyncSem, SYNC_MAX_WAIT) == pdPASS)
			printf("got\r\n");
		else
			printf("error\r\n");
		return OK;

	case 'h':
		printf(	"MOV commands:\r\n"
				"  s      - start moves\r\n"
				"  <f><n> - f=FLBRUD, n=turns\r\n"
				"  e1     - move from rest\r\n"
				"  e2     - move to rest\r\n"
				"  n      - init\r\n"
		        "  y      - sync\r\n");
		return OK;

	default:
		for (f = 0; f < FACES; f++)
		{
			if (face_chars[f] == cmd)
				break;
		}

		if (f >= FACES)
			return FAIL;

	    Send2Mov(MVC_S_MOVE,f,par1);
		return OK;
	}

	return FAIL;
}

/* =====================================================================
CLI for TEST_CDP mode.
--------------------------------------------------------------------- */

#define RESP_MAX_WAIT		(3000/portTICK_RATE_MS)

int TestCdpCLI( char cmd, int par1, int par2 )
{
	int16_t color;

	switch (cmd)
	{
	case 's':
	    Send2Mov(MVC_C_START,0,0);
		return OK;

	case 't':
	    Send2Mov(MVC_C_NEXT_TILE,0,0);
		return OK;

	case 'e':
	    Send2Mov(MVC_C_END_FACE,0,0);
		return OK;

	case 'f':
	    Send2Mov(MVC_C_NEXT_FACE,0,0);
		return OK;

	case 'm':
		xSemaphoreGive(ColdetStartSem);
		if (xQueueReceive(CdpResultQueue, &color, RESP_MAX_WAIT) == pdPASS)
			printf("Got result %d = %s\r\n",color,col_names[color]);
		else
			printf("Result wait failed\r\n");
		return OK;

	case 'n':
	    Send2Mov(MVC_INIT_MOTION,0,0);
		return OK;

	case 'y':
	    Send2Mov(MVC_SYNC,0,0);

    	printf("Waiting sync...");
		if (xSemaphoreTake(MoveSyncSem, SYNC_MAX_WAIT) == pdPASS)
			printf("got\r\n");
		else
			printf("error\r\n");
		return OK;

	case 'h':
		printf(	"CDP commands:\r\n"
				"  s - start scan\r\n"
				"  t - next tile\r\n"
				"  e - end face\r\n"
				"  f - next face\r\n"
				"  m - measure\r\n"
				"  n - init motion\r\n"
		        "  y - sync\r\n");
		return OK;
	}

	return FAIL;
}

/* =====================================================================
CLI for TEST_CUB mode.
--------------------------------------------------------------------- */

int TestCubCLI( char cmd )
{
	switch (cmd)
	{
	case 'h':
		printf(	"CUB: no commands\r\n");
		return OK;
	}

	return FAIL;
}

/* =====================================================================
USART interrupt.
--------------------------------------------------------------------- */

void USART1_IRQHandler(void)
{
	char ch;

	// Add received chars to queue
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		 // Receive the character and put to queue
		ch = USART_ReceiveData(USART1);
	    if (xQueueSendFromISR(ConsoleRxQueue, &ch, NULL) != pdPASS)
	    {
	        // Error. The queue should never be full.
	        while(1);
	    }
	}

	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
		 // Anything to send?
		if (xQueueReceiveFromISR(ConsoleTxQueue, &ch, NULL) == pdPASS)
			USART_SendData(USART1, ch); // Transmit the character
		else
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE); // Suppress interrupt when queue empty
	}
}

/* =======================================================================
Output one char to console tx queue (USART1)
----------------------------------------------------------------------- */

void putc_cons_queue(void *p, char ch)
{
	xQueueSend(ConsoleTxQueue, &ch, portMAX_DELAY);
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

/* =======================================================================
Output one char directly to USART1
----------------------------------------------------------------------- */

void putc_cons_poll(void *p, char ch)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    	;

    USART_SendData(USART1,ch);
}

/* =======================================================================
Init USART1 interrupt
----------------------------------------------------------------------- */

void InitUSART1Interrupt(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	// Enable USART1 IRQ
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART1_INT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

/* ============================ EOF ====================================== */
