/******************************************************************************

  Robik - Rubik's cube solver from junk
  =====================================

  display.c

  Display control module
  Contains DisplayTask

  8.10.2017

*****************************************************************************/

#include <stdlib.h>
#include <stdbool.h>

#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_tim.h>
#include <misc.h>

#include "tinyprintf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "common.h"
#include "display.h"

/* =====================================================================
------------------------ Constants & macros ------------------------- */

#define TASK_STACK_SIZE     100         // Stack size in words
#define TASK_PRIORITY       2

#define CMDREG               0
#define DATAREG              1

#define QUEUE_MAX_WAIT      (50/portTICK_RATE_MS)

#define YELLOW_BUTTON_PRESSED   (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == Bit_RESET)


/* =====================================================================
------------------------  Global variables  ------------------------- */

bool dsp_led = false;

static const char *blank = "                ";

/* =====================================================================
------------------------ Function prototypes ------------------------ */

void InitDisplayHw( void );
void ShiftTo595( uint8_t val );
void Write4bitToDisp( uint8_t reg, uint8_t val );
void WriteByteToDisp( uint8_t reg, uint8_t val );

/* =====================================================================
Task main function.
--------------------------------------------------------------------- */

static void DisplayTask(void *pvParameters)
{
    DSP_MSG_T msg;
    char *p = NULL;

    Delay(400);

    InitDisplayHw();

    // Main loop.
    while(1)
    {
        // Wait for a command; queue receive timeouts each 50 ms
        while (xQueueReceive(DspQueue, &msg, QUEUE_MAX_WAIT) != pdPASS)
        {
            // check display reset button state
            if (YELLOW_BUTTON_PRESSED)
            {
                msg.cmd = DSPC_INIT;
                break;
            }
        }

        if (monitor_flags & MON_DISPLAY)
            printf("DspTask got: cmd=%d, addr=0x%02X, buf=%s\r\n",msg.cmd,msg.addr,(msg.buf == NULL) ? "" : msg.buf);

        Delay(2);       // sync WriteByteToDisp() delays to tick

        switch (msg.cmd)
        {
        case DSPC_WRITE_CUR:        // write to current position
            p = msg.buf;
            break;

        case DSPC_WRITE_ROW1:       // write to beginning of row 1
            WriteByteToDisp(CMDREG,0x80);
            p = msg.buf;
            break;

        case DSPC_WRITE_ROW2:       // write to beginning of row 2
            WriteByteToDisp(CMDREG,0xC0);
            p = msg.buf;
            break;

        case DSPC_WRITE_ADDR:       // write to given address
            WriteByteToDisp(CMDREG,msg.addr);
            p = msg.buf;
            break;

        case DSPC_CLR_ROW1:     // write blanks to row 1
            WriteByteToDisp(CMDREG,0x80);
            p = (char *)blank;
            break;

        case DSPC_CLR_ROW2:     // write blanks to row 2
            WriteByteToDisp(CMDREG,0xC0);
            p = (char *)blank;
            break;

        case DSPC_UPDATE_LED:   // update led only, no change to display
            ShiftTo595(0);
            p = NULL;
            break;

        case DSPC_INIT:         // init display
            InitDisplayHw();
            p = NULL;
            break;
        }

        if (p != NULL)
        {
            while (*p != '\0')
            {
                WriteByteToDisp(DATAREG,*p);
                p++;
            }
        }
    }
}

/* =====================================================================
Task initialization function. Create queues and tasks.
--------------------------------------------------------------------- */

uint32_t DisplayInit(void)
{

    // Create a queue for receiving display commands from uart task
    DspQueue = xQueueCreate(DSP_QUEUE_SIZE, sizeof(DSP_MSG_T));
    if (DspQueue == NULL)
    {
        return(1);
    }

    // Create the task.
    if(xTaskCreate(DisplayTask, "DSP", TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + TASK_PRIORITY, NULL) != pdTRUE)
    {
        return(1);
    }

    // Success.
    return(0);
}

/* =====================================================================
Initialize display to 4 bit mode, clear display, cursor off etc.
--------------------------------------------------------------------- */

void InitDisplayHw( void )
{
    // Reset sequence
    Delay(30);
    Write4bitToDisp(CMDREG,0x03);
    Delay(5);
    Write4bitToDisp(CMDREG,0x03);
    Delay(5);
    Write4bitToDisp(CMDREG,0x03);
    Delay(5);

    // Use 4 bit mode
    Write4bitToDisp(CMDREG,0x02);
    Delay(5);

    // Other initializations
    WriteByteToDisp(CMDREG,0x28);    // function set command: 4-bit mode, 2 lines, 5x7 dots
    WriteByteToDisp(CMDREG,0x08);    // display off
    WriteByteToDisp(CMDREG,0x01);    // clear display
    WriteByteToDisp(CMDREG,0x06);    // entry mode: display ON,shift OFF
    WriteByteToDisp(CMDREG,0x0C);    // display ON, cursor OFF, blink OFF
}

/* =====================================================================
Shift 7 lowest bits of val to 595 and latch to outputs. Invert
data, because of ULN2003 after 595.

595 data bits:  7  6  5  4  3  2  1  0
                - led RS EN D7 D6 D5 D4
--------------------------------------------------------------------- */

/*
#define SET_595_DATA(d)     GPIO_WriteBit(GPIOE, GPIO_Pin_6, (d) ? Bit_RESET : Bit_SET);   // ULN2003 inverts data!
#define PULSE_595_SHIFT     { GPIO_WriteBit(GPIOE, GPIO_Pin_2, Bit_SET); GPIO_WriteBit(GPIOE, GPIO_Pin_2, Bit_RESET); }
#define PULSE_595_STROBE    { GPIO_WriteBit(GPIOE, GPIO_Pin_5, Bit_SET); GPIO_WriteBit(GPIOE, GPIO_Pin_5, Bit_RESET); }
*/

#define SET_595_DATA(d)     if (d) GPIOE->BRR = GPIO_Pin_6; else GPIOE->BSRR = GPIO_Pin_6;  // ULN2003 inverts data!
#define PULSE_595_SHIFT     { GPIOE->BSRR = GPIO_Pin_2; GPIOE->BRR = GPIO_Pin_2; }
#define PULSE_595_STROBE    { GPIOE->BSRR = GPIO_Pin_5; GPIOE->BRR = GPIO_Pin_5; }

void ShiftTo595( uint8_t val )
{
    uint32_t i;

    // update led state every time
    if (dsp_led)
        val &= ~0x40;
    else
        val |= 0x40;

    // shift 7 bits
    for (i = 0; i < 7; i++)
    {
        SET_595_DATA(val & 0x40);
        PULSE_595_SHIFT;
        val <<= 1;
    }

    PULSE_595_STROBE;
}

/* =====================================================================
Write 4 lowest bits of val to display, setting RS to reg.
--------------------------------------------------------------------- */

#define DISP_EN_MASK        0x10
#define DISP_RS_MASK        0x20

void Write4bitToDisp( uint8_t reg, uint8_t val )
{
    uint8_t byte_out;

    // write 4 bits with EN=0
    byte_out = val & 0x0F;

    if (reg == DATAREG)
        byte_out |= DISP_RS_MASK;

    ShiftTo595(byte_out);

    // pulse EN
    byte_out |= DISP_EN_MASK;
    ShiftTo595(byte_out);
    byte_out &= ~DISP_EN_MASK;
    ShiftTo595(byte_out);
}

/* =====================================================================
Write byte val to display register reg.
--------------------------------------------------------------------- */

void WriteByteToDisp( uint8_t reg, uint8_t val )
{
    Write4bitToDisp(reg, val >> 4);
    Delay(2);
    Write4bitToDisp(reg, val & 0x0F);
    Delay(2);
}

/* =====================================================================
Send message to display task
--------------------------------------------------------------------- */

void Send2Dsp( uint16_t cmd, uint16_t addr, char *buf )
{
    DSP_MSG_T msg;

    msg.cmd = cmd;
    msg.addr = addr;
    msg.buf = buf;
    xQueueSend(DspQueue, &msg, portMAX_DELAY);
}

/* ============================ EOF ====================================== */
