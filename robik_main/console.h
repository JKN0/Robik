/******************************************************************************

  Robik - Rubik's cube solver from junk
  =====================================

  console.h

  Contains ConsoleTask

  17.9.2017

*****************************************************************************/

#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#define CONS_RX_QUEUE_SIZE		10
#define CONS_TX_QUEUE_SIZE		80

xQueueHandle ConsoleRxQueue;
xQueueHandle ConsoleTxQueue;

uint32_t ConsoleInit(void);
void putc_cons_queue(void *p, char ch);
void putc_cons_poll(void *p, char ch);

#endif // __CONSOLE_H__
