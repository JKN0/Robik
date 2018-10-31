/******************************************************************************
 * Robik - Rubik's cube solver from junk
 *
 * This is the cube solver part, running on old electricity meter board,
 * CPU LPC2366. This communicates through UART1 with main controller board.
 *
 * main.cpp
 *
 * Initialize CPU, parse and check input, call solving algorithm.
 *
 *  6.1.2018
 *
 *****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <LPC23xx.h>

#include "solver.h"
#include "tinyprintf.h"

/* =====================================================================
------------------------ Constants & macros ------------------------- */

#define INBUF_LEN   80

/* =====================================================================
------------------------------ I/O ---------------------------------- */

// Meter board has 3 leds and a button in P1.xx
#define LED1        (1<<26)     // V43 in P1.26
#define LED2        (1<<29)     // V42 in P1.29
#define LED3        (1<<14)     // V41 in P1.14
#define BUTTON      (1<<27)     // S1  in P1.27

#define RESET_REQ   (1<<21)     // P0.21 reset request from main board

/* =====================================================================
------------------------  Global variables  ------------------------- */

char inbuf[INBUF_LEN];
char solver_intbl[20][4];
int uartnr = 1;

/* =====================================================================
------------------------ Function prototypes ------------------------ */

void GetLineFromUART(void);
int CheckMainComms(void);
void BlinkErrorLed( void );
void SetLed( uint32_t led, int state );
int ButtonPressed( void );
void Delay( void );
void SetClock72(void);
void InitUart0(void);
void InitUart1(void);
void InitGPIO(void);
void Uart0Put(void *p, char ch);
char Uart0Get(void);
void Uart1Put(void *p, char ch);
char Uart1Get(void);

extern int CheckColors( void );

/* =====================================================================
Main
--------------------------------------------------------------------- */

int main(void)
{
    int rc,s;

    SetClock72();
    InitGPIO();
    InitUart0();
    InitUart1();

    SetLed(LED1,1);

    // if button is pressed during reset, all communication
    // is done via UART0 for testing
    if (ButtonPressed())
        uartnr = 0;

    if (uartnr == 0)
    {
        // use UART0 for testing
        init_printf(NULL,Uart0Put);
        printf("\r\nSolver test\r\n");
    }
    else
    {
        // use UART1 for communicating with main board
        init_printf(NULL,Uart1Put);

        // check communication with main board
        if (CheckMainComms() == 1)
            BlinkErrorLed();        // does not return
    }

    while (1)
    {
        // wait for color scan result from main board
        GetLineFromUART();

        SetLed(LED2,1);     // light LED2 while solving

        // check colors and convert format for solver algorithm
        rc = CheckColors();
        if (uartnr == 0 && rc == 0)
        {
            printf("solver_intbl: ");
            for (s = 0; s < 20; s++)
                printf("%s ",solver_intbl[s]);
            printf("\r\n");
        }

        if (rc == 0)
            solver(solver_intbl);   // solve the cube
        else
            printf("E%d\r",rc);     // cannot solve

        if (uartnr == 0)
            printf("\n");

        SetLed(LED2,0);
    }
}

/* =====================================================================
Wait for a line
--------------------------------------------------------------------- */

void GetLineFromUART(void)
{
    static uint8_t idx = 0;
    char ch;

    while(1)
    {
        // wait for a char
        if (uartnr == 0)
            ch = Uart0Get() & 0x7F;
        else
            ch = Uart1Get() & 0x7F;

        if (ch == 0)
        {
            continue;
        }
        else if (ch == '\r')
        {
            inbuf[idx] = '\0';
            idx = 0;

            if (uartnr == 0)
                printf("\r\n");     // echo CR only to UART0

            return;
        }
        else if (ch >= ' ')
        {
            if (idx < INBUF_LEN-1)
            {
                inbuf[idx] = ch;
                idx++;

                if (uartnr == 0)
                    printf("%c",ch);        // echo only to UART0
            }
        }
    }
}

/* =======================================================================
Check communication with main board
Wait until main board sends "H\r", reply with "Y\r"
----------------------------------------------------------------------- */

int CheckMainComms(void)
{
    char ch;
    int state = 0;

    while (1)
    {
        ch = Uart1Get();
        if (ch != 0)
        {
            switch (state)
            {
            case 0:
                if (ch == 'H')
                    state = 1;
                break;

            case 1:
                if (ch == '\r')
                {
                    Uart1Put(NULL,'Y');
                    Uart1Put(NULL,'\r');
                    state = 2;
                }
                break;

            default:
                state = 0;
                break;
            }
        }
        else
            return (state == 2) ? 0 : 1;
    }
}

/* =====================================================================
Blink LED3 forever (does not return).
--------------------------------------------------------------------- */

void BlinkErrorLed( void )
{
    int state = 0;

    while (1)
    {
        SetLed(LED3,state);
        state ^= 1;
        Delay();
    }
}

/* =====================================================================
Set led to given state
--------------------------------------------------------------------- */

void SetLed( uint32_t led, int state )
{
    if (state == 0)
        FIO1CLR = led;
    else
        FIO1SET = led;
}

/* =====================================================================
Return 1 if button pressed
--------------------------------------------------------------------- */

int ButtonPressed( void )
{
    if ((FIO1PIN & BUTTON) == 0)
        return 1;
    else
        return 0;
}

/* =====================================================================
Return 1 if reset request is low
--------------------------------------------------------------------- */

int ResetRequestOn( void )
{
    if ((FIO0PIN & RESET_REQ) == 0)
        return 1;
    else
        return 0;
}

/* =====================================================================
Delay
--------------------------------------------------------------------- */

void Delay( void )
{
    volatile int i,c = 0;

    for (i = 0; i < 100000; i++)
        c++;
}

/* =====================================================================
Reset CPU by enabling watchdog and letting it do the reset.
--------------------------------------------------------------------- */

void ResetCPU( void )
{
    WDMOD = 0x03;       // enable watchdog
    WDTC = 50000;       // reset after 50 ms
    WDFEED = 0xAA;      // start watchdog
    WDFEED = 0x55;

    while (1);          // wait for reset
}

/*********************************************************************
************           CPU initializations               *************
**********************************************************************/

// SCS
#define GPIOM       0x01
#define OSCEN       0x20
#define OSCSTAT     0x40

// PLLCON
#define PLLE        0x01
#define PLLC        0x02

// PLLSTAT
#define PLOCK       (1<<26)

// PCONP
#define PCUART0     (1<<3)
#define PCUART1     (1<<4)

#define U0THR_EMPTY (U0LSR & (1<<5))
#define U0RDR_READY (U0LSR & 1)

#define U1THR_EMPTY (U1LSR & (1<<5))
#define U1RDR_READY (U1LSR & 1)

/* =====================================================================
Set clock freq to 72 MHz
--------------------------------------------------------------------- */

void SetClock72( void)
{
    SCS |= OSCEN;                   // main oscillator enable

    while ((SCS & OSCSTAT) == 0)    // wait for main osc ready
        ;

    CLKSRCSEL = 1;                  // select main oscillator as clock source

                                    // Fcco = 288Mhz, M=72, N=5, Fin = 10MHz
    PLLCFG = (4 << 16) | 71;        // ((N-1) << 16) | (M-1)
    PLLFEED = 0xAA;
    PLLFEED = 0x55;

    PLLCON |= PLLE;                 // activate PLL
    PLLFEED = 0xAA;
    PLLFEED = 0x55;

    CCLKCFG = 3;                    // CCLK = Fcco/4 = 72 MHz (CCLKSEL=3)

    while ((PLLSTAT & PLOCK) == 0)  // wait for PLL lock
        ;

    PLLCON |= PLLC;                 // connect PLL
    PLLFEED = 0xAA;
    PLLFEED = 0x55;
}

/* =====================================================================
Init UART0 to 19200
--------------------------------------------------------------------- */

void InitUart0( void)
{
    // pins and power
    PINSEL0 = (PINSEL0 & 0xFFFFFF0F) | 0x00000050;      // enable TXD0 and RXD0
    PCONP = (PCONP | PCUART0) & 0xFFF87FFF;             // power on UART0

    // clocks
    PCLKSEL0 = (PCLKSEL0 & 0xFFFFFF3F) | 0x00000080;    // set UART0 PCLK = CCLK/2 = 36 MHz
    U0LCR |= 0x80;                                      // set DLAB
    U0DLL = 0x75;                                       // DLM/DLL = 0x0075 = 19200bps @ 36MHz
    U0DLM = 0x00;
    //U0FDR = (1<<4) | 0; // MULVAL = 1, DIVADD = 0

    U0FCR = 0x7;                            // fifo enable, reset fifos, trig after 1 char
    U0LCR = 0x3;                            // 8-N-1, reset DLAB
}

/* =====================================================================
Init UART1 to 19200
--------------------------------------------------------------------- */

void InitUart1( void)
{
    // pins and power
    PINSEL0 = (PINSEL0 & 0x3FFFFFFF) | 0x40000000;      // enable TXD1
    PINSEL1 = (PINSEL1 & 0xFFFFFFFC) | 0x00000001;      // enable RXD1
    PCONP = (PCONP | PCUART1) & 0xFFF87FFF;             // power on UART1

    // clocks
    PCLKSEL0 = (PCLKSEL0 & 0xFFFFFCFF) | 0x00000200;    // set UART1 PCLK = CCLK/2 = 36 MHz
    U1LCR |= 0x80;                                      // set DLAB
    U1DLL = 0x75;                                       // DLM/DLL = 0x0075 = 19200bps @ 36MHz
    U1DLM = 0x00;
    //U1FDR = (1<<4) | 0; // MULVAL = 1, DIVADD = 0

    U1FCR = 0x7;                            // fifo enable, reset fifos, trig after 1 char
    U1LCR = 0x3;                            // 8-N-1, reset DLAB
}

/* =====================================================================
Init GPIO
--------------------------------------------------------------------- */

void InitGPIO( void)
{
    SCS |= GPIOM;                           // enable high speed GPIO

    FIO1DIR = LED1 | LED2 | LED3;           // led pins to ouput
    FIO1CLR = LED1 | LED2 | LED3;           // all leds off
}

/* =====================================================================
Send a char to UART0
--------------------------------------------------------------------- */

void Uart0Put(void *p, char ch)
{
    while (!U0THR_EMPTY)
        ;

    U0THR = ch;
}

/* =====================================================================
Wait for a char from UART0
--------------------------------------------------------------------- */

char Uart0Get(void)
{
    while (!U0RDR_READY)
        ;

    return U0RBR;
}

/* =====================================================================
Send a char to UART1
--------------------------------------------------------------------- */

void Uart1Put(void *p, char ch)
{
    while (!U1THR_EMPTY)
        ;

    U1THR = ch;
}

/* =====================================================================
Wait for a char from UART1. Timeout after a few seconds an return 0.
If main board requests reset, perform it.
--------------------------------------------------------------------- */

char Uart1Get(void)
{
    volatile int r_ctr,ctr = 0;

    while (!U1RDR_READY)
    {
        ctr++;
        if (ctr > 1000000)
            return 0;

        if (ResetRequestOn())
        {
            // wait for a while and check again
            r_ctr = 0;
            while (r_ctr < 5000)
                r_ctr++;

            if (ResetRequestOn())
                ResetCPU();
        }
    }

    return U1RBR;
}

/* ============================ EOF ====================================== */
