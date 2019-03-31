/******************************************************************************

  Robik - Rubik's cube solver from junk
  =====================================

  CPU I/O initializations

  16.9.2017

*****************************************************************************/

#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>

#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_adc.h>
#include <misc.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "cdprobe.h"

// -----------------------------------------------------------------------
// System clock initializations

static void InitSysClock(void)
{
#if defined(SYSCLK36)
    // Enable prefetch and 1 wait state for 36 MHz
    FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);
    FLASH_SetLatency( FLASH_Latency_1);
#endif

	// HCLK = PCLK1 = PCLK2 = SYSCLK
	RCC_HCLKConfig( RCC_SYSCLK_Div1);
	RCC_PCLK2Config( RCC_HCLK_Div1);
	RCC_PCLK1Config(RCC_HCLK_Div1);

	// Enable PLL
#if defined(SYSCLK36)
	// PLLCLK = (8MHz / 2) * 9 = 36 MHz
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_9);
#else
	// PLLCLK = (8MHz / 2) * 6 = 24 MHz
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_6);
#endif

	RCC_PLLCmd(ENABLE);

	// Wait until PLL is ready 
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        ;

	// Select PLL as system clock source
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	// Wait until PLL is used as system clock source
	while (RCC_GetSYSCLKSource() != 0x08)
        ;
}

// -----------------------------------------------------------------------
// All IO-pins are initialized here, including alternate functions

static void InitGPIO(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable all GPIO port clocks and AFIO clock
	RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA | 
        RCC_APB2Periph_GPIOB | 
        RCC_APB2Periph_GPIOC | 
        RCC_APB2Periph_GPIOD | 
        RCC_APB2Periph_GPIOE | 
        RCC_APB2Periph_AFIO, 
        ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);

    // Port A ----------------------------------------
    // PA11 = jumper for slowdown (J6 on CPU card)
    // PA10,9 = USART1
    // PA8 = green led on CPU card
    // PA3 = TIM2_CH4, PWM to color probe RGB-led/B
    // PA2 = TIM2_CH3, PWM to color probe RGB-led/G
    // PA1 = TIM2_CH2, PWM to color probe RGB-led/R
    // PA0 = TIM2_CH1, PWM to twister motor
    
    // PA11 input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA10 AF:USART1 Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA9 AF:USART1 Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA8 output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_WriteBit(GPIOA, GPIO_Pin_8, Bit_RESET);  // led off
    
    // PA3...0  AF:TIM2_CH3...0
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_2 | GPIO_Pin_1 | GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Port B ----------------------------------------
    // PB15 = upper button in display module
    // PB13 = lower button in display module
    // PB11,10 = USART3
    // PB7,6 = twister motor hall detectors
    // PB0 = cube detector (opto)
    
    // PB15,13,0 input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_13 | GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // PB11 AF:USART3 Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // PB10 AF:USART3 Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	// PB7,6 AF:TIM4_CH2,1 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Port C ----------------------------------------
    // PC12 = cube detector led
    // PC10 = yellow led on CPU card
    // PC7 = twister index (hall) 
    // PC4 = tilter index (opto) 
    // PC1 = color detector transistor, analog ADC_IN11
    
    // PC12,10 output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_WriteBit(GPIOC, GPIO_Pin_12 | GPIO_Pin_10, Bit_RESET); // leds off

    // PC7,4 input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // PC1 analog input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Port D ----------------------------------------
    // PD13 = solver reset
    // PD6...PD4 = coldet stepper motor drive
    // PD3...PD0 = tilter stepper motor drive

    // PD13,6...0 output
    GPIO_InitStructure.GPIO_Pin = 
        GPIO_Pin_13 | GPIO_Pin_6  | GPIO_Pin_5 | GPIO_Pin_4 | 
        GPIO_Pin_3  | GPIO_Pin_2 | GPIO_Pin_1 | GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_WriteBit(GPIOD, 
        GPIO_Pin_6  | GPIO_Pin_5 | GPIO_Pin_4 |
        GPIO_Pin_3 | GPIO_Pin_2 | GPIO_Pin_1 | GPIO_Pin_0, 
        Bit_RESET);                                   		// motors off
    GPIO_WriteBit(GPIOD, GPIO_Pin_13, Bit_SET);             // reset off

    // Port E ----------------------------------------
    // PE15 = power on to twister driver
    // PE13 = test point
    // PE10 = coldet index (opto)
    // PE6,5,3,2 = 595 shift register control for display
    // PE1 = twister motor direction
    
    // PE15,13,6,5,3,2,1 output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_13 | GPIO_Pin_6 | GPIO_Pin_5 | GPIO_Pin_3 | GPIO_Pin_2 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_WriteBit(GPIOE, GPIO_Pin_15 | GPIO_Pin_1, Bit_SET);    // power on, direction on
    GPIO_WriteBit(GPIOE, GPIO_Pin_13 | GPIO_Pin_6 | GPIO_Pin_5 | GPIO_Pin_3 | GPIO_Pin_2, Bit_RESET); // test off, 595 controls off

    // PE10 input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}

// -----------------------------------------------------------------------
// Initialize USARTs 1 and 3

static void InitUSART(void)
{
	USART_InitTypeDef USART_InitStructure;

    // enable clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    // Both USARTs 19200, 8-N-1, no flow control
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = 19200;
    
    // USART1
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
    
    // USART3
    USART_Init(USART3, &USART_InitStructure);
    USART_Cmd(USART3, ENABLE);
}

// -----------------------------------------------------------------------
// Initialize all timers: TIM2, TIM3, TIM4

static void InitTimer(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	// enable clocks
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

    // Timer 2 -----------------------------------------
    // clock = 12 MHz
    // CH1 = twister PWM
    // CH2 = R-led PWM
    // CH3 = G-led PWM
    // CH4 = B-led PWM

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 1;				// 24,000,000/(1+1) = 12,000,000 => 12 MHz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 1000;               // period = 1000 cycles => PWM = 12 kHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_Pulse = 0;					// PWM width relative to 1000
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	// CH1
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

	// CH2
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

	// CH3
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

	// CH4
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
	TIM_Cmd(TIM2, ENABLE);
    
    // Timer 3 -----------------------------------------
    // clock = 2 MHz
    // CH1 = tilter step clock
    // CH2 = coldet motor step clock
    // CH3 = coldet ADC clock

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 11;					// 24,000,000/(11+1) = 2,000,000  => 2 MHz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1000;

    // CH1
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

    // CH2
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);

    // CH3
	TIM_OCInitStructure.TIM_Pulse = COLDET_SAMPLE_INTERVAL;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
    
    TIM_Cmd(TIM3, ENABLE);
    
    // Timer 4 ------------------------------------------
    // clock = twister encoder
    // CH1...4 = -
    
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig (TIM4,
    							TIM_EncoderMode_TI1,
								TIM_ICPolarity_Rising,
	                            TIM_ICPolarity_Rising);

	TIM_Cmd(TIM4, ENABLE);
}

// -----------------------------------------------------------------------
// Initialize EXTI 4,7,10

static void InitEXTI(void)
{
    // tilter index: PC4 -> EXTI4
    // twister index: PC7 -> EXTI7
    // coldet index: PE10 -> EXTI10
    
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource7);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource10);
}

// -----------------------------------------------------------------------

static void InitAdc(void)
{
	 ADC_InitTypeDef ADC_InitStructure;

	/* ADCCLK = PCLK2/2 = 24/2 = 12MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div2);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_DeInit(ADC1);

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_Cmd(ADC1, ENABLE);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_28Cycles5);

	/* Calibrate */
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1))
		;

	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1))
		;
}

// -----------------------------------------------------------------------

void InitDevice(void)
{
	InitSysClock();
	InitGPIO();
	InitUSART();
	InitTimer();
	InitEXTI();
	InitAdc();

    // From: http://www.freertos.org/FreeRTOS-for-Cortex-M3-STM32-STM32F100-Discovery.html 
    // "It is also recommended to ensure that all four priority bits are assigned as being 
    // premption priority bits. This can be ensured by passing "NVIC_PriorityGroup_4" 
    // into the ST library function NVIC_PriorityGroupConfig()."
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}

