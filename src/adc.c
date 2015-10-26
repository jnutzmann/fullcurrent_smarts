/********************************************************************
adc.c

Copyright (c) 2015, Jonathan Nutzmann

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
********************************************************************/

/****************************************************************************
 * Includes
 ***************************************************************************/

#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "misc.h"

#include "system_cfg.h"


#include "adc.h"

/****************************************************************************
 * Definitions
 ***************************************************************************/

//        |  1   |  2   |  3     |  4   |  5    |
// ----------------------------------------------
// ADC 1: |  IA  |  VA  |  VN    |  T   |  TM1  |
// ADC 2: |  IB  |  VB  |  VGATE |  TN  |  HWIF |
// ADC 3: |  IC  |  VC  |  VBUS  |  x   |  x    |
// ----------------------------------------------

#define ADC_Channel_IA    ( ADC_Channel_0 )
#define ADC_Channel_IB    ( ADC_Channel_1 )
#define ADC_Channel_IC    ( ADC_Channel_2 )
#define ADC_Channel_VA    ( ADC_Channel_10 )
#define ADC_Channel_VB    ( ADC_Channel_11 )
#define ADC_Channel_VC    ( ADC_Channel_12 )
#define ADC_Channel_VN    ( ADC_Channel_13 )
#define ADC_Channel_VBUS  ( ADC_Channel_3 )
#define ADC_Channel_T     ( ADC_Channel_4 )
#define ADC_Channel_TN    ( ADC_Channel_5 )
#define ADC_Channel_TM1   ( ADC_Channel_6 )
#define ADC_Channel_VGATE ( ADC_Channel_14 )
#define ADC_Channel_HWIF  ( ADC_Channel_9 )

#define ADC_Rank_IA		  (1)
#define ADC_Rank_IB		  (1)
#define ADC_Rank_IC		  (1)
#define ADC_Rank_VA       (2)
#define ADC_Rank_VB       (2)
#define ADC_Rank_VC       (2)
#define ADC_Rank_VN       (3)
#define ADC_Rank_VBUS     (3)
#define ADC_Rank_T        (4)
#define ADC_Rank_TN		  (4)
#define ADC_Rank_TM1      (5)
#define ADC_Rank_HWIF     (5)
#define ADC_Rank_VGATE    (3)

#define ADC_ADC_IA       (ADC1)
#define ADC_ADC_IB       (ADC2)
#define ADC_ADC_IC       (ADC3)
#define ADC_ADC_VA       (ADC1)
#define ADC_ADC_VB       (ADC2)
#define ADC_ADC_VC       (ADC3)
#define ADC_ADC_VN       (ADC1)
#define ADC_ADC_VBUS     (ADC3)
#define ADC_ADC_T        (ADC1)
#define ADC_ADC_TN		 (ADC2)
#define ADC_ADC_TM1      (ADC1)
#define ADC_ADC_HWIF     (ADC2)
#define ADC_ADC_VGATE    (ADC2)


/****************************************************************************
 * Global Variables
 ***************************************************************************/

static uint16_t adcDMA [ ADC_NUM_CHANNELS * DMA_DEPTH ];
volatile uint32_t dmaTargetIndex = 0;

/****************************************************************************
 * Public Functions
 ***************************************************************************/

/**
 * Inits the ADC.
 * @param fs - Sample frequency in Hz.
 */
void adc_init( uint32_t fs )
{

	/*********************************************************
	 * Clock Initialization
	 *********************************************************/

	// Enable the ADC clocks.
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1, ENABLE );
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC2, ENABLE );
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC3, ENABLE );

	// Enable the timer clock.
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

	// Enable the DMA clock.
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2, ENABLE );

	// Enable the GPIO clocks associated with the ADC pin ports.
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );


	/*********************************************************
	 * GPIO (ADC Pins) Initialization
	 *********************************************************/

	// Turn on the ADC GPIOs
	GPIO_InitTypeDef gpio;
	GPIO_StructInit( &gpio );
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	gpio.GPIO_Mode = GPIO_Mode_AN;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOC, &gpio );

	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init( GPIOA, &gpio );

	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init( GPIOB, &gpio );


	/*********************************************************
	 * Timer 2 (ADC Trigger) Initialization
	 *********************************************************/

	// Setup the timer to run at the sample frequency specified.
	TIM_DeInit( TIM2 );

	TIM_TimeBaseInitTypeDef adcBase;
	TIM_TimeBaseStructInit( &adcBase );
	adcBase.TIM_ClockDivision = TIM_CKD_DIV1;
	adcBase.TIM_CounterMode = TIM_CounterMode_Up;
	adcBase.TIM_Period = (SYS_APB1_HZ * 2) / fs;
	TIM_TimeBaseInit( TIM2, &adcBase );

	TIM_OCInitTypeDef adcOC;
	TIM_OCStructInit( &adcOC );
	adcOC.TIM_OCIdleState = TIM_OCIdleState_Reset;
	adcOC.TIM_OCMode = TIM_OCMode_PWM1;
	adcOC.TIM_OCPolarity = TIM_OCPolarity_Low;
	adcOC.TIM_OutputState = TIM_OutputState_Enable;
	adcOC.TIM_Pulse = adcBase.TIM_Period / 2;
	TIM_OC2Init( TIM2, &adcOC );

	// Enable the timer.
	TIM_Cmd( TIM2, ENABLE );


	/*********************************************************
	 * ADC Initialization - Triple Mode, Regular Simult.
	 *********************************************************/

	ADC_DeInit();

	// Initialize the parameters common to the three ADCs.
	ADC_CommonInitTypeDef adcComInit;
	ADC_CommonStructInit(&adcComInit);

	adcComInit.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	adcComInit.ADC_Mode = ADC_TripleMode_RegSimult;
	adcComInit.ADC_Prescaler = ADC_Prescaler_Div4;

	ADC_CommonInit( &adcComInit );

	// Init each of the three ADCs individually
	ADC_InitTypeDef adcInit;
	ADC_StructInit( &adcInit );
	adcInit.ADC_ContinuousConvMode = DISABLE;
	adcInit.ADC_DataAlign = ADC_DataAlign_Right;
	adcInit.ADC_NbrOfConversion = ADC_NUM_CONVERSIONS;
	adcInit.ADC_Resolution = ADC_Resolution_12b;
	adcInit.ADC_ScanConvMode = ENABLE;
	adcInit.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
	adcInit.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;

	// Start up the three ADC modules.
	ADC_Init( ADC1, &adcInit );
	ADC_Init( ADC2, &adcInit );
	ADC_Init( ADC3, &adcInit );

    #define ADC_SAMPLE_TIME		(ADC_SampleTime_3Cycles)

	// Configure each of the channels
	ADC_RegularChannelConfig( ADC_ADC_IA,   ADC_Channel_IA,   ADC_Rank_IA,   ADC_SAMPLE_TIME );
	ADC_RegularChannelConfig( ADC_ADC_IB,   ADC_Channel_IB,   ADC_Rank_IB,   ADC_SAMPLE_TIME );
	ADC_RegularChannelConfig( ADC_ADC_IC,   ADC_Channel_IC,   ADC_Rank_IC,   ADC_SAMPLE_TIME );
	ADC_RegularChannelConfig( ADC_ADC_VA,   ADC_Channel_VA,   ADC_Rank_VA,   ADC_SAMPLE_TIME );
	ADC_RegularChannelConfig( ADC_ADC_VB,   ADC_Channel_VB,   ADC_Rank_VB,   ADC_SAMPLE_TIME );
	ADC_RegularChannelConfig( ADC_ADC_VC,   ADC_Channel_VC,   ADC_Rank_VC,   ADC_SAMPLE_TIME );
	ADC_RegularChannelConfig( ADC_ADC_VN,   ADC_Channel_VN,   ADC_Rank_VN,   ADC_SAMPLE_TIME );
	ADC_RegularChannelConfig( ADC_ADC_VBUS, ADC_Channel_VBUS, ADC_Rank_VBUS, ADC_SAMPLE_TIME );
	ADC_RegularChannelConfig( ADC_ADC_T,    ADC_Channel_T,    ADC_Rank_T,    ADC_SAMPLE_TIME );
	ADC_RegularChannelConfig( ADC_ADC_TN,   ADC_Channel_TN,   ADC_Rank_TN,   ADC_SAMPLE_TIME );
	ADC_RegularChannelConfig( ADC_ADC_TM1,  ADC_Channel_TM1,  ADC_Rank_TM1,  ADC_SAMPLE_TIME );
	ADC_RegularChannelConfig( ADC_ADC_HWIF, ADC_Channel_HWIF, ADC_Rank_HWIF, ADC_SAMPLE_TIME );
	ADC_RegularChannelConfig( ADC_ADC_VGATE,ADC_Channel_VGATE,ADC_Rank_VGATE,ADC_SAMPLE_TIME );

	// Just sample VC during the don't cares
	ADC_RegularChannelConfig( ADC_ADC_VC,   ADC_Channel_VC,   4,   ADC_SAMPLE_TIME );
	ADC_RegularChannelConfig( ADC_ADC_VC,   ADC_Channel_VC,   5,   ADC_SAMPLE_TIME );

	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	/*********************************************************
	 * DMA Initialization
	 *********************************************************/

	// Disable the stream.
	DMA_DeInit(DMA2_Stream0);

	// Wait for the stream to be disabled before configuring it.
	while ( DMA_GetCmdStatus(DMA2_Stream0) != DISABLE);

	dmaTargetIndex = 0;

	DMA_InitTypeDef dmaInit;

	DMA_StructInit(&dmaInit);

	dmaInit.DMA_BufferSize = ADC_NUM_CHANNELS ;
	dmaInit.DMA_Channel = DMA_Channel_0;
	dmaInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dmaInit.DMA_FIFOMode = DMA_FIFOMode_Enable;
	dmaInit.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	dmaInit.DMA_Memory0BaseAddr = (uint32_t)adcDMA;
	dmaInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dmaInit.DMA_Mode = DMA_Mode_Normal;
	dmaInit.DMA_PeripheralBaseAddr = (uint32_t)(0x40012308);
	dmaInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dmaInit.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_Init(DMA2_Stream0, &dmaInit);

	// Enable the transfer complete interrupt.
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);


	/*********************************************************
	 * NVIC Initialization
	 *********************************************************/

	// Enable the NVIC.
	NVIC_InitTypeDef dmaNVIC;
	dmaNVIC.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	dmaNVIC.NVIC_IRQChannelCmd = ENABLE;
	dmaNVIC.NVIC_IRQChannelPreemptionPriority = 0;
	dmaNVIC.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init( &dmaNVIC );

	// Enable the DMA
	DMA_Cmd( DMA2_Stream0, ENABLE );

	// Enable DMA
	ADC_DMACmd( ADC1, ENABLE );

	// Enable the ADCs
	ADC_Cmd( ADC1, ENABLE );
	ADC_Cmd( ADC2, ENABLE );
	ADC_Cmd( ADC3, ENABLE );
}

/****************************************************************************
 * Interrupt Functions
 ***************************************************************************/

#define HIGH_ISR_MASK           (uint32_t)0x20000000
#define RESERVED_MASK           (uint32_t)0x0F7D0F7D

void DMA2_Stream0_IRQHandler ( void )
{
	if ( DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
	{
		DMA2->LIFCR = (uint32_t)(DMA_IT_TCIF0 & RESERVED_MASK); // DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

		dmaTargetIndex = (dmaTargetIndex + 1) % DMA_DEPTH;

		DMA2_Stream0->M0AR = (uint32_t) &(adcDMA[dmaTargetIndex * ADC_NUM_CHANNELS]);
		DMA2_Stream0->NDTR = ADC_NUM_CHANNELS;
		DMA2_Stream0->CR |= (uint32_t)DMA_SxCR_EN;

		// TODO: Temperature Stuff.
	}
}
