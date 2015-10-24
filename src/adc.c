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

#include "system_cfg.h"

#include "arm_math.h"

#include "adc.h"

#include "calibration.h"
#include "fir_filters.h"

#include "stdio.h"
#include "stdlib.h"

/****************************************************************************
 * Definitions
 ***************************************************************************/

#define ADC_Channel_VA    ( ADC_Channel_10 )
#define ADC_Channel_VB    ( ADC_Channel_11 )
#define ADC_Channel_VC    ( ADC_Channel_12 )
#define ADC_Channel_VN    ( ADC_Channel_13 )
#define ADC_Channel_IA    ( ADC_Channel_0 )
#define ADC_Channel_IB    ( ADC_Channel_1 )
#define ADC_Channel_IC    ( ADC_Channel_2 )
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

#define VREF				( 2.5f )
#define BITS				( 4096 )

#define CURRENT_SENSE_VCC	 ( 3.3f )
#define CURRENT_FULL_SCALE	 ( 200.0f )
#define CURRENT_SENSE_GAIN	 ( 30000.0f / 40000.0f )
#define CURRENT_SENSE_ZERO_V ( (CURRENT_SENSE_VCC/2) * CURRENT_SENSE_GAIN )
#define CURRENT_ZERO_RAW	 ((uint16_t) ( CURRENT_SENSE_ZERO_V / (VREF / BITS) ))
#define CURRENT_HIGH_CAL_VAL ( ((VREF-CURRENT_SENSE_ZERO_V)/CURRENT_SENSE_GAIN) / (CURRENT_SENSE_VCC/2) * CURRENT_FULL_SCALE )

#define HIGH_VOLTAGE_GAIN         ( 400000.0f / 5100.0f )
#define HIGH_VOLTAGE_HIGH_CAL_VAL ( HIGH_VOLTAGE_GAIN * VREF )

#define VGATE_GAIN			( 44000.0f / 5100 )
#define VGATE_HIGH_CAL_VAL  ( VGATE_GAIN * VREF )

USE_FILTER_HAMMING_4KHZ;

/****************************************************************************
 * Global Variables
 ***************************************************************************/

//__no_init
static uint16_t adcDMA [ ADC_NUM_CHANNELS * DMA_DEPTH ];
//@ 0x2001C000;
static float         adcCache [ ADC_NUM_CHANNELS ];
volatile uint32_t    dmaTargetIndex = 0;

/****************************************************************************
 * Public Functions
 ***************************************************************************/

/**
 * Sets up the default calibration values for all of the ADC channels.
 */
void adc_setDefaultCalibrationValues ( void )
{
	cal_SetLowCal  ( &(channelCalibrations[ADC_CH_INDEX_IA]), CURRENT_ZERO_RAW, 0.0f );
	cal_SetHighCal ( &(channelCalibrations[ADC_CH_INDEX_IA]), BITS, CURRENT_HIGH_CAL_VAL );
	cal_storeCalibrationInFRAM(ADC_CH_INDEX_IA, &(channelCalibrations[ADC_CH_INDEX_IA]) );

	cal_SetLowCal  ( &(channelCalibrations[ADC_CH_INDEX_IB]), CURRENT_ZERO_RAW, 0.0f );
	cal_SetHighCal ( &(channelCalibrations[ADC_CH_INDEX_IB]), BITS, CURRENT_HIGH_CAL_VAL );
	cal_storeCalibrationInFRAM(ADC_CH_INDEX_IB, &(channelCalibrations[ADC_CH_INDEX_IB]) );

	cal_SetLowCal  ( &(channelCalibrations[ADC_CH_INDEX_IC]), CURRENT_ZERO_RAW, 0.0f );
	cal_SetHighCal ( &(channelCalibrations[ADC_CH_INDEX_IC]), BITS, CURRENT_HIGH_CAL_VAL );
	cal_storeCalibrationInFRAM(ADC_CH_INDEX_IC, &(channelCalibrations[ADC_CH_INDEX_IC]) );

	cal_SetLowCal  ( &(channelCalibrations[ADC_CH_INDEX_HWIF]), CURRENT_ZERO_RAW, 0.0f );
	cal_SetHighCal ( &(channelCalibrations[ADC_CH_INDEX_HWIF]), BITS, CURRENT_HIGH_CAL_VAL );
	cal_storeCalibrationInFRAM(ADC_CH_INDEX_HWIF, &(channelCalibrations[ADC_CH_INDEX_HWIF]) );

	cal_SetLowCal  ( &(channelCalibrations[ADC_CH_INDEX_VA]), 0, 0.0f );
	cal_SetHighCal ( &(channelCalibrations[ADC_CH_INDEX_VA]), BITS, HIGH_VOLTAGE_HIGH_CAL_VAL );
	cal_storeCalibrationInFRAM(ADC_CH_INDEX_VA, &(channelCalibrations[ADC_CH_INDEX_VA]) );

	cal_SetLowCal  ( &(channelCalibrations[ADC_CH_INDEX_VB]), 0, 0.0f );
	cal_SetHighCal ( &(channelCalibrations[ADC_CH_INDEX_VB]), BITS, HIGH_VOLTAGE_HIGH_CAL_VAL );
	cal_storeCalibrationInFRAM(ADC_CH_INDEX_VB, &(channelCalibrations[ADC_CH_INDEX_VB]) );

	cal_SetLowCal  ( &(channelCalibrations[ADC_CH_INDEX_VC]), 0, 0.0f );
	cal_SetHighCal ( &(channelCalibrations[ADC_CH_INDEX_VC]), BITS, HIGH_VOLTAGE_HIGH_CAL_VAL );
	cal_storeCalibrationInFRAM(ADC_CH_INDEX_VC, &(channelCalibrations[ADC_CH_INDEX_VC]) );

	cal_SetLowCal  ( &(channelCalibrations[ADC_CH_INDEX_VN]), 0, 0.0f );
	cal_SetHighCal ( &(channelCalibrations[ADC_CH_INDEX_VN]), BITS, HIGH_VOLTAGE_HIGH_CAL_VAL );
	cal_storeCalibrationInFRAM(ADC_CH_INDEX_VN, &(channelCalibrations[ADC_CH_INDEX_VN]) );

	cal_SetLowCal  ( &(channelCalibrations[ADC_CH_INDEX_VBUS]), 0, 0.0f );
	cal_SetHighCal ( &(channelCalibrations[ADC_CH_INDEX_VBUS]), BITS, HIGH_VOLTAGE_HIGH_CAL_VAL );
	cal_storeCalibrationInFRAM(ADC_CH_INDEX_VBUS, &(channelCalibrations[ADC_CH_INDEX_VBUS]) );

	cal_SetLowCal  ( &(channelCalibrations[ADC_CH_INDEX_VGATE]), 0, 0.0f );
	cal_SetHighCal ( &(channelCalibrations[ADC_CH_INDEX_VGATE]), BITS, VGATE_HIGH_CAL_VAL );
	cal_storeCalibrationInFRAM(ADC_CH_INDEX_VGATE, &(channelCalibrations[ADC_CH_INDEX_VGATE]) );

	// TODO: Setup temperature
}

DMA_InitTypeDef dmaInit;


/**
 * Inits the ADC.
 * @param fs - Sample frequency in Hz.
 */
void adc_Init( uint32 fs )
{
	for ( int i = 0; i < ADC_NUM_CONVERSIONS * ADC_NUM_SIM_CHANNELS; i++ )
	{
		cal_getCalibrationFromFRAM(i,&(channelCalibrations[i]));
	}

	/*********************************************************
	 * Clock Initialization
	 *********************************************************/

	// Enable the ADC clocks.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

	// Enable the timer clock.
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	// Enable the DMA clock.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	// Enable the GPIO clocks associated with the ADC pin ports.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/*********************************************************
	 * GPIO (ADC Pins) Initialization
	 *********************************************************/

	// Turn on the ADC GPIOs
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	gpio.GPIO_Mode = GPIO_Mode_AN;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &gpio);
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &gpio);
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &gpio);

	/*********************************************************
	 * Timer 2 (ADC Trigger) Initialization
	 *********************************************************/

	// Setup the timer to run at the sample frequency specified.
	TIM_DeInit(TIM2);

	TIM_TimeBaseInitTypeDef adcBase;
	TIM_TimeBaseStructInit(&adcBase);
	adcBase.TIM_ClockDivision = TIM_CKD_DIV1;
	adcBase.TIM_CounterMode = TIM_CounterMode_Up;
	adcBase.TIM_Period = (SYS_APB1_HZ * 2) / fs;
	TIM_TimeBaseInit(TIM2,&adcBase);

	TIM_OCInitTypeDef adcOC;
	TIM_OCStructInit(&adcOC);
	adcOC.TIM_OCIdleState = TIM_OCIdleState_Reset;
	adcOC.TIM_OCMode = TIM_OCMode_PWM1;
	adcOC.TIM_OCPolarity = TIM_OCPolarity_Low;
	adcOC.TIM_OutputState = TIM_OutputState_Enable;
	adcOC.TIM_Pulse = adcBase.TIM_Period / 2;
	TIM_OC2Init(TIM2,&adcOC);

	// Enable the timer.
	TIM_Cmd(TIM2,ENABLE);

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

	ADC_CommonInit(&adcComInit);

	// Init each of the three ADCs individually
	ADC_InitTypeDef adcInit;
	ADC_StructInit(&adcInit);
	adcInit.ADC_ContinuousConvMode = DISABLE;
	adcInit.ADC_DataAlign = ADC_DataAlign_Right;
	adcInit.ADC_NbrOfConversion = ADC_NUM_CONVERSIONS;
	adcInit.ADC_Resolution = ADC_Resolution_12b;
	adcInit.ADC_ScanConvMode = ENABLE;
	adcInit.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
	adcInit.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;

	// Start up the three ADC modules.
	ADC_Init(ADC1, &adcInit);
	ADC_Init(ADC2, &adcInit);
	ADC_Init(ADC3, &adcInit);

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

//	dmaTargetIndex = (dmaTargetIndex + 1) % DMA_DEPTH;
//
//	DMA_DoubleBufferModeConfig(
//			DMA2_Stream0,
//			(uint32_t)&(adcDMA[dmaTargetIndex * ADC_NUM_CONVERSIONS * ADC_NUM_SIM_CHANNELS]),
//			DMA_Memory_0);
//
//	DMA_DoubleBufferModeCmd(DMA2_Stream0,ENABLE);

	// Enable the DMA
//	DMA_Cmd(DMA2_Stream0, ENABLE);

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
	NVIC_Init(&dmaNVIC);

	// Enable the DMA
	DMA_Cmd(DMA2_Stream0, ENABLE);

	// Enable DMA
	ADC_DMACmd(ADC1, ENABLE);

	// Enable the ADCs
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);

	d1k_portal_RegisterFunction( "adcDefaultCal", adc_DefaultCalibrationCommand );
	d1k_portal_RegisterFunction( "adcHighCal"   , adc_HighCalCommand );
	d1k_portal_RegisterFunction( "adcLowCal"    , adc_LowCalCommand );
}

/**
 * Gets a reading from an ADC channel in the units defined by the calibration.
 * @param channelIndex - Index of the channel to read.
 * @param filter - Array of FIR filter coefficients.
 * @param filterLength - Length of the filter in the array.
 * @return ADC reading.
 */
float adc_GetReading ( ADC_Channel_t channelIndex, q15_t* filter, uint16 filterLength )
{
	return cal_getValue(
			  &(channelCalibrations[channelIndex]),
			  adc_GetReadingRaw( channelIndex, filter, filterLength )
			);
}

/**
 * Gets the last ADC reading for a given channel.
 * @param channelIndex - Index of the channel to get.
 * @return ADC reading.
 */
float adc_GetCachedReading( ADC_Channel_t channelIndex )
{
	return adcCache[channelIndex];
}

/**
 *  Gets a raw, filtered reading from the ADC.
 * @param channelIndex - Index of the channel to read.
 * @param filter - Array of FIR filter coefficients.
 * @param filterLength - Length of the filter in the array.
 * @return ADC reading.
 */
q15_t adc_GetReadingRaw ( ADC_Channel_t channelIndex, q15_t* filter, uint16 filterLength )
{
	sint32 result = 0;

	// Determine which DMA "slot" is the latest to be filled.  It is located two behind
	// the current target.  The current target points to the next slot to be filled.  The
	// one previous is currently being filled.  The one before that is the most recent
	// complete value.

	sint16 targetSlot = (dmaTargetIndex - 2);
	if (targetSlot < 0) { targetSlot += DMA_DEPTH; }

	sint16 offset = targetSlot * ADC_NUM_CHANNELS + channelIndex;

	if (filterLength > 0)
	{
		for (int i = 0; i < filterLength; i++)
		{
			result += (sint32) adcDMA[offset] * (sint32) filter[i] ;

			offset -= ADC_NUM_CHANNELS;

			// Wrap around correctly.
			if (offset < 0) { offset += DMA_DEPTH * ADC_NUM_CHANNELS; }
		}

		return (q15_t) (result >> 15);
	}
	else
	{
		return adcDMA[offset];
	}
}

float adc_GetMedian5Reading ( ADC_Channel_t channelIndex )
{
	return cal_getValue(
				  &(channelCalibrations[channelIndex]),
				  adc_GetMeadian5ReadingRaw( channelIndex )
				);
}


inline void swap( q15_t* a, q15_t* b )
{
	q15_t temp;

	temp = *a;
	*a = *b;
	*b = temp;
}

q15_t adc_GetMeadian5ReadingRaw ( ADC_Channel_t channelIndex )
{
	q15_t d[5];

	// Determine which DMA "slot" is the latest to be filled.  It is located two behind
	// the current target.  The current target points to the next slot to be filled.  The
	// one previous is currently being filled.  The one before that is the most recent
	// complete value.

	sint16 targetSlot = (dmaTargetIndex - 2);
	if (targetSlot < 0) { targetSlot += DMA_DEPTH; }

	sint16 offset = targetSlot * ADC_NUM_CHANNELS + channelIndex;

	for (int i = 0; i < 5; i++)
	{
		d[i] = (sint32) adcDMA[offset] ;

		offset -= ADC_NUM_CHANNELS;

		// Wrap around correctly.
		if (offset < 0) { offset += DMA_DEPTH * ADC_NUM_CHANNELS; }
	}

	if ( d[0] < d[1] )
	{
		swap(&d[0],&d[1]);
	}

	if ( d[2] < d[3] )
	{
		swap(&d[2],&d[3]);
	}

	if ( d[0] < d[2] )
	{
		swap(&d[0],&d[2]);
		swap(&d[1],&d[3]);
	}

	if ( d[1] < d[4] )
	{
		swap(&d[1],&d[4]);
	}

	if ( d[1] > d[2] )
	{
		if ( d[2] > d[4] )
		{
			return d[2];
		}
		else
		{
			return d[4];
		}
	}
	else
	{
		if ( d[1] > d[3] )
		{
			return d[1];
		}
		else
		{
			return d[3];
		}
	}
}


/****************************************************************************
 * Private Functions
 ***************************************************************************/

static void adc_DefaultCalibrationCommand( int argc, char** argv )
{
	adc_setDefaultCalibrationValues();
}

static void adc_HighCalCommand( int argc, char** argv )
{
	if (argc != 3)
	{
		printf("\nadcHighCal needs 2 arguments: channelIndex currentValue.\n");
		return;
	}

	long channelIndex = strtol(argv[1],NULL,0);
	float highCal;

	sscanf(argv[2],"%f",&highCal);

	if (channelIndex < 0 && channelIndex >= ADC_NUM_CHANNELS)
	{
		printf("\nChannel index out of range.\n");
	}

	uint16 currentValRaw = adc_GetReadingRaw( (ADC_Channel_t) channelIndex, FILTER_HAMMING_4KHZ, FILTER_HAMMING_4KHZ_LENGTH );

	cal_SetHighCal( &(channelCalibrations[channelIndex]), currentValRaw, highCal );
	cal_storeCalibrationInFRAM( channelIndex, &(channelCalibrations[channelIndex]) );

	printf("\n\nHigh Calibration complete.  Raw value: %d. Value: %f. \n", currentValRaw, highCal);
}

static void adc_LowCalCommand( int argc, char** argv )
{
	if (argc != 3)
	{
		printf("\nadcLowCal needs 2 arguments: channelIndex currentValue.\n");
		return;
	}

	long channelIndex = strtol(argv[1],NULL,0);
	float lowCal;

	sscanf(argv[2],"%f",&lowCal);

	if (channelIndex < 0 && channelIndex >= ADC_NUM_CHANNELS)
	{
		printf("\nChannel index out of range.\n");
	}

	uint16 currentValRaw = adc_GetReadingRaw( (ADC_Channel_t) channelIndex, FILTER_HAMMING_4KHZ, FILTER_HAMMING_4KHZ_LENGTH);

	cal_SetLowCal( &(channelCalibrations[channelIndex]), currentValRaw, lowCal );
	cal_storeCalibrationInFRAM( channelIndex, &(channelCalibrations[channelIndex]) );

	printf( "\n\nLow Calibration complete.  Raw value: %d. Value: %f. \n", currentValRaw, lowCal );
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
