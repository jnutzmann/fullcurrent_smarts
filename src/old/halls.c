/********************************************************************
halls.c

Copyright (c) 2014, Jonathan Nutzmann

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
#include "d1k.h"
#include "halls.h"
#include "smarts_leds.h"
#include "d1k_led.h"
#include "arm_math.h"
#include "system_cfg.h"
#include "control_loop.h"
#include "diagnostics.h"


/****************************************************************************
 * Definitions
 ***************************************************************************/

#define MIN_SPEED_MPH	(0.125f)
#define MOTOR_POLES		(16)
#define TIRE_DIAMETER_M	(0.5f)
#define TIMER_BITS		(16)

#define MPH_TO_MPS		(0.44704f)

#define MIN_SPEED_MPS	(MIN_SPEED_MPH * MPH_TO_MPS)
#define MIN_SPEED_RPS	(MIN_SPEED_MPS / (PI * TIRE_DIAMETER_M))
#define MIN_SPEED_ERPS	(MIN_SPEED_RPS * MOTOR_POLES/2)

#define MAX_TIME_BETWEEN_TICKS	(1.0f / (MIN_SPEED_ERPS * 6))
#define MAX_TIMER_COUNT         ( (float)(((uint32)1)<<TIMER_BITS) )
#define TIMER_INPUT_FREQ		( (float) (SYS_APB2_HZ * 2) )
#define TIMER_INPUT_ROLLOVER_PERIOD	( MAX_TIMER_COUNT / TIMER_INPUT_FREQ )
#define HALL_TIMER_PRESCALER 	((uint16) ( 1 + MAX_TIME_BETWEEN_TICKS / TIMER_INPUT_ROLLOVER_PERIOD ))
#define HALL_TIMER_FREQUENCY    ( TIMER_INPUT_FREQ / HALL_TIMER_PRESCALER )

/****************************************************************************
 * Global Variables
 ***************************************************************************/

static sint32 motorSegmentLookup[8] = {
	0,   // UVW = 000
	1,   // UVW = 001
   -1,   // UVW = 010
	2,   // UVW = 011
	5,   // UVW = 100
   -1,   // UVW = 101
	4,   // UVW = 110
	3    // UVW = 111
};


static motorDirection_t motorDirection = MOTOR_DIRECTION_UNKNOWN;
static sint32 lastValidHallSegment = -1;

static volatile uint32 forwardCounts = 0;
static volatile uint32 backwardCounts = 0;
static volatile bool   hallTimerValid = false;
static volatile uint32 electricalPeriod = 0;
static volatile uint32 lastHallTickTime = 0;

/****************************************************************************
 * Public Functions
 ***************************************************************************/

void halls_Init( void )
{
	// Turn on the Timer and Port clocks.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	// Set the function of the GPIO pins to Timer8 (used for hall sense)
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8);

	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &gpio);

	TIM_TimeBaseInitTypeDef tb;
	TIM_TimeBaseStructInit(&tb);
	tb.TIM_ClockDivision = TIM_CKD_DIV4;
	tb.TIM_CounterMode = TIM_CounterMode_Up;
	tb.TIM_Prescaler = HALL_TIMER_PRESCALER;
	tb.TIM_Period = 0xFFFF;
	TIM_TimeBaseInit(TIM8, &tb);

	// This enables the XOR on TI1
	TIM_SelectHallSensor(TIM8, ENABLE);

	TIM_ICInitTypeDef ic;
	TIM_ICStructInit(&ic);

	ic.TIM_Channel = TIM_Channel_1;
	ic.TIM_ICFilter = 3;
	ic.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInit(TIM8, &ic);

	TIM_Cmd(TIM8,ENABLE);

	TIM_ITConfig(TIM8,TIM_IT_CC1,ENABLE);		// This interrupts every time a hall ticks.
	TIM_ITConfig(TIM8,TIM_IT_Update,ENABLE);	// This interrupts when the timer overflows.

	lastValidHallSegment = halls_GetMotorSegment();

	NVIC_InitTypeDef nvicInit;
	nvicInit.NVIC_IRQChannel = TIM8_CC_IRQn;
	nvicInit.NVIC_IRQChannelCmd = ENABLE;
	nvicInit.NVIC_IRQChannelPreemptionPriority = 6;
	nvicInit.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init( &nvicInit );
	nvicInit.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
	NVIC_Init( &nvicInit );
}

uint32 halls_GetUVW( void )
{
	return (uint32) ((GPIO_ReadInputData(GPIOC) & ( GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 )) >> 6);
}

sint32 halls_GetMotorSegment( void )
{
	return motorSegmentLookup[halls_GetUVW()];
}

float halls_GetMotorPosition( void )
{
	if (lastValidHallSegment < 0)
	{
		return 0;
	}

	return 60.0f * lastValidHallSegment;
}

float halls_GetMotorMechanicalVelocity ( void )
{
	if ( electricalPeriod > 0 )
	{
		return (HALL_TIMER_FREQUENCY * (MOTOR_POLES / 2)) / ((float)electricalPeriod);
	}

	return 0;
}

float halls_GetMotorPositionWithEstimation( void )
{
	if (lastValidHallSegment < 0)
	{
		return 0;
	}

	float delta = 0;

	// If we are moving, calculate the estimated delta
	if (electricalPeriod > 0)
	{
		uint32 timer = TIM_GetCounter(TIM8);

		delta = ((float)(timer-lastHallTickTime)) / ((float)electricalPeriod) * 360.0f;

		if (delta > 60.0f)
		{
			delta = 60.0f;
		}
	}

	if ( motorDirection == MOTOR_DIRECTION_FORWARD
			|| motorDirection == MOTOR_DIRECTION_UNKNOWN)
	{
		return 60.0f * lastValidHallSegment + delta;
	}
	else if (motorDirection == MOTOR_DIRECTION_REVERSE)
	{
		return 60.0f * ((lastValidHallSegment+1)%6) - delta;
	}
	else
	{
		return 0.0f;
	}
}

/****************************************************************************
 * Interrupt Vectors
 ***************************************************************************/

void TIM8_UP_TIM13_IRQHandler ( void )
{
	if ( TIM_GetITStatus(TIM8,TIM_IT_Update) )
	{
		// The timer has rolled over.

		hallTimerValid = 0;
		electricalPeriod = 0;

		TIM_ClearITPendingBit(TIM8,TIM_IT_Update);
	}
}

void TIM8_CC_IRQHandler ( void )
{
	if ( TIM_GetITStatus(TIM8,TIM_IT_CC1) )
	{
		// The motor has changed hall positions.

		if (electricalPeriod == 0)
		{
			electricalPeriod = 0xFFFF;
		}

		motorDirection_t prevDir = motorDirection;

		lastHallTickTime = TIM_GetCapture1(TIM8);

		sint32 newSeg = halls_GetMotorSegment();

		if (lastValidHallSegment < 0)
		{
			lastValidHallSegment = newSeg;
			forwardCounts = 0;
			backwardCounts = 0;
		}
		else if ( (lastValidHallSegment+1)%6 == newSeg )
		{
			// We are moving forward.
			motorDirection = MOTOR_DIRECTION_FORWARD;
			lastValidHallSegment = newSeg;

			forwardCounts++;
			backwardCounts = 0;
		}
		else if ( lastValidHallSegment == (newSeg+1)%6 )
		{
			// We are moving in reverse.
			motorDirection = MOTOR_DIRECTION_REVERSE;
			lastValidHallSegment = newSeg;

			backwardCounts++;
			forwardCounts = 0;
		}
		else if ( lastValidHallSegment != newSeg)
		{
			// We missed a hall transition or have an error in the hall sensor.
			lastValidHallSegment = -1;
			motorDirection = MOTOR_DIRECTION_UNKNOWN;
			diag_ThrowError(ERROR_MOTOR_HALLS);
		}


		if (motorDirection != prevDir)
		{
			electricalPeriod = 0;
		}

		if (forwardCounts >= 6)
		{
			if (hallTimerValid)
			{
				electricalPeriod = lastHallTickTime;
			}
			else
			{
				electricalPeriod = 0xFFFF;
				hallTimerValid = true;
			}

			TIM_SetCounter(TIM8,0);
			forwardCounts = 0;
			lastHallTickTime = 0;
		}

		if (backwardCounts >= 6)
		{
			if (hallTimerValid)
			{
				electricalPeriod = lastHallTickTime;
			}
			else
			{
				electricalPeriod = 0xFFFF;
				hallTimerValid = true;
			}

			TIM_SetCounter(TIM8,0);
			backwardCounts = 0;
			lastHallTickTime = 0;
		}

		TIM_ClearITPendingBit(TIM8,TIM_IT_CC1);
	}


}


