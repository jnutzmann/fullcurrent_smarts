/********************************************************************
pwm.c

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

#include "pwm.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "system_cfg.h"
#include "FreeRTOS.h"
#include "task.h"
#include "misc.h"


#include "stdio.h"

#include "func/math_limits.h"

#define PWM_AN      ( GPIO_Pin_8 )
#define PWM_AN_SRC  ( GPIO_PinSource8 )
#define PWM_A       ( GPIO_Pin_9 )
#define PWM_A_SRC   ( GPIO_PinSource9 )
#define PWM_BN      ( GPIO_Pin_10 )
#define PWM_BN_SRC  ( GPIO_PinSource10 )
#define PWM_B       ( GPIO_Pin_11 )
#define PWM_B_SRC   ( GPIO_PinSource11 )
#define PWM_CN      ( GPIO_Pin_12 )
#define PWM_CN_SRC  ( GPIO_PinSource12 )
#define PWM_C       ( GPIO_Pin_13 )
#define PWM_C_SRC   ( GPIO_PinSource13 )

#define PWM_GPIO ( GPIOE )
#define PWM_GPIO_CLK ( RCC_AHB1Periph_GPIOE )

#define MINIMUM_DUTY_CYCLE ( 0.0f )
#define MAXIMUM_DUTY_CYCLE ( 0.96f )

static uint32_t pwm_period = 0;
static ControlLoopFxn control_loop_fxn = NULL;

static void pwm_init_timer_1 ( uint32_t frequency );

void pwm_init( uint32_t frequency, ControlLoopFxn control_loop)
{
	pwm_init_timer_1( frequency );
	control_loop_fxn = control_loop;
}

/**
 * Initializes the PWM output of the controller.
 * @param frequency - Frequency (in Hz) of the switching.
 */
static void pwm_init_timer_1 ( uint32_t frequency )
{
	// TODO: add dead time as an argument

	RCC_AHB1PeriphClockCmd( PWM_GPIO_CLK, ENABLE );
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1, ENABLE );

	// Set up the GPIO pins to be associated with Timer1
	GPIO_PinAFConfig( PWM_GPIO, PWM_AN_SRC, GPIO_AF_TIM1 );
	GPIO_PinAFConfig( PWM_GPIO, PWM_A_SRC,  GPIO_AF_TIM1 );
	GPIO_PinAFConfig( PWM_GPIO, PWM_BN_SRC, GPIO_AF_TIM1 );
	GPIO_PinAFConfig( PWM_GPIO, PWM_B_SRC,  GPIO_AF_TIM1 );
	GPIO_PinAFConfig( PWM_GPIO, PWM_CN_SRC, GPIO_AF_TIM1 );
	GPIO_PinAFConfig( PWM_GPIO, PWM_C_SRC,  GPIO_AF_TIM1 );

	// Init the GPIO pins
	GPIO_InitTypeDef gpio;
	GPIO_StructInit( &gpio );
	gpio.GPIO_Pin = PWM_AN | PWM_A | PWM_BN | PWM_B | PWM_CN | PWM_C;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init( PWM_GPIO, &gpio );

	// To calculate the period
	pwm_period = (SYS_APB2_HZ / frequency );

	// DeInit the timer before we init to make sure everything is defaulted.
	TIM_DeInit( TIM1 );

	TIM_TimeBaseInitTypeDef tb;
	TIM_TimeBaseStructInit( &tb );
	tb.TIM_ClockDivision = TIM_CKD_DIV1;
	tb.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	tb.TIM_Period = pwm_period;
	TIM_TimeBaseInit(TIM1, &tb);

	TIM_OCInitTypeDef oc;
	TIM_OCStructInit(&oc);
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
	oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	oc.TIM_OCPolarity = TIM_OCPolarity_High;
	oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
	oc.TIM_OCMode = TIM_OCMode_PWM1;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_OutputNState = TIM_OutputNState_Enable;

	TIM_OC1Init(TIM1, &oc);
	TIM_OC2Init(TIM1, &oc);
	TIM_OC3Init(TIM1, &oc);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_CCPreloadControl(TIM1, DISABLE);

	TIM_BDTRInitTypeDef bdtr;
	TIM_BDTRStructInit(&bdtr);
	bdtr.TIM_DeadTime = 40; // ???ns
	bdtr.TIM_Break = TIM_Break_Disable; // TODO: enable hardware faults
	bdtr.TIM_BreakPolarity = TIM_BreakPolarity_Low;
	bdtr.TIM_OSSIState = TIM_OSSIState_Enable;
	bdtr.TIM_OSSRState = TIM_OSSRState_Enable;

	// bdtr.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

	TIM_BDTRConfig(TIM1, &bdtr);

	TIM_Cmd(TIM1, ENABLE);

	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);

	NVIC_InitTypeDef nvicInit;
	nvicInit.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	nvicInit.NVIC_IRQChannelCmd = ENABLE;
	nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
	nvicInit.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init( &nvicInit );

	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


/**
 * Disables all channels on the motor controller.
 */
void pwm_estop(void)
{
	TIM_CtrlPWMOutputs(TIM1, DISABLE);
}

/**
 * Configures whether the PWM channel is active.  If enabled, duty cycle is set by
 * pwm_SetDutyCycles.  If disabled, all gates will be low and the FETs will be floating.
 * @param a - Whether channel A is enabled.
 * @param b - Whether channel B is enabled.
 * @param c - Whether channel C is enabled.
 */
void pwm_set_output_enable( FunctionalState a, FunctionalState b, FunctionalState c )
{
	if ( a == ENABLE )
	{
		TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Enable);
	}
	else
	{
		TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Disable);
		TIM_SetCompare1(TIM1, 0);
	}

	if ( b == ENABLE)
	{
		TIM_CCxNCmd(TIM1,TIM_Channel_2,TIM_CCxN_Enable);
	}
	else
	{
		TIM_CCxNCmd(TIM1,TIM_Channel_2,TIM_CCxN_Disable);
		TIM_SetCompare2(TIM1, 0);
	}

	if ( c == ENABLE )
	{
		TIM_CCxNCmd(TIM1,TIM_Channel_3,TIM_CCxN_Enable);
	}
	else
	{
		TIM_CCxNCmd(TIM1,TIM_Channel_3,TIM_CCxN_Disable);
		TIM_SetCompare3(TIM1, 0);
	}
}

/**
 * Sets the PWM duty cycles output of the motor controller.  Values between
 * 0 and 1 are mapped to 0 to 100% duty cycle.
 * @param a - Phase A duty cycle.
 * @param b - Phase B duty cycle.
 * @param c - Phase C duty cycle.
 */
void pwm_set_duty_cycle(float a, float b, float c)
{
	a = limitf32( a, MAXIMUM_DUTY_CYCLE, MINIMUM_DUTY_CYCLE);
	b = limitf32( b, MAXIMUM_DUTY_CYCLE, MINIMUM_DUTY_CYCLE);
	c = limitf32( c, MAXIMUM_DUTY_CYCLE, MINIMUM_DUTY_CYCLE);

	TIM_GenerateEvent( TIM1, TIM_EventSource_COM );

	TIM_SetCompare1( TIM1, (uint32_t)(a * pwm_period) );
	TIM_SetCompare2( TIM1, (uint32_t)(b * pwm_period) );
	TIM_SetCompare3( TIM1, (uint32_t)(c * pwm_period) );
}

/**
 * PWM Interrupt handler.
 */
void TIM1_UP_TIM10_IRQHandler ( void )
{
	if ( TIM_GetITStatus(TIM1,TIM_IT_Update) )
	{
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
		control_loop_fxn();
	}
}
