/********************************************************************
fan.c

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

#include "fan.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "system_cfg.h"

#define FAN_PWM_FREQUENCY (20000)
#define FAN_PWM_PERIOD	  (SYS_APB1_HZ/FAN_PWM_FREQUENCY)


void fan_Init( void )
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,  ENABLE);

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15, GPIO_AF_TIM4);

	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Pin = GPIO_Pin_15;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &gpio);

	TIM_TimeBaseInitTypeDef tb;
	TIM_TimeBaseStructInit(&tb);
	tb.TIM_ClockDivision = TIM_CKD_DIV2;
	tb.TIM_CounterMode = TIM_CounterMode_Up;
	tb.TIM_Prescaler = 1;
	tb.TIM_Period = FAN_PWM_PERIOD;
	TIM_TimeBaseInit(TIM4, &tb);

	TIM_OCInitTypeDef oc;
	TIM_OCStructInit(&oc);
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
	oc.TIM_OCPolarity = TIM_OCPolarity_High;
	oc.TIM_OCMode = TIM_OCMode_PWM1;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC4Init(TIM4, &oc);

	TIM_Cmd(TIM4, ENABLE);
	TIM_CtrlPWMOutputs(TIM4, ENABLE);
}

void fan_SetDutyCycle( float d )
{
	TIM_SetCompare4(TIM4, (uint32_t)(d * FAN_PWM_PERIOD));
}



