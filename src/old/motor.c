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

#include "motor.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "d1k.h"
#include "halls.h"
#include "diagnostics.h"

void motor_Init( void )
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);
	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_Pin = GPIO_Pin_9;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &gpio);
}

bool motor_CheckIfPresent( void )
{
	if ( false ) //GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9) )
	{
		diag_ThrowError(ERROR_MOTOR_NOT_PRESENT);
		return false;
	}

	return true;
}
