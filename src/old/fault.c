/********************************************************************
fault.c

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

#include "fault.h"

/****************************************************************************
 * Definitions
 ***************************************************************************/

#define HW_FAULT GPIO_Pin_15
#define HW_FAULT_GPIO GPIOE

/****************************************************************************
 * Public Functions
 ***************************************************************************/

/**
 * Initializes the fault handling system.
 */
void fault_Init( void )
{
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);

	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;

	// Input Fault Lines
	gpio.GPIO_Mode = GPIO_Mode_IN;

	gpio.GPIO_Pin = HW_FAULT;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	GPIO_PinAFConfig(HW_FAULT_GPIO,GPIO_PinSource15,GPIO_AF_TIM1);
	GPIO_Init(HW_FAULT_GPIO, &gpio);
}

/**
 * Checks if a hardware fault is present.
 * @return TRUE if present, else FALSE.
 */
bool fault_HardwareFaultExists( void )
{
	return ! (bool) GPIO_ReadInputDataBit(HW_FAULT_GPIO, HW_FAULT);
}
