/********************************************************************
configuration.c

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

#include "mCAN.h"
#include "configuration.h"
#include "fram_map.h"
#include "d1k_fram.h"
#include "stdio.h"
#include "fullCAN.h"
#include "d1k_portal.h"

/****************************************************************************
 * Static Definitions
 ***************************************************************************/

static void InitFramI2C ( void );

static void controllerIDCommand ( int argc, char** argv );

/****************************************************************************
 * Public Functions
 ***************************************************************************/

/**
 * Initializes the configuration module and retrieves information from flash.
 */
void cfg_Init( void )
{
	InitFramI2C( );

	d1k_portal_RegisterFunction("cid",controllerIDCommand);
}

float cfg_GetProperty( configProperty_t prop )
{
	return 0.0f;
}

bool cfg_SetProperty( configProperty_t prop, float val )
{
	return false;
}

static void InitFramI2C( void )
{
	// Connect I2C1 pins to AF
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1); // SDA

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; // we are going to use PB6 and PB7
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// init GPIOB

	d1k_fram_Init( I2C1, 0x7FFF, 0 );
}

uint8 cfg_GetControllerID( void )
{
	uint8 controllerID;

	d1k_fram_Read(FRAM_ADDRESS_CONTROLLER_ID, &controllerID, 1);

	return controllerID;
}

static void controllerIDCommand ( int argc, char** argv )
{
	if ( argc == 1 )
	{
		printf("\nController ID: %u\n",(uint16)fullCAN_GetControllerId());
	}
	else if ( argc == 2 )
	{
		uint8 id = (uint8) strtoul(argv[1],NULL,0);

		d1k_fram_Write(FRAM_ADDRESS_CONTROLLER_ID, &id, 1);

		printf("\nController ID written.  Restart to take effect.\n");
	}
}


