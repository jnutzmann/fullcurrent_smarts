/********************************************************************
mCAN.c

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
#include "d1k_can.h"
#include "system_cfg.h"
#include "smarts_leds.h"

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_can.h"

#include "control_loop.h"
#include "can_handle.h"
#include "string.h"

/****************************************************************************
 * Packet Handlers
 ***************************************************************************/
// TODO: Decide which packets to listen for.  Skylab?

static void mPolePositionCallback( CanRxMsg * packet );

CANRXEntry_t mPolePositionEntry =
{
		.mask = 0, //xFFFF,
		.idAfterMask = 0, //x4f0,
		.callback = mPolePositionCallback
};

/****************************************************************************
 * Global Variables
 ***************************************************************************/

static float motorPosition = 0;
static float ppFieldStrengeth = 0;

static float motorInductance = 0.0005f;
static float motorResistance = 0.020f;
static float motorKE = 0.5f;

static float motorPolePairs = 10.0f;

/****************************************************************************
 * Public Functions
 ***************************************************************************/

/**
 * Initializes the uCAN network.
 * @param interfaceType - current interface type configuration.
 * @param baudRate - baud rate.
 */
void mCAN_Init( uint32 baudRate )
{

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // turn on GPIOs related to CAN
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); // turn on GPIOs related to CAN

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1); // set pins to CAN mode
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	d1k_CAN_Init(CAN1, baudRate);

	d1k_CAN_RegisterHandler(CAN1, &mPolePositionEntry);
}

float mCAN_GetMotorPosition( void )
{
	return motorPosition;
}

float mCAN_GetMotorInductance( void )
{
	return motorInductance;
}

float mCAN_GetMotorResistance( void )
{
	return motorResistance;
}

float mCAN_GetMotorKE( void )
{
	return motorKE;
}

float mCAN_GetMotorPolePairs( void )
{
	return motorPolePairs;
}

float mCAN_GetPolePositionFieldStrength( void )
{
	return ppFieldStrengeth;
}

/****************************************************************************
 * Private Functions
 ***************************************************************************/

static void mPolePositionCallback( CanRxMsg * packet )
{
	if ( packet->StdId == 1264 )
	{
		memcpy( &motorPosition, &(packet->Data[0]), sizeof(float) );
		return;
	}
	else if ( packet->StdId == 1265 )
	{
		memcpy( &ppFieldStrengeth, &(packet->Data[0]), sizeof(float) );
	}
}
