/********************************************************************
fullCAN.h

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

#include "fullCAN.h"
#include "d1k_can.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#include "system_cfg.h"
#include "control_loop.h"
#include "can_handle.h"
#include "smarts_leds.h"
#include "d1k_led.h"
#include "diagnostics.h"

/****************************************************************************
 * Packet Handlers
 ***************************************************************************/

static void fullCANGenericHandler( CanRxMsg * packet );

CANRXEntry_t fullCANGeneric =
{
		.mask = FULLCAN_CONTROLLER_MASK,
		.idAfterMask = 0,
		.callback = fullCANGenericHandler
};

/****************************************************************************
 * Public Functions
 ***************************************************************************/

/**
 * Initializes the uCAN network.
 * @param interfaceType - current interface type configuration.
 * @param baudRate - baud rate.
 */
void fullCAN_Init( uint32 baudRate, uint8 controllerId )
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// fullCAN interface is on CAN2, PB12 (RX), and PB13 (TX).
	// Configure the micro to support this.  Init the network in D1K.

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE); // turn on CAN1 clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // turn on GPIOs related to CAN

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2); // set pins to CAN mode
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	d1k_CAN_Init(CAN2, baudRate);

	fullCANGeneric.idAfterMask = controllerId;

	d1k_CAN_RegisterHandler(CAN2, &fullCANGeneric);

	// Enable the GPIO pin that controls termination.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); // turn on GPIOs related to CAN
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void fullCAN_SetTerminationState( FunctionalState enabled )
{
	if (enabled == ENABLE)
	{
		GPIO_SetBits(GPIOE,GPIO_Pin_6);
	}
	else
	{
		GPIO_ResetBits(GPIOE,GPIO_Pin_6);
	}
}

/**
 * Packet handler for fullCAN network.
 */
void fullCANGenericHandler( CanRxMsg * packet )
{
	#define FULLCAN (CAN2)

	 if ( (packet->StdId & FULLCAN_PACKETID_MASK) == FULLCAN_IDBASE_DRIVEMODE )
	 {
		 GenericU8Handler( FULLCAN, packet, GetDrivemodeU8, SetDrivemodeU8 );

		 if ( packet->Data[0] == DRIVEMODE_DISABLED )
		 {
			 diag_ClearError(ERROR_CURRENT_SOFTWARE_FAULT);
		 }
	 }

	else if ( (packet->StdId & FULLCAN_PACKETID_MASK) == FULLCAN_IDBASE_IQREF)      { GenericF32Handler( FULLCAN, packet, GetIQRef, SetIQRef ); }
	else if ( (packet->StdId & FULLCAN_PACKETID_MASK) == FULLCAN_IDBASE_IDREF)      { GenericF32Handler( FULLCAN, packet, GetIDRef, SetIDRef ); }

	else if ( (packet->StdId & FULLCAN_PACKETID_MASK) == FULLCAN_IDBASE_CALIBRATE)  { DetermineMotorOffset( ); }

	else if ( (packet->StdId & FULLCAN_PACKETID_MASK) == FULLCAN_IDBASE_VDMAX)      { GenericF32Handler( FULLCAN, packet, GetVDMax, SetVDMax ); }
	else if ( (packet->StdId & FULLCAN_PACKETID_MASK) == FULLCAN_IDBASE_VDMIN)      { GenericF32Handler( FULLCAN, packet, GetVDMin, SetVDMin ); }
	else if ( (packet->StdId & FULLCAN_PACKETID_MASK) == FULLCAN_IDBASE_VQMAX)      { GenericF32Handler( FULLCAN, packet, GetVQMax, SetVQMax ); }
	else if ( (packet->StdId & FULLCAN_PACKETID_MASK) == FULLCAN_IDBASE_VQMIN)      { GenericF32Handler( FULLCAN, packet, GetVQMin, SetVQMin ); }

	else if ( (packet->StdId & FULLCAN_PACKETID_MASK) == FULLCAN_IDBASE_KI)         { GenericF32Handler( FULLCAN, packet, GetKI, SetKI ); }
	else if ( (packet->StdId & FULLCAN_PACKETID_MASK) == FULLCAN_IDBASE_KP)         { GenericF32Handler( FULLCAN, packet, GetKP, SetKP ); }
	else if ( (packet->StdId & FULLCAN_PACKETID_MASK) == FULLCAN_IDBASE_ROLOFFPER)  { GenericF32Handler( FULLCAN, packet, GetRolloffAveragingPeriod,  SetRolloffAveragingPeriod  ); }

}

void fullCAN_SendPacket( CanTxMsg * packet , fullCANPacketId_t id)
{
	packet->StdId = id + fullCANGeneric.idAfterMask;
	d1k_CAN_SendPacket(CAN2, packet);
}

uint32 fullCAN_GetControllerId ( void )
{
	return fullCANGeneric.idAfterMask;
}
