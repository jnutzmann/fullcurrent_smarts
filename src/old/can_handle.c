/********************************************************************
can_handle.c

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

#include "can_handle.h"
#include "stm32f4xx_can.h"
#include "d1k_can.h"
#include "string.h"

void GenericF32Handler( CAN_TypeDef * canModule, CanRxMsg * packet, f32_GetterFxn getter, f32_SetterFxn setter )
{
	if ( packet->RTR == CAN_RTR_Remote )
	{
		CanTxMsg reply;
		reply.StdId = packet->StdId;
		reply.RTR = CAN_RTR_Data;
		reply.DLC = sizeof(float);

		float data = getter( );
		memcpy( reply.Data, &data, sizeof(float) );

		d1k_CAN_SendPacket_ISR(canModule,&reply);
	}
	else
	{
		float data;
		memcpy(&data, packet->Data, sizeof(float));

		setter(data);
	}
}

void GenericU8Handler( CAN_TypeDef * canModule, CanRxMsg * packet, u8_GetterFxn getter, u8_SetterFxn setter )
{
	if ( packet->RTR == CAN_RTR_Remote )
	{
		CanTxMsg reply;
		reply.StdId = packet->StdId;
		reply.RTR = CAN_RTR_Data;
		reply.DLC = 1;

		reply.Data[0] = getter( );

		d1k_CAN_SendPacket_ISR(canModule,&reply);
	}
	else
	{
		setter( packet->Data[0] );
	}
}

void GenericFSHandler( CAN_TypeDef * canModule, CanRxMsg * packet, FS_GetterFxn getter, FS_SetterFxn setter )
{
	if ( packet->RTR == CAN_RTR_Remote )
	{
	CanTxMsg reply;
		reply.StdId = packet->StdId;
		reply.RTR = CAN_RTR_Data;
		reply.DLC = 1;

		reply.Data[0] = (uint8) getter( );

		d1k_CAN_SendPacket_ISR(canModule, &reply);
	}
	else
	{
		setter( (FunctionalState) packet->Data[0] );
	}
}

