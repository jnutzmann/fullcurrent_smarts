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

#ifndef FULLCAN_H
#define FULLCAN_H

#include "d1k.h"

/****************************************************************************
 *  Definitions
 ***************************************************************************/

#define FULLCAN_BAUDRATE 		(1000000)
#define	FULLCAN_CNST_OFFSET		(0x100)

#define FULLCAN_CONTROLLER_MASK (0xF)
#define FULLCAN_PACKETID_MASK	(0x7F0)

typedef enum
{
	// ===== COMMANDS ===== //
	FULLCAN_IDBASE_DRIVEMODE	= (0x130+FULLCAN_CNST_OFFSET),
	
	FULLCAN_IDBASE_IQREF		= (0x110+FULLCAN_CNST_OFFSET),
	FULLCAN_IDBASE_IDREF		= (0x120+FULLCAN_CNST_OFFSET),
	
	FULLCAN_IDBASE_CALIBRATE    = (0x130+FULLCAN_CNST_OFFSET),

	// ===== CONFIGURATION ===== //
	FULLCAN_IDBASE_VDMAX		= (0x230+FULLCAN_CNST_OFFSET),
	FULLCAN_IDBASE_VDMIN		= (0x240+FULLCAN_CNST_OFFSET),
	FULLCAN_IDBASE_VQMAX		= (0x250+FULLCAN_CNST_OFFSET),
	FULLCAN_IDBASE_VQMIN		= (0x260+FULLCAN_CNST_OFFSET),
	
	FULLCAN_IDBASE_KP			= (0x270+FULLCAN_CNST_OFFSET),
	FULLCAN_IDBASE_KI			= (0x280+FULLCAN_CNST_OFFSET),
	FULLCAN_IDBASE_ROLOFFPER    = (0x2E0+FULLCAN_CNST_OFFSET),

	// ===== FEEDBACK ===== //
	FULLCAN_IDBASE_VOLTAGE_AB	= (0x400+FULLCAN_CNST_OFFSET),
	FULLCAN_IDBASE_VOLTAGE_C	= (0x410+FULLCAN_CNST_OFFSET),
	
	FULLCAN_IDBASE_CURRENT_AB	= (0x420+FULLCAN_CNST_OFFSET),
	FULLCAN_IDBASE_CURRENT_C	= (0x430+FULLCAN_CNST_OFFSET),
	
	FULLCAN_IDBASE_CURRENT_DQ	= (0x440+FULLCAN_CNST_OFFSET),
	FULLCAN_IDBASE_VOLTAGE_DQ	= (0x450+FULLCAN_CNST_OFFSET),
	
	FULLCAN_IDBASE_VOLTAGE_12	= (0x460+FULLCAN_CNST_OFFSET),
	FULLCAN_IDBASE_VOLTAGE_3	= (0x470+FULLCAN_CNST_OFFSET),
	
	FULLCAN_IDBASE_POSITION		= (0x480+FULLCAN_CNST_OFFSET),
	
	FULLCAN_IDBASE_VBUS_VGATE	= (0x490+FULLCAN_CNST_OFFSET),
	
	FULLCAN_IDBASE_EVENTS       = (0x4A0+FULLCAN_CNST_OFFSET),

	FULLCAN_IDBASE_CALIBRATION  = (0x4B0+FULLCAN_CNST_OFFSET),
	
	FULLCAN_IDBASE_TEMPERATURE  = (0x4C0+FULLCAN_CNST_OFFSET),
	
	// ===== STDIO ===== //
	FULLCAN_IDBASE_STDIO_TX		= (0x5E0+FULLCAN_CNST_OFFSET),
	FULLCAN_IDBASE_STDIO_RX		= (0x5F0+FULLCAN_CNST_OFFSET)
	
} fullCANPacketId_t;

/****************************************************************************
 * Public Function Prototypes
 ***************************************************************************/

void   fullCAN_SendPacket          ( CanTxMsg * packet , fullCANPacketId_t id );
void   fullCAN_Init                ( uint32 baudRate, uint8 controllerId );
uint32 fullCAN_GetControllerId     ( void );
void   fullCAN_SetTerminationState ( FunctionalState enabled );


#endif /* GLOBAL_NETWORK_H */
