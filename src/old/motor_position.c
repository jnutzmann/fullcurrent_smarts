/********************************************************************
motor_position.c

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

#include "FreeRTOS.h"
#include "motor_position.h"
#include "halls.h"
#include "pole_position.h"
#include "d1k_fram.h"
#include "fram_map.h"

typedef float (*MotorFeedbackFxn)( void );
float NullMotorFeedbackFxn( void ) { return 0.0f; }

static MotorFeedbackFxn electricalPositionFxn = NullMotorFeedbackFxn;
static MotorFeedbackFxn mechanicalPositionFxn = NullMotorFeedbackFxn;

static MotorFeedbackFxn electricalVelocityFxn = NullMotorFeedbackFxn;
static MotorFeedbackFxn mechanicalVelocityFxn = NullMotorFeedbackFxn;

static float electricalPositionOffset = 0;
static bool  invertPosition = false;

void MotorPosition_Init( feedbackType_t fbty )
{
	d1k_fram_Read(FRAM_ADDRESS_MOTOR_OFFSET,&electricalPositionOffset,sizeof(electricalPositionOffset));
	d1k_fram_Read(FRAM_ADDRESS_MOTOR_POSITION_INVERSION,&invertPosition,sizeof(invertPosition));

	switch ( fbty )
	{
		case FEEDBACK_POLE_POSITION:
			electricalPositionFxn = PolePosition_GetElectricalPosition;
			mechanicalPositionFxn = PolePosition_GetMechanicalPosition;
			electricalVelocityFxn = PolePosition_GetElectricalVelocity;
			mechanicalVelocityFxn = PolePosition_GetMechanicalVelocity;
			break;
		case FEEDBACK_HALL_SENSOR:
			halls_Init();
			break;
		case FEEDBACK_HALL_SENSOR_WITH_ESTIMATION:
			halls_Init();
			electricalPositionFxn = halls_GetMotorPositionWithEstimation;
			mechanicalVelocityFxn = halls_GetMotorMechanicalVelocity;
			break;
		default:
			break;
	}
}

void MotorPosition_SetElectricalPositionOffset( float offset )
{
	electricalPositionOffset = offset;

	d1k_fram_Write(FRAM_ADDRESS_MOTOR_OFFSET,&electricalPositionOffset,sizeof(electricalPositionOffset));
}

void MotorPosition_SetPositionInversion ( bool invert )
{
	invertPosition = invert;

	d1k_fram_Write(FRAM_ADDRESS_MOTOR_POSITION_INVERSION,&invertPosition,sizeof(invertPosition));
}

bool MotorPosition_GetPositionInversion ( void )
{
	return invertPosition;
}

float MotorPosition_GetElectricalPositionOffset( void )
{
	return electricalPositionOffset;
}

float MotorPosition_GetRawElectricalPosition( void )
{
	return electricalPositionFxn();
}

float MotorPosition_GetElectricalPosition( void )
{
	return invertPosition ? WrapAngle(360.0f - electricalPositionFxn() + electricalPositionOffset) : WrapAngle(electricalPositionFxn() + electricalPositionOffset);
}
float MotorPosition_GetMechanicalPosition( void )
{
	return mechanicalPositionFxn();
}

float MotorPosition_GetElectricalVelocity( void )
{
	return invertPosition ? -electricalVelocityFxn() : electricalVelocityFxn();
}
float MotorPosition_GetMechanicalVelocity( void )
{
	return  invertPosition ? -mechanicalVelocityFxn() : mechanicalVelocityFxn();
}

float WrapAngle( float angle )
{
	while (angle > 180.0f)  { angle -= 360.0f; }
	while (angle < -180.0f) { angle += 360.0f; }

	return angle;
}

