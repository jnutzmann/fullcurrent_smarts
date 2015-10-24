/********************************************************************
motor_position.h

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
#ifndef MOTOR_POSITION_H
#define MOTOR_POSITION_H

#include "d1k.h"

typedef enum {
	FEEDBACK_POLE_POSITION = 0,
	FEEDBACK_HALL_SENSOR,
	FEEDBACK_HALL_SENSOR_WITH_ESTIMATION,
	FEEDBACK_NUM_TYPES
} feedbackType_t;

void  MotorPosition_Init( feedbackType_t fbty );

void  MotorPosition_SetElectricalPositionOffset( float offset );

float MotorPosition_GetElectricalPositionOffset( void );
float MotorPosition_GetRawElectricalPosition( void );

float MotorPosition_GetElectricalPosition( void );
float MotorPosition_GetMechanicalPosition( void );
float MotorPosition_GetElectricalVelocity( void );
float MotorPosition_GetMechanicalVelocity( void );

void MotorPosition_SetPositionInversion ( bool invert );
bool MotorPosition_GetPositionInversion ( void );

void MotorPosition_MotorCalibrationCheck( void );


float WrapAngle( float angle );

#endif /* MOTOR_POSITION_H_ */
