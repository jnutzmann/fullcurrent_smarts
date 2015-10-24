/********************************************************************
halls.h

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

#ifndef HALLS_H
#define HALLS_H

#include "d1k.h"

typedef enum {
	MOTOR_DIRECTION_FORWARD,
	MOTOR_DIRECTION_REVERSE,
	MOTOR_DIRECTION_UNKNOWN
} motorDirection_t;

/****************************************************************************
 * Public Functions
 ***************************************************************************/

uint32 halls_GetUVW            ( void );
sint32 halls_GetMotorSegment   ( void );
void   halls_Init              ( void );
float  halls_GetMotorPosition  ( void );
float  halls_GetMotorPositionWithEstimation( void );
float  halls_GetMotorMechanicalVelocity ( void );


#endif /* HALLS_H_ */
