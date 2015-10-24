/********************************************************************
pole_position.h

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

#ifndef POLE_POSITION_H_
#define POLE_POSITION_H_

#include "d1k.h"

void PolePosition_Init( void );

float PolePosition_GetElectricalVelocity( void );
float PolePosition_GetElectricalPosition( void );

float PolePosition_GetMechanicalVelocity( void );
float PolePosition_GetMechanicalPosition( void );

void PolePosition_UpdateVelocityCalculation( void );

#endif /* POLE_POSITION_H_ */
