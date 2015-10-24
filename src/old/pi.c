/********************************************************************
pi.c

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


#include "pi.h"
#include "math_limits.h"

void  PI_Init    ( pi_t *pi )
{
	pi->accumulator = 0;
}

float PI_Control ( pi_t *pi )
{
	float error = pi->reference - pi->measured;

	pi->accumulator = limitf32( (error * pi->ki * pi->dt + pi->accumulator), pi->outMax, pi->outMin);

	return limitf32( error * pi->kp + pi->accumulator, pi->outMax, pi->outMin);
}
