/********************************************************************
pi.h

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

#ifndef PI_H_
#define PI_H_

typedef struct {
	float reference;
	float measured;
	float accumulator;
	float dt;
	float kp;
	float ki;
	float outMax;
	float outMin;
} pi_t;

void  PI_Init    ( pi_t *pi );
float PI_Control ( pi_t *pi );

#endif /* PI_H_ */
