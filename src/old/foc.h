/********************************************************************
foc.h

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

#ifndef FOC_H_
#define FOC_H_

/****************************************************************************
 * Typedefs
 ***************************************************************************/

typedef struct {
	float Ia;
	float Ib;
	float Ic;
	float Ialpha;
	float Ibeta;
	float Id;
	float Iq;
	float theta;
	float sinTheta;
	float cosTheta;
	float Vd;
	float Vq;
	float Valpha;
	float Vbeta;
	float Va;
	float Vb;
	float Vc;
	float Vbus;
	float V1;
	float V2;
	float V3;
	float Vk;
} focControl_t;

/****************************************************************************
 * Public Functions
 ***************************************************************************/

void FOC_Clarke    ( focControl_t *foc );
void FOC_SinCos    ( focControl_t *foc );
void FOC_Park      ( focControl_t *foc );
void FOC_InvPark   ( focControl_t *foc );
void FOC_InvClarke ( focControl_t *foc );
void FOC_SVM       ( focControl_t *foc );


#endif /* FOC_H_ */
