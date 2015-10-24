/********************************************************************
foc.c

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

#include "foc.h"
#include "math.h"
#include "arm_math.h"

/****************************************************************************
 * Definitions
 ***************************************************************************/

#define max(a,b)	(((a) > (b)) ? (a) : (b))
#define min(a,b)	(((a) < (b)) ? (a) : (b))

#define max3(a,b,c) (max((a), max((b),(c))))
#define min3(a,b,c) (min((a), min((b),(c))))

/****************************************************************************
 * Public Functions
 ***************************************************************************/

/**
 * This does the a,b,c -> alpha,beta transform.
 * @param foc - FOC data structure.
 * [IN]  Ia
 * [IN]  Ib
 * [OUT] Ialpha
 * [OUT] Ibeta
 */
void FOC_Clarke ( focControl_t *foc )
{
	foc->Ialpha = foc->Ia;
	foc->Ibeta  = ((1.0f/sqrtf(3.0f)) * foc->Ia) + ((2.0f/sqrtf(3.0f)) * foc->Ib);
}

/**
 * This finds the sin and cos of the rotor angle.
 * @param foc - FOC data structure.
 * [IN]  theta
 * [OUT] sinTheta
 * [OUT] cosTheta
 */
void FOC_SinCos ( focControl_t *foc )
{
	arm_sin_cos_f32(foc->theta,&(foc->sinTheta),&(foc->cosTheta));
}

/**
 * Does the Ialpha,Ibeta->Id,Iq transform.
 * @param foc - FOC data structure.
 * [IN]  Ialpha
 * [IN]  Ibeta
 * [IN]  sinTheta
 * [IN]  cosTheta
 * [OUT] Id
 * [OUT] Iq
 */
void FOC_Park ( focControl_t *foc )
{
	foc->Id =  foc->Ialpha * foc->cosTheta + foc->Ibeta * foc->sinTheta;
	foc->Iq = -foc->Ialpha * foc->sinTheta + foc->Ibeta * foc->cosTheta;
}

/**
 * Does the Vd,Vq->Valpha,Vbeta transform.
 * @param foc - FOC data structure.
 * [IN]  Vd
 * [IN]  Vq
 * [IN]  sinTheta
 * [IN]  cosTheta
 * [OUT] Valpha
 * [OUT] Vbeta
 */
void FOC_InvPark ( focControl_t *foc )
{
	foc->Valpha = foc->Vd * foc->cosTheta - foc->Vq * foc->sinTheta;
	foc->Vbeta =  foc->Vd * foc->sinTheta + foc->Vq * foc->cosTheta;
}

/**
 * Does the Valpha->Vbeta transform.
 * @param foc - FOC data structure.
 * [IN]  Valpha
 * [IN]  Vbeta
 * [OUT] Va
 * [OUT] Vb
 * [OUT] Vc
 */
void FOC_InvClarke ( focControl_t *foc )
{
	foc->Va = foc->Valpha;
	foc->Vb = -foc->Valpha * 0.5f + sqrtf(3.0f)/2.0f * foc->Vbeta;
	foc->Vc = -foc->Valpha * 0.5f - sqrtf(3.0f)/2.0f * foc->Vbeta;
}

/**
 * Uses space vector modulation to determine the PWM vectors to apply to the motor.
 * @param foc - FOC data structure.
 * [IN]  Va
 * [IN]  Vb
 * [IN]  Vc
 * [OUT] Va
 * [OUT] Vb
 * [OUT] Vc
 * [OUT] V1
 * [OUT] V2
 * [OUT] V3
 * [OUT] Vk
 */
void FOC_SVM( focControl_t *foc )
{
	//foc->Va *= 0.5f;
	//foc->Vb *= 0.5f;
	//foc->Vc *= 0.5f;

	foc->Vk = ( ( max3(foc->Va, foc->Vb, foc->Vc) + min3(foc->Va, foc->Vb, foc->Vc) ) * 0.5f );

	float halfVbus = 0.5f * foc->Vbus;

	foc->V1 = (foc->Va - foc->Vk) + halfVbus;
	foc->V2 = (foc->Vb - foc->Vk) + halfVbus;
	foc->V3 = (foc->Vc - foc->Vk) + halfVbus;
}
