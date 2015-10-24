/********************************************************************
pole_position.c

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

#include "pole_position.h"
#include "control_loop.h"
#include "mCAN.h"
#include <math.h>
#include "d1k_led.h"
#include "smarts_leds.h"

static volatile float mechanicalVelocity = 0;

void PolePosition_Init( void )
{
	mCAN_Init(1000000);
}

float PolePosition_GetElectricalPosition( void )
{
	return fmodf( PolePosition_GetMechanicalPosition() * mCAN_GetMotorPolePairs() , 360.0f );
}

float PolePosition_GetMechanicalVelocity( void )
{
	return mechanicalVelocity;
}

float PolePosition_GetElectricalVelocity( void )
{
	return mechanicalVelocity / mCAN_GetMotorPolePairs();
}

#define POS_SAMP_FREQ_FOR_SPEED_HZ  ( 10000 )
#define POS_SAMPLE_WAIT_COUNT       ( CONTROL_LOOP_FREQUENCY_HZ / POS_SAMP_FREQ_FOR_SPEED_HZ )
#define VELOCITY_UPDATE_FREQ_HZ     ( 1000 )
#define POS_SAMPLE_ACCUM_COUNT      ( POS_SAMP_FREQ_FOR_SPEED_HZ / VELOCITY_UPDATE_FREQ_HZ )
#define MAX_MECHANICAL_SPEED_RPM	( 10000.0f )
#define MAX_ANGLE_DELTA_DEG			( MAX_MECHANICAL_SPEED_RPM / 60.0f * 360.0f / POS_SAMP_FREQ_FOR_SPEED_HZ )

void PolePosition_UpdateVelocityCalculation( void )
{
	static uint32 waitCount = 0;
	static uint32 accumCount = 0;
	static float posDeltaAccum = 0;
	static float lastPos = 0.0f;

	float pos = mCAN_GetMotorPosition();

	waitCount++;

	if ( waitCount >= POS_SAMPLE_WAIT_COUNT )
	{
		waitCount = 0;

		float delta = pos - lastPos;
		lastPos = pos;

		if ( delta > 0 )
		{
			if ( delta > MAX_ANGLE_DELTA_DEG )
			{
				delta -= 360.0f;
			}
		}
		else
		{
			if ( delta < -MAX_ANGLE_DELTA_DEG )
			{
				delta += 360.0f;
			}
		}

		if ( fabsf(delta) > MAX_ANGLE_DELTA_DEG )
		{
			// We have a bad angle, ignore it.
			delta = 0;
		}

		posDeltaAccum += delta;

		accumCount++;

		if ( accumCount >= POS_SAMPLE_ACCUM_COUNT )
		{
			accumCount = 0;

			float newVelocity = posDeltaAccum
					* ((float) VELOCITY_UPDATE_FREQ_HZ) * (60.0f/360.0f) ;

			mechanicalVelocity = (mechanicalVelocity * 9.0f + newVelocity)*(1.0f/10.0f);

			posDeltaAccum = 0;
		}

	}
}

float PolePosition_GetMechanicalPosition( void )
{
	return mCAN_GetMotorPosition();
}
