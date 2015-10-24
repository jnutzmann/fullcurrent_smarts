/********************************************************************
control_loop.h

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

#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include "stm32f4xx.h"
#include "d1k.h"

#define PWM_FREQUENCY_HZ			( 20000 )
#define CONTROL_LOOP_FREQUENCY_HZ	( PWM_FREQUENCY_HZ )

/****************************************************************************
 * Typedefs
 ***************************************************************************/

typedef enum {
	DRIVEMODE_FORWARD = 0,
	DRIVEMODE_REVERSE,
	DRIVEMODE_DISABLED,
	DRIVEMODE_INVALID
} drivemode_t;

/****************************************************************************
 * Public Function Prototypes
 ***************************************************************************/

void ControlLoopInit ( void );

/************ STATE PARAMETERS ************/

void SetDrivemode ( drivemode_t d );
drivemode_t GetDrivemode ( void );

void SetDrivemodeU8 ( uint8 d );
uint8 GetDrivemodeU8 ( void );

/************ Limit Parameters ************/

void  SetVDMax ( float vd );
float GetVDMax ( void );
void  SetVDMin ( float vd );
float GetVDMin ( void );

void  SetVQMax ( float vq );
float GetVQMax ( void );
void  SetVQMin ( float vq );
float GetVQMin ( void );

void  SetIQMax ( float iq );
float GetIQMax ( void );
void  SetIQMin ( float iq );
float GetIQMin ( void );

void  SetIDMax ( float id );
float GetIDMax ( void );

void  SetKP    ( float k );
void  SetKI    ( float k );

float GetKP    ( void );
float GetKI    ( void );

void DetermineMotorOffset ( void );

/************ Reference Parameters ************/

void  SetIQRef ( float iq );
float GetIQRef ( void );

void  SetIDRef ( float id );
float GetIDRef ( void );

float GetRolloffAveragingPeriod ( void );
void  SetRolloffAveragingPeriod ( float period );

#endif /* CONTROLLOOP_H_ */
