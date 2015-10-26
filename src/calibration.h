/********************************************************************
calibration.h

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

#ifndef CALIBRATION_H
#define CALIBRATION_H

/****************************************************************************
 * Includes
 ***************************************************************************/

#include "d1k.h"

/****************************************************************************
 * Typedefs
 ***************************************************************************/

typedef struct {
	uint16 highCalRaw;
	float  highCalVal;
	uint16 lowCalRaw;
	float  lowCalVal;
	float  offset;
	float  gain;
} calibration_t;

/****************************************************************************
 * Public Prototypes
 ***************************************************************************/

void  cal_SetLowCal  ( calibration_t* cal, uint16 raw, float val );
void  cal_SetHighCal ( calibration_t* cal, uint16 raw, float val );
float cal_getValue   ( calibration_t* cal, uint16 rawValue );

void cal_getCalibrationFromFRAM ( uint16 calibrationId, calibration_t* dst );
void cal_storeCalibrationInFRAM ( uint16 calibrationId, calibration_t* src );


#endif /* CALIBRATION_H_ */
