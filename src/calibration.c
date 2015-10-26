/********************************************************************
calibration.c

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

#include "calibration.h"
#include "d1k_fram.h"
#include "fram_map.h"

/****************************************************************************
 * Private Function Prototypes
 ***************************************************************************/

static void cal_UpdateScale ( calibration_t* cal );

/****************************************************************************
 * Public Functions
 ***************************************************************************/

void cal_SetLowCal ( calibration_t* cal, uint16 raw, float val )
{
	cal->lowCalRaw = raw;
	cal->lowCalVal = val;

	cal_UpdateScale(cal);
}

void cal_SetHighCal( calibration_t* cal, uint16 raw, float val )
{
	cal->highCalRaw = raw;
	cal->highCalVal = val;

	cal_UpdateScale(cal);
}

float cal_getValue( calibration_t* cal, uint16 rawValue )
{
	return cal->gain * rawValue + cal->offset;
}

void cal_getCalibrationFromFRAM( uint16 calibrationId, calibration_t* dst )
{
	d1k_fram_Read(
			FRAM_BASE_ADDRESS_CALIBRATION + sizeof(calibration_t) * calibrationId,
			dst,
			sizeof(calibration_t));
}

void cal_storeCalibrationInFRAM( uint16 calibrationId, calibration_t* src )
{
	d1k_fram_Write(
			FRAM_BASE_ADDRESS_CALIBRATION + sizeof(calibration_t) * calibrationId,
			src,
			sizeof(calibration_t));
}

/****************************************************************************
 * Private Functions
 ***************************************************************************/

static void cal_UpdateScale ( calibration_t* cal )
{
	// Find the slope of the lines
	cal->gain = (cal->highCalVal - cal->lowCalVal)
			/ (cal->highCalRaw - cal->lowCalRaw);


	// Use the high cal point to find the intercept

	cal->offset = cal->highCalVal - ( cal->gain * cal->highCalRaw );
}


