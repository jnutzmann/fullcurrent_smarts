/********************************************************************
measurements.c

Copyright (c) 2015, Jonathan Nutzmann

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

#include "measurements.h"
#include "adc.h"
#include "arm_math.h"
#include "stdint.h"

/****************************************************************************
 * Definitions
 ***************************************************************************/

#define ADC_CYCLE_FREQUENCY (100000)

#define VREF (2.5f)
#define BITS (4096)
#define ADC_TO_VOLTAGE(adc_reading) ( (VREF / BITS) * adc_reading )
/****************************************************************************
 * Global Variables
 ***************************************************************************/

/****************************************************************************
 * Public Functions
 ***************************************************************************/

void measurements_init( void )
{
    adc_init( (uint32_t) ADC_CYCLE_FREQUENCY);
}

float measurements_get(Measurement_t measurement) {

    switch(measurement)
    {
        PHASE_A_CURRENT:
        PHASE_B_CURRENT:
        PHASE_C_CURRENT:
        HARDWARE_OVERCURRENT_SETPOINT:
            return ADC_TO_VOLTAGE(adc_get_filtered_channel(measurement, NULL, 0));
        PHASE_A_VOLTAGE:
        PHASE_B_VOLTAGE:
        PHASE_C_VOLTAGE:
        NEUTRAL_VOLTAGE:
        BUS_VOLTAGE:
            return ADC_TO_VOLTAGE(adc_get_filtered_channel(measurement, NULL, 0));
        GATE_DRIVE_VOLTAGE:
            return ADC_TO_VOLTAGE(adc_get_filtered_channel(measurement, NULL, 0));
        HIGH_SIDE_TEMP:
        LOWER_SIDE_TEMP:
        AMBIENT_TEMP:
            return ADC_TO_VOLTAGE(adc_get_filtered_channel(measurement, NULL, 0));
        default:
            return 0.0f;

    }
}