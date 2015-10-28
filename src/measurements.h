/********************************************************************
measurements.h

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

#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

/****************************************************************************
 * Includes
 ***************************************************************************/

#include "adc.h"

/****************************************************************************
 * Defines
 ***************************************************************************/

/****************************************************************************
 * Typedef
 ***************************************************************************/

typedef enum
{
    PHASE_A_CURRENT = ADC_CH_INDEX_IA,
    PHASE_B_CURRENT = ADC_CH_INDEX_IB,
    PHASE_C_CURRENT = ADC_CH_INDEX_IC,
    PHASE_A_VOLTAGE = ADC_CH_INDEX_VA,
    PHASE_B_VOLTAGE = ADC_CH_INDEX_VB,
    PHASE_C_VOLTAGE = ADC_CH_INDEX_VC,
    NEUTRAL_VOLTAGE = ADC_CH_INDEX_VN,
    GATE_DRIVE_VOLTAGE = ADC_CH_INDEX_VGATE,
    BUS_VOLTAGE = ADC_CH_INDEX_VBUS,
    HIGH_SIDE_TEMP = ADC_CH_INDEX_T,
    LOWER_SIDE_TEMP = ADC_CH_INDEX_TN,
    AMBIENT_TEMP = ADC_CH_INDEX_TM1,
    HARDWARE_OVERCURRENT_SETPOINT = ADC_CH_INDEX_HWIF,
} Measurement_t;

/****************************************************************************
 * Public Function Prototypes
 ***************************************************************************/

void measurements_init( void );

#endif /* ADC_H_ */
