/********************************************************************
diagnostics.c

Copyright (c)  2015, Jonathan Nutzmann

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option)  any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
********************************************************************/

#ifndef FULLCURRENT_SMARTS_DIAGNOSTICS_H
#define FULLCURRENT_SMARTS_DIAGNOSTICS_H

/****************************************************************************
 * Includes
 ***************************************************************************/

#include "led.h"

/****************************************************************************
 * Typedefs
 ***************************************************************************/

typedef enum {
    LED_WARNING = LED_COUNT_CORE,
    LED_MOT_ERROR,
    LED_V_ERROR,
    LED_I_ERROR,
    LED_HEARTBEAT,
    LED_COUNT
} AppLEDPurpose_t;

typedef enum
{
    ERROR_MOTOR_HALLS = 0,
    ERROR_MOTOR_NOT_PRESENT,
    ERROR_MOTOR_OVERTEMP,
    ERROR_CURRENT_HARDWARE_FAULT,
    ERROR_CURRENT_SOFTWARE_FAULT,
    ERROR_CURRENT_NONE,
    ERROR_CURRENT_IMBALANCE,
    ERROR_VOLTAGE_HIGH_PHASE_A,
    ERROR_VOLTAGE_HIGH_PHASE_B,
    ERROR_VOLTAGE_HIGH_PHASE_C,
    ERROR_VOLTAGE_HIGH_BUS,
    ERROR_VOLTAGE_HIGH_15,
    ERROR_VOLTAGE_LOW_15,
    ERROR_OVERTEMP_A,
    ERROR_OVERTEMP_AN,
    ERROR_OVERTEMP_B,
    ERROR_OVERTEMP_BN,
    ERROR_OVERTEMP_C,
    ERROR_OVERTEMP_CN,
    ERROR_OVERTEMP_AMB,
    ERROR_OVERTEMP_HS,
    ERROR_COUNT,
    ERROR_INVALID = ERROR_COUNT
} ErrorEnum_t;

typedef enum
{
    WARNING_MOTOR_HALLS,
    WARNING_MOTOR_TEMP,
    WARNING_BUS_VOLTAGE,
    WARNING_HIGH_15,
    WARNING_LOW_15,
    WARNING_TEMP_A,
    WARNING_TEMP_AN,
    WARNING_TEMP_B,
    WARNING_TEMP_BN,
    WARNING_TEMP_C,
    WARNING_TEMP_CN,
    WARNING_TEMP_AMB,
    WARNING_TEMP_HS,
    WARNING_COUNT,
    WARNING_INVALID = WARNING_COUNT
} WarningEnum_t;

/****************************************************************************
 * Public Prototypes
 ***************************************************************************/

void diag_init( void ) ;

void error_throw( ErrorEnum_t error ) ;
void error_clear( ErrorEnum_t error ) ;
bool error_get_state( ErrorEnum_t error ) ;
uint32_t error_get_bitmask( void ) ;

void warning_throw( WarningEnum_t warning ) ;
void warning_clear( WarningEnum_t warning ) ;
bool warning_get_state( WarningEnum_t warning ) ;
uint32_t warning_get_bitmask( void ) ;

#endif //FULLCURRENT_SMARTS_DIAGNOSTICS_H
