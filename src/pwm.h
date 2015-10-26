/********************************************************************
pwm.h

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

#ifndef PWM_H
#define PWM_H


#include "stm32f4xx.h"
#include "stdbool.h"

typedef void (*ControlLoopFxn)( void );

void pwm_init ( uint32_t frequency, ControlLoopFxn control_loop );

void pwm_set_output_enable ( FunctionalState a, FunctionalState b, FunctionalState c );

void pwm_set_duty_cycle ( float a, float b, float c );

void pwm_estop ( void );

#endif /* PWM_H_ */
