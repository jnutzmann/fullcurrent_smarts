/********************************************************************
diagnostics.c

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

/****************************************************************************
 * Public Prototypes
 ***************************************************************************/

void diag_init();

#endif //FULLCURRENT_SMARTS_DIAGNOSTICS_H
