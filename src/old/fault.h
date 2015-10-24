/********************************************************************
fault.h

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

#ifndef FAULT_H
#define FAULT_H

/****************************************************************************
 * Includes
 ***************************************************************************/

#include "stm32f4xx.h"
#include "d1k.h"

/****************************************************************************
 * Public Function Prototypes
 ***************************************************************************/

void fault_Init( void );

void fault_SetGateEnable( FunctionalState f );

FunctionalState fault_GetGateEnable   ( void );

bool fault_HardwareFaultExists        ( void );
bool fault_HardwareCurrentFaultExists ( void );
bool fault_HardwareVoltageFaultExists ( void );

#endif /* FAULT_H_ */
