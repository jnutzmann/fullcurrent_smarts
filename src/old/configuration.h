/********************************************************************
configuration.h

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


#ifndef CONFIGURATION_H
#define CONFIGURATION_H

void cfg_Init( void );

typedef enum {
	CFG_NUM_PROPERTIES
} configProperty_t;

float cfg_GetProperty( configProperty_t prop );
bool  cfg_SetProperty( configProperty_t prop, float val );

uint8 cfg_GetControllerID( void );

#endif /* CONFIGURATION_H_ */
