/********************************************************************
fram_map.h

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

#ifndef FRAMMAP_H
#define FRAMMAP_H

#define FRAM_BASE_ADDRESS_CALIBRATION	(0x1000)
#define FRAM_BASE_ADDRESS_CONFIGURATION (0x2000)

#define FRAM_ADDRESS_CONTROLLER_ID             ( FRAM_BASE_ADDRESS_CONFIGURATION )
#define FRAM_ADDRESS_MOTOR_OFFSET  			   ( FRAM_ADDRESS_CONTROLLER_ID            + sizeof(uint8) )
#define FRAM_ADDRESS_MOTOR_POSITION_INVERSION  ( FRAM_ADDRESS_MOTOR_OFFSET             + sizeof(float) )
#define FRAM_ADDRESS_MOTOR_PHASING_SWAPPED     ( FRAM_ADDRESS_MOTOR_POSITION_INVERSION + sizeof(bool) )
#define FRAM_ADDRESS_KP                        ( FRAM_ADDRESS_MOTOR_PHASING_SWAPPED    + sizeof(bool) )
#define FRAM_ADDRESS_KI                        ( FRAM_ADDRESS_KP                       + sizeof(float) )
#define FRAM_ADDRESS_AVERAGINGPER              ( FRAM_ADDRESS_KI                       + sizeof(float) )

#endif /* FRAMMAP_H_ */
