/********************************************************************
skylab.h

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

#ifndef SKYLAB_H
#define SKYLAB_H


#include "stdbool.h"
#include "stm32f4xx_can.h"


typedef void (*OrbitSerialSend)(uint8_t* data, size_t len);
typedef void (*OrbitCANSend)(CAN )

bool orbit_init()

bool orbit_fullcurrent_analog1(float* ia, float* ib, bool serial_only);
bool orbit_fullcurrent_analog1(void* data, bool serial_only);

bool deorbit_packet_handler();

#endif /* SKYLAB_H */

// PACKET SCHEME

// ADDRESS: 11 bits
// RTR: 1 bit
// DLC: 4 bits
// DATA: 0-15 bytes (UART), 0-8 bytes (CAN)

// UART CAN BE LONGER! (32 bytes)

// CAN:  SAAA AAAA AAAA REXL LLL DDDD DDDD [x8] CCCC CCCC CCCC CCCS

// UART: SSSS SSSS AAAA AAAA AAAR LLLL DDDD DDDD [x15] CCCC CCCC
// (UART DRIVER WILL HANDLE START BIT AND CRC, as well as escape char)