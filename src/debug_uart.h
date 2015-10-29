/********************************************************************
debug_uart.h

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

#ifndef DEBUG_UART_H
#define DEBUG_UART_H

/****************************************************************************
 * Public Prototypes
 ***************************************************************************/

#include "stm32f4xx.h"
#include "stdbool.h"
#include "stdint.h"
#include "stddef.h"

#define MAX_DEBUG_DATA_SIZE   (15)

typedef struct {
    uint16_t address;
    uint16_t length;
    bool request_to_receive;
    uint8_t data[MAX_DEBUG_DATA_SIZE];
} DebugUartPayload_t;

void debug_uart_init(uint32_t baud);
bool debug_uart_send_packet(DebugUartPayload_t * payload);


// TODO: how do I handle receive? - Push into skylab handler.
// does packetization get handled here?

#endif /* UART_H */
