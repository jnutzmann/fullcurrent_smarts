/********************************************************************
main.c

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

#include "stm32f4xx.h"
#include "misc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "adc.h"
#include "diagnostics.h"
#include "measurements.h"
#include "debug_uart.h"

int main(void) {

    SystemInit();

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    diag_init();

    measurements_init();

    debug_uart_init(115200);



    vTaskStartScheduler();
}

