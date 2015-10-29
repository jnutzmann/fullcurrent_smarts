/********************************************************************
diagnostics.c

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

#include "diagnostics.h"

#include "FreeRTOS.h"
#include "task.h"
#include "debug_uart.h"

/****************************************************************************
 * Definitions
 ***************************************************************************/

#define DIAGNOSTICS_TASK_FREQUENCY (1)

#define BITMASK(e,a) (a << e)
/****************************************************************************
 * Global Variables
 ***************************************************************************/

xTaskHandle diag_task_handle;

static uint32_t errors_present = 0;
static uint32_t warnings_present = 0;

/****************************************************************************
 * Prototypes
 ***************************************************************************/

void diag_task( void * pvParameters );
static void init_leds ( void );

/****************************************************************************
 * Public Functions
 ***************************************************************************/

/**
 * Initializes the diagnostics handler.
 */
void diag_init( void )
{
    init_leds( );
    xTaskCreate( diag_task, "DIAGNOS", 1024, NULL, 3, &diag_task_handle );
}

void error_throw( ErrorEnum_t error )
{
    errors_present |= (uint32_t) (0x1 << error);
}

void error_clear( ErrorEnum_t error )
{
    errors_present &= ~((uint32_t) (0x1 << error));
}

bool error_get_state( ErrorEnum_t error )
{
    return errors_present & (0x1 << error);
}

uint32_t error_get_bitmask( void )
{
    return errors_present;
}

void warning_throw( WarningEnum_t warning )
{
    warnings_present |= (uint32_t) (0x1 << warning);
}

void diag_warning_clear( WarningEnum_t warning )
{
    warnings_present &= ~((uint32_t) (0x1 << warning));
}

bool diag_GetWarningState( WarningEnum_t warning )
{
    return warnings_present & (0x1 << warning);
}

uint32_t diag_GetWarningBitmask( void )
{
    return warnings_present;
}

/****************************************************************************
 * Private Functions
 ***************************************************************************/

void diag_task( void * pvParameters )
{
    portTickType xLastWakeTime = xTaskGetTickCount();

    while (1)
    {

        DebugUartPayload_t p;

        p.address = 0x64;
        p.length = 1;
        p.request_to_receive = false;
        p.data[0] = 0x63;

        debug_uart_send_packet(&p);

//        if ( warnings_present )
//        {
//            led_on( LED_WARNING );
//        }
//        else
//        {
//            led_off( LED_WARNING );
//        }


        if (errors_present & (
             BITMASK(ERROR_MOTOR_HALLS,1)
             | BITMASK(ERROR_MOTOR_NOT_PRESENT,1)
             | BITMASK(ERROR_MOTOR_OVERTEMP,1)))
        {
            led_on( LED_MOT_ERROR );
        }
        else
        {
            led_off( LED_MOT_ERROR );
        }

        if (errors_present & (
                BITMASK(ERROR_CURRENT_HARDWARE_FAULT,1)
                | BITMASK(ERROR_CURRENT_SOFTWARE_FAULT,1)
                | BITMASK(ERROR_CURRENT_NONE,1)
                | BITMASK(ERROR_CURRENT_IMBALANCE,1)))
        {
            led_on( LED_I_ERROR );
        }
        else
        {
            led_off( LED_I_ERROR );
        }

        if (errors_present & (
                BITMASK(ERROR_VOLTAGE_HIGH_PHASE_A,1)
                | BITMASK(ERROR_VOLTAGE_HIGH_PHASE_B,1)
                | BITMASK(ERROR_VOLTAGE_HIGH_PHASE_C,1)
                | BITMASK(ERROR_VOLTAGE_HIGH_BUS,1)
                | BITMASK(ERROR_VOLTAGE_HIGH_15,1)
                | BITMASK(ERROR_VOLTAGE_LOW_15,1)))
        {
            led_on( LED_V_ERROR );
        }
        else
        {
            led_off( LED_V_ERROR );
        }

        if (errors_present & (
                BITMASK(ERROR_OVERTEMP_A,1)
                | BITMASK(ERROR_OVERTEMP_AN,1)
                | BITMASK(ERROR_OVERTEMP_B,1)
                | BITMASK(ERROR_OVERTEMP_BN,1)
                | BITMASK(ERROR_OVERTEMP_C,1)
                | BITMASK(ERROR_OVERTEMP_CN,1)
                | BITMASK(ERROR_OVERTEMP_AMB,1)
                | BITMASK(ERROR_OVERTEMP_HS,1)))
        {
            led_on( LED_ERROR );
        }
        else
        {
            led_off( LED_ERROR );
        }
    
        vTaskDelayUntil( &xLastWakeTime, configTICK_RATE_HZ/DIAGNOSTICS_TASK_FREQUENCY );
    }
}

static void init_leds ( void )
{
    LEDInitStruct_t led_init_struct;

    led_init_struct.GPIOx = GPIOD;
    led_init_struct.off_time = 0;
    led_init_struct.on_time = 0;
    led_init_struct.GPIO_Clock = RCC_AHB1Periph_GPIOD;
    led_init_struct.GPIO_Pin = GPIO_Pin_7;
    led_init(LED_CAN, &led_init_struct);

    led_init_struct.GPIO_Pin = GPIO_Pin_2;
    led_init(LED_ERROR, &led_init_struct);

    led_init_struct.GPIO_Pin = GPIO_Pin_6;
    led_init(LED_WARNING, &led_init_struct);

    led_init_struct.GPIO_Pin = GPIO_Pin_5;
    led_init(LED_MOT_ERROR, &led_init_struct);

    led_init_struct.GPIO_Pin = GPIO_Pin_4;
    led_init(LED_V_ERROR, &led_init_struct);

    led_init_struct.GPIO_Pin = GPIO_Pin_3;
    led_init(LED_I_ERROR, &led_init_struct);

    led_init_struct.off_time = 500;
    led_init_struct.on_time = 500;

    led_init_struct.GPIOx = GPIOB;
    led_init_struct.GPIO_Clock = RCC_AHB1Periph_GPIOB;
    led_init_struct.GPIO_Pin = GPIO_Pin_5;
    led_init(LED_HEARTBEAT, &led_init_struct);
}

