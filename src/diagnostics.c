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
#include "string.h"


/****************************************************************************
 * Definitions
 ***************************************************************************/

#define DIAGNOSTICS_TASK_FREQUENCY	(5)

/****************************************************************************
 * Global Variables
 ***************************************************************************/

xTaskHandle diag_task_handle;

static uint32_t errorsPresent = 0;
static uint32_t warningsPresent = 0;

/****************************************************************************
 * Prototypes
 ***************************************************************************/

void diag_task( void * pvParameters );

static void init_leds();

/****************************************************************************
 * Public Functions
 ***************************************************************************/

/**
 * Initializes the diagnostics handler.
 */
void diag_init( void )
{
    init_leds();
    
   // xTaskCreate(diag_task, "DIAGNOS", 1024, NULL, 3, &diag_task_handle);
}


//bool diag_GetHardwareOvercurrent( void )
//{
//    return !GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1);
//}

//void diag_ThrowError( ErrorEnum_t error )
//{
//    errorsPresent |= (uint32) (0x1 << error);
//}
//
//void diag_ClearError( ErrorEnum_t error )
//{
//    errorsPresent &= ~((uint32) (0x1 << error));
//}
//
//bool diag_GetErrorState( ErrorEnum_t error )
//{
//    return errorsPresent & (0x1 << error);
//}
//
//uint32 diag_GetErrorBitmask ( void )
//{
//    return errorsPresent;
//}
//
//void diag_ThrowWarning( WarningEnum_t warning )
//{
//    warningsPresent |= (uint32) (0x1 << warning);
//}
//
//void diag_ClearWarning( WarningEnum_t warning )
//{
//    warningsPresent &= ~((uint32) (0x1 << warning));
//}
//
//bool diag_GetWarningState( WarningEnum_t warning )
//{
//    return warningsPresent & (0x1 << warning);
//}
//
//uint32 diag_GetWarningBitmask( void )
//{
//    return warningsPresent;
//}

/****************************************************************************
 * Private Functions
 ***************************************************************************/

#define BITMASK(e,a) (a << e)

static void diag_Task( void * pvParameters )
{
    portTickType xLastWakeTime = xTaskGetTickCount();

    CanTxMsg msg;

    msg.RTR = CAN_RTR_Data;
    msg.DLC = 8;

    while (1)
    {
        /*
        if (warningsPresent) {
            d1k_LED_On(WARNING_LED);
        } else {
            d1k_LED_Off(WARNING_LED);
        }

        if (errorsPresent & (
                BITMASK(ERROR_MOTOR_HALLS,1)
                | BITMASK(ERROR_MOTOR_NOT_PRESENT,1)
                | BITMASK(ERROR_MOTOR_OVERTEMP,1)
        )) {
            d1k_LED_On(MOT_ERROR_LED);
        } else {
            d1k_LED_Off(MOT_ERROR_LED);
        }

        if (errorsPresent & (
                BITMASK(ERROR_CURRENT_HARDWARE_FAULT,1)
                | BITMASK(ERROR_CURRENT_SOFTWARE_FAULT,1)
                | BITMASK(ERROR_CURRENT_NONE,1)
                | BITMASK(ERROR_CURRENT_IMBALANCE,1)
        )) {
            d1k_LED_On(I_ERROR_LED);
        } else {
            d1k_LED_Off(I_ERROR_LED);
        }

        if (errorsPresent & (
                BITMASK(ERROR_VOLTAGE_HIGH_PHASE_A,1)
                | BITMASK(ERROR_VOLTAGE_HIGH_PHASE_B,1)
                | BITMASK(ERROR_VOLTAGE_HIGH_PHASE_C,1)
                | BITMASK(ERROR_VOLTAGE_HIGH_BUS,1)
                | BITMASK(ERROR_VOLTAGE_HIGH_15,1)
                | BITMASK(ERROR_VOLTAGE_LOW_15,1)
        )) {
            d1k_LED_On(V_ERROR_LED);
        } else {
            d1k_LED_Off(V_ERROR_LED);
        }

        if (errorsPresent & (
                BITMASK(ERROR_OVERTEMP_A,1)
                | BITMASK(ERROR_OVERTEMP_AN,1)
                | BITMASK(ERROR_OVERTEMP_B,1)
                | BITMASK(ERROR_OVERTEMP_BN,1)
                | BITMASK(ERROR_OVERTEMP_C,1)
                | BITMASK(ERROR_OVERTEMP_CN,1)
                | BITMASK(ERROR_OVERTEMP_AMB,1)
                | BITMASK(ERROR_OVERTEMP_HS,1)
        )) {
            d1k_LED_On(GEN_ERROR_LED);
        } else {
            d1k_LED_Off(GEN_ERROR_LED);
        }


        memcpy(&msg.Data[0],&errorsPresent,sizeof(errorsPresent));
        memcpy(&msg.Data[4],&warningsPresent,sizeof(warningsPresent));

        //fullCAN_SendPacket(&msg,FULLCAN_IDBASE_EVENTS);
    */
        vTaskDelayUntil( &xLastWakeTime, configTICK_RATE_HZ/DIAGNOSTICS_TASK_FREQUENCY);
    }
}

static void init_leds (void)
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

