/********************************************************************
main.c

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

#include <stm32f4xx.h>
#include "stm32f4xx_gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "adc.h"

#include "d1k_stdio_can.h"
#include "string.h"

#include "smarts_leds.h"
#include "configuration.h"
#include "pwm.h"
#include "halls.h"
#include "d1k_can.h"
#include "system_cfg.h"
#include "fault.h"

#include "d1k_led.h"
#include "smarts_leds.h"
#include "d1k_portal.h"
#include "d1k_stdio.h"
#include "d1k_stdio_uart.h"
#include "string.h"
#include "stdlib.h"
#include <math.h>

#include "mCAN.h"
#include "fullCAN.h"
#include "control_loop.h"
#include "d1k_fram.h"
#include "adc.h"
#include "diagnostics.h"
#include "fan.h"
#include "motor.h"
#include "motor_position.h"
#include "pole_position.h"
#include "fram_map.h"

static void InitLEDs    ( void );
static void InitDebug ( void );

int main()
{
	// This is needed for FreeRTOS.
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	InitDebug();

	cfg_Init( );

	PolePosition_Init();

	// This MUST be initialized before fullCAN
	MotorPosition_Init(FEEDBACK_POLE_POSITION);

	fullCAN_Init( 1000000, cfg_GetControllerID() );

	if ( cfg_GetControllerID() == 0)
	{
		fullCAN_SetTerminationState(ENABLE);
	} else {
		fullCAN_SetTerminationState(DISABLE);
	}

	d1k_STDIO_CAN_Init( CAN2,
			fullCAN_GetControllerId() + FULLCAN_IDBASE_STDIO_TX,
			fullCAN_GetControllerId() + FULLCAN_IDBASE_STDIO_RX );

	d1k_portal_Init( );

	ControlLoopInit();

	InitLEDs( );

	adc_Init( 100000 );

	motor_Init( );

	fault_Init( );

	diag_Init( );

	fan_Init( );

	fan_SetDutyCycle( 1.0f );

	printf("fullStartup! Controller: %u\r\n", (uint16)fullCAN_GetControllerId());

	vTaskStartScheduler( );
}

static void InitDebug ( void )
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Configure the GPIO_LED pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOB,GPIO_Pin_14); // Ground

}

static void InitLEDs ( void )
{

	d1k_LEDInitStruct_t ledInit;
	ledInit.GIOPx = GPIOD;
	ledInit.offTime = 0;
	ledInit.onTime = 0;
	ledInit.GPIO_Clock = RCC_AHB1Periph_GPIOD;
	ledInit.purpose = D1K_LED_PURPOSE_CAN;
	ledInit.GPIO_Pin = GPIO_Pin_7;
	d1k_LED_Init(CAN_LED,&ledInit);

	ledInit.purpose = D1K_LED_PURPOSE_ERROR;
	ledInit.GPIO_Pin = GPIO_Pin_2;
	d1k_LED_Init(GEN_ERROR_LED,&ledInit);

	ledInit.purpose = D1K_LED_PURPOSE_APPLICATION;
	ledInit.GPIO_Pin = GPIO_Pin_6;
	d1k_LED_Init(WARNING_LED,&ledInit);

	ledInit.GPIO_Pin = GPIO_Pin_5;
	d1k_LED_Init(MOT_ERROR_LED,&ledInit);

	ledInit.GPIO_Pin = GPIO_Pin_4;
	d1k_LED_Init(V_ERROR_LED,&ledInit);

	ledInit.GPIO_Pin = GPIO_Pin_3;
	d1k_LED_Init(I_ERROR_LED,&ledInit);

	ledInit.offTime = 500;
	ledInit.onTime = 500;
	ledInit.GIOPx = GPIOB;
	ledInit.GPIO_Clock = RCC_AHB1Periph_GPIOB;
	ledInit.GPIO_Pin = GPIO_Pin_5;
	d1k_LED_Init(HEARTBEAT,&ledInit);
}

