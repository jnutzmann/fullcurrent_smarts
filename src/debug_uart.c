/********************************************************************
debug_uart.h

Copyright (c) 2014, Jonathan Nutzmann, Arlo Siemsen

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
********************************************************************/

#include "debug_uart.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "FreeRTOS.h"
#include "queue.h"

// TODO: check these?
#define START_CHAR  (0x7E)
#define ESCAPE_CHAR (0x7D)


#define MAX_PACKET_SIZE (19)  //
#define MAX_DATA_SIZE   (17)
// [SSSS SSSS] [AAAA AAAA] [AAAR LLLL] [DDDD DDDD] [x0-15] [CCCC CCCC]

static xQueueHandle rxQueue;                  // Receive queue (packet level)
static xQueueHandle txQueue;                  // Transmit queue (byte level)
static char rxPacketBuffer[MAX_PACKET_SIZE];  // Buffer used to hold packet before fully received.

void debug_uart_init(uint32_t baudrate)
{
    // Setup USART3
    // PB10 - TX
    // PB11 - RX

    // Setup GPIOB Pins

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    GPIO_InitTypeDef gpio_init;
    gpio_init.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio_init.GPIO_Mode = GPIO_Mode_AF;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &gpio_init);

    // Setup USART3 peripheral

    RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    USART_InitTypeDef usart_init;
    usart_init.USART_BaudRate = baudrate;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART3, &usart_init);
    USART_Cmd(USART3, ENABLE);


    // Enable receive interrupts
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    // Init the NVIC.
    NVIC_InitTypeDef NVIC_InitStruct;

    NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0; // TODO: priorities?
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStruct);
}

static void try_send()
{
    // Check to make sure the buffer is empty

    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}


bool debug_uart_send_packet(void* data, size_t len)
{

}

size_t debug_uart_receive_packet(void* data, size_t max);

# TODO: how do I handle receive? - Push into skylab handler.
# does packetization get handled here?

#endif /* UART_H */
