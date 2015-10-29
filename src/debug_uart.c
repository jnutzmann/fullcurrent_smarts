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

#include "debug_uart.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "led.h"
#include "diagnostics.h"

#define DEBUG_UART_TX_QUEUE_DEPTH (128)
#define DEBUG_UART_RX_QUEUE_DEPTH (10)


#define START_CHAR  (0x7E)
#define ESCAPE_CHAR (0x7D)

static uint8_t start_char = START_CHAR;
static uint8_t escape_char = ESCAPE_CHAR;


#define MAX_PACKET_SIZE (19)
// [SSSS SSSS] [AAAA AAAA] [AAAR LLLL] [DDDD DDDD] [x0-15] [CCCC CCCC]


static xQueueHandle rx_queue;                  // Receive queue (packet level)
static xQueueHandle tx_queue;                  // Transmit queue (byte level)
static char rxPacketBuffer[MAX_PACKET_SIZE];  // Buffer used to hold packet before fully received.

static void start_send( void );

static void send_start_char( void )
{
    //xQueueSendFromISR( tx_queue, &start_char, NULL );
}

static void send_escaped_char( uint8_t c )
{
    if ( c == ESCAPE_CHAR || c == START_CHAR )
    {
        //xQueueSendFromISR( tx_queue, &escape_char, NULL );
        //c ^= 0x20;
    }

    //xQueueSendFromISR( tx_queue, &c, NULL );
}

bool debug_uart_send_packet( DebugUartPayload_t * payload )
{
    send_start_char();

    uint8_t holder = (payload->address) >> 3;
    send_escaped_char(holder);

    holder = (payload->address) << 5
             | ((payload->request_to_receive) << 4)
             | ((payload->length));
    send_escaped_char(holder);

    for ( int i=0; i < payload->length; i++)
    {
        send_escaped_char(payload->data[i]);
    }

    holder = 0;  // TODO: CHECKSUM
    send_escaped_char(holder);

    start_send();

    return true;
}


void debug_uart_init(uint32_t baudrate)
{
    tx_queue = xQueueCreate(DEBUG_UART_TX_QUEUE_DEPTH, sizeof(uint8_t));

    // USART3, PB10 - TX, PB11 - RX

    // Setup GPIOB Pins
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    GPIO_InitTypeDef gpio_init;
    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio_init.GPIO_Mode = GPIO_Mode_AF;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &gpio_init);


    // Setup USART3 peripheral
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    USART_InitTypeDef usart_init;
    USART_StructInit(&usart_init);
    usart_init.USART_BaudRate = baudrate;
    USART_Init(USART3, &usart_init);
    USART_Cmd(USART3, ENABLE);

    // Init the NVIC.
    NVIC_InitTypeDef nvic_init;

    nvic_init.NVIC_IRQChannel = USART3_IRQn;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 5; // TODO: priorities?
    nvic_init.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&nvic_init);

    USART_ClearITPendingBit( USART3, USART_IT_RXNE );
    USART_ITConfig( USART3, USART_IT_RXNE, ENABLE );
}

static void start_send( void )
{   uint8_t c = "c";
    xQueueSendFromISR(tx_queue, &c, NULL);
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);

}

void USART3_IRQHandler( void )
{
    if ( USART_GetITStatus( USART3, USART_IT_RXNE ) )
    {
        uint8_t data = (uint8_t) USART_ReceiveData(USART3);
        xQueueSendFromISR( tx_queue, &data, NULL );
        start_send();
    }

    if ( USART_GetITStatus( USART3, USART_IT_TXE ) )
    {
        uint16_t to_send = 0;

        led_on(LED_WARNING);

        if ( xQueueReceiveFromISR( tx_queue, &to_send, NULL ) == pdTRUE )
        {
            USART_SendData(USART3, to_send);
        }
        else
        {
            // we ran out of data to send, so disable the interrupt
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
        }
    }
}
