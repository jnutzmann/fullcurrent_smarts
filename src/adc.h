/********************************************************************
adc.h

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

#ifndef ADC_H
#define ADC_H

/****************************************************************************
 * Includes
 ***************************************************************************/

#include "arm_math.h"

/****************************************************************************
 * Defines
 ***************************************************************************/

#define ADC_NUM_CONVERSIONS 	(5)
#define ADC_NUM_SIM_CHANNELS	(3)
#define ADC_NUM_CHANNELS		(ADC_NUM_CONVERSIONS * ADC_NUM_SIM_CHANNELS)

#define DMA_DEPTH				(64)

typedef enum
{
	ADC_CH_INDEX_IA     = (0) ,
	ADC_CH_INDEX_IB		= (1) ,
	ADC_CH_INDEX_IC		= (2) ,
	ADC_CH_INDEX_VA		= (3) ,
	ADC_CH_INDEX_VB		= (4) ,
	ADC_CH_INDEX_VC		= (5) ,
	ADC_CH_INDEX_VN		= (6) ,
	ADC_CH_INDEX_VGATE	= (7) ,
	ADC_CH_INDEX_VBUS   = (8) ,
	ADC_CH_INDEX_T		= (9),
	ADC_CH_INDEX_TN	    = (10),
	ADC_CH_INDEX_TM1	= (12),
	ADC_CH_INDEX_HWIF	= (13),
} ADC_Channel_t;

/****************************************************************************
 * Public Function Prototypes
 ***************************************************************************/

void adc_init( uint32_t fs )

#endif /* ADC_H_ */
