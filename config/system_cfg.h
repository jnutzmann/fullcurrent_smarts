/********************************************************************
system_cfg.h

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

#ifndef SYSTEM_STM32F4XX_CFG_H
#define SYSTEM_STM32F4XX_CFG_H

#define SYS_SYSCLK_HZ		(168000000)
#define SYS_HLK_HZ			(168000000)
#define SYS_AHB_PRESCALE 	(1)
#define SYS_APB1_PRESCALE	(4)
#define SYS_APB2_PRESCALE 	(2)
#define SYS_HSE_HZ			(8000000)

#define SYS_APB1_HZ			(SYS_HLK_HZ/SYS_APB1_PRESCALE)
#define SYS_APB2_HZ			(SYS_HLK_HZ/SYS_APB2_PRESCALE)


#endif
