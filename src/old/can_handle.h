/********************************************************************
can_handle.h

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

#ifndef CAN_HANDLE_H
#define CAN_HANDLE_H

#include "d1k.h"
#include "stm32f4xx_can.h"

typedef void  (*f32_SetterFxn)( float f );
typedef float (*f32_GetterFxn)( void );
typedef void  (*u8_SetterFxn)( uint8 f );
typedef uint8 (*u8_GetterFxn)( void);
typedef void  (*FS_SetterFxn)( FunctionalState b );
typedef FunctionalState (*FS_GetterFxn)( void );

void GenericF32Handler  ( CAN_TypeDef * canModule, CanRxMsg * packet, f32_GetterFxn getter,  f32_SetterFxn setter );
void GenericU8Handler   ( CAN_TypeDef * canModule, CanRxMsg * packet, u8_GetterFxn getter,   u8_SetterFxn setter  );
void GenericFSHandler   ( CAN_TypeDef * canModule, CanRxMsg * packet, FS_GetterFxn getter, FS_SetterFxn setter);

#endif /* CAN_HANDLE_H_ */
