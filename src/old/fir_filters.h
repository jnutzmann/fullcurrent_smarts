/********************************************************************
d1k_can.h

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

#ifndef FIR_FILTERS_H
#define FIR_FILTERS_H

// Coefficients stored in time reversed order.
#define FILTER_HAMMING_4KHZ_LENGTH  (65)
#define FILTER_HAMMING_4KHZ			filter_hamming_4khz
#define USE_FILTER_HAMMING_4KHZ		extern q15_t filter_hamming_4khz[65]

#endif /* FIR_FILTERS_H_ */
