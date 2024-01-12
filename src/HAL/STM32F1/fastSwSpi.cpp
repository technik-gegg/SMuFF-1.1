/**
 * SMuFF Firmware
 * Copyright (C) 2019-2024 Technik Gegg
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#if defined(__STM32F1XX)

#include "SMuFF.h"


/*
    Code copied from https://github.com/olikraus/u8g2/issues/749 and adopted for testing
*/

//
// Delay built by waiting n cpu clocks
//
void smuff_spi_delay(/*int x*/) {  // function overhead with parameter 69ns 
    
    /*
    while (x--) {           // adds approx. 110ns + function overhead
    }
    */
    asm volatile("nop\n"
                 "nop\n"
                 "nop\n"
                 "nop\n"
                 "nop" ::); // 13.8ns per NOP 
                            // approx. total 69ns
}

#endif