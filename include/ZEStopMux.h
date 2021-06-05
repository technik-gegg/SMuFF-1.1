/**
 * SMuFF Firmware
 * Copyright (C) 2019 Technik Gegg
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
#pragma once

#include <stdlib.h>
#include <Arduino.h>
#include "Config.h"
#if defined(__STM32F1__)
#include "HAL/STM32F1/i2c.h"
#endif

extern void __debugS(const char* fmt, ...);

#define GET_TOOL        0
#define SET_TOOL        1
#define GET_VERSION     2
#define GET_MAXPINS     3
#define RESET_MUX       99

class ZEStopMux {
private:
    uint8_t _address = 0;
 	/** instance of which to use **/
	#if !defined(USE_SW_TWI)
	TwoWire* 	_i2cBusInst = nullptr;
	#else
	SoftWire* 	_i2cBusInst = nullptr;
	#endif

public:
    ZEStopMux() { _i2cBusInst = nullptr; _address = 0; }

  	#if !defined(USE_SW_TWI)
	void begin(TwoWire* i2cInst, uint8_t address = 0x3E);
	#else
	void begin(SoftWire* i2cInst, uint8_t address = 0x3E);
	#endif
	void begin(uint8_t address = 0x3E);

    bool setTool(int8_t tool);
    bool reset();
    int8_t getTool();
    int8_t getVersion();
    int8_t getMaxPins();

};