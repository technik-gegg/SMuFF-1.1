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

/*
    The EStopMux is an Arduino Pro Micro which has all the Endstops attached to
    digital inputs and reports the status of the currently selected Endstop on
    one output pin.
    This output pin is being read from the SMuFF firmware in the same way as
    an ordinary Endstop sensor (means HIGH/LOW).
    The current Endstop in question is configured by the setTool() method.
*/
#include "ZEStopMux.h"


#if !defined(USE_SW_TWI)
void ZEStopMux::begin(TwoWire* i2cInst, uint8_t address) {
	_i2cBusInst = i2cInst;
	begin(address);
}
#else
void ZEStopMux::begin(SoftWire* i2cInst, uint8_t address) {
	_i2cBusInst = i2cInst;
	begin(address);
}
#endif

void ZEStopMux::begin(uint8_t address) {
	_address = address;
	if(_i2cBusInst != nullptr) {
		_i2cBusInst->begin();
	}
}

/**
 * @brief Send current tool to MUX
 *
 * @param Number of the tool to be set (0-5)
 *
 * @return bool True if ok, False on error
 */
bool ZEStopMux::setTool(int8_t tool) {
	if(_i2cBusInst == nullptr)
        return false;
    _i2cBusInst->beginTransmission(_address);
    _i2cBusInst->write(SET_TOOL);
    _i2cBusInst->write(tool);
    uint8_t stat = _i2cBusInst->endTransmission();
    //__debugS(PSTR("Set Tool stat %d "), stat);
    return true;
}

/**
 * @brief Issue an Soft-Reset on the MUX
 *
 * @return bool True if ok, False on error
 */
bool ZEStopMux::reset() {
	if(_i2cBusInst == nullptr)
        return false;
    _i2cBusInst->beginTransmission(_address);
    _i2cBusInst->write(RESET_MUX);
    uint8_t stat = _i2cBusInst->endTransmission();
    //__debugS(PSTR("Reset stat %d "), stat);
    return true;
}

/**
 * @brief Request current tool
 *
 * @return int8_t The current tool set
 */
int8_t ZEStopMux::getTool() {
	if(_i2cBusInst == nullptr)
        return -1;
    uint8_t tool = 0;
    _i2cBusInst->beginTransmission(_address);
    _i2cBusInst->write(GET_TOOL);
    uint8_t stat = _i2cBusInst->endTransmission();
    //__debugS(PSTR("Get Tool stat %d "), stat);

    _i2cBusInst->requestFrom(_address, 1);
    if(_i2cBusInst->available()) {
        tool = (int8_t)_i2cBusInst->read();
        __debugS(PSTR("Got: %d"), tool);
    }
    return (int8_t)tool;
}

/**
 * @brief Query the firmware version from MUX
 *
 * @return int8_t The current firmware version or -1 on error
 */
int8_t ZEStopMux::getVersion() {
	if(_i2cBusInst == nullptr)
        return -1;
    int8_t version = 0;
    _i2cBusInst->beginTransmission(_address);
    _i2cBusInst->write(GET_VERSION);
    uint8_t stat = _i2cBusInst->endTransmission();
    //__debugS(PSTR("Get Version stat %d "), stat);

    _i2cBusInst->requestFrom(_address, 1);
    if(_i2cBusInst->available()) {
        version = (int8_t)_i2cBusInst->read();
        __debugS(PSTR("Got: %d"), version);
    }
    return version;
}

/**
 * @brief Query the amount of pins supported by MUX
 *
 * @return int8_t The number of pins supported or -1 on error
 */
int8_t ZEStopMux::getMaxPins() {
	if(_i2cBusInst == nullptr)
        return -1;
    int8_t maxPins = 0;
    _i2cBusInst->beginTransmission(_address);
    _i2cBusInst->write(GET_MAXPINS);
    uint8_t stat = _i2cBusInst->endTransmission();
    //__debugS(PSTR("Get Pins stat %d "), stat);

    _i2cBusInst->requestFrom(_address, 1);
    if(_i2cBusInst->available()) {
        maxPins = (int8_t)_i2cBusInst->read();
        __debugS(PSTR("Got: %d"), maxPins);
    }
    return maxPins;
}
