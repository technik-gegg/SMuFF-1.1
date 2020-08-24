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

#include <stdlib.h>
#include <Arduino.h>
#include "Config.h"

#ifndef _DUETLASER_H
#define _DUETLASER_H    1

#define STATE_INIT              0x8000
#define STATE_IDLE              0x8001
#define STATE_WAIT_START        0x8002
#define STATE_GOT_START         0x8003
#define STATE_READING_DATA      0x8004
#define STATE_GOT_ALL_BITS      0x8005
#define STATE_PARITY_ERROR      0x8006
#define STATE_PARITY_OK         0x8007
#define STATE_GOT_POSITION      0x8008
#define STATE_GOT_ERROR_MSG     0x8009
#define STATE_GOT_QUALITY_V1    0x800A
#define STATE_GOT_QUALITY_V2    0x800B
#define STATE_INVALID           0x800C

#define E_NONE                  0x0
#define E_WRONG_PARITY          0x8000
#define E_INVALID_QUALITY       0x8001
#define E_INVALID_DATA          0x8002
#define E_INVALID_VERSION       0x8003

#define E_SENSOR_VCC            4     // Sensor has reported an VCC error
#define E_SENSOR_INIT           5     // Sensor has reported an init error

#define DIR_EXTRUDE             1
#define DIR_NONE                0
#define DIR_RETRACT             -1

class DuetLaserSensor {
public:
  DuetLaserSensor() { _pin = -1; };
  DuetLaserSensor(int pin, bool v1 = false) { attach(pin, v1); }
  
  void attach(int pin, bool v1 = false);
  void reset();
  void resetPosition();
  void service();

  bool      getSwitch() { return _switch; }
  int8_t    getDirection() { return _dir; }
  double    getPositionMM() { return _positionMM; }
  int32_t   getPosition() { return _position; }
  uint8_t   getBrightness() { return _brightness; }
  uint8_t   getVersion() { return _version; }
  uint8_t   getQuality() { return _quality; }
  uint8_t   getShutter() { return _shutter; }
  uint16_t  getError() { return _error; }
  uint8_t   getSensorError() { return _sensorError; }
  unsigned  getState() { return _state; }
  bool      isValid() { return _isValid; }
  bool      isMoving() { return _hasMoved; }
  String    getBits() { return _bits; }
  String    getStuffBits() { return _stuff; }
  void      resetBits() { _bits = ""; _stuff = ""; }

private:
  int       _pin = -1;
  bool      _switch;  
  bool      _isV1;
  bool      _hasMoved;
  int8_t    _dir;
  double    _positionMM;
  int32_t   _position;
  int32_t   _prevpos;
  uint8_t   _version;
  uint8_t   _brightness;
  uint8_t   _quality;
  uint8_t   _shutter;
  uint16_t  _error;
  uint8_t   _sensorError;
  unsigned  _state;
  unsigned  _data;
  
  bool      _gotIdle;
  bool      _gotStartbit;
  bool      _isValid;
  int       _bitCnt;
  uint8_t   _pbitCnt;
  int       _dataCnt;
  String    _bits;
  String    _stuff;
};

#endif