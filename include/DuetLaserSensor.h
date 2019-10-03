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
class DuetLaserSensor {
public:
  DuetLaserSensor() { _pin = -1; };
  DuetLaserSensor(int pin) { attach(pin); }
  
  void attach(int pin);
  void reset();
  void service();

  unsigned  getSwitch() { return _switch; }
  double    getPositionMM() { return _positionMM; }
  unsigned  getPosition() { return _position; }
  unsigned  getBrightness() { return _brightness; }
  unsigned  getVersion() { return _version; }
  unsigned  getQuality() { return _quality; }
  unsigned  getShutter() { return _shutter; }
  unsigned  getError() { return _error; }
  unsigned  getState() { return _state; }
  bool      isValid() { return _isValid; }
  String    getBits() { return _lastBits; }
  String    getStuffBits() { return _lastStuff; }
  void      resetBits() { _lastBits = ""; _lastStuff = ""; }

private:
  int       _pin = -1;
  bool      _switch;  
  bool      _isV1;
  double    _positionMM;
  unsigned  _position;
  unsigned  _version;
  unsigned  _brightness;
  unsigned  _quality;
  unsigned  _shutter;
  unsigned  _error;
  int       _lastBit;
  unsigned  _state;
  unsigned  _data;
  
  bool      _gotIdle;
  bool      _gotStartbit;
  bool      _isValid;
  int       _bitCnt;
  int       _dataCnt;
  String    _bits;
  String    _stuff;
  String    _lastBits;
  String    _lastStuff;

};

#endif