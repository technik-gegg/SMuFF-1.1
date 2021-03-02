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

#define MAX_FANS                2
#define FAN_DUTY_CYCLE          10000    // fan cycle in uS (100Hz by default)

extern void __debugS(const char* fmt, ...);

void isrFanTimerHandler();

class ZFan {
public:
  ZFan() { _pin = 0; };

  void    attach(int8_t pin);
  void    attach(int8_t pin, int8_t fanIndex) { attach(pin); setIndex(fanIndex); }
  void    setIndex(int8_t fanIndex);
  void    detach();
  void    setFanSpeed(uint8_t speed);
  void    setFan();
  void    setFanPin(uint8_t state);
  void    setPulseWidthMinMax(uint16_t min, uint16_t max) { _minSpeed = min; _maxSpeed = max; }
  void    setPulseWidthMin(uint16_t min) { _minSpeed = min; }
  void    setPulseWidthMax(uint16_t max) { _maxSpeed = max; }
  void    setTickRes(uint8_t res) { _tickRes = res; }
  uint8_t getTickRes() { return _tickRes; }
  void    setBlipTimeout(uint16_t millis) { _blipTimeout = (uint32_t)millis * 1000; }
  uint16_t getBlipTimeout() { return (uint16_t)_blipTimeout / 1000; }

private:
  int8_t   _pin;
  int8_t   _fanIndex;
  int8_t   _speed;
  uint16_t _minSpeed = 0;
  uint16_t _maxSpeed = FAN_DUTY_CYCLE;
  uint16_t _tickCnt;
  uint16_t _pulseLen;
  uint8_t  _tickRes = 1;
  uint32_t _blipTime;
  uint32_t _blipTimeout = 0;
};
