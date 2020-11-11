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
#define FAN_DUTY_CYCLE          150     // fan cycle in us (7500)

extern void __debug(const char* fmt, ...);

void isrFanTimerHandler();

class ZFan {
public:
  ZFan() { _pin = 0; };

  void attach(int8_t pin);
  void attach(int8_t pin, int8_t fanIndex) { attach(pin); setIndex(fanIndex); }
  void setIndex(int8_t fanIndex);
  void detach();
  void setFanSpeed(uint8_t speed);
  void setFan();
  void setFanPin(uint8_t state);
  void setPulseWidthMinMax(uint8_t min, uint8_t max) { _minSpeed = min; _maxSpeed = max; }
  void setPulseWidthMin(uint8_t min) { _minSpeed = min; }
  void setPulseWidthMax(uint8_t max) { _maxSpeed = max; }

private:
  int8_t   _pin;
  int8_t   _fanIndex;
  int8_t   _speed;
  uint8_t  _minSpeed = 0;
  uint8_t  _maxSpeed = FAN_DUTY_CYCLE;
  uint16_t _tickCnt;
  uint16_t _pulseLen;
};
