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

#ifndef _ZFAN_H
#define _ZFAN_H

#define MAX_FANS                2
#define FAN_DUTY_CYCLE          150     // fan cycle in us (7500)

extern void __debug(const char* fmt, ...);

void isrFanTimerHandler();

class ZFan {
public:
  ZFan() { _pin = 0; };

  void attach(int pin);
  void attach(int pin, int fanIndex) { attach(pin); setIndex(fanIndex); }
  void setIndex(int fanIndex);
  void detach();
  void setFanSpeed(int speed);
  void setFan();
  void setFanPin(int state);
  void setPulseWidthMinMax(int min, int max) { _minSpeed = min; _maxSpeed = max; }
  void setPulseWidthMin(int min) { _minSpeed = min; }
  void setPulseWidthMax(int max) { _maxSpeed = max; }

private:
  int   _pin;
  bool  _useTimer = false;
  bool  _timerStopped = false;
  int   _fanIndex;
  int   _speed;
  int   _minSpeed = 0;
  int   _maxSpeed = FAN_DUTY_CYCLE;
#ifdef __STM32F1__
  volatile uint32_t _tickCnt;
#else
  volatile int _tickCnt;
#endif
  volatile int _dutyCnt;
  int   _loopCnt;
  int   _pulseLen;
};
#endif