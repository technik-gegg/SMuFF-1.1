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
#include "ZTimerLib.h"

#ifndef _ZPWM_H
#define _ZPWM_H

#define PWM_INTERVAL          1         // equals 10 kHz on 72Mhz CPU based on a prescaler of 7200

void isrPwmTimerHandler();

class ZPwm {
public:
  ZPwm() { _pin = 0; };
  ZPwm(int pin) { attach(pin); }

  void attach(int pin) { 
    _pin = pin; 
    pinMode(_pin, OUTPUT); 
    digitalWrite(_pin, 0);
    pwmTimer.setupTimer(ZTimer::ZTIMER1, 7200);
    pwmTimer.setupTimerHook(isrPwmTimerHandler);
  }
  void write(int speed) { writePwm(speed); }
  bool writePwm(int speed);
  void setPwm() { digitalWrite(_pin, LOW); delayMicroseconds(_pulseLen); digitalWrite(_pin, HIGH); }
  void stop() { pwmTimer.stopTimer(); }

  int  getSpeed() { return _speed; }

   void isrHandler() { setPwm(); }

private:
  ZTimer    pwmTimer;
  int       _pin;
  int       _speed;
  int       _pulseLen;
};
#endif
