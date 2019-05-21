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

#ifndef _ZSERVO_H
#define _ZSERVO_H

#define US_PER_PULSE_0DEG       500       // 0 degrees
#define US_PER_PULSE_180DEG     2400      // 180 degrees
#define TIMER_INTERVAL          312       // CPU-Clock / (Prescaler * 50)-1 =  16000000 / (1024 * 50) - 1 = 311,5


void isrServoTimerHandler();

class ZServo {
public:
  ZServo() { _pin = 0; };
  ZServo(int pin) { attach(pin); }

  void attach(int pin) { 
    _pin = pin; 
    pinMode(_pin, OUTPUT); 
    digitalWrite(_pin, 0);
    servoTimer.setupTimer(ZTimer::TIMER5, ZTimer::PRESCALER1024);
    servoTimer.setupTimerHook(isrServoTimerHandler);
  }
  void write(int degree);
  bool setServoPos(int degree);
  void setServoMS(int microseconds);
  void setServo();
  void stop() { servoTimer.stopTimer(); }

  int getDegree() { return _degree; }

private:
  ZTimer  servoTimer;
  int _pin;
  int _degree;
  int _pulseLen;
};
#endif
