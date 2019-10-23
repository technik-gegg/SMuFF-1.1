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

#define MAX_SERVOS              5
#define US_PER_PULSE_0DEG       544       // microseconds for 0 degrees
#define US_PER_PULSE_180DEG     2400      // microseconds for 180 degrees
#ifdef __STM32F1__
#define TIMER_INTERVAL          800       // CPU-Clock / (Prescaler * 50Hz)-1 =  72000000 / (1800 * 50 - 1) = 800.008
#endif
#ifdef __AVR__
#define TIMER_INTERVAL          312       // CPU-Clock / (Prescaler * 50Hz)-1 =  16000000 / (1024 * 50 - 1) = 311.5
#endif


void isrServoTimerHandler();
static ZTimer  servoTimer;
static volatile bool timerSet = false;

class ZServo {
public:
  ZServo() { _pin = 0; };
  ZServo(int servoIndex, int pin) { attach(pin); setIndex(servoIndex); }

  void attach(int pin);
  void attach(int pin, bool useTimer);
  void attach(int pin, int servoIndex) { attach(pin); setIndex(servoIndex); }
  void setIndex(int servoIndex);
  void detach();
  void write(int degree);
  void writeMicroseconds(int microseconds);
  bool setServoPos(int degree);
  void setServoMS(int microseconds);
  void setServo();
  void setDegreeMinMax(int min, int max) { _minDegree = min; _maxDegree = max; }
  void setPulseWidthMinMax(int min, int max) { _minPw = min; _maxPw = max; }
  void stop() { if(_useTimer) servoTimer.stopTimer(); }

  int getDegree() { return _degree; }
  void getDegreeMinMax(int* min, int* max) { *min = _minDegree; *max = _maxDegree; }
  void getPulseWidthMinMax(int* min, int* max) { *min = _minPw; *max = _maxPw; }

private:
  int _pin;
  bool _useTimer = false;
  volatile uint32 *_pin_reg;
  uint32 _pin_set;
  uint32 _pin_reset;
  int _servoIndex;
  int _degree;
  int _lastDegree;
  uint32 _lastUpdate;
  int _loopCnt;
  int _pulseLen;
  int _minPw = US_PER_PULSE_0DEG;
  int _maxPw = US_PER_PULSE_180DEG;
  int _minDegree = 0;
  int _maxDegree = 180;
};
#endif
