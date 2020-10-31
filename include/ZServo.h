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
#include "ZTimerLib.h"

#define MAX_SERVOS              5
#define US_PER_PULSE_0DEG       1000      // microseconds for 0 degrees
#define US_PER_PULSE_180DEG     2000      // microseconds for 180 degrees
#define DUTY_CYCLE              20000     // servo cycle in us (>= 20ms)
#ifdef __ESP32__
  #define SERVO_CHANNEL         8
  #define SERVO_FREQ            50
#endif

extern void __debug(const char* fmt, ...);

void isrServoTimerHandler();
static volatile bool timerSet = false;

class ZServo {
  public:
    ZServo() { _pin = 0; };
    ZServo(int servoIndex, int pin) { attach(pin); setIndex(servoIndex); }

    void attach(int pin);
    void attach(int pin, bool useTimer, int servoIndex = -1);
    void attach(int pin, int servoIndex) { attach(pin); setIndex(servoIndex); }
    void setIndex(int servoIndex);
    void detach();
    void write(int val);
    void writeMicroseconds(int microseconds);
    bool setServoPos(int val);                  // sets the servo position if val is between 0 and 180, otherwise it sets the frequency (ms)
    void setServoMS(int microseconds);          // sets the frequency in milliseconds
    void setServo();                            // method called cyclically from an interrupt to refresh the servo position
    void setDegreeMinMax(int min, int max) { _minDegree = min; _maxDegree = max; }
    void setPulseWidthMinMax(int min, int max) { _minPw = min; _maxPw = max; }
    void setPulseWidthMin(int min) { _minPw = min; }
    void setPulseWidthMax(int max) { _maxPw = max; }
    void stop(bool state) { _timerStopped = state; }
    bool isTimerStopped() { return _timerStopped; }
    bool hasTimer() { return _useTimer; }
    void setMaxCycles(int val) { _maxCycles = val;}
    int  getMaxCycles() { return _maxCycles;}

    int getDegree() { return _degree; }
    void getDegreeMinMax(int* min, int* max) { *min = _minDegree; *max = _maxDegree; }
    void getPulseWidthMinMax(int* min, int* max) { *min = _minPw; *max = _maxPw; }

  private:
    void setServoPin(int state);
    int _pin;
    bool _useTimer = false;
    bool _timerStopped = false;
    int _servoIndex;
    int _degree;
    int _lastDegree;
    servotick_t _lastUpdate;
    volatile servotick_t _tickCnt;
    volatile int _dutyCnt;
    int _maxCycles;
    int _pulseLen;
    int _minPw = US_PER_PULSE_0DEG;
    int _maxPw = US_PER_PULSE_180DEG;
    int _minDegree = 0;
    int _maxDegree = 180;
    int _toggle = 0;
};
