/**
 * SMuFF Firmware
 * Copyright (C) 2019-2022 Technik Gegg
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

#include "ZServo.h"

extern void fastFlipDbg();

static ZServo* servoInstances[MAX_SERVOS];

/*
  To use the servo with an timer (interrupt), you have to setup an timer
  externally, call the attach method with useTimer = true and call the
  isrServoTimerHandler() method down below from within the timers
  own interrupt routine.
  The external timer has to run at 50uS (20kHz) in order to get the correct
  timing for the servos.
  Otherwise, as in case of the ESP32, the servos will be handled by
  PWM on the give pin.

  Had to realize it this way because I was running out of precious timers
  on the STM32 MCU.
*/
void ZServo::attach(pin_t pin, bool useTimer, int8_t servoIndex) {
  _useTimer = useTimer;
  attach(pin);
  setIndex(servoIndex);
  enable();

  if(!_useTimer) {
    #if defined(__ESP32__)
    ledcSetup(SERVO_CHANNEL+_servoIndex, SERVO_FREQ, 16);
    ledcAttachPin(pin, SERVO_CHANNEL+_servoIndex);
    //__debugS(D, PSTR("Servo channel: %d"), SERVO_CHANNEL+_servoIndex);
    #endif
    //__debugS(D, PSTR("Servo without timer initialized"));
  }
}

void ZServo::attach(pin_t pin) {
  _pin = pin;
  if(_pin != 0) {
    #if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)
      _fastPin = &(digitalPinToPort(_pin)->BSRR);
      _pinMask_S = digitalPinToBitMask(_pin);
      _pinMask_R = digitalPinToBitMask(_pin) << 16;
    #endif
    digitalWrite(_pin, 0);
  }
}

void ZServo::detach() {
  servoInstances[_servoIndex] = nullptr;
  #if defined(__ESP32__)
    if(!_useTimer)
      ledcDetachPin(_pin);
  #endif
  _pin = 0;
}

/*
  Disable the servo by setting the port pin to INPUT
*/
void ZServo::disable() {
  if(_pin == 0)
    return;
  pinMode(_pin, INPUT);
  _disabled = true;
  _pulseComplete = true;
  // __debugS(DEV, PSTR("ZServo::disable %d: pin = %d]"), _servoIndex, _pin);
}

/*
  Enable the servo
*/
void ZServo::enable() {
  if(_pin == 0)
    return;
  pinMode(_pin, OUTPUT);
  _disabled = false;
  // __debugS(DEV3, PSTR("ZServo::enable %d: pin = %d"), _servoIndex, _pin);
}

/*
  Main method to set the servo position.
  Values between 0 and 180 will be treated as degrees.
  Values above 499 will be treated as microseconds (usually in the range of 500-2400 us).
*/
void ZServo::write(uint16_t val) {
  // __debugS(DEV3, PSTR("ZServo::write %d: val = %d"), _servoIndex, val);
  if(val >= 0 && val <= 180)
    setServoPos((uint8_t)val);
  else if(val >= 500 && val <= 2400)
    setServoMS(val);
}

void ZServo::writeMicroseconds(uint16_t microseconds) {
  setServoMS(microseconds);
}

void ZServo::setIndex(int8_t servoIndex) {
  // __debugS(DEV3, PSTR("ZServo::setIndex %d"), servoIndex);
  if(servoIndex >= 0 && servoIndex < MAX_SERVOS) {
    _servoIndex = servoIndex;
    servoInstances[_servoIndex] = this;
  }
}

bool ZServo::setServoPos(uint8_t degree) {
  if(_pin == 0)
    return false;

  if(_disabled)
    enable();

  _lastDegree = _degree;

  #if defined(__ESP32__)
    if(!_useTimer) {
      _pulseLen = (int)(((degree/(double)_maxDegree)*_maxPw)/(double)DUTY_CYCLE*65536.0) + ((65536.0/DUTY_CYCLE)*_minPw);
      //__debugS(Dev3, PSTR("ZServo::setServoPos %d: %3d deg = %4d us (v:%d)"), _servoIndex, degree, (int)((double)_pulseLen / ((double)65536 / DUTY_CYCLE)), _pulseLen);
    }
    else {
      _pulseLen = map(degree, _minDegree, _maxDegree, _minPw, _maxPw);
      //__debugS(Ddev3, PSTR("ZServo::setServoPos %d: %3d deg = %4d us"), _servoIndex, degree, _pulseLen);
    }
  #else
    _pulseLen = (uint16_t)map((int32_t)degree, (int32_t)_minDegree, (int32_t)_maxDegree, (int32_t)_minPw, (int32_t)_maxPw);
    _pulseLen = (uint16_t)((double)_pulseLen / _tickRes +.5) * _tickRes;    // round to _tickRes
    // __debugS(DEV3, PSTR("ZServo::setServoPos %d: %3d deg = %4d us %3d"), _servoIndex, degree, _pulseLen, _pulseLen/_tickRes);
  #endif
  _degree = degree;
  _lastUpdate = millis();
  _tickCnt = 0;
  _dutyCnt = 0;

  return true;
}

void ZServo::setServoMS(uint16_t microseconds) {
  if(_disabled)
    enable();
  _pulseLen = (microseconds / _tickRes) * _tickRes;   // round to _tickRes
  _lastDegree = _degree;
  _degree = (uint8_t)map((int32_t)_pulseLen, (int32_t)_minPw, (int32_t)_maxPw, (int32_t)_minDegree, (int32_t)_maxDegree);
  _lastUpdate = millis();
  _tickCnt = 0;
  _dutyCnt = 0;
  // __debugS(DEV3, PSTR("ZServo::setServoMS %d: %3d deg = %4dus %3d"), _servoIndex, _degree, _pulseLen, _pulseLen/_tickRes);
}

/*
  This method is called every DUTY_CYCLE milliseconds to refresh or
  to set the servo position if the timer is not being used.
  If the timer is being used, this method will be called every _tickRes milliseconds 
  from within isrServoHandler().
*/
void ZServo::setServo() {

  noInterrupts();
  if(!_useTimer) {
    if(_degree != _lastDegree) { // avoid jitter on servo by ignoring this call
      #if defined(__ESP32__)
        ledcWrite(SERVO_CHANNEL+_servoIndex, _pulseLen);
      #else
        digitalWrite(_pin, HIGH);
        delayMicroseconds(_pulseLen);
        digitalWrite(_pin, LOW);
      #endif
    }
  }
  else {
    // This (timer) option increments with every call by _tickRes (uS) and when _pulseLen is
    // reached it'll reset the output to low.
    // This way the MCU isn't being blocked while the delay is active, as it's in the method above.
    if(_tickCnt <= _pulseLen) {
      if(_pulseComplete) {
        #if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)
          *_fastPin = _pinMask_S;   // set pin to HIGH
        #else
          setServoPin();
        #endif
        _pulseComplete = false;
      }
    }
    else {
      if(!_pulseComplete) {
        #if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)
          *_fastPin = _pinMask_R;   // set pin to LOW
        #else
          resetServoPin();
        #endif
        _pulseComplete = true;
      }
    }
    // reset tick counter after the duty cycle for the next cycle
    if(_tickCnt >= DUTY_CYCLE) {
      // but no more cycles than defined by _maxCycles (to reduce jitter on the servo)
      // if _maxCycles is 0, cycle keeps repeating
      if(_maxCycles == 0 || (++_dutyCnt < _maxCycles))
        _tickCnt = 0;
    }
    else {
      _tickCnt += (uint32_t)(_tickRes - _tickAdjust);
    }
  }
  interrupts();
}

void ZServo::setDelay() {
    // calculate the delay needed to allow the servo to reach the new position
    // based on a moving speed of MOVING_SPEED per 60° (default 200ms)
    uint8_t dist = abs(_lastDegree - _degree);
    if(dist) {
      double dly = ((double)dist/60)*MOVING_SPEED;
      delay((uint32_t)dly);
    }
}

void isrServoHandler() {
  // call all handlers for all servos periodically if the
  // internal timer is being used.
  // fastFlipDbg();
  for(int servoIndex=0; servoIndex < MAX_SERVOS; servoIndex++) {
    if(servoInstances[servoIndex] == nullptr || servoInstances[servoIndex]->isDisabled())
      continue;
    servoInstances[servoIndex]->setServo();
  }
  // fastFlipDbg();
}
