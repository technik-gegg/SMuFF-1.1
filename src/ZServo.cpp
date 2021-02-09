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

#include "ZServo.h"

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
void ZServo::attach(int8_t pin, bool useTimer, int8_t servoIndex) {
  _useTimer = useTimer;
  attach(pin);
  setIndex(servoIndex);

  if(!_useTimer) {
    #if defined(__ESP32__)
    ledcSetup(SERVO_CHANNEL+_servoIndex, SERVO_FREQ, 16);
    ledcAttachPin(pin, SERVO_CHANNEL+_servoIndex);
    //__debugS(PSTR("Servo channel: %d"), SERVO_CHANNEL+_servoIndex);
    #endif
    //__debugS(PSTR("Servo without timer initialized"));
  }
}

void ZServo::attach(int8_t pin) {
  _pin = pin;
  enable();
  digitalWrite(_pin, 0);
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
  if(_pin <= 0)
    return;
  _disabled = true;
  #if defined(__STM32F1__)
    #if SERVO_OPEN_DRAIN == 1
      #if defined(__BRD_SKR_MINI)
        pinMode(_pin, INPUT);               // weird... no open drain!?
      #else
        pinMode(_pin, INPUT_OPEN_DRAIN);    // set to Open Drain for the +5V pullup resistor
      #endif
    #else
    pinMode(_pin, INPUT);
    #endif
  #else
    pinMode(_pin, INPUT);
  #endif
}

/*
  Enable the servo
*/
void ZServo::enable() {
  if(_pin <= 0)
    return;
  _disabled = false;
  #if defined(__STM32F1__)
    #if SERVO_OPEN_DRAIN == 1
    pinMode(_pin, OUTPUT_OPEN_DRAIN);   // set to Open Drain for the +5V pullup resistor
    #else
    pinMode(_pin, OUTPUT);
    #endif
  #else
    pinMode(_pin, OUTPUT);
  #endif
}

/*
  Main method to set the servo position.
  Values between 0 and 180 will be treated as degrees.
  Values above 180 will be treated as milliseconds (usually in the range of 500-2400 us).
*/
void ZServo::write(uint16_t val) {
  if(val >= 0 && val <= 180)
    setServoPos(val);
  else
    setServoMS(val);
}

void ZServo::writeMicroseconds(uint16_t microseconds) {
  setServoMS(microseconds);
}

void ZServo::setIndex(int8_t servoIndex) {
  if(servoIndex >= 0 && servoIndex < MAX_SERVOS) {
    _servoIndex = servoIndex;
    servoInstances[_servoIndex] = this;
  }
}

bool ZServo::setServoPos(uint8_t degree) {
  if(_pin <= 0)
    return false;

  if(_disabled)
    enable();

  #if defined(__ESP32__)
  if(!_useTimer) {
    _pulseLen = (int)(((degree/(float)_maxDegree)*_maxPw)/(float)DUTY_CYCLE*65536.0) + ((65536.0/DUTY_CYCLE)*_minPw);
    //__debugS(PSTR("Servo %d: %d° = %d us (v:%d)"), _servoIndex, degree, (int)((float)_pulseLen / ((float)65536 / DUTY_CYCLE)), _pulseLen);
  }
  else {
    _pulseLen = map(degree, _minDegree, _maxDegree, _minPw, _maxPw);
    //__debugS(PSTR("Servo %d: %d° = %d us"), _servoIndex, degree, _pulseLen);
  }
  #else
  _pulseLen = map(degree, _minDegree, _maxDegree, _minPw, _maxPw);
  //__debugS(PSTR("Servo %d: %d deg = %d us"), _servoIndex, degree, _pulseLen);
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
  _pulseLen = microseconds;
  _degree = map(_pulseLen, _minPw, _maxPw, _minDegree, _maxDegree);
  _tickCnt = 0;
  _dutyCnt = 0;
}

/*
  This method has to be called every 20 milliseconds to refresh or
  to set the servo position if the internal timer is not being used.
  If the internal timer is being used, it'll call this method quite
  more often.
*/
void ZServo::setServo() {
  if(_disabled)
    return;

  if(!_useTimer) {
    if(_degree != _lastDegree || millis() - _lastUpdate < 200) { // avoid jitter on servo by ignoring this call
  #if defined(__ESP32__)
      ledcWrite(SERVO_CHANNEL+_servoIndex, _pulseLen);
  #else
      digitalWrite(_pin, HIGH);
      delayMicroseconds(_pulseLen);
      digitalWrite(_pin, LOW);
  #endif
      _lastDegree = _degree;
    }
  }
  else {
    // this method increments with every call by 50 (uS) and when the _pulseLen is reached it'll
    // sets the output to low.
    // This way, blocking the whole CPU while the delay is being active as it's in the method above
    // isn't happening.
    if(_tickCnt != 0) {
      if(_tickCnt <= _pulseLen)
        setServoPin(HIGH);
      else
        setServoPin(LOW);
    }
    _tickCnt += 50;
    // reset tick counter after the duty cycle for the next cycle
    if(_tickCnt >= DUTY_CYCLE) {
      if(_maxCycles == 0 || ++_dutyCnt < _maxCycles)  // but no more cycles than defined to avoid jitter on the servo
        _tickCnt = 0;
    }
  }
}

void ZServo::setServoPin(int8_t state) {
  if(_pinState == state)
    return;
  digitalWrite(_pin, state);
  _pinState = state;
}

void isrServoTimerHandler() {
  #if defined(__HW_DEBUG__) && defined(DEBUG_PIN)
  // used for internal hardware debugging only
  //if(DEBUG_PIN != -1) digitalWrite(DEBUG_PIN, !digitalRead(DEBUG_PIN));
  #endif

  // call all handlers for all servos periodically if the
  // internal timer is being used.
  for(int8_t i=0;  i< MAX_SERVOS; i++) {
    if(servoInstances[i] == nullptr || servoInstances[i]->isTimerStopped())
      continue;
    servoInstances[i]->setServo();
  }
}
