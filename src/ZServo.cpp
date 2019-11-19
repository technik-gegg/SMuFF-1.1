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

void ZServo::attach(int pin, bool useTimer, int servoIndex) { 
  _useTimer = useTimer;
  attach(pin);
  
  // To set up an independent timer, you have to use the attach()-method above
  // with useTimer = true. Otherwise you'll have to call setServo() in a 20 ms
  // period to get the servo running, which is a bit more complex but saves 
  // on timers.  
  if(!timerSet && _useTimer) {
    timerSet = true;
#ifdef __STM32F1__
    servoTimer.setupTimer(ZTimer::ZTIMER5, 1800);   // equals to 50 us on 72 MHz CPU
#else
    servoTimer.setupTimer(ZTimer::ZTIMER5, ZTimer::PRESCALER1024);
#endif
  }
  if(servoIndex != -1) {
    setIndex(servoIndex);
  }
  if(_useTimer) {
    servoTimer.setupTimerHook(isrServoTimerHandler);
    servoTimer.setNextInterruptInterval(1);
    //__debug(PSTR("Servo Timer initialized"));
  }
}

void ZServo::attach(int pin) { 
  _pin = pin;
  pinMode(_pin, OUTPUT); 
  digitalWrite(_pin, 0);
#ifdef __STM32F1__
  _pin_reg = &((PIN_MAP[_pin].gpio_device)->regs->BSRR);
  _pin_set = BIT(PIN_MAP[_pin].gpio_bit);
  _pin_reset = BIT(PIN_MAP[_pin].gpio_bit) << 16;
#endif
}

void ZServo::detach() {
  _pin = 0;
  servoInstances[_servoIndex] = NULL;
}

/*
  Main method to set the servo position.
  Values between 0 and 180 will be treated as degrees.
  Values above 180 will be treated as milliseconds (usually in the range of 500-2400 us).
*/
void ZServo::write(int val) {
  if(val >= 0 && val <= 180)
    setServoPos(val);
  else
    setServoMS(val);
}

void ZServo::writeMicroseconds(int microseconds) {
  setServoMS(microseconds);
}

void ZServo::setIndex(int servoIndex) { 
  if(servoIndex != -1 && servoIndex < MAX_SERVOS) {
    _servoIndex = servoIndex;
    servoInstances[_servoIndex] = this;
  }
}

bool ZServo::setServoPos(int degree) {
  bool stat = false;
  if(_pin > 0) {
    _pulseLen = map(degree, _minDegree, _maxDegree, _minPw, _maxPw);
    stat = true;
    _degree = degree;
    _lastUpdate = millis();
    _tickCnt = 0;
    _dutyCnt = 0;
  }
  return stat;
}

void ZServo::setServoMS(int microseconds) {
  _pulseLen = microseconds;
  _degree = map(_pulseLen, _minPw, _maxPw, _minDegree, _maxDegree);
  _tickCnt = 0;
  _dutyCnt = 0;
}

/*
  This method has to be called every 20 milliseconds to refresh or 
  to set the servo position.
  This might be called internally (via timer interrupt) or externally.
  In latter case, make sure you have the interval set correctly.
*/
void ZServo::setServo() {
  if(!_useTimer) {
    if(_degree != _lastDegree || millis() - _lastUpdate < 200) { // avoid jitter on servo by ignoring this call 
  #ifndef __STM32F1__    
      digitalWrite(_pin, HIGH);
      delayMicroseconds(_pulseLen);
      digitalWrite(_pin, LOW);
  #else
      // use the direct write (Bit set reset) method on STM32
      *_pin_reg = _pin_set;
      delay_us(_pulseLen);
      *_pin_reg = _pin_reset;
#endif
      _lastDegree = _degree;
    }
  }
  else {
    // this method increments every 50 us and when the _pulseLen is reached 
    // it sets the output to low.
    // Using this method will prevent blocking the whole CPU while the delay 
    // is being active, as it's in the method above.
    // It though is less accurate but will do its job on a servo.
    _tickCnt += 50;
#ifdef __STM32F1__
    if(_tickCnt < (uint32)_pulseLen)
      *_pin_reg = _pin_set;
    else
      *_pin_reg = _pin_reset;
#else
    if(_tickCnt < _pulseLen)
      digitalWrite(_pin, HIGH);
    else
      digitalWrite(_pin, LOW);
#endif
    if(_tickCnt >= DUTY_CYCLE) {    // restart the duty cycle
      if(_maxCycles == 0 || ++_dutyCnt < _maxCycles)   // but no more cycles than defined to avoid jitter on the servo
        _tickCnt = 0;
    }
  }
}

void isrServoTimerHandler() {
  // call all handlers for all servos periodically if the
  // internal timer is being used.
  for(int i=0;  i< MAX_SERVOS; i++) {
    if(servoInstances[i] != NULL)
      servoInstances[i]->setServo();
  }
}
