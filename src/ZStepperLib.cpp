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

 /*
  * Module implementing a stepper driver library
  */

#include "ZStepperLib.h"

extern void __debug(const char* fmt, ...);

ZStepper::ZStepper() {
  
}

ZStepper::ZStepper(int number, char* descriptor, int stepPin, int dirPin, int enablePin, float acceleration, unsigned int minStepInterval) {
  _number = number;
  _descriptor = descriptor;
  _stepPin = stepPin;
  _dirPin = dirPin;
  _enablePin = enablePin;
  _acceleration = acceleration;
  _minStepInterval = minStepInterval;
  pinMode(_stepPin,    OUTPUT);
  pinMode(_dirPin,     OUTPUT);
  pinMode(_enablePin,  OUTPUT);
}

void ZStepper::defaultStepFunc(void) {
  if(_stepPin != -1) {
    digitalWrite(_stepPin, HIGH);
    digitalWrite(_stepPin, LOW);
  }
}

void ZStepper::resetStepper() {
  _duration = _allowAcceleration ? _acceleration : _minStepInterval;
  _durationInt = _duration;
  _stepCount = 0;
  _movementDone = false;
  _endstopHit = false;
}

void ZStepper::prepareMovement(long steps, boolean ignoreEndstop /*= false */) {
  setDirection(steps < 0 ? CCW : CW);
  _totalSteps = abs(steps);
  _accelDistance = _totalSteps >> 5;
  _stepsAcceleration = (float)((_acceleration - _minStepInterval)+.1) / _accelDistance;
  _ignoreEndstop = ignoreEndstop;
  resetStepper();
}

void ZStepper::setEndstop(int pin, int triggerState, EndstopType type) {
  _endstopPin = pin;
  _endstopState = triggerState;
  _endstopType = type;
  pinMode(_endstopPin, ((triggerState == 0) ? INPUT_PULLUP : INPUT));
  _endstopHit = digitalRead(_endstopPin) == _endstopState;
}

void ZStepper::setDirection(ZStepper::MoveDirection direction) {
  if(_dirPin != -1) {
    _dir = direction;
    digitalWrite(_dirPin, !_invertDir ? (_dir == CCW ? HIGH : LOW) : (_dir == CCW ? LOW : HIGH));
  }
}

void ZStepper::setEnabled(boolean state) {
  if(_enablePin != -1) {
    digitalWrite(_enablePin, state ? LOW : HIGH);
    _enabled = state;
  }
}

void ZStepper::updateAcceleration() {
  
  if(!_allowAcceleration) {
    _durationInt = _duration = _minStepInterval;
    return;
  }
  if(_stepCount <= _accelDistance) {
    _duration -= _stepsAcceleration;    // accelerate
    if(_duration <= _minStepInterval)
      _duration = _minStepInterval;
    //__debug(PSTR("durInt: %d, %s"), _durationInt,  String(_duration).c_str());
  }
  else if (_stepCount >= _totalSteps - _accelDistance ) {
    _duration += _stepsAcceleration;    // decelerate
    if(_duration >= _acceleration)
      _duration = _acceleration;
  }
  _durationInt = _duration;
}

void ZStepper::handleISR() {

  //if(_endstopType == ORBITAL)
  //  __debug(PSTR("O: %d %d "), _stepCount, _dir);
  if((_endstopType == MIN && _dir == CCW) ||
     (_endstopType == MAX && _dir == CW) ||
     (_endstopType == ORBITAL && _dir == CCW)) {
     bool hit = (int)digitalRead(_endstopPin)==_endstopState;
    setEndstopHit(hit);
  }
  if(!_ignoreAbort && _abort) {
    setMovementDone(true);
    return;
  }
  if(!_ignoreEndstop && _endstopHit && !_movementDone){
      setMovementDone(true);
      if(_endstopType == MIN || _endstopType == ORBITAL) {
        setStepPosition(0);
      }
      else if(_endstopType == MAX)
        setStepPosition(_maxStepCount);

    if(endstopFunc != NULL)
      endstopFunc();
    return;
  }
  
  if(_ignoreEndstop && _endstopHit && _endstopType == ORBITAL){
    setStepPosition(0);
  }
  
  if(_maxStepCount != 0 && _dir == CW && _stepCount >= _maxStepCount) {
    setMovementDone(true);
  }
  else if(_stepCount < _totalSteps) {
    if(stepFunc != NULL)
      stepFunc();
     else
      defaultStepFunc();
    _stepCount++;
    _stepsTaken += _dir;    
    setStepPosition(_stepPosition + _dir);
    if(_stepCount >= _totalSteps) {
      setMovementDone(true);
      //__debug(PSTR("handleISR(): %ld / %ld"), _stepCount, _totalSteps);
    }
  }
  updateAcceleration();
}

bool ZStepper::getEndstopHit() {
  int stat = 0;
  for(int i=0; i < 5;  i++)
    stat = digitalRead(_endstopPin);
  setEndstopHit(stat==_endstopState);
  return _endstopHit; 
}

void ZStepper::home() {

  unsigned int curSpeed = getMaxSpeed();
  long distance = -_maxStepCount;
  if(_endstopPin != -1) {
    distance = -(_maxStepCount*2);
  }
  //__debug(PSTR("[ZStepper::home] Distance: %d -  max: %d"), distance, _maxStepCount);
  prepareMovement(distance);
  //__debug(PSTR("[ZStepper::home] DONE prepareMovement"));
  if(runAndWaitFunc != NULL)
    runAndWaitFunc(_number);
    //__debug(PSTR("[ZStepper::home] DONE runAndWait"));
  do {
    prepareMovement(_maxStepCount/30);
    if(runAndWaitFunc != NULL)
      runAndWaitFunc(_number);
  } while(_endstopHit);
  prepareMovement(-(_maxStepCount));
  setMaxSpeed(curSpeed*25);
  if(runAndWaitFunc != NULL)
    runAndWaitFunc(_number);
  setMaxSpeed(curSpeed);
}
