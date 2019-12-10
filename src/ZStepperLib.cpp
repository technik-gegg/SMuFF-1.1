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

ZStepper::ZStepper(int number, char* descriptor, int stepPin, int dirPin, int enablePin, unsigned int acceleration, unsigned int minStepInterval) {
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
  //_stepsTaken = 0;
  _movementDone = false;
  _endstopHit = false;
}

void ZStepper::prepareMovement(long steps, boolean ignoreEndstop /*= false */) {
  setDirection(steps < 0 ? CCW : CW);
  _totalSteps = abs(steps);
  _accelDistSteps = _endstopType == ORBITAL ? _stepsPerDegree * _accelDistance : _stepsPerMM * _accelDistance; 
  _stepsAcceleration = (float)((_acceleration - _minStepInterval)+.1) / _accelDistSteps;
  //__debug(PSTR("total: %ld  _accelDist: %ld  _stepsAccel: %s"), _totalSteps, _accelDistSteps, String(_stepsAcceleration).c_str());
  _ignoreEndstop = ignoreEndstop;
  resetStepper();
}

void ZStepper::setEndstop(int pin, int triggerState, EndstopType type, int index) {
  if(index == 1) {
    _endstopPin = pin;
    _endstopState = triggerState;
    _endstopType = type;
    if(pin != -1) {
      pinMode(_endstopPin, ((triggerState == 0) ? INPUT_PULLUP : INPUT));
      _endstopHit = digitalRead(_endstopPin) == _endstopState;
    }
  }
  else if(index == 2) {
    _endstopPin2 = pin;
    _endstopState2 = triggerState;
    _endstopType2 = type;
    if(pin != -1) {
      pinMode(_endstopPin2, ((triggerState == 0) ? INPUT_PULLUP : INPUT));
      _endstopHit2 = digitalRead(_endstopPin2) == _endstopState2;
    }
  }
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
  if(_stepCount <= _accelDistSteps) {
    _duration -= _stepsAcceleration;    // accelerate
  }
  if (_stepCount >= _totalSteps - _accelDistSteps ) {
    _duration += _stepsAcceleration;    // decelerate
  }
  if(_duration <= _minStepInterval)
    _duration = _minStepInterval;
  if(_duration >= _acceleration)
    _duration = _acceleration;
  _durationInt = _duration;
}

void ZStepper::handleISR() {

  bool hit;
  if((_endstopType == MIN && _dir == CCW) ||
     (_endstopType == MAX && _dir == CW) ||
     (_endstopType == ORBITAL)) {
     if(_endstopPin != -1) {
      hit = (int)digitalRead(_endstopPin)==_endstopState;
     }
     else {
      if(endstopCheck != NULL)
        hit = endstopCheck();
    }
    setEndstopHit(hit);
  }
  if(_endstopType2 != NONE) {
     if(_endstopPin2 != -1) {
      hit = (int)digitalRead(_endstopPin2)==_endstopState2;
     }
    setEndstopHit(hit, 2);
    if(endstop2Func != NULL)
      endstop2Func();
  }
  if(!_ignoreAbort && _abort) {
    setMovementDone(true);
    return;
  }
  if(!_ignoreEndstop && _endstopHit && !_movementDone){
    switch(_endstopType) {
      case MIN:
        setStepPosition(0);
        break;
      case MAX:
        setStepPosition(_maxStepCount);
        break;
      case ORBITAL:
        setStepPosition(0);
        break;
      default: 
        break;
    }
    setMovementDone(true);
    if(endstopFunc != NULL)
      endstopFunc();
    return;
  }
  
  if(_maxStepCount != 0 && _dir == CW && _stepCount >= _maxStepCount) {
    setMovementDone(true);
    //__debug(PSTR("Movement done: steps: %d - max: %d"), _stepCount, _maxStepCount);
  }
  else if(_stepCount < _totalSteps) {
    if(stepFunc != NULL)
      stepFunc();
    else
      defaultStepFunc();
    
    _stepCount++;
    _stepsTaken += _dir;    
    setStepPosition(getStepPosition() + _dir);
    
    if(_endstopType == ORBITAL) {
      if(getStepPosition() >= _maxStepCount) {
        //__debug(PSTR("Pos > Max: %d"), getStepPosition());
        setStepPosition(0);
      }
      else if(getStepPosition() < 0) {
        //__debug(PSTR("Pos < 0: %d"), getStepPosition());
        setStepPosition(_maxStepCount-1);
      }
    }
    if(_stepCount >= _totalSteps) {
      setMovementDone(true);
      //__debug(PSTR("handleISR() done: %ld / %ld / %ld"), _stepCount, _totalSteps, getStepPosition());
    }
  }
  updateAcceleration();
}

bool ZStepper::getEndstopHit(int index) {
  int stat = 0;
  if(index == 1) {
    if(_endstopPin != -1) {
      for(int i=0; i < 5;  i++)
        stat = digitalRead(_endstopPin);
      setEndstopHit(stat==_endstopState);
    }
    else {
      if(endstopCheck != NULL)
        setEndstopHit(endstopCheck());
      else
        setEndstopHit(false);
    }
    return _endstopHit;
  }
  if(index == 2) {
    if(_endstopPin2 != -1) {
      for(int i=0; i < 5;  i++)
        stat = digitalRead(_endstopPin2);
      setEndstopHit(stat==_endstopState2, 2);
    }
    return _endstopHit2;
  }
  return false;
}

void ZStepper::home() {
  
  // calculate the movement distances: if an endstop is set, go 20% beyond max.
  long distance = (_endstopPin != -1) ? -((long)((float)_maxStepCount*1.2)) : -_maxStepCount;
  long distanceF = 0;
  long back = -(distance/36);
  // if the endstop type is ORBITAL (Revolver) and the current position is beyond the middle, turn inverse
  if(_endstopType == ORBITAL && (getStepPosition() >= _maxStepCount/2 && getStepPosition() <= _maxStepCount)) {
    distanceF = abs(distance);
  }
  //__debug(PSTR("[ZStepper::home] Distance: %d - max: %d - back: %d"), distance, _maxStepCount, back);

  // only if the endstop is not being hit already, move to endstop position
  if(!_endstopHit) {
    prepareMovement(distanceF == 0 ? distance : distanceF);
    if(runAndWaitFunc != NULL)
      runAndWaitFunc(_number);
  }
  //else __debug(PSTR("[ZStepper::home] Endstop already hit"));

  // turn down the speed for more precision
  unsigned int curSpeed = getMaxSpeed();
  setMaxSpeed(getAcceleration());
  // go out of the endstop
  do {
    prepareMovement(back, true); // move forward by ignoring the endstop
    if(runAndWaitFunc != NULL)
      runAndWaitFunc(_number);
  } while(_endstopHit);
  // and back to home position
  prepareMovement(distance); 
  if(runAndWaitFunc != NULL)
    runAndWaitFunc(_number);
  
  // reset the speed
  setMaxSpeed(curSpeed);
}
