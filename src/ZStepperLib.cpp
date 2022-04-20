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

 /*
  * Module implementing a stepper driver library
  */

#include "ZStepperLib.h"

#include "Debug.h"

extern void fastFlipDbg();

ZStepper::ZStepper() {

}

ZStepper::ZStepper(int8_t number, char* descriptor, pin_t stepPin, pin_t dirPin, pin_t enablePin, int16_t acceleration, int16_t maxSpeed) {
  _number = number;
  _descriptor = descriptor;
  _stepPin = stepPin;
  _dirPin = dirPin;
  _enablePin = enablePin;
  _acceleration = acceleration;
  _maxSpeed = maxSpeed;
  if(_stepPin > 0)
    pinMode(_stepPin,    OUTPUT);
  if(_dirPin > 0)
    pinMode(_dirPin,     OUTPUT);
  if(_enablePin > 0)
    pinMode(_enablePin,  OUTPUT);
}

void defaultStepFunc(pin_t pin) {
  if(pin > 0) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(1);
    digitalWrite(pin, LOW);
  }
}

void ZStepper::dumpParams() {
    __debugS(DEV, PSTR("total: %6lu  _accelDist: %5lu _decelDist: %5lu  _acceleration: %lu  _stepsAcceleration: %s   _maxSpeed: %lu\n      \t\t _durationInt: %lu  _timerTrigger: %lu  _intrFactor: %d  ignoreEndstop: %s  _allowAccel: %s"), 
            _totalSteps, 
            _accelDistSteps, 
            _decelDistPos, 
            _acceleration, 
            String(_stepsAcceleration).c_str(), 
            _maxSpeed, 
            _durationInt, 
            _timerTrigger,
            _intrFactor,
            _ignoreEndstop ? "Yes" : "No",
            _allowAcceleration ? "Yes" : "No");
}

void ZStepper::resetStepper() {
  _durationF = (float)_acceleration;
  _durationInt = _acceleration;
  _timerTrigger = _durationInt;
  _stepCount = 0;
  _movementDone = false;
  _endstopHit = false;
  _stallDetected = false;
  _stallCount = 0;
  _abort = false;
  _intrFactor = 0;
  _intrCount = 0;
  dumpParams();
}

void ZStepper::prepareMovement(long steps, bool ignoreEndstop /*= false */) {
  setDirection(steps < 0 ? CCW : CW);
  _totalSteps = abs(steps)+1;
  _accelDistSteps = _accelDistance * (_endstopType == ORBITAL ? _stepsPerDegree : _stepsPerMM);
  if(_accelDistSteps > _totalSteps) {
    //__debugS(D, PSTR("Accel. dist. > total dist."));
    _accelDistSteps = _totalSteps/2;
  }
  _decelDistPos = _totalSteps - _accelDistSteps;
  
  int32_t speedDiff = _acceleration - _maxSpeed;
  if(speedDiff <= 0)
    _stepsAcceleration = 0;
  else {
    _stepsAcceleration = (float)speedDiff / _accelDistSteps;
    if(_stepsAcceleration > _accelDistSteps)
      _stepsAcceleration = 0;
  }
  _ignoreEndstop = ignoreEndstop;
  resetStepper();
}

void ZStepper::setEndstop(pin_t pin, int8_t triggerState, EndstopType type, int8_t index, void(*endstopIsrFunc)()) {
  if(index == 1) {
    _endstopPin = pin;
    _endstopState = triggerState;
    _endstopType = type;
    if(pin > 0) {
      pinMode(_endstopPin, ((triggerState == 0) ? INPUT_PULLUP : INPUT));
      if(endstopIsrFunc != nullptr)
        attachInterrupt(digitalPinToInterrupt(_endstopPin), endstopIsrFunc, CHANGE);
      _endstopHit = (int)digitalRead(_endstopPin) == (int)_endstopState;
    }
  }
  else if(index == 2) {
    _endstopPin2 = pin;
    _endstopState2 = triggerState;
    _endstopType2 = type;
    if(pin != 0) {
      pinMode(_endstopPin2, ((triggerState == 0) ? INPUT_PULLUP : INPUT));
      if(endstopIsrFunc != nullptr)
        attachInterrupt(digitalPinToInterrupt(_endstopPin2), endstopIsrFunc, CHANGE);
      _endstopHit2 = (int)digitalRead(_endstopPin2) == (int)_endstopState2;
    }
  }
}

void ZStepper::setDirection(ZStepper::MoveDirection direction) {
  if(_dirPin != 0) {
    _dir = direction;
    digitalWrite(_dirPin, !_invertDir ? (_dir == CCW ? HIGH : LOW) : (_dir == CCW ? LOW : HIGH));
    //__debugS(D, PSTR("Dir: %d"), !_invertDir ? (_dir == CCW ? HIGH : LOW) : (_dir == CCW ? LOW : HIGH));
  }
}

void ZStepper::setEnabled(bool state) {
  if(_enablePin != 0) {
    digitalWrite(_enablePin, state ? LOW : HIGH);
    _enabled = state;
  }
}

void ZStepper::updateAcceleration() {

  if(!_allowAcceleration || _stepsAcceleration == 0) {
    _durationInt  = _maxSpeed;
    return;
  }

  if(_stepCount <= _accelDistSteps) {
    _durationF -= _stepsAcceleration;         // accelerate (i.e. make the timer interval shorter)
    _durationInt = (timerVal_t)_durationF;
    if(_maxSpeed > _durationInt)
      _durationInt = _maxSpeed;
  }
  else if (_stepCount >= _decelDistPos) {
    _durationF += _stepsAcceleration;         // decelerate (i.e. make the timer interval longer)
    _durationInt = (timerVal_t)_durationF;
    if(_durationInt > _acceleration)
      _durationInt = _acceleration;
  }
  else {
    _durationInt = _maxSpeed;
  }
}

void ZStepper::adjustPositionOnEndstop(int8_t index) {

  if(index == 1) {
    if(!_ignoreEndstop && _endstopHit && !_movementDone) {
      switch(_endstopType) {
        case MIN:
        case ORBITAL:
          _stepPosition = 0L;
          break;
        case MAX:
          _stepPosition = _maxStepCount;
          break;
      }
    }
  }
  else {
    if(!_ignoreEndstop && _endstopHit2 && !_movementDone) {
      // not used yet, might be used in the future
    }
  }
}

void ZStepper::setMovementDone(bool state) {
  _movementDone = state;
  if(_movementDone)
    setStepPosition(_stepPosition); // update step position to get the position in mm right
  resetTimerTrigger();
}

bool ZStepper::handleISR() {
  if(_stepPin == 0) {      // just in case not step pin has been defined
    setMovementDone(true);  // abort handler
    return true;
  }
  
  // check for "Movement Done" conditions
  if((!_ignoreEndstop && _endstopHit && !_movementDone) ||
     (!_ignoreAbort && _abort)) {
    setMovementDone(true);
    return true;
  }
  updateAcceleration();

#ifdef HAS_TMC_SUPPORT
  // check stepper motor stall on TMC2209 if configured likewise
  if(_stallCountThreshold > 0 && _stallCount > _stallCountThreshold) {
    _stallDetected = true;
    if(_stopOnStallDetected) {
      setMovementDone(true);
      //__debugS(D, PSTR("Stop  on stall - stopped"));
      return true;
    }
  }
#endif

  if(_maxStepCount != 0 && _dir == CW && _stepCount >= _maxStepCount) {
    setMovementDone(true);
    //__debugS(D, PSTR("Movement done: steps: %d - max: %d"), _stepCount, _maxStepCount);
  }
  else if(_stepCount < _totalSteps) {
    stepFunc(_stepPin);
    _stepCount++;
    _stepsTaken += _dir;
    incrementStepPosition();
    if(_stepCount >= _totalSteps) {
      setMovementDone(true);
    }

    if(_endstopType == ORBITAL) {
      if(_stepPosition >= _maxStepCount) {
        _stepPosition = 0L;
      }
      else if(_stepPosition < 0) {
        _stepPosition = _maxStepCount-1;
      }
    }
  }
  return _movementDone;
}

bool ZStepper::getEndstopHit(int8_t index) {
  int8_t stat = 0;
  if(index == 1) {
    if(_endstopPin > 0) {
      stat = digitalRead(_endstopPin);
      setEndstopHit(stat==_endstopState);
    }
    else {
      if(endstopCheck != nullptr)
        setEndstopHit(endstopCheck()==_endstopState);
      else
        setEndstopHit(false);
    }
    return _endstopHit;
  }
  else if(index == 2) {
    if(_endstopPin2 > 0) {
      stat = digitalRead(_endstopPin2);
      setEndstopHit(stat==_endstopState2, 2);
    }
    else {
      setEndstopHit(false, 2);
    }
    return _endstopHit2;
  }
  return false;
}

void ZStepper::home() {
  // calculate the movement distances: if an endstop is set, go 20% beyond max.
  long distance = (_endstopPin > 0) ? -((long)((float)_maxStepCount*1.2)) : -_maxStepCount;
  long distanceF = 0;
  long back = -(distance/36);
  // if the endstop type is ORBITAL (Revolver) and the current position is beyond the middle, turn inverse
  if(_endstopType == ORBITAL && (getStepPosition() >= _maxStepCount/2 && getStepPosition() <= _maxStepCount)) {
    distanceF = abs(distance);
  }
  //__debugS(D, PSTR("[ZStepper::home] Distance: %d - max: %d - back: %d"), distance, _maxStepCount, back);

  // only if the endstop is not being hit already, move to endstop position
  if(!_endstopHit) {
    prepareMovement(distanceF == 0 ? distance : distanceF);
    if(runAndWaitFunc != nullptr)
      runAndWaitFunc(_number);
  }
  //else __debugS(I, PSTR("[ZStepper::home] Endstop already hit"));

  // turn down the speed for more precision
  uint16_t curSpeed = getMaxSpeed();
  setMaxSpeed(getAcceleration());
  // go out of the endstop
  do {
    prepareMovement(back, true); // move forward by ignoring the endstop
    if(runAndWaitFunc != nullptr)
      runAndWaitFunc(_number);
  } while(_endstopHit);
  // and back to home position
  prepareMovement(distance);
  if(runAndWaitFunc != nullptr)
    runAndWaitFunc(_number);

  // reset the speed
  setMaxSpeed(curSpeed);
}
