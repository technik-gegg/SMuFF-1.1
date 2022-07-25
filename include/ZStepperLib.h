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
#pragma once

#include <stdlib.h>
#include <Arduino.h>
#include "Config.h"
#include "HAL/HAL.h"

#include "Debug.h"

extern void defaultStepFunc(pin_t pin);

class ZStepper {
public:
    typedef enum {
      NONE = -1,
      MIN,                // Endstop is at position 0
      MAX,                // Endstop is at position >0
      ORBITAL,            // Endstop on rotating axis
      MINMAX              // either MIN or MAX
    } EndstopType;

    typedef enum {
      CW = 1,             // Clockwise
      CCW = -1            // Counter clockwise
    } MoveDirection;

  ZStepper();
  ZStepper(int8_t number, char* descriptor, pin_t stepPin, pin_t dirPin, pin_t enablePin, int16_t accelaration, int16_t maxSpeed);

  void          prepareMovement(long steps, bool ignoreEndstop = false);
  bool          handleISR();
  void          home();
  void          stallDetected() { _stallCount++; }

  void          (*stepFunc)(pin_t) = defaultStepFunc;
  void          (*endstopFunc)() = nullptr;
  void          (*endstop2Func)() = nullptr;
  bool          (*endstopCheck)() = nullptr;
  bool          (*stallCheck)() = nullptr;
  void          (*runAndWaitFunc)(int8_t number) = nullptr;
  void          (*runNoWaitFunc)(int8_t number) = nullptr;

  char*         getDescriptor() { return _descriptor; }
  void          setDescriptor(char* descriptor) { _descriptor = descriptor; }
  MoveDirection getDirection() { return _dir; }
  void          setDirection(MoveDirection newDir);
  bool          getEnabled() { return _enabled; }
  void          setEnabled(bool state);
  void          setEndstop(pin_t pin, int8_t triggerState, EndstopType type, int8_t index=1, void (*endstopIsrFunc)() = nullptr);
  EndstopType   getEndstopType() { return _endstopType; }
  void          setEndstopType(EndstopType type) { _endstopType = type; }
  int8_t        getEndstopState(uint8_t index = 1) { return index == 1 ? _endstopState : _endstopState2; }
  void          setEndstopState(uint8_t state, uint8_t index = 1) { index == 1 ? _endstopState = state : _endstopState2 = state; }
  bool          getEndstopHit(int8_t index = 1);
  bool          getEndstopHitAlt(int8_t index = 1) { return index == 1 ? _endstopHit : _endstopHit2; }
  void          setEndstopHit(int8_t state, int8_t index = 1) { if(index == 1) _endstopHit = state; else _endstopHit2 = state; }
  pin_t         getEndstopPin(int8_t index = 1) { if(index == 1) return _endstopPin; else return _endstopPin2; }
  void          setEndstopPin(pin_t pin, int8_t index = 1) { if(index == 1) _endstopPin = pin; else _endstopPin2 = pin; }
  bool          getIgnoreEndstop() { return _ignoreEndstop; }
  void          setIgnoreEndstop(bool state) { _ignoreEndstop = state; }

  long          getStepCount() { return _stepCount; }
  void          setStepCount(long count) { _stepCount = count; }
  long          getMaxStepCount() { return _maxStepCount; }
  void          setMaxStepCount(long count) { _maxStepCount = count; }
  void          incrementStepCount() { _stepCount++; }
  long          getTotalSteps() { return _totalSteps; }
  void          setTotalSteps(long count) { _totalSteps = count; }
  long          getStepPosition() { return _stepPosition; }
  void          setStepPosition(long position) { _stepPosition = position; _stepPositionMM = (double)((double)_stepPosition / _stepsPerMM); }
  double        getStepPositionMM() { return _stepPositionMM; }
  void          setStepPositionMM(double position) { _stepPositionMM = position; _stepPosition = (long)(position * _stepsPerMM);}
  void          incrementStepPosition() { _stepPosition += _dir; }
  bool          getMovementDone() { return _movementDone; }
  void          setMovementDone(bool state);
  uint16_t      getAcceleration() { return _acceleration; }
  void          setAcceleration(uint16_t value) { _acceleration = value; }
  uint16_t      getMaxSpeed() { return _maxSpeed; }
  void          setMaxSpeed(uint16_t value) { _maxSpeed = value; }
  uint16_t      getMaxHSpeed() { return _maxSpeedHS; }
  void          setMaxHSpeed(uint16_t value) { _maxSpeedHS = value; }
  bool          getInvertDir() { return _invertDir; }
  void          setInvertDir(bool state) { _invertDir = state; }

  timerVal_t    getDuration() { return _durationInt; }
  void          setDuration(timerVal_t value) { _durationInt = value; }
  void          subtractDuration(timerVal_t value) { _durationInt -= value; }
  bool          isDuration(timerVal_t value) { return _durationInt == value; }
  bool          isLTDuration(timerVal_t* value) { if(_durationInt < *value) { *value = _durationInt; return true; } return false; }
  uint16_t      getStepsPerMM() { return _stepsPerMM; }
  void          setStepsPerMM(uint16_t steps) { _stepsPerMM = steps; }
  double        getStepsPerDegree() { return _stepsPerDegree; }
  void          setStepsPerDegree(double steps) { _stepsPerDegree = steps; }

  bool          getAllowAccel() { return _allowAcceleration; }
  void          setAllowAccel(bool state) { _allowAcceleration = state; }
  bool          getAbort() { return _abort; }
  void          setAbort(bool state) { _abort = state; }
  bool          getIgnoreAbort() { return _ignoreAbort; }
  void          setIgnoreAbort(bool state) { _ignoreAbort = state; }
  long          getStepsTaken() { return _stepsTaken; }
  double        getStepsTakenMM() { return (double)_stepsTaken / _stepsPerMM; }
  void          setStepsTaken(long count) { _stepsTaken = count; }
  uint8_t       getAccelDistance() { return _accelDistance; }
  void          setAccelDistance(uint8_t dist) { _accelDistance = dist; }
  bool          getStallDetected() { return _stallDetected; }
  void          resetStallDetected() { _stallDetected = false; }
  bool          getStopOnStallDetected() { return _stopOnStallDetected; }
  void          setStopOnStallDetected(bool state) { _stopOnStallDetected = state; }
  uint8_t       getStallThreshold() { return _stallCountThreshold; }
  void          setStallThreshold(uint8_t max) { _stallCountThreshold = max; }
  uint32_t      getStallCount() { return _stallCount; }
  void          adjustPositionOnEndstop(int8_t index = 1);
  void          timerTrigger() { _timerTrigger--; }
  bool          isTimerTriggered(bool autoTrigger) { if(autoTrigger && _timerTrigger > 0) _timerTrigger--; return _timerTrigger == 0; } 
  void          resetTimerTrigger() { _timerTrigger = _durationInt; } 
  void          setInterruptFactor(timerVal_t factor) { _intrFactor = factor; }
  timerVal_t    getInterruptFactor() { return _intrFactor; }
  bool          checkInterrupt() { return (_intrFactor > 0) ? (++_intrCount % _intrFactor) == 0 : true; }
  bool          checkInterrupt2() { return (_intrFactor > 0) ? (_intrCount % _intrFactor) == 0 : true; }
  void          dumpParams();                         // for debugging purposes

private:
  uint8_t               _number = 0;                  // index of this stepper
  char*                 _descriptor = (char*)"";      // display name for this stepper
  pin_t                 _stepPin = 0;                 // stepping pin
  pin_t                 _dirPin = 0;                  // direction pin
  pin_t                 _enablePin = 0;               // enable pin
  bool                  _enabled = false;             // enabled state
  pin_t                 _endstopPin = 0;              // endstop pin
  pin_t                 _endstopPin2 = 0;             // 2nd endstop
  volatile bool         _endstopHit = false;          // set when endstop is being triggered
  volatile bool         _endstopHit2 = false;         // set when 2nd endstop is being triggered
  bool                  _ignoreEndstop = false;       // flag whether or not to ignore endstop trigger
  uint8_t               _endstopState = HIGH;         // value for endstop triggered
  EndstopType           _endstopType = NONE;          // type of endstop (MIN, MAX, ORBITAL etc)
  uint8_t               _endstopState2 = HIGH;        // value for 2nd endstop triggered
  EndstopType           _endstopType2 = NONE;         // type of 2nd endstop (MIN, MAX, ORBITAL etc)
  volatile long         _stepPosition = 0;            // current position of stepper (total of all movements taken so far)
  volatile MoveDirection _dir = CW;                   // current direction of movement, used to keep track of position
  volatile long         _totalSteps = 0;              // number of steps requested for current movement
  volatile bool         _movementDone = true;         // true if the current movement has been completed (used by main program to wait for completion)
  volatile timerVal_t   _acceleration = 1000;         // acceleration value
  uint8_t               _accelDistance = 5;           // distance (in millimeter or degree) need to be used for acceleration/deceleration
  timerVal_t            _maxSpeed = 100;              // ie. max speed, smaller is faster
  timerVal_t            _maxSpeedHS = 10;             // ie. max speed (HighSpeed mode), smaller is faster
  volatile long         _stepCount = 0;               // number of steps completed in current movement
  volatile long         _maxStepCount = 0;            // maximum number of steps
  uint16_t              _stepsPerMM = 0;              // steps needed for one millimeter
  double                _stepsPerDegree = 0;          // steps needed for 1 degree on orbital motion
  double                _stepPositionMM = 0;          // current position of stepper in millimeter
  bool                  _invertDir = false;           // stepper direction inversion
  bool                  _allowAcceleration = true;    // allow / disallow acceleration
  volatile bool         _abort = false;               // flag signals abortion of operation
  volatile bool         _ignoreAbort = false;         // flag signals abort not possible
  volatile long         _stepsTaken = 0;              // counter for steps currently taken
  bool                  _stallDetected = false;       // flag for TMC2209 drivers
  bool                  _stopOnStallDetected = false; // flag for TMC2209 drivers
  volatile uint32_t     _stallCount = 0;              // counter for stalls detected
  uint8_t               _stallCountThreshold = 5;     // threshold for serious stall detection
  volatile timerVal_t   _timerTrigger = 0;
  volatile timerVal_t   _intrFactor = 0;              // factor of which interrupt number to process for syncronous stepping
  volatile timerVal_t   _intrCount = 0;

  // per iteration variables (potentially changing every interrupt)
  volatile double         _durationF;                 // current interval length as double for accel/decel
  volatile timerVal_t     _durationInt;               // current interval length
  volatile long           _accelDistSteps = 0;        // amount of steps for acceleration
  volatile long           _decelDistPos = 0;          // step position for deceleration
  volatile double         _stepsAcceleration = 0;     // increment for acceleration / deceleration

  void resetStepper();                                // method to reset work params
  void updateAcceleration();                          // method to calculate and update acceleration / deceleration
};
