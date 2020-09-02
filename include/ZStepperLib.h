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

#ifndef _ZSTEPPER_H
#define _ZSTEPPER_H 1

extern void __debug(const char* fmt, ...);

class ZStepper {
public:
    typedef enum {
      NONE = -1,
      MIN,                // Endstop is at position 0
      MAX,                // Endstop is at position >0
      ORBITAL             // Endstop on rotating axis
    } EndstopType;

    typedef enum {
      CW = 1,             // Clockwise
      CCW = -1            // Counter clockwise
    } MoveDirection;

  ZStepper();
  ZStepper(int number, char* descriptor, int stepPin, int dirPin, int enablePin, unsigned int accelaration, unsigned int minStepInterval);

  void prepareMovement(long steps, bool ignoreEndstop = false);
  void handleISR();
  void home();
  void stallDetected() { _stallCount++; }

  void          (*stepFunc)() = NULL;
  void          (*endstopFunc)() = NULL;
  void          (*endstop2Func)() = NULL;
  bool          (*endstopCheck)() = NULL;
  bool          (*stallCheck)() = NULL;
  void          (*runAndWaitFunc)(int number) = NULL;
  void          (*runNoWaitFunc)(int number) = NULL;
  void          defaultStepFunc();                    // default step method, uses digitalWrite on _stepPin

  char*         getDescriptor() { return _descriptor; }
  void          setDescriptor(char* descriptor) { _descriptor = descriptor; }
  MoveDirection getDirection() { return _dir; }
  void          setDirection(MoveDirection newDir);
  bool          getEnabled() { return _enabled; }
  void          setEnabled(bool state);
  void          setEndstop(int pin, int triggerState, EndstopType type, int index=1);
  EndstopType   getEndstopType() { return _endstopType; }
  void          setEndstopType(EndstopType type) { _endstopType = type; }
  int           getEndstopState() { return _endstopState; }
  void          setEndstopState(int state) { _endstopState = state; }
  bool          getEndstopHit(int index = 1);
  bool          getEndstopHitAlt(int index = 1) { return index == 1 ? _endstopHit : _endstopHit2; }
  void          setEndstopHit(int state, int index = 1) { if(index == 1) _endstopHit = state; else _endstopHit2 = state; }
  int           getEndstopPin() { return _endstopPin; }
  void          setEndstopPin(int pin) { _endstopPin = pin; }
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
  void          setStepPosition(long position) { _stepPosition = position; _stepPositionMM = (float)((float)position / _stepsPerMM); }
  float         getStepPositionMM() { return _stepPositionMM; }
  void          setStepPositionMM(float position) { _stepPositionMM = position; _stepPosition = (long)(position * _stepsPerMM);}
  void          incrementStepPosition() { setStepPosition(getStepPosition() + _dir); }
  bool          getMovementDone() { return _movementDone; }
  void          setMovementDone(bool state) { _movementDone = state; }
  unsigned int  getAcceleration() { return _acceleration; }
  void          setAcceleration(unsigned int value) { _acceleration = value; }
  unsigned int  getMaxSpeed() { return _minStepInterval; }
  void          setMaxSpeed(unsigned int value) { _minStepInterval = value; }
  unsigned int  getMaxHSpeed() { return _minStepIntervalHS; }
  void          setMaxHSpeed(unsigned int value) { _minStepIntervalHS = value; }
  bool          getInvertDir() { return _invertDir; }
  void          setInvertDir(bool state) { _invertDir = state; }

  unsigned int  getDuration() { return _durationInt; }
  void          setDuration(unsigned int value) { _durationInt = value; }
  unsigned int  getStepsPerMM() { return _stepsPerMM; }
  void          setStepsPerMM(int steps) { _stepsPerMM = steps; }
  float         getStepsPerDegree() { return _stepsPerDegree; }
  void          setStepsPerDegree(float steps) { _stepsPerDegree = steps; }

  bool          getAllowAccel() { return _allowAcceleration; }
  void          setAllowAccel(bool state) { _allowAcceleration = state; }
  bool          getAbort() { return _abort; }
  void          setAbort(bool state) { _abort = state; }
  bool          getIgnoreAbort() { return _ignoreAbort; }
  void          setIgnoreAbort(bool state) { _ignoreAbort = state; }
  long          getStepsTaken() { return _stepsTaken; }
  float         getStepsTakenMM() { return (float)_stepsTaken / _stepsPerMM; }
  void          setStepsTaken(long count) { _stepsTaken = count; }
  unsigned int  getAccelDistance() { return _accelDistance; }
  void          setAccelDistance(unsigned dist) { _accelDistance = dist; }
  bool          getStallDetected() { return _stallDetected; }
  void          resetStallDetected() { _stallDetected = false; }
  bool          getStopOnStallDetected() { return _stopOnStallDetected; }
  void          setStopOnStallDetected(bool state) { _stopOnStallDetected = state; }
  int           getStallThreshold() { return _stallCountThreshold; }
  void          setStallThreshold(int max) { _stallCountThreshold = max; }
  int           getStallCount() { return _stallCount; }

private:
  int             _number = 0;                  // index of this stepper
  char*           _descriptor = (char*)"";      // display name for this stepper
  int             _stepPin = -1;                // stepping pin
  int             _dirPin = -1;                 // direction pin
  int             _enablePin = -1;              // enable pin
  bool            _enabled = false;             // enabled state
  int             _endstopPin = -1;             // endstop pin
  int             _endstopPin2 = -1;            // 2nd endstop
  volatile bool   _endstopHit = false;          // set when endstop is being triggered
  volatile bool   _endstopHit2 = false;         // set when 2nd endstop is being triggered
  bool            _ignoreEndstop = false;       // flag whether or not to ignore endstop trigger
  int             _endstopState = HIGH;         // value for endstop triggered
  EndstopType     _endstopType = NONE;          // type of endstop (MIN, MAX, ORBITAL etc)
  int             _endstopState2 = HIGH;        // value for 2nd endstop triggered
  EndstopType     _endstopType2 = NONE;         // type of 2nd endstop (MIN, MAX, ORBITAL etc)
  volatile long   _stepPosition = 0;            // current position of stepper (total of all movements taken so far)
  volatile MoveDirection _dir = CW;             // current direction of movement, used to keep track of position
  volatile long   _totalSteps = 0;              // number of steps requested for current movement
  volatile bool   _movementDone = true;         // true if the current movement has been completed (used by main program to wait for completion)
  unsigned int    _acceleration = 1000;         // acceleration value 
  unsigned int    _accelDistance = 5;           // distance (in millimeter or degree) need to be used for acceleration/deceleration 
  unsigned int    _minStepInterval = 100;       // ie. max speed, smaller is faster
  unsigned int    _minStepIntervalHS = 10;      // ie. max speed (HighSpeed mode), smaller is faster
  long            _stepCount = 0;               // number of steps completed in current movement
  long            _maxStepCount = 0;            // maximum number of steps
  unsigned int    _stepsPerMM = 0;              // steps needed for one millimeter 
  float           _stepsPerDegree = 0;          // steps needed for 1 degree on orbital motion
  float           _stepPositionMM = 0;          // current position of stepper in millimeter
  bool            _invertDir = false;           // stepper direction inversion
  bool            _allowAcceleration = true;    // allow / disallow acceleration
  bool            _abort = false;               // flag signals abortion of operation  
  bool            _ignoreAbort = false;         // flag signals abort not possible
  long int        _stepsTaken = 0;              // counter for steps currently taken
  bool            _stallDetected = false;       // flag for TMC2209 drivers
  bool            _stopOnStallDetected = false; // flag for TMC2209 drivers
  int             _stallCount = 0;              // counter for stalls detected
  int             _stallCountThreshold = 5;     // threshold for serious stall detection

  // per iteration variables (potentially changed every interrupt)
  volatile float          _duration;            // current interval length
  volatile unsigned int   _durationInt;         // above variable truncated
  volatile long           _accelDistSteps = 0;  // amount of steps for acceleration/deceleration 
  volatile float          _stepsAcceleration  = 0.0;

  void resetStepper();                          // method to reset work params
  void updateAcceleration();
};

#endif
