/**
 * SMuFF Firmware
 * Copyright (C) 2019-2024 Technik Gegg
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

#define __def2str(s) #s

extern void defaultStepFunc(pin_t pin, bool resetPin = false);

class ZStepper {
public:
    /**
     * @brief The type of endstop: NONE, MIN, MAX, ORBITAL or MINMAX.
     * 
     */
    typedef enum {
      NONE = -1,
      MIN,                // Endstop is at position 0
      MAX,                // Endstop is at position >0
      ORBITAL,            // Endstop on rotating axis
      MINMAX              // either MIN or MAX
    } EndstopType;

    /**
     * @brief The rotation direction of the stepper (CW or CCW).
     * 
     */
    typedef enum {
      CW = 1,             // ClockWise
      CCW = -1            // Counter ClockWise
    } MoveDirection;

  ZStepper();
  ZStepper(int8_t number, char* descriptor, pin_t stepPin, pin_t dirPin, pin_t enablePin, int16_t accelaration, int16_t maxSpeed);

  void          (*stepFunc)(pin_t pin, bool resetPin) = defaultStepFunc;
  void          (*endstopFunc)() = nullptr;
  void          (*endstop2Func)() = nullptr;
  bool          (*endstopCheck)() = nullptr;
  bool          (*stallCheck)() = nullptr;
  void          (*runAndWaitFunc)(int8_t number) = nullptr;
  void          (*runNoWaitFunc)(int8_t number) = nullptr;

  void          prepareMovement(long steps, bool ignoreEndstop = false);
  bool          handleISR();
  void          home();
  void          stallDetected();

  // getters/setters
  char*         getDescriptor();
  void          setDescriptor(char* descriptor);
  MoveDirection getDirection();
  void          setDirection(MoveDirection newDir);
  bool          getEnabled();
  void          setEnabled(bool state);
  EndstopType   getEndstopType();
  void          setEndstop(pin_t pin, int8_t triggerState, EndstopType type, int8_t index=1, void (*endstopIsrFunc)() = nullptr);
  void          setEndstopType(EndstopType type);
  uint8_t       getEndstopState(uint8_t index = 1);
  void          setEndstopState(uint8_t state, uint8_t index = 1);
  bool          getEndstopHit(int8_t index = 1);
  bool          getEndstopHitAlt(int8_t index = 1);
  void          setEndstopHit(int8_t state, int8_t index = 1);
  pin_t         getEndstopPin(int8_t index = 1);
  void          setEndstopPin(pin_t pin, int8_t index = 1);
  bool          getIgnoreEndstop();
  void          setIgnoreEndstop(bool state);
  long          getStepCount();
  void          setStepCount(long count);
  long          getMaxStepCount();
  void          setMaxStepCount(long count);
  long          getTotalSteps();
  void          setTotalSteps(long count);
  long          getStepPosition();
  void          setStepPosition(long position);
  double        getStepPositionMM();
  void          setStepPositionMM(double position);
  bool          getMovementDone();
  bool          setMovementDone(bool state);
  uint16_t      getAcceleration();
  void          setAcceleration(uint16_t value);
  uint16_t      getMaxSpeed();
  void          setMaxSpeed(uint16_t value);
  bool          getInvertDir();
  void          setInvertDir(bool state);
  timerVal_t    getDuration();
  bool          isLTDuration(timerVal_t* value);
  bool          isGTDuration(timerVal_t* value);
  uint16_t      getStepsPerMM();
  void          setStepsPerMM(uint16_t steps);
  double        getStepsPerDegree();
  void          setStepsPerDegree(double steps);
  bool          getAllowAccel();
  void          setAllowAccel(bool state);
  bool          getAbort();
  void          setAbort(bool state);
  bool          getIgnoreAbort();
  void          setIgnoreAbort(bool state);
  long          getStepsTaken();
  double        getStepsTakenMM();
  void          setStepsTaken(long count);
  uint8_t       getAccelDistance();
  void          setAccelDistance(uint8_t dist);
  bool          getStallDetected();
  void          resetStallDetected();
  bool          getStopOnStallDetected();
  void          setStopOnStallDetected(bool state);
  uint8_t       getStallThreshold();
  void          setStallThreshold(uint8_t max);
  uint32_t      getStallCount();
  void          setInterruptFactor(timerVal_t factor);
  timerVal_t    getInterruptFactor();
  bool          allowInterrupt();
  bool          evalEndstopHit(uint8_t index = 1);

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
  volatile long         _accelEvery = 0;              // accel/decel happens every n steps
  timerVal_t            _maxSpeed = 100;              // ie. max speed, smaller is faster
  timerVal_t            _maxSpeedHS = 10;             // ie. max speed (in HighSpeed mode), smaller is faster (not used yet)
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
  volatile timerVal_t   _intrFactor = 0;              // factor of which interrupt number to process for syncronous stepping
  volatile timerVal_t   _intrFactorCount = 0;         // interrupt factor counter for achieving a 'pseudo' sync movement
  volatile bool         _resetSignal = false;         // internal toggle for setting/resetting STEP signal

  // per iteration variables (potentially changing on every interrupt)
  volatile timerVal_t     _durationInt;               // current interval length
  volatile long           _accelDistSteps = 0;        // amount of steps for acceleration
  volatile long           _decelPos = 0;              // step position where deceleration starts

  void        resetStepper();
  void        dumpParams();                           // for debugging purposes only
  void        adjustPositionOnEndstop(int8_t index = 1);
  void        setDuration(timerVal_t value);
  inline void updateAcceleration();
};
