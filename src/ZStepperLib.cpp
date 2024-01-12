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

 /*
  * Module implementing a stepper driver library.
  */

#include "ZStepperLib.h"

#include "Debug.h"

/**
 * @brief Default constructor. 
 * @note Not being used. Use initializing constructor instead.
*/
ZStepper::ZStepper() {
}

/**
 * @brief Initializing constructor. Creates a new instance with the given parameters. If all pins are set to 0,
 *        a "dummy" stepper is being created, which serves no other purpose than to avoid accessing uninitlialized objects.
 * 
 * @param number        The ordinal number.
 * @param descriptor    The human readable name for this instance (i.e. "Z-Axis")
 * @param stepPin       The pin on the MCU which is attached to the STEP input pin on the stepper driver.
 * @param dirPin        The pin on the MCU which is attached to the DIR input pin on the stepper driver.
 * @param enablePin     The pin on the MCU which is attached to the ENABLE input pin on the stepper driver.
 * @param acceleration  The starting acceleration value (in ticks).
 * @param maxSpeed      The maximum speed value (in ticks).
 * 
 * @note This library always uses timer ticks for speed values.
 */
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
  __debugS(DEV2, PSTR("\tStepper '%-8s' stepPin: %4d   dirPin: %4d  enablePin: %4d"), _descriptor, _stepPin, _dirPin, _enablePin);
}

/**
 * @brief Default step function which takes place if no custom function has been assigned.
 * 
 * @param pin       The pin on the MCU where the STEP of the driver is attached to.
 * @param resetPin  Flag, whether to set or reset the pin.
 */
void defaultStepFunc(pin_t pin, bool resetPin) {
  if(pin == 0) 
    return;
  if(!resetPin)
    digitalWrite(pin, HIGH);
  else
    digitalWrite(pin, LOW);
}

/**
 * @brief Dumps current runtime after resetStepper() values for debugging purpose.
 * 
 */
void ZStepper::dumpParams() {
  __debugS(DEV3, PSTR("\tStepper '%-8s':"), _descriptor);
  __debugS(DEV3, PSTR("   _totalSteps:   %-6lu   _accelDist:     %-5lu   _decelPos:    %-6lu   _accelEvery: %-5ld"),
          _totalSteps, 
          _accelDistSteps, 
          _decelPos, 
          _accelEvery);
  __debugS(DEV3, PSTR("   _acceleration: %-6lu   _maxSpeed:      %-5lu   _durationInt: %-6lu   _allowAccel: %-5s"), 
          _acceleration, 
          _maxSpeed, 
          _durationInt,
          _allowAcceleration ? PSTR("Yes") : PSTR("No"));
  __debugS(DEV3, PSTR("   _intrFactor:   %-6d   _ignoreEndstop: %-5s    "), 
          _intrFactor,
          _ignoreEndstop ? PSTR("Yes") : PSTR("No"));
}

/**
 * @brief Set runtime values to default before the next movement is being executed.
 * 
 * @see prepareMovement()
 */
void ZStepper::resetStepper() {
  _durationInt = _acceleration;
  _stepCount = 0;
  _stallCount = 0;
  _intrFactor = 0;
  _intrFactorCount = 0;
  _movementDone = false;
  _endstopHit = false;
  _stallDetected = false;
  _abort = false;
  _resetSignal = false;
  dumpParams();
}

/**
 * @brief Prepares the next stepper movement by initialising runtime values.
 *        This method is the main function that needs to be called before any movement is being initiated.
 * 
 * @param steps         The number of steps to move.
 * @param ignoreEndstop Flag whether or not to ignore the endstop trigger.
 *
 */
void ZStepper::prepareMovement(long steps, bool ignoreEndstop /*= false */) {
  setDirection(steps < 0 ? CCW : CW);
  _totalSteps = abs(steps)+1;
  // calculate acceleration distance
  if(_accelDistance == 0)
    _accelDistSteps = (long)_totalSteps*0.2;  // set acceleration steps to 20% of total distance
  else
    _accelDistSteps = _accelDistance * (_endstopType == ORBITAL ? _stepsPerDegree : _stepsPerMM);

  if(_accelDistSteps > _totalSteps)
    _accelDistSteps = _totalSteps/2;
  // set deleceration to same amount of steps as the acceleration
  _decelPos = _totalSteps - _accelDistSteps;
  _accelEvery = _accelDistSteps / (_acceleration - _maxSpeed);
  _ignoreEndstop = ignoreEndstop;
  resetStepper();
}

/**
 * @brief Increment the stall count.
 * 
 */
void ZStepper::stallDetected() { _stallCount++; }

/**
 * @brief Set the descriptor of this instance.
 * 
 * @param descriptor  The human readable name for this instance.
 */
void ZStepper::setDescriptor(char* descriptor) { _descriptor = descriptor; }

/**
 * @brief Get the descriptor of this instance.
 * 
 * @returns The currently set human readable name of this instance.
 */
char* ZStepper::getDescriptor() { return _descriptor; }

/**
 * @brief Sets the moving direction.
 * 
 * @param direction   The direction (CW or CCW) in that the stepper motor shall move.
 * 
 * @see MoveDirection enumeration
 */
void ZStepper::setDirection(ZStepper::MoveDirection direction) {
  if(_dirPin != 0) {
    _dir = direction;
    uint32_t val = !_invertDir ? (_dir == CCW ? HIGH : LOW) : (_dir == CCW ? LOW : HIGH);
    digitalWrite(_dirPin, val);
    __debugS(DEV2, PSTR("\tStepper '%-8s' direction: %s"), _descriptor, _dir==CW ? "CW" : "CCW");
  }
}

/**
 * @brief Get the Moving Direction.
 * 
 * @returns The current stepper moving direction (CW/CCW).
 
 * @see MoveDirection enumeration
 */
ZStepper::MoveDirection ZStepper::getDirection() {
  return _dir;
}

/**
 * @brief Set the enable signal.
 * 
 * @param state   true to enable this stepper, false to disable it.
 */
void ZStepper::setEnabled(bool state) {
  if(_enablePin != 0) {
    digitalWrite(_enablePin, state ? LOW : HIGH);
    _enabled = state;
    __debugS(DEV2, PSTR("\tStepper '%-8s' %s"), _descriptor, _enabled ? PSTR("enabled") : PSTR("disabled"));
  }
}

/**
 * @brief Get the enabled state of this stepper.
 * 
 * @returns true   when this stepper is enabled.
 * @returns false  when this stepper is not enabled.
 */
bool ZStepper::getEnabled() {
  return _enabled;
}

/**
 * @brief Setup the endstop pin(s).
 * 
 * @param pin           The pin number on the MCU.
 * @param triggerState  The signal level (HIGH/LOW) which indicates the 'triggered' state.
 * @param type          The type of the endstop
 * @param index         The index of the endstop to configure (1/2)
 * @param func          The ISR function to which to attach the interrupt. If this parameter is nullptr, no interrupt will be attached.
 *
 * @see EndstopType enumeration
 */
void ZStepper::setEndstop(pin_t pin, int8_t triggerState, EndstopType type, int8_t index, void(*endstopIsrFunc)()) {
  if(index == 1) {
    _endstopPin = pin;
    _endstopState = triggerState;
    _endstopType = type;
    if(pin != 0) {
      pinMode(_endstopPin, ((triggerState == 0) ? INPUT_PULLUP : INPUT));
      if(endstopIsrFunc != nullptr) {
        attachInterrupt(digitalPinToInterrupt(_endstopPin), endstopIsrFunc, CHANGE);
        __debugS(DEV2, PSTR("\tStepper '%-8s' endstop %d ISR set for pin: %d"), _descriptor, index, _endstopPin);
      }
      _endstopHit = (bool)digitalRead(_endstopPin) == _endstopState;
    }
  }
  else if(index == 2) {
    _endstopPin2 = pin;
    _endstopState2 = triggerState;
    _endstopType2 = type;
    if(pin != 0) {
      pinMode(_endstopPin2, ((triggerState == 0) ? INPUT_PULLUP : INPUT));
      if(endstopIsrFunc != nullptr) {
        attachInterrupt(digitalPinToInterrupt(_endstopPin2), endstopIsrFunc, CHANGE);
        __debugS(DEV2, PSTR("\tStepper '%-8s' endstop %d ISR set for pin: %d"), _descriptor, index, _endstopPin2);
      }
      _endstopHit2 = (bool)digitalRead(_endstopPin2) == _endstopState2;
    }
  }
}

/**
 * @brief Set the Endstop Type.
 * 
 * @param type  The type of this endstop.
 * 
 * @note EnstopType enum
 */
void ZStepper::setEndstopType(ZStepper::EndstopType type) {
  _endstopType = type;
}

/**
 * @brief Get the Endstop Type.
 * 
 * @returns The currently set type of this endstop.
 */
ZStepper::EndstopType ZStepper::getEndstopType() {
  return _endstopType;
}

/**
 * @brief Set the endstop trigger state.
 * 
 * @param state   The state in which the endstop is considered as triggered.
 
 * @param index   The number of the endstop to set (either 1 or 2).
 */
void ZStepper::setEndstopState(uint8_t state, uint8_t index /* = 1 */) {
  index == 1 ? _endstopState = state : _endstopState2 = state; 
  __debugS(DEV2, PSTR("\tStepper '%-8s' EndstopState now set to: %s"), _descriptor, state ? PSTR("HIGH") : PSTR("LOW"));
}

/**
 * @brief Get the Endstop Trigger State.
 * 
 * @param index     The number of the endstop to query.
 * 
 * @returns The current trigger state of the endstop.
 */
uint8_t ZStepper::getEndstopState(uint8_t index /* = 1 */) {
  return index == 1 ? _endstopState : _endstopState2; 
}

/**
 * @brief Set Endstop Hit state.
 * 
 * @param state   The trigger state (true = hit, false = not hit).
 * @param index   The number of the endstop to set.
 */
void ZStepper::setEndstopHit(int8_t state, int8_t index /* = 1 */) {
  index == 1 ? _endstopHit = state : _endstopHit2 = state; 
}

/**
 * @brief Get Endstop Hit state.
 * 
 * @param   index     The index of the targeted endstop (1 or 2)
 * 
 * @returns true      if the targeted endstop was hit.
 * @returns false     if the targeted endstop was not hit.
 */
bool ZStepper::getEndstopHit(int8_t index) {
  int8_t stat = 0;
  if(index == 1) {
    if(_endstopPin > 0) {
      _endstopHit = (bool)digitalRead(_endstopPin) == _endstopState;
    }
    else {
      if(endstopCheck != nullptr)
        _endstopHit = endstopCheck() == _endstopState;
      else
        _endstopHit = false;
    }
    return _endstopHit;
  }
  else {
    if(_endstopPin2 > 0) {
      _endstopHit2 = (bool)digitalRead(_endstopPin2) == _endstopState2;
    }
    else {
      _endstopHit2 = false;
    }
    return _endstopHit2;
  }
}

/**
 * @brief Get Endstop Hit state.
 *        This is an alternative method which only returns the internally stored state.
 * 
 * @param index   The number of the endstop to query.
 * @returns true   if the indexed endstop has been hit.
 * @returns false  if the indexed endstop hasn't been hit.
 */
bool ZStepper::getEndstopHitAlt(int8_t index /* = 1 */) {
  return index == 1 ? _endstopHit : _endstopHit2;
}

/**
 * @brief Set Endstop Pin.
 * 
 * @param pin     The pin number on the MCU.
 * @param index   The number of the endstop to set.
 */
void ZStepper::setEndstopPin(pin_t pin, int8_t index /* = 1 */) {
  index == 1 ? _endstopPin = pin : _endstopPin2 = pin;
}

/**
 * @brief Get Endstop Pin.
 * 
 * @param index   The number of the endstop to query.
 * 
 * @returns The currently assigned pin number on the MCU.
 */
pin_t ZStepper::getEndstopPin(int8_t index /* = 1 */) {
  return index == 1 ? _endstopPin : _endstopPin2; 
}

/**
 * @brief Set Ignore Endstop flag.
 * 
 * @param state   true if the endstop has to be ignored, false if not. 
 */
void ZStepper::setIgnoreEndstop(bool state) {
  _ignoreEndstop = state;
}

/**
 * @brief Get Ignore Endstop flag.
 * 
 * @returns true   when the endstop will be ignored.
 * @returns false  when the endstop isn't being ignored.
 */
bool ZStepper::getIgnoreEndstop() {
  return _ignoreEndstop; 
}

/**
 * @brief Set Number of Steps taken.
 * 
 * @param count   The number of steps taken.
 */
void ZStepper::setStepCount(long count) {
  _stepCount = count; 
}

/**
 * @brief Get Number of Steps taken.
 * 
 * @returns The number of steps taken.
 */
long ZStepper::getStepCount() {
  return _stepCount;
}

/**
 * @brief Set Max. Number of Steps on this stepper.
 *        This is used to prevent the stepper from moving futher then the geometry of the machine
 *        allows.
 * 
 * @param count   The max. amount of steps this stepper can make.
 */
void ZStepper::setMaxStepCount(long count) {
  _maxStepCount = count;
}

/**
 * @brief Get Max. Number of Steps.
 * 
 * @returns The max. number of steps this stepper is allowed to move.
 */
long ZStepper::getMaxStepCount() {
  return _maxStepCount;
}

/**
 * @brief Set Total Distance for next movement in steps.
 * 
 * @param count The total number of steps for the next movement.
 */
void ZStepper::setTotalSteps(long count) {
  _totalSteps = count;
}

/**
 * @brief Get Total Distance for next movement in steps.
 * 
 * @returns The currently set total number of steps on the next movement.
 */
long ZStepper::getTotalSteps() {
  return _totalSteps;
}

/**
 * @brief Set Current Position in Steps.
 * 
 * @param position  The current position.
 */
void ZStepper::setStepPosition(long position) {
  _stepPosition = position; 
  _stepPositionMM = (double)((double)_stepPosition / _stepsPerMM);
}

/**
 * @brief Get Current Position in Steps.
 * 
 * @returns The current position.
 */
long ZStepper::getStepPosition() {
  return _stepPosition;
}

/**
 * @brief Set Current Position in Millimeter.
 * 
 * @param position  The current position in mm.
 */
void ZStepper::setStepPositionMM(double position) {
  _stepPositionMM = position;
  _stepPosition = (long)(position * _stepsPerMM);
}

/**
 * @brief Get Current Position in Millimeter.
 * 
 * @returns The current position in mm.
 */
double ZStepper::getStepPositionMM() {
  return _stepPositionMM;
}

/**
 * @brief Set the Movement Done Flag.
 *        This method also updates the current Step Position.
 * 
 * @param state   The state to which to set the movement done flag (true/false).
 * @returns true   when the movement has finished.
 * @returns false  when the movement is still ongoing.
 * 
 * @note The return value is the same in getMovementDone() and can be used to pass the status back to the caller without
 *       calling the getMovementDone() member.
 */
bool ZStepper::setMovementDone(bool state) {
  _movementDone = state;
  _resetSignal = false;
  if(_movementDone) {
    setStepPosition(_stepPosition);   // update the step position to get the position in mm
  }
  return state;
}

/**
 * @brief Get Movement Status.
 * 
 * @returns true   if the movement has finished.
 * @returns false  if the movement is still ongoing.
 */
bool ZStepper::getMovementDone() {
  return _movementDone;
}

/**
 * @brief Set the Acceleration Value.
 *        This value is usually a magnitude of the max. speed value and is being used to get the stepper
 *        spinning up from no speed. This is also the ending value for delecration.
 * 
 * @param value   The starting value in timer ticks.
 */
void ZStepper::setAcceleration(uint16_t value) {
  _acceleration = value;
}

/**
 * @brief Get the Acceleration Value.
 * 
 * @returns The currently set starting value in timer ticks. 
 */
uint16_t ZStepper::getAcceleration() {
  return _acceleration;
}

/**
 * @brief Set the Maximum Speed.
 * 
 * @param value   The max. speed in timer ticks.
 */
void ZStepper::setMaxSpeed(uint16_t value) {
  _maxSpeed = value;
}

/**
 * @brief Get the Maximum Speed.
 * 
 * @returns The currently set max. speed for this stepper in timer ticks. 
 */
uint16_t ZStepper::getMaxSpeed() {
  return _maxSpeed;
}

/**
 * @brief Set the Inversion of the Direction flag.
 *        This comes in handy if the DIR signal (usually active HIGH) is connected to an inverting driver.
 * 
 * @param state true if the direction signal shall be inverted, false otherwise.
 */
void ZStepper::setInvertDir(bool state) {
  _invertDir = state;
}

/**
 * @brief Get the Direction Inversion flag.
 * 
 * @returns true   if the direction signal (DIR) must be inverted.
 * @returns false  if the direction signal (DIR) is connected directly.
 */
bool ZStepper::getInvertDir() {
  return _invertDir;
}

/**
 * @brief Set Duration Value (timer overflow). 
 * 
 * @param value The ticks the timer has to count before the next interrupt will occur. 
 */
void ZStepper::setDuration(timerVal_t value) {
  _durationInt = value;
}

/**
 * @brief Get the Duration Value.
 * 
 * @returns The number of ticks currently set.
 */
timerVal_t ZStepper::getDuration() {
  return _durationInt;
}

/**
 * @brief Check if a value is LESS THAN the currently set Duration Value.
 * 
 * @param value   Pointer to the value to check against and assign a new value if applicable.
 * @returns true   if the given value is LESS and a new value has been assigned.
 * @returns false  if the given value is GREATER or interrupt factor has been set.
 * @see setInterruptFactor()
 */
bool ZStepper::isLTDuration(timerVal_t* value) {
  if(_intrFactor == 0 && _durationInt < *value) {   // ignore this stepper, if the interrupt factor is set (i.e. sync movement)
    *value = _durationInt; 
    return true; 
  }
  return false; 
}

/**
 * @brief Check if a value is GREATER THAN the currently set Duration Value.
 * 
 * @param value   Pointer to the value to check against and assign a new value if applicable.
 * 
 * @returns true   if the given value is GREATER and a new value has been assigned.
 * @returns false  if the given value is LESS or interrupt factor has been set.
 * @see setInterruptFactor()
 */
bool ZStepper::isGTDuration(timerVal_t* value) {
  if(_intrFactor == 0 && _durationInt > *value) {   // ignore this stepper, if the interrupt factor is set (i.e. sync movement)
    *value = _durationInt; 
    return true; 
  }
  return false; 
}

/**
 * @brief Set Steps per Millimeter.
 * 
 * @param steps   The number of steps to take for 1 mm of movement.
 */
void ZStepper::setStepsPerMM(uint16_t steps) {
  _stepsPerMM = steps;
}

/**
 * @brief Get Steps per Millimeter.
 * 
 * @returns The current steps/mm value. 
 */
uint16_t ZStepper::getStepsPerMM() {
  return _stepsPerMM;
}

/**
 * @brief Set Steps per Degree.
 * 
 * @param steps   The number of steps to take for 1 degree of movement.
 */
void ZStepper::setStepsPerDegree(double steps) {
  _stepsPerDegree = steps;
}

/**
 * @brief Get Steps per Degree.
 * 
 * @returns The number of steps it takes for 1 degree of movement.
 */
double ZStepper::getStepsPerDegree() {
  return _stepsPerDegree;
}

/**
 * @brief Set Acceleration Allowed flag.
 * 
 * @param state true = use accel/decel, false = don't use.
 */
void ZStepper::setAllowAccel(bool state) {
  _allowAcceleration = state;
}

/**
 * @brief Get Acceleration Allowed flag.
 * 
 * @returns true   if acceleration/deceleration will take place.
 * @returns false  if acceleration/deceleration doesn't take place.
 */
bool ZStepper::getAllowAccel() {
  return _allowAcceleration;
}

/**
 * @brief Set the Distance for Acceleration.
 *        If this value is set to 0, this class will use 20% of the total distance for accel/decel.
 * 
 * @param dist The distance in mm or degree (if the stepper is doing Orbital movements).
 */
void ZStepper::setAccelDistance(uint8_t dist) {
  _accelDistance = dist;
}

/**
 * @brief Get the Distance for Acceleration.
 * 
 * @returns The currently set distance in mm/degree.
 */
uint8_t ZStepper::getAccelDistance() {
  return _accelDistance;
}

/**
 * @brief Set Abort flag.
 *        Whenever the Abort flag is set and the Ignore Abort flag is false, any stepper motion will stop immediately.
  * 
 * @param state   true if abort is allowed, false if not.
 * @see setIgnoreAbort() and getIgnoreAbort()
 */
void ZStepper::setAbort(bool state) {
  _abort = state;
}

/**
 * @brief Get Abort flag.
 * 
 * @returns true   if the abort flag has been set.
 * @returns false  if the abort flag hasn't been set.
 */
bool ZStepper::getAbort() {
  return _abort;
}

/**
 * @brief Set Ignore Abort flag.
 *        If this flag is set, any change in Abort will be ignored.
 * 
 * @param state true if the abort flag can be ignored temporarily, false if not.
 */
void ZStepper::setIgnoreAbort(bool state) {
  _ignoreAbort = state;
}

/**
 * @brief Get Ignore Abort flag.
 * 
 * @returns true   when a Abort can be ignored temporarily.
 * @returns false  when a Abort must not be ignored.
 */
bool ZStepper::getIgnoreAbort() {
  return _ignoreAbort;
}

/**
 * @brief Set the number of Steps Taken.
 *        The primary use is to reset the step counter to 0.
 * 
 * @param count   The current step counter value.
 */
void ZStepper::setStepsTaken(long count) {
  _stepsTaken = count; 
}

/**
 * @brief Get Number of Steps Taken.
 * 
 * @returns The steps taken since the last reset.
 */
long ZStepper::getStepsTaken() {
  return _stepsTaken; 
}

/**
 * @brief Get Number of Steps Taken in Milimeter.
 * 
 * @returns The steps taken in milimeter since the last reset.
 */
double ZStepper::getStepsTakenMM() {
  return (double)_stepsTaken / _stepsPerMM;
}

/**
 * @brief Get the Stall Detected flag.
 * 
 * @returns true   if a stall has been detected.
 * @returns false  if no stall has been detected yet.
 */
bool ZStepper::getStallDetected() {
  return _stallDetected;
}

/**
 * @brief Clear the Stall Detected flag.
 * 
 */
void ZStepper::resetStallDetected() {
  _stallDetected = false;
}

/**
 * @brief Set Stop On Stall Detection flag.
 *        If this flag is set, the movement of this stepper will be stopped as soon as the max.
 *        threshold value has been reached.
 * 
 * @param state   Flag to enable/disable stall detection handling.
 * 
 * @see setStallThreshold()
 */
void ZStepper::setStopOnStallDetected(bool state) {
  _stopOnStallDetected = state;
}

/**
 * @brief Get Stop On Stall Detection flag.
 * 
 * @returns true   if stall detection handling has been enabled.
 * @returns false  if stall detection handling has been disabled.
 */
bool ZStepper::getStopOnStallDetected() {
  return _stopOnStallDetected;
}

/**
 * @brief Get the Stall Detection Threshold.
 * 
 * @returns The currently set threshold counter value.
 */
uint8_t ZStepper::getStallThreshold() {
  return _stallCountThreshold;
}

/**
 * @brief Set the Stall Detection Threshold. 
 *        This method is used for TMC stepper drivers which can report a stall detection. This value sets
 *        the max. number of stalls after which a Stall will be reported.
 * 
 * @param max   The allowed number of stalls.
 */
void ZStepper::setStallThreshold(uint8_t max) {
  _stallCountThreshold = max;
}

/**
 * @brief Get Stall Counter. 
 * 
 * @returns The current stall counter value.
 */
uint32_t ZStepper::getStallCount() {
  return _stallCount;
}

/**
 * @brief Set the Interrupt Factor. 
 *        This factor is needed if two or more steppers have to move in sync and defines how many steps the synchronizing
 *        stepper needs to take, before this stepper moves one step. E.g.: a value of 9 means, with every 9th step on the main 
 *        stepper, this stepper will make one step.
 * 
 * @param factor The interrupt factor value to be set.
 */
void ZStepper::setInterruptFactor(timerVal_t factor) {
  _intrFactor = factor;
}

/**
 * @brief Get the Interrupt Factor.
 * 
 * @returns The current interrupt factor value.
 */
timerVal_t ZStepper::getInterruptFactor() {
  return _intrFactor;
}

/**
 * @brief Adjust the current step position as the endstop triggers.
 *        Depending on the typeof endstop, the current position will be set differently.
 * 
 * @param   index     The index of the target endstop (1 or 2).
 * 
 * @note Index 2 is currently not being used.
 */
void ZStepper::adjustPositionOnEndstop(int8_t index) {

  if(index != 1)
    return;

  if(!_ignoreEndstop && _endstopType != MINMAX) {
    long beforeAdjust = _stepPosition;
    switch(_endstopType) {
      case MIN:
      case ORBITAL:
        setStepPosition(0L);
        break;
      case MAX:
        setStepPosition(_maxStepCount);
        break;
    }
    __debugSInt(DEV3, PSTR("\tStepper '%-8s' Adjusted step pos. %ld, was: %ld"), _descriptor, _stepPosition, beforeAdjust);
  }
}

/**
 * @brief Check whether or not it's allowed to generate a STEP pulse for this stepper driver.
 *        Mainly used when two or more steppers need to move 'in sync'. 
 * 
 * @returns true    Caller shall generate STEP signal
 * @returns false   Caller shall not generate STEP signal
 */
bool ZStepper::allowInterrupt() {
  _intrFactorCount += _intrFactor;
  if (_intrFactorCount >= 10000) {    // _intrFactorCount is uint32 to overcome issues with decimals
    _intrFactorCount = 0;
    return true;
  }
  return false;
}

/**
 * @brief Check whether the endstop was hit. 
 * @note This method is supposed to be called from inside an ISR.
 * 
 * @param index   The number of the endstop (either 1 or 2).
 * 
 * @returns true   if the indexed endstop has reached it's defined state (hit).
 * @returns false  if the indexed endstop has not reached it's defined state (not hit).
 */
bool ZStepper::evalEndstopHit(uint8_t index) {
  if(index == 1) {
    _endstopHit = (bool)digitalRead(_endstopPin) == _endstopState;
    if(_endstopHit && !_movementDone) {
      if(_endstopType == MIN && _dir == CW) { 
        // don't report endstop hit if stepper is supposed to move away from endstop
        return false;
      }
      adjustPositionOnEndstop();
    }
    return _endstopHit;
  }
  else {
    _endstopHit2 = (bool)digitalRead(_endstopPin2) == _endstopState2;
    return _endstopHit2;
  }
}

/**
 * @brief Sets the speed of the stepper motor by applying either acceleration/deceleration values 
 *        or max. speed if the stepper is out of the accel/decel phase.
 * 
 */
inline void ZStepper::updateAcceleration() {

  if(!_allowAcceleration || _accelEvery == 0) {
    _durationInt  = _maxSpeed;
    return;
  }

  if(_stepCount % _accelEvery != 0)             // no change in speed to expect, do nothing
    return;

  if(_stepCount <= _accelDistSteps) {
    _durationInt--;                             // accelerate (i.e. make the timer interval shorter)
    if(_maxSpeed > _durationInt)
      _durationInt = _maxSpeed;
  }
  else if (_stepCount >= _decelPos) {
    _durationInt++;                             // decelerate (i.e. make the timer interval longer)
    if(_durationInt > _acceleration)
      _durationInt = _acceleration;
  }
  else
    _durationInt = _maxSpeed;                   // set default (max.) speed
}

/**
 * @brief Main ISR handler.
 *        This method needs to be called twice from the main timer interrupt service routine (ISR) for each step 
 *        that needs to be generated. The first call will set the STEP pin, the second call will reset it.
 *        This way a clean square wave signal is being generated on the stepper driver pin.
 * 
 * @returns true    if the target distance has been reached, an 'Abort' signal was received or the according endstop has triggered.
 * @returns false   if the target distance has not been reached yet, further interrupts have to be generated.
 * 
 * @note 2013-05-07: Method overhauled to reduce the processing time (currently: ~ 3.5uS)
 */
bool ZStepper::handleISR() {

  if(_resetSignal) {                    // 2nd call is for resetting the STEP signal
    stepFunc(_stepPin, _resetSignal);   // hence, just reset it, do some minor condition processing and return
    _resetSignal = false;

    if(_endstopType == ORBITAL) {
      if(_stepPosition >= _maxStepCount) {
        _stepPosition = 0L;
      }
      else if(_stepPosition < 0) {
        _stepPosition = _maxStepCount-1;
      }
    }

    #ifdef HAS_TMC_SUPPORT
      // check stepper motor stall on TMC2209, if configured likewise
      if(_stallCountThreshold > 0) {
        if(_stallCount > _stallCountThreshold) {
          _stallDetected = true;
          if(_stopOnStallDetected) {
            return setMovementDone(true);
          }
        }
      }
    #endif

    // check for "Abort" condition
    if(!_ignoreAbort && _abort) {
      return setMovementDone(true);
    }
    // check for "Movement Done" conditions
    if(!_ignoreEndstop && evalEndstopHit()) {
      return setMovementDone(true);
    }
    // check for "Overflow" condition
    if(_dir == CW) {
      if(_maxStepCount != 0 && _stepCount >= _maxStepCount) {
        return setMovementDone(true);
      }
    }
    if(_stepCount >= _totalSteps) {
      return setMovementDone(true);
    }
    return _movementDone;
  }
  // otherwise, update accel/decel and set STEP pin
  updateAcceleration();
  stepFunc(_stepPin, _resetSignal);
  _resetSignal = true;
  _stepCount++;
  _stepPosition += _dir;
  _stepsTaken += _dir;
  return false;
}


/**
 * @brief Move the stepper into 'Home' position, which is reached when the endstop triggers.
 *        This method runs two cycles: 1st cycle is to get the endstop to trigger once at a high speed,
 *        2nd cycle is to get the endstop to trigger at a low speed.
 */
void ZStepper::home() {
  // calculate the movement distances: if an endstop is set, go 20% beyond max.
  long distance = (_endstopPin > 0) ? -((long)((double)_maxStepCount*1.2)) : -_maxStepCount;
  long distanceF = 0;
  long back = -(distance/36);
  // if the endstop type is ORBITAL (Revolver) and the current position is beyond the middle, turn inverse
  if(_endstopType == ORBITAL && (getStepPosition() >= _maxStepCount/2 && getStepPosition() <= _maxStepCount)) {
    distanceF = abs(distance);
  }

  // only if the endstop is not being hit already, move to endstop position
  if(!_endstopHit) {
    prepareMovement(distanceF == 0 ? distance : distanceF);
    if(runAndWaitFunc != nullptr)
      runAndWaitFunc(_number);
  }

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
  __debugS(DEV3, PSTR("\tStepper '%-8s' Position after home: %ld (%s mm)"), _descriptor, _stepPosition, String(_stepPositionMM).c_str());

  // reset the speed
  setMaxSpeed(curSpeed);
}
