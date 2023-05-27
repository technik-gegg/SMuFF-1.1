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
 * Module containing helper functions
 */

#include "SMuFF.h"


SMuFFConfig     smuffConfig;
PositionMode    positionMode = RELATIVE;
bool            feederJammed = false;
bool            isAbortRequested = false;
bool            lidOpen = true;
bool            ignoreHoming = false;
int8_t          currentSerial = -1;
int8_t          toolSelected = -1;
int8_t          toolPending = -1;
uint8_t         swapTools[MAX_TOOLS];
unsigned long   feederErrors = 0;
unsigned long   stallDetectedCountSelector = 0;
unsigned long   stallDetectedCountFeeder = 0;

#ifdef HAS_TMC_SUPPORT
extern TMC2209Stepper *showDriver;
#endif

/**
 * Parses a given buffer for newlines (delimiter) and fills the lines pointer
 * array with the lines found.
 *
 * @param lines     container for lines
 * @param maxLines  max. storage capacity of lines
 * @param message   the source buffer to be parsed for lines
 * @param delimiter the EOL character to parse for
 * @returns the number of lines found
 */
uint8_t splitStringLines(char *lines[], uint8_t maxLines, const char *message, const char *delimiter) {

  char *tok = strtok((char *)message, delimiter);
  char *lastTok = nullptr;
  int8_t cnt = -1;

  while (tok != nullptr) {
    lines[++cnt] = tok;
    lastTok = tok;
    //__debugS(DEV3, PSTR("Line: %s"), lines[cnt]);
    if (cnt >= maxLines - 1)
      break;
    tok = strtok(nullptr, delimiter);
  }
  if (lastTok != nullptr && *lastTok != 0 && cnt <= maxLines - 1) {
    lines[cnt] = lastTok; // copy the last line as well
    cnt++;
  }

  return cnt;
}

bool selectorEndstop() {
  return steppers[SELECTOR].getEndstopHit();
}

bool revolverEndstop() {
  return steppers[REVOLVER].getEndstopHit();
}

bool feederEndstop(int8_t index) {
  return steppers[FEEDER].getEndstopHit(index);
}

bool ddeEndstop() {
  return steppers[DDE_FEEDER].getEndstopHit();
}

void setAbortRequested(bool state) {
  steppers[FEEDER].setAbort(state); // stop any ongoing stepper movements
}

void setParserBusy() {
  parserBusy = true;
  // __debugS(DEV3, PSTR("Parser busy..."));
}

void setParserReady() {
  parserBusy = false;
  refreshStatus();
  // __debugS(DEV3, PSTR("Parser ready..."));
}

bool moveHome(int8_t index, bool showMessage, bool checkFeeder) {
  char errmsg[MAX_ERR_MSG];

  if (!steppers[index].getEnabled())
    steppers[index].setEnabled(true);

  if (feederJammed) {
    beep(4);
    return false;
  }
  setParserBusy();
  if (checkFeeder && feederEndstop()) {
    if (showMessage) {
      if (!showFeederLoadedMessage()) {
        setParserReady();
        return false;
      }
    }
    else {
      if (feederEndstop())
        unloadFilament(errmsg);
    }
  }
  if (smuffConfig.revolverIsServo) {
    __debugS(DEV2, PSTR("Stepper home SERVO variant"));
    // don't release the servo when homing the Feeder but
    // release it when homing something else
    if (index != FEEDER)
      setServoLid(SERVO_OPEN);
    // Revolver isn't being used on a servo variant
    if (index != REVOLVER)
      steppers[index].home();
  }
  else {
    __debugS(DEV2, PSTR("Stepper home non SERVO variant"));
    // not a servo variant, home stepper which ever it is
    if (index != REVOLVER)
      steppers[index].home();
    else {
      uint8_t retries = 5;
      do {
        steppers[index].home();
        if(steppers[index].getEndstopHit())
          break;
        delay(250);
        retries--;
        if(retries == 0)
          break;
        __debugS(DEV, PSTR("Revolver not homed, retrying %d more times"), retries);
      } while(1);
    }

    #if defined(SMUFF_V6S)
      if (index == REVOLVER) {
        steppers[index].setEnabled(false);    // turn off LID stepper when done
        if(steppers[index].getEndstopHit())
          lidOpen = true;
      }
    #endif
  }

  //__debugS(DEV2, PSTR("Stepper home done"));
  if (index == SELECTOR) {
    setFastLEDToolIndex(toolSelected, 0, true);
    toolSelected = -1;
    #if defined(USE_SPLITTER_ENDSTOPS)
    splitterMux.setTool(toolSelected);
    #endif
  }
  long pos = steppers[index].getStepPosition();
  if (index == SELECTOR || index == REVOLVER)
    dataStore.tool = toolSelected;
  dataStore.stepperPos[index] = pos;
  saveStore();
  //__debugS(DEV2, PSTR("DONE save store"));
  setParserReady();
  #if defined(SMUFF_V6S)
    return lidOpen;
  #endif
  return true;
}


void switchFeederStepper(uint8_t stepper) {
  char relay[50];
  #if !defined(USE_MULTISERVO_RELAY)
    if (RELAY_PIN == 0)
      return;
  #else
    if(servoMapping[RELAY] == -1)
      return;
  #endif
  #if defined(USE_DDE)
    steppers[DDE_FEEDER].setEnabled(false);
  #else
    steppers[FEEDER].setEnabled(false);
  #endif
  #if !defined(USE_MULTISERVO_RELAY)
    digitalWrite(RELAY_PIN, stepper == EXTERNAL ? smuffConfig.invertRelay : !smuffConfig.invertRelay);
    snprintf_P(relay, ArraySize(relay)-1, PSTR("Relay"));
  #else
    bool state = stepper == EXTERNAL ? smuffConfig.invertRelay : !smuffConfig.invertRelay;
    servoPwm.setPin(servoMapping[RELAY], state ? 4095 : 0);
    snprintf_P(relay, ArraySize(relay)-1, PSTR("Relay on Multiservo (pin %d)"), servoMapping[RELAY]);
  #endif
  smuffConfig.externalStepper = stepper == EXTERNAL;
  __debugS(DEV4, PSTR("%s set to \"%s\" state"), relay, smuffConfig.externalStepper ? P_External : P_Internal);
  delay(150);   // gain the relay some time to debounce
}

void moveFeeder(double distanceMM) {
  steppers[FEEDER].setEnabled(true);
  uint16_t curSpeed = steppers[FEEDER].getMaxSpeed();
  changeFeederSpeed(smuffConfig.insertSpeed);
  prepSteppingRelMillimeter(FEEDER, distanceMM, true);
  runAndWait(FEEDER);
  steppers[FEEDER].setMaxSpeed(curSpeed);
}

void positionRevolver() {
  // disable Feeder temporarily
  steppers[FEEDER].setEnabled(false);
  #if !defined(SMUFF_V6S)
  if (smuffConfig.resetBeforeFeed && !ignoreHoming) {
    if (smuffConfig.revolverIsServo)
      setServoLid(SERVO_OPEN);
    else
      moveHome(REVOLVER, false, false);
  }
  if (smuffConfig.revolverIsServo) {
    setServoLid(SERVO_CLOSED);
    steppers[FEEDER].setEnabled(true);
    return;
  }
  #endif

  #if defined(SMUFF_V6S)
    double newPos = stepperPosClosed[toolSelected];
    if(newPos > 0) {
      steppers[REVOLVER].setEnabled(true);   // turn on the Lid stepper
      prepSteppingRelMillimeter(REVOLVER, newPos, true); // go to position, don't mind the endstop
      //__debugS(DEV, PSTR("V6S: Position Revolver, pos=%f"), newPos);
      remainingSteppersFlag |= _BV(REVOLVER);
      runAndWait(REVOLVER);
    }
    else {
      //__debugS(DEV, PSTR("V6S: Skipping, because position for Revolver=%f"), stepperPosClosed[toolSelected]);
    }
  #else
    long pos = steppers[REVOLVER].getStepPosition();
    long newPos = smuffConfig.firstRevolverOffset + (toolSelected * smuffConfig.revolverSpacing);
    // calculate the new position and decide whether to move forward or backard
    // i.e. which ever has the shorter distance
    long delta1 = newPos - (smuffConfig.stepsPerRevolution + pos); // number of steps if moved backward
    long delta2 = newPos - pos;                                    // number of steps if moved forward
    if (abs(delta1) < abs(delta2))
      newPos = delta1;
    else
      newPos = delta2;

    // if the position hasn't changed, do nothing
    if (newPos != 0) {
      prepSteppingRel(REVOLVER, newPos, true); // go to position, don't mind the endstop
      remainingSteppersFlag |= _BV(REVOLVER);
      runAndWait(-1);
      if (smuffConfig.wiggleRevolver) {
        // wiggle the Revolver one position back and forth
        // just to adjust the gears a bit better
        delay(50);
        prepSteppingRel(REVOLVER, smuffConfig.revolverSpacing, true);
        remainingSteppersFlag |= _BV(REVOLVER);
        runAndWait(-1);
        delay(50);
        prepSteppingRel(REVOLVER, -(smuffConfig.revolverSpacing), true);
        remainingSteppersFlag |= _BV(REVOLVER);
        runAndWait(-1);
      }
    }
  #endif
  steppers[FEEDER].setEnabled(true);
  delay(150);
  #if defined(SMUFF_V6S)
    if(toolSelected != -1)
      lidOpen = false;
    steppers[REVOLVER].setEnabled(false);   // turn off the Lid stepper
  #endif
  //__debugS(DEV2, PSTR("PositionRevolver: pos: %d"), steppers[REVOLVER].getStepPosition());
}

void changeFeederSpeed(uint16_t speed) {
  unsigned long _speed = translateSpeed(speed, FEEDER);
  if (_speed != steppers[FEEDER].getMaxSpeed()) {
    __debugS(DEV2, PSTR("Changed Feeder speed from %ld to %ld"), steppers[FEEDER].getMaxSpeed(), _speed);
    steppers[FEEDER].setMaxSpeed(_speed);
  }
}

void changeDDEFeederSpeed(uint16_t speed) {
  #if defined(USE_DDE)
  unsigned long _speed = translateSpeed(speed, DDE_FEEDER);
  if (_speed != steppers[DDE_FEEDER].getMaxSpeed()) {
    __debugS(DEV2, PSTR("Changed DDE-Feeder speed from %ld to %ld"), steppers[DDE_FEEDER].getMaxSpeed(), _speed);
    steppers[DDE_FEEDER].setMaxSpeed(_speed);
  }
  #endif
}

void repositionSelector(bool retractFilament) {
  char _errmsg[MAX_ERR_MSG];
  int8_t tool = getToolSelected();
  if (retractFilament && !smuffConfig.revolverIsServo) {
    char tmp[15];
    uint16_t curSpeed = steppers[FEEDER].getMaxSpeed();
    changeFeederSpeed(smuffConfig.insertSpeed);
    ignoreHoming = true;
    // go through all tools available and retract some filament
    for (uint8_t i = 0; i < smuffConfig.toolCount; i++) {
      if (i == tool)
        continue;
      sprintf(tmp, "Y%d", i);
      G0("G0", tmp, 255, _errmsg);                                                   // position Revolver on tool
      prepSteppingRelMillimeter(FEEDER, -smuffConfig.insertLength, true); // retract
      runAndWait(FEEDER);
    }
    ignoreHoming = false;
    sprintf(tmp, "Y%d", tool);
    G0("G0", tmp, 255, _errmsg); // position Revolver on tool selected
    steppers[FEEDER].setMaxSpeed(curSpeed);
  }
  moveHome(SELECTOR, false, false); // home Revolver
  char errmsg[MAX_ERR_MSG];
  selectTool(tool, errmsg, false);          // reposition Selector
}

bool feedToEndstop(char* errmsg, bool showMessage) {
  char _dummy[MAX_ERR_MSG];
  // enable steppers if they were turned off
  steppers[FEEDER].setEnabled(true);

  // don't allow "feed to endstop" being interrupted
  steppers[FEEDER].setIgnoreAbort(true);

  positionRevolver();

  uint16_t curSpeed = steppers[FEEDER].getMaxSpeed();
  //__debugS(DEV, PSTR("InsertSpeed: %d"), smuffConfig.insertSpeed);
  uint16_t speed = smuffConfig.insertSpeed;
  changeFeederSpeed(speed);
  if (smuffConfig.accelSpeed[FEEDER] > smuffConfig.insertSpeed)
    steppers[FEEDER].setAllowAccel(false);

  uint16_t max = (uint16_t)(smuffConfig.selectorDistance * 4); // calculate a maximum distance to avoid feeding endlessly
  uint8_t n = 0;
  int8_t retries = FEED_ERROR_RETRIES; // max. retries for this operation
  if(retries < ((int8_t)smuffConfig.selectorDistance/smuffConfig.insertLength))
    retries = (int8_t)(smuffConfig.selectorDistance/smuffConfig.insertLength)+2;

  feederJammed = false;

  // is the feeder endstop already being triggered?
  if (feederEndstop()) {
    //__debugS(DEV, PSTR("Feeder endstop was already triggered"));
    
    // yes, filament is still fed, unload completelly and
    // abort this operation if that fails
    if(!smuffConfig.useSplitter) {
      if (!unloadFromNozzle(errmsg, showMessage))
        return false;
    }
  }

  steppers[FEEDER].setStepPositionMM(0);
  steppers[FEEDER].setAllowAccel(true);
  // as long as Selector endstop doesn't trigger
  // feed the configured insertLength
  while (!feederEndstop()) {
    __debugS(DEV2, PSTR("Feeding %s mm, max.: %d mm"), String(smuffConfig.insertLength).c_str(), max);
    prepSteppingRelMillimeter(FEEDER, smuffConfig.insertLength, false);
    runAndWait(FEEDER);
    // has the endstop already triggered?
    if (feederEndstop()) {
      __debugS(DEV2, PSTR("Endstop triggered, position now: %s mm"), String(steppers[FEEDER].getStepPositionMM()).c_str());
      break;
    }
    n += smuffConfig.insertLength; // increment the position of the filament
    // did the Feeder stall? (TMC2209 only)
    bool stallStat = handleFeederStall(&speed, &retries);
    // if endstop hasn't triggered yet, feed was not successful
    if (n >= max && !feederEndstop()) {
      __debugS(DEV2, PSTR("Position now: %s mm, endstop hasn't triggered, retrying"), String(steppers[FEEDER].getStepPositionMM()).c_str());
      delay(250);
      // retract half the insertLength and reset the Revolver
      prepSteppingRelMillimeter(FEEDER, -(smuffConfig.insertLength / 2), true);
      runAndWait(FEEDER);
      resetRevolver();
      feederErrors++; // global counter used for testrun 
      if (stallStat)  // did not stall means no retries decrement, though, the endstop hasn't triggered yet
        retries--;
      // if only two retries are left, try repositioning the Selector
      if (retries == 1) {
        repositionSelector(false);
      }
      // if only one retry is left, rectract filaments a bit and try repositioning the Selector
      if (retries == 0) {
        repositionSelector(true);
      }
      // close lid servo in case it got openend by the reposition operation
      if (smuffConfig.revolverIsServo)
        setServoLid(SERVO_CLOSED);
      n = 0;
    }
    //__debugS(D, PSTR("Max: %s  N: %s  Retries: %d  Endstop: %d"), String(max).c_str(), String(n).c_str(), retries, feederEndstop());
    if (!feederEndstop() && retries < 0) {
      // still got no endstop trigger, abort action
      if (showMessage) {
        moveHome(REVOLVER, false, false); // home Revolver
        M18("M18", "", 255, _dummy);              // turn all motors off
        if (smuffConfig.revolverIsServo)  // release servo, if used
          setServoLid(SERVO_OPEN);
        // if user wants to retry...
        if (showFeederFailedMessage(1) == true) {
          // reset and start over again
          steppers[FEEDER].setEnabled(true);
          positionRevolver();
          n = 0;
          retries = FEED_ERROR_RETRIES;
          continue;
        }
      }
      // otherwise, assume the feeder is jammed
      feederJammed = true;
      __debugS(D, PSTR("Feeder jammed after %d retries"), FEED_ERROR_RETRIES);
      break;
    }
  }
  // feed another "Selector Distance" slowly
  __debugS(DEV, PSTR("Feeding through fitting: %s mm"), String(smuffConfig.selectorDistance).c_str());
  prepSteppingRelMillimeter(FEEDER, smuffConfig.selectorDistance, false);
  runAndWait(FEEDER);
  if(stat)
    smuffConfig.feedLoadState[toolSelected] = LOADED_TO_SELECTOR;

  sendStates(true);
  steppers[FEEDER].setIgnoreAbort(false);
  steppers[FEEDER].setAllowAccel(true);
  steppers[FEEDER].setMaxSpeed(curSpeed);
  delay(100);
  return feederJammed ? false : true;
}

bool unloadFromSplitter(char* errmsg, bool showMessage) {
  // enable steppers if they were turned off
  steppers[FEEDER].setEnabled(true);

  // don't allow "feed to endstop" being interrupted
  positionRevolver();
  uint16_t curSpeed = steppers[FEEDER].getMaxSpeed();

  feederJammed = false;
  steppers[FEEDER].setStepPositionMM(0);
  double len = smuffConfig.bowdenLength;

  #if defined(USE_SPLITTER_ENDSTOPS)
    // invert endstop trigger state for unloading
    bool endstopState = steppers[FEEDER].getEndstopState();
    steppers[FEEDER].setEndstopState(!endstopState);

    len = smuffConfig.bowdenLength;
    // unload until Selector endstop gets released
    int8_t retries = FEED_ERROR_RETRIES;
    do {
      prepSteppingRelMillimeter(FEEDER, -len, false);  // unload to endstop
      runAndWait(FEEDER);
      delay(500);
      if (steppers[FEEDER].getEndstopHit())
        break;
      else
        retries--;
      changeFeederSpeed(smuffConfig.insertSpeed);
    } while (retries > 0);
    steppers[FEEDER].setEndstopState(endstopState);
    delay(500);
    unloadFromSelector(errmsg);
  #else
  if(smuffConfig.feedLoadState[toolSelected] == SPL_LOADED_TO_NOZZLE)
    len += smuffConfig.splitterDist;
  prepSteppingRelMillimeter(FEEDER, -len, true);
  runAndWait(FEEDER);
  #endif

  steppers[FEEDER].setIgnoreAbort(false);
  steppers[FEEDER].setAllowAccel(true);
  steppers[FEEDER].setMaxSpeed(curSpeed);
  delay(300);
  smuffConfig.feedLoadState[toolSelected] = NOT_LOADED;
  if(smuffConfig.webInterface) {
    printResponseP(P_M503S8, currentSerial);
    writefeedLoadState(getSerialInstance(currentSerial), true);
  }
  if(feederJammed) {
    snprintf_P(errmsg, MAX_ERR_MSG, P_FeederJammed);
    return false;
  }
  return true;
}

bool loadToSplitter(char* errmsg, bool showMessage) {
  // enable steppers if they were turned off
  if (!steppers[FEEDER].getEnabled())
    steppers[FEEDER].setEnabled(true);

  uint16_t curSpeed = steppers[FEEDER].getMaxSpeed();

  #if defined(USE_SPLITTER_ENDSTOPS)
    if(feedToEndstop(errmsg, showMessage)) {
      //__debugS(D, PSTR("Feeding to Splitter (%s mm)"), String(smuffConfig.bowdenLength).c_str());
      delay(200);
      changeFeederSpeed(curSpeed);
      steppers[FEEDER].setStepPositionMM(0);
      prepSteppingRelMillimeter(FEEDER, smuffConfig.bowdenLength, true);
      runAndWait(FEEDER);
    }
  #else
  // don't allow "feed to endstop" being interrupted
  steppers[FEEDER].setIgnoreAbort(true);
  positionRevolver();
  feederJammed = false;
  steppers[FEEDER].setStepPositionMM(0);
  prepSteppingRelMillimeter(FEEDER, smuffConfig.bowdenLength, true);
  runAndWait(FEEDER);
  #endif

  steppers[FEEDER].setIgnoreAbort(false);
  steppers[FEEDER].setAllowAccel(true);
  steppers[FEEDER].setMaxSpeed(curSpeed);
  delay(300);
  if(!feederJammed)
    smuffConfig.feedLoadState[toolSelected] = SPL_LOADED_TO_SPLITTER;
  if(smuffConfig.webInterface) {
    printResponseP(P_M503S8, currentSerial);
    writefeedLoadState(getSerialInstance(currentSerial), true);
  }
  return feederJammed ? false : true;
}

bool handleFeederStall(uint16_t *speed, int8_t *retries) {
  bool stat = true;
  // in case the stall detection is not configured...
  if (!smuffConfig.stepperStopOnStall[FEEDER]) {
    *retries -= 1;
    return true;
  }
  // did the Feeder stall?
  if (smuffConfig.stepperStopOnStall[FEEDER] && steppers[FEEDER].getStallDetected()) {
    stat = false;
    int16_t newSpeed;
    // yes, turn the speed down by 25%
    if (smuffConfig.speedsInMMS)
    {
      // speeds in mm/s need to go down
      newSpeed = (int16_t)(*speed * 0.75);
      if (newSpeed > 0)
        *speed = newSpeed;
      else
        *speed = 1; // set speed to absolute minimum
    }
    else {
      // whereas speeds in timer ticks need to go up
      newSpeed = (int16_t)(*speed * 1.25);
      if (newSpeed < 65500)
        *speed = newSpeed;
      else
        *speed = 65500; // set speed to absolute minimum
    }
    *retries -= 1;
    changeFeederSpeed(*speed);
    __debugS(I, PSTR("Feeder has stalled, slowing down speed to %d"), *speed);
    // counter used in testRun
    stallDetectedCountFeeder++; // for testrun only
  }
  return stat;
}

bool feedToNozzle(char* errmsg, bool showMessage) {
  bool stat = true;
  uint16_t speed = smuffConfig.maxSpeed[FEEDER];
  changeFeederSpeed(speed);

  #if defined(USE_DDE)
    uint16_t speed2 = smuffConfig.maxSpeed[DDE_FEEDER];
    changeDDEFeederSpeed(speed2);
  #endif
  
  int8_t retries = FEED_ERROR_RETRIES;
  if (smuffConfig.prusaMMU2 && smuffConfig.enableChunks) {
    // prepare to feed full speed in chunks
    double bLen = smuffConfig.bowdenLength;
    double len = bLen / smuffConfig.feedChunks;
    for (uint8_t i = 0; i < smuffConfig.feedChunks; i++) {
      prepSteppingRelMillimeter(FEEDER, len, true);
      runAndWait(FEEDER);
    }
  }
  else {
    double len = (smuffConfig.useSplitter ? smuffConfig.splitterDist : smuffConfig.bowdenLength) * .95;
    double remains = 0;
    steppers[FEEDER].setStepPositionMM(0);
    // prepare 95% to feed full speed
    do {
      #if !defined(USE_DDE)
        __debugS(D, PSTR("Feeding to nozzle 95%% (%s mm)"), String(len).c_str());
      #else
        __debugS(D, PSTR("Feeding to DDE 95%% (%s mm)"), String(len).c_str());
      #endif
      prepSteppingRelMillimeter(FEEDER, len - remains, true);
      runAndWait(FEEDER);
      // did the Feeder stall?
      stat = handleFeederStall(&speed, &retries);
      if (!stat) {
        remains = steppers[FEEDER].getStepPositionMM();
        //__debugS(D, PSTR("Len: %s  Remain: %s  To go: %s"), String(len).c_str(), String(remains).c_str(), String(len - remains).c_str());
      }
      #if !defined(USE_DDE)
        // check whether the 2nd endstop has triggered as well if configured to do so
        if (Z_END2_PIN > 0 && smuffConfig.useEndstop2 && stat) {
          if (!steppers[FEEDER].getEndstopHit(2))
            stat = false;
          if (!stat) {
            remains = steppers[FEEDER].getStepPositionMM();
            __debugS(D, PSTR("E-Stop2 failed. Len: %s  Remain: %s  To go: %s"), String(len).c_str(), String(remains).c_str(), String(len - remains).c_str());
          }
        }
      #endif
    } while (!stat && retries > 0);
    if (stat) {
      retries = FEED_ERROR_RETRIES;
      speed = smuffConfig.insertSpeed;
      double len = (smuffConfig.useSplitter ? smuffConfig.splitterDist : smuffConfig.bowdenLength) * .05;
      double remains = 0;
      uint32_t timeout = 20000;
      steppers[FEEDER].setStepPositionMM(0);
      #if defined(USE_DDE)
        if(smuffConfig.useCutter && smuffConfig.cutterOnTop) {
          // wait for the 2nd feeder endstop to trigger
          __debugS(D, PSTR("Endstop2 state: %s %s"), String(feederEndstop(2)).c_str(), feederEndstop(2)==1 ? " - waiting..." : "");
          while(feederEndstop(2)) {
            delay(500);
            timeout -= 500;
            if(timeout == 0) {
              showFeederBlockedMessage();
              break;
            }
          }
          if(timeout == 0)
            __debugS(D, PSTR("Aborted because of timeout"));
          else
            smuffConfig.feedLoadState[toolSelected] = LOADED_TO_DDE;
        }
      #else
      #endif
      sendStates(true);
      // feed rest of it slowly
      do {
        __debugS(D, PSTR("Feeding to nozzle remaining 5%% (%s mm)"), String(len - remains).c_str());
        changeFeederSpeed(speed);
        changeDDEFeederSpeed(speed);
        #if !defined (USE_DDE)
          prepSteppingRelMillimeter(FEEDER, len - remains, true);
          runAndWait(FEEDER);
          // did the Feeder stall again?
          stat = handleFeederStall(&speed, &retries);
          smuffConfig.feedLoadState[toolSelected] = smuffConfig.useSplitter ? SPL_LOADED_TO_NOZZLE : LOADED_TO_NOZZLE;
        #else
          __debugS(D, PSTR("Feeding both extruders"));
          steppers[FEEDER].setAllowAccel(false);
          steppers[DDE_FEEDER].setAllowAccel(false);
          steppers[DDE_FEEDER].setEnabled(true);
          delay(250);
          prepSteppingRelMillimeter(FEEDER, len - remains, true);
          prepSteppingRelMillimeter(DDE_FEEDER, len - remains, false);
          remainingSteppersFlag = _BV(FEEDER) | _BV(DDE_FEEDER);
          runAndWait(-1);
        #endif
        if (!stat) {
          remains = steppers[FEEDER].getStepPositionMM();
          //__debugS(D, PSTR("Len: %s  Remain: %s  To go: %s"), String(len).c_str(), String(remains).c_str(), String(len - remains).c_str());
        }
      } while (!stat && retries > 0);
      sendStates(true);

      // Purge DDE / feed to nozzle
      #if defined (USE_DDE)
        if(smuffConfig.reinforceLength > 0) {
          len = smuffConfig.reinforceLength;
          steppers[FEEDER].setIgnoreAbort(false);
          prepSteppingRelMillimeter(FEEDER, len, true);
          prepSteppingRelMillimeter(DDE_FEEDER, len, false);
          __debugS(D, PSTR("Reinforce length set, feeding both extruders yet another %s mm"), String(len).c_str());
          remainingSteppersFlag = _BV(FEEDER) | _BV(DDE_FEEDER);
          runAndWait(-1);
        }
        // reset max. speed on the DDE and acceleration on both, Feeder and DDE
        changeDDEFeederSpeed(speed2);
        steppers[FEEDER].setAllowAccel(true);
        steppers[DDE_FEEDER].setAllowAccel(true);
        setServoLid(SERVO_OPEN);
        delay(750);  // wait for the servo to react
        len = 0;
        // purge Direct Drive Extruder, if configured
        if(smuffConfig.usePurge && smuffConfig.purgeLength > 0) {
          len = smuffConfig.purgeLength;
          changeDDEFeederSpeed(smuffConfig.purgeSpeed);
          __debugS(D, PSTR("DDE purging %s mm"), String(len).c_str());
        }
        else if(smuffConfig.useCutter && !smuffConfig.cutterOnTop) {
          len = smuffConfig.ddeDist;
          __debugS(D, PSTR("DDE feeding to nozzle %s mm"), String(len).c_str());
        }
        if(len) {
          prepSteppingRelMillimeter(DDE_FEEDER, len, true);
          runAndWait(DDE_FEEDER);
        }
        if(smuffConfig.wipeBeforeUnload) {
          wipeNozzle();
        }
        steppers[DDE_FEEDER].setMaxSpeed(speed2);
        smuffConfig.feedLoadState[toolSelected] = smuffConfig.useSplitter ? SPL_LOADED_TO_NOZZLE : LOADED_TO_NOZZLE;
        sendStates(true);
      #endif
    }
  }
  return stat;
}

int controlSharedStepperOrExtFeeder() {
  #if defined(USE_DDE)
    switchFeederStepper(INTERNAL);
    __debugS(DEV3, PSTR("Using DDE, relay on I"));
    return 1;
  #else
    if (!smuffConfig.isSharedStepper && smuffConfig.extControlFeeder) {
      positionRevolver();
      signalDuetReady();
      __debugS(DEV3, PSTR("No shared stepper but external control"));
      return 2;
    }
    if (smuffConfig.isSharedStepper) {
      switchFeederStepper(INTERNAL);
      __debugS(DEV3, PSTR("Shared stepper, relay on I"));
      return 1;
    }
  #endif
  return 0;
}

void releaseCutter() {
  if (smuffConfig.useCutter) {
    // release the cutter just in case
    setServoPos(SERVO_CUTTER, smuffConfig.cutterOpen);
    delay(10);
    disableServo(SERVO_CUTTER);
    __debugS(DEV3, PSTR("Cutter opened, servo disabled"));
  }
}

bool loadFilament(char* errmsg, bool showMessage) {
  signalDuetBusy();
  if (toolSelected == -1) {
    signalNoTool();
    signalDuetReady();
    smuffConfig.feedLoadState[toolSelected] = NOT_LOADED;
    return false;
  }
  
  if(controlSharedStepperOrExtFeeder() == 2) {
    return true;
  }
  positionRevolver();
  releaseCutter();
  setParserBusy();

  uint16_t curSpeed = steppers[FEEDER].getMaxSpeed();
  if(!smuffConfig.useSplitter) {
    // feed filament until it hits the feeder endstop
    if (!feedToEndstop(errmsg, showMessage)) {
      setParserReady();
      return false;
    }
  }
  else {
    if(smuffConfig.feedLoadState[toolSelected] == NOT_LOADED) {
      loadToSplitter(errmsg, showMessage);
    }
  }
  if(smuffConfig.webInterface) {
    printResponseP(P_M503S8, currentSerial);
    writefeedLoadState(getSerialInstance(currentSerial), true);
  }

  steppers[FEEDER].setStepsTaken(0);
  // move filament until it gets to the nozzle
  if (!feedToNozzle(errmsg, showMessage)) {
    setParserReady();
    return false;
  }
  if(smuffConfig.useSplitter) {
    smuffConfig.feedLoadState[toolSelected] = SPL_LOADED_TO_NOZZLE;
  }
  if(smuffConfig.webInterface) {
    printResponseP(P_M503S8, currentSerial);
    writefeedLoadState(getSerialInstance(currentSerial), true);
  }
  sendStates(true);

  #if !defined(USE_DDE)
    if (smuffConfig.reinforceLength > 0 && !steppers[FEEDER].getAbort()) {
      resetRevolver();
      changeFeederSpeed(smuffConfig.insertSpeed);
      delay(150);
      prepSteppingRelMillimeter(FEEDER, smuffConfig.reinforceLength, true);
      runAndWait(FEEDER);
    }
    purgeFilament(0);
  #endif

  #if defined(USE_DDE)
    __debugS(DEV2, PSTR("DDE endstop state after load: %s hit: %s"), steppers[DDE_FEEDER].getEndstopState() ? P_High : P_Low, steppers[DDE_FEEDER].getEndstopHit(1) ? P_Yes : P_No);
  #endif

  steppers[FEEDER].setMaxSpeed(curSpeed);
  dataStore.stepperPos[FEEDER] = steppers[FEEDER].getStepPosition();
  saveStore();
  runHomeAfterFeed();
  signalDuetReady();
  setParserReady();
  return true;
}

void purgeFilament(double forceLen) {
  if (smuffConfig.usePurge && smuffConfig.purgeLength > 0 && !steppers[FEEDER].getAbort()) {
    #if !defined(USE_DDE)
    positionRevolver();
    #endif
    uint16_t curSpeed = steppers[FEEDER].getMaxSpeed();
    uint16_t curSpeed2 = steppers[DDE_FEEDER].getMaxSpeed();
    changeFeederSpeed(smuffConfig.purgeSpeed);
    changeDDEFeederSpeed(smuffConfig.purgeSpeed);
    double len = forceLen == 0 ? smuffConfig.purgeLength : forceLen;
    if (smuffConfig.purges[toolSelected] != 100 && forceLen == 0) {
      len *= ((double)smuffConfig.purges[toolSelected] / 100);
      __debugS(DEV2, PSTR("Purge length adjusted by: %d %%"), smuffConfig.purges[toolSelected]);
    }
    drawPurgingMessage(len, toolSelected);
    __debugS(DEV2, PSTR("Purging: %s mm"), String(len).c_str());
    #if !defined(USE_DDE)
    steppers[FEEDER].setStepPositionMM(0);
    prepSteppingRelMillimeter(FEEDER, len, true);
    runAndWait(FEEDER);
    #else
    steppers[DDE_FEEDER].setStepPositionMM(0);
    prepSteppingRelMillimeter(DDE_FEEDER, len, true);
    runAndWait(DDE_FEEDER);
    #endif
    delay(250);
    steppers[FEEDER].setMaxSpeed(curSpeed);
    steppers[DDE_FEEDER].setMaxSpeed(curSpeed2);
    drawPurgingMessage(0, 0);
  }
}

/*
  This method is used to feed the filament Prusa style (L command on MMU2).
  If first feeds the filament until the endstop is hit, then
  it pulls it back again.
*/
bool loadFilamentPMMU2(char* errmsg, bool showMessage) {
  signalDuetBusy();
  if (toolSelected == -1) {
    signalNoTool();
    signalDuetReady();
    return false;
  }

  if(controlSharedStepperOrExtFeeder() == 2)
    return true;
  positionRevolver();

  if (smuffConfig.useCutter) {
    // release the cutter just in case
    setServoPos(SERVO_CUTTER, smuffConfig.cutterOpen);
  }

  setParserBusy();
  uint16_t curSpeed = steppers[FEEDER].getMaxSpeed();
  // move filament until it hits the feeder endstop
  if (!feedToEndstop(errmsg, showMessage)) {
    return false;
  }

  steppers[FEEDER].setStepsTaken(0);
  // inhibit interrupts at this step
  steppers[FEEDER].setIgnoreAbort(true);
  // now pull it back again
  changeFeederSpeed(smuffConfig.insertSpeed);
  prepSteppingRelMillimeter(FEEDER, -smuffConfig.selectorDistance, true);
  runAndWait(FEEDER);

  if (smuffConfig.reinforceLength > 0 && !steppers[FEEDER].getAbort()) {
    resetRevolver();
    prepSteppingRelMillimeter(FEEDER, smuffConfig.reinforceLength, true);
    runAndWait(FEEDER);
  }
  steppers[FEEDER].setIgnoreAbort(false);

  steppers[FEEDER].setMaxSpeed(curSpeed);
  dataStore.stepperPos[FEEDER] = steppers[FEEDER].getStepPosition();
  saveStore();
  runHomeAfterFeed();
  signalDuetReady();
  setParserReady();
  return true;
}

bool unloadFromDDE() {
  
  bool stat = false;
  int8_t retries = FEED_ERROR_RETRIES;
  double len = smuffConfig.ddeDist * 1.2;   // retract more than configured, endstop will terminate retraction

  bool ddeEndstopState = steppers[DDE_FEEDER].getEndstopState();
  // don't feed DDE in reverse if Cutter is being used and not sitting on top
  if(!smuffConfig.useCutter || (smuffConfig.useCutter && !smuffConfig.cutterOnTop)) {
    __debugS(DEV2, PSTR("DDE endstop state: %s hit: %s"), ddeEndstopState ? P_High : P_Low, steppers[DDE_FEEDER].getEndstopHit(1) ? P_Yes : P_No);
    if(!steppers[DDE_FEEDER].getEndstopHit(1)) {
      // no filament in DDE?
      __debugS(DEV2, PSTR("DDE endstop NOT hit, skipping unloading from DDE"));
      stat = true;
    }
    else {
      // invert endstop trigger state for reverse feeding
      steppers[DDE_FEEDER].setEndstopState(!ddeEndstopState);
      changeDDEFeederSpeed(smuffConfig.maxSpeed[DDE_FEEDER]);
      do {
        steppers[DDE_FEEDER].setEnabled(true);
        steppers[DDE_FEEDER].setStepPositionMM(0);
        setServoLid(SERVO_OPEN);                    // just in case
        // only on Direct Drive Extruder
        if(len > 0) {
          __debugS(D, PSTR("Retracting from DDE %s mm"), String(len).c_str());
          prepSteppingRelMillimeter(DDE_FEEDER, -len, false);
          runAndWait(DDE_FEEDER);
          double moved = steppers[DDE_FEEDER].getStepPositionMM();
          __debugS(I, PSTR("DDE has retracted %s mm"), String(fabs(moved)).c_str());
          // check if DDE Feeder endstop has triggered
          stat = steppers[DDE_FEEDER].getEndstopHit(1);
          if(!stat) {
            __debugS(DEV2, PSTR("DDE endstop has not triggered, retrying (%d)"), retries);
            retries--;
          }
        }
        else {
          __debugS(I, PSTR("No 'DDEDist' configured, not retracting from DDE"));
          retries = 0;
          break;
        }
      } while(!stat && retries > 0);
      // restore endstop trigger state
      steppers[DDE_FEEDER].setEndstopState(ddeEndstopState);
    }
    if(stat) {
      // move it another 5 mm (or reinforceLength if set) out of the DDE Feeder, just to make sure the SMuFF is able to retract
      len = smuffConfig.reinforceLength == 0 ? 5 : smuffConfig.reinforceLength;
      __debugS(D, PSTR("DDE 2nd retraction (%s mm)"), String(len).c_str());
      prepSteppingRelMillimeter(DDE_FEEDER, -len, true);
      runAndWait(DDE_FEEDER);
      smuffConfig.feedLoadState[toolSelected] = LOADED_TO_DDE;
      setServoLid(SERVO_CLOSED);
    }
  }
  else {
    if(smuffConfig.useCutter && smuffConfig.cutterOnTop) {
      __debugS(DEV3, PSTR("Cutter sitting on top of DDE, not retracting from DDE"));
    }
    setServoLid(SERVO_CLOSED);
    stat = true;
  }
  sendStates(true);
  return stat;
}

bool unloadFromNozzle(char* errmsg, bool showMessage) {
  bool stat = true;
  uint16_t speed = smuffConfig.maxSpeed[FEEDER];
  int8_t retries = FEED_ERROR_RETRIES;
  double posNow = steppers[FEEDER].getStepPositionMM();

  // Feeder endstop not triggered means: Filament not loaded!
  // In such case, avoid any retraction
  if(!steppers[FEEDER].getEndstopHit()) {
    __debugS(W, PSTR("Feeder endstop was not triggered. Skipping unloading from nozzle."));
  }
  else {

    if (smuffConfig.prusaMMU2 && smuffConfig.enableChunks) {
      __debugS(I, PSTR("Unloading in %d chunks"), smuffConfig.feedChunks);
      // prepare to unfeed 3 times the bowden length full speed in chunks
      double bLen = -((smuffConfig.useSplitter ? smuffConfig.splitterDist : smuffConfig.bowdenLength * 3));
      double len = bLen / smuffConfig.feedChunks;
      for (uint8_t i = 0; i < smuffConfig.feedChunks; i++) {
        prepSteppingRelMillimeter(FEEDER, len);
        runAndWait(FEEDER);
      }
    }
    else {
      #if defined(USE_DDE)
        if(!unloadFromDDE()) {
          snprintf_P(errmsg, MAX_ERR_MSG, P_CantUnloadDDE);
          return false;
        }
      #endif
      // invert Feeder endstop state for unloading
      bool endstopState = steppers[FEEDER].getEndstopState();
      steppers[FEEDER].setEndstopState(!endstopState);
      setServoLid(SERVO_CLOSED);      // bugfix, Lid not being closed on DDE and CutterOnTop
      do {
        steppers[FEEDER].setAllowAccel(true);
      
        // prepare 110% to retract with full speed
        double len = (smuffConfig.useSplitter ? smuffConfig.splitterDist : smuffConfig.bowdenLength * 1.1);
        __debugS(D, PSTR("Retracting from Feeder %s mm"), String(fabs(len)).c_str());
        steppers[FEEDER].setStepPositionMM(0);
        prepSteppingRelMillimeter(FEEDER, -len, smuffConfig.useSplitter ? true : false);
        runAndWait(FEEDER);
        double moved = steppers[FEEDER].getStepPositionMM();
        __debugS(I, PSTR("Feeder has retracted %s mm"), String(fabs(moved)).c_str());
        
        // first check: If endstop not got hit, repeat retracting
        // bug fix
        stat = steppers[FEEDER].getEndstopHit();

        if(stat) {
          // did the Feeder stall?
          stat = handleFeederStall(&speed, &retries);
          if(!smuffConfig.useSplitter) {
            #if !defined(USE_DDE)
              // check whether the 2nd endstop has triggered as well if configured to do so
              if (Z_END2_PIN > 0 && smuffConfig.useEndstop2 && stat) {
                // endstop still triggered?
                if (steppers[FEEDER].getEndstopHit(2)) {
                  retries--;
                  stat = false;
                  __debugS(DEV, PSTR("E-Stop2 trigger failed. Retrying %d more times"), retries);
                  // if only one retry is left, try cutting the filament again
                  if (retries == 1) {
                    cutFilament(false);
                  }
                }
              }
            #endif
          }
        }
        else {
          retries--;
          __debugS(DEV, PSTR("E-Stop trigger failed. Retrying %d more times"), retries);
          // if only one retry is left, try cutting the filament again
          if (retries == 1) {
            cutFilament(false);
          }
        }
      } while (!stat && retries > 0);
      steppers[FEEDER].setEndstopState(endstopState);
    }
  }

  /*
  // do some calculations in order to give some hints
  double posEnd = posNow - steppers[FEEDER].getStepPositionMM();
  // calculate supposed length;
  double cutterLen = (smuffConfig.useCutter && smuffConfig.cutterOnTop) ? smuffConfig.cutterLength : 0.0;
  double dist = smuffConfig.bowdenLength + cutterLen - smuffConfig.unloadRetract;
  __debugS(I, PSTR("Unload from Nozzle distance set to %s mm"), String(dist).c_str());
  */

  if(smuffConfig.useSplitter) {
    smuffConfig.feedLoadState[toolSelected] = SPL_LOADED_TO_SPLITTER;
  }
  else {
    if(stat)
      smuffConfig.feedLoadState[toolSelected] = LOADED_TO_SELECTOR;
  }
  if(smuffConfig.webInterface) {
    printResponseP(P_M503S8, currentSerial);
    writefeedLoadState(getSerialInstance(currentSerial), true);
  }
  // reset feeder to max. speed
  changeFeederSpeed(smuffConfig.maxSpeed[FEEDER]);
  changeDDEFeederSpeed(smuffConfig.maxSpeed[DDE_FEEDER]);
  sendStates(true);
  return stat;
}

bool unloadFromSelector(char* errmsg) {
  bool stat = true;
  // only if the unload hasn't been aborted yet, unload from Selector
  if (steppers[FEEDER].getAbort() == false) {
    delay(250);
    int8_t retries = FEED_ERROR_RETRIES;
    uint16_t speed = smuffConfig.insertSpeed;
    //double dist = smuffConfig.selectorDistance;
    double dist = smuffConfig.selectorUnloadDist;
    steppers[FEEDER].setStepPosition(0);
    do {
      changeFeederSpeed(speed);
      __debugS(DEV, PSTR("Retracting from Selector: %s mm"), String(dist).c_str());
      // retract the "selector distance"
      prepSteppingRelMillimeter(FEEDER, -dist, true);
      runAndWait(FEEDER);
      double moved = steppers[FEEDER].getStepPositionMM();
      __debugS(DEV2, PSTR("Feeder has retracted %s mm from Selector"), String(fabs(moved)).c_str());
      // did the Feeder stall?
      stat = handleFeederStall(&speed, &retries);
      if(retries == 0)
        break;
    } while (!stat);
    __debugS(DEV3, PSTR("Feeder position now %s mm"), String(steppers[FEEDER].getStepPositionMM()).c_str());
  }
  else {
    __debugS(D, PSTR("Feeder Abort was set, not unloading from Selector!"));
  }
  if(stat)
    smuffConfig.feedLoadState[toolSelected] = NOT_LOADED;
  sendStates(true);
  return stat;
}

void wipeNozzle() {
  char errmsg[MAX_ERR_MSG];
  if (smuffConfig.wipeBeforeUnload)
    G12("G12", "", 255, errmsg);
}

void cutFilament(bool keepClosed /* = true */) {
  // use the filament cutter?
  if (smuffConfig.useCutter) {
    enableServo(SERVO_CUTTER);
    // run the cutter 3 times to be on the save side
    setServoPos(SERVO_CUTTER, smuffConfig.cutterClose);
    delay(500);
    setServoPos(SERVO_CUTTER, smuffConfig.cutterOpen);
    delay(300);
    setServoPos(SERVO_CUTTER, smuffConfig.cutterClose);
    delay(300);
    setServoPos(SERVO_CUTTER, smuffConfig.cutterOpen);
    delay(200);
    setServoPos(SERVO_CUTTER, smuffConfig.cutterClose);
    if (!keepClosed) {
      delay(200);
      setServoPos(SERVO_CUTTER, smuffConfig.cutterOpen);
      delay(100);
      disableServo(SERVO_CUTTER);
    }
  }
}

void (*defaultDDEEndstopFunc)() = nullptr;
bool defaultDDEEndstopState;
bool asyncDDE = false;

void endstopDDEEvent() {
  __debugS(I, PSTR("DDE endstop triggered at position: %s mm"), String(steppers[DDE_FEEDER].getStepPositionMM()).c_str());
  asyncDDE = false;
  // revert endstop trigger inversion
  steppers[DDE_FEEDER].setEndstopState(defaultDDEEndstopState);
  // reset max speed to default
  steppers[DDE_FEEDER].setMaxSpeed(smuffConfig.maxSpeed[DDE_FEEDER]);
  // reset endstop event
  steppers[DDE_FEEDER].endstopFunc = defaultDDEEndstopFunc;
}

// purge out the piece of filament that's stuck in between the cutter and the 
// DDE gears in order to free the DDE (endstop).
void purgeDDE() {
  // This applies only if the Cutter is being utilized
  if(smuffConfig.purgeDDE) {
    // only if the endstop is still triggering
    if(steppers[DDE_FEEDER].getEndstopHit()) {
      // invert endstop trigger state for purging
      defaultDDEEndstopState = steppers[DDE_FEEDER].getEndstopState();
      steppers[DDE_FEEDER].setEndstopState(!defaultDDEEndstopState);
      uint16_t curSpeed = steppers[DDE_FEEDER].getMaxSpeed();
      steppers[DDE_FEEDER].setMaxSpeed(smuffConfig.purgeSpeed);
      steppers[DDE_FEEDER].setEnabled(true);
      steppers[DDE_FEEDER].setStepPositionMM(0);
      double len = smuffConfig.ddeDist == 0 ? 100 : smuffConfig.ddeDist * 2;
      __debugS(I, PSTR("Purging DDE (max. %s mm)..."),String(len).c_str());
      // save the endstop hit function
      defaultDDEEndstopFunc = steppers[DDE_FEEDER].endstopFunc;
      steppers[DDE_FEEDER].endstopFunc = endstopDDEEvent;
      // feed twice the "DDE Distance" (endstop is supposed to interrupt purging)
      prepSteppingRelMillimeter(DDE_FEEDER, len, false);
      asyncDDE = true;
      runNoWait(DDE_FEEDER);  // purge DDE asynchronously in background
    }
    else {
      __debugS(I, PSTR("Endstop 2 not hit, skipping purge..."));
    }
  }
}

bool unloadFilament(char* errmsg) {
  signalDuetBusy();
  if (toolSelected == -1) {
    signalNoTool();
    signalDuetReady();
    return false;
  }
  
  if(controlSharedStepperOrExtFeeder() == 2)
    return true;

  steppers[FEEDER].setStepsTaken(0);
  setParserBusy();
  if (!steppers[FEEDER].getEnabled())
    steppers[FEEDER].setEnabled(true);

  #if !defined(USE_DDE)
    positionRevolver();
  #endif

  uint16_t curSpeed = steppers[FEEDER].getMaxSpeed();

  #if !defined(USE_DDE)
    if (smuffConfig.unloadRetract != 0) {
      // do the unload retraction only if filament is fully loaded
      if (smuffConfig.feedLoadState[toolSelected] != ((smuffConfig.useSplitter) ? SPL_LOADED_TO_NOZZLE : LOADED_TO_NOZZLE)) {
        __debugS(I, PSTR("Not loaded to nozzle (feedLoadState = %x), unload retraction skipped"), smuffConfig.feedLoadState[toolSelected]);
      }
      else {
        __debugS(I, PSTR("Unload retract: %s mm"), String(smuffConfig.unloadRetract).c_str());
        prepSteppingRelMillimeter(FEEDER, -smuffConfig.unloadRetract, true);
        runAndWait(FEEDER);
        if (smuffConfig.unloadPushback != 0) {
          __debugS(I, PSTR("Unload pushback: %s mm"), String(smuffConfig.unloadPushback).c_str());
          changeFeederSpeed(smuffConfig.insertSpeed);
          prepSteppingRelMillimeter(FEEDER, smuffConfig.unloadPushback, true);
          runAndWait(FEEDER);
          delay(smuffConfig.pushbackDelay);
          steppers[FEEDER].setMaxSpeed(curSpeed);
        }
      }
    }
  #endif
  // wipe nozzle if configured
  wipeNozzle();
  // cut the filament if configured likewise and keep it engaged (closed)
  cutFilament();

  #if defined(USE_DDE)
    purgeDDE();
  #endif
  // unload until Selector endstop gets released
  int8_t retries = FEED_ERROR_RETRIES;
  do {
    if(unloadFromNozzle(errmsg, false))
      break;
    if(smuffConfig.useSplitter) {
      break;
    }
    retries--;
  } while (retries > 0);

  if(!smuffConfig.useSplitter)
    unloadFromSelector(errmsg);

  releaseCutter();

  feederJammed = false;
  steppers[FEEDER].setIgnoreAbort(false);
  steppers[FEEDER].setMaxSpeed(curSpeed);
  steppers[FEEDER].setStepPosition(0);
  steppers[FEEDER].setAbort(false);

  #if defined(USE_DDE)
    __debugS(DEV2, PSTR("DDE endstop state after unload: %s hit: %s"), steppers[DDE_FEEDER].getEndstopState() ? P_High : P_Low, steppers[DDE_FEEDER].getEndstopHit(1) ? P_Yes : P_No);
  #endif

  dataStore.stepperPos[FEEDER] = steppers[FEEDER].getStepPosition();
  saveStore();
  runHomeAfterFeed();
  signalDuetReady();
  setParserReady();
  return true;
}

bool nudgeBackFilament() {
  if (toolSelected == -1)
    return false;

  if(controlSharedStepperOrExtFeeder() == 2)
    return true;
  positionRevolver();

  steppers[FEEDER].setStepsTaken(0);
  if (!steppers[FEEDER].getEnabled())
    steppers[FEEDER].setEnabled(true);

  uint16_t curSpeed = steppers[FEEDER].getMaxSpeed();
  changeFeederSpeed(smuffConfig.insertSpeed);
  prepSteppingRelMillimeter(FEEDER, smuffConfig.insertLength);
  runAndWait(FEEDER);
  steppers[FEEDER].setMaxSpeed(curSpeed);

  dataStore.stepperPos[FEEDER] = steppers[FEEDER].getStepPosition();
  saveStore();
  runHomeAfterFeed();
  return true;
}

void runHomeAfterFeed() {

  if (smuffConfig.homeAfterFeed) {
    if (smuffConfig.revolverIsServo)
      setServoLid(SERVO_OPEN);
    else
      steppers[REVOLVER].home();
  }
  steppers[FEEDER].setAbort(false);
  #if defined(USE_DDE)
  switchFeederStepper(EXTERNAL);
  #else
  if (smuffConfig.isSharedStepper)
    switchFeederStepper(EXTERNAL);
  #endif
}

bool nudgeSelector(int8_t axis, int16_t dist, const char* text) {
  delay(1000);
  steppers[axis].setEnabled(true);
  prepStepping(axis, dist, true, true);
  remainingSteppersFlag |= _BV(axis);
  runAndWait(-1);
  __debugS(DEV, PSTR("%s: %d"), text, steppers[axis].getStallCount());
  steppers[axis].setEnabled(false);
  return (steppers[axis].getStallCount() > (uint32_t)steppers[axis].getStallThreshold());
}

void handleStall(int8_t axis) {
  const char P_StallHandler[] PROGMEM = {"Stall handler: %s"};
  __debugS(DEV, P_StallHandler, " Has triggered on %c-axis", 'X' + axis);
  
  // check if stall must be handled
  if (!steppers[axis].getStopOnStallDetected())
    return;

  __debugS(DEV, P_StallHandler, "Stopped on stall");
  
  // save speed/acceleration settings
  uint16_t maxSpeed = steppers[axis].getMaxSpeed();
  uint16_t accel = steppers[axis].getAcceleration();
  // slow down speed/acceleration for next moves
  steppers[axis].setMaxSpeed(accel * 2);
  steppers[axis].setAcceleration(accel * 2);

  // in order to determine where the stall happens...
  // try to move 5mm to the left
  bool stallLeft = nudgeSelector(axis, 5, "Left");
  // try to move 5mm to the right
  bool stallRight = nudgeSelector(axis, -5, "Right");
  
  if (stallLeft && stallRight)
    __debugS(DEV, P_StallHandler, "Stalled center");
  if (stallLeft && !stallRight)
    __debugS(DEV, P_StallHandler, "Stalled left");
  if (!stallLeft && stallRight)
    __debugS(DEV, P_StallHandler, "Stalled right");

  nudgeBackFilament();
  __debugS(DEV, P_StallHandler, "Feeder nudged back");
  delay(1000);
  
  if (axis != FEEDER) {
    // Feeder can't be homed
    moveHome(axis, false, false);
    __debugS(D, P_StallHandler, "%c-Axis Homed", 'X' + axis);
  }
  else {
    // TODO: add stall handling for Feeder
  }
  
  // reset speed/acceleration
  steppers[axis].setMaxSpeed(maxSpeed);
  steppers[axis].setAcceleration(accel);
  delay(1000);
}

bool selectTool(int8_t ndx, char* errmsg, bool showMessage) {
  char _msg1[256];
  char _tmp[40];

  if (ndx < 0 || ndx >= MAX_TOOLS) {
    sprintf_P(_msg1, P_WrongTool, ndx);
    if (showMessage) {
      userBeep();
      drawUserMessage(_msg1);
    }
    else {
      snprintf(errmsg, MAX_ERR_MSG, _msg1);
      strcat(errmsg,"\n");
    }
    return false;
  }

  ndx = swapTools[ndx];
  if (feederJammed) {
    beep(4);
    sprintf_P(_msg1, P_FeederJammed);
    strcat_P(_msg1, P_Aborting);
    if (showMessage) {
      drawUserMessage(_msg1);
    }
    feederJammed = false;
    snprintf(errmsg, MAX_ERR_MSG, _msg1);
    strcat(errmsg,"\n");
    return false;
  }
  //signalSelectorBusy();

  if (toolSelected == ndx) {
    // tool is the one we already have selected, do nothing
    if (!smuffConfig.extControlFeeder)
    {
      userBeep();
      sprintf_P(_msg1, P_ToolAlreadySet);
      drawUserMessage(_msg1);
      snprintf(errmsg, MAX_ERR_MSG, _msg1);
      strcat(errmsg,"\n");
    }
    return true;
  }
  toolPending = ndx;
  setFastLEDToolIndex(toolPending, smuffConfig.toolColor, true);

  if (!steppers[SELECTOR].getEnabled())
    steppers[SELECTOR].setEnabled(true);

  if (showMessage)
  {
    if(!smuffConfig.useSplitter) {
      while (feederEndstop()) {
        if (!showFeederLoadedMessage())
          return false;
      }
    }
  }
  else
  {
    if(smuffConfig.useSplitter) {
      if(smuffConfig.feedLoadState[toolSelected] == SPL_LOADED_TO_NOZZLE) {
        __debugS(DEV, PSTR("Unloading from Nozzle (Splitter)"));
        unloadFromNozzle(errmsg, showMessage);
        printResponseP(P_M503S8, currentSerial);
        writefeedLoadState(getSerialInstance(currentSerial), true);
      }
    }
    else {
      __debugS(DEV2, PSTR("Not using Splitter"));
      if (!smuffConfig.extControlFeeder && feederEndstop()) {
        unloadFilament(errmsg);
      }
      else if (smuffConfig.extControlFeeder && feederEndstop()) {
        beep(4);
        if (smuffConfig.sendActionCmds)
        {
          // send action command to indicate a jam has happend and
          // the controller shall wait
          sprintf_P(_tmp, P_Action, P_ActionWait);
          printResponse(_tmp, 0);
          printResponse(_tmp, 1);
          printResponse(_tmp, 2);
          __debugS(DEV, PSTR("Sent action command '%s'"), _tmp);
        }
        while (feederEndstop())
        {
          char _errmsg[MAX_ERR_MSG];
          moveHome(REVOLVER, false, false); // home Revolver
          M18("M18", "", 255, _errmsg);              // motors off
          bool stat = showFeederFailedMessage(0);
          if (!stat)
            return false;
          if (smuffConfig.unloadCommand != nullptr && strlen(smuffConfig.unloadCommand) > 0)
          {
            if (CAN_USE_SERIAL2)
            {
              Serial2.print(smuffConfig.unloadCommand);
              Serial2.print("\n");
              __debugS(DEV, PSTR("Feeder jammed, sent unload command '%s'"), smuffConfig.unloadCommand);
            }
          }
        }
        if (smuffConfig.sendActionCmds)
        {
          // send action command to indicate jam cleared, continue printing
          sprintf_P(_tmp, P_Action, P_ActionCont);
          printResponse(_tmp, 0);
          printResponse(_tmp, 1);
          printResponse(_tmp, 2);
          __debugS(DEV, PSTR("Sent action command '%s'"), _tmp);
        }
      }
    }
  }
  if (smuffConfig.revolverIsServo) {
    // release servo prior moving the selector
    setServoLid(SERVO_OPEN);
  }
  else {
    #if defined(SMUFF_V6S)
    setServoLid(SERVO_OPEN);
    #endif
  }
  __debugS(DEV, PSTR("Selecting tool %d, current tool is %d"), ndx, toolSelected);
  setParserBusy();
  drawSelectingMessage(ndx);
  //__debugS(DEV3, PSTR("Message shown on display"));
  uint16_t speed = steppers[SELECTOR].getMaxSpeed();

  uint8_t retry = 3;
  bool posOk = false;
  do
  {
    steppers[SELECTOR].resetStallDetected();
    prepSteppingAbsMillimeter(SELECTOR, smuffConfig.firstToolOffset + (ndx * smuffConfig.toolSpacing));
    remainingSteppersFlag |= _BV(SELECTOR);
#if !defined(SMUFF_V5)
    if (!smuffConfig.resetBeforeFeed)
    {
      prepSteppingAbs(REVOLVER, smuffConfig.firstRevolverOffset + (ndx * smuffConfig.revolverSpacing), true);
      remainingSteppersFlag |= _BV(REVOLVER);
    }
    runAndWait(-1);
#else
    runAndWait(SELECTOR);
#endif
    __debugS(DEV2, PSTR("Selector now in position: %d"), ndx);
    if (smuffConfig.stepperStall[SELECTOR] > 0) {
      __debugS(DEV2, PSTR("Selector stall count: %d"), steppers[SELECTOR].getStallCount());
    }
    if (steppers[SELECTOR].getStallDetected()) {
      posOk = false;
      handleStall(SELECTOR);
      stallDetectedCountSelector++;
    }
    else
      posOk = true;
    retry--;
    if (!retry)
      break;
  } while (!posOk);
  steppers[SELECTOR].setMaxSpeed(speed);
  if (posOk) {
    toolSelected = ndx;
    #if defined(USE_SPLITTER_ENDSTOPS)
    splitterMux.setTool(toolSelected);
    #endif
    dataStore.tool = toolSelected;
    dataStore.stepperPos[SELECTOR] = steppers[SELECTOR].getStepPosition();
    dataStore.stepperPos[REVOLVER] = steppers[REVOLVER].getStepPosition();
    dataStore.stepperPos[FEEDER] = steppers[FEEDER].getStepPosition();
    saveStore();
    setFastLEDToolIndex(toolSelected, smuffConfig.toolColor, true);

    if (showMessage && (!smuffConfig.extControlFeeder || smuffConfig.isSharedStepper)) {
      showFeederLoadMessage();
    }
    if (smuffConfig.extControlFeeder) {
      //__debugS(DEV2, PSTR("Resetting Revolver"));
      resetRevolver();
      //__debugS(DEV2, PSTR("Revolver reset done"));
    }
  }
  setParserReady();
  __debugS(DEV, PSTR("Finished selecting tool %d"), ndx);
  toolPending = -1;
  return posOk;
}

void resetRevolver() {
  moveHome(REVOLVER, false, false);
  if (toolSelected >= 0 && toolSelected <= smuffConfig.toolCount - 1) {
    if (!smuffConfig.revolverIsServo) {
      #if !defined(SMUFF_V6S)
        prepSteppingAbs(REVOLVER, smuffConfig.firstRevolverOffset + (toolSelected * smuffConfig.revolverSpacing), true);
        runAndWait(REVOLVER);
      #else
        positionRevolver();
      #endif
    }
    else {
      setServoLid(SERVO_CLOSED);
    }
  }
}

void setStepperSteps(int8_t index, long steps, bool ignoreEndstop) {
  // make sure the servo is in off position before the Selector gets moved
  // ... just in case... you never know...
  if (smuffConfig.revolverIsServo && index == SELECTOR) {
    if (lidOpen) {
      //__debugS(D, PSTR("Positioning servo to: %d (OPEN)"), smuffConfig.revolverOffPos);
      setServoLid(SERVO_OPEN);
    }
  }
  if (steps != 0)
    steppers[index].prepareMovement(steps, ignoreEndstop);
}

void prepSteppingAbs(int8_t index, long steps, bool ignoreEndstop) {
  long pos = steppers[index].getStepPosition();
  long _steps = steps - pos;
  setStepperSteps(index, _steps, ignoreEndstop);
}

void prepSteppingAbsMillimeter(int8_t index, double millimeter, bool ignoreEndstop) {
  uint16_t stepsPerMM = steppers[index].getStepsPerMM();
  long steps = (long)((double)millimeter * stepsPerMM);
  long pos = steppers[index].getStepPosition();
  __debugS(DEV3, PSTR("prepSteppingAbsMillimeter: StepsMM: %ld  MM: %s  Dist: %s  Steps: %ld  Pos: %ld"), stepsPerMM, String((double)millimeter).c_str(), String((double)millimeter * stepsPerMM).c_str(), steps, pos);
  setStepperSteps(index, steps - pos, ignoreEndstop);
}

void prepSteppingRel(int8_t index, long steps, bool ignoreEndstop) {
  setStepperSteps(index, steps, ignoreEndstop);
}

void prepSteppingRelMillimeter(int8_t index, double millimeter, bool ignoreEndstop) {
  uint16_t stepsPerMM = steppers[index].getStepsPerMM();
  long steps = (long)((double)millimeter * stepsPerMM);
  __debugS(DEV3, PSTR("prepSteppingRelMillimeter: StepsMM: %ld  MM: %s  Dist: %s  Steps: %ld"), stepsPerMM, String((double)millimeter).c_str(), String((double)millimeter * stepsPerMM).c_str(), steps);
  setStepperSteps(index, steps, ignoreEndstop);
}

void printPeriodicalState(int8_t serial) {

  if(isListingFile) // avoid sending states while file listing is in progress 
    return;

  char tmp[256];
  const char *_triggered = "on";
  const char *_open = "off";
  int8_t tool = getToolSelected();

  snprintf_P(tmp, ArraySize(tmp)-1, PSTR("echo: states: T: T%d\tS: %s\tR: %s\tF: %s\tF2: %s\tTMC: %c%s\tSD: %s\tSC: %s\tLID: %s\tI: %s\tSPL: %d\tRLY: %s\tJAM: %s\n"),
            tool,
            selectorEndstop() ? _triggered : _open,
            revolverEndstop() ? _triggered : _open,
            feederEndstop(1) ? _triggered : _open,
            feederEndstop(2) ? _triggered : _open,
            isUsingTmc ? '+' : '-',
            tmcWarning ? _triggered : _open,
            sdRemoved ? _triggered : _open,
            settingsChanged ? _triggered : _open,
            !lidOpen ? _triggered : _open,
            isIdle ? _triggered : _open,
            smuffConfig.feedLoadState[tool],
            smuffConfig.externalStepper ? "E" : "I",
            feederJammed ? _triggered : _open
  );
  printResponse(tmp, serial);
}

int8_t getToolSelected() {
  int8_t tool;
  // rather return the swapped tool than the currently selected
  if (toolSelected >= 0 && toolSelected <= MAX_TOOLS)
    tool = swapTools[toolSelected];
  else
    tool = toolSelected;
  return tool;
}

void printDriverMode(int8_t serial) {
#ifdef HAS_TMC_SUPPORT
  char tmp[128];

  sprintf_P(tmp, P_TMC_StatusAll,
            drivers[SELECTOR] == nullptr ? P_Unknown : drivers[SELECTOR]->stealth() ? P_Stealth
                                                                                    : P_Spread,
            drivers[REVOLVER] == nullptr ? P_Unknown : drivers[REVOLVER]->stealth() ? P_Stealth
                                                                                    : P_Spread,
            drivers[FEEDER] == nullptr ? P_Unknown : drivers[FEEDER]->stealth() ? P_Stealth
                                                                                : P_Spread);
  printResponse(tmp, serial);
#endif
}

void printDriverRms(int8_t serial) {
#ifdef HAS_TMC_SUPPORT
  char tmp[128];

  sprintf_P(tmp, P_TMC_StatusAll,
            drivers[SELECTOR] == nullptr ? P_Unknown : (String(drivers[SELECTOR]->rms_current()) + String(P_MilliAmp)).c_str(),
            drivers[REVOLVER] == nullptr ? P_Unknown : (String(drivers[REVOLVER]->rms_current()) + String(P_MilliAmp)).c_str(),
            drivers[FEEDER] == nullptr ? P_Unknown : (String(drivers[FEEDER]->rms_current()) + String(P_MilliAmp)).c_str());
  printResponse(tmp, serial);
#endif
}

void printDriverMS(int8_t serial) {
#ifdef HAS_TMC_SUPPORT
  char tmp[128];

  sprintf_P(tmp, P_TMC_StatusAll,
            drivers[SELECTOR] == nullptr ? P_Unknown : String(drivers[SELECTOR]->microsteps()).c_str(),
            drivers[REVOLVER] == nullptr ? P_Unknown : String(drivers[REVOLVER]->microsteps()).c_str(),
            drivers[FEEDER] == nullptr ? P_Unknown : String(drivers[FEEDER]->microsteps()).c_str());
  printResponse(tmp, serial);
#endif
}

void printDriverStallThrs(int8_t serial) {
#ifdef HAS_TMC_SUPPORT
  char tmp[128];
  sprintf_P(tmp, P_TMC_StatusAll,
            drivers[SELECTOR] == nullptr ? P_Unknown : String(drivers[SELECTOR]->SGTHRS()).c_str(),
            drivers[REVOLVER] == nullptr ? P_Unknown : String(drivers[REVOLVER]->SGTHRS()).c_str(),
            drivers[FEEDER] == nullptr ? P_Unknown : String(drivers[FEEDER]->SGTHRS()).c_str());
  printResponse(tmp, serial);
#endif
}

void printEndstopState(int8_t serial) {
  char tmp[128];
  const char *_triggered = "triggered";
  const char *_open = "open";
  sprintf_P(tmp, P_TMC_StatusAll,
            selectorEndstop() ? _triggered : _open,
            revolverEndstop() ? _triggered : _open,
            feederEndstop() ? _triggered : _open);
  printResponse(tmp, serial);
  if (Z_END2_PIN > 0) {
    sprintf_P(tmp, PSTR("Feeder2 (Z2): %s\n"), feederEndstop(2) ? _triggered : _open);
    printResponse(tmp, serial);
    #if defined(USE_DDE)
      sprintf_P(tmp, PSTR("DDE (Z2): %s\n"), ddeEndstop() ? _triggered : _open);
      printResponse(tmp, serial);
    #endif
  }
}

void printDuetSignalStates(int8_t serial) {
  char tmp[80];
  const char *_high = "HIGH";
  const char *_low = "LOW";
  #if defined(DUET_SIG_SEL_PIN) && DUET_SIG_SEL_PIN > 0 && defined(DUET_SIG_FED_PIN) && DUET_SIG_FED_PIN > 0
  uint32_t selector = digitalRead(DUET_SIG_SEL_PIN);
  uint32_t feeder = digitalRead(DUET_SIG_FED_PIN);
  sprintf_P(tmp, P_Duet_StatusAll,
            selector ? _high : _low,
            feeder ? _high : _low);
  printResponse(tmp, serial);
  #else
  printResponseP(PSTR("Signal pins not defined!"), serial);
  #endif
}

void printSpeeds(int8_t serial) {
  char tmp[150];

  sprintf_P(tmp, !smuffConfig.speedsInMMS ? P_AccelSpeedTicks : P_AccelSpeedMms,
            smuffConfig.maxSpeed[SELECTOR],
            smuffConfig.stepDelay[SELECTOR],
            smuffConfig.maxSpeed[REVOLVER],
            smuffConfig.stepDelay[REVOLVER],
            smuffConfig.maxSpeed[FEEDER],
            smuffConfig.extControlFeeder ? " (Ext.)" : " (Int.)",
            smuffConfig.stepDelay[FEEDER],
            smuffConfig.insertSpeed);
  printResponse(tmp, serial);
}

void printAcceleration(int8_t serial) {
  char tmp[150];

  sprintf_P(tmp, !smuffConfig.speedsInMMS ? P_AccelSpeedTicks : P_AccelSpeedMms,
            smuffConfig.accelSpeed[SELECTOR],
            smuffConfig.stepDelay[SELECTOR],
            smuffConfig.accelSpeed[REVOLVER],
            smuffConfig.stepDelay[REVOLVER],
            smuffConfig.accelSpeed[FEEDER],
            smuffConfig.extControlFeeder ? " (Ext.)" : " (Int.)",
            smuffConfig.stepDelay[FEEDER],
            smuffConfig.insertSpeed);
  printResponse(tmp, serial);
}

void printSpeedAdjust(int8_t serial) {
  char tmp[150];

  sprintf_P(tmp, P_SpeedAdjust,
            String(smuffConfig.speedAdjust[SELECTOR]).c_str(),
            String(smuffConfig.speedAdjust[SELECTOR]).c_str(),
            String(smuffConfig.speedAdjust[REVOLVER]).c_str());
  printResponse(tmp, serial);
}

void printOffsets(int8_t serial) {
  char tmp[128];
  sprintf_P(tmp, P_TMC_StatusAll,
            String(smuffConfig.firstToolOffset).c_str(),
            String(smuffConfig.firstRevolverOffset).c_str(),
            "--");
  printResponse(tmp, serial);
}

void sendTMCStatus(uint8_t axis, int8_t port) {

#ifdef HAS_TMC_SUPPORT
  showDriver = drivers[axis];
#else
#define showDriver nullptr
#endif

  char tmp[80];
  if(showDriver == nullptr) {
    sprintf_P(tmp, P_TMCStatusNotUsed, P_TMCStatus, P_TMCKeyAxis, axis, P_TMCKeyInUse);
    printResponse(tmp, port);
    return;
  }

#ifdef HAS_TMC_SUPPORT
  steppers[axis].setEnabled(true);
  const char *ot_stat = P_No;

  if (showDriver->ot())  {
    if (showDriver->t157())
      ot_stat = P_OT_157;
    else if (showDriver->t150())
      ot_stat = P_OT_150;
    else if (showDriver->t143())
      ot_stat = P_OT_143;
    else if (showDriver->t120())
      ot_stat = P_OT_120;
  }
  Print* out = getSerialInstance(port);

  serializeTMCStats(out,
      axis,
      showDriver->version()- 0x20,
      showDriver->stealth(),
      smuffConfig.stepperPower[axis],
      showDriver->rms_current(),
      showDriver->microsteps(),
      showDriver->ms2(),
      showDriver->ms1(),
      showDriver->pdn_uart() ? P_Yes : P_No,
      showDriver->diag() ? P_Low : P_High,
      showDriver->ola() ? P_Yes : P_No,
      showDriver->olb() ? P_Yes : P_No,
      showDriver->s2ga() ? P_Yes : P_No,
      showDriver->s2gb() ? P_Yes : P_No,
      ot_stat);
  printResponse("\n", port);
  if(showDriver->drv_err())
    tmcWarning = false;
  #if defined(SMUFF_V6S)
  if(axis == REVOLVER)
    steppers[axis].setEnabled(false);
  #endif
#endif
}

void getStoredData() {
  recoverStore();
  steppers[SELECTOR].setStepPosition(dataStore.stepperPos[SELECTOR]);
  steppers[REVOLVER].setStepPosition(dataStore.stepperPos[REVOLVER]);
  steppers[FEEDER].setStepPosition(dataStore.stepperPos[FEEDER]);
  toolSelected = dataStore.tool;
  setFastLEDToolIndex(toolSelected, smuffConfig.toolColor, true);
  //__debugS(D, PSTR("Recovered tool: %d"), toolSelected);
}

void setSignalPort(uint8_t port, bool state) {

  if(!smuffConfig.useDuet) {
    __debugS(I, PSTR("UseDuet flag not set, skipping."));
  }
  else {
    // used for Duet3D/RRF Controller boards to signal progress of loading / unloading
    const char *_sig        = "DUET_SIG_%s_PIN %s";
    const char *_high       = "HIGH";
    const char *_low        = "LOW";
    const char *_pin        = (port == FEEDER_SIGNAL) ? "FED" : "SEL";
    const char *_notset     = "not set!";
    const char *_undefined  = "undefined!";

    if(port == FEEDER_SIGNAL) {
      #if defined(DUET_SIG_FED_PIN)
        if(DUET_SIG_FED_PIN > 0) {
          digitalWrite(DUET_SIG_FED_PIN, (smuffConfig.invertDuet ? !state : state));
          uint8_t pinState = digitalRead(DUET_SIG_FED_PIN);
          __debugS(D, PSTR(_sig), _pin, pinState ? _high : _low);
        }
        else
          __debugS(D, PSTR(_sig), _pin, _notset);
      #else
        __debugS(D, PSTR(_sig), _pin, _undefined);
      #endif
    }
    /*
      SELECTOR_SIGNAL is not being used anymore but it's in here because
      of backwards compatibility to older versions (i.e. SMUFF-Ifc).
    */
    if(port == SELECTOR_SIGNAL) {
      #if defined(DUET_SIG_SEL_PIN)
        if(DUET_SIG_SEL_PIN > 0) {
          digitalWrite(DUET_SIG_SEL_PIN, (smuffConfig.invertDuet ? !state : state));
          uint8_t pinState = digitalRead(DUET_SIG_SEL_PIN);
          __debugS(D, PSTR(_sig), _pin, pinState ? _high : _low);
        }
        else
          __debugS(D, PSTR(_sig), _pin, _notset);
      #else
        __debugS(D, PSTR(_sig), _pin, _undefined);
      #endif
    }
  }
}

void signalDuetBusy() {
  setSignalPort(FEEDER_SIGNAL, true);
}

void signalDuetReady() {
  setSignalPort(FEEDER_SIGNAL, false);
}

bool getFiles(const char *rootFolder PROGMEM, const char *pattern PROGMEM, uint8_t maxFiles, bool cutExtension, char *files) {
  char fname[40];
  char tmp[25];
  uint8_t cnt = 0;
  _File file;
  _File root;

  if (initSD(false)) {
    #if defined(USE_SDFAT)
      root.open(rootFolder, O_READ);
      while (file.openNext(&root, O_READ)) {
        if (!file.isHidden()) {
          file.getName(fname, ArraySize(fname));
          //__debugS(D, PSTR("File: %s"), fname);
          String lfn = String(fname);
          if (pattern != nullptr && !lfn.endsWith(pattern)) {
            file.close();
            continue;
          }
          if (pattern != nullptr && cutExtension)
            lfn.replace(pattern, "");
          sprintf_P(tmp, PSTR("%-20s\n"), lfn.c_str());
          strcat(files, tmp);
      }
    #else
      root = SD.open(rootFolder, FA_READ);
      while (file = root.openNextFile(FA_READ)) {
          //__debugS(D, PSTR("File: %s"), fname);
          String lfn = String(file.name());
          if (pattern != nullptr && !lfn.endsWith(pattern)) {
            file.close();
            continue;
          }
          if (pattern != nullptr && cutExtension)
            lfn.replace(pattern, "");
          sprintf_P(tmp, PSTR("%-20s\n"), lfn.c_str());
          strcat(files, tmp);
      }
    #endif
      file.close();
      if (++cnt >= maxFiles)
        break;
    }
    root.close();
    files[strlen(files) - 1] = '\0';
    return true;
  }
  return false;
}

/*
  Removes file firmware.bin from root directory in order
  to prevent re-flashing on each reset!
*/
void removeFirmwareBin() {
  if (initSD(false))
    SD.remove("firmware.bin");
}

/*
  Reads the next line from the test script and filters unwanted
  characters.
*/
uint8_t getTestLine(_File *file, char *line, int maxLen)
{
  uint8_t n = 0;
  bool isQuote = false;
  while (1)
  {
    int c = file->read();
    if (c == -1)
      break;
    if (c == ' ' || c == '\r')
      continue;
    if (c == '\n')
    {
      *(line + n) = 0;
      break;
    }
    if (c == '"')
    {
      isQuote = !isQuote;
    }
    *(line + n) = isQuote ? c : toupper(c);
    n++;
    if (n >= maxLen)
      break;
  }
  return n;
}

void printReport(const char* line, unsigned long loopCnt, unsigned long cmdCnt, long toolChanges, unsigned long secs, unsigned long endstop2Hit[], unsigned long endstop2Miss[])
{
  char report[700];
  char runtm[20], gco[40], err[10], lop[10], cmdc[10], tc[20], stallS[10], stallF[10];
  char etmp[15], ehit[smuffConfig.toolCount * 10], emiss[smuffConfig.toolCount * 10];

  if(smuffConfig.webInterface) {
    // load JSON report when in WebInterface mode
    if(!loadReport(PSTR("report"), report, "json", ArraySize(report))) {
      __debugS(W, PSTR("Failed to load report.json"));
      return;
    }
  }
  else {
    // load report from SD-Card; Can contain 10 lines at max.
    if(!loadReport(PSTR("report"), report, nullptr, ArraySize(report))) {
      __debugS(W, PSTR("Failed to load report.txt"));
      return;
    }
  }

  if(!smuffConfig.webInterface) {
    sprintf_P(gco, PSTR("%-25s"), line);
    sprintf_P(err, PSTR("%4lu"), feederErrors);
    sprintf_P(lop, PSTR("%4lu"), loopCnt);
    sprintf_P(cmdc, PSTR("%5lu"), cmdCnt);
    sprintf_P(tc, PSTR("%5ld"), toolChanges);
    sprintf_P(stallS, PSTR("%4lu"), stallDetectedCountSelector);
    sprintf_P(stallF, PSTR("%4lu"), stallDetectedCountFeeder);
    sprintf_P(runtm, PSTR("%4d:%02d:%02d"), (int)(secs / 3600), (int)(secs / 60) % 60, (int)(secs % 60));
    memset(ehit, 0, ArraySize(ehit));
    memset(emiss, 0, ArraySize(emiss));
    for (uint8_t tcnt = 0; tcnt < smuffConfig.toolCount; tcnt++){
      sprintf_P(etmp, PSTR("%5lu | "), endstop2Hit[tcnt]);
      strcat(ehit, etmp);
      sprintf_P(etmp, PSTR("%5lu | "), endstop2Miss[tcnt]);
      strcat(emiss, etmp);
    }
  }
  else {
    sprintf_P(gco, PSTR("%s"), line);
    sprintf_P(err, PSTR("%lu"), feederErrors);
    sprintf_P(lop, PSTR("%lu"), loopCnt);
    sprintf_P(cmdc, PSTR("%lu"), cmdCnt);
    sprintf_P(tc, PSTR("%ld"), toolChanges);
    sprintf_P(stallS, PSTR("%lu"), stallDetectedCountSelector);
    sprintf_P(stallF, PSTR("%lu"), stallDetectedCountFeeder);
    sprintf_P(runtm, PSTR("%4d:%02d:%02d"), (int)(secs / 3600), (int)(secs / 60) % 60, (int)(secs % 60));
    memset(ehit, 0, ArraySize(ehit));
    memset(emiss, 0, ArraySize(emiss));
    for (uint8_t tcnt = 0; tcnt < smuffConfig.toolCount; tcnt++) {
      sprintf_P(etmp, PSTR("%lu|"), endstop2Hit[tcnt]);
      strcat(ehit, etmp);
      sprintf_P(etmp, PSTR("%lu|"), endstop2Miss[tcnt]);
      strcat(emiss, etmp);
    }

  }
  char *lines[10];
  // format report to be sent to terminal
  uint8_t cnt = splitStringLines(lines, ArraySize(lines), report);
  String s;
  for (uint8_t n = 0; n < cnt; n++)
  {
    s = String(lines[n]);
    s.replace("{TIME}", runtm);
    s.replace("{GCO}", gco);
    s.replace("{ERR}", err);
    s.replace("{LOOP}", lop);
    s.replace("{CMDS}", cmdc);
    s.replace("{TC}", tc);
    s.replace("{STALL}", stallS);
    s.replace("{STALLF}", stallF);
    s.replace("{HIT}", ehit);
    s.replace("{MISS}", emiss);
    if(!smuffConfig.webInterface) {
      s.replace("{ESC:", "\033[");
      s.replace("f}", "f");
      s.replace("m}", "m");
      s.replace("J}", "J");
      s.replace("H}", "H");
      __log(s.c_str());
    }
    else {
      debugSerial->print(PSTR("echo: testrun: "));
      debugSerial->println(s.c_str());
    }
  }
}

bool isTestrun = false;
bool isTestPending = false;
char testToRun[60];

void setTestRunPending(const char* testfile) {
  strncpy(testToRun, testfile, ArraySize(testToRun));
  isTestPending = true;
}

void testRun(const char *fname)
{
  char line[50];
  char msg[80];
  char filename[50];
  _File file;
  String gCode;
  unsigned long loopCnt = 1L, cmdCnt = 1L;
  int8_t tool = 0, lastTool = 0;
  uint8_t mode = 1;
  long toolChanges = 0;
  unsigned long startTime = millis();
  unsigned long endstop2Miss[smuffConfig.toolCount], endstop2Hit[smuffConfig.toolCount];
  int16_t turn;
  uint8_t btn;
  bool isHeld, isClicked;
  bool showReport;

  debounceButton();

  if (initSD(false))
  {
    steppers[REVOLVER].setEnabled(true);
    steppers[SELECTOR].setEnabled(true);
    steppers[FEEDER].setEnabled(true);
    sprintf_P(msg, P_RunningTest, fname);
    drawUserMessage(msg);
    delay(1750);
    randomSeed(millis());
    sprintf_P(filename, PSTR("/test/%s.gcode"), fname);
    feederErrors = 0;

    for (uint8_t i = 0; i < smuffConfig.toolCount; i++)
    {
      endstop2Hit[i] = 0L;
      endstop2Miss[i] = 0L;
    }

    if(__fopen(file, filename, FILE_READ)) {
      setFastLEDStatus(FASTLED_STAT_NONE);

      stallDetectedCountFeeder = stallDetectedCountSelector = 0;

#if defined(USE_LEONERD_DISPLAY)
      encoder.setLED(LN_LED_GREEN, true);
#endif
      isTestrun = true;
      isTestPending = false;
      while (1)
      {
        setPwrSave(0);
        showReport = false;
        checkSerialPending();
        #if !defined(USE_SERIAL_DISPLAY)
        getInput(&turn, &btn, &isHeld, &isClicked, false);
        if (isHeld || isClicked || !isTestrun)
        {
          break;
        }
        if (turn < 0)
        {
          if (--mode < 0)
            mode = 3;
        }
        else if (turn > 0)
        {
          if (++mode > 3)
            mode = 0;
        }
        #endif
        unsigned long secs = (millis() - startTime) / 1000;
        uint8_t n = getTestLine(&file, line, ArraySize(line));
        if (n > 0)
        {
          if (*line == ';')
            continue;
          gCode = String(line);
          if(gCode.indexOf("{PRPT}") > -1) {
            printReport(line, loopCnt, cmdCnt, toolChanges, secs, endstop2Hit, endstop2Miss);
            continue;
          }
          if (gCode.indexOf("{RNDT}") > -1) {
            // randomize the next tool number in a loop to avoid
            // selecting the same tool that's currently set
            int8_t retry = smuffConfig.toolCount;
            do
            {
              tool = (int8_t)random(0, smuffConfig.toolCount);
              //__debugS(D, PSTR("retry: %d  tool: %d  last: %d"), retry, tool, lastTool);
              if (--retry < 0)
                break;
            } while(tool == lastTool);
            gCode.replace("{RNDT}", String(tool));
          }
          if (gCode.indexOf("{RNDTL}") > -1) {
            gCode.replace("{RNDTL}", String(tool));
          }
          parseGcode(gCode, -1);
          //__debugS(D, PSTR("GCode: %s"), gCode.c_str());
          if (*line == 'T')
          {
            tool = (int8_t)strtol(line + 1, nullptr, 10);
            lastTool = toolSelected;
            toolChanges++;
            showReport = true;
          }
          if (*line == 'C')
          {
            if (!feederEndstop(2))
              endstop2Hit[tool]++;
            else
              endstop2Miss[tool]++;
            showReport = true;
          }
          cmdCnt++;
          if (cmdCnt % 10 == 0)
          {
            mode++;
            if (mode > 3)
              mode = 0;
          }
          if (showReport)
            printReport(line, loopCnt, cmdCnt, toolChanges, secs, endstop2Hit, endstop2Miss);
        }
        else
        {
          randomSeed(analogRead(0));
          // restart from beginning and increment loop count
          file.seek(0);
          //__debugS(D, PSTR("Rewinding"));
          loopCnt++;
        }
        switch (mode)
        {
          case 3:
            sprintf_P(msg, P_FeederErrors, feederErrors);
            break;
          case 2:
            sprintf_P(msg, P_ToolChanges, toolChanges);
            break;
          case 1:
            sprintf_P(msg, P_CmdLoop, cmdCnt, tool);
            break;
          case 0:
            sprintf_P(msg, P_TestTime, (int)(secs / 3600), (int)(secs / 60) % 60, (int)(secs % 60));
            break;
        }
        drawTestrunMessage(loopCnt, msg);
        delay(150);
        if (!showReport)
            printReport(line, loopCnt, cmdCnt, toolChanges, secs, endstop2Hit, endstop2Miss);
      }
      file.close();
#if defined(USE_LEONERD_DISPLAY)
      encoder.setLED(LN_LED_GREEN, false);
#endif
    }
    else
    {
      sprintf_P(msg, P_TestFailed, fname);
      drawUserMessage(msg);
      delay(3000);
    }
  }
  #if defined(SMUFF_V6S)
    steppers[REVOLVER].setEnabled(false);
  #endif
  isTestrun = false;
}

volatile uint32_t lastYield = 0;

void yield() {
}

void listHelpFile(const char *filename PROGMEM, const char* filter, int8_t serial)
{
  char fname[80];

  sprintf_P(fname, PSTR("help/%s.txt"), filename);
  printResponseP(P_Usage, serial);
  listTextFile(fname, filter, serial);
}

#if !defined(USE_SDFAT)
size_t fgets(uint8_t* buffer, size_t maxLen, char delimiter, _File* file) {
  return ((Stream*)file)->readBytesUntil(delimiter, buffer, maxLen);
}
#endif

bool isListingFile = false;

void listTextFile(const char *filename PROGMEM, const char* filter, int8_t serial)
{
  _File file;
  uint8_t line[255];
  bool headerEnd = false;

  if(__fopen(file, filename, FILE_READ)) {
    isListingFile = true;
    #if defined(USE_SDFAT)
      while(file.fgets((char*)line, ArraySize(line) - 1, (char*)"\n") > 0) {
    #else
      while(fgets(line, ArraySize(line) - 1, '\n', &file) > 0) {
    #endif
        if(filter != nullptr) {
          // filter is set; check if the current line matches the filter
          String ln = String((const char*)line);
          ln.toUpperCase();
          if(!headerEnd) {
            // print line unless the header end has been found
            printResponse((const char*)line, serial);
            if(ln.startsWith("---"))
              headerEnd = true;
            continue;
          }
          else if (ln.indexOf(filter) > -1)
            // print line matching the filter
            printResponse((const char*)line, serial);
        }
        else
          // print line unfiltered
          printResponse((const char*)line, serial);
    }
    file.close();
    isListingFile = false;
  }
  else
  {
    sprintf_P((char*)line, P_FileNotFound, filename);
    printResponse((const char*)line, serial);
  }
  printResponse("\n", serial);
}

static char _loadmenu[1000];

/**
 * Loads a menu from SD-Card.
 *
 * @param filename  the name of the menu file to load
 * @param ordinals  the menu entry ordinals
 * @returns the contents of that file
 */
const char *loadMenu(const char *filename PROGMEM, int ordinals[], size_t maxLen)
{
  _File file;
  char fname[80];
  char ordinal[10];

  memset(_loadmenu, 0, ArraySize(_loadmenu)-1);
  memset(ordinals, -1, MAX_MENU_ORDINALS * sizeof(int));
  sprintf_P(fname, PSTR("menus/%s.mnu"), filename);
  if(__fopen(file, fname, FILE_READ)) {
    uint16_t n = 0;
    uint8_t line = 1;
    int c;
    while ((c = file.read()) != -1)
    {
      if (c == '\r')
        continue;
      // check for an separator indicating an ordinal number is following
      if (c == '|')
      {
        uint8_t on = 0;
        memset(ordinal, 0, ArraySize(ordinal));
        do {
          c = file.read();
          if(c == -1)
            break;
          if (isdigit(c)) {
            ordinal[on++] = c;
          }
          if (on >= (int)ArraySize(ordinal))
            break;
        } while (isdigit(c));
        ordinals[line] = atoi(ordinal);
        //__debugS(D, PSTR("Ordinal found: %s = %d"), ordinal, ordinals[line]);
        continue;
      }
      if(n >= maxLen) {
        __debugS(I, PSTR("Warning overflow in source array size in Menu '%s'"), filename);
        break;
      }
      _loadmenu[n] = (char)c;
      if (c == '\n')
        line++;
      // convert \n-\n to separator char (GS = Group Separator = \035)
      if (n > 2 && (_loadmenu[n] == '\n' && _loadmenu[n - 1] == '-' && _loadmenu[n - 2] == '\n')) {
        _loadmenu[n - 1] = '\035';
      }
      n++;
      if(n > ArraySize(_loadmenu)) {
        break;
        __debugS(I, PSTR("Possible overflow in Menu '%s': %d bytes read, array size %d"), filename, n, ArraySize(_loadmenu));
      }
    }
    file.close();

    if (n == 0) {
      __debugS(W, PSTR("Failed to load menu '%s'"), filename);
    }
    return _loadmenu;
  }
  else
  {
    __debugS(I, P_FileNotFound, filename);
  }
  return nullptr;
}

const char *loadOptions(const char *filename PROGMEM, size_t maxLen)
{
  _File file;
  static char opts[300];
  char fname[80];

  memset(opts, 0, ArraySize(opts));
  sprintf_P(fname, PSTR("options/%s.opt"), filename);
  if(__fopen(file, fname, FILE_READ)) {
    size_t n = 0;
    int c;
    while ((c = file.read()) != -1)
    {
      if (c == '\r')
        continue;
      opts[n] = c;
      n++;
      if(n >= maxLen) {
        __debugS(I, PSTR("Warning overflow in source array size in Option '%s'"), filename);
        break;
      }
    }
    file.close();
    if (n == 0)
    {
      __debugS(W, PSTR("Failed to load options '%s'"), filename);
    }
    //__debugS(D, PSTR("Opts: '%s' %d bytes"), filename, n);
    return opts;
  }
  else
  {
    __debugS(I, P_FileNotFound, filename);
  }
  return nullptr;
}

bool loadReport(const char *filename PROGMEM, char *buffer, const char* ext, uint16_t maxLen)
{
  _File file;
  char fname[40];

  memset(buffer, 0, maxLen);
  if(ext == nullptr)
    sprintf_P(fname, PSTR("/test/%s.txt"), filename);
  else
    sprintf_P(fname, PSTR("/test/%s.%s"), filename, ext);
  if(__fopen(file, fname, FILE_READ)) {
    int n = file.read(buffer, maxLen - 1);
    file.close();
    if (n == 0)
    {
      __debugS(W, PSTR("Failed to load report '%s'"), filename);
    }
    return true;
  }
  else
  {
    __debugS(I, P_FileNotFound, filename);
  }
  return false;
}

bool maintainingMode = false;
int8_t maintainingTool = -1;

bool maintainTool(char* errmsg)
{
  int8_t newTool;
  bool stat = true;

  while (feederEndstop()) {
    if (!showFeederBlockedMessage()) {
      snprintf_P(errmsg, MAX_ERR_MSG, P_FeederLoaded);
      return false;
    }
  }

  if (maintainingMode) {
    maintainingMode = false;
    if (maintainingTool != -1) {
      stat = selectTool(maintainingTool, errmsg, false);
    }
    maintainingTool = -1;
  }
  else {
    maintainingMode = true;
    maintainingTool = getToolSelected();

    if (toolSelected <= (smuffConfig.toolCount / 2)) {
      newTool = toolSelected + 2;
    }
    else {
      newTool = toolSelected - 2;
    }
    if (newTool >= 0 && newTool < smuffConfig.toolCount) {
      stat = selectTool(newTool, errmsg, false);
    }
  }
  return stat;
}

void blinkLED()
{
#if defined(LED_PIN)
  if (LED_PIN > 0)
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
#endif
}

/*
  Translates speeds from mm/s into MCU timer ticks.
  Please notice: Using speeds in mm/s will run the steppers far slower since it has to ensure that the
  distance per second will match.
*/
unsigned long translateSpeed(uint16_t speed, uint8_t axis, bool forceTranslation)
{
  return translateSpeed((double)speed, axis, forceTranslation);
}

unsigned long translateSpeed(double speed, uint8_t forAxis, bool forceTranslation)
{
  if(!forceTranslation)
    if (!smuffConfig.speedsInMMS)
      return speed;

  // Basic formula:   ticks = [stepper-clock] / ( [mm/s] * [steps/mm] )
  //                  mm/s = [stepper-clock] / [ticks] / [steps/mm]
  //
  //                  stepper-clock = [CPU frequency] / [Prescaler]

  // Notice: Not taking step-delay into account here! 
  unsigned long stepper_clock = F_CPU / STEPPER_PSC;
  uint16_t stepsPerMM = smuffConfig.stepsPerMM[forAxis];
  double minSpeed = 0.0, maxSpeed = 0.0;

  unsigned long ticks = (unsigned long)(stepper_clock / (double)(speed * stepsPerMM));
  //__debugS(DEV3, PSTR("Ticks before adjust: %ld (stepper clock: %ld, speed: %s, steps/mm: %d)"), ticks, stepper_clock, String(speed).c_str(), stepsPerMM);

  if(ticks > 65534) {
    ticks = 65534;  // lowest achievable speed in ticks, since that's the timer reload value which must not exceed 16 bit
    minSpeed = (double)(stepper_clock / ticks / stepsPerMM);
    if(speed < minSpeed) {
      __debugS(I, PSTR("Speed requested: %3d mm/s  - Min. Speed: %3.1f mm/s"), speed, minSpeed);
      speed = minSpeed;
    }
  }
  if(ticks < 2) {
    ticks = 2;    // highest achievable speed in ticks
    maxSpeed = (double)(stepper_clock / ticks / stepsPerMM);
    if(speed > maxSpeed) {
      __debugS(I, PSTR("Speed requested: %3d mm/s  - Max. Speed: %3.1f mm/s"), speed, maxSpeed);
      speed = maxSpeed;
    }
  }
  return ticks;
}

uint8_t scanI2CLoop(TwoWire* bus, uint8_t *devices, uint8_t maxDevices, uint8_t index) {
  uint8_t cnt = 0;
  for (uint8_t address = 1; address < 127 && cnt <= maxDevices; address++) {
    bus->beginTransmission(address);
    uint8_t stat = bus->endTransmission();
    if (stat == I2C_OK)
    {
      *(devices + cnt) = address;
      cnt++;
      // __debugS(DEV3, PSTR("\t\tDevice found on HW I2C (bus %d) at address 0x%02]"), index, address);
    }
    delay(3);
  }
  return cnt;
}

#if defined(USE_MULTISERVO) || defined(USE_SW_TWI)
uint8_t scanI2CLoop(SoftWire* bus, uint8_t *devices, uint8_t maxDevices, uint8_t index) {
  uint8_t cnt = 0;
  for (uint8_t address = 1; address < 127 && cnt <= maxDevices; address++) {
    bus->beginTransmission(address);
    uint8_t stat = bus->endTransmission();
    if (stat == I2C_OK) {
      *(devices + cnt) = address;
      cnt++;
      // __debugS(DEV3, PSTR("\t\tDevice found on SW I2C (bus %d) at address 0x%02]"), index, address);
    }
    delay(3);
  }
  return cnt;
}
#endif

void enumI2cDevices(uint8_t bus) {
  uint8_t devs[40]; // unlikey that there are more devices than that on the bus
  const char *name;
  bool encoder = false, multiservo = false, estopmux = false, i2cdisplay = false;
  uint8_t deviceCnt = 0;
  memset(devs, 0, ArraySize(devs));
  switch(bus) {
    case 1:
      I2CBus.begin();
      deviceCnt = scanI2CLoop(&I2CBus, devs, ArraySize(devs), 1);
      break;

    case 2:
      #if defined(USE_SPLITTER_ENDSTOPS)
        I2CBus2.begin();
        deviceCnt = scanI2CLoop(&I2CBus2, devs, ArraySize(devs), 2);
      #endif
      break;

    case 3:
      #if defined(USE_MULTISERVO)
        I2CBusMS.begin();
        deviceCnt = scanI2CLoop(&I2CBusMS, devs, ArraySize(devs), 3);
      #endif

      break;
    default:
      __debugS(D, PSTR("Unknown I2C bus %d. Scan rejected."), bus);
      return;
  }
  if (deviceCnt > 0) {
    for (uint8_t i = 0; i < ArraySize(devs); i++) {
      if (devs[i] == 0)
        break;
      switch (devs[i]) {
        case I2C_ENCODER_ADDRESS:
          name = PSTR("Encoder");
          encoder = true;
          break;
        case I2C_DISPLAY_ADDRESS:
          name = PSTR("Display");
          i2cdisplay = true;
          break;
        case I2C_SERVOCTL_ADDRESS:
        case I2C_SERVOBCAST_ADDRESS:
          name = PSTR("MultiServo");
          multiservo = true;
          break;
        case I2C_EEPROM_ADDRESS:
          name = PSTR("EEPROM");
          break;
        case I2C_SPL_MUX_ADDRESS:
          name = PSTR("EStop MUX Splitter");
          estopmux = true;
          break;
        default:
          name = PSTR("n.a.");
          break;
      }
      __debugS(DEV, PSTR("\tI2C device found on bus %d at address 0x%02x (%s)"), bus, devs[i], name);
    }
  }
  else {
    __debugS(I, PSTR("I2C Scan has found no devices!"));
  }
  const char noDevFound[] = { "is configured but the according device was not found!" };
  #if defined(USE_TWI_DISPLAY)
    if (!i2cdisplay) {
        __debugS(I, PSTR("I2C display %s"), noDevFound);
    }
  #endif
  #if defined(USE_SPLITTER_ENDSTOPS)
    if (!estopmux) {
        __debugS(I, PSTR("Splitter using endstops %s"), noDevFound);
    }
  #endif
  #if defined(USE_LEONERD_DISPLAY)
    if (!encoder) {
        __debugS(I, PSTR("LeoNerd's OLED %s"), noDevFound);
    }
  #endif
  #if defined(USE_MULTISERVO)
    if (!multiservo) {
      __debugS(I, PSTR("Adafruit Multiservo %s"), noDevFound);
    }
  #endif
}

static char _dbg[1024];
static char _dbgInt[1024];
static char _dbgPfx1[20];

void __initDebug__() {
  memset(_dbg, 0, ArraySize(_dbg));
  memset(_dbgInt, 0, ArraySize(_dbgInt));
  snprintf_P(_dbgPfx1, ArraySize(_dbgPfx1), PSTR("echo: dbg: "));
}

void __flushDebug__() {
  if(*_dbgInt != 0) {
    // if messages generated in interrupt handlers are pending, print them out
    debugSerial->printf(PSTR("%s\n"), _dbgInt);
    *_dbgInt = 0;
  }
}

void __debugS__(uint8_t level, bool isInt, const char *fmt, ...) {
  if(debugSerial == nullptr || (smuffConfig.dbgLevel & level) == 0 || isListingFile)
    return;

  bool useColoring = smuffConfig.useDebugColoring;
  char pfx2[40];
  memset(_dbg, 0, ArraySize(_dbg) - 1);
  va_list arguments;
  va_start(arguments, fmt);
  vsnprintf_P(_dbg, ArraySize(_dbg) - 1, fmt, arguments);
  va_end(arguments);

  size_t maxlen = ArraySize(pfx2);
  switch(level) {
    case D:     snprintf_P(pfx2, maxlen, PSTR("(D)    ")); break;
    case I:     snprintf_P(pfx2, maxlen, PSTR("(I)    %s"), !useColoring ? "" : "\033[32m"); break;
    case W:     snprintf_P(pfx2, maxlen, PSTR("(W)    %s"), !useColoring ? "" : "\033[31m"); break;
    case SP:    snprintf_P(pfx2, maxlen, PSTR("(SP)   %s"), !useColoring ? "" : "\033[36m"); break;
    case DEV:   snprintf_P(pfx2, maxlen, PSTR("(DEV)  %s"), !useColoring ? "" : "\033[33m"); break;
    case DEV2:  snprintf_P(pfx2, maxlen, PSTR("(DEV2) %s"), !useColoring ? "" : "\033[36m"); break;
    case DEV3:  snprintf_P(pfx2, maxlen, PSTR("(DEV3) %s"), !useColoring ? "" : "\033[35m"); break;
    case DEV4:  snprintf_P(pfx2, maxlen, PSTR("(DEV4) %s"), !useColoring ? "" : "\033[34m"); break;
  }
  if(!isInt) {
    __flushDebug__();
    debugSerial->printf(PSTR("%s%s%s%s\n"), _dbgPfx1, pfx2, _dbg, (!useColoring || level == D) ? "" : "\033[0m");
  }
  else {
    // messages generated in interrupt handlers will get buffered instead of directly printed out
    char tmp[256];
    snprintf(tmp, ArraySize(tmp), PSTR("%s%s%s%s%s\n"), _dbgPfx1, pfx2, (!useColoring) ? "<INT> " : "\033[37m", _dbg, (!useColoring || level == D) ? "" : "\033[0m");
    strncat(_dbgInt, tmp, ArraySize(_dbgInt)-strlen(_dbgInt));
  }
}

/* Function obsolete since V3.16
void __terminal(const char *fmt, ...)
{
  if (terminalSerial == nullptr)
    return;
  char _term[1024];
  va_list arguments;
  va_start(arguments, fmt);
  vsnprintf_P(_term, ArraySize(_term) - 1, fmt, arguments);
  va_end(arguments);
  terminalSerial->print(_term);
}
*/

void __log(const char *fmt, ...)
{
  if (logSerial == nullptr)
    return;
  char _log[1024];
  va_list arguments;
  va_start(arguments, fmt);
  vsnprintf_P(_log, ArraySize(_log) - 1, fmt, arguments);
  va_end(arguments);
  logSerial->print(_log);
}
