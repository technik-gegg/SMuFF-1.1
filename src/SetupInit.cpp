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
#include "SMuFF.h"


void initUSB() {
#if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)
  if (USB_CONNECT_PIN > 0) {
    pinMode(USB_CONNECT_PIN, OUTPUT);
    digitalWrite(USB_CONNECT_PIN, HIGH);  // USB clear connection
    delay(100);                           // give OS time to re-enumerate
    digitalWrite(USB_CONNECT_PIN, LOW);   // USB reestablish connection
  }
#endif
}

void readSequences() {
  char tuneData[500];
  size_t bufLen = ArraySize(tuneData);

  if(readTune(STARTUP_FILE, tuneData, bufLen))
    strncpy(tuneStartup, tuneData, MAX_TUNE1);

  if(readTune(USERBEEP_FILE, tuneData, bufLen))
    strncpy(tuneUser, tuneData, MAX_TUNE2);

  if(readTune(BEEP_FILE, tuneData, bufLen))
    strncpy(tuneBeep, tuneData, MAX_TUNE3);

  if(readTune(LONGBEEP_FILE, tuneData, bufLen))
    strncpy(tuneLongBeep, tuneData, MAX_TUNE3);

#if defined(USE_LEONERD_DISPLAY)
  if(readTune(ENCBEEPLEO_FILE, tuneData, bufLen))
    strncpy(tuneEncoder, tuneData, MAX_TUNE3);
#else
  if(readTune(ENCBEEP_FILE, tuneData, bufLen))
    strncpy(tuneEncoder, tuneData, MAX_TUNE3);
#endif
}

void initAdaNeoPx() {
#if defined(USES_ADAFRUIT_NPX)
  #if NEOPIXEL_PIN > 0 && defined(USE_FASTLED_BACKLIGHT)
    pinMode(NEOPIXEL_PIN, OUTPUT);
    cBackLight = new Adafruit_NeoPixel(NUM_LEDS, NEOPIXEL_PIN, COLOR_ORDER);
    cBackLight->setBrightness(BRIGHTNESS);
    __debugS(D, PSTR("[\tinitAdaNeoPx: Backlight initialized ]"));
  #else
    #if defined(NEOPIXEL_PIN)
      __debugS(D, PSTR("[\tinitAdaNeoPx: Backlight not enabled. Neopixel Pin: %d ]"), NEOPIXEL_PIN);
    #endif
  #endif
  #if NEOPIXEL_TOOL_PIN > 0 && defined(USE_FASTLED_TOOLS)
    pinMode(NEOPIXEL_TOOL_PIN, OUTPUT);
    cTools = new Adafruit_NeoPixel(smuffConfig.toolCount, NEOPIXEL_TOOL_PIN, COLOR_ORDER_TOOL);
    cTools->setBrightness(BRIGHTNESS_TOOL);
    __debugS(D, PSTR("[\tinitAdaNeoPx: Tools initialized ]"));
  #else
    #if defined(NEOPIXEL_TOOL_PIN)
      __debugS(D, PSTR("[\tinitAdaNeoPx: Neopixels for tools not enabled. Neopixel Pin: %d ]"), NEOPIXEL_TOOL_PIN);
    #endif
  #endif
#endif
}

/*
  Initialize FastLED library for NeoPixels.
  Primarily used for backlight on some displays (i.e. FYSETC 12864 Mini V2.1)
  but also for the tool status if wired and defined.
*/
void initFastLED() {
#if defined(USES_ADAFRUIT_NPX)
  initAdaNeoPx();
#else
  #if NEOPIXEL_PIN > 0 && defined(USE_FASTLED_BACKLIGHT)
    pinMode(NEOPIXEL_PIN, OUTPUT);
    cBackLight = &FastLED.addLeds<LED_TYPE, NEOPIXEL_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);
    __debugS(D, PSTR("[\tinitFastLED: Backlight initialized ]"));
  #else
    #if defined(NEOPIXEL_PIN)
    __debugS(D, PSTR("[\tinitFastLED: Neopixels for backlight not enabled. Neopixel Pin: %d ]"), NEOPIXEL_PIN);
    #endif
  #endif
  #if NEOPIXEL_TOOL_PIN > 0 && defined(USE_FASTLED_TOOLS)
    pinMode(NEOPIXEL_TOOL_PIN, OUTPUT);
    cTools = &FastLED.addLeds<LED_TYPE_TOOL, NEOPIXEL_TOOL_PIN, COLOR_ORDER_TOOL>(ledsTool, smuffConfig.toolCount).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS_TOOL);
    __debugS(D, PSTR("[\tinitFastLED: Tools initialized ]"));
  #else
    #if defined(NEOPIXEL_TOOL_PIN)
    __debugS(D, PSTR("[\tinitFastLED: Neopixels for tools not enabled. Neopixel Pin: %d ]"), NEOPIXEL_TOOL_PIN);
    #endif
  #endif
#endif
}

/*
  Initialize pin for hardware debugging.
  Primarily used to attach an oscilloscope.
*/
void initHwDebug() {
#if defined(__HW_DEBUG__) && defined(DEBUG_PIN) && DEBUG_PIN > 0
  pinMode(DEBUG_PIN, OUTPUT);
  digitalWrite(DEBUG_PIN, HIGH);
  calcHwDebugCounter();
  __debugS(D, PSTR("[\tinitHwDebug: Pin initialized, frequency is %dHz ]"), smuffConfig.dbgFreq);
#endif
}

void calcHwDebugCounter() {
  flipDbgCnt = (uint16_t)((float)(1/((float)GPTIMER_RESOLUTION/1000000L))/(smuffConfig.dbgFreq*2));
}

void setupDuetSignals() {
  #if defined(DUET_SIG_FED_PIN) && DUET_SIG_FED_PIN > 0
    pinMode(DUET_SIG_FED_PIN, OUTPUT);
    digitalWrite(DUET_SIG_FED_PIN, (smuffConfig.invertDuet ? HIGH : LOW));
    __debugS(D, PSTR("[\tsetupDuetSignals: Feeder pin initialized ]"));
  #endif

  #if defined(DUET_SIG_SEL_PIN) && DUET_SIG_SEL_PIN > 0
    pinMode(DUET_SIG_SEL_PIN, OUTPUT);
    digitalWrite(DUET_SIG_SEL_PIN, (smuffConfig.invertDuet ? HIGH : LOW));
    __debugS(D, PSTR("[\tsetupDuetSignals: Selector pin initialized ]"));
  #endif
}

bool needsReinit(uint8_t index){
  return smuffConfig.serialBaudrates[index] != 115200 && 
        (smuffConfig.serialBaudrates[index] >= 4800 && smuffConfig.serialBaudrates[index] <= 230400);
}

void reinitSerial(HardwareSerial* serial, uint8_t index) {
  serial->end();
  delay(150);
  serial->begin(smuffConfig.serialBaudrates[index]);
  delay(250);
}

void reinitSerial(USBSerial* serial, uint8_t index) {
  serial->end();
  delay(150);
  serial->begin(smuffConfig.serialBaudrates[index]);
  delay(250);
}

void setupSerial() {
  const char* initMsg = "[\tsetupSerial: SERIAL%d initialized with %ld baud ]";
  // special case:
  // if the baudrate is set to 0, the board is running out of memory
  if (smuffConfig.serialBaudrates[0] != 0)
  {
    if (smuffConfig.serialBaudrates[0] != 115200)
      reinitSerial(&Serial, 0);
    __debugS(D, PSTR(initMsg), 0, smuffConfig.serialBaudrates[0]);
  }
  else
  {
    __debugS(D, PSTR("[\tsetupSerial: Config error for serial\n--------------------\n"));
    writeMainConfig((Print *)debugSerial);
    __debugS(D, PSTR("\n-------------------- ]"));
    longBeep(3);
    showDialog(P_TitleConfigError, P_ConfigFail1, P_ConfigFail4, P_OkButtonOnly);
  }

  if (CAN_USE_SERIAL1) {
    if(needsReinit(1))
      reinitSerial(&Serial1, 1);
    __debugS(D, PSTR(initMsg), 1, smuffConfig.serialBaudrates[1]);
  }
  if (CAN_USE_SERIAL2) {
    if(needsReinit(2))
      reinitSerial(&Serial2, 2);
    __debugS(D, PSTR(initMsg), 2, smuffConfig.serialBaudrates[2]);
  }
  if (CAN_USE_SERIAL3) {
    if(needsReinit(3))
      reinitSerial(&Serial3, 3);
    __debugS(D, PSTR(initMsg), 3, smuffConfig.serialBaudrates[3]);
  }
}

void setupSwSerial0() {
#if defined(USE_SW_SERIAL0)
  swSerial0.begin(TMC_BAUDRATE);
  __debugS(D, PSTR("[\tsetupSwSerial0: Software SERIAL initialized with %ld baud ]"), TMC_BAUDRATE);
#endif
}

void setupRelay() {
  if (RELAY_PIN > 0) {
    pinMode(RELAY_PIN, OUTPUT);
    // if there's an external Feeder stepper defined (i.e. the 3D-Printer drives the Feeder),
    // switch on the external stepper by default. Otherwise, use the interal stepper.
    #if defined(USE_DDE)
    switchFeederStepper(EXTERNAL);
    #else
    if (smuffConfig.extControlFeeder)
      switchFeederStepper(EXTERNAL);
    else
      switchFeederStepper(INTERNAL);
    #endif
  }
  else {
      __debugS(D, PSTR("[\tsetupRelay: Relay pin undefined ]"));
  }
}

void setupServos() {

#if !defined(MULTISERVO)
  
  if (SERVO1_PIN > 0) {           // setup the Wiper servo
    setServoMaxCycles(SERVO_WIPER, smuffConfig.servoCycles1);
    setServoMinPwm(SERVO_WIPER, smuffConfig.servoMinPwm);
    setServoMaxPwm(SERVO_WIPER, smuffConfig.servoMaxPwm);
    setServoTickResolution(SERVO_WIPER);
    attachServo(SERVO_WIPER, SERVO1_PIN);
    uint8_t resetPos = 90, param;
    // try finding the default reset position of the wiper servo
    // from within the wipe sequence
    if ((param = getParam(String(smuffConfig.wipeSequence), (char *)"P")) != -1) {
      resetPos = (uint8_t)param;
    }
    setServoPos(SERVO_WIPER, resetPos);
    disableServo(SERVO_WIPER);
    __debugS(D, PSTR("[\tsetupServos: Wiper servo initialized ]"));
  }
  
  if (SERVO2_PIN > 0) {           // setup the Lid servo (replaces the Revolver stepper motor)
    setServoMaxCycles(SERVO_LID, smuffConfig.servoCycles1);
    setServoMinPwm(SERVO_LID, smuffConfig.servoMinPwm);
    setServoMaxPwm(SERVO_LID, smuffConfig.servoMaxPwm);
    setServoTickResolution(SERVO_LID);
    attachServo(SERVO_LID, SERVO2_PIN);
    setServoLid(SERVO_OPEN);
    __debugS(D, PSTR("[\tsetupServos: Lid servo initialized ]"));
  }
  
  if (SERVO3_PIN > 0) {           // setup the Filament-Cutter servo if defined
    setServoMaxCycles(SERVO_CUTTER, smuffConfig.servoCycles1);
    setServoMinPwm(SERVO_CUTTER, smuffConfig.servoMinPwm);
    setServoMaxPwm(SERVO_CUTTER, smuffConfig.servoMaxPwm);
    setServoTickResolution(SERVO_CUTTER);
    attachServo(SERVO_CUTTER, SERVO3_PIN);
    setServoPos(SERVO_CUTTER, smuffConfig.cutterOpen);
    delay(100);
    disableServo(SERVO_CUTTER);
    __debugS(D, PSTR("[\tsetupServos: Cutter servo initialized ]"));
  }
#else
  servoPwm.begin();
  servoPwm.setOscillatorFrequency(27000000);
  servoPwm.setPWMFreq(50);
  for (uint8_t i = 0; i < smuffConfig.toolCount; i++) {
    setServoPos(i + 10, servoPosClosed[i]);
    delay(400);
    setServoPos(i + 10, servoPosClosed[i] - SERVO_CLOSED_OFS);
  }
  __debugS(D, PSTR("[\tsetupServos: Multiservo initialized ]"));
#endif
}

void setupFan() {
  if (FAN_PIN > 0) {
    #if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)
    fan.attach(FAN_PIN, 0);
    fan.setTickRes(FAN_RESOLUTION);
    fan.setPulseWidthMax((uint16_t(((float)1/FAN_FREQUENCY)*1000000L)));
    fan.setBlipTimeout(FAN_BLIP_TIMEOUT);
    #else
    pinMode(FAN_PIN, OUTPUT);
    #endif
    if (smuffConfig.fanSpeed >= 0 && smuffConfig.fanSpeed <= 100)
    {
      #if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)
      fan.setFanSpeed(smuffConfig.fanSpeed);
      #else
      analogWrite(FAN_PIN, map(smuffConfig.fanSpeed, 0, 100, 0, 255));
      #endif
    }
  }
  //__debugS(D, PSTR("[ setupFan: DONE ]"));
}

void setupEStopMux() {
#if defined(USE_SPLITTER_ENDSTOPS)
  extern ZEStopMux splitterMux;
  #if defined(USE_I2C)
  splitterMux.begin(&I2CBus, I2C_SPL_MUX_ADDRESS);
  #else
  splitterMux.begin(&I2CBus2, I2C_SPL_MUX_ADDRESS);
  #endif
  for(uint8_t i=0; i < smuffConfig.toolCount; i++) {
    splitterMux.setTool(i);
    delay(50);
    if(Z_END_PIN > 0) {
      smuffConfig.feedLoadState[i] = ((int8_t)digitalRead(Z_END_PIN) == steppers[FEEDER].getEndstopState());
      __debugS(D, PSTR("[\tsetupEStopMux: T%d is %s ]"), i, smuffConfig.feedLoadState[i] == SPL_NOT_LOADED ? "open" : "triggered");
    }
  }
  splitterMux.setTool(toolSelected);
  __debugS(D, PSTR("[\tsetupEStopMux: DONE ]"));
#endif
}

void setupI2C() {
  // nothing to do here but maybe in the future...
}

void setupBacklight() {
#if defined(USE_RGB_BACKLIGHT) || defined(USE_FASTLED_BACKLIGHT)
  setBacklightIndex(smuffConfig.backlightColor); // turn on the LCD backlight according to the configured setting
#endif
}

void setupEncoder() {
#if !defined(USE_SERIAL_DISPLAY)
  #if defined(USE_LEONERD_DISPLAY)
    encoder.begin(&I2CBus);
    // __debugS(D, PSTR("[\tsetupEncoder: After encoder.begin() ]"));
    uint8_t ver = encoder.queryVersion();
    __debugS(D, PSTR("[\tsetupEncoder: LeoNerd's OLED Module version: %d ]"), ver);
    if (ver < 2) {
      __debugS(W, PSTR("[\tsetupEncoder: LeoNerd's OLED Module version mismatch, you need version 2! ]"));
    }
    else {
      encoder.setKeyBeepMask(BEEP_NONE);
      encoder.setButtonHoldTime(120);
      encoder.setDoubleClickEnabled(true);
    }
  #else
    encoder.setDoubleClickEnabled(true); // enable doubleclick on the rotary encoder
    __debugS(D, PSTR("[\tsetupEncoder: Encoder initialized. ]"));
  #endif
#endif
}

void setupSteppers(){

  uint16_t maxSpeed = translateSpeed(smuffConfig.maxSpeed[SELECTOR], SELECTOR);
  uint16_t accelSpeed = translateSpeed(smuffConfig.accelSpeed[SELECTOR], SELECTOR);
  steppers[SELECTOR] = ZStepper(SELECTOR, (char *)"SEL", X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, accelSpeed, maxSpeed);
  steppers[SELECTOR].setEndstop(X_END_PIN, smuffConfig.endstopTrg[SELECTOR], ZStepper::MIN, 1, isrEndstopX);
  steppers[SELECTOR].stepFunc = overrideStepX;
  steppers[SELECTOR].setMaxStepCount(smuffConfig.maxSteps[SELECTOR]);
  steppers[SELECTOR].setStepsPerMM(smuffConfig.stepsPerMM[SELECTOR]);
  steppers[SELECTOR].setInvertDir(smuffConfig.invertDir[SELECTOR]);
  steppers[SELECTOR].setAccelDistance(smuffConfig.accelDist[SELECTOR]);
  steppers[SELECTOR].setStopOnStallDetected(false);
  steppers[SELECTOR].setStallThreshold(smuffConfig.stepperMaxStallCnt[SELECTOR]);
  if (smuffConfig.stepperStealth[SELECTOR])
  {
    steppers[SELECTOR].setStopOnStallDetected(smuffConfig.stepperStopOnStall[SELECTOR]);
    #ifdef HAS_TMC_SUPPORT
      if (STALL_X_PIN > 0)
        attachInterrupt(digitalPinToInterrupt(STALL_X_PIN), isrStallDetectedX, FALLING);
    #endif
  }
  if (smuffConfig.stepperMode[SELECTOR] == 0 && smuffConfig.ms3config[SELECTOR] > 0)
  {
    #if defined(MS3_X) && MS3_X > 0
      pinMode(MS3_X, OUTPUT);
      digitalWrite(MS3_X, smuffConfig.ms3config[SELECTOR] == 1 ? LOW : HIGH);
    #endif
  }

#if !defined(SMUFF_V5) && !defined(SMUFF_V6S) && !defined(USE_DDE)
  maxSpeed = translateSpeed(smuffConfig.maxSpeed[REVOLVER], REVOLVER);
  accelSpeed = translateSpeed(smuffConfig.accelSpeed[REVOLVER], REVOLVER);
  steppers[REVOLVER] = ZStepper(REVOLVER, (char *)"REV", Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, accelSpeed, maxSpeed);
  steppers[REVOLVER].setEndstop(Y_END_PIN, smuffConfig.endstopTrg[REVOLVER], ZStepper::ORBITAL, 1, isrEndstopY);
  steppers[REVOLVER].stepFunc = overrideStepY;
  steppers[REVOLVER].setMaxStepCount(smuffConfig.stepsPerRevolution);
  steppers[REVOLVER].setStepsPerDegree(smuffConfig.stepsPerRevolution / 360);
  steppers[REVOLVER].endstopFunc = endstopEventY;
  steppers[REVOLVER].setInvertDir(smuffConfig.invertDir[REVOLVER]);
  steppers[REVOLVER].setAccelDistance(smuffConfig.accelDist[REVOLVER]);
  steppers[REVOLVER].setStopOnStallDetected(false);
  steppers[REVOLVER].setStallThreshold(smuffConfig.stepperMaxStallCnt[REVOLVER]);
  if (smuffConfig.stepperStealth[REVOLVER])
  {
    steppers[REVOLVER].setStopOnStallDetected(smuffConfig.stepperStopOnStall[REVOLVER]);
    #ifdef HAS_TMC_SUPPORT
      if (STALL_Y_PIN > 0)
        attachInterrupt(digitalPinToInterrupt(STALL_Y_PIN), isrStallDetectedY, FALLING);
    #endif
  }
  if (smuffConfig.stepperMode[REVOLVER] == 0 && smuffConfig.ms3config[REVOLVER] > 0)
  {
    #if defined(MS3_Y) && MS3_Y > 0
      pinMode(MS3_Y, OUTPUT);
      digitalWrite(MS3_Y, smuffConfig.ms3config[REVOLVER] == 1 ? LOW : HIGH);
    #endif
  }

#else
  #if !defined(SMUFF_V6S) && !defined(USE_DDE)
  // we don't use the Revolver stepper but a servo instead, although
  // create a dummy instance
  steppers[REVOLVER] = ZStepper(REVOLVER, (char *)"REV", 0, 0, Y_ENABLE_PIN, 0, 0);
  #else
  // except for V6S, which uses a linear stepper instead of a servo
  // or for Direct Drive Extruder
  maxSpeed = translateSpeed(smuffConfig.maxSpeed[REVOLVER], REVOLVER);
  accelSpeed = translateSpeed(smuffConfig.accelSpeed[REVOLVER], REVOLVER);
  steppers[REVOLVER] = ZStepper(REVOLVER, (char *)"REV", Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, accelSpeed, maxSpeed);
  #if defined(SMUFF_V6S)
    steppers[REVOLVER].setEndstop(Y_END_PIN, smuffConfig.endstopTrg[REVOLVER], ZStepper::MIN, 1, isrEndstopY);
    steppers[REVOLVER].setMaxStepCount(smuffConfig.stepsPerMM[REVOLVER]*10);  // max. movement 10mm
  #else
    steppers[REVOLVER].setEndstop(Y_END_PIN, smuffConfig.endstopTrg[REVOLVER], ZStepper::MINMAX, 1, isrEndstopY);
    steppers[REVOLVER].setMaxStepCount(smuffConfig.stepsPerMM[REVOLVER]*300); // max. movement 300mm
  #endif
  steppers[REVOLVER].stepFunc = overrideStepY;
  steppers[REVOLVER].setStepsPerMM(smuffConfig.stepsPerMM[REVOLVER]);
  steppers[REVOLVER].endstopFunc = endstopEventY;
  steppers[REVOLVER].setInvertDir(smuffConfig.invertDir[REVOLVER]);
  steppers[REVOLVER].setAccelDistance(smuffConfig.accelDist[REVOLVER]);
  steppers[REVOLVER].setStopOnStallDetected(false);
  steppers[REVOLVER].setStallThreshold(smuffConfig.stepperMaxStallCnt[REVOLVER]);
  if (smuffConfig.stepperStealth[REVOLVER])
  {
    steppers[REVOLVER].setStopOnStallDetected(smuffConfig.stepperStopOnStall[REVOLVER]);
    #ifdef HAS_TMC_SUPPORT
      if (STALL_Y_PIN > 0)
        attachInterrupt(digitalPinToInterrupt(STALL_Y_PIN), isrStallDetectedY, FALLING);
    #endif
  }
  if (smuffConfig.stepperMode[REVOLVER] == 0 && smuffConfig.ms3config[REVOLVER] > 0)
  {
    #if defined(MS3_Y) && MS3_Y > 0
      pinMode(MS3_Y, OUTPUT);
      digitalWrite(MS3_Y, smuffConfig.ms3config[REVOLVER] == 1 ? LOW : HIGH);
    #endif
  }
    #if defined(SMUFF_V6S)
      __debugS(D, PSTR("[\tsetupSteppers: Y-Stepper initialized for V6S ]"));
    #else
      __debugS(D, PSTR("[\tsetupSteppers: Y-Stepper initialized for DDE ]"));
    #endif
  #endif
#endif

  maxSpeed = translateSpeed(smuffConfig.maxSpeed[FEEDER], FEEDER);
  accelSpeed = translateSpeed(smuffConfig.accelSpeed[FEEDER], FEEDER);
  steppers[FEEDER] = ZStepper(FEEDER, (char *)"FED", Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, accelSpeed, maxSpeed);
  steppers[FEEDER].setEndstop(Z_END_PIN, smuffConfig.endstopTrg[FEEDER], ZStepper::MINMAX, 1, isrEndstopZ);
  if (Z_END2_PIN > 0) {
    steppers[FEEDER].setEndstop(Z_END2_PIN, smuffConfig.endstopTrg[3], ZStepper::MINMAX, 2, isrEndstopZ2); // optional
  }
  steppers[FEEDER].stepFunc = overrideStepZ;
  steppers[FEEDER].setStepsPerMM(smuffConfig.stepsPerMM[FEEDER]);
  steppers[FEEDER].endstopFunc = endstopEventZ;
  steppers[FEEDER].endstop2Func = endstopEventZ2;
  steppers[FEEDER].setInvertDir(smuffConfig.invertDir[FEEDER]);
  steppers[FEEDER].setAccelDistance(smuffConfig.accelDist[FEEDER]);
  steppers[FEEDER].setStopOnStallDetected(false);
  steppers[FEEDER].setStallThreshold(smuffConfig.stepperMaxStallCnt[FEEDER]);
  if (smuffConfig.stepperStealth[FEEDER])
  {
    steppers[FEEDER].setStopOnStallDetected(smuffConfig.stepperStopOnStall[FEEDER]);
    #ifdef HAS_TMC_SUPPORT
      if (STALL_Z_PIN > 0)
        attachInterrupt(digitalPinToInterrupt(STALL_Z_PIN), isrStallDetectedZ, FALLING);
    #endif
  }
  if (smuffConfig.stepperMode[FEEDER] == 0 && smuffConfig.ms3config[FEEDER] > 0)
  {
    #if defined(MS3_Z) && MS3_Z > 0
      pinMode(MS3_Z, OUTPUT);
      digitalWrite(MS3_Z, smuffConfig.ms3config[FEEDER] == 1 ? LOW : HIGH);
    #endif
  }

  for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
    steppers[i].runAndWaitFunc = runAndWait;
    steppers[i].runNoWaitFunc = runNoWait;
    steppers[i].setEnabled(true);
  }

  __debugS(D, PSTR("[\tsetupSteppers: init steppers DONE ]"));
  for (uint8_t i = 0; i < MAX_TOOLS; i++) {
    swapTools[i] = i;
  }
  //__debugS(D, PSTR("[ setupSteppers: init tool swaps DONE ]"));
}

#ifdef HAS_TMC_SUPPORT
bool hwSerialInit = false;

TMC2209Stepper *initDriver(uint8_t axis, uint16_t rx_pin, uint16_t tx_pin)
{
  uint8_t mode = smuffConfig.stepperMode[axis];
  bool tmode = smuffConfig.stepperStealth[axis];
  int8_t stall = smuffConfig.stepperStall[axis];
  uint16_t current = smuffConfig.stepperPower[axis];
  uint16_t msteps = smuffConfig.stepperMicrosteps[axis];
  int8_t csmin = smuffConfig.stepperCSmin[axis];
  int8_t csmax = smuffConfig.stepperCSmax[axis];
  int8_t csdown = smuffConfig.stepperCSdown[axis];
  float rsense = smuffConfig.stepperRSense[axis];
  uint8_t drvrAdr = (uint8_t)smuffConfig.stepperAddr[axis];
  int8_t toff = smuffConfig.stepperToff[axis];
  if(toff == -1) {
     toff = (tmode ? 4 : 3);
  }

  if (mode == 0)
  {
    __debugS(D, PSTR("[\tinitDriver: Driver for %c-axis skipped ]"), axis == FEEDER2 ? 'E' : 'X'+axis);
    return nullptr;
  }

  #if defined(TMC_SERIAL) && defined(TMC_HW_SERIAL)
  TMC2209Stepper *driver = new TMC2209Stepper(&TMC_SERIAL, rsense, drvrAdr);
  #else
  TMC2209Stepper *driver = new TMC2209Stepper(rx_pin, tx_pin, rsense, drvrAdr);
  #endif
  __debugS(D, PSTR("[\tinitDriver: Driver for %c-axis created on address %d ]"), axis == FEEDER2 ? 'E' : 'X'+axis, drvrAdr);

  #if defined(HAS_TMC_SUPPORT) && !defined(TMC_HW_SERIAL)
  driver->beginSerial(TMC_SW_BAUDRATE);
  #endif
  driver->Rsense = rsense;

  bool intRsense = driver->internal_Rsense();
  if(!intRsense) {
    // Although the TMC datasheet claims to set Rsense to internal, the following code will lead to
    // non functional stepper drivers on SKR E3 boards!
    // This might be because it conflicts somehow with the OTP on the stepper chips soldered onto the E3 1.2 and E3 2.0.
    // It's a different picture for external (Pololu style) stepper drivers.
    // So for now, this initialisation is skipped when compiling for E3 1.2 / 2.0 boards.
    #if !defined(__BRD_SKR_MINI_E3)
    __debugS(D, PSTR("[\tinitDriver: Setting RSense to internal ]"));
    steppers[axis].setEnabled(false);
    driver->internal_Rsense(true);
    #endif
  }
  steppers[axis].setEnabled(true);
  //intRsense = driver->internal_Rsense(); __debugS(D, PSTR("[ initDriver: RSense internal is %s ]"), intRsense ? "true" : "false");

  driver->toff(toff);
  driver->blank_time(36);
  driver->I_scale_analog(false);    // use internal Vref; set to true for external Vref
  driver->rms_current(current);     // set current in mA
  driver->pdn_disable(true);        // PDN disabled for UART operation
  driver->mstep_reg_select(1);      // set microstepping
  driver->microsteps(msteps);
  __debugS(D, PSTR("[\tinitDriver: Basic Init done for %c-Axis. ]"), (axis==FEEDER2) ? 'E' : 'X' + axis);

  // setup StallGuard only if TMode is set to true
  // otherwise put it in SpreadCycle mode
  if (tmode) {
    #if defined(TMC_TYPE_2130)
    stall = (uint8_t)map(stall, 1, 255, -63, 63); // remap values for TMC2130 (not tested yet!)
    #endif
    setDriverSpreadCycle(driver, false, stall, csmin, csmax, csdown, toff); // set StealthChop (enable StallGuard)
  }
  else {
    setDriverSpreadCycle(driver, true, stall, csmin, csmax, csdown, toff); // set SpreadCycle (disable StallGuard)
  }
  isUsingTmc = true;
  return driver;
}

void setDriverSpreadCycle(TMC2209Stepper *driver, bool spread, uint8_t stallThrs, uint8_t csmin, uint8_t csmax, uint8_t csdown, uint8_t toff)
{
  if (spread) { // set SpreadCycle (disable StallGuard)
    driver->en_spreadCycle(true);
    driver->SGTHRS(0);
    driver->TCOOLTHRS(0);
    driver->TPWMTHRS(0);
    driver->pwm_autoscale(false);
    driver->pwm_autograd(false);
    driver->pwm_ofs(0);
  }
  else { // set StealthChop (enable StallGuard)
    driver->en_spreadCycle(false);
    driver->TCOOLTHRS(0xFFFFF);
    driver->SGTHRS(stallThrs);
    driver->pwm_ofs(1);
    driver->pwm_autoscale(true);
    driver->pwm_autograd(true);
  }
  driver->semin(csmin);
  driver->semax(csmax);
  driver->sedn(csdown);
  driver->seup(csdown);
}

void setupTMCDrivers()
{
  isUsingTmc = false;
#if defined(TMC_HW_SERIAL)
  // make sure init of the HW-Serial is done only once
  if (!hwSerialInit) {
    __debugS(D, PSTR("[\tsetupTMCDrivers: Initializing TMC HW-Serial with Baudrate %ld ]"), TMC_HW_BAUDRATE);
    TMC_SERIAL.begin(TMC_HW_BAUDRATE);
    hwSerialInit = true;
  }
  drivers[SELECTOR] = initDriver(SELECTOR, 0, 0);
  drivers[REVOLVER] = initDriver(REVOLVER, 0, 0);
  drivers[FEEDER]   = initDriver(FEEDER, 0, 0);
  drivers[FEEDER2]  = initDriver(FEEDER2, 0, 0);             // dummy E-Stepper init
#else
#if defined(X_SERIAL_TX_PIN)
  drivers[SELECTOR] = initDriver(SELECTOR, X_SERIAL_TX_PIN, X_SERIAL_TX_PIN);
#endif
#if defined(Y_SERIAL_TX_PIN)
  drivers[REVOLVER] = initDriver(REVOLVER, Y_SERIAL_TX_PIN, Y_SERIAL_TX_PIN);
#endif
#if defined(Z_SERIAL_TX_PIN)
  drivers[FEEDER] = initDriver(FEEDER, Z_SERIAL_TX_PIN, Z_SERIAL_TX_PIN);
#endif
#if defined(E_SERIAL_TX_PIN)
  drivers[FEEDER2] = initDriver(FEEDER2, E_SERIAL_TX_PIN, E_SERIAL_TX_PIN);
#endif
#endif
  //__debugS(D, PSTR("[ setupTMCDrivers: initialized ]"));

#if defined(STALL_X_PIN)
  if (STALL_X_PIN > 0)
    pinMode(STALL_X_PIN, INPUT_PULLUP);
#endif
#if defined(STALL_Y_PIN)
  if (STALL_Y_PIN > 0)
    pinMode(STALL_Y_PIN, INPUT_PULLUP);
#endif
#if defined(STALL_Z_PIN)
  if (STALL_Z_PIN > 0)
    pinMode(STALL_Z_PIN, INPUT_PULLUP);
#endif
  //__debugS(D, PSTR("[ setupTMCDrivers: DONE ]"));
}

#endif

timerVal_t calcInterval(ZTimer* timer, uint32_t resolution) {
  uint32_t f = timer->getClockFrequency();
  uint32_t psc = timer->getPrescaler();
  timerVal_t ticks = (timerVal_t)((f/psc)*((float)resolution/1000000L));
  return ticks;
}

void setupTimers()
{
#if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)
  // *****
  // Attn:
  //    SW-Serial uses:     TIMER5 (stm32duino SoftwareSerial set by TIMER_SERIAL in SMuFF.h)
  //    Tone uses:          TIMER6 (stm32duino default set by TIMER_TONE in SMuFF.h)
  //    Servo uses:         TIMER7 (stm32duino default)
  //
  //    Steppers use:       TIMER1 CH1
  //    GP timer uses:      TIMER3 CH1 (general, encoder, fan, servo)
  //
  // Warning: If you need to modify this assignment, be sure you know what you do!
  //          Swapping timers and/or channels may lead to a non functioning device or
  //          communication interrupts/breaks. Read the STM32 MCU spec. and check
  //          the stm32duino library settings before you do so.
  // *****
  stepperTimer.setupTimer(ZTimer::_TIMER1, ZTimer::CH1, STEPPER_PSC, 0, isrStepperTimerHandler);  // prescaler set to STEPPER_PSC, timer will be calculated as needed
  stepperTimer.setPreload(false);
  __debugS(D, PSTR("[\tsetupTimers: Stepper timer initialized. Freq: %d MHz, PSC: %s MHz ]"), (int)stepperTimer.getClockFrequency()/1000000, String((float)stepperTimer.getClockFrequency()/1000000/STEPPER_PSC).c_str());

  gpTimer.setupTimer(ZTimer::_TIMER3, ZTimer::CH1, GP_PSC, 0, isrGPTimerHandler);                 // prescaler set to GP_PSC, timer will be set to 100uS
  __debugS(D, PSTR("[\tsetupTimers: GP timer initialized.      Freq: %d MHz, PSC: %s MHz ]"), (int)gpTimer.getClockFrequency()/1000000, String((float)gpTimer.getClockFrequency()/1000000/GP_PSC).c_str());

  stepperTimer.setPriority(1, 0);
  gpTimer.setPriority(0, 0);
  
#endif
  timerVal_t gpt_ticks = calcInterval(&gpTimer, GPTIMER_RESOLUTION);
  gpTimer.setNextInterruptInterval(gpt_ticks);                                  // start general purpose timer
  __debugS(D, PSTR("[\tsetupTimers: GP timer ticks set to: %ld ]"), gpt_ticks);
}
