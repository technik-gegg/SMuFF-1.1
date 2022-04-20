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

Stream                    *debugSerial = &Serial2;        // send debug output to...
Stream                    *logSerial = &Serial;           // send logs to USB by default
Stream                    *terminalSerial = &Serial2;     // send terminal emulation output to ...

#if defined(USE_SW_SERIAL0)
SoftwareSerial            swSerial0(SW_SERIAL_RX_PIN, SW_SERIAL_TX_PIN, false);  // mainly for testing purpose
#endif

dspDriver                 display = INIT_DSP();           // initialize display driver (see macro in according Display_xxx.h)
ZStepper                  steppers[NUM_STEPPERS];
ZTimer                    stepperTimer;
ZTimer                    gpTimer;
#if defined(USE_ZSERVO)
ZTimer                    servoTimer;
ZServo                    servoWiper;
ZServo                    servoLid;
ZServo                    servoCutter;
#else
Servo                     servoWiper;
Servo                     servoLid;
Servo                     servoCutter;
#endif
ZFan                      fan;

#if defined(__STM32G0XX)
HardwareSerial            Serial4(RX4_PIN, TX4_PIN);
#endif


#if defined(USE_LEONERD_DISPLAY)
LeoNerdEncoder            encoder(I2C_ENCODER_ADDRESS);
#else
  #if !defined(USE_SERIAL_DISPLAY)
  ClickEncoder            encoder(ENCODER1_PIN, ENCODER2_PIN, ENCODER_BUTTON_PIN, 4);
  #else 
  ClickEncoder            encoder(0, 0, 0, 0);
  #endif
#endif

#if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)
  #if defined(USE_SPLITTER_ENDSTOPS)
  ZEStopMux               splitterMux;
  #endif
#endif

#ifdef HAS_TMC_SUPPORT
TMC2209Stepper            *drivers[NUM_STEPPERS+1];
#endif

#if defined(MULTISERVO)
Adafruit_PWMServoDriver servoPwm = Adafruit_PWMServoDriver(I2C_SERVOCTL_ADDRESS, Wire);
int8_t                    servoMapping[18] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, -1, -1}; // last two are used for the Wiper and Cutter mapping
uint8_t                   servoPosClosed[16] = {90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};
#else
uint8_t                   servoPosClosed[MAX_TOOLS];
float                     stepperPosClosed[MAX_TOOLS];
#endif

String                    serialBuffer0, serialBuffer1, serialBuffer2, serialBuffer3;
volatile byte             nextStepperFlag = 0;
volatile byte             remainingSteppersFlag = 0;
int8_t                    toolSelections[MAX_TOOLS];
uint8_t                   remoteKey = REMOTE_NONE;
volatile uint16_t         bracketCnt = 0;
volatile uint16_t         jsonPtr = 0;
static volatile uint16_t  intervalMask;       // bit-mask for interval reached
uint16_t                  mmsMin = 1;         // minimum moving speed for stepper in mm/s
uint16_t                  mmsMax = 800;       // maximum moving speed for stepper in mm/s
uint16_t                  speedIncrement = 5; // increment for speeds in menus
uint32_t                  pwrSaveTime;
uint32_t                  uploadStart = 0;
uint32_t                  lastEvent = 0;
volatile uint16_t         gpTimer1ms = 0;
static volatile unsigned  tickCounter = 0;
static volatile unsigned  fastLEDTickCounter = 0;
volatile unsigned long    lastEncoderButtonTime = 0;
volatile bool             sdRemoved = false;
volatile bool             initDone = false;   // enables sending periodical status information to serial ports
volatile bool             isPwrSave = false;
volatile bool             showMenu = false;
volatile bool             lastZEndstopState = false;
volatile bool             isIdle = false;
bool                      tmcWarning = false;
bool                      isUsingTmc = false;
bool                      isQuote = false;
bool                      isFuncKey = false;
bool                      isCtlKey = false;
bool                      ignoreQuotes = false;
volatile bool             isUpload = false;
bool                      isReceiving = false;
volatile bool             refreshingDisplay = false;
volatile uint16_t         flipDbgCnt;
volatile uint32_t         __systick = 0;        // for benchmarking only

volatile IntervalHandler intervalHandlers[] = {
    {  10, 0, every10ms}, 
    {  20, 0, every20ms}, 
    {  50, 0, every50ms}, 
    { 100, 0, every100ms}, 
    { 250, 0, every250ms}, 
    { 500, 0, every500ms}, 
    {1000, 0, every1s}, 
    {2000, 0, every2s}, 
    {5000, 0, every5s}
};

// forward declarations of some locally used functions
void startStepperInterval();
void fncKey1();
void fncKey2();
void fncKey3();
void fncKey4();
void loopEx();

//=====================================================================================================
// Benchmarking helper functions
//=====================================================================================================
#if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)
void HAL_SYSTICK_Callback(void) {
  __systick++;    // 1000 Hz frequency
  // decrement all milliseconds counters for periodic functions and
  // set a flag when the timeout has been reached
  for (uint8_t i = 0; i < ArraySize(intervalHandlers); i++) {
    intervalHandlers[i].val--;
    if (intervalHandlers[i].val <= 0) {
      intervalMask |= _BV(i);
      intervalHandlers[i].val = intervalHandlers[i].period;
    }
  }
}

#define DELAY_LOOP(STPR)  for(int i=0; i < smuffConfig.stepDelay[STPR]; i++) asm("NOP");

volatile uint32_t        *stepper_reg_X = &(digitalPinToPort(X_STEP_PIN)->BSRR);
volatile uint32_t        *stepper_reg_Y = &(digitalPinToPort(Y_STEP_PIN)->BSRR);
volatile uint32_t        *stepper_reg_Z = &(digitalPinToPort(Z_STEP_PIN)->BSRR);
// preset of set/reset values to make the interrupt routine faster
uint32_t                  pinMask_Xs = digitalPinToBitMask(X_STEP_PIN);
uint32_t                  pinMask_Ys = digitalPinToBitMask(Y_STEP_PIN);
uint32_t                  pinMask_Zs = digitalPinToBitMask(Z_STEP_PIN);
uint32_t                  pinMask_Xr = digitalPinToBitMask(X_STEP_PIN) << 16;
uint32_t                  pinMask_Yr = digitalPinToBitMask(Y_STEP_PIN) << 16;
uint32_t                  pinMask_Zr = digitalPinToBitMask(Z_STEP_PIN) << 16;

volatile uint32_t         *debugPin = &(digitalPinToPort(DEBUG_PIN)->BSRR);
uint32_t                  pinMask_Debug = digitalPinToBitMask(DEBUG_PIN);
bool                      debugToggle = false;

void fastFlipDbg() {
  *debugPin = (debugToggle) ? pinMask_Debug : pinMask_Debug << 16;
  debugToggle = !debugToggle;
}

#endif

//=====================================================================================================
// Override functions called by ZStepper library
//=====================================================================================================

void overrideStepX(pin_t pin) {
  #if defined(X_STEP_PIN_NAME)
    *stepper_reg_X = pinMask_Xs;
    DELAY_LOOP(SELECTOR)
    *stepper_reg_X = pinMask_Xr;
  #else
    STEP_HIGH_X
    delayMicroseconds(smuffConfig.stepDelay[SELECTOR]);
    STEP_LOW_X
  #endif
}

void overrideStepY(pin_t pin) {
  #if defined(Y_STEP_PIN_NAME)
    *stepper_reg_Y = pinMask_Ys;
    DELAY_LOOP(REVOLVER)
    *stepper_reg_Y = pinMask_Yr;
  #else
    STEP_HIGH_Y
    delayMicroseconds(smuffConfig.stepDelay[REVOLVER]);
    STEP_LOW_Y
  #endif
}

void overrideStepZ(pin_t pin) {
  #if defined(Z_STEP_PIN_NAME)
    *stepper_reg_Z = pinMask_Zs;
    DELAY_LOOP(FEEDER)
    *stepper_reg_Z = pinMask_Zr;
  #else
    STEP_HIGH_Z
    delayMicroseconds(smuffConfig.stepDelay[FEEDER]);
    STEP_LOW_Z
  #endif
}


void endstopEventX() {
  // add your code here it you want to hook into the endstop event for the Selector
}

void endstopEventY() {
  // add your code here it you want to hook into the endstop event for the Revolver
}

void endstopEventZ() {
  // add your code here it you want to hook into the 1st endstop event for the Feeder
}

void endstopEventZ2() {
  // add your code here it you want to hook into the 2nd endstop event for the Feeder
}


//=====================================================================================================
// ISR handler functions
//=====================================================================================================

/*
  Interrupt handler for Endstops
 */
void isrEndstopX() {
  int hit = (int)digitalRead(steppers[SELECTOR].getEndstopPin(1))==steppers[SELECTOR].getEndstopState(1);
  steppers[SELECTOR].setEndstopHit(hit, 1);
  steppers[SELECTOR].adjustPositionOnEndstop(1);
  if(hit)
    endstopEventX();
  __debugS(DEV, PSTR("X Hit"));
}

void isrEndstopY() {
  int hit = (int)digitalRead(steppers[REVOLVER].getEndstopPin(1))==steppers[REVOLVER].getEndstopState(1);
  steppers[REVOLVER].setEndstopHit(hit, 1);
  steppers[REVOLVER].adjustPositionOnEndstop(1);
  if(hit)
    endstopEventY();
  __debugS(DEV, PSTR("Y Hit"));
}

void isrEndstopZ() {
  int hit = (int)digitalRead(steppers[FEEDER].getEndstopPin(1))==steppers[FEEDER].getEndstopState(1);
  steppers[FEEDER].setEndstopHit(hit, 1);
  steppers[FEEDER].adjustPositionOnEndstop(1);
  if(hit)
    endstopEventZ();
  __debugS(DEV, PSTR("Z Hit"));
}

void isrEndstopZ2() {
  int hit = (int)digitalRead(steppers[FEEDER].getEndstopPin(2))==steppers[FEEDER].getEndstopState(2);
  steppers[FEEDER].setEndstopHit(hit, 2);
  steppers[FEEDER].adjustPositionOnEndstop(2);
  if(hit)
    endstopEventZ2();
}

/*
  Interrupt handler for StallGuard on TMC2209
 */
void isrStallDetectedX() { steppers[SELECTOR].stallDetected(); }
void isrStallDetectedY() { steppers[REVOLVER].stallDetected(); }
void isrStallDetectedZ() { steppers[FEEDER].stallDetected(); }


/*
  Interrupt handler for FastLED
*/
void isrFastLEDTimerHandler() {
  #if defined(USE_FASTLED_BACKLIGHT) || defined(USE_FASTLED_TOOLS)
  if(!initDone || isUpload)
    return;
  fastLEDTickCounter++;
  bool slowDown = fastLEDTickCounter % 25 != 0;
  if(fastLEDTickCounter % 3 == 0) {         // about every 60ms
    fastLedHue++;                           // used for some color changing/fading effects
  }
  else if(fastLEDTickCounter % 5 == 0) {    // about every 100ms
    if(fastLedStatus > FASTLED_STAT_NONE) {
      setFastLEDStatus();
    }
    else {
      setFastLEDTools();
    }
    if(fastLedStatus == FASTLED_STAT_NONE) {     // slow down refresh if LED animation isn't being used
      if(!smuffConfig.useIdleAnimation && slowDown)    // about every 500ms
        return;
    }
    if(isServoPulseComplete(SERVO_LID))   // refresh FastLED only if the pulse cycle of the Lid servo is completed
      refreshFastLED();                   // otherwise it'll interrupt the servo ISR and make it jitter
  }
  #endif
}

/*
  Interrupt handler for the general purpose timer
  Fires every GPTIMER_RESOLUTION uS and thus increments the general counter
  every millisecond.
  Serves also as a handler/dispatcher for periodicals / encoder / fan.
*/
void isrGPTimerHandler() {

  #if defined(USE_ZSERVO)  
  isrServoHandler();        // call servo handler
  #endif
  tickCounter++; // increment tick counter
  if (tickCounter % gpTimer1ms == 0) {
    // each millisecond...
    if (initDone) {
      #if !defined(USE_LEONERD_DISPLAY) && !defined(USE_SERIAL_DISPLAY)
      encoder.service();      // service the rotary encoder
      #endif
      isrFanTimerHandler();   // call the fan interrupt routines also every GPTIMER_RESOLUTION uS
    }
  }
  #ifdef FLIPDBG
  if (initDone && (tickCounter % flipDbgCnt == 0)) {
    FLIPDBG               // flips the debug pin polarity which is supposed to generate 
                          // a periodical signal (default: 500 Hz) for debug purposes
                          // frequency can be modified in smuffConfig.dbgFreq
  }
  #endif
}

void isrSdCardDetected() {
  sdRemoved = digitalRead(SD_DETECT_PIN) == HIGH;
}


//=====================================================================================================
// ISR handler and helper functions needed for ZStepper library
//=====================================================================================================

void isrStepperTimerHandler() {

  // fastFlipDbg();         // for debugging only
  stepperTimer.stop();
  timerVal_t tmp = stepperTimer.getOverflow();

  register uint8_t mask;
  for (uint8_t i=0; i < NUM_STEPPERS; i++) {
    if (remainingSteppersFlag == 0)
      break;
    mask = _BV(i);
    if (!(mask & remainingSteppersFlag))  // current stepper doesn't need movement, continue with next one
      continue;

    if (!(mask & nextStepperFlag)) {
      // current stepper is not marked as the next one to move, adjust duration for next turn
      steppers[i].subtractDuration(tmp);
      continue;
    }

    if(steppers[i].handleISR()) {
      remainingSteppersFlag &= ~mask;     // mark current stepper as finished if handleISR returned true
      // __debugS(D, PSTR("[%s] done"), steppers[i].getDescriptor());
    }
  }
  // fastFlipDbg();       // for debugging only
  startStepperInterval();
}

void startStepperInterval() {
  timerVal_t minDuration = 0xFFFF;

  for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
    // find shortest duration (overflow value) of all steppers in action
    if ((_BV(i) & remainingSteppersFlag)) {
      if(steppers[i].getInterruptFactor() == 0)
        steppers[i].isLTDuration(&minDuration);
    }
  }

  nextStepperFlag = 0;
  for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
    // mark stepper with lowest duration to move next (pseudo sync)
    if ((_BV(i) & remainingSteppersFlag) && (steppers[i].isDuration(minDuration) || steppers[i].checkInterrupt()))
      nextStepperFlag |= _BV(i);
  }

  if (remainingSteppersFlag == 0) { // no more stepper movements pending, stop timer
    // stop stepper timer
    stepperTimer.stop();
    stepperTimer.setOverflow(0x10000);
  }
  else {
    // stepper movements still pending, set new stepper timer overflow value
    stepperTimer.setNextInterruptInterval(minDuration);
  }
}

void calcSyncMovementFactor() {

  if(!smuffConfig.allowSyncSteppers)
    return;
  
  long maxTotal = 0;
  uint8_t maxIdx = 0;

  // find largest pending movement (most total steps)
  for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
    if (!(_BV(i) & remainingSteppersFlag))
      continue;
    long ts = steppers[i].getTotalSteps();
    if(ts > maxTotal) {
      maxTotal = ts;
      maxIdx  = i;
    }
  }

  // find all smaller movements and set the factor proportionally, i.e.
  // slow down movement so all steppers start and stop in sync
  for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
    if (!(_BV(i) & remainingSteppersFlag))
      continue;
    long ts = steppers[i].getTotalSteps();
    if(i != maxIdx) {
      steppers[i].setInterruptFactor(maxTotal/ts);
      __debugS(D, PSTR("IntrFactor: %d [%s]"), steppers[i].getInterruptFactor(), steppers[i].getDescriptor());
    }
  }
}

void runNoWait(int8_t index) {
  if (index != -1)
    remainingSteppersFlag |= _BV(index);
  else 
    calcSyncMovementFactor();     // calculate movement factor if more than one stepper has to move
  startStepperInterval();
}

void runAndWait(int8_t index) {
  static uint32_t lastFeedUpdate = 0;
  bool dualFeeder = false;
  #if defined(USE_DDE)
  if(!asyncDDE && (remainingSteppersFlag & _BV(FEEDER)) && (remainingSteppersFlag & _BV(DDE_FEEDER)))
    dualFeeder = true;
  #endif
  runNoWait(index);
  while (remainingSteppersFlag)
  {
    checkSerialPending(); // not a really nice solution but needed to check serials for "Abort" command in PMMU mode
    #if defined(USE_DDE)
    // stop internal feeder when the DDE feeder has stopped
    if(dualFeeder && ((remainingSteppersFlag & _BV(FEEDER)) && !(remainingSteppersFlag & _BV(DDE_FEEDER)))) {
      steppers[FEEDER].setMovementDone(true);
      remainingSteppersFlag = 0;
      break;
    }
    #endif
    if(index != -1 && !(remainingSteppersFlag & _BV(index))) {
      break;
    }
  }
}

//=====================================================================================================
// SETUP
//=====================================================================================================

void setup() {

  const char* after = PSTR("[ \033[36m=== %-30s o.k. ===\033[0m ]");

  serialBuffer0.reserve(30);  // initialize serial buffers
  serialBuffer1.reserve(30);
  serialBuffer2.reserve(30);
  serialBuffer3.reserve(30);
  readDebugLevel();           // read Debug Level settings from SD-Card

  // Setup a fixed baudrate until the config file was read.
  // This baudrate is the default setting on the ESP32 while
  // booting up, so exceptions thrown can be shown in terminal app
  Serial.begin(115200);
  if (CAN_USE_SERIAL1) Serial1.begin(115200);
  if (CAN_USE_SERIAL2) Serial2.begin(115200);
  if (CAN_USE_SERIAL3) Serial3.begin(115200);

  __debugS(SP, PSTR("[\033[35m setup start \033[0m]"));
  
  // ------------------------------------------------------------------------------------------------------
  // Disable JTAG/SWI on these boards:
  //  On the SKR E3-DIP it's because of the buzzer signal (PA15 / JTDI).
  //  On the SKR E3 V2.0 it's because of the SCL signal (PA15 / JTDI).
  // ------------------------------------------------------------------------------------------------------
  // IMPORTANT: If debug ports are disabled, flashing the firmware / bootloader using ST-Link will fail!
  //            Hence, if you need to flash the MCU using ST-Link at some point, better comment out
  //            -D DISABLE_DEBUG_PORT in platformio.ini for that particular board.
  // ------------------------------------------------------------------------------------------------------
  #if defined(DISABLE_DEBUG_PORT)
    pin_DisconnectDebug(PA_15);   // disable Serial wire JTAG configuration (STM32F1 only)
    __debugS(D, PSTR("[\tdebug ports disabled ]"));
  #endif
  #if defined(STM32_REMAP_SPI)
    afio_remap(AFIO_REMAP_SPI1); // remap SPI3 to SPI1 if a "normal" display is being used
    __debugS(D, PSTR("[\tSPI3 remapped to SPI1 ]"));
  #endif

  initUSB();                // init the USB serial so it's being recognized by the Windows-PC
  initFastLED();            // init FastLED if configured

  #if defined(USE_I2C)
    // don't scan I2C if no I2C devices are being used; Scan will take ages
    enumI2cDevices(1);
    __debugS(D, after, "enumerating I2C Devices on bus 1");
  #endif
  #if defined(USE_SPLITTER_ENDSTOPS)
    enumI2cDevices(2);
    __debugS(D, after, "enumerating I2C Devices on bus 2");
  #endif

  setupDisplay();                                     // setup display first in order to show error messages if neccessary
  __debugS(SP, after, "setup Display");
  #if !defined(USE_SERIAL_DISPLAY)
    delay(250);
    setupEncoder();                                   // setup encoder
    __debugS(SP, after, "setup Encoder");
  #endif

  __debugS(DEV, PSTR("[\treading Configs... ]"));
  if (readConfig()) {                                 // read SMUFF.json from SD-Card
    readTmcConfig();                                  // read TMCDRVR.json from SD-Card
    readServoMapping();                               // read SERVOMAP.json from SD-Card
    readMaterials();                                  // read MATERIALS.json from SD-Card
    #if defined(SMUFF_V6S)
      readRevolverMapping();                          // read REVOLVERMAPS.json from SD-Card
    #endif
  }
  __debugS(SP, after, "read Config");
  setContrast(smuffConfig.lcdContrast);               // reset display contrast after reading config
  initHwDebug();                                      // init hardware debugging

  setupSerial();                                      // setup all serial ports
  __debugS(SP, after, "setup Serial");

  if(smuffConfig.useDuet) {
    // Duet3D uses Serial 1 for communication, Serial 3 for PanelDue (optional)
    // in this case, all debug outputs are sent to USB serial
    debugSerial = &Serial;                            // send debug output to USB
    terminalSerial = &Serial;                         // send terminal emulation output to USB
    __debugS(SP, PSTR("[\tdebug & terminal serials swapped for Duet3D/RRF]"));
  }
  setupTimers();                                      // setup all timers
  __debugS(SP, after, "setup Timers");
  setupSteppers();                                    // setup all steppers
  __debugS(SP, after, "setup Steppers");
  setupServos();                                      // setup all servos
  __debugS(SP, after, "setup Servos");
  setupRelay();                                       // setup relay board
  __debugS(SP, after, "setup Relay");
  #ifdef HAS_TMC_SUPPORT
    setupTMCDrivers();                                  // setup TMC drivers if any are used
    __debugS(SP, after, "setup TMC drivers");
  #endif
  testFastLED(false);                                 // run a test sequence on backlight FastLEDs
  testFastLED(true);                                  // run a test sequence on tools FastLEDs

  setupSwSerial0();                                   // used only for testing purposes
  setupBacklight();                                   // setup display backlight
  setupDuetSignals();                                 // setup Duet3D signal pins
  setupFan();                                         // setup internal cooling fan
  setupI2C();                                         // setup I2C/TWI devices
  __debugS(SP, after, "setup Misc.");
  getStoredData();                                    // read EEPROM.json from SD-Card; this call must happen after setupSteppers()
  
  #if defined(USE_SPLITTER_ENDSTOPS)
    setupEStopMux();                                  // setup enstops multiplexer if configured
    __debugS(SP, after, "setup EStop Mux");
  #endif
  uint32_t now = millis();
  readSequences();                                    // read sound files from SD-Card sounds folder
  __debugS(D, PSTR("[\tloading sequences took %ld ms ]"), millis()-now);
  __debugS(SP, after, "read Sequences");

  if (smuffConfig.homeAfterFeed)
    moveHome(REVOLVER, false, false);                 // home Revolver / Lid-Servo
  else
    resetRevolver();

  removeFirmwareBin();                                // deletes the firmware.bin file to prevent re-flashing on each boot
  __debugS(SP, after, "remove firmware.bin");

  #if defined(SD_DETECT_PIN)                          // setup SD-Detect ISR
    if(SD_DETECT_PIN > 0) {
      attachInterrupt(digitalPinToInterrupt(SD_DETECT_PIN), isrSdCardDetected, CHANGE);
      __debugS(D, PSTR("[\tSD-Card detect interrupt attached ]"));
    }
  #endif

  #if defined(__BRD_SKR_MINI_E3)                      // applies only to E3 V1.2 and 2.0 / 3.0 with integrated stepper drivers
    if(drivers[SELECTOR] == nullptr || drivers[FEEDER] == nullptr) {
      __debugS(W, PSTR("\nWARNING: Your controller is equipped with onboard TMC stepper drivers"));
      __debugS(W, PSTR("but at least one of them has not being initialized for UART mode"));
      __debugS(W, PSTR("and thus, your stepper motor may overheat!"));
      __debugS(W, PSTR("Please check and change your configuration accordingly!"));
    }
  #endif
  
  __debugS(D, PSTR("[ startup tune ]"));
  startupBeep();                                        // signal startup has finished
  __debugS(SP, PSTR("[\033[35m setup end \033[0m]"));

  for (uint8_t i = 0; i < (uint8_t)ArraySize(intervalHandlers); i++) // init interval handler values
    intervalHandlers[i].val = intervalHandlers[i].period;
  gpTimer1ms = (uint16_t)(1000/GPTIMER_RESOLUTION);

  sendStartResponse(0);                                 // send "start<CR><LF>" to USB serial interface
  if (CAN_USE_SERIAL1)
    sendStartResponse(1);                               // send "start<CR><LF>" to all serial interfaces allowed to use
  if (CAN_USE_SERIAL2 && smuffConfig.hasPanelDue != 2)
    sendStartResponse(2);
  if (CAN_USE_SERIAL3 && smuffConfig.hasPanelDue != 3)
    sendStartResponse(3);

  pwrSaveTime = millis();                               // init value for LCD screen timeout
  initDone = true;                                      // mark init done; enable periodically sending status, if configured
  refreshStatus();
}

#ifdef HAS_TMC_SUPPORT
void reportTMC(uint8_t axis, const char *PROGMEM msg) {
  // for now, only debug message (main screen will show "!" next to "TMC")
  __debugS(W, PSTR("Driver %c: reports '%s'"), axis==FEEDER2 ? 'E' : 'X' + axis, msg);
}

void monitorTMC(uint8_t axis) {
  uint16_t temp;
  if (drivers[axis] != nullptr) {
    // has any error occured?
    if (drivers[axis]->drv_err()) {
      tmcWarning = true;
      // check overtemp. warning
      if (drivers[axis]->otpw()) {
        reportTMC(axis, P_TMC_Status09);
      }
      // check overtemp.
      if (drivers[axis]->ot()) {
        if (drivers[axis]->t157())
          temp = 157;
        if (drivers[axis]->t150())
          temp = 150;
        if (drivers[axis]->t143())
          temp = 143;
        if (drivers[axis]->t120())
          temp = 120;
        char msg[80];
        sprintf_P(msg, PSTR("Overtemp. >= %d°C"), temp);
        reportTMC(axis, msg);
      }
      // check open lines
      if (drivers[axis]->ola())
        reportTMC(axis, P_TMC_Status03);
      if (drivers[axis]->olb())
        reportTMC(axis, P_TMC_Status04);
      // check short to grounds
      if (drivers[axis]->s2ga())
        reportTMC(axis, P_TMC_Status05);
      if (drivers[axis]->s2gb())
        reportTMC(axis, P_TMC_Status06);
    }
  }
}
#endif

//=====================================================================================================
// LOOP and other functions
//=====================================================================================================

static bool sdRemovalSet = false;
static bool firstLoop = false;

void loop() {

  // if(!firstLoop) {     // for extended debugging only
  //   __debugS(D, PSTR("[ Looping... ]"));
  //   firstLoop = true;
  // }

  // Call periodical functions as the timeout has triggered.
  // Add your specific code there, if you need to have something
  // managed periodically.
  // The main loop is the better choice for dispatching, since it'll
  // allow uninterrupted serial I/O.
  for (uint8_t i = 0; i < (uint8_t)ArraySize(intervalHandlers); i++) {
    if (_BV(i) & intervalMask) {
      if(intervalHandlers[i].func != nullptr)
        intervalHandlers[i].func();
      intervalMask &= ~_BV(i);
    }
  }

#if defined(SD_DETECT_PIN)
  // message the user if the SD-Card gets removed
  if (sdRemoved) {
    if (!sdRemovalSet) {
      userBeep();
      leoNerdBlinkRed = true;
      setFastLEDStatus(FASTLED_STAT_ERROR);
      if (isPwrSave) {
        setPwrSave(0);
      }
    }
    isIdle = false;
    sdRemovalSet = true;
    if((__systick % 500) == 0) {
      char tmp[50];
      sprintf_P(tmp, P_SDCardRemoved);
      drawUserMessage(tmp);
    }
    return;
  }
  else
  {
    if (sdRemovalSet) {
      if (initSD(false)) {
        sdRemovalSet = false;
        leoNerdBlinkRed = false;
        setFastLEDStatus(FASTLED_STAT_NONE);
        lastEvent = millis();
        refreshStatus();
      }
    }
  }
#endif

  bool state = feederEndstop();
  if (state != lastZEndstopState) {
    refreshStatus();
    lastZEndstopState = state;
  }
  #if !defined(USE_SERIAL_DISPLAY)
    if(checkUserMessage()) {
      pwrSaveTime = millis();
    }
  #endif

  isIdle = ((uint16_t)((millis() - lastEvent) / 1000) >= smuffConfig.powerSaveTimeout);

  #if !defined(USE_SERIAL_DISPLAY)
  int16_t turn;
  uint8_t button;
  bool isHeld, isClicked;

  if (!showMenu) {
    switch (remoteKey)
    {
      case REMOTE_HOME:
        remoteKey = REMOTE_NONE;
        break;
      case REMOTE_PF1:
        fncKey1();
        remoteKey = REMOTE_NONE;
        break;
      case REMOTE_PF2:
        fncKey2();
        remoteKey = REMOTE_NONE;
        break;
      case REMOTE_PF3:
        fncKey3();
        remoteKey = REMOTE_NONE;
        break;
      case REMOTE_PF4:
        fncKey4();
        remoteKey = REMOTE_NONE;
        break;
    }

    getInput(&turn, &button, &isHeld, &isClicked);
    if (isClicked || turn != 0) {
      if(isClicked && isUpload)
        resetUpload();
      if (isPwrSave) {
        setPwrSave(0);
        return;
      }
      lastEvent = millis();
      isIdle = false;
    }
    #endif
    
    if(!isIdle) {
      if(lastFastLedStatus != FASTLED_STAT_NONE) {
        setFastLEDStatus(FASTLED_STAT_NONE);
        #if defined(USE_FASTLED_BACKLIGHT)
        setBacklightIndex(smuffConfig.backlightColor);       // turn the backlight back on
        #endif
      }
    }
    else {
      if(smuffConfig.useIdleAnimation && lastFastLedStatus != FASTLED_STAT_MARQUEE) {
        setFastLEDStatus(FASTLED_STAT_MARQUEE);
        #if defined(USE_FASTLED_BACKLIGHT)
        setBacklightIndex(0);                               // turn the backlight off
        #endif
      }
    }

    #if !defined(USE_SERIAL_DISPLAY)
    if ((button == MainButton && isClicked) || (button == WheelButton && isHeld)) {
      showMenu = true;
      char title[] = {"Settings"};
      terminalClear(true);
      showSettingsMenu(title);
      showMenu = false;
      terminalClear();
      debounceButton();
    }
    else if (button == LeftButton) {
      // applies to LeoNerd's display only
      if (isClicked) {
        fncKey1();
      }
      else if (isHeld) {
        fncKey2();
      }
    }
    else if (button == RightButton) {
      if (isClicked) {
        fncKey3();
      }
      else if (isHeld) {
        fncKey4();
      }
    }
    else if (turn != 0) {
      resetAutoClose();
      displayingUserMessage = false;
      showMenu = true;
      terminalClear(true);
      if (turn == -1) {
        showMainMenu();
      }
      else {
        showToolsMenu();
      }
      pwrSaveTime = millis();
      showMenu = false;
      terminalClear();
    }
  }

  if ((millis() - pwrSaveTime) / 1000 >= smuffConfig.powerSaveTimeout && !isPwrSave) {
    setPwrSave(1);
  }
  #endif

#ifdef HAS_TMC_SUPPORT
  monitorTMC(SELECTOR);
  monitorTMC(REVOLVER);
  monitorTMC(FEEDER);
#endif

  if(isTestPending) {
    isTestPending = false;
    showMenu = true;
    testRun(testToRun);
    showMenu = false;
  }
  
  loopEx();
}

/*
* For testing only
*/
void loopEx() {

#if defined(__BRD_SKR_MINI_E3) || defined(__BRD_SKR_MINI_E3DIP)
  #if defined(USE_SW_SERIAL0)         // for testing SoftwareSerial only
    if((__systick % 2000) == 0) {     // every two seconds send string "*TG" to SW-Serial
      swSerial0.println("*TG");
    }
  #endif
  
#endif

}

void fncKey1() {
  if (strlen(smuffConfig.lButtonDown) > 0) {
    parseGcode(String(smuffConfig.lButtonDown), -1);
  }
}

void fncKey2() {
  if (strlen(smuffConfig.lButtonHold) > 0) {
    parseGcode(String(smuffConfig.lButtonHold), -1);
  }
}

void fncKey3() {
  if (strlen(smuffConfig.rButtonDown) > 0) {
    parseGcode(String(smuffConfig.rButtonDown), -1);
  }
}

void fncKey4() {
  if (strlen(smuffConfig.rButtonHold) > 0) {
    parseGcode(String(smuffConfig.rButtonHold), -1);
  }
  else {
    // open / close LID servo by default
    if (lidOpen)
      setServoLid(SERVO_CLOSED);
    else
      setServoLid(SERVO_OPEN);
  }
}

void setPwrSave(int8_t state) {
  setDisplayPowerSave(state);

  isPwrSave = state == 1;
  if (!isPwrSave) {
    pwrSaveTime = millis();
    refreshStatus();
    #if defined(USE_FASTLED_BACKLIGHT)
    setBacklightIndex(smuffConfig.backlightColor);    // turn back on backlight
    #endif
  }
  else {
    #if defined(USE_FASTLED_BACKLIGHT)
    setBacklightIndex(0);                             // turn off backlight
    #endif
  }
}

bool checkUserMessage() {
  #if !defined(USE_SERIAL_DISPLAY)
    bool isClicked = getEncoderButton(true);
    if (displayingUserMessage && (isClicked || (millis() - userMessageTime > USER_MESSAGE_RESET * 1000))) {
      displayingUserMessage = false;
    }
    return displayingUserMessage;
  #else
    return false;
  #endif
}

void checkSerialPending() {
  if (Serial.available()) {
    serialEvent();
    lastEvent = millis();
  }
  if (CAN_USE_SERIAL1) {
    if (Serial1.available()) {
      serialEvent1();
      lastEvent = millis();
    }
  }
  if (CAN_USE_SERIAL2) {
    if (Serial2.available()) {
      serialEvent2();
      lastEvent = millis();
    }
  }
  if (CAN_USE_SERIAL3) {
    if (Serial3.available()) {
      serialEvent3();
      lastEvent = millis();
    }
  }
}

void resetSerialBuffer(int8_t serial) {
  switch (serial) {
  case 0:
    serialBuffer0 = "";
    break;
  case 1:
    serialBuffer1 = "";
    break;
  case 2:
    serialBuffer2 = "";
    break;
  case 3:
    serialBuffer3 = "";
    break;
  }
}

void filterSerialInput(String &buffer, char in) {
  // function key sequence starts with 'ESC['
  if (!isFuncKey && in == 0x1b) {
    isFuncKey = true;
    return;
  }
  if (isFuncKey) {
    //__debugS(D, PSTR("%02x"), in);
    if (in == 'P') { // second escape char 'P' - swallow that, set ignore quotes
      ignoreQuotes = true;
      isFuncKey = false;
      return;
    }
    if (in == 'U') { // second escape char 'U' - swallow that, set isUpload flag
      drawUpload(uploadLen);
      isUpload = true;
      parserBusy = true;
      isFuncKey = false;
      uploadStart = 0;
      return;
    }
    if (in == '[' || in == 'O') // second escape char '[' or 'O' - swallow that
      return;
    isFuncKey = false;
    switch (in) {
      case 0x42:
        remoteKey = REMOTE_UP;
        return; // CursorUp   = turn right
      case 0x41:
        remoteKey = REMOTE_DOWN;
        return; // CursorDown  = turn left
      case 0x43:
        remoteKey = REMOTE_SELECT;
        return;  // CursorRight = wheel click
      case 0x1b: // ESC Key
      case 0x44:
        remoteKey = REMOTE_ESCAPE;
        return; // CursorLeft = main click
      case 0x31:
        remoteKey = REMOTE_HOME;
        return; // Home Key
      case 0x34:
        remoteKey = REMOTE_END;
        return; // End Key  (not used yet)
      case 0x35:
        remoteKey = REMOTE_PGUP;
        return; // PageUp Key (not used yet)
      case 0x36:
        remoteKey = REMOTE_PGDN;
        return; // PageDown Key (not used yet)
      case 0x50:
        remoteKey = REMOTE_PF1;
        return; // F1 Key = fncKey1()
      case 0x51:
        remoteKey = REMOTE_PF2;
        return; // F2 Key = fncKey2()
      case 0x52:
        remoteKey = REMOTE_PF3;
        return; // F3 Key = fncKey3()
      case 0x53:
        remoteKey = REMOTE_PF4;
        return; // F4 Key = fncKey4()
      default:
        return; // ignore any other code not in the list
    }
  }
  isFuncKey = false;
  // special function for Duet3D: if "\n" is transmitted (two characters)
  // then threat that as a line-feed eventually. Otherwise if it's a "\\"
  // store that as a single "\" in the buffer or if it's a "\s" ignore that
  // control string (used in earlier versions of the Duet3D in conjunction with SMuFF-Ifc).
  if (in == '\\') {
    if (isCtlKey) {
      isCtlKey = false;
      if(in != 's')       // ignore a '\s'
        buffer += in;
    }
    else {
      isCtlKey = true;
    }
    return;
  }
  if (in >= 'a' && in <= 'z') {
    if (!isQuote && !ignoreQuotes)
      in = in - 0x20;
  }
  switch (in) {
    case '\b': {
        if(buffer.substring(buffer.length() - 1)=="\"")
          isQuote = !isQuote;
        buffer = buffer.substring(0, buffer.length() - 1);
      }
      break;
    case '\r':
      break;
    case '"':
      isQuote = !isQuote;
      buffer += in;
      break;
    case ' ':
      if (isQuote)
        buffer += in;
      break;
    default:
      if (buffer.length() < 4096) {
        if (in >= 0x21 && in <= 0x7e) // read over non-ascii characters, just in case
          buffer += in;
      }
      else {
        __debugS(W, PSTR("Buffer exceeded 4096 bytes!"));
      }
      break;
  }
}

void sendToDuet3D(char in) {
  switch (smuffConfig.duet3Dport) {
    case 0:
      return;
    case 1:
      if (CAN_USE_SERIAL1)
        Serial1.write(in);
      break;
    case 2:
      if (CAN_USE_SERIAL2)
        Serial2.write(in);
      break;
    case 3:
      if (CAN_USE_SERIAL3)
        Serial3.write(in);
      break;
  }
}

void sendToPanelDue(char in) {
  // only if PanelDue is configured...
  switch (smuffConfig.hasPanelDue) {
    case 0:
      return;
    case 1:
      if (CAN_USE_SERIAL1)
        Serial1.write(in);
      break;
    case 2:
      if (CAN_USE_SERIAL2)
        Serial2.write(in);
      break;
    case 3:
      if (CAN_USE_SERIAL3)
        Serial3.write(in);
      break;
  }
}

bool isJsonData(char in, uint8_t port) {
  // check for JSON formatted data
  if (in == '{') {
    bracketCnt++;
  }
  else if (in == '}') {
    bracketCnt--;
  }
  if (bracketCnt > 0) {
    jsonPtr++;
    //__debugS(D, PSTR("JSON nesting level: %d"), bracketCnt);
  }

  if (jsonPtr > 0) {
    // data not comming from PanelDue port...
    if(port != smuffConfig.hasPanelDue) {
      if(smuffConfig.duet3Dport == 0)
        smuffConfig.duet3Dport = port;
      // send to PanelDue
      sendToPanelDue(in);
    }
    else { // data sent from PanelDue...
      sendToDuet3D(in);
    }
    if (bracketCnt > 0)
      return true;
  }
  if (bracketCnt == 0 && jsonPtr > 0) {
    jsonPtr = 0;
    return true;
  }
  return false;
}

void resetUpload() {
  upload.close();
  isUpload = false;
  gotFirmware = false;
  parserBusy = false;
  uploadStart = 0;
}

void handleUpload(const char* buffer, size_t len, Stream* serial) {
  if(uploadStart > 0 && millis()-uploadStart > 10000) {
    resetUpload();
    __debugS(W, PSTR("Upload aborted because of timeout"), firmware);
    return;
  }
  if (isPwrSave)
    setPwrSave(0);
  drawUpload(uploadLen);
  if(!isUpload)
    return;
  //sendXoff(serial);
  upload.write(buffer, len);
  //upload.flush();
  //sendXon(serial);
  uploadLen -= len;
  uploadStart = millis();

  if(uploadLen <= 0) {
    resetUpload();
    __debugS(I, PSTR("Upload of '%s' finished"), firmware);
  }
}

void handleSerial(const char* in, size_t len, String& buffer, uint8_t port) {
  for(size_t i=0; i< len; i++) {
    // handle Ctrl-C sequence, which will interrupt a running test
    if (in[i] == 0x03) {
      isTestrun = false;
      resetSerialBuffer(port);
      //__debugS(D, PSTR("Ctrl-C received from port %d"), port);
      return;
    }
    // do not proceed any further if a test is running
    if(isTestrun)
      return;
    // parse for JSON data coming from Duet3D
    if(port == 1 || port == 3) {
      if(isJsonData(in[i], port)) {
        __debugS(D, PSTR("JSON data received on port %d"), port);
        continue;
      }
    }
    if (in[i] == '\n' || (isCtlKey && in[i] == 'n')) {
      parseGcode(buffer, port);
      isQuote = false;
      actionOk = false;
      isCtlKey = false;
      ignoreQuotes = false;
    }
    else if(isCtlKey && in[i] == '"') {
      // don't handle quotes if they are escaped
      isCtlKey = false;
      buffer += '\\';
      buffer += in[i];
      return;
    }
    else {
      filterSerialInput(buffer, in[i]);
    }
  }
}

size_t readSerialToBuffer(Stream* serial, char* buffer, size_t maxLen) {
  size_t got = 0;
  int avail = serial->available();
  if(avail != -1) {
    size_t len = avail;
    if((size_t)avail > maxLen)
      len = maxLen;
    got = serial->readBytes(buffer, len);
  }
  return got;
}

void serialEvent() {  // USB-Serial port
  char tmp[256];
  memset(tmp, 0, ArraySize(tmp));
  size_t got = readSerialToBuffer(&Serial, tmp, ArraySize(tmp)-1);
  if(got > 0) {
    if(isUpload)
      handleUpload(tmp, got, &Serial);
    else {
      if(smuffConfig.useDuet) {
        if(smuffConfig.traceUSBTraffic)
          __debugS(I, PSTR("Recv(0): %s"), tmp);
      }
      handleSerial(tmp, got, serialBuffer0, 0);
    }
  }
}

void serialEvent1() {
  char tmp[256];
  memset(tmp, 0, ArraySize(tmp));
  size_t got = readSerialToBuffer(&Serial1, tmp, ArraySize(tmp)-1);
  if(got > 0) {
    if(isUpload)
      handleUpload(tmp, got, &Serial1);
    else
      handleSerial(tmp, got, serialBuffer1, 1);
  }
}

void serialEvent2() {
  char tmp[256];
  memset(tmp, 0, ArraySize(tmp));
  size_t got = readSerialToBuffer(&Serial2, tmp, ArraySize(tmp)-1);
  if(got > 0) {
    if(isUpload)
      handleUpload(tmp, got, &Serial2);
    else {
      if(smuffConfig.useDuet) {
        if(smuffConfig.traceUSBTraffic)
          __debugS(I, PSTR("Recv(2): %s"), tmp);
        }
      handleSerial(tmp, got, serialBuffer2, 2);
    }
  }
}

void serialEvent3() {
  char tmp[256];
  memset(tmp, 0, ArraySize(tmp));
  size_t got = readSerialToBuffer(&Serial3, tmp, ArraySize(tmp)-1);
  if(got > 0) {
    if(isUpload)
      handleUpload(tmp, got, &Serial3);
    else
      handleSerial(tmp, got, serialBuffer3, 3);
  }
}
