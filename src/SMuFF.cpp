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
#include "SMuFF.h"
#include "InputDialogs.h"

#if !defined(USE_DUET3D)
Stream                    *debugSerial = &Serial2;        // send debug output to...
Stream                    *logSerial = &Serial;           // send logs to USB by default
Stream                    *terminalSerial = &Serial2;     // send terminal emulation output to ...
#else
// Duet3D uses Serial 1 for communication, Serial 3 for PanelDue (optional)
// in this case, all debug outputs are sent to USB serial
Stream                    *debugSerial = &Serial;         // send debug output to USB
Stream                    *logSerial = &Serial;           // send logs to USB by default
Stream                    *terminalSerial = &Serial;      // send terminal emulation output to USB
#endif

#if defined(USE_SW_SERIAL)
SoftwareSerial            swSer0(SW_SERIAL_RX_PIN, SW_SERIAL_TX_PIN, false);  // mainly for testing purpose
#endif

#ifdef DEBUG
bool                      testMode = true;
#else
bool                      testMode = false;
#endif

dspDriver                 display = INIT_DSP();           // initialize display driver (see macro in according Display_xxx.h)
ZStepper                  steppers[NUM_STEPPERS];
ZTimer                    stepperTimer;
ZTimer                    gpTimer;
ZTimer                    fastLEDTimer;
ZTimer                    servoTimer;
ZServo                    servoWiper;
ZServo                    servoLid;
ZServo                    servoCutter;
ZFan                      fan;
DuetLaserSensor           duetLS;

#if defined(USE_LEONERD_DISPLAY)
LeoNerdEncoder            encoder(I2C_ENCODER_ADDRESS, -1);
#else
ClickEncoder              encoder(ENCODER1_PIN, ENCODER2_PIN, ENCODER_BUTTON_PIN, 4);
#endif

#if defined(__STM32F1__) || defined(__STM32F4__)
#if defined(USE_COMPOSITE_SERIAL)
USBMassStorage            MassStorage;
USBCompositeSerial        CompositeSerial;
#endif
#if defined(USE_SPLITTER_ENDSTOPS)
ZEStopMux                 splitterMux;
#endif
#if defined(__LIBMAPLE__)
  #if defined(__STM32F1__)
  volatile uint32_t         *stepper_reg_X = &((PIN_MAP[X_STEP_PIN].gpio_device)->regs->BSRR);
  volatile uint32_t         *stepper_reg_Y = &((PIN_MAP[Y_STEP_PIN].gpio_device)->regs->BSRR);
  volatile uint32_t         *stepper_reg_Z = &((PIN_MAP[Z_STEP_PIN].gpio_device)->regs->BSRR);
  uint32_t                  pinMask_X = BIT(PIN_MAP[X_STEP_PIN].gpio_bit);
  uint32_t                  pinMask_Y = BIT(PIN_MAP[Y_STEP_PIN].gpio_bit);
  uint32_t                  pinMask_Z = BIT(PIN_MAP[Z_STEP_PIN].gpio_bit);
  #else
  volatile uint32_t         *stepper_reg_X = &(digitalPinToPort(X_STEP_PIN)->regs->BSRR);
  volatile uint32_t         *stepper_reg_Y = &(digitalPinToPort(Y_STEP_PIN)->regs->BSRR);
  volatile uint32_t         *stepper_reg_Z = &(digitalPinToPort(Z_STEP_PIN)->regs->BSRR);
  uint32_t                  pinMask_X = digitalPinToBitMask(X_STEP_PIN);
  uint32_t                  pinMask_Y = digitalPinToBitMask(Y_STEP_PIN);
  uint32_t                  pinMask_Z = digitalPinToBitMask(Z_STEP_PIN);
  #endif
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
volatile uint8_t          fastLedHue = 0;
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
static volatile unsigned  generalCounter = 0;
static volatile unsigned  tickCounter = 0;
static volatile unsigned  fastLEDTickCounter = 0;
volatile unsigned long    lastEncoderButtonTime = 0;
bool                      timerRunning = false;
bool                      fastLEDTimerRunning = false;
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
bool                      isUpload = false;
bool                      isReceiving = false;

IntervalHandler intervalHandlers[] = {
    {  10,   10, every10ms}, 
    {  20,   20, every20ms}, 
    {  50,   50, every50ms}, 
    { 100,  100, every100ms}, 
    { 250,  250, every250ms}, 
    { 500,  500, every500ms}, 
    {1000, 1000, every1s}, 
    {2000, 2000, every2s}, 
    {5000, 5000, every5s}
};


void overrideStepX() {
#if defined(__LIBMAPLE__) && (defined(__STM32F1__) || defined(__STM32F4__))
  *stepper_reg_X = pinMask_X;
  if (smuffConfig.stepDelay[SELECTOR] > 0)
    delayMicroseconds(smuffConfig.stepDelay[SELECTOR]);
  *stepper_reg_X = pinMask_X << 16;
#else
  STEP_HIGH_X
  if (smuffConfig.stepDelay[SELECTOR] > 0)
    delayMicroseconds(smuffConfig.stepDelay[SELECTOR]);
  STEP_LOW_X
#endif
}

void overrideStepY() {
#if defined(__LIBMAPLE__) && (defined(__STM32F1__) || defined(__STM32F4__))
  *stepper_reg_Y = pinMask_Y;
  if (smuffConfig.stepDelay[REVOLVER] > 0)
    delayMicroseconds(smuffConfig.stepDelay[REVOLVER]);
  *stepper_reg_Y = pinMask_Y << 16;
#else
  STEP_HIGH_Y
  if (smuffConfig.stepDelay[REVOLVER] > 0)
    delayMicroseconds(smuffConfig.stepDelay[REVOLVER]);
  STEP_LOW_Y
#endif
}

void overrideStepZ() {
#if defined(__LIBMAPLE__) && (defined(__STM32F1__) || defined(__STM32F4__))
  *stepper_reg_Z = pinMask_Z;
  if (smuffConfig.stepDelay[FEEDER] > 0)
    delayMicroseconds(smuffConfig.stepDelay[FEEDER]);
  *stepper_reg_Z = pinMask_Z << 16;
#else
  STEP_HIGH_Z
  if (smuffConfig.stepDelay[FEEDER] > 0)
    delayMicroseconds(smuffConfig.stepDelay[FEEDER]);
  STEP_LOW_Z
#endif
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

/*
  Interrupt handler for StallGuard on TMC2209 (X-Axis/Selector)
 */
void isrStallDetectedX() {
  steppers[SELECTOR].stallDetected();
}

/*
  Interrupt handler for StallGuard on TMC2209 (Y-Axis/Revolver)
 */
void isrStallDetectedY() {
  steppers[REVOLVER].stallDetected();
}

/*
  Interrupt handler for StallGuard on TMC2209 (Z-Axis/Feeder)
 */
void isrStallDetectedZ() {
  steppers[FEEDER].stallDetected();
}

bool checkDuetEndstop() {
  if (smuffConfig.useDuetLaser) {
    return duetLS.getSwitch();
  }
  return false;
}

void isrFastLEDTimerHandler() {
  #if defined(USE_FASTLED_BACKLIGHT) || defined(USE_FASTLED_TOOLS)
  if(!initDone || isUpload)
    return;
  fastLEDTickCounter++;
  if(fastLEDTickCounter % 3 == 0) {   // every 60ms
    fastLedHue++;                     // used for some color changing/fading effects
  }
  if(fastLEDTickCounter % 5 == 0) {  // about every 100ms
    if(fastLedStatus > FASTLED_STAT_NONE) {
      setFastLEDStatus();
    }
    else {
      setFastLEDTools();
    }
    if(servoLid.isPulseComplete())  // refresh FastLED only if the pulse cycle of the Lid servo is completed
      refreshFastLED();             // otherwise it'll interrput the servo ISR and make it jitter
  }
  #endif
}

void isrServoTimerHandler() {
  #if !defined(MULTISERVO)
  noInterrupts();
  // call the servos interrupt routines frequently
  isrServoHandler();
  interrupts();
  #endif
}

/*
  Handles the general purpose timer interrupt.
  Fires every 50uS and thus increments the general counter
  every millisecond.
  Also serves as a handler dispatcher for servos / encoder.
*/
void isrGPTimerHandler() {
  noInterrupts();
  timerRunning = true;
  tickCounter++; // increment tick counter
  if (tickCounter % 20 == 0) {
    // each millisecond...
    // decrement all milliseconds counters for periodic functions and
    // set a flag when the timeout has been reached
    for (uint8_t i = 0; i < LAST_INTERVAL; i++) {
      intervalHandlers[i].val--;
      if (intervalHandlers[i].val <= 0) {
        intervalMask |= _BV(i);
        intervalHandlers[i].val = intervalHandlers[i].period;
      }
    }
    generalCounter++;
    if (initDone) {
      #if !defined(USE_LEONERD_DISPLAY)
      encoder.service();    // service the rotary encoder
      #endif
    }
    isrFanTimerHandler();   // call the fan interrupt routines also every 50uS
  }
  #ifdef FLIPDBG
  uint16_t dbgCnt = (uint16_t)((float)(1/0.00005)/(smuffConfig.dbgFreq*2));
  if (initDone && tickCounter % dbgCnt == 0) {
    FLIPDBG               // flips the debug pin polarity which is supposed to generate 
                          // a periodical signal (default: 500 Hz) for debug purposes
  }
  #endif

  timerRunning = false;
  interrupts();
}

void setup() {
  serialBuffer0.reserve(30); // initialize serial comm. buffers
  serialBuffer1.reserve(30);
  serialBuffer2.reserve(30);
  serialBuffer3.reserve(30);

// Setup a fixed baudrate until the config file was read.
// This baudrate is the default setting on the ESP32 while
// booting up, so exceptions thrown can be shown in terminal app
#if !defined(USE_COMPOSITE_SERIAL)
  Serial.begin(115200);
#endif
  if (CAN_USE_SERIAL1)
    Serial1.begin(115200);
  if (CAN_USE_SERIAL2)
    Serial2.begin(115200);
  if (CAN_USE_SERIAL3)
    Serial3.begin(115200);

  __debugS(PSTR("[ setup start ]"));

#if defined(__BRD_FYSETC_AIOII) || defined(__BRD_SKR_MINI_E3DIP) || defined(__BRD_SKR_MINI) || defined(__BRD_SKR_MINI_E3)
  // Disable JTAG for these boards!
  // On the FYSETC AIOII it's because of the display DSP_DC_PIN/DOG_A0 signal (PA15 / JTDI).
  // On the SKR MINI E3-DIP it's because of the buzzer signal (PA15 / JTDI).
  // On the SKR MINI E3 V2.0 it's because of the SCL signal (PA15 / JTDI).
  #if defined(__LIBMAPLE__)
  disableDebugPorts();
  #else
  pinF1_DisconnectDebug(PA_15); // disable Serial wire JTAG configuration
  #endif
  __debugS(PSTR("[ debug ports disabled ]"));
  #if defined(__BRD_FYSETC_AIOII) && !defined(USE_TWI_DISPLAY) && defined(STM32_REMAP_SPI)
    afio_remap(AFIO_REMAP_SPI1); // remap SPI3 to SPI1 if a "normal" display is being used
    __debugS(PSTR("[ SPI3 remapped to SPI1 ]"));
  #endif
#endif

  initUSB();                // init the USB serial so it's being recognized by the Windows-PC
#if defined(USE_COMPOSITE_SERIAL)
  CompositeSerial.begin(115200);
#endif
  initFastLED();            // init FastLED if configured
  initHwDebug();            // init hardware debugging

#if defined(USE_I2C)
  // don't scan I2C if no I2C devices are being used; Scan will take ages
  enumI2cDevices(1);
  __debugS(PSTR("[ after enumI2cDevices (I2C1) ]"));
#endif
#if defined(USE_SPLITTER_ENDSTOPS)
  enumI2cDevices(2);
  __debugS(PSTR("[ after enumI2cDevices (I2C2) ]"));
#endif
//#endif
  setupDisplay();           // setup display first in order to show error messages if neccessary
  __debugS(PSTR("[ after setupDisplay ]"));
  setupEncoder();           // setup encoder - only relevant on LeoNerd display
  __debugS(PSTR("[ after setupEncoder ]"));
  __debugS(PSTR("[ getting configs... ]"));
  if (readConfig())
  {                         // read SMUFF.json from SD-Card
    readTmcConfig();        // read TMCDRVR.json from SD-Card
    readServoMapping();     // read SERVOMAP.json from SD-Card
    readMaterials();        // read MATERIALS.json from SD-Card
    #if defined(SMUFF_V6S)
    readRevolverMapping();  // read REVOLVERMAPS.json from SD-Card
    #endif
  }
  setContrast(smuffConfig.lcdContrast); // reset contrast after reading config
  __debugS(PSTR("[ after readConfig ]"));
  testFastLED(false);       // run a test sequence on backlight FastLEDs
  testFastLED(true);        // run a test sequence on tools FastLEDs
  setupSerial();            // setup all serial ports
  __debugS(PSTR("[ after setupSerial ]"));
  setupSteppers();
  __debugS(PSTR("[ after setupSteppers ]"));
  setupTimers();
  __debugS(PSTR("[ after setupTimers ]"));
  setupServos();
  __debugS(PSTR("[ after setupServos ]"));
  setupRelay();
  __debugS(PSTR("[ after setupRelay ]"));
#ifdef HAS_TMC_SUPPORT
  setupTMCDrivers();        // setup TMC drivers if any are used
  __debugS(PSTR("[ after setupTMCdrivers ]"));
#endif
  setupSwSerial0();         // used only for testing purposes
  setupBacklight();
  setupDuetSignals();       // setup Duet3D signal pins
  setupDuetLaserSensor();   // setup other peripherials
  setupFan();
  setupI2C();
  getStoredData();          // read EEPROM.json from SD-Card; this call must happen after setupSteppers()
  #if defined(USE_SPLITTER_ENDSTOPS)
  setupEStopMux();
  __debugS(PSTR("[ after setupEStopMux ]"));
  #endif
  uint32_t now = millis();
  readSequences();          // read sound files from SD-Card sounds folder
  __debugS(PSTR("[ after readSequences ] (loading took %ld ms)"), millis()-now);

  if (smuffConfig.homeAfterFeed)
    moveHome(REVOLVER, false, false);
  else
    resetRevolver();
  //__debugS(PSTR("[ DONE reset Revolver ]"));

  sendStartResponse(0);     // send "start<CR><LF>" to USB serial interface
  if (CAN_USE_SERIAL1)
    sendStartResponse(1);   // send "start<CR><LF>" to all serial interfaces allowed to use
  if (CAN_USE_SERIAL2 && smuffConfig.hasPanelDue != 2)
    sendStartResponse(2);
  if (CAN_USE_SERIAL3 && smuffConfig.hasPanelDue != 3)
    sendStartResponse(3);

  removeFirmwareBin();      // deletes the firmware.bin file to prevent re-flashing on each boot
  refreshStatus(true, false);
  
  #if defined(__BRD_SKR_MINI_E3)  // applies only to E3 V1.2 and 2.0 with integrated stepper drivers
  if(drivers[SELECTOR] == nullptr || drivers[FEEDER] == nullptr) {
    __debugS(PSTR("\nWARNING: Your controller is equipped with onboard TMC stepper drivers"));
    __debugS(PSTR("but at least one of them is not being initialized for UART mode and"));
    __debugS(PSTR("thus, your stepper motor may overheat!"));
    __debugS(PSTR("Please check and change your configuration accordingly!"));
  }
  #endif
  __debugS(PSTR("[ startup tune ]"));
  startupBeep();              // signal startup has finished
  pwrSaveTime = millis();     // init value for LCD screen timeout
  __debugS(PSTR("[ setup end ]"));
  initDone = true;            // mark init done; enable periodically sending status, if configured
}

void startStepperInterval() {
  timerVal_t minDuration = 65535;
  for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
    if ((_BV(i) & remainingSteppersFlag) && steppers[i].getDuration() < minDuration)
      minDuration = steppers[i].getDuration();
  }

  nextStepperFlag = 0;
  for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
    if ((_BV(i) & remainingSteppersFlag) && steppers[i].getDuration() == minDuration)
      nextStepperFlag |= _BV(i);
  }

  if (remainingSteppersFlag == 0) {
    stepperTimer.stop();
    stepperTimer.setOverflow(65535);
  }
  else {
    stepperTimer.setNextInterruptInterval(minDuration);
  }
}

void isrStepperHandler() {
  stepperTimer.stop();
  timerVal_t tmp = stepperTimer.getOverflow();
  stepperTimer.setOverflow(65535);

  for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
    if (!(_BV(i) & remainingSteppersFlag))
      continue;

    if (!(nextStepperFlag & _BV(i))) {
      steppers[i].setDuration(steppers[i].getDuration() - tmp);
      continue;
    }

    steppers[i].handleISR();
    if (steppers[i].getMovementDone()) {
      remainingSteppersFlag &= ~_BV(i);
    }
  }
  startStepperInterval();
}

void runNoWait(int8_t index) {
  if (index != -1)
    remainingSteppersFlag |= _BV(index);
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

bool refreshingDisplay = false;

void refreshStatus(bool withLogo, bool feedOnly) {
  if(refreshingDisplay)
    return;
  if (initDone && !isPwrSave && !showMenu && !displayingUserMessage && !isTestrun && !isUpload) {
    refreshingDisplay = true;
    if (feedOnly) {
      drawFeed();
    }
    else {
      display.clearBuffer();
      drawStatus();
      display.updateDisplay();
      // note to myself: if it's hanging here, there's something wrong with Timer 4 CH1 (STM32)
    }
    refreshingDisplay = false;
  }
}

#ifdef HAS_TMC_SUPPORT
void reportTMC(uint8_t axis, const char *PROGMEM msg) {
  // for now, only debug message (main screen will show "!" next to "TMC")
  __debugS(PSTR("Driver %c: reports '%s'"), axis==FEEDER2 ? 'E' : 'X' + axis, msg);
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

/*
* For testing only
*/
void loopEx() {

#if defined(__BRD_SKR_MINI_E3) || defined(__BRD_SKR_MINI_E3DIP)
  /*
  if(interval1s) {
    #if defined(USE_SW_SERIAL)
    // for testing SoftwareSerial
    swSer0.write('*');
    swSer0.write('T');
    swSer0.write('G');
    #endif
    interval1s = false;
  }
  */
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

void loop() {

  // Call periodical functions as the timeout has triggered.
  // Add your specific code there, if you need to have something
  // managed periodically.
  // The main loop is the better choice for dispatching, since it'll
  // allow uninterrupted serial I/O.
  for (uint8_t i = 0; i < LAST_INTERVAL; i++) {
    if (_BV(i) & intervalMask) {
      intervalHandlers[i].func();
      intervalMask &= ~_BV(i);
    }
  }

#if defined(USE_COMPOSITE_SERIAL)
  MassStorage.loop();
#endif

#if defined(SD_DETECT_PIN)
  // message the user if the SD-Card gets removed
  if (digitalRead(SD_DETECT_PIN) == HIGH) {
    if (!sdRemoved) {
      userBeep();
      leoNerdBlinkRed = true;
      setFastLEDStatus(FASTLED_STAT_ERROR);
      if (isPwrSave) {
        setPwrSave(0);
      }
    }
    isIdle = false;
    sdRemoved = true;
    if((generalCounter % 500) == 0) {
      char tmp[50];
      sprintf_P(tmp, P_SDCardRemoved);
      drawUserMessage(tmp);
    }
    return;
  }
  else
  {
    if (sdRemoved) {
      if (initSD(false)) {
        sdRemoved = false;
        leoNerdBlinkRed = false;
        setFastLEDStatus(FASTLED_STAT_NONE);
        lastEvent = millis();
        refreshStatus(true, false);
      }
    }
    else
      sdRemoved = false;
  }
#endif

  bool state = feederEndstop();
  if (state != lastZEndstopState) {
    refreshStatus(true, false);
    lastZEndstopState = state;
    // for Duet3D only
    setSignalPort(FEEDER_SIGNAL, state);
    delay(20);
    setSignalPort(FEEDER_SIGNAL, !state);
    delay(20);
    setSignalPort(FEEDER_SIGNAL, state);
  }
  if(checkUserMessage()) {
    pwrSaveTime = millis();
  }

  isIdle = ((millis() - lastEvent) / 1000 >= (unsigned long)smuffConfig.powerSaveTimeout);

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
    if(!isIdle) {
      if(lastFastLedStatus != FASTLED_STAT_NONE) {
        setFastLEDStatus(FASTLED_STAT_NONE);
        #if defined(USE_FASTLED_BACKLIGHT)
        setBacklightIndex(smuffConfig.backlightColor);       // turn back on the backlight
        #endif
      }
    }
    else {
      if(smuffConfig.useIdleAnimation && lastFastLedStatus != FASTLED_STAT_MARQUEE) {
        setFastLEDStatus(FASTLED_STAT_MARQUEE);
        #if defined(USE_FASTLED_BACKLIGHT)
        setBacklightIndex(0);       // turn off the backlight
        #endif
      }
    }

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

  if ((millis() - pwrSaveTime) / 1000 >= (unsigned long)smuffConfig.powerSaveTimeout && !isPwrSave) {
    setPwrSave(1);
  }

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
  // For testing purpose only
  loopEx();
}

void setPwrSave(int8_t state) {
  display.setPowerSave(state);

  isPwrSave = state == 1;
  if (!isPwrSave) {
    pwrSaveTime = millis();
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
  bool isClicked;

  isClicked = getEncoderButton(true);
  if (displayingUserMessage && (isClicked || (millis() - userMessageTime > USER_MESSAGE_RESET * 1000))) {
    displayingUserMessage = false;
  }
  return displayingUserMessage;
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
    //__debugS(PSTR("%02x"), in);
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
  // control string (used in earlier versions of the Duet3D in conjunction to SMuFF-Ifc).
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
        __debugS(PSTR("Buffer exceeded 4096 bytes!"));
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
    //__debugS(PSTR("JSON nesting level: %d"), bracketCnt);
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
    __debugS(PSTR("Upload aborted because of timeout"), firmware);
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
    __debugS(PSTR("Upload of '%s' finished"), firmware);
    return;
  }
}

void handleSerial(const char* in, size_t len, String& buffer, uint8_t port) {
  for(size_t i=0; i< len; i++) {
    // handle Ctrl-C sequence, which will interrupt a running test
    if (in[i] == 0x03) {
      isTestrun = false;
      resetSerialBuffer(port);
      //__debugS(PSTR("Ctrl-C received from port %d"), port);
      return;
    }
    // do not proceed any further if a test is running
    if(isTestrun)
      return;
    // parse for JSON data coming from Duet3D
    if(port == 1 || port == 3) {
      if(isJsonData(in[i], port)) {
        __debugS(PSTR("JSON data received on port %d"), port);
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
    noInterrupts();
    size_t len = avail;
    if((size_t)avail > maxLen)
      len = maxLen;
    got = serial->readBytes(buffer, len);
    interrupts();
  }
  return got;
}

void serialEvent() {  // USB-Serial port
  char tmp[128];
  memset(tmp, 0, ArraySize(tmp));
  size_t got = readSerialToBuffer(&Serial, tmp, ArraySize(tmp)-1);
  if(got > 0) {
    if(isUpload)
      handleUpload(tmp, got, &Serial);
    else {
      if(smuffConfig.traceUSBTraffic)
        __debugS(PSTR("Recv(0): %s"), tmp);
      handleSerial(tmp, got, serialBuffer0, 0);
    }
  }
}

void serialEvent1() {
  char tmp[128];
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
  char tmp[128];
  memset(tmp, 0, ArraySize(tmp));
  size_t got = readSerialToBuffer(&Serial2, tmp, ArraySize(tmp)-1);
  if(got > 0) {
    if(isUpload)
      handleUpload(tmp, got, &Serial2);
    else
      handleSerial(tmp, got, serialBuffer2, 2);
  }
}

void serialEvent3() {
  char tmp[128];
  memset(tmp, 0, ArraySize(tmp));
  size_t got = readSerialToBuffer(&Serial3, tmp, ArraySize(tmp)-1);
  if(got > 0) {
    if(isUpload)
      handleUpload(tmp, got, &Serial3);
    else
      handleSerial(tmp, got, serialBuffer3, 3);
  }
}
