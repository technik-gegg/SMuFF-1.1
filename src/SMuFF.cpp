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
#include "Config.h"
#include "InputDialogs.h"

// Please notice: If you make any changes / additions here, you'll have to add
// an "external" declaration in SMuFF.h as well
#ifdef __BRD_I3_MINI
U8G2_ST7565_64128N_F_4W_HW_SPI  display(U8G2_R2, /* cs=*/ DSP_CS_PIN, /* dc=*/ DSP_DC_PIN, /* reset=*/ DSP_RESET_PIN);
#endif

#if defined(__BRD_SKR_MINI) || defined(__BRD_SKR_MINI_E3) || defined(__BRD_SKR_MINI_E3DIP)
  #if defined(USE_TWI_DISPLAY)
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
  #elif defined(USE_LEONERD_DISPLAY)
  U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);
  #elif defined(USE_ANET_DISPLAY)
  //
  //  Attn.: Instructions to modify the ANET display can be found here: https://www.thingiverse.com/thing:4009810
  //
  #pragma error "Before you use this display, you have to make some heavy modifications on the wiring for the display connector! Please check, then comment out this line."
  U8G2_ST7920_128X64_F_2ND_HW_SPI display(U8G2_R0, /* cs=*/ DSP_CS_PIN, /* reset=*/ U8X8_PIN_NONE);
  // if the hardware SPI doesn't work, you may try software SPI instead
  //U8G2_ST7920_128X64_F_SW_SPI display(U8G2_R0, /* clock=*/ DSP_DC_PIN, /* data=*/ DSP_DATA_PIN, /* cs=*/ DSP_CS_PIN, /* reset=*/ U8X8_PIN_NONE);
  #elif defined(USE_MINI12864_PANEL_V21) || defined(USE_MINI12864_PANEL_V20)
  U8G2_ST7567_JLX12864_F_2ND_4W_HW_SPI display(U8G2_R0, /* cs=*/ DSP_CS_PIN, /* dc=*/ DSP_DC_PIN, /* reset=*/ DSP_RESET_PIN);
  #elif defined(USE_CREALITY_DISPLAY)
    // works only with software SPI, hence it's remarkably slower than hardware SPI
    U8G2_ST7920_128X64_F_SW_SPI display(U8G2_R0, /* clock=*/ DSP_DC_PIN, /* data=*/ DSP_DATA_PIN, /* cc=*/ DSP_CS_PIN, /* reset=*/ DSP_RESET_PIN);
  #else
  // Notice: This constructor is feasible for the MKS-MINI12864 V2.0 RepRap display
  U8G2_ST7567_ENH_DG128064_F_2ND_4W_HW_SPI  display(U8G2_R2, /* cs=*/ DSP_CS_PIN, /* dc=*/ DSP_DC_PIN, /* reset=*/ DSP_RESET_PIN);
  //U8G2_UC1701_MINI12864_F_2ND_4W_HW_SPI display(U8G2_R0, /* cs=*/ DSP_CS_PIN, /* dc=*/ DSP_DC_PIN, /* reset=*/ DSP_RESET_PIN);
  #endif

#elif defined(__BRD_ESP32)
  #if defined(USE_TWI_DISPLAY)
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
  #elif defined(USE_LEONERD_DISPLAY)
  U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);
  #else
  // Notice: This constructor is feasible for the MKS-MINI12864 V2.0 RepRap display
  U8G2_ST7567_ENH_DG128064_F_4W_HW_SPI display(U8G2_R2, /* cs=*/ DSP_CS_PIN, /* dc=*/ DSP_DC_PIN, /* reset=*/ DSP_RESET_PIN);
  #endif

#elif defined(__BRD_FYSETC_AIOII)
U8G2_UC1701_MINI12864_F_4W_HW_SPI display(U8G2_R0, /* cs=*/ DSP_CS_PIN, /* dc=*/ DSP_DC_PIN, /* reset=*/ DSP_RESET_PIN);
#endif

#if defined(__AVR__)
Stream*                 debugSerial = &Serial;

#elif defined(__STM32F1__)
  #if defined(__BRD_SKR_MINI_E3) || defined(__BRD_SKR_MINI_E3DIP)
Stream*                 debugSerial = &Serial2;
  #else
Stream*                 debugSerial = &Serial1;
  #endif

#elif defined(__ESP32__)
BluetoothSerial SerialBT;                 // used for debugging or mirroring traffic to PanelDue
#if defined(__DEBUG_BT__)
Stream*                 debugSerial = &SerialBT;  // decide which serial port to use for debug outputs
#else
Stream*                 debugSerial = &Serial;    // decide which serial port to use for debug outputs
#endif
HardwareSerial          Serial3(1);               // dummy declaration to keep the compiler happy,
                                                  // won't be used though because of the CAN_USE_SERIAL3 definition
#endif
Stream*                 logSerial = &Serial;

ZStepper                steppers[NUM_STEPPERS];
ZTimer                  stepperTimer;
ZTimer                  gpTimer;
ZServo                  servo;
ZServo                  servoLid;
ZFan                    fan;
DuetLaserSensor         duetLS;
#if defined(USE_LEONERD_DISPLAY)
LeoNerdEncoder          encoder(I2C_ENCODER_ADDRESS, -1);
#else
ClickEncoder            encoder(ENCODER1_PIN, ENCODER2_PIN, ENCODER_BUTTON_PIN, 4);
#endif
#if defined(USE_SW_SERIAL)
SoftwareSerial          swSer0(SW_SERIAL_RX_PIN, SW_SERIAL_TX_PIN, false);
#endif
#if defined(__STM32F1__)
#if defined(USE_COMPOSITE_SERIAL)
USBMassStorage          MassStorage;
USBCompositeSerial      CompositeSerial;
#endif
#elif defined(__ESP32__)
ZPortExpander           portEx;
#endif

TMC2209Stepper* drivers[NUM_STEPPERS] { nullptr, nullptr, nullptr };

#if defined(MULTISERVO)
Adafruit_PWMServoDriver servoPwm = Adafruit_PWMServoDriver(I2C_SERVOCTL_ADDRESS, Wire);
uint8_t servoMapping[17]         = {   0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, -1 }; // last one is used for the Wiper mapping
uint8_t servoPosClosed[16]       = {  90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90 };
#else
uint8_t servoPosClosed[MAX_TOOLS];
#endif

String                          wirelessHostname = "";
volatile byte                   nextStepperFlag = 0;
volatile byte                   remainingSteppersFlag = 0;
volatile unsigned long          lastEncoderButtonTime = 0;
bool                            testMode = false;
bool                            timerRunning = false;
int8_t                          toolSelections[MAX_TOOLS];
uint32_t                        pwrSaveTime;
volatile bool                   isPwrSave = false;
volatile bool                   showMenu = false;
volatile bool                   lastZEndstopState = false;
static volatile unsigned        generalCounter = 0;
static volatile unsigned        tickCounter = 0;
volatile uint16_t               bracketCnt = 0;
volatile uint16_t               jsonPtr = 0;
volatile bool                   initDone = false;             // enables sending periodical status information to serial ports
volatile bool                   processingSerial0;            // set when GCode is incoming on a serial port
volatile bool                   processingSerial1;
volatile bool                   processingSerial2;
volatile bool                   processingSerial3;
String                          serialBuffer0, serialBuffer1, serialBuffer2, serialBuffer3;
uint8_t                         remoteKey = REMOTE_NONE;
volatile bool                   sdRemoved = false;
uint16_t                        mmsMin = 1;               // minimum moving speed for stepper in mm/s
uint16_t                        mmsMax = 800;             // maximum moving speed for stepper in mm/s
uint16_t                        speedIncrement = 5;       // increment for speeds in menus


static volatile uint16_t        intervalMask;  // bit-mask for interval reached
IntervalHandler intervalHandlers[] = {
  {   10,   10, every10ms },  {   20,   20, every20ms },  {   50,   50, every50ms },
  {  100,  100, every100ms }, {  250,  250, every250ms }, {  500,  500, every500ms },
  { 1000, 1000, every1s },    { 2000, 2000, every2s },    { 5000, 5000, every5s } };

#ifdef __STM32F1__
volatile uint32_t *stepper_reg_X = &((PIN_MAP[X_STEP_PIN].gpio_device)->regs->BSRR);
volatile uint32_t *stepper_reg_Y = &((PIN_MAP[Y_STEP_PIN].gpio_device)->regs->BSRR);
volatile uint32_t *stepper_reg_Z = &((PIN_MAP[Z_STEP_PIN].gpio_device)->regs->BSRR);
uint32_t pinMask_X = BIT(PIN_MAP[X_STEP_PIN].gpio_bit);
uint32_t pinMask_Y = BIT(PIN_MAP[Y_STEP_PIN].gpio_bit);
uint32_t pinMask_Z = BIT(PIN_MAP[Z_STEP_PIN].gpio_bit);
#endif

void overrideStepX() {
#ifdef __STM32F1__
  *stepper_reg_X = pinMask_X;
  if(smuffConfig.stepDelay[SELECTOR] > 0)
    delayMicroseconds(smuffConfig.stepDelay[SELECTOR]);
  *stepper_reg_X = pinMask_X << 16;
#else
  STEP_HIGH_X
  if(smuffConfig.stepDelay[SELECTOR] > 0)
    delayMicroseconds(smuffConfig.stepDelay[SELECTOR]);
  STEP_LOW_X
#endif
}

void overrideStepY() {
#ifdef __STM32F1__
  *stepper_reg_Y = pinMask_Y;
  if(smuffConfig.stepDelay[REVOLVER] > 0)
    delayMicroseconds(smuffConfig.stepDelay[REVOLVER]);
  *stepper_reg_Y = pinMask_Y << 16;
#else
  STEP_HIGH_Y
  if(smuffConfig.stepDelay[REVOLVER] > 0)
    delayMicroseconds(smuffConfig.stepDelay[REVOLVER]);
  STEP_LOW_Y
#endif
}

void overrideStepZ() {
#ifdef __STM32F1__
  *stepper_reg_Z = pinMask_Z;
  if(smuffConfig.stepDelay[FEEDER] > 0)
    delayMicroseconds(smuffConfig.stepDelay[FEEDER]);
  *stepper_reg_Z = pinMask_Z << 16;
#else
  STEP_HIGH_Z
  if(smuffConfig.stepDelay[FEEDER] > 0)
    delayMicroseconds(smuffConfig.stepDelay[FEEDER]);
  STEP_LOW_Z
#endif
}

void endstopEventY() {
  // add your code here it you want to hook into the endstop event for the Revolver
  //__debug(PSTR("Endstop Revolver: %d"), steppers[REVOLVER].getStepPosition());
}

void endstopEventZ() {
  // add your code here it you want to hook into the 1st endstop event for the Feeder
  //__debug(PSTR("Endstop Revolver: %d"), steppers[REVOLVER].getStepPosition());
}

void endstopEventZ2() {
  // add your code here it you want to hook into the 2nd endstop event for the Feeder
  //__debug(PSTR("Z2 hit: %d"), endstop2Hit++);
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
  if(smuffConfig.useDuetLaser) {
    return duetLS.getSwitch();
  }
  return false;
}

/*
  Handles the genaral purpose timer interrupt.
  Fires every 50uS and thus increments the generalCounter
  every millisecond.
  Also serves as a handler dispatcher for servos / encoder.
*/
void isrGPTimerHandler() {
  noInterrupts();
  timerRunning = true;
  tickCounter++;                      // increment tick counter
  if(tickCounter % 20 == 0) {         // each millisecond...
    // decrement all milliseconds counters for periodic functions and
    // set a flag when the timeout has been reached
    for(uint8_t i=0; i< LAST_INTERVAL; i++) {
      intervalHandlers[i].val--;
      if(intervalHandlers[i].val <= 0) {
        intervalMask  |= _BV(i);
        intervalHandlers[i].val = intervalHandlers[i].period;
      }
    }
    #if !defined(USE_LEONERD_DISPLAY)
    if(initDone)
      encoder.service();                // service the rotary encoder
    #endif
    if(smuffConfig.useDuetLaser) {
      duetLS.service();               // service the Duet3D laser Sensor reader
    }
    playSequenceBackgnd();            // handle background playing of a sequence
    FLIPDBG
    isrFanTimerHandler();             // call the fan interrupt routines also every 1ms
  }
  if(tickCounter == 1000)             // reset counter to avoid overrun
    tickCounter = 0;
  #if !defined(MULTISERVO)
  // call the servos interrupt routines also every 50uS
  isrServoTimerHandler();
  #endif
  interrupts();
}

void enumI2cDevices() {
  uint8_t devs[40];   // unlikey, that there are more devices than that on the bus
  const char* name;
  bool encoder = false;
  uint8_t deviceCnt = scanI2CDevices(devs);
  if(deviceCnt > 0) {
    for(uint8_t i=0; i< ArraySize(devs); i++) {
      if(devs[i]==0)
        break;
      switch(devs[i]) {
        case I2C_ENCODER_ADDRESS:     name = PSTR("Encoder"); encoder = true; break;
        case I2C_DISPLAY_ADDRESS:     name = PSTR("Display"); break;
        case I2C_SERVOCTL_ADDRESS:
        case I2C_SERVOBCAST_ADDRESS:  name = PSTR("MultiServo"); break;
        default: name = PSTR("n.a."); break;
      }
      __debug(PSTR("I2C device @ 0x%02x (%s)"), devs[i], name);
    }
  }
  if(!encoder) {
    #if defined(USE_LEONERD_DISPLAY)
    __debug(PSTR("LeoNerd's OLED configured but no encoder was found!"));
    #endif
  }
}

void setup() {

  serialBuffer0.reserve(40);          // initialize serial comm. buffers
  serialBuffer1.reserve(40);
  serialBuffer2.reserve(40);
  serialBuffer3.reserve(40);

  #if defined(__BRD_FYSETC_AIOII) || defined(__BRD_SKR_MINI_E3DIP) || defined(__BRD_SKR_MINI)
  // Disable JTAG for these boards!
  // On the FYSETC AIOII it's because of the display DSP_DC_PIN/DOG_A0 signal (PA15 / JTDI).
  // On the SKR MINI E3-DIP it's because of the buzzer signal (PA15 / JTDI).
  disableDebugPorts();
  #endif

  // Setup a fixed baudrate until the config file was read.
  // This baudrate is the default setting on the ESP32 while
  // booting up, so exceptions thrown can be shown in terminal app
  #if !defined(USE_COMPOSITE_SERIAL)
  Serial.begin(115200);
  #endif
  if(CAN_USE_SERIAL1) Serial1.begin(115200);
  if(CAN_USE_SERIAL2) Serial2.begin(115200);
  if(CAN_USE_SERIAL3) Serial3.begin(115200);

  initUSB();                          // init the USB serial so it's being recognized by the Windows-PC
  #if defined(USE_COMPOSITE_SERIAL)
  CompositeSerial.begin(115200);
  #endif
  initFastLED();                      // init FastLED if configured
  testFastLED();                      // run a test sequence on FastLEDs
  initHwDebug();                      // init hardware debugging

  __debug(PSTR("[ setup start ]"));
  enumI2cDevices();
  setupBuzzer();                      // setup buzzer before reading config
  setupDeviceName();                  // used for SerialBT on ESP32 only
  setupSerialBT();                    // used for debugging on ESP32 only
  setupDisplay();                     // setup display first in order to show error messages if neccessary
  //__debug(PSTR("[ after setupDisplay ]"));
  setupEncoder();                     // setup encoder - only relevant on LeoNerd display
  //__debug(PSTR("[ after setupEncoder ]"));
  if(readConfig()) {                  // read SMUFF.CFG from SD-Card
    readTmcConfig();                  // read TMCDRVR.CFG from SD-Card
    readServoMapping();               // read SERVOMAP.CFG from SD-Card
    readMaterials();                  // read MATERIALS.CFG from SD-Card
  }
  // __debug(PSTR("[ after readConfig ]"));
  setupSerial();                      // setup all components according to the values in SMUFF.CFG
  // __debug(PSTR("[ after setupSerial ]"));
  setupSteppers();
  // __debug(PSTR("[ after setupSteppers ]"));
  setupTimers();
  // __debug(PSTR("[ after setupTimers ]"));
  setupServos();
  // __debug(PSTR("[ after setupServos ]"));
  setupRelay();
  //__debug(PSTR("[ after setupRelay ]"));
  setupTMCDrivers();                  // setup TMC drivers if any were used
  //__debug(PSTR("[ after setupTMCdrivers ]"));
  setupSwSerial0();                   // used only for testing purposes
  setupBacklight();
  setupDuetLaserSensor();             // setup other peripherials
  setupHeaterBed();
  setupFan();
  setupPortExpander();
  setupI2C();
  setupHBridge();
  getStoredData();                    // read EEPROM.DAT from SD-Card; this call must happen after setupSteppers()
  //__debug(PSTR("readSequences start"));
  //uint32_t now = millis();
  //readSequences();
  //__debug(PSTR("readSequences took %d ms"), millis()-now);

  if(smuffConfig.homeAfterFeed) {
    moveHome(REVOLVER, false, false);
  }
  else {
    resetRevolver();
  }
  // __debug(PSTR("DONE reset Revolver"));

  sendStartResponse(0);       // send "start<CR><LF>" to USB serial interface
  if(CAN_USE_SERIAL1)
    sendStartResponse(1);     // send "start<CR><LF>" to all serial interfaces allowed to use
  if(CAN_USE_SERIAL2 && smuffConfig.hasPanelDue != 2)
    sendStartResponse(2);
  if(CAN_USE_SERIAL3 && smuffConfig.hasPanelDue != 3)
    sendStartResponse(3);

  removeFirmwareBin();      // deletes the firmware.bin file to prevent re-flashing on each boot
  initDone = true;          // mark init done; enable periodically sending status, if configured
  startupBeep();            // signal startup has finished
  refreshStatus(true, false);
  pwrSaveTime = millis();   // init value for LCD screen timeout
}

void startStepperInterval() {
  uint16_t minDuration = 65535;
  for(uint8_t i = 0; i < NUM_STEPPERS; i++) {
    if((_BV(i) & remainingSteppersFlag) && steppers[i].getDuration() < minDuration ) {
      minDuration = steppers[i].getDuration();
    }
  }

  nextStepperFlag = 0;
  for(uint8_t i = 0; i < NUM_STEPPERS; i++) {
    if((_BV(i) & remainingSteppersFlag) && steppers[i].getDuration() == minDuration )
      nextStepperFlag |= _BV(i);
  }

  if(remainingSteppersFlag == 0) {
    stepperTimer.stopTimer();
    stepperTimer.setOverflow(65535);
  }
  else {
    stepperTimer.setNextInterruptInterval(minDuration);
  }
}

void isrStepperHandler() {
  stepperTimer.stopTimer();
  uint16_t tmp = stepperTimer.getOverflow();
  stepperTimer.setOverflow(65535);

  for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
    if(!(_BV(i) & remainingSteppersFlag))
      continue;

    if(!(nextStepperFlag & _BV(i))) {
      steppers[i].setDuration(steppers[i].getDuration() - tmp);
      continue;
    }

    steppers[i].handleISR();
    if(steppers[i].getMovementDone()) {
      remainingSteppersFlag &= ~_BV(i);
    }
  }
  //__debug(PSTR("ISR(): %d"), remainingSteppersFlag);
  startStepperInterval();
}

void runNoWait(int8_t index) {
  if(index != -1)
    remainingSteppersFlag |= _BV(index);
  startStepperInterval();
  //__debug(PSTR("Started stepper %d"), index);
}

void runAndWait(int8_t index) {
  runNoWait(index);
  while(remainingSteppersFlag) {
    checkSerialPending(); // not a really nice solution but needed to check serials for "Abort" command in PMMU mode

#if defined(__STM32F1__) // || defined(__ESP32__)
    if((remainingSteppersFlag & _BV(FEEDER)) && !showMenu) {
      //refreshStatus(false, true);
    }
#endif
  }
  //__debug(PSTR("RunAndWait done"));
  //if(index==FEEDER) __debug(PSTR("Fed: %smm"), String(steppers[index].getStepsTakenMM()).c_str());
}

void refreshStatus(bool withLogo, bool feedOnly) {

  if(feedOnly) {
    drawFeed();
  }
  else {
    display.firstPage();
    do {
      if(withLogo)
        drawLogo();
      drawStatus();
      drawSDRemoved(sdRemoved);
    } while(display.nextPage());
  }
}

void reportTMC(uint8_t axis, const char* PROGMEM msg) {
  // for now, only debug message, subject to change in the future
  __debug(PSTR("Driver %c: reports '%s'"), 'X'+axis, msg);
}

void monitorTMC(uint8_t axis) {

  uint16_t temp;
  if(drivers[axis] != nullptr) {
    // has any error occured?
    if(drivers[axis]->drv_err()) {
      // check overtemp. warning
      if(drivers[axis]->otpw()) {
        reportTMC(axis, P_TMC_Status09);
      }
      // check overtemp.
      if(drivers[axis]->ot()) {
        if(drivers[axis]->t157())
          temp = 157;
        if(drivers[axis]->t150())
          temp = 150;
        if(drivers[axis]->t143())
          temp = 143;
        if(drivers[axis]->t120())
          temp = 120;
        char msg[80];
        sprintf_P(msg, PSTR("Overtemp. >= %d°C"), temp);
        reportTMC(axis, msg);
      }
      // check open lines
      if(drivers[axis]->ola()) {
        reportTMC(axis, P_TMC_Status03);
      }
      if(drivers[axis]->olb()) {
        reportTMC(axis, P_TMC_Status04);
      }
      // check short to grounds
      if(drivers[axis]->s2ga()) {
        reportTMC(axis, P_TMC_Status05);
      }
      if(drivers[axis]->s2gb()) {
        reportTMC(axis, P_TMC_Status06);
      }
    }
  }
}

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

#if defined(__ESP32__)
  // call this method only if you have serial ports assigned
  // to the Port Expander
  //portEx.service();

  /* FOR TESTING ONLY */
  // if(interval500ms) { portEx.togglePin(1); interval500ms = false; }
  /*
  if(interval5s) { portEx.testSerial("Testing PortExpander serial...\n"); interval5s = false; }
  while(portEx.serialAvailable(0)) {
    char c = portEx.serialRead(0);
    char cc = (c >= 0x20 && c < 0x7F) ? c : '.';
    __debug(PSTR("Got: %3d - '%c' - 0x%02x"), c, cc, c);
  }
  */
#endif
}

void loop() {
  // Call periodical functions as the timeout has reached.
  // Add your specific code there, if you need to have something
  // managed periodically.
  // The main loop is the better choice for dispatching, since it'll
  // allow uninterrupted serial I/O.
  for(uint8_t i=0; i < LAST_INTERVAL; i++) {
    if(_BV(i) & intervalMask) {
      intervalHandlers[i].func();
      intervalMask &= ~_BV(i);
    }
  }

  #if defined(USE_COMPOSITE_SERIAL)
  MassStorage.loop();
  #endif

  #if defined(SD_DETECT_PIN)
  // message the user if the SD-Card gets removed
  if(digitalRead(SD_DETECT_PIN) == HIGH) {
    if(!sdRemoved) {
      userBeep();
      LeoNerdBlinkRed = true;
    }
    sdRemoved = true;
    char tmp[128];
    sprintf_P(tmp, P_SDCardRemoved);
    drawUserMessage(tmp);
    return;
  }
  else {
    if(sdRemoved) {
      if(initSD(false)) {
        sdRemoved = false;
        refreshStatus(true, false);
        LeoNerdBlinkRed = false;
      }
    }
    else
      sdRemoved = false;
  }
  #endif

  bool state = feederEndstop();
  if(state != lastZEndstopState) {
    lastZEndstopState = state;
    refreshStatus(true, false);
    // for Duet3D only
    setSignalPort(FEEDER_SIGNAL, state);
    delay(200);
    setSignalPort(FEEDER_SIGNAL, !state);
    delay(200);
    setSignalPort(FEEDER_SIGNAL, state);
  }
  checkUserMessage();

  if(!isPwrSave && !showMenu && !displayingUserMessage && ((generalCounter % 250) == 0)) {
    refreshStatus(true, false);   // refresh display every 250ms
  }

  int16_t turn;
  uint8_t button;
  bool isHeld, isClicked;

  if(!showMenu) {
    if(remoteKey == REMOTE_HOME)
      remoteKey = REMOTE_NONE;

    getInput(&turn, &button, &isHeld, &isClicked);
    if(isPwrSave && (isClicked || turn != 0)) {
      setPwrSave(0);
      refreshStatus(true, false);
    }

    if((button == MainButton && isClicked) || (button == WheelButton && isHeld)) {
      showMenu = true;
      char title[] = {"Settings"};
      showSettingsMenu(title);
      showMenu = false;
      debounceButton();
    }
    else if(button == LeftButton) {
      // applies to LeoNerd's display only
      if(isClicked) {
        if(strlen(smuffConfig.lButtonDown)>0) {
          parseGcode(String(smuffConfig.lButtonDown), -1);
        }
      }
      else if(isHeld) {
        if(strlen(smuffConfig.lButtonHold)>0) {
          parseGcode(String(smuffConfig.lButtonHold), -1);
        }
      }
    }
    else if(button == RightButton) {
      if(isClicked) {
        if(strlen(smuffConfig.rButtonDown)>0) {
          parseGcode(String(smuffConfig.rButtonDown), -1);
        }
      }
      else if(isHeld) {
        if(strlen(smuffConfig.rButtonHold)>0) {
          parseGcode(String(smuffConfig.rButtonHold), -1);
        }
        else {
          // open / close LID servo by default
          if(lidOpen)
            setServoLid(SERVO_CLOSED);
          else
            setServoLid(SERVO_OPEN);
        }
      }
    }
    else if(turn != 0) {
      resetAutoClose();
      displayingUserMessage = false;
      showMenu = true;
      if(turn == -1) {
        showMainMenu();
      }
      else {
        showToolsMenu();
      }
      pwrSaveTime = millis();
      showMenu = false;
      refreshStatus(true, false);
    }
  }

  if((millis() - pwrSaveTime)/1000 >= (unsigned long)smuffConfig.powerSaveTimeout && !isPwrSave) {
    //__debug(PSTR("Power save mode after %d seconds (%d)"), (millis() - pwrSaveTime)/1000, smuffConfig.powerSaveTimeout);
    refreshStatus(true, false);
    setPwrSave(1);
  }

  if(smuffConfig.stepperStall[SELECTOR]) {
    monitorTMC(SELECTOR);
  }
  if(smuffConfig.stepperStall[REVOLVER]) {
    monitorTMC(REVOLVER);
  }
  if(smuffConfig.stepperStall[FEEDER]) {
    monitorTMC(FEEDER);
  }
  // For testing only
  // loopEx();
}


void setPwrSave(int8_t state) {
  display.setPowerSave(state);
  isPwrSave = state == 1;
  if(!isPwrSave) {
    delay(1200);
    pwrSaveTime = millis();
  }
}

bool checkUserMessage() {
  bool button = getEncoderButton(false);
  if(button && displayingUserMessage) {
    displayingUserMessage = false;
  }
  //__debug(PSTR("%ld"), (millis()-userMessageTime)/1000);
  if(millis()-userMessageTime > USER_MESSAGE_RESET*1000) {
    displayingUserMessage = false;
  }
  return displayingUserMessage;
}


void checkSerialPending() {
#ifndef __AVR__
  if(Serial.available()) {
    serialEvent();
  }
  if(CAN_USE_SERIAL1) {
    if(Serial1.available())
      serialEvent1();
  }
  if(CAN_USE_SERIAL2) {
    if(Serial2.available()) {
      serialEvent2();
    }
  }
  if(CAN_USE_SERIAL3) {
    if(Serial3.available())
      serialEvent3();
  }
#endif
}

void resetSerialBuffer(int8_t serial) {
  switch(serial) {
    case 0: serialBuffer0 = ""; break;
    case 1: serialBuffer1 = ""; break;
    case 2: serialBuffer2 = ""; break;
    case 3: serialBuffer3 = ""; break;
  }
}

bool isQuote = false;
bool isFuncKey = false;

void filterSerialInput(String& buffer, char in) {
  // function key sequence starts with 'ESC['
  if(!isFuncKey && in == 0x1b) {
    isFuncKey = true;
    return;
  }
  if(isFuncKey) {
    //__debug(PSTR("%02x"), in);
    if(in == '[')  // second escape char '[' - swallow that
      return;
    isFuncKey = false;
    switch(in) {
      case 0x42: remoteKey = REMOTE_UP;     return;   // CursorUp   = turn right
      case 0x41: remoteKey = REMOTE_DOWN;   return;   // CursorDown  = turn left
      case 0x43: remoteKey = REMOTE_SELECT; return;   // CursorRight = wheel click
      case 0x1b:                                      // ESC Key
      case 0x44: remoteKey = REMOTE_ESCAPE; return;   // CursorLeft = main click
      case 0x31: remoteKey = REMOTE_HOME; return;     // Home Key
      case 0x34: remoteKey = REMOTE_END; return;      // End Key  (not used yet)
      case 0x35: remoteKey = REMOTE_PGUP; return;     // PageUp Key (not used yet)
      case 0x36: remoteKey = REMOTE_PGDN; return;     // PageDown Key (not used yet)
      default:  return;                               // ignore any other code not in the list
    }
  }
  isFuncKey = false;
  if(in >= 'a' && in <='z') {
    if(!isQuote)
      in = in - 0x20;
  }
  switch(in) {
    case '\b':
      {
        buffer = buffer.substring(0, buffer.length()-1);
      }
      break;
    case '\r':
      break;
    case '"':
      isQuote = !isQuote;
      buffer += in;
      break;
    case ' ':
      if(isQuote)
        buffer += in;
      break;
    default:
      if(buffer.length() < 4096)
        if(in >=0x21 && in <= 0x7e)   // read over non-ascii characters, just in case
          buffer += in;
      break;
  }
}

void sendToPanelDue(char in) {
    // only if PanelDue is configured...
  switch(smuffConfig.hasPanelDue) {
    case 0: return;
    case 1: if(CAN_USE_SERIAL1) Serial1.write(in); break;
    case 2: if(CAN_USE_SERIAL2) Serial2.write(in); break;
    case 3: if(CAN_USE_SERIAL3) Serial3.write(in); break;
  }
}

bool isJsonData(char in) {
  // check for JSON formatted data
  if(in == '{') {
    bracketCnt++;
  }
  else if(in == '}') {
    bracketCnt--;
  }
  if(bracketCnt > 0) {
    jsonPtr++;
    //__debug(PSTR("JSON nesting level: %d"), bracketCnt);
  }

  if(jsonPtr > 0) {
    sendToPanelDue(in);
    if(bracketCnt > 0)
      return true;
  }
  if(bracketCnt == 0 && jsonPtr > 0) {
    /*
    if(smuffConfig.hasPanelDue) {
      __debug(PSTR("JSON data passed through to PanelDue: %d"), jsonPtr+1);
    }
    else {
      __debug(PSTR("PanelDue not configured. JSON data ignored"));
    }
    */
    jsonPtr = 0;
    return true;
  }
  return false;
}


void serialEvent() {

  uint16_t avail = 0;
  while((avail = Serial.available())) {
    processingSerial0 = true;
    char in = (char)Serial.read();
    // check for JSON data first
    if(isJsonData(in))
      continue;
    if (in == '\n') {
      //__debug(PSTR("Received-0: %s"), serialBuffer0.c_str());
      parseGcode(serialBuffer0, 0);
      isQuote = false;
      actionOk = false;
    }
    else {
      filterSerialInput(serialBuffer0, in);
    }
  }
  processingSerial0 = false;
}

void serialEvent2() {
  uint16_t avail = 0;
  while((avail = Serial2.available())) {
    processingSerial2 = true;
    char in = (char)Serial2.read();
    //__log(PSTR("Avail %d 0x%02x\n"), avail, in);
    // in case of PanelDue connected, route everthing to Duet3D - do not process it any further
    if(smuffConfig.hasPanelDue == 1) {
      if(CAN_USE_SERIAL1) Serial1.write(in);
    }
    else {
      if (in == '\n') {
        //__debug(PSTR("Received-2: %s"), serialBuffer2.c_str());
        parseGcode(serialBuffer2, 2);
        isQuote = false;
        actionOk = false;
      }
      else {
        filterSerialInput(serialBuffer2, in);
      }
    }
  }
  processingSerial2 = false;
}

#ifndef __AVR__
void serialEvent1() {

  uint16_t avail = 0;
  while((avail = Serial1.available())) {
    processingSerial1 = true;
    char in = (char)Serial1.read();
    // check for JSON data first
    if(isJsonData(in))
      continue;
    if (in == '\n') {
      //__debug(PSTR("Received-1: %s"), serialBuffer1.c_str());
      parseGcode(serialBuffer1, 1);
      isQuote = false;
      actionOk = false;
    }
    else {
      filterSerialInput(serialBuffer1, in);
    }
  }
  processingSerial1 = false;

}

void serialEvent3() {

  uint16_t avail = 0;
  while((avail = Serial3.available())) {
    processingSerial3 = true;
    char in = (char)Serial3.read();
    if(smuffConfig.hasPanelDue == 2) {
      if(CAN_USE_SERIAL2) Serial2.write(in);
    }
    else {
      if (in == '\n') {
        //__debug(PSTR("Received-3: %s"), serialBuffe3.c_str());
        parseGcode(serialBuffer3, 3);
        isQuote = false;
        actionOk = false;
      }
      else {
        filterSerialInput(serialBuffer3, in);
      }
    }
  }
  processingSerial3 = false;
}
#endif
