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
#include "ZTimerLib.h"
#include "ZStepperLib.h"
#include "ZServo.h"
#include "ZPortExpander.h"
#include "DuetLaserSensor.h"

#ifdef __BRD_I3_MINI
U8G2_ST7565_64128N_F_4W_HW_SPI  display(U8G2_R2, /* cs=*/ DSP_CS_PIN, /* dc=*/ DSP_DC_PIN, /* reset=*/ DSP_RESET_PIN);
#endif

#ifdef __BRD_SKR_MINI
  #ifdef USE_TWI_DISPLAY
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); 
  #elif USE_ANET_DISPLAY
  /*
    Attn.: In order to use this display you have to make some heavy modifications on the wiring
    for the display connector!
    Instructions can be found here: https://www.thingiverse.com/thing:4009810
  */
  U8G2_ST7920_128X64_F_2ND_HW_SPI display(U8G2_R0, /* cs=*/ DSP_CS_PIN, /* reset=*/ U8X8_PIN_NONE); 
  // if the hardware SPI doesn't work, you may try software SPI instead
  //U8G2_ST7920_128X64_F_SW_SPI display(U8G2_R0, /* clock=*/ DSP_DC, /* data=*/ DSP_DATA, /* cs=*/ DSP_CS, /* reset=*/ U8X8_PIN_NONE);
  #elif USE_MINI12864_PANEL_V21
  U8G2_ST7567_JLX12864_F_2ND_4W_HW_SPI display(U8G2_R0, /* cs=*/ DSP_CS_PIN, /* dc=*/ DSP_DC_PIN, /* reset=*/ DSP_RESET_PIN);
  #else
  // Notice: This constructor is feasible for the MKS-MINI12864 V2.0 RepRap display
  U8G2_ST7567_ENH_DG128064_F_2ND_4W_HW_SPI  display(U8G2_R2, /* cs=*/ DSP_CS_PIN, /* dc=*/ DSP_DC_PIN, /* reset=*/ DSP_RESET_PIN);
  //U8G2_UC1701_MINI12864_F_2ND_4W_HW_SPI display(U8G2_R0, /* cs=*/ DSP_CS_PIN, /* dc=*/ DSP_DC_PIN, /* reset=*/ DSP_RESET_PIN);
  #endif
#endif

#ifdef __BRD_ESP32
  #ifdef USE_TWI_DISPLAY
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C  display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
  #else
  // Notice: This constructor is feasible for the MKS-MINI12864 V2.0 RepRap display
  U8G2_ST7567_ENH_DG128064_F_4W_HW_SPI display(U8G2_R2, /* cs=*/ DSP_CS_PIN, /* dc=*/ DSP_DC_PIN, /* reset=*/ DSP_RESET_PIN);
  #endif
#endif

#ifdef __BRD_FYSETC_AIOII
U8G2_UC1701_MINI12864_F_4W_HW_SPI display(U8G2_R0, /* cs=*/ DSP_CS_PIN, /* dc=*/ DSP_DC_PIN, /* reset=*/ DSP_RESET_PIN);
#endif

#if defined(__ESP32__)
BluetoothSerial SerialBT;                 // used for debugging or mirroring traffic to PanelDue 
#if defined(__DEBUG_BT__)
Stream*                 debugSerial = &SerialBT;   // decide which serial port to use for debug outputs 
#else
Stream*                 debugSerial = &Serial;   // decide which serial port to use for debug outputs 
#endif
#elif defined(__STM32F1__)
Stream*                 debugSerial = &Serial1;   
#else
Stream*                 debugSerial = &Serial;   
#endif
ZStepper                steppers[NUM_STEPPERS];
ZTimer                  stepperTimer;
ZTimer                  encoderTimer;
ZServo                  servo;
ZServo                  servoRevolver;
#if defined(__ESP32__)
ZPortExpander           portEx;
#endif
DuetLaserSensor         duetLS;
ClickEncoder            encoder(ENCODER1_PIN, ENCODER2_PIN, ENCODER_BUTTON_PIN, 4);

volatile byte           nextStepperFlag = 0;
volatile byte           remainingSteppersFlag = 0;
volatile unsigned long  lastEncoderButtonTime = 0;
bool                    testMode = false;
int                     toolSelections[MAX_TOOLS]; 
volatile unsigned long  pwrSaveTime;
volatile bool           isPwrSave = false;
volatile bool           showMenu = false; 
volatile bool           lastZEndstopState = false;
volatile bool           lastZEndstop2State = false;
unsigned long           endstopZ2HitCnt = 0;
static unsigned long    lastDisplayRefresh = 0;
volatile unsigned long  generalCounter = 0;
volatile int            bracketCnt = 0;
volatile int            jsonPtr = 0;
//char                    jsonData[MAX_JSON];         // temporary buffer for json data coming from Duet3D (not used yet)
volatile bool           enablePS = false;

String serialBuffer0, serialBuffer2, serialBuffer9; 
String traceSerial2;
String _hostname = "";

extern int  swapTools[MAX_TOOLS];
extern void setToneTimerChannel(uint8_t ntimer, uint8_t channel);    // defined in tone library


void isrStepperHandler();       // forward declarations ... makes every compiler happy
void isrEncoderHandler();
void refreshStatus(bool withLogo);


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
  if(smuffConfig.stepDelay_X > 0)
    delayMicroseconds(smuffConfig.stepDelay_X);
  *stepper_reg_X = pinMask_X << 16;
#else
  STEP_HIGH_X
  // if(smuffConfig.delayBetweenPulses) __asm__ volatile ("nop");
  STEP_LOW_X
#endif
}

void overrideStepY() {
#ifdef __STM32F1__
  *stepper_reg_Y = pinMask_Y;
  if(smuffConfig.stepDelay_Y > 0)
    delayMicroseconds(smuffConfig.stepDelay_Y);
  *stepper_reg_Y = pinMask_Y << 16;
#else
  STEP_HIGH_Y
  // if(smuffConfig.delayBetweenPulses) __asm__ volatile ("nop");
  STEP_LOW_Y
#endif
}

void overrideStepZ() {
#ifdef __STM32F1__
  *stepper_reg_Z = pinMask_Z;
  if(smuffConfig.stepDelay_Z > 0)
    delayMicroseconds(smuffConfig.stepDelay_Z);
  *stepper_reg_Z = pinMask_Z << 16;
#else
  STEP_HIGH_Z
  // if(smuffConfig.delayBetweenPulses) __asm__ volatile ("nop");
  STEP_LOW_Z
#endif
}

void endstopYevent() {
  //__debug(PSTR("Endstop Revolver: %d"), steppers[REVOLVER].getStepPosition());
}

void endstopZevent() {
  //__debug(PSTR("Endstop Revolver: %d"), steppers[REVOLVER].getStepPosition());
}

void endstopZ2event() {
}

void duetLSHandler() {
  duetLS.service();
}

volatile boolean  interval20ms; 
void every20ms() {
  interval20ms = true;
  // do the servos interrupt routines so we save one timer 
  if(!servo.hasTimer())
    servo.setServo();
  if(!servoRevolver.hasTimer())
    servoRevolver.setServo();
}

volatile boolean  interval100ms; 
void every100ms() {
  interval100ms = true;
  // Add your periodical code here 
}

volatile boolean  interval250ms; 
void every250ms() {
  interval250ms = true;
  // Add your periodical code here
}

volatile boolean  interval500ms; 
void every500ms() {
  interval500ms = true;
  // Add your periodical code here 
}

volatile unsigned long lastTick;
volatile unsigned gcInterval;
volatile boolean  interval1s; 
void every1s() {
    interval1s = true;
    unsigned long tmp = millis();
    gcInterval = tmp-lastTick;
    lastTick = tmp;
  // Add your periodical code here 
#if defined(__STM32F1__)
    // __debug("%d ms", gcInterval);
#endif
}

void every2s() {
  // send status of endstops and current tool to all listeners, if configured
  if(!sendingResponse && smuffConfig.sendPeriodicalStats && enablePS && !parserBusy) {
    printPeriodicalState(0);
    printPeriodicalState(1);
    printPeriodicalState(2);
  }
}

volatile boolean  interval5s; 
void every5s() {
  interval5s = true;
  // Add your periodical code here 
}

/*
  Handles the timer interrupt for the rotary encoder.
  Also serves as a general purpose timer dispatcher since the 
  interrupt occures every millisecond.
*/
void isrEncoderHandler() {
  encoder.service();
  generalCounter++;
  if(generalCounter % 20 == 0) { // every 20 ms
    every20ms();
  }
  if(generalCounter % 100 == 0) { // every 100 ms
    every100ms();
  }
  if(generalCounter % 250 == 0) { // every 250 ms
    every250ms();
  }
  if(generalCounter % 500 == 0) { // every 500 ms
    every500ms();
  }
  if(generalCounter % 1000 == 0) { // every 1000 ms
    every1s();
  }
  if(generalCounter % 2000 == 0) { // every 2000 ms
    every2s();
  }
  if(generalCounter % 5000 == 0) { // every 5000 ms
    every5s();
  }
  //duetLSHandler();
}

bool checkDuetEndstop() {
  if(smuffConfig.useDuetLaser) {
    return duetLS.getSwitch();
  }
  return false;
}

void blinkLED() {
#if defined(LED_PIN)
  if(LED_PIN != -1)
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
#endif
}

void setupDeviceName() {
  String appendix = "";
#if defined(__ESP32__)
  appendix = WiFi.macAddress().substring(9);
  appendix.replace(":", "");
#endif
  _hostname = String("SMuFF") + "_" + appendix;
}

/*
  Initialize FastLED library for NeoPixels.
  Primarily used for backlight on some displays (i.e. FYSETC 12864 Mini V2.1)
*/
void initFastLED() {
  #if NEOPIXEL_PIN != -1
    FastLED.addLeds<LED_TYPE, NEOPIXEL_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);
  #endif
}

void setup() {

#ifdef __STM32F1__
  #ifndef USE_TWI_DISPLAY
    #ifdef STM32_REMAP_SPI
    afio_remap(AFIO_REMAP_SPI1);  // remap SPI3 to SPI1 if a "normal" display is being used
    #endif

  #warning("**** Don't forget to short Z+ input to GND for programming the device! ****")
  if(DEBUG_OFF_PIN != -1) {
    pinMode(DEBUG_OFF_PIN, INPUT_PULLUP);
    /* ============ WARNING ================
      If debug ports (JTAG) are disabled, you won't 
      be able to program the device via STLink anymore!
      Therefore, before programming the device,
      set Z+ input to ground and reset the device
      to reenable debug ports.
      =====================================*/
    if(digitalRead(DEBUG_OFF_PIN)==HIGH)
      disableDebugPorts();      // disable JTAG if Z+ is unconnected
  }
  #endif
#endif

  delay(500);
  
  serialBuffer0.reserve(40);
  serialBuffer2.reserve(40);
  traceSerial2.reserve(40);
  
  initFastLED();
  testFastLED();

#if defined(USE_RGB_BACKLIGHT) || defined(USE_FASTLED_BACKLIGHT)
  setBacklightIndex(7); // set backlight color to white by default - gets overwritten after config has been read
#endif

  setupDeviceName();

#ifdef __ESP32__
  // this line must be kept, otherwise BT power down will cause a reset
  SerialBT.begin(_hostname);
  if(BEEPER_PIN != -1) {
    ledcSetup(BEEPER_CHANNEL, 5000, 8);
    ledcAttachPin(BEEPER_PIN, BEEPER_CHANNEL);
  }
#endif

  Serial.begin(115200);        // set fixed baudrate until config file was read
#if defined(__STM32F1__)
  Serial1.begin(115200);       
  Serial2.begin(115200);
  Serial3.begin(115200);
#else       
  Serial2.begin(115200);
#endif
  //__debug(PSTR("[ setup() ]"));

  setupDisplay(); 
  readConfig();
// turn on the LCD backlight accroding to the configured setting
#if defined(USE_RGB_BACKLIGHT) || defined(USE_FASTLED_BACKLIGHT)
  setBacklightIndex(smuffConfig.backlightColor);
#endif


  // special case: 
  // if the baudrate is set to 0, the board is running out of working memory
  if(smuffConfig.serial1Baudrate != 0) { 
    if(smuffConfig.serial1Baudrate != 115200) {
      Serial.end();
      Serial.begin(smuffConfig.serial1Baudrate);
    }
  }
  else {
    writeConfig(&Serial);
    longBeep(3);
    showDialog(P_TitleConfigError, P_ConfigFail1, P_ConfigFail4, P_OkButtonOnly);
  }

#ifndef __AVR__
  Serial1.begin(smuffConfig.serial2Baudrate);
  Serial2.begin(smuffConfig.serial2Baudrate);
  #ifndef __ESP32__
  Serial3.begin(smuffConfig.serial2Baudrate);
  #endif
#else
#endif
  //__debug(PSTR("DONE init SERIAL"));

  setupSteppers();
  setupTimers();
  
  if(SERVO1_PIN != -1) {
    servo.setMaxCycles(smuffConfig.servoCycles1);
    servo.setPulseWidthMinMax(smuffConfig.servoMinPwm, smuffConfig.servoMaxPwm);
    #if defined(__ESP32__)
      // we'll be using the internal ledcWrite for servo control on ESP32
      servo.attach(SERVO1_PIN, false, 0);
    #elif defined(__STM32F1__)
      servo.attach(SERVO1_PIN, true, 0);
    #else
      servo.attach(SERVO1_PIN, true, 0);
    #endif
    setServoPos(0, 90);
  }
  
  // Replace the Revolver stepper motor with a servo motor
  if(SERVO2_PIN != -1) {
    servoRevolver.setMaxCycles(smuffConfig.servoCycles2);
    servoRevolver.setPulseWidthMinMax(smuffConfig.servoMinPwm, smuffConfig.servoMaxPwm);
    #if defined(__ESP32__)
      // we'll be using the internal ledcWrite for servo control on ESP32
      servoRevolver.attach(SERVO2_PIN, false, 1);
    #elif defined(__STM32F1__)
      servoRevolver.attach(SERVO2_PIN, true, 1);
    #else
      servoRevolver.attach(SERVO2_PIN, true, 1);  
    #endif
    setServoPos(1, smuffConfig.revolverOffPos);
  }
  // this call must happen after setupSteppers()
  getStoredData();

  // Duet Laser Sensor is not being used yet because the 
  // measurements are somewhat unreliable. Not sure whether it's the 
  // sensor itself or just some mechanical issue.
  /*
  if(smuffConfig.useDuetLaser) {
    duetLS.attach(Z_END_PIN);
  }
  */
  
  // Please note: All the PWM pins on the SKR are not working as
  // expected. Maybe it's a common libmaple issue, maybe it's a 
  // STM32 timer related thing or maybe it's the
  // hardware design of the board itself. 
  // Can't tell for sure, need some more investigation.
  if(HEATER0_PIN != -1) {
    pinMode(HEATER0_PIN, OUTPUT);
  }
#if defined(__STM32F1__) || defined(__ESP32__)
  if(HEATBED_PIN != -1) {
    #if defined(__STM32F1__)
      pinMode(HEATBED_PIN, PWM); 
    #else 
      pinMode(HEATBED_PIN, OUTPUT);
    #endif
  }
#endif
  
  if(FAN_PIN != -1) {
#ifdef __STM32F1__
  pinMode(FAN_PIN, PWM);
#elif defined(__ESP32__) 
  ledcSetup(FAN_CHANNEL, FAN_FREQ, 8);
  ledcAttachPin(FAN_PIN, FAN_CHANNEL);
  //__debug(PSTR("DONE FAN PIN CONFIG"));
#else
  pinMode(FAN_PIN, OUTPUT);
#endif      
  if(smuffConfig.fanSpeed >= 0 && smuffConfig.fanSpeed <= 100) {
    #if defined (__ESP32__)
    ledcWrite(FAN_PIN, map(smuffConfig.fanSpeed, 0, 100, 0, 65535));
    #elif defined(__STM32F1__) 
    analogWrite(FAN_PIN, map(smuffConfig.fanSpeed, 0, 100, 0, 65535));    
    #else
    analogWrite(FAN_PIN, map(smuffConfig.fanSpeed, 0, 100, 0, 255));    
    #endif
  }
  //__debug(PSTR("DONE FAN init"));
  #if defined( __ESP32__)
    // init the PCF8574 port expander and set pin modes (0-5 OUTPUT, 6-7 INPUT)
    portEx.begin(PORT_EXPANDER_ADDRESS, false);
    for(int i=0; i< 6; i++) {
      portEx.pinMode(i, OUTPUT);
      portEx.setPin(i);
    }
    portEx.pinMode(6, INPUT_PULLUP);
    portEx.pinMode(7, INPUT_PULLUP);
    portEx.resetPin(0);
    //__debug(PSTR("DONE PortExpander init"));
  #endif
  }

#ifdef __AVR__
  // We can't do Master and Slave on this device. 
  // Slave mode is used for the I2C OLE Display on SKR mini
  if(smuffConfig.i2cAddress != 0) {
    Wire.begin(smuffConfig.i2cAddress);
    Wire.onReceive(wireReceiveEvent);
  }
  //__debug(PSTR("DONE I2C init"));
 #endif
  if(smuffConfig.homeAfterFeed) {
    moveHome(REVOLVER, false, false);
  }
  else {
    resetRevolver();
  }
  //__debug(PSTR("DONE reset Revolver"));
  
  sendStartResponse(0);
  if(smuffConfig.prusaMMU2) {
#ifdef __STM32F1__    
    sendStartResponse(1);
    sendStartResponse(3);
#else
    sendStartResponse(2);
#endif
  }
  
  pwrSaveTime = millis();
  
  initBeep();
  enablePS = true;    // enable periodically sending status, if configured
}

void setupSteppers() {

  steppers[SELECTOR] = ZStepper(SELECTOR, (char*)"Selector", X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, smuffConfig.acceleration_X, smuffConfig.maxSpeed_X);
  steppers[SELECTOR].setEndstop(X_END_PIN, smuffConfig.endstopTrigger_X, ZStepper::MIN);
  steppers[SELECTOR].stepFunc = overrideStepX;
  steppers[SELECTOR].setMaxStepCount(smuffConfig.maxSteps_X);
  steppers[SELECTOR].setStepsPerMM(smuffConfig.stepsPerMM_X);
  steppers[SELECTOR].setInvertDir(smuffConfig.invertDir_X);
  steppers[SELECTOR].setMaxHSpeed(smuffConfig.maxSpeedHS_X);
  steppers[SELECTOR].setAccelDistance(smuffConfig.accelDistance_X);

#if !defined(SMUFF_V5)
  steppers[REVOLVER] = ZStepper(REVOLVER, (char*)"Revolver", Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, smuffConfig.acceleration_Y, smuffConfig.maxSpeed_Y);
  steppers[REVOLVER].setEndstop(Y_END_PIN, smuffConfig.endstopTrigger_Y, ZStepper::ORBITAL);
  steppers[REVOLVER].stepFunc = overrideStepY;
  steppers[REVOLVER].setMaxStepCount(smuffConfig.stepsPerRevolution_Y);
  steppers[REVOLVER].setStepsPerDegree(smuffConfig.stepsPerRevolution_Y/360);
  steppers[REVOLVER].endstopFunc = endstopYevent;
  steppers[REVOLVER].setInvertDir(smuffConfig.invertDir_Y);
  steppers[REVOLVER].setMaxHSpeed(smuffConfig.maxSpeedHS_Y);
  steppers[REVOLVER].setAccelDistance(smuffConfig.accelDistance_Y);
#endif

  steppers[FEEDER] = ZStepper(FEEDER, (char*)"Feeder", Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, smuffConfig.acceleration_Z, smuffConfig.maxSpeed_Z);
  /*
  if(smuffConfig.useDuetLaser) {
    steppers[FEEDER].setEndstop(-1, smuffConfig.endstopTrigger_Z, ZStepper::MIN);
    steppers[FEEDER].endstopCheck = checkDuetEndstop;
  }
  else
  */
  steppers[FEEDER].setEndstop(Z_END_PIN, smuffConfig.endstopTrigger_Z, ZStepper::MIN);
  if(Z_END2_PIN != -1)
    steppers[FEEDER].setEndstop(Z_END2_PIN, smuffConfig.endstopTrigger_Z, ZStepper::MIN, 2); // optional; used for testing only
  steppers[FEEDER].stepFunc = overrideStepZ;
  steppers[FEEDER].setStepsPerMM(smuffConfig.stepsPerMM_Z);
  steppers[FEEDER].endstopFunc = endstopZevent;
  steppers[FEEDER].endstop2Func = endstopZ2event;
  steppers[FEEDER].setInvertDir(smuffConfig.invertDir_Z);
  steppers[FEEDER].setMaxHSpeed(smuffConfig.maxSpeedHS_Z);
  steppers[FEEDER].setAccelDistance(smuffConfig.accelDistance_Z);

  for(int i=0; i < NUM_STEPPERS; i++) {
      steppers[i].runAndWaitFunc = runAndWait;
      steppers[i].runNoWaitFunc = runNoWait;
      steppers[i].setEnabled(true);
  }
  //__debug(PSTR("DONE enabling steppers"));

  for(int i=0; i < MAX_TOOLS; i++) {
    swapTools[i] = i;
  }
  //__debug(PSTR("DONE initializing swaps"));
}

void setupTimers() {
#if defined(__AVR__)
  // *****
  // Attn: Servo uses TIMER5 if it's setup to create its own timer 
  // *****
  stepperTimer.setupTimer(ZTimer::ZTIMER4, ZTimer::PRESCALER1);
  encoderTimer.setupTimer(ZTimer::ZTIMER3, ZTimer::PRESCALER256); // round about 1ms on 16MHz CPU
#elif defined(__ESP32__)
  // *****
  // Attn: Servo uses TIMER4 if it's setup to create its own timer 
  //       PortExpander uses TIMER3
  // *****
  stepperTimer.setupTimer(ZTimer::ZTIMER1, 4, 1);                 // prescaler set to 20MHz, timer will be calculated as needed
  encoderTimer.setupTimer(ZTimer::ZTIMER2, 80, 1000);             // 1ms on 80MHz Timer Clock
#else
  // *****
  // Attn: 
  //    Servo uses TIMER5 CH1 if it's set up to create its own timer 
  //    Fan uses TIMER8 CH3
  //    Beeper uses TIMER4 CH3
  //    PC9 (Heatbed) uses TIMER1 CH1
  // *****
  stepperTimer.setupTimer(ZTimer::ZTIMER2, 3, 1, 1);               // prescaler set to 72MHz, timer will be calculated as needed
  encoderTimer.setupTimer(ZTimer::ZTIMER1, 1, 8, 9000);            // 1ms on 72MHz CPU
  setToneTimerChannel((uint8_t)ZTimer::ZTIMER4, 3);                // force TIMER4 / CH3 on STM32F1x for tone library
#endif

  stepperTimer.setupTimerHook(isrStepperHandler);
  encoderTimer.setupTimerHook(isrEncoderHandler);
  encoder.setDoubleClickEnabled(true);

#if defined(__STM32F1__)
  encoderTimer.setNextInterruptInterval(9000);    // run encoder timer (STM32)
#elif defined(__ESP32__)
  encoderTimer.setNextInterruptInterval(1000);    // run encoder timer (ESP32)
#else
  encoderTimer.setNextInterruptInterval(40);      // run encoder timer (AVR)
#endif
  //__debug(PSTR("DONE setup timers"));
}

void startStepperInterval() {
  unsigned int minDuration = 65535;
  for(int i = 0; i < NUM_STEPPERS; i++) {
    if((_BV(i) & remainingSteppersFlag) && steppers[i].getDuration() < minDuration ) {
      minDuration = steppers[i].getDuration();
    }
  }

  nextStepperFlag = 0;
  for(int i = 0; i < NUM_STEPPERS; i++) {
    if ( (_BV(i) & remainingSteppersFlag) && steppers[i].getDuration() == minDuration )
      nextStepperFlag |= _BV(i);
  }

  if(remainingSteppersFlag == 0) {
    stepperTimer.stopTimer();
    stepperTimer.setOverflow(65534);
  }
  else {
    //__debug(PSTR("minDuration: %d"), minDuration);
    stepperTimer.setNextInterruptInterval(minDuration);
  }
}

void isrStepperHandler() {
  stepperTimer.stopTimer();
  unsigned int tmp = stepperTimer.getOverflow(); 
  stepperTimer.setOverflow(65534);

  for (int i = 0; i < NUM_STEPPERS; i++) {
    if(!(_BV(i) & remainingSteppersFlag))
      continue;

    if(!(nextStepperFlag & _BV(i))) {
      steppers[i].setDuration(steppers[i].getDuration() - tmp);
      continue;
    }
    
    steppers[i].handleISR();
    if(steppers[i].getMovementDone())
      remainingSteppersFlag &= ~_BV(i); 
  }
  //__debug(PSTR("ISR(): %d"), remainingSteppersFlag);
  startStepperInterval();
}

void runNoWait(volatile int index) {
  if(index != -1)
    remainingSteppersFlag |= _BV(index);
  startStepperInterval();
  //__debug(PSTR("started stepper %d"), index);
}

void runAndWait(volatile int index) {
  runNoWait(index);
  while(remainingSteppersFlag) {
    checkSerialPending(); // not a really nice solution but needed to check serials for "Abort" command in PMMU mode
#if defined(__STM32F1__) // || defined(__ESP32__)
    if(!showMenu)
      refreshStatus(true);
#endif
  }
  //__debug(PSTR("RunAndWait done"));
  //if(index==FEEDER) __debug(PSTR("Fed: %smm"), String(steppers[index].getStepsTakenMM()).c_str());
}

void refreshStatus(bool withLogo) {
  display.firstPage();
  do {
    if(withLogo) 
      drawLogo();
    drawStatus();
  } while(display.nextPage());
  lastDisplayRefresh = millis();
}

void loop() {

  //__debug(PSTR("gcInterval: %ld"), gcInterval);
#if defined(__STM32F1__) || defined(__ESP32__)
  checkSerialPending();
#endif
  if(feederEndstop() != lastZEndstopState) {
    lastZEndstopState = feederEndstop();
    bool state = feederEndstop();
    setSignalPort(FEEDER_SIGNAL, state);
    delay(200);
    setSignalPort(FEEDER_SIGNAL, !state);
    delay(200);
    setSignalPort(FEEDER_SIGNAL, state);
  }
  if(feederEndstop(2) != lastZEndstop2State) {
    lastZEndstop2State = feederEndstop(2);
    endstopZ2HitCnt++;
    //__debug(PSTR("Endstop Z2: %d"), endstopZ2HitCnt);
  }
  //__debug(PSTR("Mem: %d"), freeMemory());

  checkUserMessage();
  if(!displayingUserMessage) {
    if(!isPwrSave && !showMenu) {
      unsigned long elapsed = millis()-lastDisplayRefresh;
#if defined(__STM32F1__)
      if(elapsed > 250) { // refresh display every 250ms
        refreshStatus(true);
      }
#elif defined(__ESP32__)
      if(elapsed > 500) { // refresh display every 500ms
        refreshStatus(true);
      }
#else
      if(elapsed > 500) { // refresh display every 500ms
        refreshStatus(true);
      }
#endif  
    }
  }

  if(!showMenu) {
    int button = encoder.getButton();
    if(button == ClickEncoder::Pressed && isPwrSave) {
      setPwrSave(0);
    }
    else if(button == ClickEncoder::Held) {
      setPwrSave(0);
      showMenu = true;
      char title[] = {"Settings"};
      showSettingsMenu(title);
      showMenu = false;
      debounceButton();
#ifdef __STM32F1__
#endif
    }
    else {
      int turn = encoder.getValue();
      if(turn != 0) {
        if(isPwrSave) {
          setPwrSave(0);
        }
        else {
          displayingUserMessage = false;
          showMenu = true;
          if(turn == -1) {
            showMainMenu();
          }
          else {
            showToolsMenu();
          }
          showMenu = false;
        }
      }
    }
  }
  
  if((millis() - pwrSaveTime)/1000 >= (unsigned long)smuffConfig.powerSaveTimeout && !isPwrSave) {
    //__debug(PSTR("Power save mode after %d seconds (%d)"), (millis() - pwrSaveTime)/1000, smuffConfig.powerSaveTimeout);
    setPwrSave(1);
  }
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

void setPwrSave(int state) {
  display.setPowerSave(state);
  isPwrSave = state == 1;
  if(!isPwrSave) {
    delay(1200);
    pwrSaveTime = millis();
  }
}

bool checkUserMessage() {
  int button = digitalRead(ENCODER_BUTTON_PIN);
  if(button == LOW && displayingUserMessage) {
    displayingUserMessage = false;
  }
  //__debug(PSTR("%ld"), (millis()-userMessageTime)/1000);
  if(millis()-userMessageTime > USER_MESSAGE_RESET*1000) {
    displayingUserMessage = false;
  }
  return displayingUserMessage;
}


#ifndef __AVR__
void serialEventRun() {
  if(Serial.available()) serialEvent();
  if(Serial1.available()) serialEvent1();
  if(Serial2.available()) serialEvent2();
  
  #ifdef __STM32F1__
  if(Serial3.available()) serialEvent3();
  #endif
}
#endif

void checkSerialPending() {
  serialEventRun(); 
}

void resetSerialBuffer(int serial) {
  switch(serial) {
    case 0: serialBuffer0 = ""; break;
    case 1: serialBuffer0 = ""; break;
    case 2: serialBuffer2 = ""; break;
    case 3: serialBuffer2 = ""; break;
    case 9: serialBuffer9 = ""; break;
  }
}

bool isQuote = false;
void filterSerialInput(String& buffer, char in) {
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
  if(smuffConfig.hasPanelDue) {
    #if defined(__ESP32__)
    Serial2.write(in);
    #elif defined(__STM32F1__)
    Serial3.write(in);
    #endif
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

volatile bool processingSerial0;

void serialEvent() {
  if(processingSerial0)
    return;
    
  while(Serial.available()) {
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

volatile bool processingSerial2;

void serialEvent2() {
  if(processingSerial2)
    return;
  while(Serial2.available()) {
    processingSerial2 = true;
    char in = (char)Serial2.read();
    // in case of PanelDue connected, route everthing to Duet3D - do not process it any further 
    if(smuffConfig.hasPanelDue) {
      Serial.write(in);
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

volatile bool processingSerial1;
 
#ifndef __AVR__
void serialEvent1() {
  if(processingSerial1)
    return;
  
  while(Serial1.available()) {
    processingSerial1 = true;
    char in = (char)Serial1.read();
    // check for JSON data first
    if(isJsonData(in))
      continue;
    if (in == '\n') {
      //__debug(PSTR("Received-1: %s"), serialBuffer0.c_str());
      parseGcode(serialBuffer0, 1);
      isQuote = false;
      actionOk = false;
    }
    else {
      filterSerialInput(serialBuffer0, in);
    }
  }
  processingSerial1 = false;

}

#ifndef __ESP32__
volatile bool processingSerial3;

void serialEvent3() {
  if(processingSerial3)
    return;
  
  while(Serial3.available()) {
    processingSerial3 = true;
    char in = (char)Serial3.read();
    //Serial3.write(in);
    if (in == '\n') {
      //__debug(PSTR("Received-3: %s"), serialBuffer2.c_str());
      parseGcode(serialBuffer2, 3);
      isQuote = false;
      actionOk = false;
    }
    else {
      filterSerialInput(serialBuffer2, in);
    }
  }
  processingSerial3 = false;
}
#endif
#endif


#ifdef __AVR__
void wireReceiveEvent(int numBytes) {
  while (Wire.available()) {
    char in = (char)Wire.read();
    if (in == '\n') {
      parseGcode(serialBuffer9, 9);
    }
    else
        serialBuffer9 += in;
  }
}
#endif

