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

#if defined(__STM32F1__)
#define PRODUCT_ID   0x29                     // for CompositeSerial
#endif

#if defined(USE_COMPOSITE_SERIAL)
bool writeSDCard(const uint8_t *writebuff, uint32_t startSector, uint16_t numSectors) {
  return SD.card()->writeSectors(startSector, writebuff, numSectors);
}

bool readSDCard(uint8_t *readbuff, uint32_t startSector, uint16_t numSectors) {
  return SD.card()->readSectors(startSector, readbuff, numSectors);
}
#endif

void initUSB() {
#if defined(__STM32F1__)
  if(USB_CONNECT_PIN != -1) {
    pinMode(USB_CONNECT_PIN, OUTPUT);
    digitalWrite(USB_CONNECT_PIN, HIGH);      // USB clear connection
    delay(1000);                              // give OS time to notice
    digitalWrite(USB_CONNECT_PIN, LOW);       // USB reestablish connection
  }

  #if defined(USE_COMPOSITE_SERIAL)
  /* Not tested yet */
  bool sdStat;
  if(SDCS_PIN != -1) {
    sdStat = SD.begin(SDCS_PIN, SD_SCK_MHZ(4));
  }
  else {
    sdStat = SD.begin();
  }
  if(sdStat) {
    USBComposite.setProductId(PRODUCT_ID);
    MassStorage.setDriveData(0, SD.card()->sectorCount(), readSDCard, writeSDCard);
    MassStorage.registerComponent();
    CompositeSerial.registerComponent();
    USBComposite.begin();
    delay(2000);
  }  
  #endif
#endif
}

void setupDeviceName() {
  String appendix = "";
#if defined(__ESP32__)
  appendix = WiFi.macAddress().substring(9);
  appendix.replace(":", "");
#endif
  wirelessHostname = String("SMuFF") + "_" + appendix;
}

void setupBuzzer() {
  if(BEEPER_PIN != -1) {
#if defined(__ESP32__)
    ledcSetup(BEEPER_CHANNEL, 5000, 8);
    ledcAttachPin(BEEPER_PIN, BEEPER_CHANNEL);
#else
#endif
  }
}


/*
  Initialize FastLED library for NeoPixels.
  Primarily used for backlight on some displays (i.e. FYSETC 12864 Mini V2.1)
*/
void initFastLED() {
  #if NEOPIXEL_PIN != -1 && defined(USE_FASTLED_BACKLIGHT)
    FastLED.addLeds<LED_TYPE, NEOPIXEL_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);
  #endif
}

/*
  Initialize pin for hardware debugging.
  Primarily used to attach an oscilloscope.
*/
void initHwDebug() {
  #if defined(__HW_DEBUG__)
  pinMode(DEBUG_PIN, OUTPUT);
  digitalWrite(DEBUG_PIN, HIGH);
  #endif
}

void setupDuetLaserSensor() {
  // Duet Laser Sensor is being used as the Feeder endstop
  if(smuffConfig.useDuetLaser) {
    duetLS.attach(Z_END_DUET_PIN); 
  }
}

void setupSerialBT() {
#ifdef __ESP32__
  // this line must be kept, otherwise BT power down will cause a reset
  SerialBT.begin(wirelessHostname);
#endif
}

void setupSerial() {
  // special case: 
  // if the baudrate is set to 0, the board is running out of memory
  if(smuffConfig.serial0Baudrate != 0) { 
    if(smuffConfig.serial0Baudrate != 115200) {
      Serial.end();
      Serial.begin(smuffConfig.serial0Baudrate);
    }
  }
  else {
    writeConfig(&Serial);
    longBeep(3);
    showDialog(P_TitleConfigError, P_ConfigFail1, P_ConfigFail4, P_OkButtonOnly);
  }
  
  Serial.begin(smuffConfig.serial0Baudrate);
  if(CAN_USE_SERIAL1) Serial1.begin(smuffConfig.serial1Baudrate);
  if(CAN_USE_SERIAL2) Serial2.begin(smuffConfig.serial2Baudrate);
  if(CAN_USE_SERIAL3) Serial3.begin(smuffConfig.serial3Baudrate);
  //__debug(PSTR("DONE init SERIAL"));
}

void setupSwSerial0() {
#if defined(USE_SW_SERIAL)
  swSer0.begin(TMC_BAUDRATE);
#endif
}

void setupRelay() {
  if(RELAIS_PIN != -1 && smuffConfig.revolverIsServo) {
    // Relay mode will only work on servo variants
    pinMode(RELAIS_PIN, OUTPUT);
    // if there's an external Feeder stepper defined (i.e. the 3D-Printer drives the Feeder),
    // switch on the external stepper by default. Otherwise, use the interal stepper.
    if(smuffConfig.externalControl_Z)
      switchFeederStepper(EXTERNAL);
    else
      switchFeederStepper(INTERNAL);
  }
}

void setupServos() {

  // setup the Wiper servo
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
    int resetPos = 90, param;
    // try to find out the default reset position of the wiper servo from 
    // within the wipe sequence
    if((param = getParam(String(smuffConfig.wipeSequence), (char*)"P")) != -1) {
      resetPos = param;
    }
    setServoPos(SERVO_WIPER, resetPos);
  }
  
  // setup the Lid servo (replaces the Revolver stepper motor)
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
    setServoPos(SERVO_LID, smuffConfig.revolverOffPos);
  }
}

void setupHeaterBed() {
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
}

void setupFan() {

  if(FAN_PIN != -1) {
  #ifdef __STM32F1__
    fan.attach(FAN_PIN, 0);
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
      //pwmWrite(FAN_PIN, map(smuffConfig.fanSpeed, 0, 100, 0, 65535));    
      fan.setFanSpeed(smuffConfig.fanSpeed);
      #else
      analogWrite(FAN_PIN, map(smuffConfig.fanSpeed, 0, 100, 0, 255));    
      #endif
    }
  }
  //__debug(PSTR("DONE FAN init"));
}

void setupPortExpander() {
  #if defined(__ESP32__)
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

void setupI2C() {
  #ifdef __AVR__
  // We can't do Master and Slave on this device. 
  // Slave mode is used for the I2C OLE Display on SKR mini
  if(smuffConfig.i2cAddress != 0) {
    Wire.begin(smuffConfig.i2cAddress);
    Wire.onReceive(wireReceiveEvent);
  }
  //__debug(PSTR("DONE I2C init"));
 #endif
}

void setupBacklight() {
  #if defined(USE_RGB_BACKLIGHT) || defined(USE_FASTLED_BACKLIGHT)
  for(int i=0; i< 8; i++) {
    setBacklightIndex(i);                                 // flip through all colors
    delay(250);
  }
  setBacklightIndex(smuffConfig.backlightColor);          // turn on the LCD backlight according to the configured setting
  #endif
}

void setupEncoder() {
  encoder.setDoubleClickEnabled(true);                    // enable doubleclick on the rotary encoder
  encoder.setEnableSound(smuffConfig.encoderTickSound);
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
  if(smuffConfig.stepperStall[SELECTOR] > 0)
    steppers[SELECTOR].stallCheck = stallCheckX;

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
  if(smuffConfig.stepperStall[REVOLVER] > 0)
    steppers[REVOLVER].stallCheck = stallCheckY;
#else
  // we don't use the Revolver stepper but an servo instead, although
  // create a dummy instance
  steppers[REVOLVER] = ZStepper(REVOLVER, (char*)"Revolver", -1, -1, Y_ENABLE_PIN, 0, 0);
#endif

  steppers[FEEDER] = ZStepper(FEEDER, (char*)"Feeder", Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, smuffConfig.acceleration_Z, smuffConfig.maxSpeed_Z);
  if(smuffConfig.useDuetLaser) {
    steppers[FEEDER].setEndstop(-1, smuffConfig.endstopTrigger_Z, ZStepper::MIN);
    steppers[FEEDER].endstopCheck = checkDuetEndstop;
  }
  else
    steppers[FEEDER].setEndstop(Z_END_PIN, smuffConfig.endstopTrigger_Z, ZStepper::MIN);
  if(Z_END2_PIN != -1)
    steppers[FEEDER].setEndstop(Z_END2_PIN, smuffConfig.endstopTrigger_Z, ZStepper::MIN, 2); // optional; used for testing only
  steppers[FEEDER].stepFunc = overrideStepZ;
  steppers[FEEDER].setStepsPerMM(smuffConfig.stepsPerMM_Z);
  steppers[FEEDER].endstopFunc = endstopEventZ;
  steppers[FEEDER].endstop2Func = endstopEventZ2;
  steppers[FEEDER].setInvertDir(smuffConfig.invertDir_Z);
  steppers[FEEDER].setMaxHSpeed(smuffConfig.maxSpeedHS_Z);
  steppers[FEEDER].setAccelDistance(smuffConfig.accelDistance_Z);
  if(smuffConfig.stepperStall[FEEDER] > 0)
    steppers[FEEDER].stallCheck = stallCheckZ;

  for(int i=0; i < NUM_STEPPERS; i++) {
      steppers[i].runAndWaitFunc = runAndWait;
      steppers[i].runNoWaitFunc = runNoWait;
      steppers[i].setEnabled(true);
  }
  
  __debug(PSTR("DONE init/enabling steppers"));
  for(int i=0; i < MAX_TOOLS; i++) {
    swapTools[i] = i;
  }
  //__debug(PSTR("DONE initializing swaps"));
}


TMC2209Stepper* initDriver(int axis, int rx_pin, int tx_pin) {
  int mode    = smuffConfig.stepperMode[axis];
  int stall   = smuffConfig.stepperStall[axis];
  int current = smuffConfig.stepperPower[axis];
  int msteps  = smuffConfig.stepperMicrosteps[axis];
  int csmin   = smuffConfig.stepperCSmin[axis];
  int csmax   = smuffConfig.stepperCSmax[axis];
  int csdown  = smuffConfig.stepperCSdown[axis];
  float rsense= smuffConfig.stepperRSense[axis];
  int drvrAdr = smuffConfig.stepperAddr[axis];
  int toff    = stall == 0 ? 4 : 3; // smuffConfig.stepperToff[axis];
  bool spread = smuffConfig.stepperSpread[axis];

  if(mode == 0) {
    //__debug(PSTR("Driver for %c-axis skipped"), 'X'+axis);
    return NULL;
  }

  TMC2209Stepper* driver = new TMC2209Stepper(rx_pin, tx_pin, rsense, drvrAdr);
  //__debug(PSTR("Driver for %c-axis initialized"), 'X'+axis);

  steppers[axis].setEnabled(true);
  driver->beginSerial(TMC_BAUDRATE);
  delay(50);
  driver->toff(toff);
  driver->blank_time(24);
  driver->internal_Rsense(true);
  driver->Rsense = rsense;
  driver->I_scale_analog(true);     // set external Vref
  driver->rms_current(current);     // set current in mA
  driver->mstep_reg_select(1);      // set microstepping
  driver->microsteps(msteps);       
  
  // setup StallGuard only if stepperStall value is between 1 and 255
  // otherwise put it in SpreadCycle mode
  if(stall >= 1 && stall <= 255) {
    #if defined(TMC_TYPE_2130)
    stall = (int)map(stall, 1,  255, -63, 63);  // remap values for TMC2130 (not tested yet!)
    #endif
    driver->pwm_autoscale(true);
    driver->TCOOLTHRS(0xFFFFF); 
    if(csmin >0 && csmax >0) {
      driver->semin(csmin);
      driver->semax(csmax);
      driver->sedn(csdown);
    }
    driver->SGTHRS(0);
    driver->en_spreadCycle(false);   // set StealthChop (enable StallGuard)
  }
  else {
    driver->TCOOLTHRS(0); 
    driver->en_spreadCycle(true);   // set SpreadCycle (disable StallGuard)
  }
  return driver;
}

void setupTMCDrivers() {

  #if defined(X_SERIAL_TX_PIN)
  driverX = initDriver(SELECTOR, X_SERIAL_TX_PIN, X_SERIAL_TX_PIN);
  #endif
  #if defined(Y_SERIAL_TX_PIN)
  driverY = initDriver(REVOLVER, Y_SERIAL_TX_PIN, Y_SERIAL_TX_PIN);
  #endif
  #if defined(Z_SERIAL_TX_PIN)
  driverZ = initDriver(FEEDER,   Z_SERIAL_TX_PIN, Z_SERIAL_TX_PIN);
  #endif
  #if defined(E_SERIAL_TX_PIN)
  driverE = initDriver(FEEDER,   E_SERIAL_TX_PIN, E_SERIAL_TX_PIN);
  #endif
  __debug(PSTR("DONE initializing TMC Steppers"));
}

void setupTimers() {
#if defined(__AVR__)
  // *****
  // Attn: 
  //    Servo uses:         TIMER5 (if it's setup to create its own timer)
  //    Steppers use:       TIMER4
  //    Encoder uses:       gpTimer 
  // *****
  stepperTimer.setupTimer(ZTimer::ZTIMER4, ZTimer::PRESCALER1);
  gpTimer.setupTimer(ZTimer::ZTIMER3, ZTimer::PRESCALER256);      // round about 1ms on 16MHz CPU

#elif defined(__ESP32__)
  // *****
  // Attn:
  //    Servo uses:         TIMER3 (if it's setup to create its own timer)
  //    PortExpander uses:  TIMER3 (via general purpose timer)
  //    Steppers use:       TIMER1
  //    Encoder uses:       gpTimer
  // *****
  stepperTimer.setupTimer(ZTimer::ZTIMER1, 4);          // prescaler set to 20MHz, timer will be calculated as needed
  gp.setupTimer(ZTimer::ZTIMER2, 80);                   // 1ms on 80MHz timer clock
#else
  // *****
  // Attn: 
  //    PA8 (Fan) uses:     TIMER1 CH1 (predefined by libmaple for PWM)
  //    Steppers use:       TIMER5 CH1 (may corrupt TH0 readings)
  //    GP timer uses:      TIMER8 CH1 (general, encoder, servo)
  //    Beeper uses:        TIMER4 CH3
  //    SW-Serial uses:     TIMER3 CH4 (see SoftwareSerialM library)
  //    PC8 (Heater0) uses: TIMER8 CH3 (predefined by libmaple for PWM)
  //    PC9 (Heatbed) uses: TIMER8 CH4 (predefined by libmaple for PWM)
  //
  // Warning: If you need to modify this assignment, be sure you know what you do!
  //          Swapping timers and/or channels may lead to a non functioning device or
  //          communication interrupts/breaks. Read the STM32F1 MCU spec. and check
  //          the libmaple library settings before you do so.
  // *****
  stepperTimer.setupTimer(ZTimer::ZTIMER2, ZTimer::CH1, 1, 1);    // prescaler set to 72MHz, timer will be calculated as needed
  gpTimer.setupTimer(ZTimer::ZTIMER8, ZTimer::CH1, 8, 0);         // prescaler set to 9MHz, timer will be set to 50uS
  setToneTimerChannel(ZTimer::ZTIMER4, ZTimer::CH3);              // force TIMER4 / CH3 on STM32F1x for tone library
#endif

  stepperTimer.setupTimerHook(isrStepperHandler);         // setup the ISR for the steppers
  gpTimer.setupTimerHook(isrGPTimerHandler);              // setup the ISR for rotary encoder, servo and general timers 

#if defined(__STM32F1__)
  gpTimer.setNextInterruptInterval(450);                  // run general purpose (gp)timer on 50uS (STM32) 
#elif defined(__ESP32__)
  gpTimer.setNextInterruptInterval(50);                   // run general purpose (gp)timer on 50uS (ESP32) 
#else
  gpTimer.setNextInterruptInterval(3);                    // run general purpose (gp)timer on 48uS (AVR)
#endif
  //__debug(PSTR("DONE setup timers"));
}
