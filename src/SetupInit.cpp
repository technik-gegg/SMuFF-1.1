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
#include <../stm32f1/include/series/nvic.h>
#define PRODUCT_ID 0x29 // for CompositeSerial
#endif

#if defined(USE_COMPOSITE_SERIAL)
extern SdFat SD;

bool writeSDCard(const uint8_t *writebuff, uint32_t startSector, uint16_t numSectors)
{
  return SD.card()->writeSectors(startSector, writebuff, numSectors);
}

bool readSDCard(uint8_t *readbuff, uint32_t startSector, uint16_t numSectors)
{
  return SD.card()->readSectors(startSector, readbuff, numSectors);
}
#endif

void initUSB()
{
#if defined(__STM32F1__)
  if (USB_CONNECT_PIN != -1)
  {
    pinMode(USB_CONNECT_PIN, OUTPUT);
    digitalWrite(USB_CONNECT_PIN, HIGH); // USB clear connection
    delay(250);                          // give OS time to notice
    digitalWrite(USB_CONNECT_PIN, LOW);  // USB reestablish connection
  }

#if defined(USE_COMPOSITE_SERIAL)
  /* Not tested yet */
  bool sdStat = initSD(false);
  __debugS(PSTR("SD status"), sdStat);
  if (sdStat)
  {
    USBComposite.setProductId(PRODUCT_ID);
    MassStorage.setDriveData(0, SD.card()->sectorCount(), readSDCard, writeSDCard);
    MassStorage.registerComponent();
    CompositeSerial.registerComponent();
    USBComposite.begin();
    __debugS(PSTR("USB Composite started"));
    delay(2000);
  }
#endif
#endif
}

void setupDeviceName()
{
  String appendix = "";
#if defined(__ESP32__)
  appendix = WiFi.macAddress().substring(9);
  appendix.replace(":", "");
#endif
  wirelessHostname = String("SMuFF") + "_" + appendix;
}

void setupBuzzer()
{
  if (BEEPER_PIN != -1)
  {
#if defined(__ESP32__)
    ledcSetup(BEEPER_CHANNEL, 5000, 8);
    ledcAttachPin(BEEPER_PIN, BEEPER_CHANNEL);
#else
#endif
  }
}

void readSequences()
{
  char tuneData[150];
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

/*
  Initialize FastLED library for NeoPixels.
  Primarily used for backlight on some displays (i.e. FYSETC 12864 Mini V2.1)
  but also for the tool status if wired and defined.
*/
void initFastLED()
{

#if NEOPIXEL_PIN != -1 && defined(USE_FASTLED_BACKLIGHT)
  pinMode(NEOPIXEL_PIN, OUTPUT);
  cBackLight = &FastLED.addLeds<LED_TYPE, NEOPIXEL_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  __debugS(PSTR("FastLED for backlight initialized"));
#else
  __debugS(PSTR("FastLED for backlight not enabled. Neopixel Pin: %d"), NEOPIXEL_PIN);
#endif
#if NEOPIXEL_TOOL_PIN != -1 && defined(USE_FASTLED_TOOLS)
  pinMode(NEOPIXEL_TOOL_PIN, OUTPUT);
  cTools = &FastLED.addLeds<LED_TYPE_TOOL, NEOPIXEL_TOOL_PIN, COLOR_ORDER_TOOL>(ledsTool, smuffConfig.toolCount).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS_TOOL);
  __debugS(PSTR("FastLED for tools initialized"));
#else
  __debugS(PSTR("FastLED for tools not enabled. Neopixel Pin: %d"), NEOPIXEL_TOOL_PIN);
#endif
}

/*
  Initialize pin for hardware debugging.
  Primarily used to attach an oscilloscope.
*/
void initHwDebug()
{
#if defined(__HW_DEBUG__) && defined(DEBUG_PIN) && DEBUG_PIN != -1
  pinMode(DEBUG_PIN, OUTPUT);
  digitalWrite(DEBUG_PIN, HIGH);
  __debugS(PSTR("Hardware Debug Pin initialized"));
#endif
}

void setupDuetSignals()
{
#if defined(DUET_SIG_FED_PIN) && DUET_SIG_FED_PIN != -1
  pinMode(DUET_SIG_FED_PIN, OUTPUT);
  digitalWrite(DUET_SIG_FED_PIN, LOW);
#endif
#if defined(DUET_SIG_SEL_PIN) && DUET_SIG_SEL_PIN != -1
  pinMode(DUET_SIG_SEL_PIN, OUTPUT);
  digitalWrite(DUET_SIG_SEL_PIN, LOW);
#endif
}

void setupDuetLaserSensor()
{
  // Duet Laser Sensor is being used as the Feeder endstop
  if (smuffConfig.useDuetLaser)
  {
    duetLS.attach(Z_END_DUET_PIN);
  }
}

void setupSerialBT()
{
#ifdef __ESP32__
  // this line must be kept, otherwise BT power down will cause a reset
  extern BluetoothSerial SerialBT;
  SerialBT.begin(wirelessHostname);
#endif
}

void setupSerial()
{
  //__debugS(PSTR("Setting up serial"));
  // special case:
  // if the baudrate is set to 0, the board is running out of memory
  if (smuffConfig.serialBaudrates[0] != 0)
  {
    if (smuffConfig.serialBaudrates[0] != 115200)
    {
//__debugS(PSTR("End SERIAL0 -> %ld"), smuffConfig.serialBaudrates[0]);
#if defined(USE_COMPOSITE_SERIAL)
      CompositeSerial.end();
      delay(250);
      CompositeSerial.begin(smuffConfig.serialBaudrate[0]);
#else
      Serial.end();
      delay(250);
      Serial.begin(smuffConfig.serialBaudrates[0]);
#endif
    }
  }
  else
  {
    __debugS(PSTR("Config error for serial\n--------------------\n"));
    writeConfig((Print *)debugSerial);
    __debugS(PSTR("\n--------------------"));
    longBeep(3);
    showDialog(P_TitleConfigError, P_ConfigFail1, P_ConfigFail4, P_OkButtonOnly);
  }

  if (CAN_USE_SERIAL1 && smuffConfig.serialBaudrates[1] != 115200 && smuffConfig.serialBaudrates[1] >= 4800 && smuffConfig.serialBaudrates[1] <= 230400)
  {
    //__debugS(PSTR("End SERIAL1 -> %ld"), smuffConfig.serialBaudrates[1]);
    Serial1.end();
    delay(150);
    Serial1.begin(smuffConfig.serialBaudrates[1]);
    delay(250);
    //__debugS(PSTR("DONE init SERIAL1"));
  }
  if (CAN_USE_SERIAL2 && smuffConfig.serialBaudrates[2] != 115200 && smuffConfig.serialBaudrates[2] >= 4800 && smuffConfig.serialBaudrates[2] <= 230400)
  {
    //__debugS(PSTR("End SERIAL2 -> %ld"), smuffConfig.serialBaudrates[2]);
    Serial2.end();
    delay(150);
    Serial2.begin(smuffConfig.serialBaudrates[2]);
    delay(250);
    //__debugS(PSTR("DONE init SERIAL2"));
  }
  if (CAN_USE_SERIAL3 && smuffConfig.serialBaudrates[3] != 115200 && smuffConfig.serialBaudrates[3] >= 4800 && smuffConfig.serialBaudrates[3] <= 230400)
  {
    //__debugS(PSTR("End SERIAL3 -> %ld"), smuffConfig.serialBaudrates[3]);
    Serial3.end();
    delay(150);
    Serial3.begin(smuffConfig.serialBaudrates[3]);
    delay(250);
    //__debugS(PSTR("DONE init SERIAL3"));
  }
  //__debugS(PSTR("DONE init SERIAL"));
}

void setupSwSerial0()
{
#if defined(USE_SW_SERIAL)
  swSer0.begin(TMC_BAUDRATE);
#endif
}

void setupRelay()
{
  if (RELAY_PIN != -1)
  {
    pinMode(RELAY_PIN, OUTPUT);
    // if there's an external Feeder stepper defined (i.e. the 3D-Printer drives the Feeder),
    // switch on the external stepper by default. Otherwise, use the interal stepper.
    if (smuffConfig.extControlFeeder)
      switchFeederStepper(EXTERNAL);
    else
      switchFeederStepper(INTERNAL);
  }
}

void setupServos()
{

  // setup the Wiper servo
  if (SERVO1_PIN != -1)
  {
    servoWiper.setMaxCycles(smuffConfig.servoCycles1);
    servoWiper.setPulseWidthMinMax(smuffConfig.servoMinPwm, smuffConfig.servoMaxPwm);
    servoWiper.setTickRes(SERVO_RESOLUTION);
#if defined(__ESP32__)
    // we'll be using the internal ledcWrite for servo control on ESP32
    servoWiper.attach(SERVO1_PIN, false, SERVO_WIPER);
#elif defined(__STM32F1__)
    servoWiper.attach(SERVO1_PIN, true, SERVO_WIPER);
#else
    servoWiper.attach(SERVO1_PIN, true, SERVO_WIPER);
#endif
    uint8_t resetPos = 90, param;
    // try to find out the default reset position of the wiper servo from
    // within the wipe sequence
    if ((param = getParam(String(smuffConfig.wipeSequence), (char *)"P")) != -1)
    {
      resetPos = (uint8_t)param;
    }
    setServoPos(SERVO_WIPER, resetPos);
    disableServo(SERVO_WIPER);
  }

#if !defined(MULTISERVO)
  // setup the Lid servo (replaces the Revolver stepper motor)
  if (SERVO2_PIN != -1)
  {
    servoLid.setMaxCycles(smuffConfig.servoCycles2);
    servoLid.setPulseWidthMinMax(smuffConfig.servoMinPwm, smuffConfig.servoMaxPwm);
    servoLid.setTickRes(SERVO_RESOLUTION);

#if defined(__ESP32__)
    // we'll be using the internal ledcWrite for servo control on ESP32
    servoLid.attach(SERVO2_PIN, false, SERVO_LID);
#elif defined(__STM32F1__)
    servoLid.attach(SERVO2_PIN, true, SERVO_LID);
#else
    servoLid.attach(SERVO2_PIN, true, SERVO_LID);
#endif
    setServoLid(SERVO_OPEN);
  }
  // setup the Filament-Cutter servo if defined
  if (SERVO3_PIN != -1)
  {
    servoCutter.setMaxCycles(0);
    servoCutter.setPulseWidthMinMax(smuffConfig.servoMinPwm, smuffConfig.servoMaxPwm);
    servoCutter.setTickRes(SERVO_RESOLUTION);

#if defined(__ESP32__)
    // we'll be using the internal ledcWrite for servo control on ESP32
    servoCutter.attach(SERVO3_PIN, false, SERVO_CUTTER);
#elif defined(__STM32F1__)
    servoCutter.attach(SERVO3_PIN, true, SERVO_CUTTER);
#else
    servoCutter.attach(SERVO3_PIN, true, SERVO_CUTTER);
#endif
    setServoPos(SERVO_CUTTER, smuffConfig.cutterOpen);
  }
#else
  servoPwm.begin();
  servoPwm.setOscillatorFrequency(27000000);
  servoPwm.setPWMFreq(50);
  for (uint8_t i = 0; i < smuffConfig.toolCount; i++)
  {
    setServoPos(i + 10, servoPosClosed[i]);
    delay(400);
    setServoPos(i + 10, servoPosClosed[i] - SERVO_CLOSED_OFS);
  }
#endif
}

void setupHBridge()
{
#if defined(MOTOR_IN1_PIN) && defined(MOTOR_IN2_PIN)
  if (MOTOR_IN1_PIN != -1)
  {
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    digitalWrite(MOTOR_IN1_PIN, LOW);
  }
  if (MOTOR_IN2_PIN != -1)
  {
    pinMode(MOTOR_IN2_PIN, OUTPUT);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  }
#endif
}

void setupHeaterBed()
{
  // Please note: All the PWM pins on the SKR are not working as
  // expected. Maybe it's a common libmaple issue, maybe it's a
  // STM32 timer related thing or maybe it's the
  // hardware design of the board itself.
  // Can't tell for sure, need some more investigation.
  if (HEATER0_PIN != -1)
  {
    pinMode(HEATER0_PIN, OUTPUT);
  }
#if defined(__STM32F1__) || defined(__ESP32__)
  if (HEATBED_PIN != -1)
  {
#if defined(__STM32F1__)
    pinMode(HEATBED_PIN, PWM);
    analogWrite(HEATBED_PIN, 0);
#else
    pinMode(HEATBED_PIN, OUTPUT);
#endif
  }
#endif
}

void setupFan()
{
  if (FAN_PIN != -1)
  {
#ifdef __STM32F1__
    fan.attach(FAN_PIN, 0);
    fan.setTickRes(FAN_RESOLUTION);
    fan.setPulseWidthMax((uint16_t(((float)1/FAN_FREQUENCY)*1000000L)));
    fan.setBlipTimeout(FAN_BLIP_TIMEOUT);
#elif defined(__ESP32__)
    ledcSetup(FAN_CHANNEL, FAN_FREQ, 8);
    ledcAttachPin(FAN_PIN, FAN_CHANNEL);
    //__debugS(PSTR("DONE FAN PIN CONFIG"));
#else
    pinMode(FAN_PIN, OUTPUT);
#endif
    if (smuffConfig.fanSpeed >= 0 && smuffConfig.fanSpeed <= 100)
    {
#if defined(__ESP32__)
      ledcWrite(FAN_PIN, map(smuffConfig.fanSpeed, 0, 100, 0, 65535));
#elif defined(__STM32F1__)
      fan.setFanSpeed(smuffConfig.fanSpeed);
#else
      analogWrite(FAN_PIN, map(smuffConfig.fanSpeed, 0, 100, 0, 255));
#endif
    }
  }
  //__debugS(PSTR("DONE FAN init"));
}

void setupPortExpander()
{
#if defined(__ESP32__)
  extern ZPortExpander portEx;

  // init the PCF8574 port expander and set pin modes (0-5 OUTPUT, 6-7 INPUT)
  portEx.begin(PORT_EXPANDER_ADDRESS, false);
  for (uint8_t i = 0; i < 6; i++)
  {
    portEx.pinMode(i, OUTPUT);
    portEx.setPin(i);
  }
  portEx.pinMode(6, INPUT_PULLUP);
  portEx.pinMode(7, INPUT_PULLUP);
  portEx.resetPin(0);
//__debugS(PSTR("DONE PortExpander init"));
#endif
}

void setupI2C()
{
}

void setupBacklight()
{
#if defined(USE_RGB_BACKLIGHT) || defined(USE_FASTLED_BACKLIGHT)
  setBacklightIndex(smuffConfig.backlightColor); // turn on the LCD backlight according to the configured setting
#endif
}

void setupEncoder()
{
#if defined(USE_LEONERD_DISPLAY)
#if defined(USE_SW_TWI)
  //__debugS(PSTR("Before encoder.begin(&I2CBus)"));
  encoder.begin(&I2CBus);
#else
  //__debugS(PSTR("Before encoder.begin()"));
  encoder.begin();
#endif
  //__debugS(PSTR("After encoder.begin()"));
  uint8_t ver = encoder.queryVersion();
  //__debugS(PSTR("After encoder.queryVersion() = %d"), ver);
  if (ver < 2)
  {
    __debugS(PSTR("Warning: Encoder version mismatch! Version is: %d"), ver);
  }
  else
  {
    encoder.setKeyBeepMask(BEEP_NONE);
    encoder.setButtonHoldTime(120);
    /*
    // check whether or not the left soft button is mapped to GPIO 2 (RESET)
    uint8_t tmp = encoder.queryButtonMapping(LeftButton);
    const char P_Reprog[] PROGMEM = { "Reprogramming Encoder button mapping %s" };
    if(tmp != 1) {
      // if not, program the ecnoder accordingly
      __debugS(P_Reprog,"");
      encoder.setEepromValue(REG_EEPROM_BTN_MAPPING, 4);

    }
    tmp = encoder.queryButtonMappingPolarity(LeftButton);
    if(tmp != 1) {
      __debugS(P_Reprog,"polarity");
      encoder.setEepromValue(REG_EEPROM_BTN_POLARITY, 4);
    }
    */
  }
  encoder.setDoubleClickEnabled(true);
#else
  encoder.setDoubleClickEnabled(true); // enable doubleclick on the rotary encoder
#endif
}

void setupSteppers()
{

  uint16_t maxSpeed = translateSpeed(smuffConfig.maxSpeed[SELECTOR], SELECTOR);
  uint16_t accelSpeed = translateSpeed(smuffConfig.accelSpeed[SELECTOR], SELECTOR);
  steppers[SELECTOR] = ZStepper(SELECTOR, (char *)"Selector", X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, accelSpeed, maxSpeed);
  steppers[SELECTOR].setEndstop(X_END_PIN, smuffConfig.endstopTrg[SELECTOR], ZStepper::MIN);
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
    if (STALL_X_PIN != -1)
      attachInterrupt(STALL_X_PIN, isrStallDetectedX, FALLING);
#endif
  }
  if (smuffConfig.stepperMode[SELECTOR] == 0 && smuffConfig.ms3config[SELECTOR] > 0)
  {
#if defined(MS3_X) && MS3_X != -1
    pinMode(MS3_X, OUTPUT);
    digitalWrite(MS3_X, smuffConfig.ms3config[SELECTOR] == 1 ? LOW : HIGH);
#endif
  }

#if !defined(SMUFF_V5) && !defined(SMUFF_V6S)
  maxSpeed = translateSpeed(smuffConfig.maxSpeed[REVOLVER], REVOLVER);
  accelSpeed = translateSpeed(smuffConfig.accelSpeed[REVOLVER], REVOLVER);
  steppers[REVOLVER] = ZStepper(REVOLVER, (char *)"Revolver", Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, accelSpeed, maxSpeed);
  steppers[REVOLVER].setEndstop(Y_END_PIN, smuffConfig.endstopTrg[REVOLVER], ZStepper::ORBITAL);
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
    if (STALL_Y_PIN != -1)
      attachInterrupt(STALL_Y_PIN, isrStallDetectedY, FALLING);
#endif
  }
  if (smuffConfig.stepperMode[REVOLVER] == 0 && smuffConfig.ms3config[REVOLVER] > 0)
  {
#if defined(MS3_Y) && MS3_Y != -1
    pinMode(MS3_Y, OUTPUT);
    digitalWrite(MS3_Y, smuffConfig.ms3config[REVOLVER] == 1 ? LOW : HIGH);
#endif
  }

#else
  #if !defined(SMUFF_V6S)
  // we don't use the Revolver stepper but a servo instead, although
  // create a dummy instance
  steppers[REVOLVER] = ZStepper(REVOLVER, (char *)"Revolver", -1, -1, Y_ENABLE_PIN, 0, 0);
  #else
  // except for V6S, which uses a linear stepper instead of a servo
  maxSpeed = translateSpeed(smuffConfig.maxSpeed[REVOLVER], REVOLVER);
  accelSpeed = translateSpeed(smuffConfig.accelSpeed[REVOLVER], REVOLVER);
  steppers[REVOLVER] = ZStepper(REVOLVER, (char *)"Revolver", Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, accelSpeed, maxSpeed);
  steppers[REVOLVER].setEndstop(Y_END_PIN, smuffConfig.endstopTrg[REVOLVER], ZStepper::MIN);
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
    if (STALL_Y_PIN != -1)
      attachInterrupt(STALL_Y_PIN, isrStallDetectedY, FALLING);
#endif
  }
  if (smuffConfig.stepperMode[REVOLVER] == 0 && smuffConfig.ms3config[REVOLVER] > 0)
  {
#if defined(MS3_Y) && MS3_Y != -1
    pinMode(MS3_Y, OUTPUT);
    digitalWrite(MS3_Y, smuffConfig.ms3config[REVOLVER] == 1 ? LOW : HIGH);
#endif
  }
  __debugS(PSTR("Y-Stepper initialized for V6S"));
  #endif
#endif

  maxSpeed = translateSpeed(smuffConfig.maxSpeed[FEEDER], FEEDER);
  accelSpeed = translateSpeed(smuffConfig.accelSpeed[FEEDER], FEEDER);
  steppers[FEEDER] = ZStepper(FEEDER, (char *)"Feeder", Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, accelSpeed, maxSpeed);
  if (smuffConfig.useDuetLaser)
  {
    steppers[FEEDER].setEndstop(-1, smuffConfig.endstopTrg[FEEDER], ZStepper::MINMAX);
    steppers[FEEDER].endstopCheck = checkDuetEndstop;
  }
  else
    steppers[FEEDER].setEndstop(Z_END_PIN, smuffConfig.endstopTrg[FEEDER], ZStepper::MINMAX);
  if (Z_END2_PIN != -1)
    steppers[FEEDER].setEndstop(Z_END2_PIN, smuffConfig.endstopTrg[3], ZStepper::MINMAX, 2); // optional
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
    if (STALL_Z_PIN != -1)
      attachInterrupt(STALL_Z_PIN, isrStallDetectedZ, FALLING);
#endif
  }
  if (smuffConfig.stepperMode[FEEDER] == 0 && smuffConfig.ms3config[FEEDER] > 0)
  {
#if defined(MS3_Z) && MS3_Z != -1
    pinMode(MS3_Z, OUTPUT);
    digitalWrite(MS3_Z, smuffConfig.ms3config[FEEDER] == 1 ? LOW : HIGH);
#endif
  }

  for (uint8_t i = 0; i < NUM_STEPPERS; i++)
  {
    steppers[i].runAndWaitFunc = runAndWait;
    steppers[i].runNoWaitFunc = runNoWait;
    steppers[i].setEnabled(true);
  }

  //__debugS(PSTR("DONE init/enabling steppers"));
  for (uint8_t i = 0; i < MAX_TOOLS; i++)
  {
    swapTools[i] = i;
  }
  //__debugS(PSTR("DONE initializing swaps"));
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
    __debugS(PSTR("Driver for %c-axis skipped"), axis == FEEDER2 ? 'E' : 'X'+axis);
    return nullptr;
  }

  #if defined(TMC_SERIAL) && defined(TMC_HW_SERIAL)
  TMC2209Stepper *driver = new TMC2209Stepper(&TMC_SERIAL, rsense, drvrAdr);
  __debugS(PSTR("[initDriver] Driver for %c-axis created on address %d"), axis == FEEDER2 ? 'E' : 'X'+axis, drvrAdr);
  #else
  TMC2209Stepper *driver = new TMC2209Stepper(rx_pin, tx_pin, rsense, drvrAdr);
  #endif
  __debugS(PSTR("Driver for %c-axis initialized"), 'X'+axis);

  #if defined(HAS_TMC_SUPPORT) && !defined(TMC_HW_SERIAL)
  driver->beginSerial(TMC_SW_BAUDRATE);
  #endif
  driver->Rsense = rsense;

  bool intRsense = driver->internal_Rsense();
  if(!intRsense) {
    // Although the TMC datasheet says to set Rsense to internal, the following code will lead to the
    // stepper drivers not stepping anymore on SKR E3 boards!
    // This might be because it conflicts somehow with the OTP on the stepper chips soldered onto the E3 1.2 and E3 2.0.
    // It's a different picture for external (Pololu style) stepper drivers.
    // So for now, this code is disabled when compiling for E3 1.2 / 2.0 boards.
    #if !defined(__BRD_SKR_MINI_E3)
    __debugS(PSTR("[initDriver] Setting RSense to internal"));
    steppers[axis].setEnabled(false);
    driver->internal_Rsense(true);
    #endif
  }
  steppers[axis].setEnabled(true);
  intRsense = driver->internal_Rsense();
  __debugS(PSTR("[initDriver] RSense internal %d"), intRsense);

  driver->toff(toff);
  driver->blank_time(36);
  driver->I_scale_analog(false);    // use internal Vref; set to true for external Vref
  driver->rms_current(current);     // set current in mA
  driver->pdn_disable(true);        // PDN disabled for UART operation
  driver->mstep_reg_select(1);      // set microstepping
  driver->microsteps(msteps);
  __debugS(PSTR("[initDriver] Basic Init done for %c-Axis."), (axis==FEEDER2) ? 'E' : 'X' + axis);

  // setup StallGuard only if TMode is set to true
  // otherwise put it in SpreadCycle mode
  if (tmode)
  {
    #if defined(TMC_TYPE_2130)
    stall = (uint8_t)map(stall, 1, 255, -63, 63); // remap values for TMC2130 (not tested yet!)
    #endif
    setDriverSpreadCycle(driver, false, stall, csmin, csmax, csdown, toff); // set StealthChop (enable StallGuard)
  }
  else
  {
    setDriverSpreadCycle(driver, true, stall, csmin, csmax, csdown, toff); // set SpreadCycle (disable StallGuard)
  }
  isUsingTmc = true;
  return driver;
}

void setDriverSpreadCycle(TMC2209Stepper *driver, bool spread, uint8_t stallThrs, uint8_t csmin, uint8_t csmax, uint8_t csdown, uint8_t toff)
{
  if (spread)
  { // set SpreadCycle (disable StallGuard)
    driver->en_spreadCycle(true);
    driver->SGTHRS(0);
    driver->TCOOLTHRS(0);
    driver->TPWMTHRS(0);
    driver->pwm_autoscale(false);
    driver->pwm_autograd(false);
    driver->pwm_ofs(0);
  }
  else
  { // set StealthChop (enable StallGuard)
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
  if (!hwSerialInit)
  {
    __debugS(PSTR("[setupTMCDrivers] Initializing TMC HW-Serial with Baudrate %ld"), TMC_HW_BAUDRATE);
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
  //__debugS(PSTR("[setupTMCDrivers] initialized"));

#if defined(STALL_X_PIN)
  if (STALL_X_PIN != -1)
    pinMode(STALL_X_PIN, INPUT_PULLUP);
#endif
#if defined(STALL_Y_PIN)
  if (STALL_Y_PIN != -1)
    pinMode(STALL_Y_PIN, INPUT_PULLUP);
#endif
#if defined(STALL_Z_PIN)
  if (STALL_Z_PIN != -1)
    pinMode(STALL_Z_PIN, INPUT_PULLUP);
#endif
  //__debugS(PSTR("[setupTMCDrivers] DONE"));
}

#endif

void setupTimers()
{
#if defined(__STM32F1__)
  // *****
  // Attn:
  //    Steppers use:       TIMER2 CH1 (may corrupt TH0 readings)
  //    SW-Serial uses:     TIMER3 CH4 (see SoftwareSerialM library)
  //    I2C uses:           TIMER4 CH1 (using this timer/channel will kill I2C for some reason)
  //    Beeper uses:        TIMER5 CH3
  //    Servo uses:         TIMER5 CH1
  //    GP timer uses:      TIMER8 CH1 (general, encoder, servos)
  //    Heater0 uses:       TIMER8 CH3 (predefined by libmaple for PWM)
  //    Heatbed uses:       TIMER8 CH4 (predefined by libmaple for PWM)
  //
  // Warning: If you need to modify this assignment, be sure you know what you do!
  //          Swapping timers and/or channels may lead to a non functioning device or
  //          communication interrupts/breaks. Read the STM32F1 MCU spec. and check
  //          the libmaple library settings before you do so.
  // *****
  stepperTimer.setupTimer(Timer::TIMER2, Timer::CH1, STEPPER_PSC, 1); // prescaler set to STEPPER_PSC, timer will be calculated as needed
  gpTimer.setupTimer(Timer::TIMER8, Timer::CH1, 8, 0);                // prescaler set to 9 MHz, timer will be set to 50uS
  servoTimer.setupTimer(Timer::TIMER5, Timer::CH1, 8, 0);             // prescaler set to 9 MHz
#if !defined(USE_LEONERD_DISPLAY)
  setToneTimerChannel(Timer::TIMER5, Timer::CH3); // force TIMER5 / CH3 on STM32F1x for tone library
#endif
  nvic_irq_set_priority(NVIC_TIMER8_CC, 1);
  nvic_irq_set_priority(NVIC_TIMER2, 1);
  nvic_irq_set_priority(NVIC_TIMER4, 10);
  nvic_irq_set_priority(NVIC_TIMER5, 0);

#elif defined(__ESP32__)
  // *****
  // Attn:
  //    Steppers use:       TIMER1
  //    Servo uses:         TIMER3 (if it's setup to create its own timer)
  //    PortExpander uses:  TIMER3 (via general purpose timer)
  //    Encoder uses:       gpTimer
  // *****
  stepperTimer.setupTimer(Timer::TIMER1, STEPPER_PSC); // prescaler set to 4MHz, timer will be calculated as needed
  gpTimer.setupTimer(Timer::TIMER2, 80);               // 1us on 80MHz timer clock
#endif

  stepperTimer.setupHook(isrStepperHandler);      // setup the ISR for the steppers
  gpTimer.setupHook(isrGPTimerHandler);           // setup the ISR for rotary encoder, fan and general timers
  servoTimer.setupHook(isrServoTimerHandler);     // setup the ISR for servos

#if defined(__STM32F1__)
  gpTimer.setNextInterruptInterval(450);        // run general purpose (gp)timer on 50uS (STM32)
  servoTimer.setNextInterruptInterval(378);     // run servo timer on 42uS (STM32)
#elif defined(__ESP32__)
  gpTimer.setNextInterruptInterval(50);         // run general purpose (gp)timer on 50uS (ESP32)
#endif
  //__debugS(PSTR("DONE setup timers"));
}
