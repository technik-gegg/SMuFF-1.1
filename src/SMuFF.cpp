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

// Please notice: If you make any changes / additions here, you'll have to add
// an "external" declaration in SMuFF.h as well
#ifdef __BRD_I3_MINI
U8G2_ST7565_64128N_F_4W_HW_SPI  display(U8G2_R2, /* cs=*/ DSP_CS_PIN, /* dc=*/ DSP_DC_PIN, /* reset=*/ DSP_RESET_PIN);
#endif

#if defined(__BRD_SKR_MINI) || defined(__BRD_SKR_MINI_E3) || defined(__BRD_SKR_MINI_E3DIP)
  #ifdef USE_TWI_DISPLAY
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); 
  #elif USE_ANET_DISPLAY
  //
  //  Attn.: Instructions to modify the ANET display can be found here: https://www.thingiverse.com/thing:4009810
  //
  #pragma error "Before you use this display, you have to make some heavy modifications on the wiring for the display connector! Please check, then comment out this line."
  U8G2_ST7920_128X64_F_2ND_HW_SPI display(U8G2_R0, /* cs=*/ DSP_CS_PIN, /* reset=*/ U8X8_PIN_NONE); 
  // if the hardware SPI doesn't work, you may try software SPI instead
  //U8G2_ST7920_128X64_F_SW_SPI display(U8G2_R0, /* clock=*/ DSP_DC_PIN, /* data=*/ DSP_DATA_PIN, /* cs=*/ DSP_CS_PIN, /* reset=*/ U8X8_PIN_NONE);
  #elif USE_MINI12864_PANEL_V21 || USE_MINI12864_PANEL_V20
  U8G2_ST7567_JLX12864_F_2ND_4W_HW_SPI display(U8G2_R0, /* cs=*/ DSP_CS_PIN, /* dc=*/ DSP_DC_PIN, /* reset=*/ DSP_RESET_PIN);
  #elif USE_CREALITY_DISPLAY
    // works only with software SPI, hence it's remarkably slower than hardware SPI
    U8G2_ST7920_128X64_F_SW_SPI display(U8G2_R0, /* clock=*/ DSP_DC_PIN, /* data=*/ DSP_DATA_PIN, /* cc=*/ DSP_CS_PIN, /* reset=*/ DSP_RESET_PIN);
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
Stream*                 debugSerial = &SerialBT;  // decide which serial port to use for debug outputs 
#else
Stream*                 debugSerial = &Serial;    // decide which serial port to use for debug outputs 
#endif
HardwareSerial          Serial3(1);               // dummy declaration to keep the compiler happy, 
                                                  // won't be used though because of the CAN_USE_SERIAL3 definition
#elif defined(__STM32F1__)
  #if defined(__BRD_SKR_MINI_E3) || defined(__BRD_SKR_MINI_E3DIP)
Stream*                 debugSerial = &Serial2;
  #else
Stream*                 debugSerial = &Serial1;
  #endif
#else
Stream*                 debugSerial = &Serial;   
#endif

ZStepper                steppers[NUM_STEPPERS];
ZTimer                  stepperTimer;
ZTimer                  gpTimer;
ZServo                  servo;
ZServo                  servoRevolver;
ZFan                    fan;
DuetLaserSensor         duetLS;
ClickEncoder            encoder(ENCODER1_PIN, ENCODER2_PIN, ENCODER_BUTTON_PIN, 4);
#if defined(USE_SW_SERIAL)
SoftwareSerial          swSer0(SW_SERIAL_RX_PIN, SW_SERIAL_TX_PIN, false);
#endif
#if defined(__ESP32__)
ZPortExpander           portEx;
#endif
#if defined(__STM32F1__)
USBMassStorage          MassStorage;
USBCompositeSerial      CompositeSerial;
#endif

TMC2209Stepper* drivers[NUM_STEPPERS] { NULL, NULL, NULL };


String                          wirelessHostname = "";
volatile byte                   nextStepperFlag = 0;
volatile byte                   remainingSteppersFlag = 0;
volatile unsigned long          lastEncoderButtonTime = 0;
bool                            testMode = false;
int                             toolSelections[MAX_TOOLS]; 
volatile unsigned long          pwrSaveTime;
volatile bool                   isPwrSave = false;
volatile bool                   showMenu = false; 
volatile bool                   lastZEndstopState = false;
volatile bool                   lastZEndstop2State = false;
unsigned long                   endstopZ2HitCnt = 0;
static unsigned long            lastDisplayRefresh = 0;
static volatile unsigned long   generalCounter = 0;
static volatile unsigned long   tickCounter = 0;
volatile int                    bracketCnt = 0;
volatile int                    jsonPtr = 0;
//char                            jsonData[MAX_JSON];           // temporary buffer for json data coming from Duet3D (not used yet)
volatile bool                   enablePeriStat = false;       // enables sending periodical status information to serial ports
volatile bool                   processingSerial0;            // set when GCode is incoming on a serial port
volatile bool                   processingSerial1;
volatile bool                   processingSerial2;
volatile bool                   processingSerial3;
String                          serialBuffer0, serialBuffer1, serialBuffer2, serialBuffer3, serialBuffer9, traceSerial2;

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
  if(smuffConfig.stepDelay_X > 0)
    delayMicroseconds(smuffConfig.stepDelay_X);
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
  if(smuffConfig.stepDelay_Y > 0)
    delayMicroseconds(smuffConfig.stepDelay_Y);
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
  if(smuffConfig.stepDelay_Z > 0)
    delayMicroseconds(smuffConfig.stepDelay_Z);
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
  cli();
  tickCounter++;                      // increment tick counter
  if(tickCounter % 20) {              // after one ms
    generalCounter++;                 // increment milliseconds counter
    encoder.service();                // service the rotary encoder
    if(smuffConfig.useDuetLaser) {
      duetLS.service();               // service the Duet3D laser Sensor reader
    }
    #if defined(__HW_DEBUG__) && defined(DEBUG_PIN)
    // used for internal hardware debugging only - will produce a 500Hz signal on the output pin
    if(DEBUG_PIN != -1) digitalWrite(DEBUG_PIN, !digitalRead(DEBUG_PIN));
    #endif
  }
  // call the servos & fan interrupt routines also every 50uS
  isrServoTimerHandler();
  isrFanTimerHandler();
  sei();
}

void setup() {

  serialBuffer0.reserve(40);          // initialize serial comm. buffers
  serialBuffer1.reserve(40);
  serialBuffer2.reserve(40);
  serialBuffer3.reserve(40);
  traceSerial2.reserve(40);

  #if defined(__BRD_FYSETC_AIOII) || defined(__BRD_SKR_MINI_E3DIP) || defined(__BRD_SKR_MINI)
  // Disable JTAG for these boards!
  // On the FYSETC AIOII it's because of the display DSP_DC_PIN/DOG_A0 signal (PA15 / JTDI).
  // On the SKR MINI E3-DIP it's because of the buzzer signal (PA15 / JTDI).
  disableDebugPorts();
  #endif

  initUSB();                          // init the USB serial so it's being recognized by the Windows-PC
  delay(500);
  initFastLED();                      // init FastLED if configured
  testFastLED();                      // run a test sequence on FastLEDs
  initHwDebug();                      // init hardware debugging
  
  // Setup a fixed baudrate until the config file was read.
  // This baudrate is the default setting on the ESP32 while
  // booting up, so exceptions thrown can be shown in terminal app
  Serial.begin(115200);        
  if(CAN_USE_SERIAL1) Serial1.begin(115200);       
  if(CAN_USE_SERIAL2) Serial2.begin(115200);
  if(CAN_USE_SERIAL3) Serial3.begin(115200);
  //__debug(PSTR("[ setup() ]"));
  setupBuzzer();                      // setup buzzer before reading config
  setupDeviceName();                  // used for SerialBT on ESP32 only
  setupSerialBT();                    // used for debugging on ESP32 only
  setupDisplay();                     // setup display first in order to show error messages if neccessary
  readConfig();                       // read SMUFF.CFG from SD-Card
  setupSerial();                      // setup all components according to the values in SMUFF.CFG
  setupSteppers();
  setupTimers();
  setupServos();
  setupRelay();
  setupTMCDrivers();                  // setup TMC drivers if any were used
  setupSwSerial0();                   // used only for testing purposes
  setupBacklight();
  setupDuetLaserSensor();             // setup other peripherials
  setupEncoder();
  setupHeaterBed();
  setupFan();
  setupPortExpander();
  setupI2C();
  getStoredData();                    // read EEPROM.DAT from SD-Card; this call must happen after setupSteppers()
  readSequences();

  if(smuffConfig.homeAfterFeed) {
    moveHome(REVOLVER, false, false);
  }
  else {
    resetRevolver();
  }
  //__debug(PSTR("DONE reset Revolver"));
  
  sendStartResponse(0);       // send "start<CR><LF>" to USB serial interface
  if(CAN_USE_SERIAL1) 
    sendStartResponse(1);     // send "start<CR><LF>" to all serial interfaces allowed to use
  if(CAN_USE_SERIAL2 && smuffConfig.hasPanelDue != 2)
    sendStartResponse(2);
  if(CAN_USE_SERIAL3 && smuffConfig.hasPanelDue != 3)
    sendStartResponse(3);

  removeFirmwareBin();      // deletes the firmware.bin file to prevent re-flashing on each boot
  enablePeriStat = true;    // enable periodically sending status, if configured
  startupBeep();            // signal startup has finished
  pwrSaveTime = millis();   // init value for LCD screen timeout
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
    if(steppers[i].getMovementDone()) {
      remainingSteppersFlag &= ~_BV(i);
      //__debug(PSTR("ISR(): movement of %d done; Flag %d"), i, remainingSteppersFlag);
    } 
  }
  //__debug(PSTR("ISR(): %d"), remainingSteppersFlag);
  startStepperInterval();
}

void runNoWait(int index) {
  if(index != -1)
    remainingSteppersFlag |= _BV(index);
  startStepperInterval();
  //__debug(PSTR("Started stepper %d"), index);
}

void runAndWait(int index) {
  runNoWait(index);
  while(remainingSteppersFlag) {
    checkSerialPending(); // not a really nice solution but needed to check serials for "Abort" command in PMMU mode

#if defined(__STM32F1__) // || defined(__ESP32__)
    if((remainingSteppersFlag & _BV(FEEDER)) && !showMenu) {
      refreshStatus(false, true);
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
    } while(display.nextPage());
  }
  lastDisplayRefresh = millis();
}

void monitorTMC(int axis) {
  int temp;
  if(drivers[axis] != NULL) {
    if(drivers[axis]->otpw()) {
      __debug(PSTR("Driver %c reports 'Overtemperature Warning'"), 'X'+axis);
    }
    if(drivers[axis]->ot()) {
      if(drivers[axis]->t157())
        temp = 157;
      if(drivers[axis]->t150())
        temp = 150;
      if(drivers[axis]->t143())
        temp = 143;
      if(drivers[axis]->t120())
        temp = 120;
      __debug(PSTR("Driver %c reports 'Overtemperature' => %d°C"), 'X'+axis, temp);
    }
    if(drivers[axis]->ola()) {
      __debug(PSTR("Driver %c reports 'Open Phase A'"), 'X'+axis);
    }
    if(drivers[axis]->olb()) {
      __debug(PSTR("Driver %c reports 'Open Phase B'"), 'X'+axis);
    }
    if(drivers[axis]->s2ga()) {
      __debug(PSTR("Driver %c reports 'Short to GND on Phase A'"), 'X'+axis);
    }
    if(drivers[axis]->s2gb()) {
      __debug(PSTR("Driver %c reports 'Short to GND on Phase B'"), 'X'+axis);
    }
  }
}

void loop() {
  
  // Call periodical functions as the timeout has reached. 
  // Add your specific code there, if you need to have something 
  // managed periodically.
  // The main loop is the better choice for dispatching, since it'll
  // allow uninterrupted serial I/O.
  if(generalCounter % 20 == 0)    every20ms(); 
  if(generalCounter % 50 == 0)    every50ms(); 
  if(generalCounter % 100 == 0)   every100ms();
  if(generalCounter % 250 == 0)   every250ms();
  if(generalCounter % 500 == 0)   every500ms();
  if(generalCounter % 1000 == 0)  every1s();
  if(generalCounter % 2000 == 0)  every2s();
  if(generalCounter % 5000 == 0)  every5s();
  
  #if defined(USE_COMPOSITE_SERIAL)
  MassStorage.loop();
  #endif

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
      if(elapsed > 250)   // refresh display every 250ms
#else
      if(elapsed > 500)   // refresh display every 500ms
#endif  
      {
        refreshStatus(true, false);
      }
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

  if(smuffConfig.stepperStall[SELECTOR]) {
    monitorTMC(SELECTOR);
  }
  if(smuffConfig.stepperStall[REVOLVER]) {
    monitorTMC(REVOLVER);
  }
  if(smuffConfig.stepperStall[FEEDER]) {
    monitorTMC(FEEDER);
  }

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
  if(CAN_USE_SERIAL1)
    if(Serial1.available()) serialEvent1();
  if(CAN_USE_SERIAL2)
    if(Serial2.available()) serialEvent2();
  if(CAN_USE_SERIAL3)
    if(Serial3.available()) serialEvent3();
}
#endif

void checkSerialPending() {
  serialEventRun(); 
}

void resetSerialBuffer(int serial) {
  switch(serial) {
    case 0: serialBuffer0 = ""; break;
    case 1: serialBuffer1 = ""; break;
    case 2: serialBuffer2 = ""; break;
    case 3: serialBuffer3 = ""; break;
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
  
  if(processingSerial0)
    if(!smuffConfig.prusaMMU2)
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

void serialEvent2() {
  
  if(processingSerial2)
    if(!smuffConfig.prusaMMU2)
      return;
  
  while(Serial2.available()) {
    processingSerial2 = true;
    char in = (char)Serial2.read();
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
  
  if(processingSerial1)
    if(!smuffConfig.prusaMMU2)
      return;
  
  while(Serial1.available()) {
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
  
  if(processingSerial3)
    if(!smuffConfig.prusaMMU2)
      return;
  
  while(Serial3.available()) {
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


#ifdef __AVR__
// not really used anymore; Kept only to not break compatibility
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

