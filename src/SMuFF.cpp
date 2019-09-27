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

#ifdef __BRD_I3_MINI
U8G2_ST7565_64128N_F_4W_HW_SPI  display(U8G2_R2, /* cs=*/ DSP_CS_PIN, /* dc=*/ DSP_DC_PIN, /* reset=*/ DSP_RESET_PIN);
#endif
#ifdef __BRD_SKR_MINI
  #ifdef USE_TWI_DISPLAY
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); 
  #else
  U8G2_UC1701_MINI12864_1_2ND_4W_HW_SPI display(U8G2_R0, /* cs=*/ DSP_CS_PIN, /* dc=*/ DSP_DC_PIN, /* reset=*/ DSP_RESET_PIN);
  #endif
#endif

ZStepper                steppers[NUM_STEPPERS];
ZTimer                  stepperTimer;
ZTimer                  encoderTimer;
ZServo                  servo;
#define ENC_HALFSTEP 1
ClickEncoder            encoder(ENCODER1_PIN, ENCODER2_PIN, ENCODER_BUTTON_PIN, 4);
//CRGB                    leds[NUM_LEDS];

volatile byte           nextStepperFlag = 0;
volatile byte           remainingSteppersFlag = 0;
volatile unsigned long  lastEncoderButtonTime = 0;
bool                    testMode = false;
int                     toolSelections[MAX_TOOLS]; 
unsigned long           pwrSaveTime;
bool                    isPwrSave = false;
static bool             showMenu = false; 
static bool             lastZEndstopState = 0;
static unsigned long    lastDisplayRefresh = 0;

String serialBuffer0, serialBuffer2, serialBuffer9; 
String mainList;
String toolsList;
String offsetsList;
String swapList;
String traceSerial2;
char   tmp[128];

extern char _title[128];
extern int  swapTools[MAX_TOOLS];

void isrStepperHandler();       // forward declarations ... makes every compiler happy
void isrEncoderHandler();

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

void isrEncoderHandler() {
  encoder.service();
}

void setup() {

#ifdef __STM32F1__
  #ifndef USE_TWI_DISPLAY
  afio_remap(AFIO_REMAP_SPI1);  // remap SPI3 to SPI1 if a "normal" display is being used

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

  delay(1000);

  serialBuffer0.reserve(80);
  serialBuffer2.reserve(80);
  serialBuffer9.reserve(80);
  traceSerial2.reserve(80);
  
  /* No go. Maybe at a later stage
  FastLED.addLeds<LED_TYPE, NEOPIXEL_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  */

  Serial.begin(57600);        // set fixed baudrate until config file was read
  setupDisplay(); 
  readConfig();
  //__debug(PSTR("DONE reading CONFIG"));
  Serial.end();

  Serial.begin(smuffConfig.serial1Baudrate);
#ifdef __STM32F1__
  Serial1.begin(smuffConfig.serial2Baudrate);
  Serial3.begin(smuffConfig.serial2Baudrate);
#else
  Serial2.begin(smuffConfig.serial2Baudrate);
#endif
  
  setupSteppers();
  setupTimers();
  setupMainMenu();
  setupOffsetMenu();
  servo.attach(SERVO1_PIN);
  
  // must happen after setupSteppers()
  getStoredData();
  
  if(HEATER0_PIN != -1) {
    pinMode(HEATER0_PIN, OUTPUT); 
  }
#ifdef __STM32F1__
  if(HEATBED_PIN != -1) {
    pinMode(HEATBED_PIN, OUTPUT); 
  }
#endif
  
  if(FAN_PIN != -1){
#ifdef __STM32F1__
    pinMode(FAN_PIN, PWM);
#else
    pinMode(FAN_PIN, OUTPUT);
#endif      
    if(smuffConfig.fanSpeed > 0 && smuffConfig.fanSpeed <= 255) {
      analogWrite(FAN_PIN, smuffConfig.fanSpeed);
    }
  }

#ifndef __STM32F1__
  if(smuffConfig.i2cAddress != 0) {
    Wire.begin(smuffConfig.i2cAddress);
    Wire.onReceive(wireReceiveEvent);
  }
  //__debug(PSTR("DONE I2C init"));
 #endif
  
  //__debug(PSTR("DONE reset Revolver"));
  if(smuffConfig.homeAfterFeed) {
    moveHome(REVOLVER, false, false);
  }
  else {
    resetRevolver();
  }
  
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
  
}

void setupSteppers() {

  steppers[SELECTOR] = ZStepper(SELECTOR, (char*)"Selector", X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, smuffConfig.acceleration_X, smuffConfig.maxSpeed_X);
  steppers[SELECTOR].setEndstop(X_END_PIN, smuffConfig.endstopTrigger_X, ZStepper::MIN);
  steppers[SELECTOR].stepFunc = overrideStepX;
  steppers[SELECTOR].setMaxStepCount(smuffConfig.maxSteps_X);
  steppers[SELECTOR].setStepsPerMM(smuffConfig.stepsPerMM_X);
  steppers[SELECTOR].setInvertDir(smuffConfig.invertDir_X);

  steppers[REVOLVER] = ZStepper(REVOLVER, (char*)"Revolver", Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, smuffConfig.acceleration_Y, smuffConfig.maxSpeed_Y);
  steppers[REVOLVER].setEndstop(Y_END_PIN, smuffConfig.endstopTrigger_Y, ZStepper::ORBITAL);
  steppers[REVOLVER].stepFunc = overrideStepY;
  steppers[REVOLVER].setMaxStepCount(smuffConfig.stepsPerRevolution_Y);
  steppers[REVOLVER].endstopFunc = endstopYevent;
  steppers[REVOLVER].setInvertDir(smuffConfig.invertDir_Y);
  
  steppers[FEEDER] = ZStepper(FEEDER, (char*)"Feeder", Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, smuffConfig.acceleration_Z, smuffConfig.maxSpeed_Z);
  steppers[FEEDER].setEndstop(Z_END_PIN, smuffConfig.endstopTrigger_Z, ZStepper::MIN);
  steppers[FEEDER].stepFunc = overrideStepZ;
  steppers[FEEDER].setStepsPerMM(smuffConfig.stepsPerMM_Z);
  steppers[FEEDER].endstopFunc = endstopZevent;
  steppers[FEEDER].setInvertDir(smuffConfig.invertDir_Z);

  for(int i=0; i < NUM_STEPPERS; i++) {
      steppers[i].runAndWaitFunc = runAndWait;
      steppers[i].runNoWaitFunc = runNoWait;
      steppers[i].setEnabled(true);
  }
  //__debug(PSTR("DONE enabling steppers"));

  for(int i=0; i < MAX_TOOLS; i++) {
    swapTools[i] = i;
  }
}

void setupTimers() {
    // *****
  // Attn: Servo uses TIMER5
  // *****
#ifdef __BRD_I3_MINI
  stepperTimer.setupTimer(ZTimer::ZTIMER3, ZTimer::PRESCALER1);
  encoderTimer.setupTimer(ZTimer::ZTIMER4, ZTimer::PRESCALER1024);
  encoderTimer.setNextInterruptInterval(16);    // equals to 1ms on 16MHz CPU
#else
  stepperTimer.setupTimer(ZTimer::ZTIMER2, 3, 1);
  encoderTimer.setupTimer(ZTimer::ZTIMER1, 5);
  encoderTimer.setNextInterruptInterval(16);    // equals to 1ms on 72MHz CPU
#endif

  stepperTimer.setupTimerHook(isrStepperHandler);
  encoderTimer.setupTimerHook(isrEncoderHandler);
}

void setupToolsMenu() {
  char menu[256];
  sprintf_P(menu, P_MenuItemBack, NULL);
  memset(toolSelections, 0, sizeof(int)*MAX_TOOLS);
  int n = 0;
  for(int i=0; i< smuffConfig.toolCount; i++) {
    if(i == toolSelected)
      continue;
    toolSelections[n] = i;
    sprintf_P(tmp, P_ToolMenu, i);
    strcat(menu, tmp);
    strcat(menu, (char*)"\n");
    n++;
  }
  menu[strlen(menu)-1] = '\0';
  toolsList = String(menu);
}

void setupMainMenu() {
  char menu[256];
  sprintf_P(menu, P_MenuItemBack, NULL);
  strcat_P(menu, P_MenuItems);
  if(smuffConfig.prusaMMU2)
    strcat_P(menu, P_MenuItemsPMMU);
  mainList = String(menu);
  //__debug(PSTR("DONE setting Main menu"));
}

void setupOffsetMenu() {
  char menu[256];
  sprintf_P(menu, P_MenuItemBack, NULL);
  strcat_P(menu, P_OfsMenuItems);
  offsetsList = String(menu);
  //__debug(PSTR("DONE setting Offset menu"));
}

void setupSwapMenu() {
  char menu[256];
  sprintf_P(menu, P_MenuItemBack, NULL);
  sprintf_P(tmp, P_SwapReset, NULL);
  strcat(menu, tmp);
  for(int i=0; i< smuffConfig.toolCount; i++) {
    sprintf_P(tmp, P_SwapMenu, i, swapTools[i]);
    strcat(menu, tmp);
    strcat(menu, (char*)"\n");
  }
  menu[strlen(menu)-1] = '\0';
  swapList = String(menu);
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

void runNoWait(int index) {
  if(index != -1)
    remainingSteppersFlag |= _BV(index);
  startStepperInterval();
}

void runAndWait(int index) {
  runNoWait(index);
  while(remainingSteppersFlag) {
    checkSerialPending(); // not a really nice solution but needed to check serials for Abort command
    delayMicroseconds(1000);
  }
  //if(index==FEEDER) __debug(PSTR("Fed: %smm"), String(steppers[index].getStepsTakenMM()).c_str());
}

void loop() {
#ifdef __STM32F1__
  if(Serial1.available()) serialEvent1();
  if(Serial3.available()) serialEvent3();
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
  //__debug(PSTR("Mem: %d"), freeMemory());

  if(!checkUserMessage()) {
    if(!isPwrSave) {
      if(millis()-lastDisplayRefresh > 500) { // refresh display every 500ms
        display.firstPage();
        do {
          drawLogo();
          drawStatus();
        } while(display.nextPage());
        lastDisplayRefresh = millis();
      }
    }
  }
  else {
    if(millis()-userMessageTime > USER_MESSAGE_RESET*1000) {
      displayingUserMessage = false;
    }
  }

  int button = encoder.getButton();
  if(button == ClickEncoder::Button_e::Pressed && isPwrSave) {
    setPwrSave(0);
  }
  else {
    int turn = encoder.getValue();

    if(!showMenu && turn != 0) {
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
  
  delay(10);
  if((millis() - pwrSaveTime)/1000 >= (unsigned long)smuffConfig.powerSaveTimeout && !isPwrSave) {
    //PSTR("Power save mode after %d seconds (%d)"), (millis() - pwrSaveTime)/1000, smuffConfig.powerSaveTimeout);
    setPwrSave(1);
  }
}


void setPwrSave(int state) {
  display.setPowerSave(state);
  isPwrSave = state == 1;
  if(!isPwrSave) {
    delay(2000);
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

void showMainMenu() {

  bool stopMenu = false;
  unsigned int startTime = millis();
  uint8_t current_selection = 0;
  sprintf_P(_title, P_TitleMainMenu, NULL);
  do {
    resetAutoClose();
    while(checkUserMessage());
    current_selection = display.userInterfaceSelectionList(_title, current_selection, mainList.c_str());

    if(current_selection == 0)
      return;

    else {
      switch(current_selection) {
        case 1:
          stopMenu = true;
          break;
        
        case 2:
          moveHome(SELECTOR);
          moveHome(REVOLVER, true, false);
          startTime = millis();
          break;
        
        case 3:
          steppers[SELECTOR].setEnabled(false);
          steppers[REVOLVER].setEnabled(false);
          steppers[FEEDER].setEnabled(false);
          startTime = millis();
          break;

        case 4:
          showOffsetsMenu();
          break;
          
        case 5:
          if(smuffConfig.prusaMMU2)
            loadFilamentPMMU2();
          else
            loadFilament();
          startTime = millis();
          break;
        
        case 6:
          unloadFilament();
          startTime = millis();
          break;

        case 7:
          showSwapMenu();
          break;
        
        case 8:
          loadFilament();
          break;
      }
    }
    if (millis() - startTime > (unsigned long)smuffConfig.menuAutoClose * 1000) {
      stopMenu = true;
    }
  } while(!stopMenu);
}

void drawSwapTool(int from, int with) {
  sprintf_P(tmp, P_SwapToolDialog, from, with);
  drawUserMessage(String(tmp));
}

uint8_t swapTool(uint8_t index) {
  uint8_t ndx = 0;
  
  while(digitalRead(ENCODER_BUTTON_PIN) == LOW) {
    delay(20);
  }
  delay(250);
  
  drawSwapTool(index, ndx);

  while(1) {
    if(digitalRead(ENCODER_BUTTON_PIN) == LOW) {
      break;
    }
    int turn = encoder.getValue();
    if(turn == 0)
      continue;
    ndx += turn;

    if(ndx >= smuffConfig.toolCount)
      ndx = smuffConfig.toolCount-1;
    if(ndx == 255)
      ndx = smuffConfig.toolCount-1;
    drawSwapTool(index, ndx);
  }
  return ndx;
}

void showSwapMenu() {
  bool stopMenu = false;
  uint8_t current_selection = 0;
  sprintf_P(_title, P_TitleSwapMenu, NULL);
  
  do {
    setupSwapMenu();
    resetAutoClose();
    while(checkUserMessage());
    current_selection = display.userInterfaceSelectionList(_title, current_selection, swapList.c_str());

    if(current_selection == 0)
      return;
    else if(current_selection == 1) {
      stopMenu = true;
    }
    else if(current_selection == 2) {
      for(int i=0; i < MAX_TOOLS; i++) {
        swapTools[i] = i;
      }
    }
    else {
      uint8_t tool = swapTool(current_selection-3);
      uint8_t tmp = swapTools[current_selection-3];
      swapTools[current_selection-3] = tool;
      swapTools[tool] = tmp;
    }
  } while(!stopMenu);
  saveStore();
}


void showOffsetsMenu() {
  bool stopMenu = false;
  unsigned int startTime = millis();
  uint8_t current_selection = 0;
  sprintf_P(_title, P_TitleOffsetsMenu, NULL);
  
  do {
    resetAutoClose();
    while(checkUserMessage());
    current_selection = display.userInterfaceSelectionList(_title, current_selection, offsetsList.c_str());

    if(current_selection == 0)
      return;

    else {
      switch(current_selection) {
        case 1:
          stopMenu = true;
          break;
        
        case 2:
          changeOffset(SELECTOR);
          startTime = millis();
          break;
        
        case 3:
          changeOffset(REVOLVER);
          startTime = millis();
          break;
      }
    }
    if (millis() - startTime > (unsigned long)smuffConfig.menuAutoClose * 1000) {
      stopMenu = true;
    }
  } while(!stopMenu);  
}

void changeOffset(int index) {
  int steps = (smuffConfig.stepsPerRevolution_Y / 360);
  int turn;
  float stepsF = 0.1f;

  while(digitalRead(ENCODER_BUTTON_PIN) == LOW) {
    delay(20);
  }
  delay(250);
  
  moveHome(index);
  long pos = steppers[index].getStepPosition();
  float posF = steppers[index].getStepPositionMM();
  unsigned int curSpeed = steppers[index].getMaxSpeed();
  steppers[FEEDER].setMaxSpeed(500);
  encoder.setAccelerationEnabled(true);
  
  drawOffsetPosition(index);

  while(1) {
    if(digitalRead(ENCODER_BUTTON_PIN) == LOW)
      break;
    if((turn=encoder.getValue()) == 0)
      continue;
    if(turn == -1) {
      pos = -steps;
      posF = -stepsF;
    }
    else if(turn == 1) {
      pos = steps;
      posF = stepsF;
    }
    if(index == REVOLVER) {
      prepSteppingRel(REVOLVER, pos, true);
    }
    if(index == SELECTOR) {
      prepSteppingRelMillimeter(SELECTOR, posF, true);
    }
    runAndWait(index);
    drawOffsetPosition(index);
  }
  steppers[index].setMaxSpeed(curSpeed);
  encoder.setAccelerationEnabled(false);
}

void drawOffsetPosition(int index) {
  if(index == REVOLVER) {
    sprintf_P(tmp, PSTR("%s\n%d"), steppers[index].getDescriptor(), steppers[index].getStepPosition());
  }
  if(index == SELECTOR) {
    sprintf_P(tmp, PSTR("%s\n%s"), steppers[index].getDescriptor(), String(steppers[index].getStepPositionMM()).c_str());
  }
  drawUserMessage(String(tmp));
}

void showToolsMenu() {

  bool stopMenu = false;
  unsigned int startTime = millis();
  uint8_t current_selection = 0;
  sprintf_P(_title, P_TitleToolsMenu, NULL);
  do {
    setupToolsMenu();
    resetAutoClose();
    while(checkUserMessage());
    uint8_t startPos = toolSelected == 255 ? 0 : toolSelected+1;
    current_selection = display.userInterfaceSelectionList(_title, startPos, toolsList.c_str());

    if(current_selection <= 1)
      stopMenu = true;
    else {
      int tool = toolSelections[current_selection-2];
      if(!smuffConfig.duetDirect) {
        selectTool(tool);
      }
      else {
        selectTool(tool);
        // TODO: do tool change using Duet3D 
        // not yet possible due to Duet3D is being blocked waiting for endstop
        /*
        sprintf_P(tmp, PSTR("T%d\n"), tool); 
        Serial2.print(tmp);
        */
      }
      startTime = millis();
    }
    if (millis() - startTime > (unsigned long)smuffConfig.menuAutoClose * 1000) {
      stopMenu = true;
    }
  } while(!stopMenu);
}

void resetAutoClose() {
  lastEncoderButtonTime = millis();
}

bool checkAutoClose() {
  if (millis() - lastEncoderButtonTime >= (unsigned long)smuffConfig.menuAutoClose*1000) {
    return true;
  }
  return false;
}

#ifdef __STM32F1__
void serialEventRun() {
  if(Serial.available()) serialEvent();
  if(Serial1.available()) serialEvent1();
  if(Serial2.available()) serialEvent2();
  if(Serial3.available()) serialEvent3();
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
      buffer += in;
      break;
  }
}

void serialEvent() {
  while(Serial.available()) {
    char in = (char)Serial.read();
    if (in == '\n') {
      //__debug(PSTR("Received-0: %s"), serialBuffer0.c_str());
      parseGcode(serialBuffer0, 0);
      isQuote = false;
    }
    else {
      filterSerialInput(serialBuffer0, in);
    }
  }
}

void serialEvent2() {
  while(Serial2.available()) {
    char in = (char)Serial2.read();
    if (in == '\n') {
      //__debug(PSTR("Received-2: %s"), serialBuffer2.c_str());
      parseGcode(serialBuffer2, 2);
      isQuote = false;
    }
    else {
      filterSerialInput(serialBuffer2, in);
    }
  }
}

#ifdef __STM32F1__
void serialEvent1() {
  while(Serial1.available()) {
    char in = (char)Serial1.read();
    //Serial1.write(in);
    if (in == '\n') {
      //__debug(PSTR("Received-1: %s"), serialBuffer0.c_str());
      parseGcode(serialBuffer0, 1);
      isQuote = false;
    }
    else {
      filterSerialInput(serialBuffer0, in);
    }
  }
}

void serialEvent3() {
  while(Serial3.available()) {
    char in = (char)Serial3.read();
    //Serial3.write(in);
    if (in == '\n') {
      //__debug(PSTR("Received-3: %s"), serialBuffer2.c_str());
      parseGcode(serialBuffer2, 3);
      isQuote = false;
    }
    else {
      filterSerialInput(serialBuffer2, in);
    }
  }
}
#endif


#ifndef __STM32F1__
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

