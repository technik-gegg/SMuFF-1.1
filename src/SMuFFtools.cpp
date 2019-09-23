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

/*
 * Module containing helper functions
 */

#include <Arduino.h>
#include "SMuFF.h"
#include "SMuFFBitmaps.h"
#include "Config.h"
#include "ZTimerLib.h"
#include "ZStepperLib.h"
#include "ZServo.h"
#ifdef __STM32F1__
#include "libmaple/libmaple.h"
SPIClass SPI_3(3);
#define _tone(p, d, f)  playTone(p,d,f)
#define _noTone(p)      muteTone(p)
#else
#define _tone(p, d, f)  tone(p,d,f)
#define _noTone(p)      noTone(p)
#endif


extern void setToneTimerChannel(uint8_t ntimer, uint8_t channel);    // in tone library
#undef USE_BSSR

extern ZStepper       steppers[];
extern ZServo         servo;
extern char           tmp[128];

SMuFFConfig           smuffConfig;
int                   lastEncoderTurn = 0;
byte                  toolSelected = -1;
bool                  feederJammed = false;
PositionMode          positionMode = RELATIVE;
bool                  displayingUserMessage = false;
bool                  isAbortRequested = false;
unsigned int          userMessageTime = 0;
char                  _sel[128];
char                  _wait[128];
char                  _title[128];
char                  _msg1[256];
char                  _msg2[128];
char                  _btn[128];
int                   swapTools[MAX_TOOLS];


const char brand[] = VERSION_STRING;

void setupDisplay() {
#ifdef __STM32F1__
  setToneTimerChannel(4, 1);      // force TIMER4 / CH1 on STM32F1x for tone library
#endif
  display.begin(/*Select=*/ ENCODER_BUTTON_PIN,  /* menu_next_pin= */ U8X8_PIN_NONE, /* menu_prev_pin= */ U8X8_PIN_NONE, /* menu_home_pin= */ U8X8_PIN_NONE);
  display.enableUTF8Print();
  resetDisplay();
  display.setContrast(smuffConfig.lcdContrast);
}

void drawLogo() {
  //__debug(PSTR("drawLogo start..."));
  display.setBitmapMode(1);
  display.drawXBMP(0, 0, logo_width, logo_height, logo_bits);
  display.setFont(LOGO_FONT);
  display.setFontMode(0);
  display.setFontDirection(0);
  display.setDrawColor(1);
  display.setCursor(display.getDisplayWidth() - display.getStrWidth(brand) - 1, display.getDisplayHeight() - display.getMaxCharHeight());
  display.print(brand);
  //__debug(PSTR("drawLogo end..."));
}

void drawStatus() {
  //__debug(PSTR("drawStatus start..."));
  display.setFont(STATUS_FONT);
  display.setFontMode(0);
  display.setDrawColor(1);
  sprintf_P(tmp, P_CurrentTool);
  display.drawStr(display.getDisplayWidth() - display.getStrWidth(tmp) - 5, 14, tmp);
  display.drawStr(display.getDisplayWidth() - display.getStrWidth("X") - 5, 14, (toolSelected >= 0 && toolSelected < smuffConfig.toolCount) ? String(toolSelected).c_str() : "-");
  sprintf_P(tmp, P_Feed);
  display.drawStr(display.getDisplayWidth() - display.getStrWidth(tmp) - 5, 34, tmp);
  display.setFontMode(1);
  display.setFont(SMALL_FONT);
  display.setDrawColor(2);
  display.drawBox(0, display.getDisplayHeight()-display.getMaxCharHeight()+2, display.getDisplayWidth(), display.getMaxCharHeight());
  sprintf_P(_wait, parserBusy ? P_Busy : (smuffConfig.prusaMMU2) ? P_Pemu : P_Ready);
#ifdef __AVR__
  sprintf_P(tmp, PSTR("M:%d | %-4s | %-5s "), freeMemory(), traceSerial2.c_str(), _wait);
#else
  sprintf_P(tmp, PSTR("M:---- | %-4s | %-5s "), traceSerial2.c_str(), _wait);
#endif
  display.drawStr(1, display.getDisplayHeight(), tmp);
  display.setFontMode(0);
  display.setDrawColor(1);
  if(steppers[FEEDER].getMovementDone()) {
    display.setFont(ICONIC_FONT);
    display.drawGlyph(110, 38, feederEndstop() ? 0x41 : 0x42);
    display.setFont(BASE_FONT);
  }
  else {
    drawFeed();
  }
  //__debug(PSTR("drawStatus end..."));
}

void drawFeed() {
  display.setFont(STATUS_FONT);
  display.setFontMode(0);
  display.setDrawColor(1);
  sprintf_P(tmp, PSTR("%smm"), String(steppers[FEEDER].getStepsTakenMM()).c_str());
  display.drawStr(95, 38, tmp);
}

void resetDisplay() {
  display.clearDisplay();
  display.setFont(BASE_FONT);
  display.setFontMode(0);
  display.setDrawColor(1);
  //delay(1);
}

void drawSelectingMessage(int tool) {
  display.firstPage();
  do {
    resetDisplay();
    sprintf_P(_sel, P_Selecting);
    sprintf_P(_wait, P_Wait);
    if(*smuffConfig.materials[tool] != 0) {
      sprintf(tmp,"%s", smuffConfig.materials[tool]);
    }
    else {
      sprintf_P(tmp, "%s%d", P_Tool, tool);
    }
    display.drawStr((display.getDisplayWidth() - display.getStrWidth(_sel))/2, (display.getDisplayHeight() - display.getMaxCharHeight())/2-10, _sel);
    display.setFont(BASE_FONT_BIG);
    display.drawStr((display.getDisplayWidth() - display.getStrWidth(tmp))/2, (display.getDisplayHeight() - display.getMaxCharHeight())/2+9, tmp);
    display.setFont(BASE_FONT);
    display.drawStr((display.getDisplayWidth() - display.getStrWidth(_wait))/2, (display.getDisplayHeight() - display.getMaxCharHeight())/2 + display.getMaxCharHeight()+10, _wait);
  } while(display.nextPage());
}

int splitStringLines(char lines[][MAX_LINE_LENGTH], String message) {

  for(int i=0; i < MAX_LINES; i++) {
    memset(&lines[i], 0, MAX_LINE_LENGTH);
  }

  int pos2 = (message.indexOf('\n') == -1) ? message.length() : message.indexOf('\n');
  int ln = 0;
  
  do {
    int len = (pos2 > MAX_LINE_LENGTH-1) ? MAX_LINE_LENGTH-1 : pos2+1;
    message.substring(0, pos2).toCharArray(lines[ln], len);
    if(ln >= MAX_LINES)
      break;
    ln++;
    message = message.substring(len);
    pos2 = message.indexOf('\n');
    if(pos2 == -1)
      break;
  } while(1);
  
  if(message.length()>0 && ln < MAX_LINES) {
    int len = message.length()+1;
    message.substring(0).toCharArray(lines[ln++], len);
  }
  return ln;
}

void drawUserMessage(String message) {

  char lines[MAX_LINES][MAX_LINE_LENGTH];
  int lineCnt = splitStringLines(lines, message);
  if(isPwrSave) {
    setPwrSave(0);
  }
  display.setDrawColor(0);
  display.drawBox(1, 1, display.getDisplayWidth()-2, display.getDisplayHeight()-2);
  display.setDrawColor(1);
  display.drawFrame(0, 0, display.getDisplayWidth(), display.getDisplayHeight());
  display.firstPage();
  do {
    display.setFont(BASE_FONT_BIG);
    int y = (display.getDisplayHeight()-(lineCnt-1)*display.getMaxCharHeight())/2;
    display.firstPage();
    do {
      for(int i=0; i< lineCnt; i++) {
        display.drawStr((display.getDisplayWidth() - display.getStrWidth(lines[i]))/2, y, lines[i]);
        y += display.getMaxCharHeight();
      }
    } while(display.nextPage());
    display.setFont(BASE_FONT);
  } while(display.nextPage());  
  displayingUserMessage  = true;
  userMessageTime = millis();
}


void drawSDStatus(int stat) {
  resetDisplay();
  switch(stat) {
    case SD_ERR_INIT:
      sprintf_P(tmp, P_SD_InitError);
      longBeep(2);
      break;
    case SD_ERR_NOCONFIG:
      sprintf_P(tmp, P_SD_NoConfig);
      longBeep(1);
      break;
    case SD_READING_CONFIG:
      sprintf_P(tmp, P_SD_ReadingConfig);
      break;
  }
  display.firstPage();
  do {
    drawLogo();
    display.setCursor((display.getDisplayWidth() - display.getStrWidth(tmp))/2, display.getDisplayHeight());
    display.print(tmp);
  } while(display.nextPage());
}

bool selectorEndstop() {
  return steppers[SELECTOR].getEndstopHit();
}

bool revolverEndstop() {
  return steppers[REVOLVER].getEndstopHit();
}

bool feederEndstop() {
  return steppers[FEEDER].getEndstopHit();
}

void setAbortRequested(bool state) {
  steppers[FEEDER].setAbort(state);  // stop any ongoing stepper movements
}

uint8_t u8x8_GetMenuEvent(u8x8_t *u8x8)
{
  int stat = 0;
  int button = digitalRead(ENCODER_BUTTON_PIN);
  int turn = encoder.getValue();
  
  //if(turn != 0) __debug(PSTR("Encoder: %d"), turn);

  if (button == LOW) {
    delay(20);
    button = digitalRead(ENCODER_BUTTON_PIN);
    if (button == LOW && u8x8->debounce_state == HIGH) {
      stat = U8X8_MSG_GPIO_MENU_SELECT;
      resetAutoClose();
      turn = encoder.getValue();
    }
  }
  else {
    if (turn != 0) {
      resetAutoClose();
      switch (turn)
      {
        case 1:
          stat = U8X8_MSG_GPIO_MENU_NEXT;
          break;
        case -1:
          stat =  U8X8_MSG_GPIO_MENU_PREV;
          break;
      }
    }
  }
  u8x8->debounce_state = button;
  checkSerialPending();
  if(checkAutoClose()) {
    stat = U8X8_MSG_GPIO_MENU_HOME;
  }
  return stat;
}

#ifdef __STM32F1__
  #ifndef USE_TWI_DISPLAY

/* =========================================
ATTENTION:
In order to make this function work, you have to add 
these lines to your U8G2 library source code 
file U8x8lib.cpp:

#if defined(__GNUC__) && !defined(__CYGWIN__)
# pragma weak  u8x8_byte_arduino_2nd_hw_spi
#endif

in front of function:
extern "C" uint8_t u8x8_byte_arduino_2nd_hw_spi(...)

Otherwise the linker will throw an error message.

This function is a copy of the above mentioned
and it sends data to SPI3 instead of SPI1.
=========================================== */
uint8_t u8x8_byte_arduino_2nd_hw_spi(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
  uint8_t *data;
  uint8_t internal_spi_mode;
 
  switch(msg)
  {
    case U8X8_MSG_BYTE_SEND:
      
      // 1.6.5 offers a block transfer, but the problem is, that the
      // buffer is overwritten with the incoming data
      // so it can not be used...
      // SPI.transfer((uint8_t *)arg_ptr, arg_int);
      
      data = (uint8_t *)arg_ptr;
      while( arg_int > 0 )
      {
        SPI_3.transfer((uint8_t)*data);
        data++;
        arg_int--;
      }
  
      break;
    case U8X8_MSG_BYTE_INIT:
      if ( u8x8->bus_clock == 0 ) 	/* issue 769 */
      	u8x8->bus_clock = u8x8->display_info->sck_clock_hz;
      /* disable chipselect */
      u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
      /* no wait required here */
      
      /* for SPI1: setup correct level of the clock signal */
      // removed, use SPI.begin() instead: pinMode(11, OUTPUT);
      // removed, use SPI.begin() instead: pinMode(13, OUTPUT);
      // removed, use SPI.begin() instead: digitalWrite(13, u8x8_GetSPIClockPhase(u8x8));
      
      /* setup hardware with SPI.begin() instead of previous digitalWrite() and pinMode() calls */
      SPI_3.begin();	

      break;
      
    case U8X8_MSG_BYTE_SET_DC:
      u8x8_gpio_SetDC(u8x8, arg_int);
      break;
      
    case U8X8_MSG_BYTE_START_TRANSFER:
      /* SPI1 mode has to be mapped to the mode of the current controller, at least Uno, Due, 101 have different SPI_MODEx values */
      internal_spi_mode =  0;
      switch(u8x8->display_info->spi_mode)
      {
        case 0: internal_spi_mode = SPI_MODE0; break;
        case 1: internal_spi_mode = SPI_MODE1; break;
        case 2: internal_spi_mode = SPI_MODE2; break;
        case 3: internal_spi_mode = SPI_MODE3; break;
      }
      
#if ARDUINO >= 10600
      SPI_3.beginTransaction(SPISettings(u8x8->bus_clock, MSBFIRST, internal_spi_mode));
#else
      SPI_3.begin();
      
      if ( u8x8->display_info->sck_pulse_width_ns < 70 )
      	SPI3.setClockDivider( SPI_CLOCK_DIV2 );
      else if ( u8x8->display_info->sck_pulse_width_ns < 140 )
	      SPI3.setClockDivider( SPI_CLOCK_DIV4 );
      else
	      SPI3.setClockDivider( SPI_CLOCK_DIV8 );
      SPI3.setDataMode(internal_spi_mode);
      SPI3.setBitOrder(MSBFIRST);
#endif
      
      u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_enable_level);  
      u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_NANO, u8x8->display_info->post_chip_enable_wait_ns, NULL);
      break;
      
    case U8X8_MSG_BYTE_END_TRANSFER:      
      u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_NANO, u8x8->display_info->pre_chip_disable_wait_ns, NULL);
      u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);

#if ARDUINO >= 10600
      SPI_3.endTransaction();
#else
      SPI_3.end();
#endif

      break;
    default:
      return 0;
  }
  
  return 1;
}
  #endif
#endif


bool moveHome(int index, bool showMessage, bool checkFeeder) {
  if(!steppers[index].getEnabled())
    steppers[index].setEnabled(true);

  if(feederJammed) {
    beep(4);
    return false;
  }
  parserBusy = true;
  if (checkFeeder && feederEndstop()) {
    if (showMessage) {
      if (!showFeederLoadedMessage()) {
        parserBusy = false;
        return false;
      }
    }
    else {
      if (feederEndstop()) {
        unloadFilament();
      }
    }
  }
  //__debug(PSTR("Stepper home"));
  steppers[index].home();
  //__debug(PSTR("DONE Stepper home"));
  if (index == SELECTOR) {
    toolSelected = -1;
  }
  long pos = steppers[index].getStepPosition();
  if (index == SELECTOR || index == REVOLVER) {
    dataStore.tool = toolSelected;
  }
  dataStore.stepperPos[index] = pos;
  saveStore();

  parserBusy = false;
  return true;
}

bool showFeederLoadedMessage() {
  bool state = false;
  lastEncoderButtonTime = millis();
  beep(1);
  int button = showDialog(P_TitleWarning, P_FeederLoaded, P_AskUnload, P_YesNoButtons);
  if (button == 1) {
    drawStatus();
    unloadFilament();
    state = true;
  }
  display.clearDisplay();
  return state;
}

bool showFeederLoadMessage() {
  bool state = false;
  lastEncoderButtonTime = millis();
  beep(1);
  int button = showDialog(P_TitleSelected, P_SelectedTool, P_AskLoad, P_YesNoButtons);
  if (button == 1) {
    drawStatus();
    if(smuffConfig.prusaMMU2)
      loadFilamentPMMU2();
    else
      loadFilament();
    state = true;
  }
  display.clearDisplay();
  return state;
}

bool showFeederFailedMessage(int state) {
  lastEncoderButtonTime = millis();
  beep(3);
  showDialog(P_TitleWarning, state == 1 ? P_CantLoad : P_CantUnload, P_CheckUnit, P_OkButtonOnly);
  display.clearDisplay();
  return false;
}

int showDialog(PGM_P title, PGM_P message, PGM_P addMessage, PGM_P buttons) {
  if(isPwrSave) {
    setPwrSave(0);
  }
  sprintf_P(_title, title);
  sprintf_P(_msg1, message);
  sprintf_P(_msg2, addMessage);
  sprintf_P(_btn, buttons);
  return display.userInterfaceMessage(_title, _msg1, _msg2, _btn);
}

void signalNoTool() {
  userBeep();
  sprintf_P(_msg1, P_NoTool);
  strcat_P(_msg1, P_Aborting);
  drawUserMessage(_msg1);
}

void positionRevolver() {
  if(smuffConfig.resetBeforeFeed_Y)
    resetRevolver();
  else {
    long newPos = 0;
    if(smuffConfig.homeAfterFeed) {
      newPos = smuffConfig.firstRevolverOffset + (toolSelected *smuffConfig.revolverSpacing);
    }
    else {
      newPos = steppers[REVOLVER].getStepPosition() - (smuffConfig.firstRevolverOffset + (toolSelected *smuffConfig.revolverSpacing));
      if(newPos < 0)
        newPos = smuffConfig.stepsPerRevolution_Y + newPos;
      else if(newPos > smuffConfig.stepsPerRevolution_Y)
        newPos = smuffConfig.stepsPerRevolution_Y + newPos;
    }
    prepSteppingAbs(REVOLVER, newPos, true);
    remainingSteppersFlag |= _BV(REVOLVER);
    runAndWait(-1);
  }
}

bool feedToEndstop(bool showMessage) {   
  // enable steppers if they were turned off
  if(!steppers[FEEDER].getEnabled())
    steppers[FEEDER].setEnabled(true);

  // don't let interrupt feed to endstop
  steppers[FEEDER].setIgnoreAbort(true);

  positionRevolver();

  unsigned int curSpeed = steppers[FEEDER].getMaxSpeed();
  steppers[FEEDER].setMaxSpeed(smuffConfig.insertSpeed_Z);

  int n = 100;
  while (!feederEndstop()) {
    prepSteppingRelMillimeter(FEEDER, 2.5, true);
    runAndWait(FEEDER);
    if (n == 50) {
      resetRevolver();
      prepSteppingRelMillimeter(FEEDER, -smuffConfig.selectorDistance, true);
      runAndWait(FEEDER);
    }
    if (n <= 0) {
      if (showMessage)
        showFeederFailedMessage(1);
      steppers[FEEDER].setMaxSpeed(curSpeed);
      feederJammed = true;
      parserBusy = false;
      //__debug(PSTR("Load status: Abort: %d IgnoreAbort: %d Jammed:%d"), steppers[FEEDER].getAbort(), steppers[FEEDER].getIgnoreAbort(), feederJammed);
      steppers[FEEDER].setIgnoreAbort(false);
      return false;
    }
    n--;
  }
  steppers[FEEDER].setIgnoreAbort(false);
  steppers[FEEDER].setMaxSpeed(curSpeed);
  feederJammed = false;
  return true;
}

void feedToNozzle() {
  
  if(smuffConfig.prusaMMU2 && smuffConfig.enableChunks) {
    // prepare to feed full speed in chunks
    float bLen = smuffConfig.bowdenLength;
    float len = bLen/smuffConfig.feedChunks;
    for(int i=0; i<smuffConfig.feedChunks; i++) {
      prepSteppingRelMillimeter(FEEDER, len, true);
      runAndWait(FEEDER);
    }
  }
  else {
    // prepare 95% to feed full speed
    prepSteppingRelMillimeter(FEEDER, smuffConfig.bowdenLength*.95, true);
    runAndWait(FEEDER);
    // rest of it feed slowly
    steppers[FEEDER].setMaxSpeed(smuffConfig.insertSpeed_Z);
    prepSteppingRelMillimeter(FEEDER, smuffConfig.bowdenLength*.05, true);
    runAndWait(FEEDER);
  }
}

bool loadFilament(bool showMessage) {
  if (toolSelected == 255) {
    signalNoTool();
    return false;
  }
  if(smuffConfig.externalControl_Z) {
    positionRevolver();
    signalLoadFilament();
    return true;
  }

  parserBusy = true;
  unsigned int curSpeed = steppers[FEEDER].getMaxSpeed();
    // move filament until it hits the feeder endstop
  if(!feedToEndstop(showMessage))
    return false;

  steppers[FEEDER].setStepsTaken(0);
  // move filament until it gets to the nozzle
  feedToNozzle();

  if(smuffConfig.reinforceLength > 0 && !steppers[FEEDER].getAbort()) {
    resetRevolver();
    prepSteppingRelMillimeter(FEEDER, smuffConfig.reinforceLength, true);
    runAndWait(FEEDER);
  }
  
  steppers[FEEDER].setMaxSpeed(curSpeed);
  dataStore.stepperPos[FEEDER] = steppers[FEEDER].getStepPosition(); 
  saveStore();

  if(smuffConfig.homeAfterFeed)
    steppers[REVOLVER].home();
  steppers[FEEDER].setAbort(false);

  parserBusy = false;
  return true;
}

/*
  This method is used to feed the filament Prusa style (L command on MMU2).
  If first feeds the filament until the endstop is hit, then 
  it pulls it back again.
*/
bool loadFilamentPMMU2(bool showMessage) {
  if (toolSelected == 255) {
    signalNoTool();
    return false;
  }
  if(smuffConfig.externalControl_Z) {
    positionRevolver();
    signalLoadFilament();
    return true;
  }
  parserBusy = true;
  unsigned int curSpeed = steppers[FEEDER].getMaxSpeed();
  // move filament until it hits the feeder endstop
  if(!feedToEndstop(showMessage))
    return false;
  steppers[FEEDER].setStepsTaken(0);
  // inhibit interrupts at this step
  steppers[FEEDER].setIgnoreAbort(true);
  // now pull it back again
  prepSteppingRelMillimeter(FEEDER, -smuffConfig.selectorDistance, true);
  runAndWait(FEEDER);

  if(smuffConfig.reinforceLength > 0 && !steppers[FEEDER].getAbort()) {
    resetRevolver();
    prepSteppingRelMillimeter(FEEDER, smuffConfig.reinforceLength, true);
    runAndWait(FEEDER);
  }
  steppers[FEEDER].setIgnoreAbort(false);
  
  steppers[FEEDER].setMaxSpeed(curSpeed);
  dataStore.stepperPos[FEEDER] = steppers[FEEDER].getStepPosition();
  saveStore(); 

  if(smuffConfig.homeAfterFeed)
    steppers[REVOLVER].home();
  steppers[FEEDER].setAbort(false);

  parserBusy = false;
  return true;
}

void unloadFromNozzle() {
  if(smuffConfig.prusaMMU2 && smuffConfig.enableChunks) {
    // prepare to unfeed 3 times the bowden length full speed in chunks
    float bLen = -smuffConfig.bowdenLength*3;
    float len = bLen/smuffConfig.feedChunks;
    for(int i=0; i<smuffConfig.feedChunks; i++) {
      prepSteppingRelMillimeter(FEEDER, len);
      runAndWait(FEEDER);
    }
  }
  else {
    prepSteppingRelMillimeter(FEEDER, -(smuffConfig.bowdenLength*3));
    runAndWait(FEEDER);
  }
}

bool unloadFilament() {
  if (toolSelected == 255) {
    signalNoTool();
    return false;
  }
  if(smuffConfig.externalControl_Z) {
    positionRevolver();
    signalUnloadFilament();
    return true;
  }
  steppers[FEEDER].setStepsTaken(0);
  parserBusy = true;
  if(!steppers[FEEDER].getEnabled())
    steppers[FEEDER].setEnabled(true);

  positionRevolver();  

  unsigned int curSpeed = steppers[FEEDER].getMaxSpeed();
  steppers[FEEDER].setEndstopState(!steppers[FEEDER].getEndstopState());
  if(smuffConfig.unloadRetract != 0) {
    prepSteppingRelMillimeter(FEEDER, smuffConfig.unloadRetract);
    runAndWait(FEEDER);
    if(smuffConfig.unloadPushback != 0) {
      steppers[FEEDER].setMaxSpeed(smuffConfig.insertSpeed_Z);
      prepSteppingRelMillimeter(FEEDER, smuffConfig.unloadPushback);
      runAndWait(FEEDER);
      delay(smuffConfig.pushbackDelay*1000);
      steppers[FEEDER].setMaxSpeed(curSpeed);
    }
  }

  unloadFromNozzle();

  // only if the unload hasn't been aborted yet, unload from Selector as well
  if(steppers[FEEDER].getAbort() == false) {
    steppers[FEEDER].setMaxSpeed(smuffConfig.insertSpeed_Z);
    steppers[FEEDER].setIgnoreAbort(true);
    int n = 200;
    do {
      prepSteppingRelMillimeter(FEEDER, -smuffConfig.selectorDistance, true);
      runAndWait(FEEDER);
      if(!feederEndstop()) {
        if((n > 0 && n < 200) && n % 50 == 0) {
          resetRevolver();
          prepSteppingRelMillimeter(FEEDER, 15.0, true);
          runAndWait(FEEDER);
        }
        if (n <= 0) {
          showFeederFailedMessage(0);
          steppers[FEEDER].setMaxSpeed(curSpeed);
          feederJammed = true;
          parserBusy = false;
          steppers[FEEDER].setIgnoreAbort(false);
          return false;
        }
      }
      n--;
    } while (!feederEndstop());
  }
  feederJammed = false;
  steppers[FEEDER].setIgnoreAbort(false);
  steppers[FEEDER].setMaxSpeed(curSpeed);
  steppers[FEEDER].setEndstopState(!steppers[FEEDER].getEndstopState());
  steppers[FEEDER].setStepPosition(0);
  steppers[FEEDER].setAbort(false);

  dataStore.stepperPos[FEEDER] = steppers[FEEDER].getStepPosition();
  saveStore();

  steppers[FEEDER].setAbort(false);
  if(smuffConfig.homeAfterFeed)
    steppers[REVOLVER].home();

  parserBusy = false;
  return true;
}

bool selectTool(int ndx, bool showMessage) {

  ndx = swapTools[ndx];
  if(feederJammed) {
    beep(4);
    sprintf_P(_msg1, P_FeederJammed);
    strcat_P(_msg1, P_Aborting);
    drawUserMessage(_msg1);
    feederJammed = false;
    return false;
  }
  signalSelectorBusy();
  if(toolSelected == ndx) { // tool is the one we already have selected, do nothing
    if(!smuffConfig.externalControl_Z) {
      userBeep();
      sprintf_P(_msg1, P_ToolAlreadySet);
      drawUserMessage(_msg1);
    }
    if(smuffConfig.externalControl_Z) {
      signalSelectorReady();
    }
    return true;
  }
  if(!steppers[SELECTOR].getEnabled())
    steppers[SELECTOR].setEnabled(true);
  
  if (showMessage) {
    while(feederEndstop()) {
      if (!showFeederLoadedMessage())
        return false;
    }
  }
  else {
    if (!smuffConfig.externalControl_Z && feederEndstop()) {
      unloadFilament();
    }
    else if (smuffConfig.externalControl_Z && feederEndstop()) {
      // TODO: Signal Duet3D to retract 2mm
      beep(4);
      char buf[] = {'Y'};
      while(feederEndstop()) {
        M18("M18", buf, 0);   // motors off
        moveHome(REVOLVER, false, false);   // home Revolver
        showFeederFailedMessage(0);
        if(smuffConfig.unloadCommand != NULL && strlen(smuffConfig.unloadCommand) > 0) {
          Serial2.print(smuffConfig.unloadCommand);
          Serial2.print("\n");
          __debug(PSTR("Feeder jammed, sent unload command '%s'\n"), smuffConfig.unloadCommand);
        }
      }
    }
  }
  //__debug(PSTR("Selecting tool: %d"), ndx);
  parserBusy = true;
  drawSelectingMessage(ndx);
  prepSteppingAbsMillimeter(SELECTOR, smuffConfig.firstToolOffset + (ndx * smuffConfig.toolSpacing));
  remainingSteppersFlag |= _BV(SELECTOR);
  if(!smuffConfig.resetBeforeFeed_Y) {
    prepSteppingAbs(REVOLVER, smuffConfig.firstRevolverOffset + (ndx *smuffConfig.revolverSpacing), true);
    remainingSteppersFlag |= _BV(REVOLVER);
  }
  runAndWait(-1);
  toolSelected = ndx;

  dataStore.tool = toolSelected;
  dataStore.stepperPos[SELECTOR] = steppers[SELECTOR].getStepPosition();
  dataStore.stepperPos[REVOLVER] = steppers[REVOLVER].getStepPosition();
  dataStore.stepperPos[FEEDER] = steppers[FEEDER].getStepPosition();
  saveStore();

  if (!smuffConfig.externalControl_Z && showMessage) {
    showFeederLoadMessage();
  }
  if(smuffConfig.externalControl_Z) {
    resetRevolver();
    signalSelectorReady();
  }
  if(testMode) {
    Serial2.print("T");
    Serial2.println(ndx); 
  }
  parserBusy = false;
  return true;
}

void resetRevolver() {
  //__debug(PSTR("resetting revolver"));
  moveHome(REVOLVER, false, false);
  //__debug(PSTR("DONE resetting revolver"));
  if (toolSelected >=0 && toolSelected <= smuffConfig.toolCount-1) {
    prepSteppingAbs(REVOLVER, smuffConfig.firstRevolverOffset + (toolSelected*smuffConfig.revolverSpacing), true);
    runAndWait(REVOLVER);
  }
}

void setStepperSteps(int index, long steps, bool ignoreEndstop) {
  if (steps != 0)
    steppers[index].prepareMovement(steps, ignoreEndstop);
}

void prepSteppingAbs(int index, long steps, bool ignoreEndstop) {
  long pos = steppers[index].getStepPosition();
  long _steps = steps - pos;
  setStepperSteps(index, _steps, ignoreEndstop);
}

void prepSteppingAbsMillimeter(int index, float millimeter, bool ignoreEndstop) {
  unsigned int stepsPerMM = steppers[index].getStepsPerMM();
  long steps = (long)((float)millimeter * stepsPerMM);
  long pos = steppers[index].getStepPosition();
  setStepperSteps(index, steps - pos, ignoreEndstop);
}

void prepSteppingRel(int index, long steps, bool ignoreEndstop) {
  setStepperSteps(index, steps, ignoreEndstop);
}

void prepSteppingRelMillimeter(int index, float millimeter, bool ignoreEndstop) {
  unsigned int stepsPerMM = steppers[index].getStepsPerMM();
  long steps = (long)((float)millimeter * stepsPerMM);
  setStepperSteps(index, steps, ignoreEndstop);
}

void printEndstopState(int serial) {
  const char* _triggered = "triggered";
  const char* _open      = "open";
  sprintf_P(tmp, PSTR("Selector: %s\tRevolver: %s\tFeeder: %s\n"),
          selectorEndstop()  ? _triggered : _open,
          revolverEndstop()  ? _triggered : _open,
          feederEndstop()    ? _triggered : _open);
  printResponse(tmp, serial);
}

void printSpeeds(int serial) {
  sprintf_P(tmp, P_AccelSpeed,
          String(steppers[SELECTOR].getMaxSpeed()).c_str(),
          String(steppers[REVOLVER].getMaxSpeed()).c_str(),
          smuffConfig.externalControl_Z ? "external" : String(steppers[FEEDER].getMaxSpeed()).c_str());
  printResponse(tmp, serial);
}

void printAcceleration(int serial) {
  sprintf_P(tmp, P_AccelSpeed,
          String(steppers[SELECTOR].getAcceleration()).c_str(),
          String(steppers[REVOLVER].getAcceleration()).c_str(),
          smuffConfig.externalControl_Z ? "external" : String(steppers[FEEDER].getAcceleration()).c_str());
  printResponse(tmp, serial);
}

void printOffsets(int serial) {
  sprintf_P(tmp, P_AccelSpeed,
          String((int)(smuffConfig.firstToolOffset*10)).c_str(),
          String(smuffConfig.firstRevolverOffset).c_str(),
          "--");
  printResponse(tmp, serial);
}

void printPos(int index, int serial) {
  sprintf_P(buf, PSTR("Pos. '%s': %ld\n"), steppers[index].getDescriptor(), steppers[index].getStepPosition());
  printResponseP(buf, serial);
}

void showLed(int mode, int count) {
  /*
  CRGB color;
  switch(mode) {
    case 0: // off
      color = CRGB::Black;
      break;
    case 1: // beep
      color = CRGB::Red;
      break;
    case 2: // longBeep
      color = CRGB::Cyan;
      break;
    case 3: // userBeep
      color = CRGB::Orange;
      break;
  }
  for(int i=0; i< NUM_LEDS; i++)
    leds[i] = color;
  FastLED.show();
  */
}

#ifdef __STM32F1__
/*
  Special function since the tone() function from the 
  libmaple library crashes the I2C Display somehow.

*/
void playTone(int pin, int freq, int duration) {
  
  //tone(pin, freq, duration);  return;
  pinMode(pin, OUTPUT);
  for (long i = 0; i < duration * 500L; i += freq) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(freq/10);
    digitalWrite(pin, LOW);
    delayMicroseconds(freq/10);
  }
}

void muteTone(int pin) {
  pinMode(pin, INPUT);
}
#endif

void beep(int count) {
  showLed(1, count);
  if(BEEPER_PIN == -1)
    return;
  for (int i = 0; i < count; i++) {
    _tone(BEEPER_PIN, BEEPER_FREQUENCY, BEEPER_DURATION);
    delayMicroseconds(BEEPER_DURATION*200);
  }
  _noTone(BEEPER_PIN);
}

void longBeep(int count) {
  showLed(2, count);
  if(BEEPER_PIN == -1)
    return;
  for (int i = 0; i < count; i++) {
    _tone(BEEPER_PIN, BEEPER_FREQUENCY, BEEPER_DURATION*5);
    delay(BEEPER_DURATION*5+50);
    _noTone(BEEPER_PIN);
  }
}

void userBeep() {
  showLed(3, 1);
  if(BEEPER_PIN == -1)
    return;
  _tone(BEEPER_PIN, BEEPER_FREQUENCY, BEEPER_DURATION);
  delay(BEEPER_DURATION*2);
  _noTone(BEEPER_PIN);
  _tone(BEEPER_PIN, BEEPER_UFREQUENCY, BEEPER_UDURATION);
  delay(BEEPER_UDURATION*2);
  _noTone(BEEPER_PIN);
  _tone(BEEPER_PIN, BEEPER_UFREQUENCY, BEEPER_UDURATION);
  delay(BEEPER_UDURATION*2);
  _noTone(BEEPER_PIN);
}

void initBeep() {
  showLed(4, 1);
  if(BEEPER_PIN == -1)
    return;
  _tone(BEEPER_PIN, 1760, 90); delay(90); _noTone(BEEPER_PIN);
  _tone(BEEPER_PIN, 1975, 90); delay(90); _noTone(BEEPER_PIN);
  _tone(BEEPER_PIN, 2093, 90); delay(90); _noTone(BEEPER_PIN);
  _tone(BEEPER_PIN, 1975, 90); delay(90); _noTone(BEEPER_PIN);
  _tone(BEEPER_PIN, 1760, 200); delay(250); _noTone(BEEPER_PIN);
}

bool setServoPos(int degree) {
  return servo.setServoPos(degree);
}

void getStoredData() {
  recoverStore();
  steppers[SELECTOR].setStepPosition(dataStore.stepperPos[SELECTOR]);
  steppers[REVOLVER].setStepPosition(dataStore.stepperPos[REVOLVER]);
  steppers[FEEDER].setStepPosition(dataStore.stepperPos[FEEDER]);
  toolSelected = dataStore.tool;
  __debug(PSTR("Recovered tool: %d"), toolSelected);
}

void setSignalPort(int port, bool state) {
  if(!smuffConfig.prusaMMU2) {
    sprintf(tmp,"%c%c%s", 0x1b, port, state ? "1" : "0");
    Serial2.write(tmp);
  }
}

void signalSelectorReady() {
  setSignalPort(SELECTOR_SIGNAL, false);
  //__debug(PSTR("Signalling Selector ready"));
}

void signalSelectorBusy() {
  setSignalPort(SELECTOR_SIGNAL, true);
  //__debug(PSTR("Signalling Selector busy"));
}

void signalLoadFilament() {
  setSignalPort(FEEDER_SIGNAL, true);
  //__debug(PSTR("Signalling load filament"));
}

void signalUnloadFilament() {
  setSignalPort(FEEDER_SIGNAL, false);
  //__debug(PSTR("Signalling unload filament"));
}

void listDir(File root, int numTabs, int serial) {
  while (true) {
    File entry =  root.openNextFile();
    if (!entry)
      break;

    for (int i = 1; i < numTabs; i++) {
      printResponse("\t", serial);
    }
    printResponse(entry.name(), serial);
    if (entry.isDirectory()) {
      printResponse("/\r\n", serial);
      //listDir(entry, numTabs + 1, serial);
    } 
    else {
      sprintf(tmp, "\t\t%ld\r\n", entry.size());
      printResponse(tmp, serial);
    }
    entry.close();
  }
}

void __debug(const char* fmt, ...) {
#ifdef DEBUG
  char _tmp[1024];
  va_list arguments;
  va_start(arguments, fmt); 
  vsnprintf_P(_tmp, 1024, fmt, arguments);
  va_end (arguments); 
  Serial.println(_tmp);
  //Serial1.println(_tmp);
#endif
}

