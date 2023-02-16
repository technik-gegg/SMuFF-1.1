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
#include "SMuFFBitmaps.h"
#include "ConfigNamesExt.h"

bool            displayingUserMessage = false;
bool            isWarning;
uint16_t        userMessageTime = 0;
static unsigned termRefresh = 0;
static uint32_t dialogId = 1;

#ifdef HAS_TMC_SUPPORT
TMC2209Stepper *showDriver = nullptr;
#endif

#if !defined(USE_SERIAL_DISPLAY)
#define BASE_FONT               u8g2_font_6x12_t_symbols
#define BASE_FONT_BIG           u8g2_font_7x14_tr
#define SMALL_FONT              u8g2_font_6x10_tr
#define STATUS_FONT             BASE_FONT_BIG
#define LOGO_FONT               BASE_FONT
#define ICONIC_FONT             u8g2_font_open_iconic_check_2x_t
#define ICONIC_FONT2            u8g2_font_open_iconic_embedded_2x_t
#define ICONIC_FONT3            u8g2_font_open_iconic_other_2x_t
#define TOOL_FONT               u8g2_font_logisoso22_tr

void setupDisplay() {
  // The next code line (display.setI2CAddress) changes the address for your TWI_DISPLAY if it's
  // configured differently.
  // Usually, those displays are pre-configured at I2C address 0x78, which equals to 0x3c
  // from the software side because of the 7-Bit address mode.
  // If it's configured at 0x7a, you need to change the I2C_DISPLAY_ADDRESS in Config.h to 0x3d.
  #if I2C_DISPLAY_ADDRESS != 0x3C
  display.setI2CAddress(I2C_DISPLAY_ADDRESS);
  __debugS(I, PSTR("setupDisplay: I2C address set to 0x%02X"), I2C_DISPLAY_ADDRESS);
  #endif
  #if defined(USE_LEONERD_DISPLAY)
    display.begin();
  #else
  display.begin(/*Select=*/ENCODER_BUTTON_PIN, /* menu_next_pin= */ U8X8_PIN_NONE, /* menu_prev_pin= */ U8X8_PIN_NONE, /* menu_home_pin= */ U8X8_PIN_NONE);
  #endif
  display.enableUTF8Print();
  resetDisplay();
  display.setContrast(smuffConfig.lcdContrast);
}

void drawVersion() {
  char brand[8] = VERSION_STRING;
#if defined(DEBUG)
  strcat(brand, "D");
#endif
  display.setFont(LOGO_FONT);
  display.setFontMode(0);
  display.setFontDirection(0);
  display.setDrawColor(1);
  display.setCursor(display.getDisplayWidth() - display.getStrWidth(brand) - 1, display.getDisplayHeight() - display.getMaxCharHeight());
  display.print(brand);
}

void drawLogo() {
  display.setBitmapMode(1);
  display.drawXBMP(0, 0, logo_width, logo_height, logo_bits);
}

void drawStatus() {
  char brand[8] = VERSION_STRING;
  #if defined(DEBUG)
  strcat(brand, "D");
  #endif
  char tmp[80];
  char tool[10];
  char mode[10];
  char relay[10];
  char driver[20];

  display.setFont(TOOL_FONT);
  display.setFontMode(0);
  display.setDrawColor(1);
  if ((toolSelected >= 0 && toolSelected < smuffConfig.toolCount))
    sprintf_P(tool, PSTR("T%-2d"), toolSelected);
  else
    sprintf_P(tool, PSTR("T--"));
  display.drawStr(12, 28, tool);

  display.setFont(STATUS_FONT);
  sprintf_P(tmp, PSTR("F1"));
  display.drawStr(display.getDisplayWidth() - display.getStrWidth(tmp) - 26, 15, tmp);
  sprintf_P(tmp, PSTR("F2"));
  display.drawStr(display.getDisplayWidth() - display.getStrWidth(tmp) - 26, 32, tmp);

  display.setFontMode(1);
  uint16_t y = display.getDisplayHeight()-display.getMaxCharHeight() + 1;
  uint8_t yBox = y-15;
  uint8_t yText = y-2;
  uint8_t boxHeight = display.getMaxCharHeight()+2;

  display.drawFrame(0, yBox, 10, boxHeight);
  sprintf_P(relay, PSTR("%1s"), smuffConfig.externalStepper ? PSTR("E") : PSTR("I"));
  display.drawStr(2, yText, relay);

  display.drawFrame(14, yBox, 38, boxHeight);
  sprintf_P(mode, PSTR("%5s"), (smuffConfig.prusaMMU2) ? P_Pemu : PSTR("SMuFF"));
  display.drawStr(16, yText, mode);

#if HAS_TMC_SUPPORT
  display.drawFrame(56, yBox, 27, boxHeight);
  display.drawBox(83, yBox, 9, boxHeight);

  display.drawStr(59, yText, PSTR("TMC"));
  if(!isUsingTmc) {
    // draw an X near "TMC" if no driver was configured at all
    display.setDrawColor(2);
    display.drawStr(84, yText-1, PSTR("x"));
    display.setDrawColor(1);
  }
  else {
    //tmcWarning = true; // for testing only
    if(tmcWarning) {
      display.setDrawColor(2);
      display.drawStr(84, yText, PSTR("!"));
      display.setDrawColor(1);
    }
  }
#else
  display.drawFrame(56, yBox, 36, boxHeight);
  display.setFont(SMALL_FONT);
  display.drawStr(60, yText-1, PSTR("A4988"));
  display.setFont(STATUS_FONT);
#endif

  display.drawFrame(96, yBox, 11, boxHeight);
  if(smuffConfig.usePurge) {
    display.drawStr(99, yText, PSTR("P"));
  }
  else {
    display.setFont(SMALL_FONT);
    display.drawStr(99, yText-2, PSTR("x"));
    display.setFont(STATUS_FONT);
  }

  display.setFont(ICONIC_FONT2);
  if(sendingStatesToggle)
    display.drawGlyph(111, y+1, 0x4F);
  else
    display.drawStr(111, y+1, " ");
  if(!smuffConfig.sendPeriodicalStats) {
    display.setFont(SMALL_FONT);
    display.drawStr(117, yText-2, PSTR("x"));
  }

  display.setFont(SMALL_FONT);
  sprintf_P(tmp, PSTR("        %5s  %6s"), parserBusy ? P_Busy : (feederJammed ? P_Jammed : P_Ready), brand);
  display.drawStr(1, display.getDisplayHeight()-1, tmp);

  display.setFontMode(0);
  display.setDrawColor(1);
  if (steppers[FEEDER].getMovementDone())
  {
    display.setFont(ICONIC_FONT);
    if(!smuffConfig.useSplitter) {
      display.drawGlyph(display.getDisplayWidth() - 18, 18, feederEndstop() ? 0x41 : 0x42);
      if (smuffConfig.useEndstop2)
        display.drawGlyph(display.getDisplayWidth() - 18, 35, feederEndstop(2) ? 0x41 : 0x42);
    }
    else {
      display.drawGlyph(display.getDisplayWidth() - 18, 18, smuffConfig.feedLoadState[getToolSelected()] > NOT_LOADED ? 0x41 : 0x42);
      display.setFont(ICONIC_FONT3);
      display.drawGlyph(display.getDisplayWidth() - 18, 35, smuffConfig.feedLoadState[getToolSelected()] == SPL_LOADED_TO_NOZZLE ? 0x47 : 0x20);
    }
  }
  drawFeed(false);

  #if !defined(USE_TERMINAL_MENUS)
  if(smuffConfig.menuOnTerminal) {
    termRefresh++;
    if(termRefresh % 5 == 0) {
      char tmcStat;
      char purge = (uint8_t)(smuffConfig.usePurge ? TERM_PRESENT_CHR : TERM_NOTAVAIL_CHR);
      char pstat = (uint8_t)(smuffConfig.sendPeriodicalStats ? TERM_PRESENT_CHR : TERM_NOTAVAIL_CHR);
      char f1 = (uint8_t)(feederEndstop() ? TERM_PRESENT_CHR : TERM_NOTAVAIL_CHR);
      char f2 = (uint8_t)(feederEndstop(2) ? TERM_PRESENT_CHR : TERM_NOTAVAIL_CHR);
      if (!smuffConfig.useEndstop2)
        f2 = '-';
      #if HAS_TMC_SUPPORT
        sprintf_P(driver, PSTR(" TMC %c"), (isUsingTmc ? (tmcWarning ? '!' : ' ') : '-'));
      #else
        sprintf_P(driver, PSTR(" A4988 "));
      #endif
      __terminal(P_SendTermStatus,
            tool, TERM_VERTLINE_CHR,
            f1, TERM_VERTLINE_CHR,
            f2, TERM_VERTLINE_CHR,
            mode, TERM_VERTLINE_CHR,
            relay, TERM_VERTLINE_CHR,
            driver, TERM_VERTLINE_CHR,
            purge, TERM_VERTLINE_CHR,
            pstat, TERM_VERTLINE_CHR,
            brand);
    }
  }
  #endif
  // __debugS(DEV2, PSTR("drawStatus done"));
}

void drawFeed(bool updateBuffer) {
  char tmp[20];
  sprintf_P(tmp, PSTR("%-4.1f "), steppers[FEEDER].getStepsTakenMM());
  display.setFont(SMALL_FONT);
  display.setFontMode(0);
  display.setDrawColor(1);
  uint16_t x = 0;
  uint16_t y = display.getDisplayHeight() - display.getMaxCharHeight() + 1;
  uint16_t w = 40;
  uint16_t h = display.getMaxCharHeight();
  //display.drawBox(x,y,w,h);
  display.drawStr(x + 1, display.getDisplayHeight()-1, tmp);
  if(updateBuffer)
    display.sendBuffer();
}

void drawUpload(uint32_t remain) {
  char _sel[20];
  char _wait[30];
  char tmp[50];
  sprintf_P(_sel, P_Upload);
  sprintf_P(_wait, P_Wait);
  if((remain > 2048))
    sprintf_P(tmp, P_BytesRemain, remain/1024, "K");
  else
    sprintf_P(tmp, P_BytesRemain, remain, "");
  display.clearDisplay();
  display.drawStr((display.getDisplayWidth() - display.getStrWidth(_sel)) / 2, (display.getDisplayHeight() - display.getMaxCharHeight()) / 2 - 10, _sel);
  display.setFont(BASE_FONT);
  display.drawStr((display.getDisplayWidth() - display.getStrWidth(tmp)) / 2, (display.getDisplayHeight() - display.getMaxCharHeight()) / 2 + 4, tmp);
  display.drawStr((display.getDisplayWidth() - display.getStrWidth(_wait)) / 2, (display.getDisplayHeight() - display.getMaxCharHeight()) / 2 + display.getMaxCharHeight() + 15, _wait);
  display.updateDisplay();
}

#ifdef HAS_TMC_SUPPORT

void drawStallCallback() {
  uint16_t symbols[] = {0x0020, 0x21af, 0x0020, 0x2607};
  if (showDriver == nullptr)
    return;
  bool stat = showDriver->diag();
  display.setFont(ICONIC_FONT);
  display.drawGlyph(100, 11, symbols[stat]);
  stat = showDriver->SG_RESULT() < showDriver->SGTHRS() * 2;
  display.drawGlyph(114, 11, symbols[stat + 2]);
}
#endif

void showTMCStatus(uint8_t axis) {
  debounceButton();
  encoder.setAccelerationEnabled(true);
#ifdef HAS_TMC_SUPPORT
  showDriver = drivers[axis];
#else
#define showDriver nullptr
#endif
  if (showDriver == nullptr)
  {
    debounceButton();
    drawUserMessage(P_StepperNotCfg);
    delay(3000);
    return;
  }
#ifdef HAS_TMC_SUPPORT
  char _msg[256];
  int16_t turn;
  uint8_t btn;
  bool isHeld, isClicked;

  steppers[axis].setEnabled(true);
  displayingUserMessage = true;
  uint8_t n = 0;
  int8_t pg = 0;
  while (1)
  {
    getInput(&turn, &btn, &isHeld, &isClicked);
    if (isHeld || isClicked)
      break;
    if(turn < 0)
      pg--;
    else if(turn > 0)
      pg++;
    if(pg > 1)
      pg = 0;
    if(pg < 0)
      pg = 1;

    const char *ot_stat = P_No;
    if (showDriver->ot())
    {
      if (showDriver->t157())
        ot_stat = P_OT_157;
      else if (showDriver->t150())
        ot_stat = P_OT_150;
      else if (showDriver->t143())
        ot_stat = P_OT_143;
      else if (showDriver->t120())
        ot_stat = P_OT_120;
    }
    if(pg == 1) {
      sprintf_P(_msg, P_TMC_Status1,
                showDriver->stealth() ? P_Stealth : P_Spread,
                smuffConfig.stepperPower[axis],
                showDriver->ola() ? P_Yes : P_No,
                showDriver->olb() ? P_Yes : P_No,
                showDriver->s2ga() ? P_Yes : P_No,
                showDriver->s2gb() ? P_Yes : P_No,
                ot_stat);
    }
    else {
      sprintf_P(_msg, P_TMC_Status0,
                showDriver->stealth() ? P_Stealth : P_Spread,
                showDriver->rms_current(),
                showDriver->microsteps(),
                showDriver->ms2(), showDriver->ms1(),
                showDriver->pdn_uart() ? P_Yes : P_No,
                showDriver->diag() ? P_Low : P_High);
    }

    drawUserMessage(_msg, true, false, showDriver->stealth() ? drawStallCallback : nullptr);
    delay(100);
    if (showDriver->diag()) {
      //if(n==0) __debugS(DEV3, PSTR("Stepper stall detected @ %ld"), showDriver->SG_RESULT());
      n++;
    }
    // reset stall flag after 3 secs.
    if (n % 30 == 0 && showDriver->diag()) {
      n = 0;
      //__debugS(DEV3, PSTR("Stepper stall has been reset!"));
    }
  }
  tmcWarning = false;
  displayingUserMessage = false;
#endif
}

void resetDisplay() {
  display.clearDisplay();
  display.setFont(BASE_FONT);
  display.setFontMode(0);
  display.setDrawColor(1);
}

void drawSelectingMessage(uint8_t tool) {
  char _sel[20];
  char _wait[30];
  char tmp[40];

  if (displayingUserMessage) // don't show if something else is being displayed
    return;
  display.clearBuffer();
  sprintf_P(_sel, P_Selecting);
  sprintf_P(_wait, P_Wait);
  if (*smuffConfig.materialNames[tool] != 0)
    sprintf_P(tmp, PSTR("%s"), smuffConfig.materialNames[tool]);
  else
    sprintf_P(tmp, P_ToolMenu, tool);
  display.drawStr((display.getDisplayWidth() - display.getStrWidth(_sel)) / 2, (display.getDisplayHeight() - display.getMaxCharHeight()) / 2 - 10, _sel);
  display.setFont(BASE_FONT_BIG);
  display.drawStr((display.getDisplayWidth() - display.getStrWidth(tmp)) / 2, (display.getDisplayHeight() - display.getMaxCharHeight()) / 2 + 9, tmp);
  display.setFont(BASE_FONT);
  display.drawStr((display.getDisplayWidth() - display.getStrWidth(_wait)) / 2, (display.getDisplayHeight() - display.getMaxCharHeight()) / 2 + display.getMaxCharHeight() + 10, _wait);
  display.updateDisplay();
#if defined(USE_TERMINAL_MENUS)
  if(smuffConfig.menuOnTerminal) {
    terminalClear(true);
    terminalSend(1, 1, _sel, true, 0);
    terminalSend(3, 1, tmp, true, 2);
    terminalSend(5, 1, _wait, true, 0);
  }
#endif
}

void drawPurgingMessage(uint16_t len, uint8_t tool) {
  char _sel[20];
  char _wait[30];
  char tmp[50];

  if (displayingUserMessage) // don't show if something else is being displayed
    return;
  if(len == 0 && tool == 0) {
    if(currentSerial != SM_SERIAL_PORT_NULL) {
      sprintf_P(tmp, PSTR("echo: purging: done\n"));
      printResponse(tmp, currentSerial);
    }
    return;
  }
  display.clearBuffer();
  sprintf_P(_sel, P_Purging);
  sprintf_P(_wait, P_Wait);
  sprintf_P(tmp, P_PurgeLen, smuffConfig.purges[tool], String(len).c_str(), String((double)len * 2.4).c_str());
  display.drawStr((display.getDisplayWidth() - display.getStrWidth(_sel)) / 2, (display.getDisplayHeight() - display.getMaxCharHeight()) / 2 - 10, _sel);
  display.setFont(BASE_FONT);
  display.drawStr((display.getDisplayWidth() - display.getStrWidth(tmp)) / 2, (display.getDisplayHeight() - display.getMaxCharHeight()) / 2 + 4, tmp);
  sprintf_P(tmp, P_PurgeCubic, String((double)len * 2.4).c_str());
  display.drawStr((display.getDisplayWidth() - display.getStrWidth(tmp)) / 2, (display.getDisplayHeight() - display.getMaxCharHeight()) / 2 + 15, tmp);
  display.setFont(BASE_FONT);
  display.drawStr((display.getDisplayWidth() - display.getStrWidth(_wait)) / 2, (display.getDisplayHeight() - display.getMaxCharHeight()) / 2 + display.getMaxCharHeight() + 15, _wait);
  display.updateDisplay();
  if(smuffConfig.webInterface) {
    sprintf_P(tmp, PSTR("echo: purging: T:%d L:%d C:%f\n"), tool, len, len*2.4);
    if(currentSerial != SM_SERIAL_PORT_NULL)
      printResponse(tmp, currentSerial);
  }
#if defined(USE_TERMINAL_MENUS)
  if(smuffConfig.menuOnTerminal) {
    terminalClear(true);
    terminalSend(1, 1, _sel, true, 0);
    terminalSend(3, 1, tmp, true, 2);
    terminalSend(5, 1, _wait, true, 0);
  }
#endif
}

void drawTestrunMessage(unsigned long loop, char *msg) {
  char _sel[30];
  char _wait[30];
  display.clearBuffer();
  sprintf_P(_sel, P_RunningCmd, loop);
  sprintf_P(_wait, P_ButtonToStop);
  display.drawStr((display.getDisplayWidth() - display.getStrWidth(_sel)) / 2, (display.getDisplayHeight() - display.getMaxCharHeight()) / 2 - 10, _sel);
  display.setFont(BASE_FONT_BIG);
  display.drawStr((display.getDisplayWidth() - display.getStrWidth(msg)) / 2, (display.getDisplayHeight() - display.getMaxCharHeight()) / 2 + 12, msg);
  display.setFont(BASE_FONT);
  display.drawStr((display.getDisplayWidth() - display.getStrWidth(_wait)) / 2, (display.getDisplayHeight() - display.getMaxCharHeight()) / 2 + display.getMaxCharHeight() + 15, _wait);
  display.updateDisplay();
#if defined(USE_TERMINAL_MENUS)
  if(smuffConfig.menuOnTerminal) {
    terminalSend(1, 1, _sel, true, 0);
    terminalSend(3, 1, msg, true, 2);
    terminalSend(5, 1, _wait, true, 0);
  }
#endif
}

void drawUserMessage(String message, bool smallFont /* = false */, bool center /* = true */, void (*drawCallbackFunc)() /* = nullptr */) {
  char *lines[8];
  uint8_t lineCnt = splitStringLines(lines, (int)ArraySize(lines), message.c_str());

  if (isPwrSave) {
    setPwrSave(0);
  }
  terminalClear(true);
  display.clearBuffer();
  display.setDrawColor(1);
  display.setFont(smallFont ? BASE_FONT : BASE_FONT_BIG);
  uint16_t y = (display.getDisplayHeight() - ((lineCnt-1) * display.getMaxCharHeight())) / 2;
  uint8_t tp = 1+((TERM_LINES-lineCnt)/2);
  bool isSeparator = false;
  for (uint8_t i = 0; i < lineCnt; i++) {
    uint16_t x = center ? (display.getDisplayWidth() - display.getStrWidth(lines[i])) / 2 : 0;
    display.drawStr(x, y, lines[i]);
    if (i == 0) {
      if (lineCnt > 1 && strcmp(lines[1], " ") == 0) {
        display.drawHLine(0, y + 3, display.getDisplayWidth());
        isSeparator = true;
      }
    }
    y += display.getMaxCharHeight();
    if(smuffConfig.menuOnTerminal) {
      if (i==1 && isSeparator)
        terminalDrawSeparator(tp + i, 1, 36);
      else
        terminalSend(tp + i, 1, lines[i], center, 0, true);
    }
  }
  if (drawCallbackFunc != nullptr)
    drawCallbackFunc();
  display.updateDisplay();
  display.setFont(BASE_FONT);
  displayingUserMessage = true;
  userMessageTime = millis();
}

void drawSDStatus(int8_t stat)
{
  char tmp[40];

  switch (stat) {
    case SD_ERR_INIT:
      sprintf_P(tmp, P_SD_InitError);
      longBeep(2);
      break;
    case SD_ERR_NOCONFIG:
      sprintf_P(tmp, P_SD_NoConfig);
      longBeep(1);
      break;
    case SD_READING_CONFIG:
      sprintf_P(tmp, P_SD_Reading, P_SD_ReadingConfig);
      break;
    case SD_READING_TMC:
      sprintf_P(tmp, P_SD_Reading, P_SD_ReadingTmc);
      break;
    case SD_READING_SERVOS:
      sprintf_P(tmp, P_SD_Reading, P_SD_ReadingServos);
      break;
    case SD_READING_MATERIALS:
      sprintf_P(tmp, P_SD_Reading, P_SD_ReadingMaterials);
      break;
    case SD_READING_PURGES:
      sprintf_P(tmp, P_SD_Reading, P_SD_ReadingPurges);
      break;
    case SD_READING_STEPPERS:
      sprintf_P(tmp, P_SD_Reading, P_SD_ReadingSteppers);
      break;
  }
  // __debugS(D, PSTR("\tdrawSDStatus: drawing status"));
  display.clearBuffer();
  drawLogo();
  drawVersion();
  display.setCursor((display.getDisplayWidth() - display.getStrWidth(tmp)) / 2, display.getDisplayHeight()-1);
  display.print(tmp);
  // __debugS(D, PSTR("\tdrawSDStatus: updating display"));
  display.updateDisplay();
  __debugS(DEV, PSTR("\tdrawSDStatus: %d -> %s"), stat, tmp);
}

bool showFeederBlockedMessage() {
  char errmsg[MAX_ERR_MSG];
  bool state = false;
  lastEncoderButtonTime = millis();
  beep(1);
  uint8_t button = showDialog(P_TitleWarning, P_FeederLoaded, P_RemoveMaterial, P_CancelRetryButtons);
  if (button == 1) {
    refreshStatus();
    state = unloadFilament(errmsg);
    __debugS(DEV3, PSTR("showFeederBlockedMessage: returned [%s]"), errmsg);
  }
  display.clearDisplay();
  return state;
}

bool showFeederLoadedMessage() {
  char errmsg[MAX_ERR_MSG];
  bool state = false;
  lastEncoderButtonTime = millis();
  beep(1);
  uint8_t button = showDialog(P_TitleWarning, P_FeederLoaded, P_AskUnload, P_YesNoButtons);
  if (button == 1) {
    refreshStatus();
    state = unloadFilament(errmsg);
    __debugS(DEV3, PSTR("showFeederLoadedMessage: returned [%s]"), errmsg);
  }
  display.clearDisplay();
  return state;
}

bool showFeederLoadMessage() {
  char errmsg[MAX_ERR_MSG];
  bool state = false;
  lastEncoderButtonTime = millis();
  beep(1);
  uint8_t button = showDialog(P_TitleSelected, P_SelectedTool, P_AskLoad, P_YesNoButtons);
  if (button == 1) {
    refreshStatus();
    if (smuffConfig.prusaMMU2)
      state = loadFilamentPMMU2(errmsg);
    else
      state = loadFilament(errmsg);
    __debugS(DEV3, PSTR("showFeederLoadMessage: returned [%s]"), errmsg);
  }
  display.clearDisplay();
  return state;
}

bool showFeederFailedMessage(int8_t opt) {
  lastEncoderButtonTime = millis();
  beep(3);
  uint8_t button = 99;
  isWarning = true;
  do {
    button = showDialog(P_TitleWarning, opt == 1 ? P_CantLoad : P_CantUnload, P_CheckUnit, P_CancelRetryButtons);
  } while (button != 1 && button != 2);
  isWarning = false;
  display.clearDisplay();
  debounceButton();
  return button == 1 ? false : true;
}

uint8_t showDialog(PGM_P title, PGM_P message, PGM_P addMessage, PGM_P buttons, int32_t timeout) {
  //__debugS(DEV2, PSTR("showDialog: %S"), title);
  if (isPwrSave)
    setPwrSave(0);
  setFastLEDStatus(FASTLED_STAT_WARNING);
  char _title[80];
  char msg1[256];
  char msg2[80];
  char btn[40];
  sprintf_P(_title, title);
  sprintf_P(msg1, message);
  sprintf_P(msg2, addMessage);
  sprintf_P(btn, buttons);
  terminalClear(true);
  uint8_t stat = display.userInterfaceMessage(_title, msg1, msg2, btn);
  setFastLEDStatus(FASTLED_STAT_NONE);
  return stat;
}

void signalNoTool() {
  char _msg1[30];
  userBeep();
  sprintf_P(_msg1, P_NoTool);
  strcat_P(_msg1, P_Aborting);
  drawUserMessage(_msg1);
}

void refreshStatus(bool feedOnly /* = false */) {
  #if defined(USE_LEONERD_DISPLAY)
    if(encoder.busy()) {
      __debugS(DEV2, PSTR("refreshStatus: Encoder busy, aborting"));
      return;
    }
  #endif
  if(refreshingDisplay)
    return;
  if (initDone && !isPwrSave && !showMenu && !displayingUserMessage && !isTestrun && !isUpload) {
    refreshingDisplay = true;
    if (feedOnly) {
      drawFeed();
    }
    else {
      // __debugS(DEV2, PSTR("refreshStatus: updating display"));
      display.clearBuffer();
      drawStatus();
      display.updateDisplay();
      // __debugS(DEV2, PSTR("refreshStatus: display updated"));
      // note to myself: if it's hanging here, you did something wrong!
    }
    refreshingDisplay = false;
  }
}

void setDisplayPowerSave(bool state) {
  __debugS(DEV3,PSTR("setDisplayPowerSave: %s"), state ? P_Yes : P_No);
  display.setPowerSave(state);
}

#else

void setupDisplay() {
    __debugS(D, PSTR("\tsetupDisplay: Serial Display configured on port SERIAL %d"), smuffConfig.displaySerial);
}

void drawVersion() {}                                       // not used
void drawLogo() {}                                          // not used
void drawStatus() {}                                        // not used
void drawFeed(bool updateBuffer) {}                         // not used
void drawUpload(uint32_t remain) {}                         // not used
void drawSymbolCallback() {}                                // not used
#ifdef HAS_TMC_SUPPORT
void drawStallCallback() {}                                 // not used
#endif
void showTMCStatus(uint8_t axis) {}                         // not used
void resetDisplay() {}                                      // not used
void refreshStatus(bool feedOnly) {}                        // not used
void setDisplayPowerSave(bool state) {}                     // not used
void drawTestrunMessage(unsigned long loop, char *msg) {}   // not used
void drawSelectingMessage(uint8_t tool) {}                  // not used
void drawSDStatus(int8_t stat) {}                           // not used
void signalNoTool() {}                                      // not used

void drawPurgingMessage(uint16_t len, uint8_t tool) {
  char tmp[128];
  if(len == 0 && tool == 0)
    sprintf_P(tmp, PSTR("echo: purging: done\n"));
  else
    sprintf_P(tmp, PSTR("echo: purging: T:%d L:%d C:%f\n"), tool, len, len*2.4);
  printResponse(tmp, smuffConfig.displaySerial);
}

void drawUserMessage(String message, bool smallFont, bool center, void (*drawCallbackFunc)()) {
  char msg[1024];
  message.replace("\n", "\\n");
  snprintf(msg, ArraySize(msg)-1, "{\"%s\": \"%s\"}\n", userMessage, message.c_str());
  printResponse(msg, smuffConfig.displaySerial);
}

uint8_t showDialog(PGM_P title, PGM_P message, PGM_P addMessage, PGM_P buttons, int32_t timeout) {
  uint8_t state = 0;
  char dlg[1024];

  // no action if no display serial port is defined
  if(smuffConfig.displaySerial == SM_SERIAL_PORT_NULL) {
    return state;
  }

  setFastLEDStatus(FASTLED_STAT_WARNING);
  String btn(buttons);
  String msg(message);
  String addMsg(addMessage);
  // replace all '\n' because it might mess with the JSON parser on the other end
  btn.replace("\n","|");
  msg.replace("\n","\\n");
  addMsg.replace("\n","\\n");
  
  snprintf(dlg, ArraySize(dlg)-1, "{\"%s\": { \"Id\": %d, \"Title\": \"%s\", \"Message\": \"%s\", \"Action\": \"%s\", \"Buttons\": \"%s\"}}\n", userDialog, dialogId, title, msg.c_str(), addMsg.c_str(), btn.c_str());
  printResponse(dlg, smuffConfig.displaySerial);
  
  waitForDlgId = dialogId++;
  gotDlgId = false;
  setParserBusy();
  do {
    // wait for response
    delay(100);
    checkSerialPending();
    if(timeout != -1) {
      if(--timeout == 0)
        break;
    }
  } while(!gotDlgId);
  setParserReady();
  setFastLEDStatus(FASTLED_STAT_NONE);

  if(gotDlgId) {
    state = dlgButton;
  }
  return state;
}

bool showFeederBlockedMessage() {
  bool state = false;
  char errmsg[MAX_ERR_MSG];

  uint8_t button = showDialog(P_TitleWarning, P_FeederLoaded, P_RemoveMaterial, P_CancelRetryButtons, -1);
  if (button == 1) {
    state = unloadFilament(errmsg);
    __debugS(DEV3, PSTR("showFeederBlockedMessage: returned [%s]"), state ? P_OkButtonOnly : errmsg);
  }
  else {
    __debugS(DEV3, PSTR("showFeederBlockedMessage: cancelled"));
  }
 return state;
}

bool showFeederLoadedMessage() {
  bool state = false;
  char errmsg[MAX_ERR_MSG];

  uint8_t button = showDialog(P_TitleWarning, P_FeederLoaded, P_AskUnload, P_YesNoButtons, -1);
  if (button == 1) {
    state = unloadFilament(errmsg);
    __debugS(DEV3, PSTR("showFeederLoadedMessage: returned [%s]"), state ? P_OkButtonOnly : errmsg);
  }
  else {
    __debugS(DEV3, PSTR("showFeederLoadedMessage: cancelled"));
  }
  return state;
}

bool showFeederLoadMessage() {
  bool state = false;
  char errmsg[MAX_ERR_MSG];

  uint8_t button = showDialog(P_TitleSelected, P_SelectedTool, P_AskLoad, P_YesNoButtons, 300);
  if (button == 1) {
    if (smuffConfig.prusaMMU2)
      state = loadFilamentPMMU2(errmsg);
    else
      state = loadFilament(errmsg);
    __debugS(DEV3, PSTR("showFeederLoadMessage: returned [%s]"), state ? P_OkButtonOnly : errmsg);
  }
  else {
    __debugS(DEV3, PSTR("showFeederLoadMessage: cancelled"));
  }
  return state;
}

bool showFeederFailedMessage(int8_t opt) {
  uint8_t button;
  do {
    button = showDialog(P_TitleWarning, opt == 1 ? P_CantLoad : P_CantUnload, P_CheckUnit, P_CancelRetryButtons, 300);
  } while (button != 1 && button != 2);
  return button == 1 ? false : true;
}

#endif