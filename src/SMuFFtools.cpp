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
#include "InputDialogs.h"

#ifdef __ESP32__
extern BluetoothSerial SerialBT;
#endif

extern ZStepper steppers[];
extern ZServo servo;
extern ZServo servoLid;
extern SdFat SD;

SMuFFConfig smuffConfig;
int8_t toolSelected = -1;
bool feederJammed = false;
PositionMode positionMode = RELATIVE;
bool displayingUserMessage = false;
bool isAbortRequested = false;
uint16_t userMessageTime = 0;
uint8_t swapTools[MAX_TOOLS];
bool isWarning;
bool lidOpen = true;
unsigned long feederErrors = 0;
bool ignoreHoming = false;
unsigned long stallDetectedCountSelector = 0;
unsigned long stallDetectedCountFeeder = 0;
char PROGMEM tuneStartup[MAX_TUNE1] = {"F1760D90.F1975D90.F2093D90.F1975D90.F1760D200P50."}; // the "traditional" tune
char PROGMEM tuneUser[MAX_TUNE2] = {"F1760D90P90.F440D90P90.F440D90P90."};
char PROGMEM tuneBeep[MAX_TUNE3] = {"F1760D90P200."};
char PROGMEM tuneLongBeep[MAX_TUNE3] = {"F1760D450P500."};
#if defined(USE_LEONERD_DISPLAY)
char PROGMEM tuneEncoder[MAX_TUNE3] = {"F330D10P10."};
#else
char PROGMEM tuneEncoder[MAX_TUNE3] = {"F1440D3."};
#endif

uint16_t sequence[MAX_SEQUENCE][3]; // store for tune sequence for background playing
uint8_t sequenceCnt = 0;
int8_t currentSerial = -1;

#define _F_ 0
#define _D_ 1
#define _P_ 2

void setupDisplay()
{
// The next code line (display.setI2CAddress) changes the address for your TWI_DISPLAY if it's
// configured differently.
// Usually, thoses displays are pre-configured at I2C address 0x78, which equals to 0x3c
// from the software side because of the 7-Bit address mode.
// If it's configured at 0x7a, you need to change the I2C_DISPLAY_ADDRESS in Config.h to 0x3d.
#if I2C_DISPLAY_ADDRESS != 0x3C
  display.setI2CAddress(I2C_DISPLAY_ADDRESS);
  __debugS(PSTR("I2C display address set to 0x%02X"), I2C_DISPLAY_ADDRESS);
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

void drawLogo()
{
  display.setBitmapMode(1);
  display.drawXBMP(0, 0, logo_width, logo_height, logo_bits);
}

static unsigned termRefresh = 0;

void drawStatus()
{
  char brand[8] = VERSION_STRING;
  #if defined(DEBUG)
  strcat(brand, "D");
  #endif
  char tmp[80];
  char tool [5];
  char mode[6];
  char relay[4];
  char driver[10];

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
  display.drawGlyph(111, y+1, 0x4F);
  if(!smuffConfig.sendPeriodicalStats) {
    display.setFont(SMALL_FONT);
    display.drawStr(117, yText-2, PSTR("x"));
  }

  display.setFont(SMALL_FONT);
  sprintf_P(tmp, PSTR("        %5s  %6s"), parserBusy ? P_Busy : P_Ready, brand);
  display.drawStr(1, display.getDisplayHeight()-1, tmp);

  display.setFontMode(0);
  display.setDrawColor(1);
  if (steppers[FEEDER].getMovementDone())
  {
    display.setFont(ICONIC_FONT);
    display.drawGlyph(display.getDisplayWidth() - 18, 18, feederEndstop() ? 0x41 : 0x42);
    if (smuffConfig.useEndstop2)
      display.drawGlyph(display.getDisplayWidth() - 18, 35, feederEndstop(2) ? 0x41 : 0x42);
  }
  drawFeed(false);

  #if defined(USE_TERMINAL_MENUS)
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
  //__debugS(PSTR("[drawStatus] end..."));
}

void drawFeed(bool updateBuffer)
{
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

uint8_t duetTurn = 0;
void drawSymbolCallback()
{
  uint16_t symbols[] = {0x25ef, 0x25d0, 0x25d3, 0x25d1, 0x25d2};

  if (duetLS.isMoving())
  {
    if (++duetTurn > 4)
      duetTurn = 1;
  }
  else
  {
    duetTurn = 0;
  }
  display.setFont(ICONIC_FONT);
  display.drawGlyph(118, 10, symbols[duetTurn]);
}

void showDuetLS()
{
  char _msg[128];
  char _addData[15];
  char _dir[3];
  int16_t turn;
  uint8_t btn;
  bool isHeld, isClicked;

  bool extStepper = smuffConfig.externalStepper;
  switchFeederStepper(INTERNAL);
  debounceButton();
  encoder.setAccelerationEnabled(true);
  while (1)
  {
    getInput(&turn, &btn, &isHeld, &isClicked);
    if (isHeld || isClicked)
    {
      break;
    }
    // if the encoder knob is being turned, extrude / retract filament by the value defined
    switch (encoder.getValue())
    {
    case -1:
      moveFeeder(-0.25);
      break;
    case 1:
      moveFeeder(0.25);
      break;
    case -2:
      moveFeeder(-0.50);
      break;
    case 2:
      moveFeeder(0.50);
      break;
    case -3:
      moveFeeder(-1.00);
      break;
    case 3:
      moveFeeder(1.00);
      break;
    case -4:
      moveFeeder(-2.00);
      break;
    case 4:
      moveFeeder(2.00);
      break;
    case -5:
      moveFeeder(-5.00);
      break;
    case 5:
      moveFeeder(5.00);
      break;
    default:
      break;
    }
    uint8_t err = duetLS.getSensorError();
    if (err != 0)
      sprintf_P(_addData, PSTR("%4s/%04x"), err == E_SENSOR_INIT ? "INIT" : err == E_SENSOR_VCC ? "VCC"
                                                                                                : String(err).c_str(),
                duetLS.getError());
    else
    {
      if (!duetLS.isValid())
        sprintf_P(_addData, PSTR("-invalid-"));
      else
        sprintf_P(_addData, PSTR("none/%04x"), duetLS.getError());
    }
    switch (duetLS.getDirection())
    {
    case DIR_NONE:
      sprintf(_dir, "  ");
      break;
    case DIR_EXTRUDE:
      sprintf(_dir, "<<");
      break;
    case DIR_RETRACT:
      sprintf(_dir, ">>");
      break;
    }

    sprintf_P(_msg, P_DuetLSData, _dir, String(duetLS.getPositionMM()).c_str(), duetLS.getQuality(), duetLS.getBrightness(), duetLS.getShutter(), duetLS.getSwitch() ? P_On : P_Off, _addData, duetLS.getVersion());
    drawUserMessage(_msg, true, false, drawSymbolCallback);
    delay(100);
  }
  if (extStepper)
    switchFeederStepper(EXTERNAL);
}

#ifdef HAS_TMC_SUPPORT
TMC2209Stepper *showDriver = nullptr;

void drawStallCallback()
{
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

void showTMCStatus(uint8_t axis)
{
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
    if (showDriver->diag())
    {
      //if(n==0)
      //  __debugS(PSTR("Stepper stall detected @ %ld"), showDriver->SG_RESULT());
      n++;
    }
    // reset stall flag after 3 secs.
    if (n % 30 == 0 && showDriver->diag())
    {
      n = 0;
      //__debugS(PSTR("Stall has been reset!"));
    }
  }
  tmcWarning = false;
  displayingUserMessage = false;
#endif
}

void sendTMCStatus(uint8_t axis, int8_t port) {

#ifdef HAS_TMC_SUPPORT
  showDriver = drivers[axis];
#else
#define showDriver nullptr
#endif

  char tmp[80];
  if(showDriver == nullptr) {
    sprintf_P(tmp, P_TMCStatusNotUsed, P_TMCStatus, P_TMCKeyAxis, axis, P_TMCKeyInUse);
    printResponse(tmp, port);
    return;
  }

  steppers[axis].setEnabled(true);
  const char *ot_stat = P_No;

  if (showDriver->ot())  {
    if (showDriver->t157())
      ot_stat = P_OT_157;
    else if (showDriver->t150())
      ot_stat = P_OT_150;
    else if (showDriver->t143())
      ot_stat = P_OT_143;
    else if (showDriver->t120())
      ot_stat = P_OT_120;
  }
  Print* out = getSerialInstance(port);

  serializeTMCStats(out,
      axis,
      showDriver->version()- 0x20,
      showDriver->stealth(),
      smuffConfig.stepperPower[axis],
      showDriver->rms_current(),
      showDriver->microsteps(),
      showDriver->ms2(),
      showDriver->ms1(),
      showDriver->pdn_uart() ? P_Yes : P_No,
      showDriver->diag() ? P_Low : P_High,
      showDriver->ola() ? P_Yes : P_No,
      showDriver->olb() ? P_Yes : P_No,
      showDriver->s2ga() ? P_Yes : P_No,
      showDriver->s2gb() ? P_Yes : P_No,
      ot_stat);
  printResponse("\n", port);
  #if defined(SMUFF_V6S)
  if(axis == REVOLVER)
    steppers[axis].setEnabled(false);
  #endif
}

void resetDisplay()
{
  display.clearDisplay();
  display.setFont(BASE_FONT);
  display.setFontMode(0);
  display.setDrawColor(1);
}

void drawSelectingMessage(uint8_t tool)
{
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

void drawPurgingMessage(uint16_t len, uint8_t tool)
{
  char _sel[20];
  char _wait[30];
  char tmp[50];

  if (displayingUserMessage) // don't show if something else is being displayed
    return;
  display.clearBuffer();
  sprintf_P(_sel, P_Purging);
  sprintf_P(_wait, P_Wait);
  sprintf_P(tmp, P_PurgeLen, smuffConfig.purges[tool], String(len).c_str(), String((float)len * 2.4).c_str());
  display.drawStr((display.getDisplayWidth() - display.getStrWidth(_sel)) / 2, (display.getDisplayHeight() - display.getMaxCharHeight()) / 2 - 10, _sel);
  display.setFont(BASE_FONT);
  display.drawStr((display.getDisplayWidth() - display.getStrWidth(tmp)) / 2, (display.getDisplayHeight() - display.getMaxCharHeight()) / 2 + 4, tmp);
  sprintf_P(tmp, P_PurgeCubic, String((float)len * 2.4).c_str());
  display.drawStr((display.getDisplayWidth() - display.getStrWidth(tmp)) / 2, (display.getDisplayHeight() - display.getMaxCharHeight()) / 2 + 15, tmp);
  display.setFont(BASE_FONT);
  display.drawStr((display.getDisplayWidth() - display.getStrWidth(_wait)) / 2, (display.getDisplayHeight() - display.getMaxCharHeight()) / 2 + display.getMaxCharHeight() + 15, _wait);
  display.updateDisplay();
  if(smuffConfig.webInterface) {
    sprintf_P(tmp, PSTR("echo: purging: T:%d L:%d C:%f\n"), tool, len, len*2.4);
    if(currentSerial != -1)
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

void drawTestrunMessage(unsigned long loop, char *msg)
{
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

/**
 * Parses a given buffer for newlines (delimiter) and fills the lines pointer
 * array with the lines found.
 *
 * @param lines     container for lines
 * @param maxLines  max. storage capacity of lines
 * @param message   the source buffer to be parsed for lines
 * @param delimiter the EOL character to parse for
 * @returns the number of lines found
 */
uint8_t splitStringLines(char *lines[], uint8_t maxLines, const char *message, const char *delimiter)
{

  char *tok = strtok((char *)message, delimiter);
  char *lastTok = nullptr;
  int8_t cnt = -1;

  while (tok != nullptr)
  {
    lines[++cnt] = tok;
    lastTok = tok;
    //__debugS(PSTR("Line: %s"), lines[cnt]);
    if (cnt >= maxLines - 1)
      break;
    tok = strtok(nullptr, delimiter);
  }
  if (lastTok != nullptr && *lastTok != 0 && cnt <= maxLines - 1)
  {
    lines[cnt] = lastTok; // copy the last line as well
    cnt++;
  }

  return cnt;
}

void drawUserMessage(String message, bool smallFont /* = false */, bool center /* = true */, void (*drawCallbackFunc)() /* = nullptr */)
{
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
  for (uint8_t i = 0; i < lineCnt; i++)
  {
    uint16_t x = center ? (display.getDisplayWidth() - display.getStrWidth(lines[i])) / 2 : 0;
    display.drawStr(x, y, lines[i]);
    if (i == 0)
    {
      if (lineCnt > 1 && strcmp(lines[1], " ") == 0)
      {
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

  switch (stat)
  {
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
  display.clearBuffer();
  drawLogo();
  drawVersion();
  display.setCursor((display.getDisplayWidth() - display.getStrWidth(tmp)) / 2, display.getDisplayHeight()-1);
  display.print(tmp);
  display.updateDisplay();
  //__debugS(PSTR("[drawSDStatus] %d -> %s"), stat, tmp);
}

bool selectorEndstop()
{
  return steppers[SELECTOR].getEndstopHit();
}

bool revolverEndstop()
{
  return steppers[REVOLVER].getEndstopHit();
}

bool feederEndstop(int8_t index)
{
  return steppers[FEEDER].getEndstopHit(index);
}

void setAbortRequested(bool state)
{
  steppers[FEEDER].setAbort(state); // stop any ongoing stepper movements
}

void setParserBusy()
{
  parserBusy = true;
}

void setParserReady()
{
  parserBusy = false;
  drawStatus();
}

bool moveHome(int8_t index, bool showMessage, bool checkFeeder)
{
  if (!steppers[index].getEnabled())
    steppers[index].setEnabled(true);

  if (feederJammed)
  {
    beep(4);
    return false;
  }
  setParserBusy();
  if (checkFeeder && feederEndstop())
  {
    if (showMessage)
    {
      if (!showFeederLoadedMessage())
      {
        setParserReady();
        return false;
      }
    }
    else
    {
      if (feederEndstop())
        unloadFilament();
    }
  }
  if (smuffConfig.revolverIsServo)
  {
    //__debugS(PSTR("Stepper home SERVO variant"));
    // don't release the servo when homing the Feeder but
    // release it when homing something else
    if (index != FEEDER)
      setServoLid(SERVO_OPEN);
    // Revolver isn't being used on a servo variant
    if (index != REVOLVER)
      steppers[index].home();
  }
  else
  {
    //__debugS(PSTR("Stepper home non SERVO variant"));
    // not a servo variant, home stepper which ever it is
    if (index != REVOLVER)
      steppers[index].home();
    else {
      uint8_t retries = 5;
      do {
        steppers[index].home();
        if(steppers[index].getEndstopHit())
          break;
        delay(250);
        retries--;
        if(retries == 0)
          break;
        __debugS(PSTR("Revolver not homed, retrying: %d"), retries);
      } while(1);
    }

    #if defined(SMUFF_V6S)
    if (index == REVOLVER) {
      steppers[index].setEnabled(false);    // turn off LID stepper when done
      if(steppers[index].getEndstopHit())
        lidOpen = true;
    }
    #endif
  }

  //__debugS(PSTR("DONE Stepper home"));
  if (index == SELECTOR)
  {
    setFastLEDToolIndex(toolSelected, 0, true);
    toolSelected = -1;
  }
  long pos = steppers[index].getStepPosition();
  if (index == SELECTOR || index == REVOLVER)
    dataStore.tool = toolSelected;
  dataStore.stepperPos[index] = pos;
  saveStore();
  //__debugS(PSTR("DONE save store"));
  setParserReady();
  #if defined(SMUFF_V6S)
    return lidOpen;
  #endif
  return true;
}

bool showFeederBlockedMessage()
{
  bool state = false;
  lastEncoderButtonTime = millis();
  beep(1);
  uint8_t button = showDialog(P_TitleWarning, P_FeederLoaded, P_RemoveMaterial, P_CancelRetryButtons);
  if (button == 1)
  {
    drawStatus();
    unloadFilament();
    state = true;
  }
  display.clearDisplay();
  return state;
}

bool showFeederLoadedMessage()
{
  bool state = false;
  lastEncoderButtonTime = millis();
  beep(1);
  uint8_t button = showDialog(P_TitleWarning, P_FeederLoaded, P_AskUnload, P_YesNoButtons);
  if (button == 1)
  {
    drawStatus();
    unloadFilament();
    state = true;
  }
  display.clearDisplay();
  return state;
}

bool showFeederLoadMessage()
{
  bool state = false;
  lastEncoderButtonTime = millis();
  beep(1);
  uint8_t button = showDialog(P_TitleSelected, P_SelectedTool, P_AskLoad, P_YesNoButtons);
  if (button == 1)
  {
    drawStatus();
    if (smuffConfig.prusaMMU2)
      loadFilamentPMMU2();
    else
      loadFilament();
    state = true;
  }
  display.clearDisplay();
  return state;
}

bool showFeederFailedMessage(int8_t state)
{
  lastEncoderButtonTime = millis();
  beep(3);
  uint8_t button = 99;
  isWarning = true;
  do
  {
    button = showDialog(P_TitleWarning, state == 1 ? P_CantLoad : P_CantUnload, P_CheckUnit, P_CancelRetryButtons);
  } while (button != 1 && button != 2);
  isWarning = false;
  display.clearDisplay();
  debounceButton();
  return button == 1 ? false : true;
}

uint8_t showDialog(PGM_P title, PGM_P message, PGM_P addMessage, PGM_P buttons)
{
  //__debugS(PSTR("showDialog: %S"), title);
  if (isPwrSave)
  {
    setPwrSave(0);
  }
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

void signalNoTool()
{
  char _msg1[30];
  userBeep();
  sprintf_P(_msg1, P_NoTool);
  strcat_P(_msg1, P_Aborting);
  drawUserMessage(_msg1);
}

void switchFeederStepper(uint8_t stepper)
{
  if (RELAY_PIN == -1)
    return;
  steppers[FEEDER].setEnabled(false);
  digitalWrite(RELAY_PIN, stepper == EXTERNAL ? smuffConfig.invertRelay : !smuffConfig.invertRelay);
  smuffConfig.externalStepper = stepper == EXTERNAL;
  delay(250);   // gain the relay some time to debounce
}

void moveFeeder(float distanceMM)
{
  steppers[FEEDER].setEnabled(true);
  uint16_t curSpeed = steppers[FEEDER].getMaxSpeed();
  changeFeederSpeed(smuffConfig.insertSpeed);
  prepSteppingRelMillimeter(FEEDER, distanceMM, true);
  runAndWait(FEEDER);
  steppers[FEEDER].setMaxSpeed(curSpeed);
}

void positionRevolver()
{
  // disable Feeder temporarily
  steppers[FEEDER].setEnabled(false);
  #if !defined(SMUFF_V6S)
  if (smuffConfig.resetBeforeFeed && !ignoreHoming)
  {
    if (smuffConfig.revolverIsServo)
      setServoLid(SERVO_OPEN);
    else
      moveHome(REVOLVER, false, false);
  }
  if (smuffConfig.revolverIsServo)
  {
    setServoLid(SERVO_CLOSED);
    steppers[FEEDER].setEnabled(true);
    return;
  }
  #endif

  #if defined(SMUFF_V6S)
    float newPos = stepperPosClosed[toolSelected];
    if(newPos > 0) {
      steppers[REVOLVER].setEnabled(true);   // turn on the Lid stepper
      prepSteppingRelMillimeter(REVOLVER, newPos, true); // go to position, don't mind the endstop
      //__debugS(PSTR("Position Revolver, pos=%f"), newPos);
      remainingSteppersFlag |= _BV(REVOLVER);
      runAndWait(REVOLVER);
    }
  #else
  long pos = steppers[REVOLVER].getStepPosition();
  long newPos = smuffConfig.firstRevolverOffset + (toolSelected * smuffConfig.revolverSpacing);
  // calculate the new position and decide whether to move forward or backard
  // i.e. which ever has the shorter distance
  long delta1 = newPos - (smuffConfig.stepsPerRevolution + pos); // number of steps if moved backward
  long delta2 = newPos - pos;                                    // number of steps if moved forward
  if (abs(delta1) < abs(delta2))
    newPos = delta1;
  else
    newPos = delta2;

  // if the position hasn't changed, do nothing
  if (newPos != 0)
  {
    prepSteppingRel(REVOLVER, newPos, true); // go to position, don't mind the endstop
    remainingSteppersFlag |= _BV(REVOLVER);
    runAndWait(-1);
    if (smuffConfig.wiggleRevolver)
    {
      // wiggle the Revolver one position back and forth
      // just to adjust the gears a bit better
      delay(50);
      prepSteppingRel(REVOLVER, smuffConfig.revolverSpacing, true);
      remainingSteppersFlag |= _BV(REVOLVER);
      runAndWait(-1);
      delay(50);
      prepSteppingRel(REVOLVER, -(smuffConfig.revolverSpacing), true);
      remainingSteppersFlag |= _BV(REVOLVER);
      runAndWait(-1);
    }
  }
  #endif
  steppers[FEEDER].setEnabled(true);
  delay(150);
  #if defined(SMUFF_V6S)
  if(toolSelected != -1)
    lidOpen = false;
  steppers[REVOLVER].setEnabled(false);   // turn off the Lid stepper
  #endif
  //__debugS(PSTR("PositionRevolver: pos: %d"), steppers[REVOLVER].getStepPosition());
}

void changeFeederSpeed(uint16_t speed)
{
  unsigned long maxSpeed = translateSpeed(speed, FEEDER);
  //__debugS(PSTR("Changing Feeder speed to %3d (%6ld ticks)"), speed, maxSpeed);
  steppers[FEEDER].setMaxSpeed(maxSpeed);
}

void repositionSelector(bool retractFilament)
{
  int8_t tool = getToolSelected();
  if (retractFilament && !smuffConfig.revolverIsServo)
  {
    char tmp[15];
    uint16_t curSpeed = steppers[FEEDER].getMaxSpeed();
    changeFeederSpeed(smuffConfig.insertSpeed);
    ignoreHoming = true;
    // go through all tools available and retract some filament
    for (uint8_t i = 0; i < smuffConfig.toolCount; i++)
    {
      if (i == tool)
        continue;
      sprintf(tmp, "Y%d", i);
      G0("G0", tmp, 255);                                                   // position Revolver on tool
      prepSteppingRelMillimeter(FEEDER, -smuffConfig.insertLength, true); // retract
      runAndWait(FEEDER);
    }
    ignoreHoming = false;
    sprintf(tmp, "Y%d", tool);
    G0("G0", tmp, 255); // position Revolver on tool selected
    steppers[FEEDER].setMaxSpeed(curSpeed);
  }
  moveHome(SELECTOR, false, false); // home Revolver
  selectTool(tool, false);          // reposition Selector
}

bool feedToEndstop(bool showMessage)
{
  // enable steppers if they were turned off
  if (!steppers[FEEDER].getEnabled())
    steppers[FEEDER].setEnabled(true);

  // don't allow "feed to endstop" being interrupted
  steppers[FEEDER].setIgnoreAbort(true);

  positionRevolver();

  uint16_t curSpeed = steppers[FEEDER].getMaxSpeed();
  //__debugS(PSTR("InsertSpeed: %d"), smuffConfig.insertSpeed);
  uint16_t speed = smuffConfig.insertSpeed;
  changeFeederSpeed(speed);
  if (smuffConfig.accelSpeed[FEEDER] > smuffConfig.insertSpeed)
    steppers[FEEDER].setAllowAccel(false);

  uint16_t max = (uint16_t)(smuffConfig.selectorDistance * 2); // calculate a maximum distance to avoid feeding endlesly
  uint8_t n = 0;
  int8_t retries = FEED_ERROR_RETRIES; // max. retries for this operation

  feederJammed = false;

  // is the feeder endstop already being triggered?
  if (feederEndstop())
  {
    // yes, filament is still fed, unload completelly and
    // abort this operation if that fails
    if (!unloadFromNozzle(showMessage))
      return false;
  }

  steppers[FEEDER].setStepPositionMM(0);
  // as long as Selector endstop doesn't trigger
  // feed the configured insertLength
  while (!feederEndstop())
  {
    prepSteppingRelMillimeter(FEEDER, smuffConfig.insertLength, false);
    runAndWait(FEEDER);
    // has the endstop already triggered?
    if (feederEndstop())
    {
      //__debugS(PSTR("Position now: %s"), String(steppers[FEEDER].getStepPositionMM()).c_str());
      break;
    }
    n += smuffConfig.insertLength; // increment the position of the filament
    // did the Feeder stall (TMC2209 only)?
    bool stallStat = handleFeederStall(&speed, &retries);
    // if endstop hasn't triggered yet, feed was not successful
    if (n >= max && !feederEndstop())
    {
      delay(250);
      // retract half a insertLength and reset the Revolver
      prepSteppingRelMillimeter(FEEDER, -(smuffConfig.insertLength / 2), true);
      runAndWait(FEEDER);
      resetRevolver();
      feederErrors++; // global counter used for testing only
      if (stallStat)  // did not stall means no retries decrement, though, the endstop hasn't triggered yet
        retries--;
      // if only two retries are left, try repositioning the Selector
      if (retries == 1)
      {
        repositionSelector(false);
      }
      // if only one retry is left, rectract filaments a bit and try repositioning the Selector
      if (retries == 0)
      {
        repositionSelector(true);
      }
      // close lid servo in case it got openend by the reposition operation
      if (smuffConfig.revolverIsServo)
        setServoLid(SERVO_CLOSED);
      n = 0;
    }
    //__debugS(PSTR("Max: %s  N: %s  Retries: %d  Endstop: %d"), String(max).c_str(), String(n).c_str(), retries, feederEndstop());
    if (!feederEndstop() && retries < 0)
    {
      // still got no endstop trigger, abort action
      if (showMessage)
      {
        moveHome(REVOLVER, false, false); // home Revolver
        M18("M18", "", 255);              // turn all motors off
        if (smuffConfig.revolverIsServo)  // release servo, if used
          setServoLid(SERVO_OPEN);
        // if user wants to retry...
        if (showFeederFailedMessage(1) == true)
        {
          // reset and start over again
          steppers[FEEDER].setEnabled(true);
          positionRevolver();
          n = 0;
          retries = FEED_ERROR_RETRIES;
          continue;
        }
      }
      // otherwise, assume the feeder is jammed
      feederJammed = true;
      break;
    }
  }
  steppers[FEEDER].setIgnoreAbort(false);
  steppers[FEEDER].setAllowAccel(true);
  steppers[FEEDER].setMaxSpeed(curSpeed);
  delay(300);
  return feederJammed ? false : true;
}

bool handleFeederStall(uint16_t *speed, int8_t *retries)
{
  bool stat = true;
  // did the Feeder stall?
  if (smuffConfig.stepperStopOnStall[FEEDER] && steppers[FEEDER].getStallDetected())
  {
    stat = false;
    int16_t newSpeed;
    // yes, turn the speed down by 25%
    if (smuffConfig.speedsInMMS)
    {
      // speeds in mm/s need to go down
      newSpeed = (int16_t)(*speed * 0.75);
      if (newSpeed > 0)
        *speed = newSpeed;
      else
        *speed = 1; // set speed to absolute minimum
    }
    else
    {
      // whereas speeds in timer ticks need to go up
      newSpeed = (int16_t)(*speed * 1.25);
      if (newSpeed < 65500)
        *speed = newSpeed;
      else
        *speed = 65500; // set speed to absolute minimum
    }
    *retries -= 1;
    changeFeederSpeed(*speed);
    __debugS(PSTR("Feeder has stalled, slowing down speed to %d"), *speed);
    // counter used in testRun
    stallDetectedCountFeeder++; // for testrun only
  }
  return stat;
}

bool feedToNozzle(bool showMessage)
{
  bool stat = true;
  uint16_t speed = smuffConfig.maxSpeed[FEEDER];
  int8_t retries = FEED_ERROR_RETRIES;

  if (smuffConfig.prusaMMU2 && smuffConfig.enableChunks)
  {
    // prepare to feed full speed in chunks
    float bLen = smuffConfig.bowdenLength;
    float len = bLen / smuffConfig.feedChunks;
    for (uint8_t i = 0; i < smuffConfig.feedChunks; i++)
    {
      prepSteppingRelMillimeter(FEEDER, len, true);
      runAndWait(FEEDER);
    }
  }
  else
  {
    float len = smuffConfig.bowdenLength * .95;
    float remains = 0;
    steppers[FEEDER].setStepPositionMM(0);
    // prepare 95% to feed full speed
    do
    {
      prepSteppingRelMillimeter(FEEDER, len - remains, true);
      runAndWait(FEEDER);
      // did the Feeder stall?
      stat = handleFeederStall(&speed, &retries);
      if (!stat)
      {
        remains = steppers[FEEDER].getStepPositionMM();
        __debugS(PSTR("Len: %s  Remain: %s  To go: %s"), String(len).c_str(), String(remains).c_str(), String(len - remains).c_str());
      }
      // check whether the 2nd endstop has triggered as well if configured to do so
      if (Z_END2_PIN != -1 && smuffConfig.useEndstop2 && stat)
      {
        if (!steppers[FEEDER].getEndstopHit(2))
          stat = false;
        if (!stat)
        {
          remains = steppers[FEEDER].getStepPositionMM();
          __debugS(PSTR("E-Stop2 failed. Len: %s  Remain: %s  To go: %s"), String(len).c_str(), String(remains).c_str(), String(len - remains).c_str());
        }
      }
    } while (!stat && retries > 0);
    if (stat)
    {
      retries = FEED_ERROR_RETRIES;
      speed = smuffConfig.insertSpeed;
      float len = smuffConfig.bowdenLength * .05;
      float remains = 0;
      steppers[FEEDER].setStepPositionMM(0);
      // rest of it feed slowly
      do
      {
        changeFeederSpeed(speed);
        prepSteppingRelMillimeter(FEEDER, len - remains, true);
        runAndWait(FEEDER);
        // did the Feeder stall again?
        stat = handleFeederStall(&speed, &retries);
        if (!stat)
        {
          remains = steppers[FEEDER].getStepPositionMM();
          __debugS(PSTR("Len: %s  Remain: %s  To go: %s"), String(len).c_str(), String(remains).c_str(), String(len - remains).c_str());
        }
      } while (!stat && retries > 0);
    }
  }
  return stat;
}

bool loadFilament(bool showMessage)
{
  signalDuetBusy();
  if (toolSelected == -1)
  {
    signalNoTool();
    signalDuetReady();
    return false;
  }
  if (smuffConfig.extControlFeeder)
  {
    if (!smuffConfig.isSharedStepper)
    {
      positionRevolver();
      signalDuetReady();
      return true;
    }
    else
    {
      switchFeederStepper(INTERNAL);
    }
  }
  positionRevolver();

  if (smuffConfig.useCutter)
  {
    // release the cutter just in case
    setServoPos(SERVO_CUTTER, smuffConfig.cutterOpen);
  }

  setParserBusy();
  uint16_t curSpeed = steppers[FEEDER].getMaxSpeed();
  // move filament until it hits the feeder endstop
  if (!feedToEndstop(showMessage))
    return false;

  steppers[FEEDER].setStepsTaken(0);
  // move filament until it gets to the nozzle
  if (!feedToNozzle(showMessage))
    return false;

  if (smuffConfig.reinforceLength > 0 && !steppers[FEEDER].getAbort())
  {
    resetRevolver();
    changeFeederSpeed(smuffConfig.insertSpeed);
    delay(150);
    prepSteppingRelMillimeter(FEEDER, smuffConfig.reinforceLength, true);
    runAndWait(FEEDER);
  }

  purgeFilament();
  steppers[FEEDER].setMaxSpeed(curSpeed);
  dataStore.stepperPos[FEEDER] = steppers[FEEDER].getStepPosition();
  saveStore();
  runHomeAfterFeed();
  signalDuetReady();
  setParserReady();
  return true;
}

void purgeFilament() {
  if (smuffConfig.usePurge && smuffConfig.purgeLength > 0 && !steppers[FEEDER].getAbort())
  {
    positionRevolver();
    uint16_t curSpeed = steppers[FEEDER].getMaxSpeed();
    changeFeederSpeed(smuffConfig.purgeSpeed);
    uint16_t len = smuffConfig.purgeLength;
    if (smuffConfig.purges[toolSelected] > 100)
    {
      len *= ((float)smuffConfig.purges[toolSelected] / 100);
    }
    drawPurgingMessage(len, toolSelected);
    prepSteppingRelMillimeter(FEEDER, len, true);
    runAndWait(FEEDER);
    delay(250);
    steppers[FEEDER].setMaxSpeed(curSpeed);
  }
}

/*
  This method is used to feed the filament Prusa style (L command on MMU2).
  If first feeds the filament until the endstop is hit, then
  it pulls it back again.
*/
bool loadFilamentPMMU2(bool showMessage)
{
  signalDuetBusy();
  if (toolSelected == -1)
  {
    signalNoTool();
    signalDuetReady();
    return false;
  }
  if (smuffConfig.extControlFeeder && !smuffConfig.isSharedStepper)
  {
    positionRevolver();
    signalDuetReady();
    return true;
  }
  if (smuffConfig.extControlFeeder && smuffConfig.isSharedStepper)
  {
    switchFeederStepper(INTERNAL);
  }
  positionRevolver();

  if (smuffConfig.useCutter)
  {
    // release the cutter just in case
    setServoPos(SERVO_CUTTER, smuffConfig.cutterOpen);
  }

  setParserBusy();
  uint16_t curSpeed = steppers[FEEDER].getMaxSpeed();
  // move filament until it hits the feeder endstop
  if (!feedToEndstop(showMessage)) {
    return false;
  }

  steppers[FEEDER].setStepsTaken(0);
  // inhibit interrupts at this step
  steppers[FEEDER].setIgnoreAbort(true);
  // now pull it back again
  changeFeederSpeed(smuffConfig.insertSpeed);
  prepSteppingRelMillimeter(FEEDER, -smuffConfig.selectorDistance, true);
  runAndWait(FEEDER);

  if (smuffConfig.reinforceLength > 0 && !steppers[FEEDER].getAbort())
  {
    resetRevolver();
    prepSteppingRelMillimeter(FEEDER, smuffConfig.reinforceLength, true);
    runAndWait(FEEDER);
  }
  steppers[FEEDER].setIgnoreAbort(false);

  steppers[FEEDER].setMaxSpeed(curSpeed);
  dataStore.stepperPos[FEEDER] = steppers[FEEDER].getStepPosition();
  saveStore();
  runHomeAfterFeed();
  signalDuetReady();
  setParserReady();
  return true;
}

bool unloadFromNozzle(bool showMessage)
{
  bool stat = true;
  uint16_t speed = smuffConfig.maxSpeed[FEEDER];
  int8_t retries = FEED_ERROR_RETRIES;
  float posNow = steppers[FEEDER].getStepPositionMM();

  if (smuffConfig.prusaMMU2 && smuffConfig.enableChunks)
  {
    __debugS(PSTR("Unloading in %d chunks "), smuffConfig.feedChunks);
    // prepare to unfeed 3 times the bowden length full speed in chunks
    float bLen = -smuffConfig.bowdenLength * 3;
    float len = bLen / smuffConfig.feedChunks;
    for (uint8_t i = 0; i < smuffConfig.feedChunks; i++)
    {
      prepSteppingRelMillimeter(FEEDER, len);
      runAndWait(FEEDER);
    }
  }
  else
  {
    do
    {
      // prepare 110% to retract with full speed
      prepSteppingRelMillimeter(FEEDER, -(smuffConfig.bowdenLength * 1.1));
      runAndWait(FEEDER);
      // did the Feeder stall?
      stat = handleFeederStall(&speed, &retries);
      // check whether the 2nd endstop has triggered as well if configured to do so
      if (Z_END2_PIN != -1 && smuffConfig.useEndstop2 && stat)
      {
        // endstop still triggered?
        if (steppers[FEEDER].getEndstopHit(2))
        {
          retries--;
          stat = false;
          __debugS(PSTR("E-Stop2 failed. Retrying"));
          // if only one retry is left, try cutting the filament again (if the cutter is configured)
          if (retries == 1)
          {
            cutFilament();
          }
        }
      }
    } while (!stat && retries > 0);
  }
  // do some calculations in order to give some hints
  float posEnd = posNow - steppers[FEEDER].getStepPositionMM();
  // calculate supposed length;
  float dist = smuffConfig.bowdenLength + smuffConfig.cutterLength - smuffConfig.unloadRetract;
  if (posEnd < dist - 5 || posEnd > dist + 5)
  {
    __debugS(PSTR("HINT: Feeder endstop triggered after %s mm where it's supposed to be %s mm"), String(posEnd).c_str(), String(dist).c_str());
    __debugS(PSTR("Params: Bowden length: %s mm; Unload retract: -%s mm"), String(smuffConfig.bowdenLength).c_str(), String(smuffConfig.unloadRetract).c_str());
    __debugS(PSTR("Suggestion: Try adjusting BowdenLength to %s mm"), String(smuffConfig.bowdenLength + (posEnd - dist)).c_str());
  }
  // reset feeder to max. speed
  changeFeederSpeed(smuffConfig.maxSpeed[FEEDER]);
  delay(500);
  return stat;
}

bool unloadFromSelector()
{
  bool stat = true;
  // only if the unload hasn't been aborted yet, unload from Selector
  if (steppers[FEEDER].getAbort() == false)
  {
    int8_t retries = FEED_ERROR_RETRIES;
    uint16_t speed = smuffConfig.insertSpeed;
    do
    {
      // retract the selector distance
      changeFeederSpeed(speed);
      prepSteppingRelMillimeter(FEEDER, -smuffConfig.selectorDistance, true);
      runAndWait(FEEDER);
      // did the Feeder stall?
      stat = handleFeederStall(&speed, &retries);
    } while (!stat && retries > 0);
  }
  return stat;
}

void wipeNozzle()
{
  if (smuffConfig.wipeBeforeUnload)
    G12("G12", "", 255);
}

void cutFilament(bool keepClosed /* = true */)
{
  // use the filament cutter?
  if (smuffConfig.useCutter)
  {
    // run the cutter 3 times to be on the save side
    setServoPos(SERVO_CUTTER, smuffConfig.cutterClose);
    delay(500);
    setServoPos(SERVO_CUTTER, smuffConfig.cutterOpen);
    delay(300);
    setServoPos(SERVO_CUTTER, smuffConfig.cutterClose);
    delay(300);
    setServoPos(SERVO_CUTTER, smuffConfig.cutterOpen);
    delay(200);
    setServoPos(SERVO_CUTTER, smuffConfig.cutterClose);
    if (!keepClosed)
    {
      delay(200);
      setServoPos(SERVO_CUTTER, smuffConfig.cutterOpen);
    }
  }
}

bool unloadFilament()
{
  signalDuetBusy();
  if (toolSelected == -1)
  {
    signalNoTool();
    signalDuetReady();
    return false;
  }
  if (smuffConfig.extControlFeeder && !smuffConfig.isSharedStepper)
  {
    positionRevolver();
    signalDuetReady();
    return true;
  }
  else if (smuffConfig.extControlFeeder && smuffConfig.isSharedStepper)
  {
    switchFeederStepper(INTERNAL);
  }
  steppers[FEEDER].setStepsTaken(0);
  setParserBusy();
  if (!steppers[FEEDER].getEnabled())
    steppers[FEEDER].setEnabled(true);

  positionRevolver();

  uint16_t curSpeed = steppers[FEEDER].getMaxSpeed();

  if (smuffConfig.unloadRetract != 0)
  {
    __debugS(PSTR("Unload retract: %s mm"), String(smuffConfig.unloadRetract).c_str());
    prepSteppingRelMillimeter(FEEDER, -smuffConfig.unloadRetract, true);
    runAndWait(FEEDER);
    if (smuffConfig.unloadPushback != 0)
    {
      __debugS(PSTR("Unload pushback: %s mm"), String(smuffConfig.unloadPushback).c_str());
      changeFeederSpeed(smuffConfig.insertSpeed);
      prepSteppingRelMillimeter(FEEDER, smuffConfig.unloadPushback, true);
      runAndWait(FEEDER);
      delay(smuffConfig.pushbackDelay * 1000);
      steppers[FEEDER].setMaxSpeed(curSpeed);
    }
  }
  // wipe nozzle if configured
  wipeNozzle();
  // cut the filament if configured likewise
  cutFilament();

  // invert endstop trigger state for unloading
  bool endstopState = steppers[FEEDER].getEndstopState();
  steppers[FEEDER].setEndstopState(!endstopState);
  // unload until Selector endstop gets released
  int8_t retries = FEED_ERROR_RETRIES;
  do
  {
    unloadFromNozzle(false);
    if (steppers[FEEDER].getEndstopHit())
      break;
    else
      retries--;
  } while (retries > 0);
  // reset endstop state
  steppers[FEEDER].setEndstopState(endstopState);

  unloadFromSelector();

  if (smuffConfig.useCutter)
    setServoPos(SERVO_CUTTER, smuffConfig.cutterOpen); // release the cutter

  feederJammed = false;
  steppers[FEEDER].setIgnoreAbort(false);
  steppers[FEEDER].setMaxSpeed(curSpeed);
  steppers[FEEDER].setStepPosition(0);
  steppers[FEEDER].setAbort(false);

  dataStore.stepperPos[FEEDER] = steppers[FEEDER].getStepPosition();
  saveStore();
  runHomeAfterFeed();
  signalDuetReady();
  setParserReady();
  return true;
}

bool nudgeBackFilament()
{
  if (toolSelected == -1)
  {
    return false;
  }
  if (smuffConfig.extControlFeeder && !smuffConfig.isSharedStepper)
  {
    positionRevolver();
    signalDuetReady();
    return true;
  }
  else if (smuffConfig.extControlFeeder && smuffConfig.isSharedStepper)
  {
    switchFeederStepper(INTERNAL);
  }
  steppers[FEEDER].setStepsTaken(0);
  if (!steppers[FEEDER].getEnabled())
    steppers[FEEDER].setEnabled(true);

  positionRevolver();

  uint16_t curSpeed = steppers[FEEDER].getMaxSpeed();
  changeFeederSpeed(smuffConfig.insertSpeed);
  prepSteppingRelMillimeter(FEEDER, smuffConfig.insertLength);
  runAndWait(FEEDER);
  steppers[FEEDER].setMaxSpeed(curSpeed);

  dataStore.stepperPos[FEEDER] = steppers[FEEDER].getStepPosition();
  saveStore();
  runHomeAfterFeed();
  return true;
}

void runHomeAfterFeed() {

  if (smuffConfig.homeAfterFeed)
  {
    if (smuffConfig.revolverIsServo)
      setServoLid(SERVO_OPEN);
    else
      steppers[REVOLVER].home();
  }
  steppers[FEEDER].setAbort(false);
  if (smuffConfig.extControlFeeder && smuffConfig.isSharedStepper)
    switchFeederStepper(EXTERNAL);
}

void handleStall(int8_t axis)
{
  const char P_StallHandler[] PROGMEM = {"Stall handler: %s"};
  __debugS(P_StallHandler, "Triggered on %c-axis", 'X' + axis);
  // check if stall must be handled
  if (steppers[axis].getStopOnStallDetected())
  {
    __debugS(P_StallHandler, "Stopped on stall");
    // save speed/acceleration settings
    uint16_t maxSpeed = steppers[axis].getMaxSpeed();
    uint16_t accel = steppers[axis].getAcceleration();
    // slow down speed/acceleration for next moves
    steppers[axis].setMaxSpeed(accel * 2);
    steppers[axis].setAcceleration(accel * 2);
    steppers[axis].setEnabled(false);

    // in order to determine where the stall happens...
    // try to move 5mm to the left
    delay(1000);
    steppers[axis].setEnabled(true);
    prepStepping(axis, 5, true, true);
    remainingSteppersFlag |= _BV(axis);
    runAndWait(-1);
    bool stallLeft = steppers[axis].getStallCount() > (uint32_t)steppers[axis].getStallThreshold();
    __debugS(PSTR("Left: %d"), steppers[axis].getStallCount());
    steppers[axis].setEnabled(false);
    delay(1000);
    steppers[axis].setEnabled(true);
    // try to move 5mm to the right
    prepStepping(axis, -5, true, true);
    remainingSteppersFlag |= _BV(axis);
    runAndWait(-1);
    bool stallRight = steppers[axis].getStallCount() > (uint32_t)steppers[axis].getStallThreshold();
    __debugS(PSTR("Right: %d"), steppers[axis].getStallCount());

    if (stallLeft && stallRight)
      __debugS(P_StallHandler, "Stalled center");
    if (stallLeft && !stallRight)
      __debugS(P_StallHandler, "Stalled left");
    if (!stallLeft && stallRight)
      __debugS(P_StallHandler, "Stalled right");

    nudgeBackFilament();
    __debugS(P_StallHandler, "Feeder nudged back");
    delay(1000);
    if (axis != FEEDER)
    { // Feeder can't be homed
      moveHome(axis, false, false);
      __debugS(P_StallHandler, "%c-Axis Homed", 'X' + axis);
    }
    else
    {
      // TODO: add stall handling for Feeder
    }
    // reset speed/acceleration
    steppers[axis].setMaxSpeed(maxSpeed);
    steppers[axis].setAcceleration(accel);
    delay(1000);
  }
}

bool selectTool(int8_t ndx, bool showMessage)
{
  char _msg1[256];
  char _tmp[40];

  if (ndx < 0 || ndx >= MAX_TOOLS)
  {
    if (showMessage)
    {
      userBeep();
      sprintf_P(_msg1, P_WrongTool, ndx);
      drawUserMessage(_msg1);
    }
    return false;
  }

  ndx = swapTools[ndx];
  if (feederJammed)
  {
    beep(4);
    sprintf_P(_msg1, P_FeederJammed);
    strcat_P(_msg1, P_Aborting);
    drawUserMessage(_msg1);
    feederJammed = false;
    return false;
  }
  //signalSelectorBusy();

  if (toolSelected == ndx)
  {
    // tool is the one we already have selected, do nothing
    if (!smuffConfig.extControlFeeder)
    {
      userBeep();
      sprintf_P(_msg1, P_ToolAlreadySet);
      drawUserMessage(_msg1);
    }
    /*
    if (smuffConfig.extControlFeeder)
    {
      signalSelectorReady();
    }
    */
    return true;
  }
  if (!steppers[SELECTOR].getEnabled())
    steppers[SELECTOR].setEnabled(true);

  if (showMessage)
  {
    while (feederEndstop())
    {
      if (!showFeederLoadedMessage())
        return false;
    }
  }
  else
  {
    if (!smuffConfig.extControlFeeder && feederEndstop())
    {
      unloadFilament();
    }
    else if (smuffConfig.extControlFeeder && feederEndstop())
    {
      beep(4);
      if (smuffConfig.sendActionCmds)
      {
        // send action command to indicate a jam has happend and
        // the controller shall wait
        sprintf_P(_tmp, P_Action, P_ActionWait);
        printResponse(_tmp, 0);
        printResponse(_tmp, 1);
        printResponse(_tmp, 2);
      }
      while (feederEndstop())
      {
        moveHome(REVOLVER, false, false); // home Revolver
        M18("M18", "", 255);              // motors off
        bool stat = showFeederFailedMessage(0);
        if (!stat)
          return false;
        if (smuffConfig.unloadCommand != nullptr && strlen(smuffConfig.unloadCommand) > 0)
        {
          if (CAN_USE_SERIAL2)
          {
            Serial2.print(smuffConfig.unloadCommand);
            Serial2.print("\n");
            //__debugS(PSTR("Feeder jammed, sent unload command '%s'\n"), smuffConfig.unloadCommand);
          }
        }
      }
      if (smuffConfig.sendActionCmds)
      {
        // send action command to indicate jam cleared, continue printing
        sprintf_P(_tmp, P_Action, P_ActionCont);
        printResponse(_tmp, 0);
        printResponse(_tmp, 1);
        printResponse(_tmp, 2);
      }
    }
  }
  if (smuffConfig.revolverIsServo)
  {
    // release servo prior moving the selector
    setServoLid(SERVO_OPEN);
  }
  else {
    #if defined(SMUFF_V6S)
    setServoLid(SERVO_OPEN);
    #endif
  }
  //__debugS(PSTR("Selecting tool: %d"), ndx);
  setParserBusy();
  drawSelectingMessage(ndx);
  //__debugS(PSTR("Message shown"));
  uint16_t speed = steppers[SELECTOR].getMaxSpeed();

  uint8_t retry = 3;
  bool posOk = false;
  do
  {
    steppers[SELECTOR].resetStallDetected();
    prepSteppingAbsMillimeter(SELECTOR, smuffConfig.firstToolOffset + (ndx * smuffConfig.toolSpacing));
    remainingSteppersFlag |= _BV(SELECTOR);
#if !defined(SMUFF_V5)
    if (!smuffConfig.resetBeforeFeed)
    {
      prepSteppingAbs(REVOLVER, smuffConfig.firstRevolverOffset + (ndx * smuffConfig.revolverSpacing), true);
      remainingSteppersFlag |= _BV(REVOLVER);
    }
    runAndWait(-1);
#else
    runAndWait(SELECTOR);
#endif
    //__debugS(PSTR("Selector in position: %d"), ndx);
    if (smuffConfig.stepperStall[SELECTOR] > 0)
    {
      //__debugS(PSTR("Selector stall count: %d"), steppers[SELECTOR].getStallCount());
    }
    if (steppers[SELECTOR].getStallDetected())
    {
      posOk = false;
      handleStall(SELECTOR);
      stallDetectedCountSelector++;
    }
    else
      posOk = true;
    retry--;
    if (!retry)
      break;
  } while (!posOk);
  steppers[SELECTOR].setMaxSpeed(speed);
  if (posOk)
  {
    toolSelected = ndx;
    dataStore.tool = toolSelected;
    dataStore.stepperPos[SELECTOR] = steppers[SELECTOR].getStepPosition();
    dataStore.stepperPos[REVOLVER] = steppers[REVOLVER].getStepPosition();
    dataStore.stepperPos[FEEDER] = steppers[FEEDER].getStepPosition();
    saveStore();
    setFastLEDToolIndex(toolSelected, smuffConfig.toolColor, true);

    if (showMessage && (!smuffConfig.extControlFeeder || (smuffConfig.extControlFeeder && smuffConfig.isSharedStepper)))
    {
      showFeederLoadMessage();
    }
    if (smuffConfig.extControlFeeder)
    {
      //__debugS(PSTR("Resetting Revolver"));
      resetRevolver();
      //signalSelectorReady();
      //__debugS(PSTR("Revolver reset done"));
    }
  }
  setParserReady();
  //__debugS(PSTR("Finished selecting tool"));
  return posOk;
}

void resetRevolver()
{
  moveHome(REVOLVER, false, false);
  if (toolSelected >= 0 && toolSelected <= smuffConfig.toolCount - 1)
  {
    if (!smuffConfig.revolverIsServo)
    {
      #if !defined(SMUFF_V6S)
      prepSteppingAbs(REVOLVER, smuffConfig.firstRevolverOffset + (toolSelected * smuffConfig.revolverSpacing), true);
      runAndWait(REVOLVER);
      #else
      positionRevolver();
      #endif
    }
    else
    {
      setServoLid(SERVO_CLOSED);
    }
  }
}

void setStepperSteps(int8_t index, long steps, bool ignoreEndstop)
{
  // make sure the servo is in off position before the Selector gets moved
  // ... just in case... you never know...
  if (smuffConfig.revolverIsServo && index == SELECTOR)
  {
    if (lidOpen)
    {
      //__debugS(PSTR("Positioning servo to: %d (OPEN)"), smuffConfig.revolverOffPos);
      setServoLid(SERVO_OPEN);
    }
  }
  if (steps != 0)
    steppers[index].prepareMovement(steps, ignoreEndstop);
}

void prepSteppingAbs(int8_t index, long steps, bool ignoreEndstop)
{
  long pos = steppers[index].getStepPosition();
  long _steps = steps - pos;
  setStepperSteps(index, _steps, ignoreEndstop);
}

void prepSteppingAbsMillimeter(int8_t index, float millimeter, bool ignoreEndstop)
{
  uint16_t stepsPerMM = steppers[index].getStepsPerMM();
  long steps = (long)((float)millimeter * stepsPerMM);
  long pos = steppers[index].getStepPosition();
  setStepperSteps(index, steps - pos, ignoreEndstop);
}

void prepSteppingRel(int8_t index, long steps, bool ignoreEndstop)
{
  setStepperSteps(index, steps, ignoreEndstop);
}

void prepSteppingRelMillimeter(int8_t index, float millimeter, bool ignoreEndstop)
{
  uint16_t stepsPerMM = steppers[index].getStepsPerMM();
  long steps = (long)((float)millimeter * stepsPerMM);
  setStepperSteps(index, steps, ignoreEndstop);
}

void printPeriodicalState(int8_t serial)
{
  char tmp[100];

  const char *_triggered = "on";
  const char *_open = "off";
  int8_t tool = getToolSelected();

  sprintf_P(tmp, PSTR("echo: states: T: T%d\tS: %s\tR: %s\tF: %s\tF2: %s\tTMC: %c%s\tSD: %s\tSC: %s\tLID: %s\n"),
            tool,
            selectorEndstop() ? _triggered : _open,
            revolverEndstop() ? _triggered : _open,
            feederEndstop(1) ? _triggered : _open,
            feederEndstop(2) ? _triggered : _open,
            isUsingTmc ? '+' : '-',
            tmcWarning ? _triggered : _open,
            sdRemoved ? _triggered : _open,
            settingsChanged ? _triggered : _open,
            !lidOpen ? _triggered : _open);
  printResponse(tmp, serial);
}

int8_t getToolSelected()
{
  int8_t tool;
  // rather return the swapped tool than the currently selected
  if (toolSelected >= 0 && toolSelected <= MAX_TOOLS)
    tool = swapTools[toolSelected];
  else
    tool = toolSelected;
  return tool;
}

void printDriverMode(int8_t serial)
{
  char tmp[128];

  sprintf_P(tmp, P_TMC_StatusAll,
            drivers[SELECTOR] == nullptr ? P_Unknown : drivers[SELECTOR]->stealth() ? P_Stealth
                                                                                    : P_Spread,
            drivers[REVOLVER] == nullptr ? P_Unknown : drivers[REVOLVER]->stealth() ? P_Stealth
                                                                                    : P_Spread,
            drivers[FEEDER] == nullptr ? P_Unknown : drivers[FEEDER]->stealth() ? P_Stealth
                                                                                : P_Spread);
  printResponse(tmp, serial);
}

void printDriverRms(int8_t serial)
{
  char tmp[128];

  sprintf_P(tmp, P_TMC_StatusAll,
            drivers[SELECTOR] == nullptr ? P_Unknown : (String(drivers[SELECTOR]->rms_current()) + String(P_MilliAmp)).c_str(),
            drivers[REVOLVER] == nullptr ? P_Unknown : (String(drivers[REVOLVER]->rms_current()) + String(P_MilliAmp)).c_str(),
            drivers[FEEDER] == nullptr ? P_Unknown : (String(drivers[FEEDER]->rms_current()) + String(P_MilliAmp)).c_str());
  printResponse(tmp, serial);
}

void printDriverMS(int8_t serial)
{
  char tmp[128];

  sprintf_P(tmp, P_TMC_StatusAll,
            drivers[SELECTOR] == nullptr ? P_Unknown : String(drivers[SELECTOR]->microsteps()).c_str(),
            drivers[REVOLVER] == nullptr ? P_Unknown : String(drivers[REVOLVER]->microsteps()).c_str(),
            drivers[FEEDER] == nullptr ? P_Unknown : String(drivers[FEEDER]->microsteps()).c_str());
  printResponse(tmp, serial);
}

void printDriverStallThrs(int8_t serial)
{
  char tmp[128];
  sprintf_P(tmp, P_TMC_StatusAll,
            drivers[SELECTOR] == nullptr ? P_Unknown : String(drivers[SELECTOR]->SGTHRS()).c_str(),
            drivers[REVOLVER] == nullptr ? P_Unknown : String(drivers[REVOLVER]->SGTHRS()).c_str(),
            drivers[FEEDER] == nullptr ? P_Unknown : String(drivers[FEEDER]->SGTHRS()).c_str());
  printResponse(tmp, serial);
}

void printEndstopState(int8_t serial)
{
  char tmp[128];
  const char *_triggered = "triggered";
  const char *_open = "open";
  sprintf_P(tmp, P_TMC_StatusAll,
            selectorEndstop() ? _triggered : _open,
            revolverEndstop() ? _triggered : _open,
            feederEndstop() ? _triggered : _open);
  printResponse(tmp, serial);
  if (Z_END2_PIN != -1)
  {
    sprintf_P(tmp, PSTR("Feeder2 (Z2): %s\n"), feederEndstop(2) ? _triggered : _open);
    printResponse(tmp, serial);
  }
}

void printDuetSignalStates(int8_t serial) {
  char tmp[80];
  const char *_high = "HIGH";
  const char *_low = "LOW";
  #if defined(DUET_SIG_SEL_PIN) && DUET_SIG_SEL_PIN != -1 && defined(DUET_SIG_FED_PIN) && DUET_SIG_FED_PIN != -1
  uint32_t selector = digitalRead(DUET_SIG_SEL_PIN);
  uint32_t feeder = digitalRead(DUET_SIG_FED_PIN);
  sprintf_P(tmp, P_Duet_StatusAll,
            selector ? _high : _low,
            feeder ? _high : _low);
  printResponse(tmp, serial);
  #else
  printResponseP(PSTR("Signal pins not defined!"), serial);
  #endif
}

void printSpeeds(int8_t serial)
{
  char tmp[150];

  sprintf_P(tmp, !smuffConfig.speedsInMMS ? P_AccelSpeedTicks : P_AccelSpeedMms,
            smuffConfig.maxSpeed[SELECTOR],
            smuffConfig.stepDelay[SELECTOR],
            smuffConfig.maxSpeed[REVOLVER],
            smuffConfig.stepDelay[REVOLVER],
            smuffConfig.maxSpeed[FEEDER],
            smuffConfig.extControlFeeder ? " (Ext.)" : " (Int.)",
            smuffConfig.stepDelay[FEEDER],
            smuffConfig.insertSpeed);
  printResponse(tmp, serial);
}

void printAcceleration(int8_t serial)
{
  char tmp[150];

  sprintf_P(tmp, !smuffConfig.speedsInMMS ? P_AccelSpeedTicks : P_AccelSpeedMms,
            smuffConfig.accelSpeed[SELECTOR],
            smuffConfig.stepDelay[SELECTOR],
            smuffConfig.accelSpeed[REVOLVER],
            smuffConfig.stepDelay[REVOLVER],
            smuffConfig.accelSpeed[FEEDER],
            smuffConfig.extControlFeeder ? " (Ext.)" : " (Int.)",
            smuffConfig.stepDelay[FEEDER],
            smuffConfig.insertSpeed);
  printResponse(tmp, serial);
}

void printSpeedAdjust(int8_t serial)
{
  char tmp[150];

  sprintf_P(tmp, P_SpeedAdjust,
            String(smuffConfig.speedAdjust[SELECTOR]).c_str(),
            String(smuffConfig.speedAdjust[SELECTOR]).c_str(),
            String(smuffConfig.speedAdjust[REVOLVER]).c_str());
  printResponse(tmp, serial);
}

void printOffsets(int8_t serial)
{
  char tmp[128];
  sprintf_P(tmp, P_TMC_StatusAll,
            String(smuffConfig.firstToolOffset).c_str(),
            String(smuffConfig.firstRevolverOffset).c_str(),
            "--");
  printResponse(tmp, serial);
}

#ifdef __STM32F1__
/*
  Simple wrapper for tone()
*/
void playTone(int8_t pin, int16_t freq, int16_t duration)
{
#if defined(USE_LEONERD_DISPLAY)
  encoder.playFrequency(freq, duration);
#else
  if (pin != -1)
    tone(pin, freq, duration);
#endif
}

void muteTone(int8_t pin)
{
#if defined(USE_LEONERD_DISPLAY)
  encoder.muteTone();
#else
  if (pin != -1)
    pinMode(pin, INPUT);
#endif
}
#endif

void beep(uint8_t count)
{
  prepareSequence(tuneBeep, false);
  for (uint8_t i = 0; i < count; i++)
  {
    playSequence();
  }
}

void longBeep(uint8_t count)
{
#if defined(USE_LEONERD_DISPLAY)
  encoder.setLED(LED_RED, true);
#endif
  prepareSequence(tuneLongBeep, false);
  for (uint8_t i = 0; i < count; i++)
  {
    playSequence();
  }
}

void userBeep()
{
#if defined(USE_LEONERD_DISPLAY)
  encoder.setLED(LED_RED, true);
#endif
  prepareSequence(tuneUser, false);
  playSequence();
}

void encoderBeep(uint8_t count)
{
  prepareSequence(tuneEncoder, false);
  playSequence();
}

void startupBeep()
{
#if defined(USE_LEONERD_DISPLAY)
  showLed(4, 1);
#endif
  prepareSequence(tuneStartup, true);
}

/*
  Prepares a tune sequence from string to an array of notes to be played in background.
  The format is: F{frequency} D{duration} [P{pause}].F{frequency}D{duration}[P{pause}]. ...
  Example: "F440D120P80." plays an A (440Hz) with a duration of 120mS and pauses 80mS
           after the tone has played.
           The '.' at the end of a tone is needed to play that tone and must not be omitted.
*/
void prepareSequence(const char *seq, bool autoPlay)
{
#if !defined(USE_LEONERD_DISPLAY)
  if (BEEPER_PIN == -1)
    return;
#endif
  if (seq == nullptr || *seq == 0)
    return;

  uint16_t f = 0, d = 0, p = 0;
  uint8_t n = 0;
  while (*seq)
  {
    if (*seq == '"' || *seq == ' ' || *seq=='\r' || *seq=='\n' || *seq=='\t') // skip quotes, spaces and newlines
      seq++;
    switch(toupper(*seq))
    {
      case 'F': f = atoi(++seq); break;
      case 'D': d = atoi(++seq); break;
      case 'P': p = atoi(++seq); break;
    }
    if (*seq == '.')
    {
      if (f && d)
      {
        sequence[n][_F_] = (uint16_t)f;
        sequence[n][_D_] = (uint16_t)d;
      }
      sequence[n][_P_] = (uint16_t)p;
      f = d = p = 0;
      if (n < MAX_SEQUENCE - 1)
        n++;
      else
        break;
    }
    seq++;
  }
  // mark end-of-sequence
  memset(&sequence[n], 0, 3*sizeof(uint16_t));
  if (autoPlay)
    playSequence();
}

void playSequence()
{
  for (uint8_t i = 0; i < MAX_SEQUENCE; i++)
  {
    if (sequence[i][_F_] == 0)
      return;
    _tone(sequence[i][_F_], sequence[i][_D_]);
    delay(sequence[i][_D_]);
    if (sequence[i][_P_] > 0)
      delay(sequence[i][_P_]);
  }
}

ZServo* getServoInstance(int8_t servoNum) {
  switch(servoNum) {
    case SERVO_WIPER:   return &servoWiper;
    case SERVO_LID:     return &servoLid;
    case SERVO_CUTTER:  return &servoCutter;
  }
  return nullptr;
}

void setServoMinPwm(int8_t servoNum, uint16_t pwm)
{
  ZServo* instance = getServoInstance(servoNum);
  if(instance != nullptr)
      instance->setPulseWidthMin(pwm);
}

void setServoMaxPwm(int8_t servoNum, uint16_t pwm)
{
  ZServo* instance = getServoInstance(servoNum);
  if(instance != nullptr)
      instance->setPulseWidthMax(pwm);
}

void disableServo(int8_t servoNum)
{
  ZServo* instance = getServoInstance(servoNum);
  if(instance != nullptr)
    instance->disable();
}

void enableServo(int8_t servoNum)
{
  ZServo* instance = getServoInstance(servoNum);
  if(instance != nullptr)
    instance->enable();
}

bool setServoPos(int8_t servoNum, uint8_t degree)
{
#if defined(MULTISERVO)
  uint16_t pulseLen = map(degree, 0, 180, smuffConfig.servoMinPwm, smuffConfig.servoMaxPwm);
  setServoMS(servoNum, pulseLen);
#else
  ZServo* instance = getServoInstance(servoNum);
  if(instance != nullptr) {
    instance->write(degree);
    instance->setDelay();
    return true;
  }
#endif
  return false;
}

bool setServoMS(int8_t servoNum, uint16_t microseconds)
{
#if defined(MULTISERVO)
  int8_t index = -1;
  switch(servoNum) {
    case SERVO_WIPER:   index = 16; break;
    case SERVO_CUTTER:  index = 17; break;
    default:
      if(servoNum >= 10 && servoNum <= 26)
        index = servoNum - 10;
      break;
  }
  if(index != -1) {
    //__debugS(PSTR("Servo mapping: %d -> %d (pulse len: %d ms)"), servoNum-10, index, microseconds);
    if(servoMapping[index] != -1)
      servoPwm.writeMicroseconds(servoMapping[index], microseconds);
    return true;
  }
#else
  ZServo* instance = getServoInstance(servoNum);
  if(instance != nullptr) {
    instance->writeMicroseconds(microseconds);
    instance->setDelay();
    return true;
  }
#endif
  return false;
}

void getStoredData()
{
  recoverStore();
  steppers[SELECTOR].setStepPosition(dataStore.stepperPos[SELECTOR]);
  steppers[REVOLVER].setStepPosition(dataStore.stepperPos[REVOLVER]);
  steppers[FEEDER].setStepPosition(dataStore.stepperPos[FEEDER]);
  toolSelected = dataStore.tool;
  setFastLEDToolIndex(toolSelected, smuffConfig.toolColor, true);
  //__debugS(PSTR("Recovered tool: %d"), toolSelected);
}

void setSignalPort(uint8_t port, bool state)
{
  /*
  char tmp[10];
  if (!smuffConfig.prusaMMU2 && !smuffConfig.sendPeriodicalStats)
  {
    sprintf_P(tmp, PSTR("%c%c%s"), 0x1b, port, state ? "1" : "0");
    if (CAN_USE_SERIAL1)
      Serial1.write(tmp);
    if (CAN_USE_SERIAL2)
      Serial2.write(tmp);
    if (CAN_USE_SERIAL3)
      Serial3.write(tmp);
  }
  */
  // used for Duet3D Controller boards to signal progress of loading / unloading
  if(port == FEEDER_SIGNAL) {
    #if defined(DUET_SIG_FED_PIN)
    if(DUET_SIG_FED_PIN != -1) {
      digitalWrite(DUET_SIG_FED_PIN, state);
      //__debugS(PSTR("Duet Feeder Signal %s"), state ? "HIGH" : "LOW");
    }
    #endif
  }
  if(port == SELECTOR_SIGNAL) {
    #if defined(DUET_SIG_SEL_PIN)
    if(DUET_SIG_SEL_PIN != -1) {
      digitalWrite(DUET_SIG_SEL_PIN, state);
      //__debugS(PSTR("Duet Selector Signal %s"), state ? "HIGH" : "LOW");
    }
    #endif
  }
}

void signalDuetBusy()
{
  setSignalPort(FEEDER_SIGNAL, true);
}

void signalDuetReady()
{
  setSignalPort(FEEDER_SIGNAL, false);
}


bool getFiles(const char *rootFolder PROGMEM, const char *pattern PROGMEM, uint8_t maxFiles, bool cutExtension, char *files)
{
  char fname[40];
  char tmp[25];
  uint8_t cnt = 0;
  SdFile file;
  SdFile root;
  SdFat SD;

  if (initSD(false))
  {
    root.open(rootFolder, O_READ);
    while (file.openNext(&root, O_READ))
    {
      if (!file.isHidden())
      {
        file.getName(fname, ArraySize(fname));
        //__debugS(PSTR("File: %s"), fname);
        String lfn = String(fname);
        if (pattern != nullptr && !lfn.endsWith(pattern))
        {
          file.close();
          continue;
        }
        if (pattern != nullptr && cutExtension)
          lfn.replace(pattern, "");
        sprintf_P(tmp, PSTR("%-20s\n"), lfn.c_str());
        strcat(files, tmp);
      }
      /*
      else {
        file.getName(fname, ArraySize(fname));
        __debugS(PSTR("Hidden file: %s"), fname);
      }
      */
      file.close();
      if (++cnt >= maxFiles)
        break;
    }
    root.close();
    files[strlen(files) - 1] = '\0';
    return true;
  }
  return false;
}

/*
  Removes file firmware.bin from root directory in order
  to prevent re-flashing on each reset!
*/
void removeFirmwareBin()
{
  if (initSD(false))
    SD.remove("firmware.bin");
}

void terminalDrawFrame(bool clear) {
  if(!smuffConfig.menuOnTerminal)
    return;

#if defined(USE_TERMINAL_MENUS)
  char vert[TERM_LINE_WIDTH+5];
  char horz[TERM_LINE_WIDTH+5];

  memset(horz, clear ? 0x20 : TERM_HORZLINE_CHR, TERM_LINE_WIDTH+4);
  memset(vert, 0x20, TERM_LINE_WIDTH+4);
  horz[TERM_LINE_WIDTH+4] = 0;
  vert[TERM_LINE_WIDTH+4] = 0;

  if(!clear) {
    horz[0] = (uint8_t)TERM_CORNERUL_CHR;  horz[TERM_LINE_WIDTH+3] = (uint8_t)TERM_CORNERUR_CHR;
    vert[0] = (uint8_t)TERM_VERTLINE_CHR;  vert[TERM_LINE_WIDTH+3] = (uint8_t)TERM_VERTLINE_CHR;
  }

  __terminal(P_SendTermAt, TERM_OFFS_Y, TERM_OFFS_X-1, horz);
  for(uint8_t i=1; i<= TERM_LINES; i++)
    __terminal(P_SendTermAt, TERM_OFFS_Y+i, TERM_OFFS_X-1, vert);

  if(!clear) {
    horz[0] = (uint8_t)TERM_CORNERLL_CHR;  horz[TERM_LINE_WIDTH+3] = (uint8_t)TERM_CORNERLR_CHR;
  }
  __terminal(P_SendTermAt, TERM_OFFS_Y+TERM_LINES+1, TERM_OFFS_X-1, horz);
#endif
}

void terminalDrawSeparator(uint8_t y, uint8_t x, uint8_t color) {
#if defined(USE_TERMINAL_MENUS)
  char ln[TERM_LINE_WIDTH+1];
  memset(ln, TERM_SEPARATOR_CHR, TERM_LINE_WIDTH);
  ln[TERM_LINE_WIDTH] = 0;
  __terminal(P_SendTermAttr, color);
  __terminal(P_SendTermAt, y+TERM_OFFS_Y, x+TERM_OFFS_X, ln);
#endif
}

void terminalClear(bool drawFrame) {
  if(!smuffConfig.menuOnTerminal)
    return;
#if defined(USE_TERMINAL_MENUS)
  static bool cursorSaved = false;
  if(drawFrame) {
    if(!cursorSaved)
      __terminal(P_SendTermCsrSave);  // save cursor position and attributes
    cursorSaved = true;
    __terminal(P_SendTermCsrHide);    // turn cursor off
    terminalDrawFrame();
  }
  else {
    terminalDrawFrame(true);
    if(cursorSaved)
      __terminal(P_SendTermCsrRestore);     // restore cursor position and switch it on again
    __terminal(P_SendTermCsrShow);
    cursorSaved = false;
  }
#endif
}

void terminalSend(uint8_t y, uint8_t x, const char* str, bool isCenter, uint8_t isInvert, bool clearLine) {
  if(!smuffConfig.menuOnTerminal)
    return;

#if defined(USE_TERMINAL_MENUS)
  char txt[TERM_LINE_WIDTH+1];
  memset(txt, ' ', TERM_LINE_WIDTH);
  uint8_t len = strlen(str);
  if(len > ArraySize(txt))
    len = ArraySize(txt);
  if(isCenter) {
    uint8_t pos = (TERM_LINE_WIDTH - len)/2;
    strncpy(&txt[pos], str, len);
  }
  else {
    strncpy(txt, str, len);
    if(!clearLine)
      txt[len] = 0;
  }
  txt[TERM_LINE_WIDTH] = 0;

  uint8_t color = 0, color2 = TERM_FGC_NONE;
  switch(isInvert) {
    case 1: color = TERM_INVERTED; break;
    case 2: color = TERM_FGC_CYAN; break;
    case 3: color = TERM_BGC_CYAN; break;
    case 4: color = TERM_FGC_MAGENTA; break;
    case 5: color = TERM_BGC_MAGENTA; break;
    case 6: color = TERM_UNDERLINE; color2 = TERM_FGC_CYAN; break;
  }
  if(clearLine) {
    char clr[TERM_LINE_WIDTH+2];
    memset(clr, 0x20, TERM_LINE_WIDTH+1);
    clr[TERM_LINE_WIDTH+1] = 0;
    __terminal(P_SendTermAt, y+TERM_OFFS_Y, TERM_OFFS_X, clr);
  }
  if(*txt == 0x1d) {
    terminalDrawSeparator(y, x, color);
  }
  else {
    __terminal(P_SendTermAttr,color);                               // set main attribute/color
    if(color2 != TERM_FGC_NONE) __terminal(P_SendTermAttr, color2); // set 2nd attribute/color if applicable
    __terminal(P_SendTermAt, y+TERM_OFFS_Y, x+TERM_OFFS_X, txt);    // print at position
  }
  __terminal(P_SendTermAttr, 0); // reset attributes
#endif
}

uint8_t terminalSendLines(uint8_t y, uint8_t x, const char* str, bool isCenter, uint8_t isInvert, bool clearLine) {

  if(!smuffConfig.menuOnTerminal)
    return 0;
#if defined(USE_TERMINAL_MENUS)
  char* lines[TERM_LINES+1];
  uint8_t ln = y;
  uint8_t lineCnt = splitStringLines(lines, ArraySize(lines), str);
  if(lineCnt==0)
    return ln;

  for(uint8_t i=0; i< lineCnt; i++) {
    if(strlen(lines[i])>0)
      terminalSend(ln, x, lines[i], isCenter, isInvert, clearLine);
    ln++;
  }
  return ln;
#else
  return y;
#endif
}

/*
  Reads the next line from the test script and filters unwanted
  characters.
*/
uint8_t getTestLine(SdFile *file, char *line, int maxLen)
{
  uint8_t n = 0;
  bool isQuote = false;
  while (1)
  {
    int c = file->read();
    if (c == -1)
      break;
    if (c == ' ' || c == '\r')
      continue;
    if (c == '\n')
    {
      *(line + n) = 0;
      break;
    }
    if (c == '"')
    {
      isQuote = !isQuote;
    }
    *(line + n) = isQuote ? c : toupper(c);
    n++;
    if (n >= maxLen)
      break;
  }
  return n;
}

extern SdFat SD;

void printReport(const char* line, unsigned long loopCnt, unsigned long cmdCnt, long toolChanges, unsigned long secs, unsigned long endstop2Hit[], unsigned long endstop2Miss[])
{
  char report[700];
  char runtm[20], gco[40], err[10], lop[10], cmdc[10], tc[20], stallS[10], stallF[10];
  char etmp[15], ehit[smuffConfig.toolCount * 10], emiss[smuffConfig.toolCount * 10];

  if(smuffConfig.webInterface) {
    // load JSON report when in WebInterface mode
    if(!loadReport(PSTR("report"), report, "json", ArraySize(report))) {
      __debugS(PSTR("Failed to load report.json"));
      return;
    }
  }
  else {
    // load report from SD-Card; Can contain 10 lines at max.
    if(!loadReport(PSTR("report"), report, nullptr, ArraySize(report))) {
      __debugS(PSTR("Failed to load report.txt"));
      return;
    }
  }

  if(!smuffConfig.webInterface) {
    sprintf_P(gco, PSTR("%-25s"), line);
    sprintf_P(err, PSTR("%4lu"), feederErrors);
    sprintf_P(lop, PSTR("%4lu"), loopCnt);
    sprintf_P(cmdc, PSTR("%5lu"), cmdCnt);
    sprintf_P(tc, PSTR("%5ld"), toolChanges);
    sprintf_P(stallS, PSTR("%4lu"), stallDetectedCountSelector);
    sprintf_P(stallF, PSTR("%4lu"), stallDetectedCountFeeder);
    sprintf_P(runtm, PSTR("%4d:%02d:%02d"), (int)(secs / 3600), (int)(secs / 60) % 60, (int)(secs % 60));
    memset(ehit, 0, ArraySize(ehit));
    memset(emiss, 0, ArraySize(emiss));
    for (uint8_t tcnt = 0; tcnt < smuffConfig.toolCount; tcnt++){
      sprintf_P(etmp, PSTR("%5lu | "), endstop2Hit[tcnt]);
      strcat(ehit, etmp);
      sprintf_P(etmp, PSTR("%5lu | "), endstop2Miss[tcnt]);
      strcat(emiss, etmp);
    }
  }
  else {
    sprintf_P(gco, PSTR("%s"), line);
    sprintf_P(err, PSTR("%lu"), feederErrors);
    sprintf_P(lop, PSTR("%lu"), loopCnt);
    sprintf_P(cmdc, PSTR("%lu"), cmdCnt);
    sprintf_P(tc, PSTR("%ld"), toolChanges);
    sprintf_P(stallS, PSTR("%lu"), stallDetectedCountSelector);
    sprintf_P(stallF, PSTR("%lu"), stallDetectedCountFeeder);
    sprintf_P(runtm, PSTR("%4d:%02d:%02d"), (int)(secs / 3600), (int)(secs / 60) % 60, (int)(secs % 60));
    memset(ehit, 0, ArraySize(ehit));
    memset(emiss, 0, ArraySize(emiss));
    for (uint8_t tcnt = 0; tcnt < smuffConfig.toolCount; tcnt++) {
      sprintf_P(etmp, PSTR("%lu|"), endstop2Hit[tcnt]);
      strcat(ehit, etmp);
      sprintf_P(etmp, PSTR("%lu|"), endstop2Miss[tcnt]);
      strcat(emiss, etmp);
    }

  }
  char *lines[10];
  // format report to be sent to terminal
  uint8_t cnt = splitStringLines(lines, ArraySize(lines), report);
  String s;
  for (uint8_t n = 0; n < cnt; n++)
  {
    s = String(lines[n]);
    s.replace("{TIME}", runtm);
    s.replace("{GCO}", gco);
    s.replace("{ERR}", err);
    s.replace("{LOOP}", lop);
    s.replace("{CMDS}", cmdc);
    s.replace("{TC}", tc);
    s.replace("{STALL}", stallS);
    s.replace("{STALLF}", stallF);
    s.replace("{HIT}", ehit);
    s.replace("{MISS}", emiss);
    if(!smuffConfig.webInterface) {
      s.replace("{ESC:", "\033[");
      s.replace("f}", "f");
      s.replace("m}", "m");
      s.replace("J}", "J");
      s.replace("H}", "H");
      __log(s.c_str());
    }
    else {
      debugSerial->print(PSTR("echo: testrun: "));
      debugSerial->println(s.c_str());
    }
  }
}

bool isTestrun = false;
bool isTestPending = false;
char testToRun[80];

void setTestRunPending(const char* testfile) {
  strncpy(testToRun, testfile, ArraySize(testToRun));
  isTestPending = true;
}

void testRun(const char *fname)
{
  char line[50];
  char msg[80];
  char filename[50];
  SdFile file;
  String gCode;
  unsigned long loopCnt = 1L, cmdCnt = 1L;
  int8_t tool = 0, lastTool = 0;
  uint8_t mode = 1;
  long toolChanges = 0;
  unsigned long startTime = millis();
  unsigned long endstop2Miss[smuffConfig.toolCount], endstop2Hit[smuffConfig.toolCount];
  int16_t turn;
  uint8_t btn;
  bool isHeld, isClicked;
  bool showReport;

  debounceButton();

  if (initSD(false))
  {
    steppers[REVOLVER].setEnabled(true);
    steppers[SELECTOR].setEnabled(true);
    steppers[FEEDER].setEnabled(true);
    sprintf_P(msg, P_RunningTest, fname);
    drawUserMessage(msg);
    delay(1750);
    randomSeed(millis());
    sprintf_P(filename, PSTR("/test/%s.gcode"), fname);
    feederErrors = 0;

    __log(P_SendTermCls); // clear screen on VT100
    terminalClear(true);
    for (uint8_t i = 0; i < smuffConfig.toolCount; i++)
    {
      endstop2Hit[i] = 0L;
      endstop2Miss[i] = 0L;
    }

    if (file.open(filename, O_READ))
    {
      fastLedStatus = FASTLED_STAT_NONE;

      stallDetectedCountFeeder = stallDetectedCountSelector = 0;

#if defined(USE_LEONERD_DISPLAY)
      encoder.setLED(LED_GREEN, true);
#endif
      isTestrun = true;
      isTestPending = false;
      while (1)
      {
        setPwrSave(0);
        showReport = false;
        checkSerialPending();
        getInput(&turn, &btn, &isHeld, &isClicked, false);
        if (isHeld || isClicked || !isTestrun)
        {
          break;
        }
        if (turn < 0)
        {
          if (--mode < 0)
            mode = 3;
        }
        else if (turn > 0)
        {
          if (++mode > 3)
            mode = 0;
        }
        unsigned long secs = (millis() - startTime) / 1000;
        uint8_t n = getTestLine(&file, line, ArraySize(line));
        if (n > 0)
        {
          if (*line == ';')
            continue;
          gCode = String(line);
          if(gCode.indexOf("{PRPT}") > -1) {
            printReport(line, loopCnt, cmdCnt, toolChanges, secs, endstop2Hit, endstop2Miss);
            continue;
          }
          if (gCode.indexOf("{RNDT}") > -1) {
            // randomize the next tool number in a loop to avoid
            // selecting the same tool that's currently set
            int8_t retry = smuffConfig.toolCount;
            do
            {
              tool = (int8_t)random(0, smuffConfig.toolCount);
              //__debugS(PSTR("retry: %d  tool: %d  last: %d"), retry, tool, lastTool);
              if (--retry < 0)
                break;
            } while(tool == lastTool);
            gCode.replace("{RNDT}", String(tool));
          }
          if (gCode.indexOf("{RNDTL}") > -1) {
            gCode.replace("{RNDTL}", String(tool));
          }
          parseGcode(gCode, -1);
          //__debugS(PSTR("GCode: %s"), gCode.c_str());
          if (*line == 'T')
          {
            tool = (int8_t)strtol(line + 1, nullptr, 10);
            lastTool = toolSelected;
            toolChanges++;
            showReport = true;
          }
          if (*line == 'C')
          {
            if (!feederEndstop(2))
              endstop2Hit[tool]++;
            else
              endstop2Miss[tool]++;
            showReport = true;
          }
          cmdCnt++;
          if (cmdCnt % 10 == 0)
          {
            mode++;
            if (mode > 3)
              mode = 0;
          }
          if (showReport)
            printReport(line, loopCnt, cmdCnt, toolChanges, secs, endstop2Hit, endstop2Miss);
        }
        else
        {
          randomSeed(analogRead(0));
          // restart from beginning and increment loop count
          file.rewind();
          //__debugS(PSTR("Rewinding"));
          loopCnt++;
        }
        switch (mode)
        {
          case 3:
            sprintf_P(msg, P_FeederErrors, feederErrors);
            break;
          case 2:
            sprintf_P(msg, P_ToolChanges, toolChanges);
            break;
          case 1:
            sprintf_P(msg, P_CmdLoop, cmdCnt, tool);
            break;
          case 0:
            sprintf_P(msg, P_TestTime, (int)(secs / 3600), (int)(secs / 60) % 60, (int)(secs % 60));
            break;
        }
        drawTestrunMessage(loopCnt, msg);
        delay(150);
        if (!showReport)
            printReport(line, loopCnt, cmdCnt, toolChanges, secs, endstop2Hit, endstop2Miss);
      }
      file.close();
#if defined(USE_LEONERD_DISPLAY)
      encoder.setLED(LED_GREEN, false);
#endif
    }
    else
    {
      sprintf_P(msg, P_TestFailed, fname);
      drawUserMessage(msg);
      delay(3000);
    }
  }
  #if defined(SMUFF_V6S)
  steppers[REVOLVER].setEnabled(false);
  #endif
  isTestrun = false;
}

volatile uint32_t lastYield = 0;

void yield() {
  if(millis()-lastYield > 20 && isTestrun) {
    isrFastLEDTimerHandler();
    lastYield = millis();
  }
}

volatile bool fastLedRefresh = false;

void refreshFastLED() {
#if defined(USE_FASTLED_TOOLS)
  if(!fastLedRefresh) {
    fastLedRefresh = true;
    cTools->showLeds();
    fastLedRefresh = false;
  }
#endif
}

void listHelpFile(const char *filename PROGMEM, int8_t serial)
{
  char fname[80];

  sprintf_P(fname, PSTR("help/%s.txt"), filename);
  printResponseP(P_Usage, serial);
  listTextFile(fname, serial);
}

void listTextFile(const char *filename PROGMEM, int8_t serial)
{
  SdFile file;
  char line[80];
  char delimiter[] = {"\n"};

  if (file.open(filename, O_READ))
  {
    while (file.fgets(line, sizeof(line) - 1, delimiter) > 0)
    {
      printResponse(line, serial);
    }
    file.close();
  }
  else
  {
    sprintf_P(line, P_FileNotFound, filename);
    printResponse(line, serial);
  }
  printResponse(delimiter, serial);
}


/**
 * Loads a menu from SD-Card.
 *
 * @param filename  the name of the menu file to load
 * @param ordinals  the menu entry ordinals
 * @returns the contents of that file
 */
const char *loadMenu(const char *filename PROGMEM, uint8_t ordinals[])
{
  SdFile file;
  static char menu[700];
  char fname[80];
  char ordinal[10];

  memset(menu, 0, ArraySize(menu));
  memset(ordinals, -1, MAX_MENU_ORDINALS * sizeof(int));
  sprintf_P(fname, PSTR("menus/%s.mnu"), filename);
  if (file.open(fname, O_READ))
  {
    uint16_t n = 0;
    uint8_t ln = 1;
    int c;
    while ((c = file.read()) != -1)
    {
      if (c == '\r')
        continue;
      // check for an separator indicating an ordinal number is following
      if (c == '|')
      {
        uint8_t on = 0;
        memset(ordinal, 0, ArraySize(ordinal));
        do
        {
          c = file.read();
          if (isdigit(c))
          {
            ordinal[on++] = c;
          }
          if (on >= (int)ArraySize(ordinal))
            break;
        } while (isdigit(c));
        ordinals[ln] = atoi(ordinal);
        //__debugS(PSTR("Ordinal found: %s = %d"), ordinal, ordinals[n]);
        continue;
      }
      menu[n] = c;
      if (c == '\n')
        ln++;
      // convert \n-\n to separator char (GS = Group Separator = \035)
      if (n > 2 && menu[n] == '\n' && menu[n - 1] == '-' && menu[n - 2] == '\n')
      {
        menu[n - 1] = '\035';
      }
      n++;
    }
    file.close();

    if (n == 0)
    {
      __debugS(PSTR("Failed to load menu '%s'"), filename);
    }
    //__debugS(PSTR("Menu: '%s' %d lines  %d bytes\n%s"), filename, ln, n, menu);
    return menu;
  }
  else
  {
    __debugS(P_FileNotFound, filename);
  }
  return nullptr;
}

const char *loadOptions(const char *filename PROGMEM)
{
  SdFile file;
  static char opts[300];
  char fname[80];

  memset(opts, 0, ArraySize(opts));
  sprintf_P(fname, PSTR("options/%s.opt"), filename);
  if (file.open(fname, O_READ))
  {
    int16_t n = 0;
    int c;
    while ((c = file.read()) != -1)
    {
      if (c == '\r')
        continue;
      opts[n] = c;
      n++;
    }
    file.close();
    if (n == 0)
    {
      __debugS(PSTR("Failed to load options '%s'"), filename);
    }
    //__debugS(PSTR("Opts: '%s' %d bytes"), filename, n);
    return opts;
  }
  else
  {
    __debugS(P_FileNotFound, filename);
  }
  return nullptr;
}

bool loadReport(const char *filename PROGMEM, char *buffer, const char* ext, uint16_t maxLen)
{
  SdFile file;
  char fname[40];

  memset(buffer, 0, maxLen);
  if(ext == nullptr)
    sprintf_P(fname, PSTR("/test/%s.txt"), filename);
  else
    sprintf_P(fname, PSTR("/test/%s.%s"), filename, ext);
  if (file.open(fname, O_READ))
  {
    int n = file.read(buffer, maxLen - 1);
    file.close();
    if (n == 0)
    {
      __debugS(PSTR("Failed to load report '%s'"), filename);
    }
    return true;
  }
  else
  {
    __debugS(P_FileNotFound, filename);
  }
  return false;
}

bool maintainingMode = false;
int8_t maintainingTool = -1;

void maintainTool()
{
  int8_t newTool;

  while (feederEndstop())
  {
    if (!showFeederBlockedMessage())
      return;
  }

  if (maintainingMode)
  {
    maintainingMode = false;
    if (maintainingTool != -1)
    {
      selectTool(maintainingTool, false);
    }
    maintainingTool = -1;
  }
  else
  {
    maintainingMode = true;
    maintainingTool = getToolSelected();

    if (toolSelected <= (smuffConfig.toolCount / 2))
    {
      newTool = toolSelected + 2;
    }
    else
    {
      newTool = toolSelected - 2;
    }
    if (newTool >= 0 && newTool < smuffConfig.toolCount)
    {
      selectTool(newTool, false);
    }
  }
}

void blinkLED()
{
#if defined(LED_PIN)
  if (LED_PIN != -1)
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
#endif
}

/*
  Translates speeds from mm/s into MCU timer ticks.
  Please notice: Using speeds in mm/s will run the steppers far slower since it has to ensure that the
  distance per second will match.
*/
unsigned long translateSpeed(uint16_t speed, uint8_t axis, bool forceTranslation)
{
  if(!forceTranslation)
    if (!smuffConfig.speedsInMMS)
      return speed;
  uint16_t stepsPerMM = (axis == REVOLVER) ? smuffConfig.stepsPerRevolution / 360 : smuffConfig.stepsPerMM[axis];
  uint8_t delay = smuffConfig.stepDelay[axis];
  // correction can be set via M202 GCode command if needed; Default is 1.0 (i.e. no correction)
  float correction = smuffConfig.speedAdjust[axis];
  unsigned long freq = F_CPU / STEPPER_PSC;
  double oneTick = (double)1 / freq;
  // check for lowest speed possible and correct it if it's below
  uint16_t minSpeed = (uint16_t)ceil(((double)((unsigned long)freq/stepsPerMM)/0xFFFF));
  if(minSpeed > speed) {
    __debugS(PSTR("Speed requested: %3d mm/s  - Min. Speed: %3d mm/s"), speed, minSpeed);
    speed = minSpeed;
  }
  unsigned long pulses = speed * stepsPerMM;
  double delayTot = pulses * (delay * 0.000001);
  double timeTot = ((double) oneTick * pulses) + delayTot;
  // if time total is more than 1 sec. its impossible to meet speed/s, so set a save value of 40mm/s.
  unsigned long ticks = (unsigned long)((timeTot <= 1.0f) ? (1 / (timeTot * correction)) : 500); // 500 equals round about 40 mm/s
  /*
  char ttl[30], dtl[30], ot[30];
  dtostrf(timeTot, 12, 6, ttl);
  dtostrf(delayTot, 12, 6, dtl);
  dtostrf(oneTick, 12, 10, ot);
  __debugS(PSTR("\nONE T:\t%s\nSPEED:\t\t%4d\nStepsMM:\t%4d\nPULSES:\t%12lu\nTICKS:\t%12lu\nTIME:\t%s\nDELAY:\t%s"), ot, speed, stepsPerMM, pulses, ticks, ttl, dtl);
  */
  if (timeTot > 1)
    __debugS(PSTR("Speed too fast, has been reset to 40mm/s! Slow down speed in mm/s."));
  return ticks;
}


void setServoLid(uint8_t pos)
{
#if !defined(SMUFF_V6S)
  #if !defined(MULTISERVO)
  uint8_t posForTool = (toolSelected < 0 || toolSelected > smuffConfig.toolCount-1) ? 0 : servoPosClosed[toolSelected];
  uint8_t p = (pos == SERVO_OPEN) ? smuffConfig.revolverOffPos : (posForTool == 0) ? smuffConfig.revolverOnPos : posForTool;
  if(servoLid.getDegree() != p) {
    //__debugS(PSTR("setServoLid called with %d (posForTool: %d)"), pos, posForTool);
    setServoPos(SERVO_LID, p);
  }
  #else
  uint8_t p = (pos == SERVO_OPEN) ? servoPosClosed[toolSelected] - SERVO_CLOSED_OFS : servoPosClosed[toolSelected];
  //__debugS(PSTR("Tool%d = %d"), toolSelected, p);
  setServoPos(toolSelected + 10, p);
  #endif
  lidOpen = pos == SERVO_OPEN;
#else
  if(pos == SERVO_OPEN)
    moveHome(REVOLVER, false, false);
  else
    positionRevolver();
#endif
}

uint8_t scanI2CDevices(uint8_t *devices, uint8_t maxDevices)
{
  uint8_t cnt = 0;
  I2CBus.begin();
  memset(devices, 0, maxDevices);
  for (uint8 address = 1; address < 127 && cnt <= maxDevices; address++)
  {
    I2CBus.beginTransmission(address);
    uint8_t stat = I2CBus.endTransmission();
    //__debugS(PSTR("Scanning at address 0x%02x returned 0x%x"), address, stat);
    if (stat == I2C_SUCCESS)
    {
      *(devices + cnt) = address;
      cnt++;
      //__debugS(PSTR("I2C device found at address 0x%02x"), address);
    }
    delay(3);
  }
  return cnt;
}

void __debugS(const char *fmt, ...)
{
  if (debugSerial == nullptr)
    return;
#ifdef DEBUG
  if (testMode)
  {
    char _dbg[512];
    va_list arguments;
    va_start(arguments, fmt);
    vsnprintf_P(_dbg, ArraySize(_dbg) - 1, fmt, arguments);
    va_end(arguments);
    debugSerial->print(F("echo: dbg: "));
    debugSerial->println(_dbg);
  }
#endif
}

void __terminal(const char *fmt, ...)
{
  if (terminalSerial == nullptr)
    return;
  char _term[256];
  va_list arguments;
  va_start(arguments, fmt);
  vsnprintf_P(_term, ArraySize(_term) - 1, fmt, arguments);
  va_end(arguments);
  terminalSerial->print(_term);
}

void __log(const char *fmt, ...)
{
  if (logSerial == nullptr)
    return;
  char _log[256];
  va_list arguments;
  va_start(arguments, fmt);
  vsnprintf_P(_log, ArraySize(_log) - 1, fmt, arguments);
  va_end(arguments);
  logSerial->print(_log);
}
