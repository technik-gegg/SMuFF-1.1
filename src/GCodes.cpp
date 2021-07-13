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
 * Module for handling all the G-Codes supported
 */

#include <Arduino.h>
#include "SMuFF.h"
#include "GCodes.h"
#include "ConfigNamesExt.h"
#include "StrPrint.h"
#ifdef __ESP32__
#include "ZPortExpander.h"

extern ZPortExpander portEx;
#endif

void setTMCStall(int8_t axis, int8_t stallPin, int param, int cool, int trigg);
void setTMCMaxStall(int8_t axis, int param);
void setTMCTmode(int8_t axis, int8_t stallPin, int param);
void setTMCMicrosteps(int8_t axis, int param);
void setTMCPower(int8_t axis, int param, int8_t irun, int8_t ihold);
void setTMCCSMin(int8_t axis, int param);
void setTMCCSMax(int8_t axis, int param);
void setTMCCSDown(int8_t axis, int param);
void setTMCTOff(int8_t axis, int param);
bool isMStepValid(int param);
void xlateSpeed205(int8_t axis, uint16_t speed, int8_t serial);
void rangeError(int8_t serial, int min, int max);
void rangeError(int8_t serial, long min, long max);
void rangeError(int8_t serial, float min, float max);

extern SdFat SD;
extern uint32_t lastEvent;
extern int8_t currentSerial;

char firmware[80];
bool gotFirmware = false;
int32_t uploadLen = 0;
SdFile upload;


const char *S_Param = (char *)"S";
const char *P_Param = (char *)"P";
const char *X_Param = (char *)"X";
const char *Y_Param = (char *)"Y";
const char *Z_Param = (char *)"Z";
const char *E_Param = (char *)"E";
const char *F_Param = (char *)"F";
const char *C_Param = (char *)"C";
const char *T_Param = (char *)"T";
const char *M_Param = (char *)"M";
const char *N_Param = (char *)"N";
const char *I_Param = (char *)"I";
const char *J_Param = (char *)"J";
const char *K_Param = (char *)"K";
const char *L_Param = (char *)"L";
const char *R_Param = (char *)"R";
const char *U_Param = (char *)"U";
const char *B_Param = (char *)"B";
const char *D_Param = (char *)"D";
const char *H_Param = (char *)"H";
const char *W_Param = (char *)"W";

GCodeFunctions gCodeFuncsM[] PROGMEM = {
    {0, dummy}, // used in Prusa Emulation mode to switch to normal mode
    {1, dummy}, // used in Prusa Emulation mode to switch to stealth mode
    {80, dummy},
    {81, dummy},
    {104, dummy},
    {105, dummy},
    {108, dummy},
    {109, dummy},
    {220, dummy},
    {221, dummy},

    {17, M17},
    {18, M18},
    {20, M20},
    {42, M42},
    {84, M18},
    {98, M98},
    {100, M100},
    {106, M106},
    {107, M107},
    {110, M110},
    {111, M111},
    {114, M114},
    {115, M115},
    {117, M117},
    {119, M119},
    {122, M122},
    {145, M145},
    {150, M150},
    {201, M201},
    {202, M202},
    {203, M203},
    {205, M205},
    {206, M206},
    {250, M250},
    {280, M280},
    {300, M300},
    {350, M350},
    {412, M412},
    {500, M500},
    {502, M502},
    {503, M503},
    {504, M504},
    {569, M569},
    {575, M575},
    {577, M577},
    {700, M700},
    {701, M701},
    {906, M906},
    {914, M914},
    {997, M997},
    {999, M999},
    {2000, M2000},
    {2001, M2001},
    {-1, nullptr}};

GCodeFunctions gCodeFuncsG[] PROGMEM = {
    {0, G0},
    {1, G1},
    {4, G4},
    {12, G12},
    {28, G28},
    {90, G90},
    {91, G91},
    {-1, nullptr},
};

int param;

/*========================================================
 * Class M
 ========================================================*/
bool dummy(const char *msg, String buf, int8_t serial)
{
  if (!smuffConfig.prusaMMU2)
  {
    uint16_t code = buf.toInt();
    __debugS(PSTR("Ignored M-Code: M%d"), code);
  }
  return true;
}

/*
  Deviating from the GCode standard, this method is used for switching the Feeder stepper motor
  from internal (SMuFF) to external (3D-Printer).
*/
bool M17(const char *msg, String buf, int8_t serial)
{
  bool stat = true;
  printResponse(msg, serial);
  char tmp[20];
  if (buf.indexOf(E_Param) != -1)
    switchFeederStepper(EXTERNAL);
  else if (buf.indexOf(I_Param) != -1)
    switchFeederStepper(INTERNAL);
  else
  {
    sprintf_P(tmp, PSTR("echo: %s\n"), smuffConfig.externalStepper ? P_External : P_Internal);
    printResponse(tmp, serial);
  }
  return stat;
}

bool M18(const char *msg, String buf, int8_t serial)
{
  bool stat = true;
  printResponse(msg, serial);
  if (buf.length() == 0)
  {
    steppers[SELECTOR].setEnabled(false);
    steppers[REVOLVER].setEnabled(false);
    steppers[FEEDER].setEnabled(false);
  }
  else
  {
    if (buf.indexOf(X_Param) != -1)
    {
      steppers[SELECTOR].setEnabled(false);
    }
    else if (buf.indexOf(Y_Param) != -1)
    {
      steppers[REVOLVER].setEnabled(false);
    }
    else if (buf.indexOf(Z_Param) != -1)
    {
      steppers[FEEDER].setEnabled(false);
    }
    else
    {
      stat = false;
    }
  }
  return stat;
}

bool M20(const char *msg, String buf, int8_t serial)
{
  char tmp[80];
  char fmt[20];
  bool lsInString = false;

  if(!getParamString(buf, L_Param, tmp, ArraySize(tmp))) {
    if(!getParamString(buf, S_Param, tmp, ArraySize(tmp))) {
      sprintf(tmp, "/");
    }
    if(!getParamString(buf, F_Param, fmt, ArraySize(fmt))) {
      sprintf(fmt, "DSR");
    }
  }
  else {
    listTextFile(tmp, serial);
    return true;
  }
  Print *out;
  StrPrint *strOut;
  if (initSD(false))
  {
    switch (serial)
    {
      case 0:
        out = &Serial;
        break;
      case 1:
        out = &Serial1;
        break;
      case 2:
        out = &Serial2;
        break;
      default:
        out = &Serial;
        break;
    }
    if(hasParam(buf, W_Param)) {
      strOut = new StrPrint();
      lsInString = true;
    }
    String f = String(fmt);
    uint8_t flags = 0;
    if(f.indexOf("D") > -1)  flags |= LS_DATE;
    if(f.indexOf("S") > -1)  flags |= LS_SIZE;
    if(f.indexOf("R") > -1)  flags |= LS_R;
    if(lsInString) {
      SD.ls((Print*)strOut, tmp, flags);
      String json = "/* Testscripts */[\"" + ((StrPrint*)strOut)->get();
      json.replace("\r\n", "\",\"");
      json += "\"]";
      //__debugS(PSTR("LS in String: %s"), json.c_str());
      out->println(json.c_str());
    }
    else {
      SD.ls(out, tmp, flags);
    }
  }
  else
  {
    sprintf_P(tmp, P_SD_InitError);
    printResponse(tmp, serial);
    return false;
  }
  return true;
}

bool M42(const char *msg, String buf, int8_t serial)
{
  bool stat = true;
  int8_t pin;
  int8_t mode;
  char tmp[128];
  printResponse(msg, serial);
  //__debugS(PSTR("M42->%s"), buf);

  if ((pin = getParam(buf, P_Param)) != -1)
  {
    // pins over 1000 go to the port expander
    if (pin >= 1000)
    {
#if defined(__ESP32__)
      pin -= 1000;
      if ((mode = getParam(buf, M_Param)) != -1)
      {
        // 0=INPUT, 1=OUTPUT, 2=INPUT_PULLUP, 3=INPUT_PULLDOWN
        switch (mode)
        {
        case 0:
          portEx.pinMode(pin, INPUT);
          break;
        case 1:
          portEx.pinMode(pin, OUTPUT);
          break;
        case 2:
          portEx.pinMode(pin, INPUT_PULLUP);
          break;
        case 3:
          portEx.pinMode(pin, INPUT_PULLDOWN);
          break;
        }
      }
      else
      {
        mode = 1;
        portEx.pinMode(pin, OUTPUT);
      }
      if ((param = getParam(buf, S_Param)) != -1 && mode == 1)
      {
        //__debugS(PSTR("Pin%d set to %s"), pin, param==0 ? "LOW" : "HIGH");
        portEx.writePin(pin, param);
      }
      if (mode != 1)
      {
        uint8_t state = portEx.readPin(pin);
        sprintf_P(tmp, PSTR("echo: P%d: %s\n"), pin + 1000, state == 0 ? "LOW" : "HIGH");
        printResponse(tmp, serial);
      }
#endif
    }
    else
    {
      if ((mode = getParam(buf, M_Param)) != -1)
      {
        // 0=INPUT, 1=OUTPUT, 2=INPUT_PULLUP, 3=INPUT_PULLDOWN
        switch (mode)
        {
        case 0:
          pinMode(pin, INPUT);
          break;
        case 1:
          pinMode(pin, OUTPUT);
          break;
        case 2:
          pinMode(pin, INPUT_PULLUP);
          break;
        case 3:
          pinMode(pin, INPUT_PULLDOWN);
          break;
        }
      }
      else
      {
        mode = 1;
        pinMode(pin, OUTPUT);
      }
      if ((param = getParam(buf, S_Param)) != -1 && mode == 1)
      {
        if (param >= 0 && param <= 255)
        {
#ifdef __STM32F1__
          pwmWrite(pin, param);
#elif __ESP32__
          ledcWrite(pin, param);
#else
          analogWrite(pin, param);
#endif
        }
      }
      if (mode != 1)
      {
        uint8_t state = digitalRead(pin);
        sprintf(tmp, "echo: P%d: %d\n", pin, state);
        printResponse(tmp, serial);
      }
    }
  }
  return stat;
}

bool M98(const char *msg, String buf, int8_t serial)
{
  printResponse(msg, serial);
  char cmd[80];
  if ((getParamString(buf, P_Param, cmd, ArraySize(cmd)))) {
    setTestRunPending(cmd);
    return true;
  }
  return false;
}

bool M100(const char *msg, String buf, int8_t serial)
{
  printResponse(msg, serial);
  return true;
}

bool M106(const char *msg, String buf, int8_t serial)
{
  int freq = 0;
  printResponse(msg, serial);
  if ((param = getParam(buf, S_Param)) == -1)
  {
    param = 100;
  }
  if ((freq = getParam(buf, F_Param)) == -1)
  {
    freq = 0;
  }
  //__debugS(PSTR("Fan speed: %d%%"), param);
#ifdef __STM32F1__
  // set the frequency if applied
  if(freq > 0 && freq <= 500) {
    uint16_t max = (uint16_t)(uint32_t(((float)1/freq)*1000000L));
    fan.setPulseWidthMax(max);
  }
  fan.setFanSpeed(param);
#elif __ESP32__
  ledcWrite(FAN_PIN, map(param, 0, 100, 0, 255));
#else
  analogWrite(FAN_PIN, map(param, 0, 100, 0, 255));
#endif
  return true;
}

bool M107(const char *msg, String buf, int8_t serial)
{
  printResponse(msg, serial);
#ifdef __STM32F1__
  fan.setFanSpeed(0);
#elif __ESP32__
  ledcWrite(FAN_PIN, 0);
#else
  analogWrite(FAN_PIN, 0);
#endif
  return true;
}

bool M110(const char *msg, String buf, int8_t serial)
{
  printResponse(msg, serial);
  if ((param = getParam(buf, N_Param)) != -1)
  {
    currentLine = (uint16_t)param;
  }
  return true;
}

bool M111(const char *msg, String buf, int8_t serial)
{
  if ((param = getParam(buf, S_Param)) != -1)
  {
    testMode = param == 1;
  }
  return true;
}

bool M114(const char *msg, String buf, int8_t serial)
{
  char tmp[128];
  printResponse(msg, serial);
  sprintf_P(tmp, P_TMC_StatusAll,
            String(steppers[SELECTOR].getStepPositionMM()).c_str(),
            String(steppers[REVOLVER].getStepPosition()).c_str(),
            String(steppers[FEEDER].getStepPositionMM()).c_str());
  printResponse(tmp, serial);
  return true;
}

bool M115(const char *msg, String buf, int8_t serial)
{
  char tmp[200];
  char ver[8];
  char options[80] = {0};
  sprintf(ver, VERSION_STRING);
#if defined(DEBUG)
  strcat(ver, "D");
#endif
#if defined(MULTISERVO)
  strcat(options, "MULTISERVO|");
#endif
#if defined(HAS_TMC_SUPPORT)
  strcat(options, "TMC|");
#endif
#if defined(USE_FASTLED_TOOLS) || defined(USE_FASTLED_BACKLIGHT)
  strcat(options, "NEOPIXELS|");
#endif
#if defined(USE_SPLITTER_ENDSTOPS)
  strcat(options, "ESTOP-MUX|");
#endif
#if defined(USE_DDE)
  strcat(options, "DDE|");
#endif
#if defined(USE_TWI_DISPLAY)
  strcat(options, "TWI");
#elif defined(USE_LEONERD_DISPLAY)
  strcat(options, "LEONERD");
#elif defined(USE_ANET_DISPLAY)
  strcat(options, "ANET");
#elif defined(USE_MINI12864_PANEL_V21)
  strcat(options, "MINIPANEL V2.1");
#elif defined(USE_MINI12864_PANEL_V20)
  strcat(options, "MINIPANEL V2.0");
#elif defined(USE_CREALITY_DISPLAY)
  strcat(options, "CREALITY");
  #if defined(CREALITY_HW_SPI)
    strcat(options, " HW-SPI");
  #endif
#else
  strcat(options, "REPRAP");
#endif
#if defined(SMUFF_V6S)
#define SMUFF_MODE  "SMuFF V6S"
  strcat(options, "|V6S");
#else
#define SMUFF_MODE  "SMuFF"
#endif
  sprintf_P(tmp, P_GVersion, ver, BOARD_INFO, VERSION_DATE, smuffConfig.prusaMMU2 ? "PMMU" : SMUFF_MODE, options);
  printResponse(tmp, serial);
  return true;
}

bool M117(const char *msg, String buf, int8_t serial)
{
  String umsg = buf;
  if (umsg.length() > 0)
  {
    umsg.replace("_", " ");
    beep(1);
    drawUserMessage(umsg);
    return true;
  }
  return false;
}

bool M119(const char *msg, String buf, int8_t serial)
{
  printResponse(msg, serial);
  if ((param = getParam(buf, Z_Param)) != -1)
  {
    steppers[FEEDER].setEndstopHit(param);
  }
  printEndstopState(serial);
  return true;
}

/*
  Print out all information available on the TMC steppers
  if there are any configured.

  Looks a bit messy and could have been done quite a bit more
  modular, although, as you might need it only once or twice
  it's ok for now.
*/
bool M122(const char *msg, String buf, int8_t serial)
{
#ifndef HAS_TMC_SUPPORT
  return false;
#else
  char dbg[20];
  uint8_t numSteppers = NUM_STEPPERS;
  #if defined(TMC_HW_SERIAL)          // only E3 2.0
  numSteppers++;
  digitalWrite(E_ENABLE_PIN, LOW);
  #endif
  printResponse(msg, serial);

  if (drivers[SELECTOR] != nullptr)
  {
    steppers[SELECTOR].setEnabled(true);
  }
  if (drivers[REVOLVER] != nullptr)
  {
    steppers[REVOLVER].setEnabled(true);
  }
  if (drivers[FEEDER] != nullptr)
  {
    steppers[FEEDER].setEnabled(true);
  }
  char spacer[] = {"          "};
  char eol[] = {"\n"};

  // special format for WebInterface
  if ((param = getParam(buf, W_Param)) != -1) {
    if(param >= 0 && param <= 3) {
      sendTMCStatus(param, serial);
      return true;
    }
    else
      return false;
  }

  printResponseP(P_TMC_Setup00, serial);
  // IC-Version
  printResponseP(P_TMC_Setup01, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4s, String(drivers[i]->version() - 0x20).c_str());
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Enabled
  printResponseP(P_TMC_Setup02, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4s, drivers[i]->isEnabled() ? P_Yes : P_No);
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Power set
  printResponseP(P_TMC_Setup03, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4d, smuffConfig.stepperPower[i]);
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Power RMS
  printResponseP(P_TMC_Setup03a, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4d, drivers[i]->rms_current());
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Microsteps
  printResponseP(P_TMC_Setup04, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4d, drivers[i]->microsteps());
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // TOff
  printResponseP(P_TMC_Setup05, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4d, drivers[i]->toff());
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Blank Time
  printResponseP(P_TMC_Setup06, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4d, drivers[i]->blank_time());
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // PDN/UART
  printResponseP(P_TMC_Setup07, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4s, drivers[i]->pdn_uart() ? P_Yes : P_No);
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // MS1 / MS2
  printResponseP(P_TMC_Setup08, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, PSTR("  %1d%1d      "), drivers[i]->ms2(), drivers[i]->ms1());
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Diag
  printResponseP(P_TMC_Setup09, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4s, drivers[i]->diag() ? P_High : P_Low);
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // StallGuard THRS
  printResponseP(P_TMC_Setup10, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4d, drivers[i]->SGTHRS());
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // StallGuard Result
  printResponseP(P_TMC_Setup11, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4d, drivers[i]->SG_RESULT());
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Max Stall Count
  printResponseP(P_TMC_Setup12, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4d, smuffConfig.stepperMaxStallCnt[i]);
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // semin
  printResponseP(P_TMC_Setup13, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4d, drivers[i]->semin());
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // semax
  printResponseP(P_TMC_Setup14, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4d, drivers[i]->semax());
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // sedn
  printResponseP(P_TMC_Setup15, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4d, drivers[i]->sedn());
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // CoolStep THRS
  printResponseP(P_TMC_Setup16, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F8x, drivers[i]->TCOOLTHRS());
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // PWM THRS
  printResponseP(P_TMC_Setup17, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F8x, drivers[i]->TPWMTHRS());
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // TSTEP
  printResponseP(P_TMC_Setup18, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F8x, drivers[i]->TSTEP());
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // IRUN
  printResponseP(P_TMC_Setup19, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4d, drivers[i]->cs2rms(drivers[i]->irun()));
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // IHOLD
  printResponseP(P_TMC_Setup20, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4d, drivers[i]->cs2rms(drivers[i]->ihold()));
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // IHOLD delay
  printResponseP(P_TMC_Setup21, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4d, drivers[i]->iholddelay());
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // TPOWERDOWN
  printResponseP(P_TMC_Setup22, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4s, String((float)(drivers[i]->TPOWERDOWN() * 0.021875)).c_str());
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // PWM Gradient
  printResponseP(P_TMC_Setup23, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F33d, drivers[i]->pwm_grad(), drivers[i]->pwm_autograd());
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // PWM Scale
  printResponseP(P_TMC_Setup24, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F33d, drivers[i]->pwm_ofs(), drivers[i]->pwm_scale_sum());
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  printResponse(eol, serial);
  // ---- STATUS ----
  printResponseP(P_TMC_Status00, serial);
  // Mode
  printResponseP(P_TMC_Status01, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_FL7s, drivers[i]->stealth() ? P_Stealth : P_Spread);
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Standstill
  printResponseP(P_TMC_Status02, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4s, drivers[i]->stst() ? P_Yes : P_No);
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Phase A Open
  printResponseP(P_TMC_Status03, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4s, drivers[i]->ola() ? P_Yes : P_No);
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Phase B Open
  printResponseP(P_TMC_Status04, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4s, drivers[i]->olb() ? P_Yes : P_No);
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Phase A Short to GND
  printResponseP(P_TMC_Status05, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4s, drivers[i]->s2ga() ? P_Yes : P_No);
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Phase B Short to GND
  printResponseP(P_TMC_Status06, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4s, drivers[i]->s2gb() ? P_Yes : P_No);
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Phase A Short MOSFET
  printResponseP(P_TMC_Status07, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4s, drivers[i]->s2vsa() ? P_Yes : P_No);
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Phase B Short MOSFET
  printResponseP(P_TMC_Status08, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4s, drivers[i]->s2vsb() ? P_Yes : P_No);
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Overtemp. Warning
  printResponseP(P_TMC_Status09, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4s, drivers[i]->otpw() ? P_Yes : P_No);
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Overtemp.
  printResponseP(P_TMC_Status10, serial);
  for (uint8_t i = 0; i < numSteppers; i++)
  {
    if (drivers[i] != nullptr)
    {
      sprintf_P(dbg, P_F4s, drivers[i]->ot() ? P_Yes : P_No);
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  return true;
#endif
}

bool M145(const char *msg, String buf, int8_t serial)
{
  char color[50];
  char tmp[40];
  uint8_t tool;

  printResponse(msg, serial);
  if ((param = getParam(buf, S_Param)) != -1)
  {
    tool = (uint8_t)param;
    if (tool >= 0 && tool <= smuffConfig.toolCount)
    {
      if (getParamString(buf, P_Param, color, sizeof(color)))
      {
        strncpy(smuffConfig.materialNames[tool], color, ArraySize(smuffConfig.materialNames[tool]));
        return true;
      }
      else if ((param = getParam(buf, F_Param)) != -1)
      {
        if (param > 50 && param < 500)
          smuffConfig.purges[tool] = param;
        return true;
      }
      else if (getParamString(buf, C_Param, color, ArraySize(color)))
      {
        long colorVal;
        if(sscanf(color, "%lx", &colorVal)) {
          smuffConfig.materialColors[tool] = (uint32_t)colorVal;
        }
        return true;
      }
      else
      {
        sprintf_P(tmp, P_ToolMaterial, tool, smuffConfig.materialNames[tool], smuffConfig.purges[tool]);
        printResponse(tmp, serial);
        return true;
      }
    }
    else
      rangeError(serial, 0, smuffConfig.toolCount);
  }
  return false;
}

bool M150(const char *msg, String buf, int8_t serial)
{
  printResponse(msg, serial);

#if defined(USE_FASTLED_BACKLIGHT) || defined(USE_FASTLED_TOOLS)
  uint8_t red = 0, green = 0, blue = 0;
  int8_t index = -1, colorNdx = -1;
  uint8_t intensity = 255;

  if (NEOPIXEL_PIN == -1)
    return false;

  if ((param = getParam(buf, R_Param)) != -1)
  {
    red = (uint8_t)param;
  }
  if ((param = getParam(buf, U_Param)) != -1)
  {
    green = (uint8_t)param;
  }
  if ((param = getParam(buf, B_Param)) != -1)
  {
    blue = (uint8_t)param;
  }
  if ((param = getParam(buf, S_Param)) != -1)
  {
    index = (int8_t)param;
  }
  if ((param = getParam(buf, P_Param)) != -1)
  {
    intensity = (uint8_t)param;
  }
  if ((param = getParam(buf, C_Param)) != -1)
  {
    colorNdx = (int8_t)param;
  }
  if ((param = getParam(buf, T_Param)) != -1)
  {
    index = (int8_t)param;
    setFastLEDIntensity(intensity);
    setFastLEDToolIndex(index, colorNdx, true);
    return true;
  }
  setFastLEDIntensity(intensity);
  if (colorNdx != -1)
  {
    if (index == -1)
      setBacklightIndex(colorNdx);
    else if (index >= 0 && index < NUM_LEDS)
      setFastLEDIndex(index, colorNdx);
  }
  else
  {
    if (index == -1 || index > NUM_LEDS)
      return false;
    CRGB color = CRGB(red, green, blue);
    setFastLED(index, color);
  }
  return true;
#else
  return false;
#endif
}

bool M201(const char *msg, String buf, int8_t serial)
{
  bool stat = true;
  printResponse(msg, serial);
  if (buf.length() == 0)
  {
    printAcceleration(serial);
    return stat;
  }
  if ((param = getParam(buf, X_Param)) != -1)
  {
    if ((unsigned int)param >= mmsMin && (unsigned int)param <= mmsMax)
    {
      smuffConfig.accelSpeed[SELECTOR] = (uint16_t)param;
      steppers[SELECTOR].setAcceleration(translateSpeed(smuffConfig.accelSpeed[SELECTOR], SELECTOR));
    }
    else
      stat = false;
  }
  if ((param = getParam(buf, Y_Param)) != -1)
  {
    if ((unsigned int)param >= mmsMin && (unsigned int)param <= mmsMax)
    {
      smuffConfig.accelSpeed[REVOLVER] = (uint16_t)param;
      steppers[REVOLVER].setAcceleration(translateSpeed(smuffConfig.accelSpeed[REVOLVER], REVOLVER));
    }
    else
      stat = false;
  }
  if ((param = getParam(buf, Z_Param)) != -1)
  {
    if ((unsigned int)param >= mmsMin && (unsigned int)param <= mmsMax)
    {
      smuffConfig.accelSpeed[FEEDER] = (uint16_t)param;
      steppers[FEEDER].setAcceleration(translateSpeed(smuffConfig.accelSpeed[FEEDER], FEEDER));
    }
    else
      stat = false;
  }
  return stat;
}

bool M202(const char *msg, String buf, int8_t serial)
{
  bool stat = true;
  printResponse(msg, serial);
  if (buf.length() == 0)
  {
    printSpeedAdjust(serial);
    return stat;
  }
  float paramF;
  if ((paramF = getParamF(buf, X_Param)) != -1)
  {
    if(paramF > 0.0 && paramF <= 3.0) {
      smuffConfig.speedAdjust[SELECTOR] = paramF;
      steppers[SELECTOR].setMaxSpeed(translateSpeed(smuffConfig.maxSpeed[SELECTOR], SELECTOR));
      steppers[SELECTOR].setAcceleration(translateSpeed(smuffConfig.accelSpeed[SELECTOR], SELECTOR));
    }
  }
  if ((paramF = getParamF(buf, Y_Param)) != -1)
  {
    if(paramF > 0.0 && paramF <= 3.0) {
      smuffConfig.speedAdjust[REVOLVER] = paramF;
      steppers[REVOLVER].setMaxSpeed(translateSpeed(smuffConfig.maxSpeed[REVOLVER], REVOLVER));
      steppers[REVOLVER].setAcceleration(translateSpeed(smuffConfig.accelSpeed[REVOLVER], REVOLVER));
    }
  }
  if ((paramF = getParamF(buf, Z_Param)) != -1)
  {
    if(paramF > 0.0 && paramF <= 3.0) {
      smuffConfig.speedAdjust[FEEDER] = paramF;
      steppers[FEEDER].setMaxSpeed(translateSpeed(smuffConfig.maxSpeed[FEEDER], FEEDER));
      steppers[FEEDER].setAcceleration(translateSpeed(smuffConfig.accelSpeed[FEEDER], FEEDER));
    }
  }
  return stat;
}

bool M203(const char *msg, String buf, int8_t serial)
{
  bool stat = true;
  printResponse(msg, serial);
  if (buf.length() == 0)
  {
    printSpeeds(serial);
    return stat;
  }
  if ((param = getParam(buf, X_Param)) != -1)
  {
    if ((unsigned int)param > mmsMin && (unsigned int)param <= mmsMax)
    {
      smuffConfig.maxSpeed[SELECTOR] = (uint16_t)param;
      steppers[SELECTOR].setMaxSpeed(translateSpeed(smuffConfig.maxSpeed[SELECTOR], SELECTOR));
    }
    else
      stat = false;
    if ((param = getParam(buf, D_Param)) != -1)
    {
      smuffConfig.stepDelay[SELECTOR] = (uint8_t)param;
    }
  }
  if ((param = getParam(buf, Y_Param)) != -1)
  {
    if ((unsigned int)param > mmsMin && (unsigned int)param <= mmsMax)
    {
      smuffConfig.maxSpeed[REVOLVER] = (uint16_t)param;
      steppers[REVOLVER].setMaxSpeed(translateSpeed(smuffConfig.maxSpeed[REVOLVER], REVOLVER));
    }
    else
      stat = false;
    if ((param = getParam(buf, D_Param)) != -1)
    {
      smuffConfig.stepDelay[REVOLVER] = (uint8_t)param;
    }
  }
  if ((param = getParam(buf, Z_Param)) != -1)
  {
    if ((unsigned int)param > mmsMin && (unsigned int)param <= mmsMax)
    {
      smuffConfig.maxSpeed[FEEDER] = (uint16_t)param;
      steppers[FEEDER].setMaxSpeed(translateSpeed(smuffConfig.maxSpeed[FEEDER], FEEDER));
    }
    else
      stat = false;
    if ((param = getParam(buf, D_Param)) != -1)
    {
      smuffConfig.stepDelay[FEEDER] = (uint8_t)param;
    }
    if ((param = getParam(buf, F_Param)) != -1)
    {
      smuffConfig.insertSpeed = (uint16_t)param;
    }
  }
  return stat;
}

void rangeError(int8_t serial, int min, int max)
{
  char tmp[80];
  sprintf_P(tmp, P_UseRangeI, min, max);
  printResponseP(P_RangeError, serial);
  printResponse(tmp, serial);
}

void rangeError(int8_t serial, long min, long max)
{
  char tmp[80];
  sprintf_P(tmp, P_UseRangeL, min, max);
  printResponseP(P_RangeError, serial);
  printResponse(tmp, serial);
}

void rangeError(int8_t serial, float min, float max)
{
  char tmp[80];
  sprintf_P(tmp, P_UseRangeF, String(min).c_str(), String(max).c_str());
  printResponseP(P_RangeError, serial);
  printResponse(tmp, serial);
}

bool M205(const char *msg, String buf, int8_t serial)
{
  bool stat = true;
  printResponse(msg, serial);

  if (buf.length() == 0)
  {
    sendM205List(serial);
    return stat;
  }

  char cmd[80];
  char tmp[50];
  char strpar[40] = {0};
  //__debugS(PSTR("Got paramsS: %s [%s]"), paramS ? "Yes" : "No", strpar);
  if (getParamString(buf, P_Param, cmd, sizeof(cmd)))
  {
    int8_t axis = -1;
    int8_t stallPin = -1;
    if (hasParam(buf, X_Param)) { axis = SELECTOR; stallPin = STALL_X_PIN; }
    if (hasParam(buf, Y_Param)) { axis = REVOLVER; stallPin = STALL_Y_PIN; }
    if (hasParam(buf, Z_Param)) { axis = FEEDER;   stallPin = STALL_Z_PIN; }
    int   index = getParam(buf, I_Param);
    bool  paramS = getParamString(buf, S_Param, strpar, sizeof(strpar));
    float fParam = getParamF(buf, S_Param);
    param = getParam(buf, S_Param);

    //__debugS("%d | %f | %s", param, fParam, paramS ? strpar : "");
    if(paramS) {
      if (strcmp(cmd, wipeSequence) == 0)           { strncpy(smuffConfig.wipeSequence, strpar, sizeof(smuffConfig.wipeSequence)); }
      else if (strcmp(cmd, material) == 0)          { if (index != -1) strncpy(smuffConfig.materials[index], strpar, MAX_MATERIAL_LEN); else stat = false; }
      else if (strcmp(cmd, color) == 0)             { if (index != -1) strncpy(smuffConfig.materialNames[index], strpar, MAX_MATERIAL_NAME_LEN); else stat = false; }
      else if (strcmp(cmd, lBtnDown) == 0)          { strncpy(smuffConfig.lButtonDown, strpar, MAX_BUTTON_LEN); }
      else if (strcmp(cmd, lBtnHold) == 0)          { strncpy(smuffConfig.lButtonHold, strpar, MAX_BUTTON_LEN); }
      else if (strcmp(cmd, rBtnDown) == 0)          { strncpy(smuffConfig.rButtonDown, strpar, MAX_BUTTON_LEN); }
      else if (strcmp(cmd, rBtnHold) == 0)          { strncpy(smuffConfig.rButtonHold, strpar, MAX_BUTTON_LEN); }
      else if (strcmp(cmd, devName) == 0)           { strncpy(smuffConfig.deviceName, strpar, MAX_BUTTON_LEN); }
      else if (strcmp(cmd, serialBaudrate) == 0)    { if (index != -1) { uint32_t baud = 0; sscanf(strpar, "%lu", &baud); if(baud >= 4800 && baud <= 230400) smuffConfig.serialBaudrates[index] = baud; } else stat = false; }
      else if (strcmp(cmd, colorVal) == 0)          { if (index != -1) { uint32_t cval = 0; sscanf(strpar, "%lu", &cval); smuffConfig.materialColors[index] = cval; } else stat = false; }
      else {
        sprintf_P(tmp, P_UnknownParam, cmd);
        printResponse(tmp, serial);
        stat = false;
      }
    }
    else if (param != -1) {
      if (strcmp(cmd, toolCount) == 0)                { if (param >= 1 && param <= MAX_TOOLS) smuffConfig.toolCount = (uint8_t)param; else rangeError(serial, 1, MAX_TOOLS); }

      else if (strcmp(cmd, spacing) == 0)             { smuffConfig.toolSpacing = fParam;  }
      else if (strcmp(cmd, bowdenLength) == 0)        { smuffConfig.bowdenLength = fParam; }
      else if (strcmp(cmd, insertLength) == 0)        { smuffConfig.insertLength = fParam; }
      else if (strcmp(cmd, purgeLength) == 0)         { smuffConfig.purgeLength = fParam;  }
      else if (strcmp(cmd, reinforceLength) == 0)     { smuffConfig.reinforceLength = fParam; }
      else if (strcmp(cmd, selectorDist) == 0)        { smuffConfig.selectorDistance = fParam; }
      else if (strcmp(cmd, splitterDist) == 0)        { smuffConfig.splitterDist = fParam; }
      else if (strcmp(cmd, offset) == 0)              { smuffConfig.firstToolOffset = fParam; }
      else if (strcmp(cmd, unloadRetract) == 0)       { smuffConfig.unloadRetract = fParam;  }
      else if (strcmp(cmd, rsense) == 0)              { if (fParam >= 0 && fParam <= 1.0 && axis != -1) smuffConfig.stepperRSense[axis] = fParam;   else { rangeError(serial, 0, 1.0); stat = false; } }
      else if (strcmp(cmd, revolverClosed) == 0)      { smuffConfig.revolverClose = fParam; stepperPosClosed[toolSelected] = smuffConfig.revolverClose; }
      else if (strcmp(cmd, ddeDist) == 0)             { smuffConfig.ddeDist = fParam; }

      else if (strcmp(cmd, useDDE) == 0)              { smuffConfig.useDDE = (param > 0); }
      else if (strcmp(cmd, homeAfterFeed) == 0)       { smuffConfig.homeAfterFeed = (param > 0); }
      else if (strcmp(cmd, resetBeforeFeed) == 0)     { smuffConfig.resetBeforeFeed = (param > 0); }
      else if (strcmp(cmd, emulatePrusa) == 0)        { smuffConfig.prusaMMU2 = (param > 0); }
      else if (strcmp(cmd, useServo) == 0)            { smuffConfig.revolverIsServo = (param > 0); }
      else if (strcmp(cmd, sendAction) == 0)          { smuffConfig.sendActionCmds = (param > 0); }
      else if (strcmp(cmd, sharedStepper) == 0)       { smuffConfig.isSharedStepper = (param > 0); }
      else if (strcmp(cmd, encoderTicks) == 0)        { smuffConfig.encoderTickSound = (param > 0); }
      else if (strcmp(cmd, periodicalStats) == 0)     { smuffConfig.sendPeriodicalStats = (param > 0); }
      else if (strcmp(cmd, idleAnim) == 0)            { smuffConfig.useIdleAnimation = (param > 0); }
      else if (strcmp(cmd, menuOnTerm) == 0)          { smuffConfig.menuOnTerminal = (param > 0); }
      else if (strcmp(cmd, webInterface) == 0)        { smuffConfig.webInterface = (param > 0); if(smuffConfig.webInterface) smuffConfig.menuOnTerminal = false; }
      else if (strcmp(cmd, usePurge) == 0)            { smuffConfig.usePurge = (param > 0); }
      else if (strcmp(cmd, speedsInMMS) == 0)         { smuffConfig.speedsInMMS = (param > 0); }
      else if (strcmp(cmd, invertRelay) == 0)         { smuffConfig.invertRelay = (param > 0); switchFeederStepper(smuffConfig.externalStepper); }
      else if (strcmp(cmd, useCutter) == 0)           { smuffConfig.useCutter = (param > 0); }
      else if (strcmp(cmd, autoWipe) == 0)            { smuffConfig.wipeBeforeUnload = (param > 0); }
      else if (strcmp(cmd, sendAction) == 0)          { smuffConfig.sendActionCmds = (param > 0); }
      else if (strcmp(cmd, externalControl) == 0)     { smuffConfig.extControlFeeder = (param > 0); }
      else if (strcmp(cmd, enableChunks) == 0)        { smuffConfig.enableChunks = (param > 0); }
      else if (strcmp(cmd, endstop2) == 0)            { smuffConfig.useEndstop2 = (param > 0); }
      else if (strcmp(cmd, invertDir) == 0)           { if (axis != -1) smuffConfig.invertDir[axis] = (param > 0); else stat = false; }
      else if (strcmp(cmd, useSplitter) == 0)         { smuffConfig.useSplitter = (param > 0); }
      else if (strcmp(cmd, purgeDDE) == 0)            { smuffConfig.purgeDDE = (param > 0); }

      else if (strcmp(cmd, purgeSpeed) == 0)          { smuffConfig.purgeSpeed = (uint16_t)param; }
      else if (strcmp(cmd, servoOffPos) == 0)         { if(index == 99) setServoPos(SERVO_LID, (uint8_t) param); else smuffConfig.revolverOffPos = (uint8_t)param; }
      else if (strcmp(cmd, servoOnPos) == 0)          { if(index == 99) setServoPos(SERVO_LID, (uint8_t) param); else smuffConfig.revolverOnPos = (uint8_t)param; servoPosClosed[toolSelected] = smuffConfig.revolverOnPos; }
      else if (strcmp(cmd, autoClose) == 0)           { smuffConfig.menuAutoClose = (uint8_t)param; }
      else if (strcmp(cmd, psTimeout) == 0)           { smuffConfig.powerSaveTimeout = (uint16_t)param; }
      else if (strcmp(cmd, endstopTest) == 0)         { smuffConfig.endstopTrg[3] = (uint8_t)param; steppers[FEEDER].setEndstopState(smuffConfig.endstopTrg[FEEDER], 2); }
      else if (strcmp(cmd, stepsPerRevolution) == 0)  { smuffConfig.stepsPerRevolution = param; steppers[REVOLVER].setStepsPerDegree(param / 360); }
      else if (strcmp(cmd, hasPanelDue) == 0)         { smuffConfig.hasPanelDue = (uint8_t)param; }
      else if (strcmp(cmd, feedChunks) == 0)          { smuffConfig.feedChunks = (uint8_t)param; }
      else if (strcmp(cmd, servoMinPwm) == 0)         { smuffConfig.servoMinPwm = (uint16_t)param; for(int8_t i=0; i< 3; i++) setServoMinPwm(i, smuffConfig.servoMinPwm); }
      else if (strcmp(cmd, servoMaxPwm) == 0)         { smuffConfig.servoMaxPwm = (uint16_t)param; for(int8_t i=0; i< 3; i++) setServoMaxPwm(i, smuffConfig.servoMaxPwm); }
      else if (strcmp(cmd, servo1Cycles) == 0)        { smuffConfig.servoCycles1 = (uint8_t)param; servoWiper.setMaxCycles(smuffConfig.servoCycles1); }
      else if (strcmp(cmd, servo2Cycles) == 0)        { smuffConfig.servoCycles2 = (uint8_t)param; servoLid.setMaxCycles(smuffConfig.servoCycles2); }
      else if (strcmp(cmd, cutterOpen) == 0)          { smuffConfig.cutterOpen = (uint8_t)param; }
      else if (strcmp(cmd, cutterClose) == 0)         { smuffConfig.cutterClose = (uint8_t)param; }
      else if (strcmp(cmd, insertSpeed) == 0)         { smuffConfig.insertSpeed = (uint16_t)param; }

      else if (strcmp(cmd, backlightColor) == 0)      { if (param >= 0 && param <= 15) { smuffConfig.backlightColor = (uint8_t)param; setBacklightIndex(smuffConfig.backlightColor); } else stat = false; }
      else if (strcmp(cmd, toolColor) == 0)           { if (param >= 0 && param <= 15) { smuffConfig.toolColor = (uint8_t)param; setToolColorIndex(smuffConfig.toolColor); } else stat = false; }
      else if (strcmp(cmd, animBpm) == 0)             { if (param > 0 && param <= 255) { smuffConfig.animationBPM = (uint8_t)param; lastEvent = millis() - smuffConfig.powerSaveTimeout*1500; isIdle = true; } else stat = false; }
      else if (strcmp(cmd, statusBpm) == 0)           { if (param > 0 && param <= 255) smuffConfig.statusBPM = (uint8_t)param; else stat = false; }
      else if (strcmp(cmd, fanSpeed) == 0)            { if (param >= 0 && param <= 100) { smuffConfig.fanSpeed = (uint8_t)param; setupFan(); } else stat = false; }
      else if (strcmp(cmd, contrast) == 0)            { if (param >= 60 && param <= 255) { smuffConfig.lcdContrast = (uint8_t)param; display.setContrast(smuffConfig.lcdContrast); } else stat = false; }
      else if (strcmp(cmd, xlateSpeed) == 0)          { if (axis != -1) { xlateSpeed205(axis, (uint16_t)param, serial); } else stat = false; }
      else if (strcmp(cmd, stepsPerMillimeter) == 0)  { if (axis != -1) { smuffConfig.stepsPerMM[axis] = (uint16_t)param; steppers[axis].setStepsPerMM(smuffConfig.stepsPerMM[axis]); } else stat = false; }
      else if (strcmp(cmd, endstopTrig) == 0)         { if (axis != -1) { smuffConfig.endstopTrg[axis] = (uint8_t)param; steppers[axis].setEndstopState(smuffConfig.endstopTrg[axis]); } else stat = false; }
      else if (strcmp(cmd, maxSpeed) == 0)            { if (axis != -1) { smuffConfig.maxSpeed[axis] = (uint16_t)param; steppers[axis].setMaxSpeed(smuffConfig.maxSpeed[axis]); } else stat = false; }
      else if (strcmp(cmd, accelSpeed) == 0)          { if (axis != -1) { smuffConfig.accelSpeed[axis] = (uint16_t)param; steppers[axis].setAcceleration(smuffConfig.accelSpeed[axis]); } else stat = false; }
      else if (strcmp(cmd, accelDist) == 0)           { if (axis != -1) { smuffConfig.accelDist[axis] = (uint8_t)param; steppers[axis].setAccelDistance(smuffConfig.accelDist[axis]); } else stat = false; }
      else if (strcmp(cmd, stepDelay) == 0)           { if (axis != -1) { smuffConfig.stepDelay[axis] = (uint8_t)param; } else stat = false; }
      else if (strcmp(cmd, stopOnStall) == 0)         { if (axis != -1) { smuffConfig.stepperStopOnStall[axis] = (param > 0); steppers[axis].setStopOnStallDetected(param > 0); } else stat = false; }
      else if (strcmp(cmd, ms3Config) == 0)           { if (axis != -1) smuffConfig.ms3config[axis] = (int8_t)param; else stat = false; }
      else if (strcmp(cmd, stall) == 0)               { if (axis != -1) setTMCStall(axis, stallPin, param, 0, -1); else stat = false; }
      else if (strcmp(cmd, msteps) == 0)              { if (axis != -1 && isMStepValid(param)) setTMCMicrosteps(axis, param); else stat = false; }
      else if (strcmp(cmd, mode) == 0)                { if (axis != -1 && param >= 0 && param <= 2) smuffConfig.stepperMode[axis] = (uint8_t)param; else { rangeError(serial, 0, 2); stat = false; } }
      else if (strcmp(cmd, drvrAdr) == 0)             { if (axis != -1 && param >= 0 && param <= 3) smuffConfig.stepperAddr[axis] = (int8_t)param;  else { rangeError(serial, 0, 3); stat = false; } }
      else if (strcmp(cmd, maxStallCount) == 0)       { if (axis != -1 && param >= 0 && param <= MAX_STALL_COUNT) setTMCMaxStall(axis, param); else { rangeError(serial, 0, MAX_STALL_COUNT); stat = false; } }
      else if (strcmp(cmd, power) == 0)               { if (axis != -1 && param > 0 && param <= MAX_POWER) setTMCPower(axis, param, -1, -1); else stat = false; }
      else if (strcmp(cmd, tmode) == 0)               { if (axis != -1 && param >= 0 && param <= 1 ) setTMCTmode(axis, stallPin, param);  else stat = false; }
      else if (strcmp(cmd, cstepmin) == 0)            { if (axis != -1 && param >= 0 && param <= 15) setTMCCSMin(axis, param);  else { rangeError(serial, 0, 15); stat = false; } }
      else if (strcmp(cmd, cstepmax) == 0)            { if (axis != -1 && param >= 0 && param <= 15) setTMCCSMax(axis, param);  else { rangeError(serial, 0, 15); stat = false; } }
      else if (strcmp(cmd, cstepdown) == 0)           { if (axis != -1 && param >= 0 && param <= 15) setTMCCSDown(axis, param); else { rangeError(serial, 0, 15); stat = false; } }
      else if (strcmp(cmd, toff) == 0)                { if (axis != -1 && param >= 0 && param <= 15) setTMCTOff(axis, param);   else { rangeError(serial, 0, 15); stat = false; } }
      else if (strcmp(cmd, pfactor) == 0)             { if (index != -1) { smuffConfig.purges[index] = (uint16_t) param; } else stat = false; }
      #if defined(MULTISERVO)
      else if (strcmp(cmd, servoOutput) == 0)         { if (index >= 0 && index < 16) { servoMapping[index] = (uint8_t) param; } else stat = false; }
      else if (strcmp(cmd, servoClosed) == 0)         { if (index >= 0 && index < 16) { servoPosClosed[index] = (uint8_t) param; } else stat = false; }
      else if (strcmp(cmd, wiper) == 0)               { servoMapping[16] = (uint8_t) param; } }
      #else
      else if (strcmp(cmd, servoClosed) == 0)         { if (index >= 0 && index < MAX_TOOLS) { servoPosClosed[index] = (uint8_t) param; } else stat = false; }
      else if (strcmp(cmd, servoOutput) == 0)         { /* ignore this */ }
      else if (strcmp(cmd, wiper) == 0)               { /* ignore this */ }
      #endif
      else {
        sprintf_P(tmp, P_UnknownParam, cmd);
        printResponse(tmp, serial);
        stat = false;
      }
    }
    else {
      sprintf_P(tmp, P_NoValue, cmd);
      printResponse(tmp, serial);
      stat = false;
    }
  }
  return stat;
}

void setTMCStall(int8_t axis, int8_t stallPin, int param, int cool, int trigg) {
  smuffConfig.stepperStall[axis] = (int8_t)param;
  steppers[axis].setEnabled(true);
  #ifdef HAS_TMC_SUPPORT
  if (drivers[axis] != nullptr && drivers[axis]->stealth())
  {
    drivers[axis]->SGTHRS(param);
    if (cool > 0)
      drivers[axis]->TCOOLTHRS(cool);
    if (trigg != -1)
      steppers[axis].setStallThreshold(trigg);
    if (stallPin != -1)
      attachInterrupt(digitalPinToInterrupt(stallPin), (axis == 0 ? isrStallDetectedX : axis == 1 ? isrStallDetectedY : isrStallDetectedZ), FALLING);
  }
  else {
    if (stallPin != -1)
      detachInterrupt(stallPin);
  }
    #if defined(SMUFF_V6S)
    if(axis == REVOLVER) steppers[REVOLVER].setEnabled(false);
    #endif
  #endif
}

void setTMCMaxStall(int8_t axis, int param) {
  smuffConfig.stepperMaxStallCnt[axis] = (int8_t)param;
  steppers[axis].setStallThreshold(smuffConfig.stepperMaxStallCnt[axis]);
}

void setTMCTmode(int8_t axis, int8_t stallPin, int param) {
  smuffConfig.stepperStealth[axis] = (param == 0);
  #ifdef HAS_TMC_SUPPORT
  steppers[axis].setEnabled(true);
  if (drivers[axis] != nullptr) {
    uint8_t toff = smuffConfig.stepperToff[axis] == -1 ? (param == 0) ? 3 : 4 : smuffConfig.stepperToff[axis];
    setDriverSpreadCycle(drivers[axis], (param > 0), smuffConfig.stepperStall[axis], smuffConfig.stepperCSmin[axis], smuffConfig.stepperCSmax[axis], smuffConfig.stepperCSdown[axis], toff);
    if (param == 0 && (stallPin != -1))
      attachInterrupt(digitalPinToInterrupt(stallPin), (axis == 0 ? isrStallDetectedX : axis == 1 ? isrStallDetectedY : isrStallDetectedZ), FALLING);
    else if (param == 1 && stallPin != -1)
      detachInterrupt(stallPin);
  }
  else {
    if (stallPin != -1)
      detachInterrupt(stallPin);
  }
    #if defined(SMUFF_V6S)
    if(axis == REVOLVER) steppers[REVOLVER].setEnabled(false);
    #endif
  #endif
}

void setTMCMicrosteps(int8_t axis, int param) {
  smuffConfig.stepperMicrosteps[axis] = (uint16_t)param;
  #ifdef HAS_TMC_SUPPORT
  steppers[axis].setEnabled(true);
  if (drivers[axis] != nullptr)
    drivers[axis]->microsteps(smuffConfig.stepperMicrosteps[axis]);
    #if defined(SMUFF_V6S)
    if(axis == REVOLVER) steppers[REVOLVER].setEnabled(false);
    #endif
  #endif
}

void setTMCPower(int8_t axis, int param, int8_t irun, int8_t ihold) {
  smuffConfig.stepperPower[axis] = (uint16_t)param;
  #ifdef HAS_TMC_SUPPORT
  steppers[axis].setEnabled(true);
  if (drivers[axis] != nullptr) {
    drivers[axis]->rms_current(smuffConfig.stepperPower[axis]);
    if (irun >= 0 && irun <= 31)
      drivers[axis]->irun(irun);
    if (ihold >= 0 && ihold <= 31)
      drivers[axis]->ihold(ihold);
  }
    #if defined(SMUFF_V6S)
    if(axis == REVOLVER) steppers[REVOLVER].setEnabled(false);
    #endif
  #endif
}

void setTMCCSMin(int8_t axis, int param) {
  smuffConfig.stepperCSmin[axis] = (int8_t)param;
  #ifdef HAS_TMC_SUPPORT
  steppers[axis].setEnabled(true);
  if (drivers[axis] != nullptr)
    drivers[axis]->semin(smuffConfig.stepperCSmin[axis]);
    #if defined(SMUFF_V6S)
    if(axis == REVOLVER) steppers[REVOLVER].setEnabled(false);
    #endif
  #endif
}

void setTMCCSMax(int8_t axis, int param) {
  smuffConfig.stepperCSmax[axis] = (int8_t)param;
  #ifdef HAS_TMC_SUPPORT
  steppers[axis].setEnabled(true);
  if (drivers[axis] != nullptr)
    drivers[axis]->semax(smuffConfig.stepperCSmax[axis]);
    #if defined(SMUFF_V6S)
    if(axis == REVOLVER) steppers[REVOLVER].setEnabled(false);
    #endif
  #endif
}

void setTMCCSDown(int8_t axis, int param) {
  smuffConfig.stepperCSdown[axis] = (int8_t)param;
  #ifdef HAS_TMC_SUPPORT
  steppers[axis].setEnabled(true);
  if (drivers[axis] != nullptr) {
    drivers[axis]->sedn(smuffConfig.stepperCSdown[axis]);
    drivers[axis]->seup(smuffConfig.stepperCSdown[axis]);
  }
    #if defined(SMUFF_V6S)
    if(axis == REVOLVER) steppers[REVOLVER].setEnabled(false);
    #endif
  #endif
}

void setTMCTOff(int8_t axis, int param) {
  smuffConfig.stepperToff[axis] = (int8_t)param;
  #ifdef HAS_TMC_SUPPORT
  steppers[axis].setEnabled(true);
  if (drivers[axis] != nullptr)
    drivers[axis]->toff(smuffConfig.stepperToff[SELECTOR]);
    #if defined(SMUFF_V6S)
    if(axis == REVOLVER) steppers[REVOLVER].setEnabled(false);
    #endif
  #endif
}

bool M206(const char *msg, String buf, int8_t serial)
{
  bool stat = true;
  printResponse(msg, serial);
  if (buf.length() == 0)
  {
    printOffsets(serial);
    return stat;
  }
  float paramF;
  if ((paramF = getParamF(buf, X_Param)) != -1)
  {
    if (paramF >= 0.0 && paramF <= 21.0)
      smuffConfig.firstToolOffset = paramF;
    else
      stat = false;
  }
  if ((param = getParam(buf, Y_Param)) != -1)
  {
    if (param > 0 && param <= 8640)
    {
      smuffConfig.firstRevolverOffset = (uint16_t)param;
    }
    else
      stat = false;
  }
  return stat;
}

void xlateSpeed205(int8_t axis, uint16_t speed, int8_t serial) {
  unsigned long spd = translateSpeed(speed, axis, true);
  char tmp[50];
  sprintf_P(tmp, P_XlateSpeedResponse, String(spd).c_str());
  printResponse(tmp, serial);
}


bool M250(const char *msg, String buf, int8_t serial)
{
  bool stat = true;
  if ((param = getParam(buf, C_Param)) != -1)
  {
    if (param >= 60 && param < 256)
    {
      display.setContrast(param);
      smuffConfig.lcdContrast = (uint8_t)param;
      printResponse(msg, serial);
    }
    else
      stat = false;
  }
  else
  {
    printResponse(msg, serial);
    char tmp[50];
    sprintf_P(tmp, P_M250Response, smuffConfig.lcdContrast);
    printResponse(tmp, serial);
  }
  return stat;
}

bool M280(const char *msg, String buf, int8_t serial)
{
  bool stat = true;
  int8_t servoIndex = 0;
  char tmp[80];
  printResponse(msg, serial);
  if ((param = getParam(buf, I_Param)) != -1)
  {
    smuffConfig.servoMinPwm = (uint16_t)param;
    setServoMinPwm(servoIndex, param);
  }
  if ((param = getParam(buf, J_Param)) != -1)
  {
    smuffConfig.servoMaxPwm = (uint16_t)param;
    setServoMaxPwm(servoIndex, param);
  }
  if ((param = getParam(buf, P_Param)) != -1)
  {
    servoIndex = param;
  }
  if ((param = getParam(buf, S_Param)) != -1)
  {
    if (!setServoPos(servoIndex, (uint8_t)param))
      stat = false;
  }
  else if ((param = getParam(buf, F_Param)) != -1)
  {
    if (!setServoMS(servoIndex, (uint16_t)param))
      stat = false;
  }
  else
    stat = false;
  if ((param = getParam(buf, R_Param)) != -1)
  {
    setServoLid(param == 1 ? SERVO_CLOSED : SERVO_OPEN);
    stat = true;
  }
  if ((param = getParam(buf, T_Param)) != -1)
  {
    if(param == 0)
      param = 5;
    for (uint8_t i = 0; i <= 180; i += param)
    {
      sprintf_P(tmp, PSTR("Servo pos.: %d deg\n"), i);
      printResponse(tmp, serial);
      setServoPos(servoIndex, i);
      if(param >= 5)
        delay(750);
      stat = true;
    }
  }
  delay(50);
  return stat;
}

bool M300(const char *msg, String buf, int8_t serial)
{
  bool stat = true;
  printResponse(msg, serial);
  char sequence[300];
  char tuneData[150];
  char filename[80];
  if ((param = getParam(buf, S_Param)) != -1)
  {
    int frequency = param;
    if ((param = getParam(buf, P_Param)) != -1)
    {
      _tone(frequency, param);
    }
    else
      stat = false;
  }
  else if (getParamString(buf, F_Param, filename, ArraySize(filename)))
  {
    if(readTune(filename, tuneData, ArraySize(tuneData)))
      prepareSequence(tuneData, true);
  }
  else if (getParamString(buf, T_Param, sequence, ArraySize(sequence) - 1))
  {
    prepareSequence(sequence, true);
  }
  else
    stat = false;
  return stat;
}

bool isMStepValid(int param) {
  return (param == 0 || param == 2 || param == 4 || param == 8 || param == 16 || param == 32 || param == 64 || param == 128 || param == 256);
}

bool M350(const char *msg, String buf, int8_t serial)
{
  bool stat = true;
  printResponse(msg, serial);

  if (!hasParam(buf, X_Param) && !hasParam(buf, Y_Param) && !hasParam(buf, Z_Param))
  {
    printDriverMS(serial);
    return true;
  }

  if ((param = getParam(buf, X_Param)) != -1) {
    if (isMStepValid(param)) setTMCMicrosteps(SELECTOR, param); else stat = false;
  }

  if ((param = getParam(buf, Y_Param)) != -1) {
    if (isMStepValid(param)) setTMCMicrosteps(REVOLVER, param); else stat = false;
  }

  if ((param = getParam(buf, Z_Param)) != -1) {
    if (isMStepValid(param)) setTMCMicrosteps(FEEDER, param); else stat = false;
  }
  return stat;
}

bool M412(const char *msg, String buf, int8_t serial)
{
  bool stat = true;
  printResponse(msg, serial);
  if ((param = getParam(buf, S_Param)) != -1)
  {
    smuffConfig.runoutDetection = param == 1;
  }
  else
  {
    printResponseP(smuffConfig.runoutDetection ? P_On : P_Off, serial);
  }
  return stat;
}

bool M500(const char *msg, String buf, int8_t serial)
{
  printResponse(msg, serial);
  bool stat = false;
  if (writeConfig()) {
    if (writeTmcConfig()) {
      if (writeServoMapping()) {
        if (writeMaterials()) {
          if (writeRevolverMapping()) {
            stat = true;
          }
        }
      }
    }
  }
  return stat;
}

bool M502(const char *msg, String buf, int8_t serial)
{
  printResponse(msg, serial);
  return false;
}

Print* getSerialInstance(int8_t serial) {
  Print *_print = &Serial;
  switch (serial)
  {
    case 0:
      _print = &Serial;
      break;
    case 1:
      _print = &Serial1;
      break;
    case 2:
      _print = &Serial2;
      break;
  #if !defined(__ESP32__)
    case 3:
      _print = &Serial3;
      break;
  #endif
    default:
      break;
  }
  return _print;

}

bool M503(const char *msg, String buf, int8_t serial)
{
  printResponse(msg, serial);
  Print *_print = getSerialInstance(serial);
  int8_t part = 0;
  bool useWI = false;
  if ((param = getParam(buf, S_Param)) != -1)
  {
    part = param;
  }
  if(hasParam(buf, W_Param))
    useWI = true;
  if (part == 0 || part == 1) {
    printResponseP(P_M503S1, serial);
    writeMainConfig(_print, useWI);
  }
  if (part == 0 || part == 2) {
    printResponseP(P_M503S2, serial);
    writeSteppersConfig(_print, useWI);
  }
  if (part == 0 || part == 3) {
    printResponseP(P_M503S3, serial);
    writeTmcConfig(_print, useWI);
  }
  if (part == 0 || part == 4) {
    printResponseP(P_M503S4, serial);
    writeServoMapping(_print, useWI);
  }
  if (part == 0 || part == 5) {
    printResponseP(P_M503S5, serial);
    writeMaterials(_print, useWI);
  }
  if (part == 6) {
    printResponseP(P_M503S6, serial);
    writeSwapTools(_print, useWI);
  }
  if (part == 7) {
    printResponseP(P_M503S7, serial);
    writeRevolverMapping(_print, useWI);
  }
  if (part == 8) {
    printResponseP(P_M503S8, serial);
    writefeedLoadState(_print, useWI);
  }
  printResponseP(PSTR("\n"), serial);
  return true;
}

bool writeToCfgFile(const char* buf, const char* filename) {

  __debugS(PSTR("Attempt to write '%s'"), filename);
  return true;
}

bool M504(const char *msg, String buf, int8_t serial)
{
  char cfg[128];
  char target[40];
  printResponse(msg, serial);
  if (!getParamString(buf, P_Param, target, ArraySize(target))) {
    __debugS(PSTR("No target applied"));
    return false;
  }
  //__debugS(PSTR("buf: %s"), buf.c_str());
  if (getParamString(buf, S_Param, cfg, ArraySize(cfg))) {
    //__debugS(PSTR("cfg: %s"), cfg);
    if(strcmp(target, "Swaps") == 0) {
      if(deserializeSwapTools(cfg)) {
        saveStore();
        return true;
      }
    }
    else {
      __debugS(PSTR("Unknown target '%s'"), target);
      return false;
    }
  }
  else {
    __debugS(PSTR("No data applied"));
  }
  return false;
}

bool M569(const char *msg, String buf, int8_t serial)
{
#ifndef HAS_TMC_SUPPORT
  return false;
#else
  bool stat = true;
  char tmp[128];
  printResponse(msg, serial);

  if (!hasParam(buf, X_Param) && !hasParam(buf, Y_Param) && !hasParam(buf, Z_Param)) {
    printDriverMode(serial);
    return true;
  }

  if ((param = getParam(buf, X_Param)) != -1) {
    if (param >= 0 && param <= 1) setTMCTmode(SELECTOR, STALL_X_PIN, param); else stat = false;
  }

  if ((param = getParam(buf, Y_Param)) != -1) {
    if (param >= 0 && param <= 1) setTMCTmode(REVOLVER, STALL_Y_PIN, param); else stat = false;
  }

  if ((param = getParam(buf, Z_Param)) != -1) {
    if (param >= 0 && param <= 1) setTMCTmode(FEEDER, STALL_Z_PIN, param); else stat = false;
  }

  return stat;
#endif
}

bool M575(const char *msg, String buf, int8_t serial)
{
  bool stat = true;
  long paramL;
  int8_t port = -1;
  printResponse(msg, serial);
  if ((param = getParam(buf, P_Param)) != -1)
  {
    port = param;
  }
  if ((paramL = getParamL(buf, S_Param)) != -1)
  {
    if (port != -1)
    {
      switch (port)
      {
      case 0:
        smuffConfig.serialBaudrates[0] = paramL;
        break;
      case 1:
        smuffConfig.serialBaudrates[1] = paramL;
        break;
      case 2:
        smuffConfig.serialBaudrates[2] = paramL;
        break;
#if !defined(__ESP32__)
      case 3:
        smuffConfig.serialBaudrates[3] = paramL;
        break;
#endif
      }
    }
    else
    {
      smuffConfig.serialBaudrates[0] = paramL;
      smuffConfig.serialBaudrates[1] = paramL;
      smuffConfig.serialBaudrates[2] = paramL;
#if !defined(__ESP32__)
      smuffConfig.serialBaudrates[3] = paramL;
#endif
    }
    setupSerial();
  }
  else
    stat = false;
  return stat;
}

bool M577(const char *msg, String buf, int8_t serial) {
  if ((param = getParam(buf, S_Param)) != -1) {
    setSignalPort(SELECTOR_SIGNAL, param & 1);
    setSignalPort(FEEDER_SIGNAL, param & 2);
  }
  else
    printDuetSignalStates(serial);
  return true;
}

bool M700(const char *msg, String buf, int8_t serial)
{
  bool stat = false;
  printResponse(msg, serial);
  currentSerial = serial;
  int8_t tool = toolSelected;
  if (tool >= 0 && tool <= MAX_TOOLS)
  {
    if(hasParam(buf, P_Param)) {
      purgeFilament();
      stat = true;
    }
    else if(hasParam(buf, S_Param)) {
      if(smuffConfig.feedLoadState[tool] == SPL_NOT_LOADED) {
        stat = loadToSplitter(false);
      }
    }
    else
      stat = loadFilament();
  }
  return stat;
}

bool M701(const char *msg, String buf, int8_t serial)
{
  bool stat = false;
  printResponse(msg, serial);
  currentSerial = serial;
  int8_t tool = getToolSelected();
  if(hasParam(buf, R_Param)) {
    int slot = getParam(buf, R_Param);
    if(slot == -1) {
      // reset all feed loaded states
      for(uint8_t i=0; i < MAX_TOOLS; i++) {
        smuffConfig.feedLoadState[i] = SPL_NOT_LOADED;
      }
      stat = true;
    }
    else if(slot >=0 && slot < MAX_TOOLS) {
      // reset feed loaded status for this particular slot only
      smuffConfig.feedLoadState[slot] = SPL_NOT_LOADED;
      stat = true;
    }
    saveStore();
    if(smuffConfig.webInterface) {
      printResponseP(P_M503S8, serial);
      writefeedLoadState(getSerialInstance(serial), true);
    }
  }
  else if (tool >= 0 && tool <= MAX_TOOLS) {
    if(hasParam(buf, S_Param) && smuffConfig.useSplitter) {
      if(smuffConfig.feedLoadState[tool] == SPL_NOT_LOADED) {
        //__debugS(PSTR("Not loaded. Doing nothing"));
        stat = true;
      }
      if(smuffConfig.feedLoadState[tool] > SPL_NOT_LOADED) {
        //__debugS(PSTR("Unloading from Splitter"));
        stat = unloadFromSplitter(false);
      }
    }
    else {
      //__debugS(PSTR("Unloading from Nozzle"));
      stat = unloadFilament();
    }
  }
  else {
    __debugS(PSTR("No tool selected"));
  }
  return stat;
}

bool M906(const char *msg, String buf, int8_t serial)
{
#ifndef HAS_TMC_SUPPORT
  return false;
#else
  bool stat = true;
  printResponse(msg, serial);

  if (!hasParam(buf, X_Param) && !hasParam(buf, Y_Param) && !hasParam(buf, Z_Param)) {
    printDriverRms(serial);
    return true;
  }
  int8_t irun = (int8_t)getParam(buf, R_Param);  // motor run current
  int8_t ihold = (int8_t)getParam(buf, H_Param); // motor standstill current

  if ((param = getParam(buf, X_Param)) != -1) {
    if (param > 0 && param <= MAX_POWER) setTMCPower(SELECTOR, param, irun, ihold); else stat = false;
  }

  if ((param = getParam(buf, Y_Param)) != -1) {
    if (param > 0 && param <= MAX_POWER) setTMCPower(REVOLVER, param, irun, ihold); else stat = false;
  }

  if ((param = getParam(buf, Z_Param)) != -1) {
    if (param > 0 && param <= MAX_POWER) setTMCPower(FEEDER, param, irun, ihold); else stat = false;
  }
  return stat;
#endif
}

bool M914(const char *msg, String buf, int8_t serial)
{
#ifndef HAS_TMC_SUPPORT
  return false;
#else
  bool stat = true;
  int cool = 0;
  printResponse(msg, serial);

  if (!hasParam(buf, X_Param) && !hasParam(buf, Y_Param) && !hasParam(buf, Z_Param))
  {
    printDriverStallThrs(serial);
    return true;
  }
  if (hasParam(buf, C_Param))  {
    cool = getParam(buf, C_Param);
  }
  int trigg = getParam(buf, T_Param);

  if ((param = getParam(buf, X_Param)) != -1) {
    if (param > 0 && param <= 255) setTMCStall(SELECTOR, STALL_X_PIN, param, cool, trigg); else stat = false;
  }

  if ((param = getParam(buf, Y_Param)) != -1) {
    if (param > 0 && param <= 255) setTMCStall(REVOLVER, STALL_Y_PIN, param, cool, trigg); else stat = false;
  }

  if ((param = getParam(buf, Z_Param)) != -1) {
    if (param > 0 && param <= 255) setTMCStall(FEEDER, STALL_Z_PIN, param, cool, trigg); else stat = false;
  }

  return stat;
#endif
}

bool M997(const char *msg, String buf, int8_t serial)
{
  bool stat = false;
  printResponse(msg, serial);
  gotFirmware = false;
  memset(firmware, 0, ArraySize(firmware));

  if(getParamString(buf, P_Param, firmware, ArraySize(firmware))) {
    SD.remove(firmware);
    if(hasParam(buf, L_Param)) {
      if(initSD()) {
        if(!upload.open(firmware, O_WRITE | O_CREAT)) {
          __debugS(PSTR("Can't open file '%s' for downloading"), firmware);
          return false;
        }
      }
      else {
        __debugS(PSTR("Can't init SD-Card for file '%s'"), firmware);
        return false;
      }
      uploadLen = getParamL(buf, L_Param);
      gotFirmware = true;
      isUpload = true;
      stat = true;
    }
  }
  return stat;
}

bool M999(const char *msg, String buf, int8_t serial)
{
  printResponse(msg, serial);
  // turn off all stepper motors, just in case
  steppers[SELECTOR].setEnabled(false);
  steppers[REVOLVER].setEnabled(false);
  steppers[FEEDER].setEnabled(false);
  delay(500);
#if __STM32F1__
  nvic_sys_reset();
#elif __ESP32__
  ESP.restart();
#endif
  return true;
}

bool M2000(const char *msg, String buf, int8_t serial)
{
  char s[80];
  char tmp[128];
  printResponse(msg, serial);
  getParamString(buf, S_Param, tmp, sizeof(tmp));
  if (strlen(tmp) > 0)
  {
    printResponseP(PSTR("B"), serial);
    for (unsigned i = 0; i < strlen(tmp); i++)
    {
      sprintf_P(s, PSTR("%d:"), (char)tmp[i]);
      printResponse(s, serial);
    }
    printResponseP(PSTR("10\n"), serial);
  }
  return true;
}

bool M2001(const char *msg, String buf, int8_t serial)
{
  char tmp[128];
  printResponse(msg, serial);
  getParamString(buf, S_Param, tmp, ArraySize(tmp));
  String data = String(tmp);
  data.trim();
  if (data.length() > 0)
  {
    uint8_t ndx = 0;
    uint8_t pos = 0;
    if (data.startsWith("B"))
    {
      printResponseP(PSTR(">>"), serial);
      ndx++;
      do
      {
        pos = data.indexOf(":", ndx);
        uint8_t c;
        if (pos != -1)
        {
          c = data.substring(ndx, pos).toInt();
        }
        else
        {
          c = data.substring(ndx).toInt();
        }
        if (c == 10)
        {
          printResponseP(PSTR("\\n"), serial);
        }
        else
        {
          sprintf_P(tmp, PSTR("%c"), c);
          printResponse(tmp, serial);
        }
        ndx = pos + 1;
      } while (pos != -1);
      printResponseP(PSTR("<<\n"), serial);
    }
    else
    {
      printResponseP(P_WrongFormat, serial);
      return false;
    }
  }
  else
    return false;
  return true;
}

/*========================================================
 * Class G
 ========================================================*/
bool G0(const char *msg, String buf, int8_t serial)
{
  printResponse(msg, serial);
  if ((param = getParam(buf, Y_Param)) != -1)
  {
    steppers[REVOLVER].setEnabled(true);
    if (getParam(buf, S_Param))
    {
      // for testing only
      toolSelected = (int8_t)param;
      positionRevolver();
    }
    else
    {
      #if defined(SMUFF_V6S)
      prepSteppingAbsMillimeter(REVOLVER, stepperPosClosed[toolSelected], true);
      #else
      prepSteppingAbs(REVOLVER, smuffConfig.firstRevolverOffset + ((param)*smuffConfig.revolverSpacing), true);
      #endif
      runAndWait(REVOLVER);
      #if defined(SMUFF_V6S)
      lidOpen = steppers[REVOLVER].getEndstopHit();
      steppers[REVOLVER].setEnabled(false);
      #endif
    }
  }
  if ((param = getParam(buf, X_Param)) != -1)
  {
    steppers[SELECTOR].setEnabled(true);
    bool posOk = false;
    uint8_t retry = 3;
    do
    {
      steppers[SELECTOR].resetStallDetected();
      prepSteppingAbsMillimeter(SELECTOR, smuffConfig.firstToolOffset + (param * smuffConfig.toolSpacing));
      runAndWait(SELECTOR);
      if (steppers[SELECTOR].getStallDetected())
      {
        handleStall(SELECTOR);
        posOk = false;
      }
      else
        posOk = true;
      retry--;
      if (!retry)
        break;
    } while (!posOk);
  }
  return true;
}

uint16_t handleFeedSpeed(String buf, uint8_t axis)
{
  if (!hasParam(buf, F_Param))
  {
    // if no F param was applied, return the default speed
    return smuffConfig.maxSpeed[axis];
  }
  else
  {
    uint16_t fspeed = (uint16_t)getParam(buf, F_Param);
    if (fspeed < mmsMin)
      fspeed = mmsMin;
    if (fspeed > mmsMax)
      fspeed = mmsMax;
    uint16_t ticks = (uint16_t)translateSpeed(fspeed, axis);
    steppers[axis].setMaxSpeed(ticks);
    if (ticks > steppers[axis].getAcceleration())
    {
      // lower the acceleration if it's above the feed speed by factor 3
      uint32_t faccel = (uint32_t)(ticks * 3);
      if (faccel >= 65535)
        faccel = 65535;
      steppers[axis].setAcceleration((uint16_t)faccel);
      //__debugS(PSTR("faccel ticks now = %u"), steppers[axis].getAcceleration());
    }
    return fspeed;
  }
}

bool G1(const char *msg, String buf, int8_t serial)
{
  long curSpeed[NUM_STEPPERS], accel[NUM_STEPPERS];
  printResponse(msg, serial);

  // save current speeds
  curSpeed[SELECTOR] = steppers[SELECTOR].getMaxSpeed();
  accel[SELECTOR] = steppers[SELECTOR].getAcceleration();
  curSpeed[REVOLVER] = steppers[REVOLVER].getMaxSpeed();
  accel[REVOLVER] = steppers[REVOLVER].getAcceleration();
  curSpeed[FEEDER] = steppers[FEEDER].getMaxSpeed();
  accel[FEEDER] = steppers[FEEDER].getAcceleration();

  bool isMill = !hasParam(buf, T_Param);
  float paramF;
  long paramL;
  uint16_t speed;
  const char P_DIinfo[] PROGMEM = { "[G1] Moving %c: %2.f %s with speed %ld %s" };

  if (hasParam(buf, X_Param))
  {
    paramF = getParamF(buf, X_Param);
    paramL = isMill ? round(paramF * steppers[SELECTOR].getStepsPerMM()) : round(paramF);
    speed = handleFeedSpeed(buf, SELECTOR);
    __debugS(P_DIinfo, 'X', paramF, isMill ? "mm" : "steps", speed, smuffConfig.speedsInMMS ? "mm/s" : "ticks");
    steppers[SELECTOR].setEnabled(true);
    prepStepping(SELECTOR, paramL, false, true);
    toolSelected = -1;
  }
  if (hasParam(buf, Y_Param))
  {
    paramF = getParamF(buf, Y_Param);
    paramL = isMill ? round(paramF * steppers[REVOLVER].getStepsPerMM()) : round(paramF);
    speed = handleFeedSpeed(buf, REVOLVER);
    __debugS(P_DIinfo, 'Y', paramF, isMill ? "mm" : "steps", speed, smuffConfig.speedsInMMS ? "mm/s" : "ticks");
    steppers[REVOLVER].setEnabled(true);
    prepStepping(REVOLVER, paramL, false, true);
  }
  if (hasParam(buf, Z_Param))
  {
    paramF = getParamF(buf, Z_Param);
    paramL = isMill ? round(paramF * steppers[FEEDER].getStepsPerMM()) : round(paramF);
    speed = handleFeedSpeed(buf, FEEDER);
    __debugS(P_DIinfo, 'Z', paramF, isMill ? "mm" : "steps", speed, smuffConfig.speedsInMMS ? "mm/s" : "ticks");
    steppers[FEEDER].setEnabled(true);
    prepStepping(FEEDER, paramL, false, true);
  }
  uint32_t start = millis();
  runAndWait(-1);

  // for testing only: check if stall was detected
  __debugS(PSTR("[G1] Move took: %d ms"), millis() - start);
#ifdef HAS_TMC_SUPPORT
  const char P_StallRes[] PROGMEM = {"[G1] StallResult %c: %d  Stalled: %s"};
  if (hasParam(buf, X_Param) && drivers[SELECTOR] != nullptr && !drivers[SELECTOR]->spread_en())
  {
    uint16_t sr = drivers[SELECTOR]->SG_RESULT();
    __debugS(P_StallRes, 'X', sr, steppers[SELECTOR].getStallDetected() ? P_Yes : P_No);
  }
  if (hasParam(buf, Y_Param) && drivers[REVOLVER] != nullptr && !drivers[REVOLVER]->spread_en())
  {
    uint16_t sr = drivers[REVOLVER]->SG_RESULT();
    __debugS(P_StallRes, 'Y', sr, steppers[REVOLVER].getStallDetected() ? P_Yes : P_No);
  }
  if (hasParam(buf, Z_Param) && drivers[FEEDER] != nullptr && !drivers[FEEDER]->spread_en())
  {
    uint16_t sr = drivers[FEEDER]->SG_RESULT();
    __debugS(P_StallRes, 'Z', sr, steppers[FEEDER].getStallDetected() ? P_Yes : P_No);
  }
#endif
  // set all speeds back to the configured values
  if (hasParam(buf, F_Param))
  {
    steppers[SELECTOR].setMaxSpeed(curSpeed[SELECTOR]);
    steppers[SELECTOR].setAcceleration(accel[SELECTOR]);
    steppers[REVOLVER].setMaxSpeed(curSpeed[REVOLVER]);
    steppers[REVOLVER].setAcceleration(accel[REVOLVER]);
    steppers[FEEDER].setMaxSpeed(curSpeed[FEEDER]);
    steppers[FEEDER].setAcceleration(accel[FEEDER]);
  }
  #if defined(SMUFF_V6S)
  lidOpen = steppers[REVOLVER].getEndstopHit();
  steppers[REVOLVER].setEnabled(false);
  #endif

  return true;
}

bool G4(const char *msg, String buf, int8_t serial)
{
  bool stat = true;
  printResponse(msg, serial);
  long val;
  if ((param = getParam(buf, S_Param)) != -1)
  {
    if (param > 0 && param < 500) {
      delay(param * 1000);
    }
  }
  else if ((val = getParamL(buf, P_Param)) != -1) {
    delay(val);
  }
  else
  {
    stat = false;
  }
  return stat;
}

bool G12(const char *msg, String buf, int8_t serial)
{
  printResponse(msg, serial);
  uint16_t wait = 500;
  uint8_t pos1 = 20;
  uint8_t pos2 = 45;
  uint8_t pos0 = 110;
  unsigned repeat = 10;
  String p;
  String seq = String(smuffConfig.wipeSequence);

  if (buf.length() == 0 && seq.length() == 0)
    return false;
  // sent sequence overrides internal sequence
  if (buf.length() == 0)
    p = seq;
  else
    p = buf;

  if ((param = getParam(p, S_Param)) != -1) {
    wait = (uint16_t)param; // defines the wipe speed
  }
  if ((param = getParam(p, I_Param)) != -1) {
    pos1 = (uint8_t)param; // defines position 1 when wiping
  }
  if ((param = getParam(p, J_Param)) != -1) {
    pos2 = (uint8_t)param; // defines position 2 when wiping
  }
  if ((param = getParam(p, P_Param)) != -1) {
    pos0 = (uint8_t)param; // defines release position
  }
  if ((param = getParam(p, R_Param)) != -1) {
    repeat = param; // defines the number of repeats
  }
  if (hasParam(p, C_Param)) {
    if(smuffConfig.useCutter) {
      cutFilament(false);
      return true;
    }
    return false;
  }

  unsigned n = 1;
  for (n = 0; n < repeat; n++)
  {
    setServoPos(SERVO_WIPER, pos1);
    delay(wait);
    setServoPos(SERVO_WIPER, pos2);
    delay(wait);
  }
  setServoPos(SERVO_WIPER, pos0);
  delay(100);
  disableServo(SERVO_WIPER);
  return true;
}

bool G28(const char *msg, String buf, int8_t serial)
{
  bool stat = true;
  printResponse(msg, serial);
  if (buf.length() == 0)
  {
    #if !defined(SMUFF_V6S)
    stat = moveHome(SELECTOR, false, true);
    if (stat)
      moveHome(REVOLVER, false, false);
    #else
    stat = moveHome(REVOLVER, false, false);
    if (stat)
      moveHome(SELECTOR, false, true);
    #endif
  }
  else
  {
    if (buf.indexOf(X_Param) != -1) {
      stat = moveHome(SELECTOR, false, false);
    }
    if (buf.indexOf(Y_Param) != -1) {
      stat = moveHome(REVOLVER, false, false);
    }
  }
  return stat;
}

bool G90(const char *msg, String buf, int8_t serial)
{
  printResponse(msg, serial);
  positionMode = ABSOLUTE;
  return true;
}

bool G91(const char *msg, String buf, int8_t serial)
{
  printResponse(msg, serial);
  positionMode = RELATIVE;
  return true;
}
