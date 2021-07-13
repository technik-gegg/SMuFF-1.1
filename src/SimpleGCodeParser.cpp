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
 * Module implementing a simple G-Code parser
 */

#include "SMuFF.h"

extern ZStepper steppers[];
extern int8_t currentSerial;

volatile bool parserBusy = false;
volatile bool sendingResponse = false;
uint16_t currentLine = 0;
volatile bool actionOk = false;

void parseGcode(const String& serialBuffer, int8_t serial) {

  String line = String(serialBuffer);
  resetSerialBuffer(serial);

  if(line.length()==0)
    return;

  if(line.startsWith("//ACTION:")) {    // check for //action commands sent from controller
    String tmp = String(line).substring(9);
    parse_Action(tmp, serial);
    return;
  }

  //__debugS(PSTR("Line: %s %d"), line.c_str(), line.length());

  int16_t pos;
  if((pos = line.lastIndexOf("*")) > -1) {
    line = line.substring(0, pos);
  }
  if((pos = line.lastIndexOf(";")) > -1) {
    if(pos==0) {
      return;
    }
    line = line.substring(0, pos);
  }
  currentLine = 0;
  if(line.startsWith("N")) {
    char ln[15];
    if((int)(currentLine = getParam(line, (char*)"N")) != -1) {
      sprintf_P(ln, PSTR("%d"), currentLine);
      line = line.substring(strlen(ln)+1);
    }
  }

  if(parserBusy || !steppers[FEEDER].getMovementDone()) {
    if(!smuffConfig.prusaMMU2) {
      sendErrorResponseP(serial, P_Busy);
      return;
    }
    else if(!line.startsWith("A") &&
            !line.startsWith("P") &&
            !line.startsWith("T") &&
            !line.startsWith("C") &&
            !line.startsWith("U") ) { // only ABORT or FINDA commands are being processed while parser is busy
            //__debugS(PSTR("NO valid PMMU GCode"));
      return;
    }
    //__debugS(PSTR("Error: parserBusy: %d feederBusy: %d"), parserBusy, steppers[FEEDER].getMovementDone());
  }
  if(smuffConfig.prusaMMU2) {
    char ptmp[80];
    if(line.startsWith("T")) {
      // If a tool change command is pending while the feeder is still active,
      // request a resend of the last command, so we don't ignore and loose the command
      // as we do with the 'U' and 'A' commands.
      // The printer is supposed to send another 'T' command after a while.
      if(!steppers[FEEDER].getMovementDone()) {
        //__debugS(PSTR("Wait after 'T' 500ms"));
        delay(500);
        #if !defined(MARLIN2_ONLY)
        if(currentLine > 0)
          sprintf_P(ptmp, PSTR("M998 %d\n"), currentLine);
        else
          sprintf_P(ptmp, PSTR("M998\n"));
        printResponseP(ptmp, serial);
        //__debugS(PSTR("Resend 'T' sent"));
        #endif
        return;
      }
    }
    if(line.startsWith("U") || line.startsWith("C")) {
      if(!steppers[FEEDER].getMovementDone()) {
        #if !defined(MARLIN2_ONLY)
        sendOkResponse(serial);
        //__debugS(PSTR("Cancelling U/C"));
        #endif
        return;
      }
    }
    else if(line.startsWith("A")) {
      //__debugS(PSTR("*ABORT* received"));
      if(!steppers[FEEDER].getMovementDone()) {
        setAbortRequested(true);
        //__debugS(PSTR("Abort set"));
      }
      sendOkResponse(serial);
      return;
    }
  }

  setParserBusy();
  currentSerial = serial;

  if(line.startsWith("G")) {
    if(parse_G(line.substring(1), serial))
      sendOkResponse(serial);
    else
      sendErrorResponseP(serial);
    setParserReady();
    return;
  }
  else if(line.startsWith("M")) {
    if(parse_M(line.substring(1), serial))
      sendOkResponse(serial);
    else
      sendErrorResponseP(serial);
    setParserReady();
    return;
  }
  else if(line.startsWith("T")) {
    if(parse_T(line.substring(1), serial))
      sendOkResponse(serial);
    else
      sendErrorResponseP(serial);
    setParserReady();
    return;
  }
  else if(line.startsWith("S") || // GCodes for Prusa MMU2 emulation
          line.startsWith("P") ||
          line.startsWith("C") ||
          line.startsWith("L") ||
          line.startsWith("U") ||
          line.startsWith("E") ||
          line.startsWith("K") ||
          line.startsWith("X") ||
          line.startsWith("F") ||
          line.startsWith("R") ||
          line.startsWith("W") ||
          line.startsWith("A")) {
    //if(!line.startsWith("P")) __debugS(PSTR("From Prusa: '%s'"), line.c_str());
    parse_PMMU2(line.charAt(0), line.substring(1), serial);
    setParserReady();
    return;
  }
  else {
    char tmp[256];
    sprintf_P(tmp, P_UnknownCmd, line.c_str());
    __debugS(PSTR("ParseGcode err: %s"), tmp);
    if(!smuffConfig.prusaMMU2) {
      sendErrorResponseP(serial, tmp);
    }
  }
  setParserReady();
  currentSerial = -1;
}

bool parse_T(const String& buf, int8_t serial) {
  bool stat = true;

  if(buf.length()==0) {
    sendToolResponse(serial);
    return stat;
  }
  if(toUpperCase(buf[0])=='M') {
    maintainTool();
    return true;
  }
  int8_t tool = buf.toInt();
  int16_t param;

  char msg[20];
  uint16_t ofs = 0;
  sprintf_P(msg, P_TResponse, tool);
  ofs = String(msg).length()-2;

  if(tool == -1 || tool == 255) {
    parse_G(String("28"), serial);
  }
  else if(tool >= 0 && tool <= smuffConfig.toolCount-1) {
    //__debugS(PSTR("Tool change requested: T%d"), tool);
    if(feederEndstop()) {
      // Prusa expects the MMU to unload filament on its own before tool change
      // Same goes if the Feeder stepper is declared shared
      if(smuffConfig.prusaMMU2 || smuffConfig.isSharedStepper)
        //__debugS(PSTR("must unload first!"));
        if(!smuffConfig.useSplitter)
          unloadFilament();
    }
    stat = selectTool(tool, false);
    if(stat) {
      if(!smuffConfig.prusaMMU2) {
        if((param = getParam(buf.substring(ofs), (char*)"S")) != -1) {
          //__debugS(PSTR("Tx has S Param: %d"), param);
          if(param == 1)
            loadFilament(false);
          else if(param == 0)
            unloadFilament();
        }
      }
    }
    #if defined(USE_DDE)
    switchFeederStepper(EXTERNAL);
    #else
    if(smuffConfig.isSharedStepper)
      switchFeederStepper(EXTERNAL);
    #endif

    if(!smuffConfig.prusaMMU2)  // Prusa doesn't expect "Tx" as a response
      printResponse(msg, serial);
  }
  else {
    sendErrorResponseP(serial, PSTR("Wrong tool selected."));
    stat = false;
  }

  return stat;
}

bool parse_Action(const String& buf, int8_t serial) {

  char tmp[256];

  if(buf.length()==0) {
    return false;
  }
  if(buf.startsWith("T:")) {
    actionOk = false;
    String msg = buf.substring(2);
    if(msg == "OK")
      actionOk = true;
    else {
      sprintf_P(tmp, P_ActionMsg, msg.c_str());
      drawUserMessage(tmp);
    }
  }
  else if(buf.startsWith("PING")) {
    char msg[30];
    sprintf_P(msg, P_Action, P_ActionPong);
    printResponse(msg, serial);
  }
  return false;
}

bool parse_G(const String& buf, int8_t serial) {
  bool stat = true;

  if(buf.length()==0) {
    sendGList(serial);
    return stat;
  }
  uint16_t code = buf.toInt();
  //__debugS(PSTR("G[%s]: >%d< %d"), buf.c_str(), code, buf.length());

  char msg[10];
  char tmp[50];
  sprintf_P(msg, P_GResponse, code);
  uint8_t ofs = String(msg).length()-2;

  for(uint16_t i=0; i< 999; i++) {
    if(gCodeFuncsG[i].code == -1)
      break;
    // deviation from standard GCode parser;
    // list help file if the first char is a question mark (?)
    if(buf.substring(ofs).charAt(0)=='?') {
      sprintf(tmp, "g%d", code);
      listHelpFile(tmp, serial);
      return true;
    }
    if(gCodeFuncsG[i].code == code) {
      return gCodeFuncsG[i].func(msg, buf.substring(ofs), serial);
    }
  }
  return false;
}

bool parse_M(const String& buf, int8_t serial) {
  bool stat = true;

  if(buf.length()==0) {
    sendMList(serial);
    return stat;
  }
  uint16_t code = buf.toInt();

  char msg[10];
  char tmp[50];
  sprintf_P(msg, P_MResponse, code);
  uint8_t ofs = String(msg).length()-2;

  for(uint16_t i=0; i< 999; i++) {
    if(gCodeFuncsM[i].code == -1)
      break;
    if(gCodeFuncsM[i].code == code) {
      // deviation from standard GCode parser;
      // list help file if the first char is a question mark (?)
      if(buf.substring(ofs).charAt(0)=='?') {
        sprintf(tmp, "m%d", code);
        listHelpFile(tmp, serial);
        return true;
      }
      //__debugS(PSTR("Calling: M"), gCodeFuncsM[i].code);
      return gCodeFuncsM[i].func(msg, buf.substring(ofs), serial);
    }
  }
  return false;
}

/**
 *  Parse pseudo GCodes to emulate a Prusa MMU2
 **/
bool parse_PMMU2(char cmd, const String& buf, int8_t serial) {

  char  tmp[80];
  bool toolOk = true;

  if(!smuffConfig.prusaMMU2) {
    sprintf_P(tmp, P_NoPrusa);
    sendErrorResponseP(serial, tmp);
    //__debugS(PSTR("No Prusa Emulation configured!"));
    return false;
  }

  bool  stat = true;
  int8_t   type = -1;
  if(buf.length()>0)
    type = buf.toInt();
  switch(cmd) {
    case 'A':
      // Aborted - we've already handeled that

      break;
    case 'S':     // Init (S0 | S1 | S2 | S3)
      switch(type) {
        case 0:
          sprintf_P(tmp,PSTR("ok\n"));
          break;
        case 1:
          sprintf_P(tmp,PSTR("%dok\n"), PMMU_VERSION);
          break;
        case 2:
          sprintf_P(tmp,PSTR("%dok\n"), PMMU_BUILD);
          break;
        case 3:
          sprintf_P(tmp,PSTR("%dok\n"), 0);
          break;
      }
      printResponse(tmp,serial);
      //__debugS(("To Prusa (S%d): '%s'"), type, tmp);
      break;

    case 'P':     // FINDA status (Feeder endstop)
        sprintf_P(tmp,PSTR("%dok\n"), feederEndstop() ? 1 : 0);
        printResponse(tmp,serial);
        //__debugS(PSTR("To Prusa (P%d): '%s'"), type, tmp);
      break;

    case 'C':     // Push filament to nozzle
      if(loadFilament()) {
        sendOkResponse(serial);
        //__debugS(PSTR("To Prusa (C%d): ok<CR>"), type);
      }
      break;

    case 'L':     // Load filament
      if(toolSelected != type)
        toolOk = selectTool(type, false);
      if(toolOk) {
        loadFilamentPMMU2();
        sendOkResponse(serial);
      }
      else
        sendErrorResponse(serial);
      //__debugS(PSTR("To Prusa (L%d): ok<CR>"), type);
      break;

    case 'U':     // Unload filament
      if(unloadFilament()) {
        sendOkResponse(serial);
        //__debugS(PSTR("To Prusa (U%d): ok<CR>"), type);
      }
      break;

    case 'E':     // Eject filament
      unloadFilament();
      sendOkResponse(serial);
      //__debugS(PSTR("To Prusa (E%d): ok<CR>"), type);
      break;

    case 'K':     // Cut filament
    case 'F':     // Set filament
    case 'R':     // Recover after eject
      sendOkResponse(serial);
      //__debugS(PSTR("To Prusa (%c%d): ok<CR>"), cmd, type);
      break;

    case 'W': {    // Wait for user click
      userBeep();
      uint16_t button = 999;
      do {
        button = showDialog(P_PMMU_Title, P_PMMU_Wait, P_PMMU_WaitAdd, P_OkButtonOnly);
      } while (button != 1);
      sendOkResponse(serial);
      //__debugS(PSTR("To Prusa (W%d): ok<CR>"), type);
      break;
     }
    case 'X':     // Reset MMU
      sendOkResponse(serial);
      //__debugS(PSTR("To Prusa (X%d): ok<CR>"), type);
    #if !defined(SOFTRESET)
      M999("", tmp, serial);
    #else
      // SOFTRESET needed if some bootloader sends messages at boot.
      // This will cause an buffer overrun in Marlins MMU code.
      // Hence, don't reset, just pretend to.
      delay(1000);
      sendStartResponse(serial);
    #endif
      break;

    default:
      sendErrorResponseP(serial);
      //__debugS(PSTR("To Prusa (%c%d): Error:...<CR>"), cmd, type);
      break;
  }
  return stat;
}

int16_t findTokenPos(String buf, const char* token) {
  bool isQuote = false;
  for(uint16_t i=0; i< buf.length(); i++) {
    if(buf.charAt(i) == '\"' && !isQuote) {
      isQuote = true;
      continue;
    }
    if(buf.charAt(i) == '\"' && isQuote) {
      isQuote = false;
      continue;
    }
    if(buf.charAt(i) == *token && !isQuote) {
      //__debugS(PSTR("Token found @: %d"), i);
      return (int)i;
    }
  }
  return -1;
}

bool hasParam(String buf, const char* token) {
  int16_t pos = findTokenPos(buf, token);
  //__debugS(PSTR("hasParam: %d >%s<\n"),pos, buf.c_str());
  return pos != -1;
}

int getParam(String buf, const char* token) {
  int16_t pos = findTokenPos(buf, token);
  if(pos != -1) {
    //__debugS(PSTR("getParam:pos: %d"),pos);
    return atoi(buf.substring(pos+1).c_str());
  }
  else
    return -1;
}

float getParamF(String buf, const char* token) {
  int16_t pos = findTokenPos(buf, token);
  if(pos != -1) {
    //__debugS(PSTR("getParam:pos: %d"),pos);
    return atof(buf.substring(pos+1).c_str());
  }
  else
    return -1;
}

long getParamL(String buf, const char* token) {
  int16_t pos = findTokenPos(buf, token);
  if(pos != -1) {
    //__debugS(PSTR("getParam:pos: %d"),pos);
    return atol(buf.substring(pos+1).c_str());
  }
  else
    return -1;
}

bool getParamString(String buf, const char* token, char* dest, int16_t bufLen) {
  int16_t pos = findTokenPos(buf, token);
  if(pos != -1) {
    if(buf.substring(pos+1).startsWith("\"")) {
      int16_t endPos = buf.substring(pos+2).indexOf("\"");
      //__debugS(PSTR("String @: %d-%d\n"), pos, endPos);
      if(endPos != -1) {
        memset(dest, 0, bufLen);
        if(endPos+1 < bufLen) {
          String tmp = buf.substring(pos+2, pos+endPos+3);
          tmp.replace("''", "\"");
          if(dest != nullptr) {
            tmp.toCharArray(dest, endPos+1);
            //__debugS(PSTR("ptmp: >%s< (%d)\n"), dest, strlen(dest));
          }
          return true;
        }
      }
    }
  }
  return false;
}

void prepStepping(int8_t index, long param, bool Millimeter /* = true */, bool ignoreEndstop /* = false */) {
  if(param != 0) {
    if(positionMode == RELATIVE) {
      if(Millimeter) prepSteppingRelMillimeter(index, param, ignoreEndstop);
      else prepSteppingRel(index, param, ignoreEndstop);
    }
    else {
      if(Millimeter) prepSteppingAbsMillimeter(index, param, ignoreEndstop);
      else prepSteppingAbs(index, param, ignoreEndstop);
    }
    remainingSteppersFlag |= _BV(index);
  }
}

void sendGList(int8_t serial) {
  listHelpFile(PSTR("gcmds"), serial);
}

void sendMList(int8_t serial) {
  listHelpFile(PSTR("mcmds"), serial);
}

void sendM205List(int8_t serial) {
  listHelpFile(PSTR("m205"), serial);
}

void sendToolResponse(int8_t serial) {
  char tmp[80];
  int8_t tool = getToolSelected();
  sprintf_P(tmp, P_TResponse, tool);
  printResponse(tmp, serial);
}

void sendErrorResponse(int8_t serial, const char* msg /* = nullptr */) {
  if(msg != nullptr)
    printResponse(msg, serial);
  sendOkResponse(serial);
}

void sendErrorResponseP(int8_t serial, const char* msg /* = nullptr */) {
  char tmp[128];
  sprintf_P(tmp, P_Error, msg == nullptr ? "" : msg);
  printResponse(tmp, serial);
  sendOkResponse(serial);
}

void sendOkResponse(int8_t serial) {
  printResponseP(P_Ok, serial);
}

void sendXon(Stream* serial) {
  char seq[] = { 0x11 };
  serial->write(seq, 1);
}

void sendXoff(Stream* serial) {
  char seq[] = { 0x13 };
  serial->write(seq, 1);
}

void sendStartResponse(int8_t serial){
  char tmp[50];
  char tmp1[15];
  if(!smuffConfig.prusaMMU2) {
    sprintf_P(tmp1, PSTR("Serial: %d"), serial);
    sprintf_P(tmp, P_Echo, tmp1);
    printResponse(tmp, serial);
  }
  printResponseP(P_Start, serial);
}

void printResponse(const char* response, int8_t serial) {
  sendingResponse = true;
  switch(serial) {
    case 0: Serial.print(response); break;
    case 1: if(CAN_USE_SERIAL1) Serial1.print(response); break;
    case 2: if(CAN_USE_SERIAL2) Serial2.print(response); break;
    case 3: if(CAN_USE_SERIAL3) Serial3.print(response); break;
  }
  sendingResponse = false;
}

void printResponseP(const char* response, int8_t serial) {
  sendingResponse = true;
  switch(serial) {
    case 0: Serial.print((__FlashStringHelper*)response); break;
    case 1: if(CAN_USE_SERIAL1) Serial1.print((__FlashStringHelper*)response); break;
    case 2: if(CAN_USE_SERIAL2) Serial2.print((__FlashStringHelper*)response); break;
    case 3: if(CAN_USE_SERIAL3) Serial3.print((__FlashStringHelper*)response); break;
  }
  sendingResponse = false;
}
