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

/*
 * Module implementing a simple G-Code parser
 */

#include "SMuFF.h"

volatile bool parserBusy = false;
volatile bool sendingResponse = false;
uint16_t currentLine = 0;
volatile bool actionOk = false;

void parseGcode(const String& serialBuffer, int8_t serial) {

  char errmsg[MAX_ERR_MSG];
  String line = String(serialBuffer);
  resetSerialBuffer(serial);

  if(line.length()==0)
    return;

  if(line.startsWith("//ACTION:")) {    // check for //action commands sent from controller
    String tmp = String(line).substring(9);
    parse_Action(tmp, serial, errmsg);
    return;
  }

  //__debugS(DEV3, PSTR("Line: %s %d"), line.c_str(), line.length());

  int16_t pos;
  if((pos = line.lastIndexOf("*")) > -1) {
    line = line.substring(0, pos);
  }
  if((pos = line.lastIndexOf(";")) > -1) {
    if(pos==0) {
      // line starts with comment char, dump that line completely
      return;
    }
    // otherwise eliminate comments behind commands
    line = line.substring(0, pos);
  }
  
  currentLine = 0;
  if(line.startsWith("N")) { // check whether the GCode starts with a line number, if so, extract it
    char ln[15];
    if((int)(currentLine = getParam(line, (char*)"N")) != -1) {
      sprintf_P(ln, PSTR("%d"), currentLine);
      line = line.substring(strlen(ln)+1);
    }
  }

  if(parserBusy || !steppers[FEEDER].getMovementDone()) {
    if(!smuffConfig.prusaMMU2) {
      if(!line.startsWith("M2")) {
        sendBusyResponse(serial);
        return;
      }
      else {
        // reset "Parser Busy" flag only
        setParserReady();
        return;
      }
    }
    else if(!line.startsWith("A") &&
            !line.startsWith("P") &&
            !line.startsWith("T") &&
            !line.startsWith("C") &&
            !line.startsWith("U") ) { // only ABORT or FINDA commands are being processed while parser is busy
            __debugS(D, PSTR("parseGcode: Not a valid PMMU GCode: '%s'"), line.substring(0, 1).c_str());
      return;
    }
    //__debugS(D, PSTR("Error: parserBusy: %d feederBusy: %d"), parserBusy, steppers[FEEDER].getMovementDone());
  }
  if(smuffConfig.prusaMMU2) {
    char ptmp[20];
    if(line.startsWith("T")) {
      // If a tool change command is pending while the feeder is still active,
      // request a resend of the last command, so we don't ignore and loose the command
      // as we do with the 'U' and 'A' commands.
      // The printer is supposed to send another 'T' command after a while.
      if(!steppers[FEEDER].getMovementDone()) {
        //__debugS(D, PSTR("Wait after 'T' 500ms"));
        delay(500);
        #if !defined(MARLIN2_ONLY)
          if(currentLine > 0)
            sprintf_P(ptmp, PSTR("M998 %d\n"), currentLine);
          else
            sprintf_P(ptmp, PSTR("M998\n"));
          printResponseP(ptmp, serial);
          __debugS(DEV3, PSTR("parseGcode: Resend 'T' sent"));
        #endif
        return;
      }
    }
    if(line.startsWith("U") || line.startsWith("C")) {
      if(!steppers[FEEDER].getMovementDone()) {
        #if !defined(MARLIN2_ONLY)
          sendOkResponse(serial);
          __debugS(DEV3, PSTR("parseGcode: Cancelling U/C command"));
        #endif
        return;
      }
    }
    else if(line.startsWith("A")) {
      __debugS(DEV3, PSTR("parseGcode: *ABORT* command received"));
      if(!steppers[FEEDER].getMovementDone()) {
        setAbortRequested(true);
        //__debugS(D, PSTR("Abort set"));
      }
      sendOkResponse(serial);
      return;
    }
  }

  setParserBusy();
  currentSerial = serial;

  if(line.startsWith("G")) {
    if(parse_G(line.substring(1), serial, errmsg))
      sendOkResponse(serial);
    else
      sendErrorResponse(serial, errmsg);
    setParserReady();
    return;
  }
  else if(line.startsWith("M")) {
    if(parse_M(line.substring(1), serial, errmsg))
      sendOkResponse(serial);
    else
      sendErrorResponse(serial, errmsg);
    setParserReady();
    return;
  }
  else if(line.startsWith("T")) {
    if(parse_T(line.substring(1), serial, errmsg))
      sendOkResponse(serial);
    else
      sendErrorResponse(serial, errmsg);
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
    //if(!line.startsWith("P")) __debugS(D, PSTR("From Prusa: '%s'"), line.c_str());
    parse_PMMU2(line.charAt(0), line.substring(1), serial, errmsg);
    setParserReady();
    return;
  }
  else {
    char tmp[256];
    sprintf_P(tmp, P_UnknownCmd, line.c_str());
    __debugS(I, PSTR("ParseGcode err: %s"), tmp);
    if(!smuffConfig.prusaMMU2) {
      sendErrorResponse(serial, tmp);
    }
  }
  setParserReady();
  currentSerial = -1;
}

bool parse_T(const String& buf, int8_t serial, char* errmsg) {
  bool stat = true;

  if(buf.length()==0) {
    // print currently selected tool only, without further action
    sendToolResponse(serial);
    return true;
  }
  if(toupper(buf[0])=='M') {
    return maintainTool(errmsg);
  }
  int8_t tool = (int8_t)buf.toInt();
  int16_t param;

  char msg[20];
  uint16_t ofs = 0;
  sprintf_P(msg, P_TResponse, tool);
  ofs = (uint16_t)String(msg).length()-2;

  if(tool == -1 || tool == 255) {
    parse_G(String("28"), serial, errmsg);
  }
  else if(tool >= 0 && tool < smuffConfig.toolCount) {
    if(!smuffConfig.prusaMMU2)  // Prusa doesn't expect "Tx" as a response
      printResponse(msg, serial);
    //__debugS(D, PSTR("Tool change requested: T%d"), tool);
    if(feederEndstop()) {
      // Prusa expects the MMU to unload filament on its own before tool change
      // Same goes if the Feeder stepper is declared as shared
      if(smuffConfig.prusaMMU2 || smuffConfig.isSharedStepper)
        //__debugS(D, PSTR("must unload first!"));
        if(!smuffConfig.useSplitter)
          unloadFilament(errmsg);
    }

    bool showMsg = SM_SHOULD_SHOW_MESSAGE(serial);
    
    if((stat = selectTool(tool, errmsg, showMsg))) {
      if(!smuffConfig.prusaMMU2) {
        if((param = getParam(buf.substring(ofs), (char*)"S")) != -1) {
          //__debugS(D, PSTR("Tx has S Param: %d"), param);
          if(param == 1) {
            stat = loadFilament(errmsg, showMsg);
            if(!stat) {
              snprintf_P(errmsg, MAX_ERR_MSG, P_CantLoad);
              strcat(errmsg,"\n");
            }
          }
          else if(param == 0) {
            stat = unloadFilament(errmsg);
            if(!stat) {
              snprintf_P(errmsg, MAX_ERR_MSG, P_CantUnload);
              strcat(errmsg,"\n");
            }
          }
        }
      }
    }
    #if defined(USE_DDE)
      switchFeederStepper(EXTERNAL);
    #else
    if(smuffConfig.isSharedStepper)
      switchFeederStepper(EXTERNAL);
    #endif
  }
  else {
    if(!smuffConfig.prusaMMU2)  // Prusa doesn't expect "Tx" as a response
      printResponse(msg, serial);
    snprintf_P(errmsg, MAX_ERR_MSG, P_WrongTool, tool);
    strcat(errmsg,"\n");
    stat = false;
  }

  return stat;
}

bool parse_Action(const String& buf, int8_t serial, char* errmsg) {

  char tmp[256];
  bool stat = true;

  if(buf.length()==0) {
    return false;
  }
  if(buf.startsWith("T:")) {
    actionOk = false;
    String msg = buf.substring(2);
    if(msg == "OK")
      actionOk = true;
    else {
      snprintf_P(tmp, ArraySize(tmp), P_ActionMsg, msg.c_str());
      drawUserMessage(tmp);
    }
  }
  else if(buf.startsWith("PING")) {
    char msg[30];
    sprintf_P(msg, P_Action, P_ActionPong);
    printResponse(msg, serial);
    __debugS(DEV3, PSTR("parse_Action: Sent action command '%s' to Serial%d"), msg, serial);
  }
  else {
    __debugS(DEV3, PSTR("parse_Action: Got unknown command '%s' from Serial%d"), buf.c_str(), serial);
    stat = false;
  }
  return stat;
}

bool parse_G(const String& buf, int8_t serial, char* errmsg) {
  bool stat = true;

  if(buf.length()==0) {
    sendGList(serial);
    return stat;
  }
  uint16_t code = (uint16_t)buf.toInt();
  //__debugS(D, PSTR("G[%s]: >%d< %d"), buf.c_str(), code, buf.length());

  char msg[10];
  char tmp[50];
  char filter[20];
  bool hasFilter = false;
  sprintf_P(msg, P_GResponse, code);
  uint8_t ofs = String(msg).length()-2;

  for(uint16_t i=0; i< 999; i++) {
    if(gCodeFuncsG[i].code == -1) {
      snprintf_P(errmsg, MAX_ERR_MSG, P_GUnknown, code);
      return false;
    }
    // deviation from standard GCode parser;
    // list help file if the first char after the GCode is a question mark (?)
    if(buf.substring(ofs).charAt(0)=='?') {
      if(ofs+1 < buf.length()) {
        snprintf(filter, ArraySize(filter), buf.substring(ofs+1).c_str());
        __debugS(DEV3, "Help filter: '*%s*'", filter);
        hasFilter = true;
      }
      snprintf(tmp, ArraySize(tmp), "g%d", code);
      listHelpFile(tmp, hasFilter ? filter : nullptr, serial);
      return true;
    }
    if(gCodeFuncsG[i].code == code) {
      memset(errmsg, 0, MAX_ERR_MSG);
      stat = gCodeFuncsG[i].func(msg, buf.substring(ofs), serial, errmsg);
      if(!stat) {
        if(*errmsg == 0)
          snprintf_P(errmsg, MAX_ERR_MSG, P_GFailed, gCodeFuncsM[i].code);
      }
      return stat;
    }
  }
  return false;
}

bool parse_M(const String& buf, int8_t serial, char* errmsg) {
  bool stat = true;

  if(buf.length()==0) {
    sendMList(serial);
    return stat;
  }
  uint16_t code = (uint16_t)buf.toInt();

  char msg[10];
  char tmp[50];
  char filter[50];
  bool hasFilter = false;
  sprintf_P(msg, P_MResponse, code);
  uint8_t ofs = String(msg).length()-2;

  for(uint16_t i=0; i< 999; i++) {
    if(gCodeFuncsM[i].code == -1) {
      snprintf_P(errmsg, MAX_ERR_MSG, P_MUnknown, code);
      return false;
    }
    if(gCodeFuncsM[i].code == code) {
      // deviation from standard GCode parser;
      // list help file if the first char after the GCode is a question mark (?)
      if(buf.substring(ofs).charAt(0)=='?') {
        if(ofs+1 < buf.length()) {
          snprintf(filter, ArraySize(filter), buf.substring(ofs+1).c_str());
          __debugS(DEV3, "Help filter: '*%s*'", filter);
          hasFilter = true;
        }
        snprintf(tmp, ArraySize(tmp), "m%d", code);
        listHelpFile(tmp, hasFilter ? filter : nullptr, serial);
        return true;
      }
      //__debugS(D, PSTR("Executing: M"), gCodeFuncsM[i].code);
      memset(errmsg, 0, MAX_ERR_MSG);
      stat = gCodeFuncsM[i].func(msg, buf.substring(ofs), serial, errmsg);
      if(!stat) {
        if(*errmsg == 0)
          snprintf_P(errmsg, MAX_ERR_MSG, P_MFailed, gCodeFuncsM[i].code);
      }
      return stat;
    }
  }
  return false;
}

/**
 *  Parse pseudo GCodes to emulate a Prusa MMU2
 **/
bool parse_PMMU2(char cmd, const String& buf, int8_t serial, char* errmsg) {

  char  tmp[80];
  char  _errmsg[MAX_ERR_MSG];
  bool  toolOk = true;
  const char* prologue = "Sent to Prusa";

  if(!smuffConfig.prusaMMU2) {
    sprintf_P(tmp, P_NoPrusa);
    sendErrorResponse(serial, tmp);
    __debugS(W, PSTR("parse_PMMU2: Prusa Emulation was not configured!"));
    return false;
  }

  bool stat = true;
  bool showMsg = isTestrun ? false : true;
  long type = -1;
  
  if(buf.length()>0)
    type = buf.toInt();
  
  switch(cmd) {
    case 'A':
      // Abort - we've already handeled that
      break;

    case 'S':     // Init (S0 | S1 | S2 | S3)
      switch(type) {
        case 0:
          sprintf_P(tmp, PSTR("ok\n"));
          break;
        case 1:
        case 2:
        case 3:
          sprintf_P(tmp, PSTR("%dok\n"), (type==1 ? PMMU_VERSION : type == 2 ? PMMU_BUILD : 0));
          break;
      }
      printResponse(tmp, serial);
      //__debugS(DEV3, ("%s (S%d): '%s'"), prologue, type, tmp);
      break;

    case 'P':     // FINDA status (Feeder endstop)
        sprintf_P(tmp, PSTR("%dok\n"), feederEndstop() ? 1 : 0);
        printResponse(tmp, serial);
        //__debugS(DEV3, PSTR("%s (P%d): '%s'"), prologue, type, tmp);
      break;

    case 'C':     // Push filament to nozzle
      if(loadFilament(errmsg, showMsg)) {
        sendOkResponse(serial);
        //__debugS(DEV3, PSTR("%s (C%d): ok<CR>"), prologue, type);
      }
      break;

    case 'L':     // Load filament
      if(toolSelected != type)
        toolOk = selectTool(type, errmsg, false);
      if(toolOk) {
        loadFilamentPMMU2(errmsg, showMsg);
        sendOkResponse(serial);
      }
      else
        sendErrorResponse(serial, errmsg);
      //__debugS(DEV3, PSTR("%s (L%d): ok<CR>"), prologue, type);
      break;

    case 'U':     // Unload filament
      if(unloadFilament(errmsg)) {
        sendOkResponse(serial);
        //__debugS(DEV3, PSTR("%s (U%d): ok<CR>"), prologue, type);
      }
      break;

    case 'E':     // Eject filament
      unloadFilament(errmsg);
      sendOkResponse(serial);
      //__debugS(DEV3, PSTR("%s (E%d): ok<CR>"), prologue, type);
      break;

    case 'K':     // Cut filament
    case 'F':     // Set filament
    case 'R':     // Recover after eject
      // ignore those commands, just send an ok response
      sendOkResponse(serial);
      //__debugS(DEV3, PSTR("%s (%c%d): ok<CR>"), prologue, cmd, type);
      break;

    case 'W': {    // Wait for user click
      userBeep();
      uint16_t button = 999;
      do {
        button = showDialog(P_PMMU_Title, P_PMMU_Wait, P_PMMU_WaitAdd, P_OkButtonOnly);
      } while (button != 1);
      sendOkResponse(serial);
      //__debugS(DEV3, PSTR("%s (W%d): ok<CR>"), prologue, type);
      break;
     }
    case 'X':     // Reset MMU
      sendOkResponse(serial);
      //__debugS(DEV3, PSTR("%s (X%d): ok<CR>"), prologue, type);
    #if !defined(SOFTRESET)
      M999("", tmp, serial, _errmsg);
    #else
      // SOFTRESET needed if some bootloader sends messages at boot.
      // This will cause an buffer overrun in Marlins MMU code.
      // Hence, don't reset, just pretend to.
      delay(1000);
      sendStartResponse(serial);
    #endif
      break;

    default:
      sendErrorResponse(serial);
      __debugS(W, PSTR("%s (%c%d): Error:...<CR>"), prologue, cmd, type);
      break;
  }
  return stat;
}

int16_t findTokenPos(String buf, const char* token) {
  bool isQuote = false;
  for(int16_t i=0; i< buf.length(); i++) {
    if(buf.charAt(i) == '\"') {
      isQuote = !isQuote;
      continue;
    }
    if(buf.charAt(i) == *token && !isQuote) {
      //__debugS(D, PSTR("Token found @: %d"), i);
      return i;
    }
  }
  return -1;
}

bool hasParam(String buf, const char* token) {
  int16_t pos = findTokenPos(buf, token);
  //__debugS(D, PSTR("hasParam: %d >%s<\n"),pos, buf.c_str());
  return pos != -1;
}

int getParam(String buf, const char* token) {
  int16_t pos = findTokenPos(buf, token);
  if(pos != -1) {
    //__debugS(D, PSTR("getParam:pos: %d"),pos);
    return atoi(buf.substring(pos+1).c_str());
  }
  else
    return -1;
}

double getParamF(String buf, const char* token) {
  int16_t pos = findTokenPos(buf, token);
  if(pos != -1) {
    //__debugS(D, PSTR("getParam:pos: %d"),pos);
    return atof(buf.substring(pos+1).c_str());
  }
  else
    return -1;
}

long getParamL(String buf, const char* token) {
  int16_t pos = findTokenPos(buf, token);
  if(pos != -1) {
    //__debugS(D, PSTR("getParam:pos: %d"),pos);
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
      //__debugS(D, PSTR("String @: %d-%d\n"), pos, endPos);
      if(endPos != -1) {
        memset(dest, 0, bufLen);
        if(endPos+1 < bufLen) {
          String tmp = buf.substring(pos+2, pos+endPos+3);
          tmp.replace("''", "\"");
          if(dest != nullptr) {
            tmp.toCharArray(dest, endPos+1);
            //__debugS(D, PSTR("ptmp: >%s< (%d)\n"), dest, strlen(dest));
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
  listHelpFile(PSTR("gcmds"), nullptr, serial);
}

void sendMList(int8_t serial) {
  listHelpFile(PSTR("mcmds"), nullptr, serial);
}

void sendM205List(int8_t serial) {
  listHelpFile(PSTR("m205"), nullptr, serial);
}

void sendToolResponse(int8_t serial) {
  char tmp[80];
  int8_t tool = getToolSelected();
  sprintf_P(tmp, P_TResponse, tool);
  printResponse(tmp, serial);
}

void sendErrorResponse(int8_t serial, const char* msg /* = nullptr */) {
  char tmp[128];
  snprintf_P(tmp, ArraySize(tmp), P_Error, msg == nullptr ? "" : msg);
  printResponse(tmp, serial);
  sendOkResponse(serial);
}

void sendBusyResponse(int8_t serial) {
  char tmp[20];
  sprintf_P(tmp, P_Echo, P_IsBusy);
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

void printResponse(const char* response) {
  sendingResponse = true;
  // send to all allowed serial ports except USB serial
  // Serial.print(response);
  if(CAN_USE_SERIAL1) Serial1.print(response);
  if(CAN_USE_SERIAL2) Serial2.print(response);
  if(CAN_USE_SERIAL3) Serial3.print(response);
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
