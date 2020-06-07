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
#include "Config.h"
#include "ZTimerLib.h"
#include "ZStepperLib.h"

extern ZStepper steppers[];
char ptmp[80];
volatile bool parserBusy = false;
volatile bool sendingResponse = false;
unsigned int  currentLine = 0;
volatile bool actionOk = false;

void parseGcode(const String& serialBuffer, int serial) {

  String line;
  line.reserve(80);
  line = String(serialBuffer);
  resetSerialBuffer(serial);

  if(line.length()==0)
    return;

  if(line.startsWith("//ACTION:")) {
    line = line.substring(9);
    parse_Action(line, serial);
    return;
  }

  if(serial == 2)
    traceSerial2 = String(line);
  //__debug(PSTR("Line: %s %d"), line.c_str(), line.length());

  int pos;
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
            !line.startsWith("U") ) { // only Abort or FINDA command is being processed while parser is busy 
            //__debug(PSTR("NO valid PMMU GCode"));
      return;
    }
    //__debug(PSTR("Error: parserBusy: %d feederBusy: %d"), parserBusy, steppers[FEEDER].getMovementDone());
  }
  if(smuffConfig.prusaMMU2) {
    if(line.startsWith("T")) {
      // If a tool change command is pending while the feeder is still active,
      // request a resend of the last command, so we don't ignore and loose the command
      // as we do with the 'U' and 'A' commands.
      // The printer is supposed to send another 'T' command after a while.
      if(!steppers[FEEDER].getMovementDone()) { 
        //__debug(PSTR("Wait after 'T' 500ms")); 
        delay(500);
        if(currentLine > 0)
          sprintf_P(ptmp, PSTR("M998 %d\n"), currentLine);
        else
          sprintf_P(ptmp, PSTR("M998\n"));
        printResponseP(ptmp, serial);
        //__debug(PSTR("Resend 'T' sent"));  
        return;
      }
    }
    if(line.startsWith("U") || line.startsWith("C")) {
      if(!steppers[FEEDER].getMovementDone()) {
        sendOkResponse(serial);  
        //__debug(PSTR("Cancelling U/C"));  
        return;
      }
    }
    else if(line.startsWith("A")) {
      //__debug(PSTR("*ABORT* received"));
      if(!steppers[FEEDER].getMovementDone()) {
        setAbortRequested(true);
        //__debug(PSTR("Abort set"));
      }
      sendOkResponse(serial);  
      return;
    }
  }

  parserBusy = true;
  
  if(line.startsWith("G")) {
    if(parse_G(line.substring(1), serial))
      sendOkResponse(serial);  
    else
      sendErrorResponseP(serial);
    parserBusy = false;
    return;
  }
  else if(line.startsWith("M")) {
    if(parse_M(line.substring(1), serial))
      sendOkResponse(serial);  
    else
      sendErrorResponseP(serial);
    parserBusy = false;
    return;
  }
  else if(line.startsWith("T")) {
    if(parse_T(line.substring(1), serial))
      sendOkResponse(serial);  
    else
      sendErrorResponseP(serial);  
    parserBusy = false;
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
    //if(!line.startsWith("P")) __debug(PSTR("From Prusa: '%s'"), line.c_str());
    parse_PMMU2(line.charAt(0), line.substring(1), serial);
    parserBusy = false;
    return;
  }
  else {
    char tmp[256];
    sprintf_P(tmp, P_UnknownCmd, line.c_str());
    __debug(PSTR("ParseGcode err: %s"), tmp);
    if(!smuffConfig.prusaMMU2) {
      sendErrorResponseP(serial, tmp);
    }
  }
  parserBusy = false;
}

bool parse_T(const String& buf, int serial) {
  bool stat = true;

  if(buf.length()==0) {
    sendToolResponse(serial);
    return stat;
  }
  if(toUpperCase(buf[0])=='M') {
    maintainTool();
    return true;
  }
  int tool = buf.toInt();
  int param;

  char msg[20];
  int ofs = 0;
  sprintf_P(msg, P_TResponse, tool);
  ofs = String(msg).length()-2;

  if(tool == -1 || tool == 255) {
    parse_G(String("28"), serial);
  }
  else if(tool >= 0 && tool <= smuffConfig.toolCount-1) {
    //__debug(PSTR("Tool change requested: T%d"), tool);
    // Prusa expects the MMU to unload filament on its own before tool change
    if(smuffConfig.prusaMMU2 && feederEndstop()) { 
      //__debug(PSTR("must unload first!"));
      unloadFilament();
    }
    stat = selectTool(tool, false);
    if(stat) {
      if(!smuffConfig.prusaMMU2) {
        if((param = getParam(buf.substring(ofs), (char*)"S")) != -1) {
          if(param == 1)
            loadFilament(false);
          else if(param == 0)
            unloadFilament();
        }
      }
    }
    if(!smuffConfig.prusaMMU2)  // Prusa expects not Tx as response
      printResponse(msg, serial);
  }
  else {
    sendErrorResponseP(serial, PSTR("Wrong tool selected."));
    stat = false;
  }

  return stat;
}

bool parse_Action(const String& buf, int serial) {

  if(buf.length()==0) {
    return false;
  }
  actionOk = false;
  if(buf.startsWith("T:")) {
    String msg = buf.substring(2);
    if(msg == "OK")
      actionOk = true;
    else {
      drawUserMessage(msg);
    }
  }
  return false;
}

bool parse_G(const String& buf, int serial) {
  bool stat = true;

  if(buf.length()==0) {
    sendGList(serial);
    return stat;
  }
  int code = buf.toInt();
  //__debug(PSTR("G[%s]: >%d< %d"), buf.c_str(), code, buf.length());
 
  char msg[10];
  sprintf_P(msg, P_GResponse, code);
  int ofs = String(msg).length()-2;

  for(int i=0; i< 999; i++) {
    if(gCodeFuncsG[i].code == -1)
      break;
    if(gCodeFuncsG[i].code == code) {
      return gCodeFuncsG[i].func(msg, buf.substring(ofs), serial);
    }
  }
  char tmp[256];
  sprintf_P(tmp, P_UnknownCmd, buf.c_str());
  return false;
}

bool parse_M(const String& buf, int serial) {
  bool stat = true;
  
  if(buf.length()==0) {
    sendMList(serial);
    return stat;
  }
  int code = buf.toInt();
 
  char msg[10];
  sprintf_P(msg, P_MResponse, code);
  int ofs = String(msg).length()-2;

  for(int i=0; i< 999; i++) {
    if(gCodeFuncsM[i].code == -1)
      break;
    if(gCodeFuncsM[i].code == code) {
      //__debug(PSTR("Calling: M"), gCodeFuncsM[i].code);
      return gCodeFuncsM[i].func(msg, buf.substring(ofs), serial);
    }
  }
  char tmp[256];
  sprintf_P(tmp, P_UnknownCmd, buf.c_str());
  return false;
}

/**
 *  Parse pseudo GCodes to emulate a Prusa MMU2 
 **/
bool parse_PMMU2(char cmd, const String& buf, int serial) {

  char  tmp[80];
  bool toolOk = true;

  if(!smuffConfig.prusaMMU2) {
    sprintf_P(tmp, P_NoPrusa);
    sendErrorResponseP(serial, tmp);
    //__debug(PSTR("No Prusa Emulation configured!"));
    return false;
  }

  bool  stat = true;
  int   type = -1;
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
      //__debug(("To Prusa (S%d): '%s'"), type, tmp);
      break;

    case 'P':     // FINDA status (Feeder endstop)
        sprintf_P(tmp,PSTR("%dok\n"), feederEndstop() ? 1 : 0);
        printResponse(tmp,serial);
        //__debug(PSTR("To Prusa (P%d): '%s'"), type, tmp);
      break;

    case 'C':     // Push filament to nozzle
      if(loadFilament()) {
        sendOkResponse(serial);
        //__debug(PSTR("To Prusa (C%d): ok<CR>"), type);
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
      //__debug(PSTR("To Prusa (L%d): ok<CR>"), type);
      break;

    case 'U':     // Unload filament
      if(unloadFilament()) {
        sendOkResponse(serial);
        //__debug(PSTR("To Prusa (U%d): ok<CR>"), type);
      }
      break;

    case 'E':     // Eject filament
      unloadFilament();
      sendOkResponse(serial);
      //__debug(PSTR("To Prusa (E%d): ok<CR>"), type);
      break;

    case 'K':     // Cut filament
    case 'F':     // Set filament
    case 'R':     // Recover after eject
      sendOkResponse(serial);
      //__debug(PSTR("To Prusa (%c%d): ok<CR>"), cmd, type);
      break;

    case 'W': {    // Wait for user click
      userBeep();
      int button = 999;
      do {
        button = showDialog(P_PMMU_Title, P_PMMU_Wait, P_PMMU_WaitAdd, P_OkButtonOnly);
      } while (button != 1);
      sendOkResponse(serial);
      //__debug(PSTR("To Prusa (W%d): ok<CR>"), type);
      break;
     }
    case 'X':     // Reset MMU
      sendOkResponse(serial);
      //__debug(PSTR("To Prusa (X%d): ok<CR>"), type);
      M999("", tmp, serial);
      break;

    default:
      sendErrorResponseP(serial);
      //__debug(PSTR("To Prusa (%c%d): Error:...<CR>"), cmd, type);
      break;
  }
  return stat;
}

int hasParam(String buf, char* token) {
  int pos = buf.indexOf(token);
  //__debug(PSTR("hasParam: %s\n"),buf.c_str());
  return pos;
}

int getParam(String buf, char* token) {
  int pos = buf.indexOf(token);
  //__debug(PSTR("getParam: %s\n"),buf.c_str());
  if(pos != -1) {
    //__debug(PSTR("getParam:pos: %d"),pos);
    if(buf.charAt(pos+1)=='-') {
      int val = buf.substring(pos+2).toInt();
      //__debug(PSTR("Negative: %d"), 0-val);
      return 0-val;
    }
    return buf.substring(pos+1).toInt();
  }
  else 
    return -1; 
}

long getParamL(String buf, char* token) {
  int pos = buf.indexOf(token);
  //__debug(PSTR("getParam: %s\n"),buf.c_str());
  if(pos != -1) {
    //__debug(PSTR("getParam:pos: %d"),pos);
    if(buf.charAt(pos+1)=='-') {
      long val = buf.substring(pos+2).toInt();
      //__debug(PSTR("Negative: %d"), 0-val);
      return 0-val;
    }
    return buf.substring(pos+1).toInt();
  }
  else 
    return -1; 
}

bool getParamString(String buf, char* token, char* dest, int bufLen) {
  int pos = buf.indexOf(token);
  //__debug(PSTR("getParamString: %s\n"),buf.c_str());
  if(pos != -1) {
    if(buf.substring(pos+1).startsWith("\"")) {
      int endPos = buf.substring(pos+2).indexOf("\"");
      //__debug(PSTR("End of string: %d\n"), endPos);
      if(endPos != -1) {
        memset(dest, 0, bufLen);
        if(endPos+1 < bufLen) {
          if(dest != NULL) {
            buf.substring(pos+2, endPos+3).toCharArray(dest, endPos+1);
            //__debug(PSTR("ptmp: >%s< (%d)\n"), dest, strlen(dest));
          }
          return true;
        }
      }
    }
  }
  return false;
}

void prepStepping(int index, long param, bool Millimeter /* = true */, bool ignoreEndstop /* = false */) {
  if(param != 0) {
    if(positionMode == RELATIVE) {
      if(Millimeter) prepSteppingRelMillimeter(index, param);
      else prepSteppingRel(index, param);
    }
    else {
      if(Millimeter) prepSteppingAbsMillimeter(index, param);
      else prepSteppingAbs(index, param);
    }
    remainingSteppersFlag |= _BV(index);
  }
}

void sendGList(int serial) {
  printResponseP(P_GCmds, serial);
}

void sendMList(int serial) {
  printResponseP(P_MCmds, serial);
}

void sendToolResponse(int serial) { 
  char tmp[80];
  sprintf_P(tmp, P_TResponse, toolSelected);
  printResponse(tmp, serial);
}

void sendErrorResponse(int serial, const char* msg /* = NULL */) {
  if(msg != NULL)
    printResponse(msg, serial);
  sendOkResponse(serial);
}

void sendErrorResponseP(int serial, const char* msg /* = NULL */) {
  char tmp[128];
  sprintf_P(tmp, P_Error, msg == NULL ? "" : msg);
  printResponse(tmp, serial);
  sendOkResponse(serial);
}

void sendOkResponse(int serial) {
  printResponseP(P_Ok, serial);
}

void sendStartResponse(int serial){
  char tmp[50];
  char tmp1[15];
  if(!smuffConfig.prusaMMU2) {
    sprintf_P(tmp1, PSTR("Serial: %d"), serial);
    sprintf_P(tmp, P_Echo, tmp1);
    printResponse(tmp, serial);
  }
  printResponseP(P_Start, serial);
}

void saveSettings(int serial) {
  printResponseP(P_AlreadySaved, serial);
}

void reportSettings(int serial) {
  char tmp[128];
  
  sprintf_P(tmp, P_SelectorPos,   dataStore.stepperPos[SELECTOR]); printResponse(tmp, serial);
  sprintf_P(tmp, P_RevolverPos,   dataStore.stepperPos[REVOLVER]); printResponse(tmp, serial);
  sprintf_P(tmp, P_FeederPos,     dataStore.stepperPos[FEEDER]); printResponse(tmp, serial);
  sprintf_P(tmp, P_ToolSelected,  dataStore.tool); printResponse(tmp, serial);
  sprintf_P(tmp, P_Contrast,      smuffConfig.lcdContrast); printResponse(tmp, serial);
  sprintf_P(tmp, P_ToolsConfig,   smuffConfig.toolCount); printResponse(tmp, serial);
}

void printResponse(const char* response, int serial) {
  sendingResponse = true;
  switch(serial) {
    case 0: Serial.print(response); break;
    case 1: Serial1.print(response); break;
    case 2: Serial2.print(response); break;
#ifndef __ESP32__
    case 3: Serial3.print(response); break;
#endif
  }
  sendingResponse = false;
}

void printResponseP(const char* response, int serial) {
  sendingResponse = true;
  switch(serial) {
    case 0: Serial.print((__FlashStringHelper*)response); break;
    case 1: Serial1.print((__FlashStringHelper*)response); break;
    case 2: Serial2.print((__FlashStringHelper*)response); break;
#ifndef __ESP32__
    case 3: Serial3.print((__FlashStringHelper*)response); break;
#endif
  }
  sendingResponse = false;
}
