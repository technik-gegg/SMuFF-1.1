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

extern ZStepper steppers[NUM_STEPPERS];
char ptmp[80];
bool parserBusy = false;
unsigned int currentLine = 0;

void parseGcode(String serialBuffer, int serial) {

    if(parserBusy) {
      sendErrorResponseP(serial, P_Busy);
      return;
    }
    serialBuffer.replace(" ","");
    serialBuffer.replace("\r","");
    serialBuffer.replace("\n","");
    
    if(serialBuffer.length()==0)
      return;

    String line = String(serialBuffer);
    parserBusy = true;
    int pos;
    if((pos = line.lastIndexOf("*")) > -1) {
      line = line.substring(0, pos);
    }
    if((pos = line.lastIndexOf(";")) > -1) {
      if(pos==0) {
        parserBusy = false;
        return;
      }
      line = line.substring(0, pos);
    }
    if(line.startsWith("N")) {
      char ln[15];
      if((int)(currentLine = getParam(line, (char*)"N")) != -1) {
        sprintf(ln, "%d", currentLine);
        line = line.substring(strlen(ln)+1);
      }
    }
    //__debug("Line: %s %d", line.c_str(), line.length());
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
            line.startsWith("W")) {
      parse_PMMU2(line.charAt(0), line.substring(1), serial);
      parserBusy = false;
      return;
    }
    else {
      char tmp[256];
      sprintf(tmp, "%s '%s'\n", P_UnknownCmd, line.c_str());
      //__debug("Err: %s", tmp);
      sendErrorResponse(serial, tmp);
    }
    parserBusy = false;
}

bool parse_T(String buf, int serial) {
  bool stat = true;

  if(buf.length()==0) {
    sendToolResponse(serial);
    return stat;
  }
  int tool = buf.toInt();
  int param;

  char msg[10];
  sprintf_P(msg, P_TResponse, tool);
  int ofs = String(msg).length()-2;

  if(tool == -1 || tool == 255) {
    parse_G(String("28"), serial);
  }
  else if(tool >= 0 && tool <= smuffConfig.toolCount-1) {
    stat = selectTool(tool, false);
    if(stat) {
      if((param = getParam(buf.substring(ofs), (char*)"S")) != -1) {
        if(param == 1)
          loadFilament(false);
        else if(param == 0)
          unloadFilament();
      }
    }
    printResponse(msg, serial);
  }
  else {
    sendErrorResponse(serial, "Wrong tool selected.");
    stat = false;
  }

  return stat;
}

bool parse_G(String buf, int serial) {
  bool stat = true;

  if(buf.length()==0) {
    sendGList(serial);
    return stat;
  }
  int code = buf.toInt();
  //__debug("G[%s]: >%d< %d", buf.c_str(), code, buf.length());
 
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

bool parse_M(String buf, int serial) {
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
      //__debug("Calling: M", gCodeFuncsM[i].code);
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
bool parse_PMMU2(char cmd, String buf, int serial) {

  char  tmp[80];

  if(!smuffConfig.prusaMMU2) {
    sprintf_P(tmp, P_NoPrusa);
    sendErrorResponseP(serial, tmp);
    return false;
  }

  bool  stat = true;
  int   type = buf.toInt();
  switch(cmd) {
    case 'S':     // Init (S0 | S1 | S2 | S3)
      switch(type) {
        case 0:
          sprintf(tmp,"ok\n");
          break;
        case 1:
          sprintf(tmp,"%dok\n", PMMU_VERSION);
          break;
        case 2:
          sprintf(tmp,"%dok\n", PMMU_BUILD);
          break;
        case 3:
          sprintf(tmp,"%dok\n", 0);
          break;
      }
      printResponse(tmp,serial);
      break;

    case 'P':     // FINDA status (Feeder endstop)
        sprintf(tmp,"%dok\n", feederEndstop() ? 1 : 0);
        printResponse(tmp,serial);
      break;

    case 'C':     // Push filament to nozzle
      loadFilament(); 
      /*
      if(smuffConfig.reinforceLength > 0) {
        prepSteppingRelMillimeter(FEEDER, smuffConfig.reinforceLength, true);
        runAndWait(FEEDER);
      }
      */
      sendOkResponse(serial);
      break;

    case 'L':     // Load filament
      loadFilamentPemu();
      sendOkResponse(serial);
      break;

    case 'U':     // Unload filament
      unloadFilament();
      sendOkResponse(serial);
      break;

    case 'E':     // Eject filament
      unloadFilament();
      sendOkResponse(serial);
      break;

    case 'K':     // Cut filament
    case 'F':     // Set filament
    case 'R':     // Recover after eject
      sendOkResponse(serial);
      break;

    case 'W': {    // Wait for user click
      userBeep();
      int button = 999;
      do {
        button = showDialog(P_PMMU_Title, P_PMMU_Wait, P_PMMU_WaitAdd, P_OkButtonOnly);
      } while (button != 1);
      sendOkResponse(serial);
      break;
     }
    case 'X':     // Reset MMU
      M999("", tmp, serial);
      break;

    default:
      sendErrorResponse(serial);
      break;
  }
  return stat;
}

int getParam(String buf, char* token) {
  int pos = buf.indexOf(token);
  //__debug("getParam: %s\n",buf.c_str());
  if(pos != -1) {
    //__debug("getParam:pos: %d",pos);
    if(buf.charAt(pos+1)=='-') {
      int val = buf.substring(pos+2).toInt();
      //__debug("Negative: %d", 0-val);
      return 0-val;
    }
    return buf.substring(pos+1).toInt();
  }
  else 
    return -1; 
}

bool getParamString(String buf, char* token, char* dest, int bufLen) {
  int pos = buf.indexOf(token);
  //__debug("getParamString: %s\n",buf.c_str());
  if(pos != -1) {
    if(buf.substring(pos+1).startsWith("\"")) {
      int endPos = buf.substring(pos+2).indexOf("\"");
      //__debug("End of string: %d\n", endPos);
      if(endPos != -1) {
        memset(dest, 0, bufLen);
        if(endPos+1 < bufLen) {
          if(dest != NULL) {
            buf.substring(pos+2, endPos+3).toCharArray(dest, endPos+1);
            //__debug("ptmp: >%s< (%d)\n", dest, strlen(dest));
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
  printResponseP(P_Start, serial);
}

void saveSettings(int serial) {
  printResponseP(P_AlreadySaved, serial);
}

void reportSettings(int serial) {
  char tmp[128];
  long ldummy;
  byte bdummy;
  
  EEPROM.get(EEPROM_SELECTOR_POS, ldummy);  sprintf_P(tmp, P_SelectorPos,   EEPROM_SELECTOR_POS,    ldummy); printResponse(tmp, serial);
  EEPROM.get(EEPROM_REVOLVER_POS, ldummy);  sprintf_P(tmp, P_RevolverPos,   EEPROM_REVOLVER_POS,    ldummy); printResponse(tmp, serial);
  EEPROM.get(EEPROM_FEEDER_POS, ldummy);    sprintf_P(tmp, P_FeederPos,     EEPROM_FEEDER_POS,      ldummy); printResponse(tmp, serial);
  EEPROM.get(EEPROM_TOOL, bdummy);          sprintf_P(tmp, P_ToolSelected,  EEPROM_TOOL,            bdummy); printResponse(tmp, serial);
  EEPROM.get(EEPROM_CONTRAST, bdummy);      sprintf_P(tmp, P_Contrast,      EEPROM_CONTRAST,        bdummy); printResponse(tmp, serial);
  EEPROM.get(EEPROM_TOOL_COUNT, bdummy);    sprintf_P(tmp, P_ToolsConfig,   EEPROM_TOOL_COUNT,      bdummy); printResponse(tmp, serial);
}

void printResponse(const char* response, int serial) {
  switch(serial) {
    case 0: Serial.print(response); break;
    case 1: Serial1.print(response); break;
    case 2: Serial2.print(response); break;
    case 3: Serial3.print(response); break;
  }
}

void printResponseP(const char* response, int serial) {
  switch(serial) {
    case 0: Serial.print((__FlashStringHelper*)response); break;
    case 1: Serial1.print((__FlashStringHelper*)response); break;
    case 2: Serial2.print((__FlashStringHelper*)response); break;
    case 3: Serial3.print((__FlashStringHelper*)response); break;
  }
}
