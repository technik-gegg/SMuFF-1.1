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
#include "ZTimerLib.h"
#include "ZStepperLib.h"
#include "ZServo.h"
#include "ZPortExpander.h"
#include "GCodes.h"
#ifdef __STM32F1__
#include "libmaple/nvic.h"
#endif
#ifdef __ESP32__
#include "Tone32.h"
#endif

extern ZStepper       steppers[];
extern ZServo         servo;
#if defined(__ESP32__)
extern ZPortExpander  portEx;
#endif

char* S_Param = (char*)"S";
char* P_Param = (char*)"P";
char* X_Param = (char*)"X";
char* Y_Param = (char*)"Y";
char* Z_Param = (char*)"Z";
char* E_Param = (char*)"E";
char* F_Param = (char*)"F";
char* C_Param = (char*)"C";
char* T_Param = (char*)"T";
char* M_Param = (char*)"M";
char* N_Param = (char*)"N";
char* I_Param = (char*)"I";
char* J_Param = (char*)"J";
char* K_Param = (char*)"K";
char* R_Param = (char*)"R";

GCodeFunctions gCodeFuncsM[] = {
  {   0, dummy },     // used in Prusa Emulation mode to switch to normal mode
  {   1, dummy },     // used in Prusa Emulation mode to switch to stealth mode
  {  80, dummy },
  {  81, dummy },
  { 104, dummy },
  { 105, dummy },
  { 108, dummy },
  { 109, dummy },
  { 220, dummy },
  { 221, dummy },

  {  18, M18 },
  {  20, M20 },
  {  42, M42 },
  {  84, M18 },
  {  98, M98 },
  { 106, M106 },
  { 107, M107 }, 
  { 110, M110 },
  { 111, M111 },
  { 114, M114 },
  { 115, M115 },
  { 117, M117 },
  { 119, M119 },
  { 201, M201 },
  { 203, M203 },
  { 205, M205 },
  { 206, M206 },
  { 250, M250 },
  { 280, M280 },
  { 300, M300 },
  { 500, M500 },
  { 503, M503 },
  { 575, M575 },
  { 700, M700 },
  { 701, M701 },
  { 999, M999 },
  { 2000, M2000 },
  { 2001, M2001 },
  { -1, NULL }
};

GCodeFunctions gCodeFuncsG[] = {
  {   0, G0 },
  {   1, G1 },
  {   4, G4 },
  {  12, G12 },
  {  28, G28 },
  {  90, G90 },
  {  91, G91 },
  { -1, NULL},
};

int param;
char tmp[256];

/*========================================================
 * Class M
 ========================================================*/
bool dummy(const char* msg, String buf, int serial) {
  if(!smuffConfig.prusaMMU2) {
    int code = buf.toInt();
    __debug(PSTR("Ignored M-Code: M%d"), code);
  }
  return true;
}

bool M18(const char* msg, String buf, int serial) {
  bool stat = true;
  printResponse(msg, serial); 
  if(buf.length()==0) {
    steppers[SELECTOR].setEnabled(false);
    steppers[REVOLVER].setEnabled(false);
    steppers[FEEDER].setEnabled(false);
  }
  else {
    if(buf.indexOf(X_Param) != -1) {
      steppers[SELECTOR].setEnabled(false);
    }
    else if(buf.indexOf(Y_Param) != -1) {
      steppers[REVOLVER].setEnabled(false);
    }
    else if(buf.indexOf(Z_Param) != -1) {
      steppers[FEEDER].setEnabled(false);
    }
    else {
      stat = false;
    }
  }
  return stat;
}

bool M20(const char* msg, String buf, int serial) {
  
  if(!getParamString(buf, S_Param, tmp, sizeof(tmp))){
    sprintf(tmp,"/");
  }
  SdFs SD;
  Print* out = &Serial;
#if defined(__ESP32__)
  if (SD.begin(SDCS_PIN, SD_SCK_MHZ(4))) {
#else
  if (SD.begin()) {
#endif
    switch(serial) {
      case 0: out = &Serial; break;
      case 1: out = &Serial1; break;
      case 2: out = &Serial2; break;
      default: break;
    }
    SD.ls(out, LS_DATE | LS_SIZE | LS_R);
  }
  else {
    sprintf_P(tmp, P_SD_InitError);
    printResponse(tmp, serial); 
    return false;
  }
  return true;
}

bool M42(const char* msg, String buf, int serial) {
  bool stat = true;
  int pin;
  int mode;
  printResponse(msg, serial); 
  //__debug(PSTR("M42->%s"), buf);
  
  if((pin = getParam(buf, P_Param)) != -1) {
    // pins over 1000 go to the port expander
    if(pin >= 1000) {
      #if defined(__ESP32__)
      pin -= 1000;
      if((mode = getParam(buf, M_Param)) != -1) {
         // 0=INPUT, 1=OUTPUT, 2=INPUT_PULLUP, 3=INPUT_PULLDOWN
         switch(mode) {
          case 0: portEx.pinMode(pin, INPUT); break;
          case 1: portEx.pinMode(pin, OUTPUT); break;
          case 2: portEx.pinMode(pin, INPUT_PULLUP); break;
          case 3: portEx.pinMode(pin, INPUT_PULLDOWN); break;
         }
      }
      else {
        mode = 1;
        portEx.pinMode(pin, OUTPUT);
      }
      if((param = getParam(buf, S_Param)) != -1 && mode == 1) {
        //__debug(PSTR("Pin%d set to %s"), pin, param==0 ? "LOW" : "HIGH");
        portEx.writePin(pin, param);
      }
      if(mode != 1) {
        int state = portEx.readPin(pin);
        sprintf(tmp, "echo: P%d: %s\n", pin+1000, state==0 ? "LOW" : "HIGH");
        printResponse(tmp, serial);
      }
      #endif
    }
    else {
      if((mode = getParam(buf, M_Param)) != -1) {
         // 0=INPUT, 1=OUTPUT, 2=INPUT_PULLUP, 3=INPUT_PULLDOWN
         switch(mode) {
          case 0: pinMode(pin, INPUT); break;
          case 1: pinMode(pin, OUTPUT); break;
          case 2: pinMode(pin, INPUT_PULLUP); break;
          case 3: 
            #if defined(__AVR__)
            pinMode(pin, INPUT);
            #else
            pinMode(pin, INPUT_PULLDOWN); 
            #endif
            break;
         }
      }
      else {
        mode = 1;
        pinMode(pin, OUTPUT);
      }
      if((param = getParam(buf, S_Param)) != -1 && mode == 1) {
        if(param >= 0 && param <= 255) {
          #ifdef __STM32F1__
            pwmWrite(pin, param);
          #elif __ESP32__
            ledcWrite(pin, param);
          #else
            analogWrite(pin, param);
          #endif
        }
      }
      if(mode != 1) {
        int state = digitalRead(pin);
        sprintf(tmp, "echo: P%d: %d\n", pin, state);
        printResponse(tmp, serial);
      }
    }
  }
  return stat;
}

bool M98(const char* msg, String buf, int serial) {
  printResponse(msg, serial);
  char cmd[80];
  if((getParamString(buf, P_Param, cmd, sizeof(cmd)))) {
    showMenu = true;
    testRun(String(cmd));
    showMenu = false;
  }
  return true;
}

bool M106(const char* msg, String buf, int serial) {
  printResponse(msg, serial); 
  if((param = getParam(buf, S_Param)) == -1) {
    param = 100;
  }
  __debug(PSTR("Fan speed: %d%%"), param);
#ifdef __STM32F1__
  pwmWrite(FAN_PIN, map(param, 0, 100, 0, 65535));
#elif __ESP32__
  ledcWrite(FAN_PIN, map(param, 0, 100, 0, 255));
#else
  analogWrite(FAN_PIN, map(param, 0, 100, 0, 255));
#endif
  return true;
}
 
bool M107(const char* msg, String buf, int serial) {
  printResponse(msg, serial); 
#ifdef __STM32F1__
  pwmWrite(FAN_PIN, 0);
#elif __ESP32__
  ledcWrite(FAN_PIN, 0);
#else
  analogWrite(FAN_PIN, 0);
#endif
  return true;
}

bool M110(const char* msg, String buf, int serial) {
  printResponse(msg, serial); 
  if((param = getParam(buf, N_Param)) != -1) {
    currentLine = param;
  }
  return true;
}

bool M111(const char* msg, String buf, int serial) {
  if((param = getParam(buf, S_Param)) != -1) {
    testMode = param == 1;
  }      
  return true;
}

bool M114(const char* msg, String buf, int serial) {
  printResponse(msg, serial); 
  sprintf_P(tmp, P_Positions, 
  String(steppers[SELECTOR].getStepPositionMM()).c_str(),
  String(steppers[REVOLVER].getStepPosition()).c_str(),
  String(steppers[FEEDER].getStepPositionMM()).c_str());
  printResponse(tmp, serial); 
  return true;
}

bool M115(const char* msg, String buf, int serial) {
  sprintf_P(tmp, P_GVersion, VERSION_STRING, BOARD_INFO, VERSION_DATE, smuffConfig.prusaMMU2 ? "PMMU" : "Duet");
  printResponse(tmp, serial); 
  return true;
}

bool M117(const char* msg, String buf, int serial) {
  String umsg = buf;
  umsg.replace("_", " ");
  beep(1);
  drawUserMessage(umsg);
  return true;
}

bool M119(const char* msg, String buf, int serial) {
  printResponse(msg, serial); 
  if((param = getParam(buf, Z_Param)) != -1) {
    steppers[FEEDER].setEndstopHit(param);
  }
  printEndstopState(serial); 
  return true;
}

bool M201(const char* msg, String buf, int serial) {
  bool stat = true;
  printResponse(msg, serial); 
  if(buf.length()==0) {
    printAcceleration(serial);
    return stat;
  }
  if((param = getParam(buf, X_Param))  != -1) {
    if(param >= 200 && param <= 65000) {
      steppers[SELECTOR].setAcceleration(param);
      smuffConfig.acceleration_X = param;
    }
    else stat = false;
  }
  if((param = getParam(buf, Y_Param))  != -1) {
    if(param >= 200 && param <= 65000) {
      steppers[REVOLVER].setAcceleration(param);
      smuffConfig.acceleration_Y = param;
    }
    else stat = false;
  }
  if((param = getParam(buf, Z_Param))  != -1) {
    if(param >= 200 && param <= 65000) {
      steppers[FEEDER].setAcceleration(param);
      smuffConfig.acceleration_Z = param;
      if(smuffConfig.insertSpeed_Z > smuffConfig.acceleration_Z)
        smuffConfig.acceleration_Z = smuffConfig.insertSpeed_Z;
    }
    else stat = false;
  }
  return stat;
}

bool M203(const char* msg, String buf, int serial) {
  bool stat = true;
  printResponse(msg, serial); 
  if(buf.length()==0) {
    printSpeeds(serial);
    return stat;
  }
  if((param = getParam(buf, X_Param))  != -1) {
    if(param > 0 && param <= 65000) {
      steppers[SELECTOR].setMaxSpeed(param);
      smuffConfig.maxSpeed_X = param;
    }
    else stat = false;
    if((param = getParam(buf, S_Param))  != -1) {
      smuffConfig.stepDelay_X = (int)param;
    }
    if((param = getParam(buf, P_Param))  != -1) {
      steppers[SELECTOR].setMaxHSpeed(param);
      smuffConfig.maxSpeedHS_X = param;
    }
  }
  if((param = getParam(buf, Y_Param))  != -1) {
    if(param > 0 && param <= 65000) {
      steppers[REVOLVER].setMaxSpeed(param);
      smuffConfig.maxSpeed_Y = param;
    }
    else stat = false;
    if((param = getParam(buf, S_Param))  != -1) {
      smuffConfig.stepDelay_Y = (int)param;
    }
    if((param = getParam(buf, P_Param))  != -1) {
      steppers[REVOLVER].setMaxHSpeed(param);
      smuffConfig.maxSpeedHS_Y = param;
    }
  }
  if((param = getParam(buf, Z_Param))  != -1) {
    if(param > 0 && param <= 65000) {
      steppers[FEEDER].setMaxSpeed(param);
      smuffConfig.maxSpeed_Z = param;
    }
    else stat = false;
    if((param = getParam(buf, S_Param))  != -1) {
      smuffConfig.stepDelay_Z = (int)param;
    }
    if((param = getParam(buf, P_Param))  != -1) {
      steppers[FEEDER].setMaxHSpeed(param);
      smuffConfig.maxSpeedHS_Z = param;
    }
    if((param = getParam(buf, F_Param))  != -1) {
      smuffConfig.insertSpeed_Z = param;
      if(smuffConfig.insertSpeed_Z > smuffConfig.acceleration_Z)
        smuffConfig.acceleration_Z = smuffConfig.insertSpeed_Z;
    }
  }
  return stat;
}

bool M205(const char* msg, String buf, int serial) {
  bool stat = true;
  printResponse(msg, serial); 
  if(buf.length()==0) {
    //printAdvancedSettings(serial);
    return stat;
  }
  char cmd[80];
  if((param = getParamString(buf, P_Param, cmd, sizeof(cmd)))  != -1) {
    if((param = getParam(buf, S_Param)) != -1) {
      if(strcmp_P(cmd, PSTR("BowdenLength"))==0) {
        smuffConfig.bowdenLength = param;
      }
      else if(strcmp_P(cmd, PSTR("InsertLength"))==0) {
        smuffConfig.insertLength = param;
      }
      else if(strcmp_P(cmd, PSTR("ReinforceLength"))==0) {
        smuffConfig.reinforceLength = param;
      }
      else if(strcmp_P(cmd, PSTR("SelectorDist"))==0) {
        smuffConfig.selectorDistance = param;
      }
      else if(strcmp_P(cmd, PSTR("HomeAfterFeed"))==0) {
        smuffConfig.homeAfterFeed = (param > 0);
      }
      else if(strcmp_P(cmd, PSTR("ResetBeforeFeed"))==0) {
        smuffConfig.resetBeforeFeed_Y = (param > 0);
      }
      else if(strcmp_P(cmd, PSTR("EmulatePrusa"))==0) {
        smuffConfig.prusaMMU2 = (param > 0);
      }
      else if(strcmp_P(cmd, PSTR("UseServo"))==0) {
        smuffConfig.revolverIsServo = (param > 0);
      }
      else if(strcmp_P(cmd, PSTR("ServoOpened"))==0) {
        smuffConfig.revolverOffPos = param;
      }
      else if(strcmp_P(cmd, PSTR("ServoClosed"))==0) {
        smuffConfig.revolverOnPos = param;
      }
    }
  }
  return stat;
}

bool M206(const char* msg, String buf, int serial) {
  bool stat = true;
  printResponse(msg, serial); 
  if(buf.length()==0) {
    printOffsets(serial);
    return stat;
  }
  if((param = getParam(buf, X_Param))  != -1) {
    if(param > 0 && param <= 10000)
      smuffConfig.firstToolOffset = (float)param/10;
    else stat = false;
  }
  if((param = getParam(buf, Y_Param))  != -1) {
    if(param > 0 && param <= 8640) {
      smuffConfig.firstRevolverOffset = param;
    }
    else stat = false;
  }
  return stat;
}

bool M250(const char* msg, String buf, int serial) {
  bool stat = true;
  if((param = getParam(buf, C_Param)) != -1) {
    if(param >= 60 && param < 256) {
      display.setContrast(param);
      smuffConfig.lcdContrast = param;
      printResponse(msg, serial); 
    }
    else
      stat = false;
  }
  else {
      printResponse(msg, serial);
      char tmp[50];
      sprintf_P(tmp, P_M250Response, smuffConfig.lcdContrast);
      printResponse(tmp, serial); 
  }
  return stat;
}

bool M280(const char* msg, String buf, int serial) {
  bool stat = true;
  int servoIndex = 0;
  printResponse(msg, serial);
  if((param = getParam(buf, I_Param)) != -1) {
    smuffConfig.servoMinPwm = param;
    setServoMinPwm(servoIndex, param);
  }
  if((param = getParam(buf, J_Param)) != -1) {
    smuffConfig.servoMaxPwm = param;
    setServoMaxPwm(servoIndex, param);
  }
  if((param = getParam(buf, P_Param)) != -1) {
    servoIndex = param;
  }
  if((param = getParam(buf, S_Param)) != -1) {
    if(!setServoPos(servoIndex, param))
      stat = false;
  }
  else if((param = getParam(buf, F_Param)) != -1) {
    if(!setServoMS(servoIndex, param))
      stat = false;
  }
  else stat = false;
  if((param = getParam(buf, R_Param)) != -1) {
    setServoPos(servoIndex, param == 1 ? smuffConfig.revolverOnPos : smuffConfig.revolverOffPos);
    stat = true;
  }
  if((param = getParam(buf, T_Param)) != -1) {
    for(int i=10; i <= 170; i += 10) {
      setServoPos(servoIndex, i);
      sprintf(tmp,"Servo pos.: %d deg\n", i);
      printResponse(tmp, serial);
      delay(900);
      stat = true;
    }
  }
  delay(50);
  return stat;
}

bool M300(const char* msg, String buf, int serial) {
  bool stat = true;
  printResponse(msg, serial);
  if((param = getParam(buf, S_Param)) != -1) {
    int frequency = param;
    if((param = getParam(buf, P_Param)) != -1) {
      tone(BEEPER_PIN, frequency, param);
    }
    else 
      stat = false;
  }
  else 
    stat = false;
  return stat;
}

bool M500(const char* msg, String buf, int serial) {
  printResponse(msg, serial);
  return writeConfig();
}

bool M503(const char* msg, String buf, int serial) {
  printResponse(msg, serial);
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
      default:
        break;
    }
  return writeConfig(_print);
}

bool M575(const char* msg, String buf, int serial) {
  bool stat = true;
  long paramL;
  int port = -1;
  printResponse(msg, serial);
  if((param = getParam(buf, P_Param)) != -1) {
    port = param;
  }
  if((paramL = getParamL(buf, S_Param)) != -1) {
    if(port != -1) {
      switch(port) {
        case 1: smuffConfig.serial1Baudrate = paramL; break;
        case 2: smuffConfig.serial2Baudrate = paramL; break;
      }
    }
    else {
      smuffConfig.serial1Baudrate = paramL;
      smuffConfig.serial2Baudrate = paramL;
    }
  }
  else 
    stat = false;
  return stat;
}

bool M700(const char* msg, String buf, int serial) {
  bool stat = true;
  printResponse(msg, serial);
  if(toolSelected > 0 && toolSelected <= MAX_TOOLS) {
    getParamString(buf, S_Param, smuffConfig.materials[toolSelected], sizeof(smuffConfig.materials[0]));
    //__debug(PSTR("Material: %s\n"),smuffConfig.materials[toolSelected]);
    return loadFilament();
  }
  else 
    stat = false;
  return stat;
}

bool M701(const char* msg, String buf, int serial) {
  printResponse(msg, serial);
  return unloadFilament();
}

bool M999(const char* msg, String buf, int serial) {
  printResponse(msg, serial); 
  delay(500); 
#ifdef __AVR__
  __asm__ volatile ("jmp 0x0000"); 
#elif __STM32F1__
  nvic_sys_reset();
#elif __ESP32__
  ESP.restart();
#endif
  return true;
}

bool M2000(const char* msg, String buf, int serial) {
  char s[80];
  printResponse(msg, serial); 
  getParamString(buf, S_Param, tmp, sizeof(tmp));
  if(strlen(tmp)>0) {
    printResponseP(PSTR("B"), serial);
    for(unsigned i=0; i< strlen(tmp); i++) {
      sprintf(s,"%d:", (char)tmp[i]);
      printResponse(s, serial); 
    }
    printResponseP(PSTR("10\n"), serial);
  }
  return true;
}

bool M2001(const char* msg, String buf, int serial) {
  printResponse(msg, serial); 
  getParamString(buf, S_Param, tmp, sizeof(tmp));
  String data = String(tmp);
  data.trim();
  if(data.length() > 0) {
    int ndx = 0;
    int pos = 0;
    if(data.startsWith("B")) {
      printResponseP(PSTR(">>"), serial); 
      ndx++;
      do {
        pos = data.indexOf(":", ndx);
        int c;
        if(pos != -1) {
          c = data.substring(ndx, pos).toInt();
        }
        else {
          c = data.substring(ndx).toInt();
        }
        if(c == 10) {
          printResponseP(PSTR("\\n"), serial); 
        }
        else {
          sprintf(tmp, "%c", c);
          printResponse(tmp, serial); 
        }
        ndx = pos + 1;
      } while(pos != -1);
      printResponseP(PSTR("<<\n"), serial); 
    }
    else {
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
bool G0(const char* msg, String buf, int serial) {
  printResponse(msg, serial);
  if((param = getParam(buf, Y_Param)) != -1) {
    steppers[REVOLVER].setEnabled(true);
    if(getParam(buf, S_Param)) {
      // for testing only 
      toolSelected = param;
      positionRevolver();
    }
    else {
      prepSteppingAbs(REVOLVER, smuffConfig.firstRevolverOffset + ((param)*smuffConfig.revolverSpacing), true);
      runAndWait(REVOLVER);
    }
  }
  if((param = getParam(buf, X_Param)) != -1) {
    steppers[SELECTOR].setEnabled(true);
    prepSteppingAbsMillimeter(SELECTOR, smuffConfig.firstToolOffset + (param * smuffConfig.toolSpacing));
    runAndWait(SELECTOR);
  }
  return true;
}

bool G1(const char* msg, String buf, int serial) {
  printResponse(msg, serial);
  bool isMill = true;
  if((param = getParam(buf, T_Param)) != -1) {
    isMill = (param == 1);
  }
  if((param = getParam(buf, Y_Param)) != -1) {
    //__debug(PSTR("G1 moving Y: %d %S"), param, isMill ? PSTR("mm") : PSTR("steps"));
    steppers[REVOLVER].setEnabled(true);
    prepStepping(REVOLVER, param, isMill);
  }
  if((param = getParam(buf, X_Param)) != -1) {
    //__debug(PSTR("G1 moving X: %d %S"), param, isMill ? PSTR("mm") : PSTR("steps"));
    steppers[SELECTOR].setEnabled(true);
    prepStepping(SELECTOR, param, isMill, true);
  }
  if((param = getParam(buf, Z_Param)) != -1) {
    //__debug(PSTR("G1 moving Z: %d %S"), param, isMill ? PSTR("mm") : PSTR("steps"));
    steppers[FEEDER].setEnabled(true);
    prepStepping(FEEDER, param, isMill);
  }
  runAndWait(-1);
  return true;
}

bool G4(const char* msg, String buf, int serial) {
  bool stat = true;
  printResponse(msg, serial);
  if((param = getParam(buf, S_Param)) != -1) {
    if(param > 0 && param < 500)
      delay(param*1000);
  }
  else if((param = getParam(buf, P_Param)) != -1) {
      delay(param);
  }
  else {
    stat = false;
  }
  return stat;
}

bool G12(const char* msg, String buf, int serial) {
  printResponse(msg, serial);
  int wait = 500;
  int pos1 = 20;
  int pos2 = 45;
  int pos0 = 110;
  unsigned repeat = 10;
  String p;
  String seq = String(smuffConfig.wipeSequence);

  if(buf.length()==0 && seq.length()==0)
    return false;
  // sent sequence overrides internal sequence
  if(buf.length()==0)
    p = seq;
  else
    p = buf;

  if((param = getParam(p, S_Param)) != -1) {  
    wait = param; // defines the wipe speed
  }
  if((param = getParam(p, I_Param)) != -1) {
    pos1 = param; // defines position 1 when wiping
  }
  if((param = getParam(p, J_Param)) != -1) {
    pos2 = param; // defines position 2 when wiping
  }
  if((param = getParam(p, P_Param)) != -1) {
    pos0 = param; // defines release position
  }
  if((param = getParam(p, R_Param)) != -1) {
    repeat = param; // defines the number of repeats
  }
  unsigned n = 1;
  for(n=0; n < repeat; n++) {
    servo.write(pos1);
    delay(wait);
    servo.write(pos2);
    delay(wait);
  }
  servo.write(pos0);
  delay(100);
  return true;
}

bool G28(const char* msg, String buf, int serial) {
  bool stat = true;
  printResponse(msg, serial);
  if(buf.length()==0) {
    stat = moveHome(SELECTOR, false, true); 
    if(stat)
      moveHome(REVOLVER, false, false); 
  }
  else {
    if(buf.indexOf(X_Param) != -1) {
      stat = moveHome(SELECTOR, false, false); 
    }
    if(buf.indexOf(Y_Param) != -1) {
      stat = moveHome(REVOLVER, false, false); 
    }
  }
  return stat;
}

bool G90(const char* msg, String buf, int serial) {
  printResponse(msg, serial);
  positionMode = ABSOLUTE;
  return true;
}

bool G91(const char* msg, String buf, int serial) {
  printResponse(msg, serial);
  positionMode = RELATIVE;
  return true;
}
