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
char* U_Param = (char*)"U";
char* B_Param = (char*)"B";
char* D_Param = (char*)"D";
char* H_Param = (char*)"H";

extern char* toolCount;
extern char* bowdenLength;
extern char* selectorDist;
extern char* autoClose;
extern char* psTimeout;
extern char* sendAction;
extern char* emulatePrusa;
extern char* hasPanelDue;
extern char* periodicalStats;
extern char* wipeSequence;
extern char* backlightColor;
extern char* encoderTicks;
extern char* maxSpeed;
extern char* acceleration;
extern char* invertDir;
extern char* endstopTrig;
extern char* stepDelay;
extern char* stepsPerMillimeter;
extern char* offset;
extern char* power;
extern char* mode;
extern char* rsense;
extern char* msteps;
extern char* stall;
extern char* cstepmin;
extern char* cstepmax;
extern char* cstepdown;
extern char* drvrAdr;
extern char* toff;
extern char* stopOnStall;
extern char* maxStallCount;
extern char* spacing;
extern char* externalControl;
extern char* insertSpeed;
extern char* reinforceLength;
extern char* unloadRetract;
extern char* unloadPushback;
extern char* pushbackDelay;
extern char* enableChunks;
extern char* feedChunks;
extern char* insertLength;
extern char* duetLaser;
extern char* sharedStepper;
extern char* wiggle;
extern char* useServo;
extern char* servoOffPos;
extern char* servoOnPos;
extern char* servo1Cycles;
extern char* servo2Cycles;
extern char* resetBeforeFeed;
extern char* homeAfterFeed;
extern char* stepsPerRevolution;

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

  {  17, M17 },
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
  { 122, M122 },
  { 150, M150 },
  { 201, M201 },
  { 203, M203 },
  { 205, M205 },
  { 206, M206 },
  { 250, M250 },
  { 280, M280 },
  { 300, M300 },
  { 350, M350 },
  { 412, M412 },
  { 500, M500 },
  { 503, M503 },
  { 569, M569 },
  { 575, M575 },
  { 700, M700 },
  { 701, M701 },
  { 906, M906 },
  { 914, M914 },
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

/*
  Deviating from the GCode standard, this method is used for switching the Feeder stepper motor
  from internal (SMuFF) to external (3D-Printer).
*/
bool M17(const char* msg, String buf, int serial) {
  bool stat = true;
  int stepper;
  if(buf.indexOf(E_Param) != -1) {
    switchFeederStepper(EXTERNAL);
  }
  else if(buf.indexOf(I_Param) != -1) {
    switchFeederStepper(INTERNAL);
  }
  else {
    sprintf_P(tmp, "echo: %s\n", smuffConfig.externalStepper ? P_External : P_Internal);
    printResponse(tmp, serial); 
  }
  printResponse(msg, serial); 
  return stat;
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
  //__debug(PSTR("Fan speed: %d%%"), param);
#ifdef __STM32F1__
  fan.setFanSpeed(param);
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
  fan.setFanSpeed(0);
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
  sprintf_P(tmp, P_TMC_StatusAll, 
  String(steppers[SELECTOR].getStepPositionMM()).c_str(),
  String(steppers[REVOLVER].getStepPosition()).c_str(),
  String(steppers[FEEDER].getStepPositionMM()).c_str(),
  "");
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

/*
  Print out all infomration available on the TMC steppers
  if there are any configured.

  Looks a bit messy and could have been done quite a bit more
  modular, although, as you might need it only once or twice
  it's ok for now.
*/
bool M122(const char* msg, String buf, int serial) {
  char dbg[20];
  printResponse(msg, serial);

  if(drivers[SELECTOR] != NULL) {
    steppers[SELECTOR].setEnabled(true);
  }
  if(drivers[REVOLVER] != NULL) {
    steppers[REVOLVER].setEnabled(true);
  }
  if(drivers[FEEDER] != NULL) {
    steppers[FEEDER].setEnabled(true);
  }
  char spacer[] = {"          "};
  char eol[] = {"\n"};

  printResponseP(P_TMC_Setup00, serial); 
  // IC-Version
  printResponseP(P_TMC_Setup01, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4s      ", String(drivers[i]->version()-0x20).c_str()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Enabled
  printResponseP(P_TMC_Setup02, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4s      ", drivers[i]->isEnabled() ? P_Yes : P_No); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Power set
  printResponseP(P_TMC_Setup03, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4d      ", smuffConfig.stepperPower[i]); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Power RMS
  printResponseP(P_TMC_Setup03a, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4d      ", drivers[i]->rms_current()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Microsteps
  printResponseP(P_TMC_Setup04, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4d      ", drivers[i]->microsteps()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // TOff
  printResponseP(P_TMC_Setup05, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4d      ", drivers[i]->toff()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Blank Time
  printResponseP(P_TMC_Setup06, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4d      ", drivers[i]->blank_time()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // PDN/UART
  printResponseP(P_TMC_Setup07, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4s      ", drivers[i]->pdn_uart() ? P_Yes : P_No); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // MS1 / MS2
  printResponseP(P_TMC_Setup08, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "  %1d%1d      ", drivers[i]->ms2(), drivers[i]->ms1()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Diag
  printResponseP(P_TMC_Setup09, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4s      ", drivers[i]->diag() ? P_High : P_Low); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // StallGuard THRS  
  printResponseP(P_TMC_Setup10, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4d      ", drivers[i]->SGTHRS()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // StallGuard Result  
  printResponseP(P_TMC_Setup11, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4d      ", drivers[i]->SG_RESULT()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Max Stall Count
  printResponseP(P_TMC_Setup12, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4d      ", smuffConfig.stepperMaxStallCnt[i]); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // semin  
  printResponseP(P_TMC_Setup13, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4d      ", drivers[i]->semin()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // semax
  printResponseP(P_TMC_Setup14, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4d      ", drivers[i]->semax()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // sedn
  printResponseP(P_TMC_Setup15, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4d      ", drivers[i]->sedn()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // CoolStep THRS
  printResponseP(P_TMC_Setup16, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%08lx  ", drivers[i]->TCOOLTHRS()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // PWM THRS
  printResponseP(P_TMC_Setup17, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%08lx  ", drivers[i]->TPWMTHRS()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // TSTEP
  printResponseP(P_TMC_Setup18, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%08lx  ", drivers[i]->TSTEP()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // IRUN
  printResponseP(P_TMC_Setup19, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4s      ", String(drivers[i]->rms_current()/32 * drivers[i]->irun()).c_str()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // IHOLD
  printResponseP(P_TMC_Setup20, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4s      ", String(drivers[i]->rms_current()/32 * drivers[i]->ihold()).c_str()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // IHOLD delay
  printResponseP(P_TMC_Setup21, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4d      ", drivers[i]->iholddelay()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // TPOWERDOWN
  printResponseP(P_TMC_Setup22, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4s      ", String((float)(drivers[i]->TPOWERDOWN()*0.021875)).c_str()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // PWM Gradient
  printResponseP(P_TMC_Setup23, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%3d/%3d   ", drivers[i]->pwm_grad(), drivers[i]->pwm_autograd()); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // PWM Scale
  printResponseP(P_TMC_Setup24, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%3d/%3d   ", drivers[i]->pwm_ofs(), drivers[i]->pwm_ofs_auto()); 
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
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%-7s   ", drivers[i]->stealth() ? P_Stealth : P_Spread); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Standstill
  printResponseP(P_TMC_Status02, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4s      ", drivers[i]->stst() ? P_Yes : P_No); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Phase A Open
  printResponseP(P_TMC_Status03, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4s      ", drivers[i]->ola() ? P_Yes : P_No); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Phase B Open
  printResponseP(P_TMC_Status04, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4s      ", drivers[i]->olb() ? P_Yes : P_No); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Phase A Short to GND
  printResponseP(P_TMC_Status05, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4s      ", drivers[i]->s2ga() ? P_Yes : P_No); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Phase B Short to GND
  printResponseP(P_TMC_Status06, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4s      ", drivers[i]->s2gb() ? P_Yes : P_No); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Phase A Short MOSFET
  printResponseP(P_TMC_Status07, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4s      ", drivers[i]->s2vsa() ? P_Yes : P_No); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Phase B Short MOSFET
  printResponseP(P_TMC_Status08, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4s      ", drivers[i]->s2vsb() ? P_Yes : P_No); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Overtemp. Warning
  printResponseP(P_TMC_Status09, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4s      ", drivers[i]->otpw() ? P_Yes : P_No); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  // Overtemp. 
  printResponseP(P_TMC_Status10, serial);
  for(int i=0; i<NUM_STEPPERS; i++) {
    if(drivers[i] != NULL) {
      sprintf(dbg, "%4s      ", drivers[i]->ot() ? P_Yes : P_No); 
      printResponse(dbg, serial);
    }
    else
      printResponse(spacer, serial);
  }
  printResponse(eol, serial);
  return true;
}

bool M150(const char* msg, String buf, int serial) {
  int red=0, green=0, blue=0, index=-1, intensity = 255, colorNdx=-1;
  printResponse(msg, serial); 

#if defined(USE_FASTLED_BACKLIGHT)
  if(NEOPIXEL_PIN == -1)
      return false;

  if((param = getParam(buf, R_Param)) != -1) {
    red = param;
  }
  if((param = getParam(buf, U_Param)) != -1) {
    green = param;
  }
  if((param = getParam(buf, B_Param)) != -1) {
    blue = param;
  }
  if((param = getParam(buf, S_Param)) != -1) {
    index = param;
  }
  if((param = getParam(buf, P_Param)) != -1) {
    intensity = param;
  }
  if((param = getParam(buf, C_Param)) != -1) {
    colorNdx = param;
  }
  setFastLEDIntensity(intensity);
  if(colorNdx != -1) {
    if(index == -1)
      setBacklightIndex(colorNdx);
    else
      if(index >= 0 && index < NUM_LEDS)
        setFastLEDIndex(index, colorNdx);
  }
  else {
    if(index == -1 || index > NUM_LEDS)
      return false;
    CRGB color = CRGB(red, green, blue);
    setFastLED(index, color);
  }
  return true;
#else
  return false;
#endif 
}

bool M201(const char* msg, String buf, int serial) {
  bool stat = true;
  printResponse(msg, serial); 
  if(buf.length()==0) {
    printAcceleration(serial);
    return stat;
  }
  if((param = getParam(buf, X_Param))  != -1) {
    if(param >= MIN_MMS && param <= MAX_MMS) {
      param = translateTicks(param, smuffConfig.stepsPerMM_X);
      steppers[SELECTOR].setAcceleration(param);
      smuffConfig.acceleration_X = param;
    }
    else stat = false;
  }
  if((param = getParam(buf, Y_Param))  != -1) {
    if(param >= MIN_MMS && param <= MAX_MMS) {
      param = translateTicks(param, smuffConfig.stepsPerRevolution_Y/360);
      steppers[REVOLVER].setAcceleration(param);
      smuffConfig.acceleration_Y = param;
    }
    else stat = false;
  }
  if((param = getParam(buf, Z_Param))  != -1) {
    if(param >= MIN_MMS && param <= MAX_MMS) {
      param = translateTicks(param, smuffConfig.stepsPerMM_Z);
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
    if(param > MIN_MMS && param <= MAX_MMS) {
      param = translateTicks(param, smuffConfig.stepsPerMM_X);
      steppers[SELECTOR].setMaxSpeed(param);
      smuffConfig.maxSpeed_X = param;
    }
    else stat = false;
    if((param = getParam(buf, S_Param))  != -1) {
      smuffConfig.stepDelay_X = (int)param;
    }
    if((param = getParam(buf, D_Param))  != -1) {
      smuffConfig.stepDelay_X = (int)param;
    }
  }
  if((param = getParam(buf, Y_Param))  != -1) {
    if(param > MIN_MMS && param <= MAX_MMS) {
      param = translateTicks(param, smuffConfig.stepsPerRevolution_Y/360);
      steppers[REVOLVER].setMaxSpeed(param);
      smuffConfig.maxSpeed_Y = param;
    }
    else stat = false;
    if((param = getParam(buf, S_Param))  != -1) {
      smuffConfig.stepDelay_Y = (int)param;
    }
    if((param = getParam(buf, D_Param))  != -1) {
      smuffConfig.stepDelay_Y = (int)param;
    }
  }
  if((param = getParam(buf, Z_Param))  != -1) {
    if(param > MIN_MMS && param <= MAX_MMS) {
      param = translateTicks(param, smuffConfig.stepsPerMM_Z);
      steppers[FEEDER].setMaxSpeed(param);
      smuffConfig.maxSpeed_Z = param;
    }
    else stat = false;
    if((param = getParam(buf, S_Param))  != -1) {
      smuffConfig.stepDelay_Z = (int)param;
    }
    if((param = getParam(buf, D_Param))  != -1) {
      smuffConfig.stepDelay_Z = (int)param;
    }
    if((param = getParam(buf, F_Param))  != -1) {
      smuffConfig.insertSpeed_Z = param;
      if(smuffConfig.insertSpeed_Z > smuffConfig.acceleration_Z)
        smuffConfig.acceleration_Z = smuffConfig.insertSpeed_Z;
    }
  }
  return stat;
}

void rangeError(int serial, int min, int max) {
  char tmp[80];
  sprintf_P(tmp, P_UseRangeI, min, max);
  printResponseP(P_RangeError, serial);
  printResponse(tmp, serial);
}

void rangeError(int serial, long min, long max) {
  char tmp[80];
  sprintf_P(tmp, P_UseRangeL, min, max);
  printResponseP(P_RangeError, serial);
  printResponse(tmp, serial);
}

void rangeError(int serial, float min, float max) {
  char tmp[80];
  sprintf_P(tmp, P_UseRangeF, String(min).c_str(), String(max).c_str());
  printResponseP(P_RangeError, serial);
  printResponse(tmp, serial);
}

bool M205(const char* msg, String buf, int serial) {
  bool stat = true;
  printResponse(msg, serial); 

  char cmd[80];
  char tmp[50];
  if((param = getParamString(buf, P_Param, cmd, sizeof(cmd)))  != -1) {
    float fParam = getParamF(buf, S_Param);
    if((param = getParam(buf, S_Param)) != -1) {
      //__debug(PSTR("Value: %d"), param);

      if(strcmp(cmd, toolCount)==0) {
        if(param >= 1 && param <= MAX_TOOLS)
          smuffConfig.toolCount = param;
        else
          rangeError(serial, 1, MAX_TOOLS);
      }
      else if(strcmp(cmd, spacing)==0) {
        smuffConfig.toolSpacing = fParam;
      }
      else if(strcmp(cmd, bowdenLength)==0) {
        smuffConfig.bowdenLength = fParam;
      }
      else if(strcmp(cmd, insertLength)==0) {
        smuffConfig.insertLength = fParam;
      }
      else if(strcmp(cmd, reinforceLength)==0) {
        smuffConfig.reinforceLength = fParam;
      }
      else if(strcmp(cmd, selectorDist)==0) {
        smuffConfig.selectorDistance = fParam;
      }
      else if(strcmp(cmd, homeAfterFeed)==0) {
        smuffConfig.homeAfterFeed = (param > 0);
      }
      else if(strcmp(cmd, resetBeforeFeed)==0) {
        smuffConfig.resetBeforeFeed_Y = (param > 0);
      }
      else if(strcmp(cmd, emulatePrusa)==0) {
        smuffConfig.prusaMMU2 = (param > 0);
      }
      else if(strcmp(cmd, useServo)==0) {
        smuffConfig.revolverIsServo = (param > 0);
      }
      else if(strcmp(cmd, servoOffPos)==0) {
        smuffConfig.revolverOffPos = param;
      }
      else if(strcmp(cmd, servoOnPos)==0) {
        smuffConfig.revolverOnPos = param;
      }
      else if(strcmp(cmd, wipeSequence)==0) {
        getParamString(cmd, S_Param, tmp, sizeof(smuffConfig.wipeSequence));
        #if defined(__STM32F1__) || defined(__ESP32__)
        strncpy(smuffConfig.wipeSequence, tmp, sizeof(smuffConfig.wipeSequence));
        #else
        strlcpy(smuffConfig.wipeSequence, tmp, sizeof(smuffConfig.wipeSequence));
        #endif
      }
      else if(strcmp(cmd, stepsPerMillimeter)==0) {
        if(hasParam(buf, X_Param)) {
          smuffConfig.stepsPerMM_X = param;
          steppers[SELECTOR].setStepsPerMM(param);
        }
        if(hasParam(buf, Z_Param)) {
          smuffConfig.stepsPerMM_Z = param;
          steppers[FEEDER].setStepsPerMM(param);
        }
      }
      else if(strcmp(cmd, stepsPerRevolution)==0) {
        smuffConfig.stepsPerRevolution_Y = param;
        steppers[REVOLVER].setStepsPerDegree(param/360);
      }
      else if(strcmp(cmd, invertDir)==0) {
        if(hasParam(buf, X_Param))
          smuffConfig.invertDir_X = (param > 0);
        if(hasParam(buf, Y_Param))
          smuffConfig.invertDir_Y = (param > 0);
        if(hasParam(buf, Z_Param))
          smuffConfig.invertDir_Z = (param > 0);
      }
      else if(strcmp(cmd, endstopTrig)==0) {
        if(hasParam(buf, X_Param))
          smuffConfig.endstopTrigger_X = param;
        if(hasParam(buf, Y_Param))
          smuffConfig.endstopTrigger_Y = param;
        if(hasParam(buf, Z_Param))
          smuffConfig.endstopTrigger_Z = param;
      }
      else if(strcmp(cmd, autoClose)==0) {
        smuffConfig.menuAutoClose = param;
      }
      else if(strcmp(cmd, psTimeout)==0) {
        smuffConfig.powerSaveTimeout = param;
      }
      else if(strcmp(cmd, sendAction)==0) {
        smuffConfig.sendActionCmds = (param > 0);
      }
      else if(strcmp(cmd, backlightColor)==0) {
        if(param >=0 && param <=15)
          smuffConfig.backlightColor = param;
      }
      else if(strcmp(cmd, hasPanelDue)==0) {
        smuffConfig.hasPanelDue = (param > 0);
      }
      else if(strcmp(cmd, duetLaser)==0) {
        smuffConfig.useDuetLaser = (param > 0);
      }
      else if(strcmp(cmd, sharedStepper)==0) {
        smuffConfig.isSharedStepper = (param > 0);
      }
      else if(strcmp(cmd, encoderTicks)==0) {
        smuffConfig.encoderTickSound = (param > 0);
      }
      else if(strcmp(cmd, periodicalStats)==0) {
        smuffConfig.sendPeriodicalStats = (param > 0);
      }
      else if(strcmp(cmd, power)==0) {
        if(param >=0 && param <= MAX_POWER) {
          if(hasParam(buf, X_Param)) {
            smuffConfig.stepperPower[SELECTOR] = param;
            if(drivers[SELECTOR] != NULL)
              drivers[SELECTOR]->rms_current(param);
          }
          if(hasParam(buf, Y_Param)) {
            smuffConfig.stepperPower[REVOLVER] = param;
            if(drivers[REVOLVER] != NULL)
              drivers[REVOLVER]->rms_current(param);
          }
          if(hasParam(buf, Z_Param)) {
            smuffConfig.stepperPower[FEEDER] = param;
            if(drivers[FEEDER] != NULL)
              drivers[FEEDER]->rms_current(param);
          }
        }
        else
          rangeError(serial, 0, MAX_POWER);
      }
      else if(strcmp(cmd, rsense)==0) {
        if(fParam >= 0 && fParam <= 1.0) {
          if(hasParam(buf, X_Param)) {
            smuffConfig.stepperRSense[SELECTOR] = fParam;
          }
          if(hasParam(buf, Y_Param)) {
            smuffConfig.stepperPower[REVOLVER] = fParam;
          }
          if(hasParam(buf, Z_Param)) {
            smuffConfig.stepperPower[FEEDER] = fParam;
          }
        }
        else
          rangeError(serial, 0, 1.0);
      }
      else if(strcmp(cmd, mode)==0) {
        if(param >=0 && param <= 2) {
          if(hasParam(buf, X_Param)) {
            smuffConfig.stepperMode[SELECTOR] = param;
          }
          if(hasParam(buf, Y_Param)) {
            smuffConfig.stepperMode[REVOLVER] = param;
          }
          if(hasParam(buf, Z_Param)) {
            smuffConfig.stepperMode[FEEDER] = param;
          }
        }
        else
          rangeError(serial, 0, 2);
      }
      else if(strcmp(cmd, msteps)==0) {
        if(param == 1 || param == 2 || param == 4 || param == 8 || param == 16 || param == 32 || param == 64 || param == 128) {
          if(hasParam(buf, X_Param)) {
            smuffConfig.stepperMicrosteps[SELECTOR] = param;
            if(drivers[SELECTOR] != NULL)
              drivers[SELECTOR]->microsteps(param);
          }
          if(hasParam(buf, Y_Param)) {
            smuffConfig.stepperMicrosteps[REVOLVER] = param;
            if(drivers[REVOLVER] != NULL)
              drivers[REVOLVER]->microsteps(param);
          }
          if(hasParam(buf, Z_Param)) {
            smuffConfig.stepperMicrosteps[FEEDER] = param;
            if(drivers[FEEDER] != NULL)
              drivers[FEEDER]->microsteps(param);
          }
        }
        else
          rangeError(serial, 1, 128);
      }
      else if(strcmp(cmd, stall)==0) {
        if(param >=0 && param <=255) {
          if(hasParam(buf, X_Param)) {
            smuffConfig.stepperStall[SELECTOR] = param;
            if(drivers[SELECTOR] != NULL)
              drivers[SELECTOR]->SGTHRS(param);
          }
          if(hasParam(buf, Y_Param)) {
            smuffConfig.stepperStall[REVOLVER] = param;
            if(drivers[REVOLVER] != NULL)
              drivers[REVOLVER]->SGTHRS(param);
          }
          if(hasParam(buf, Z_Param)) {
            smuffConfig.stepperStall[FEEDER] = param;
            if(drivers[FEEDER] != NULL)
              drivers[FEEDER]->SGTHRS(param);
          }
        }
        else
          rangeError(serial, 0, 255);
      }
      else if(strcmp(cmd, cstepmin)==0) {
        if(param >=0 && param <= 15) {
          if(hasParam(buf, X_Param)) {
            smuffConfig.stepperCSmin[SELECTOR] = param;
            if(drivers[SELECTOR] != NULL)
              drivers[SELECTOR]->semin(param);
          }
          if(hasParam(buf, Y_Param)) {
            smuffConfig.stepperCSmin[REVOLVER] = param;
            if(drivers[REVOLVER] != NULL)
              drivers[REVOLVER]->semin(param);
          }
          if(hasParam(buf, Z_Param)) {
            smuffConfig.stepperCSmin[FEEDER] = param;
            if(drivers[FEEDER] != NULL)
              drivers[FEEDER]->semin(param);
          }
        }
        else
          rangeError(serial, 0, 15);
      }
      else if(strcmp(cmd, cstepmax)==0) {
        if(param >=0 && param <= 15) {
          if(hasParam(buf, X_Param)) {
            smuffConfig.stepperCSmax[SELECTOR] = param;
            if(drivers[SELECTOR] != NULL)
              drivers[SELECTOR]->semax(param);
          }
          if(hasParam(buf, Y_Param)) {
            smuffConfig.stepperCSmax[REVOLVER] = param;
            if(drivers[REVOLVER] != NULL)
              drivers[REVOLVER]->semax(param);
          }
          if(hasParam(buf, Z_Param)) {
            smuffConfig.stepperCSmax[FEEDER] = param;
            if(drivers[FEEDER] != NULL)
              drivers[FEEDER]->semax(param);
          }
        }
        else
          rangeError(serial, 0, 15);
      }
      else if(strcmp(cmd, cstepdown)==0) {
        if(param >=0 && param <= 15) {
          if(hasParam(buf, X_Param)) {
            smuffConfig.stepperCSdown[SELECTOR] = param;
            if(drivers[SELECTOR] != NULL)
              drivers[SELECTOR]->sedn(param);
          }
          if(hasParam(buf, Y_Param)) {
            smuffConfig.stepperCSdown[REVOLVER] = param;
            if(drivers[REVOLVER] != NULL)
              drivers[REVOLVER]->sedn(param);
          }
          if(hasParam(buf, Z_Param)) {
            smuffConfig.stepperCSdown[FEEDER] = param;
            if(drivers[FEEDER] != NULL)
              drivers[FEEDER]->sedn(param);
          }
        }
        else
          rangeError(serial, 0, 15);
      }
      else if(strcmp(cmd, drvrAdr)==0) {
        if(param >=0 && param <= 3) {
          if(hasParam(buf, X_Param)) {
            smuffConfig.stepperAddr[SELECTOR] = param;
          }
          if(hasParam(buf, Y_Param)) {
            smuffConfig.stepperAddr[REVOLVER] = param;
          }
          if(hasParam(buf, Z_Param)) {
            smuffConfig.stepperAddr[FEEDER] = param;
          }
        }
        else
          rangeError(serial, 0, 3);
      }
      else if(strcmp(cmd, toff)==0) {
        if(param >=0 && param <= 15) {
          if(hasParam(buf, X_Param)) {
            smuffConfig.stepperToff[SELECTOR] = param;
            if(drivers[SELECTOR] != NULL)
              drivers[SELECTOR]->toff(param);
          }
          if(hasParam(buf, Y_Param)) {
            smuffConfig.stepperToff[REVOLVER] = param;
            if(drivers[REVOLVER] != NULL)
              drivers[REVOLVER]->toff(param);
          }
          if(hasParam(buf, Z_Param)) {
            smuffConfig.stepperToff[FEEDER] = param;
            if(drivers[FEEDER] != NULL)
              drivers[FEEDER]->toff(param);
          }
        }
        else
          rangeError(serial, 0, 15);
      }
      else if(strcmp(cmd, stopOnStall)==0) {
        if(hasParam(buf, X_Param)) {
          smuffConfig.stepperStopOnStall[SELECTOR] = (param > 0);
          steppers[SELECTOR].setStopOnStallDetected(param > 0);
        }
        if(hasParam(buf, Y_Param)) {
          smuffConfig.stepperStopOnStall[REVOLVER] = (param > 0);
          steppers[REVOLVER].setStopOnStallDetected(param > 0);
        }
        if(hasParam(buf, Z_Param)) {
          smuffConfig.stepperStopOnStall[FEEDER] = (param > 0);
          steppers[FEEDER].setStopOnStallDetected(param > 0);
        }
      }
      else if(strcmp(cmd, maxStallCount)==0) {
        if(param >=0 && param <= MAX_STALL_COUNT) {
          if(hasParam(buf, X_Param)) {
            smuffConfig.stepperMaxStallCnt[SELECTOR] = param;
            steppers[SELECTOR].setStallThreshold(param);
          }
          if(hasParam(buf, Y_Param)) {
            smuffConfig.stepperMaxStallCnt[REVOLVER] = param;
            steppers[REVOLVER].setStallThreshold(param);
          }
          if(hasParam(buf, Z_Param)) {
            smuffConfig.stepperMaxStallCnt[FEEDER] = param;
            steppers[FEEDER].setStallThreshold(param);
          }
        }
        else
          rangeError(serial, 0, MAX_STALL_COUNT);
      }
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
    for(int i=0; i <= 180; i += 10) {
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
  char sequence[500];
  if((param = getParam(buf, S_Param)) != -1) {
    int frequency = param;
    if((param = getParam(buf, P_Param)) != -1) {
      tone(BEEPER_PIN, frequency, param);
    }
    else 
      stat = false;
  }
  else if(getParamString(buf, T_Param, sequence, 499)) {
    playSequence(sequence);
  }
  else 
    stat = false;
  return stat;
}

bool M350(const char* msg, String buf, int serial) {
  bool stat = true;
  printResponse(msg, serial); 
  if((param = getParam(buf, X_Param))  != -1) {
    if(param==1 || param==2 || param==4 || param==8 || param==16 || param==32 || param==64 || param==128) {
      smuffConfig.stepperMicrosteps[SELECTOR] = param;
      steppers[SELECTOR].setEnabled(true);
      if(drivers[SELECTOR] != NULL)
        drivers[SELECTOR]->microsteps(param);
    }
    else stat = false;
  }
  if((param = getParam(buf, Y_Param))  != -1) {
    if(param==1 || param==2 || param==4 || param==8 || param==16 || param==32 || param==64 || param==128) {
      smuffConfig.stepperMicrosteps[REVOLVER] = param;
      steppers[REVOLVER].setEnabled(true);
      if(drivers[REVOLVER] != NULL)
        drivers[REVOLVER]->microsteps(param);
    }
    else stat = false;
  }
  if((param = getParam(buf, Z_Param))  != -1) {
    if(param==1 || param==2 || param==4 || param==8 || param==16 || param==32 || param==64 || param==128) {
      smuffConfig.stepperMicrosteps[FEEDER] = param;
      steppers[FEEDER].setEnabled(true);
      if(drivers[FEEDER] != NULL)
        drivers[FEEDER]->microsteps(param);
    }
    else stat = false;
  }
  return stat;
}

bool M412(const char* msg, String buf, int serial) {
  bool stat = true;
  printResponse(msg, serial);
  if((param = getParam(buf, S_Param)) != -1) {
    smuffConfig.runoutDetection = param == 1;
  }
  else {
    printResponseP(smuffConfig.runoutDetection ? P_On : P_Off ,serial);
  }
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
      #if !defined(__ESP32__)
      case 3:
        _print = &Serial3;
        break;
      #endif
      default:
        break;
    }
  return writeConfig(_print);
}

bool M569(const char* msg, String buf, int serial) {
  bool stat = true;
  char tmp[128];
  printResponse(msg, serial);

  if(!hasParam(buf, X_Param) && !hasParam(buf, Y_Param) && !hasParam(buf, Z_Param)) {
    printDriverMode(serial);
    return true;
  }

  if((param = getParam(buf, X_Param))  != -1) {
    if(param >= 0 && param <= 1) {
      steppers[SELECTOR].setEnabled(true);
      if(drivers[SELECTOR] != NULL) {
        setDriverSpreadCycle(drivers[SELECTOR], (param >0), smuffConfig.stepperStall[SELECTOR], smuffConfig.stepperCSmin[SELECTOR], smuffConfig.stepperCSmax[SELECTOR], smuffConfig.stepperCSdown[SELECTOR]);
        if(param == 0 && STALL_X_PIN != -1)
          attachInterrupt(STALL_X_PIN, isrStallDetectedX, RISING);
      }
      else {
        printResponseP(P_StepperNotCfg, serial);
        if(STALL_X_PIN != -1)
          detachInterrupt(STALL_X_PIN);
      }
    }
    else stat = false;
  }

  if((param = getParam(buf, Y_Param))  != -1) {
    if(param >= 0 && param <= 1) {
      steppers[REVOLVER].setEnabled(true);
      if(drivers[REVOLVER] != NULL) {
        setDriverSpreadCycle(drivers[REVOLVER], (param >0), smuffConfig.stepperStall[REVOLVER], smuffConfig.stepperCSmin[REVOLVER], smuffConfig.stepperCSmax[REVOLVER], smuffConfig.stepperCSdown[REVOLVER]);
        if(param == 0 && STALL_Y_PIN != -1)
          attachInterrupt(STALL_Y_PIN, isrStallDetectedY, RISING);
      }
      else {
        printResponseP(P_StepperNotCfg, serial);
        if(STALL_Y_PIN != -1)
          detachInterrupt(STALL_Y_PIN);
      }
    }
    else stat = false;
  }

  if((param = getParam(buf, Z_Param))  != -1) {
    if(param >= 0 && param <= 1) {
      steppers[FEEDER].setEnabled(true);
      if(drivers[FEEDER] != NULL) {
        setDriverSpreadCycle(drivers[FEEDER], (param >0), smuffConfig.stepperStall[FEEDER], smuffConfig.stepperCSmin[FEEDER], smuffConfig.stepperCSmax[FEEDER], smuffConfig.stepperCSdown[FEEDER]);
        if(param == 0 && STALL_Z_PIN != -1)
          attachInterrupt(STALL_Z_PIN, isrStallDetectedZ, RISING);
      }
      else {
        printResponseP(P_StepperNotCfg, serial);
        if(STALL_Z_PIN != -1)
          detachInterrupt(STALL_Z_PIN);
      }
    }
    else stat = false;
  }

  return stat;
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
        case 0: smuffConfig.serial0Baudrate = paramL; break;
        case 1: smuffConfig.serial1Baudrate = paramL; break;
        case 2: smuffConfig.serial2Baudrate = paramL; break;
        #if !defined(__ESP32__)
        case 3: smuffConfig.serial3Baudrate = paramL; break;
        #endif
      }
    }
    else {
      smuffConfig.serial0Baudrate = paramL;
      smuffConfig.serial1Baudrate = paramL;
      smuffConfig.serial2Baudrate = paramL;
      #if !defined(__ESP32__)
      smuffConfig.serial3Baudrate = paramL;
      #endif
    }
    setupSerial();
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

bool M906(const char* msg, String buf, int serial) {
  bool stat = true;
  printResponse(msg, serial); 

  if(!hasParam(buf, X_Param) && !hasParam(buf, Y_Param) && !hasParam(buf, Z_Param)) {
    printDriverRms(serial);
    return true;
  }
  int irun = getParam(buf, R_Param);
  int ihold = getParam(buf, H_Param);

  if((param = getParam(buf, X_Param))  != -1) {
    if(param > 0 && param <= MAX_POWER) {
      smuffConfig.stepperPower[SELECTOR] = param;
      steppers[SELECTOR].setEnabled(true);
      if(drivers[SELECTOR] != NULL) {
        drivers[SELECTOR]->rms_current(param);
        if(irun >=0 && irun <= 31) 
          drivers[SELECTOR]->irun(irun);
        if(ihold >=0 && irun <= 31) 
          drivers[SELECTOR]->ihold(ihold);
      }
      else
        printResponseP(P_StepperNotCfg, serial);
    }
    else stat = false;
  }
  if((param = getParam(buf, Y_Param))  != -1) {
    if(param > 0 && param <= MAX_POWER) {
      smuffConfig.stepperPower[REVOLVER] = param;
      steppers[REVOLVER].setEnabled(true);
      if(drivers[REVOLVER] != NULL) {
        drivers[REVOLVER]->rms_current(param);
        if(irun >=0 && irun <= 31) 
          drivers[REVOLVER]->irun(irun);
        if(ihold >=0 && irun <= 31) 
          drivers[REVOLVER]->ihold(ihold);
      }
      else
        printResponseP(P_StepperNotCfg, serial);
    }
    else stat = false;
  }
  if((param = getParam(buf, Z_Param))  != -1) {
    if(param > 0 && param <= MAX_POWER) {
      smuffConfig.stepperPower[FEEDER] = param;
      steppers[FEEDER].setEnabled(true);
      if(drivers[FEEDER] != NULL) {
        drivers[FEEDER]->rms_current(param);
        if(irun >=0 && irun <= 31) 
          drivers[FEEDER]->irun(irun);
        if(ihold >=0 && irun <= 31) 
          drivers[FEEDER]->ihold(ihold);
      }
      else
        printResponseP(P_StepperNotCfg, serial);
    }
    else stat = false;
  }
  return stat;
}

bool M914(const char* msg, String buf, int serial) {
  bool stat = true;
  uint16_t cool = 0;
  printResponse(msg, serial); 
  
  if(!hasParam(buf, X_Param) && !hasParam(buf, Y_Param) && !hasParam(buf, Z_Param)) {
    printDriverStallThrs(serial);
    return true;
  }
  if(hasParam(buf, C_Param)) {
    cool = getParam(buf, C_Param);
  }
  int trigg = getParam(buf, T_Param);

  if((param = getParam(buf, X_Param))  != -1) {
    if(param > 0 && param <= 255) {
      smuffConfig.stepperStall[SELECTOR] = param;
      steppers[SELECTOR].setEnabled(true);
      if(drivers[SELECTOR] != NULL && drivers[SELECTOR]->stealth()) {
        drivers[SELECTOR]->SGTHRS(param);
        if(cool > 0)
          drivers[SELECTOR]->TCOOLTHRS(cool);
        if(trigg != -1)
          steppers[SELECTOR].setStallThreshold(trigg);
        if(STALL_X_PIN != -1)
          attachInterrupt(STALL_X_PIN, isrStallDetectedX, RISING);
      }
      else {
        printResponseP(P_StepperNotCfg, serial);
        if(STALL_X_PIN != -1)
          detachInterrupt(STALL_X_PIN);
      }
    }
    else stat = false;
  }
  if((param = getParam(buf, Y_Param))  != -1) {
    if(param > 0 && param <= 255) {
      smuffConfig.stepperStall[REVOLVER] = param;
      steppers[REVOLVER].setEnabled(true);
      if(drivers[REVOLVER] != NULL && drivers[REVOLVER]->stealth()) {
        drivers[REVOLVER]->SGTHRS(param);
        if(cool > 0)
          drivers[REVOLVER]->TCOOLTHRS(cool);
        if(trigg != -1)
          steppers[REVOLVER].setStallThreshold(trigg);
        if(STALL_Y_PIN != -1)
          attachInterrupt(STALL_Y_PIN, isrStallDetectedY, RISING);
      }
      else {
        printResponseP(P_StepperNotCfg, serial);
        if(STALL_Y_PIN != -1)
          detachInterrupt(STALL_Y_PIN);
      }
    }
    else stat = false;
  }
  if((param = getParam(buf, Z_Param))  != -1) {
    if(param > 0 && param <= 255) {
      smuffConfig.stepperStall[FEEDER] = param;
      steppers[FEEDER].setEnabled(true);
      if(drivers[FEEDER] != NULL && drivers[FEEDER]->stealth()) {
        drivers[FEEDER]->SGTHRS(param);
        if(cool > 0)
          drivers[FEEDER]->TCOOLTHRS(cool);
        if(trigg != -1)
          steppers[FEEDER].setStallThreshold(trigg);
        if(STALL_Z_PIN != -1)
          attachInterrupt(STALL_Z_PIN, isrStallDetectedZ, RISING);
      }
      else {
        printResponseP(P_StepperNotCfg, serial);
        if(STALL_Z_PIN != -1)
          detachInterrupt(STALL_Z_PIN);
      }
    }
    else stat = false;
  }
  return stat;
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
    bool posOk = false;
    int retry = 3;
    do {
      steppers[SELECTOR].resetStallDetected();
      prepSteppingAbsMillimeter(SELECTOR, smuffConfig.firstToolOffset + (param * smuffConfig.toolSpacing));
      runAndWait(SELECTOR);
      if(steppers[SELECTOR].getStallDetected()) {
        handleStall(SELECTOR);
        posOk = false;
      }
      else 
        posOk = true;
      retry--;
      if(!retry)
        break;
    } while(!posOk);
  }
  return true;
}

void handleFeedSpeed(String buf, int axis) {
  long fspeed;
  if((fspeed = getParam(buf, F_Param)) == -1)
    return;
  else {
    if(fspeed < MIN_MMS) fspeed = MIN_MMS;
    if(fspeed > MAX_MMS) fspeed = MAX_MMS;
    fspeed = translateSpeed((unsigned int)fspeed, steppers[axis].getStepsPerMM());
    steppers[axis].setMaxSpeed(fspeed);
    if(fspeed > (long)steppers[axis].getAcceleration()) {
      long faccel = fspeed*3;
      if(faccel >= 65500)
        faccel = 65500;
      steppers[axis].setAcceleration(faccel);
    }
  }
}

bool G1(const char* msg, String buf, int serial) {
  long curSpeed[NUM_STEPPERS], accel[NUM_STEPPERS];
  printResponse(msg, serial);
  
  // save current speeds
  curSpeed[SELECTOR]  = steppers[SELECTOR].getMaxSpeed(); accel[SELECTOR] = steppers[SELECTOR].getAcceleration();
  curSpeed[REVOLVER]  = steppers[REVOLVER].getMaxSpeed(); accel[REVOLVER] = steppers[REVOLVER].getAcceleration();
  curSpeed[FEEDER]    = steppers[FEEDER].getMaxSpeed();   accel[FEEDER]   = steppers[FEEDER].getAcceleration();

  bool isMill = !hasParam(buf, T_Param);

  if((param = getParam(buf, Y_Param)) != -1) {
    handleFeedSpeed(buf, REVOLVER);
    __debug(PSTR("G1 moving Y: %d %s with speed %ld mm/s"), param, isMill ? "mm" : "steps", translateTicks(steppers[REVOLVER].getMaxSpeed(), steppers[REVOLVER].getStepsPerDegree()));
    steppers[REVOLVER].setEnabled(true);
    prepStepping(REVOLVER, param, isMill);
  }
  if((param = getParam(buf, X_Param)) != -1) {
    handleFeedSpeed(buf, SELECTOR);
    __debug(PSTR("G1 moving X: %d %s with speed %ld mm/s"), param, isMill ? "mm" : "steps", translateTicks(steppers[SELECTOR].getMaxSpeed(), steppers[SELECTOR].getStepsPerMM()));
    steppers[SELECTOR].setEnabled(true);
    prepStepping(SELECTOR, param, isMill, true);
  }
  if((param = getParam(buf, Z_Param)) != -1) {
    handleFeedSpeed(buf, FEEDER);
    __debug(PSTR("G1 moving Z: %d %s with speed %ld mm/s"), param, isMill ? "mm" : "steps", translateTicks(steppers[FEEDER].getMaxSpeed(), steppers[FEEDER].getStepsPerMM()));
    steppers[FEEDER].setEnabled(true);
    prepStepping(FEEDER, param, isMill);
  }
  runAndWait(-1);
  if(hasParam(buf, X_Param) && drivers[SELECTOR] != NULL) {
    uint16_t sr = drivers[SELECTOR]->SG_RESULT();
    __debug(PSTR("G1 StallResult X: %d  Stalled: %s"), sr, steppers[SELECTOR].getStallDetected() ? P_Yes : P_No);
  }
  if(hasParam(buf, Y_Param) && drivers[REVOLVER] != NULL) {
    uint16_t sr = drivers[REVOLVER]->SG_RESULT();
    __debug(PSTR("G1 StallResult Y: %d  Stalled: %s"), sr, steppers[REVOLVER].getStallDetected() ? P_Yes : P_No);
  }
  if(hasParam(buf, Z_Param) && drivers[FEEDER] != NULL) {
    uint16_t sr = drivers[FEEDER]->SG_RESULT();
    __debug(PSTR("G1 StallResult Z: %d  Stalled: %s"), sr, steppers[FEEDER].getStallDetected() ? P_Yes : P_No);
  }
  // set all speeds back to the configured values
  if(hasParam(buf, F_Param)) {
    if(hasParam(buf, X_Param)) {
      steppers[SELECTOR].setMaxSpeed(curSpeed[SELECTOR]);
      steppers[SELECTOR].setAcceleration(accel[SELECTOR]);
    }
    if(hasParam(buf, Y_Param)) {
      steppers[REVOLVER].setMaxSpeed(curSpeed[REVOLVER]);
      steppers[REVOLVER].setAcceleration(accel[REVOLVER]);
    }
    if(hasParam(buf, Z_Param)) {
      steppers[FEEDER].setMaxSpeed(curSpeed[FEEDER]);
      steppers[FEEDER].setAcceleration(accel[FEEDER]);
    }
  }
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
