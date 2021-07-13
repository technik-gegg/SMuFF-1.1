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
#include "SMuFF.h"
#include "Config.h"
#include "ZStepperLib.h"
#include "ZServo.h"
#include "InputDialogs.h"

extern uint8_t  swapTools[];
extern ZStepper steppers[];
extern int8_t   toolSelections[];
bool            forceStopMenu = false;

char            selColorName[2][10];
char            duePort[10];
char            driverMode[10];
char            ms3State[10];

uint8_t        menuOrdinals[MAX_MENU_ORDINALS];

#define CHECK_MENU      1

void checkMenuSize(const char* PROGMEM name, char* menu, size_t size) {
  #if defined(CHECK_MENU)
  if(strlen(menu) > size) {
    char tmp[80];
    sprintf_P(tmp, name);
    __debugS(PSTR("Overrun in %s Menu: size=%d len=%d\n\a\a"), tmp, size, strlen(menu));
  }
  #endif
}

/*
  Checks if the related speed (destVal) is beyond the percentage of the
  initial value. If so, the related speed will be set to the calculated value.
  Used to check whether the acceleration is faster than max. speed for example.
*/
void validateSpeed(int current, uint16_t* destVal, uint8_t percent) {
  unsigned percentVal = (unsigned)round(((float)(current / 100) * percent));
  if(smuffConfig.speedsInMMS) {
    if(percentVal < *destVal)
      *destVal = percentVal;
    if(*destVal == 0)
      *destVal = 1;
  }
  else {
    if(*destVal < current+percentVal)
      *destVal = current+percentVal;
  }
}

/*
  Extract title from current menu entry for sub-dialogs
*/
char* extractTitle(const char* menu, uint8_t index) {
  char* tok = strtok((char*)menu, "\n");
  int8_t cnt = -1;
  // find the line with given index
  while(tok != nullptr) {
    if(++cnt == index)
      break;
    tok = strtok(nullptr, "\n");
  }
  // cut off all extra characters after tab
  if(tok != nullptr) {
    char* tok2 = tok;
    while(*tok2 != '\t') {
      tok2++;
    }
    *tok2 = 0;
    //__debugS(PSTR("Menu: %s tok: %s @index: %d"), menu, tok, index);
    return tok;
  }
  return nullptr;
}

char* extractFile(const char* files, uint8_t index) {
  char* tok = strtok((char*)files, "\n");
  int8_t cnt = -1;
  // find the line with given index
  while(tok != nullptr) {
    if(++cnt == index)
      break;
    tok = strtok(nullptr, "\n");
  }
  return tok;
}

void setupToolsMenu(char* menu) {
  char tmp[50];
  sprintf_P(menu, P_MenuItemBack);
  memset(toolSelections, 0, sizeof(int)*MAX_TOOLS);
  uint8_t n = 0;
  int8_t tool = toolSelected;
  for(uint8_t i=0; i< smuffConfig.toolCount; i++) {
    if(i == tool)
      continue;
    toolSelections[n] = i;
    sprintf_P(tmp, P_ToolMenu, i);
    strcat(menu, tmp);
    strcat(menu, (char*)"\n");
    n++;
  }
  menu[strlen(menu)-1] = '\0';
}

void setupMainMenu(char* menu) {
    char motors[10];
    char servo[30];
    char maint[10];

    steppers[SELECTOR].getEnabled() ? sprintf_P(motors, P_Off) : sprintf_P(motors, P_On);
    lidOpen ? sprintf_P(servo, P_Close) : sprintf_P(servo, P_Open);
    maintainingMode ? sprintf_P(maint, P_Off) : sprintf_P(maint, P_On);

  if(smuffConfig.revolverIsServo && smuffConfig.prusaMMU2) {
    sprintf(menu, loadMenu(P_MnuMain0, menuOrdinals), motors, servo, maint);
  }
  else if(smuffConfig.revolverIsServo && !smuffConfig.prusaMMU2 && !smuffConfig.useSplitter) {
    sprintf(menu, loadMenu(P_MnuMain1, menuOrdinals), motors, servo, maint);
  }
  else if(!smuffConfig.revolverIsServo && smuffConfig.prusaMMU2) {
    sprintf(menu, loadMenu(P_MnuMain2, menuOrdinals), motors, maint);
  }
  else if(smuffConfig.revolverIsServo && !smuffConfig.prusaMMU2 && smuffConfig.useSplitter) {
    sprintf(menu, loadMenu(P_MnuMain4, menuOrdinals), motors, servo, maint);
  }
  else {
    sprintf(menu, loadMenu(P_MnuMain3, menuOrdinals), motors, maint);
  }
}

void setupStatusInfoMenu(char* menu) {
  sprintf(menu, loadMenu(P_MnuStatus, menuOrdinals));
}

void setupSwapMenu(char* menu) {
  char tmp[128];
  sprintf_P(menu, P_MenuItemBack);
  sprintf_P(tmp, P_SwapReset);
  strcat(menu, tmp);
  for(uint8_t i=0; i< smuffConfig.toolCount; i++) {
    sprintf_P(tmp, P_SwapMenu, i, swapTools[i]);
    strcat(menu, tmp);
    strcat(menu, (char*)"\n");
  }
  menu[strlen(menu)-1] = '\0';
}

void setupSettingsMenu(char* menu) {
  sprintf(menu, loadMenu(P_MnuSettings, menuOrdinals),
    smuffConfig.toolCount,
    String(smuffConfig.bowdenLength).c_str(),
    String(smuffConfig.selectorDistance).c_str()
  );
}

void setupOptionsMenu(char* menu) {
  sprintf(menu, loadMenu(P_MnuOptions, menuOrdinals),
    smuffConfig.menuAutoClose,
    smuffConfig.fanSpeed,
    smuffConfig.prusaMMU2 ? P_Yes : P_No,
    smuffConfig.sendPeriodicalStats ? P_Yes : P_No,
    smuffConfig.useDuetLaser ? P_Yes : P_No,
    smuffConfig.hasPanelDue==0 ? P_None : translatePanelDuePort(smuffConfig.hasPanelDue),
    smuffConfig.speedsInMMS ? P_Yes : P_No,
    smuffConfig.servoMinPwm,
    smuffConfig.servoMaxPwm,
    smuffConfig.useCutter ? P_Yes : P_No,
    smuffConfig.cutterOpen,
    smuffConfig.cutterClose,
    smuffConfig.invertRelay ? P_Yes : P_No,
    smuffConfig.useSplitter ? P_Yes : P_No,
    String(smuffConfig.splitterDist).c_str(),
    #if defined(USE_DDE)
    P_Yes,
    #else
    P_No,
    #endif
    String(smuffConfig.ddeDist).c_str(),
    smuffConfig.purgeDDE ? P_Yes : P_No
  );
}

void setupPurgeMenu(char* menu) {
  char tmp[50];

  sprintf(menu, loadMenu(P_MnuPurge, menuOrdinals),
    smuffConfig.usePurge  ? P_Yes : P_No,
    smuffConfig.purgeSpeed,
    String(smuffConfig.purgeLength).c_str(),
    String(smuffConfig.unloadRetract).c_str()
  );

  uint8_t n=7;    // next possible ordinal (after separator)
  for(uint8_t i=0; i< smuffConfig.toolCount; i++) {
    uint8_t ndx = swapTools[i];
    sprintf_P(tmp, P_ToolPurgeMenu, i, smuffConfig.materialNames[ndx], smuffConfig.purges[ndx]);
    strcat(menu, tmp);
    menuOrdinals[n++] = 10+i;
  }
  menu[strlen(menu)-1] = '\0';
}

void setupBaudrateMenu(char* menu) {
  char none[] = { "n.a." };
  sprintf(menu, loadMenu(P_MnuBaudrates, menuOrdinals),
    String(smuffConfig.serialBaudrates[0]).c_str(),
    CAN_USE_SERIAL1 ? String(smuffConfig.serialBaudrates[1]).c_str() : none,
    CAN_USE_SERIAL2 ? String(smuffConfig.serialBaudrates[2]).c_str() : none,
    CAN_USE_SERIAL3 ? String(smuffConfig.serialBaudrates[3]).c_str() : none);
}

void setupSteppersMenu(char *menu) {
  sprintf(menu, loadMenu((smuffConfig.revolverIsServo) ? P_MnuSteppersServo : P_MnuSteppers, menuOrdinals));
}

void setupTMCMenu(char* menu, uint8_t axis) {
  sprintf(menu, loadMenu(P_MnuTmc, menuOrdinals),
    translateTMCDriverMode(smuffConfig.stepperMode[axis]),
    smuffConfig.stepperStealth[axis] ? P_Yes : P_No,
    smuffConfig.stepperPower[axis],
    String(smuffConfig.stepperRSense[axis]).c_str(),
    smuffConfig.stepperMicrosteps[axis],
    smuffConfig.stepperStall[axis],
    smuffConfig.stepperCSmin[axis],
    smuffConfig.stepperCSmax[axis],
    smuffConfig.stepperCSdown[axis],
    smuffConfig.stepperAddr[axis],
    smuffConfig.stepperToff[axis],
    smuffConfig.stepperStopOnStall[axis] ? P_Yes : P_No,
    smuffConfig.stepperMaxStallCnt[axis]
  );
}

void setupServoMenu(char* menu) {
  sprintf(menu, loadMenu(P_MnuServo, menuOrdinals),
    smuffConfig.homeAfterFeed ? P_Yes : P_No,
    smuffConfig.resetBeforeFeed ? P_Yes : P_No,
    smuffConfig.revolverIsServo ? P_Yes : P_No,
    #if defined(MULTISERVO)
    servoPosClosed[toolSelected]-SERVO_CLOSED_OFS,
    servoPosClosed[toolSelected],
    #else
    smuffConfig.revolverOffPos,
    servoPosClosed[toolSelected] == 0 ? smuffConfig.revolverOnPos : servoPosClosed[toolSelected],
    #endif
    smuffConfig.servoCycles1,
    smuffConfig.servoCycles2
  );
}

void setupRevolverMenu(char* menu) {
  sprintf(menu, loadMenu(P_MnuRevolver, menuOrdinals),
    translateTMCDriverMode(smuffConfig.stepperMode[REVOLVER]),
    smuffConfig.invertDir[REVOLVER] ? P_Yes : P_No,
    smuffConfig.endstopTrg[REVOLVER] ? P_High : P_Low,
    smuffConfig.stepDelay[REVOLVER],
    smuffConfig.maxSpeed[REVOLVER],
    smuffConfig.accelSpeed[REVOLVER],
    smuffConfig.accelDist[REVOLVER],
    smuffConfig.firstRevolverOffset,
    smuffConfig.stepsPerRevolution,
    smuffConfig.homeAfterFeed ? P_Yes : P_No,
    smuffConfig.resetBeforeFeed ? P_Yes : P_No,
    smuffConfig.wiggleRevolver ? P_Yes : P_No,
    smuffConfig.revolverIsServo ? P_Yes : P_No,
    translateMS3State(smuffConfig.ms3config[REVOLVER])
    );
}

void setupFeederMenu(char* menu) {
  sprintf(menu, loadMenu(P_MnuFeeder, menuOrdinals),
    translateTMCDriverMode(smuffConfig.stepperMode[FEEDER]),
    smuffConfig.invertDir[FEEDER] ? P_Yes : P_No,
    smuffConfig.endstopTrg[FEEDER] ? P_High : P_Low,
    smuffConfig.stepDelay[FEEDER],
    smuffConfig.maxSpeed[FEEDER],
    smuffConfig.accelSpeed[FEEDER],
    smuffConfig.accelDist[FEEDER],
    smuffConfig.stepsPerMM[FEEDER],
    smuffConfig.enableChunks ? P_Yes : P_No,
    smuffConfig.feedChunks,
    String(smuffConfig.insertLength).c_str(),
    smuffConfig.insertSpeed,
    String(smuffConfig.reinforceLength).c_str(),
    smuffConfig.extControlFeeder ? P_Yes : P_No,
    smuffConfig.isSharedStepper ? P_Yes : P_No,
    translateMS3State(smuffConfig.ms3config[FEEDER]),
    smuffConfig.endstopTrg[3] ? P_High : P_Low,
    smuffConfig.useEndstop2 ? P_Yes : P_No,
    smuffConfig.wipeBeforeUnload ? P_Yes : P_No
  );
}

void setupSelectorMenu(char* menu) {
  sprintf(menu, loadMenu(P_MnuSelector, menuOrdinals),
    translateTMCDriverMode(smuffConfig.stepperMode[SELECTOR]),
    smuffConfig.invertDir[SELECTOR] ? P_Yes : P_No,
    smuffConfig.endstopTrg[SELECTOR] ? P_High : P_Low,
    smuffConfig.stepDelay[SELECTOR],
    smuffConfig.maxSpeed[SELECTOR],
    smuffConfig.accelSpeed[SELECTOR],
    smuffConfig.accelDist[SELECTOR],
    String(smuffConfig.firstToolOffset).c_str(),
    String(smuffConfig.stepsPerMM[SELECTOR]).c_str(),
    translateMS3State(smuffConfig.ms3config[SELECTOR])
  );

}

void setupDisplayMenu(char* menu) {
  sprintf(menu, loadMenu(P_MnuDisplay, menuOrdinals),
    smuffConfig.powerSaveTimeout,
    smuffConfig.lcdContrast,
    smuffConfig.encoderTickSound ? P_Yes : P_No,
    translateColor(smuffConfig.backlightColor, 0),
    translateColor(smuffConfig.toolColor, 1),
    smuffConfig.useIdleAnimation ? P_Yes : P_No,
    smuffConfig.animationBPM,
    smuffConfig.statusBPM
  );
}

void setupTestrunMenu(char* menu, uint8_t maxFiles) {
  char items[maxFiles*30];

  sprintf_P(menu, P_MenuItemBack);
  memset(items, 0, ArraySize(items));
  if(getFiles(PSTR("test/"), PSTR(".gcode"), maxFiles, true, items)) {
    strcat(menu, items);
  }
  //__debugS(PSTR("Test-Files:\n%s"), items);
}

void showMainMenu() {
  bool stopMenu = false;
  uint32_t startTime = millis();
  uint8_t current_selection = 0;
  char tmp[128];
  char _title[40];
  char _menu[700];

  while(!stopMenu) {
    sprintf_P(_title, P_TitleMainMenu);
    setupMainMenu(_menu);
    #if defined(CHECK_MENU)
    checkMenuSize(P_MnuMain0, _menu, ArraySize(_menu));
    #endif
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(_title, current_selection, _menu);
    uint8_t fnc = menuOrdinals[current_selection];
    //__debugS(PSTR("Cur now: %d -> %d"), current_selection, fnc);

    if(current_selection == 0)
      return;
    else {
      char* title = extractTitle(_menu, current_selection-1);
      bool enabled = steppers[SELECTOR].getEnabled();

      switch(fnc) {
        case 1:
          stopMenu = true;
          break;

        case 2:
          moveHome(SELECTOR);
          moveHome(REVOLVER, true, false);
          break;

        case 3:
          steppers[SELECTOR].setEnabled(!enabled);
          steppers[REVOLVER].setEnabled(!enabled);
          steppers[FEEDER].setEnabled(!enabled);
          if(smuffConfig.revolverIsServo) {
            setServoLid(SERVO_OPEN);
          }
          break;

        case 4:
          if(lidOpen)
            setServoLid(SERVO_CLOSED);
          else
            setServoLid(SERVO_OPEN);
          break;

        case 5:
          maintainTool();
          break;

        case 6:
          feederJammed = false;
          beep(2);
          sprintf_P(tmp, P_JamCleared);
          drawUserMessage(tmp);
          break;

        case 7:
          if(smuffConfig.prusaMMU2)
              loadFilamentPMMU2();
          else
              loadFilament();
          break;

        case 8:
          unloadFilament();
          break;

        case 9:
          loadFilament();
          break;

        case 10: // Wipe Nozzle
          G12("G12", "", 255);
          break;

        case 11: // Cut Filament
          cutFilament(false);
          break;

        case 13:
          showSwapMenu(title);
          break;

        case 14:
          showStatusInfoMenu(title);
          break;

        case 16:
          showSettingsMenu(title);
          current_selection = 1;
          break;

        case 18:
          showTestrunMenu(title);
          current_selection = 1;
          break;

        case 19:
          loadToSplitter(true);
          break;

        case 20:
          unloadFromSplitter(true);
          break;

      }
      startTime = millis();
    }
    debounceButton();
  }
}

void showTestrunMenu(char* menuTitle) {
  bool stopMenu = false;
  uint32_t startTime = millis();
  uint8_t current_selection = 0;
  char _menu[600];
  char* _file;

  while(!stopMenu) {
    setupTestrunMenu(_menu, 20);
    #if defined(CHECK_MENU)
    checkMenuSize(PSTR("Testrun"), _menu, ArraySize(_menu));
    #endif
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);

    switch (current_selection) {
      case 0:
        return;
      case 1:
        stopMenu = true;
        break;
      default:
        _file = extractFile(_menu, current_selection-1);
        char *p;
        while((p = strrchr(_file,' ')) != nullptr) {
          *p = 0;
        }
        //__debugS(PSTR("Selected file: >%s<"), _file);
        testRun(_file);
        break;
    }
  }
}

const char* translateColor(uint8_t color, uint8_t index) {
  char* colorNames[16];
  char tmp[80];
  sprintf_P(tmp, loadOptions(P_OptColors));
  splitStringLines(colorNames, (int)ArraySize(colorNames), tmp);
  if(color >= 0 && color < (int)ArraySize(colorNames))
    strcpy(selColorName[index], colorNames[color]);
  else sprintf_P(selColorName[index], P_Undefined);
  return selColorName[index];
}

const char* translatePanelDuePort(uint8_t port) {
  char* ports[3];
  char tmp[40];
  sprintf_P(tmp, loadOptions(P_OptPanelDue));
  splitStringLines(ports, (int)ArraySize(ports), tmp);
  if(port >= 0 && port < (int)ArraySize(ports))
    strcpy(duePort, ports[port]);
  else sprintf_P(duePort, P_Undefined);
  return duePort;
}

const char* translateTMCDriverMode(uint8_t mode) {
  char* modes[3];
  char tmp[40];
  sprintf_P(tmp, loadOptions(P_OptTmcModes));
  splitStringLines(modes, (int)ArraySize(modes), tmp);
  if(mode >= 0 && mode < (int)ArraySize(modes))
    strcpy(driverMode, modes[mode]);
  else sprintf_P(driverMode, P_Undefined);
  return driverMode;
}

const char* translateMS3State(uint8_t mode) {
  char* states[3];
  char tmp[40];
  sprintf_P(tmp, loadOptions(P_OptMS3States));
  splitStringLines(states, (int)ArraySize(states), tmp);
  if(mode >= 0 && mode < (int)ArraySize(states))
    strcpy(ms3State, states[mode]);
  else sprintf_P(ms3State, P_Undefined);
  return ms3State;
}

void selectPanelDuePort(char* menuTitle) {
  int val = smuffConfig.hasPanelDue;
  if(showInputDialog(menuTitle, P_PanelDuePort, &val, String(loadOptions(P_OptPanelDue)), nullptr, true))
    smuffConfig.hasPanelDue = val;
}

bool selectBaudrate(uint8_t port, char* menuTitle) {
  unsigned long val;
  char tmp[128];
  sprintf(tmp, loadOptions(P_OptBaudrates));
  if(port == 0) {
      val = smuffConfig.serialBaudrates[0];
      if(showInputDialog(menuTitle, P_Baud, &val, String(tmp))) {
        smuffConfig.serialBaudrates[0] = val;
        Serial.end();
        delay(500);
        Serial.begin(smuffConfig.serialBaudrates[0]);
      }
  }
  else if(port == 1) {
      if(CAN_USE_SERIAL1) {
        val = smuffConfig.serialBaudrates[1];
        if(showInputDialog(menuTitle, P_Baud, &val, String(tmp))) {
          smuffConfig.serialBaudrates[1] = val;
          Serial1.end();
          delay(500);
          Serial1.begin(smuffConfig.serialBaudrates[1]);
        }
      }
  }
  else if(port == 2) {
      if(CAN_USE_SERIAL2) {
        val = smuffConfig.serialBaudrates[2];
        if(showInputDialog(menuTitle, P_Baud, &val, String(tmp))) {
          smuffConfig.serialBaudrates[2] = val;
          Serial2.end();
          delay(500);
          Serial2.begin(smuffConfig.serialBaudrates[2]);
        }
      }
  }
  else {
      if(CAN_USE_SERIAL3) {
        val = smuffConfig.serialBaudrates[3];
        if(showInputDialog(menuTitle, P_Baud, &val, String(tmp))) {
          smuffConfig.serialBaudrates[3] = val;
          Serial3.end();
          delay(500);
          Serial3.begin(smuffConfig.serialBaudrates[3]);
        }
      }
  }
  return true;
}

bool selectBacklightColor(int color, char* menuTitle) {
  int val = color;
  char tmp[80];
  sprintf_P(tmp, loadOptions(P_OptColors));
  if(showInputDialog(menuTitle, P_Color, &val, String(tmp), setBacklightIndex, true)) {
    smuffConfig.backlightColor = val;
    //__debugS(PSTR("Backlight: %d"), val);
  }
  return true;
}

bool selectToolColor(int color, char* menuTitle) {
  int val = color;
  char tmp[80];
  sprintf_P(tmp, loadOptions(P_OptColors));
  if(showInputDialog(menuTitle, P_Color, &val, String(tmp), setToolColorIndex, true)) {
    smuffConfig.toolColor = val;
  }
  else {
    smuffConfig.toolColor = color;
  }
  return true;
}

void positionServoCallback(int val) {
  #if defined(MULTISERVO)
  setServoPos(toolSelected+10, val);
  #else
  setServoPos(SERVO_LID, val);
  #endif
}

void animationBpmCallback(int val) {
  #if defined(USE_FASTLED_TOOLS)
    for(int i=0; i< smuffConfig.toolCount*2; i++) {
      fadeToBlackBy(ledsTool, smuffConfig.toolCount, 100);
      int8_t pos = beatsin8(val, 0, smuffConfig.toolCount-1);
      ledsTool[pos] += CHSV(fastLedHue, 255, 200);
      FastLED.delay(25);
    }
  #endif
}

void showTMCMenu(char* menuTitle, uint8_t axis) {
  bool stopMenu = false;
  uint32_t startTime = millis();
  uint8_t current_selection = 0;
  bool bVal;
  int iVal;
  float fVal;
  char* title;
  char _menu[450];
  char tmp[50];

  while(!stopMenu) {
    setupTMCMenu(_menu, axis);
    #if defined(CHECK_MENU)
    checkMenuSize(P_MnuTmc, _menu, ArraySize(_menu));
    #endif
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);
    uint8_t fnc = menuOrdinals[current_selection];

    if(current_selection == 0)
      return;
    else {
      title = extractTitle(_menu, current_selection-1);
      switch(fnc) {
        case 1:
            stopMenu = true;
            break;

        case 2: // Mode
            iVal = smuffConfig.stepperMode[axis];
            sprintf_P(tmp, loadOptions(P_OptTmcModes));
            if(showInputDialog(title, P_DriverMode, &iVal, String(tmp), nullptr, true))
              smuffConfig.stepperMode[axis] = (uint8_t)iVal;
            break;

        case 3: // TMode
            bVal = smuffConfig.stepperStealth[axis];
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.stepperStealth[axis] = bVal;
            break;

        case 4: // Power
            iVal = smuffConfig.stepperPower[axis];
            if(showInputDialog(title, P_InMilliAmpere, &iVal, 0, MAX_POWER, nullptr, 10)) {
              smuffConfig.stepperPower[axis] = (uint16_t)iVal;
#ifdef HAS_TMC_SUPPORT
              if(drivers[axis] != nullptr)
                drivers[axis]->rms_current(smuffConfig.stepperPower[axis]);
#endif
            }
            break;

        case 5: // RSense
            fVal = smuffConfig.stepperRSense[axis];
            if(showInputDialog(title, P_InOhm, &fVal, 0, 1, nullptr, 0.01)) {
              smuffConfig.stepperRSense[axis] = fVal;
#ifdef HAS_TMC_SUPPORT
              if(drivers[axis] != nullptr)
                drivers[axis]->Rsense = fVal;
#endif
            }
            break;

        case 6: // Microsteps
            iVal = smuffConfig.stepperMicrosteps[axis];
            sprintf_P(tmp, loadOptions(P_OptMicrosteps));
            if(showInputDialog(title, P_Microsteps, &iVal, String(tmp), nullptr, false)) {
              smuffConfig.stepperMicrosteps[axis] = (uint16_t)iVal;
#ifdef HAS_TMC_SUPPORT
              if(drivers[axis] != nullptr)
                drivers[axis]->microsteps(smuffConfig.stepperMicrosteps[axis]);
#endif
            }
            break;

        case 7: // Stall
            iVal = smuffConfig.stepperStall[axis];
            if(showInputDialog(title, P_Threshold, &iVal, 0, 255)) {
              smuffConfig.stepperStall[axis] = (int8_t)iVal;
#ifdef HAS_TMC_SUPPORT
              if(drivers[axis] != nullptr)
                drivers[axis]->SGTHRS(smuffConfig.stepperStall[axis]);
#endif
            }
            break;

        case 8: // CoolStep min
            iVal = smuffConfig.stepperCSmin[axis];
            if(showInputDialog(title, P_Min, &iVal, 0, 15)) {
              smuffConfig.stepperCSmin[axis] = (int8_t)iVal;
#ifdef HAS_TMC_SUPPORT
              if(drivers[axis] != nullptr)
                drivers[axis]->semin(smuffConfig.stepperCSmin[axis]);
#endif
            }
            break;

        case 9: // CoolStep max
            iVal = smuffConfig.stepperCSmax[axis];
            if(showInputDialog(title, P_Max, &iVal, 0, 15)) {
              smuffConfig.stepperCSmax[axis] = (int8_t)iVal;
#ifdef HAS_TMC_SUPPORT
              if(drivers[axis] != nullptr)
                drivers[axis]->semax(smuffConfig.stepperCSmax[axis]);
#endif
            }
            break;

        case 10: // CoolStep down
            iVal = smuffConfig.stepperCSdown[axis];
            if(showInputDialog(title, P_Down, &iVal, 0, 15)) {
              smuffConfig.stepperCSdown[axis] = (int8_t)iVal;
#ifdef HAS_TMC_SUPPORT
              if(drivers[axis] != nullptr) {
                drivers[axis]->sedn(smuffConfig.stepperCSdown[axis]);
                drivers[axis]->seup(smuffConfig.stepperCSdown[axis]);
              }
#endif
            }
            break;

        case 11: // Driver Address
            iVal = smuffConfig.stepperAddr[axis];
            if(showInputDialog(title, P_Address, &iVal, 0, 3)) {
              smuffConfig.stepperAddr[axis] = (int8_t)iVal;
            }
            break;

        case 12: // TOff
            iVal = smuffConfig.stepperToff[axis];
            if(showInputDialog(title, P_Value, &iVal, 0, 15)) {
              smuffConfig.stepperToff[axis] = (int8_t)iVal;
#ifdef HAS_TMC_SUPPORT
              if(drivers[axis] != nullptr)
                drivers[axis]->toff(smuffConfig.stepperToff[axis]);
#endif
            }
            break;

        case 13: // Stop on stall
            bVal = smuffConfig.stepperStopOnStall[axis];
            if(showInputDialog(title, P_YesNo, &bVal)) {
              smuffConfig.stepperStopOnStall[axis] = bVal;
              steppers[axis].setStopOnStallDetected(bVal);
            }
            break;

        case 14: // Stall trigger count
            iVal = smuffConfig.stepperMaxStallCnt[axis];
            if(showInputDialog(title, P_Value, &iVal, 0, MAX_STALL_COUNT)) {
              smuffConfig.stepperMaxStallCnt[axis] = (int8_t)iVal;
              steppers[axis].setStallThreshold(smuffConfig.stepperMaxStallCnt[axis]);
            }
            break;
      }
      startTime = millis();
    }
  }
}

void showServoMenu(char* menuTitle) {
  bool stopMenu = false;
  uint32_t startTime = millis();
  uint8_t current_selection = 0;
  char* title;
  bool bVal;
  int iVal;
  uint8_t posForTool;
  char _menu[350];

  while(!stopMenu) {
    setupServoMenu(_menu);
    #if defined(CHECK_MENU)
    checkMenuSize(P_MnuServo, _menu, ArraySize(_menu));
    #endif
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);
    uint8_t fnc = menuOrdinals[current_selection];

    if(current_selection == 0)
      return;
    else {
      title = extractTitle(_menu, current_selection-1);
      switch(fnc) {
        case 1:
            stopMenu = true;
            break;

        case 2: // Home after feed
            bVal = smuffConfig.homeAfterFeed;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.homeAfterFeed = bVal;
            break;

        case 3: // Reset before feed
            bVal = smuffConfig.resetBeforeFeed;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.resetBeforeFeed = bVal;
            break;

        case 4: // Use Servo
            bVal = smuffConfig.revolverIsServo;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.revolverIsServo = bVal;
            break;

        case 5: // Servo open
            #if !defined(MULTISERVO)
            iVal = smuffConfig.revolverOffPos;
            if(showInputDialog(title, P_OpenPos, &iVal, 0, 180, positionServoCallback)) {
              smuffConfig.revolverOffPos = (uint8_t)iVal;
            }
            #endif
            break;

        case 6: // Servo closed
            #if defined(MULTISERVO)
            iVal = servoPosClosed[toolSelected];
            #else
            posForTool = servoPosClosed[toolSelected];
            iVal = posForTool == 0 ? smuffConfig.revolverOnPos : posForTool;
            #endif
            if(showInputDialog(title, P_ClosedPos, &iVal, 0, 180, positionServoCallback)) {
              #if defined(MULTISERVO)
              servoPosClosed[toolSelected] = (uint8_t)iVal;
              #else
              servoPosClosed[toolSelected] = (uint8_t)iVal;
              smuffConfig.revolverOnPos = (uint8_t)iVal;
              #endif
            }
            break;

        case 7: // Servo 1 cycles
            iVal = smuffConfig.servoCycles1;
            if(showInputDialog(title, P_ServoCycles, &iVal, 0, 50)) {
              smuffConfig.servoCycles1 = (uint8_t)iVal;
              servoWiper.setMaxCycles(smuffConfig.servoCycles1);
            }
            break;

        case 8: // Servo 2 cycles
            iVal = smuffConfig.servoCycles2;
            if(showInputDialog(title, P_ServoCycles, &iVal, 0, 50)) {
              smuffConfig.servoCycles2 = (uint8_t)iVal;
              servoLid.setMaxCycles(smuffConfig.servoCycles2);
            }
            break;
      }
      startTime = millis();
    }
  }
}

void showRevolverMenu(char* menuTitle) {
  bool stopMenu = false;
  uint32_t startTime = millis();
  uint8_t current_selection = 0;
  char* title;
  bool bVal;
  int iVal;
  uint16_t uiVal;
  char _menu[350];
  char tmp[50];

  while(!stopMenu) {
    setupRevolverMenu(_menu);
    #if defined(CHECK_MENU)
    checkMenuSize(P_MnuRevolver, _menu, ArraySize(_menu));
    #endif
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);
    uint8_t fnc = menuOrdinals[current_selection];

    if(current_selection == 0)
      return;
    else {
      title = extractTitle(_menu, current_selection-1);
      switch(fnc) {
        case 1:
            stopMenu = true;
            break;

        case 2: // TMC-Parameters
            showTMCMenu(title, REVOLVER);
            current_selection = 1;
            break;

        case 3: // Invert dir
            bVal = smuffConfig.invertDir[REVOLVER];
            if(showInputDialog(title, P_YesNo, &bVal)) {
              smuffConfig.invertDir[REVOLVER] = bVal;
              steppers[REVOLVER].setInvertDir(bVal);
            }
            break;

        case 4: // Endstop trigger
            iVal = smuffConfig.endstopTrg[REVOLVER];
            if(showInputDialog(title, P_TriggerOn, &iVal, 0, 1)) {
              smuffConfig.endstopTrg[REVOLVER] = (uint8_t)iVal;
              steppers[REVOLVER].setEndstopState(smuffConfig.endstopTrg[REVOLVER]);
            }
            break;

        case 5: // Step Delay
            iVal = smuffConfig.stepDelay[REVOLVER];
            if(showInputDialog(title, P_InMicroseconds, &iVal, 0, 100)) {
              smuffConfig.stepDelay[REVOLVER] = (uint8_t)iVal;
            }
            break;

        case 6: // Max. Speed
            uiVal = smuffConfig.maxSpeed[REVOLVER];
            if(showInputDialog(title, smuffConfig.speedsInMMS ?  P_InMMS : P_InTicks, &uiVal, mmsMin, mmsMax, nullptr, speedIncrement)) {
              validateSpeed(iVal, &smuffConfig.accelSpeed[REVOLVER], 2);
              smuffConfig.maxSpeed[REVOLVER] = uiVal;
              steppers[REVOLVER].setMaxSpeed(translateSpeed(uiVal, REVOLVER));
            }
            break;

        case 7: // Acceleration
            uiVal = smuffConfig.accelSpeed[REVOLVER];
            if(showInputDialog(title, smuffConfig.speedsInMMS ?  P_InMMS : P_InTicks, &uiVal, mmsMin, mmsMax, nullptr, speedIncrement)) {
              smuffConfig.accelSpeed[REVOLVER] = uiVal;
              steppers[REVOLVER].setAcceleration(translateSpeed(uiVal, REVOLVER));
            }
            break;

        case 8: // Acceleration Distance
            iVal = smuffConfig.accelDist[REVOLVER];
            if(showInputDialog(title, P_InMillimeter, &iVal, 1, 200, nullptr, 1)) {
              smuffConfig.accelDist[REVOLVER] = (uint8_t)iVal;
              steppers[REVOLVER].setAccelDistance(smuffConfig.accelDist[REVOLVER]);
            }
            break;

        case 9: // Offset
            changeOffset(REVOLVER);
            break;

        case 10: // Steps Per Rev
            iVal = smuffConfig.stepsPerRevolution;
            if(showInputDialog(title, P_InSteps, &iVal, 1, 20000, nullptr, 50)) {
              smuffConfig.stepsPerRevolution = iVal;
              steppers[REVOLVER].setMaxStepCount(iVal);
              steppers[REVOLVER].setStepsPerDegree(iVal/360);
            }
            break;

        case 11: // Home after feed
            bVal = smuffConfig.homeAfterFeed;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.homeAfterFeed = bVal;
            break;

        case 12: // Reset before feed
            bVal = smuffConfig.resetBeforeFeed;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.resetBeforeFeed = bVal;
            break;

        case 13: // Wiggle
            bVal = smuffConfig.wiggleRevolver;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.wiggleRevolver = bVal;
            break;

        case 14: // Use Servo
            bVal = smuffConfig.revolverIsServo;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.revolverIsServo = bVal;
            break;

        case 15: // MS3 Pin State
            iVal = smuffConfig.ms3config[REVOLVER];
            sprintf_P(tmp, loadOptions(P_OptMS3States));
            if(showInputDialog(title, P_MS3State, &iVal, String(tmp), nullptr, true))
              smuffConfig.ms3config[REVOLVER] = (int8_t)iVal;
            break;
      }
      startTime = millis();
    }
  }
}

void showSelectorMenu(char* menuTitle) {
  bool stopMenu = false;
  uint32_t startTime = millis();
  uint8_t current_selection = 0;
  char* title;
  bool bVal;
  int iVal;
  uint16_t uiVal;
  char _menu[350];
  char tmp[50];

  while(!stopMenu) {
    setupSelectorMenu(_menu);
    #if defined(CHECK_MENU)
    checkMenuSize(P_MnuSelector, _menu, ArraySize(_menu));
    #endif
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);
    uint8_t fnc = menuOrdinals[current_selection];

    if(current_selection == 0)
      return;
    else {
      title = extractTitle(_menu, current_selection-1);
      switch(fnc) {
        case 1:
            stopMenu = true;
            break;

        case 2: // TMC-Paramters
            showTMCMenu(title, SELECTOR);
            current_selection = 1;
            break;

        case 3: // Invert dir
            bVal = smuffConfig.invertDir[SELECTOR];
            if(showInputDialog(title, P_YesNo, &bVal)) {
              smuffConfig.invertDir[SELECTOR] = bVal;
              steppers[SELECTOR].setInvertDir(bVal);
            }
            break;

        case 4: // Endstop trigger
            iVal = smuffConfig.endstopTrg[SELECTOR];
            if(showInputDialog(title, P_TriggerOn, &iVal, 0, 1)) {
              smuffConfig.endstopTrg[SELECTOR] = (uint8_t)iVal;
              steppers[SELECTOR].setEndstopState(smuffConfig.endstopTrg[SELECTOR]);
            }
            break;

        case 5: // Step Delay
            iVal = smuffConfig.stepDelay[SELECTOR];
            if(showInputDialog(title, P_InMicroseconds, &iVal, 0, 100)) {
              smuffConfig.stepDelay[SELECTOR] = (uint8_t)iVal;
            }
            break;

        case 6: // Max. Speed
            uiVal = smuffConfig.maxSpeed[SELECTOR];
            if(showInputDialog(title, smuffConfig.speedsInMMS ?  P_InMMS : P_InTicks, &uiVal, mmsMin, mmsMax, nullptr, speedIncrement)) {
              validateSpeed(iVal, &smuffConfig.accelSpeed[SELECTOR], 5);
              smuffConfig.maxSpeed[SELECTOR] = uiVal;
              steppers[SELECTOR].setMaxSpeed(translateSpeed(uiVal, SELECTOR));
            }
            break;

        case 7: // Acceleration
            uiVal = smuffConfig.accelSpeed[SELECTOR];
            if(showInputDialog(title, smuffConfig.speedsInMMS ?  P_InMMS : P_InTicks, &uiVal, mmsMin, mmsMax, nullptr, speedIncrement)) {
              smuffConfig.accelSpeed[SELECTOR] = uiVal;
              steppers[SELECTOR].setAcceleration(translateSpeed(uiVal, SELECTOR));
            }
            break;

        case 8: // Acceleration Distance
            iVal = smuffConfig.accelDist[SELECTOR];
            if(showInputDialog(title, P_InMillimeter, &iVal, 1, 200, nullptr, 1)) {
              smuffConfig.accelDist[SELECTOR] = (uint8_t)iVal;
              steppers[SELECTOR].setAccelDistance(smuffConfig.accelDist[SELECTOR]);
            }
            break;

        case 9: // Offset
            changeOffset(SELECTOR);
            break;

        case 10: // Steps Per MM
            iVal = smuffConfig.stepsPerMM[SELECTOR];
            if(showInputDialog(title, P_InSteps, &iVal, 1, 10000)) {
              smuffConfig.stepsPerMM[SELECTOR] = (uint16_t)iVal;
              steppers[SELECTOR].setStepsPerMM(smuffConfig.stepsPerMM[SELECTOR]);
            }
            break;

        case 11: // MS3 Pin State
            iVal = smuffConfig.ms3config[SELECTOR];
            sprintf_P(tmp, loadOptions(P_OptMS3States));
            if(showInputDialog(title, P_MS3State, &iVal, String(tmp), nullptr, true))
              smuffConfig.ms3config[SELECTOR] = (int8_t)iVal;
            break;
      }
      startTime = millis();
    }
  }
}

void showFeederMenu(char* menuTitle) {
  bool stopMenu = false;
  uint32_t startTime = millis();
  uint8_t current_selection = 0;
  char* title;
  bool bVal;
  int iVal;
  uint16_t uiVal;
  float fVal;
  char _menu[700];
  char tmp[50];

  while(!stopMenu) {
    setupFeederMenu(_menu);
    #if defined(CHECK_MENU)
    checkMenuSize(P_MnuFeeder, _menu, ArraySize(_menu));
    #endif
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);
    uint8_t fnc = menuOrdinals[current_selection];

    if(current_selection == 0)
      return;
    else {
      title = extractTitle(_menu, current_selection-1);
      switch(fnc) {
        case 1:
            stopMenu = true;
            break;

        case 2: // TMC-Paramters
            showTMCMenu(title, FEEDER);
            current_selection = 1;
            break;

        case 3: // Invert dir
            bVal = smuffConfig.invertDir[FEEDER];
            if(showInputDialog(title, P_YesNo, &bVal)) {
              smuffConfig.invertDir[FEEDER] = bVal;
              steppers[FEEDER].setInvertDir(bVal);
            }
            break;

        case 4: // Endstop trigger
            iVal = smuffConfig.endstopTrg[FEEDER];
            if(showInputDialog(title, P_TriggerOn, &iVal, 0, 1)) {
              smuffConfig.endstopTrg[FEEDER] = (uint8_t)iVal;
              steppers[FEEDER].setEndstopState(smuffConfig.endstopTrg[FEEDER]);
            }
            break;

        case 5: // Step Delay
            iVal = smuffConfig.stepDelay[FEEDER];
            if(showInputDialog(title, P_InMicroseconds, &iVal, 0, 100)) {
              smuffConfig.stepDelay[FEEDER] = (uint8_t)iVal;
            }
            break;

        case 6: // Max. Speed
            uiVal = smuffConfig.maxSpeed[FEEDER];
            if(showInputDialog(title, smuffConfig.speedsInMMS ?  P_InMMS : P_InTicks, &uiVal, mmsMin, mmsMax, nullptr, speedIncrement)) {
              validateSpeed(uiVal, &smuffConfig.accelSpeed[FEEDER], 5);
              validateSpeed(uiVal, &smuffConfig.insertSpeed, 10);
              smuffConfig.maxSpeed[FEEDER] = uiVal;
              steppers[FEEDER].setMaxSpeed(translateSpeed(uiVal, FEEDER));
            }
            break;

        case 7: // Acceleration
            uiVal = smuffConfig.accelSpeed[FEEDER];
            if(showInputDialog(title, smuffConfig.speedsInMMS ?  P_InMMS : P_InTicks, &uiVal, mmsMin, mmsMax, nullptr, speedIncrement)) {
              smuffConfig.accelSpeed[FEEDER] = uiVal;
              steppers[FEEDER].setAcceleration(translateSpeed(uiVal, FEEDER));
            }
            break;

        case 8: // Acceleration Distance
            iVal = smuffConfig.accelDist[FEEDER];
            if(showInputDialog(title, P_InMillimeter, &iVal, 1, 200, nullptr, 1)) {
              smuffConfig.accelDist[FEEDER] = (uint8_t)iVal;
              steppers[FEEDER].setAccelDistance(smuffConfig.accelDist[FEEDER]);
            }
            break;

        case 9: // Steps Per MM
            iVal = smuffConfig.stepsPerMM[FEEDER];
            if(showInputDialog(title, P_InSteps, &iVal, 1, 10000)) {
              smuffConfig.stepsPerMM[FEEDER] = (uint16_t)iVal;
              steppers[FEEDER].setStepsPerMM(smuffConfig.stepsPerMM[FEEDER]);
            }
            break;

        case 10: // Enable chunks
            bVal = smuffConfig.enableChunks;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.enableChunks = bVal;
            break;

        case 11: // Feed chunks
            iVal = smuffConfig.feedChunks;
            if(showInputDialog(title, P_NoOfChunks, &iVal, 0, 100))
              smuffConfig.feedChunks = (uint8_t)iVal;
            if(iVal == 0)
                smuffConfig.enableChunks = false;
            break;

        case 12: // Insert length
            fVal = smuffConfig.insertLength;
            if(showInputDialog(title, P_InMillimeter, &fVal, 1, smuffConfig.selectorDistance))
              smuffConfig.insertLength = fVal;
            break;

        case 13: // Insert Speed
            uiVal = smuffConfig.insertSpeed;
            if(showInputDialog(title, smuffConfig.speedsInMMS ?  P_InMMS : P_InTicks, &uiVal, mmsMin, mmsMax, nullptr, speedIncrement)) {
              smuffConfig.insertSpeed = uiVal;
            }
            break;

        case 14: // Reinforce length
            fVal = smuffConfig.reinforceLength;
            if(showInputDialog(title, P_InMillimeter, &fVal, 0, 10))
              smuffConfig.reinforceLength = fVal;
            break;

        case 15: // Ext. Feeder Ctl
            bVal = smuffConfig.extControlFeeder;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.extControlFeeder = bVal;
            break;

        case 16: // Shared Stepper
            bVal = smuffConfig.isSharedStepper;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.isSharedStepper = bVal;
            break;

        case 17: // MS3 Pin State
            iVal = smuffConfig.ms3config[FEEDER];
            sprintf_P(tmp, loadOptions(P_OptMS3States));
            if(showInputDialog(title, P_MS3State, &iVal, String(tmp), nullptr, true))
              smuffConfig.ms3config[FEEDER] = (int8_t)iVal;
            break;

        case 18: // Endstop2 trigger
            iVal = smuffConfig.endstopTrg[3];
            if(showInputDialog(title, P_TriggerOn, &iVal, 0, 1)) {
              smuffConfig.endstopTrg[3] = (uint8_t)iVal;
              steppers[FEEDER].setEndstopState(smuffConfig.endstopTrg[3], 2);
            }
            break;

        case 19: // Use Endstop2
            bVal = smuffConfig.useEndstop2;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.useEndstop2 = bVal;
            break;

        case 20: // Auto Wipe
            bVal = smuffConfig.wipeBeforeUnload;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.wipeBeforeUnload = bVal;
            break;

      }
      startTime = millis();
    }
  }
}

void showSteppersMenu(char* menuTitle) {
  bool stopMenu = false;
  uint32_t startTime = millis();
  uint8_t current_selection = 0;
  char* title;
  char _menu[128];

  while(!stopMenu) {
    setupSteppersMenu(_menu);
    #if defined(CHECK_MENU)
    checkMenuSize(P_MnuSteppers, _menu, ArraySize(_menu));
    #endif
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);
    uint8_t fnc = menuOrdinals[current_selection];

    if(current_selection == 0)
      return;
    else {
      title = extractTitle(_menu, current_selection-1);
      switch(fnc) {
        case 1:
            stopMenu = true;
            break;

        case 2: // Selector
            showSelectorMenu(title);
            current_selection = 1;
            break;

        case 3: // Feeder
            showFeederMenu(title);
            current_selection = 1;
            break;

        case 4: // Revolver
            showRevolverMenu(title);
            current_selection = 1;
            break;

        case 5: // Servo
            showServoMenu(title);
            current_selection = 1;
            break;
      }
      startTime = millis();
    }
  }
}

void showDisplayMenu(char* menuTitle) {
  bool stopMenu = false;
  uint32_t startTime = millis();
  uint8_t current_selection = 0;
  char* title;
  char _menu[180];
  int iVal;
  bool bVal;

  while(!stopMenu) {
    setupDisplayMenu(_menu);
    #if defined(CHECK_MENU)
    checkMenuSize(P_MnuDisplay, _menu, ArraySize(_menu));
    #endif
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);
    uint8_t fnc = menuOrdinals[current_selection];

    if(current_selection == 0)
      return;
    else {
      title = extractTitle(_menu, current_selection-1);
      switch(fnc) {
        case 1:
            stopMenu = true;
            break;

        case 2: // Power Save Timeout
            iVal = smuffConfig.powerSaveTimeout;
            if(showInputDialog(title, P_InSeconds, &iVal, 0, 480))
              smuffConfig.powerSaveTimeout = (uint16_t)iVal;
            break;

        case 3: // LCD Contrast
            iVal = smuffConfig.lcdContrast;
            if(showInputDialog(title, P_InValue, &iVal, MIN_CONTRAST, MAX_CONTRAST))
              smuffConfig.lcdContrast = (uint8_t)iVal;
            break;

        case 4: // Encoder Ticks
            bVal = smuffConfig.encoderTickSound;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.encoderTickSound = bVal;
            break;

        case 5: // Backlight Color
            selectBacklightColor(smuffConfig.backlightColor, title);
            break;

        case 6: // Tool Color
            selectToolColor(smuffConfig.toolColor, title);
            break;

        case 7: // Idle Animation
            bVal = smuffConfig.useIdleAnimation;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.useIdleAnimation = bVal;
            break;

        case 8: // Animation BPM
            iVal = smuffConfig.animationBPM;
            if(showInputDialog(title, P_InBPM, &iVal, 1, 255))
              smuffConfig.animationBPM = (uint8_t)iVal;
            break;

        case 9: // Status BPM
            iVal = smuffConfig.statusBPM;
            if(showInputDialog(title, P_InBPM, &iVal, 1, 255))
              smuffConfig.statusBPM = (uint8_t)iVal;
            break;
      }
      startTime = millis();
    }
  }
}

void saveSettings() {
  bool stat = false;
  if(writeConfig()) {
    if(writeTmcConfig()){
      if(writeServoMapping()) {
        if(writeMaterials()) {
          stat = true;
        }
      }
    }
  }
  char msg[60];
  if(stat) {
    beep(1);
    sprintf_P(msg, P_ConfigWriteSuccess);
    settingsChanged = false;
  }
  else {
    beep(3);
    sprintf_P(msg, P_ConfigWriteFail);
  }
  drawUserMessage(msg);
  delay(2000);
}

void checkSaveSettings() {
  userBeep();
  uint8_t button = showDialog(P_TitleWarning, P_SettingsChanged, P_AskSave, P_YesNoButtons);
  sendStates();  // send states once
  if (button == 1) {
    saveSettings();
  }
  settingsChanged = false;
}

void showSettingsMenu(char* menuTitle) {
  bool stopMenu = false;
  uint32_t startTime = millis();
  uint8_t current_selection = 0;
  float fVal;
  int iVal;
  char *title;
  char _menu[300];

  while(!stopMenu) {
    setupSettingsMenu(_menu);
    #if defined(CHECK_MENU)
    checkMenuSize(P_MnuSettings, _menu, ArraySize(_menu));
    #endif
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);
    uint8_t fnc = menuOrdinals[current_selection];

    if(current_selection == 0) {
      if(settingsChanged) {
        checkSaveSettings();
      }
      return;
    }
    else {
      title = extractTitle(_menu, current_selection-1);
      switch(fnc) {
        case 1:
            if(settingsChanged) {
              checkSaveSettings();
            }
            stopMenu = true;
            break;

        case 2: // Tool count
            iVal = smuffConfig.toolCount;
            if(showInputDialog(title, P_ToolCount, &iVal, MIN_TOOLS, MAX_TOOLS))
              smuffConfig.toolCount = (uint8_t)iVal;
            break;

        case 3: // Bowden length
            fVal = smuffConfig.bowdenLength;
            if(showInputDialog(title, P_InMillimeter, &fVal, 20, 2000))
            smuffConfig.bowdenLength = fVal;
            break;

        case 4: // Selector distance
            fVal = smuffConfig.selectorDistance;
            if(showInputDialog(title, P_InMillimeter, &fVal, 10, 150))
              smuffConfig.selectorDistance = fVal;
            break;

        case 6: // Options
            showOptionsMenu(title);
            current_selection = 1;
            break;

        case 7: // Baudrates
            showBaudratesMenu(title);
            current_selection = 1;
            break;

        case 8: // Steppers
            showSteppersMenu(title);
            current_selection = 1;
            break;

        case 9: // Display
            showDisplayMenu(title);
            current_selection = 1;
            break;

        case 10: // Purge Control
            showPurgeMenu(title);
            current_selection = 1;
            break;

        case 12: // Save To SD-Card
            saveSettings();
            current_selection = 1;
            break;
      }
      startTime = millis();
    }
  }
}

void setLiveFanSpeed(int val) {
  #if defined (__STM32F1__)
  fan.setFanSpeed(val);
  #elif defined (__ESP32__)
  ledcWrite(FAN_PIN, map(val, 0, 100, 0, 255));
  #else
  analogWrite(FAN_PIN, map(val, 0, 100, 0, 255));
  #endif
}

void showOptionsMenu(char* menuTitle) {
  bool stopMenu = false;
  uint32_t startTime = millis();
  uint8_t current_selection = 0;
  int iVal;
  float fVal;
  bool bVal;
  char *title;
  char _menu[350];

  while(!stopMenu) {
    setupOptionsMenu(_menu);
    #if defined(CHECK_MENU)
    checkMenuSize(P_MnuOptions, _menu, ArraySize(_menu));
    #endif
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);
    uint8_t fnc = menuOrdinals[current_selection];

    if(current_selection == 0)
      return;
    else {
      title = extractTitle(_menu, current_selection-1);
      switch(fnc) {
        case 1:
            stopMenu = true;
            break;

        case 2: // Menu auto close
            iVal = smuffConfig.menuAutoClose;
            if(showInputDialog(title, P_InSeconds, &iVal, 0, 300))
              smuffConfig.menuAutoClose = iVal;
            break;

        case 3: // Fan speed
            iVal = smuffConfig.fanSpeed;
            if(showInputDialog(title, P_InPercent, &iVal, 0, 100, setLiveFanSpeed)) {
              smuffConfig.fanSpeed = iVal;
            }
            break;

        case 4: // Prusa Emulation
            bVal = smuffConfig.prusaMMU2;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.prusaMMU2 = bVal;
            break;

        case 5: // Send Status Info
            bVal = smuffConfig.sendPeriodicalStats;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.sendPeriodicalStats = bVal;
            break;

        case 6: // Duet Laser Sensor
            bVal = smuffConfig.useDuetLaser;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.useDuetLaser = bVal;
            break;

        case 7: // PanelDue
            selectPanelDuePort(title);
            break;

        case 8: // Speeds in MMS
            bVal = smuffConfig.speedsInMMS;
            if(showInputDialog(title, P_YesNo, &bVal)) {
              smuffConfig.speedsInMMS = bVal;
              if(bVal) {
                mmsMax = MAX_MMS;
                speedIncrement = INC_MMS;
              }
              else {
                mmsMax = MAX_TICKS;
                speedIncrement = INC_TICKS;
              }
            }
            break;

        case 9: // Servo Min PWM
            iVal = smuffConfig.servoMinPwm;
            if(showInputDialog(title, P_InMilliseconds, &iVal, 400, 1000, nullptr, 20)) {
              smuffConfig.servoMinPwm = iVal;
              servoWiper.setPulseWidthMin(iVal);
              servoLid.setPulseWidthMin(iVal);
            }
            break;

        case 10: // Servo Max PWM
            iVal = smuffConfig.servoMaxPwm;
            if(showInputDialog(title, P_InMilliseconds, &iVal, 1000, 3000, nullptr, 20)) {
              smuffConfig.servoMaxPwm = iVal;
              servoWiper.setPulseWidthMax(iVal);
              servoLid.setPulseWidthMax(iVal);
            }
            break;

        case 11: // Use Cutter
            bVal = smuffConfig.useCutter;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.useCutter = bVal;
            break;

        case 12: // Cutter Open
            iVal = smuffConfig.cutterOpen;
            if(showInputDialog(title, P_OpenPos, &iVal, 0, 180, nullptr, 1)) {
              smuffConfig.cutterOpen = iVal;
            }
            break;

        case 13: // Cutter Close
            iVal = smuffConfig.cutterClose;
            if(showInputDialog(title, P_ClosedPos, &iVal, 0, 180, nullptr, 1)) {
              smuffConfig.cutterClose = iVal;
            }
            break;

        case 14: // Invert Relay
            bVal = smuffConfig.invertRelay;
            if(showInputDialog(title, P_YesNo, &bVal)) {
              smuffConfig.invertRelay = bVal;
              switchFeederStepper(smuffConfig.externalStepper);
            }
            break;

        case 15: // Use Splitter
            bVal = smuffConfig.useSplitter;
            if(showInputDialog(title, P_YesNo, &bVal)) {
              smuffConfig.useSplitter = bVal;
            }
            break;

        case 16: // Splitter Distance
            fVal = smuffConfig.splitterDist;
            if(showInputDialog(title, P_InMillimeter, &fVal, 0, 200, nullptr, 1)) {
              smuffConfig.splitterDist = fVal;
            }
            break;

        case 17: // Use DDE
            // does nothing since it's defined during compilation
            break;

        case 18: // DDE Distance
            fVal = smuffConfig.ddeDist;
            if(showInputDialog(title, P_InMillimeter, &fVal, 0, 400, nullptr, 1)) {
              smuffConfig.ddeDist = fVal;
            }
            break;

        case 19: // Purge DDE
            bVal = smuffConfig.purgeDDE;
            if(showInputDialog(title, P_YesNo, &bVal)) {
              smuffConfig.purgeDDE = bVal;
            }
            break;
      }
      startTime = millis();
    }
  }
}

void showPurgeMenu(char* menuTitle) {
  bool stopMenu = false;
  uint32 startTime = millis();
  uint8_t current_selection = 0;
  float fVal;
  int iVal;
  bool bVal;
  char *title;
  char msg[128];
  char _menu[300];

  while(!stopMenu) {
    setupPurgeMenu(_menu);
    #if defined(CHECK_MENU)
    checkMenuSize(P_MnuPurge, _menu, ArraySize(_menu));
    #endif
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);
    uint8_t fnc = menuOrdinals[current_selection];

    //__debugS(PSTR("Ordinal FNC: %d"), fnc);

    if(current_selection == 0)
      return;
    else {
      title = extractTitle(_menu, current_selection-1);
      switch(fnc) {
        case 1:
            stopMenu = true;
            break;
        case 2: // Use Purge
            bVal = smuffConfig.usePurge;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.usePurge = bVal;
            break;
        case 3: // Purge Speed
            iVal = smuffConfig.purgeSpeed;
            if(showInputDialog(title, smuffConfig.speedsInMMS ?  P_InMMS : P_InTicks, &iVal, mmsMin, mmsMax, nullptr, speedIncrement)) {
              smuffConfig.purgeSpeed = (uint16_t)iVal;
            }
            break;

        case 4: // Purge length
            fVal = smuffConfig.purgeLength;
            if(showInputDialog(title, P_InMillimeter, &fVal, 1, 400))
              smuffConfig.purgeLength = fVal;
            break;

        case 5: // Cutter length
            fVal = smuffConfig.unloadRetract;
            if(showInputDialog(title, P_InMillimeter, &fVal, 0, 150))
              smuffConfig.unloadRetract = fVal;
            break;

        default:
          if(fnc >=10 && fnc <= 10+MAX_TOOLS) {
            uint8_t ndx = swapTools[fnc-10];
            iVal = smuffConfig.purges[ndx];
            if(showInputDialog(title, P_InPercent, &iVal, 50, 500, nullptr, 1)) {
              smuffConfig.purges[ndx] = (uint16_t)iVal;
            }
          }
      }
    }
  }
}

void drawSwapTool(uint8_t from, uint8_t with) {
  char tmp[256];
  sprintf_P(tmp, P_SwapToolDialog, from, with);
  drawUserMessage(String(tmp));
}

uint8_t swapTool(uint8_t index) {
  uint8_t ndx = 0;
  int16_t turn;
  uint8_t btn;
  bool isHeld, isClicked;

  debounceButton();
  delay(250);

  drawSwapTool(index, ndx);

  while(1) {
    getInput(&turn, &btn, &isHeld, &isClicked);
    if(isHeld || isClicked) {
      break;
    }
    if(turn == 0)
      continue;
    ndx += turn;

    if(ndx >= smuffConfig.toolCount)
      ndx = smuffConfig.toolCount-1;
    if(ndx == 255)
      ndx = smuffConfig.toolCount-1;
    drawSwapTool(index, ndx);
  }
  return ndx;
}

void showSwapMenu(char* menuTitle) {
  bool stopMenu = false;
  uint32_t startTime = millis();
  uint8_t current_selection = 0;
  char _menu[128];

  while(!stopMenu) {
    setupSwapMenu(_menu);
    #if defined(CHECK_MENU)
    checkMenuSize(PSTR("Swap"), _menu, ArraySize(_menu));
    #endif
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);

    if(current_selection == 0)
      return;
    else if(current_selection == 1) {
      stopMenu = true;
    }
    else if(current_selection == 2) {
      for(uint8_t i=0; i < MAX_TOOLS; i++) {
        swapTools[i] = i;
      }
    }
    else {
      uint8_t tool = swapTool(current_selection-3);
      uint8_t tmp = swapTools[current_selection-3];
      swapTools[current_selection-3] = tool;
      swapTools[tool] = tmp;
      current_selection = 1;
    }
  }
  saveStore();
}

void showBaudratesMenu(char* menuTitle) {
  bool stopMenu = false;
  uint32_t startTime = millis();
  uint8_t current_selection = 0;
  char* title;
  char _menu[300];

  while(!stopMenu) {
    setupBaudrateMenu(_menu);
    #if defined(CHECK_MENU)
    checkMenuSize(P_MnuBaudrates, _menu, ArraySize(_menu));
    #endif
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);
    uint8_t fnc = menuOrdinals[current_selection];

    if(current_selection == 0)
      return;
    else {
      title = extractTitle(_menu, current_selection-1);
      switch(fnc) {
        case 1:
          stopMenu = true;
          break;

        case 2:
          selectBaudrate(0, title);
          current_selection = 1;
          break;

        case 3:
          selectBaudrate(1, title);
          current_selection = 1;
          break;

        case 4:
          selectBaudrate(2, title);
          current_selection = 1;
          break;

        case 5:
          selectBaudrate(3, title);
          current_selection = 1;
          break;
      }
      startTime = millis();
    }
  }
}

void showStatusInfoMenu(char* menuTitle) {
  bool stopMenu = false;
  uint32_t startTime = millis();
  uint8_t current_selection = 0;
  char _menu[120];

  while(!stopMenu) {
    setupStatusInfoMenu(_menu);
    #if defined(CHECK_MENU)
    checkMenuSize(P_MnuStatus, _menu, ArraySize(_menu));
    #endif
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);
    uint8_t fnc = menuOrdinals[current_selection];

    if(current_selection == 0)
      return;
    else {
      switch(fnc) {
        case 1:
          stopMenu = true;
          break;

        case 2:
          if(smuffConfig.useDuetLaser)
            showDuetLS();
          else {
            debounceButton();
            drawUserMessage(P_DuetLSDisabled);
            delay(3000);
          }
          break;

        case 3:
          showTMCStatus(SELECTOR);
          break;

        case 4:
          showTMCStatus(REVOLVER);
          break;

        case 5:
          showTMCStatus(FEEDER);
          break;
      }
      startTime = millis();
    }
  }
}

void changeOffset(uint8_t index) {
  uint16_t steps = smuffConfig.stepsPerRevolution/360;
  float stepsF = 0.1f;
  long pos;
  float posF;
  int16_t turn;
  uint8_t btn;
  bool stat = true, isHeld, isClicked;

  debounceButton();

  moveHome(index);
  if(index == REVOLVER) {
    prepSteppingRel(REVOLVER, smuffConfig.firstRevolverOffset, true);
  }
  if(index == SELECTOR) {
    prepSteppingRelMillimeter(SELECTOR, smuffConfig.firstToolOffset, true);
  }
  runAndWait(index);
  pos = steppers[index].getStepPosition();
  posF = steppers[index].getStepPositionMM();

  uint16_t curSpeed = steppers[index].getMaxSpeed();
  steppers[index].setMaxSpeed(steppers[index].getAcceleration());
  encoder.setAccelerationEnabled(true);

  if(index == REVOLVER) {
    drawValue(steppers[index].getDescriptor(), P_InSteps, String(steppers[index].getStepPosition()));
  }
  if(index == SELECTOR) {
    drawValue(steppers[index].getDescriptor(), P_InMillimeter, String(steppers[index].getStepPositionMM()));
  }

  while(1) {
    getEncoderButton(&turn, &btn, &isHeld, &isClicked);
    if(isHeld || isClicked) {
      stat = isHeld ? false : true;
      break;
    }
    if(turn != 0) {
      if(index == REVOLVER) {
        pos += (steps*turn);
        prepSteppingAbs(REVOLVER, pos, true);
      }
      else if(index == SELECTOR) {
        posF += (stepsF*turn);
        prepSteppingAbsMillimeter(SELECTOR, posF, true);
      }
      //__debugS(PSTR("Turn: %d  Pos: %d   PosF: %s"), turn, pos, String(posF).c_str());
      runAndWait(index);
      if(index == REVOLVER) {
        drawValue(steppers[index].getDescriptor(), P_InSteps, String(steppers[index].getStepPosition()));
      }
      if(index == SELECTOR) {
        drawValue(steppers[index].getDescriptor(), P_InMillimeter, String(steppers[index].getStepPositionMM()));
      }
    }
  }
  if(index == REVOLVER) {
    if(stat)
      smuffConfig.firstRevolverOffset = pos;
    else
      moveHome(index);
  }
  if(index == SELECTOR) {
    if(stat)
      smuffConfig.firstToolOffset = posF;
    else
      moveHome(index);
  }
  steppers[index].setMaxSpeed(curSpeed);
  encoder.setAccelerationEnabled(false);
}


void showToolsMenu() {

  bool stopMenu = false;
  uint32_t startTime = millis();
  uint8_t current_selection = 0;
  char _title[60];
  char _menu[128];
  char _tmp[40];

  while(!stopMenu) {
    sprintf_P(_title, P_TitleToolsMenu);
    setupToolsMenu(_menu);
    #if defined(CHECK_MENU)
    checkMenuSize(PSTR("Tools"), _menu, ArraySize(_menu));
    #endif
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    uint8_t startPos = toolSelected == -1 ? 0 : toolSelected+1;
    current_selection = display.userInterfaceSelectionList(_title, startPos, _menu);

    if(current_selection <= 1)
      stopMenu = true;
    else {
      int8_t tool = toolSelections[current_selection-2];
      if(!smuffConfig.sendActionCmds) {
        selectTool(tool);
      }
      else {
        // send "Tool Change" action to controller
        char _tool[10];
        sprintf_P(_tool, P_Tool, tool);
        sprintf_P(_tmp, P_Action, _tool);
        printResponse(_tmp, 0);
        printResponse(_tmp, 1);
        printResponse(_tmp, 2);
        // TODO: do tool change using Duet3D
        // not yet possible due to Duet3D is being blocked waiting for endstop
        /*
        sprintf_P(tmp, PSTR("T%d\n"), tool);
        Serial2.print(tmp);
        */
      }
      startTime = millis();
    }
  }
}

bool checkStopMenu(unsigned startTime) {
  if(forceStopMenu)
    return true;
  if(remoteKey == REMOTE_HOME)
    return true;
  if(millis() - startTime > (unsigned long)smuffConfig.menuAutoClose * 1000)
    return true;
  return false;
}

bool checkAutoClose() {
  if(forceStopMenu)
    return true;
  if(remoteKey == REMOTE_HOME)
    return true;
  if (millis() - lastEncoderButtonTime >= (unsigned long)smuffConfig.menuAutoClose * 1000)
    return true;
  return false;
}

void resetAutoClose() {
  lastEncoderButtonTime = millis();
}

void debounceButton() {
  #if !defined(USE_LEONERD_DISPLAY) && !defined(USE_CREALITY_DISPLAY)
  delay(20);
  while(getEncoderButton()) {
      delay(20);
  }
  #endif
}
