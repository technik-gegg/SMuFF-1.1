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

extern int      swapTools[];
extern ZStepper steppers[];
extern ZServo   servo, servoRevolver;
extern int      toolSelections[];

char* extractTitle(const char* menu PROGMEM, int index) {
  char* tok = strtok((char*)menu, "\n");
  int cnt = -1;
  while(tok != NULL) {
    if(++cnt == index)
      break;
    tok = strtok(NULL, "\n");
  }

  if(tok != NULL) {
    char* tok2 = tok+strlen(tok)-1;
    if(tok2 != NULL) {
      while(*--tok2 != ' ')
        *tok2 = 0;
      while(*--tok2 == ' ')
        *tok2 = 0;
    }
    //__debug(PSTR("Menu: %s tok:%s"), menu, tok);
    return tok;
    }
  return NULL;
}

void setupToolsMenu(char* menu) {
  char tmp[50];
  sprintf_P(menu, P_MenuItemBack);
  memset(toolSelections, 0, sizeof(int)*MAX_TOOLS);
  int n = 0;
  for(int i=0; i< smuffConfig.toolCount; i++) {
    if(i == toolSelected)
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
  char items[450];
  #if !defined(__AVR__)
    char items2[200];
  #endif
  char stat[10];
  char sstat[10];
  char mstat[10];
  char opt[30] = "";

  steppers[SELECTOR].getEnabled() ? sprintf_P(stat, P_Off) : sprintf_P(stat, P_On);
  servoRevolver.getDegree() == smuffConfig.revolverOnPos ? sprintf_P(sstat, P_Open) : sprintf_P(sstat, P_Close);
  maintainingMode ? sprintf_P(mstat, P_Off) : sprintf_P(mstat, P_On);
  if(smuffConfig.prusaMMU2) {
    sprintf_P(opt, P_MenuItemsPMMU);
  }
  sprintf_P(menu, P_MenuItemBack);
  #if !defined(__AVR__)
    sprintf_P(items2, P_MenuItemsDefault, P_MenuItemSeparator);
    #if defined(SMUFF_V5)
      sprintf_P(items, P_MenuItems, stat, sstat, mstat, opt, P_MenuItemSeparator, items2);
    #else
      sprintf_P(items, P_MenuItems, stat, opt, P_MenuItemSeparator, items2);
    #endif
  #else
  // need two different menus because of low memory issues on the ATMEGA
  sprintf_P(items, P_MenuItems, stat, opt);
  #endif
  strcat(menu, items);
}

void setupOffsetMenu(char* menu) {
  char items[120];
  sprintf_P(menu, P_MenuItemBack);
  sprintf_P(items, P_OfsMenuItems, String(smuffConfig.firstToolOffset).c_str(), String(smuffConfig.firstRevolverOffset).c_str());
  strcat(menu, items);
}

void setupSwapMenu(char* menu) {
  char tmp[128];
  sprintf_P(menu, P_MenuItemBack);
  sprintf_P(tmp, P_SwapReset);
  strcat(menu, tmp);
  for(int i=0; i< smuffConfig.toolCount; i++) {
    sprintf_P(tmp, P_SwapMenu, i, swapTools[i]);
    strcat(menu, tmp);
    strcat(menu, (char*)"\n");
  }
  menu[strlen(menu)-1] = '\0';
}

void setupSettingsMenu(char* menu) {
  char items[400];
  sprintf_P(menu, P_MenuItemBack);
  sprintf_P(items, P_SettingsMenuItems,
  String(smuffConfig.toolCount).c_str(),
  String(smuffConfig.bowdenLength).c_str(),
  String(smuffConfig.selectorDistance).c_str(),
  String(smuffConfig.menuAutoClose).c_str(),
  String(smuffConfig.fanSpeed).c_str(),
  smuffConfig.prusaMMU2 ? P_Yes : P_No,
  smuffConfig.sendPeriodicalStats ? P_Yes : P_No,
  #if !defined(__AVR__)
    "\u25b8",
    "\u25b8",
    "\u25b8",
    "\u25b8",
  #else
    ">",
    ">",
    ">",
  #endif
  P_MenuItemSeparator);
  strcat(menu, items);
}

void setupBaudrateMenu(char* menu) {
  char items[128];
  sprintf_P(menu, P_MenuItemBack);
  sprintf_P(items, P_BaudMenuItems,
  String(smuffConfig.serial1Baudrate).c_str(),
  String(smuffConfig.serial2Baudrate).c_str());
  strcat(menu, items);
}

void setupSteppersMenu(char *menu) {
  char items[128];
  sprintf_P(menu, P_MenuItemBack);
  sprintf_P(items, P_SteppersMenuItems,
#if !defined(__AVR__)
  "\u25b8",
  "\u25b8",
  "\u25b8");
#else
  ">",
  ">",
  ">");
#endif
  strcat(menu, items);
}

void setupRevolverMenu(char* menu) {
  char items1[170];
  #if !defined(SMUFF_V5)
    char items2[240];
  #endif
  sprintf_P(menu, P_MenuItemBack);
  sprintf_P(items1, P_AllSteppersMenuItems,
  smuffConfig.invertDir_Y ? P_Yes : P_No,
  smuffConfig.endstopTrigger_Y ? P_High : P_Low,
  String(smuffConfig.stepDelay_Y).c_str(),
  String(smuffConfig.maxSpeed_Y).c_str(),
  String(smuffConfig.maxSpeedHS_Y).c_str(),
  String(smuffConfig.acceleration_Y).c_str());
  #if defined(SMUFF_V5)
    sprintf_P(items1, P_RevolverMenuItems,
    smuffConfig.homeAfterFeed ? P_Yes : P_No,
    smuffConfig.resetBeforeFeed_Y ? P_Yes : P_No,
    smuffConfig.revolverIsServo ? P_Yes : P_No,
    String(smuffConfig.revolverOffPos).c_str(),
    String(smuffConfig.revolverOnPos).c_str(),
    String(smuffConfig.servoCycles1).c_str(),
    String(smuffConfig.servoCycles2).c_str());
    strcat(menu, items1);
  #else
    sprintf_P(items2, P_RevolverMenuItems,
    String(smuffConfig.stepsPerRevolution_Y).c_str(),
    smuffConfig.homeAfterFeed ? P_Yes : P_No,
    smuffConfig.resetBeforeFeed_Y ? P_Yes : P_No,
    smuffConfig.wiggleRevolver ? P_Yes : P_No,
    smuffConfig.revolverIsServo ? P_Yes : P_No,
    String(smuffConfig.revolverOffPos).c_str(),
    String(smuffConfig.revolverOnPos).c_str(),
    String(smuffConfig.servoCycles1).c_str(),
    String(smuffConfig.servoCycles2).c_str());
    strcat(menu, items1);
    strcat(menu, items2);
  #endif
}

void setupFeederMenu(char* menu) {
  char items1[256];
  char items2[256];
  sprintf_P(menu, P_MenuItemBack);
  sprintf_P(items1, P_AllSteppersMenuItems,
  smuffConfig.invertDir_Z ? P_Yes : P_No,
  smuffConfig.endstopTrigger_Z ? P_High : P_Low,
  String(smuffConfig.stepDelay_Z).c_str(),
  String(smuffConfig.maxSpeed_Z).c_str(),
  String(smuffConfig.maxSpeedHS_Z).c_str(),
  String(smuffConfig.acceleration_Z).c_str());
  sprintf_P(items2, P_FeederMenuItems,
  String(smuffConfig.stepsPerMM_Z).c_str(),
  smuffConfig.enableChunks ? P_Yes : P_No,
  String(smuffConfig.feedChunks).c_str(),
  String(smuffConfig.insertLength).c_str(),
  String(smuffConfig.insertSpeed_Z).c_str(),
  String(smuffConfig.reinforceLength).c_str());
  strcat(menu, items1);
  strcat(menu, items2);
}

void setupSelectorMenu(char* menu) {
  char items1[150];
  char items2[56];
  sprintf_P(menu, P_MenuItemBack);
  sprintf_P(items1, P_AllSteppersMenuItems,
  smuffConfig.invertDir_X ? P_Yes : P_No,
  smuffConfig.endstopTrigger_X ? P_High : P_Low,
  String(smuffConfig.stepDelay_X).c_str(),
  String(smuffConfig.maxSpeed_X).c_str(),
  String(smuffConfig.maxSpeedHS_X).c_str(),
  String(smuffConfig.acceleration_X).c_str());
  sprintf_P(items2, P_SelectorMenuItems,
  String(smuffConfig.stepsPerMM_X).c_str());
  strcat(menu, items1);
  strcat(menu, items2);
}

const char* translateColor(int color) {
  char* colorNames[16];
  char tmp[80];
  sprintf_P(tmp, P_Colors);
  splitStringLines(colorNames, (int)(sizeof(colorNames)/sizeof(colorNames[0])), tmp);
  if(color >= 0 && color < (int)sizeof(colorNames))
    return colorNames[color];
  return "???";
}

void setupDisplayMenu(char* menu) {
  char items1[200];
  sprintf_P(menu, P_MenuItemBack);
  sprintf_P(items1, P_DisplayMenuItems,
  String(smuffConfig.powerSaveTimeout).c_str(),
  String(smuffConfig.lcdContrast).c_str(),
  translateColor(smuffConfig.backlightColor));
  strcat(menu, items1);
}

void setupTestrunMenu(char* menu) {
  char items[410];

  sprintf_P(menu, P_MenuItemBack);
  memset(items, 0, sizeof(items));
  if(getFiles("/", ".gcode", 20, true, items)) {
    strcat(menu, items);
  }
}

void showMainMenu() {
  bool stopMenu = false;
  unsigned int startTime = millis();
  uint8_t current_selection = 0;
  char tmp[128];
  char _title[40];
  char _menu[512];

  do {
    setupMainMenu(_menu);
    //__debug(PSTR("MainList: %s %d"), _menu, strlen(_menu));
    sprintf_P(_title, P_TitleMainMenu);
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(_title, current_selection, _menu);

    if(current_selection == 0)
      return;
    else {
      char* title = extractTitle(_menu, current_selection-1);
      bool enabled = steppers[SELECTOR].getEnabled();

      switch(current_selection) {
        case 1:
          stopMenu = true;
          break;

        case 2:
          moveHome(SELECTOR);
          moveHome(REVOLVER, true, false);
          startTime = millis();
          break;

        case 3:
          steppers[SELECTOR].setEnabled(!enabled);
          steppers[REVOLVER].setEnabled(!enabled);
          steppers[FEEDER].setEnabled(!enabled);
          if(smuffConfig.revolverIsServo) {
            setServoPos(1, smuffConfig.revolverOffPos);
          }
          startTime = millis();
          break;
          #if defined(SMUFF_V5)
            case 4:
              if(servoRevolver.getDegree() == smuffConfig.revolverOffPos)
                setServoPos(1, smuffConfig.revolverOnPos);
              else
                setServoPos(1, smuffConfig.revolverOffPos);
              startTime = millis();
              break;

            case 5:
              maintainTool();
              startTime = millis();
              break;

            case 6:
              feederJammed = false;
              beep(2);
              sprintf_P(tmp, P_JamCleared);
              drawUserMessage(tmp);
              startTime = millis();
              break;

            case 7:
              showSwapMenu(title);
              startTime = millis();
              break;

            case 8:
              if(smuffConfig.prusaMMU2)
                loadFilamentPMMU2();
              else
                loadFilament();
              startTime = millis();
              break;

            case 9:
              unloadFilament();
              startTime = millis();
              break;

            case 10:
            #if !defined(__AVR__)
              if(smuffConfig.prusaMMU2)
                loadFilament();
            #else
              showOffsetsMenu(title);
            #endif
            startTime = millis();
            break;

            case 11:
              if(!smuffConfig.prusaMMU2)
                showSettingsMenu(title);
              current_selection = 1;
              startTime = millis();
              break;

            case 12:
              if(smuffConfig.prusaMMU2)
                showSettingsMenu(title);
              current_selection = 1;
              startTime = millis();
              break;

            case 13:
              if(!smuffConfig.prusaMMU2)
                showTestrunMenu(title);
              current_selection = 1;
              startTime = millis();
              break;

            case 14:
              if(smuffConfig.prusaMMU2)
                showTestrunMenu(title);
              current_selection = 1;
              startTime = millis();
              break;
          #else
            case 4:
              feederJammed = false;
              beep(2);
              sprintf_P(tmp, P_JamCleared);
              drawUserMessage(tmp);
              startTime = millis();
              break;

            case 5:
              showSwapMenu(title);
              startTime = millis();
              break;

            case 6:
              if(smuffConfig.prusaMMU2)
                loadFilamentPMMU2();
              else
                loadFilament();
              startTime = millis();
              break;

            case 7:
              unloadFilament();
              startTime = millis();
              break;

            case 8:
              #if !defined(__AVR__)
                if(smuffConfig.prusaMMU2)
                  loadFilament();
              #else
                showOffsetsMenu(title);
              #endif
              startTime = millis();
              break;

            case 9:
              if(!smuffConfig.prusaMMU2)
                showSettingsMenu(title);
              current_selection = 1;
              startTime = millis();
              break;

            case 10:
              if(smuffConfig.prusaMMU2)
                showSettingsMenu(title);
              current_selection = 1;
              startTime = millis();
              break;

            case 11:
              if(!smuffConfig.prusaMMU2)
                showTestrunMenu(title);
              current_selection = 1;
              startTime = millis();
              break;

            case 12:
              if(smuffConfig.prusaMMU2)
                showTestrunMenu(title);
              current_selection = 1;
              startTime = millis();
              break;
          #endif
      }
    }
    debounceButton();
  } while(!stopMenu);
}

void showTestrunMenu(char* menuTitle) {
  bool stopMenu = false;
  uint8_t current_selection = 0;
  char _menu[400];
  char* fnames[16];
  String _file;

  do {
    setupTestrunMenu(_menu);
    resetAutoClose();

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);

    if(current_selection == 0)
      return;
    else if(current_selection == 1) {
      stopMenu = true;
    }
    else if(current_selection >= 2) {
      splitStringLines(fnames, (int)(sizeof(fnames)/sizeof(fnames[0])), (const char*)_menu);
      _file = String(fnames[current_selection-1]);
      _file.trim();
      //__debug(PSTR("Selected file: %s"), _file.c_str());
      testRun(_file);
    }
  } while(!stopMenu);
}

bool selectBaudrate(int port, char* menuTitle) {
  unsigned long val;
  char tmp[128];
  sprintf_P(tmp, P_Baudrates);
  if(port == 1) {
    val = smuffConfig.serial1Baudrate;
    if(showInputDialog(menuTitle, P_Baud, &val, String(tmp)))
      smuffConfig.serial1Baudrate = val;
  }
  else {
      val = smuffConfig.serial2Baudrate;
      if(showInputDialog(menuTitle, P_Baud, &val, String(tmp)))
        smuffConfig.serial2Baudrate = val;
  }
  return true;
}

bool selectBacklightColor(int color, char* menuTitle) {
    int val = color;
    char tmp[80];
    sprintf_P(tmp, P_Colors);
    if(showInputDialog(menuTitle, P_Color, &val, String(tmp), setBacklightIndex, true)) {
      smuffConfig.backlightColor = val;
      //__debug(PSTR("Backlight: %d"), val);
    }
    return true;
}

void positionServoCallback(int val) {
  setServoPos(1, val);
}

void showRevolverMenu(char* menuTitle) {
  bool stopMenu = false;
  unsigned int startTime = millis();
  uint8_t current_selection = 0;
  char* title;
  bool bVal;
  int iVal;
  char _menu[350];

  do {
    setupRevolverMenu(_menu);
    //__debug(PSTR("Revolver Menu: %s %d"), _menu, strlen(_menu));
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);

    if(current_selection == 0)
      return;
    else {
      title = extractTitle(_menu, current_selection-1);
      switch(current_selection) {
        case 1:
          stopMenu = true;
          break;

        #if defined(SMUFF_V5)
          case 2: // Home after feed
            bVal = smuffConfig.homeAfterFeed;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.homeAfterFeed = bVal;
            startTime = millis();
            break;

          case 3: // Reset before feed
            bVal = smuffConfig.resetBeforeFeed_Y;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.resetBeforeFeed_Y = bVal;
            startTime = millis();
            break;

          case 4: // Use Servo
            bVal = smuffConfig.revolverIsServo;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.revolverIsServo = bVal;
            startTime = millis();
            break;

          case 5: // Servo open
            iVal = smuffConfig.revolverOffPos;
            if(showInputDialog(title, P_OpenPos, &iVal, 0, 2400, positionServoCallback))
              smuffConfig.revolverOffPos = iVal;
            startTime = millis();
            break;

          case 6: // Servo closed
            iVal = smuffConfig.revolverOnPos;
            if(showInputDialog(title, P_ClosedPos, &iVal, 0, 2400, positionServoCallback))
              smuffConfig.revolverOnPos = iVal;
            startTime = millis();
            break;

          case 7: // Servo 1 cycles
            iVal = smuffConfig.servoCycles1;
            if(showInputDialog(title, P_ServoCycles, &iVal, 0, 50)) {
              smuffConfig.servoCycles1 = iVal;
              servo.setMaxCycles(iVal);
            }
            startTime = millis();
            break;

          case 8: // Servo 2 cycles
            iVal = smuffConfig.servoCycles2;
            if(showInputDialog(title, P_ServoCycles, &iVal, 0, 50)) {
              smuffConfig.servoCycles2 = iVal;
              servoRevolver.setMaxCycles(iVal);
            }
            startTime = millis();
            break;
        #else
          case 2: // Invert dir
            bVal = smuffConfig.invertDir_Y;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.invertDir_Y = bVal;
            startTime = millis();
            break;

          case 3: // Endstop trigger
            iVal = smuffConfig.endstopTrigger_Y;
            if(showInputDialog(title, P_TriggerOn, &iVal, 0, 1))
              smuffConfig.endstopTrigger_Y = iVal;
            startTime = millis();
            break;

          case 4: // Step Delay
            iVal = smuffConfig.stepDelay_Y;
            if(showInputDialog(title, P_InMicroseconds, &iVal, 0, 100))
              smuffConfig.stepDelay_Y = iVal;
            startTime = millis();
            break;

          case 5: // Max. Speed
            iVal = smuffConfig.maxSpeed_Y;
            if(showInputDialog(title, P_InTicks, &iVal, 1, 50000))
              smuffConfig.maxSpeed_Y = iVal;
            startTime = millis();
            break;

          case 6: // Max. Speed HS
            iVal = smuffConfig.maxSpeedHS_Y;
            if(showInputDialog(title, P_InTicks, &iVal, 1, 50000))
              smuffConfig.maxSpeedHS_Y = iVal;
            startTime = millis();
            break;

          case 7: // Acceleration
            iVal = smuffConfig.acceleration_Y;
            if(showInputDialog(title, P_InTicks, &iVal, 1, 60000))
              smuffConfig.acceleration_Y = iVal;
            startTime = millis();
            break;

          case 8: // Steps Per Rev
            iVal = smuffConfig.stepsPerRevolution_Y;
            if(showInputDialog(title, P_InSteps, &iVal, 1, 10000))
              smuffConfig.stepsPerRevolution_Y = iVal;
            startTime = millis();
            break;

          case 9: // Home after feed
            bVal = smuffConfig.homeAfterFeed;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.homeAfterFeed = bVal;
            startTime = millis();
            break;

          case 10: // Reset before feed
            bVal = smuffConfig.resetBeforeFeed_Y;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.resetBeforeFeed_Y = bVal;
            startTime = millis();
            break;

          case 11: // Wiggle
            bVal = smuffConfig.wiggleRevolver;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.wiggleRevolver = bVal;
            startTime = millis();
            break;

          case 12: // Use Servo
            bVal = smuffConfig.revolverIsServo;
            if(showInputDialog(title, P_YesNo, &bVal))
              smuffConfig.revolverIsServo = bVal;
            startTime = millis();
            break;

          case 13: // Servo open
            iVal = smuffConfig.revolverOffPos;
            if(showInputDialog(title, P_OpenPos, &iVal, 0, 2400, positionServoCallback))
              smuffConfig.revolverOffPos = iVal;
            startTime = millis();
            break;

          case 14: // Servo closed
            iVal = smuffConfig.revolverOnPos;
            if(showInputDialog(title, P_ClosedPos, &iVal, 0, 2400, positionServoCallback))
              smuffConfig.revolverOnPos = iVal;
            startTime = millis();
            break;

          case 15: // Servo 1 cycles
            iVal = smuffConfig.servoCycles1;
            if(showInputDialog(title, P_ServoCycles, &iVal, 0, 50))
              smuffConfig.servoCycles1 = iVal;
              servo.setMaxCycles(iVal);
            startTime = millis();
            break;

          case 16: // Servo 2 cycles
            iVal = smuffConfig.servoCycles2;
            if(showInputDialog(title, P_ServoCycles, &iVal, 0, 50))
              smuffConfig.servoCycles2 = iVal;
              servoRevolver.setMaxCycles(iVal);
            startTime = millis();
            break;
        #endif
      }
    }
  } while(!stopMenu);
}

void showSelectorMenu(char* menuTitle) {
  bool stopMenu = false;
  unsigned int startTime = millis();
  uint8_t current_selection = 0;
  char* title;
  bool bVal;
  int iVal;
  char _menu[300];

  do {
    setupSelectorMenu(_menu);
    //__debug(PSTR("Selector Menu: %s %d"), _menu, strlen(_menu));
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);

    if(current_selection == 0)
      return;
    else {
      title = extractTitle(_menu, current_selection-1);
      switch(current_selection) {
      case 1:
        stopMenu = true;
        break;

      case 2: // Invert dir
        bVal = smuffConfig.invertDir_X;
        if(showInputDialog(title, P_YesNo, &bVal))
          smuffConfig.invertDir_X = bVal;
        startTime = millis();
        break;

      case 3: // Endstop trigger
        iVal = smuffConfig.endstopTrigger_X;
        if(showInputDialog(title, P_TriggerOn, &iVal, 0, 1))
          smuffConfig.endstopTrigger_X = iVal;
        startTime = millis();
        break;

      case 4: // Step Delay
        iVal = smuffConfig.stepDelay_X;
        if(showInputDialog(title, P_InMicroseconds, &iVal, 0, 100))
          smuffConfig.stepDelay_X = iVal;
        startTime = millis();
        break;

      case 5: // Max. Speed
        iVal = smuffConfig.maxSpeed_X;
        if(showInputDialog(title, P_InTicks, &iVal, 1, 50000))
          smuffConfig.maxSpeed_X = iVal;
        startTime = millis();
        break;

      case 6: // Max. Speed HS
        iVal = smuffConfig.maxSpeedHS_X;
        if(showInputDialog(title, P_InTicks, &iVal, 1, 50000))
          smuffConfig.maxSpeedHS_X = iVal;
        startTime = millis();
        break;

      case 7: // Acceleration
        iVal = smuffConfig.acceleration_X;
        if(showInputDialog(title, P_InTicks, &iVal, 1, 60000))
          smuffConfig.acceleration_X = iVal;
        startTime = millis();
        break;

      case 8: // Steps Per MM
        iVal = smuffConfig.stepsPerMM_X;
        if(showInputDialog(title, P_InSteps, &iVal, 1, 10000))
          smuffConfig.stepsPerMM_X = iVal;
        startTime = millis();
        break;
      }
    }
  } while(!stopMenu);
}

void showFeederMenu(char* menuTitle) {
  bool stopMenu = false;
  unsigned int startTime = millis();
  uint8_t current_selection = 0;
  char* title;
  bool bVal;
  int iVal;
  float fVal;
  char _menu[300];

  do {
    setupFeederMenu(_menu);
    //__debug(PSTR("Feeder Menu: %s %d"), _menu, strlen(_menu));
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);

    if(current_selection == 0)
      return;
    else {
      title = extractTitle(_menu, current_selection-1);
      switch(current_selection) {
      case 1:
        stopMenu = true;
        break;

      case 2: // Invert dir
        bVal = smuffConfig.invertDir_Z;
        if(showInputDialog(title, P_YesNo, &bVal))
          smuffConfig.invertDir_Z = bVal;
        startTime = millis();
        break;

      case 3: // Endstop trigger
        iVal = smuffConfig.endstopTrigger_Z;
        if(showInputDialog(title, P_TriggerOn, &iVal, 0, 1))
          smuffConfig.endstopTrigger_Z = iVal;
        startTime = millis();
        break;

      case 4: // Step Delay
        iVal = smuffConfig.stepDelay_Z;
        if(showInputDialog(title, P_InMicroseconds, &iVal, 0, 100))
          smuffConfig.stepDelay_Z = iVal;
        startTime = millis();
        break;

      case 5: // Max. Speed
        iVal = smuffConfig.maxSpeed_Z;
        if(showInputDialog(title, P_InTicks, &iVal, 1, 50000))
          smuffConfig.maxSpeed_Z = iVal;
        startTime = millis();
        break;

      case 6: // Max. Speed HS
        iVal = smuffConfig.maxSpeedHS_Z;
        if(showInputDialog(title, P_InTicks, &iVal, 1, 50000))
          smuffConfig.maxSpeedHS_Z = iVal;
        startTime = millis();
        break;

      case 7: // Acceleration
        iVal = smuffConfig.acceleration_Z;
        if(showInputDialog(title, P_InTicks, &iVal, 1, 60000)) {
          smuffConfig.acceleration_Z = iVal;
          if(smuffConfig.insertSpeed_Z > smuffConfig.acceleration_Z)
          smuffConfig.acceleration_Z = smuffConfig.insertSpeed_Z;
        }
        startTime = millis();
        break;

      case 8: // Steps Per MM
        iVal = smuffConfig.stepsPerMM_Z;
        if(showInputDialog(title, P_InSteps, &iVal, 1, 10000))
          smuffConfig.stepsPerMM_Z = iVal;
        startTime = millis();
        break;

      case 9: // Enable chunks
        bVal = smuffConfig.enableChunks;
        if(showInputDialog(title, P_YesNo, &bVal))
          smuffConfig.enableChunks = bVal;
        startTime = millis();
        break;

      case 10: // Feed chunks
        iVal = smuffConfig.feedChunks;
        if(showInputDialog(title, P_NoOfChunks, &iVal, 0, 100))
          smuffConfig.feedChunks = iVal;
        if(iVal == 0)
          smuffConfig.enableChunks = false;
        startTime = millis();
        break;

      case 11: // Insert length
        fVal = smuffConfig.insertLength;
        if(showInputDialog(title, P_InMillimeter, &fVal, 1, smuffConfig.selectorDistance))
          smuffConfig.insertLength = fVal;
        startTime = millis();
        break;

      case 12: // Insert Speed
        iVal = smuffConfig.insertSpeed_Z;
        if(showInputDialog(title, P_InTicks, &iVal, 1, 60000)) {
          smuffConfig.insertSpeed_Z = iVal;
          if(smuffConfig.insertSpeed_Z > smuffConfig.acceleration_Z)
          smuffConfig.acceleration_Z = smuffConfig.insertSpeed_Z;
        }
        startTime = millis();
        break;

      case 13: // Reinforce length
        fVal = smuffConfig.reinforceLength;
        if(showInputDialog(title, P_InMillimeter, &fVal, 0, 10))
          smuffConfig.reinforceLength = fVal;
        startTime = millis();
        break;
      }
    }
  } while(!stopMenu);
}

void showSteppersMenu(char* menuTitle) {
  bool stopMenu = false;
  unsigned int startTime = millis();
  uint8_t current_selection = 0;
  char* title;
  char _menu[128];

  do {
    setupSteppersMenu(_menu);
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);

    if(current_selection == 0)
      return;
    else {
      title = extractTitle(_menu, current_selection-1);
      switch(current_selection) {
      case 1:
        stopMenu = true;
        break;

      case 2: // Selector
        showSelectorMenu(title);
        current_selection = 1;
        startTime = millis();
        break;

      case 3: // Feeder
        showFeederMenu(title);
        current_selection = 1;
        startTime = millis();
        break;

      case 4: // Revolver or Servo
        showRevolverMenu(title);
        current_selection = 1;
        startTime = millis();
        break;
      }
    }
  } while(!stopMenu);
}

void showDisplayMenu(char* menuTitle) {
  bool stopMenu = false;
  unsigned int startTime = millis();
  uint8_t current_selection = 0;
  char* title;
  char _menu[128];
  int iVal;
  bool bVal;

  do {
    setupDisplayMenu(_menu);
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);

    if(current_selection == 0)
      return;
    else {
      title = extractTitle(_menu, current_selection-1);
      switch(current_selection) {
        case 1:
          stopMenu = true;
          break;

        case 2: // Power Save Timeout
          iVal = smuffConfig.powerSaveTimeout;
          if(showInputDialog(title, P_InSeconds, &iVal, 0, 480))
            smuffConfig.powerSaveTimeout = iVal;
          startTime = millis();
          break;

        case 3: // LCD Contrast
          iVal = smuffConfig.lcdContrast;
          if(showInputDialog(title, P_InValue, &iVal, MIN_CONTRAST, MAX_CONTRAST))
            smuffConfig.lcdContrast = iVal;
          startTime = millis();
          break;

        case 4: // Backlight Color
          selectBacklightColor(smuffConfig.backlightColor, title);
          startTime = millis();
          break;
      }
    }
  } while(!stopMenu);
}

void showSettingsMenu(char* menuTitle) {
  bool stopMenu = false;
  unsigned int startTime = millis();
  uint8_t current_selection = 0;
  float fVal;
  int iVal;
  bool bVal;
  char *title;
  char msg[128];
  char _menu[300];

  do {
    setupSettingsMenu(_menu);
    //__debug(PSTR("Settings Menu: %d %d"), strlen(_menu), 0 /*freeMemory()*/);
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);

    if(current_selection == 0)
      return;
    else {
      title = extractTitle(_menu, current_selection-1);
      switch(current_selection) {
        case 1:
          stopMenu = true;
          break;

        case 2: // Tool count
          iVal = smuffConfig.toolCount;
          if(showInputDialog(title, P_ToolCount, &iVal, MIN_TOOLS, MAX_TOOLS))
            smuffConfig.toolCount = iVal;
          startTime = millis();
          break;

        case 3: // Bowden length
          fVal = smuffConfig.bowdenLength;
          if(showInputDialog(title, P_InMillimeter, &fVal, 20, 2000))
          smuffConfig.bowdenLength = fVal;
          startTime = millis();
          break;

        case 4: // Selector distance
          fVal = smuffConfig.selectorDistance;
          if(showInputDialog(title, P_InMillimeter, &fVal, 10, 50))
            smuffConfig.selectorDistance = fVal;
          startTime = millis();
          break;

        case 5: // Menu auto close
          iVal = smuffConfig.menuAutoClose;
          if(showInputDialog(title, P_InSeconds, &iVal, 0, 300))
            smuffConfig.menuAutoClose = iVal;
          startTime = millis();
          break;

        case 6: // Fan speed
          iVal = smuffConfig.fanSpeed;
          if(showInputDialog(title, P_InPercent, &iVal, 0, 100)) {
            smuffConfig.fanSpeed = iVal;
            #if defined(__STM32F1__)
              pwmWrite(FAN_PIN, map(smuffConfig.fanSpeed, 0, 100, 0, 255));
            #elif defined(__ESP32__)
              ledcWrite(FAN_PIN, map(smuffConfig.fanSpeed, 0, 100, 0, 255));
            #else
              analogWrite(FAN_PIN, map(smuffConfig.fanSpeed, 0, 100, 0, 255));
            #endif
          }
          startTime = millis();
          break;

        case 7: // Prusa Emulation
          bVal = smuffConfig.prusaMMU2;
          if(showInputDialog(title, P_YesNo, &bVal))
            smuffConfig.prusaMMU2 = bVal;
          startTime = millis();
          break;

        case 8: // Send Status Info
          bVal = smuffConfig.sendPeriodicalStats;
          if(showInputDialog(title, P_YesNo, &bVal))
            smuffConfig.sendPeriodicalStats = bVal;
          startTime = millis();
          break;

        case 9: // Baudrates
          showBaudratesMenu(title);
          current_selection = 1;
          startTime = millis();
          break;

        case 10: // Offsets
          showOffsetsMenu(title);
          current_selection = 1;
          startTime = millis();
          break;

        case 11: // Steppers
          showSteppersMenu(title);
          current_selection = 1;
          startTime = millis();
          break;

        case 12: // Display
          showDisplayMenu(title);
          current_selection = 1;
          startTime = millis();
          break;

        case 13: // Separator
          break;

        case 14: // STORE CONFIG
          if(writeConfig()) {
            beep(1);
            sprintf_P(msg, P_ConfigWriteSuccess);
          }
          else {
            beep(3);
            sprintf_P(msg, P_ConfigWriteFail);
          }
          drawUserMessage(msg);
          delay(3000);
          current_selection = 1;
          startTime = millis();
          break;
      }
    }
  } while(!stopMenu);
}

void drawSwapTool(int from, int with) {
  char tmp[256];
  sprintf_P(tmp, P_SwapToolDialog, from, with);
  drawUserMessage(String(tmp));
}

uint8_t swapTool(uint8_t index) {
  uint8_t ndx = 0;

  debounceButton();
  delay(250);

  drawSwapTool(index, ndx);

  while(1) {
  if(digitalRead(ENCODER_BUTTON_PIN) == LOW) {
    break;
  }
  int turn = encoder.getValue();
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
  uint8_t current_selection = 0;
  char _menu[128];

  do {
    setupSwapMenu(_menu);
    resetAutoClose();

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);

    if(current_selection == 0)
      return;
    else if(current_selection == 1) {
      stopMenu = true;
    }
    else if(current_selection == 2) {
      for(int i=0; i < MAX_TOOLS; i++) {
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
  } while(!stopMenu);
  saveStore();
}

void showBaudratesMenu(char* menuTitle) {
  bool stopMenu = false;
  unsigned int startTime = millis();
  uint8_t current_selection = 0;
  char* title;
  char _menu[128];

  do {
    setupBaudrateMenu(_menu);
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);

    if(current_selection == 0)
      return;
    else {
      title = extractTitle(_menu, current_selection-1);
      switch(current_selection) {
      case 1:
        stopMenu = true;
        break;

      case 2:
        selectBaudrate(1, title);
        current_selection = 1;
        startTime = millis();
        break;

      case 3:
        selectBaudrate(2, title);
        current_selection = 1;
        startTime = millis();
        break;
      }
    }
  } while(!stopMenu);
}

void showOffsetsMenu(char* menuTitle) {
  bool stopMenu = false;
  unsigned int startTime = millis();
  uint8_t current_selection = 0;
  char _menu[120];

  do {
    setupOffsetMenu(_menu);
    //__debug(PSTR("Offsets Menu: %s %d %d"), _menu, strlen(_menu), 0 /*freeMemory()*/);
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    current_selection = display.userInterfaceSelectionList(menuTitle, current_selection, _menu);

    if(current_selection == 0)
      return;
    else {
      switch(current_selection) {
      case 1:
        stopMenu = true;
        break;

      case 2:
        changeOffset(SELECTOR);
        current_selection = 1;
        startTime = millis();
        break;

      case 3:
        changeOffset(REVOLVER);
        current_selection = 1;
        startTime = millis();
        break;
      }
    }
  } while(!stopMenu);
}

void changeOffset(int index) {
  int steps = smuffConfig.stepsPerRevolution_Y/360;
  float stepsF = 0.1f;
  long pos;
  float posF;
  int turn, btn;
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

  unsigned int curSpeed = steppers[index].getMaxSpeed();
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
      //__debug(PSTR("Turn: %d  Pos: %d   PosF: %s"), turn, pos, String(posF).c_str());
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
  unsigned int startTime = millis();
  uint8_t current_selection = 0;
  char _title[60];
  char _menu[128];
  char _tmp[40];

  do {
    sprintf_P(_title, P_TitleToolsMenu);
    setupToolsMenu(_menu);
    resetAutoClose();
    stopMenu = checkStopMenu(startTime);

    uint8_t startPos = toolSelected == 255 ? 0 : toolSelected+1;
    current_selection = display.userInterfaceSelectionList(_title, startPos, _menu);

    if(current_selection <= 1)
      stopMenu = true;
    else {
      int tool = toolSelections[current_selection-2];
      if(!smuffConfig.duetDirect) {
      selectTool(tool);
      }
      else {
      //selectTool(tool);
      // the duetDirect flag is a synonym of "communicate directly with the controller attached"
      if(smuffConfig.duetDirect) {
        sprintf(_tmp, "//action: T%d\n", tool);
        printResponse(_tmp, 0);
        printResponse(_tmp, 1);
        printResponse(_tmp, 2);
      }
      // TODO: do tool change using Duet3D
      // not yet possible due to Duet3D is being blocked waiting for endstop
      /*
      sprintf_P(tmp, PSTR("T%d\n"), tool);
      Serial2.print(tmp);
      */
      }
      startTime = millis();
    }
  } while(!stopMenu);
}

bool checkStopMenu(unsigned startTime) {
  if(millis() - startTime > (unsigned long)smuffConfig.menuAutoClose * 1000) {
    return true;
  }
  return false;
}

void resetAutoClose() {
  lastEncoderButtonTime = millis();
}

bool checkAutoClose() {
  if (millis() - lastEncoderButtonTime >= (unsigned long)smuffConfig.menuAutoClose*1000) {
    return true;
  }
  return false;
}

void debounceButton() {
  delay(150);
  while(digitalRead(ENCODER_BUTTON_PIN) == LOW) {
    delay(20);
  }
  encoder.resetButton();
}
