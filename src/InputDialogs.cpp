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
#include "InputDialogs.h"

bool settingsChanged = false;

void drawValue(const char* title, const char* PROGMEM message, String val) {
    char tmp[256];
    char msg[128];
    sprintf_P(msg, message);
    sprintf_P(tmp, PSTR("%s\n \n%-11s%7s"), title, msg, val.c_str());
    drawUserMessage(String(tmp));
}

bool getEncoderButton(bool encoderOnly) {
  bool stat = false;
  stat = encoder.getButton() == Clicked;
  #if defined(USE_LEONERD_DISPLAY)
  if(!encoderOnly && encoder.getButton(MainButton) == Clicked) {
    stat = true;
  }
  #endif
  return stat;
}

void getEncoderButton(int16_t* turn, uint8_t* button, bool* isHeld, bool* isClicked) {
  *turn = encoder.getValue();
  ButtonState wheelBtn = encoder.getButton();
  ButtonState second;
  *isHeld = false;
  *isClicked = false;
  *button = NoButton;

  switch(remoteKey) {
    case REMOTE_NONE:
      break;
    case REMOTE_UP:
      remoteKey = REMOTE_NONE;
      *turn = 1;
      return;
    case REMOTE_DOWN:
      remoteKey = REMOTE_NONE;
      *turn = -1;
      return;
    case REMOTE_SELECT:
      remoteKey = REMOTE_NONE;
      *button = WheelButton;
      *isClicked = true;
      return;
    case REMOTE_ESCAPE:
      remoteKey = REMOTE_NONE;
      *button = WheelButton;
      *isHeld = true;
      return;
  }
  #if defined(USE_LEONERD_DISPLAY)
  if(wheelBtn == Open) {
    // special case for Main button on LeoNerds encoder
    // used as "Back" button (click) or "Home" button (long click)
    second = encoder.getButton(MainButton);
    if(second == Clicked || second == LongClicked) {
      *button = MainButton;
      *isClicked = second == Clicked;
      if(second == LongClicked) {
        encoder.setLED(LED_GREEN, true);
        encoder.setLED(LED_RED, true);
        delay(300);
        encoder.setLED(LED_GREEN, false);
        encoder.setLED(LED_RED, false);
        remoteKey = REMOTE_HOME;
      }
      return;
    }
    second = encoder.getButton(RightButton);
    if(second == Clicked || second == LongClicked) {
      *button = RightButton;
      *isClicked = second == Clicked;
      *isHeld = second == LongClicked;
      return;
    }
    second = encoder.getButton(LeftButton);
    if(second == Clicked || second == LongClicked) {
      *button = LeftButton;
      *isClicked = second == Clicked;
      *isHeld = second == LongClicked;
      return;
    }
  }
  #endif
  if(wheelBtn != Open) {
    if(wheelBtn == LongClicked || wheelBtn == Held) {
      *button = WheelButton;
      *isHeld = true;
    }
    else if(wheelBtn == Clicked) {
      *button = WheelButton;
      *isClicked = true;
    }
  }
}

void getInput(int16_t* turn, uint8_t* button, bool* isHeld, bool* isClicked, bool checkSerial) {
  #if defined(USE_LEONERD_DISPLAY)
  encoder.loop();
  #endif
  #if defined(__STM32F1__) || defined(__ESP32__)
  if(checkSerial)
    checkSerialPending();
  #endif
  getEncoderButton(turn, button, isHeld, isClicked);
  if(*isClicked || *isHeld || *turn != 0) {
    if(smuffConfig.encoderTickSound)
      encoderBeep(1);
    #if defined(USE_LEONERD_DISPLAY)
      encoder.setLED(LED_RED, false);
      encoder.setLED(LED_GREEN, false);
    #endif
  }
}

bool showInputDialog(const char* title, const char* PROGMEM message, float* val, float min, float max, fCallback cb, float increment) {
  bool stat = true;
  int16_t turn;
  uint8_t wheelBtn;
  bool isHeld, isClicked;

  debounceButton();
  encoder.setAccelerationEnabled(true);
  drawValue(title, message, String(*val));
  if(cb != nullptr) {
    cb(*val);
  }

  while(1) {
    getInput(&turn, &wheelBtn, &isHeld, &isClicked);
    if(isHeld || isClicked) {
      stat = isHeld ? false : true;
      break;
    }
    if(turn != 0) {
      *val += increment*turn;
      if(turn < 0) {
        if(*val < min) {
          *val = min;
          beep(1);
        }
      }
      else if(turn > 0) {
        if(*val > max) {
          *val = max;
          beep(1);
        }
      }
      drawValue(title, message, String(*val));
      if(cb != nullptr) {
        cb(*val);
      }
    }
  }
  encoder.setAccelerationEnabled(false);
  settingsChanged = settingsChanged | stat;
  return stat;
}

bool showInputDialog(const char* title, const char* PROGMEM message, int* val, int16_t min, int16_t max, iCallback cb, int16_t increment) {
  bool stat = true;
  int16_t turn;
  uint8_t wheelBtn;
  bool isHeld, isClicked;

  if(cb == nullptr && (min==0 && max==1)) {
    // if there's no callback set, and min, max equals 0,1 don't show the dialog
    // just return the inverted value (used for HI / LO)
    *val = (*val==min ? max : min);
    settingsChanged = settingsChanged | stat;
    return true;
  }
  debounceButton();
  encoder.setAccelerationEnabled(true);
  drawValue(title, message, String(*val));
  if(cb != nullptr) {
    cb(*val);
  }

  while(1) {
    getInput(&turn, &wheelBtn, &isHeld, &isClicked);
    if(isHeld || isClicked) {
      stat = isHeld ? false : true;
      break;
    }
    if(turn != 0) {
      *val += (turn*increment);
      if(turn < 0) {
        if(*val < min) {
          *val = min;
          beep(1);
        }
      }
      else if(turn > 0) {
        if(*val > max) {
          *val = max;
          beep(1);
        }
      }
      drawValue(title, message, String(*val));
      if(cb != nullptr) {
        cb(*val);
      }
    }
  }
  //__debug(PSTR("Stopped %d"),stat);
  encoder.setAccelerationEnabled(false);
  settingsChanged = settingsChanged | stat;
  return stat;
}

bool showInputDialog(const char* title, const char* PROGMEM message, bool* val, bCallback cb) {
  bool stat = true;
  int16_t turn;
  uint8_t wheelBtn;
  bool isHeld, isClicked;
  char _yes[5], _no[5];

  if(cb == nullptr) {
    // if there's no callback set, don't show the dialog
    // just return the inverted value
    *val = !*val;
    settingsChanged = settingsChanged | stat;
    return true;
  }
  sprintf_P(_yes, P_Yes);
  sprintf_P(_no, P_No);

  debounceButton();
  drawValue(title, message, String(*val ? _yes : _no));
  if(cb != nullptr) {
    cb(*val);
  }

  while(1) {
    getInput(&turn, &wheelBtn, &isHeld, &isClicked);
    if(isHeld || isClicked) {
      stat = isHeld ? false : true;
      break;
    }
    if(turn == 0)
      continue;
    *val = !*val;
    drawValue(title, message, String(*val ? _yes : _no));
    if(cb != nullptr) {
        cb(*val);
    }

  }
  settingsChanged = settingsChanged | stat;
  return stat;
}

bool showInputDialog(const char* title, const char* PROGMEM message, int* val, String list, iCallback cb, bool valIsIndex) {
  bool stat = true;
  int16_t turn;
  uint8_t wheelBtn;
  bool isHeld, isClicked;
  uint8_t opt = 0;
  char* options[16];

  debounceButton();
  uint8_t lineCnt = splitStringLines(options, (int)(sizeof(options) / sizeof(options[0])), list.c_str());

  if(lineCnt==0)
    return false;

    if(valIsIndex) {
      opt = *val;
    }
    else {
      for(uint8_t i=0; i< lineCnt; i++) {
        if(String(options[i]) == String(*val)) {
            opt = i;
            //__debug(PSTR("Current selection: %s"), options[i]);
        }
      }
    }
  drawValue(title, message, String(options[opt]));

  while(1) {
    getInput(&turn, &wheelBtn, &isHeld, &isClicked);
    if(isHeld || isClicked) {
      stat = isHeld ? false : true;
      break;
    }
    if(turn != 0) {
      if(turn == -1) {
        if(opt > 0)
          opt--;
        else opt = lineCnt-1;
      }
      else if(turn == 1) {
        if(opt < lineCnt-1)
          opt++;
        else opt = 0;
      }
      drawValue(title, message, String(options[opt]));
      if(cb != nullptr && valIsIndex) {
        cb(opt);
      }
    }
  }
  if(valIsIndex)
    *val = opt;
  else
    *val = atoi(options[opt]);
  settingsChanged = settingsChanged | stat;
  return stat;
}

bool showInputDialog(const char* title, const char* PROGMEM message, unsigned long* val, String list) {
  bool stat = true;
  int16_t turn;
  uint8_t wheelBtn;
  bool isHeld, isClicked;
  uint8_t opt = 0;
  char* options[10];

  debounceButton();
  uint8_t lineCnt = splitStringLines(options, (int)ArraySize(options), list.c_str());

  if(lineCnt==0)
    return false;

  for(uint8_t i=0; i< lineCnt; i++) {
    if(String(options[i]) == String(*val)) {
        opt = i;
        //__debug(PSTR("Current selection: %s"), options[i]);
    }
  }
  drawValue(title, message, String(options[opt]));

  while(1) {
    getInput(&turn, &wheelBtn, &isHeld, &isClicked);
    if(isHeld || isClicked) {
      stat = isHeld ? false : true;
      break;
    }
    if(turn != 0) {
      if(turn == -1) {
        if(opt > 0)
          opt--;
        else opt = lineCnt-1;
      }
      else if(turn == 1) {
        if(opt < lineCnt-1)
          opt++;
        else opt = 0;
      }
      drawValue(title, message, String(options[opt]));
    }
  }
  *val = strtol(options[opt], nullptr, 10);
  settingsChanged = settingsChanged | stat;
  return stat;
}

