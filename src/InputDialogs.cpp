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


void drawValue(const char* title, const char* PROGMEM message, String val) {
    char tmp[256];
    char msg[128];
    sprintf_P(msg, message);
    sprintf(tmp,"%s\n \n%-12s%5s", title, msg, val.c_str());
    drawUserMessage(String(tmp));
    checkSerialPending();   // allow serial commands to be read and handled
}

void getEncoderButton(int* turn, int* button, bool* isHeld, bool* isClicked) {
  *turn = encoder.getValue();
  *button = encoder.getButton();
  *isHeld = false;
  *isClicked = false;
  if(*button != 0) {
    if(*button == ClickEncoder::Held) {
      *isHeld = true;
    }
    else if(*button == ClickEncoder::Clicked) {
      *isClicked = true;
    }
  }
}

bool showInputDialog(const char* title, const char* PROGMEM message, float* val, float min, float max, fCallback cb) {
  bool stat = true;
  float steps = 1.0F;
  int turn, btn;
  bool isHeld, isClicked;

  debounceButton();
  encoder.setAccelerationEnabled(true);
  drawValue(title, message, String(*val));
  if(cb != NULL) {
    cb(*val);
  }

  while(1) {
    getEncoderButton(&turn, &btn, &isHeld, &isClicked);
    if(isHeld || isClicked) {
      stat = isHeld ? false : true;
      break;
    }
    if(turn != 0) {
      *val += steps*turn;
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
      if(cb != NULL) {
        cb(*val);
      }
    }
  }
  encoder.setAccelerationEnabled(false);
  return stat;
}

bool showInputDialog(const char* title, const char* PROGMEM message, int* val, int min, int max, iCallback cb) {
  bool stat = true;
  int turn, btn;
  bool isHeld, isClicked;

  debounceButton();
  encoder.setAccelerationEnabled(true);
  drawValue(title, message, String(*val));
  if(cb != NULL) {
    cb(*val);
  }
  
  while(1) {
    getEncoderButton(&turn, &btn, &isHeld, &isClicked);
    if(isHeld || isClicked) {
      stat = isHeld ? false : true;
      break;
    }
    if(turn != 0) {
      *val += turn;
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
      if(cb != NULL) {
        cb(*val);
      }
    }
  }
  //__debug(PSTR("Stopped %d"),stat);
  encoder.setAccelerationEnabled(false);
  return stat;
}

bool showInputDialog(const char* title, const char* PROGMEM message, bool* val, bCallback cb) {
  bool stat = true;
  int turn, btn;
  bool isHeld, isClicked;
  char _yes[10], _no[10];
  
  sprintf_P(_yes, P_Yes);
  sprintf_P(_no, P_No);

  debounceButton();
  drawValue(title, message, String(*val ? _yes : _no));
  if(cb != NULL) {
    cb(*val);
  }
 
  while(1) {
    getEncoderButton(&turn, &btn, &isHeld, &isClicked);
    if(isHeld || isClicked) {
      stat = isHeld ? false : true;
      break;
    }
    if(turn == 0)
      continue;
    *val = !*val;
    drawValue(title, message, String(*val ? _yes : _no));
    if(cb != NULL) {
        cb(*val);
    }

  }
  return stat;
}

bool showInputDialog(const char* title, const char* PROGMEM message, unsigned long* val, String list) {
  bool stat = true;
  int turn, btn;
  bool isHeld, isClicked;
  int opt = 0;
  char* options[10];

  debounceButton();
  int lineCnt = splitStringLines(options, 10, list.c_str());

  if(lineCnt==0)
    return false;
  
  for(int i=0; i< lineCnt; i++) {
    if(String(options[i]) == String(*val)) {
        opt = i;
        //__debug(PSTR("Current selection: %s"), options[i]);
    }
  }
  drawValue(title, message, String(options[opt]));

  while(1) {
    getEncoderButton(&turn, &btn, &isHeld, &isClicked);
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
  *val = strtol(options[opt], NULL, 10);
  return stat;
}

