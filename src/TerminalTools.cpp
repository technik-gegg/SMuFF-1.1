/**
 * SMuFF Firmware
 * Copyright (C) 2019-2024 Technik Gegg
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

/*
void terminalDrawFrame(bool clear) {
  if(!smuffConfig.menuOnTerminal)
    return;

#if defined(USE_TERMINAL_MENUS)
  char vert[TERM_LINE_WIDTH+5];
  char horz[TERM_LINE_WIDTH+5];

  memset(horz, clear ? 0x20 : TERM_HORZLINE_CHR, TERM_LINE_WIDTH+4);
  memset(vert, 0x20, TERM_LINE_WIDTH+4);
  horz[TERM_LINE_WIDTH+4] = 0;
  vert[TERM_LINE_WIDTH+4] = 0;

  if(!clear) {
    horz[0] = (uint8_t)TERM_CORNERUL_CHR;  horz[TERM_LINE_WIDTH+3] = (uint8_t)TERM_CORNERUR_CHR;
    vert[0] = (uint8_t)TERM_VERTLINE_CHR;  vert[TERM_LINE_WIDTH+3] = (uint8_t)TERM_VERTLINE_CHR;
  }

  __terminal(P_SendTermAt, TERM_OFFS_Y, TERM_OFFS_X-1, horz);
  for(uint8_t i=1; i<= TERM_LINES; i++)
    __terminal(P_SendTermAt, TERM_OFFS_Y+i, TERM_OFFS_X-1, vert);

  if(!clear) {
    horz[0] = (uint8_t)TERM_CORNERLL_CHR;  horz[TERM_LINE_WIDTH+3] = (uint8_t)TERM_CORNERLR_CHR;
  }
  __terminal(P_SendTermAt, TERM_OFFS_Y+TERM_LINES+1, TERM_OFFS_X-1, horz);
#endif
}

void terminalDrawSeparator(uint8_t y, uint8_t x, uint8_t color) {
#if defined(USE_TERMINAL_MENUS)
  char ln[TERM_LINE_WIDTH+1];
  memset(ln, TERM_SEPARATOR_CHR, TERM_LINE_WIDTH);
  ln[TERM_LINE_WIDTH] = 0;
  __terminal(P_SendTermAttr, color);
  __terminal(P_SendTermAt, y+TERM_OFFS_Y, x+TERM_OFFS_X, ln);
#endif
}

void terminalClear(bool drawFrame) {
  if(!smuffConfig.menuOnTerminal)
    return;
#if defined(USE_TERMINAL_MENUS)
  static bool cursorSaved = false;
  if(drawFrame) {
    if(!cursorSaved)
      __terminal(P_SendTermCsrSave);  // save cursor position and attributes
    cursorSaved = true;
    __terminal(P_SendTermCsrHide);    // turn cursor off
    terminalDrawFrame();
  }
  else {
    terminalDrawFrame(true);
    if(cursorSaved)
      __terminal(P_SendTermCsrRestore);     // restore cursor position and switch it on again
    __terminal(P_SendTermCsrShow);
    cursorSaved = false;
  }
#endif
}

void terminalSend(uint8_t y, uint8_t x, const char* str, bool isCenter, uint8_t isInvert, bool clearLine) {
  if(!smuffConfig.menuOnTerminal)
    return;

#if defined(USE_TERMINAL_MENUS)
  char txt[TERM_LINE_WIDTH+1];
  memset(txt, ' ', TERM_LINE_WIDTH);
  uint8_t len = strlen(str);
  if(len > ArraySize(txt))
    len = ArraySize(txt);
  if(isCenter) {
    uint8_t pos = (TERM_LINE_WIDTH - len)/2;
    strncpy(&txt[pos], str, len);
  }
  else {
    strncpy(txt, str, len);
    if(!clearLine)
      txt[len] = 0;
  }
  txt[TERM_LINE_WIDTH] = 0;

  uint8_t color = 0, color2 = TERM_FGC_NONE;
  switch(isInvert) {
    case 1: color = TERM_INVERTED; break;
    case 2: color = TERM_FGC_CYAN; break;
    case 3: color = TERM_BGC_CYAN; break;
    case 4: color = TERM_FGC_MAGENTA; break;
    case 5: color = TERM_BGC_MAGENTA; break;
    case 6: color = TERM_UNDERLINE; color2 = TERM_FGC_CYAN; break;
  }
  if(clearLine) {
    char clr[TERM_LINE_WIDTH+2];
    memset(clr, 0x20, TERM_LINE_WIDTH+1);
    clr[TERM_LINE_WIDTH+1] = 0;
    __terminal(P_SendTermAt, y+TERM_OFFS_Y, TERM_OFFS_X, clr);
  }
  if(*txt == 0x1d) {
    terminalDrawSeparator(y, x, color);
  }
  else {
    __terminal(P_SendTermAttr,color);                               // set main attribute/color
    if(color2 != TERM_FGC_NONE) __terminal(P_SendTermAttr, color2); // set 2nd attribute/color if applicable
    __terminal(P_SendTermAt, y+TERM_OFFS_Y, x+TERM_OFFS_X, txt);    // print at position
  }
  __terminal(P_SendTermAttr, 0); // reset attributes
#endif
}

uint8_t terminalSendLines(uint8_t y, uint8_t x, const char* str, bool isCenter, uint8_t isInvert, bool clearLine) {

  if(!smuffConfig.menuOnTerminal)
    return 0;
#if defined(USE_TERMINAL_MENUS)
  char* lines[TERM_LINES+1];
  uint8_t ln = y;
  uint8_t lineCnt = splitStringLines(lines, ArraySize(lines), str);
  if(lineCnt==0)
    return ln;

  for(uint8_t i=0; i< lineCnt; i++) {
    if(strlen(lines[i])>0)
      terminalSend(ln, x, lines[i], isCenter, isInvert, clearLine);
    ln++;
  }
  return ln;
#else
  return y;
#endif
}
*/