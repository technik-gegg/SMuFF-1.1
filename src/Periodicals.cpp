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

void every10ms() {
  // Add your periodical code here
}

void every20ms() {
  // Add your periodical code here
}

void every50ms() {
  // Add your periodical code here
}

void every100ms() {
  // Add your periodical code here
}

void every250ms() {
  // Add your periodical code here
}

void every500ms() {
  // Add your periodical code here
}

bool leoNerdBlinkState  = false;
bool LeoNerdBlinkGreen  = false;
bool LeoNerdBlinkRed    = false;

void every1s() {
  #if defined(USE_LEONERD_DISPLAY)
  if(LeoNerdBlinkGreen || LeoNerdBlinkRed) {
    leoNerdBlinkState = !leoNerdBlinkState;
    encoder.setLED((LeoNerdBlinkGreen ? LED_GREEN : LED_RED), leoNerdBlinkState);
  }
  else {
    if(leoNerdBlinkState) {
      leoNerdBlinkState = false;
      encoder.setLED(LED_GREEN, false);
      encoder.setLED(LED_RED, false);
    }
  }
  #endif
  // Add your periodical code here
}

void every2s() {
  if(parserBusy || sendingResponse) {
    __debug(PSTR("Parser busy: %s  Sending Response: %s"), parserBusy ? P_Yes : P_No, sendingResponse ? P_Yes : P_No);
  }
  // send status of endstops and current tool to all listeners, if configured
  if(!sendingResponse && smuffConfig.sendPeriodicalStats && initDone && !parserBusy) {
    printPeriodicalState(0);
    if(CAN_USE_SERIAL1)                                   printPeriodicalState(1);
    if(CAN_USE_SERIAL2 && smuffConfig.hasPanelDue != 2)   printPeriodicalState(2);
    if(CAN_USE_SERIAL3 && smuffConfig.hasPanelDue != 3)   printPeriodicalState(3);
  }
  // Add your periodical code here
}

void every5s() {
  // Add your periodical code here
}

