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
  isrFastLEDTimerHandler();
  // Add your periodical code here
}

void every50ms() {
  // Add your periodical code here
}

void every100ms() {
  // Add your periodical code here
}

void every250ms() {
  refreshStatus(true, false);     // refresh main screen
  // Add your periodical code here
}

void every500ms() {
  if(smuffConfig.webInterface) {
    sendStates();
  }
  // Add your periodical code here
}

void sendStates() {
  /*
  if(parserBusy || sendingResponse) {
    __debugS(PSTR("Parser busy: %s  Sending Response: %s"), parserBusy ? P_Yes : P_No, sendingResponse ? P_Yes : P_No);
  }
  */
  // send status of endstops and current tool to all listeners, if configured
  if(!sendingResponse && smuffConfig.sendPeriodicalStats && initDone && !parserBusy) {
    printPeriodicalState(0);
    if(CAN_USE_SERIAL1)                                   printPeriodicalState(1);
    if(CAN_USE_SERIAL2 && smuffConfig.hasPanelDue != 2)   printPeriodicalState(2);
    if(CAN_USE_SERIAL3 && smuffConfig.hasPanelDue != 3)   printPeriodicalState(3);
  }
  // Add your periodical code here
}

volatile bool leoNerdBlinkState  = false;
volatile bool leoNerdBlinkGreen  = false;
volatile bool leoNerdBlinkRed    = false;

void every1s() {
  #if defined(USE_LEONERD_DISPLAY)
  if(leoNerdBlinkGreen || leoNerdBlinkRed) {
    leoNerdBlinkState = !leoNerdBlinkState;
    if(leoNerdBlinkGreen)
      encoder.setLED(LED_GREEN, leoNerdBlinkState);
    if(leoNerdBlinkRed)
      encoder.setLED(LED_RED, leoNerdBlinkState);
  }
  else {
    leoNerdBlinkState = false;
    encoder.setLED(LED_GREEN, false);
    encoder.setLED(LED_RED, false);
  }
  #endif
  // Add your periodical code here
}

void every2s() {
  if(!smuffConfig.webInterface) {
    sendStates();
  }
}

void every5s() {
  // Add your periodical code here
}
