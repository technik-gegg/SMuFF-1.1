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

volatile bool leoNerdBlinkState  = false;
volatile bool leoNerdBlinkGreen  = false;
volatile bool leoNerdBlinkRed    = false;
volatile bool sendingStatesToggle = false;

void sendStates(bool override) {
  if(!initDone)
    return;
  
  if(!override && (parserBusy || sendingResponse)) {
    __debugS(DEV, PSTR("Parser busy: %s  Sending Response: %s"), parserBusy ? P_Yes : P_No, sendingResponse ? P_Yes : P_No);
    return;
  }
  if(override)
    refreshStatus();
  // send status of endstops and current tool to all listeners, if configured
  if(!sendingResponse && smuffConfig.sendPeriodicalStats && initDone) {
    if(parserBusy && !override)
      return;
    printPeriodicalState(0);
    if(CAN_USE_SERIAL1 && (smuffConfig.hasPanelDue != 1 && smuffConfig.duet3Dport != 1))
      printPeriodicalState(1);
    if(CAN_USE_SERIAL2 && (smuffConfig.hasPanelDue != 2 && smuffConfig.duet3Dport != 2))
      printPeriodicalState(2);
    if(CAN_USE_SERIAL3 && (smuffConfig.hasPanelDue != 3 && smuffConfig.duet3Dport != 3))
      printPeriodicalState(3);
    sendingStatesToggle = !sendingStatesToggle;
  }
  // Add your periodical code here
}

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
  refreshStatus();     // refresh main screen
  // Add your periodical code here
}

void every1s() {
  #if defined(USE_LEONERD_DISPLAY)
    if(leoNerdBlinkGreen || leoNerdBlinkRed) {
      leoNerdBlinkState = !leoNerdBlinkState;
      if(leoNerdBlinkGreen)
        encoder.setLED(LN_LED_GREEN, leoNerdBlinkState);
      if(leoNerdBlinkRed)
        encoder.setLED(LN_LED_RED, leoNerdBlinkState);
    }
    else {
      leoNerdBlinkState = false;
      encoder.setLED(LN_LED_GREEN, false);
      encoder.setLED(LN_LED_RED, false);
    }
  #endif
  // send states to WebInterface
  if(smuffConfig.webInterface) {
    sendStates();
  }
  // Add your periodical code here
}

void every2s() {
  if(!smuffConfig.webInterface) {
    sendStates();
  }
  // Add your periodical code here
}

static long fiveSecCounter = 0;
void every5s() {
  fiveSecCounter++;
  if(fiveSecCounter % 12 == 0)
      showFreeMemory();       // dump memory info once a minute
  // Add your periodical code here
}
