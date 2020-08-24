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

volatile bool       interval10ms;     // set each time when function is called; reset yourself if needed 
volatile bool       interval20ms;
volatile bool       interval50ms; 
volatile bool       interval100ms; 
volatile bool       interval250ms; 
volatile bool       interval500ms; 
volatile bool       interval1s; 
volatile bool       interval2s; 
volatile bool       interval5s; 
static volatile uint32_t last10ms;
static volatile uint32_t last20ms;
static volatile uint32_t last50ms;
static volatile uint32_t last100ms; 
static volatile uint32_t last250ms; 
static volatile uint32_t last500ms; 
static volatile uint32_t last1s; 
static volatile uint32_t last2s; 
static volatile uint32_t last5s; 

void every10ms() {
  if(millis()-last10ms < 10)
    return;
  last10ms = millis();
  interval10ms = true;
  // Add your periodical code here
}

void every20ms() {
  if(millis()-last20ms < 20)
    return;
  last20ms = millis();
  interval20ms = true;
  // Add your periodical code here
}

void every50ms() {
  if(millis()-last50ms < 50)
    return;
  last50ms = millis();
  interval50ms = true;
  // Add your periodical code here
}

void every100ms() {
  if(millis()-last100ms < 100)
    return;
  last100ms = millis();
  interval100ms = true;
  // Add your periodical code here 
}

void every250ms() {
  if(millis()-last250ms < 250)
    return;
  last250ms = millis();
  interval250ms = true;
  // Add your periodical code here
}

void every500ms() {
  if(millis()-last500ms < 500)
    return;
  last500ms = millis();
  interval500ms = true;
  // Add your periodical code here 
}

void every1s() {
  if(millis()-last1s < 1000)
    return;
  last1s = millis();
  interval1s = true;
  // Add your periodical code here 
}

void every2s() {
  if(millis()-last2s < 2000)
    return;
  last2s = millis();
  interval2s = true;

  if(parserBusy || sendingResponse) {
    __debug(PSTR("Parser busy: %s  Sending Response: %s"), parserBusy ? P_Yes : P_No, sendingResponse ? P_Yes : P_No);
  }
  // send status of endstops and current tool to all listeners, if configured
  if(!sendingResponse && smuffConfig.sendPeriodicalStats && enablePeriStat && !parserBusy) {
    printPeriodicalState(0);
    if(CAN_USE_SERIAL1)                                   printPeriodicalState(1);
    if(CAN_USE_SERIAL2 && smuffConfig.hasPanelDue != 2)   printPeriodicalState(2);
    if(CAN_USE_SERIAL3 && smuffConfig.hasPanelDue != 3)   printPeriodicalState(3);
  }
  // Add your periodical code here 
}

void every5s() {
  if(millis()-last5s < 5000)
    return;
  last5s = millis();
  interval5s = true;
  // Add your periodical code here
}

