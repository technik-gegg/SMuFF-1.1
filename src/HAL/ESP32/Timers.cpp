/**
 * SMuFF Firmware
 * Copyright (C) 2019-2021 Technik Gegg
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

#ifdef __ESP32__

/*
 * ESP32 HAL timers handling
 */
#include "timers.h"

static ESP32_timer_t timers[] = {
  { .timer = nullptr,
    .mux = portMUX_INITIALIZER_UNLOCKED,
    .serviceFunPtr = nullptr },
  { .timer = nullptr,
    .mux = portMUX_INITIALIZER_UNLOCKED,
    .serviceFunPtr = nullptr },
  { .timer = nullptr,
    .mux = portMUX_INITIALIZER_UNLOCKED,
    .serviceFunPtr = nullptr },
  { .timer = nullptr,
    .mux = portMUX_INITIALIZER_UNLOCKED,
    .serviceFunPtr = nullptr }
};

void timerISRService(ZTimer::IsrTimer t) {
  if (timers[t].serviceFunPtr == nullptr)
    return;

  portENTER_CRITICAL_ISR(&timers[t].mux);
  timers[t].serviceFunPtr();
  portEXIT_CRITICAL_ISR(&timers[t].mux);
}

void IRAM_ATTR ISR1() {
  timerISRService(ZTimer::IsrTimer::ZTIMER1);
}

void IRAM_ATTR ISR2() {
  timerISRService(ZTimer::IsrTimer::ZTIMER2);
}

void IRAM_ATTR ISR3() {
  timerISRService(ZTimer::IsrTimer::ZTIMER3);
}

void IRAM_ATTR ISR4() {
  timerISRService(ZTimer::IsrTimer::ZTIMER4);
}

void ZTimer::setupTimer(IsrTimer timer, uint16_t prescaler, timerVal_t compare) {
  if (timer < ZTIMER1 || (unsigned)timer >= COUNT(timers))
    return;

  _timer = timer;

  noInterrupts();
  hw_timer_t* hwTimer = timers[_timer].timer = timerBegin(_timer, prescaler, true);
  timerAttachInterrupt(hwTimer, ((void (*[])(void)) { &ISR1, &ISR2, &ISR3, &ISR4 })[_timer], true);
  timerAlarmWrite(hwTimer, compare, true);
  interrupts();
}

void ZTimer::setupTimerHook(void (*function)(void)) {
  if (_timer != ZUNDEFINED)
    timers[_timer].serviceFunPtr = function;
}

timerVal_t ZTimer::getOverflow() {
  return (_timer != ZUNDEFINED) ? timerAlarmReadMicros(timers[_timer].timer) : 0;
}

void ZTimer::setOverflow(timerVal_t value) {
  if (_timer != ZUNDEFINED)
    timerAlarmWrite(timers[_timer].timer, value, true);
}

void ZTimer::setCompare(timerVal_t value) {
  // Not implemented
}

void ZTimer::setCounter(timerVal_t value) {
  // Not implemented
}

void ZTimer::startTimer() {
  if (_timer != ZUNDEFINED)
    timerAlarmEnable(timers[_timer].timer);
}

void ZTimer::stopTimer() {
  if (_timer != ZUNDEFINED)
    timerAlarmDisable(timers[_timer].timer);
}

#endif // __ESP32__
