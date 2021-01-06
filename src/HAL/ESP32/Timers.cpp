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

static struct {
  hw_timer_t * timer;
  portMUX_TYPE mux;
  void (* serviceFunPtr)(void);
  } timers[] = {
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

void timerISRService(Timer::timerNum_t t) {
  if (timers[t].serviceFunPtr == nullptr)
    return;

  portENTER_CRITICAL_ISR(&timers[t].mux);
  timers[t].serviceFunPtr();
  portEXIT_CRITICAL_ISR(&timers[t].mux);
}

void IRAM_ATTR ISR1() {
  timerISRService(Timer::TIMER1);
}

void IRAM_ATTR ISR2() {
  timerISRService(Timer::TIMER2);
}

void IRAM_ATTR ISR3() {
  timerISRService(Timer::TIMER3);
}

void IRAM_ATTR ISR4() {
  timerISRService(Timer::TIMER4);
}

void Timer::setupTimer(timerNum_t timer, uint16_t prescaler, timerVal_t compare) {
  if (timer < TIMER1 || timer >= MAX_TIMERS)
    return;

  _timer = timer;

  noInterrupts();
  hw_timer_t* hwTimer = timers[_timer].timer = timerBegin(_timer, prescaler, true);
  timerAttachInterrupt(hwTimer, ((void (*[])(void)) { &ISR1, &ISR2, &ISR3, &ISR4 })[_timer], true);
  timerAlarmWrite(hwTimer, compare, true);
  interrupts();
}

void Timer::setupHook(void (*function)(void)) {
  if (_timer != UNDEFINED)
    timers[_timer].serviceFunPtr = function;
}

void Timer::setNextInterruptInterval(timerVal_t interval) {
  stop();
  setOverflow(interval);
  start();
}

timerVal_t Timer::getOverflow() {
  return (_timer != UNDEFINED) ? timerAlarmReadMicros(timers[_timer].timer) : 0;
}

void Timer::setOverflow(timerVal_t value) {
  if (_timer != UNDEFINED)
    timerAlarmWrite(timers[_timer].timer, value, true);
}

void Timer::start() {
  if (_timer != UNDEFINED)
    timerAlarmEnable(timers[_timer].timer);
}

void Timer::stop() {
  if (_timer != UNDEFINED)
    timerAlarmDisable(timers[_timer].timer);
}

#endif // __ESP32__
