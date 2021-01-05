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

#ifdef __STM32F1__

/*
 * STM32F1 HAL timers handling
 */
#include "timers.h"

static STM32_timer_t timers[] = {
  { .timer = HardwareTimer(1),
    .serviceFunPtr = nullptr },
  { .timer = HardwareTimer(2),
    .serviceFunPtr = nullptr },
  { .timer = HardwareTimer(3),
    .serviceFunPtr = nullptr },
  { .timer = HardwareTimer(4),
    .serviceFunPtr = nullptr },
#ifdef STM32_HIGH_DENSITY
  { .timer = HardwareTimer(5),
    .serviceFunPtr = nullptr },
  { .timer = HardwareTimer(6),
    .serviceFunPtr = nullptr },
  { .timer = HardwareTimer(7),
    .serviceFunPtr = nullptr },
  { .timer = HardwareTimer(8),
    .serviceFunPtr = nullptr }
#endif
};

void timerISRService(ZTimer::IsrTimer t) {
  if (timers[t].serviceFunPtr != nullptr)
    timers[t].serviceFunPtr();
}

void ISR1() {
  timerISRService(ZTimer::IsrTimer::ZTIMER1);
}

void ISR2() {
  timerISRService(ZTimer::IsrTimer::ZTIMER2);
}

void ISR3() {
  timerISRService(ZTimer::IsrTimer::ZTIMER3);
}

void ISR4() {
  timerISRService(ZTimer::IsrTimer::ZTIMER4);
}

#ifdef STM32_HIGH_DENSITY
void ISR5() {
  timerISRService(ZTimer::IsrTimer::ZTIMER5);
}

void ISR6() {
  timerISRService(ZTimer::IsrTimer::ZTIMER6);
}

void ISR7() {
  timerISRService(ZTimer::IsrTimer::ZTIMER7);
}

void ISR8() {
  timerISRService(ZTimer::IsrTimer::ZTIMER8);
}
#endif

void ZTimer::setupTimer(IsrTimer timer, uint8_t channel, uint16_t prescaler, uint16_t compare) {
  if (timer < ZTIMER1 || (unsigned)timer >= COUNT(timers))
    return;

  _timer = timer;
  _channel = channel;

  HardwareTimer * hwTimer = &timers[_timer].timer;
  hwTimer->pause();
  noInterrupts();
  if (_timer == ZTIMER6 || _timer == ZTIMER7) {
    // since these timers don't have a compare mode, we're using
    // the compare value as a period (in uS)
    hwTimer->setPeriod(compare);
  }
  else {
    hwTimer->setMode(channel, TIMER_OUTPUT_COMPARE);
    hwTimer->setPrescaleFactor(prescaler);
    hwTimer->setCompare(channel, compare);
  }
  hwTimer->attachInterrupt(channel, ((void (*[])(void)) { &ISR1, &ISR2, &ISR3, &ISR4,
#ifdef STM32_HIGH_DENSITY
    &ISR5, &ISR6, &ISR7, &ISR8,
#endif
    })[_timer]);
  interrupts();
}

void ZTimer::setupTimerHook(void (*function)(void)) {
  if (_timer != ZUNDEFINED)
    timers[_timer].serviceFunPtr = function;
}

timerVal_t ZTimer::getOverflow() {
  return (_timer != ZUNDEFINED) ? timers[_timer].timer.getOverflow() : 0;
}

void ZTimer::setOverflow(timerVal_t value) {
  if (_timer != ZUNDEFINED)
    timers[_timer].timer.setOverflow(value);
}

void ZTimer::setCompare(timerVal_t value) {
  if (_timer != ZUNDEFINED)
    timers[_timer].timer.setCompare(_channel, value);
}

void ZTimer::setCounter(timerVal_t value) {
  if (_timer != ZUNDEFINED)
    timers[_timer].timer.setCount(value);
}

void ZTimer::startTimer() {
  if (_timer != ZUNDEFINED) {
    HardwareTimer * hwTimer = &timers[_timer].timer;
    hwTimer->refresh();
    hwTimer->resume();
  }
}

void ZTimer::stopTimer() {
  if (_timer != ZUNDEFINED)
    timers[_timer].timer.pause();
}

#endif // __STM32F1__
