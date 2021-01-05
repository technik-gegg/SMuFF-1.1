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

#ifdef __AVR__

/*
 * AVR HAL timers handling
 */
#include "timers.h"

static AVR_timer_t timers[] = {
  { .TIMSK = TIMSK1,
    .IEMask = _BV(OCIE1A),
    .OCRxA = OCR1A,
    .TCNTx = TCNT1,
    .serviceFunPtr = nullptr },
  { .TIMSK = nullptr },   // Not used
  { .TIMSK = TIMSK3,
    .IEMask = _BV(OCIE3A),
    .OCRxA = OCR3A,
    .TCNTx = TCNT3,
    .serviceFunPtr = nullptr },
  { .TIMSK = TIMSK4,
    .IEMask = _BV(OCIE4A),
    .OCRxA = OCR4A,
    .TCNTx = TCNT4,
    .serviceFunPtr = nullptr },
  { .TIMSK = TIMSK5,
    .IEMask = _BV(OCIE5A),
    .OCRxA = OCR5A,
    .TCNTx = TCNT5,
    .serviceFunPtr = nullptr }
};

ISR(TIMER1_COMPA_vect) {
  if(__timer1Hook != nullptr)
    __timer1Hook();
}

ISR(TIMER3_COMPA_vect) {
  if(__timer3Hook != nullptr)
    __timer3Hook();
}

ISR(TIMER4_COMPA_vect) {
  if(__timer4Hook != nullptr)
    __timer4Hook();
}

ISR(TIMER5_COMPA_vect) {
  if(__timer5Hook != nullptr)
    __timer5Hook();
}

void ZTimer::setupTimer(IsrTimer timer, TimerPrescaler prescaler) {
  if (timer < ZTIMER1 || (unsigned)timer >= COUNT(timers) || timers[timer].TIMSK == nullptr)
    return;

  _timer = timer;

  stopTimer();
  noInterrupts();
  switch (_timer) {
    case ZTIMER1:
      TCCR1A = 0;
      TCCR1B = prescaler | _BV(WGM12);    // CTC mode
      break;
    case ZTIMER3:
      TCCR3A = 0;
      TCCR3B = prescaler | _BV(WGM32);    // CTC mode
      break;
    case ZTIMER4:
      TCCR4A = 0;
      TCCR4B = prescaler | _BV(WGM42);    // CTC mode
      break;
    case ZTIMER5:
      TCCR5A = 0;
      TCCR5B = prescaler | _BV(WGM52);    // CTC mode
      break;
  }
  interrupts();
}

  void ZTimer::setupTimerHook(void (*function)(void)) {
    if (_timer != ZUNDEFINED)
      timers[_timer].serviceFunPtr = function;
  }

  timerVal_t ZTimer::getOverflow() {
    return (_timer != ZUNDEFINED) ? timers[_timer].OCRxA : 0;
  }

  void ZTimer::setOverflow(timerVal_t value) {
    if (_timer != ZUNDEFINED)
      timers[_timer].OCRxA = value;
  }

  void ZTimer::setCompare(timerVal_t value) {
    // Not implemented
  }

  void ZTimer::setCounter(timerVal_t value) {
    if (_timer != ZUNDEFINED)
      timers[_timer].TCNTx = value
  }

  void ZTimer::startTimer() {
    if (_timer != ZUNDEFINED)
      timers[_timer].TIMSK |= timers[_timer].IEMask;
  }

  void ZTimer::stopTimer() {
    if (_timer != ZUNDEFINED)
      timers[_timer].TIMSK &= ~timers[_timer].IEMask;
  }

#endif // __AVR__
