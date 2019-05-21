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

/*
 * Module for timer & ISR routines
 */
 
#include "ZTimerLib.h"

static void (*__timer1Hook)(void) = NULL;
static void (*__timer3Hook)(void) = NULL;
static void (*__timer4Hook)(void) = NULL;
static void (*__timer5Hook)(void) = NULL;

void ZTimer::setupTimer(IsrTimer timer, TimerPrescaler prescaler) {
  _timer = timer;

  stopTimer();
  noInterrupts();
  switch(_timer) {
    case TIMER1:
      TCCR1A = 0;
      TCCR1B = prescaler;
      TCCR1B |= _BV(WGM12);                     // CTC mode
      break;
    case TIMER3:
      TCCR3A = 0;
      TCCR3B = prescaler;
      TCCR3B |= _BV(WGM32);                     // CTC mode
      break;
    case TIMER4:
      TCCR4A = 0;
      TCCR4B = prescaler;
      TCCR4B |= _BV(WGM42);                     // CTC mode
      break;
    case TIMER5:
      TCCR5A = 0;
      TCCR5B = prescaler;
      TCCR5B |= _BV(WGM52);                     // CTC mode
      break;
  }
 
  setNextInterruptInterval(1000);
  interrupts();
}

ISR(TIMER1_COMPA_vect) {
  if(__timer1Hook != NULL)
    __timer1Hook();
}

ISR(TIMER3_COMPA_vect) {
  if(__timer3Hook != NULL)
    __timer3Hook();
}

ISR(TIMER4_COMPA_vect) {
  if(__timer4Hook != NULL)
    __timer4Hook();
}

ISR(TIMER5_COMPA_vect) {
  if(__timer5Hook != NULL)
    __timer5Hook();
}

void ZTimer::setupTimerHook(void (*function)(void))
{
  switch(_timer) {
    case TIMER1: __timer1Hook = function; break;
    case TIMER3: __timer3Hook = function; break;
    case TIMER4: __timer4Hook = function; break;
    case TIMER5: __timer5Hook = function; break;
  }
}

void ZTimer::setNextInterruptInterval(unsigned int interval) {
  stopTimer();
  switch(_timer) {
    case TIMER1: OCR1A = interval; TCNT1 = 0; break;
    case TIMER3: OCR3A = interval; TCNT3 = 0; break;
    case TIMER4: OCR4A = interval; TCNT4 = 0; break;
    case TIMER5: OCR5A = interval; TCNT5 = 0; break;
  }
  startTimer();
}

unsigned int ZTimer::getOCRxA() {
  switch(_timer) {
    case TIMER1: return OCR1A;
    case TIMER3: return OCR3A;
    case TIMER4: return OCR4A;
    case TIMER5: return OCR5A;
  }
  return 0;
}

void ZTimer::setOCRxA(unsigned int value) {
  switch(_timer) {
    case TIMER1: OCR1A = value; break;
    case TIMER3: OCR3A = value; break;
    case TIMER4: OCR4A = value; break;
    case TIMER5: OCR5A = value; break;
  }
}

void ZTimer::setTCNTx(unsigned int value) {
  switch(_timer) {
    case TIMER1: TCNT1 = value; break;
    case TIMER3: TCNT3 = value; break;
    case TIMER4: TCNT4 = value; break;
    case TIMER5: TCNT5 = value; break;
  }
}

void ZTimer::startTimer() {
  switch(_timer) {
    case TIMER1: TIMSK1 |=  _BV(OCIE1A); break;
    case TIMER3: TIMSK3 |=  _BV(OCIE3A); break;
    case TIMER4: TIMSK4 |=  _BV(OCIE4A); break;
    case TIMER5: TIMSK5 |=  _BV(OCIE5A); break;
  }
}

void ZTimer::stopTimer() {
  switch(_timer) {
    case TIMER1: TIMSK1 &= ~_BV(OCIE1A); break;
    case TIMER3: TIMSK3 &= ~_BV(OCIE3A); break;
    case TIMER4: TIMSK4 &= ~_BV(OCIE4A); break;
    case TIMER5: TIMSK5 &= ~_BV(OCIE5A); break;
  }
}
