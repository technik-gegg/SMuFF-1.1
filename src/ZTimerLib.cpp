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
#ifdef __STM32F1__
#include <libmaple/libmaple.h>
#endif

static void (*__timer1Hook)(void) = NULL;
static void (*__timer2Hook)(void) = NULL;
static void (*__timer3Hook)(void) = NULL;
static void (*__timer4Hook)(void) = NULL;
static void (*__timer5Hook)(void) = NULL;
static void (*__timer6Hook)(void) = NULL;
static void (*__timer7Hook)(void) = NULL;
static void (*__timer8Hook)(void) = NULL;

#if defined(__AVR__)
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
#endif

#if defined(__STM32F1__)
HardwareTimer hwTimer1(1);
HardwareTimer hwTimer2(2);
HardwareTimer hwTimer3(3);
HardwareTimer hwTimer4(4);
HardwareTimer hwTimer5(5);
HardwareTimer hwTimer6(6);
HardwareTimer hwTimer7(7);
HardwareTimer hwTimer8(8);

void ISR1() {
  if(__timer1Hook != NULL)
    __timer1Hook();
}

void ISR2() {
  if(__timer2Hook != NULL)
    __timer2Hook();
}

void ISR3() {
  if(__timer3Hook != NULL)
    __timer3Hook();
}

void ISR4() {
  if(__timer4Hook != NULL)
    __timer4Hook();

}

void ISR5() {
  if(__timer5Hook != NULL)
    __timer5Hook();
}

void ISR6() {
  if(__timer6Hook != NULL)
    __timer6Hook();
}

void ISR7() {
  if(__timer7Hook != NULL)
    __timer7Hook();
}

void ISR8() {
  if(__timer8Hook != NULL)
    __timer8Hook();
}
#endif

#if defined(__AVR__)
void ZTimer::setupTimer(IsrTimer timer, TimerPrescaler prescaler) {
  _timer = timer;

  stopTimer();
  noInterrupts();
  switch(_timer) {
    case ZTIMER1:
      TCCR1A = 0;
      TCCR1B = prescaler;
      TCCR1B |= _BV(WGM12);        // CTC mode
      break;
    case ZTIMER2:                 // 8-Bit Timer not used
      break;
    case ZTIMER3:
      TCCR3A = 0;
      TCCR3B = prescaler;
      TCCR3B |= _BV(WGM32);        // CTC mode
      break;
    case ZTIMER4:
      TCCR4A = 0;
      TCCR4B = prescaler;
      TCCR4B |= _BV(WGM42);        // CTC mode
      break;
    case ZTIMER5:
      TCCR5A = 0;
      TCCR5B = prescaler;
      TCCR5B |= _BV(WGM52);        // CTC mode
      break;
    case ZTIMER6:                 // not available on AVR
    case ZTIMER7:                 // not available on AVR
    case ZTIMER8:                 // not available on AVR
      break;
  }
  interrupts();
}
#endif

#if defined(__STM32F1__)
void ZTimer::setupTimer(IsrTimer timer, unsigned int prescaler) {
    setupTimer(timer, 1, prescaler, 1);
}

void ZTimer::setupTimer(IsrTimer timer, int channel, unsigned int prescaler, unsigned int compare) {
  _timer = timer;

  stopTimer();
  noInterrupts();
  switch(_timer) {

    case ZTIMER1:
      hwTimer1.setMode(channel, TIMER_OUTPUT_COMPARE);
      hwTimer1.setPrescaleFactor(prescaler);
      hwTimer1.setCompare(channel, compare);
      hwTimer1.attachInterrupt(channel, ISR1);
      break;
    case ZTIMER2:
      hwTimer2.setMode(channel, TIMER_OUTPUT_COMPARE);
      hwTimer2.setPrescaleFactor(prescaler);
      hwTimer2.setCompare(channel, compare);
      hwTimer2.attachInterrupt(channel, ISR2);
      break;
    case ZTIMER3:
      hwTimer3.setMode(channel, TIMER_OUTPUT_COMPARE);
      hwTimer3.setPrescaleFactor(prescaler);
      hwTimer3.setCompare(channel, compare);
      hwTimer3.attachInterrupt(channel, ISR3);
      break;
    case ZTIMER4:
      hwTimer4.setMode(channel, TIMER_OUTPUT_COMPARE);
      hwTimer4.setPrescaleFactor(prescaler);
      hwTimer4.setCompare(channel, compare);
      hwTimer4.attachInterrupt(channel, ISR4);
      break;
    case ZTIMER5:
      hwTimer5.setMode(channel, TIMER_OUTPUT_COMPARE);
      hwTimer5.setPrescaleFactor(prescaler);
      hwTimer5.setCompare(channel, compare);
      hwTimer5.attachInterrupt(channel, ISR5);
      break;
    case ZTIMER6:
      hwTimer6.setMode(channel, TIMER_OUTPUT_COMPARE);
      hwTimer6.setPrescaleFactor(prescaler);
      hwTimer6.setCompare(channel, compare);
      hwTimer6.attachInterrupt(channel, ISR6);
      break;
    case ZTIMER7:
      hwTimer7.setMode(channel, TIMER_OUTPUT_COMPARE);
      hwTimer7.setPrescaleFactor(prescaler);
      hwTimer7.setCompare(channel, compare);
      hwTimer7.attachInterrupt(channel, ISR7);
      break;
    case ZTIMER8:
      hwTimer8.setMode(channel, TIMER_OUTPUT_COMPARE);
      hwTimer8.setPrescaleFactor(prescaler);
      hwTimer8.setCompare(channel, compare);
      hwTimer8.attachInterrupt(channel, ISR8);
      break;
  }
  interrupts();
}
#endif

void ZTimer::setupTimerHook(void (*function)(void))
{
  switch(_timer) {
    case ZTIMER1: __timer1Hook = function; break;
    case ZTIMER2: __timer2Hook = function; break;
    case ZTIMER3: __timer3Hook = function; break;
    case ZTIMER4: __timer4Hook = function; break;
    case ZTIMER5: __timer5Hook = function; break;
    case ZTIMER6: __timer6Hook = function; break;
    case ZTIMER7: __timer7Hook = function; break;
    case ZTIMER8: __timer8Hook = function; break;
  }
}

void ZTimer::setNextInterruptInterval(unsigned int interval) {
  stopTimer();
  setOverflow(interval);
  setCounter(0);
  startTimer();
}

unsigned int ZTimer::getOverflow() {
  switch(_timer) {
#if defined(__AVR__)
    case ZTIMER1: return OCR1A;
    case ZTIMER2: return 0;
    case ZTIMER3: return OCR3A;
    case ZTIMER4: return OCR4A;
    case ZTIMER5: return OCR5A;
    case ZTIMER6:
    case ZTIMER7:
    case ZTIMER8: return 0;
#endif
#if defined(__STM32F1__)
    case ZTIMER1: return hwTimer1.getOverflow(); 
    case ZTIMER2: return hwTimer2.getOverflow();
    case ZTIMER3: return hwTimer3.getOverflow();
    case ZTIMER4: return hwTimer4.getOverflow();
    case ZTIMER5: return hwTimer5.getOverflow();
    case ZTIMER6: return hwTimer6.getOverflow();
    case ZTIMER7: return hwTimer7.getOverflow();
    case ZTIMER8: return hwTimer8.getOverflow();
#endif
  }
  return 0;
}

void ZTimer::setOverflow(unsigned int value) {
  switch(_timer) {
#if defined(__AVR__)
    case ZTIMER1: OCR1A = value; break;
    case ZTIMER2: break;
    case ZTIMER3: OCR3A = value; break;
    case ZTIMER4: OCR4A = value; break;
    case ZTIMER5: OCR5A = value; break;
    case ZTIMER6:
    case ZTIMER7:
    case ZTIMER8: break;
#endif
#if defined(__STM32F1__)
    case ZTIMER1: hwTimer1.setOverflow(value); break;
    case ZTIMER2: hwTimer2.setOverflow(value); break;
    case ZTIMER3: hwTimer3.setOverflow(value); break;
    case ZTIMER4: hwTimer4.setOverflow(value); break;
    case ZTIMER5: hwTimer5.setOverflow(value); break;
    case ZTIMER6: hwTimer6.setOverflow(value); break;
    case ZTIMER7: hwTimer7.setOverflow(value); break;
    case ZTIMER8: hwTimer8.setOverflow(value); break;
#endif
  }
}

void ZTimer::setCounter(unsigned int value) {
  switch(_timer) {
#if defined(__AVR__)
    case ZTIMER1: TCNT1 = value; break;
    case ZTIMER2: break;
    case ZTIMER3: TCNT3 = value; break;
    case ZTIMER4: TCNT4 = value; break;
    case ZTIMER5: TCNT5 = value; break;
    case ZTIMER6:
    case ZTIMER7:
    case ZTIMER8: break;
#endif
#if defined(__STM32F1__)
    case ZTIMER1: hwTimer1.setCount(value); break;
    case ZTIMER2: hwTimer2.setCount(value); break;
    case ZTIMER3: hwTimer3.setCount(value); break;
    case ZTIMER4: hwTimer4.setCount(value); break;
    case ZTIMER5: hwTimer5.setCount(value); break;
    case ZTIMER6: hwTimer6.setCount(value); break;
    case ZTIMER7: hwTimer7.setCount(value); break;
    case ZTIMER8: hwTimer8.setCount(value); break;
#endif
  }
}

void ZTimer::startTimer() {
  switch(_timer) {
#if defined(__AVR__)
    case ZTIMER1: TIMSK1 |= _BV(OCIE1A); break;
    case ZTIMER2: break;
    case ZTIMER3: TIMSK3 |= _BV(OCIE3A); break;
    case ZTIMER4: TIMSK4 |= _BV(OCIE4A); break;
    case ZTIMER5: TIMSK5 |= _BV(OCIE5A); break;
    case ZTIMER6:
    case ZTIMER7:
    case ZTIMER8: break;
#endif
#if defined(__STM32F1__)
    case ZTIMER1: hwTimer1.refresh(); hwTimer1.resume(); break;
    case ZTIMER2: hwTimer2.refresh(); hwTimer2.resume(); break;
    case ZTIMER3: hwTimer3.refresh(); hwTimer3.resume(); break;
    case ZTIMER4: hwTimer4.refresh(); hwTimer4.resume(); break;
    case ZTIMER5: hwTimer5.refresh(); hwTimer5.resume(); break;
    case ZTIMER6: hwTimer6.refresh(); hwTimer6.resume(); break;
    case ZTIMER7: hwTimer7.refresh(); hwTimer7.resume(); break;
    case ZTIMER8: hwTimer8.refresh(); hwTimer8.resume(); break;
#endif
  }
}

void ZTimer::stopTimer() {
  switch(_timer) {
#if defined(__AVR__)
    case ZTIMER1: TIMSK1 &= ~_BV(OCIE1A); break;
    case ZTIMER2: break;
    case ZTIMER3: TIMSK3 &= ~_BV(OCIE3A); break;
    case ZTIMER4: TIMSK4 &= ~_BV(OCIE4A); break;
    case ZTIMER5: TIMSK5 &= ~_BV(OCIE5A); break;
    case ZTIMER6:
    case ZTIMER7:
    case ZTIMER8: break;
#endif
#if defined(__STM32F1__)
    case ZTIMER1: hwTimer1.pause(); break;
    case ZTIMER2: hwTimer2.pause(); break;
    case ZTIMER3: hwTimer3.pause(); break;
    case ZTIMER4: hwTimer4.pause(); break;
    case ZTIMER5: hwTimer5.pause(); break;
    case ZTIMER6: hwTimer6.pause(); break;
    case ZTIMER7: hwTimer7.pause(); break;
    case ZTIMER8: hwTimer8.pause(); break;
#endif
  }
}
