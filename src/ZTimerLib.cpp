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
  }
  //setNextInterruptInterval(1000);
  interrupts();
}
#endif

#if defined(__STM32F1__)
void ZTimer::setupTimer(IsrTimer timer, unsigned int prescaler) {
  _timer = timer;

  stopTimer();
  noInterrupts();
  switch(_timer) {

    case ZTIMER1:
      hwTimer1.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
      hwTimer1.setPrescaleFactor(prescaler);
      hwTimer1.setCompare(TIMER_CH1, 1);
      hwTimer1.attachCompare1Interrupt(ISR1);
      break;
    case ZTIMER2:
      hwTimer2.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
      hwTimer2.setPrescaleFactor(prescaler);
      hwTimer2.setCompare(TIMER_CH1, 1);
      hwTimer2.attachCompare1Interrupt(ISR2);
      break;
    case ZTIMER3:
      hwTimer3.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
      hwTimer3.setPrescaleFactor(prescaler);
      hwTimer3.setCompare(TIMER_CH1, 1);
      hwTimer3.attachCompare1Interrupt(ISR3);
      break;
    case ZTIMER4:
      hwTimer4.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
      hwTimer4.setPrescaleFactor(prescaler);
      hwTimer4.setCompare(TIMER_CH1, 1);
      hwTimer4.attachCompare1Interrupt(ISR4);
      break;
    case ZTIMER5:
      hwTimer5.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
      hwTimer5.setPrescaleFactor(prescaler);
      hwTimer5.setCompare(TIMER_CH1, 1);
      hwTimer5.attachCompare1Interrupt(ISR5);
      break;
  }
  //setNextInterruptInterval(1000);
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
#endif
#if defined(__STM32F1__)
    case ZTIMER1: return hwTimer1.getOverflow(); 
    case ZTIMER2: return hwTimer2.getOverflow();
    case ZTIMER3: return hwTimer3.getOverflow();
    case ZTIMER4: return hwTimer4.getOverflow();
    case ZTIMER5: return hwTimer5.getOverflow();
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
#endif
#if defined(__STM32F1__)
    case ZTIMER1: hwTimer1.setOverflow(value); break;
    case ZTIMER2: hwTimer2.setOverflow(value); break;
    case ZTIMER3: hwTimer3.setOverflow(value); break;
    case ZTIMER4: hwTimer4.setOverflow(value); break;
    case ZTIMER5: hwTimer5.setOverflow(value); break;
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
#endif
#if defined(__STM32F1__)
    case ZTIMER1: hwTimer1.setCount(value); break;
    case ZTIMER2: hwTimer2.setCount(value); break;
    case ZTIMER3: hwTimer3.setCount(value); break;
    case ZTIMER4: hwTimer4.setCount(value); break;
    case ZTIMER5: hwTimer5.setCount(value); break;
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
#endif
#if defined(__STM32F1__)
    case ZTIMER1: hwTimer1.refresh(); hwTimer1.resume(); break;
    case ZTIMER2: hwTimer2.refresh(); hwTimer2.resume(); break;
    case ZTIMER3: hwTimer3.refresh(); hwTimer3.resume(); break;
    case ZTIMER4: hwTimer4.refresh(); hwTimer4.resume(); break;
    case ZTIMER5: hwTimer5.refresh(); hwTimer5.resume(); break;
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
#endif
#if defined(__STM32F1__)
    case ZTIMER1: hwTimer1.pause(); break;
    case ZTIMER2: hwTimer2.pause(); break;
    case ZTIMER3: hwTimer3.pause(); break;
    case ZTIMER4: hwTimer4.pause(); break;
    case ZTIMER5: hwTimer5.pause(); break;
#endif
  }
}
