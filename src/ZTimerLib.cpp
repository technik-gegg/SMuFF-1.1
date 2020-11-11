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
#if defined(__STM32F1__)
//#include <libmaple/libmaple.h>
#elif defined(__ESP32__)
#include "esp32-hal.h"
#endif

static void (*__timer1Hook)(void) = nullptr;
static void (*__timer2Hook)(void) = nullptr;
static void (*__timer3Hook)(void) = nullptr;
static void (*__timer4Hook)(void) = nullptr;
static void (*__timer5Hook)(void) = nullptr;
static void (*__timer6Hook)(void) = nullptr;
static void (*__timer7Hook)(void) = nullptr;
static void (*__timer8Hook)(void) = nullptr;

#if defined(__AVR__)
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

#elif defined(__STM32F1__)
HardwareTimer hwTimer1(1);
HardwareTimer hwTimer2(2);
HardwareTimer hwTimer3(3);
HardwareTimer hwTimer4(4);
HardwareTimer hwTimer5(5);
HardwareTimer hwTimer6(6);
HardwareTimer hwTimer7(7);
HardwareTimer hwTimer8(8);

void ISR1() {
  if(__timer1Hook != nullptr)
    __timer1Hook();
}

void ISR2() {
  if(__timer2Hook != nullptr)
    __timer2Hook();
}

void ISR3() {
  if(__timer3Hook != nullptr)
    __timer3Hook();
}

void ISR4() {
  if(__timer4Hook != nullptr)
    __timer4Hook();

}

void ISR5() {
  if(__timer5Hook != nullptr)
    __timer5Hook();
}

void ISR6() {
  if(__timer6Hook != nullptr)
    __timer6Hook();
}

void ISR7() {
  if(__timer7Hook != nullptr)
    __timer7Hook();
}

void ISR8() {
  if(__timer8Hook != nullptr)
    __timer8Hook();
}

#elif defined(__ESP32__)
hw_timer_t* hwTimer1 = nullptr;
hw_timer_t* hwTimer2 = nullptr;
hw_timer_t* hwTimer3 = nullptr;
hw_timer_t* hwTimer4 = nullptr;
portMUX_TYPE timer1Mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timer2Mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timer3Mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timer4Mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR ISR1() {
  if(__timer1Hook != nullptr) {
    portENTER_CRITICAL_ISR(&timer1Mux);
    __timer1Hook();
    portEXIT_CRITICAL_ISR(&timer1Mux);
  }
}

void IRAM_ATTR ISR2() {
  if(__timer2Hook != nullptr) {
    portENTER_CRITICAL_ISR(&timer2Mux);
    __timer2Hook();
    portEXIT_CRITICAL_ISR(&timer2Mux);
  }
}

void IRAM_ATTR ISR3() {
  if(__timer3Hook != nullptr) {
    portENTER_CRITICAL_ISR(&timer3Mux);
    __timer3Hook();
    portEXIT_CRITICAL_ISR(&timer3Mux);
  }
}

void IRAM_ATTR ISR4() {
  if(__timer4Hook != nullptr) {
    portENTER_CRITICAL_ISR(&timer4Mux);
    __timer4Hook();
    portEXIT_CRITICAL_ISR(&timer4Mux);
  }
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

#elif defined(__STM32F1__)
void ZTimer::setupTimer(IsrTimer timer, uint16_t prescaler) {
    setupTimer(timer, 1, prescaler, 1);
}

void ZTimer::setupTimer(IsrTimer timer, uint8_t channel, uint16_t prescaler, uint16_t compare) {
  _timer = timer;
  _channel = channel;

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
      /*
      hwTimer6.setMode(channel, TIMER_OUTPUT_COMPARE);
      hwTimer6.setPrescaleFactor(prescaler);
      hwTimer6.setCompare(channel, compare);
      */
      // since this timer doesn't have a compare mode, we're using
      // the compare value as a period (in uS)
      hwTimer6.setPeriod(compare);
      hwTimer6.attachInterrupt(channel, ISR6);
      break;
    case ZTIMER7:
      /*
      hwTimer7.setMode(channel, TIMER_OUTPUT_COMPARE);
      hwTimer7.setPrescaleFactor(prescaler);
      hwTimer7.setCompare(channel, compare);
      */
      // since this timer doesn't have a compare mode, we're using
      // the compare value as a period (in uS)
      hwTimer7.setPeriod(compare);
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

#elif defined (__ESP32__)
void ZTimer::setupTimer(IsrTimer timer, uint16_t prescaler) {
    setupTimer(timer, prescaler, 1);
}

void ZTimer::setupTimer(IsrTimer timer, uint16_t prescaler, uint64_t compare) {
  _timer = timer;

  noInterrupts();
  switch(_timer) {
    case ZTIMER1:
      hwTimer1 = timerBegin(0, prescaler, true);
      timerAttachInterrupt(hwTimer1, &ISR1, true);
      timerAlarmWrite(hwTimer1, compare, true);
      break;
    case ZTIMER2:
      hwTimer2 = timerBegin(1, prescaler, true);
      timerAttachInterrupt(hwTimer2, &ISR2, true);
      timerAlarmWrite(hwTimer2, compare, true);
      break;
    case ZTIMER3:
      hwTimer3 = timerBegin(2, prescaler, true);
      timerAttachInterrupt(hwTimer3, &ISR3, true);
      timerAlarmWrite(hwTimer3, compare, true);
      break;
    case ZTIMER4:
      hwTimer4 = timerBegin(3, prescaler, true);
      timerAttachInterrupt(hwTimer4, &ISR4, true);
      timerAlarmWrite(hwTimer4, compare, true);
      break;
    default:
      break;
  }
  interrupts();
}
#endif

void ZTimer::setupTimerHook(void (*function)(void)) {
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

void ZTimer::setNextInterruptInterval(timerVal_t interval) {
  stopTimer();
  setOverflow(interval);
  setCounter(0);
  startTimer();
}

timerVal_t ZTimer::getOverflow() {
  switch(_timer) {
    default: return 0;

#if defined(__AVR__)
    case ZTIMER1: return OCR1A;
    case ZTIMER2: return 0;
    case ZTIMER3: return OCR3A;
    case ZTIMER4: return OCR4A;
    case ZTIMER5: return OCR5A;

#elif defined(__STM32F1__)
    case ZTIMER1: return hwTimer1.getOverflow();
    case ZTIMER2: return hwTimer2.getOverflow();
    case ZTIMER3: return hwTimer3.getOverflow();
    case ZTIMER4: return hwTimer4.getOverflow();
    case ZTIMER5: return hwTimer5.getOverflow();
    case ZTIMER6: return hwTimer6.getOverflow();
    case ZTIMER7: return hwTimer7.getOverflow();
    case ZTIMER8: return hwTimer8.getOverflow();

#elif defined(__ESP32__)
    case ZTIMER1: return timerAlarmReadMicros(hwTimer1);
    case ZTIMER2: return timerAlarmReadMicros(hwTimer2);
    case ZTIMER3: return timerAlarmReadMicros(hwTimer3);
    case ZTIMER4: return timerAlarmReadMicros(hwTimer4);
#endif
  }
}

void ZTimer::setOverflow(timerVal_t value) {
  switch(_timer) {
    default: break;

#if defined(__AVR__)
    case ZTIMER1: OCR1A = value; break;
    case ZTIMER2: break;
    case ZTIMER3: OCR3A = value; break;
    case ZTIMER4: OCR4A = value; break;
    case ZTIMER5: OCR5A = value; break;

#elif defined(__STM32F1__)
    case ZTIMER1: hwTimer1.setOverflow(value); break;
    case ZTIMER2: hwTimer2.setOverflow(value); break;
    case ZTIMER3: hwTimer3.setOverflow(value); break;
    case ZTIMER4: hwTimer4.setOverflow(value); break;
    case ZTIMER5: hwTimer5.setOverflow(value); break;
    case ZTIMER6: hwTimer6.setOverflow(value); break;
    case ZTIMER7: hwTimer7.setOverflow(value); break;
    case ZTIMER8: hwTimer8.setOverflow(value); break;

#elif defined(__ESP32__)
    case ZTIMER1: timerAlarmWrite(hwTimer1, value, true); break;
    case ZTIMER2: timerAlarmWrite(hwTimer2, value, true); break;
    case ZTIMER3: timerAlarmWrite(hwTimer3, value, true); break;
    case ZTIMER4: timerAlarmWrite(hwTimer4, value, true); break;
#endif
  }
}

void ZTimer::setCompare(timerVal_t value) {
  switch(_timer) {
    default: break;

#if defined(__AVR__)
    // Not implemented

#elif defined(__STM32F1__)
    case ZTIMER1: hwTimer1.setCompare(_channel, value); break;
    case ZTIMER2: hwTimer2.setCompare(_channel, value); break;
    case ZTIMER3: hwTimer3.setCompare(_channel, value); break;
    case ZTIMER4: hwTimer4.setCompare(_channel, value); break;
    case ZTIMER5: hwTimer5.setCompare(_channel, value); break;
    case ZTIMER6: hwTimer6.setCompare(_channel, value); break;
    case ZTIMER7: hwTimer7.setCompare(_channel, value); break;
    case ZTIMER8: hwTimer8.setCompare(_channel, value); break;

#elif defined(__ESP32__)
    // Not implemented
#endif
  }
}

void ZTimer::setCounter(timerVal_t value) {
  switch(_timer) {
    default: break;

#if defined(__AVR__)
    case ZTIMER1: TCNT1 = value; break;
    case ZTIMER2: break;
    case ZTIMER3: TCNT3 = value; break;
    case ZTIMER4: TCNT4 = value; break;
    case ZTIMER5: TCNT5 = value; break;

#elif defined(__STM32F1__)
    case ZTIMER1: hwTimer1.setCount(value); break;
    case ZTIMER2: hwTimer2.setCount(value); break;
    case ZTIMER3: hwTimer3.setCount(value); break;
    case ZTIMER4: hwTimer4.setCount(value); break;
    case ZTIMER5: hwTimer5.setCount(value); break;
    case ZTIMER6: hwTimer6.setCount(value); break;
    case ZTIMER7: hwTimer7.setCount(value); break;
    case ZTIMER8: hwTimer8.setCount(value); break;

#elif defined(__ESP32__)
    case ZTIMER1: break;
    case ZTIMER2: break;
    case ZTIMER3: break;
    case ZTIMER4: break;
#endif
  }
}

void ZTimer::startTimer() {
  switch(_timer) {
    default: break;

#if defined(__AVR__)
    case ZTIMER1: TIMSK1 |= _BV(OCIE1A); break;
    case ZTIMER2: break;
    case ZTIMER3: TIMSK3 |= _BV(OCIE3A); break;
    case ZTIMER4: TIMSK4 |= _BV(OCIE4A); break;
    case ZTIMER5: TIMSK5 |= _BV(OCIE5A); break;

#elif defined(__STM32F1__)
    case ZTIMER1: hwTimer1.refresh(); hwTimer1.resume(); break;
    case ZTIMER2: hwTimer2.refresh(); hwTimer2.resume(); break;
    case ZTIMER3: hwTimer3.refresh(); hwTimer3.resume(); break;
    case ZTIMER4: hwTimer4.refresh(); hwTimer4.resume(); break;
    case ZTIMER5: hwTimer5.refresh(); hwTimer5.resume(); break;
    case ZTIMER6: hwTimer6.refresh(); hwTimer6.resume(); break;
    case ZTIMER7: hwTimer7.refresh(); hwTimer7.resume(); break;
    case ZTIMER8: hwTimer8.refresh(); hwTimer8.resume(); break;

#elif defined(__ESP32__)
    case ZTIMER1: timerAlarmEnable(hwTimer1); break;
    case ZTIMER2: timerAlarmEnable(hwTimer2); break;
    case ZTIMER3: timerAlarmEnable(hwTimer3); break;
    case ZTIMER4: timerAlarmEnable(hwTimer4); break;
#endif
  }
}

void ZTimer::stopTimer() {
  switch(_timer) {
    default: break;

#if defined(__AVR__)
    case ZTIMER1: TIMSK1 &= ~_BV(OCIE1A); break;
    case ZTIMER2: break;
    case ZTIMER3: TIMSK3 &= ~_BV(OCIE3A); break;
    case ZTIMER4: TIMSK4 &= ~_BV(OCIE4A); break;
    case ZTIMER5: TIMSK5 &= ~_BV(OCIE5A); break;

#elif defined(__STM32F1__)
    case ZTIMER1: hwTimer1.pause(); break;
    case ZTIMER2: hwTimer2.pause(); break;
    case ZTIMER3: hwTimer3.pause(); break;
    case ZTIMER4: hwTimer4.pause(); break;
    case ZTIMER5: hwTimer5.pause(); break;
    case ZTIMER6: hwTimer6.pause(); break;
    case ZTIMER7: hwTimer7.pause(); break;
    case ZTIMER8: hwTimer8.pause(); break;

#elif defined(__ESP32__)
    case ZTIMER1: timerAlarmDisable(hwTimer1); break;
    case ZTIMER2: timerAlarmDisable(hwTimer2); break;
    case ZTIMER3: timerAlarmDisable(hwTimer3); break;
    case ZTIMER4: timerAlarmDisable(hwTimer4); break;
#endif
  }
}
