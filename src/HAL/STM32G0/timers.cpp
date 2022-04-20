/**
 * SMuFF Firmware
 * Copyright (C) 2019-2022 Technik Gegg
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

#if defined(__STM32G0XX)

/*
 * STM32G0 HAL timers handling
 */
#include <Arduino.h>
#include "timers.h"

static struct {
  HardwareTimer*  timer;
  TIM_TypeDef*    instance;
  IRQn_Type       irqN;
  void            (*serviceOVFunPtr)(void);
  } timers[] = {
    { .timer = nullptr,
      .instance = TIM1,
      .irqN = TIM1_IRQn,
      .serviceOVFunPtr = nullptr },
    { .timer = nullptr,
      .instance = TIM2,
      .irqN = TIM2_IRQn,
      .serviceOVFunPtr = nullptr },
    { .timer = nullptr,
      .instance = TIM3,
      .irqN = TIM3_IRQn,
      .serviceOVFunPtr = nullptr },
    { .timer = nullptr,
      .instance = TIM4,
      .irqN = TIM4_IRQn,
      .serviceOVFunPtr = nullptr },
    { .timer = nullptr,
      .instance = TIM6,
      .irqN = TIM6_IRQn,
      .serviceOVFunPtr = nullptr },
    { .timer = nullptr,
      .instance = TIM7,
      .irqN = TIM7_IRQn,
      .serviceOVFunPtr = nullptr },
    { .timer = nullptr,
      .instance = TIM14,
      .irqN = TIM14_IRQn,
      .serviceOVFunPtr = nullptr },
    { .timer = nullptr,
      .instance = TIM15,
      .irqN = TIM15_IRQn,
      .serviceOVFunPtr = nullptr },
    { .timer = nullptr,
      .instance = TIM16,
      .irqN = TIM16_IRQn,
      .serviceOVFunPtr = nullptr },
    { .timer = nullptr,
      .instance = TIM17,
      .irqN = TIM17_IRQn,
      .serviceOVFunPtr = nullptr }
};

bool ZTimer::isValid() {
   return _timer != UNDEFINED && timers[_timer].timer != nullptr;
}

void ZTimer::setupTimer(timerNum_t timer, timerChannel_t channel, uint32_t prescaler, timerVal_t compare, void(*serviceFunPtr)()) {
  if (timer < _TIMER1 || timer >= MAX_TIMERS)
    return;

  _timer = timer;
  _channel = channel;

  timers[_timer].timer = new HardwareTimer(timers[_timer].instance);
  HardwareTimer* hwTimer = timers[_timer].timer;
  hwTimer->pause();
  hwTimer->setMode(_channel, TIMER_DISABLED);
  hwTimer->setPrescaleFactor(prescaler);
  hwTimer->setOverflow(compare);
  hwTimer->setPreloadEnable(false);
  if(serviceFunPtr != nullptr)
    hwTimer->attachInterrupt(serviceFunPtr);
}

void ZTimer::setupOVHook(void (*function)(void)) {
  if (isValid())
    timers[_timer].serviceOVFunPtr = function;
}

void ZTimer::setNextInterruptInterval(timerVal_t interval, bool onChannel) {
  if (!isValid())
    return;

  if(onChannel)
    stopChannel();
  else
    stop();
  if(interval != 0) {
    setOverflow(interval);
    start();
  }
}

timerVal_t ZTimer::getCompare() {
  return (isValid()) ? timers[_timer].timer->getCaptureCompare(_channel) : 0;
}

void ZTimer::setCompare(timerVal_t value) {
  if (isValid())
    timers[_timer].timer->setCaptureCompare(_channel, value);
}

timerVal_t ZTimer::getOverflow() {
  return (isValid()) ? timers[_timer].timer->getOverflow() : 0;
}

void ZTimer::setOverflow(timerVal_t value) {
  if (isValid()) {
    timers[_timer].timer->setOverflow(value);
    timers[_timer].timer->setCount(0);
  }
}

void ZTimer::start() {
  if (isValid()) {
    timers[_timer].timer->refresh();
    timers[_timer].timer->resume();
  }
}

void ZTimer::startChannel() {
  if (isValid()) {
    timers[_timer].timer->refresh();
    timers[_timer].timer->resumeChannel(_channel);
  }
}

void ZTimer::stop() {
  if (isValid())
    timers[_timer].timer->pause();
}

void ZTimer::stopChannel() {
  if (isValid())
    timers[_timer].timer->pauseChannel(_channel);
}

uint32_t ZTimer::getClockFrequency() {
  return (isValid()) ? timers[_timer].timer->getTimerClkFreq() : 0;
}

uint32_t ZTimer::getPrescaler() {
  return (isValid()) ? timers[_timer].timer->getPrescaleFactor() : 0;
}

void ZTimer::setPriority(uint32_t preempt, uint32_t sub) {
  if (isValid())
    HAL_NVIC_SetPriority(timers[_timer].irqN, preempt, sub);
}

void ZTimer::setPreload(bool preload) {
  if (isValid())
    timers[_timer].timer->setPreloadEnable(preload);
}
#endif
