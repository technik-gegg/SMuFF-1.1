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
#pragma once

#include <stdint.h>

typedef uint32_t timerVal_t;

#if !defined(STM32_CORE_VERSION)
typedef uint8_t     pin_t;
#else
typedef uint32_t    pin_t;
#endif

class ZTimer {
  public:
    typedef enum {
      UNDEFINED = -1,
      _TIMER1 = 0,
      _TIMER2 = 1,
      _TIMER3 = 2,
      _TIMER4 = 3,
      _TIMER6 = 4,
      _TIMER7 = 5,
      _TIMER14 = 6,
      _TIMER15 = 7,
      _TIMER16 = 8,
      _TIMER17 = 9,
      MAX_TIMERS
    } timerNum_t;

    typedef enum {
      CH1 = 1,
      CH2 = 2,
      CH3 = 3,
      CH4 = 4
    } timerChannel_t;

    ZTimer() { _timer = UNDEFINED; };

    void setupTimer(timerNum_t timer, timerChannel_t channel, uint32_t prescaler, timerVal_t compare = 1, void (* serviceFunPtr)() = nullptr);
    void setNextInterruptInterval(timerVal_t interval, bool onChannel = false);
    void setupOVHook(void (*function)(void));
    timerVal_t getCompare();
    timerVal_t getOverflow();
    void setCompare(timerVal_t value);
    void setOverflow(timerVal_t value);
    void start();
    void stop();
    void startChannel();
    void stopChannel();
    timerVal_t getClockFrequency();
    timerVal_t getPrescaler();
    void setPriority(uint32_t preempt, uint32_t sub);
    void setupPWM(pin_t pin, uint32_t frequency, uint32_t dutycycle) {
      _pin        = pin;
      _frequency  = frequency;
      _dutycycle  = dutycycle;
    };
    void setPreload(bool preload);
    bool isValid();

  private:
    timerNum_t      _timer;
    timerChannel_t  _channel;
    pin_t           _pin = 0;
    uint32_t        _frequency = 0;
    uint32_t        _dutycycle = 0;
};
