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
#pragma once

#include <stdint.h>

typedef uint16_t timerVal_t;

class Timer {
  public:
    typedef enum {
      UNDEFINED = -1,
      TIMER1 = 0,
      TIMER2 = 1,
      TIMER3 = 2,
      TIMER4 = 3,
#ifdef STM32_HIGH_DENSITY
      TIMER5 = 4,
      TIMER6 = 5,
      TIMER7 = 6,
      TIMER8 = 7,
#endif
      MAX_TIMERS
    } timerNum_t;

    typedef enum {
      CH1 = 1,
      CH2 = 2,
      CH3 = 3,
      CH4 = 4
    } timerChannel_t;

    Timer() { _timer = UNDEFINED; };

    void setupTimer(timerNum_t timer, timerChannel_t channel, uint32_t prescaler, timerVal_t compare = 1);
    void setNextInterruptInterval(timerVal_t interval);
    void setupHook(void (*function)(void));
    timerVal_t getOverflow();
    void setOverflow(timerVal_t value);
    void start();
    void stop();

  private:
    timerNum_t _timer;
    timerChannel_t _channel;
};
