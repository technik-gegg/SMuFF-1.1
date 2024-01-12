/**
 * SMuFF Firmware
 * Copyright (C) 2019-2024 Technik Gegg
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

#include <Arduino.h>

typedef uint64_t timerVal_t;

class ZTimer {
  public:
    typedef enum {
      UNDEFINED = -1,
      _TIMER1 = 0,
      _TIMER2 = 1,
      _TIMER3 = 2,
      _TIMER4 = 3,
      MAX_TIMERS
    } timerNum_t;

    ZTimer() { _timer = UNDEFINED; };

    void setupTimer(timerNum_t timer, uint16_t prescaler, timerVal_t compare=1);
    void setupHook(void (*function)(void));
    void setNextInterruptInterval(timerVal_t interval);
    timerVal_t getOverflow();
    void setOverflow(timerVal_t value);
    void start();
    void stop();

  private:
    timerNum_t _timer;
};
