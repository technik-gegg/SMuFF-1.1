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

#include <stdlib.h>
#include <Arduino.h>
#include "Config.h"

#ifndef _ZTIMER_H
#define _ZTIMER_H

extern void __debug(const char* fmt, ...);

class ZTimer {
public:
    typedef enum {
      TIMER1 = 1,
      TIMER3 = 3,
      TIMER4 = 4,
      TIMER5 = 5
    } IsrTimer;

    typedef enum {
      PRESCALER1      = 1,
      PRESCALER8      = 2,
      PRESCALER64     = 3,
      PRESCALER256    = 4,
      PRESCALER1024   = 5
    } TimerPrescaler;

    ZTimer() { };
    
    void           setupTimer(IsrTimer timer, TimerPrescaler prescaler);
    void           setupTimerHook(void (*function)(void));
    void           setNextInterruptInterval(unsigned int interval);
    unsigned int   getOCRxA();
    void           setOCRxA(unsigned int value);
    void           setTCNTx(unsigned int value);
    void           startTimer();
    void           stopTimer();

private:
    IsrTimer      _timer;
};

#endif
