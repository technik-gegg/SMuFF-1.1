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
#pragma once

#include <stdlib.h>
#include <Arduino.h>
#include "CommonMacros.h"
#include "Config.h"

extern void __debugS(const char* fmt, ...);

class ZTimer {
public:
    typedef enum {
      ZTIMER1 = 1,
      ZTIMER2 = 2,
      ZTIMER3 = 3,
      ZTIMER4 = 4,
      ZTIMER5 = 5,
      ZTIMER6 = 6,
      ZTIMER7 = 7,
      ZTIMER8 = 8
    } IsrTimer;

    typedef enum {
      CH1 = 1,
      CH2 = 2,
      CH3 = 3,
      CH4 = 4
    } IsrTimerChannel;

    typedef enum {
      PRESCALER1      = 1,
      PRESCALER8      = 2,
      PRESCALER64     = 3,
      PRESCALER256    = 4,
      PRESCALER1024   = 5
    } TimerPrescaler;

    ZTimer() { };

#if defined(__AVR__)
    void           setupTimer(IsrTimer timer, TimerPrescaler prescaler);
#elif defined(__STM32F1__)
    void           setupTimer(IsrTimer timer, uint16_t prescaler);
    void           setupTimer(IsrTimer timer, uint8_t channel, uint16_t prescaler, timerVal_t compare = 1);
#elif defined(__ESP32__)
    void           setupTimer(IsrTimer timer, uint16_t prescaler);
    void           setupTimer(IsrTimer timer, uint16_t prescaler, timerVal_t compare);
#endif
    timerVal_t     getOverflow();
    void           setOverflow(timerVal_t value);
    void           setNextInterruptInterval(timerVal_t interval);
    void           setupTimerHook(void (*function)(void));
    void           setCompare(timerVal_t value);
    void           setCounter(timerVal_t value);
    void           startTimer();
    void           stopTimer();

private:
    IsrTimer      _timer;
    uint8_t       _channel;
};
