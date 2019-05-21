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

#include "ZServo.h"

ZServo* instance; 

void ZServo::write(int degree) {
  setServoPos(degree);
}

bool ZServo::setServoPos(int degree) {
  bool stat = false;
  if(_pin > 0) {
    _pulseLen = map(degree, 0, 180, US_PER_PULSE_0DEG, US_PER_PULSE_180DEG);
    instance = this;
    //__debug("Pulse length set to: %dms", _pulseLen);
    servoTimer.setNextInterruptInterval(TIMER_INTERVAL);
    stat = true;
    _degree = degree;
  }
  return stat;
}

void ZServo::setServoMS(int microseconds) {
  _pulseLen = microseconds;
  servoTimer.setNextInterruptInterval(TIMER_INTERVAL);
}

void ZServo::setServo() {
  digitalWrite(_pin, HIGH);
  delayMicroseconds(_pulseLen);
  digitalWrite(_pin, LOW);
}

void isrServoTimerHandler() {
  if(instance != NULL)
    instance->setServo();
}
