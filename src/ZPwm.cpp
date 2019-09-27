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

#include "ZPwm.h"

ZPwm* pwmInstance; 

bool ZPwm::writePwm(int speed) {
  bool stat = false;
  if(_pin > 0) {
    pwmTimer.stopTimer();
    _speed = speed;
    _pulseLen = map(speed, 0, 100, 0, 5000);
    pwmInstance = this;
    __debug(PSTR("Pulse length set to: %dus"), _pulseLen);
    pwmTimer.setNextInterruptInterval(PWM_INTERVAL);
    stat = true;
  }
  return stat;
}

void isrPwmTimerHandler() {
  if(pwmInstance != NULL)
    pwmInstance->setPwm();
}
