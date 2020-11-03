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

#include "ZFan.h"

static ZFan* fanInstances[MAX_FANS];

void ZFan::attach(int8_t pin) {
  _pin = pin;
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, 0);
}

void ZFan::detach() {
  digitalWrite(_pin, 0);
  fanInstances[_fanIndex] = nullptr;
  _pin = 0;
}

void ZFan::setIndex(int8_t fanIndex) {
  if(fanIndex != -1 && fanIndex < MAX_FANS) {
    _fanIndex = fanIndex;
    fanInstances[_fanIndex] = this;
  }
}

void ZFan::setFanSpeed(uint8_t speed) {
  _speed = speed;
  _pulseLen = map(speed, 0, 100, _minSpeed, _maxSpeed);
}

void ZFan::setFan() {
  _tickCnt += 50;
  if(_tickCnt <= (uint32_t)_pulseLen)
    setFanPin(HIGH);
  else
    setFanPin(LOW);
  if(_tickCnt >= (uint32_t)(FAN_DUTY_CYCLE)) {
    _tickCnt = 0;
  }
}

void ZFan::setFanPin(int8_t state) {
  digitalWrite(_pin, state);
}

void isrFanTimerHandler() {
  // call all handlers for all fans periodically if the
  // internal timer is being used.
  for(uint8_t i=0;  i< MAX_FANS; i++) {
    if(fanInstances[i] != nullptr) {
      fanInstances[i]->setFan();
    }
  }
}
