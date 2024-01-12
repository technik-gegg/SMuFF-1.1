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

#include "ZFan.h"

static ZFan* fanInstances[MAX_FANS];

void ZFan::attach(pin_t pin) {
  _pin = pin;
  if(_pin != 0) {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, 0);
  }
}

void ZFan::detach() {
  if(_pin != 0)
    digitalWrite(_pin, 0);
  fanInstances[_fanIndex] = nullptr;
  _pin = 0;
}

void ZFan::setIndex(int8_t fanIndex) {
  if(fanIndex >= 0 && fanIndex < MAX_FANS) {
    _fanIndex = fanIndex;
    fanInstances[_fanIndex] = this;
  }
}

void ZFan::setFanSpeed(uint8_t speed) {
  _speed = speed;
  _pulseLen = map(speed, 0, 100, _minSpeed, _maxSpeed);
  _blipTime = 0;
}

void ZFan::setFan() {
  _tickCnt += _tickRes;
  _blipTime += _tickRes;
  if(_tickCnt <= _pulseLen)
    setFanPin(HIGH);
  else {
    if(_blipTimeout > 0 && _blipTime > _blipTimeout)
      setFanPin(LOW);
  }
  if(_tickCnt >= _maxSpeed) {
    _tickCnt = 0;
  }
}

void ZFan::setFanPin(uint8_t state) {
  if(_pin != 0)
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
