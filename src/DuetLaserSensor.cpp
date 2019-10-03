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

#include "DuetLaserSensor.h"

extern void __debug(const char* fmt, ...);


void DuetLaserSensor::attach(int pin) {
    _pin = pin;
    pinMode(_pin, INPUT);
    reset();
}

void DuetLaserSensor::reset() {
    _gotStartbit = false;
    _gotIdle = false;
    _bitCnt = 0;
    _dataCnt = 0;
    _data = 0;
    _lastBit = -1;
    _lastBits = String(_bits);
    _lastStuff = String(_stuff);
    _bits = "";
    _stuff = "";
}

void DuetLaserSensor::service() {
    if(_pin == -1)
        return;
    volatile int _b = digitalRead(_pin);
    if(!_gotIdle) {
        _state = 0x8000;
        if(_b == LOW) {
            _bitCnt++;
        }
        else {
            if(_bitCnt >= 8) {
                _gotIdle = true;
                _bitCnt = 0;
            }
            else {
                reset();
            }
        }
    }
    if(_gotIdle && !_gotStartbit) {
        _state = 0x8001;
        if(_b == HIGH && _bitCnt == 0) {
            _bitCnt++;
        }
        else if(_b == LOW && _bitCnt == 1) {
            _state = 0x8002;
            _gotStartbit = true;
            _bitCnt = 19;
            _dataCnt = 15;
            return;
        }
        else {
            reset();
            return;
        }
    }
    if(_gotIdle && _gotStartbit)  {
         _state = 0x8003;
        _bits += _bitCnt%5==0 ? " " : (_b ? "1" : "0");
        _stuff += _bitCnt%5==0 ? (_b ? "1" : "0") : " ";
        if(_bitCnt%5!=0) {
            _data |= (_b<<_dataCnt);
            _dataCnt--;
        }
        _bitCnt--;
        if(_bitCnt == -1) {
            // all bits read
            /*
            volatile int v[4];
            int idx;
            int n;
            for(idx = 0,  n=0; idx < 20; idx += 5, n++) {
                const char* p = _bits.substring(idx, idx+4).c_str();
                v[n] = strtol(p, NULL, 2);
            }
            unsigned dataWord = (v[0] << 12) | (v[1] << 8) | (v[2] << 4) | v[3];
            if(_data != dataWord)
                _state = 0x9000;
            else {
            */
                _isValid = false;
                if(_data > 0) {
                    _state = 0x8005;
                    uint8_t data8 = (uint8_t)((_data >> 8) ^ _data);
                    data8 ^= (data8 >> 4);
                    data8 ^= (data8 >> 2);
                    data8 ^= (data8 >> 1);
                    if ((data8 & 1) != 0)
                    {
                        _error = 0x8000;
                        _state = 0x8006;
                        reset();
                    }
                    else {
                        _state = 0x8007;
                        _isValid = true;
                        int cmd = (_data & 0x6000)>>13;
                        switch(cmd) {
                            case 0:
                                _state = 0x8008;
                                _switch = (_data & 0x1000) == 0;
                                _positionMM = _isV1 ? (double)((_data & 0x03ff)) * 0.02f : (double)((_data & 0x07ff)) *0.01f;
                                _position = _isV1 ? (_data & 0x03ff) : (_data & 0x07ff);
                                break;
                            case 1:
                                _state = 0x8009;
                                _error = _data & 0x0f;
                                break;
                            case 2:
                                _state = 0x800A;
                                _isV1 = true;
                                _version = 1;
                                _shutter = (_data & 0x1f00) >> 8;
                                _brightness = (_data & 0x00ff);
                                break;
                            case 3: 
                                {
                                    _state = 0x800B;
                                    int opt = (_data & 0x0f00) >> 8;
                                    switch(opt) {
                                        case 0:
                                            _version = _data & 0xff;
                                            break;
                                        case 1:
                                            _quality = _data & 0xff;
                                            break;
                                        case 2:
                                            _brightness = _data & 0xff;
                                            break;
                                        case 3:
                                            _shutter = _data & 0xff;
                                            break;
                                        default: 
                                            _error = 0x8001;
                                            break;
                                    }
                                }
                                break;
                            default:
                                _state = 0x800C;
                                _error = 0x8002;
                                break;
                        }
                    }
                }
            //}
            reset();
        }
    }
}
