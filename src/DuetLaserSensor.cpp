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

#include "DuetLaserSensor.h"

#include "Debug.h"

/* ==============================================================================================================================
    Duet3D laser Sensor Protocol (according to the Duet3D Wiki)
    P = partity bit
   ==============================================================================================================================
    v1 Data word	        P00S 00pp pppp pppp	    S = switch open, pppppppppp = 10-bit filament position (50 counts/mm)
    v1 Error word	        P010 0000 0000 0000
    v1 Quality word	        P10s ssss bbbb bbbb	    sssss = shutter, bbbbbbbb = brightness

    v2 Data word	        P00S 1ppp pppp pppp	    S = switch open, ppppppppppp = 11-bit filament position (100 counts/mm)
    v2 Error word	        P010 0000 0000 eeee	    eeee = error code, will not be zero
    v2 Version word	        P110 0000 vvvv vvvv	    vvvvvvvv = sensor/firmware version, at least 2
    v2 Image quality word	P110 0001 qqqq qqqq	    qqqqqqqq = image quality
    v2 Brightness word	    P110 0010 bbbb bbbb	    bbbbbbbb = brightness
    v2 Shutter word	        P110 0011 ssss ssss	    ssssssss = shutter
 ============================================================================================================================== */

void DuetLaserSensor::attach(pin_t pin, bool v1) {
    _pin = pin;
    _isV1 = v1;
    pinMode(_pin, INPUT);
    reset();
    resetPosition();
}

void DuetLaserSensor::resetPosition() {
    _position = 0;
    _positionMM = 0;
}

void DuetLaserSensor::reset() {
    _gotStartbit = false;
    _gotIdle = false;
    _bitCnt = 0;
    _dataCnt = 0;
    _data = 0;
    resetBits();
}

/*  Method which reads out the Duet3D laser Sensor.
    This service method must be called once each milisecond.

    The data is being transmitted from the sensor in the following format:

    Idle state: the line must be be at 0 for at least 8 bit times
    Start bits: 1 followed by 0
    Data bits 15, 14, 13, 12
    Stuffing bit (inverse of bit 12)
    Data bits 11,10,9,8
    Stuffing bit (inverse of bit 8)
    Data bits 7,6,5,4
    Stuffing bit (inverse of bit 4)
    Data bits 3,2,1,0
    Stuffing bit (inverse of bit 0)
    After the last stuffing bit, the line returns to 0 until the next start bit.
*/
void DuetLaserSensor::service() {
    if(_pin == 0)
        return;

    volatile uint8_t _b = digitalRead(_pin);
    // wait for idle to be signaled (=8 bits low)
    if(!_gotIdle) {
        _state = STATE_INIT;
        if(_b == LOW) {
            _bitCnt++;
        }
        else {
            if(_bitCnt >= 8) {
                _gotIdle = true;
                _bitCnt = 0;
                _state = STATE_IDLE;
            }
            else {
                reset();
            }
        }
    }
    // wait for startbits
    if(_gotIdle && !_gotStartbit) {
        if(_b == HIGH && _bitCnt == 0) {
            _bitCnt++;
            _state = STATE_WAIT_START;
        }
        else if(_b == LOW && _bitCnt == 1) {
            _state = STATE_GOT_START;
            _gotStartbit = true;
            _bitCnt = 19;
            _pbitCnt = 0;
            _dataCnt = 15;
            return;
        }
        else {
            reset();
            return;
        }
    }
    // read data bits
    if(_gotIdle && _gotStartbit)  {
         _state = STATE_READING_DATA;
         /* FOR DEBUGGING ONLY
        _bits += _bitCnt%5==0 ? " " : (_b ? "1" : "0");
        _stuff += _bitCnt%5==0 ? (_b ? "1" : "0") : " ";
        */
        if(_bitCnt%5!=0) {
            _data |= (_b<<_dataCnt);
            _dataCnt--;
            _pbitCnt += _b ? 1 : 0;     // count all '1's for parity check
        }
        _bitCnt--;
        if(_bitCnt == -1) {
            // all bits read
            _isValid = false;
            if(_data > 0) {
                _state = STATE_GOT_ALL_BITS;
                _error = E_NONE;
                // check parity
                if (_pbitCnt % 2 != (uint8_t)(_data & 0x8000) >> 15)
                {
                    _error = E_WRONG_PARITY;
                    _state = STATE_PARITY_ERROR;
                    reset();
                }
                else {
                    _state = STATE_PARITY_OK;
                    uint16_t cmd = (_data & 0x6000)>>13;
                    switch(cmd) {
                        case 0: {
                            _state = STATE_GOT_POSITION;
                            _switch = (_data & 0x1000) == 0;
                            _prevpos = _position;
                            int32_t pos = _isV1 ? (_data & 0x03ff) : (_data & 0x07ff);

                            if(pos == _prevpos) {
                                _dir = DIR_NONE;
                            }
                            else if(pos < _prevpos) {
                                _dir = (pos - _prevpos < 0) ? DIR_RETRACT : DIR_EXTRUDE;
                            }
                            else if(pos > _prevpos) {
                                _dir = (pos - _prevpos > 0) ? DIR_EXTRUDE : DIR_RETRACT;
                            }
                            _position = pos;
                            _positionMM = _isV1 ? (double)_position * 0.02f : (double)_position * 0.01f;
                            _hasMoved = _prevpos != _position;
                            }
                            break;
                        case 1:
                            _state = STATE_GOT_ERROR_MSG;
                            _sensorError = _data & 0x0f;
                            break;
                        case 2:
                            if(_isV1) {
                                _state = STATE_GOT_QUALITY_V1;
                                _version = 1;
                                _shutter = (_data & 0x1f00) >> 8;
                                _brightness = (_data & 0x00ff);
                            }
                            else {
                                _state = STATE_INVALID;
                                _error = E_INVALID_VERSION;
                            }
                            break;
                        case 3:
                            {
                                _state = STATE_GOT_QUALITY_V2;
                                _error = E_NONE;
                                uint16_t opt = (_data & 0x0f00) >> 8;
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
                                        _error = E_INVALID_QUALITY;
                                        break;
                                }
                            }
                            break;
                        default:
                            _state = STATE_INVALID;
                            _error = E_INVALID_DATA;
                            break;
                    }
                    if(_error == E_NONE)
                        _isValid = true;
                }
            }
            reset();
        }
    }
}
