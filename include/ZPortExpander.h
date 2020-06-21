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

#ifndef _PORTEXPANDER_H
#define _PORTEXPANDER_H 1

#include "SMuFF.h"
#include "ZTimerLib.h"
#include "PCF857X.h"

extern void __debug(const char* fmt, ...);

#define MAX_SERIAL      4
#define MAX_TX_BUFFER   256
#define MAX_RX_BUFFER   256

#define SERIAL0_TX      0
#define SERIAL0_RX      1
#define SERIAL1_TX      2
#define SERIAL1_RX      3
#define SERIAL2_TX      4
#define SERIAL2_RX      5
#define SERIAL3_TX      6
#define SERIAL3_RX      7

#define E_WRONG_SERIAL_INDEX    -1
#define E_SERIAL_IN_USE         -2

void isrSerialTimerHandler();

static ZTimer serialTimer;

typedef enum {
    BAUD_300_TIM    = 3333,
    BAUD_600_TIM    = 1667,
    BAUD_1200_TIM   = 833,
    BAUD_2400_TIM   = 417,
    BAUD_4800_TIM   = 204,
    BAUD_9600_TIM   = 104,
    BAUD_19200_TIM  = 52
} ZPortExpanderBaudrate;

class ZPortExpander {
public:
    ZPortExpander() { }
    ZPortExpander(int i2cAddress) { begin(i2cAddress, false); }
    ZPortExpander(int i2cAddress, bool is8575) { begin(i2cAddress, is8575); }

    void    begin(int i2cAddress, bool is8575, ZPortExpanderBaudrate baudrate = BAUD_600_TIM);
    void    pinMode(int pin, int mode)          { _pcf857x.pinMode(pin, mode); }
    int     digitalRead(int pin)                { return readPin(pin); }
    void    digitalWrite(int pin, int state)    { writePin(pin, state); }
    int     readPin(int pin)                    { return (pin == -1 || pin > _maxPin) ? 0 : _pcf857x.digitalRead(pin); }
    void    writePin(int pin, int state)        { if(pin != -1) _pcf857x.digitalWrite(pin, state); }
    void    setPin(int pin)                     { writePin(pin, HIGH); }
    void    resetPin(int pin)                   { writePin(pin, LOW); }
    void    togglePin(int pin)                  { _pcf857x.toggle(pin); }
    int     addSerial(int index = 0, int txPin = SERIAL0_TX, int rxPin = SERIAL0_RX);
    int     removeSerial(int index);
    int     beginSerial(int index);
    int     serialWrite(int index, const char* buffer);
    char    serialRead(int index);
    int     serialAvailable(int index);
    void    service();
    void    testSerial(const char* string);
    ZPortExpanderBaudrate getBaudrate() { return _baudrate; }

private:
    PCF857X _pcf857x = PCF857X();
    int     _maxPin = 7;
    ZPortExpanderBaudrate _baudrate;
};

class ZPESerial {
public:
    ZPESerial() { _txPin = -1; _rxPin = -1; _parent = NULL;}
    ZPESerial(ZPortExpander* parent, int txPin, int rxPin);

    void    receive();
    void    transmit();
    int     write(const char* buffer);
    char    read();
    ZPortExpander* getParent() { return _parent; }
    int     getRxPin() { return _rxPin; }
    int     getTxPin() { return _txPin; }
    int     available() { return availableRx(); }
    int     availableRx() { return (_rxBufferH < _rxBufferL) ? _rxBufferL - _rxBufferH : _rxBufferH - _rxBufferL; }
    int     availableTx() { return (_txBufferH < _txBufferL) ? _txBufferL - _txBufferH : _txBufferH - _txBufferL; }

private:
    ZPortExpander* _parent;
    byte    _txBuffer[MAX_TX_BUFFER];
    byte    _rxBuffer[MAX_RX_BUFFER];
    int     _txBufferH;
    int     _txBufferL;
    int     _rxBufferH;
    int     _rxBufferL;
    int     _rxPin;
    int     _txPin;
    bool    _rxReady;
    bool    _txReady;
    int8_t  _bitCntRx;
    int8_t  _bitCntTx;
    int8_t  _dataRx;
    int16_t _dataTx;
    bool    _gotStart;
    bool    _gotStop;
    int8_t  _prevState;
};

static ZPESerial        peSerials[MAX_SERIAL];

#endif
