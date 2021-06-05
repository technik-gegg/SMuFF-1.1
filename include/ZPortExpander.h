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

#include "SMuFF.h"
#include "PCF857X.h"

extern void __debugS(const char* fmt, ...);

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

static Timer serialTimer;

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
    ZPortExpander(uint8_t i2cAddress) { begin(i2cAddress, false); }
    ZPortExpander(uint8_t i2cAddress, bool is8575) { begin(i2cAddress, is8575); }
    #if !defined(USE_SW_TWI)
    ZPortExpander(TwoWire* i2cInst, uint8_t i2cAddress, bool is8575) { begin(i2cInst, i2cAddress, is8575); }
    #else
    ZPortExpander(SoftWire* i2cInst, uint8_t i2cAddress, bool is8575) { begin(i2cInst, i2cAddress, is8575); }
    #endif

    #if !defined(USE_SW_TWI)
    void        begin(TwoWire* i2cInst, uint8_t i2cAddress, bool is8575, ZPortExpanderBaudrate baudrate = BAUD_600_TIM);
    #else
    void        begin(SoftWire* i2cInst, uint8_t i2cAddress, bool is8575, ZPortExpanderBaudrate baudrate = BAUD_600_TIM);
    #endif
    void        begin(uint8_t i2cAddress, bool is8575, ZPortExpanderBaudrate baudrate = BAUD_600_TIM);
    void        pinMode(uint8_t pin, uint8_t mode)     { _pcf857x.pinMode(pin, mode); }
    uint8_t     digitalRead(int8_t pin)                { return readPin(pin); }
    void        digitalWrite(int8_t pin, int8_t state) { writePin(pin, state); }
    uint8_t     readPin(int8_t pin)                    { return (pin == -1 || pin > _maxPin) ? 0 : _pcf857x.digitalRead(pin); }
    void        writePin(int8_t pin, uint8_t state)    { if(pin != -1) _pcf857x.digitalWrite(pin, state); }
    void        setPin(int8_t pin)                     { writePin(pin, HIGH); }
    void        resetPin(int8_t pin)                   { writePin(pin, LOW); }
    void        togglePin(int8_t pin)                  { _pcf857x.toggle(pin); }
    int8_t      addSerial(uint8_t index = 0, int8_t txPin = SERIAL0_TX, int8_t rxPin = SERIAL0_RX);
    int8_t      removeSerial(uint8_t index);
    int8_t      beginSerial(uint8_t index);
    uint16_t    serialWrite(uint8_t index, const char* buffer);
    char        serialRead(uint8_t index);
    uint16_t    serialAvailable(uint8_t index);
    void        service();
    void        testSerial(const char* string);
    ZPortExpanderBaudrate getBaudrate() { return _baudrate; }

private:
    PCF857X _pcf857x = PCF857X();
    int8_t  _maxPin = 7;
    ZPortExpanderBaudrate _baudrate;
};

class ZPESerial {
public:
    ZPESerial() { _txPin = -1; _rxPin = -1; _parent = nullptr;}
    ZPESerial(ZPortExpander* parent, int8_t txPin, int8_t rxPin);

    void        receive();
    void        transmit();
    uint16_t    write(const char* buffer);
    char        read();
    ZPortExpander* getParent() { return _parent; }
    int8_t      getRxPin() { return _rxPin; }
    int8_t      getTxPin() { return _txPin; }
    uint16_t    available() { return availableRx(); }
    uint16_t    availableRx() { return (_rxBufferH < _rxBufferL) ? _rxBufferL - _rxBufferH : _rxBufferH - _rxBufferL; }
    uint16_t    availableTx() { return (_txBufferH < _txBufferL) ? _txBufferL - _txBufferH : _txBufferH - _txBufferL; }

private:
    ZPortExpander* _parent;
    byte        _txBuffer[MAX_TX_BUFFER];
    byte        _rxBuffer[MAX_RX_BUFFER];
    uint16_t    _txBufferH;
    uint16_t    _txBufferL;
    uint16_t    _rxBufferH;
    uint16_t    _rxBufferL;
    int8_t      _rxPin;
    int8_t      _txPin;
    bool        _rxReady;
    bool        _txReady;
    int8_t      _bitCntRx;
    int8_t      _bitCntTx;
    int8_t      _dataRx;
    int16_t     _dataTx;
    bool        _gotStart;
    bool        _gotStop;
    int8_t      _prevState;
};

static ZPESerial        peSerials[MAX_SERIAL];
