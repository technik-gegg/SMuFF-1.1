/**
 * SMuFF Firmware
 * Copyright (C) 2020 Technik Gegg
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

#include "ZPortExpander.h"

static bool timerAction;

#ifdef __ESP32__
//static TaskHandle_t taskHandle;

/**
 * Experimental...
 * Tried to overcome the panic crash with relocating the serial port
 * handling (I2C) from timer ISR to an seperate task.
 * Unfortunatelly, the result is just the same as in the timer ISR.
 */
void timerServiceTask(void* parameter) {
    //ZPortExpander* instance = (ZPortExpander*)parameter;
    uint32_t val;
    for(;;) {
        xTaskNotifyWait(0, ULONG_MAX, &val, portMAX_DELAY);
        if(val == 0xAFFE) {
            for(uint8_t i=0; i < MAX_SERIAL; i++) {
                if(peSerials[i].getParent() != nullptr) {
                    peSerials[i].receive();
                    peSerials[i].transmit();
                }
            }
        }
    }
    vTaskDelete(nullptr);
}
#endif

#if !defined(USE_SW_TWI)
void ZPortExpander::begin(TwoWire* i2cInst, uint8_t i2cAddress, bool is8575, ZPortExpanderBaudrate baudrate) {
    _pcf857x.begin(i2cInst, i2cAddress, is8575 ? CHIP_PCF8575 : CHIP_PCF8574);
    _maxPin = is8575 ? 15 : 7;
    _baudrate = baudrate;
}
#else
void ZPortExpander::begin(SoftWire* i2cInst, uint8_t i2cAddress, bool is8575, ZPortExpanderBaudrate baudrate) {
    _pcf857x.begin(i2cInst, i2cAddress, is8575 ? CHIP_PCF8575 : CHIP_PCF8574);
    _maxPin = is8575 ? 15 : 7;
    _baudrate = baudrate;
}
#endif

void ZPortExpander::begin(uint8_t i2cAddress, bool is8575, ZPortExpanderBaudrate baudrate) {

    _pcf857x.begin(i2cAddress, is8575 ? CHIP_PCF8575 : CHIP_PCF8574);
    _maxPin = is8575 ? 15 : 7;
    _baudrate = baudrate;

    //__debugS(PSTR("Baud rate timing: %d"), (uint16_t)_baudrate);
#if defined(__ESP32__)
    // xTaskCreate(timerServiceTask, "ZPEServiceTask", 10000, this, 1, &taskHandle);

    serialTimer.setupTimer(Timer::TIMER3, 80);                  // 1us on 80MHz Timer Clock
    serialTimer.setupHook(isrSerialTimerHandler);
    serialTimer.setNextInterruptInterval((timerVal_t)_baudrate); // run serial timer to generate clock for baud rate
#endif
    for(uint8_t i=0; i < MAX_SERIAL; i++) {
        peSerials[i] = ZPESerial();
    }
}

/**
* This method must be called from within the main loop()
* if a serial port has been set up for communication.
* It checks whether the timer has triggered a bit clock
* and calls transmit() and receive() on all assigned
* serial ports.
*/
void ZPortExpander::service() {
    if(timerAction) {
        for(uint8_t i=0; i < MAX_SERIAL; i++) {
            if(peSerials[i].getParent() != nullptr) {
                peSerials[i].receive();
                peSerials[i].transmit();
            }
        }
        timerAction = false;
    }
}

/*
    Used for testing the software serial port on the PCF8574.
    Setup as: TX pin 4, RX on pin 3
*/
void ZPortExpander::testSerial(const char* string) {

    if(beginSerial(0) != -1) {
        serialWrite(0, string);
    }
}

/**
 * Convenience method to set up a virtual serial Port on
 * the Port Expander.
 * Pass in the index of the serial port (up to 4 allowed).
 * The index is needed in send and read operations.
 * For each serial port a pair of pins are used on the port
 * expander. Even pin numbers are TX lines (0,2,4,6), odd pin numbers
 * are RX lines (1,3,5,7).
 * If you need a different pin assignment, use addSerial() instead.
 */
int8_t ZPortExpander::beginSerial(uint8_t index) {

    int8_t retVal = 0;
    switch(index) {
        case 0:
            retVal = addSerial(index, SERIAL0_TX, SERIAL0_RX);
            break;
        case 1:
            retVal = addSerial(index, SERIAL1_TX, SERIAL1_RX);
            break;
        case 2:
            retVal = addSerial(index, SERIAL2_TX, SERIAL2_RX);
            break;
        case 3:
            retVal = addSerial(index, SERIAL3_TX, SERIAL3_RX);
            break;
    }
    return retVal;
}

/**
 * Adds a virtual serial port on the Port Expander.
 * Define the index (number) of the port and assign the
 * receive and transmit pins. If you only need to transmit,
 * assign the receive pin to -1 and vice versa.
*/
int8_t ZPortExpander::addSerial(uint8_t index, int8_t txPin, int8_t rxPin) {
    if(index < 0 || index > MAX_SERIAL) {
        // wrong serial index
        return E_WRONG_SERIAL_INDEX;
    }
    if(peSerials[index].getParent() != nullptr) {
        // serial already added
        return E_SERIAL_IN_USE;
    }
    ZPESerial serial = ZPESerial(this, txPin, rxPin);
    peSerials[index] = serial;
    return 0;
}

int8_t ZPortExpander::removeSerial(uint8_t index) {
    ZPESerial serial = ZPESerial();
    peSerials[index] = serial;
    return 0;
}

/**
 * Reads the next character from the receive buffer on the
 * given channel (index).
 * Before calling this method, it's neccessary to check whether
 * or not new data is pending by calling available().
 * Otherwise you'll get 0 as an results.
 * Returns a character from the buffer.
 */
char ZPortExpander::serialRead(uint8_t index) {
    return peSerials[index].read();
}

/**
 * Checks if there is something in the receive buffer on the
 * given channel (index).
 * Returns the number of characters available.
*/
uint16_t ZPortExpander::serialAvailable(uint8_t index) {
    return peSerials[index].available();
}

/**
 * Writes data to the transmit buffer for sending.
 * The buffer is being copied character wise. If
 * a buffer overflow occurs, it may not send the whole
 * buffer but stop somewhere in the middle.
 * To prevent this, make sure you're not sending overly
 * large strings.
 * Returns the amount of bytes copied to the transmit buffer.
 */
uint16_t ZPortExpander::serialWrite(uint8_t index, const char* buffer) {
    if(index >=0 && index < MAX_SERIAL) {
        if(strlen(buffer) > 0) {
            int cnt = peSerials[index].write(buffer);
            return cnt;
        }
    }
    return 0;
}

/**
 * Subordinate class to mimic a serial communication over the pins
 * of the Port Expander.
 * PLEASE NOTE:
 * This method allows a reliable serial communication with up to 600 baud
 * in the 8,N,1 format (8 data bits, no parity, 1 stop bit).
 * It'll work up to 1200 baud in case your main thread ain't to busy and
 * you have some higher-level protocol which recognizes and filters transmission
 * errors.
 * So it's feasible for a serial data exchange where the amount of data is
 * very little and speed doesn't matter much (i.e. sending/receiving temperature
 * or humidity data to/from a subsystem).
 * If you need something faster than this, you'll have to use a
 *  hardware based I2C to UART solution.
 * The bit clock for this serial interface is being generated through a timer,
 * which guarantees a decent implementation of the timing for the serial protocol.
 */
ZPESerial::ZPESerial(ZPortExpander* parent, int8_t txPin, int8_t rxPin) {
    _parent = parent;
    _txPin = txPin;
    _rxPin = rxPin;
    _txBufferH = 0;
    _txBufferL = 0;
    _rxBufferH = 0;
    _rxBufferL = 0;
    _gotStart = false;
    _gotStop = true;
    _rxReady = false;
    _txReady = true;
    memset(_txBuffer, 0, MAX_TX_BUFFER);
    memset(_rxBuffer, 0, MAX_RX_BUFFER);
    if(txPin != -1) {
        _parent->pinMode(_txPin, OUTPUT);
        _parent->writePin(_txPin, HIGH);
    }
    if(rxPin != -1) {
        _parent->pinMode(_rxPin, INPUT_PULLUP);
        _prevState = HIGH;
    }
}

/**
 * Main method to write a string into the transmit buffer
 * and maintain the underlying ring buffer accordingly.
 * Returns the amount of bytes copied to the transmit buffer.
 */
uint16_t ZPESerial::write(const char* buffer) {
    // copy everything into the transmit buffer
    size_t cnt=0;
    for(size_t i=0; i < strlen(buffer); i++) {
        _txBuffer[_txBufferH] = buffer[i];
        cnt++;
        _txBufferH = (_txBufferH+1 >= MAX_TX_BUFFER) ?  0 : _txBufferH+1;
        if(_txBufferH == _txBufferL) {
            delay(50);
            // check and abort on buffer overflow
            if(_txBufferH == _txBufferL)
                return cnt;
        }
    }
    return cnt;
    //__debugS(PSTR("TX-L: %d TX-H: %d / RX-L: %d RX-H: %d / Avail: TX: %d RX: %d"), _txBufferL, _txBufferH, _rxBufferL, _rxBufferH, availableTx(), available());
}

/**
 * This method is similar to the usual Serial.read().
 * It'll return the next character from the receive.
 * To figure out whether or not received data is pending,
 * use the available() method.
 * For the ZPortExpander you don't access this method
 * directly, you rather use serialWrite()
 */
char ZPESerial::read() {
    if(availableRx() == 0)
        return 0;
    char c = _rxBuffer[_rxBufferL];
    _rxBufferL = (_rxBufferL+1 >= MAX_RX_BUFFER) ? 0 : _rxBufferL+1;
    return c;
}

static uint8_t syncCnt = 0;
/**
 * Main receiver method.
 * This method is being called periodically by service()
 * for each bit to read at the given bit clock speed
 * in order to collect a byte from the RX line.
 */
void ZPESerial::receive() {

    int8_t curState = _parent->digitalRead(_rxPin);
    // loop until we get at least 10 highs in a row just
    // to make sure we don't read something in the middle
    // of an ongoing transmission
    if(!_rxReady) {
        syncCnt = (curState == HIGH) ? syncCnt+1 : 0;
        if(syncCnt >= 10)
            _rxReady = true;
    }
    else {
        // waiting for start bit
        if(!_gotStart){
            if(_prevState == HIGH && curState == LOW) {
                // got start bit
                _gotStart = true;
                _gotStop = false;
                _dataRx = 0;
                _bitCntRx = 0;
            }
        }
        else {
            // reading 8 data bits
            if(_bitCntRx <= 7) {
                if(curState)
                    _dataRx |= 1 << _bitCntRx;
                _bitCntRx++;
            }
            else {
                // reading stop bit
                if(curState == HIGH) {
                    _gotStop = true;
                    _gotStart = false;
                }
                // got all bits and stop bit, put byte in receive buffer
                _rxBuffer[_rxBufferH] = (byte)_dataRx;
                _rxBufferH = (_rxBufferH+1 >= MAX_RX_BUFFER) ? 0 : _rxBufferH+1;
                if(_rxBufferH == _rxBufferL) {
                    _rxBufferL = (_rxBufferL+1 >= MAX_RX_BUFFER) ? 0 : _rxBufferL+1;
                }
            }
        }
    }
    _prevState = curState;
}

/**
 * Main transmitter method.
 * This method is being called periodically by service()
 * for each bit to send at the given bit clock speed
 * in order to transmitt all data in the transmit buffer.
 */
void ZPESerial::transmit() {
    if(_txPin == -1)
        return;
    if(_txReady && !availableTx()) {
        // nothing in the buffer to send yet
        return;
    }
    else {
        if(_txReady) {
            _txReady = false;
            // setup the data packet (8,N,1) with an inverted order: stop bit + 8 data bits + start bit (10 bits total)
            _dataTx = 0xFE00 | (int16_t)(_txBuffer[_txBufferL] << 1);
            _bitCntTx = 0;
            if(_txBufferL++ == MAX_TX_BUFFER)
                _txBufferL = 0;
        }
        if(!_txReady && (_bitCntTx <= 10)) {
            ((_dataTx & (1 << _bitCntTx)) == 0) ? _parent->resetPin(_txPin) : _parent->setPin(_txPin); // send next bit
            _bitCntTx++;
        }
        if(!_txReady && (_bitCntTx > 10)) {
            _txReady = true;
            _parent->setPin(_txPin);
        }
    }
}

/*
 This ISR is called periodically from the timer to generate
 a bit clock for the software serial.
 Would have been much nicer to process sending in
 this ISR, but unfortunatelly this causes a panic
 when I2C is being accessed within an ISR on ESP32.
 Hence, we only set a flag in here, which gets processed
 by the service() method, which is also responsible to reset
 the flag.
 */
void isrSerialTimerHandler() {
/*
#if defined(__ESP32__)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(taskHandle, 0xAFFE, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR();
#endif
*/
    timerAction = true;
}
