/*
 * See header file for details
 *
 *  This program is free software: you can redistribute it and/or modify\n
 *  it under the terms of the GNU General Public License as published by\n
 *  the Free Software Foundation, either version 3 of the License, or\n
 *  (at your option) any later version.\n
 *
 *  This program is distributed in the hope that it will be useful,\n
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of\n
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n
 *  GNU General Public License for more details.\n
 *
 *  You should have received a copy of the GNU General Public License\n
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.\n
 */

/*
	Technik Gegg 2020-05-06:
		Corrected ::updateGPIO() method to respect either 8 bit (8574) or 16 bit (8575)
	Technik Gegg 2021-05-31:
		changed I2C access to use HAL
*/

/* Dependencies */
//#include <Wire.h>
#include "PCF857X.h"
#ifdef PCF857X_INTERRUPT_SUPPORT
#include "PCint.h"
#endif

extern void __debugS(const char *fmt, ...);

// max pins of chips: 8 pins for PCF8574, 16 pins for PCF8575
uint8_t PCF857X_MAX_PINS[2] = { 8, 16 };

PCF857X::PCF857X() :
		_PORT(0), _PIN(0), _DDR(0), _address(0)
#ifdef PCF857X_INTERRUPT_SUPPORT
		, _oldPIN(0), _isrIgnore(0), _pcintPin(0), _intMode(), _intCallback()
#endif
{
}

#if !defined(USE_SW_TWI)
void PCF857X::begin(TwoWire* i2cInst, uint8_t address, uint8_t chip) {
	_i2cBusInst = i2cInst;
	begin(address, chip);
}
#else
void PCF857X::begin(SoftWire* i2cInst, uint8_t address, uint8_t chip) {
	_i2cBusInst = i2cInst;
	begin(address, chip);
}
#endif

void PCF857X::begin(uint8_t address, uint8_t chip) {
	/* Store the I2C address */
	_address = address;
	/* Store the chip type */
	_chip = chip;
	/* Init the Wire library */
	if(_i2cBusInst != nullptr) {
		_i2cBusInst->begin();
		readGPIO();
	}
	else {
		__debugS(PSTR("I2C Bus Instance not set"));
	}
}

void PCF857X::pinMode(uint8_t pin, uint8_t mode) {

	/* Switch according mode */
	switch (mode) {
	case INPUT:
		_DDR &= ~(1 << pin);
		_PORT &= ~(1 << pin);
		break;

	case INPUT_PULLUP:
		_DDR &= ~(1 << pin);
		_PORT |= (1 << pin);
		break;

	case OUTPUT:
		_DDR |= (1 << pin);
		_PORT &= ~(1 << pin);
		break;

	default:
		break;
	}

	/* Update GPIO values */
	updateGPIO();
}

void PCF857X::digitalWrite(uint8_t pin, uint8_t value) {

	/* Set PORT bit value */
	if (value)
		_PORT |= (1 << pin);
	else
		_PORT &= ~(1 << pin);

	/* Update GPIO values */
	updateGPIO();
}

uint8_t PCF857X::digitalRead(uint8_t pin) {

	/* Read GPIO */
	readGPIO();

#ifdef PCF857X_INTERRUPT_SUPPORT
	/* Check for interrupt (manual detection) */
	//checkForInterrupt();
#endif

	/* Read and return the pin state */
	return (_PIN & (1 << pin)) ? HIGH : LOW;
}

void PCF857X::write(uint16_t value) {

	/* Store pins values and apply */
	_PORT = value;

	/* Update GPIO values */
	updateGPIO();
}

uint16_t PCF857X::read() {

	/* Read GPIO */
	readGPIO();

#ifdef PCF857X_INTERRUPT_SUPPORT
	/* Check for interrupt (manual detection) */
	//checkForInterrupt();
#endif

	/* Return current pins values */
	return _PIN;
}

void PCF857X::pullUp(uint8_t pin) {

	/* Same as pinMode(INPUT_PULLUP) */
	pinMode(pin, INPUT_PULLUP); // /!\ pinMode form THE LIBRARY
}

void PCF857X::pullDown(uint8_t pin) {

	/* Same as pinMode(INPUT) */
	pinMode(pin, INPUT); // /!\ pinMode form THE LIBRARY
}

void PCF857X::clear() {

	/* User friendly wrapper for write() */
	if (_chip == CHIP_PCF8575) {
		write(0x0000);
	} else {
		write(0x00);
	}
}

void PCF857X::set() {

	/* User friendly wrapper for write() */
	if (_chip == CHIP_PCF8575) {
		write(0xFFFF);
	} else {
		write(0xFF);
	}
}

void PCF857X::toggle(uint8_t pin) {

	/* Toggle pin state */
	_PORT ^= (1 << pin);

	/* Update GPIO values */
	updateGPIO();
}

void PCF857X::blink(uint8_t pin, uint16_t count, uint32_t duration) {

	/* Compute steps duration */
	duration /= count * 2;

	/* Loop n times */
	while (count--) {

		/* Toggle pin 2 times */
		toggle(pin);
		delay(duration);
		toggle(pin);
		delay(duration);
	}
}

#ifdef PCF857X_INTERRUPT_SUPPORT
void PCF857X::enableInterrupt(uint8_t pin, void (*selfCheckFunction)(void)) {

	/* Store interrupt pin number */
	_pcintPin = pin;

	/* Setup interrupt pin */
#if ARDUINO >= 100
	::pinMode(pin, INPUT_PULLUP); // /!\ pinMode form THE ARDUINO CORE
#else
	::pinMode(pin, INPUT); // /!\ pinMode form THE ARDUINO CORE
	::digitalWrite(pin, HIGH); // /!\ digitalWrite form THE ARDUINO CORE
#endif

	/* Attach interrupt handler */
	PCattachInterrupt(pin, selfCheckFunction, FALLING);
}

void PCF857X::disableInterrupt() {

	/* Detach interrupt handler */
	PCdetachInterrupt(_pcintPin);
}

void PCF857X::checkForInterrupt() {

	/* Avoid nested interrupt triggered by I2C read/write */
	if(_isrIgnore)
		return;
	else
		_isrIgnore = 1;

	/* Re-enable interrupts to allow Wire library to work */
	sei();

	/* Read current pins values */
	readGPIO();

	/* Check all pins */
	for (uint8_t i = 0; i < PCF857X_MAX_PINS[_chip]; ++i) {

		/* Check for interrupt handler */
		if (!_intCallback[i])
			continue;

		/* Check for interrupt event */
		switch (_intMode[i]) {
		case CHANGE:
			if ((1 << i) & (_PIN ^ _oldPIN))
				_intCallback[i]();
			break;

		case LOW:
			if (!(_PIN & (1 << i)))
				_intCallback[i]();
			break;

		case FALLING:
			if ((_oldPIN & (1 << i)) && !(_PIN & (1 << i)))
				_intCallback[i]();
			break;

		case RISING:
			if (!(_oldPIN & (1 << i)) && (_PIN & (1 << i)))
				_intCallback[i]();
			break;
		}
	}

	/* Turn off ISR ignore flag */
	_isrIgnore = 0;
}

void PCF857X::attachInterrupt(uint8_t pin, void (*userFunc)(void),
		uint8_t mode) {

	/* Store interrupt mode and callback */
	_intMode[pin] = mode;
	_intCallback[pin] = userFunc;
}

void PCF857X::detachInterrupt(uint8_t pin) {

	/* Void interrupt handler */
	_intCallback[pin] = 0;
}
#endif

void PCF857X::readGPIO() {

#ifdef PCF857X_INTERRUPT_SUPPORT
	/* Store old _PIN value */
	_oldPIN = _PIN;
#endif

	/* Start request, wait for data and receive GPIO values as byte */
	if (_chip == CHIP_PCF8575) {
		_i2cBusInst->requestFrom(_address, (uint8_t) 0x02);
		if (_i2cBusInst->available() >= 2) {
			_PIN = I2CREAD(); /* LSB first */
			_PIN |= I2CREAD() << 8;
		}
	}
	else {
		_i2cBusInst->requestFrom(_address, (uint8_t) 0x01);
		if (_i2cBusInst->available() >= 1) {
			// Technik Gegg: Corrected wrong assignment & shifting
			_PIN = I2CREAD();
		}
	}
}

/*
	Technik Gegg: Corrected write function to respect either 8 bit (8574) or 16 bit (8575)
*/
void PCF857X::updateGPIO() {

	/* Start communication and send GPIO values as byte */
	_i2cBusInst->beginTransmission(_address);
	if (_chip == CHIP_PCF8575) {
		/* Compute new GPIO states */
		uint16_t value = (_PIN & ~_DDR) | _PORT;
		I2CWRITE(value & 0x00FF);
		I2CWRITE((value & 0xFF00) >> 8);
	}
	else {
		/* Compute new GPIO states */
		uint8_t value = (_PIN & ~_DDR) | _PORT;
		I2CWRITE(value & 0xFF);
	}
	_i2cBusInst->endTransmission();
}
