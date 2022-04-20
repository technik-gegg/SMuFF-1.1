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
#pragma once

#include <U8g2lib.h>

#define I2C_SUCCESS         SUCCESS
#define I2C_TX_BUFFER_SIZE  32
#define I2C_RX_BUFFER_SIZE  32

#if !defined(USE_SW_TWI)
#include <Wire.h>
  extern TwoWire I2CBus;
  extern TwoWire I2CBus2;
#else
  #include <SoftWire.h>
  extern SoftWire I2CBus;
  extern SoftWire I2CBus2;
  extern void setupSoftWire(uint8_t bus=1);
#endif

uint8_t u8x8_byte_smuff_i2c(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr);

class SMUFF_SH1106_128X64_NONAME_F_I2C : public U8G2 {
  public: SMUFF_SH1106_128X64_NONAME_F_I2C(const u8g2_cb_t *rotation, uint8_t reset, uint8_t clock, uint8_t data) : U8G2() {
    u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, rotation, u8x8_byte_smuff_i2c, u8x8_gpio_and_delay_arduino);
    u8x8_SetPin_HW_I2C(getU8x8(), reset, clock, data);
  }
};

class SMUFF_SSD1306_128X64_NONAME_F_I2C : public U8G2 {
  public: SMUFF_SSD1306_128X64_NONAME_F_I2C(const u8g2_cb_t *rotation, uint8_t reset, uint8_t clock, uint8_t data) : U8G2() {
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, rotation, u8x8_byte_smuff_i2c, u8x8_gpio_and_delay_arduino);
    u8x8_SetPin_HW_I2C(getU8x8(), reset, clock, data);
  }
};

