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

#include "SMuFF.h"
#include "./commonI2C.h"

#if defined(USE_MULTISERVO)
  // Adafruit Multiservo is always on software I2C
  SoftWire I2CBusMS(ADASERVO_SDA, ADASERVO_SCL, SOFT_FAST);
#endif

#if !defined(USE_SW_TWI)
  #if defined(USE_I2C)
  TwoWire I2CBus(DSP_SDA, DSP_SCL);
  #else
  TwoWire I2CBus;
  #endif
  #if defined(USE_SPLITTER_ENDSTOPS)
  TwoWire I2CBus2(SPLITTER_SDA, SPLITTER_SCL);
  #endif
#else
  SoftWire I2CBus(DSP_SDA, DSP_SCL);
  #if defined(USE_SPLITTER_ENDSTOPS)
  SoftWire I2CBus2(SPLITTER_SDA, SPLITTER_SCL);
  #endif
#endif

static uint32_t transmissionCnt = 0;

uint8_t u8x8_byte_smuff_i2c(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr) {
  
  switch(msg) {
    case U8X8_MSG_BYTE_SEND:
      #if defined(USE_LEONERD_DISPLAY)
        while (encoder.busy())
          ;
      #endif
      I2CBus.write((uint8_t *)arg_ptr, (size_t)arg_int);
      break;

    case U8X8_MSG_BYTE_INIT:
      if (u8x8->bus_clock == 0)
        u8x8->bus_clock = u8x8->display_info->i2c_bus_clock_100kHz * 100000UL;
      __debugS(DEV, PSTR("\tBegin I2C called, clock: %ld"), u8x8->bus_clock);
      I2CBus.begin();
      __debugS(DEV, PSTR("\tBegin I2C done"));
      break;

    case U8X8_MSG_BYTE_SET_DC:
      break;

    case U8X8_MSG_BYTE_START_TRANSFER:
      #if defined(USE_LEONERD_DISPLAY)
        while (encoder.busy())
          ;
      #endif
      I2CBus.setClock(u8x8->bus_clock);
      I2CBus.beginTransmission(u8x8_GetI2CAddress(u8x8)>>1);
      break;

    case U8X8_MSG_BYTE_END_TRANSFER:
      #if defined(USE_LEONERD_DISPLAY)
        while (encoder.busy())
          ;
      #endif
      I2CBus.endTransmission();
      break;

    default:
      return 0;
  }
  return 1;
}
