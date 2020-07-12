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

#ifndef _SMUFF_CONFIG_H
#define _SMUFF_CONFIG_H

#define VERSION_STRING    "V2.09"
#define PMMU_VERSION      106               // Version number for Prusa MMU2 Emulation mode
#define PMMU_BUILD        372               // Build number for Prusa MMU2 Emulation mode
#define VERSION_DATE      "2020-07-06"
#define CONFIG_FILE       "SMUFF.CFG"
#define DATASTORE_FILE    "EEPROM.DAT"
#if defined(__STM32F1__)
#define MAX_JSON          2048              // 2K of temporary buffer for the JSON data
#elif defined(__ESP32__)
#define MAX_JSON          4096              // 4K of temporary buffer for the JSON data
#endif

#define NUM_STEPPERS      3
#define SELECTOR          0
#define REVOLVER          1
#define FEEDER            2

#define MIN_TOOLS         2
#define MAX_TOOLS         15

#define BEEPER_FREQUENCY    1760
#define BEEPER_DURATION     90
#define BEEPER_UFREQUENCY   440
#define BEEPER_UDURATION    90

#define DSP_CONTRAST        200
#define MIN_CONTRAST        60
#define MAX_CONTRAST        250

#define I2C_SLAVE_ADDRESS   0x88

#include "Pins.h"               // path is defined in build environment of platformio.ini (-I)

#define FIRST_TOOL_OFFSET       1.2   // value in millimeter
#define TOOL_SPACING            21.0  // value im millimeter
#define FIRST_REVOLVER_OFFSET   320   // value in steps
#define REVOLVER_SPACING        320   // value im steps
#define USER_MESSAGE_RESET      15    // value in seconds
#define MAX_LINES               5
#define MAX_LINE_LENGTH         80
#define POWER_SAVE_TIMEOUT      15    // value in seconds

#if !defined(NUM_LEDS)
#define NUM_LEDS                1     // number of Neopixel LEDS
#define BRIGHTNESS              64
#define LED_TYPE                WS2812B
#define COLOR_ORDER             GRB
#endif

#define BASE_FONT             u8g2_font_6x12_t_symbols
#define BASE_FONT_BIG         u8g2_font_7x14B_tf
#define SMALL_FONT            u8g2_font_6x10_mr
#define STATUS_FONT           u8g2_font_7x14_tf
#define LOGO_FONT             u8g2_font_helvR08_tf
#define ICONIC_FONT           u8g2_font_open_iconic_check_2x_t

#endif