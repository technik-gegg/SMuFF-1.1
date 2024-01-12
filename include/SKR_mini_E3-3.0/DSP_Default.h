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
/*
 * Pins configuration file for "standard" 1701 RRF Display
 */
#pragma once
//#pragma message "Compiling for Standard RRF Display"

// SPECIAL CONFIGURATION, WORKS ONLY WITH CUSTOM MADE CABLE!
#define DSP_DATA_PIN        -1      // USE MOSI ON SPI1 HEADER
#define DSP_CS_PIN          PB9     // EXP1.5 = LCD_PINS_RS = ST9720 CS
#define DSP_DC_PIN          PA15    // EXP1.9
#define DSP_RESET_PIN       -1
//#define DSP_RESET_PIN       PB15    // EXP1.3

#define ENCODER1_PIN        PA9     // EXP1.8
#define ENCODER2_PIN        PA10    // EXP1.6
#define ENCODER_BUTTON_PIN  PB8     // EXP1.4

#define BEEPER_PIN          PB5     // EXP1.10
