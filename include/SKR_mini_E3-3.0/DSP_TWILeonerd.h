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
 * Pins configuration file for TWI/I2C and Leonerd Display
 */
#pragma once

#if !defined(USE_SW_TWI)
#define DSP_SCL             PA9     // EXP1.8
#define DSP_SDA             PA10    // EXP1.6
#else
#define DSP_SCL             PA15    // EXP1.9
#define DSP_SDA             PD6     // EXP1.3
#endif

#define DSP_CS_PIN          -1
#define DSP_DC_PIN          -1
#define DSP_RESET_PIN       -1

#if !defined(USE_LEONERD_DISPLAY)
//#pragma message "Compiling for TWI/I2C Display"
#define ENCODER1_PIN        PB9     // EXP1.5
#define ENCODER2_PIN        PD6     // EXP1.3
#define ENCODER_BUTTON_PIN  PB8     // EXP1.4
#else
//#pragma message "Compiling for Leonerd Display"
#endif 

#define BEEPER_PIN          PB5     // EXP1.10

