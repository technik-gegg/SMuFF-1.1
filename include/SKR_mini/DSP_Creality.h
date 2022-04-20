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
/*
 * Pins configuration file for Creality E3/CR10 Display
 */
#pragma once

#if !defined(CREALITY_HW_SPI)
//#pragma message "Invalid configuration on this board"
#else
//#pragma message "Compiling for Creality E3/CR10 Display HW SPI"

#define DSP_DATA_PIN        -1      // USE MOSI ON SPI1 HEADER
#define DSP_CS_PIN          PB7     // EXP1.5
#define DSP_DC_PIN          PC15    // EXP1.4 
#define DSP_RESET_PIN       -1

#define ENCODER1_PIN        PD2     // EXP2.8
#define ENCODER2_PIN        PB8     // EXP2.6
#define ENCODER_BUTTON_PIN  PC11    // EXP1.9

#endif

#define BEEPER_PIN          PC10    // EXP1.10
