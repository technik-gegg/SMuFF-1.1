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
/*
 * Pins configuration file for WANHAO i3 mini board
 */
#pragma once

#define BOARD_INFO          "Wanhao i3-Mini"
// SELECTOR
#define STEP_HIGH_X         PORTA |=  0b00000001;
#define STEP_LOW_X          PORTA &= ~0b00000001;
#define X_STEP_PIN          22
#define X_DIR_PIN           23
#define X_ENABLE_PIN        57
#define X_END_PIN           19
// REVOLVER
#define STEP_HIGH_Y         PORTA |=  0b00001000;
#define STEP_LOW_Y          PORTA &= ~0b00001000;
#define Y_STEP_PIN          25
#define Y_DIR_PIN           26
#define Y_ENABLE_PIN        24
#define Y_END_PIN           18
// FEEDER
#define STEP_HIGH_Z         PORTA |=  0b10000000;
#define STEP_LOW_Z          PORTA &= ~0b10000000;
#define Z_STEP_PIN          29
#define Z_DIR_PIN           39
#define Z_ENABLE_PIN        28
#define Z_END_PIN           38
#define Z_END2_PIN          -1
#define Z_END_DUET_PIN      -1

#define BEEPER_PIN          37

#define SERVO1_PIN          44
#define SERVO2_PIN          14
#define FAN_PIN             12
#define HEATER0_PIN         4

#define ENCODER1_PIN        2
#define ENCODER2_PIN        3
#define ENCODER_BUTTON_PIN  5

#define DSP_CS_PIN          41
#define DSP_DC_PIN          40
#define DSP_RESET_PIN       27

#define TX2_PIN             16
#define RX2_PIN             17

#define TX3_PIN             67
#define RX3_PIN             44
