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
 * Pins configuration file for ESP32 board
 */
#pragma once

#define BOARD_INFO          "ESP32"
// SELECTOR
#define STEP_HIGH_X         digitalWrite(X_STEP_PIN, HIGH);
#define STEP_LOW_X          digitalWrite(X_STEP_PIN, LOW);
#define X_STEP_PIN          27      // IO27
#define X_DIR_PIN           26      // IO26
#define X_ENABLE_PIN        25      // IO25
#define X_END_PIN           34      // IO34
// REVOLVER - not used at all
#define STEP_HIGH_Y         digitalWrite(Y_STEP_PIN, HIGH);
#define STEP_LOW_Y          digitalWrite(Y_STEP_PIN, LOW);
#define Y_STEP_PIN          -1
#define Y_DIR_PIN           -1
#define Y_ENABLE_PIN        -1
#define Y_END_PIN           -1
// FEEDER
#define STEP_HIGH_Z         digitalWrite(Z_STEP_PIN, HIGH);
#define STEP_LOW_Z          digitalWrite(Z_STEP_PIN, LOW);
#define Z_STEP_PIN          14      // IO14
#define Z_DIR_PIN           12      // IO12
#define Z_ENABLE_PIN        33      // IO33
#define Z_END_PIN           35      // IO35
#define Z_END2_PIN          -1
#define Z_END_DUET_PIN      -1

#define LED_PIN             2       // IO2

#define DEBUG_PIN           -1     
#define RELAIS_PIN          -1     // Relais for stepper motor switching

#define BEEPER_CHANNEL      1
#define BEEPER_PIN          2       // IO2

#define SERVO1_PIN          4       // IO4
#define SERVO2_PIN          15      // IO15

#define FAN_CHANNEL         1
#define FAN_FREQ            5000
#define FAN_PIN             13      // IO13
#define HEATER0_PIN         -1      
#define HEATBED_PIN         -1

#define SDCS_PIN            5       // IO5
#define DSP_CS_PIN          13      // IO13
#define DSP_DC_PIN          19      // IO19 (MISO)
#define DSP_RESET_PIN       -1      

#define DSP_SCL             22      // SCL     
#define DSP_SDA             21      // SDA

#ifndef USE_TWI_DISPLAY
#define ENCODER1_PIN        39      // IO37 (aka CapVP)
#define ENCODER2_PIN        36      // IO38 (aka CapVN)
#else
#define ENCODER1_PIN        39      // IO37 (aka CapVP)
#define ENCODER2_PIN        36      // IO38 (aka CapVN)
#endif
#define ENCODER_BUTTON_PIN  32      // IO39 (aka SenseVN)

#define CAN_USE_SERIAL1     true
#define CAN_USE_SERIAL2     true
#define CAN_USE_SERIAL3     false

#define TX1_PIN             17      // Serial 1
#define RX1_PIN             16

#define TX2_PIN             19      // Serial 2 (occupied by SPI)
#define RX2_PIN             18

#define TX3_PIN             1       // Serial 0
#define RX3_PIN             3    

#define NEOPIXEL_PIN        13       
#define NUM_LEDS            5     // number of Neopixel LEDS
#define BRIGHTNESS          127
#define LED_TYPE            WS2812B
#define COLOR_ORDER         GRB
