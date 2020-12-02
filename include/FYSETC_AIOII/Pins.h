/**
 * SMuFF Firmware
 * Copyright (C) 2019 Technik Gegg
 *               2020 sL1pKn07
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
 * Pins configuration file for FYSETC AIO II V3.2 board
 */
#pragma once

#define BOARD_INFO          "FYSETC AIO II V3.2"
// SELECTOR (X)
#define STEP_HIGH_X         digitalWrite(X_STEP_PIN, HIGH);
#define STEP_LOW_X          digitalWrite(X_STEP_PIN, LOW);
#define X_STEP_PIN          PB8
#define X_DIR_PIN           PB9
#define X_ENABLE_PIN        PA8
#define X_END_PIN           PA1
// REVOLVER (Y)
#define STEP_HIGH_Y         digitalWrite(Y_STEP_PIN, HIGH);
#define STEP_LOW_Y          digitalWrite(Y_STEP_PIN, LOW);
#define Y_STEP_PIN          PB2
#define Y_DIR_PIN           PB3
#define Y_ENABLE_PIN        PB1
#define Y_END_PIN           PA0
// FEEDER (Z)
#define STEP_HIGH_Z         digitalWrite(Z_STEP_PIN, HIGH);
#define STEP_LOW_Z          digitalWrite(Z_STEP_PIN, LOW);
#define Z_STEP_PIN          PC0
#define Z_DIR_PIN           PC1
#define Z_ENABLE_PIN        PC2
#define Z_END_PIN           PB14
#define Z_END2_PIN          -1
#define Z_END_DUET_PIN      -1       // for testing only

#define BEEPER_PIN          PC9

#define SERVO1_PIN          -1
#define SERVO2_PIN          PB15     // FIL-DET
#define FAN_PIN             PC8      // FAN1
#define HEATER0_PIN         PC7      // BED
#define HEATBED_PIN         PC6      // END

#define NEOPIXEL_PIN        -1
#define DEBUG_PIN           -1
#define RELAY_PIN           -1      // Relais for stepper motor switching

#define SDCS_PIN            PA4

#define USB_CONNECT_PIN     -1
#define SD_DETECT_PIN       -1

#define DSP_CS_PIN          PB5     // DOGLCD-CS
#define DSP_DC_PIN          PA15    // DOGLCD-A0
#define DSP_RESET_PIN       PB4     // DOGLCD-RST

#define ENCODER1_PIN        PC10
#define ENCODER2_PIN        PC11
#define ENCODER_BUTTON_PIN  PC12

#define DEBUG_OFF_PIN       -1

#define STALL_X_PIN         -1
#define STALL_Y_PIN         -1
#define STALL_Z_PIN         -1

#define CAN_USE_SERIAL1     true
#define CAN_USE_SERIAL2     false
#define CAN_USE_SERIAL3     true

#define TX3_PIN             PB10     // P4-7
#define RX3_PIN             PB11     // P4-5

#define TX2_PIN             PA2      // P4-8
#define RX2_PIN             PA3      // P4-6

#define RGB_LED_R_PIN       PB0
#define RGB_LED_G_PIN       PB6
#define RGB_LED_B_PIN       PB7
