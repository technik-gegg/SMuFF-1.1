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
 * Pins configuration file for SKR V1.3 / 1.4 board
 */
#pragma once

#define BOARD_INFO          "SKR V1.3"
// SELECTOR (X)
#define STEP_HIGH_X         digitalWrite(X_STEP_PIN, HIGH);
#define STEP_LOW_X          digitalWrite(X_STEP_PIN, LOW);
#define X_STEP_PIN          P2_02
#define X_DIR_PIN           P2_06
#define X_ENABLE_PIN        P2_01
#define X_END_PIN           P1_29
// REVOLVER (Y)
#define STEP_HIGH_Y         digitalWrite(Y_STEP_PIN, HIGH);
#define STEP_LOW_Y          digitalWrite(Y_STEP_PIN, LOW);
#define Y_STEP_PIN          P0_19
#define Y_DIR_PIN           P0_20
#define Y_ENABLE_PIN        P2_08
#define Y_END_PIN           P1_27
// FEEDER (E)
// moved from Z to E because of the pins for 2nd Serial port
#define STEP_HIGH_Z         digitalWrite(Z_STEP_PIN, HIGH);
#define STEP_LOW_Z          digitalWrite(Z_STEP_PIN, LOW);
#define Z_STEP_PIN          P0_22
#define Z_DIR_PIN           P2_11
#define Z_ENABLE_PIN        P0_21
#define Z_END_PIN           P1_25
#define Z_END2_PIN          P1_24
#define Z_END_DUET_PIN      -1

#define BEEPER_PIN          P1_30

#define SERVO1_PIN          P2_00       // Servo-Pin
#define SERVO2_PIN          P1_26       // Endstop Y+
#define FAN_PIN             P2_03
#define HEATER0_PIN         P2_07
#define HEATBED_PIN         P0_23_A0

#define NEOPIXEL_PIN        P1_21
#define NEOPIXEL_TOOL_PIN   -1          // for tools

#define DSP_CS_PIN          P1_21       // These pins are only valid if a SPI display is being used
#define DSP_DC_PIN          P1_22
#define DSP_RESET_PIN       P1_20

#define DSP_SCL             P0_01       // By default we run the SMuFF controller display on TWI (I2C)
#define DSP_SDA             P0_00

#ifndef USE_TWI_DISPLAY
#define ENCODER1_PIN        P3_26
#define ENCODER2_PIN        P3_25
#else
#define ENCODER1_PIN        P1_18
#define ENCODER2_PIN        P1_20
#endif
#define ENCODER_BUTTON_PIN  P1_30


#define TX3_PIN             P1_01     // on SKR usually used for stepper driver serial connection
#define RX3_PIN             P1_10

#define TX2_PIN             P1_00     // on SKR usually used for stepper driver serial connection
#define RX2_PIN             P1_09     // on SKR Mini already used for SD-Card DATA2

