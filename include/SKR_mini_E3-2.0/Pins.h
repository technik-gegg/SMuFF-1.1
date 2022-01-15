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
 * Pins configuration file for SKR mini E3 V2.0 board - NOT E3 DIP!
 */
#pragma once

#define BOARD_INFO          "SKR mini E3 V2.0"
// SELECTOR (X)
#define STEP_HIGH_X         digitalWrite(X_STEP_PIN, HIGH);
#define STEP_LOW_X          digitalWrite(X_STEP_PIN, LOW);
#define X_STEP_PIN          PB13
#define X_DIR_PIN           PB12
#define X_ENABLE_PIN        PB14
#define X_END_PIN           PC0     // X-STOP
// REVOLVER (Y)
#define STEP_HIGH_Y         digitalWrite(Y_STEP_PIN, HIGH);
#define STEP_LOW_Y          digitalWrite(Y_STEP_PIN, LOW);
#if defined(SWAP_Y_STEPPER) // flag swaps driver Y and E
#define Y_STEP_PIN          PB3
#define Y_DIR_PIN           PB4
#define Y_ENABLE_PIN        PD2
#else
#define Y_STEP_PIN          PB10
#define Y_DIR_PIN           PB2
#define Y_ENABLE_PIN        PB11
#endif
#define Y_END_PIN           PC1     // Y-STOP
// FEEDER (Z)
#define STEP_HIGH_Z         digitalWrite(Z_STEP_PIN, HIGH);
#define STEP_LOW_Z          digitalWrite(Z_STEP_PIN, LOW);
#define Z_STEP_PIN          PB0
#define Z_DIR_PIN           PC5
#define Z_ENABLE_PIN        PB1
#define Z_END_PIN           PC2     // Z-STOP
#define Z_END2_PIN          PC15    // E0-STOP
#define Z_END_DUET_PIN      Z_END2_PIN

// (E) - Not used yet, just in case
#define STEP_HIGH_E         digitalWrite(E_STEP_PIN, HIGH);
#define STEP_LOW_E          digitalWrite(E_STEP_PIN, LOW);
#if defined(SWAP_Y_STEPPER) // flag swaps driver Y and E
#define E_STEP_PIN          PB10
#define E_DIR_PIN           PB2
#define E_ENABLE_PIN        PB11
#else
#define E_STEP_PIN          PB3
#define E_DIR_PIN           PB4
#define E_ENABLE_PIN        PD2
#endif
#define E_END_PIN           PC15    // E0-STOP

#define RELAY_PIN           PC13    // PS-ON (Relay for stepper motor switching)

#define SERVO_OPEN_DRAIN    0
#define SERVO1_PIN          PC14    // Z-PROBE.1  (Wiper Servo)
#define SERVO2_PIN          PA1     // Z-PROBE.3  (Lid Servo)
#define SERVO3_PIN          PC12    // PT-DET     (Cutter Servo)

#define FAN_PIN             PC6     // FAN0

#define NEOPIXEL_TOOL_PIN   PA8     // for tools (NEOPIXEL)
//#define NEOPIXEL_TOOL_PIN   PA9     // alternative for tools (EXT1.8); Important: _only_ usable with TWI / LEONERD display
//#define NEOPIXEL_TOOL_PIN   PB15    // alternative for tools (EXT1.3); Important: not! usable with TWI / LEONERD display
#define BRIGHTNESS_TOOL     127
#define LED_TYPE_TOOL       WS2812B
#define COLOR_ORDER_TOOL    GRB

#define SDCS_PIN            -1      // use default
#define DEBUG_PIN           -1      // PC3 - TB0 (using this header will lead to a sine wave on the output if freq. succseeds 100Hz  - see schematic)

#define USB_CONNECT_PIN     PA14    // SWCLK
#define SD_DETECT_PIN       PC4

#if defined(USE_SPLITTER_ENDSTOPS)
// using the same pins as for TWI displays (SW-I2C)
#define SPLITTER_SCL        PA15    // EXP1.9
#define SPLITTER_SDA        PB15    // EXP1.3
#endif

#define DUET_SIG_FED_PIN    PC3      // THB (thermistor output pins will work fine up to 100Hz - see schematic)
#define DUET_SIG_SEL_PIN    PA0      // TH0

#define DEBUG_OFF_PIN       -1

#define STALL_X_PIN         PA13    // SWDIO (cannot be used with FYSETC Minipanel 12864)
#define STALL_Y_PIN         -1
#define STALL_Z_PIN         -1

// the following pins cannot be used directly from the according headers/terminals, since those are signals
// used to drive the Mosfets for heaters/fan. If you need one of those signals, you have to wire it up on the 
// according driver input pin of U8 (see schematic).
#define SPARE1              PC8     // HE0
#define SPARE2              PC9     // BED
#define SPARE3              PC7     // FAN1


// -----------------------------------------------------
// Serial Ports section
// -----------------------------------------------------
#define SW_SERIAL_TX_PIN    -1
#define SW_SERIAL_RX_PIN    -1

#define TMC_HW_SERIAL       1
#define TMC_SERIAL          Serial4

// SERIAL1 - Cannot be used for serial comm.
#define CAN_USE_SERIAL1     false
#define TX1_PIN             PA9     // EXP1.8 - ENCODER1_PIN
#define RX1_PIN             PA10    // EXP1.6 - ENCODER2_PIN

// SERIAL2 - Can be used for serial comm.
#define CAN_USE_SERIAL2     true
#define TX2_PIN             PA2     // TX on TFT header
#define RX2_PIN             PA3     // RX on TFT header

// SERIAL3 - Cannot be used for serial comm.
#define CAN_USE_SERIAL3     false
#define TX3_PIN             PB10    // Y-Axis STEP
#define RX3_PIN             PB11    // Y-Axis ENABLE

#if !defined(__LIBMAPLE__)
#define SPI3_SCLK           PB3
#define SPI3_MISO           PB4
#define SPI3_MOSI           PB5
#define SPI3_CS             PA15
#endif

// -----------------------------------------------------
// Display section
// -----------------------------------------------------
#include "../Display/Displays.h"
