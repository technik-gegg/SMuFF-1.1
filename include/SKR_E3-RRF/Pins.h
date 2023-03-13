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
 * Pins configuration file for SKR E3 RRF V1.1 board
 */
#pragma once

#define BOARD_INFO          "SKR E3 RRF V1.1"
// SELECTOR (X)
#define STEP_HIGH_X         digitalWrite(X_STEP_PIN, HIGH);
#define STEP_LOW_X          digitalWrite(X_STEP_PIN, LOW);
#if defined(__STM32F4XX)
#define X_STEP_PIN_NAME     PD_5
#endif
#define X_STEP_PIN          PD5
#define X_DIR_PIN           PD4
#define X_ENABLE_PIN        PD7
#define X_END_PIN           PC0     // X-STOP
// REVOLVER (Y)
#define STEP_HIGH_Y         digitalWrite(Y_STEP_PIN, HIGH);
#define STEP_LOW_Y          digitalWrite(Y_STEP_PIN, LOW);
#if defined(__STM32F4XX)
#define Y_STEP_PIN_NAME     PD_0
#endif
#define Y_STEP_PIN          PD0
#define Y_DIR_PIN           PA15
#define Y_ENABLE_PIN        PD3
#define Y_END_PIN           PC1     // Y-STOP
// FEEDER (Z)
#define STEP_HIGH_Z         digitalWrite(Z_STEP_PIN, HIGH);
#define STEP_LOW_Z          digitalWrite(Z_STEP_PIN, LOW);
#if defined(__STM32F4XX)
#define Z_STEP_PIN_NAME     PC_6
#endif
#define Z_STEP_PIN          PC6
#define Z_DIR_PIN           PC7
#define Z_ENABLE_PIN        PD14
#define Z_END_PIN           PC2     // Z-STOP
#define Z_END2_PIN          PC3     // E0-STOP
#define Z_END_DUET_PIN      Z_END2_PIN

// (E) - Not used yet, just in case
#define STEP_HIGH_E         digitalWrite(E_STEP_PIN, HIGH);
#define STEP_LOW_E          digitalWrite(E_STEP_PIN, LOW);
#define E_STEP_PIN          PD12
#define E_DIR_PIN           PD13
#define E_ENABLE_PIN        PD10
#define E_END_PIN           PC3     // E0-STOP

#define RELAY_PIN           PE1     // PS-ON (Relay for stepper motor switching)

#define SERVO_OPEN_DRAIN    0
#define SERVO1_PIN          PC5     // Z-PROBE.1  (Wiper Servo)
#define SERVO2_PIN          PB0     // Z-PROBE.3  (Lid Servo)
#define SERVO3_PIN          PE0     // PT-DET     (Cutter Servo)

#define FAN_PIN             PB5     // FAN0

#define NEOPIXEL_TOOL_PIN   PB7     // for tools (NEOPIXEL)
#define BRIGHTNESS_TOOL     127
#define COLOR_ORDER_TOOL    NEO_GRB + NEO_KHZ800

#define SDCS_PIN            0       // use default
#define DEBUG_PIN           0       // PC3 - TB0 (using this header will lead to a sine wave on the output if freq. succseeds 100Hz  - see schematic)

#define USB_CONNECT_PIN     0       // ?
#define SD_DETECT_PIN       PC4

#if defined(USE_SPLITTER_ENDSTOPS)
// using the same pins as for TWI displays (SW-I2C)
#define SPLITTER_SCL        PE9     // EXP1.9
#define SPLITTER_SDA        PE11    // EXP1.3
#endif

#if defined(USE_MULTISERVO)
// software I2C is being used on those pins
#define ADASERVO_SCL        PC5     // Z-PROBE.1
#define ADASERVO_SDA        PB0     // Z-PROBE.3
#endif

#define DUET_SIG_FED_PIN    PA1      // THB (thermistor output pins will work fine up to 100Hz - see schematic)
#define DUET_SIG_SEL_PIN    PA0      // TH0

#define DEBUG_OFF_PIN       0

#define STALL_X_PIN         0
#define STALL_Y_PIN         0
#define STALL_Z_PIN         0

// the following pins cannot be used directly from the according headers/terminals, since those are signals
// used to drive the Mosfets for heaters/fan. If you need one of those signals, you have to wire it up on the 
// according driver input pin of U8 (see schematic).
#define SPARE1              PB3     // HE0
#define SPARE2              PB4     // BED
#define SPARE3              PB6     // FAN1


// -----------------------------------------------------
// Serial Ports section
// -----------------------------------------------------
#define SW_SERIAL_TX_PIN    0
#define SW_SERIAL_RX_PIN    0

#define X_SERIAL_TX_PIN     PD6     // XUART
#define Y_SERIAL_TX_PIN     PD1     // YUART
#define Z_SERIAL_TX_PIN     PD11    // EUART
//#define E_SERIAL_TX_PIN     PD15    // ZUART (not used anyways)

/*
    NEEDS TO BE CHECKED!
*/

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


// -----------------------------------------------------
// Display section
// -----------------------------------------------------
#include "../Display/Displays.h"
