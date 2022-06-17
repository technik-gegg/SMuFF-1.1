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
 * Pins configuration file for SKR mini V1.1 board (_not_ E3 mini)
 */
#pragma once

#define BOARD_INFO          "SKR mini V1.1"
// SELECTOR (X)
#define STEP_HIGH_X         digitalWrite(X_STEP_PIN, HIGH);
#define STEP_LOW_X          digitalWrite(X_STEP_PIN, LOW);
#if defined(__STM32F1XX)
#define X_STEP_PIN_NAME     PC_6
#endif
#define X_STEP_PIN          PC6
#define X_DIR_PIN           PC7
#define X_ENABLE_PIN        PB15
#define X_END_PIN           PC2     // Endstop X-
// REVOLVER (Y)
#define STEP_HIGH_Y         digitalWrite(Y_STEP_PIN, HIGH);
#define STEP_LOW_Y          digitalWrite(Y_STEP_PIN, LOW);
#if defined(__STM32F1XX)
#define Y_STEP_PIN_NAME     PB_13
#endif
#define Y_STEP_PIN          PB13
#define Y_DIR_PIN           PB14
#define Y_ENABLE_PIN        PB12
#define Y_END_PIN           PC1     // Endstop Y-
// FEEDER (E)
// moved from Z to E because of the pins for 3rd Serial port,
// so don't get confused by the pin names
#define STEP_HIGH_Z         digitalWrite(Z_STEP_PIN, HIGH);
#define STEP_LOW_Z          digitalWrite(Z_STEP_PIN, LOW);
#if defined(__STM32F1XX)
#define Z_STEP_PIN_NAME     PC_5
#endif
#define Z_STEP_PIN          PC5
#define Z_DIR_PIN           PB0
#define Z_ENABLE_PIN        PC4
#define Z_END_PIN           PC0     // Endstop Z-
#define Z_END2_PIN          PA2     // Endstop X+
#define Z_END_DUET_PIN      Z_END2_PIN

#define RELAY_PIN           PC1     // Endstop Y- (Relay for stepper motor switching)

#if !defined(SMUFF_V5)              // Configuration when using Revolver (a.k.a. V4)
#define SERVO_OPEN_DRAIN    0       // change this to 1 if you have pullups attached to your servo signal pins
#define SERVO1_PIN          PA1     // Endstop Y+ (Wiper Servo)
#define SERVO2_PIN          PC3     // Endstop Z+ (Lid Servo)
#define SERVO3_PIN          0       // Endstop Z+ (Cutter Servo) -- can use only one servo; pick either WIPER or CUTTER
#else
#define SERVO_OPEN_DRAIN    0
#define SERVO1_PIN          PB13    // Y STEP pin   (Wiper Servo)
#define SERVO2_PIN          PB14    // Y DIR pin    (Lid Servo)
#define SERVO3_PIN          PB12    // Y EN pin     (Cutter Servo)
#endif

#define FAN_PIN             PC8

#define NEOPIXEL_TOOL_PIN   PB8     // for tools (EXP2.6)
#define BRIGHTNESS_TOOL     127
#if !defined(USES_ADAFRUIT_NPX)
#define LED_TYPE_TOOL       WS2812B
#define COLOR_ORDER_TOOL    GRB
#else
#define COLOR_ORDER_TOOL    NEO_GRB + NEO_KHZ800
#endif

#define SDCS_PIN            0       // use default
#define DEBUG_PIN           PA1     // Endstop Y+

#define USB_CONNECT_PIN     0       // not avail
#define SD_DETECT_PIN       PA3
#if !defined(USE_SERIAL_DISPLAY)
#define USE_TERMINAL_MENUS  1
#endif

#if defined(USE_SPLITTER_ENDSTOPS)
// using the same pins as for TWI displays (HW-I2C)
#define SPLITTER_SCL        PB6    // EXP1.8
#define SPLITTER_SDA        PB7    // EXP1.6
#endif

#if defined(USE_MULTISERVO)
// this option is not available on this board
#define ADASERVO_SCL        0
#define ADASERVO_SDA        0
#endif

#define DUET_SIG_FED_PIN    PB1      // THB (thermistor output pins will work fine up to 100Hz - see schematic)
#define DUET_SIG_SEL_PIN    PA0      // TH0

#define DEBUG_OFF_PIN       0

#undef HAS_TMC_SUPPORT              // not supported by the board design but can be wired manually if needed
                                    // in order not to waste serial ports, software serial is recommended in such case
#define STALL_X_PIN         0       // not used since stepper driver DIAG pins are not wired on this board
#define STALL_Y_PIN         0
#define STALL_Z_PIN         0

// the following pins cannot be used directly from the according headers/terminals, since those are signals
// used to drive the Mosfets for heaters/fan. If you need one of those signals, you have to wire it up on the 
// according driver input pin of U8 (see schematic).
#define SPARE1              PA8     // HE0
#define SPARE2              PC9     // BED


// -----------------------------------------------------
// Serial Ports section
// -----------------------------------------------------
#define SW_SERIAL_TX_PIN    0
#define SW_SERIAL_RX_PIN    0

// SERIAL1 - Can be used for serial comm.
#define CAN_USE_SERIAL1     true
#define TX1_PIN             PA9     // TFT header TX
#define RX1_PIN             PA10    // TFT header RX

// SERIAL2 - Cannot be used for serial comm.
#define CAN_USE_SERIAL2     false
#define TX2_PIN             PA2     // on SKR Mini already used for X+ endstop (but might be reconfigured)
#define RX2_PIN             PA3     // on SKR Mini already used for SD-Card DATA2

// SERIAL3 - Can be used for serial comm.
#define CAN_USE_SERIAL3     true
#define TX3_PIN             PB10    // on SKR Mini usually used for Z-Axis STEP
#define RX3_PIN             PB11    // on SKR Mini usually used for Z-Axis DIR

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
