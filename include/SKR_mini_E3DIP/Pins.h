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
 * Pins configuration file for SKR mini E3-DIP V1.1 board - NOT E3 1.2
 */
#pragma once

#define BOARD_INFO "SKR mini E3-DIP V1.1"
// SELECTOR (X)
#define STEP_HIGH_X digitalWrite(X_STEP_PIN, HIGH);
#define STEP_LOW_X digitalWrite(X_STEP_PIN, LOW);
#define X_STEP_PIN PC6
#define X_DIR_PIN PB15
#define X_ENABLE_PIN PC7
#define X_END_PIN PC1 // X-STOP
// REVOLVER (Y)
#define STEP_HIGH_Y digitalWrite(Y_STEP_PIN, HIGH);
#define STEP_LOW_Y digitalWrite(Y_STEP_PIN, LOW);
#define Y_STEP_PIN PB13
#define Y_DIR_PIN PB12
#define Y_ENABLE_PIN PB14
#define Y_END_PIN PC0 // Y-STOP
// Feeder (E)
// moved from Z to E because of the pins for 3rd Serial port,
// so don't get confused by the pin names
#define STEP_HIGH_Z digitalWrite(Z_STEP_PIN, HIGH);
#define STEP_LOW_Z digitalWrite(Z_STEP_PIN, LOW);
#define Z_STEP_PIN PB0
#define Z_DIR_PIN PC5
#define Z_ENABLE_PIN PB1
#define Z_END_PIN PC15 // Z-STOP
#define Z_END2_PIN PC2 // E0-STOP
#define Z_END_DUET_PIN Z_END2_PIN

// SPI for stepper drivers
#define ST_MISO_PIN PB4 // MISO3
#define ST_MOSI_PIN PB5 // MOSI3
#define ST_SCLK_PIN PB3 // SCK3
#define X_CS_PIN PC10   // doubles as XUART when used in serial mode
#define Y_CS_PIN PC11   // doubles as YUART when used in serial mode
#define Z_CS_PIN PC12   // doubles as ZUART when used in serial mode
#define E_CS_PIN PD2    // doubles as EUART when used in serial mode

#define BEEPER_PIN PA15 // EXP1.10

#define RELAY_PIN PC14 // PROBE (Relay for stepper motor switching)

#if !defined(SMUFF_V5)
#if defined(SMUFF_V6S)  // V6S uses linear stepper for lid; servo signals move to Z-Driver socket
#define SERVO_OPEN_DRAIN 0
#define SERVO1_PIN PC5  // Z-DIR (Wiper Servo)
#define SERVO2_PIN -1   // not used
#define SERVO3_PIN PD2  // SERVO (Cutter Servo)-- can use only one servo; pick either WIPER or CUTTER
#else
#define SERVO_OPEN_DRAIN 0
#define SERVO1_PIN PC2 // E0-STOP (Wiper Servo)
#define SERVO2_PIN PA1 // SERVO (Lid Servo)
#define SERVO3_PIN -1  // SERVO (Cutter Servo)-- can use only one servo; pick either WIPER or CUTTER
#endif
#else
#define SERVO_OPEN_DRAIN 0
#define SERVO1_PIN PB13 // Y STEP pin (Wiper Servo) used because of 5V tolerance
#define SERVO2_PIN PB12 // Y DIR pin (Lid Servo)
#define SERVO3_PIN PB14 // Y EN pin (Cutter Servo)
#endif

#define FAN_PIN PA8     // FAN0
#define HEATER0_PIN PC8 // HE0
#define HEATBED_PIN PC9 // BED

#if defined(USE_FASTLED_BACKLIGHT) || defined(USE_FASTLED_TOOLS)
#include "FastLED.h"
//_DEFPIN_ARM(PC7, 7, C);             // needed to compensate "Invalid pin specified"
// while compiling if NEOXPIXEL_PIN is in use
#endif

#if defined(USE_MINI12864_PANEL_V21)
#define NEOPIXEL_PIN        PB7
#else
#define NEOPIXEL_PIN        -1
#endif
#define NEOPIXEL_TOOL_PIN   PA1         // SERVOS (NeoPixel for tools)
//#define NEOPIXEL_TOOL_PIN   -1          // NeoPixel for tools
#define NUM_LEDS            3           // number of Neopixel LEDS
#if defined(USE_MINI12864_PANEL_V21)
#define BRIGHTNESS          200
#define LED_TYPE            WS2812B
#define COLOR_ORDER         RGB
#else
#define BRIGHTNESS          127
#define LED_TYPE            WS2812B
#define COLOR_ORDER         GRB
#endif
#define BRIGHTNESS_TOOL     127
#define LED_TYPE_TOOL       WS2812B
#define COLOR_ORDER_TOOL    GRB

#define SDCS_PIN -1 // use default

#define USB_CONNECT_PIN PC13
#define SD_DETECT_PIN PC4
#define USE_TERMINAL_MENUS 1

#if defined(USE_CREALITY_DISPLAY)
#if !defined(CREALITY_HW_SPI)
#define DSP_DATA_PIN PB7 // EXP1.3 = LCD_PINS_EN = ST9720 DAT
#define DSP_CS_PIN PB8   // EXP1.4 = LCD_PINS_RS = ST9720 CS
#define DSP_DC_PIN PB9   // EXP1.5 = LCD_PINS_D4 = ST9720 CLK
#define DSP_RESET_PIN -1

#define ENCODER1_PIN PA9       // EXP1.8
#define ENCODER2_PIN PA10      // EXP1.6
#define ENCODER_BUTTON_PIN PB6 // EXP1.9
#else
// SPECIAL CONFIGURATION, WORKS ONLY WITH CUSTOM MADE CABLE!
#define DSP_DATA_PIN -1 // USE MOSI ON SPI1 HEADER
#define DSP_CS_PIN PB9  // EXP1.5 = LCD_PINS_RS = ST9720 CS
#define DSP_DC_PIN -1   // USE SCK ON SPI1 HEADER
#define DSP_RESET_PIN -1

#define ENCODER1_PIN PA9       // EXP1.8
#define ENCODER2_PIN PA10      // EXP1.6
#define ENCODER_BUTTON_PIN PB8 // EXP1.4
#endif

#define DEBUG_PIN -1

#elif defined(USE_TWI_DISPLAY)

#define DSP_SCL PB6 // EXP1.9
#define DSP_SDA PB7 // EXP1.3

#define DSP_CS_PIN -1
#define DSP_DC_PIN -1
#define DSP_RESET_PIN -1

#define ENCODER1_PIN PA9       // EXP1.8
#define ENCODER2_PIN PA10      // EXP1.6
#define ENCODER_BUTTON_PIN PB8 // EXP1.4

#define DEBUG_PIN PB9 // EXP1.5

#elif defined(USE_MINI12864_PANEL_V21) || defined(USE_MINI12864_PANEL_V20)
#define DSP_CS_PIN          PB9     // DOGLCD_CS
#define DSP_DC_PIN          PB6     // DOGLCD_A0
#define DSP_RESET_PIN       PA13    // SWDIO --- IMPORTANT: This display needs a RESET signal!

#define ENCODER1_PIN        PA9
#define ENCODER2_PIN        PA10
#define ENCODER_BUTTON_PIN  PB8

#define DEBUG_PIN           -1
#else
// SPECIAL CONFIGURATION, WORKS ONLY WITH CUSTOM MADE CABLE!
#define DSP_DATA_PIN -1 // USE MOSI ON SPI1 HEADER
#define DSP_CS_PIN PB9  // EXP1.5 = LCD_PINS_RS = ST9720 CS
#define DSP_DC_PIN PB6  // EXP1.9
#define DSP_RESET_PIN -1

#define ENCODER1_PIN PA9       // EXP1.8
#define ENCODER2_PIN PA10      // EXP1.6
#define ENCODER_BUTTON_PIN PB8 // EXP1.4

#define DEBUG_PIN -1

#endif

#define DUET_SIG_FED_PIN     PB2    // Z-Axis DIR
#define DUET_SIG_SEL_PIN     PC12   // Z-Axis MS3
#define DEBUG_OFF_PIN       -1      // not needed on TWI display

#define X_SERIAL_TX_PIN PC10 // XUART - UART4 TX
#define Y_SERIAL_TX_PIN PC11 // YUART - UART4 RX
#define Z_SERIAL_TX_PIN PD2  // EUART - UART5 RX
//#define E_SERIAL_TX_PIN PC12 // ZUART - UART5 TX (not used anyways)

#define MS3_X PC10 // the MS3 pin (if needed for the stepper driver)
#define MS3_Y PC11 // is connected to the MCU and thus
#if !defined(SMUFF_V6S)
#define MS3_Z PD2  // must be controlled via software on this controller board
#else
#define MS3_Z -1    // used for Cutter servo
#endif

#define STALL_X_PIN PA13 // SWDIO
#define STALL_Y_PIN -1   //
#define STALL_Z_PIN PA14 // SWCLK

// SERIAL1 - Cannot be used for serial comm.
#define CAN_USE_SERIAL1 false // used for encoder on EXP1

#define TX1_PIN PA9  // EXP1.8 - ENCODER1_PIN
#define RX1_PIN PA10 // EXP1.6 - ENCODER2_PIN

// SERIAL2 - Can be used for serial comm.
#define CAN_USE_SERIAL2 true // TFT header

#define TX2_PIN PA2 // TX on TFT header
#define RX2_PIN PA3 // RX on TFT header

// SERIAL3 - Cannot be used for serial comm. on E3 but can on E3-DIP
#define CAN_USE_SERIAL3 true // if no Z-Axis driver is being used

#define TX3_PIN PB10 // Z-Axis STEP
#define RX3_PIN PB11 // Z-Axis ENABLE

//#define MOTOR_IN1_PIN       PC2     // E0-STOP (experimental)
//#define MOTOR_IN2_PIN       PC0     // Y-STOP  (experimental)