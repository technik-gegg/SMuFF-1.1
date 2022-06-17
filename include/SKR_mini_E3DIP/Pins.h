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
 * Pins configuration file for SKR mini E3-DIP V1.1 board - NOT E3 1.2
 */
#pragma once

#define BOARD_INFO "SKR mini E3-DIP V1.1"
// SELECTOR (X)
#define STEP_HIGH_X     digitalWrite(X_STEP_PIN, HIGH);
#define STEP_LOW_X      digitalWrite(X_STEP_PIN, LOW);
#if defined(__STM32F1XX)
#define X_STEP_PIN_NAME PC_6
#endif
#define X_STEP_PIN      PC6
#define X_DIR_PIN       PB15
#define X_ENABLE_PIN    PC7
#define X_END_PIN       PC1 // X-STOP
// REVOLVER (Y)
#define STEP_HIGH_Y     digitalWrite(Y_STEP_PIN, HIGH);
#define STEP_LOW_Y      digitalWrite(Y_STEP_PIN, LOW);
#if defined(__STM32F1XX)
#define Y_STEP_PIN_NAME PB_13
#endif
#define Y_STEP_PIN      PB13
#define Y_DIR_PIN       PB12
#define Y_ENABLE_PIN    PB14
#if defined(USE_DDE)
#define Y_END_PIN       PC2 // E0-STOP
#else
#define Y_END_PIN       PC0 // Y-STOP
#endif
// Feeder (E)
// moved from Z to E because of the pins for 3rd Serial port,
// so don't get confused by the pin names
#define STEP_HIGH_Z     digitalWrite(Z_STEP_PIN, HIGH);
#define STEP_LOW_Z      digitalWrite(Z_STEP_PIN, LOW);
#if defined(__STM32F1XX)
#define Z_STEP_PIN_NAME PB_0
#endif
#define Z_STEP_PIN      PB0
#define Z_DIR_PIN       PC5
#define Z_ENABLE_PIN    PB1
#define Z_END_PIN       PC15 // Z-STOP
#define Z_END2_PIN      PC2  // E0-STOP
#define Z_END_DUET_PIN  Z_END2_PIN

// SPI for stepper drivers
#define ST_MISO_PIN     PB4     // MISO3
#define ST_MOSI_PIN     PB5     // MOSI3
#define ST_SCLK_PIN     PB3     // SCK3
#define X_CS_PIN        PC10   // doubles as XUART when used in serial mode
#define Y_CS_PIN        PC11   // doubles as YUART when used in serial mode
#define Z_CS_PIN        PC12   // doubles as ZUART when used in serial mode
#define E_CS_PIN        PD2    // doubles as EUART when used in serial mode


#if defined(RELAY_ON_PROBE)
#define RELAY_PIN PC14   // PROBE (Relay for stepper motor switching)
//#define RELAY_PIN PC0    // Y-STOP (alternative for Relay; if nothing else is connected to)
#else
#define RELAY_PIN PC12   // Z-MS3
#endif


#if !defined(SMUFF_V5)
    #if defined(SMUFF_V6S)  // V6S uses linear stepper for lid; servo signals move to Z-Driver socket
        #define SERVO_OPEN_DRAIN    0
        #define SERVO1_PIN          PC5  // Z-DIR (Wiper Servo)
        #define SERVO2_PIN          0    // not used because of the linear stepper
        #define SERVO3_PIN          PD2  // Z-MS3 (Cutter Servo)
    #else
        #define SERVO_OPEN_DRAIN    0
        #define SERVO1_PIN          PC2 // E0-STOP (Wiper Servo)
        #define SERVO2_PIN          PA1 // SERVO (Lid Servo)
        #define SERVO3_PIN          0   // SERVO (Cutter Servo)-- can use only one servo; pick either WIPER or CUTTER
    #endif
    #else
    #if !defined(USE_DDE)
        #define SERVO_OPEN_DRAIN    0
        #define SERVO1_PIN          PB13 // Y STEP pin (Wiper Servo) used because of 5V tolerance
        #define SERVO2_PIN          PB12 // Y DIR pin (Lid Servo)
        #define SERVO3_PIN          PB14 // Y EN pin (Cutter Servo)
    #else
        // relocate servo pins to Z-Axis driver socket because Y-Axis is being used for Shared Stepper.
        // Please notice: If USE_DDE is set, Serial3 can't be used anymore for serial communication
        #define SERVO1_PIN          PB10 // Z-STEP (Wiper Servo)
        #define SERVO2_PIN          PB2  // Z-DIR  (Lid-Servo)
        #define SERVO3_PIN          PB11 // Z-EN   (Cutter Servo)
    #endif
#endif

#define FAN_PIN PA8         // FAN0

#define NEOPIXEL_TOOL_PIN   PA1         // SERVOS (NeoPixel for tools)
#define BRIGHTNESS_TOOL     127
#if !defined(USES_ADAFRUIT_NPX)
#define LED_TYPE_TOOL       WS2812B
#define COLOR_ORDER_TOOL    GRB
#else
#define COLOR_ORDER_TOOL    NEO_GRB + NEO_KHZ800
#endif

#define SDCS_PIN            0           // use default

#if defined(RELAY_ON_PROBE)
    #if defined(__STM32F1XX)
    #define DEBUG_PIN_NAME      PA_0
    #endif
    #define DEBUG_PIN           PA0         // TH0 //-1
#else
    #if defined(__STM32F1XX)
    #define DEBUG_PIN_NAME      PC_14
    #endif
    #define DEBUG_PIN           PC14        // PROBE
#endif

#define USB_CONNECT_PIN     PC13
#define SD_DETECT_PIN       PC4

#if !defined(USE_SERIAL_DISPLAY)
    #define USE_TERMINAL_MENUS  1
#endif

#if defined(USE_SPLITTER_ENDSTOPS)
// only describing pins, since the 2nd hardware I2C is being used and pins are pre-defined
#define SPLITTER_SCL        PB10        // Z-Axis STEP
#define SPLITTER_SDA        PB11        // Z-Axis ENABLE
#endif

#if defined(USE_MULTISERVO)
    #if !defined(USE_DDE)
        // software I2C is being used on those pins
        #define ADASERVO_SCL        PB12        // Y-Axis DIR
        #define ADASERVO_SDA        PC11        // Y-Axis MS3
    #else
        #undef RELAY_PIN                        // Relay on MS3 pin can't be used in this configuration
        #if !defined(USE_MULTISERVO_RELAY)
            #define RELAY_PIN       PA0         // use TH0 instead
        #else
            #define RELAY_PIN       0           // or use Multiservo output 5 
        #endif
        // software I2C is being used on those pins
        #define ADASERVO_SCL        PB2         // Z-Axis DIR
        #define ADASERVO_SDA        PC12        // Z-Axis MS3
    #endif
#endif

// moved to thermistor pins in V2.41
#define DUET_SIG_FED_PIN    PC3         // THB (thermistor output pins will work fine up to 100Hz - see schematic)
// #define DUET_SIG_SEL_PIN    PA0         // TH0 (not used on Duet but for relay in Multiservo mode)

#define DEBUG_OFF_PIN       0

#define STALL_X_PIN         PA13        // SWDIO (cannot be used with FYSETC Minipanel 12864)
#define STALL_Y_PIN         0           //
#define STALL_Z_PIN         PA14        // SWCLK

#define MS3_X               PC10        // the MS3 pin (if needed for the stepper driver)
#define MS3_Y               PC11        // is connected to the MCU and thus
#if !defined(SMUFF_V6S)
#define MS3_Z               PD2         // must be controlled via software on this controller board
#else
#define MS3_Z               0           // used for Cutter servo
#endif

// the following pins cannot be used directly from the according headers/terminals, since those are signals
// used to drive the Mosfets for heaters. If you need one of those signals, you have to wire it up on the 
// according driver input pin of U5 (see schematic).
#define SPARE1              PC8     // HE0
#define SPARE2              PC9     // BED

// -----------------------------------------------------
// Serial Ports section
// -----------------------------------------------------
#define SW_SERIAL_TX_PIN    0
#define SW_SERIAL_RX_PIN    0

#define X_SERIAL_TX_PIN     PC10    // XUART - UART4 TX
#define Y_SERIAL_TX_PIN     PC11    // YUART - UART4 RX
#define Z_SERIAL_TX_PIN     PD2     // EUART - UART5 RX
//#define E_SERIAL_TX_PIN     PC12    // ZUART - UART5 TX (not used anyways)

// SERIAL1 - Cannot be used for serial comm.
#if defined(USE_SERIAL_DISPLAY)
#define CAN_USE_SERIAL1     true
#else
#define CAN_USE_SERIAL1     false
#endif
#define TX1_PIN             PA9     // EXP1.8 - ENCODER1_PIN
#define RX1_PIN             PA10    // EXP1.6 - ENCODER2_PIN

// SERIAL2 - Can be used for serial comm.
#define CAN_USE_SERIAL2     true // TFT header
#define TX2_PIN             PA2 // TX on TFT header
#define RX2_PIN             PA3 // RX on TFT header

// SERIAL3 - Cannot be used for serial comm.
#if !defined(USE_SPLITTER_ENDSTOPS)
    #if defined(USE_DDE)
        #if defined(USE_MULTISERVO)
            #define CAN_USE_SERIAL3 true   // Serial3 can be used when using Adafruit Multiservo board
        #else
            #define CAN_USE_SERIAL3 false   // Serial3 cannot be used when using DDE
        #endif
    #else
    #define CAN_USE_SERIAL3 true    // if no Z-Axis driver is being used
    #endif
#else
    #define CAN_USE_SERIAL3 false   // Serial3 cannot be used when Splitter with endstops is being used
#endif
#define TX3_PIN             PB10    // Z-Axis STEP
#define RX3_PIN             PB11    // Z-Axis ENABLE

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
