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

#ifndef _SMUFF_CONFIG_H
#define _SMUFF_CONFIG_H

#define VERSION_STRING    "V1.41"
#define VERSION_MAJOR     1
#define VERSION_MINOR     2  
#define PMMU_VERSION      106               // Version number for Prusa MMU2 Emulation mode
#define PMMU_BUILD        372               // Build number for Prusa MMU2 Emulation mode
#define VERSION_DATE      "2019-09-12"
#define CONFIG_FILE       "SMUFF.CFG"
#define DATASTORE_FILE    "EEPROM.DAT"

#define SELECTOR          0
#define REVOLVER          1
#define FEEDER            2

#define NUM_STEPPERS      3

#define MIN_TOOLS         2
#define MAX_TOOLS         9

#define BEEPER_FREQUENCY    1760
#define BEEPER_DURATION     90
#define BEEPER_UFREQUENCY   440
#define BEEPER_UDURATION    90

#define ENCODER_DELAY       1
#define ENCODER_DELAY_MENU  1
#define ENCODER_DELAY_OFS   1

#define DSP_CONTRAST        200
#define MIN_CONTRAST        60
#define MAX_CONTRAST        250

#define I2C_SLAVE_ADDRESS   0x88

#define X_STEPS_PER_MM      800
#define Z_STEPS_PER_MM      136

#ifdef __BRD_I3_MINI
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

#define BEEPER_PIN          37

#define SERVO1_PIN          44
#define SERVO2_PIN          14
#define SD_SS_PIN           53
#define FAN_PIN             12
#define HEATER0_PIN         4

#define ENCODER1_PIN        2
#define ENCODER2_PIN        3
#define ENCODER_BUTTON_PIN  5

#define DSP_CLOCK_PIN       52
#define DSP_DATA_PIN        51
#define DSP_CS_PIN          41
#define DSP_DC_PIN          40
#define DSP_RESET_PIN       27

#define TX2_PIN             16
#define RX2_PIN             17

#define TX3_PIN             67
#define RX3_PIN             44
#endif

#ifdef __BRD_SKR_MINI
// SELECTOR
#define STEP_HIGH_X         digitalWrite(X_STEP_PIN, HIGH);
#define STEP_LOW_X          digitalWrite(X_STEP_PIN, LOW);
#define X_STEP_PIN          PC6
#define X_DIR_PIN           PC7
#define X_ENABLE_PIN        PB15
#define X_END_PIN           PC2
// REVOLVER
#define STEP_HIGH_Y         digitalWrite(Y_STEP_PIN, HIGH);
#define STEP_LOW_Y          digitalWrite(Y_STEP_PIN, LOW);
#define Y_STEP_PIN          PB13
#define Y_DIR_PIN           PB14
#define Y_ENABLE_PIN        PB12
#define Y_END_PIN           PC1
// FEEDER
#define STEP_HIGH_Z         digitalWrite(Z_STEP_PIN, HIGH);
#define STEP_LOW_Z          digitalWrite(Z_STEP_PIN, LOW);
#define Z_STEP_PIN          PB10
#define Z_DIR_PIN           PB11
#define Z_ENABLE_PIN        PB2
#define Z_END_PIN           PC0

#define BEEPER_PIN          PC10

#define SERVO1_PIN          PC9  // Bed heater
#define SERVO2_PIN          -1   // TBD
#define SD_SS_PIN           PA4
#define FAN_PIN             PC8
#define HEATER0_PIN         PA8

#define ENCODER1_PIN        PD2
#define ENCODER2_PIN        PB8
#define ENCODER_BUTTON_PIN  PC11

#define DSP_CLOCK_PIN       PB3
#define DSP_DATA_PIN        PB5
#define DSP_CS_PIN          PB6
#define DSP_DC_PIN          PC13
#define DSP_RESET_PIN       PC12 

#define TX2_PIN             PA2
#define RX2_PIN             PA3

#define TX3_PIN             PA12
#define RX3_PIN             PA11
#endif

#define FIRST_TOOL_OFFSET       1.2   // value in millimeter
#define TOOL_SPACING            21.0  // value im millimeter
#define FIRST_REVOLVER_OFFSET   320   // value in steps
#define REVOLVER_SPACING        320   // value im steps
#define USER_MESSAGE_RESET      15    // value in seconds
#define MAX_LINES               5
#define MAX_LINE_LENGTH         80
#define POWER_SAVE_TIMEOUT      15    // value in seconds

#define BASE_FONT             u8g2_font_6x12_t_symbols
#define BASE_FONT_BIG         u8g2_font_7x14B_tf
#define SMALL_FONT            u8g2_font_6x10_mr
#define STATUS_FONT           u8g2_font_7x14_tf
#define LOGO_FONT             u8g2_font_helvR08_tf
#define ICONIC_FONT           u8g2_font_open_iconic_check_2x_t

#endif
