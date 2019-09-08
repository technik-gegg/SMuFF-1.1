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

#define VERSION_STRING    "V1.3"
#define VERSION_MAJOR     1
#define VERSION_MINOR     2  
#define PMMU_VERSION      106               // Version number for Prusa MMU2 Emulation mode
#define PMMU_BUILD        372               // Build number for Prusa MMU2 Emulation mode
#define VERSION_DATE      "2019-09-08"
#define CONFIG_FILE       "SMUFF.CFG"

#define SELECTOR          0
#define REVOLVER          1
#define FEEDER            2

#define NUM_STEPPERS      3

#define MIN_TOOLS         2
#define MAX_TOOLS         9

#define STEP_HIGH_X         PORTA |=  0b00000001;
#define STEP_LOW_X          PORTA &= ~0b00000001;
#define X_STEP_PIN          22
#define X_DIR_PIN           23
#define X_ENABLE_PIN        57
#define X_END_PIN           19
#define X_STEPS_PER_MM      800

#define STEP_HIGH_Y         PORTA |=  0b00001000;
#define STEP_LOW_Y          PORTA &= ~0b00001000;
#define Y_STEP_PIN          25
#define Y_DIR_PIN           26
#define Y_ENABLE_PIN        24
#define Y_END_PIN           18

#define STEP_HIGH_Z         PORTA |=  0b10000000;
#define STEP_LOW_Z          PORTA &= ~0b10000000;
#define Z_STEP_PIN          29
#define Z_DIR_PIN           39
#define Z_ENABLE_PIN        28
#define Z_END_PIN           38
#define Z_STEPS_PER_MM      136

#define BEEPER_PIN          37
#define BEEPER_FREQUENCY    1760
#define BEEPER_DURATION     90
#define BEEPER_UFREQUENCY   440
#define BEEPER_UDURATION    90

#define SERVO1_PIN          44
#define SERVO2_PIN          14
#define SD_SS_PIN           53
#define FAN_PIN             12
#define HEATER0_PIN         4

#define ENCODER1_PIN        2
#define ENCODER2_PIN        3
#define ENCODER_BUTTON_PIN  5
#define ENCODER_DELAY       4
#define ENCODER_DELAY_MENU  2
#define ENCODER_DELAY_OFS   1

#define DSP_CLOCK_PIN       52
#define DSP_DATA_PIN        51
#define DSP_CS_PIN          41
#define DSP_DC_PIN          40
#define DSP_RESET_PIN       27
#define DSP_BACKLIGHT_PIN   65
#define DSP_CONTRAST        200
#define MIN_CONTRAST        60
#define MAX_CONTRAST        250

#define TX2_PIN             16
#define RX2_PIN             17
#define I2C_SLAVE_ADDRESS   0x88

#define TX3_PIN             67
#define RX3_PIN             44

#define FIRST_TOOL_OFFSET       1.2   // values in millimeter
#define TOOL_SPACING            21.0  // values im millimeter
#define FIRST_REVOLVER_OFFSET   320   // values in steps
#define REVOLVER_SPACING        320   // values im steps
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

#define EEPROM_SELECTOR_POS   SELECTOR*sizeof(long)
#define EEPROM_REVOLVER_POS   REVOLVER*sizeof(long)
#define EEPROM_FEEDER_POS     FEEDER*sizeof(long)
#define EEPROM_TOOL           20
#define EEPROM_CONTRAST       22
#define EEPROM_TOOL_COUNT     24
#define EEPROM_FEED_LENGTH    26
#define EEPROM_1ST_TOOL_OFS   30
#define EEPROM_TOOL_SPACING   34
#define EEPROM_1ST_REV_OFS    38
#define EEPROM_REV_SPACING    42
#endif
