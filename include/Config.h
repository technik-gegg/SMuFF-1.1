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
#pragma once

#define VERSION_STRING    "V2.28"
#define PMMU_VERSION      106               // Version number for Prusa MMU2 Emulation mode
#define PMMU_BUILD        372               // Build number for Prusa MMU2 Emulation mode
#define VERSION_DATE      "2021-05-15"
#define CONFIG_FILE       "/SMUFF.json"
#define STEPPERS_FILE     "/STEPPERS.json"
#define MATERIALS_FILE    "/MATERIALS.json"
#define TMC_CONFIG_FILE   "/TMCDRVR.json"
#define SERVOMAP_FILE     "/SERVOMAP.json"
#define STEPPERMAP_FILE   "/REVOLVERMAP.json"
#define DATASTORE_FILE    "/EEPROM.json"
#define STARTUP_FILE      "STARTUP.DAT"
#define BEEP_FILE         "BEEP.DAT"
#define LONGBEEP_FILE     "LBEEP.DAT"
#define USERBEEP_FILE     "UBEEP.DAT"
#define ENCBEEP_FILE      "EBEEP.DAT"
#define ENCBEEPLEO_FILE   "EBEEP_LEONERD.DAT"

#define MAX_MATERIAL_LEN        5                 // max. length of materials
#define MAX_MATERIAL_NAME_LEN   10                // max. length of material names
#define MAX_UNLOAD_COMMAND      20                // max. length of unload command
#define MAX_WIPE_SEQUENCE       25                // max. length of wipe sequence
#define MAX_BUTTON_LEN          15                // max. length of button commands

#define NUM_STEPPERS      3
#define SELECTOR          0
#define REVOLVER          1
#define FEEDER            2
#define FEEDER2           3                 // added for boards with pre-installed stepper drivers

#define MIN_TOOLS         2
#define MAX_TOOLS         12

#define DSP_CONTRAST        200
#define MIN_CONTRAST        60
#define MAX_CONTRAST        250

#define I2C_SLAVE_ADDRESS       0x88
#define I2C_DISPLAY_ADDRESS     0x3C        // supposed to be wired by default on OLED (alternative 0x3D)
#define I2C_ENCODER_ADDRESS     0x3D        // default address for the LeoNerd Encoder
#define I2C_SERVOCTL_ADDRESS    0x40        // default address for multi servo controller
#define I2C_EEPROM_ADDRESS      0x50        // default address for EEPROM on E3 2.0
#define I2C_SERVOBCAST_ADDRESS  0x70        // default address for multi servo controller (Broadcast Address)

#define SERVO_WIPER         0
#define SERVO_LID           1
#define SERVO_CUTTER        2

#define SERVO_CLOSED_OFS    35          // for Multiservo
#define SERVO_RESOLUTION    42          // servo ISR service routine called every SERVO_RESOLUTION uS

#define FAN_RESOLUTION      50          // fan ISR service routine called every FAN_RESOLUTION uS
                                        // basically same as SERVO_RESOLUTION
#define FAN_FREQUENCY       100         // fan frequency in Hz
#define FAN_BLIP_TIMEOUT    1000        // fan blip timeout in millis (0 to turn bliping off)

#define FEED_ERROR_RETRIES  4

#define REMOTE_NONE         0
#define REMOTE_UP           1
#define REMOTE_DOWN         2
#define REMOTE_SELECT       3
#define REMOTE_ESCAPE       4
#define REMOTE_HOME         5
#define REMOTE_END          6
#define REMOTE_PGUP         7
#define REMOTE_PGDN         8
#define REMOTE_PF1          9
#define REMOTE_PF2          10
#define REMOTE_PF3          11
#define REMOTE_PF4          12

#if defined(__STM32F1__)
#define STEPPER_PSC         3           // 24MHz on STM32 (72MHz MCU)
#elif defined(__ESP32__)
#define STEPPER_PSC         10          // 8MHz on ESP32 (80MHz MCU)
#else
#define STEPPER_PSC         2           // 8MHz on AVR (16MHz MCU)
#endif
#define MAX_POWER           2000        // maximum allowed power for rms_current()
#define MAX_STALL_COUNT     100         // maximum stall counter for stepper
#define MAX_MMS             700         // maximum mm/s for in menus
#define MAX_TICKS           65000       // maximum ticks in menus
#define INC_MMS             5           // speed increment for mm/s
#define INC_TICKS           50          // speed increment for ticks
#define MAX_MENU_ORDINALS   40

#include "Pins.h"                       // path is defined in build environment of platformio.ini (-I)

#define FIRST_TOOL_OFFSET       1.2     // value in millimeter
#define TOOL_SPACING            21.0    // value im millimeter
#define FIRST_REVOLVER_OFFSET   320     // value in steps
#define REVOLVER_SPACING        320     // value im steps
#define USER_MESSAGE_RESET      15      // value in seconds
#define MAX_LINES               5
#define MAX_LINE_LENGTH         80
#define POWER_SAVE_TIMEOUT      15      // value in seconds

#if !defined(NUM_LEDS)
#define NUM_LEDS                1       // number of Neopixel LEDS
#define BRIGHTNESS              64
#define LED_TYPE                WS2812B
#define COLOR_ORDER             GRB
#endif
#define LED_BLACK_COLOR         0       // color codes for RGB LEDs
#define LED_RED_COLOR           1
#define LED_GREEN_COLOR         2
#define LED_BLUE_COLOR          3
#define LED_CYAN_COLOR          4
#define LED_MAGENTA_COLOR       5
#define LED_YELLOW_COLOR        6
#define LED_WHITE_COLOR         7

#define BASE_FONT               u8g2_font_6x12_t_symbols
#define BASE_FONT_BIG           u8g2_font_7x14_tf
#define SMALL_FONT              u8g2_font_6x10_tr
#define STATUS_FONT             BASE_FONT_BIG
#define LOGO_FONT               BASE_FONT
#define ICONIC_FONT             u8g2_font_open_iconic_check_2x_t
#define ICONIC_FONT2            u8g2_font_open_iconic_embedded_2x_t
#define TOOL_FONT               u8g2_font_logisoso22_tr

#define FASTLED_STAT_NONE       0
#define FASTLED_STAT_MARQUEE    1
#define FASTLED_STAT_RAINBOW    2
#define FASTLED_STAT_ERROR      3
#define FASTLED_STAT_WARNING    4
#define FASTLED_STAT_OK         5

#define TERM_LINE_WIDTH         25
#define TERM_LINES              6
#define TERM_OFFS_X             40
#define TERM_OFFS_Y             3

const char terminalLineChrs[] PROGMEM = { 0xC4, 0xCD, 0xBA, 0xC9, 0xBB, 0xC8, 0xBC, 0x78, 0x6F };
// Alternative line drawing chars; Only if no "Terminal" font is available
// const char terminalLineChrs[] PROGMEM = { '-', '=', '|', '+', '+', '+', '+', 'x', 'o' };

#define TERM_SEPARATOR_CHR      terminalLineChrs[0]
#define TERM_HORZLINE_CHR       terminalLineChrs[1]
#define TERM_VERTLINE_CHR       terminalLineChrs[2]
#define TERM_CORNERUL_CHR       terminalLineChrs[3]
#define TERM_CORNERUR_CHR       terminalLineChrs[4]
#define TERM_CORNERLL_CHR       terminalLineChrs[5]
#define TERM_CORNERLR_CHR       terminalLineChrs[6]
#define TERM_NOTAVAIL_CHR       terminalLineChrs[7]
#define TERM_PRESENT_CHR        terminalLineChrs[8]

#define TERM_INVERTED           7
#define TERM_UNDERLINE          4
#define TERM_FGC_BLACK          30
#define TERM_FGC_RED            31
#define TERM_FGC_GREEN          32
#define TERM_FGC_YELLOW         33
#define TERM_FGC_BLUE           34
#define TERM_FGC_MAGENTA        35
#define TERM_FGC_CYAN           36
#define TERM_FGC_WHITE          37
#define TERM_BGC_BLACK          40
#define TERM_BGC_RED            41
#define TERM_BGC_GREEN          42
#define TERM_BGC_YELLOW         43
#define TERM_BGC_BLUE           44
#define TERM_BGC_MAGENTA        45
#define TERM_BGC_CYAN           46
#define TERM_BGC_WHITE          47
#define TERM_FGC_NONE           255
