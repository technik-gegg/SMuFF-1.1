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
#pragma once

#include "Pins.h"                       // path is defined in build environment of platformio.ini (-I)

#if !defined(STM32_CORE_VERSION)
typedef uint8_t     pin_t;
#else
typedef uint32_t    pin_t;
#endif

#define VERSION_STRING    "V3.23"
#define PMMU_VERSION      106               // Version number for Prusa MMU2 Emulation mode
#define PMMU_BUILD        372               // Build number for Prusa MMU2 Emulation mode
#define VERSION_DATE      "2023-12-22"
#define DEBUG_FILE        "/debug.txt"
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

#define MAX_MATERIAL_LEN        5                   // max. length of materials
#define MAX_MATERIAL_NAME_LEN   10                  // max. length of material names
#define MAX_UNLOAD_COMMAND      20                  // max. length of unload command
#define MAX_WIPE_SEQUENCE       25                  // max. length of wipe sequence
#define MAX_BUTTON_LEN          15                  // max. length of button commands

#define MAX_ERR_MSG             255                 // max. length of error messages

#define NUM_STEPPERS            3
#define SELECTOR                0
#define REVOLVER                1
#define DDE_FEEDER              1
#define FEEDER                  2
#define FEEDER2                 3                   // added for boards with pre-installed stepper drivers

#define MIN_TOOLS               2
#define MAX_TOOLS               12                  // might be extended to 15

#define DSP_CONTRAST            200
#define MIN_CONTRAST            60
#define MAX_CONTRAST            250

#define I2C_SLAVE_ADDRESS       0x88        // default address if the SMuFF I2C is running in slave mode (obsolete)
#define I2C_DISPLAY_ADDRESS     0x3C        // default address for the OLED (alternative 0x3D)
#define I2C_ENCODER_ADDRESS     0x3D        // default address for the LeoNerd Encoder
#define I2C_SERVOCTL_ADDRESS    0x40        // default address for Multiservo controller
#define I2C_MOTORCTL1_ADDRESS   0x41        // default address for 1st motor controller
#define I2C_MOTORCTL2_ADDRESS   0x42        // default address for 2nd motor controller
#define I2C_MOTORCTL3_ADDRESS   0x43        // default address for 3rd motor controller
#define I2C_EEPROM_ADDRESS      0x50        // default address for EEPROM on E3 2.0, 3.0
#define I2C_SERVOBCAST_ADDRESS  0x70        // default address for Multiservo controller (Broadcast Address)
#define I2C_PORTEX_ADDRESS      0x3F        // default address for Port extender (obsolete)
#define I2C_SPL_MUX_ADDRESS     0x3E        // default address for Splitter endstops controller

#define SERVO_WIPER         0               // output assignment on PCA9685 (Multiservo)
#define SERVO_LID           1
#define SERVO_CUTTER        2
#define SERVO_SPARE1        3
#define SERVO_SPARE2        4
#define RELAY               5
#define SERVO_USER1         6
#define SERVO_USER2         7

#define OUT1                6               // yet unused output assignment on PCA9685 (Multiservo)
#define OUT2                7
#define OUT3                8
#define OUT4                9
#define OUT5                10
#define OUT6                11
#define OUT7                12
#define OUT8                13
#define OUT9                14
#define OUT10               15

#define MOTORS_PER_CTRL     4               // number of motors controlled by one Iduino ME704 Shield
                                            // could be 5 on a custom PCB
#define MOTOR_DEFAULT_SPEED 25

#define MA_PWM              0               // output assignment on PCA9685 (Iduino ME704 Shield)
#define MA_IN1              1
#define MA_IN2              2
#define MB_PWM              3
#define MB_IN1              4
#define MB_IN2              5
#define MC_PWM              6
#define MC_IN1              7
#define MC_IN2              8
#define MD_PWM              9
#define MD_IN1              10
#define MD_IN2              11
#define ME_PWM              12              // not on ME704! but possible on a custom made PCB
#define ME_IN1              13
#define ME_IN2              14
// IN1  IN2   PWM   Mode
//------------------------
//  L    H    SPD   CW
//  H    L    SPD   CCW
//  H    H     H    STOP
#define MOTOR_DIR_CW        0
#define MOTOR_DIR_CCW       1
#define MOTOR_DIR_STOP      2


#define MS_MODE_UNSET       -1
#define MS_MODE_PWM         0
#define MS_MODE_OUTPUT      1

#define SERVO_CLOSED_OFS    35                          // for Multiservo

#define GPTIMER_RESOLUTION  250                         // general purpose timer ISR called every n microseconds
#define SERVO_RESOLUTION    50                          // servo ISR called every n microseconds
#define FAN_RESOLUTION      GPTIMER_RESOLUTION          // fan ISR service interval same as GP-Timer
#define LED_RESOLUTION      10000                       // led ISR called every n microseconds

#define FAN_FREQUENCY       100                         // fan frequency in Hz
#define FAN_BLIP_TIMEOUT    1000                        // fan blip timeout in millis (0 to turn blipping off)

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

#if defined(__STM32F1XX)
#define STEPPER_PSC         36          // 2MHz on STM32F1 (72MHz MCU/SysClock)
#define GP_PSC              72          // 1MHz 
#define SERVO_PSC           72          // 1MHz
#define LED_PSC             72          // 1MHz
#elif defined(__STM32F4XX)
#define STEPPER_PSC         84          // 2MHz on STM32F4 (168MHz MCU/SysClock)
#define GP_PSC              168         // 1MHz 
#define SERVO_PSC           168         // 1MHz
#define LED_PSC             168         // 1MHz
#elif defined(__STM32G0XX)
#define STEPPER_PSC         32          // 2MHz on STM32G0 (64MHz MCU/SysClock)
#define GP_PSC              64          // 1MHz 
#define SERVO_PSC           64          // 1MHz
#define LED_PSC             64          // 1MHz
#else
#define STEPPER_PSC         8           // 2MHz on AVR (16MHz MCU)
#endif
#define MAX_POWER           2000        // maximum allowed power for rms_current()
#define MAX_STALL_COUNT     100         // maximum stall counter for stepper
#define MAX_MMS             700         // maximum mm/s for in menus
#define MAX_TICKS           65000       // maximum ticks in menus
#define INC_MMS             5           // speed increment for mm/s
#define INC_TICKS           50          // speed increment for ticks
#define MAX_MENU_ORDINALS   40

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

#define FASTLED_STAT_NONE       0
#define FASTLED_STAT_MARQUEE    1
#define FASTLED_STAT_RAINBOW    2
#define FASTLED_STAT_CYLON      3
#define FASTLED_STAT_ERROR      10
#define FASTLED_STAT_WARNING    11
#define FASTLED_STAT_OK         12

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


#if defined(ARDUINO_ARCH_AVR)
#include <util/atomic.h>
#define CRITICAL_SECTION ATOMIC_BLOCK(ATOMIC_RESTORESTATE)

#elif defined(ARDUINO_ARCH_SAM)
  // Workaround as suggested by Stackoverflow user "Notlikethat"
  // http://stackoverflow.com/questions/27998059/atomic-block-for-reading-vs-arm-systicks

  static inline int __int_disable_irq(void) {
    int primask;
    asm volatile("mrs %0, PRIMASK\n" : "=r"(primask));
    asm volatile("cpsid i\n");
    return primask & 1;
  }

  static inline void __int_restore_irq(int *primask) {
    if (!(*primask)) {
      asm volatile ("" ::: "memory");
      asm volatile("cpsie i\n");
    }
  }
  // This critical section macro borrows heavily from
  // avr-libc util/atomic.h
  // --> http://www.nongnu.org/avr-libc/user-manual/atomic_8h_source.html
  #define CRITICAL_SECTION for (int primask_save __attribute__((__cleanup__(__int_restore_irq))) = __int_disable_irq(), __ToDo = 1; __ToDo; __ToDo = 0)

#elif defined(ARDUINO_ARCH_STM32)
#include <Arduino.h>
#include <cmsis_gcc.h>
	// exact same as above only using predefined CMSIS functions

  static inline int __int_disable_irq(void) {
    int primask = __get_PRIMASK();
    __disable_irq();
    return primask & 1;
  }

  static inline void __int_restore_irq(int *primask) {
    if (!(*primask)) {
      __enable_irq();
    }
  }
  #define CRITICAL_SECTION for (int primask_save __attribute__((__cleanup__(__int_restore_irq))) = __int_disable_irq(), __ToDo = 1; __ToDo; __ToDo = 0)
#else
  #error Unsupported controller architecture
#endif