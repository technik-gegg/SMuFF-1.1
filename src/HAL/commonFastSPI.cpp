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

#include "SMuFF.h"
#include "./commonFastSPI.h"

/*
    Code copied from https://github.com/olikraus/u8g2/issues/749 and adopted for testing
*/

uint8_t u8x8_byte_smuff_4wire_sw_spi(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr) {
    uint8_t i, b;
    uint8_t* data;
    uint8_t takeover_edge = u8x8_GetSPIClockPhase(u8x8);

    /* the following static vars are recalculated in U8X8_MSG_BYTE_START_TRANSFER */
    /* so, it should be possible to use multiple displays with different pins */

    static volatile uint32_t* arduino_clock_port;

    static uint32_t arduino_clock_mask;
    static uint32_t arduino_clock_n_mask;

    static volatile uint32_t* arduino_data_port;
    static uint32_t arduino_data_mask;
    static uint32_t arduino_data_n_mask;

    switch (msg) {
    case U8X8_MSG_BYTE_SEND:

        data = (uint8_t*)arg_ptr;
        if (takeover_edge == 0) {
            while (arg_int > 0) {
                b = *data;
                data++;
                arg_int--;
                {
                    for (i = 0; i < 8; i++) {
                        if (b & 128)
                            *arduino_data_port |= arduino_data_mask;
                        else
                            *arduino_data_port &= arduino_data_n_mask;

                        smuff_spi_delay();
                        *arduino_clock_port |= arduino_clock_mask;
                        b <<= 1;
                        smuff_spi_delay();
                        *arduino_clock_port &= arduino_clock_n_mask;
                    }
                }
            }
        } else {
            while (arg_int > 0) {
                b = *data;
                data++;
                arg_int--;
                {
                    for (i = 0; i < 8; i++) {
                        if (b & 128)
                            *arduino_data_port |= arduino_data_mask;
                        else
                            *arduino_data_port &= arduino_data_n_mask;

                        smuff_spi_delay();
                        *arduino_clock_port &= arduino_clock_n_mask;
                        b <<= 1;
                        smuff_spi_delay();
                        *arduino_clock_port |= arduino_clock_mask;
                    }
                }
            }
        }
        break;

    case U8X8_MSG_BYTE_INIT:
        /* disable chipselect */
        u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
        /* no wait required here */

        /* for SPI: setup correct level of the clock signal */
        u8x8_gpio_SetSPIClock(u8x8, u8x8_GetSPIClockPhase(u8x8));
        break;
    case U8X8_MSG_BYTE_SET_DC:
        u8x8_gpio_SetDC(u8x8, arg_int);
        break;
    case U8X8_MSG_BYTE_START_TRANSFER:
        u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_enable_level);
        u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_NANO, u8x8->display_info->post_chip_enable_wait_ns, NULL);

        /* there is no consistency checking for u8x8->pins[U8X8_PIN_SPI_CLOCK] */

        arduino_clock_port = portOutputRegister(digitalPinToPort(u8x8->pins[U8X8_PIN_SPI_CLOCK]));
        arduino_clock_mask = digitalPinToBitMask(u8x8->pins[U8X8_PIN_SPI_CLOCK]);
        arduino_clock_n_mask = ~arduino_clock_mask;

        /* there is no consistency checking for u8x8->pins[U8X8_PIN_SPI_DATA] */

        arduino_data_port = portOutputRegister(digitalPinToPort(u8x8->pins[U8X8_PIN_SPI_DATA]));
        arduino_data_mask = digitalPinToBitMask(u8x8->pins[U8X8_PIN_SPI_DATA]);
        arduino_data_n_mask = ~arduino_data_mask;

        break;
    case U8X8_MSG_BYTE_END_TRANSFER:
        u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_NANO, u8x8->display_info->pre_chip_disable_wait_ns, NULL);
        u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
        break;
    default:
        return 0;
    }
    return 1;
}