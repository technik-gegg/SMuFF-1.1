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

#include <Arduino.h>

#define D       1
#define W       2
#define I       4
#define SP      8
#define DEV     16
#define DEV2    32
#define DEV3    64
#define ALL D | W | I

#if defined(DEBUG)
extern void __debugS__(uint8_t level, const char* fmt, ...);
#define __debugS(level, fmt, ...)   __debugS__(level, fmt, ## __VA_ARGS__)
#else
#define __debugS(fmt, ...)
#endif

