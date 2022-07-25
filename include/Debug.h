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
extern void __initDebug__();
extern void __flushDebug__();
extern void __debugS__(uint8_t level, bool isInt, const char* fmt, ...);
#define __initDebug()                   __initDebug__()
#define __debugS(level, fmt, ...)       __debugS__(level, false, fmt, ## __VA_ARGS__)
#define __debugSInt(level, fmt, ...)    __debugS__(level, true, fmt, ## __VA_ARGS__)
#define __flushDebug()                  __flushDebug__()
#else
#define __initDebug()
#define __debugS(fmt, ...)
#define __debugSInt(fmt, ...)
#define __flushDebug()
#endif

