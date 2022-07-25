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

#include "SMuFF.h"

typedef void(*iCallback)(int val);
typedef void(*fCallback)(double val);
typedef void(*bCallback)(bool val);

bool getEncoderButton(bool encoderOnly = false);
void getEncoderButton(int16_t* turn, uint8_t* button, bool* isHeld, bool* isClicked);
void getInput(int16_t* turn, uint8_t* button, bool* isHeld, bool* isClicked, bool checkSerial = true);
void drawValue(const char* title, const char* PROGMEM message, String val);
bool showInputDialog(const char* title, const char* PROGMEM message, double* val, double min, double max, fCallback cb = nullptr, double increment = 1.0);
bool showInputDialog(const char* title, const char* PROGMEM message, int* val, int16_t min, int16_t max, iCallback cb = nullptr, int16_t increment = 1);
bool showInputDialog(const char* title, const char* PROGMEM message, bool* val, bCallback cb = nullptr);
bool showInputDialog(const char* title, const char* PROGMEM message, unsigned long* val, String list);
bool showInputDialog(const char* title, const char* PROGMEM message, int* val, String list, iCallback cb = nullptr, bool valIsIndex = true);
bool showInputDialog(const char* title, const char* PROGMEM message, uint16_t* val, uint16_t min, uint16_t max, iCallback cb = nullptr, int16_t increment = 1);
