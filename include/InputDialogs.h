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

#ifndef _INPUT_DIALOGS_H
#define _INPUT_DIALOGS_H 1

#include "SMuFF.h"
typedef void(*iCallback)(int val);
typedef void(*fCallback)(float val);
typedef void(*bCallback)(bool val);

void getEncoderButton(int* turn, int* button, bool* isHeld, bool* isClicked);
void drawValue(const char* title, const char* PROGMEM message, String val);
bool showInputDialog(const char* title, const char* PROGMEM message, float* val, float min, float max, fCallback cb = NULL);
bool showInputDialog(const char* title, const char* PROGMEM message, int* val, int min, int max, iCallback cb = NULL);
bool showInputDialog(const char* title, const char* PROGMEM message, bool* val, bCallback cb = NULL);
bool showInputDialog(const char* title, const char* PROGMEM message, unsigned long* val, String list);
bool showInputDialog(const char* title, const char* PROGMEM message, int* val, String list, iCallback cb = NULL, bool valIsIndex = true);

#endif
