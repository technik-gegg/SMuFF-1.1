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

#include <stdlib.h>
#include <Arduino.h>

typedef struct {
  int16_t code;
  bool (*func)(const char *msg, String buf, int8_t serial, char* errmsg);
} GCodeFunctions;

extern uint16_t currentLine;

extern bool dummy(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M0(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M2(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M3(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M4(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M5(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M17(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M18(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M20(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M42(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M98(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M100(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M106(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M107(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M110(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M111(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M114(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M115(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M117(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M118(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M119(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M122(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M145(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M150(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M155(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M201(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M202(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M203(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M205(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M206(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M250(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M260(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M261(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M280(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M300(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M350(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M412(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M500(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M502(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M503(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M504(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M569(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M562(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M575(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M577(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M700(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M701(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M906(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M914(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M997(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M999(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M2000(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool M2001(const char *msg, String buf, int8_t serial, char* errmsg);

extern bool G0(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool G1(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool G4(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool G12(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool G28(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool G90(const char *msg, String buf, int8_t serial, char* errmsg);
extern bool G91(const char *msg, String buf, int8_t serial, char* errmsg);

extern uint16_t handleFeedSpeed(String buf, uint8_t axis);
extern void changeFeederSpeed(uint16_t speed);
extern void changeDDEFeederSpeed(uint16_t speed);
