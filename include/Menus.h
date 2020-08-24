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

#ifndef _MENUS_H
#define _MENUS_H 1

extern void setupMainMenu(char* menu);
extern void setupToolsMenu(char* menu);
extern void setupStatusInfoMenu(char* menu);
extern void setupSwapMenu(char* menu);
extern void setupSettingsMenu(char* menu);
extern void setupTestrunMenu(char* menu);
extern void setupOptionsMenu(char* menu);

extern void showMainMenu();
extern void showToolsMenu();
extern void showStatusInfoMenu(char* menuTitle);
extern void showSwapMenu(char* menuTitle);
extern void showSettingsMenu(char* menuTitle);
extern void showBaudratesMenu(char* menuTitle);
extern void showOptionsMenu(char* menuTitle);
extern void showTestrunMenu(char* menuTitle);
extern void showTMCStatus(int axis);
extern void changeOffset(int index);
extern void drawOffsetPosition(int index);
extern bool selectBaudrate(int port, char* menuTitle);
extern void selectPanelDuePort(char* menuTitle);

extern const char* translateColor(int color);
extern const char* translatePanelDuePort(int port);
extern const char* translateTMCDriverMode(int mode);
#endif