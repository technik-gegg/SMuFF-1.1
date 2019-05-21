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

#ifndef _SMUFF_STRINGS_H
#define _SMUFF_STRINGS_H

#include <avr/pgmspace.h>

#define SD_ERR_INIT           1
#define SD_ERR_NOCONFIG       2
#define SD_READING_CONFIG     0

const char P_MenuItemBack [] PROGMEM        = { "\u25c0 BACK\n" };
const char P_MenuItems [] PROGMEM           = { "Home\nMotors off\nOffsets\nLoad feeder\nUnload feeder\nSwap tools" };
const char P_OfsMenuItems [] PROGMEM        = { "Selector\nRevolver" };
const char P_OkButtonOnly [] PROGMEM        = { " Ok " };
const char P_CancelButtonOnly [] PROGMEM    = { " Cancel " };
const char P_OkCancelButtons [] PROGMEM     = { " Ok \n Cancel " };
const char P_YesNoButtons [] PROGMEM        = { " Yes \n No " };
const char P_TitleWarning [] PROGMEM        = { "WARNING" };
const char P_TitleSelected [] PROGMEM       = { "TOOL SELECTED" };
const char P_FeederLoaded [] PROGMEM        = { "Feeder is loaded!\n" };
const char P_AskUnload [] PROGMEM           = { "Want me to unload\nit now?" };
const char P_AskLoad [] PROGMEM             = { "Want me to load\nit now?" };
const char P_SelectedTool [] PROGMEM        = { "\n" };
const char P_CantLoad [] PROGMEM            = { "Can't load feeder." };
const char P_CantUnload [] PROGMEM          = { "Can't unload feeder." };
const char P_CheckUnit [] PROGMEM           = { "Please check unit!" };
const char P_TitleConfigError [] PROGMEM    = { "CONFIG FAILURE" };
const char P_ConfigFail1 [] PROGMEM         = { "Your config file is" };
const char P_ConfigFail2 [] PROGMEM         = { "possibly corrupted,\nplease check!" };
const char P_ConfigFail3 [] PROGMEM         = { "too big,\nplease reduce content!" };
const char P_Tool [] PROGMEM                = { "Tool " };
const char P_ToolMenu [] PROGMEM            = { "Tool %d" };
const char P_SwapMenu [] PROGMEM            = { "Slot %d: T%d" };
const char P_SwapReset [] PROGMEM           = { "Reset swaps\n" };
const char P_SwapToolDialog [] PROGMEM      = { "Swap Tool %d\nwith Tool %d" };
const char P_Selecting [] PROGMEM           = { "Selecting" };
const char P_Wait [] PROGMEM                = { "please wait..." };
const char P_TitleMainMenu [] PROGMEM       = { "Main menu" };
const char P_TitleToolsMenu [] PROGMEM      = { "Tool Selection" };
const char P_TitleOffsetsMenu [] PROGMEM    = { "Offsets calibration" };
const char P_TitleSwapMenu [] PROGMEM       = { "Swap tools" };
const char P_Busy[] PROGMEM                 = { "busy..." };
const char P_Ready[] PROGMEM                = { "ready." };

const char P_SD_ReadingConfig[] PROGMEM = { "Reading config..." };
const char P_SD_InitError[] PROGMEM     = { "SD-Card not ready!" };
const char P_SD_NoConfig[] PROGMEM      = { "No config file found!" };

const char P_Ok[] PROGMEM             = { "ok\n" };
const char P_Start[] PROGMEM          = { "start\n" };
const char P_Error[] PROGMEM          = { "Error: %s\n" };
const char P_UnknownCmd[] PROGMEM     = { "Unknown command:" };
const char P_AlreadySaved[] PROGMEM   = { "Already saved.\n" };
const char P_GVersion[] PROGMEM       = { "FIRMWARE_NAME: Smart.Multi.Filament.Feeder (SMuFF) FIRMWARE_VERSION: %s ELECTRONICS: Wanhao i3-Mini DATE: %s\n" };
const char P_TResponse[] PROGMEM      = { "T%d\n" };
const char P_GResponse[] PROGMEM      = { "G%d\n" };
const char P_MResponse[] PROGMEM      = { "M%d\n" };
const char P_M250Response[] PROGMEM   = { "M250 C%d\n" };

const char P_SelectorPos[] PROGMEM    = { "%3d: Selector position = %ld\n" };
const char P_RevolverPos[] PROGMEM    = { "%3d: Revolver position = %ld\n" };
const char P_FeederPos[] PROGMEM      = { "%3d: Feeder position = %ld\n" };
const char P_ToolSelected[] PROGMEM   = { "%3d: Tool selected = %d\n" };
const char P_Contrast[] PROGMEM       = { "%3d: Display contrast = %d\n" };
const char P_ToolsConfig[] PROGMEM    = { "%3d: Tools configured = %d\n" };
const char P_AccelSpeed[] PROGMEM     = { "X (Selector):\t%s\nY (Revolver):\t%s\nZ (Feeder):\t%s\n" };

const char P_CurrentTool[] PROGMEM    = {"Tool    " };
const char P_Feed[] PROGMEM           = {"Feed    " };

const char P_NoTool [] PROGMEM        = { "No tool set.\n" };
const char P_Aborting [] PROGMEM      = { "Aborting..."};
const char P_FeederJamed [] PROGMEM   = { "Feeder is jamed.\n" };
const char P_ToolAlreadySet [] PROGMEM  = { "Tool already set." };

const char P_WrongFormat [] PROGMEM   = { "Wrong format. Use Bdd:dd:dd...\n" };

const char P_GCmds[] PROGMEM = { 
  "G0\t-\tMove\n" \
  "G1\t-\tMove\n" \
  "G4\t-\tDwell\n" \
  "G12\t-\tWipe filament\n" \
  "G28\t-\tHome\n" \
  "G90\t-\tAbsolute positioning\n" \
  "G91\t-\tRelative positioning\n" };

const char P_MCmds[] PROGMEM = { 
  "M18\t-\tMotors off\n" \
  "M84\t-\tMotors off\n" \
  "M20\t-\tList SD-Card\n" \
  "M42\t-\tSet pin state\n" \
  "M106\t-\tFan on\n" \
  "M107\t-\tFan off\n" \
  "M114\t-\tReport current positions\n" \
  "M115\t-\tReport version\n" \
  "M117\t-\tDisplay message\n" \
  "M119\t-\tReport endstop status\n" \
  "M120\t-\tEnable endstops\n" \
  "M121\t-\tDisable endstops\n" \
  "M122\t-\tReport Diagnostics\n" \
  "M201\t-\tSet max acceleration\n" \
  "M203\t-\tSet max feedrate\n" \
  "M206\t-\tSet offsets\n" \
  "M250\t-\tLCD contrast\n" \
  "M300\t-\tBeep\n" \
  "M500\t-\tSave settings\n" \
  "M503\t-\tReport settings\n" \
  "M700\t-\tLoad filament\n" \
  "M701\t-\tUnload filament\n" \
  "M999\t-\tReset\n" \
  "M2000\t-\tText to decimal\n" \
  "M2001\t-\tDecimal to text\n"};

                             
#endif
