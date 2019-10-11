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

#ifdef __STM32F1__
const char P_MenuItemBack [] PROGMEM        = { "\u25c0 BACK\n" };
const char P_MenuItemSeparator [] PROGMEM   = { "\u25ab\u25ab\u25ab\u25ab\u25ab\n"};
const char P_MenuItems [] PROGMEM           = { "Home All\nMotors %s\nReset Feeder Jam\nSwap Tools \u25b8\nLoad Filament\nUnload Filament\n%s%s%s" };
const char P_MenuItemsDefault[] PROGMEM     = { "Settings \u25b8\n%sTestrun \u25b8" };
const char P_OfsMenuItems [] PROGMEM        = { "Selector         %4s\nRevolver        %5s" };
#else
const char P_MenuItemBack [] PROGMEM        = { "< BACK\n" };
const char P_MenuItemSeparator [] PROGMEM   = { "-----\n"};
const char P_MenuItems [] PROGMEM           = { "Home All\nMotors %s\nReset Feeder Jam\nSwap Tools >\nLoad Filament\nUnload Filament\nOffsets >" };
//const char P_MenuItems [] PROGMEM           = { "Home All\nMotors %s\nReset Feeder Jam\nSwap Tools >\nLoad Filament\nUnload Filament\n%s%S%S" };
const char P_MenuItemsDefault[] PROGMEM     = { "Settings >" };
const char P_OfsMenuItems [] PROGMEM        = { "Selector         %4s\nRevolver        %5s" };
#endif
const char P_MenuItemsPMMU [] PROGMEM       = { "Load To Nozzle\n" };
const char P_OkButtonOnly [] PROGMEM        = { " Ok " };
const char P_CancelButtonOnly [] PROGMEM    = { " Cancel " };
const char P_OkCancelButtons [] PROGMEM     = { " Ok \n Cancel " };
const char P_CancelRetryButtons [] PROGMEM  = { " Cancel \n Retry " };
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
const char P_ConfigFail4 [] PROGMEM         = { "data inconsistent\nor memory failure!" };
const char P_ToolMenu [] PROGMEM            = { "Tool %d" };
const char P_SwapMenu [] PROGMEM            = { "Slot %d: T%d" };
const char P_SwapReset [] PROGMEM           = { "Reset swaps\n" };
const char P_SwapToolDialog [] PROGMEM      = { "Swap Tool %d\nwith Tool %d" };
const char P_Selecting [] PROGMEM           = { "Selecting" };
const char P_Wait [] PROGMEM                = { "please wait..." };
const char P_TitleMainMenu [] PROGMEM       = { "Main Menu" };
const char P_TitleToolsMenu [] PROGMEM      = { "Tool Selection" };
const char P_Busy[] PROGMEM                 = { "busy..." };
const char P_Ready[] PROGMEM                = { "ready." };
const char P_Pemu[] PROGMEM                 = { "PMMU2" };
#ifdef __STM32F1__
const char P_SettingsMenuItems[] PROGMEM    = { "Tool Count      %5s\nBowden Length   %5s\nSelector Dist.  %5s\nMenu Auto Close  %4s\nFan Speed       %5s\nPower Save Time %5s\nPrusa MMU2 Emul. %4s\nBaudrates          %4s\nOffsets            %4s\nSteppers           %4s\n%s\u25b9 SAVE TO SD-CARD \u25c3" };
#else
const char P_SettingsMenuItems[] PROGMEM    = { "Tool Count      %5s\nBowden Length   %5s\nSelector Dist.  %5s\nMenu Auto Close  %4s\nFan Speed       %5s\nPower Save Time %5s\nPrusa MMU2 Emul. %4S\nBaudrates           %s\nOffsets             %s\nSteppers            %s\n%S> SAVE TO SD-CARD <" };
#endif
const char P_Off[] PROGMEM                  = { "OFF" };
const char P_On[] PROGMEM                   = { "ON" };
const char P_Yes[] PROGMEM                  = { "Yes" };
const char P_No[] PROGMEM                   = { "No" };
const char P_High[] PROGMEM                 = { "HI" };
const char P_Low[] PROGMEM                  = { "LO" };

const char P_ToolCount[] PROGMEM            = { "# of tools:" };
const char P_InMillimeter[] PROGMEM         = { "in mm:" };
const char P_InSeconds[] PROGMEM            = { "in seconds:" };
const char P_InPercent[] PROGMEM            = { "in percent:" };
const char P_YesNo[] PROGMEM                = { "yes / no:" };
const char P_Baud[] PROGMEM                 = { "Baudrate:" };
const char P_InMicroseconds[] PROGMEM       = { "in uS:" };
const char P_InTicks[] PROGMEM              = { "in ticks:" };
const char P_InSteps[] PROGMEM              = { "in steps:" };
const char P_TriggerOn[] PROGMEM            = { "on:" };
const char P_NoOfChunks[] PROGMEM           = { "# of chunks:" };
const char P_BaudMenuItems[] PROGMEM        = { "USB-Serial     %6s\n2nd Serial     %6s" };
const char P_Baudrates[] PROGMEM            = { "4800\n9600\n19200\n38400\n56700\n115200\n230400" };
#ifdef __STM32F1__
const char P_SteppersMenuItems[] PROGMEM    = { "Selector            %2s\nRevolver            %2s\nFeeder              %2s" };
const char P_AllSteppersMenuItems[] PROGMEM = { "Invert DIR       %4s\nEndstop Trigger  %4s\nStep Delay       %4s\nMax. Speed      %5s\nMax. Speed HS   %5s\nAcceleration    %5s" };
const char P_RevolverMenuItems[] PROGMEM    = { "\nSteps Per Rev.   %5s\nHome After Feed  %4s\nReset Bef. Feed  %4s" };
const char P_FeederMenuItems[] PROGMEM      = { "\nSteps Per MM    %5s\nEnable Chunks    %4s\nFeed Chunks      %4s\nInsert Length    %5s\nInsert Speed     %5s\nReinforce Len.  %5s" };
const char P_SelectorMenuItems[] PROGMEM    = { "\nSteps Per MM    %5s" };
#else
const char P_SteppersMenuItems[] PROGMEM    = { "Selector            >\nRevolver            >\nFeeder              >" };
const char P_AllSteppersMenuItems[] PROGMEM = { "Invert DIR       %4S\nEndstop Trigger  %4S\nStep Delay       %4s\nMax. Speed      %5s\nMax. Speed HS   %5s\nAcceleration    %5s" };
const char P_RevolverMenuItems[] PROGMEM    = { "\nSteps Per Rev.   %5s\nHome After Feed  %4s\nReset Bef. Feed  %4s" };
const char P_FeederMenuItems[] PROGMEM      = { "\nSteps Per MM    %5s\nEnable Chunks    %4S\nFeed Chunks      %4s\nInsert Length    %5s\nInsert Speed     %5s\nReinforce Len.  %5s" };
const char P_SelectorMenuItems[] PROGMEM    = { "\nSteps Per MM    %5s" };
#endif
const char P_ConfigWriteSuccess[] PROGMEM   = { "Config success-\nfully written." };
const char P_ConfigWriteFail[] PROGMEM      = { "Config write failed!\nPlease check SD-Card." };

const char P_RunningTest[] PROGMEM          = { "Starting\n\n%s" };
const char P_TestFailed[] PROGMEM           = { "Failed to open\n\n%s" };
const char P_RunningCmd[] PROGMEM           = { "Running loop %ld" };
const char P_CmdLoop[] PROGMEM              = { "CMD: %-7ld T%d" };
const char P_ToolChanges[] PROGMEM          = { "Tool change: %5ld" };
const char P_TestTime[] PROGMEM             = { "Elapsed: %3d:%02d:%02d" };
const char P_ButtonToStop[] PROGMEM         = { "Press Button To Stop" };

const char P_SD_ReadingConfig[] PROGMEM = { "Reading config..." };
const char P_SD_InitError[] PROGMEM     = { "SD-Card not ready!" };
const char P_SD_NoConfig[] PROGMEM      = { "No config file found!" };

const char P_Ok[] PROGMEM             = { "ok\n" };
const char P_Start[] PROGMEM          = { "start\n" };
const char P_Error[] PROGMEM          = { "Error: %s\n" };
const char P_UnknownCmd[] PROGMEM     = { "Unknown command:" };
const char P_AlreadySaved[] PROGMEM   = { "Already saved.\n" };
const char P_GVersion[] PROGMEM       = { "FIRMWARE_NAME: Smart.Multi.Filament.Feeder (SMuFF) FIRMWARE_VERSION: %s ELECTRONICS: %s DATE: %s MODE: %s\n" };
const char P_TResponse[] PROGMEM      = { "T%d\n" };
const char P_GResponse[] PROGMEM      = { "G%d\n" };
const char P_MResponse[] PROGMEM      = { "M%d\n" };
const char P_M250Response[] PROGMEM   = { "M250 C%d\n" };

const char P_SelectorPos[] PROGMEM    = { "Selector position = %ld\n" };
const char P_RevolverPos[] PROGMEM    = { "Revolver position = %ld\n" };
const char P_FeederPos[] PROGMEM      = { "Feeder position = %ld\n" };
const char P_ToolSelected[] PROGMEM   = { "Tool selected = %d\n" };
const char P_Contrast[] PROGMEM       = { "Display contrast = %d\n" };
const char P_ToolsConfig[] PROGMEM    = { "Tools configured = %d\n" };
const char P_AccelSpeed[] PROGMEM     = { "X (Selector):\t%s, D:%s\nY (Revolver):\t%s, D:%s\nZ (Feeder):\t%s, D:%s\n" };
const char P_Positions[] PROGMEM      = { "X (Selector): %s, Y (Revolver): %s, Z (Feeder): %s\n" };

const char P_CurrentTool[] PROGMEM    = {"Tool    " };
const char P_Feed[] PROGMEM           = {"Feed    " };

const char P_NoTool [] PROGMEM        = { "No tool set.\n" };
const char P_Aborting [] PROGMEM      = { "Aborting..."};
const char P_FeederJammed [] PROGMEM  = { "Feeder is jammed.\n" };
const char P_JamCleared [] PROGMEM    = { "Feeder Jam has\nbeen reset." };
const char P_ToolAlreadySet [] PROGMEM= { "Tool already set." };

const char P_WrongFormat [] PROGMEM   = { "Wrong format. Use Bdd:dd:dd...\n" };

const char P_NoPrusa [] PROGMEM       = { "Prusa MMU2 mode was not configured." };
const char P_PMMU_Title [] PROGMEM    = { "Waiting..." };
const char P_PMMU_Wait [] PROGMEM     = { "Please click the" };
const char P_PMMU_WaitAdd [] PROGMEM  = { "encoder button" };

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
  "M201\t-\tSet max acceleration\n" \
  "M203\t-\tSet max feedrate\n" \
  "M205\t-\tSet advanced options\n" \
  "M206\t-\tSet offsets\n" \
  "M250\t-\tLCD contrast\n" \
  "M300\t-\tBeep\n" \
  "M500\t-\tSave settings\n" \
  "M503\t-\tReport settings\n" \
  "M575\t-\tSet serial port baudrate\n" \
  "M700\t-\tLoad filament\n" \
  "M701\t-\tUnload filament\n" \
  "M999\t-\tReset\n" \
  "M2000\t-\tText to decimal\n" \
  "M2001\t-\tDecimal to text\n"};

                             
#endif
