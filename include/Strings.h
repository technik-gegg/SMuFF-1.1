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

#if !defined (__ESP32__) && !defined(__NXP__)
#include <avr/pgmspace.h>
#endif
 
#define SD_ERR_INIT           1
#define SD_ERR_NOCONFIG       2
#define SD_READING_CONFIG     0

#if defined (__STM32F1__) || defined (__ESP32__)
const char P_MenuItemBack [] PROGMEM        = { "\u25c0 BACK\n" };
const char P_MenuItemSeparator [] PROGMEM   = { "\u00ad\u00ad\u00ad\u00ad\u00ad\u00ad\u00ad\u00ad\u00ad\u00ad\u00ad\u00ad\u00ad\u00ad\u00ad\u00ad\u00ad\u00ad\u00ad\u00ad\u00ad\u00ad\n"};
const char P_MenuItems [] PROGMEM           = { "Home All\nMotors %s\n%sTool Maint. %s\nReset Feeder Jam\nLoad Filament\nUnload Filament\n%s%s" };
const char P_MenuItemsDefault[] PROGMEM     = { "%sSwap Tools \u25b8\nStatus Info \u25b8\n%sSettings \u25b8\n%sTestrun \u25b8" };
#if !defined(SMUFF_V5)
const char P_MenuItemsDefault[] PROGMEM     = { "Settings \u25b8\n%sTestrun \u25b8" };
#endif
const char P_StatusInfoMenuItems [] PROGMEM = { "Duet Laser Sensor   \u25b8%s%s%s" };
const char P_StatusInfoMenuItemsX [] PROGMEM= { "\nTMC Driver X        \u25b8" };
const char P_StatusInfoMenuItemsY [] PROGMEM= { "\nTMC Driver Y        \u25b8" };
const char P_StatusInfoMenuItemsZ [] PROGMEM= { "\nTMC Driver Z        \u25b8" };
                                                  
#else
const char P_MenuItemBack [] PROGMEM        = { "< BACK\n" };
const char P_MenuItemSeparator [] PROGMEM   = { "-----\n"};
const char P_MenuItems [] PROGMEM           = { "Home All\nMotors %s\nReset Feeder Jam\nSwap Tools >\nLoad Filament\nUnload Filament\nOffsets >" };
//const char P_MenuItems [] PROGMEM           = { "Home All\nMotors %s\nReset Feeder Jam\nSwap Tools >\nLoad Filament\nUnload Filament\n%s%S%S" };
const char P_MenuItemsDefault[] PROGMEM     = { "Settings >" };
const char P_OfsMenuItems [] PROGMEM        = { "Selector         %4s\nRevolver        %5s" };
#endif
const char P_MenuItemsServo [] PROGMEM      = { "%s Servo\n" };
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
const char P_RemoveMaterial[] PROGMEM       = { "Please remove material!" };
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
#if defined (__STM32F1__) || defined (__ESP32__)
const char P_SettingsMenuItems[] PROGMEM    = { "Tool Count      %5s\nBowden Length   %5s\nSelector Dist.  %5s\n%sOptions            %4s\nBaudrates          %4s\nSteppers/Servo     %4s\nDisplay            %4s\n%s\u25b9 SAVE TO SD-CARD \u25c3" };
#else
const char P_SettingsMenuItems[] PROGMEM    = { "Tool Count      %5s\nBowden Length   %5s\nSelector Dist.  %5s\n%sOptions            %4S\nBaudrates           %s\nSteppers            %s\n%S> SAVE TO SD-CARD <" };
#endif
const char P_Off[] PROGMEM                  = { "OFF" };
const char P_On[] PROGMEM                   = { "ON" };
const char P_Yes[] PROGMEM                  = { "Yes" };
const char P_No[] PROGMEM                   = { "No" };
const char P_No_[] PROGMEM                  = { "   " };
const char P_Unknown[] PROGMEM              = { "---" };
const char P_High[] PROGMEM                 = { "HIGH" };
const char P_Low[] PROGMEM                  = { "LOW" };
const char P_Open[] PROGMEM                 = { "OPEN" };
const char P_Close[] PROGMEM                = { "CLOSE" };
const char P_None[] PROGMEM                 = { "None" };
const char P_Stealth[] PROGMEM              = { "Stealth" };
const char P_Spread[] PROGMEM               = { "Spread" };

const char P_ToolCount[] PROGMEM            = { "# of tools:" };
const char P_InMillimeter[] PROGMEM         = { "in mm:" };
const char P_InSeconds[] PROGMEM            = { "in seconds:" };
const char P_InPercent[] PROGMEM            = { "in percent:" };
const char P_InValue[] PROGMEM              = { "as value:" };
const char P_YesNo[] PROGMEM                = { "yes / no:" };
const char P_Baud[] PROGMEM                 = { "baudrate:" };
const char P_Color[] PROGMEM                = { "color:" };
const char P_InMicroseconds[] PROGMEM       = { "in uS:" };
const char P_InTicks[] PROGMEM              = { "in ticks:" };
const char P_InSteps[] PROGMEM              = { "in steps:" };
const char P_TriggerOn[] PROGMEM            = { "on:" };
const char P_OpenPos[] PROGMEM              = { "open @:" };
const char P_ClosedPos[] PROGMEM            = { "closed @:" };
const char P_ServoCycles[] PROGMEM          = { "cycles:" };
const char P_NoOfChunks[] PROGMEM           = { "# of chunks:" };
const char P_DriverMode[] PROGMEM           = { "drvr. mode:" };
const char P_InMilliAmpere[] PROGMEM        = { "in mA:" };
const char P_InOhm[] PROGMEM                = { "in ohm:" };
const char P_Threshold[] PROGMEM            = { "threshold:" };
const char P_Min[] PROGMEM                  = { "min.:" };
const char P_Max[] PROGMEM                  = { "max.:" };
const char P_Down[] PROGMEM                 = { "down:" };
const char P_Address[] PROGMEM              = { "address:" };
const char P_Microsteps[] PROGMEM           = { "microsteps:" };
const char P_Value[] PROGMEM                = { "value:" };
const char P_MicrostepItems[] PROGMEM       = { "1\n2\n4\n8\n16\n32\n64\n128" };
const char P_TMCModeItems[] PROGMEM         = { "OFF\nUART\nSPI" };
const char P_BaudMenuItems[] PROGMEM        = { "USB-Serial     %6s\n1st Serial     %6s\n2nd Serial     %6s\n3rd Serial     %6s" };
const char P_Baudrates[] PROGMEM            = { "4800\n9600\n19200\n38400\n56700\n115200\n230400" };
const char P_Colors[] PROGMEM               = { "Black\nRed\nGreen\nBlue\nCyan\nMagenta\nYellow\nWhite" };
const char P_PanelDueOptions[] PROGMEM      = { "None\nSer.2\nSer.3" };

#if defined (__STM32F1__) || defined (__ESP32__)
const char P_SteppersMenuItems[] PROGMEM    = { "Selector            %2s\nFeeder              %2s\n%-8s            %2s" };
const char P_AllSteppersMenuItems[] PROGMEM = { "TMC Settings    %4s\u25b8\nInvert DIR       %4s\nEndstop Trigger  %4s\nStep Delay       %4s\nMax. Speed      %5s\nMax. Speed HS   %5s\nAcceleration    %5s" };
const char P_RevolverMenuItems[] PROGMEM    = { "\n1st Tool Offset %5s\nSteps per Rev.  %5s\nHome After Feed  %4s\nReset Bef. Feed  %4s\nWiggle           %4s\nUse Lid Servo    %4s\nLid Servo open  %5s" };
const char P_FeederMenuItems[] PROGMEM      = { "\nSteps per mm    %5s\nEnable Chunks    %4s\nFeed Chunks      %4s\nInsert Length    %5s\nInsert Speed     %5s\nReinforce Len.  %5s\nExternal Ctrl.   %4s\nIs Shared        %4s" };
const char P_SelectorMenuItems[] PROGMEM    = { "\n1st Tool Offset %5s\nSteps per mm    %5s" };
const char P_ServoMenuItems[] PROGMEM       = { "Home After Feed  %4s\nReset Bef. Feed  %4s\nUse Lid Servo    %4s\nLid Servo Open  %5s\nLid Servo Close %5s\nWiper Servo Cycles%3s\nLid Servo Cycles  %3s" };
const char P_TMCMenuItems[] PROGMEM         = { "Mode             %4s\nPower            %4s\nR-Sense          %4s\nMicrosteps       %4s\nStall Thrs.     %5s\nCoolStep min.    %4s\nCoolStep max.    %4s\nCoolStep down    %4s\nDriver Address   %4s\nTOff             %4s" };
const char P_DisplayMenuItems[] PROGMEM     = { "Screen Timeout   %4s\nLCD Contrast     %4s\nBacklight    %8s\nEncoder Ticks    %4s" };
#else
const char P_SteppersMenuItems[] PROGMEM    = { "Selector            >\nRevolver            >\nFeeder              >" };
const char P_AllSteppersMenuItems[] PROGMEM = { "Invert DIR       %4S\nEndstop Trigger  %4S\nStep Delay       %4s\nMax. Speed      %5s\nMax. Speed HS   %5s\nAcceleration    %5s" };
const char P_RevolverMenuItems[] PROGMEM    = { "\nSteps per Rev.  %5s\nHome After Feed  %4s\nReset Bef. Feed  %4s\nWiggle           %4s\nUse Servo        %4s\nServo open      %5s\nServo closed    %5s\nServo cycles    %5s" };
const char P_FeederMenuItems[] PROGMEM      = { "\nSteps per MM    %5s\nEnable Chunks    %4S\nFeed Chunks      %4s\nInsert Length    %5s\nInsert Speed     %5s\nReinforce Len.  %5s" };
const char P_SelectorMenuItems[] PROGMEM    = { "\nSteps per MM    %5s" };
#endif
#if defined (__STM32F1__) || defined (__ESP32__)
const char P_OptionsMenuItems[] PROGMEM    = { "Menu Auto Close  %4s\nFan Speed       %5s\nPrusa MMU2 Emul. %4s\nSend Status Info %4s\nUse Laser Sensor %4s\nPanelDue Port   %5s" };
#else
const char P_OptionsMenuItems[] PROGMEM    = { "Menu Auto Close  %4s\nFan Speed       %5s\nPower Save Time %5s\nPrusa MMU2 Emul. %4S\n" };
#endif

const char P_ConfigWriteSuccess[] PROGMEM   = { "Config success-\nfully written." };
const char P_ConfigWriteFail[] PROGMEM      = { "Config write failed!\nPlease check SD-Card." };

const char P_RunningTest[] PROGMEM          = { "Starting\n\n%s" };
const char P_TestFailed[] PROGMEM           = { "Failed to open\n\n%s" };
const char P_RunningCmd[] PROGMEM           = { "Running loop %ld" };
const char P_CmdLoop[] PROGMEM              = { "CMD: %-7ld T%d" };
const char P_ToolChanges[] PROGMEM          = { "Tool change: %5ld" };
const char P_TestTime[] PROGMEM             = { "Elapsed: %3d:%02d:%02d" };
const char P_FeederErrors[] PROGMEM         = { "Feed errors: %5ld" };
const char P_ButtonToStop[] PROGMEM         = { "Press Button To Stop" };
const char P_DuetLSStat[] PROGMEM           = { "Duet3D LS Status" };
const char P_DuetLSDisabled[] PROGMEM       = { "Duet3D LaserSensor\nnot enabled!" };
const char P_DuetLSData[] PROGMEM           = { "Position:%s%6smm\nQ/B/S:    %3d/%3d/%3d\nSwitch:           %3s\nError:      %4s\nVersion:          %3d" };
const char P_PanelDuePort[] PROGMEM         = { "Use Port:" };

const char P_SD_ReadingConfig[] PROGMEM = { "Reading config..." };
const char P_SD_InitError[] PROGMEM     = { "SD-Card not ready!" };
const char P_SD_NoConfig[] PROGMEM      = { "No config file found!" };

const char P_Ok[] PROGMEM             = { "ok\n" };
const char P_Start[] PROGMEM          = { "start\n" };
const char P_Error[] PROGMEM          = { "error: %s\n" };
const char P_Echo[] PROGMEM           = { "echo: %s\n" };
const char P_UnknownCmd[] PROGMEM     = { "Unknown command: %s" };
const char P_UnknownParam[] PROGMEM   = { "Unknown parameter '%s'\n" };
const char P_NoValue[] PROGMEM        = { "Value ('Sn') missing for parameter '%s'\n" };
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
const char P_AccelSpeed[] PROGMEM     = { "X (Selector):\t%s, D:%s\nY (Revolver):\t%s, D:%s\nZ (Feeder):\t%s%s, D:%s\n" };
const char P_Positions[] PROGMEM      = { "X (Selector): %s, Y (Revolver): %s, Z (Feeder): %s\n" };

const char P_CurrentTool[] PROGMEM    = {"Tool    " };
const char P_Feed[] PROGMEM           = {"Feed    " };
const char P_External[] PROGMEM       = {"EXT." };
const char P_Internal[] PROGMEM       = {"INT." };

const char P_NoTool [] PROGMEM        = { "No tool set.\n" };
const char P_Aborting [] PROGMEM      = { "Aborting..."};
const char P_FeederJammed [] PROGMEM  = { "Feeder is jammed.\n" };
const char P_JamCleared [] PROGMEM    = { "Feeder Jam has\nbeen reset." };
const char P_ToolAlreadySet [] PROGMEM= { "Tool already set." };
const char P_WrongTool [] PROGMEM     = { "Tool index %d invalid." };
const char P_ActionMsg [] PROGMEM     = { "Controller says:\n%s" };

const char P_WrongFormat [] PROGMEM   = { "Wrong format. Use Bdd:dd:dd...\n" };

const char P_NoPrusa [] PROGMEM       = { "Prusa MMU2 mode is not configured." };
const char P_PMMU_Title [] PROGMEM    = { "Waiting..." };
const char P_PMMU_Wait [] PROGMEM     = { "Please click the" };
const char P_PMMU_WaitAdd [] PROGMEM  = { "encoder button" };

const char P_StepperMode [] PROGMEM   = { "%s: %s" };
const char P_StepperNotCfg [] PROGMEM = { "Driver not in use!" };
const char P_StepperUART [] PROGMEM   = { "UART" };
const char P_StepperPDN [] PROGMEM    = { "PDN" };

const char P_TMC_Setup00[] PROGMEM  = { "                     -X-       -Y-       -Z-       -E-\n" };
const char P_TMC_Setup01[] PROGMEM  = { "IC Version         " };
const char P_TMC_Setup02[] PROGMEM  = { "Enabled            " };
const char P_TMC_Setup03[] PROGMEM  = { "Current Config.    " };
const char P_TMC_Setup03a[] PROGMEM = { "RMS Current        " };
const char P_TMC_Setup04[] PROGMEM  = { "Microsteps         " };
const char P_TMC_Setup05[] PROGMEM  = { "Off Time           " };
const char P_TMC_Setup06[] PROGMEM  = { "Blank Time         " };
const char P_TMC_Setup07[] PROGMEM  = { "UART Mode          " };
const char P_TMC_Setup08[] PROGMEM  = { "MS2 / MS1          " };
const char P_TMC_Setup09[] PROGMEM  = { "Diag               " };
const char P_TMC_Setup10[] PROGMEM  = { "StallGuard THRS    " };
const char P_TMC_Setup11[] PROGMEM  = { "StallGuard Result  " };
const char P_TMC_Setup12[] PROGMEM  = { "CoolStep Min.      " };
const char P_TMC_Setup13[] PROGMEM  = { "CoolStep Max.      " };
const char P_TMC_Setup14[] PROGMEM  = { "CoolStep Down      " };
const char P_TMC_Setup15[] PROGMEM  = { "CoolStep THRS  " };
const char P_TMC_Status00[] PROGMEM = { "                   -------------= STATUS =-------------\n" };
const char P_TMC_Status01[] PROGMEM = { "Mode               "};
const char P_TMC_Status02[] PROGMEM = { "Stand Still        "};
const char P_TMC_Status03[] PROGMEM = { "Ph. A open         "};
const char P_TMC_Status04[] PROGMEM = { "Ph. B open         "};
const char P_TMC_Status05[] PROGMEM = { "Ph. A short        "};
const char P_TMC_Status06[] PROGMEM = { "Ph. B short        "};
const char P_TMC_Status07[] PROGMEM = { "Ph. A short MOS    "};
const char P_TMC_Status08[] PROGMEM = { "Ph. B short MOS    "};
const char P_TMC_Status09[] PROGMEM = { "Overtemp. Warning  "};
const char P_TMC_Status10[] PROGMEM = { "Overtemperature    "};
//                                       .....................\n.....................\n.....................\n.....................\n.....................
const char P_TMC_Status[] PROGMEM   = { "Mode          %7s\nRMS Cur.       %4dmA\nPh. A/B open  %3s/%3s\nPh. A/B short %3s/%3s\nOvertemperature   %3s"};

const char P_GCmds[] PROGMEM = { 
  "G0\t-\tMove\n" \
  "G1\t-\tMove\n" \
  "G4\t-\tDwell\n" \
  "G12\t-\tWipe filament\n" \
  "G28\t-\tHome\n" \
  "G90\t-\tAbsolute positioning\n" \
  "G91\t-\tRelative positioning\n" };

const char P_MCmds[] PROGMEM = { 
  "M17\t-\tSwitch feeder stepper control (Relay)\n" \
  "M18\t-\tMotors off\n" \
  "M84\t-\tMotors off\n" \
  "M20\t-\tList SD-Card\n" \
  "M42\t-\tSet pin state\n" \
  "M98\t-\tExecute test run\n" \
  "M106\t-\tFan on\n" \
  "M107\t-\tFan off\n" \
  "M114\t-\tReport current positions\n" \
  "M115\t-\tReport version\n" \
  "M117\t-\tDisplay message\n" \
  "M119\t-\tReport endstop status\n" \
  "M122\t-\tReport TMC driver status\n" \
  "M150\t-\tSet FastLED color\n" \
  "M201\t-\tSet max acceleration\n" \
  "M203\t-\tSet max feedrate\n" \
  "M205\t-\tSet advanced options\n" \
  "M206\t-\tSet offsets\n" \
  "M250\t-\tLCD contrast\n" \
  "M412\t-\tJAM detection on/off\n" \
  "M300\t-\tPlay a tone or a tune\n" \
  "M350\t-\tSet motor microstepping\n" \
  "M500\t-\tSave settings\n" \
  "M503\t-\tReport settings\n" \
  "M569\t-\tSet SpreadCycle on/off\n" \
  "M575\t-\tSet serial port baudrate\n" \
  "M700\t-\tLoad filament\n" \
  "M701\t-\tUnload filament\n" \
  "M906\t-\tSet motor current\n" \
  "M914\t-\tSet motor stall sensitivity\n" \
  "M999\t-\tReset\n" \
  "M2000\t-\tText to decimal\n" \
  "M2001\t-\tDecimal to text\n"};
                             
#endif
