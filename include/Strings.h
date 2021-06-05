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

#if !defined (__ESP32__) && !defined(__NXP__)
#include <avr/pgmspace.h>
#endif

#define SD_ERR_INIT           1
#define SD_ERR_NOCONFIG       2
#define SD_READING_CONFIG     0
#define SD_READING_MATERIALS  3
#define SD_READING_TMC        4
#define SD_READING_SERVOS     5
#define SD_READING_PURGES     6
#define SD_READING_STEPPERS   7

const char P_MenuItemBack [] PROGMEM        = { "< BACK\n" };
const char P_MenuItemSeparator [] PROGMEM   = { "\035\n"};
/*
  Menus and Options have moved into files contained in the menus/options folder!
*/
const char P_MnuMain0 [] PROGMEM            = { "main0" };
const char P_MnuMain1 [] PROGMEM            = { "main1" };
const char P_MnuMain2 [] PROGMEM            = { "main2" };
const char P_MnuMain3 [] PROGMEM            = { "main3" };
const char P_MnuMain4 [] PROGMEM            = { "main4" };
const char P_MnuSettings [] PROGMEM         = { "settings" };
const char P_MnuBaudrates [] PROGMEM        = { "baudrates" };
const char P_MnuStatus [] PROGMEM           = { "status" };
const char P_MnuOptions [] PROGMEM          = { "options" };
const char P_MnuTmc [] PROGMEM              = { "tmc" };
const char P_MnuServo [] PROGMEM            = { "servo" };
const char P_MnuRevolver [] PROGMEM         = { "revolver" };
const char P_MnuFeeder [] PROGMEM           = { "feeder" };
const char P_MnuSelector [] PROGMEM         = { "selector" };
const char P_MnuDisplay [] PROGMEM          = { "display" };
const char P_MnuSteppers [] PROGMEM         = { "steppers1" };
const char P_MnuSteppersServo [] PROGMEM    = { "steppers0" };
const char P_MnuPurge [] PROGMEM            = { "purge" };

const char P_OptColors [] PROGMEM           = { "colors" };
const char P_OptPanelDue [] PROGMEM         = { "paneldue" };
const char P_OptBaudrates [] PROGMEM        = { "baudrates" };
const char P_OptTmcModes [] PROGMEM         = { "tmc-modes" };
const char P_OptMicrosteps [] PROGMEM       = { "microsteps" };
const char P_OptMS3States [] PROGMEM        = { "ms3-states" };

const char P_OkButtonOnly [] PROGMEM        = { " Ok " };
const char P_CancelButtonOnly [] PROGMEM    = { " Cancel " };
const char P_OkCancelButtons [] PROGMEM     = { " Ok \n Cancel " };
const char P_CancelRetryButtons [] PROGMEM  = { " Cancel \n Retry " };
const char P_YesNoButtons [] PROGMEM        = { " Yes \n No " };

const char P_TitleWarning [] PROGMEM        = { "WARNING" };
const char P_TitleSelected [] PROGMEM       = { "TOOL SELECTED" };
const char P_TitleConfigError [] PROGMEM    = { "CONFIG FAIL" };
const char P_TitleMainMenu [] PROGMEM       = { "Main Menu" };
const char P_TitleToolsMenu [] PROGMEM      = { "Tool Selection" };

const char P_FeederLoaded [] PROGMEM        = { "Feeder is loaded!\n" };
const char P_SettingsChanged [] PROGMEM     = { "Settings have changed!\n" };

const char P_AskUnload [] PROGMEM           = { "Want me to unload\nit now?" };
const char P_AskLoad [] PROGMEM             = { "Want me to load\nit now?" };
const char P_AskSave [] PROGMEM             = { "Want me to save\nthem now?" };

const char P_RemoveMaterial[] PROGMEM       = { "Please remove material!" };
const char P_SelectedTool [] PROGMEM        = { " \n" };
const char P_CantLoad [] PROGMEM            = { "Can't load feeder." };
const char P_CantUnload [] PROGMEM          = { "Can't unload feeder." };
const char P_CheckUnit [] PROGMEM           = { "Please check unit!" };

const char P_ConfigFail1 [] PROGMEM         = { "Config file is" };
const char P_ConfigFail5 [] PROGMEM         = { "Materials file is" };
const char P_ConfigFail6 [] PROGMEM         = { "TMC config file is" };
const char P_ConfigFail7 [] PROGMEM         = { "ServoMap file is" };
const char P_ConfigFail8 [] PROGMEM         = { "Steppers file is" };
const char P_ConfigFail2 [] PROGMEM         = { "possibly corrupted,\nplease check!" };
const char P_ConfigFail3 [] PROGMEM         = { "too big,\nplease reduce content!" };
const char P_ConfigFail4 [] PROGMEM         = { "data inconsistent\nor memory failure!" };

const char P_ToolMenu [] PROGMEM            = { "Tool %d" };
const char P_SwapMenu [] PROGMEM            = { "Slot %d: T%d" };
const char P_SwapReset [] PROGMEM           = { "Reset swaps\n" };
const char P_SwapToolDialog [] PROGMEM      = { "Swap Tool %d\nwith Tool %d" };
const char P_ToolPurgeMenu [] PROGMEM       = { "%d - %s\t%d%%\n" };
const char P_ToolMaterial [] PROGMEM        = { "Tool %d: %s, %d%%\n" };

const char P_Selecting [] PROGMEM           = { "Selecting" };
const char P_Wait [] PROGMEM                = { "please wait..." };
const char P_Busy[] PROGMEM                 = { "BUSY " };
const char P_Ready[] PROGMEM                = { "READY" };
const char P_Pemu[] PROGMEM                 = { "PMMU2" };
const char P_Purging [] PROGMEM             = { "Purging" };
const char P_PurgeLen [] PROGMEM            = { "%d%% = %smm" };
const char P_PurgeCubic [] PROGMEM          = { "(= %smm3)" };
const char P_Usage [] PROGMEM               = { "USAGE:\n" };
const char P_Upload [] PROGMEM              = { "Upload in Progress" };
const char P_BytesRemain [] PROGMEM         = { "%ld %sBytes left" };

const char P_Off[] PROGMEM                  = { "OFF" };
const char P_On[] PROGMEM                   = { "ON" };
const char P_Yes[] PROGMEM                  = { "Yes" };
const char P_No[] PROGMEM                   = { "No" };
const char P_Unknown[] PROGMEM              = { "---" };
const char P_Undefined[] PROGMEM            = { "???" };
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
const char P_InMicroseconds[] PROGMEM       = { "in uS:" };
const char P_InMilliseconds[] PROGMEM       = { "in mS:" };
const char P_InTicks[] PROGMEM              = { "in ticks:" };
const char P_InMMS[] PROGMEM                = { "in mm/s:" };
const char P_InSteps[] PROGMEM              = { "in steps:" };
const char P_InMilliAmpere[] PROGMEM        = { "in mA:" };
const char P_InOhm[] PROGMEM                = { "in ohm:" };
const char P_InBPM[] PROGMEM                = { "in BPM:" };

const char P_InValue[] PROGMEM              = { "as value:" };
const char P_YesNo[] PROGMEM                = { "yes / no:" };
const char P_Baud[] PROGMEM                 = { "baudrate:" };
const char P_Color[] PROGMEM                = { "color:" };
const char P_TriggerOn[] PROGMEM            = { "on:" };
const char P_OpenPos[] PROGMEM              = { "opened @:" };
const char P_ClosedPos[] PROGMEM            = { "closed @:" };
const char P_ServoCycles[] PROGMEM          = { "cycles:" };
const char P_NoOfChunks[] PROGMEM           = { "# of chunks:" };
const char P_DriverMode[] PROGMEM           = { "drvr. mode:" };
const char P_Threshold[] PROGMEM            = { "threshold:" };
const char P_Min[] PROGMEM                  = { "min.:" };
const char P_Max[] PROGMEM                  = { "max.:" };
const char P_Down[] PROGMEM                 = { "down:" };
const char P_Address[] PROGMEM              = { "address:" };
const char P_Microsteps[] PROGMEM           = { "microsteps:" };
const char P_Value[] PROGMEM                = { "value:" };
const char P_MS3State[] PROGMEM             = { "state:" };

const char P_M503S1[] PROGMEM               = { "\n/* Basic */\n" };
const char P_M503S2[] PROGMEM               = { "\n/* Steppers */\n" };
const char P_M503S3[] PROGMEM               = { "\n/* TMC Driver */\n" };
const char P_M503S4[] PROGMEM               = { "\n/* Servo mapping */\n" };
const char P_M503S5[] PROGMEM               = { "\n/* Materials */\n" };
const char P_M503S6[] PROGMEM               = { "\n/* Tool swaps */\n" };
const char P_M503S7[] PROGMEM               = { "\n/* Revolver mapping */\n" };
const char P_M503S8[] PROGMEM               = { "\n/* Feed State */\n" };

const char P_ConfigWriteSuccess[] PROGMEM   = { "Config\nsuccessfully\nwritten!" };
const char P_ConfigWriteFail[] PROGMEM      = { "Saving failed!\nPlease check SD-Card." };

const char P_SDCardRemoved[] PROGMEM        = { "SD-Card removed.\nPlease re-insert!" };

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
const char P_PanelDuePort[] PROGMEM         = { "On Port:" };
const char P_FreeMemory[] PROGMEM           = { "Estimated Free Memory: %d Bytes\n" };

const char P_SD_Reading[] PROGMEM           = { "Reading %s..." };
const char P_SD_ReadingConfig[] PROGMEM     = { "Config" };
const char P_SD_ReadingMaterials[] PROGMEM  = { "Materials" };
const char P_SD_ReadingTmc[] PROGMEM        = { "TMC" };
const char P_SD_ReadingServos[] PROGMEM     = { "Servos" };
const char P_SD_ReadingPurges[] PROGMEM     = { "Purges" };
const char P_SD_ReadingSteppers[] PROGMEM   = { "Steppers" };
const char P_SD_InitError[] PROGMEM         = { "SD-Card not ready!" };
const char P_SD_NoConfig[] PROGMEM          = { "No config file found!" };

const char P_Ok[] PROGMEM                   = { "ok\n" };
const char P_Start[] PROGMEM                = { "start\n" };
const char P_Error[] PROGMEM                = { "error: %s\n" };
const char P_Echo[] PROGMEM                 = { "echo: %s\n" };
const char P_UnknownCmd[] PROGMEM           = { "Unknown command: %s" };
const char P_UnknownParam[] PROGMEM         = { "Unknown parameter '%s'\n" };
const char P_NoValue[] PROGMEM              = { "Value ('Sn') missing for parameter '%s'\n" };
const char P_GVersion[] PROGMEM             = { "FIRMWARE_NAME: Smart.Multi.Filament.Feeder (SMuFF) FIRMWARE_VERSION: %s ELECTRONICS: %s DATE: %s MODE: %s OPTIONS: %s\n" };
const char P_TResponse[] PROGMEM            = { "T%d\n" };
const char P_GResponse[] PROGMEM            = { "G%d\n" };
const char P_MResponse[] PROGMEM            = { "M%d\n" };
const char P_M250Response[] PROGMEM         = { "M250 C%d\n" };
const char P_XlateSpeedResponse[] PROGMEM   = { "echo: speed: %s\n" };
const char P_Tool[] PROGMEM                 = { "T%d" };
const char P_AccelSpeedMms[] PROGMEM        = { "X (S):\t%d mm/s, Delay: %d\nY (R):\t%d mm/s, Delay: %d\nZ (F):\t%d mm/s%s, Delay: %d, Insert: %d mm/s\n" };
const char P_AccelSpeedTicks[] PROGMEM      = { "X (S):\t%d ticks, Delay: %d\nY (R):\t%d ticks, Delay: %d\nZ (F):\t%d ticks%s, Delay: %d, Insert: %d ticks\n" };
const char P_SpeedAdjust[] PROGMEM          = { "X (S):\t%s\nY (R):\t%s\nZ (F):\t%s\n" };

const char P_CurrentTool[] PROGMEM          = {"Tool    " };
const char P_Feed[] PROGMEM                 = {"Feed    " };
const char P_External[] PROGMEM             = {"EXT." };
const char P_Internal[] PROGMEM             = {"INT." };

const char P_NoTool [] PROGMEM              = { "No tool set.\n" };
const char P_Aborting [] PROGMEM            = { "Aborting..."};
const char P_FeederJammed [] PROGMEM        = { "Feeder is jammed.\n" };
const char P_JamCleared [] PROGMEM          = { "Feeder Jam has\nbeen reset." };
const char P_ToolAlreadySet [] PROGMEM      = { "Tool already set." };
const char P_WrongTool [] PROGMEM           = { "Tool index %d invalid." };
const char P_ActionMsg [] PROGMEM           = { "Printer says:\n%s" };
const char P_FileNotFound [] PROGMEM        = { "File '%s' not found!" };

const char P_WrongFormat [] PROGMEM         = { "Wrong format. Use Bdd:dd:dd...\n" };
const char P_RangeError[] PROGMEM           = { "Invalid parameter value. Allowed range: " };
const char P_UseRangeI[] PROGMEM            = { "%d...%d\n" };
const char P_UseRangeF[] PROGMEM            = { "%s...%s\n" };
const char P_UseRangeL[] PROGMEM            = { "%ld...%ld\n" };

const char P_NoPrusa [] PROGMEM             = { "Prusa MMU2 mode is not configured." };
const char P_PMMU_Title [] PROGMEM          = { "Waiting..." };
const char P_PMMU_Wait [] PROGMEM           = { "Please click the" };
const char P_PMMU_WaitAdd [] PROGMEM        = { "encoder button" };

const char P_StepperMode [] PROGMEM         = { "%s: %s" };
const char P_StepperNotCfg [] PROGMEM       = { "Driver unused!" };
const char P_StepperUART [] PROGMEM         = { "UART" };
const char P_StepperPDN [] PROGMEM          = { "PDN" };
const char P_F4s [] PROGMEM                 = { "%4s      " };
const char P_F4d [] PROGMEM                 = { "%4d      " };
const char P_FL7s [] PROGMEM                = { "%-7s   " };
const char P_F8x [] PROGMEM                 = { "%8lx  " };
const char P_F33d [] PROGMEM                = { "%3d/%3d   " };

const char P_TMC_Setup00[] PROGMEM          = { "                    S/X       R/Y       F/Z\n" };
const char P_TMC_Setup01[] PROGMEM          = { "IC Version         " };
const char P_TMC_Setup02[] PROGMEM          = { "Enabled            " };
const char P_TMC_Setup03[] PROGMEM          = { "Current Config.    " };
const char P_TMC_Setup03a[] PROGMEM         = { "RMS Current        " };
const char P_TMC_Setup04[] PROGMEM          = { "Microsteps         " };
const char P_TMC_Setup05[] PROGMEM          = { "Off Time           " };
const char P_TMC_Setup06[] PROGMEM          = { "Blank Time         " };
const char P_TMC_Setup07[] PROGMEM          = { "UART Mode          " };
const char P_TMC_Setup08[] PROGMEM          = { "MS2 / MS1          " };
const char P_TMC_Setup09[] PROGMEM          = { "Diag               " };
const char P_TMC_Setup10[] PROGMEM          = { "StallGuard THRS    " };
const char P_TMC_Setup11[] PROGMEM          = { "StallGuard Result  " };
const char P_TMC_Setup12[] PROGMEM          = { "StallGuard Trigger " };
const char P_TMC_Setup13[] PROGMEM          = { "CoolStep Min.      " };
const char P_TMC_Setup14[] PROGMEM          = { "CoolStep Max.      " };
const char P_TMC_Setup15[] PROGMEM          = { "CoolStep Down      " };
const char P_TMC_Setup16[] PROGMEM          = { "CoolStep THRS  " };
const char P_TMC_Setup17[] PROGMEM          = { "PWM THRS       " };
const char P_TMC_Setup18[] PROGMEM          = { "TSTEP          " };
const char P_TMC_Setup19[] PROGMEM          = { "IRUN (mA)          " };
const char P_TMC_Setup20[] PROGMEM          = { "IHOLD (mA)         " };
const char P_TMC_Setup21[] PROGMEM          = { "IHOLD Delay        " };
const char P_TMC_Setup22[] PROGMEM          = { "TPOWERDOWN (secs.) " };
const char P_TMC_Setup23[] PROGMEM          = { "PWM Gradient    " };
const char P_TMC_Setup24[] PROGMEM          = { "PWM Ofs/ScaleSum" };

const char P_TMC_Status00[] PROGMEM         = { "                   --------= STATUS =--------\n" };
const char P_TMC_Status01[] PROGMEM         = { "Mode               "};
const char P_TMC_Status02[] PROGMEM         = { "Stand Still        "};
const char P_TMC_Status03[] PROGMEM         = { "Phase A open       "};
const char P_TMC_Status04[] PROGMEM         = { "Phase B open       "};
const char P_TMC_Status05[] PROGMEM         = { "Phase A short      "};
const char P_TMC_Status06[] PROGMEM         = { "Phase B short      "};
const char P_TMC_Status07[] PROGMEM         = { "Phase A short MOS  "};
const char P_TMC_Status08[] PROGMEM         = { "Phase B short MOS  "};
const char P_TMC_Status09[] PROGMEM         = { "Overtemp. Warning  "};
const char P_TMC_Status10[] PROGMEM         = { "Overtemperature    "};

const char P_TMC_Status0[] PROGMEM          = { "Mode: %-7s\nRMS Cur.       %4dmA\nMicrosteps        %3d\nMS1/MS2            %1d%1d\nUART mode         %3s"};
const char P_TMC_Status1[] PROGMEM          = { "Mode: %-7s\nConfig. Cur.   %4dmA\nPh. A/B open  %3s/%3s\nPh. A/B short %3s/%3s\nOvertemp.     %7s"};
const char P_TMC_StatusAll[] PROGMEM        = { "Selector (X): %s\nRevolver (Y): %s\nFeeder (Z)  : %s\n" };
const char P_OT_157[] PROGMEM               = { "> 157°C" };
const char P_OT_150[] PROGMEM               = { "> 150°C" };
const char P_OT_143[] PROGMEM               = { "> 143°C" };
const char P_OT_120[] PROGMEM               = { "> 120°C" };
const char P_MilliAmp[] PROGMEM             = { "mA" };

const char P_Action[] PROGMEM               = { "//action: %s" };
const char P_ActionWait[] PROGMEM           = { "WAIT" };
const char P_ActionCont[] PROGMEM           = { "CONTINUE" };
const char P_ActionPong[] PROGMEM           = { "PONG" };

const char P_Duet_StatusAll[] PROGMEM       = { "Selector: %s\nFeeder:   %s\n" };

const char P_TMCStatus[] PROGMEM            = { "TMCStatus" };
const char P_TMCStatusNotUsed[] PROGMEM     = { "{\"%s\": {\"%s\": %d, \"%s\": false}}\n" };
const char P_TMCKeyAxis[] PROGMEM           = { "Axis" };
const char P_TMCKeyVersion[] PROGMEM        = { "Version" };
const char P_TMCKeyInUse[] PROGMEM          = { "InUse" };
const char P_TMCKeyMode[] PROGMEM           = { "Mode"};
const char P_TMCKeyPwrCfg[] PROGMEM         = { "Power"};
const char P_TMCKeyPwrRms[] PROGMEM         = { "RMS"};
const char P_TMCKeyMS[] PROGMEM             = { "MS"};
const char P_TMCKeyAddr[] PROGMEM           = { "DrvAddr"};
const char P_TMCKeyUart[] PROGMEM           = { "UART"};
const char P_TMCKeyDiag[] PROGMEM           = { "Diag"};
const char P_TMCKeyOLA[] PROGMEM            = { "OLA"};
const char P_TMCKeyOLB[] PROGMEM            = { "OLB"};
const char P_TMCKeyS2GA[] PROGMEM           = { "S2GA"};
const char P_TMCKeyS2GB[] PROGMEM           = { "S2GB"};
const char P_TMCKeyOT[] PROGMEM             = { "OT"};

// VT-100 Escape codes
const char P_SendTermCls[] PROGMEM          = { "\033[2J" };
const char P_SendTermAt[] PROGMEM           = { "\033[%d;%dH%s" };
const char P_SendTermAttr[] PROGMEM         = { "\033[%dm" };
const char P_SendTermHome[] PROGMEM         = { "\033[H" };
const char P_SendTermCsrSave[] PROGMEM      = { "\0337" };
const char P_SendTermCsrRestore[] PROGMEM   = { "\0338" };
const char P_SendTermCsrHide[] PROGMEM      = { "\033[?25l" };
const char P_SendTermCsrShow[] PROGMEM      = { "\033[?25h" };
const char P_SendTermScroll[] PROGMEM       = { "\0337\033[2;50r\0338" };

const char P_SendTermStatus[] PROGMEM       = { "\0337\033[?25l\033[7m\033[H  \033[45m%s\033[40m%c F1: \033[45m%c\033[40m %c F2: \033[45m%c\033[40m %c \033[45m%s\033[40m %c RLY: \033[45m%s\033[40m %c DRVR: \033[45m%s  \033[40m %c PG: \033[45m%c\033[40m %c ST: \033[45m%c\033[40m %c \033[45m%s \033[0m\033[?25h\0338" };
