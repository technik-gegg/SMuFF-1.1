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

#ifndef _SMUFF_CONFIG_NAMES_H
#define _SMUFF_CONFIG_NAMES_H

// used for the JSON parameters and M205 GCode command
const char* selector PROGMEM           = "Selector";
const char* revolver PROGMEM           = "Revolver";
const char* feeder PROGMEM             = "Feeder";
const char* toolCount PROGMEM          = "ToolCount";
const char* serial0Baudrate PROGMEM    = "Serial0Baudrate";
const char* serial1Baudrate PROGMEM    = "Serial1Baudrate";
const char* serial2Baudrate PROGMEM    = "Serial2Baudrate";
const char* serial3Baudrate PROGMEM    = "Serial3Baudrate";
const char* bowdenLength PROGMEM       = "BowdenLength";
const char* selectorDist PROGMEM       = "SelectorDist";
const char* contrast PROGMEM           = "LCDContrast";
const char* i2cAdr PROGMEM             = "I2CAddress";
const char* autoClose PROGMEM          = "MenuAutoClose";
const char* fanSpeed PROGMEM           = "FanSpeed";
const char* psTimeout PROGMEM          = "PowerSaveTimeout";
const char* sendAction PROGMEM         = "SendActionCmds";
const char* emulatePrusa PROGMEM       = "EmulatePrusa";
const char* unloadCommand PROGMEM      = "UnloadCommand";
const char* hasPanelDue PROGMEM        = "HasPanelDue";
const char* servoMinPwm PROGMEM        = "ServoMinPwm";
const char* servoMaxPwm PROGMEM        = "ServoMaxPwm";
const char* periodicalStats  PROGMEM   = "SendPeriodicalStats";
const char* wipeSequence PROGMEM       = "WipeSequence";
const char* backlightColor PROGMEM     = "BacklightColor";
const char* encoderTicks PROGMEM       = "EncoderTicks";
const char* maxSpeed PROGMEM           = "MaxSpeed";
const char* accelSpeed PROGMEM         = "AccelSpeed";
const char* accelDist PROGMEM          = "AccelDistance";
const char* invertDir PROGMEM          = "InvertDir";
const char* endstopTrig PROGMEM        = "EndstopTrigger";
const char* endstopTest PROGMEM        = "EndstopTest";
const char* stepDelay PROGMEM          = "StepDelay";
const char* stepsPerMillimeter PROGMEM = "StepsPerMillimeter";
const char* offset PROGMEM             = "Offset";
const char* power PROGMEM              = "Power";
const char* mode PROGMEM               = "Mode";
const char* tmode PROGMEM              = "TMode";
const char* rsense PROGMEM             = "RSense";
const char* msteps PROGMEM             = "Microsteps";
const char* stall PROGMEM              = "Stall";
const char* cstepmin PROGMEM           = "CoolStepMin";
const char* cstepmax PROGMEM           = "CoolStepMax";
const char* cstepdown PROGMEM          = "CoolStepDown";
const char* drvrAdr PROGMEM            = "DriverAddress";
const char* toff PROGMEM               = "TOff";
const char* stopOnStall PROGMEM        = "StopOnStall";
const char* maxStallCount PROGMEM      = "MaxStallCount";
const char* spacing PROGMEM            = "Spacing";
const char* externalControl PROGMEM    = "ExternalControl";
const char* insertSpeed PROGMEM        = "InsertSpeed";
const char* reinforceLength PROGMEM    = "ReinforceLength";
const char* unloadRetract PROGMEM      = "UnloadRetract";
const char* unloadPushback PROGMEM     = "UnloadPushback";
const char* pushbackDelay PROGMEM      = "PushbackDelay";
const char* enableChunks PROGMEM       = "EnableChunks";
const char* feedChunks PROGMEM         = "FeedChunks";
const char* insertLength PROGMEM       = "InsertLength";
const char* duetLaser PROGMEM          = "DuetLaser";
const char* sharedStepper PROGMEM      = "SharedStepper";
const char* wiggle PROGMEM             = "Wiggle";
const char* useServo PROGMEM           = "UseServo";
const char* servoOffPos PROGMEM        = "ServoOffPos";
const char* servoOnPos PROGMEM         = "ServoOnPos";
const char* servo1Cycles PROGMEM       = "Servo1Cycles";
const char* servo2Cycles PROGMEM       = "Servo2Cycles";
const char* resetBeforeFeed PROGMEM    = "ResetBeforeFeed";
const char* homeAfterFeed PROGMEM      = "HomeAfterFeed";
const char* stepsPerRevolution PROGMEM = "StepsPerRevolution";
const char* material PROGMEM           = "Material";
const char* materials PROGMEM          = "Materials";
const char* servoOutput PROGMEM        = "Output";
const char* servoClosed PROGMEM        = "Close";
const char* servoOpened PROGMEM        = "Open";
const char* wiper PROGMEM              = "Wiper";
const char* tool PROGMEM               = "Tool";
const char* positions PROGMEM          = "Positions";
const char* swaps PROGMEM              = "SwapTools";
const char* lButtonDown PROGMEM        = "LBtnDown";
const char* lButtonHold PROGMEM        = "LBtnHold";
const char* rButtonDown PROGMEM        = "RBtnDown";
const char* rButtonHold PROGMEM        = "RBtnHold";
const char* speedsInMMS PROGMEM        = "SpeedsInMms";
const char* ms3Config PROGMEM          = "MS3";
const char* motDelay PROGMEM           = "MDly";

#endif