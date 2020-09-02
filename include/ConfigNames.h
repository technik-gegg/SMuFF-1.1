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

// used for the JSON parametser and M205 GCode command
const char* selector            = "Selector";
const char* revolver            = "Revolver";
const char* feeder              = "Feeder";
const char* toolCount           = "ToolCount";
const char* serial0Baudrate     = "Serial0Baudrate";
const char* serial1Baudrate     = "Serial1Baudrate";
const char* serial2Baudrate     = "Serial2Baudrate";
const char* serial3Baudrate     = "Serial3Baudrate";
const char* bowdenLength        = "BowdenLength";
const char* selectorDist        = "SelectorDist";
const char* contrast            = "LCDContrast";
const char* i2cAdr              = "I2CAddress";
const char* autoClose           = "MenuAutoClose";
const char* fanSpeed            = "FanSpeed";
const char* psTimeout           = "PowerSaveTimeout";
const char* sendAction          = "SendActionCmds";
const char* emulatePrusa        = "EmulatePrusa";
const char* unloadCommand       = "UnloadCommand";
const char* hasPanelDue         = "HasPanelDue";
const char* servoMinPwm         = "ServoMinPwm";
const char* servoMaxPwm         = "ServoMaxPwm";
const char* periodicalStats     = "SendPeriodicalStats";
const char* wipeSequence        = "WipeSequence";
const char* backlightColor      = "BacklightColor";
const char* encoderTicks        = "EncoderTicks";
const char* maxSpeed            = "MaxSpeed";
const char* acceleration        = "Acceleration";
const char* invertDir           = "InvertDir";
const char* endstopTrig         = "EndstopTrigger";
const char* stepDelay           = "StepDelay";
const char* stepsPerMillimeter  = "StepsPerMillimeter";
const char* offset              = "Offset";
const char* power               = "Power";
const char* mode                = "Mode";
const char* rsense              = "RSense";
const char* msteps              = "Microsteps";
const char* stall               = "Stall";
const char* cstepmin            = "CoolStepMin";
const char* cstepmax            = "CoolStepMax";
const char* cstepdown           = "CoolStepDown";
const char* drvrAdr             = "DriverAddress";
const char* toff                = "TOff";
const char* stopOnStall         = "StopOnStall";
const char* maxStallCount       = "MaxStallCount";
const char* spacing             = "Spacing";
const char* externalControl     = "ExternalControl";
const char* insertSpeed         = "InsertSpeed";
const char* reinforceLength     = "ReinforceLength";
const char* unloadRetract       = "UnloadRetract";
const char* unloadPushback      = "UnloadPushback";
const char* pushbackDelay       = "PushbackDelay";
const char* enableChunks        = "EnableChunks";
const char* feedChunks          = "FeedChunks";
const char* insertLength        = "InsertLength";
const char* duetLaser           = "DuetLaser";
const char* sharedStepper       = "SharedStepper";
const char* wiggle              = "Wiggle";
const char* useServo            = "UseServo";
const char* servoOffPos         = "ServoOffPos";
const char* servoOnPos          = "ServoOnPos";
const char* servo1Cycles        = "Servo1Cycles";
const char* servo2Cycles        = "Servo2Cycles";
const char* resetBeforeFeed     = "ResetBeforeFeed";
const char* homeAfterFeed       = "HomeAfterFeed";
const char* stepsPerRevolution  = "StepsPerRevolution";
const char* material            = "Material";
const char* materials           = "Materials";

#endif