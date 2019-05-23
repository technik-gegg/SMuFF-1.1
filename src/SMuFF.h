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

#ifndef _SMUFF_H
#define _SMUFF_H

#define DEBUG 1

#include "Config.h"
#include "Strings.h"
#include "GCodes.h"
#include "Encoder.h"
#include <EEPROM.h>
#include <Wire.h>
#include <SD.h>
#include "U8g2lib.h"
#include "MemoryFree.h"

#define FEEDER_SIGNAL     1
#define SELECTOR_SIGNAL   2
#define REVOLVER_SIGNAL   3
#define LED_SIGNAL        4

typedef enum {
  ABSOLUTE,
  RELATIVE
} PositionMode;

typedef struct {
  int   toolCount           = 5;
  float firstToolOffset     = FIRST_TOOL_OFFSET;
  float toolSpacing         = TOOL_SPACING;
  int   firstRevolverOffset = FIRST_REVOLVER_OFFSET;
  int   revolverSpacing     = REVOLVER_SPACING;
  long  stepsPerMM_X        = X_STEPS_PER_MM;
  long  maxSteps_X          = 68000;
  int   maxSpeed_X          = 10;
  int   acceleration_X      = 510;
  bool  invertDir_X         = false;
  int   endstopTrigger_X    = HIGH;
  
  long  stepsPerRevolution_Y= 9600;
  long  maxSteps_Y          = 9600;
  int   maxSpeed_Y          = 800;
  int   acceleration_Y      = 2000;
  bool  resetBeforeFeed_Y   = true;
  bool  invertDir_Y         = false;
  int   endstopTrigger_Y    = HIGH;
  
  bool  externalControl_Z   = false;
  long  stepsPerMM_Z        = Z_STEPS_PER_MM;
  int   maxSpeed_Z          = 10;
  int   insertSpeed_Z       = 1000;
  int   acceleration_Z      = 300;
  bool  invertDir_Z         = false;
  int   endstopTrigger_Z    = LOW;
  
  float unloadRetract       = -20.0f;
  float unloadPushback      = 5.0f;
  float pushbackDelay       = 1.5f;
  float reinforceLength     = 3.0f;
  bool  homeAfterFeed       = true;
  float bowdenLength        = 400.0f;
  int   i2cAddress          = 0x58;
  int   lcdContrast         = DSP_CONTRAST;
  int   menuAutoClose       = 20;
  bool  delayBetweenPulses  = false;
  unsigned long serial1Baudrate = 57600;
  unsigned long serial2Baudrate = 57600;
  unsigned long serialDueBaudrate = 57600;
  bool  duetDirect          = false;
  int   fanSpeed            = 0;
  char  materials[MAX_TOOLS][20];
  long  powerSaveTimeout    = 300;
  char  unloadCommand[80]   = {{ 0 }};
  int   wipeSequence[20]    = { 150,20,45,20,45,20,45,20,45,20,45,20,45,20,45,20,45,20,110,-1 };
  bool  prusaMMU2           = true;
} SMuFFConfig;

extern U8G2_ST7565_64128N_F_4W_HW_SPI   display;
extern Encoder                          encoder;

extern SMuFFConfig    smuffConfig;
extern GCodeFunctions gCodeFuncsM[];
extern GCodeFunctions gCodeFuncsG[];

extern const char     brand[];
extern volatile byte  nextStepperFlag;
extern volatile byte  remainingSteppersFlag;
extern volatile unsigned long lastEncoderButtonTime;
extern char           buf[];
extern byte           toolSelected;
extern PositionMode   positionMode;
extern String         serialBuffer0, serialBuffer2, serialBuffer3, serialBuffer9, traceSerial2; 
extern bool           displayingUserMessage;
extern unsigned int   userMessageTime;
extern bool           testMode;
extern bool           feederJamed;
extern bool           parserBusy;
extern bool           isPwrSave;

extern void setupDisplay();
extern void drawLogo();
extern void drawStatus();
extern void drawSelectingMessage();
extern void drawUserMessage(String message);
extern void drawSDStatus(int stat);
extern void resetDisplay();
extern bool selectorEndstop();
extern bool revolverEndstop();
extern bool feederEndstop();
extern bool showFeederLoadedMessage();
extern bool showFeederLoadMessage();
extern bool showFeederFailedMessage(int state);
extern int  showDialog(PGM_P title, PGM_P message, PGM_P addMessage, PGM_P buttons);
extern bool moveHome(int index, bool showMessage = true, bool checkFeeder = true);
extern bool loadFilament(bool showMessage = true);
extern bool unloadFilament();
extern void runAndWait(int index);
extern void runNoWait(int index);
extern bool selectTool(int ndx, bool showMessage = true);
extern void setStepperSteps(int index, long steps, bool ignoreEndstop);
extern void prepSteppingAbs(int index, long steps, bool ignoreEndstop = false);
extern void prepSteppingAbsMillimeter(int index, float millimeter, bool ignoreEndstop = false);
extern void prepSteppingRel(int index, long steps, bool ignoreEndstop = false);
extern void prepSteppingRelMillimeter(int index, float millimeter, bool ignoreEndstop = false);
extern void resetRevolver();
extern void serialEvent();
extern void serialEvent2();
extern void wireReceiveEvent(int numBytes);
extern void beep(int count);
extern void longBeep(int count);
extern void userBeep();
extern void setSignalPort(int port, bool state);
extern void signalNoTool();
extern void signalLoadFilament();
extern void signalUnloadFilament();
extern void signalSelectorBusy();
extern void signalSelectorReady();
extern bool setServoPos(int degree);
extern void getEepromData();
extern void readConfig();
extern bool checkAutoClose();
extern void resetAutoClose();
extern void listDir(File root, int numTabs, int serial);
extern void setPwrSave(int state);
extern void __debug(const char* fmt, ...);

extern void printEndstopState(int serial);
extern void printPos(int index, int serial);
extern void printAcceleration(int serial);
extern void printSpeeds(int serial);
extern void sendGList(int serial);
extern void sendMList(int serial);
extern void sendToolResponse(int serial);
extern void sendStartResponse(int serial);
extern void sendOkResponse(int serial);
extern void sendErrorResponse(int serial, const char* msg = NULL);
extern void sendErrorResponseP(int serial, const char* msg = NULL);
extern void parseGcode(String serialBuffer, int serial);
extern bool parse_G(String buf, int serial);
extern bool parse_M(String buf, int serial);
extern bool parse_T(String buf, int serial);
extern bool parse_PMMU2(char cmd, String buf, int serial);
extern int  getParam(String buf, char* token);
extern bool getParamString(String buf, char* token, char* dest, int bufLen);
extern void prepStepping(int index, long param, bool Millimeter = true, bool ignoreEndstop = false);
extern void saveSettings(int serial);
extern void reportSettings(int serial);
extern void printResponse(const char* response, int serial);
extern void printResponseP(const char* response, int serial);
extern void printOffsets(int serial);

#endif
