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
 
 /*
  * Module for reading configuration file (JSON-Format) from SD-Card
  */

#include "SMuFF.h"
#include "ConfigNames.h"
#include <ArduinoJson.h>

SdFs SD;


#if defined(__STM32F1__)
const size_t capacity = 3500;     // if this value gets exceeded, dump or reduce the material names
#elif defined(__ESP32__)
const size_t capacity = 4800;     // since the ESP32 has more memory, we can do this
#else
const size_t capacity = 1300;
#endif

void readConfig()
{
  DynamicJsonDocument jsonDoc(capacity);

  if(SDCS_PIN != -1) {
    if (!SD.begin(SDCS_PIN, SD_SCK_MHZ(4))) {
      drawSDStatus(SD_ERR_INIT);
      delay(5000);
      return;
    }
  }
  else {
    if (!SD.begin()) {
      drawSDStatus(SD_ERR_INIT);
      delay(5000);
      return;
    }
  }

  //__debug(PSTR("Trying to open config file '%s'"), CONFIG_FILE);
  FsFile cfg;
  if(cfg.open(CONFIG_FILE))
  {
    if(cfg.fileSize() > capacity) {
      longBeep(2);
      showDialog(P_TitleConfigError, P_ConfigFail1, P_ConfigFail3, P_OkButtonOnly);
      cfg.close();
      return;
    }
    
    auto error = deserializeJson(jsonDoc, cfg);
    if (error) {
      longBeep(2);
      __debug(PSTR("deserializeJson() failed with code %s"), error.c_str());
      showDialog(P_TitleConfigError, P_ConfigFail1, P_ConfigFail2, P_OkButtonOnly);
    }
    else {

      drawSDStatus(SD_READING_CONFIG);
      int toolCnt =                             jsonDoc[toolCount];
      /*
      SELECTOR
      */
      smuffConfig.toolCount = (toolCnt > MIN_TOOLS && toolCnt <= MAX_TOOLS) ? toolCnt : 5;
      smuffConfig.firstToolOffset =             jsonDoc[selector][offset];
      smuffConfig.toolSpacing =                 jsonDoc[selector][spacing];
      smuffConfig.stepsPerMM_X =                jsonDoc[selector][stepsPerMillimeter];
      smuffConfig.maxSteps_X = ((smuffConfig.toolCount-1)*smuffConfig.toolSpacing+smuffConfig.firstToolOffset) * smuffConfig.stepsPerMM_X;
      int speed =                               jsonDoc[selector][maxSpeed];
      int accel =                               jsonDoc[selector][acceleration];
      if(speed < MIN_MMS || speed > MAX_MMS)
        speed = MIN_MMS;
      if(accel < MIN_MMS || accel > MAX_MMS)
        accel = MIN_MMS;
      smuffConfig.maxSpeed_X =                  translateSpeed(speed, smuffConfig.stepsPerMM_X);
      smuffConfig.acceleration_X =              translateSpeed(accel, smuffConfig.stepsPerMM_X);
      smuffConfig.invertDir_X =                 jsonDoc[selector][invertDir];
      smuffConfig.endstopTrigger_X =            jsonDoc[selector][endstopTrig];
      smuffConfig.stepDelay_X =                 jsonDoc[selector][stepDelay];
      smuffConfig.stepperPower[SELECTOR] =      jsonDoc[selector][power];
      smuffConfig.stepperMode[SELECTOR] =       jsonDoc[selector][mode];
      smuffConfig.stepperRSense[SELECTOR] =     jsonDoc[selector][rsense];
      smuffConfig.stepperMicrosteps[SELECTOR] = jsonDoc[selector][msteps];
      smuffConfig.stepperStall[SELECTOR] =      jsonDoc[selector][stall];
      smuffConfig.stepperCSmin[SELECTOR] =      jsonDoc[selector][cstepmin];
      smuffConfig.stepperCSmax[SELECTOR] =      jsonDoc[selector][cstepmax];
      smuffConfig.stepperCSdown[SELECTOR] =     jsonDoc[selector][cstepdown];
      smuffConfig.stepperAddr[SELECTOR] =       jsonDoc[selector][drvrAdr];
      smuffConfig.stepperToff[SELECTOR] =       jsonDoc[selector][toff];
      smuffConfig.stepperStopOnStall[SELECTOR]= jsonDoc[selector][stopOnStall];
      smuffConfig.stepperMaxStallCnt[SELECTOR]= jsonDoc[selector][maxStallCount];
      /*
      REVOLVER
      */
      smuffConfig.stepsPerRevolution_Y =        jsonDoc[revolver][stepsPerRevolution];
      smuffConfig.firstRevolverOffset =         jsonDoc[revolver][offset];
      smuffConfig.revolverSpacing =     smuffConfig.stepsPerRevolution_Y / 10;
      speed =                                   jsonDoc[revolver][maxSpeed];
      accel =                                   jsonDoc[revolver][acceleration];
      if(speed < MIN_MMS || speed > MAX_MMS)
        speed = MIN_MMS;
      if(accel < MIN_MMS || accel > MAX_MMS)
        accel = MIN_MMS;
      smuffConfig.maxSpeed_Y =                  translateSpeed(speed, smuffConfig.stepsPerRevolution_Y/360);
      smuffConfig.acceleration_Y =              translateSpeed(accel, smuffConfig.stepsPerRevolution_Y/360);
      smuffConfig.resetBeforeFeed_Y =           jsonDoc[revolver][resetBeforeFeed];
      smuffConfig.homeAfterFeed =               jsonDoc[revolver][homeAfterFeed];
      smuffConfig.invertDir_Y =                 jsonDoc[revolver][invertDir];
      smuffConfig.endstopTrigger_Y =            jsonDoc[revolver][endstopTrig];
      smuffConfig.stepDelay_Y =                 jsonDoc[revolver][stepDelay];
      smuffConfig.wiggleRevolver =              jsonDoc[revolver][wiggle];
      smuffConfig.revolverIsServo =             jsonDoc[revolver][useServo];
      smuffConfig.revolverOffPos =              jsonDoc[revolver][servoOffPos];
      smuffConfig.revolverOnPos =               jsonDoc[revolver][servoOnPos];
      smuffConfig.servoCycles1 =                jsonDoc[revolver][servo1Cycles];
      smuffConfig.servoCycles2 =                jsonDoc[revolver][servo2Cycles];
      smuffConfig.stepperPower[REVOLVER] =      jsonDoc[revolver][power];
      smuffConfig.stepperMode[REVOLVER] =       jsonDoc[revolver][mode];
      smuffConfig.stepperRSense[REVOLVER] =     jsonDoc[revolver][rsense];
      smuffConfig.stepperMicrosteps[REVOLVER] = jsonDoc[revolver][msteps];
      smuffConfig.stepperStall[REVOLVER] =      jsonDoc[revolver][stall];
      smuffConfig.stepperCSmin[REVOLVER] =      jsonDoc[revolver][cstepmin];
      smuffConfig.stepperCSmax[REVOLVER] =      jsonDoc[revolver][cstepmax];
      smuffConfig.stepperCSdown[REVOLVER] =     jsonDoc[revolver][cstepdown];
      smuffConfig.stepperAddr[REVOLVER] =       jsonDoc[revolver][drvrAdr];
      smuffConfig.stepperToff[REVOLVER] =       jsonDoc[revolver][toff];
      smuffConfig.stepperStopOnStall[REVOLVER]= jsonDoc[revolver][stopOnStall];
      smuffConfig.stepperMaxStallCnt[REVOLVER]= jsonDoc[revolver][maxStallCount];
      /*
      FEEDER
      */
      smuffConfig.externalControl_Z =           jsonDoc[feeder][externalControl];
      smuffConfig.stepsPerMM_Z =                jsonDoc[feeder][stepsPerMillimeter];
      speed =                                   jsonDoc[feeder][maxSpeed];
      accel =                                   jsonDoc[feeder][acceleration];
      int ispeed =                              jsonDoc[feeder][insertSpeed];
      if(speed < MIN_MMS || speed > MAX_MMS)
        speed = MIN_MMS;
      if(accel < MIN_MMS || accel > MAX_MMS)
        accel = MIN_MMS;
      if(ispeed < MIN_MMS || ispeed > MAX_MMS)
        ispeed = MIN_MMS;
      smuffConfig.maxSpeed_Z =                  translateSpeed(speed, smuffConfig.stepsPerMM_Z);
      smuffConfig.acceleration_Z =              translateSpeed(accel, smuffConfig.stepsPerMM_Z);
      smuffConfig.insertSpeed_Z =               translateSpeed(ispeed, smuffConfig.stepsPerMM_Z);
      if(smuffConfig.insertSpeed_Z > smuffConfig.acceleration_Z)
        smuffConfig.acceleration_Z = smuffConfig.insertSpeed_Z;
      smuffConfig.invertDir_Z =                 jsonDoc[feeder][invertDir];
      smuffConfig.endstopTrigger_Z =            jsonDoc[feeder][endstopTrig];
      smuffConfig.stepDelay_Z =                 jsonDoc[feeder][stepDelay];
      smuffConfig.reinforceLength =             jsonDoc[feeder][reinforceLength];
      smuffConfig.unloadRetract =               jsonDoc[feeder][unloadRetract];
      smuffConfig.unloadPushback =              jsonDoc[feeder][unloadPushback];
      smuffConfig.pushbackDelay =               jsonDoc[feeder][pushbackDelay];
      smuffConfig.enableChunks =                jsonDoc[feeder][enableChunks];
      smuffConfig.feedChunks =                  jsonDoc[feeder][feedChunks];
      if(smuffConfig.feedChunks == 0)
        smuffConfig.feedChunks = 20;
      smuffConfig.insertLength =                jsonDoc[feeder][insertLength];
      if(smuffConfig.insertLength == 0)
        smuffConfig.insertLength = 5;
      smuffConfig.useDuetLaser =                jsonDoc[feeder][duetLaser];
      smuffConfig.isSharedStepper =             jsonDoc[feeder][sharedStepper];
      smuffConfig.stepperPower[FEEDER] =        jsonDoc[feeder][power];
      smuffConfig.stepperMode[FEEDER] =         jsonDoc[feeder][mode];
      smuffConfig.stepperRSense[FEEDER] =       jsonDoc[feeder][rsense];
      smuffConfig.stepperMicrosteps[FEEDER] =   jsonDoc[feeder][msteps];
      smuffConfig.stepperStall[FEEDER] =        jsonDoc[feeder][stall];
      smuffConfig.stepperCSmin[FEEDER] =        jsonDoc[feeder][cstepmin];
      smuffConfig.stepperCSmax[FEEDER] =        jsonDoc[feeder][cstepmax];
      smuffConfig.stepperCSdown[FEEDER] =       jsonDoc[feeder][cstepdown];
      smuffConfig.stepperAddr[FEEDER] =         jsonDoc[feeder][drvrAdr];
      smuffConfig.stepperToff[FEEDER] =         jsonDoc[feeder][toff];
      smuffConfig.stepperStopOnStall[FEEDER]=   jsonDoc[feeder][stopOnStall];
      smuffConfig.stepperMaxStallCnt[FEEDER]=   jsonDoc[feeder][maxStallCount];

      smuffConfig.sendPeriodicalStats =         jsonDoc[periodicalStats];
      int contrast =                            jsonDoc[contrast];
      smuffConfig.lcdContrast = (contrast >= MIN_CONTRAST && contrast <= MAX_CONTRAST) ? contrast : DSP_CONTRAST;
      int backlightColor =                      jsonDoc[backlightColor];
      smuffConfig.backlightColor = (backlightColor == 0 ? 7 : backlightColor);   // set backlight color to white if not set
      smuffConfig.encoderTickSound =            jsonDoc[encoderTicks];

      smuffConfig.bowdenLength =                jsonDoc[bowdenLength];
      smuffConfig.selectorDistance =            jsonDoc[selectorDist];
      int i2cAdr =                              jsonDoc[i2cAdr];
      smuffConfig.i2cAddress = (i2cAdr > 0 && i2cAdr < 255) ? i2cAdr : I2C_SLAVE_ADDRESS;
      smuffConfig.menuAutoClose =               jsonDoc[autoClose];
      smuffConfig.serial0Baudrate =             jsonDoc[serial0Baudrate];
      smuffConfig.serial1Baudrate =             jsonDoc[serial1Baudrate];
      smuffConfig.serial2Baudrate =             jsonDoc[serial2Baudrate];
      smuffConfig.serial3Baudrate =             jsonDoc[serial3Baudrate];
      smuffConfig.fanSpeed =                    jsonDoc[fanSpeed];
      smuffConfig.powerSaveTimeout =            jsonDoc[psTimeout];
      smuffConfig.sendActionCmds =              jsonDoc[sendAction];
      const char* p1 =                          jsonDoc[unloadCommand];
      const char* p2 =                          jsonDoc[wipeSequence];
      if(p1 != NULL && strlen(p1) > 0) {
#if defined(__STM32F1__) || defined(__ESP32__)
        strncpy(smuffConfig.unloadCommand, p1, sizeof(smuffConfig.unloadCommand));
#else
        strlcpy(smuffConfig.unloadCommand, p1, sizeof(smuffConfig.unloadCommand));
#endif
      }
      if(p2 != NULL && strlen(p2) > 0) {
#if defined(__STM32F1__) || defined(__ESP32__)
        strncpy(smuffConfig.wipeSequence, p2, sizeof(smuffConfig.wipeSequence));
#else
        strlcpy(smuffConfig.wipeSequence, p2, sizeof(smuffConfig.wipeSequence));
#endif
      }
      smuffConfig.prusaMMU2 =                   jsonDoc[emulatePrusa];
      smuffConfig.hasPanelDue =                 jsonDoc[hasPanelDue];
      smuffConfig.servoMinPwm =                 jsonDoc[servoMinPwm];
      smuffConfig.servoMaxPwm =                 jsonDoc[servoMaxPwm];
      if(smuffConfig.servoMinPwm == 0)
        smuffConfig.servoMinPwm = 800;
      if(smuffConfig.servoMaxPwm == 0)
        smuffConfig.servoMaxPwm = 2400;

      // read materials if running on 32-Bit MCU
#if defined(__STM32F1__) || defined(__ESP32__)
      for(int i=0; i < smuffConfig.toolCount; i++) {
        char tmp[16];
        sprintf(tmp,"T%d", i);
        if(jsonDoc[material][tmp] == NULL)
          sprintf(tmp,"Tool%d", i);
        memset(smuffConfig.materials[i], 0, sizeof(smuffConfig.materials[i]));
        strncpy(smuffConfig.materials[i], jsonDoc[materials][tmp], sizeof(smuffConfig.materials[i])); 
      }
#else
      for(int i=0; i < smuffConfig.toolCount; i++) {
        memset(smuffConfig.materials[i], 0, sizeof(smuffConfig.materials[i]));
      }
#endif
      
      __debug(PSTR("DONE reading config"));
    }
    cfg.close();
  }
  else {
    longBeep(2);
    __debug(PSTR("Open config file failed: handle = %s"), !cfg ? "FALSE" : "TRUE");
    drawSDStatus(SD_ERR_NOCONFIG);
    delay(5000);
  }
}

bool writeConfig(Print* dumpTo)
{
  bool stat = false;

  StaticJsonDocument<capacity> jsonDoc;
  if(dumpTo == NULL) {
    if(SDCS_PIN != -1) {
      if (!SD.begin(SDCS_PIN, SD_SCK_MHZ(4))) {
        drawSDStatus(SD_ERR_INIT);
        delay(5000);
        return false;
      }
    }
    else {
      if(!SD.begin()) {
        drawSDStatus(SD_ERR_INIT);
        delay(5000);
        return false;
      }
    }
  }
  JsonObject jsonObj = jsonDoc.to<JsonObject>();
  jsonDoc[serial0Baudrate]      = smuffConfig.serial0Baudrate;
  jsonDoc[serial1Baudrate]      = smuffConfig.serial1Baudrate;
  jsonDoc[serial2Baudrate]      = smuffConfig.serial2Baudrate;
  jsonDoc[serial3Baudrate]      = smuffConfig.serial3Baudrate;
  jsonDoc[toolCount]            = smuffConfig.toolCount;
  jsonDoc[bowdenLength]         = smuffConfig.bowdenLength;
  jsonDoc[selectorDist]         = smuffConfig.selectorDistance;
  jsonDoc[contrast]             = smuffConfig.lcdContrast;
  jsonDoc[i2cAdr]               = smuffConfig.i2cAddress;
  jsonDoc[autoClose]            = smuffConfig.menuAutoClose;
  jsonDoc[fanSpeed]             = smuffConfig.fanSpeed;
  jsonDoc[psTimeout]            = smuffConfig.powerSaveTimeout;
  jsonDoc[sendAction]           = smuffConfig.sendActionCmds;
  jsonDoc[emulatePrusa]         = smuffConfig.prusaMMU2;
  jsonDoc[unloadCommand]        = smuffConfig.unloadCommand;
  jsonDoc[hasPanelDue]          = smuffConfig.hasPanelDue;
  jsonDoc[servoMinPwm]          = smuffConfig.servoMinPwm;
  jsonDoc[servoMaxPwm]          = smuffConfig.servoMaxPwm;
  jsonDoc[periodicalStats]      = smuffConfig.sendPeriodicalStats;
  jsonDoc[wipeSequence]         = smuffConfig.wipeSequence;
  jsonDoc[backlightColor]       = smuffConfig.backlightColor;
  jsonDoc[encoderTicks]         = smuffConfig.encoderTickSound;

  JsonObject node = jsonObj.createNestedObject(selector);
  node[offset]                = smuffConfig.firstToolOffset;
  node[spacing]               = smuffConfig.toolSpacing;
  node[stepsPerMillimeter]    = smuffConfig.stepsPerMM_X;
  node[stepDelay]             = smuffConfig.stepDelay_X;
  node[maxSpeed]              = translateSpeed(smuffConfig.maxSpeed_X, smuffConfig.stepsPerMM_X);
  node[acceleration]          = translateSpeed(smuffConfig.acceleration_X, smuffConfig.stepsPerMM_X);
  node[invertDir]             = smuffConfig.invertDir_X;
  node[endstopTrig]           = smuffConfig.endstopTrigger_X;
  node[power]                 = smuffConfig.stepperPower[SELECTOR];
  node[mode]                  = smuffConfig.stepperMode[SELECTOR];
  node[rsense]                = smuffConfig.stepperRSense[SELECTOR];
  node[msteps]                = smuffConfig.stepperMicrosteps[SELECTOR];
  node[stall]                 = smuffConfig.stepperStall[SELECTOR];
  node[cstepmin]              = smuffConfig.stepperCSmin[SELECTOR];
  node[cstepmax]              = smuffConfig.stepperCSmax[SELECTOR];
  node[cstepdown]             = smuffConfig.stepperCSdown[SELECTOR];
  node[drvrAdr]               = smuffConfig.stepperAddr[SELECTOR];
  node[toff]                  = smuffConfig.stepperToff[SELECTOR];
  node[stopOnStall]           = smuffConfig.stepperStopOnStall[SELECTOR];
  node[maxStallCount]         = smuffConfig.stepperMaxStallCnt[SELECTOR];

  node = jsonObj.createNestedObject(revolver);
  node[offset]                = smuffConfig.firstRevolverOffset;
  node[stepsPerRevolution]    = smuffConfig.stepsPerRevolution_Y;
  node[stepDelay]             = smuffConfig.stepDelay_Y;
  node[maxSpeed]              = translateSpeed(smuffConfig.maxSpeed_Y, smuffConfig.stepsPerRevolution_Y/360);
  node[acceleration]          = translateSpeed(smuffConfig.acceleration_Y, smuffConfig.stepsPerRevolution_Y/360);
  node[resetBeforeFeed]       = smuffConfig.resetBeforeFeed_Y;
  node[homeAfterFeed]         = smuffConfig.homeAfterFeed;
  node[invertDir]             = smuffConfig.invertDir_Y;
  node[endstopTrig]           = smuffConfig.endstopTrigger_Y;
  node[wiggle]                = smuffConfig.wiggleRevolver;
  node[useServo]              = smuffConfig.revolverIsServo;
  node[servoOffPos]           = smuffConfig.revolverOffPos;
  node[servoOnPos]            = smuffConfig.revolverOnPos;
  node[servo1Cycles]          = smuffConfig.servoCycles1;
  node[servo2Cycles]          = smuffConfig.servoCycles2;
  node[power]                 = smuffConfig.stepperPower[REVOLVER];
  node[mode]                  = smuffConfig.stepperMode[REVOLVER];
  node[rsense]                = smuffConfig.stepperRSense[REVOLVER];
  node[msteps]                = smuffConfig.stepperMicrosteps[REVOLVER];
  node[stall]                 = smuffConfig.stepperStall[REVOLVER];
  node[cstepmin]              = smuffConfig.stepperCSmin[REVOLVER];
  node[cstepmax]              = smuffConfig.stepperCSmax[REVOLVER];
  node[cstepdown]             = smuffConfig.stepperCSdown[REVOLVER];
  node[drvrAdr]               = smuffConfig.stepperAddr[REVOLVER];
  node[toff]                  = smuffConfig.stepperToff[REVOLVER];
  node[stopOnStall]           = smuffConfig.stepperStopOnStall[REVOLVER];
  node[maxStallCount]         = smuffConfig.stepperMaxStallCnt[REVOLVER];

  node = jsonObj.createNestedObject(feeder);
  node[externalControl]       = smuffConfig.externalControl_Z;
  node[stepsPerMillimeter]    = smuffConfig.stepsPerMM_Z;
  node[stepDelay]             = smuffConfig.stepDelay_Z;
  node[maxSpeed]              = translateSpeed(smuffConfig.maxSpeed_Z, smuffConfig.stepsPerMM_Z);
  node[acceleration]          = translateSpeed(smuffConfig.acceleration_Z, smuffConfig.stepsPerMM_Z);
  node[insertSpeed]           = translateSpeed(smuffConfig.insertSpeed_Z, smuffConfig.stepsPerMM_Z);
  node[invertDir]             = smuffConfig.invertDir_Z;
  node[endstopTrig]           = smuffConfig.endstopTrigger_Z;
  node[reinforceLength]       = smuffConfig.reinforceLength;
#if defined(__STM32F1__) || defined(__ESP32__)
  node[unloadRetract]         = smuffConfig.unloadRetract;
  node[unloadPushback]        = smuffConfig.unloadPushback;
  node[pushbackDelay]         = smuffConfig.pushbackDelay;
#endif
  node[enableChunks]          = smuffConfig.enableChunks;
  node[feedChunks]            = smuffConfig.feedChunks;
  node[insertLength]          = smuffConfig.insertLength;
  node[duetLaser]             = smuffConfig.useDuetLaser;
  node[sharedStepper]         = smuffConfig.isSharedStepper;
  node[power]                 = smuffConfig.stepperPower[FEEDER];
  node[mode]                  = smuffConfig.stepperMode[FEEDER];
  node[rsense]                = smuffConfig.stepperRSense[FEEDER];
  node[msteps]                = smuffConfig.stepperMicrosteps[FEEDER];
  node[stall]                 = smuffConfig.stepperStall[FEEDER];
  node[cstepmin]              = smuffConfig.stepperCSmin[FEEDER];
  node[cstepmax]              = smuffConfig.stepperCSmax[FEEDER];
  node[cstepdown]             = smuffConfig.stepperCSdown[FEEDER];
  node[drvrAdr]               = smuffConfig.stepperAddr[FEEDER];
  node[toff]                  = smuffConfig.stepperToff[FEEDER];
  node[stopOnStall]           = smuffConfig.stepperStopOnStall[FEEDER];
  node[maxStallCount]         = smuffConfig.stepperMaxStallCnt[FEEDER];

#if defined(__STM32F1__) || defined(__ESP32__)
  node = jsonObj.createNestedObject(materials);
  for(int i=0; i < smuffConfig.toolCount; i++) {
    char tmp[16];
    sprintf(tmp,"T%d", i);
    node[tmp] = smuffConfig.materials[i] != NULL ?  smuffConfig.materials[i] : "";
  }
#endif

  if(dumpTo == NULL) {
    FsFile cfg;
    if(cfg.open(CONFIG_FILE, (uint8_t)(O_WRITE | O_CREAT | O_TRUNC))) {
      serializeJsonPretty(jsonDoc, cfg);
      stat = true;
    }
    cfg.close();  
  }
  else {
    serializeJsonPretty(jsonDoc, *dumpTo);
    stat = true;
  }
  return stat;
}