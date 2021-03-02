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

SdFat SD;

#if defined(__STM32F1__)
const size_t capacity = 2400;
const size_t scapacity = 1600;
#elif defined(__ESP32__)
const size_t capacity = 4500;     // since the ESP32 has more memory, we can do this
const size_t scapacity = 1000;
#else
const size_t capacity = 1300;
const size_t scapacity = 1000;
#endif

bool initSD(bool showStatus) {
  #if !defined(USE_COMPOSITE_SERIAL)
  bool sdStat;
  if(SDCS_PIN != -1)
    sdStat = SD.begin(SDCS_PIN, SD_SCK_MHZ(4));
  else
    sdStat = SD.begin();

  if (!sdStat) {
    if(showStatus) {
      drawSDStatus(SD_ERR_INIT);
      delay(5000);
    }
    return false;
  }
  #endif
  return true;
}

void showDeserializeFailed(DeserializationError error, const char* PROGMEM errMsg) {
  __debugS(PSTR("deserializeJson() failed with code %s"), error.c_str());
  longBeep(2);
  showDialog(P_TitleConfigError, errMsg, P_ConfigFail2, P_OkButtonOnly);
}

void showOpenFailed(SdFile* file, const char* cfgFile) {
  __debugS(PSTR("Opening file '%s' failed: handle = %s"), cfgFile, !file ? "FALSE" : "TRUE");
  longBeep(2);
  drawSDStatus(SD_ERR_NOCONFIG);
  delay(5000);
}

bool checkFileSize(SdFile* file, size_t cap, const char* PROGMEM errMsg) {
  if(file != nullptr && file->fileSize() > cap) {
    longBeep(2);
    showDialog(P_TitleConfigError, errMsg, P_ConfigFail3, P_OkButtonOnly);
    file->close();
    return false;
  }
  //__debugS(PSTR("config file '%s' open"), CONFIG_FILE);
  return true;
}

SdFile cfgOut;
Print* openCfgFileWrite(const char* filename) {
  if(cfgOut.open(filename, (uint8_t)(O_WRITE | O_CREAT | O_TRUNC))) {
    return &cfgOut;
  }
  return nullptr;
}

void closeCfgFile() {
  if(cfgOut.isOpen())
    cfgOut.close();
}

SdFile cfg;

/*
  Reads main config from SD-Card
*/
bool readConfig()
{
  DynamicJsonDocument jsonDoc(capacity); // use memory from heap to deserialize
  //StaticJsonDocument<capacity> jsonDoc; // use memory from stack to deserialize

  if(!initSD())
    return false;
  //__debugS(PSTR("Trying to open config file '%s'"), CONFIG_FILE);

  if(!cfg.open(CONFIG_FILE)) {
    showOpenFailed(&cfg, CONFIG_FILE);
    return false;
  }
  else {
    if(!checkFileSize(&cfg, capacity, P_ConfigFail1))
      return false;
    DeserializationError error = deserializeJson(jsonDoc, cfg);
    if (error)
      showDeserializeFailed(error, P_ConfigFail1);
    else {
      drawSDStatus(SD_READING_CONFIG);
      uint8_t toolCnt =                         jsonDoc[toolCount];
      smuffConfig.toolCount = (toolCnt > MIN_TOOLS && toolCnt <= MAX_TOOLS) ? toolCnt : 5;
      uint8_t _contrast =                        jsonDoc[contrast];
      smuffConfig.lcdContrast = (_contrast >= MIN_CONTRAST && _contrast <= MAX_CONTRAST) ? _contrast : DSP_CONTRAST;
      uint8_t _backlightColor =                  jsonDoc[backlightColor];
      smuffConfig.backlightColor = (_backlightColor == 0 ? 7 : _backlightColor);  // set backlight color to White if not set
      uint8_t _toolColor =                       jsonDoc[toolColor];
      smuffConfig.toolColor = (_toolColor == 0 ? 5 : _toolColor);                 // set tool color to Magenta if not set
      smuffConfig.encoderTickSound =            jsonDoc[encoderTicks];
      smuffConfig.bowdenLength =                jsonDoc[bowdenLength];
      smuffConfig.selectorDistance =            jsonDoc[selectorDist];
      uint8_t _i2cAdr =                          jsonDoc[i2cAdr];
      smuffConfig.i2cAddress = (_i2cAdr > 0 && _i2cAdr < 128) ? _i2cAdr : I2C_SLAVE_ADDRESS;
      smuffConfig.menuAutoClose =               jsonDoc[autoClose];
      smuffConfig.serialBaudrates[0] =          jsonDoc[serial0Baudrate];
      smuffConfig.serialBaudrates[1] =          jsonDoc[serial1Baudrate];
      smuffConfig.serialBaudrates[2] =          jsonDoc[serial2Baudrate];
      smuffConfig.serialBaudrates[3] =          jsonDoc[serial3Baudrate];
      smuffConfig.fanSpeed =                    jsonDoc[fanSpeed];
      smuffConfig.powerSaveTimeout =            jsonDoc[psTimeout];
      smuffConfig.sendActionCmds =              jsonDoc[sendAction];
      const char* p1 =                          jsonDoc[unloadCommand];
      const char* p2 =                          jsonDoc[wipeSequence];
      const char* p3 =                          jsonDoc[lButtonDown];
      const char* p4 =                          jsonDoc[lButtonHold];
      const char* p5 =                          jsonDoc[rButtonDown];
      const char* p6 =                          jsonDoc[rButtonHold];
      if(p1 != nullptr && strlen(p1) > 0) {
        strncpy(smuffConfig.unloadCommand, p1, ArraySize(smuffConfig.unloadCommand));
      }
      if(p2 != nullptr && strlen(p2) > 0) {
        strncpy(smuffConfig.wipeSequence, p2, ArraySize(smuffConfig.wipeSequence));
      }
      if(p3 != nullptr && strlen(p3) > 0) {
        strncpy(smuffConfig.lButtonDown, p3, ArraySize(smuffConfig.lButtonDown));
      }
      if(p4 != nullptr && strlen(p4) > 0) {
        strncpy(smuffConfig.lButtonHold, p4, ArraySize(smuffConfig.lButtonHold));
      }
      if(p5 != nullptr && strlen(p5) > 0) {
        strncpy(smuffConfig.rButtonDown, p5, ArraySize(smuffConfig.rButtonDown));
      }
      if(p6 != nullptr && strlen(p6) > 0) {
        strncpy(smuffConfig.rButtonHold, p6, ArraySize(smuffConfig.rButtonHold));
      }
      smuffConfig.prusaMMU2 =                   jsonDoc[emulatePrusa];
      smuffConfig.hasPanelDue =                 jsonDoc[hasPanelDue];
      smuffConfig.servoMinPwm =                 jsonDoc[servoMinPwm];
      smuffConfig.servoMaxPwm =                 jsonDoc[servoMaxPwm];
      if(smuffConfig.servoMinPwm == 0)
        smuffConfig.servoMinPwm = 800;
      if(smuffConfig.servoMaxPwm == 0)
        smuffConfig.servoMaxPwm = 2400;
      smuffConfig.sendPeriodicalStats =         jsonDoc[periodicalStats];
      smuffConfig.speedsInMMS =                 jsonDoc[speedsInMMS];
      if(!smuffConfig.speedsInMMS) {
        mmsMax = MAX_TICKS;
        speedIncrement = INC_TICKS;
      }
      else {
        mmsMax = MAX_MMS;
        speedIncrement = INC_MMS;
      }
      smuffConfig.motorOnDelay =                jsonDoc[motDelay];
      smuffConfig.useCutter =                   jsonDoc[useCutter];
      smuffConfig.cutterOpen =                  jsonDoc[cutterOpen];
      smuffConfig.cutterClose =                 jsonDoc[cutterClose];
      smuffConfig.usePurge =                    jsonDoc[usePurge];
      smuffConfig.cutterLength =                jsonDoc[cutterLength];
      smuffConfig.useIdleAnimation =            jsonDoc[idleAnim];

      /*
      SELECTOR
      */
      smuffConfig.firstToolOffset =             jsonDoc[selector][offset];
      smuffConfig.toolSpacing =                 jsonDoc[selector][spacing];
      smuffConfig.stepsPerMM[SELECTOR] =        jsonDoc[selector][stepsPerMillimeter];
      smuffConfig.maxSteps[SELECTOR] = ((smuffConfig.toolCount-1)*smuffConfig.toolSpacing+smuffConfig.firstToolOffset) * smuffConfig.stepsPerMM[SELECTOR];
      uint16_t speed =                          jsonDoc[selector][maxSpeed];
      uint16_t accel =                          jsonDoc[selector][accelSpeed];
      if(speed < mmsMin || speed > mmsMax)
        speed = mmsMin;
      if(accel < mmsMin || accel > mmsMax)
        accel = mmsMin;
      smuffConfig.maxSpeed[SELECTOR] =          speed;
      smuffConfig.accelSpeed[SELECTOR] =        accel;
      smuffConfig.accelDist[SELECTOR] =         jsonDoc[selector][accelDist];
      smuffConfig.invertDir[SELECTOR] =         jsonDoc[selector][invertDir];
      smuffConfig.endstopTrg[SELECTOR] =        jsonDoc[selector][endstopTrig];
      smuffConfig.stepDelay[SELECTOR] =         jsonDoc[selector][stepDelay];
      smuffConfig.ms3config[SELECTOR] =         jsonDoc[selector][ms3Config];
      /*
      REVOLVER
      */
      smuffConfig.stepsPerRevolution =          jsonDoc[revolver][stepsPerRevolution];
      smuffConfig.firstRevolverOffset =         jsonDoc[revolver][offset];
      smuffConfig.revolverSpacing =             smuffConfig.stepsPerRevolution / 10;
      speed =                                   jsonDoc[revolver][maxSpeed];
      accel =                                   jsonDoc[revolver][accelSpeed];
      if(speed < mmsMin || speed > mmsMax)
        speed = mmsMin;
      if(accel < mmsMin || accel > mmsMax)
        accel = mmsMin;
      smuffConfig.maxSpeed[REVOLVER] =          speed;
      smuffConfig.accelSpeed[REVOLVER] =        accel;
      smuffConfig.accelDist[REVOLVER] =         jsonDoc[revolver][accelDist];
      smuffConfig.resetBeforeFeed =             jsonDoc[revolver][resetBeforeFeed];
      smuffConfig.homeAfterFeed =               jsonDoc[revolver][homeAfterFeed];
      smuffConfig.invertDir[REVOLVER] =         jsonDoc[revolver][invertDir];
      smuffConfig.endstopTrg[REVOLVER] =        jsonDoc[revolver][endstopTrig];
      smuffConfig.stepDelay[REVOLVER] =         jsonDoc[revolver][stepDelay];
      smuffConfig.wiggleRevolver =              jsonDoc[revolver][wiggle];
      smuffConfig.revolverIsServo =             jsonDoc[revolver][useServo];
      smuffConfig.revolverOffPos =              jsonDoc[revolver][servoOffPos];
      smuffConfig.revolverOnPos =               jsonDoc[revolver][servoOnPos];
      smuffConfig.servoCycles1 =                jsonDoc[revolver][servo1Cycles];
      smuffConfig.servoCycles2 =                jsonDoc[revolver][servo2Cycles];
      smuffConfig.ms3config[REVOLVER] =         jsonDoc[revolver][ms3Config];
      /*
      FEEDER
      */
      smuffConfig.extControlFeeder =            jsonDoc[feeder][externalControl];
      smuffConfig.stepsPerMM[FEEDER] =          jsonDoc[feeder][stepsPerMillimeter];
      speed =                                   jsonDoc[feeder][maxSpeed];
      accel =                                   jsonDoc[feeder][accelSpeed];
      uint16_t ispeed =                         jsonDoc[feeder][insertSpeed];
      uint16_t pspeed =                         jsonDoc[feeder][purgeSpeed];
      if(speed < mmsMin || speed > mmsMax)
        speed = mmsMin;
      if(accel < mmsMin || accel > mmsMax)
        accel = mmsMin;
      if(ispeed < mmsMin || ispeed > mmsMax)
        ispeed = mmsMin;
      if(pspeed < mmsMin || ispeed > mmsMax)
        pspeed = mmsMin;
      smuffConfig.maxSpeed[FEEDER] =            speed;
      smuffConfig.accelSpeed[FEEDER] =          accel;
      smuffConfig.insertSpeed =                 ispeed;
      smuffConfig.purgeSpeed =                  pspeed;
      smuffConfig.accelDist[FEEDER] =           jsonDoc[feeder][accelDist];
      smuffConfig.invertDir[FEEDER] =           jsonDoc[feeder][invertDir];
      smuffConfig.endstopTrg[FEEDER] =          jsonDoc[feeder][endstopTrig];
      smuffConfig.endstopTrg[3] =               jsonDoc[feeder][endstopTest];
      smuffConfig.useEndstop2 =                 jsonDoc[feeder][endstop2];
      smuffConfig.stepDelay[FEEDER] =           jsonDoc[feeder][stepDelay];
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
      smuffConfig.ms3config[FEEDER] =           jsonDoc[feeder][ms3Config];
      smuffConfig.purgeLength =                 jsonDoc[feeder][purgeLength];
      smuffConfig.wipeBeforeUnload =            jsonDoc[feeder][autoWipe];

      __debugS(PSTR("Config: DONE reading config"));
    }
    cfg.close();
  }
  return true;
}

/*
  Reads TMC driver config from SD-Card
*/
bool readTmcConfig()
{
  //DynamicJsonDocument jsonDoc(scapacity); // use memory from heap to deserialize
  StaticJsonDocument<scapacity> jsonDoc; // use memory on stack to deserialize

  if(!initSD())
    return false;
  //__debugS(PSTR("Trying to open TMC config file '%s'"), TMC_CONFIG_FILE);
  if(!cfg.open(TMC_CONFIG_FILE)) {
    showOpenFailed(&cfg, TMC_CONFIG_FILE);
    return false;
  }
  else {
    if(!checkFileSize(&cfg, scapacity, P_ConfigFail6))
      return false;
    DeserializationError error = deserializeJson(jsonDoc, cfg);
    if (error)
      showDeserializeFailed(error, P_ConfigFail6);
    else {
      drawSDStatus(SD_READING_TMC);
      /*
      SELECTOR
      */
      smuffConfig.stepperPower[SELECTOR] =      jsonDoc[selector][power];
      smuffConfig.stepperMode[SELECTOR] =       jsonDoc[selector][mode];
      smuffConfig.stepperStealth[SELECTOR] =    jsonDoc[selector][tmode];
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
      smuffConfig.stepperPower[REVOLVER] =      jsonDoc[revolver][power];
      smuffConfig.stepperMode[REVOLVER] =       jsonDoc[revolver][mode];
      smuffConfig.stepperStealth[REVOLVER] =    jsonDoc[revolver][tmode];
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
      smuffConfig.stepperPower[FEEDER] =        jsonDoc[feeder][power];
      smuffConfig.stepperMode[FEEDER] =         jsonDoc[feeder][mode];
      smuffConfig.stepperStealth[FEEDER] =      jsonDoc[feeder][tmode];
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
      /*
      FEEDER2 (only for boards with integarted drivers)
      */
      smuffConfig.stepperPower[FEEDER2] =       jsonDoc[feeder2][power];
      smuffConfig.stepperMode[FEEDER2] =        jsonDoc[feeder2][mode];
      smuffConfig.stepperStealth[FEEDER2] =     jsonDoc[feeder2][tmode];
      smuffConfig.stepperRSense[FEEDER2] =      jsonDoc[feeder2][rsense];
      smuffConfig.stepperMicrosteps[FEEDER2] =  jsonDoc[feeder2][msteps];
      smuffConfig.stepperStall[FEEDER2] =       jsonDoc[feeder2][stall];
      smuffConfig.stepperCSmin[FEEDER2] =       jsonDoc[feeder2][cstepmin];
      smuffConfig.stepperCSmax[FEEDER2] =       jsonDoc[feeder2][cstepmax];
      smuffConfig.stepperCSdown[FEEDER2] =      jsonDoc[feeder2][cstepdown];
      smuffConfig.stepperAddr[FEEDER2] =        jsonDoc[feeder2][drvrAdr];
      smuffConfig.stepperToff[FEEDER2] =        jsonDoc[feeder2][toff];
      smuffConfig.stepperStopOnStall[FEEDER2]=  jsonDoc[feeder2][stopOnStall];
      smuffConfig.stepperMaxStallCnt[FEEDER2]=  jsonDoc[feeder2][maxStallCount];

      __debugS(PSTR("Config: DONE reading TMC config"));
    }
    cfg.close();
  }
  return true;
}

/*
  Reads mapping of the servos from SD-Card.
*/
bool readServoMapping() {
  StaticJsonDocument<scapacity> jsonDoc; // use memory on stack to deserialize

  if(!initSD())
    return false;
  //__debugS(PSTR("Trying to open Servo Mapping file '%s'"), SERVOMAP_FILE);
  if(!cfg.open(SERVOMAP_FILE)) {
    showOpenFailed(&cfg, SERVOMAP_FILE);
    return false;
  }
  else {
    if(!checkFileSize(&cfg, scapacity, P_ConfigFail7))
      return false;
    DeserializationError error = deserializeJson(jsonDoc, cfg);
    if (error)
      showDeserializeFailed(error, P_ConfigFail7);
    else {
      drawSDStatus(SD_READING_SERVOS);
      // read servo mappings
      char item[15];
      for(uint8_t i=0; i < smuffConfig.toolCount; i++) {
        sprintf_P(item, P_Tool, i);

        if(jsonDoc[item] == nullptr) {
          #if defined(MULTISERVO)
          servoMapping[i] = -1;
          #else
          #endif
        }
        else {
          #if defined(MULTISERVO)
          servoMapping[i] = jsonDoc[item][servoOutput];
          #endif
          servoPosClosed[i] = jsonDoc[item][servoClosed];
        }
      }
      #if defined(MULTISERVO)
      // read the Wiper servo output pin only
      servoMapping[16] = jsonDoc[wiper][servoOutput];
      #endif
      __debugS(PSTR("Config: DONE reading servo mappings"));
    }
    cfg.close();
  }
  return true;
}

/*
  Reads materials assignment.
*/
bool readMaterials() {
  StaticJsonDocument<scapacity> jsonDoc; // use memory on stack to deserialize

  if(!initSD())
    return false;
  //__debugS(PSTR("Trying to open TMC config file '%s'"), MATERIALS_FILE);
  if(!cfg.open(MATERIALS_FILE)) {
    showOpenFailed(&cfg, MATERIALS_FILE);
    return false;
  }
  else {
    if(!checkFileSize(&cfg, scapacity, P_ConfigFail5))
      return false;
    DeserializationError error = deserializeJson(jsonDoc, cfg);
    if (error)
      showDeserializeFailed(error, P_ConfigFail5);
    else {
      drawSDStatus(SD_READING_MATERIALS);
      // read materials if running on 32-Bit MCU
#if defined(__STM32F1__) || defined(__ESP32__)
      char item[15];
      for(uint8_t i=0; i < smuffConfig.toolCount; i++) {
        memset(smuffConfig.materials[i], 0, ArraySize(smuffConfig.materials[i]));
        sprintf_P(item, P_Tool, i);
        const char* pItem = jsonDoc[item][color];
        if(pItem == nullptr) {
          sprintf(smuffConfig.materials[i],"Tool%d", i);
        }
        else {
          strncpy(smuffConfig.materials[i], pItem, ArraySize(smuffConfig.materials[i]));
        }
        //__debugS(PSTR("%s: %s"), item, smuffConfig.materials[i]);
        uint16_t len = jsonDoc[item][pfactor];
        if(len > 0) {
          smuffConfig.purges[i] = len;
        }
        else {
          smuffConfig.purges[i] = 100;
        }
        const char* pFItem = jsonDoc[item][material];
        if(pFItem != nullptr) {
          __debugS(PSTR("%s: %s"), item, pFItem);
        }
      }
#else
      for(uint8_t i=0; i < smuffConfig.toolCount; i++) {
        memset(smuffConfig.materials[i], 0, ArraySize(smuffConfig.materials[i]));
      }
#endif
      __debugS(PSTR("Config: DONE reading materials"));
    }
    cfg.close();
  }
  return true;
}


/*
  Writes the basic configuration to SD-Card or Serial
*/
bool writeConfig(Print* dumpTo) {
  StaticJsonDocument<capacity> jsonDoc;

  if(dumpTo == nullptr) {
    if(!initSD())
      return false;
  }

  JsonObject jsonObj = jsonDoc.to<JsonObject>();
  jsonDoc[serial0Baudrate]      = smuffConfig.serialBaudrates[0];
  jsonDoc[serial1Baudrate]      = smuffConfig.serialBaudrates[1];
  jsonDoc[serial2Baudrate]      = smuffConfig.serialBaudrates[2];
  jsonDoc[serial3Baudrate]      = smuffConfig.serialBaudrates[3];
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
  jsonDoc[toolColor]            = smuffConfig.toolColor;
  jsonDoc[encoderTicks]         = smuffConfig.encoderTickSound;
  jsonDoc[lButtonDown]          = smuffConfig.lButtonDown;
  jsonDoc[lButtonHold]          = smuffConfig.lButtonHold;
  jsonDoc[rButtonDown]          = smuffConfig.rButtonDown;
  jsonDoc[rButtonHold]          = smuffConfig.rButtonHold;
  jsonDoc[speedsInMMS]          = smuffConfig.speedsInMMS;
  jsonDoc[motDelay]             = smuffConfig.motorOnDelay;
  jsonDoc[useCutter]            = smuffConfig.useCutter;
  jsonDoc[cutterOpen]           = smuffConfig.cutterOpen;
  jsonDoc[cutterClose]          = smuffConfig.cutterClose;
  jsonDoc[usePurge]             = smuffConfig.usePurge;
  jsonDoc[cutterLength]         = smuffConfig.cutterLength;
  jsonDoc[idleAnim]             = smuffConfig.useIdleAnimation;


  JsonObject node = jsonObj.createNestedObject(selector);
  node[offset]                = smuffConfig.firstToolOffset;
  node[spacing]               = smuffConfig.toolSpacing;
  node[stepsPerMillimeter]    = smuffConfig.stepsPerMM[SELECTOR];
  node[stepDelay]             = smuffConfig.stepDelay[SELECTOR];
  node[maxSpeed]              = smuffConfig.maxSpeed[SELECTOR];
  node[accelSpeed]            = smuffConfig.accelSpeed[SELECTOR];
  node[accelDist]             = smuffConfig.accelDist[SELECTOR];
  node[invertDir]             = smuffConfig.invertDir[SELECTOR];
  node[endstopTrig]           = smuffConfig.endstopTrg[SELECTOR];
  node[ms3Config]             = smuffConfig.ms3config[SELECTOR];

  node = jsonObj.createNestedObject(revolver);
  node[offset]                = smuffConfig.firstRevolverOffset;
  node[stepsPerRevolution]    = smuffConfig.stepsPerRevolution;
  node[stepDelay]             = smuffConfig.stepDelay[REVOLVER];
  node[maxSpeed]              = smuffConfig.maxSpeed[REVOLVER];
  node[accelSpeed]            = smuffConfig.accelSpeed[REVOLVER];
  node[accelDist]             = smuffConfig.accelDist[REVOLVER];
  node[resetBeforeFeed]       = smuffConfig.resetBeforeFeed;
  node[homeAfterFeed]         = smuffConfig.homeAfterFeed;
  node[invertDir]             = smuffConfig.invertDir[REVOLVER];
  node[endstopTrig]           = smuffConfig.endstopTrg[REVOLVER];
  node[wiggle]                = smuffConfig.wiggleRevolver;
  node[useServo]              = smuffConfig.revolverIsServo;
  node[servoOffPos]           = smuffConfig.revolverOffPos;
  node[servoOnPos]            = smuffConfig.revolverOnPos;
  node[servo1Cycles]          = smuffConfig.servoCycles1;
  node[servo2Cycles]          = smuffConfig.servoCycles2;
  node[ms3Config]             = smuffConfig.ms3config[REVOLVER];

  node = jsonObj.createNestedObject(feeder);
  node[externalControl]       = smuffConfig.extControlFeeder;
  node[stepsPerMillimeter]    = smuffConfig.stepsPerMM[FEEDER];
  node[stepDelay]             = smuffConfig.stepDelay[FEEDER];
  node[maxSpeed]              = smuffConfig.maxSpeed[FEEDER];
  node[accelSpeed]            = smuffConfig.accelSpeed[FEEDER];
  node[accelDist]             = smuffConfig.accelDist[FEEDER];
  node[insertSpeed]           = smuffConfig.insertSpeed;
  node[invertDir]             = smuffConfig.invertDir[FEEDER];
  node[endstopTrig]           = smuffConfig.endstopTrg[FEEDER];
  node[endstopTest]           = smuffConfig.endstopTrg[3];
  node[reinforceLength]       = smuffConfig.reinforceLength;
  node[unloadRetract]         = smuffConfig.unloadRetract;
  node[unloadPushback]        = smuffConfig.unloadPushback;
  node[pushbackDelay]         = smuffConfig.pushbackDelay;
  node[enableChunks]          = smuffConfig.enableChunks;
  node[feedChunks]            = smuffConfig.feedChunks;
  node[insertLength]          = smuffConfig.insertLength;
  node[duetLaser]             = smuffConfig.useDuetLaser;
  node[sharedStepper]         = smuffConfig.isSharedStepper;
  node[ms3Config]             = smuffConfig.ms3config[FEEDER];
  node[purgeSpeed]            = smuffConfig.purgeSpeed;
  node[purgeLength]           = smuffConfig.purgeLength;
  node[endstop2]              = smuffConfig.useEndstop2;
  node[autoWipe]              = smuffConfig.wipeBeforeUnload;

  if(dumpTo == nullptr) {
    dumpTo = openCfgFileWrite(CONFIG_FILE);
  }
  if(dumpTo != nullptr) {
    serializeJsonPretty(jsonDoc, *dumpTo);
    closeCfgFile();
    //__debugS(PSTR("Serializing '%s' done"), CONFIG_FILE);
    return true;
  }
  return false;
}

/*
  Writes the TMC configuration to SD-Card or Serial
*/
bool writeTmcConfig(Print* dumpTo) {
  StaticJsonDocument<scapacity> jsonDoc; // use memory on stack to serialize

  if(dumpTo == nullptr) {
    if(!initSD())
      return false;
  }

  JsonObject jsonObj = jsonDoc.to<JsonObject>();
  JsonObject node = jsonObj.createNestedObject(selector);
  node[power]                 = smuffConfig.stepperPower[SELECTOR];
  node[mode]                  = smuffConfig.stepperMode[SELECTOR];
  node[tmode]                 = smuffConfig.stepperStealth[SELECTOR];
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
  node[power]                 = smuffConfig.stepperPower[REVOLVER];
  node[mode]                  = smuffConfig.stepperMode[REVOLVER];
  node[tmode]                 = smuffConfig.stepperStealth[REVOLVER];
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
  node[power]                 = smuffConfig.stepperPower[FEEDER];
  node[mode]                  = smuffConfig.stepperMode[FEEDER];
  node[tmode]                 = smuffConfig.stepperStealth[FEEDER];
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

  node = jsonObj.createNestedObject(feeder2);
  node[power]                 = smuffConfig.stepperPower[FEEDER2];
  node[mode]                  = smuffConfig.stepperMode[FEEDER2];
  node[tmode]                 = smuffConfig.stepperStealth[FEEDER2];
  node[rsense]                = smuffConfig.stepperRSense[FEEDER2];
  node[msteps]                = smuffConfig.stepperMicrosteps[FEEDER2];
  node[stall]                 = smuffConfig.stepperStall[FEEDER2];
  node[cstepmin]              = smuffConfig.stepperCSmin[FEEDER2];
  node[cstepmax]              = smuffConfig.stepperCSmax[FEEDER2];
  node[cstepdown]             = smuffConfig.stepperCSdown[FEEDER2];
  node[drvrAdr]               = smuffConfig.stepperAddr[FEEDER2];
  node[toff]                  = smuffConfig.stepperToff[FEEDER2];
  node[stopOnStall]           = smuffConfig.stepperStopOnStall[FEEDER2];
  node[maxStallCount]         = smuffConfig.stepperMaxStallCnt[FEEDER2];

  if(dumpTo == nullptr) {
    dumpTo = openCfgFileWrite(TMC_CONFIG_FILE);
  }
  if(dumpTo != nullptr) {
    serializeJsonPretty(jsonDoc, *dumpTo);
    closeCfgFile();
    //__debugS(PSTR("Serializing '%s' done"), TMC_CONFIG_FILE);
    return true;
  }
  return false;
}

/*
  Writes the servo mapping to SD-Card or Serial
*/
bool writeServoMapping(Print* dumpTo)
{
  StaticJsonDocument<scapacity> jsonDoc; // use memory on stack to serialize

  if(dumpTo == nullptr) {
    if(!initSD())
      return false;
  }
  // create servo mappings
  JsonObject jsonObj = jsonDoc.to<JsonObject>();
  char item[15];
  for(uint8_t i=0; i < smuffConfig.toolCount; i++) {
    sprintf_P(item, P_Tool, i);
    JsonObject node = jsonObj.createNestedObject(item);
    #if defined(MULTISERVO)
    node[servoOutput] = servoMapping[i];
    #endif
    node[servoClosed] = servoPosClosed[i];
  }
  JsonObject node = jsonObj.createNestedObject(wiper);
  #if defined(MULTISERVO)
  // create the Wiper servo output pin only
  node[servoOutput] = servoMapping[16];
  #else
  node[servoOutput] = 0;
  #endif

  if(dumpTo == nullptr) {
    dumpTo = openCfgFileWrite(SERVOMAP_FILE);
  }
  if(dumpTo != nullptr) {
    serializeJsonPretty(jsonDoc, *dumpTo);
    closeCfgFile();
    //__debugS(PSTR("Serializing '%s' done"), SERVOMAP_FILE);
    return true;
  }
  return false;
}

/*
  Writes materials configuration.
*/
bool writeMaterials(Print* dumpTo) {
  StaticJsonDocument<scapacity> jsonDoc; // use memory on stack to deserialize

  if(dumpTo == nullptr) {
    if(!initSD())
      return false;
  }
  // create materials
  JsonObject jsonObj = jsonDoc.to<JsonObject>();
  char item[15];
  for(uint8_t i=0; i < smuffConfig.toolCount; i++) {
    sprintf_P(item, P_Tool, i);
    JsonObject node = jsonObj.createNestedObject(item);
    node[color] = smuffConfig.materials[i];
    node[pfactor] = smuffConfig.purges[i];
  }
  if(dumpTo == nullptr) {
    dumpTo = openCfgFileWrite(MATERIALS_FILE);
  }
  if(dumpTo != nullptr) {
    serializeJsonPretty(jsonDoc, *dumpTo);
    closeCfgFile();
    //__debugS(PSTR("Serializing '%s' done"), MATERIALS_FILE);
    return true;
  }
  return false;
}
