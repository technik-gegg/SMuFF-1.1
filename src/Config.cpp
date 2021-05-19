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

#if defined(__STM32F1__) || defined(__ESP32__)
const size_t capacity = 2000;
const size_t scapacity = 1300;
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

uint8_t toolsMinMax(uint8_t toolCnt) {
  return (toolCnt > MIN_TOOLS && toolCnt <= MAX_TOOLS) ? toolCnt : 5;
}

uint8_t contrastMinMax(uint8_t contrast) {
  return (contrast >= MIN_CONTRAST && contrast <= MAX_CONTRAST) ? contrast : DSP_CONTRAST;
}

uint8_t i2cAdrMinMax(uint8_t i2cAdr) {
  return (i2cAdr > 0 && i2cAdr < 128) ? i2cAdr : I2C_SLAVE_ADDRESS;
}

uint16_t speedMinMax(uint16_t speed) {
  return (speed < mmsMin || speed > mmsMax) ? mmsMin : speed;
}

/*
  Reads main config from SD-Card
*/
bool readMainConfig()
{
  if(!initSD())
    return false;
  //__debugS(PSTR("Trying to open config file '%s'"), CONFIG_FILE);

  SdFile cfg;
  if(!cfg.open(CONFIG_FILE)) {
    showOpenFailed(&cfg, CONFIG_FILE);
    return false;
  }
  else {
    if(!checkFileSize(&cfg, capacity, P_ConfigFail1)) {
      cfg.close();
      return false;
    }

    DynamicJsonDocument jsonDoc(capacity);      // use memory from heap to deserialize
    DeserializationError error = deserializeJson(jsonDoc, cfg);
    __debugS(PSTR("[readMainConfig] after deserialize... (%lu bytes)"), jsonDoc.memoryUsage());
    cfg.close();
    if (error)
      showDeserializeFailed(error, P_ConfigFail1);
    else {
      drawSDStatus(SD_READING_CONFIG);
      smuffConfig.toolCount =                   toolsMinMax(jsonDoc[toolCount]);
      smuffConfig.lcdContrast =                 contrastMinMax(jsonDoc[contrast]);
      smuffConfig.backlightColor =              jsonDoc[backlightColor] | 7;   // set backlight color to White if not set
      smuffConfig.toolColor =                   jsonDoc[toolColor] | 5;       // set tool color to Magenta if not set
      smuffConfig.encoderTickSound =            jsonDoc[encoderTicks];
      smuffConfig.bowdenLength =                jsonDoc[bowdenLength];
      smuffConfig.selectorDistance =            jsonDoc[selectorDist];
      smuffConfig.i2cAddress =                  i2cAdrMinMax(jsonDoc[i2cAdr]);
      smuffConfig.menuAutoClose =               jsonDoc[autoClose];
      smuffConfig.serialBaudrates[0] =          jsonDoc[serialBaudrate][0];
      smuffConfig.serialBaudrates[1] =          jsonDoc[serialBaudrate][1];
      smuffConfig.serialBaudrates[2] =          jsonDoc[serialBaudrate][2];
      smuffConfig.serialBaudrates[3] =          jsonDoc[serialBaudrate][3];
      smuffConfig.fanSpeed =                    jsonDoc[fanSpeed];
      smuffConfig.powerSaveTimeout =            jsonDoc[psTimeout];
      smuffConfig.sendActionCmds =              jsonDoc[sendAction];

      const char* p2 =                          jsonDoc[wipeSequence];
      const char* p3 =                          jsonDoc[lBtnDown];
      const char* p4 =                          jsonDoc[lBtnHold];
      const char* p5 =                          jsonDoc[rBtnDown];
      const char* p6 =                          jsonDoc[rBtnHold];
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
      smuffConfig.servoMinPwm =                 jsonDoc[servoMinPwm] | 800;
      smuffConfig.servoMaxPwm =                 jsonDoc[servoMaxPwm] | 2400;
      smuffConfig.sendPeriodicalStats =         jsonDoc[periodicalStats];
      smuffConfig.speedsInMMS =                 jsonDoc[speedsInMMS];
      smuffConfig.useCutter =                   jsonDoc[useCutter];
      smuffConfig.cutterOpen =                  jsonDoc[cutterOpen];
      smuffConfig.cutterClose =                 jsonDoc[cutterClose];
      smuffConfig.usePurge =                    jsonDoc[usePurge];
      smuffConfig.cutterLength =                jsonDoc[cutterLength];
      smuffConfig.useIdleAnimation =            jsonDoc[idleAnim];
      smuffConfig.animationBPM =                jsonDoc[animBpm];
      smuffConfig.statusBPM =                   jsonDoc[statusBpm];
      smuffConfig.invertRelay =                 jsonDoc[invertRelay];
      smuffConfig.menuOnTerminal =              jsonDoc[menuOnTerm];
      smuffConfig.servoCycles1 =                jsonDoc[servo1Cycles];
      smuffConfig.servoCycles2 =                jsonDoc[servo2Cycles];
      smuffConfig.revolverClose =               jsonDoc[revolverClosed];

      if(smuffConfig.speedsInMMS) {
        mmsMax = MAX_MMS;
        speedIncrement = INC_MMS;
      }
      else {
        mmsMax = MAX_TICKS;
        speedIncrement = INC_TICKS;
      }
      jsonDoc.clear();
      __debugS(PSTR("Config: DONE reading config"));
    }
  }
  return true;
}

bool readSteppersConfig()
{
  //__debugS(PSTR("Trying to open steppers config file '%s'"), STEPPERS_FILE);

  SdFile cfg;
  if(!cfg.open(STEPPERS_FILE)) {
    showOpenFailed(&cfg, STEPPERS_FILE);
    return false;
  }
  else {
    if(!checkFileSize(&cfg, capacity, P_ConfigFail8)) {
      cfg.close();
      return false;
    }

    DynamicJsonDocument jsonDoc(capacity);      // use memory from heap to deserialize
    DeserializationError error = deserializeJson(jsonDoc, cfg);
    __debugS(PSTR("[readSteppersConfig] after deserialize... (%lu bytes)"), jsonDoc.memoryUsage());
    cfg.close();
    if (error)
      showDeserializeFailed(error, P_ConfigFail8);
    else {
      drawSDStatus(SD_READING_CONFIG);
      /*
      SELECTOR
      */
      smuffConfig.firstToolOffset =             jsonDoc[selector][offset];
      smuffConfig.toolSpacing =                 jsonDoc[selector][spacing];
      smuffConfig.stepsPerMM[SELECTOR] =        jsonDoc[selector][stepsPerMillimeter];
      smuffConfig.maxSteps[SELECTOR] =          ((smuffConfig.toolCount-1)*smuffConfig.toolSpacing+smuffConfig.firstToolOffset) * smuffConfig.stepsPerMM[SELECTOR];
      smuffConfig.maxSpeed[SELECTOR] =          speedMinMax(jsonDoc[selector][maxSpeed]);
      smuffConfig.accelSpeed[SELECTOR] =        speedMinMax(jsonDoc[selector][accelSpeed]);
      smuffConfig.accelDist[SELECTOR] =         jsonDoc[selector][accelDist];
      smuffConfig.invertDir[SELECTOR] =         jsonDoc[selector][invertDir];
      smuffConfig.endstopTrg[SELECTOR] =        jsonDoc[selector][endstopTrig];
      smuffConfig.stepDelay[SELECTOR] =         jsonDoc[selector][stepDelay];
      smuffConfig.ms3config[SELECTOR] =         jsonDoc[selector][ms3Config];
      /*
      REVOLVER
      */
      smuffConfig.stepsPerRevolution =          jsonDoc[revolver][stepsPerRevolution];
      smuffConfig.stepsPerMM[REVOLVER] =        jsonDoc[revolver][stepsPerMillimeter];
      smuffConfig.firstRevolverOffset =         jsonDoc[revolver][offset];
      smuffConfig.revolverSpacing =             smuffConfig.stepsPerRevolution / 10;
      smuffConfig.maxSpeed[REVOLVER] =          speedMinMax(jsonDoc[revolver][maxSpeed]);
      smuffConfig.accelSpeed[REVOLVER] =        speedMinMax(jsonDoc[revolver][accelSpeed]);
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
      smuffConfig.ms3config[REVOLVER] =         jsonDoc[revolver][ms3Config];
      #if defined(SMUFF_V6S)
      smuffConfig.revolverIsServo = false;
      #elif defined(SMUFF_V5)
      smuffConfig.revolverIsServo = true;
      #endif
      /*
      FEEDER
      */
      smuffConfig.extControlFeeder =            jsonDoc[feeder][externalControl];
      smuffConfig.stepsPerMM[FEEDER] =          jsonDoc[feeder][stepsPerMillimeter];
      smuffConfig.maxSpeed[FEEDER] =            speedMinMax(jsonDoc[feeder][maxSpeed]);
      smuffConfig.accelSpeed[FEEDER] =          speedMinMax(jsonDoc[feeder][accelSpeed]);
      smuffConfig.insertSpeed =                 speedMinMax(jsonDoc[feeder][insertSpeed]);
      smuffConfig.purgeSpeed =                  speedMinMax(jsonDoc[feeder][purgeSpeed]);
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
      smuffConfig.feedChunks =                  jsonDoc[feeder][feedChunks] | 20;
      smuffConfig.insertLength =                jsonDoc[feeder][insertLength] | 5;
      smuffConfig.useDuetLaser =                jsonDoc[feeder][duetLaser];
      smuffConfig.isSharedStepper =             jsonDoc[feeder][sharedStepper];
      smuffConfig.ms3config[FEEDER] =           jsonDoc[feeder][ms3Config];
      smuffConfig.purgeLength =                 jsonDoc[feeder][purgeLength];
      smuffConfig.wipeBeforeUnload =            jsonDoc[feeder][autoWipe];

      jsonDoc.clear();
      __debugS(PSTR("Config: DONE reading steppers config"));
    }
  }
  return true;
}

bool readConfig() {
  if(!initSD())
    return false;

  if(readMainConfig()) {
    if(readSteppersConfig())
      return true;
  }
  return false;
}

/*
  Reads TMC driver config from SD-Card
*/
bool readTmcConfig()
{
  if(!initSD())
    return false;
  //__debugS(PSTR("Trying to open TMC config file '%s'"), TMC_CONFIG_FILE);
  SdFile cfg;
  if(!cfg.open(TMC_CONFIG_FILE)) {
    showOpenFailed(&cfg, TMC_CONFIG_FILE);
    return false;
  }
  else {
    if(!checkFileSize(&cfg, scapacity, P_ConfigFail6)) {
      cfg.close();
      return false;
    }

    DynamicJsonDocument jsonDoc(scapacity);       // use memory from heap to deserialize
    DeserializationError error = deserializeJson(jsonDoc, cfg);
    __debugS(PSTR("[readTmcConfig] after deserialize... (%lu bytes)"), jsonDoc.memoryUsage());
    cfg.close();
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

      jsonDoc.clear();
      __debugS(PSTR("Config: DONE reading TMC config"));
    }
  }
  return true;
}

/*
  Reads mapping of the servos from SD-Card.
*/
bool readServoMapping() {

  if(!initSD())
    return false;
  //__debugS(PSTR("Trying to open Servo Mapping file '%s'"), SERVOMAP_FILE);
  SdFile cfg;
  if(!cfg.open(SERVOMAP_FILE)) {
    showOpenFailed(&cfg, SERVOMAP_FILE);
    return false;
  }
  else {
    if(!checkFileSize(&cfg, scapacity, P_ConfigFail7)) {
      cfg.close();
      return false;
    }

    DynamicJsonDocument jsonDoc(scapacity);       // use memory from heap to deserialize
    DeserializationError error = deserializeJson(jsonDoc, cfg);
    cfg.close();
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

      jsonDoc.clear();
      __debugS(PSTR("Config: DONE reading servo mappings"));
    }
  }
  return true;
}

/*
  Reads mapping for the Revolver stepper from SD-Card.
*/
bool readRevolverMapping() {

  if(!initSD())
    return false;
  //__debugS(PSTR("Trying to open Servo Mapping file '%s'"), SERVOMAP_FILE);
  SdFile cfg;
  if(!cfg.open(STEPPERMAP_FILE)) {
    showOpenFailed(&cfg, STEPPERMAP_FILE);
    return false;
  }
  else {
    if(!checkFileSize(&cfg, scapacity, P_ConfigFail7)) {
      cfg.close();
      return false;
    }

    DynamicJsonDocument jsonDoc(scapacity);       // use memory from heap to deserialize
    DeserializationError error = deserializeJson(jsonDoc, cfg);
    cfg.close();
    if (error)
      showDeserializeFailed(error, P_ConfigFail8);
    else {
      drawSDStatus(SD_READING_STEPPERS);
      // read servo mappings
      char item[15];
      for(uint8_t i=0; i < smuffConfig.toolCount; i++) {
        sprintf_P(item, P_Tool, i);
        if(jsonDoc[item] == nullptr)
          stepperPosClosed[i] = smuffConfig.revolverClose;
        else
          stepperPosClosed[i] = (float)jsonDoc[item];
      }
      jsonDoc.clear();
      __debugS(PSTR("Config: DONE reading Revolver stepper mappings"));
    }
  }
  return true;
}

/*
  Reads materials assignment.
*/
bool readMaterials() {

  if(!initSD())
    return false;
  //__debugS(PSTR("Trying to open TMC config file '%s'"), MATERIALS_FILE);
  SdFile cfg;
  if(!cfg.open(MATERIALS_FILE)) {
    showOpenFailed(&cfg, MATERIALS_FILE);
    return false;
  }
  else {
    if(!checkFileSize(&cfg, scapacity, P_ConfigFail5)) {
      cfg.close();
      return false;
    }
    DynamicJsonDocument jsonDoc(scapacity);       // use memory from heap to deserialize
    DeserializationError error = deserializeJson(jsonDoc, cfg);
    cfg.close();
    if (error)
      showDeserializeFailed(error, P_ConfigFail5);
    else {
      drawSDStatus(SD_READING_MATERIALS);
      // read materials
      char item[15];
      for(uint8_t i=0; i < smuffConfig.toolCount; i++) {
        memset(smuffConfig.materialNames[i], 0, MAX_MATERIAL_NAME_LEN);
        memset(smuffConfig.materials[i], 0, MAX_MATERIAL_LEN);
        sprintf_P(item, P_Tool, i);
        const char* pItem = jsonDoc[item][color];
        if(pItem == nullptr) {
          sprintf_P(smuffConfig.materialNames[i], PSTR("Tool%d"), i);
        }
        else {
          strncpy(smuffConfig.materialNames[i], pItem, MAX_MATERIAL_NAME_LEN);
        }
        //__debugS(PSTR("%s: %s"), item, smuffConfig.materialNames[i]);
        uint16_t len = jsonDoc[item][pfactor];
        if(len > 0) {
          smuffConfig.purges[i] = len;
        }
        else {
          smuffConfig.purges[i] = 100;
        }
        const char* cval = jsonDoc[item][colorVal];
        if(cval != nullptr) {
          long color;
          if(sscanf(cval,"%lx", &color) > 0) {
            smuffConfig.materialColors[i] = (uint32_t)color;
            //__debugS(PSTR("Material color: #%lX"), color);
          }
        }
        const char* mat = jsonDoc[item][material];
        if(mat != nullptr) {
          //__debugS(PSTR("%s: %s"), item, mat);
          strncpy(smuffConfig.materials[i], mat, MAX_MATERIAL_LEN);
        }
      }

      jsonDoc.clear();
      __debugS(PSTR("Config: DONE reading materials"));
    }
  }
  return true;
}

bool dumpConfig(Print* dumpTo, bool useWebInterface, const char* filename, DynamicJsonDocument& jsonDoc) {
  if(dumpTo == nullptr && filename == nullptr)
    return false;
  bool hasFile = false;
  if(dumpTo == nullptr && filename != nullptr) {
    dumpTo = openCfgFileWrite(filename);
    hasFile = true;
  }
  if(dumpTo != nullptr) {
    if(useWebInterface)
      serializeJson(jsonDoc, *dumpTo);
    else
      serializeJsonPretty(jsonDoc, *dumpTo);
    if(hasFile)
      closeCfgFile();
    //__debugS(PSTR("Serializing '%s' done"), filename);
    return true;
  }
  return false;
}

/*
  Writes the basic configuration to SD-Card or Serial
*/
bool writeMainConfig(Print* dumpTo, bool useWebInterface) {

  if(dumpTo == nullptr) {
    if(!initSD())
      return false;
  }

  DynamicJsonDocument jsonDoc(capacity);
  jsonDoc[serialBaudrate][0]    = smuffConfig.serialBaudrates[0];
  jsonDoc[serialBaudrate][1]    = smuffConfig.serialBaudrates[1];
  jsonDoc[serialBaudrate][2]    = smuffConfig.serialBaudrates[2];
  jsonDoc[serialBaudrate][3]    = smuffConfig.serialBaudrates[3];
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
  jsonDoc[hasPanelDue]          = smuffConfig.hasPanelDue;
  jsonDoc[servoMinPwm]          = smuffConfig.servoMinPwm;
  jsonDoc[servoMaxPwm]          = smuffConfig.servoMaxPwm;
  jsonDoc[periodicalStats]      = smuffConfig.sendPeriodicalStats;
  jsonDoc[wipeSequence]         = smuffConfig.wipeSequence;
  jsonDoc[backlightColor]       = smuffConfig.backlightColor;
  jsonDoc[toolColor]            = smuffConfig.toolColor;
  jsonDoc[encoderTicks]         = smuffConfig.encoderTickSound;
  jsonDoc[lBtnDown]             = smuffConfig.lButtonDown;
  jsonDoc[lBtnHold]             = smuffConfig.lButtonHold;
  jsonDoc[rBtnDown]             = smuffConfig.rButtonDown;
  jsonDoc[rBtnHold]             = smuffConfig.rButtonHold;
  jsonDoc[speedsInMMS]          = smuffConfig.speedsInMMS;
  jsonDoc[useCutter]            = smuffConfig.useCutter;
  jsonDoc[cutterOpen]           = smuffConfig.cutterOpen;
  jsonDoc[cutterClose]          = smuffConfig.cutterClose;
  jsonDoc[usePurge]             = smuffConfig.usePurge;
  jsonDoc[cutterLength]         = smuffConfig.cutterLength;
  jsonDoc[idleAnim]             = smuffConfig.useIdleAnimation;
  jsonDoc[animBpm]              = smuffConfig.animationBPM;
  jsonDoc[statusBpm]            = smuffConfig.statusBPM;
  jsonDoc[invertRelay]          = smuffConfig.invertRelay;
  jsonDoc[menuOnTerm]           = smuffConfig.menuOnTerminal;
  jsonDoc[servo1Cycles]         = smuffConfig.servoCycles1;
  jsonDoc[servo2Cycles]         = smuffConfig.servoCycles2;
  jsonDoc[revolverClosed]       = smuffConfig.revolverClose;

  return dumpConfig(dumpTo, useWebInterface, CONFIG_FILE, jsonDoc);
}

/*
  Writes the steppers configuration to SD-Card or Serial
*/
bool writeSteppersConfig(Print* dumpTo, bool useWebInterface) {

  if(dumpTo == nullptr) {
    if(!initSD())
      return false;
  }

  DynamicJsonDocument jsonDoc(capacity);
  JsonObject node = jsonDoc.createNestedObject(selector);
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

  node = jsonDoc.createNestedObject(revolver);
  node[offset]                = smuffConfig.firstRevolverOffset;
  node[stepsPerRevolution]    = smuffConfig.stepsPerRevolution;
  node[stepsPerMillimeter]    = smuffConfig.stepsPerMM[REVOLVER];
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
  node[ms3Config]             = smuffConfig.ms3config[REVOLVER];

  node = jsonDoc.createNestedObject(feeder);
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

  return dumpConfig(dumpTo, useWebInterface, STEPPERS_FILE, jsonDoc);
}

/*
  Write main a& steppers config combined
*/
bool writeConfig(Print* dumpTo, bool useWebInterface) {

  if(writeMainConfig(dumpTo, useWebInterface)) {
    if(writeSteppersConfig(dumpTo, useWebInterface))
      return true;
  }
  return false;
}

/*
  Writes the TMC configuration to SD-Card or Serial
*/
bool writeTmcConfig(Print* dumpTo, bool useWebInterface) {

  if(dumpTo == nullptr) {
    if(!initSD())
      return false;
  }

  DynamicJsonDocument jsonDoc(capacity); // use memory on heap to serialize
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

  return dumpConfig(dumpTo, useWebInterface, TMC_CONFIG_FILE, jsonDoc);
}

/*
  Writes the servo mapping to SD-Card or Serial
*/
bool writeServoMapping(Print* dumpTo, bool useWebInterface)
{
  if(dumpTo == nullptr) {
    if(!initSD())
      return false;
  }
  DynamicJsonDocument jsonDoc(scapacity); // use memory on heap to serialize
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

  return dumpConfig(dumpTo, useWebInterface, SERVOMAP_FILE, jsonDoc);
}

/*
  Writes the Revolver stepper mapping to SD-Card or Serial
*/
bool writeRevolverMapping(Print* dumpTo, bool useWebInterface) {
  if(dumpTo == nullptr) {
    if(!initSD())
      return false;
  }
  DynamicJsonDocument jsonDoc(scapacity); // use memory on heap to serialize
  // create revolver mappings
  char item[15];
  for(uint8_t i=0; i < smuffConfig.toolCount; i++) {
    sprintf_P(item, P_Tool, i);
    jsonDoc[item] = stepperPosClosed[i];
  }
  return dumpConfig(dumpTo, useWebInterface, STEPPERMAP_FILE, jsonDoc);
}

/*
  Writes materials configuration.
*/
bool writeMaterials(Print* dumpTo, bool useWebInterface) {
  if(dumpTo == nullptr) {
    if(!initSD())
      return false;
  }
  DynamicJsonDocument jsonDoc(scapacity); // use memory on heap to deserialize
  // create materials
  JsonObject jsonObj = jsonDoc.to<JsonObject>();
  char item[15];
  char tmp[10];
  for(uint8_t i=0; i < smuffConfig.toolCount; i++) {
    sprintf_P(item, P_Tool, i);
    JsonObject node = jsonObj.createNestedObject(item);
    node[material] = smuffConfig.materials[i];
    node[color] = smuffConfig.materialNames[i];
    node[pfactor] = smuffConfig.purges[i];
    sprintf(tmp,"%06lX", smuffConfig.materialColors[i]);
    node[colorVal] = tmp;
  }
  return dumpConfig(dumpTo, useWebInterface, MATERIALS_FILE, jsonDoc);
}

/*
  Writes tools swapping configuration.
*/
bool writeSwapTools(Print* dumpTo, bool useWebInterface) {
  if(dumpTo == nullptr) {
    return false;
  }
  DynamicJsonDocument jsonDoc(scapacity); // use memory on heap to deserialize
  char tmp[16];
  for(uint8_t i=0; i < MAX_TOOLS; i++) {
    sprintf_P(tmp, P_Tool, i);
    jsonDoc[tmp] = swapTools[i];
  }
  return dumpConfig(dumpTo, useWebInterface, nullptr, jsonDoc);
}

bool deserializeSwapTools(const char* cfg) {
    DynamicJsonDocument jsonDoc(scapacity);       // use memory from heap to deserialize
    DeserializationError error = deserializeJson(jsonDoc, cfg);
    if (error) {
      __debugS(PSTR("deserializeJson() failed with code %s (Input: >%s<)"), error.c_str(), cfg);
      return false;
    }
    else {
      char tmp[10];
      for(uint8_t i=0; i < MAX_TOOLS; i++) {
        sprintf_P(tmp, P_Tool, i);
        swapTools[i] = jsonDoc[tmp];
        //__debugS(PSTR("T%d = %d"), i, swapTools[i]);
      }
    }
    return true;
}

bool serializeTMCStats(Print* dumpTo, uint8_t axis, int8_t version, bool isStealth, uint16_t powerCfg, uint16_t powerRms, uint16_t microsteps, bool ms1, bool ms2, const char* uart,
      const char* diag, const char* ola, const char* olb, const char* s2ga, const char* s2gb, const char* ot_stat) {
  DynamicJsonDocument jsonDoc(scapacity); // use memory on stack to deserialize
  // create status
  JsonObject jsonObj = jsonDoc.to<JsonObject>();
  JsonObject node = jsonObj.createNestedObject(P_TMCStatus);
  char tmp[4];
  sprintf_P(tmp,"%d%d", ms2, ms1);
  node[P_TMCKeyAxis]  = axis;
  node[P_TMCKeyVersion]  = version;
  node[P_TMCKeyInUse]  = true;
  node[P_TMCKeyMode]  = isStealth ? "StealthChop" : "SpreadCycle";
  node[P_TMCKeyPwrCfg]  = powerCfg;
  node[P_TMCKeyPwrRms]  = powerRms;
  node[P_TMCKeyMS]  = microsteps;
  node[P_TMCKeyAddr]  = tmp;
  node[P_TMCKeyUart]  = uart;
  node[P_TMCKeyDiag]  = diag;
  node[P_TMCKeyOLA]  = ola;
  node[P_TMCKeyOLB]  = olb;
  node[P_TMCKeyS2GA]  = s2ga;
  node[P_TMCKeyS2GB]  = s2gb;
  node[P_TMCKeyOT] = ot_stat;
  serializeJson(jsonDoc, *dumpTo);
  return true;
}

bool saveConfig(String& buffer) {
  if(!initSD())
    return false;
  String cfgType;

  int16_t epos = buffer.indexOf("*/");
  if(epos != -1) {
    cfgType = buffer.substring(2, epos);
  }
  else
    return false;
  const char* filename;
  if(cfgType.equals("S1")) {
    filename = CONFIG_FILE;
  }
  if(cfgType.equals("S2")) {
    filename = STEPPERS_FILE;
  }
  if(cfgType.equals("S3")) {
    filename = TMC_CONFIG_FILE;
  }
  if(cfgType.equals("S4")) {
    filename = SERVOMAP_FILE;
  }
  if(cfgType.equals("S5")) {
    filename = MATERIALS_FILE;
  }
  //__debugS(PSTR("Processing config '%s' Data: \n%s"), filename, buffer.substring(epos+2).c_str());
  DynamicJsonDocument jsonDoc(capacity);       // use memory from heap to deserialize
  DeserializationError error = deserializeJson(jsonDoc, buffer.substring(epos+2));
  if (error) {
    __debugS(PSTR("deserializeJson() failed with code %s"), error.c_str());
    return false;
  }
  else {
    Print* dumpTo = openCfgFileWrite(filename);
    serializeJsonPretty(jsonDoc, *dumpTo);
    closeCfgFile();
  }

  return true;
}