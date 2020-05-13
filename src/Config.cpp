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
#include <ArduinoJson.h>

SdFs SD;

#if defined(__STM32F1__)
const size_t capacity = 2400;
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

  __debug(PSTR("Trying to open config file '%s'"), CONFIG_FILE);
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
      const char* selector    = "Selector";
      const char* revolver    = "Revolver";
      const char* feeder      = "Feeder";
      const char* maxSpeed    = "MaxSpeed";
      const char* maxSpeedHS  = "MaxSpeedHS";
      const char* acceleration= "Acceleration";
      const char* invertDir   = "InvertDir";
      const char* endstopTrig = "EndstopTrigger";
      const char* stepDelay   = "StepDelay";
      drawSDStatus(SD_READING_CONFIG);
      int toolCnt =                     jsonDoc["ToolCount"];
      smuffConfig.toolCount = (toolCnt > MIN_TOOLS && toolCnt <= MAX_TOOLS) ? toolCnt : 5;
      smuffConfig.firstToolOffset =     jsonDoc[selector]["Offset"];
      smuffConfig.toolSpacing =         jsonDoc[selector]["Spacing"];
      smuffConfig.stepsPerMM_X =        jsonDoc[selector]["StepsPerMillimeter"];
      smuffConfig.maxSteps_X = ((smuffConfig.toolCount-1)*smuffConfig.toolSpacing+smuffConfig.firstToolOffset) * smuffConfig.stepsPerMM_X;
      smuffConfig.maxSpeed_X =          jsonDoc[selector][maxSpeed];
      smuffConfig.acceleration_X =      jsonDoc[selector][acceleration];
      smuffConfig.invertDir_X =         jsonDoc[selector][invertDir];
      smuffConfig.endstopTrigger_X =    jsonDoc[selector][endstopTrig];
      smuffConfig.stepDelay_X =         jsonDoc[selector][stepDelay];
      smuffConfig.maxSpeedHS_X =        jsonDoc[selector][maxSpeedHS];
      smuffConfig.stepsPerRevolution_Y= jsonDoc[revolver]["StepsPerRevolution"];
      smuffConfig.firstRevolverOffset = jsonDoc[revolver]["Offset"];
      smuffConfig.revolverSpacing =     smuffConfig.stepsPerRevolution_Y / 10;
      smuffConfig.maxSpeed_Y =          jsonDoc[revolver][maxSpeed];
      smuffConfig.acceleration_Y =      jsonDoc[revolver][acceleration];
      smuffConfig.resetBeforeFeed_Y =   jsonDoc[revolver]["ResetBeforeFeed"];
      smuffConfig.homeAfterFeed =       jsonDoc[revolver]["HomeAfterFeed"];
      smuffConfig.invertDir_Y =         jsonDoc[revolver][invertDir];
      smuffConfig.endstopTrigger_Y =    jsonDoc[revolver][endstopTrig];
      smuffConfig.stepDelay_Y =         jsonDoc[revolver][stepDelay];
      smuffConfig.maxSpeedHS_Y =        jsonDoc[revolver][maxSpeedHS];
      smuffConfig.wiggleRevolver =      jsonDoc[revolver]["Wiggle"];
      smuffConfig.revolverIsServo =     jsonDoc[revolver]["UseServo"];
      smuffConfig.revolverOffPos =      jsonDoc[revolver]["ServoOffPos"];
      smuffConfig.revolverOnPos =       jsonDoc[revolver]["ServoOnPos"];
      smuffConfig.servoCycles =         jsonDoc[revolver]["ServoCycles"];

      smuffConfig.externalControl_Z =   jsonDoc[feeder]["ExternalControl"];
      smuffConfig.stepsPerMM_Z =        jsonDoc[feeder]["StepsPerMillimeter"];
      smuffConfig.acceleration_Z =      jsonDoc[feeder][acceleration];
      smuffConfig.maxSpeed_Z =          jsonDoc[feeder][maxSpeed];
      smuffConfig.insertSpeed_Z =       jsonDoc[feeder]["InsertSpeed"];
      if(smuffConfig.insertSpeed_Z > smuffConfig.acceleration_Z)
        smuffConfig.acceleration_Z = smuffConfig.insertSpeed_Z;
      smuffConfig.invertDir_Z =         jsonDoc[feeder][invertDir];
      smuffConfig.endstopTrigger_Z =    jsonDoc[feeder][endstopTrig];
      smuffConfig.stepDelay_Z =         jsonDoc[feeder][stepDelay];
      smuffConfig.reinforceLength =     jsonDoc[feeder]["ReinforceLength"];
      smuffConfig.unloadRetract =       jsonDoc[feeder]["UnloadRetract"];
      smuffConfig.unloadPushback =      jsonDoc[feeder]["UnloadPushback"];
      smuffConfig.pushbackDelay =       jsonDoc[feeder]["PushbackDelay"];
      smuffConfig.enableChunks =        jsonDoc[feeder]["EnableChunks"];
      smuffConfig.feedChunks =          jsonDoc[feeder]["FeedChunks"];
      if(smuffConfig.feedChunks == 0)
        smuffConfig.feedChunks = 20;
      smuffConfig.insertLength =        jsonDoc[feeder]["InsertLength"];
      if(smuffConfig.insertLength == 0)
        smuffConfig.insertLength = 5;
      smuffConfig.maxSpeedHS_Z =        jsonDoc[feeder][maxSpeedHS];
      smuffConfig.useDuetLaser =        jsonDoc[feeder]["DuetLaser"];

      int contrast =                    jsonDoc["LCDContrast"];
      smuffConfig.lcdContrast = (contrast > MIN_CONTRAST && contrast < MAX_CONTRAST) ? contrast : DSP_CONTRAST;
      smuffConfig.bowdenLength =        jsonDoc["BowdenLength"];
      smuffConfig.selectorDistance =    jsonDoc["SelectorDist"];
      int i2cAdr =                      jsonDoc["I2CAddress"];
      smuffConfig.i2cAddress = (i2cAdr > 0 && i2cAdr < 255) ? i2cAdr : I2C_SLAVE_ADDRESS;
      smuffConfig.menuAutoClose =       jsonDoc["MenuAutoClose"];
      smuffConfig.delayBetweenPulses =  jsonDoc["DelayBetweenPulses"];
      smuffConfig.serial1Baudrate =     jsonDoc["Serial1Baudrate"];
      smuffConfig.serial2Baudrate =     jsonDoc["Serial2Baudrate"];
      smuffConfig.serialDueBaudrate =   jsonDoc["SerialDueBaudrate"];
      smuffConfig.fanSpeed =            jsonDoc["FanSpeed"];
      smuffConfig.powerSaveTimeout =    jsonDoc["PowerSaveTimeout"];
      smuffConfig.duetDirect =          jsonDoc["Duet3DDirect"];
      const char* p =                   jsonDoc["UnloadCommand"];
      if(p != NULL && strlen(p) > 0) {
#if defined(__STM32F1__) || defined(__ESP32__)
        strncpy(smuffConfig.unloadCommand, p, sizeof(smuffConfig.unloadCommand));
#else
        strlcpy(smuffConfig.unloadCommand, p, sizeof(smuffConfig.unloadCommand));
#endif
      }
      smuffConfig.prusaMMU2 =           jsonDoc["EmulatePrusa"];
      smuffConfig.hasPanelDue =         jsonDoc["HasPanelDue"];
      smuffConfig.servoMinPwm =         jsonDoc["ServoMinPwm"];
      smuffConfig.servoMaxPwm =         jsonDoc["ServoMaxPwm"];
      if(smuffConfig.servoMinPwm == 0)
        smuffConfig.servoMinPwm = 550;
      if(smuffConfig.servoMaxPwm == 0)
        smuffConfig.servoMaxPwm = 2400;

      // read materials if running on 32-Bit MCU
#if defined(__STM32F1__) || defined(__ESP32__)
      for(int i=0; i < smuffConfig.toolCount; i++) {
        char tmp[16];
        sprintf(tmp,"T%d", i);
        if(jsonDoc["Material"][tmp] == NULL)
          sprintf(tmp,"Tool%d", i);
        memset(smuffConfig.materials[i], 0, sizeof(smuffConfig.materials[i]));
        strncpy(smuffConfig.materials[i], jsonDoc["Materials"][tmp], sizeof(smuffConfig.materials[i])); 
      }
#else
      for(int i=0; i < smuffConfig.toolCount; i++) {
        memset(smuffConfig.materials[i], 0, sizeof(smuffConfig.materials[i]));
      }
#endif
      
      __debug(PSTR("DONE reading config"));
    }
    if(smuffConfig.maxSpeedHS_X == 0)
      smuffConfig.maxSpeedHS_X = smuffConfig.maxSpeed_X;
    if(smuffConfig.maxSpeedHS_Y == 0)
      smuffConfig.maxSpeedHS_Y = smuffConfig.maxSpeed_Y;
    if(smuffConfig.maxSpeedHS_Z == 0)
      smuffConfig.maxSpeedHS_Z = smuffConfig.maxSpeed_Z;
    cfg.close();
  }
  else {
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
  jsonDoc["Serial1Baudrate"]      = smuffConfig.serial1Baudrate;
  jsonDoc["Serial2Baudrate"]      = smuffConfig.serial2Baudrate;
  jsonDoc["SerialDueBaudrate"]    = smuffConfig.serialDueBaudrate;
  jsonDoc["ToolCount"]            = smuffConfig.toolCount;
  jsonDoc["BowdenLength"]         = smuffConfig.bowdenLength;
  jsonDoc["SelectorDist"]         = smuffConfig.selectorDistance;
  jsonDoc["LCDContrast"]          = smuffConfig.lcdContrast;
  jsonDoc["I2CAddress"]           = smuffConfig.i2cAddress;
  jsonDoc["MenuAutoClose"]        = smuffConfig.menuAutoClose;
  jsonDoc["FanSpeed"]             = smuffConfig.fanSpeed;
  jsonDoc["DelayBetweenPulses"]   = smuffConfig.delayBetweenPulses;
  jsonDoc["PowerSaveTimeout"]     = smuffConfig.powerSaveTimeout;
  jsonDoc["Duet3DDirect"]         = smuffConfig.duetDirect;
  jsonDoc["EmulatePrusa"]         = smuffConfig.prusaMMU2;
  jsonDoc["UnloadCommand"]        = smuffConfig.unloadCommand;
  jsonDoc["HasPanelDue"]          = smuffConfig.hasPanelDue;
  jsonDoc["ServoMinPwm"]          = smuffConfig.servoMinPwm;
  jsonDoc["ServoMaxPwm"]          = smuffConfig.servoMaxPwm;

  JsonObject node = jsonObj.createNestedObject("Selector");
  node["Offset"]              = smuffConfig.firstToolOffset;
  node["Spacing"]             = smuffConfig.toolSpacing;
  node["StepsPerMillimeter"]  = smuffConfig.stepsPerMM_X;
  node["StepDelay"]           = smuffConfig.stepDelay_X;
  node["MaxSpeed"]            = smuffConfig.maxSpeed_X;
  node["MaxSpeedHS"]          = smuffConfig.maxSpeedHS_X;
  node["Acceleration"]        = smuffConfig.acceleration_X;
  node["InvertDir"]           = smuffConfig.invertDir_X;
  node["EndstopTrigger"]      = smuffConfig.endstopTrigger_X;

  node = jsonObj.createNestedObject("Revolver");
  node["Offset"]              = smuffConfig.firstRevolverOffset;
  node["StepsPerRevolution"]  = smuffConfig.stepsPerRevolution_Y;
  node["StepDelay"]           = smuffConfig.stepDelay_Y;
  node["MaxSpeed"]            = smuffConfig.maxSpeed_Y;
  node["MaxSpeedHS"]          = smuffConfig.maxSpeedHS_Y;
  node["Acceleration"]        = smuffConfig.acceleration_Y;
  node["ResetBeforeFeed"]     = smuffConfig.resetBeforeFeed_Y;
  node["HomeAfterFeed"]       = smuffConfig.homeAfterFeed;
  node["InvertDir"]           = smuffConfig.invertDir_Y;
  node["EndstopTrigger"]      = smuffConfig.endstopTrigger_Y;
  node["Wiggle"]              = smuffConfig.wiggleRevolver;
  node["UseServo"]            = smuffConfig.revolverIsServo;
  node["ServoOffPos"]         = smuffConfig.revolverOffPos;
  node["ServoOnPos"]          = smuffConfig.revolverOnPos;
  node["ServoCycles"]         = smuffConfig.servoCycles;

  node = jsonObj.createNestedObject("Feeder");
  node["ExternalControl"]     = smuffConfig.externalControl_Z;
  node["StepsPerMillimeter"]  = smuffConfig.stepsPerMM_Z;
  node["StepDelay"]           = smuffConfig.stepDelay_Z;
  node["MaxSpeed"]            = smuffConfig.maxSpeed_Z;
  node["MaxSpeedHS"]          = smuffConfig.maxSpeedHS_Z;
  node["Acceleration"]        = smuffConfig.acceleration_Z;
  node["InsertSpeed"]         = smuffConfig.insertSpeed_Z;
  node["InvertDir"]           = smuffConfig.invertDir_Z;
  node["EndstopTrigger"]      = smuffConfig.endstopTrigger_Z;
  node["ReinforceLength"]     = smuffConfig.reinforceLength;
#if defined(__STM32F1__) || defined(__ESP32__)
  node["UnloadRetract"]       = smuffConfig.unloadRetract;
  node["UnloadPushback"]      = smuffConfig.unloadPushback;
  node["PushbackDelay"]       = smuffConfig.pushbackDelay;
#endif
  node["EnableChunks"]        = smuffConfig.enableChunks;
  node["FeedChunks"]          = smuffConfig.feedChunks;
  node["InsertLength"]        = smuffConfig.insertLength;

#if defined(__STM32F1__) || defined(__ESP32__)
  node = jsonObj.createNestedObject("Materials");
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