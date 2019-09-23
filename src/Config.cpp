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

#ifdef __STM32F1__
const size_t capacity = 2400;
#else
const size_t capacity = 1300;
#endif

void readConfig()
{
  
  DynamicJsonDocument jsonDoc(capacity);

  if (!SD.begin()) {
    drawSDStatus(SD_ERR_INIT);
    delay(5000);
    return;
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
      const char* selector = "Selector";
      const char* revolver = "Revolver";
      const char* feeder   = "Feeder";
      const char* maxSpeed = "MaxSpeed";
      const char* invertDir = "InvertDir";
      drawSDStatus(SD_READING_CONFIG);
      int toolCnt =                     jsonDoc[PSTR("ToolCount")];
      smuffConfig.toolCount = (toolCnt > MIN_TOOLS && toolCnt <= MAX_TOOLS) ? toolCnt : 5;
      smuffConfig.firstToolOffset =     jsonDoc[selector]["Offset"];
      smuffConfig.toolSpacing =         jsonDoc[selector]["Spacing"];
      smuffConfig.stepsPerMM_X =        jsonDoc[selector]["StepsPerMillimeter"];
      smuffConfig.maxSteps_X = ((smuffConfig.toolCount-1)*smuffConfig.toolSpacing+smuffConfig.firstToolOffset) * smuffConfig.stepsPerMM_X;
      smuffConfig.maxSpeed_X =          jsonDoc[selector][maxSpeed];
      smuffConfig.acceleration_X =      jsonDoc[selector]["Acceleration"];
      smuffConfig.invertDir_X =         jsonDoc[selector][invertDir];
      smuffConfig.endstopTrigger_X =    jsonDoc[selector]["EndstopTrigger"];
      smuffConfig.stepDelay_X =         jsonDoc[selector]["StepDelay"];
      smuffConfig.stepsPerRevolution_Y= jsonDoc[revolver]["StepsPerRevolution"];
      smuffConfig.firstRevolverOffset = jsonDoc[revolver]["Offset"];
      smuffConfig.revolverSpacing =     smuffConfig.stepsPerRevolution_Y / 10;
      smuffConfig.maxSpeed_Y =          jsonDoc[revolver][maxSpeed];
      smuffConfig.acceleration_Y =      jsonDoc[revolver]["Acceleration"];
      smuffConfig.resetBeforeFeed_Y =   jsonDoc[revolver]["ResetBeforeFeed"];
      smuffConfig.homeAfterFeed =       jsonDoc[revolver]["HomeAfterFeed"];
      smuffConfig.invertDir_Y =         jsonDoc[revolver][invertDir];
      smuffConfig.endstopTrigger_Y =    jsonDoc[revolver]["EndstopTrigger"];
      smuffConfig.stepDelay_Y =         jsonDoc[revolver]["StepDelay"];
      smuffConfig.externalControl_Z =   jsonDoc[feeder]["ExternalControl"];
      smuffConfig.stepsPerMM_Z =        jsonDoc[feeder]["StepsPerMillimeter"];
      smuffConfig.acceleration_Z =      jsonDoc[feeder]["Acceleration"];
      smuffConfig.maxSpeed_Z =          jsonDoc[feeder][maxSpeed];
      smuffConfig.insertSpeed_Z =       jsonDoc[feeder]["InsertSpeed"];
      smuffConfig.invertDir_Z =         jsonDoc[feeder][invertDir];
      smuffConfig.endstopTrigger_Z =    jsonDoc[feeder]["EndstopTrigger"];
      smuffConfig.stepDelay_Z =         jsonDoc[feeder]["StepDelay"];
      smuffConfig.reinforceLength =     jsonDoc[feeder]["ReinforceLength"];
      smuffConfig.unloadRetract =       jsonDoc[feeder]["UnloadRetract"];
      smuffConfig.unloadPushback =      jsonDoc[feeder]["UnloadPushback"];
      smuffConfig.pushbackDelay =       jsonDoc[feeder]["PushbackDelay"];
      smuffConfig.enableChunks =        jsonDoc[feeder]["EnableChunks"];
      smuffConfig.feedChunks =          jsonDoc[feeder]["FeedChunks"];
      if(smuffConfig.feedChunks == 0)
        smuffConfig.feedChunks = 20;
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
#ifdef __STM32F1__
        strncpy(smuffConfig.unloadCommand, p, sizeof(smuffConfig.unloadCommand));
#else
        strlcpy(smuffConfig.unloadCommand, p, sizeof(smuffConfig.unloadCommand));
#endif
      }
      smuffConfig.prusaMMU2 =           jsonDoc["EmulatePrusa"];

      for(int i=0; i < smuffConfig.toolCount; i++) {
        char tmp[16];
        sprintf(tmp,"T%d", i);
        if(jsonDoc["Material"][tmp] == NULL)
          sprintf(tmp,"Tool%d", i);
        memset(smuffConfig.materials[i], 0, sizeof(smuffConfig.materials[i]));
#ifdef __STM32F1__
        strncpy(smuffConfig.materials[i], jsonDoc["Materials"][tmp], sizeof(smuffConfig.materials[i])); 
#else
        strlcpy(smuffConfig.materials[i], jsonDoc["Materials"][tmp], sizeof(smuffConfig.materials[i])); 
#endif
      }
      //__debug(PSTR("DONE reading config"));
    }
    cfg.close();
  }
  else {
    __debug(PSTR("Open config file failed: handle = %s"), !cfg ? "FALSE" : "TRUE");
    drawSDStatus(SD_ERR_NOCONFIG);
    delay(5000);
  } 
}
