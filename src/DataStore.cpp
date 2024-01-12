/**
 * SMuFF Firmware
 * Copyright (C) 2019-2024 Technik Gegg
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
 * Module for storing/reading data as a replacement for EEPROM
 */

#include "SMuFF.h"
#include "ArduinoJson.h"
#include "ConfigNamesExt.h"

DataStore dataStore;
extern uint8_t  swapTools[];

void saveStore() {

  if(isTestrun)   // don't save if in testrun mode
    return;
  StaticJsonDocument<512> jsonDoc;
  JsonObject jsonObj = jsonDoc.to<JsonObject>();

  jsonDoc[tool] = dataStore.tool;
  JsonObject pos = jsonObj.createNestedObject(positions);
  pos[selector] = dataStore.stepperPos[SELECTOR];
  pos[revolver] = dataStore.stepperPos[REVOLVER];
  pos[feeder] = dataStore.stepperPos[FEEDER];
  JsonObject swps = jsonObj.createNestedObject(swaps);
  JsonObject splitter = jsonObj.createNestedObject(feedState);
  char tmp[16];
  for(uint8_t i=0; i < MAX_TOOLS; i++) {
    sprintf_P(tmp, P_Tool, i);
    swps[tmp] = swapTools[i];
    splitter[tmp] = smuffConfig.feedLoadState[i];
  }

  if(initSD(false)) {
    _File* cfg = openCfgFileWrite(DATASTORE_FILE);
    if(cfg != nullptr) {
      serializeJsonPretty(jsonDoc, *cfg);
      closeCfgFile();
    }
    else {
      __debugS(W, PSTR("ERROR: Failed to open/create %s"), DATASTORE_FILE);
    }
  }
  else {
    __debugS(W, PSTR("ERROR: %s not updated. Can't init SD-Card!"), DATASTORE_FILE);
  }
}

void recoverStore() {
  StaticJsonDocument<512> jsonDoc;
  if (initSD(false)) {
    _File cfg;
    if(!__fopen(cfg, DATASTORE_FILE, FILE_READ)) {
      __debugS(W, PSTR("Data store file '%s' not found!\n"), DATASTORE_FILE);
    }
    else {
      auto error = deserializeJson(jsonDoc, cfg);
      if (error) {
        __debugS(W, PSTR("Data store file possibly corrupted or too large!\n"));
      }
      else {
        //__debugS(I, PSTR("Data store recovered\n"));
        dataStore.stepperPos[SELECTOR]  = jsonDoc[positions][selector];
        dataStore.stepperPos[REVOLVER]  = jsonDoc[positions][revolver];
        dataStore.stepperPos[FEEDER]    = jsonDoc[positions][feeder];
        dataStore.tool = jsonDoc["Tool"];
        char tmp[16];
        for(uint8_t i=0; i< MAX_TOOLS; i++) {
          sprintf_P(tmp, P_Tool, i);
          if(jsonDoc[swaps][tmp] != nullptr) {
            swapTools[i] = jsonDoc[swaps][tmp];
          }
          if(jsonDoc[feedState][tmp] != nullptr) {
            smuffConfig.feedLoadState[i] = jsonDoc[feedState][tmp];
          }
          else {
            smuffConfig.feedLoadState[i] = NOT_LOADED;
          }
        }
      }
      cfg.close();
    }
  }
}

bool readTune(const char* filename, char* buffer, size_t bufLen) {
  char fname[80];

  sprintf_P(fname, PSTR("sounds/%s"), filename);
  if (initSD(false)) {
    _File tune;
    if(!__fopen(tune, fname, FILE_READ)) {
      __debugS(W, PSTR("Tune file '%s' not found!\n"), fname);
    }
    else {
      memset(buffer, 0, bufLen);
      tune.read(buffer, bufLen-1);
      tune.close();
      return true;
    }
  }
  return false;
}
