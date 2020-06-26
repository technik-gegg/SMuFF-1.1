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
 * Module for storing/reading data as a replacement for EEPROM
 */

#include "SMuFF.h"
#include "ArduinoJson.h"

DataStore dataStore;
extern SdFs SD;
extern int  swapTools[];

void saveStore() {
    StaticJsonDocument<512> jsonDoc;
    JsonObject jsonObj = jsonDoc.to<JsonObject>();

    jsonDoc["Tool"] = dataStore.tool;
    JsonObject positions = jsonObj.createNestedObject("Positions");
    positions["Selector"] = dataStore.stepperPos[SELECTOR];
    positions["Revolver"] = dataStore.stepperPos[REVOLVER];
    positions["Feeder"] = dataStore.stepperPos[FEEDER];
    JsonObject swaps = jsonObj.createNestedObject("SwapTools");
    char tmp[16];
    for(int i=0; i < MAX_TOOLS; i++) {
      sprintf(tmp,"T%d", i);
      swaps[tmp] = swapTools[i];
    }

    //__debug(PSTR("Updating dataStore"));
    #if defined(__ESP32__)
      if (SD.begin(SDCS_PIN, SD_SCK_MHZ(4))) {
    #else
      if (SD.begin()) {
    #endif
    FsFile cfg;
    if(cfg.open(DATASTORE_FILE, (uint8_t)(O_WRITE | O_CREAT | O_TRUNC))) {
        serializeJsonPretty(jsonDoc, cfg);
    }
    cfg.close();
    //__debug(PSTR("DataStore updated"));
  }
}

void recoverStore() {
  StaticJsonDocument<512> jsonDoc;
  #if defined(__ESP32__)
    if (SD.begin(SDCS_PIN, SD_SCK_MHZ(4))) {
  #else
    if (SD.begin()) {
  #endif
    FsFile cfg;
    if (!cfg.open(DATASTORE_FILE)){
      __debug(PSTR("Data store file '%s' not found!\n"), DATASTORE_FILE);
    }
    else {
    auto error = deserializeJson(jsonDoc, cfg);
    if (error) {
      __debug(PSTR("Data store file possibly corrupted or too large!\n"));
    }
    else {
      //__debug(PSTR("Data store recovered\n"));
      dataStore.stepperPos[SELECTOR]  = jsonDoc["Positions"]["Selector"];
      dataStore.stepperPos[REVOLVER]  = jsonDoc["Positions"]["Revolver"];
      dataStore.stepperPos[FEEDER]    = jsonDoc["Positions"]["Feeder"];
      dataStore.tool = jsonDoc["Tool"];
      char tmp[16];
      for(int i=0; i< MAX_TOOLS; i++) {
        sprintf(tmp,"T%d", i);
        if(jsonDoc["SwapTools"][tmp] != NULL) {
            swapTools[i] = jsonDoc["SwapTools"][tmp];
        }
      }
    }
    cfg.close();
    }
  }
}
