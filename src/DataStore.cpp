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

void saveStore() {
    StaticJsonDocument<256> jsonDoc;
    JsonObject jsonObj = jsonDoc.to<JsonObject>();

    jsonDoc["Tool"] = dataStore.tool;
    JsonObject positions = jsonObj.createNestedObject("Positions");
    positions["Selector"] = dataStore.stepperPos[SELECTOR];
    positions["Revolver"] = dataStore.stepperPos[REVOLVER];
    positions["Feeder"] = dataStore.stepperPos[FEEDER];

    FsFile cfg;
    if(cfg.open(DATASTORE_FILE, O_WRITE | O_CREAT | O_TRUNC)) {
        serializeJsonPretty(jsonDoc, cfg);
    }
    cfg.close();  
}

void recoverStore() {
    StaticJsonDocument<256> jsonDoc;
    
    FsFile cfg;
    if (!cfg.open(DATASTORE_FILE)){
      __debug(PSTR("Data store file not found!\n"));
    } 
    else {
      auto error = deserializeJson(jsonDoc, cfg);
      if (error) {
        __debug(PSTR("Data store file possibly corrupted!\n"));
      } 
      else {
        //__debug(PSTR("Data store recovered\n"));
        dataStore.stepperPos[SELECTOR]  = jsonDoc["Positions"]["Selector"];
        dataStore.stepperPos[REVOLVER]  = jsonDoc["Positions"]["Revolver"];
        dataStore.stepperPos[FEEDER]    = jsonDoc["Positions"]["Feeder"];
        dataStore.tool = jsonDoc["Tool"];
      }
      cfg.close();
    }

}